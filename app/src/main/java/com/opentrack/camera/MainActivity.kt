package com.opentrack.camera

import android.Manifest
import android.content.pm.PackageManager
import android.hardware.SensorManager
import android.hardware.camera2.CameraManager
import android.os.Bundle
import android.os.SystemClock
import android.view.Surface
import android.view.SurfaceHolder
import android.view.WindowManager
import android.widget.Button
import android.widget.SeekBar
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import androidx.core.view.WindowCompat
import androidx.core.view.WindowInsetsCompat
import androidx.core.view.WindowInsetsControllerCompat
import com.opentrack.camera.camera.CameraController
import com.opentrack.camera.camera.CameraProfile
import com.opentrack.camera.camera.CameraSettings
import com.opentrack.camera.detect.AircraftDetector
import com.opentrack.camera.pipeline.AnalysisEngine
import com.opentrack.camera.pipeline.OverlayState
import com.opentrack.camera.pipeline.PreviewMode
import com.opentrack.camera.pipeline.RenderPipeline
import com.opentrack.camera.record.Recorder
import com.opentrack.camera.record.RecordingConfig
import com.opentrack.camera.sensor.GyroSource
import com.opentrack.camera.ui.OverlayView
import com.opentrack.tracking.CameraMotion
import com.opentrack.tracking.FramingController
import com.opentrack.tracking.GyroMotionModel
import com.opentrack.tracking.TargetTracker
import com.opentrack.tracking.TrackState
import com.opentrack.tracking.TrackerConfig
import kotlin.concurrent.thread
import kotlin.math.roundToInt

class MainActivity : AppCompatActivity() {

    private lateinit var overlay: OverlayView
    private lateinit var status: TextView
    private lateinit var recordButton: Button

    private var profile: CameraProfile? = null
    private var camera: CameraController? = null
    private var pipeline: RenderPipeline? = null
    private var analysis: AnalysisEngine? = null
    private var gyro: GyroSource? = null
    private var gyroModel: GyroMotionModel? = null
    private var recorder: Recorder? = null
    private var previewSurface: Surface? = null

    private var settings = CameraSettings()
    private var cropZoom = 2.0
    private var previewMode = PreviewMode.WIDE
    private var autoLock = true
    private var detectorMode = AnalysisEngine.Mode.HYBRID
    private var outputWidth = 1920
    private var outputHeight = 1080
    private var lastStatusMs = 0L

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        WindowCompat.getInsetsController(window, window.decorView).apply {
            hide(WindowInsetsCompat.Type.systemBars())
            systemBarsBehavior = WindowInsetsControllerCompat.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
        }

        overlay = findViewById(R.id.overlay)
        status = findViewById(R.id.status)
        recordButton = findViewById(R.id.record)
        overlay.onTap = { p -> analysis?.selectAt(p) }

        findViewById<android.view.SurfaceView>(R.id.preview).holder.addCallback(object : SurfaceHolder.Callback {
            override fun surfaceCreated(holder: SurfaceHolder) {
                previewSurface = holder.surface
                pipeline?.setPreviewSurface(holder.surface)
            }

            override fun surfaceChanged(holder: SurfaceHolder, format: Int, width: Int, height: Int) = Unit

            override fun surfaceDestroyed(holder: SurfaceHolder) {
                previewSurface = null
                pipeline?.setPreviewSurface(null)
            }
        })

        recordButton.setOnClickListener { toggleRecording() }
        findViewById<Button>(R.id.viewMode).setOnClickListener {
            previewMode = if (previewMode == PreviewMode.WIDE) PreviewMode.OUTPUT else PreviewMode.WIDE
            pipeline?.previewMode = previewMode
            refreshButtons()
        }
        findViewById<Button>(R.id.autoLock).setOnClickListener {
            autoLock = !autoLock
            analysis?.setAutoAcquire(autoLock)
            refreshButtons()
        }
        findViewById<Button>(R.id.releaseTarget).setOnClickListener { analysis?.clearTarget() }
        findViewById<Button>(R.id.detector).setOnClickListener {
            val modes = AnalysisEngine.Mode.entries
            detectorMode = modes[(detectorMode.ordinal + 1) % modes.size]
            analysis?.mode = detectorMode
            refreshButtons()
        }
        findViewById<Button>(R.id.focus).setOnClickListener {
            updateSettings(settings.copy(focusInfinity = !settings.focusInfinity))
        }
        findViewById<Button>(R.id.shutter).setOnClickListener {
            updateSettings(settings.copy(shutterPriority = !settings.shutterPriority))
        }
        refreshButtons()

        if (!hasPermission(Manifest.permission.CAMERA)) {
            requestPermissions(arrayOf(Manifest.permission.CAMERA, Manifest.permission.RECORD_AUDIO), REQUEST_PERMISSIONS)
        }
    }

    override fun onRequestPermissionsResult(requestCode: Int, permissions: Array<out String>, grantResults: IntArray) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode != REQUEST_PERMISSIONS) return
        if (hasPermission(Manifest.permission.CAMERA)) startPipeline()
        else Toast.makeText(this, R.string.permission_needed, Toast.LENGTH_LONG).show()
    }

    override fun onResume() {
        super.onResume()
        if (hasPermission(Manifest.permission.CAMERA)) startPipeline()
    }

    override fun onPause() {
        stopPipeline()
        super.onPause()
    }

    private fun startPipeline() {
        if (pipeline != null) return
        val manager = getSystemService(CameraManager::class.java)
        val p = CameraProfile.select(manager) ?: run {
            Toast.makeText(this, R.string.no_camera, Toast.LENGTH_LONG).show()
            return
        }
        profile = p
        val srcW = p.sourceSize.width
        val srcH = p.sourceSize.height
        if (srcW >= 2560) { outputWidth = 1920; outputHeight = 1080 } else { outputWidth = 1280; outputHeight = 720 }
        cropZoom = FramingController.nativeZoom(srcW, outputWidth)

        val rotationQuarter = display?.rotation ?: Surface.ROTATION_90
        val model = GyroMotionModel().apply {
            focalPx = p.focalPx * settings.zoomRatio
            axisMap = GyroMotionModel.forDisplayRotation(rotationQuarter * 90)
        }
        val sensors = getSystemService(SensorManager::class.java)
        val g = GyroSource(sensors, model)
        val motion: CameraMotion = if (g.available) model else CameraMotion.NONE
        if (g.available) g.start()
        gyro = g
        gyroModel = model
        // Base clock offset camera → gyro (SensorEvent timestamps use elapsedRealtimeNanos).
        val clockOffsetNs = if (p.timestampRealtime) 0L else SystemClock.elapsedRealtimeNanos() - System.nanoTime()

        // Without gyro, hand shake shows up as target motion: the filter must be more agile.
        val tracker = TargetTracker(motion, TrackerConfig(accelNoise = if (g.available) 1.5e6 else 8e6, autoAcquire = autoLock))
        val detector = AircraftDetector.create(this)
        if (detector == null) Toast.makeText(this, R.string.no_neural, Toast.LENGTH_SHORT).show()

        val analysisW = 640
        val analysisH = ((analysisW.toDouble() * srcH / srcW) / 2).roundToInt() * 2
        val engine = AnalysisEngine(tracker, detector, analysisW, analysisH, srcW.toDouble() / analysisW).also { it.mode = detectorMode }
        analysis = engine

        val framing = FramingController(srcW, srcH, outputWidth.toDouble() / outputHeight, motion).also { it.zoom = cropZoom }
        val rp = RenderPipeline(p, motion, tracker, engine, framing, rotationQuarter, object : RenderPipeline.Listener {
            override fun onCameraSurface(surface: Surface) {
                runOnUiThread { openCamera(p, surface, model, clockOffsetNs) }
            }

            override fun onOverlay(state: OverlayState) {
                overlay.state = state
                val now = SystemClock.uptimeMillis()
                if (now - lastStatusMs > 250) {
                    lastStatusMs = now
                    runOnUiThread { showStatus(state) }
                }
            }
        })
        rp.previewMode = previewMode
        pipeline = rp
        rp.start()
        previewSurface?.let { rp.setPreviewSurface(it) }
        setupSliders(p)
    }

    private fun openCamera(p: CameraProfile, surface: Surface, model: GyroMotionModel, clockOffsetNs: Long) {
        if (pipeline == null) return
        val cam = CameraController(getSystemService(CameraManager::class.java), p)
        cam.listener = object : CameraController.Listener {
            override fun onFrameMetadata(timestampNs: Long, exposureNs: Long, rollingShutterSkewNs: Long) {
                // Compare against the gyro at the middle of the exposure of the middle row.
                model.cameraToGyroOffsetNs = clockOffsetNs + rollingShutterSkewNs / 2 + exposureNs / 2
            }

            override fun onError(message: String) {
                runOnUiThread { Toast.makeText(this@MainActivity, message, Toast.LENGTH_LONG).show() }
            }
        }
        cam.update(settings)
        cam.open(surface)
        camera = cam
    }

    private fun stopPipeline() {
        if (recorder?.isRecording == true) stopRecording(blocking = true)
        camera?.close()
        camera = null
        gyro?.stop()
        gyro = null
        pipeline?.release()
        pipeline = null
        analysis?.release()
        analysis = null
    }

    private fun updateSettings(s: CameraSettings) {
        settings = s
        camera?.update(s)
        profile?.let { p -> gyroModel?.focalPx = p.focalPx * s.zoomRatio }
        refreshButtons()
    }

    private fun setupSliders(p: CameraProfile) {
        val crop = findViewById<SeekBar>(R.id.cropZoom)
        val cropLabel = findViewById<TextView>(R.id.cropZoomLabel)
        // Crop zoom 1.0x … 4.0x in 0.05 steps.
        crop.max = 60
        crop.progress = ((cropZoom - 1.0) / 0.05).roundToInt()
        cropLabel.text = getString(R.string.crop_zoom, cropZoom)
        crop.setOnSeekBarChangeListener(onChange { v ->
            cropZoom = 1.0 + v * 0.05
            pipeline?.framing?.zoom = cropZoom
            cropLabel.text = getString(R.string.crop_zoom, cropZoom)
        })

        val lens = findViewById<SeekBar>(R.id.lensZoom)
        val lensLabel = findViewById<TextView>(R.id.lensZoomLabel)
        val zMin = p.zoomRange.lower
        val zMax = minOf(p.zoomRange.upper, 10f)
        lens.max = ((zMax - zMin) * 10).roundToInt().coerceAtLeast(0)
        lens.progress = ((settings.zoomRatio - zMin) * 10).roundToInt()
        lensLabel.text = getString(R.string.lens_zoom, settings.zoomRatio)
        lens.setOnSeekBarChangeListener(onChange { v ->
            val ratio = zMin + v / 10f
            lensLabel.text = getString(R.string.lens_zoom, ratio)
            updateSettings(settings.copy(zoomRatio = ratio))
        })

        val ev = findViewById<SeekBar>(R.id.ev)
        val evLabel = findViewById<TextView>(R.id.evLabel)
        val evMin = p.aeCompensationRange.lower
        ev.max = p.aeCompensationRange.upper - evMin
        ev.progress = settings.aeCompensation - evMin
        evLabel.text = getString(R.string.ev, settings.aeCompensation)
        ev.setOnSeekBarChangeListener(onChange { v ->
            evLabel.text = getString(R.string.ev, evMin + v)
            updateSettings(settings.copy(aeCompensation = evMin + v))
        })
    }

    private fun onChange(block: (Int) -> Unit) = object : SeekBar.OnSeekBarChangeListener {
        override fun onProgressChanged(seekBar: SeekBar, progress: Int, fromUser: Boolean) {
            if (fromUser) block(progress)
        }

        override fun onStartTrackingTouch(seekBar: SeekBar) = Unit
        override fun onStopTrackingTouch(seekBar: SeekBar) = Unit
    }

    private fun toggleRecording() {
        if (recorder?.isRecording == true) stopRecording(blocking = false) else startRecording()
    }

    private fun startRecording() {
        val p = profile ?: return
        val rp = pipeline ?: return
        val rec = Recorder(contentResolver)
        try {
            val enc = rec.start(
                RecordingConfig(width = outputWidth, height = outputHeight, fps = p.fps),
                cameraTimestampRealtime = p.timestampRealtime,
                audioAllowed = hasPermission(Manifest.permission.RECORD_AUDIO),
            )
            rp.attachEncoder(enc.inputSurface, enc.width, enc.height)
            recorder = rec
            recordButton.isActivated = true
        } catch (e: Exception) {
            rec.stop()
            Toast.makeText(this, getString(R.string.save_failed) + ": " + e.message, Toast.LENGTH_LONG).show()
        }
    }

    private fun stopRecording(blocking: Boolean) {
        val rec = recorder ?: return
        recorder = null
        pipeline?.detachEncoder()
        recordButton.isActivated = false
        val finish = {
            val uri = rec.stop()
            runOnUiThread {
                val msg = if (uri != null) getString(R.string.saved, "Movies/OpenTrack") else getString(R.string.save_failed)
                Toast.makeText(this, msg, Toast.LENGTH_SHORT).show()
            }
        }
        if (blocking) finish() else thread(name = "recorder-stop") { finish() }
    }

    private fun showStatus(s: OverlayState) {
        val p = profile ?: return
        val stateText = getString(
            when (s.trackState) {
                TrackState.IDLE -> R.string.state_idle
                TrackState.TRACKING -> R.string.state_tracking
                TrackState.COASTING -> R.string.state_coasting
                TrackState.LOST -> R.string.state_lost
            },
        )
        status.text = getString(R.string.status, p.label, outputWidth, outputHeight, p.fps, s.fps, s.analysisMs, s.neuralMs, stateText)
    }

    private fun refreshButtons() {
        findViewById<Button>(R.id.viewMode).setText(if (previewMode == PreviewMode.WIDE) R.string.view_wide else R.string.view_output)
        findViewById<Button>(R.id.autoLock).setText(if (autoLock) R.string.auto_lock_on else R.string.auto_lock_off)
        findViewById<Button>(R.id.detector).setText(
            when (detectorMode) {
                AnalysisEngine.Mode.HYBRID -> R.string.detector_hybrid
                AnalysisEngine.Mode.NEURAL_ONLY -> R.string.detector_neural
                AnalysisEngine.Mode.SKY_ONLY -> R.string.detector_sky
            },
        )
        findViewById<Button>(R.id.focus).setText(if (settings.focusInfinity) R.string.focus_inf_on else R.string.focus_inf_off)
        findViewById<Button>(R.id.shutter).setText(if (settings.shutterPriority) R.string.shutter_fast_on else R.string.shutter_fast_off)
    }

    private fun hasPermission(p: String) = ContextCompat.checkSelfPermission(this, p) == PackageManager.PERMISSION_GRANTED

    private companion object {
        const val REQUEST_PERMISSIONS = 1
    }
}
