package com.opentrack.camera

import android.Manifest
import android.content.Intent
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
import androidx.core.content.edit
import androidx.core.view.WindowCompat
import androidx.core.view.WindowInsetsCompat
import androidx.core.view.WindowInsetsControllerCompat
import com.opentrack.camera.camera.CameraController
import com.opentrack.camera.camera.CameraProfile
import com.opentrack.camera.camera.CameraReport
import com.opentrack.camera.camera.CameraSettings
import com.opentrack.camera.camera.VendorKeys
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
import com.opentrack.tracking.RollingShutter
import com.opentrack.tracking.TargetTracker
import com.opentrack.tracking.TrackState
import com.opentrack.tracking.TrackerConfig
import com.opentrack.tracking.ZoomConfig
import com.opentrack.tracking.StabilizerConfig
import com.opentrack.tracking.VisualMotionEstimator
import kotlin.concurrent.thread
import kotlin.math.roundToInt

class MainActivity : AppCompatActivity() {

    private lateinit var overlay: OverlayView
    private lateinit var status: TextView
    private lateinit var recordButton: Button

    private var profile: CameraProfile? = null
    private val prefs by lazy { getSharedPreferences("camera", MODE_PRIVATE) }
    private var camera: CameraController? = null
    private var pipeline: RenderPipeline? = null
    private var analysis: AnalysisEngine? = null
    private var gyro: GyroSource? = null
    private var gyroModel: GyroMotionModel? = null
    private var tracker: TargetTracker? = null
    private var recorder: Recorder? = null
    private var previewSurface: Surface? = null

    private var settings = CameraSettings()
    private var cropZoom = 2.0
    private var zoomConfig = ZoomConfig(auto = true, minZoom = 1.0, maxZoom = 3.0)
    private var stabPreset = StabPreset.LOCK
    private var horizonLock = true

    /**
     * Look-ahead (seconds of delay), path smoothness, and whether the frame is locked to the
     * tracking point on the aircraft (point-tracker style) or follows it loosely.
     */
    private enum class StabPreset(val lookaheadS: Double, val smoothness: Double, val lock: Boolean, val label: Int) {
        LOCK(0.33, 0.5, true, R.string.stab_lock),
        RESPONSIVE(0.2, 0.4, false, R.string.stab_responsive),
        BALANCED(0.33, 0.6, false, R.string.stab_balanced),
        CINEMATIC(0.5, 0.8, false, R.string.stab_cinematic),
    }
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
        findViewById<Button>(R.id.stabilizer).setOnClickListener {
            val all = StabPreset.entries
            val nextPreset = all[(stabPreset.ordinal + 1) % all.size]
            if (applyStabilizer(nextPreset, horizonLock)) stabPreset = nextPreset
            refreshButtons()
        }
        findViewById<Button>(R.id.horizon).setOnClickListener {
            if (applyStabilizer(stabPreset, !horizonLock)) horizonLock = !horizonLock
            refreshButtons()
        }
        findViewById<Button>(R.id.resolution).setOnClickListener { cycleResolution() }
        findViewById<Button>(R.id.cameraReport).setOnClickListener { shareCameraReport() }
        findViewById<Button>(R.id.vendorUnlock).setOnClickListener {
            if (recorder?.isRecording == true) return@setOnClickListener
            prefs.edit { putBoolean(PREF_VENDOR, !prefs.getBoolean(PREF_VENDOR, false)) }
            if (prefs.getBoolean(PREF_VENDOR, false)) Toast.makeText(this, R.string.vendor_unlock_hint, Toast.LENGTH_LONG).show()
            stopPipeline()
            startPipeline()
            refreshButtons()
        }
        findViewById<Button>(R.id.autoZoom).setOnClickListener {
            updateZoomConfig(zoomConfig.copy(auto = !zoomConfig.auto))
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
        val p = CameraProfile.select(manager, prefs.getString(PREF_PROFILE, null), prefs.getBoolean(PREF_FORCE_4K60, false)) ?: run {
            Toast.makeText(this, R.string.no_camera, Toast.LENGTH_LONG).show()
            return
        }
        profile = p
        refreshButtons()
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
        // Gyro when available (exact, no latency); otherwise estimate camera motion from the image.
        val visual = if (g.available) null else VisualMotionEstimator()
        val motion: CameraMotion = if (g.available) model else visual!!
        if (g.available) g.start()
        gyro = g
        gyroModel = model
        // Base clock offset camera → gyro (SensorEvent timestamps use elapsedRealtimeNanos).
        val clockOffsetNs = if (p.timestampRealtime) 0L else SystemClock.elapsedRealtimeNanos() - System.nanoTime()

        // Image-space search: when the operator pans with the aircraft (the usual case), it stays
        // put in the image while moving fast in the world; searching there keeps the lock.
        val tracker = TargetTracker(
            motion,
            TrackerConfig.forSource(srcW, hasGyro = g.available).copy(autoAcquire = autoLock, imageSpaceSearch = true),
        )
        this.tracker = tracker
        val detector = AircraftDetector.create(this)
        if (detector == null) Toast.makeText(this, R.string.no_neural, Toast.LENGTH_SHORT).show()

        val analysisW = 640
        val analysisH = ((analysisW.toDouble() * srcH / srcW) / 2).roundToInt() * 2
        val engine = AnalysisEngine(tracker, detector, visual, motion, analysisW, analysisH, srcW.toDouble() / analysisW).also { it.mode = detectorMode }
        analysis = engine

        val framing = FramingController(srcW, srcH, outputWidth.toDouble() / outputHeight, motion).also {
            it.zoom = cropZoom
            it.zoomConfig = zoomConfig
        }
        val rp = RenderPipeline(p, motion, tracker, engine, framing, stabilizerConfig(p, stabPreset, horizonLock), outputWidth, outputHeight, rotationQuarter, object : RenderPipeline.Listener {
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
        val cam = CameraController(getSystemService(CameraManager::class.java), p, vendorExperiment = prefs.getBoolean(PREF_VENDOR, false))
        cam.listener = object : CameraController.Listener {
            override fun onFrameMetadata(timestampNs: Long, exposureNs: Long, rollingShutterSkewNs: Long) {
                // Compare against the gyro at the middle of the exposure of the middle row.
                model.cameraToGyroOffsetNs = clockOffsetNs + rollingShutterSkewNs / 2 + exposureNs / 2
                updateRollingShutter(p, rollingShutterSkewNs)
            }

            override fun onError(message: String) {
                runOnUiThread { Toast.makeText(this@MainActivity, message, Toast.LENGTH_LONG).show() }
            }
        }
        cam.update(settings)
        cam.open(surface)
        camera = cam
    }

    /**
     * Rolling shutter correction needs per-row camera motion, i.e. the gyro. The skew is
     * reported per frame but rarely changes; the model is only replaced when it does.
     */
    private fun updateRollingShutter(p: CameraProfile, skewNs: Long) {
        val rp = pipeline ?: return
        if (gyro?.available != true || skewNs <= 0) {
            rp.rollingShutter = null
            tracker?.rollingShutter = null
            return
        }
        val current = rp.rollingShutter
        if (current != null && kotlin.math.abs(current.skewNs - skewNs) < 200_000L) return
        val dir = RollingShutter.readoutDirection(p.sensorOrientation, display?.rotation ?: Surface.ROTATION_90)
        val extent = if (dir.x != 0.0) p.sourceSize.width.toDouble() else p.sourceSize.height.toDouble()
        val rs = RollingShutter(skewNs, dir, extent)
        rp.rollingShutter = rs
        tracker?.rollingShutter = rs
    }

    /**
     * Next stream profile the phone offers (best first), then an experimental forced 4K60 when
     * the phone does not list it (some makers keep 4K60 for their own camera app).
     */
    private fun cycleResolution() {
        if (recorder?.isRecording == true) return
        val manager = getSystemService(CameraManager::class.java)
        val labels = CameraProfile.available(manager).map { it.label }.toMutableList()
        if (labels.none { it.startsWith("3840x2160@60") }) labels += FORCED_4K60
        val current = profile?.label
        val next = labels[(labels.indexOf(current) + 1) % labels.size]
        prefs.edit {
            putString(PREF_PROFILE, next.removeSuffix("!"))
            putBoolean(PREF_FORCE_4K60, next == FORCED_4K60)
        }
        if (next == FORCED_4K60) Toast.makeText(this, R.string.forced_4k60_hint, Toast.LENGTH_LONG).show()
        stopPipeline()
        startPipeline()
        refreshButtons()
    }

    /** Shares what the camera stack offers to this app, to see where 4K60 went. */
    private fun shareCameraReport() {
        val text = runCatching {
            "Üretici deneyi: ${if (prefs.getBoolean(PREF_VENDOR, false)) "açık (" + VendorKeys.lastLog + ")" else "kapalı"}, " +
                "ölçülen fps: ${"%.1f".format(lastMeasuredFps)}\n" + CameraReport.build(getSystemService(CameraManager::class.java), profile)
        }.getOrElse { it.stackTraceToString() }
        val send = Intent(Intent.ACTION_SEND).setType("text/plain")
            .putExtra(Intent.EXTRA_SUBJECT, "Open Track Camera – kamera raporu")
            .putExtra(Intent.EXTRA_TEXT, text)
        startActivity(Intent.createChooser(send, getString(R.string.camera_report)))
    }

    private fun stopPipeline() {
        if (recorder?.isRecording == true) stopRecording(blocking = true)
        camera?.close()
        camera = null
        gyro?.stop()
        gyro = null
        pipeline?.release()
        pipeline = null
        tracker = null
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
        crop.isEnabled = !zoomConfig.auto
        crop.setOnSeekBarChangeListener(onChange { v ->
            cropZoom = 1.0 + v * 0.05
            pipeline?.framing?.zoom = cropZoom
            cropLabel.text = getString(R.string.crop_zoom, cropZoom)
        })

        // Auto zoom limits, same 1.0x … 4.0x scale as the crop zoom.
        val autoMinBar = findViewById<SeekBar>(R.id.zoomMin)
        val autoMaxBar = findViewById<SeekBar>(R.id.zoomMax)
        val autoMinLabel = findViewById<TextView>(R.id.zoomMinLabel)
        val autoMaxLabel = findViewById<TextView>(R.id.zoomMaxLabel)
        val native = FramingController.nativeZoom(p.sourceSize.width, outputWidth)
        fun label(v: Double) = getString(if (v > native + 1e-6) R.string.zoom_limit_upscaled else R.string.zoom_limit, v)
        for (bar in listOf(autoMinBar, autoMaxBar)) bar.max = 60
        autoMinBar.progress = ((zoomConfig.minZoom - 1.0) / 0.05).roundToInt()
        autoMaxBar.progress = ((zoomConfig.maxZoom - 1.0) / 0.05).roundToInt()
        autoMinLabel.text = getString(R.string.zoom_min, label(zoomConfig.minZoom))
        autoMaxLabel.text = getString(R.string.zoom_max, label(zoomConfig.maxZoom))
        autoMinBar.setOnSeekBarChangeListener(onChange { v ->
            val z = 1.0 + v * 0.05
            // Keep min <= max by pushing the other limit.
            if (z > zoomConfig.maxZoom) autoMaxBar.progress = v
            updateZoomConfig(zoomConfig.copy(minZoom = z, maxZoom = maxOf(z, zoomConfig.maxZoom)))
            autoMinLabel.text = getString(R.string.zoom_min, label(z))
            autoMaxLabel.text = getString(R.string.zoom_max, label(zoomConfig.maxZoom))
        })
        autoMaxBar.setOnSeekBarChangeListener(onChange { v ->
            val z = 1.0 + v * 0.05
            if (z < zoomConfig.minZoom) autoMinBar.progress = v
            updateZoomConfig(zoomConfig.copy(maxZoom = z, minZoom = minOf(z, zoomConfig.minZoom)))
            autoMaxLabel.text = getString(R.string.zoom_max, label(z))
            autoMinLabel.text = getString(R.string.zoom_min, label(zoomConfig.minZoom))
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

    private fun stabilizerConfig(p: CameraProfile, preset: StabPreset, horizon: Boolean): StabilizerConfig {
        val base = StabilizerConfig(
            delayFrames = (preset.lookaheadS * p.fps).roundToInt(),
            smoothness = preset.smoothness,
            rollStabilization = horizon,
        )
        // Point lock: the aircraft is pinned near the center, stabilized on its tracking point.
        return if (preset.lock) base.copy(lockToPoint = true, targetWeight = 20.0, maxTargetOffset = 0.3) else base
    }

    /** Returns false (and tells the user) when the change needs a new look-ahead buffer while recording. */
    private fun applyStabilizer(preset: StabPreset, horizon: Boolean): Boolean {
        val p = profile ?: return true
        val rp = pipeline ?: return true
        val ok = rp.setStabilizer(stabilizerConfig(p, preset, horizon))
        if (!ok) Toast.makeText(this, R.string.stab_locked_while_recording, Toast.LENGTH_SHORT).show()
        return ok
    }

    private fun updateZoomConfig(c: ZoomConfig) {
        val wasAuto = zoomConfig.auto
        zoomConfig = c
        pipeline?.framing?.let { f ->
            // Leaving auto mode keeps the current zoom as the new manual value.
            if (wasAuto && !c.auto) cropZoom = f.zoom
            f.zoomConfig = c
            if (!c.auto) f.zoom = cropZoom
        }
        val crop = findViewById<SeekBar>(R.id.cropZoom)
        crop.isEnabled = !c.auto
        crop.progress = ((cropZoom - 1.0) / 0.05).roundToInt()
        findViewById<TextView>(R.id.cropZoomLabel).text = getString(R.string.crop_zoom, cropZoom)
        refreshButtons()
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

    /** Frame rate the pipeline actually gets, for the camera report. */
    private var lastMeasuredFps = 0.0

    private fun showStatus(s: OverlayState) {
        lastMeasuredFps = s.fps
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
        if (zoomConfig.auto) {
            findViewById<TextView>(R.id.cropZoomLabel).text = getString(R.string.crop_zoom, s.zoom)
            findViewById<SeekBar>(R.id.cropZoom).progress = ((s.zoom - 1.0) / 0.05).roundToInt()
        }
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
        findViewById<Button>(R.id.autoZoom).setText(if (zoomConfig.auto) R.string.auto_zoom_on else R.string.auto_zoom_off)
        val manager = getSystemService(CameraManager::class.java)
        val vendor = findViewById<Button>(R.id.vendorUnlock)
        val hasVendorKeys = runCatching { profile?.let { VendorKeys.isAvailable(manager.getCameraCharacteristics(it.cameraId)) } ?: false }.getOrDefault(false)
        vendor.visibility = if (hasVendorKeys) android.view.View.VISIBLE else android.view.View.GONE
        vendor.setText(if (prefs.getBoolean(PREF_VENDOR, false)) R.string.vendor_unlock_on else R.string.vendor_unlock_off)
        findViewById<Button>(R.id.resolution).text = getString(R.string.resolution, profile?.label?.replace("3840x2160", "4K")?.replace("2560x1440", "1440p")?.replace("1920x1080", "1080p") ?: "-")
        findViewById<Button>(R.id.stabilizer).setText(stabPreset.label)
        findViewById<Button>(R.id.horizon).setText(if (horizonLock) R.string.horizon_on else R.string.horizon_off)
    }

    private fun hasPermission(p: String) = ContextCompat.checkSelfPermission(this, p) == PackageManager.PERMISSION_GRANTED

    private companion object {
        const val REQUEST_PERMISSIONS = 1
        const val PREF_PROFILE = "profile"
        const val PREF_FORCE_4K60 = "force4k60"
        const val PREF_VENDOR = "vendorUnlock"
        const val FORCED_4K60 = "3840x2160@60!"
    }
}
