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
import androidx.activity.result.PickVisualMediaRequest
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import androidx.core.content.edit
import androidx.core.net.toUri
import androidx.core.view.WindowCompat
import androidx.core.view.WindowInsetsCompat
import androidx.core.view.WindowInsetsControllerCompat
import com.opentrack.camera.camera.CameraController
import com.opentrack.camera.camera.CameraProfile
import com.opentrack.camera.camera.CameraReport
import com.opentrack.camera.camera.CameraSettings
import com.opentrack.camera.camera.VendorKeys
import com.opentrack.camera.diag.CrashLog
import com.opentrack.camera.diag.Telemetry
import com.opentrack.camera.offline.VideoProcessor
import com.opentrack.camera.detect.DetectorHost
import com.opentrack.camera.detect.DetectorModel
import com.opentrack.camera.detect.TargetClasses
import com.opentrack.camera.pipeline.AnalysisEngine
import com.opentrack.camera.pipeline.OverlayState
import com.opentrack.camera.pipeline.PreviewMode
import com.opentrack.camera.pipeline.RenderPipeline
import com.opentrack.camera.record.Recorder
import com.opentrack.camera.record.RecordingConfig
import com.opentrack.camera.record.VideoCodec
import com.opentrack.camera.record.bitrateFor
import com.opentrack.camera.sensor.GyroSource
import com.opentrack.camera.ui.OverlayView
import com.opentrack.tracking.CameraMotion
import com.opentrack.tracking.FramingController
import com.opentrack.tracking.GyroMotionModel
import com.opentrack.tracking.HybridZoom
import com.opentrack.tracking.LensCalibrator
import com.opentrack.tracking.LensModel
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
    /** The lens chosen with the lens button (null = the main lens, the phone's default choice). */
    private var lens: CameraProfile.Lens? = null
    /** The camera delivered at least one frame since it was opened (a failing lens never does). */
    @Volatile private var framesSeen = false
    /** Last camera / render errors, for the camera report. */
    private var lastCameraError: String? = null
    private var lastRenderError: String? = null
    /** Frame-by-frame log of the current recording (see [Telemetry]), when enabled. */
    private var telemetry: Telemetry? = null
    private var lastLoggedSettings: String? = null
    /**
     * One neural detector for the activity's lifetime, living on its own thread (see
     * [DetectorHost]) and shared by the live pipeline and gallery processing.
     */
    private var detector: DetectorHost? = null
    /** Why the app last died (see [CrashLog]), included in the camera report. */
    private var lastCrash: String? = null
    /** Full text of the last gallery processing error (the toast cuts it off). */
    private var lastProcessingError: String? = null

    private fun sharedDetector(): DetectorHost = detector ?: DetectorHost(applicationContext).also { detector = it }
    /** Gallery video being processed (the camera is closed meanwhile). */
    private var processor: VideoProcessor? = null
    private val pickVideo = registerForActivityResult(ActivityResultContracts.PickVisualMedia()) { uri -> uri?.let { processVideo(it) } }
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
    /** Hybrid zoom (seamless main ↔ telephoto on a multi-camera) wanted by the user. */
    private var hybridOn = true
    /** The camera zoom ratio hybrid zoom asked for (null = the user's, [settings]). */
    private var hybridTarget: Float? = null
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
    private var targetClasses = TargetClasses.AIRCRAFT
    /** Use the COCO model for aircraft too (compare with the own model). */
    private var preferCoco = false
    private val detectorModel get() = DetectorModel.forTarget(targetClasses, preferCoco)
    private var outputWidth = 1920
    private var outputHeight = 1080
    /** Output long side for 4K-class sources: 1920 (1080p) or 2560 (1440p). */
    private var outputLongSide = 1920
    private var codec = VideoCodec.AVC
    /** The last saved recording or processed video, for the share button. */
    private var lastVideo: android.net.Uri? = null
    private var lastStatusMs = 0L

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        CrashLog.install(this)
        settings = settings.copy(shutterNs = prefs.getLong(PREF_SHUTTER, CameraSettings().shutterNs), ois = prefs.getBoolean(PREF_OIS, false))
        detectorMode = AnalysisEngine.Mode.entries.firstOrNull { it.name == prefs.getString(PREF_DETECTOR, null) } ?: detectorMode
        targetClasses = TargetClasses.entries.firstOrNull { it.name == prefs.getString(PREF_TARGET, null) } ?: targetClasses
        outputLongSide = prefs.getInt(PREF_OUTPUT, 1920)
        preferCoco = prefs.getBoolean(PREF_COCO, false)
        codec = VideoCodec.entries.firstOrNull { it.name == prefs.getString(PREF_CODEC, null) } ?: codec
        lastVideo = prefs.getString(PREF_LAST_VIDEO, null)?.toUri()
        hybridOn = prefs.getBoolean(PREF_HYBRID, true)
        lastCrash = CrashLog.last(this)
        if (lastCrash != null) Toast.makeText(this, R.string.crash_recorded, Toast.LENGTH_LONG).show()
        setContentView(R.layout.activity_main)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        WindowCompat.getInsetsController(window, window.decorView).apply {
            hide(WindowInsetsCompat.Type.systemBars())
            systemBarsBehavior = WindowInsetsControllerCompat.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
        }

        overlay = findViewById(R.id.overlay)
        status = findViewById(R.id.status)
        recordButton = findViewById(R.id.record)
        // The wide view shows the camera frame: a tap there is mapped to the reference space.
        overlay.onTap = { p -> analysis?.selectAt(if (previewMode == PreviewMode.WIDE) pipeline?.frameMap?.toRef(p) ?: p else p) }

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
            prefs.edit { putString(PREF_DETECTOR, detectorMode.name) }
            refreshButtons()
        }
        findViewById<Button>(R.id.targetType).setOnClickListener {
            val all = TargetClasses.entries
            targetClasses = all[(targetClasses.ordinal + 1) % all.size]
            analysis?.targetClasses = targetClasses
            analysis?.model = detectorModel
            prefs.edit { putString(PREF_TARGET, targetClasses.name) }
            refreshButtons()
        }
        findViewById<Button>(R.id.focus).setOnClickListener {
            updateSettings(settings.copy(focusInfinity = !settings.focusInfinity))
        }
        findViewById<Button>(R.id.shutter).setOnClickListener {
            val next = SHUTTERS[(SHUTTERS.indexOf(settings.shutterNs) + 1).mod(SHUTTERS.size)]
            prefs.edit { putLong(PREF_SHUTTER, next) }
            updateSettings(settings.copy(shutterNs = next))
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
        findViewById<Button>(R.id.lens).setOnClickListener { cycleLens() }
        findViewById<Button>(R.id.lens).setOnLongClickListener { resetLensCalibration(); true }
        findViewById<Button>(R.id.hybrid).setOnClickListener {
            hybridOn = !hybridOn
            prefs.edit { putBoolean(PREF_HYBRID, hybridOn) }
            applyHybrid(announce = true)
        }
        findViewById<Button>(R.id.model).setOnClickListener {
            preferCoco = !preferCoco
            prefs.edit { putBoolean(PREF_COCO, preferCoco) }
            analysis?.model = detectorModel
            refreshButtons()
        }
        findViewById<Button>(R.id.codec).setOnClickListener {
            if (recorder?.isRecording == true) return@setOnClickListener
            val all = VideoCodec.entries
            val next = all[(codec.ordinal + 1) % all.size]
            val restart = next.tenBit != codec.tenBit // the GL output's bit depth is chosen at start
            codec = next
            prefs.edit { putString(PREF_CODEC, next.name) }
            if (!next.supported(outputWidth, outputHeight, profile?.fps ?: 60)) {
                Toast.makeText(this, getString(R.string.codec_unsupported, codecName(next)), Toast.LENGTH_LONG).show()
            }
            if (restart) {
                stopPipeline()
                startPipeline()
            }
            refreshButtons()
        }
        findViewById<Button>(R.id.share).setOnClickListener { shareLastVideo() }
        findViewById<Button>(R.id.ois).setOnClickListener {
            prefs.edit { putBoolean(PREF_OIS, !settings.ois) }
            updateSettings(settings.copy(ois = !settings.ois))
        }
        findViewById<Button>(R.id.outputRes).setOnClickListener {
            if (recorder?.isRecording == true) return@setOnClickListener
            outputLongSide = if (outputLongSide == 1920) 2560 else 1920
            prefs.edit { putInt(PREF_OUTPUT, outputLongSide) }
            stopPipeline()
            startPipeline()
            refreshButtons()
        }
        findViewById<Button>(R.id.telemetry).setOnClickListener {
            prefs.edit { putBoolean(PREF_TELEMETRY, !prefs.getBoolean(PREF_TELEMETRY, true)) }
            refreshButtons()
        }
        findViewById<Button>(R.id.cameraReport).setOnClickListener { shareCameraReport() }
        findViewById<Button>(R.id.processVideo).setOnClickListener {
            val running = processor
            when {
                running != null -> running.cancel()
                recorder?.isRecording == true -> Unit
                else -> pickVideo.launch(PickVisualMediaRequest(ActivityResultContracts.PickVisualMedia.VideoOnly))
            }
        }
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

    override fun onDestroy() {
        processor?.cancel()
        detector?.close()
        super.onDestroy()
    }

    private fun startPipeline() {
        if (pipeline != null || processor != null) return
        val manager = getSystemService(CameraManager::class.java)
        lens = prefs.getString(PREF_LENS, null)?.let { key ->
            CameraProfile.lenses(manager).drop(1).firstOrNull { it.key == key }
        }
        val p = CameraProfile.select(manager, prefs.getString(PREF_PROFILE, null), prefs.getBoolean(PREF_FORCE_4K60, false), lens) ?: run {
            Toast.makeText(this, R.string.no_camera, Toast.LENGTH_LONG).show()
            return
        }
        profile = p
        lowFpsSinceMs = 0L
        lowFpsWarned = false
        refreshButtons()
        val srcW = p.sourceSize.width
        val srcH = p.sourceSize.height
        // 4K-class sources (the telephoto's 3648 px too): 1080p or 1440p as chosen; else 720p.
        if (srcW >= 2560) { outputWidth = outputLongSide; outputHeight = outputLongSide * 9 / 16 } else { outputWidth = 1280; outputHeight = 720 }
        // Crop zooms are relative to the main lens: the stream's own pixels at the slider's zoom.
        cropZoom = FramingController.nativeZoom(srcW, outputWidth) * settings.zoomRatio

        val rotationQuarter = display?.rotation ?: Surface.ROTATION_90
        val model = GyroMotionModel().apply {
            // The reference space (see RenderPipeline.zoomAt, refZoom): set by applyReference.
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
        val detector = sharedDetector()

        val analysisW = 640
        val analysisH = ((analysisW.toDouble() * srcH / srcW) / 2).roundToInt() * 2
        val engine = AnalysisEngine(tracker, detector, visual, motion, analysisW, analysisH, srcW.toDouble() / analysisW).also {
            it.mode = detectorMode
            it.targetClasses = targetClasses
            it.model = detectorModel
            it.outputWidth = outputWidth
            it.onSceneLuma = { m, ts, blur -> camera?.onSceneLuma(m, ts, blur) }
            it.onLensCalibrated = { key, r -> onLensCalibrated(key, r) }
        }
        analysis = engine

        val framing = FramingController(srcW, srcH, outputWidth.toDouble() / outputHeight, motion).also {
            it.zoom = maxOf(1.0, cropZoom / settings.zoomRatio)
            it.zoomConfig = zoomConfig
        }
        var rpRef: RenderPipeline? = null
        val rp = RenderPipeline(p, motion, tracker, engine, framing, stabilizerConfig(p.fps, stabPreset, horizonLock), outputWidth, outputHeight, rotationQuarter, object : RenderPipeline.Listener {
            override fun onCameraSurface(surface: Surface) {
                runOnUiThread { openCamera(p, surface, model, clockOffsetNs) }
            }

            override fun onRenderError(message: String) {
                runOnUiThread {
                    lastRenderError = message
                    if (codec.tenBit && pipeline != null) {
                        // The 10-bit surfaces do not work on this phone: back to 8-bit HEVC.
                        codec = VideoCodec.HEVC
                        prefs.edit { putString(PREF_CODEC, codec.name) }
                        Toast.makeText(this@MainActivity, getString(R.string.recover_ten_bit, message), Toast.LENGTH_LONG).show()
                        stopPipeline()
                        startPipeline()
                        refreshButtons()
                    } else {
                        Toast.makeText(this@MainActivity, message, Toast.LENGTH_LONG).show()
                    }
                }
            }

            override fun onZoomRequest(ratio: Double) {
                runOnUiThread {
                    if (pipeline !== rpRef) return@runOnUiThread
                    val from = lensKeyFor(null, cameraZoom().toDouble())
                    hybridTarget = ratio.toFloat()
                    val to = lensKeyFor(null, ratio)
                    // Same brightness on the new lens: its aperture lets in more or less light.
                    camera?.updateForLens(cameraSettings(), to.substringAfter('/', "").takeIf { lens?.multi == true }, apertureLight(to) / apertureLight(from))

                }
            }

            override fun onOverlay(state: OverlayState) {
                overlay.state = state
                val now = SystemClock.uptimeMillis()
                if (now - lastStatusMs > 250) {
                    lastStatusMs = now
                    runOnUiThread { showStatus(state) }
                }
            }
        }, tenBit = codec.tenBit)
        rp.previewMode = previewMode
        rp.focalPx = p.focalPx * settings.zoomRatio
        rp.baseZoom = settings.zoomRatio.toDouble()
        rp.zoomRequested(settings.zoomRatio.toDouble())
        rp.zoomAt = { ts -> camera?.zoomAt(ts) }
        loadLensModels(p)
        engine.calibratedLenses.addAll(lensModels.keys)
        rp.lensKeyAt = { ts, z -> lensKeyFor(ts, z) }
        rp.lensModel = { key -> lensModelFor(key) }
        rpRef = rp
        pipeline = rp
        applyHybrid(announce = false)
        rp.start()
        previewSurface?.let { rp.setPreviewSurface(it) }
        setupSliders(p)
    }

    private fun openCamera(p: CameraProfile, surface: Surface, model: GyroMotionModel, clockOffsetNs: Long) {
        if (pipeline == null) return
        val cam = CameraController(getSystemService(CameraManager::class.java), p, vendorExperiment = prefs.getBoolean(PREF_VENDOR, false))
        framesSeen = false
        cam.listener = object : CameraController.Listener {
            override fun onFrameMetadata(timestampNs: Long, exposureNs: Long, rollingShutterSkewNs: Long) {
                framesSeen = true
                // Compare against the gyro at the middle of the exposure of the middle row.
                model.cameraToGyroOffsetNs = clockOffsetNs + rollingShutterSkewNs / 2 + exposureNs / 2
                updateRollingShutter(p, rollingShutterSkewNs)
            }

            override fun onError(message: String) {
                runOnUiThread {
                    lastCameraError = "${p.label} kamera ${p.lensKey}: $message"
                    if (!framesSeen && camera === cam) recoverFromNoFrames(message)
                    else Toast.makeText(this@MainActivity, message, Toast.LENGTH_LONG).show()
                }
            }
        }
        cam.update(cameraSettings())
        camera = cam
        cam.open(surface)
        // A camera that opens but never delivers a frame (a forced mode the sensor refuses, the
        // vendor keys, a hidden lens) reports no error: check after a while.
        overlay.postDelayed({
            if (camera === cam && !framesSeen) {
                lastCameraError = "${p.label} kamera ${p.lensKey}: ${FRAME_TIMEOUT_MS / 1000} s içinde kare gelmedi"
                recoverFromNoFrames(getString(R.string.no_frames))
            }
        }, FRAME_TIMEOUT_MS)
    }

    /**
     * The camera gives no frames: undo, one per attempt, the experimental choices that can cause
     * it (vendor keys, a hidden lens, a forced 4K60), then restart. Each step changes a setting,
     * so this cannot loop; with nothing left to undo it only says so.
     */
    private fun recoverFromNoFrames(message: String) {
        val p = profile
        val step = when {
            prefs.getBoolean(PREF_VENDOR, false) -> {
                prefs.edit { putBoolean(PREF_VENDOR, false) }
                R.string.recover_vendor
            }
            lens != null -> {
                prefs.edit { remove(PREF_LENS) }
                R.string.lens_failed
            }
            p?.forced == true -> {
                prefs.edit { putBoolean(PREF_FORCE_4K60, false); remove(PREF_PROFILE) }
                R.string.recover_forced
            }
            else -> {
                Toast.makeText(this, message, Toast.LENGTH_LONG).show()
                return
            }
        }
        Toast.makeText(this, getString(step, message), Toast.LENGTH_LONG).show()
        stopPipeline()
        startPipeline()
        refreshButtons()
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
        val profiles = CameraProfile.available(manager, lens)
        val labels = profiles.map { it.label }.toMutableList()
        if (profiles.none { it.sourceSize.width >= 3456 && it.fps == 60 }) labels += FORCED_4K60
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

    /**
     * Next back lens (main, then widest to longest). Lenses the phone does not list for other
     * apps are offered too; when one does not open, [openCamera] falls back to the main lens.
     */
    private fun cycleLens() {
        if (recorder?.isRecording == true) return
        val all = CameraProfile.lenses(getSystemService(CameraManager::class.java))
        if (all.size < 2) return
        val current = all.indexOfFirst { it.key == (lens?.key ?: all.first().key) }
        val next = all[(current + 1).mod(all.size)]
        prefs.edit { if (next.key == all.first().key) remove(PREF_LENS) else putString(PREF_LENS, next.key) }
        if (!next.listed) Toast.makeText(this, R.string.lens_hidden_hint, Toast.LENGTH_LONG).show()
        stopPipeline()
        startPipeline()
        refreshButtons()
    }

    private fun codecName(c: VideoCodec): String = getString(
        when (c) {
            VideoCodec.AVC -> R.string.codec_avc
            VideoCodec.HEVC -> R.string.codec_hevc
            VideoCodec.HEVC_10 -> R.string.codec_hevc10
        },
    )

    private fun rememberVideo(uri: android.net.Uri) {
        lastVideo = uri
        prefs.edit { putString(PREF_LAST_VIDEO, uri.toString()) }
        refreshButtons()
    }

    /**
     * Shares the last recording (or processed video) as a file through Android's share sheet:
     * Instagram, WhatsApp etc. take it like a gallery upload, their own cameras never involved.
     */
    private fun shareLastVideo() {
        val uri = lastVideo ?: return
        val exists = runCatching { contentResolver.openFileDescriptor(uri, "r")?.close() }.isSuccess
        if (!exists) {
            Toast.makeText(this, R.string.share_missing, Toast.LENGTH_SHORT).show()
            lastVideo = null
            prefs.edit { remove(PREF_LAST_VIDEO) }
            refreshButtons()
            return
        }
        val send = Intent(Intent.ACTION_SEND).setType("video/mp4")
            .putExtra(Intent.EXTRA_STREAM, uri)
            .addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
        startActivity(Intent.createChooser(send, getString(R.string.share)))
    }

    /** Button text for the current lens. */
    private fun lensLabel(): String {
        val l = lens ?: return getString(R.string.lens, getString(R.string.lens_main), 1.0, "")
        if (l.multi) {
            val r = l.zoomRange
            return getString(R.string.lens_multi, r?.lower ?: 1f, r?.upper ?: 1f, if (l.listed) "" else getString(R.string.lens_hidden))
        }
        val kind = when {
            l.zoomFactor < 0.85 -> R.string.lens_wide
            l.zoomFactor > 1.3 -> R.string.lens_tele
            else -> R.string.lens_main
        }
        return getString(R.string.lens, getString(kind), l.zoomFactor, if (l.listed) "" else getString(R.string.lens_hidden))
    }

    /**
     * Processes a gallery video (e.g. 4K60 from the phone's own camera app) like the live
     * pipeline does and saves the 1080p result; the camera is closed meanwhile.
     */
    private fun processVideo(uri: android.net.Uri) {
        stopPipeline()
        status.text = getString(R.string.processing, 0, 0.0)
        val p = VideoProcessor(
            applicationContext, uri, sharedDetector(), zoomConfig.copy(minZoom = maxOf(1.0, zoomConfig.minZoom), maxZoom = maxOf(1.0, zoomConfig.maxZoom)), { fps -> stabilizerConfig(fps, stabPreset, horizonLock) },
            targetClasses.labels, detectorModel, detectorMode != AnalysisEngine.Mode.NEURAL_ONLY, outputLongSide, codec,
            object : VideoProcessor.Listener {
            override fun onProgress(fraction: Double, processingFps: Double) = runOnUiThread {
                status.text = getString(R.string.processing, (fraction * 100).roundToInt(), processingFps) + " · " + (detector?.shortStatus() ?: "NN -")
            }

            override fun onFinished(output: android.net.Uri?, error: String?) = runOnUiThread {
                output?.let { rememberVideo(it) }
                processor = null
                if (error != null) lastProcessingError = error
                refreshButtons()
                val msg = when {
                    output != null -> getString(R.string.processing_done)
                    error != null -> getString(R.string.processing_failed, error)
                    else -> getString(R.string.processing_cancelled)
                }
                Toast.makeText(this@MainActivity, msg, Toast.LENGTH_LONG).show()
                if (hasPermission(Manifest.permission.CAMERA) && !isFinishing) startPipeline()
            }
        })
        processor = p
        refreshButtons()
        p.start()
    }

    /** Shares what the camera stack offers to this app, to see where 4K60 went. */
    private fun shareCameraReport() {
        val text = runCatching {
            (lastCrash?.let { "$it\n\n" } ?: "") + (detector?.report()?.let { "$it\n" } ?: "") + (lastProcessingError?.let { "İşleme hatası: $it\n" } ?: "") +
                (lastCameraError?.let { "Son kamera hatası: $it\n" } ?: "") + ((lastRenderError ?: pipeline?.lastError)?.let { "Görüntü hatası: $it\n" } ?: "") +
                "Lens kalibrasyonu: ${lensModels.entries.joinToString { (k, m) -> "$k odak %.0f px distorsiyon %.2f".format(java.util.Locale.ROOT, m.kappa, m.a) }.ifEmpty { "henüz yok" }}" +
                    " (önsel odak %.0f px)\n".format(java.util.Locale.ROOT, profile?.focalPx ?: 0.0) +
                "Codec: ${codec.name}, 10-bit GPU: ${if (pipeline?.tenBitActive == true) "evet" else "hayır"}, lens: ${lens?.key ?: "ana"}, kare geldi: ${if (framesSeen) "evet" else "hayır"}\n" + "Üretici deneyi: ${if (prefs.getBoolean(PREF_VENDOR, false)) "açık (" + VendorKeys.lastLog + ")" else "kapalı"}, " +
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
        val zoomChanged = s.zoomRatio != settings.zoomRatio
        settings = s
        // Without hybrid zoom the camera zoom slider sets the reference space.
        if (zoomChanged && pipeline?.hybrid == null) applyReference()
        camera?.update(cameraSettings())
        refreshButtons()
    }

    /** What the camera gets: the user's settings, at the zoom ratio hybrid zoom chose. */
    private fun cameraSettings(): CameraSettings = settings.copy(zoomRatio = cameraZoom())

    /** The camera zoom ratio: hybrid zoom's choice, else the camera zoom slider. */
    private fun cameraZoom(): Float = ladder()?.takeIf { pipeline?.hybrid != null }?.let { hybridTarget ?: it.first() } ?: settings.zoomRatio

    /**
     * The camera zoom ratio of the reference space: tracking, framing and stabilization work in
     * that frame (see RenderPipeline.zoomAt): with hybrid zoom its widest lens, else the camera
     * zoom slider. Crop zooms shown and set are relative to the main lens (x this).
     */
    private fun refZoom(): Double = (ladder()?.takeIf { pipeline?.hybrid != null }?.first() ?: settings.zoomRatio).toDouble()

    /** Lens kinds of a multi-camera's physical lenses, by camera id: angle of view vs the main lens. */
    private var lensFactors: Map<String, Double> = emptyMap()

    /**
     * The camera zoom ratios hybrid zoom moves between on the current multi-camera: the main
     * lens (1x) and its telephoto (the ratio its HAL switches to it, rounded up a little); null
     * without both.
     */
    private fun ladder(): List<Float>? {
        val l = lens?.takeIf { it.multi } ?: return null
        // (Per stream width too: the lens priors are in stream px.)
        val cacheKey = "${l.key}@${profile?.sourceSize?.width}"
        ladderCache?.let { (key, r) -> if (key == cacheKey) return r }
        return computeLadder(l).also { ladderCache = cacheKey to it }
    }

    /** Light a lens lets in, relative (1 / f-number²); 1 when unknown. */
    private fun apertureLight(key: String): Double {
        val id = key.substringAfter('/', "").ifEmpty { return 1.0 }
        val n = apertures.getOrPut(id) {
            runCatching {
                getSystemService(CameraManager::class.java).getCameraCharacteristics(id)
                    .get(android.hardware.camera2.CameraCharacteristics.LENS_INFO_AVAILABLE_APERTURES)?.firstOrNull()?.toDouble()
            }.getOrNull() ?: 0.0
        }
        return if (n > 0) 1 / (n * n) else 1.0
    }

    private val apertures = HashMap<String, Double>()

    /** Prior center focal per unit zoom ratio (stream px) of each physical lens: see [lensPrior]. */
    private fun physicalPriors(physical: List<CameraProfile.Lens>, lowest: Double): Map<String, Double> {
        val w = profile?.sourceSize?.width ?: return emptyMap()
        val manager = getSystemService(CameraManager::class.java)
        val full = physical.mapNotNull { l ->
            val id = l.physicalId ?: l.cameraId
            val c = runCatching { manager.getCameraCharacteristics(id) }.getOrNull() ?: return@mapNotNull null
            // The stream spans the sensor's width.
            CameraProfile.focalArray(c)?.let { (f, arrayW, _) -> Triple(id, l.zoomFactor, f * w / arrayW) }
        }
        val main = full.minByOrNull { kotlin.math.abs(it.second - 1.0) } ?: return emptyMap()
        return full.associate { (id, factor, f) ->
            id to when {
                factor < 0.85 -> f / lowest // its whole sensor at the lowest ratio
                else -> main.third // main: 1x; telephoto: the ratio is its real scale
            }
        }
    }

    /** [ladder] per multi-camera (listing the lenses opens every camera's characteristics). */
    private var ladderCache: Pair<String, List<Float>?>? = null

    private fun computeLadder(l: CameraProfile.Lens): List<Float>? {
        val range = l.zoomRange ?: return null
        val physical = CameraProfile.lenses(getSystemService(CameraManager::class.java))
            .filter { !it.multi && (it.cameraId in l.physicalIds || it.physicalId in l.physicalIds) }
        lensFactors = physical.associate { (it.physicalId ?: it.cameraId) to it.zoomFactor }
        physicalPriors = physicalPriors(physical, range.lower.toDouble())
        val out = ArrayList<Float>()
        // Main and telephoto only: the wide angle (its own scale and strong distortion) made the
        // switches visible, and a close aircraft fits the main lens's crop anyway.
        out += 1f.coerceIn(range.lower, range.upper)
        physical.maxOfOrNull { it.zoomFactor }?.takeIf { it >= 1.3 }?.let { tele ->
            out += (kotlin.math.ceil(tele * 10 - 1e-6) / 10).toFloat().coerceIn(range.lower, range.upper)
        }
        return out.distinct().takeIf { it.size >= 2 }
    }

    /** The crop zoom (vs the main lens) from which the telephoto is used, or null. */
    private fun hybridFromZoom(): Double? = ladder()?.takeIf { pipeline?.hybrid != null && it.last() > 1.3f }?.let { scaleOf(it.last().toDouble()) * HybridZoom.ENTER_ROOM }

    /** Creates (or removes) hybrid zoom for the current camera and settings. */
    private fun applyHybrid(announce: Boolean) {
        val rp = pipeline
        val p = profile
        val steps = ladder()
        val wasOn = rp?.hybrid != null
        val hz = if (hybridOn && rp != null && p != null && steps != null && analysis?.supportsWindows == true) {
            HybridZoom(hybridLevels(steps) ?: steps.map { it.toDouble() / steps.first() }, p.sourceSize.width, p.sourceSize.height)
        } else null
        rp?.hybrid = hz
        hybridTarget = null
        applyReference()
        if (hz != null && !wasOn && steps != null) {
            // The auto zoom must reach from the wide angle to the telephoto.
            val far = scaleOf(steps.last().toDouble()) * HybridZoom.ENTER_ROOM
            updateZoomConfig(zoomConfig.copy(minZoom = uiRef(), maxZoom = minOf(ZOOM_MAX, maxOf(zoomConfig.maxZoom, kotlin.math.ceil(far + 2.0)))))
        } else if (zoomConfig.minZoom < uiRef()) {
            // Narrower reference (hybrid zoom off): limits below its whole frame mean that frame.
            updateZoomConfig(zoomConfig.copy(minZoom = uiRef(), maxZoom = maxOf(uiRef(), zoomConfig.maxZoom)))
        }
        camera?.update(cameraSettings())
        p?.let { setupSliders(it) }
        if (announce) {
            val msg = when {
                !hybridOn -> getString(R.string.hybrid_off_hint)
                hz == null || steps == null -> getString(R.string.hybrid_unavailable)
                else -> getString(R.string.hybrid_on_hint, steps.joinToString(" / ") { "%.1fx".format(java.util.Locale.ROOT, scaleOf(it.toDouble())) }, hybridFromZoom() ?: 0.0)
            }
            Toast.makeText(this, msg, Toast.LENGTH_LONG).show()
        }
        refreshButtons()
    }

    /** The reference space changed (camera zoom slider, hybrid zoom on/off): focal length and crop zoom follow. */
    private fun applyReference() {
        val ref = refZoom()
        profile?.let { p ->
            // The reference lens's real center focal at that ratio (see LensModel).
            val f = lensModelFor(lensKeyFor(null, ref)).focalAt(ref)
            gyroModel?.focalPx = f
            pipeline?.focalPx = f
        }
        pipeline?.baseZoom = ref
        pipeline?.zoomRequested(cameraZoom().toDouble())
        applyFraming()
    }

    /** Lens models measured on this phone (LensCalibrator), by lens key; the rest use [lensPrior]. */
    private val lensModels = java.util.concurrent.ConcurrentHashMap<String, LensModel>()

    /** Calibrations that came in while recording: applied when it stops (the mapping must not jump mid-video). */
    private val pendingLensModels = HashMap<String, LensModel>()

    /**
     * Without a measurement: each physical lens's own calibrated focal (LENS_INTRINSIC_CALIBRATION,
     * scaled to the stream), at the zoom ratio where its whole sensor is used (the wide angle at
     * the lowest ratio; the main lens at 1x; the telephoto as the ratio says); a pinhole.
     */
    private fun lensPrior(key: String): LensModel {
        val id = key.substringAfter('/', "")
        return physicalPriors[id]?.let { LensModel(it) } ?: LensModel(profile?.focalPx ?: 1.0)
    }

    /** [lensPrior] kappa per physical lens id of the multi-camera (see [computeLadder]). */
    @Volatile private var physicalPriors: Map<String, Double> = emptyMap()

    private fun lensModelFor(key: String): LensModel = lensModels[key] ?: lensPrior(key)

    /**
     * The physical lens a frame came from: the one the camera reports for it ([ts]), else, on a
     * multi-camera, the one its zoom ratio selects (the wide angle below 1x, the telephoto from
     * its ratio); "camera id/physical id".
     */
    private fun lensKeyFor(ts: Long?, zoom: Double): String {
        val p = profile ?: return "-"
        if (lens?.multi != true) return p.physicalCameraId?.let { "${p.cameraId}/$it" } ?: p.cameraId
        val reported = ts?.let { camera?.physicalAt(it) }
        val id = reported ?: run {
            val factors = lensFactors
            val tele = ladder()?.last()?.toDouble() ?: Double.MAX_VALUE
            when {
                zoom < 0.95 -> factors.minByOrNull { it.value }?.key
                zoom >= tele - 0.05 && tele > 1.3 -> factors.maxByOrNull { it.value }?.key
                else -> factors.minByOrNull { kotlin.math.abs(it.value - 1.0) }?.key
            } ?: "z%.1f".format(java.util.Locale.ROOT, zoom)
        }
        return "${p.cameraId}/$id"
    }

    /** Loads this phone's lens calibrations (stored per stream width). */
    private fun loadLensModels(p: CameraProfile) {
        lensModels.clear()
        for ((k, v) in prefs.all) {
            if (!k.startsWith(PREF_LENS_CAL)) continue
            val parts = (v as? String)?.split(";") ?: continue
            val kw = parts.getOrNull(0)?.toDoubleOrNull() ?: continue
            val a = parts.getOrNull(1)?.toDoubleOrNull() ?: 0.0
            lensModels[k.removePrefix(PREF_LENS_CAL)] = LensModel(kw * p.sourceSize.width, a)
        }
    }

    /** A lens calibration converged (analysis): store it, use it (after the recording, if one runs). */
    private fun onLensCalibrated(key: String, r: LensCalibrator.Result) = runOnUiThread {
        val p = profile ?: return@runOnUiThread
        prefs.edit { putString(PREF_LENS_CAL + key, "${r.model.kappa / p.sourceSize.width};${r.model.a}") }
        val before = lensModelFor(key)
        if (recorder?.isRecording == true) pendingLensModels[key] = r.model else applyLensModel(key, r.model)
        Toast.makeText(
            this,
            getString(R.string.lens_calibrated, lensNameFor(key), r.model.kappa / before.kappa, r.model.a, r.priorRmsPx, r.rmsPx),
            Toast.LENGTH_LONG,
        ).show()
    }

    private fun applyLensModel(key: String, m: LensModel) {
        lensModels[key] = m
        // The reference lens: the gyroscope's focal length changes with it.
        if (key == lensKeyFor(null, refZoom())) applyReference()
        // Hybrid zoom's lens views follow (staying on the lens it is on).
        val rp = pipeline
        val p = profile
        val old = rp?.hybrid
        val levels = ladder()?.let { hybridLevels(it) }
        if (rp != null && p != null && old != null && levels != null && levels.size == old.levels.size) {
            rp.hybrid = HybridZoom(levels, p.sourceSize.width, p.sourceSize.height, startLevel = old.level)
        }
        p?.let { setupSliders(it) }
        refreshButtons()
    }

    /** "GENİŞ"/"ANA"/"TELE" for a lens key. */
    private fun lensNameFor(key: String): String {
        val id = key.substringAfter('/', key)
        val factor = lensFactors[id] ?: lens?.zoomFactor ?: 1.0
        return getString(
            when {
                factor < 0.85 -> R.string.lens_wide
                factor > 1.3 -> R.string.lens_tele
                else -> R.string.lens_main
            },
        ).uppercase(java.util.Locale.getDefault())
    }

    /** Forgets this phone's lens calibrations (long press on the lens button); they are measured again. */
    private fun resetLensCalibration() {
        prefs.edit { prefs.all.keys.filter { it.startsWith(PREF_LENS_CAL) }.forEach { remove(it) } }
        lensModels.clear()
        pendingLensModels.clear()
        analysis?.calibratedLenses?.clear()
        applyReference()
        Toast.makeText(this, R.string.lens_calibration_reset, Toast.LENGTH_LONG).show()
    }

    /** Crop zoom settings (vs the main lens) → the framing, which works in the reference space. */
    private fun applyFraming() {
        val f = pipeline?.framing ?: return
        val ref = uiRef()
        f.zoomConfig = zoomConfig.copy(minZoom = maxOf(1.0, zoomConfig.minZoom / ref), maxZoom = maxOf(1.0, zoomConfig.maxZoom / ref))
        if (!zoomConfig.auto) f.zoom = maxOf(1.0, cropZoom / ref)
    }

    private fun setupSliders(p: CameraProfile) {
        // Crop zooms are relative to the main lens (like the phone's own camera): from the
        // reference space's whole frame (0.4x with the wide angle) to ZOOM_MAX, in 0.05 steps.
        val low = uiRef()
        val steps = ((ZOOM_MAX - low) / 0.05).roundToInt()
        fun progressOf(z: Double) = ((z - low) / 0.05).roundToInt().coerceIn(0, steps)
        fun zoomOf(v: Int) = low + v * 0.05
        val crop = findViewById<SeekBar>(R.id.cropZoom)
        val cropLabel = findViewById<TextView>(R.id.cropZoomLabel)
        crop.max = steps
        cropZoom = cropZoom.coerceIn(low, ZOOM_MAX)
        crop.progress = progressOf(cropZoom)
        cropLabel.text = getString(R.string.crop_zoom, cropZoom)
        crop.isEnabled = !zoomConfig.auto
        crop.setOnSeekBarChangeListener(onChange { v ->
            cropZoom = zoomOf(v)
            applyFraming()
            cropLabel.text = getString(R.string.crop_zoom, cropZoom)
        })

        // Auto zoom limits, same scale as the crop zoom.
        val autoMinBar = findViewById<SeekBar>(R.id.zoomMin)
        val autoMaxBar = findViewById<SeekBar>(R.id.zoomMax)
        val autoMinLabel = findViewById<TextView>(R.id.zoomMinLabel)
        val autoMaxLabel = findViewById<TextView>(R.id.zoomMaxLabel)
        // Up to here the longest camera zoom (the telephoto with hybrid zoom) has pixels for the output.
        val native = FramingController.nativeZoom(p.sourceSize.width, outputWidth) * cameraZoomMax()
        fun label(v: Double) = getString(if (v > native + 1e-6) R.string.zoom_limit_upscaled else R.string.zoom_limit, v)
        for (bar in listOf(autoMinBar, autoMaxBar)) bar.max = steps
        autoMinBar.progress = progressOf(zoomConfig.minZoom)
        autoMaxBar.progress = progressOf(zoomConfig.maxZoom)
        autoMinLabel.text = getString(R.string.zoom_min, label(zoomConfig.minZoom))
        autoMaxLabel.text = getString(R.string.zoom_max, label(zoomConfig.maxZoom))
        autoMinBar.setOnSeekBarChangeListener(onChange { v ->
            val z = zoomOf(v)
            // Keep min <= max by pushing the other limit.
            if (z > zoomConfig.maxZoom) autoMaxBar.progress = v
            updateZoomConfig(zoomConfig.copy(minZoom = z, maxZoom = maxOf(z, zoomConfig.maxZoom)))
            autoMinLabel.text = getString(R.string.zoom_min, label(z))
            autoMaxLabel.text = getString(R.string.zoom_max, label(zoomConfig.maxZoom))
        })
        autoMaxBar.setOnSeekBarChangeListener(onChange { v ->
            val z = zoomOf(v)
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
        // With hybrid zoom the camera zoom is chosen automatically (the lens per crop).
        val hybridActive = pipeline?.hybrid != null
        lens.isEnabled = !hybridActive
        if (hybridActive) lensLabel.text = getString(R.string.lens_zoom_hybrid)
        lens.setOnSeekBarChangeListener(onChange { v ->
            val ratio = zMin + v / 10f
            lensLabel.text = getString(R.string.lens_zoom, ratio)
            updateSettings(settings.copy(zoomRatio = ratio))
        })

        val ev = findViewById<SeekBar>(R.id.ev)
        val evLabel = findViewById<TextView>(R.id.evLabel)
        // −3..+3 EV in thirds of a stop.
        ev.max = 18
        ev.progress = settings.evThirds + 9
        evLabel.text = getString(R.string.ev, settings.evThirds / 3.0)
        ev.setOnSeekBarChangeListener(onChange { v ->
            evLabel.text = getString(R.string.ev, (v - 9) / 3.0)
            updateSettings(settings.copy(evThirds = v - 9))
        })
    }

    private fun stabilizerConfig(fps: Int, preset: StabPreset, horizon: Boolean): StabilizerConfig {
        val base = StabilizerConfig(
            delayFrames = (preset.lookaheadS * fps).roundToInt(),
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
        val ok = rp.setStabilizer(stabilizerConfig(p.fps, preset, horizon))
        if (!ok) Toast.makeText(this, R.string.stab_locked_while_recording, Toast.LENGTH_SHORT).show()
        return ok
    }

    private fun updateZoomConfig(c: ZoomConfig) {
        val wasAuto = zoomConfig.auto
        zoomConfig = c
        // Leaving auto mode keeps the current zoom as the new manual value.
        pipeline?.framing?.let { f -> if (wasAuto && !c.auto) cropZoom = f.zoom * uiRef() }
        applyFraming()
        val crop = findViewById<SeekBar>(R.id.cropZoom)
        crop.isEnabled = !c.auto
        crop.progress = ((cropZoom - uiRef()) / 0.05).roundToInt()
        findViewById<TextView>(R.id.cropZoomLabel).text = getString(R.string.crop_zoom, cropZoom)
        refreshButtons()
    }

    /** The longest camera zoom ratio in use: the telephoto's with hybrid zoom, else the slider's. */
    private fun cameraZoomMax(): Double = scaleOf((ladder()?.takeIf { pipeline?.hybrid != null }?.last() ?: settings.zoomRatio).toDouble())

    /**
     * The real zoom (vs the main lens at 1x) of a frame taken at camera zoom ratio [z]: from the
     * lens models, not the ratio (the wide angle's 0.4x is really ~0.67x on this phone).
     */
    private fun scaleOf(z: Double): Double {
        val main = lensModelFor(lensKeyFor(null, 1.0)).focalAt(1.0)
        return lensModelFor(lensKeyFor(null, z)).focalAt(z) / main
    }

    /** [scaleOf] the reference space: crop zooms shown and set are relative to the main lens. */
    private fun uiRef(): Double = scaleOf(refZoom())

    /** Hybrid zoom's levels: each lens's real view relative to the widest; null when not rising. */
    private fun hybridLevels(steps: List<Float>): List<Double>? {
        val base = scaleOf(steps.first().toDouble())
        val levels = steps.map { scaleOf(it.toDouble()) / base }
        return levels.takeIf { it.first() == 1.0 && it.zipWithNext().all { (a, b) -> b > a * 1.05 } }
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
                RecordingConfig(width = outputWidth, height = outputHeight, fps = p.fps, bitrate = bitrateFor(outputWidth), codec = codec),
                cameraTimestampRealtime = p.timestampRealtime,
                audioAllowed = hasPermission(Manifest.permission.RECORD_AUDIO),
            )
            when {
                enc.codec != codec -> Toast.makeText(this, getString(R.string.codec_fallback, codecName(codec), codecName(enc.codec)), Toast.LENGTH_LONG).show()
                codec.tenBit && !rp.tenBitActive -> Toast.makeText(this, R.string.codec_no_10bit_gl, Toast.LENGTH_LONG).show()
            }
            if (prefs.getBoolean(PREF_TELEMETRY, true)) startTelemetry(rec.baseName ?: "OTC", p)
            telemetry?.line("codec")?.s("codec", enc.codec.name)?.b("glTenBit", rp.tenBitActive)?.i("bitrate", bitrateFor(outputWidth))?.send()
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
        for ((k, m) in pendingLensModels) applyLensModel(k, m)
        pendingLensModels.clear()
        pipeline?.detachEncoder()
        recordButton.isActivated = false
        val log = telemetry
        telemetry = null
        camera?.telemetry = null
        analysis?.telemetry = null
        pipeline?.telemetry = null
        val finish = {
            val uri = rec.stop()
            log?.close()
            runOnUiThread {
                if (uri != null) rememberVideo(uri)
                val msg = if (uri == null) getString(R.string.save_failed)
                else if (log != null) getString(R.string.saved_with_log) else getString(R.string.saved, "Movies/OpenTrack")
                Toast.makeText(this, msg, Toast.LENGTH_SHORT).show()
            }
        }
        if (blocking) finish() else thread(name = "recorder-stop") { finish() }
    }

    /**
     * Opens the recording's log and writes its header: everything needed to make sense of the
     * per-frame records (stream, lens, focal length, settings, modes).
     */
    private fun startTelemetry(baseName: String, p: CameraProfile) {
        val t = Telemetry.create(contentResolver, baseName) ?: return
        val version = runCatching { packageManager.getPackageInfo(packageName, 0).versionName }.getOrNull()
        t.line("start").s("video", "$baseName.mp4").s("app", version).s("device", "${android.os.Build.MANUFACTURER} ${android.os.Build.MODEL}")
            .s("android", android.os.Build.VERSION.RELEASE).s("profile", p.label).s("camera", p.cameraId).s("physical", p.physicalCameraId)
            .s("lens", lens?.key).n("lensFocalMm", lens?.focalMm?.toDouble(), 2).n("lensFactor", lens?.zoomFactor, 2)
            .i("srcW", p.sourceSize.width).i("srcH", p.sourceSize.height).i("fps", p.fps).i("captureFps", p.captureFps)
            .n("focalPx", p.focalPx).i("sensorOrientation", p.sensorOrientation).i("displayRotation", display?.rotation ?: -1)
            .i("outW", outputWidth).i("outH", outputHeight).b("gyro", gyro?.available == true).b("ois", p.hasOis)
            .n("rsSkewMs", pipeline?.rollingShutter?.let { it.skewNs / 1e6 }, 2)
            // Camera timestamps ("ts") are elapsedRealtime when true, else another clock; "rt" is always elapsedRealtime.
            .b("tsRealtime", p.timestampRealtime).i("rt", SystemClock.elapsedRealtimeNanos()).send()
        telemetry = t
        lastLoggedSettings = null
        logSettings()
        camera?.telemetry = t
        analysis?.telemetry = t
        pipeline?.telemetry = t
    }

    /** A "set" record whenever a user setting changed during the recording. */
    private fun logSettings() {
        val t = telemetry ?: return
        val line = t.line("set").i("shutterNs", settings.shutterNs).n("ev", settings.evThirds / 3.0, 2).b("focusInf", settings.focusInfinity)
            .n("cameraZoom", settings.zoomRatio.toDouble(), 2).b("hybrid", pipeline?.hybrid != null)
            .s("ladder", ladder()?.joinToString(",")).n("refZoom", refZoom(), 2).b("ois", settings.ois).s("stab", stabPreset.name).b("horizon", horizonLock)
            .b("autoZoom", zoomConfig.auto).n("zoomMin", zoomConfig.minZoom, 2).n("zoomMax", zoomConfig.maxZoom, 2).n("cropZoom", cropZoom, 2)
            .s("detector", detectorMode.name).s("target", targetClasses.name).s("model", detectorModel.name).b("autoLock", autoLock).s("view", previewMode.name).s("codec", codec.name).i("outLong", outputLongSide)
        // Only when something changed (the line without its timestamp is the comparison key).
        val key = line.toString()
        if (key == lastLoggedSettings) return
        lastLoggedSettings = key
        line.i("rt", SystemClock.elapsedRealtimeNanos()).send()
    }

    /** Frame rate the pipeline actually gets, for the camera report. */
    private var lastMeasuredFps = 0.0

    /**
     * The lens the picture comes from right now and the camera zoom ratio, e.g. "TELE 2.6x".
     * On a multi-camera hybrid zoom switches them: the lens is the one the camera reports
     * (else guessed from the ratio).
     */
    private fun lensNow(): String {
        val zoom = (pipeline?.frameMap?.zoom ?: 1.0) * uiRef()
        val factor = if (lens?.multi == true) {
            camera?.activePhysicalId?.let { lensFactors[it] }
                ?: if (zoom < 0.95) 0.6 else if (zoom >= (ladder()?.last() ?: 99f) - 0.05) 2.0 else 1.0
        } else lens?.zoomFactor ?: 1.0
        val kind = getString(
            when {
                factor < 0.85 -> R.string.lens_wide
                factor > 1.3 -> R.string.lens_tele
                else -> R.string.lens_main
            },
        ).uppercase(java.util.Locale.getDefault())
        val id = if (lens?.multi == true) camera?.activePhysicalId?.let { " ($it)" } ?: "" else ""
        // Lens calibration under way: pan over textured scenery (buildings, trees, clouds).
        val cal = analysis?.calibrationProgress?.takeIf { (k, _) -> !lensModels.containsKey(k) }?.let { (k, f) ->
            " · " + getString(R.string.lens_calibrating, lensNameFor(k), (f * 100).roundToInt())
        } ?: ""
        return "$kind ${"%.1f".format(java.util.Locale.ROOT, zoom)}x$id$cal"
    }

    /** Since when the frame rate is far below the stream's (ms), and whether that was said. */
    private var lowFpsSinceMs = 0L
    private var lowFpsWarned = false

    private fun showStatus(s: OverlayState) {
        lastMeasuredFps = s.fps
        val p = profile ?: return
        // A forced rate the camera does not hold (e.g. 4K60 on the multi-camera): say so once.
        val now = SystemClock.uptimeMillis()
        if (s.fps > 1 && s.fps < p.fps * 0.75) {
            if (lowFpsSinceMs == 0L) lowFpsSinceMs = now
            if (!lowFpsWarned && now - lowFpsSinceMs > 4000) {
                lowFpsWarned = true
                Toast.makeText(this, getString(R.string.low_fps, p.label, s.fps), Toast.LENGTH_LONG).show()
            }
        } else lowFpsSinceMs = 0L
        val stateText = getString(
            when (s.trackState) {
                TrackState.IDLE -> R.string.state_idle
                TrackState.TRACKING -> R.string.state_tracking
                TrackState.COASTING -> R.string.state_coasting
                TrackState.LOST -> R.string.state_lost
            },
        )
        val exposure = camera?.takeIf { it.lastExposureNs > 0 }?.let { "1/${(1e9 / it.lastExposureNs).roundToInt()} s ISO ${it.lastIso}" } ?: "-"
        status.text = lensNow() + " · " + getString(R.string.status, p.label, outputWidth, outputHeight, p.fps, s.fps, s.analysisMs, s.neuralMs, stateText, exposure)
        if (zoomConfig.auto) {
            val z = s.zoom * uiRef()
            findViewById<TextView>(R.id.cropZoomLabel).text = getString(R.string.crop_zoom, z)
            findViewById<SeekBar>(R.id.cropZoom).progress = ((z - uiRef()) / 0.05).roundToInt()
        }
    }

    private fun refreshButtons() {
        logSettings()
        findViewById<Button>(R.id.lens).text = lensLabel()
        findViewById<Button>(R.id.hybrid).apply {
            visibility = if (ladder() != null) android.view.View.VISIBLE else android.view.View.GONE
            text = if (hybridOn) getString(R.string.hybrid_on, hybridFromZoom() ?: 0.0) else getString(R.string.hybrid_off)
        }
        findViewById<Button>(R.id.model).setText(
            when (detectorModel) {
                DetectorModel.AIRCRAFT -> R.string.model_own
                DetectorModel.COCO -> if (targetClasses == TargetClasses.VEHICLES) R.string.model_coco_vehicles else R.string.model_coco
            },
        )
        findViewById<Button>(R.id.codec).text = getString(R.string.codec, codecName(codec))
        findViewById<Button>(R.id.share).visibility = if (lastVideo != null) android.view.View.VISIBLE else android.view.View.GONE
        findViewById<Button>(R.id.ois).apply {
            visibility = if (profile?.hasOis == true) android.view.View.VISIBLE else android.view.View.GONE
            setText(if (settings.ois) R.string.ois_on else R.string.ois_off)
        }
        findViewById<Button>(R.id.outputRes).text = getString(R.string.output_res, if (outputLongSide >= 2560) "1440p" else "1080p")
        findViewById<Button>(R.id.telemetry).setText(if (prefs.getBoolean(PREF_TELEMETRY, true)) R.string.telemetry_on else R.string.telemetry_off)
        findViewById<Button>(R.id.viewMode).setText(if (previewMode == PreviewMode.WIDE) R.string.view_wide else R.string.view_output)
        findViewById<Button>(R.id.autoLock).setText(if (autoLock) R.string.auto_lock_on else R.string.auto_lock_off)
        findViewById<Button>(R.id.detector).setText(
            when (detectorMode) {
                AnalysisEngine.Mode.HYBRID -> R.string.detector_hybrid
                AnalysisEngine.Mode.NEURAL_ONLY -> R.string.detector_neural
                AnalysisEngine.Mode.SKY_ONLY -> R.string.detector_sky
            },
        )
        findViewById<Button>(R.id.targetType).setText(
            when (targetClasses) {
                TargetClasses.AIRCRAFT -> R.string.target_aircraft
                TargetClasses.VEHICLES -> R.string.target_vehicles
            },
        )
        findViewById<Button>(R.id.focus).setText(if (settings.focusInfinity) R.string.focus_inf_on else R.string.focus_inf_off)
        findViewById<Button>(R.id.shutter).text = when {
            settings.shutterNs == CameraSettings.SHUTTER_SMART -> getString(R.string.shutter_smart)
            settings.shutterNs > 0 -> getString(R.string.shutter_fixed, (1e9 / settings.shutterNs).roundToInt())
            else -> getString(R.string.shutter_auto)
        }
        findViewById<Button>(R.id.autoZoom).setText(if (zoomConfig.auto) R.string.auto_zoom_on else R.string.auto_zoom_off)
        findViewById<Button>(R.id.processVideo).setText(if (processor != null) R.string.process_cancel else R.string.process_video)
        val manager = getSystemService(CameraManager::class.java)
        val vendor = findViewById<Button>(R.id.vendorUnlock)
        val hasVendorKeys = runCatching { profile?.let { VendorKeys.isAvailable(manager.getCameraCharacteristics(it.cameraId)) } ?: false }.getOrDefault(false)
        vendor.visibility = if (hasVendorKeys) android.view.View.VISIBLE else android.view.View.GONE
        vendor.setText(if (prefs.getBoolean(PREF_VENDOR, false)) R.string.vendor_unlock_on else R.string.vendor_unlock_off)
        findViewById<Button>(R.id.resolution).text = getString(R.string.resolution, profile?.label?.replace("3840x2160", "4K")?.replace("4096x2304", "4K 4096")?.replace("3648x2048", "4K 3648")?.replace("2560x1440", "1440p")?.replace("1920x1080", "1080p") ?: "-")
        findViewById<Button>(R.id.stabilizer).setText(stabPreset.label)
        findViewById<Button>(R.id.horizon).setText(if (horizonLock) R.string.horizon_on else R.string.horizon_off)
    }

    private fun hasPermission(p: String) = ContextCompat.checkSelfPermission(this, p) == PackageManager.PERMISSION_GRANTED

    private companion object {
        const val REQUEST_PERMISSIONS = 1
        const val PREF_PROFILE = "profile"
        const val PREF_FORCE_4K60 = "force4k60"
        const val PREF_VENDOR = "vendorUnlock"
        /** How long an opened camera may go without a frame before [recoverFromNoFrames]. */
        const val FRAME_TIMEOUT_MS = 4000L
        const val PREF_SHUTTER = "shutterNs"
        const val PREF_DETECTOR = "detectorMode"
        const val PREF_TARGET = "targetClasses"
        const val PREF_LENS = "lens"
        const val PREF_HYBRID = "hybridZoom"
        /** Lens calibrations: key + lens key → "center focal / stream width;distortion". */
        const val PREF_LENS_CAL = "lensCal:"
        /** Crop zoom sliders: up to this (vs the main lens). */
        const val ZOOM_MAX = 10.0
        const val PREF_COCO = "preferCoco"
        const val PREF_CODEC = "codec"
        const val PREF_LAST_VIDEO = "lastVideo"
        const val PREF_OIS = "ois"
        const val PREF_OUTPUT = "outputLongSide"
        const val PREF_TELEMETRY = "telemetry"
        /** Shutter choices: automatic, smart (follows the motion), then 1/250 … 1/2000 s. */
        val SHUTTERS = listOf(0L, CameraSettings.SHUTTER_SMART, 4_000_000L, 2_000_000L, 1_000_000L, 500_000L)
        const val FORCED_4K60 = "3840x2160@60!"
    }
}
