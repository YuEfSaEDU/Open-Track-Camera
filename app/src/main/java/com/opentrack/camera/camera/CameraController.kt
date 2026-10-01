package com.opentrack.camera.camera

import android.annotation.SuppressLint
import android.hardware.camera2.CameraCaptureSession
import android.hardware.camera2.CameraConstrainedHighSpeedCaptureSession
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CameraMetadata
import android.hardware.camera2.CaptureRequest
import android.hardware.camera2.CaptureResult
import android.hardware.camera2.TotalCaptureResult
import android.hardware.camera2.params.OutputConfiguration
import android.hardware.camera2.params.SessionConfiguration
import android.os.Build
import android.os.Handler
import android.os.HandlerThread
import android.os.SystemClock
import android.util.Log
import android.view.Surface
import com.opentrack.tracking.ShutterPriorityExposure
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executor
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.roundToInt

/**
 * Camera2 wrapper with a single output (the render pipeline's SurfaceTexture). All camera
 * work happens on a private handler thread.
 */
class CameraController(
    private val manager: CameraManager,
    val profile: CameraProfile,
    /** Experimental: send Honor's vendor keys (see [VendorKeys]). */
    private val vendorExperiment: Boolean = false,
) {

    interface Listener {
        /** Per-frame metadata useful for motion compensation (all in ns). */
        fun onFrameMetadata(timestampNs: Long, exposureNs: Long, rollingShutterSkewNs: Long) {}
        fun onError(message: String) {}
    }

    var listener: Listener? = null

    private val thread = HandlerThread("camera").apply { start() }
    private val handler = Handler(thread.looper)
    private val executor = Executor { handler.post(it) }
    private var device: CameraDevice? = null
    private var session: CameraCaptureSession? = null
    private var surface: Surface? = null
    private var settings = CameraSettings()

    // Last auto-exposure values, used to derive ISO for shutter priority.
    @Volatile private var lastAutoExposureNs = 0L
    @Volatile private var lastAutoIso = 0
    /** The latest automatic exposure frame was settled (converged or locked). */
    @Volatile private var aeSettled = false
    /** When automatic exposure was first requested (elapsed realtime ns), 0 = not yet. */
    private var autoSinceNs = 0L

    @SuppressLint("MissingPermission")
    fun open(target: Surface) {
        surface = target
        manager.openCamera(profile.cameraId, executor, object : CameraDevice.StateCallback() {
            override fun onOpened(camera: CameraDevice) {
                device = camera
                createSession(camera, target)
            }

            override fun onDisconnected(camera: CameraDevice) {
                camera.close()
                device = null
            }

            override fun onError(camera: CameraDevice, error: Int) {
                camera.close()
                device = null
                listener?.onError("camera error $error")
            }
        })
    }

    private val characteristics by lazy { manager.getCameraCharacteristics(profile.cameraId) }

    private fun createSession(camera: CameraDevice, target: Surface) {
        val output = OutputConfiguration(target).apply {
            if (!profile.highSpeed) {
                profile.physicalCameraId?.let { setPhysicalCameraId(it) }
                // Declare the stream as video recording: some HALs only offer their video sensor
                // modes (e.g. 4K60) to streams with this use case.
                if (Build.VERSION.SDK_INT >= 33 && profile.supportsStreamUseCase) {
                    streamUseCase = CameraMetadata.SCALER_AVAILABLE_STREAM_USE_CASES_VIDEO_RECORD.toLong()
                }
            }
        }
        val config = SessionConfiguration(
            if (profile.highSpeed) SessionConfiguration.SESSION_HIGH_SPEED else SessionConfiguration.SESSION_REGULAR,
            listOf(output),
            executor,
            object : CameraCaptureSession.StateCallback() {
                override fun onConfigured(s: CameraCaptureSession) {
                    session = s
                    applySettings(settings)
                }

                override fun onConfigureFailed(s: CameraCaptureSession) {
                    listener?.onError("session configuration failed for ${profile.label}")
                }
            },
        )
        // The frame rate range is a session parameter on many HALs: the sensor mode is picked when
        // the session is configured, so pass it up front, not only in the repeating request.
        runCatching {
            config.sessionParameters = camera.createCaptureRequest(CameraDevice.TEMPLATE_RECORD).apply {
                set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, profile.fpsRange)
                if (vendorExperiment) Log.i(TAG, "vendor session keys: " + VendorKeys.apply(this, characteristics))
            }.build()
        }.onFailure { Log.w(TAG, "session parameters rejected", it) }
        camera.createCaptureSession(config)
    }

    fun update(newSettings: CameraSettings) {
        handler.post { applySettings(newSettings) }
    }

    private fun applySettings(s: CameraSettings) {
        settings = s
        val cam = device ?: return
        val ses = session ?: return
        val out = surface ?: return
        val b = cam.createCaptureRequest(CameraDevice.TEMPLATE_RECORD)
        b.addTarget(out)
        b.set(CaptureRequest.CONTROL_MODE, CameraMetadata.CONTROL_MODE_AUTO)
        b.set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, profile.fpsRange)
        // Our own crop-based stabilization needs the full, unwarped sensor stream.
        b.set(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE, CameraMetadata.CONTROL_VIDEO_STABILIZATION_MODE_OFF)
        if (profile.hasOis) {
            b.set(CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE, CameraMetadata.LENS_OPTICAL_STABILIZATION_MODE_ON)
        }
        b.set(CaptureRequest.CONTROL_ZOOM_RATIO, s.zoomRatio.coerceIn(profile.zoomRange.lower, profile.zoomRange.upper))

        if (s.focusInfinity && profile.supportsManualFocus) {
            b.set(CaptureRequest.CONTROL_AF_MODE, CameraMetadata.CONTROL_AF_MODE_OFF)
            b.set(CaptureRequest.LENS_FOCUS_DISTANCE, 0f)
        } else {
            b.set(CaptureRequest.CONTROL_AF_MODE, CameraMetadata.CONTROL_AF_MODE_CONTINUOUS_VIDEO)
        }

        if (vendorExperiment) VendorKeys.apply(b, characteristics)

        val exp = profile.exposureRangeNs
        val iso = profile.isoRange
        // Shutter priority, once auto exposure has shown how bright the scene should be.
        // High-speed sessions only honor automatic exposure (and 1/120 s is short already).
        val manual = !profile.highSpeed && wantsShutterPriority(s) && exp != null && iso != null && ae != null &&
            lastAutoExposureNs > 0 && ae.ready
        ae?.ev = s.evThirds / 3.0
        if (manual && exp != null) {
            val shutter = s.shutterNs.coerceIn(exp.lower, minOf(exp.upper, 1_000_000_000L / profile.fps))
            // In the dark the time may grow to half the frame interval (1/120 s at 60 fps).
            val longest = maxOf(shutter, minOf(exp.upper, 1_000_000_000L / (2L * profile.fps)))
            if (!manualActive || shutter != manualShutterNs) ae!!.start(lastAutoIso, lastAutoExposureNs, shutter, longest)
            manualActive = true
            manualShutterNs = shutter
            b.set(CaptureRequest.CONTROL_AE_MODE, CameraMetadata.CONTROL_AE_MODE_OFF)
            // See ShutterPriorityExposure: the chosen time is preferred; shorter in bright light
            // at minimum ISO, longer in the dark once ISO reaches its noise limit.
            b.set(CaptureRequest.SENSOR_EXPOSURE_TIME, ae!!.exposureNs)
            b.set(CaptureRequest.SENSOR_SENSITIVITY, ae.iso)
            b.set(CaptureRequest.SENSOR_FRAME_DURATION, 1_000_000_000L / profile.fps)
        } else {
            manualActive = false
            b.set(CaptureRequest.CONTROL_AE_MODE, CameraMetadata.CONTROL_AE_MODE_ON)
            if (autoSinceNs == 0L) autoSinceNs = SystemClock.elapsedRealtimeNanos()
            // While auto exposure only teaches shutter priority its brightness, keep it neutral
            // (the user's EV is applied on top of what it learns); otherwise it is the EV.
            val steps = if (wantsShutterPriority(s) && !profile.highSpeed) 0 else (s.evThirds / 3.0 / profile.aeCompensationStep).roundToInt()
            b.set(CaptureRequest.CONTROL_AE_EXPOSURE_COMPENSATION, steps.coerceIn(profile.aeCompensationRange.lower, profile.aeCompensationRange.upper))
            b.set(CaptureRequest.CONTROL_AE_LOCK, s.aeLock)
        }

        try {
            if (ses is CameraConstrainedHighSpeedCaptureSession) {
                // Frames come in bursts (e.g. 4 per request at 120 fps).
                ses.setRepeatingBurstRequests(ses.createHighSpeedRequestList(b.build()), executor, captureCallback)
            } else {
                ses.setSingleRepeatingRequest(b.build(), executor, captureCallback)
            }
        } catch (e: Exception) {
            Log.e(TAG, "setRepeatingRequest failed", e)
            listener?.onError(e.message ?: "repeating request failed")
        }
    }

    private val ae = profile.isoRange?.let { ShutterPriorityExposure(it.lower, it.upper, minExposureNs = profile.exposureRangeNs?.lower ?: 1_000L) }
    private var manualActive = false
    private var manualShutterNs = 0L

    /** Actual exposure time (ns) and ISO of the latest frame, for the status line. */
    @Volatile var lastExposureNs = 0L
        private set
    @Volatile var lastIso = 0
        private set

    private fun wantsShutterPriority(s: CameraSettings) = s.shutterNs > 0 && profile.supportsManualSensor

    /**
     * Mean luma of an analysis frame (~10 Hz) taken at [timestampNs]. Teaches the auto
     * exposure brightness, or drives ISO / exposure time in shutter priority, judged against
     * the settings that frame was actually taken with.
     */
    fun onSceneLuma(mean: Double, timestampNs: Long) = handler.post {
        val e = ae ?: return@post
        if (manualActive) {
            val shot = shotSettings(timestampNs) ?: return@post
            if (e.update(mean, shot.second, shot.first)) applySettings(settings)
        } else {
            // Learn only from settled auto exposure: right after the camera opens the frames are
            // far too dark, and taking over from those made the whole recording dark.
            // (Only while its compensation is neutral, i.e. shutter priority is wanted: with the
            // shutter on auto it applies the user's EV, which would then count twice.)
            if (wantsShutterPriority(settings) && !profile.highSpeed && lastAutoExposureNs > 0) {
                if (aeSettled) e.observeAuto(mean)
                if (!e.ready && autoSinceNs > 0 && SystemClock.elapsedRealtimeNanos() - autoSinceNs > AE_SETTLE_TIMEOUT_NS) e.useDefaultTarget()
                if (e.ready) applySettings(settings)
            }
        }
    }

    // Exposure time and ISO of recent frames by sensor timestamp (capture results arrive on the
    // camera executor; read on the camera handler).
    private val shotTs = LongArray(SHOT_HISTORY)
    private val shotExposure = LongArray(SHOT_HISTORY)
    private val shotIso = IntArray(SHOT_HISTORY)
    private var shotNext = 0
    private val shotLock = Any()

    private fun shotSettings(timestampNs: Long): Pair<Long, Int>? = synchronized(shotLock) {
        // The frame itself, or failing that the closest one within 20 ms; if the clocks do not
        // match at all, the latest result (a few frames off, still far better than nothing).
        var best = -1
        var bestDt = 20_000_000L
        for (k in 0 until SHOT_HISTORY) {
            if (shotExposure[k] <= 0) continue
            val dt = abs(shotTs[k] - timestampNs)
            if (dt <= bestDt) { bestDt = dt; best = k }
        }
        if (best < 0) best = (shotNext - 1 + SHOT_HISTORY) % SHOT_HISTORY
        if (shotExposure[best] <= 0 || shotIso[best] <= 0) null else shotExposure[best] to shotIso[best]
    }

    private val captureCallback = object : CameraCaptureSession.CaptureCallback() {
        override fun onCaptureCompleted(session: CameraCaptureSession, request: CaptureRequest, result: TotalCaptureResult) {
            val ts = result.get(CaptureResult.SENSOR_TIMESTAMP) ?: return
            val exposure = result.get(CaptureResult.SENSOR_EXPOSURE_TIME) ?: 0L
            val skew = result.get(CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW) ?: 0L
            lastExposureNs = exposure
            lastIso = result.get(CaptureResult.SENSOR_SENSITIVITY) ?: lastIso
            synchronized(shotLock) {
                shotTs[shotNext] = ts
                shotExposure[shotNext] = exposure
                shotIso[shotNext] = lastIso
                shotNext = (shotNext + 1) % SHOT_HISTORY
            }
            if (result.get(CaptureResult.CONTROL_AE_MODE) != CameraMetadata.CONTROL_AE_MODE_OFF) {
                aeSettled = when (result.get(CaptureResult.CONTROL_AE_STATE)) {
                    CameraMetadata.CONTROL_AE_STATE_CONVERGED, CameraMetadata.CONTROL_AE_STATE_LOCKED, CameraMetadata.CONTROL_AE_STATE_FLASH_REQUIRED -> true
                    else -> false
                }
                lastAutoExposureNs = exposure
                lastAutoIso = result.get(CaptureResult.SENSOR_SENSITIVITY) ?: lastAutoIso
            }
            listener?.onFrameMetadata(ts, exposure, skew)
        }
    }

    /** Blocks until the device is closed, so the output surface can be released safely afterwards. */
    fun close() {
        val closed = CountDownLatch(1)
        handler.post {
            session?.close()
            session = null
            device?.close()
            device = null
            closed.countDown()
        }
        closed.await(2, TimeUnit.SECONDS)
        thread.quitSafely()
    }

    private companion object {
        const val TAG = "CameraController"

        /** Without a settled auto exposure report by then, shutter priority aims for middle grey. */
        const val AE_SETTLE_TIMEOUT_NS = 3_000_000_000L

        /** Capture results remembered for matching analysis frames (~0.5 s at 60 fps). */
        const val SHOT_HISTORY = 32
    }
}
