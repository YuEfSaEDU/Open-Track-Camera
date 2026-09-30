package com.opentrack.camera.camera

import android.annotation.SuppressLint
import android.hardware.camera2.CameraCaptureSession
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CameraMetadata
import android.hardware.camera2.CaptureRequest
import android.hardware.camera2.CaptureResult
import android.hardware.camera2.TotalCaptureResult
import android.hardware.camera2.params.OutputConfiguration
import android.hardware.camera2.params.SessionConfiguration
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import android.view.Surface
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executor
import java.util.concurrent.TimeUnit

/**
 * Camera2 wrapper with a single output (the render pipeline's SurfaceTexture). All camera
 * work happens on a private handler thread.
 */
class CameraController(private val manager: CameraManager, val profile: CameraProfile) {

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

    private fun createSession(camera: CameraDevice, target: Surface) {
        val config = SessionConfiguration(
            SessionConfiguration.SESSION_REGULAR,
            listOf(OutputConfiguration(target)),
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

        val exp = profile.exposureRangeNs
        val iso = profile.isoRange
        if (s.shutterPriority && profile.supportsManualSensor && exp != null && iso != null && lastAutoExposureNs > 0) {
            val shutter = s.shutterNs.coerceIn(exp.lower, minOf(exp.upper, 1_000_000_000L / profile.fps))
            val derivedIso = (lastAutoIso.toDouble() * lastAutoExposureNs / shutter).toInt().coerceIn(iso.lower, iso.upper)
            b.set(CaptureRequest.CONTROL_AE_MODE, CameraMetadata.CONTROL_AE_MODE_OFF)
            b.set(CaptureRequest.SENSOR_EXPOSURE_TIME, shutter)
            b.set(CaptureRequest.SENSOR_SENSITIVITY, derivedIso)
            b.set(CaptureRequest.SENSOR_FRAME_DURATION, 1_000_000_000L / profile.fps)
        } else {
            b.set(CaptureRequest.CONTROL_AE_MODE, CameraMetadata.CONTROL_AE_MODE_ON)
            b.set(CaptureRequest.CONTROL_AE_EXPOSURE_COMPENSATION, s.aeCompensation.coerceIn(profile.aeCompensationRange.lower, profile.aeCompensationRange.upper))
            b.set(CaptureRequest.CONTROL_AE_LOCK, s.aeLock)
        }

        try {
            ses.setSingleRepeatingRequest(b.build(), executor, captureCallback)
        } catch (e: Exception) {
            Log.e(TAG, "setRepeatingRequest failed", e)
            listener?.onError(e.message ?: "repeating request failed")
        }
    }

    private val captureCallback = object : CameraCaptureSession.CaptureCallback() {
        override fun onCaptureCompleted(session: CameraCaptureSession, request: CaptureRequest, result: TotalCaptureResult) {
            val ts = result.get(CaptureResult.SENSOR_TIMESTAMP) ?: return
            val exposure = result.get(CaptureResult.SENSOR_EXPOSURE_TIME) ?: 0L
            val skew = result.get(CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW) ?: 0L
            if (result.get(CaptureResult.CONTROL_AE_MODE) != CameraMetadata.CONTROL_AE_MODE_OFF) {
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
    }
}
