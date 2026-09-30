package com.opentrack.camera.camera

import android.graphics.SurfaceTexture
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CameraMetadata
import android.util.Range
import android.util.Size

/** Everything the pipeline needs to know about the chosen camera + stream configuration. */
data class CameraProfile(
    val cameraId: String,
    /** Camera stream size in sensor (landscape) orientation. */
    val sourceSize: Size,
    val fps: Int,
    val fpsRange: Range<Int>,
    val sensorOrientation: Int,
    /** Focal length in source-stream pixels at zoom ratio 1. */
    val focalPx: Double,
    val zoomRange: Range<Float>,
    val supportsManualFocus: Boolean,
    val supportsManualSensor: Boolean,
    val exposureRangeNs: Range<Long>?,
    val isoRange: Range<Int>?,
    val aeCompensationRange: Range<Int>,
    val timestampRealtime: Boolean,
    val hasOis: Boolean,
) {
    val label: String get() = "${sourceSize.width}x${sourceSize.height}@$fps"

    companion object {
        private data class Candidate(val size: Size, val fps: Int)

        /** In order of preference. The 4K60 → 1080p60 crop is the whole point of the app. */
        private val WANTED = listOf(
            Candidate(Size(3840, 2160), 60),
            Candidate(Size(3840, 2160), 30),
            Candidate(Size(2560, 1440), 60),
            Candidate(Size(1920, 1080), 60),
            Candidate(Size(1920, 1080), 30),
        )

        fun select(manager: CameraManager): CameraProfile? {
            val backIds = manager.cameraIdList.filter {
                manager.getCameraCharacteristics(it).get(CameraCharacteristics.LENS_FACING) == CameraMetadata.LENS_FACING_BACK
            }
            for (wanted in WANTED) {
                for (id in backIds) {
                    build(id, manager.getCameraCharacteristics(id), wanted.size, wanted.fps)?.let { return it }
                }
            }
            return null
        }

        private fun build(id: String, c: CameraCharacteristics, size: Size, fps: Int): CameraProfile? {
            val map = c.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP) ?: return null
            val sizes = map.getOutputSizes(SurfaceTexture::class.java) ?: return null
            if (sizes.none { it == size }) return null
            val minFrameNs = map.getOutputMinFrameDuration(SurfaceTexture::class.java, size)
            if (minFrameNs > 1_000_000_000L / fps + 200_000L) return null
            val ranges = c.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES) ?: return null
            // Prefer a fixed range (no frame rate drop in low light), else anything reaching fps.
            val range = ranges.firstOrNull { it.lower == fps && it.upper == fps }
                ?: ranges.filter { it.upper == fps }.maxByOrNull { it.lower }
                ?: return null

            val capabilities = c.get(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES)?.toSet().orEmpty()
            val focal = c.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)?.firstOrNull() ?: 4.5f
            val physical = c.get(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE)
            val pixelArray = c.get(CameraCharacteristics.SENSOR_INFO_PIXEL_ARRAY_SIZE)
            val active = c.get(CameraCharacteristics.SENSOR_INFO_ACTIVE_ARRAY_SIZE)
            val focalPx = if (physical != null && pixelArray != null && active != null) {
                // f in pixel-array pixels, then scaled to the stream (which spans the active array width).
                val fArray = focal / physical.width * pixelArray.width
                fArray * size.width / active.width()
            } else {
                size.width * 0.8 // ~ 64° horizontal FOV fallback
            }
            val ois = c.get(CameraCharacteristics.LENS_INFO_AVAILABLE_OPTICAL_STABILIZATION)
                ?.contains(CameraMetadata.LENS_OPTICAL_STABILIZATION_MODE_ON) == true

            return CameraProfile(
                cameraId = id,
                sourceSize = size,
                fps = fps,
                fpsRange = range,
                sensorOrientation = c.get(CameraCharacteristics.SENSOR_ORIENTATION) ?: 90,
                focalPx = focalPx.toDouble(),
                zoomRange = c.get(CameraCharacteristics.CONTROL_ZOOM_RATIO_RANGE) ?: Range(1f, 1f),
                supportsManualFocus = (c.get(CameraCharacteristics.LENS_INFO_MINIMUM_FOCUS_DISTANCE) ?: 0f) > 0f,
                supportsManualSensor = CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_MANUAL_SENSOR in capabilities,
                exposureRangeNs = c.get(CameraCharacteristics.SENSOR_INFO_EXPOSURE_TIME_RANGE),
                isoRange = c.get(CameraCharacteristics.SENSOR_INFO_SENSITIVITY_RANGE),
                aeCompensationRange = c.get(CameraCharacteristics.CONTROL_AE_COMPENSATION_RANGE) ?: Range(0, 0),
                timestampRealtime = c.get(CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE) ==
                    CameraMetadata.SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME,
                hasOis = ois,
            )
        }
    }
}
