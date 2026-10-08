package com.opentrack.camera.camera

import android.graphics.SurfaceTexture
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CameraMetadata
import android.media.MediaRecorder
import android.os.Build

/**
 * Plain-text dump of what the camera stack offers to this (third-party) app: every camera,
 * including the physical cameras hidden behind logical ones, with its large stream sizes and
 * their maximum frame rates, AE frame rate ranges, high-speed video modes and vendor keys.
 * Phone makers often keep 4K60 for their own camera app; this shows where it went missing.
 */
object CameraReport {
    fun build(manager: CameraManager, chosen: CameraProfile?): String = buildString {
        appendLine("Open Track Camera – kamera raporu")
        appendLine("${Build.MANUFACTURER} ${Build.MODEL} (${Build.DEVICE}), Android ${Build.VERSION.RELEASE} / API ${Build.VERSION.SDK_INT}")
        appendLine("Seçilen profil: ${chosen?.label ?: "-"} kamera ${chosen?.cameraId ?: "-"}${chosen?.physicalCameraId?.let { " (fiziksel $it)" } ?: ""}")
        appendLine("Kullanılabilir profiller: ${runCatching { CameraProfile.available(manager).joinToString { it.label + (it.physicalCameraId?.let { p -> " [fiz $p]" } ?: "") } }.getOrElse { it.toString() }}")
        appendLine("Arka lensler: ${runCatching { CameraProfile.lenses(manager).joinToString { l -> "${l.key} ${l.focalMm} mm %.2fx".format(l.zoomFactor) + (if (l.multi) " çoklu ${l.zoomRange} [${l.physicalIds.joinToString()}]" else "") + if (l.listed) "" else " (listede yok)" } }.getOrElse { it.toString() }}")
        appendLine()
        val seen = LinkedHashSet<String>()
        // The chosen camera first (the report may be cut off by the app it is shared to).
        val queue = ArrayDeque(listOfNotNull(chosen?.physicalCameraId, chosen?.cameraId) + manager.cameraIdList.toList())
        // Some phones only answer for ids they never list; probe the usual small numbers too.
        for (i in 0..9) if (i.toString() !in queue) queue += i.toString()
        while (queue.isNotEmpty()) {
            val id = queue.removeFirst()
            if (!seen.add(id)) continue
            val c = runCatching { manager.getCameraCharacteristics(id) }.getOrNull() ?: continue
            val listed = id in manager.cameraIdList
            appendLine("== Kamera $id${if (listed) "" else " (listede yok)"}")
            describe(c)
            val physical = c.physicalCameraIds
            if (physical.isNotEmpty()) {
                appendLine("  fiziksel kameralar: ${physical.joinToString()}")
                queue.addAll(physical)
            }
            appendLine()
        }
    }

    private fun StringBuilder.describe(c: CameraCharacteristics) {
        val facing = when (c.get(CameraCharacteristics.LENS_FACING)) {
            CameraMetadata.LENS_FACING_BACK -> "arka"
            CameraMetadata.LENS_FACING_FRONT -> "ön"
            else -> "harici"
        }
        val level = when (c.get(CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL)) {
            CameraMetadata.INFO_SUPPORTED_HARDWARE_LEVEL_LEGACY -> "LEGACY"
            CameraMetadata.INFO_SUPPORTED_HARDWARE_LEVEL_LIMITED -> "LIMITED"
            CameraMetadata.INFO_SUPPORTED_HARDWARE_LEVEL_FULL -> "FULL"
            CameraMetadata.INFO_SUPPORTED_HARDWARE_LEVEL_3 -> "LEVEL_3"
            CameraMetadata.INFO_SUPPORTED_HARDWARE_LEVEL_EXTERNAL -> "EXTERNAL"
            else -> "?"
        }
        appendLine("  $facing, seviye $level, odak ${c.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)?.joinToString()} mm, " +
            "zoom ${c.get(CameraCharacteristics.CONTROL_ZOOM_RATIO_RANGE)}, sensör yönü ${c.get(CameraCharacteristics.SENSOR_ORIENTATION)}")
        appendLine("  yetenekler: ${c.get(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES)?.joinToString { capability(it) }}")
        appendLine("  AE fps aralıkları: ${c.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES)?.joinToString()}")
        val ois = c.get(CameraCharacteristics.LENS_INFO_AVAILABLE_OPTICAL_STABILIZATION)?.contains(CameraMetadata.LENS_OPTICAL_STABILIZATION_MODE_ON) == true
        val oisData = c.get(CameraCharacteristics.STATISTICS_INFO_AVAILABLE_OIS_DATA_MODES)?.contains(CameraMetadata.STATISTICS_OIS_DATA_MODE_ON) == true
        val dist = c.get(CameraCharacteristics.DISTORTION_CORRECTION_AVAILABLE_MODES)?.joinToString {
            when (it) { CameraMetadata.DISTORTION_CORRECTION_MODE_OFF -> "OFF"; CameraMetadata.DISTORTION_CORRECTION_MODE_FAST -> "FAST"; CameraMetadata.DISTORTION_CORRECTION_MODE_HIGH_QUALITY -> "HQ"; else -> "$it" }
        }
        appendLine("  distorsiyon düzeltme: ${dist ?: "yok"}, katsayılar: ${c.get(CameraCharacteristics.LENS_DISTORTION)?.joinToString { "%.4f".format(it) } ?: "-"}, " +
            "iç parametreler: ${c.get(CameraCharacteristics.LENS_INTRINSIC_CALIBRATION)?.joinToString { "%.1f".format(it) } ?: "-"}")
        run {
            // Focal length in array px: the one used (calibration if plausible) and the mm estimate.
            val used = CameraProfile.focalArray(c)
            val focal = c.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)?.firstOrNull()
            val physical = c.get(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE)
            val pixelArray = c.get(CameraCharacteristics.SENSOR_INFO_PIXEL_ARRAY_SIZE)
            val est = if (focal != null && physical != null && pixelArray != null && physical.width > 0) focal / physical.width * pixelArray.width else null
            appendLine("  odak (dizi px): kullanılan ${used?.let { "%.1f (%s, dizi %d)".format(it.first, it.third, it.second) } ?: "-"}, mm tahmini ${est?.let { "%.1f".format(it) } ?: "-"}" +
                " (sensör ${physical?.width}x${physical?.height} mm, dizi ${pixelArray?.width}x${pixelArray?.height}, " +
                "aktif ${c.get(CameraCharacteristics.SENSOR_INFO_ACTIVE_ARRAY_SIZE)?.let { "${it.width()}x${it.height()}" }})")
        }
        appendLine("  OIS: ${if (ois) "var" else "yok"}, OIS kayması verisi: ${if (oisData) "var" else "yok"}, zaman damgası: " +
            if (c.get(CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE) == CameraMetadata.SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME) "realtime" else "bilinmiyor")
        val map = c.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP) ?: return
        for ((name, cls) in listOf("SurfaceTexture" to SurfaceTexture::class.java, "MediaRecorder" to MediaRecorder::class.java)) {
            val sizes = map.getOutputSizes(cls)?.filter { it.width >= 1920 }?.sortedByDescending { it.width * it.height } ?: continue
            appendLine("  $name: " + sizes.joinToString { s ->
                val ns = map.getOutputMinFrameDuration(cls, s)
                "${s.width}x${s.height}@" + if (ns > 0) "%.0f".format(1e9 / ns) else "?"
            })
        }
        val hs = map.highSpeedVideoSizes
        if (hs.isNotEmpty()) {
            appendLine("  yüksek hız: " + hs.joinToString { s -> "${s.width}x${s.height} ${map.getHighSpeedVideoFpsRangesFor(s).joinToString()}" })
        }
        val vendor = (c.availableCaptureRequestKeys.map { it.name } + c.keys.map { it.name })
            .filterNot { it.startsWith("android.") }.distinct().sorted()
        // Long key lists: counts only (the full lists made the report too long to share whole).
        appendLine("  oturum anahtarları: ${c.availableSessionKeys.size}, üretici anahtarları: ${vendor.size}")
    }

    private fun capability(v: Int) = when (v) {
        CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_MANUAL_SENSOR -> "MANUAL_SENSOR"
        CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_MANUAL_POST_PROCESSING -> "MANUAL_POST"
        CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_RAW -> "RAW"
        CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_BURST_CAPTURE -> "BURST"
        CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_CONSTRAINED_HIGH_SPEED_VIDEO -> "HIGH_SPEED"
        CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_LOGICAL_MULTI_CAMERA -> "LOGICAL_MULTI"
        CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_ULTRA_HIGH_RESOLUTION_SENSOR -> "ULTRA_HIGH_RES"
        CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_DYNAMIC_RANGE_TEN_BIT -> "10BIT"
        CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_STREAM_USE_CASE -> "STREAM_USE_CASE"
        else -> v.toString()
    }
}
