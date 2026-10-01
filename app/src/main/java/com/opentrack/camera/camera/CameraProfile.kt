package com.opentrack.camera.camera

import android.graphics.SurfaceTexture
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CameraMetadata
import android.os.Build
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
    /** EV per auto exposure compensation step (often 1/3 or 1/6). */
    val aeCompensationStep: Double,
    val timestampRealtime: Boolean,
    val hasOis: Boolean,
    /**
     * Stream from this physical camera of the logical [cameraId] (multi-camera phones may
     * offer a mode on a physical lens that the logical camera does not list), or null.
     */
    val physicalCameraId: String? = null,
    /** Requested although the camera does not advertise it (experimental; may fail or run slower). */
    val forced: Boolean = false,
    /** The camera accepts stream use cases (Android 13+), so the stream can be tagged as video recording. */
    val supportsStreamUseCase: Boolean = false,
    /**
     * Constrained high-speed session (the only way some phones offer more than 30 fps to other
     * apps): the sensor runs at [captureFps] and the pipeline uses every n-th frame for [fps].
     */
    val highSpeed: Boolean = false,
    val captureFps: Int = fps,
) {
    val label: String get() = "${sourceSize.width}x${sourceSize.height}@$fps" + (if (highSpeed) " HS" else "") + if (forced) "!" else ""

    /** The lens this profile streams from, as a [Lens.key]. */
    val lensKey: String get() = Lens.key(cameraId, physicalCameraId)

    /**
     * A back lens to stream from: a camera the phone lists, one it does not list but describes
     * (makers often keep the ultra wide or telephoto for their own app; opening it may fail),
     * or a physical lens behind a logical multi-camera (streamed through [cameraId]).
     */
    data class Lens(
        val cameraId: String,
        val physicalId: String?,
        val focalMm: Float,
        /** Angle of view relative to the main lens: 0.6 for an ultra wide, 2.5 for a 2.5x telephoto. */
        val zoomFactor: Double,
        /** Listed by the phone (or reached through a listed logical camera). */
        val listed: Boolean,
    ) {
        val key: String get() = key(cameraId, physicalId)

        companion object {
            fun key(cameraId: String, physicalId: String?) = cameraId + (physicalId?.let { "/$it" } ?: "")
        }
    }

    companion object {
        private data class Candidate(val size: Size, val fps: Int, val highSpeed: Boolean = false)

        /** In order of preference. The 4K60 → 1080p60 crop is the whole point of the app. */
        private val WANTED = listOf(
            Candidate(Size(3840, 2160), 60),
            Candidate(Size(3840, 2160), 30),
            Candidate(Size(2560, 1440), 60),
            Candidate(Size(1920, 1080), 60),
            Candidate(Size(1920, 1080), 60, highSpeed = true),
            Candidate(Size(1920, 1080), 30),
        )

        /**
         * The best available profile, or the one labelled [preferred] if it is available; from
         * [lens] when given (null = the phone's default choice).
         */
        fun select(manager: CameraManager, preferred: String? = null, force4k60: Boolean = false, lens: Lens? = null): CameraProfile? {
            val all = available(manager, lens)
            if (force4k60 && all.none { it.sourceSize == FOUR_K && it.fps == 60 }) {
                if (lens != null) {
                    characteristics(manager, lens)?.let { c -> build(lens.cameraId, c, FOUR_K, 60, force = true)?.copy(physicalCameraId = lens.physicalId) }?.let { return it }
                } else {
                    backCameras(manager).firstNotNullOfOrNull { (id, c) -> build(id, c, FOUR_K, 60, force = true) }?.let { return it }
                }
            }
            return all.firstOrNull { it.label == preferred } ?: all.firstOrNull()
        }

        /**
         * Every back lens the phone describes, main lens first, then by angle of view (widest
         * first). Ids the phone does not list are probed like in the camera report.
         */
        fun lenses(manager: CameraManager): List<Lens> {
            val listedIds = manager.cameraIdList.toList()
            val found = ArrayList<Pair<Lens, Double>>() // lens, focal / sensor width
            val seen = HashSet<String>()
            fun add(cameraId: String, physicalId: String?, c: CameraCharacteristics, listed: Boolean) {
                // (A logical multi-camera counts too: it switches between its lenses with the zoom.)
                if (c.get(CameraCharacteristics.LENS_FACING) != CameraMetadata.LENS_FACING_BACK) return
                val focal = c.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)?.firstOrNull() ?: return
                val width = c.get(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE)?.width?.takeIf { it > 0 } ?: return
                found += Lens(cameraId, physicalId, focal, 1.0, listed) to (focal / width).toDouble()
            }
            val probe = listedIds + (0..9).map { it.toString() }.filter { it !in listedIds }
            for (id in probe) {
                if (!seen.add(id)) continue
                val c = runCatching { manager.getCameraCharacteristics(id) }.getOrNull() ?: continue
                val listed = id in listedIds
                add(id, null, c, listed)
                if (listed) {
                    // Physical lenses are reached through their logical camera, not probed on their own.
                    for (phys in c.physicalCameraIds) {
                        val pc = runCatching { manager.getCameraCharacteristics(phys) }.getOrNull() ?: continue
                        seen += phys
                        add(id, phys, pc, true)
                    }
                }
            }
            if (found.isEmpty()) return emptyList()
            // The main lens: the one the phone's default profile uses, else the first listed.
            val defaultKey = runCatching { available(manager).firstOrNull()?.lensKey }.getOrNull()
            val main = found.firstOrNull { it.first.key == defaultKey } ?: found.firstOrNull { it.first.listed } ?: found.first()
            val lenses = found.map { (l, f) -> l.copy(zoomFactor = f / main.second) }
            return listOf(lenses.first { it.key == main.first.key }) + lenses.filter { it.key != main.first.key }.sortedBy { it.zoomFactor }
        }

        private fun characteristics(manager: CameraManager, lens: Lens): CameraCharacteristics? =
            runCatching { manager.getCameraCharacteristics(lens.physicalId ?: lens.cameraId) }.getOrNull()

        /**
         * All wanted profiles the phone offers, best first: each back camera, and each hidden
         * physical camera behind a logical one (streamed through the logical camera); with
         * [lens], only that lens's.
         */
        fun available(manager: CameraManager, lens: Lens? = null): List<CameraProfile> {
            if (lens != null) {
                val c = characteristics(manager, lens) ?: return emptyList()
                return WANTED.mapNotNull { wanted ->
                    if (wanted.highSpeed) {
                        // High-speed sessions cannot pick a physical lens.
                        if (lens.physicalId != null) null else buildHighSpeed(lens.cameraId, c, wanted.size, wanted.fps)
                    } else {
                        build(lens.cameraId, c, wanted.size, wanted.fps)?.copy(physicalCameraId = lens.physicalId)
                    }
                    // Like below: high speed only where the regular session cannot do that rate.
                }.let { list -> list.filter { p -> !p.highSpeed || list.none { !it.highSpeed && it.sourceSize == p.sourceSize && it.fps == p.fps } } }
            }
            val cams = backCameras(manager)
            val out = ArrayList<CameraProfile>()
            for (wanted in WANTED) {
                if (wanted.highSpeed) {
                    // Only worth it when the regular session cannot do this rate.
                    if (out.none { it.sourceSize == wanted.size && it.fps == wanted.fps }) {
                        cams.firstNotNullOfOrNull { (id, c) -> buildHighSpeed(id, c, wanted.size, wanted.fps) }?.let { out += it }
                    }
                    continue
                }
                val p = cams.firstNotNullOfOrNull { (id, c) -> build(id, c, wanted.size, wanted.fps) }
                    ?: cams.firstNotNullOfOrNull { (id, c) ->
                        c.physicalCameraIds.firstNotNullOfOrNull { phys ->
                            runCatching { manager.getCameraCharacteristics(phys) }.getOrNull()
                                ?.let { build(id, it, wanted.size, wanted.fps)?.copy(physicalCameraId = phys) }
                        }
                    }
                if (p != null) out += p
            }
            return out
        }

        private val FOUR_K = Size(3840, 2160)

        private fun backCameras(manager: CameraManager): List<Pair<String, CameraCharacteristics>> =
            manager.cameraIdList.map { it to manager.getCameraCharacteristics(it) }
                .filter { it.second.get(CameraCharacteristics.LENS_FACING) == CameraMetadata.LENS_FACING_BACK }

        /**
         * [fps] from a constrained high-speed session: a fixed high-speed range that is a
         * multiple of [fps] (e.g. [120, 120] for 60), decimated by the pipeline.
         */
        private fun buildHighSpeed(id: String, c: CameraCharacteristics, size: Size, fps: Int): CameraProfile? {
            val caps = c.get(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES) ?: return null
            if (CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_CONSTRAINED_HIGH_SPEED_VIDEO !in caps) return null
            val map = c.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP) ?: return null
            if (map.highSpeedVideoSizes.none { it == size }) return null
            val range = map.getHighSpeedVideoFpsRangesFor(size)
                .filter { it.lower == it.upper && it.upper >= fps && it.upper % fps == 0 }
                .minByOrNull { it.upper } ?: return null
            return build(id, c, size, fps, force = true)?.copy(fpsRange = range, forced = false, highSpeed = true, captureFps = range.upper)
        }

        private fun build(id: String, c: CameraCharacteristics, size: Size, fps: Int, force: Boolean = false): CameraProfile? {
            val map = c.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP) ?: return null
            val sizes = map.getOutputSizes(SurfaceTexture::class.java) ?: return null
            if (sizes.none { it == size }) return null
            val minFrameNs = map.getOutputMinFrameDuration(SurfaceTexture::class.java, size)
            if (!force && minFrameNs > 1_000_000_000L / fps + 200_000L) return null
            val ranges = c.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES) ?: return null
            // Prefer a fixed range (no frame rate drop in low light), else anything reaching fps.
            val range = ranges.firstOrNull { it.lower == fps && it.upper == fps }
                ?: ranges.filter { it.upper == fps }.maxByOrNull { it.lower }
                ?: if (force) Range(fps, fps) else return null

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
                aeCompensationStep = c.get(CameraCharacteristics.CONTROL_AE_COMPENSATION_STEP)?.toDouble()?.takeIf { it > 0 } ?: (1.0 / 3),
                timestampRealtime = c.get(CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE) ==
                    CameraMetadata.SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME,
                hasOis = ois,
                forced = force,
                supportsStreamUseCase = Build.VERSION.SDK_INT >= 33 &&
                    CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_STREAM_USE_CASE in capabilities,
            )
        }
    }
}
