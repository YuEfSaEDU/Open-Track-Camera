package com.opentrack.tracking

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class AutoZoomTest {
    private val frameNs = 16_666_667L

    private fun tracked(t: Long, size: Double, state: TrackState = TrackState.TRACKING, lastMeasured: Long = t) =
        TrackPrediction(state, t, Vec2(1920.0, 1080.0), Vec2.ZERO, size, size * 0.4, 1.0, lastMeasured)

    private fun run(f: FramingController, seconds: Double, track: (Long) -> TrackPrediction, start: Long = 0L): Long {
        var t = start
        repeat((seconds * 60).toInt()) {
            f.update(t, track(t))
            t += frameNs
        }
        return t
    }

    @Test
    fun zoomsInToKeepTargetSize() {
        val f = FramingController(3840, 2160, 16.0 / 9, CameraMotion.NONE)
        f.zoom = 1.0
        f.zoomConfig = ZoomConfig(auto = true, minZoom = 1.0, maxZoom = 6.0, targetFill = 0.25)
        run(f, 8.0, { t -> tracked(t, 240.0) })
        // 0.25 * 3840 / 240 = 4x; the deadband allows ~12% error.
        assertEquals(4.0, f.zoom, 0.5)
    }

    @Test
    fun respectsUserLimits() {
        val f = FramingController(3840, 2160, 16.0 / 9, CameraMotion.NONE)
        f.zoom = 1.5
        f.zoomConfig = ZoomConfig(auto = true, minZoom = 1.5, maxZoom = 2.5)
        run(f, 10.0, { t -> tracked(t, 20.0) }) // tiny target wants ~48x
        assertEquals(2.5, f.zoom, 0.04)
        assertTrue(f.zoom <= 2.5)
        run(f, 10.0, { t -> TrackPrediction.IDLE.copy(timestampNs = t) }, start = 10_000_000_000L)
        assertEquals(1.5, f.zoom, 1e-3)
    }

    @Test
    fun widensWhileCoasting() {
        val f = FramingController(3840, 2160, 16.0 / 9, CameraMotion.NONE)
        f.zoom = 3.0
        f.zoomConfig = ZoomConfig(auto = true, minZoom = 1.0, maxZoom = 6.0, targetFill = 0.25)
        val end = run(f, 4.0, { t -> tracked(t, 320.0) }) // wants 3x: steady
        val steady = f.zoom
        run(f, 1.5, { t -> tracked(t, 320.0, TrackState.COASTING, lastMeasured = end) }, start = end)
        assertTrue("zoom ${f.zoom} should drop below $steady", f.zoom < steady * 0.8)
    }

    @Test
    fun noBreathingOnNoisySize() {
        val f = FramingController(3840, 2160, 16.0 / 9, CameraMotion.NONE)
        f.zoom = 4.0
        f.zoomConfig = ZoomConfig(auto = true, minZoom = 1.0, maxZoom = 6.0, targetFill = 0.25)
        var minZ = Double.MAX_VALUE
        var maxZ = 0.0
        run(f, 6.0, { t ->
            val noisy = 240.0 * (1 + 0.08 * kotlin.math.sin(t * 1e-9 * 17)) // ±8% size noise
            tracked(t, noisy).also { minZ = minOf(minZ, f.zoom); maxZ = maxOf(maxZ, f.zoom) }
        })
        assertTrue("zoom range $minZ..$maxZ", maxZ - minZ < 1e-9)
    }
}
