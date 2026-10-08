package com.opentrack.tracking

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs
import kotlin.math.ln

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

    @Test
    fun zoomStartsAndStopsGentlyWithoutOvershoot() {
        val f = FramingController(3840, 2160, 16.0 / 9, CameraMotion.NONE)
        f.zoom = 2.0
        val cfg = ZoomConfig(auto = true, minZoom = 1.0, maxZoom = 6.0, targetFill = 0.25)
        f.zoomConfig = cfg
        // Steady at 2x (target 480 px), then the aircraft is suddenly half as big: 4x wanted.
        var t = run(f, 2.0, { t -> tracked(t, 480.0) })
        val zs = ArrayList<Double>()
        repeat(10 * 60) {
            f.update(t, tracked(t, 240.0))
            zs += ln(f.zoom)
            t += frameNs
        }
        val dt = frameNs * 1e-9
        val v = zs.zipWithNext { a, b -> (b - a) / dt }
        val acc = v.zipWithNext { a, b -> abs(b - a) / dt }
        // Gentle: the speed ramps up and down (acceleration limited), never jumps.
        assertTrue("max accel ${acc.max()}", acc.max() <= cfg.maxZoomOutAccel + 1e-6)
        // (zs[0] already moved one frame: the first difference is two frames of acceleration)
        assertTrue("first step ${v.first()}", v.first() <= 2 * cfg.maxZoomInAccel * dt * 1.01)
        assertTrue("max speed ${v.max()}", v.max() <= cfg.maxZoomInRate + 1e-9)
        // Monotonic, no overshoot, arrives.
        assertTrue(v.all { it >= -1e-12 })
        assertTrue("overshoot ${zs.max()}", zs.max() <= ln(4.0) + 1e-9)
        assertEquals(4.0, f.zoom, 0.5)
    }

    @Test
    fun manualZoomGlidesInsteadOfJumping() {
        val f = FramingController(3840, 2160, 16.0 / 9, CameraMotion.NONE)
        f.zoom = 2.0
        var t = run(f, 0.5, { t -> TrackPrediction.IDLE.copy(timestampNs = t) })
        f.zoom = 4.0
        val widths = ArrayList<Double>()
        repeat(150) {
            widths += f.update(t, TrackPrediction.IDLE.copy(timestampNs = t)).width
            t += frameNs
        }
        // The first frame after the slider moved barely changes; 2.5 seconds later it is there.
        assertTrue("first ${widths.first()}", widths.first() > 1920.0 * 0.99)
        assertEquals(960.0, widths.last(), 5.0)
        assertTrue(widths.zipWithNext().all { (a, b) -> b <= a + 1e-9 })
    }
}
