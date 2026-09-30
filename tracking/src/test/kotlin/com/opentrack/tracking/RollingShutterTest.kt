package com.opentrack.tracking

import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class RollingShutterTest {
    private val w = 3840.0
    private val h = 2160.0
    private val center = Vec2(w / 2, h / 2)
    private val rs = RollingShutter(skewNs = 16_000_000L, readoutDir = Vec2(0.0, 1.0), extentPx = h)

    /** Panning at 0.4 rad/s with f = 2800 px (1120 px/s), plus a wobble and some roll. */
    private val motion = CameraMotion { t ->
        val s = t * 1e-9
        CameraPose(Vec2(1120.0 * s + 40 * sin(2 * Math.PI * 9 * s), -300.0 * s + 25 * cos(2 * Math.PI * 7 * s)), 0.3 * s + 0.004 * sin(2 * Math.PI * 5 * s))
    }

    /** Where a rolling shutter sensor records [world] in the frame of [t] (solved to convergence). */
    private fun rawImage(world: Vec2, t: Long): Vec2 {
        var p = motion.poseAt(t).toImage(world, center)
        repeat(20) { p = motion.poseAt(t + (rs.skewNs * ((p.y - center.y) / h)).toLong()).toImage(world, center) }
        return p
    }

    @Test
    fun worldRoundTripThroughTheRawFrame() {
        val t = 1_234_000_000L
        for (world in listOf(Vec2(100.0, 50.0), Vec2(1900.0, 1100.0), Vec2(3700.0, 2100.0), Vec2(600.0, 1900.0))) {
            val raw = rawImage(world, t)
            val back = rs.toWorld(motion, raw, t, center)
            assertTrue("toWorld $world -> $raw -> $back", (back - world).length() < 0.05)
            val again = rs.toImage(motion, world, t, center)
            assertTrue("toImage $world -> $again vs $raw", (again - raw).length() < 0.05)
        }
    }

    @Test
    fun ignoringTheShutterIsWrongByPixels() {
        // Sanity check of the test setup: the effect being corrected is real.
        val t = 1_234_000_000L
        val world = Vec2(1900.0, 2100.0)
        val raw = rawImage(world, t)
        val naive = motion.poseAt(t).toWorld(raw, center)
        assertTrue("shift ${(naive - world).length()}", (naive - world).length() > 5)
    }

    @Test
    fun rendererCorrectionsMapTheIdealFrameOntoTheRawFrame() {
        val t = 2_000_000_000L
        val n = 9
        val corr = rs.corrections(motion, t, n)
        var worst = 0.0
        for (gy in 0..8) for (gx in 0..8) {
            // A grid over the visible frame.
            val ideal = Vec2(200 + gx * 430.0, 100 + gy * 245.0)
            val world = motion.poseAt(t).toWorld(ideal, center)
            // What the output shader does: interpolate the samples at the ideal point's readout position.
            val f = rs.fraction(ideal, center).coerceIn(-0.5, 0.5)
            val x = (f + 0.5) * (n - 1)
            val k = x.toInt().coerceAtMost(n - 2)
            val a = x - k
            fun lerp(i: Int) = corr[k * 3 + i] * (1 - a) + corr[(k + 1) * 3 + i] * a
            val ang = lerp(2)
            val d = ideal - center
            val mapped = center + Vec2(cos(ang) * d.x - sin(ang) * d.y, sin(ang) * d.x + cos(ang) * d.y) + Vec2(lerp(0), lerp(1))
            worst = maxOf(worst, (mapped - rawImage(world, t)).length())
        }
        // The shader picks the row from the ideal position (one fixed-point step): < 0.5 px at 4K.
        assertTrue("worst error $worst px", worst < 0.5)
    }

    @Test
    fun readoutDirectionFollowsSensorAndDisplayRotation() {
        // Typical back camera (sensor mounted at 90°), landscape UI: rows run down the image.
        assertEquals(Vec2(0.0, 1.0), RollingShutter.readoutDirection(90, 1))
        // Reverse landscape: the image is upside down relative to the sensor.
        assertEquals(Vec2(0.0, -1.0), RollingShutter.readoutDirection(90, 3))
        // Portrait: the readout runs across the image.
        assertEquals(1.0, abs(RollingShutter.readoutDirection(90, 0).x), 0.0)
        assertEquals(Vec2(0.0, -1.0), RollingShutter.readoutDirection(270, 1))
    }
}
