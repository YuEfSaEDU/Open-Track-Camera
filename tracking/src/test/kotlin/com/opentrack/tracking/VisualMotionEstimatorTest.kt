package com.opentrack.tracking

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.cos
import kotlin.math.exp
import kotlin.math.sin
import kotlin.random.Random

class VisualMotionEstimatorTest {
    private val w = 320
    private val h = 180
    private val blobs = Random(11).let { r -> List(260) { Triple(r.nextDouble(-200.0, 520.0), r.nextDouble(-200.0, 380.0), r.nextDouble(2.0, 7.0)) to r.nextInt(-70, 70) } }

    /** Textured scene seen by a camera with the given pose (about the image center). */
    private fun render(pose: CameraPose, t: Long, scale: Double = 1.0, moving: Vec2? = null): LumaFrame {
        val c = Vec2(w / 2.0, h / 2.0)
        val data = ByteArray(w * h)
        for (y in 0 until h) for (x in 0 until w) {
            val world = pose.toWorld(Vec2(x.toDouble(), y.toDouble()), c)
            var v = 128.0
            for ((b, amp) in blobs) {
                val dx = world.x - b.first
                val dy = world.y - b.second
                val d2 = dx * dx + dy * dy
                if (d2 < 9 * b.third * b.third) v += amp * exp(-d2 / (2 * b.third * b.third))
            }
            if (moving != null && (Vec2(x.toDouble(), y.toDouble()) - moving).length() < 6) v = 20.0
            data[y * w + x] = v.toInt().coerceIn(0, 255).toByte()
        }
        return LumaFrame(w, h, data, t, scale)
    }

    @Test
    fun recoversShiftAndRoll() {
        val est = VisualMotionEstimator()
        val poses = (0..6).map { i -> CameraPose(Vec2(3.0 * i + 2 * sin(i.toDouble()), -2.0 * i), 0.004 * i * cos(i * 0.5)) }
        for ((i, p) in poses.withIndex()) est.process(render(p, i * 16_666_667L))
        val last = poses.last()
        val got = est.poseAt(6 * 16_666_667L)
        val first = poses.first()
        // The estimator's reference is the first frame: compare relative motion.
        assertEquals(last.roll - first.roll, got.roll, 0.002)
        val expected = last.offset - first.offset.rotated(last.roll - first.roll)
        assertEquals(expected.x, got.offset.x, 0.6)
        assertEquals(expected.y, got.offset.y, 0.6)
        assertTrue(est.lastStep!!.reliable)
    }

    @Test
    fun ignoresMovingTargetAndReportsSourcePixels() {
        val est = VisualMotionEstimator()
        est.process(render(CameraPose.IDENTITY, 0, scale = 6.0, moving = Vec2(100.0, 90.0)))
        // Camera pans 4 px (analysis) while a dark object moves 15 px: only the camera counts.
        val step = est.process(render(CameraPose(Vec2(4.0, 0.0), 0.0), 16_666_667L, scale = 6.0, moving = Vec2(115.0, 90.0)), Box(600.0, 540.0, 80.0, 80.0))!!
        assertEquals(24.0, step.shift.x, 3.0)
        assertEquals(0.0, step.shift.y, 3.0)
    }

    @Test
    fun flatSkyIsUnreliable() {
        val est = VisualMotionEstimator()
        est.process(SyntheticSky.frame())
        val step = est.process(SyntheticSky.frame(timestampNs = 16_666_667L, seed = 2))!!
        assertFalse(step.reliable)
    }
}
