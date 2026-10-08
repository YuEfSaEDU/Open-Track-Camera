package com.opentrack.tracking

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.random.Random

class LensCalibratorTest {
    private fun synthetic(truth: LensModel, prior: LensModel, outliers: Double = 0.1, zoom: Double = 1.0): LensCalibrator {
        val cal = LensCalibrator(prior)
        val rnd = Random(3)
        while (cal.size < 3000) {
            // A handheld pan between two frames: up to ~1° of tilt, a little roll.
            val wx = rnd.nextDouble(-0.015, 0.015)
            val wy = rnd.nextDouble(-0.01, 0.01)
            val roll = rnd.nextDouble(-0.003, 0.003)
            repeat(100) {
                val x0 = rnd.nextDouble(-1900.0, 1900.0)
                val y0 = rnd.nextDouble(-1060.0, 1060.0)
                val p = LensCalibrator.predict(LensCalibrator.Sample(x0, y0, 0.0, 0.0, zoom, wx, wy, roll), truth) ?: return@repeat
                val (x1, y1) = if (rnd.nextDouble() < outliers) {
                    // Something moving on its own.
                    p.x + rnd.nextDouble(-40.0, 40.0) to p.y + rnd.nextDouble(-40.0, 40.0)
                } else {
                    p.x + rnd.nextDouble(-0.4, 0.4) to p.y + rnd.nextDouble(-0.4, 0.4)
                }
                cal.add(LensCalibrator.Sample(x0, y0, x1, y1, zoom, wx, wy, roll))
            }
        }
        return cal
    }

    @Test
    fun recoversTheFocalAndDistortionOfAWideAngle() {
        // The HAL says 0.4x (focal 1114 px), the lens really has 1650 px at the center and barrel distortion.
        val r = synthetic(LensModel(1650.0, 0.6), LensModel(1114.0)).fit()!!
        assertEquals(1650.0, r.model.kappa, 1650.0 * 0.01)
        assertEquals(0.6, r.model.a, 0.05)
        assertTrue("rms ${r.rmsPx} vs prior ${r.priorRmsPx}", r.rmsPx < r.priorRmsPx / 3)
    }

    @Test
    fun aPinholeLensStaysAPinhole() {
        val r = synthetic(LensModel(2786.0, 0.0), LensModel(2786.0), zoom = 1.0).fit()!!
        assertEquals(2786.0, r.model.kappa, 2786.0 * 0.005)
        assertEquals(0.0, r.model.a, 0.03)
    }

    @Test
    fun zoomRatioScalesTheFocal() {
        // A crop of the lens at ratio 2: the frame's center focal is 2 kappa.
        val r = synthetic(LensModel(1000.0, 0.3), LensModel(900.0), zoom = 2.0).fit()!!
        assertEquals(1000.0, r.model.kappa, 10.0)
    }

    @Test
    fun rotateMatchesSmallAngleShift() {
        val u = LensCalibrator.rotate(0.0, 0.0, 0.01, -0.02, 0.0)!!
        assertEquals(kotlin.math.tan(hypot()) * 0.01 / hypot(), u.x, 1e-9)
        assertEquals(kotlin.math.tan(hypot()) * -0.02 / hypot(), u.y, 1e-9)
    }

    @Test
    fun samplesFromFramesAgreeWithTheMotionModel() {
        // The pose model moves the image by +30 px in x and rolls it a little between two frames;
        // a background point does exactly that. Its sample must fit the pinhole lens perfectly.
        val f = 1000.0
        val motion = CameraMotion { t -> if (t == 0L) CameraPose.IDENTITY else CameraPose(Vec2(30.0, -12.0), 0.004) }
        val lens = FrameLens("0", 1.0, 1.0, LensModel(f), f)
        fun frame(t: Long) = LumaFrame(200, 100, ByteArray(200 * 100), t, 10.0, lens = lens)
        val a = frame(0L)
        val b = frame(16_000_000L)
        val center = Vec2(1000.0, 500.0)
        // Near the center, where the (small-angle, shift + roll) pose model is exact enough.
        val p0 = Vec2(1050.0, 520.0)
        val p1 = motion.poseAt(b.timestampNs).toImage(motion.poseAt(a.timestampNs).toWorld(p0, center), center)
        val s = calibrationSamples(a, b, listOf(a.toAnalysis(p0) to b.toAnalysis(p1)), motion).single()
        val pred = LensCalibrator.predict(s, LensModel(f))!!
        assertEquals(s.x1, pred.x, 0.5)
        assertEquals(s.y1, pred.y, 0.5)
        // A wrong focal does not fit.
        val wrong = LensCalibrator.predict(s, LensModel(f * 1.3))!!
        assertTrue(kotlin.math.hypot(wrong.x - s.x1, wrong.y - s.y1) > 5)
    }

    private fun hypot() = kotlin.math.hypot(0.01, 0.02)
}
