package com.opentrack.tracking

import org.junit.Assert.assertEquals
import org.junit.Test

class GyroMotionModelTest {
    @Test
    fun integratesAndInterpolates() {
        val m = GyroMotionModel()
        m.focalPx = 1000.0
        m.axisMap = GyroMotionModel.forDisplayRotation(90)
        // 0.1 rad/s around device X for one second, sampled at 200 Hz.
        for (i in 0..200) m.onGyroSample(i * 5_000_000L, 0.1, 0.0)
        val end = m.offsetAt(1_000_000_000L)
        assertEquals(100.0, end.x, 1e-6)   // f * 0.1 rad
        assertEquals(0.0, end.y, 1e-9)
        val mid = m.offsetAt(502_500_000L)
        assertEquals(50.25, mid.x, 1e-6)
        // Extrapolation past the last sample uses the last rate, capped.
        val ahead = m.offsetAt(1_010_000_000L)
        assertEquals(101.0, ahead.x, 1e-6)
    }

    @Test
    fun panningRightMovesSceneLeftInLandscape() {
        // Landscape (display rotation 90): world "up" is device +X; panning right is a
        // negative rotation around it, and the scene must move towards -x in the image.
        val m = GyroMotionModel()
        m.focalPx = 1000.0
        m.axisMap = GyroMotionModel.forDisplayRotation(90)
        for (i in 0..10) m.onGyroSample(i * 10_000_000L, -0.5, 0.0)
        assert(m.offsetAt(100_000_000L).x < 0)
    }

    @Test
    fun tiltingUpMovesSceneDownInLandscape() {
        // Tilting up in landscape is a positive rotation around the screen-right axis,
        // which is device -Y, so the gyro reports negative Y rates.
        val m = GyroMotionModel()
        m.focalPx = 1000.0
        m.axisMap = GyroMotionModel.forDisplayRotation(90)
        for (i in 0..10) m.onGyroSample(i * 10_000_000L, 0.0, -0.5)
        assert(m.offsetAt(100_000_000L).y > 0)
    }
}
