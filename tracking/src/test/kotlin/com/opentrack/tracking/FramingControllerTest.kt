package com.opentrack.tracking

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs
import kotlin.math.sin

class FramingControllerTest {
    @Test
    fun cropIs1080pAtNativeZoomFor4k() {
        val f = FramingController(3840, 2160, 16.0 / 9, CameraMotion.NONE)
        f.zoom = FramingController.nativeZoom(3840, 1920)
        val (w, h) = f.cropSize()
        assertEquals(1920.0, w, 1e-9)
        assertEquals(1080.0, h, 1e-9)
    }

    @Test
    fun cancelsShakeWhileStabilizing() {
        val shake = CameraMotion { t -> Vec2(40.0 * sin(t * 1e-9 * 30), 0.0) }
        val f = FramingController(3840, 2160, 16.0 / 9, shake)
        // A static world point: in the image it moves with the shake; in the crop it must not.
        val worldPoint = Vec2(1900.0, 1080.0)
        var maxJitter = 0.0
        var first: Double? = null
        for (i in 0 until 120) {
            val t = i * 16_666_667L
            val crop = f.update(t, TrackPrediction.IDLE)
            val inCrop = worldPoint.x + shake.offsetAt(t).x - crop.left
            if (first == null) first = inCrop
            maxJitter = maxOf(maxJitter, abs(inCrop - first))
        }
        assertTrue("jitter $maxJitter px", maxJitter < 4.0)
    }

    @Test
    fun keepsCropInsideFrame() {
        val f = FramingController(3840, 2160, 16.0 / 9, CameraMotion.NONE)
        val edge = TrackPrediction(TrackState.TRACKING, 0, Vec2(10.0, 10.0), Vec2.ZERO, 50.0, 50.0, 1.0)
        val crop = f.update(0, edge)
        assertEquals(960.0, crop.cx, 1e-9)
        assertEquals(540.0, crop.cy, 1e-9)
    }

    @Test
    fun followsTargetSmoothly() {
        val f = FramingController(3840, 2160, 16.0 / 9, CameraMotion.NONE)
        var crop: CropWindow? = null
        for (i in 0 until 180) {
            val t = i * 16_666_667L
            val pos = Vec2(1000.0 + 400.0 * t * 1e-9, 1000.0)
            crop = f.update(t, TrackPrediction(TrackState.TRACKING, t, pos, Vec2(400.0, 0.0), 40.0, 40.0, 1.0))
        }
        val targetX = 1000.0 + 400.0 * 179 * 16_666_667L * 1e-9
        // Lead time puts the crop slightly ahead of the target, in the direction of motion.
        assertTrue(crop!!.cx > targetX)
        assertTrue(crop.cx - targetX < 100.0)
    }
}
