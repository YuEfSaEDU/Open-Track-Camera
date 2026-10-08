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
        val shake = CameraMotion { t -> CameraPose(Vec2(40.0 * sin(t * 1e-9 * 30), 0.0), 0.0) }
        val f = FramingController(3840, 2160, 16.0 / 9, shake)
        // A static world point: in the image it moves with the shake; in the crop it must not.
        val worldPoint = Vec2(1900.0, 1080.0)
        var maxJitter = 0.0
        var first: Double? = null
        for (i in 0 until 120) {
            val t = i * 16_666_667L
            val crop = f.update(t, TrackPrediction.IDLE)
            val inCrop = worldPoint.x + shake.poseAt(t).offset.x - crop.left
            if (first == null) first = inCrop
            maxJitter = maxOf(maxJitter, abs(inCrop - first))
        }
        assertTrue("jitter $maxJitter px", maxJitter < 4.0)
    }

    @Test
    fun keepsCropInsideFrame() {
        val f = FramingController(3840, 2160, 16.0 / 9, CameraMotion.NONE, FramingConfig(acquireS = 0.0))
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

    @Test
    fun cancelsRollShakeByRotatingTheCrop() {
        // Camera rolls back and forth by ±1°; a static world point off-center must stay still in the output.
        val roll = CameraMotion { t -> CameraPose(Vec2.ZERO, Math.toRadians(1.0) * sin(t * 1e-9 * 25)) }
        val f = FramingController(3840, 2160, 16.0 / 9, roll)
        val center = Vec2(1920.0, 1080.0)
        val world = Vec2(2300.0, 900.0)
        var first: Vec2? = null
        var maxJitter = 0.0
        for (i in 0 until 180) {
            val t = i * 16_666_667L
            val crop = f.update(t, TrackPrediction.IDLE)
            val local = crop.toLocal(roll.poseAt(t).toImage(world, center))
            if (first == null) first = local
            maxJitter = maxOf(maxJitter, (local - first).length())
        }
        // Uncompensated this point would move by ~2 * 440 px * sin(1°) ≈ 15 px.
        assertTrue("jitter $maxJitter px", maxJitter < 2.0)
    }

    @Test
    fun glidesOntoANewLockInsteadOfJumping() {
        // Plain stabilization, then a lock on an aircraft far from the center.
        val f = FramingController(3840, 2160, 16.0 / 9, CameraMotion.NONE)
        f.zoom = 2.0
        val target = Vec2(2700.0, 1450.0)
        var prev: CropWindow? = null
        var maxStep = 0.0
        var crop: CropWindow? = null
        for (i in 0 until 180) {
            val t = i * 16_666_667L
            val p = if (i < 30) TrackPrediction.IDLE.copy(timestampNs = t) else TrackPrediction(TrackState.TRACKING, t, target, Vec2.ZERO, 60.0, 30.0, 1.0)
            crop = f.update(t, p)
            prev?.let { maxStep = maxOf(maxStep, crop.center.minus(it.center).length()) }
            prev = crop
        }
        // ~1000 px to travel: no single frame may move more than a smooth glide would (~45 px).
        assertTrue("largest step $maxStep px", maxStep < 50.0)
        // ...and it gets there.
        assertTrue("ended ${crop!!.center}", crop.center.minus(target).length() < 30.0)
    }

    @Test
    fun tentativeLockIsNotFramed() {
        val tentative = TrackPrediction(TrackState.TRACKING, 5, Vec2(3000.0, 1500.0), Vec2.ZERO, 30.0, 30.0, 1.0, framable = false)
        val framed = tentative.forFraming()
        assertTrue(!framed.hasTarget)
        assertEquals(5L, framed.timestampNs)
        assertTrue(tentative.copy(framable = true).forFraming().hasTarget)
    }

    @Test
    fun glidesWhenTheTrackJumps() {
        // A re-seed moves the track 400 px in one frame (it had drifted onto the smoke trail).
        fun maxStep(acquireS: Double): Double {
            val f = FramingController(3840, 2160, 16.0 / 9, CameraMotion.NONE, FramingConfig(acquireS = acquireS))
            f.zoom = 2.0
            var prev: CropWindow? = null
            var maxStep = 0.0
            for (i in 0 until 180) {
                val t = i * 16_666_667L
                val pos = if (i < 90) Vec2(1700.0, 1000.0) else Vec2(2100.0, 1100.0)
                val crop = f.update(t, TrackPrediction(TrackState.TRACKING, t, pos, Vec2.ZERO, 60.0, 30.0, 1.0))
                if (i > 60) prev?.let { maxStep = maxOf(maxStep, (crop.center - it.center).length()) }
                prev = crop
            }
            return maxStep
        }
        val snap = maxStep(0.0)
        val glide = maxStep(0.8)
        assertTrue("glide step $glide px vs snap $snap px", glide < snap / 2)
    }
}
