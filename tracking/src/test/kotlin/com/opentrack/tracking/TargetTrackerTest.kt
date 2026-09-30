package com.opentrack.tracking

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class TargetTrackerTest {
    private val frameNs = 16_666_667L

    @Test
    fun autoAcquiresAndTracksWithCameraShake() {
        // Camera shake: sinusoidal offset known to the tracker through CameraMotion.
        val shake = CameraMotion { t -> CameraPose(Vec2(30.0 * kotlin.math.sin(t * 1e-9 * 20), 18.0 * kotlin.math.cos(t * 1e-9 * 13)), 0.0) }
        val tracker = TargetTracker(shake)
        val scale = 4.0
        val detector = SkyBlobDetector()
        val worldStart = Vec2(300.0, 400.0)
        val worldVel = Vec2(300.0, -100.0) // source px/s, stays inside the 1280x720 source frame

        var last = TrackPrediction.IDLE
        for (i in 0 until 120) {
            val t = i * frameNs
            val image = worldStart + worldVel * (t * 1e-9) + shake.poseAt(t).offset
            val frame = SyntheticSky.frame(timestampNs = t, scale = scale, objects = listOf(image / scale to 2.0), seed = i)
            val dets = if (i % 6 == 0) detector.detect(frame) else emptyList()
            last = tracker.process(frame, dets)
        }
        assertEquals(TrackState.TRACKING, last.state)
        val tEnd = 119 * frameNs
        val expected = worldStart + worldVel * (tEnd * 1e-9)
        assertEquals(expected.x, last.world.x, 6.0)
        assertEquals(expected.y, last.world.y, 6.0)
        assertEquals(worldVel.x, last.velocity.x, 60.0)
        assertEquals(worldVel.y, last.velocity.y, 60.0)
    }

    @Test
    fun coastsThenLosesTarget() {
        val tracker = TargetTracker(CameraMotion.NONE, TrackerConfig(maxCoastNs = 200_000_000L))
        val pos = Vec2(100.0, 60.0)
        tracker.process(SyntheticSky.frame(objects = listOf(pos to 2.0)), emptyList())
        tracker.selectAt(pos)
        tracker.process(SyntheticSky.frame(timestampNs = frameNs, objects = listOf(pos to 2.0)), emptyList())
        assertTrue(tracker.prediction.hasTarget)
        var state = TrackState.TRACKING
        for (i in 2..40) state = tracker.process(SyntheticSky.frame(timestampNs = i * frameNs, seed = i), emptyList()).state
        assertEquals(TrackState.LOST, state)
    }

    @Test
    fun neuralDetectorOverridesWrongSkyBlobLock() {
        // A dust speck is locked first (sky blob); the neural detector keeps seeing an aircraft elsewhere.
        val tracker = TargetTracker(CameraMotion.NONE)
        val speck = Vec2(40.0, 30.0)
        val plane = Vec2(200.0, 110.0)
        val detector = SkyBlobDetector()
        tracker.process(SyntheticSky.frame(objects = listOf(speck to 1.5)), detector.detect(SyntheticSky.frame(objects = listOf(speck to 1.5))))
        assertEquals(40.0, tracker.prediction.world.x, 3.0)
        for (i in 1..60) {
            val t = i * frameNs
            val frame = SyntheticSky.frame(timestampNs = t, objects = listOf(speck to 1.5, plane to 6.0), seed = i)
            val neural = if (i % 6 == 0) {
                listOf(Detection(Box(plane.x, plane.y, 14.0, 14.0), 0.8f, "airplane", DetectionSource.NEURAL, t))
            } else emptyList()
            tracker.process(frame, neural)
        }
        assertEquals(plane.x, tracker.prediction.world.x, 3.0)
        assertEquals(plane.y, tracker.prediction.world.y, 3.0)
    }

    @Test
    fun userSelectedTargetIsNotOverridden() {
        val tracker = TargetTracker(CameraMotion.NONE)
        val speck = Vec2(40.0, 30.0)
        val plane = Vec2(200.0, 110.0)
        tracker.process(SyntheticSky.frame(objects = listOf(speck to 1.5, plane to 6.0)), emptyList())
        tracker.selectAt(speck)
        for (i in 1..60) {
            val t = i * frameNs
            val frame = SyntheticSky.frame(timestampNs = t, objects = listOf(speck to 1.5, plane to 6.0), seed = i)
            val neural = if (i % 6 == 0) {
                listOf(Detection(Box(plane.x, plane.y, 14.0, 14.0), 0.8f, "airplane", DetectionSource.NEURAL, t))
            } else emptyList()
            tracker.process(frame, neural)
        }
        assertEquals(speck.x, tracker.prediction.world.x, 3.0)
    }
}
