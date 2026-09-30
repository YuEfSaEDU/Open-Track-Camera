package com.opentrack.tracking

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class SkyBlobDetectorTest {
    @Test
    fun findsTinyAircraftInSky() {
        val frame = SyntheticSky.frame(objects = listOf(Vec2(200.0, 50.0) to 1.5), scale = 6.0)
        val dets = SkyBlobDetector().detect(frame)
        assertTrue("expected a detection", dets.isNotEmpty())
        val best = dets.first()
        assertEquals(1200.0, best.box.cx, 6.0) // reported in source pixels
        assertEquals(300.0, best.box.cy, 6.0)
        assertEquals(DetectionSource.SKY_BLOB, best.source)
    }

    @Test
    fun ignoresTexturedGround() {
        val frame = SyntheticSky.frame(texturedBottomRows = 60)
        val dets = SkyBlobDetector().detect(frame)
        assertTrue("textured ground must not produce blobs: $dets", dets.none { it.box.cy > 120 })
    }
}
