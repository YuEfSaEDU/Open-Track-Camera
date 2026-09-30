package com.opentrack.tracking

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Test

class TemplateTrackerTest {
    @Test
    fun followsMovingSmallObject() {
        val tracker = TemplateTracker()
        var pos = Vec2(60.0, 90.0)
        val step = Vec2(3.3, -1.2)
        tracker.init(SyntheticSky.frame(objects = listOf(pos to 2.5)), Box(pos.x, pos.y, 7.0, 7.0))
        var predicted = pos
        for (i in 1..50) {
            pos += step
            val frame = SyntheticSky.frame(objects = listOf(pos to 2.5), seed = i)
            val match = tracker.track(frame, predicted, radius = 8)
            assertNotNull(match)
            assertEquals("x at frame $i", pos.x, match!!.center.x, 0.8)
            assertEquals("y at frame $i", pos.y, match.center.y, 0.8)
            predicted = match.center + step
        }
    }
}
