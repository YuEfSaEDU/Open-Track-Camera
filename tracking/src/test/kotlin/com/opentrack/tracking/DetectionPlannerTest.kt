package com.opentrack.tracking

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class DetectionPlannerTest {
    private val w = 3840
    private val h = 2160

    private fun tracking(t: Long, size: Double = 60.0) =
        TrackPrediction(TrackState.TRACKING, t, Vec2(1000.0, 500.0), Vec2.ZERO, size, size * 0.6, 0.9, lastMeasuredNs = t)

    @Test
    fun zoomsOnTheTargetBetweenFullFrames() {
        val planner = DetectionPlanner(w, h)
        val target = Vec2(3700.0, 100.0) // near a corner: the region is shifted inside the frame
        val regions = (0 until 6).map { planner.next(tracking(it * 100_000_000L), target, it * 100_000_000L) }
        assertNull(regions[0])
        assertNull(regions[3])
        for (r in listOf(regions[1], regions[2], regions[4], regions[5])) {
            assertNotNull(r)
            r!!
            assertEquals(r.width, r.height, 1e-9)
            assertTrue(r.width >= 0.15 * h)
            assertTrue(r.left >= 0 && r.top >= 0 && r.right <= w && r.bottom <= h)
            assertTrue(r.contains(target))
        }
    }

    @Test
    fun regionGrowsWhileTheTargetIsUnseen() {
        val planner = DetectionPlanner(w, h)
        val seen = planner.region(Vec2(1920.0, 1080.0), 60.0, 0.0)
        val unseen = planner.region(Vec2(1920.0, 1080.0), 60.0, 1.0)
        assertTrue(unseen.width > seen.width * 1.5)
    }

    @Test
    fun scansTheWholeFrameWithoutTarget() {
        val planner = DetectionPlanner(w, h)
        val tiles = (0 until 60).mapNotNull { planner.next(TrackPrediction.IDLE, null, it * 100_000_000L) }
        assertTrue(tiles.size >= 30)
        // Every point of the frame is inside some tile.
        for (y in listOf(1.0, 700.0, 1400.0, 2159.0)) for (x in listOf(1.0, 1000.0, 2000.0, 3000.0, 3839.0)) {
            assertTrue("($x, $y) not scanned", tiles.any { it.contains(Vec2(x, y)) })
        }
    }
}
