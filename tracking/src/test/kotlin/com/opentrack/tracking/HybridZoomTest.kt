package com.opentrack.tracking

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class HybridZoomTest {
    private val w = 3840
    private val h = 2160

    @Test
    fun zoomMapRoundTripsAndShowsTheMiddle() {
        val m = ZoomMap(3.0, w, h)
        val p = Vec2(2500.0, 900.0)
        val s = m.toStream(p)
        assertEquals(1920 + (2500 - 1920) * 3.0, s.x, 1e-9)
        assertEquals(p.x, m.toRef(s).x, 1e-9)
        assertEquals(p.y, m.toRef(s).y, 1e-9)
        // The telephoto frame shows the middle third of the main lens's view.
        assertEquals(1280.0, m.visible.width, 1e-9)
        assertEquals(720.0, m.visible.height, 1e-9)
        // The stream's corners are the visible window's corners.
        val corner = m.toRef(Vec2(0.0, 0.0))
        assertEquals(m.visible.left, corner.x, 1e-9)
        assertEquals(m.visible.top, corner.y, 1e-9)
        val b = Box(2000.0, 1000.0, 300.0, 120.0)
        val back = m.toRef(m.toStream(b))
        assertEquals(b.width, back.width, 1e-9)
        assertEquals(b.cx, back.cx, 1e-9)
    }

    @Test
    fun intersectsBoxes() {
        val a = Box.fromEdges(0.0, 0.0, 100.0, 100.0)
        val i = a.intersect(Box.fromEdges(50.0, 80.0, 200.0, 300.0))!!
        assertEquals(50.0, i.left, 1e-9)
        assertEquals(100.0, i.bottom, 1e-9)
        assertNull(a.intersect(Box.fromEdges(150.0, 0.0, 200.0, 50.0)))
    }

    private fun crop(zoom: Double, cx: Double = w / 2.0, cy: Double = h / 2.0) = CropWindow(cx, cy, w / zoom, h / zoom)

    @Test
    fun switchesToTheTelephotoOnlyWhenTheCropFitsWithRoom() {
        val z = HybridZoom(3.0, w, h)
        var t = 0L
        fun step(c: CropWindow, s: Double = 0.1): Double { t += (s * 1e9).toLong(); return z.update(t, c) }
        // 3x crop: exactly the telephoto's view, no room: stays on the main lens.
        repeat(20) { assertEquals(1.0, step(crop(3.0)), 0.0) }
        // 4.5x crop centered: fits with 1.35x room (needs > 4.05x); after the delay → telephoto.
        assertEquals(1.0, step(crop(4.5)), 0.0)
        repeat(4) { step(crop(4.5)) }
        assertEquals(3.0, z.ratio, 0.0)
        // Zooming out to 3.6x: still fits with the exit room (1.1 → needs > 3.3x): stays.
        repeat(5) { assertEquals(3.0, step(crop(3.6)), 0.0) }
        // 3.2x: no longer fits → back to the main lens at once.
        assertEquals(1.0, step(crop(3.2)), 0.0)
    }

    @Test
    fun anOffCenterCropKeepsTheMainLensAndAQuickFlipIsHeldOff() {
        val z = HybridZoom(3.0, w, h)
        var t = 0L
        fun step(c: CropWindow) { t += 100_000_000L; z.update(t, c) }
        // 5x crop but far off-center (the aircraft near the frame edge): the telephoto cannot see it.
        repeat(20) { step(crop(5.0, cx = 3000.0)) }
        assertEquals(1.0, z.ratio, 0.0)
        // Centered: switches.
        repeat(5) { step(crop(5.0)) }
        assertEquals(3.0, z.ratio, 0.0)
        // Leaves (drifts off) right away...
        step(crop(5.0, cx = 3000.0))
        assertEquals(1.0, z.ratio, 0.0)
        // ...and back only after the hold time (1 s), not on the next frames.
        repeat(5) { step(crop(5.0)) }
        assertEquals(1.0, z.ratio, 0.0)
        repeat(10) { step(crop(5.0)) }
        assertEquals(3.0, z.ratio, 0.0)
    }

    @Test
    fun framingStaysInsideTheVisibleWindow() {
        val f = FramingController(w, h, 16.0 / 9, CameraMotion.NONE)
        f.zoom = 4.0
        f.visible = ZoomMap(3.0, w, h).visible
        // A target far right of the telephoto's view: the crop stops at the window's edge.
        val target = TrackPrediction(TrackState.TRACKING, 0L, Vec2(3200.0, 1080.0), Vec2.ZERO, 40.0, 20.0, 1.0)
        var c = f.update(0L, target)
        repeat(120) { k -> c = f.update((k + 1) * 16_666_667L, target.copy(timestampNs = (k + 1) * 16_666_667L)) }
        val vis = f.visible!!
        assertTrue("crop right ${c.cx + c.width / 2} > ${vis.right}", c.cx + c.width / 2 <= vis.right + 1e-6)
        assertTrue(c.cx - c.width / 2 >= vis.left - 1e-6)
    }
}
