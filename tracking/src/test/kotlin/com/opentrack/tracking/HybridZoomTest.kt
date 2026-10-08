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
    fun zoomsTheMainLensGraduallyThenMovesToTheTelephoto() {
        val z = HybridZoom(w, h, tele = 3.0, maxRatio = 6.0)
        var t = 0L
        fun step(zoom: Double, n: Int = 1): Double { var r = 0.0; repeat(n) { t += 100_000_000L; r = z.update(t, crop(zoom)) }; return r }
        // Wide crop: no camera zoom.
        assertEquals(1.0, step(1.2), 0.0)
        // The camera zoom follows the crop on the main lens: crop / headroom, in 1% steps...
        assertEquals(2.0 / 1.35, step(2.0), 0.01)
        assertEquals(3.0 / 1.35, step(3.0), 0.02)
        assertTrue(!z.onTele)
        // ...held below the telephoto's ratio (no hovering at the HAL's switch point)...
        assertEquals(2.7, step(3.9, 5), 0.03)
        assertTrue(!z.onTele)
        // ...until the crop fits the telephoto with room (4.05x): then the telephoto, after a delay.
        step(4.5, 5)
        assertTrue(z.onTele)
        assertEquals(4.5 / 1.35, z.ratio, 0.04)
        // On the telephoto the ratio never drops below its own.
        assertEquals(3.0, step(3.6, 5), 0.01)
        // Out of its view (exit room 1.1: below 3.3x) → main lens at once.
        assertEquals(3.2 / 1.35, step(3.2), 0.03)
        assertTrue(!z.onTele)
    }

    @Test
    fun anOffCenterCropNeedsLessCameraZoom() {
        val z = HybridZoom(w, h, tele = 3.0, maxRatio = 6.0)
        // The same 5x crop near the frame's edge: a centered camera crop must still contain it.
        val centered = z.coverRatio(crop(5.0), 1.35)
        val offCenter = z.coverRatio(crop(5.0, cx = 3000.0), 1.35)
        assertEquals(5.0 / 1.35, centered, 1e-9)
        assertTrue("$offCenter", offCenter < 2.0)
        var t = 0L
        repeat(30) { t += 100_000_000L; z.update(t, crop(5.0, cx = 3000.0)) }
        assertTrue(!z.onTele)
    }

    @Test
    fun aQuickFlipBackToTheTelephotoIsHeldOff() {
        val z = HybridZoom(w, h, tele = 3.0, maxRatio = 6.0)
        var t = 0L
        fun step(c: CropWindow) { t += 100_000_000L; z.update(t, c) }
        repeat(5) { step(crop(5.0)) }
        assertTrue(z.onTele)
        step(crop(5.0, cx = 3000.0))
        assertTrue(!z.onTele)
        repeat(5) { step(crop(5.0)) }
        assertTrue("held off", !z.onTele)
        repeat(10) { step(crop(5.0)) }
        assertTrue(z.onTele)
    }

    @Test
    fun rescaledTemplateStillMatchesTheZoomedImage() {
        // A blob tracked in a 640 px image of the whole frame, then in a 2x window of it.
        // (a 2x window centered on the frame: p' = c + 2 (p - c), the shapes twice as large)
        val full = SyntheticSky.frame(640, 360, scale = 6.0, objects = listOf(Vec2(330.0, 170.0) to 4.0, Vec2(338.0, 170.0) to 2.0))
        val tt = TemplateTracker()
        tt.init(full, Box(332.0, 170.0, 14.0, 9.0))
        val zoomed = SyntheticSky.frame(640, 360, 1L, scale = 3.0, objects = listOf(Vec2(340.0, 160.0) to 8.0, Vec2(356.0, 160.0) to 4.0), seed = 2)
        tt.rescale(2.0)
        val m = tt.track(zoomed, Vec2(342.0, 162.0), 10)!!
        assertEquals(344.0, m.center.x, 1.0)
        assertEquals(160.0, m.center.y, 1.0)
        assertTrue("score ${m.score}", m.score > 0.8)
    }

    @Test
    fun lumaFrameWindowMapsToTheSource() {
        val f = LumaFrame(640, 360, ByteArray(640 * 360), 0L, 2.0, originX = 1280.0, originY = 720.0, sourceWidth = 3840.0, sourceHeight = 2160.0)
        assertTrue(f.windowed)
        val p = f.toSource(Vec2(320.0, 180.0))
        assertEquals(1920.0, p.x, 1e-9)
        assertEquals(1080.0, p.y, 1e-9)
        assertEquals(320.0, f.toAnalysis(p).x, 1e-9)
        assertEquals(1280.0, f.window.width, 1e-9)
    }

    @Test
    fun trackerKeepsTheLockWhenTheAnalysisImageSwitchesToATelephotoWindow() {
        // 1280x720 source; after 1 s the analysis image becomes a 2x window of it (a telephoto
        // analysed at its own resolution): twice the detail, the aircraft twice as large.
        val tracker = TargetTracker(CameraMotion.NONE)
        val start = Vec2(500.0, 400.0)
        val vel = Vec2(100.0, -30.0)
        val ns = 16_666_667L
        var last = TrackPrediction.IDLE
        for (i in 0 until 120) {
            val t = i * ns
            val p = start + vel * (t * 1e-9)
            val frame = if (i < 60) {
                SyntheticSky.frame(timestampNs = t, scale = 4.0, objects = listOf(p / 4.0 to 2.0), seed = i)
            } else {
                val a = (p - Vec2(320.0, 180.0)) / 2.0
                val f = SyntheticSky.frame(timestampNs = t, scale = 2.0, objects = listOf(a to 4.0), seed = i)
                LumaFrame(f.width, f.height, f.data, t, 2.0, originX = 320.0, originY = 180.0, sourceWidth = 1280.0, sourceHeight = 720.0)
            }
            if (i == 1) tracker.selectAt(p)
            last = tracker.process(frame, emptyList())
            if (i > 2) {
                assertEquals("frame $i", TrackState.TRACKING, last.state)
                assertEquals("frame $i x", p.x, last.world.x, 4.0)
                assertEquals("frame $i y", p.y, last.world.y, 4.0)
            }
        }
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
