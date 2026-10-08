package com.opentrack.tracking

import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin
import kotlin.math.sqrt
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class PerspectiveViewTest {
    private val w = 3840
    private val h = 2160
    private val f = 2700.0 // ~71° horizontal field of view: a typical phone main camera

    private fun norm(v: DoubleArray): DoubleArray {
        val n = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
        return doubleArrayOf(v[0] / n, v[1] / n, v[2] / n)
    }

    private fun dot(a: DoubleArray, b: DoubleArray) = a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

    @Test
    fun equalsTheFlatCropAtTheCenter() {
        val v = PerspectiveView(w, h, f)
        val crop = CropWindow(w / 2.0, h / 2.0, 1920.0, 1080.0, 0.03)
        val hm = v.homography(crop)
        val flat = PerspectiveView(w, h, 0.0).homography(crop)
        for (u in listOf(Vec2(0.0, 0.0), Vec2(960.0, 540.0), Vec2(-960.0, 300.0))) {
            val a = PerspectiveView.map(hm, u)!!
            val b = PerspectiveView.map(flat, u)!!
            assertEquals(b.x, a.x, 1e-6)
            assertEquals(b.y, a.y, 1e-6)
        }
    }

    @Test
    fun offAxisTargetLooksAsIfTheCameraPointedAtIt() {
        // The phone turned ~18° away from the locked target: it sits off-center on the sensor.
        val t = norm(doubleArrayOf(sin(0.31) * cos(0.12), sin(0.12), cos(0.31) * cos(0.12)))
        val center = Vec2(w / 2.0 + f * t[0] / t[2], h / 2.0 + f * t[1] / t[2])
        val crop = CropWindow(center.x, center.y, 1920.0, 1080.0)
        // Reference: a camera with the same lens looking straight at the target (no roll).
        val x0 = norm(doubleArrayOf(1 - t[0] * t[0], -t[0] * t[1], -t[0] * t[2]))
        val y0 = doubleArrayOf(t[1] * x0[2] - t[2] * x0[1], t[2] * x0[0] - t[0] * x0[2], t[0] * x0[1] - t[1] * x0[0])
        val view = PerspectiveView(w, h, f).homography(crop)
        val flat = PerspectiveView(w, h, 0.0).homography(crop)
        var worst = 0.0
        var worstFlat = 0.0
        for (gx in -4..4) for (gy in -2..2) {
            // A scene point seen at output position u by the reference camera...
            val u = Vec2(gx * 220.0, gy * 220.0)
            val ray = doubleArrayOf(x0[0] * u.x + y0[0] * u.y + t[0] * f, x0[1] * u.x + y0[1] * u.y + t[1] * f, x0[2] * u.x + y0[2] * u.y + t[2] * f)
            // ...is recorded by the real camera here.
            val truth = Vec2(w / 2.0 + f * ray[0] / ray[2], h / 2.0 + f * ray[1] / ray[2])
            val p = PerspectiveView.map(view, u)!!
            val q = PerspectiveView.map(flat, u)!!
            worst = max(worst, (p - truth).length())
            worstFlat = max(worstFlat, (q - truth).length())
        }
        assertTrue("perspective view error $worst px", worst < 0.01)
        // The flat crop is visibly distorted there (tens of px across a 1080p output).
        assertTrue("flat crop error $worstFlat px", worstFlat > 30.0)
    }

    @Test
    fun strengthBacksOffToStayInsideTheCapturedRegion() {
        val v = PerspectiveView(w, h, f)
        val crop = CropWindow(2700.0, 1400.0, 1600.0, 900.0)
        // Roomy region (the whole sensor): full correction.
        assertEquals(1.0, v.fittingStrength(crop, Box(w / 2.0, h / 2.0, w.toDouble(), h.toDouble())), 1e-9)
        // Just the flat crop plus 2%: the outer side of the corrected view does not fit.
        val tight = Box(crop.cx, crop.cy, crop.width * 1.02, crop.height * 1.02)
        val s = v.fittingStrength(crop, tight)
        assertTrue("strength $s", s > 0.0 && s < 1.0)
        val hm = v.homography(crop, s)
        for (u in listOf(Vec2(-800.0, -450.0), Vec2(800.0, -450.0), Vec2(800.0, 450.0), Vec2(-800.0, 450.0))) {
            val p = PerspectiveView.map(hm, u)!!
            assertTrue("$p outside", abs(p.x - tight.cx) <= tight.width / 2 && abs(p.y - tight.cy) <= tight.height / 2)
        }
    }
}
