package com.opentrack.tracking

import kotlin.math.abs
import kotlin.math.pow
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class ShutterPriorityExposureTest {
    /** Toy camera: luma from light × exposure × ISO, gamma encoded and clipped. */
    private fun luma(light: Double, exposureNs: Long, iso: Int): Double =
        (255 * (light * exposureNs * 1e-9 * iso).coerceIn(0.0, 1.0).pow(1 / 2.2))

    @Test
    fun keepsTheAutoExposureBrightnessAtAShortShutter() {
        val ae = ShutterPriorityExposure(50, 6400)
        val light = 0.5
        // Auto exposure ran at 1/60 s, ISO 100.
        repeat(30) { ae.observeAuto(luma(light, 16_666_667, 100)) }
        val target = ae.targetLuma!!
        // Switch to 1/1000 s: ISO must rise ~16.7x to keep the brightness.
        ae.start(100, 16_666_667, 1_000_000)
        assertEquals(1666.0, ae.iso.toDouble(), 2.0)
        var l = luma(light, 1_000_000, ae.iso)
        assertEquals(target, l, 2.0)
        // The light halves (a cloud): ISO follows within a couple of seconds at 10 Hz, without overshoot.
        var maxIso = 0
        repeat(30) {
            l = luma(light / 2, 1_000_000, ae.iso)
            ae.update(l)
            maxIso = maxOf(maxIso, ae.iso)
        }
        assertTrue("brightness ${luma(light / 2, 1_000_000, ae.iso)} vs $target", abs(luma(light / 2, 1_000_000, ae.iso) - target) < target * 0.05)
        assertTrue("overshoot $maxIso", maxIso <= ae.iso * 1.06)
    }

    @Test
    fun ignoresSmallChangesAndStaysInRange() {
        val ae = ShutterPriorityExposure(100, 3200)
        repeat(10) { ae.observeAuto(120.0) }
        ae.start(800, 10_000_000, 1_000_000) // would need ISO 8000: clamped
        assertEquals(3200, ae.iso)
        assertNull(ae.update(118.0)) // inside the dead band
        // Far too dark at max ISO: stays at max, never above.
        repeat(5) { ae.update(30.0) }
        assertEquals(3200, ae.iso)
    }
}
