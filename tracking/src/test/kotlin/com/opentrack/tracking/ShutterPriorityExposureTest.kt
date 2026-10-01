package com.opentrack.tracking

import kotlin.math.abs
import kotlin.math.pow
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
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
        assertFalse(ae.update(118.0)) // inside the dead band
        // Far too dark at max ISO: stays at max, never above.
        repeat(5) { ae.update(30.0) }
        assertEquals(3200, ae.iso)
    }

    @Test
    fun shortensTheShutterWhenIsoCannotGoLower() {
        // Bright sun: auto exposure chose 1/4000 s at ISO 100. Shutter priority at 1/500 s
        // would need ISO 12.5; the sensor's minimum is 100, so the exposure gets shorter.
        val ae = ShutterPriorityExposure(100, 6400, minExposureNs = 20_000)
        val light = 30.0
        repeat(30) { ae.observeAuto(luma(light, 250_000, 100)) }
        val target = ae.targetLuma!!
        ae.start(100, 250_000, 2_000_000)
        assertEquals(100, ae.iso)
        assertEquals(250_000.0, ae.exposureNs.toDouble(), 2_000.0)
        assertEquals(target, luma(light, ae.exposureNs, ae.iso), 2.0)
        // A cloud: 10x less light. The time goes back to 1/500 s first (never longer), then ISO rises.
        var maxExposure = 0L
        repeat(40) {
            ae.update(luma(light / 10, ae.exposureNs, ae.iso))
            maxExposure = maxOf(maxExposure, ae.exposureNs)
        }
        assertEquals(2_000_000L, ae.exposureNs)
        assertEquals(125.0, ae.iso.toDouble(), 8.0)
        assertTrue(maxExposure <= 2_000_000L)
        assertTrue(abs(luma(light / 10, ae.exposureNs, ae.iso) - target) < target * 0.05)
        // Darker still: now ISO goes up, the time stays at 1/500 s.
        repeat(40) { ae.update(luma(light / 32, ae.exposureNs, ae.iso)) }
        assertEquals(2_000_000L, ae.exposureNs)
        assertEquals(400.0, ae.iso.toDouble(), 40.0) // within the dead band (±4% luma ≈ ±9% light)
    }
}
