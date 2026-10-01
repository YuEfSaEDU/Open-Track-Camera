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
            l = luma(light / 2, ae.exposureNs, ae.iso)
            ae.update(l, ae.iso, ae.exposureNs)
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
        assertFalse(ae.update(118.0, ae.iso, ae.exposureNs)) // inside the dead band
        // Far too dark at max ISO: stays at max, never above.
        repeat(5) { ae.update(30.0, ae.iso, ae.exposureNs) }
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
            ae.update(luma(light / 10, ae.exposureNs, ae.iso), ae.iso, ae.exposureNs)
            maxExposure = maxOf(maxExposure, ae.exposureNs)
        }
        assertEquals(2_000_000L, ae.exposureNs)
        assertEquals(125.0, ae.iso.toDouble(), 12.0) // within the dead band
        assertTrue(maxExposure <= 2_000_000L)
        assertTrue(abs(luma(light / 10, ae.exposureNs, ae.iso) - target) < target * 0.05)
        // Darker still: now ISO goes up, the time stays at 1/500 s.
        repeat(40) { ae.update(luma(light / 32, ae.exposureNs, ae.iso), ae.iso, ae.exposureNs) }
        assertEquals(2_000_000L, ae.exposureNs)
        assertEquals(400.0, ae.iso.toDouble(), 40.0) // within the dead band (±4% luma ≈ ±9% light)
    }

    /** Camera whose settings take effect [latency] measurements late (request queue + analysis). */
    private fun runWithLatency(ae: ShutterPriorityExposure, light: (Int) -> Double, steps: Int, latency: Int): List<Pair<Long, Int>> {
        val applied = ArrayDeque<Pair<Long, Int>>()
        repeat(latency) { applied.addLast(ae.exposureNs to ae.iso) }
        val history = ArrayList<Pair<Long, Int>>()
        for (k in 0 until steps) {
            val (e, i) = applied.removeFirst() // what this frame was really shot with
            ae.update(luma(light(k), e, i), i, e)
            applied.addLast(ae.exposureNs to ae.iso)
            history += ae.exposureNs to ae.iso
        }
        return history
    }

    @Test
    fun settingsLatencyCausesNoOvershootOrRunaway() {
        // Bright sun, then a sudden 30x drop, with the camera applying settings 3 updates late.
        val ae = ShutterPriorityExposure(100, 6400, minExposureNs = 10_000)
        repeat(30) { ae.observeAuto(luma(30.0, 250_000, 100)) }
        val target = ae.targetLuma!!
        ae.start(100, 250_000, 2_000_000)
        val hist = runWithLatency(ae, { k -> if (k < 40) 30.0 else 1.0 }, 120, latency = 3)
        // Bright phase: settles at ~1/4000 s, never runs away to the shortest exposure.
        val bright = hist.subList(0, 40)
        assertTrue("ran away to ${bright.minOf { it.first }} ns", bright.minOf { it.first } > 200_000)
        // Dark phase: back to 1/500 s and ISO ~750, without overshooting by more than 15%.
        val (e, i) = hist.last()
        assertEquals(2_000_000L, e)
        assertEquals(target, luma(1.0, e, i), target * 0.06)
        val peakIso = hist.subList(40, hist.size).maxOf { it.second }
        assertTrue("overshoot: peak ISO $peakIso vs final $i", peakIso <= i * 1.15)
    }

    @Test
    fun recoversFromABlackFrame() {
        // The reported failure: stuck at 1/8962 s ISO 100 on a black image indoors.
        val ae = ShutterPriorityExposure(100, 6400, minExposureNs = 100_000)
        repeat(30) { ae.observeAuto(110.0) }
        ae.start(100, 100_000, 2_000_000) // starts at 1/10000 s
        val indoor = 0.05
        assertTrue(luma(indoor, ae.exposureNs, ae.iso) < 10)
        val hist = runWithLatency(ae, { indoor }, 60, latency = 3)
        val (e, i) = hist.last()
        assertEquals(2_000_000L, e)
        assertEquals(110.0, luma(indoor, e, i), 110.0 * 0.06)
    }

    @Test
    fun waitsForSettledAutoExposureAndFollowsTheUsersEv() {
        val ae = ShutterPriorityExposure(100, 6400, minExposureNs = 10_000)
        assertFalse(ae.ready)
        repeat(ShutterPriorityExposure.MIN_OBSERVATIONS - 1) { ae.observeAuto(120.0) }
        assertFalse(ae.ready) // one settled frame is not enough (the reported "too dark" start)
        ae.observeAuto(120.0)
        assertTrue(ae.ready)
        // EV +1 aims for twice the light, EV -1 for half: the exposure follows.
        val light = 2.0
        ae.start(100, 10_000_000, 2_000_000)
        ae.ev = 1.0
        repeat(40) { ae.update(luma(light, ae.exposureNs, ae.iso), ae.iso, ae.exposureNs) }
        val plus = ae.iso.toDouble() * ae.exposureNs
        ae.ev = -1.0
        repeat(40) { ae.update(luma(light, ae.exposureNs, ae.iso), ae.iso, ae.exposureNs) }
        val minus = ae.iso.toDouble() * ae.exposureNs
        assertEquals(4.0, plus / minus, 0.6) // two stops apart (within the dead band)
        assertEquals(ae.aimLuma!!, luma(light, ae.exposureNs, ae.iso), ae.aimLuma!! * 0.06)
    }

    @Test
    fun defaultsToMiddleGreyWhenAutoExposureNeverSettles() {
        val ae = ShutterPriorityExposure(100, 6400)
        ae.useDefaultTarget()
        assertTrue(ae.ready)
        assertEquals(ShutterPriorityExposure.MIDDLE_GREY_LUMA, ae.targetLuma!!, 1e-9)
    }
}
