package com.opentrack.tracking

import kotlin.math.abs
import kotlin.math.pow
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class ExposureMeterTest {
    /** 640x360 analysis frame of a 3840x2160 source (scale 6), filled by [f] (analysis px). */
    private fun frame(f: (Int, Int) -> Int): LumaFrame {
        val w = 640
        val h = 360
        val d = ByteArray(w * h) { i -> f(i % w, i / w).coerceIn(0, 255).toByte() }
        return LumaFrame(w, h, d, 0L, 6.0)
    }

    @Test
    fun metersTheCropNotTheWholeFrame() {
        // Dark ground in the lower half, sky above; the crop sits in the sky.
        val f = frame { _, y -> if (y < 180) 200 else 40 }
        val whole = ExposureMeter.measure(f, null, null)
        assertEquals(120.0, whole.mean, 2.0)
        val crop = CropWindow(1920.0, 500.0, 1920.0, 800.0)
        val m = ExposureMeter.measure(f, crop, null)
        assertEquals(200.0, m.mean, 0.5)
        assertEquals(200.0, m.highlight, 0.5)
    }

    @Test
    fun weighsInALargeEnoughTarget() {
        // A dark aircraft (60) in a bright sky (210), 30x12 analysis px.
        val f = frame { x, y -> if (x in 300 until 330 && y in 170 until 182) 60 else 210 }
        val crop = CropWindow(1920.0, 1080.0, 1920.0, 1080.0)
        val target = Box(315.0 * 6, 176.0 * 6, 30.0 * 6, 12.0 * 6)
        val sky = ExposureMeter.measure(f, crop, null).mean
        val m = ExposureMeter.measure(f, crop, target).mean
        // Half the weight on the aircraft: halfway between sky and aircraft.
        assertEquals((sky + 60) / 2, m, 2.0)
        // A tiny target (2x2 px) barely counts.
        val tiny = ExposureMeter.measure(f, crop, Box(301.0 * 6, 171.0 * 6, 12.0, 12.0)).mean
        assertTrue("tiny $tiny vs sky $sky", abs(tiny - sky) < 0.05 * sky)
    }

    @Test
    fun highlightIsTheBrightEnd() {
        // 5% of the crop clipped (a sunlit cloud), the rest mid grey.
        val f = frame { x, _ -> if (x < 32) 255 else 110 }
        val m = ExposureMeter.measure(f, null, null)
        assertEquals(255.0, m.highlight, 0.0)
        val g = frame { x, _ -> if (x < 6) 255 else 110 } // ~1%: below the 98th percentile
        assertEquals(110.0, ExposureMeter.measure(g, null, null).highlight, 0.0)
    }

    @Test
    fun blurLimitFollowsSpeedAndCrop() {
        // 600 px/s on the sensor, 2x crop (1920 source px → 1920 output px): 1.5 px in 2.5 ms.
        assertEquals(2_500_000L, ExposureMeter.blurLimitNs(600.0, 1920.0, 1920)!!)
        // 3x crop: each source px is bigger in the output, so a shorter time.
        assertEquals(1_666_666.0, ExposureMeter.blurLimitNs(600.0, 1280.0, 1920)!!.toDouble(), 1.0)
        assertNull(ExposureMeter.blurLimitNs(0.0, 1920.0, 1920))
    }

    @Test
    fun imageSpeedOfCameraAndTarget() {
        // The camera pans at 1000 px/s (pose offset grows), a world-fixed target moves with it.
        val motion = CameraMotion { ts -> CameraPose(Vec2(ts * 1e-6, 0.0), 0.0) }
        val c = Vec2(1920.0, 1080.0)
        val ts = 1_000_000_000L
        assertEquals(1000.0, ExposureMeter.imageSpeed(motion, ts, c, c, 960.0), 1e-6)
        // A target flying along with the pan stays put on the sensor.
        val follow = ExposureMeter.imageSpeed(motion, ts, c, c, 960.0, point = { t -> Vec2(1000.0 - t * 1e-6, 500.0) })
        assertEquals(0.0, follow, 1e-6)
    }

    @Test
    fun smartShutterIsQuickToShortenSlowToLengthen() {
        val s = SmartShutter(250_000, 8_333_333)
        s.reset(2_000_000)
        s.update(500_000)
        s.update(500_000)
        assertTrue("after 2 updates ${s.ns}", s.ns < 700_000)
        repeat(3) { s.update(null) }
        assertTrue("slowly back ${s.ns}", s.ns < 1_500_000)
        repeat(100) { s.update(null) }
        assertEquals(8_333_333.0, s.ns.toDouble(), 100_000.0)
    }

    /** Toy camera as in ShutterPriorityExposureTest, but per pixel: sky and a bright cloud. */
    private fun luma(light: Double, exposureNs: Long, iso: Int): Double =
        (255 * (light * exposureNs * 1e-9 * iso).coerceIn(0.0, 1.0).pow(1 / 2.2))

    /** Stops darker than the plain brightness aim after settling, for a cloud [cloudFactor] x brighter than the mean. */
    private fun protectedStops(cloudFactor: Double, clipped: Boolean = true): Double {
        val ae = ShutterPriorityExposure(100, 6400)
        repeat(10) { ae.observeAuto(118.0) }
        ae.start(100, 2_000_000, 2_000_000)
        // Mean luma 118 needs light × exposure × ISO ≈ 0.18 (toy camera).
        val sky = 0.18 / (2e-3 * 100)
        repeat(80) {
            val mean = luma(sky, ae.exposureNs, ae.iso)
            val cloud = luma(sky * cloudFactor, ae.exposureNs, ae.iso)
            ae.update(mean, ae.iso, ae.exposureNs, highlight = if (clipped) cloud else null)
        }
        return ln2(2e-3 * 100 / (ae.exposureNs * 1e-9 * ae.iso))
    }

    @Test
    fun highlightProtectionDarkensJustEnoughAndAtMostOneStop() {
        // A cloud 8x brighter than the mean: darkened until it sits at 245 (~0.65 stops).
        val just = protectedStops(8.0)
        assertTrue("8x: $just stops", just in 0.55..0.75)
        // 16x brighter: it would need more than a stop; held at one stop.
        val capped = protectedStops(16.0)
        assertTrue("16x: $capped stops", capped in 0.85..1.05) // (within the dead band)
        // No clipping (1.5x): nothing changes.
        assertEquals(0.0, protectedStops(1.5), 0.05)
    }

    @Test
    fun preferredShutterChangesKeepTheExposure() {
        val ae = ShutterPriorityExposure(100, 6400)
        repeat(10) { ae.observeAuto(118.0) }
        // ISO 100 at 1/500 s; 1/2000 s needs ISO 400 (below the 800 noise limit).
        ae.start(100, 2_000_000, 2_000_000, longestNs = 8_333_333)
        val total = ae.iso.toDouble() * ae.exposureNs
        assertTrue(ae.setPreferred(500_000))
        assertEquals(500_000L, ae.exposureNs)
        assertEquals(total, ae.iso.toDouble() * ae.exposureNs, total * 0.01)
        ae.setPreferred(8_333_333)
        assertEquals(total, ae.iso.toDouble() * ae.exposureNs, total * 0.02)
        assertEquals(100, ae.iso)
    }

    private fun ln2(x: Double) = kotlin.math.ln(x) / kotlin.math.ln(2.0)
}
