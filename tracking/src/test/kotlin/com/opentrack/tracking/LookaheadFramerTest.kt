package com.opentrack.tracking

import org.junit.Assert.assertTrue
import org.junit.Assert.assertEquals
import org.junit.Test
import kotlin.math.abs
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.random.Random

class LookaheadFramerTest {
    private val frameNs = 16_666_667L
    private val center = Vec2(1920.0, 1080.0)

    /** RMS of the high-frequency residual (minus 0.5 s centered moving average). */
    private fun hfRms(v: List<Vec2>): Double {
        val win = 15
        var sum = 0.0
        var n = 0
        for (i in win until v.size - win) {
            var mx = 0.0; var my = 0.0
            for (j in -win..win) { mx += v[i + j].x; my += v[i + j].y }
            mx /= 2 * win + 1; my /= 2 * win + 1
            sum += (v[i].x - mx) * (v[i].x - mx) + (v[i].y - my) * (v[i].y - my)
            n++
        }
        return sqrt(sum / n)
    }

    private fun run(framer: LookaheadFramer, frames: Int, track: (Long) -> TrackPrediction): List<LookaheadFramer.Output> {
        val out = ArrayList<LookaheadFramer.Output>()
        for (i in 0 until frames) {
            val t = i * frameNs
            framer.addFrame(t, 1920.0, 1080.0)
            framer.updateTrack(track(t))
            framer.emit()?.let { out += it }
        }
        out += framer.flush()
        return out
    }

    @Test
    fun cancelsKnownShakeAndRollWithoutTarget() {
        val shake = CameraMotion { t ->
            val s = t * 1e-9
            CameraPose(Vec2(30 * sin(s * 31) + 400 * s, 20 * sin(s * 23)), Math.toRadians(0.8) * sin(s * 17))
        }
        val framer = LookaheadFramer(3840, 2160, shake, StabilizerConfig(delayFrames = 20))
        val out = run(framer, 360) { t -> TrackPrediction.IDLE.copy(timestampNs = t) }
        // A static world point seen through the output: should move smoothly (pan) without shake.
        val world = Vec2(2100.0, 1000.0)
        val seen = out.map { o -> o.crop.toLocal(shake.poseAt(o.timestampNs).toImage(world, center)) }
        val jitter = hfRms(seen.subList(60, 300))
        // Input: ~21 px RMS translation shake plus ±0.8° roll.
        assertTrue("output jitter $jitter px", jitter < 2.0)
    }

    @Test
    fun lookaheadIsSmootherThanCausalOnNoisyTarget() {
        val rnd = Random(3)
        val noise = List(400) { Vec2(rnd.nextDouble(-12.0, 12.0), rnd.nextDouble(-12.0, 12.0)) }
        fun track(t: Long): TrackPrediction {
            val i = (t / frameNs).toInt()
            val s = t * 1e-9
            val pos = Vec2(1500 + 150 * s, 1000 + 80 * sin(s * 2)) + noise[i]
            return TrackPrediction(TrackState.TRACKING, t, pos, Vec2(150.0, 0.0), 60.0, 30.0, 1.0)
        }
        fun targetInOutput(delay: Int): Pair<Double, Double> {
            val f = LookaheadFramer(3840, 2160, CameraMotion.NONE, StabilizerConfig(delayFrames = delay, acquireS = 0.0))
            val out = run(f, 400) { track(it) }
            val truth = out.map { o -> val s = o.timestampNs * 1e-9; Vec2(1500 + 150 * s, 1000 + 80 * sin(s * 2)) }
            val path = out.map { it.crop.center }
            val maxOff = out.zip(truth).maxOf { (o, q) -> o.crop.toLocal(q).length() / (o.crop.width / 2) }
            return hfRms(path.subList(30, 370)) to maxOff
        }
        val (causal, _) = targetInOutput(0)
        val (ahead, maxOff) = targetInOutput(30)
        assertTrue("look-ahead $ahead should be < causal $causal / 2", ahead < causal / 2)
        assertTrue("target stayed central: $maxOff", maxOff < 0.35)
    }

    @Test
    fun keepsCropInsideSensor() {
        val f = LookaheadFramer(3840, 2160, CameraMotion.NONE, StabilizerConfig(delayFrames = 10))
        val out = run(f, 120) { t -> TrackPrediction(TrackState.TRACKING, t, Vec2(50.0, 60.0), Vec2.ZERO, 40.0, 40.0, 1.0) }
        for (o in out) {
            val e = o.crop.halfExtents()
            assertTrue(o.crop.cx - e.x >= -1e-6 && o.crop.cy - e.y >= -1e-6)
        }
    }

    @Test
    fun pointLockCentersTheMeasuredPointNotTheFilteredTrack() {
        // The filtered track lags the aircraft by 60 px; the per-frame measurement is exact.
        fun truth(t: Long) = Vec2(1600 + 200 * t * 1e-9, 1000 + 60 * sin(t * 1e-9 * 3))
        fun track(t: Long) = TrackPrediction(
            TrackState.TRACKING, t, truth(t) - Vec2(60.0, 0.0), Vec2(200.0, 0.0), 80.0, 40.0, 1.0,
            measuredWorld = truth(t),
        )
        fun maxOffset(lock: Boolean): Double {
            val f = LookaheadFramer(3840, 2160, CameraMotion.NONE, StabilizerConfig(delayFrames = 20, lockToPoint = lock, targetWeight = 20.0, acquireS = 0.0))
            val out = run(f, 240) { track(it) }
            return out.subList(30, 210).maxOf { o -> o.crop.toLocal(truth(o.timestampNs)).length() }
        }
        val loose = maxOffset(false)
        val locked = maxOffset(true)
        assertTrue("locked $locked px should be well under loose $loose px", locked < 10.0 && loose > 40.0)
    }

    @Test
    fun lockStabilizesOnThePointButCentersTheAircraft() {
        // The template sits on the aircraft's dark core, 30 px above its center.
        fun point(t: Long) = Vec2(1600 + 200 * t * 1e-9, 1000 + 60 * sin(t * 1e-9 * 3))
        val toCenter = Vec2(0.0, 30.0)
        fun track(t: Long) = TrackPrediction(
            TrackState.TRACKING, t, point(t), Vec2(200.0, 0.0), 80.0, 40.0, 1.0, measuredWorld = point(t), centerOffset = toCenter,
        )
        val f = LookaheadFramer(3840, 2160, CameraMotion.NONE, StabilizerConfig(delayFrames = 20, lockToPoint = true, targetWeight = 20.0, acquireS = 0.0))
        val out = run(f, 240) { track(it) }
        val worst = out.subList(30, 210).maxOf { o -> o.crop.toLocal(point(o.timestampNs) + toCenter).length() }
        assertTrue("aircraft center off by $worst px", worst < 10.0)
    }

    @Test
    fun glidesOntoANewLockInsteadOfJumping() {
        // Stabilization only, then a confirmed lock on an aircraft ~1000 px from the center.
        val target = Vec2(2700.0, 1450.0)
        fun maxStep(acquireS: Double): Pair<Double, Double> {
            val f = LookaheadFramer(3840, 2160, CameraMotion.NONE, StabilizerConfig(delayFrames = 20, targetWeight = 20.0, maxTargetOffset = 0.3, acquireS = acquireS))
            val out = run(f, 240) { t ->
                if (t < 60 * frameNs) TrackPrediction.IDLE.copy(timestampNs = t)
                else TrackPrediction(TrackState.TRACKING, t, target, Vec2.ZERO, 60.0, 30.0, 1.0, measuredWorld = target)
            }
            val steps = out.zipWithNext { a, b -> (b.crop.center - a.crop.center).length() }
            return steps.max() to (out.last().crop.center - target).length()
        }
        val (snap, _) = maxStep(0.0)
        val (glide, end) = maxStep(0.8)
        assertTrue("glide step $glide px vs snap $snap px", glide < snap / 2 && glide < 40.0)
        assertTrue("ended $end px from the target", end < 10.0)
    }

    @Test
    fun cropSizeStepIsEasedWithoutLagOrPixelsItDoesNotHave() {
        val f = LookaheadFramer(3840, 2160, CameraMotion.NONE, StabilizerConfig(delayFrames = 30))
        val ns = 16_666_667L
        val outs = ArrayList<LookaheadFramer.Output>()
        val stepAt = 60
        for (i in 0 until 180) {
            val t = i * ns
            val w = if (i < stepAt) 1920.0 else 960.0
            f.addFrame(t, w, w * 9 / 16, Box(1920.0, 1080.0, w * 1.25, w * 1.25 * 9 / 16))
            f.updateTrack(TrackPrediction.IDLE.copy(timestampNs = t))
            f.emit()?.let { outs += it }
        }
        val widths = outs.associate { (it.timestampNs / ns).toInt() to it.crop.width }
        // Eased: no single frame jumps by the whole step; it starts before the step (zero lag)
        // but never wider than the frame's capture window (1.25 x its own crop).
        val steps = (1 until 140).mapNotNull { k -> widths[k]?.let { a -> widths[k - 1]?.let { b -> abs(a - b) } } }
        assertTrue("largest step ${steps.max()}", steps.max() < 960.0 * 0.2)
        assertTrue("eases in before the step: ${widths[stepAt - 2]}", widths[stepAt - 2]!! < 1920.0)
        for ((k, w) in widths) assertTrue("frame $k width $w", w <= (if (k < stepAt) 1920.0 else 960.0) * 1.25 + 1e-6)
        assertEquals(960.0, widths[stepAt + 40]!!, 1e-6)
    }
}
