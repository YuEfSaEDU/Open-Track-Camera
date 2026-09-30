package com.opentrack.tracking

import org.junit.Assert.assertTrue
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
            val f = LookaheadFramer(3840, 2160, CameraMotion.NONE, StabilizerConfig(delayFrames = delay))
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
}
