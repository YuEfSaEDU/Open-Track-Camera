package com.opentrack.tracking

import kotlin.math.PI
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow

data class StabilizerConfig(
    /** Frames of look-ahead: the output lags the camera by this much. 0 = causal. */
    val delayFrames: Int = 20,
    /**
     * 0 = follow closely (cut-off ~2 Hz), 1 = very smooth (~0.2 Hz). Maps to the path
     * optimizer's derivative weights.
     */
    val smoothness: Double = 0.6,
    /**
     * L1-like path (static / constant pan / eased segments) instead of plain L2 smoothing.
     * Off by default: in a sliding window the sparsity pattern can change from frame to frame.
     */
    val cinematic: Boolean = false,
    /**
     * Hard limit: the target never gets further than this fraction of the half crop from the
     * center. Centering itself is a soft goal (the target term of the optimizer), because
     * tracker positions are noisy; a tight hard limit would make the crop copy that noise.
     */
    val maxTargetOffset: Double = 0.7,
    /**
     * Lead room: frame slightly ahead of a moving target (seconds of motion, capped). Off by
     * default: it amplifies velocity noise from the tracker.
     */
    val leadTimeS: Double = 0.0,
    val maxLeadFraction: Double = 0.15,
    /** Weight of the plain stabilization goal (frame center) relative to a target (1.0). */
    val eisGoalWeight: Double = 1.0,
    /** Roll (horizon) stabilization: smoothness of the virtual roll and max crop rotation. */
    val rollStabilization: Boolean = true,
    val rollSmoothness: Double = 0.8,
    val maxRoll: Double = 0.09,
    /** Past frames in the optimization window, as a multiple of [delayFrames] (free: adds no latency). */
    val pastFactor: Double = 2.0,
    /** See [PathSmoother.Weights.constraintWeight]. */
    val constraintWeight: Double = 50.0,
    /** Huber scale of the target goal, as a fraction of the crop width (NaN = plain L2). */
    val robustGoalScale: Double = 0.04,
) {
    /** Cut-off frequency of the equivalent L2 smoother in Hz. */
    val cutoffHz: Double get() = 2.0 * 0.1.pow(smoothness.coerceIn(0.0, 1.0))

    fun pathWeights(cutoff: Double = cutoffHz): PathSmoother.Weights {
        val w = 2 * PI * cutoff
        return PathSmoother.Weights(
            velocity = 0.05 / (w * w),
            acceleration = 1.0 / (w * w * w * w),
            jerk = 0.3 / (w * w * w * w * w * w),
            l1 = cinematic,
            constraintWeight = constraintWeight,
            robustGoalScale = robustGoalScale,
        )
    }
}

/**
 * Look-ahead stabilizer / framer. Frames are registered as they are captured
 * ([addFrame]), tracker results are attached as they become available ([updateTrack]), and
 * each frame's final crop is produced [StabilizerConfig.delayFrames] frames later ([emit]),
 * when the camera path around it — past and future — is known.
 *
 * Each emission optimizes the path over a symmetric window (up to `delay` frames before and
 * after the frame) with [PathSmoother] and keeps only the center sample. The window is built
 * from the *goals* (target / frame-center positions), never from previously emitted crops, so
 * there is no feedback: the result is a zero-phase smoother of the goal path that respects
 * the crop constraints, not a receding-horizon controller (which can oscillate).
 *
 * All path math happens in world coordinates (camera motion removed), so hand shake measured
 * by the gyro or the visual estimator is cancelled exactly, and only the intentional part of
 * the motion (following the aircraft, slow pans) is shaped by the optimizer.
 *
 * Not thread safe; use from a single (render) thread.
 */
class LookaheadFramer(
    val sourceWidth: Int,
    val sourceHeight: Int,
    private val motion: CameraMotion,
    var config: StabilizerConfig = StabilizerConfig(),
) {
    data class Output(val timestampNs: Long, val crop: CropWindow, val track: TrackPrediction?)

    private class Rec(
        val ts: Long,
        val cropW: Double,
        val cropH: Double,
        /** Region of the image actually available for this frame (app's capture window), image px. */
        val capture: Box?,
    ) {
        var track: TrackPrediction? = null
    }

    private val past = ArrayDeque<Rec>()
    private val pending = ArrayDeque<Rec>()
    private val smoother = PathSmoother()
    private val center = Vec2(sourceWidth / 2.0, sourceHeight / 2.0)
    private var lastDtS = 1.0 / 30

    val pendingFrames: Int get() = pending.size

    fun reset() {
        past.clear()
        pending.clear()
    }

    /** Registers a captured frame with its crop size (from the zoom controller) and optional capture window. */
    fun addFrame(timestampNs: Long, cropWidth: Double, cropHeight: Double, capture: Box? = null) {
        pending.lastOrNull()?.let { if (timestampNs > it.ts) lastDtS = (timestampNs - it.ts) * 1e-9 }
        pending.addLast(Rec(timestampNs, cropWidth, cropHeight, capture))
    }

    /** Attaches tracker output: each frame keeps the newest prediction made at or before its time. */
    fun updateTrack(p: TrackPrediction) {
        for (r in pending) {
            val cur = r.track
            if (p.timestampNs <= r.ts && (cur == null || p.timestampNs >= cur.timestampNs)) r.track = p
        }
    }

    /** Produces the next delayed frame's crop once enough look-ahead is available. */
    fun emit(): Output? = if (pending.size > config.delayFrames) emitOne() else null

    /** Emits all remaining frames (end of recording), using whatever look-ahead is left. */
    fun flush(): List<Output> {
        val out = ArrayList<Output>()
        while (pending.isNotEmpty()) out += emitOne()
        return out
    }

    private fun emitOne(): Output {
        val c = config
        val window = ArrayList<Rec>(past.size + pending.size)
        window.addAll(past)
        window.addAll(pending)
        val cur = past.size
        val n = window.size
        val xs = DoubleArray(n)
        val ys = DoubleArray(n)
        val gx = DoubleArray(n)
        val gy = DoubleArray(n)
        val a = DoubleArray(n)
        val loX = DoubleArray(n) { Double.NEGATIVE_INFINITY }
        val hiX = DoubleArray(n) { Double.POSITIVE_INFINITY }
        val loY = DoubleArray(n) { Double.NEGATIVE_INFINITY }
        val hiY = DoubleArray(n) { Double.POSITIVE_INFINITY }
        val rolls = DoubleArray(n)
        val rollGoal = DoubleArray(n)
        val rollW = DoubleArray(n) { 1.0 }
        val rollLo = DoubleArray(n)
        val rollHi = DoubleArray(n)

        // Normalize positions by the crop width so weights are resolution independent.
        val scale = window[cur].cropW
        for ((k, r) in window.withIndex()) {
            val pose = motion.poseAt(r.ts)
            val t = r.track
            val ext = CropWindow.rotatedHalfExtents(r.cropW, r.cropH, if (c.rollStabilization) rotationLimit(r) else 0.0)
            // Where the crop center may be in the image: inside the sensor and the capture window.
            var ix0 = ext.x
            var ix1 = sourceWidth - ext.x
            var iy0 = ext.y
            var iy1 = sourceHeight - ext.y
            r.capture?.let { cap ->
                ix0 = max(ix0, cap.left + ext.x); ix1 = min(ix1, cap.right - ext.x)
                iy0 = max(iy0, cap.top + ext.y); iy1 = min(iy1, cap.bottom - ext.y)
            }
            if (ix0 > ix1) { val m = (ix0 + ix1) / 2; ix0 = m; ix1 = m }
            if (iy0 > iy1) { val m = (iy0 + iy1) / 2; iy0 = m; iy1 = m }
            // Image bounds → world bounds (roll coupling is small; the exact clamp happens at emission).
            val wLo = pose.toWorld(Vec2(ix0, iy0), center)
            val wHi = pose.toWorld(Vec2(ix1, iy1), center)
            val fLoX = min(wLo.x, wHi.x) / scale
            val fHiX = max(wLo.x, wHi.x) / scale
            val fLoY = min(wLo.y, wHi.y) / scale
            val fHiY = max(wLo.y, wHi.y) / scale

            if (t != null && t.hasTarget) {
                val q = t.worldAt(r.ts)
                var lead = t.velocity * c.leadTimeS
                val maxLead = c.maxLeadFraction * r.cropW / 2
                if (lead.length() > maxLead) lead = lead * (maxLead / lead.length())
                gx[k] = (q.x + lead.x) / scale
                gy[k] = (q.y + lead.y) / scale
                a[k] = 1.0
                val mx = c.maxTargetOffset * r.cropW / 2 / scale
                val my = c.maxTargetOffset * r.cropH / 2 / scale
                loX[k] = q.x / scale - mx; hiX[k] = q.x / scale + mx
                loY[k] = q.y / scale - my; hiY[k] = q.y / scale + my
            } else {
                val w = pose.toWorld(center, center)
                gx[k] = w.x / scale
                gy[k] = w.y / scale
                a[k] = c.eisGoalWeight
            }
            // Sensor / capture bounds always win over target containment.
            loX[k] = max(loX[k], fLoX).coerceAtMost(fHiX); hiX[k] = min(hiX[k], fHiX).coerceAtLeast(loX[k])
            loY[k] = max(loY[k], fLoY).coerceAtMost(fHiY); hiY[k] = min(hiY[k], fHiY).coerceAtLeast(loY[k])
            xs[k] = gx[k].clampSafe(loX[k], hiX[k])
            ys[k] = gy[k].clampSafe(loY[k], hiY[k])

            rollGoal[k] = pose.roll
            rollLo[k] = pose.roll - c.maxRoll
            rollHi[k] = pose.roll + c.maxRoll
            rolls[k] = pose.roll
        }

        val dt = lastDtS
        val weights = c.pathWeights()
        smoother.solve(xs, gx, a, loX, hiX, 0, dt, weights)
        smoother.solve(ys, gy, a, loY, hiY, 0, dt, weights)
        if (c.rollStabilization) {
            // The horizon only needs to be steady, not locked: follow slow roll changes.
            smoother.solve(rolls, rollGoal, rollW, rollLo, rollHi, 0, dt, c.copy(cinematic = false).pathWeights(c.cutoffHz * (1 - 0.7 * c.rollSmoothness)).copy(robustGoalScale = Double.NaN))
        }

        // Emit the center frame with an exact clamp in image space.
        val r = pending.removeFirst()
        past.addLast(r)
        while (past.size > max((c.delayFrames * c.pastFactor).toInt(), MIN_PAST)) past.removeFirst()
        val pose = motion.poseAt(r.ts)
        val virtualRoll = if (c.rollStabilization) rolls[cur] else pose.roll
        val limit = rotationLimit(r)
        val rotation = (pose.roll - virtualRoll).coerceIn(-limit, limit)
        val ext = CropWindow.rotatedHalfExtents(r.cropW, r.cropH, rotation)
        var img = pose.toImage(Vec2(xs[cur] * scale, ys[cur] * scale), center)
        var x0 = ext.x
        var x1 = sourceWidth - ext.x
        var y0 = ext.y
        var y1 = sourceHeight - ext.y
        r.capture?.let { cap ->
            x0 = max(x0, cap.left + ext.x); x1 = min(x1, cap.right - ext.x)
            y0 = max(y0, cap.top + ext.y); y1 = min(y1, cap.bottom - ext.y)
        }
        img = Vec2(img.x.clampSafe(x0, x1), img.y.clampSafe(y0, y1))
        return Output(r.ts, CropWindow(img.x, img.y, r.cropW, r.cropH, rotation), r.track)
    }

    /** Largest crop rotation for this frame that still fits in the sensor and capture window. */
    private fun rotationLimit(r: Rec): Double {
        val availW = min(sourceWidth.toDouble(), r.capture?.width ?: Double.MAX_VALUE)
        val availH = min(sourceHeight.toDouble(), r.capture?.height ?: Double.MAX_VALUE)
        return CropWindow.maxFittingRotation(r.cropW, r.cropH, availW, availH, config.maxRoll)
    }

    private companion object {
        /** Past frames kept for the window even when the look-ahead is short. */
        const val MIN_PAST = 8
    }
}
