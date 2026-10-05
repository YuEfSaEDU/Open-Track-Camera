package com.opentrack.tracking

import kotlin.math.abs
import kotlin.math.exp
import kotlin.math.ln
import kotlin.math.max
import kotlin.math.sqrt

data class ZoomConfig(
    /** Automatic zoom on/off. When off, [FramingController.zoom] is used as set by the user. */
    val auto: Boolean = false,
    /** User limits for the automatic zoom (1 = widest crop). */
    val minZoom: Double = 1.0,
    val maxZoom: Double = 4.0,
    /** Desired size of the target's larger side as a fraction of the crop width. */
    val targetFill: Double = 0.35,
    /** Relative size error (in ln units, ~12%) tolerated before the zoom starts moving. */
    val deadband: Double = 0.12,
    /** Time constant of the zoom response, seconds (the approach to the goal at the end). */
    val responseS: Double = 1.2,
    /** ...when zooming out (widening to keep the target is more urgent). */
    val responseOutS: Double = 0.6,
    /** Max zoom speed, ln units per second. Zooming out is allowed to be faster. */
    val maxZoomInRate: Double = 0.3,
    val maxZoomOutRate: Double = 0.9,
    /**
     * Max zoom acceleration, ln units per second²: a zoom starts and stops gently instead of at
     * full speed at once (like an operator easing a zoom ring). Out is quicker (losing the
     * target is worse than a loose frame).
     */
    val maxZoomInAccel: Double = 0.35,
    val maxZoomOutAccel: Double = 1.2,
    /** While the tracker only predicts, widen by this factor per second of coasting. */
    val coastWidenPerS: Double = 0.3,
    /** Zoom out when the target drifts further than this from the crop center (fraction of half crop). */
    val offsetWiden: Double = 0.6,
)

/**
 * Automatic zoom within user limits: keeps the target at a constant size in the output
 * while it is tracked, widens when the tracker is only predicting or the framing falls
 * behind, and returns to the widest allowed zoom when there is no target so it can be
 * re-acquired. Zoom moves in log space with a deadband (no "breathing" on noisy size
 * estimates) and asymmetric speed limits (losing the target is worse than a loose frame).
 */
class AutoZoom(config: ZoomConfig = ZoomConfig()) {
    @Volatile
    var config: ZoomConfig = config

    private val ease = SmoothZoom()
    private var goal = Double.NaN
    private var moving = false

    val zoom: Double get() = exp(ease.logZoom)

    fun reset(current: Double) {
        ease.reset(ln(current))
        goal = ease.logZoom
        moving = false
    }

    /**
     * @param coastingS seconds since the target was last measured (0 while tracking)
     * @param targetOffset distance of the target from the crop center, fraction of half crop size
     */
    fun update(dtS: Double, current: Double, track: TrackPrediction, sourceWidth: Int, outputAspect: Double, coastingS: Double, targetOffset: Double): Double {
        val c = config
        val lo = max(1.0, minOf(c.minZoom, c.maxZoom))
        val hi = max(lo, c.maxZoom)
        if (ease.logZoom.isNaN()) reset(current.coerceIn(lo, hi))

        var desired = if (track.hasTarget && track.width > 0) {
            // Size in "crop width" terms: a tall target must fit vertically as well.
            val size = max(track.width, track.height * outputAspect)
            c.targetFill * sourceWidth / size
        } else {
            lo
        }
        if (track.hasTarget) {
            if (coastingS > 0) desired *= exp(-c.coastWidenPerS * coastingS)
            if (targetOffset > c.offsetWiden) desired *= exp(-(targetOffset - c.offsetWiden) * 2)
        }
        val wanted = ln(desired.coerceIn(lo, hi))

        // Hysteresis: start moving once the wanted zoom leaves the deadband, follow it while
        // moving, stop following once close (the ease then settles on the last goal). Without a
        // target there is no noisy size estimate, so go all the way to the widest zoom.
        val error = wanted - ease.logZoom
        if (moving && abs(error) < c.deadband / 10) moving = false
        if (abs(error) > c.deadband || (!track.hasTarget && abs(error) > 1e-4)) moving = true
        if (moving || goal.isNaN()) goal = wanted
        goal = goal.coerceIn(ln(lo), ln(hi))
        ease.step(dtS, goal, c.responseS, c.responseOutS, c.maxZoomInRate, c.maxZoomOutRate, c.maxZoomInAccel, c.maxZoomOutAccel)
        ease.clamp(ln(lo), ln(hi))
        return zoom
    }
}

/**
 * Eases a zoom (in log units, so 1x → 2x looks like 2x → 4x) towards a goal: speed and
 * acceleration limited, slowing down early enough to arrive without overshoot, and approaching
 * exponentially at the end ([step]'s responseS). Starts and stops are gentle ramps, never a jump
 * in zoom speed.
 */
class SmoothZoom {
    var logZoom = Double.NaN
        private set
    /** Current zoom speed, ln units per second (+ = zooming in). */
    var velocity = 0.0
        private set

    fun reset(log: Double) {
        logZoom = log
        velocity = 0.0
    }

    fun step(dtS: Double, goal: Double, responseS: Double, responseOutS: Double, inRate: Double, outRate: Double, inAccel: Double, outAccel: Double) {
        if (logZoom.isNaN()) reset(goal)
        if (dtS <= 0) return
        val err = goal - logZoom
        val accel = if (err >= 0) inAccel else outAccel
        // The speed wanted now: the exponential approach, the speed limit, and the speed from
        // which the acceleration limit can still stop exactly at the goal.
        val stopping = sqrt(2 * accel * abs(err))
        val speed = minOf(abs(err) / (if (err >= 0) responseS else responseOutS).coerceAtLeast(1e-3), if (err >= 0) inRate else outRate, stopping)
        val wanted = if (err >= 0) speed else -speed
        // Braking (or reversing) may use the larger of the two accelerations.
        val limit = if (abs(wanted) < abs(velocity) || wanted * velocity < 0) max(inAccel, outAccel) else accel
        velocity += (wanted - velocity).coerceIn(-limit * dtS, limit * dtS)
        val next = logZoom + velocity * dtS
        // Never step past the goal (the last frames of an approach).
        logZoom = if ((goal - logZoom) * (goal - next) < 0) goal.also { velocity = 0.0 } else next
    }

    fun clamp(lo: Double, hi: Double) {
        if (logZoom < lo) { logZoom = lo; velocity = max(velocity, 0.0) }
        if (logZoom > hi) { logZoom = hi; velocity = minOf(velocity, 0.0) }
    }
}
