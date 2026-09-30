package com.opentrack.tracking

import kotlin.math.abs
import kotlin.math.exp
import kotlin.math.ln
import kotlin.math.max

data class ZoomConfig(
    /** Automatic zoom on/off. When off, [FramingController.zoom] is used as set by the user. */
    val auto: Boolean = false,
    /** User limits for the automatic zoom (1 = widest crop). */
    val minZoom: Double = 1.0,
    val maxZoom: Double = 4.0,
    /** Desired size of the target's larger side as a fraction of the crop width. */
    val targetFill: Double = 0.25,
    /** Relative size error (in ln units, ~12%) tolerated before the zoom starts moving. */
    val deadband: Double = 0.12,
    /** Time constant of the zoom response, seconds. */
    val responseS: Double = 0.7,
    /** Max zoom speed, ln units per second. Zooming out is allowed to be faster. */
    val maxZoomInRate: Double = 0.45,
    val maxZoomOutRate: Double = 1.2,
    /** While the tracker only predicts, widen by this factor per second of coasting. */
    val coastWidenPerS: Double = 0.6,
    /** Zoom out when the target drifts further than this from the crop center (fraction of half crop). */
    val offsetWiden: Double = 0.3,
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

    private var logZoom = Double.NaN
    private var moving = false

    val zoom: Double get() = exp(logZoom)

    fun reset(current: Double) {
        logZoom = ln(current)
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
        if (logZoom.isNaN()) logZoom = ln(current.coerceIn(lo, hi))

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
        val goal = ln(desired.coerceIn(lo, hi))

        val error = goal - logZoom
        // Hysteresis: start moving outside the deadband, stop once close to the goal. Without a
        // target there is no noisy size estimate, so go all the way to the widest zoom.
        if (abs(error) < c.deadband / 10) moving = false
        if (abs(error) > c.deadband || (!track.hasTarget && abs(error) > 1e-4)) moving = true
        if (moving && dtS > 0) {
            val rate = (error / c.responseS).coerceIn(-c.maxZoomOutRate, c.maxZoomInRate)
            logZoom += rate * dtS
        }
        logZoom = logZoom.coerceIn(ln(lo), ln(hi))
        return zoom
    }
}
