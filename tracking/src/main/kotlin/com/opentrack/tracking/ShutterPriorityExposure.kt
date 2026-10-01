package com.opentrack.tracking

import kotlin.math.abs
import kotlin.math.exp
import kotlin.math.ln

/**
 * Shutter priority exposure: a short exposure time (against motion blur: a stabilized crop
 * shows every frame's blur, while the shake that caused it is gone) with the sensitivity (ISO)
 * adjusted continuously to keep the brightness the camera's own auto exposure chose.
 *
 * Camera2 cannot cap the auto exposure time, so exposure is driven here from the mean luma of
 * the analysis frames. While the camera's auto exposure runs, [observeAuto] learns the
 * brightness it aims for; in shutter priority, [update] moves the total exposure (ISO × time)
 * towards that brightness in damped, rate-limited steps (no flicker, no pumping on a passing
 * cloud). Each measurement is judged against the ISO and time that frame was actually taken
 * with, so the camera's settings latency (several frames) cannot make the loop overshoot or
 * oscillate: the required exposure is computed, not integrated from stale errors. ISO moves first; the chosen shutter time is a ceiling: only when ISO is already at
 * its minimum and the scene is still too bright (bright sun at 1/500 s) does the exposure time
 * get shorter, and it returns to the chosen time before ISO rises again. Pure logic, no Android.
 */
class ShutterPriorityExposure(
    private val isoMin: Int,
    private val isoMax: Int,
    /** Fraction of the remaining (log) exposure error corrected per update (updates at ~10 Hz). */
    private val gain: Double = 0.5,
    /** Largest exposure change per update, as a factor. */
    private val maxStep: Double = 1.4,
    /** Errors smaller than this (fraction of the target) are ignored. */
    private val deadband: Double = 0.04,
    /** Shortest exposure time the sensor supports (ns). */
    private val minExposureNs: Long = 1_000L,
) {
    /** Brightness (mean luma, 0..255) auto exposure produced; null until observed. */
    var targetLuma: Double? = null
        private set

    /** Settled auto exposure frames observed so far. */
    var observations = 0
        private set

    private var defaulted = false

    /** Enough of the settled auto exposure has been seen to take over (see [observeAuto]). */
    val ready: Boolean get() = targetLuma != null && (observations >= MIN_OBSERVATIONS || defaulted)

    /** Brightness offset chosen by the user, in EV (stops): +1 = twice the light. */
    var ev: Double = 0.0

    /** The brightness aimed for: the learned one moved by [ev] (luma is gamma ~2.2 encoded). */
    val aimLuma: Double? get() = targetLuma?.let { (it * Math.pow(2.0, ev / 2.2)).coerceIn(8.0, 235.0) }

    /**
     * Auto exposure is running and has settled (call only for such frames: while it is still
     * converging, e.g. right after the camera opened, frames are much too dark): learn its
     * brightness (slow average, ignores single frames).
     */
    fun observeAuto(meanLuma: Double) {
        val t = targetLuma
        targetLuma = if (t == null || defaulted) meanLuma else t + (meanLuma - t) * 0.1
        defaulted = false
        observations++
    }

    /** Auto exposure never reported settling: aim for middle grey. */
    fun useDefaultTarget() {
        if (targetLuma != null && observations > 0) return
        targetLuma = MIDDLE_GREY_LUMA
        defaulted = true
    }

    var iso: Int = isoMin
        private set

    /** Exposure time to use (ns): the chosen shutter time, or shorter when ISO cannot go lower. */
    var exposureNs: Long = 0L
        private set

    /** The chosen shutter time: the longest exposure used (motion blur limit). */
    private var maxExposureNs: Long = 0L

    /** Entering shutter priority: same light as the last auto frame, at the new exposure time. */
    fun start(autoIso: Int, autoExposureNs: Long, shutterNs: Long) {
        maxExposureNs = shutterNs
        val total = if (autoIso > 0 && autoExposureNs > 0) autoIso.toDouble() * autoExposureNs else isoMin.toDouble() * shutterNs
        allocate(total)
    }

    /**
     * New measurement in shutter priority: [meanLuma] of a frame taken at [frameIso] and
     * [frameExposureNs]. Returns true when ISO or exposure time changed enough to be worth a
     * new capture request.
     */
    fun update(meanLuma: Double, frameIso: Int, frameExposureNs: Long): Boolean {
        val target = aimLuma ?: return false
        if (maxExposureNs <= 0 || frameIso <= 0 || frameExposureNs <= 0) return false
        // A black (or white) frame still says which way to go: clamp instead of ignoring it.
        val ratio = target / meanLuma.coerceIn(2.0, 253.0)
        // Luma is gamma encoded (~2.2): the light ratio is the luma ratio to that power.
        val needed = frameIso.toDouble() * frameExposureNs * Math.pow(ratio, 2.2)
        val current = iso.toDouble() * exposureNs
        val error = ln(needed / current)
        if (abs(ratio - 1) < deadband && abs(error) < 2.2 * deadband) return false
        val logStep = (gain * error).coerceIn(-ln(maxStep), ln(maxStep))
        val oldIso = iso
        val oldExposure = exposureNs
        allocate(current * exp(logStep))
        return iso != oldIso || abs(exposureNs - oldExposure) > oldExposure * 0.01
    }

    /** Splits a total exposure (ISO × ns) into ISO first, then a shorter time below minimum ISO. */
    private fun allocate(total: Double) {
        if (total >= isoMin.toDouble() * maxExposureNs) {
            exposureNs = maxExposureNs
            iso = (total / maxExposureNs).toInt().coerceIn(isoMin, isoMax)
        } else {
            iso = isoMin
            exposureNs = (total / isoMin).toLong().coerceIn(minOf(minExposureNs, maxExposureNs), maxExposureNs)
        }
    }

    companion object {
        /** Settled auto exposure frames to learn from before taking over (~1 s at 10 Hz). */
        const val MIN_OBSERVATIONS = 10

        /** 18% grey in sRGB. */
        const val MIDDLE_GREY_LUMA = 118.0

        /** Mean luma of an 8-bit frame, sampling every [step]-th pixel in both directions. */
        fun meanLuma(frame: LumaFrame, step: Int = 4): Double {
            var sum = 0L
            var n = 0
            var y = 0
            while (y < frame.height) {
                var x = 0
                val row = y * frame.width
                while (x < frame.width) {
                    sum += frame.data[row + x].toInt() and 0xFF
                    n++
                    x += step
                }
                y += step
            }
            return if (n == 0) 0.0 else sum.toDouble() / n
        }
    }
}
