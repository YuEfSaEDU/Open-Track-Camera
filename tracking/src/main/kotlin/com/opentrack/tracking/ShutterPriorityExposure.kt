package com.opentrack.tracking

import kotlin.math.abs
import kotlin.math.exp
import kotlin.math.ln

/**
 * Shutter priority exposure: a fixed short exposure time (against motion blur: a stabilized
 * crop shows every frame's blur, while the shake that caused it is gone) with the sensitivity
 * (ISO) adjusted continuously to keep the brightness the camera's own auto exposure chose.
 *
 * Camera2 cannot cap the auto exposure time, so ISO is driven here from the mean luma of the
 * analysis frames. While the camera's auto exposure runs, [observeAuto] learns the brightness
 * it aims for; in shutter priority, [update] moves ISO towards that brightness in damped,
 * rate-limited steps (no flicker, no pumping on a passing cloud). Pure logic, no Android.
 */
class ShutterPriorityExposure(
    private val isoMin: Int,
    private val isoMax: Int,
    /** Fraction of the remaining (log) error corrected per update (updates at ~10 Hz). */
    private val gain: Double = 0.35,
    /** Largest ISO change per update, as a factor. */
    private val maxStep: Double = 1.25,
    /** Errors smaller than this (fraction of the target) are ignored. */
    private val deadband: Double = 0.04,
) {
    /** Brightness (mean luma, 0..255) auto exposure produced; null until observed. */
    var targetLuma: Double? = null
        private set

    var iso: Int = isoMin
        private set

    /** Auto exposure is running: learn its brightness (slow average, ignores single frames). */
    fun observeAuto(meanLuma: Double) {
        val t = targetLuma
        targetLuma = if (t == null) meanLuma else t + (meanLuma - t) * 0.1
    }

    /** Entering shutter priority: same light as the last auto frame, at the new exposure time. */
    fun start(autoIso: Int, autoExposureNs: Long, shutterNs: Long) {
        val i = if (autoIso > 0 && autoExposureNs > 0) autoIso.toDouble() * autoExposureNs / shutterNs else isoMin.toDouble()
        iso = i.toInt().coerceIn(isoMin, isoMax)
    }

    /**
     * New measurement in shutter priority. Returns the ISO to use when it changed enough to
     * be worth a new capture request, else null.
     */
    fun update(meanLuma: Double): Int? {
        val target = targetLuma ?: return null
        if (meanLuma <= 1.0) return null
        val ratio = target / meanLuma
        if (abs(ratio - 1) < deadband) return null
        // Luma is gamma encoded (~2.2): the light ratio is the luma ratio to that power.
        val logStep = (gain * 2.2 * ln(ratio)).coerceIn(-ln(maxStep), ln(maxStep))
        val next = (iso * exp(logStep)).toInt().coerceIn(isoMin, isoMax)
        if (next == iso) return null
        iso = next
        return next
    }

    companion object {
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
