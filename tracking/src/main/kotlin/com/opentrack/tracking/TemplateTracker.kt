package com.opentrack.tracking

import kotlin.math.max
import kotlin.math.min
import kotlin.math.roundToInt
import kotlin.math.sqrt

/**
 * Normalized cross-correlation template tracker working on [LumaFrame]s (analysis
 * resolution). Coarse-to-fine search around a predicted position with sub-pixel peak
 * refinement, plus a slowly adapting template so it survives gradual appearance changes
 * (banking aircraft, lighting).
 */
class TemplateTracker(
    private val minTemplate: Int = 9,
    private val maxTemplate: Int = 41,
    /** Context around the object box included in the template. */
    private val contextScale: Double = 1.4,
    /** Template blend rate when a match is confident. */
    private val adaptRate: Float = 0.08f,
    private val adaptMinScore: Double = 0.82,
    /** The adaptive template is only updated while it still resembles the anchor this much. */
    private val adaptMinAnchorScore: Double = 0.7,
) {
    /**
     * @param score NCC against the adaptive template
     * @param anchorScore NCC of the same position against the anchor template (the
     *   appearance when the track was seeded from a detection); guards against drift.
     * @param contrast std-dev (gray levels) of the matched patch
     * @param contrastRatio [contrast] / anchor std-dev. NCC ignores contrast, so a faded
     *   target and flat noisy sky can still score high; this catches it.
     */
    data class Match(val center: Vec2, val score: Double, val anchorScore: Double, val contrast: Double, val contrastRatio: Double)

    private var template = FloatArray(0)
    private var anchor = FloatArray(0)
    private var tw = 0
    private var th = 0

    val isInitialized: Boolean get() = tw > 0

    val templateWidth: Int get() = tw
    val templateHeight: Int get() = th

    /** Initializes from an object box expressed in analysis pixels. */
    fun init(frame: LumaFrame, box: Box) {
        tw = oddClamp(box.width * contextScale)
        th = oddClamp(box.height * contextScale)
        template = FloatArray(tw * th)
        extract(frame, box.cx.roundToInt(), box.cy.roundToInt(), template)
        anchor = template.copyOf()
    }

    /** Refreshes the anchor at [center] (analysis px), e.g. when a detection confirms the track. */
    fun reanchor(frame: LumaFrame, center: Vec2) {
        if (!isInitialized) return
        extract(frame, center.x.roundToInt(), center.y.roundToInt(), anchor)
    }

    fun reset() {
        tw = 0
        th = 0
        template = FloatArray(0)
        anchor = FloatArray(0)
    }

    /**
     * Searches within [radius] analysis pixels of [predicted]. Returns null when the search
     * window falls entirely outside the frame or the template is flat.
     */
    fun track(frame: LumaFrame, predicted: Vec2, radius: Int, adapt: Boolean = true): Match? {
        if (!isInitialized) return null
        val hw = tw / 2
        val hh = th / 2
        val px = predicted.x.roundToInt()
        val py = predicted.y.roundToInt()
        val x0 = max(hw, px - radius)
        val x1 = min(frame.width - 1 - hw, px + radius)
        val y0 = max(hh, py - radius)
        val y1 = min(frame.height - 1 - hh, py + radius)
        if (x0 > x1 || y0 > y1) return null

        val (tz, tNorm) = zeroMean(template) ?: return null

        // Coarse pass on a 2px grid, then a dense pass around the best coarse hit.
        val step = if (radius > 6) 2 else 1
        var best = Double.NEGATIVE_INFINITY
        var bx = px.coerceIn(x0, x1)
        var by = py.coerceIn(y0, y1)
        var y = y0
        while (y <= y1) {
            var x = x0
            while (x <= x1) {
                val s = ncc(frame, x, y, tz, tNorm)
                if (s > best) { best = s; bx = x; by = y }
                x += step
            }
            y += step
        }
        if (step > 1) {
            val cx = bx
            val cy = by
            for (yy in max(y0, cy - 1)..min(y1, cy + 1)) for (xx in max(x0, cx - 1)..min(x1, cx + 1)) {
                val s = ncc(frame, xx, yy, tz, tNorm)
                if (s > best) { best = s; bx = xx; by = yy }
            }
        }

        // Parabolic sub-pixel refinement.
        var sx = bx.toDouble()
        var sy = by.toDouble()
        if (bx in (x0 + 1)..(x1 - 1)) {
            sx += parabolicPeak(ncc(frame, bx - 1, by, tz, tNorm), best, ncc(frame, bx + 1, by, tz, tNorm))
        }
        if (by in (y0 + 1)..(y1 - 1)) {
            sy += parabolicPeak(ncc(frame, bx, by - 1, tz, tNorm), best, ncc(frame, bx, by + 1, tz, tNorm))
        }

        val anchorZm = zeroMean(anchor)
        val anchorScore = anchorZm?.let { (az, aNorm) -> ncc(frame, bx, by, az, aNorm) } ?: 0.0
        val n = template.size
        val contrast = patchStd(frame, bx, by)
        val anchorStd = anchorZm?.let { it.second / sqrt(n.toDouble()) } ?: 0.0
        val ratio = if (anchorStd > 1e-6) contrast / anchorStd else 0.0
        val m = Match(Vec2(sx, sy), best, anchorScore, contrast, ratio)
        if (adapt) adaptTo(frame, m)
        return m
    }

    /** Blends the template towards [m]'s location when the match is confident and still anchored. */
    fun adaptTo(frame: LumaFrame, m: Match) {
        if (m.score >= adaptMinScore && m.anchorScore >= adaptMinAnchorScore && m.contrastRatio > 0.6) {
            adapt(frame, m.center.x.roundToInt(), m.center.y.roundToInt())
        }
    }

    private fun patchStd(frame: LumaFrame, cx: Int, cy: Int): Double {
        val left = cx - tw / 2
        val top = cy - th / 2
        var sum = 0.0
        var sumSq = 0.0
        for (row in 0 until th) {
            var idx = (top + row) * frame.width + left
            for (col in 0 until tw) {
                val v = (frame.data[idx++].toInt() and 0xFF).toDouble()
                sum += v
                sumSq += v * v
            }
        }
        val n = (tw * th).toDouble()
        val mean = sum / n
        return sqrt(max(0.0, sumSq / n - mean * mean))
    }

    private fun zeroMean(t: FloatArray): Pair<FloatArray, Double>? {
        var mean = 0.0
        for (v in t) mean += v
        mean /= t.size
        var variance = 0.0
        for (v in t) variance += (v - mean) * (v - mean)
        if (variance < 1e-6) return null
        return FloatArray(t.size) { (t[it] - mean).toFloat() } to sqrt(variance)
    }

    private fun ncc(frame: LumaFrame, cx: Int, cy: Int, tz: FloatArray, tNorm: Double): Double {
        val w = frame.width
        val data = frame.data
        val left = cx - tw / 2
        val top = cy - th / 2
        var sum = 0.0
        var sumSq = 0.0
        var cross = 0.0
        var i = 0
        for (row in 0 until th) {
            var idx = (top + row) * w + left
            for (col in 0 until tw) {
                val v = (data[idx].toInt() and 0xFF).toDouble()
                sum += v
                sumSq += v * v
                cross += v * tz[i]
                idx++
                i++
            }
        }
        val n = tz.size
        val iVar = sumSq - sum * sum / n
        if (iVar < 1e-6) return 0.0
        // Σ(I - Ī)(T - T̄) == Σ I (T - T̄) because Σ(T - T̄) == 0.
        return cross / (sqrt(iVar) * tNorm)
    }

    private fun adapt(frame: LumaFrame, cx: Int, cy: Int) {
        val fresh = FloatArray(template.size)
        extract(frame, cx, cy, fresh)
        val a = adaptRate
        for (i in template.indices) template[i] = template[i] * (1 - a) + fresh[i] * a
    }

    private fun extract(frame: LumaFrame, cx: Int, cy: Int, out: FloatArray) {
        val left = cx - tw / 2
        val top = cy - th / 2
        var i = 0
        for (row in 0 until th) {
            val y = (top + row).coerceIn(0, frame.height - 1)
            for (col in 0 until tw) {
                val x = (left + col).coerceIn(0, frame.width - 1)
                out[i++] = frame.at(x, y).toFloat()
            }
        }
    }

    private fun oddClamp(v: Double): Int {
        val n = v.roundToInt().coerceIn(minTemplate, maxTemplate)
        return if (n % 2 == 0) n + 1 else n
    }

    private fun parabolicPeak(left: Double, center: Double, right: Double): Double {
        val denom = left - 2 * center + right
        if (denom >= 0) return 0.0
        return (0.5 * (left - right) / denom).coerceIn(-0.5, 0.5)
    }
}
