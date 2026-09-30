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
) {
    data class Match(val center: Vec2, val score: Double)

    private var template = FloatArray(0)
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
    }

    fun reset() {
        tw = 0
        th = 0
        template = FloatArray(0)
    }

    /**
     * Searches within [radius] analysis pixels of [predicted]. Returns null when the search
     * window falls entirely outside the frame or the template is flat.
     */
    fun track(frame: LumaFrame, predicted: Vec2, radius: Int): Match? {
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

        var tMean = 0.0
        for (v in template) tMean += v
        tMean /= template.size
        var tVar = 0.0
        for (v in template) tVar += (v - tMean) * (v - tMean)
        if (tVar < 1e-6) return null
        val tNorm = sqrt(tVar)
        val tz = FloatArray(template.size) { (template[it] - tMean).toFloat() }

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

        if (best >= adaptMinScore) adapt(frame, bx, by)
        return Match(Vec2(sx, sy), best)
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
