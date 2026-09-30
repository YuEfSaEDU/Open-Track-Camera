package com.opentrack.tracking

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/**
 * Classical detector for small objects against the sky: finds compact blobs that differ
 * strongly from their local background while the surrounding ring is smooth. Distant
 * aircraft are often only a few pixels wide, well below what a neural detector running on a
 * downscaled frame can see, but they stand out clearly against a uniform sky.
 */
class SkyBlobDetector(
    /** Box blur radius (analysis px) used to estimate the local background. */
    private val backgroundRadius: Int = 12,
    /** Minimum absolute contrast against background, in gray levels. */
    private val minContrast: Int = 14,
    /** Contrast threshold as a multiple of the robust global noise level. */
    private val noiseFactor: Double = 6.0,
    private val minArea: Int = 2,
    private val maxArea: Int = 900,
    /** Maximum std-dev (gray levels) of the ring around a blob for it to count as "sky". */
    private val maxRingStd: Double = 9.0,
    private val maxResults: Int = 8,
) {
    // Scratch buffers reused across frames (the detector runs many times per second).
    private var integral = LongArray(0)
    private var integralSq = LongArray(0)
    private var diff = IntArray(0)
    private var labels = IntArray(0)
    private var stack = IntArray(0)
    private val histogram = IntArray(256)

    /** Not thread safe: scratch buffers are shared between calls. */
    fun detect(frame: LumaFrame): List<Detection> {
        val w = frame.width
        val h = frame.height
        val n = w * h
        if (diff.size != n) {
            integral = LongArray((w + 1) * (h + 1))
            integralSq = LongArray((w + 1) * (h + 1))
            diff = IntArray(n)
            labels = IntArray(n)
            stack = IntArray(n)
        } else {
            labels.fill(0)
        }
        histogram.fill(0)
        val integral = integral
        val integralSq = integralSq
        val diff = diff
        val labels = labels
        val stack = stack
        buildIntegrals(frame, integral, integralSq)

        // Difference to local mean.
        val r = backgroundRadius
        for (y in 0 until h) {
            val y0 = max(0, y - r)
            val y1 = min(h, y + r + 1)
            for (x in 0 until w) {
                val x0 = max(0, x - r)
                val x1 = min(w, x + r + 1)
                val area = (x1 - x0) * (y1 - y0)
                val mean = rectSum(integral, w, x0, y0, x1, y1) / area
                val d = frame.at(x, y) - mean.toInt()
                diff[y * w + x] = d
                histogram[min(255, abs(d))]++
            }
        }
        // Median absolute deviation as a robust noise estimate (most pixels are background).
        val median = percentile(histogram, n, 0.5)
        val threshold = max(minContrast.toDouble(), noiseFactor * 1.4826 * max(1, median))

        val out = ArrayList<Detection>()
        var nextLabel = 1
        for (start in 0 until n) {
            if (labels[start] != 0 || abs(diff[start]) < threshold) continue
            val sign = if (diff[start] > 0) 1 else -1
            val label = nextLabel++
            // Flood fill (8-connected) over pixels with the same contrast polarity.
            var sp = 0
            stack[sp++] = start
            labels[start] = label
            var area = 0
            var minX = w; var minY = h; var maxX = -1; var maxY = -1
            var wSum = 0.0; var wx = 0.0; var wy = 0.0
            var peak = 0
            while (sp > 0) {
                val p = stack[--sp]
                val px = p % w
                val py = p / w
                area++
                val mag = abs(diff[p])
                wSum += mag; wx += mag * px; wy += mag * py
                if (mag > peak) peak = mag
                if (px < minX) minX = px
                if (px > maxX) maxX = px
                if (py < minY) minY = py
                if (py > maxY) maxY = py
                if (area > maxArea) continue
                for (dy in -1..1) {
                    val ny = py + dy
                    if (ny < 0 || ny >= h) continue
                    for (dx in -1..1) {
                        val nx = px + dx
                        if (nx < 0 || nx >= w) continue
                        val q = ny * w + nx
                        if (labels[q] == 0 && diff[q] * sign >= threshold) {
                            labels[q] = label
                            stack[sp++] = q
                        }
                    }
                }
            }
            if (area < minArea || area > maxArea) continue
            val bw = maxX - minX + 1
            val bh = maxY - minY + 1
            if (area.toDouble() / (bw * bh) < 0.2) continue // thin/irregular: cloud edge, wire...

            // Ring between 2x and 3x the blob size must be smooth sky.
            val ringStd = ringStd(integral, integralSq, w, h, minX, minY, maxX + 1, maxY + 1)
            if (ringStd > maxRingStd) continue

            val cx = wx / wSum
            val cy = wy / wSum
            val size = max(bw, bh).toDouble()
            val score = (peak / (threshold + 3 * ringStd)).coerceAtMost(4.0) / 4.0
            val s = frame.scale
            out += Detection(
                box = Box(cx * s, cy * s, max(bw.toDouble(), 3.0) * s, max(bh.toDouble(), 3.0) * s),
                score = score.toFloat(),
                label = if (size <= 4) "dot" else "object",
                source = DetectionSource.SKY_BLOB,
                timestampNs = frame.timestampNs,
            )
        }
        return out.sortedByDescending { it.score }.take(maxResults)
    }

    private fun ringStd(ii: LongArray, iiSq: LongArray, w: Int, h: Int, x0: Int, y0: Int, x1: Int, y1: Int): Double {
        val bw = x1 - x0
        val bh = y1 - y0
        fun expand(k: Double): IntArray {
            val ex = (bw * (k - 1) / 2).toInt() + 2
            val ey = (bh * (k - 1) / 2).toInt() + 2
            return intArrayOf(max(0, x0 - ex), max(0, y0 - ey), min(w, x1 + ex), min(h, y1 + ey))
        }
        val inner = expand(2.0)
        val outer = expand(3.0)
        val sum = rectSum(ii, w, outer[0], outer[1], outer[2], outer[3]) - rectSum(ii, w, inner[0], inner[1], inner[2], inner[3])
        val sumSq = rectSum(iiSq, w, outer[0], outer[1], outer[2], outer[3]) - rectSum(iiSq, w, inner[0], inner[1], inner[2], inner[3])
        val count = (outer[2] - outer[0]) * (outer[3] - outer[1]) - (inner[2] - inner[0]) * (inner[3] - inner[1])
        if (count <= 0) return Double.MAX_VALUE
        val mean = sum.toDouble() / count
        return sqrt(max(0.0, sumSq.toDouble() / count - mean * mean))
    }

    private fun buildIntegrals(frame: LumaFrame, ii: LongArray, iiSq: LongArray) {
        val w = frame.width
        val stride = w + 1
        for (y in 0 until frame.height) {
            var rowSum = 0L
            var rowSq = 0L
            for (x in 0 until w) {
                val v = frame.at(x, y).toLong()
                rowSum += v
                rowSq += v * v
                ii[(y + 1) * stride + x + 1] = ii[y * stride + x + 1] + rowSum
                iiSq[(y + 1) * stride + x + 1] = iiSq[y * stride + x + 1] + rowSq
            }
        }
    }

    private fun rectSum(ii: LongArray, w: Int, x0: Int, y0: Int, x1: Int, y1: Int): Long {
        val s = w + 1
        return ii[y1 * s + x1] - ii[y0 * s + x1] - ii[y1 * s + x0] + ii[y0 * s + x0]
    }

    private fun percentile(histogram: IntArray, total: Int, q: Double): Int {
        val target = (total * q).toLong()
        var acc = 0L
        for (i in histogram.indices) {
            acc += histogram[i]
            if (acc >= target) return i
        }
        return histogram.size - 1
    }
}
