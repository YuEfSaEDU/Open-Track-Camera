package com.opentrack.tracking

import kotlin.math.abs
import kotlin.math.exp
import kotlin.math.floor
import kotlin.math.hypot
import kotlin.math.ln
import kotlin.math.max
import kotlin.math.min

/**
 * What the exposure loop measures on an analysis frame.
 *
 * @param mean brightness to regulate (mean luma, 0..255): the recorded crop, with the locked
 *   target weighted in.
 * @param highlight the crop's bright end ([ExposureMeter.HIGHLIGHT_PERCENTILE] luma): how
 *   close the brightest part of the recording is to clipping.
 */
data class SceneMetering(val mean: Double, val highlight: Double)

/**
 * Metering for the exposure loop. The whole sensor frame is not what gets recorded: at 2-3x
 * crop zoom most of it (ground, sun, a bright cloud bank) is outside the picture. So the
 * brightness is measured on the recorded crop, and a locked target is weighted in (an aircraft
 * against a bright sky should not turn into a silhouette). Pure logic, no Android.
 */
object ExposureMeter {
    /** Share of the brightness taken from the target box once it is large enough. */
    const val TARGET_WEIGHT = 0.5

    /** Target size (analysis px) from which it gets the full [TARGET_WEIGHT]; smaller ones get less. */
    const val TARGET_FULL_WEIGHT_PX = 64

    /** The crop's bright end used for highlight protection (98th percentile). */
    const val HIGHLIGHT_PERCENTILE = 0.98

    /**
     * Meters [frame] over [crop] (source px; the whole frame when null), with [target] (source px)
     * weighted in when given.
     */
    fun measure(frame: LumaFrame, crop: CropWindow?, target: Box?): SceneMetering {
        val s = frame.scale
        // The crop's axis-aligned bounds (its small roll does not matter for metering).
        val (ax0, ay0, ax1, ay1) = if (crop == null) {
            listOf(0, 0, frame.width, frame.height)
        } else {
            val (hx, hy) = crop.halfExtents()
            bounds((crop.cx - hx) / s, (crop.cy - hy) / s, (crop.cx + hx) / s, (crop.cy + hy) / s, frame)
        }
        // Only what the camera saw (not the black around a telephoto frame).
        val (vx0, vy0, vx1, vy1) = frame.visible?.let { bounds(it.left / s, it.top / s, it.right / s, it.bottom / s, frame) }
            ?: listOf(0, 0, frame.width, frame.height)
        val cx0 = max(ax0, vx0)
        val cy0 = max(ay0, vy0)
        val cx1 = min(ax1, vx1)
        val cy1 = min(ay1, vy1)
        // Coarse sampling of a large crop keeps this well under a millisecond.
        val area = (cx1 - cx0).toLong() * (cy1 - cy0)
        val step = if (area > 60_000) 2 else 1
        val hist = IntArray(256)
        var sum = 0L
        var n = 0
        var y = cy0
        while (y < cy1) {
            val row = y * frame.width
            var x = cx0
            while (x < cx1) {
                val v = frame.data[row + x].toInt() and 0xFF
                hist[v]++
                sum += v
                n++
                x += step
            }
            y += step
        }
        if (n == 0) return SceneMetering(meanAll(frame), 0.0)
        var mean = sum.toDouble() / n
        val highlight = percentile(hist, n, HIGHLIGHT_PERCENTILE)

        if (target != null) {
            val (tx0, ty0, tx1, ty1) = bounds(target.left / s, target.top / s, target.right / s, target.bottom / s, frame)
            var tSum = 0L
            var tn = 0
            for (ty in ty0 until ty1) for (tx in tx0 until tx1) {
                tSum += frame.data[ty * frame.width + tx].toInt() and 0xFF
                tn++
            }
            if (tn >= 4) {
                val w = TARGET_WEIGHT * min(1.0, tn.toDouble() / TARGET_FULL_WEIGHT_PX)
                mean = (1 - w) * mean + w * tSum.toDouble() / tn
            }
        }
        return SceneMetering(mean, highlight)
    }

    private fun bounds(x0: Double, y0: Double, x1: Double, y1: Double, frame: LumaFrame): List<Int> =
        listOf(
            floor(x0).toInt().coerceIn(0, frame.width),
            floor(y0).toInt().coerceIn(0, frame.height),
            floor(x1).toInt().coerceIn(0, frame.width),
            floor(y1).toInt().coerceIn(0, frame.height),
        )

    private fun percentile(hist: IntArray, n: Int, q: Double): Double {
        val rank = q * (n - 1)
        var seen = 0
        for (v in 0 until 256) {
            seen += hist[v]
            if (seen > rank) return v.toDouble()
        }
        return 255.0
    }

    private fun meanAll(frame: LumaFrame) = ShutterPriorityExposure.meanLuma(frame)

    /**
     * Longest exposure time (ns) that keeps motion blur within [budgetOutputPx] output pixels,
     * for content moving at [speedPxPerS] (source px per second on the sensor) in a crop
     * [cropWidth] source px wide rendered [outputWidth] px wide; null when it does not move.
     */
    fun blurLimitNs(speedPxPerS: Double, cropWidth: Double, outputWidth: Int, budgetOutputPx: Double = BLUR_BUDGET_PX): Long? {
        if (speedPxPerS <= 1e-6 || cropWidth <= 0 || outputWidth <= 0) return null
        val allowedSourcePx = budgetOutputPx * cropWidth / outputWidth
        return (allowedSourcePx / speedPxPerS * 1e9).toLong()
    }

    /**
     * Speed (source px/s) at which [point] (world coordinates, see [CameraPose]) crosses the
     * sensor around [timestampNs]: its own motion plus the camera's, over [dtNs]. With [point]
     * null, the speed of the background at [center] (the camera's motion alone; roll moves
     * the crop's edges, [radius] px from its center).
     */
    fun imageSpeed(
        motion: CameraMotion,
        timestampNs: Long,
        center: Vec2,
        sensorCenter: Vec2,
        radius: Double,
        point: ((Long) -> Vec2)? = null,
        dtNs: Long = 50_000_000L,
    ): Double {
        val t0 = timestampNs - dtNs
        val a = motion.poseAt(t0)
        val b = motion.poseAt(timestampNs)
        val dt = dtNs * 1e-9
        return if (point != null) {
            val p0 = a.toImage(point(t0), sensorCenter)
            val p1 = b.toImage(point(timestampNs), sensorCenter)
            (p1 - p0).length() / dt
        } else {
            // Where the scene point under the crop center moves, plus roll at the crop's edge.
            val w = a.toWorld(center, sensorCenter)
            val moved = b.toImage(w, sensorCenter)
            hypot((moved - center).length(), abs(b.roll - a.roll) * radius) / dt
        }
    }

    /** Blur allowed in a frame of the output (output px): about a pixel, invisible at 1080p. */
    const val BLUR_BUDGET_PX = 1.5
}

/**
 * Smart shutter: the preferred exposure time follows how fast the subject crosses the sensor,
 * as short as needed to keep it sharp, as long as possible otherwise (less ISO noise). Reacts
 * quickly when things get faster (a pass), slowly when they calm down (no shutter pumping).
 */
class SmartShutter(
    private val shortestNs: Long,
    private val longestNs: Long,
    /** Fraction of the (log) gap closed per update towards a shorter time. */
    private val attack: Double = 0.6,
    /** ...and towards a longer time. */
    private val release: Double = 0.1,
) {
    var ns: Long = (shortestNs + longestNs) / 2
        private set

    /** New blur limit (null = nothing moves); returns the smoothed preferred time. */
    fun update(limitNs: Long?): Long {
        val goal = (limitNs ?: longestNs).coerceIn(shortestNs, longestNs).toDouble()
        val cur = ns.toDouble()
        val k = if (goal < cur) attack else release
        ns = exp(ln(cur) + (ln(goal) - ln(cur)) * k).toLong().coerceIn(shortestNs, longestNs)
        return ns
    }

    fun reset(startNs: Long) {
        ns = startNs.coerceIn(shortestNs, max(shortestNs, longestNs))
    }
}
