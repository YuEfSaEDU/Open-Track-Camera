package com.opentrack.tracking

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.random.Random

/**
 * Estimates camera motion from the video itself, like the point-tracking stage of desktop
 * stabilizers: Shi-Tomasi corners spread over a grid, pyramidal Lucas-Kanade flow to the next
 * frame, and a RANSAC fit of a rigid transform (shift + roll about the image center). The
 * tracked aircraft is excluded so its own motion is not mistaken for camera shake.
 *
 * Produces a [CameraPose] history, so it can stand in for (or complement) the gyroscope.
 * With no usable background texture (clear sky) it falls back to a decaying constant-velocity
 * prediction and reports [Step.reliable] = false.
 *
 * Not thread safe: call [process] from the analysis thread; [poseAt] is synchronized.
 */
class VisualMotionEstimator(
    private val gridCols: Int = 16,
    private val gridRows: Int = 9,
    /** Half size of the Lucas-Kanade window (window = 2r + 1). */
    private val radius: Int = 5,
    private val levels: Int = 3,
    private val minInliers: Int = 8,
    private val ransacIterations: Int = 80,
    /** RANSAC inlier threshold, analysis pixels. */
    private val inlierThreshold: Double = 0.75,
    historyCapacity: Int = 1024,
) : CameraMotion, MotionReliability, CorrectableMotion {

    /** Motion between the previous and the current frame, source px / radians. */
    data class Step(val shift: Vec2, val rotation: Double, val features: Int, val inliers: Int, val reliable: Boolean)

    var lastStep: Step? = null
        private set

    private var prev: Pyramid? = null
    private var lastGood = Step(Vec2.ZERO, 0.0, 0, 0, false)
    private var failures = 0
    private val rnd = Random(7)

    // Pose history.
    private val times = LongArray(historyCapacity)
    private val offX = DoubleArray(historyCapacity)
    private val offY = DoubleArray(historyCapacity)
    private val rolls = DoubleArray(historyCapacity)
    private val reliable = BooleanArray(historyCapacity)
    private var head = -1
    private var count = 0
    private var poseOffset = Vec2.ZERO
    private var poseRoll = 0.0
    private val lock = Any()

    fun reset() {
        prev = null
        failures = 0
        lastGood = Step(Vec2.ZERO, 0.0, 0, 0, false)
        synchronized(lock) {
            head = -1
            count = 0
            poseOffset = Vec2.ZERO
            poseRoll = 0.0
        }
    }

    /**
     * Processes the next frame. [exclude] (source px) masks the tracked object in the previous
     * frame. Returns the motion from the previous frame (null for the first frame).
     */
    fun process(frame: LumaFrame, exclude: Box? = null): Step? {
        val cur = Pyramid(frame, levels)
        val p = prev
        prev = cur
        if (p == null) {
            record(frame.timestampNs, true)
            return null
        }
        val s = frame.scale
        val step = estimate(p, cur, exclude?.scaled(1 / s), s)
        lastStep = step
        // Compose: image_k = c + R(dθ)(image_{k-1} - c) + d  ⇒  φ += dθ, O = R(dθ) O + d.
        synchronized(lock) {
            poseRoll += step.rotation
            poseOffset = poseOffset.rotated(step.rotation) + step.shift
        }
        record(frame.timestampNs, step.reliable)
        return step
    }

    override fun isReliableAt(timestampNs: Long): Boolean = synchronized(lock) {
        if (count == 0) return true
        val size = times.size
        for (i in 0 until count) {
            val k = (head - i + size) % size
            if (times[k] <= timestampNs) return reliable[k]
        }
        return reliable[(head - count + 1 + size) % size]
    }

    override fun correct(timestampNs: Long, offsetDelta: Vec2) = synchronized(lock) {
        if (count == 0 || times[head] != timestampNs) return
        poseOffset += offsetDelta
        offX[head] = poseOffset.x
        offY[head] = poseOffset.y
    }

    private fun estimate(p: Pyramid, cur: Pyramid, exclude: Box?, scale: Double): Step {
        val w = p.w[0]
        val h = p.h[0]
        val guess = lastGood.shift / scale
        val features = detectFeatures(p, exclude)
        val src = ArrayList<Vec2>(features.size)
        val dst = ArrayList<Vec2>(features.size)
        for (f in features) {
            val d = lucasKanade(p, cur, f, guess) ?: continue
            src += f
            dst += f + d
        }
        val fit = if (src.size >= minInliers) ransacRigid(src, dst) else null
        if (fit == null || fit.inliers < minInliers) {
            // No trustworthy background (e.g. clear sky): assume the recent motion continues, fading out.
            failures++
            val decay = Math.pow(0.85, failures.toDouble())
            return Step(lastGood.shift * decay, lastGood.rotation * decay, features.size, fit?.inliers ?: 0, false)
        }
        failures = 0
        // Absolute rigid fit q = R p + t  →  centered form q = c + R (p - c) + d.
        val c = Vec2(w / 2.0, h / 2.0)
        val d = Vec2(fit.tx, fit.ty) - c + c.rotated(fit.theta)
        val step = Step(d * scale, fit.theta, features.size, fit.inliers, true)
        lastGood = step
        return step
    }

    override fun poseAt(timestampNs: Long): CameraPose = synchronized(lock) {
        if (count == 0) return CameraPose.IDENTITY
        val size = times.size
        val newest = head
        if (timestampNs >= times[newest]) {
            return CameraPose(Vec2(offX[newest], offY[newest]), rolls[newest])
        }
        val oldest = (head - count + 1 + size) % size
        if (timestampNs <= times[oldest]) return CameraPose(Vec2(offX[oldest], offY[oldest]), rolls[oldest])
        var lo = 0
        var hi = count - 1
        while (hi - lo > 1) {
            val mid = (lo + hi) ushr 1
            if (times[(oldest + mid) % size] <= timestampNs) lo = mid else hi = mid
        }
        val a = (oldest + lo) % size
        val b = (oldest + hi) % size
        val span = (times[b] - times[a]).toDouble()
        val k = if (span <= 0) 0.0 else (timestampNs - times[a]) / span
        return CameraPose(
            Vec2(offX[a] + (offX[b] - offX[a]) * k, offY[a] + (offY[b] - offY[a]) * k),
            rolls[a] + (rolls[b] - rolls[a]) * k,
        )
    }

    private fun record(t: Long, ok: Boolean) = synchronized(lock) {
        head = (head + 1) % times.size
        times[head] = t
        offX[head] = poseOffset.x
        offY[head] = poseOffset.y
        rolls[head] = poseRoll
        reliable[head] = ok
        if (count < times.size) count++
    }

    // ---------------------------------------------------------------- features

    private var ixx = DoubleArray(0)
    private var iyy = DoubleArray(0)
    private var ixy = DoubleArray(0)

    /** Best Shi-Tomasi corner per grid cell (min eigenvalue of the 7x7 structure tensor). */
    private fun detectFeatures(p: Pyramid, exclude: Box?): List<Vec2> {
        val w = p.w[0]
        val h = p.h[0]
        val gx = p.gx[0]
        val gy = p.gy[0]
        val stride = w + 1
        val n = stride * (h + 1)
        if (ixx.size != n) {
            ixx = DoubleArray(n)
            iyy = DoubleArray(n)
            ixy = DoubleArray(n)
        }
        for (y in 0 until h) {
            var sxx = 0.0
            var syy = 0.0
            var sxy = 0.0
            for (x in 0 until w) {
                val i = y * w + x
                val a = gx[i].toDouble()
                val b = gy[i].toDouble()
                sxx += a * a
                syy += b * b
                sxy += a * b
                val o = (y + 1) * stride + x + 1
                ixx[o] = ixx[o - stride] + sxx
                iyy[o] = iyy[o - stride] + syy
                ixy[o] = ixy[o - stride] + sxy
            }
        }
        val margin = radius * (1 shl (levels - 1)) + 4
        val half = 3
        val ex = exclude?.let { Box(it.cx, it.cy, it.width * 1.6 + 8, it.height * 1.6 + 8) }
        val cellW = (w - 2 * margin).toDouble() / gridCols
        val cellH = (h - 2 * margin).toDouble() / gridRows
        val best = DoubleArray(gridCols * gridRows)
        val bestX = IntArray(gridCols * gridRows)
        val bestY = IntArray(gridCols * gridRows)
        var globalMax = 0.0
        var y = margin
        while (y < h - margin) {
            var x = margin
            while (x < w - margin) {
                if (ex == null || !ex.contains(Vec2(x.toDouble(), y.toDouble()))) {
                    val x0 = x - half
                    val y0 = y - half
                    val x1 = x + half + 1
                    val y1 = y + half + 1
                    val a = rect(ixx, stride, x0, y0, x1, y1)
                    val c = rect(iyy, stride, x0, y0, x1, y1)
                    val b = rect(ixy, stride, x0, y0, x1, y1)
                    val lambda = (a + c) / 2 - sqrt(((a - c) / 2) * ((a - c) / 2) + b * b)
                    val cell = min(gridRows - 1, ((y - margin) / cellH).toInt()) * gridCols + min(gridCols - 1, ((x - margin) / cellW).toInt())
                    if (lambda > best[cell]) {
                        best[cell] = lambda
                        bestX[cell] = x
                        bestY[cell] = y
                    }
                    if (lambda > globalMax) globalMax = lambda
                }
                x += 2
            }
            y += 2
        }
        val threshold = max(0.01 * globalMax, MIN_CORNER_STRENGTH)
        val out = ArrayList<Vec2>()
        for (i in best.indices) if (best[i] > threshold) out += Vec2(bestX[i].toDouble(), bestY[i].toDouble())
        return out
    }

    private fun rect(ii: DoubleArray, stride: Int, x0: Int, y0: Int, x1: Int, y1: Int): Double =
        ii[y1 * stride + x1] - ii[y0 * stride + x1] - ii[y1 * stride + x0] + ii[y0 * stride + x0]

    // ---------------------------------------------------------------- Lucas-Kanade

    private val tpl = FloatArray((2 * 7 + 1) * (2 * 7 + 1))
    private val tgx = FloatArray(tpl.size)
    private val tgy = FloatArray(tpl.size)

    /** Displacement of [pt] (level-0 px) from [a] to [b], or null when it cannot be tracked. */
    private fun lucasKanade(a: Pyramid, b: Pyramid, pt: Vec2, guess: Vec2): Vec2? {
        val r = radius
        var dx = guess.x / (1 shl (levels - 1))
        var dy = guess.y / (1 shl (levels - 1))
        for (level in levels - 1 downTo 0) {
            val k = 1.0 / (1 shl level)
            val px = pt.x * k
            val py = pt.y * k
            val w = a.w[level]
            val h = a.h[level]
            if (px - r - 1 < 0 || py - r - 1 < 0 || px + r + 1 >= w || py + r + 1 >= h) return null
            var gxx = 0.0
            var gyy = 0.0
            var gxy = 0.0
            var i = 0
            for (j in -r..r) for (q in -r..r) {
                val sx = px + q
                val sy = py + j
                tpl[i] = bilinear(a.img[level], w, sx, sy)
                val ix = bilinear(a.gx[level], w, sx, sy)
                val iy = bilinear(a.gy[level], w, sx, sy)
                tgx[i] = ix
                tgy[i] = iy
                gxx += ix * ix
                gyy += iy * iy
                gxy += ix * iy
                i++
            }
            val det = gxx * gyy - gxy * gxy
            // Smallest eigenvalue of the gradient matrix: reject flat or edge-only windows (aperture problem).
            val lambdaMin = (gxx + gyy) / 2 - sqrt(((gxx - gyy) / 2) * ((gxx - gyy) / 2) + gxy * gxy)
            if (lambdaMin < MIN_LK_EIGEN * (2 * r + 1) * (2 * r + 1)) return null
            for (iter in 0 until 12) {
                val cx = px + dx
                val cy = py + dy
                if (cx - r - 1 < 0 || cy - r - 1 < 0 || cx + r + 1 >= w || cy + r + 1 >= h) return null
                var bx = 0.0
                var by = 0.0
                i = 0
                for (j in -r..r) for (q in -r..r) {
                    val e = tpl[i] - bilinear(b.img[level], w, cx + q, cy + j)
                    bx += e * tgx[i]
                    by += e * tgy[i]
                    i++
                }
                val ux = (gyy * bx - gxy * by) / det
                val uy = (gxx * by - gxy * bx) / det
                dx += ux
                dy += uy
                if (abs(ux) < 0.01 && abs(uy) < 0.01) break
            }
            if (level > 0) {
                dx *= 2
                dy *= 2
            }
        }
        return Vec2(dx, dy)
    }

    private fun bilinear(img: FloatArray, w: Int, x: Double, y: Double): Float {
        val x0 = x.toInt()
        val y0 = y.toInt()
        val fx = (x - x0).toFloat()
        val fy = (y - y0).toFloat()
        val i = y0 * w + x0
        val top = img[i] + (img[i + 1] - img[i]) * fx
        val bottom = img[i + w] + (img[i + w + 1] - img[i + w]) * fx
        return top + (bottom - top) * fy
    }

    // ---------------------------------------------------------------- RANSAC

    private class Rigid(val theta: Double, val tx: Double, val ty: Double, val inliers: Int)

    private fun ransacRigid(src: List<Vec2>, dst: List<Vec2>): Rigid? {
        val n = src.size
        var bestInliers = 0
        var bestMask = BooleanArray(n)
        val mask = BooleanArray(n)
        repeat(ransacIterations) {
            val i = rnd.nextInt(n)
            val j = rnd.nextInt(n)
            if (i == j) return@repeat
            val vp = src[j] - src[i]
            if (vp.length() < 20) return@repeat
            val vq = dst[j] - dst[i]
            val theta = atan2(vq.y, vq.x) - atan2(vp.y, vp.x)
            val c = cos(theta)
            val s = sin(theta)
            val tx = dst[i].x - (c * src[i].x - s * src[i].y)
            val ty = dst[i].y - (s * src[i].x + c * src[i].y)
            var inl = 0
            for (k in 0 until n) {
                val ex = c * src[k].x - s * src[k].y + tx - dst[k].x
                val ey = s * src[k].x + c * src[k].y + ty - dst[k].y
                mask[k] = hypot(ex, ey) < inlierThreshold
                if (mask[k]) inl++
            }
            if (inl > bestInliers) {
                bestInliers = inl
                bestMask = mask.copyOf()
            }
        }
        if (bestInliers < 2) return null
        // Least-squares refinement (2D Procrustes) on the inliers.
        var mpx = 0.0; var mpy = 0.0; var mqx = 0.0; var mqy = 0.0
        for (k in 0 until n) if (bestMask[k]) {
            mpx += src[k].x; mpy += src[k].y; mqx += dst[k].x; mqy += dst[k].y
        }
        mpx /= bestInliers; mpy /= bestInliers; mqx /= bestInliers; mqy /= bestInliers
        var sCross = 0.0
        var sDot = 0.0
        for (k in 0 until n) if (bestMask[k]) {
            val ax = src[k].x - mpx
            val ay = src[k].y - mpy
            val bx = dst[k].x - mqx
            val by = dst[k].y - mqy
            sCross += ax * by - ay * bx
            sDot += ax * bx + ay * by
        }
        val theta = atan2(sCross, sDot)
        val c = cos(theta)
        val s = sin(theta)
        return Rigid(theta, mqx - (c * mpx - s * mpy), mqy - (s * mpx + c * mpy), bestInliers)
    }

    // ---------------------------------------------------------------- pyramid

    private class Pyramid(frame: LumaFrame, levels: Int) {
        val w = IntArray(levels)
        val h = IntArray(levels)
        val img = Array(levels) { FloatArray(0) }
        val gx = Array(levels) { FloatArray(0) }
        val gy = Array(levels) { FloatArray(0) }

        init {
            w[0] = frame.width
            h[0] = frame.height
            img[0] = FloatArray(frame.width * frame.height) { (frame.data[it].toInt() and 0xFF).toFloat() }
            for (l in 1 until levels) {
                val pw = w[l - 1]
                val nw = pw / 2
                val nh = h[l - 1] / 2
                val src = img[l - 1]
                val out = FloatArray(nw * nh)
                for (y in 0 until nh) for (x in 0 until nw) {
                    val i = 2 * y * pw + 2 * x
                    out[y * nw + x] = (src[i] + src[i + 1] + src[i + pw] + src[i + pw + 1]) * 0.25f
                }
                w[l] = nw
                h[l] = nh
                img[l] = out
            }
            for (l in 0 until levels) {
                val ww = w[l]
                val hh = h[l]
                val im = img[l]
                val ax = FloatArray(ww * hh)
                val ay = FloatArray(ww * hh)
                for (y in 1 until hh - 1) for (x in 1 until ww - 1) {
                    val i = y * ww + x
                    ax[i] = (im[i + 1] - im[i - 1]) * 0.5f
                    ay[i] = (im[i + ww] - im[i - ww]) * 0.5f
                }
                gx[l] = ax
                gy[l] = ay
            }
        }
    }

    private companion object {
        /** Minimum structure tensor eigenvalue (7x7 window) for a usable corner. */
        const val MIN_CORNER_STRENGTH = 49.0 * 0.5

        /** Minimum mean squared gradient (per window pixel) along the weakest direction. */
        const val MIN_LK_EIGEN = 0.5
    }
}
