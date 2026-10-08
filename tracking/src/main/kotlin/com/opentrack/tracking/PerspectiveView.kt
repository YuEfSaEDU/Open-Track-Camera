package com.opentrack.tracking

import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Renders a crop as a virtual camera turned towards the crop's center, instead of cutting a
 * flat rectangle out of the image.
 *
 * A wide phone lens is rectilinear: off its axis everything is stretched (by 1/cos² of the
 * angle along the radius) and keystoned. A flat crop that follows a locked target across the
 * sensor while the phone turns carries that changing stretch into the output — the scene
 * looks like a printed photo being bent in perspective. Rotating the camera about its center
 * is a pure homography (exact for distant scenes): looking along the crop center's ray with a
 * virtual camera of the same focal length shows the target exactly as when it was centered.
 *
 * Output coordinates u are source px relative to the output center (x right, y down, before
 * scaling to the output size); at the image center the view equals the flat crop. With
 * [strength] < 1 the lens is treated as longer (focal / strength), which blends towards the
 * flat crop (strength 0) — used when the corrected footprint would leave the captured region.
 */
class PerspectiveView(
    private val sourceWidth: Int,
    private val sourceHeight: Int,
    /** Focal length in source px; 0 or less disables the correction. */
    val focalPx: Double,
) {
    val enabled: Boolean get() = focalPx > 0

    /**
     * Homography (row-major 3x3) from output-local u (homogeneous (u.x, u.y, 1)) to source px.
     */
    fun homography(crop: CropWindow, strength: Double = 1.0): DoubleArray {
        val cx0 = sourceWidth / 2.0
        val cy0 = sourceHeight / 2.0
        val c = cos(crop.rotation)
        val s = sin(crop.rotation)
        if (!enabled || strength <= 1e-6) {
            // Flat crop: center + R(rotation) u.
            return doubleArrayOf(c, -s, crop.cx, s, c, crop.cy, 0.0, 0.0, 1.0)
        }
        val f = focalPx / strength.coerceAtMost(1.0)
        // View direction: the ray through the crop center.
        var dx = crop.cx - cx0
        var dy = crop.cy - cy0
        var dz = f
        val dn = sqrt(dx * dx + dy * dy + dz * dz)
        dx /= dn; dy /= dn; dz /= dn
        // View x axis: the image x axis made perpendicular to the view direction.
        var xx = 1 - dx * dx
        var xy = -dx * dy
        var xz = -dx * dz
        val xn = sqrt(xx * xx + xy * xy + xz * xz)
        xx /= xn; xy /= xn; xz /= xn
        // y = d × x (x right, y down, z forward is right-handed).
        val yx = dy * xz - dz * xy
        val yy = dz * xx - dx * xz
        val yz = dx * xy - dy * xx
        // Crop rotation about the view direction (as in the flat crop: axis x → (cos, sin)).
        val ax = doubleArrayOf(c * xx + s * yx, c * xy + s * yy, c * xz + s * yz)
        val ay = doubleArrayOf(-s * xx + c * yx, -s * xy + c * yy, -s * xz + c * yz)
        // ray = ax u.x + ay u.y + f d;  source = K ray (K = [[f, 0, cx0], [0, f, cy0], [0, 0, 1]]).
        val bz = doubleArrayOf(f * dx, f * dy, f * dz)
        fun row(k: Int, i: Int): Double {
            val col = when (i) { 0 -> ax; 1 -> ay; else -> bz }
            return when (k) {
                0 -> f * col[0] + cx0 * col[2]
                1 -> f * col[1] + cy0 * col[2]
                else -> col[2]
            }
        }
        val h = DoubleArray(9) { row(it / 3, it % 3) }
        // Normalize so the center maps with w = 1 (keeps float precision in the shader).
        val w = h[8]
        for (i in 0 until 9) h[i] /= w
        return h
    }

    /**
     * Largest strength in [0, 1] (to [steps] binary steps) for which the corrected crop, with
     * [margin] px to spare, stays inside [region] (source px).
     */
    fun fittingStrength(crop: CropWindow, region: Box, margin: Double = 1.0, steps: Int = 8): Double {
        if (!enabled) return 0.0
        fun fits(strength: Double): Boolean {
            val h = homography(crop, strength)
            for (k in 0 until 8) {
                // Corners and edge midpoints of the output.
                val ux = OUTLINE[2 * k] * crop.width / 2
                val uy = OUTLINE[2 * k + 1] * crop.height / 2
                val p = map(h, Vec2(ux, uy)) ?: return false
                if (p.x < region.left + margin || p.x > region.right - margin || p.y < region.top + margin || p.y > region.bottom - margin) return false
            }
            return true
        }
        if (fits(1.0)) return 1.0
        if (!fits(0.0)) return 0.0
        var lo = 0.0
        var hi = 1.0
        repeat(steps) {
            val mid = (lo + hi) / 2
            if (fits(mid)) lo = mid else hi = mid
        }
        return lo
    }

    /** Bounding box (source px) of the corrected view of [crop] at full strength. */
    fun footprint(crop: CropWindow): Box {
        val h = homography(crop)
        var x0 = Double.MAX_VALUE
        var y0 = Double.MAX_VALUE
        var x1 = -Double.MAX_VALUE
        var y1 = -Double.MAX_VALUE
        for (k in 0 until 8) {
            val p = map(h, Vec2(OUTLINE[2 * k] * crop.width / 2, OUTLINE[2 * k + 1] * crop.height / 2)) ?: continue
            x0 = min(x0, p.x); x1 = max(x1, p.x); y0 = min(y0, p.y); y1 = max(y1, p.y)
        }
        if (x0 > x1) return Box(crop.cx, crop.cy, crop.width, crop.height)
        return Box((x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0)
    }

    companion object {
        private val OUTLINE = doubleArrayOf(-1.0, -1.0, 0.0, -1.0, 1.0, -1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0, -1.0, 1.0, -1.0, 0.0)

        /** Applies a row-major homography; null behind the camera. */
        fun map(h: DoubleArray, u: Vec2): Vec2? {
            val w = h[6] * u.x + h[7] * u.y + h[8]
            if (w <= 1e-9) return null
            return Vec2((h[0] * u.x + h[1] * u.y + h[2]) / w, (h[3] * u.x + h[4] * u.y + h[5]) / w)
        }
    }
}
