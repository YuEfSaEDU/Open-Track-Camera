package com.opentrack.tracking

import kotlin.math.cos
import kotlin.math.sin

/**
 * Rolling shutter timing of the image: sensor rows are read out one after another over
 * [skewNs], so each part of the frame is captured at a slightly different time. With the
 * camera turning, that shears and wobbles the picture ("jello"), which becomes very visible
 * once the frame is stabilized and cropped (zoomed).
 *
 * [CameraMotion.poseAt] of a frame's timestamp refers to its middle row (the app folds half
 * the skew into the gyro time offset); a point at image position p was captured at
 * `t + timeOffsetNs(p)`.
 *
 * @param readoutDir unit vector (image axes) along which the readout progresses; depends on
 *   the sensor orientation relative to the displayed image, see [readoutDirection].
 * @param extentPx image size along [readoutDir] (source px).
 */
data class RollingShutter(val skewNs: Long, val readoutDir: Vec2, val extentPx: Double) {
    /** Readout position of [p] from -0.5 (first row) to 0.5 (last row). */
    fun fraction(p: Vec2, center: Vec2): Double = ((p.x - center.x) * readoutDir.x + (p.y - center.y) * readoutDir.y) / extentPx

    fun timeOffsetNs(p: Vec2, center: Vec2): Long = (skewNs * fraction(p, center)).toLong()

    /** World position of a point seen at image position [image] in the frame of [timestampNs]. */
    fun toWorld(motion: CameraMotion, image: Vec2, timestampNs: Long, center: Vec2): Vec2 =
        motion.poseAt(timestampNs + timeOffsetNs(image, center)).toWorld(image, center)

    /** Image position (in the raw, uncorrected frame of [timestampNs]) where [world] was captured. */
    fun toImage(motion: CameraMotion, world: Vec2, timestampNs: Long, center: Vec2): Vec2 {
        // Fixed point: the row depends on the position. Shifts are a few px, two passes are exact.
        var p = motion.poseAt(timestampNs).toImage(world, center)
        repeat(2) { p = motion.poseAt(timestampNs + timeOffsetNs(p, center)).toImage(world, center) }
        return p
    }

    /**
     * Rigid corrections sampled at [n] readout positions from -0.5 to 0.5. For a point p' of
     * the ideal (global shutter, pose at [timestampNs]) image at readout position f, the raw
     * frame holds it at `center + R(angle(f)) (p' - center) + shift(f)`; the renderer
     * interpolates between the samples. Returned as n triples (shiftX, shiftY, angle).
     */
    fun corrections(motion: CameraMotion, timestampNs: Long, n: Int): FloatArray {
        val out = FloatArray(n * 3)
        val ref = motion.poseAt(timestampNs)
        for (k in 0 until n) {
            val f = if (n == 1) 0.0 else -0.5 + k.toDouble() / (n - 1)
            val pose = motion.poseAt(timestampNs + (skewNs * f).toLong())
            // p = c + R(r_k)(R(-r_0)(p' - c - o_0)) + o_k = c + R(dr)(p' - c) + (o_k - R(dr) o_0)
            val dr = pose.roll - ref.roll
            val c = cos(dr)
            val s = sin(dr)
            val o0 = ref.offset
            out[k * 3] = (pose.offset.x - (c * o0.x - s * o0.y)).toFloat()
            out[k * 3 + 1] = (pose.offset.y - (s * o0.x + c * o0.y)).toFloat()
            out[k * 3 + 2] = dr.toFloat()
        }
        return out
    }

    companion object {
        /**
         * Readout direction in the displayed image. The sensor reads its rows top to bottom in
         * its own orientation; the image shown is the sensor image turned clockwise by
         * [sensorOrientationDegrees] (to the device's natural orientation) and then by the
         * display rotation, as FrameRenderer does ([displayQuarterTurns] = Display rotation / 90).
         */
        fun readoutDirection(sensorOrientationDegrees: Int, displayQuarterTurns: Int): Vec2 {
            // Sensor "down" turned clockwise (y down) by the sensor orientation: natural orientation.
            val n = Vec2(0.0, 1.0).rotated(Math.toRadians(sensorOrientationDegrees.toDouble()))
            // FrameRenderer.imgToTex maps image -> natural; invert it for directions.
            val d = when (displayQuarterTurns and 3) {
                1 -> Vec2(n.y, -n.x)
                2 -> Vec2(-n.x, -n.y)
                3 -> Vec2(-n.y, n.x)
                else -> n
            }
            return Vec2(Math.round(d.x).toDouble(), Math.round(d.y).toDouble())
        }
    }
}
