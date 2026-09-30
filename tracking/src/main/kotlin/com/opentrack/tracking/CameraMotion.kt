package com.opentrack.tracking

import kotlin.math.cos
import kotlin.math.sin

/**
 * Camera orientation relative to a fixed (arbitrary) reference, expressed as what it does to
 * the image of distant objects: a rotation by [roll] radians about the image center followed
 * by a shift of [offset] source pixels. Positive roll turns the image clockwise on screen
 * (image y axis points down).
 *
 * "World" coordinates are image coordinates with this pose removed: a static distant object
 * keeps the same world position while the camera shakes, pans or rolls.
 */
data class CameraPose(val offset: Vec2, val roll: Double) {
    fun toImage(world: Vec2, center: Vec2): Vec2 = center + (world - center).rotated(roll) + offset
    fun toWorld(image: Vec2, center: Vec2): Vec2 = center + (image - center - offset).rotated(-roll)

    companion object {
        val IDENTITY = CameraPose(Vec2.ZERO, 0.0)
    }
}

/** Source of [CameraPose] over time (gyroscope, visual motion estimation, or none). */
fun interface CameraMotion {
    fun poseAt(timestampNs: Long): CameraPose

    companion object {
        val NONE = CameraMotion { CameraPose.IDENTITY }
    }
}

/**
 * Optional capability of a [CameraMotion] whose estimate can be missing on some frames (e.g.
 * image-based motion over a featureless sky). A gyroscope is always reliable.
 */
interface MotionReliability {
    fun isReliableAt(timestampNs: Long): Boolean
}

/**
 * Optional capability: lets the tracker fix the latest pose when it knows better. Over a
 * featureless sky the aircraft is the only reference, so the tracker attributes its unexpected
 * image motion to the camera and keeps the aircraft's world path smooth.
 */
interface CorrectableMotion {
    /** Adds [offsetDelta] to the pose of the latest frame (only if [timestampNs] is that frame). */
    fun correct(timestampNs: Long, offsetDelta: Vec2)
}

fun Vec2.rotated(angle: Double): Vec2 {
    if (angle == 0.0) return this
    val c = cos(angle)
    val s = sin(angle)
    return Vec2(c * x - s * y, s * x + c * y)
}
