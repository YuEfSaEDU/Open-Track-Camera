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

fun Vec2.rotated(angle: Double): Vec2 {
    if (angle == 0.0) return this
    val c = cos(angle)
    val s = sin(angle)
    return Vec2(c * x - s * y, s * x + c * y)
}
