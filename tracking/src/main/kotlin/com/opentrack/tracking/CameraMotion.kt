package com.opentrack.tracking

/**
 * Where the image content has moved because the camera itself rotated.
 *
 * [offsetAt] returns, in source pixels, how far a point at infinity appears displaced at
 * [timestampNs] relative to an arbitrary (but fixed) reference orientation. Subtracting it
 * from an image position yields a "world" (orientation-stabilized) position; adding it
 * back maps a world position into the image of that frame.
 */
fun interface CameraMotion {
    fun offsetAt(timestampNs: Long): Vec2

    companion object {
        val NONE = CameraMotion { Vec2.ZERO }
    }
}
