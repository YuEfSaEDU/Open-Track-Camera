package com.opentrack.tracking

import kotlin.math.hypot

/** 2D vector in pixel units. */
data class Vec2(val x: Double, val y: Double) {
    operator fun plus(o: Vec2) = Vec2(x + o.x, y + o.y)
    operator fun minus(o: Vec2) = Vec2(x - o.x, y - o.y)
    operator fun times(s: Double) = Vec2(x * s, y * s)
    operator fun div(s: Double) = Vec2(x / s, y / s)
    fun length(): Double = hypot(x, y)

    companion object {
        val ZERO = Vec2(0.0, 0.0)
    }
}

/**
 * Axis aligned box described by its center and size, in pixels. Image coordinates:
 * x grows to the right, y grows downwards, origin at the top-left corner.
 */
data class Box(val cx: Double, val cy: Double, val width: Double, val height: Double) {
    val center: Vec2 get() = Vec2(cx, cy)
    val left: Double get() = cx - width / 2
    val top: Double get() = cy - height / 2
    val right: Double get() = cx + width / 2
    val bottom: Double get() = cy + height / 2

    fun scaled(s: Double) = Box(cx * s, cy * s, width * s, height * s)
    fun movedTo(c: Vec2) = Box(c.x, c.y, width, height)
    fun contains(p: Vec2) = p.x in left..right && p.y in top..bottom

    fun iou(o: Box): Double {
        val ix = (minOf(right, o.right) - maxOf(left, o.left)).coerceAtLeast(0.0)
        val iy = (minOf(bottom, o.bottom) - maxOf(top, o.top)).coerceAtLeast(0.0)
        val inter = ix * iy
        val union = width * height + o.width * o.height - inter
        return if (union <= 0) 0.0 else inter / union
    }

    companion object {
        fun fromEdges(left: Double, top: Double, right: Double, bottom: Double) =
            Box((left + right) / 2, (top + bottom) / 2, right - left, bottom - top)
    }
}
