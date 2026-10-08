package com.opentrack.tracking

import kotlin.math.abs
import kotlin.math.exp
import kotlin.math.ln

/**
 * Maps a frame taken at camera zoom ratio [zoom] (the HAL's centered crop, or another lens of a
 * multi-camera aligned to it) to the reference space: the main lens at 1x, [width] x [height]
 * px. Tracking, stabilization and framing all run in the reference space, so their state stays
 * continuous when the zoom ratio, and with it the lens, changes; only reading pixels from a
 * frame (and drawing over it) needs this mapping.
 */
data class ZoomMap(val zoom: Double, val width: Int, val height: Int) {
    private val cx = width / 2.0
    private val cy = height / 2.0

    fun toStream(p: Vec2): Vec2 = Vec2(cx + (p.x - cx) * zoom, cy + (p.y - cy) * zoom)

    fun toRef(p: Vec2): Vec2 = Vec2(cx + (p.x - cx) / zoom, cy + (p.y - cy) / zoom)

    fun toStream(b: Box): Box = toStream(b.center).let { Box(it.x, it.y, b.width * zoom, b.height * zoom) }

    fun toRef(b: Box): Box = toRef(b.center).let { Box(it.x, it.y, b.width / zoom, b.height / zoom) }

    /** The part of the reference frame the frame shows. */
    val visible: Box get() = Box(cx, cy, width / zoom, height / zoom)

    companion object {
        fun identity(width: Int, height: Int) = ZoomMap(1.0, width, height)
    }
}

/** Intersection of two boxes, or null when they do not overlap. */
fun Box.intersect(o: Box): Box? {
    val l = maxOf(left, o.left)
    val t = maxOf(top, o.top)
    val r = minOf(right, o.right)
    val b = minOf(bottom, o.bottom)
    return if (r <= l || b <= t) null else Box.fromEdges(l, t, r, b)
}

/**
 * Hybrid zoom: drives the camera zoom ratio of a multi-camera (whose HAL changes lens with it)
 * from the crop, the way the phone's own camera zooms: the ratio grows with the crop zoom on
 * the main lens (the camera's own zoom, which keeps more detail than enlarging a crop of the
 * whole frame), and past a crop size it moves on to the telephoto.
 *
 * The camera frame at ratio z (relative to the reference space) sees the middle 1/z of the
 * reference frame; the ratio is chosen so that the crop, grown by [headroom] (room for the
 * look-ahead stabilization and target motion), stays inside it, also when the crop is off
 * center. Tracking, framing and stabilization run in the reference space and every frame is
 * mapped with its own ratio, so the picture stays continuous however the ratio moves.
 *
 * [tele]: the telephoto's ratio (null: main lens only). The main lens is held below
 * [mainLimit] x [tele] (the HAL's own switch point is near [tele]: hovering around it would
 * flip lenses); once the crop fits the telephoto's view with [headroom], the ratio steps to
 * [tele] or beyond, and back below once it no longer fits with [exitRoom], holding each lens
 * for at least [minHoldS].
 */
class HybridZoom(
    private val width: Int,
    private val height: Int,
    val tele: Double?,
    /** Highest ratio to request (relative). */
    private val maxRatio: Double,
    private val headroom: Double = HEADROOM,
    private val exitRoom: Double = 1.1,
    private val mainLimit: Double = 0.9,
    private val minHoldS: Double = 1.0,
    /** The crop must fit the telephoto for this long before switching to it. */
    private val enterDelayS: Double = 0.3,
    /** Start on the telephoto (new settings while on it). */
    startOnTele: Boolean = false,
) {
    /** The zoom ratio to request, relative to the reference space (1 = its whole frame). */
    var ratio = 1.0
        private set

    /** Whether the telephoto is in use. */
    var onTele = startOnTele && tele != null
        private set

    private var lastSwitchNs = Long.MIN_VALUE
    private var fitsSinceNs = Long.MIN_VALUE

    /** Largest ratio whose (centered) view still contains [crop] grown by [room]. */
    fun coverRatio(crop: CropWindow, room: Double): Double {
        val e = crop.halfExtents()
        val rx = width / (2 * (abs(crop.cx - width / 2.0) + e.x * room))
        val ry = height / (2 * (abs(crop.cy - height / 2.0) + e.y * room))
        return minOf(rx, ry).coerceAtLeast(1.0)
    }

    /** New live crop (reference px) at [timestampNs]; returns the zoom ratio to request. */
    fun update(timestampNs: Long, crop: CropWindow): Double {
        val need = coverRatio(crop, headroom)
        val t = tele
        if (t == null) return set(need.coerceAtMost(maxRatio))
        val held = lastSwitchNs == Long.MIN_VALUE || (timestampNs - lastSwitchNs) * 1e-9 >= minHoldS
        if (onTele) {
            // Leaving is never delayed: the crop is about to need pixels the telephoto lacks.
            if (coverRatio(crop, exitRoom) < t) {
                onTele = false
                lastSwitchNs = timestampNs
                fitsSinceNs = Long.MIN_VALUE
            }
        } else if (need >= t) {
            if (fitsSinceNs == Long.MIN_VALUE) fitsSinceNs = timestampNs
            if (held && (timestampNs - fitsSinceNs) * 1e-9 >= enterDelayS) {
                onTele = true
                lastSwitchNs = timestampNs
            }
        } else {
            fitsSinceNs = Long.MIN_VALUE
        }
        return set(if (onTele) need.coerceIn(t, maxOf(t, maxRatio)) else need.coerceAtMost(t * mainLimit))
    }

    fun reset() {
        ratio = 1.0
        onTele = false
        lastSwitchNs = Long.MIN_VALUE
        fitsSinceNs = Long.MIN_VALUE
    }

    /** Steps of 1% (no capture request per tiny change). */
    private fun set(r: Double): Double {
        ratio = exp(Math.round(ln(r) / STEP) * STEP)
        return ratio
    }

    companion object {
        /** Default [headroom]: the crop is at most 1 / this of the camera frame. */
        const val HEADROOM = 1.35
        private const val STEP = 0.01
    }
}
