package com.opentrack.tracking

import kotlin.math.abs

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
 * Hybrid zoom: picks the camera zoom ratio of a multi-camera (whose HAL changes lens with it)
 * from the crop the framing wants, so a zoomed-in crop is taken from the telephoto lens, with
 * real detail, instead of being enlarged from the main lens.
 *
 * The telephoto sees only the middle [teleRatio]-th of the main lens's view. It is used while
 * the crop, with room around it ([enterRoom]: look-ahead stabilization and target motion),
 * fits in that window; it is left as soon as the crop, with less room ([exitRoom]), no longer
 * does (zooming out, the aircraft drifting off-center). The gap between the two and a minimum
 * time on each lens ([minHoldS]) keep it from flipping back and forth.
 */
class HybridZoom(
    val teleRatio: Double,
    private val width: Int,
    private val height: Int,
    private val enterRoom: Double = ENTER_ROOM,
    private val exitRoom: Double = 1.1,
    private val minHoldS: Double = 1.0,
    /** The crop must fit for this long before switching to the telephoto. */
    private val enterDelayS: Double = 0.3,
) {
    /** The zoom ratio to request: 1 (main lens) or [teleRatio]. */
    var ratio = 1.0
        private set

    private var lastSwitchNs = Long.MIN_VALUE
    private var fitsSinceNs = Long.MIN_VALUE

    private val teleWindow = ZoomMap(teleRatio, width, height).visible

    /** Whether [crop] (reference px), grown by [room], lies inside the telephoto's view. */
    fun fitsTele(crop: CropWindow, room: Double): Boolean {
        val e = crop.halfExtents()
        return abs(crop.cx - teleWindow.cx) + e.x * room <= teleWindow.width / 2 &&
            abs(crop.cy - teleWindow.cy) + e.y * room <= teleWindow.height / 2
    }

    /** New live crop (reference px) at [timestampNs]; returns the zoom ratio to request. */
    fun update(timestampNs: Long, crop: CropWindow): Double {
        val held = lastSwitchNs == Long.MIN_VALUE || (timestampNs - lastSwitchNs) * 1e-9 >= minHoldS
        if (ratio == 1.0) {
            if (fitsTele(crop, enterRoom)) {
                if (fitsSinceNs == Long.MIN_VALUE) fitsSinceNs = timestampNs
                if (held && (timestampNs - fitsSinceNs) * 1e-9 >= enterDelayS) switchTo(teleRatio, timestampNs)
            } else {
                fitsSinceNs = Long.MIN_VALUE
            }
        } else if (!fitsTele(crop, exitRoom)) {
            // Leaving is never delayed: the crop is about to need pixels the telephoto lacks.
            switchTo(1.0, timestampNs)
        }
        return ratio
    }

    fun reset() {
        ratio = 1.0
        lastSwitchNs = Long.MIN_VALUE
        fitsSinceNs = Long.MIN_VALUE
    }

    private fun switchTo(r: Double, timestampNs: Long) {
        ratio = r
        lastSwitchNs = timestampNs
        fitsSinceNs = Long.MIN_VALUE
    }

    companion object {
        /** Default [enterRoom]: the telephoto is used from a crop zoom of teleRatio x this. */
        const val ENTER_ROOM = 1.35
    }
}
