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
 * from the crop the framing wants, so the crop is always taken from the longest lens that
 * sees it: the wide angle when the aircraft is close (a crop wider than the main lens's view),
 * the main lens, the telephoto when it is far (real detail instead of enlarging).
 *
 * [levels] are the zoom ratios to choose from relative to the reference space (ascending, the
 * first 1: the whole reference frame); level k sees the middle 1/levels[k] of it. A longer
 * lens is used once the crop, with room around it ([enterRoom]: look-ahead stabilization and
 * target motion), has fitted in its view for [enterDelayS]; it is left as soon as the crop,
 * with less room ([exitRoom]), no longer does (zooming out, the aircraft drifting off-center),
 * for the longest lens it still fits. The gap between the two and a minimum time on each lens
 * ([minHoldS]) keep it from flipping back and forth.
 */
class HybridZoom(
    val levels: List<Double>,
    private val width: Int,
    private val height: Int,
    private val enterRoom: Double = ENTER_ROOM,
    private val exitRoom: Double = 1.1,
    private val minHoldS: Double = 1.0,
    /** The crop must fit for this long before switching to a longer lens. */
    private val enterDelayS: Double = 0.3,
    /** The level to start on (e.g. new [levels] for the same lenses while on one of them). */
    startLevel: Int = 0,
) {
    /** Two levels: the reference (1) and a telephoto at [teleRatio]. */
    constructor(teleRatio: Double, width: Int, height: Int) : this(listOf(1.0, teleRatio), width, height)

    init {
        require(levels.isNotEmpty() && levels.first() == 1.0 && levels.zipWithNext().all { (a, b) -> b > a }) { "levels must rise from 1: $levels" }
    }

    /** The longest lens's ratio. */
    val teleRatio: Double get() = levels.last()

    /** Index of the current level in [levels]. */
    var level = startLevel.coerceIn(0, levels.size - 1)
        private set

    /** The zoom ratio to request (relative to the reference space). */
    val ratio: Double get() = levels[level]

    private var lastSwitchNs = Long.MIN_VALUE
    private var fitsSinceNs = Long.MIN_VALUE

    private val windows = levels.map { ZoomMap(it, width, height).visible }

    /** Whether [crop] (reference px), grown by [room], lies inside the view of level [k]. */
    fun fits(k: Int, crop: CropWindow, room: Double): Boolean {
        if (k == 0) return true // the whole reference frame: framing keeps the crop in it
        val win = windows[k]
        val e = crop.halfExtents()
        return abs(crop.cx - win.cx) + e.x * room <= win.width / 2 &&
            abs(crop.cy - win.cy) + e.y * room <= win.height / 2
    }

    /** Whether [crop] fits the longest lens's view with [room]. */
    fun fitsTele(crop: CropWindow, room: Double): Boolean = fits(levels.size - 1, crop, room)

    /** New live crop (reference px) at [timestampNs]; returns the zoom ratio to request. */
    fun update(timestampNs: Long, crop: CropWindow): Double {
        if (!fits(level, crop, exitRoom)) {
            // Leaving is never delayed: the crop is about to need pixels this lens lacks.
            var k = level - 1
            while (k > 0 && !fits(k, crop, exitRoom)) k--
            switchTo(k, timestampNs)
            return ratio
        }
        val held = lastSwitchNs == Long.MIN_VALUE || (timestampNs - lastSwitchNs) * 1e-9 >= minHoldS
        if (level + 1 < levels.size && fits(level + 1, crop, enterRoom)) {
            if (fitsSinceNs == Long.MIN_VALUE) fitsSinceNs = timestampNs
            if (held && (timestampNs - fitsSinceNs) * 1e-9 >= enterDelayS) switchTo(level + 1, timestampNs)
        } else {
            fitsSinceNs = Long.MIN_VALUE
        }
        return ratio
    }

    fun reset() {
        level = 0
        lastSwitchNs = Long.MIN_VALUE
        fitsSinceNs = Long.MIN_VALUE
    }

    private fun switchTo(k: Int, timestampNs: Long) {
        level = k
        lastSwitchNs = timestampNs
        fitsSinceNs = Long.MIN_VALUE
    }

    companion object {
        /** Default [enterRoom]: a lens is used from a crop zoom of its ratio x this. */
        const val ENTER_ROOM = 1.35
    }
}
