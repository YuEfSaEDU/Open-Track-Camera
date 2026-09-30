package com.opentrack.tracking

import kotlin.math.max
import kotlin.math.min

data class DetectionPlanConfig(
    /** While a target exists, this many zoomed runs follow each full-frame run. */
    val roiRunsPerFullFrame: Int = 2,
    /** Region side as a multiple of the tracked object's size... */
    val sizeFactor: Double = 5.0,
    /** ...but never smaller than this fraction of the frame height (keeps the aircraft's surroundings). */
    val minSideFraction: Double = 0.15,
    /** Extra side per second the target has not been measured (the search area grows while unseen). */
    val growthPerSFraction: Double = 0.25,
    /** Stop zooming in on a target lost for this long and scan instead. */
    val maxLostNs: Long = 3_000_000_000L,
    /**
     * Without a target, runs between full frames scan the frame in tiles of this fraction of
     * the frame height (0 = no scan), so a small aircraft can be found in the first place.
     */
    val scanTileFraction: Double = 0.5,
    /** Scanning: tiles per full-frame run. */
    val scanRunsPerFullFrame: Int = 2,
)

/**
 * Decides which part of the frame the next neural detector run looks at.
 *
 * The detector sees its input at 320 x 320: a whole 4K frame shrinks a distant fighter to a
 * few pixels, too small to recognize. While a target exists, most runs are therefore spent
 * on a square region around it at up to native resolution (confirmations and re-seeds on
 * small aircraft), interleaved with whole frames (new aircraft, and the neural override that
 * rescues a track stuck on something else).
 */
class DetectionPlanner(
    private val sourceWidth: Int,
    private val sourceHeight: Int,
    var config: DetectionPlanConfig = DetectionPlanConfig(),
) {
    private var runs = 0L
    private var scanIndex = 0

    /**
     * Region (square, source px, inside the frame) for the next run, or null for the whole
     * frame. [target] is the aircraft's expected image position at the time of the frame.
     */
    fun next(p: TrackPrediction, target: Vec2?, timestampNs: Long): Box? {
        val n = runs++
        val unseenS = ((timestampNs - p.lastMeasuredNs) * 1e-9).coerceAtLeast(0.0)
        if (target == null || p.state == TrackState.IDLE || (p.state == TrackState.LOST && unseenS * 1e9 > config.maxLostNs)) {
            if (config.scanTileFraction <= 0 || n % (config.scanRunsPerFullFrame + 1) == 0L) return null
            return scanTile()
        }
        if (n % (config.roiRunsPerFullFrame + 1) == 0L) return null
        return region(target, max(p.width, p.height), unseenS)
    }

    /** Next tile of the scan: overlapping squares covering the frame, top row (sky) first. */
    private fun scanTile(): Box {
        val side = sourceHeight * config.scanTileFraction
        val cols = kotlin.math.ceil((sourceWidth - side) / side).toInt() + 1
        val rows = kotlin.math.ceil((sourceHeight - side) / side).toInt() + 1
        val i = scanIndex++ % (cols * rows)
        val col = i % cols
        val row = i / cols
        val x = if (cols == 1) sourceWidth / 2.0 else side / 2 + col * (sourceWidth - side) / (cols - 1)
        val y = if (rows == 1) sourceHeight / 2.0 else side / 2 + row * (sourceHeight - side) / (rows - 1)
        return Box(x, y, side, side)
    }

    fun region(center: Vec2, objectSize: Double, unseenS: Double = 0.0): Box {
        val h = sourceHeight.toDouble()
        val side = max(objectSize * config.sizeFactor, h * config.minSideFraction) + h * config.growthPerSFraction * unseenS
        val s = min(side, h)
        val cx = center.x.coerceIn(s / 2, sourceWidth - s / 2)
        val cy = center.y.coerceIn(s / 2, h - s / 2)
        return Box(cx, cy, s, s)
    }

    fun reset() {
        runs = 0
        scanIndex = 0
    }
}
