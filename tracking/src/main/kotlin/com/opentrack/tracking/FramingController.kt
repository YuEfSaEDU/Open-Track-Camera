package com.opentrack.tracking

import kotlin.math.exp
import kotlin.math.max
import kotlin.math.min

/** Crop window inside the source frame, in source pixels. */
data class CropWindow(val cx: Double, val cy: Double, val width: Double, val height: Double) {
    val left: Double get() = cx - width / 2
    val top: Double get() = cy - height / 2
}

data class FramingConfig(
    /** Natural frequency (rad/s) of the camera-path spring while following a target. */
    val followStiffness: Double = 9.0,
    /** Natural frequency while no target: plain electronic stabilization of the hand path. */
    val stabilizeStiffness: Double = 2.5,
    /** Max distance of the target from the crop center, as a fraction of the half crop size. */
    val maxTargetOffset: Double = 0.45,
    /** Seconds of velocity lead: frames slightly ahead of a moving target. */
    val leadTimeS: Double = 0.12,
    /** Lead is capped to this fraction of the half crop size. */
    val maxLeadFraction: Double = 0.25,
)

/**
 * Turns tracker output plus camera motion into a smooth crop window for every rendered
 * frame. Works in world (motion compensated) coordinates: the virtual camera follows a
 * critically damped spring there, and the measured camera shake is added back exactly, so
 * hand tremor is cancelled with no lag while the virtual camera itself moves smoothly.
 *
 * Call from the render thread only.
 */
class FramingController(
    val sourceWidth: Int,
    val sourceHeight: Int,
    private val outputAspect: Double,
    private val motion: CameraMotion,
    var config: FramingConfig = FramingConfig(),
) {
    /** Digital zoom: 1 = widest crop that fits the output aspect; 2 = 1:1 pixels for 4K → 1080p. */
    @Volatile
    var zoom: Double = 2.0

    private var initialized = false
    private var sx = 0.0
    private var sy = 0.0
    private var vx = 0.0
    private var vy = 0.0
    private var lastNs = 0L

    fun reset() {
        initialized = false
    }

    fun cropSize(): Pair<Double, Double> {
        var w = sourceWidth / zoom.coerceAtLeast(1.0)
        var h = w / outputAspect
        if (h > sourceHeight) {
            h = sourceHeight.toDouble()
            w = h * outputAspect
        }
        if (w > sourceWidth) {
            w = sourceWidth.toDouble()
            h = w / outputAspect
        }
        return w to h
    }

    fun update(timestampNs: Long, track: TrackPrediction): CropWindow {
        val (cw, ch) = cropSize()
        val offset = motion.offsetAt(timestampNs)
        val following = track.hasTarget

        val goal: Vec2 = if (following) {
            val w = track.worldAt(timestampNs)
            val maxLead = config.maxLeadFraction * min(cw, ch) / 2
            var lead = track.velocity * config.leadTimeS
            if (lead.length() > maxLead) lead = lead * (maxLead / lead.length())
            w + lead
        } else {
            // World position of the current frame center: following it slowly == EIS.
            Vec2(sourceWidth / 2.0, sourceHeight / 2.0) - offset
        }

        if (!initialized) {
            sx = goal.x; sy = goal.y; vx = 0.0; vy = 0.0
            lastNs = timestampNs
            initialized = true
        }
        val dt = ((timestampNs - lastNs) * 1e-9).coerceIn(0.0, 0.1)
        lastNs = timestampNs
        val omega = if (following) config.followStiffness else config.stabilizeStiffness
        // A critically damped spring chasing a goal moving at v lags by 2v/ω; feed the
        // target velocity forward so a steadily moving aircraft sits where the lead puts it.
        stepSpring(if (following) goal + track.velocity * (2 / omega) else goal, omega, dt)

        if (following) {
            // Never let the target leave the central part of the crop.
            val target = track.worldAt(timestampNs)
            val limX = config.maxTargetOffset * cw / 2
            val limY = config.maxTargetOffset * ch / 2
            sx = sx.coerceIn(target.x - limX, target.x + limX)
            sy = sy.coerceIn(target.y - limY, target.y + limY)
        }

        // Back to image space and keep the crop inside the sensor frame.
        val cx = (sx + offset.x).coerceIn(cw / 2, sourceWidth - cw / 2)
        val cy = (sy + offset.y).coerceIn(ch / 2, sourceHeight - ch / 2)
        // Anti wind-up: when clamped, move the virtual camera to where the crop really is.
        if (cx != sx + offset.x) { sx = cx - offset.x; vx = 0.0 }
        if (cy != sy + offset.y) { sy = cy - offset.y; vy = 0.0 }
        return CropWindow(cx, cy, cw, ch)
    }

    /** Exact step of a critically damped spring towards [goal]; stable for any dt. */
    private fun stepSpring(goal: Vec2, omega: Double, dt: Double) {
        if (dt <= 0) return
        val e = exp(-omega * dt)
        fun axis(pos: Double, vel: Double, target: Double): Pair<Double, Double> {
            val x0 = pos - target
            val c = vel + omega * x0
            val x = (x0 + c * dt) * e
            val v = (vel - omega * c * dt) * e
            return (target + x) to v
        }
        val (nx, nvx) = axis(sx, vx, goal.x)
        val (ny, nvy) = axis(sy, vy, goal.y)
        sx = nx; vx = nvx; sy = ny; vy = nvy
    }

    companion object {
        /** Largest zoom that keeps 1:1 (or better) pixel density for the given output width. */
        fun nativeZoom(sourceWidth: Int, outputWidth: Int): Double = max(1.0, sourceWidth.toDouble() / outputWidth)
    }
}
