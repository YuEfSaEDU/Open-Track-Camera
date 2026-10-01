package com.opentrack.tracking

import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.exp
import kotlin.math.sin
import kotlin.math.max
import kotlin.math.min

/**
 * Crop window inside the source frame, in source pixels, rotated by [rotation] radians
 * (clockwise on screen) about its center. Output pixel u (relative to the output center,
 * scaled to source px) comes from image position center + R(rotation) u.
 */
data class CropWindow(val cx: Double, val cy: Double, val width: Double, val height: Double, val rotation: Double = 0.0) {
    val left: Double get() = cx - width / 2
    val top: Double get() = cy - height / 2
    val center: Vec2 get() = Vec2(cx, cy)

    /** Maps an image point into crop-local coordinates (source px, origin at the crop center). */
    fun toLocal(image: Vec2): Vec2 = (image - center).rotated(-rotation)

    /** Half extents of the axis aligned bounding box of the rotated crop. */
    fun halfExtents(): Vec2 = rotatedHalfExtents(width, height, rotation)

    companion object {
        fun rotatedHalfExtents(w: Double, h: Double, rotation: Double): Vec2 {
            val c = abs(cos(rotation))
            val s = abs(sin(rotation))
            return Vec2((c * w + s * h) / 2, (s * w + c * h) / 2)
        }

        /** Largest |rotation| ≤ [limit] for which a w×h crop rotated by it fits in availW×availH. */
        fun maxFittingRotation(w: Double, h: Double, availW: Double, availH: Double, limit: Double): Double {
            fun fits(r: Double): Boolean {
                val e = rotatedHalfExtents(w, h, r)
                return e.x <= availW / 2 + 1e-9 && e.y <= availH / 2 + 1e-9
            }
            if (fits(limit)) return limit
            if (!fits(0.0)) return 0.0
            var lo = 0.0
            var hi = limit
            repeat(20) {
                val mid = (lo + hi) / 2
                if (fits(mid)) lo = mid else hi = mid
            }
            return lo
        }
    }
}

/** coerceIn that tolerates an empty range (returns its midpoint). */
internal fun Double.clampSafe(lo: Double, hi: Double): Double = if (lo <= hi) coerceIn(lo, hi) else (lo + hi) / 2

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
    /** Time constant of the virtual camera's roll (horizon) path; 0 disables roll stabilization. */
    val rollSmoothS: Double = 0.8,
    /** Largest crop rotation used to cancel roll, radians. */
    val maxRoll: Double = 0.09,
    /**
     * Seconds to glide onto a newly locked target: the view eases from plain stabilization to
     * following it instead of jumping. 0 = snap.
     */
    val acquireS: Double = 0.8,
    /** Seconds for that engagement to fade after the target is lost (a brief loss resumes quickly). */
    val releaseS: Double = 1.0,
)

/** Smooth 0..1 ease (zero slope at both ends). */
internal fun smoothstep(x: Double): Double {
    val t = x.coerceIn(0.0, 1.0)
    return t * t * (3 - 2 * t)
}

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
    /**
     * Digital zoom: 1 = widest crop that fits the output aspect; 2 = 1:1 pixels for 4K → 1080p.
     * Set by the user in manual mode; driven by [autoZoom] when [ZoomConfig.auto] is on.
     */
    @Volatile
    var zoom: Double = 2.0

    val autoZoom = AutoZoom()

    /** Safe to set from any thread. */
    var zoomConfig: ZoomConfig
        get() = autoZoom.config
        set(value) {
            if (value.auto && !autoZoom.config.auto) autoZoom.reset(zoom)
            autoZoom.config = value
        }

    private var lastTargetOffset = 0.0

    private var initialized = false
    private var sx = 0.0
    private var sy = 0.0
    private var vx = 0.0
    private var vy = 0.0
    private var lastNs = 0L
    private var virtualRoll = 0.0

    /** 0 = plain stabilization, 1 = fully following the target; ramps on lock and loss. */
    private var engagement = 0.0
    private var lastTargetWorld: Vec2? = null
    private var hadTarget = false
    /** Containment limit while gliding (fraction of the half crop): it only ever tightens. */
    private var glideLimit = Double.POSITIVE_INFINITY

    /** Current engagement (eased), for tests and diagnostics. */
    val engaged: Double get() = smoothstep(engagement)

    fun reset() {
        engagement = 0.0
        hadTarget = false
        lastTargetWorld = null
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
        val dtZoom = if (initialized) ((timestampNs - lastNs) * 1e-9).coerceIn(0.0, 0.1) else 0.0
        if (zoomConfig.auto) {
            val coastingS = if (track.state == TrackState.COASTING) ((timestampNs - track.lastMeasuredNs) * 1e-9).coerceAtLeast(0.0) else 0.0
            zoom = autoZoom.update(dtZoom, zoom, track, sourceWidth, outputAspect, coastingS, lastTargetOffset)
        }
        val (cw, ch) = cropSize()
        val pose = motion.poseAt(timestampNs)
        val center = Vec2(sourceWidth / 2.0, sourceHeight / 2.0)
        val following = track.hasTarget

        val eisGoal = pose.toWorld(center, center)
        val target = if (following) track.framingAt(timestampNs) else null

        if (!initialized) {
            sx = eisGoal.x; sy = eisGoal.y; vx = 0.0; vy = 0.0
            virtualRoll = pose.roll
            lastNs = timestampNs
            initialized = true
        }
        val dt = ((timestampNs - lastNs) * 1e-9).coerceIn(0.0, 0.1)
        lastNs = timestampNs

        // Engagement: glide onto a new lock instead of jumping to it. A target that jumps (a
        // switch to another object, a re-seed that moved the track) starts a new glide; the
        // same aircraft re-found nearby after a short loss continues where it was.
        var restart = target != null && !hadTarget
        if (target != null) {
            val prev = lastTargetWorld
            if (prev != null && (target - prev).length() > config.maxTargetOffset * min(cw, ch) / 2) {
                engagement = 0.0
                restart = true
            }
            lastTargetWorld = target
            engagement = if (config.acquireS > 0) min(1.0, engagement + dt / config.acquireS) else 1.0
        } else {
            engagement = if (config.releaseS > 0) max(0.0, engagement - dt / config.releaseS) else 0.0
        }
        val s = if (target != null) smoothstep(engagement) else 0.0
        if (restart) glideLimit = if (config.acquireS > 0) Double.POSITIVE_INFINITY else config.maxTargetOffset
        hadTarget = target != null

        val omega = config.stabilizeStiffness + (config.followStiffness - config.stabilizeStiffness) * s
        if (target != null) {
            val maxLead = config.maxLeadFraction * min(cw, ch) / 2
            var lead = track.velocity * config.leadTimeS
            if (lead.length() > maxLead) lead = lead * (maxLead / lead.length())
            // A critically damped spring chasing a goal moving at v lags by 2v/ω; feed the
            // target velocity forward so a steadily moving aircraft sits where the lead puts it.
            val follow = target + lead + track.velocity * (2 / omega)
            stepSpring(eisGoal + (follow - eisGoal) * s, omega, dt)
        } else {
            // World position of the current frame center: following it slowly == EIS.
            stepSpring(eisGoal, omega, dt)
        }

        // Horizon: the virtual camera's roll follows the real roll slowly (removes roll shake).
        virtualRoll = if (config.rollSmoothS > 0) {
            val a = 1 - exp(-dt / config.rollSmoothS)
            (virtualRoll + (pose.roll - virtualRoll) * a).coerceIn(pose.roll - config.maxRoll, pose.roll + config.maxRoll)
        } else pose.roll
        // Only rotate as far as the rotated crop still fits in the sensor (none at 1x zoom).
        val rotLimit = CropWindow.maxFittingRotation(cw, ch, sourceWidth.toDouble(), sourceHeight.toDouble(), config.maxRoll)
        val rotation = (pose.roll - virtualRoll).coerceIn(-rotLimit, rotLimit)
        virtualRoll = pose.roll - rotation

        if (target != null) {
            // Never let the target leave the central part of the crop. A new lock may start
            // anywhere in the view: the limit then starts at the target's current offset and
            // only tightens as the glide brings it in (no yank), while still stopping the
            // target from drifting further out.
            val offset = max(abs(sx - target.x) / (cw / 2), abs(sy - target.y) / (ch / 2))
            glideLimit = max(config.maxTargetOffset, min(glideLimit, offset))
            val limX = glideLimit * cw / 2
            val limY = glideLimit * ch / 2
            sx = sx.coerceIn(target.x - limX, target.x + limX)
            sy = sy.coerceIn(target.y - limY, target.y + limY)
        }

        // Back to image space and keep the (rotated) crop inside the sensor frame.
        val ext = CropWindow.rotatedHalfExtents(cw, ch, rotation)
        val wanted = pose.toImage(Vec2(sx, sy), center)
        val cx = wanted.x.clampSafe(ext.x, sourceWidth - ext.x)
        val cy = wanted.y.clampSafe(ext.y, sourceHeight - ext.y)
        if (cx != wanted.x || cy != wanted.y) {
            // Anti wind-up: when clamped, move the virtual camera to where the crop really is.
            val w = pose.toWorld(Vec2(cx, cy), center)
            if (cx != wanted.x) vx = 0.0
            if (cy != wanted.y) vy = 0.0
            sx = w.x
            sy = w.y
        }
        val crop = CropWindow(cx, cy, cw, ch, rotation)

        lastTargetOffset = if (following) {
            val local = crop.toLocal(pose.toImage(track.framingAt(timestampNs), center))
            max(abs(local.x) / (cw / 2), abs(local.y) / (ch / 2))
        } else 0.0
        return crop
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
