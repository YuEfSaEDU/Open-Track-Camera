package com.opentrack.tracking

import kotlin.math.max
import kotlin.math.roundToInt
import kotlin.math.sqrt

enum class TrackState {
    /** No target selected. */
    IDLE,
    /** Target measured on the latest frame. */
    TRACKING,
    /** Target not measured recently; position extrapolated by the motion model. */
    COASTING,
    /** Target missing for too long; waiting for a detection to re-acquire. */
    LOST,
}

/**
 * Immutable tracker output, published after every analysed frame. Positions are
 * "world" coordinates (image position minus [CameraMotion] offset), in source pixels.
 * Safe to read from any thread; [worldAt] extrapolates to the timestamp being rendered.
 */
data class TrackPrediction(
    val state: TrackState,
    val timestampNs: Long,
    val world: Vec2,
    val velocity: Vec2,
    val width: Double,
    val height: Double,
    val confidence: Double,
) {
    val hasTarget: Boolean get() = state == TrackState.TRACKING || state == TrackState.COASTING

    fun worldAt(timestampNs: Long): Vec2 {
        val dt = (timestampNs - this.timestampNs) * 1e-9
        // Don't run far ahead on stale data.
        val clamped = dt.coerceIn(-0.1, 0.25)
        return world + velocity * clamped
    }

    companion object {
        val IDLE = TrackPrediction(TrackState.IDLE, 0L, Vec2.ZERO, Vec2.ZERO, 0.0, 0.0, 0.0)
    }
}

data class TrackerConfig(
    /** Process noise of the constant velocity model (source px²/s³). */
    val accelNoise: Double = 1.5e6,
    /** Std-dev of a template measurement, in analysis pixels. */
    val matchStdAnalysisPx: Double = 0.6,
    /** Std-dev of a detection measurement as a fraction of the object size. */
    val detectionStdFraction: Double = 0.15,
    val minMatchScore: Double = 0.55,
    /** Match score below which the template is re-seeded from an associated detection. */
    val reseedScore: Double = 0.75,
    val minSearchRadius: Int = 6,
    val maxSearchRadius: Int = 48,
    val maxCoastNs: Long = 1_200_000_000L,
    val lostTimeoutNs: Long = 8_000_000_000L,
    /** Automatically lock onto the best detection while idle / lost. */
    val autoAcquire: Boolean = true,
    /** Allow sky-blob detections (not just neural ones) to start a new track. */
    val acquireFromSkyBlobs: Boolean = true,
    /** Size of the box used when the user taps on something no detector reported, source px. */
    val defaultTapBoxPx: Double = 160.0,
    val sizeSmoothing: Double = 0.2,
)

/**
 * Single-target tracker. Combines a per-frame NCC template tracker (fast, precise, drifts
 * slowly) with periodic detections (slow, absolute, used to re-seed and re-acquire), all fused
 * in a constant-velocity Kalman filter that runs in camera-motion compensated coordinates.
 *
 * Not thread safe: call [process], [selectAt], [clear] from a single analysis thread and read
 * [prediction] from anywhere.
 */
class TargetTracker(
    private val motion: CameraMotion,
    var config: TrackerConfig = TrackerConfig(),
) {
    @Volatile
    var prediction: TrackPrediction = TrackPrediction.IDLE
        private set

    /** Most recent detections, for UI and tap-to-select. */
    @Volatile
    var latestDetections: List<Detection> = emptyList()
        private set

    private val fx = KalmanCv1D(config.accelNoise)
    private val fy = KalmanCv1D(config.accelNoise)
    private val template = TemplateTracker()
    private var state = TrackState.IDLE
    private var filterTimeNs = 0L
    private var lastSeenNs = 0L
    private var width = 0.0
    private var height = 0.0
    private var confidence = 0.0
    private var pendingTap: Vec2? = null
    private var pendingClear = false

    /** Requests a target at [point] (image source px, on the latest frame). Applied on the next frame. */
    fun selectAt(point: Vec2) {
        pendingTap = point
    }

    fun clear() {
        pendingClear = true
    }

    fun process(frame: LumaFrame, detections: List<Detection>): TrackPrediction {
        val t = frame.timestampNs
        updateLatestDetections(detections, t)

        fx.accelNoise = config.accelNoise
        fy.accelNoise = config.accelNoise
        if (pendingClear) {
            pendingClear = false
            pendingTap = null
            state = TrackState.IDLE
            template.reset()
        }
        pendingTap?.let { tap ->
            pendingTap = null
            val det = latestDetections.filter { it.box.contains(tap) || (it.box.center - tap).length() < config.defaultTapBoxPx / 2 }
                .minByOrNull { (it.box.center - tap).length() }
            val box = det?.let { projectDetection(it, t, null) } ?: Box(tap.x, tap.y, config.defaultTapBoxPx, config.defaultTapBoxPx)
            startTrack(frame, box)
            return publish(t)
        }

        when (state) {
            TrackState.IDLE -> if (config.autoAcquire) acquire(frame, detections, null)
            TrackState.TRACKING, TrackState.COASTING -> step(frame, detections)
            TrackState.LOST -> {
                predictTo(t)
                if (!acquire(frame, detections, gateFor(t)) && t - lastSeenNs > config.lostTimeoutNs) {
                    state = TrackState.IDLE
                    template.reset()
                }
            }
        }
        return publish(t)
    }

    /** Detectors run at different rates: keep the newest list per source, expire stale ones. */
    private fun updateLatestDetections(fresh: List<Detection>, t: Long) {
        val sources = fresh.mapTo(HashSet()) { it.source }
        latestDetections = latestDetections.filter { it.source !in sources && t - it.timestampNs < DETECTION_TTL_NS } + fresh
    }

    private fun step(frame: LumaFrame, detections: List<Detection>) {
        val t = frame.timestampNs
        predictTo(t)
        val offset = motion.offsetAt(t)
        val s = frame.scale
        val predictedImage = Vec2(fx.position, fy.position) + offset

        val sigma = sqrt(max(fx.positionVariance, fy.positionVariance)) / s
        val radius = (3 * sigma + 2).roundToInt().coerceIn(config.minSearchRadius, config.maxSearchRadius)
        val match = template.track(frame, predictedImage / s, radius)
        val measured = match != null && match.score >= config.minMatchScore
        if (measured) {
            val z = match!!.center * s - offset
            val r = (config.matchStdAnalysisPx * s).let { it * it } / (match.score * match.score)
            fx.update(z.x, r)
            fy.update(z.y, r)
            lastSeenNs = t
            confidence = match.score
            state = TrackState.TRACKING
        } else {
            confidence *= 0.9
        }

        val det = associate(detections, t)
        if (det != null) {
            val box = projectDetection(det, t, Vec2(fx.velocity, fy.velocity))
            width += (box.width - width) * config.sizeSmoothing
            height += (box.height - height) * config.sizeSmoothing
            if (!measured || match!!.score < config.reseedScore) {
                val z = box.center - offset
                val r = (config.detectionStdFraction * max(box.width, box.height)).let { it * it }
                fx.update(z.x, r)
                fy.update(z.y, r)
                template.init(frame, box.scaled(1 / s))
                lastSeenNs = t
                confidence = det.score.toDouble()
                state = TrackState.TRACKING
            }
        }

        if (state == TrackState.TRACKING && lastSeenNs != t) state = TrackState.COASTING
        if (state == TrackState.COASTING && t - lastSeenNs > config.maxCoastNs) state = TrackState.LOST
    }

    /** Picks a detection to (re)start tracking. With [gate] set, only near the extrapolated track. */
    private fun acquire(frame: LumaFrame, detections: List<Detection>, gate: Double?): Boolean {
        val t = frame.timestampNs
        val candidates = detections.filter { it.source == DetectionSource.NEURAL || config.acquireFromSkyBlobs }
        val chosen = if (gate != null) {
            val predicted = Vec2(fx.position, fy.position)
            candidates.map { it to (projectDetection(it, t, null).center - motion.offsetAt(t) - predicted).length() }
                .filter { it.second < gate }
                .minByOrNull { it.second }?.first
        } else {
            candidates.maxByOrNull { it.score + if (it.source == DetectionSource.NEURAL) 1f else 0f }
        } ?: return false
        startTrack(frame, projectDetection(chosen, t, null))
        return true
    }

    private fun startTrack(frame: LumaFrame, box: Box) {
        val t = frame.timestampNs
        val w = box.center - motion.offsetAt(t)
        val posVar = (0.1 * max(box.width, box.height)).let { it * it } + 4.0
        val velVar = 2000.0 * 2000.0
        fx.reset(w.x, posVar, 0.0, velVar)
        fy.reset(w.y, posVar, 0.0, velVar)
        filterTimeNs = t
        lastSeenNs = t
        width = box.width
        height = box.height
        confidence = 1.0
        template.init(frame, box.scaled(1 / frame.scale))
        state = TrackState.TRACKING
    }

    /**
     * Detections come from an earlier frame (neural inference is slower than the frame
     * rate). Maps the box to the image of time [t] using the camera motion in between and,
     * if known, the target velocity.
     */
    private fun projectDetection(det: Detection, t: Long, velocity: Vec2?): Box {
        val worldAtDet = det.box.center - motion.offsetAt(det.timestampNs)
        val dt = ((t - det.timestampNs) * 1e-9).coerceIn(0.0, 0.5)
        val worldNow = worldAtDet + (velocity ?: Vec2.ZERO) * dt
        return det.box.movedTo(worldNow + motion.offsetAt(t))
    }

    private fun associate(detections: List<Detection>, t: Long): Detection? {
        if (detections.isEmpty()) return null
        val predicted = Vec2(fx.position, fy.position)
        val vel = Vec2(fx.velocity, fy.velocity)
        val offset = motion.offsetAt(t)
        val gate = gateFor(t)
        return detections
            .map { it to (projectDetection(it, t, vel).center - offset - predicted).length() }
            .filter { it.second < gate }
            .minByOrNull { it.second - it.first.score * 10 }?.first
    }

    private fun gateFor(t: Long): Double {
        val sigma = sqrt(max(fx.positionVariance, fy.positionVariance))
        val lostFor = ((t - lastSeenNs) * 1e-9).coerceAtLeast(0.0)
        return max(width, height) * 1.5 + 3 * sigma + 400.0 * lostFor
    }

    private fun predictTo(t: Long) {
        val dt = (t - filterTimeNs) * 1e-9
        if (dt > 0) {
            fx.predict(dt)
            fy.predict(dt)
            filterTimeNs = t
        }
    }

    private fun publish(t: Long): TrackPrediction {
        val p = if (state == TrackState.IDLE) {
            TrackPrediction.IDLE.copy(timestampNs = t)
        } else {
            TrackPrediction(
                state = state,
                timestampNs = filterTimeNs,
                world = Vec2(fx.position, fy.position),
                velocity = if (state == TrackState.LOST) Vec2.ZERO else Vec2(fx.velocity, fy.velocity),
                width = width,
                height = height,
                confidence = confidence,
            )
        }
        prediction = p
        return p
    }

    private companion object {
        const val DETECTION_TTL_NS = 700_000_000L
    }
}
