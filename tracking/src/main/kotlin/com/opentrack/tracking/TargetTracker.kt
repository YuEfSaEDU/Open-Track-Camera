package com.opentrack.tracking

import kotlin.math.exp
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
 * "world" coordinates (image position with the [CameraPose] removed), in source pixels.
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
    /** Time of the last actual measurement (template match or detection). */
    val lastMeasuredNs: Long = timestampNs,
    /**
     * The tracking point as measured on this very frame (sub-pixel template match, or the
     * re-seeded point), in world coordinates, before any filtering; null when the frame was
     * only predicted. Framing can lock to it: this is what a point tracker stabilizes on.
     */
    val measuredWorld: Vec2? = null,
    /**
     * From the tracking point to the aircraft's center (source px, slowly varying): the point is
     * where the template sits (often the dark core the track started on); framing centers
     * the aircraft while stabilizing on the point.
     */
    val centerOffset: Vec2 = Vec2.ZERO,
    /**
     * Whether framing (crop, zoom) should follow this target. A lock not yet confirmed as an
     * aircraft (a sky blob on a cloud edge, a floor speck, a bird) is tracked but not framed:
     * the view stays wide until the neural detector confirms it, or until it is too small to
     * confirm but moves across the sky like an aircraft (clouds and specks do not).
     */
    val framable: Boolean = true,
) {
    val hasTarget: Boolean get() = state == TrackState.TRACKING || state == TrackState.COASTING

    /** This prediction as framing should see it: no target while the lock is tentative. */
    fun forFraming(): TrackPrediction = if (framable || !hasTarget) this else IDLE.copy(timestampNs = timestampNs)

    /** Where framing should put the output center: the aircraft's center at [timestampNs]. */
    fun framingAt(timestampNs: Long): Vec2 = worldAt(timestampNs) + centerOffset

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
    /** A match must also resemble the detection-seeded anchor template this much. */
    val minAnchorScore: Double = 0.45,
    /** Matched patch contrast relative to the anchor template, and absolute minimum (gray levels). */
    val minContrastRatio: Double = 0.45,
    val minContrast: Double = 2.5,
    /** Match score below which the template is re-seeded from an associated detection. */
    val reseedScore: Double = 0.75,
    val minSearchRadius: Int = 6,
    val maxSearchRadius: Int = 48,
    val maxCoastNs: Long = 1_200_000_000L,
    val lostTimeoutNs: Long = 8_000_000_000L,
    /** Switch to a neural aircraft after this many consecutive neural results disagreeing with the track... */
    val neuralOverrideCount: Int = 4,
    /** ...and no neural confirmation of the track for this long. */
    val neuralOverrideNs: Long = 1_500_000_000L,
    /** With auto lock on, a lost track may jump to any detection after this long. */
    val reacquireAnywhereNs: Long = 1_000_000_000L,
    /** Automatically lock onto the best detection while idle / lost. */
    val autoAcquire: Boolean = true,
    /** Allow sky-blob detections (not just neural ones) to start a new track. */
    val acquireFromSkyBlobs: Boolean = true,
    /** Size of the box used when the user taps on something no detector reported, source px. */
    val defaultTapBoxPx: Double = 160.0,
    val sizeSmoothing: Double = 0.2,
    /** How fast the re-acquisition gate grows while the target is unseen, source px/s. */
    val gateGrowthPxPerS: Double = 400.0,
    /** Allowed size ratio between an associated detection and the tracked object. */
    val skySizeTolerance: Double = 2.5,
    val neuralSizeTolerance: Double = 4.0,
    /**
     * Center the template search on the aircraft's image-space motion instead of the world
     * prediction (useful when the operator pans with the aircraft and the world motion is
     * large and manoeuvring).
     */
    val imageSpaceSearch: Boolean = false,
    /** Upper bound of the association gate as a fraction of the frame width. */
    val maxGateFraction: Double = 0.2,
    /**
     * Verification: a track not started from a neural detection (a sky blob) is dropped when
     * this many neural runs that looked at it closely enough to recognize an aircraft found
     * none there...
     */
    val verifyRuns: Int = 5,
    /** ...and a neural-confirmed one after this many (it drifted onto smoke, a cloud, the crowd). */
    val reverifyRuns: Int = 12,
    /** Smallest object (px in the detector's input) the neural detector is expected to recognize... */
    val minRecognizablePx: Double = 14.0,
    /** ...and in source px (an upscaled handful of pixels is still unrecognizable). */
    val minRecognizableSourcePx: Double = 12.0,
    /** A rejected spot is not acquired again for this long. */
    val rejectBlacklistNs: Long = 3_000_000_000L,
    /**
     * A target too small for the detector to confirm is framed once it moves across the sky
     * (world coordinates) at least this fast, source px/s at 3840 px width: an aircraft does,
     * a cloud or a speck on the ground does not.
     */
    val minFramableSpeedPxS: Double = 60.0,
    /** Smallest zoomed detector region as a fraction of the frame height (DetectionPlanConfig.minSideFraction). */
    val roiMinSideFraction: Double = 0.15,
    /** Neural detections disagreeing with the template this many times in a row force a re-seed. */
    val inconsistentReseedCount: Int = 2,
    /**
     * While coasting, the extrapolated velocity decays with this time constant: an aircraft
     * gone into a cloud or out of frame should not drag the framing along a straight line.
     */
    val coastVelocityDecayS: Double = 0.5,
) {
    companion object {
        /**
         * Pixel-valued defaults are tuned for a 3840 px wide source; this scales them to
         * [sourceWidth]. Without a gyro, hand shake appears as target motion, so the motion
         * model must be more agile.
         */
        fun forSource(sourceWidth: Int, hasGyro: Boolean): TrackerConfig {
            val k = sourceWidth / 3840.0
            val base = TrackerConfig()
            return base.copy(
                accelNoise = (if (hasGyro) 1.5e6 else 8e6) * k * k,
                defaultTapBoxPx = base.defaultTapBoxPx * k,
                gateGrowthPxPerS = base.gateGrowthPxPerS * k,
            )
        }
    }
}

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

    /**
     * Rolling shutter timing of the analysed frames; when set (gyro motion only), image
     * positions are converted with the camera pose at their own row's readout time.
     */
    @Volatile
    var rollingShutter: RollingShutter? = null

    /** Most recent detections, for UI and tap-to-select. */
    @Volatile
    var latestDetections: List<Detection> = emptyList()
        private set

    /** Diagnostics of the last processed frame (template match and what updated the track). */
    var lastMatch: TemplateTracker.Match? = null
        private set
    var lastEvent: String = ""
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
    private var frameWidthPx = 3840.0
    private var frameCenter = Vec2(1920.0, 1080.0)
    private var userSelected = false
    private var measurement = Vec2.ZERO
    private var lastImage = Vec2.ZERO
    private var lastImageNs = Long.MIN_VALUE
    private var imageVelocity = Vec2.ZERO
    private var measurementNs = Long.MIN_VALUE
    /** Tracking point relative to the detection box center, in units of the box size. */
    private var pointOffset = Vec2.ZERO
    private var pointOffsetValid = false
    /** [TrackPrediction.centerOffset], smoothed more than [pointOffset] (it moves the framing). */
    private var centerOffset = Vec2.ZERO
    private var neuralDisagreements = 0
    private var lastNeuralConfirmNs = Long.MIN_VALUE / 2
    /** A neural detection has confirmed this track (or the user picked it). */
    private var verified = false
    /** Neural runs that should have recognized the target since its last confirmation, but did not. */
    private var unconfirmedRuns = 0
    private var inconsistentNeural = 0
    /** The tracked size comes from a neural box (not only from a sky blob, i.e. the dark core). */
    private var sizeFromNeural = false
    private var confirmedThisFrame = false
    /** Framing may follow this track (see [TrackPrediction.framable]); latched until the next track. */
    private var framableLatched = false
    /** Neural runs have been reported at least once (otherwise there is nothing to verify with). */
    private var neuralSeen = false
    private var frameHeightPx = 2160.0
    private class Rejected(val world: Vec2, val radius: Double, val untilNs: Long)
    private val rejected = ArrayList<Rejected>()

    /** Requests a target at [point] (image source px, on the latest frame). Applied on the next frame. */
    fun selectAt(point: Vec2) {
        pendingTap = point
    }

    fun clear() {
        pendingClear = true
    }

    /**
     * @param neuralRuns neural runs whose results are part of [detections] (possibly none),
     *   used to verify tracks; empty when unknown (then tracks are never rejected).
     */
    fun process(frame: LumaFrame, detections: List<Detection>, neuralRuns: List<NeuralRun> = emptyList()): TrackPrediction {
        val t = frame.timestampNs
        rejected.removeAll { it.untilNs <= t }
        if (neuralRuns.isNotEmpty()) neuralSeen = true
        frameHeightPx = frame.height * frame.scale
        frameWidthPx = frame.width * frame.scale
        frameCenter = Vec2(frameWidthPx / 2, frame.height * frame.scale / 2)
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
            userSelected = true
            verified = true
            sizeFromNeural = det?.source == DetectionSource.NEURAL
            return publish(t)
        }

        when (state) {
            TrackState.IDLE -> if (config.autoAcquire) acquire(frame, detections, null)
            TrackState.TRACKING, TrackState.COASTING -> {
                step(frame, detections)
                verify(frame, neuralRuns)
            }
            TrackState.LOST -> {
                predictTo(t)
                // Near the last position first; after a while anything the auto lock would take.
                val reacquired = acquire(frame, detections, gateFor(t)) ||
                    (config.autoAcquire && t - lastSeenNs > config.reacquireAnywhereNs && acquire(frame, detections, null))
                if (!reacquired && t - lastSeenNs > config.lostTimeoutNs) {
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
        confirmedThisFrame = false
        val frameDt = (t - filterTimeNs) * 1e-9
        predictTo(t)
        val s = frame.scale
        // Without a trustworthy camera pose for this frame (featureless sky), search around the
        // aircraft's own image-space motion instead of the world prediction.
        val motionOk = (motion as? MotionReliability)?.isReliableAt(t) ?: true
        val imageDt = ((t - lastImageNs) * 1e-9).coerceIn(0.0, 0.25)
        val worldPrediction = toImage(Vec2(fx.position, fy.position), t)
        val imagePrediction = if (lastImageNs == Long.MIN_VALUE) null else lastImage + imageVelocity * imageDt

        val sigma = sqrt(max(fx.positionVariance, fy.positionVariance)) / s
        val radius = (3 * sigma + 2).roundToInt().coerceIn(config.minSearchRadius, config.maxSearchRadius)
        // Search center: the world prediction, unless the camera pose of this frame is unknown
        // (then the aircraft's own image motion), or image-space search is forced by config.
        // A second, fallback search was tried and rejected: it lets a similar-looking patch
        // (smoke, cloud edges) steal the track.
        val useImage = imagePrediction != null && (!motionOk || config.imageSpaceSearch)
        val center = if (useImage) imagePrediction!! else worldPrediction
        val match = template.track(frame, center / s, radius)
        lastMatch = match
        lastEvent = if (match == null) "no-match" else "match"
        val measured = isGood(match)
        if (measured) {
            val image = match!!.center * s
            val r = (config.matchStdAnalysisPx * s).let { it * it } / (match.score * match.score)
            val correctable = motion as? CorrectableMotion
            if (!motionOk && correctable != null) {
                // The aircraft is the only reference: its unexpected image motion is camera motion.
                // Fix the pose so the aircraft stays on its smooth world path.
                correctable.correct(t, image - toImage(Vec2(fx.position, fy.position), t))
            }
            val z = toWorld(image, t)
            measurement = z
            measurementNs = t
            fx.update(z.x, r)
            fy.update(z.y, r)
            if (lastImageNs != Long.MIN_VALUE && t > lastImageNs) {
                val v = (image - lastImage) / ((t - lastImageNs) * 1e-9)
                imageVelocity = if (imageDt > 0.2) v else imageVelocity + (v - imageVelocity) * 0.3
            }
            lastImage = image
            lastImageNs = t
            lastSeenNs = t
            confidence = match.score
            state = TrackState.TRACKING
        } else {
            confidence *= 0.9
        }

        // Neural detections first: they are what confirms (verifies) the track.
        val det = associate(detections.filter { it.source == DetectionSource.NEURAL }, t) ?: associate(detections, t)
        if (det != null) {
            val box = projectDetection(det, t, Vec2(fx.velocity, fy.velocity))
            // Does the detection agree with where the template matched?
            val consistent = measured && (box.center - match!!.center * s).length() <= max(2 * s, 0.5 * max(box.width, box.height))
            if ((consistent || !measured) && det.source == DetectionSource.NEURAL && !sizeFromNeural) {
                // First neural box: the blob only measured the aircraft's dark core.
                width = box.width
                height = box.height
                sizeFromNeural = true
            } else if (consistent || !measured) {
                width += (box.width - width) * config.sizeSmoothing
                height += (box.height - height) * config.sizeSmoothing
            }
            if (det.source == DetectionSource.NEURAL) {
                inconsistentNeural = if (consistent || !measured) 0 else inconsistentNeural + 1
                if (consistent) confirmNeural(t)
            }
            if (consistent) {
                // A detector confirmed the track: refresh the drift anchor where the template is
                // registered (detection boxes jitter by several pixels; the match does not).
                template.reanchor(frame, match!!.center)
                // Remember where the tracking point sits inside the detection box, so a later
                // re-seed lands on the same spot of the aircraft instead of the box center.
                val size = max(box.width, box.height)
                val rel = (match.center * s - box.center) / size
                pointOffset = if (pointOffsetValid) pointOffset + (rel - pointOffset) * 0.2 else rel
                val toCenter = (box.center - match.center * s)
                centerOffset = if (pointOffsetValid) centerOffset + (toCenter - centerOffset) * 0.1 else toCenter
                pointOffsetValid = true
                lastEvent = "confirm:${det.source}"
            }
            // Re-seed when the template lost the target, or when the neural detector (more
            // trustworthy than sky blobs, which also fire on cloud edges and glare) disagrees
            // with a weak match, or keeps disagreeing with a good one (a template matching next
            // to the aircraft, typically on its own smoke trail).
            val reseed = !measured ||
                (!consistent && det.source == DetectionSource.NEURAL &&
                    (match!!.score < config.reseedScore || inconsistentNeural >= config.inconsistentReseedCount))
            if (reseed) {
                val point = trackingPoint(box)
                val z = toWorld(point, t)
                measurement = z
                measurementNs = t
                // The track point moves to the aircraft outright, keeping the velocity: the old
                // point was wrong (smoke trail, coasting drift), the aircraft did not jump. A
                // partial filter update would leave a lag that the next frame's image-space
                // match turns into a fake camera-pose correction (a jump of the whole view),
                // and its innovation would spike the velocity that framing feeds forward.
                fx.moveTo(z.x)
                fy.moveTo(z.y)
                template.init(frame, box.movedTo(point).scaled(1 / s))
                // The image-space search continues from here (not from the stale last match).
                if (!measured && lastImageNs != Long.MIN_VALUE && t > lastImageNs && t - lastImageNs < 250_000_000L) {
                    imageVelocity = (point - lastImage) / ((t - lastImageNs) * 1e-9)
                }
                lastImage = point
                lastImageNs = t
                lastEvent = "reseed:${det.source}"
                if (det.source == DetectionSource.NEURAL) {
                    confirmNeural(t)
                    inconsistentNeural = 0
                }
                lastSeenNs = t
                confidence = det.score.toDouble()
                state = TrackState.TRACKING
            }
        }

        checkNeuralOverride(frame, detections)

        if (lastSeenNs != t && frameDt > 0) {
            val k = exp(-frameDt / config.coastVelocityDecayS)
            fx.dampVelocity(k)
            fy.dampVelocity(k)
        }
        if (state == TrackState.TRACKING && lastSeenNs != t) state = TrackState.COASTING
        if (state == TrackState.COASTING && t - lastSeenNs > config.maxCoastNs) state = TrackState.LOST
    }

    /**
     * Sky-blob tracks can latch onto non-aircraft (lens dust, a bird, a smoke puff). If the
     * neural detector keeps reporting an aircraft elsewhere while never confirming the current
     * track, switch to that aircraft. Only in auto lock mode and never for a user-selected target.
     */
    private fun checkNeuralOverride(frame: LumaFrame, detections: List<Detection>) {
        val neural = detections.filter { it.source == DetectionSource.NEURAL }
        if (neural.isEmpty()) return
        val t = frame.timestampNs
        if (associate(neural, t) != null) {
            lastNeuralConfirmNs = t
            neuralDisagreements = 0
            return
        }
        neuralDisagreements++
        // An unverified track (a sky blob nothing has recognized yet) yields quickly to an
        // aircraft the network keeps seeing (twice: single false positives happen); a
        // verified one only after repeated disagreement.
        val override = if (verified) {
            neuralDisagreements >= config.neuralOverrideCount && t - lastNeuralConfirmNs > config.neuralOverrideNs
        } else neuralDisagreements >= 2
        if (config.autoAcquire && !userSelected && override) {
            val best = neural.maxBy { it.score }
            startTrack(frame, projectDetection(best, t, null))
            lastNeuralConfirmNs = t
            verified = true
            sizeFromNeural = true
            lastEvent = "switch:NEURAL"
        }
    }

    /** Picks a detection to (re)start tracking. With [gate] set, only near the extrapolated track. */
    private fun acquire(frame: LumaFrame, detections: List<Detection>, gate: Double?): Boolean {
        val t = frame.timestampNs
        val candidates = detections.filter { d ->
            (d.source == DetectionSource.NEURAL || config.acquireFromSkyBlobs) &&
                rejected.none { (toWorld(projectDetection(d, t, null).center, t) - it.world).length() < it.radius }
        }
        val chosen = if (gate != null) {
            val predicted = Vec2(fx.position, fy.position)
            candidates.map { it to (toWorld(projectDetection(it, t, null).center, t) - predicted).length() }
                .filter { it.second < gate }
                .minByOrNull { it.second }?.first
        } else {
            candidates.maxByOrNull { it.score + if (it.source == DetectionSource.NEURAL) 1f else 0f }
        } ?: return false
        startTrack(frame, projectDetection(chosen, t, null))
        verified = chosen.source == DetectionSource.NEURAL
        sizeFromNeural = verified
        if (verified) lastNeuralConfirmNs = t
        return true
    }

    private fun confirmNeural(t: Long) {
        verified = true
        unconfirmedRuns = 0
        confirmedThisFrame = true
        lastNeuralConfirmNs = t
    }

    /**
     * Counts neural runs that looked at the target closely enough to recognize an aircraft and
     * did not confirm it; too many and the track is dropped, and the spot ignored for a while.
     * This is what keeps a sky-blob lock on a person, a bag or a boat from sticking, and what
     * ends a track that slid onto smoke. Never applied to a target the user picked.
     */
    private fun verify(frame: LumaFrame, runs: List<NeuralRun>) {
        if (runs.isEmpty() || userSelected || !state.let { it == TrackState.TRACKING || it == TrackState.COASTING }) return
        if (confirmedThisFrame) return
        val t = frame.timestampNs
        val size = max(width, height)
        for (run in runs) {
            val at = toImage(Vec2(fx.position, fy.position), run.timestampNs)
            val region = run.region ?: Box(frameCenter.x, frameCenter.y, frameWidthPx, frameCenter.y * 2)
            if (!region.contains(at)) continue
            // Size of the target in the detector's input; upscaling adds no detail, so a
            // target also needs enough source pixels (e.g. a few px blob in a 480p video).
            val inputSize = size * run.inputPx / max(region.width, region.height)
            if (inputSize >= config.minRecognizablePx && size >= config.minRecognizableSourcePx) unconfirmedRuns++
        }
        val limit = if (verified) config.reverifyRuns else config.verifyRuns
        if (unconfirmedRuns >= limit) {
            val world = Vec2(fx.position, fy.position)
            rejected += Rejected(world, max(size * 2, 0.05 * frameWidthPx), t + config.rejectBlacklistNs)
            lastEvent = if (verified) "reject:stale" else "reject:unverified"
            state = TrackState.IDLE
            template.reset()
        }
    }

    private fun isFramable(): Boolean {
        if (framableLatched || verified || userSelected || !neuralSeen) return true
        val size = max(width, height)
        val smallestRegion = config.roiMinSideFraction * frameHeightPx
        val verifiable = size >= config.minRecognizableSourcePx && size * NEURAL_INPUT_PX / smallestRegion >= config.minRecognizablePx
        val speed = sqrt(fx.velocity * fx.velocity + fy.velocity * fy.velocity)
        if (!verifiable && state == TrackState.TRACKING && speed >= config.minFramableSpeedPxS * frameWidthPx / 3840.0) framableLatched = true
        return framableLatched
    }

    private fun isGood(m: TemplateTracker.Match?): Boolean =
        m != null && m.score >= config.minMatchScore && m.anchorScore >= config.minAnchorScore &&
            m.contrastRatio >= config.minContrastRatio && m.contrast >= config.minContrast

    /** The stable tracking point for a detection box: box center plus the learned offset. */
    private fun trackingPoint(box: Box): Vec2 =
        if (pointOffsetValid) box.center + pointOffset * max(box.width, box.height) else box.center

    private fun startTrack(frame: LumaFrame, box: Box) {
        val t = frame.timestampNs
        val w = toWorld(box.center, t)
        val posVar = (0.1 * max(box.width, box.height)).let { it * it } + 4.0
        val velVar = 2000.0 * 2000.0
        fx.reset(w.x, posVar, 0.0, velVar)
        fy.reset(w.y, posVar, 0.0, velVar)
        filterTimeNs = t
        lastSeenNs = t
        width = box.width
        height = box.height
        sizeFromNeural = false
        confidence = 1.0
        template.init(frame, box.scaled(1 / frame.scale))
        state = TrackState.TRACKING
        userSelected = false
        verified = false
        framableLatched = false
        unconfirmedRuns = 0
        inconsistentNeural = 0
        neuralDisagreements = 0
        pointOffset = Vec2.ZERO
        pointOffsetValid = false
        centerOffset = Vec2.ZERO
        lastImage = box.center
        lastImageNs = t
        imageVelocity = Vec2.ZERO
    }

    /**
     * Detections come from an earlier frame (neural inference is slower than the frame
     * rate). Maps the box to the image of time [t] using the camera motion in between and,
     * if known, the target velocity.
     */
    private fun projectDetection(det: Detection, t: Long, velocity: Vec2?): Box {
        val worldAtDet = toWorld(det.box.center, det.timestampNs)
        val dt = ((t - det.timestampNs) * 1e-9).coerceIn(0.0, 0.5)
        val worldNow = worldAtDet + (velocity ?: Vec2.ZERO) * dt
        return det.box.movedTo(toImage(worldNow, t))
    }

    private fun associate(detections: List<Detection>, t: Long): Detection? {
        if (detections.isEmpty()) return null
        val predicted = Vec2(fx.position, fy.position)
        val vel = Vec2(fx.velocity, fy.velocity)
        val gate = gateFor(t)
        return detections
            .filter { sizeCompatible(it) }
            .map { it to (toWorld(projectDetection(it, t, vel).center, t) - predicted).length() }
            .filter { it.second < gate }
            .minByOrNull { it.second - it.first.score * 10 }?.first
    }

    /**
     * Rejects detections whose size is implausible for the tracked object (e.g. a glare
     * streak next to a distant aircraft). Neural boxes vary more, so their tolerance is wider.
     */
    private fun sizeCompatible(d: Detection): Boolean {
        val tracked = max(width, height)
        if (tracked <= 0) return true
        val ratio = max(d.box.width, d.box.height) / tracked
        // A sky blob is only the aircraft's dark core; its full neural box can be much larger.
        val tol = if (d.source != DetectionSource.NEURAL) config.skySizeTolerance
        else if (sizeFromNeural) config.neuralSizeTolerance else config.neuralSizeTolerance * 2
        return ratio in (1 / tol)..tol
    }

    private fun toWorld(image: Vec2, t: Long): Vec2 =
        rollingShutter?.toWorld(motion, image, t, frameCenter) ?: motion.poseAt(t).toWorld(image, frameCenter)

    private fun toImage(world: Vec2, t: Long): Vec2 =
        rollingShutter?.toImage(motion, world, t, frameCenter) ?: motion.poseAt(t).toImage(world, frameCenter)

    private fun gateFor(t: Long): Double {
        val sigma = sqrt(max(fx.positionVariance, fy.positionVariance))
        val lostFor = ((t - lastSeenNs) * 1e-9).coerceAtLeast(0.0)
        val gate = max(width, height) * 1.5 + 3 * sigma + config.gateGrowthPxPerS * lostFor
        return gate.coerceAtMost(max(config.maxGateFraction * frameWidthPx, max(width, height) * 2))
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
                lastMeasuredNs = lastSeenNs,
                measuredWorld = if (measurementNs == t && state != TrackState.LOST) measurement else null,
                centerOffset = centerOffset,
                framable = isFramable(),
            )
        }
        prediction = p
        return p
    }

    private companion object {
        const val DETECTION_TTL_NS = 700_000_000L
        const val NEURAL_INPUT_PX = 320.0
    }
}
