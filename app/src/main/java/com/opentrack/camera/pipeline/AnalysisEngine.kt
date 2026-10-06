package com.opentrack.camera.pipeline

import android.graphics.Bitmap
import android.os.Handler
import android.os.HandlerThread
import android.os.SystemClock
import android.util.Log
import androidx.core.graphics.createBitmap
import com.opentrack.camera.detect.DetectorHost
import com.opentrack.camera.detect.DetectorModel
import com.opentrack.camera.diag.Telemetry
import com.opentrack.camera.detect.TargetClasses
import com.opentrack.tracking.CameraMotion
import com.opentrack.tracking.Detection
import com.opentrack.tracking.CropWindow
import com.opentrack.tracking.DetectionPlanner
import com.opentrack.tracking.ExposureMeter
import com.opentrack.tracking.SceneMetering
import com.opentrack.tracking.LumaFrame
import com.opentrack.tracking.NeuralRun
import com.opentrack.tracking.SkyBlobDetector
import com.opentrack.tracking.TargetTracker
import com.opentrack.tracking.TrackPrediction
import com.opentrack.tracking.TrackState
import com.opentrack.tracking.Vec2
import com.opentrack.tracking.VisualMotionEstimator
import com.opentrack.tracking.Box
import com.opentrack.tracking.FrameLens
import com.opentrack.tracking.LensCalibrator
import com.opentrack.tracking.calibrationSamples
import java.nio.ByteBuffer
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.ConcurrentLinkedQueue
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.roundToInt

/**
 * Runs the tracker on every analysis frame (analysis thread) and the neural detector
 * asynchronously at a lower rate (detector thread). Frames are dropped rather than queued
 * when the analysis thread falls behind, so latency never builds up.
 */
class AnalysisEngine(
    private val tracker: TargetTracker,
    private val neural: DetectorHost?,
    /** Camera motion from the image, used when the device has no gyroscope (null otherwise). */
    private val visualMotion: VisualMotionEstimator?,
    /** The camera motion the tracker uses (gyro or [visualMotion]). */
    private val motion: CameraMotion,
    val width: Int,
    val height: Int,
    /** Source px per analysis px. */
    val scale: Double,
) {
    enum class Mode { HYBRID, NEURAL_ONLY, SKY_ONLY }

    @Volatile var mode = Mode.HYBRID
    /** Which neural classes may become targets. */
    @Volatile var targetClasses = TargetClasses.AIRCRAFT
    /** Neural model (see [DetectorModel.forTarget]). */
    @Volatile var model = DetectorModel.AIRCRAFT
    @Volatile var neuralIntervalMs = 100L
    /**
     * Exposure metering, ~10 Hz (analysis thread): brightness of the recorded crop with the
     * target weighted in, the frame's timestamp, and the longest exposure time that keeps the
     * subject sharp (the target, or without one the scene; null when nothing moves).
     */
    @Volatile var onSceneLuma: ((SceneMetering, Long, Long?) -> Unit)? = null

    /** The live crop (source px), set by the render thread: what metering looks at. */
    @Volatile var meterCrop: CropWindow? = null

    /** Output width (px), for the motion blur limit. */
    @Volatile var outputWidth = 1920
    /** While recording: detections and the tracker's state per analysis frame go here. */
    @Volatile var telemetry: Telemetry? = null
    @Volatile var lastAnalysisMs = 0.0
        private set
    @Volatile var lastNeuralMs = 0.0
        private set

    /**
     * A lens calibration converged (see [LensCalibrator]): lens key, result. Analysis thread.
     * Only with the gyroscope (the calibration compares its rotation with the image motion).
     */
    @Volatile var onLensCalibrated: ((String, LensCalibrator.Result) -> Unit)? = null

    /** Lenses already calibrated (or not to calibrate): no more samples for them. */
    val calibratedLenses: MutableSet<String> = java.util.Collections.synchronizedSet(HashSet())

    /** Lens being calibrated and how far (0..1, two agreeing fits needed), for the status line. */
    @Volatile var calibrationProgress: Pair<String, Double>? = null
        private set

    private val calibrators = HashMap<String, LensCalibrator>()
    private val lastFits = HashMap<String, LensCalibrator.Result>()
    private var calPrev: LumaFrame? = null
    private val calFlow = VisualMotionEstimator()
    private val calExecutor = Executors.newSingleThreadExecutor { Thread(it, "lens-calibration").apply { priority = Thread.MIN_PRIORITY } }
    private val calBusy = AtomicBoolean(false)
    private val calFitSize = HashMap<String, Int>()

    private val skyBlob = SkyBlobDetector()
    private val sourceCenter = Vec2(width * scale / 2, height * scale / 2)
    private var lastTargetBox: Box? = null
    private val thread = HandlerThread("analysis").apply { start() }
    private val handler = Handler(thread.looper)
    private val detectorExecutor = Executors.newSingleThreadExecutor { Thread(it, "detector") }
    private val freeBuffers = ArrayBlockingQueue<ByteArray>(3).apply { repeat(3) { add(ByteArray(width * height)) } }
    private val neuralResults = ConcurrentLinkedQueue<Pair<NeuralRun, List<Detection>>>()
    private val neuralBusy = AtomicBoolean(false)
    private val bitmap: Bitmap? = neural?.let { createBitmap(width, height) }
    private val roiBitmap: Bitmap? = neural?.let { createBitmap(ROI_SIZE, ROI_SIZE) }
    private val sourceW = width * scale
    private val sourceH = height * scale
    private val planner = DetectionPlanner(sourceW.roundToInt(), sourceH.roundToInt())
    private var lastNeuralStart = 0L
    private var frameIndex = 0L
    @Volatile private var released = false

    /** True when the render thread should read back an RGBA frame for the neural detector. */
    fun wantsNeuralFrame(): Boolean =
        neural != null && mode != Mode.SKY_ONLY && !neuralBusy.get() &&
            SystemClock.uptimeMillis() - lastNeuralStart >= neuralIntervalMs

    /**
     * Render thread, when [wantsNeuralFrame]: the part of the frame of [timestampNs] the next
     * run should look at (source px, a [ROI_SIZE] square to render), or null for the whole
     * frame (analysis size). Zoomed regions around the target let the detector recognize
     * aircraft far too small for it in the whole frame; see [DetectionPlanner].
     */
    fun neuralRegion(timestampNs: Long, view: Box? = null): Box? {
        val p = tracker.prediction
        val target = if (p.state == TrackState.IDLE) null else motion.poseAt(timestampNs).toImage(p.worldAt(timestampNs), sourceCenter)
        return planner.next(p, target, timestampNs, view)
    }

    /**
     * Render thread: [rgba] holds the RGBA pixels (top row first) of [region] (source px) of
     * the frame of [timestampNs]: [ROI_SIZE] square, or the whole frame at analysis size when
     * [region] is null.
     */
    fun submitNeuralFrame(rgba: ByteBuffer, timestampNs: Long, region: Box?, window: Box? = null) {
        val bmp = (if (region == null) bitmap else roiBitmap) ?: return
        val det = neural ?: return
        if (!neuralBusy.compareAndSet(false, true)) return
        lastNeuralStart = SystemClock.uptimeMillis()
        bmp.copyPixelsFromBuffer(rgba)
        // [window]: the whole-frame run covered only that part (a telephoto frame, see submitLuma).
        val area = region ?: window ?: Box(sourceW / 2, sourceH / 2, sourceW, sourceH)
        val runRegion = region ?: window
        detectorExecutor.execute {
            try {
                if (released) return@execute
                val t0 = SystemClock.elapsedRealtimeNanos()
                val m = model
                val dets = det.detect(bmp, timestampNs, area, targetClasses.labels, m) ?: return@execute
                lastNeuralMs = (SystemClock.elapsedRealtimeNanos() - t0) / 1e6
                neuralResults.add(NeuralRun(timestampNs, runRegion, m.inputPx) to dets)
                telemetry?.line("nn")?.i("ts", timestampNs)?.n("ms", lastNeuralMs)?.i("n", dets.size)
                    ?.let { l -> if (runRegion != null) l.a("roi", runRegion.cx, runRegion.cy, runRegion.width, runRegion.height, digits = 0) else l }?.send()
            } catch (e: Exception) {
                Log.e(TAG, "neural detection failed", e)
            } finally {
                neuralBusy.set(false)
            }
        }
    }

    /** Render thread: packed luma (width x height bytes, top row first). Dropped if busy. */
    /**
     * [visible]: the part of the (reference space) frame with camera pixels, null = all.
     * [window]: the part of the reference frame the image shows (null = all of it): a
     * telephoto frame is analysed at its own resolution (see [LumaFrame.windowed]).
     */
    fun submitLuma(luma: ByteBuffer, timestampNs: Long, visible: Box? = null, window: Box? = null, lens: FrameLens? = null) {
        val buf = freeBuffers.poll() ?: return
        luma.get(buf, 0, width * height)
        val frame = if (window == null) LumaFrame(width, height, buf, timestampNs, scale, visible, lens = lens)
        else LumaFrame(width, height, buf, timestampNs, window.width / width, visible, window.left, window.top, sourceW, sourceH, lens)
        handler.post { process(frame) }
    }

    /**
     * Whether the image may be a window of the reference frame: not with the image-based
     * motion estimate (no gyro), which compares consecutive images pixel for pixel.
     */
    val supportsWindows: Boolean get() = visualMotion == null

    private fun process(frame: LumaFrame) {
        try {
            if (released) return
            val t0 = SystemClock.elapsedRealtimeNanos()
            val detections = ArrayList<Detection>()
            val runs = ArrayList<NeuralRun>()
            while (true) {
                val (run, dets) = neuralResults.poll() ?: break
                runs += run
                detections += dets
            }
            val tracking = tracker.prediction.state == TrackState.TRACKING
            val skyEvery = if (tracking) 12 else 3
            if (mode != Mode.NEURAL_ONLY && frameIndex % skyEvery == 0L) detections += skyBlob.detect(frame).filter { d -> frame.visible?.let { inside(d.box, it) } ?: true }
            val meterNow = frameIndex % 6 == 0L
            frameIndex++
            visualMotion?.process(frame, lastTargetBox)
            if (visualMotion == null) calibrate(frame)
            val p = tracker.process(frame, detections, runs)
            if (meterNow) meter(frame, p)
            // Mask the aircraft from the next motion estimate, so its own motion is not taken for shake.
            lastTargetBox = if (visualMotion != null && p.hasTarget) {
                val c = visualMotion.poseAt(frame.timestampNs).toImage(p.worldAt(frame.timestampNs), sourceCenter)
                Box(c.x, c.y, maxOf(p.width, 24 * scale), maxOf(p.height, 24 * scale))
            } else null
            lastAnalysisMs = (SystemClock.elapsedRealtimeNanos() - t0) / 1e6
            telemetry?.let { logTrack(it, frame.timestampNs, p, detections) }
        } finally {
            freeBuffers.offer(frame.data)
        }
    }

    /**
     * Lens calibration: every few frames, background features matched into the next frame
     * against the gyro's rotation (see [LensCalibrator]); fits run on their own thread.
     */
    private fun calibrate(frame: LumaFrame) {
        val lens = frame.lens ?: return
        if (lens.key in calibratedLenses) { calPrev = null; return }
        val prev = calPrev
        calPrev = null
        if (frameIndex % CAL_EVERY == 0L) {
            // Kept for the next frame (the buffer goes back to the pool).
            calPrev = LumaFrame(frame.width, frame.height, frame.data.copyOf(), frame.timestampNs, frame.scale, frame.visible, frame.originX, frame.originY, frame.sourceWidth, frame.sourceHeight, lens)
            return
        }
        if (prev == null || prev.lens?.sameAs(lens) != true || prev.scale != frame.scale || prev.originX != frame.originX ||
            frame.timestampNs - prev.timestampNs > 60_000_000L) return
        val pa = motion.poseAt(prev.timestampNs)
        val pb = motion.poseAt(frame.timestampNs)
        val guess = (pb.offset - pa.offset) / frame.scale
        // A still camera says nothing about the lens's scale.
        if (guess.length() < 0.8) return
        val pairs = calFlow.correspondences(prev, frame, lastTargetBox, guess)
        if (pairs.size < 20) return
        var c = calibrators[lens.key]
        if (c == null || c.prior != lens.model) {
            // (A new model in use: its samples start over.)
            c = LensCalibrator(lens.model).also { calibrators[lens.key] = it }
            calFitSize.remove(lens.key)
            lastFits.remove(lens.key)
        }
        for (s in calibrationSamples(prev, frame, pairs, motion)) c.add(s)
        calibrationProgress = lens.key to minOf(0.99, c.size / (2.0 * c.minSamples))
        val last = calFitSize[lens.key] ?: 0
        if (c.size >= c.minSamples && (last == 0 || c.size >= last + 1500 || c.size == 6000 && frameIndex % 300 == 0L) && calBusy.compareAndSet(false, true)) {
            calFitSize[lens.key] = c.size
            val data = c.snapshot()
            val key = lens.key
            calExecutor.execute {
                try {
                    val r = c.fit(data) ?: return@execute
                    handler.post { onFit(key, r) }
                } catch (e: Exception) {
                    Log.e(TAG, "lens calibration failed", e)
                } finally {
                    calBusy.set(false)
                }
            }
        }
    }

    /** Two fits in a row that agree: the lens is calibrated. */
    private fun onFit(key: String, r: LensCalibrator.Result) {
        telemetry?.line("lens")?.s("key", key)?.n("kappa", r.model.kappa, 1)?.n("a", r.model.a, 3)?.n("rms", r.rmsPx, 2)?.n("priorRms", r.priorRmsPx, 2)?.i("n", r.samples)?.send()
        val prev = lastFits[key]
        lastFits[key] = r
        if (prev == null) return
        val agree = kotlin.math.abs(prev.model.kappa / r.model.kappa - 1) < 0.015 && kotlin.math.abs(prev.model.a - r.model.a) < 0.05
        if (agree && key !in calibratedLenses) {
            calibratedLenses += key
            calibrationProgress = null
            onLensCalibrated?.invoke(key, r)
        }
    }

    /** Exposure metering of [frame] (see [onSceneLuma]), with the tracker's state after it. */
    private fun meter(frame: LumaFrame, p: TrackPrediction) {
        val cb = onSceneLuma ?: return
        val ts = frame.timestampNs
        val crop = meterCrop
        val framed = p.hasTarget && p.framable
        val pose = motion.poseAt(ts)
        val target = if (framed) pose.toImage(p.worldAt(ts), sourceCenter).let { Box(it.x, it.y, p.width, p.height) } else null
        val m = ExposureMeter.measure(frame, crop, target)
        val cropCenter = crop?.center ?: sourceCenter
        val cropW = crop?.width ?: sourceW
        // The subject must be sharp: the locked aircraft (a pan blurs the background, which is
        // fine), otherwise the scene under the crop (hand shake).
        val speed = ExposureMeter.imageSpeed(motion, ts, cropCenter, sourceCenter, cropW / 2, point = if (framed) p::worldAt else null)
        cb(m, ts, ExposureMeter.blurLimitNs(speed, cropW, outputWidth))
    }

    /**
     * One "trk" record: the tracker's state after this analysis frame (target in image px of
     * that frame) and every detection it was given ([label, score, cx, cy, w, h, source]).
     */
    private fun logTrack(t: Telemetry, ts: Long, p: TrackPrediction, detections: List<Detection>) {
        val line = t.line("trk").i("ts", ts).s("state", p.state.name).n("ms", lastAnalysisMs)
        if (p.hasTarget) {
            val c = motion.poseAt(ts).toImage(p.worldAt(ts), sourceCenter)
            line.a("box", c.x, c.y, p.width, p.height).a("vel", p.velocity.x, p.velocity.y).n("conf", p.confidence, 2)
                .b("framable", p.framable).b("measured", p.measuredWorld != null).n("age", (ts - p.lastMeasuredNs) / 1e6, 0)
        }
        if (detections.isNotEmpty()) {
            val sb = StringBuilder("[")
            detections.forEachIndexed { k, d ->
                if (k > 0) sb.append(',')
                sb.append("[\"").append(d.label.replace("\"", "")).append("\",").append(Telemetry.num(d.score.toDouble(), 2))
                for (v in doubleArrayOf(d.box.cx, d.box.cy, d.box.width, d.box.height)) sb.append(',').append(Telemetry.num(v, 0))
                sb.append(",\"").append(d.source.name).append("\"]")
            }
            line.raw("dets", sb.append(']').toString())
        }
        line.send()
    }

    /** Whether [b] is well inside [v]: a blob touching a telephoto frame's black edge is that edge. */
    private fun inside(b: Box, v: Box): Boolean {
        val m = EDGE_MARGIN_PX * scale
        return b.left >= v.left + m && b.right <= v.right - m && b.top >= v.top + m && b.bottom <= v.bottom - m
    }

    fun selectAt(point: Vec2) = handler.post { tracker.selectAt(point) }

    fun clearTarget() = handler.post { tracker.clear() }

    fun setAutoAcquire(enabled: Boolean) = handler.post { tracker.config = tracker.config.copy(autoAcquire = enabled) }

    fun release() {
        released = true
        thread.quitSafely()
        // The detector belongs to the activity (shared with gallery processing): not closed here.
        detectorExecutor.shutdown()
        calExecutor.shutdown()
    }

    companion object {
        private const val TAG = "AnalysisEngine"
        /** Lens calibration: one frame pair out of this many frames. */
        private const val CAL_EVERY = 4L
        /** Analysis px a sky blob must keep from a telephoto frame's edge. */
        private const val EDGE_MARGIN_PX = 2.0
        /**
         * Side of the rendered zoomed detector region: the own model's input (384); MediaPipe
         * shrinks it for the COCO model (320).
         */
        const val ROI_SIZE = 384
    }
}
