package com.opentrack.camera.pipeline

import android.graphics.Bitmap
import android.os.Handler
import android.os.HandlerThread
import android.os.SystemClock
import android.util.Log
import androidx.core.graphics.createBitmap
import com.opentrack.camera.detect.DetectorHost
import com.opentrack.tracking.CameraMotion
import com.opentrack.tracking.Detection
import com.opentrack.tracking.DetectionPlanner
import com.opentrack.tracking.LumaFrame
import com.opentrack.tracking.NeuralRun
import com.opentrack.tracking.ShutterPriorityExposure
import com.opentrack.tracking.SkyBlobDetector
import com.opentrack.tracking.TargetTracker
import com.opentrack.tracking.TrackState
import com.opentrack.tracking.Vec2
import com.opentrack.tracking.VisualMotionEstimator
import com.opentrack.tracking.Box
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
    @Volatile var neuralIntervalMs = 100L
    /** Mean luma of the analysis frames, ~10 Hz, for the exposure loop (analysis thread). */
    @Volatile var onSceneLuma: ((Double) -> Unit)? = null
    @Volatile var lastAnalysisMs = 0.0
        private set
    @Volatile var lastNeuralMs = 0.0
        private set

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
    fun neuralRegion(timestampNs: Long): Box? {
        val p = tracker.prediction
        val target = if (p.state == TrackState.IDLE) null else motion.poseAt(timestampNs).toImage(p.worldAt(timestampNs), sourceCenter)
        return planner.next(p, target, timestampNs)
    }

    /**
     * Render thread: [rgba] holds the RGBA pixels (top row first) of [region] (source px) of
     * the frame of [timestampNs]: [ROI_SIZE] square, or the whole frame at analysis size when
     * [region] is null.
     */
    fun submitNeuralFrame(rgba: ByteBuffer, timestampNs: Long, region: Box?) {
        val bmp = (if (region == null) bitmap else roiBitmap) ?: return
        val det = neural ?: return
        if (!neuralBusy.compareAndSet(false, true)) return
        lastNeuralStart = SystemClock.uptimeMillis()
        bmp.copyPixelsFromBuffer(rgba)
        val area = region ?: Box(sourceW / 2, sourceH / 2, sourceW, sourceH)
        detectorExecutor.execute {
            try {
                if (released) return@execute
                val t0 = SystemClock.elapsedRealtimeNanos()
                val dets = det.detect(bmp, timestampNs, area) ?: return@execute
                lastNeuralMs = (SystemClock.elapsedRealtimeNanos() - t0) / 1e6
                neuralResults.add(NeuralRun(timestampNs, region, NEURAL_INPUT) to dets)
            } catch (e: Exception) {
                Log.e(TAG, "neural detection failed", e)
            } finally {
                neuralBusy.set(false)
            }
        }
    }

    /** Render thread: packed luma (width x height bytes, top row first). Dropped if busy. */
    fun submitLuma(luma: ByteBuffer, timestampNs: Long) {
        val buf = freeBuffers.poll() ?: return
        luma.get(buf, 0, width * height)
        handler.post { process(LumaFrame(width, height, buf, timestampNs, scale)) }
    }

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
            if (mode != Mode.NEURAL_ONLY && frameIndex % skyEvery == 0L) detections += skyBlob.detect(frame)
            if (frameIndex % 6 == 0L) onSceneLuma?.invoke(ShutterPriorityExposure.meanLuma(frame))
            frameIndex++
            visualMotion?.process(frame, lastTargetBox)
            val p = tracker.process(frame, detections, runs)
            // Mask the aircraft from the next motion estimate, so its own motion is not taken for shake.
            lastTargetBox = if (visualMotion != null && p.hasTarget) {
                val c = visualMotion.poseAt(frame.timestampNs).toImage(p.worldAt(frame.timestampNs), sourceCenter)
                Box(c.x, c.y, maxOf(p.width, 24 * scale), maxOf(p.height, 24 * scale))
            } else null
            lastAnalysisMs = (SystemClock.elapsedRealtimeNanos() - t0) / 1e6
        } finally {
            freeBuffers.offer(frame.data)
        }
    }

    fun selectAt(point: Vec2) = handler.post { tracker.selectAt(point) }

    fun clearTarget() = handler.post { tracker.clear() }

    fun setAutoAcquire(enabled: Boolean) = handler.post { tracker.config = tracker.config.copy(autoAcquire = enabled) }

    fun release() {
        released = true
        thread.quitSafely()
        // The detector belongs to the activity (shared with gallery processing): not closed here.
        detectorExecutor.shutdown()
    }

    companion object {
        private const val TAG = "AnalysisEngine"
        /** Side of the zoomed detector input (EfficientDet-Lite0 runs at 320 x 320). */
        const val ROI_SIZE = 320
        private const val NEURAL_INPUT = 320
    }
}
