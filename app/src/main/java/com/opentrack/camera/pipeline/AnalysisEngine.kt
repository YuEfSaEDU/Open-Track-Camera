package com.opentrack.camera.pipeline

import android.graphics.Bitmap
import android.os.Handler
import android.os.HandlerThread
import android.os.SystemClock
import android.util.Log
import androidx.core.graphics.createBitmap
import com.opentrack.camera.detect.AircraftDetector
import com.opentrack.tracking.Detection
import com.opentrack.tracking.LumaFrame
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

/**
 * Runs the tracker on every analysis frame (analysis thread) and the neural detector
 * asynchronously at a lower rate (detector thread). Frames are dropped rather than queued
 * when the analysis thread falls behind, so latency never builds up.
 */
class AnalysisEngine(
    private val tracker: TargetTracker,
    private val neural: AircraftDetector?,
    /** Camera motion from the image, used when the device has no gyroscope (null otherwise). */
    private val visualMotion: VisualMotionEstimator?,
    val width: Int,
    val height: Int,
    /** Source px per analysis px. */
    val scale: Double,
) {
    enum class Mode { HYBRID, NEURAL_ONLY, SKY_ONLY }

    @Volatile var mode = Mode.HYBRID
    @Volatile var neuralIntervalMs = 100L
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
    private val neuralResults = ConcurrentLinkedQueue<List<Detection>>()
    private val neuralBusy = AtomicBoolean(false)
    private val bitmap: Bitmap? = neural?.let { createBitmap(width, height) }
    private var lastNeuralStart = 0L
    private var frameIndex = 0L
    @Volatile private var released = false

    /** True when the render thread should read back an RGBA frame for the neural detector. */
    fun wantsNeuralFrame(): Boolean =
        neural != null && mode != Mode.SKY_ONLY && !neuralBusy.get() &&
            SystemClock.uptimeMillis() - lastNeuralStart >= neuralIntervalMs

    /** Render thread: [rgba] holds width x height RGBA pixels, top row first. */
    fun submitNeuralFrame(rgba: ByteBuffer, timestampNs: Long) {
        val bmp = bitmap ?: return
        val det = neural ?: return
        if (!neuralBusy.compareAndSet(false, true)) return
        lastNeuralStart = SystemClock.uptimeMillis()
        bmp.copyPixelsFromBuffer(rgba)
        detectorExecutor.execute {
            try {
                if (released) return@execute
                val t0 = SystemClock.elapsedRealtimeNanos()
                val dets = det.detect(bmp, timestampNs, scale)
                lastNeuralMs = (SystemClock.elapsedRealtimeNanos() - t0) / 1e6
                neuralResults.add(dets)
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
            while (true) detections += neuralResults.poll() ?: break
            val tracking = tracker.prediction.state == TrackState.TRACKING
            val skyEvery = if (tracking) 12 else 3
            if (mode != Mode.NEURAL_ONLY && frameIndex % skyEvery == 0L) detections += skyBlob.detect(frame)
            frameIndex++
            visualMotion?.process(frame, lastTargetBox)
            val p = tracker.process(frame, detections)
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
        detectorExecutor.execute { neural?.close() }
        detectorExecutor.shutdown()
    }

    private companion object {
        const val TAG = "AnalysisEngine"
    }
}
