package com.opentrack.camera.detect

import android.content.Context
import android.graphics.Bitmap
import android.os.SystemClock
import android.util.Log
import com.google.mediapipe.tasks.core.Delegate
import com.opentrack.tracking.Box
import com.opentrack.tracking.Detection
import java.util.concurrent.Callable
import java.util.concurrent.Executors

/**
 * Owns the neural detector on one dedicated thread: it is created, run and closed there only
 * (MediaPipe's GPU delegate may be bound to the thread that created it), and shared by the
 * live pipeline and gallery processing. If the GPU delegate fails at creation or while
 * detecting, it falls back to the CPU delegate once. Errors are kept for the camera report
 * instead of being lost in the log.
 */
class DetectorHost(private val context: Context) : AutoCloseable {
    private val thread = Executors.newSingleThreadExecutor { Thread(it, "detector") }
    private var detector: AircraftDetector? = null
    private var delegate: Delegate? = null
    private var gaveUp = false

    @Volatile var status: String = "henüz çalışmadı"
        private set
    @Volatile private var runs = 0L
    @Volatile private var failures = 0L
    @Volatile private var lastError: String? = null

    /**
     * Runs a detection on the detector thread and waits for it. Null when the detector failed
     * or is unavailable: the caller must not report such a run to the tracker, which would
     * take it for "looked closely, no aircraft" and drop a correct track.
     */
    fun detect(bitmap: Bitmap, timestampNs: Long, region: Box): List<Detection>? =
        try {
            thread.submit(Callable { detectHere(bitmap, timestampNs, region) }).get()
        } catch (e: Exception) {
            null
        }

    /** Short status for the screen: delegate, runs and failures. */
    fun shortStatus(): String = "NN $status $runs/$failures"

    /** One-line summary for the camera report. */
    fun report(): String = "Sinir ağı: $status, $runs çalışma, $failures hata" + (lastError?.let { ", son hata: $it" } ?: "")

    private fun detectHere(bitmap: Bitmap, timestampNs: Long, region: Box): List<Detection>? {
        val d = ensure() ?: return null
        return try {
            val t0 = SystemClock.elapsedRealtimeNanos()
            d.detect(bitmap, timestampNs, region).also {
                runs++
                lastMs = (SystemClock.elapsedRealtimeNanos() - t0) / 1e6
            }
        } catch (e: Exception) {
            failures++
            lastError = e.toString().take(400)
            Log.e(TAG, "detection failed ($delegate)", e)
            if (delegate == Delegate.GPU) {
                // Fall back to the CPU delegate for good.
                runCatching { d.close() }
                detector = null
                create(listOf(Delegate.CPU))
            }
            null
        }
    }

    /** Duration of the last successful detection (ms). */
    @Volatile var lastMs = 0.0
        private set

    private fun ensure(): AircraftDetector? {
        // CPU first: the bundled model is the int8 EfficientDet-Lite0, which MediaPipe's GPU
        // delegate does not support (on a Honor 200 Pro it failed with "ToTensorConverter: input
        // data size does not match" and gave poor tracking); the bench runs it on CPU too.
        if (detector == null && !gaveUp) create(listOf(Delegate.CPU, Delegate.GPU))
        return detector
    }

    private fun create(delegates: List<Delegate>) {
        for (dg in delegates) {
            try {
                detector = AircraftDetector.create(context, dg)
                delegate = dg
                status = "$dg"
                return
            } catch (e: Exception) {
                lastError = "$dg oluşturulamadı: ${e.toString().take(300)}"
                Log.w(TAG, "delegate $dg unavailable", e)
            }
        }
        gaveUp = true
        status = "kullanılamıyor"
    }

    override fun close() {
        thread.execute {
            // Closing reports a detection error left pending in the graph: never crash on it.
            runCatching { detector?.close() }
            detector = null
        }
        thread.shutdown()
    }

    private companion object {
        const val TAG = "DetectorHost"
    }
}
