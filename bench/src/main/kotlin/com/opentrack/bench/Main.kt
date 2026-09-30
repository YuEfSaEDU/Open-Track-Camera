package com.opentrack.bench

import com.opentrack.tracking.Box
import com.opentrack.tracking.CameraMotion
import com.opentrack.tracking.Detection
import com.opentrack.tracking.DetectionSource
import com.opentrack.tracking.FramingController
import com.opentrack.tracking.LumaFrame
import com.opentrack.tracking.SkyBlobDetector
import com.opentrack.tracking.TargetTracker
import com.opentrack.tracking.TrackState
import com.opentrack.tracking.TrackerConfig
import com.opentrack.tracking.ZoomConfig
import java.io.BufferedInputStream
import java.io.File
import java.io.FileInputStream
import java.util.Locale

/**
 * Replays a video through the tracking stack.
 *
 * Input: raw 8-bit luma frames at analysis resolution (--luma), optional neural detections
 * CSV (frame,x0,y0,x1,y1,score in source px). Output: per-frame CSV with tracker state,
 * target box and crop window. Mirrors the scheduling of the app's AnalysisEngine and the
 * render thread's FramingController usage; there is no gyro in recorded videos, so camera
 * motion is NONE and the tracker runs with the no-gyro process noise.
 */
fun main(args: Array<String>) {
    val a = Args(args)
    val w = a.int("width")
    val h = a.int("height")
    val srcW = a.int("source-width")
    val srcH = a.int("source-height")
    val fps = a.double("fps")
    val frameNs = (1e9 / fps).toLong()
    val latencyNs = (a.double("det-latency-ms", 60.0) * 1e6).toLong()
    val outW = a.int("output-width", 1920)
    val outH = a.int("output-height", 1080)
    val scale = srcW.toDouble() / w

    val neural = a.opt("dets")?.let { loadDetections(File(it), frameNs) } ?: emptyList()
    var trackerConfig = TrackerConfig.forSource(srcW, hasGyro = false)
    a.opt("accel-noise")?.let { trackerConfig = trackerConfig.copy(accelNoise = it.toDouble()) }
    a.opt("min-anchor")?.let { trackerConfig = trackerConfig.copy(minAnchorScore = it.toDouble()) }
    val tracker = TargetTracker(CameraMotion.NONE, trackerConfig)
    val framing = FramingController(srcW, srcH, outW.toDouble() / outH, CameraMotion.NONE)
    framing.zoom = a.double("zoom", FramingController.nativeZoom(srcW, outW))
    if (a.flag("auto-zoom")) {
        framing.zoomConfig = ZoomConfig(
            auto = true,
            minZoom = a.double("zoom-min", 1.0),
            maxZoom = a.double("zoom-max", 3.0),
            targetFill = a.double("fill", 0.25),
        )
    }
    val sky = SkyBlobDetector()
    val skyEnabled = a.opt("mode") != "neural"
    val neuralEnabled = a.opt("mode") != "sky"

    val out = File(a.str("out")).printWriter()
    out.println("frame,state,tx,ty,tw,th,cx,cy,cw,ch,zoom,conf,ms,n_neural,n_sky,score,anchor,contrast,contrast_ratio,event")
    val input = BufferedInputStream(FileInputStream(a.str("luma")), 1 shl 20)
    var pending = 0 // index into neural (sorted by delivery time)
    var frame = 0
    var totalMs = 0.0
    var maxMs = 0.0
    while (true) {
        val buf = ByteArray(w * h)
        if (input.readNBytes(buf, 0, buf.size) < buf.size) break
        val t = frame * frameNs
        val lf = LumaFrame(w, h, buf, t, scale)
        val t0 = System.nanoTime()
        val dets = ArrayList<Detection>()
        // Neural results become available after the inference latency, like the async detector thread.
        while (pending < neural.size && neural[pending].timestampNs + latencyNs <= t) {
            if (neuralEnabled) dets += neural[pending]
            pending++
        }
        val nNeural = dets.size
        val skyEvery = if (tracker.prediction.state == TrackState.TRACKING) 12 else 3
        if (skyEnabled && frame % skyEvery == 0) dets += sky.detect(lf)
        val p = tracker.process(lf, dets)
        val ms = (System.nanoTime() - t0) / 1e6
        totalMs += ms
        maxMs = maxOf(maxMs, ms)
        // Render thread: crop for this frame's timestamp.
        val crop = framing.update(t, p)
        val tgt = if (p.hasTarget) p.worldAt(t) else null
        val m = tracker.lastMatch
        out.println(
            String.format(
                Locale.US, "%d,%s,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.3f,%.3f,%.2f,%d,%d,%.3f,%.3f,%.2f,%.2f,%s",
                frame, p.state, tgt?.x ?: -1.0, tgt?.y ?: -1.0, p.width, p.height,
                crop.cx, crop.cy, crop.width, crop.height, framing.zoom, p.confidence, ms, nNeural, dets.size - nNeural,
                m?.score ?: 0.0, m?.anchorScore ?: 0.0, m?.contrast ?: 0.0, m?.contrastRatio ?: 0.0, tracker.lastEvent,
            ),
        )
        frame++
    }
    out.close()
    System.err.println(String.format(Locale.US, "frames=%d avgMs=%.2f maxMs=%.2f", frame, totalMs / maxOf(1, frame), maxMs))
}

private fun loadDetections(file: File, frameNs: Long): List<Detection> =
    file.readLines().drop(1).filter { it.isNotBlank() }.map { line ->
        val c = line.split(",")
        Detection(
            box = Box.fromEdges(c[1].toDouble(), c[2].toDouble(), c[3].toDouble(), c[4].toDouble()),
            score = c[5].toFloat(),
            label = "airplane",
            source = DetectionSource.NEURAL,
            timestampNs = c[0].toLong() * frameNs,
        )
    }.sortedBy { it.timestampNs }

private class Args(argv: Array<String>) {
    private val map = HashMap<String, String>()
    private val flags = HashSet<String>()

    init {
        var i = 0
        while (i < argv.size) {
            val k = argv[i].removePrefix("--")
            if (i + 1 < argv.size && !argv[i + 1].startsWith("--")) {
                map[k] = argv[i + 1]
                i += 2
            } else {
                flags += k
                i++
            }
        }
    }

    fun opt(k: String) = map[k]
    fun str(k: String) = map[k] ?: error("missing --$k")
    fun int(k: String, d: Int? = null) = map[k]?.toInt() ?: d ?: error("missing --$k")
    fun double(k: String, d: Double? = null) = map[k]?.toDouble() ?: d ?: error("missing --$k")
    fun flag(k: String) = k in flags
}
