package com.opentrack.bench

import com.opentrack.tracking.Box
import com.opentrack.tracking.CameraMotion
import com.opentrack.tracking.CropWindow
import com.opentrack.tracking.Detection
import com.opentrack.tracking.DetectionPlanner
import com.opentrack.tracking.DetectionSource
import com.opentrack.tracking.FramingConfig
import com.opentrack.tracking.FramingController
import com.opentrack.tracking.LookaheadFramer
import com.opentrack.tracking.LumaFrame
import com.opentrack.tracking.NeuralRun
import com.opentrack.tracking.SkyBlobDetector
import com.opentrack.tracking.StabilizerConfig
import com.opentrack.tracking.TargetTracker
import com.opentrack.tracking.TrackPrediction
import com.opentrack.tracking.TrackState
import com.opentrack.tracking.TrackerConfig
import com.opentrack.tracking.Vec2
import com.opentrack.tracking.VisualMotionEstimator
import com.opentrack.tracking.ZoomConfig
import java.io.BufferedInputStream
import java.io.File
import java.io.InputStream
import java.io.FileInputStream
import java.util.Locale
import kotlin.math.roundToInt

/**
 * Replays a video through the tracking stack.
 *
 * Input: raw 8-bit luma frames at analysis resolution (--luma), optional neural detections
 * CSV (frame,x0,y0,x1,y1,score in source px). Output: per-frame CSV with tracker state,
 * target box and crop window. Mirrors the scheduling of the app's AnalysisEngine and the
 * render thread: the causal FramingController sets zoom and the capture window, and the
 * LookaheadFramer produces the delayed, stabilized output crop. Recorded videos have no gyro,
 * so camera motion comes from the VisualMotionEstimator (--motion none disables it).
 *
 * --live: frames arrive on stdin from run_bench.py, which also runs zoomed (ROI) neural
 * detections on request, like the app's DetectionPlanner. Per frame, stdin carries a header
 * line "F n", n result lines "x0 y0 x1 y1 score" (source px) for the previous request (n = -1:
 * none pending), then the luma bytes; the bench answers on stdout with "R x0 y0 x1 y1" (run
 * the detector on this region of this frame) or "N". Full-frame detections still come from
 * --dets, used only for the runs the planner assigns to the whole frame.
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
    val live = a.flag("live")
    val roiEnabled = live && a.opt("roi") != "false"
    // Neural runs happen at ~10 Hz on the frames where --dets has results (run_bench.py's cadence).
    val neuralStep = maxOf(1, (fps * 0.1).roundToInt())
    val planner = DetectionPlanner(srcW, srcH)
    val fullFrameRuns = HashSet<Long>()
    val roiResults = ArrayList<Detection>()
    var roiRequestNs = -1L
    val pendingRuns = ArrayList<NeuralRun>()
    // Camera motion: recorded videos have no gyro, so estimate it from the image (like the
    // app's no-gyro fallback), or ignore it entirely for comparison.
    val estimator = if (a.opt("motion") == "none") null else VisualMotionEstimator()
    val motion: CameraMotion = estimator ?: CameraMotion.NONE
    // Visual motion is less exact than a gyro (clear-sky frames only extrapolate), so the tracker
    // keeps the no-gyro process noise, as the app does for its visual fallback.
    var trackerConfig = TrackerConfig.forSource(srcW, hasGyro = false)
    a.opt("accel-noise")?.let { trackerConfig = trackerConfig.copy(accelNoise = it.toDouble()) }
    a.opt("min-anchor")?.let { trackerConfig = trackerConfig.copy(minAnchorScore = it.toDouble()) }
    if (a.flag("image-search")) trackerConfig = trackerConfig.copy(imageSpaceSearch = true)
    val tracker = TargetTracker(motion, trackerConfig)
    val framing = FramingController(srcW, srcH, outW.toDouble() / outH, motion, FramingConfig(acquireS = a.double("acquire-s", FramingConfig().acquireS)))
    framing.zoom = a.double("zoom", FramingController.nativeZoom(srcW, outW))
    if (a.flag("auto-zoom")) {
        framing.zoomConfig = ZoomConfig(
            auto = true,
            minZoom = a.double("zoom-min", 1.0),
            maxZoom = a.double("zoom-max", 3.0),
            targetFill = a.double("fill", ZoomConfig().targetFill),
            offsetWiden = a.double("offset-widen", ZoomConfig().offsetWiden),
            coastWidenPerS = a.double("coast-widen", ZoomConfig().coastWidenPerS),
        )
    }
    val useLookahead = a.opt("stabilizer") != "old"
    val stabConfig = StabilizerConfig(
        delayFrames = (a.double("lookahead-s", 0.5) * fps).roundToInt(),
        smoothness = a.double("smoothness", 0.6),
        cinematic = a.opt("cinematic") != "false",
        rollStabilization = a.opt("roll") != "false",
        maxTargetOffset = a.double("max-target-offset", StabilizerConfig().maxTargetOffset),
        leadTimeS = a.double("lead-s", StabilizerConfig().leadTimeS),
        pastFactor = a.double("past-factor", StabilizerConfig().pastFactor),
        constraintWeight = a.double("constraint-weight", StabilizerConfig().constraintWeight),
        robustGoalScale = a.double("robust-goal", StabilizerConfig().robustGoalScale),
        targetWeight = a.double("target-weight", StabilizerConfig().targetWeight),
        lockToPoint = a.flag("lock"),
        acquireS = a.double("acquire-s", StabilizerConfig().acquireS),
    )
    val framer = LookaheadFramer(srcW, srcH, motion, stabConfig)
    val captureMargin = a.double("capture-margin", 1.25)
    val sky = SkyBlobDetector()
    val skyEnabled = a.opt("mode") != "neural"
    val neuralEnabled = a.opt("mode") != "sky"

    val out = File(a.str("out")).printWriter()
    out.println("frame,state,tx,ty,tw,th,cx,cy,cw,ch,rot,zoom,conf,ms,n_neural,n_sky,score,anchor,contrast,contrast_ratio,event,reliable,est_dx,est_dy,est_rot,est_features,est_inliers")
    val info = HashMap<Int, FrameInfo>()
    val center = Vec2(srcW / 2.0, srcH / 2.0)
    fun write(frameIndex: Int, crop: CropWindow, trackAt: TrackPrediction?) {
        val fi = info.remove(frameIndex) ?: return
        val t = frameIndex * frameNs
        // The tracker's own position (framing may ignore a tentative lock, see forFraming).
        val src = if (trackAt != null && trackAt.hasTarget) trackAt else fi.pred
        val tgt = if (src.hasTarget) motion.poseAt(t).toImage(src.worldAt(t), center) else null
        val hadTarget = fi.state == TrackState.TRACKING || fi.state == TrackState.COASTING
        out.println(
            String.format(
                Locale.US, "%d,%s,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.5f,%.3f,%.3f,%.2f,%d,%d,%.3f,%.3f,%.2f,%.2f,%s,%d,%.3f,%.3f,%.6f,%d,%d",
                frameIndex, if (tgt == null && hadTarget) TrackState.LOST else fi.state,
                tgt?.x ?: -1.0, tgt?.y ?: -1.0, fi.tw, fi.th,
                crop.cx, crop.cy, crop.width, crop.height, crop.rotation, fi.zoom, fi.conf, fi.ms, fi.nNeural, fi.nSky,
                fi.score, fi.anchor, fi.contrast, fi.contrastRatio, fi.event, if (fi.reliable) 1 else 0, fi.estDx, fi.estDy, fi.estRot, fi.estFeatures, fi.estInliers,
            ),
        )
    }

    val input: InputStream = if (live) BufferedInputStream(System.`in`, 1 shl 20) else BufferedInputStream(FileInputStream(a.str("luma")), 1 shl 20)
    val control = System.out.bufferedWriter()
    var pending = 0 // index into neural (sorted by delivery time)
    var frame = 0
    var totalMs = 0.0
    var maxMs = 0.0
    var lastTargetBox: Box? = null
    while (true) {
        val t = frame * frameNs
        if (live) {
            val header = readLine(input) ?: break
            val n = header.removePrefix("F ").trim().toInt()
            repeat(maxOf(0, n)) {
                val c = readLine(input)!!.trim().split(" ").map { it.toDouble() }
                roiResults += Detection(Box.fromEdges(c[0], c[1], c[2], c[3]), c[4].toFloat(), "airplane", DetectionSource.NEURAL, roiRequestNs)
            }
        }
        val buf = ByteArray(w * h)
        if (input.readNBytes(buf, 0, buf.size) < buf.size) break
        val lf = LumaFrame(w, h, buf, t, scale)
        val t0 = System.nanoTime()
        val step = estimator?.process(lf, lastTargetBox)
        // Neural run on this frame: whole frame or zoomed region, decided from the latest prediction.
        var request = "N"
        if (live && frame % neuralStep == 0) {
            val prev = tracker.prediction
            val tgt = if (prev.hasTarget || prev.state == TrackState.LOST) motion.poseAt(t).toImage(prev.worldAt(t), center) else null
            val roi = if (roiEnabled) planner.next(prev, tgt, t) else null
            pendingRuns += NeuralRun(t, roi)
            if (roi == null) {
                fullFrameRuns += t
            } else {
                request = String.format(Locale.US, "R %.1f %.1f %.1f %.1f", roi.left, roi.top, roi.right, roi.bottom)
                roiRequestNs = t
            }
        }
        val dets = ArrayList<Detection>()
        // Neural results become available after the inference latency, like the async detector thread.
        while (pending < neural.size && neural[pending].timestampNs + latencyNs <= t) {
            val d = neural[pending]
            if (neuralEnabled && (!live || d.timestampNs in fullFrameRuns)) dets += d
            pending++
        }
        roiResults.removeAll { d -> (d.timestampNs + latencyNs <= t).also { if (it && neuralEnabled) dets += d } }
        // Simulates a detector that fails on the device: runs are reported, results never arrive.
        if (a.flag("neural-fails")) dets.removeAll { it.source == DetectionSource.NEURAL }
        val runs = ArrayList<NeuralRun>()
        pendingRuns.removeAll { r -> (r.timestampNs + latencyNs <= t).also { if (it && neuralEnabled) runs += r } }
        val nNeural = dets.size
        val skyEvery = if (tracker.prediction.state == TrackState.TRACKING) 12 else 3
        if (skyEnabled && frame % skyEvery == 0) dets += sky.detect(lf)
        val p = tracker.process(lf, dets, if (a.opt("verify") == "false") emptyList() else runs)
        val ms = (System.nanoTime() - t0) / 1e6
        totalMs += ms
        maxMs = maxOf(maxMs, ms)
        lastTargetBox = if (p.hasTarget) {
            val c = motion.poseAt(t).toImage(p.worldAt(t), center)
            Box(c.x, c.y, maxOf(p.width, 24 * scale), maxOf(p.height, 24 * scale))
        } else null

        // Render thread: causal crop (zoom + capture window), then the delayed stabilized crop.
        val noGate = a.flag("no-framing-gate")
        val framed = if (noGate) p else p.forFraming()
        val causal = framing.update(t, framed)
        val m = tracker.lastMatch
        info[frame] = FrameInfo(
            p.state, p.width, p.height, framing.zoom, p.confidence, ms, nNeural, dets.size - nNeural,
            m?.score ?: 0.0, m?.anchorScore ?: 0.0, m?.contrast ?: 0.0, m?.contrastRatio ?: 0.0, tracker.lastEvent, step?.reliable ?: false,
            step?.shift?.x ?: 0.0, step?.shift?.y ?: 0.0, step?.rotation ?: 0.0, step?.features ?: 0, step?.inliers ?: 0, p,
        )
        if (useLookahead) {
            val capture = if (captureMargin > 0) Box(causal.cx, causal.cy, causal.width * captureMargin, causal.height * captureMargin) else null
            framer.addFrame(t, causal.width, causal.height, capture)
            framer.updateTrack(framed)
            framer.emit()?.let { o -> write((o.timestampNs / frameNs).toInt(), o.crop, o.track) }
        } else {
            write(frame, causal, p)
        }
        if (live) {
            control.write(request)
            control.newLine()
            control.flush()
        }
        frame++
    }
    if (useLookahead) for (o in framer.flush()) write((o.timestampNs / frameNs).toInt(), o.crop, o.track)
    out.close()
    System.err.println(String.format(Locale.US, "frames=%d avgMs=%.2f maxMs=%.2f", frame, totalMs / maxOf(1, frame), maxMs))
}

/** Reads one '\n'-terminated ASCII line, or null at end of stream. */
private fun readLine(input: InputStream): String? {
    val sb = StringBuilder()
    while (true) {
        val c = input.read()
        if (c < 0) return if (sb.isEmpty()) null else sb.toString()
        if (c == '\n'.code) return sb.toString()
        sb.append(c.toChar())
    }
}

private class FrameInfo(
    val state: TrackState, val tw: Double, val th: Double, val zoom: Double, val conf: Double, val ms: Double,
    val nNeural: Int, val nSky: Int, val score: Double, val anchor: Double, val contrast: Double, val contrastRatio: Double,
    val event: String, val reliable: Boolean, val estDx: Double, val estDy: Double, val estRot: Double, val estFeatures: Int, val estInliers: Int,
    val pred: TrackPrediction,
)

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
