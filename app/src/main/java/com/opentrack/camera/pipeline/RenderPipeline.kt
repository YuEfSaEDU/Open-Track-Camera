package com.opentrack.camera.pipeline

import android.graphics.SurfaceTexture
import android.opengl.EGLSurface
import android.opengl.GLES20
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import android.view.Surface
import com.opentrack.camera.camera.CameraProfile
import com.opentrack.camera.diag.Telemetry
import com.opentrack.camera.gl.EglCore
import com.opentrack.camera.gl.FrameRenderer
import com.opentrack.camera.gl.ReadbackTarget
import com.opentrack.camera.gl.YuvRing
import com.opentrack.tracking.Box
import com.opentrack.tracking.CameraMotion
import com.opentrack.tracking.CropWindow
import com.opentrack.tracking.FramingController
import com.opentrack.tracking.FrameLens
import com.opentrack.tracking.HybridZoom
import com.opentrack.tracking.LensModel
import com.opentrack.tracking.LookaheadFramer
import com.opentrack.tracking.PerspectiveView
import com.opentrack.tracking.RollingShutter
import com.opentrack.tracking.StabilizerConfig
import com.opentrack.tracking.TargetTracker
import com.opentrack.tracking.TrackPrediction
import com.opentrack.tracking.Vec2
import com.opentrack.tracking.ZoomMap
import com.opentrack.tracking.intersect
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import kotlin.math.max
import kotlin.math.min

/**
 * The GL render thread. For every camera frame:
 *  1. read back a small luma image (and occasionally RGBA) for analysis,
 *  2. ask the causal [FramingController] for zoom and the expected crop (the live view),
 *  3. store the region around it ("capture window") in the [YuvRing],
 *  4. let the [LookaheadFramer] produce the stabilized crop of the frame from
 *     [StabilizerConfig.delayFrames] frames ago, now that its future is known, and render it
 *     from the ring into the encoder (with that frame's camera timestamp) and output preview,
 *  5. render the live preview and publish an overlay snapshot for the UI.
 */
class RenderPipeline(
    private val profile: CameraProfile,
    private val motion: CameraMotion,
    private val tracker: TargetTracker,
    private val analysis: AnalysisEngine,
    val framing: FramingController,
    stabilizer: StabilizerConfig,
    private val outputWidth: Int,
    private val outputHeight: Int,
    private val displayRotationQuarterTurns: Int,
    private val listener: Listener,
    /** Render with 10-bit colour (for a 10-bit HEVC recording). */
    private val tenBit: Boolean = false,
) : SurfaceTexture.OnFrameAvailableListener {

    interface Listener {
        /** The surface the camera must stream into. Called on the render thread. */
        fun onCameraSurface(surface: Surface)
        fun onOverlay(state: OverlayState)
        /** Every frame has been failing to render (e.g. a 10-bit surface the phone cannot draw); once. */
        fun onRenderError(message: String) {}
        /** Hybrid zoom wants the camera at this zoom ratio (1 = main lens, else the telephoto's). */
        fun onZoomRequest(ratio: Double) {}
    }

    /**
     * The zoom ratio a frame was taken with (from its capture result), by sensor timestamp; null
     * until the result is in. Tracking, framing and stabilization run in the reference space
     * (main lens at 1x); each frame is mapped into it with its own ratio, so a zoom or lens
     * change keeps them continuous.
     */
    @Volatile var zoomAt: ((Long) -> Double?)? = null

    /** The camera zoom ratio that is the reference space (the user's; hybrid zoom goes beyond it). */
    @Volatile var baseZoom = 1.0

    /** Picks the zoom ratio (lens) of a multi-camera from the crop, relative to [baseZoom]; null = fixed zoom. */
    @Volatile var hybrid: HybridZoom? = null

    /** Which physical lens took the frame of (timestamp, zoom ratio): the key of its [lensModel]. */
    @Volatile var lensKeyAt: ((Long, Double) -> String)? = null

    /**
     * The lens's real center focal and distortion (see LensModel), measured on the phone; null
     * = trust the zoom ratio, no distortion. Frames are mapped to the reference space with it
     * and drawn rectified, so lines stay straight when stabilization moves the crop.
     */
    @Volatile var lensModel: ((String) -> LensModel?)? = null

    /** The zoom ratio last asked of the camera (by hybrid zoom or the user). */
    @Volatile var requestedZoom = 1.0
        private set
    private var requestedAtNs = 0L
    private var lastZoom = 1.0
    private var zoomReported = false

    /** The mapping of the latest camera frame (for the overlay and taps). */
    @Volatile var frameMap: ZoomMap = ZoomMap.identity(profile.sourceSize.width, profile.sourceSize.height)
        private set

    /** The user (camera zoom slider) or hybrid zoom asked the camera for [ratio]. */
    fun zoomRequested(ratio: Double) {
        // Only a jump (a lens change) is worth waiting for: small steps are mapped with the
        // previous ratio for the frame or two before their results are in.
        if (kotlin.math.abs(kotlin.math.ln(ratio / requestedZoom)) > 0.05) requestedAtNs = System.nanoTime()
        requestedZoom = ratio
    }

    private var lastZoomRequestNs = 0L

    /** The last render error, for the camera report. */
    @Volatile var lastError: String? = null
        private set
    private var failedFrames = 0

    @Volatile var previewMode = PreviewMode.WIDE

    /** The GL output really is 10-bit (the phone offered a 10-bit EGL config). */
    @Volatile var tenBitActive = false
        private set

    /**
     * While recording: the live crop and camera pose of every frame ("live") and the crop each
     * recorded frame was rendered with ("out", with its time in the video).
     */
    @Volatile var telemetry: Telemetry? = null
    private var firstWrittenNs = Long.MIN_VALUE

    /**
     * Rolling shutter timing of the camera stream; with gyro motion the output is corrected for
     * it (null = off, e.g. without a gyroscope: image-based motion has no per-row timing).
     */
    @Volatile var rollingShutter: RollingShutter? = null
    private var shutterTs = Long.MIN_VALUE
    private var shutter: YuvRing.ShutterCorrection? = null

    /**
     * Focal length of the stream in source px (lens zoom included). Above 0, each output crop
     * is rendered as a virtual camera turned towards it (PerspectiveView): a target held in
     * frame while the phone turns keeps its shape instead of stretching and keystoning like
     * a photo bent in perspective near the edges of the wide lens.
     */
    @Volatile var focalPx: Double = 0.0
    private var view: PerspectiveView? = null
    private var viewTs = Long.MIN_VALUE
    private var viewStrength = 0.0
    private var viewHomography: DoubleArray? = null

    private val thread = HandlerThread("render").apply { start() }
    private val handler = Handler(thread.looper)
    private lateinit var egl: EglCore
    private lateinit var pbuffer: EGLSurface
    private lateinit var renderer: FrameRenderer
    private lateinit var lumaTarget: ReadbackTarget
    private lateinit var rgbTarget: ReadbackTarget
    private lateinit var roiTarget: ReadbackTarget
    private var ring: YuvRing? = null
    private var texture = 0
    private var surfaceTexture: SurfaceTexture? = null
    private var cameraSurface: Surface? = null
    private var preview: EGLSurface? = null
    private var encoder: EGLSurface? = null
    private var encoderWidth = 0
    private var encoderHeight = 0
    private var recordFromNs = Long.MAX_VALUE
    private val texMatrix = FloatArray(16)
    private var released = false

    private val srcW = profile.sourceSize.width
    private val srcH = profile.sourceSize.height
    private var framer = LookaheadFramer(srcW, srcH, motion, stabilizer)
    private var lastOutput: LookaheadFramer.Output? = null

    private var lastKeptNs = Long.MIN_VALUE
    /** Slightly under the output frame interval, so timestamp jitter does not drop frames. */
    private val keepIntervalNs = (1_000_000_000L / profile.fps) * 3 / 4

    private var fpsWindowStart = 0L
    private var fpsFrames = 0
    private var fps = 0.0
    private var frameCounter = 0L

    fun start() = handler.post {
        egl = EglCore(tenBit)
        tenBitActive = egl.tenBit
        pbuffer = egl.createPbuffer()
        egl.makeCurrent(pbuffer)
        renderer = FrameRenderer().also { it.displayRotationQuarterTurns = displayRotationQuarterTurns }
        lumaTarget = ReadbackTarget(analysis.width / 4, analysis.height)
        rgbTarget = ReadbackTarget(analysis.width, analysis.height)
        roiTarget = ReadbackTarget(AnalysisEngine.ROI_SIZE, AnalysisEngine.ROI_SIZE)
        createRing()
        texture = renderer.createOesTexture()
        val st = SurfaceTexture(texture)
        st.setDefaultBufferSize(srcW, srcH)
        st.setOnFrameAvailableListener(this, handler)
        surfaceTexture = st
        val s = Surface(st)
        cameraSurface = s
        listener.onCameraSurface(s)
    }

    /**
     * Applies new stabilizer settings. A different look-ahead needs a different ring, so it is
     * only accepted while not recording; returns false when it was rejected.
     */
    fun setStabilizer(config: StabilizerConfig): Boolean {
        var ok = true
        runSync {
            if (config.delayFrames != framer.config.delayFrames) {
                if (encoder != null) {
                    ok = false
                    return@runSync
                }
                framer = LookaheadFramer(srcW, srcH, motion, config)
                lastOutput = null
                createRing()
            } else {
                framer.config = config
            }
        }
        return ok
    }

    fun setPreviewSurface(surface: Surface?) = runSync {
        preview?.let { egl.makeCurrent(pbuffer); egl.releaseSurface(it) }
        preview = surface?.let { egl.createWindowSurface(it) }
    }

    fun attachEncoder(surface: Surface, width: Int, height: Int) = runSync {
        encoder = egl.createWindowSurface(surface)
        encoderWidth = width
        encoderHeight = height
        // Only frames captured from now on go into the file (older ones are still in the ring).
        recordFromNs = Long.MAX_VALUE
        firstWrittenNs = Long.MIN_VALUE
        pendingRecordStart = true
    }

    /** Stops recording: the frames still waiting for look-ahead are finished and written first. */
    fun detachEncoder() = runSync {
        encoder?.let { enc ->
            for (o in framer.flush()) writeEncoder(enc, o)
            egl.makeCurrent(pbuffer)
            egl.releaseSurface(enc)
        }
        encoder = null
        recordFromNs = Long.MAX_VALUE
    }

    private var pendingRecordStart = false

    fun release() {
        runSync {
            released = true
            surfaceTexture?.setOnFrameAvailableListener(null)
            preview?.let { egl.releaseSurface(it) }
            encoder?.let { egl.releaseSurface(it) }
            preview = null
            encoder = null
            egl.makeCurrent(pbuffer)
            lumaTarget.release()
            rgbTarget.release()
            roiTarget.release()
            ring?.release()
            renderer.release()
            GLES20.glDeleteTextures(1, intArrayOf(texture), 0)
            cameraSurface?.release()
            surfaceTexture?.release()
            egl.releaseSurface(pbuffer)
            egl.release()
        }
        thread.quitSafely()
    }

    override fun onFrameAvailable(st: SurfaceTexture) {
        if (released) return
        try {
            drawFrame(st)
            failedFrames = 0
        } catch (e: Exception) {
            Log.e(TAG, "frame failed", e)
            lastError = e.toString().take(300)
            if (++failedFrames == FAILED_FRAMES_REPORT) listener.onRenderError(lastError ?: "?")
        }
    }

    private fun createRing() {
        egl.makeCurrent(pbuffer)
        ring?.release()
        // The capture window is CAPTURE_MARGIN times the crop, stored at output resolution.
        val w = even(outputWidth * CAPTURE_MARGIN)
        val h = even(outputHeight * CAPTURE_MARGIN)
        ring = YuvRing(w, h, framer.config.delayFrames + 3).also {
            Log.i(TAG, "look-ahead ${framer.config.delayFrames} frames, ring ${it.bytes / (1 shl 20)} MB")
        }
    }

    private fun drawFrame(st: SurfaceTexture) {
        egl.makeCurrent(pbuffer)
        st.updateTexImage()
        st.getTransformMatrix(texMatrix)
        val ts = st.timestamp
        // High-speed capture (e.g. 120 fps for a 60 fps pipeline): keep one frame per output
        // interval, chosen by timestamp since high-speed frames arrive in bursts.
        if (profile.captureFps > profile.fps) {
            if (lastKeptNs != Long.MIN_VALUE && ts - lastKeptNs < keepIntervalNs) return
            lastKeptNs = if (lastKeptNs == Long.MIN_VALUE || ts - lastKeptNs > 2 * keepIntervalNs) ts else lastKeptNs + 1_000_000_000L / profile.fps
        }
        if (pendingRecordStart) {
            recordFromNs = ts
            pendingRecordStart = false
        }

        // 0. Where this frame sits in the reference space (its own zoom ratio / lens).
        val z = frameZoom(ts)
        // (Known for sure only once the frame's capture result is in; see YuvRing.remap.)
        val exactZoom = zoomAt?.invoke(ts) != null || !zoomReported
        val lensKey = lensKeyAt?.invoke(ts, z)
        val model = lensKey?.let { lensModel?.invoke(it) }
        val f = focalPx
        val map = mapFor(z, model)
        renderer.setLens(model?.a ?: 0.0, (model?.focalAt(z) ?: srcW.toDouble()) / srcW, srcW.toDouble() / srcH)
        val frameLens = if (lensKey != null && model != null && f > 0) FrameLens(lensKey, z, map.zoom, model, f) else null
        frameMap = map
        val visible = if (map.zoom > 1.0) map.visible else null
        val whole = map.toStream(Box(srcW / 2.0, srcH / 2.0, srcW.toDouble(), srcH.toDouble()))
        // A longer lens's frame is analysed at its own resolution: the image covers just the part
        // of the reference frame it shows (else the reference frame, black around the frame).
        val window = if (visible != null && analysis.supportsWindows) visible else null
        val analysed = if (window != null) map.toStream(window) else whole

        // 1. Analysis readbacks.
        lumaTarget.bind()
        renderer.drawPackedLuma(
            texture, texMatrix, analysis.width, analysis.height,
            (analysed.left / srcW).toFloat(), (analysed.top / srcH).toFloat(), (analysed.width / srcW).toFloat(), (analysed.height / srcH).toFloat(),
        )
        analysis.submitLuma(lumaTarget.read(), ts, visible, window, frameLens)
        if (analysis.wantsNeuralFrame()) {
            // The whole frame, or a zoomed region around the target (see DetectionPlanner).
            val region = analysis.neuralRegion(ts, visible)
            val target = if (region == null) rgbTarget else roiTarget
            target.bind()
            val s = if (region == null) analysed else map.toStream(region)
            renderer.drawRegionFiltered(
                texture, texMatrix, (s.left / srcW).toFloat(), (s.top / srcH).toFloat(),
                (s.width / srcW).toFloat(), (s.height / srcH).toFloat(), target.width, target.height,
            )
            analysis.submitNeuralFrame(target.read(), ts, region, if (region == null) window else null)
        }

        // 2-3. Causal (live) crop sets zoom and where to capture; keep that region in the ring.
        val prediction = tracker.prediction
        // A tentative lock (not yet confirmed as an aircraft) is tracked but not framed.
        val framed = prediction.forFraming()
        framing.visible = visible
        val live = framing.update(ts, framed)
        analysis.meterCrop = live
        hybrid?.let { hz ->
            val wasTele = hz.onTele
            val r = hz.update(ts, live) * baseZoom
            // The camera zoom follows the crop continuously, in small steps up to ~30 per second (each
            // is a new repeating request, like a pinch zoom); a lens change goes out at once.
            val change = kotlin.math.abs(kotlin.math.ln(r / requestedZoom))
            val now = System.nanoTime()
            if (hz.onTele != wasTele || (change >= ZOOM_REQUEST_STEP && now - lastZoomRequestNs >= ZOOM_REQUEST_INTERVAL_NS) || (change > 1e-6 && (r == baseZoom))) {
                lastZoomRequestNs = now
                zoomRequested(r)
                listener.onZoomRequest(r)
            }
        }
        val capture = captureFor(live)
        ring?.store(renderer, texture, texMatrix, srcW, srcH, capture, ts, map, exactZoom)
        telemetry?.let { t ->
            val pose = motion.poseAt(ts)
            t.line("live").i("ts", ts).a("crop", live.cx, live.cy, live.width, live.height).n("rot", live.rotation, 4)
                .n("zoom", framing.zoom, 3).a("pose", pose.offset.x, pose.offset.y).n("roll", pose.roll, 4)
                .s("state", prediction.state.name).b("framed", framed.hasTarget).n("z", map.zoom, 2).send()
        }

        // 4. Stabilized output for the delayed frame.
        // The look-ahead path may only use pixels the camera has (inside a telephoto frame).
        framer.addFrame(ts, live.width, live.height, visible?.let { capture.intersect(it) } ?: capture)
        framer.updateTrack(framed)
        framer.emit()?.let { out ->
            lastOutput = out
            encoder?.let { writeEncoder(it, out) }
        }

        // 5. Preview, letterboxed to the content aspect.
        preview?.let { pv ->
            egl.makeCurrent(pv)
            val (vw, vh) = egl.surfaceSize(pv)
            GLES20.glViewport(0, 0, vw, vh)
            GLES20.glClearColor(0f, 0f, 0f, 1f)
            GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT)
            val out = lastOutput
            if (previewMode == PreviewMode.OUTPUT && out != null) {
                val (x, y, cw, ch) = fit(vw, vh, out.crop.width / out.crop.height)
                GLES20.glViewport(x, y, cw, ch)
                ring?.let { r -> findSlot(r, out.timestampNs)?.let { r.draw(it, out.crop, shutterFor(out.timestampNs, it.zoom), homographyFor(out, it), YuvRing.sharpenFor(outputWidth / (out.crop.width * it.zoom))) } }
            } else {
                val (x, y, cw, ch) = fit(vw, vh, srcW.toDouble() / srcH)
                GLES20.glViewport(x, y, cw, ch)
                // The camera's own frame (whatever lens hybrid zoom is on); the overlay is mapped to it.
                renderer.drawRegion(texture, texMatrix, 0f, 0f, 1f, 1f)
            }
            egl.swap(pv)
        }

        // Stats + overlay (every other frame is plenty for the UI).
        fpsFrames++
        if (fpsWindowStart == 0L) fpsWindowStart = ts
        if (ts - fpsWindowStart >= 1_000_000_000L) {
            fps = fpsFrames * 1e9 / (ts - fpsWindowStart)
            fpsFrames = 0
            fpsWindowStart = ts
        }
        if (frameCounter++ % 2 == 0L) {
            // The overlay must match what is on screen: the live frame, or the delayed output.
            val out = lastOutput
            if (previewMode == PreviewMode.OUTPUT && out != null) publishOverlay(out.timestampNs, out.crop, out.track ?: prediction)
            else publishOverlay(ts, live, prediction, map)
        }
    }

    private fun writeEncoder(enc: EGLSurface, out: LookaheadFramer.Output) {
        if (out.timestampNs < recordFromNs) return
        val slot = ring?.let { findSlot(it, out.timestampNs) } ?: return
        egl.makeCurrent(enc)
        GLES20.glViewport(0, 0, encoderWidth, encoderHeight)
        val h = homographyFor(out, slot)
        // Enlargement of the camera's own pixels: a telephoto slot has zoom x more per reference px.
        ring?.draw(slot, out.crop, shutterFor(out.timestampNs, slot.zoom), h, YuvRing.sharpenFor(encoderWidth / (out.crop.width * slot.zoom)))
        egl.setPresentationTime(enc, out.timestampNs)
        egl.swap(enc)
        telemetry?.let { t ->
            if (firstWrittenNs == Long.MIN_VALUE) firstWrittenNs = out.timestampNs
            val c = out.crop
            val line = t.line("out").n("t", (out.timestampNs - firstWrittenNs) / 1e9, 4).i("ts", out.timestampNs)
                .a("crop", c.cx, c.cy, c.width, c.height).n("rot", c.rotation, 4).n("persp", if (h == null) 0.0 else viewStrength, 2)
            out.track?.takeIf { it.hasTarget }?.let { p ->
                val img = motion.poseAt(out.timestampNs).toImage(p.worldAt(out.timestampNs), Vec2(srcW / 2.0, srcH / 2.0))
                line.a("target", img.x, img.y, p.width, p.height).s("state", p.state.name)
            }
            line.send()
        }
    }

    /**
     * Region of the frame kept in the ring: the live crop with room around it. With the
     * perspective view on, it shifts (within its margin) towards the side where the corrected
     * view of an off-axis crop needs more pixels.
     */
    /**
     * The zoom ratio frame [ts] was taken with. Its capture result usually arrives before the
     * frame; right after a zoom request it may not have, and guessing wrong there would jump the
     * picture for a frame, so it is briefly waited for.
     */
    /** Reference space → frame of zoom ratio [z] taken by a lens with [model] (null: trust the ratio). */
    private fun mapFor(z: Double, model: LensModel?): ZoomMap {
        val f = focalPx
        return ZoomMap(if (model != null && f > 0) model.focalAt(z) / f else z / baseZoom, srcW, srcH)
    }

    /** The ring slot of frame [ts], its mapping corrected if it was stored before the frame's zoom was known. */
    private fun findSlot(r: YuvRing, ts: Long): YuvRing.Slot? {
        val slot = r.find(ts) ?: return null
        if (!slot.exact) {
            val z = zoomAt?.invoke(ts)
            if (z != null && z > 0) {
                val model = lensKeyAt?.invoke(ts, z)?.let { lensModel?.invoke(it) }
                r.remap(slot, mapFor(z, model))
            }
        }
        return slot
    }

    private fun frameZoom(ts: Long): Double {
        val lookup = zoomAt ?: return lastZoom
        var z = lookup(ts)
        if (z == null && System.nanoTime() - requestedAtNs < ZOOM_WAIT_WINDOW_NS) {
            val until = System.nanoTime() + ZOOM_WAIT_NS
            while (z == null && System.nanoTime() < until) {
                Thread.sleep(1)
                z = lookup(ts)
            }
        }
        if (z != null && z > 0) {
            lastZoom = z
            zoomReported = true
        }
        // A camera that never reports its zoom ratio: assume it does what it was asked.
        return if (zoomReported) lastZoom else requestedZoom
    }

    private fun captureFor(live: CropWindow): Box {
        var cx = live.cx
        var cy = live.cy
        val f = focalPx
        if (f > 0) {
            val v = view?.takeIf { it.focalPx == f } ?: PerspectiveView(srcW, srcH, f).also { view = it }
            val fp = v.footprint(live)
            val maxX = (CAPTURE_MARGIN - 1) / 2 * live.width
            val maxY = (CAPTURE_MARGIN - 1) / 2 * live.height
            cx += (fp.cx - live.cx).coerceIn(-maxX, maxX)
            cy += (fp.cy - live.cy).coerceIn(-maxY, maxY)
        }
        return Box(cx, cy, live.width * CAPTURE_MARGIN, live.height * CAPTURE_MARGIN)
    }

    /**
     * Virtual camera view for an output frame (computed once per frame), or null for the flat
     * crop. Where the corrected view would need pixels outside the stored capture window or
     * the sensor (the outer side of an off-axis view is larger), the correction backs off
     * just enough, and comes back gradually so the view never changes shape abruptly.
     */
    private fun homographyFor(out: LookaheadFramer.Output, slot: YuvRing.Slot): DoubleArray? {
        val f = focalPx
        if (f <= 0) return null
        if (out.timestampNs == viewTs) return viewHomography
        viewTs = out.timestampNs
        val v = view?.takeIf { it.focalPx == f } ?: PerspectiveView(srcW, srcH, f).also { view = it }
        // Only the part of the slot with camera pixels (inside the sensor, inside a telephoto frame).
        val cap = slot.valid ?: slot.capture ?: return null
        val l = max(cap.left, 0.0)
        val t = max(cap.top, 0.0)
        val r = min(cap.right, srcW.toDouble())
        val b = min(cap.bottom, srcH.toDouble())
        val region = Box((l + r) / 2, (t + b) / 2, r - l, b - t)
        val fit = v.fittingStrength(out.crop, region, margin = 2.0)
        viewStrength = min(fit, viewStrength + VIEW_STRENGTH_RISE)
        viewHomography = v.homography(out.crop, viewStrength)
        return viewHomography
    }

    /** Rolling shutter correction for the frame of [ts] (computed once per output frame). */
    private fun shutterFor(ts: Long, zoom: Double): YuvRing.ShutterCorrection? {
        val rs = rollingShutter ?: return null
        if (ts != shutterTs) {
            shutterTs = ts
            shutter = YuvRing.ShutterCorrection(
                rs.corrections(motion, ts, YuvRing.ShutterCorrection.SAMPLES),
                                // A zoomed frame's readout sweeps only the part of the reference frame it shows.
                rs.readoutDir.x.toFloat(), rs.readoutDir.y.toFloat(), (rs.extentPx / zoom).toFloat(), srcW / 2f, srcH / 2f,
            )
        }
        return shutter
    }

    /** [map]: the wide view shows the camera frame, boxes (reference px) are mapped into it. */
    private fun publishOverlay(ts: Long, crop: CropWindow, prediction: TrackPrediction, map: ZoomMap? = null) {
        val pose = motion.poseAt(ts)
        val center = Vec2(srcW / 2.0, srcH / 2.0)
        val target = if (prediction.hasTarget) {
            val c = pose.toImage(prediction.worldAt(ts), center)
            Box(c.x, c.y, prediction.width, prediction.height)
        } else null
        val dets = tracker.latestDetections.map { d ->
            // Move boxes from their capture time to the displayed frame using camera motion.
            val c = pose.toImage(motion.poseAt(d.timestampNs).toWorld(d.box.center, center), center)
            d.copy(box = d.box.movedTo(c))
        }
        val m = map?.takeIf { it.zoom != 1.0 }
        listener.onOverlay(
            OverlayState(
                srcW, srcH, previewMode,
                m?.let { z -> z.toStream(crop.center).let { c -> crop.copy(cx = c.x, cy = c.y, width = crop.width * z.zoom, height = crop.height * z.zoom) } } ?: crop,
                framing.zoom, prediction.state, m?.let { z -> target?.let { z.toStream(it) } } ?: target,
                m?.let { z -> dets.map { it.copy(box = z.toStream(it.box)) } } ?: dets, fps, analysis.lastAnalysisMs, analysis.lastNeuralMs,
                targetConfirmed = prediction.framable,
            ),
        )
    }

    private fun fit(vw: Int, vh: Int, aspect: Double): IntArray {
        var w = vw
        var h = (vw / aspect).toInt()
        if (h > vh) {
            h = vh
            w = (vh * aspect).toInt()
        }
        return intArrayOf((vw - w) / 2, (vh - h) / 2, w, h)
    }

    private fun even(v: Double): Int = (v / 2).toInt() * 2

    private fun runSync(block: () -> Unit) {
        val latch = CountDownLatch(1)
        handler.post {
            try {
                block()
            } catch (e: Exception) {
                Log.e(TAG, "render task failed", e)
            } finally {
                latch.countDown()
            }
        }
        latch.await(3, TimeUnit.SECONDS)
    }

    private companion object {
        const val TAG = "RenderPipeline"

        /** Capture window size relative to the crop: room for the look-ahead path to deviate. */
        const val CAPTURE_MARGIN = 1.25

        /** Consecutive failed frames (~0.5 s at 60 fps) before the failure is reported. */
        const val FAILED_FRAMES_REPORT = 30

        /** Per output frame: how fast the perspective correction returns after backing off. */
        const val VIEW_STRENGTH_RISE = 0.02

        /** After a zoom request: frames whose zoom ratio is not known yet wait for it, up to... */
        const val ZOOM_WAIT_WINDOW_NS = 500_000_000L
        const val ZOOM_WAIT_NS = 12_000_000L

        /** Hybrid zoom: smallest camera zoom change worth a request (ln), and the least time between them. */
        const val ZOOM_REQUEST_STEP = 0.003
        const val ZOOM_REQUEST_INTERVAL_NS = 33_000_000L
    }
}
