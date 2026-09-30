package com.opentrack.camera.pipeline

import android.graphics.SurfaceTexture
import android.opengl.EGLSurface
import android.opengl.GLES20
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import android.view.Surface
import com.opentrack.camera.camera.CameraProfile
import com.opentrack.camera.gl.EglCore
import com.opentrack.camera.gl.FrameRenderer
import com.opentrack.camera.gl.ReadbackTarget
import com.opentrack.tracking.Box
import com.opentrack.tracking.CameraMotion
import com.opentrack.tracking.CropWindow
import com.opentrack.tracking.FramingController
import com.opentrack.tracking.TargetTracker
import com.opentrack.tracking.TrackPrediction
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit

/**
 * The GL render thread. For every camera frame:
 *  1. read back a small luma image (and occasionally RGBA) for analysis,
 *  2. ask the [FramingController] for the stabilized crop window at this frame's timestamp,
 *  3. render the crop into the encoder surface (1080p) with the camera timestamp,
 *  4. render the preview (wide view or output view),
 *  5. publish an overlay snapshot for the UI.
 */
class RenderPipeline(
    private val profile: CameraProfile,
    private val motion: CameraMotion,
    private val tracker: TargetTracker,
    private val analysis: AnalysisEngine,
    val framing: FramingController,
    private val displayRotationQuarterTurns: Int,
    private val listener: Listener,
) : SurfaceTexture.OnFrameAvailableListener {

    interface Listener {
        /** The surface the camera must stream into. Called on the render thread. */
        fun onCameraSurface(surface: Surface)
        fun onOverlay(state: OverlayState)
    }

    @Volatile var previewMode = PreviewMode.WIDE

    private val thread = HandlerThread("render").apply { start() }
    private val handler = Handler(thread.looper)
    private lateinit var egl: EglCore
    private lateinit var pbuffer: EGLSurface
    private lateinit var renderer: FrameRenderer
    private lateinit var lumaTarget: ReadbackTarget
    private lateinit var rgbTarget: ReadbackTarget
    private var texture = 0
    private var surfaceTexture: SurfaceTexture? = null
    private var cameraSurface: Surface? = null
    private var preview: EGLSurface? = null
    private var encoder: EGLSurface? = null
    private var encoderWidth = 0
    private var encoderHeight = 0
    private val texMatrix = FloatArray(16)
    private var released = false

    private var fpsWindowStart = 0L
    private var fpsFrames = 0
    private var fps = 0.0
    private var frameCounter = 0L

    private val srcW = profile.sourceSize.width
    private val srcH = profile.sourceSize.height

    fun start() = handler.post {
        egl = EglCore()
        pbuffer = egl.createPbuffer()
        egl.makeCurrent(pbuffer)
        renderer = FrameRenderer().also { it.displayRotationQuarterTurns = displayRotationQuarterTurns }
        lumaTarget = ReadbackTarget(analysis.width / 4, analysis.height)
        rgbTarget = ReadbackTarget(analysis.width, analysis.height)
        texture = renderer.createOesTexture()
        val st = SurfaceTexture(texture)
        st.setDefaultBufferSize(srcW, srcH)
        st.setOnFrameAvailableListener(this, handler)
        surfaceTexture = st
        val s = Surface(st)
        cameraSurface = s
        listener.onCameraSurface(s)
    }

    fun setPreviewSurface(surface: Surface?) = runSync {
        preview?.let { egl.makeCurrent(pbuffer); egl.releaseSurface(it) }
        preview = surface?.let { egl.createWindowSurface(it) }
    }

    fun attachEncoder(surface: Surface, width: Int, height: Int) = runSync {
        encoder = egl.createWindowSurface(surface)
        encoderWidth = width
        encoderHeight = height
    }

    fun detachEncoder() = runSync {
        encoder?.let { egl.makeCurrent(pbuffer); egl.releaseSurface(it) }
        encoder = null
    }

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
        } catch (e: Exception) {
            Log.e(TAG, "frame failed", e)
        }
    }

    private fun drawFrame(st: SurfaceTexture) {
        egl.makeCurrent(pbuffer)
        st.updateTexImage()
        st.getTransformMatrix(texMatrix)
        val ts = st.timestamp

        // 1. Analysis readbacks.
        lumaTarget.bind()
        renderer.drawPackedLuma(texture, texMatrix, analysis.width, analysis.height)
        analysis.submitLuma(lumaTarget.read(), ts)
        if (analysis.wantsNeuralFrame()) {
            rgbTarget.bind()
            renderer.drawRegion(texture, texMatrix, 0f, 0f, 1f, 1f, forReadback = true)
            analysis.submitNeuralFrame(rgbTarget.read(), ts)
        }

        // 2. Stabilized crop for this exact frame time.
        val prediction = tracker.prediction
        val crop = framing.update(ts, prediction)
        val l = (crop.left / srcW).toFloat()
        val t = (crop.top / srcH).toFloat()
        val w = (crop.width / srcW).toFloat()
        val h = (crop.height / srcH).toFloat()

        // 3. Encoder.
        encoder?.let { enc ->
            egl.makeCurrent(enc)
            GLES20.glViewport(0, 0, encoderWidth, encoderHeight)
            renderer.drawRegion(texture, texMatrix, l, t, w, h)
            egl.setPresentationTime(enc, ts)
            egl.swap(enc)
        }

        // 4. Preview, letterboxed to the content aspect.
        preview?.let { pv ->
            egl.makeCurrent(pv)
            val (vw, vh) = egl.surfaceSize(pv)
            GLES20.glViewport(0, 0, vw, vh)
            GLES20.glClearColor(0f, 0f, 0f, 1f)
            GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT)
            val aspect = if (previewMode == PreviewMode.WIDE) srcW.toDouble() / srcH else crop.width / crop.height
            val (x, y, cw, ch) = fit(vw, vh, aspect)
            GLES20.glViewport(x, y, cw, ch)
            if (previewMode == PreviewMode.WIDE) renderer.drawRegion(texture, texMatrix, 0f, 0f, 1f, 1f)
            else renderer.drawRegion(texture, texMatrix, l, t, w, h)
            egl.swap(pv)
        }

        // 5. Stats + overlay (every other frame is plenty for the UI).
        fpsFrames++
        if (fpsWindowStart == 0L) fpsWindowStart = ts
        if (ts - fpsWindowStart >= 1_000_000_000L) {
            fps = fpsFrames * 1e9 / (ts - fpsWindowStart)
            fpsFrames = 0
            fpsWindowStart = ts
        }
        if (frameCounter++ % 2 == 0L) publishOverlay(ts, crop, prediction)
    }

    private fun publishOverlay(ts: Long, crop: CropWindow, prediction: TrackPrediction) {
        val offset = motion.offsetAt(ts)
        val target = if (prediction.hasTarget) {
            val c = prediction.worldAt(ts) + offset
            Box(c.x, c.y, prediction.width, prediction.height)
        } else null
        val dets = tracker.latestDetections.map { d ->
            // Move boxes from their (older) capture time to this frame using camera motion.
            val c = d.box.center - motion.offsetAt(d.timestampNs) + offset
            d.copy(box = d.box.movedTo(c))
        }
        listener.onOverlay(
            OverlayState(srcW, srcH, previewMode, crop, framing.zoom, prediction.state, target, dets, fps, analysis.lastAnalysisMs, analysis.lastNeuralMs),
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
    }
}
