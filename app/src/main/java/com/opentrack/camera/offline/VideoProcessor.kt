package com.opentrack.camera.offline

import android.content.ContentValues
import android.content.Context
import android.graphics.SurfaceTexture
import android.media.MediaCodec
import android.media.MediaExtractor
import android.media.MediaFormat
import android.media.MediaMetadataRetriever
import android.net.Uri
import android.opengl.EGLSurface
import android.opengl.GLES20
import android.os.Handler
import android.os.HandlerThread
import android.provider.MediaStore
import android.util.Log
import android.view.Surface
import androidx.core.graphics.createBitmap
import com.opentrack.camera.detect.AircraftDetector
import com.opentrack.camera.gl.EglCore
import com.opentrack.camera.gl.FrameRenderer
import com.opentrack.camera.gl.ReadbackTarget
import com.opentrack.camera.gl.YuvRing
import com.opentrack.camera.record.MuxerSink
import com.opentrack.camera.record.VideoEncoder
import com.opentrack.tracking.Box
import com.opentrack.tracking.Detection
import com.opentrack.tracking.DetectionPlanner
import com.opentrack.tracking.FramingController
import com.opentrack.tracking.LookaheadFramer
import com.opentrack.tracking.LumaFrame
import com.opentrack.tracking.NeuralRun
import com.opentrack.tracking.SkyBlobDetector
import com.opentrack.tracking.StabilizerConfig
import com.opentrack.tracking.TargetTracker
import com.opentrack.tracking.TrackState
import com.opentrack.tracking.TrackerConfig
import com.opentrack.tracking.Vec2
import com.opentrack.tracking.VisualMotionEstimator
import com.opentrack.tracking.ZoomConfig
import java.nio.ByteBuffer
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.concurrent.Semaphore
import java.util.concurrent.TimeUnit
import kotlin.math.roundToInt

/**
 * Processes a video from the gallery the way the live pipeline processes the camera: aircraft
 * detection and tracking, look-ahead stabilization, auto zoom, crop to 1080p (720p for small
 * sources). For phones that keep 4K60 for their own camera app: film with it, then process
 * here. There is no gyro for a recorded video, so camera motion comes from the image
 * ([VisualMotionEstimator]) and rolling shutter is not corrected. The audio track is copied.
 *
 * Runs on its own thread with its own GL context; frames are decoded into a SurfaceTexture
 * and handled one at a time (nothing is dropped, unlike live).
 */
class VideoProcessor(
    private val context: Context,
    private val input: Uri,
    private val zoomConfig: ZoomConfig,
    /** Stabilizer settings for [fps] → [StabilizerConfig]. */
    private val stabilizer: (fps: Int) -> StabilizerConfig,
    private val listener: Listener,
) {
    interface Listener {
        /** Progress 0..1 and frames per second of processing, from the worker thread. */
        fun onProgress(fraction: Double, processingFps: Double)
        fun onFinished(output: Uri?, error: String?)
    }

    @Volatile private var cancelled = false
    private val thread = Thread({ run() }, "offline")

    fun start() = thread.start()

    fun cancel() {
        cancelled = true
    }

    private fun run() {
        var out: Uri? = null
        var error: String? = null
        try {
            out = process()
        } catch (e: Throwable) {
            Log.e(TAG, "processing failed", e)
            error = e.message ?: e.toString()
        }
        listener.onFinished(out, if (cancelled && out == null) null else error)
    }

    private fun process(): Uri? {
        // --- source ---
        val extractor = MediaExtractor().apply { setDataSource(context, input, null) }
        val videoIndex = (0 until extractor.trackCount).firstOrNull { extractor.getTrackFormat(it).getString(MediaFormat.KEY_MIME)?.startsWith("video/") == true }
            ?: error("no video track")
        val audioIndex = (0 until extractor.trackCount).firstOrNull { extractor.getTrackFormat(it).getString(MediaFormat.KEY_MIME)?.startsWith("audio/") == true }
        val vFormat = extractor.getTrackFormat(videoIndex)
        extractor.selectTrack(videoIndex)
        val codedW = vFormat.getInteger(MediaFormat.KEY_WIDTH)
        val codedH = vFormat.getInteger(MediaFormat.KEY_HEIGHT)
        val meta = MediaMetadataRetriever().apply { setDataSource(context, input) }
        val rotation = meta.extractMetadata(MediaMetadataRetriever.METADATA_KEY_VIDEO_ROTATION)?.toIntOrNull() ?: 0
        val durationUs = meta.extractMetadata(MediaMetadataRetriever.METADATA_KEY_DURATION)?.toLongOrNull()?.times(1000) ?: 1L
        val frameCount = meta.extractMetadata(MediaMetadataRetriever.METADATA_KEY_VIDEO_FRAME_COUNT)?.toLongOrNull()
        meta.release()
        val fps = when {
            vFormat.containsKey(MediaFormat.KEY_FRAME_RATE) -> vFormat.getNumber(MediaFormat.KEY_FRAME_RATE)!!.toDouble()
            frameCount != null && durationUs > 0 -> frameCount * 1e6 / durationUs
            else -> 30.0
        }.roundToInt().coerceIn(1, 240)
        // The decoder's buffer is unrotated; the renderer turns it upright (see FrameRenderer.imgToTex).
        val quarter = when (((rotation % 360) + 360) % 360) {
            90 -> 3
            180 -> 2
            270 -> 1
            else -> 0
        }
        val srcW = if (quarter % 2 == 1) codedH else codedW
        val srcH = if (quarter % 2 == 1) codedW else codedH
        val big = maxOf(srcW, srcH) >= 2560
        val (outW, outH) = when {
            srcW >= srcH -> if (big) 1920 to 1080 else 1280 to 720
            else -> if (big) 1080 to 1920 else 720 to 1280
        }

        // --- GL ---
        val egl = EglCore()
        val pbuffer = egl.createPbuffer()
        egl.makeCurrent(pbuffer)
        val renderer = FrameRenderer().also { it.displayRotationQuarterTurns = quarter }
        val texture = renderer.createOesTexture()
        val frameReady = Semaphore(0)
        val callbacks = HandlerThread("offline-frames").apply { start() }
        val st = SurfaceTexture(texture).apply { setOnFrameAvailableListener({ frameReady.release() }, Handler(callbacks.looper)) }
        val decoderSurface = Surface(st)
        val texMatrix = FloatArray(16)

        val analysisW = 640
        val analysisH = ((analysisW.toDouble() * srcH / srcW) / 2).roundToInt() * 2
        val scale = srcW.toDouble() / analysisW
        val lumaTarget = ReadbackTarget(analysisW / 4, analysisH)
        val rgbTarget = ReadbackTarget(analysisW, analysisH)
        val roiTarget = ReadbackTarget(ROI, ROI)

        // --- tracking stack, as live but with image-based camera motion ---
        val motion = VisualMotionEstimator()
        val tracker = TargetTracker(motion, TrackerConfig.forSource(srcW, hasGyro = false).copy(imageSpaceSearch = true))
        val detector = AircraftDetector.create(context)
        val planner = DetectionPlanner(srcW, srcH)
        val sky = SkyBlobDetector()
        val framing = FramingController(srcW, srcH, outW.toDouble() / outH, motion).also {
            it.zoom = FramingController.nativeZoom(srcW, outW)
            it.zoomConfig = zoomConfig
        }
        val stab = stabilizer(fps)
        val framer = LookaheadFramer(srcW, srcH, motion, stab)
        val ring = YuvRing(even(outW * CAPTURE_MARGIN), even(outH * CAPTURE_MARGIN), stab.delayFrames + 3)
        val fullBitmap = createBitmap(analysisW, analysisH)
        val roiBitmap = createBitmap(ROI, ROI)
        val neuralEvery = maxOf(1, (fps / 10.0).roundToInt())
        val center = Vec2(srcW / 2.0, srcH / 2.0)

        // --- output ---
        val name = "OTC_" + SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date()) + "_islenmis.mp4"
        val resolver = context.contentResolver
        val outUri = resolver.insert(
            MediaStore.Video.Media.getContentUri(MediaStore.VOLUME_EXTERNAL_PRIMARY),
            ContentValues().apply {
                put(MediaStore.Video.Media.DISPLAY_NAME, name)
                put(MediaStore.Video.Media.MIME_TYPE, "video/mp4")
                put(MediaStore.Video.Media.RELATIVE_PATH, "Movies/OpenTrack")
                put(MediaStore.Video.Media.IS_PENDING, 1)
            },
        ) ?: error("MediaStore insert failed")
        val pfd = resolver.openFileDescriptor(outUri, "rw") ?: error("cannot open output")
        val audioFormat = audioIndex?.let { extractor.getTrackFormat(it) }
        val sink = MuxerSink(pfd.fileDescriptor, audioFormat != null)
        audioFormat?.let { sink.addAudioTrack(it) }
        val encoder = VideoEncoder(outW, outH, fps, if (big) 40_000_000 else 16_000_000, MediaFormat.MIMETYPE_VIDEO_AVC, sink, 0L)
        val encSurface: EGLSurface = egl.createWindowSurface(encoder.inputSurface)

        fun writeOutput(o: LookaheadFramer.Output) {
            val slot = ring.find(o.timestampNs) ?: return
            egl.makeCurrent(encSurface)
            GLES20.glViewport(0, 0, outW, outH)
            ring.draw(slot, o.crop)
            egl.setPresentationTime(encSurface, o.timestampNs)
            egl.swap(encSurface)
            egl.makeCurrent(pbuffer)
        }

        // --- decode loop ---
        val decoder = MediaCodec.createDecoderByType(vFormat.getString(MediaFormat.KEY_MIME)!!)
        decoder.configure(vFormat, decoderSurface, null, 0)
        decoder.start()
        val info = MediaCodec.BufferInfo()
        var inputDone = false
        var frame = 0L
        var lastTargetBox: Box? = null
        val startMs = System.currentTimeMillis()
        var ok = false
        try {
            while (!cancelled) {
                if (!inputDone) {
                    val i = decoder.dequeueInputBuffer(10_000)
                    if (i >= 0) {
                        val buf = decoder.getInputBuffer(i)!!
                        val n = extractor.readSampleData(buf, 0)
                        if (n < 0) {
                            decoder.queueInputBuffer(i, 0, 0, 0, MediaCodec.BUFFER_FLAG_END_OF_STREAM)
                            inputDone = true
                        } else {
                            decoder.queueInputBuffer(i, 0, n, extractor.sampleTime, 0)
                            extractor.advance()
                        }
                    }
                }
                val o = decoder.dequeueOutputBuffer(info, 10_000)
                if (o < 0) continue
                val eos = info.flags and MediaCodec.BUFFER_FLAG_END_OF_STREAM != 0
                val render = info.size > 0
                decoder.releaseOutputBuffer(o, render)
                if (render) {
                    if (!frameReady.tryAcquire(2, TimeUnit.SECONDS)) error("decoder frame timeout")
                    st.updateTexImage()
                    st.getTransformMatrix(texMatrix)
                    val ts = info.presentationTimeUs * 1000

                    // Analysis (every frame; nothing is dropped offline).
                    lumaTarget.bind()
                    renderer.drawPackedLuma(texture, texMatrix, analysisW, analysisH)
                    val luma = ByteArray(analysisW * analysisH)
                    lumaTarget.read().get(luma)
                    val lf = LumaFrame(analysisW, analysisH, luma, ts, scale)
                    motion.process(lf, lastTargetBox)
                    val dets = ArrayList<Detection>()
                    val runs = ArrayList<NeuralRun>()
                    if (detector != null && frame % neuralEvery == 0L) {
                        val prev = tracker.prediction
                        val tgt = if (prev.state == TrackState.IDLE) null else motion.poseAt(ts).toImage(prev.worldAt(ts), center)
                        val region = planner.next(prev, tgt, ts)
                        val target = if (region == null) rgbTarget else roiTarget
                        target.bind()
                        if (region == null) {
                            renderer.drawRegionFiltered(texture, texMatrix, 0f, 0f, 1f, 1f, target.width, target.height)
                        } else {
                            renderer.drawRegionFiltered(
                                texture, texMatrix, (region.left / srcW).toFloat(), (region.top / srcH).toFloat(),
                                (region.width / srcW).toFloat(), (region.height / srcH).toFloat(), target.width, target.height,
                            )
                        }
                        val bmp = if (region == null) fullBitmap else roiBitmap
                        bmp.copyPixelsFromBuffer(target.read())
                        dets += detector.detect(bmp, ts, region ?: Box(srcW / 2.0, srcH / 2.0, srcW.toDouble(), srcH.toDouble()))
                        runs += NeuralRun(ts, region, ROI)
                    }
                    val skyEvery = if (tracker.prediction.state == TrackState.TRACKING) 12 else 3
                    if (frame % skyEvery == 0L) dets += sky.detect(lf)
                    val p = tracker.process(lf, dets, runs)
                    lastTargetBox = if (p.hasTarget) {
                        val c = motion.poseAt(ts).toImage(p.worldAt(ts), center)
                        Box(c.x, c.y, maxOf(p.width, 24 * scale), maxOf(p.height, 24 * scale))
                    } else null

                    // Framing, ring, look-ahead output.
                    val live = framing.update(ts, p)
                    val capture = Box(live.cx, live.cy, live.width * CAPTURE_MARGIN, live.height * CAPTURE_MARGIN)
                    ring.store(renderer, texture, texMatrix, srcW, srcH, capture, ts)
                    framer.addFrame(ts, live.width, live.height, capture)
                    framer.updateTrack(p)
                    framer.emit()?.let { writeOutput(it) }

                    frame++
                    if (frame % 15 == 0L) {
                        val secs = (System.currentTimeMillis() - startMs) / 1000.0
                        listener.onProgress((info.presentationTimeUs.toDouble() / durationUs).coerceIn(0.0, 1.0), frame / maxOf(secs, 1e-3))
                    }
                }
                if (eos) break
            }
            if (!cancelled) {
                for (o in framer.flush()) writeOutput(o)
                ok = true
            }
        } finally {
            runCatching { decoder.stop() }
            decoder.release()
            egl.makeCurrent(pbuffer)
            egl.releaseSurface(encSurface)
            encoder.stop()
            if (ok && audioIndex != null) copyAudio(audioIndex, sink)
            val written = runCatching { sink.stop() }.getOrDefault(false)
            pfd.close()
            ok = ok && written
            if (ok) {
                resolver.update(outUri, ContentValues().apply { put(MediaStore.Video.Media.IS_PENDING, 0) }, null, null)
            } else {
                resolver.delete(outUri, null, null)
            }
            detector?.close()
            ring.release()
            lumaTarget.release()
            rgbTarget.release()
            roiTarget.release()
            renderer.release()
            GLES20.glDeleteTextures(1, intArrayOf(texture), 0)
            decoderSurface.release()
            st.release()
            callbacks.quitSafely()
            egl.releaseSurface(pbuffer)
            egl.release()
            extractor.release()
        }
        return if (ok) outUri else null
    }

    /** Copies the source's audio samples into [sink] (same clock as the video timestamps). */
    private fun copyAudio(track: Int, sink: MuxerSink) {
        val ex = MediaExtractor().apply { setDataSource(context, input, null) }
        try {
            ex.selectTrack(track)
            val buf = ByteBuffer.allocateDirect(1 shl 20)
            val info = MediaCodec.BufferInfo()
            while (true) {
                val n = ex.readSampleData(buf, 0)
                if (n < 0) break
                info.set(0, n, ex.sampleTime, if (ex.sampleFlags and MediaExtractor.SAMPLE_FLAG_SYNC != 0) MediaCodec.BUFFER_FLAG_KEY_FRAME else 0)
                sink.write(false, buf, info)
                ex.advance()
            }
        } finally {
            ex.release()
        }
    }

    private fun even(v: Double) = (v / 2).roundToInt() * 2

    private companion object {
        const val TAG = "VideoProcessor"
        const val ROI = 320
        const val CAPTURE_MARGIN = 1.25
    }
}
