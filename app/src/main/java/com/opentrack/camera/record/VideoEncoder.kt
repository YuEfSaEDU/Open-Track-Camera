package com.opentrack.camera.record

import android.media.MediaCodec
import android.media.MediaCodecInfo
import android.media.MediaFormat
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import android.view.Surface
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit

/**
 * Surface-input hardware video encoder. Frames are rendered into [inputSurface] by the GL
 * pipeline with eglPresentationTimeANDROID set to the camera timestamp; [ptsOffsetNs] maps
 * that camera clock to CLOCK_MONOTONIC for the muxer.
 */
class VideoEncoder(
    val width: Int,
    val height: Int,
    fps: Int,
    bitrate: Int,
    val codec: VideoCodec,
    private val sink: MuxerSink,
    private val ptsOffsetNs: Long,
) {
    private val thread = HandlerThread("video-encoder").apply { start() }
    private val encoder = MediaCodec.createEncoderByType(codec.mime)
    private val done = CountDownLatch(1)
    val inputSurface: Surface

    init {
        val format = MediaFormat.createVideoFormat(codec.mime, width, height).apply {
            setInteger(MediaFormat.KEY_COLOR_FORMAT, MediaCodecInfo.CodecCapabilities.COLOR_FormatSurface)
            setInteger(MediaFormat.KEY_BIT_RATE, bitrate)
            setInteger(MediaFormat.KEY_FRAME_RATE, fps)
            setInteger(MediaFormat.KEY_OPERATING_RATE, fps)
            setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, 1)
            setInteger(MediaFormat.KEY_BITRATE_MODE, MediaCodecInfo.EncoderCapabilities.BITRATE_MODE_VBR)
            codec.configure(this)
        }
        encoder.setCallback(object : MediaCodec.Callback() {
            override fun onInputBufferAvailable(codec: MediaCodec, index: Int) = Unit

            override fun onOutputBufferAvailable(codec: MediaCodec, index: Int, info: MediaCodec.BufferInfo) {
                val buffer = codec.getOutputBuffer(index)
                if (buffer != null && info.size > 0 && info.flags and MediaCodec.BUFFER_FLAG_CODEC_CONFIG == 0) {
                    info.presentationTimeUs += ptsOffsetNs / 1000
                    sink.write(true, buffer, info)
                }
                codec.releaseOutputBuffer(index, false)
                if (info.flags and MediaCodec.BUFFER_FLAG_END_OF_STREAM != 0) done.countDown()
            }

            override fun onOutputFormatChanged(codec: MediaCodec, format: MediaFormat) = sink.addVideoTrack(format)

            override fun onError(codec: MediaCodec, e: MediaCodec.CodecException) {
                Log.e(TAG, "encoder error", e)
                done.countDown()
            }
        }, Handler(thread.looper))
        encoder.configure(format, null, null, MediaCodec.CONFIGURE_FLAG_ENCODE)
        inputSurface = encoder.createInputSurface()
        encoder.start()
    }

    /** Call after the GL pipeline stopped drawing into [inputSurface]. Blocks until drained. */
    fun stop() {
        try {
            encoder.signalEndOfInputStream()
            if (!done.await(3, TimeUnit.SECONDS)) Log.w(TAG, "timed out waiting for end of stream")
            encoder.stop()
        } catch (e: Exception) {
            Log.e(TAG, "stop failed", e)
        } finally {
            encoder.release()
            inputSurface.release()
            thread.quitSafely()
        }
    }

    private companion object {
        const val TAG = "VideoEncoder"
    }
}
