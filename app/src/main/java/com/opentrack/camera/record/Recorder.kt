package com.opentrack.camera.record

import android.content.ContentResolver
import android.content.ContentValues
import android.net.Uri
import android.os.ParcelFileDescriptor
import android.os.SystemClock
import android.provider.MediaStore
import android.util.Log
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/**
 * Video bit rate for an output whose long side is [longSide] px at up to 60 fps: 1440p gets
 * more (sky gradients and fine detail at 1.8x the pixels of 1080p).
 */
fun bitrateFor(longSide: Int): Int = when {
    longSide >= 2560 -> 60_000_000
    longSide >= 1920 -> 40_000_000
    else -> 16_000_000
}

data class RecordingConfig(
    val width: Int = 1920,
    val height: Int = 1080,
    val fps: Int = 60,
    val bitrate: Int = 40_000_000,
    val codec: VideoCodec = VideoCodec.AVC,
    val audio: Boolean = true,
)

/**
 * Owns one recording: MediaStore entry, muxer, video + audio encoders. The caller hands
 * [VideoEncoder.inputSurface] to the render pipeline between [start] and [stop].
 */
class Recorder(private val resolver: ContentResolver) {
    private var uri: Uri? = null
    private var pfd: ParcelFileDescriptor? = null
    private var sink: MuxerSink? = null
    private var audio: AudioEncoder? = null
    var video: VideoEncoder? = null
        private set

    val isRecording: Boolean get() = video != null

    /** File name of the recording without extension (e.g. OTC_20261001_110059), once started. */
    var baseName: String? = null
        private set

    /**
     * @param cameraTimestampRealtime whether camera timestamps are CLOCK_BOOTTIME based
     *   (SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME); they are converted to CLOCK_MONOTONIC.
     */
    fun start(config: RecordingConfig, cameraTimestampRealtime: Boolean, audioAllowed: Boolean): VideoEncoder {
        val base = "OTC_" + SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
        baseName = base
        val name = "$base.mp4"
        val values = ContentValues().apply {
            put(MediaStore.Video.Media.DISPLAY_NAME, name)
            put(MediaStore.Video.Media.MIME_TYPE, "video/mp4")
            put(MediaStore.Video.Media.RELATIVE_PATH, "Movies/OpenTrack")
            put(MediaStore.Video.Media.IS_PENDING, 1)
        }
        val u = resolver.insert(MediaStore.Video.Media.getContentUri(MediaStore.VOLUME_EXTERNAL_PRIMARY), values)
            ?: error("MediaStore insert failed")
        uri = u
        val fd = resolver.openFileDescriptor(u, "rw") ?: error("cannot open $u")
        pfd = fd
        val wantAudio = config.audio && audioAllowed
        val s = MuxerSink(fd.fileDescriptor, wantAudio)
        sink = s
        val offsetNs = if (cameraTimestampRealtime) System.nanoTime() - SystemClock.elapsedRealtimeNanos() else 0L
        val codec = VideoCodec.pick(config.codec, config.width, config.height, config.fps)
        val v = VideoEncoder(config.width, config.height, config.fps, config.bitrate, codec, s, offsetNs)
        video = v
        if (wantAudio) {
            val a = AudioEncoder(s)
            if (a.start()) audio = a else s.giveUpAudio()
        }
        return v
    }

    /** Call after the render pipeline detached the encoder surface. Returns the saved file. */
    fun stop(): Uri? {
        val v = video ?: return null
        video = null
        v.stop()
        audio?.stop()
        audio = null
        val ok = try {
            sink?.stop() ?: false
        } catch (e: Exception) {
            Log.e("Recorder", "muxer stop failed", e)
            false
        }
        sink = null
        pfd?.close()
        pfd = null
        val u = uri ?: return null
        uri = null
        if (!ok) {
            resolver.delete(u, null, null)
            return null
        }
        resolver.update(u, ContentValues().apply { put(MediaStore.Video.Media.IS_PENDING, 0) }, null, null)
        return u
    }
}
