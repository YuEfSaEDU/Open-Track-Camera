package com.opentrack.camera.record

import android.media.MediaCodec
import android.media.MediaFormat
import android.media.MediaMuxer
import java.io.FileDescriptor
import java.nio.ByteBuffer

/**
 * Thread-safe MP4 muxer shared by the video and audio encoders. Samples that arrive before
 * every expected track has reported its format are buffered, so the first video keyframe is
 * never lost. All timestamps are expected in the CLOCK_MONOTONIC domain (µs) and are
 * rebased to the first video frame.
 */
class MuxerSink(fd: FileDescriptor, expectAudio: Boolean) {
    private val muxer = MediaMuxer(fd, MediaMuxer.OutputFormat.MUXER_OUTPUT_MPEG_4)
    private var videoTrack = -1
    private var audioTrack = -1
    private var audioExpected = expectAudio
    private var started = false
    private var stopped = false
    private var baseUs = Long.MIN_VALUE
    private val pending = ArrayList<Sample>()
    private var pendingBytes = 0

    private class Sample(val video: Boolean, val data: ByteBuffer, val info: MediaCodec.BufferInfo)

    @Synchronized
    fun addVideoTrack(format: MediaFormat) {
        if (videoTrack < 0) videoTrack = muxer.addTrack(format)
        maybeStart()
    }

    @Synchronized
    fun addAudioTrack(format: MediaFormat) {
        if (audioTrack < 0 && !started) audioTrack = muxer.addTrack(format)
        maybeStart()
    }

    /** Called when the audio pipeline failed to start: record video only. */
    @Synchronized
    fun giveUpAudio() {
        audioExpected = false
        maybeStart()
    }

    @Synchronized
    fun write(video: Boolean, data: ByteBuffer, info: MediaCodec.BufferInfo) {
        if (stopped || info.size <= 0) return
        if (!started) {
            if (pendingBytes > MAX_PENDING_BYTES) return
            val copy = ByteBuffer.allocateDirect(info.size)
            data.position(info.offset)
            data.limit(info.offset + info.size)
            copy.put(data)
            copy.flip()
            val i = MediaCodec.BufferInfo().apply { set(0, info.size, info.presentationTimeUs, info.flags) }
            pending += Sample(video, copy, i)
            pendingBytes += info.size
            return
        }
        writeNow(video, data, info)
    }

    private fun writeNow(video: Boolean, data: ByteBuffer, info: MediaCodec.BufferInfo) {
        if (baseUs == Long.MIN_VALUE) {
            if (!video) return // audio before the first video frame is dropped
            baseUs = info.presentationTimeUs
        }
        val pts = info.presentationTimeUs - baseUs
        if (pts < 0) return
        val track = if (video) videoTrack else audioTrack
        if (track < 0) return
        val out = MediaCodec.BufferInfo().apply { set(info.offset, info.size, pts, info.flags) }
        muxer.writeSampleData(track, data, out)
    }

    private fun maybeStart() {
        if (started || videoTrack < 0 || (audioExpected && audioTrack < 0)) return
        muxer.start()
        started = true
        // The base timestamp comes from the first video frame; earlier audio is dropped.
        pending.filter { it.video }.minOfOrNull { it.info.presentationTimeUs }?.let { baseUs = it }
        for (s in pending.sortedBy { it.info.presentationTimeUs }) writeNow(s.video, s.data, s.info)
        pending.clear()
        pendingBytes = 0
    }

    /** Returns true when a playable file was written. */
    @Synchronized
    fun stop(): Boolean {
        if (stopped) return false
        stopped = true
        val ok = started && baseUs != Long.MIN_VALUE
        try {
            if (started) muxer.stop()
        } finally {
            muxer.release()
        }
        return ok
    }

    private companion object {
        const val MAX_PENDING_BYTES = 32 * 1024 * 1024
    }
}
