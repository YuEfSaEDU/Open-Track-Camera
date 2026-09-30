package com.opentrack.camera.record

import android.annotation.SuppressLint
import android.media.AudioFormat
import android.media.AudioRecord
import android.media.AudioTimestamp
import android.media.MediaCodec
import android.media.MediaCodecInfo
import android.media.MediaFormat
import android.media.MediaRecorder
import android.util.Log

/** Microphone → AAC encoder running on its own thread, timestamps in CLOCK_MONOTONIC. */
class AudioEncoder(private val sink: MuxerSink) {
    @Volatile
    private var running = false
    private var thread: Thread? = null

    @SuppressLint("MissingPermission")
    fun start(): Boolean {
        val minBuf = AudioRecord.getMinBufferSize(SAMPLE_RATE, AudioFormat.CHANNEL_IN_MONO, AudioFormat.ENCODING_PCM_16BIT)
        if (minBuf <= 0) return false
        val record = try {
            AudioRecord(MediaRecorder.AudioSource.CAMCORDER, SAMPLE_RATE, AudioFormat.CHANNEL_IN_MONO, AudioFormat.ENCODING_PCM_16BIT, minBuf * 4)
        } catch (e: Exception) {
            Log.w(TAG, "AudioRecord unavailable", e)
            return false
        }
        if (record.state != AudioRecord.STATE_INITIALIZED) {
            record.release()
            return false
        }
        val format = MediaFormat.createAudioFormat(MediaFormat.MIMETYPE_AUDIO_AAC, SAMPLE_RATE, 1).apply {
            setInteger(MediaFormat.KEY_AAC_PROFILE, MediaCodecInfo.CodecProfileLevel.AACObjectLC)
            setInteger(MediaFormat.KEY_BIT_RATE, 160_000)
            setInteger(MediaFormat.KEY_MAX_INPUT_SIZE, minBuf * 2)
        }
        val codec = MediaCodec.createEncoderByType(MediaFormat.MIMETYPE_AUDIO_AAC)
        codec.configure(format, null, null, MediaCodec.CONFIGURE_FLAG_ENCODE)
        codec.start()
        record.startRecording()
        running = true
        thread = Thread({ loop(record, codec) }, "audio-encoder").apply { start() }
        return true
    }

    private fun loop(record: AudioRecord, codec: MediaCodec) {
        val info = MediaCodec.BufferInfo()
        val ts = AudioTimestamp()
        var framesRead = 0L
        var anchorNs = -1L
        var anchorFrame = 0L
        var eosQueued = false
        try {
            while (true) {
                if (!eosQueued) {
                    val index = codec.dequeueInputBuffer(10_000)
                    if (index >= 0) {
                        val buf = codec.getInputBuffer(index)!!
                        buf.clear()
                        if (!running) {
                            codec.queueInputBuffer(index, 0, 0, ptsUs(anchorNs, anchorFrame, framesRead), MediaCodec.BUFFER_FLAG_END_OF_STREAM)
                            eosQueued = true
                        } else {
                            val n = record.read(buf, buf.capacity())
                            if (n > 0) {
                                if (record.getTimestamp(ts, AudioTimestamp.TIMEBASE_MONOTONIC) == AudioRecord.SUCCESS) {
                                    anchorNs = ts.nanoTime
                                    anchorFrame = ts.framePosition
                                } else if (anchorNs < 0) {
                                    anchorNs = System.nanoTime()
                                    anchorFrame = framesRead
                                }
                                codec.queueInputBuffer(index, 0, n, ptsUs(anchorNs, anchorFrame, framesRead), 0)
                                framesRead += n / 2
                            } else {
                                codec.queueInputBuffer(index, 0, 0, 0, 0)
                            }
                        }
                    }
                }
                var out = codec.dequeueOutputBuffer(info, 0)
                while (out != MediaCodec.INFO_TRY_AGAIN_LATER) {
                    if (out == MediaCodec.INFO_OUTPUT_FORMAT_CHANGED) {
                        sink.addAudioTrack(codec.outputFormat)
                    } else if (out >= 0) {
                        val buf = codec.getOutputBuffer(out)
                        if (buf != null && info.flags and MediaCodec.BUFFER_FLAG_CODEC_CONFIG == 0) sink.write(false, buf, info)
                        codec.releaseOutputBuffer(out, false)
                        if (info.flags and MediaCodec.BUFFER_FLAG_END_OF_STREAM != 0) return
                    }
                    out = codec.dequeueOutputBuffer(info, 0)
                }
            }
        } catch (e: Exception) {
            Log.e(TAG, "audio loop failed", e)
            sink.giveUpAudio()
        } finally {
            record.stop()
            record.release()
            codec.stop()
            codec.release()
        }
    }

    private fun ptsUs(anchorNs: Long, anchorFrame: Long, frame: Long): Long =
        (anchorNs + (frame - anchorFrame) * 1_000_000_000L / SAMPLE_RATE) / 1000

    fun stop() {
        running = false
        thread?.join(3000)
        thread = null
    }

    private companion object {
        const val TAG = "AudioEncoder"
        const val SAMPLE_RATE = 48_000
    }
}
