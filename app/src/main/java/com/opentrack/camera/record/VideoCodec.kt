package com.opentrack.camera.record

import android.media.MediaCodecInfo
import android.media.MediaCodecList
import android.media.MediaFormat

/**
 * Recording codec. HEVC gives better quality at the same bit rate. 10-bit HEVC (Main10) keeps
 * smooth gradients (sky): the encoder's own quantization is what bands a plain sky at 8 bits,
 * and the GPU's interpolation and sharpening produce in-between values that 8 bits round away.
 * The camera stream itself stays 8-bit SDR (10-bit capture on Android means HDR / HLG).
 */
enum class VideoCodec(val mime: String, val tenBit: Boolean) {
    AVC(MediaFormat.MIMETYPE_VIDEO_AVC, false),
    HEVC(MediaFormat.MIMETYPE_VIDEO_HEVC, false),
    HEVC_10(MediaFormat.MIMETYPE_VIDEO_HEVC, true);

    /** A hardware-capable encoder for this codec at [width] x [height] @ [fps] exists. */
    fun supported(width: Int, height: Int, fps: Int): Boolean =
        MediaCodecList(MediaCodecList.REGULAR_CODECS).codecInfos.any { info ->
            info.isEncoder && info.supportedTypes.any { it.equals(mime, ignoreCase = true) } &&
                runCatching {
                    val caps = info.getCapabilitiesForType(mime)
                    caps.videoCapabilities.areSizeAndRateSupported(width, height, fps.toDouble()) &&
                        (!tenBit || caps.profileLevels.any { it.profile == MediaCodecInfo.CodecProfileLevel.HEVCProfileMain10 })
                }.getOrDefault(false)
        }

    /** Sets the codec's profile and the (BT.709 SDR) colour description on [format]. */
    fun configure(format: MediaFormat) {
        if (tenBit) format.setInteger(MediaFormat.KEY_PROFILE, MediaCodecInfo.CodecProfileLevel.HEVCProfileMain10)
        format.setInteger(MediaFormat.KEY_COLOR_STANDARD, MediaFormat.COLOR_STANDARD_BT709)
        format.setInteger(MediaFormat.KEY_COLOR_TRANSFER, MediaFormat.COLOR_TRANSFER_SDR_VIDEO)
        format.setInteger(MediaFormat.KEY_COLOR_RANGE, MediaFormat.COLOR_RANGE_LIMITED)
    }

    companion object {
        /** [preferred] if the phone can encode it at that size and rate, else the best that can (H.264 last). */
        fun pick(preferred: VideoCodec, width: Int, height: Int, fps: Int): VideoCodec =
            (listOf(preferred) + listOf(HEVC, AVC).filter { it != preferred }).firstOrNull { it.supported(width, height, fps) } ?: AVC
    }
}
