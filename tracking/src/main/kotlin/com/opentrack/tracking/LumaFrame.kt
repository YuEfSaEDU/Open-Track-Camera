package com.opentrack.tracking

/**
 * A downscaled 8-bit luminance image used for analysis.
 *
 * @param scale number of source (full resolution sensor stream) pixels per analysis pixel.
 *   Everything the tracker publishes is expressed in source pixels.
 */
class LumaFrame(
    val width: Int,
    val height: Int,
    val data: ByteArray,
    val timestampNs: Long,
    val scale: Double,
    /**
     * The part of the image the camera actually saw (source px), when less than the whole
     * frame: a zoomed (telephoto) frame drawn in the main lens's reference space is black
     * around it. Null = everything.
     */
    val visible: Box? = null,
) {
    init {
        require(data.size >= width * height) { "buffer too small: ${data.size} < ${width * height}" }
    }

    fun at(x: Int, y: Int): Int = data[y * width + x].toInt() and 0xFF
}
