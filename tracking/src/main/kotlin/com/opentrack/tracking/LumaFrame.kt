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
    /**
     * Where analysis px (0, 0) is in the source (reference) frame: nonzero when the image is a
     * window of it (a telephoto frame, analysed at its own resolution, see [windowed]).
     */
    val originX: Double = 0.0,
    val originY: Double = 0.0,
    /** Size of the whole source (reference) frame, px; the image may cover only part of it. */
    val sourceWidth: Double = width * scale,
    val sourceHeight: Double = height * scale,
) {
    init {
        require(data.size >= width * height) { "buffer too small: ${data.size} < ${width * height}" }
    }

    fun at(x: Int, y: Int): Int = data[y * width + x].toInt() and 0xFF

    /** Analysis px → source (reference) px. */
    fun toSource(p: Vec2): Vec2 = Vec2(originX + p.x * scale, originY + p.y * scale)

    fun toSource(b: Box): Box = toSource(b.center).let { Box(it.x, it.y, b.width * scale, b.height * scale) }

    /** Source (reference) px → analysis px. */
    fun toAnalysis(p: Vec2): Vec2 = Vec2((p.x - originX) / scale, (p.y - originY) / scale)

    fun toAnalysis(b: Box): Box = toAnalysis(b.center).let { Box(it.x, it.y, b.width / scale, b.height / scale) }

    /** The part of the source frame the image covers. */
    val window: Box get() = Box(originX + width * scale / 2, originY + height * scale / 2, width * scale, height * scale)

    /** Whether the image is only a window of the source frame. */
    val windowed: Boolean get() = originX != 0.0 || originY != 0.0 || width * scale < sourceWidth - 1e-6
}
