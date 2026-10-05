package com.opentrack.camera.gl

import android.opengl.GLES20
import android.opengl.GLES30
import com.opentrack.tracking.Box
import com.opentrack.tracking.CropWindow
import com.opentrack.tracking.ZoomMap
import com.opentrack.tracking.intersect
import java.nio.FloatBuffer
import kotlin.math.cos
import kotlin.math.sin

/**
 * GPU ring buffer of past frames for look-ahead stabilization. Each slot keeps the frame's
 * capture window (the region around the expected crop) as two planes: full-resolution luma
 * (R8) and half-resolution chroma (RG8), 1.5 bytes per pixel — the same layout the video
 * encoder uses, at a third of the memory of RGBA.
 */
class YuvRing(val slotWidth: Int, val slotHeight: Int, capacity: Int) {
    /**
     * Rolling shutter correction for one frame (see RollingShutter.corrections): rigid
     * (shiftX, shiftY, angle) samples along the readout, direction and extent of the readout,
     * and the source image center, all in source px.
     */
    class ShutterCorrection(val samples: FloatArray, val dirX: Float, val dirY: Float, val extent: Float, val centerX: Float, val centerY: Float) {
        init {
            require(samples.size == SAMPLES * 3) { "need $SAMPLES samples" }
        }

        companion object {
            const val SAMPLES = 9
        }
    }

    class Slot(val yTex: Int, val uvTex: Int, val yFbo: Int, val uvFbo: Int) {
        var timestampNs = Long.MIN_VALUE
        /** The stored region (reference px). */
        var capture: Box? = null
        /** The part of [capture] with camera pixels (the rest is black: outside a telephoto frame). */
        var valid: Box? = null
        /** The frame's camera zoom ratio. */
        var zoom = 1.0
    }

    private val slots: Array<Slot> = Array(capacity) { createSlot() }
    private var next = 0
    private val output = OutputProgram()

    val bytes: Long get() = slots.size.toLong() * slotWidth * slotHeight * 3 / 2

    /**
     * Stores the current camera frame's [capture] region (reference px: the main lens at 1x) in
     * the next slot; [map] places the frame, taken at some camera zoom ratio, in that space.
     */
    fun store(
        renderer: FrameRenderer, texture: Int, texMatrix: FloatArray, sourceW: Int, sourceH: Int, capture: Box, timestampNs: Long,
        map: ZoomMap = ZoomMap.identity(sourceW, sourceH),
    ) {
        val slot = slots[next]
        next = (next + 1) % slots.size
        val s = map.toStream(capture)
        val l = (s.left / sourceW).toFloat()
        val t = (s.top / sourceH).toFloat()
        val w = (s.width / sourceW).toFloat()
        val h = (s.height / sourceH).toFloat()
        GLES20.glBindFramebuffer(GLES20.GL_FRAMEBUFFER, slot.yFbo)
        GLES20.glViewport(0, 0, slotWidth, slotHeight)
        renderer.drawLumaPlane(texture, texMatrix, l, t, w, h, slotWidth, slotHeight, sourceW, sourceH)
        GLES20.glBindFramebuffer(GLES20.GL_FRAMEBUFFER, slot.uvFbo)
        GLES20.glViewport(0, 0, slotWidth / 2, slotHeight / 2)
        renderer.drawChromaPlane(texture, texMatrix, l, t, w, h, w / slotWidth, h / slotHeight)
        GLES20.glBindFramebuffer(GLES20.GL_FRAMEBUFFER, 0)
        slot.timestampNs = timestampNs
        slot.capture = capture
        slot.valid = if (map.zoom > 1.0) capture.intersect(map.visible) else capture
        slot.zoom = map.zoom
    }

    fun find(timestampNs: Long): Slot? = slots.firstOrNull { it.timestampNs == timestampNs }

    /**
     * Draws [crop] (source px, may be rotated) of the stored frame into the current viewport,
     * undoing the rolling shutter distortion with [shutter] when given. [homography] (row-major,
     * output-local source px → source px; see PerspectiveView) renders the crop as a virtual
     * camera view; null draws the flat crop. [sharpen] (see [sharpenFor]) is the amount of
     * luma sharpening.
     */
    fun draw(slot: Slot, crop: CropWindow, shutter: ShutterCorrection? = null, homography: DoubleArray? = null, sharpen: Float = 0f) {
        val cap = slot.capture ?: return
        val h = homography ?: run {
            val c = cos(crop.rotation)
            val s = sin(crop.rotation)
            doubleArrayOf(c, -s, crop.cx, s, c, crop.cy, 0.0, 0.0, 1.0)
        }
        // GLSL matrices are column-major.
        for (r in 0 until 3) for (c in 0 until 3) hColumnMajor[c * 3 + r] = h[r * 3 + c].toFloat()
        output.draw(slot, hColumnMajor, (crop.width / 2).toFloat(), (crop.height / 2).toFloat(), cap, shutter, 1f / slotWidth, 1f / slotHeight, sharpen)
    }

    private val hColumnMajor = FloatArray(9)

    companion object {
        /**
         * Sharpening for an output that enlarges the source by [scale] (output px per source
         * px): none when shrinking, growing with the enlargement (a 3x crop of the sensor is
         * soft however well it is interpolated). Tuned in tools/shader_check/upscale.mjs.
         */
        fun sharpenFor(scale: Double): Float = (SHARPEN_BASE + SHARPEN_PER_SCALE * (scale - 1.0)).coerceIn(0.0, SHARPEN_MAX).toFloat()

        const val SHARPEN_BASE = 0.0
        const val SHARPEN_PER_SCALE = 0.5
        const val SHARPEN_MAX = 1.0
    }

    fun release() {
        for (s in slots) {
            GLES20.glDeleteFramebuffers(2, intArrayOf(s.yFbo, s.uvFbo), 0)
            GLES20.glDeleteTextures(2, intArrayOf(s.yTex, s.uvTex), 0)
        }
        output.release()
    }

    private fun createSlot(): Slot {
        val tex = IntArray(2)
        val fbo = IntArray(2)
        GLES20.glGenTextures(2, tex, 0)
        GLES20.glGenFramebuffers(2, fbo, 0)
        plane(tex[0], fbo[0], slotWidth, slotHeight, GLES30.GL_R8, GLES30.GL_RED)
        plane(tex[1], fbo[1], slotWidth / 2, slotHeight / 2, GLES30.GL_RG8, GLES30.GL_RG)
        return Slot(tex[0], tex[1], fbo[0], fbo[1])
    }

    private fun plane(tex: Int, fbo: Int, w: Int, h: Int, internal: Int, format: Int) {
        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, tex)
        GLES30.glTexImage2D(GLES20.GL_TEXTURE_2D, 0, internal, w, h, 0, format, GLES20.GL_UNSIGNED_BYTE, null)
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_LINEAR)
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR)
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_S, GLES20.GL_CLAMP_TO_EDGE)
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_T, GLES20.GL_CLAMP_TO_EDGE)
        GLES20.glBindFramebuffer(GLES20.GL_FRAMEBUFFER, fbo)
        GLES20.glFramebufferTexture2D(GLES20.GL_FRAMEBUFFER, GLES20.GL_COLOR_ATTACHMENT0, GLES20.GL_TEXTURE_2D, tex, 0)
        check(GLES20.glCheckFramebufferStatus(GLES20.GL_FRAMEBUFFER) == GLES20.GL_FRAMEBUFFER_COMPLETE) { "ring plane framebuffer incomplete" }
        GLES20.glBindFramebuffer(GLES20.GL_FRAMEBUFFER, 0)
    }

    /** YUV planes → RGB through a projective crop mapping (homography) to source px. */
    private class OutputProgram {
        private val quad: FloatBuffer = GlUtil.floatBuffer(-1f, -1f, 1f, -1f, -1f, 1f, 1f, 1f)
        private val id = GlUtil.program(VERTEX, FRAGMENT)
        private val aPos = GLES20.glGetAttribLocation(id, "aPos")
        private val uH = GLES20.glGetUniformLocation(id, "uH")
        private val uHalf = GLES20.glGetUniformLocation(id, "uHalf")
        private val sY = GLES20.glGetUniformLocation(id, "sY")
        private val sUv = GLES20.glGetUniformLocation(id, "sUV")
        private val uRsOn = GLES20.glGetUniformLocation(id, "uRsOn")
        private val uRs = GLES20.glGetUniformLocation(id, "uRs")
        private val uCapOrigin = GLES20.glGetUniformLocation(id, "uCapOrigin")
        private val uCapSize = GLES20.glGetUniformLocation(id, "uCapSize")
        private val uCenter = GLES20.glGetUniformLocation(id, "uCenter")
        private val uReadout = GLES20.glGetUniformLocation(id, "uReadout")
        private val uTexel = GLES20.glGetUniformLocation(id, "uTexel")
        private val uSharp = GLES20.glGetUniformLocation(id, "uSharp")

        fun draw(slot: Slot, h: FloatArray, halfW: Float, halfH: Float, cap: Box, shutter: ShutterCorrection?, texelW: Float, texelH: Float, sharpen: Float) {
            GLES20.glUseProgram(id)
            GLES20.glActiveTexture(GLES20.GL_TEXTURE0)
            GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, slot.yTex)
            GLES20.glUniform1i(sY, 0)
            GLES20.glActiveTexture(GLES20.GL_TEXTURE1)
            GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, slot.uvTex)
            GLES20.glUniform1i(sUv, 1)
            GLES20.glUniformMatrix3fv(uH, 1, false, h, 0)
            GLES20.glUniform2f(uHalf, halfW, halfH)
            GLES20.glUniform2f(uCapOrigin, cap.left.toFloat(), cap.top.toFloat())
            GLES20.glUniform2f(uCapSize, cap.width.toFloat(), cap.height.toFloat())
            GLES20.glUniform1i(uRsOn, if (shutter != null) 1 else 0)
            GLES20.glUniform2f(uTexel, texelW, texelH)
            GLES20.glUniform1f(uSharp, sharpen)
            if (shutter != null) {
                GLES20.glUniform3fv(uRs, ShutterCorrection.SAMPLES, shutter.samples, 0)
                GLES20.glUniform2f(uCenter, shutter.centerX, shutter.centerY)
                GLES20.glUniform3f(uReadout, shutter.dirX, shutter.dirY, shutter.extent)
            }
            GLES20.glEnableVertexAttribArray(aPos)
            GLES20.glVertexAttribPointer(aPos, 2, GLES20.GL_FLOAT, false, 0, quad)
            GLES20.glDrawArrays(GLES20.GL_TRIANGLE_STRIP, 0, 4)
            GLES20.glDisableVertexAttribArray(aPos)
            GLES20.glActiveTexture(GLES20.GL_TEXTURE0)
        }

        fun release() = GLES20.glDeleteProgram(id)

        private companion object {
            // Output pixel u (x right, y down, relative to the output center, in source px)
            // samples the source at H (u, 1): the crop's center, rotation and size, and with a
            // perspective homography the virtual camera view (projective, so divided per pixel).
            const val VERTEX = """
                attribute vec2 aPos;
                uniform mat3 uH;
                uniform vec2 uHalf;
                varying highp vec3 vSrc;
                void main() {
                    gl_Position = vec4(aPos, 0.0, 1.0);
                    vSrc = uH * vec3(aPos.x * uHalf.x, -aPos.y * uHalf.y, 1.0);
                }
            """
            // Rolling shutter: p is where a point sits in the ideal (global shutter) frame; the
            // sensor recorded it shifted and turned by the camera motion at its row's readout
            // time. The rigid correction is interpolated from 9 samples along the readout.
            const val FRAGMENT = """
                precision highp float;
                uniform sampler2D sY;
                uniform sampler2D sUV;
                uniform int uRsOn;
                uniform vec3 uRs[9];
                uniform vec2 uCapOrigin;
                uniform vec2 uCapSize;
                uniform vec2 uCenter;
                uniform vec3 uReadout;
                uniform vec2 uTexel;
                uniform float uSharp;
                varying highp vec3 vSrc;
                // Catmull-Rom bicubic luma from 9 bilinear taps: the slot is drawn about 1:1,
                // where bilinear would soften every frame by its sub-pixel offset.
                float lumaBicubic(vec2 at) {
                    vec2 s = at / uTexel;
                    vec2 t1 = floor(s - 0.5) + 0.5;
                    vec2 f = s - t1;
                    vec2 w0 = f * (-0.5 + f * (1.0 - 0.5 * f));
                    vec2 w1 = 1.0 + f * f * (-2.5 + 1.5 * f);
                    vec2 w2 = f * (0.5 + f * (2.0 - 1.5 * f));
                    vec2 w3 = f * f * (-0.5 + 0.5 * f);
                    vec2 w12 = w1 + w2;
                    vec2 a = (t1 - 1.0) * uTexel;
                    vec2 b = (t1 + w2 / w12) * uTexel;
                    vec2 c = (t1 + 2.0) * uTexel;
                    return (texture2D(sY, vec2(a.x, a.y)).r * w0.x + texture2D(sY, vec2(b.x, a.y)).r * w12.x + texture2D(sY, vec2(c.x, a.y)).r * w3.x) * w0.y
                         + (texture2D(sY, vec2(a.x, b.y)).r * w0.x + texture2D(sY, vec2(b.x, b.y)).r * w12.x + texture2D(sY, vec2(c.x, b.y)).r * w3.x) * w12.y
                         + (texture2D(sY, vec2(a.x, c.y)).r * w0.x + texture2D(sY, vec2(b.x, c.y)).r * w12.x + texture2D(sY, vec2(c.x, c.y)).r * w3.x) * w3.y;
                }
                vec2 unshutter(vec2 p) {
                    float f = clamp(dot(p - uCenter, uReadout.xy) / uReadout.z + 0.5, 0.0, 1.0) * 8.0;
                    vec3 r = uRs[0];
                    for (int i = 0; i < 8; i++) {
                        if (float(i) <= f) r = mix(uRs[i], uRs[i + 1], clamp(f - float(i), 0.0, 1.0));
                    }
                    vec2 d = p - uCenter;
                    float c = cos(r.z);
                    float s = sin(r.z);
                    return uCenter + vec2(c * d.x - s * d.y, s * d.x + c * d.y) + r.xy;
                }
                void main() {
                    vec2 p = vSrc.xy / vSrc.z;
                    if (uRsOn == 1) p = unshutter(p);
                    vec2 at = (p - uCapOrigin) / uCapSize;
                    float y = lumaBicubic(at);
                    if (uSharp > 0.0) {
                        // Unsharp mask against the 4 neighbours, limited to their range (no halos).
                        float n = texture2D(sY, at - vec2(0.0, uTexel.y)).r;
                        float so = texture2D(sY, at + vec2(0.0, uTexel.y)).r;
                        float e = texture2D(sY, at + vec2(uTexel.x, 0.0)).r;
                        float w = texture2D(sY, at - vec2(uTexel.x, 0.0)).r;
                        float lo = min(min(min(n, so), min(e, w)), y);
                        float hi = max(max(max(n, so), max(e, w)), y);
                        y = clamp(y + uSharp * (y - 0.25 * (n + so + e + w)), lo, hi);
                    }
                    y = clamp(y, 0.0, 1.0);
                    vec2 c = texture2D(sUV, at).rg - 0.5;
                    gl_FragColor = vec4(y + 1.402 * c.y, y - 0.344136 * c.x - 0.714136 * c.y, y + 1.772 * c.x, 1.0);
                }
            """
        }
    }
}
