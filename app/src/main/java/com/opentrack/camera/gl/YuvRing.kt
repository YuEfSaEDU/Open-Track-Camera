package com.opentrack.camera.gl

import android.opengl.GLES20
import android.opengl.GLES30
import com.opentrack.tracking.Box
import com.opentrack.tracking.CropWindow
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
    class Slot(val yTex: Int, val uvTex: Int, val yFbo: Int, val uvFbo: Int) {
        var timestampNs = Long.MIN_VALUE
        var capture: Box? = null
    }

    private val slots: Array<Slot> = Array(capacity) { createSlot() }
    private var next = 0
    private val output = OutputProgram()

    val bytes: Long get() = slots.size.toLong() * slotWidth * slotHeight * 3 / 2

    /** Stores the current camera frame's [capture] region (source px) in the next slot. */
    fun store(renderer: FrameRenderer, texture: Int, texMatrix: FloatArray, sourceW: Int, sourceH: Int, capture: Box, timestampNs: Long) {
        val slot = slots[next]
        next = (next + 1) % slots.size
        val l = (capture.left / sourceW).toFloat()
        val t = (capture.top / sourceH).toFloat()
        val w = (capture.width / sourceW).toFloat()
        val h = (capture.height / sourceH).toFloat()
        GLES20.glBindFramebuffer(GLES20.GL_FRAMEBUFFER, slot.yFbo)
        GLES20.glViewport(0, 0, slotWidth, slotHeight)
        renderer.drawLumaPlane(texture, texMatrix, l, t, w, h)
        GLES20.glBindFramebuffer(GLES20.GL_FRAMEBUFFER, slot.uvFbo)
        GLES20.glViewport(0, 0, slotWidth / 2, slotHeight / 2)
        renderer.drawChromaPlane(texture, texMatrix, l, t, w, h, w / slotWidth, h / slotHeight)
        GLES20.glBindFramebuffer(GLES20.GL_FRAMEBUFFER, 0)
        slot.timestampNs = timestampNs
        slot.capture = capture
    }

    fun find(timestampNs: Long): Slot? = slots.firstOrNull { it.timestampNs == timestampNs }

    /** Draws [crop] (source px, may be rotated) of the stored frame into the current viewport. */
    fun draw(slot: Slot, crop: CropWindow) {
        val cap = slot.capture ?: return
        val c = cos(crop.rotation)
        val s = sin(crop.rotation)
        output.draw(
            slot,
            ((crop.cx - cap.left) / cap.width).toFloat(), ((crop.cy - cap.top) / cap.height).toFloat(),
            (c * crop.width / cap.width).toFloat(), (s * crop.width / cap.height).toFloat(),
            (-s * crop.height / cap.width).toFloat(), (c * crop.height / cap.height).toFloat(),
        )
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

    /** YUV planes → RGB with an affine crop mapping (position, rotation, scale) in the vertex shader. */
    private class OutputProgram {
        private val quad: FloatBuffer = GlUtil.floatBuffer(-1f, -1f, 1f, -1f, -1f, 1f, 1f, 1f)
        private val id = GlUtil.program(VERTEX, FRAGMENT)
        private val aPos = GLES20.glGetAttribLocation(id, "aPos")
        private val uOrigin = GLES20.glGetUniformLocation(id, "uOrigin")
        private val uAxisX = GLES20.glGetUniformLocation(id, "uAxisX")
        private val uAxisY = GLES20.glGetUniformLocation(id, "uAxisY")
        private val sY = GLES20.glGetUniformLocation(id, "sY")
        private val sUv = GLES20.glGetUniformLocation(id, "sUV")

        fun draw(slot: Slot, ox: Float, oy: Float, axx: Float, axy: Float, ayx: Float, ayy: Float) {
            GLES20.glUseProgram(id)
            GLES20.glActiveTexture(GLES20.GL_TEXTURE0)
            GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, slot.yTex)
            GLES20.glUniform1i(sY, 0)
            GLES20.glActiveTexture(GLES20.GL_TEXTURE1)
            GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, slot.uvTex)
            GLES20.glUniform1i(sUv, 1)
            GLES20.glUniform2f(uOrigin, ox, oy)
            GLES20.glUniform2f(uAxisX, axx, axy)
            GLES20.glUniform2f(uAxisY, ayx, ayy)
            GLES20.glEnableVertexAttribArray(aPos)
            GLES20.glVertexAttribPointer(aPos, 2, GLES20.GL_FLOAT, false, 0, quad)
            GLES20.glDrawArrays(GLES20.GL_TRIANGLE_STRIP, 0, 4)
            GLES20.glDisableVertexAttribArray(aPos)
            GLES20.glActiveTexture(GLES20.GL_TEXTURE0)
        }

        fun release() = GLES20.glDeleteProgram(id)

        private companion object {
            // Output pixel u (x right, y down, relative to the output center) samples the slot at
            // origin + u.x * axisX + u.y * axisY: the crop's center, rotation and size in slot units.
            const val VERTEX = """
                attribute vec2 aPos;
                uniform vec2 uOrigin;
                uniform vec2 uAxisX;
                uniform vec2 uAxisY;
                varying highp vec2 vSlot;
                void main() {
                    gl_Position = vec4(aPos, 0.0, 1.0);
                    vec2 u = vec2(aPos.x * 0.5, -aPos.y * 0.5);
                    vSlot = uOrigin + uAxisX * u.x + uAxisY * u.y;
                }
            """
            const val FRAGMENT = """
                precision mediump float;
                uniform sampler2D sY;
                uniform sampler2D sUV;
                varying highp vec2 vSlot;
                void main() {
                    float y = texture2D(sY, vSlot).r;
                    vec2 c = texture2D(sUV, vSlot).rg - 0.5;
                    gl_FragColor = vec4(y + 1.402 * c.y, y - 0.344136 * c.x - 0.714136 * c.y, y + 1.772 * c.x, 1.0);
                }
            """
        }
    }
}
