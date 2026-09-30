package com.opentrack.camera.gl

import android.opengl.GLES11Ext
import android.opengl.GLES20
import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * Draws the camera's external OES texture. All regions are given in "image" coordinates:
 * the frame as it should appear upright on the (landscape) display, x right / y down,
 * normalized to [0, 1].
 *
 * The SurfaceTexture transform matrix already rotates the buffer upright for the device's
 * natural orientation, so the only extra step is the display rotation.
 */
class FrameRenderer {
    private val quad = GlUtil.floatBuffer(-1f, -1f, 1f, -1f, -1f, 1f, 1f, 1f)
    private val display = Program(DISPLAY_FRAGMENT)
    private val luma = Program(LUMA_FRAGMENT)
    private val planeY = Program(PLANE_Y_FRAGMENT)
    private val planeUv = Program(PLANE_UV_FRAGMENT)

    var displayRotationQuarterTurns: Int = 1

    fun createOesTexture(): Int {
        val t = IntArray(1)
        GLES20.glGenTextures(1, t, 0)
        GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, t[0])
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_LINEAR)
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR)
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_WRAP_S, GLES20.GL_CLAMP_TO_EDGE)
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_WRAP_T, GLES20.GL_CLAMP_TO_EDGE)
        return t[0]
    }

    /** Draws the [left, top, width, height] image region to the whole current viewport. */
    fun drawRegion(texture: Int, texMatrix: FloatArray, left: Float, top: Float, width: Float, height: Float, forReadback: Boolean = false) {
        display.draw(texture, texMatrix, left, top, width, height, forReadback, 0f, 0f)
    }

    /**
     * Renders the whole frame as box-filtered luminance, four analysis pixels packed per
     * RGBA texel. The target framebuffer must be (analysisWidth / 4) x analysisHeight.
     */
    fun drawPackedLuma(texture: Int, texMatrix: FloatArray, analysisWidth: Int, analysisHeight: Int) {
        luma.draw(texture, texMatrix, 0f, 0f, 1f, 1f, true, 1f / analysisWidth, 1f / analysisHeight)
    }

    /**
     * Stores the [left, top, width, height] image region as BT.601 full-range luma into the
     * bound R8 framebuffer (image top in texture row 0).
     */
    fun drawLumaPlane(texture: Int, texMatrix: FloatArray, left: Float, top: Float, width: Float, height: Float) {
        planeY.draw(texture, texMatrix, left, top, width, height, true, 0f, 0f)
    }

    /**
     * Stores the region's chroma (Cb, Cr + 0.5) into the bound half-resolution RG8
     * framebuffer, averaging 2x2 full-resolution samples. [pxW]/[pxH]: size of one
     * full-resolution plane pixel in normalized image units.
     */
    fun drawChromaPlane(texture: Int, texMatrix: FloatArray, left: Float, top: Float, width: Float, height: Float, pxW: Float, pxH: Float) {
        planeUv.draw(texture, texMatrix, left, top, width, height, true, pxW, pxH)
    }

    fun release() {
        display.release()
        luma.release()
        planeY.release()
        planeUv.release()
    }

    private inner class Program(fragment: String) {
        val id = GlUtil.program(VERTEX, fragment)
        val aPos = GLES20.glGetAttribLocation(id, "aPos")
        val uCrop = GLES20.glGetUniformLocation(id, "uCrop")
        val uFlipY = GLES20.glGetUniformLocation(id, "uFlipY")
        val uTexMatrix = GLES20.glGetUniformLocation(id, "uTexMatrix")
        val uRotation = GLES20.glGetUniformLocation(id, "uRotation")
        val uPx = GLES20.glGetUniformLocation(id, "uPx")
        val uTex = GLES20.glGetUniformLocation(id, "sTex")

        fun draw(texture: Int, m: FloatArray, l: Float, t: Float, w: Float, h: Float, flip: Boolean, px: Float, py: Float) {
            GLES20.glUseProgram(id)
            GLES20.glActiveTexture(GLES20.GL_TEXTURE0)
            GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, texture)
            GLES20.glUniform1i(uTex, 0)
            GLES20.glUniform4f(uCrop, l, t, w, h)
            // Readback rows start at the bottom of the framebuffer; flip so memory row 0 is the image top.
            GLES20.glUniform1f(uFlipY, if (flip) -1f else 1f)
            GLES20.glUniformMatrix4fv(uTexMatrix, 1, false, m, 0)
            GLES20.glUniform1i(uRotation, displayRotationQuarterTurns and 3)
            if (uPx >= 0) GLES20.glUniform2f(uPx, px, py)
            GLES20.glEnableVertexAttribArray(aPos)
            GLES20.glVertexAttribPointer(aPos, 2, GLES20.GL_FLOAT, false, 0, quad)
            GLES20.glDrawArrays(GLES20.GL_TRIANGLE_STRIP, 0, 4)
            GLES20.glDisableVertexAttribArray(aPos)
        }

        fun release() = GLES20.glDeleteProgram(id)
    }

    private companion object {
        const val VERTEX = """
            attribute vec2 aPos;
            uniform vec4 uCrop;
            uniform float uFlipY;
            varying highp vec2 vImg;
            void main() {
                gl_Position = vec4(aPos.x, aPos.y * uFlipY, 0.0, 1.0);
                vImg = vec2(uCrop.x + (aPos.x * 0.5 + 0.5) * uCrop.z, uCrop.y + (0.5 - aPos.y * 0.5) * uCrop.w);
            }
        """

        const val COMMON = """
            #extension GL_OES_EGL_image_external : require
            precision highp float;
            uniform samplerExternalOES sTex;
            uniform mat4 uTexMatrix;
            uniform int uRotation;
            varying highp vec2 vImg;
            vec2 imgToTex(vec2 img) {
                vec2 n;
                if (uRotation == 1) n = vec2(1.0 - img.y, img.x);
                else if (uRotation == 3) n = vec2(img.y, 1.0 - img.x);
                else if (uRotation == 2) n = vec2(1.0 - img.x, 1.0 - img.y);
                else n = img;
                return (uTexMatrix * vec4(n.x, 1.0 - n.y, 0.0, 1.0)).xy;
            }
        """

        const val DISPLAY_FRAGMENT = COMMON + """
            void main() {
                gl_FragColor = texture2D(sTex, imgToTex(vImg));
            }
        """

        const val PLANE_Y_FRAGMENT = COMMON + """
            void main() {
                float y = dot(texture2D(sTex, imgToTex(vImg)).rgb, vec3(0.299, 0.587, 0.114));
                gl_FragColor = vec4(y, 0.0, 0.0, 1.0);
            }
        """

        const val PLANE_UV_FRAGMENT = COMMON + """
            uniform vec2 uPx;
            void main() {
                vec3 c = texture2D(sTex, imgToTex(vImg + vec2(-0.5, -0.5) * uPx)).rgb
                       + texture2D(sTex, imgToTex(vImg + vec2(0.5, -0.5) * uPx)).rgb
                       + texture2D(sTex, imgToTex(vImg + vec2(-0.5, 0.5) * uPx)).rgb
                       + texture2D(sTex, imgToTex(vImg + vec2(0.5, 0.5) * uPx)).rgb;
                c *= 0.25;
                float cb = -0.168736 * c.r - 0.331264 * c.g + 0.5 * c.b + 0.5;
                float cr = 0.5 * c.r - 0.418688 * c.g - 0.081312 * c.b + 0.5;
                gl_FragColor = vec4(cb, cr, 0.0, 1.0);
            }
        """

        const val LUMA_FRAGMENT = COMMON + """
            uniform vec2 uPx;
            float lumaAt(vec2 c) {
                vec3 acc = vec3(0.0);
                for (int j = -1; j <= 1; j++) {
                    for (int i = -1; i <= 1; i++) {
                        acc += texture2D(sTex, imgToTex(c + vec2(float(i), float(j)) * uPx / 3.0)).rgb;
                    }
                }
                return dot(acc / 9.0, vec3(0.299, 0.587, 0.114));
            }
            void main() {
                gl_FragColor = vec4(
                    lumaAt(vImg + vec2(-1.5 * uPx.x, 0.0)),
                    lumaAt(vImg + vec2(-0.5 * uPx.x, 0.0)),
                    lumaAt(vImg + vec2(0.5 * uPx.x, 0.0)),
                    lumaAt(vImg + vec2(1.5 * uPx.x, 0.0)));
            }
        """
    }
}

/** Offscreen RGBA8 render target with synchronous readback. */
class ReadbackTarget(val width: Int, val height: Int) {
    private val fbo: Int
    private val tex: Int
    val buffer: ByteBuffer = ByteBuffer.allocateDirect(width * height * 4).order(ByteOrder.nativeOrder())

    init {
        val ids = IntArray(1)
        GLES20.glGenTextures(1, ids, 0)
        tex = ids[0]
        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, tex)
        GLES20.glTexImage2D(GLES20.GL_TEXTURE_2D, 0, GLES20.GL_RGBA, width, height, 0, GLES20.GL_RGBA, GLES20.GL_UNSIGNED_BYTE, null)
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_NEAREST)
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_NEAREST)
        GLES20.glGenFramebuffers(1, ids, 0)
        fbo = ids[0]
        GLES20.glBindFramebuffer(GLES20.GL_FRAMEBUFFER, fbo)
        GLES20.glFramebufferTexture2D(GLES20.GL_FRAMEBUFFER, GLES20.GL_COLOR_ATTACHMENT0, GLES20.GL_TEXTURE_2D, tex, 0)
        check(GLES20.glCheckFramebufferStatus(GLES20.GL_FRAMEBUFFER) == GLES20.GL_FRAMEBUFFER_COMPLETE) { "incomplete framebuffer" }
        GLES20.glBindFramebuffer(GLES20.GL_FRAMEBUFFER, 0)
    }

    fun bind() {
        GLES20.glBindFramebuffer(GLES20.GL_FRAMEBUFFER, fbo)
        GLES20.glViewport(0, 0, width, height)
    }

    /** Reads the framebuffer into [buffer] (rows top to bottom, see FrameRenderer flip) and unbinds. */
    fun read(): ByteBuffer {
        buffer.position(0)
        GLES20.glReadPixels(0, 0, width, height, GLES20.GL_RGBA, GLES20.GL_UNSIGNED_BYTE, buffer)
        GLES20.glBindFramebuffer(GLES20.GL_FRAMEBUFFER, 0)
        buffer.position(0)
        return buffer
    }

    fun release() {
        GLES20.glDeleteFramebuffers(1, intArrayOf(fbo), 0)
        GLES20.glDeleteTextures(1, intArrayOf(tex), 0)
    }
}
