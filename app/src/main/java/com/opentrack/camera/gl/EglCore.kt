package com.opentrack.camera.gl

import android.opengl.EGL14
import android.opengl.EGLConfig
import android.opengl.EGLContext
import android.opengl.EGLDisplay
import android.opengl.EGLExt
import android.opengl.EGLSurface
import android.view.Surface

/**
 * Minimal EGL wrapper: one GLES 3 context whose config is recordable, so the same context
 * can render into the preview SurfaceView and into the MediaCodec input surface.
 */
class EglCore {
    val display: EGLDisplay = EGL14.eglGetDisplay(EGL14.EGL_DEFAULT_DISPLAY)
    private val config: EGLConfig
    private val supportsPbuffer: Boolean
    val context: EGLContext

    init {
        check(display != EGL14.EGL_NO_DISPLAY) { "no EGL display" }
        val version = IntArray(2)
        check(EGL14.eglInitialize(display, version, 0, version, 1)) { "eglInitialize failed" }
        // Prefer a config that also supports pbuffers; otherwise run surfaceless when no window is bound.
        val withPbuffer = chooseConfig(EGL14.EGL_WINDOW_BIT or EGL14.EGL_PBUFFER_BIT)
        supportsPbuffer = withPbuffer != null
        config = withPbuffer ?: chooseConfig(EGL14.EGL_WINDOW_BIT) ?: error("no recordable GLES3 EGL config")
        context = EGL14.eglCreateContext(
            display, config, EGL14.EGL_NO_CONTEXT,
            intArrayOf(EGL14.EGL_CONTEXT_CLIENT_VERSION, 3, EGL14.EGL_NONE), 0,
        )
        check(context != EGL14.EGL_NO_CONTEXT) { "eglCreateContext failed: 0x${Integer.toHexString(EGL14.eglGetError())}" }
    }

    fun createWindowSurface(surface: Surface): EGLSurface {
        val s = EGL14.eglCreateWindowSurface(display, config, surface, intArrayOf(EGL14.EGL_NONE), 0)
        check(s != EGL14.EGL_NO_SURFACE) { "eglCreateWindowSurface failed: 0x${Integer.toHexString(EGL14.eglGetError())}" }
        return s
    }

    /**
     * 1x1 pbuffer so the context can be current before any window exists. Returns
     * EGL_NO_SURFACE when pbuffers are unsupported: making that current relies on
     * EGL_KHR_surfaceless_context, which all GLES3 Android devices provide.
     */
    fun createPbuffer(): EGLSurface =
        if (supportsPbuffer) {
            EGL14.eglCreatePbufferSurface(display, config, intArrayOf(EGL14.EGL_WIDTH, 1, EGL14.EGL_HEIGHT, 1, EGL14.EGL_NONE), 0)
        } else {
            EGL14.EGL_NO_SURFACE
        }

    private fun chooseConfig(surfaceType: Int): EGLConfig? {
        val attribs = intArrayOf(
            EGL14.EGL_RED_SIZE, 8,
            EGL14.EGL_GREEN_SIZE, 8,
            EGL14.EGL_BLUE_SIZE, 8,
            EGL14.EGL_ALPHA_SIZE, 8,
            EGL14.EGL_RENDERABLE_TYPE, EGLExt.EGL_OPENGL_ES3_BIT_KHR,
            EGL14.EGL_SURFACE_TYPE, surfaceType,
            EGL_RECORDABLE_ANDROID, 1,
            EGL14.EGL_NONE,
        )
        val configs = arrayOfNulls<EGLConfig>(1)
        val num = IntArray(1)
        return if (EGL14.eglChooseConfig(display, attribs, 0, configs, 0, 1, num, 0) && num[0] > 0) configs[0] else null
    }

    fun makeCurrent(surface: EGLSurface) {
        check(EGL14.eglMakeCurrent(display, surface, surface, context)) { "eglMakeCurrent failed" }
    }

    fun swap(surface: EGLSurface): Boolean = EGL14.eglSwapBuffers(display, surface)

    fun setPresentationTime(surface: EGLSurface, ns: Long) {
        EGLExt.eglPresentationTimeANDROID(display, surface, ns)
    }

    fun surfaceSize(surface: EGLSurface): Pair<Int, Int> {
        val v = IntArray(1)
        EGL14.eglQuerySurface(display, surface, EGL14.EGL_WIDTH, v, 0)
        val w = v[0]
        EGL14.eglQuerySurface(display, surface, EGL14.EGL_HEIGHT, v, 0)
        return w to v[0]
    }

    fun releaseSurface(surface: EGLSurface) {
        if (surface != EGL14.EGL_NO_SURFACE) EGL14.eglDestroySurface(display, surface)
    }

    fun release() {
        EGL14.eglMakeCurrent(display, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_CONTEXT)
        EGL14.eglDestroyContext(display, context)
        EGL14.eglReleaseThread()
        EGL14.eglTerminate(display)
    }

    private companion object {
        const val EGL_RECORDABLE_ANDROID = 0x3142
    }
}
