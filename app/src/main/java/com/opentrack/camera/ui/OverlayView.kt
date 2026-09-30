package com.opentrack.camera.ui

import android.annotation.SuppressLint
import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.RectF
import android.util.AttributeSet
import android.view.MotionEvent
import android.view.View
import com.opentrack.camera.pipeline.OverlayState
import com.opentrack.camera.pipeline.PreviewMode
import com.opentrack.tracking.Box
import com.opentrack.tracking.DetectionSource
import com.opentrack.tracking.TrackState
import com.opentrack.tracking.Vec2

/**
 * Draws the crop window, the tracked target and raw detections over the preview, and turns
 * taps into target selections (in source pixels).
 */
class OverlayView @JvmOverloads constructor(context: Context, attrs: AttributeSet? = null) : View(context, attrs) {

    @Volatile
    var state: OverlayState? = null
        set(value) {
            field = value
            postInvalidateOnAnimation()
        }

    /** Receives taps converted to source pixel coordinates. */
    var onTap: ((Vec2) -> Unit)? = null

    private val density = resources.displayMetrics.density
    private val cropPaint = stroke(Color.WHITE, 1.5f)
    private val trackPaint = stroke(Color.rgb(60, 230, 90), 2.5f)
    private val coastPaint = stroke(Color.rgb(255, 200, 40), 2.5f)
    private val neuralPaint = stroke(Color.rgb(80, 200, 255), 1.2f)
    private val blobPaint = stroke(Color.rgb(255, 80, 220), 1.2f)
    private val reticlePaint = stroke(Color.argb(160, 255, 255, 255), 1f)
    private val textPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        textSize = 11f * density
        setShadowLayer(2f, 0f, 0f, Color.BLACK)
    }
    private val rect = RectF()

    private fun stroke(c: Int, widthDp: Float) = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = c
        style = Paint.Style.STROKE
        strokeWidth = widthDp * resources.displayMetrics.density
    }

    override fun onDraw(canvas: Canvas) {
        val s = state ?: return
        if (s.previewMode == PreviewMode.WIDE) {
            val kx = width / s.sourceWidth.toDouble()
            val ky = height / s.sourceHeight.toDouble()
            rect.set((s.crop.left * kx).toFloat(), (s.crop.top * ky).toFloat(), ((s.crop.left + s.crop.width) * kx).toFloat(), ((s.crop.top + s.crop.height) * ky).toFloat())
            canvas.drawRect(rect, cropPaint)
        } else {
            val cx = width / 2f
            val cy = height / 2f
            val r = 14 * density
            canvas.drawLine(cx - r, cy, cx - r / 3, cy, reticlePaint)
            canvas.drawLine(cx + r / 3, cy, cx + r, cy, reticlePaint)
            canvas.drawLine(cx, cy - r, cx, cy - r / 3, reticlePaint)
            canvas.drawLine(cx, cy + r / 3, cx, cy + r, reticlePaint)
        }
        for (d in s.detections) {
            mapBox(s, d.box, rect)
            if (d.source == DetectionSource.SKY_BLOB) {
                canvas.drawCircle(rect.centerX(), rect.centerY(), maxOf(rect.width(), rect.height(), 8 * density), blobPaint)
            } else {
                canvas.drawRect(rect, neuralPaint)
                canvas.drawText("${d.label} ${(d.score * 100).toInt()}%", rect.left, rect.top - 3 * density, textPaint)
            }
        }
        s.target?.let { t ->
            mapBox(s, t, rect)
            // Keep tiny targets visible.
            val minHalf = 10 * density
            if (rect.width() < 2 * minHalf) rect.inset(rect.width() / 2 - minHalf, 0f)
            if (rect.height() < 2 * minHalf) rect.inset(0f, rect.height() / 2 - minHalf)
            val p = if (s.trackState == TrackState.TRACKING) trackPaint else coastPaint
            drawCorners(canvas, rect, p)
        }
    }

    private fun drawCorners(canvas: Canvas, r: RectF, p: Paint) {
        val l = minOf(r.width(), r.height()) / 3
        canvas.drawLine(r.left, r.top, r.left + l, r.top, p); canvas.drawLine(r.left, r.top, r.left, r.top + l, p)
        canvas.drawLine(r.right, r.top, r.right - l, r.top, p); canvas.drawLine(r.right, r.top, r.right, r.top + l, p)
        canvas.drawLine(r.left, r.bottom, r.left + l, r.bottom, p); canvas.drawLine(r.left, r.bottom, r.left, r.bottom - l, p)
        canvas.drawLine(r.right, r.bottom, r.right - l, r.bottom, p); canvas.drawLine(r.right, r.bottom, r.right, r.bottom - l, p)
    }

    /** Source pixels → view pixels for the current preview mode. */
    private fun mapBox(s: OverlayState, b: Box, out: RectF) {
        val (ox, oy, sw, sh) = viewport(s)
        val kx = width / sw
        val ky = height / sh
        out.set(((b.left - ox) * kx).toFloat(), ((b.top - oy) * ky).toFloat(), ((b.right - ox) * kx).toFloat(), ((b.bottom - oy) * ky).toFloat())
    }

    private fun viewport(s: OverlayState): DoubleArray =
        if (s.previewMode == PreviewMode.WIDE) doubleArrayOf(0.0, 0.0, s.sourceWidth.toDouble(), s.sourceHeight.toDouble())
        else doubleArrayOf(s.crop.left, s.crop.top, s.crop.width, s.crop.height)

    @SuppressLint("ClickableViewAccessibility")
    override fun onTouchEvent(event: MotionEvent): Boolean {
        if (event.actionMasked == MotionEvent.ACTION_UP) {
            val s = state ?: return true
            val (ox, oy, sw, sh) = viewport(s)
            onTap?.invoke(Vec2(ox + event.x / width * sw, oy + event.y / height * sh))
        }
        return true
    }
}
