package com.opentrack.camera.ui

import android.content.Context
import android.util.AttributeSet
import android.widget.FrameLayout

/** FrameLayout that sizes itself to the largest [aspectRatio] rectangle fitting its bounds. */
class AspectRatioFrameLayout @JvmOverloads constructor(context: Context, attrs: AttributeSet? = null) : FrameLayout(context, attrs) {
    var aspectRatio: Double = 16.0 / 9.0
        set(value) {
            field = value
            requestLayout()
        }

    override fun onMeasure(widthMeasureSpec: Int, heightMeasureSpec: Int) {
        val maxW = MeasureSpec.getSize(widthMeasureSpec)
        val maxH = MeasureSpec.getSize(heightMeasureSpec)
        var w = maxW
        var h = (maxW / aspectRatio).toInt()
        if (h > maxH) {
            h = maxH
            w = (maxH * aspectRatio).toInt()
        }
        super.onMeasure(MeasureSpec.makeMeasureSpec(w, MeasureSpec.EXACTLY), MeasureSpec.makeMeasureSpec(h, MeasureSpec.EXACTLY))
    }
}
