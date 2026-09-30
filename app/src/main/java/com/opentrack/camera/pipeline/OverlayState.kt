package com.opentrack.camera.pipeline

import com.opentrack.tracking.Box
import com.opentrack.tracking.CropWindow
import com.opentrack.tracking.Detection
import com.opentrack.tracking.TrackState

enum class PreviewMode {
    /** Whole sensor frame with the crop window drawn on top: best for spotting / acquiring. */
    WIDE,
    /** Exactly what is being recorded. */
    OUTPUT,
}

/** Snapshot published by the render thread for the UI overlay. All boxes in source pixels. */
data class OverlayState(
    val sourceWidth: Int,
    val sourceHeight: Int,
    val previewMode: PreviewMode,
    val crop: CropWindow,
    /** Current digital zoom (changes on its own in auto zoom mode). */
    val zoom: Double,
    val trackState: TrackState,
    val target: Box?,
    val detections: List<Detection>,
    val fps: Double,
    val analysisMs: Double,
    val neuralMs: Double,
)
