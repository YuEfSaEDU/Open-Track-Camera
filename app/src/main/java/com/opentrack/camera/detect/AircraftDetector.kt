package com.opentrack.camera.detect

import android.content.Context
import android.graphics.Bitmap
import android.util.Log
import com.google.mediapipe.framework.image.BitmapImageBuilder
import com.google.mediapipe.tasks.core.BaseOptions
import com.google.mediapipe.tasks.core.Delegate
import com.google.mediapipe.tasks.vision.core.RunningMode
import com.google.mediapipe.tasks.vision.objectdetector.ObjectDetector
import com.opentrack.tracking.Box
import com.opentrack.tracking.Detection
import com.opentrack.tracking.DetectionSource

/**
 * Neural aircraft detector: MediaPipe EfficientDet-Lite (COCO), restricted to the
 * "airplane" class. Additional classes (birds, drones, ...) can be enabled later through
 * [labels] or a custom model.
 */
class AircraftDetector private constructor(private val detector: ObjectDetector) : AutoCloseable {

    /**
     * Runs detection on [bitmap] (analysis resolution). Boxes are converted to source pixels
     * with [scale] = source px per bitmap px.
     */
    fun detect(bitmap: Bitmap, timestampNs: Long, scale: Double): List<Detection> {
        val result = detector.detect(BitmapImageBuilder(bitmap).build())
        return result.detections().mapNotNull { d ->
            val category = d.categories().firstOrNull() ?: return@mapNotNull null
            val r = d.boundingBox()
            Detection(
                box = Box.fromEdges(r.left.toDouble(), r.top.toDouble(), r.right.toDouble(), r.bottom.toDouble()).scaled(scale),
                score = category.score(),
                label = category.categoryName(),
                source = DetectionSource.NEURAL,
                timestampNs = timestampNs,
            )
        }
    }

    override fun close() = detector.close()

    companion object {
        const val MODEL = "models/efficientdet_lite0.tflite"
        val labels = listOf("airplane")

        fun create(context: Context, scoreThreshold: Float = 0.35f): AircraftDetector? {
            for (delegate in listOf(Delegate.GPU, Delegate.CPU)) {
                try {
                    val options = ObjectDetector.ObjectDetectorOptions.builder()
                        .setBaseOptions(BaseOptions.builder().setModelAssetPath(MODEL).setDelegate(delegate).build())
                        .setRunningMode(RunningMode.IMAGE)
                        .setMaxResults(5)
                        .setScoreThreshold(scoreThreshold)
                        .setCategoryAllowlist(labels)
                        .build()
                    return AircraftDetector(ObjectDetector.createFromOptions(context, options))
                } catch (e: Exception) {
                    Log.w("AircraftDetector", "delegate $delegate unavailable", e)
                }
            }
            return null
        }
    }
}
