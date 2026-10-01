package com.opentrack.camera.detect

import android.content.Context
import android.graphics.Bitmap
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
    private var closed = false

    /**
     * Runs detection on [bitmap], an image of the source [region] (source px; the whole frame
     * or a zoomed part of it). Boxes are returned in source pixels.
     */
    @Synchronized
    fun detect(bitmap: Bitmap, timestampNs: Long, region: Box): List<Detection> {
        // One instance serves the live pipeline and gallery processing in turn; calls are
        // serialized and a closed detector just reports nothing.
        if (closed) return emptyList()
        val result = detector.detect(BitmapImageBuilder(bitmap).build())
        val kx = region.width / bitmap.width
        val ky = region.height / bitmap.height
        return result.detections().mapNotNull { d ->
            val category = d.categories().firstOrNull() ?: return@mapNotNull null
            val r = d.boundingBox()
            Detection(
                box = Box.fromEdges(
                    region.left + r.left * kx, region.top + r.top * ky, region.left + r.right * kx, region.top + r.bottom * ky,
                ),
                score = category.score(),
                label = category.categoryName(),
                source = DetectionSource.NEURAL,
                timestampNs = timestampNs,
            )
        }
    }

    @Synchronized
    override fun close() {
        if (closed) return
        closed = true
        detector.close()
    }

    companion object {
        const val MODEL = "models/efficientdet_lite0.tflite"
        val labels = listOf("airplane")

        /**
         * Creates the detector with [delegate]; throws when it is unavailable. Use it on the
         * thread that created it (see [DetectorHost]).
         */
        fun create(context: Context, delegate: Delegate, scoreThreshold: Float = 0.35f): AircraftDetector {
            val options = ObjectDetector.ObjectDetectorOptions.builder()
                .setBaseOptions(BaseOptions.builder().setModelAssetPath(MODEL).setDelegate(delegate).build())
                .setRunningMode(RunningMode.IMAGE)
                .setMaxResults(5)
                .setScoreThreshold(scoreThreshold)
                .setCategoryAllowlist(labels)
                .build()
            return AircraftDetector(ObjectDetector.createFromOptions(context, options))
        }
    }
}
