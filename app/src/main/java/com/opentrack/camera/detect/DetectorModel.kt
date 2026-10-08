package com.opentrack.camera.detect

/** Neural detector model (MediaPipe .tflite with labels in its metadata). */
enum class DetectorModel(val asset: String, val inputPx: Int, val scoreThreshold: Float) {
    /**
     * The app's own model (tools/detector): airplane, helicopter, drone, bird, trained with
     * small, distant aircraft; float16, 384 px input.
     */
    AIRCRAFT("models/aircraft_fp16.tflite", 384, AIRCRAFT_THRESHOLD),

    /** COCO EfficientDet-Lite0, int8, 320 px: the only one with land and water vehicles. */
    COCO("models/efficientdet_lite0.tflite", 320, 0.35f);

    companion object {
        /** The model for [target]: vehicles need COCO; aircraft use the own model unless [preferCoco]. */
        fun forTarget(target: TargetClasses, preferCoco: Boolean): DetectorModel =
            if (target == TargetClasses.VEHICLES || preferCoco) COCO else AIRCRAFT
    }
}

/**
 * The own model scores real aircraft higher than COCO does (median 0.69 vs 0.61 on the
 * SOLOTÜRK videos) and its false alarms too; this threshold gives about COCO's precision at 0.35.
 */
private const val AIRCRAFT_THRESHOLD = 0.5f
