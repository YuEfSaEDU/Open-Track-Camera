package com.opentrack.tracking

enum class DetectionSource { NEURAL, SKY_BLOB, USER }

/** A candidate object, [box] in source pixels, observed in the frame taken at [timestampNs]. */
data class Detection(
    val box: Box,
    val score: Float,
    val label: String,
    val source: DetectionSource,
    val timestampNs: Long,
)

/**
 * A completed neural detector run, reported even when it found nothing: which part of the
 * frame of [timestampNs] it looked at ([region] in source px, null = the whole frame) and at
 * what input size. The tracker uses it to tell "not an aircraft" from "not looked at".
 */
data class NeuralRun(val timestampNs: Long, val region: Box?, val inputPx: Int = 320)
