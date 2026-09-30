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
