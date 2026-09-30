package com.opentrack.camera.camera

/** User adjustable capture settings, applied to the repeating request. */
data class CameraSettings(
    /** Camera2 zoom ratio; on multi-camera phones values above ~3 switch to the telephoto lens. */
    val zoomRatio: Float = 1f,
    /** Lock focus at infinity: aircraft are always "far", and AF hunts badly on plain sky. */
    val focusInfinity: Boolean = true,
    val aeCompensation: Int = 0,
    /** Emulated shutter priority: fixed short exposure, ISO derived from the last auto exposure. */
    val shutterPriority: Boolean = false,
    val shutterNs: Long = 1_000_000L, // 1/1000 s
    val aeLock: Boolean = false,
)
