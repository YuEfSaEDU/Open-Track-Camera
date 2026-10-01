package com.opentrack.camera.camera

/** User adjustable capture settings, applied to the repeating request. */
data class CameraSettings(
    /** Camera2 zoom ratio; on multi-camera phones values above ~3 switch to the telephoto lens. */
    val zoomRatio: Float = 1f,
    /** Lock focus at infinity: aircraft are always "far", and AF hunts badly on plain sky. */
    val focusInfinity: Boolean = true,
    val aeCompensation: Int = 0,
    /**
     * Shutter priority exposure time (0 = automatic exposure). A short shutter keeps each frame
     * sharp: stabilization removes the shake between frames, not the blur inside them. ISO is
     * then regulated by [com.opentrack.tracking.ShutterPriorityExposure].
     */
    val shutterNs: Long = 2_000_000L, // 1/500 s
    val aeLock: Boolean = false,
)
