package com.opentrack.camera.camera

/** User adjustable capture settings, applied to the repeating request. */
data class CameraSettings(
    /** Camera2 zoom ratio; on multi-camera phones values above ~3 switch to the telephoto lens. */
    val zoomRatio: Float = 1f,
    /** Lock focus at infinity: aircraft are always "far", and AF hunts badly on plain sky. */
    val focusInfinity: Boolean = true,
    /**
     * Brightness in thirds of a stop, -9..+9 (−3..+3 EV). With automatic exposure it becomes
     * the camera's exposure compensation; in shutter priority it moves the brightness the
     * ISO / exposure time loop aims for.
     */
    val evThirds: Int = 0,
    /**
     * Shutter priority exposure time (0 = automatic exposure). A short shutter keeps each frame
     * sharp: stabilization removes the shake between frames, not the blur inside them. ISO is
     * then regulated by [com.opentrack.tracking.ShutterPriorityExposure]. [SHUTTER_SMART] picks
     * the time from the motion.
     */
    val shutterNs: Long = 2_000_000L, // 1/500 s
    val aeLock: Boolean = false,
) {
    companion object {
        /**
         * [shutterNs] value for the smart shutter: shutter priority whose preferred time follows
         * how fast the subject crosses the sensor (see [com.opentrack.tracking.SmartShutter]).
         */
        const val SHUTTER_SMART = -1L
    }
}
