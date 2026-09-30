package com.opentrack.tracking

/**
 * Integrates gyroscope samples and turns the camera rotation into an image-space offset
 * (see [CameraMotion]). Uses the small-angle pinhole approximation: a rotation of θ radians
 * moves the image by roughly f·θ pixels, which is accurate near the optical center and good
 * enough over the short horizons the tracker and framing filters care about.
 *
 * Thread safe: samples are pushed from the sensor thread and queried from the render and
 * analysis threads.
 */
class GyroMotionModel(capacity: Int = 4096) : CameraMotion {

    /**
     * Linear map from the device-frame integrated angles (θx, θy) to image displacement
     * direction. Depends on how the device is held relative to the image; see [forDisplayRotation].
     */
    data class AxisMap(val xFromThetaX: Double, val xFromThetaY: Double, val yFromThetaX: Double, val yFromThetaY: Double)

    /** Focal length expressed in source pixels (update when the zoom ratio changes). */
    @Volatile
    var focalPx: Double = 0.0

    @Volatile
    var axisMap: AxisMap = forDisplayRotation(90)

    /** Added to camera timestamps to express them in the gyroscope clock (readout/exposure skew). */
    @Volatile
    var cameraToGyroOffsetNs: Long = 0L

    /** Maximum distance the model extrapolates past the latest gyro sample. */
    var maxExtrapolationNs: Long = 30_000_000L

    private val times = LongArray(capacity)
    private val thetaX = DoubleArray(capacity)
    private val thetaY = DoubleArray(capacity)
    private val thetaZ = DoubleArray(capacity)
    private var head = -1
    private var count = 0
    private var lastRateX = 0.0
    private var lastRateY = 0.0
    private var lastRateZ = 0.0
    private val lock = Any()

    fun reset() = synchronized(lock) {
        head = -1
        count = 0
        lastRateX = 0.0
        lastRateY = 0.0
        lastRateZ = 0.0
    }

    /** Angular rates in rad/s around the device X, Y and Z axes (Android sensor convention). */
    fun onGyroSample(timestampNs: Long, rateX: Double, rateY: Double, rateZ: Double = 0.0) = synchronized(lock) {
        if (count == 0) {
            push(timestampNs, 0.0, 0.0, 0.0)
        } else {
            val prevT = times[head]
            if (timestampNs <= prevT) return@synchronized
            val dt = (timestampNs - prevT) * 1e-9
            // Trapezoidal integration.
            val tx = thetaX[head] + 0.5 * (lastRateX + rateX) * dt
            val ty = thetaY[head] + 0.5 * (lastRateY + rateY) * dt
            val tz = thetaZ[head] + 0.5 * (lastRateZ + rateZ) * dt
            push(timestampNs, tx, ty, tz)
        }
        lastRateX = rateX
        lastRateY = rateY
        lastRateZ = rateZ
    }

    private fun push(t: Long, tx: Double, ty: Double, tz: Double) {
        head = (head + 1) % times.size
        times[head] = t
        thetaX[head] = tx
        thetaY[head] = ty
        thetaZ[head] = tz
        if (count < times.size) count++
    }

    /**
     * Rotation about the optical axis: the device turning counter-clockwise (positive rate
     * around device Z, which points out of the screen towards the user) makes the scene seen
     * by the back camera turn clockwise on screen, i.e. positive image roll.
     */
    @Volatile
    var rollSign: Double = 1.0

    override fun poseAt(timestampNs: Long): CameraPose {
        val f = focalPx
        if (f <= 0.0) return CameraPose.IDENTITY
        val a = synchronized(lock) { anglesAt(timestampNs + cameraToGyroOffsetNs) } ?: return CameraPose.IDENTITY
        val m = axisMap
        val offset = Vec2(f * (m.xFromThetaX * a[0] + m.xFromThetaY * a[1]), f * (m.yFromThetaX * a[0] + m.yFromThetaY * a[1]))
        return CameraPose(offset, rollSign * a[2])
    }

    private fun anglesAt(t: Long): DoubleArray? {
        if (count == 0) return null
        val size = times.size
        val newest = head
        if (t >= times[newest]) {
            val dt = (minOf(t - times[newest], maxExtrapolationNs)) * 1e-9
            return doubleArrayOf(thetaX[newest] + lastRateX * dt, thetaY[newest] + lastRateY * dt, thetaZ[newest] + lastRateZ * dt)
        }
        val oldest = (head - count + 1 + size) % size
        if (t <= times[oldest]) return doubleArrayOf(thetaX[oldest], thetaY[oldest], thetaZ[oldest])
        // Binary search over the logical (chronological) index.
        var lo = 0
        var hi = count - 1
        while (hi - lo > 1) {
            val mid = (lo + hi) ushr 1
            if (times[(oldest + mid) % size] <= t) lo = mid else hi = mid
        }
        val a = (oldest + lo) % size
        val b = (oldest + hi) % size
        val span = (times[b] - times[a]).toDouble()
        val w = if (span <= 0) 0.0 else (t - times[a]) / span
        return doubleArrayOf(
            thetaX[a] + (thetaX[b] - thetaX[a]) * w,
            thetaY[a] + (thetaY[b] - thetaY[a]) * w,
            thetaZ[a] + (thetaZ[b] - thetaZ[a]) * w,
        )
    }

    companion object {
        /**
         * Axis map for the back camera when the UI is shown at [displayRotationDegrees]
         * (the value of Display.getRotation() in degrees) and frames are rendered upright
         * for that orientation.
         *
         * Derivation (device axes: x right, y up, z out of the screen; the back camera looks
         * along -z): a rotation ω·dt moves a point at infinity by (+ωy·dt, -ωx·dt) in the
         * device x/y plane. Projecting the device axes onto the upright image axes for each
         * display rotation gives the matrices below.
         */
        fun forDisplayRotation(displayRotationDegrees: Int): AxisMap = when (((displayRotationDegrees % 360) + 360) % 360) {
            0 -> AxisMap(0.0, 1.0, 1.0, 0.0)      // image x = +device x, image y = -device y
            90 -> AxisMap(1.0, 0.0, 0.0, -1.0)    // image x = -device y, image y = -device x
            180 -> AxisMap(0.0, -1.0, -1.0, 0.0)
            270 -> AxisMap(-1.0, 0.0, 0.0, 1.0)   // image x = +device y, image y = +device x
            else -> throw IllegalArgumentException("rotation must be a multiple of 90: $displayRotationDegrees")
        }
    }
}
