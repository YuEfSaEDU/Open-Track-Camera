package com.opentrack.tracking

/**
 * One dimensional constant-velocity Kalman filter (state = position, velocity) with a
 * continuous white-noise-acceleration process model, so it handles irregular frame
 * intervals correctly.
 *
 * @param accelNoise spectral density of the unmodelled acceleration, in units²/s³.
 *   Larger values let the filter follow manoeuvres faster at the cost of more jitter.
 */
class KalmanCv1D(var accelNoise: Double) {
    var position = 0.0
        private set
    var velocity = 0.0
        private set

    // Covariance [[p00, p01], [p01, p11]]
    private var p00 = 0.0
    private var p01 = 0.0
    private var p11 = 0.0

    val positionVariance: Double get() = p00

    fun reset(position: Double, positionVariance: Double, velocity: Double = 0.0, velocityVariance: Double) {
        this.position = position
        this.velocity = velocity
        p00 = positionVariance
        p01 = 0.0
        p11 = velocityVariance
    }

    fun predict(dt: Double) {
        if (dt <= 0.0) return
        position += velocity * dt
        val q = accelNoise
        val dt2 = dt * dt
        val dt3 = dt2 * dt
        // P = F P F^T + Q
        val n00 = p00 + 2 * dt * p01 + dt2 * p11 + q * dt3 / 3
        val n01 = p01 + dt * p11 + q * dt2 / 2
        val n11 = p11 + q * dt
        p00 = n00
        p01 = n01
        p11 = n11
    }

    /** Fuses a position measurement with variance [measurementVariance]. Returns the innovation. */
    fun update(measurement: Double, measurementVariance: Double): Double {
        val innovation = measurement - position
        val s = p00 + measurementVariance
        val k0 = p00 / s
        val k1 = p01 / s
        position += k0 * innovation
        velocity += k1 * innovation
        val n00 = (1 - k0) * p00
        val n01 = (1 - k0) * p01
        val n11 = p11 - k1 * p01
        p00 = n00
        p01 = n01
        p11 = n11
        return innovation
    }
}
