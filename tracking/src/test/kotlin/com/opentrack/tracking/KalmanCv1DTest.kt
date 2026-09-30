package com.opentrack.tracking

import org.junit.Assert.assertEquals
import org.junit.Test
import kotlin.random.Random

class KalmanCv1DTest {
    @Test
    fun convergesToConstantVelocity() {
        val f = KalmanCv1D(accelNoise = 100.0)
        f.reset(0.0, 100.0, 0.0, 1e6)
        val rnd = Random(3)
        val dt = 1.0 / 60
        for (i in 1..240) {
            f.predict(dt)
            val truth = 500.0 * i * dt
            f.update(truth + rnd.nextDouble(-2.0, 2.0), 4.0)
        }
        assertEquals(500.0, f.velocity, 10.0)
        assertEquals(500.0 * 4, f.position, 2.0)
    }

    @Test
    fun predictionGrowsUncertainty() {
        val f = KalmanCv1D(accelNoise = 1000.0)
        f.reset(0.0, 1.0, 0.0, 1.0)
        val before = f.positionVariance
        f.predict(0.5)
        assert(f.positionVariance > before)
    }
}
