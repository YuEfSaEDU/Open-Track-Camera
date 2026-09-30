package com.opentrack.tracking

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs
import kotlin.random.Random

class PathSmootherTest {
    private val dt = 1.0 / 60

    /** Reference: dense normal equations solved with Gaussian elimination. */
    private fun dense(goal: DoubleArray, a: DoubleArray, x0: DoubleArray, fixed: Int, w: PathSmoother.Weights): DoubleArray {
        val n = goal.size
        val m = Array(n) { DoubleArray(n) }
        val b = DoubleArray(n)
        for (i in 0 until n) { m[i][i] += a[i]; b[i] += a[i] * goal[i] }
        val coeffs = listOf(doubleArrayOf(-1.0, 1.0) to w.velocity / (dt * dt), doubleArrayOf(1.0, -2.0, 1.0) to w.acceleration / Math.pow(dt, 4.0),
            doubleArrayOf(-1.0, 3.0, -3.0, 1.0) to w.jerk / Math.pow(dt, 6.0))
        for ((c, wt) in coeffs) for (k in 0 until n - c.size + 1) for (i in c.indices) for (j in c.indices) m[k + i][k + j] += wt * c[i] * c[j]
        // Fix the first samples.
        val x = x0.copyOf()
        val free = (fixed until n).toList()
        val mm = Array(free.size) { i -> DoubleArray(free.size) { j -> m[free[i]][free[j]] } }
        val bb = DoubleArray(free.size) { i -> b[free[i]] - (0 until fixed).sumOf { j -> m[free[i]][j] * x0[j] } }
        for (col in bb.indices) {
            val p = (col until bb.size).maxBy { abs(mm[it][col]) }
            mm[col] = mm[p].also { mm[p] = mm[col] }; bb[col] = bb[p].also { bb[p] = bb[col] }
            for (r in col + 1 until bb.size) {
                val f = mm[r][col] / mm[col][col]
                for (cc in col until bb.size) mm[r][cc] -= f * mm[col][cc]
                bb[r] -= f * bb[col]
            }
        }
        for (r in bb.indices.reversed()) {
            var v = bb[r]
            for (cc in r + 1 until bb.size) v -= mm[r][cc] * x[free[cc]]
            x[free[r]] = v / mm[r][r]
        }
        return x
    }

    @Test
    fun bandedSolverMatchesDenseSolution() {
        val rnd = Random(5)
        val n = 30
        val goal = DoubleArray(n) { it * 0.01 + rnd.nextDouble(-0.05, 0.05) }
        val a = DoubleArray(n) { if (it % 7 == 3) 0.02 else 1.0 }
        val x = DoubleArray(n) { if (it < 3) goal[it] else 0.0 }
        val w = PathSmoother.Weights(1e-4, 1e-5, 1e-7, l1 = false, iterations = 1)
        val expected = dense(goal, a, x, 3, w)
        val inf = DoubleArray(n) { Double.POSITIVE_INFINITY }
        PathSmoother().solve(x, goal, a, DoubleArray(n) { Double.NEGATIVE_INFINITY }, inf, 3, dt, w)
        for (i in 0 until n) assertEquals("x[$i]", expected[i], x[i], 1e-7)
    }

    @Test
    fun respectsBoundsAndFixedSamples() {
        val n = 40
        val goal = DoubleArray(n) { if (it < 20) 0.0 else 1.0 } // a step the smoother would round off
        val a = DoubleArray(n) { 1.0 }
        val lo = DoubleArray(n) { if (it >= 25) 0.9 else Double.NEGATIVE_INFINITY }
        val hi = DoubleArray(n) { Double.POSITIVE_INFINITY }
        val x = DoubleArray(n)
        PathSmoother().solve(x, goal, a, lo, hi, 3, dt, StabilizerConfig(smoothness = 0.8).pathWeights())
        for (i in 25 until n) assertTrue("x[$i]=${x[i]}", x[i] >= 0.9 - 1e-9)
        for (i in 0 until 3) assertEquals(0.0, x[i], 0.0)
    }

    @Test
    fun robustGoalResistsTrackerGlitches() {
        val n = 60
        val goal = DoubleArray(n) { 0.2 * it / n }
        // A tracker re-seed glitch: the measured position sits off by a third of the crop for 6 frames.
        for (i in 28..33) goal[i] += 0.3
        val a = DoubleArray(n) { 1.0 }
        val inf = DoubleArray(n) { Double.POSITIVE_INFINITY }
        val ninf = DoubleArray(n) { Double.NEGATIVE_INFINITY }
        val base = StabilizerConfig(smoothness = 0.3).pathWeights()
        val plain = DoubleArray(n)
        PathSmoother().solve(plain, goal, a, ninf, inf, 0, dt, base)
        val robust = DoubleArray(n)
        PathSmoother().solve(robust, goal, a, ninf, inf, 0, dt, base.copy(robustGoalScale = 0.02))
        val clean = 0.2 * 30 / n
        assertTrue("plain L2 is pulled by the glitch: ${plain[30]}", plain[30] - clean > 0.01)
        // Measured: plain ≈ 0.011, robust ≈ 0.005 of a crop width.
        assertTrue("robust path is pulled less: ${robust[30]}", abs(robust[30] - clean) < (plain[30] - clean) * 0.6)
    }
}
