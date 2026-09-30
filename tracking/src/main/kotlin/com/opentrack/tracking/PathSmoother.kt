package com.opentrack.tracking

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

/**
 * One dimensional camera path optimizer over a short window of frames:
 *
 *   minimize  Σ a_k (x_k − g_k)²                      (stay near the goal: target / EIS path)
 *           + Σ w1 (Δx)² / dt² + w2 (Δ²x)² / dt⁴ + w3 (Δ³x)² / dt⁶   (velocity, acceleration, jerk)
 *   subject to lo_k ≤ x_k ≤ hi_k, and the first [fixed] samples equal to the already emitted path.
 *
 * The derivative terms are scaled by the frame interval so behaviour does not depend on the
 * frame rate. With [l1] set, iteratively reweighted least squares turns the derivative terms
 * into (approximately) L1 norms, which favour piecewise static / constant-velocity /
 * constant-acceleration segments: the "tripod, dolly, ease" look of optimal stabilizers.
 * Bounds are enforced with an active-set penalty and a final clamp.
 *
 * The normal equations are a symmetric band matrix (half bandwidth 3) solved with a banded
 * LDLᵀ factorization in O(n).
 */
class PathSmoother {
    data class Weights(
        val velocity: Double,
        val acceleration: Double,
        val jerk: Double,
        val l1: Boolean,
        val iterations: Int = 4,
        /**
         * Penalty weight pulling a sample that violates its bound back to it (relative to a goal
         * weight of 1). NaN = effectively hard (dominates every other term). Moderate values
         * bend the path gently towards the bound instead of pinning it, which avoids polynomial
         * swings in short windows; the final clamp still guarantees the bound.
         */
        val constraintWeight: Double = Double.NaN,
        /**
         * Huber scale for the goal term (same units as x). Goal samples further than this from
         * the path get proportionally less weight, so isolated jumps in the goal (tracker
         * re-seeds, detector outliers) barely move the camera. NaN disables it.
         */
        val robustGoalScale: Double = Double.NaN,
    )

    private var n = 0
    private var band = Array(0) { DoubleArray(4) }
    private var rhs = DoubleArray(0)
    private var l = Array(0) { DoubleArray(4) }
    private var d = DoubleArray(0)

    /**
     * @param x initial guess and output (length n); the first [fixed] entries are kept as is.
     * @param dt frame interval in seconds.
     */
    fun solve(
        x: DoubleArray,
        goal: DoubleArray,
        goalWeight: DoubleArray,
        lo: DoubleArray,
        hi: DoubleArray,
        fixed: Int,
        dt: Double,
        weights: Weights,
    ) {
        val count = x.size
        if (count == 0 || fixed >= count) return
        ensure(count)
        val w1 = DoubleArray(count)
        val w2 = DoubleArray(count)
        val w3 = DoubleArray(count)
        val penalty = DoubleArray(count)
        val penaltyTarget = DoubleArray(count)
        val base1 = weights.velocity / (dt * dt)
        val base2 = weights.acceleration / (dt * dt * dt * dt)
        val base3 = weights.jerk / (dt * dt * dt * dt * dt * dt)
        w1.fill(base1)
        w2.fill(base2)
        w3.fill(base3)
        // Large compared to every other term, so violated bounds are pulled back hard.
        val bigPenalty = if (weights.constraintWeight.isNaN()) {
            1e3 * (1 + goalWeight.maxOrNull()!! + 4 * base1 + 16 * base2 + 64 * base3)
        } else {
            weights.constraintWeight
        }

        val robust = !weights.robustGoalScale.isNaN()
        val a = goalWeight.copyOf()
        for (iter in 0 until max(1, weights.iterations)) {
            build(count, goal, a, w1, w2, w3, penalty, penaltyTarget)
            solveBanded(count, x, fixed)
            var changed = false
            for (k in fixed until count) {
                if (x[k] < lo[k] - 1e-9 && penalty[k] == 0.0) { penalty[k] = bigPenalty; penaltyTarget[k] = lo[k]; changed = true }
                if (x[k] > hi[k] + 1e-9 && penalty[k] == 0.0) { penalty[k] = bigPenalty; penaltyTarget[k] = hi[k]; changed = true }
            }
            if (robust) {
                // Huber IRLS weights on the goal residual.
                val delta = weights.robustGoalScale
                for (k in 0 until count) {
                    val r = abs(x[k] - goal[k])
                    a[k] = if (r <= delta) goalWeight[k] else goalWeight[k] * delta / r
                }
            }
            if (weights.l1) {
                reweight(x, w1, 1, base1, dt)
                reweight(x, w2, 2, base2, dt)
                reweight(x, w3, 3, base3, dt)
            } else if (!changed && !robust) {
                break
            }
        }
        for (k in fixed until count) x[k] = x[k].coerceIn(min(lo[k], hi[k]), max(lo[k], hi[k]))
    }

    /** IRLS: w = base · s / max(|Dᵐx| / dtᵐ, ε) makes Σ w (Dᵐx)² ≈ base · s · Σ |Dᵐx| / dtᵐ. */
    private fun reweight(x: DoubleArray, w: DoubleArray, order: Int, base: Double, dt: Double) {
        if (base == 0.0) return
        val c = COEFFS[order]
        var scale = 0.0
        val mags = DoubleArray(x.size)
        for (k in 0 until x.size - order) {
            var v = 0.0
            for (a in 0..order) v += c[a] * x[k + a]
            mags[k] = abs(v) / Math.pow(dt, order.toDouble())
            scale += mags[k]
        }
        val rows = x.size - order
        if (rows <= 0) return
        // Normalize so the average weight stays at `base` (keeps λ meaning comparable to L2).
        val mean = max(scale / rows, 1e-12)
        val eps = 0.05 * mean + 1e-9
        for (k in 0 until rows) w[k] = base * mean / max(mags[k], eps)
    }

    private fun ensure(count: Int) {
        if (n >= count) return
        n = count
        band = Array(count) { DoubleArray(4) }
        rhs = DoubleArray(count)
        l = Array(count) { DoubleArray(4) }
        d = DoubleArray(count)
    }

    private fun build(
        count: Int, goal: DoubleArray, a: DoubleArray, w1: DoubleArray, w2: DoubleArray, w3: DoubleArray,
        penalty: DoubleArray, penaltyTarget: DoubleArray,
    ) {
        for (i in 0 until count) {
            band[i].fill(0.0)
            band[i][0] = a[i] + penalty[i]
            rhs[i] = a[i] * goal[i] + penalty[i] * penaltyTarget[i]
        }
        addDifference(count, 1, w1)
        addDifference(count, 2, w2)
        addDifference(count, 3, w3)
    }

    private fun addDifference(count: Int, order: Int, w: DoubleArray) {
        val c = COEFFS[order]
        for (k in 0 until count - order) {
            val wk = w[k]
            if (wk == 0.0) continue
            for (i in 0..order) for (j in i..order) {
                band[k + i][j - i] += wk * c[i] * c[j]
            }
        }
    }

    /** Solves the band system with x[0 until fixed] held constant (moved to the right hand side). */
    private fun solveBanded(count: Int, x: DoubleArray, fixed: Int) {
        // Eliminate fixed unknowns.
        for (i in 0 until count) {
            if (i < fixed) continue
            for (k in 1..3) {
                val j = i - k
                if (j in 0 until fixed) rhs[i] -= band[j][k] * x[j]
            }
        }
        val m = count - fixed
        // LDLᵀ on the free block (indices fixed until count).
        for (ii in 0 until m) {
            val i = ii + fixed
            // Columns left to right (k descending): L(i, i-k) needs L(i, q) for the columns before it.
            for (k in 3 downTo 1) {
                val jj = ii - k
                if (jj < 0) { l[ii][k] = 0.0; continue }
                val j = jj + fixed
                var v = band[j][k] // A(j, i) with i = j + k
                for (q in k + 1..3) {
                    val kk = ii - q
                    if (kk < 0) continue
                    // L(i, kk) D(kk) L(j, kk) where j - kk = q - k
                    v -= l[ii][q] * d[kk] * l[jj][q - k]
                }
                l[ii][k] = v / d[jj]
            }
            var diag = band[i][0]
            for (k in 1..3) if (ii - k >= 0) diag -= l[ii][k] * l[ii][k] * d[ii - k]
            d[ii] = if (diag > 1e-12) diag else 1e-12
        }
        // Forward (L y = b), diagonal, backward (Lᵀ x = z).
        val y = DoubleArray(m)
        for (ii in 0 until m) {
            var v = rhs[ii + fixed]
            for (k in 1..3) if (ii - k >= 0) v -= l[ii][k] * y[ii - k]
            y[ii] = v
        }
        for (ii in 0 until m) y[ii] /= d[ii]
        for (ii in m - 1 downTo 0) {
            var v = y[ii]
            for (k in 1..3) if (ii + k < m) v -= l[ii + k][k] * y[ii + k]
            y[ii] = v
        }
        for (ii in 0 until m) x[ii + fixed] = y[ii]
    }

    private companion object {
        val COEFFS = arrayOf(
            doubleArrayOf(1.0),
            doubleArrayOf(-1.0, 1.0),
            doubleArrayOf(1.0, -2.0, 1.0),
            doubleArrayOf(-1.0, 3.0, -3.0, 1.0),
        )
    }
}
