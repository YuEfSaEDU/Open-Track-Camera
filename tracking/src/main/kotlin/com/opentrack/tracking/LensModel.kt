package com.opentrack.tracking

import kotlin.math.abs
import kotlin.math.atan
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.tan

/**
 * What a lens of the camera really does to the picture, per camera zoom ratio: its focal
 * length at the image center ([kappa] stream px per unit of the HAL's zoom ratio: the frame's
 * center focal at ratio z is kappa x z) and its radial distortion [a].
 *
 * Distortion model: a ray at pinhole (undistorted) radius r_u (normalized by the center focal)
 * lands at r_d = atan(a r_u) / a; a = 0 is a pinhole, a = 1 an equidistant fisheye, between
 * them the barrel distortion of wide angle lenses. One parameter, monotonic for any a >= 0.
 *
 * Phones report neither for third-party apps here (LENS_DISTORTION is all zeros, no HAL
 * correction), and the zoom ratio of a multi-camera's wide angle need not match its real
 * scale; straight lines then bend as stabilization moves the crop across the frame. The values
 * are measured on the phone by [LensCalibrator].
 */
data class LensModel(val kappa: Double, val a: Double = 0.0) {
    /** Center focal (stream px) of a frame taken at zoom ratio [zoom]. */
    fun focalAt(zoom: Double): Double = kappa * zoom

    companion object {
        /** Distorted radius for undistorted (pinhole) radius [ru], both normalized by the focal. */
        fun distort(ru: Double, a: Double): Double = if (a < 1e-6) ru else atan(a * ru) / a

        /** Inverse of [distort]; NaN beyond the lens's field of view (a r_d >= pi/2). */
        fun undistort(rd: Double, a: Double): Double {
            if (a < 1e-6) return rd
            val x = a * rd
            return if (x >= 1.55) Double.NaN else tan(x) / a
        }
    }
}

/**
 * Measures a lens's [LensModel] from the video and the gyroscope while the phone moves: each
 * background feature tracked from one frame to the next moved the way the camera's rotation
 * (gyro) moved its ray; how far it moved in pixels, at the center and towards the edges, gives
 * the center focal and the distortion. Fit by grid search with a robust (Huber) loss, so the
 * aircraft, cars or birds moving on their own do not matter.
 */
class LensCalibrator(
    /** The model in use (its kappa centers the search). */
    val prior: LensModel,
    val minSamples: Int = 1500,
    private val maxSamples: Int = 6000,
) {
    /**
     * One feature: positions in the two frames (stream px, relative to the image center, as
     * the lens delivered them), the frame's zoom ratio, and the camera's rotation between the
     * frames: tilt of the optical axis ([wx], [wy], radians, image x / y) then roll ([roll]).
     */
    class Sample(val x0: Double, val y0: Double, val x1: Double, val y1: Double, val zoom: Double, val wx: Double, val wy: Double, val roll: Double)

    data class Result(val model: LensModel, val rmsPx: Double, val priorRmsPx: Double, val samples: Int)

    private val samples = ArrayList<Sample>()
    private var next = 0

    val size: Int get() = samples.size

    fun add(s: Sample) {
        if (samples.size < maxSamples) samples += s else { samples[next] = s; next = (next + 1) % maxSamples }
    }

    fun clear() {
        samples.clear()
        next = 0
    }

    /** A copy of the samples (to [fit] on another thread). */
    fun snapshot(): List<Sample> = samples.toList()

    /** Best model for [data] (default: the samples so far), or null with too few. */
    fun fit(data: List<Sample> = snapshot()): Result? {
        if (data.size < minSamples) return null
        var bestK = 1.0
        var bestA = 0.0
        var best = Double.MAX_VALUE
        fun search(ks: DoubleProgression, `as`: DoubleProgression) {
            for (k in ks) for (a in `as`) {
                val c = cost(data, LensModel(prior.kappa * k, a))
                if (c < best) { best = c; bestK = k; bestA = a }
            }
        }
        search(DoubleProgression(0.6, 1.8, 0.02), DoubleProgression(0.0, 1.2, 0.05))
        search(DoubleProgression(bestK - 0.02, bestK + 0.02, 0.002), DoubleProgression(maxOf(0.0, bestA - 0.05), bestA + 0.05, 0.005))
        // Distortion only when it really explains the motion better (a narrow lens cannot tell).
        val flat = LensModel(prior.kappa * bestK, 0.0)
        val model = if (bestA > 0 && cost(data, flat) <= best * 1.01) flat else LensModel(prior.kappa * bestK, bestA)
        return Result(model, rms(data, model), rms(data, prior), data.size)
    }

    private fun cost(data: List<Sample>, m: LensModel): Double {
        var sum = 0.0
        for (s in data) {
            val e = error(s, m)
            sum += if (e <= HUBER_PX) e * e else 2 * HUBER_PX * e - HUBER_PX * HUBER_PX
        }
        return sum
    }

    private fun rms(data: List<Sample>, m: LensModel): Double {
        var sum = 0.0
        var n = 0
        for (s in data) {
            val e = error(s, m)
            if (e <= 4 * HUBER_PX) { sum += e * e; n++ }
        }
        return if (n == 0) Double.NaN else sqrt(sum / n)
    }

    /** Distance (stream px) between the measured second position and the model's prediction. */
    private fun error(s: Sample, m: LensModel): Double {
        val p = predict(s, m) ?: return 4 * HUBER_PX
        return hypot(p.x - s.x1, p.y - s.y1)
    }

    companion object {
        private const val HUBER_PX = 1.5

        /** Where the feature at ([Sample.x0], [Sample.y0]) lands after the rotation, under [m]. */
        fun predict(s: Sample, m: LensModel): Vec2? {
            val f = m.focalAt(s.zoom)
            val dx = s.x0 / f
            val dy = s.y0 / f
            val rd = hypot(dx, dy)
            val ru = LensModel.undistort(rd, m.a)
            if (ru.isNaN()) return null
            val k = if (rd < 1e-12) 1.0 else ru / rd
            val u = rotate(dx * k, dy * k, s.wx, s.wy, s.roll) ?: return null
            val ru1 = hypot(u.x, u.y)
            val k1 = if (ru1 < 1e-12) 1.0 else LensModel.distort(ru1, m.a) / ru1
            return Vec2(u.x * k1 * f, u.y * k1 * f)
        }

        /**
         * The pinhole image point (normalized) of ray (ux, uy, 1) after tilting the optical axis
         * by ([wx], [wy]) (the image center moves to (tan |w|) w / |w|) and rolling by [roll].
         */
        fun rotate(ux: Double, uy: Double, wx: Double, wy: Double, roll: Double): Vec2? {
            val t = hypot(wx, wy)
            var vx = ux
            var vy = uy
            var vz = 1.0
            if (t > 1e-12) {
                // Rodrigues about the axis (-wy, wx, 0) / t by angle t.
                val kx = -wy / t
                val ky = wx / t
                val c = cos(t)
                val sn = sin(t)
                val dot = kx * vx + ky * vy
                val cx = ky * vz
                val cy = -kx * vz
                val cz = kx * vy - ky * vx
                val nx = vx * c + cx * sn + kx * dot * (1 - c)
                val ny = vy * c + cy * sn + ky * dot * (1 - c)
                val nz = vz * c + cz * sn
                vx = nx; vy = ny; vz = nz
            }
            if (vz <= 1e-6) return null
            val px = vx / vz
            val py = vy / vz
            if (abs(roll) < 1e-12) return Vec2(px, py)
            val cr = cos(roll)
            val sr = sin(roll)
            return Vec2(px * cr - py * sr, px * sr + py * cr)
        }
    }
}

/**
 * How a frame relates to its lens: [key] names the physical lens, [halZoom] the zoom ratio it
 * was taken at, [mapZoom] the scale from the reference space to the frame (rectified stream px
 * per reference px), [model] the lens model in use, [focalRef] the reference space's focal
 * (the gyroscope's, px).
 */
data class FrameLens(val key: String, val halZoom: Double, val mapZoom: Double, val model: LensModel, val focalRef: Double) {
    /** Reference px → where the lens really put it: stream px relative to the image center. */
    fun toLens(ref: Vec2, center: Vec2): Vec2 {
        val s = (ref - center) * mapZoom
        val f = model.focalAt(halZoom)
        val ru = hypot(s.x, s.y) / f
        if (ru < 1e-12 || model.a < 1e-6) return s
        return s * (LensModel.distort(ru, model.a) / ru)
    }

    /** Same lens, ratio and mapping: features can be compared between the two frames. */
    fun sameAs(o: FrameLens): Boolean = key == o.key && abs(halZoom - o.halZoom) < 1e-4 && abs(mapZoom - o.mapZoom) < 1e-6 && model == o.model
}

/**
 * Lens calibration samples from features matched between two frames of the same lens ([pairs]:
 * analysis px in [a] and [b]), with the camera's rotation between them from [motion]
 * (reference px, focal [FrameLens.focalRef]).
 */
fun calibrationSamples(a: LumaFrame, b: LumaFrame, pairs: List<Pair<Vec2, Vec2>>, motion: CameraMotion): List<LensCalibrator.Sample> {
    val la = a.lens ?: return emptyList()
    val lb = b.lens ?: return emptyList()
    if (!la.sameAs(lb)) return emptyList()
    val pa = motion.poseAt(a.timestampNs)
    val pb = motion.poseAt(b.timestampNs)
    val w = (pb.offset - pa.offset) / la.focalRef
    val roll = pb.roll - pa.roll
    val center = Vec2(a.sourceWidth / 2, a.sourceHeight / 2)
    return pairs.map { (p0, p1) ->
        val s0 = la.toLens(a.toSource(p0), center)
        val s1 = lb.toLens(b.toSource(p1), center)
        LensCalibrator.Sample(s0.x, s0.y, s1.x, s1.y, la.halZoom, w.x, w.y, roll)
    }
}

/** A plain range of doubles (start..end inclusive, by step). */
class DoubleProgression(private val start: Double, private val end: Double, private val step: Double) : Iterable<Double> {
    override fun iterator(): Iterator<Double> = object : Iterator<Double> {
        private var i = 0
        override fun hasNext() = start + i * step <= end + 1e-9
        override fun next() = start + step * i++
    }
}
