package com.opentrack.tracking

import kotlin.math.hypot
import kotlin.random.Random

/** Renders simple synthetic analysis frames: a sky gradient, optional texture and dark "aircraft". */
object SyntheticSky {
    fun frame(
        width: Int = 320,
        height: Int = 180,
        timestampNs: Long = 0L,
        scale: Double = 1.0,
        objects: List<Pair<Vec2, Double>> = emptyList(),
        noise: Int = 2,
        seed: Int = 1,
        texturedBottomRows: Int = 0,
    ): LumaFrame {
        val rnd = Random(seed)
        val data = ByteArray(width * height)
        for (y in 0 until height) for (x in 0 until width) {
            var v = 150 + (y * 40) / height // brighter towards the horizon
            if (y >= height - texturedBottomRows) v = if (((x / 3) + (y / 3)) % 2 == 0) 60 else 200
            for ((c, r) in objects) {
                val d = hypot(x - c.x, y - c.y)
                if (d <= r) v = 40 else if (d <= r + 1) v = (40 + v) / 2
            }
            v += rnd.nextInt(-noise, noise + 1)
            data[y * width + x] = v.coerceIn(0, 255).toByte()
        }
        return LumaFrame(width, height, data, timestampNs, scale)
    }
}
