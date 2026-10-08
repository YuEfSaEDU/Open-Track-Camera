package com.opentrack.camera.camera

import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CaptureRequest

/**
 * Experimental: Honor's camera HAL (Qualcomm based) takes vendor session keys that tell it
 * which app is asking (`packageName`, `thirdPartyCamera`, `hwCamera2Flag`). Third-party apps
 * get 30 fps at most; these keys are what the stock camera app is presumed to send. Their
 * value types are not published, so each key is tried with the likely types until the
 * framework accepts one. Nothing here is guaranteed to have any effect.
 */
object VendorKeys {
    private const val PREFIX = "com.hihonor.capture.metadata."
    private const val STOCK_CAMERA = "com.hihonor.camera"

    /** What the last [apply] did, for the camera report and the status toast. */
    @Volatile var lastLog: String = "-"
        private set

    fun isAvailable(c: CameraCharacteristics): Boolean =
        c.availableSessionKeys.any { it.name.startsWith(PREFIX) }

    /** Sets the keys this camera declares on [b]; returns a short log. */
    fun apply(b: CaptureRequest.Builder, c: CameraCharacteristics): String {
        val declared = (c.availableSessionKeys.map { it.name } + c.availableCaptureRequestKeys.map { it.name }).toSet()
        val log = ArrayList<String>()
        fun number(name: String, value: Int) {
            if (PREFIX + name !in declared) return
            log += name + "=" + (setFirst(b, PREFIX + name, listOf(value.toByte(), value, value.toLong())) ?: "reddedildi")
        }
        number("hwCamera2Flag", 1)
        number("thirdPartyCamera", 0)
        if (PREFIX + "packageName" in declared) {
            val bytes = (STOCK_CAMERA + "\u0000").toByteArray()
            log += "packageName=" + (setFirst(b, PREFIX + "packageName", listOf(STOCK_CAMERA, bytes)) ?: "reddedildi")
        }
        return log.joinToString().ifEmpty { "anahtar yok" }.also { lastLog = it }
    }

    /** Tries [values] in order; returns the accepted value's type name, or null. */
    @Suppress("UNCHECKED_CAST")
    private fun setFirst(b: CaptureRequest.Builder, name: String, values: List<Any>): String? {
        for (v in values) {
            try {
                val key = CaptureRequest.Key(name, v.javaClass as Class<Any>)
                b.set(key, v)
                return v.javaClass.simpleName
            } catch (_: Exception) {
                // Wrong type for this tag: try the next one.
            }
        }
        return null
    }
}
