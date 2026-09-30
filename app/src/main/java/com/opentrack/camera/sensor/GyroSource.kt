package com.opentrack.camera.sensor

import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Handler
import android.os.HandlerThread
import com.opentrack.tracking.GyroMotionModel

/** Streams the (bias-calibrated) gyroscope into a [GyroMotionModel] at the fastest rate. */
class GyroSource(private val sensors: SensorManager, private val model: GyroMotionModel) : SensorEventListener {
    private var thread: HandlerThread? = null

    val available: Boolean get() = sensors.getDefaultSensor(Sensor.TYPE_GYROSCOPE) != null

    fun start() {
        val gyro = sensors.getDefaultSensor(Sensor.TYPE_GYROSCOPE) ?: return
        val t = HandlerThread("gyro").apply { start() }
        thread = t
        model.reset()
        sensors.registerListener(this, gyro, SensorManager.SENSOR_DELAY_FASTEST, Handler(t.looper))
    }

    fun stop() {
        sensors.unregisterListener(this)
        thread?.quitSafely()
        thread = null
    }

    override fun onSensorChanged(event: SensorEvent) {
        model.onGyroSample(event.timestamp, event.values[0].toDouble(), event.values[1].toDouble())
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit
}
