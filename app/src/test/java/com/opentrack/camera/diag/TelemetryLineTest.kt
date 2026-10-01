package com.opentrack.camera.diag

import org.junit.Assert.assertEquals
import org.junit.Test

class TelemetryLineTest {
    private fun line(kind: String, build: Telemetry.Line.() -> Unit): String {
        var out = ""
        Telemetry.Line(kind) { out = it }.apply(build).send()
        return out
    }

    @Test
    fun buildsOneJsonObject() {
        val s = line("trk") {
            i("ts", 123456789012L).s("state", "TRACKING").n("conf", 0.876, 2).n("age", Double.NaN).n("aim", null)
                .b("framable", true).a("box", 1920.04, 1080.0, 33.333, 21.0).raw("dets", "[[\"airplane\",0.71,1,2,3,4,\"NEURAL\"]]")
        }
        assertEquals(
            "{\"k\":\"trk\",\"ts\":123456789012,\"state\":\"TRACKING\",\"conf\":0.88,\"age\":null,\"aim\":null," +
                "\"framable\":true,\"box\":[1920.0,1080.0,33.3,21.0],\"dets\":[[\"airplane\",0.71,1,2,3,4,\"NEURAL\"]]}",
            s,
        )
    }

    @Test
    fun escapesStringsAndUsesDotDecimalsInAnyLocale() {
        val saved = java.util.Locale.getDefault()
        java.util.Locale.setDefault(java.util.Locale.forLanguageTag("tr-TR"))
        try {
            val s = line("start") { s("device", "HONOR \"ELP\"\\x\n").n("focalPx", 2716.25, 1).n("t", 2.5, 0) }
            assertEquals("{\"k\":\"start\",\"device\":\"HONOR \\\"ELP\\\"\\\\x \",\"focalPx\":2716.3,\"t\":3}", s)
        } finally {
            java.util.Locale.setDefault(saved)
        }
    }
}
