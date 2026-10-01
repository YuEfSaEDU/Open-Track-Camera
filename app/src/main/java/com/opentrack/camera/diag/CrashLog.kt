package com.opentrack.camera.diag

import android.app.ActivityManager
import android.app.ApplicationExitInfo
import android.content.Context
import java.io.File

/**
 * Remembers why the app last died, for the camera report: the stack trace of an uncaught
 * Java exception (written when it happens), or Android's record of the process exit (native
 * crashes, ANRs, the system killing it), read on the next launch.
 */
object CrashLog {
    private const val FILE = "last_crash.txt"

    fun install(context: Context) {
        val file = File(context.filesDir, FILE)
        val previous = Thread.getDefaultUncaughtExceptionHandler()
        Thread.setDefaultUncaughtExceptionHandler { thread, e ->
            runCatching { file.writeText("Çökme (${thread.name}, ${java.util.Date()}):\n${e.stackTraceToString()}") }
            previous?.uncaughtException(thread, e)
        }
    }

    /** Text about the last abnormal exit, or null when the app last ended normally. */
    fun last(context: Context): String? {
        val parts = ArrayList<String>()
        val am = context.getSystemService(ActivityManager::class.java)
        val exit = runCatching { am.getHistoricalProcessExitReasons(context.packageName, 0, 1).firstOrNull() }.getOrNull()
        if (exit != null && exit.reason in setOf(
                ApplicationExitInfo.REASON_CRASH, ApplicationExitInfo.REASON_CRASH_NATIVE, ApplicationExitInfo.REASON_ANR,
                ApplicationExitInfo.REASON_LOW_MEMORY, ApplicationExitInfo.REASON_INITIALIZATION_FAILURE,
            )
        ) {
            val kind = when (exit.reason) {
                ApplicationExitInfo.REASON_CRASH -> "Java çökmesi"
                ApplicationExitInfo.REASON_CRASH_NATIVE -> "native çökme"
                ApplicationExitInfo.REASON_ANR -> "yanıt vermiyor (ANR)"
                ApplicationExitInfo.REASON_LOW_MEMORY -> "bellek yetersiz"
                else -> "başlatma hatası"
            }
            parts += "Son çıkış: $kind, ${java.util.Date(exit.timestamp)}: ${exit.description}"
            // ANR traces are text; native tombstones are binary protobuf, take the readable strings.
            runCatching {
                exit.traceInputStream?.use { s ->
                    val bytes = s.readBytes()
                    val text = if (exit.reason == ApplicationExitInfo.REASON_ANR) String(bytes)
                    else Regex("[\\x20-\\x7e]{6,}").findAll(String(bytes, Charsets.ISO_8859_1)).take(80).joinToString("\n") { it.value }
                    parts += text.take(6000)
                }
            }
        }
        val file = File(context.filesDir, FILE)
        if (file.exists()) parts += file.readText().take(6000)
        return parts.takeIf { it.isNotEmpty() }?.joinToString("\n")
    }

    fun clear(context: Context) {
        File(context.filesDir, FILE).delete()
    }
}
