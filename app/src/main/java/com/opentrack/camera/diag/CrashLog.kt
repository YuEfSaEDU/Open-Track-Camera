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
    /** Timestamp of the last process exit already reported (each is reported once). */
    private const val SEEN_EXIT = "last_exit_seen.txt"

    fun install(context: Context) {
        val file = File(context.filesDir, FILE)
        val previous = Thread.getDefaultUncaughtExceptionHandler()
        val version = runCatching { context.packageManager.getPackageInfo(context.packageName, 0).versionName }.getOrNull()
        val built = runCatching { context.packageManager.getPackageInfo(context.packageName, 0).lastUpdateTime }.getOrNull()?.let { java.util.Date(it) }
        Thread.setDefaultUncaughtExceptionHandler { thread, e ->
            runCatching { file.writeText("Çökme (${thread.name}, ${java.util.Date()}, sürüm $version kurulum $built):\n${e.stackTraceToString()}") }
            previous?.uncaughtException(thread, e)
        }
    }

    /**
     * Text about the last abnormal exit, or null when the app last ended normally. Each crash is
     * returned once (by the launch after it), so an old crash does not show up in every report.
     */
    fun last(context: Context): String? {
        val parts = ArrayList<String>()
        val am = context.getSystemService(ActivityManager::class.java)
        val seen = File(context.filesDir, SEEN_EXIT)
        val seenTs = runCatching { seen.readText().trim().toLong() }.getOrDefault(0L)
        val exit = runCatching { am.getHistoricalProcessExitReasons(context.packageName, 0, 1).firstOrNull() }.getOrNull()
            ?.takeIf { it.timestamp != seenTs }
        exit?.let { e -> runCatching { seen.writeText(e.timestamp.toString()) } }
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
        if (file.exists()) {
            parts += file.readText().take(6000)
            file.delete()
        }
        return parts.takeIf { it.isNotEmpty() }?.joinToString("\n")
    }

    fun clear(context: Context) {
        File(context.filesDir, FILE).delete()
    }
}
