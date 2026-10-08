package com.opentrack.camera.diag

import android.content.ContentResolver
import android.content.ContentValues
import android.net.Uri
import android.provider.MediaStore
import android.util.Log
import java.io.BufferedWriter
import java.io.OutputStreamWriter
import java.util.Locale
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.atomic.AtomicInteger
import kotlin.concurrent.thread

/**
 * Frame-by-frame log of a recording (JSON lines, one object per line, saved next to the video
 * in Download/OpenTrack). It records what the phone actually did — camera settings per frame,
 * the exposure loop's decisions, detections, the tracker's state, the live and the recorded
 * crop — so a recording can be diagnosed (and tuned against) without guessing from the video.
 *
 * Every record has `k` (kind) and `ts` (camera timestamp, ns) where it applies; see
 * docs/ARCHITECTURE.md for the kinds. Lines are built on the calling thread and written on a
 * background thread; when the writer falls behind, lines are dropped and counted, never
 * blocking the camera or render threads.
 */
class Telemetry private constructor(private val resolver: ContentResolver, private val uri: Uri, private val writer: BufferedWriter) {
    private val queue = LinkedBlockingQueue<String>(QUEUE_LINES)
    private val dropped = AtomicInteger()
    @Volatile private var closed = false
    private val worker = thread(name = "telemetry") {
        try {
            while (true) {
                val line = queue.take()
                if (line === END) break
                writer.write(line)
                writer.write("\n")
            }
        } catch (e: Exception) {
            Log.e(TAG, "telemetry write failed", e)
        }
    }

    /** Starts a record of kind [kind]; finish it with [Line.send]. */
    fun line(kind: String): Line = Line(kind, ::offer)

    internal fun offer(line: String) {
        if (closed || !queue.offer(line)) dropped.incrementAndGet()
    }

    /** Flushes, closes and publishes the file. */
    fun close() {
        if (closed) return
        line("end").i("dropped", dropped.get().toLong()).send()
        closed = true
        queue.put(END)
        worker.join(3000)
        runCatching { writer.close() }
        runCatching { resolver.update(uri, ContentValues().apply { put(MediaStore.MediaColumns.IS_PENDING, 0) }, null, null) }
    }

    /** One JSON object, built field by field. */
    class Line internal constructor(kind: String, private val sink: (String) -> Unit) {
        private val sb = StringBuilder(160).append("{\"k\":\"").append(kind).append('"')

        fun i(key: String, v: Long): Line = apply { key(key).append(v) }
        fun i(key: String, v: Int): Line = i(key, v.toLong())
        /** A number rounded to [digits] decimals (null, NaN and infinite become null). */
        fun n(key: String, v: Double?, digits: Int = 1): Line = apply { key(key).append(if (v == null) "null" else num(v, digits)) }
        fun b(key: String, v: Boolean): Line = apply { key(key).append(if (v) "true" else "false") }
        fun s(key: String, v: String?): Line = apply { key(key); if (v == null) sb.append("null") else quote(v) }
        /** An array of numbers. */
        fun a(key: String, vararg v: Double, digits: Int = 1): Line = apply {
            key(key).append('[')
            v.forEachIndexed { k, x -> if (k > 0) sb.append(','); sb.append(num(x, digits)) }
            sb.append(']')
        }
        /** Pre-built JSON (an array or object). */
        fun raw(key: String, json: String): Line = apply { key(key).append(json) }

        fun send() = sink(sb.append('}').toString())

        /** The fields so far (for comparing records). */
        override fun toString(): String = sb.toString()

        private fun key(k: String): StringBuilder = sb.append(",\"").append(k).append("\":")

        private fun quote(v: String) {
            sb.append('"')
            for (c in v) when {
                c == '"' -> sb.append("\\\"")
                c == '\\' -> sb.append("\\\\")
                c < ' ' -> sb.append(' ')
                else -> sb.append(c)
            }
            sb.append('"')
        }
    }

    companion object {
        private const val TAG = "Telemetry"
        private const val END = "\u0000end"
        /** ~10 s of lines at the usual rate. */
        private const val QUEUE_LINES = 4096

        fun num(v: Double, digits: Int): String =
            if (v.isNaN() || v.isInfinite()) "null" else if (digits == 0) Math.round(v).toString() else String.format(Locale.US, "%.${digits}f", v)

        /** Creates Download/OpenTrack/[baseName].jsonl; null when that fails (the recording goes on without it). */
        fun create(resolver: ContentResolver, baseName: String): Telemetry? = try {
            val values = ContentValues().apply {
                put(MediaStore.MediaColumns.DISPLAY_NAME, "$baseName.jsonl")
                // octet-stream keeps the .jsonl extension (a text type would get .txt appended).
                put(MediaStore.MediaColumns.MIME_TYPE, "application/octet-stream")
                put(MediaStore.MediaColumns.RELATIVE_PATH, "Download/OpenTrack")
                put(MediaStore.MediaColumns.IS_PENDING, 1)
            }
            val uri = resolver.insert(MediaStore.Downloads.getContentUri(MediaStore.VOLUME_EXTERNAL_PRIMARY), values)
                ?: error("MediaStore insert failed")
            val stream = resolver.openOutputStream(uri) ?: error("cannot open $uri")
            Telemetry(resolver, uri, BufferedWriter(OutputStreamWriter(stream, Charsets.UTF_8), 1 shl 16))
        } catch (e: Exception) {
            Log.e(TAG, "telemetry not started", e)
            null
        }
    }
}
