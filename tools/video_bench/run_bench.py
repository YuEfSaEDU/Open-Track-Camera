#!/usr/bin/env python3
"""Replay real aircraft videos through the Open Track Camera tracking stack.

For each video:
  1. decode to 640-wide 8-bit luma (the app's analysis resolution),
  2. run the same MediaPipe EfficientDet-Lite0 "airplane" detector the app uses (~10 Hz),
  3. run the Kotlin tracker/framing/auto-zoom code (``:bench``) frame by frame,
  4. compute metrics and render a side-by-side video (wide view | output crop) plus a
     contact sheet of sampled frames.

Requirements: pip install imageio-ffmpeg numpy pillow mediapipe ; ./gradlew :bench:installDist
Recorded videos have no gyro data, so this measures the no-gyro path (the hardest case for
stabilization: hand shake is indistinguishable from target motion).
"""
import argparse
import csv
import json
import math
import os
import re
import subprocess
import sys

import imageio_ffmpeg
import numpy as np
from PIL import Image, ImageDraw

FFMPEG = imageio_ffmpeg.get_ffmpeg_exe()
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MODEL = os.path.join(ROOT, "app/src/main/assets/models/efficientdet_lite0.tflite")
BENCH = os.path.join(ROOT, "bench/build/install/bench/bin/bench")
AW = 640  # analysis width, as in MainActivity


def probe(path):
    out = subprocess.run([FFMPEG, "-hide_banner", "-i", path], capture_output=True, text=True).stderr
    m = re.search(r"Video:.*?(\d{3,5})x(\d{3,5})", out)
    fps = re.search(r"([\d.]+) fps", out)
    return int(m.group(1)), int(m.group(2)), float(fps.group(1))


def frames(path, w, h, fmt, max_s, scale=None):
    """Yields decoded frames as numpy arrays."""
    ch = 1 if fmt == "gray" else 3
    vf = f"scale={w}:{h}:flags=area,format={fmt}" if scale else f"format={fmt}"
    cmd = [FFMPEG, "-loglevel", "error", "-i", path, "-t", str(max_s), "-vf", vf, "-f", "rawvideo", "-pix_fmt", fmt, "-"]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=10**8)
    n = w * h * ch
    while True:
        b = p.stdout.read(n)
        if len(b) < n:
            break
        yield np.frombuffer(b, np.uint8).reshape((h, w, ch) if ch > 1 else (h, w))
    p.wait()


def neural_detections(path, aw, ah, fps, sx, max_s, out_csv):
    import mediapipe as mp
    from mediapipe.tasks.python import BaseOptions, vision

    opts = vision.ObjectDetectorOptions(
        base_options=BaseOptions(model_asset_path=MODEL), score_threshold=0.35, max_results=5, category_allowlist=["airplane"]
    )
    det = vision.ObjectDetector.create_from_options(opts)
    step = max(1, round(fps * 0.1))  # ~10 Hz like AnalysisEngine.neuralIntervalMs
    n = 0
    with open(out_csv, "w") as f:
        f.write("frame,x0,y0,x1,y1,score\n")
        for i, rgb in enumerate(frames(path, aw, ah, "rgb24", max_s, scale=True)):
            if i % step:
                continue
            res = det.detect(mp.Image(image_format=mp.ImageFormat.SRGB, data=np.ascontiguousarray(rgb)))
            for d in res.detections:
                b = d.bounding_box
                f.write(f"{i},{b.origin_x*sx:.1f},{b.origin_y*sx:.1f},{(b.origin_x+b.width)*sx:.1f},{(b.origin_y+b.height)*sx:.1f},{d.categories[0].score:.3f}\n")
                n += 1
    det.close()
    return n


def load_csv(p):
    with open(p) as f:
        return list(csv.DictReader(f))


def metrics(rows, dets, fps, src_w):
    n = len(rows)
    states = [r["state"] for r in rows]
    count = {s: states.count(s) / n for s in ("TRACKING", "COASTING", "LOST", "IDLE")}
    first = next((i for i, s in enumerate(states) if s == "TRACKING"), None)
    acq = sum(1 for a, b in zip(states, states[1:]) if b == "TRACKING" and a in ("IDLE", "LOST"))
    has = [r for r in rows if r["state"] in ("TRACKING", "COASTING")]
    offs = [max(abs(float(r["tx"]) - float(r["cx"])) / (float(r["cw"]) / 2), abs(float(r["ty"]) - float(r["cy"])) / (float(r["ch"]) / 2)) for r in has]

    def second_diff_rms(key_x, key_y, subset):
        v = []
        for a, b, c in zip(subset, subset[1:], subset[2:]):
            if int(c["frame"]) - int(a["frame"]) != 2:
                continue
            ax = float(a[key_x]) - 2 * float(b[key_x]) + float(c[key_x])
            ay = float(a[key_y]) - 2 * float(b[key_y]) + float(c[key_y])
            v.append(ax * ax + ay * ay)
        return math.sqrt(sum(v) / len(v)) if v else float("nan")

    # Weak ground truth: neural boxes. Agreement = tracked center within the (1.5x) box.
    by_frame = {}
    for d in dets:
        by_frame.setdefault(int(d["frame"]), []).append(d)
    agree = total = 0
    for r in has:
        ds = by_frame.get(int(r["frame"]))
        if not ds:
            continue
        total += 1
        tx, ty = float(r["tx"]), float(r["ty"])
        for d in ds:
            x0, y0, x1, y1 = (float(d[k]) for k in ("x0", "y0", "x1", "y1"))
            mx, my = (x1 - x0) * 0.25, (y1 - y0) * 0.25
            if x0 - mx <= tx <= x1 + mx and y0 - my <= ty <= y1 + my:
                agree += 1
                break
    zooms = [float(r["zoom"]) for r in rows]
    ms = [float(r["ms"]) for r in rows]
    return {
        "frames": n,
        "seconds": round(n / fps, 1),
        "state_ratio": {k: round(v, 3) for k, v in count.items()},
        "first_lock_s": None if first is None else round(first / fps, 2),
        "acquisitions": acq,
        "target_offset_mean": round(sum(offs) / len(offs), 3) if offs else None,
        "target_inside_crop": round(sum(o <= 1 for o in offs) / len(offs), 3) if offs else None,
        # Smoothness in px/frame^2 normalized by source width (x1000); lower is smoother.
        "crop_jerk_x1000": round(second_diff_rms("cx", "cy", has) / src_w * 1000, 3),
        "target_jerk_x1000": round(second_diff_rms("tx", "ty", has) / src_w * 1000, 3),
        "neural_agreement": None if total == 0 else round(agree / total, 3),
        "neural_frames_checked": total,
        "zoom_min": round(min(zooms), 2),
        "zoom_max": round(max(zooms), 2),
        "tracker_ms_avg": round(sum(ms) / n, 2),
        "tracker_ms_p99": round(sorted(ms)[int(n * 0.99) - 1], 2),
    }


COLORS = {"TRACKING": (60, 230, 90), "COASTING": (255, 200, 40), "LOST": (255, 60, 60), "IDLE": (180, 180, 180)}


def render(path, src_w, src_h, fps, rows, dets, max_s, out_mp4, sheet_png, sheet_every_s):
    pw = 960
    ph = round(pw * src_h / src_w / 2) * 2
    ow, oh = 960, 540
    by_frame = {}
    for d in dets:
        by_frame.setdefault(int(d["frame"]), []).append(d)
    enc = subprocess.Popen(
        [FFMPEG, "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{pw + ow}x{max(ph, oh)}", "-r", str(fps), "-i", "-",
         "-c:v", "libx264", "-preset", "veryfast", "-crf", "26", "-pix_fmt", "yuv420p", out_mp4],
        stdin=subprocess.PIPE,
    )
    k = pw / src_w
    sheet = []
    every = max(1, round(sheet_every_s * fps))
    last_dets = []
    last_det_frame = 0
    for i, rgb in enumerate(frames(path, src_w, src_h, "rgb24", max_s)):
        if i >= len(rows):
            break
        r = rows[i]
        img = Image.fromarray(rgb)
        cx, cy, cw, ch = (float(r[x]) for x in ("cx", "cy", "cw", "ch"))
        crop = img.crop((round(cx - cw / 2), round(cy - ch / 2), round(cx + cw / 2), round(cy + ch / 2))).resize((ow, oh), Image.BILINEAR)
        wide = img.resize((pw, ph), Image.BILINEAR)
        dr = ImageDraw.Draw(wide)
        dr.rectangle([(cx - cw / 2) * k, (cy - ch / 2) * k, (cx + cw / 2) * k, (cy + ch / 2) * k], outline=(255, 255, 255), width=2)
        if i in by_frame:
            last_dets, last_det_frame = by_frame[i], i
        if last_dets and i - last_det_frame > fps * 0.3:
            last_dets = []  # detections are ~10 Hz; don't draw stale boxes
        for d in last_dets:
            dr.rectangle([float(d["x0"]) * k, float(d["y0"]) * k, float(d["x1"]) * k, float(d["y1"]) * k], outline=(80, 200, 255), width=1)
        col = COLORS[r["state"]]
        if r["state"] in ("TRACKING", "COASTING"):
            tx, ty = float(r["tx"]) * k, float(r["ty"]) * k
            hw = max(float(r["tw"]) * k / 2, 8)
            hh = max(float(r["th"]) * k / 2, 8)
            dr.rectangle([tx - hw, ty - hh, tx + hw, ty + hh], outline=col, width=2)
        dr.text((6, 6), f"{i / fps:5.1f}s {r['state']} zoom {float(r['zoom']):.2f}x", fill=col)
        cd = ImageDraw.Draw(crop)
        cd.line([ow / 2 - 12, oh / 2, ow / 2 + 12, oh / 2], fill=(255, 255, 255))
        cd.line([ow / 2, oh / 2 - 12, ow / 2, oh / 2 + 12], fill=(255, 255, 255))
        canvas = Image.new("RGB", (pw + ow, max(ph, oh)))
        canvas.paste(wide, (0, 0))
        canvas.paste(crop, (pw, 0))
        enc.stdin.write(canvas.tobytes())
        if i % every == 0:
            sheet.append(canvas.resize(((pw + ow) // 2, max(ph, oh) // 2)))
    enc.stdin.close()
    enc.wait()
    if sheet:
        cols = 2
        rws = math.ceil(len(sheet) / cols)
        tw, th = sheet[0].size
        s = Image.new("RGB", (tw * cols, th * rws))
        for j, t in enumerate(sheet):
            s.paste(t, ((j % cols) * tw, (j // cols) * th))
        s.save(sheet_png)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("videos", nargs="+")
    ap.add_argument("--out", default="bench_out")
    ap.add_argument("--max-seconds", type=float, default=60)
    ap.add_argument("--auto-zoom", action="store_true")
    ap.add_argument("--zoom-min", type=float, default=1.0)
    ap.add_argument("--zoom-max", type=float, default=3.0)
    ap.add_argument("--zoom", type=float, default=None, help="fixed crop zoom when auto zoom is off")
    ap.add_argument("--mode", choices=["hybrid", "neural", "sky"], default="hybrid")
    ap.add_argument("--no-render", action="store_true")
    ap.add_argument("--reuse", action="store_true", help="reuse cached neural detections in --out")
    ap.add_argument("--sheet-every", type=float, default=3.0)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    summary = {}
    for v in a.videos:
        name = os.path.splitext(os.path.basename(v))[0][:40]
        w, h, fps = probe(v)
        ah = round(AW * h / w / 2) * 2
        sx = w / AW
        luma = os.path.join(a.out, name + ".gray")
        with open(luma, "wb") as f:
            for fr in frames(v, AW, ah, "gray", a.max_seconds, scale=True):
                f.write(fr.tobytes())
        dets_csv = os.path.join(a.out, name + ".dets.csv")
        if a.reuse and os.path.exists(dets_csv):
            n_det = len(load_csv(dets_csv))
        else:
            n_det = neural_detections(v, AW, ah, fps, sx, a.max_seconds, dets_csv)
        track_csv = os.path.join(a.out, name + ".track.csv")
        out_w = 1920 if w >= 2560 else min(1280, w)
        out_h = round(out_w * 9 / 16)
        cmd = [BENCH, "--luma", luma, "--width", str(AW), "--height", str(ah), "--source-width", str(w), "--source-height", str(h),
               "--fps", str(fps), "--dets", dets_csv, "--out", track_csv, "--mode", a.mode,
               "--output-width", str(out_w), "--output-height", str(out_h)]
        if a.zoom is not None:
            cmd += ["--zoom", str(a.zoom)]
        if a.auto_zoom:
            cmd += ["--auto-zoom", "--zoom-min", str(a.zoom_min), "--zoom-max", str(a.zoom_max)]
        subprocess.run(cmd, check=True)
        os.remove(luma)
        rows = load_csv(track_csv)
        dets = load_csv(dets_csv)
        m = metrics(rows, dets, fps, w)
        m.update({"video": os.path.basename(v), "resolution": f"{w}x{h}@{fps:g}", "neural_detections": n_det})
        summary[name] = m
        print(json.dumps(m), flush=True)
        if not a.no_render:
            render(v, w, h, fps, rows, dets, a.max_seconds, os.path.join(a.out, name + ".mp4"), os.path.join(a.out, name + ".sheet.jpg"), a.sheet_every)
    with open(os.path.join(a.out, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)


if __name__ == "__main__":
    main()
