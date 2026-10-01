#!/usr/bin/env python3
"""Replay real aircraft videos through the Open Track Camera tracking stack.

For each video:
  1. decode to 640-wide 8-bit luma (the app's analysis resolution),
  2. run the same MediaPipe EfficientDet-Lite0 "airplane" detector the app uses (~10 Hz),
  3. run the Kotlin tracker/framing/auto-zoom code (``:bench``) frame by frame,
  4. compute metrics and render a side-by-side video (wide view | output crop) plus a
     contact sheet of sampled frames.

Requirements: pip install imageio-ffmpeg numpy opencv-python-headless mediapipe ; ./gradlew :bench:installDist
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
import cv2

from shake_metric import ShakeMeter, hf_rms

FFMPEG = imageio_ffmpeg.get_ffmpeg_exe()
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MODEL = os.path.join(ROOT, "app/src/main/assets/models/efficientdet_lite0.tflite")
LABELS = ["airplane"]  # target classes (--labels)
SCORE = 0.35
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

    det = create_detector()
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


def create_detector():
    from mediapipe.tasks.python import BaseOptions, vision

    opts = vision.ObjectDetectorOptions(
        base_options=BaseOptions(model_asset_path=MODEL), score_threshold=SCORE, max_results=5, category_allowlist=LABELS
    )
    return vision.ObjectDetector.create_from_options(opts)


ROI_INPUT = 320  # the app renders zoomed detector regions at this size (EfficientDet-Lite0 input)


def run_live(path, w, h, fps, aw, ah, max_s, cmd, roi_csv):
    """Runs the Kotlin bench in --live mode: streams frames and answers zoomed (ROI) detection requests."""
    import mediapipe as mp

    det = create_detector()
    p = subprocess.Popen(cmd + ["--live"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
    results = None
    with open(roi_csv, "w") as log:
        log.write("frame,kind,x0,y0,x1,y1,score\n")
        for i, rgb in enumerate(frames(path, w, h, "rgb24", max_s)):
            gray = cv2.cvtColor(cv2.resize(rgb, (aw, ah), interpolation=cv2.INTER_AREA), cv2.COLOR_RGB2GRAY)
            hdr = "F -1\n" if results is None else f"F {len(results)}\n" + "".join(f"{b[0]:.1f} {b[1]:.1f} {b[2]:.1f} {b[3]:.1f} {b[4]:.3f}\n" for b in results)
            try:
                p.stdin.write(hdr.encode() + gray.tobytes())
            except BrokenPipeError:
                break
            line = p.stdout.readline().decode().strip()
            if not line:
                break
            results = None
            if line.startswith("R"):
                x0, y0, x1, y1 = (float(v) for v in line.split()[1:5])
                ix0, iy0 = max(0, int(round(x0))), max(0, int(round(y0)))
                ix1, iy1 = min(w, int(round(x1))), min(h, int(round(y1)))
                crop = cv2.resize(rgb[iy0:iy1, ix0:ix1], (ROI_INPUT, ROI_INPUT), interpolation=cv2.INTER_AREA)
                res = det.detect(mp.Image(image_format=mp.ImageFormat.SRGB, data=np.ascontiguousarray(crop)))
                kx, ky = (ix1 - ix0) / ROI_INPUT, (iy1 - iy0) / ROI_INPUT
                results = []
                log.write(f"{i},roi,{ix0},{iy0},{ix1},{iy1},0\n")
                for d in res.detections:
                    b = d.bounding_box
                    r = (ix0 + b.origin_x * kx, iy0 + b.origin_y * ky, ix0 + (b.origin_x + b.width) * kx, iy0 + (b.origin_y + b.height) * ky, d.categories[0].score)
                    results.append(r)
                    log.write(f"{i},det,{r[0]:.1f},{r[1]:.1f},{r[2]:.1f},{r[3]:.1f},{r[4]:.3f}\n")
        p.stdin.close()
    p.wait()
    det.close()
    if p.returncode:
        raise RuntimeError(f"bench failed: {p.returncode}")


def load_csv(p):
    with open(p) as f:
        return list(csv.DictReader(f))


def gt_metrics(rows, gt):
    """Scores the track against dense tiled detections (gt_tiles.py): on an aircraft or not."""
    by_frame = {}
    for d in gt:
        by_frame.setdefault(int(d["frame"]), []).append(d)

    on = off = miss = 0
    # Framing as the viewer sees it: the ground-truth aircraft nearest to the output center.
    offsets = []
    for f in by_frame:
        if f >= len(rows):
            continue
        r = rows[f]
        cx, cy, cw, ch = (float(r[k]) for k in ("cx", "cy", "cw", "ch"))
        best = min(max(abs((float(d["x0"]) + float(d["x1"])) / 2 - cx) / (cw / 2), abs((float(d["y0"]) + float(d["y1"])) / 2 - cy) / (ch / 2))
                   for d in by_frame[f])
        offsets.append(best)
        if r["state"] not in ("TRACKING", "COASTING") or float(r["tx"]) < 0:
            miss += 1
            continue
        tx, ty = float(r["tx"]), float(r["ty"])
        hit = False
        for d in by_frame[f]:
            x0, y0, x1, y1 = (float(d[k]) for k in ("x0", "y0", "x1", "y1"))
            mx, my = max((x1 - x0) * 0.25, 8), max((y1 - y0) * 0.25, 8)
            if x0 - mx <= tx <= x1 + mx and y0 - my <= ty <= y1 + my:
                hit = True
                break
        on += hit
        off += not hit
    n = on + off + miss
    k = len(offsets)
    return {"gt_frames": n, "gt_on_target": round(on / n, 3) if n else None, "gt_off_target": round(off / n, 3) if n else None,
            "gt_missed": round(miss / n, 3) if n else None,
            "gt_center_offset": round(sorted(offsets)[k // 2], 3) if k else None,
            "gt_in_crop": round(sum(o <= 1 for o in offsets) / k, 3) if k else None,
            "gt_well_framed": round(sum(o <= 0.5 for o in offsets) / k, 3) if k else None}


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
    # Output framing: how far the aircraft sits from the output center (fraction of the half crop).
    centering = [max(abs(float(r["tx"]) - float(r["cx"])) / (float(r["cw"]) / 2), abs(float(r["ty"]) - float(r["cy"])) / (float(r["ch"]) / 2))
                 for r in rows if float(r["tx"]) >= 0]
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
        "zoom_mean": round(sum(zooms) / n, 2),
        "center_offset_mean": round(sum(centering) / len(centering), 3) if centering else None,
        "target_outside_crop": round(sum(c > 1 for c in centering) / len(centering), 3) if centering else None,
        "zoom_min": round(min(zooms), 2),
        "zoom_max": round(max(zooms), 2),
        "tracker_ms_avg": round(sum(ms) / n, 2),
        "tracker_ms_p99": round(sorted(ms)[int(n * 0.99) - 1], 2),
    }


COLORS = {"TRACKING": (60, 230, 90), "COASTING": (255, 200, 40), "LOST": (255, 60, 60), "IDLE": (180, 180, 180)}


def crop_affine(r, ow, oh):
    """2x3 matrix mapping output pixels -> source pixels for the (possibly rotated) crop."""
    cx, cy, cw, ch = (float(r[x]) for x in ("cx", "cy", "cw", "ch"))
    rot = float(r.get("rot", 0.0) or 0.0)
    sx, sy = cw / ow, ch / oh
    c, s = math.cos(rot), math.sin(rot)
    # p = center + R(rot) * diag(sx, sy) * (u - out_center)
    a = np.array([[c * sx, -s * sy], [s * sx, c * sy]])
    t = np.array([cx, cy]) - a @ np.array([ow / 2, oh / 2])
    return np.hstack([a, t[:, None]])


def target_in_output(r, ow, oh):
    m = crop_affine(r, ow, oh)
    a, t = m[:, :2], m[:, 2]
    return np.linalg.solve(a, np.array([float(r["tx"]), float(r["ty"])]) - t)


def render(path, src_w, src_h, fps, rows, dets, max_s, out_mp4, sheet_png, sheet_every_s, write_video=True):
    """Renders wide | output video + contact sheet and measures shake of input and output."""
    pw = 960
    ph = round(pw * src_h / src_w / 2) * 2
    ow, oh = 960, 540
    mw, mh = 640, 360  # measurement resolution of the output
    by_frame = {}
    for d in dets:
        by_frame.setdefault(int(d["frame"]), []).append(d)
    enc = None
    if write_video:
        enc = subprocess.Popen(
            [FFMPEG, "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{pw + ow}x{max(ph, oh)}", "-r", str(fps), "-i", "-",
             "-c:v", "libx264", "-preset", "veryfast", "-crf", "24", "-pix_fmt", "yuv420p", out_mp4],
            stdin=subprocess.PIPE,
        )
    k = pw / src_w
    in_w = 640
    in_h = round(in_w * src_h / src_w)
    kin = in_w / src_w
    out_w_ref = 1920.0
    meter_in = ShakeMeter(fps, px_scale=out_w_ref / in_w)
    meter_out = ShakeMeter(fps, px_scale=out_w_ref / mw)
    tgt_out = []
    sheet = []
    every = max(1, round(sheet_every_s * fps))
    last_dets, last_det_frame = [], 0
    for i, rgb in enumerate(frames(path, src_w, src_h, "rgb24", max_s)):
        if i >= len(rows):
            break
        r = rows[i]
        has_t = r["state"] in ("TRACKING", "COASTING")
        tw, th = float(r["tw"]), float(r["th"])
        # --- measurement ---
        gin = cv2.cvtColor(cv2.resize(rgb, (in_w, in_h), interpolation=cv2.INTER_AREA), cv2.COLOR_RGB2GRAY)
        excl_in = None
        if has_t:
            tx, ty = float(r["tx"]) * kin, float(r["ty"]) * kin
            hx, hy = max(tw * kin, 12), max(th * kin, 12)
            excl_in = (tx - hx, ty - hy, tx + hx, ty + hy)
        meter_in.add(gin, excl_in)
        m_meas = crop_affine(r, mw, mh)
        gout = cv2.cvtColor(cv2.warpAffine(rgb, m_meas, (mw, mh), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP, borderMode=cv2.BORDER_REPLICATE), cv2.COLOR_RGB2GRAY)
        excl_out = None
        if has_t:
            u = target_in_output(r, mw, mh)
            ks = mw / float(r["cw"])
            hx, hy = max(tw * ks, 16), max(th * ks, 16)
            excl_out = (u[0] - hx, u[1] - hy, u[0] + hx, u[1] + hy)
            tgt_out.append(target_in_output(r, out_w_ref, out_w_ref * 9 / 16))
        elif tgt_out:
            tgt_out.append(None)
        meter_out.add(gout, excl_out)
        # --- visualization ---
        if not (write_video or i % every == 0):
            continue
        crop = cv2.warpAffine(rgb, crop_affine(r, ow, oh), (ow, oh), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP, borderMode=cv2.BORDER_CONSTANT)
        wide = cv2.resize(rgb, (pw, ph), interpolation=cv2.INTER_AREA)
        corners = crop_affine(r, 1, 1) @ np.array([[0, 1, 1, 0], [0, 0, 1, 1], [1, 1, 1, 1]], dtype=float)
        cv2.polylines(wide, [np.round(corners.T * k).astype(np.int32)], True, (255, 255, 255), 2)
        if i in by_frame:
            last_dets, last_det_frame = by_frame[i], i
        if last_dets and i - last_det_frame > fps * 0.3:
            last_dets = []  # detections are ~10 Hz; don't draw stale boxes
        for d in last_dets:
            kind = d.get("roi")
            color = (220, 80, 255) if kind == "box" else (80, 255, 255) if kind == "1" else (80, 200, 255)
            cv2.rectangle(wide, (int(float(d["x0"]) * k), int(float(d["y0"]) * k)), (int(float(d["x1"]) * k), int(float(d["y1"]) * k)), color, 1)
        col = COLORS[r["state"]]
        if has_t:
            tx, ty = float(r["tx"]) * k, float(r["ty"]) * k
            hw, hh = max(tw * k / 2, 8), max(th * k / 2, 8)
            cv2.rectangle(wide, (int(tx - hw), int(ty - hh)), (int(tx + hw), int(ty + hh)), col, 2)
        cv2.putText(wide, f"{i / fps:5.1f}s {r['state']} zoom {float(r['zoom']):.2f}x", (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
        cv2.line(crop, (ow // 2 - 12, oh // 2), (ow // 2 + 12, oh // 2), (255, 255, 255), 1)
        cv2.line(crop, (ow // 2, oh // 2 - 12), (ow // 2, oh // 2 + 12), (255, 255, 255), 1)
        canvas = np.zeros((max(ph, oh), pw + ow, 3), np.uint8)
        canvas[:ph, :pw] = wide
        canvas[:oh, pw:] = crop
        if enc:
            enc.stdin.write(canvas.tobytes())
        if i % every == 0:
            sheet.append(cv2.resize(canvas, ((pw + ow) // 2, max(ph, oh) // 2), interpolation=cv2.INTER_AREA))
    if enc:
        enc.stdin.close()
        enc.wait()
    if sheet and sheet_png:
        cols = 2
        rws = math.ceil(len(sheet) / cols)
        th_, tw_ = sheet[0].shape[:2]
        s = np.zeros((th_ * rws, tw_ * cols, 3), np.uint8)
        for j, t in enumerate(sheet):
            s[(j // cols) * th_:(j // cols + 1) * th_, (j % cols) * tw_:(j % cols + 1) * tw_] = t
        cv2.imwrite(sheet_png, cv2.cvtColor(s, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 85])
    tin = meter_in.result()
    tout = meter_out.result()
    # Target jitter in the output (px at 1920 wide), over the longest runs with a target.
    runs, cur = [], []
    for u in tgt_out:
        if u is None:
            if len(cur) > fps:
                runs.append(cur)
            cur = []
        else:
            cur.append(u)
    if len(cur) > fps:
        runs.append(cur)
    jit = [hf_rms(run, fps) for run in runs]
    n = sum(len(run) for run in runs)
    target_jitter = None if n == 0 else round(math.sqrt(sum(j * j * len(run) for j, run in zip(jit, runs) if j is not None) / n), 2)
    return {
        "input_shake_px": tin["shake_px"], "input_shake_deg": tin["shake_deg"], "input_coverage": tin["coverage"],
        "output_shake_px": tout["shake_px"], "output_shake_deg": tout["shake_deg"], "output_jerk_px": tout["jerk_px"], "output_coverage": tout["coverage"],
        "target_jitter_px": target_jitter,
    }


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
    ap.add_argument("--measure", action="store_true", help="measure shake even with --no-render")
    ap.add_argument("--tag", default="", help="suffix for output files (compare variants)")
    ap.add_argument("--bench-args", default="", help="extra arguments passed to the Kotlin bench")
    ap.add_argument("--sheet-every", type=float, default=3.0)
    ap.add_argument("--gt-dir", default=None, help="directory with <name>.gt.csv from gt_tiles.py (default: --out)")
    ap.add_argument("--live", action="store_true", help="zoomed (ROI) neural detection around the track, as in the app")
    ap.add_argument("--model", default=None, help="detector .tflite (default: the app's EfficientDet-Lite0)")
    ap.add_argument("--labels", default="airplane", help="comma-separated target classes of --model")
    ap.add_argument("--score", type=float, default=0.35)
    ap.add_argument("--roi-input", type=int, default=320, help="side of the zoomed detector regions (model input)")
    a = ap.parse_args()
    global MODEL, LABELS, SCORE, ROI_INPUT
    MODEL = a.model or MODEL
    LABELS = a.labels.split(",")
    SCORE = a.score
    ROI_INPUT = a.roi_input
    os.makedirs(a.out, exist_ok=True)
    summary = {}
    for v in a.videos:
        name = os.path.splitext(os.path.basename(v))[0][:40]
        w, h, fps = probe(v)
        ah = round(AW * h / w / 2) * 2
        sx = w / AW
        luma = os.path.join(a.out, name + a.tag + ".gray")
        if not a.live:
            with open(luma, "wb") as f:
                for fr in frames(v, AW, ah, "gray", a.max_seconds, scale=True):
                    f.write(fr.tobytes())
        dets_csv = os.path.join(a.out, name + ".dets.csv")
        if a.reuse and os.path.exists(dets_csv):
            n_det = len(load_csv(dets_csv))
        else:
            n_det = neural_detections(v, AW, ah, fps, sx, a.max_seconds, dets_csv)
        track_csv = os.path.join(a.out, name + a.tag + ".track.csv")
        out_w = 1920 if w >= 2560 else min(1280, w)
        out_h = round(out_w * 9 / 16)
        cmd = [BENCH, "--luma", luma, "--width", str(AW), "--height", str(ah), "--source-width", str(w), "--source-height", str(h),
               "--fps", str(fps), "--dets", dets_csv, "--out", track_csv, "--mode", a.mode,
               "--output-width", str(out_w), "--output-height", str(out_h)]
        if a.zoom is not None:
            cmd += ["--zoom", str(a.zoom)]
        if a.auto_zoom:
            cmd += ["--auto-zoom", "--zoom-min", str(a.zoom_min), "--zoom-max", str(a.zoom_max)]
        cmd += a.bench_args.split()
        roi_csv = os.path.join(a.out, name + a.tag + ".roi.csv")
        if a.live:
            run_live(v, w, h, fps, AW, ah, a.max_seconds, cmd, roi_csv)
        else:
            subprocess.run(cmd, check=True)
            os.remove(luma)
        rows = load_csv(track_csv)
        dets = load_csv(dets_csv)
        if a.live:
            # Only the whole-frame runs the planner actually made count, plus the zoomed ones.
            roi = load_csv(roi_csv)
            roi_frames = {int(r["frame"]) for r in roi if r["kind"] == "roi"}
            dets = [d for d in dets if int(d["frame"]) not in roi_frames] + [
                dict(frame=r["frame"], x0=r["x0"], y0=r["y0"], x1=r["x1"], y1=r["y1"], score=r["score"], roi="1") for r in roi if r["kind"] == "det"]
            m_roi = {"roi_runs": len(roi_frames), "roi_detections": sum(r["kind"] == "det" for r in roi)}
        else:
            roi, m_roi = [], {}
        m = metrics(rows, dets, fps, w)
        gt_csv = os.path.join(a.gt_dir or a.out, name + ".gt.csv")
        if os.path.exists(gt_csv):
            m.update(gt_metrics(rows, load_csv(gt_csv)))
        m.update({"video": os.path.basename(v), "resolution": f"{w}x{h}@{fps:g}", "neural_detections": n_det, **m_roi})
        summary[name] = m
        if not a.no_render or a.measure:
            m.update(render(v, w, h, fps, rows, dets + [dict(r, roi="box") for r in roi if r["kind"] == "roi"], a.max_seconds, os.path.join(a.out, name + a.tag + ".mp4"),
                            os.path.join(a.out, name + a.tag + ".sheet.jpg"), a.sheet_every, write_video=not a.no_render))
        print(json.dumps(m), flush=True)
    with open(os.path.join(a.out, f"summary{a.tag}.json"), "w") as f:
        json.dump(summary, f, indent=2)


if __name__ == "__main__":
    main()
