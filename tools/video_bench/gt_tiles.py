#!/usr/bin/env python3
"""Weak ground truth for the bench: dense, tiled full-resolution aircraft detection.

The app's detector sees a 320 px input; a distant fighter in a 4K frame is a few pixels there.
This runs the same EfficientDet-Lite0 on overlapping tiles at (close to) native resolution,
a few times per second, so every aircraft big enough to recognize is found independently of
the tracker. Far too slow for the phone; only used to score the tracker.

Output: <out>/<name>.gt.csv with frame,x0,y0,x1,y1,score (source px, merged across tiles).
"""
import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from run_bench import create_detector, frames, probe  # noqa: E402


def tiles(w, h, size, overlap):
    stride = size - overlap
    xs = list(range(0, max(1, w - size) + 1, stride))
    ys = list(range(0, max(1, h - size) + 1, stride))
    if xs[-1] + size < w:
        xs.append(w - size)
    if ys[-1] + size < h:
        ys.append(h - size)
    return [(x, y) for y in ys for x in xs]


def nms(boxes, iou=0.4):
    boxes = sorted(boxes, key=lambda b: -b[4])
    keep = []
    for b in boxes:
        ok = True
        for k in keep:
            ix = max(0.0, min(b[2], k[2]) - max(b[0], k[0]))
            iy = max(0.0, min(b[3], k[3]) - max(b[1], k[1]))
            inter = ix * iy
            union = (b[2] - b[0]) * (b[3] - b[1]) + (k[2] - k[0]) * (k[3] - k[1]) - inter
            # Tiles cut aircraft: also merge a box mostly contained in a kept one.
            small = min((b[2] - b[0]) * (b[3] - b[1]), (k[2] - k[0]) * (k[3] - k[1]))
            if union > 0 and (inter / union > iou or inter > 0.6 * small):
                ok = False
                break
        if ok:
            keep.append(b)
    return keep


def main():
    import mediapipe as mp

    ap = argparse.ArgumentParser()
    ap.add_argument("videos", nargs="+")
    ap.add_argument("--out", default="bench_out")
    ap.add_argument("--hz", type=float, default=5.0)
    ap.add_argument("--tile", type=int, default=640, help="tile side in source px (resized to 320)")
    ap.add_argument("--max-seconds", type=float, default=60)
    a = ap.parse_args()
    det = create_detector()
    for v in a.videos:
        name = os.path.splitext(os.path.basename(v))[0][:40]
        w, h, fps = probe(v)
        tile = min(a.tile, h)
        grid = tiles(w, h, tile, tile // 5)
        step = max(1, round(fps / a.hz))
        out = os.path.join(a.out, name + ".gt.csv")
        n = 0
        with open(out, "w") as f:
            f.write("frame,x0,y0,x1,y1,score\n")
            for i, rgb in enumerate(frames(v, w, h, "rgb24", a.max_seconds)):
                if i % step:
                    continue
                found = []
                for x, y in grid:
                    crop = cv2.resize(rgb[y:y + tile, x:x + tile], (320, 320), interpolation=cv2.INTER_AREA)
                    k = tile / 320
                    for d in det.detect(mp.Image(image_format=mp.ImageFormat.SRGB, data=np.ascontiguousarray(crop))).detections:
                        b = d.bounding_box
                        found.append((x + b.origin_x * k, y + b.origin_y * k, x + (b.origin_x + b.width) * k, y + (b.origin_y + b.height) * k, d.categories[0].score))
                # Whole-frame pass too, for aircraft larger than a tile.
                small = cv2.resize(rgb, (640, round(640 * h / w)), interpolation=cv2.INTER_AREA)
                k = w / 640
                for d in det.detect(mp.Image(image_format=mp.ImageFormat.SRGB, data=np.ascontiguousarray(small))).detections:
                    b = d.bounding_box
                    found.append((b.origin_x * k, b.origin_y * k, (b.origin_x + b.width) * k, (b.origin_y + b.height) * k, d.categories[0].score))
                for b in nms(found):
                    f.write(f"{i},{b[0]:.1f},{b[1]:.1f},{b[2]:.1f},{b[3]:.1f},{b[4]:.3f}\n")
                    n += 1
        print(name, "gt boxes:", n, flush=True)
    det.close()


if __name__ == "__main__":
    main()
