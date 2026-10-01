#!/usr/bin/env python3
"""Test frames for upscale.mjs: luma of 4K video frames, the same frames shrunk by 1.5 / 2 / 3
(a coarser "sensor"), truth crops of the most detailed 1600x900 region, and area-filtered truth
for the shrinking case.

  python3 tools/shader_check/upscale_frames.py DIR VIDEO:FRAME [VIDEO:FRAME ...]
"""
import json
import os
import sys

import cv2
import numpy as np


def main():
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    meta = []
    for k, spec in enumerate(sys.argv[2:]):
        path, idx = spec.rsplit(":", 1)
        c = cv2.VideoCapture(path)
        c.set(cv2.CAP_PROP_POS_FRAMES, int(idx))
        ok, f = c.read()
        if not ok:
            sys.exit(f"cannot read {spec}")
        y = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        if y.shape != (2160, 3840):
            sys.exit(f"{path}: need 3840x2160, got {y.shape[1]}x{y.shape[0]}")
        name = f"f{k}"
        g = np.abs(cv2.Sobel(y.astype(np.float32), cv2.CV_32F, 1, 0)) + np.abs(cv2.Sobel(y.astype(np.float32), cv2.CV_32F, 0, 1))
        best = max(((g[cy - 450:cy + 450, cx - 800:cx + 800].mean(), cx, cy)
                    for cy in range(600, 2160 - 600, 60) for cx in range(1050, 3840 - 1050, 60)))
        _, cx, cy = best
        y.tofile(f"{out}/{name}_T.raw")
        for u in (1.5, 2, 3):
            cv2.resize(y, (round(3840 / u), round(2160 / u)), interpolation=cv2.INTER_AREA).tofile(f"{out}/{name}_S{u}.raw")
        y[cy - 450:cy + 450, cx - 800:cx + 800].copy().tofile(f"{out}/{name}_TC.raw")
        dcx, dcy = min(max(cx, 1300), 3840 - 1300), min(max(cy, 740), 2160 - 740)
        cv2.resize(y[dcy - 720:dcy + 720, dcx - 1280:dcx + 1280], (1600, 900), interpolation=cv2.INTER_AREA).tofile(f"{out}/{name}_D.raw")
        meta.append(dict(name=name, source=spec, cx=cx, cy=cy, dcx=dcx, dcy=dcy))
    with open(f"{out}/meta.json", "w") as f:
        json.dump(meta, f, indent=1)
    print(json.dumps(meta, indent=1))


if __name__ == "__main__":
    main()
