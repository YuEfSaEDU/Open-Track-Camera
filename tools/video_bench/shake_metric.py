"""Independent (OpenCV based) measurement of perceived shake in a frame sequence.

For every pair of consecutive frames it estimates a similarity transform (translation +
rotation) of the *background* with Shi-Tomasi corners, pyramidal Lucas-Kanade flow and
RANSAC, ignoring an exclusion box (the tracked aircraft). The cumulative camera path is then
split into a smooth part (centered moving average) and a residual; the residual is what a
viewer perceives as shake. Intentional smooth pans are therefore not penalized.

This deliberately uses a different implementation (OpenCV) than the Kotlin stabilizer so the
stabilizer is not graded by its own motion estimate.
"""
import math

import cv2
import numpy as np


class ShakeMeter:
    def __init__(self, fps, px_scale=1.0, smooth_s=0.5):
        self.fps = fps
        self.px_scale = px_scale  # multiply measured px to express them at the reference width
        self.win = max(3, int(round(smooth_s * fps)) | 1)
        self.prev = None
        self.prev_excl = None
        self.steps = []  # (dx, dy, dtheta) or None when unreliable

    def add(self, gray, exclude=None):
        """gray: uint8 HxW; exclude: (x0, y0, x1, y1) in gray pixels or None."""
        if self.prev is not None:
            self.steps.append(self._estimate(self.prev, gray, self.prev_excl))
        self.prev = gray
        self.prev_excl = exclude

    def _estimate(self, a, b, excl):
        mask = np.full(a.shape, 255, np.uint8)
        if excl is not None:
            x0, y0, x1, y1 = (int(round(v)) for v in excl)
            mask[max(0, y0):max(0, y1), max(0, x0):max(0, x1)] = 0
        pts = cv2.goodFeaturesToTrack(a, maxCorners=200, qualityLevel=0.01, minDistance=8, mask=mask, blockSize=7)
        if pts is None or len(pts) < 12:
            return None
        nxt, st, _ = cv2.calcOpticalFlowPyrLK(a, b, pts, None, winSize=(21, 21), maxLevel=4)
        ok = st.reshape(-1) == 1
        if ok.sum() < 12:
            return None
        m, inl = cv2.estimateAffinePartial2D(pts[ok], nxt[ok], method=cv2.RANSAC, ransacReprojThreshold=1.0)
        if m is None or inl is None or inl.sum() < 12:
            return None
        # Express the shift about the frame center: q = c + A (p - c) + d. Otherwise a zoom
        # (scale change about the center, e.g. auto zoom) would leak into the translation.
        c = np.array([a.shape[1] / 2.0, a.shape[0] / 2.0])
        d = m[:, 2] + m[:, :2] @ c - c
        return (d[0], d[1], math.atan2(m[1, 0], m[0, 0]))

    def result(self):
        n = len(self.steps)
        valid = [s is not None for s in self.steps]
        steps = np.array([s if s is not None else (0.0, 0.0, 0.0) for s in self.steps], dtype=float).reshape(-1, 3)
        if n < self.win + 2 or sum(valid) < 0.3 * n:
            return {"coverage": round(sum(valid) / max(1, n), 3), "shake_px": None, "shake_deg": None, "jerk_px": None}
        path = np.cumsum(steps, axis=0)
        k = np.ones(self.win) / self.win
        pad = self.win // 2
        smooth = np.stack([np.convolve(np.pad(path[:, i], pad, mode="edge"), k, mode="valid") for i in range(3)], axis=1)
        resid = path - smooth
        # Only score stretches where both neighbouring estimates are reliable.
        v = np.array(valid)
        good = v.copy()
        good[1:] &= v[:-1]
        trans = np.hypot(resid[:, 0], resid[:, 1])[good] * self.px_scale
        rot = np.degrees(np.abs(resid[:, 2]))[good]
        acc = steps[1:, :2] - steps[:-1, :2]
        acc_ok = v[1:] & v[:-1]
        jerk = np.hypot(acc[:, 0], acc[:, 1])[acc_ok] * self.px_scale
        return {
            "coverage": round(float(v.mean()), 3),
            "shake_px": round(float(np.sqrt(np.mean(trans ** 2))), 2),
            "shake_deg": round(float(np.sqrt(np.mean(rot ** 2))), 3),
            "jerk_px": round(float(np.sqrt(np.mean(jerk ** 2))), 2),
        }


def hf_rms(values, fps, smooth_s=0.5):
    """RMS of the high-frequency residual of a 1D/2D series (rows = samples)."""
    a = np.asarray(values, dtype=float)
    if a.ndim == 1:
        a = a[:, None]
    win = max(3, int(round(smooth_s * fps)) | 1)
    if len(a) < win + 2:
        return None
    pad = win // 2
    k = np.ones(win) / win
    smooth = np.stack([np.convolve(np.pad(a[:, i], pad, mode="edge"), k, mode="valid") for i in range(a.shape[1])], axis=1)
    r = a - smooth
    return float(np.sqrt(np.mean(np.sum(r ** 2, axis=1))))
