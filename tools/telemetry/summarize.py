#!/usr/bin/env python3
"""Summarizes an Open Track Camera recording log (Download/OpenTrack/OTC_*.jsonl).

    python3 tools/telemetry/summarize.py OTC_20261001_110059.jsonl [--events]

Prints the header, frame timing (camera / recorded), exposure (time, ISO, luma vs aim),
detections and tracking (states, locks, losses) and the largest jumps of the recorded crop,
each with its time in the video, so a moment seen in the video can be found in the log.
"""
import argparse
import json
import math
import statistics
import sys
from collections import Counter


def load(path):
    recs, bad = [], 0
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                recs.append(json.loads(line))
            except json.JSONDecodeError:
                bad += 1  # a cut-off last line when the app was killed
    return recs, bad


def pct(values, q):
    if not values:
        return float("nan")
    s = sorted(values)
    return s[min(len(s) - 1, int(q * (len(s) - 1) + 0.5))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--events", action="store_true", help="list every state change")
    ap.add_argument("--top", type=int, default=5, help="how many crop jumps to list")
    a = ap.parse_args()

    recs, bad = load(a.log)
    by = {}
    for r in recs:
        by.setdefault(r.get("k"), []).append(r)
    start = (by.get("start") or [{}])[0]
    print("== header")
    for k, v in start.items():
        if k != "k":
            print(f"  {k}: {v}")
    if bad:
        print(f"  ({bad} unreadable lines)")
    end = by.get("end")
    print(f"  dropped lines: {end[0].get('dropped') if end else 'unknown (no end record: app killed?)'}")

    # Video time of a camera timestamp, from the recorded frames.
    outs = by.get("out", [])
    ts0 = outs[0]["ts"] - outs[0]["t"] * 1e9 if outs else None

    def vt(ts):
        return "  -  " if ts0 is None else f"{(ts - ts0) / 1e9:7.2f}s"

    print("\n== frames")
    for kind in ("cam", "live", "out"):
        ts = [r["ts"] for r in by.get(kind, [])]
        if len(ts) < 2:
            print(f"  {kind}: {len(ts)} records")
            continue
        d = [(b - a_) / 1e6 for a_, b in zip(ts, ts[1:])]
        dur = (ts[-1] - ts[0]) / 1e9
        print(f"  {kind}: {len(ts)} frames over {dur:.1f} s = {(len(ts) - 1) / dur:.1f} fps; "
              f"interval median {statistics.median(d):.1f} ms, max {max(d):.1f} ms, gaps >1.5x: {sum(x > 1.5 * statistics.median(d) for x in d)}")

    cams = by.get("cam", [])
    if cams:
        print("\n== exposure")
        exp = [r["exp"] / 1e6 for r in cams if r.get("exp")]
        iso = [r["iso"] for r in cams if r.get("iso")]
        print(f"  time ms: min {min(exp):.3f} median {statistics.median(exp):.3f} max {max(exp):.3f}  (1/{1000 / statistics.median(exp):.0f} s)")
        print(f"  ISO: min {min(iso)} median {statistics.median(iso):.0f} max {max(iso)}")
        fl = Counter(r.get("fl") for r in cams)
        print(f"  focal lengths (mm): {dict(fl)}" + ("  <- lens switched during the recording" if len(fl) > 1 else ""))
        phys = Counter(r.get("phys") for r in cams if r.get("phys"))
        if phys:
            print(f"  active physical lens: {dict(phys)}")
        aes = by.get("ae", [])
        man = [r for r in aes if r.get("mode") == "manual" and r.get("aim")]
        if man:
            err = [math.log2(max(r["luma"], 1) / r["aim"]) for r in man]
            print(f"  shutter priority: {len(man)} updates, luma/aim (stops) median {statistics.median(err):+.2f}, "
                  f"p5 {pct(err, 0.05):+.2f}, p95 {pct(err, 0.95):+.2f}; changes {sum(r.get('changed', False) for r in man)}")
        auto = [r for r in aes if r.get("mode") == "auto"]
        if auto:
            print(f"  auto/learning: {len(auto)} updates, ready at {vt(next((r['ts'] for r in auto if r.get('ready')), auto[-1]['ts']))}")

    trk = by.get("trk", [])
    if trk:
        print("\n== tracking")
        states = Counter(r["state"] for r in trk)
        print("  analysis frames: " + ", ".join(f"{k} {v / len(trk):.0%}" for k, v in states.most_common()))
        ms = [r["ms"] for r in trk if "ms" in r]
        print(f"  analysis ms: median {statistics.median(ms):.1f}, p95 {pct(ms, 0.95):.1f}")
        dets = Counter()
        for r in trk:
            for d in r.get("dets", []):
                dets[(d[6], d[0])] += 1
        print(f"  detections: {dict(dets)}")
        nn = by.get("nn", [])
        if nn:
            print(f"  neural runs: {len(nn)} ({sum(1 for r in nn if 'roi' in r)} zoomed), "
                  f"ms median {statistics.median(r['ms'] for r in nn):.0f}, with results {sum(1 for r in nn if r['n'] > 0)}")
        prev = None
        changes = []
        for r in trk:
            key = (r["state"], r.get("framable"))
            if key != prev:
                changes.append((r["ts"], key))
                prev = key
        print(f"  state changes: {len(changes) - 1}")
        if a.events:
            for ts, (st, fr) in changes:
                print(f"    {vt(ts)} {st}{'' if fr is None else (' framed' if fr else ' tentative')}")

    if len(outs) > 2:
        print("\n== recorded crop")
        jumps = []
        for p, c, n in zip(outs, outs[1:], outs[2:]):
            # Deviation of the centre from the straight line through its neighbours, % of crop width.
            mx = (p["crop"][0] + n["crop"][0]) / 2
            my = (p["crop"][1] + n["crop"][1]) / 2
            jumps.append((math.hypot(c["crop"][0] - mx, c["crop"][1] - my) / c["crop"][2] * 100, c["t"]))
        jumps.sort(reverse=True)
        print(f"  centre jerk (% of crop width): median {statistics.median(j for j, _ in jumps):.2f}, max {jumps[0][0]:.2f}")
        for j, t in jumps[: a.top]:
            print(f"    {t:7.2f}s  {j:.2f}%")
        persp = [r.get("persp", 0) for r in outs]
        print(f"  perspective strength: median {statistics.median(persp):.2f}, below 1: {sum(p < 0.99 for p in persp) / len(persp):.0%}")
        zoom = [1 / (r["crop"][2] / start.get("srcW", r["crop"][2])) for r in outs]
        print(f"  zoom: min {min(zoom):.2f} median {statistics.median(zoom):.2f} max {max(zoom):.2f}")

    for r in by.get("set", []):
        print(f"\n== settings at {r.get('rt')}: " + ", ".join(f"{k}={v}" for k, v in r.items() if k not in ("k", "rt")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
