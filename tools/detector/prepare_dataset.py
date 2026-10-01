#!/usr/bin/env python3
"""Builds the training set for Open Track Camera's own aircraft detector.

Classes (ids as in the exported model, 0 = background):
  1 airplane    fixed-wing aircraft: jets, airliners, props, gliders, military transports
  2 helicopter  helicopters (tilt-rotors are left out: they look like both)
  3 drone       multirotor drones and fixed-wing military UAVs (TB2, Akinci, MQ-9...)
  4 bird        the most common false alarm in the sky; the app never locks on it

Sources (each optional, capped with --max-*):
  coco      COCO 2017: airplane, bird                                   (CC BY 4.0 annotations)
  oi        Open Images V7: fixed-wing aircraft, helicopter, birds      (CC BY 4.0 / CC BY 2.0)
  military  Military Aircraft Detection Dataset (Kaggle, mirrored on Hugging Face):
            103 military types + 11 airliners, mapped to the classes above
  drones    Seraphim Drone Detection Dataset (Hugging Face, CC BY 4.0)
  synth     small objects: objects photographed against plain sky, cut out with a soft mask,
            shrunk to 6-50 px and blended into object-free backgrounds (sky, clouds, ground)
            cut from the same images. The app
            looks for aircraft that are often only tens of pixels in its 320-384 px detector
            input; the photo datasets are mostly frame-filling aircraft.

Output (MediaPipe Model Maker's COCO folder layout):
  OUT/train/images/*.jpg, OUT/train/labels.json, OUT/val/..., OUT/stats.json

Usage (Colab runs this; a quick local check uses the "tiny" preset):
  python prepare_dataset.py --out /content/dataset --preset medium
"""
import argparse
import csv
import hashlib
import io
import json
import os
import random
import sys
import time
import urllib.request
import zipfile
from collections import Counter, defaultdict
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np

CLASSES = ["airplane", "helicopter", "drone", "bird"]
CLS = {name: k + 1 for k, name in enumerate(CLASSES)}

# Images are stored at most this large (long side): enough detail for crops and small objects,
# much less disk and decode time than the originals (many are 4-6k px).
MAX_SIDE = 1024

PRESETS = {
    # name: per-source caps (images) and synthetic images
    "tiny": dict(coco=12, coco_bird=6, oi_air=8, oi_heli=8, oi_bird=4, military=16, drones=12, synth=16),
    "small": dict(coco=1500, coco_bird=800, oi_air=1500, oi_heli=1500, oi_bird=800, military=4000, drones=3000, synth=3000),
    "medium": dict(coco=3000, coco_bird=1500, oi_air=4000, oi_heli=4000, oi_bird=2000, military=9000, drones=6000, synth=7000),
    "large": dict(coco=3500, coco_bird=3000, oi_air=12000, oi_heli=8000, oi_bird=4000, military=20000, drones=12000, synth=15000),
}

HF = "https://huggingface.co/datasets"
MILITARY = f"{HF}/a2015003713/military-aircraft-detection-dataset/resolve/main"
SERAPHIM = f"{HF}/lgrzybowski/seraphim-drone-detection-dataset/resolve/main"
OI_CSV = {
    "train": "https://storage.googleapis.com/openimages/v6/oidv6-train-annotations-bbox.csv",
    "validation": "https://storage.googleapis.com/openimages/v5/validation-annotations-bbox.csv",
}
OI_HIERARCHY = "https://storage.googleapis.com/openimages/2018_04/bbox_labels_600_hierarchy.json"
OI_IMAGE = "https://open-images-dataset.s3.amazonaws.com/{split}/{id}.jpg"
COCO_ANN = "http://images.cocodataset.org/annotations/annotations_trainval2017.zip"

MILITARY_HELICOPTERS = {"AH64", "CH47", "CH53", "Ka27", "Ka52", "Mi24", "Mi26", "Mi28", "Mi8", "UH60", "WZ10", "Z10", "Z19", "Z21", "NH90"}
MILITARY_DRONES = {"MQ9", "RQ4", "TB2", "TB001", "AKINCI", "WZ7", "WZ9", "MQ20", "MQ25", "MQ28", "MQ35", "XQ58", "KIZILELMA"}
MILITARY_SKIP = {"V22", "V280"}  # tilt-rotors


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def fetch(url, retries=4, timeout=60):
    for k in range(retries):
        try:
            req = urllib.request.Request(url, headers={"User-Agent": "open-track-camera-dataset"})
            with urllib.request.urlopen(req, timeout=timeout) as r:
                return r.read()
        except Exception as e:  # noqa: BLE001 - network: retry, then give up on this file
            if k == retries - 1:
                print(f"  failed {url}: {e}", file=sys.stderr)
                return None
            time.sleep(2 ** k)


def fetch_to(url, path):
    if os.path.exists(path) and os.path.getsize(path) > 0:
        return path
    os.makedirs(os.path.dirname(path), exist_ok=True)
    log("download", url)
    tmp = path + ".part"
    req = urllib.request.Request(url, headers={"User-Agent": "open-track-camera-dataset"})
    with urllib.request.urlopen(req, timeout=120) as r, open(tmp, "wb") as f:
        while True:
            chunk = r.read(1 << 22)
            if not chunk:
                break
            f.write(chunk)
    os.replace(tmp, path)
    return path


def is_val(key, fraction):
    """Deterministic split by image key."""
    return int(hashlib.md5(key.encode()).hexdigest()[:8], 16) / 0xFFFFFFFF < fraction


class Writer:
    """Collects images and boxes; writes Model Maker COCO folders."""

    def __init__(self, out, val_fraction):
        self.out = out
        self.val_fraction = val_fraction
        self.items = {"train": [], "val": []}
        for s in self.items:
            os.makedirs(os.path.join(out, s, "images"), exist_ok=True)

    def add(self, key, img_bytes_or_array, boxes, source, split=None):
        """boxes: [(class name, x, y, w, h)] in pixels of the given image. Returns the stored path or None."""
        img = img_bytes_or_array
        if isinstance(img, (bytes, bytearray)):
            img = cv2.imdecode(np.frombuffer(img, np.uint8), cv2.IMREAD_COLOR)
        if img is None or img.ndim != 3:
            return None
        h, w = img.shape[:2]
        s = min(1.0, MAX_SIDE / max(h, w))
        if s < 1.0:
            img = cv2.resize(img, (round(w * s), round(h * s)), interpolation=cv2.INTER_AREA)
        h2, w2 = img.shape[:2]
        out_boxes = []
        for name, x, y, bw, bh in boxes:
            x, y, bw, bh = x * s, y * s, bw * s, bh * s
            x0, y0 = max(0.0, x), max(0.0, y)
            x1, y1 = min(w2, x + bw), min(h2, y + bh)
            if x1 - x0 < 2 or y1 - y0 < 2:
                continue
            out_boxes.append((name, x0, y0, x1 - x0, y1 - y0))
        if boxes and not out_boxes:
            return None
        split = split or ("val" if is_val(key, self.val_fraction) else "train")
        fname = f"{source}_{hashlib.md5(key.encode()).hexdigest()[:16]}.jpg"
        path = os.path.join(self.out, split, "images", fname)
        cv2.imwrite(path, img, [cv2.IMWRITE_JPEG_QUALITY, 92])
        self.items[split].append(dict(file=fname, w=w2, h=h2, boxes=out_boxes, source=source))
        return path

    def finish(self):
        stats = {}
        for split, items in self.items.items():
            images, anns = [], []
            for i, it in enumerate(items):
                images.append(dict(id=i, file_name=it["file"], width=it["w"], height=it["h"]))
                for name, x, y, w, h in it["boxes"]:
                    anns.append(dict(id=len(anns), image_id=i, category_id=CLS[name], bbox=[round(x, 1), round(y, 1), round(w, 1), round(h, 1)],
                                     area=round(w * h, 1), iscrowd=0))
            cats = [dict(id=0, name="background")] + [dict(id=CLS[n], name=n) for n in CLASSES]
            with open(os.path.join(self.out, split, "labels.json"), "w") as f:
                json.dump(dict(images=images, annotations=anns, categories=cats), f)
            per_class = Counter(CLASSES[a["category_id"] - 1] for a in anns)
            per_source = Counter(it["source"] for it in items)
            # Object size (long side) relative to the image's long side: what the detector sees at its input size.
            sizes = Counter()
            for it in items:
                for _, _, _, w, h in it["boxes"]:
                    r = max(w, h) / max(it["w"], it["h"]) * 384
                    sizes["<16px" if r < 16 else "16-32px" if r < 32 else "32-96px" if r < 96 else ">=96px"] += 1
            stats[split] = dict(images=len(items), objects=len(anns), per_class=per_class, per_source=per_source, size_at_384=sizes)
            log(split, json.dumps(stats[split]))
        with open(os.path.join(self.out, "stats.json"), "w") as f:
            json.dump(stats, f, indent=1)
        return stats


def parallel(fn, jobs, workers=32):
    done = 0
    with ThreadPoolExecutor(workers) as ex:
        for r in ex.map(fn, jobs):
            done += 1
            if done % 500 == 0:
                log(f"  {done}/{len(jobs)}")
            yield r


# ---------------------------------------------------------------- COCO

def source_coco(wr, cache, n_air, n_bird, rng):
    z = fetch_to(COCO_ANN, os.path.join(cache, "coco_ann.zip"))
    jobs = []
    with zipfile.ZipFile(z) as zf:
        for split_name in ("train2017", "val2017"):
            d = json.load(zf.open(f"annotations/instances_{split_name}.json"))
            cats = {c["id"]: c["name"] for c in d["categories"]}
            by_img = defaultdict(list)
            for a in d["annotations"]:
                if cats[a["category_id"]] in ("airplane", "bird"):
                    by_img[a["image_id"]].append(a)
            imgs = {i["id"]: i for i in d["images"]}
            air, bird = [], []
            for iid, anns in by_img.items():
                if any(a["iscrowd"] for a in anns):
                    continue
                (air if any(cats[a["category_id"]] == "airplane" for a in anns) else bird).append(iid)
            rng.shuffle(air)
            rng.shuffle(bird)
            # COCO's own val split stays validation (a small share of the caps).
            k = 0.1 if split_name == "val2017" else 0.9
            for iid in air[: int(n_air * k)] + bird[: int(n_bird * k)]:
                im = imgs[iid]
                boxes = [(cats[a["category_id"]], *a["bbox"]) for a in by_img[iid]]
                jobs.append((im["coco_url"], f"coco{iid}", boxes, "val" if split_name == "val2017" else "train"))
    log(f"coco: {len(jobs)} images")

    def run(j):
        url, key, boxes, split = j
        b = fetch(url)
        return b is not None and wr.add(key, b, boxes, "coco", split) is not None

    return sum(parallel(run, jobs))


# ---------------------------------------------------------------- Open Images

def oi_descendants(h, label):
    def find(n):
        if n.get("LabelName") == label:
            return n
        for c in n.get("Subcategory", []):
            r = find(c)
            if r:
                return r
        return None

    def walk(n):
        out = [n["LabelName"]]
        for c in n.get("Subcategory", []):
            out += walk(c)
        return out

    n = find(h)
    return set(walk(n)) if n else {label}


def source_oi(wr, cache, n_air, n_heli, n_bird, rng, splits=("train", "validation")):
    hier = json.loads(fetch(OI_HIERARCHY))
    mapping = {"/m/0cmf2": "airplane", "/m/09ct_": "helicopter"}
    for l in oi_descendants(hier, "/m/015p6"):
        mapping[l] = "bird"
    aircraft_other = oi_descendants(hier, "/m/0k5j") - {"/m/0cmf2", "/m/09ct_"}  # "Aircraft", rockets
    jobs = []
    for split in splits:
        path = fetch_to(OI_CSV[split], os.path.join(cache, f"oi_{split}.csv"))
        boxes = defaultdict(list)
        bad = set()
        with open(path, newline="") as f:
            for r in csv.DictReader(f):
                lab = r["LabelName"]
                if lab in aircraft_other:
                    bad.add(r["ImageID"])  # a generic "Aircraft" box: unsure what it is
                    continue
                name = mapping.get(lab)
                if name is None:
                    continue
                if r["IsDepiction"] == "1" or r["IsGroupOf"] == "1":
                    bad.add(r["ImageID"])  # drawings, toys, boxes around groups
                    continue
                boxes[r["ImageID"]].append((name, float(r["XMin"]), float(r["YMin"]), float(r["XMax"]), float(r["YMax"])))
        ids = [i for i in boxes if i not in bad]
        rng.shuffle(ids)
        caps = {"airplane": n_air, "helicopter": n_heli, "bird": n_bird}
        if split == "validation":
            caps = {k: max(1, v // 10) for k, v in caps.items()}
        taken = Counter()
        for iid in ids:
            names = {b[0] for b in boxes[iid]}
            main = "helicopter" if "helicopter" in names else "airplane" if "airplane" in names else "bird"
            if taken[main] >= caps[main]:
                continue
            taken[main] += 1
            jobs.append((split, iid, boxes[iid]))
        log(f"oi {split}: {dict(taken)}")

    def run(j):
        split, iid, bx = j
        b = fetch(OI_IMAGE.format(split=split, id=iid))
        if b is None:
            return False
        img = cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return False
        h, w = img.shape[:2]
        pix = [(n, x0 * w, y0 * h, (x1 - x0) * w, (y1 - y0) * h) for n, x0, y0, x1, y1 in bx]
        return wr.add(f"oi{iid}", img, pix, "oi", "val" if split == "validation" else None) is not None

    return sum(parallel(run, jobs))


# ---------------------------------------------------------------- Military aircraft

def source_military(wr, cache, n, rng):
    jobs = []
    for split in ("train", "val", "test"):
        d = json.loads(fetch(f"{MILITARY}/annotations/coco/{split}.json"))
        cats = {c["id"]: c["name"] for c in d["categories"]}
        by_img = defaultdict(list)
        for a in d["annotations"]:
            by_img[a["image_id"]].append(a)
        imgs = [i for i in d["images"] if i["id"] in by_img]
        rng.shuffle(imgs)
        cap = int(n * {"train": 0.8, "val": 0.15, "test": 0.05}[split])
        for im in imgs[:cap]:
            names = [cats[a["category_id"]] for a in by_img[im["id"]]]
            if any(t in MILITARY_SKIP for t in names):
                continue
            boxes = []
            for a, t in zip(by_img[im["id"]], names):
                c = "helicopter" if t in MILITARY_HELICOPTERS else "drone" if t in MILITARY_DRONES else "airplane"
                boxes.append((c, *a["bbox"]))
            jobs.append((im["file_name"], boxes, "train" if split == "train" else "val"))
    log(f"military: {len(jobs)} images")

    def run(j):
        name, boxes, split = j
        b = fetch(f"{MILITARY}/images/{name[0]}/{name}")
        return b is not None and wr.add(f"mil{name}", b, boxes, "military", split) is not None

    return sum(parallel(run, jobs, workers=16))


# ---------------------------------------------------------------- Drones

def source_drones(wr, cache, n, rng, batch="batch_003"):
    labels = zipfile.ZipFile(fetch_to(f"{SERAPHIM}/train/labels/batch_001.zip", os.path.join(cache, "seraphim_labels.zip")))
    label_by_stem = {}
    for name in labels.namelist():
        if name.endswith(".txt"):
            label_by_stem[os.path.splitext(os.path.basename(name))[0]] = name
    if n <= 50:
        # Quick checks: read a few images straight out of the remote zip.
        from remotezip import RemoteZip
        zf = RemoteZip(f"{SERAPHIM}/train/images/{batch}.zip")
    else:
        zf = zipfile.ZipFile(fetch_to(f"{SERAPHIM}/train/images/{batch}.zip", os.path.join(cache, f"seraphim_{batch}.zip")))
    names = [m for m in zf.namelist() if m.lower().endswith((".jpg", ".jpeg", ".png"))]
    rng.shuffle(names)
    count = 0
    for m in names:
        if count >= n:
            break
        stem = os.path.splitext(os.path.basename(m))[0]
        lab = label_by_stem.get(stem)
        if lab is None:
            continue
        img = cv2.imdecode(np.frombuffer(zf.read(m), np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            continue
        h, w = img.shape[:2]
        boxes = []
        for line in labels.read(lab).decode().split("\n"):
            p = line.split()
            if len(p) != 5:
                continue
            cx, cy, bw, bh = (float(v) for v in p[1:])
            boxes.append(("drone", (cx - bw / 2) * w, (cy - bh / 2) * h, bw * w, bh * h))
        if boxes and wr.add(f"drone{stem}", img, boxes, "drones") is not None:
            count += 1
    log(f"drones: {count} images")
    return count


# ---------------------------------------------------------------- Small objects

def crops_and_backgrounds(wr, rng, max_crops=6000, max_bgs=4000):
    """Object crops (with a margin) and object-free background patches from the stored train images."""
    crops, bgs = [], []
    items = [it for it in wr.items["train"] if it["source"] != "synth"]
    rng.shuffle(items)
    for it in items:
        if len(crops) >= max_crops and len(bgs) >= max_bgs:
            break
        img = cv2.imread(os.path.join(wr.out, "train", "images", it["file"]))
        if img is None:
            continue
        h, w = img.shape[:2]
        for name, x, y, bw, bh in it["boxes"]:
            if len(crops) < max_crops and max(bw, bh) >= 48 and min(bw, bh) >= 12:
                m = 0.12
                x0, y0 = int(max(0, x - bw * m)), int(max(0, y - bh * m))
                x1, y1 = int(min(w, x + bw * (1 + m))), int(min(h, y + bh * (1 + m)))
                crop = img[y0:y1, x0:x1].copy()
                mask = object_mask(crop)
                if mask is not None:
                    crops.append((name, crop, mask, (x - x0, y - y0, bw, bh)))
        # Drone images are letterboxed (flat padding): no backgrounds from them.
        if len(bgs) < max_bgs and it["source"] != "drones":
            for _ in range(3):
                s = int(rng.uniform(0.25, 0.6) * min(h, w))
                if s < 96:
                    break
                bx, by = rng.randint(0, w - s), rng.randint(0, h - s)
                if all(bx + s < x or bx > x + bw or by + s < y or by > y + bh for _, x, y, bw, bh in it["boxes"]):
                    bgs.append(img[by:by + s, bx:bx + s].copy())
                    break
    return crops, bgs


def object_mask(crop):
    """
    Soft mask of the object in a crop whose surroundings are plain (sky): pixels that differ
    from the crop's border colour. None when the border is not plain (ground, branches, crowds:
    pasting those would show a rectangle) or the object hardly differs from it.
    """
    h, w = crop.shape[:2]
    b = max(2, min(h, w) // 20)
    ring = np.concatenate([crop[:b].reshape(-1, 3), crop[-b:].reshape(-1, 3), crop[:, :b].reshape(-1, 3), crop[:, -b:].reshape(-1, 3)]).astype(np.float32)
    if ring.std(0).max() > 14:
        return None
    d = np.abs(crop.astype(np.float32) - ring.mean(0)).max(2)
    m = np.clip((d - 8) / 22, 0, 1)
    k = max(3, (min(h, w) // 40) | 1)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
    m = cv2.GaussianBlur(m, (k, k), 0)
    if (m > 0.5).mean() < 0.04:
        return None
    return m


def paste(canvas, crop, mask, inner, size, x, y):
    """Blends the object of [crop] scaled so its long side is [size] px, object top-left at (x, y). Returns the box."""
    ox, oy, ow, oh = inner
    s = size / max(ow, oh)
    ch, cw = crop.shape[:2]
    tw, th = max(2, round(cw * s)), max(2, round(ch * s))
    small = cv2.resize(crop, (tw, th), interpolation=cv2.INTER_AREA).astype(np.float32)
    a = cv2.resize(mask, (tw, th), interpolation=cv2.INTER_AREA)[..., None]
    H, W = canvas.shape[:2]
    px, py = int(x - ox * s), int(y - oy * s)
    if px < 0 or py < 0 or px + tw > W or py + th > H:
        return None
    region = canvas[py:py + th, px:px + tw].astype(np.float32)
    # Shift the object's colours by the difference between the canvas and its own sky, so it
    # looks lit like the new background (a dark jet stays darker than the sky it is put in).
    border = np.concatenate([small[0], small[-1], small[:, 0], small[:, -1]])
    rb = np.concatenate([region[0], region[-1], region[:, 0], region[:, -1]])
    small += 0.6 * (rb.mean(0) - border.mean(0))
    canvas[py:py + th, px:px + tw] = np.clip(region * (1 - a) + small * a, 0, 255).astype(np.uint8)
    return (px + ox * s, py + oy * s, ow * s, oh * s)


def source_synth(wr, n, rng):
    crops, bgs = crops_and_backgrounds(wr, rng)
    if not crops or not bgs:
        log("synth: no crops/backgrounds")
        return 0
    by_class = defaultdict(list)
    for c in crops:
        by_class[c[0]].append(c)
    log("synth crops:", {k: len(v) for k, v in by_class.items()}, "backgrounds:", len(bgs))
    count = 0
    for i in range(n):
        side = 640
        bg = bgs[rng.randrange(len(bgs))]
        canvas = cv2.resize(bg, (side, side), interpolation=cv2.INTER_LINEAR)
        if rng.random() < 0.5:
            canvas = cv2.flip(canvas, 1)
        boxes = []
        # ~8% negatives: backgrounds alone (clouds, contrails, terrain).
        k = 0 if rng.random() < 0.08 else rng.choice([1, 1, 1, 2, 2, 3, 4])
        for _ in range(k):
            cls = rng.choice(["airplane", "airplane", "helicopter", "drone", "bird"])
            pool = by_class.get(cls) or crops
            name, crop, mask, inner = pool[rng.randrange(len(pool))]
            # Long side 10-80 px on the 640 canvas: ~6-48 px at the 384 px model input.
            size = int(np.exp(rng.uniform(np.log(10), np.log(80))))
            if rng.random() < 0.5:
                crop, mask = cv2.flip(crop, 1), cv2.flip(mask, 1)
                inner = (crop.shape[1] - inner[0] - inner[2], inner[1], inner[2], inner[3])
            for _try in range(10):
                x, y = rng.uniform(4, side - size - 4), rng.uniform(4, side - size - 4)
                if all(x > bx + bw + 4 or x + size < bx - 4 or y > by + bh + 4 or y + size < by - 4 for _, bx, by, bw, bh in boxes):
                    b = paste(canvas, crop, mask, inner, size, x, y)
                    if b:
                        boxes.append((name, *b))
                    break
        if rng.random() < 0.3:
            # Sensor noise / mild blur, like a zoomed phone video frame.
            canvas = cv2.GaussianBlur(canvas, (3, 3), rng.uniform(0.3, 0.9))
            noise = np.random.default_rng(i).normal(0, rng.uniform(1, 4), canvas.shape)
            canvas = np.clip(canvas + noise, 0, 255).astype(np.uint8)
        if k == 0 or boxes:
            if wr.add(f"synth{i}", canvas, boxes, "synth") is not None:
                count += 1
    log(f"synth: {count} images")
    return count


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--cache", default=None, help="downloads (annotation files, zips); default OUT/_cache")
    ap.add_argument("--preset", default="medium", choices=PRESETS)
    ap.add_argument("--sources", default="coco,oi,military,drones,synth")
    ap.add_argument("--val-fraction", type=float, default=0.06)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--oi-validation-only", action="store_true", help="use only Open Images' small validation CSV (quick checks)")
    a = ap.parse_args()
    caps = PRESETS[a.preset]
    cache = a.cache or os.path.join(a.out, "_cache")
    os.makedirs(cache, exist_ok=True)
    rng = random.Random(a.seed)
    wr = Writer(a.out, a.val_fraction)
    src = set(a.sources.split(","))
    if "coco" in src:
        source_coco(wr, cache, caps["coco"], caps["coco_bird"], rng)
    if "oi" in src:
        source_oi(wr, cache, caps["oi_air"], caps["oi_heli"], caps["oi_bird"], rng,
                  splits=("validation",) if a.oi_validation_only else ("train", "validation"))
    if "military" in src:
        source_military(wr, cache, caps["military"], rng)
    if "drones" in src:
        source_drones(wr, cache, caps["drones"], rng)
    if "synth" in src:
        source_synth(wr, caps["synth"], rng)
    wr.finish()


if __name__ == "__main__":
    main()
