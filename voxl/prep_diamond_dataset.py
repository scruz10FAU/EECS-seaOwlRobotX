#!/usr/bin/env python3
"""Square-crop, downscale and greyscale a RoboFlow COCO keypoint export.

Each image is cropped to the largest 1:1 square that keeps every annotation
(bbox + visible keypoints) in frame, then resized to SIZE x SIZE and converted
to single-channel greyscale. If the annotations span more than the image's
short side, the whole image is letterboxed to a square instead.

The crop offset is chosen at random within the feasible range (seeded from the
file name, so runs are reproducible) rather than centred on the annotations, so
the model doesn't learn that beacons always sit mid-frame.

Usage:
    python3 prep_diamond_dataset.py "~/Downloads/Beacon Keypoints.v3i.coco.zip"
    python3 prep_diamond_dataset.py <zip-or-dir> -o <out_dir> --size 640 --margin 8
"""

import argparse
import copy
import hashlib
import json
import random
import sys
import zipfile
from pathlib import Path

import cv2
import numpy as np

LETTERBOX_FILL = 114  # ultralytics' letterbox pad value


class Source:
    """Read files from either a RoboFlow zip or an extracted directory."""

    def __init__(self, path):
        self.path = Path(path).expanduser()
        self.zip = zipfile.ZipFile(self.path) if self.path.is_file() else None

    def names(self):
        if self.zip:
            return self.zip.namelist()
        return [p.relative_to(self.path).as_posix() for p in self.path.rglob("*") if p.is_file()]

    def read(self, name):
        if self.zip:
            return self.zip.read(name)
        return (self.path / name).read_bytes()


def annotation_extent(anns):
    """(xmin, ymin, xmax, ymax) over all bboxes and visible keypoints, or None."""
    xs, ys = [], []
    for a in anns:
        x, y, w, h = a["bbox"]
        xs += [x, x + w]
        ys += [y, y + h]
        kp = a.get("keypoints", [])
        for i in range(0, len(kp), 3):
            if kp[i + 2] > 0:
                xs.append(kp[i])
                ys.append(kp[i + 1])
    if not xs:
        return None
    return min(xs), min(ys), max(xs), max(ys)


def choose_transform(w, h, extent, margin_src, rng):
    """Return (mode, x0, y0, side, pad_x, pad_y).

    Crop mode: take the square [x0, x0+side) x [y0, y0+side) from the source.
    Letterbox mode: pad the full image into a side x side canvas at (pad_x, pad_y).
    """
    side = min(w, h)
    long_len = max(w, h)
    if extent is None:
        lo, hi = 0, long_len - side
    else:
        xmin, ymin, xmax, ymax = extent
        amin, amax = (xmin, xmax) if w >= h else (ymin, ymax)
        # Offset o must satisfy o <= amin - margin and o + side >= amax + margin,
        # and stay within the image.
        lo = max(0.0, amax + margin_src - side)
        hi = min(float(long_len - side), amin - margin_src)
        if lo > hi:
            # Try again without the margin before giving up on cropping.
            lo = max(0.0, amax - side)
            hi = min(float(long_len - side), amin)
    if lo <= hi:
        off = int(round(rng.uniform(lo, hi)))
        off = min(max(off, int(np.ceil(lo))), int(np.floor(hi)))
        x0, y0 = (off, 0) if w >= h else (0, off)
        return "crop", x0, y0, side, 0, 0
    side = long_len
    return "letterbox", 0, 0, side, (side - w) // 2, (side - h) // 2


def transform_ann(a, shift_x, shift_y, scale, size):
    """Map one COCO annotation through x' = (x + shift_x) * scale."""
    a = copy.deepcopy(a)
    x, y, w, h = a["bbox"]
    x1 = np.clip((x + shift_x) * scale, 0, size)
    y1 = np.clip((y + shift_y) * scale, 0, size)
    x2 = np.clip((x + w + shift_x) * scale, 0, size)
    y2 = np.clip((y + h + shift_y) * scale, 0, size)
    a["bbox"] = [round(float(x1), 3), round(float(y1), 3),
                 round(float(x2 - x1), 3), round(float(y2 - y1), 3)]
    a["area"] = round(float(a.get("area", w * h)) * scale * scale, 3)
    kp = a.get("keypoints", [])
    for i in range(0, len(kp), 3):
        if kp[i + 2] > 0:  # leave invisible placeholders (0, 0, 0) untouched
            kp[i] = round(float((kp[i] + shift_x) * scale), 3)
            kp[i + 1] = round(float((kp[i + 1] + shift_y) * scale), 3)
    return a


def process_split(src, json_name, out_root, size, margin_out, quality):
    split_dir = json_name.rsplit("/", 1)[0] if "/" in json_name else ""
    coco = json.loads(src.read(json_name))
    anns_by_img = {}
    for a in coco["annotations"]:
        anns_by_img.setdefault(a["image_id"], []).append(a)

    out_dir = out_root / split_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    new_anns, stats = [], {"crop": 0, "letterbox": 0}

    for img in coco["images"]:
        rel = f"{split_dir}/{img['file_name']}" if split_dir else img["file_name"]
        buf = np.frombuffer(src.read(rel), np.uint8)
        im = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
        if im is None:
            sys.exit(f"cannot decode {rel}")
        h, w = im.shape[:2]
        if (w, h) != (img["width"], img["height"]):
            print(f"  warning: {rel} is {w}x{h}, JSON says {img['width']}x{img['height']}")

        anns = anns_by_img.get(img["id"], [])
        side_guess = min(w, h)
        margin_src = margin_out * side_guess / size
        rng = random.Random(int(hashlib.md5(img["file_name"].encode()).hexdigest(), 16))
        mode, x0, y0, side, pad_x, pad_y = choose_transform(
            w, h, annotation_extent(anns), margin_src, rng)
        stats[mode] += 1

        if mode == "crop":
            square = im[y0:y0 + side, x0:x0 + side]
            shift_x, shift_y = -x0, -y0
        else:
            square = np.full((side, side), LETTERBOX_FILL, np.uint8)
            square[pad_y:pad_y + h, pad_x:pad_x + w] = im
            shift_x, shift_y = pad_x, pad_y
        scale = size / side
        out = cv2.resize(square, (size, size), interpolation=cv2.INTER_AREA)
        cv2.imwrite(str(out_dir / img["file_name"]), out,
                    [cv2.IMWRITE_JPEG_QUALITY, quality])

        img["width"], img["height"] = size, size
        img.setdefault("extra", {})["prep"] = {
            "mode": mode, "src_wh": [w, h], "x0": x0, "y0": y0,
            "pad": [pad_x, pad_y], "side": side, "scale": scale}
        new_anns += [transform_ann(a, shift_x, shift_y, scale, size) for a in anns]

    coco["annotations"] = new_anns
    (out_dir / Path(json_name).name).write_text(json.dumps(coco))
    print(f"  {split_dir or '.'}: {len(coco['images'])} images, {len(new_anns)} annotations, "
          f"{stats['crop']} cropped, {stats['letterbox']} letterboxed")
    return stats


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("source", help="RoboFlow COCO export (.zip or extracted directory)")
    ap.add_argument("-o", "--out", help="output directory (default: <source>.sq<size>gray)")
    ap.add_argument("--size", type=int, default=640, help="output side length (px)")
    ap.add_argument("--margin", type=float, default=8.0,
                    help="min distance from annotations to the crop edge, in output px")
    ap.add_argument("--quality", type=int, default=95, help="JPEG quality")
    args = ap.parse_args()

    src = Source(args.source)
    stem = src.path.name[:-4] if src.path.suffix == ".zip" else src.path.name
    out_root = Path(args.out).expanduser() if args.out else \
        src.path.with_name(f"{stem}.sq{args.size}gray")
    if out_root.exists() and any(out_root.iterdir()):
        sys.exit(f"refusing to write into non-empty {out_root}")

    jsons = sorted(n for n in src.names() if n.endswith("_annotations.coco.json"))
    if not jsons:
        sys.exit("no *_annotations.coco.json found")
    print(f"{src.path} -> {out_root}")
    for j in jsons:
        process_split(src, j, out_root, args.size, args.margin, args.quality)


if __name__ == "__main__":
    main()
