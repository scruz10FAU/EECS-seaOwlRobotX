#!/usr/bin/env python3
"""
build_dataset.py -- reviewed beacon boxes -> YOLO-pose training set for the
2-class beacon/diamond model (models/YOLOv11n_diamond_detect.pt, Nov 8).

Input: a folder with images/ and labels/ in YOLO format, after review
(prelabel_beacons.py output, corrected in CVAT / labelImg / Roboflow):
    "0 cx cy w h"                         beacon box (class 0), no keypoints
    "c cx cy w h  x1 y1 v1 ... x4 y4 v4"  a full pose line (e.g. a diamond,
                                          class 1, corners bottom/left/top/right)
                                          is passed through unchanged
Every image in images/ is used; a missing or empty label file = a frame with
no beacons (a useful negative). Label EVERY visible beacon.

Output (new or empty folder):
    images/{train,val}/*.jpg   frames STRETCHED to --size x --size, as
                               beacon_overlay feeds the model (--resize stretch);
                               normalised YOLO coordinates are unchanged by it
    labels/{train,val}/*.txt   pose lines: beacon boxes get 4 invisible
                               keypoints (0 0 0) -- the model's kpt_shape is [4, 3]
    data.yaml                  kpt_shape [4, 3], names {0: beacon, 1: diamond},
                               flip_idx [0, 3, 2, 1] (without it Ultralytics turns
                               horizontal flips off for pose)

Split: by TIME, not at random -- neighbouring frames are near-identical, so a
random split would leak them into val. Frames whose name (timestamp) is in
[--val-start, --val-end] go to val; without those, the last --val-frac of the
frames in name order.

    python3 training/build_dataset.py <reviewed_dir> -o <dataset_dir>
    python3 training/build_dataset.py <reviewed_dir> -o <dataset_dir> \
        --val-start 20261111_015030 --val-end 20261111_015100
"""

import argparse
import sys
from pathlib import Path

import cv2

NUM_KPTS = 4
NAMES = {0: "beacon", 1: "diamond"}
FLIP_IDX = [0, 3, 2, 1]   # bottom, left, top, right: a horizontal flip swaps left/right


def convert(line: str, lineno: int, path: Path) -> str:
    v = line.split()
    if len(v) == 5:                                   # box only -> invisible keypoints
        return " ".join(v + ["0"] * (3 * NUM_KPTS))
    if len(v) == 5 + 3 * NUM_KPTS:                    # already a pose line
        return " ".join(v)
    sys.exit(f"{path}:{lineno}: expected 5 or {5 + 3 * NUM_KPTS} values, got {len(v)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("reviewed", help="folder with images/ and labels/ (YOLO format, reviewed)")
    ap.add_argument("-o", "--out", required=True, help="new or empty output folder")
    ap.add_argument("--size", type=int, default=640, help="stretch frames to SIZE x SIZE")
    ap.add_argument("--val-start", default="", help="first frame name (timestamp) of the val block")
    ap.add_argument("--val-end", default="", help="last frame name of the val block")
    ap.add_argument("--val-frac", type=float, default=0.2,
                    help="without --val-start/--val-end: the last this fraction of frames by name")
    a = ap.parse_args()

    src, out = Path(a.reviewed), Path(a.out)
    if out.exists() and any(out.iterdir()):
        sys.exit(f"{out} is not empty -- choose a new folder (nothing is overwritten)")
    images = sorted(p for p in (src / "images").iterdir() if p.suffix.lower() in (".jpg", ".jpeg", ".png"))
    if not images:
        sys.exit(f"no images in {src / 'images'}")

    if a.val_start or a.val_end:
        lo, hi = a.val_start or images[0].stem, a.val_end or images[-1].stem
        is_val = {p: lo <= p.stem <= hi for p in images}
    else:
        n_val = max(1, int(round(len(images) * a.val_frac)))
        is_val = {p: i >= len(images) - n_val for i, p in enumerate(images)}

    for split in ("train", "val"):
        (out / "images" / split).mkdir(parents=True, exist_ok=True)
        (out / "labels" / split).mkdir(parents=True, exist_ok=True)

    counts = {"train": [0, 0], "val": [0, 0]}   # frames, objects
    for p in images:
        split = "val" if is_val[p] else "train"
        img = cv2.imread(str(p))
        if img is None:
            sys.exit(f"unreadable image {p}")
        img = cv2.resize(img, (a.size, a.size), interpolation=cv2.INTER_AREA)   # stretch
        cv2.imwrite(str(out / "images" / split / f"{p.stem}.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 95])
        lab = src / "labels" / f"{p.stem}.txt"
        lines = [l for l in lab.read_text().splitlines() if l.strip()] if lab.exists() else []
        conv = [convert(l, i + 1, lab) for i, l in enumerate(lines)]
        (out / "labels" / split / f"{p.stem}.txt").write_text("\n".join(conv) + ("\n" if conv else ""))
        counts[split][0] += 1
        counts[split][1] += len(conv)

    names = "\n".join(f"  {k}: {v}" for k, v in NAMES.items())
    (out / "data.yaml").write_text(
        f"# Built by training/build_dataset.py from {src.resolve()}\n"
        f"path: {out.resolve().as_posix()}\n"
        f"train: images/train\n"
        f"val: images/val\n"
        f"kpt_shape: [{NUM_KPTS}, 3]\n"
        f"flip_idx: {FLIP_IDX}\n"
        f"names:\n{names}\n")
    for split, (nf, no) in counts.items():
        print(f"{split}: {nf} frames, {no} objects")
    if counts["val"][0] == 0 or counts["train"][0] == 0:
        print("WARNING: one split is empty -- check --val-start/--val-end")
    print(f"-> {out / 'data.yaml'}")


if __name__ == "__main__":
    main()
