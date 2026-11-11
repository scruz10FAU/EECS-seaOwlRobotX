#!/usr/bin/env python3
"""
prelabel_beacons.py -- propose "beacon" boxes on tracking_front frames for review.

The beacons in the Course Charlie frames are bright white pyramids on dark
water, so a brightness threshold finds most of them. This writes one candidate
box per bright spot on the water in standard YOLO *detection* format, so they
can be reviewed (moved / deleted / added) in any box tool -- CVAT ("YOLO 1.1" /
"Ultralytics YOLO Detection"), labelImg, Roboflow (YOLO TXT). No keypoints are
needed for "beacon"; build_dataset.py adds the pose fields afterwards.

    python3 training/prelabel_beacons.py <images_dir> -o <out_dir>
    python3 training/prelabel_beacons.py <images_dir> -o <out_dir> --every 3 --preview

Output (<out_dir> must be new or empty -- nothing is overwritten):
    images/<name>.jpg        copies of the selected frames (original size)
    labels/<name>.txt        "0 cx cy w h" per candidate (normalised), class 0 = beacon
    classes.txt / obj.names  "beacon"
    preview/<name>.jpg       (--preview) frames with the candidates drawn

It WILL propose some non-beacons (white floating platforms, the shore, glare)
and miss dim / far beacons: the point is fewer clicks, not a finished label set.
Label EVERY visible beacon during review -- an unlabelled beacon teaches the
model that beacons are background.
"""

import argparse
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np


def candidates(gray, a):
    """Bright, beacon-shaped spots surrounded by dark water -> [(x1, y1, x2, y2)] px.

    A beacon here is a small, roughly square bright object with dark water all
    round it. Rejected: wrong size / elongated spots (signs, railings), spots
    whose surroundings aren't dark like water (dock, tents, shore, sky), and
    spots with many other bright specks nearby (sun glint fields)."""
    h, w = gray.shape
    y0 = int(h * a.roi_top)
    roi = gray[y0:]
    water = float(np.median(roi))
    # Bright relative to this frame's water, and bright in absolute terms.
    level = max(a.thresh, water + a.rel)
    mask = (roi >= level).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    n, _, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
    spots = [i for i in range(1, n) if stats[i][4] >= 2]          # every bright speck, for density
    pts = np.array([cents[i] for i in spots]) if spots else np.zeros((0, 2))
    boxes = []
    for i in range(1, n):
        x, y, bw, bh, area = stats[i]
        if not (a.min_area <= area <= a.max_area):
            continue
        if not (1 / a.max_aspect <= bw / bh <= a.max_aspect):
            continue
        # Glint field: many other bright specks within cluster_radius.
        if len(pts) and (np.hypot(*(pts - cents[i]).T) <= a.cluster_radius).sum() - 1 > a.cluster_max:
            continue
        # Surroundings must be dark like water: ring of one box-size around it.
        rx1, ry1 = max(0, x - bw), max(0, y - bh)
        rx2, ry2 = min(roi.shape[1], x + 2 * bw), min(roi.shape[0], y + 2 * bh)
        ring = roi[ry1:ry2, rx1:rx2].astype(np.float32).copy()
        ring[y - ry1:y - ry1 + bh, x - rx1:x - rx1 + bw] = np.nan
        if np.nanmedian(ring) > water + a.ring_max:
            continue
        # The bright part is the sunlit face; grow the box to the whole beacon.
        px, py = max(2, int(bw * a.pad)), max(2, int(bh * a.pad))
        boxes.append((max(0, x - px), max(0, y0 + y - py),
                      min(w, x + bw + px), min(h, y0 + y + bh + py)))
    return boxes


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("images", help="folder of frames (e.g. Robonation_real_Data/images)")
    ap.add_argument("-o", "--out", required=True, help="new or empty output folder")
    ap.add_argument("--every", type=int, default=3,
                    help="take every Nth frame (consecutive frames are near-identical)")
    ap.add_argument("--start", default="", help="only frames whose name sorts >= this (e.g. 20261111_014908)")
    ap.add_argument("--end", default="", help="only frames whose name sorts <= this")
    ap.add_argument("--roi-top", type=float, default=0.52,
                    help="only look below this fraction of the height (the water)")
    ap.add_argument("--thresh", type=float, default=150, help="absolute brightness for a beacon (0-255)")
    ap.add_argument("--rel", type=float, default=60, help="...and at least this much above the water median")
    ap.add_argument("--min-area", type=int, default=6, help="smallest bright spot (px)")
    ap.add_argument("--max-area", type=int, default=800, help="largest bright spot (px)")
    ap.add_argument("--max-aspect", type=float, default=2.5,
                    help="bright spot no more than this many times wider than tall (or taller than wide)")
    ap.add_argument("--ring-max", type=float, default=25,
                    help="the area round a spot may be at most this much brighter than the water median")
    ap.add_argument("--cluster-radius", type=float, default=40, help="px, for the glint test")
    ap.add_argument("--cluster-max", type=int, default=4,
                    help="more bright specks than this within --cluster-radius = sun glint, skipped")
    ap.add_argument("--pad", type=float, default=0.25, help="grow each box by this fraction per side")
    ap.add_argument("--preview", action="store_true", help="also write frames with the boxes drawn")
    a = ap.parse_args()

    src = Path(a.images)
    out = Path(a.out)
    if out.exists() and any(out.iterdir()):
        sys.exit(f"{out} is not empty -- choose a new folder (nothing is overwritten)")
    files = sorted(p for p in src.iterdir() if p.suffix.lower() in (".jpg", ".jpeg", ".png"))
    files = [p for p in files if (not a.start or p.stem >= a.start) and (not a.end or p.stem <= a.end)]
    files = files[::max(1, a.every)]
    if not files:
        sys.exit("no frames selected")

    for sub in ("images", "labels") + (("preview",) if a.preview else ()):
        (out / sub).mkdir(parents=True, exist_ok=True)
    for name in ("classes.txt", "obj.names"):
        (out / name).write_text("beacon\n")

    total = empty = 0
    for p in files:
        img = cv2.imread(str(p))
        if img is None:
            print(f"skip unreadable {p.name}")
            continue
        h, w = img.shape[:2]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
        boxes = candidates(gray, a)
        shutil.copy2(p, out / "images" / p.name)
        lines = [f"0 {(x1 + x2) / 2 / w:.6f} {(y1 + y2) / 2 / h:.6f} {(x2 - x1) / w:.6f} {(y2 - y1) / h:.6f}"
                 for x1, y1, x2, y2 in boxes]
        (out / "labels" / f"{p.stem}.txt").write_text("\n".join(lines) + ("\n" if lines else ""))
        total += len(boxes)
        empty += not boxes
        if a.preview:
            vis = img.copy()
            cv2.line(vis, (0, int(h * a.roi_top)), (w, int(h * a.roi_top)), (255, 128, 0), 1)
            for x1, y1, x2, y2 in boxes:
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 0, 255), 1)
            cv2.imwrite(str(out / "preview" / f"{p.stem}.jpg"), vis)
    print(f"{len(files)} frames -> {out}: {total} candidate boxes "
          f"({total / max(len(files), 1):.1f}/frame), {empty} frames with none")


if __name__ == "__main__":
    main()
