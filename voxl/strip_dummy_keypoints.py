#!/usr/bin/env python3
"""Remove placeholder keypoints from a RoboFlow COCO export.

RoboFlow refuses a keypoint skeleton with zero keypoints, so a box-only class
(e.g. "beacon") is given a single placeholder keypoint ("dummy") that RoboFlow
auto-places on every instance. Those points carry no information and would be
trained as real labels, so this strips them:

  - the keypoint is removed from the category's "keypoints" list and the
    skeleton is renumbered (edges touching it are dropped)
  - its (x, y, v) triple is removed from every annotation of that category
  - if no keypoints remain, the category gets keypoints/skeleton = [] and its
    annotations lose "keypoints"/"num_keypoints" entirely -- the same shape
    RoboFlow itself exports for a box-only class

--mode hide instead keeps the keypoint (RoboFlow drops a keypoint-project
class that has none) but marks it invisible -- (0, 0, 0) -- on every instance,
so it carries no training signal. Categories are left untouched.

Images are copied byte-for-byte. Other classes are untouched.

Usage:
    python3 strip_dummy_keypoints.py "~/Downloads/Beacon_Keypoints_Tracking.v4i.coco.zip"
    python3 strip_dummy_keypoints.py <zip-or-dir> --mode hide
    python3 strip_dummy_keypoints.py <zip-or-dir> -o out.zip --keypoint dummy
"""

import argparse
import json
import sys
import zipfile
from pathlib import Path


def strip_categories(coco, kp_name):
    """Edit categories in place; return {category_id: index of the removed keypoint}."""
    removed = {}
    for cat in coco["categories"]:
        kps = cat.get("keypoints") or []
        if kp_name not in kps:
            continue
        idx = kps.index(kp_name)
        removed[cat["id"]] = idx
        cat["keypoints"] = kps[:idx] + kps[idx + 1:]
        old = idx + 1  # skeleton edges are 1-based
        cat["skeleton"] = [[a - (a > old), b - (b > old)]
                           for a, b in (cat.get("skeleton") or []) if old not in (a, b)]
    return removed


def placeholder_index(coco, kp_name):
    """{category_id: index of kp_name} without editing the categories."""
    return {c["id"]: c["keypoints"].index(kp_name)
            for c in coco["categories"] if kp_name in (c.get("keypoints") or [])}


def hide_annotations(coco, found):
    n = 0
    for ann in coco["annotations"]:
        idx = found.get(ann["category_id"])
        if idx is None or "keypoints" not in ann:
            continue
        ann["keypoints"][3 * idx:3 * idx + 3] = [0, 0, 0]
        ann["num_keypoints"] = sum(1 for v in ann["keypoints"][2::3] if v > 0)
        n += 1
    return n


def strip_annotations(coco, removed):
    n = 0
    for ann in coco["annotations"]:
        idx = removed.get(ann["category_id"])
        if idx is None or "keypoints" not in ann:
            continue
        kp = ann["keypoints"]
        kp = kp[:3 * idx] + kp[3 * idx + 3:]
        if kp:
            ann["keypoints"] = kp
            ann["num_keypoints"] = sum(1 for v in kp[2::3] if v > 0)
        else:
            ann.pop("keypoints", None)
            ann.pop("num_keypoints", None)
        n += 1
    return n


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("source", help="RoboFlow COCO export (.zip or extracted directory)")
    ap.add_argument("-o", "--out", help="output .zip (default: <source>.nodummy.zip)")
    ap.add_argument("--keypoint", default="dummy", help="placeholder keypoint name")
    ap.add_argument("--mode", choices=("strip", "hide"), default="strip",
                    help="strip: delete the keypoint; hide: keep it, mark invisible (0,0,0)")
    args = ap.parse_args()

    src = Path(args.source).expanduser()
    stem = src.name[:-4] if src.suffix == ".zip" else src.name
    suffix = "nodummy" if args.mode == "strip" else "dummyhidden"
    out = Path(args.out).expanduser() if args.out else src.with_name(f"{stem}.{suffix}.zip")
    if out.exists():
        sys.exit(f"refusing to overwrite {out}")

    if src.is_file():
        zin = zipfile.ZipFile(src)
        names, read = zin.namelist(), zin.read
    else:
        files = [p for p in src.rglob("*") if p.is_file()]
        names = [p.relative_to(src).as_posix() for p in files]
        read = lambda n: (src / n).read_bytes()  # noqa: E731

    total_ann, total_cat = 0, set()
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as zout:
        for name in names:
            data = read(name)
            if name.endswith("_annotations.coco.json"):
                coco = json.loads(data)
                if args.mode == "strip":
                    removed = strip_categories(coco, args.keypoint)
                    n = strip_annotations(coco, removed)
                else:
                    removed = placeholder_index(coco, args.keypoint)
                    n = hide_annotations(coco, removed)
                names_by_id = {c["id"]: c["name"] for c in coco["categories"]}
                total_cat |= {names_by_id[c] for c in removed}
                total_ann += n
                print(f"  {name}: {args.mode} '{args.keypoint}' in "
                      f"{[names_by_id[c] for c in removed]} ({n} annotations)")
                data = json.dumps(coco).encode()
            zout.writestr(name, data)
    if not total_cat:
        print(f"note: no category had a '{args.keypoint}' keypoint; output is a plain copy")
    print(f"{src} -> {out}  ({total_ann} annotations, mode={args.mode})")


if __name__ == "__main__":
    main()
