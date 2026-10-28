#!/usr/bin/env python3
"""
test_disc_detection.py — measure the downward beacon disc detector
(find_white_box + find_disc_in_box from utils/downward_beacon_classifier.py)
on telemetry_viewer.py recordings, plus the per-frame LED colour and the
per-window colour/blink the full classifier gives.

Per video it reports:
  detection   frames with a disc found / frames checked, longest miss streak
  radius      median and IQR, disc diameter / box side (10 in / 17 in = 0.59)
  jitter      median frame-to-frame disc centre jump (large = false hits /
              switching objects)
  colour      per-frame LED colour counts (classify_frame on the LED region)
  windows     the full classify_beacon_downward() result per --window
              (colour, blink), same windowing/pose as replay_downward_classifier.py
  accuracy    with labels: precision, recall, median centre error, false
              positives on frames with no beacon

Labels (--label): click the disc centre, 'n' = no beacon in view,
space = skip, 'b' = back, 'q' = save and quit. Saved to <video>_disc_labels.csv
next to the video and used by every later run.

Output under <video folder>\\replay\\disc_test\\: <video>_frames.csv (one row
per checked frame) and <video>_sheet_NN.jpg contact sheets (crop around each
detection, circle drawn; MISS tiles centred on the frame).

Usage:
    py -3.10 tests\\test_disc_detection.py                     # every video in the default folder
    py -3.10 tests\\test_disc_detection.py --every 5 C:\\...\\blue_blink.mkv
    py -3.10 tests\\test_disc_detection.py --label --every 10 C:\\...\\blue_blink.mkv
"""

import argparse
import contextlib
import csv
import glob
import io
import math
import os
import sys
import types

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import beacon_detector_config as bdc  # noqa: E402
import utils.downward_beacon_classifier as dbc  # noqa: E402
import replay_downward_classifier as replay  # noqa: E402

TILE_W, TILE_H, SHEET_COLS, SHEET_ROWS = 240, 180, 8, 6
CROP_HALF = 60   # px around the detection, before scaling to the tile


def detect(frame):
    """(box, (x, y), r) for the largest white box with a real disc, or None."""
    for box in dbc.find_white_box(frame, return_all=True):
        disc = dbc.find_disc_in_box(frame, box)
        if disc is not None:
            return box, disc[0], disc[1]
    return None


def led_color(frame, xy, r):
    """Colour of the LED region in the disc centre, as the classifier samples it."""
    mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    cv2.circle(mask, (round(xy[0]), round(xy[1])),
               max(1, round(max(2.0, dbc._LED_RADIUS_FRAC * r))), 255, thickness=-1)
    with contextlib.redirect_stdout(io.StringIO()):   # classify_beacon_color prints hue stats
        color, conf, _intensity, _votes, _hue = dbc.classify_frame(frame, mask)
    return color, conf


def load_labels(path):
    if not os.path.exists(path):
        return {}
    out = {}
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            out[int(r["frame"])] = None if r["x"] == "" else (float(r["x"]), float(r["y"]))
    return out


def save_labels(path, labels):
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["frame", "x", "y"])
        for f in sorted(labels):
            xy = labels[f]
            w.writerow([f, "", ""] if xy is None else [f, f"{xy[0]:.1f}", f"{xy[1]:.1f}"])


def label_video(mkv, every, labels):
    """Click the disc centre on every Nth frame; returns the updated labels."""
    cap = cv2.VideoCapture(mkv)
    frames = []
    idx = 0
    while True:
        ok, f = cap.read()
        if not ok:
            break
        if idx % every == 0:
            frames.append((idx, f))
        idx += 1
    cap.release()
    win = "label: click disc centre | n=no beacon  space=skip  b=back  q=quit"
    click = {}
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, lambda e, x, y, *_: click.update(xy=(x, y)) if e == cv2.EVENT_LBUTTONDOWN else None)
    k = 0
    while 0 <= k < len(frames):
        fi, f = frames[k]
        view = f.copy()
        if fi in labels and labels[fi] is not None:
            cv2.drawMarker(view, (round(labels[fi][0]), round(labels[fi][1])), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
        state = "unlabelled" if fi not in labels else ("no beacon" if labels[fi] is None else "labelled")
        cv2.putText(view, f"frame {fi} ({k + 1}/{len(frames)}) {state}", (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow(win, view)
        click.clear()
        while True:
            key = cv2.waitKey(30) & 0xFF
            if "xy" in click:
                labels[fi] = (float(click["xy"][0]), float(click["xy"][1]))
                k += 1
                break
            if key == ord("n"):
                labels[fi] = None
                k += 1
                break
            if key == ord(" "):
                k += 1
                break
            if key == ord("b"):
                k = max(0, k - 1)
                break
            if key == ord("q"):
                k = len(frames)
                break
    cv2.destroyWindow(win)
    return labels


def window_results(mkv, cfg, window, step, max_hz, fallback_h):
    """Full classifier per window, via replay_downward_classifier.replay_video()."""
    rows = []

    class _Collect:
        def writerow(self, r):
            rows.append(r)

    args = types.SimpleNamespace(window=window, step=step, max_sample_hz=max_hz,
                                 fallback_height_agl=fallback_h, expected_radius_px=None,
                                 debug_dir=None)
    with contextlib.redirect_stdout(io.StringIO()):   # the classifier prints per frame
        replay.replay_video(mkv, cfg, args, _Collect())
    return rows


def write_sheets(tiles, out_dir, name):
    per = SHEET_COLS * SHEET_ROWS
    for s in range(0, len(tiles), per):
        chunk = tiles[s:s + per]
        while len(chunk) % SHEET_COLS:
            chunk.append(np.zeros((TILE_H, TILE_W, 3), np.uint8))
        rows = [np.hstack(chunk[i:i + SHEET_COLS]) for i in range(0, len(chunk), SHEET_COLS)]
        cv2.imwrite(os.path.join(out_dir, f"{name}_sheet_{s // per:02d}.jpg"), np.vstack(rows))


def make_tile(frame, det, fi, color, label_xy):
    o = frame.copy()
    if det is not None:
        box, (x, y), r = det
        half = box[2] / 2.0
        cv2.rectangle(o, (round(box[0] - half), round(box[1] - half)),
                      (round(box[0] + half), round(box[1] + half)), (255, 0, 255), 1)
        cv2.circle(o, (round(x), round(y)), max(1, round(r)), (255, 255, 0), 1)
        cx, cy = round(x), round(y)
    else:
        cx, cy = frame.shape[1] // 2, frame.shape[0] // 2
    if label_xy is not None:
        cv2.drawMarker(o, (round(label_xy[0]), round(label_xy[1])), (0, 255, 255), cv2.MARKER_CROSS, 10, 1)
    h, w = o.shape[:2]
    x0, y0 = min(max(0, cx - CROP_HALF * 4 // 3), w - 1), min(max(0, cy - CROP_HALF), h - 1)
    crop = o[y0:y0 + 2 * CROP_HALF, x0:x0 + CROP_HALF * 8 // 3]
    tile = cv2.resize(crop, (TILE_W, TILE_H), interpolation=cv2.INTER_NEAREST)
    text = f"f{fi} " + (f"r={det[2]:.1f} {color}" if det is not None else "MISS")
    cv2.putText(tile, text, (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    return tile


def evaluate(mkv, cfg, args, out_dir):
    name = os.path.splitext(os.path.basename(mkv))[0]
    label_path = os.path.splitext(mkv)[0] + "_disc_labels.csv"
    labels = load_labels(label_path)
    if args.label:
        labels = label_video(mkv, args.every, labels)
        save_labels(label_path, labels)
        print(f"[disc-test] {len(labels)} labels -> {label_path}")

    cap = cv2.VideoCapture(mkv)
    rows, tiles = [], []
    idx = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if idx % args.every == 0:
            det = detect(frame)
            color, conf = led_color(frame, det[1], det[2]) if det else ("", 0.0)
            lab = labels.get(idx, "unlabelled")
            err = ""
            if det and lab not in ("unlabelled", None):
                err = math.hypot(det[1][0] - lab[0], det[1][1] - lab[1])
            rows.append({
                "frame": idx, "found": int(det is not None),
                "x": f"{det[1][0]:.1f}" if det else "", "y": f"{det[1][1]:.1f}" if det else "",
                "radius_px": f"{det[2]:.2f}" if det else "",
                "box_side_px": f"{det[0][2]:.1f}" if det else "",
                "led_color": color, "led_conf": f"{conf:.2f}" if det else "",
                "label": "" if lab == "unlabelled" else ("none" if lab is None else f"{lab[0]:.1f};{lab[1]:.1f}"),
                "center_err_px": f"{err:.1f}" if err != "" else "",
            })
            tiles.append(make_tile(frame, det, idx, color, lab if lab not in ("unlabelled", None) else None))
        idx += 1
    cap.release()

    with open(os.path.join(out_dir, f"{name}_frames.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()) if rows else ["frame"])
        w.writeheader()
        w.writerows(rows)
    write_sheets(tiles, out_dir, name)

    found = [r for r in rows if r["found"]]
    radii = np.array([float(r["radius_px"]) for r in found])
    ratios = np.array([2 * float(r["radius_px"]) / float(r["box_side_px"]) for r in found])
    streak = longest = 0
    for r in rows:
        streak = 0 if r["found"] else streak + 1
        longest = max(longest, streak)
    jumps = [math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"]))
             for a, b in zip(rows, rows[1:]) if a["found"] and b["found"]]
    colors = {}
    for r in found:
        colors[r["led_color"]] = colors.get(r["led_color"], 0) + 1

    print(f"\n[disc-test] {name}  (every {args.every} frames)")
    print(f"  detection  {len(found)}/{len(rows)} ({100 * len(found) / max(1, len(rows)):.0f}%)"
          f"  longest miss streak {longest} checked frames")
    if len(radii):
        q1, q3 = np.percentile(radii, [25, 75])
        print(f"  radius     median {np.median(radii):.1f}px  IQR {q1:.1f}-{q3:.1f}px"
              f"  disc/box {np.median(ratios):.2f} (expect ~0.59)")
    if jumps:
        print(f"  jitter     median {np.median(jumps):.1f}px  90th pct {np.percentile(jumps, 90):.1f}px"
              f"  max {max(jumps):.1f}px")
    print(f"  LED colour " + ", ".join(f"{c or '?'} {n}" for c, n in sorted(colors.items(), key=lambda t: -t[1])))

    labelled = [r for r in rows if r["label"]]
    if labelled:
        pos = [r for r in labelled if r["label"] != "none"]
        neg = [r for r in labelled if r["label"] == "none"]
        tp = [r for r in pos if r["found"] and float(r["center_err_px"]) <= args.tol]
        wrong = [r for r in pos if r["found"] and float(r["center_err_px"]) > args.tol]
        fp_neg = [r for r in neg if r["found"]]
        n_det = len(tp) + len(wrong) + len(fp_neg)
        errs = [float(r["center_err_px"]) for r in pos if r["found"]]
        print(f"  accuracy   {len(labelled)} labelled ({len(pos)} with beacon, {len(neg)} without), tol {args.tol}px")
        print(f"             precision {len(tp)}/{n_det}  recall {len(tp)}/{len(pos)}"
              f"  median centre error {np.median(errs):.1f}px" if errs else
              f"             precision {len(tp)}/{n_det}  recall {len(tp)}/{len(pos)}")
        print(f"             wrong spot {len(wrong)}  false positives with no beacon {len(fp_neg)}")
    else:
        print("  accuracy   no labels yet (run with --label)")

    if not args.no_windows:
        wins = window_results(mkv, cfg, args.window, args.step, args.max_sample_hz, args.fallback_height_agl)
        print("  windows    " + "  ".join(
            f"{w['window']}:{w['blink_color']}/{'B' if w['blink'] is True else 'S' if w['blink'] is False else '?'}"
            for w in wins))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("videos", nargs="*", default=[replay.DEFAULT_VIDEO_DIR],
                    help=f".mkv files or folders (default: {replay.DEFAULT_VIDEO_DIR})")
    ap.add_argument("--config", default=replay.DEFAULT_CONFIG)
    ap.add_argument("--every", type=int, default=1, help="Check every Nth frame (default: 1)")
    ap.add_argument("--label", action="store_true", help="Label disc centres before evaluating")
    ap.add_argument("--tol", type=float, default=5.0, help="Centre error counted as correct, px (default: 5)")
    ap.add_argument("--window", type=float, default=None, help="Blink window s (default: downward_test.duration)")
    ap.add_argument("--step", type=float, default=None, help="Window step s (default: --window)")
    ap.add_argument("--max-sample-hz", type=float, default=None)
    ap.add_argument("--no-windows", action="store_true", help="Skip the per-window colour/blink run")
    ap.add_argument("--out-dir", default=None, help="Default: <video folder>\\replay\\disc_test")
    args = ap.parse_args()

    cfg = bdc.load_config(args.config)
    # classify_frame uses the configured hue bands -- same merge as classify_beacon_downward()
    bdc._apply_color_config({**cfg["detection"], **cfg["downward_camera"].get("color_overrides", {})})
    t = cfg["downward_test"]
    args.window = args.window or t["duration"]
    args.step = args.step or args.window
    args.max_sample_hz = t["max_sample_hz"] if args.max_sample_hz is None else args.max_sample_hz
    args.fallback_height_agl = t["fallback_height_agl"]

    paths = []
    for p in args.videos:
        paths += sorted(glob.glob(os.path.join(p, "*.mkv"))) if os.path.isdir(p) else [p]
    if not paths:
        sys.exit(f"[disc-test] no .mkv files in {args.videos}")
    out_dir = args.out_dir or os.path.join(os.path.dirname(os.path.abspath(paths[0])), "replay", "disc_test")
    os.makedirs(out_dir, exist_ok=True)
    for p in paths:
        evaluate(p, cfg, args, out_dir)
    print(f"\n[disc-test] frame CSVs and contact sheets -> {out_dir}")


if __name__ == "__main__":
    main()
