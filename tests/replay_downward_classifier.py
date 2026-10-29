#!/usr/bin/env python3
"""
replay_downward_classifier.py — run the real downward beacon classifier
(utils/downward_beacon_classifier.classify_beacon_downward) offline on
recordings from telemetry_viewer.py (<ts>.mkv + <ts>.csv), no drone or ROS.

Each video is split into --window second windows by the CSV's frame_stamp,
downsampled to --max-sample-hz like the live node, and classified once per
window with the pose of the window's LAST frame (as the live node samples
pose at the end of its window):
  height     height_agl_m (or --fallback-height-agl if blank)
  attitude   qw..qz (PX4 FRD->NED) converted to FLU->ENU, same as
             utils/beacon_camera.py _on_px4_odometry()
  position   drone at local [0, 0, h], GPS origin = drone's fix with
             altitude gps_alt - h -- the CSV has no EKF origin, so the
             drone's own fix is the consistent pairing.

Needs: cv2, numpy, scipy (pip install scipy), and beacon_detector_config's
imports (ultralytics). Run from the repo root.

Usage:
    py -3.10 tests\\replay_downward_classifier.py                         # every video in the default folder
    py -3.10 tests\\replay_downward_classifier.py C:\\path\\to\\20261028_094926_837303.mkv
    py -3.10 tests\\replay_downward_classifier.py --window 8 --step 2 --debug-dir replay_debug
"""

import argparse
import csv
import glob
import math
import os
import sys
from datetime import datetime

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import beacon_detector_config as bdc  # noqa: E402
from utils.downward_beacon_classifier import classify_beacon_downward  # noqa: E402

DEFAULT_VIDEO_DIR = os.path.join(os.path.expanduser("~"), "captures", "videos")
DEFAULT_CONFIG = os.path.join("configs", "modal_config.json")

# Same frame swaps as utils/beacon_camera.py.
_R_NED_TO_ENU = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=float)
_R_FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])

_RESULT_FIELDS = ["video", "window", "start_frame", "end_frame", "start_stamp", "end_stamp",
                  "n_frames", "n_housing", "height_agl_m", "color", "blink", "blink_color",
                  "blink_hz", "phase", "latitude", "longitude", "altitude",
                  "drone_lat", "drone_lon",
                  "GT color", "GT status", "Color val", "status val", "Overall"]
_GT_COLORS = ("red", "green", "blue")
_GT_STATUS = {"blink": True, "solid": False}


def _ground_truth(name):
    """(color, is_blinking) from a <color>_<blink|solid>.mkv name, else (None, None)."""
    parts = os.path.splitext(name)[0].lower().split("_")
    if len(parts) >= 2 and parts[0] in _GT_COLORS and parts[1] in _GT_STATUS:
        return parts[0], _GT_STATUS[parts[1]]
    return None, None


def _f(row, key):
    v = row.get(key, "")
    return float(v) if v not in ("", None) else None


def _quat_frd_ned_to_flu_enu(qw, qx, qy, qz):
    from scipy.spatial.transform import Rotation
    R = _R_NED_TO_ENU @ Rotation.from_quat([qx, qy, qz, qw]).as_matrix() @ _R_FLU_TO_FRD
    x, y, z, w = Rotation.from_matrix(R).as_quat()
    return [w, x, y, z]


def _load_rows(csv_path):
    with open(csv_path, newline="") as fh:
        return {int(r["frame"]): r for r in csv.DictReader(fh)}


def _read_frames(mkv_path, rows, max_sample_hz):
    """(frame_idx, stamp, bgr) for frames that have a CSV row, rate-limited by stamp."""
    cap = cv2.VideoCapture(mkv_path)
    if not cap.isOpened():
        raise RuntimeError(f"cannot open {mkv_path}")
    min_dt = 1.0 / max_sample_hz if max_sample_hz else 0.0
    last = None
    idx = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        row = rows.get(idx)
        stamp = _f(row, "frame_stamp") if row else None
        if stamp is not None and (last is None or stamp - last >= min_dt):
            yield idx, stamp, frame
            last = stamp
        idx += 1
    cap.release()


def _windows(frames, window_s, step_s):
    """Group (idx, stamp, bgr) into windows of window_s seconds every step_s."""
    frames = list(frames)
    if not frames:
        return
    t0, t_end = frames[0][1], frames[-1][1]
    start = t0
    while start <= t_end:
        win = [f for f in frames if start <= f[1] < start + window_s]
        if win:
            yield win
        start += step_s


def replay_video(mkv_path, cfg, args, writer):
    csv_path = os.path.splitext(mkv_path)[0] + ".csv"
    if not os.path.exists(csv_path):
        print(f"[replay] skip {mkv_path}: no {os.path.basename(csv_path)}")
        return
    rows = _load_rows(csv_path)
    down_cfg = cfg["downward_camera"]
    beacon_h = down_cfg["beacon_height_m"]
    name = os.path.basename(mkv_path)
    print(f"[replay] {name}: {len(rows)} telemetry rows")

    gt_color, gt_blink = _ground_truth(name)
    scores = []   # (video, color_ok, status_ok) per window with ground truth
    frames = _read_frames(mkv_path, rows, args.max_sample_hz)
    for w, win in enumerate(_windows(frames, args.window, args.step)):
        last = rows[win[-1][0]]
        h = _f(last, "height_agl_m")
        if h is None or h <= beacon_h:
            h = args.fallback_height_agl
        qw, qx, qy, qz = (_f(last, k) for k in ("qw", "qx", "qy", "qz"))
        quat = (_quat_frd_ned_to_flu_enu(qw, qx, qy, qz) if None not in (qw, qx, qy, qz)
                else [1.0, 0.0, 0.0, 0.0])
        lat, lon, alt = (_f(last, k) for k in ("gps_lat", "gps_lon", "gps_alt_msl_m"))
        fix = (_f(last, "gps_fix_type") or 0) >= 3 and lat is not None and lon is not None
        gps_origin = (lat, lon, (alt or 0.0) - h) if fix else None

        debug_dir = (os.path.join(args.debug_dir, os.path.splitext(name)[0], f"win{w:03d}")
                     if args.debug_dir else None)
        result = classify_beacon_downward(
            [(stamp, bgr) for _, stamp, bgr in win],
            [0.0, 0.0, h], quat, gps_origin, h,
            down_cfg, cfg["detection"], args.window,
            debug_image_dir=debug_dir,
            expected_radius_px_override=args.expected_radius_px,
        )
        n_housing = sum(1 for r in result["frame_log"] if r["source"] in ("detected", "locked"))
        gps = result["gps_position"] or {}
        gt = {}
        if gt_color is not None:
            # Unknown (None) blink never matches.
            color_ok = result["blink_color"] == gt_color
            status_ok = result["blink"] is gt_blink
            gt = {"GT color": gt_color, "GT status": gt_blink, "Color val": color_ok,
                  "status val": status_ok, "Overall": color_ok and status_ok}
            scores.append((name, color_ok, status_ok))
        writer.writerow({
            **gt,
            "video": name, "window": w, "start_frame": win[0][0], "end_frame": win[-1][0],
            "start_stamp": f"{win[0][1]:.3f}", "end_stamp": f"{win[-1][1]:.3f}",
            "n_frames": len(win), "n_housing": n_housing, "height_agl_m": f"{h:.2f}",
            "color": result["color"], "blink": result["blink"],
            "blink_color": result["blink_color"], "blink_hz": result["blink_hz"],
            "phase": result["phase"], "latitude": gps.get("latitude", ""),
            "longitude": gps.get("longitude", ""), "altitude": gps.get("altitude", ""),
            "drone_lat": lat if fix else "", "drone_lon": lon if fix else "",
        })
        where = (f"lat={gps['latitude']:.7f} lon={gps['longitude']:.7f}" if gps
                 else "gps=none")
        print(f"  win {w:3d} frames {win[0][0]:5d}-{win[-1][0]:5d}  h={h:5.2f}m  "
              f"housing {n_housing:2d}/{len(win):2d}  color={result['color']:7s} "
              f"blink={str(result['blink']):5s} blink_color={result['blink_color']:7s} {where}")
    return scores


def _print_scores(scores):
    """Color / status / overall accuracy, total and per video."""
    if not scores:
        return
    def line(rows):
        n = len(rows)
        c = sum(1 for _, ok, _s in rows if ok)
        s = sum(1 for _, _c, ok in rows if ok)
        o = sum(1 for _, cok, sok in rows if cok and sok)
        return f"color {c}/{n}  status {s}/{n}  overall {o}/{n}"
    print(f"[replay] score: {line(scores)}")
    for video in dict.fromkeys(v for v, _, _ in scores):
        print(f"  {video:20s} {line([r for r in scores if r[0] == video])}")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("videos", nargs="*", default=[DEFAULT_VIDEO_DIR],
                    help=f".mkv files or folders (default: {DEFAULT_VIDEO_DIR})")
    ap.add_argument("--config", default=DEFAULT_CONFIG,
                    help=f"Seabird JSON config (default: {DEFAULT_CONFIG})")
    ap.add_argument("--window", type=float, default=None,
                    help="Seconds per classification (default: downward_test.duration)")
    ap.add_argument("--step", type=float, default=None,
                    help="Seconds between window starts (default: --window, no overlap)")
    ap.add_argument("--max-sample-hz", type=float, default=None,
                    help="Frame sampling rate (default: downward_test.max_sample_hz)")
    ap.add_argument("--fallback-height-agl", type=float, default=None,
                    help="Height when the CSV has none (default: downward_test.fallback_height_agl)")
    ap.add_argument("--expected-radius-px", type=float, default=None,
                    help="Override the height-derived disc radius")
    ap.add_argument("--debug-dir", default=None,
                    help="Write the classifier's per-frame debug images under this folder "
                         "(a relative name goes under <video folder>\\replay\\)")
    ap.add_argument("--out", default=None,
                    help="Summary CSV, one row per window (default: "
                         "<video folder>\\replay\\replay_results_<YYYYmmdd_HHMMSS>.csv)")
    args = ap.parse_args()

    cfg = bdc.load_config(args.config)
    test_cfg = cfg["downward_test"]
    args.window = args.window or test_cfg["duration"]
    args.step = args.step or args.window
    if args.max_sample_hz is None:
        args.max_sample_hz = test_cfg["max_sample_hz"]
    if args.fallback_height_agl is None:
        args.fallback_height_agl = test_cfg["fallback_height_agl"]
    if args.expected_radius_px is None:
        args.expected_radius_px = test_cfg.get("expected_radius_px")

    paths = []
    for p in args.videos:
        paths += sorted(glob.glob(os.path.join(p, "*.mkv"))) if os.path.isdir(p) else [p]
    if not paths:
        sys.exit(f"[replay] no .mkv files in {args.videos}")

    # Keep results with the recordings, not in the repo / current directory.
    replay_dir = os.path.join(os.path.dirname(os.path.abspath(paths[0])), "replay")
    if args.out is None:
        args.out = os.path.join(
            replay_dir, f"replay_results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    if args.debug_dir and not os.path.isabs(args.debug_dir):
        args.debug_dir = os.path.join(replay_dir, args.debug_dir)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)

    # "x": never overwrite an earlier results file (e.g. a reused --out).
    if os.path.exists(args.out):
        sys.exit(f"[replay] {args.out} already exists -- not overwriting it; pick another --out")
    scores = []
    with open(args.out, "x", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=_RESULT_FIELDS)
        writer.writeheader()
        for path in paths:
            scores += replay_video(path, cfg, args, writer) or []
            fh.flush()
    print(f"[replay] results -> {args.out}")
    _print_scores(scores)


if __name__ == "__main__":
    main()
