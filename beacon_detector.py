#!/usr/bin/env python3
"""
beacon_detector.py — Two-stage beacon detection and color identification.

Stage 1 — Beacon detection (one_beacon.pt):
  Locates the beacon in the full frame and returns a bounding box.

Stage 2 — Lit-area isolation (best_crop.pt):
  The beacon bounding box is cropped and passed to a second model that
  isolates the glowing/lit portion of the beacon. Supports both detection
  (bbox output) and segmentation (mask output) model types. Falls back to
  HSV brightness thresholding if the crop model finds nothing.

Stage 3 — Color classification:
  Runs classify_beacon_color() on only the isolated lit region.
  1. Convert the lit crop to HSV.
  2. Mask pixels with high Value (≥ 160) and moderate Saturation (≥ 60).
  3. Compute the circular mean hue (handles red wrap-around at 0°/180°).
  4. Map the hue to: red, green, blue, or unknown.

Usage:
    python3 beacon_detector.py                          # ROS live mode
    python3 beacon_detector.py -d                       # ROS live mode with display
    python3 beacon_detector.py -rv footage.mp4          # video + ROS
    python3 beacon_detector.py -v footage.mp4           # video only, no ROS
    python3 beacon_detector.py -cm models/best_crop.pt  # custom crop model
"""

import sys
import argparse
import os
sys.path.insert(0, os.path.expanduser("~/seabird/scripts"))

import csv
import numpy as np
from typing import Tuple
import json
import time
import cv2

from blink_detector import BlinkDetector, _get_blink_detector

EARTH_RADIUS_M = 6378137.0

# Lazily imported only when ROS mode is used
def _import_ros():
    global rclpy, String, camera_to_world, BeaconCamera
    import rclpy as _rclpy; rclpy = _rclpy
    from std_msgs.msg import String as _Str; String = _Str
    from seabird_config import camera_to_world as _c2w; camera_to_world = _c2w
    from beacon_camera import BeaconCamera as _BC; BeaconCamera = _BC

# ── Color classification ───────────────────────────────────────────────────────

# HSV saturation/value thresholds for "bright, lit" pixels
_SAT_MIN = 60    # ignore nearly-grey pixels
_VAL_MIN = 160   # only consider bright pixels (the light itself)

# Hue bands for the four supported beacon colors (degrees, 0-180 in OpenCV).
# Each entry: (hue_center, half_width, label)
# Bands use STRICT membership — a hue must fall within center±half_width to match.
# Gaps between bands intentionally fall through to "other" rather than mis-snap.
_HUE_BANDS = [
    (  0, 20, "red"),    # 0–20  (widened to capture orange-red LEDs at hue 15–20)
    ( 65, 30, "green"),  # 35–95 (wide to cover teal-ish LEDs)
    (120, 15, "blue"),   # 105–135
    (165, 15, "red"),    # 150–180 (wrap-around)
]

# Minimum red-pixel fraction to declare "red", even when blue pixels outnumber red.
# When a red LED is on, vote_red is always ≥0.33 in practice.
# When the beacon is off (housing appears blue), vote_red is ≈0.
# Setting this to 0.25 catches dim/transitioning red LEDs without false-triggering on
# the off state.
_RED_THRESHOLD = 0.1


def _hue_votes(hues: np.ndarray) -> dict:
    """
    For each lit pixel, check which hue band it belongs to (strict membership).
    Returns fraction of lit pixels in each color: {"red", "green", "blue", "other"}.
    "other" = pixels whose hue does not fall in any defined band (gaps/ambiguous).
    Uses numpy vectorized ops — no Python pixel loop.
    """
    n = max(len(hues), 1)
    hues_f = hues.astype(np.float32)
    matched = np.zeros(len(hues), dtype=bool)
    label_counts: dict = {}

    for center, half, label in _HUE_BANDS:
        dist = np.abs(hues_f - center)
        dist = np.minimum(dist, 180.0 - dist)
        in_band = (dist <= half) & ~matched
        label_counts[label] = label_counts.get(label, 0) + int(np.sum(in_band))
        matched |= in_band

    result = {"red": 0, "green": 0, "blue": 0, "other": int(np.sum(~matched))}
    for label, count in label_counts.items():
        result[label] = result.get(label, 0) + count
    return {k: result[k] / n for k in result}


def classify_beacon_color(bgr_crop: np.ndarray) -> Tuple[str, float, np.ndarray, float, dict]:
    """
    Given a BGR crop of a beacon bounding box, return:
        (color_name, color_confidence, light_mask, intensity, hue_votes)

    color_confidence: fraction of crop pixels that are "lit" (higher = cleaner read).
    light_mask:       uint8 mask of the bright pixels used (same H×W as crop).
    intensity:        mean brightness (Value channel) of lit pixels, 0.0–1.0.
                      Low intensity on a "blue" result may indicate the beacon is off
                      or too dim to read reliably in daylight.
    hue_votes:        {"red": 0.0–1.0, "green": 0.0–1.0, "blue": 0.0–1.0, "other": 0.0–1.0}
                      Fraction of lit pixels that fall in each color's hue band.
                      Useful for diagnosing mis-classifications and tuning _HUE_BANDS.
    """
    _empty_votes = {"red": 0.0, "green": 0.0, "blue": 0.0, "other": 0.0}
    if bgr_crop is None or bgr_crop.size == 0:
        return "unknown", 0.0, np.zeros((1, 1), dtype=np.uint8), 0.0, _empty_votes

    hsv = cv2.cvtColor(bgr_crop, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]

    # Bright, saturated pixels → the light source
    light_mask = ((s >= _SAT_MIN) & (v >= _VAL_MIN)).astype(np.uint8) * 255

    lit_pixels   = np.count_nonzero(light_mask)
    total_pixels = bgr_crop.shape[0] * bgr_crop.shape[1]
    color_conf   = lit_pixels / max(total_pixels, 1)

    if lit_pixels < 5:
        # Not enough saturated pixels — check for white (high V, low S)
        very_bright = (v >= 220)
        bright_count = int(np.count_nonzero(very_bright))
        if bright_count > total_pixels * 0.1:
            intensity = float(np.mean(v[very_bright]) / 255.0)
            return "white", float(bright_count / total_pixels), light_mask, intensity, _empty_votes
        return "unknown", 0.0, light_mask, 0.0, _empty_votes

    hues      = h[light_mask > 0]
    intensity = float(np.mean(v[light_mask > 0]) / 255.0)

    # Per-pixel hue vote — majority of lit pixels determines the color.
    votes = _hue_votes(hues)

    # Red gets priority: if enough red pixels are present, call it red even if
    # blue background pixels outnumber them slightly. When the beacon is off the
    # housing reads vote_red≈0, so this never fires on a genuinely off beacon.
    if votes["red"] >= _RED_THRESHOLD:
        color = "red"
    else:
        winner = max(("green", "blue"), key=lambda c: votes[c])
        color = winner if votes[winner] >= 0.30 else "unknown"

    return color, color_conf, light_mask, intensity, votes


def isolate_and_classify(beacon_crop: np.ndarray, crop_model,
                         conf: float = 0.3) -> Tuple[str, float, np.ndarray, float, dict]:
    """
    Run best_crop.pt on a beacon bounding-box crop to find the lit area,
    then classify its color.

    Works with both detection models (returns a bbox) and segmentation
    models (returns a pixel mask). Falls back to classify_beacon_color on
    the full crop if the model finds nothing.

    Returns: (color_name, color_confidence, display_mask, intensity, hue_votes)
        display_mask — uint8 mask the same H×W as beacon_crop;
                       255 where the lit area is, 0 elsewhere.
        intensity    — mean brightness (Value) of lit pixels, 0.0–1.0.
        hue_votes    — {"red", "green", "blue", "other"} fraction of lit pixels
                       in each color's hue band; useful for diagnosing results.
    """
    _empty = ("unknown", 0.0, np.zeros((1, 1), dtype=np.uint8), 0.0,
              {"red": 0.0, "green": 0.0, "blue": 0.0, "other": 0.0})
    if beacon_crop is None or beacon_crop.size == 0:
        return _empty

    h, w = beacon_crop.shape[:2]
    display_mask = np.zeros((h, w), dtype=np.uint8)

    results = crop_model(beacon_crop.copy(), conf=conf, verbose=False)
    boxes   = results[0].boxes

    if len(boxes) == 0:
        return classify_beacon_color(beacon_crop)

    if results[0].masks is not None:
        # Segmentation output — use the mask of the top-confidence detection
        best_idx = int(np.argmax([float(b.conf[0]) for b in boxes]))
        seg_mask = results[0].masks.data[best_idx].cpu().numpy()
        seg_resized = cv2.resize(seg_mask, (w, h), interpolation=cv2.INTER_NEAREST)
        display_mask = (seg_resized > 0.5).astype(np.uint8) * 255
    else:
        # Detection output — fill the bbox of the top-confidence detection
        best_box = max(boxes, key=lambda b: float(b.conf[0]))
        lx1, ly1, lx2, ly2 = map(int, best_box.xyxy[0].tolist())
        lx1 = max(0, lx1); ly1 = max(0, ly1)
        lx2 = min(w, lx2); ly2 = min(h, ly2)
        if lx2 > lx1 and ly2 > ly1:
            display_mask[ly1:ly2, lx1:lx2] = 255

    if not display_mask.any():
        return classify_beacon_color(beacon_crop)

    # Tight-crop to the bounding box of the lit mask, then classify color
    rows = np.any(display_mask > 0, axis=1)
    cols = np.any(display_mask > 0, axis=0)
    rmin, rmax = np.where(rows)[0][[0, -1]]
    cmin, cmax = np.where(cols)[0][[0, -1]]
    lit_region = beacon_crop[rmin:rmax + 1, cmin:cmax + 1]

    color, color_conf, _, intensity, votes = classify_beacon_color(lit_region)
    return color, color_conf, display_mask, intensity, votes


# ── Detection logger ──────────────────────────────────────────────────────────

_LOG_HEADER = [
    "timestamp", "frame", "color", "color_confidence", "intensity",
    "vote_red", "vote_green", "vote_blue", "vote_other",
    "det_confidence", "x1", "y1", "x2", "y2", "tracking_id",
    "pos3d_x", "pos3d_y", "pos3d_z",
]


def _open_log(path: str):
    """Open a CSV log file, write the header, and return (file_handle, csv_writer)."""
    fh = open(path, "w", newline="")
    writer = csv.writer(fh)
    writer.writerow(_LOG_HEADER)
    print(f"[beacon] Logging detections → {path}")
    return fh, writer


def _write_log_row(log_writer, frame_idx: int, color: str,
                   color_conf: float, intensity: float, votes: dict,
                   det_conf: float, bbox, tracking_id: int = -1,
                   pos3d=None) -> None:
    x1, y1, x2, y2 = bbox
    px = py = pz = ""
    if pos3d is not None:
        px, py, pz = f"{pos3d[0]:.4f}", f"{pos3d[1]:.4f}", f"{pos3d[2]:.4f}"
    log_writer.writerow([
        f"{time.time():.3f}", frame_idx,
        color, f"{color_conf:.4f}", f"{intensity:.4f}",
        f"{votes.get('red',0):.4f}", f"{votes.get('green',0):.4f}",
        f"{votes.get('blue',0):.4f}", f"{votes.get('other',0):.4f}",
        f"{det_conf:.4f}", x1, y1, x2, y2, tracking_id,
        px, py, pz,
    ])


# ── Helper (used by ROS modes) ────────────────────────────────────────────────

def local_enu_to_gps(world_pos: np.ndarray,
                     origin_lat: float,
                     origin_lon: float,
                     origin_alt: float) -> Tuple[float, float, float]:
    east, north, up = world_pos[0], world_pos[1], world_pos[2]
    dlat = np.degrees(north / EARTH_RADIUS_M)
    dlon = np.degrees(east / (EARTH_RADIUS_M * np.cos(np.radians(origin_lat))))
    return (origin_lat + dlat, origin_lon + dlon, origin_alt + up)


# ── Video test mode (no ROS) ───────────────────────────────────────────────────

_COLOR_BGR = {
    "red":     (0,   0,   255),
    "green":   (0,   200,   0),
    "blue":    (255,  80,   0),
    "white":   (255, 255, 255),
    "unknown": (180, 180, 180),
}


def _annotate_frame(frame: np.ndarray, boxes, names: dict, crop_model,
                    log_writer=None, frame_idx: int = 0,
                    blink_detector: BlinkDetector = None,
                    video_ts: float = None) -> np.ndarray:
    """Draw detections + color classification on a single BGR frame."""
    for box in boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        conf  = float(box.conf[0])
        cls   = int(box.cls[0])
        label = names.get(cls, str(cls))

        crop = frame[max(y1, 0):max(y2, 1), max(x1, 0):max(x2, 1)]
        beacon_color, color_conf, light_mask, intensity, votes = isolate_and_classify(crop, crop_model)
        draw_color = _COLOR_BGR.get(beacon_color, (180, 180, 180))

        blink_info = None
        if blink_detector is not None:
            ts = video_ts if video_ts is not None else time.time()
            blink_info = blink_detector.update(ts, beacon_color, intensity)

        cv2.rectangle(frame, (x1, y1), (x2, y2), draw_color, 2)

        if light_mask is not None and light_mask.any():
            lm_full = np.zeros(frame.shape[:2], dtype=np.uint8)
            lm_h = min(light_mask.shape[0], y2 - y1)
            lm_w = min(light_mask.shape[1], x2 - x1)
            lm_full[y1:y1+lm_h, x1:x1+lm_w] = light_mask[:lm_h, :lm_w]
            contours, _ = cv2.findContours(lm_full, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(frame, contours, -1, draw_color, 2)

        txt = (f"{label} [{beacon_color}] det={conf:.2f} int={intensity:.2f} "
               f"r={votes['red']:.0%} g={votes['green']:.0%} b={votes['blue']:.0%}")
        if blink_info:
            if blink_info["is_blinking"]:
                txt += f" blink={blink_info['blink_hz']:.2f}Hz"
            elif blink_info["is_blinking"] is None:
                txt += " blink=?"
        cv2.putText(frame, txt, (x1, max(y1 - 6, 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, draw_color, 1)

        print(f"  {txt}  bbox=({x1},{y1},{x2},{y2})")

        if log_writer is not None:
            _write_log_row(log_writer, frame_idx, beacon_color, color_conf,
                           intensity, votes, conf, (x1, y1, x2, y2))

    return frame


def run_video(video_path: str,
              model_path: str = "models/one_beacon.pt",
              crop_model_path: str = "models/best_crop.pt",
              save_output: bool = False,
              conf: float = 0.5,
              log: bool = False) -> None:
    """
    Run beacon detection + color classification on a local video file.
    No ROS required — uses ultralytics.YOLO directly.

    Press 'q' to quit, SPACE to pause/resume.
    """
    try:
        from ultralytics import YOLO
    except ImportError:
        print("[beacon-video] ultralytics not installed: pip install ultralytics")
        return

    if not os.path.exists(model_path):
        print(f"[beacon-video] Model not found: {model_path}")
        return

    if not os.path.exists(crop_model_path):
        print(f"[beacon-video] Crop model not found: {crop_model_path}")
        return

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"[beacon-video] Cannot open video: {video_path}")
        return

    model       = YOLO(model_path)
    crop_model  = YOLO(crop_model_path)
    fps    = cap.get(cv2.CAP_PROP_FPS) or 30.0
    width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total  = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"[beacon-video] {video_path}  {width}x{height} @ {fps:.1f}fps  {total} frames")
    print(f"[beacon-video] Beacon model : {model_path}  conf≥{conf}")
    print(f"[beacon-video] Crop model   : {crop_model_path}")
    print("[beacon-video] Press 'q' to quit, SPACE to pause")

    writer = None
    if save_output:
        out_path = os.path.splitext(video_path)[0] + "_beacon_out.mp4"
        fourcc   = cv2.VideoWriter_fourcc(*"mp4v")
        writer   = cv2.VideoWriter(out_path, fourcc, fps, (width, height))
        print(f"[beacon-video] Saving output → {out_path}")

    log_fh = log_writer = None
    if log:
        ts = time.strftime("%Y%m%d_%H%M%S")
        log_path = os.path.splitext(video_path)[0] + f"_beacon_log_{ts}.csv"
        log_fh, log_writer = _open_log(log_path)

    blink_detector = BlinkDetector()
    frame_idx     = 0
    paused        = False
    display_frame = None

    cv2.namedWindow("Beacon Detector — Video Test", cv2.WINDOW_AUTOSIZE)

    try:
        while True:
            if not paused:
                ret, raw = cap.read()
                if not ret:
                    print("[beacon-video] End of video")
                    break
                frame_idx += 1

                display_frame = raw.copy()           # safe copy — YOLO never touches this
                video_ts = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
                results = model(raw, conf=conf, verbose=False)
                boxes   = results[0].boxes
                names   = results[0].names

                print(f"Frame {frame_idx}/{total} — {len(boxes)} detection(s)")
                display_frame = _annotate_frame(display_frame, boxes, names, crop_model,
                                                log_writer=log_writer, frame_idx=frame_idx,
                                                blink_detector=blink_detector,
                                                video_ts=video_ts)

                if writer:
                    writer.write(display_frame)

            if display_frame is not None:
                cv2.imshow("Beacon Detector — Video Test", display_frame)
            key = cv2.waitKey(10 if not paused else 50) & 0xFF
            if key == ord("q"):
                break
            elif key == ord(" "):
                paused = not paused

    except KeyboardInterrupt:
        print("\n[beacon-video] Interrupted")
    finally:
        cap.release()
        if writer:
            writer.release()
        if log_fh:
            log_fh.close()
        cv2.destroyAllWindows()
        print(f"[beacon-video] Done — {frame_idx} frames processed")


# ── Video + ROS mode ──────────────────────────────────────────────────────────

def run_video_ros(video_path: str,
                  model_path: str = "models/one_beacon.pt",
                  crop_model_path: str = "models/best_crop.pt",
                  save_output: bool = False,
                  conf: float = 0.5,
                  log: bool = False,
                  display: bool = False) -> None:
    """
    Read frames from a local video file and publish detections to ROS.

    Initializes a ROS2 node for:
      - Publishing to /seabird/beacon_detections
      - Receiving drone pose from /mavros/local_position/pose
      - Receiving GPS origin from /mavros/global_position/gp_origin

    Frames come from cv2.VideoCapture — no camera topic subscriptions needed.
    Pass display=True to open a live OpenCV window.
    When display is on: press 'q' to quit, SPACE to pause/resume.
    """
    _import_ros()

    try:
        from ultralytics import YOLO
    except ImportError:
        print("[beacon-ros-video] ultralytics not installed: pip install ultralytics")
        return

    if not os.path.exists(model_path):
        print(f"[beacon-ros-video] Model not found: {model_path}")
        return

    if not os.path.exists(crop_model_path):
        print(f"[beacon-ros-video] Crop model not found: {crop_model_path}")
        return

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"[beacon-ros-video] Cannot open video: {video_path}")
        return

    fps    = cap.get(cv2.CAP_PROP_FPS) or 30.0
    width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total  = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"[beacon-ros-video] {video_path}  {width}x{height} @ {fps:.1f}fps  {total} frames")
    print(f"[beacon-ros-video] Model: {model_path}  conf≥{conf}")
    print("[beacon-ros-video] Publishing → /seabird/beacon_detections")
    if display:
        print("[beacon-ros-video] Press 'q' to quit, SPACE to pause")

    writer = None
    if save_output:
        out_path = os.path.splitext(video_path)[0] + "_beacon_ros_out.mp4"
        fourcc   = cv2.VideoWriter_fourcc(*"mp4v")
        writer   = cv2.VideoWriter(out_path, fourcc, fps, (width, height))
        print(f"[beacon-ros-video] Saving output → {out_path}")

    log_fh = log_writer = None
    if log:
        ts = time.strftime("%Y%m%d_%H%M%S")
        log_path = os.path.splitext(video_path)[0] + f"_beacon_log_{ts}.csv"
        log_fh, log_writer = _open_log(log_path)

    blink_detector = BlinkDetector()
    rclpy.init()
    cam        = BeaconCamera()
    model      = YOLO(model_path)
    crop_model = YOLO(crop_model_path)
    print(f"[beacon-ros-video] Beacon model : {model_path}  conf≥{conf}")
    print(f"[beacon-ros-video] Crop model   : {crop_model_path}")

    if not cam.open_for_video():
        print("[beacon-ros-video] Failed to open ROS node")
        rclpy.shutdown()
        cap.release()
        return

    frame_idx     = 0
    paused        = False
    display_frame = None

    if display:
        cv2.namedWindow("Beacon Detector — Video + ROS", cv2.WINDOW_AUTOSIZE)

    try:
        while rclpy.ok():
            # Spin once to receive latest pose / GPS
            rclpy.spin_once(cam, timeout_sec=0.0)

            if not paused:
                ret, raw = cap.read()
                if not ret:
                    print("[beacon-ros-video] End of video")
                    break
                frame_idx += 1

                drone_pos, _ = cam.get_drone_pose()
                display_frame = raw.copy()           # safe copy — YOLO never touches this
                video_ts = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0

                results = model(raw, conf=conf, verbose=False)
                boxes   = results[0].boxes

                print(f"Frame {frame_idx}/{total} — {len(boxes)} detection(s)")

                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    det_conf = float(box.conf[0])

                    crop = display_frame[max(y1, 0):max(y2, 1), max(x1, 0):max(x2, 1)]
                    beacon_color, color_conf, light_mask, intensity, votes = isolate_and_classify(crop, crop_model)
                    draw_color = _COLOR_BGR.get(beacon_color, (180, 180, 180))

                    blink_info = blink_detector.update(video_ts, beacon_color, intensity)

                    cv2.rectangle(display_frame, (x1, y1), (x2, y2), draw_color, 2)

                    if light_mask is not None and light_mask.any():
                        lm_full = np.zeros(display_frame.shape[:2], dtype=np.uint8)
                        lm_h = min(light_mask.shape[0], y2 - y1)
                        lm_w = min(light_mask.shape[1], x2 - x1)
                        lm_full[y1:y1+lm_h, x1:x1+lm_w] = light_mask[:lm_h, :lm_w]
                        contours, _ = cv2.findContours(lm_full, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        cv2.drawContours(display_frame, contours, -1, draw_color, 2)

                    label_txt = (
                        f"beacon [{beacon_color}] det={det_conf:.2f} int={intensity:.2f} "
                        f"r={votes['red']:.0%} g={votes['green']:.0%} b={votes['blue']:.0%}"
                    )
                    if blink_info["is_blinking"]:
                        label_txt += f" blink={blink_info['blink_hz']:.2f}Hz"
                    elif blink_info["is_blinking"] is None:
                        label_txt += " blink=?"
                    cv2.putText(display_frame, label_txt, (x1, max(y1 - 6, 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, draw_color, 1)

                    # Publish to ROS
                    msg = String()
                    msg.data = json.dumps({
                        "color":            beacon_color,
                        "blink":            blink_info,
                        "label":            "beacon",
                        "color_confidence": color_conf,
                        "intensity":        intensity,
                        "hue_votes":        votes,
                        "confidence":       det_conf,
                        "bbox":             [x1, y1, x2, y2],
                        "position_3d":      None,
                        "world_position":   None,
                        "gps_position":     None,
                        "drone_position":   drone_pos.tolist() if drone_pos is not None else None,
                        "tracking_id":      -1,
                        "timestamp":        time.time(),
                    })
                    cam.detection_pub.publish(msg)
                    print(f"  {label_txt}")

                    if log_writer is not None:
                        _write_log_row(log_writer, frame_idx, beacon_color, color_conf,
                                       intensity, votes, det_conf, (x1, y1, x2, y2))

                if writer:
                    writer.write(display_frame)

            if display:
                if display_frame is not None:
                    cv2.imshow("Beacon Detector — Video + ROS", display_frame)
                key = cv2.waitKey(10 if not paused else 50) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord(" "):
                    paused = not paused

    except KeyboardInterrupt:
        print("\n[beacon-ros-video] Interrupted")
    finally:
        cap.release()
        if writer:
            writer.release()
        if log_fh:
            log_fh.close()
        cam.close()
        if display:
            cv2.destroyAllWindows()
        rclpy.shutdown()
        print(f"[beacon-ros-video] Done — {frame_idx} frames processed")


# ── Main loop (ROS mode) ───────────────────────────────────────────────────────

def main(model: str = "models/one_beacon.pt",
         display: bool = False,
         true_distance: float = 0.4826,
         crop_model_path: str = "models/best_crop.pt",
         log: bool = False) -> None:

    _import_ros()   # pull in ROS2 / camera_interface / seabird_config

    DEBUG_DIR   = os.path.expanduser("~/seabird_dataset/beacon_debug")
    SAVE_EVERY_N = 30
    os.makedirs(DEBUG_DIR, exist_ok=True)

    rclpy.init()
    cam = BeaconCamera()

    if not cam.open():
        print("[beacon] Failed to open camera")
        rclpy.shutdown()
        return

    if not os.path.exists(model):
        print(f"[beacon] Model not found: {model}")
        cam.close()
        rclpy.shutdown()
        return

    if not os.path.exists(crop_model_path):
        print(f"[beacon] Crop model not found: {crop_model_path}")
        cam.close()
        rclpy.shutdown()
        return

    from ultralytics import YOLO
    print(f"[beacon] Loading model: {model}")
    print(f"[beacon] Loading crop model: {crop_model_path}")
    crop_model = YOLO(crop_model_path)
    if not cam.enable_detection(model):
        print("[beacon] Detection failed to start")
        cam.close()
        rclpy.shutdown()
        return

    print("[beacon] Detection ENABLED — class: beacon (color determined by CV)")
    print("[beacon] Publishing → /seabird/beacon_detections")

    if display:
        cv2.namedWindow("Beacon Detector", cv2.WINDOW_AUTOSIZE)

    log_fh = log_writer = None
    if log:
        ts = time.strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(DEBUG_DIR, f"beacon_log_{ts}.csv")
        log_fh, log_writer = _open_log(log_path)

    frame_count       = 0
    intrinsics_printed = False

    try:
        while rclpy.ok():
            if not cam.grab():
                continue

            frame_count += 1
            rgb        = cam.get_rgb()
            depth      = cam.get_depth()
            intr       = cam._intrinsics
            drone_pos, drone_quat = cam.get_drone_pose()

            if intr and not intrinsics_printed:
                print(f"[beacon] Intrinsics ready: {intr.width}x{intr.height} "
                      f"fx={intr.fx:.1f} fy={intr.fy:.1f}")
                intrinsics_printed = True

            if rgb is None:
                continue

            dets = cam.get_detections()

            for d in dets:
                x1, y1, x2, y2 = d.bbox_2d

                # ── Color determination from the beacon's light area ──────────
                crop = rgb[max(y1, 0):max(y2, 1), max(x1, 0):max(x2, 1)]
                beacon_color, color_conf, light_mask, intensity, votes = isolate_and_classify(crop, crop_model)
                blink_info = _get_blink_detector(d.tracking_id).update(
                    time.time(), beacon_color, intensity
                )
                # ─────────────────────────────────────────────────────────────

                draw_color = _COLOR_BGR.get(beacon_color, (180, 180, 180))

                cv2.rectangle(rgb, (x1, y1), (x2, y2), draw_color, 2)

                if light_mask is not None and light_mask.any():
                    lm_full = np.zeros(rgb.shape[:2], dtype=np.uint8)
                    lm_h = min(light_mask.shape[0], y2 - y1)
                    lm_w = min(light_mask.shape[1], x2 - x1)
                    lm_full[y1:y1+lm_h, x1:x1+lm_w] = light_mask[:lm_h, :lm_w]
                    contours, _ = cv2.findContours(lm_full, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    cv2.drawContours(rgb, contours, -1, draw_color, 2)

                label_txt = (
                    f"beacon [{beacon_color}] conf={d.confidence:.2f} int={intensity:.2f} "
                    f"r={votes['red']:.0%} g={votes['green']:.0%} b={votes['blue']:.0%}"
                )
                if blink_info["is_blinking"]:
                    label_txt += f" blink={blink_info['blink_hz']:.2f}Hz"
                elif blink_info["is_blinking"] is None:
                    label_txt += " blink=?"
                if d.tracking_id >= 0:
                    label_txt += f" #{d.tracking_id}"

                cv2.putText(rgb, label_txt, (x1, y1 - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, draw_color, 1)

                # ── World-frame position (if pose available) ─────────────────
                world_pos  = None
                gps_coords = None
                if d.position_3d is not None and drone_pos is not None:
                    world_pos = camera_to_world(d.position_3d, drone_pos, drone_quat)
                    origin    = cam.get_gps_origin()
                    if origin is not None:
                        lat, lon, alt = local_enu_to_gps(world_pos, *origin)
                        gps_coords    = {"latitude": lat, "longitude": lon, "altitude": alt}

                    cv2.putText(
                        rgb,
                        f"W({world_pos[0]:.1f},{world_pos[1]:.1f},{world_pos[2]:.1f})",
                        (x1, y2 + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, draw_color, 1,
                    )

                # ── Publish ──────────────────────────────────────────────────
                msg = String()
                msg.data = json.dumps({
                    "color":            beacon_color,
                    "blink":            blink_info,
                    "label":            "beacon",
                    "color_confidence": color_conf,
                    "intensity":        intensity,
                    "hue_votes":        votes,
                    "confidence":       float(d.confidence),
                    "bbox":           list(d.bbox_2d),
                    "position_3d":    d.position_3d.tolist() if d.position_3d is not None else None,
                    "world_position": world_pos.tolist()     if world_pos     is not None else None,
                    "gps_position":   gps_coords,
                    "drone_position": drone_pos.tolist()     if drone_pos     is not None else None,
                    "tracking_id":    d.tracking_id,
                    "timestamp":      time.time(),
                })
                cam.detection_pub.publish(msg)

                if log_writer is not None:
                    _write_log_row(log_writer, frame_count, beacon_color, color_conf,
                                   intensity, votes, float(d.confidence), d.bbox_2d,
                                   tracking_id=d.tracking_id,
                                   pos3d=d.position_3d)

                print(f"[beacon] {label_txt}"
                      + (f" pos3d=({d.position_3d[2]:.2f}m)" if d.position_3d is not None else ""))

            # ── Display ───────────────────────────────────────────────────────
            if display and rgb is not None:
                cv2.imshow("Beacon Detector", rgb)
                if cv2.waitKey(10) & 0xFF == ord("q"):
                    break

            # ── Periodic save ─────────────────────────────────────────────────
            if frame_count % SAVE_EVERY_N == 0 and rgb is not None:
                out_path = os.path.join(DEBUG_DIR, f"frame_{frame_count:06d}.png")
                cv2.imwrite(out_path, rgb)
                print(f"[beacon] Saved {out_path}")

    except KeyboardInterrupt:
        print("\n[beacon] Interrupted")
    finally:
        if log_fh:
            log_fh.close()
        cam.close()
        cv2.destroyAllWindows()
        rclpy.shutdown()
        print(f"[beacon] Done — {frame_count} frames processed")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="beacon_detector",
        description="Beacon detector with CV-based color classification",
    )
    parser.add_argument(
        "--model", "-m",
        default="models/one_beacon.pt",
        type=str,
        help="Path to YOLO beacon model (default: models/one_beacon.pt)",
    )
    parser.add_argument("--display", "-d", action="store_true", help="Show live cv2 window (ROS mode)")
    parser.add_argument(
        "--true_dist", "-td",
        type=float,
        default=0.4826,
        help="Known ground-truth distance to object in meters (ROS mode)",
    )
    parser.add_argument(
        "--video", "-v",
        default=None,
        type=str,
        metavar="VIDEO_PATH",
        help="Test on a local video file — no ROS, pure CV only",
    )
    parser.add_argument(
        "--ros-video", "-rv",
        default=None,
        type=str,
        metavar="VIDEO_PATH",
        help="Read frames from a video file AND publish detections to ROS topics",
    )
    parser.add_argument(
        "--save", "-s",
        action="store_true",
        help="Save annotated output video alongside the input (video modes only)",
    )
    parser.add_argument(
        "--conf", "-c",
        type=float,
        default=0.5,
        help="Detection confidence threshold (video modes, default 0.5)",
    )
    parser.add_argument(
        "--crop-model", "-cm",
        default="models/best_crop.pt",
        type=str,
        metavar="CROP_MODEL_PATH",
        help="Path to YOLO lit-area isolation model (default: models/best_crop.pt)",
    )
    parser.add_argument(
        "--log", "-l",
        action="store_true",
        help="Save a CSV detection log (color, intensity, confidence, bbox) for each run",
    )
    args = parser.parse_args()

    if args.ros_video is not None:
        run_video_ros(args.ros_video, model_path=args.model, crop_model_path=args.crop_model,
                      save_output=args.save, conf=args.conf, log=args.log,
                      display=args.display)
    elif args.video is not None:
        run_video(args.video, model_path=args.model, crop_model_path=args.crop_model,
                  save_output=args.save, conf=args.conf, log=args.log)
    else:
        main(args.model, args.display, args.true_dist, crop_model_path=args.crop_model,
             log=args.log)
