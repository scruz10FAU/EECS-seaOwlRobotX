#!/usr/bin/env python3
"""
burst_beacon_detector.py — Burst-collection beacon detector.

Waits for the first beacon detection, then collects one frame every
BURST_INTERVAL seconds for a total of BURST_COUNT frames (default:
8 frames × 0.5 s = 4 s of data).  After collection the full color-
classification and blink-detection pipeline runs on all stored frames
and a single consolidated result is published / printed.

State machine:  searching → collecting (8 frames) → analysis → searching

Modes (selected by config key):
  ros_video  — video file + ROS publishing   (key "ros_video": "/path/to/file")
  video      — video file, console only       (key "video": "/path/to/file")
  (neither)  — live ROS camera               (default)

Usage:
    python3 burst_beacon_detector.py --config configs/modal_config_fred.json
"""

import os
import sys
import json
import time
import argparse
from collections import namedtuple

import cv2

from beacon_detector_config import (
    load_config,
    _import_ros,
    _make_beacon_camera,
    _apply_color_config,
    isolate_and_classify,
    trace_vote_mask,
    estimate_distance_from_bbox,
    _camera_to_world,
    local_enu_to_gps,
    gps_ground_truth_distance,
    _open_log,
    _write_log_row,
    _DEFAULT_CONFIG,
)
from blink_detector import BlinkDetector

# ── Burst parameters (can be overridden via config key "burst") ───────────────
_BURST_INTERVAL = 0.5   # seconds between captured frames
_BURST_COUNT    = 8     # frames per burst  →  4 s of data

# Lightweight stand-in for live detection objects when processing video frames.
# bbox_2d: [x1, y1, x2, y2], position_3d: None (no depth in video mode),
# confidence: float, tracking_id: int (frame-local index; no real tracker)
_VideoDet = namedtuple("_VideoDet", ["bbox_2d", "position_3d", "confidence", "tracking_id"])


# ── Shared analysis ───────────────────────────────────────────────────────────

def _analyse_burst(burst, crop_model, cfg, depth_source,
                   save_crops_dir=None, det_images_dir=None, color_pixels_dir=None,
                   target_color=None, target_blinking=None,
                   log_writer=None, burst_number=None,
                   get_gps_origin_fn=None):
    """
    Run color classification and blink detection over a collected burst.

    burst: list of (frame_ts, rgb_clean, dets, depth, drone_pos, drone_quat)
    Returns last_valid dict (empty dict if no valid detection found).
    """
    blink_detector = BlinkDetector(use_variance=cfg.get("detection", {}).get("use_variance_mode", False))
    last_valid: dict = {}
    date_tag = time.strftime("%Y%m%d")

    gps_gt_cfg = cfg.get("gps_ground_truth", {})
    obj_height_agl = cfg.get("detection", {}).get("beacon_z_m", 0.0)

    for frame_idx, (b_ts, b_rgb, b_dets, _, b_dpos, b_dquat) in enumerate(burst):
        if not b_dets:
            continue
        for det_idx, d in enumerate(b_dets):
            x1, y1, x2, y2 = [int(v) for v in d.bbox_2d]

            pos3d = d.position_3d
            if pos3d is None and depth_source == "bbox":
                det_cfg = cfg["detection"]
                pos3d = estimate_distance_from_bbox(
                    x1, y1, x2, y2,
                    cfg["camera"]["fx"], cfg["camera"]["fy"],
                    cfg["camera"]["cx"], cfg["camera"]["cy"],
                    b_dpos[2] if b_dpos is not None else 0.0,
                    det_cfg.get("beacon_z_m", 0.0),
                    det_cfg.get("beacon_height_m", 0.3048),
                )

            crop = b_rgb[max(y1, 0):max(y2, 1), max(x1, 0):max(x2, 1)]
            (beacon_color, color_conf, _, intensity, votes,
             lit_region, hue_var, hue_mean, hue_median, hue_mode, vote_mask) = \
                isolate_and_classify(crop, crop_model)
            ts_after_classify = time.time()

            if beacon_color == "no_top":
                (beacon_color, color_conf, _, intensity, votes,
                    lit_region, hue_var, hue_mean, hue_median, hue_mode, vote_mask) = \
                    isolate_and_classify(b_rgb, crop_model)
                ts_after_classify = time.time()


            if det_images_dir is not None and beacon_color == "no_top":
                pad = 20
                h_img, w_img = b_rgb.shape[:2]
                ix1 = max(x1 - pad, 0)
                iy1 = max(y1 - pad, 0)
                ix2 = min(x2 + pad, w_img)
                iy2 = min(y2 + pad, h_img)
                det_crop = b_rgb[iy1:iy2, ix1:ix2].copy()
                _gt = (f"gt-{target_color or 'unk'}-"
                       f"{'blink' if target_blinking is True else 'steady' if target_blinking is False else 'unk'}")
                _bn = f"b{burst_number:03d}_" if burst_number is not None else ""
                fname = (f"det_{date_tag}_{_bn}f{frame_idx:06d}_t{int(d.tracking_id):02d}_notop"
                         f"_conf{int(d.confidence*100):02d}_det-notop_{_gt}.png")
                cv2.imwrite(os.path.join(det_images_dir, fname), det_crop)

            if beacon_color == "no_top":
                print(f"[burst]   ts={b_ts:.2f}: no top — skipped")
                continue

            blink_info = blink_detector.update(b_ts, beacon_color, intensity, color_conf,
                                               hue_variance=hue_var)
            ts_after_blink = time.time()
            print(f"[burst]   ts={b_ts:.2f}  color={beacon_color:7s} "
                  f"conf={color_conf:.2f}  "
                  f"blink_color={blink_info['blink_color']}  "
                  f"is_blinking={blink_info['is_blinking']}")

            gps_gt_info = None
            if gps_gt_cfg.get("enabled") and get_gps_origin_fn is not None and b_dpos is not None:
                origin = get_gps_origin_fn()
                if origin is not None:
                    drone_lat, drone_lon, _ = origin
                    drone_height_agl = b_dpos[2]
                    dist, horiz, vert = gps_ground_truth_distance(
                        drone_lat, drone_lon, drone_height_agl,
                        gps_gt_cfg["latitude"], gps_gt_cfg["longitude"], obj_height_agl,
                    )
                    gps_gt_info = (dist, horiz, vert,
                                   gps_gt_cfg["latitude"], gps_gt_cfg["longitude"], drone_height_agl)

            last_valid = dict(
                beacon_color=beacon_color, color_conf=color_conf,
                intensity=intensity, votes=votes,
                hue_var=hue_var, hue_mean=hue_mean,
                hue_median=hue_median, hue_mode=hue_mode,
                blink_info=blink_info, det_conf=float(d.confidence),
                bbox=d.bbox_2d, pos3d=pos3d,
                drone_pos=b_dpos, drone_quat=b_dquat,
                frame_ts=b_ts, lit_region=lit_region,
                tracking_id=int(d.tracking_id),
                img_w=b_rgb.shape[1], img_h=b_rgb.shape[0],
                gps_gt=gps_gt_info,
            )

            if log_writer is not None:
                _write_log_row(
                    log_writer, frame_idx,
                    beacon_color, color_conf,
                    intensity, votes,
                    float(d.confidence), d.bbox_2d,
                    tracking_id=int(d.tracking_id),
                    pos3d=pos3d,
                    blink_info=blink_info,
                    target_color=target_color,
                    target_blinking=target_blinking,
                    gps_gt_info=gps_gt_info,
                    img_w=b_rgb.shape[1], img_h=b_rgb.shape[0],
                    hue_variance=hue_var,
                    hue_mean=hue_mean,
                    hue_median=hue_median,
                    hue_mode=hue_mode,
                    frame_ts=b_ts,
                    ts_after_classify=ts_after_classify,
                    ts_after_blink=ts_after_blink,
                    burst_number=burst_number,
                )

            _gt  = (f"gt-{target_color or 'unk'}-"
                    f"{'blink' if target_blinking is True else 'steady' if target_blinking is False else 'unk'}")
            _b   = blink_info.get("is_blinking")
            _det = (f"det-blink{blink_info.get('blink_hz', 0):.2f}hz" if _b is True
                    else "det-steady" if _b is False else "det-acc")
            _bn  = f"b{burst_number:03d}_" if burst_number is not None else ""

            if save_crops_dir is not None and lit_region.size > 0:
                fname = (f"crop_{date_tag}_{_bn}f{frame_idx:06d}_d{det_idx:02d}_{beacon_color}"
                         f"_{_det}_r{int(votes['red']*100)}g{int(votes['green']*100)}b{int(votes['blue']*100)}"
                         f"_{_gt}.png")
                cv2.imwrite(os.path.join(save_crops_dir, fname), trace_vote_mask(lit_region, vote_mask))

            if color_pixels_dir is not None and lit_region.size > 0 and vote_mask is not None:
                fname = (f"pixels_{date_tag}_{_bn}f{frame_idx:06d}_d{det_idx:02d}_{beacon_color}"
                         f"_r{int(votes['red']*100)}g{int(votes['green']*100)}b{int(votes['blue']*100)}"
                         f"_{_gt}.png")
                color_pixels = cv2.bitwise_and(lit_region, lit_region, mask=vote_mask)
                cv2.imwrite(os.path.join(color_pixels_dir, fname), color_pixels)

            if det_images_dir is not None:
                pad = 20
                h_img, w_img = b_rgb.shape[:2]
                ix1 = max(x1 - pad, 0)
                iy1 = max(y1 - pad, 0)
                ix2 = min(x2 + pad, w_img)
                iy2 = min(y2 + pad, h_img)
                det_crop = b_rgb[iy1:iy2, ix1:ix2].copy()
                fname = (f"det_{date_tag}_{_bn}f{frame_idx:06d}_t{int(d.tracking_id):02d}_{beacon_color}"
                         f"_conf{int(d.confidence*100):02d}_{_det}_{_gt}.png")
                cv2.imwrite(os.path.join(det_images_dir, fname), det_crop)

    # All burst frames fed — finalise so timing guards don't force None.
    if last_valid:
        blink_detector.finalise()
        last_valid["blink_info"] = blink_detector._estimate()

    return last_valid


def _publish_result(lv, burst, burst_count, cfg, get_gps_origin_fn, publish_fn):
    """
    Compute world/GPS coords, print and optionally publish the burst result.

    publish_fn: callable(json_str) or None for console-only mode.
    get_gps_origin_fn: callable() -> (lat, lon, alt) or None.
    """
    world_pos  = None
    gps_coords = None
    if lv["pos3d"] is not None and lv["drone_pos"] is not None:
        world_pos = _camera_to_world(
            lv["pos3d"], lv["drone_pos"], lv["drone_quat"],
            cfg["camera"]["_mount_offset"],
            cfg["camera"]["_R_body_to_cam"],
        )
        origin = get_gps_origin_fn()
        if origin is not None:
            lat, lon, alt = local_enu_to_gps(world_pos, *origin)
            gps_coords = {"latitude": lat, "longitude": lon, "altitude": alt}

    bi = lv["blink_info"]
    gps_gt = lv.get("gps_gt")
    print(f"[burst] ── Burst #{burst_count} result ─────────────")
    print(f"[burst]   color={lv['beacon_color']}  "
          f"is_blinking={bi['is_blinking']}  "
          f"blink_color={bi['blink_color']}  "
          f"blink_hz={bi['blink_hz']}")
    if gps_gt is not None:
        print(f"[burst]   gps_ground_truth: dist={gps_gt[0]:.2f}m "
              f"(horiz={gps_gt[1]:.2f}m vert={gps_gt[2]:.2f}m)")

    payload = {
        "color":            lv["beacon_color"],
        "blink":            bi,
        "label":            "beacon",
        "color_confidence": round(lv["color_conf"], 4),
        "intensity":        round(lv["intensity"], 4),
        "hue_votes":        {k: round(v, 4) for k, v in lv["votes"].items()},
        "confidence":       round(lv["det_conf"], 4),
        "bbox":             [int(v) for v in lv["bbox"]],
        "position_3d":      [round(v, 4) for v in lv["pos3d"]]
                            if lv["pos3d"] is not None else None,
        "world_position":   [round(v, 4) for v in world_pos.tolist()]
                            if world_pos is not None else None,
        "gps_position":     gps_coords,
        "drone_position":   lv["drone_pos"].tolist()
                            if lv["drone_pos"] is not None else None,
        "gps_ground_truth": {
            "distance_m":   round(gps_gt[0], 4),
            "horizontal_m": round(gps_gt[1], 4),
            "vertical_m":   round(gps_gt[2], 4),
        } if gps_gt is not None else None,
        "tracking_id":      lv["tracking_id"],
        "burst_frames":     len(burst),
        "timestamp":        round(lv["frame_ts"], 3),
    }
    if publish_fn is not None:
        publish_fn(json.dumps(payload))


# ── Live ROS camera mode ──────────────────────────────────────────────────────

def run_burst_ros(cfg: dict) -> None:
    burst_cfg = cfg.get("burst", {})
    interval  = burst_cfg.get("interval_sec", _BURST_INTERVAL)
    count     = burst_cfg.get("frame_count",  _BURST_COUNT)

    model_path      = cfg["model"]
    crop_model_path = cfg["crop_model"]
    display         = cfg["display"]
    log             = cfg["log"]
    topics          = cfg["topics"]
    save_crops      = cfg.get("save_crops", False)
    save_det_images = cfg.get("save_det_images", False)
    save_color_pixels = cfg.get("save_color_pixels", False)
    target_color    = cfg.get("target_color")
    target_blinking = cfg.get("target_blinking")

    _import_ros()
    import rclpy
    from std_msgs.msg import String
    from ultralytics import YOLO

    DEBUG_DIR = os.path.expanduser("seabird_dataset/beacon_debug")
    os.makedirs(DEBUG_DIR, exist_ok=True)

    rclpy.init()
    cam = _make_beacon_camera(topics, cfg["camera"], cfg["detection"])

    if not cam.open():
        print("[burst] Failed to open camera")
        try: rclpy.shutdown()
        except Exception: pass
        return

    for label, path in [("Model", model_path), ("Crop model", crop_model_path)]:
        if not os.path.exists(path):
            print(f"[burst] {label} not found: {path}")
            cam.close()
            try: rclpy.shutdown()
            except Exception: pass
            return

    crop_model = YOLO(crop_model_path)
    if not cam.enable_detection(model_path, imgsz=cfg["detection"].get("imgsz", 640)):
        print("[burst] Detection failed to start")
        cam.close()
        try: rclpy.shutdown()
        except Exception: pass
        return

    print(f"[burst] Live ROS mode — waiting for beacon to trigger {count}-frame burst")
    print(f"[burst]   interval={interval}s  total={interval*count:.1f}s")
    print(f"[burst] Publishing → {topics['detections_pub']}")

    gps_gt_cfg = cfg.get("gps_ground_truth", {})
    if gps_gt_cfg.get("enabled"):
        print(f"[burst] GPS ground truth enabled  "
              f"target=({gps_gt_cfg['latitude']:.7f}, {gps_gt_cfg['longitude']:.7f})  "
              f"object_height_agl={cfg['detection'].get('beacon_z_m', 0.0):.2f}m")

    log_fh = log_writer = None
    if log:
        ts_tag   = time.strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(DEBUG_DIR, f"burst_log_{ts_tag}.csv")
        log_fh, log_writer = _open_log(log_path)

    crops_dir = None
    if save_crops:
        crops_dir = os.path.join(DEBUG_DIR, "beacon_crops")
        os.makedirs(crops_dir, exist_ok=True)
        print(f"[burst] Saving crops → {crops_dir}/")

    det_images_dir = None
    if save_det_images:
        det_images_dir = os.path.join(DEBUG_DIR, "beacon_det_images")
        os.makedirs(det_images_dir, exist_ok=True)
        print(f"[burst] Saving detection images → {det_images_dir}/")

    color_pixels_dir = None
    if save_color_pixels:
        color_pixels_dir = os.path.join(DEBUG_DIR, "beacon_color_pixels")
        os.makedirs(color_pixels_dir, exist_ok=True)
        print(f"[burst] Saving color-vote pixels → {color_pixels_dir}/")

    if display:
        cv2.namedWindow("Burst Detector", cv2.WINDOW_NORMAL)

    _apply_color_config(cfg["detection"])
    depth_source = cfg["detection"].get("depth_source", "topic")
    burst_count  = 0

    state         = "searching"
    burst: list   = []
    last_burst_ts = -999.0

    def _publish(json_str):
        msg      = String()
        msg.data = json_str
        cam.detection_pub.publish(msg)

    try:
        while rclpy.ok():
            if not cam.grab():
                continue

            rgb = cam.get_rgb()
            if rgb is None:
                continue

            rgb_clean             = rgb.copy()
            depth                 = cam.get_depth()
            drone_pos, drone_quat = cam.get_drone_pose()
            frame_ts              = cam.get_frame_timestamp() or time.time()
            dets                  = cam.get_detections()

            if state == "searching":
                if dets:
                    print(f"[burst] Beacon detected — starting collection "
                          f"({count} frames × {interval}s)")
                    state         = "collecting"
                    burst         = [(frame_ts, rgb_clean, dets, depth, drone_pos, drone_quat)]
                    last_burst_ts = frame_ts
                    print(f"[burst]   1/{count}")

            elif state == "collecting":
                if frame_ts - last_burst_ts >= interval:
                    burst.append((frame_ts, rgb_clean, dets, depth, drone_pos, drone_quat))
                    last_burst_ts = frame_ts
                    print(f"[burst]   {len(burst)}/{count}")

                if len(burst) >= count:
                    print("[burst] Collection complete — analysing")
                    burst_count += 1
                    lv = _analyse_burst(burst, crop_model, cfg, depth_source,
                                        save_crops_dir=crops_dir,
                                        det_images_dir=det_images_dir,
                                        color_pixels_dir=color_pixels_dir,
                                        target_color=target_color,
                                        target_blinking=target_blinking,
                                        log_writer=log_writer,
                                        burst_number=burst_count,
                                        get_gps_origin_fn=cam.get_gps_origin)
                    if not lv:
                        print("[burst] No valid detections in burst")
                    else:
                        _publish_result(lv, burst, burst_count, cfg,
                                        cam.get_gps_origin, _publish)
                    burst = []
                    state = "searching"
                    print("[burst] Resuming search")

            if display:
                overlay = rgb.copy()
                if state == "collecting":
                    label_txt = f"COLLECTING  {len(burst)}/{count}"
                    color_bgr = (0, 255, 255)
                else:
                    label_txt = "SEARCHING"
                    color_bgr = (180, 180, 180)
                cv2.putText(overlay, label_txt, (10, 36),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, color_bgr, 2)
                cv2.imshow("Burst Detector", overlay)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except KeyboardInterrupt:
        print("\n[burst] Interrupted")
    finally:
        if log_fh:
            log_fh.close()
        cam.close()
        if display:
            cv2.destroyAllWindows()
        try: rclpy.shutdown()
        except Exception: pass
        print(f"[burst] Done — {burst_count} burst(s) completed")


# ── Video file mode ───────────────────────────────────────────────────────────

def run_burst_video(cfg: dict, video_path: str, use_ros: bool) -> None:
    """
    Burst detector running from a recorded video file.

    use_ros=True:  initialise ROS, use cam.open_for_video() for drone pose +
                   GPS origin, publish results to detections topic.
    use_ros=False: console-only; no ROS dependency required.

    Burst timing is based on video timestamps (CAP_PROP_POS_MSEC), so
    "every 0.5 s" means 0.5 s of video time, not wall-clock time.
    """
    burst_cfg = cfg.get("burst", {})
    interval  = burst_cfg.get("interval_sec", _BURST_INTERVAL)
    count     = burst_cfg.get("frame_count",  _BURST_COUNT)

    model_path      = cfg["model"]
    crop_model_path = cfg["crop_model"]
    display         = cfg["display"]
    log             = cfg["log"]
    save_crops      = cfg.get("save_crops", False)
    save_det_images = cfg.get("save_det_images", False)
    save_color_pixels = cfg.get("save_color_pixels", False)
    target_color    = cfg.get("target_color")
    target_blinking = cfg.get("target_blinking")

    from ultralytics import YOLO

    for label, path in [(f"Video", video_path), ("Model", model_path),
                        ("Crop model", crop_model_path)]:
        if not os.path.exists(path):
            print(f"[burst] {label} not found: {path}")
            return

    model      = YOLO(model_path)
    crop_model = YOLO(crop_model_path)

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"[burst] Cannot open video: {video_path}")
        return

    fps   = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"[burst] Video mode — {video_path}")
    print(f"[burst]   {total} frames @ {fps:.1f} fps  "
          f"({total/fps:.1f}s)  use_ros={use_ros}")
    print(f"[burst]   interval={interval}s  total={interval*count:.1f}s  "
          f"frames/burst={count}")

    # ── ROS setup (optional) ─────────────────────────────────────────────
    cam        = None
    rclpy      = None
    publish_fn = None
    get_gps_fn = lambda: None

    if use_ros:
        _import_ros()
        import rclpy as _rclpy
        rclpy = _rclpy
        from std_msgs.msg import String

        topics = cfg["topics"]
        rclpy.init()
        cam = _make_beacon_camera(topics, cfg["camera"], cfg["detection"])
        if not cam.open_for_video():
            print("[burst] ROS video setup failed")
            try: rclpy.shutdown()
            except Exception: pass
            cap.release()
            return

        def _publish(json_str):
            msg      = String()
            msg.data = json_str
            cam.detection_pub.publish(msg)

        publish_fn = _publish
        get_gps_fn = cam.get_gps_origin
        print(f"[burst] Publishing → {topics['detections_pub']}")

    gps_gt_cfg = cfg.get("gps_ground_truth", {})
    if gps_gt_cfg.get("enabled"):
        if not use_ros:
            print("[burst] GPS ground truth configured but use_ros=False — "
                  "no drone GPS available in this mode, ground truth disabled")
        else:
            print(f"[burst] GPS ground truth enabled  "
                  f"target=({gps_gt_cfg['latitude']:.7f}, {gps_gt_cfg['longitude']:.7f})  "
                  f"object_height_agl={cfg['detection'].get('beacon_z_m', 0.0):.2f}m")

    DEBUG_DIR = os.path.expanduser("seabird_dataset/beacon_debug")
    os.makedirs(DEBUG_DIR, exist_ok=True)

    crops_dir = None
    if save_crops:
        crops_dir = os.path.join(DEBUG_DIR, "beacon_crops")
        os.makedirs(crops_dir, exist_ok=True)
        print(f"[burst] Saving crops → {crops_dir}/")

    det_images_dir = None
    if save_det_images:
        det_images_dir = os.path.join(DEBUG_DIR, "beacon_det_images")
        os.makedirs(det_images_dir, exist_ok=True)
        print(f"[burst] Saving detection images → {det_images_dir}/")

    color_pixels_dir = None
    if save_color_pixels:
        color_pixels_dir = os.path.join(DEBUG_DIR, "beacon_color_pixels")
        os.makedirs(color_pixels_dir, exist_ok=True)
        print(f"[burst] Saving color-vote pixels → {color_pixels_dir}/")

    log_fh = log_writer = None
    if log:
        ts_tag    = time.strftime("%Y%m%d_%H%M%S")
        vid_stem  = os.path.splitext(os.path.basename(video_path))[0]
        log_path  = os.path.join(DEBUG_DIR, f"burst_video_log_{vid_stem}_{ts_tag}.csv")
        log_fh, log_writer = _open_log(log_path)

    if display:
        cv2.namedWindow("Burst Detector", cv2.WINDOW_NORMAL)

    _apply_color_config(cfg["detection"])
    conf         = cfg["detection"].get("conf", 0.5)
    depth_source = cfg["detection"].get("depth_source", "topic")
    burst_count  = 0

    state         = "searching"
    burst: list   = []
    last_burst_ts = -999.0

    try:
        while True:
            ret, raw = cap.read()
            if not ret:
                break

            frame_ts  = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
            rgb_clean = raw.copy()  # keep BGR — cv2.imwrite and classify_beacon_color both expect BGR

            # Run YOLO directly on the raw frame
            results = model(raw, conf=conf, verbose=False)
            boxes   = results[0].boxes

            drone_pos = drone_quat = None
            if cam is not None:
                drone_pos, drone_quat = cam.get_drone_pose()

            # Build _VideoDet list (position_3d is None; no depth in video mode)
            dets = [
                _VideoDet(
                    bbox_2d=list(map(int, box.xyxy[0].tolist())),
                    position_3d=None,
                    confidence=float(box.conf[0]),
                    tracking_id=i,
                )
                for i, box in enumerate(boxes)
            ]

            if state == "searching":
                if dets:
                    print(f"[burst] Beacon detected at t={frame_ts:.2f}s — "
                          f"starting collection ({count} frames × {interval}s)")
                    state         = "collecting"
                    burst         = [(frame_ts, rgb_clean, dets, None, drone_pos, drone_quat)]
                    last_burst_ts = frame_ts
                    print(f"[burst]   1/{count}  (t={frame_ts:.2f}s)")

            elif state == "collecting":
                if frame_ts - last_burst_ts >= interval:
                    burst.append((frame_ts, rgb_clean, dets, None, drone_pos, drone_quat))
                    last_burst_ts = frame_ts
                    print(f"[burst]   {len(burst)}/{count}  (t={frame_ts:.2f}s)")

                if len(burst) >= count:
                    print("[burst] Collection complete — analysing")
                    burst_count += 1
                    lv = _analyse_burst(burst, crop_model, cfg, depth_source,
                                        save_crops_dir=crops_dir,
                                        det_images_dir=det_images_dir,
                                        color_pixels_dir=color_pixels_dir,
                                        target_color=target_color,
                                        target_blinking=target_blinking,
                                        log_writer=log_writer,
                                        burst_number=burst_count,
                                        get_gps_origin_fn=get_gps_fn)
                    if not lv:
                        print("[burst] No valid detections in burst")
                    else:
                        _publish_result(lv, burst, burst_count, cfg,
                                        get_gps_fn, publish_fn)
                    burst = []
                    state = "searching"
                    print("[burst] Resuming search")

            if display:
                overlay = raw.copy()
                for det in dets:
                    x1, y1, x2, y2 = det.bbox_2d
                    cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 200, 0), 2)
                if state == "collecting":
                    label_txt = f"COLLECTING  {len(burst)}/{count}"
                    color_bgr = (0, 255, 255)
                else:
                    label_txt = f"SEARCHING  t={frame_ts:.1f}s"
                    color_bgr = (180, 180, 180)
                cv2.putText(overlay, label_txt, (10, 36),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, color_bgr, 2)
                cv2.imshow("Burst Detector", overlay)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord(" "):
                    while cv2.waitKey(0) & 0xFF != ord(" "):
                        pass

    except KeyboardInterrupt:
        print("\n[burst] Interrupted")
    finally:
        if log_fh:
            log_fh.close()
        cap.release()
        if display:
            cv2.destroyAllWindows()
        if rclpy is not None:
            if cam is not None:
                cam.close()
            try: rclpy.shutdown()
            except Exception: pass
        print(f"[burst] Done — {burst_count} burst(s) completed")


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="burst_beacon_detector",
        description="Burst-collection beacon detector",
    )
    parser.add_argument(
        "--config", "-cfg",
        default=_DEFAULT_CONFIG,
        metavar="CONFIG_PATH",
        help=f"Path to JSON config file (default: {_DEFAULT_CONFIG})",
    )
    args = parser.parse_args()

    if not os.path.exists(args.config):
        print(f"[burst] Config not found: {args.config}")
        sys.exit(1)

    cfg = load_config(args.config)
    print(f"[burst] Loaded config: {args.config}")

    if cfg.get("ros_video") is not None:
        run_burst_video(cfg, cfg["ros_video"], use_ros=True)
    elif cfg.get("video") is not None:
        run_burst_video(cfg, cfg["video"], use_ros=False)
    else:
        run_burst_ros(cfg)
