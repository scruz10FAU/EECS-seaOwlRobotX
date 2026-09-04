#!/usr/bin/env python3
"""
staged_beacon_detector.py — Color-first staged beacon detector.

Uses a single color-aware detector (config key "model") trained on four
classes — red / blue / green / unknown — to localize and color-classify the
beacon in one pass, instead of the generic single-class "beacon" detector
used by burst_beacon_detector.py:

  red / green   -> reported immediately, per detection. On this hardware
                    red and green beacons are always blinking, so blink
                    status isn't measured for them — is_blinking is set to
                    True by convention rather than observed.
  blue / unknown -> not reliably resolved from a single frame, so these
                    fall through to stage 2: the existing crop_model +
                    classify_beacon_color() + BlinkDetector burst pipeline
                    (identical to burst_beacon_detector.py), which collects
                    several frames and measures real blink status.

Stage 2 is imported directly from burst_beacon_detector.py — no logic is
duplicated, this script only adds the color-gating in front of it.

Modes (selected by config key, same as burst_beacon_detector.py):
  ros_video  — video file + ROS publishing   (key "ros_video": "/path/to/file")
  video      — video file, console only       (key "video": "/path/to/file")
  (neither)  — live ROS camera               (default)

Usage:
    python3 staged_beacon_detector.py --config configs/modal_config_fred.json
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
    estimate_distance_from_bbox,
    _camera_to_world,
    local_enu_to_gps,
    gps_ground_truth_distance,
    _open_log,
    _write_log_row,
    _DEFAULT_CONFIG,
)
from burst_beacon_detector import (
    _VideoDet,
    _analyse_burst,
    _publish_result,
    _BURST_INTERVAL,
    _BURST_COUNT,
)

# Colors resolved immediately by the stage-1 model alone, no stage 2 needed.
_IMMEDIATE_COLORS = ("red", "green")
# Colors that fall through to the existing crop_model + blink_detector burst pipeline.
_STAGE2_COLORS    = ("blue", "unknown")


def _normalize_color_class(name: str) -> str:
    """
    Map a stage-1 model class name to one of red/green/blue/unknown.
    Tolerant of class names like "red_beacon" or "beacon_red", not just
    an exact "red" — matches by substring so the model's own naming
    convention doesn't have to be guessed exactly.
    """
    n = (name or "").strip().lower()
    for c in ("red", "green", "blue"):
        if c in n:
            return c
    return "unknown"


# ── Immediate (red/green) reporting — no stage 2, no burst collection ─────────

def _publish_immediate(color, det_conf, bbox, frame_ts, drone_pos, drone_quat, cfg,
                       get_gps_origin_fn, publish_fn, log_writer, frame_idx, det_idx,
                       img_w, img_h):
    """
    Report a red/green detection right away: no burst collection, no
    crop_model, no HSV vote. Blink status is asserted True (not measured) --
    red and green beacons are always blinking on this hardware.
    """
    x1, y1, x2, y2 = bbox
    det_cfg = cfg["detection"]
    depth_source = det_cfg.get("depth_source", "topic")

    pos3d = None
    if depth_source == "bbox":
        pos3d = estimate_distance_from_bbox(
            x1, y1, x2, y2,
            cfg["camera"]["fx"], cfg["camera"]["fy"],
            cfg["camera"]["cx"], cfg["camera"]["cy"],
            drone_pos[2] if drone_pos is not None else 0.0,
            det_cfg.get("beacon_z_m", 0.0),
            det_cfg.get("beacon_height_m", 0.3048),
        )

    world_pos  = None
    gps_coords = None
    if pos3d is not None and drone_pos is not None and drone_quat is not None:
        world_pos = _camera_to_world(
            pos3d, drone_pos, drone_quat,
            cfg["camera"]["_mount_offset"], cfg["camera"]["_R_body_to_cam"],
        )
        origin = get_gps_origin_fn() if get_gps_origin_fn is not None else None
        if origin is not None:
            lat, lon, alt = local_enu_to_gps(world_pos, *origin)
            gps_coords = {"latitude": lat, "longitude": lon, "altitude": alt}

    gps_gt_cfg     = cfg.get("gps_ground_truth", {})
    obj_height_agl = det_cfg.get("beacon_z_m", 0.0)
    gps_gt_info    = None
    if gps_gt_cfg.get("enabled") and get_gps_origin_fn is not None and drone_pos is not None:
        origin = get_gps_origin_fn()
        if origin is not None:
            d_lat, d_lon, _ = origin
            d_height_agl = drone_pos[2]
            dist, horiz, vert = gps_ground_truth_distance(
                d_lat, d_lon, d_height_agl,
                gps_gt_cfg["latitude"], gps_gt_cfg["longitude"], obj_height_agl,
            )
            gps_gt_info = (dist, horiz, vert, gps_gt_cfg["latitude"], gps_gt_cfg["longitude"], d_height_agl)

    # Not measured -- red/green are always blinking on this hardware by convention.
    blink_info = {"is_blinking": True, "blink_color": color, "blink_hz": None, "phase": "on"}
    votes = {"red": 0.0, "green": 0.0, "blue": 0.0, "other": 0.0}
    votes[color] = 1.0

    print(f"[staged] ★ immediate {color}  stage1_conf={det_conf:.2f}  "
          f"(blinking assumed True, not measured)")

    if log_writer is not None:
        _write_log_row(
            log_writer, frame_idx,
            color, det_conf, 1.0, votes,
            det_conf, bbox,
            tracking_id=det_idx,
            pos3d=pos3d,
            blink_info=blink_info,
            gps_gt_info=gps_gt_info,
            img_w=img_w, img_h=img_h,
            frame_ts=frame_ts,
            burst_color=color,
            burst_blink_info=blink_info,
        )

    if publish_fn is not None:
        payload = {
            "color":            color,
            "blink":            blink_info,
            "label":            "beacon",
            "color_confidence": 1.0,
            "intensity":        1.0,
            "hue_votes":        votes,
            "confidence":       round(det_conf, 4),
            "bbox":             [int(v) for v in bbox],
            "position_3d":      [round(v, 4) for v in pos3d] if pos3d is not None else None,
            "world_position":   [round(v, 4) for v in world_pos.tolist()] if world_pos is not None else None,
            "gps_position":     gps_coords,
            "drone_position":   drone_pos.tolist() if drone_pos is not None else None,
            "gps_ground_truth": {
                "distance_m":   round(gps_gt_info[0], 4),
                "horizontal_m": round(gps_gt_info[1], 4),
                "vertical_m":   round(gps_gt_info[2], 4),
            } if gps_gt_info is not None else None,
            "tracking_id":      det_idx,
            "stage":            "immediate",
            "timestamp":        round(frame_ts, 3),
        }
        publish_fn(json.dumps(payload))


def _split_detections(boxes, names):
    """
    Split a stage-1 result's boxes into (immediate, stage2) lists.
    immediate: [(color, conf, (x1,y1,x2,y2)), ...]
    stage2:    [_VideoDet, ...]
    """
    immediate = []
    stage2 = []
    for i, box in enumerate(boxes):
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        conf = float(box.conf[0])
        cls_name = names.get(int(box.cls[0]), "unknown")
        color = _normalize_color_class(cls_name)
        if color in _IMMEDIATE_COLORS:
            immediate.append((color, conf, (x1, y1, x2, y2)))
        else:
            stage2.append(_VideoDet(bbox_2d=[x1, y1, x2, y2], position_3d=None,
                                    confidence=conf, tracking_id=i))
    return immediate, stage2


# ── Live ROS camera mode ──────────────────────────────────────────────────────

def run_staged_ros(cfg: dict) -> None:
    burst_cfg = cfg.get("burst", {})
    interval  = burst_cfg.get("interval_sec", _BURST_INTERVAL)
    count     = burst_cfg.get("frame_count",  _BURST_COUNT)
    nodet_save_interval = burst_cfg.get("nodet_save_interval_sec", 2.0)

    model_path      = cfg["model"]        # stage-1 color-aware detector (red/blue/green/unknown)
    crop_model_path = cfg["crop_model"]    # stage-2 top isolator (blue/unknown path only)
    display         = cfg["display"]
    log             = cfg["log"]
    topics          = cfg["topics"]
    save_crops        = cfg.get("save_crops", False)
    save_det_images   = cfg.get("save_det_images", False)
    save_color_pixels = cfg.get("save_color_pixels", False)
    save_frames       = cfg.get("save_frames", False)
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
        print("[staged] Failed to open camera")
        try: rclpy.shutdown()
        except Exception: pass
        return

    for label, path in [("Model", model_path), ("Crop model", crop_model_path)]:
        if not os.path.exists(path):
            print(f"[staged] {label} not found: {path}")
            cam.close()
            try: rclpy.shutdown()
            except Exception: pass
            return

    color_model = YOLO(model_path)
    crop_model  = YOLO(crop_model_path)

    print(f"[staged] Live ROS mode — color-first pipeline")
    print(f"[staged]   immediate colors: {_IMMEDIATE_COLORS}  (blink asserted True)")
    print(f"[staged]   stage-2 colors:   {_STAGE2_COLORS}  "
          f"(burst: {count} frames x {interval}s -> crop_model + blink_detector)")
    print(f"[staged] Publishing → {topics['detections_pub']}")

    gps_gt_cfg = cfg.get("gps_ground_truth", {})
    if gps_gt_cfg.get("enabled"):
        print(f"[staged] GPS ground truth enabled  "
              f"target=({gps_gt_cfg['latitude']:.7f}, {gps_gt_cfg['longitude']:.7f})  "
              f"object_height_agl={cfg['detection'].get('beacon_z_m', 0.0):.2f}m")

    log_fh = log_writer = None
    if log:
        ts_tag   = time.strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(DEBUG_DIR, f"staged_log_{ts_tag}.csv")
        log_fh, log_writer = _open_log(log_path)

    crops_dir = None
    if save_crops:
        crops_dir = os.path.join(DEBUG_DIR, "beacon_crops")
        os.makedirs(crops_dir, exist_ok=True)
        print(f"[staged] Saving crops → {crops_dir}/")

    det_images_dir = None
    if save_det_images:
        det_images_dir = os.path.join(DEBUG_DIR, "beacon_det_images")
        os.makedirs(det_images_dir, exist_ok=True)
        print(f"[staged] Saving detection images → {det_images_dir}/")

    color_pixels_dir = None
    if save_color_pixels:
        color_pixels_dir = os.path.join(DEBUG_DIR, "beacon_color_pixels")
        os.makedirs(color_pixels_dir, exist_ok=True)
        print(f"[staged] Saving color-vote pixels → {color_pixels_dir}/")

    frames_dir = None
    if save_frames:
        frames_dir = os.path.join(DEBUG_DIR, "full_frames")
        os.makedirs(frames_dir, exist_ok=True)
        print(f"[staged] Saving frames → {frames_dir}/ "
              f"(one every {nodet_save_interval:.1f}s while searching with no detections)")

    if display:
        cv2.namedWindow("Staged Detector", cv2.WINDOW_NORMAL)

    _apply_color_config(cfg["detection"])
    depth_source = cfg["detection"].get("depth_source", "topic")
    conf         = cfg["detection"].get("conf", 0.5)
    imgsz        = cfg["detection"].get("imgsz", 640)
    burst_count  = 0
    immediate_count = 0

    state             = "searching"
    burst: list       = []
    last_burst_ts     = -999.0
    last_nodet_save   = -999.0

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

            results = color_model(rgb, conf=conf, imgsz=imgsz, verbose=False)
            boxes   = results[0].boxes
            names   = results[0].names
            immediate, stage2_dets = _split_detections(boxes, names)

            date_tag = time.strftime("%Y%m%d_%H%M%S")
            for color, det_conf, bbox in immediate:
                immediate_count += 1
                _publish_immediate(color, det_conf, bbox, frame_ts, drone_pos, drone_quat, cfg,
                                   cam.get_gps_origin, _publish, log_writer,
                                   frame_idx=immediate_count, det_idx=0,
                                   img_w=rgb.shape[1], img_h=rgb.shape[0])
                if det_images_dir is not None:
                    x1, y1, x2, y2 = bbox
                    pad = 20
                    h_img, w_img = rgb.shape[:2]
                    ix1, iy1 = max(x1 - pad, 0), max(y1 - pad, 0)
                    ix2, iy2 = min(x2 + pad, w_img), min(y2 + pad, h_img)
                    det_crop = rgb_clean[iy1:iy2, ix1:ix2].copy()
                    fname = (f"det_{date_tag}_immediate_{color}"
                             f"_conf{int(det_conf*100):02d}.png")
                    cv2.imwrite(os.path.join(det_images_dir, fname), det_crop)

            if state == "searching":
                if stage2_dets:
                    print(f"[staged] Blue/unknown detected — starting stage-2 collection "
                          f"({count} frames × {interval}s)")
                    state         = "collecting"
                    burst         = [(frame_ts, rgb_clean, stage2_dets, depth, drone_pos, drone_quat)]
                    last_burst_ts = frame_ts
                    print(f"[staged]   1/{count}")
                elif frames_dir is not None and frame_ts - last_nodet_save >= nodet_save_interval:
                    fname = f"nodet_{time.strftime('%Y%m%d')}_t{frame_ts:.2f}.png"
                    cv2.imwrite(os.path.join(frames_dir, fname), rgb_clean)
                    last_nodet_save = frame_ts

            elif state == "collecting":
                if immediate:
                    print(f"[staged] {immediate[0][0]} detected during stage-2 collection — "
                          f"abandoning burst (already reported above), resuming search")
                    burst = []
                    state = "searching"

                elif frame_ts - last_burst_ts >= interval:
                    burst.append((frame_ts, rgb_clean, stage2_dets, depth, drone_pos, drone_quat))
                    last_burst_ts = frame_ts
                    print(f"[staged]   {len(burst)}/{count}")

                if state == "collecting" and len(burst) >= count:
                    print("[staged] Stage-2 collection complete — analysing")
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
                        print("[staged] No valid detections in stage-2 burst")
                    else:
                        _publish_result(lv, burst, burst_count, cfg,
                                        cam.get_gps_origin, _publish)
                    burst = []
                    state = "searching"
                    print("[staged] Resuming search")

            if display:
                overlay = rgb.copy()
                if state == "collecting":
                    label_txt = f"STAGE2 COLLECTING  {len(burst)}/{count}"
                    color_bgr = (0, 255, 255)
                else:
                    label_txt = "SEARCHING"
                    color_bgr = (180, 180, 180)
                cv2.putText(overlay, label_txt, (10, 36),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, color_bgr, 2)
                cv2.imshow("Staged Detector", overlay)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except KeyboardInterrupt:
        print("\n[staged] Interrupted")
    finally:
        if log_fh:
            log_fh.close()
        cam.close()
        if display:
            cv2.destroyAllWindows()
        try: rclpy.shutdown()
        except Exception: pass
        print(f"[staged] Done — {immediate_count} immediate + {burst_count} stage-2 burst(s) completed")


# ── Video file mode ───────────────────────────────────────────────────────────

def run_staged_video(cfg: dict, video_path: str, use_ros: bool) -> None:
    """
    Staged detector running from a recorded video file. Mirrors
    burst_beacon_detector.run_burst_video's structure, with the same
    color-gating added in front of stage 2.
    """
    burst_cfg = cfg.get("burst", {})
    interval  = burst_cfg.get("interval_sec", _BURST_INTERVAL)
    count     = burst_cfg.get("frame_count",  _BURST_COUNT)

    model_path      = cfg["model"]
    crop_model_path = cfg["crop_model"]
    display         = cfg["display"]
    log             = cfg["log"]
    save_crops        = cfg.get("save_crops", False)
    save_det_images   = cfg.get("save_det_images", False)
    save_color_pixels = cfg.get("save_color_pixels", False)
    target_color    = cfg.get("target_color")
    target_blinking = cfg.get("target_blinking")

    from ultralytics import YOLO

    for label, path in [("Video", video_path), ("Model", model_path),
                        ("Crop model", crop_model_path)]:
        if not os.path.exists(path):
            print(f"[staged] {label} not found: {path}")
            return

    color_model = YOLO(model_path)
    crop_model  = YOLO(crop_model_path)

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"[staged] Cannot open video: {video_path}")
        return

    fps   = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"[staged] Video mode — {video_path}")
    print(f"[staged]   {total} frames @ {fps:.1f} fps  ({total/fps:.1f}s)  use_ros={use_ros}")
    print(f"[staged]   immediate colors: {_IMMEDIATE_COLORS}  stage-2 colors: {_STAGE2_COLORS}")
    print(f"[staged]   stage-2 burst: interval={interval}s  frames/burst={count}")

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
            print("[staged] ROS video setup failed")
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
        print(f"[staged] Publishing → {topics['detections_pub']}")

    gps_gt_cfg = cfg.get("gps_ground_truth", {})
    if gps_gt_cfg.get("enabled"):
        if not use_ros:
            print("[staged] GPS ground truth configured but use_ros=False — "
                  "no drone GPS available in this mode, ground truth disabled")
        else:
            print(f"[staged] GPS ground truth enabled  "
                  f"target=({gps_gt_cfg['latitude']:.7f}, {gps_gt_cfg['longitude']:.7f})  "
                  f"object_height_agl={cfg['detection'].get('beacon_z_m', 0.0):.2f}m")

    DEBUG_DIR = os.path.expanduser("seabird_dataset/beacon_debug")
    os.makedirs(DEBUG_DIR, exist_ok=True)

    crops_dir = None
    if save_crops:
        crops_dir = os.path.join(DEBUG_DIR, "beacon_crops")
        os.makedirs(crops_dir, exist_ok=True)
        print(f"[staged] Saving crops → {crops_dir}/")

    det_images_dir = None
    if save_det_images:
        det_images_dir = os.path.join(DEBUG_DIR, "beacon_det_images")
        os.makedirs(det_images_dir, exist_ok=True)
        print(f"[staged] Saving detection images → {det_images_dir}/")

    color_pixels_dir = None
    if save_color_pixels:
        color_pixels_dir = os.path.join(DEBUG_DIR, "beacon_color_pixels")
        os.makedirs(color_pixels_dir, exist_ok=True)
        print(f"[staged] Saving color-vote pixels → {color_pixels_dir}/")

    log_fh = log_writer = None
    if log:
        ts_tag    = time.strftime("%Y%m%d_%H%M%S")
        vid_stem  = os.path.splitext(os.path.basename(video_path))[0]
        log_path  = os.path.join(DEBUG_DIR, f"staged_video_log_{vid_stem}_{ts_tag}.csv")
        log_fh, log_writer = _open_log(log_path)

    if display:
        cv2.namedWindow("Staged Detector", cv2.WINDOW_NORMAL)

    _apply_color_config(cfg["detection"])
    conf         = cfg["detection"].get("conf", 0.5)
    imgsz        = cfg["detection"].get("imgsz", 640)
    depth_source = cfg["detection"].get("depth_source", "topic")
    burst_count      = 0
    immediate_count  = 0

    state         = "searching"
    burst: list   = []
    last_burst_ts = -999.0

    try:
        while True:
            ret, raw = cap.read()
            if not ret:
                break

            frame_ts  = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
            rgb_clean = raw.copy()

            results = color_model(raw, conf=conf, imgsz=imgsz, verbose=False)
            boxes   = results[0].boxes
            names   = results[0].names
            immediate, stage2_dets = _split_detections(boxes, names)

            drone_pos = drone_quat = None
            if cam is not None:
                drone_pos, drone_quat = cam.get_drone_pose()

            date_tag = time.strftime("%Y%m%d_%H%M%S")
            for color, det_conf, bbox in immediate:
                immediate_count += 1
                print(f"[staged] Immediate {color} at t={frame_ts:.2f}s  conf={det_conf:.2f}")
                _publish_immediate(color, det_conf, bbox, frame_ts, drone_pos, drone_quat, cfg,
                                   get_gps_fn, publish_fn, log_writer,
                                   frame_idx=immediate_count, det_idx=0,
                                   img_w=raw.shape[1], img_h=raw.shape[0])
                if det_images_dir is not None:
                    x1, y1, x2, y2 = bbox
                    pad = 20
                    h_img, w_img = raw.shape[:2]
                    ix1, iy1 = max(x1 - pad, 0), max(y1 - pad, 0)
                    ix2, iy2 = min(x2 + pad, w_img), min(y2 + pad, h_img)
                    det_crop = rgb_clean[iy1:iy2, ix1:ix2].copy()
                    fname = f"det_{date_tag}_immediate_{color}_conf{int(det_conf*100):02d}.png"
                    cv2.imwrite(os.path.join(det_images_dir, fname), det_crop)

            if state == "searching":
                if stage2_dets:
                    print(f"[staged] Blue/unknown at t={frame_ts:.2f}s — "
                          f"starting stage-2 collection ({count} frames × {interval}s)")
                    state         = "collecting"
                    burst         = [(frame_ts, rgb_clean, stage2_dets, None, drone_pos, drone_quat)]
                    last_burst_ts = frame_ts
                    print(f"[staged]   1/{count}  (t={frame_ts:.2f}s)")

            elif state == "collecting":
                if immediate:
                    print(f"[staged] {immediate[0][0]} detected during stage-2 collection — "
                          f"abandoning burst (already reported above), resuming search")
                    burst = []
                    state = "searching"

                elif frame_ts - last_burst_ts >= interval:
                    burst.append((frame_ts, rgb_clean, stage2_dets, None, drone_pos, drone_quat))
                    last_burst_ts = frame_ts
                    print(f"[staged]   {len(burst)}/{count}  (t={frame_ts:.2f}s)")

                if state == "collecting" and len(burst) >= count:
                    print("[staged] Stage-2 collection complete — analysing")
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
                        print("[staged] No valid detections in stage-2 burst")
                    else:
                        _publish_result(lv, burst, burst_count, cfg,
                                        get_gps_fn, publish_fn)
                    burst = []
                    state = "searching"
                    print("[staged] Resuming search")

            if display:
                overlay = raw.copy()
                for det in stage2_dets:
                    x1, y1, x2, y2 = det.bbox_2d
                    cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 200, 0), 2)
                for color, det_conf, bbox in immediate:
                    x1, y1, x2, y2 = bbox
                    cv2.rectangle(overlay, (x1, y1), (x2, y2), (255, 0, 255), 2)
                if state == "collecting":
                    label_txt = f"STAGE2 COLLECTING  {len(burst)}/{count}"
                    color_bgr = (0, 255, 255)
                else:
                    label_txt = f"SEARCHING  t={frame_ts:.1f}s"
                    color_bgr = (180, 180, 180)
                cv2.putText(overlay, label_txt, (10, 36),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, color_bgr, 2)
                cv2.imshow("Staged Detector", overlay)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord(" "):
                    while cv2.waitKey(0) & 0xFF != ord(" "):
                        pass

    except KeyboardInterrupt:
        print("\n[staged] Interrupted")
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
        print(f"[staged] Done — {immediate_count} immediate + {burst_count} stage-2 burst(s) completed")


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="staged_beacon_detector",
        description="Color-first staged beacon detector (red/green immediate, blue/unknown -> stage 2)",
    )
    parser.add_argument(
        "--config", "-cfg",
        default=_DEFAULT_CONFIG,
        metavar="CONFIG_PATH",
        help=f"Path to JSON config file (default: {_DEFAULT_CONFIG})",
    )
    args = parser.parse_args()

    if not os.path.exists(args.config):
        print(f"[staged] Config not found: {args.config}")
        sys.exit(1)

    cfg = load_config(args.config)
    print(f"[staged] Loaded config: {args.config}")

    if cfg.get("ros_video") is not None:
        run_staged_video(cfg, cfg["ros_video"], use_ros=True)
    elif cfg.get("video") is not None:
        run_staged_video(cfg, cfg["video"], use_ros=False)
    else:
        run_staged_ros(cfg)
