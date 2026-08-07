#!/usr/bin/env python3
"""
burst_beacon_detector.py — Burst-collection beacon detector (live ROS).

Waits for the first beacon detection, then collects one frame every
BURST_INTERVAL seconds for a total of BURST_COUNT frames (default:
8 frames × 0.5 s = 4 s of data).  After collection the full color-
classification and blink-detection pipeline runs on all stored frames
and a single consolidated result is published.

State machine:  searching → collecting (8 frames) → analysis → searching

Usage:
    python3 burst_beacon_detector.py --config configs/modal_config_fred.json
"""

import os
import sys
import json
import time
import argparse

import cv2

from beacon_detector_config import (
    load_config,
    _import_ros,
    _make_beacon_camera,
    _apply_color_config,
    isolate_and_classify,
    estimate_distance_from_bbox,
    _camera_to_world,
    local_enu_to_gps,
    _open_log,
    _write_log_row,
    _DEFAULT_CONFIG,
)
from blink_detector import BlinkDetector

# ── Burst parameters (can be overridden via config key "burst") ───────────────
_BURST_INTERVAL = 0.5   # seconds between captured frames
_BURST_COUNT    = 8     # frames per burst  →  4 s of data


def run_burst_ros(cfg: dict) -> None:
    burst_cfg     = cfg.get("burst", {})
    interval      = burst_cfg.get("interval_sec", _BURST_INTERVAL)
    count         = burst_cfg.get("frame_count",  _BURST_COUNT)

    model_path      = cfg["model"]
    crop_model_path = cfg["crop_model"]
    display         = cfg["display"]
    log             = cfg["log"]
    topics          = cfg["topics"]

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

    print(f"[burst] Ready — waiting for beacon to trigger {count}-frame burst")
    print(f"[burst]   interval={interval}s  total={interval*count:.1f}s")
    print(f"[burst] Publishing → {topics['detections_pub']}")

    log_fh = log_writer = None
    if log:
        ts_tag   = time.strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(DEBUG_DIR, f"burst_log_{ts_tag}.csv")
        log_fh, log_writer = _open_log(log_path)

    if display:
        cv2.namedWindow("Burst Detector", cv2.WINDOW_NORMAL)

    _apply_color_config(cfg["detection"])
    depth_source = cfg["detection"].get("depth_source", "topic")
    burst_count  = 0   # completed bursts

    # Each entry: (frame_ts, rgb_clean, dets, depth, drone_pos, drone_quat)
    state         = "searching"
    burst: list   = []
    last_burst_ts = -999.0

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

            # ── Searching ─────────────────────────────────────────────────
            if state == "searching":
                if dets:
                    print(f"[burst] Beacon detected — starting collection "
                          f"({count} frames × {interval}s)")
                    state         = "collecting"
                    burst         = [(frame_ts, rgb_clean, dets, depth, drone_pos, drone_quat)]
                    last_burst_ts = frame_ts
                    print(f"[burst]   1/{count}")

            # ── Collecting ────────────────────────────────────────────────
            elif state == "collecting":
                if frame_ts - last_burst_ts >= interval:
                    burst.append((frame_ts, rgb_clean, dets, depth, drone_pos, drone_quat))
                    last_burst_ts = frame_ts
                    print(f"[burst]   {len(burst)}/{count}")

                if len(burst) >= count:
                    # ── Analysis ──────────────────────────────────────────
                    print("[burst] Collection complete — analysing")
                    blink_detector = BlinkDetector()
                    last_valid: dict = {}

                    for b_ts, b_rgb, b_dets, b_depth, b_dpos, b_dquat in burst:
                        if not b_dets:
                            continue
                        for d in b_dets:
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
                            (beacon_color, color_conf, light_mask, intensity, votes,
                             lit_region, hue_var, hue_mean, hue_median, hue_mode) = \
                                isolate_and_classify(crop, crop_model)

                            if beacon_color == "no_top":
                                print(f"[burst]   ts={b_ts:.2f}: no top — skipped")
                                continue

                            blink_info = blink_detector.update(
                                b_ts, beacon_color, intensity, color_conf)
                            print(f"[burst]   ts={b_ts:.2f}  color={beacon_color:7s} "
                                  f"conf={color_conf:.2f}  "
                                  f"blink_color={blink_info['blink_color']}  "
                                  f"is_blinking={blink_info['is_blinking']}")

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
                            )

                    if not last_valid:
                        print("[burst] No valid detections in burst")
                    else:
                        lv         = last_valid
                        world_pos  = None
                        gps_coords = None
                        if lv["pos3d"] is not None and lv["drone_pos"] is not None:
                            world_pos = _camera_to_world(
                                lv["pos3d"], lv["drone_pos"], lv["drone_quat"],
                                cfg["camera"]["_mount_offset"],
                                cfg["camera"]["_R_body_to_cam"],
                            )
                            origin = cam.get_gps_origin()
                            if origin is not None:
                                lat, lon, alt = local_enu_to_gps(world_pos, *origin)
                                gps_coords = {"latitude": lat, "longitude": lon,
                                              "altitude": alt}

                        bi = lv["blink_info"]
                        print(f"[burst] ── Burst #{burst_count + 1} result ─────────────")
                        print(f"[burst]   color={lv['beacon_color']}  "
                              f"is_blinking={bi['is_blinking']}  "
                              f"blink_color={bi['blink_color']}  "
                              f"blink_hz={bi['blink_hz']}")

                        msg      = String()
                        msg.data = json.dumps({
                            "color":            lv["beacon_color"],
                            "blink":            bi,
                            "label":            "beacon",
                            "color_confidence": round(lv["color_conf"], 4),
                            "intensity":        round(lv["intensity"], 4),
                            "hue_votes":        {k: round(v, 4)
                                                 for k, v in lv["votes"].items()},
                            "confidence":       round(lv["det_conf"], 4),
                            "bbox":             [int(v) for v in lv["bbox"]],
                            "position_3d":      [round(v, 4) for v in lv["pos3d"]]
                                                if lv["pos3d"] is not None else None,
                            "world_position":   [round(v, 4) for v in world_pos.tolist()]
                                                if world_pos is not None else None,
                            "gps_position":     gps_coords,
                            "drone_position":   lv["drone_pos"].tolist()
                                                if lv["drone_pos"] is not None else None,
                            "tracking_id":      lv["tracking_id"],
                            "burst_frames":     len(burst),
                            "timestamp":        round(lv["frame_ts"], 3),
                        })
                        cam.detection_pub.publish(msg)
                        burst_count += 1

                        if log_writer is not None:
                            _write_log_row(
                                log_writer, burst_count,
                                lv["beacon_color"], lv["color_conf"],
                                lv["intensity"], lv["votes"],
                                lv["det_conf"], lv["bbox"],
                                tracking_id=lv["tracking_id"],
                                pos3d=lv["pos3d"],
                                blink_info=bi,
                                target_color=cfg.get("target_color"),
                                target_blinking=cfg.get("target_blinking"),
                                img_w=lv["img_w"], img_h=lv["img_h"],
                                hue_variance=lv["hue_var"],
                                hue_mean=lv["hue_mean"],
                                hue_median=lv["hue_median"],
                                hue_mode=lv["hue_mode"],
                                frame_ts=lv["frame_ts"],
                            )

                    burst  = []
                    state  = "searching"
                    print("[burst] Resuming search")

            # ── Display ───────────────────────────────────────────────────
            if display:
                overlay = rgb.copy()
                if state == "collecting":
                    label_txt = f"COLLECTING  {len(burst)}/{count}"
                    color     = (0, 255, 255)
                else:
                    label_txt = "SEARCHING"
                    color     = (180, 180, 180)
                cv2.putText(overlay, label_txt, (10, 36),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
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
    run_burst_ros(cfg)
