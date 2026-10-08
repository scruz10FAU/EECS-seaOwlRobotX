"""
downward_beacon_classifier.py — color/blink/GPS classification of a beacon's
circular top face from a downward-facing camera, assuming the drone is
already hovering directly above it.

No trained model needed: the beacon's top diameter is known, the drone's AGL
height is known, so the pinhole model gives an expected pixel radius for the
circle — a classical-CV isolation (Hough circle transform, constrained to
that expected size near image center) finds it directly. Color classification
reuses beacon_detector_config.classify_beacon_color()'s hue-voting logic (via
its `seg_mask` parameter) so thresholds/behavior stay consistent with the
oblique-view pipeline; blink detection reuses BlinkDetector unchanged.

No ROS/rclpy dependency — this module is pure numpy/cv2/stdlib so it's
directly unit-testable with synthetic frames. beacon_detector_config is
imported lazily (inside functions, not at module load) since it pulls in
ultralytics/cv2-heavy top-level imports that aren't needed just to isolate a
circle.
"""

import time
from typing import Optional, Tuple

import cv2
import numpy as np

from utils.blink_detector import BlinkDetector


def expected_pixel_radius(fx: float, fy: float, depth_m: float, diameter_m: float) -> float:
    """
    Expected circle radius in pixels for a known-diameter circle at a known
    depth, via the pinhole model. Uses the average of fx/fy since a true
    nadir shot views the circle face-on (no foreshortening).
    """
    f_avg = (fx + fy) / 2.0
    return (diameter_m / 2.0) * f_avg / depth_m


def isolate_circle(
    bgr_frame: np.ndarray, cx: float, cy: float, expected_radius_px: float,
    tolerance_frac: float = 0.35,
) -> Optional[Tuple[np.ndarray, Tuple[float, float, float]]]:
    """
    Classical-CV isolation of the beacon's circular top face. Assumes the
    drone is hovering roughly centered above it, so the real circle should be
    near the image center (cx, cy) and close to expected_radius_px in size.

    Returns (mask, (circle_x, circle_y, circle_r)) in pixel coordinates, or
    None if nothing matching was found this frame.
    """
    h, w = bgr_frame.shape[:2]
    gray = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (9, 9), 2)

    min_r = max(1, int(expected_radius_px * (1 - tolerance_frac)))
    max_r = max(min_r + 1, int(expected_radius_px * (1 + tolerance_frac)))

    circles = cv2.HoughCircles(
        gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=max(h, w),
        param1=80, param2=25, minRadius=min_r, maxRadius=max_r,
    )
    if circles is None:
        return None

    # Multiple candidates can survive minDist filtering in pathological
    # cases -- pick the one closest to the expected center, since hovering
    # accurately means the real beacon should be there, not necessarily
    # wherever Hough's strongest response landed.
    candidates = circles[0]
    dists = (candidates[:, 0] - cx) ** 2 + (candidates[:, 1] - cy) ** 2
    bx, by, br = candidates[np.argmin(dists)]

    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.circle(mask, (int(round(bx)), int(round(by))), int(round(br)), 255, thickness=-1)
    return mask, (float(bx), float(by), float(br))


def classify_frame(bgr_frame: np.ndarray, mask: np.ndarray) -> Tuple[str, float, float, dict]:
    """
    Crop to the mask's bounding box (same crop-then-classify pattern
    beacon_detector_config.isolate_and_classify() uses) and reuse
    beacon_detector_config.classify_beacon_color() for the actual hue-vote
    decision, passing our circle mask as seg_mask -- same thresholds/logic
    as the oblique-view pipeline, no reimplementation.
    """
    import beacon_detector_config as bdc

    rows = np.any(mask > 0, axis=1)
    cols = np.any(mask > 0, axis=0)
    if not rows.any() or not cols.any():
        return "unknown", 0.0, 0.0, {"red": 0.0, "green": 0.0, "blue": 0.0, "other": 0.0}

    rmin, rmax = np.where(rows)[0][[0, -1]]
    cmin, cmax = np.where(cols)[0][[0, -1]]
    crop      = bgr_frame[rmin:rmax + 1, cmin:cmax + 1]
    crop_mask = mask[rmin:rmax + 1, cmin:cmax + 1]

    color, color_conf, _light_mask, intensity, votes, *_ = bdc.classify_beacon_color(
        crop, seg_mask=crop_mask
    )
    return color, color_conf, intensity, votes


def classify_beacon_downward(
    frame_source, drone_pos, drone_quat_wxyz, gps_origin, drone_height_agl,
    downward_cam_cfg: dict, detection_cfg: dict, duration_s: float,
) -> dict:
    """
    One-shot downward-camera beacon classification.

    frame_source:      iterable of (timestamp, bgr_frame) pairs, spanning
                        roughly duration_s seconds -- caller-supplied so this
                        function has no ROS dependency of its own.
    drone_pos:          (3,) ENU world position of the drone, metres.
    drone_quat_wxyz:    (4,) [w, x, y, z] drone orientation quaternion.
    gps_origin:         (lat, lon, alt) of the local ENU origin.
    drone_height_agl:   drone's measured AGL height, metres.
    downward_cam_cfg:   cfg["downward_camera"] dict from load_config() --
                        fx, fy, cx, cy, _mount_offset, _R_body_to_cam,
                        beacon_height_m, beacon_top_diameter_m.
    detection_cfg:      cfg["detection"] dict -- blink/color thresholds.
    duration_s:         informational only; the caller controls how long
                        frame_source actually spans (see downward_beacon_
                        classifier_node.py for the live-sampling loop).

    Returns a dict matching this repo's standard detection JSON shape:
        {"color", "blink": {"is_blinking","blink_color","blink_hz","phase"},
         "gps_position": {"latitude","longitude","altitude"} | None,
         "world_position": [x, y, z] | None, "n_frames_used": int}
    """
    import beacon_detector_config as bdc
    bdc._apply_color_config(detection_cfg)

    fx, fy = downward_cam_cfg["fx"], downward_cam_cfg["fy"]
    cx, cy = downward_cam_cfg["cx"], downward_cam_cfg["cy"]
    beacon_height_m = downward_cam_cfg["beacon_height_m"]
    beacon_diam_m   = downward_cam_cfg["beacon_top_diameter_m"]

    depth_m = drone_height_agl - beacon_height_m
    if depth_m <= 0:
        raise ValueError(
            f"drone_height_agl ({drone_height_agl:.2f}m) must exceed the beacon "
            f"height ({beacon_height_m:.2f}m) -- the drone must be above the beacon top"
        )
    radius_px = expected_pixel_radius(fx, fy, depth_m, beacon_diam_m)

    blink_detector = BlinkDetector()
    blink_result = {"is_blinking": None, "blink_color": "unknown", "blink_hz": None, "phase": "unknown"}
    last_color  = "unknown"
    last_circle = None   # (bx, by, br) px, from the most recent successful isolation
    n_used = 0

    for ts, frame in frame_source:
        found = isolate_circle(frame, cx, cy, radius_px)
        if found is None:
            continue
        mask, circle = found
        color, color_conf, intensity, _votes = classify_frame(frame, mask)
        blink_result = blink_detector.update(ts, color, intensity, color_conf)
        last_color  = color
        last_circle = circle
        n_used += 1

    gps_position = None
    world_position = None
    if last_circle is not None:
        bx, by, _br = last_circle
        # Pinhole back-projection at the known depth (equivalent to
        # camera_interface.Intrinsics.back_project(), inlined to avoid
        # needing a CameraInterface instance just for this).
        p_cam = np.array([
            (bx - cx) * depth_m / fx,
            (by - cy) * depth_m / fy,
            depth_m,
        ])
        world_position = bdc._camera_to_world(
            p_cam, np.asarray(drone_pos, dtype=np.float64),
            np.asarray(drone_quat_wxyz, dtype=np.float64),
            downward_cam_cfg["_mount_offset"], downward_cam_cfg["_R_body_to_cam"],
        )
        lat, lon, alt = bdc.local_enu_to_gps(
            world_position, gps_origin[0], gps_origin[1], gps_origin[2]
        )
        gps_position = {"latitude": lat, "longitude": lon, "altitude": alt}
        world_position = [float(v) for v in world_position]

    return {
        "color": last_color,
        "blink": blink_result,
        "gps_position": gps_position,
        "world_position": world_position,
        "n_frames_used": n_used,
        "timestamp": time.time(),
    }
