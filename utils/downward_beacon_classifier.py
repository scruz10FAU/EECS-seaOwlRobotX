"""
downward_beacon_classifier.py — color/blink/GPS classification of a beacon's
circular top face from a downward-facing camera, assuming the drone is
already hovering directly above it.

No trained model needed. Two attempts at pure per-frame shape detection
(Hough circles on the LED's own edge/contrast, then adaptive thresholding
for a "always-dark disc") both failed against real footage: the disc's
absolute brightness isn't reliably darker *or* brighter than its
surroundings frame-to-frame (it depends entirely on what's nearby), so no
single-frame shape/contrast cue is robust enough on its own.

Current approach -- POSITION PERSISTENCE: when the LED is actually lit, it's
trivially easy to find (a small, strongly saturated+bright blob against
anything) -- find_lit_blob() does exactly that, restricted to a search
window around wherever the beacon was last confirmed (or the geometric
prediction from drone height, before any lock exists), so it isn't
distracted by unrelated bright clutter elsewhere in a cluttered frame. Once
a lit detection confirms the real position, classify_beacon_downward() LOCKS
onto it for the rest of the sampling window (the drone is holding roughly
still) -- frames where the LED happens to be off simply sample that same
locked position (no re-search needed, and no frame is ever skipped, so
BlinkDetector gets a real on/off signal every frame instead of silence).
Color classification reuses beacon_detector_config.classify_beacon_color()'s
hue-voting logic on a crop sized to the known disc diameter, centered on
whatever position was determined (detected / locked / geometric fallback).
Blink detection reuses BlinkDetector unchanged.

No ROS/rclpy dependency — this module is pure numpy/cv2/stdlib so it's
directly unit-testable with synthetic frames. beacon_detector_config is
imported lazily (inside functions, not at module load) since it pulls in
ultralytics/cv2-heavy top-level imports that aren't needed just to isolate a
circle.
"""

import os
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


def find_lit_blob(
    bgr_frame: np.ndarray, search_center: Tuple[float, float], search_radius: float,
    min_pixels: int = 15, sat_min: int = 40, val_min: int = 80,
) -> Optional[Tuple[float, float, float]]:
    """
    Find the largest sufficiently lit (saturated + bright) blob within a
    circular search window, via simple HSV thresholding -- reuses the same
    sat_min/val_min convention as beacon_detector_config's own
    _SAT_MIN/_VAL_MIN. A real, on LED is trivially easy to find this way
    (strongly saturated and bright against almost anything); restricting the
    search to a window around the expected/locked position is what keeps
    this from being distracted by unrelated bright clutter elsewhere in a
    cluttered frame.

    Returns (blob_x, blob_y, pixel_count) -- the lit blob's centroid and
    size -- or None if nothing large enough qualifies within the window.
    """
    h, w = bgr_frame.shape[:2]
    hsv = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)
    s, v = hsv[:, :, 1], hsv[:, :, 2]
    lit = ((s >= sat_min) & (v >= val_min)).astype(np.uint8) * 255

    window = np.zeros((h, w), dtype=np.uint8)
    cv2.circle(window, (int(round(search_center[0])), int(round(search_center[1]))),
               int(round(search_radius)), 255, thickness=-1)
    lit = cv2.bitwise_and(lit, window)
    lit = cv2.morphologyEx(lit, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    contours, _ = cv2.findContours(lit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    best = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(best)
    if area < min_pixels:
        return None
    moments = cv2.moments(best)
    if moments["m00"] == 0:
        return None
    return moments["m10"] / moments["m00"], moments["m01"] / moments["m00"], area


def _crop_to_mask_bbox(bgr_frame: np.ndarray, mask: np.ndarray):
    """
    Crop both the frame and the mask to the mask's bounding box. Shared by
    classify_frame() (the actual classification input) and the debug-image
    saver (so what gets saved to disk is pixel-for-pixel what the
    classifier saw) -- single source of truth for "what region is this
    frame's classification actually based on".

    Returns (crop, crop_mask), or (None, None) if the mask is empty.
    """
    rows = np.any(mask > 0, axis=1)
    cols = np.any(mask > 0, axis=0)
    if not rows.any() or not cols.any():
        return None, None
    rmin, rmax = np.where(rows)[0][[0, -1]]
    cmin, cmax = np.where(cols)[0][[0, -1]]
    return bgr_frame[rmin:rmax + 1, cmin:cmax + 1], mask[rmin:rmax + 1, cmin:cmax + 1]


def classify_frame(bgr_frame: np.ndarray, mask: np.ndarray) -> Tuple[str, float, float, dict]:
    """
    Crop to the mask's bounding box (same crop-then-classify pattern
    beacon_detector_config.isolate_and_classify() uses) and reuse
    beacon_detector_config.classify_beacon_color() for the actual hue-vote
    decision, passing our circle mask as seg_mask -- same thresholds/logic
    as the oblique-view pipeline, no reimplementation.
    """
    import beacon_detector_config as bdc

    crop, crop_mask = _crop_to_mask_bbox(bgr_frame, mask)
    if crop is None:
        return "unknown", 0.0, 0.0, {"red": 0.0, "green": 0.0, "blue": 0.0, "other": 0.0}

    color, color_conf, _light_mask, intensity, votes, *_ = bdc.classify_beacon_color(
        crop, seg_mask=crop_mask
    )
    return color, color_conf, intensity, votes


_SOURCE_COLORS = {
    "detected": (0, 255, 0),     # green -- a lit blob was found this frame
    "locked":   (0, 255, 255),   # yellow -- reusing a previously-confirmed position (LED off this frame)
    "fallback": (0, 165, 255),   # orange -- no lock yet and nothing lit; pure geometric guess
}


def _save_debug_images(debug_image_dir: str, frame_idx: int, ts: float, frame: np.ndarray,
                       circle, color: str, source: str,
                       intensity: float = None, color_conf: float = None) -> None:
    """
    Write out exactly what a given frame's classification was based on:
      frame_NNNN_<ts>_<color>_<source>_overlay.png  -- full frame with the
                               sampled region outline + label drawn on it.
                               source is "detected" (a lit blob was actually
                               found this frame), "locked" (LED off this
                               frame, reusing a previously-confirmed
                               position), or "fallback" (no lock yet and
                               nothing lit -- pure geometric guess, least
                               reliable).
      frame_NNNN_<ts>_<color>_<source>_crop.png     -- the cropped region
                               actually passed to classify_beacon_color()
      frame_NNNN_<ts>_<color>_<source>_mask.png     -- the binary mask
                               (within that crop) marking which pixels were
                               counted as "lit"
    """
    os.makedirs(debug_image_dir, exist_ok=True)
    tag = f"frame_{frame_idx:04d}_{ts:.3f}_{color}_{source}"

    bx, by, br = circle
    outline_color = _SOURCE_COLORS.get(source, (255, 255, 255))
    overlay = frame.copy()
    cv2.circle(overlay, (int(round(bx)), int(round(by))), int(round(br)), outline_color, 2)
    cv2.drawMarker(overlay, (int(round(bx)), int(round(by))), outline_color,
                   cv2.MARKER_CROSS, 12, 1)
    label = f"frame={frame_idx} t={ts:.2f}s color={color} src={source}"
    if intensity is not None:
        label += f" int={intensity:.2f}"
    if color_conf is not None:
        label += f" cc={color_conf:.2f}"
    cv2.putText(overlay, label, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, outline_color, 1, cv2.LINE_AA)
    cv2.imwrite(os.path.join(debug_image_dir, f"{tag}_overlay.png"), overlay)

    mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    cv2.circle(mask, (int(round(bx)), int(round(by))), int(round(br)), 255, thickness=-1)
    crop, crop_mask = _crop_to_mask_bbox(frame, mask)
    if crop is not None:
        cv2.imwrite(os.path.join(debug_image_dir, f"{tag}_crop.png"), crop)
        cv2.imwrite(os.path.join(debug_image_dir, f"{tag}_mask.png"), crop_mask)


def classify_beacon_downward(
    frame_source, drone_pos, drone_quat_wxyz, gps_origin, drone_height_agl,
    downward_cam_cfg: dict, detection_cfg: dict, duration_s: float,
    debug_image_dir: Optional[str] = None,
    expected_radius_px_override: Optional[float] = None,
) -> dict:
    """
    One-shot downward-camera beacon classification.

    frame_source:      iterable of (timestamp, bgr_frame) pairs, spanning
                        roughly duration_s seconds -- caller-supplied so this
                        function has no ROS dependency of its own.
    drone_pos:          (3,) ENU world position of the drone, metres.
    drone_quat_wxyz:    (4,) [w, x, y, z] drone orientation quaternion.
    gps_origin:         (lat, lon, alt) of the local ENU origin.
    drone_height_agl:   drone's measured AGL height, metres. Still required
                        (and still validated below) even when
                        expected_radius_px_override is set, since depth_m
                        derived from it is also used for the GPS/world-
                        position back-projection, not just circle sizing.
    downward_cam_cfg:   cfg["downward_camera"] dict from load_config() --
                        fx, fy, cx, cy, _mount_offset, _R_body_to_cam,
                        beacon_height_m, beacon_top_diameter_m.
    detection_cfg:      cfg["detection"] dict -- blink/color thresholds.
    duration_s:         informational only; the caller controls how long
                        frame_source actually spans (see downward_beacon_
                        classifier_node.py for the live-sampling loop).
    debug_image_dir:    if set, write out per-frame debug images (overlay,
                        crop, mask -- or the raw frame tagged "notfound")
                        showing exactly what each frame's classification was
                        based on. See _save_debug_images(). None = no I/O,
                        same as before (default, keeps this function pure
                        for unit testing).
    expected_radius_px_override: if set, used directly as the expected disc
                        radius in pixels instead of deriving it from
                        drone_height_agl/beacon_top_diameter_m/fx/fy. Useful
                        when height telemetry and/or lens specs aren't
                        trustworthy yet (e.g. hand-holding the drone for a
                        bench test at an unmeasured distance) -- measure the
                        disc's actual pixel radius from one debug image and
                        pass it directly rather than fighting the height
                        math to reproduce it indirectly.

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
    radius_px = (
        expected_radius_px_override if expected_radius_px_override is not None
        else expected_pixel_radius(fx, fy, depth_m, beacon_diam_m)
    )

    blink_detector = BlinkDetector()
    blink_result = {"is_blinking": None, "blink_color": "unknown", "blink_hz": None, "phase": "unknown"}
    last_color  = "unknown"
    last_circle = None   # (bx, by, br) px, from the most recent sampled position
    n_used = 0

    # Position persistence: once a lit blob confirms the real position, lock
    # onto it for the rest of the window instead of re-searching every frame
    # (see module docstring for why per-frame shape/contrast detection of
    # the disc itself isn't reliable). search_radius is generous enough to
    # catch the LED glow within the disc's general vicinity while still
    # excluding unrelated bright clutter elsewhere in a cluttered frame.
    #
    # anchor_pos is the search window's center and is set ONCE, from the
    # first confirmed detection, and never moved again. Confirmed via real
    # debug footage that recentering the search window on the latest
    # detection (instead of a pinned anchor) lets drift compound frame over
    # frame: frame 0 correctly locked onto the real disc, but by frame 100
    # the window had walked onto a floor-glare reflection, and by frame 198
    # onto a nearby coiled wire -- each individual step passed the
    # red/green/blue confidence gate, so the gate alone didn't stop it. The
    # drone/beacon are both essentially static during the sampling window,
    # so the real position shouldn't move at all; pinning the window to the
    # first lock caps any possible drift at one search_radius from the real
    # disc, instead of letting it ratchet further away every frame.
    anchor_pos = None
    locked_pos = None
    search_radius = max(radius_px * 1.5, 40.0)
    min_blob_pixels = max(15, 0.05 * np.pi * radius_px ** 2)

    def _classify_at(frame, bx, by):
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.circle(mask, (int(round(bx)), int(round(by))), int(round(radius_px)), 255, thickness=-1)
        return classify_frame(frame, mask)

    for frame_idx, (ts, frame) in enumerate(frame_source):
        search_center = anchor_pos if anchor_pos is not None else (cx, cy)
        blob = find_lit_blob(frame, search_center, search_radius, min_pixels=min_blob_pixels)

        color = color_conf = intensity = None
        if blob is not None:
            bx, by, _area = blob
            # A blob just means "something passed a loose SAT/VAL check" --
            # mundane clutter (floor glare, a colored mat edge) can too.
            # Only trust it enough to (re)lock the position if it actually
            # classifies as a real target color, not "unknown"/"white".
            cand_color, cand_conf, cand_intensity, _votes = _classify_at(frame, bx, by)
            if cand_color in ("red", "green", "blue"):
                if anchor_pos is None:
                    anchor_pos = (bx, by)
                locked_pos = (bx, by)
                color, color_conf, intensity = cand_color, cand_conf, cand_intensity
                source = "detected"

        if color is None:
            if locked_pos is not None:
                bx, by = locked_pos
                source = "locked"
            else:
                bx, by = cx, cy
                source = "fallback"
            color, color_conf, intensity, _votes = _classify_at(frame, bx, by)

        circle = (bx, by, radius_px)
        blink_result = blink_detector.update(ts, color, intensity, color_conf)
        last_color  = color
        last_circle = circle
        n_used += 1
        if debug_image_dir:
            _save_debug_images(debug_image_dir, frame_idx, ts, frame, circle, color,
                               source, intensity, color_conf)

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
