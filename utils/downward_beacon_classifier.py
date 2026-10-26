"""Downward beacon color/blink/GPS classifier (API-compatible rewrite)."""
import os
import time
from typing import Optional, Tuple
import cv2
import numpy as np
from utils.blink_detector import BlinkDetector, _BLINK_CC_ON_THRESHOLD
def expected_pixel_radius(fx: float, fy: float, depth_m: float, diameter_m: float) -> float:
    """
    Expected circle radius in pixels for a known-diameter circle at a known
    depth, via the pinhole model. Uses the average of fx/fy since a true
    nadir shot views the circle face-on (no foreshortening).
    """
    f_avg = (fx + fy) / 2.0
    return (diameter_m / 2.0) * f_avg / depth_m
def _disc_candidates(bgr_frame, search_center, search_radius, disc_radius_px,
                     radius_tolerance=0.3, center_frac=0.30,
                     ring_inner_frac=0.45, ring_outer_frac=0.85):
    """Find and rank housing circles; return [(score, (x, y), radius, metrics), ...].

    Bright-center contrast is useful when lit; circumference support and a
    mostly-dark annulus provide independent evidence when the LED is off.
    """
    h, w = bgr_frame.shape[:2]
    gray = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 1.5)
    min_r = max(1, int(round(disc_radius_px * (1 - radius_tolerance))))
    max_r = max(min_r + 1, int(round(disc_radius_px * (1 + radius_tolerance))))
    circles = cv2.HoughCircles(
        blurred, cv2.HOUGH_GRADIENT, dp=1.2,
        minDist=max(1.0, disc_radius_px),
        param1=120, param2=35, minRadius=min_r, maxRadius=max_r,
    )
    if circles is None:
        return []

    # Edge evidence computed on the same blurred image as Hough.
    edges = cv2.Canny(blurred, 60, 120)
    yy, xx = np.ogrid[:h, :w]
    scx, scy = search_center
    candidates = []
    for x, y, r in circles[0]:
        x, y, r = float(x), float(y), float(r)
        if (x - scx)**2 + (y - scy)**2 > search_radius**2:
            continue
        dist_sq = (xx - x)**2 + (yy - y)**2
        center = dist_sq <= max(3.0, r * center_frac)**2
        ring = ((dist_sq >= (r * ring_inner_frac)**2) &
                (dist_sq <= (r * ring_outer_frac)**2))
        if not np.any(center) or not np.any(ring):
            continue
        contrast = float(gray[center].mean() - gray[ring].mean())

        # Sample the perimeter across 72 angular bins. Allow a +/- 3 px radial
        # tolerance for imperfect circle edges and antialiasing.
        angles = np.linspace(0, 2*np.pi, 72, endpoint=False)
        supported = 0
        for angle in angles:
            for delta in (-3, -2, -1, 0, 1, 2, 3):
                px = int(round(x + (r + delta)*np.cos(angle)))
                py = int(round(y + (r + delta)*np.sin(angle)))
                if 0 <= px < w and 0 <= py < h and edges[py, px]:
                    supported += 1
                    break
        edge_support = supported / len(angles)
        ring_gray = gray[ring]
        # Preference for low-variance/dark annuli rather than global darkness.
        ring_std = float(ring_gray.std())
        # Keep contrast in original grayscale units, with modest bonuses for
        # coherent outer edges and a uniform interior annulus.
        score = contrast + 22.0 * edge_support - 0.08 * ring_std
        candidates.append((score, (x, y), r, {
            "contrast": contrast, "edge_support": edge_support,
            "ring_std": ring_std,
        }))
    return sorted(candidates, key=lambda c: c[0], reverse=True)


def find_disc(
    bgr_frame: np.ndarray, search_center: Tuple[float, float], search_radius: float,
    disc_radius_px: float, radius_tolerance: float = 0.3,
    center_frac: float = 0.30, ring_inner_frac: float = 0.45, ring_outer_frac: float = 0.85,
    min_score: float = 20.0,
) -> Optional[Tuple[float, float]]:
    """Return the best accepted housing center (x, y), or None.

    Public signature and return type are unchanged. The minimum score now
    applies to a combined contrast/edge-support score, not contrast alone.
    """
    candidates = _disc_candidates(
        bgr_frame, search_center, search_radius, disc_radius_px,
        radius_tolerance, center_frac, ring_inner_frac, ring_outer_frac,
    )
    if not candidates or candidates[0][0] < min_score:
        return None
    return candidates[0][1]


def find_lit_blob(
    bgr_frame: np.ndarray, search_center: Tuple[float, float], search_radius: float,
    min_pixels: int = 15, sat_min: int = 40, val_min: int = 80,
    min_circularity: float = 0.65,
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
    The SAT/VAL gate alone isn't enough, though -- confirmed against real
    bench-test footage that plain indoor flooring can satisfy it over a huge,
    irregularly-shaped area (wood grain highlights, ambient glare), which
    `max(contourArea)` then happily prefers over the real (small, round) LED.
    The disc/LED is circular; floor glare isn't. So any candidate is also
    required to be reasonably circular (4*pi*area/perimeter^2, 1.0 = perfect
    circle) -- measured directly against real footage: the false-positive
    floor blob that caused a wrong lock scored circularity 0.08, while every
    other fragment in that same frame scored well under this threshold too
    (0.16-0.53) -- i.e. the correct answer for that frame was "nothing
    qualifies", not "take the biggest one anyway".
    Returns (blob_x, blob_y, pixel_count) -- the lit blob's centroid and
    size -- or None if nothing large enough and round enough qualifies within
    the window.
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
    candidates = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_pixels:
            continue
        perim = cv2.arcLength(c, True)
        if perim == 0:
            continue
        circularity = 4.0 * np.pi * area / (perim * perim)
        if circularity < min_circularity:
            continue
        candidates.append((area, c))
    if not candidates:
        return None
    _area, best = max(candidates, key=lambda t: t[0])
    moments = cv2.moments(best)
    if moments["m00"] == 0:
        return None
    return moments["m10"] / moments["m00"], moments["m01"] / moments["m00"], _area
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
def classify_frame(bgr_frame: np.ndarray, mask: np.ndarray) -> Tuple[str, float, float, dict, dict]:
    """
    Crop to the mask's bounding box (same crop-then-classify pattern
    beacon_detector_config.isolate_and_classify() uses) and reuse
    beacon_detector_config.classify_beacon_color() for the actual hue-vote
    decision, passing our circle mask as seg_mask -- same thresholds/logic
    as the oblique-view pipeline, no reimplementation.

    The 5th return value, hue_stats, carries the same per-frame hue
    median/mean/mode/variance/min/max/n that classify_beacon_color() already
    computes and prints to stdout on every call -- previously discarded here
    (via `*_`), so that data only ever existed as terminal scrollback, never
    in the per-run JSON result or the CSV log. Returning it lets the caller
    log it per frame instead.
    """
    import beacon_detector_config as bdc
    empty_votes = {"red": 0.0, "green": 0.0, "blue": 0.0, "other": 0.0}
    empty_hue_stats = {"median": 0.0, "mean": 0.0, "mode": 0.0, "variance": 0.0, "min": 0.0, "max": 0.0, "n": 0}
    crop, crop_mask = _crop_to_mask_bbox(bgr_frame, mask)
    if crop is None:
        return "unknown", 0.0, 0.0, empty_votes, empty_hue_stats
    color, color_conf, _light_mask, intensity, votes, hue_var, hue_mean, hue_median, hue_mode = bdc.classify_beacon_color(
        crop, seg_mask=crop_mask
    )
    # _light_mask already reflects classify_beacon_color()'s own refinement
    # (seg_mask intersected, then narrowed to the single best blob-shaped
    # contour) -- the exact same pixels its hue_var/mean/median/mode were
    # computed from, so reusing it here (rather than re-deriving a mask)
    # guarantees min/max describe that identical pixel set.
    n_lit = int(np.count_nonzero(_light_mask))
    if n_lit > 0:
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        lit_hues = hsv[:, :, 0][_light_mask > 0]
        hue_min, hue_max = float(lit_hues.min()), float(lit_hues.max())
    else:
        hue_min = hue_max = 0.0
    hue_stats = {
        "median": hue_median, "mean": hue_mean, "mode": hue_mode,
        "variance": hue_var, "min": hue_min, "max": hue_max, "n": n_lit,
    }
    return color, color_conf, intensity, votes, hue_stats
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
def _save_housing_diagnostics(debug_image_dir, frame_idx, ts, frame,
                              housing, blob, circle, source):
    """Diagnostic overlay: blue housing, green LED blob, yellow ROI."""
    os.makedirs(debug_image_dir, exist_ok=True)
    overlay = frame.copy()
    x, y, r = circle
    cv2.circle(overlay, (round(x), round(y)), round(r), (0, 255, 255), 2)
    if housing is not None:
        hx, hy = housing
        cv2.drawMarker(overlay, (round(hx), round(hy)), (255, 0, 0),
                       cv2.MARKER_CROSS, 24, 2)
        cv2.putText(overlay, "HOUSING", (round(hx)+12, round(hy)-8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 0), 2)
    if blob is not None:
        lx, ly, _area = blob
        cv2.drawMarker(overlay, (round(lx), round(ly)), (0, 255, 0),
                       cv2.MARKER_CROSS, 16, 2)
    cv2.putText(overlay, "blue=housing green=LED yellow=ROI " + source,
                (8, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 2)
    tag = f"frame_{frame_idx:04d}_{ts:.3f}"
    cv2.imwrite(os.path.join(debug_image_dir, tag + "_diagnostic.png"), overlay)


def classify_beacon_downward(
    frame_source, drone_pos, drone_quat_wxyz, gps_origin, drone_height_agl,
    downward_cam_cfg: dict, detection_cfg: dict, duration_s: float,
    debug_image_dir: Optional[str] = None,
    expected_radius_px_override: Optional[float] = None,
    tracking_id: int = -1,
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
    tracking_id:        passthrough only -- this module has no tracker of its
                        own (one call = one already-positioned beacon, not
                        multiple simultaneous detections), so there's nothing
                        here to assign an ID from. -1 (default) matches this
                        repo's convention for "no real tracker" (see
                        utils/tflite_hexagon_detector.py). Set it if the
                        caller (e.g. a multi-beacon mission script) wants the
                        result tagged with which beacon in the mission this
                        was.
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
    # One record per sampled frame -- the same intensity/hue data
    # classify_beacon_color() already prints to stdout on every call, kept
    # here instead of only ever existing as terminal scrollback, so the
    # caller can write it to a CSV log alongside this run's ground truth.
    # Popped from the result dict before publishing, same as n_frames_used.
    frame_log = []
    # Housing center is the authoritative beacon position. An LED glint
    # never overwrites it. Reacquire every five frames, with local search
    # near the previous center to reduce jumps between similar objects.
    disc_pos = None
    last_verified_frame = None
    disc_search_radius = max(radius_px * 1.5, 150.0)
    #disc_search_radius = 1280.0
    lit_search_radius = max(radius_px * 0.65, 12.0)
    min_blob_pixels = max(20, 0.01 * np.pi * radius_px**2)
    reacquire_every = 5
    max_stale_frames = 10

    def _classify_at(frame, bx, by):
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.circle(mask, (int(round(bx)), int(round(by))),
                   int(round(radius_px)), 255, thickness=-1)
        return classify_frame(frame, mask)

    def _pixel_to_gps(px, py):
        """Back-project pixel (px, py) at depth_m to (world_position, (lat, lon, alt)).
        GPS is None without a gps_origin. Uses the single end-of-window pose."""
        # Pinhole back-projection at the known depth (equivalent to
        # camera_interface.Intrinsics.back_project(), inlined to avoid
        # needing a CameraInterface instance just for this).
        p_cam = np.array([
            (px - cx) * depth_m / fx,
            (py - cy) * depth_m / fy,
            depth_m,
        ])
        world = bdc._camera_to_world(
            p_cam, np.asarray(drone_pos, dtype=np.float64),
            np.asarray(drone_quat_wxyz, dtype=np.float64),
            downward_cam_cfg["_mount_offset"], downward_cam_cfg["_R_body_to_cam"],
        )
        if gps_origin is None:
            return world, None
        return world, bdc.local_enu_to_gps(world, gps_origin[0], gps_origin[1], gps_origin[2])

    for frame_idx, (ts, frame) in enumerate(frame_source):
        new_detection = None
        # Try initial acquisition on every frame, or local reacquisition
        # every fifth frame once localized.
        if disc_pos is None or frame_idx % reacquire_every == 0:
            if disc_pos is None:
                center, search = (cx, cy), disc_search_radius
            else:
                center = disc_pos
                search = max(25.0, radius_px * 0.75)
            
            print(
                f"FRAME={frame.shape[1]}x{frame.shape[0]} "
                f"CX={cx} CY={cy} "
                f"RADIUS={radius_px:.1f} "
                f"SEARCH={search:.1f}"
            )
            
            new_detection = find_disc(frame, center, search, radius_px)
            if new_detection is not None:
                disc_pos = new_detection
                last_verified_frame = frame_idx

        if (last_verified_frame is not None and
                frame_idx - last_verified_frame > max_stale_frames):
            disc_pos = None

        # Never use arbitrary bright blobs to establish housing localization.
        # Color can still be sampled at the geometric center, but if no
        # housing is localized the output GPS position remains None.
        _empty_votes = {"red": 0.0, "green": 0.0, "blue": 0.0, "other": 0.0}
        _empty_hue_stats = {"median": 0.0, "mean": 0.0, "mode": 0.0, "variance": 0.0, "min": 0.0, "max": 0.0, "n": 0}
        if disc_pos is not None:
            bx, by = disc_pos
            blob = find_lit_blob(frame, disc_pos, lit_search_radius,
                                 min_pixels=min_blob_pixels)
            color, color_conf, intensity, votes, hue_stats = _classify_at(frame, bx, by)
            source = "detected" if new_detection is not None else "locked"
        else:
            bx, by = cx, cy
            blob = None
            color, color_conf, intensity = "unknown", 0.0, 0.0
            votes, hue_stats = _empty_votes, _empty_hue_stats
            source = "fallback"

        circle = (bx, by, radius_px)
        blink_result = blink_detector.update(ts, color, intensity, color_conf)
        last_color = color
        # Keep the last *verified* housing location for GPS back-projection;
        # an arbitrary center fallback must never produce coordinates.
        if disc_pos is not None:
            last_circle = circle
        n_used += 1
        # Per-frame GPS for the log: the housing when localized, otherwise
        # the camera nadir (bx, by == cx, cy), tagged so the two aren't confused.
        _, frame_gps = _pixel_to_gps(bx, by)
        frame_lat, frame_lon, frame_alt = frame_gps if frame_gps is not None else (None, None, None)
        frame_log.append({
            "frame_idx": frame_idx, "timestamp": ts, "source": source,
            "bx": bx, "by": by,
            "latitude": frame_lat, "longitude": frame_lon, "altitude": frame_alt,
            "gps_source": "housing" if disc_pos is not None else "nadir",
            "color": color, "color_conf": color_conf,
            "intensity": intensity, **{f"hue_{k}": v for k, v in hue_stats.items()},
            "vote_red": votes["red"], "vote_green": votes["green"],
            "vote_blue": votes["blue"], "vote_other": votes["other"],
        })
        if debug_image_dir:
            _save_debug_images(debug_image_dir, frame_idx, ts, frame, circle,
                               color, source, intensity, color_conf)
            _save_housing_diagnostics(debug_image_dir, frame_idx, ts, frame,
                                      disc_pos, blob, circle, source)

    gps_position = None
    world_position = None
    if last_circle is not None:
        bx, by, _br = last_circle
        world_position, gps = _pixel_to_gps(bx, by)
        if gps is not None:
            lat, lon, alt = gps
            gps_position = {"latitude": lat, "longitude": lon, "altitude": alt}
        world_position = [float(v) for v in world_position]

    # BlinkDetector defaults blink_color to "blue" whenever it sees no
    # confident red/green -- so an unlit beacon, or no beacon at all, comes
    # out as steady blue (the Task 1 EXIT state). Decide those cases here from
    # the housing frames instead: never localized -> unknown (not reportable);
    # localized but never lit -> "off".
    housing_frames = [r for r in frame_log if r["source"] in ("detected", "locked")]
    lit = any(r["color"] != "unknown" and r["color_conf"] >= _BLINK_CC_ON_THRESHOLD
              for r in housing_frames)
    if not housing_frames:
        blink_result = {"is_blinking": None, "blink_color": "unknown",
                        "blink_hz": None, "phase": "unknown"}
        last_color = "unknown"
    elif not lit:
        blink_result = {"is_blinking": False, "blink_color": "off",
                        "blink_hz": None, "phase": "off"}
        last_color = "off"
    """
    return {
        "color": last_color,
        "blink": blink_result,
        "gps_position": gps_position,
        "world_position": world_position,
        "n_frames_used": n_used,
        "timestamp": time.time(),
    }
    """
    return {
        "gps_position": gps_position,
        "blink": blink_result["is_blinking"],
        "blink_color": blink_result["blink_color"],
        "blink_hz": blink_result["blink_hz"],
        "phase": blink_result["phase"],
        "color": last_color,
        "tracking_id":  tracking_id,
        "timestamp": time.time(),
        "n_frames_used": n_used,
        "frame_log": frame_log,
    }
