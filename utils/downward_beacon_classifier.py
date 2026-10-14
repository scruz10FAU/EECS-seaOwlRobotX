"""
downward_beacon_classifier.py — color/blink/GPS classification of a beacon's
circular top face from a downward-facing camera, assuming the drone is
already hovering directly above it.

No trained model needed. Four earlier attempts at isolating the disc/LED
all failed against real footage:
  1. Hough circles on the LED's own edge/contrast -- too fragile.
  2. Adaptive thresholding assuming the disc is "always darker" than its
     surroundings -- false: its absolute brightness depends entirely on
     what's nearby, not something you can threshold frame-to-frame.
  3. Position persistence keyed on find_lit_blob() alone (a saturated+bright
     blob is "the LED"), locking onto the first confident hit and reusing it
     for LED-off frames -- false: confirmed via real footage that ordinary
     indoor flooring (and probably other matte surfaces, e.g. choppy water)
     routinely contains small, round, highly saturated+bright specular
     highlights -- measured examples reaching S=255/V=255 -- so no
     brightness/saturation/circularity gate on its own reliably excludes
     "floor glint" while still accepting "real LED". It doesn't help that
     this also means the very first frame can anchor onto the wrong thing
     before the LED has even been seen.
  4. Locating the housing by lowest average saturation (achromaticity) --
     false: confirmed via real footage that a brightly-lit LED's glow tints
     the whole disc face, and under strong/warm indoor lighting even the
     housing's own supposedly-neutral material reads with elevated, non-low
     saturation (measured ~60-120 at the housing's own edge, not the ~30
     measured on a different, dimmer-lit test) -- so "achromatic" isn't a
     stable signal either, and plain Hough circles alone are far too
     fragile: in one real frame, 40 circles were detected at the beacon's
     expected radius, almost all of them from ordinary floor-grain texture
     and nearby furniture, with no vote-count threshold that keeps the real
     one while dropping the noise (tightening the threshold loses the real
     circle before it loses several rounded furniture edges, which rack up
     more combined votes than the beacon's one clean ring).

Current approach -- HOUGH CIRCLES, THEN SCORE BY BRIGHTNESS CONTRAST:
find_disc() still runs Hough (cv2.HoughCircles) restricted to radii near the
expected disc size, same as attempt #4 -- but instead of trusting Hough's
own vote count to pick the right one (it doesn't reliably), each candidate
is scored by how much brighter its small center region is than its own
surrounding ring, in plain grayscale -- i.e. literally "does this candidate
look like a lit beacon: bright core, darker housing around it". This is
unaffected by the whole-scene color-cast problem that sank attempt #4
(it never looks at hue/saturation, only relative brightness between a
candidate's own center and its own ring), and confirmed directly against
real footage to separate the beacon from every other circular object in a
cluttered room (a remote control, a round vacuum canister, suitcase corners)
by a wide, decisive margin -- e.g. the real beacon scored 63 vs. the next
candidate's 16, and 107 vs. 17, on two different real frames, while false
candidates scored near zero or negative (their "center" was no brighter, or
even darker, than their own surroundings -- the opposite of a lit beacon).
Position persistence still applies (the drone is holding roughly still):
once found, the housing's location is reused for the rest of the sampling
window instead of re-searching every frame. find_lit_blob() then only has
to search a small window tightly scoped to the housing itself -- not the
whole frame -- so the floor is structurally excluded rather than merely
out-thresholded. Color classification reuses
beacon_detector_config.classify_beacon_color()'s hue-voting logic on a crop
sized to the known disc diameter, centered on whatever position was
determined (detected / locked / geometric fallback). Blink detection reuses
BlinkDetector unchanged.

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


def find_disc(
    bgr_frame: np.ndarray, search_center: Tuple[float, float], search_radius: float,
    disc_radius_px: float, radius_tolerance: float = 0.3,
    center_frac: float = 0.30, ring_inner_frac: float = 0.45, ring_outer_frac: float = 0.85,
    min_score: float = 20.0,
) -> Optional[Tuple[float, float]]:
    """
    Locate the beacon: run Hough circle detection restricted to radii near
    disc_radius_px, then -- since Hough's own vote count isn't a reliable
    way to pick the right candidate among several real circular objects in a
    cluttered room (see module docstring) -- score each candidate by how
    much brighter its small center region is than its own surrounding ring,
    in plain grayscale. That's the beacon's actual physical structure (a
    bright lit core inside a darker housing), checked directly rather than
    inferred from Hough's internal vote count, and it's unaffected by the
    whole-scene warm color cast that broke the earlier achromaticity-based
    approach, since it only ever compares a candidate's own center to its
    own ring, never absolute hue/saturation.

    search_center/search_radius: candidates are restricted to within
    search_radius of search_center, same role as in earlier versions -- the
    drone/camera is expected to be roughly over the beacon, so this excludes
    other real circular objects (furniture, luggage) known to be well
    outside any plausible hover position.
    radius_tolerance: Hough's minRadius/maxRadius are disc_radius_px scaled
    by (1 -+ radius_tolerance).
    center_frac/ring_inner_frac/ring_outer_frac: the center/ring sample
    regions, as fractions of each candidate's own detected radius.
    min_score: minimum (center_brightness - ring_brightness) required to
    accept a candidate at all -- confirmed against real footage that the
    real beacon scores several times higher than this (63, 107 in two
    different frames) while false candidates score near zero or negative,
    so this is a wide, forgiving margin, not a tight tuned cutoff.

    Returns (cx, cy) of the best-scoring candidate, or None if no Hough
    candidate was found in the window, or none scored above min_score.
    """
    h, w = bgr_frame.shape[:2]
    gray = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 1.5)

    min_r = max(1, int(round(disc_radius_px * (1 - radius_tolerance))))
    max_r = max(min_r + 1, int(round(disc_radius_px * (1 + radius_tolerance))))
    circles = cv2.HoughCircles(
        blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=disc_radius_px,
        param1=120, param2=35, minRadius=min_r, maxRadius=max_r,
    )
    if circles is None:
        return None

    yy, xx = np.ogrid[:h, :w]
    scx, scy = search_center
    best_pos, best_score = None, -np.inf
    for x, y, r in circles[0]:
        x, y, r = float(x), float(y), float(r)
        if (x - scx) ** 2 + (y - scy) ** 2 > search_radius ** 2:
            continue
        light_r = max(3.0, r * center_frac)
        dist_sq = (xx - x) ** 2 + (yy - y) ** 2
        center_mask = dist_sq <= light_r ** 2
        ring_mask = (dist_sq >= (r * ring_inner_frac) ** 2) & (dist_sq <= (r * ring_outer_frac) ** 2)
        if not center_mask.any() or not ring_mask.any():
            continue
        score = float(np.mean(gray[center_mask])) - float(np.mean(gray[ring_mask]))
        if score > best_score:
            best_score, best_pos = score, (x, y)

    if best_pos is None or best_score < min_score:
        return None
    return best_pos


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

    # Housing localization: find_disc() locates the beacon's housing --
    # large, circular, nearly achromatic -- which works every frame
    # regardless of whether the LED is lit (see module docstring for why
    # relying on the LED alone, or on the disc's brightness, both failed
    # against real footage). Located once and persisted for the rest of the
    # sampling window rather than re-searched every frame, same reasoning as
    # before: the drone/beacon are both essentially static while hovering,
    # so re-locating it every frame only risks frame-to-frame jitter between
    # similarly-plausible low-saturation spots for no benefit.
    #
    # disc_search_radius is generous -- find_disc() needs room to locate the
    # housing even if the drone/camera isn't perfectly centred over it. This
    # is deliberately NOT scaled down for a small radius_px (e.g. a
    # height-derived radius at typical hover altitude is much smaller than a
    # bench-test --expected-radius-px override): it represents how far off
    # center the hover/test framing might be, not how big the disc itself
    # is -- measured directly against real footage where the disc sat ~100-
    # 150px from the frame's geometric center despite a radius_px of only
    # ~25. lit_search_radius is tight -- once the housing is found,
    # find_lit_blob() only needs to look inside it for the actual LED, which
    # structurally excludes the floor around it rather than merely
    # out-thresholding it.
    disc_pos = None
    disc_search_radius = max(radius_px * 1.5, 150.0)
    lit_search_radius  = max(radius_px * 1.1, 20.0)

    locked_pos = None
    # Now that find_lit_blob()'s search window is scoped tightly to the
    # housing itself, a real LED glint can be a small fraction of the full
    # disc's area -- no longer needs a size floor sized to exclude a
    # whole-floor false positive (that's find_disc()'s/the tight window's
    # job now).
    min_blob_pixels = max(20, 0.01 * np.pi * radius_px ** 2)

    def _classify_at(frame, bx, by):
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.circle(mask, (int(round(bx)), int(round(by))), int(round(radius_px)), 255, thickness=-1)
        return classify_frame(frame, mask)

    for frame_idx, (ts, frame) in enumerate(frame_source):
        if disc_pos is None:
            disc_pos = find_disc(frame, (cx, cy), disc_search_radius, radius_px)

        if disc_pos is not None:
            lit_center, lit_radius = disc_pos, lit_search_radius
        else:
            lit_center, lit_radius = (cx, cy), disc_search_radius

        blob = find_lit_blob(frame, lit_center, lit_radius, min_pixels=min_blob_pixels)

        color = color_conf = intensity = None
        if blob is not None:
            bx, by, _area = blob
            # A blob just means "something passed a loose SAT/VAL check" --
            # mundane clutter can too. Only trust it enough to (re)lock the
            # position if it actually classifies as a real target color, not
            # "unknown"/"white".
            cand_color, cand_conf, cand_intensity, _votes = _classify_at(frame, bx, by)
            if cand_color in ("red", "green", "blue"):
                locked_pos = (bx, by)
                color, color_conf, intensity = cand_color, cand_conf, cand_intensity
                source = "detected"

        if color is None:
            if locked_pos is not None:
                bx, by = locked_pos
                source = "locked"
            elif disc_pos is not None:
                bx, by = disc_pos
                source = "fallback"
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
    }

