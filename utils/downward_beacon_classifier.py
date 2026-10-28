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


_MIN_DISC_SCORE = 20.0
# Hough radius tolerance when the expected disc radius comes from the white
# box's pixel size: off-nadir, the visible side faces make the box blob
# larger than its top face, so allow more slack than the height-based 0.3.
_BOX_RADIUS_TOLERANCE = 0.4
# Colour-sampling radius as a fraction of a box-measured disc radius: the LED
# is a small dot in the middle of the black disc (~2-3 px of ~8-10 px).
_LED_RADIUS_FRAC = 0.4
# Window colour: red/green must be at least this share of the lit frames (and
# outnumber blue) -- see BlinkDetector(min_color_frac).
_MIN_COLOR_FRAC = 0.25
# OFF: fewer lit housing frames than this count or fraction = beacon off.
_OFF_MIN_LIT_FRAMES = 3
_OFF_MIN_LIT_FRAC = 0.15
_OFF_MIN_HOUSING_FRAMES = 10   # fewer housing frames than this can't establish OFF
# Depth from the measured disc size needs at least this many box-measured discs.
_DISC_DEPTH_MIN_FRAMES = 3
# Box mode with no usable height: depth used only to size the Hough fallback.
_NOMINAL_DEPTH_M = 3.0


def find_white_box(bgr_frame: np.ndarray, search_center=None, search_radius=None,
                   min_side_px: float = 10.0, max_side_px: float = 300.0,
                   sat_max: int = 60, val_min: int = 170,
                   dark_val_max: int = 90, min_dark_frac: float = 0.08,
                   max_dark_frac: float = 0.6, return_all: bool = False):
    """
    Locate the beacon's white box: the largest bright, low-saturation (white,
    not the cyan floats), roughly square blob, optionally within a circular
    search window. Returns (x, y, side_px) or None. side_px is the shorter
    minAreaRect side -- closest to the top face when side faces also show.
    The dark disc/diamonds are holes inside the external contour, so they
    don't break it up -- and they're what tells the box apart from other
    white blobs (sun glints on the floats, buildings): a candidate must have
    min_dark_frac..max_dark_frac of its rectangle dark (V < dark_val_max).
    """
    h, w = bgr_frame.shape[:2]
    hsv = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)
    white = ((hsv[:, :, 1] < sat_max) & (hsv[:, :, 2] > val_min)).astype(np.uint8) * 255
    if search_center is not None and search_radius is not None:
        window = np.zeros((h, w), dtype=np.uint8)
        cv2.circle(window, (int(round(search_center[0])), int(round(search_center[1]))),
                   int(round(search_radius)), 255, thickness=-1)
        white = cv2.bitwise_and(white, window)
    white = cv2.morphologyEx(white, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    white = cv2.morphologyEx(white, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    found = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < 0.5 * min_side_px ** 2:
            continue
        (rx, ry), (rw, rh), _ang = cv2.minAreaRect(c)
        side = min(rw, rh)
        if not (min_side_px <= side <= max_side_px) or max(rw, rh) / max(side, 1e-6) > 1.6:
            continue
        # Squareness from the convex hull, not the white pixels: the disc and
        # diamonds split the white face into a U/ring (solidity ~0.55, hull
        # filling ~0.65 of its rectangle on real footage). The dark-content
        # and aspect checks do the rejecting.
        hull_area = cv2.contourArea(cv2.convexHull(c))
        if hull_area <= 0 or hull_area / max(rw * rh, 1e-6) < 0.55:
            continue
        rect_mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(rect_mask, [np.int32(cv2.boxPoints(((rx, ry), (rw, rh), _ang)))], 255)
        inside = rect_mask > 0
        dark_frac = float(np.count_nonzero(hsv[:, :, 2][inside] < dark_val_max)) / max(1, np.count_nonzero(inside))
        if not (min_dark_frac <= dark_frac <= max_dark_frac):
            continue
        found.append((area, (float(rx), float(ry), float(side))))
    found.sort(key=lambda t: t[0], reverse=True)
    if return_all:
        return [b for _, b in found]
    return found[0][1] if found else None


def find_disc_in_box(bgr_frame: np.ndarray, box, dark_val_max: int = 90,
                     min_area_px: float = 8.0,
                     min_diam_frac: float = 0.25, max_diam_frac: float = 0.9,
                     max_center_offset_frac: float = 0.35):
    """
    The disc is a dark circle on the box's white top, always at its centre --
    far easier to measure there than with Hough over grass at 5-10 px. Within
    the box's white face, the dark blob holding the largest inscribed circle
    near the box centre is the disc; its centre/radius are that blob's
    centroid/equal-area radius after opening away attached diamonds. It must
    also be disc-sized for the box (diameter
    min_diam_frac..max_diam_frac of the side; real 10 in / 17 in = 0.59) and
    near the centre -- sun glitter on grass passes find_white_box's checks but
    has no such disc. Returns ((x, y), radius_px) or None.
    """
    bx, by, side = box
    h, w = bgr_frame.shape[:2]
    # The white blob is often smaller than the real face (disc/diamonds cut
    # into it), so search a bit past it or the disc gets clipped at the edge.
    half = 0.75 * side
    x0, y0 = max(0, int(bx - half)), max(0, int(by - half))
    x1, y1 = min(w, int(np.ceil(bx + half)) + 1), min(h, int(np.ceil(by + half)) + 1)
    if x1 - x0 < 3 or y1 - y0 < 3:
        return None
    hsv = cv2.cvtColor(bgr_frame[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
    v = hsv[:, :, 2]
    # Only dark pixels within the white face's convex hull: grass (V ~80)
    # is "dark" too and would otherwise merge with the disc.
    white = ((hsv[:, :, 1] < 60) & (v > 170)).astype(np.uint8) * 255
    white = cv2.morphologyEx(white, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(white)
    if n < 2:
        return None
    face = (labels == 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))).astype(np.uint8)
    pts = cv2.findNonZero(face)
    hull_mask = np.zeros_like(face)
    cv2.fillPoly(hull_mask, [cv2.convexHull(pts)], 1)
    hull_mask = cv2.dilate(hull_mask, np.ones((3, 3), np.uint8))
    dark = ((v < dark_val_max) & (hull_mask > 0)).astype(np.uint8) * 255
    # Close first: LED glow or a reflection often breaks the dark ring around
    # a lit LED, and an open "C" isn't filled below -- its thickest part (where
    # the inscribed circle lands) is beside the LED, not on it.
    k = max(3, int(round(0.59 * side / 6.0)) | 1)   # ~1/3 of the expected disc radius
    dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
    # Fill holes so a lit LED (bright centre) doesn't turn the disc into a ring.
    contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filled = np.zeros_like(dark)
    cv2.drawContours(filled, [c for c in contours if cv2.contourArea(c) >= min_area_px], -1, 255, -1)
    # The largest inscribed circle picks WHICH dark blob is the disc (a
    # diamond touching it changes the outline, not the inscribed circle)...
    dist = cv2.distanceTransform(filled, cv2.DIST_L2, 3)
    yy, xx = np.ogrid[:dist.shape[0], :dist.shape[1]]
    near = (xx + x0 - bx) ** 2 + (yy + y0 - by) ** 2 <= (max_center_offset_frac * side) ** 2
    dist[~near] = 0
    _, r_in, _, (px, py) = cv2.minMaxLoc(dist)
    if r_in < 1.5:
        return None
    # ...but the centre and size come from the whole disc: open away anything
    # thinner than the disc (attached diamonds), then centroid + equal-area radius.
    ko = max(3, int(round(r_in)) | 1)
    disc = cv2.morphologyEx(filled, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ko, ko)))
    n, labels, stats, cents = cv2.connectedComponentsWithStats(disc)
    lab = labels[py, px] if disc[py, px] else 0
    if lab == 0:
        cx_, cy_, r = float(px), float(py), float(r_in) + 0.5
    else:
        cx_, cy_ = cents[lab]
        r = float(np.sqrt(stats[lab, cv2.CC_STAT_AREA] / np.pi))
    if r < 2.0 or not (min_diam_frac <= 2.0 * r / side <= max_diam_frac):
        return None
    return (float(cx_ + x0), float(cy_ + y0)), r


def find_disc(
    bgr_frame: np.ndarray, search_center: Tuple[float, float], search_radius: float,
    disc_radius_px: float, radius_tolerance: float = 0.3,
    center_frac: float = 0.30, ring_inner_frac: float = 0.45, ring_outer_frac: float = 0.85,
    min_score: float = _MIN_DISC_SCORE,
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
def _draw_search(overlay, search_info):
    """
    Housing search for this frame: dashed white search window, every Hough
    candidate (cyan = accepted, red = in window but scored below
    _MIN_DISC_SCORE, grey = outside the window) with score/contrast/edge
    support, and the radius range Hough was allowed to return.
    """
    (scx, scy), search, radius_px, candidates, tol, box = search_info[:6]
    disc_fit = search_info[6] if len(search_info) > 6 else None
    if disc_fit is not None and disc_fit[0] is not None:  # disc measured inside the box: cyan
        (dx, dy), dr, dsrc = disc_fit
        cv2.circle(overlay, (round(dx), round(dy)), max(1, round(dr)), (255, 255, 0), 1)
        cv2.putText(overlay, f"{dsrc} r={dr:.1f}", (round(dx + dr) + 3, round(dy) + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
    if box is not None:  # white box that set the scale: magenta square + side length
        bx_, by_, side = box
        half = side / 2.0
        cv2.rectangle(overlay, (round(bx_ - half), round(by_ - half)),
                      (round(bx_ + half), round(by_ + half)), (255, 0, 255), 1)
        cv2.putText(overlay, f"box {side:.0f}px", (round(bx_ - half), round(by_ - half) - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
    c, s = (round(scx), round(scy)), round(search)
    for a in range(0, 360, 12):  # dashed circle
        cv2.ellipse(overlay, c, (s, s), 0, a, a + 6, (255, 255, 255), 1)
    accepted = None
    in_window = [k for k in candidates if (k[1][0] - scx) ** 2 + (k[1][1] - scy) ** 2 <= search ** 2]
    if in_window and in_window[0][0] >= _MIN_DISC_SCORE:
        accepted = in_window[0]
    for k in candidates:
        score, (x, y), r, m = k
        color = ((255, 255, 0) if k is accepted
                 else (0, 0, 255) if k in in_window else (150, 150, 150))
        cv2.circle(overlay, (round(x), round(y)), max(1, round(r)), color, 1)
        cv2.putText(overlay, f"{score:.0f} c{m['contrast']:.0f} e{m['edge_support']:.2f}",
                    (round(x + r) + 3, round(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)
    lo = max(1, int(round(radius_px * (1 - tol))))
    hi = max(lo + 1, int(round(radius_px * (1 + tol))))
    cv2.putText(overlay,
                f"search r={search:.0f}px  disc r={radius_px:.1f}px from {'box' if box else 'height'} "
                f"(Hough {lo}-{hi})  "
                f"candidates {len(candidates)} ({len(in_window)} in window)  min_score {_MIN_DISC_SCORE:.0f}",
                (8, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)


def _save_housing_diagnostics(debug_image_dir, frame_idx, ts, frame,
                              housing, blob, circle, source, search_info=None):
    """Diagnostic overlay: blue housing, green LED blob, yellow ROI, plus the
    housing search (see _draw_search) on frames where one ran."""
    os.makedirs(debug_image_dir, exist_ok=True)
    overlay = frame.copy()
    if search_info is not None:
        _draw_search(overlay, search_info)
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
    # downward_camera.color_overrides: the down camera's own hue bands etc.
    # (its LEDs read differently from the front camera's), on top of the
    # shared detection config.
    bdc._apply_color_config({**detection_cfg, **downward_cam_cfg.get("color_overrides", {})})
    fx, fy = downward_cam_cfg["fx"], downward_cam_cfg["fy"]
    cx, cy = downward_cam_cfg["cx"], downward_cam_cfg["cy"]
    beacon_height_m = downward_cam_cfg["beacon_height_m"]
    beacon_diam_m   = downward_cam_cfg["beacon_top_diameter_m"]
    # Optional image-based scale: with beacon_box_width_m set, the disc radius
    # (and where to look for it) comes from the white box's pixel size each
    # search, so height/lens errors don't put the disc outside Hough's range.
    # An explicit expected_radius_px_override still wins for the radius.
    box_width_m = downward_cam_cfg.get("beacon_box_width_m")
    K, D = downward_cam_cfg.get("_K"), downward_cam_cfg.get("_D")
    fisheye = D is not None and downward_cam_cfg.get("_dist_model") == "fisheye"
    depth_m = (drone_height_agl - beacon_height_m) if drone_height_agl is not None else None
    depth_src = "height"
    if depth_m is None or depth_m <= 0:
        if not box_width_m:
            raise ValueError(
                f"drone_height_agl ({drone_height_agl}) must exceed the beacon "
                f"height ({beacon_height_m:.2f}m) -- the drone must be above the beacon top"
            )
        # Box mode measures depth from the disc below; this only sizes the
        # Hough fallback until then.
        depth_m, depth_src = _NOMINAL_DEPTH_M, "nominal"
    radius_px = (
        expected_radius_px_override if expected_radius_px_override is not None
        else expected_pixel_radius(fx, fy, depth_m, beacon_diam_m)
    )
    disc_measurements = []   # (x, y, r) px of every disc measured in the box
    # The downward view sees the LED itself, so the window's colour is the
    # majority of lit frames -- a few stray red/green samples don't flip blue.
    blink_detector = BlinkDetector(min_color_frac=_MIN_COLOR_FRAC)
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
    reacquire_every = 5
    max_stale_frames = 10
    box = None          # latest (x, y, side_px) white box
    lock_src = None     # disc_src of the detection the current lock came from
    radius_src = "override" if expected_radius_px_override is not None else "height"

    def _classify_at(frame, bx, by, r):
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.circle(mask, (int(round(bx)), int(round(by))),
                   max(1, int(round(r))), 255, thickness=-1)
        return classify_frame(frame, mask)

    def _normalized(px, py):
        """Pixel -> normalized image coords (x/z, y/z): fisheye-undistorted with
        the calibrated K/D when configured, else plain pinhole."""
        if fisheye:
            n = cv2.fisheye.undistortPoints(np.array([[[px, py]]], dtype=np.float64), K, D)
            return float(n[0, 0, 0]), float(n[0, 0, 1])
        return (px - cx) / fx, (py - cy) / fy

    def _pixel_to_gps(px, py, depth):
        """Back-project pixel (px, py) at depth (m along the optical axis) to
        (world_position, (lat, lon, alt)). GPS is None without a gps_origin.
        Uses the single end-of-window pose."""
        xn, yn = _normalized(px, py)
        p_cam = np.array([xn * depth, yn * depth, depth])
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
        search_info = None
        disc_src = None   # how this frame's housing was found: box_dark / hough
        # Try initial acquisition on every frame, or local reacquisition
        # every fifth frame once localized. The box/disc path is cheap and
        # the LED sample is only a few px, so with it every frame searches --
        # a lock up to a second old drifts off the LED as the drone moves.
        if disc_pos is None or box_width_m or frame_idx % reacquire_every == 0:
            if disc_pos is None:
                center, search = (cx, cy), disc_search_radius
            else:
                center = disc_pos
                search = max(25.0, radius_px * 0.75)
            tol = 0.3
            if box_width_m:
                # Whole frame until locked, then near the lock. Accept the
                # largest white-box candidate that has a real disc inside;
                # a "box" without one (sun glitter) doesn't count.
                boxes = (find_white_box(frame, return_all=True) if disc_pos is None else
                         find_white_box(frame, disc_pos, max(60.0, 3.0 * radius_px), return_all=True))
                box = None
                for cand in boxes:
                    disc = find_disc_in_box(frame, cand)
                    if disc is not None:
                        box = cand
                        new_detection, disc_src = disc[0], "box_dark"
                        disc_measurements.append((disc[0][0], disc[0][1], disc[1]))
                        if expected_radius_px_override is None:
                            radius_px, radius_src = disc[1], "disc"
                        center, search, tol = (box[0], box[1]), max(box[2], 25.0), _BOX_RADIUS_TOLERANCE
                        if debug_image_dir:
                            search_info = (center, search, radius_px, [], tol, box,
                                           (new_detection, radius_px, disc_src))
                        break
                if box is None and boxes and expected_radius_px_override is None:
                    # No disc confirmed: still size Hough from the biggest box.
                    radius_px = (beacon_diam_m / 2.0) * boxes[0][2] / box_width_m
                    radius_src = "box"

        if new_detection is None and (disc_pos is None or frame_idx % reacquire_every == 0):
            disc_src = "hough"
            print(
                f"FRAME={frame.shape[1]}x{frame.shape[0]} "
                f"CX={cx} CY={cy} "
                f"RADIUS={radius_px:.1f} "
                f"SEARCH={search:.1f}"
            )
            
            if debug_image_dir:
                # Same decision as find_disc(), but keep every candidate
                # (including out-of-window ones) for the diagnostic image --
                # one Hough pass either way.
                candidates = _disc_candidates(frame, center, float("inf"), radius_px, tol)
                in_window = [k for k in candidates
                             if (k[1][0] - center[0]) ** 2 + (k[1][1] - center[1]) ** 2 <= search ** 2]
                if in_window and in_window[0][0] >= _MIN_DISC_SCORE:
                    new_detection = in_window[0][1]
                search_info = (center, search, radius_px, candidates, tol, box)
            else:
                new_detection = find_disc(frame, center, search, radius_px, tol)
        if new_detection is not None:
            disc_pos = new_detection
            last_verified_frame = frame_idx
            lock_src = disc_src

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
            # radius_px can change per search (box scale), so size these per frame.
            blob = find_lit_blob(frame, disc_pos, max(radius_px * 0.65, 12.0),
                                 min_pixels=max(20, 0.01 * np.pi * radius_px ** 2))
            # A disc measured in the box is mostly black with a small LED in
            # the middle -- sample just the LED, or the black disc and box
            # edges swamp its hue and on/off intensity. When the LED is lit,
            # centre the sample on it (the disc centre can be a few px off);
            # unlit, the disc centre. bx/by stay the disc centre for GPS.
            sx, sy = bx, by
            if lock_src == "box_dark":
                sample_r = max(2.0, _LED_RADIUS_FRAC * radius_px)
                led = find_lit_blob(frame, disc_pos, radius_px, min_pixels=3, min_circularity=0.3)
                if led is not None:
                    sx, sy = led[0], led[1]
                    blob = led
            else:
                sample_r = radius_px
            color, color_conf, intensity, votes, hue_stats = _classify_at(frame, sx, sy, sample_r)
            source = "detected" if new_detection is not None else "locked"
        else:
            bx, by = cx, cy
            blob = None
            color, color_conf, intensity = "unknown", 0.0, 0.0
            votes, hue_stats = _empty_votes, _empty_hue_stats
            source = "fallback"

        circle = (bx, by, sample_r if disc_pos is not None else radius_px)   # housing (GPS)
        sample_circle = (sx, sy, sample_r) if disc_pos is not None else circle   # what colour came from
        blink_result = blink_detector.update(ts, color, intensity, color_conf)
        last_color = color
        # Keep the last *verified* housing location for GPS back-projection;
        # an arbitrary center fallback must never produce coordinates.
        if disc_pos is not None:
            last_circle = circle
        n_used += 1
        # Per-frame GPS (latitude/longitude/altitude) is filled in after the
        # loop, once the window's depth is known. The housing when localized,
        # otherwise the camera nadir (bx, by == cx, cy), tagged so the two
        # aren't confused.
        frame_log.append({
            "frame_idx": frame_idx, "timestamp": ts, "source": source,
            "bx": bx, "by": by,
            "radius_px": radius_px, "radius_src": radius_src,
            "box_side_px": box[2] if box is not None else None,
            "disc_src": disc_src if new_detection is not None else None,
            "latitude": None, "longitude": None, "altitude": None,
            "gps_source": "housing" if disc_pos is not None else "nadir",
            "color": color, "color_conf": color_conf,
            "intensity": intensity, **{f"hue_{k}": v for k, v in hue_stats.items()},
            "vote_red": votes["red"], "vote_green": votes["green"],
            "vote_blue": votes["blue"], "vote_other": votes["other"],
        })
        if debug_image_dir:
            _save_debug_images(debug_image_dir, frame_idx, ts, frame, sample_circle,
                               color, source, intensity, color_conf)
            _save_housing_diagnostics(debug_image_dir, frame_idx, ts, frame,
                                      disc_pos, blob, sample_circle, source, search_info)

    # Depth from the measured disc: a 0.254 m disc whose (undistorted)
    # normalized radius is r_n sits at depth (D/2) / r_n along the optical
    # axis -- independent of AGL telemetry. Median over the window.
    if len(disc_measurements) >= _DISC_DEPTH_MIN_FRAMES:
        r_norm = []
        for (x, y, r) in disc_measurements:
            c = np.array(_normalized(x, y))
            edge = [np.array(_normalized(x + dx * r, y + dy * r))
                    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))]
            r_norm.append(np.mean([np.linalg.norm(e - c) for e in edge]))
        rn = float(np.median(r_norm))
        if rn > 0:
            depth_m, depth_src = (beacon_diam_m / 2.0) / rn, "disc"

    for rec in frame_log:
        _, g = _pixel_to_gps(rec["bx"], rec["by"], depth_m)
        if g is not None:
            rec["latitude"], rec["longitude"], rec["altitude"] = g
        rec["depth_m"], rec["depth_src"] = depth_m, depth_src

    gps_position = None
    world_position = None
    if last_circle is not None:
        bx, by, _br = last_circle
        world_position, gps = _pixel_to_gps(bx, by, depth_m)
        if gps is not None:
            lat, lon, alt = gps
            gps_position = {"latitude": lat, "longitude": lon, "altitude": alt}
        world_position = [float(v) for v in world_position]

    # BlinkDetector defaults blink_color to "blue" whenever it sees no
    # confident red/green -- so an unlit beacon, or no beacon at all, comes
    # out as steady blue (the Task 1 EXIT state). Decide those cases here from
    # the housing frames instead: never localized -> unknown (not reportable);
    # localized but (almost) never lit -> "off". "Almost": one or two stray
    # coloured samples (box edge tape, glare) mustn't turn an unlit beacon
    # into a blinking/steady one. A 0.5 Hz blinker is lit ~half of an 8 s
    # window, far above these limits.
    housing_frames = [r for r in frame_log if r["source"] in ("detected", "locked")]
    n_lit = sum(1 for r in housing_frames
                if r["color"] != "unknown" and r["color_conf"] >= _BLINK_CC_ON_THRESHOLD)
    lit = n_lit >= _OFF_MIN_LIT_FRAMES and n_lit >= _OFF_MIN_LIT_FRAC * len(housing_frames)
    # Too few housing frames to call it OFF (e.g. a clip ending) -> unknown.
    if not lit and len(housing_frames) < _OFF_MIN_HOUSING_FRAMES:
        housing_frames = []
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
        "depth_m": depth_m,          # camera->beacon top used for GPS
        "depth_src": depth_src,      # "disc" (measured), "height" (AGL) or "nominal"
        "n_frames_used": n_used,
        "frame_log": frame_log,
    }
