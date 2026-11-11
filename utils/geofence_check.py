"""
geofence_check.py — VALID / INVALID checks for the RoboCommand UAV geofence,
shared by beacon_mavlink_ground.py (before uploading to PX4) and seaowls_ocs
(before declaring to RoboCommand). Pure Python: no ROS, no MAVLink.

Rings are lists of (lat, lon). RoboCommand rings are closed (first point ==
last point); validate_course() requires that, the other functions accept
either and return closed rings.

Every check returns a list of human-readable problems -- empty means VALID.
INVALID is what RoboCommand refuses (fence outside the course + transit area,
not closed / too few points, bad ceiling) plus coordinates that can't be a
course at all. Corner order and size are NOT invalid: simple_ring() turns
crossing corners into a non-crossing polygon, and course_warnings() flags an
implausible size without blocking.

The fence-vs-course check mirrors RoboCommand's official test server
(rc_test/test_server.py geofence_problems): every point along the fence's
edges, sampled every EDGE_STEP_M, must lie within the course or the transit
area to within a tolerance (1 m there).
"""

import math

EARTH_RADIUS_M = 6371000.0   # as the official test server's _xy()
EDGE_STEP_M = 0.5            # as the official test server
FENCE_TOLERANCE_M = 1.0      # as the official test server's GEOFENCE_TOLERANCE_M
MAX_UAV_ALTITUDE_HAE_M = 50.0  # official test server default; set to the competition's limit
MIN_SPAN_M = 5.0             # a course smaller than this across is not a real course
MAX_SPAN_M = 1000.0          # nor one larger than this
MIN_FENCE_AREA_M2 = 0.25     # an inset fence smaller than this has collapsed to ~nothing


# ── Geometry helpers (local east/north metres around an origin) ──────────────

def _xy(p, origin):
    lat0 = math.radians(origin[0])
    return (math.radians(p[1] - origin[1]) * EARTH_RADIUS_M * math.cos(lat0),
            math.radians(p[0] - origin[0]) * EARTH_RADIUS_M)


def _latlon(q, origin):
    lat0 = math.radians(origin[0])
    return (origin[0] + math.degrees(q[1] / EARTH_RADIUS_M),
            origin[1] + math.degrees(q[0] / (EARTH_RADIUS_M * math.cos(lat0))))


def _open(ring):
    ring = [tuple(p) for p in ring]
    return ring[:-1] if len(ring) > 1 and ring[0] == ring[-1] else ring


def _closed(ring):
    ring = [tuple(p) for p in ring]
    return ring + ring[:1] if ring and ring[0] != ring[-1] else ring


def _signed_area(pts):
    return sum(pts[i - 1][0] * pts[i][1] - pts[i][0] * pts[i - 1][1] for i in range(len(pts))) / 2.0


def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _segments_intersect(p1, p2, q1, q2):
    """True if segment p1-p2 touches or crosses q1-q2."""
    d1, d2 = _cross(q1, q2, p1), _cross(q1, q2, p2)
    d3, d4 = _cross(p1, p2, q1), _cross(p1, p2, q2)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)) and 0 not in (d1, d2, d3, d4):
        return True

    def on_segment(a, b, c):   # c collinear with a-b and within its box
        return (min(a[0], b[0]) <= c[0] <= max(a[0], b[0])
                and min(a[1], b[1]) <= c[1] <= max(a[1], b[1]))
    return ((d1 == 0 and on_segment(q1, q2, p1)) or (d2 == 0 and on_segment(q1, q2, p2))
            or (d3 == 0 and on_segment(p1, p2, q1)) or (d4 == 0 and on_segment(p1, p2, q2)))


def _self_intersects(pts):
    """pts: open polygon in metres. True if any two non-adjacent edges meet."""
    n = len(pts)
    edges = [(pts[i], pts[(i + 1) % n]) for i in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            if j == i + 1 or (i == 0 and j == n - 1):
                continue   # adjacent edges share a vertex
            if _segments_intersect(*edges[i], *edges[j]):
                return True
    return False


def _collinear(pts, tol_m2=1e-6):
    """True if every point lies on the line through the first two distinct ones."""
    a = pts[0]
    b = next((p for p in pts[1:] if p != a), a)
    return all(abs(_cross(a, b, p)) < tol_m2 for p in pts)


def _inside(p, pts):
    """Ray casting; pts is an open polygon."""
    inside = False
    for i in range(len(pts)):
        (x1, y1), (x2, y2) = pts[i - 1], pts[i]
        if (y1 > p[1]) != (y2 > p[1]) and p[0] < x1 + (p[1] - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


def _dist_to_edges(p, pts):
    best = math.inf
    for i in range(len(pts)):
        (ax, ay), (bx, by) = pts[i - 1], pts[i]
        dx, dy = bx - ax, by - ay
        seg2 = dx * dx + dy * dy
        t = 0.0 if seg2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / seg2))
        best = min(best, math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy)))
    return best


def _outside_by(p, pts):
    """0 inside the polygon, else metres to its nearest edge."""
    return 0.0 if _inside(p, pts) else _dist_to_edges(p, pts)


# ── Checks ──────────────────────────────────────────────────────────────────

def _unique(pts):
    """Distinct corners, first occurrence order."""
    seen, out = set(), []
    for p in pts:
        if p not in seen:
            seen.add(p)
            out.append(p)
    return out


def validate_course(ring):
    """
    INVALID problems with a course boundary (or transit area) ring as
    received: not number pairs, invalid coordinates, not closed, or fewer than
    3 distinct corners that span an area. Corner ORDER is not a problem --
    simple_ring() repairs crossing edges -- and size is only a warning
    (course_warnings()).
    """
    try:
        ring = [(float(lat), float(lon)) for lat, lon in ring]
    except (TypeError, ValueError):
        return ["points are not (latitude, longitude) number pairs"]
    if len(ring) < 4:
        return [f"has {len(ring)} point(s); a closed polygon needs at least 4 "
                f"(3 corners + the repeated first point)"]
    bad = [i + 1 for i, (lat, lon) in enumerate(ring)
           if not (math.isfinite(lat) and math.isfinite(lon) and -90 <= lat <= 90 and -180 <= lon <= 180)]
    if bad:
        return [f"invalid latitude/longitude at point(s) {bad}"]
    problems = []
    if ring[0] != ring[-1]:
        problems.append("not closed: the first and last points must be the same")
    pts = _unique(_open(ring))
    if len(pts) < 3:
        problems.append(f"only {len(pts)} distinct corner(s); needs at least 3")
    elif _collinear([_xy(p, pts[0]) for p in pts]):
        problems.append("has zero area (all corners on a line)")
    return problems


def course_warnings(ring, min_span_m=MIN_SPAN_M, max_span_m=MAX_SPAN_M):
    """Non-blocking notes on a ring that passed validate_course()."""
    pts = [(float(a), float(b)) for a, b in _open(ring)]
    uniq = _unique(pts)
    warnings = []
    if len(uniq) != len(pts):
        warnings.append("repeats a corner: the duplicate was dropped")
    xy = [_xy(p, uniq[0]) for p in uniq]
    if _self_intersects(xy):
        warnings.append("corners are in an order whose edges cross: reordered into a "
                        "non-crossing polygon through the same corners")
    span = max(math.dist(a, b) for a in xy for b in xy)
    if span < min_span_m:
        warnings.append(f"is only {span:.1f} m across (a real course is >= {min_span_m:g} m: "
                        f"check the coordinates)")
    elif span > max_span_m:
        warnings.append(f"is {span:.0f} m across (a real course is <= {max_span_m:g} m: "
                        f"check the coordinates)")
    return warnings


def _convex_hull(pts):
    """Andrew's monotone chain, counter-clockwise, on (x, y) points."""
    pts = sorted(set(pts))
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and _cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and _cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def simple_ring(ring):
    """
    (closed ring, how) -- the course as a NON-CROSSING polygon, whatever
    order its corners arrived in. A ring that doesn't cross (and repeats no
    corner) is returned exactly as given, so concave courses keep their
    shape: how = None. Otherwise its distinct corners are ordered by angle
    around their centre (how = "reordered"), which passes through every
    corner without crossing; if equal angles still leave a crossing, the
    convex hull is used (how = "convex hull"). Coordinates are never changed.
    """
    pts = [(float(a), float(b)) for a, b in _open(ring)]
    uniq = _unique(pts)
    origin = uniq[0]
    xy = {p: _xy(p, origin) for p in uniq}
    if len(uniq) == len(pts) and not _self_intersects([xy[p] for p in uniq]):
        return _closed(uniq), None
    cx = sum(q[0] for q in xy.values()) / len(uniq)
    cy = sum(q[1] for q in xy.values()) / len(uniq)
    ordered = sorted(uniq, key=lambda p: (math.atan2(xy[p][1] - cy, xy[p][0] - cx),
                                          math.hypot(xy[p][0] - cx, xy[p][1] - cy)))
    if not _self_intersects([xy[p] for p in ordered]):
        return _closed(ordered), "reordered"
    by_xy = {v: k for k, v in xy.items()}
    hull = [by_xy[q] for q in _convex_hull(list(xy.values()))]
    return _closed(hull), "convex hull"


def validate_altitude(max_alt_hae_m, limit_hae_m=MAX_UAV_ALTITUDE_HAE_M,
                      home_hae_m=None, margin_m=0.0):
    """Problems with a geofence ceiling (metres HAE). With home_hae_m, the
    ceiling minus margin_m must also be above home (PX4's ceiling is set
    relative to home and must be > 0)."""
    try:
        alt = float(max_alt_hae_m)
    except (TypeError, ValueError):
        return [f"altitude limit {max_alt_hae_m!r} is not a number"]
    if not math.isfinite(alt):
        return [f"altitude limit {alt} is not a finite number"]
    problems = []
    if alt <= 0:
        problems.append(f"altitude limit {alt:g} m HAE must be above 0")
    if alt > limit_hae_m:
        problems.append(f"altitude limit {alt:g} m HAE is above the maximum {limit_hae_m:g} m HAE")
    if home_hae_m is not None and alt - margin_m <= home_hae_m:
        problems.append(f"altitude limit {alt:g} m HAE (minus {margin_m:g} m margin) is not above "
                        f"home ({home_hae_m:.1f} m HAE)")
    return problems


def inset_polygon(ring, meters):
    """
    The polygon moved `meters` inward on every edge (mitred corners), as a
    closed ring. Raises ValueError if it collapses, flips or self-intersects
    (the inset is larger than the polygon allows).
    """
    pts = _open(ring)
    if meters <= 0:
        return _closed(pts)
    origin = pts[0]
    xy = [_xy(p, origin) for p in pts]
    area = _signed_area(xy)
    if area == 0:
        raise ValueError("cannot inset a zero-area polygon")
    sign = 1.0 if area > 0 else -1.0   # CCW: interior is to the left of each edge
    n = len(xy)

    def offset_line(i):
        (ax, ay), (bx, by) = xy[i], xy[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        length = math.hypot(dx, dy)
        nx, ny = -dy / length * sign, dx / length * sign   # inward normal
        return (ax + nx * meters, ay + ny * meters), (dx, dy)

    lines = [offset_line(i) for i in range(n)]
    out = []
    for i in range(n):
        (p1, d1), (p2, d2) = lines[i - 1], lines[i]
        denom = d1[0] * d2[1] - d1[1] * d2[0]
        if abs(denom) < 1e-9:   # collinear edges: the offset point is just shifted
            out.append(p2)
            continue
        t = ((p2[0] - p1[0]) * d2[1] - (p2[1] - p1[1]) * d2[0]) / denom
        out.append((p1[0] + d1[0] * t, p1[1] + d1[1] * t))

    new_area = _signed_area(out)
    # An inset larger than the polygon allows can turn it inside out into a
    # smaller copy with the SAME orientation (a square mirrors through its
    # centre), so also require every corner to be ~`meters` from the edges.
    if new_area * area <= 0 or abs(new_area) >= abs(area) or abs(new_area) < MIN_FENCE_AREA_M2 \
            or _self_intersects(out) \
            or any(not _inside(q, xy) for q in out) \
            or any(_dist_to_edges(q, xy) < 0.99 * meters for q in out):
        raise ValueError(f"a {meters:g} m inset collapses this polygon (too small for it)")
    return _closed([_latlon(q, origin) for q in out])


def clip_transit(course, transit, fraction):
    """
    The part of the transit area nearest the course, as a closed ring: the
    course edge facing the transit area is found, and the transit area is cut
    by a line parallel to it at `fraction` of the transit's depth from it
    (0.5 = the half next to the course; >= 1 returns the transit unchanged).
    Raises ValueError if the transit area doesn't lie outside a course edge or
    the cut leaves nothing.
    """
    t_open = _open(transit)
    if fraction >= 1.0:
        return _closed(t_open)
    if fraction <= 0.0:
        raise ValueError(f"transit fraction must be > 0 (got {fraction:g})")
    c_open = _open(course)
    origin = c_open[0]
    cxy = [_xy(p, origin) for p in c_open]
    txy = [_xy(p, origin) for p in t_open]
    sign = 1.0 if _signed_area(cxy) > 0 else -1.0   # CCW: interior left of each edge
    tc = (sum(x for x, _ in txy) / len(txy), sum(y for _, y in txy) / len(txy))

    # Course edge facing the transit area: the transit centroid is on its
    # outer side, and it is the nearest such edge to that centroid.
    best = None
    n = len(cxy)
    for i in range(n):
        (ax, ay), (bx, by) = cxy[i], cxy[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        length = math.hypot(dx, dy)
        if length == 0:
            continue
        nx, ny = dy / length * sign, -dx / length * sign   # outward normal
        if (tc[0] - ax) * nx + (tc[1] - ay) * ny <= 0:
            continue
        d = _dist_to_edges(tc, [(ax, ay), (bx, by)])
        if best is None or d < best[0]:
            best = (d, (ax, ay), (nx, ny))
    if best is None:
        raise ValueError("transit area is not outside any course edge")
    _, (ax, ay), (nx, ny) = best

    def depth(q):
        return (q[0] - ax) * nx + (q[1] - ay) * ny

    cut = fraction * max(depth(q) for q in txy)
    if cut <= 0:
        raise ValueError("transit area has no depth beyond the course edge")

    # Sutherland-Hodgman against the half-plane depth <= cut.
    out = []
    for i in range(len(txy)):
        p, q = txy[i - 1], txy[i]
        dp, dq = depth(p) - cut, depth(q) - cut
        if (dp <= 0) != (dq <= 0):
            t = dp / (dp - dq)
            out.append((p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t))
        if dq <= 0:
            out.append(q)
    if len(_unique(out)) < 3 or abs(_signed_area(out)) < MIN_FENCE_AREA_M2:
        raise ValueError(f"cutting the transit area at {fraction:g} of its depth leaves nothing")
    return _closed([_latlon(q, origin) for q in out])


def merge_transit(course, transit, tol_m=FENCE_TOLERANCE_M):
    """
    The union of the course and the transit area as ONE closed ring -- an
    outline that can be uploaded to PX4 and declared to RoboCommand as the same
    fence, with no gap between the two areas. Handles a full shared edge, a
    partly shared edge (transit narrower/wider or shifted along it), corners
    that don't quite match, overlapping areas, and a gap of up to tol_m: transit
    corners within tol_m of the course are first snapped onto it (to its corner
    if one is that close, else onto its edge).

    Raises ValueError if the union is not one simple polygon: the areas are
    further apart than tol_m, touch only at a point, or would enclose a hole.
    """
    EPS = 1e-6   # metres: points this close are the same point
    c_open = _open(course)
    origin = c_open[0]
    c = [_xy(p, origin) for p in c_open]
    t = [_xy(p, origin) for p in _open(transit)]
    if _signed_area(c) < 0:
        c = c[::-1]
    if _signed_area(t) < 0:
        t = t[::-1]

    def closest_on_seg(p, a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        seg2 = dx * dx + dy * dy
        u = 0.0 if seg2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / seg2))
        return (a[0] + u * dx, a[1] + u * dy)

    def dist(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    # 1. Snap transit corners near the course onto it (closes small gaps and
    #    makes "almost shared" edges exactly shared).
    snapped = []
    for p in t:
        corner = min(c, key=lambda q: dist(p, q))
        if dist(p, corner) <= tol_m:
            snapped.append(corner)
            continue
        best = min((closest_on_seg(p, c[i - 1], c[i]) for i in range(len(c))),
                   key=lambda q: dist(p, q))
        snapped.append(best if dist(p, best) <= tol_m else p)
    t = [p for i, p in enumerate(snapped) if dist(p, snapped[i - 1]) > EPS]
    if len(t) < 3 or abs(_signed_area(t)) < MIN_FENCE_AREA_M2:
        raise ValueError("transit area collapses when snapped onto the course")
    if _signed_area(t) < 0:
        t = t[::-1]

    # 2. Split every edge of each polygon where it meets the other polygon.
    def seg_hits(a, b, q1, q2):
        """Parameters along a-b where segment q1-q2 touches it."""
        out = []
        dx, dy = b[0] - a[0], b[1] - a[1]
        ex, ey = q2[0] - q1[0], q2[1] - q1[1]
        seg2 = dx * dx + dy * dy
        if seg2 == 0:
            return out
        denom = dx * ey - dy * ex
        if abs(denom) > 1e-12:
            s = ((q1[0] - a[0]) * ey - (q1[1] - a[1]) * ex) / denom
            u = ((q1[0] - a[0]) * dy - (q1[1] - a[1]) * dx) / denom
            if -1e-9 <= s <= 1 + 1e-9 and -1e-9 <= u <= 1 + 1e-9:
                out.append(min(1.0, max(0.0, s)))
        # Endpoints of q lying on a-b (covers collinear overlap too).
        for q in (q1, q2):
            s = ((q[0] - a[0]) * dx + (q[1] - a[1]) * dy) / seg2
            if 0 <= s <= 1 and dist(q, (a[0] + s * dx, a[1] + s * dy)) <= EPS:
                out.append(s)
        return out

    def split(poly, other):
        edges = []
        for i in range(len(poly)):
            a, b = poly[i], poly[(i + 1) % len(poly)]
            ts = {0.0, 1.0}
            for j in range(len(other)):
                ts.update(seg_hits(a, b, other[j], other[(j + 1) % len(other)]))
            pts = [(a[0] + s * (b[0] - a[0]), a[1] + s * (b[1] - a[1])) for s in sorted(ts)]
            pts[0], pts[-1] = a, b
            edges += [(pts[k], pts[k + 1]) for k in range(len(pts) - 1)
                      if dist(pts[k], pts[k + 1]) > EPS]
        return edges

    def on_boundary(p, poly):
        return _dist_to_edges(p, poly) <= EPS

    def key(p):
        return (round(p[0], 5), round(p[1], 5))

    # 3. Keep the pieces on the union's outline: outside the other polygon, or
    #    on a shared edge running the same way (kept once). A shared edge
    #    running opposite ways is the seam between the two areas -- dropped.
    c_edges, t_edges = split(c, t), split(t, c)
    t_keys = {(key(a), key(b)) for a, b in t_edges}
    kept = []
    for edges, other, is_course in ((c_edges, t, True), (t_edges, c, False)):
        for a, b in edges:
            mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
            if on_boundary(mid, other):
                same_way = (key(a), key(b)) in t_keys if is_course else False
                if is_course and same_way:
                    kept.append((a, b))      # outer edge both share: keep one copy
                continue                     # seam, or the transit's copy
            if not _inside(mid, other):
                kept.append((a, b))

    # 4. Chain the kept pieces into one ring.
    nxt = {}
    for a, b in kept:
        if key(a) in nxt:
            raise ValueError("course and transit area touch at a single point only -- "
                             "not one closed outline")
        nxt[key(a)] = (a, b)
    if not kept:
        raise ValueError("course and transit area have no outline")
    start = key(kept[0][0])
    ring, k = [], start
    for _ in range(len(kept)):
        a, b = nxt[k]
        ring.append(a)
        k = key(b)
        if k == start:
            break
    if k != start or len(ring) != len(kept):
        raise ValueError(f"course and transit area are more than {tol_m:g} m apart or "
                         "enclose a hole -- not one closed outline")

    # 5. Drop corners that sit on a straight line (from the edge splits).
    clean = []
    for i, p in enumerate(ring):
        prev, nx = ring[i - 1], ring[(i + 1) % len(ring)]
        if abs(_cross(prev, p, nx)) > 1e-6 * max(1.0, dist(prev, nx)):
            clean.append(p)
    if len(clean) < 3 or _self_intersects(clean):
        raise ValueError("course and transit area give a self-intersecting outline")
    return _closed([_latlon(q, origin) for q in clean])


def fence_problems(fence, course, transit=None, tolerance_m=FENCE_TOLERANCE_M,
                   step_m=EDGE_STEP_M):
    """
    Problems with a UAV geofence against the course (and optional transit
    area), as RoboCommand checks a RunDeclaration: closed, >= 4 points, and
    every point along its edges within the course or transit area to within
    tolerance_m.
    """
    fence = [tuple(p) for p in fence]
    if len(fence) < 4:
        return [f"geofence has {len(fence)} point(s), needs at least 4"]
    if fence[0] != fence[-1]:
        return ["geofence isn't closed: its first and last points must be the same"]
    origin = _open(course)[0]
    areas = [[_xy(p, origin) for p in _open(r)] for r in (course, transit) if r]
    pts = [_xy(p, origin) for p in fence]
    worst, where = 0.0, None
    for i, (a, b) in enumerate(zip(pts, pts[1:])):
        steps = max(1, math.ceil(math.dist(a, b) / step_m))
        for k in range(steps + 1):
            q = (a[0] + (b[0] - a[0]) * k / steps, a[1] + (b[1] - a[1]) * k / steps)
            out = min(_outside_by(q, area) for area in areas)
            if out > worst:
                worst = out
                where = (f"point {i + 1}" if k == 0 else f"point {i + 2}" if k == steps
                         else f"edge {i + 1}-{i + 2}")
    if worst > tolerance_m:
        return [f"geofence leaves the course and transit area by {worst:.1f} m at {where}"]
    return []


def min_clearance_m(fence, course):
    """Smallest distance (m) from any fence vertex to the course edges --
    e.g. ~1.0 after a 1 m inset. Negative if a vertex is outside the course."""
    origin = _open(course)[0]
    area = [_xy(p, origin) for p in _open(course)]
    best = math.inf
    for p in _open(fence):
        q = _xy(p, origin)
        d = _dist_to_edges(q, area)
        best = min(best, d if _inside(q, area) else -d)
    return best
