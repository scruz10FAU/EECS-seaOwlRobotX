#!/usr/bin/env python3
"""
mock_beacon_map_publisher.py — publishes synthetic /seabird/beacon_map messages
for testing consumers of beacon_overlay.py's fused beacon map (e.g.
sweep_rrt_overfly_rework.py) without a camera, model, PX4 or GPS fix.

Standard library only (plus rclpy to publish; --dry-run needs nothing).

Simulates a drone flying over beacons at known positions. Each frame, every
beacon within --detect-range may produce a noisy fix, with the method chosen by
horizontal range like beacon_pose.locate_beacons (pnp up close, box at mid
range, ray far out). Each beacon's map entry is a running estimate of its fixes
whose uncertainty grows while it's out of view. Messages use the same JSON
schema beacon_overlay.py publishes:

  {"stamp", "source": "live"|"replay", "frame", "drone": {"ned", "lat", "lon"},
   "water_z_ned",
   "beacons": [{"id", "ned", "lat", "lon", "sigma_m", "n", "outliers", "age_s",
                "seen"}],                    # the map, entries with >= --min-fixes
   "fixes":   [{"method", "ned", "lat", "lon", "sigma_m", "range_m", "horiz_m",
                "rms_px", "n_diamonds", "n_boxes"}]}   # this frame

ned is PX4 local NED (m) about --ref-lat/--ref-lon/--ref-alt; lat/lon are null
with --no-gps (as when PX4 has no global reference).

Usage:
    python3 mock_beacon_map_publisher.py                        # sweep over the 5 default beacons at 5 Hz
    python3 mock_beacon_map_publisher.py --prime                # beacons already mapped in the first message
    python3 mock_beacon_map_publisher.py --path hover --hover-ned 0,0
    python3 mock_beacon_map_publisher.py --path orbit --orbit-radius 30
    python3 mock_beacon_map_publisher.py --beacon 26.3712,-80.1034 --beacon 26.3715,-80.1031
    python3 mock_beacon_map_publisher.py --beacon-ned 10,5 --beacon-ned -8,12
    python3 mock_beacon_map_publisher.py --no-gps               # lat/lon null
    python3 mock_beacon_map_publisher.py --dry-run --count 20   # print the JSON, no ROS needed
"""

import argparse
import json
import math
import random
import sys
import time

DEFAULT_TOPIC = "/seabird/beacon_map"

# Same beacon positions as mock_beacon_publisher.py's _SAMPLE_FIXTURES
DEFAULT_BEACONS_LL = [
    (26.371200, -80.103400),
    (26.371350, -80.103250),
    (26.371500, -80.103100),
    (26.371650, -80.102950),
    (26.371800, -80.102800),
]

# Fix method by horizontal drone-to-beacon distance, mirroring locate_beacons'
# priority (pnp needs a close, well-resolved diamond; ray is the far fallback).
PNP_MAX_HORIZ_M = 6.0
BOX_MAX_HORIZ_M = 15.0
# Horizontal 1-sigma per axis = base + per_m * slant range (m)
FIX_NOISE = {"pnp": (0.1, 0.01), "box": (0.25, 0.02), "ray": (0.3, 0.03)}
FIX_NOISE_Z_M = 0.05
OUTLIER_OFFSET_M = 2.5            # injected bad fixes: counted as outliers, never fused

EARTH_RADIUS_M = 6371000.0        # PX4 CONSTANTS_RADIUS_OF_EARTH


def _pair(text: str):
    try:
        a, b = (float(v) for v in text.split(","))
    except ValueError:
        raise argparse.ArgumentTypeError(f"expected two comma-separated numbers, got {text!r}")
    return a, b


# ─── Geodesy (PX4 MapProjection, azimuthal equidistant — as beacon_pose.py) ──

def ned_to_ll(ned, ref_lat, ref_lon):
    xr, yr = ned[0] / EARTH_RADIUS_M, ned[1] / EARTH_RADIUS_M
    lat0, lon0 = math.radians(ref_lat), math.radians(ref_lon)
    c = math.hypot(xr, yr)
    if c == 0:
        return ref_lat, ref_lon
    sc, cc = math.sin(c), math.cos(c)
    lat = math.asin(cc * math.sin(lat0) + xr * sc * math.cos(lat0) / c)
    lon = lon0 + math.atan2(yr * sc, c * math.cos(lat0) * cc - xr * math.sin(lat0) * sc)
    return math.degrees(lat), math.degrees(lon)


def ll_to_ne(lat, lon, ref_lat, ref_lon):
    lat0, lon0 = math.radians(ref_lat), math.radians(ref_lon)
    la, lo = math.radians(lat), math.radians(lon)
    cos_d = math.cos(lo - lon0)
    c = math.acos(max(-1.0, min(1.0, math.sin(lat0) * math.sin(la)
                                + math.cos(lat0) * math.cos(la) * cos_d)))
    k = c / math.sin(c) if abs(c) > 1e-12 else 1.0
    n = k * (math.cos(lat0) * math.sin(la) - math.sin(lat0) * math.cos(la) * cos_d) * EARTH_RADIUS_M
    e = k * math.cos(la) * math.sin(lo - lon0) * EARTH_RADIUS_M
    return n, e


# ─── Simulated drone path ─────────────────────────────────────────────

def _polyline_at(points, s):
    """(n, e) at arc length s along a closed polyline (wraps)."""
    segs = list(zip(points, points[1:] + points[:1]))
    lengths = [math.dist(a, b) for a, b in segs]
    total = sum(lengths)
    if total == 0:
        return points[0]
    s %= total
    for (a, b), length in zip(segs, lengths):
        if s <= length:
            f = s / length if length else 0.0
            return a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f
        s -= length
    return points[0]


def make_path(args, beacons_ne):
    """-> function(elapsed_s) giving the drone's (north, east)."""
    if args.path == "hover":
        hover = args.hover_ned or beacons_ne[0]
        return lambda t: hover
    cn = sum(p[0] for p in beacons_ne) / len(beacons_ne)
    ce = sum(p[1] for p in beacons_ne) / len(beacons_ne)
    if args.path == "orbit":
        omega = args.speed / args.orbit_radius
        r = args.orbit_radius
        return lambda t: (cn + r * math.cos(omega * t), ce + r * math.sin(omega * t))
    # sweep: back and forth over every beacon, ordered along their longest spread
    spread_n = max(p[0] for p in beacons_ne) - min(p[0] for p in beacons_ne)
    spread_e = max(p[1] for p in beacons_ne) - min(p[1] for p in beacons_ne)
    pts = sorted(beacons_ne, key=lambda p: p[0] if spread_n >= spread_e else p[1])
    loop = pts + pts[-2:0:-1]
    return lambda t: _polyline_at(loop, args.speed * t)


# ─── Synthetic fixes and map ──────────────────────────────────────────

def make_fix(rng, truth, drone, outlier_prob):
    """-> (fix dict with internal fields, is_outlier)"""
    dn, de, dd = (truth[i] - drone[i] for i in range(3))
    horiz = math.hypot(dn, de)
    slant = math.sqrt(dn * dn + de * de + dd * dd)
    method = "pnp" if horiz <= PNP_MAX_HORIZ_M else "box" if horiz <= BOX_MAX_HORIZ_M else "ray"
    base, per_m = FIX_NOISE[method]
    s = base + per_m * slant
    pos = [truth[0] + rng.gauss(0, s), truth[1] + rng.gauss(0, s), truth[2] + rng.gauss(0, FIX_NOISE_Z_M)]
    outlier = rng.random() < outlier_prob
    if outlier:
        a = rng.uniform(0, 2 * math.pi)
        pos[0] += OUTLIER_OFFSET_M * math.cos(a)
        pos[1] += OUTLIER_OFFSET_M * math.sin(a)
    n_diamonds, n_boxes, rms = {"pnp": (rng.randint(1, 2), 1, round(rng.uniform(0.4, 2.0), 3)),
                                "box": (0, 1, None),
                                "ray": (1, 0, None)}[method]
    return dict(method=method, pos=pos, var=s * s, range_m=slant, horiz_m=horiz, rms_px=rms,
                n_diamonds=n_diamonds, n_boxes=n_boxes), outlier


class SimpleBeaconMap:
    """
    One entry per simulated beacon, keyed by its index (the mock knows which
    beacon each fix came from, so no association step). The horizontal estimate
    is a 1-D Kalman filter per axis whose variance grows by drift^2 per second
    between fixes — the same idea as beacon_pose.BeaconMap, minus association
    and merging.
    """

    def __init__(self, drift_mps):
        self.q = drift_mps ** 2
        self.entries = {}

    def var_at(self, b, t):
        return b["var"] + self.q * max(0.0, t - b["last_t"])

    def update(self, i, fix, outlier, t):
        b = self.entries.get(i)
        if b is None:
            if outlier:
                return
            self.entries[i] = dict(id=i, pos=list(fix["pos"]), var=fix["var"], n=1, outliers=0, last_t=t)
            return
        if outlier:
            b["outliers"] += 1
            return
        p = self.var_at(b, t)
        k = p / (p + fix["var"])
        b["pos"] = [b["pos"][j] + k * (fix["pos"][j] - b["pos"][j]) for j in range(3)]
        b["var"] = (1 - k) * p
        b["n"] += 1
        b["last_t"] = t


# ─── Message (same shape as beacon_overlay.BeaconOverlay._publish) ────

def build_message(bmap, t, source, frame, drone, water_z, fixes, seen, min_fixes, lat_lon):
    def ll(pos):
        v = lat_lon(pos)
        return (round(v[0], 8), round(v[1], 8)) if v else (None, None)

    def ned(pos):
        return [round(float(v), 3) for v in pos]

    beacons = []
    for b in sorted(bmap.entries.values(), key=lambda b: b["id"]):
        if b["n"] < min_fixes:
            continue
        lat, lon = ll(b["pos"])
        beacons.append(dict(id=b["id"], ned=ned(b["pos"]), lat=lat, lon=lon,
                            sigma_m=round(math.sqrt(2 * bmap.var_at(b, t)), 3), n=b["n"],
                            outliers=b["outliers"], age_s=round(t - b["last_t"], 3),
                            seen=b["id"] in seen))
    fx = []
    for f in fixes:
        lat, lon = ll(f["pos"])
        fx.append(dict(method=f["method"], ned=ned(f["pos"]), lat=lat, lon=lon,
                       sigma_m=round(math.sqrt(2 * f["var"]), 3), range_m=round(f["range_m"], 3),
                       horiz_m=round(f["horiz_m"], 3), rms_px=f["rms_px"],
                       n_diamonds=f["n_diamonds"], n_boxes=f["n_boxes"]))
    lat, lon = ll(drone)
    return json.dumps(dict(
        stamp=t, source=source, frame=frame,
        drone=dict(ned=ned(drone), lat=lat, lon=lon), water_z_ned=water_z,
        beacons=beacons, fixes=fx))


# ─── Output ───────────────────────────────────────────────────────────

def ros_output(topic):
    import rclpy
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node("mock_beacon_map_publisher")
    pub = node.create_publisher(String, topic, 10)
    node.get_logger().info(f"Publishing synthetic beacon map on {topic}")

    def publish(text):
        msg = String()
        msg.data = text
        pub.publish(msg)

    def close():
        node.destroy_node()
        rclpy.shutdown()
    return publish, node.get_logger().info, close


def stdout_output():
    def log(text):
        print(text, file=sys.stderr, flush=True)
    return (lambda text: print(text, flush=True)), log, (lambda: None)


# ─── Main loop ────────────────────────────────────────────────────────

def run(args, publish, log):
    rng = random.Random(args.seed)
    water_z = args.water_z
    if args.beacon_ned:
        beacons_ne = list(args.beacon_ned)
    else:
        beacons_ne = [ll_to_ne(lat, lon, args.ref_lat, args.ref_lon) for lat, lon in args.beacon or DEFAULT_BEACONS_LL]
    truths = [[n, e, water_z] for n, e in beacons_ne]
    path = make_path(args, beacons_ne)
    drone_z = water_z - args.alt

    def lat_lon(pos):
        return None if args.no_gps else ned_to_ll(pos, args.ref_lat, args.ref_lon)

    bmap = SimpleBeaconMap(args.drift)
    dt = 1.0 / args.rate if args.rate > 0 else 0.2
    t0 = time.time()
    wall0 = time.monotonic()

    if args.prime:   # min_fixes overhead fixes per beacon, so the map is populated from frame 0
        for _ in range(args.min_fixes):
            for i, truth in enumerate(truths):
                fix, _ = make_fix(rng, truth, [truth[0], truth[1], drone_z], 0.0)
                bmap.update(i, fix, False, t0)

    log(f"{len(truths)} beacons (NED): " + ", ".join(f"({n:.1f}, {e:.1f})" for n, e in beacons_ne))
    frame = 0
    while not args.count or frame < args.count:
        t = t0 + frame * dt
        if args.beacon_drift > 0:
            for truth in truths:
                truth[0] += rng.gauss(0, args.beacon_drift * math.sqrt(dt))
                truth[1] += rng.gauss(0, args.beacon_drift * math.sqrt(dt))
        drone = [*path(frame * dt), drone_z]

        fixes, seen = [], set()
        for i, truth in enumerate(truths):
            if (math.hypot(truth[0] - drone[0], truth[1] - drone[1]) <= args.detect_range
                    and rng.random() < args.detect_prob):
                fix, outlier = make_fix(rng, truth, drone, args.outlier_prob)
                fixes.append(fix)
                bmap.update(i, fix, outlier, t)
                seen.add(i)
        publish(build_message(bmap, t, args.source, frame, drone, water_z, fixes, seen,
                              args.min_fixes, lat_lon))

        if args.rate > 0 and frame % max(1, int(round(args.rate))) == 0:
            mapped = [b for b in bmap.entries.values() if b["n"] >= args.min_fixes]
            log(f"frame {frame}: drone NED ({drone[0]:.1f}, {drone[1]:.1f}), {len(fixes)} fixes, "
                f"{len(mapped)} mapped: " + ", ".join(
                    f"B{b['id']}({b['pos'][0]:.1f}, {b['pos'][1]:.1f}) n={b['n']}" for b in mapped))

        frame += 1
        if args.rate > 0:
            time.sleep(max(0.0, wall0 + frame * dt - time.monotonic()))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default=DEFAULT_TOPIC, help=f"Topic to publish on (default: {DEFAULT_TOPIC})")
    ap.add_argument("--rate", type=float, default=5.0,
                    help="Messages per second, like beacon_overlay's inference rate (default: 5; "
                         "<= 0 = as fast as possible)")
    ap.add_argument("--count", type=int, default=0, help="Stop after this many messages (default: 0 = unlimited)")
    ap.add_argument("--dry-run", action="store_true", help="Print the JSON to stdout instead of publishing (no ROS)")
    ap.add_argument("--seed", type=int, default=None, help="Random seed for a repeatable run")

    g = ap.add_argument_group("beacons")
    g.add_argument("--beacon", type=_pair, action="append", metavar="LAT,LON",
                   help="True beacon position (repeatable; default: the 5 mock_beacon_publisher.py fixtures)")
    g.add_argument("--beacon-ned", type=_pair, action="append", metavar="N,E",
                   help="True beacon position in local NED metres (repeatable; overrides --beacon)")
    g.add_argument("--beacon-drift", type=float, default=0.0,
                   help="Actual beacon random walk, m/sqrt(s) (default: 0 = moored)")
    g.add_argument("--ref-lat", type=float, default=26.371000, help="PX4 local origin latitude")
    g.add_argument("--ref-lon", type=float, default=-80.103600, help="PX4 local origin longitude")
    g.add_argument("--water-z", type=float, default=0.0, help="Water surface in local NED z (default: 0)")
    g.add_argument("--no-gps", action="store_true", help="Publish lat/lon as null (no global reference)")

    g = ap.add_argument_group("drone")
    g.add_argument("--path", choices=["sweep", "orbit", "hover"], default="sweep",
                   help="sweep: back and forth over every beacon (default); orbit: circle their "
                        "centroid; hover: stay at --hover-ned")
    g.add_argument("--hover-ned", type=_pair, metavar="N,E", help="Hover position (default: over the first beacon)")
    g.add_argument("--orbit-radius", type=float, default=25.0, help="Orbit radius in m (default: 25)")
    g.add_argument("--speed", type=float, default=3.0, help="Ground speed in m/s (default: 3)")
    g.add_argument("--alt", type=float, default=12.0, help="Height above the water in m (default: 12)")

    g = ap.add_argument_group("detection and map")
    g.add_argument("--detect-range", type=float, default=25.0,
                   help="Max horizontal distance at which a beacon yields fixes (default: 25 m)")
    g.add_argument("--detect-prob", type=float, default=0.85,
                   help="Chance an in-range beacon yields a fix in a given frame (default: 0.85)")
    g.add_argument("--outlier-prob", type=float, default=0.02,
                   help="Chance a fix is pushed 2.5 m off and counted as an outlier (default: 0.02)")
    g.add_argument("--min-fixes", type=int, default=3,
                   help="List a beacon only once it has this many fixes (beacon_overlay default: 3)")
    g.add_argument("--drift", type=float, default=0.15,
                   help="Map uncertainty growth while a beacon is unseen, m/sqrt(s) (beacon_overlay default: 0.15)")
    g.add_argument("--prime", action="store_true",
                   help="Start with every beacon already mapped (min-fixes fixes each)")
    g.add_argument("--source", choices=["live", "replay"], default="live", help='"source" field (default: live)')
    args = ap.parse_args()

    publish, log, close = stdout_output() if args.dry_run else ros_output(args.topic)
    try:
        run(args, publish, log)
    except KeyboardInterrupt:
        pass
    finally:
        close()


if __name__ == "__main__":
    main()
