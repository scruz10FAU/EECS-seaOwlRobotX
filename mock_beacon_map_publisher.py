#!/usr/bin/env python3
"""
mock_beacon_map_publisher.py — publishes synthetic /seabird/beacon_map messages
for testing consumers of beacon_overlay.py's fused beacon map (e.g.
sweep_rrt_overfly_rework.py) without a camera, model, PX4 or GPS fix.

Simulates a drone flying over beacons at known positions. Each frame, every
beacon within --detect-range may produce a noisy fix, with the method chosen by
horizontal range the way beacon_pose.locate_beacons does it (pnp up close, box
at mid range, ray far out). Fixes are fused by the real beacon_pose.BeaconMap
and serialised exactly as beacon_overlay.py's _publish does, so the JSON matches
field for field:

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
import sys
import time

import numpy as np

from beacon_pose import BeaconFix, BeaconMap, lla_to_ned, ned_to_lla

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
# Horizontal 1-sigma per axis = base + per_m * slant range (m). Kept well inside
# BeaconMap's 3 m association gate, or one beacon splits into several entries.
FIX_NOISE = {"pnp": (0.1, 0.01), "box": (0.25, 0.02), "ray": (0.3, 0.03)}
FIX_NOISE_Z_M = 0.05
OUTLIER_OFFSET_M = 2.5            # inside BeaconMap's 3 m gate, so it's counted, not spawned


def _pair(text: str):
    try:
        a, b = (float(v) for v in text.split(","))
    except ValueError:
        raise argparse.ArgumentTypeError(f"expected two comma-separated numbers, got {text!r}")
    return a, b


# ─── Simulated drone path ─────────────────────────────────────────────

def _polyline_at(points, s):
    """Point at arc length s along a closed polyline (wraps)."""
    segs = list(zip(points, points[1:] + points[:1]))
    lengths = [float(np.linalg.norm(b - a)) for a, b in segs]
    total = sum(lengths)
    if total == 0:
        return points[0].copy()
    s %= total
    for (a, b), length in zip(segs, lengths):
        if s <= length:
            return a + (b - a) * (s / length if length else 0.0)
        s -= length
    return points[0].copy()


def make_path(args, beacons_ned):
    """-> function(elapsed_s) giving the drone's horizontal NED position."""
    if args.path == "hover":
        hover = np.array(args.hover_ned if args.hover_ned else beacons_ned[0][:2], float)
        return lambda t: hover
    centroid = np.mean([b[:2] for b in beacons_ned], axis=0)
    if args.path == "orbit":
        omega = args.speed / args.orbit_radius
        return lambda t: centroid + args.orbit_radius * np.array([math.cos(omega * t), math.sin(omega * t)])
    # sweep: back and forth over every beacon, ordered along the longest spread axis
    pts = [np.asarray(b[:2], float) for b in beacons_ned]
    axis = np.argmax(np.ptp(np.array(pts), axis=0)) if len(pts) > 1 else 0
    pts.sort(key=lambda p: p[axis])
    loop = pts + pts[-2:0:-1]
    return lambda t: _polyline_at(loop, args.speed * t)


# ─── Synthetic fixes ──────────────────────────────────────────────────

def make_fix(rng, truth, drone, outlier_prob):
    d = truth - drone
    horiz = math.hypot(d[0], d[1])
    slant = float(np.linalg.norm(d))
    method = "pnp" if horiz <= PNP_MAX_HORIZ_M else "box" if horiz <= BOX_MAX_HORIZ_M else "ray"
    base, per_m = FIX_NOISE[method]
    s = base + per_m * slant
    pos = truth + np.array([rng.normal(0, s), rng.normal(0, s), rng.normal(0, FIX_NOISE_Z_M)])
    if rng.random() < outlier_prob:
        a = rng.uniform(0, 2 * math.pi)
        pos[:2] += OUTLIER_OFFSET_M * np.array([math.cos(a), math.sin(a)])
    cov = np.diag([s * s, s * s, FIX_NOISE_Z_M ** 2])
    if method == "pnp":
        members, box_members, rms = list(range(int(rng.integers(1, 3)))), [0], float(rng.uniform(0.4, 2.0))
    elif method == "box":
        members, box_members, rms = [], [0], None
    else:
        members, box_members, rms = [0], [], None
    return BeaconFix(pos, cov, method, members, slant, rms_px=rms, box_members=box_members)


# ─── Serialisation (same as beacon_overlay.BeaconOverlay._publish) ────

def build_message(bmap, t, source, frame, drone_ned, water_z, fixes, seen, min_fixes, lat_lon):
    def ll(pos):
        v = lat_lon(pos)
        return (round(v[0], 8), round(v[1], 8)) if v else (None, None)

    def sig(cov):
        return round(math.sqrt(max(float(np.trace(cov[:2, :2])), 0.0)), 3)

    def ned(pos):
        return [round(float(v), 3) for v in pos]

    beacons = []
    for b in bmap.beacons:
        if b["n"] < min_fixes:
            continue
        lat, lon = ll(b["pos"])
        beacons.append(dict(id=b["id"], ned=ned(b["pos"]), lat=lat, lon=lon,
                            sigma_m=sig(bmap.cov_at(b, t)), n=b["n"],
                            outliers=b["outliers"], age_s=round(t - b["last_t"], 3),
                            seen=b["id"] in seen))
    fx = []
    for f in fixes:
        lat, lon = ll(f.pos_ned)
        fx.append(dict(method=f.method, ned=ned(f.pos_ned), lat=lat, lon=lon,
                       sigma_m=sig(f.cov), range_m=round(f.range_m, 3),
                       horiz_m=round(float(np.linalg.norm(f.pos_ned[:2] - drone_ned[:2])), 3),
                       rms_px=None if f.rms_px is None else round(f.rms_px, 3),
                       n_diamonds=len(f.members), n_boxes=len(f.box_members)))
    lat, lon = ll(drone_ned)
    return json.dumps(dict(
        stamp=t, source=source, frame=frame,
        drone=dict(ned=ned(drone_ned), lat=lat, lon=lon), water_z_ned=water_z,
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
    rng = np.random.default_rng(args.seed)
    ref = (args.ref_lat, args.ref_lon, args.ref_alt)
    water_z = args.water_z
    if args.beacon_ned:
        truths = [np.array([n, e, water_z]) for n, e in args.beacon_ned]
    else:
        lls = args.beacon or DEFAULT_BEACONS_LL
        truths = [np.array([*lla_to_ned(lat, lon, ref[2], *ref)[:2], water_z]) for lat, lon in lls]
    path = make_path(args, truths)
    alt_z = water_z - args.alt

    def lat_lon(pos):
        return None if args.no_gps else ned_to_lla(pos, *ref)[:2]

    bmap = BeaconMap(cov_floor_m=args.map_floor, drift_mps=args.drift)
    dt = 1.0 / args.rate if args.rate > 0 else 0.2
    t0 = time.time()
    wall0 = time.monotonic()

    if args.prime:   # min_fixes overhead pnp fixes per beacon, so the map is populated from frame 0
        for _ in range(args.min_fixes):
            for truth in truths:
                bmap.update([make_fix(rng, truth, truth + [0, 0, -args.alt], 0.0)], t0)

    log(f"{len(truths)} beacons (NED): " + ", ".join(f"({p[0]:.1f}, {p[1]:.1f})" for p in truths))
    frame = 0
    while not args.count or frame < args.count:
        t = t0 + frame * dt
        if args.beacon_drift > 0:
            for truth in truths:
                truth[:2] += rng.normal(0, args.beacon_drift * math.sqrt(dt), 2)
        drone = np.array([*path(frame * dt), alt_z])

        fixes = []
        for truth in truths:
            if (math.hypot(*(truth[:2] - drone[:2])) <= args.detect_range
                    and rng.random() < args.detect_prob):
                fixes.append(make_fix(rng, truth, drone, args.outlier_prob))
        seen = {b["id"] for b in bmap.update(fixes, t)}
        publish(build_message(bmap, t, args.source, frame, drone, water_z, fixes, seen,
                              args.min_fixes, lat_lon))

        if args.rate > 0 and frame % max(1, int(round(args.rate))) == 0:
            mapped = [b for b in bmap.beacons if b["n"] >= args.min_fixes]
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
    g.add_argument("--ref-alt", type=float, default=0.0, help="PX4 local origin altitude AMSL (m)")
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
                   help="Chance a fix is pushed 2.5 m off, to exercise outlier counting (default: 0.02)")
    g.add_argument("--min-fixes", type=int, default=3,
                   help="List a beacon only once it has this many fixes (beacon_overlay default: 3)")
    g.add_argument("--map-floor", type=float, default=0.5, help="BeaconMap cov_floor_m (beacon_overlay default: 0.5)")
    g.add_argument("--drift", type=float, default=0.15, help="BeaconMap drift_mps (beacon_overlay default: 0.15)")
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
