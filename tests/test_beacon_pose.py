"""
Synthetic checks for beacon_pose.py.

Renders the beacon's diamonds through the real tracking_front fisheye
calibration and extrinsics (values copied from the flight drone on
2026-10-11), adds keypoint noise, and checks the recovered base position.

Run:  python3 tests/test_beacon_pose.py      (prints an error-vs-range table)
      python3 -m pytest tests/test_beacon_pose.py
"""

import math
import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import beacon_pose as bp  # noqa: E402

# /data/modalai/opencv_tracking_front_intrinsics.yml
CAM = bp.CameraModel(
    K=np.array([[446.72757074770652, 0.0, 669.29601423812687],
                [0.0, 446.75209915602363, 379.33821474579594],
                [0.0, 0.0, 1.0]]),
    D=np.array([0.065019883402398324, 0.021202530907395534,
                -0.010548422991715257, 0.0011109285291500428]),
    model="fisheye", width=1280, height=800)

# /etc/modalai/extrinsics.conf (subset)
EXTRINSICS_CONF = """/* comment */
{ "name": "test", "extrinsics": [
  {"parent": "imu_apps", "child": "tracking_front",
   "T_child_wrt_parent": [0.039, 0.0, 0.0006], "RPY_parent_to_child": [0, 90, 90]},
  {"parent": "body", "child": "imu_apps",
   "T_child_wrt_parent": [0.0303, -0.0057, -0.0116], "RPY_parent_to_child": [0, 0, 0]}
]}"""

MODEL = bp.BeaconModel()

# Beacon body for synthetic bboxes, z-up metres: skirt, CAD white section, and
# the lit top (assumed = white-section top width, as in BeaconModel).
_Z_WHITE0, _Z_WHITE1 = 0.0782, 0.3697
_BODY_RINGS = [(0.2795, 0.0), (0.2795, _Z_WHITE0), (0.216, _Z_WHITE0), (0.127, _Z_WHITE1),
               (0.127, MODEL.height_m)]
BODY_ZUP = np.array([[sx * h, sy * h, z] for h, z in _BODY_RINGS
                     for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1))])


def _extrinsics():
    with tempfile.NamedTemporaryFile("w", suffix=".conf", delete=False) as f:
        f.write(EXTRINSICS_CONF)
    try:
        return bp.load_modalai_extrinsics(f.name, "tracking_front")
    finally:
        os.unlink(f.name)


R_BODY_CAM, T_BODY_CAM = _extrinsics()


def _quat_from_euler(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy])


def render(rng, range_m, alt_m, noise_px, att_deg=3.0, max_view_deg=75.0, with_box=False,
           box_noise_px=1.0):
    """Random scene -> (diamonds, pose, true_base[, box]). Beacon base on water (NED z = 0).

    box is a BeaconBoxObs around the projected body (None if off-image), with
    each edge jittered by box_noise_px.
    """
    yaw = rng.uniform(-math.pi, math.pi)
    q = _quat_from_euler(*np.radians(rng.normal(0, att_deg, 2)), yaw)
    drone = np.array([0.0, 0.0, -alt_m])
    pose = bp.camera_pose_from_drone(drone, q, R_BODY_CAM, T_BODY_CAM)
    # Beacon ahead of the camera, horizontally within +/-30 deg of boresight.
    fwd = pose.R_ned_cam[:, 2].copy()
    fwd[2] = 0
    fwd /= np.linalg.norm(fwd)
    side = np.cross([0, 0, 1.0], fwd)
    a = rng.uniform(-0.5, 0.5)
    horiz = math.sqrt(max(range_m ** 2 - alt_m ** 2, 1.0))
    base = drone + horiz * (math.cos(a) * fwd + math.sin(a) * side)
    base[2] = 0.0
    byaw = rng.uniform(0, math.pi / 2)
    diamonds = []
    for k in range(4):
        P = base + MODEL.faces[k] @ bp.rot_z(byaw).T
        n = np.cross(P[2] - P[0], P[3] - P[1])
        centroid = P.mean(axis=0)
        if n[:2] @ (centroid - base)[:2] < 0:
            n = -n
        view = pose.c_ned - centroid
        cosang = n @ view / (np.linalg.norm(n) * np.linalg.norm(view))
        if cosang < math.cos(math.radians(max_view_deg)):
            continue
        Pc = (P - pose.c_ned) @ pose.R_ned_cam
        if np.any(Pc[:, 2] <= 0.1):
            continue
        px = CAM.project(Pc) + rng.normal(0, noise_px, (4, 2))
        if np.any(px < 0) or np.any(px[:, 0] >= CAM.width) or np.any(px[:, 1] >= CAM.height):
            continue
        diamonds.append(bp.DiamondObs(px, np.full(4, 0.9)))
    if not with_box:
        return diamonds, pose, base
    box = None
    body = base + (BODY_ZUP * np.array([1.0, -1.0, -1.0])) @ bp.rot_z(byaw).T
    # Edge midpoints too: fisheye bends straight edges.
    edges = np.concatenate([body, (body[:-4] + body[4:]) / 2])
    Pc = (edges - pose.c_ned) @ pose.R_ned_cam
    if np.all(Pc[:, 2] > 0.1):
        px = CAM.project(Pc)
        b = np.r_[px.min(0), px.max(0)] + rng.normal(0, box_noise_px, 4)
        if b[0] >= 0 and b[1] >= 0 and b[2] < CAM.width and b[3] < CAM.height:
            box = bp.BeaconBoxObs(tuple(b))
    return diamonds, pose, base, box


# ─── Tests ────────────────────────────────────────────────────────────

def test_extrinsics_forward_level():
    # tracking_front: camera z = body forward, x = body right, y = body down.
    assert np.allclose(R_BODY_CAM, [[0, 0, 1], [1, 0, 0], [0, 1, 0]], atol=1e-12)
    assert np.allclose(T_BODY_CAM, [0.0693, -0.0057, -0.011])


def test_model_geometry():
    f0 = MODEL.faces[0]                 # bottom, left, top, right
    assert abs(np.linalg.norm(f0[2] - f0[0]) - 0.2032) < 1e-4
    assert abs(np.linalg.norm(f0[3] - f0[1]) - 0.2032) < 1e-4
    assert abs((f0[2] - f0[0]) @ (f0[3] - f0[1])) < 1e-9
    normal = np.cross(f0[2] - f0[0], f0[3] - f0[1])
    assert abs(normal @ (f0[1] - f0[0])) < 1e-9          # coplanar
    tilt = math.degrees(math.asin(abs(normal[2]) / np.linalg.norm(normal)))
    assert 16.5 < tilt < 17.5                            # ~17 deg off horizontal normal


def test_camera_roundtrip():
    P = np.array([[0.3, -0.2, 2.0], [-1.5, 0.4, 3.0], [2.0, 1.0, 1.5]])
    n = CAM.normalize(CAM.project(P))
    assert np.allclose(n, P[:, :2] / P[:, 2:], atol=1e-6)


def test_camera_strided_input():
    # Detector keypoints are (K, 3); callers pass the [:, :2] view. OpenCV misread
    # that strided buffer silently (garbage points, so PnP never fit on real frames).
    P = np.array([[0.3, -0.2, 2.0], [-1.5, 0.4, 3.0], [2.0, 1.0, 1.5]])
    px = CAM.project(P)
    kpts = np.c_[px, np.full(len(px), 0.9)]
    assert not kpts[:, :2].flags["C_CONTIGUOUS"]
    assert np.allclose(CAM.normalize(kpts[:, :2]), CAM.normalize(px), atol=1e-12)
    P4 = np.c_[P, np.ones(len(P))]
    assert np.allclose(CAM.project(P4[:, :3]), px, atol=1e-9)


def test_state_buffer_vector():
    b = bp.StateBuffer()
    for t, v in [(10.0, (0, 0, 0)), (10.1, (1, 2, 0)), (10.1, (9, 9, 9)), (10.05, (9, 9, 9))]:
        b.add(t, v)                                 # duplicate / out-of-order ignored
    assert np.allclose(b.at(10.025), [0.25, 0.5, 0])
    assert b.at(9.99) is None                       # before the history
    assert np.allclose(b.at(10.12), [1, 2, 0])      # within max_extrapolate: newest
    assert b.at(10.2) is None                       # stale


def test_state_buffer_quaternion():
    # 90 deg yaw over 0.1 s; at a 35 ms capture lag the latest sample is 31.5 deg off.
    b = bp.StateBuffer(quat=True)
    b.add(0.0, (1, 0, 0, 0))
    b.add(0.1, (math.cos(math.pi / 4), 0, 0, math.sin(math.pi / 4)))
    q = b.at(0.065)
    yaw = math.degrees(2 * math.atan2(q[3], q[0]))
    assert abs(yaw - 58.5) < 1e-6 and abs(np.linalg.norm(q) - 1) < 1e-12


def test_lla_roundtrip():
    ref = (26.7117290, -80.2865831, 5.0)
    for ned in ([0, 0, 0], [120.0, -45.0, -3.0], [-800.0, 600.0, 10.0]):
        lla = bp.ned_to_lla(ned, *ref)
        assert np.allclose(bp.lla_to_ned(*lla, *ref), ned, atol=1e-6)


def test_front_faces_wind_positive():
    # Every diamond rendered from its front must pass the winding check.
    rng = np.random.default_rng(4)
    n = 0
    for _ in range(300):
        diamonds, _, _ = render(rng, range_m=rng.uniform(2, 6), alt_m=1.5, noise_px=0.0)
        for d in diamonds:
            ok, why = bp.diamond_quality(d, bp.LocateParams(min_diag_px=0))
            assert why != "back_facing", why
            n += 1
    assert n >= 100


def test_swapped_left_right_rejected():
    # left/right swapped == the diamond seen from behind, which can't happen.
    rng = np.random.default_rng(5)
    for _ in range(50):
        diamonds, pose, base = render(rng, range_m=3.0, alt_m=1.5, noise_px=0.0)
        if diamonds:
            break
    d = diamonds[0]
    swapped = bp.DiamondObs(d.kpts[[0, 3, 2, 1]], d.conf)
    assert bp.diamond_quality(swapped, bp.LocateParams())[1] == "back_facing"
    fixes = bp.locate_beacons([swapped], CAM, pose, water_z_ned=0.0)
    assert len(fixes) == 1 and fixes[0].method == "ray"


def test_collapsed_keypoints_rejected():
    # A real quad fills its box; corners pulled toward the box centre (the
    # off-distribution failure seen on tracking_front) must be rejected.
    rng = np.random.default_rng(6)
    for _ in range(50):
        diamonds, _, _ = render(rng, range_m=3.0, alt_m=1.5, noise_px=0.0)
        if diamonds:
            break
    d = diamonds[0]
    q = np.asarray(d.kpts, float)
    box = (*q.min(0), *q.max(0))
    p = bp.LocateParams(min_diag_px=0)
    assert bp.diamond_quality(bp.DiamondObs(q, d.conf, bbox=box), p) == (True, "ok")
    c = q.mean(0)
    collapsed = bp.DiamondObs(c + 0.35 * (q - c), d.conf, bbox=box)
    assert bp.diamond_quality(collapsed, p)[1] == "kp_collapsed"


def test_pose_config_complete_and_strict():
    cfg_path = os.path.join(os.path.dirname(__file__), "..", "configs", "beacon_pose.json")
    params, nms = bp.load_pose_config(cfg_path)
    import json
    keys = set(json.load(open(cfg_path))) - {"nms_iou"}
    assert keys == {f.name for f in bp.fields(bp.LocateParams)}   # every knob is in the file
    assert 0 < nms < 1 and params.kp_dedupe is True
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
        json.dump({"nms_iou": 0.5, "kp_dedup": True}, f)               # typo
    try:
        bp.load_pose_config(f.name)
        assert False, "typo'd key accepted"
    except ValueError:
        pass
    finally:
        os.unlink(f.name)


def test_dedupe_drops_duplicate_keeps_adjacent():
    rng = np.random.default_rng(6)
    for _ in range(400):
        diamonds, pose, base = render(rng, range_m=3.0, alt_m=1.5, noise_px=0.3)
        if len(diamonds) == 2:
            break
    assert len(diamonds) == 2
    a, b = diamonds
    a.score, b.score = 0.94, 0.83
    # Second, weaker detection of diamond a (cf. the 0.94 / 0.49 pair on the
    # RoboFlow frame, box IoU ~0.6): corners shifted by ~1-2 px.
    dup = bp.DiamondObs(a.kpts + rng.normal(0, 1.0, (4, 2)), a.conf, score=0.49)
    obs = [a, b, dup]
    assert bp.dedupe_diamonds(obs, 0.15) == [0, 1]                    # adjacent pair survives
    fixes = bp.locate_beacons(obs, CAM, pose, water_z_ned=0.0)
    assert len(fixes) == 1 and fixes[0].method == "pnp"
    assert np.linalg.norm(fixes[0].pos_ned[:2] - base[:2]) < 0.1
    assert sorted(fixes[0].members) == [0, 1]                          # original indices


def test_box_only_far_range():
    rng = np.random.default_rng(7)
    errs, in3 = [], 0
    for _ in range(150):
        _, pose, base, box = render(rng, range_m=25.0, alt_m=6.0, noise_px=1.0, with_box=True)
        if box is None:
            continue
        fixes = bp.locate_beacons([], CAM, pose, water_z_ned=0.0, boxes=[box])
        assert len(fixes) == 1 and fixes[0].method == "box" and fixes[0].box_members == [0]
        e = fixes[0].pos_ned[:2] - base[:2]
        errs.append(np.linalg.norm(e))
        in3 += float(e @ np.linalg.solve(fixes[0].cov[:2, :2], e)) < 11.62
    assert len(errs) > 50
    assert np.median(errs) < 1.0
    assert in3 / len(errs) > 0.9


def test_box_absorbs_small_diamonds():
    rng = np.random.default_rng(8)
    seen = 0
    for _ in range(300):
        diamonds, pose, base, box = render(rng, range_m=10.0, alt_m=4.0, noise_px=0.5, with_box=True)
        if box is None or not diamonds:
            continue
        plain = bp.locate_beacons(diamonds, CAM, pose, 0.0)
        if not plain or plain[0].method != "ray":
            continue                                  # want the small-diamond case
        fixes = bp.locate_beacons(diamonds, CAM, pose, 0.0, boxes=[box])
        assert len(fixes) == 1 and fixes[0].method == "box"
        assert fixes[0].members == list(range(len(diamonds))) and fixes[0].box_members == [0]
        seen += 1
    assert seen >= 10


def test_box_attaches_to_pnp():
    rng = np.random.default_rng(9)
    seen = 0
    for _ in range(100):
        diamonds, pose, base, box = render(rng, range_m=3.0, alt_m=1.5, noise_px=0.5, with_box=True)
        if box is None or not diamonds:
            continue
        fixes = bp.locate_beacons(diamonds, CAM, pose, 0.0, boxes=[box])
        assert len(fixes) == 1 and fixes[0].method == "pnp" and fixes[0].box_members == [0]
        seen += 1
    assert seen >= 10


def test_box_inconsistent_rejected():
    # Water height wrong by 3 m: the bottom ray and the size range disagree.
    rng = np.random.default_rng(10)
    rejected = n = 0
    for _ in range(100):
        _, pose, base, box = render(rng, range_m=15.0, alt_m=5.0, noise_px=1.0, with_box=True)
        if box is None:
            continue
        n += 1
        rejected += not bp.locate_beacons([], CAM, pose, water_z_ned=3.0, boxes=[box])
    assert n > 30 and rejected / n > 0.8


def test_pnp_noiseless_exact():
    rng = np.random.default_rng(0)
    n_checked = 0
    for _ in range(40):
        diamonds, pose, base = render(rng, range_m=3.0, alt_m=1.5, noise_px=0.0)
        if not diamonds:
            continue
        fixes = bp.locate_beacons(diamonds, CAM, pose, water_z_ned=0.0)
        assert len(fixes) == 1
        assert fixes[0].method == "pnp"
        assert np.linalg.norm(fixes[0].pos_ned - base) < 1e-3
        n_checked += 1
    assert n_checked >= 20


def test_two_faces_one_beacon():
    rng = np.random.default_rng(1)
    seen = 0
    for _ in range(200):
        diamonds, pose, base = render(rng, range_m=3.0, alt_m=1.5, noise_px=0.5)
        if len(diamonds) < 2:
            continue
        fixes = bp.locate_beacons(diamonds, CAM, pose, water_z_ned=0.0)
        assert len(fixes) == 1 and len(fixes[0].members) == len(diamonds)
        seen += 1
    assert seen >= 10


def test_ray_far_range():
    rng = np.random.default_rng(2)
    errs = []
    for _ in range(100):
        diamonds, pose, base = render(rng, range_m=25.0, alt_m=6.0, noise_px=1.0)
        if not diamonds:
            continue
        fixes = bp.locate_beacons(diamonds, CAM, pose, water_z_ned=0.0)
        assert len(fixes) == 1 and fixes[0].method == "ray"
        errs.append(np.linalg.norm(fixes[0].pos_ned - base))
    assert np.median(errs) < 1.0


def test_beacon_map_converges():
    rng = np.random.default_rng(3)
    clean, pose, base = render(np.random.default_rng(100), 4.0, 2.0, noise_px=0.0)
    assert clean
    m = bp.BeaconMap()
    for i in range(60):
        noisy = [bp.DiamondObs(d.kpts + rng.normal(0, 1.0, (4, 2)), d.conf) for d in clean]
        m.update(bp.locate_beacons(noisy, CAM, pose, 0.0), t=float(i))
    assert len(m.beacons) == 1
    assert np.linalg.norm(m.beacons[0]["pos"] - base) < 0.1


def _fix(pos, sigma=0.1):
    return bp.BeaconFix(np.asarray(pos, float), sigma ** 2 * np.eye(3), "box", [], 7.0)


def test_beacon_map_follows_drift():
    # One floating beacon drifting 2 m in 70 s (the 2026-10-15 pool replay), fixes
    # at 10 Hz with 0.5 m scatter but a 0.1 m reported sigma: a single entry that
    # ends near the beacon's final position.
    rng = np.random.default_rng(5)
    m = bp.BeaconMap(cov_floor_m=0.5, drift_mps=0.15)
    for k in range(700):
        t = k / 10
        true = np.array([5.4 + 1.3 * t / 70, -7.4 + 2.0 * t / 70, 1.6])
        m.update([_fix(true + np.r_[rng.normal(0, 0.5, 2), 0])], t)
    assert len(m.beacons) == 1, [b["pos"] for b in m.beacons]
    assert np.linalg.norm(m.beacons[0]["pos"][:2] - [6.7, -5.4]) < 0.6


def test_beacon_map_keeps_separate_beacons():
    # Two beacons 5 m apart, seen alternately and out of view for 60 s: both stay,
    # and seeing the first again re-attaches instead of spawning a third.
    rng = np.random.default_rng(6)
    m = bp.BeaconMap(cov_floor_m=0.5, drift_mps=0.15)
    A, B = np.array([0.0, 0.0, 1.6]), np.array([5.0, 0.0, 1.6])
    for k in range(200):
        m.update([_fix((A if k % 2 else B) + np.r_[rng.normal(0, 0.5, 2), 0])], k / 10)
    for k in range(20):
        m.update([_fix(A + np.r_[0.8, 0.5, 0] + np.r_[rng.normal(0, 0.5, 2), 0])], 80 + k / 10)
    assert len(m.beacons) == 2, [b["pos"] for b in m.beacons]
    pa = min(m.beacons, key=lambda b: np.linalg.norm(b["pos"] - A))
    assert pa["n"] == 120


def test_beacon_map_static_unchanged():
    # drift 0 is plain information fusion: the mean of equal-weight fixes.
    m = bp.BeaconMap(cov_floor_m=0.0)
    pts = [np.r_[1.0, 2.0, 0.0], np.r_[1.2, 1.8, 0.0], np.r_[0.8, 2.1, 0.0]]
    for k, p in enumerate(pts):
        m.update([_fix(p, 0.3)], float(k))
    assert len(m.beacons) == 1
    assert np.allclose(m.beacons[0]["pos"], np.mean(pts, axis=0), atol=1e-9)


# ─── Report ───────────────────────────────────────────────────────────

def report(trials=150, noise_px=1.0, att_err_deg=0.5, alt_err_m=0.3):
    print(f"tracking_front fisheye, keypoint noise {noise_px} px, "
          f"PX4 attitude error {att_err_deg} deg, altitude error {alt_err_m} m (1 sigma)")
    print(f"{'range':>6} {'alt':>4} {'n':>4} {'pnp%':>5} {'horiz med':>9} {'horiz p90':>9} "
          f"{'vert med':>8} {'h in 3sig':>9} {'diag px':>8}")
    for rng_m, alt in [(2, 1), (3, 1.5), (4, 2), (6, 3), (8, 4), (12, 5), (20, 6), (30, 8)]:
        rng = np.random.default_rng(int(rng_m * 10))
        errs, verr, pnp, ok3, diag = [], [], 0, 0, []
        for _ in range(trials):
            diamonds, pose, base = render(rng, rng_m, alt, noise_px)
            if not diamonds:
                continue
            # Solver sees PX4's estimate, not the truth: attitude and altitude errors.
            dR = bp.rot_x(math.radians(rng.normal(0, att_err_deg))) @ \
                bp.rot_y(math.radians(rng.normal(0, att_err_deg)))
            est = bp.CameraPose(dR @ pose.R_ned_cam,
                                pose.c_ned + np.array([0, 0, rng.normal(0, alt_err_m)]))
            fixes = bp.locate_beacons(diamonds, CAM, est, 0.0)
            if len(fixes) != 1:
                continue
            f = fixes[0]
            e = f.pos_ned - base
            errs.append(np.linalg.norm(e[:2]))
            verr.append(abs(e[2]))
            pnp += f.method == "pnp"
            ok3 += float(e[:2] @ np.linalg.solve(f.cov[:2, :2] + 1e-9 * np.eye(2), e[:2])) < 11.62  # chi2(2) 99.7%
            diag.append(np.median([np.linalg.norm(d.kpts[2] - d.kpts[0]) for d in diamonds]))
        if errs:
            print(f"{rng_m:>5}m {alt:>3}m {len(errs):>4} {100 * pnp / len(errs):>4.0f}% "
                  f"{np.median(errs):>8.3f}m {np.percentile(errs, 90):>8.3f}m "
                  f"{np.median(verr):>7.3f}m {100 * ok3 / len(errs):>8.0f}% {np.median(diag):>8.1f}")

def report_box(trials=150, att_err_deg=0.5, alt_err_m=0.3):
    print(f"\nbeacon box only (no diamonds), box edges +/-1 px, PX4 attitude error "
          f"{att_err_deg} deg, altitude error {alt_err_m} m")
    print(f"{'range':>6} {'alt':>4} {'n':>4} {'kept':>5} {'horiz med':>9} {'horiz p90':>9} "
          f"{'h in 3sig':>9} {'box h px':>8}")
    for rng_m, alt in [(8, 4), (12, 5), (20, 6), (30, 8), (45, 8), (60, 10)]:
        rng = np.random.default_rng(int(rng_m * 7))
        errs, ok3, hpx, n = [], 0, [], 0
        for _ in range(trials):
            _, pose, base, box = render(rng, rng_m, alt, 1.0, with_box=True)
            if box is None:
                continue
            n += 1
            dR = bp.rot_x(math.radians(rng.normal(0, att_err_deg))) @ \
                bp.rot_y(math.radians(rng.normal(0, att_err_deg)))
            est = bp.CameraPose(dR @ pose.R_ned_cam,
                                pose.c_ned + np.array([0, 0, rng.normal(0, alt_err_m)]))
            fixes = bp.locate_beacons([], CAM, est, 0.0, boxes=[box])
            if len(fixes) != 1:
                continue
            e = fixes[0].pos_ned[:2] - base[:2]
            errs.append(np.linalg.norm(e))
            ok3 += float(e @ np.linalg.solve(fixes[0].cov[:2, :2], e)) < 11.62
            hpx.append(box.bbox[3] - box.bbox[1])
        if errs:
            print(f"{rng_m:>5}m {alt:>3}m {n:>4} {100 * len(errs) / n:>4.0f}% {np.median(errs):>8.3f}m "
                  f"{np.percentile(errs, 90):>8.3f}m {100 * ok3 / len(errs):>8.0f}% {np.median(hpx):>8.1f}")


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"PASS {t.__name__}")
    print()
    report()
    report_box()
