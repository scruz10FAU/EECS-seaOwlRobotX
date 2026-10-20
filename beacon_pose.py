"""
Beacon localisation from YOLO-pose diamond keypoints.

Turns per-diamond keypoints (bottom, left, top, right) into the beacon's base
position in PX4's local NED frame, and from there into lat/lon via PX4's
reference origin. No ROS here -- pure numpy/OpenCV so it can be tested offline
and run in the drone container without scipy.

Frames:
  camera  OpenCV (x right, y down, z forward), as the image is delivered
  body    FRD (PX4 / ModalAI)
  NED     PX4 vehicle_local_position frame; q from vehicle_attitude is body->NED
  beacon  origin at base centre; stored z-DOWN (x, -y, -z of the measured
          z-up coordinates) so it composes directly with NED

Two methods per beacon:
  "pnp"  close range: per-diamond IPPE seeds a gravity-constrained 4-DoF
         (x, y, z, yaw) fit over every diamond of the beacon
  "box"  no usable diamond pose but a "beacon" bbox: range from the box's
         angular size, fused with a ray through its bottom edge to the water
  "ray"  far range / fallback: diamond-centre ray intersected with the water
         plane raised by the diamond-centre height
"""

import itertools
import json
import math
import re
from collections import deque
from dataclasses import dataclass, field, fields
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

KPT_NAMES = ("bottom", "left", "top", "right")   # YOLO keypoint index order
PX4_EARTH_RADIUS_M = 6371000.0                   # CONSTANTS_RADIUS_OF_EARTH

# Face-0 diamond corners, metres, beacon frame with Z UP, origin at base centre
# [reported, user-measured]. Faces 1-3 are this face rotated k*90 deg about Z.
FACE0_CORNERS_ZUP = {
    "top":    (0.0,     -0.14074, 0.32499),
    "bottom": (0.0,     -0.2,     0.13063),
    "left":   (-0.1016, -0.17037, 0.22781),
    "right":  (0.1016,  -0.17037, 0.22781),
}


# ─── Rotations ────────────────────────────────────────────────────────

def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def quat_wxyz_to_R(q):
    """Rotation matrix from a [w, x, y, z] quaternion (PX4 order)."""
    w, x, y, z = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


def _slerp(q0, q1, f):
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    d = float(q0 @ q1)
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + f * (q1 - q0)
        return q / np.linalg.norm(q)
    th = math.acos(d)
    return (math.sin((1 - f) * th) * q0 + math.sin(f * th) * q1) / math.sin(th)


# ─── Camera model ─────────────────────────────────────────────────────

@dataclass
class CameraModel:
    """Intrinsics + distortion. model is "fisheye" (cv2.fisheye) or "pinhole"."""
    K: np.ndarray
    D: np.ndarray
    model: str
    width: int
    height: int

    @classmethod
    def from_modalai_yml(cls, path):
        """Load /data/modalai/opencv_<cam>_intrinsics.yml."""
        fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            raise FileNotFoundError(path)
        K, D = fs.getNode("M").mat(), fs.getNode("D").mat()
        model = fs.getNode("distortion_model").string() or "pinhole"
        w, h = int(fs.getNode("width").real()), int(fs.getNode("height").real())
        fs.release()
        return cls(K.astype(float), D.reshape(-1).astype(float),
                   "fisheye" if "fisheye" in model else "pinhole", w, h)

    @property
    def f(self):
        return 0.5 * (self.K[0, 0] + self.K[1, 1])

    def normalize(self, px):
        """Pixel coords (N,2) -> undistorted normalised image coords (N,2)."""
        # Contiguous: OpenCV misreads a strided view (e.g. keypoints[:, :2] of an
        # (N,3) array) and returns garbage points without raising.
        pts = np.ascontiguousarray(px, dtype=float).reshape(-1, 1, 2)
        if self.model == "fisheye":
            out = cv2.fisheye.undistortPoints(pts, self.K, self.D.reshape(4, 1))
        else:
            out = cv2.undistortPoints(pts, self.K, self.D)
        return out.reshape(-1, 2)

    def project(self, P_cam):
        """Camera-frame points (N,3) -> pixel coords (N,2)."""
        P = np.ascontiguousarray(P_cam, dtype=float).reshape(-1, 1, 3)
        z = np.zeros((3, 1))
        if self.model == "fisheye":
            out, _ = cv2.fisheye.projectPoints(P, z, z, self.K, self.D.reshape(4, 1))
        else:
            out, _ = cv2.projectPoints(P, z, z, self.K, self.D)
        return out.reshape(-1, 2)


# ─── Extrinsics / poses ───────────────────────────────────────────────

def load_modalai_extrinsics(path, child, parent="body"):
    """Return (R_parent_child, t_parent_child) from /etc/modalai/extrinsics.conf.

    p_parent = R @ p_child + t. Walks the child->...->parent chain.
    RPY_parent_to_child is intrinsic XYZ in degrees, so R = Rx(r) Ry(p) Rz(y).
    """
    text = open(path).read()
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    text = re.sub(r"(?m)^\s*//.*$", "", text)
    entries = {e["child"]: e for e in json.loads(text)["extrinsics"]}
    R, t, node = np.eye(3), np.zeros(3), child
    while node != parent:
        if node not in entries:
            raise KeyError(f"no extrinsic chain from {child} to {parent} (stuck at {node})")
        e = entries[node]
        r, p, y = np.radians(e["RPY_parent_to_child"])
        R_e = rot_x(r) @ rot_y(p) @ rot_z(y)
        t_e = np.asarray(e["T_child_wrt_parent"], float)
        R, t = R_e @ R, R_e @ t + t_e
        node = e["parent"]
    return R, t


@dataclass
class CameraPose:
    R_ned_cam: np.ndarray   # camera axes expressed in NED
    c_ned: np.ndarray       # camera centre in NED


def camera_pose_from_drone(pos_ned, q_wxyz, R_body_cam, t_body_cam):
    """Compose PX4 body pose (vehicle_local_position + vehicle_attitude) with extrinsics."""
    R_ned_body = quat_wxyz_to_R(q_wxyz)
    return CameraPose(R_ned_body @ R_body_cam,
                      np.asarray(pos_ned, float) + R_ned_body @ t_body_cam)


class PoseBuffer:
    """Short history of (t, pos_ned, q_wxyz) for interpolating to image stamps."""

    def __init__(self, maxlen=400):
        self._buf = deque(maxlen=maxlen)

    def add(self, t, pos_ned, q_wxyz):
        if self._buf and t <= self._buf[-1][0]:
            return
        self._buf.append((float(t), np.asarray(pos_ned, float), np.asarray(q_wxyz, float)))

    def at(self, t, max_extrapolate=0.05):
        """Interpolated (pos, q) at time t, or None if t is outside the buffer."""
        if not self._buf:
            return None
        if t >= self._buf[-1][0]:
            return self._buf[-1][1:] if t - self._buf[-1][0] <= max_extrapolate else None
        if t < self._buf[0][0]:
            return None
        lo, hi = 0, len(self._buf) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if self._buf[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        (t0, p0, q0), (t1, p1, q1) = self._buf[lo], self._buf[hi]
        f = (t - t0) / (t1 - t0)
        return p0 + f * (p1 - p0), _slerp(q0, q1, f)


# ─── Geodesy (PX4 MapProjection, azimuthal equidistant) ───────────────

def ned_to_lla(ned, ref_lat, ref_lon, ref_alt):
    """Local NED (m) -> (lat, lon, alt AMSL) about PX4's ref_lat/ref_lon/ref_alt."""
    x, y, z = ned
    lat0, lon0 = math.radians(ref_lat), math.radians(ref_lon)
    xr, yr = x / PX4_EARTH_RADIUS_M, y / PX4_EARTH_RADIUS_M
    c = math.hypot(xr, yr)
    if c > 0:
        sc, cc = math.sin(c), math.cos(c)
        lat = math.asin(cc * math.sin(lat0) + xr * sc * math.cos(lat0) / c)
        lon = lon0 + math.atan2(yr * sc, c * math.cos(lat0) * cc - xr * math.sin(lat0) * sc)
    else:
        lat, lon = lat0, lon0
    return math.degrees(lat), math.degrees(lon), ref_alt - z


def lla_to_ned(lat, lon, alt, ref_lat, ref_lon, ref_alt):
    """Inverse of ned_to_lla."""
    lat0, lon0 = math.radians(ref_lat), math.radians(ref_lon)
    la, lo = math.radians(lat), math.radians(lon)
    cos_d = math.cos(lo - lon0)
    arg = max(-1.0, min(1.0, math.sin(lat0) * math.sin(la) + math.cos(lat0) * math.cos(la) * cos_d))
    c = math.acos(arg)
    k = c / math.sin(c) if abs(c) > 1e-12 else 1.0
    x = k * (math.cos(lat0) * math.sin(la) - math.sin(lat0) * math.cos(la) * cos_d) * PX4_EARTH_RADIUS_M
    y = k * math.cos(la) * math.sin(lo - lon0) * PX4_EARTH_RADIUS_M
    return np.array([x, y, ref_alt - alt])


# ─── Beacon model / observations ──────────────────────────────────────

@dataclass
class BeaconModel:
    corners_zup: dict = field(default_factory=lambda: dict(FACE0_CORNERS_ZUP))
    kpt_names: Tuple[str, ...] = KPT_NAMES
    height_m: float = 0.52605          # [reported] full beacon, base to top
    footprint_m: float = 0.559         # [reported] skirt, square
    top_width_m: float = 0.254         # [assumed] lit top, unmeasured: CAD white-section top width

    def __post_init__(self):
        f0 = np.array([self.corners_zup[n] for n in self.kpt_names], float)
        self.center_height = float(f0[:, 2].mean())           # diamond centre above base
        self.diag_tb = float(np.linalg.norm(
            np.subtract(self.corners_zup["top"], self.corners_zup["bottom"])))
        f0 = f0 * np.array([1.0, -1.0, -1.0])                  # z-up -> z-down, right-handed
        self.faces = np.stack([f0 @ rot_z(k * math.pi / 2).T for k in range(4)])  # (4,4,3)
        normals = []
        for q in self.faces:
            n = np.cross(q[2] - q[0], q[3] - q[1])
            if n[:2] @ q.mean(axis=0)[:2] < 0:               # point away from the axis
                n = -n
            normals.append(n / np.linalg.norm(n))
        self.normals = np.array(normals)                      # (4,3) outward unit normals

        # Bbox geometry under unknown yaw: the skirt spans footprint (face-on) to
        # footprint*sqrt(2) (corner-on). Treated as uniform between the two.
        lo, hi = self.footprint_m, self.footprint_m * math.sqrt(2)
        self.box_width_mean, self.box_width_sigma = (lo + hi) / 2, (hi - lo) / math.sqrt(12)
        # Near skirt edge -> base centre, along the view.
        self.edge_to_center_mean = self.box_width_mean / 2
        self.edge_to_center_sigma = self.box_width_sigma / 2
        # Depth that adds to the apparent height when seen from above: near skirt
        # edge to the far edge of the top.
        top_half = self.top_width_m * (1 + math.sqrt(2)) / 4
        self.box_depth_mean = self.edge_to_center_mean + top_half
        self.box_depth_sigma = math.hypot(self.edge_to_center_sigma,
                                          self.top_width_m * (math.sqrt(2) - 1) / 4 / math.sqrt(3))


@dataclass
class DiamondObs:
    """One YOLO-pose detection: keypoints in original-image pixels, KPT_NAMES order."""
    kpts: np.ndarray                  # (4,2)
    conf: np.ndarray                  # (4,)
    score: float = 1.0
    bbox: Optional[Tuple[float, float, float, float]] = None   # x1, y1, x2, y2

    def center_px(self, min_conf=0.3):
        ok = np.asarray(self.conf) >= min_conf
        if ok.sum() >= 2:
            return np.asarray(self.kpts)[ok].mean(axis=0)
        if self.bbox is not None:
            x1, y1, x2, y2 = self.bbox
            return np.array([(x1 + x2) / 2, (y1 + y2) / 2])
        return np.asarray(self.kpts).mean(axis=0)


@dataclass
class BeaconBoxObs:
    """One "beacon" class detection: pose-less bbox in original-image pixels."""
    bbox: Tuple[float, float, float, float]   # x1, y1, x2, y2
    score: float = 1.0


@dataclass
class LocateParams:
    min_kpt_conf: float = 0.5
    min_diag_px: float = 12.0          # top-bottom diagonal; below this PnP is noise
    min_edge_ratio: float = 0.15       # shortest / longest quad edge
    min_kp_box_frac: float = 0.5       # keypoint extent / bbox extent, worse axis; labels >= 0.65,
                                       #   collapsed predictions (all kpts near the centre) ~0.3-0.4
    max_tilt_deg: float = 25.0         # IPPE solution must be near-upright
    max_rms_px: float = 3.0            # joint fit residual gate
    sigma_px: float = 1.0              # keypoint noise floor
    sigma_att_deg: float = 0.5         # attitude noise
    sigma_water_z_m: float = 0.3       # water-plane height noise
    cluster_m: float = 0.6             # diamonds closer than this are one beacon
    cluster_frac: float = 0.15         # ... or within this fraction of range
    min_ray_down: float = 0.02         # ray must point at least this much down (NED z)
    kp_dedupe: bool = True             # drop diamonds whose keypoints duplicate a better one
    kp_dedupe_frac: float = 0.6        # ... all 4 corners within this fraction of its diagonal
    box_size_rel_sigma: float = 0.08   # box size-range error, relative: labelling looseness; use
                                       #   ~0.15 without the correction (look-down bias up to ~18% short)
    box_height_elev_correction: bool = True   # apparent height = H cos(e) + depth sin(e); False = H
    box_consistency_chi2: float = 13.82  # reject a box whose size and ray fixes disagree (0 = off)
    box_contain_margin: float = 0.1    # diamond centre may sit this fraction outside its beacon box
                                       #     (real duplicates ~0.37, adjacent faces >= 1.04)


@dataclass
class BeaconFix:
    pos_ned: np.ndarray                # beacon base centre
    cov: np.ndarray                    # 3x3, m^2
    method: str                        # "pnp" | "ray"
    members: List[int]                 # indices into the input diamond list
    range_m: float
    yaw_rad: Optional[float] = None    # modulo 90 deg (4-fold symmetry)
    rms_px: Optional[float] = None
    size_range_m: Optional[float] = None   # range from apparent size (diamond or box)
    box_members: List[int] = field(default_factory=list)   # indices into the input box list
    chi2: Optional[float] = None       # box: size-vs-ray disagreement (2 dof)


def load_pose_config(path):
    """configs/beacon_pose.json -> (LocateParams, nms_iou). Unknown keys are an error,
    so a typo can't silently fall back to a default."""
    with open(path) as f:
        cfg = json.load(f)
    nms_iou = float(cfg.pop("nms_iou", 0.65))
    unknown = set(cfg) - {f.name for f in fields(LocateParams)}
    if unknown:
        raise ValueError(f"{path}: unknown key(s) {sorted(unknown)}")
    return LocateParams(**cfg), nms_iou


def dedupe_diamonds(diamonds: Sequence[DiamondObs], frac: float) -> List[int]:
    """Indices of diamonds to keep after dropping keypoint duplicates.

    Box NMS can't separate a duplicate detection of one diamond (IoU ~0.6 seen
    on real frames) from genuine diamonds on adjacent faces (IoU ~0.4, higher
    when viewed corner-on). Keypoints can: a duplicate has every corner on top
    of the same-named corner of the better detection, while an adjacent-face
    diamond's same-named corners are a face width away. Highest score wins.
    """
    order = sorted(range(len(diamonds)), key=lambda i: -diamonds[i].score)
    kept = []
    for i in order:
        q = np.asarray(diamonds[i].kpts, float)
        for j in kept:
            r = np.asarray(diamonds[j].kpts, float)
            tol = frac * max(np.linalg.norm(r[2] - r[0]), np.linalg.norm(r[3] - r[1]), 1.0)
            if np.all(np.linalg.norm(q - r, axis=1) <= tol):
                break
        else:
            kept.append(i)
    return sorted(kept)


def diamond_quality(d: DiamondObs, p: LocateParams):
    """(ok, reason). Rejects collapsed / non-convex / tiny / low-confidence quads."""
    if np.min(d.conf) < p.min_kpt_conf:
        return False, "low_conf"
    q = np.asarray(d.kpts, float)                       # bottom, left, top, right: cyclic
    if d.bbox is not None and p.min_kp_box_frac > 0:
        # Off-distribution, the pose head regresses every corner toward the box
        # centre: a tiny quad inside a correctly sized box, which would pass
        # the shape checks below and read as a far-away diamond.
        x1, y1, x2, y2 = d.bbox
        if x2 > x1 and y2 > y1 and min(np.ptp(q[:, 0]) / (x2 - x1),
                                       np.ptp(q[:, 1]) / (y2 - y1)) < p.min_kp_box_frac:
            return False, "kp_collapsed"
    if np.linalg.norm(q[2] - q[0]) < p.min_diag_px:
        return False, "small"
    edges = np.roll(q, -1, axis=0) - q
    lens = np.linalg.norm(edges, axis=1)
    if lens.min() < p.min_edge_ratio * lens.max():
        return False, "degenerate"
    cross = edges[:, 0] * np.roll(edges, -1, axis=0)[:, 1] - edges[:, 1] * np.roll(edges, -1, axis=0)[:, 0]
    # Seen from the front, bottom -> left -> top -> right winds with positive
    # signed area in image coords (y down); fisheye and 180 deg rotation keep
    # that. The back of a diamond is never visible, so the other winding means
    # mislabelled keypoints (e.g. left/right swapped).
    if np.all(cross < 0):
        return False, "back_facing"
    if not np.all(cross > 0):
        return False, "non_convex"
    return True, "ok"


# ─── Solvers ──────────────────────────────────────────────────────────

def _num_jac(fun, x, r0, eps=1e-6):
    J = np.empty((r0.size, x.size))
    for j in range(x.size):
        h = eps * max(1.0, abs(x[j]))
        xp = x.copy()
        xp[j] += h
        J[:, j] = (fun(xp) - r0) / h
    return J


def _levenberg_marquardt(fun, x0, iters=40, lam=1e-3):
    x = np.asarray(x0, float).copy()
    r = fun(x)
    cost = float(r @ r)
    for _ in range(iters):
        J = _num_jac(fun, x, r)
        A, g = J.T @ J, J.T @ r
        while True:
            dx = -np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-12), g)
            r_new = fun(x + dx)
            c_new = float(r_new @ r_new)
            if c_new < cost:
                x, r, cost = x + dx, r_new, c_new
                lam = max(lam / 10, 1e-12)
                break
            lam *= 10
            if lam > 1e10:
                return x, r, _num_jac(fun, x, r)
        if np.linalg.norm(dx) < 1e-9 * (1 + np.linalg.norm(x)):
            break
    return x, r, _num_jac(fun, x, r)


def _ippe_init(d, cam, pose, model, p):
    """Per-diamond IPPE -> (base_ned, yaw, tilt_deg, err_px) of the most upright solution."""
    norm_pts = cam.normalize(d.kpts)
    n, rvecs, tvecs, errs = cv2.solvePnPGeneric(
        model.faces[0], norm_pts, np.eye(3), None, flags=cv2.SOLVEPNP_IPPE)
    if not n:
        return None
    errs = np.asarray(errs).reshape(-1) * cam.f
    best = None
    for rv, tv, e in zip(rvecs, tvecs, errs):
        if e > max(2 * errs.min(), errs.min() + p.sigma_px):
            continue
        R_cam_b = cv2.Rodrigues(rv)[0]
        centroid_cam = model.faces[0].mean(axis=0) @ R_cam_b.T + tv.reshape(3)
        if (R_cam_b @ model.normals[0]) @ -centroid_cam <= 0:
            continue                                          # back face: never visible
        R_ned_b = pose.R_ned_cam @ R_cam_b
        tilt = math.degrees(math.acos(np.clip(R_ned_b[2, 2], -1, 1)))
        if best is None or tilt < best[2]:
            base = pose.c_ned + pose.R_ned_cam @ tv.reshape(3)
            best = (base, math.atan2(R_ned_b[1, 0], R_ned_b[0, 0]), tilt, float(e))
    if best is None or best[2] > p.max_tilt_deg:
        return None
    return best


def _fit_cluster(obs, cam, pose, model, init_base, init_yaw):
    """Gravity-constrained fit of (x, y, z, yaw) over diamonds; tries face assignments."""
    R_cam_ned, c = pose.R_ned_cam.T, pose.c_ned
    obs_px = np.concatenate([d.kpts for d in obs]).astype(float)
    best = None
    for perm in itertools.permutations(range(1, 4), len(obs) - 1):
        faces = (0,) + perm

        def front_facing(x, faces=faces):
            Rz = rot_z(x[3])
            return all((Rz @ model.normals[k]) @ (c - x[:3] - Rz @ model.faces[k].mean(axis=0)) > 0
                       for k in faces)

        def resid(x, faces=faces):
            Rz = rot_z(x[3])
            P = np.concatenate([x[:3] + model.faces[k] @ Rz.T for k in faces])
            Pc = (P - c) @ R_cam_ned.T
            if np.any(Pc[:, 2] < 0.05) or not front_facing(x):
                return np.full(obs_px.size, 1e3)
            return (cam.project(Pc) - obs_px).reshape(-1)

        x0 = np.r_[init_base, init_yaw]
        if not front_facing(x0):
            continue                      # e.g. opposite faces: one would show its back
        x, r, J = _levenberg_marquardt(resid, x0)
        cost = float(r @ r)
        if best is None or cost < best[0]:
            best = (cost, x, r, J)
    return best[1:] if best else (None, None, None)


def locate_by_ray(cam, pose, center_px, water_z_ned, model, p):
    """Ray through the diamond centre ∩ plane at diamond-centre height. Returns (base, cov, range)."""
    def intersect(px, R_ned_cam, water_z):
        n = cam.normalize(np.asarray(px, float).reshape(1, 2))[0]
        d = R_ned_cam @ np.array([n[0], n[1], 1.0])
        d /= np.linalg.norm(d)
        if d[2] < p.min_ray_down:
            return None
        plane_z = water_z - model.center_height           # NED: up is negative
        s = (plane_z - pose.c_ned[2]) / d[2]
        if s <= 0:
            return None
        P = pose.c_ned + s * d
        return np.array([P[0], P[1], water_z]), s

    hit = intersect(center_px, pose.R_ned_cam, water_z_ned)
    if hit is None:
        return None
    base, rng = hit
    cov = np.zeros((3, 3))
    a = math.radians(p.sigma_att_deg)
    perturb = [(np.r_[p.sigma_px, 0], np.eye(3), 0.0),
               (np.r_[0, p.sigma_px], np.eye(3), 0.0),
               (np.zeros(2), rot_x(a), 0.0),
               (np.zeros(2), rot_y(a), 0.0),
               (np.zeros(2), np.eye(3), p.sigma_water_z_m)]
    for dpx, dR, dz in perturb:
        h = intersect(np.asarray(center_px) + dpx, dR @ pose.R_ned_cam, water_z_ned + dz)
        if h is None:
            return None
        e = h[0] - base
        cov += np.outer(e, e)
    return base, cov, rng


def _unit_rays(cam, R_ned_cam, px):
    n = cam.normalize(np.asarray(px, float).reshape(-1, 2))
    d = np.c_[n, np.ones(len(n))] @ R_ned_cam.T
    return d / np.linalg.norm(d, axis=1, keepdims=True)


def _horizontal_cov(u_xy, radial_sigma, lateral_sigma, z_sigma):
    """3x3 covariance from radial/lateral horizontal sigmas about direction u_xy."""
    u = np.r_[u_xy, 0.0]
    u /= np.linalg.norm(u)
    v = np.array([-u[1], u[0], 0.0])
    return (radial_sigma ** 2 * np.outer(u, u) + lateral_sigma ** 2 * np.outer(v, v)
            + np.diag([0.0, 0.0, z_sigma ** 2]))


def locate_box(cam, pose, box: BeaconBoxObs, water_z_ned, model, p) -> Optional[BeaconFix]:
    """Beacon bbox -> base-centre fix from (a) angular size and (b) a ray through
    the bbox bottom to the water, fused when both exist.

    (a) range = beacon height / angular height of the box. A small box is a far
        beacon seen at a shallow angle, so the side height is a fair stand-in for
        the apparent height (biased short by the look-down angle; optional
        correction). Needs no altitude; error grows ~linearly with range.
    (b) is exact about shape (the skirt sits on the water) but grazes the water
        when the beacon is near the horizon; its error grows ~range^2 / height.
    Returns None when neither works or when they disagree beyond
    p.box_consistency_chi2 (false detection, or a wrong water height).
    """
    x1, y1, x2, y2 = map(float, box.bbox)
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    ang = math.hypot(p.sigma_px / cam.f, math.radians(p.sigma_att_deg))   # per-ray angular noise

    # (a) Size: slant range along the centre ray from the box's angular height.
    rt, rb, rc = _unit_rays(cam, pose.R_ned_cam, [(cx, y1), (cx, y2), (cx, cy)])
    th = math.acos(float(np.clip(rt @ rb, -1, 1)))
    S, sS = model.height_m, 0.0
    if p.box_height_elev_correction:
        dep = math.asin(float(np.clip(rc[2], -1, 1)))         # depression below horizontal
        S = model.height_m * math.cos(dep) + model.box_depth_mean * abs(math.sin(dep))
        sS = model.box_depth_sigma * abs(math.sin(dep))
    ests = []
    if th > 1e-6 and y2 > y1:
        rel = math.sqrt((sS / S) ** 2 + p.box_size_rel_sigma ** 2 + (p.sigma_px / (y2 - y1)) ** 2)
        ests.append((S / th, S / th * rel))
    size_fix = None
    if ests and np.linalg.norm(rc[:2]) > 1e-6:
        w = np.array([1 / s ** 2 for _, s in ests])
        r_size = float(sum(r * wi for (r, _), wi in zip(ests, w)) / w.sum())
        s_size = float(1 / math.sqrt(w.sum()))
        P = pose.c_ned + r_size * rc
        hn = float(np.linalg.norm(rc[:2]))
        cov = _horizontal_cov(rc[:2], math.hypot(s_size * hn, r_size * ang * abs(rc[2])),
                              r_size * ang, p.sigma_water_z_m)
        size_fix = (np.array([P[0], P[1], water_z_ned]), cov, r_size)

    # (b) Ray through the bbox bottom (skirt on the water), pushed back to the base centre.
    def hit(px, R, wz):
        d = _unit_rays(cam, R, [px])[0]
        if d[2] < p.min_ray_down:
            return None
        s_ = (wz - pose.c_ned[2]) / d[2]
        if s_ <= 0 or np.linalg.norm(d[:2]) < 1e-6:
            return None
        P_ = pose.c_ned + s_ * d
        u = d[:2] / np.linalg.norm(d[:2])
        return np.array([P_[0] + model.edge_to_center_mean * u[0],
                         P_[1] + model.edge_to_center_mean * u[1], wz]), s_, u

    ray_fix = None
    h0 = hit((cx, y2), pose.R_ned_cam, water_z_ned)
    if h0 is not None:
        base, rng, u = h0
        a = math.radians(p.sigma_att_deg)
        cov = _horizontal_cov(u, model.edge_to_center_sigma, 0.0, 0.0)
        ok = True
        for dpx, dR, dz in [(np.r_[p.sigma_px, 0], np.eye(3), 0.0),
                            (np.r_[0, p.sigma_px], np.eye(3), 0.0),
                            (np.zeros(2), rot_x(a), 0.0),
                            (np.zeros(2), rot_y(a), 0.0),
                            (np.zeros(2), np.eye(3), p.sigma_water_z_m)]:
            h = hit(np.r_[cx, y2] + dpx, dR @ pose.R_ned_cam, water_z_ned + dz)
            if h is None:
                ok = False
                break
            e = h[0] - base
            e[2] = 0.0
            cov += np.outer(e, e)
        if ok:
            cov[2, 2] = p.sigma_water_z_m ** 2
            ray_fix = (base, cov, rng)

    if size_fix is None and ray_fix is None:
        return None
    if size_fix is None or ray_fix is None:
        pos, cov, rng = size_fix or ray_fix
        return BeaconFix(pos, cov, "box", [], float(rng),
                         size_range_m=size_fix[2] if size_fix else None)

    # Fuse horizontally (both put z on the water, so z is not independent).
    (ps, cs, rs), (pr, cr, rr_) = size_fix, ray_fix
    d = pr[:2] - ps[:2]
    chi2 = float(d @ np.linalg.solve(cs[:2, :2] + cr[:2, :2], d))
    if p.box_consistency_chi2 and chi2 > p.box_consistency_chi2:
        return None
    Is, Ir = np.linalg.inv(cs[:2, :2]), np.linalg.inv(cr[:2, :2])
    c2 = np.linalg.inv(Is + Ir)
    xy = c2 @ (Is @ ps[:2] + Ir @ pr[:2])
    cov = np.zeros((3, 3))
    cov[:2, :2] = c2
    cov[2, 2] = p.sigma_water_z_m ** 2
    return BeaconFix(np.array([xy[0], xy[1], water_z_ned]), cov, "box", [], float(rr_),
                     size_range_m=rs, chi2=chi2)


def _inside(pt, bbox, margin):
    x1, y1, x2, y2 = bbox
    mx, my = margin * (x2 - x1), margin * (y2 - y1)
    return x1 - mx <= pt[0] <= x2 + mx and y1 - my <= pt[1] <= y2 + my


def size_range(cam, d, model):
    """Range from the angle subtended by the top-bottom diagonal (approximate)."""
    n = cam.normalize(np.asarray(d.kpts)[[0, 2]])
    v = np.c_[n, np.ones(2)]
    v /= np.linalg.norm(v, axis=1, keepdims=True)
    ang = math.acos(np.clip(v[0] @ v[1], -1, 1))
    return model.diag_tb / ang if ang > 1e-6 else float("inf")


def locate_beacons(diamonds: Sequence[DiamondObs], cam: CameraModel, pose: CameraPose,
                   water_z_ned: float, model: Optional[BeaconModel] = None,
                   p: Optional[LocateParams] = None,
                   boxes: Sequence[BeaconBoxObs] = ()) -> List[BeaconFix]:
    """All diamonds and beacon boxes from one frame -> one BeaconFix per physical beacon.

    Priority per beacon: "pnp" (a qualifying diamond) > "box" (beacon bbox) >
    "ray" (diamond centre). A box claims the diamonds whose centres fall inside
    it; a box with no diamonds at all still yields a fix.
    """
    model = model or BeaconModel()
    p = p or LocateParams()
    keep = dedupe_diamonds(diamonds, p.kp_dedupe_frac) if p.kp_dedupe else range(len(diamonds))
    items = []
    for i in keep:
        d = diamonds[i]
        ok, _ = diamond_quality(d, p)
        init = _ippe_init(d, cam, pose, model, p) if ok else None
        ray = locate_by_ray(cam, pose, d.center_px(), water_z_ned, model, p)
        est = init[0] if init else (ray[0] if ray else None)
        if est is not None:
            items.append(dict(i=i, d=d, init=init, ray=ray, est=est))

    # Greedy clustering on the per-diamond base estimates.
    clusters = []
    for it in items:
        rng = float(np.linalg.norm(it["est"] - pose.c_ned))
        for cl in clusters:
            if np.linalg.norm(it["est"] - cl[0]["est"]) < max(p.cluster_m, p.cluster_frac * rng):
                cl.append(it)
                break
        else:
            clusters.append([it])

    fixes = []
    for cl in clusters:
        members = [it["i"] for it in cl]
        pnp = [it for it in cl if it["init"]][:4]
        sr = [size_range(cam, it["d"], model) for it in pnp] or None
        if pnp:
            base0, yaw0 = pnp[0]["init"][0], pnp[0]["init"][1]
            x, r, J = _fit_cluster([it["d"] for it in pnp], cam, pose, model, base0, yaw0)
            rms = float(np.sqrt(np.mean(r ** 2))) if x is not None else float("inf")
            if rms <= p.max_rms_px:
                sig = max(rms, p.sigma_px)
                cov = sig ** 2 * np.linalg.pinv(J.T @ J)[:3, :3]
                rng = float(np.linalg.norm(x[:3] - pose.c_ned))
                cov += (rng * math.radians(p.sigma_att_deg)) ** 2 * np.eye(3)
                cov[2, 2] += p.sigma_water_z_m ** 2      # camera height above water is uncertain too
                fixes.append(BeaconFix(x[:3], cov, "pnp", members, rng,
                                       yaw_rad=float(x[3] % (math.pi / 2)), rms_px=rms,
                                       size_range_m=float(np.median(sr))))
                continue
        rays = [it["ray"] for it in cl if it["ray"]]
        if rays:
            fixes.append(BeaconFix(np.mean([r[0] for r in rays], axis=0),
                                   np.mean([r[1] for r in rays], axis=0), "ray", members,
                                   float(np.mean([r[2] for r in rays])),
                                   size_range_m=float(np.median(sr)) if sr else None))

    for bi, b in enumerate(boxes):
        inside = {i for i in keep if _inside(diamonds[i].center_px(), b.bbox, p.box_contain_margin)}
        owners = [f for f in fixes if inside & set(f.members)]
        pnp_owners = [f for f in owners if f.method == "pnp"]
        rays_inside = {id(f) for f in owners if f.method == "ray" and set(f.members) <= inside}
        if pnp_owners:
            for f in pnp_owners:
                f.box_members.append(bi)
            fixes = [f for f in fixes if id(f) not in rays_inside]
            continue
        bf = locate_box(cam, pose, b, water_z_ned, model, p)
        if bf is None:
            continue                              # keep any diamond-ray fixes as they are
        bf.members = sorted(inside | {m for f in owners if id(f) in rays_inside for m in f.members})
        bf.box_members = [bi]
        fixes = [f for f in fixes if id(f) not in rays_inside] + [bf]
    return fixes



# ─── Temporal fusion ──────────────────────────────────────────────────

class BeaconMap:
    """Beacon map in NED: one Kalman filter per beacon, kept for the whole mission.

    Association uses horizontal distance (beacons are sparse); a fix that is
    within gate_m but statistically inconsistent (Mahalanobis > chi2_gate) is
    counted as an outlier rather than fused or spawned as a new beacon.

    drift_mps: horizontal random walk per beacon (m/sqrt(s)). A beacon's
    covariance grows by drift_mps^2 * dt between fixes, so recent fixes weigh
    more and the estimate follows a floating beacon (or slow drone GPS drift)
    instead of splitting it into several entries; a beacon out of view keeps
    its entry and re-attaches with a wider gate when seen again. 0 = static
    beacons (plain information fusion).

    After each update, entries within gate_m of each other whose estimates
    agree (chi2_gate) are merged into the older one.
    """

    def __init__(self, gate_m=3.0, chi2_gate=11.34, cov_floor_m=0.05, drift_mps=0.0):
        self.gate_m, self.chi2_gate, self.cov_floor = gate_m, chi2_gate, cov_floor_m ** 2
        self.q = drift_mps ** 2
        self.beacons = []
        self._next_id = 0

    def cov_at(self, b, t):
        """Beacon covariance predicted to time t (drift since its last fix)."""
        cov = b["cov"].copy()
        dt = max(0.0, t - b["last_t"])
        cov[0, 0] += self.q * dt
        cov[1, 1] += self.q * dt
        return cov

    def update(self, fixes: Sequence[BeaconFix], t: float):
        touched = []
        for f in fixes:
            cov = f.cov + self.cov_floor * np.eye(3)
            best, best_d = None, None
            for b in self.beacons:
                dh = float(np.linalg.norm(f.pos_ned[:2] - b["pos"][:2]))
                if dh < self.gate_m and (best_d is None or dh < best_d):
                    best, best_d = b, dh
            if best is None:
                best = dict(id=self._next_id, n=1, outliers=0, pos=f.pos_ned.copy(), cov=cov,
                            last_t=t)
                self._next_id += 1
                self.beacons.append(best)
                touched.append(best)
                continue
            P = self.cov_at(best, t)
            e = f.pos_ned - best["pos"]
            S = P + cov
            if float(e @ np.linalg.solve(S, e)) > self.chi2_gate:
                best["outliers"] += 1
                touched.append(best)
                continue
            K = P @ np.linalg.inv(S)
            best["pos"] = best["pos"] + K @ e
            best["cov"] = (np.eye(3) - K) @ P
            best["cov"] = 0.5 * (best["cov"] + best["cov"].T)
            best["n"] += 1
            best["last_t"] = t
            touched.append(best)
        self._merge(t)
        return [b for b in touched if b in self.beacons]

    def _merge(self, t):
        merged = True
        while merged:
            merged = False
            for a, b in itertools.combinations(sorted(self.beacons, key=lambda x: x["id"]), 2):
                if np.linalg.norm(a["pos"][:2] - b["pos"][:2]) >= self.gate_m:
                    continue
                Pa, Pb = self.cov_at(a, t), self.cov_at(b, t)
                d = a["pos"] - b["pos"]
                if float(d @ np.linalg.solve(Pa + Pb, d)) > self.chi2_gate:
                    continue
                Ia, Ib = np.linalg.inv(Pa), np.linalg.inv(Pb)
                a["cov"] = np.linalg.inv(Ia + Ib)
                a["pos"] = a["cov"] @ (Ia @ a["pos"] + Ib @ b["pos"])
                a["n"] += b["n"]
                a["outliers"] += b["outliers"]
                a["last_t"] = t
                self.beacons.remove(b)
                merged = True
                break
