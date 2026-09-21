"""
seabird_config.py — Single source of truth for all Seabird parameters.

Every script imports from here. No magic numbers scattered across files.
If a value changes (camera swap, new buoy layout, different resolution),
change it HERE and everything stays consistent.

Lives at: ~/seabird/scripts/seabird_config.py
"""

import os
import numpy as np

# ═══════════════════════════════════════════════════════════════
# PATHS
# ═══════════════════════════════════════════════════════════════

HOME = os.path.expanduser("~")
SCRIPTS_DIR   = f"{HOME}/seabird/scripts"
DATASET_DIR   = f"{HOME}/seabird_dataset"
LOGS_DIR      = f"{HOME}/seabird/logs"
PX4_DIR       = f"{HOME}/seabird/PX4-Autopilot"
ASSETS_DIR    = f"{HOME}/seabird/assets"

# Isaac / USD prim paths
DRONE_PRIM_PATH  = "/World/Iris"
DRONE_BODY_PATH  = "/World/Iris/body"
CAMERA_PRIM_PATH = "/World/Iris/body/front_cam"


# ═══════════════════════════════════════════════════════════════
# CAMERA — ZED 2i Wide Lens
# ═══════════════════════════════════════════════════════════════
# These are the USD prim attributes that init_scene.py writes.
# The labeler and any other script that needs intrinsics computes
# them from these values — never hardcoded separately.

CAMERA_FOCAL_LENGTH_MM  = 2.1     # ZED 2i wide lens
CAMERA_H_APERTURE_MM    = 6.0     # horizontal sensor size
CAMERA_V_APERTURE_MM    = 4.5     # vertical sensor size
CAMERA_CLIPPING_NEAR    = 0.1     # meters
CAMERA_CLIPPING_FAR     = 200.0   # meters

# Render resolution (must match Replicator render_product)
IMG_W = 640
IMG_H = 480

# Derived intrinsics — computed once, used everywhere
#   fx = focal_length_mm * image_width_px / h_aperture_mm
#   fy = focal_length_mm * image_height_px / v_aperture_mm
CAMERA_FX = CAMERA_FOCAL_LENGTH_MM * IMG_W / CAMERA_H_APERTURE_MM   # ≈ 224.0
CAMERA_FY = CAMERA_FOCAL_LENGTH_MM * IMG_H / CAMERA_V_APERTURE_MM   # ≈ 224.0
CAMERA_CX = IMG_W / 2.0   # 320.0
CAMERA_CY = IMG_H / 2.0   # 240.0

# Camera mount — position relative to drone body frame, pitch angle
CAM_OFFSET_BODY  = np.array([0.30, 0.0, 0.05])   # meters, in body FLU frame
CAM_PITCH_DEG    = 15.0                            # nose-down tilt


# ═══════════════════════════════════════════════════════════════
# BUOYS
# ═══════════════════════════════════════════════════════════════

BUOY_RADIUS_M = 0.2286   # 18" diameter / 2

# Class mapping (the "bouy" misspelling matches the USD prim names)
CLASS_MAP = {
    "red_buoy":   0,
    "green_buoy": 1,
    "blue_buoy":  2,
}
CLASS_NAMES = ["red_buoy", "green_buoy", "blue_buoy"]

# Colors for debug drawing (BGR for OpenCV)
CLASS_COLORS_BGR = [(0, 0, 255), (0, 255, 0), (255, 0, 0)]

# Known world positions — matches init_scene.py spawn locations
# These are the authoritative positions; the labeler uses these directly
# rather than reading from USD (which can have transform chain issues).
BUOY_POSITIONS = {
    "red_buoy":   (3.079, -13.458, 0.150),   # bbox center — mesh offset from prim origin!
    "green_buoy": (-3.284, -14.000, 0.000),   # bbox center == prim origin
    "blue_buoy":  (0.000, -15.299, 0.000),    # bbox center ≈ prim origin
}


# ═══════════════════════════════════════════════════════════════
# DRONE SPAWN
# ═══════════════════════════════════════════════════════════════

DRONE_SPAWN_POS = [0.0, -8.0, 2.5]              # meters, world frame
DRONE_SPAWN_QUAT_WXYZ = [0.707, 0.0, 0.0, -0.707]  # facing -Y toward buoys


# ═══════════════════════════════════════════════════════════════
# PX4
# ═══════════════════════════════════════════════════════════════

PX4_CONNECTION_TYPE = "tcpin"       # Rule #5: NOT "tcp" or "tcpout"
PX4_TAKEOFF_ALT_M   = 1.25         # MIS_TAKEOFF_ALT (must set each session if not param saved)


# ═══════════════════════════════════════════════════════════════
# BODY-TO-CAMERA ROTATION (derived from mount config)
# ═══════════════════════════════════════════════════════════════
# Computed once at import time. Used by labeler, future detector, etc.
#
# Isaac body frame (FLU): X=forward, Y=left, Z=up
# OpenCV camera frame:    X=right,   Y=down, Z=forward
#
# Without pitch:
#   cam_X (right)   = -body_Y
#   cam_Y (down)    = -body_Z
#   cam_Z (forward) =  body_X

_R_BASE = np.array([
    [0, -1,  0],
    [0,  0, -1],
    [1,  0,  0],
], dtype=np.float64)

_pr = np.radians(CAM_PITCH_DEG)
_R_PITCH = np.array([
    [1,  0,           0          ],
    [0,  np.cos(_pr), -np.sin(_pr)],
    [0,  np.sin(_pr),  np.cos(_pr)],
], dtype=np.float64)

R_BODY_TO_CAM = _R_PITCH @ _R_BASE


# ═══════════════════════════════════════════════════════════════
# LABELER DEFAULTS
# ═══════════════════════════════════════════════════════════════

LABELER_SAVE_EVERY_N = 10      # save every Nth render frame
LABELER_MAX_FRAMES   = 2000    # stop after this many saved frames
LABELER_DEBUG_MODE   = True    # draw bboxes on debug images
LABELER_MIN_BBOX_PX  = 6       # minimum bbox dimension to save


# ═══════════════════════════════════════════════════════════════
# CONVENIENCE — print config summary
# ═══════════════════════════════════════════════════════════════

def print_camera_config():
    """Print camera config for verification in Script Editor console."""
    hfov = 2 * np.degrees(np.arctan(CAMERA_H_APERTURE_MM / (2 * CAMERA_FOCAL_LENGTH_MM)))
    print(f"[config] Camera: ZED 2i wide — fl={CAMERA_FOCAL_LENGTH_MM}mm  "
          f"h_ap={CAMERA_H_APERTURE_MM}mm  v_ap={CAMERA_V_APERTURE_MM}mm")
    print(f"[config] Resolution: {IMG_W}x{IMG_H}")
    print(f"[config] Intrinsics: fx={CAMERA_FX:.1f}  fy={CAMERA_FY:.1f}  "
          f"cx={CAMERA_CX:.1f}  cy={CAMERA_CY:.1f}")
    print(f"[config] HFOV: {hfov:.1f}°  Pitch: {CAM_PITCH_DEG}°")
    print(f"[config] Mount offset: {CAM_OFFSET_BODY}")
    
    
# ═══════════════════════════════════════════════════════════════
# CONVENIENCE ALIASES — shorter names for scripts that import often
# ═══════════════════════════════════════════════════════════════
# If these already exist in your file, skip this block.

FX = CAMERA_FX
FY = CAMERA_FY
CX = CAMERA_CX
CY = CAMERA_CY


# ═══════════════════════════════════════════════════════════════
# TRANSFORMS — camera frame → world frame
# ═══════════════════════════════════════════════════════════════

def camera_to_world(p_cam, drone_pos, drone_quat_wxyz):
    """
    Transform a 3D point from camera frame to Isaac world frame (ENU).

    Chain: camera → body → world
      p_body = R_cam_to_body @ p_cam + mount_offset
      p_world = R_body_to_world @ p_body + drone_position

    Args:
        p_cam:  (3,) array — point in OpenCV camera frame (X-right, Y-down, Z-forward)
        drone_pos: (3,) array — drone position in Isaac world frame
        drone_quat_wxyz: (4,) — [w, x, y, z] quaternion from Isaac get_world_pose()
                                 or from ROS PoseStamped (reorder before calling!)

    Returns:
        (3,) array — point in Isaac world frame (ENU)
    """
    from scipy.spatial.transform import Rotation

    # Step 1: Camera → Body
    # R_BODY_TO_CAM is orthogonal, so inverse = transpose
    R_cam_to_body = R_BODY_TO_CAM.T
    p_body = R_cam_to_body @ np.asarray(p_cam) + CAM_OFFSET_BODY

    # Step 2: Body → World
    # scipy.Rotation.from_quat() expects [x, y, z, w] — reorder from wxyz
    w, x, y, z = drone_quat_wxyz
    R_body_to_world = Rotation.from_quat([x, y, z, w]).as_matrix()
    p_world = R_body_to_world @ p_body + np.asarray(drone_pos)

    return p_world


def nearest_buoy_error(p_world):
    """
    Find the nearest known buoy to a world-frame position estimate.

    Args:
        p_world: (3,) array — estimated position in Isaac world frame

    Returns:
        (name, error_m, gt_pos) — closest buoy name, Euclidean error in meters,
                                   and its ground truth position tuple
    """
    best_name, best_dist, best_pos = None, float('inf'), None
    pw = np.asarray(p_world)
    for name, pos in BUOY_POSITIONS.items():
        dist = np.linalg.norm(pw - np.array(pos))
        if dist < best_dist:
            best_name, best_dist, best_pos = name, dist, pos
    return best_name, best_dist, best_pos