"""
Seabird Camera Interface — Abstract Base Class

Two-layer design:
  Level 1 (Data Source): open, grab, get_rgb, get_depth, get_intrinsics
  Level 2 (Perception):  enable_detection, get_detections

Implementations:
  ZEDCamera  — wraps pyzed.sl for real hardware (ZED 2i / ZED X)
  SimCamera  — pulls from Isaac Sim (direct API now, ROS2 topics later)

Both return identical types so mission code never knows which backend is active.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import numpy as np


# ─── Detection Object ────────────────────────────────────────────────

@dataclass
class Detection:
    """Single detected object — the universal contract for mission code."""
    tracking_id: int              # persistent ID across frames (-1 if no tracker)
    label: str                    # class name: "red_buoy", "green_buoy", "blue_buoy"
    confidence: float             # 0.0 – 1.0
    bbox_2d: Tuple[int,int,int,int]  # (x_min, y_min, x_max, y_max) in pixels
    position_3d: Optional[Tuple[float,float,float]] = None  # (x, y, z) meters, camera frame
    velocity_3d: Optional[Tuple[float,float,float]] = None  # (vx, vy, vz) m/s, camera frame


# ─── Camera Intrinsics ────────────────────────────────────────────────

@dataclass
class Intrinsics:
    """Pinhole camera intrinsics — same format whether ZED or Isaac provides them."""
    fx: float        # focal length x (pixels)
    fy: float        # focal length y (pixels)
    cx: float        # principal point x (pixels)
    cy: float        # principal point y (pixels)
    width: int       # image width
    height: int      # image height


# ─── Camera Config ────────────────────────────────────────────────────

@dataclass
class CameraConfig:
    """Configuration applied before opening the camera."""
    width: int = 640
    height: int = 480
    fps: int = 30
    depth_enabled: bool = True
    depth_mode: str = "performance"   # "performance", "quality", "neural" (ZED-specific)


# ─── Abstract Base Class ──────────────────────────────────────────────

class CameraInterface(ABC):
    """
    Universal camera contract for Seabird.

    Lifecycle:
        1. configure()   — set resolution, fps, depth mode
        2. open()        — start the camera / connect to sim
        3. grab()        — advance to next frame (call in loop)
        4. get_rgb()     — retrieve current RGB frame
           get_depth()   — retrieve current depth map
           get_intrinsics() — camera parameters (stable after open)
        5. enable_detection() — turn on object detection (optional)
           get_detections()  — retrieve current frame's detections
        6. close()       — release resources
    """

    # ── Lifecycle ──

    @abstractmethod
    def configure(self, config: CameraConfig) -> None:
        """Apply camera settings. Must be called before open()."""
        ...

    @abstractmethod
    def open(self) -> bool:
        """Open camera stream. Returns True on success."""
        ...

    @abstractmethod
    def close(self) -> None:
        """Release camera resources."""
        ...

    @abstractmethod
    def grab(self) -> bool:
        """
        Capture next frame. Returns True if a fresh frame is available.
        Must be called before get_rgb() / get_depth() / get_detections().
        """
        ...

    # ── Level 1: Data Source ──

    @abstractmethod
    def get_rgb(self) -> Optional[np.ndarray]:
        """
        Current RGB frame as numpy array, shape (H, W, 3), dtype uint8, BGR.
        Returns None if no frame available (grab() not called or failed).
        """
        ...

    @abstractmethod
    def get_depth(self) -> Optional[np.ndarray]:
        """
        Current depth map as numpy array, shape (H, W), dtype float32, meters.
        Returns None if depth not available.
        """
        ...

    @abstractmethod
    def get_intrinsics(self) -> Optional[Intrinsics]:
        """
        Camera intrinsics. Stable after open() — safe to cache.
        Returns None if camera not yet opened.
        """
        ...

    # ── Level 2: Perception ──

    @abstractmethod
    def enable_detection(self, model_path: str, class_names: List[str],
                         enable_tracking: bool = True) -> bool:
        """
        Enable object detection.
        
        Args:
            model_path:      Path to ONNX model (ZED native) or .pt weights (sim/PyTorch)
            class_names:     Ordered class list, e.g. ["red_buoy", "green_buoy", "blue_buoy"]
            enable_tracking: Persistent IDs across frames
            
        Returns True if detection engine initialized successfully.
        """
        ...

    @abstractmethod
    def get_detections(self) -> List[Detection]:
        """
        Detections from the current frame. Returns empty list if:
        - detection not enabled
        - grab() not called
        - no objects found
        """
        ...

    # ── Convenience ──

    def is_open(self) -> bool:
        """Override in subclass if needed. Default: False."""
        return False

    def back_project(self, u: int, v: int, depth_m: float) -> Optional[Tuple[float,float,float]]:
        """
        Project a pixel (u, v) with known depth back to 3D camera-frame coords.
        Uses pinhole model: x = (u - cx) * z / fx, y = (v - cy) * z / fy
        
        Provided as a default implementation — subclasses can override
        if the SDK provides a more accurate method (e.g., ZED point cloud).
        """
        intr = self.get_intrinsics()
        if intr is None or depth_m <= 0:
            return None
        x = (u - intr.cx) * depth_m / intr.fx
        y = (v - intr.cy) * depth_m / intr.fy
        z = depth_m
        return (x, y, z)