"""
YoloDetector — Standalone perception module for Seabird.

Takes RGB frames, runs YOLOv8n inference, returns List[Detection].
Optionally fuses depth + intrinsics for camera-frame 3D position.

This module knows NOTHING about:
  - Which camera backend is running (sim vs ZED)
  - Where the drone is (no pose, no world frame)
  - ROS2, MAVSDK, or any middleware

It's pure: image in → detections out.

Usage:
    from yolo_detector import YoloDetector
    det = YoloDetector(weights="path/to/best.pt", class_names=["red_buoy", ...])
    det.start(enable_tracking=True)
    detections = det.detect(rgb_frame, depth_map, intrinsics)
"""

import sys
# Only insert 3.11 site-packages when running inside Isaac's Python (3.11).
# When running as a ROS2 node under system Python 3.10, ultralytics must
# be installed separately: python3 -m pip install --user ultralytics
if sys.version_info[:2] == (3, 11):
    sys.path.insert(0, os.path.expanduser("~/.local/lib/python3.11/site-packages"))

from typing import List, Optional, Tuple
import numpy as np

from camera_interface import Detection, Intrinsics

# ultralytics import deferred to start() so the module can be imported
# without GPU overhead until actually needed
_YOLO = None


def _lazy_import_yolo():
    """Import ultralytics on first use — avoids import-time GPU init."""
    global _YOLO
    if _YOLO is None:
        from ultralytics import YOLO as _Y
        _YOLO = _Y
    return _YOLO


class YoloDetector:
    """
    Stateless-ish YOLO detector that maps ultralytics Results → Detection.

    Args:
        weights:      Path to .pt weights file
        class_names:  Ordered list matching training class indices
                      e.g. ["red_buoy", "green_buoy", "blue_buoy"]
        imgsz:        Inference resolution (should match training — 320)
        conf_thresh:  Minimum confidence to keep a detection
        device:       "cuda:0", "cpu", or None (auto)
    """

    def __init__(
        self,
        weights: str,
        class_names: List[str],
        imgsz: int = 320,
        conf_thresh: float = 0.5,
        device: Optional[str] = None,
    ):
        self.weights = weights
        self.class_names = class_names
        self.imgsz = imgsz
        self.conf_thresh = conf_thresh
        self.device = device

        self._model = None
        self._tracking = False

    # ── Lifecycle ──

    def start(self, enable_tracking: bool = True) -> bool:
        """
        Load the YOLO model onto GPU. Call once before detect().

        Args:
            enable_tracking: Use ultralytics built-in tracker (ByteTrack)
                             for persistent IDs across frames.
        Returns:
            True if model loaded successfully.
        """
        try:
            YOLO = _lazy_import_yolo()
            self._model = YOLO(self.weights)
            self._tracking = enable_tracking
            # Warm up — first inference is slow due to TensorRT/CUDA setup
            dummy = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
            self._model.predict(
                dummy, imgsz=self.imgsz, conf=self.conf_thresh,
                verbose=False, device=self.device,
            )
            print(f"[yolo] Model loaded: {self.weights}")
            print(f"[yolo] Classes: {self.class_names}")
            print(f"[yolo] Tracking: {self._tracking}")
            return True
        except Exception as e:
            print(f"[yolo] ERROR loading model: {e}")
            self._model = None
            return False

    # ── Core ──

    def detect(
        self,
        rgb: np.ndarray,
        depth: Optional[np.ndarray] = None,
        intrinsics: Optional[Intrinsics] = None,
    ) -> List[Detection]:
        """
        Run inference on a single RGB frame.

        Args:
            rgb:        (H, W, 3) uint8 BGR image
            depth:      (H, W) float32 depth in meters (optional)
            intrinsics: Camera intrinsics for back-projection (optional)

        Returns:
            List of Detection objects. position_3d is filled ONLY if
            both depth and intrinsics are provided — and it's in
            CAMERA FRAME, not world frame.
        """
        if self._model is None:
            return []

        # Run inference — track() for persistent IDs, predict() otherwise
        if self._tracking:
            results = self._model.track(
                rgb, imgsz=self.imgsz, conf=self.conf_thresh,
                verbose=False, persist=True, device=self.device,
            )
        else:
            results = self._model.predict(
                rgb, imgsz=self.imgsz, conf=self.conf_thresh,
                verbose=False, device=self.device,
            )

        # ultralytics returns a list (one per image in batch) — we send one
        if not results or len(results) == 0:
            return []

        return self._parse_results(results[0], depth, intrinsics)

    # ── Internals ──

    def _parse_results(
        self,
        result,
        depth: Optional[np.ndarray],
        intrinsics: Optional[Intrinsics],
    ) -> List[Detection]:
        """
        Convert ultralytics Result → List[Detection].

        ultralytics box formats:
          .xyxy  — (N, 4) tensor, pixel coords in ORIGINAL image space
          .conf  — (N,) confidence scores
          .cls   — (N,) class indices (float → int)
          .id    — (N,) tracking IDs if tracking enabled, else None
        """
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return []

        detections: List[Detection] = []

        # Pull tensors to numpy once
        xyxy = boxes.xyxy.cpu().numpy()       # (N, 4)
        confs = boxes.conf.cpu().numpy()       # (N,)
        cls_ids = boxes.cls.cpu().numpy().astype(int)  # (N,)

        # Tracking IDs — None if predict() was used instead of track()
        track_ids = None
        if boxes.id is not None:
            track_ids = boxes.id.cpu().numpy().astype(int)

        for i in range(len(xyxy)):
            x1, y1, x2, y2 = xyxy[i].astype(int)
            cls_idx = cls_ids[i]
            conf = float(confs[i])

            # Map class index → label
            if cls_idx < 0 or cls_idx >= len(self.class_names):
                continue  # unknown class — skip
            label = self.class_names[cls_idx]

            # Tracking ID: from tracker, or -1 if no tracker
            tid = int(track_ids[i]) if track_ids is not None else -1

            # 3D position via depth back-projection (camera frame)
            pos_3d = None
            if depth is not None and intrinsics is not None:
                pos_3d = self._back_project_bbox_center(
                    x1, y1, x2, y2, depth, intrinsics
                )

            detections.append(Detection(
                tracking_id=tid,
                label=label,
                confidence=conf,
                bbox_2d=(x1, y1, x2, y2),
                position_3d=pos_3d,
                velocity_3d=None,  # no velocity estimation yet
            ))

        return detections

    def _back_project_bbox_center(
        self,
        x1: int, y1: int, x2: int, y2: int,
        depth: np.ndarray,
        intrinsics: Intrinsics,
    ) -> Optional[Tuple[float, float, float]]:
        """
        Back-project the center of a 2D bbox to 3D camera-frame coords.

        Strategy: sample a small patch around bbox center in the depth map,
        take the median (robust to noisy/missing pixels), then pinhole
        back-project.

        Returns None if depth is invalid (NaN, inf, zero, negative).
        """
        cx_px = (x1 + x2) // 2
        cy_px = (y1 + y2) // 2

        h, w = depth.shape[:2]

        # Sample a 5x5 patch centered on bbox center (clipped to image)
        patch_r = 2
        py0 = max(0, cy_px - patch_r)
        py1 = min(h, cy_px + patch_r + 1)
        px0 = max(0, cx_px - patch_r)
        px1 = min(w, cx_px + patch_r + 1)

        patch = depth[py0:py1, px0:px1]

        # Filter out invalid depth values
        valid = patch[np.isfinite(patch) & (patch > 0.0)]
        if len(valid) == 0:
            return None

        z = float(np.median(valid))

        # Pinhole back-projection: pixel → camera frame
        x = (cx_px - intrinsics.cx) * z / intrinsics.fx
        y = (cy_px - intrinsics.cy) * z / intrinsics.fy

        return (x, y, z)
