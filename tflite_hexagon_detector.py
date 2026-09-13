"""
TFLiteHexagonDetector — Hexagon-NPU inference backend for Seabird.

Drop-in alternative to YoloDetector (see yolo_detector.py) for use on
ModalAI VOXL2 boards (e.g. Starling2 Max, Qualcomm QRB5165), which expose a
Hexagon DSP/NPU that Ultralytics' native .pt inference never touches. Same
public interface as YoloDetector — start() / detect() -> List[Detection] —
so BeaconCamera.enable_detection() can pick either one via config.

Requires an int8-quantized .tflite export of the model (see
export_tflite.py) and, to actually use the NPU, a VOXL2-SDK-provided
Hexagon delegate shared library. That delegate's path is board/SDK-version
specific and must be supplied via `delegate_path`; if it's absent or fails
to load, inference falls back to the interpreter's default CPU backend.

Usage:
    from tflite_hexagon_detector import TFLiteHexagonDetector
    det = TFLiteHexagonDetector(weights="path/to/model_int8.tflite",
                                 class_names=["beacon"],
                                 delegate_path="/usr/lib/libhexagon_delegate.so")
    det.start()
    detections = det.detect(rgb_frame, depth_map, intrinsics)
"""

from typing import List, Optional, Tuple

import cv2
import numpy as np

from camera_interface import Detection, Intrinsics

_tflite = None


def _lazy_import_tflite():
    """Import a TFLite interpreter on first use. Tries tflite-runtime first
    (the lightweight package VOXL2/ModalAI images typically ship), then
    falls back to the interpreter bundled with full tensorflow."""
    global _tflite
    if _tflite is None:
        try:
            import tflite_runtime.interpreter as _t
        except ImportError:
            import tensorflow.lite as _t
        _tflite = _t
    return _tflite


class TFLiteHexagonDetector:
    """
    Image in -> List[Detection] out, via a TFLite interpreter (optionally
    accelerated by the Hexagon delegate). Mirrors YoloDetector's interface
    so the two are interchangeable from BeaconCamera.enable_detection().

    Args:
        weights:       Path to an int8-quantized .tflite model (see export_tflite.py)
        class_names:   Ordered list matching training class indices
        imgsz:         Model input size (square) — must match the export
        conf_thresh:   Minimum confidence to keep a detection
        iou_thresh:    IoU threshold for NMS
        delegate_path: Path to the Hexagon delegate .so (VOXL2-SDK-specific).
                       None, missing, or a failed load falls back to CPU.
        num_threads:   CPU thread count when not using the delegate.
    """

    def __init__(
        self,
        weights: str,
        class_names: List[str],
        imgsz: int = 640,
        conf_thresh: float = 0.5,
        iou_thresh: float = 0.45,
        delegate_path: Optional[str] = None,
        num_threads: int = 4,
    ):
        self.weights = weights
        self.class_names = class_names
        self.imgsz = imgsz
        self.conf_thresh = conf_thresh
        self.iou_thresh = iou_thresh
        self.delegate_path = delegate_path
        self.num_threads = num_threads

        self._interpreter = None
        self._input_detail = None
        self._output_detail = None
        self._using_delegate = False

    # ── Lifecycle ──

    def start(self, enable_tracking: bool = True) -> bool:
        """
        Load the .tflite model, attempting the Hexagon delegate first if a
        path was given. `enable_tracking` is accepted for interface parity
        with YoloDetector but has no effect — this backend always reports
        tracking_id=-1 (see detect()); there is no ByteTrack-equivalent for
        raw TFLite output here.
        """
        try:
            tflite = _lazy_import_tflite()

            delegates = []
            if self.delegate_path:
                try:
                    delegates = [tflite.load_delegate(self.delegate_path)]
                    self._using_delegate = True
                except Exception as e:
                    print(f"[tflite-hexagon] WARNING: failed to load delegate "
                          f"'{self.delegate_path}' ({e}) -- falling back to CPU")
                    delegates = []
                    self._using_delegate = False

            self._interpreter = tflite.Interpreter(
                model_path=self.weights,
                experimental_delegates=delegates,
                num_threads=self.num_threads,
            )
            self._interpreter.allocate_tensors()
            self._input_detail = self._interpreter.get_input_details()[0]
            self._output_detail = self._interpreter.get_output_details()[0]

            # Warm up — first inference pays for delegate/graph setup
            dummy = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
            self.detect(dummy)

            print(f"[tflite-hexagon] Model loaded: {self.weights}")
            print(f"[tflite-hexagon] Classes: {self.class_names}")
            print(f"[tflite-hexagon] Hexagon delegate active: {self._using_delegate}")
            return True
        except Exception as e:
            print(f"[tflite-hexagon] ERROR loading model: {e}")
            self._interpreter = None
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

        tracking_id is always -1 (no persistent tracker in this backend).
        position_3d is filled only if both depth and intrinsics are
        provided, same convention as YoloDetector.detect().
        """
        if self._interpreter is None:
            return []

        letterboxed, scale, pad_x, pad_y = self._letterbox(rgb, self.imgsz)
        input_tensor = self._prepare_input(letterboxed)

        self._interpreter.set_tensor(self._input_detail["index"], input_tensor)
        self._interpreter.invoke()
        raw = self._interpreter.get_tensor(self._output_detail["index"])

        h, w = rgb.shape[:2]
        boxes, scores, cls_ids = self._decode(raw, scale, pad_x, pad_y, w, h)
        if len(boxes) == 0:
            return []

        nms_rects = [[int(b[0]), int(b[1]), int(b[2] - b[0]), int(b[3] - b[1])] for b in boxes]
        keep = cv2.dnn.NMSBoxes(nms_rects, scores.tolist(), self.conf_thresh, self.iou_thresh)
        if len(keep) == 0:
            return []
        keep = np.array(keep).reshape(-1)

        detections: List[Detection] = []
        for i in keep:
            x1, y1, x2, y2 = boxes[i].astype(int)
            cls_idx = int(cls_ids[i])
            if cls_idx < 0 or cls_idx >= len(self.class_names):
                continue  # unknown class — skip
            label = self.class_names[cls_idx]

            pos_3d = None
            if depth is not None and intrinsics is not None:
                pos_3d = self._back_project_bbox_center(x1, y1, x2, y2, depth, intrinsics)

            detections.append(Detection(
                tracking_id=-1,
                label=label,
                confidence=float(scores[i]),
                bbox_2d=(x1, y1, x2, y2),
                position_3d=pos_3d,
                velocity_3d=None,
            ))

        return detections

    # ── Internals ──

    @staticmethod
    def _letterbox(rgb: np.ndarray, size: int) -> Tuple[np.ndarray, float, int, int]:
        """
        Resize + pad to a square size x size canvas, preserving aspect ratio
        (same convention Ultralytics uses) — decoded boxes need the same
        scale/pad correction applied in reverse, see _decode().
        """
        h, w = rgb.shape[:2]
        scale = min(size / w, size / h)
        nw, nh = int(round(w * scale)), int(round(h * scale))
        resized = cv2.resize(rgb, (nw, nh), interpolation=cv2.INTER_LINEAR)
        pad_x, pad_y = (size - nw) // 2, (size - nh) // 2
        canvas = np.full((size, size, 3), 114, dtype=np.uint8)
        canvas[pad_y:pad_y + nh, pad_x:pad_x + nw] = resized
        return canvas, scale, pad_x, pad_y

    def _prepare_input(self, letterboxed: np.ndarray) -> np.ndarray:
        """Cast the letterboxed uint8 frame to whatever dtype the model's
        input tensor expects, applying its quantization scale/zero-point for
        an int8/uint8 input, or plain [0,1] normalization for float32."""
        dtype = self._input_detail["dtype"]
        batched = letterboxed[np.newaxis, ...]
        if dtype in (np.uint8, np.int8):
            scale, zero_point = self._input_detail["quantization"]
            if not scale:
                return batched.astype(dtype)
            quantized = (batched.astype(np.float32) / 255.0) / scale + zero_point
            return np.clip(quantized, np.iinfo(dtype).min, np.iinfo(dtype).max).astype(dtype)
        return batched.astype(np.float32) / 255.0

    def _decode(self, raw: np.ndarray, scale: float, pad_x: int, pad_y: int,
                orig_w: int, orig_h: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Decode a raw YOLOv8-style TFLite output tensor — [1, 4+num_classes,
        num_anchors], no built-in NMS — into (boxes_xyxy, scores, cls_ids)
        in ORIGINAL frame pixel coordinates. Confidence-thresholded but not
        yet NMS'd (NMS happens in detect(), which also builds Detections).
        """
        out_scale, out_zero = self._output_detail["quantization"]
        if out_scale:
            raw = (raw.astype(np.float32) - out_zero) * out_scale

        raw = raw[0].T  # (num_anchors, 4+num_classes)
        box_xywh = raw[:, :4]
        class_scores = raw[:, 4:]
        cls_ids = np.argmax(class_scores, axis=1)
        confs = class_scores[np.arange(len(cls_ids)), cls_ids]

        keep = confs >= self.conf_thresh
        if not np.any(keep):
            return np.zeros((0, 4)), np.zeros((0,)), np.zeros((0,), dtype=int)

        box_xywh = box_xywh[keep]
        confs = confs[keep]
        cls_ids = cls_ids[keep]

        cx, cy, bw, bh = box_xywh[:, 0], box_xywh[:, 1], box_xywh[:, 2], box_xywh[:, 3]
        x1 = (cx - bw / 2.0 - pad_x) / scale
        y1 = (cy - bh / 2.0 - pad_y) / scale
        x2 = (cx + bw / 2.0 - pad_x) / scale
        y2 = (cy + bh / 2.0 - pad_y) / scale

        x1 = np.clip(x1, 0, orig_w - 1)
        y1 = np.clip(y1, 0, orig_h - 1)
        x2 = np.clip(x2, 0, orig_w - 1)
        y2 = np.clip(y2, 0, orig_h - 1)

        return np.stack([x1, y1, x2, y2], axis=1), confs, cls_ids

    def _back_project_bbox_center(
        self,
        x1: int, y1: int, x2: int, y2: int,
        depth: np.ndarray,
        intrinsics: Intrinsics,
    ) -> Optional[Tuple[float, float, float]]:
        """
        Same patch-median pinhole back-projection as
        YoloDetector._back_project_bbox_center — duplicated locally rather
        than shared, to avoid touching the existing, working YoloDetector
        code path for this change.
        """
        cx_px = (x1 + x2) // 2
        cy_px = (y1 + y2) // 2

        h, w = depth.shape[:2]
        patch_r = 2
        py0 = max(0, cy_px - patch_r)
        py1 = min(h, cy_px + patch_r + 1)
        px0 = max(0, cx_px - patch_r)
        px1 = min(w, cx_px + patch_r + 1)

        patch = depth[py0:py1, px0:px1]
        valid = patch[np.isfinite(patch) & (patch > 0.0)]
        if len(valid) == 0:
            return None

        z = float(np.median(valid))
        x = (cx_px - intrinsics.cx) * z / intrinsics.fx
        y = (cy_px - intrinsics.cy) * z / intrinsics.fy
        return (x, y, z)
