"""
TFLiteHexagonDetector — Hexagon-NPU inference backend for Seabird.

Drop-in alternative to YoloDetector (see yolo_detector.py) for use on
ModalAI VOXL2 boards (e.g. Starling2 Max, Qualcomm QRB5165), which expose a
Hexagon DSP/NPU that Ultralytics' native .pt inference never touches. Same
public interface as YoloDetector — start() / detect() -> List[Detection] —
so BeaconCamera.enable_detection() can pick either one via config. Persistent
tracking_ids come from a greedy-IoU tracker here (see _track()) rather than from
ultralytics' .track(persist=True).

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
        class_names: Optional[List[str]] = None,
        imgsz: int = 640,
        conf_thresh: float = 0.5,
        iou_thresh: float = 0.45,
        delegate_path: Optional[str] = None,
        num_threads: int = 4,
        track_iou_thresh: float = 0.3,
        track_max_age: int = 30,
        input_is_bgr: bool = True,
    ):
        self.weights = weights
        self.class_names = class_names
        self.imgsz = imgsz
        self.conf_thresh = conf_thresh
        self.iou_thresh = iou_thresh
        self.delegate_path = delegate_path
        self.num_threads = num_threads
        self.track_iou_thresh = track_iou_thresh
        self.track_max_age = track_max_age
        self.input_is_bgr = input_is_bgr

        self._tracking = False
        self._tracks: List[dict] = []   # {"id", "bbox", "misses"}
        self._next_track_id = 0
        self._interpreter = None
        self._input_detail = None
        self._output_detail = None
        self._using_delegate = False

    # ── Lifecycle ──

    def start(self, enable_tracking: bool = True) -> bool:
        """
        Load the .tflite model, attempting the Hexagon delegate first if a
        path was given. `enable_tracking` turns on the greedy-IoU tracker in
        _track() — this backend has no ultralytics tracker behind it, so that
        is what supplies persistent tracking_ids. With it off, every Detection
        reports tracking_id=-1.
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

            # imgsz comes from JSON config, but the model's input size is fixed
            # at export. If they disagree, set_tensor() throws -- and worse,
            # _decode() scales normalised boxes by imgsz, so a silent mismatch
            # would misplace every box. Trust the model.
            in_shape = self._input_detail["shape"]
            if len(in_shape) == 4 and int(in_shape[1]) > 0:
                model_sz = int(in_shape[1])
                if int(in_shape[1]) != int(in_shape[2]):
                    print(f"[tflite-hexagon] WARNING: non-square model input "
                          f"{in_shape[1]}x{in_shape[2]}; _letterbox assumes square.")
                if model_sz != self.imgsz:
                    print(f"[tflite-hexagon] imgsz {self.imgsz} != model input "
                          f"{model_sz}; using {model_sz}.")
                    self.imgsz = model_sz

            # A .tflite carries no class names, but a YOLOv8-style output tensor
            # is [1, 4+num_classes, num_anchors], so the COUNT is recoverable.
            # Reconcile it with what the caller supplied: a short list would make
            # detect() discard every detection of the missing classes.
            self._reconcile_class_names()

            self._tracking = enable_tracking

            # Warm up — first inference pays for delegate/graph setup
            dummy = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
            self.detect(dummy)
            self._reset_tracks()  # discard anything the dummy frame invented

            print(f"[tflite-hexagon] Model loaded: {self.weights}")
            print(f"[tflite-hexagon] Classes: {self.class_names}")
            print(f"[tflite-hexagon] Hexagon delegate active: {self._using_delegate}")
            return True
        except Exception as e:
            print(f"[tflite-hexagon] ERROR loading model: {e}")
            self._interpreter = None
            return False

    def _reconcile_class_names(self) -> None:
        """Derive num_classes from the output tensor and make self.class_names
        match it. Names can't be recovered from the model, so any missing ones
        become positional placeholders — wrong labels are still far better than
        silently dropped detections."""
        shape = self._output_detail["shape"]
        if len(shape) != 3 or shape[1] >= shape[2]:
            return  # not the [1, 4+nc, anchors] layout _decode() expects
        num_classes = int(shape[1]) - 4
        if num_classes < 1:
            return

        if not self.class_names:
            self.class_names = [f"class_{i}" for i in range(num_classes)]
        elif len(self.class_names) < num_classes:
            print(f"[tflite-hexagon] WARNING: config lists {len(self.class_names)} "
                  f"class(es) but {self.weights} outputs {num_classes}. Detections "
                  f"of the extra classes would be discarded -- padding the list.")
            self.class_names = list(self.class_names) + [
                f"class_{i}" for i in range(len(self.class_names), num_classes)
            ]

    # ── Core ──

    def detect(
        self,
        rgb: np.ndarray,
        depth: Optional[np.ndarray] = None,
        intrinsics: Optional[Intrinsics] = None,
    ) -> List[Detection]:
        """
        Run inference on a single RGB frame.

        tracking_id is assigned by _track() when tracking is enabled, else
        -1. position_3d is filled only if both depth and intrinsics are
        provided, same convention as YoloDetector.detect().
        """
        if self._interpreter is None:
            return []

        # The pipeline hands us BGR (beacon_camera.py:294 uses cv_bridge
        # desired_encoding='bgr8'; camera_interface.get_rgb() documents BGR).
        # YoloDetector gets away with it because ultralytics converts BGR->RGB
        # internally; this backend must do it explicitly or the model sees R and
        # B swapped. voxl-tflite-server feeds RGB too (CV_YUV2RGB_*,
        # model_helper.cpp:237). Measured on one real frame: the same beacon
        # scores 0.494 as BGR vs 0.637 as RGB -- either side of conf 0.5.
        frame = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB) if self.input_is_bgr else rgb

        letterboxed, scale, pad_x, pad_y = self._letterbox(frame, self.imgsz)
        input_tensor = self._prepare_input(letterboxed)

        self._interpreter.set_tensor(self._input_detail["index"], input_tensor)
        self._interpreter.invoke()
        raw = self._interpreter.get_tensor(self._output_detail["index"])

        h, w = rgb.shape[:2]
        boxes, scores, cls_ids = self._decode(raw, scale, pad_x, pad_y, w, h)
        if len(boxes) == 0:
            return self._track([])

        # NOTE: class-AGNOSTIC NMS, unlike ultralytics' per-class default. That
        # is deliberate here: one physical beacon can score on several of the
        # four colour classes at once, and per-class NMS would emit a duplicate
        # detection per class -- each then paying a full stage-2 pass. Colour is
        # decided downstream by classify_beacon_color() anyway.
        nms_rects = [[int(b[0]), int(b[1]), int(b[2] - b[0]), int(b[3] - b[1])] for b in boxes]
        keep = cv2.dnn.NMSBoxes(nms_rects, scores.tolist(), self.conf_thresh, self.iou_thresh)
        if len(keep) == 0:
            return self._track([])
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
                tracking_id=-1,   # replaced by _track() below when tracking
                label=label,
                confidence=float(scores[i]),
                bbox_2d=(x1, y1, x2, y2),
                position_3d=pos_3d,
                velocity_3d=None,
            ))

        return self._track(detections)

    # ── Tracking ──

    def _reset_tracks(self) -> None:
        """Drop all track state (used after the warm-up frame)."""
        self._tracks = []
        self._next_track_id = 0

    @staticmethod
    def _iou(a: Tuple[int, int, int, int], b: Tuple[int, int, int, int]) -> float:
        """IoU of two (x1, y1, x2, y2) boxes."""
        ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
        ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
        iw, ih = ix2 - ix1, iy2 - iy1
        if iw <= 0 or ih <= 0:
            return 0.0
        inter = float(iw * ih)
        area_a = float(max(a[2] - a[0], 0) * max(a[3] - a[1], 0))
        area_b = float(max(b[2] - b[0], 0) * max(b[3] - b[1], 0))
        union = area_a + area_b - inter
        return inter / union if union > 0 else 0.0

    def _track(self, detections: List[Detection]) -> List[Detection]:
        """
        Assign persistent tracking_ids by greedy IoU matching against the
        previous frame's boxes. Stands in for the ByteTrack/BoT-SORT that
        YoloDetector gets free from ultralytics' .track(persist=True);
        without it every Detection carries the same id and the per-target
        state keyed on it downstream (blink_detector._get_blink_detector,
        _tracker_colors) collapses all beacons into one.

        Matching deliberately IGNORES the class label: stage 1 flips a given
        beacon between unknown/red/green/blue between frames, and colour is
        decided downstream by classify_beacon_color() anyway, so gating on it
        would fragment one beacon into several tracks.

        Returns `detections` with tracking_id populated (mutated in place).
        """
        if not self._tracking:
            return detections

        n_existing = len(self._tracks)

        # Score every (detection, existing track) pair, then take them
        # best-first — each detection and each track used at most once.
        pairs = []
        for di, det in enumerate(detections):
            for ti in range(n_existing):
                iou = self._iou(det.bbox_2d, self._tracks[ti]["bbox"])
                if iou >= self.track_iou_thresh:
                    pairs.append((iou, di, ti))
        pairs.sort(key=lambda p: p[0], reverse=True)

        det_to_track = {}
        claimed_tracks = set()
        for _iou, di, ti in pairs:
            if di in det_to_track or ti in claimed_tracks:
                continue
            det_to_track[di] = ti
            claimed_tracks.add(ti)

        for di, det in enumerate(detections):
            if di in det_to_track:
                track = self._tracks[det_to_track[di]]
                track["bbox"] = det.bbox_2d
                track["misses"] = 0
                det.tracking_id = track["id"]
            else:
                det.tracking_id = self._next_track_id
                self._tracks.append({"id": self._next_track_id,
                                     "bbox": det.bbox_2d, "misses": 0})
                self._next_track_id += 1

        # Age only tracks that existed coming into this frame. A blinking
        # beacon is undetectable for its whole off-phase, so track_max_age
        # must exceed that gap or the beacon returns as a NEW id and its
        # blink history restarts -- defeating the measurement it feeds.
        for ti in range(n_existing):
            if ti not in claimed_tracks:
                self._tracks[ti]["misses"] += 1
        self._tracks = [t for i, t in enumerate(self._tracks)
                        if i >= n_existing or t["misses"] <= self.track_max_age]

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

        # Ultralytics TFLite exports emit NORMALIZED xywh in [0,1] relative to
        # the model's square input, NOT pixels in the letterboxed frame. Lift
        # them into letterbox pixel space before undoing the letterbox, or every
        # box collapses to (0,0,0,0). voxl-tflite-server does the equivalent at
        # yolov8_model_helper.cpp:106-110 (it multiplies by the camera frame
        # dims because it stretch-resizes instead of letterboxing).
        cx = box_xywh[:, 0] * self.imgsz
        cy = box_xywh[:, 1] * self.imgsz
        bw = box_xywh[:, 2] * self.imgsz
        bh = box_xywh[:, 3] * self.imgsz
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
