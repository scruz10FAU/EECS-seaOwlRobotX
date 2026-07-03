#!/usr/bin/env python3
"""
data_recorder.py — Phase 1 data collection for Seabird buoy detection

Subscribes to:
  /iris_0/front_cam/rgb          (sensor_msgs/Image)
  /iris_0/front_cam/depth        (sensor_msgs/Image, 32FC1)
  /seabird/buoy_detections       (std_msgs/String, JSON with bbox)
  /drone_0/state                 (geometry_msgs/PoseStamped — for metadata)

Saves to ~/drone_sim/workspace/dataset/<run_NNN>/
  images/      rgb_XXXXXX.png
  depth/       depth_XXXXXX.npy
  labels/      rgb_XXXXXX.txt    (YOLO format: class x_center y_center w h)
  meta/        rgb_XXXXXX.json   (drone pose, timestamp, detection details)
  dataset.yaml                   (YOLO training config — written on shutdown)

Usage:
  python3 data_recorder.py                       # auto-increments run number
  python3 data_recorder.py --run 5               # force run_005
  python3 data_recorder.py --save-every 3        # save every 3rd synced frame (reduce disk)

Expects buoy_detector.py JSON to include:
  {"color": "red", "bbox": [x, y, w, h], "cx": 160, "cy": 120, "area": 450}
  bbox = [x_top_left, y_top_left, width, height] in pixels
"""

import argparse
import json
import os
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

try:
    import message_filters
except ImportError:
    print("[data_recorder] FATAL: pip install ros2-message-filters")
    raise

try:
    from cv_bridge import CvBridge
except ImportError:
    print("[data_recorder] FATAL: sudo apt install ros-humble-cv-bridge")
    raise

# ─── Config ───────────────────────────────────────────────────
CLASS_MAP = {"red": 0, "green": 1, "blue": 2}
IMG_W, IMG_H = 320, 240  # must match ROS2CameraGraph resolution
DATASET_ROOT = Path.home() / "drone_sim" / "workspace" / "dataset"
DET_MATCH_WINDOW = 0.15  # seconds — how close detection must be to frame


class DataRecorder(Node):
    def __init__(self, run_id: int, save_every: int = 1):
        super().__init__("data_recorder")

        self.bridge = CvBridge()
        self.frame_count = 0
        self.saved_count = 0
        self.save_every = save_every
        self.det_buffer: list[dict] = []
        self.latest_pose: dict | None = None

        # ── Directory setup ──
        self.run_dir = DATASET_ROOT / f"run_{run_id:03d}"
        self.img_dir = self.run_dir / "images"
        self.depth_dir = self.run_dir / "depth"
        self.label_dir = self.run_dir / "labels"
        self.meta_dir = self.run_dir / "meta"
        for d in [self.img_dir, self.depth_dir, self.label_dir, self.meta_dir]:
            d.mkdir(parents=True, exist_ok=True)

        # ── Synced RGB + depth ──
        rgb_sub = message_filters.Subscriber(self, Image, "/iris_0/front_cam/rgb")
        depth_sub = message_filters.Subscriber(self, Image, "/iris_0/front_cam/depth")
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.05
        )
        self.sync.registerCallback(self._frame_cb)

        # ── Detections (async — buffered) ──
        self.create_subscription(
            String, "/seabird/buoy_detections", self._det_cb, 10
        )

        # ── Drone state (async — latest only) ──
        self.create_subscription(
            PoseStamped, "/drone_0/state", self._pose_cb, 10
        )

        self.get_logger().info(
            f"Recording run {run_id:03d} → {self.run_dir}  (save_every={save_every})"
        )

    # ── Callbacks ─────────────────────────────────────────────

    def _det_cb(self, msg: String):
        try:
            det = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        det["_t"] = time.time()
        self.det_buffer.append(det)
        # Prune older than 2s
        cutoff = time.time() - 2.0
        self.det_buffer = [d for d in self.det_buffer if d["_t"] > cutoff]

    def _pose_cb(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation
        self.latest_pose = {
            "pos": [p.x, p.y, p.z],
            "quat": [q.x, q.y, q.z, q.w],
        }

    def _frame_cb(self, rgb_msg: Image, depth_msg: Image):
        self.frame_count += 1
        if self.frame_count % self.save_every != 0:
            return

        # ── Decode ──
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"RGB decode failed: {e}")
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
        except Exception as e:
            self.get_logger().warn(f"Depth decode failed: {e}")
            return

        fid = f"{self.saved_count:06d}"
        now = time.time()

        # ── Save RGB ──
        cv2.imwrite(str(self.img_dir / f"rgb_{fid}.png"), rgb)

        # ── Save depth (float32 numpy) ──
        np.save(str(self.depth_dir / f"depth_{fid}.npy"), depth)

        # ── Match detections within time window ──
        recent_dets = [
            d for d in self.det_buffer if abs(now - d["_t"]) < DET_MATCH_WINDOW
        ]

        label_lines = []
        det_details = []
        for det in recent_dets:
            color = det.get("color", "")
            bbox = det.get("bbox")  # [x, y, w, h] pixels
            if color not in CLASS_MAP:
                continue
            if not bbox or len(bbox) != 4:
                continue

            bx, by, bw, bh = bbox
            if bw <= 0 or bh <= 0:
                continue

            cls_id = CLASS_MAP[color]

            # YOLO normalized format: class x_center y_center width height
            xc = min(max((bx + bw / 2.0) / IMG_W, 0.0), 1.0)
            yc = min(max((by + bh / 2.0) / IMG_H, 0.0), 1.0)
            wn = min(max(bw / IMG_W, 0.0), 1.0)
            hn = min(max(bh / IMG_H, 0.0), 1.0)

            label_lines.append(f"{cls_id} {xc:.6f} {yc:.6f} {wn:.6f} {hn:.6f}")
            det_details.append({
                "color": color,
                "class_id": cls_id,
                "bbox_px": bbox,
                "yolo": [cls_id, round(xc, 6), round(yc, 6), round(wn, 6), round(hn, 6)],
                "area": det.get("area", 0),
            })

        # ── Save YOLO label (empty file = negative sample — good for training) ──
        with open(self.label_dir / f"rgb_{fid}.txt", "w") as f:
            f.write("\n".join(label_lines))

        # ── Save metadata ──
        meta = {
            "frame_id": fid,
            "timestamp": now,
            "drone_pose": self.latest_pose,
            "detections": det_details,
            "n_detections": len(det_details),
            "img_shape": [IMG_H, IMG_W],
        }
        with open(self.meta_dir / f"rgb_{fid}.json", "w") as f:
            json.dump(meta, f, indent=2)

        self.saved_count += 1

        # Progress log
        if self.saved_count % 50 == 0:
            n_pos = sum(
                1
                for p in self.label_dir.iterdir()
                if p.stat().st_size > 0
            )
            self.get_logger().info(
                f"Saved: {self.saved_count} | With detections: {n_pos} / {self.saved_count}"
            )

    # ── Write YOLO dataset.yaml on shutdown ──

    def write_dataset_yaml(self):
        yaml_path = self.run_dir / "dataset.yaml"
        content = (
            f"# Auto-generated by data_recorder.py\n"
            f"path: {self.run_dir}\n"
            f"train: images\n"
            f"val: images\n"
            f"\n"
            f"names:\n"
            f"  0: red_buoy\n"
            f"  1: green_buoy\n"
            f"  2: blue_buoy\n"
            f"\n"
            f"# Total frames: {self.saved_count}\n"
        )
        with open(yaml_path, "w") as f:
            f.write(content)
        self.get_logger().info(f"Wrote {yaml_path}")


# ─── Helpers ──────────────────────────────────────────────────

def find_next_run_id() -> int:
    """Auto-increment run number."""
    DATASET_ROOT.mkdir(parents=True, exist_ok=True)
    existing = [
        int(d.name.split("_")[1])
        for d in DATASET_ROOT.iterdir()
        if d.is_dir() and d.name.startswith("run_")
    ]
    return max(existing, default=-1) + 1


def main():
    parser = argparse.ArgumentParser(description="Seabird data recorder")
    parser.add_argument("--run", type=int, default=None, help="Force run ID")
    parser.add_argument(
        "--save-every",
        type=int,
        default=1,
        help="Save every Nth synced frame (default: 1 = all)",
    )
    args = parser.parse_args()

    run_id = args.run if args.run is not None else find_next_run_id()

    rclpy.init()
    node = DataRecorder(run_id=run_id, save_every=args.save_every)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_dataset_yaml()
        node.get_logger().info(
            f"Done. {node.saved_count} frames saved to {node.run_dir}"
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()