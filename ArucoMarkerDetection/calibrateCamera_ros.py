#!/usr/bin/env python3
"""
calibrateCamera_ros.py — Camera calibration using the image topic from aruco_config.json.

Subscribes to the same image topic the ArUco detector uses, so calibration is done
against the exact camera feed that will be used for detection.

Instructions
------------
1. Print a 10×7 checkerboard and attach it to a flat board.
   Printable PDF: https://github.com/kyle-bersani/opencv-examples/blob/master/CalibrationByChessboard/chessboard-to-print.pdf
2. Measure one printed square in metres and pass it via --square-size (default 0.0254 m).
3. Run the script. Left-click the preview window when the detection overlay is visible
   to capture that frame. Aim for 15–20 images at varied angles and distances.
4. Press 'q' to finish capture and run calibration automatically.
5. Outputs camera_calibration.npz in this folder, ready for use by poseEstimator.py.

Usage
-----
  python3 calibrateCamera_ros.py                              # default config + ROS topic
  python3 calibrateCamera_ros.py --config configs/my.json    # custom config
  python3 calibrateCamera_ros.py --square-size 0.030         # 30 mm squares
  python3 calibrateCamera_ros.py --calibrate-only            # skip capture, re-run math
  python3 calibrateCamera_ros.py --video 0                   # use webcam instead of ROS
"""

import sys
import os
import argparse
import json
import threading
import glob

import numpy as np
import cv2

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

# ---------------------------------------------------------------------------
# Paths and defaults
# ---------------------------------------------------------------------------

_DEFAULT_CONFIG = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "configs", "aruco_config.json"
)
_IMAGES_DIR  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calib_images")
_OUTPUT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "camera_calibration.npz")

_DEFAULT_CHECKERBOARD = (9, 6)   # inner corners of a standard 10×7 board
_DEFAULT_SQUARE_SIZE  = 0.0254   # metres

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

def load_image_topic(config_path: str) -> str:
    """Read the image topic from the config file."""
    with open(config_path) as fh:
        raw = json.load(fh)
    topics = raw.get("topics", {})
    # Fall back to camera_prefix if image is not set explicitly
    return topics.get("image", topics.get("camera_prefix", "/hires_front_small_color"))

# ---------------------------------------------------------------------------
# Lightweight ROS2 image subscriber (no BeaconCamera / YOLO dependency)
# ---------------------------------------------------------------------------

rclpy = None


def _import_ros():
    global rclpy
    import rclpy as _rclpy
    rclpy = _rclpy


class _ImageSubscriber:
    """
    Minimal ROS2 node: subscribes to one image topic and exposes the latest frame.
    Does not import beacon_camera, seabird_config, or yolo_detector.
    """

    def __init__(self, image_topic: str):
        self._topic = image_topic
        self._frame = None
        self._lock  = threading.Lock()
        self._node  = None

    def start(self):
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge

        bridge = CvBridge()
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        outer = self

        class _Node(Node):
            def __init__(self):
                super().__init__("aruco_calibrator")
                self.create_subscription(Image, outer._topic, self._cb, qos)

            def _cb(self, msg):
                bgr = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                with outer._lock:
                    outer._frame = bgr

        self._node = _Node()
        print(f"Subscribed to: {self._topic}")

    def spin_once(self):
        if self._node:
            rclpy.spin_once(self._node, timeout_sec=0.05)

    def get_frame(self):
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def shutdown(self):
        if self._node:
            self._node.destroy_node()

# ---------------------------------------------------------------------------
# Capture loops (shared by ROS and webcam modes)
# ---------------------------------------------------------------------------

def _capture_loop(get_frame_fn, spin_fn, checkerboard, images_dir):
    """
    Display-based capture loop. Left-click to save; 'q' to finish.
    Requires a connected display (X11 / GUI environment).
    Returns number of images saved.
    """
    os.makedirs(images_dir, exist_ok=True)
    print("Left-click when the detection overlay is visible to capture.")
    print("Press 'q' to finish and run calibration.")

    count = 0
    capture_requested = False

    def on_mouse(event, x, y, flags, param):
        nonlocal capture_requested
        if event == cv2.EVENT_LBUTTONDOWN:
            capture_requested = True

    cv2.namedWindow("Calibration Capture")
    cv2.setMouseCallback("Calibration Capture", on_mouse)

    while True:
        spin_fn()
        frame = get_frame_fn()
        if frame is None:
            continue

        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, checkerboard, None)

        display = frame.copy()
        if found:
            cv2.drawChessboardCorners(display, checkerboard, corners, found)
        cv2.putText(
            display, f"Captured: {count}", (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
        )
        cv2.imshow("Calibration Capture", display)

        if capture_requested and found:
            path = os.path.join(images_dir, f"calib_{count:02d}.png")
            cv2.imwrite(path, frame)
            print(f"Saved {path}")
            count += 1
        capture_requested = False

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()
    print(f"\nCaptured {count} images. Aim for at least 15–20 before calibrating.")
    return count


def _capture_loop_keyboard(get_frame_fn, spin_fn, checkerboard, images_dir, target=20):
    """
    Keyboard-controlled headless capture. No display required.

    The ROS spin runs in a background thread so frames keep arriving while the
    main thread blocks on input(). Press Enter to attempt a capture; type 'q'
    and Enter to stop early.
    """
    import threading
    os.makedirs(images_dir, exist_ok=True)
    print(f"Keyboard mode: {target} images needed.")
    print("Position the checkerboard, then press Enter to capture. Type 'q' + Enter to stop.\n")

    stop_spin = threading.Event()

    def _spin_worker():
        while not stop_spin.is_set():
            spin_fn()

    spin_thread = threading.Thread(target=_spin_worker, daemon=True)
    spin_thread.start()

    count = 0
    try:
        while count < target:
            try:
                key = input(f"[{count:2d}/{target}] Enter to capture, 'q' to finish: ")
            except EOFError:
                break
            if key.strip().lower() == "q":
                break

            frame = get_frame_fn()
            if frame is None:
                print("  No frame received yet — is the camera publishing?")
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, _ = cv2.findChessboardCorners(gray, checkerboard, None)
            if found:
                path = os.path.join(images_dir, f"calib_{count:02d}.png")
                cv2.imwrite(path, frame)
                count += 1
                print(f"  Saved {path}")
            else:
                print("  No checkerboard detected — reposition and try again.")
    finally:
        stop_spin.set()
        spin_thread.join(timeout=1.0)

    print(f"\nCaptured {count} images.")
    return count


def _capture_loop_headless(get_frame_fn, spin_fn, checkerboard, images_dir,
                           target=20, cooldown=2.0):
    """
    Headless capture loop — no display, no mouse. Runs entirely in the terminal.

    Automatically saves a frame whenever a checkerboard is detected, subject to a
    cooldown between captures so each saved image comes from a distinct pose.
    Move the board to a new angle/distance between beeps (terminal bell).

    target   : stop after this many images (default 20)
    cooldown : minimum seconds between captures (default 2.0)
    """
    import time
    os.makedirs(images_dir, exist_ok=True)
    print(f"Headless mode: auto-capturing up to {target} images.")
    print(f"Hold the checkerboard still for {cooldown}s between poses. Ctrl-C to stop early.\n")

    count = 0
    last_capture = 0.0

    try:
        while count < target:
            spin_fn()
            frame = get_frame_fn()
            if frame is None:
                continue

            now = time.monotonic()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, _ = cv2.findChessboardCorners(gray, checkerboard, None)

            if found:
                if now - last_capture >= cooldown:
                    path = os.path.join(images_dir, f"calib_{count:02d}.png")
                    cv2.imwrite(path, frame)
                    last_capture = now
                    count += 1
                    print(f"  [{count:2d}/{target}] Saved {path}")
                    print("\a", end="", flush=True)  # terminal bell
                else:
                    remaining = cooldown - (now - last_capture)
                    print(f"\r  Checkerboard detected — move to new pose in {remaining:.1f}s …",
                          end="", flush=True)
            else:
                print(f"\r  [{count:2d}/{target}] Waiting for checkerboard …       ",
                      end="", flush=True)

    except KeyboardInterrupt:
        print()

    print(f"\nCaptured {count} images.")
    return count

# ---------------------------------------------------------------------------
# Calibration math
# ---------------------------------------------------------------------------

def run_calibration(checkerboard, square_size, images_dir, output_file):
    """Compute and save camera_matrix + dist_coeffs from saved checkerboard images."""
    objp = np.zeros((checkerboard[0] * checkerboard[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:checkerboard[0], 0:checkerboard[1]].T.reshape(-1, 2)
    objp *= square_size

    objpoints, imgpoints = [], []
    images = glob.glob(os.path.join(images_dir, "*.png"))

    if len(images) < 5:
        print("Not enough calibration images found. Run capture first.")
        return False

    img_shape = None
    for fname in sorted(images):
        img  = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_shape = gray.shape[::-1]
        found, corners = cv2.findChessboardCorners(gray, checkerboard, None)
        if found:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            objpoints.append(objp)
            imgpoints.append(corners)

    print(f"Using {len(objpoints)} of {len(images)} images for calibration…")
    ret, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
        objpoints, imgpoints, img_shape, None, None
    )

    print("\nCalibration complete.")
    print(f"Reprojection error: {ret:.4f}  (aim for < 0.5)")
    print("Camera matrix:\n", camera_matrix)
    print("Distortion coefficients:\n", dist_coeffs)

    np.savez(output_file, camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)
    print(f"\nSaved to {output_file}")
    return True

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Calibrate camera using the image topic from aruco_config.json"
    )
    parser.add_argument(
        "--config", default=_DEFAULT_CONFIG,
        help="JSON config file to read the image topic from (default: configs/aruco_config.json)",
    )
    parser.add_argument(
        "--square-size", type=float, default=_DEFAULT_SQUARE_SIZE,
        help="Physical size of one checkerboard square in metres (default: 0.0254)",
    )
    parser.add_argument(
        "--checkerboard", default="9x6",
        help="Inner corner count as WxH (default: 9x6 for a standard 10×7 board)",
    )
    parser.add_argument(
        "--calibrate-only", action="store_true",
        help="Skip capture and re-run calibration on existing images in calib_images/",
    )
    parser.add_argument(
        "--video", default=None,
        help="Use a video file or webcam index instead of ROS (e.g. --video 0 for webcam)",
    )
    parser.add_argument(
        "--headless", action="store_true",
        help="No display — auto-capture whenever a checkerboard is detected (terminal only)",
    )
    parser.add_argument(
        "--keyboard", action="store_true",
        help="No display — press Enter to capture each pose, 'q'+Enter to stop",
    )
    parser.add_argument(
        "--count", type=int, default=20,
        help="Target number of images for --headless / --keyboard (default: 20)",
    )
    parser.add_argument(
        "--cooldown", type=float, default=2.0,
        help="--headless only: minimum seconds between auto-captures (default: 2.0)",
    )
    args = parser.parse_args()

    # Parse checkerboard dimensions
    try:
        w, h = args.checkerboard.lower().split("x")
        checkerboard = (int(w), int(h))
    except ValueError:
        print(f"Invalid --checkerboard '{args.checkerboard}' — expected format: 9x6")
        sys.exit(1)

    if not args.calibrate_only:
        if args.video is not None:
            # Standalone webcam / video-file mode
            try:
                src = int(args.video)
            except ValueError:
                src = args.video
            cap = cv2.VideoCapture(src)
            if not cap.isOpened():
                print(f"Error: could not open video source {src!r}")
                sys.exit(1)
            print(f"Video source: {src!r}")
            if args.keyboard:
                loop_fn, kwargs = _capture_loop_keyboard, {"target": args.count}
            elif args.headless:
                loop_fn, kwargs = _capture_loop_headless, {"target": args.count, "cooldown": args.cooldown}
            else:
                loop_fn, kwargs = _capture_loop, {}
            n = loop_fn(
                get_frame_fn=lambda: cap.read()[1],
                spin_fn=lambda: None,
                checkerboard=checkerboard,
                images_dir=_IMAGES_DIR,
                **kwargs,
            )
            cap.release()
        else:
            # ROS2 mode — read topic from config
            image_topic = load_image_topic(args.config)
            _import_ros()
            rclpy.init()
            cam = _ImageSubscriber(image_topic)
            cam.start()
            if args.keyboard:
                loop_fn, kwargs = _capture_loop_keyboard, {"target": args.count}
            elif args.headless:
                loop_fn, kwargs = _capture_loop_headless, {"target": args.count, "cooldown": args.cooldown}
            else:
                loop_fn, kwargs = _capture_loop, {}
            n = loop_fn(
                get_frame_fn=cam.get_frame,
                spin_fn=cam.spin_once,
                checkerboard=checkerboard,
                images_dir=_IMAGES_DIR,
                **kwargs,
            )
            cam.shutdown()
            rclpy.shutdown()

        if n < 5:
            print("Too few images captured — skipping calibration.")
            return

    run_calibration(checkerboard, args.square_size, _IMAGES_DIR, _OUTPUT_FILE)


if __name__ == "__main__":
    main()
