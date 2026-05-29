#!/usr/bin/env python3
"""
batch_detect.py — Run beacon_detector on multiple videos and generate a detection summary.

Processes each video headlessly (no display window), writes per-frame detections to a
combined CSV, and prints a human-readable summary of what was found in each video.

Usage:
    python3 batch_detect.py video1.mp4 video2.mp4 video3.mp4 ...
    python3 batch_detect.py -m models/one_beacon.pt -cm models/best_crop.pt videos/*.mp4
    python3 batch_detect.py --output-dir /path/to/logs video1.mp4 ...

Output files (written to --output-dir, default: same dir as first video):
    batch_detections_<timestamp>.csv   — every per-frame detection across all videos
    batch_summary_<timestamp>.txt      — human-readable summary per video
"""

import sys
import os
import csv
import argparse
import time
from pathlib import Path
from collections import Counter, defaultdict

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)


# ── Batch CSV header ──────────────────────────────────────────────────────────

_BATCH_HEADER = [
    "video", "timestamp", "frame",
    "color", "color_confidence", "intensity",
    "is_blinking", "blink_hz", "blink_phase",
    "vote_red", "vote_green", "vote_blue", "vote_other",
    "det_confidence", "x1", "y1", "x2", "y2",
]


# ── Per-video stats accumulator ───────────────────────────────────────────────

class VideoStats:
    def __init__(self, video_path: str):
        self.video_path  = video_path
        self.video_name  = Path(video_path).name
        self.total_frames   = 0
        self.detect_frames  = 0   # frames with ≥1 detection
        self.color_counts: Counter = Counter()
        self.blink_true     = 0   # frames where is_blinking == True
        self.blink_false    = 0   # frames where is_blinking == False
        self.blink_unknown  = 0   # frames where is_blinking == None
        self.blink_hz_samples: list = []
        self.error: str = ""

    def record_detection(self, color: str, is_blinking, blink_hz):
        self.detect_frames += 1
        self.color_counts[color] += 1
        if is_blinking is True:
            self.blink_true += 1
            if blink_hz is not None:
                self.blink_hz_samples.append(blink_hz)
        elif is_blinking is False:
            self.blink_false += 1
        else:
            self.blink_unknown += 1

    def summary_lines(self) -> list:
        lines = [f"Video : {self.video_name}"]
        if self.error:
            lines.append(f"  ERROR: {self.error}")
            return lines

        det_rate = (self.detect_frames / self.total_frames * 100) if self.total_frames else 0
        lines.append(f"  Frames processed : {self.total_frames}")
        lines.append(f"  Frames with beacon: {self.detect_frames} ({det_rate:.1f}%)")

        if self.detect_frames == 0:
            lines.append("  No beacons detected.")
            return lines

        lines.append("  Color breakdown:")
        for color, count in self.color_counts.most_common():
            pct = count / self.detect_frames * 100
            lines.append(f"    {color:10s}  {count:5d} frames  ({pct:.1f}%)")

        total_blink_frames = self.blink_true + self.blink_false + self.blink_unknown
        if total_blink_frames > 0:
            lines.append("  Blink status (of detected frames):")
            lines.append(f"    Blinking  : {self.blink_true:5d} ({self.blink_true / total_blink_frames * 100:.1f}%)")
            lines.append(f"    Not blinking: {self.blink_false:5d} ({self.blink_false / total_blink_frames * 100:.1f}%)")
            lines.append(f"    Unknown   : {self.blink_unknown:5d} ({self.blink_unknown / total_blink_frames * 100:.1f}%)")
            if self.blink_hz_samples:
                avg_hz = sum(self.blink_hz_samples) / len(self.blink_hz_samples)
                lines.append(f"    Avg blink frequency: {avg_hz:.2f} Hz")
        return lines


# ── Core processing ───────────────────────────────────────────────────────────

def process_video(
    video_path: str,
    model,
    crop_model,
    conf: float,
    batch_writer,
) -> VideoStats:
    """
    Process one video headlessly and write per-frame detections to batch_writer.
    Returns a VideoStats instance summarising results.
    """
    import cv2
    from beacon_detector import isolate_and_classify, BlinkDetector

    stats = VideoStats(video_path)
    video_name = Path(video_path).name

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        stats.error = f"Cannot open video: {video_path}"
        print(f"  [ERROR] {stats.error}")
        return stats

    total  = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps    = cap.get(cv2.CAP_PROP_FPS) or 30.0
    width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"  {width}x{height} @ {fps:.1f}fps  {total} frames")

    stats.total_frames = total
    blink_detector = BlinkDetector()
    frame_idx = 0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            frame_idx += 1

            results = model(frame, conf=conf, verbose=False)
            boxes   = results[0].boxes
            names   = results[0].names

            if len(boxes) == 0:
                continue

            # Use the video's own timestamp so blink detection timing is correct
            # regardless of how fast frames are decoded.  CAP_PROP_POS_MSEC gives
            # the presentation timestamp of the most-recently decoded frame in ms.
            ts = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                det_conf = float(box.conf[0])

                crop = frame[max(y1, 0):max(y2, 1), max(x1, 0):max(x2, 1)]
                beacon_color, color_conf, _mask, intensity, votes = isolate_and_classify(crop, crop_model)

                blink_info = blink_detector.update(ts, beacon_color, intensity)
                is_blinking = blink_info["is_blinking"]
                blink_hz    = blink_info["blink_hz"]
                blink_phase = blink_info["phase"]

                stats.record_detection(beacon_color, is_blinking, blink_hz)

                batch_writer.writerow([
                    video_name,
                    f"{ts:.3f}",
                    frame_idx,
                    beacon_color,
                    f"{color_conf:.4f}",
                    f"{intensity:.4f}",
                    "" if is_blinking is None else str(is_blinking),
                    "" if blink_hz    is None else f"{blink_hz:.2f}",
                    blink_phase,
                    f"{votes.get('red',    0):.4f}",
                    f"{votes.get('green',  0):.4f}",
                    f"{votes.get('blue',   0):.4f}",
                    f"{votes.get('other',  0):.4f}",
                    f"{det_conf:.4f}",
                    x1, y1, x2, y2,
                ])

    except KeyboardInterrupt:
        print("  [interrupted]")
    finally:
        cap.release()

    stats.total_frames = frame_idx
    pct = stats.detect_frames / frame_idx * 100 if frame_idx else 0
    print(f"  Done — {frame_idx} frames, {stats.detect_frames} with detections ({pct:.1f}%)")
    return stats


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        prog="batch_detect",
        description="Run beacon_detector on multiple videos and produce a combined log + summary",
    )
    parser.add_argument(
        "videos",
        nargs="+",
        metavar="VIDEO",
        help="Video files to process (up to 6 recommended)",
    )
    parser.add_argument(
        "--model", "-m",
        default=os.path.join(SCRIPT_DIR, "models", "one_beacon.pt"),
        help="Path to YOLO beacon model",
    )
    parser.add_argument(
        "--crop-model", "-cm",
        default=os.path.join(SCRIPT_DIR, "models", "best_crop.pt"),
        help="Path to YOLO lit-area crop model",
    )
    parser.add_argument(
        "--conf", "-c",
        type=float,
        default=0.5,
        help="Detection confidence threshold (default 0.5)",
    )
    parser.add_argument(
        "--output-dir", "-o",
        default=None,
        help="Directory for output files (default: directory of first video)",
    )
    args = parser.parse_args()

    # Validate videos
    missing = [v for v in args.videos if not os.path.exists(v)]
    if missing:
        for m in missing:
            print(f"[batch] File not found: {m}")
        sys.exit(1)

    # Resolve output directory
    out_dir = args.output_dir or os.path.dirname(os.path.abspath(args.videos[0]))
    os.makedirs(out_dir, exist_ok=True)

    ts_tag = time.strftime("%Y%m%d_%H%M%S")
    csv_path  = os.path.join(out_dir, f"batch_detections_{ts_tag}.csv")
    summ_path = os.path.join(out_dir, f"batch_summary_{ts_tag}.txt")

    # Load models once
    try:
        from ultralytics import YOLO
    except ImportError:
        print("[batch] ultralytics not installed: pip install ultralytics")
        sys.exit(1)

    if not os.path.exists(args.model):
        print(f"[batch] Model not found: {args.model}")
        sys.exit(1)
    if not os.path.exists(args.crop_model):
        print(f"[batch] Crop model not found: {args.crop_model}")
        sys.exit(1)

    print(f"[batch] Loading models...")
    model      = YOLO(args.model)
    crop_model = YOLO(args.crop_model)
    print(f"[batch] Beacon model : {args.model}")
    print(f"[batch] Crop model   : {args.crop_model}")
    print(f"[batch] Detections → {csv_path}")
    print(f"[batch] Summary     → {summ_path}")
    print()

    all_stats = []

    with open(csv_path, "w", newline="") as csv_fh:
        writer = csv.writer(csv_fh)
        writer.writerow(_BATCH_HEADER)

        for i, video_path in enumerate(args.videos, 1):
            print(f"[{i}/{len(args.videos)}] {os.path.basename(video_path)}")
            stats = process_video(video_path, model, crop_model, args.conf, writer)
            all_stats.append(stats)
            print()

    # Build and write summary
    sep = "=" * 60
    summary_lines = [
        sep,
        "BATCH DETECTION SUMMARY",
        f"Run at : {time.strftime('%Y-%m-%d %H:%M:%S')}",
        f"Videos : {len(args.videos)}",
        f"Log    : {csv_path}",
        sep,
        "",
    ]

    for stats in all_stats:
        summary_lines.extend(stats.summary_lines())
        summary_lines.append("")

    # Overall totals
    total_frames_all   = sum(s.total_frames   for s in all_stats)
    total_detect_all   = sum(s.detect_frames  for s in all_stats)
    all_colors: Counter = Counter()
    for s in all_stats:
        all_colors.update(s.color_counts)
    total_blink = sum(s.blink_true for s in all_stats)

    summary_lines += [
        sep,
        "OVERALL TOTALS",
        f"  Total frames processed  : {total_frames_all}",
        f"  Total detection frames  : {total_detect_all}",
        f"  Colors seen             : {', '.join(f'{c}={n}' for c, n in all_colors.most_common())}",
        f"  Frames with blinking=True: {total_blink}",
        sep,
    ]

    summary_text = "\n".join(summary_lines)
    print(summary_text)

    with open(summ_path, "w") as f:
        f.write(summary_text + "\n")

    print(f"\n[batch] Complete.  Detections → {csv_path}")
    print(f"[batch]            Summary     → {summ_path}")


if __name__ == "__main__":
    main()
