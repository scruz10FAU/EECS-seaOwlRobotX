#!/usr/bin/env python3
"""
export_tflite.py — export an Ultralytics YOLO .pt model to int8 TFLite for
Hexagon-NPU inference on the ModalAI Starling2 Max (see
tflite_hexagon_detector.py).

This is an offline, one-time (per model) step run on a dev machine — it is
NOT part of the runtime import graph and is not needed unless you're
switching detection.backend to "tflite_hexagon" in the JSON config.

int8 quantization requires a REPRESENTATIVE CALIBRATION IMAGE SET: a small
YOLO-format dataset (~100-300 images spanning the lighting/color conditions
you expect to fly in) referenced by a data.yaml, in the same format used for
training (see https://docs.ultralytics.com/datasets/detect/ for the schema).
Quantization accuracy is sensitive to how well this set matches real flight
imagery — don't just point it at the training set's val split if that
doesn't cover your deployment conditions.

Usage:
    python3 export_tflite.py --weights models/one_beacon.pt \\
        --data calibration/data.yaml --imgsz 640 --output-dir models/

    # Skip int8 (plain float32 TFLite — no calibration set needed, but
    # won't get you the same Hexagon NPU speedup):
    python3 export_tflite.py --weights models/one_beacon.pt --no-int8
"""

import argparse
import os
import shutil


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--weights", required=True,
                     help="Path to the source .pt model (e.g. models/one_beacon.pt)")
    ap.add_argument("--data", default=None,
                     help="YOLO-format data.yaml pointing at the calibration image set "
                          "(required for --int8, ignored otherwise)")
    ap.add_argument("--imgsz", type=int, default=640,
                     help="Export input size — must match what tflite_hexagon_detector's "
                          "imgsz config is set to")
    ap.add_argument("--int8", dest="int8", action="store_true", default=True,
                     help="Quantize to int8 (default) — needed for the Hexagon delegate")
    ap.add_argument("--no-int8", dest="int8", action="store_false",
                     help="Export plain float32 TFLite instead (CPU-only, no calibration set needed)")
    ap.add_argument("--output-dir", default="models",
                     help="Where to place the resulting .tflite file")
    args = ap.parse_args()

    if args.int8 and not args.data:
        ap.error("--data is required for int8 export (see script docstring)")

    from ultralytics import YOLO

    print(f"[export] Loading {args.weights}")
    model = YOLO(args.weights)

    export_kwargs = {"format": "tflite", "imgsz": args.imgsz}
    if args.int8:
        export_kwargs["int8"] = True
        export_kwargs["data"] = args.data

    print(f"[export] Exporting: {export_kwargs}")
    exported_path = model.export(**export_kwargs)

    os.makedirs(args.output_dir, exist_ok=True)
    dest = os.path.join(args.output_dir, os.path.basename(exported_path))
    if os.path.abspath(exported_path) != os.path.abspath(dest):
        shutil.copy2(exported_path, dest)

    print(f"[export] Done -> {dest}")
    print("[export] Set this path as detection.model in your JSON config, "
          "and detection.backend = \"tflite_hexagon\", to use it.")


if __name__ == "__main__":
    main()
