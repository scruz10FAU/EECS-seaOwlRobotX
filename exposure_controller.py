#!/usr/bin/env python3
"""
exposure_controller.py -- keep the beacons on the water from blowing out.

On Course Charlie the tracking_front auto exposure metered on the dark water,
so the white beacons were clipped to pure white and the diamond markings the
detector looks for were gone. This node runs alongside beacon_overlay and,
about once a second, measures the brightest pixels of the WATER part of the
frame (the lower part of the image: no sky, no buildings) and steers the
manual exposure so they sit just below clipping:

  clipped pixels >= --clip-px      -> exposure x --down (x --fast-down when
                                      over 10x --clip-px)
  highlight < --low (too dim)      -> exposure x target/highlight (at most x --max-step)
  otherwise                        -> hold

The beacons are a tiny part of the frame: on Charlie, 92-618 clipped pixels
(0.02-0.13 % of the water region) with beacons in view, so "clipped pixels" is
a count, and "highlight" is a very high percentile (--percentile 99.97, which
tracks the beacons there; 99.5 tracked the water). Clipped spots smaller than
--min-blob-px (sun-glint / noise specks: on Charlie 7,649 of 10,735 clipped
spots were 1-3 px) are not counted, and --clip-px more must remain before it
steps down. Increases are slow (--max-step) so that with no beacon in view it
doesn't chase the water brightness and overexpose the next beacon. Exposure
stays within [--min-exp, --max-exp] ms at a fixed --gain.

Start: it begins at --start-exp (high, 8 ms) and comes down fast while the
beacons/scene clip, so start it before takeoff (it settles in a few seconds).
After a frame gap it retakes control at the last exposure, not the start one.
A status line is printed every --status-every seconds, plus a one-time warning
while stuck at a limit (e.g. at --min-exp and still clipped).

Safety: on a normal stop (SIGINT/SIGTERM), on an exception, and whenever no
frame has arrived for --stale-s, it sends "start_ae" so the camera is never
left on a fixed exposure without this node steering it. (A SIGKILL can't be
caught -- stop the container with SIGTERM/SIGINT, as run_exposure_controller.sh does.)

Commands go to voxl-camera-server's control pipe <mpa-dir>/<camera>/control,
as voxl-send-command and telemetry_stream.py do; the container needs /run/mpa
mounted read-write. The state is published on --status-topic (std_msgs/String
JSON: exp_ms, gain, highlight, mode) so a flight recording shows what exposure
each moment had (record_flight.sh bags it).

Usage (inside a ROS 2 Humble container, see run_exposure_controller.sh):
    python3 exposure_controller.py                    # control
    python3 exposure_controller.py --dry-run          # measure + log, send nothing
"""

import argparse
import json
import os
import signal
import sys
import threading
import time

import numpy as np

try:
    import cv2   # connected components for the glint filter
except ImportError:
    cv2 = None


class CameraControl:
    """Writes voxl-camera-server commands into <mpa_dir>/<camera>/control."""

    def __init__(self, mpa_dir: str, camera: str, dry_run: bool):
        self.path = os.path.join(mpa_dir, camera, "control")
        self.dry_run = dry_run

    def send(self, cmd: str) -> bool:
        if self.dry_run:
            print(f"[exposure] (dry run) would send: {cmd}", flush=True)
            return True
        try:
            # Non-blocking: fails with ENXIO instead of hanging if the server is down.
            fd = os.open(self.path, os.O_WRONLY | os.O_NONBLOCK)
            try:
                os.write(fd, cmd.encode() + b"\0")   # NUL-terminated, as libmodal_pipe sends
            finally:
                os.close(fd)
            return True
        except OSError as e:
            print(f"[exposure] ERROR: '{cmd}' -> {self.path}: {e}", flush=True)
            return False


def to_gray(msg):
    """sensor_msgs/Image -> 2-D uint8 array, or None for an unsupported encoding."""
    h, w, step = msg.height, msg.width, msg.step
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding == "mono8":
        return buf.reshape(h, step)[:, :w]
    if msg.encoding in ("bgr8", "rgb8"):
        return buf.reshape(h, step)[:, :w * 3].reshape(h, w, 3).max(axis=2)  # brightest channel
    return None


def measure(gray, roi_top: float, percentile: float, clip_value: float, min_blob_px: int = 1):
    """(highlight percentile, clipped pixel count, median "base" brightness) of
    the water region (full-resolution rows from roi_top down). Clipped spots
    smaller than min_blob_px (glint/noise specks) are left out of the count."""
    roi = gray[int(gray.shape[0] * roi_top):, :]
    hl = float(np.percentile(roi, percentile))
    base = float(np.median(roi))
    mask = roi >= clip_value
    if min_blob_px <= 1 or cv2 is None or not mask.any():
        return hl, int(np.count_nonzero(mask)), base
    _, _, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
    areas = stats[1:, cv2.CC_STAT_AREA]
    return hl, int(areas[areas >= min_blob_px].sum()), base


MIN_DOWN_FACTOR = 0.25   # largest single decrease, however long the down streak


def next_exposure(exp_ms, hl, clipped, a, base=None, streak=0):
    """(new exposure, mode) for one control step.

    With the water region's median brightness `base`:
      floor   -- never darken the scene below a.floor (the brightest object
                 stays partly clipped instead); a scene under 80 % of it goes up.
      ceiling -- never brighten past a.ceiling: with no beacon in view it holds
                 near a beacon-ready exposure instead of climbing to max.
    `streak` = consecutive down steps so far: each one multiplies the decrease
    by a.accel (never below MIN_DOWN_FACTOR), so a newly clipped beacon is
    brought down in a few seconds."""
    floor = getattr(a, "floor", 0) if base is not None else 0
    ceiling = getattr(a, "ceiling", 0) if base is not None else 0
    if floor > 0 and base < 0.8 * floor:
        new, mode = exp_ms * min(a.max_step, floor / max(base, 0.5)), f"below floor ({base:.0f}): up"
    elif clipped >= a.clip_px:
        factor = a.fast_down if clipped >= 10 * a.clip_px else a.down
        factor = max(MIN_DOWN_FACTOR, factor * getattr(a, "accel", 1.0) ** streak)
        if floor > 0:
            factor = max(factor, floor / max(base, 1e-6))   # land on the floor, not under it
            if factor >= 1.0:
                return exp_ms, f"clipped ({clipped} px) but at floor ({base:.0f}): hold"
        new, mode = exp_ms * factor, f"clipped ({clipped} px): down"
    elif hl < a.low:
        factor = min(a.max_step, a.target / max(hl, 1.0))
        if ceiling > 0:
            factor = min(factor, ceiling / max(base, 1e-6))   # land on the ceiling, not over it
            if factor <= 1.0:
                return exp_ms, f"dim but at ceiling ({base:.0f}): hold"
        new, mode = exp_ms * factor, "dim: up"
    else:
        return exp_ms, "hold"
    return min(a.max_exp, max(a.min_exp, new)), mode


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default="/tracking_front", help="sensor_msgs/Image to measure")
    ap.add_argument("--camera", default="tracking_front", help="voxl-camera-server camera name")
    ap.add_argument("--mpa-dir", default="/run/mpa")
    ap.add_argument("--period", type=float, default=1.0, help="seconds between adjustments")
    ap.add_argument("--roi-top", type=float, default=0.55,
                    help="measure rows from this fraction of the height down (0.55 = below "
                         "the horizon in the Charlie frames)")
    ap.add_argument("--percentile", type=float, default=99.97,
                    help="highlight percentile of the water region (beacons are ~0.1 %% of it)")
    ap.add_argument("--clip-value", type=float, default=250, help="pixel value counted as clipped")
    ap.add_argument("--clip-px", type=int, default=50,
                    help="this many clipped pixels -> step down (fewer = sun glints, ignored)")
    ap.add_argument("--floor", type=float, default=8,
                    help="never darken the water region's median brightness below this (0-255; "
                         "0 = off). A much brighter object (flashlight, big reflection) then stays "
                         "partly clipped instead of blacking out the frame. Charlie's water under "
                         "auto exposure was ~24, so 8 allows ~3x darker")
    ap.add_argument("--ceiling", type=float, default=30,
                    help="never brighten the water region's median brightness past this "
                         "(0-255; 0 = off), so with no beacon in view it stays near a "
                         "beacon-ready exposure (Charlie's water under auto exposure: ~24)")
    ap.add_argument("--accel", type=float, default=0.7,
                    help="each consecutive down step multiplies the decrease by this "
                         f"(1 = constant steps; never below x{MIN_DOWN_FACTOR})")
    ap.add_argument("--min-blob-px", type=int, default=4,
                    help="clipped spots smaller than this (px) aren't counted: glint/noise "
                         "specks (1 = count every clipped pixel)")
    ap.add_argument("--low", type=float, default=170, help="highlight below = too dim")
    ap.add_argument("--target", type=float, default=215, help="highlight aimed for when raising")
    ap.add_argument("--down", type=float, default=0.7, help="exposure factor when clipped")
    ap.add_argument("--fast-down", type=float, default=0.5,
                    help="exposure factor when over 10x --clip-px pixels are clipped")
    ap.add_argument("--max-step", type=float, default=1.25, help="largest exposure increase factor")
    ap.add_argument("--min-exp", type=float, default=0.1, help="ms")
    ap.add_argument("--max-exp", type=float, default=10.0, help="ms")
    ap.add_argument("--start-exp", type=float, default=8.0,
                    help="ms, first manual exposure (high: it comes down fast while clipped)")
    ap.add_argument("--status-every", type=float, default=5.0,
                    help="seconds between status lines (0 = only log changes)")
    ap.add_argument("--gain", type=int, default=100)
    ap.add_argument("--stale-s", type=float, default=5.0,
                    help="no frame for this long -> back to auto exposure until frames return")
    ap.add_argument("--status-topic", default="/seabird/exposure")
    ap.add_argument("--dry-run", action="store_true", help="measure and log, send no commands")
    a = ap.parse_args()
    if not (0 < a.min_exp <= a.start_exp <= a.max_exp) or not (a.low < a.target < a.clip_value):
        sys.exit("need min_exp <= start_exp <= max_exp and low < target < clip_value")
    if a.floor > 0 and a.ceiling > 0 and a.ceiling <= a.floor:
        sys.exit("need --ceiling > --floor (or 0 to turn one off)")
    if not (0 < a.accel <= 1):
        sys.exit("need 0 < --accel <= 1")

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    cam = CameraControl(a.mpa_dir, a.camera, a.dry_run)
    latest = {"img": None, "t": 0.0}
    lock = threading.Lock()
    warned = set()

    rclpy.init()
    node = rclpy.create_node("exposure_controller")
    status_pub = node.create_publisher(String, a.status_topic, 10)

    def on_image(msg):
        g = to_gray(msg)
        if g is None:
            if msg.encoding not in warned:
                warned.add(msg.encoding)
                print(f"[exposure] unsupported encoding '{msg.encoding}' on {a.topic}", flush=True)
            return
        with lock:
            latest["img"], latest["t"] = g, time.monotonic()

    node.create_subscription(Image, a.topic, on_image, qos_profile_sensor_data)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    state = {"manual": False, "exp": a.start_exp, "streak": 0}   # streak: consecutive down steps
    stop = threading.Event()

    def to_auto(why):
        if state["manual"]:
            cam.send("start_ae")
            state["manual"] = False
            print(f"[exposure] back to auto exposure ({why})", flush=True)

    def on_signal(signum, _frame):
        stop.set()
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    min_blob = a.min_blob_px
    if min_blob > 1 and cv2 is None:
        print("[exposure] WARNING: cv2 not available -- glint filter off, every clipped "
              "pixel counts", flush=True)
        min_blob = 1
    print(f"[exposure] {a.topic} -> {cam.path}{' (DRY RUN)' if a.dry_run else ''}: "
          f"rows >= {a.roi_top:.2f}H, down at >= {a.clip_px} px >= {a.clip_value:g} "
          f"(spots >= {min_blob} px), up when p{a.percentile:g} < {a.low:g}, "
          f"floor {a.floor:g}{' (off)' if a.floor <= 0 else ''}, "
          f"ceiling {a.ceiling:g}{' (off)' if a.ceiling <= 0 else ''}, accel {a.accel:g}, "
          f"{a.min_exp:g}-{a.max_exp:g} ms, start {a.start_exp:g} ms, gain {a.gain}", flush=True)
    last_status = 0.0
    at_limit = None          # "min" / "max" while pinned there (warned once)
    try:
        while not stop.wait(a.period):
            with lock:
                img, t = latest["img"], latest["t"]
            if img is None or time.monotonic() - t > a.stale_s:
                to_auto(f"no frame for {a.stale_s:g} s" if img is not None else "no frames yet")
                continue
            hl, clipped, base = measure(img, a.roi_top, a.percentile, a.clip_value, min_blob)
            if not state["manual"]:
                # (Re)take control at the last exposure we used.
                if cam.send(f"set_exp_gain {state['exp']:.3f} {a.gain}"):
                    state["manual"] = True
                    print(f"[exposure] manual control at {state['exp']:.3f} ms gain {a.gain} "
                          f"(highlight {hl:.0f})", flush=True)
                mode = "take control"
            else:
                new, mode = next_exposure(state["exp"], hl, clipped, a, base, state["streak"])
                state["streak"] = state["streak"] + 1 if mode.endswith(": down") else 0
                if abs(new - state["exp"]) > 1e-6:
                    if cam.send(f"set_exp_gain {new:.3f} {a.gain}"):
                        print(f"[exposure] highlight {hl:.0f}, {mode}: "
                              f"{state['exp']:.3f} -> {new:.3f} ms", flush=True)
                        state["exp"] = new
            # One warning per stay at a limit, while it still wants to go further.
            limit = ("min" if state["exp"] <= a.min_exp + 1e-9 and clipped >= a.clip_px else
                     "max" if state["exp"] >= a.max_exp - 1e-9 and hl < a.low else None)
            if limit != at_limit and limit is not None:
                print(f"[exposure] WARNING: at {'minimum' if limit == 'min' else 'maximum'} "
                      f"{state['exp']:.3f} ms and still "
                      f"{f'clipped ({clipped} px)' if limit == 'min' else f'dim (highlight {hl:.0f})'}"
                      f" -- {'a light source in view?' if limit == 'min' else 'raise --max-exp for dark scenes'}",
                      flush=True)
            at_limit = limit
            now = time.monotonic()
            if a.status_every > 0 and now - last_status >= a.status_every:
                last_status = now
                print(f"[exposure] status: {state['exp']:.3f} ms gain {a.gain}, highlight {hl:.0f}, "
                      f"clipped {clipped} px, base {base:.0f}, {mode}"
                      f"{' (DRY RUN)' if a.dry_run else ''}", flush=True)
            status_pub.publish(String(data=json.dumps(dict(
                exp_ms=round(state["exp"], 3), gain=a.gain, highlight=round(hl, 1), clipped_px=clipped,
                base=round(base, 1), floor=a.floor,
                mode=mode, manual=state["manual"], dry_run=a.dry_run, t=time.time()))))
    except Exception as e:
        print(f"[exposure] ERROR: {type(e).__name__}: {e}", flush=True)
        raise
    finally:
        to_auto("stopping")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
