#!/usr/bin/env python3
"""
Telemetry viewer — laptop side of telemetry_stream.py.

Shows the drone's tracking_front stream with its GPS / altitude / attitude / IMU.
"Save frame" (or Space) writes the displayed JPEG to <out-dir>/images/<timestamp>.jpg
and appends that frame's telemetry to <out-dir>/captures.csv, keyed by filename.
"Record" / "Stop" writes every received frame, unre-encoded, to
<out-dir>/videos/<timestamp>.mkv (MJPEG, arrival-time timestamps) plus
<timestamp>.csv with one telemetry row per video frame.
The Exposure slider (log scale, ms) and Gain box set tracking_front's manual
exposure (voxl-camera-server set_exp_gain); "Auto" returns it to auto exposure.

Needs Pillow beyond the standard library (pip install pillow), and ffmpeg on PATH
for recording.

Usage:
    python3 telemetry_viewer.py                          # flight drone, 192.168.50.121
    python3 telemetry_viewer.py --host 192.168.50.104 --out-dir ~/captures
"""

import argparse
import csv
import io
import json
import math
import os
import shutil
import subprocess
import threading
import time
import tkinter as tk
import urllib.error
import urllib.request
from datetime import datetime

from PIL import Image, ImageTk

STALE_S = 1.0

# Exposure slider: 0..EXP_STEPS maps log-uniformly onto EXP_MIN_MS..EXP_MAX_MS
# (30 ms ~ one frame at 30 fps). Gain default from exposure_commands_test.txt.
EXP_MIN_MS, EXP_MAX_MS, EXP_STEPS = 0.05, 30.0, 1000
EXP_DEFAULT_MS, GAIN_DEFAULT = 3.0, 100

CSV_FIELDS = [
    "filename", "laptop_time_iso", "frame_stamp",
    "gps_lat", "gps_lon", "gps_alt_msl_m", "gps_fix_type", "gps_sats", "gps_eph", "gps_epv",
    "height_agl_m", "local_x", "local_y", "local_z", "heading_deg",
    "roll_deg", "pitch_deg", "yaw_deg", "qw", "qx", "qy", "qz",
    "gyro_x", "gyro_y", "gyro_z", "accel_x", "accel_y", "accel_z",
    "gps_age_s", "local_age_s", "att_age_s", "imu_age_s",
]

# (csv column, telemetry group, key in that group)
CSV_MAP = [
    ("gps_lat", "gps", "lat"), ("gps_lon", "gps", "lon"), ("gps_alt_msl_m", "gps", "alt_msl_m"),
    ("gps_fix_type", "gps", "fix_type"), ("gps_sats", "gps", "satellites"),
    ("gps_eph", "gps", "eph"), ("gps_epv", "gps", "epv"),
    ("height_agl_m", "local", "height_agl_m"), ("local_x", "local", "x"),
    ("local_y", "local", "y"), ("local_z", "local", "z"), ("heading_deg", "local", "heading_deg"),
    ("roll_deg", "attitude", "roll_deg"), ("pitch_deg", "attitude", "pitch_deg"),
    ("yaw_deg", "attitude", "yaw_deg"), ("qw", "attitude", "qw"), ("qx", "attitude", "qx"),
    ("qy", "attitude", "qy"), ("qz", "attitude", "qz"),
    ("gyro_x", "imu", "gyro_x"), ("gyro_y", "imu", "gyro_y"), ("gyro_z", "imu", "gyro_z"),
    ("accel_x", "imu", "accel_x"), ("accel_y", "imu", "accel_y"), ("accel_z", "imu", "accel_z"),
    ("gps_age_s", "gps", "age_s"), ("local_age_s", "local", "age_s"),
    ("att_age_s", "attitude", "age_s"), ("imu_age_s", "imu", "age_s"),
]


# ─── Stream reader ────────────────────────────────────────────────────

class StreamReader:
    """Reads the multipart MJPEG stream; keeps only the newest (jpeg, telemetry)."""

    def __init__(self, url):
        self.url = url
        self._lock = threading.Lock()
        self._latest = None          # (seq, jpeg bytes, telemetry dict)
        self._seq = 0
        self.status = "connecting…"
        self.fps = 0.0
        self.sink = None             # callable(jpeg, telem) run on every frame, e.g. Recorder.write
        threading.Thread(target=self._run, daemon=True).start()

    def latest(self):
        with self._lock:
            return self._latest

    def _run(self):
        while True:
            try:
                self.status = f"connecting to {self.url}"
                with urllib.request.urlopen(self.url, timeout=5) as resp:
                    self.status = "connected"
                    self._read_parts(resp)
            except Exception as e:  # network errors of every kind: retry
                self.status = f"disconnected ({e.__class__.__name__}: {e}) — retrying"
            self.fps = 0.0
            time.sleep(1.0)

    def _read_parts(self, resp):
        last = None
        while True:
            line = resp.readline()
            if not line:
                raise ConnectionError("stream closed")
            if not line.startswith(b"--frame"):
                continue
            headers = {}
            while True:
                h = resp.readline()
                if not h:
                    raise ConnectionError("stream closed")
                h = h.strip()
                if not h:
                    break
                k, _, v = h.partition(b":")
                headers[k.strip().lower()] = v.strip()
            n = int(headers.get(b"content-length", b"0"))
            jpeg = resp.read(n)
            if len(jpeg) != n:
                raise ConnectionError("short read")
            try:
                telem = json.loads(headers.get(b"x-telemetry", b"{}"))
            except ValueError:
                telem = {}
            now = time.monotonic()
            if last is not None:  # first interval includes connection setup
                dt = max(now - last, 1e-6)
                self.fps = 1 / dt if self.fps == 0 else 0.9 * self.fps + 0.1 / dt
            last = now
            with self._lock:
                self._seq += 1
                self._latest = (self._seq, jpeg, telem)
            sink = self.sink
            if sink is not None:
                sink(jpeg, telem)


def telemetry_row(telem, now: datetime) -> dict:
    """CSV_FIELDS minus filename, from one frame's telemetry ('' where missing)."""
    row = {"laptop_time_iso": now.isoformat(),
           "frame_stamp": (telem.get("frame") or {}).get("stamp")}
    for col, g, k in CSV_MAP:
        row[col] = (telem.get(g) or {}).get(k)
    return {k: "" if v is None else v for k, v in row.items()}


# ─── Recording ────────────────────────────────────────────────────────

class Recorder:
    """Pipes the stream's JPEGs into ffmpeg as-is (-c:v copy, no re-encode), stamped
    with arrival time, plus one telemetry CSV row per frame beside the video."""

    def __init__(self, video_path):
        self.path = video_path
        self.csv_path = os.path.splitext(video_path)[0] + ".csv"
        self.frames = 0
        self.error = None
        self._lock = threading.Lock()
        self._proc = subprocess.Popen(
            ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
             "-use_wallclock_as_timestamps", "1", "-f", "mjpeg", "-i", "-",
             "-c:v", "copy", video_path],
            stdin=subprocess.PIPE)
        self._csv_file = open(self.csv_path, "w", newline="")
        self._csv = csv.DictWriter(self._csv_file, fieldnames=["frame"] + CSV_FIELDS[1:])
        self._csv.writeheader()

    def write(self, jpeg, telem):
        """Called on the reader thread for every received frame."""
        with self._lock:
            if self._proc is None or self.error:
                return
            try:
                self._proc.stdin.write(jpeg)
            except OSError as e:
                self.error = f"ffmpeg stopped ({e})"
                return
            self._csv.writerow({"frame": self.frames, **telemetry_row(telem, datetime.now())})
            self.frames += 1

    def stop(self):
        with self._lock:
            proc, self._proc = self._proc, None
            self._csv_file.close()
        if proc is None:
            return
        try:
            proc.stdin.close()
        except OSError:
            pass
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            self.error = "ffmpeg did not finish; video may be truncated"
        if self.frames == 0:  # nothing arrived: leave no empty files behind
            for p in (self.path, self.csv_path):
                if os.path.exists(p):
                    os.remove(p)


# ─── GUI ──────────────────────────────────────────────────────────────

def fmt(v, spec=".2f"):
    return "—" if v is None else format(v, spec)


class Viewer:
    def __init__(self, root, reader: StreamReader, out_dir, base_url):
        self.root, self.reader, self.base_url = root, reader, base_url
        self._exp_job = None         # pending debounced exposure send
        self.img_dir = os.path.join(out_dir, "images")
        self.csv_path = os.path.join(out_dir, "captures.csv")
        self.vid_dir = os.path.join(out_dir, "videos")
        self.shown = None            # (seq, jpeg, telem) currently on screen
        self.photo = None
        self.n_saved = 0
        self.recorder = None
        self.rec_started = 0.0

        root.title(f"Telemetry viewer — {reader.url}")
        root.configure(bg="#111")
        root.geometry("1280x720")

        self.canvas = tk.Label(root, bg="#111")
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        side = tk.Frame(root, bg="#1b1b1b", padx=10, pady=10)
        side.pack(side=tk.RIGHT, fill=tk.Y)
        mono = ("Courier", 11)
        self.groups = {}
        for g, title in (("gps", "GPS"), ("local", "Altitude / local"),
                         ("attitude", "Attitude"), ("imu", "IMU")):
            tk.Label(side, text=title, fg="#8cf", bg="#1b1b1b", anchor="w",
                     font=("Helvetica", 11, "bold")).pack(fill=tk.X, pady=(8, 0))
            lbl = tk.Label(side, text="—", fg="#ddd", bg="#1b1b1b", anchor="w",
                           justify=tk.LEFT, font=mono)
            lbl.pack(fill=tk.X)
            self.groups[g] = lbl

        tk.Label(side, text="Exposure", fg="#8cf", bg="#1b1b1b", anchor="w",
                 font=("Helvetica", 11, "bold")).pack(fill=tk.X, pady=(12, 0))
        self.exp_scale = tk.Scale(side, from_=0, to=EXP_STEPS, orient=tk.HORIZONTAL,
                                  showvalue=False, bg="#1b1b1b", fg="#ddd",
                                  troughcolor="#333", highlightthickness=0)
        self.exp_scale.set(self.exp_to_pos(EXP_DEFAULT_MS))
        self.exp_scale.pack(fill=tk.X)
        row = tk.Frame(side, bg="#1b1b1b")
        row.pack(fill=tk.X)
        self.exp_val = tk.Label(row, text=f"{EXP_DEFAULT_MS:.2f} ms", width=9, anchor="w",
                                fg="#ddd", bg="#1b1b1b", font=mono)
        self.exp_val.pack(side=tk.LEFT)
        tk.Label(row, text="gain", fg="#aaa", bg="#1b1b1b").pack(side=tk.LEFT)
        self.gain_var = tk.StringVar(value=str(GAIN_DEFAULT))
        self.gain_box = tk.Spinbox(row, from_=1, to=16000, increment=10, width=6,
                                   textvariable=self.gain_var, command=self.on_exposure)
        self.gain_box.pack(side=tk.LEFT, padx=(4, 8))
        self.gain_box.bind("<Return>", lambda _e: self.on_exposure())
        tk.Button(row, text="Auto", command=self.auto_exposure).pack(side=tk.RIGHT)
        self.exp_lbl = tk.Label(side, text="not set (camera keeps its current exposure)",
                                fg="#aaa", bg="#1b1b1b", anchor="w", justify=tk.LEFT,
                                wraplength=260, font=("Helvetica", 9))
        self.exp_lbl.pack(fill=tk.X)
        # Hooked up after set() so the initial position isn't sent to the drone.
        self.exp_scale.config(command=lambda _v: self.on_exposure())

        self.save_btn = tk.Button(side, text="Save frame  [Space]", command=self.save,
                                  font=("Helvetica", 12, "bold"), bg="#2a6", fg="white",
                                  activebackground="#3b7", padx=10, pady=8)
        self.save_btn.pack(fill=tk.X, pady=(20, 6))
        self.save_lbl = tk.Label(side, text=f"saving to {os.path.abspath(out_dir)}",
                                 fg="#aaa", bg="#1b1b1b", anchor="w", justify=tk.LEFT,
                                 wraplength=260, font=("Helvetica", 9))
        self.save_lbl.pack(fill=tk.X)

        self.rec_btn = tk.Button(side, text="Record", command=self.toggle_record,
                                 font=("Helvetica", 12, "bold"), bg="#b33", fg="white",
                                 activebackground="#c44", padx=10, pady=8)
        self.rec_btn.pack(fill=tk.X, pady=(14, 6))
        self.rec_lbl = tk.Label(side, text="", fg="#aaa", bg="#1b1b1b", anchor="w",
                                justify=tk.LEFT, wraplength=260, font=("Helvetica", 9))
        self.rec_lbl.pack(fill=tk.X)

        self.status_lbl = tk.Label(side, text="", fg="#aaa", bg="#1b1b1b", anchor="w",
                                   justify=tk.LEFT, wraplength=260, font=("Helvetica", 9))
        self.status_lbl.pack(side=tk.BOTTOM, fill=tk.X)

        root.bind("<space>", lambda _e: self.save())
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.tick()

    def tick(self):
        item = self.reader.latest()
        if item is not None and (self.shown is None or item[0] != self.shown[0]):
            self.shown = item
            self.show_image(item[1])
            self.show_telemetry(item[2])
        self.status_lbl.config(text=f"{self.reader.status}\n{self.reader.fps:4.1f} fps")
        rec = self.recorder
        if rec is not None:
            secs = int(time.monotonic() - self.rec_started)
            if rec.error:
                self.rec_lbl.config(text=f"● {rec.error}", fg="#c55")
            else:
                self.rec_lbl.config(text=f"● REC {secs // 60:02d}:{secs % 60:02d}  "
                                         f"{rec.frames} frames\n{os.path.basename(rec.path)}",
                                    fg="#e55")
        self.root.after(30, self.tick)

    def toggle_record(self):
        if self.recorder is None:
            if shutil.which("ffmpeg") is None:
                self.rec_lbl.config(text="ffmpeg not found on PATH", fg="#c55")
                return
            os.makedirs(self.vid_dir, exist_ok=True)
            name = datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".mkv"
            try:
                self.recorder = Recorder(os.path.join(self.vid_dir, name))
            except OSError as e:
                self.rec_lbl.config(text=f"cannot start recording: {e}", fg="#c55")
                return
            self.rec_started = time.monotonic()
            self.reader.sink = self.recorder.write
            self.rec_btn.config(text="Stop", bg="#555", activebackground="#666")
        else:
            rec, self.recorder = self.recorder, None
            self.reader.sink = None
            rec.stop()
            self.rec_btn.config(text="Record", bg="#b33", activebackground="#c44")
            msg = (f"saved {os.path.basename(rec.path)} ({rec.frames} frames)" if rec.frames
                   else "no frames received — nothing saved")
            self.rec_lbl.config(text=f"{msg}\n{rec.error}" if rec.error else msg,
                                fg="#c55" if rec.error else "#6c6")

    @staticmethod
    def exp_to_pos(ms):
        return round(EXP_STEPS * math.log(ms / EXP_MIN_MS) / math.log(EXP_MAX_MS / EXP_MIN_MS))

    @staticmethod
    def pos_to_exp(pos):
        return EXP_MIN_MS * (EXP_MAX_MS / EXP_MIN_MS) ** (float(pos) / EXP_STEPS)

    def on_exposure(self):
        """Slider/gain changed: update the readout now, send once it settles."""
        self.exp_val.config(text=f"{self.pos_to_exp(self.exp_scale.get()):.2f} ms")
        if self._exp_job is not None:
            self.root.after_cancel(self._exp_job)
        self._exp_job = self.root.after(150, self.send_exposure)

    def send_exposure(self):
        self._exp_job = None
        try:
            gain = int(float(self.gain_var.get()))
            if gain <= 0:
                raise ValueError
        except ValueError:
            self.exp_lbl.config(text="gain must be a positive number", fg="#c55")
            return
        ms = self.pos_to_exp(self.exp_scale.get())
        self.camera_cmd(f"exp_ms={ms:.3f}&gain={gain}")

    def auto_exposure(self):
        if self._exp_job is not None:
            self.root.after_cancel(self._exp_job)
            self._exp_job = None
        self.camera_cmd("ae=1")

    def camera_cmd(self, query):
        """POST /camera?<query> off the UI thread; result lands in exp_lbl."""
        def run():
            req = urllib.request.Request(f"{self.base_url}/camera?{query}", data=b"",
                                         method="POST")
            try:
                with urllib.request.urlopen(req, timeout=3) as resp:
                    msg, ok = json.loads(resp.read()).get("cmd", "ok"), True
            except urllib.error.HTTPError as e:
                try:
                    msg = json.loads(e.read()).get("error", str(e))
                except ValueError:
                    msg = str(e)
                ok = False
            except Exception as e:  # network errors of every kind
                msg, ok = f"{e.__class__.__name__}: {e}", False
            self.root.after(0, lambda: self.exp_lbl.config(
                text=msg if ok else f"failed: {msg}", fg="#6c6" if ok else "#c55"))

        threading.Thread(target=run, daemon=True).start()

    def close(self):
        if self.recorder is not None:
            self.toggle_record()
        self.root.destroy()

    def show_image(self, jpeg):
        try:
            img = Image.open(io.BytesIO(jpeg))
            img.load()
        except Exception:
            return
        w = max(self.canvas.winfo_width(), 1)
        h = max(self.canvas.winfo_height(), 1)
        s = min(w / img.width, h / img.height)
        if s > 0 and abs(s - 1) > 0.01:
            img = img.resize((max(1, int(img.width * s)), max(1, int(img.height * s))),
                             Image.BILINEAR)
        self.photo = ImageTk.PhotoImage(img)
        self.canvas.config(image=self.photo)

    def show_telemetry(self, t):
        gps, loc, att, imu = (t.get(g) for g in ("gps", "local", "attitude", "imu"))
        text = {
            "gps": gps and (f"lat  {fmt(gps['lat'], '.7f')}\nlon  {fmt(gps['lon'], '.7f')}\n"
                            f"alt  {fmt(gps['alt_msl_m'])} m MSL\n"
                            f"fix  {gps['fix_type']}  sats {gps['satellites']}  "
                            f"eph {fmt(gps['eph'], '.1f')}"),
            "local": loc and (f"AGL  {fmt(loc['height_agl_m'])} m\n"
                              f"NED  {fmt(loc['x'], '.1f')} {fmt(loc['y'], '.1f')} "
                              f"{fmt(loc['z'], '.1f')}\nhdg  {fmt(loc['heading_deg'], '.1f')}°"),
            "attitude": att and (f"roll  {fmt(att['roll_deg'], '7.2f')}°\n"
                                 f"pitch {fmt(att['pitch_deg'], '7.2f')}°\n"
                                 f"yaw   {fmt(att['yaw_deg'], '7.2f')}°"),
            "imu": imu and (f"gyro  {fmt(imu['gyro_x'], '6.2f')} {fmt(imu['gyro_y'], '6.2f')} "
                            f"{fmt(imu['gyro_z'], '6.2f')}\n"
                            f"accel {fmt(imu['accel_x'], '6.2f')} {fmt(imu['accel_y'], '6.2f')} "
                            f"{fmt(imu['accel_z'], '6.2f')}"),
        }
        for g, lbl in self.groups.items():
            d = t.get(g)
            if not d:
                lbl.config(text="no data", fg="#c55")
                continue
            age = d.get("age_s") or 0.0
            lbl.config(text=f"{text[g]}\nage  {age:.2f} s", fg="#ddd" if age < STALE_S else "#c55")

    def save(self):
        if self.shown is None:
            self.save_lbl.config(text="nothing to save yet", fg="#c55")
            return
        _, jpeg, telem = self.shown
        now = datetime.now()
        name = now.strftime("%Y%m%d_%H%M%S_%f") + ".jpg"
        os.makedirs(self.img_dir, exist_ok=True)
        with open(os.path.join(self.img_dir, name), "wb") as f:
            f.write(jpeg)

        new = not os.path.exists(self.csv_path)
        with open(self.csv_path, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            if new:
                w.writeheader()
            w.writerow({"filename": name, **telemetry_row(telem, now)})
        self.n_saved += 1
        self.save_lbl.config(text=f"saved #{self.n_saved}: {name}", fg="#6c6")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--host", default="192.168.50.121", help="drone IP")
    ap.add_argument("--port", type=int, default=8092)
    ap.add_argument("--out-dir", default="captures", help="where images/ and captures.csv go")
    args = ap.parse_args()

    base_url = f"http://{args.host}:{args.port}"
    reader = StreamReader(f"{base_url}/stream.mjpg")
    root = tk.Tk()
    Viewer(root, reader, os.path.expanduser(args.out_dir), base_url)
    root.mainloop()


if __name__ == "__main__":
    main()
