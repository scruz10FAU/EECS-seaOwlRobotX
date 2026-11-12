#!/usr/bin/env python3
"""
seabird_ground_gui.py — Seabird UAV ground station control panel (Windows).

One window to run the UAV ground station at the competition or on the bench:

  * Competition / Test profile switch. Each profile keeps its own settings
    (saved to seabird_ground_gui_settings.json next to this file).
      Competition: real RoboCommand on the course network. The RoboCommand
                   address is taken from the RoboCommand-facing adapter's
                   DHCP default gateway (Team Handbook 3.4.3: the gateway IS
                   the course RoboCommand address), or typed in.
      Test:        a local Mosquitto broker in WSL and a mock course
                   (tests/mock_robocommand_course.py), including the
                   deliberately INVALID courses for rehearsing the demo.
  * Preflight checks: latest RoboCommand protobuf, UAV pilot callsign and
    registration formats, altitude limit, and (Competition) the Handbook
    3.4.3 network rules -- DHCP, address in the course subnet, gateway =
    RoboCommand, no bridge / Internet sharing / forwarding, MQTT port
    (setting, default 1883) reachable, WSL mirrored networking.
  * Start / stop of the ELRS router (Windows), and in WSL the broker (Test),
    seaowls_ocs and beacon_mavlink_ground.py, with live logs. Every run's
    logs are kept in a new logs/ground_gui/<timestamp>/ folder.
  * Status: UAV heartbeats to RoboCommand, the geofence round trip
    (course -> UAV -> read back -> RunDeclaration: VALID / REFUSED + reason),
    the declaration and RunStart.

Run on Windows from this folder:
    py -3.10 seabird_ground_gui.py
"""

import json
import os
import queue
import re
import socket
import subprocess
import sys
import threading
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import messagebox, ttk

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
sys.path.insert(0, str(REPO))
from utils.geofence_check import validate_altitude   # noqa: E402

SETTINGS_FILE = HERE / "seabird_ground_gui_settings.json"
LOG_ROOT = REPO / "logs" / "ground_gui"
CREATE_NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)
ROBOCOMMAND_UPSTREAM = "https://github.com/robonation/robocommand.git"

# Same rules as seaowls_ocs (ocs_node.py).
CALLSIGN_RE = re.compile(r"^[A-Za-z]+(-[A-Za-z0-9]+)?$")
REGISTRATION_RE = re.compile(r"^[A-Z0-9]+-[0-9]+$")

# Team Handbook 3.4.2 course networks (+ the validation network from the
# RoboNation validation instructions): third octet -> (name, RoboCommand hosts).
COURSE_NETWORKS = {
    65: ("Course Alpha", {2}),
    66: ("Course Bravo", {2}),
    67: ("Course Charlie", {2}),
    68: ("Course Delta", {2}),
    69: ("Ground/Flight Validation", {2, 3, 4, 5, 6}),
}

DEFAULTS = {
    "profile": "Test",
    "common": {
        "wsl_distro": "Ubuntu-22.04",
        "robocommand_repo": str(REPO.parent / "seaowls-robotx-robocommand"),
        "mqtt_port": "1883",   # RoboCommand MQTT broker port (Handbook 3.4.3: 1883)
        "com_port": "COM6",
        "baud": "460800",
        # Drone's static IP on the shared WiFi access point: the router adds
        # WiFi as a second link (elrs_windows_router.py --wifi). Blank = ELRS only.
        "drone_wifi_ip": "",
        "team_id": "FLAU",
        "vehicles": "UAV1",
        "pilot_first_name": "",
        "uapl_id": "",
        "team_code": "FLAU",
        "caas_sticker": "",
        "altitude_limit_hae_m": "40",
        "competition_max_hae_m": "50",
        "fence_inset_m": "1.0",
        "ceiling_margin_m": "1.0",
        "declare_auto": "0",   # "0": the operator sends RunDeclaration with the button
        # Tier attempted per task (RunDeclaration); the OCS's original values.
        "task1_tier": "CORE", "task2_tier": "NONE", "task3_tier": "NONE", "task4_tier": "NONE",
        "task4_vehicle_ack": "0",   # "1": vehicles send their own Task 4 acknowledgement
    },
    "Competition": {"adapter": "", "robocommand_host": ""},
    "Test": {
        "robocommand_host": "localhost",
        "mock_use_gps": "1",
        "mock_center": "",
        "mock_size": "25,25",
        "mock_invalid": "VALID course",
        "bench_home_hae_m": "0",
        "task4_vehicle_type": "USV",   # vehicle type the Task 4 rehearsal addresses
        "mock_no_wait_auto": "1",      # mock RoboCommand: RunStart without waiting for AUTO
    },
}
# RunDeclaration tiers (common.proto TaskTier, without TIER_UNKNOWN).
TIER_NAMES = ("NONE", "CORE", "ADVANCED", "DISRUPTIVE")

# Mock course kinds: GUI label (with the expected outcome) -> the
# mock_robocommand_course.py --invalid value (None = a proper course).
COURSE_KINDS = {
    "VALID course": None,
    "VALID after repair: corners in crossing order": "self-intersecting",
    "VALID + warning: too large (> 1 km)": "too-large",
    "INVALID: open ring (not closed)": "open",
    "INVALID: only two corners": "two-points",
    "INVALID: bad coordinates": "bad-coords",
    "INVALID: too small (1 m inset collapses it)": "too-small",
}
# Labels saved by earlier versions of this window -> current label.
_OLD_KIND_LABELS = {
    "(valid)": "VALID course",
    **{v: k for k, v in COURSE_KINDS.items() if v},
    "INVALID: self-intersecting (edges cross)": "VALID after repair: corners in crossing order",
    "INVALID: too small (< 5 m)": "INVALID: too small (1 m inset collapses it)",
    "INVALID: too large (> 1 km)": "VALID + warning: too large (> 1 km)",
}


# ── helpers ─────────────────────────────────────────────────────────────────

def to_wsl_path(path) -> str:
    p = str(Path(path).resolve())
    return f"/mnt/{p[0].lower()}{p[2:].replace(chr(92), '/')}"


def sh_quote(s: str) -> str:
    return "'" + str(s).replace("'", "'\"'\"'") + "'"


def callsign(c) -> str:
    first, uapl = c["pilot_first_name"].strip(), c["uapl_id"].strip()
    return f"{first}-{uapl}" if uapl else first


def registration(c) -> str:
    sticker = c["caas_sticker"].strip().upper()
    if sticker.startswith("AA"):
        sticker = sticker[2:]
    code = c["team_code"].strip().upper()
    return f"{code}-{sticker}" if code and sticker else ""


def run_powershell(script: str, timeout=20) -> str:
    out = subprocess.run(["powershell.exe", "-NoProfile", "-NonInteractive", "-Command", script],
                         capture_output=True, text=True, timeout=timeout,
                         creationflags=CREATE_NO_WINDOW)
    return out.stdout.strip()


def list_adapters():
    try:
        raw = run_powershell("Get-NetAdapter | Select-Object Name, InterfaceDescription, Status "
                             "| ConvertTo-Json -Compress")
        data = json.loads(raw) if raw else []
        data = [data] if isinstance(data, dict) else data
        return [f"{a['Name']}  [{a['Status']}]  {a['InterfaceDescription']}" for a in data]
    except Exception:
        return []


def adapter_name(entry: str) -> str:
    return entry.split("  [", 1)[0].strip()


def adapter_state(name: str) -> dict:
    n = name.replace("'", "''")
    script = f"""
$n = '{n}'
$ip = Get-NetIPInterface -InterfaceAlias $n -AddressFamily IPv4 -ErrorAction SilentlyContinue
$cfg = Get-NetIPConfiguration -InterfaceAlias $n -ErrorAction SilentlyContinue
$bind = Get-NetAdapterBinding -Name $n -ComponentID ms_bridge -ErrorAction SilentlyContinue
$ics = $null
try {{
  $m = New-Object -ComObject HNetCfg.HNetShare
  foreach ($c in $m.EnumEveryConnection) {{
    $p = $m.NetConnectionProps.Invoke($c)
    if ($p.Name -eq $n) {{ $ics = $m.INetSharingConfigurationForINetConnection.Invoke($c).SharingEnabled }}
  }}
}} catch {{}}
[pscustomobject]@{{
  Found = [bool]$ip
  Dhcp = [string]$ip.Dhcp
  Forwarding = [string]$ip.Forwarding
  Address = @($cfg.IPv4Address | ForEach-Object {{ $_.IPAddress }})
  Gateway = @($cfg.IPv4DefaultGateway | ForEach-Object {{ $_.NextHop }})
  Bridge = [string]$bind.Enabled
  Ics = $ics
}} | ConvertTo-Json -Compress
"""
    raw = run_powershell(script)
    return json.loads(raw) if raw else {"Found": False}


def parse_port(text):
    """A TCP port number from text, or None if it isn't 1-65535."""
    try:
        port = int(str(text).strip())
    except ValueError:
        return None
    return port if 1 <= port <= 65535 else None


def tcp_reachable(host: str, port: int, timeout=3.0) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


# ── managed processes ───────────────────────────────────────────────────────

class Managed:
    """One child process whose output streams into the GUI and a log file."""

    def __init__(self, gui, name, stop_pattern=None):
        self.gui, self.name, self.stop_pattern = gui, name, stop_pattern
        self.proc = None
        self.log_file = None

    @property
    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self, argv, env=None):
        if self.running:
            return
        log_dir = self.gui.run_log_dir()
        self.log_file = open(log_dir / f"{self.name.lower()}.log", "a", encoding="utf-8")
        self.gui.log(self.name, f"$ {' '.join(argv)}\n")
        self.proc = subprocess.Popen(
            argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
            text=True, encoding="utf-8", errors="replace", bufsize=1, env=env,
            creationflags=CREATE_NO_WINDOW,
        )
        threading.Thread(target=self._pump, args=(self.proc, self.log_file), daemon=True).start()

    def _pump(self, proc, log_file):
        for line in proc.stdout:
            log_file.write(line)
            log_file.flush()
            self.gui.lines.put((self.name, line))
        code = proc.wait()
        self.gui.lines.put((self.name, f"--- {self.name} exited (code {code})\n"))
        log_file.close()

    def stop(self):
        if not self.running:
            return
        if self.stop_pattern:   # a WSL process: signal it inside Linux, then the wsl.exe client
            subprocess.run(["wsl.exe", "-d", self.gui.common("wsl_distro"), "--", "pkill", "-INT",
                            "-f", self.stop_pattern], creationflags=CREATE_NO_WINDOW,
                           capture_output=True, timeout=15)
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                subprocess.run(["wsl.exe", "-d", self.gui.common("wsl_distro"), "--", "pkill",
                                "-TERM", "-f", self.stop_pattern], creationflags=CREATE_NO_WINDOW,
                               capture_output=True, timeout=15)
        if self.running:
            self.proc.terminate()


# ── the window ──────────────────────────────────────────────────────────────

class GroundGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Seabird ground station")
        self.geometry("1180x820")
        self.minsize(980, 700)
        self.settings = self._load_settings()
        self.lines = queue.Queue()
        self._log_dir = None
        self.vars = {}
        self.profile = tk.StringVar(value=self.settings["profile"])
        self.status = {}
        self.last_heartbeat = None
        self._pending_preflight = None   # results handed over by the preflight worker
        self.drone_fix = None            # (lat, lon, sats, monotonic time) from Ground's '[pos]' lines
        self.fence_warnings = []         # Ground's '[fence] WARNING' lines for the current course
        self.task4_waiting_ready = []    # (vehicle, AssistanceRequest seq) not yet reported ready
        self.last_delivery_request = None   # (task, resource, circle) from the USV, for the UAV echo
        self.active_run_id = None           # set by a verified RunStart: the OCS mustn't restart now
        self.uav_mode = None                # (PX4 mode, STATE_*, armed) from Ground's '[mode]' lines
        self.drone_fix_lost = False

        self.procs = {
            "Router": Managed(self, "Router"),
            "Broker": Managed(self, "Broker", stop_pattern="mosquitto -p"),
            "OCS": Managed(self, "OCS", stop_pattern="seaowls_ocs"),
            "Ground": Managed(self, "Ground", stop_pattern="beacon_mavlink_ground.py"),
            # Long-running: the mock course stays up as RoboCommand (--serve).
            "Mock": Managed(self, "Mock", stop_pattern="tests/mock_robocommand_course.py"),
            "Declare": Managed(self, "Declare"),   # one-shot: asks the OCS to send RunDeclaration
            "Task4": Managed(self, "Task4"),       # one-shot: ReadinessReport / Task 4 rehearsal
            "Restart": Managed(self, "Restart"),   # one-shot: asks Ground to restart the drone mission
        }
        self._build()
        self._on_profile()
        self.after(100, self._drain)
        self.after(1000, self._tick)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ── settings ──
    def _load_settings(self):
        s = json.loads(json.dumps(DEFAULTS))
        try:
            saved = json.loads(SETTINGS_FILE.read_text(encoding="utf-8"))
            for k in ("common", "Competition", "Test"):
                s[k].update(saved.get(k, {}))
            s["profile"] = saved.get("profile", s["profile"])
        except (OSError, ValueError):
            pass
        return s

    def _save_settings(self):
        for key, var in self.vars.items():
            section, name = key
            self.settings[section][name] = var.get()
        self.settings["profile"] = self.profile.get()
        SETTINGS_FILE.write_text(json.dumps(self.settings, indent=2), encoding="utf-8")

    def common(self, name):
        return self.vars[("common", name)].get()

    def mqtt_port(self):
        """The MQTT port setting, or None when it isn't a valid port number."""
        return parse_port(self.common("mqtt_port"))

    def prof(self, name):
        return self.vars[(self.profile.get(), name)].get()

    def _var(self, section, name):
        v = tk.StringVar(value=self.settings[section].get(name, ""))
        self.vars[(section, name)] = v
        return v

    # ── layout ──
    def _build(self):
        top = ttk.Frame(self, padding=8)
        top.pack(fill="x")
        ttk.Label(top, text="Profile:", font=("Segoe UI", 11, "bold")).pack(side="left")
        for p in ("Competition", "Test"):
            ttk.Radiobutton(top, text=p, value=p, variable=self.profile,
                            command=self._on_profile).pack(side="left", padx=6)
        ttk.Button(top, text="Save settings", command=self._save_settings).pack(side="left", padx=12)
        ttk.Button(top, text="Stop all", command=self.stop_all).pack(side="right")
        ttk.Button(top, text="Start all", command=self.start_all).pack(side="right", padx=6)
        ttk.Button(top, text="Run preflight", command=self.run_preflight).pack(side="right", padx=6)

        body = ttk.Panedwindow(self, orient="horizontal")
        body.pack(fill="both", expand=True, padx=8)
        left = ttk.Frame(body)
        right = ttk.Frame(body)
        body.add(left, weight=1)
        body.add(right, weight=2)

        # settings (common)
        f = ttk.LabelFrame(left, text="Settings (both profiles)", padding=6)
        f.pack(fill="x")
        rows = [
            ("ELRS COM port", "com_port"), ("Baud", "baud"),
            ("Drone WiFi IP (blank = ELRS only)", "drone_wifi_ip"), ("Team ID", "team_id"),
            ("Vehicles (comma)", "vehicles"), ("Pilot first name", "pilot_first_name"),
            ("UAPL ID (blank for Ground Validation)", "uapl_id"),
            ("Team code", "team_code"), ("CAAS sticker number", "caas_sticker"),
            ("Altitude limit (m HAE)", "altitude_limit_hae_m"),
            ("Competition maximum (m HAE)", "competition_max_hae_m"),
            ("Fence inset (m)", "fence_inset_m"), ("Ceiling margin (m)", "ceiling_margin_m"),
            ("WSL distro", "wsl_distro"), ("RoboCommand repo", "robocommand_repo"),
            ("MQTT port", "mqtt_port"),
        ]
        for i, (label, name) in enumerate(rows):
            ttk.Label(f, text=label).grid(row=i, column=0, sticky="w")
            ttk.Entry(f, textvariable=self._var("common", name), width=30).grid(
                row=i, column=1, sticky="ew", pady=1)
        f.columnconfigure(1, weight=1)
        self.derived = ttk.Label(f, foreground="#555")
        self.derived.grid(row=len(rows), column=0, columnspan=2, sticky="w", pady=(4, 0))
        ttk.Checkbutton(f, text="Send RunDeclaration automatically when the UAV fence is VALID\n"
                                "(unticked: send it with the button on the Declaration row)",
                        variable=self._var("common", "declare_auto"), onvalue="1", offvalue="0").grid(
            row=len(rows) + 1, column=0, columnspan=2, sticky="w", pady=(4, 0))
        tiers = ttk.Frame(f)
        tiers.grid(row=len(rows) + 2, column=0, columnspan=2, sticky="w", pady=(4, 0))
        ttk.Label(tiers, text="Tiers declared:").pack(side="left")
        for t in (1, 2, 3, 4):
            ttk.Label(tiers, text=f"  T{t}").pack(side="left")
            ttk.Combobox(tiers, textvariable=self._var("common", f"task{t}_tier"), values=TIER_NAMES,
                         state="readonly", width=10).pack(side="left")
        ttk.Checkbutton(f, text="Task 4: vehicles send their own IncidentAck\n"
                                "(unticked: the OCS acknowledges as soon as it routes the command)",
                        variable=self._var("common", "task4_vehicle_ack"), onvalue="1", offvalue="0").grid(
            row=len(rows) + 3, column=0, columnspan=2, sticky="w", pady=(4, 0))
        for name in ("pilot_first_name", "uapl_id", "team_code", "caas_sticker"):
            self.vars[("common", name)].trace_add("write", lambda *_: self._update_derived())
        self._update_derived()

        # Competition settings
        self.comp_frame = ttk.LabelFrame(left, text="Competition", padding=6)
        ttk.Label(self.comp_frame, text="RoboCommand-facing adapter").grid(row=0, column=0, sticky="w")
        self.adapter_box = ttk.Combobox(self.comp_frame, textvariable=self._var("Competition", "adapter"),
                                        width=34)
        self.adapter_box.grid(row=0, column=1, sticky="ew")
        ttk.Button(self.comp_frame, text="↻", width=3, command=self._refresh_adapters).grid(row=0, column=2)
        ttk.Label(self.comp_frame, text="RoboCommand host (blank = DHCP gateway)").grid(row=1, column=0, sticky="w")
        ttk.Entry(self.comp_frame, textvariable=self._var("Competition", "robocommand_host")).grid(
            row=1, column=1, columnspan=2, sticky="ew")
        self.comp_frame.columnconfigure(1, weight=1)

        # Test settings
        self.test_frame = ttk.LabelFrame(left, text="Test (local broker + mock course)", padding=6)
        ttk.Checkbutton(self.test_frame, text="Centre the course on the drone's GPS position",
                        variable=self._var("Test", "mock_use_gps"), onvalue="1", offvalue="0",
                        command=self._on_use_gps).grid(row=0, column=0, columnspan=2, sticky="w")
        ttk.Label(self.test_frame, text="Drone position").grid(row=1, column=0, sticky="w")
        self.drone_pos_label = tk.Label(self.test_frame, text="—", anchor="w")
        self.drone_pos_label.grid(row=1, column=1, sticky="w")
        ttk.Label(self.test_frame, text="Manual centre LAT,LON\n(used when there's no GPS fix)").grid(
            row=2, column=0, sticky="w")
        self.manual_center = ttk.Entry(self.test_frame, textvariable=self._var("Test", "mock_center"))
        self.manual_center.grid(row=2, column=1, sticky="ew")
        ttk.Label(self.test_frame, text="Course size W,L (m)").grid(row=3, column=0, sticky="w")
        ttk.Entry(self.test_frame, textvariable=self._var("Test", "mock_size")).grid(row=3, column=1, sticky="ew")
        ttk.Label(self.test_frame, text="Course kind (VALID / INVALID demo)").grid(row=4, column=0, sticky="w")
        kind_var = self._var("Test", "mock_invalid")
        kind_var.set(_OLD_KIND_LABELS.get(kind_var.get(), kind_var.get()))
        if kind_var.get() not in COURSE_KINDS:
            kind_var.set("VALID course")
        ttk.Combobox(self.test_frame, textvariable=kind_var, values=list(COURSE_KINDS),
                     state="readonly", width=36).grid(row=4, column=1, sticky="ew")
        ttk.Label(self.test_frame, text="Home HAE when there's no GPS fix (m)\n(blank = 0)").grid(
            row=5, column=0, sticky="w")
        ttk.Entry(self.test_frame, textvariable=self._var("Test", "bench_home_hae_m")).grid(
            row=5, column=1, sticky="ew")
        b = ttk.Frame(self.test_frame)
        b.grid(row=6, column=0, columnspan=2, sticky="w", pady=(4, 0))
        ttk.Button(b, text="Publish mock course", command=self.publish_mock).pack(side="left")
        ttk.Button(b, text="Clear mock course", command=self.clear_mock).pack(side="left", padx=6)
        ttk.Checkbutton(self.test_frame,
                        text="Mock RoboCommand sends RunStart without waiting for AUTO (props-off bench)",
                        variable=self._var("Test", "mock_no_wait_auto"), onvalue="1", offvalue="0").grid(
            row=8, column=0, columnspan=2, sticky="w", pady=(2, 0))
        t4 = ttk.Frame(self.test_frame)
        t4.grid(row=7, column=0, columnspan=2, sticky="w", pady=(4, 0))
        ttk.Button(t4, text="Rehearse Task 4", command=self.rehearse_task4).pack(side="left")
        ttk.Label(t4, text=" addressing ").pack(side="left")
        ttk.Combobox(t4, textvariable=self._var("Test", "task4_vehicle_type"), values=("USV", "UUV", "UAV"),
                     state="readonly", width=5).pack(side="left")
        ttk.Label(t4, text=" (Core, Advanced, Disruptive)").pack(side="left")
        self.test_frame.columnconfigure(1, weight=1)
        self._var("Test", "robocommand_host")
        self._on_use_gps()

        # status
        st = ttk.LabelFrame(right, text="Status", padding=6)
        st.pack(fill="x")
        for i, key in enumerate(("Router", "Broker", "OCS", "Ground", "Heartbeats", "Geofence",
                                 "Declaration", "RunStart", "Commands", "Task 4", "Reports",
                                 "Drone mission")):
            ttk.Label(st, text=key + ":", width=12).grid(row=i, column=0, sticky="w")
            lbl = tk.Label(st, text="—", anchor="w", justify="left", wraplength=640)
            lbl.grid(row=i, column=1, sticky="w")
            self.status[key] = lbl
            if key in self.procs:
                bf = ttk.Frame(st)
                bf.grid(row=i, column=2, sticky="e")
                ttk.Button(bf, text="Start", width=6, command=lambda k=key: self.start_one(k)).pack(side="left")
                ttk.Button(bf, text="Stop", width=6, command=lambda k=key: self.stop_one(k)).pack(side="left")
            elif key == "Declaration":
                ttk.Button(st, text="Send RunDeclaration", command=self.send_declaration).grid(
                    row=i, column=2, sticky="e")
            elif key == "Task 4":
                ttk.Button(st, text="Send ReadinessReport", command=self.send_readiness).grid(
                    row=i, column=2, sticky="e")
            elif key == "Reports":
                ttk.Button(st, text="Echo delivery request (UAV)", command=self.echo_delivery).grid(
                    row=i, column=2, sticky="e")
            elif key == "Drone mission":
                ttk.Button(st, text="Restart drone mission", command=self.restart_drone_mission).grid(
                    row=i, column=2, sticky="e")
        st.columnconfigure(1, weight=1)

        # preflight
        pf = ttk.LabelFrame(right, text="Preflight", padding=6)
        pf.pack(fill="x", pady=6)
        self.pre = ttk.Treeview(pf, columns=("result", "detail"), show="tree headings", height=9)
        self.pre.heading("#0", text="Check")
        self.pre.heading("result", text="Result")
        self.pre.heading("detail", text="Detail")
        self.pre.column("#0", width=210)
        self.pre.column("result", width=70, anchor="center")
        self.pre.column("detail", width=470)
        self.pre.tag_configure("ok", foreground="#1b7f1b")
        self.pre.tag_configure("warn", foreground="#a86b00")
        self.pre.tag_configure("bad", foreground="#c0182b")
        self.pre.pack(fill="x")

        # logs
        self.logs = ttk.Notebook(self)
        self.logs.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self.log_text = {}
        for name in self.procs:
            frame = ttk.Frame(self.logs)
            text = tk.Text(frame, height=12, wrap="none", font=("Consolas", 9))
            sb = ttk.Scrollbar(frame, command=text.yview)
            text.configure(yscrollcommand=sb.set)
            sb.pack(side="right", fill="y")
            text.pack(fill="both", expand=True)
            self.logs.add(frame, text=name)
            self.log_text[name] = text

    def _on_profile(self):
        comp = self.profile.get() == "Competition"
        self.comp_frame.pack_forget()
        self.test_frame.pack_forget()
        (self.comp_frame if comp else self.test_frame).pack(fill="x", pady=6)
        if comp and not self.adapter_box["values"]:
            self._refresh_adapters()
        self.status["Broker"].config(text="not used in Competition" if comp else "—", fg="#555")

    def _on_use_gps(self):
        # The manual centre stays editable either way: it's the no-fix fallback.
        self.manual_center.config(state="normal")

    # A fix older than this isn't used to centre the course.
    FIX_MAX_AGE_S = 30.0

    def no_fix_home_hae(self) -> float:
        """Test profile: home height (m HAE) the round trip uses until the UAV
        has a GPS fix. Blank or not a number -> 0, so it never stalls indoors."""
        try:
            return float(self.vars[("Test", "bench_home_hae_m")].get().strip() or 0)
        except ValueError:
            return 0.0

    def drone_position(self):
        """(lat, lon) of a 3D fix seen in the last FIX_MAX_AGE_S, else None."""
        if self.drone_fix is None or self.drone_fix_lost:
            return None
        lat, lon, _, t = self.drone_fix
        return (lat, lon) if time.monotonic() - t <= self.FIX_MAX_AGE_S else None

    def _refresh_adapters(self):
        self.adapter_box["values"] = list_adapters()

    def _update_derived(self):
        c = {k[1]: v.get() for k, v in self.vars.items() if k[0] == "common"}
        self.derived.config(text=f"uav_pilot_callsign = {callsign(c)!r}    "
                                 f"uav_registration = {registration(c)!r}")

    # ── logs and status ──
    def run_log_dir(self) -> Path:
        if self._log_dir is None:
            self._log_dir = LOG_ROOT / datetime.now().strftime("%Y%m%d_%H%M%S")
            self._log_dir.mkdir(parents=True, exist_ok=True)
        return self._log_dir

    def log(self, name, text):
        self.lines.put((name, text))

    def _drain(self):
        try:
            for _ in range(500):
                name, line = self.lines.get_nowait()
                t = self.log_text[name]
                t.insert("end", line)
                if int(t.index("end-1c").split(".")[0]) > 4000:
                    t.delete("1.0", "1000.0")
                t.see("end")
                self._parse(name, line)
        except queue.Empty:
            pass
        if self._pending_preflight is not None:
            results, self._pending_preflight = self._pending_preflight, None
            self._show_preflight(results)
        self.after(100, self._drain)

    def _set(self, key, text, color="#222"):
        self.status[key].config(text=text, fg=color)

    def _parse(self, name, line):
        if name == "Router":
            m = re.search(r"serial_bytes=(\d+)", line)
            w = re.search(r"wifi_frames=(\d+).*uplink=(\w+)", line)
            if m and w:
                # WiFi redundancy on: OK while either link delivers.
                elrs, wifi, up = int(m.group(1)) > 0, int(w.group(1)) > 0, w.group(2)
                self._set("Router", f"ELRS: {'OK' if elrs else 'NOTHING'} ({m.group(1)} B)  |  "
                          f"WiFi: {'OK' if wifi else 'NOTHING'} ({w.group(1)} frames)  |  "
                          f"uplink: {up.upper()}",
                          "#1b7f1b" if elrs and wifi else "#a86b00" if elrs or wifi else "#c0182b")
            elif m:
                ok = int(m.group(1)) > 0
                self._set("Router", f"radio bytes in last interval: {m.group(1)}"
                          + ("" if ok else "  — nothing from the radio: wrong COM port?"),
                          "#1b7f1b" if ok else "#c0182b")
        elif name == "Ground":
            w = re.search(r"\[fence\] WARNING course '([^']*)': (.*)", line)
            m = re.search(r"\[fence\] (UPLOADING|WAITING|VALID|REFUSED|FAILED) course '([^']*)'(?:: (.*))?",
                          line)
            if w:
                self.fence_warnings.append(w.group(2))
            elif m:
                state, course, why = m.groups()
                color = {"VALID": "#1b7f1b", "UPLOADING": "#a86b00", "WAITING": "#a86b00"}.get(state, "#c0182b")
                label = "WAITING for GPS" if state == "WAITING" else state
                notes = "".join(f"\n⚠ {x}" for x in self.fence_warnings)
                self._set("Geofence", f"{label} — course {course}" + (f": {why}" if why else "")
                          + ("  (on the UAV, read back)" if state == "VALID" else "") + notes, color)
                if state in ("VALID", "REFUSED", "FAILED"):
                    self.fence_warnings = []   # the next course starts fresh
            elif "ceiling HAE uses the ASSUMED home" in line:
                self.status["Geofence"].config(
                    text=self.status["Geofence"].cget("text") + "  [ASSUMED home HAE: bench only]")
            elif "Heartbeat received from system" in line:
                self._set("Ground", "receiving UAV MAVLink", "#1b7f1b")
            m = re.search(r"\[mode\] (\S+) (\S+) -> (STATE_\w+) \((armed|disarmed)\)", line)
            if m:
                self.uav_mode = m.group(2, 3, 4)   # PX4 mode, RoboCommand state, armed
            # RunStart relay to the UAV (Handbook 3.4.10: the UAV holds until it arrives)
            m = re.search(r"\[runstart\] RunStart run_id=(\S+) verified by the OCS", line)
            if m:
                self._set("RunStart", f"verified: run_id {m.group(1)} — relaying to the UAV…", "#a86b00")
            m = re.search(r"\[runstart\] UAV acknowledged RunStart run_id=(\S+)", line)
            if m:
                self._set("RunStart", f"verified: run_id {m.group(1)} — UAV acknowledged, the run "
                                      f"may begin", "#1b7f1b")
            m = re.search(r"\[runstart\] still no acknowledgement from the UAV for run_id=(\S+) \((\d+) sends\)",
                          line)
            if m:
                self._set("RunStart", f"verified: run_id {m.group(1)} — NO acknowledgement from the UAV "
                                      f"yet ({m.group(2)} sends): check the ELRS link and "
                                      f"beacon_mavlink_bridge.py on the drone", "#c0182b")
            # Remote restart of the drone's seabird-mission (over ELRS)
            m = re.search(r"\[restart\] (.*)", line)
            if m:
                text = m.group(1)
                if text.startswith("request "):
                    self._set("Drone mission", "restart sent over ELRS — waiting for the drone…", "#a86b00")
                elif text.startswith("drone: OK"):
                    self._set("Drone mission", "drone: OK — seabird-mission restarting; it waits for "
                                               "the CH7 trigger again", "#1b7f1b")
                elif text.startswith("drone: "):
                    self._set("Drone mission", f"drone {text[len('drone: '):]}", "#c0182b")
                elif text.startswith("REFUSED"):
                    self._set("Drone mission", f"ground {text}", "#c0182b")
                elif text.startswith("no acknowledgement"):
                    self._set("Drone mission", text, "#c0182b")
            m = re.search(r"\[pos\] 3D fix lat=(-?[\d.]+) lon=(-?[\d.]+) sats=(\d+)", line)
            if m:
                self.drone_fix = (float(m.group(1)), float(m.group(2)), int(m.group(3)), time.monotonic())
                self.drone_fix_lost = False
            elif "[pos] no GPS fix" in line:
                self.drone_fix_lost = True
        elif name == "OCS":
            if re.search(r"UAV1 heartbeat seq=\d+", line):
                self.last_heartbeat = time.monotonic()
            m = re.search(r"RunDeclaration seq=(\d+).*course=(\S+)(.*)", line)
            if m:
                self._set("Declaration", f"sent seq {m.group(1)}, course {m.group(2)}{m.group(3)}",
                          "#1b7f1b")
            m = re.search(r"(REFUSED UAV1 geofence.*|RunDeclaration NOT sent.*|RunDeclaration waiting .*)", line)
            if m:
                bad = not m.group(1).startswith("RunDeclaration waiting")
                self._set("Declaration", m.group(1), "#c0182b" if bad else "#a86b00")
            m = re.search(r"RunDeclaration READY for course (\S+)(.*?) -- manual mode", line)
            if m:
                self._set("Declaration", f"READY — course {m.group(1)}{m.group(2)}: press "
                                         f"Send RunDeclaration when you're ready to go", "#a86b00")
            m = re.search(r"RunStart declaration_seq=(\d+) run_id=(\S+)", line)
            if m:
                self.active_run_id = m.group(2)   # guards against restarting the OCS mid-run
                self._set("RunStart", f"received and verified: declaration_seq {m.group(1)}, "
                                      f"run_id {m.group(2)}", "#1b7f1b")
            m = re.search(r"RunStart IGNORED: (.*)", line)
            if m:
                self._set("RunStart", f"IGNORED (not for our declaration): {m.group(1)}", "#c0182b")
            # 3.4.13: command receipt and the reports we send, visible to the operator.
            now = time.strftime("%H:%M:%S")
            m = re.search(r"RxCommand seq=(\d+) body=(\S+)", line)
            if m:
                self._set("Commands", f"last received: {m.group(2)} (command seq {m.group(1)}) at {now}",
                          "#222")
            m = re.search(r"ignored command for team (\S+)", line)
            if m:
                self._set("Commands", f"ignored a command addressed to team {m.group(1)} at {now}",
                          "#a86b00")
            m = re.search(r"invalid RxCommand: (.*)", line)
            if m:
                self._set("Commands", f"malformed RxCommand rejected at {now}: {m.group(1)}", "#c0182b")
            m = re.search(r"report (\S+) seq=(\d+) (\w+)", line)
            if m:
                self._set("Reports", f"last sent: {m.group(3)} as {m.group(1)} (report seq {m.group(2)}) "
                                     f"at {now}", "#1b7f1b")
            # Tasks 2/3 resource delivery and Task 2 pipeline survey (3.4.12)
            m = re.search(r"\[resource\] request from (\S+) task=(\d+) resource=(\d+) circle=(\d+)", line)
            if m:
                self.last_delivery_request = tuple(int(x) for x in m.group(2, 3, 4))
                self._set("Reports", f"ResourceDeliveryRequest from {m.group(1)} (task {m.group(2)}) at {now}; "
                                     f"press Echo when the UAV begins the delivery", "#a86b00")
            m = re.search(r"\[(resource|pipeline)\] (.* not sent: .*)", line)
            if m:
                self._set("Reports", f"{m.group(1)}: {m.group(2)}", "#c0182b")
            # Task 4 command-response chains (3.4.12)
            m = re.search(r"\[task4\] (\w+) seq=(\d+) -> (\S+)(.*)", line)
            if m:
                name, seq, v, where = m.groups()
                if name == "AssistanceRequest":
                    self.task4_waiting_ready.append((v, seq))
                    nxt = "IncidentAck, then ReadinessReport when on station"
                elif name == "MovingObjectAlert":
                    nxt = "no acknowledgement in this chain"
                else:
                    nxt = "IncidentAck"
                self._set("Task 4", f"{name} seq {seq} → {v}{where} at {now}; next: {nxt}", "#a86b00")
            m = re.search(r"\[task4\] IncidentAck (\S+) command_seq=(\d+) \(report seq=(\d+)\)", line)
            if m:
                self._set("Task 4", f"IncidentAck sent as {m.group(1)} for command seq {m.group(2)} "
                                    f"(report seq {m.group(3)}) at {now}", "#1b7f1b")
            m = re.search(r"\[task4\] waiting for (\S+) to acknowledge seq=(\d+)", line)
            if m:
                self._set("Task 4", f"waiting for {m.group(1)} to acknowledge command seq {m.group(2)}",
                          "#a86b00")
            m = re.search(r"\[task4\] ReadinessReport (\S+) for AssistanceRequest seq=(\d+) \(report seq=(\d+)\)",
                          line)
            if m:
                if (m.group(1), m.group(2)) in self.task4_waiting_ready:
                    self.task4_waiting_ready.remove((m.group(1), m.group(2)))
                self._set("Task 4", f"ReadinessReport sent as {m.group(1)} (report seq {m.group(3)}) "
                                    f"at {now}; waiting for ReadinessConfirm", "#a86b00")
            m = re.search(r"\[task4\] ReadinessConfirm (\S+) report seq=(\d+)", line)
            if m:
                self._set("Task 4", f"ReadinessConfirm for {m.group(1)} (report seq {m.group(2)}) at {now}: "
                                    f"cleared to resume — chain complete", "#1b7f1b")
            m = re.search(r"\[task4\] (.*(?:not routed|doesn't match|isn't an AssistanceRequest).*)", line)
            if m:
                self._set("Task 4", m.group(1), "#c0182b")
            if "MQTT connected" in line:
                self._set("OCS", "connected to RoboCommand broker", "#1b7f1b")
            elif "MQTT disconnected" in line:
                self._set("OCS", "MQTT disconnected — reconnecting", "#c0182b")

    def _tick(self):
        for key, p in self.procs.items():
            if key in ("Mock", "Declare", "Task4", "Restart") or (key == "Broker" and self.profile.get() == "Competition"):
                continue
            text = self.status[key].cget("text")
            if p.running:
                if text in ("—", "stopped"):
                    self._set(key, "running", "#a86b00")
            elif text not in ("—", "stopped") and not text.startswith("already running"):
                self._set(key, "stopped", "#555")
        if self.drone_fix is None:
            pos_text, pos_color = ("no position yet (needs Ground running and a GPS fix)"
                                   if not self.drone_fix_lost else "no GPS fix"), "#a86b00"
        else:
            lat, lon, sats, t = self.drone_fix
            age = time.monotonic() - t
            if self.drone_fix_lost:
                pos_text, pos_color = f"GPS fix lost (last {lat:.7f}, {lon:.7f})", "#c0182b"
            elif age > self.FIX_MAX_AGE_S:
                pos_text, pos_color = f"{lat:.7f}, {lon:.7f} — stale ({age:.0f} s old)", "#c0182b"
            else:
                pos_text, pos_color = f"{lat:.7f}, {lon:.7f}  (3D fix, {sats} sats, {age:.0f} s ago)", "#1b7f1b"
        self.drone_pos_label.config(text=pos_text, fg=pos_color)
        if self.last_heartbeat is not None:
            age = time.monotonic() - self.last_heartbeat
            if age < 2.0:
                text, color = "UAV1 → RoboCommand at ~2 Hz", "#1b7f1b"
                if self.uav_mode:
                    mode, state, armed = self.uav_mode
                    label = {"STATE_AUTO": "AUTONOMOUS", "STATE_MANUAL": "manual",
                             "STATE_KILLED": "KILLED"}.get(state, state)
                    text += f" — {mode}, {label} ({armed})"
                    color = {"STATE_AUTO": "#1b7f1b", "STATE_KILLED": "#c0182b"}.get(state, "#a86b00")
                self._set("Heartbeats", text, color)
            else:
                self._set("Heartbeats", f"none for {age:.0f} s (OCS suppresses stale UAV telemetry)",
                          "#c0182b")
        self.after(1000, self._tick)

    # ── preflight ──
    def run_preflight(self):
        self._save_settings()
        for i in self.pre.get_children():
            self.pre.delete(i)
        self.pre.insert("", "end", iid="running", text="running…", values=("", ""))
        # Snapshot on the GUI thread: tkinter variables must not be read from the worker.
        snap = {k[1]: v.get() for k, v in self.vars.items() if k[0] == "common"}
        snap.update(profile=self.profile.get(),
                    adapter=self.vars[("Competition", "adapter")].get(),
                    rc_host=self.vars[("Competition", "robocommand_host")].get().strip(),
                    mock_center=self.vars[("Test", "mock_center")].get().strip(),
                    mock_use_gps=self.vars[("Test", "mock_use_gps")].get() == "1",
                    drone_position=self.drone_position(),
                    bench_home=self.vars[("Test", "bench_home_hae_m")].get().strip())
        threading.Thread(target=self._preflight_worker, args=(snap,), daemon=True).start()

    def _preflight_worker(self, c):
        results = []

        def add(check, level, detail):
            results.append((check, level, detail))

        comp = c["profile"] == "Competition"

        # Latest protobuf
        vendor = Path(c["robocommand_repo"]) / "vendor" / "robocommand"
        try:
            local = subprocess.run(["git", "-C", str(vendor), "rev-parse", "HEAD"], capture_output=True,
                                   text=True, timeout=10, creationflags=CREATE_NO_WINDOW).stdout.strip()
        except (OSError, subprocess.TimeoutExpired):
            local = ""
        if not local:
            add("RoboCommand protobuf", "bad", f"can't read the vendored repo at {vendor}")
        else:
            try:
                remote = subprocess.run(["git", "ls-remote", ROBOCOMMAND_UPSTREAM, "HEAD"],
                                        capture_output=True, text=True, timeout=12,
                                        creationflags=CREATE_NO_WINDOW).stdout.split()
            except (OSError, subprocess.TimeoutExpired):
                remote = []
            if not remote:
                add("RoboCommand protobuf", "warn", f"local {local[:7]}; GitHub unreachable "
                    "(course networks have no Internet) — confirm it's latest before leaving the pits")
            elif remote[0] == local:
                add("RoboCommand protobuf", "ok", f"{local[:7]} = latest robonation/robocommand")
            else:
                add("RoboCommand protobuf", "bad", f"local {local[:7]} but upstream is {remote[0][:7]}: "
                    f"update vendor/robocommand and rebuild")

        # Identity
        cs = callsign(c)
        if CALLSIGN_RE.match(cs):
            add("uav_pilot_callsign", "ok", cs + ("" if c["uapl_id"].strip()
                                                   else "  (no UAPL ID: Ground Validation form)"))
        else:
            add("uav_pilot_callsign", "bad", f"{cs!r}: needs '<first name>' or '<first name>-<UAPL ID>'")
        reg = registration(c)
        if not reg:
            add("uav_registration", "warn", "empty: heartbeats go without uav_registration")
        elif REGISTRATION_RE.match(reg):
            add("uav_registration", "ok", reg)
        else:
            add("uav_registration", "bad", f"{reg!r}: needs '<team code>-<CAAS number>' (no AA)")

        # Altitude
        try:
            problems = validate_altitude(float(c["altitude_limit_hae_m"]), float(c["competition_max_hae_m"]))
        except ValueError:
            problems = ["altitude fields must be numbers"]
        add("Altitude limit", "bad" if problems else "ok",
            "; ".join(problems) + ("  (OK only for the INVALID demonstration)" if problems else "")
            if problems else f"{c['altitude_limit_hae_m']} m HAE ≤ {c['competition_max_hae_m']} m HAE; "
                            f"PX4 ceiling {float(c['altitude_limit_hae_m']) - float(c['ceiling_margin_m']):g} m HAE")

        # Declared tiers
        tiers = {t: (c.get(f"task{t}_tier") or "NONE") for t in (1, 2, 3, 4)}
        text = ", ".join(f"T{t} {name}" for t, name in tiers.items())
        vehicles = {v.strip() for v in c.get("vehicles", "").split(",") if v.strip()}
        if all(name == "NONE" for name in tiers.values()):
            add("Declared tiers", "warn", f"{text}: no task attempted")
        elif "UAV1" in vehicles and tiers[1] == "CORE":
            add("Declared tiers", "warn", f"{text}: Task 1 by the UAV is Advanced/Disruptive "
                "(RoboCommand: SafePassage is sent by USV/UUV at Core, UAV at Advanced/Disruptive)")
        else:
            add("Declared tiers", "ok", text)

        # Clocks (3.4.8: sent_at must be the publish time, OCS clock synchronised).
        # The OCS runs in WSL, whose clock can drift from Windows' after sleep.
        try:
            t_before = time.time()
            out = subprocess.run(["wsl.exe", "-d", c["wsl_distro"], "--", "date", "+%s.%N"],
                                 capture_output=True, text=True, timeout=30,
                                 creationflags=CREATE_NO_WINDOW).stdout.strip()
            t_after = time.time()
            skew = float(out) - (t_before + t_after) / 2
            margin = (t_after - t_before) / 2   # the call's own duration bounds the error
            ok = abs(skew) <= 1.0 + margin
            add("WSL clock", "ok" if ok else "bad",
                f"WSL is {skew:+.2f} s from Windows" + ("" if ok else
                " — sent_at timestamps will be off: run 'wsl --shutdown' and restart, or "
                "'sudo hwclock -s' in WSL"))
        except (OSError, ValueError, subprocess.TimeoutExpired) as e:
            add("WSL clock", "warn", f"couldn't read the WSL clock: {e}")
        try:
            status = subprocess.run(["w32tm", "/query", "/status"], capture_output=True, text=True,
                                    timeout=10, creationflags=CREATE_NO_WINDOW).stdout
            last = re.search(r"Last Successful Sync Time:\s*(.+)", status)
            if "has not been started" in status or "0x80070426" in status:
                add("Windows clock sync", "bad", "the Windows Time service isn't running, so the clock "
                    "can't sync: Settings → Time & language → Set time automatically → Sync now "
                    "(or admin PowerShell: Start-Service w32time; w32tm /resync)")
            elif last and "unspecified" not in last.group(1).lower():
                add("Windows clock sync", "ok", f"last synced {last.group(1).strip()} — "
                    f"sync before leaving for the course (course networks have no Internet)")
            else:
                add("Windows clock sync", "warn", "no successful sync recorded — sync the clock "
                    "(Settings → Time → Sync now) before the course")
        except (OSError, subprocess.TimeoutExpired):
            add("Windows clock sync", "warn", "couldn't query w32tm — sync the clock before the course")

        # WSL mirrored networking
        try:
            wslconfig = (Path.home() / ".wslconfig").read_text(encoding="utf-8", errors="replace")
        except OSError:
            wslconfig = ""
        mirrored = re.search(r"(?im)^\s*networkingMode\s*=\s*mirrored", wslconfig)
        add("WSL networking", "ok" if mirrored else "bad",
            "mirrored (WSL uses Windows' adapters)" if mirrored
            else "not mirrored: OCS in WSL can't reach the course network / router uplink")

        host = c["rc_host"] if comp else "localhost"
        port = parse_port(c.get("mqtt_port", "1883"))
        if port is None:
            add("MQTT port", "bad", f"{c.get('mqtt_port')!r} isn't a port number (1-65535)")
        if comp:
            name = adapter_name(c["adapter"])
            if not name:
                add("RoboCommand adapter", "bad", "choose the Ethernet adapter on the provided RJ-45 cable")
            else:
                try:
                    s = adapter_state(name)
                except Exception as e:
                    s = {"Found": False, "error": str(e)}
                if not s.get("Found"):
                    add("RoboCommand adapter", "bad", f"{name}: not found / no IPv4 ({s.get('error', '')})")
                else:
                    add("DHCP", "ok" if s["Dhcp"] == "Enabled" else "bad",
                        f"{name}: DHCP {s['Dhcp']}" + ("" if s["Dhcp"] == "Enabled"
                                                       else " — must be DHCP, no static address (3.4.3)"))
                    addrs = [a for a in s.get("Address") or [] if a]
                    gws = [g for g in s.get("Gateway") or [] if g and g != "0.0.0.0"]
                    net = None
                    for a in addrs:
                        o = a.split(".")
                        if len(o) == 4 and o[0] == "192" and o[1] == "168" and int(o[2]) in COURSE_NETWORKS:
                            net = int(o[2])
                            in_range = 15 <= int(o[3]) <= 254
                            add("Address", "ok" if in_range else "bad",
                                f"{a} on {COURSE_NETWORKS[net][0]} (192.168.{net}.0/24)"
                                + ("" if in_range else " — outside the DHCP range .15–.254"))
                            break
                    if net is None:
                        add("Address", "warn" if addrs else "bad",
                            f"{', '.join(addrs) or 'none'}: not a known course subnet "
                            f"(Team Village demo network, or not connected?)")
                    if gws:
                        g = gws[0]
                        ok = net is not None and g.startswith(f"192.168.{net}.") and \
                            int(g.split(".")[3]) in COURSE_NETWORKS[net][1]
                        add("Default gateway", "ok" if ok else "warn",
                            f"{g}" + (" = course RoboCommand address" if ok else
                                      " — expected the course RoboCommand address (3.4.3 step 7)"))
                        host = host or g
                    else:
                        add("Default gateway", "bad", "none from DHCP")
                    add("Bridging", "ok" if s["Bridge"] != "True" else "bad",
                        "not bridged" if s["Bridge"] != "True" else "network bridge enabled — disable (3.4.3)")
                    add("Internet sharing", "ok" if not s.get("Ics") else "bad",
                        "ICS off" if not s.get("Ics") else "ICS enabled — disable (3.4.3)")
                    add("IP forwarding", "ok" if s["Forwarding"] != "Enabled" else "bad",
                        "off" if s["Forwarding"] != "Enabled"
                        else "on — other networks must not be routed through this interface")
            if port and port != 1883:
                add("MQTT port", "warn", f"{port} — Handbook 3.4.3 says RoboCommand uses 1883")
            if host and port:
                reachable = tcp_reachable(host, port)
                add("RoboCommand MQTT", "ok" if reachable else "bad",
                    f"{host}:{port} " + ("reachable" if reachable
                                         else "NOT reachable (cable, subnet, firewall, port?)"))
        elif port:
            listening = tcp_reachable("127.0.0.1", port)
            add("Local broker", "ok" if listening else "warn",
                f"localhost:{port} listening" if listening
                else "not running yet — Start all starts Mosquitto in WSL")
            p = c["drone_position"] if c["mock_use_gps"] else None
            if p:
                add("Mock course centre", "ok", f"drone GPS {p[0]:.7f}, {p[1]:.7f}")
            elif c["mock_center"]:
                add("Mock course centre", "warn", f"no GPS fix: using the manual centre {c['mock_center']}")
            else:
                add("Mock course centre", "bad", "no GPS fix and no manual centre LAT,LON")
            try:
                home = float(c["bench_home"] or 0)
                add("No-fix home HAE", "ok" if p else "warn",
                    f"{home:g} m HAE{' (blank = 0)' if not c['bench_home'] else ''}, used until the UAV "
                    f"reports a fix" + ("" if p else " — the declared ceiling is only as right as this value"))
            except ValueError:
                add("No-fix home HAE", "bad", f"{c['bench_home']!r} is not a number (0 will be used)")

        self._pending_preflight = results   # picked up by _drain() on the GUI thread

    def _show_preflight(self, results):
        for i in self.pre.get_children():
            self.pre.delete(i)
        marks = {"ok": "✓ OK", "warn": "⚠ WARN", "bad": "✗ FAIL"}
        for check, level, detail in results:
            self.pre.insert("", "end", text=check, values=(marks[level], detail), tags=(level,))
        self.preflight_failures = [r for r in results if r[1] == "bad"]

    # ── starting things ──
    def _wsl(self, name, script_body):
        """Run a bash script in WSL with the user's ~/.bashrc (pymavlink, ROS env)."""
        scripts = self.run_log_dir() / "scripts"
        scripts.mkdir(exist_ok=True)
        path = scripts / f"{name.lower()}.sh"
        path.write_text(script_body.replace("\r\n", "\n"), encoding="utf-8", newline="\n")
        return ["wsl.exe", "-d", self.common("wsl_distro"), "--", "bash", "-ic",
                f"bash {sh_quote(to_wsl_path(path))}"]

    def _ros_prelude(self):
        rc = to_wsl_path(self.common("robocommand_repo"))
        return (f"source /opt/ros/humble/setup.bash\n"
                f"source {sh_quote(rc + '/ros2_ws/install/setup.bash')}\n"
                f"export PYTHONUNBUFFERED=1 RCUTILS_LOGGING_BUFFERED_STREAM=0\n"
                # Background: it can take ~20 s here and nodes don't need it to talk.
                f"(ros2 daemon start >/dev/null 2>&1 &)\n")

    def _robocommand_host(self):
        if self.profile.get() == "Test":
            return "localhost"
        host = self.vars[("Competition", "robocommand_host")].get().strip()
        if host:
            return host
        name = adapter_name(self.vars[("Competition", "adapter")].get())
        try:
            gws = [g for g in adapter_state(name).get("Gateway") or [] if g and g != "0.0.0.0"]
        except Exception:
            gws = []
        return gws[0] if gws else ""

    def _leftovers_ok(self, key) -> bool:
        """Before starting Ground / OCS: a copy already running in WSL that this
        window didn't start (an earlier GUI, a closed window) would publish
        too -- a stale ground script sends its old status (frozen altitude,
        STATE_UNKNOWN), a second OCS talks to RoboCommand as the team. Offer to
        stop them. False = don't start. If WSL can't be checked, warn and start."""
        managed = self.procs[key]
        if managed.running or not managed.stop_pattern:
            return True
        distro = self.common("wsl_distro")
        try:
            out = subprocess.run(["wsl.exe", "-d", distro, "--", "ps", "-eo", "pid=,lstart=,args="],
                                 capture_output=True, text=True, encoding="utf-8", errors="replace",
                                 timeout=15, creationflags=CREATE_NO_WINDOW).stdout
        except (OSError, subprocess.SubprocessError) as e:
            self.log(key, f"WARNING: couldn't check WSL for a {key} already running ({e}) -- "
                          f"starting anyway\n")
            return True
        # pid, 5 lstart fields (e.g. "Thu Nov 12 15:27:21 2026"), args
        found = []
        for line in out.splitlines():
            parts = line.split(None, 6)
            if len(parts) == 7 and managed.stop_pattern in parts[6]:
                found.append((parts[0], " ".join(parts[2:5]), parts[6][:90]))
        if not found:
            return True
        listing = "\n".join(f"  PID {pid}  started {t}\n    {args}" for pid, t, args in found)
        if not messagebox.askyesno(
                f"{key} already running",
                f"{len(found)} {key} process(es) are already running in WSL, not started by this "
                f"window:\n\n{listing}\n\nA second copy would also publish to the OCS / "
                f"RoboCommand (stale status, mode flipping).\n\nStop them and start a fresh {key}?"):
            self.log(key, f"not started: {len(found)} {key} process(es) already running in WSL\n")
            return False
        pids = [pid for pid, _, _ in found]
        self.log(key, f"stopping leftover {key} PID(s) {' '.join(pids)}\n")
        for sig, wait in (("-INT", 5.0), ("-TERM", 3.0), ("-KILL", 1.0)):
            subprocess.run(["wsl.exe", "-d", distro, "--", "kill", sig, *pids],
                           capture_output=True, timeout=15, creationflags=CREATE_NO_WINDOW)
            deadline = time.monotonic() + wait
            while time.monotonic() < deadline:
                alive = subprocess.run(["wsl.exe", "-d", distro, "--", "ps", "-o", "pid=", "-p",
                                        ",".join(pids)], capture_output=True, text=True,
                                       timeout=15, creationflags=CREATE_NO_WINDOW).stdout.split()
                if not alive:
                    return True
                pids = alive
                time.sleep(0.5)
        self.log(key, f"WARNING: PID(s) {' '.join(pids)} still running -- not starting {key}\n")
        messagebox.showerror(key, f"Couldn't stop PID(s) {' '.join(pids)}; {key} not started.")
        return False

    def start_one(self, key):
        self._save_settings()
        c = {k[1]: v.get() for k, v in self.vars.items() if k[0] == "common"}
        if key in ("OCS", "Ground") and not self._leftovers_ok(key):
            return
        if key == "Router":
            cmd = [sys.executable, "-u", str(HERE / "elrs_windows_router.py"),
                   "--com", c["com_port"], "--baud", c["baud"]]
            wifi_ip = c.get("drone_wifi_ip", "").strip()
            if wifi_ip:
                cmd += ["--wifi", wifi_ip if ":" in wifi_ip else f"{wifi_ip}:14550"]
            self.procs["Router"].start(cmd)
        elif key == "Broker":
            if self.profile.get() == "Competition":
                messagebox.showinfo("Broker", "Competition uses RoboCommand's broker; no local broker.")
                return
            port = self.mqtt_port()
            if port is None:
                messagebox.showerror("Broker", "MQTT port must be a number from 1 to 65535.")
                return
            if tcp_reachable("127.0.0.1", port):
                self.log("Broker", f"localhost:{port} already listening — using it\n")
                self._set("Broker", "already running (not started by this window)", "#1b7f1b")
                return
            self.procs["Broker"].start(self._wsl("broker", f"exec mosquitto -p {port} -v\n"))
        elif key == "OCS":
            host = self._robocommand_host()
            if not host:
                messagebox.showerror("OCS", "No RoboCommand host: choose the adapter (DHCP gateway) "
                                            "or type the host.")
                return
            port = self.mqtt_port()
            if port is None:
                messagebox.showerror("OCS", "MQTT port must be a number from 1 to 65535.")
                return
            env = {
                "ROBOCOMMAND_ROOT": to_wsl_path(c["robocommand_repo"]),
                "ROBOCOMMAND_HOST": host, "ROBOCOMMAND_PORT": str(port),
                "TEAM_ID": c["team_id"].strip(), "VEHICLES": c["vehicles"].replace(" ", ""),
                "UAV_GEOFENCE_MAX_ALTITUDE_HAE_M": c["altitude_limit_hae_m"],
                "UAV_MAX_ALTITUDE_HAE_M": c["competition_max_hae_m"],
                "UAV_PILOT_CALLSIGN": callsign(c), "UAV_REGISTRATION": registration(c),
                "OCS_DECLARE_MODE": "auto" if c.get("declare_auto") == "1" else "manual",
                **{f"TASK{t}_TIER": c.get(f"task{t}_tier") or "NONE" for t in (1, 2, 3, 4)},
                "TASK4_ACK_MODE": "vehicle" if c.get("task4_vehicle_ack") == "1" else "auto",
            }
            exports = "".join(f"export {k}={sh_quote(v)}\n" for k, v in env.items())
            self.log("OCS", f"RoboCommand {host}:{port}, callsign {callsign(c)!r}, "
                            f"registration {registration(c)!r}\n")
            self.procs["OCS"].start(self._wsl("ocs", self._ros_prelude() + exports
                                              + "exec ros2 run seaowls_robocommand seaowls_ocs\n"))
        elif key == "Ground":
            repo = to_wsl_path(REPO)
            cmd = (f"cd {sh_quote(repo)}\n"
                   f"exec python3 -u beacon_mavlink_ground.py --mavlink-endpoint udpin:0.0.0.0:14551 "
                   f"--publish-ros --fence-inset-m {float(c['fence_inset_m']):g} "
                   f"--ceiling-margin-m {float(c['ceiling_margin_m']):g} "
                   f"--max-alt-limit-hae-m {float(c['competition_max_hae_m']):g}")
            if self.profile.get() == "Test":   # never in Competition
                cmd += f" --assume-home-hae-m {self.no_fix_home_hae():g}"
            self.procs["Ground"].start(self._wsl("ground", self._ros_prelude() + cmd + "\n"))

    def start_all(self):
        self._save_settings()
        c = {k[1]: v.get() for k, v in self.vars.items() if k[0] == "common"}
        problems = []
        if not CALLSIGN_RE.match(callsign(c)):
            problems.append(f"callsign {callsign(c)!r} is not in the required form — the OCS will not declare")
        reg = registration(c)
        if reg and not REGISTRATION_RE.match(reg):
            problems.append(f"registration {reg!r} is not in the required form — the OCS will not declare")
        try:
            problems += validate_altitude(float(c["altitude_limit_hae_m"]), float(c["competition_max_hae_m"]))
        except ValueError:
            problems.append("altitude fields must be numbers")
        if problems and not messagebox.askyesno(
                "Start with INVALID settings?",
                "\n".join(problems) + "\n\nStart anyway (e.g. for the INVALID demonstration)?"):
            return
        keys = ["Router", "OCS", "Ground"]
        if self.profile.get() == "Test":
            keys.insert(1, "Broker")
        for i, key in enumerate(keys):
            self.after(i * 1500, lambda k=key: self.start_one(k))

    def ocs_stop_ok(self) -> bool:
        """Handbook 3.4.15: sequence counters must not reset during an active run,
        and they live in the OCS -- confirm before stopping it mid-run."""
        if not (self.active_run_id and self.procs["OCS"].running):
            return True
        if not messagebox.askyesno(
                "Run in progress",
                f"Run {self.active_run_id} is active (RunStart verified).\n\n"
                f"Stopping the OCS resets every vehicle's report sequence counter, which must not "
                f"happen during a run (Handbook 3.4.15), and stops the heartbeats.\n\nStop it anyway?"):
            return False
        self.active_run_id = None
        return True

    def stop_one(self, key):
        if key == "OCS" and not self.ocs_stop_ok():
            return
        self.procs[key].stop()

    def stop_all(self):
        if not self.ocs_stop_ok():
            return

        def worker():
            for key in ("Ground", "OCS", "Broker", "Router", "Mock", "Declare", "Task4", "Restart"):
                self.procs[key].stop()
        threading.Thread(target=worker, daemon=True).start()

    # ── Tasks 2/3: the UAV echoes the resource delivery request ──
    def restart_drone_mission(self):
        """Ask beacon_mavlink_ground.py to restart seabird-mission on the drone
        over ELRS (String request id on /seaowls/uav1/restart_mission). Refused
        while the UAV is armed -- here, in the ground station and on the drone."""
        if not self.procs["Ground"].running:
            messagebox.showerror("Restart drone mission",
                                 "Start Ground first: it sends the request over the ELRS link.")
            return
        if self.uav_mode and self.uav_mode[2] == "armed":
            messagebox.showerror("Restart drone mission",
                                 "The UAV is ARMED — the mission can't be restarted while armed.")
            return
        if self.procs["Restart"].running:
            return   # a request is already on its way
        if not messagebox.askyesno("Restart drone mission",
                                   "Restart seabird-mission on the drone?\n\n"
                                   "It stops the running mission script and waits for the "
                                   "CH7 trigger again."):
            return
        request_id = time.strftime("%H%M%S")
        publish = "\n".join([
            "import time, rclpy",
            "from std_msgs.msg import String",
            "rclpy.init(); n = rclpy.create_node('restart_drone_mission')",
            "p = n.create_publisher(String, '/seaowls/uav1/restart_mission', 10)",
            "end = time.time() + 30",
            "while p.get_subscription_count() == 0 and time.time() < end:",
            "    rclpy.spin_once(n, timeout_sec=0.1)",
            "if p.get_subscription_count() == 0:",
            "    print('Ground (beacon_mavlink_ground.py) not found on ROS after 30 s'); "
            "raise SystemExit(1)",
            f"p.publish(String(data='{request_id}')); print('restart request {request_id} sent')",
            "end = time.time() + 1",
            "while time.time() < end: rclpy.spin_once(n, timeout_sec=0.1)",
        ])
        self._set("Drone mission", f"asking Ground to send restart request {request_id}…", "#a86b00")
        self.procs["Restart"].start(self._wsl(
            "restart_mission", self._ros_prelude() + f"exec python3 -u -c {sh_quote(publish)}\n"))

    def echo_delivery(self):
        """Operator: the UAV has begun the delivery -> republish the last request on
        /seaowls/uav1/report/resource_delivery, sent to RoboCommand as UAV1."""
        if not self.procs["OCS"].running:
            messagebox.showerror("Echo delivery request", "Start the OCS first.")
            return
        if self.last_delivery_request is None:
            messagebox.showinfo("Echo delivery request", "No ResourceDeliveryRequest received yet.")
            return
        if self.procs["Task4"].running:
            return
        task, resource, circle = self.last_delivery_request
        if not messagebox.askyesno("Echo delivery request",
                                   f"The UAV is starting the delivery: echo the request "
                                   f"(task {task}, resource {resource}, circle {circle}) as UAV1?"):
            return
        publish = "\n".join([
            "import time, rclpy",
            "from seaowls_interfaces.msg import ResourceDeliveryRequest as R",
            "rclpy.init(); n = rclpy.create_node('echo_delivery')",
            "p = n.create_publisher(R, '/seaowls/uav1/report/resource_delivery', 10)",
            "end = time.time() + 30",
            "while p.get_subscription_count() == 0 and time.time() < end:",
            "    rclpy.spin_once(n, timeout_sec=0.1)",
            "if p.get_subscription_count() == 0:",
            "    print('OCS not found on ROS after 30 s: is it running?'); raise SystemExit(1)",
            f"p.publish(R(task={task}, resource_color={resource}, delivery_circle_color={circle}))",
            "print('echoed the delivery request as UAV1')",
            "end = time.time() + 1",
            "while time.time() < end: rclpy.spin_once(n, timeout_sec=0.1)",
        ])
        self._set("Reports", "echoing the delivery request as UAV1…", "#a86b00")
        self.procs["Task4"].start(self._wsl(
            "uav_echo", self._ros_prelude() + f"exec python3 -u -c {sh_quote(publish)}\n"))

    # ── Task 4 ──
    def send_readiness(self):
        """Operator: the vehicle is on station after the latest AssistanceRequest
        -> std_msgs/UInt32 (its seq) on /seaowls/<vehicle>/task4/ready, and the
        OCS sends the ReadinessReport."""
        if not self.procs["OCS"].running:
            messagebox.showerror("Send ReadinessReport", "Start the OCS first.")
            return
        if not self.task4_waiting_ready:
            messagebox.showinfo("Send ReadinessReport", "No AssistanceRequest is waiting for a ReadinessReport.")
            return
        if self.procs["Task4"].running:
            return
        v, seq = self.task4_waiting_ready[-1]
        if not messagebox.askyesno("Send ReadinessReport",
                                   f"Report {v} on station for AssistanceRequest seq {seq}?"):
            return
        topic = f"/seaowls/{v.lower()}/task4/ready"
        publish = "\n".join([
            "import time, rclpy",
            "from std_msgs.msg import UInt32",
            "rclpy.init(); n = rclpy.create_node('send_task4_ready')",
            f"p = n.create_publisher(UInt32, '{topic}', 10)",
            "end = time.time() + 30",
            "while p.get_subscription_count() == 0 and time.time() < end:",
            "    rclpy.spin_once(n, timeout_sec=0.1)",
            "if p.get_subscription_count() == 0:",
            "    print('OCS not found on ROS after 30 s: is it running?'); raise SystemExit(1)",
            f"p.publish(UInt32(data={int(seq)})); print('reported {v} ready for seq {seq}')",
            "end = time.time() + 1",
            "while time.time() < end: rclpy.spin_once(n, timeout_sec=0.1)",
        ])
        self._set("Task 4", f"sending ReadinessReport for {v} (AssistanceRequest seq {seq})…", "#a86b00")
        self.procs["Task4"].start(self._wsl(
            "task4_ready", self._ros_prelude() + f"exec python3 -u -c {sh_quote(publish)}\n"))

    def rehearse_task4(self):
        """Test profile: play RoboCommand's side of Task 4 against the running OCS."""
        if self.profile.get() != "Test":
            messagebox.showerror("Rehearse Task 4", "Only in the Test profile (it plays RoboCommand).")
            return
        if not self.procs["OCS"].running:
            messagebox.showerror("Rehearse Task 4", "Start the OCS first.")
            return
        if self.procs["Task4"].running:
            return
        self._save_settings()
        vtype = self.vars[("Test", "task4_vehicle_type")].get() or "USV"
        vehicles = [v.strip() for v in self.common("vehicles").split(",") if v.strip()]
        if not any(v.upper().startswith(vtype) for v in vehicles):
            messagebox.showwarning("Rehearse Task 4", f"No {vtype} in Vehicles ({', '.join(vehicles)}): "
                                                     f"the OCS will log the commands as not routed.")
        repo = to_wsl_path(REPO)
        cmd = (f"cd {sh_quote(repo)}\n"
               f"export ROBOCOMMAND_ROOT={sh_quote(to_wsl_path(self.common('robocommand_repo')))}\n"
               f"export TEAM_ID={sh_quote(self.common('team_id').strip())}\n"
               f"export ROBOCOMMAND_PORT={self.mqtt_port() or 1883}\n"
               f"exec python3 -u tests/mock_robocommand_task4.py --scenario all --vehicle-type {vtype} "
               f"--timeout 120\n")
        self.procs["Task4"].start(self._wsl("task4_rehearsal", cmd))
        self.logs.select(list(self.procs).index("Task4"))

    # ── RunDeclaration (sent on request) ──
    def send_declaration(self):
        """Ask seaowls_ocs to send its RunDeclaration now (std_msgs/Empty on
        /seaowls/ocs/send_run_declaration). The OCS answers in its log:
        'RunDeclaration seq=N …' when sent, or 'NOT sent (requested): why'."""
        if not self.procs["OCS"].running:
            messagebox.showerror("Send RunDeclaration", "Start the OCS first.")
            return
        if self.procs["Declare"].running:
            return   # a request is already on its way
        publish = "\n".join([
            "import time, rclpy",
            "from std_msgs.msg import Empty",
            "rclpy.init(); n = rclpy.create_node('send_run_declaration')",
            "p = n.create_publisher(Empty, '/seaowls/ocs/send_run_declaration', 10)",
            "end = time.time() + 30",
            "while p.get_subscription_count() == 0 and time.time() < end:",
            "    rclpy.spin_once(n, timeout_sec=0.1)",
            "if p.get_subscription_count() == 0:",
            "    print('OCS not found on ROS after 30 s: is it running?'); raise SystemExit(1)",
            "p.publish(Empty()); print('asked the OCS to send RunDeclaration')",
            "end = time.time() + 1",
            "while time.time() < end: rclpy.spin_once(n, timeout_sec=0.1)",
        ])
        self._set("Declaration", "sending — asking the OCS…", "#a86b00")
        self.procs["Declare"].start(self._wsl(
            "declare", self._ros_prelude() + f"exec python3 -u -c {sh_quote(publish)}\n"))

    # ── mock course (Test) ──
    def _mock(self, extra):
        """(Re)start the mock RoboCommand. A previous one (it keeps serving)
        is stopped first, so only one plays RoboCommand."""
        port = self.mqtt_port() or 1883   # read on the GUI thread
        def start():
            if self.procs["Mock"].running:
                self.procs["Mock"].stop()
            repo = to_wsl_path(REPO)
            cmd = (f"cd {sh_quote(repo)}\n"
                   f"export ROBOCOMMAND_ROOT={sh_quote(to_wsl_path(self.common('robocommand_repo')))}\n"
                   f"export TEAM_ID={sh_quote(self.common('team_id').strip())}\n"
                   f"export ROBOCOMMAND_PORT={port}\n"
                   f"exec python3 -u tests/mock_robocommand_course.py {extra}\n")
            self.procs["Mock"].start(self._wsl("mock", cmd))
        self.logs.select(list(self.procs).index("Mock"))
        # stop() can take seconds (pkill in WSL): keep the window responsive.
        threading.Thread(target=start, daemon=True).start()

    def publish_mock(self):
        self._save_settings()
        size = self.vars[("Test", "mock_size")].get().replace(" ", "") or "25,25"
        kind = self.vars[("Test", "mock_invalid")].get()
        manual = self.vars[("Test", "mock_center")].get().replace(" ", "")
        manual_ok = re.match(r"^-?\d+(\.\d+)?,-?\d+(\.\d+)?$", manual)
        pos = self.drone_position() if self.vars[("Test", "mock_use_gps")].get() == "1" else None
        if pos is not None:
            center = f"{pos[0]:.7f},{pos[1]:.7f}"
            self.log("Mock", f"centring the course on the drone's GPS position {center}\n")
        elif manual_ok:
            center = manual
            if self.vars[("Test", "mock_use_gps")].get() == "1":
                self.log("Mock", f"no recent GPS fix from the drone -- using the manual centre {center}\n")
            if not self.procs["Ground"].running:
                self.log("Mock", "note: Ground isn't running -- start it to upload this course\n")
        else:
            messagebox.showerror(
                "Mock course",
                "No recent 3D GPS fix from the drone and no manual centre.\n\n"
                "Enter the manual centre as LAT,LON (used whenever there's no fix).")
            return
        # '=' form: argparse would read a negative "-27.4,153.0" as an option, not a value.
        extra = f"--center={center} --size={size}"
        invalid = COURSE_KINDS.get(_OLD_KIND_LABELS.get(kind, kind))
        if invalid:
            extra += f" --invalid {invalid}"
        # Keep serving as RoboCommand: check the RunDeclaration and send RunStart.
        extra += f" --serve --max-alt-hae {float(self.common('competition_max_hae_m') or 50):g}"
        if self.vars[("Test", "mock_no_wait_auto")].get() == "1":
            extra += " --no-wait-auto"
        self.log("Mock", f"publishing: {kind}\n")
        self._mock(extra)

    def clear_mock(self):
        if self.procs["OCS"].running and not messagebox.askyesno(
                "Clear mock course",
                "The OCS is running and would read the empty clear message as an empty course.\n"
                "Stop the OCS first?"):
            return
        if self.procs["OCS"].running:
            if not self.ocs_stop_ok():
                return
            self.procs["OCS"].stop()
        self._mock("--clear")

    def _on_close(self):
        if not self.ocs_stop_ok():
            return
        if any(p.running for p in self.procs.values()) and not messagebox.askyesno(
                "Quit", "Stop everything and quit?"):
            return
        self._save_settings()
        for key in ("Ground", "OCS", "Broker", "Router", "Mock", "Declare", "Task4", "Restart"):
            self.procs[key].stop()
        self.destroy()


if __name__ == "__main__":
    GroundGui().mainloop()
