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
    RoboCommand, no bridge / Internet sharing / forwarding, MQTT 1883
    reachable, WSL mirrored networking.
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
        "com_port": "COM6",
        "baud": "460800",
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
    },
    "Competition": {"adapter": "", "robocommand_host": ""},
    "Test": {
        "robocommand_host": "localhost",
        "mock_center": "",
        "mock_size": "25,25",
        "mock_invalid": "(valid)",
    },
}
INVALID_KINDS = ["(valid)", "open", "two-points", "self-intersecting", "bad-coords",
                 "too-small", "too-large"]


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

        self.procs = {
            "Router": Managed(self, "Router"),
            "Broker": Managed(self, "Broker", stop_pattern="mosquitto -p 1883"),
            "OCS": Managed(self, "OCS", stop_pattern="seaowls_ocs"),
            "Ground": Managed(self, "Ground", stop_pattern="beacon_mavlink_ground.py"),
            "Mock": Managed(self, "Mock"),
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
            ("ELRS COM port", "com_port"), ("Baud", "baud"), ("Team ID", "team_id"),
            ("Vehicles (comma)", "vehicles"), ("Pilot first name", "pilot_first_name"),
            ("UAPL ID (blank for Ground Validation)", "uapl_id"),
            ("Team code", "team_code"), ("CAAS sticker number", "caas_sticker"),
            ("Altitude limit (m HAE)", "altitude_limit_hae_m"),
            ("Competition maximum (m HAE)", "competition_max_hae_m"),
            ("Fence inset (m)", "fence_inset_m"), ("Ceiling margin (m)", "ceiling_margin_m"),
            ("WSL distro", "wsl_distro"), ("RoboCommand repo", "robocommand_repo"),
        ]
        for i, (label, name) in enumerate(rows):
            ttk.Label(f, text=label).grid(row=i, column=0, sticky="w")
            ttk.Entry(f, textvariable=self._var("common", name), width=30).grid(
                row=i, column=1, sticky="ew", pady=1)
        f.columnconfigure(1, weight=1)
        self.derived = ttk.Label(f, foreground="#555")
        self.derived.grid(row=len(rows), column=0, columnspan=2, sticky="w", pady=(4, 0))
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
        ttk.Label(self.test_frame, text="Course centre LAT,LON (drone's position)").grid(row=0, column=0, sticky="w")
        ttk.Entry(self.test_frame, textvariable=self._var("Test", "mock_center")).grid(row=0, column=1, sticky="ew")
        ttk.Label(self.test_frame, text="Course size W,L (m)").grid(row=1, column=0, sticky="w")
        ttk.Entry(self.test_frame, textvariable=self._var("Test", "mock_size")).grid(row=1, column=1, sticky="ew")
        ttk.Label(self.test_frame, text="Course kind (INVALID demo)").grid(row=2, column=0, sticky="w")
        ttk.Combobox(self.test_frame, textvariable=self._var("Test", "mock_invalid"),
                     values=INVALID_KINDS, state="readonly").grid(row=2, column=1, sticky="ew")
        b = ttk.Frame(self.test_frame)
        b.grid(row=3, column=0, columnspan=2, sticky="w", pady=(4, 0))
        ttk.Button(b, text="Publish mock course", command=self.publish_mock).pack(side="left")
        ttk.Button(b, text="Clear mock course", command=self.clear_mock).pack(side="left", padx=6)
        self.test_frame.columnconfigure(1, weight=1)
        self._var("Test", "robocommand_host")

        # status
        st = ttk.LabelFrame(right, text="Status", padding=6)
        st.pack(fill="x")
        for i, key in enumerate(("Router", "Broker", "OCS", "Ground", "Heartbeats", "Geofence",
                                 "Declaration", "RunStart")):
            ttk.Label(st, text=key + ":", width=12).grid(row=i, column=0, sticky="w")
            lbl = tk.Label(st, text="—", anchor="w", justify="left", wraplength=640)
            lbl.grid(row=i, column=1, sticky="w")
            self.status[key] = lbl
            if key in self.procs:
                bf = ttk.Frame(st)
                bf.grid(row=i, column=2, sticky="e")
                ttk.Button(bf, text="Start", width=6, command=lambda k=key: self.start_one(k)).pack(side="left")
                ttk.Button(bf, text="Stop", width=6, command=lambda k=key: self.procs[k].stop()).pack(side="left")
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
            if m:
                ok = int(m.group(1)) > 0
                self._set("Router", f"radio bytes in last interval: {m.group(1)}"
                          + ("" if ok else "  — nothing from the radio: wrong COM port?"),
                          "#1b7f1b" if ok else "#c0182b")
        elif name == "Ground":
            m = re.search(r"\[fence\] (UPLOADING|VALID|REFUSED|FAILED) course '([^']*)'(?:: (.*))?", line)
            if m:
                state, course, why = m.groups()
                color = {"VALID": "#1b7f1b", "UPLOADING": "#a86b00"}.get(state, "#c0182b")
                self._set("Geofence", f"{state} — course {course}" + (f": {why}" if why else "")
                          + ("  (on the UAV, read back)" if state == "VALID" else ""), color)
            elif "Heartbeat received from system" in line:
                self._set("Ground", "receiving UAV MAVLink", "#1b7f1b")
        elif name == "OCS":
            if re.search(r"UAV1 heartbeat seq=\d+", line):
                self.last_heartbeat = time.monotonic()
            m = re.search(r"RunDeclaration seq=(\d+).*course=(\S+)(.*)", line)
            if m:
                self._set("Declaration", f"sent seq {m.group(1)}, course {m.group(2)}{m.group(3)}",
                          "#1b7f1b")
            m = re.search(r"(REFUSED UAV1 geofence.*|RunDeclaration NOT sent: .*|RunDeclaration waiting .*)", line)
            if m:
                bad = not m.group(1).startswith("RunDeclaration waiting")
                self._set("Declaration", m.group(1), "#c0182b" if bad else "#a86b00")
            m = re.search(r"RunStart declaration_seq=(\d+) run_id=(\S+)", line)
            if m:
                self._set("RunStart", f"received: declaration_seq {m.group(1)}, run_id {m.group(2)}",
                          "#1b7f1b")
            if "MQTT connected" in line:
                self._set("OCS", "connected to RoboCommand broker", "#1b7f1b")
            elif "MQTT disconnected" in line:
                self._set("OCS", "MQTT disconnected — reconnecting", "#c0182b")

    def _tick(self):
        for key, p in self.procs.items():
            if key == "Mock" or (key == "Broker" and self.profile.get() == "Competition"):
                continue
            text = self.status[key].cget("text")
            if p.running:
                if text in ("—", "stopped"):
                    self._set(key, "running", "#a86b00")
            elif text not in ("—", "stopped") and not text.startswith("already running"):
                self._set(key, "stopped", "#555")
        if self.last_heartbeat is not None:
            age = time.monotonic() - self.last_heartbeat
            if age < 2.0:
                self._set("Heartbeats", "UAV1 → RoboCommand at ~2 Hz", "#1b7f1b")
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
                    mock_center=self.vars[("Test", "mock_center")].get().strip())
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
            if host:
                add("RoboCommand MQTT", "ok" if tcp_reachable(host, 1883) else "bad",
                    f"{host}:1883 " + ("reachable" if tcp_reachable(host, 1883)
                                        else "NOT reachable (cable, subnet, firewall?)"))
        else:
            add("Local broker", "ok" if tcp_reachable("127.0.0.1", 1883) else "warn",
                "localhost:1883 listening" if tcp_reachable("127.0.0.1", 1883)
                else "not running yet — Start all starts Mosquitto in WSL")
            if not c["mock_center"]:
                add("Mock course centre", "warn", "set LAT,LON to the drone's position (QGC) before publishing")

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

    def start_one(self, key):
        self._save_settings()
        c = {k[1]: v.get() for k, v in self.vars.items() if k[0] == "common"}
        if key == "Router":
            self.procs["Router"].start([sys.executable, "-u", str(HERE / "elrs_windows_router.py"),
                                        "--com", c["com_port"], "--baud", c["baud"]])
        elif key == "Broker":
            if self.profile.get() == "Competition":
                messagebox.showinfo("Broker", "Competition uses RoboCommand's broker; no local broker.")
                return
            if tcp_reachable("127.0.0.1", 1883):
                self.log("Broker", "localhost:1883 already listening — using it\n")
                self._set("Broker", "already running (not started by this window)", "#1b7f1b")
                return
            self.procs["Broker"].start(self._wsl("broker", "exec mosquitto -p 1883 -v\n"))
        elif key == "OCS":
            host = self._robocommand_host()
            if not host:
                messagebox.showerror("OCS", "No RoboCommand host: choose the adapter (DHCP gateway) "
                                            "or type the host.")
                return
            env = {
                "ROBOCOMMAND_ROOT": to_wsl_path(c["robocommand_repo"]),
                "ROBOCOMMAND_HOST": host, "ROBOCOMMAND_PORT": "1883",
                "TEAM_ID": c["team_id"].strip(), "VEHICLES": c["vehicles"].replace(" ", ""),
                "UAV_GEOFENCE_MAX_ALTITUDE_HAE_M": c["altitude_limit_hae_m"],
                "UAV_MAX_ALTITUDE_HAE_M": c["competition_max_hae_m"],
                "UAV_PILOT_CALLSIGN": callsign(c), "UAV_REGISTRATION": registration(c),
            }
            exports = "".join(f"export {k}={sh_quote(v)}\n" for k, v in env.items())
            self.log("OCS", f"RoboCommand {host}:1883, callsign {callsign(c)!r}, "
                            f"registration {registration(c)!r}\n")
            self.procs["OCS"].start(self._wsl("ocs", self._ros_prelude() + exports
                                              + "exec ros2 run seaowls_robocommand seaowls_ocs\n"))
        elif key == "Ground":
            repo = to_wsl_path(REPO)
            cmd = (f"cd {sh_quote(repo)}\n"
                   f"exec python3 -u beacon_mavlink_ground.py --mavlink-endpoint udpin:0.0.0.0:14551 "
                   f"--publish-ros --fence-inset-m {float(c['fence_inset_m']):g} "
                   f"--ceiling-margin-m {float(c['ceiling_margin_m']):g} "
                   f"--max-alt-limit-hae-m {float(c['competition_max_hae_m']):g}\n")
            self.procs["Ground"].start(self._wsl("ground", self._ros_prelude() + cmd))

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

    def stop_all(self):
        def worker():
            for key in ("Ground", "OCS", "Broker", "Router", "Mock"):
                self.procs[key].stop()
        threading.Thread(target=worker, daemon=True).start()

    # ── mock course (Test) ──
    def _mock(self, extra):
        repo = to_wsl_path(REPO)
        cmd = (f"cd {sh_quote(repo)}\nexport ROBOCOMMAND_ROOT={sh_quote(to_wsl_path(self.common('robocommand_repo')))}\n"
               f"exec python3 -u tests/mock_robocommand_course.py {extra}\n")
        self.procs["Mock"].start(self._wsl("mock", cmd))
        self.logs.select(list(self.procs).index("Mock"))

    def publish_mock(self):
        self._save_settings()
        center = self.vars[("Test", "mock_center")].get().replace(" ", "")
        size = self.vars[("Test", "mock_size")].get().replace(" ", "") or "25,25"
        kind = self.vars[("Test", "mock_invalid")].get()
        if not re.match(r"^-?\d+(\.\d+)?,-?\d+(\.\d+)?$", center):
            messagebox.showerror("Mock course", "Set the course centre as LAT,LON (the drone's position).")
            return
        extra = f"--center {center} --size {size}"
        if kind and kind != "(valid)":
            extra += f" --invalid {kind}"
        self._mock(extra)

    def clear_mock(self):
        if self.procs["OCS"].running and not messagebox.askyesno(
                "Clear mock course",
                "The OCS is running and would read the empty clear message as an empty course.\n"
                "Stop the OCS first?"):
            return
        if self.procs["OCS"].running:
            self.procs["OCS"].stop()
        self._mock("--clear")

    def _on_close(self):
        if any(p.running for p in self.procs.values()) and not messagebox.askyesno(
                "Quit", "Stop everything and quit?"):
            return
        self._save_settings()
        for key in ("Ground", "OCS", "Broker", "Router", "Mock"):
            self.procs[key].stop()
        self.destroy()


if __name__ == "__main__":
    GroundGui().mainloop()
