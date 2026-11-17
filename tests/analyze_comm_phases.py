#!/usr/bin/env python3
"""
analyze_comm_phases.py -- communication metrics for proposal phases 2, 3 and 4,
computed from ground-side logs only (no drone-side data needed).

  Phase 2  flight:  UAV -> ELRS/WiFi -> router -> ground bridge
  Phase 3  bench :  UAV -> ground -> OCS -> RoboCommand MQTT / USV guidance
  Phase 4  flight:  UAV -> ground -> OCS -> RoboCommand MQTT / USV guidance

Inputs (read-only):
  logs/ground_gui/<session>/{router,ground,ocs,broker}.log
  QGroundControl .tlog files (default: %USERPROFILE%/OneDrive/Documents/QGroundControl/Telemetry)

Outputs (always a NEW folder, nothing is overwritten):
  analysis/comm_phases_<YYYYmmdd_HHMMSS>/
      sessions.csv    one row per ground-GUI session
      flights.csv     one row per armed segment found in the tlogs
      roundtrip.csv   RoboCommand RunStart -> drone ACK, restart request -> drone ACK
      summary.md      per-phase tables + limitations

Usage (Windows; pymavlink is installed for 3.10):
  py -3.10 tests/analyze_comm_phases.py
  py -3.10 tests/analyze_comm_phases.py --tlog-dir "D:/other/tlogs" --max-tlog-mb 1000
"""

import argparse
import csv
import datetime as dt
import os
import re
import statistics
import sys
from collections import defaultdict
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
DEFAULT_LOGS = REPO / "logs" / "ground_gui"
DEFAULT_TLOG_DIR = Path(os.environ.get("USERPROFILE", str(Path.home()))) / "OneDrive" / "Documents" / "QGroundControl" / "Telemetry"
ROUTER_PERIOD_S = 5.0          # elrs_windows_router.py --stats-period default
SEABIRD_COMPS = (191, 192)     # mission gate / restart helper components on the UAV
RTT_MAX_S = 300.0              # an ACK more than this after the request is not matched


# ── small helpers ─────────────────────────────────────────────────────────────

def pct(sorted_vals, p):
    if not sorted_vals:
        return None
    k = max(0, min(len(sorted_vals) - 1, int(round(p / 100.0 * len(sorted_vals) + 0.5)) - 1))
    return sorted_vals[k]


def gaps(ts):
    ts = sorted(ts)
    return [b - a for a, b in zip(ts, ts[1:])]


RUN_SPLIT_S = 300.0   # a gap longer than this = the process was not running (separate runs in one log)


def gap_stats(ts, prefix):
    allg = gaps(ts)
    g = sorted(x for x in allg if x <= RUN_SPLIT_S)
    out = {f"{prefix}_count": len(ts), f"{prefix}_runs": 1 + sum(1 for x in allg if x > RUN_SPLIT_S) if ts else 0}
    if g:
        span = sum(g)
        out[f"{prefix}_rate_hz"] = round(len(g) / span, 3) if span > 0 else None
        out[f"{prefix}_gap_median_s"] = round(statistics.median(g), 3)
        out[f"{prefix}_gap_p95_s"] = round(pct(g, 95), 3)
        out[f"{prefix}_gap_max_s"] = round(g[-1], 3)
        out[f"{prefix}_gaps_over_2s"] = sum(1 for x in g if x > 2.0)
    return out


def r(x, n=3):
    return None if x is None else round(x, n)


def read_lines(p):
    try:
        with open(p, "r", encoding="utf-8", errors="replace") as f:
            return f.read().splitlines()
    except FileNotFoundError:
        return []


def local_epoch(d, hh, mm, ss):
    return dt.datetime(d.year, d.month, d.day, hh, mm, ss).timestamp()


# ── router.log ────────────────────────────────────────────────────────────────

RE_KV = re.compile(r"(\w+)=(\S+)")


def parse_router(lines):
    iv = []
    switches = 0
    for ln in lines:
        if ln.startswith("Last interval:"):
            d = {}
            for k, v in RE_KV.findall(ln):
                try:
                    d[k] = int(v)
                except ValueError:
                    d[k] = v
            iv.append(d)
        elif ln.startswith("Uplink now over"):
            switches += 1
    out = {"router_intervals": len(iv)}
    if not iv:
        return out
    S = lambda k: sum(d.get(k, 0) for d in iv if isinstance(d.get(k, 0), int))
    valid, lost = S("valid_frames"), S("lost_frames_estimate")
    dur = len(iv) * ROUTER_PERIOD_S
    rx_intervals = [d for d in iv if d.get("serial_bytes", 0) or d.get("wifi_frames", 0)]
    out.update({
        "router_duration_s": dur,
        "router_valid_frames": valid,
        "router_lost_frames": lost,
        "router_frames_per_s": r(valid / dur, 2),
        "router_seq_loss_pct": r(100.0 * lost / (valid + lost), 1) if valid + lost else None,
        "router_duplicates": S("duplicate_frames"),
        "router_out_of_order": S("out_of_order_frames"),
        "router_bad_bytes": S("bad_data"),
        "router_parser_errors": S("parser_errors"),
        "router_beacons": S("beacons"),
        "router_elrs_frames": S("elrs_frames"),
        "router_wifi_frames": S("wifi_frames"),
        "router_wifi_dup_dropped": S("dup_dropped"),
        "router_uplink_wifi_intervals": sum(1 for d in iv if d.get("uplink") == "wifi"),
        "router_uplink_switches": switches,
        "router_zero_frame_intervals": sum(1 for d in iv if d.get("valid_frames", 0) == 0),
        "router_zero_frame_intervals_while_link_seen": sum(
            1 for d in iv[iv.index(rx_intervals[0]):] if d.get("valid_frames", 0) == 0) if rx_intervals else 0,
        "router_wsl_send_failed": S("wsl_failed"),
        "router_uplink_dropped": S("uplink_dropped"),
    })
    return out


# ── ground.log (beacon_mavlink_ground.py) ─────────────────────────────────────

G = {
    "armed": re.compile(r"\[mode\] UAV\d \S+ -> STATE_\w+ \(armed\)"),
    "rs_relay": re.compile(r"\[runstart\] RunStart run_id=(\d+) verified"),
    "rs_ack": re.compile(r"UAV acknowledged RunStart run_id=(\d+)"),
    "rs_noack": re.compile(r"still no acknowledgement from the UAV for run_id=(\d+) \((\d+) sends\)"),
    "rst_req": re.compile(r"\[restart\] request (\d{6}) sent"),
    "rst_ack": re.compile(r"\[restart\] drone: OK .*request (\d{6})"),
    "f_up": re.compile(r"\[fence\] UPLOADING course"),
    "f_acc": re.compile(r"\[fence\] .*Fence accepted by PX4"),
    "f_valid": re.compile(r"\[fence\] VALID course"),
    "f_fail": re.compile(r"\[fence\] FAILED course"),
    "f_wait": re.compile(r"\[fence\] WAITING course"),
    "hold": re.compile(r"\[link\] no UAV heartbeat for (\d+(?:\.\d+)?) s"),
    "back": re.compile(r"\[link\] UAV heartbeat back after (\d+(?:\.\d+)?) s"),
    "beacons": re.compile(r"(\d+) unique beacon messages, (\d+) duplicates"),
    "sp": re.compile(r"SafePassage published: (\d+) buoys"),
    "killed": re.compile(r"-> STATE_KILLED"),
}


def parse_ground(lines):
    relayed, acked, noack_sends = set(), set(), defaultdict(int)
    rst_req, rst_ack = [], set()
    c = defaultdict(int)
    backs, holds = [], []
    uniq = dup = 0
    for ln in lines:
        if G["armed"].search(ln):
            c["armed_lines"] += 1
        if G["killed"].search(ln):
            c["killed"] += 1
        m = G["rs_relay"].search(ln)
        if m:
            relayed.add(int(m.group(1)))
        m = G["rs_ack"].search(ln)
        if m:
            acked.add(int(m.group(1)))
        m = G["rs_noack"].search(ln)
        if m:
            noack_sends[int(m.group(1))] = max(noack_sends[int(m.group(1))], int(m.group(2)))
        m = G["rst_req"].search(ln)
        if m:
            rst_req.append(m.group(1))
        m = G["rst_ack"].search(ln)
        if m:
            rst_ack.add(m.group(1))
        for k in ("f_up", "f_acc", "f_valid", "f_fail", "f_wait"):
            if G[k].search(ln):
                c[k] += 1
        m = G["hold"].search(ln)
        if m:
            holds.append(float(m.group(1)))
        m = G["back"].search(ln)
        if m:
            backs.append(float(m.group(1)))
        m = G["beacons"].search(ln)
        if m:
            uniq += int(m.group(1)); dup += int(m.group(2))
        m = G["sp"].search(ln)
        if m:
            c["safepassage_published"] += 1
    return {
        "g_armed": c["armed_lines"] > 0,
        "g_killed_events": c["killed"],
        "g_runstart_relayed": len(relayed),
        "g_runstart_acked": len(relayed & acked),
        "g_runstart_never_acked": ",".join(str(x) for x in sorted(relayed - acked)),
        "g_runstart_max_sends_without_ack": max(noack_sends.values()) if noack_sends else 0,
        "g_restart_requests": len(rst_req),
        "g_restart_acked": sum(1 for x in rst_req if x in rst_ack),
        "g_fence_uploads": c["f_up"],
        "g_fence_accepted": c["f_acc"],
        "g_fence_valid": c["f_valid"],
        "g_fence_failed": c["f_fail"],
        "g_fence_waiting": c["f_wait"],
        "g_link_gaps_recovered": len(backs),
        "g_link_gap_max_s": max(backs + holds) if (backs or holds) else 0,
        "g_link_gap_total_s": r(sum(backs), 1),
        "g_beacons_unique": uniq,
        "g_beacons_duplicate": dup,
        "g_safepassage_published": c["safepassage_published"],
        "_restart_ids": rst_req,
    }


# ── ocs.log (seaowls_ocs) ─────────────────────────────────────────────────────

RE_OCS = re.compile(r"^\[(INFO|WARN|ERROR)\] \[(\d+\.\d+)\] \[seaowls_ocs\]: (.*)")
O = {
    "hb": re.compile(r"\[SEAOWLS OCS\] UAV\d heartbeat seq=\d+ speed=(\S+) heading=(\S+) altitude=(\S+)"),
    "ros": re.compile(r"\[SEAOWLS ROS2\] UAV\d speed="),
    "stale": re.compile(r"STALE telemetry UAV"),
    "mqtt": re.compile(r"MQTT connected"),
    "rs": re.compile(r"RunStart declaration_seq=\d+ run_id=(\d+)"),
    "decl": re.compile(r"\[SEAOWLS OCS\] RunDeclaration seq="),
    "decl_not": re.compile(r"RunDeclaration NOT sent"),
    "fence_pub": re.compile(r"Published UAV geofence"),
    "fence_rb": re.compile(r"UAV\d onboard geofence course="),
    "sp": re.compile(r"SafePassage UAV\d -> RoboCommand"),
    "usv_cmd": re.compile(r"\[task\d\] \w+ .*-> USV"),
}


def parse_ocs(lines):
    hb_t, ros_t, runstarts = [], [], []
    c = defaultdict(int)
    for ln in lines:
        if ln.startswith("Traceback"):
            c["tracebacks"] += 1
        if "disconnected() takes" in ln:
            c["shutdown_tracebacks"] += 1
        m = RE_OCS.match(ln)
        if not m:
            continue
        t, body = float(m.group(2)), m.group(3)
        if m.group(1) == "ERROR":
            c["errors"] += 1
        mm = O["hb"].search(body)
        if mm:
            hb_t.append(t)
            if "nan" in mm.groups():
                c["hb_nan"] += 1
            continue
        if O["ros"].search(body):
            ros_t.append(t)
            continue
        for k in ("stale", "mqtt", "decl", "decl_not", "fence_pub", "fence_rb", "sp", "usv_cmd"):
            if O[k].search(body):
                c[k] += 1
        mm = O["rs"].search(body)
        if mm:
            runstarts.append((int(mm.group(1)), t))
    out = {
        "o_mqtt_connects": c["mqtt"],
        "o_heartbeats_nan": c["hb_nan"],
        "o_stale_suppressed": c["stale"],
        "o_runstarts_received": len(runstarts),
        "o_rundeclarations_sent": c["decl"],
        "o_rundeclarations_not_sent": c["decl_not"],
        "o_geofence_published_to_uav": c["fence_pub"],
        "o_geofence_readback": c["fence_rb"],
        "o_safepassage_forwarded": c["sp"],
        "o_usv_commands_forwarded": c["usv_cmd"],
        "o_errors": c["errors"],
        "o_tracebacks": c["tracebacks"],
        "o_tracebacks_at_shutdown": c["shutdown_tracebacks"],
        "o_first_t": min(hb_t + ros_t) if (hb_t or ros_t) else None,
        "o_last_t": max(hb_t + ros_t) if (hb_t or ros_t) else None,
    }
    out.update(gap_stats(hb_t, "o_hb"))
    out.update(gap_stats(ros_t, "o_status_in"))
    out["_hb_t"] = hb_t
    out["_runstarts"] = runstarts
    return out


# ── broker.log (mosquitto) ────────────────────────────────────────────────────

def parse_broker(lines):
    conn = sum(1 for l in lines if "New client connected" in l)
    disc = sum(1 for l in lines if re.search(r"Client \S+ (closed its connection|disconnected)", l))
    tout = sum(1 for l in lines if "has exceeded timeout" in l)
    return {"b_client_connects": conn, "b_client_disconnects": disc, "b_client_timeouts": tout}


# ── tlogs ─────────────────────────────────────────────────────────────────────

RE_TLOG_NAME = re.compile(r"(\d{4})-(\d{2})-(\d{2}) \d{2}-\d{2}-\d{2}\.tlog$")


class SeqTracker:
    def __init__(self):
        self.last, self.recv, self.lost, self.ooo = None, 0, 0, 0

    def add(self, seq):
        self.recv += 1
        if self.last is not None:
            g = (seq - self.last - 1) % 256
            if g > 128:            # went backwards: duplicate / out of order, not loss
                self.ooo += 1
                return
            self.lost += g
        self.last = seq

    def loss_pct(self):
        n = self.recv + self.lost
        return r(100.0 * self.lost / n, 2) if n else None


def scan_tlog(path):
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(str(path), dialect="common", robust_parsing=True)
    msgs = []        # (t, comp, seq, type, armed_flag_or_None, text)
    while True:
        try:
            x = m.recv_msg()
        except Exception:
            continue
        if x is None:
            break
        if x.get_type() == "BAD_DATA" or x.get_srcSystem() != 1:
            continue          # only what the UAV sent (QGC's own TX is also in the tlog)
        typ = x.get_type()
        armed = None
        if typ == "HEARTBEAT" and x.get_srcComponent() == 1:
            armed = bool(x.base_mode & 128)
        text = x.text if typ == "STATUSTEXT" else None
        msgs.append((x._timestamp, x.get_srcComponent(), x.get_seq(), typ, armed, text))
    return msgs


def tlog_segments(msgs):
    segs, cur = [], None
    for t, comp, seq, typ, armed, text in msgs:
        if armed is None:
            continue
        if armed and cur is None:
            cur = [t, None]
        elif not armed and cur is not None:
            cur[1] = t
            segs.append(tuple(cur)); cur = None
    if cur is not None:
        cur[1] = msgs[-1][0]
        segs.append((cur[0], cur[1]))
    return segs


def window_stats(msgs, t0, t1):
    w = [x for x in msgs if t0 <= x[0] <= t1]
    trk = defaultdict(SeqTracker)
    hb, task, allt = [], [], []
    for t, comp, seq, typ, armed, text in w:
        trk[comp].add(seq)
        allt.append(t)
        if typ == "HEARTBEAT" and comp == 1:
            hb.append(t)
        if text and text.startswith("SEABIRD_TASK"):
            task.append(t)
    px4 = trk.get(1, SeqTracker())
    sb = SeqTracker()
    for c in SEABIRD_COMPS:
        if c in trk:
            sb.recv += trk[c].recv; sb.lost += trk[c].lost; sb.ooo += trk[c].ooo
    dur = t1 - t0
    out = {
        "duration_s": r(dur, 1),
        "uav_msgs": len(w),
        "uav_msgs_per_s": r(len(w) / dur, 2) if dur > 0 else None,
        "px4_seq_loss_pct": px4.loss_pct(),
        "seabird_msgs": sb.recv,
        "seabird_seq_loss_pct": sb.loss_pct(),
        "out_of_order": sum(t.ooo for t in trk.values()),
        "any_msg_gap_max_s": r(max(gaps(allt)), 2) if len(allt) > 1 else None,
    }
    out.update(gap_stats(hb, "px4_hb"))
    out.update(gap_stats(task, "seabird_task"))
    return out


def first_acks(msgs):
    rs, rst = defaultdict(list), defaultdict(list)
    for t, comp, seq, typ, armed, text in msgs:
        if not text:
            continue
        mm = re.match(r"SEABIRD_RUNSTART_ACK (\d+)", text)
        if mm:
            rs[int(mm.group(1))].append(t)
        mm = re.match(r"SEABIRD_RESTART_ACK (\d{6})", text)
        if mm:
            rst[mm.group(1)].append(t)
    return rs, rst


# ── competition log package (router_wsl*.csv, ocs_*.log, ground.log, qgc_tlogs) ─

AIRBORNE_REL_ALT_M = 1.0    # same definition as the package README
AIRBORNE_MERGE_S = 30.0     # airborne samples closer than this belong to one window (GPI is ~1 Hz or less over ELRS)
AIRBORNE_PAD_S = 1.0        # widen each window by this on both sides (one GPI period)
DECODE_TYPES = {"GLOBAL_POSITION_INT", "HEARTBEAT", "STATUSTEXT"}


def utc(t):
    return dt.datetime.fromtimestamp(t, tz=dt.timezone.utc).strftime("%Y-%m-%d %H:%M:%SZ")


def read_router_csv(path):
    """Rows of the router per-message CSV, with GPI/HEARTBEAT/STATUSTEXT decoded."""
    from pymavlink.dialects.v20 import common as mavlink2
    parser = mavlink2.MAVLink(None)
    parser.robust_parsing = True
    rows = []
    with open(path, newline="", encoding="utf-8", errors="replace") as f:
        for d in csv.DictReader(f):
            try:
                t = float(d["epoch_time"])
            except (TypeError, ValueError):
                continue
            row = {"t": t, "status": d.get("status", ""), "type": d.get("msg_type", ""),
                   "bytes": int(d.get("bytes") or 0), "sys": d.get("src_system", ""),
                   "comp": d.get("src_component", ""), "seq": d.get("mav_seq", "")}
            if row["status"] == "SENT" and row["type"] in DECODE_TYPES and d.get("hex_payload"):
                try:
                    msgs = parser.parse_buffer(bytes.fromhex(d["hex_payload"])) or []
                except Exception:
                    msgs = []
                for m in msgs:
                    if m.get_type() == "GLOBAL_POSITION_INT":
                        row["rel_alt_m"] = m.relative_alt / 1000.0
                    elif m.get_type() == "HEARTBEAT" and m.get_srcComponent() == 1:
                        row["armed"] = bool(m.base_mode & 128)
                    elif m.get_type() == "STATUSTEXT":
                        row["text"] = m.text
            rows.append(row)
    rows.sort(key=lambda x: x["t"])
    return rows


def airborne_windows(rows):
    ts = [x["t"] for x in rows if x.get("rel_alt_m") is not None and x["rel_alt_m"] > AIRBORNE_REL_ALT_M]
    wins = []
    for t in ts:
        if wins and t - wins[-1][1] <= AIRBORNE_MERGE_S:
            wins[-1][1] = t
            wins[-1][2] += 1
        else:
            wins.append([t, t, 1])
    return [(a - AIRBORNE_PAD_S, b + AIRBORNE_PAD_S, n) for a, b, n in wins]


def csv_window_stats(rows, t0, t1):
    w = [x for x in rows if t0 <= x["t"] <= t1]
    good = [x for x in w if x["status"] == "SENT"]
    bad = [x for x in w if x["status"] != "SENT"]
    trk = defaultdict(SeqTracker)
    for x in good:
        if x["sys"] == "1" and x["seq"] not in ("", None):
            try:
                trk[int(x["comp"])].add(int(x["seq"]))
            except ValueError:
                pass
    sb = SeqTracker()
    for c in SEABIRD_COMPS:
        if c in trk:
            sb.recv += trk[c].recv; sb.lost += trk[c].lost; sb.ooo += trk[c].ooo
    dur = max(t1 - t0, 1e-9)
    gt = [x["t"] for x in good]
    out = {
        "duration_s": r(t1 - t0, 1),
        "msgs": len(good),
        "msgs_per_s": r(len(good) / dur, 2),
        "bytes_per_s": r(sum(x["bytes"] for x in good) / dur, 1),
        "bad_frames": len(bad),
        "bad_crc": sum(1 for x in bad if x["type"] == "crc"),
        "bad_prefix": sum(1 for x in bad if x["type"] == "bad_prefix"),
        "bad_per_100_good": r(100.0 * len(bad) / len(good), 2) if good else None,
        "px4_seq_loss_pct": trk[1].loss_pct() if 1 in trk else None,
        "seabird_msgs": sb.recv,
        "seabird_seq_loss_pct": sb.loss_pct(),
        "out_of_order": sum(t.ooo for t in trk.values()),
        "silence_max_s": r(max(gaps(gt)), 3) if len(gt) > 1 else None,
        "silence_p95_s": r(pct(sorted(gaps(gt)), 95), 3) if len(gt) > 1 else None,
    }
    for typ, key in (("HEARTBEAT", "hb"), ("GLOBAL_POSITION_INT", "gpi"), ("VFR_HUD", "vfr")):
        ts = [x["t"] for x in good if x["type"] == typ and x["comp"] == "1"]
        st = gap_stats(ts, key)
        out[f"{key}_rate_hz"] = st.get(f"{key}_rate_hz")
        out[f"{key}_gap_max_s"] = st.get(f"{key}_gap_max_s")
    return out


RE_OCS_VEH_HB = re.compile(r"\[SEAOWLS OCS\] (UAV\d|USV\d) heartbeat seq=\d+ speed=(\S+) heading=(\S+) altitude=(\S+)")
RE_OCS_STALE = re.compile(r"STALE telemetry (UAV\d|USV\d)")
RE_OCS_RX_RUNSTART = re.compile(r"RxCommand seq=(\d+) body=run_start")
RE_OCS_FWD = re.compile(r"run_start seq=(\d+) forwarded as JSON -> \[(.*)\]")
RE_OCS_TEENSY = re.compile(r"(USV\d) TeensyStatus auto=(True|False) kill=(True|False)")


def parse_package_ocs(lines):
    hb, stale = defaultdict(list), defaultdict(int)
    nan = defaultdict(int)
    rx, fwd, runstarts = {}, {}, []
    teensy = defaultdict(int)
    for ln in lines:
        m = RE_OCS.match(ln)
        if not m:
            continue
        t, body = float(m.group(2)), m.group(3)
        mm = RE_OCS_VEH_HB.search(body)
        if mm:
            hb[mm.group(1)].append(t)
            if "nan" in mm.groups()[1:]:
                nan[mm.group(1)] += 1
            continue
        mm = RE_OCS_STALE.search(body)
        if mm:
            stale[mm.group(1)] += 1
        mm = RE_OCS_RX_RUNSTART.search(body)
        if mm:
            rx[mm.group(1)] = t
        mm = RE_OCS_FWD.search(body)
        if mm:
            fwd[mm.group(1)] = (t, mm.group(2).replace("'", "").replace(" ", ""))
        mm = O["rs"].search(body)
        if mm:
            runstarts.append((int(mm.group(1)), t))
        mm = RE_OCS_TEENSY.search(body)
        if mm:
            teensy[(mm.group(1), "auto" if mm.group(2) == "True" else "manual",
                    "kill" if mm.group(3) == "True" else "")] += 1
    return hb, stale, nan, rx, fwd, runstarts, teensy


def analyze_package(pkg, out_dir, write_csv):
    pkg = Path(pkg)
    md = [f"\n## Competition runs alongside the USV (log package `{pkg.name}`)\n",
          "Source: per-message router CSVs (UTC epoch arrival times), OCS log, ground.log, QGC tlogs. "
          f"Airborne = GLOBAL_POSITION_INT relative altitude > {AIRBORNE_REL_ALT_M} m.\n"]
    link_rows, ocs_rows, rtt_rows = [], [], []
    acks = defaultdict(list)          # run_id -> arrival times of SEABIRD_RUNSTART_ACK
    airborne_all = []

    for csvp in sorted(pkg.rglob("router_wsl*.csv")):
        label = str(csvp.parent.relative_to(pkg)).replace("\\", "/")
        print(f"[package] {label}/{csvp.name} ...", flush=True)
        rows = read_router_csv(csvp)
        if not rows:
            continue
        for x in rows:
            mm = re.match(r"SEABIRD_RUNSTART_ACK (\d+)", x.get("text") or "")
            if mm:
                acks[int(mm.group(1))].append(x["t"])
        t0, t1 = rows[0]["t"], rows[-1]["t"]
        wins3 = airborne_windows(rows)
        wins = [(a, b) for a, b, n in wins3]
        airborne_all += [(label, a, b) for a, b in wins]
        segs = [("whole log", t0, t1)]
        if wins:
            segs.append(("on ground (outside airborne)", None, None))
        segs += [(f"airborne {i} ({n} alt samples)", a, b) for i, (a, b, n) in enumerate(wins3, 1)]
        for name, a, b in segs:
            if a is None:     # everything outside the airborne windows
                ground = [x for x in rows if not any(wa <= x["t"] <= wb for wa, wb in wins)]
                st = csv_window_stats(ground, t0, t1)
                st["duration_s"] = r((t1 - t0) - sum(wb - wa for wa, wb in wins), 1)
                st["msgs_per_s"] = r(st["msgs"] / st["duration_s"], 2) if st["duration_s"] else None
                st["silence_max_s"] = st["silence_p95_s"] = None   # gaps would span the flights
                a, b = t0, t1
            else:
                st = csv_window_stats(rows, a, b)
            link_rows.append({"run": label, "segment": name, "from_utc": utc(a), "to_utc": utc(b), **st})
        g = parse_ground(read_lines(csvp.parent / "ground.log"))
        for row in link_rows:
            if row["run"] == label and row["segment"] == "whole log":
                row.update({k: v for k, v in g.items() if k.startswith(("g_runstart", "g_link", "g_beacons", "g_fence"))})

    # tlogs: extra ACK arrival times (QGC's own capture)
    for tp in sorted((pkg / "qgc_tlogs").glob("*.tlog")) if (pkg / "qgc_tlogs").exists() else []:
        print(f"[package] qgc_tlogs/{tp.name} ...", flush=True)
        ra, _ = first_acks(scan_tlog(tp))
        for k, v in ra.items():
            acks[k].extend(v)

    # OCS logs (UAV + USV under one OCS)
    for op in sorted(pkg.rglob("ocs_*.log")):
        label = str(op.parent.relative_to(pkg)).replace("\\", "/")
        hb, stale, nan, rx, fwd, runstarts, teensy = parse_package_ocs(read_lines(op))
        for veh in sorted(hb):
            st = gap_stats(hb[veh], "hb")
            row = {"run": label, "vehicle": veh, "scope": "whole log", **st,
                   "stale_suppressed": stale.get(veh, 0), "nan_heartbeats": nan.get(veh, 0)}
            ocs_rows.append(row)
            for i, (rl, a, b) in enumerate([w for w in airborne_all if w[0] == label], 1):
                ins = [t for t in hb[veh] if a <= t <= b]
                st2 = gap_stats(ins, "hb")
                if len(ins) < 2:     # include the edges so a short window still shows its gap
                    st2["hb_count"] = len(ins)
                ocs_rows.append({"run": label, "vehicle": veh, "scope": f"UAV airborne {i}", **st2})
        for seq, t in rx.items():
            if seq in fwd:
                rtt_rows.append({"run": label, "kind": "RoboCommand run_start -> forwarded to vehicles",
                                 "id": seq, "sent_utc": utc(t), "targets": fwd[seq][1],
                                 "latency_s": r(fwd[seq][0] - t, 4)})
        for run_id, t in runstarts:
            cand = [x for x in acks.get(run_id, []) if 0 <= x - t <= RTT_MAX_S]
            rtt_rows.append({"run": label, "kind": "RunStart OCS -> UAV -> ACK at ground", "id": run_id,
                             "sent_utc": utc(t), "targets": "UAV1", "latency_s": r(min(cand) - t, 3) if cand else None})
        if teensy:
            md.append(f"\nUSV status in `{label}` OCS log (TeensyStatus samples): " +
                      ", ".join(f"{k[0]} {k[1]}{' +kill' if k[2] else ''}={v}" for k, v in sorted(teensy.items())) + "\n")

    write_csv("package_link.csv", link_rows)
    write_csv("package_ocs.csv", ocs_rows)
    write_csv("package_roundtrip.csv", rtt_rows)

    md.append("\n### Phase 2 - UAV -> ground link (per-message router CSV)\n\n" + md_table(
        ["run", "segment", "from (UTC)", "dur s", "msgs/s", "bad /100 good", "PX4 seq loss %", "Seabird loss %",
         "HB Hz", "HB gap max s", "longest silence s"],
        [(x["run"], x["segment"], x["from_utc"][11:], x["duration_s"], x["msgs_per_s"], x["bad_per_100_good"],
          x["px4_seq_loss_pct"], x["seabird_seq_loss_pct"], x["hb_rate_hz"], x["hb_gap_max_s"], x["silence_max_s"])
         for x in link_rows]))
    cmd = [x for x in link_rows if x["segment"] == "whole log" and "g_runstart_relayed" in x]
    if cmd:
        md.append("\nCommand path from ground.log (counts only, no timestamps):\n\n" + md_table(
            ["run", "RunStart relayed", "acked", "never acked", "max sends w/o ack", "link gaps recovered", "longest gap s", "beacons"],
            [(x["run"], x["g_runstart_relayed"], x["g_runstart_acked"], x["g_runstart_never_acked"],
              x["g_runstart_max_sends_without_ack"], x["g_link_gaps_recovered"], x["g_link_gap_max_s"],
              x["g_beacons_unique"]) for x in cmd]))
    if ocs_rows:
        md.append("\n### Phase 4 - OCS to RoboCommand, UAV and USV together\n\n" + md_table(
            ["run", "vehicle", "scope", "heartbeats", "Hz", "gap p95 s", "gap max s", "gaps > 2 s", "stale suppressed"],
            [(x["run"], x["vehicle"], x["scope"], x.get("hb_count"), x.get("hb_rate_hz"), x.get("hb_gap_p95_s"),
              x.get("hb_gap_max_s"), x.get("hb_gaps_over_2s"), x.get("stale_suppressed", "")) for x in ocs_rows]))
    if rtt_rows:
        md.append("\nCommand latency:\n\n" + md_table(
            ["run", "path", "id", "sent (UTC)", "targets", "latency s"],
            [(x["run"], x["kind"], x["id"], x["sent_utc"], x["targets"], x["latency_s"]) for x in rtt_rows]))
    md.append("""
Package caveats: no beacon messages were sent in these flights (ground counted 0); the SafePassage reports in the
OCS log came from the USV. No onboard .ulg, so latency is measured at the ground (round trip / forwarding only).
The 11-13 runs include the crash flight. Router CSV and OCS ran on the same laptop clock.
""")
    return md


# ── main ──────────────────────────────────────────────────────────────────────

def session_window(sdir):
    d = dt.datetime.strptime(sdir.name, "%Y%m%d_%H%M%S")
    mt = [p.stat().st_mtime for p in sdir.glob("*.log")]
    return d, d.timestamp(), (max(mt) if mt else d.timestamp())


def agg(rows, key, fn=sum):
    v = [x[key] for x in rows if x.get(key) is not None and x.get(key) != ""]
    return fn(v) if v else None


def md_table(headers, rows):
    s = "| " + " | ".join(headers) + " |\n|" + "---|" * len(headers) + "\n"
    for row in rows:
        s += "| " + " | ".join("" if c is None else str(c) for c in row) + " |\n"
    return s


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--logs", type=Path, default=DEFAULT_LOGS)
    ap.add_argument("--tlog-dir", type=Path, action="append",
                    help=f"tlog folder (repeatable; default {DEFAULT_TLOG_DIR})")
    ap.add_argument("--max-tlog-mb", type=float, default=200.0, help="skip tlogs larger than this")
    ap.add_argument("--out", type=Path, default=REPO / "analysis")
    ap.add_argument("--package", type=Path, action="append",
                    help="competition log package folder (router_wsl*.csv, ocs_*.log, ground.log, qgc_tlogs/); repeatable")
    a = ap.parse_args()

    out_dir = a.out / ("comm_phases_" + dt.datetime.now().strftime("%Y%m%d_%H%M%S"))
    out_dir.mkdir(parents=True, exist_ok=False)          # never reuse / overwrite

    # ── sessions ──
    sessions = []
    for sdir in sorted(p for p in a.logs.iterdir() if p.is_dir()):
        try:
            day, t0, t1 = session_window(sdir)
        except ValueError:
            continue
        row = {"session": sdir.name, "start_epoch": t0, "end_epoch": t1, "_day": day}
        row.update(parse_router(read_lines(sdir / "router.log")))
        row.update(parse_ground(read_lines(sdir / "ground.log")))
        row.update(parse_ocs(read_lines(sdir / "ocs.log")))
        row.update(parse_broker(read_lines(sdir / "broker.log")))
        if row.get("o_last_t"):
            row["end_epoch"] = max(row["end_epoch"], row["o_last_t"])
        sessions.append(row)
    if not sessions:
        sys.exit(f"no sessions under {a.logs}")
    days = {s["_day"].date() for s in sessions}

    # ── tlogs ──
    tdirs = a.tlog_dir or [DEFAULT_TLOG_DIR]
    flights, rs_acks, rst_acks, skipped = [], defaultdict(list), defaultdict(list), []
    for tdir in tdirs:
        for p in sorted(Path(tdir).glob("*.tlog")):
            mm = RE_TLOG_NAME.search(p.name)
            if not mm:
                continue
            d = dt.date(int(mm.group(1)), int(mm.group(2)), int(mm.group(3)))
            if not any(abs((d - sd).days) <= 1 for sd in days):
                continue
            if p.stat().st_size > a.max_tlog_mb * 1e6:
                skipped.append(p.name); continue
            print(f"[tlog] {p.name} ...", flush=True)
            msgs = scan_tlog(p)
            if not msgs:
                continue
            ra, rsa = first_acks(msgs)
            for k, v in ra.items():
                rs_acks[k].extend(v)
            for k, v in rsa.items():
                rst_acks[k].extend(v)
            for i, (f0, f1) in enumerate(tlog_segments(msgs), 1):
                if f1 - f0 < 5:
                    continue          # arm/disarm blips
                row = {"tlog": p.name, "flight": i,
                       "armed_at": dt.datetime.fromtimestamp(f0).isoformat(timespec="seconds"),
                       "_t0": f0, "_t1": f1}
                row.update(window_stats(msgs, f0, f1))
                sess = next((s for s in sessions if s["start_epoch"] - 60 <= f0 <= s["end_epoch"]), None)
                row["session"] = sess["session"] if sess else ""
                if sess:
                    sess["_flight"] = True
                    hb_in = [t for t in sess.get("_hb_t", []) if f0 <= t <= f1]
                    st = gap_stats(hb_in, "ocs_hb_inflight")
                    row.update(st)
                flights.append(row)

    # ── phase membership ──
    for s in sessions:
        s["is_flight"] = bool(s.get("_flight") or s["g_armed"])
        s["has_ocs"] = s.get("o_hb_count", 0) > 0
        s["phase"] = ",".join(p for p, ok in (
            ("2", s["is_flight"] and s.get("router_intervals", 0) > 0),
            ("3", (not s["is_flight"]) and s["has_ocs"]),
            ("4", s["is_flight"] and s["has_ocs"])) if ok)

    # ── round trips ──
    rtt = []
    for s in sessions:
        for run_id, t in s.get("_runstarts", []):
            cand = [x for x in rs_acks.get(run_id, []) if 0 <= x - t <= RTT_MAX_S]
            rtt.append({"session": s["session"], "phase": s["phase"], "kind": "RunStart (RoboCommand->OCS->UAV->ACK at QGC)",
                        "id": run_id, "sent": dt.datetime.fromtimestamp(t).isoformat(timespec="milliseconds"),
                        "rtt_s": r(min(cand) - t) if cand else None})
        for rid in s.get("_restart_ids", []):
            t = local_epoch(s["_day"], int(rid[:2]), int(rid[2:4]), int(rid[4:]))
            cand = [x for x in rst_acks.get(rid, []) if -1 <= x - t <= RTT_MAX_S]
            rtt.append({"session": s["session"], "phase": s["phase"], "kind": "Restart (ground->UAV->ACK at QGC, 1 s resolution)",
                        "id": rid, "sent": dt.datetime.fromtimestamp(t).isoformat(timespec="seconds"),
                        "rtt_s": r(min(cand) - t, 1) if cand else None})

    # ── write CSVs ──
    def write_csv(name, rows):
        keys = []
        for row in rows:
            for k in row:
                if not k.startswith("_") and k not in keys:
                    keys.append(k)
        with open(out_dir / name, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
            w.writeheader()
            for row in rows:
                w.writerow({k: row.get(k) for k in keys})

    write_csv("sessions.csv", sessions)
    write_csv("flights.csv", flights)
    write_csv("roundtrip.csv", rtt)

    # ── summary.md ──
    P2 = [s for s in sessions if "2" in s["phase"]]
    P3 = [s for s in sessions if "3" in s["phase"]]
    P4 = [s for s in sessions if "4" in s["phase"]]
    md = [f"# Communication metrics, phases 2-4\n\nGenerated {dt.datetime.now():%Y-%m-%d %H:%M} from "
          f"{len(sessions)} ground-GUI sessions and {len({f['tlog'] for f in flights})} tlogs with flights.\n"]

    def router_block(rows):
        valid = agg(rows, "router_valid_frames") or 0
        lost = agg(rows, "router_lost_frames") or 0
        dur = agg(rows, "router_duration_s") or 0
        return [
            ("Router time logged (s)", dur),
            ("Frames received / s", r(valid / dur, 2) if dur else None),
            ("Seq-gap loss % (all PX4 traffic, includes onboard rate limiting)",
             r(100.0 * lost / (valid + lost), 1) if valid + lost else None),
            ("Corrupted bytes", agg(rows, "router_bad_bytes")),
            ("5 s intervals with no frames after first contact (includes drone powered off)",
             agg(rows, "router_zero_frame_intervals_while_link_seen")),
            ("Frames via ELRS / WiFi", f"{agg(rows, 'router_elrs_frames')} / {agg(rows, 'router_wifi_frames')}"),
            ("Cross-link duplicates removed", agg(rows, "router_wifi_dup_dropped")),
            ("Uplink switches ELRS<->WiFi", agg(rows, "router_uplink_switches")),
        ]

    # Phase 2
    md.append(f"## Phase 2 - flight, UAV -> ground\n\nSessions: {len(P2)}. Armed segments in tlogs: {len(flights)}.\n")
    if flights:
        ft = sum(f["duration_s"] or 0 for f in flights)
        md.append(md_table(["Metric", "Value"], [
            ("Total armed time (s)", r(ft, 1)),
            ("UAV messages / s (mean of flights)", r(statistics.mean([f["uav_msgs_per_s"] for f in flights if f["uav_msgs_per_s"]]), 2)),
            ("PX4 seq-gap loss %, median of flights (includes rate limiting)", r(statistics.median([f["px4_seq_loss_pct"] for f in flights if f["px4_seq_loss_pct"] is not None]), 2)),
            ("Seabird status msgs received", sum(f["seabird_msgs"] for f in flights)),
            ("Seabird seq-gap loss % (median of flights)", r(statistics.median([f["seabird_seq_loss_pct"] for f in flights if f["seabird_seq_loss_pct"] is not None]), 2) if any(f["seabird_seq_loss_pct"] is not None for f in flights) else None),
            ("PX4 heartbeat gap max (s)", max((f.get("px4_hb_gap_max_s") or 0) for f in flights)),
            ("Longest silence, any UAV message (s)", max((f.get("any_msg_gap_max_s") or 0) for f in flights)),
        ]))
        md.append("\nPer flight:\n\n" + md_table(
            ["tlog", "armed at", "dur s", "msgs/s", "PX4 loss %", "Seabird loss %", "HB gap max s", "OCS HB in flight", "OCS HB gap max s"],
            [(f["tlog"], f["armed_at"], f["duration_s"], f["uav_msgs_per_s"], f["px4_seq_loss_pct"],
              f["seabird_seq_loss_pct"], f.get("px4_hb_gap_max_s"), f.get("ocs_hb_inflight_count"),
              f.get("ocs_hb_inflight_gap_max_s")) for f in flights]))
    md.append("\nRouter, flight sessions:\n\n" + md_table(["Metric", "Value"], router_block(P2)))
    md.append("\nGround bridge heartbeat gaps (flight sessions, whole session incl. drone powered off/rebooting): "
              f"{agg(P2, 'g_link_gaps_recovered') or 0} recovered, longest {agg(P2, 'g_link_gap_max_s', max)} s. "
              "In-flight gaps are the 'HB gap max' column above.\n")

    def ocs_block(rows):
        rs_rel, rs_ack = agg(rows, "g_runstart_relayed") or 0, agg(rows, "g_runstart_acked") or 0
        rq, rqa = agg(rows, "g_restart_requests") or 0, agg(rows, "g_restart_acked") or 0
        hb = [s for s in rows if s.get("o_hb_rate_hz")]
        return [
            ("Status heartbeats to RoboCommand", agg(rows, "o_hb_count")),
            ("Heartbeat rate Hz (median of sessions)", r(statistics.median([s["o_hb_rate_hz"] for s in hb]), 3) if hb else None),
            ("Heartbeat gap p95 / max (s)", f"{agg(rows, 'o_hb_gap_p95_s', max)} / {agg(rows, 'o_hb_gap_max_s', max)}"),
            ("Heartbeat gaps > 2 s", agg(rows, "o_hb_gaps_over_2s")),
            ("Heartbeats suppressed as stale", agg(rows, "o_stale_suppressed")),
            ("Heartbeats with NaN fields", agg(rows, "o_heartbeats_nan")),
            ("MQTT (re)connects", agg(rows, "o_mqtt_connects")),
            ("RunStart received from RoboCommand", agg(rows, "o_runstarts_received")),
            ("RunStart relayed to UAV / acknowledged", f"{rs_rel} / {rs_ack} ({r(100 * rs_ack / rs_rel, 1) if rs_rel else '-'} %)"),
            ("Restart requests / acknowledged", f"{rq} / {rqa}"),
            ("Geofence uploads / accepted / VALID / FAILED",
             f"{agg(rows, 'g_fence_uploads')} / {agg(rows, 'g_fence_accepted')} / {agg(rows, 'g_fence_valid')} / {agg(rows, 'g_fence_failed')}"),
            ("RunDeclarations sent / blocked", f"{agg(rows, 'o_rundeclarations_sent')} / {agg(rows, 'o_rundeclarations_not_sent')}"),
            ("SafePassage reports forwarded to RoboCommand + USV", agg(rows, "o_safepassage_forwarded")),
            ("Task commands forwarded to USV", agg(rows, "o_usv_commands_forwarded")),
            ("OCS runs (restarts split at > 300 s silence)", agg(rows, "o_hb_runs")),
            ("OCS ERROR lines", agg(rows, "o_errors")),
            ("OCS tracebacks: total / at shutdown (MQTT on_disconnect signature bug)",
             f"{agg(rows, 'o_tracebacks')} / {agg(rows, 'o_tracebacks_at_shutdown')}"),
            ("Broker client connects / disconnects / timeouts",
             f"{agg(rows, 'b_client_connects')} / {agg(rows, 'b_client_disconnects')} / {agg(rows, 'b_client_timeouts')}"),
        ]

    def rtt_block(phase):
        rows = []
        for kind in sorted({x["kind"] for x in rtt}):
            v = sorted(x["rtt_s"] for x in rtt if x["kind"] == kind and phase in x["phase"] and x["rtt_s"] is not None)
            n = sum(1 for x in rtt if x["kind"] == kind and phase in x["phase"])
            if n:
                rows.append((kind, n, len(v), r(v[0]) if v else None, r(statistics.median(v)) if v else None,
                             r(pct(v, 95)) if v else None, r(v[-1]) if v else None))
        return md_table(["Round trip", "requests", "matched", "min s", "median s", "p95 s", "max s"], rows) if rows else "No matched round trips.\n"

    md.append(f"\n## Phase 3 - bench, UAV -> ground -> OCS/USV\n\nSessions: {len(P3)}.\n\n"
              + md_table(["Metric", "Value"], ocs_block(P3)) + "\n" + rtt_block("3"))
    md.append(f"\n## Phase 4 - flight, UAV -> ground -> OCS/USV\n\nSessions: {len(P4)}.\n\n"
              + md_table(["Metric", "Value"], ocs_block(P4)) + "\n" + rtt_block("4"))

    md.append("""
## Limitations

- No beacon messages were sent in flight; phases 2 and 4 measure the link and status/command traffic, not beacon delivery.
- The USV's own receipt is not logged here; measurements stop at the OCS output to RoboCommand MQTT / USV guidance topics.
- Over ELRS the UAV rate-limits PX4 telemetry on purpose, so PX4 sequence-gap "loss" mixes intentional drops with radio loss.
  Seabird component messages (191/192) are the better radio-loss indicator.
- Round-trip times compare OCS timestamps (WSL clock) with QGC tlog arrival times (Windows clock); they assume the clocks agree.
  Restart round trips have 1 s resolution (the request id is HHMMSS).
- tlogs exist only while QGC was running; tlogs larger than --max-tlog-mb were skipped: """ + (", ".join(skipped) or "none") + "\n")
    for pkg in a.package or []:
        md.extend(analyze_package(pkg, out_dir, write_csv))
    (out_dir / "summary.md").write_text("\n".join(md), encoding="utf-8")
    print(f"\nWrote {out_dir}")


if __name__ == "__main__":
    main()
