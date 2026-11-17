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
  tests/elrs_testing/analysis/comm_phases_<YYYYmmdd_HHMMSS>/
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
    first = next((float(m.group(1)) for m in (re.match(r"^(\d{9,11}):", l) for l in lines) if m), None)
    conn = sum(1 for l in lines if "New client connected" in l)
    disc = sum(1 for l in lines if re.search(r"Client \S+ (closed its connection|disconnected)", l))
    tout = sum(1 for l in lines if "has exceeded timeout" in l)
    return {"b_client_connects": conn, "b_client_disconnects": disc, "b_client_timeouts": tout, "b_first_t": first}


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


PKG_DATA = []    # raw arrays kept for the charts, one dict per --package


def analyze_package(pkg, out_dir, write_csv):
    pkg = Path(pkg)
    keep = {"name": pkg.name, "runs": {}, "ocs": {}}
    PKG_DATA.append(keep)
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
        keep["runs"][label] = {"rows": rows, "wins": wins3}
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
        keep["ocs"][label] = {"hb": hb, "runstarts": runstarts}
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

    keep.update({"link_rows": link_rows, "ocs_rows": ocs_rows, "rtt_rows": rtt_rows, "acks": acks})
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


# ── charts (style copied from tests/elrs_testing/evaluate_beacon_tests.py) ───

SURFACE, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
STATUS = {"good": "#0ca30c", "warning": "#fab219", "serious": "#ec835a", "critical": "#d03b3b"}


def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    installed = {f.name for f in font_manager.fontManager.ttflist}
    family = next((f for f in ("Segoe UI", "Helvetica", "Arial") if f in installed), "DejaVu Sans")
    plt.rcParams.update({
        "font.family": [family, "DejaVu Sans"],
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
        "axes.edgecolor": AXIS, "axes.labelcolor": INK2, "xtick.color": MUTED, "ytick.color": MUTED,
        "xtick.labelcolor": INK2, "ytick.labelcolor": INK2, "text.color": INK,
        "axes.titlesize": 11, "axes.labelsize": 9, "xtick.labelsize": 8.5, "ytick.labelsize": 8.5,
        "legend.fontsize": 8.5, "legend.frameon": False, "savefig.dpi": 150,
    })
    return plt


def _style(ax, title=None, ylabel=None, xlabel=None, grid="y"):
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    if grid:
        ax.grid(axis=grid, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    if title:
        ax.set_title(title, loc="left", color=INK, fontweight="semibold", pad=8)
    if ylabel:
        ax.set_ylabel(ylabel)
    if xlabel:
        ax.set_xlabel(xlabel)


def _num(v):
    try:
        return float(v) if v is not None and v != "" else float("nan")
    except (TypeError, ValueError):
        return float("nan")


def _labels(ax, x, vals, fmt, pad_frac=0.02, color=INK2):
    import math
    top = max([v for v in vals if not math.isnan(v)] or [1])
    dense = len(vals) > 15
    fs, rot = (6.5, 90) if dense else (8, 0)
    for xi, v in zip(x, vals):
        if math.isnan(v):
            ax.text(xi, top * 0.03, "n/a", ha="center", va="bottom", fontsize=fs, color=MUTED, rotation=rot)
        else:
            ax.text(xi, v + top * pad_frac, fmt.format(v), ha="center", va="bottom", fontsize=fs, color=color,
                    rotation=rot)


def _ref(ax, x, y, label):
    ax.plot(x, [y] * len(x), linestyle="none", marker="_", markersize=30, markeredgewidth=2, color=INK, label=label)


def _day(t):
    return dt.datetime.fromtimestamp(t, tz=dt.timezone.utc).strftime("%m-%d")


def chart_competition(out_dir, keep):
    import math
    import numpy as np
    from matplotlib.patches import Patch
    plt = _plt()
    days = sorted({_day(w[0]) for run in keep["runs"].values() for w in run["wins"]})
    day_color = {d: SERIES[i % len(SERIES)] for i, d in enumerate(days)}

    # segments: every airborne window >= 5 s, plus "on ground" for runs that flew
    seg, nth = [], defaultdict(int)
    for x in keep["link_rows"]:
        if x["segment"].startswith("airborne") and (x["duration_s"] or 0) >= 5:
            nth[x["from_utc"][5:10]] += 1
            seg.append((x, f"{x['from_utc'][5:10]}\nflight {nth[x['from_utc'][5:10]]}\n{x['from_utc'][11:16]}Z", True))
    flown = {x["run"] for x, _, _ in seg}
    for x in keep["link_rows"]:
        if x["segment"].startswith("on ground") and x["run"] in flown:
            seg.append((x, f"{x['from_utc'][5:10]}\nground\n{x['from_utc'][11:16]}Z log", False))
    if not seg:
        return []
    xs = np.arange(len(seg))
    cols = [day_color.get(s[0]["from_utc"][5:10], SERIES[2]) for s in seg]
    alph = [1.0 if s[2] else 0.45 for s in seg]
    names = [s[1] for s in seg]
    V = lambda k: [_num(s[0].get(k)) for s in seg]

    fig, axes = plt.subplots(2, 3, figsize=(16, 9.5))

    def bars(ax, vals, title, ylabel, fmt, ref=None, ref_label=None, flights_only=False):
        idx = [i for i in range(len(seg)) if seg[i][2] or not flights_only]
        px = np.arange(len(idx))
        v2 = [vals[i] for i in idx]
        for xi, i, v in zip(px, idx, v2):
            ax.bar(xi, 0 if math.isnan(v) else v, 0.6, color=cols[i], alpha=alph[i])
        _labels(ax, px, v2, fmt)
        if ref is not None:
            _ref(ax, px, ref, ref_label)
            ax.legend(loc="upper right")
        ax.set_xticks(px, [names[i] for i in idx])
        _style(ax, title, ylabel)

    bars(axes[0, 0], V("msgs_per_s"), "Messages received from the UAV", "messages per second", "{:.1f}")
    bars(axes[0, 1], V("bad_per_100_good"), "Corrupted frames", "per 100 good frames", "{:.1f}")

    ax = axes[0, 2]
    w = 0.38
    sb, px = V("seabird_seq_loss_pct"), V("px4_seq_loss_pct")
    ax.bar(xs - w / 2, [0 if math.isnan(v) else v for v in sb], w, color=SERIES[0], label="Seabird status msgs lost (radio)")
    ax.bar(xs + w / 2, [0 if math.isnan(v) else v for v in px], w, color=SERIES[1],
           label="PX4 telemetry gaps (incl. onboard rate limit)")
    _labels(ax, xs - w / 2, sb, "{:.0f}%")
    ax.set_xticks(xs, names)
    ax.set_ylim(0, 115)
    ax.legend(loc="upper left")
    _style(ax, "Message loss (sequence gaps)", "% of messages")

    bars(axes[1, 0], V("hb_gap_max_s"), "Worst outage in flight (longest UAV heartbeat gap)", "seconds", "{:.1f}s",
         ref=1.0, ref_label="Heartbeat period (1 s)", flights_only=True)

    ax = axes[1, 1]
    orows = keep["ocs_rows"]
    scopes = list(dict.fromkeys(x["scope"] for x in orows))
    vehs = sorted({x["vehicle"] for x in orows})
    if orows:
        sx = np.arange(len(scopes))
        w = 0.8 / max(1, len(vehs))
        for i, v in enumerate(vehs):
            vals = [_num(next((x.get("hb_rate_hz") for x in orows if x["vehicle"] == v and x["scope"] == s), None)) for s in scopes]
            gmax = [_num(next((x.get("hb_gap_max_s") for x in orows if x["vehicle"] == v and x["scope"] == s), None)) for s in scopes]
            pos = sx - 0.4 + w * (i + 0.5)
            ax.bar(pos, [0 if math.isnan(q) else q for q in vals], w * 0.9, color=SERIES[i], label=v)
            for p, q, g in zip(pos, vals, gmax):
                if not math.isnan(q):
                    ax.text(p, q / 2, f"{q:.2f} Hz\nmax gap\n{g:.1f}s", ha="center", va="center", fontsize=7.5,
                            color="white", fontweight="semibold")
        _ref(ax, sx, 2.0, "Target 2 Hz")
        ax.set_xticks(sx, [s.replace("UAV airborne", "during UAV flight") for s in scopes])
        ax.set_ylim(0, 2.6)
        ax.legend(loc="upper right")
    _style(ax, "OCS heartbeats to RoboCommand", "heartbeats per second (Hz)")

    ax = axes[1, 2]
    rr = [x for x in keep["rtt_rows"] if x.get("latency_s") is not None]
    if rr:
        ys = np.arange(len(rr))
        vals = [x["latency_s"] for x in rr]
        ax.barh(ys, vals, 0.5, color=[SERIES[2] if "forwarded" in x["kind"] else SERIES[0] for x in rr])
        for y, v in zip(ys, vals):
            ax.text(v + max(vals) * 0.02, y, f"{v * 1000:.0f} ms" if v < 1 else f"{v:.2f} s", va="center", fontsize=9, color=INK2)
        ax.set_yticks(ys, [("RoboCommand run_start\n-> USV + UAV (OCS)" if "forwarded" in x["kind"]
                            else f"RunStart {x['id']} -> UAV\n-> ACK at ground") for x in rr])
        ax.set_xlim(0, max(vals) * 1.3)
        ax.invert_yaxis()
    _style(ax, "Command latency (competition run)", xlabel="seconds", grid="x")

    handles = [Patch(color=c, label=f"{d} (UTC)") for d, c in day_color.items()]
    handles.append(Patch(color=MUTED, alpha=0.45, label="faded = drone on the ground"))
    fig.legend(handles=handles, loc="upper right", ncol=len(handles), bbox_to_anchor=(0.99, 0.985))
    fig.suptitle("UAV -> ground -> RoboCommand/USV: competition runs", x=0.01, y=0.985, ha="left",
                 fontsize=15, fontweight="semibold", color=INK)
    fig.text(0.01, 0.005,
             "Flight = GLOBAL_POSITION_INT relative altitude > 1 m (windows padded 1 s). "
             "Seabird status = drone mission/gate messages (comp 191/192), not rate-limited: best radio-loss measure.\n"
             "No beacon messages were sent in these flights. 11-13 includes the crash flight. "
             "Source: router per-message CSV, OCS log, QGC tlogs from the log package.",
             fontsize=8, color=INK2, va="bottom", ha="left")
    fig.tight_layout(rect=(0, 0.04, 1, 0.95))
    fig.savefig(out_dir / "competition_overview.png")
    plt.close(fig)
    return ["competition_overview.png"]


def chart_timeline(out_dir, keep):
    import numpy as np
    import matplotlib.dates as mdates
    plt = _plt()
    runs = [(k, v) for k, v in keep["runs"].items() if any(b - a >= 5 for a, b, n in v["wins"])]
    if not runs:
        return []
    fig, axes = plt.subplots(len(runs), 2, figsize=(16, 3.6 * len(runs)), squeeze=False,
                             gridspec_kw={"width_ratios": [1.6, 1]})
    tz = dt.timezone.utc
    D = lambda t: dt.datetime.fromtimestamp(t, tz=tz)
    for row, (label, run) in enumerate(runs):
        rows = run["rows"]
        good = np.array([x["t"] for x in rows if x["status"] == "SENT"])
        bad = np.array([x["t"] for x in rows if x["status"] != "SENT"])
        acks = [x["t"] for x in rows if (x.get("text") or "").startswith("SEABIRD_RUNSTART_ACK")]
        sent = [t for (_, t) in keep["ocs"].get(label, {}).get("runstarts", [])]
        fl = [(a, b) for a, b, n in run["wins"] if b - a >= 5]
        zoom = (min(a for a, b in fl) - 60, max(b for a, b in fl) + 60)
        for col, (t0, t1, bin_s) in enumerate([(rows[0]["t"], rows[-1]["t"], 10.0), (zoom[0], zoom[1], 1.0)]):
            ax = axes[row, col]
            edges = np.arange(t0, t1 + bin_s, bin_s)
            if len(edges) < 2:
                continue
            g, _ = np.histogram(good, edges)
            b, _ = np.histogram(bad, edges)
            mids = [D(t) for t in (edges[:-1] + bin_s / 2)]
            ax.plot(mids, g / bin_s, color=SERIES[0], linewidth=1.1, label="Good messages / s")
            ax.plot(mids, b / bin_s, color=STATUS["critical"], linewidth=1.1, label="Corrupted frames / s")
            for a, bb, n in run["wins"]:
                if bb >= t0 and a <= t1:
                    ax.axvspan(D(max(a, t0)), D(min(bb, t1)), color=STATUS["good"], alpha=0.15, linewidth=0)
            for t in sent:
                if t0 <= t <= t1:
                    ax.axvline(D(t), color=INK, linestyle="--", linewidth=1)
            ymax = max(1.0, (g / bin_s).max() if len(g) else 1)
            ack_in = [D(t) for t in acks if t0 <= t <= t1]
            if ack_in:
                ax.plot(ack_in, [ymax * 1.05] * len(ack_in), linestyle="none", marker="v", markersize=7,
                        color=SERIES[2], label="RunStart ACK from UAV")
            ax.set_xlim(D(t0), D(t1))
            ax.set_ylim(0, ymax * 1.15)
            ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M" if col == 0 else "%H:%M:%S", tz=tz))
            title = (f"{label.split('/')[-1]}  —  whole log ({bin_s:.0f} s bins)" if col == 0
                     else f"zoom on flight ({bin_s:.0f} s bins)")
            _style(ax, title, "per second" if col == 0 else None, "UTC" if row == len(runs) - 1 else None)
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    fig.legend(handles=[Line2D([], [], color=SERIES[0], label="Good messages / s"),
                        Line2D([], [], color=STATUS["critical"], label="Corrupted frames / s"),
                        Patch(color=STATUS["good"], alpha=0.15, label="UAV airborne"),
                        Line2D([], [], color=INK, linestyle="--", label="RunStart from RoboCommand (OCS)"),
                        Line2D([], [], marker="v", linestyle="none", color=SERIES[2], label="RunStart ACK from UAV")],
               loc="upper right", ncol=5, bbox_to_anchor=(0.99, 0.995))
    fig.suptitle("UAV -> ground link over time", x=0.01, y=0.995, ha="left", fontsize=15,
                 fontweight="semibold", color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_dir / "competition_timeline.png")
    plt.close(fig)
    return ["competition_timeline.png"]


def chart_intervals(out_dir, keep, sessions, flights):
    import numpy as np
    plt = _plt()
    panels = []    # (title, intervals, nominal period)
    for label, run in keep["runs"].items():
        fl = [(a, b) for a, b, n in run["wins"] if b - a >= 5]
        if not fl:
            continue
        hb = [x["t"] for x in run["rows"] if x["status"] == "SENT" and x["type"] == "HEARTBEAT" and x["comp"] == "1"
              and any(a <= x["t"] <= b for a, b in fl)]
        iv = []                       # per flight, so the gap between two flights is not counted
        for a, b in fl:
            iv += gaps([t for t in hb if a <= t <= b])
        panels.append((f"UAV heartbeat at ground, in flight ({_day(fl[0][0])})", iv, 1.0))
    for label, o in keep["ocs"].items():
        fl = [(a, b) for r in keep["runs"].values() for a, b, n in r["wins"] if b - a >= 5]
        for veh, ts in sorted(o["hb"].items()):
            iv = [g for g in gaps(ts) if g <= RUN_SPLIT_S]
            panels.append((f"OCS {veh} heartbeat to RoboCommand, whole run ({_day(ts[0])})", iv, 0.5))
    lap, n_used = [], 0
    for f in flights:
        s = next((s for s in sessions if s["session"] == f.get("session")), None)
        if s:
            g = gaps([t for t in s.get("_hb_t", []) if f["_t0"] <= t <= f["_t1"]])
            n_used += 1 if g else 0
            lap += g
    if lap:
        panels.append((f"OCS UAV1 heartbeat during laptop-log flights (Nov 9-13, {n_used} flights with OCS running)",
                       lap, 0.5))
    panels = [p for p in panels if len(p[1]) >= 2]
    if not panels:
        return []
    cols = 2
    rows_n = (len(panels) + 1 + cols - 1) // cols      # +1 slot for the explanation note
    fig, axes = plt.subplots(rows_n, cols, figsize=(15, 2.9 * rows_n), squeeze=False)
    for ax in axes.flat:
        ax.set_visible(False)
    for ax, (title, iv, nominal) in zip(axes.flat, panels):
        ax.set_visible(True)
        iv = np.array(iv)
        xmax = max(3 * nominal, min(float(np.percentile(iv, 99.5)) * 1.2, 10.0))
        over = int((iv > xmax).sum())
        ax.hist(np.clip(iv, 0, xmax), bins=np.linspace(0, xmax, 60), color=SERIES[0], rwidth=0.85)
        ax.axvline(nominal, color=INK, linewidth=2)
        ax.text(nominal, ax.get_ylim()[1] * 0.92, f" expected period {nominal:g}s", fontsize=8, color=INK)
        hz = len(iv) / iv.sum() if iv.sum() > 0 else float("nan")
        _style(ax, f"{title}\n{hz:.2f} Hz achieved, max gap {iv.max():.2f}s"
                   + (f", {over} gap(s) > {xmax:.1f}s shown at the edge" if over else ""), "count")
    axes.flat[len(panels) - 1].set_xlabel("seconds between consecutive messages")
    note = axes.flat[len(panels)] if len(panels) < rows_n * cols else None
    if note is not None:
        note.set_visible(True)
        note.axis("off")
        note.text(0.5, 0.5, "Bars: time between consecutive messages at the ground.\n"
                            "Mass to the right of the expected-period line = missed / late updates.",
                  ha="center", va="center", fontsize=10, color=INK2, transform=note.transAxes)
    fig.suptitle("Update intervals", x=0.01, y=0.995, ha="left", fontsize=15, fontweight="semibold", color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out_dir / "update_intervals.png")
    plt.close(fig)
    return ["update_intervals.png"]


def chart_laptop_flights(out_dir, flights, sessions, rtt):
    import math
    import numpy as np
    from matplotlib.patches import Patch
    plt = _plt()
    if not flights:
        return []
    fl = sorted(flights, key=lambda f: f["_t0"])
    sess = {s["session"]: s for s in sessions}

    def link(f):
        s = sess.get(f.get("session"))
        if not s:
            return "no ground session"
        return "WiFi + ELRS" if (s.get("router_wifi_frames") or 0) > 0 else "ELRS only"

    kinds = ["ELRS only", "WiFi + ELRS", "no ground session"]
    kc = {"ELRS only": SERIES[0], "WiFi + ELRS": SERIES[1], "no ground session": AXIS}
    cols = [kc[link(f)] for f in fl]
    xs = np.arange(len(fl))
    names = [utc(f["_t0"])[5:16].replace(" ", " ") for f in fl]     # "MM-DD HH:MM" UTC
    V = lambda k: [_num(f.get(k)) for f in fl]

    fig, axes = plt.subplots(2, 3, figsize=(17, 9.5))

    def bars(ax, vals, title, ylabel, fmt, ref=None, ref_label=None):
        ax.bar(xs, [0 if math.isnan(v) else v for v in vals], 0.7, color=cols)
        _labels(ax, xs, vals, fmt)
        if ref is not None:
            ax.axhline(ref, color=INK, linewidth=1.5, label=ref_label)
            ax.legend(loc="upper right")
        ax.set_xticks(xs, names, fontsize=6.5, rotation=90)
        _style(ax, title, ylabel)

    bars(axes[0, 0], V("uav_msgs_per_s"), "Messages received from the UAV", "messages per second", "{:.0f}")
    bars(axes[0, 1], V("seabird_seq_loss_pct"), "Seabird status messages lost (radio loss)", "% of messages", "{:.0f}")
    bars(axes[0, 2], V("px4_hb_gap_max_s"), "Worst outage (longest UAV heartbeat gap)", "seconds", "{:.1f}",
         ref=1.0, ref_label="Heartbeat period (1 s)")
    bars(axes[1, 0], V("ocs_hb_inflight_gap_max_s"), "OCS -> RoboCommand: longest heartbeat gap in flight",
         "seconds", "{:.2f}", ref=0.5, ref_label="Heartbeat period (0.5 s)")

    for ax, kind, title in ((axes[1, 1], "RunStart", "RunStart: RoboCommand -> OCS -> UAV -> ACK"),
                            (axes[1, 2], "Restart", "Restart: ground -> UAV -> ACK (1 s resolution)")):
        groups = [("Phase 3\nbench", "3"), ("Phase 4\nflight", "4")]
        for gi, (gname, ph) in enumerate(groups):
            v = np.array([x["rtt_s"] for x in rtt if x["kind"].startswith(kind) and ph in x["phase"] and x["rtt_s"] is not None])
            n = sum(1 for x in rtt if x["kind"].startswith(kind) and ph in x["phase"])
            if len(v):
                jitter = (np.random.default_rng(0).random(len(v)) - 0.5) * 0.25
                ax.scatter(gi + jitter, v, s=22, color=SERIES[gi], edgecolors=SURFACE, linewidths=0.6, zorder=3)
                med = float(np.median(v))
                ax.plot([gi - 0.25, gi + 0.25], [med, med], color=INK, linewidth=2)
                ax.text(gi + 0.28, med, f"median {med:.2f}s", va="center", fontsize=8, color=INK2)
            ax.text(gi, 0.02, f"{len(v)} of {n} matched", transform=ax.get_xaxis_transform(), ha="center",
                    fontsize=8, color=MUTED)
        ax.set_yscale("log")
        ax.set_xticks([0, 1], [g[0] for g in groups])
        ax.set_xlim(-0.6, 1.9)
        _style(ax, title, "seconds (log scale)")

    fig.legend(handles=[Patch(color=kc[k], label=k) for k in kinds], loc="upper right", ncol=3,
               bbox_to_anchor=(0.99, 0.985))
    fig.suptitle("Laptop-log flights, Nov 9-13 (phases 2 and 4)", x=0.01, y=0.985, ha="left", fontsize=15,
                 fontweight="semibold", color=INK)
    fig.text(0.01, 0.005,
             "One bar per armed segment in the QGC tlogs (armed time, UTC). Colour = link used in that ground session "
             "(router saw WiFi frames or not). n/a = no Seabird status messages / no OCS in that flight.\n"
             "Long round trips include waiting for the drone's mission script to start, not only transit time.",
             fontsize=8, color=INK2, va="bottom", ha="left")
    fig.tight_layout(rect=(0, 0.04, 1, 0.95))
    fig.savefig(out_dir / "laptop_flights_overview.png")
    plt.close(fig)
    return ["laptop_flights_overview.png"]


def make_charts(out_dir, sessions, flights, rtt):
    made = []
    for name, fn in [("laptop flights", lambda: chart_laptop_flights(out_dir, flights, sessions, rtt))] + \
            [(f"competition {k['name']}", (lambda k=k: chart_competition(out_dir, k) + chart_timeline(out_dir, k)
                                            + chart_intervals(out_dir, k, sessions, flights))) for k in PKG_DATA]:
        try:
            made += fn()
        except Exception as e:      # a chart problem must not lose the tables/CSVs
            import traceback
            print(f"[charts] {name} failed: {e}")
            traceback.print_exc()
    return made


# ── main ──────────────────────────────────────────────────────────────────────

SITE_UTC_OFFSETS_H = (8, -5, -4)   # Singapore (SGT), Florida (EST / EDT): where the GUI ran


def session_window(sdir, first_epoch=None, last_epoch=None):
    """Session folder names are the GUI laptop's LOCAL time, and the laptop changed time zone
    between Florida and Singapore. Pick the site offset that puts the folder time just before the
    first timestamped event (OCS / broker), else just before the newest log file's mtime."""
    d = dt.datetime.strptime(sdir.name, "%Y%m%d_%H%M%S")
    mt = [p.stat().st_mtime for p in sdir.glob("*.log")]
    ref = first_epoch if first_epoch is not None else (max(mt) if mt else None)
    naive_utc = d.replace(tzinfo=dt.timezone.utc).timestamp()
    if ref is None:
        off = SITE_UTC_OFFSETS_H[0]
    else:
        # implied offset; an OCS log can start before the folder (OCS already running), so take the
        # nearest site offset rather than requiring the log to start after the folder time
        implied_h = (naive_utc - ref) / 3600.0
        off = min(SITE_UTC_OFFSETS_H, key=lambda o: abs(o - implied_h))
    start = naive_utc - off * 3600
    if first_epoch is not None:
        start = min(start, first_epoch)
    end = last_epoch if last_epoch is not None else (max(mt) if mt else start)
    return d, start, max(end, start), off


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
    ap.add_argument("--out", type=Path, default=REPO / "tests" / "elrs_testing" / "analysis")
    ap.add_argument("--package", type=Path, action="append",
                    help="competition log package folder (router_wsl*.csv, ocs_*.log, ground.log, qgc_tlogs/); repeatable")
    a = ap.parse_args()

    out_dir = a.out / ("comm_phases_" + dt.datetime.now().strftime("%Y%m%d_%H%M%S"))
    out_dir.mkdir(parents=True, exist_ok=False)          # never reuse / overwrite

    # ── sessions ──
    sessions = []
    for sdir in sorted(p for p in a.logs.iterdir() if p.is_dir()):
        if not re.fullmatch(r"\d{8}_\d{6}", sdir.name):
            continue
        row = {"session": sdir.name}
        row.update(parse_router(read_lines(sdir / "router.log")))
        row.update(parse_ground(read_lines(sdir / "ground.log")))
        row.update(parse_ocs(read_lines(sdir / "ocs.log")))
        row.update(parse_broker(read_lines(sdir / "broker.log")))
        firsts = [x for x in (row.get("o_first_t"), row.get("b_first_t")) if x]
        day, t0, t1, off = session_window(sdir, min(firsts) if firsts else None, row.get("o_last_t"))
        row.update({"start_epoch": t0, "end_epoch": t1, "utc_offset_h": off, "_day": day,
                    "start_utc": utc(t0)})
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
                       "armed_at_utc": utc(f0),
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
            # request id is HHMMSS in the GUI laptop's local time on the session date
            off = s.get("utc_offset_h") or 0
            loc = dt.datetime(s["_day"].year, s["_day"].month, s["_day"].day, int(rid[:2]), int(rid[2:4]), int(rid[4:]))
            if loc < s["_day"] - dt.timedelta(hours=1):
                loc += dt.timedelta(days=1)            # session ran past midnight
            t = (loc - dt.timedelta(hours=off)).replace(tzinfo=dt.timezone.utc).timestamp()
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
            ["tlog", "armed at (UTC)", "dur s", "msgs/s", "PX4 loss %", "Seabird loss %", "HB gap max s", "OCS HB in flight", "OCS HB gap max s"],
            [(f["tlog"], f["armed_at_utc"], f["duration_s"], f["uav_msgs_per_s"], f["px4_seq_loss_pct"],
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
    charts = make_charts(out_dir, sessions, flights, rtt)
    if charts:
        md.insert(1, "Charts: " + ", ".join(f"[{c}]({c})" for c in charts) + "\n")
    (out_dir / "summary.md").write_text("\n".join(md), encoding="utf-8")
    print(f"\nWrote {out_dir}")


if __name__ == "__main__":
    main()
