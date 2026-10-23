#!/usr/bin/env python3
"""
evaluate_beacon_tests.py — evaluate the ELRS beacon-relay test runs in this folder.

Each testN/ folder holds the logs from one run of the pipeline

    ROS publisher (UAV) -> beacon_mavlink_bridge (UAV) -> ELRS radio ->
    elrs_windows_router (Windows) -> beacon_mavlink_ground (WSL)

and NOTES.md has a one-line description of the settings used in each run.
Log file names were not kept consistent between runs, so every CSV is
identified by its columns instead of its name:

    source - has `topic`                       what the UAV published (ground truth)
    relay  - has `ros_timestamp_unix_seconds`, what the bridge put on the MAVLink link
             `copy` or `copies`
    router - has `hex_payload`                 every frame the Windows router forwarded
    ground - has `is_blinking` (none of above) what the ground decoder output

For every test it reports
    data accuracy    - decoded lat/lon/color/blink vs. the published detection
    delivery rate    - detections that reached the ground, end-to-end and per hop
    latency          - publish -> ground output, split into UAV / radio / ground hops
    update frequency - achieved ground update rate vs. publish rate, update gaps
    stability        - latency jitter and drift, outages, loss streaks, link health

Results go to <out>/: summary_metrics.csv, per_message.csv, report.md and PNG charts.

Latencies subtract timestamps taken on different machines (UAV, Windows, WSL),
so they include any clock offset between them. Sync the clocks (NTP/chrony)
before reading the absolute numbers literally; comparisons between tests are
still meaningful if the offset stayed put.

Usage:
    python elrs_testing/evaluate_beacon_tests.py
    python elrs_testing/evaluate_beacon_tests.py --root elrs_testing --out elrs_testing/results
"""

import argparse
import re
import struct
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

# Beacon wire format — see utils/beacon_mavlink_protocol.py
BEACON_MAVLINK_ADDRESS = 0xBEAC
MEMORY_VECT_ID = 249
MEMORY_VECT_CRC_EXTRA = 204
MEMORY_VECT_LEN = 36            # address u16, ver u8, type u8, value int8[32]
TRACKING_ID_OFFSET = 4 + 10     # value starts at 4; lat i32, lon i32, color u8, blink u8, then tracking_id
SEQ_MOD = 256

POSITION_EXACT_M = 0.05         # lat/lon travel as int32 x1e7 (~1 cm), so below this is lossless
EARTH_RADIUS_M = 6371000.0
WINDOW_PRE_S = 5.0              # log rows this far outside the publish window are left-overs from other runs
WINDOW_POST_S = 15.0
ROLLING_WINDOW = 10             # messages per rolling delivery-rate window
DEFAULT_COPY_GAP_S = 0.5        # beacon_mavlink_bridge.py --repeat-gap default
TELEMETRY_STREAMS = ["ATTITUDE", "ATTITUDE_QUATERNION", "VFR_HUD"]  # the streams NOTES.md retunes

# Chart palette (validated reference palette — categorical order, status, chrome)
SURFACE, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
STATUS = {"good": "#0ca30c", "warning": "#fab219", "serious": "#ec835a", "critical": "#d03b3b"}


# --------------------------------------------------------------------------- NOTES.md

def test_number(name: str) -> int:
    m = re.search(r"\d+", name)
    return int(m.group()) if m else 0


def parse_notes(path: Path) -> dict:
    notes = {}
    if path.exists():
        for line in path.read_text(encoding="utf-8").splitlines():
            m = re.match(r"\s*(test\d+)\s*[-:]\s*(.+)", line, re.I)
            if m:
                notes[m.group(1).lower()] = m.group(2).strip()
    return notes


def configs_from_notes(notes: dict) -> dict:
    """Settings each note states. Anything a note doesn't mention carries over from the previous test."""
    configs, publish_hz, copies, windows_decode, stream_hz = {}, None, 1, False, {}
    for name in sorted(notes, key=test_number):
        text = notes[name].lower()
        m = re.search(r"publish\w*\s+(?:\w+\s+)?at\s+([\d.]+)\s*hz", text)
        if m:
            publish_hz = float(m.group(1))
        m = re.search(r"(\d+)\s*(?:times|copies)", text)
        if m:
            copies = int(m.group(1))
        elif "twice" in text:
            copies = 2
        elif "three times" in text or "thrice" in text:
            copies = 3
        if "pymavlink" in text and "windows" in text:
            windows_decode = True
        m = re.search(r"change\s+(.+?)\s+frequency.*?to\s+([\d.]+)\s*hz", text)
        if m:
            for stream in re.split(r"\s*(?:,|\band\b)\s*", m.group(1)):
                if stream:
                    stream_hz[stream.strip().upper()] = float(m.group(2))
        configs[name] = {
            "notes": notes[name],
            "notes_publish_hz": publish_hz,
            "notes_copies": copies,
            "dedup": "duplicate" in text,
            "windows_decode": windows_decode,
            "notes_stream_hz": dict(stream_hz),
        }
    return configs


# --------------------------------------------------------------------------- loading

def read_csv(path: Path) -> pd.DataFrame:
    return pd.read_csv(path, dtype=str, keep_default_na=False)


def classify(columns) -> str:
    cols = set(columns)
    if "hex_payload" in cols:
        return "router"
    if "topic" in cols:
        return "source"
    if cols & {"copy", "copies", "ros_timestamp_unix_seconds"}:
        return "relay"
    if "is_blinking" in cols:
        return "ground"
    return ""


def num(series) -> pd.Series:
    return pd.to_numeric(series, errors="coerce")


def blink_str(value) -> str:
    """'True' / 'False' / 'None' — kept as text so None compares equal to None after merges."""
    return {"true": "True", "false": "False"}.get(str(value).strip().lower(), "None")


def norm_source(df):
    return pd.DataFrame({
        "tracking_id": num(df["tracking_id"]).astype(int),
        "t_pub": num(df["epoch_time"]),
        "lat": num(df["latitude"]), "lon": num(df["longitude"]),
        "color": df["color"].str.strip().str.lower(),
        "blink": df["blink"].map(blink_str),
    }).sort_values("t_pub").reset_index(drop=True)


def norm_relay(df):
    tcol = "ros_timestamp_unix_seconds" if "ros_timestamp_unix_seconds" in df else "epoch_time"
    return pd.DataFrame({
        "t": num(df[tcol]),
        "tracking_id": num(df["tracking_id"]),
        "copy": num(df["copy"]) if "copy" in df else 1,
        "msg_id": num(df["msg_id"]) if "msg_id" in df else np.nan,
        "copies": num(df["copies"]) if "copies" in df else np.nan,
        "has_copy_rows": "copy" in df,
    }).dropna(subset=["t", "tracking_id"])


def norm_ground(df):
    return pd.DataFrame({
        "t": num(df["epoch_time"]),
        "tracking_id": num(df["tracking_id"]),
        "lat": num(df["latitude"]), "lon": num(df["longitude"]),
        "color": df["color"].str.strip().str.lower(),
        "blink": df["is_blinking"].map(blink_str),
    }).dropna(subset=["t", "tracking_id"])


def x25_crc(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


_V1_BEACON = re.compile(rb"\xfe(.)(.)(.)(.)\xf9", re.S)                   # len seq sys comp msgid
_V2_BEACON = re.compile(rb"\xfd(.)\x00(.)(.)(.)(.)\xf9\x00\x00", re.S)    # len incompat compat seq sys comp msgid


def beacons_from_raw_stream(df) -> pd.DataFrame:
    """
    Early router logs (test1) stored raw serial chunks, not parsed frames.
    Rebuild the byte stream and pull out CRC-valid beacon MEMORY_VECT frames,
    timestamped by the chunk holding each frame's last byte.
    """
    chunks = [bytes.fromhex(h) for h in df["hex_payload"]]
    ends = np.cumsum([len(c) for c in chunks])
    times = num(df["epoch_time"]).to_numpy()
    stream = b"".join(chunks)
    rows = []
    for pattern, hdr_len in ((_V1_BEACON, 6), (_V2_BEACON, 10)):
        for m in pattern.finditer(stream):
            plen = m.group(1)[0]
            start = m.start()
            end = start + hdr_len + plen + 2
            if plen > MEMORY_VECT_LEN or end > len(stream):
                continue
            crc = int.from_bytes(stream[end - 2:end], "little")
            if x25_crc(stream[start + 1:end - 2] + bytes([MEMORY_VECT_CRC_EXTRA])) != crc:
                continue
            payload = stream[start + hdr_len:end - 2].ljust(MEMORY_VECT_LEN, b"\x00")
            if struct.unpack_from("<H", payload, 0)[0] != BEACON_MAVLINK_ADDRESS:
                continue
            seq = m.group(2)[0] if hdr_len == 6 else m.group(4)[0]
            rows.append({
                "t": times[np.searchsorted(ends, end - 1, side="right")],
                "seq": seq,
                "tracking_id": struct.unpack_from("<i", payload, TRACKING_ID_OFFSET)[0],
            })
    return pd.DataFrame(rows, columns=["t", "seq", "tracking_id"]).sort_values("t")


def norm_router(df):
    """-> (all parsed frames or None for raw logs, beacon frames)"""
    if "msg_type" not in df:
        return None, beacons_from_raw_stream(df)
    frames = pd.DataFrame({
        "t": num(df["epoch_time"]),
        "msg_type": df["msg_type"],
        "src_system": num(df["src_system"]),
        "src_component": num(df["src_component"]),
        "seq": num(df["mav_seq"]),
        "tracking_id": num(df["tracking_id"]),
        "bytes": num(df["bytes"]),
    })
    beacons = frames[(frames.msg_type == "MEMORY_VECT") & frames.tracking_id.notna()]
    return frames, beacons[["t", "seq", "tracking_id"]].sort_values("t")


def load_test(test_dir: Path) -> dict:
    logs = {}
    for path in sorted(test_dir.glob("*.csv")):
        df = read_csv(path)
        role = classify(df.columns)
        if not role:
            print(f"  [{test_dir.name}] skipping {path.name}: unrecognised columns")
        elif role in logs:
            print(f"  [{test_dir.name}] skipping {path.name}: already have a {role} log ({logs[role][0]})")
        else:
            logs[role] = (path.name, df)
    return logs


# --------------------------------------------------------------------------- analysis

def haversine_m(lat1, lon1, lat2, lon2):
    p1, p2 = np.radians(lat1), np.radians(lat2)
    dp, dl = p2 - p1, np.radians(lon2) - np.radians(lon1)
    a = np.sin(dp / 2) ** 2 + np.cos(p1) * np.cos(p2) * np.sin(dl / 2) ** 2
    return 2 * EARTH_RADIUS_M * np.arcsin(np.sqrt(a))


def describe(values, prefix: str) -> dict:
    x = pd.Series(values, dtype=float).dropna().to_numpy()
    keys = ["median", "mean", "p95", "min", "max", "std"]
    if len(x) == 0:
        return {f"{prefix}_{k}": np.nan for k in keys}
    return {
        f"{prefix}_median": np.median(x), f"{prefix}_mean": x.mean(),
        f"{prefix}_p95": np.percentile(x, 95), f"{prefix}_min": x.min(),
        f"{prefix}_max": x.max(), f"{prefix}_std": x.std(ddof=1) if len(x) > 1 else 0.0,
    }


def longest_run(flags) -> int:
    best = cur = 0
    for f in flags:
        cur = cur + 1 if f else 0
        best = max(best, cur)
    return best


def assign_copies(beacons, msg_index, copies):
    """
    Which copy (1..copies) each received beacon frame was. The bridge's MAVLink
    component sends nothing but beacons, so its sequence number advances once per
    copy: seq = offset + copies*msg_index + (copy - 1)  (mod 256).
    """
    beacons = beacons.copy()
    if copies <= 1 or beacons.empty:
        beacons["copy"] = 1
        return beacons
    k = (beacons.seq - copies * beacons.tracking_id.map(msg_index)) % SEQ_MOD
    candidates = k.dropna().unique()
    offset = max(candidates, key=lambda o: int(((k - o) % SEQ_MOD < copies).sum()))
    copy = (k - offset) % SEQ_MOD + 1
    beacons["copy"] = copy.where(copy <= copies)
    return beacons


def evaluate_test(name: str, logs: dict, cfg: dict):
    """-> (summary dict, per-message DataFrame, extras for plotting)"""
    source = norm_source(logs["source"][1])
    t0, t1 = source.t_pub.min(), source.t_pub.max()
    lo, hi = t0 - WINDOW_PRE_S, t1 + WINDOW_POST_S

    def in_window(df, label):
        keep = df[(df.t >= lo) & (df.t <= hi)]
        if len(keep) < len(df):
            print(f"  [{name}] ignoring {len(df) - len(keep)} {label} rows outside this run's time window")
        return keep

    relay = in_window(norm_relay(logs["relay"][1]), "relay") if "relay" in logs else None
    ground = in_window(norm_ground(logs["ground"][1]), "ground") if "ground" in logs else None
    frames, beacons = norm_router(logs["router"][1]) if "router" in logs else (None, pd.DataFrame())
    if not beacons.empty:
        beacons = in_window(beacons, "router beacon")
        beacons = beacons[beacons.tracking_id.isin(source.tracking_id)]
    if ground is None:
        ground = pd.DataFrame(columns=["t", "tracking_id", "lat", "lon", "color", "blink"])

    # ---- configuration actually used (data wins over notes; notes fill gaps)
    publish_hz = 1.0 / np.median(np.diff(source.t_pub)) if len(source) > 1 else np.nan
    copies = 1
    if relay is not None and relay.copies.notna().any():
        copies = int(relay.copies.max())
    elif relay is not None and relay.has_copy_rows.any():
        copies = int(relay["copy"].max())
    elif not beacons.empty:
        copies = int(beacons.groupby("tracking_id").size().max())
    if cfg.get("notes_copies") and cfg["notes_copies"] != copies:
        print(f"  [{name}] NOTES.md says {cfg['notes_copies']} copies, logs show {copies}")
    if cfg.get("notes_publish_hz") and abs(publish_hz / cfg["notes_publish_hz"] - 1) > 0.1:
        print(f"  [{name}] NOTES.md says {cfg['notes_publish_hz']} Hz, logs show {publish_hz:.2f} Hz")

    if relay is not None and relay.msg_id.notna().any():
        msg_index = relay.groupby("tracking_id").msg_id.first()
    else:
        msg_index = pd.Series(np.arange(len(source)), index=source.tracking_id)
    beacons = assign_copies(beacons, msg_index, copies)

    copy_gap = DEFAULT_COPY_GAP_S
    if copies > 1:
        if relay is not None and relay.has_copy_rows.any():
            piv = relay.pivot_table(index="tracking_id", columns="copy", values="t", aggfunc="min")
            if {1, 2} <= set(piv.columns):
                copy_gap = float(np.nanmedian(piv[2] - piv[1]))
        elif not beacons.empty:
            piv = beacons.pivot_table(index="tracking_id", columns="copy", values="t", aggfunc="min")
            if {1, 2} <= set(piv.columns) and (piv[2] - piv[1]).notna().any():
                copy_gap = float(np.nanmedian(piv[2] - piv[1]))

    # ---- one row per published detection
    msgs = source.copy()
    tid = msgs.tracking_id
    if relay is not None:
        msgs["t_relay"] = tid.map(relay.groupby("tracking_id").t.min())
    else:
        msgs["t_relay"] = np.nan
    if not beacons.empty:
        b = beacons.sort_values("t")
        first = b.groupby("tracking_id").first()
        msgs["t_router"] = tid.map(first.t)
        msgs["router_first_copy"] = tid.map(first["copy"])
        msgs["router_frames"] = tid.map(b.groupby("tracking_id").size()).fillna(0).astype(int)
        msgs["copy1_received"] = tid.isin(b.loc[b["copy"] == 1, "tracking_id"])
    else:
        msgs["t_router"] = np.nan
        msgs["router_first_copy"] = np.nan
        msgs["router_frames"] = 0
        msgs["copy1_received"] = False
    g_first = ground.sort_values("t").groupby("tracking_id").first()
    msgs["t_ground"] = tid.map(g_first.t)
    msgs["ground_rows"] = tid.map(ground.groupby("tracking_id").size()).fillna(0).astype(int)

    # send time of whichever copy reached the router first
    if relay is not None and relay.has_copy_rows.any():
        send = relay.set_index(["tracking_id", "copy"]).t
        send = send[~send.index.duplicated()]
        keys = list(zip(msgs.tracking_id, msgs.router_first_copy.fillna(1)))
        msgs["t_sent"] = [send.get(k, np.nan) for k in keys]
    else:
        base = msgs.t_relay if relay is not None else msgs.t_pub
        msgs["t_sent"] = base + (msgs.router_first_copy.fillna(1) - 1) * copy_gap

    msgs["lat_uav_s"] = msgs.t_relay - msgs.t_pub
    msgs["lat_radio_s"] = msgs.t_router - msgs.t_sent
    msgs["lat_ground_s"] = msgs.t_ground - msgs.t_router
    msgs["lat_e2e_s"] = msgs.t_ground - msgs.t_pub
    msgs["delivered"] = msgs.t_ground.notna()

    status = np.where(msgs.delivered, "delivered", "lost_radio")
    status = np.where(msgs.delivered & (copies > 1) & ~msgs.copy1_received & msgs.t_router.notna(),
                      "delivered_copy2", status)
    status = np.where(~msgs.delivered & msgs.t_router.notna(), "lost_ground", status)
    if relay is not None:
        status = np.where(~msgs.delivered & msgs.t_relay.isna(), "not_relayed", status)
    msgs["status"] = status
    msgs.insert(0, "test", name)

    n_pub = len(msgs)
    n_relayed = int(msgs.t_relay.notna().sum()) if relay is not None else np.nan
    n_router = int(msgs.t_router.notna().sum())
    n_ground = int(msgs.delivered.sum())
    radio_base = n_relayed if relay is not None else n_pub

    s = {
        "test": name, "notes": cfg.get("notes", ""),
        "publish_hz": publish_hz, "nominal_hz": cfg.get("notes_publish_hz") or round(publish_hz, 2),
        "copies": copies,
        "copy_gap_s": copy_gap if copies > 1 else np.nan,
        "duration_s": t1 - t0,
        "logs": ", ".join(f"{r}={logs[r][0]}" for r in ("source", "relay", "router", "ground") if r in logs),
    }

    # ---- data accuracy (every ground row, duplicates included)
    g = ground.merge(source, on="tracking_id", how="left", suffixes=("", "_src"))
    phantom = g.lat_src.isna()
    gk = g[~phantom]
    err = haversine_m(gk.lat, gk.lon, gk.lat_src, gk.lon_src)
    color_ok = gk.color == gk.color_src
    blink_ok = gk.blink == gk.blink_src
    pos_ok = err < POSITION_EXACT_M
    s.update({
        "ground_rows": len(ground), "phantom_rows": int(phantom.sum()),
        "pos_err_mean_m": err.mean(), "pos_err_max_m": err.max(),
        "position_match_pct": 100 * pos_ok.mean() if len(gk) else np.nan,
        "color_match_pct": 100 * color_ok.mean() if len(gk) else np.nan,
        "blink_match_pct": 100 * blink_ok.mean() if len(gk) else np.nan,
        "all_fields_match_pct": 100 * (pos_ok & color_ok & blink_ok).mean() if len(gk) else np.nan,
    })

    # ---- delivery
    s.update({
        "published": n_pub, "relayed": n_relayed, "router_rx": n_router, "delivered": n_ground,
        "e2e_delivery_pct": 100 * n_ground / n_pub,
        "relay_pct": 100 * n_relayed / n_pub if relay is not None else np.nan,
        "radio_delivery_pct": 100 * n_router / radio_base if radio_base else np.nan,
        "router_to_ground_pct": 100 * (msgs.delivered & msgs.t_router.notna()).sum() / n_router if n_router else np.nan,
        "ground_duplicate_rows": int(len(ground) - ground.tracking_id.nunique()),
    })
    if relay is not None and relay.has_copy_rows.any():
        frames_sent = len(relay[relay.tracking_id.isin(source.tracking_id)])
    else:
        frames_sent = radio_base * copies
    frames_rx = len(beacons)
    frame_pdr = frames_rx / frames_sent if frames_sent else np.nan
    s.update({
        "radio_frames_sent": frames_sent, "radio_frames_rx": frames_rx,
        "radio_frame_delivery_pct": 100 * frame_pdr,
        # what the per-message radio delivery would be if frame losses were independent
        "radio_delivery_if_independent_pct": 100 * (1 - (1 - frame_pdr) ** copies),
    })
    if copies > 1:
        relayed_mask = msgs.t_relay.notna() if relay is not None else pd.Series(True, index=msgs.index)
        s["copy1_delivery_pct"] = 100 * msgs.copy1_received[relayed_mask].mean()
        s["saved_by_repeat"] = int((msgs.status == "delivered_copy2").sum())
        s["both_copies_rx"] = int((msgs.router_frames >= 2).sum())

    # ---- latency
    s.update(describe(msgs.lat_e2e_s, "e2e_latency_s"))
    s.update(describe(msgs.lat_uav_s, "uav_hop_s"))
    s.update(describe(msgs.lat_radio_s, "radio_hop_s"))
    s.update(describe(msgs.lat_ground_s, "ground_hop_s"))
    s["negative_hop_latencies"] = int(sum((msgs[c] < 0).sum() for c in
                                          ("lat_uav_s", "lat_radio_s", "lat_ground_s", "lat_e2e_s")))

    # ---- update frequency (ground-side, one update per unique detection)
    arrivals = np.sort(msgs.t_ground.dropna().to_numpy())
    intervals = np.diff(arrivals)
    achieved_hz = (len(arrivals) - 1) / (arrivals[-1] - arrivals[0]) if len(arrivals) > 1 else np.nan
    s.update({
        "achieved_update_hz": achieved_hz,
        "update_rate_ratio_pct": 100 * achieved_hz / publish_hz,
        "interval_median_s": np.median(intervals) if len(intervals) else np.nan,
        "interval_p95_s": np.percentile(intervals, 95) if len(intervals) else np.nan,
        "max_update_gap_s": intervals.max() if len(intervals) else np.nan,
        "interval_cv": intervals.std() / intervals.mean() if len(intervals) > 1 else np.nan,
    })

    # ---- stability
    lat = msgs[["t_pub", "lat_e2e_s"]].dropna()
    drift = np.polyfit((lat.t_pub - t0) / 60, lat.lat_e2e_s, 1)[0] if len(lat) > 2 else np.nan
    rolling = msgs.delivered.rolling(ROLLING_WINDOW).mean().dropna()
    half = n_pub // 2
    s.update({
        "latency_jitter_s": s["e2e_latency_s_std"],
        "latency_tail_s": s["e2e_latency_s_p95"] - s["e2e_latency_s_median"],
        "latency_drift_s_per_min": drift,
        "max_gap_publish_periods": s["max_update_gap_s"] * publish_hz,
        "longest_loss_streak": longest_run(~msgs.delivered),
        "rolling_delivery_min_pct": 100 * rolling.min() if len(rolling) else np.nan,
        "delivery_first_half_pct": 100 * msgs.delivered[:half].mean(),
        "delivery_second_half_pct": 100 * msgs.delivered[half:].mean(),
    })

    link = {}
    if frames is not None:
        f = frames[(frames.t >= t0) & (frames.t <= t1)]
        dur = t1 - t0
        corrupt = f.msg_type.str.startswith("UNKNOWN_")
        # Log order, not a re-sort: many frames share a timestamp and an unstable
        # sort would swap them, turning seq 227->228 into a 255-frame "gap".
        fc = f[(f.src_system == 1) & (f.src_component == 1)]
        step = fc.seq.diff().dropna() % SEQ_MOD
        lost = (step[(step > 0) & (step < SEQ_MOD // 2)] - 1).sum()  # large steps = reorder/duplicate, not loss
        s.update({
            "link_frames_per_s": len(f) / dur,
            "link_bytes_per_s": f.bytes.sum() / dur,
            "corrupt_frame_pct": 100 * corrupt.mean(),
            "fc_telemetry_loss_pct": 100 * lost / (lost + len(fc)) if len(fc) else np.nan,
        })
        for stream in TELEMETRY_STREAMS:
            s[f"{stream.lower()}_hz"] = (f.msg_type == stream).sum() / dur
        groups = np.where(corrupt, "corrupt (unparsed)",
                          np.where(f.msg_type.isin(TELEMETRY_STREAMS + ["MEMORY_VECT"]), f.msg_type, "other telemetry"))
        link = (pd.Series(groups).value_counts() / dur).to_dict()
    else:
        raw = logs["router"][1] if "router" in logs else None
        if raw is not None:
            t = num(raw["epoch_time"])
            s["link_bytes_per_s"] = num(raw["bytes"])[(t >= t0) & (t <= t1)].sum() / (t1 - t0)

    extras = {"intervals": intervals, "link": link, "t0": t0}
    return s, msgs, extras


# --------------------------------------------------------------------------- charts

def setup_style():
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


def style(ax, title=None, ylabel=None, xlabel=None, grid="y"):
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


def test_label(row) -> str:
    label = f"T{test_number(row['test'])}\n{row['nominal_hz']:g} Hz"
    return label + (f" ×{int(row['copies'])}" if row["copies"] > 1 else "")


def bar_labels(ax, x, values, fmt, color=INK2, pad=1.0):
    for xi, v in zip(x, values):
        if np.isfinite(v):
            ax.text(xi, v + pad, fmt.format(v), ha="center", va="bottom", fontsize=8, color=color)


def plot_overview(summary, msgs, notes_lines, out):
    labels = [test_label(r) for _, r in summary.iterrows()]
    x = np.arange(len(summary))
    fig, axes = plt.subplots(2, 3, figsize=(16, 9.5))
    w = 0.38

    ax = axes[0, 0]
    ax.bar(x - w / 2, summary.e2e_delivery_pct, w, color=SERIES[0], label="Detections delivered (end-to-end)")
    ax.bar(x + w / 2, summary.radio_frame_delivery_pct, w, color=SERIES[1], label="Radio frames received")
    bar_labels(ax, x - w / 2, summary.e2e_delivery_pct, "{:.0f}%")
    ax.set_ylim(0, 115)
    style(ax, "Delivery rate", "% of published / sent")
    ax.legend(loc="upper left", ncol=1)

    ax = axes[0, 1]
    data = [msgs.loc[msgs.test == t, "lat_e2e_s"].dropna().to_numpy() for t in summary.test]
    bp = ax.boxplot(data, positions=x, widths=0.5, patch_artist=True, showfliers=True,
                    medianprops={"color": INK, "linewidth": 1.5},
                    whiskerprops={"color": SERIES[0]}, capprops={"color": SERIES[0]},
                    flierprops={"marker": "o", "markersize": 3, "markerfacecolor": SERIES[0],
                                "markeredgecolor": "none", "alpha": 0.6})
    for patch in bp["boxes"]:
        patch.set(facecolor=SERIES[0] + "40", edgecolor=SERIES[0])
    for xi, d in zip(x, data):
        if len(d):
            ax.text(xi, np.max(d) + 0.08, f"{np.median(d):.2f}s", ha="center", va="bottom", fontsize=8, color=INK2)
    style(ax, "End-to-end latency (publish → ground output)", "seconds")

    ax = axes[0, 2]
    ax.bar(x, summary.achieved_update_hz, 0.55, color=SERIES[0], label="Achieved at ground")
    ax.plot(x, summary.publish_hz, linestyle="none", marker="_", markersize=30, markeredgewidth=2,
            color=INK, label="Published rate")
    for xi, v, r in zip(x, summary.achieved_update_hz, summary.update_rate_ratio_pct):
        ax.text(xi, v / 2, f"{r:.0f}%", ha="center", va="center", fontsize=8, color="white", fontweight="semibold")
    ax.set_ylim(0, summary.publish_hz.max() * 1.25)
    style(ax, "Update frequency", "updates per second (Hz)")
    ax.legend(loc="upper right")

    ax = axes[1, 0]
    ax.bar(x, summary.max_update_gap_s, 0.55, color=SERIES[0], label="Longest gap between updates")
    ax.plot(x, 1 / summary.publish_hz, linestyle="none", marker="_", markersize=30, markeredgewidth=2,
            color=INK, label="Publish period")
    bar_labels(ax, x, summary.max_update_gap_s, "{:.1f}s", pad=0.3)
    ax.set_ylim(0, summary.max_update_gap_s.max() * 1.2)
    style(ax, "Worst outage", "seconds")
    ax.legend(loc="upper right")

    ax = axes[1, 1]
    ax.bar(x, summary.all_fields_match_pct, 0.55, color=SERIES[0])
    for xi, v, e in zip(x, summary.all_fields_match_pct, summary.pos_err_max_m):
        if np.isfinite(v):
            ax.text(xi, v + 1, f"{v:.0f}%\nmax err {e * 100:.1f} cm", ha="center", va="bottom", fontsize=8, color=INK2)
    ax.set_ylim(0, 125)
    style(ax, "Data accuracy (all fields correct)", "% of received detections")

    ax = axes[1, 2]
    corrupt = summary.get("corrupt_frame_pct", pd.Series(np.nan, index=summary.index))
    fcloss = summary.get("fc_telemetry_loss_pct", pd.Series(np.nan, index=summary.index))
    ax.bar(x - w / 2, corrupt, w, color=SERIES[0], label="Corrupt / unparsed frames")
    ax.bar(x + w / 2, fcloss, w, color=SERIES[1], label="Autopilot telemetry frames lost")
    top = np.nanmax(np.r_[corrupt.to_numpy(float), fcloss.to_numpy(float), 1.0])
    for xi, c in zip(x, corrupt):
        if not np.isfinite(c):
            ax.text(xi, top * 0.04, "raw log\nn/a", ha="center", va="bottom", fontsize=8, color=MUTED)
    ax.set_ylim(0, top * 1.35)
    style(ax, "Link health (whole MAVLink stream)", "% of frames")
    ax.legend(loc="upper left")

    for ax in axes.flat:
        ax.set_xticks(x, labels)
    fig.suptitle("ELRS beacon relay — test comparison", x=0.01, ha="left", fontsize=15,
                 fontweight="semibold", color=INK)
    fig.text(0.01, 0.005, "\n".join(notes_lines), fontsize=8, color=INK2, va="bottom", ha="left")
    fig.tight_layout(rect=(0, 0.02 + 0.016 * len(notes_lines), 1, 0.96))
    fig.savefig(out / "overview.png")
    plt.close(fig)


STATUS_STYLE = [  # (status, colour, legend label)
    ("delivered", SERIES[0], "Delivered"),
    ("delivered_copy2", SERIES[2], "Delivered only by the repeat copy"),
    ("lost_ground", STATUS["warning"], "Lost after radio (router → WSL)"),
    ("lost_radio", STATUS["critical"], "Lost over the radio"),
    ("not_relayed", AXIS, "Never sent by the UAV bridge"),
]


def plot_delivery_timeline(summary, msgs, out):
    codes = {s: i for i, (s, _, _) in enumerate(STATUS_STYLE)}
    tests = list(summary.test)
    width = int(msgs.groupby("test").size().max())
    grid = np.full((len(tests), width), np.nan)
    for i, t in enumerate(tests):
        st = msgs.loc[msgs.test == t, "status"].map(codes).to_numpy()
        grid[i, :len(st)] = st
    from matplotlib.colors import ListedColormap
    cmap = ListedColormap([c for _, c, _ in STATUS_STYLE])
    fig, ax = plt.subplots(figsize=(16, 0.55 * len(tests) + 1.8))
    ax.pcolormesh(np.ma.masked_invalid(grid), cmap=cmap, vmin=-0.5, vmax=len(STATUS_STYLE) - 0.5,
                  edgecolors=SURFACE, linewidth=1.5)
    ax.set_yticks(np.arange(len(tests)) + 0.5, [test_label(r).replace("\n", "  ") for _, r in summary.iterrows()])
    ax.invert_yaxis()
    ax.set_xlabel("Detection # in publish order")
    for i, (_, r) in enumerate(summary.iterrows()):
        ax.text(width + 0.8, i + 0.5, f"{int(r.delivered)}/{int(r.published)}", va="center", fontsize=9, color=INK2)
    ax.set_xlim(0, width + 6)
    style(ax, "Fate of every published detection", grid=None)
    for side in ("left", "bottom"):
        ax.spines[side].set_visible(False)
    present = set(msgs.status)
    ax.legend(handles=[Patch(color=c, label=l) for s, c, l in STATUS_STYLE if s in present],
              loc="upper left", bbox_to_anchor=(0, -0.12 - 0.25 / len(tests)), ncol=5)
    fig.tight_layout()
    fig.savefig(out / "delivery_timeline.png")
    plt.close(fig)


def small_multiples(n):
    cols = 2
    rows = int(np.ceil((n + 1) / cols))  # one spare cell holds the legend
    fig, axes = plt.subplots(rows, cols, figsize=(15, 2.7 * rows), squeeze=False)
    return fig, axes.flat


def plot_latency_timeseries(summary, msgs, out):
    fig, axes = small_multiples(len(summary))
    axes = list(axes)
    ymax = np.nanmax(msgs.lat_e2e_s.to_numpy(float)) * 1.1
    for ax, (_, r) in zip(axes, summary.iterrows()):
        m = msgs[msgs.test == r.test]
        el = m.t_pub - m.t_pub.min()
        c1 = m.delivered & (m.router_first_copy.fillna(1) == 1)
        c2 = m.delivered & ~c1
        ax.scatter(el[c1], m.lat_e2e_s[c1], s=16, color=SERIES[0], edgecolors=SURFACE, linewidths=0.6, zorder=3)
        ax.scatter(el[c2], m.lat_e2e_s[c2], s=16, color=SERIES[2], edgecolors=SURFACE, linewidths=0.6, zorder=3)
        lost = el[~m.delivered]
        ax.plot(lost, np.full(len(lost), ymax * 0.03), linestyle="none", marker="|", markersize=10,
                markeredgewidth=1.6, color=STATUS["critical"], zorder=3)
        med = m.lat_e2e_s.median()
        ax.axhline(med, color=MUTED, linewidth=1)
        ax.text(el.max(), med, f" median {med:.2f}s", va="bottom", ha="right", fontsize=8, color=INK2)
        ax.set_ylim(0, ymax)
        style(ax, f"{test_label(r).replace(chr(10), '  ')}  —  {int(r.delivered)}/{int(r.published)} delivered, "
                  f"jitter σ {r.latency_jitter_s:.2f}s", "latency (s)")
        ax.title.set_fontsize(9.5)
    for ax in axes[len(summary):]:
        ax.axis("off")
    axes[len(summary)].legend(handles=[
        Line2D([], [], marker="o", linestyle="none", color=SERIES[0], label="Delivered (first copy)"),
        Line2D([], [], marker="o", linestyle="none", color=SERIES[2], label="Delivered by repeat copy"),
        Line2D([], [], marker="|", linestyle="none", markersize=10, markeredgewidth=1.6,
               color=STATUS["critical"], label="Lost (publish time)"),
        Line2D([], [], color=MUTED, label="Median latency"),
    ], loc="center", fontsize=10)
    for ax in axes[max(0, len(summary) - 2):len(summary)]:
        ax.set_xlabel("seconds since first publish")
    fig.suptitle("End-to-end latency over each run", x=0.01, ha="left", fontsize=14, fontweight="semibold")
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out / "latency_over_time.png")
    plt.close(fig)


def plot_latency_hops(summary, msgs, out):
    hops = [("lat_uav_s", "UAV: publish → bridge send"),
            ("lat_radio_s", "ELRS radio: bridge send → Windows router"),
            ("lat_ground_s", "Ground: Windows router → WSL output")]
    fig, axes = plt.subplots(1, 3, figsize=(16, 0.5 * len(summary) + 2.2), sharey=True)
    y = np.arange(len(summary))
    for ax, (col, title) in zip(axes, hops):
        for yi, t in zip(y, summary.test):
            d = msgs.loc[msgs.test == t, col].dropna().to_numpy()
            if not len(d):
                ax.text(0, yi, "no log", va="center", fontsize=8, color=MUTED)
                continue
            p5, p25, p50, p75, p95 = np.percentile(d, [5, 25, 50, 75, 95])
            ax.plot([p5, p95], [yi, yi], color=SERIES[0], linewidth=1, alpha=0.6)
            ax.plot([p25, p75], [yi, yi], color=SERIES[0], linewidth=5, solid_capstyle="round", alpha=0.45)
            ax.plot(p50, yi, "o", color=SERIES[0], markersize=8, markeredgecolor=SURFACE, markeredgewidth=1.5)
            ax.text(p50, yi - 0.2, f"{p50 * 1000:.0f} ms", va="bottom", ha="center", fontsize=8, color=INK2)
        ax.axvline(0, color=AXIS, linewidth=1)
        style(ax, title, xlabel="seconds", grid="x")
        ax.title.set_fontsize(10)
    axes[0].set_yticks(y, [test_label(r).replace("\n", "  ") for _, r in summary.iterrows()])
    axes[0].invert_yaxis()
    fig.suptitle("Where the latency goes — median (dot), interquartile range (thick), 5–95% (thin)",
                 x=0.01, ha="left", fontsize=13, fontweight="semibold")
    fig.text(0.01, 0.01, "Hops span different machines' clocks (UAV / Windows / WSL): negative values mean clock "
                         "offset, not time travel. Without a bridge log (test5) the radio hop is measured from publish.",
             fontsize=8, color=INK2)
    fig.tight_layout(rect=(0, 0.04, 1, 0.93))
    fig.savefig(out / "latency_by_hop.png")
    plt.close(fig)


def plot_update_intervals(summary, extras, out):
    fig, axes = small_multiples(len(summary))
    axes = list(axes)
    top = max((e["intervals"].max() for e in extras.values() if len(e["intervals"])), default=1)
    bins = np.arange(0, top + 0.25, 0.25)
    for ax, (_, r) in zip(axes, summary.iterrows()):
        iv = extras[r.test]["intervals"]
        ax.hist(iv, bins=bins, color=SERIES[0], edgecolor=SURFACE, linewidth=1)
        period = 1 / r.publish_hz
        ax.axvline(period, color=INK, linewidth=1.5)
        ax.text(period, ax.get_ylim()[1] * 0.92, f"  publish period {period:.2f}s", fontsize=8, color=INK)
        style(ax, f"{test_label(r).replace(chr(10), '  ')}  —  {r.achieved_update_hz:.2f} Hz achieved, "
                  f"max gap {r.max_update_gap_s:.1f}s", "count")
        ax.title.set_fontsize(9.5)
        ax.set_xlim(0, top + 0.25)
    for ax in axes[len(summary):]:
        ax.axis("off")
    axes[len(summary)].text(0.5, 0.5, "Bars: time between consecutive new detections at the ground.\n"
                                      "Mass to the right of the publish-period line = missed updates.",
                            ha="center", va="center", fontsize=10, color=INK2, transform=axes[len(summary)].transAxes)
    for ax in axes[max(0, len(summary) - 2):len(summary)]:
        ax.set_xlabel("seconds between ground updates")
    fig.suptitle("Ground update intervals", x=0.01, ha="left", fontsize=14, fontweight="semibold")
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out / "update_intervals.png")
    plt.close(fig)


def plot_link_load(summary, extras, out):
    cats = [("ATTITUDE", SERIES[0]), ("ATTITUDE_QUATERNION", SERIES[1]), ("VFR_HUD", SERIES[2]),
            ("MEMORY_VECT", SERIES[3]), ("other telemetry", AXIS), ("corrupt (unparsed)", STATUS["critical"])]
    labels = {"MEMORY_VECT": "beacon (MEMORY_VECT)"}
    x = np.arange(len(summary))
    fig, ax = plt.subplots(figsize=(13, 5.5))
    bottom = np.zeros(len(summary))
    for cat, color in cats:
        vals = np.array([extras[t]["link"].get(cat, 0.0) for t in summary.test])
        ax.bar(x, vals, 0.6, bottom=bottom, color=color, edgecolor=SURFACE, linewidth=1.5,
               label=labels.get(cat, cat))
        bottom += vals
    for xi, total in enumerate(bottom):
        ax.text(xi, total + 0.4, f"{total:.1f}/s" if total else "raw log\nn/a", ha="center", va="bottom",
                fontsize=8, color=INK2 if total else MUTED)
    ax.set_xticks(x, [test_label(r) for _, r in summary.iterrows()])
    style(ax, "What shares the ELRS link — MAVLink frames per second received at the router", "frames / s")
    ax.legend(loc="upper left", bbox_to_anchor=(1.0, 1.0))
    fig.tight_layout()
    fig.savefig(out / "link_load.png")
    plt.close(fig)


# --------------------------------------------------------------------------- report

def fmt(v, spec=".2f", suffix=""):
    if v is None or (isinstance(v, float) and not np.isfinite(v)):
        return "—"
    return f"{v:{spec}}{suffix}"


def md_table(summary, columns):
    head = "| Test | " + " | ".join(c[1] for c in columns) + " |"
    sep = "|---|" + "|".join("---:" for _ in columns) + "|"
    rows = []
    for _, r in summary.iterrows():
        rows.append(f"| {r.test} | " + " | ".join(fmt(r.get(c[0], np.nan), *c[2:]) for c in columns) + " |")
    return "\n".join([head, sep, *rows])


def best(summary, col, lowest=False):
    s = summary[col].dropna()
    if s.empty:
        return "—"
    i = s.idxmin() if lowest else s.idxmax()
    return f"**{summary.test[i]}** ({fmt(s[i])})"


def write_report(summary, out):
    cfg_rows = ["| Test | NOTES.md | Publish Hz (measured) | Copies | Copy gap | ATTITUDE Hz | ATT_QUAT Hz | VFR_HUD Hz | Logs |",
                "|---|---|---:|---:|---:|---:|---:|---:|---|"]
    for _, r in summary.iterrows():
        cfg_rows.append(f"| {r.test} | {r.notes} | {fmt(r.publish_hz)} | {int(r.copies)} | {fmt(r.copy_gap_s, '.2f', 's')} | "
                        f"{fmt(r.get('attitude_hz'))} | {fmt(r.get('attitude_quaternion_hz'))} | "
                        f"{fmt(r.get('vfr_hud_hz'))} | {r.logs} |")

    lines = [
        "# ELRS beacon relay — test evaluation",
        "",
        "Generated by `evaluate_beacon_tests.py`. Pipeline: ROS publisher (UAV) → beacon_mavlink_bridge (UAV) → "
        "ELRS radio → elrs_windows_router (Windows) → beacon_mavlink_ground (WSL).",
        "",
        "## Test configurations",
        "Publish rate and copies are measured from the logs; telemetry stream rates are measured at the router.",
        "",
        *cfg_rows,
        "",
        "## Headline",
        "",
        f"- Best end-to-end delivery: {best(summary, 'e2e_delivery_pct')} %",
        f"- Lowest median latency: {best(summary, 'e2e_latency_s_median', lowest=True)} s",
        f"- Lowest latency jitter (σ): {best(summary, 'latency_jitter_s', lowest=True)} s",
        f"- Highest ground update rate: {best(summary, 'achieved_update_hz')} Hz",
        f"- Shortest worst-case outage: {best(summary, 'max_update_gap_s', lowest=True)} s",
        "",
        "![overview](overview.png)",
        "",
        "## 1. Data accuracy",
        "Every ground row (duplicates included) compared with the detection published under the same tracking id. "
        f"Position counts as correct within {POSITION_EXACT_M * 100:.0f} cm (the wire format is 1e-7°).",
        "",
        md_table(summary, [("ground_rows", "Ground rows", "d"), ("phantom_rows", "Unknown id", "d"),
                           ("position_match_pct", "Position ok", ".1f", "%"), ("pos_err_max_m", "Max pos err (m)", ".3f"),
                           ("color_match_pct", "Color ok", ".1f", "%"), ("blink_match_pct", "Blink ok", ".1f", "%"),
                           ("all_fields_match_pct", "All fields", ".1f", "%")]),
        "",
        "## 2. Message delivery rate",
        "Per hop: bridge = published detections the UAV bridge sent; radio = sent detections the Windows router "
        "received (any copy); ground = router detections that came out of the WSL decoder. "
        "*If independent* is the radio delivery expected from the per-frame rate if losses were random; observed "
        "values below it mean losses come in bursts.",
        "",
        md_table(summary, [("published", "Published", "d"), ("delivered", "Delivered", "d"),
                           ("e2e_delivery_pct", "End-to-end", ".1f", "%"), ("relay_pct", "Bridge", ".1f", "%"),
                           ("radio_delivery_pct", "Radio", ".1f", "%"), ("router_to_ground_pct", "Ground", ".1f", "%"),
                           ("radio_frame_delivery_pct", "Radio frames", ".1f", "%"),
                           ("radio_delivery_if_independent_pct", "If independent", ".1f", "%"),
                           ("copy1_delivery_pct", "Copy 1 only", ".1f", "%"), ("saved_by_repeat", "Saved by repeat", ".0f"),
                           ("ground_duplicate_rows", "Dup rows at ground", "d")]),
        "",
        "![delivery timeline](delivery_timeline.png)",
        "",
        "## 3. End-to-end latency",
        "Publish (UAV clock) → ground decoder output (WSL clock), first copy to arrive. Hop columns are medians.",
        "",
        md_table(summary, [("e2e_latency_s_median", "Median (s)"), ("e2e_latency_s_mean", "Mean (s)"),
                           ("e2e_latency_s_p95", "p95 (s)"), ("e2e_latency_s_max", "Max (s)"),
                           ("uav_hop_s_median", "UAV hop (s)", ".3f"), ("radio_hop_s_median", "Radio hop (s)", ".3f"),
                           ("ground_hop_s_median", "Ground hop (s)", ".3f"),
                           ("negative_hop_latencies", "Negative values", "d")]),
        "",
        "![latency by hop](latency_by_hop.png)",
        "",
        "## 4. Update frequency",
        "New detections per second at the ground station, against the publish rate.",
        "",
        md_table(summary, [("publish_hz", "Published (Hz)"), ("achieved_update_hz", "Achieved (Hz)"),
                           ("update_rate_ratio_pct", "Achieved / published", ".0f", "%"),
                           ("interval_median_s", "Median interval (s)"), ("interval_p95_s", "p95 interval (s)"),
                           ("max_update_gap_s", "Max gap (s)"), ("interval_cv", "Interval CV")]),
        "",
        "![update intervals](update_intervals.png)",
        "",
        "## 5. System stability",
        f"Jitter = σ of end-to-end latency; drift = latency trend over the run; rolling min = worst delivery rate over "
        f"any {ROLLING_WINDOW} consecutive detections. Link columns cover the whole MAVLink stream during the run.",
        "",
        md_table(summary, [("latency_jitter_s", "Jitter σ (s)"), ("latency_tail_s", "p95 − median (s)"),
                           ("latency_drift_s_per_min", "Drift (s/min)", ".3f"),
                           ("max_gap_publish_periods", "Max gap (periods)", ".1f"),
                           ("longest_loss_streak", "Longest loss streak", "d"),
                           ("rolling_delivery_min_pct", "Rolling min", ".0f", "%"),
                           ("delivery_first_half_pct", "1st half", ".0f", "%"),
                           ("delivery_second_half_pct", "2nd half", ".0f", "%"),
                           ("corrupt_frame_pct", "Corrupt frames", ".1f", "%"),
                           ("fc_telemetry_loss_pct", "Telemetry loss", ".1f", "%"),
                           ("link_frames_per_s", "Frames/s", ".1f"), ("link_bytes_per_s", "Bytes/s", ".0f")]),
        "",
        "![latency over time](latency_over_time.png)",
        "",
        "![link load](link_load.png)",
        "",
        "## Caveats",
        "- Latencies subtract timestamps from three clocks (UAV, Windows, WSL). Any offset between them is baked into "
        "the absolute values; a negative hop is the visible symptom. Sync clocks before trusting absolute latency.",
        "- Test1's router log stored raw serial chunks: beacon frames were recovered by CRC-checking the byte stream, "
        "but per-type link statistics are unavailable and its bytes/s counts raw serial bytes, not parsed frames.",
        "- Test5 has no bridge log, so its radio hop is measured from the ROS publish time.",
        "- Rows from earlier runs appended to the same log (seen in test4 and test7 bridge logs) are dropped "
        "using the publish time window.",
    ]
    (out / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


# --------------------------------------------------------------------------- main

def main():
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--root", type=Path, default=here, help="folder holding NOTES.md and testN/ (default: script folder)")
    ap.add_argument("--out", type=Path, default=None, help="output folder (default: <root>/results)")
    args = ap.parse_args()
    out = args.out or args.root / "results"
    out.mkdir(parents=True, exist_ok=True)

    notes = parse_notes(args.root / "NOTES.md")
    configs = configs_from_notes(notes)
    test_dirs = sorted((d for d in args.root.iterdir() if d.is_dir() and re.fullmatch(r"test\d+", d.name)),
                       key=lambda d: test_number(d.name))

    rows, all_msgs, extras = [], [], {}
    for d in test_dirs:
        print(f"Evaluating {d.name}")
        logs = load_test(d)
        if "source" not in logs:
            print(f"  [{d.name}] no source log (column `topic`) — skipped")
            continue
        s, m, e = evaluate_test(d.name, logs, configs.get(d.name, {}))
        rows.append(s)
        all_msgs.append(m)
        extras[d.name] = e

    if not rows:
        raise SystemExit("No evaluable tests found.")
    summary = pd.DataFrame(rows)
    msgs = pd.concat(all_msgs, ignore_index=True)
    summary.to_csv(out / "summary_metrics.csv", index=False, float_format="%.4f")
    msgs.to_csv(out / "per_message.csv", index=False, float_format="%.6f")

    setup_style()
    notes_lines = [f"T{test_number(t)}: {notes[t]}" for t in summary.test if t in notes]
    plot_overview(summary, msgs, notes_lines, out)
    plot_delivery_timeline(summary, msgs, out)
    plot_latency_timeseries(summary, msgs, out)
    plot_latency_hops(summary, msgs, out)
    plot_update_intervals(summary, extras, out)
    plot_link_load(summary, extras, out)
    write_report(summary, out)

    view = summary[["test", "publish_hz", "copies", "e2e_delivery_pct", "e2e_latency_s_median",
                    "latency_jitter_s", "achieved_update_hz", "max_update_gap_s", "all_fields_match_pct"]]
    print()
    print(view.to_string(index=False, float_format=lambda v: f"{v:.2f}"))
    print(f"\nWrote results to {out}")


if __name__ == "__main__":
    main()
