#!/usr/bin/env python3
"""Bench-test ELRS MAVLink serial -> QGC + WSL, QGC UDP -> serial router.

Only this program should open the radio COM port.  Radio data is parsed as
complete MAVLink frames, validated by pymavlink, then forwarded as the
original, unmodified frame bytes (including MAVLink 2 signatures).

WiFi redundancy (--wifi DRONE_IP:14550, off by default):
  The drone's voxl-mavlink-server serves a full-rate GCS stream over WiFi to
  any address that sends it heartbeats; this router sends them. Then:
  - Downlink: frames from BOTH links go to QGC + WSL. A frame already
    forwarded from the other link within DEDUP_S (byte-identical: same
    sender, sequence number and content -- voxl-mavlink-server and the drone
    ELRS bridge both forward PX4's frames unmodified) is dropped, so
    beacon_mavlink_ground.py, the OCS and QGC see one merged stream.
  - The heartbeat also goes to DRONE_IP:--wifi-onboard-port, so
    beacon_mavlink_bridge.py sends its beacons, SEABIRD_TASK, alerts and
    RunStart ACKs back over WiFi too (it bypasses voxl-mavlink-server).
  - Uplink (QGC / WSL -> drone): ONE link only -- WiFi while drone heartbeats
    arrive over it (within --wifi-alive-s), otherwise ELRS. Never both: a
    duplicated MISSION_COUNT restarts a fence/mission upload, and a duplicated
    COMMAND_LONG runs the command twice.
    Exception: datagrams carrying STATUSTEXT "SEABIRD_*" (RunStart, restart)
    go over BOTH links, and also straight to DRONE_IP:--wifi-onboard-port
    (beacon_mavlink_bridge.py --wifi-listen), since the drone only copies
    SEABIRD_* to its onboard listeners from the ELRS side. Repeats are
    harmless: the drone acts once per run_id / request id.
  - RC_CHANNELS_OVERRIDE is never sent over WiFi (the drone ELRS bridge
    blocks it on the ELRS side).
  - The router keeps running without the COM port (WiFi only); --com none
    disables the serial link.
"""

import argparse
import csv
import os
import socket
import sys
import time
from collections import Counter
from datetime import datetime

import serial
from serial import SerialException, SerialTimeoutException
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2

# Optional, project-specific beacon decoding. The router still runs without it.
# utils/ lives at the repo root, one level up -- not on sys.path when run
# from elrs_windows_routing/ (as start_seabird_ground.ps1 does).
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from utils.beacon_mavlink_protocol import (
        BEACON_MAVLINK_ADDRESS,
        BEACON_MAVLINK_VERSION,
        unpack_value_array,
    )
except ImportError:
    BEACON_MAVLINK_ADDRESS = None
    BEACON_MAVLINK_VERSION = None
    unpack_value_array = None

SERIAL_RETRY_S = 3.0
# A byte-identical frame from the other link within this window is the same
# PX4 message delivered twice. ELRS can lag WiFi by a second or more (the
# drone bridge queues and rate-limits), hence 2 s.
DEDUP_S = 2.0
RC_OVERRIDE_IDS = {69, 70}       # RC_CHANNELS_OVERRIDE -- never over WiFi
SEABIRD_PREFIX = "SEABIRD_"      # uplink STATUSTEXT sent over both links


def address(value):
    try:
        host, port = value.rsplit(":", 1)
        port = int(port)
        if not host or not 1 <= port <= 65535:
            raise ValueError()
        return host, port
    except ValueError as exc:
        raise argparse.ArgumentTypeError("Expected HOST:PORT") from exc


def new_parser():
    # MAVLink dialect bundled with pymavlink includes standard MEMORY_VECT.
    # No signing key is configured: CRC is validated, signatures are preserved
    # in the original bytes but not cryptographically verified here.
    parser = mavutil.mavlink.MAVLink(None)
    parser.robust_parsing = True
    return parser


def bad_reason_key(reason):
    """Group pymavlink's BAD_DATA reason strings into a few categories."""
    r = (reason or "").lower()
    if "prefix" in r:
        return "bad_prefix"        # byte outside any frame
    if "crc" in r:
        return "crc"               # frame corrupted on the link
    if "unknown" in r:
        return "unknown_msgid"     # message not in the parser's dialect
    if "signature" in r:
        return "signature"
    if "length" in r or "invalid" in r:
        return "length"            # truncated / wrong-length frame
    return "other"


BAD_KEYS = ("bad_prefix", "crc", "unknown_msgid", "length", "signature", "other")


def write_full(port, data):
    """Avoid silently discarding partial serial writes."""
    offset = 0
    while offset < len(data):
        count = port.write(data[offset:])
        if not count:
            raise SerialTimeoutException("Serial write returned zero bytes")
        offset += count


def recv_all(sock):
    """All datagrams waiting on a non-blocking UDP socket, as (data, sender)."""
    out = []
    while True:
        try:
            out.append(sock.recvfrom(65535))
        except (BlockingIOError, ConnectionResetError):
            break
        except OSError as exc:
            if getattr(exc, "winerror", None) == 10054:   # ICMP port unreachable
                break
            raise
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--com", default="COM6",
                    help="ELRS radio COM port; 'none' = no serial link (WiFi only)")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--qgc", type=address, default=("127.0.0.1", 14550))
    ap.add_argument("--wsl", type=address, default=("127.0.0.1", 14551))
    ap.add_argument("--listen", type=int, default=14600,
                    help="Local port for QGC-to-radio datagrams")
    ap.add_argument("--wifi", type=address, default=None, metavar="DRONE_IP:PORT",
                    help="Also link to the drone over WiFi: its voxl-mavlink-server GCS "
                         "port, e.g. 192.168.1.20:14550 (default: off, ELRS only)")
    ap.add_argument("--wifi-onboard-port", type=int, default=14572,
                    help="Drone port where beacon_mavlink_bridge.py --wifi-listen takes "
                         "SEABIRD_* STATUSTEXT (RunStart) over WiFi (default: 14572)")
    ap.add_argument("--wifi-alive-s", type=float, default=3.0,
                    help="WiFi carries the uplink while a drone heartbeat arrived over it "
                         "within this many seconds (default: 3)")
    ap.add_argument("--wsl-log", metavar="CSV", default=None,
                    help="CSV record of complete frames sent to WSL")
    ap.add_argument("--raw-rx-log", metavar="BIN", default=None,
                    help="Optional binary recording of raw bytes received from radio (may grow rapidly)")
    ap.add_argument("--top-messages", type=int, default=8,
                    help="Number of most frequent MAVLink message types to print per interval")
    ap.add_argument("--stats-period", type=float, default=5.0)
    ap.add_argument("--heartbeat-period", type=float, default=1.0)
    args = ap.parse_args()
    use_serial = args.com.lower() != "none"
    if not use_serial and args.wifi is None:
        ap.error("--com none needs --wifi (no link left)")

    log_file = None
    log_writer = None
    if args.wsl_log:
        empty = not os.path.exists(args.wsl_log) or os.path.getsize(args.wsl_log) == 0
        log_file = open(args.wsl_log, "a", newline="", encoding="utf-8")
        log_writer = csv.writer(log_file)
        if empty:
            log_writer.writerow([
                "iso_time", "epoch_time", "status", "bytes", "msg_type",
                "src_system", "src_component", "mav_seq", "tracking_id", "hex_payload"
            ])
            log_file.flush()

    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(("127.0.0.1", args.listen))
    udp.setblocking(False)

    wifi = None
    if args.wifi is not None:
        wifi = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        wifi.bind(("0.0.0.0", 0))
        wifi.setblocking(False)

    print(f"ELRS router | " + (f"{args.com} @ {args.baud}" if use_serial else "no serial link"),
          flush=True)
    print(f"Radio -> QGC: {args.qgc}, WSL: {args.wsl}", flush=True)
    print(f"QGC -> radio: 127.0.0.1:{args.listen}", flush=True)
    if wifi is not None:
        print(f"WiFi link: drone {args.wifi[0]}:{args.wifi[1]} (local port "
              f"{wifi.getsockname()[1]}); SEABIRD_* also to {args.wifi[0]}:"
              f"{args.wifi_onboard_port}; uplink over WiFi while drone heartbeats arrive "
              f"within {args.wifi_alive_s:g} s, else ELRS", flush=True)
    if BEACON_MAVLINK_ADDRESS is None:
        print("NOTE: utils.beacon_mavlink_protocol not importable; "
              "MEMORY_VECT counted, but beacon tracking IDs unavailable.", flush=True)
    print("Only this router should open COM port. Ctrl+C to stop.", flush=True)

    raw_log = open(args.raw_rx_log, "ab", buffering=0) if args.raw_rx_log else None
    port = None
    next_serial_try = 0.0
    parser = new_parser()
    wifi_parser = new_parser()
    totals = Counter()
    recent = Counter()
    last_stats = time.monotonic()
    gcs_mav = mavlink2.MAVLink(
        None,
        srcSystem=255,
        srcComponent=190
    )

    last_heartbeat = 0.0
    last_wifi_heartbeat = 0.0
    wifi_drone_hb = float("-inf")   # last drone (non-GCS) heartbeat received over WiFi
    uplink_link = None              # "wifi" / "elrs" / None -- logged when it changes
    forwarded = {}                  # frame bytes -> (link, time) for cross-link dedup
    # BAD_DATA diagnostics: consecutive BAD_DATA with the same reason form one
    # junk run (one row in --wsl-log, one sample per reason per interval);
    # MAVLink sequence gaps per (link, sender) count frames that never arrived.
    junk, junk_key, junk_reason = bytearray(), None, ""
    samples = {}
    last_seq = {}
    rx_types = Counter()
    tx_types = Counter()

    def count(key, n=1):
        recent[key] += n
        totals[key] += n

    def inspect_qgc_datagram(data):
        """Inspect QGC / WSL traffic without altering, dropping, or repacking it.

        A UDP datagram may contain multiple MAVLink frames. This is only a
        diagnostic parser; original bytes are always written unchanged.
        Returns (has SEABIRD_* STATUSTEXT, has RC_CHANNELS_OVERRIDE).
        """
        probe = new_parser()
        found = 0
        invalid = 0
        seabird = rc_override = False
        for b in data:
            try:
                message = probe.parse_char(bytes((b,)))
            except Exception:
                invalid += 1
                probe = new_parser()
                continue
            if message is None:
                continue
            if message.get_type() == "BAD_DATA":
                invalid += 1
                continue
            tx_types[message.get_type()] += 1
            found += 1
            if message.get_msgId() in RC_OVERRIDE_IDS:
                rc_override = True
            elif (message.get_type() == "STATUSTEXT"
                    and str(message.text).strip("\x00 ").startswith(SEABIRD_PREFIX)):
                seabird = True
        count("qgc_rx_frames", found)
        count("qgc_rx_bad_data", invalid)
        return seabird, rc_override

    def flush_junk():
        nonlocal junk, junk_key, junk_reason
        if junk_key is None:
            return
        count(f"runs_{junk_key}")
        samples.setdefault(junk_key, (junk_reason, bytes(junk[:48]), len(junk)))
        if log_writer is not None:
            stamp = time.time()
            log_writer.writerow([
                datetime.fromtimestamp(stamp).isoformat(timespec="milliseconds"),
                f"{stamp:.6f}", "BAD_DATA", len(junk), junk_key, "", "", "", "",
                bytes(junk).hex(),
            ])
        junk, junk_key, junk_reason = bytearray(), None, ""

    def forward(msg, link, now):
        """A complete frame from the drone (link "elrs" / "wifi") -> QGC + WSL,
        unless the other link already delivered the same frame."""
        nonlocal forwarded, wifi_drone_hb
        frame = bytes(msg.get_msgbuf())
        msg_type = msg.get_type()
        if (link == "wifi" and msg_type == "HEARTBEAT"
                and msg.type != mavutil.mavlink.MAV_TYPE_GCS):
            wifi_drone_hb = now
        prev = forwarded.get(frame)
        if prev is not None and prev[0] != link and now - prev[1] < DEDUP_S:
            count("dup_dropped")
            return
        forwarded[frame] = (link, now)
        if len(forwarded) > 8192:
            forwarded = {f: v for f, v in forwarded.items() if now - v[1] < DEDUP_S}

        rx_types[msg_type] += 1
        count("valid_frames")
        count(f"{link}_frames")
        src = (link, msg.get_srcSystem(), msg.get_srcComponent())
        seq = msg.get_seq()
        if src in last_seq:
            delta = (seq - last_seq[src]) % 256
            if delta == 0:
                count("duplicate_frames")
            elif delta <= 127:
                count("lost_frames_estimate", delta - 1)
                last_seq[src] = seq
            else:
                # Backward sequence, source restart, or gap >=128:
                # ambiguous without further protocol context.
                count("out_of_order_frames")
        else:
            last_seq[src] = seq
        tracking_id = ""
        if msg_type == "MEMORY_VECT":
            count("memory_vect")
            if (BEACON_MAVLINK_ADDRESS is not None
                    and msg.address == BEACON_MAVLINK_ADDRESS
                    and msg.ver == BEACON_MAVLINK_VERSION):
                count("beacons")
                try:
                    tracking_id = unpack_value_array(msg.value).tracking_id
                except Exception as exc:
                    print(f"Beacon decode warning: {exc}", flush=True)
                print(f"BEACON Windows RX ({link}) tracking_id={tracking_id} "
                      f"MAVLink_seq={msg.get_seq()}", flush=True)

        for label, destination in (("qgc", args.qgc), ("wsl", args.wsl)):
            status = "SENT"
            try:
                udp.sendto(frame, destination)
                count(f"{label}_sent")
            except OSError as exc:
                status = "SEND_FAILED"
                count(f"{label}_failed")
                print(f"UDP send to {destination} failed: {exc}", flush=True)
            if label == "wsl" and log_writer is not None:
                stamp = time.time()
                log_writer.writerow([
                    datetime.fromtimestamp(stamp).isoformat(timespec="milliseconds"),
                    f"{stamp:.6f}", status, len(frame), msg_type,
                    msg.get_srcSystem(), msg.get_srcComponent(),
                    msg.get_seq(), tracking_id, frame.hex(),
                ])

    def close_serial(reason):
        nonlocal port, next_serial_try
        print(f"Serial disconnected: {reason}; reconnecting", flush=True)
        try:
            port.close()
        except OSError:
            pass
        port = None
        next_serial_try = time.monotonic() + 2.0

    def serial_write(data):
        """Write to the radio; False if there's no open port (or it failed)."""
        if port is None:
            return False
        try:
            write_full(port, data)
            return True
        except (SerialException, SerialTimeoutException, OSError) as exc:
            close_serial(exc)
            return False

    def wifi_send(data, dest=None):
        try:
            wifi.sendto(data, dest or args.wifi)
            return True
        except OSError as exc:
            count("wifi_send_failed")
            if recent["wifi_send_failed"] == 1:
                print(f"WiFi send to {dest or args.wifi} failed: {exc}", flush=True)
            return False

    try:
        while True:
            now = time.monotonic()

            # -- serial (ELRS) link: (re)open without blocking the WiFi link --
            if use_serial and port is None and now >= next_serial_try:
                try:
                    port = serial.Serial(port=None, baudrate=args.baud, timeout=0,
                                         write_timeout=0.5)
                    port.dtr = False
                    port.rts = False
                    port.port = args.com
                    port.open()
                    last_heartbeat = 0.0
                    parser = new_parser()  # don't carry partial frames across reconnects
                    for k in [k for k in last_seq if k[0] == "elrs"]:
                        del last_seq[k]    # sender sequence counters may reset on reconnect
                    flush_junk()
                    print(f"Connected to {args.com}", flush=True)
                except (SerialException, OSError) as exc:
                    port = None
                    next_serial_try = now + SERIAL_RETRY_S
                    print(f"Unable to open {args.com}: {exc}; retrying in "
                          f"{SERIAL_RETRY_S:.0f}s", flush=True)

            if port is not None:
                # Keep standalone ELRS transmitter activated using MAVLink
                if now - last_heartbeat >= args.heartbeat_period:
                    packet = gcs_mav.heartbeat_encode(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0, 0, mavutil.mavlink.MAV_STATE_ACTIVE,
                    ).pack(gcs_mav)
                    if serial_write(packet):
                        count("gcs_heartbeat_tx")
                    last_heartbeat = now
            if port is not None:
                try:
                    pending = port.in_waiting
                    raw = port.read(min(pending, 4096)) if pending else b""
                except (SerialException, OSError) as exc:
                    close_serial(exc)
                    raw = b""
                if raw:
                    count("serial_bytes", len(raw))
                    if raw_log is not None:
                        raw_log.write(raw)

                    # Feed one byte at a time to avoid dropping frames when a
                    # single serial read contains multiple MAVLink packets.
                    for byte in raw:
                        try:
                            msg = parser.parse_char(bytes((byte,)))
                        except Exception:
                            count("parser_errors")
                            # Parser may be out of sync following corrupt bytes.
                            parser = new_parser()
                            continue
                        if msg is None:
                            continue
                        if msg.get_type() == "BAD_DATA":
                            count("bad_data")
                            key = bad_reason_key(getattr(msg, "reason", ""))
                            count(f"bad_{key}")
                            if key != junk_key:
                                flush_junk()
                                junk_key, junk_reason = key, getattr(msg, "reason", "")
                            junk.extend(bytes(getattr(msg, "data", b"") or b""))
                            continue
                        flush_junk()
                        forward(msg, "elrs", time.monotonic())

            # -- WiFi link: heartbeat so voxl-mavlink-server streams to us --
            if wifi is not None:
                if now - last_wifi_heartbeat >= args.heartbeat_period:
                    packet = gcs_mav.heartbeat_encode(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0, 0, mavutil.mavlink.MAV_STATE_ACTIVE,
                    ).pack(gcs_mav)
                    wifi_send(packet)
                    # Also to beacon_mavlink_bridge.py --wifi-listen: it only
                    # sends its beacons / task / alerts / ACKs back over WiFi to
                    # addresses that have sent to it.
                    wifi_send(packet, (args.wifi[0], args.wifi_onboard_port))
                    last_wifi_heartbeat = now
                for data, sender in recv_all(wifi):
                    if sender[0] != args.wifi[0]:
                        count("wifi_foreign")   # not the drone -- ignored
                        continue
                    count("wifi_bytes", len(data))
                    for b in data:
                        try:
                            msg = wifi_parser.parse_char(bytes((b,)))
                        except Exception:
                            count("wifi_bad_data")
                            wifi_parser = new_parser()
                            continue
                        if msg is None:
                            continue
                        if msg.get_type() == "BAD_DATA":
                            count("wifi_bad_data")
                            continue
                        forward(msg, "wifi", time.monotonic())

            # -- uplink: QGC / WSL -> drone, over ONE link (SEABIRD_* over both) --
            now = time.monotonic()
            wifi_up = wifi is not None and now - wifi_drone_hb < args.wifi_alive_s
            link = "wifi" if wifi_up else "elrs" if port is not None else None
            if link != uplink_link:
                if wifi is not None:
                    print(f"Uplink now over {link.upper() if link else 'NO LINK'}"
                          + ("" if wifi_up else " (no drone heartbeat over WiFi)"), flush=True)
                uplink_link = link
            for incoming, sender in recv_all(udp):
                if not incoming:
                    continue
                seabird, rc_override = inspect_qgc_datagram(incoming)
                # Deliberately preserve the complete original datagram:
                # this may include multiple frames or MAVLink signatures.
                sent = False
                if link == "elrs" or (seabird and port is not None):
                    if serial_write(incoming):
                        count("qgc_to_serial_bytes", len(incoming))
                        sent = True
                if wifi is not None and (link == "wifi" or seabird):
                    if rc_override:
                        count("rc_override_blocked_wifi")
                    elif wifi_send(incoming):
                        count("qgc_to_wifi_bytes", len(incoming))
                        sent = True
                if seabird and wifi is not None:
                    wifi_send(incoming, (args.wifi[0], args.wifi_onboard_port))
                if not sent:
                    count("uplink_dropped")

            if time.monotonic() - last_stats >= args.stats_period:
                flush_junk()  # include the final bad-data run in this interval
                fields = ("serial_bytes", "valid_frames", "lost_frames_estimate",
                          "duplicate_frames", "out_of_order_frames", "memory_vect",
                          "beacons", "bad_data", "parser_errors", "wsl_sent", "wsl_failed",
                          "qgc_sent", "qgc_failed", "qgc_to_serial_bytes",
                          "qgc_rx_frames", "qgc_rx_bad_data")
                if wifi is not None:
                    fields += ("elrs_frames", "wifi_frames", "wifi_bytes", "dup_dropped",
                               "qgc_to_wifi_bytes", "uplink_dropped", "wifi_bad_data",
                               "wifi_send_failed", "rc_override_blocked_wifi", "wifi_foreign")
                print("Last interval: " + " ".join(f"{k}={recent[k]}" for k in fields)
                      + (f" uplink={uplink_link or 'none'}" if wifi is not None else ""),
                      flush=True)
                if recent["bad_data"]:
                    # bad_<reason> = BAD_DATA count; runs = consecutive groups.
                    print("  bad_data by reason: " + " ".join(
                        f"{k}={recent[f'bad_{k}']}(runs {recent[f'runs_{k}']})"
                        for k in BAD_KEYS if recent[f"bad_{k}"]), flush=True)
                    for k, (reason, data, n) in samples.items():
                        print(f"    e.g. {k} ({n} bytes, '{reason}'): {data.hex(' ')}"
                              + (" ..." if n > len(data) else ""), flush=True)
                if rx_types:
                    print("  radio RX messages: " + ", ".join(
                        f"{name}={n}" for name, n in rx_types.most_common(args.top_messages)), flush=True)
                if tx_types:
                    print("  QGC TX messages: " + ", ".join(
                        f"{name}={n}" for name, n in tx_types.most_common(args.top_messages)), flush=True)
                rx_types.clear()
                tx_types.clear()
                samples.clear()
                if log_file is not None:
                    log_file.flush()
                recent.clear()
                last_stats = time.monotonic()

            time.sleep(0.002)
    except KeyboardInterrupt:
        print("Stopping.", flush=True)
    finally:
        flush_junk()
        print("Totals: " + ", ".join(f"{k}={v}" for k, v in sorted(totals.items())), flush=True)
        if port is not None:
            port.close()
        udp.close()
        if wifi is not None:
            wifi.close()
        if log_file is not None:
            log_file.close()
        if raw_log is not None:
            raw_log.close()


if __name__ == "__main__":
    main()
