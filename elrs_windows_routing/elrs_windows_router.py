#!/usr/bin/env python3
"""Bench-test ELRS MAVLink serial -> QGC + WSL, QGC UDP -> serial router.

Only this program should open the radio COM port.  Radio data is parsed as
complete MAVLink frames, validated by pymavlink, then forwarded as the
original, unmodified frame bytes (including MAVLink 2 signatures).

NOT flight-qualified. Test QGC commands and telemetry on the bench first.
"""

import argparse
import csv
import os
import select
import socket
import time
from collections import Counter
from datetime import datetime

import serial
from serial import SerialException, SerialTimeoutException
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2

# Optional, project-specific beacon decoding. The router still runs without it.
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


def write_full(port, data):
    """Avoid silently discarding partial serial writes."""
    offset = 0
    while offset < len(data):
        count = port.write(data[offset:])
        if not count:
            raise SerialTimeoutException("Serial write returned zero bytes")
        offset += count


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--com", default="COM6")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--qgc", type=address, default=("127.0.0.1", 14550))
    ap.add_argument("--wsl", type=address, default=("127.0.0.1", 14551))
    ap.add_argument("--listen", type=int, default=14600,
                    help="Local port for QGC-to-radio datagrams")
    ap.add_argument("--wsl-log", metavar="CSV", default=None,
                    help="CSV record of complete frames sent to WSL")
    ap.add_argument("--stats-period", type=float, default=5.0)
    ap.add_argument("--heartbeat-period", type=float, default=1.0)
    args = ap.parse_args()

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

    print(f"ELRS router | {args.com} @ {args.baud}", flush=True)
    print(f"Radio -> QGC: {args.qgc}, WSL: {args.wsl}", flush=True)
    print(f"QGC -> radio: 127.0.0.1:{args.listen}", flush=True)
    if BEACON_MAVLINK_ADDRESS is None:
        print("NOTE: utils.beacon_mavlink_protocol not importable; "
              "MEMORY_VECT counted, but beacon tracking IDs unavailable.", flush=True)
    print("Bench-test mode. Only this router should open COM port. Ctrl+C to stop.", flush=True)

    port = None
    parser = new_parser()
    totals = Counter()
    recent = Counter()
    last_stats = time.monotonic()
    gcs_mav = mavlink2.MAVLink(
        None,
        srcSystem=255,
        srcComponent=190
    )

    last_heartbeat = 0.0

    try:
        while True:
            if port is None:
                try:
                    #port = serial.Serial(args.com, args.baud, timeout=0,
                    #                     write_timeout=0.5)
                    port = serial.Serial(port=None, baudrate = args.baud, timeout=0, write_timeout=0.5)
                    port.dtr = False
                    port.rts = False
                    port.port = args.com
                    port.open()
                    last_heartbeat = 0.0
                    parser = new_parser()  # don't carry partial frames across reconnects
                    print(f"Connected to {args.com}", flush=True)
                except (SerialException, OSError) as exc:
                    print(f"Unable to open {args.com}: {exc}; retrying in 3s", flush=True)
                    time.sleep(3)
                    continue

            try:
                # Keep standalone ELRS transmitter activated using MAVLink
                now = time.monotonic()

                if now - last_heartbeat >= args.heartbeat_period:
                    msg = gcs_mav.heartbeat_encode(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0,
                        0,
                        mavutil.mavlink.MAV_STATE_ACTIVE
                    )

                    packet = msg.pack(gcs_mav)
                    """
                    print(
                        f"[TX] MAVLink header=0x{packet[0]:02X} "
                        f"bytes={len(packet)}",
                        flush=True
                    )
                    """
                    write_full(port, packet)

                    recent["gcs_heartbeat_tx"] += 1
                    totals["gcs_heartbeat_tx"] += 1

                    last_heartbeat = now
                pending = port.in_waiting
                if pending:
                    raw = port.read(min(pending, 4096))
                    recent["serial_bytes"] += len(raw)
                    totals["serial_bytes"] += len(raw)

                    # Feed one byte at a time to avoid dropping frames when a
                    # single serial read contains multiple MAVLink packets.
                    for byte in raw:
                        try:
                            msg = parser.parse_char(bytes((byte,)))
                        except Exception as exc:
                            recent["parser_errors"] += 1
                            totals["parser_errors"] += 1
                            # Parser may be out of sync following corrupt bytes.
                            parser = new_parser()
                            continue
                        if msg is None:
                            continue
                        msg_type = msg.get_type()
                        if msg_type == "BAD_DATA":
                            recent["bad_data"] += 1
                            totals["bad_data"] += 1
                            continue

                        frame = bytes(msg.get_msgbuf())
                        recent["valid_frames"] += 1
                        totals["valid_frames"] += 1
                        tracking_id = ""
                        if msg_type == "MEMORY_VECT":
                            recent["memory_vect"] += 1
                            totals["memory_vect"] += 1
                            if (BEACON_MAVLINK_ADDRESS is not None
                                    and msg.address == BEACON_MAVLINK_ADDRESS
                                    and msg.ver == BEACON_MAVLINK_VERSION):
                                recent["beacons"] += 1
                                totals["beacons"] += 1
                                try:
                                    tracking_id = unpack_value_array(msg.value).tracking_id
                                except Exception as exc:
                                    print(f"Beacon decode warning: {exc}", flush=True)
                                print(f"BEACON Windows RX tracking_id={tracking_id} "
                                      f"MAVLink_seq={msg.get_seq()}", flush=True)

                        for label, destination in (("qgc", args.qgc), ("wsl", args.wsl)):
                            status = "SENT"
                            try:
                                udp.sendto(frame, destination)
                                recent[f"{label}_sent"] += 1
                                totals[f"{label}_sent"] += 1
                            except OSError as exc:
                                status = "SEND_FAILED"
                                recent[f"{label}_failed"] += 1
                                totals[f"{label}_failed"] += 1
                                print(f"UDP send to {destination} failed: {exc}", flush=True)
                            if label == "wsl" and log_writer is not None:
                                stamp = time.time()
                                log_writer.writerow([
                                    datetime.fromtimestamp(stamp).isoformat(timespec="milliseconds"),
                                    f"{stamp:.6f}", status, len(frame), msg_type,
                                    msg.get_srcSystem(), msg.get_srcComponent(),
                                    msg.get_seq(), tracking_id, frame.hex(),
                                ])

                # Preserve the QGC -> ELRS command path without MAVLink
                # re-encoding (preserves signed messages).
                while True:
                    try:
                        incoming, sender = udp.recvfrom(65535)
                    except (BlockingIOError, ConnectionResetError):
                        break
                    except OSError as exc:
                        if getattr(exc, "winerror", None) == 10054:
                            break
                        raise
                    if incoming:
                        write_full(port, incoming)
                        recent["qgc_to_serial_bytes"] += len(incoming)
                        totals["qgc_to_serial_bytes"] += len(incoming)

                if time.monotonic() - last_stats >= args.stats_period:
                    fields = ("serial_bytes", "valid_frames", "memory_vect", "beacons",
                              "bad_data", "parser_errors", "wsl_sent", "wsl_failed",
                              "qgc_sent", "qgc_failed", "qgc_to_serial_bytes")
                    print("Last interval: " + " ".join(f"{k}={recent[k]}" for k in fields),
                          flush=True)
                    if log_file is not None:
                        log_file.flush()
                    recent.clear()
                    last_stats = time.monotonic()

                time.sleep(0.002)
            except (SerialException, SerialTimeoutException, OSError) as exc:
                print(f"Serial disconnected: {exc}; reconnecting", flush=True)
                try:
                    port.close()
                except OSError:
                    pass
                port = None
                time.sleep(2)
    except KeyboardInterrupt:
        print("Stopping.", flush=True)
    finally:
        print("Totals: " + ", ".join(f"{k}={v}" for k, v in sorted(totals.items())), flush=True)
        if port is not None:
            port.close()
        udp.close()
        if log_file is not None:
            log_file.close()


if __name__ == "__main__":
    main()
