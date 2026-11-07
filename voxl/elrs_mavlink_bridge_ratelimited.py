#!/usr/bin/env python3

"""Rate-limited bidirectional ELRS serial-to-UDP MAVLink bridge.

Copy of the drone's /usr/local/bin/elrs_mavlink_bridge.py with a rate
limiter on the VOXL/PX4 -> ELRS direction.

Why: voxl-mavlink-server sends this bridge the full GCS stream, and the
original wrote all of it to the 460800-baud UART (~46 KB/s). The ELRS
receiver can only send ~1 KB/s over the air, so its buffer overflowed and
dropped bytes mid-frame -- the ground saw truncated frames (CRC / bad-prefix
BAD_DATA) and lost most messages, including one-shot STATUSTEXT warnings.

Downlink (PX4 -> ELRS) now:
  - is parsed into whole MAVLink frames; only whole frames are ever sent or
    dropped, so the ground never receives a cut frame from this bridge;
  - is paced to --max-bps bytes/s with bursts of at most --burst bytes, so
    the ELRS receiver's buffer doesn't overflow;
  - sends, in order: due HEARTBEATs, then PRIORITY_IDS (warnings, command
    acks, beacons, mission/param replies) first-in first-out, then the
    THROTTLE_HZ telemetry at reduced rates (latest frame per message and
    sender -- stale ones are superseded, never queued up);
  - drops every other message type unless --default-hz > 0.
Uplink (ELRS -> PX4) is unchanged: RC_CHANNELS_OVERRIDE (69/70) is blocked.
--no-limit restores the original forward-everything downlink for comparison.
"""

import argparse
import collections
import errno
import select
import signal
import socket
import sys
import time

import serial
from pymavlink import mavutil
# Explicit MAVLink 2 dialect for the downlink parser: PX4 sends MAVLink 2,
# and common includes MEMORY_VECT (the beacon messages).
from pymavlink.dialects.v20 import common as mavlink2


running = True

UDP_SOCKET_BUFFER_SIZE = 4 * 1024 * 1024
MAX_SERIAL_TX_QUEUE = 1024 * 1024
SERIAL_READ_SIZE = 8192
SELECT_TIMEOUT = 0.005
REPORT_INTERVAL = 5.0

# Ground-control commands that must never reach PX4 through this bridge.
BLOCKED_GCS_TO_PX4_MESSAGE_IDS = {69, 70}  # RC_CHANNELS_OVERRIDE

HEARTBEAT_ID = 0
# Always sent, first-in first-out, ahead of telemetry: one-shot or
# request/reply messages the ground can't do without.
PRIORITY_IDS = {
    253,    # STATUSTEXT (warnings, older PX4 / non-event messages)
    410,    # EVENT (PX4 events: manual control lost, failsafes, arming checks)
    413,    # RESPONSE_EVENT_ERROR (reply to QGC's REQUEST_EVENT)
    77,     # COMMAND_ACK
    76,     # COMMAND_LONG
    249,    # MEMORY_VECT (beacon detections)
    22,     # PARAM_VALUE
    110,    # FILE_TRANSFER_PROTOCOL (QGC parameter / file transfer)
    148,    # AUTOPILOT_VERSION
    300,    # PROTOCOL_VERSION
    39, 40, 44, 46, 47, 51, 73,   # mission protocol replies
}
# Telemetry kept at reduced rates (Hz), per sender. Includes what the
# ground station's VehicleStatus uses (HEARTBEAT, SYS_STATUS, GPS_RAW_INT,
# VFR_HUD, ATTITUDE).
THROTTLE_HZ = {
    HEARTBEAT_ID: 1.0,
    1: 1.0,      # SYS_STATUS
    24: 1.0,     # GPS_RAW_INT
    33: 1.0,     # GLOBAL_POSITION_INT
    74: 1.0,     # VFR_HUD
    30: 2.0,     # ATTITUDE
    147: 0.5,    # BATTERY_STATUS
    245: 0.5,    # EXTENDED_SYS_STATE
    42: 0.5,     # MISSION_CURRENT
    109: 0.5,    # RADIO_STATUS
    242: 0.2,    # HOME_POSITION
    411: 1.0,    # CURRENT_EVENT_SEQUENCE (lets QGC spot and request missed EVENTs)
}
MAX_PRIORITY_BYTES = 64 * 1024   # enough to queue a full parameter download
MAX_FRAME_BYTES = 280            # largest MAVLink 2 frame (with signature)


def stop_handler(signum, frame):
    """Request a clean shutdown."""
    del signum, frame
    global running
    running = False


def frame_msg_id(frame):
    """Message ID from a raw MAVLink 1/2 frame header, or None."""
    if len(frame) >= 10 and frame[0] == 0xFD:
        return frame[7] | (frame[8] << 8) | (frame[9] << 16)
    if len(frame) >= 6 and frame[0] == 0xFE:
        return frame[5]
    return None


def message_name(msg_id):
    cls = mavlink2.mavlink_map.get(msg_id)
    return getattr(cls, "msgname", None) or getattr(cls, "name", None) or str(msg_id)


class SerialTransmitQueue:
    """FIFO byte queue that preserves data across partial serial writes."""

    def __init__(self):
        self._chunks = collections.deque()
        self.total_bytes = 0

    def append(self, data):
        if not data:
            return
        chunk = memoryview(bytes(data))
        self._chunks.append(chunk)
        self.total_bytes += len(chunk)

    def first(self):
        if not self._chunks:
            return None
        return self._chunks[0]

    def consume(self, count):
        if count <= 0 or not self._chunks:
            return
        chunk = self._chunks[0]
        if count >= len(chunk):
            self._chunks.popleft()
            self.total_bytes -= len(chunk)
        else:
            self._chunks[0] = chunk[count:]
            self.total_bytes -= count

    def __bool__(self):
        return bool(self._chunks)


class DownlinkLimiter:
    """PX4 -> ELRS: whole MAVLink frames only, within a byte budget."""

    def __init__(self, max_bps, burst_bytes, default_hz):
        self.parser = mavlink2.MAVLink(None)
        self.parser.robust_parsing = True
        self.max_bps = float(max_bps)
        self.capacity = float(max(burst_bytes, MAX_FRAME_BYTES))
        self.tokens = self.capacity
        self.last_refill = time.monotonic()
        self.default_hz = default_hz
        self.priority = collections.deque()   # (msg_id, frame)
        self.priority_bytes = 0
        self.slots = collections.OrderedDict()   # (msg_id, sys, comp) -> latest frame
        self.next_due = {}
        self.offered_bytes = 0
        self.sent_bytes = 0
        self.malformed_bytes = 0
        self.sent = collections.Counter()
        self.filtered = collections.Counter()     # type not forwarded at all
        self.superseded = collections.Counter()   # replaced by a newer frame before its turn
        self.priority_overflow = collections.Counter()

    def rate_for(self, msg_id):
        return THROTTLE_HZ.get(msg_id, self.default_hz)

    def add(self, data):
        self.offered_bytes += len(data)
        for msg in self.parser.parse_buffer(data) or []:
            frame = bytes(msg.get_msgbuf())
            # Messages this pymavlink's dialect doesn't define (e.g. EVENT
            # 410-413 before pymavlink knew them) arrive as UNKNOWN_<id> with
            # the whole frame -- or, in some versions, as BAD_DATA "unknown
            # MAVLink message ID". Route both by the ID in the frame header.
            if msg.get_type() == "BAD_DATA":
                reason = str(getattr(msg, "reason", "")).lower()
                msg_id = frame_msg_id(frame) if "unknown" in reason else None
                if msg_id is None:
                    self.malformed_bytes += len(frame)
                    continue
                if msg_id not in PRIORITY_IDS and msg_id not in THROTTLE_HZ:
                    self.filtered[msg_id] += 1
                    continue
                key = (msg_id, frame[5], frame[6]) if frame[0] == 0xFD else (msg_id, frame[3], frame[4])
            else:
                msg_id = msg.get_msgId()
                if msg_id < 0:   # UNKNOWN_<id> reports a placeholder ID (-2)
                    msg_id = frame_msg_id(frame)
                    if msg_id is None:
                        self.malformed_bytes += len(frame)
                        continue
                key = (msg_id, msg.get_srcSystem(), msg.get_srcComponent())
            if msg_id in PRIORITY_IDS:
                if self.priority_bytes + len(frame) > MAX_PRIORITY_BYTES:
                    self.priority_overflow[msg_id] += 1
                    continue
                self.priority.append((msg_id, frame))
                self.priority_bytes += len(frame)
                continue
            if self.rate_for(msg_id) <= 0:
                self.filtered[msg_id] += 1
                continue
            if key in self.slots:
                self.superseded[msg_id] += 1
            self.slots[key] = frame

    def _pick(self, now):
        """Next frame to send: due heartbeats, then priority, then the
        throttled slot that has been due longest."""
        due = [k for k in self.slots if now >= self.next_due.get(k, 0.0)]
        for key in due:
            if key[0] == HEARTBEAT_ID:
                return "slot", key, self.slots[key]
        if self.priority:
            msg_id, frame = self.priority[0]
            return "priority", msg_id, frame
        if due:
            key = min(due, key=lambda k: self.next_due.get(k, 0.0))
            return "slot", key, self.slots[key]
        return None

    def take(self, now):
        """Bytes to write now: whole frames, in priority order, that fit the
        budget. Stops at the first frame that doesn't fit, so nothing lower
        priority jumps ahead of it."""
        self.tokens = min(self.capacity,
                          self.tokens + (now - self.last_refill) * self.max_bps)
        self.last_refill = now
        out = bytearray()
        while True:
            pick = self._pick(now)
            if pick is None:
                break
            kind, key, frame = pick
            if len(frame) > self.tokens:
                break
            if kind == "priority":
                self.priority.popleft()
                self.priority_bytes -= len(frame)
                msg_id = key
            else:
                del self.slots[key]
                msg_id = key[0]
                self.next_due[key] = now + 1.0 / self.rate_for(msg_id)
            self.tokens -= len(frame)
            self.sent[msg_id] += 1
            self.sent_bytes += len(frame)
            out.extend(frame)
        return bytes(out)

    def report(self):
        dropped = self.filtered + self.superseded + self.priority_overflow
        top = ", ".join("{}={}".format(message_name(i), n)
                        for i, n in dropped.most_common(6))
        line = ("Downlink limiter: sent_frames={} filtered={} superseded={} "
                "priority_overflow={} priority_queue={} B malformed={} B; "
                "top dropped: {}".format(
                    sum(self.sent.values()), sum(self.filtered.values()),
                    sum(self.superseded.values()),
                    sum(self.priority_overflow.values()), self.priority_bytes,
                    self.malformed_bytes, top or "none"))
        self.sent.clear()
        self.filtered.clear()
        self.superseded.clear()
        self.priority_overflow.clear()
        return line


# Ground -> onboard: STATUSTEXT whose text starts with this is ALSO copied to
# --onboard-port, where beacon_mavlink_bridge.py listens (e.g. the
# "SEABIRD_RUNSTART <run_id>" relay). Everything still goes to PX4 as before.
ONBOARD_STATUSTEXT_PREFIX = "SEABIRD_"


def filter_gcs_to_px4(parser, data):
    """Return allowed MAVLink frames, the frames to copy onboard, and
    filtering statistics."""
    forwarded = bytearray()
    onboard = bytearray()
    blocked_messages = 0
    malformed_bytes = 0

    messages = parser.parse_buffer(data) or []
    for message in messages:
        frame = message.get_msgbuf()

        if message.get_type() == "BAD_DATA":
            malformed_bytes += len(frame)
            continue

        if message.get_msgId() in BLOCKED_GCS_TO_PX4_MESSAGE_IDS:
            blocked_messages += 1
            continue

        forwarded.extend(frame)
        if (message.get_type() == "STATUSTEXT"
                and str(message.text).startswith(ONBOARD_STATUSTEXT_PREFIX)):
            onboard.extend(frame)

    return bytes(forwarded), bytes(onboard), blocked_messages, malformed_bytes


def main():
    parser = argparse.ArgumentParser(
        description="Rate-limited bidirectional ELRS serial-to-VOXL MAVLink bridge"
    )
    parser.add_argument(
        "--serial", default="/dev/ttyHS1",
        help="Serial device connected to the airborne ELRS receiver",
    )
    parser.add_argument(
        "--baud", type=int, default=460800,
        help="ELRS UART baud rate",
    )
    parser.add_argument(
        "--local-ip", default="127.0.0.2",
        help="UDP address on which this bridge listens",
    )
    parser.add_argument(
        "--local-port", type=int, default=14550,
        help="UDP port on which this bridge listens",
    )
    parser.add_argument(
        "--server-ip", default="127.0.0.1",
        help="VOXL MAVLink server address",
    )
    parser.add_argument(
        "--server-port", type=int, default=14550,
        help="VOXL MAVLink server port",
    )
    parser.add_argument(
        "--onboard-ip", default="127.0.0.1",
        help="Where ground STATUSTEXT starting with 'SEABIRD_' is also copied "
             "(beacon_mavlink_bridge.py's onboard listener)",
    )
    parser.add_argument(
        "--onboard-port", type=int, default=14570,
        help="UDP port for those copies (0 = don't copy)",
    )
    parser.add_argument(
        "--max-bps", type=float, default=800.0,
        help="Downlink budget in bytes/s; keep below the ELRS air rate (~1000)",
    )
    parser.add_argument(
        "--burst", type=int, default=300,
        help="Largest downlink burst in bytes (raised to one max-size frame)",
    )
    parser.add_argument(
        "--default-hz", type=float, default=0.0,
        help="Rate for message types not in PRIORITY_IDS/THROTTLE_HZ (0 = drop)",
    )
    parser.add_argument(
        "--no-limit", action="store_true",
        help="Forward the downlink unchanged, like the original bridge",
    )
    parser.add_argument(
        "--debug", action="store_true",
        help="Print individual transfer events; avoid during normal operation",
    )
    args = parser.parse_args()

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    serial_port = serial.Serial(
        port=args.serial,
        baudrate=args.baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0,
        write_timeout=0,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    )

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setblocking(False)
    udp_socket.setsockopt(
        socket.SOL_SOCKET, socket.SO_RCVBUF, UDP_SOCKET_BUFFER_SIZE
    )
    udp_socket.setsockopt(
        socket.SOL_SOCKET, socket.SO_SNDBUF, UDP_SOCKET_BUFFER_SIZE
    )

    local_endpoint = (args.local_ip, args.local_port)
    server_endpoint = (args.server_ip, args.server_port)
    udp_socket.bind(local_endpoint)

    serial_tx_queue = SerialTransmitQueue()
    limiter = None if args.no_limit else DownlinkLimiter(
        args.max_bps, args.burst, args.default_hz)

    # This parser persists across serial reads so split MAVLink frames are
    # reassembled correctly. Robust parsing converts noise into BAD_DATA.
    gcs_to_px4_parser = mavutil.mavlink.MAVLink(None)
    gcs_to_px4_parser.robust_parsing = True

    serial_to_udp_bytes = 0
    udp_to_serial_bytes = 0
    serial_read_errors = 0
    serial_write_errors = 0
    udp_read_errors = 0
    udp_write_errors = 0
    dropped_serial_bytes = 0
    blocked_gcs_to_px4_messages = 0
    malformed_gcs_to_px4_bytes = 0

    last_report = time.monotonic()
    previous_serial_to_udp = 0
    previous_udp_to_serial = 0
    previous_offered = 0

    actual_receive_buffer = udp_socket.getsockopt(
        socket.SOL_SOCKET, socket.SO_RCVBUF
    )
    actual_send_buffer = udp_socket.getsockopt(
        socket.SOL_SOCKET, socket.SO_SNDBUF
    )

    print("Serial: {} at {} baud".format(args.serial, args.baud), flush=True)
    print(
        "Local GCS endpoint: {}:{}".format(*local_endpoint), flush=True
    )
    print(
        "VOXL MAVLink server: {}:{}".format(*server_endpoint), flush=True
    )
    print(
        "UDP buffers: receive={} bytes, send={} bytes".format(
            actual_receive_buffer, actual_send_buffer
        ),
        flush=True,
    )
    print(
        "Blocking GCS->PX4 MAVLink message IDs: {}".format(
            sorted(BLOCKED_GCS_TO_PX4_MESSAGE_IDS)
        ),
        flush=True,
    )
    print(
        "Ground STATUSTEXT '{}*' also copied onboard to {}".format(
            ONBOARD_STATUSTEXT_PREFIX,
            "{}:{}".format(args.onboard_ip, args.onboard_port) if args.onboard_port else "(disabled)"
        ),
        flush=True,
    )
    if limiter is None:
        print("Downlink: NOT rate-limited (--no-limit)", flush=True)
    else:
        print(
            "Downlink: rate-limited to {:.0f} B/s, bursts <= {:.0f} B, "
            "other message types at {} Hz".format(
                limiter.max_bps, limiter.capacity, args.default_hz
            ),
            flush=True,
        )
    print("Waiting for MAVLink traffic. Press Ctrl+C to stop.", flush=True)

    try:
        serial_fd = serial_port.fileno()

        while running:
            readable = [serial_fd, udp_socket]
            writable = [serial_fd] if serial_tx_queue else []

            try:
                ready_read, ready_write, _ = select.select(
                    readable, writable, [], SELECT_TIMEOUT
                )
            except InterruptedError:
                continue

            # ELRS/GCS -> VOXL/PX4: parse frames and block message ID 70.
            if serial_fd in ready_read:
                try:
                    waiting = serial_port.in_waiting
                    read_size = min(max(waiting, 1), SERIAL_READ_SIZE)
                    data = serial_port.read(read_size)

                    if data:
                        filtered, onboard, blocked, malformed = filter_gcs_to_px4(
                            gcs_to_px4_parser, data
                        )
                        if onboard and args.onboard_port:
                            try:
                                udp_socket.sendto(onboard, (args.onboard_ip, args.onboard_port))
                            except OSError:
                                pass   # nothing listening onboard: PX4 still gets it below
                        blocked_gcs_to_px4_messages += blocked
                        malformed_gcs_to_px4_bytes += malformed

                        if blocked and args.debug:
                            print(
                                "FILTER: blocked {} message(s) with ID 70".format(
                                    blocked
                                ),
                                flush=True,
                            )

                        if filtered:
                            try:
                                sent = udp_socket.sendto(filtered, server_endpoint)
                                serial_to_udp_bytes += sent
                            except OSError as error:
                                udp_write_errors += 1
                                print(
                                    "UDP send error: {}".format(error),
                                    file=sys.stderr,
                                    flush=True,
                                )
                            else:
                                if args.debug:
                                    print(
                                        "SER -> UDP: {} bytes".format(sent),
                                        flush=True,
                                    )

                except serial.SerialException as error:
                    serial_read_errors += 1
                    raise RuntimeError("Serial read failed: {}".format(error))
                except OSError as error:
                    serial_read_errors += 1
                    raise RuntimeError("Serial read failed: {}".format(error))

            # VOXL/PX4 -> ELRS/GCS: into the rate limiter (or, with
            # --no-limit, forwarded unchanged).
            if udp_socket in ready_read:
                while running:
                    try:
                        data, source = udp_socket.recvfrom(65535)
                    except BlockingIOError:
                        break
                    except OSError as error:
                        if error.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                            break
                        udp_read_errors += 1
                        print(
                            "UDP receive error: {}".format(error),
                            file=sys.stderr,
                            flush=True,
                        )
                        break

                    if not data:
                        continue

                    if limiter is not None:
                        limiter.add(data)
                        continue

                    projected_queue_size = serial_tx_queue.total_bytes + len(data)
                    if projected_queue_size > MAX_SERIAL_TX_QUEUE:
                        dropped_serial_bytes += len(data)
                        print(
                            "ERROR: serial queue full; dropping {} bytes. "
                            "Queue currently contains {} bytes.".format(
                                len(data), serial_tx_queue.total_bytes
                            ),
                            file=sys.stderr,
                            flush=True,
                        )
                        continue

                    serial_tx_queue.append(data)
                    if args.debug:
                        print(
                            "UDP queued for serial: {} bytes from {}:{}".format(
                                len(data), source[0], source[1]
                            ),
                            flush=True,
                        )

            # Hand the limiter's next frames to the serial queue once the
            # previous ones are written, so pacing stays with the limiter.
            if limiter is not None and not serial_tx_queue:
                chunk = limiter.take(time.monotonic())
                if chunk:
                    serial_tx_queue.append(chunk)
                    if args.debug:
                        print(
                            "LIMITER -> serial queue: {} bytes".format(len(chunk)),
                            flush=True,
                        )

            # Preserve unwritten data when PySerial performs a partial write.
            if serial_tx_queue and serial_fd in ready_write:
                while serial_tx_queue:
                    chunk = serial_tx_queue.first()
                    if chunk is None:
                        break

                    try:
                        written = serial_port.write(chunk)
                    except serial.SerialTimeoutException:
                        break
                    except serial.SerialException as error:
                        serial_write_errors += 1
                        raise RuntimeError("Serial write failed: {}".format(error))
                    except OSError as error:
                        if error.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                            break
                        serial_write_errors += 1
                        raise RuntimeError("Serial write failed: {}".format(error))

                    if not written:
                        break

                    serial_tx_queue.consume(written)
                    udp_to_serial_bytes += written
                    if args.debug:
                        print(
                            "UDP -> SER: wrote {} bytes".format(written),
                            flush=True,
                        )

            now = time.monotonic()
            elapsed = now - last_report
            if elapsed >= REPORT_INTERVAL:
                serial_to_udp_rate = (
                    serial_to_udp_bytes - previous_serial_to_udp
                ) / elapsed
                udp_to_serial_rate = (
                    udp_to_serial_bytes - previous_udp_to_serial
                ) / elapsed

                print(
                    "Traffic: "
                    "SER->UDP total={} B rate={:.0f} B/s, "
                    "UDP->SER total={} B rate={:.0f} B/s, "
                    "serial_queue={} B, dropped={} B, "
                    "blocked_id70={} messages, malformed={} B, "
                    "errors: serial_read={} serial_write={} "
                    "udp_read={} udp_write={}".format(
                        serial_to_udp_bytes,
                        serial_to_udp_rate,
                        udp_to_serial_bytes,
                        udp_to_serial_rate,
                        serial_tx_queue.total_bytes,
                        dropped_serial_bytes,
                        blocked_gcs_to_px4_messages,
                        malformed_gcs_to_px4_bytes,
                        serial_read_errors,
                        serial_write_errors,
                        udp_read_errors,
                        udp_write_errors,
                    ),
                    flush=True,
                )
                if limiter is not None:
                    offered_rate = (limiter.offered_bytes - previous_offered) / elapsed
                    previous_offered = limiter.offered_bytes
                    print(
                        "  PX4 offered {:.0f} B/s; {}".format(
                            offered_rate, limiter.report()
                        ),
                        flush=True,
                    )

                previous_serial_to_udp = serial_to_udp_bytes
                previous_udp_to_serial = udp_to_serial_bytes
                last_report = now

    finally:
        try:
            serial_port.close()
        finally:
            udp_socket.close()
        print("Bridge stopped.", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print("Bridge error: {}".format(error), file=sys.stderr, flush=True)
        sys.exit(1)
