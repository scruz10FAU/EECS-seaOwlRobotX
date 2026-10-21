#!/usr/bin/env python3
"""Bidirectional ELRS serial-to-UDP fanout for Windows QGC and WSL.

This is a bench-test / development bridge, not flight-qualified.
Only this process should open the radio COM port.
"""
import argparse
import socket
import time
from datetime import datetime

import serial
from serial import SerialException, SerialTimeoutException


def address(value):
    try:
        host, port = value.rsplit(':', 1)
        port = int(port)
        if not host or not 1 <= port <= 65535:
            raise ValueError()
        return (host, port)
    except ValueError as e:
        raise argparse.ArgumentTypeError('Use HOST:PORT, e.g. 127.0.0.1:14550') from e


def log_wsl_send(log, data, status='SENT'):
    now = time.time()
    iso = datetime.fromtimestamp(now).isoformat(timespec='milliseconds')
    log.write(f'{iso},{now:.6f},{status},{len(data)},{data.hex()}\n')
    log.flush()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--com', default='COM6', help='Radio serial port (default: COM6)')
    p.add_argument('--baud', type=int, default=460800)
    p.add_argument('--qgc', type=address, default=('127.0.0.1', 14550), help='QGC UDP destination')
    p.add_argument('--wsl', type=address, default=('127.0.0.1', 14551), help='WSL UDP destination')
    p.add_argument('--listen', type=int, default=14600, help='UDP port for QGC commands')
    p.add_argument('--wsl-log', metavar='PATH', default=None,
                   help='Append a CSV row (time, status, bytes, hex) for every chunk sent to WSL')
    args = p.parse_args()

    wsl_log = None
    if args.wsl_log:
        wsl_log = open(args.wsl_log, 'a', newline='')
        if wsl_log.tell() == 0:
            wsl_log.write('iso_time,epoch_time,status,bytes,hex_payload\n')
            wsl_log.flush()

    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(('127.0.0.1', args.listen))
    udp.setblocking(False)
    # Do not call socket.ioctl(): certain Windows Python versions reject SIO_UDP_CONNRESET.
    print(f'ELRS router | {args.com} @ {args.baud}', flush=True)
    print(f'Radio -> QGC: {args.qgc[0]}:{args.qgc[1]} | WSL: {args.wsl[0]}:{args.wsl[1]}', flush=True)
    print(f'QGC -> radio: 127.0.0.1:{args.listen}', flush=True)
    print('QGC must use UDP, not direct serial. Ctrl+C to stop.', flush=True)

    serial_port = None
    last_stats = time.monotonic()
    rx_bytes = 0
    rx_packets = 0
    try:
        while True:
            if serial_port is None:
                try:
                    serial_port = serial.Serial(args.com, args.baud, timeout=0, write_timeout=0.5)
                    print(f'Connected to {args.com}', flush=True)
                except (SerialException, OSError) as e:
                    print(f'Cannot open {args.com}: {e}; retrying in 3s', flush=True)
                    time.sleep(3)
                    continue
            try:
                n = serial_port.in_waiting
                if n:
                    data = serial_port.read(min(n, 4096))
                    if data:
                        rx_bytes += len(data)
                        rx_packets += 1
                        for dest in (args.qgc, args.wsl):
                            try:
                                udp.sendto(data, dest)
                                if wsl_log and dest is args.wsl:
                                    log_wsl_send(wsl_log, data)
                            except OSError as e:
                                print(f'UDP send to {dest} failed: {e}', flush=True)
                                if wsl_log and dest is args.wsl:
                                    log_wsl_send(wsl_log, data, 'SEND_FAILED')

                while True:
                    try:
                        data, sender = udp.recvfrom(65535)
                    except (BlockingIOError, ConnectionResetError):
                        # WinError 10054 can follow an ICMP port-unreachable response.
                        break
                    except OSError as e:
                        if getattr(e, 'winerror', None) == 10054:
                            break
                        raise
                    if data:
                        serial_port.write(data)
                time.sleep(0.002)
                if time.monotonic() - last_stats >= 5:
                    print(
                        f"Serial RX: {rx_bytes} bytes, "
                        f"UDP fanout: {rx_packets} chunks per destination / 5 sec",
                        flush=True
                    )
                    rx_bytes = 0
                    rx_packets = 0
                    last_stats = time.monotonic()
            except (SerialException, SerialTimeoutException, OSError) as e:
                print(f'Serial connection lost: {e}; reconnecting', flush=True)
                try:
                    serial_port.close()
                except OSError:
                    pass
                serial_port = None
                time.sleep(2)
    except KeyboardInterrupt:
        print('\nStopping.', flush=True)
    finally:
        if serial_port is not None:
            serial_port.close()
        udp.close()
        if wsl_log is not None:
            wsl_log.close()


if __name__ == '__main__':
    main()
