#!/usr/bin/env python3
"""VOXL2 system + PX4 battery CSV logger. Python standard library only.

Reads PX4 battery_status using `px4-listener battery_status` in a background
thread so a slow/unavailable listener never holds up CPU measurements.
Battery values are snapshots, not a direct measurement of VOXL2's 5 V input.
"""
import argparse
import csv
import glob
import os
import re
import shlex
import shutil
import signal
import subprocess
import threading
import time
from datetime import datetime, timezone

running = True

def stop(_sig, _frame):
    global running
    running = False

signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)

def read(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except (OSError, ValueError):
        return ''

def cpu_times():
    values = {}
    for line in read('/proc/stat').splitlines():
        s = line.split()
        if s and (s[0] == 'cpu' or (s[0].startswith('cpu') and s[0][3:].isdigit())):
            nums = [int(x) for x in s[1:]]
            idle = nums[3] + (nums[4] if len(nums) > 4 else 0)
            values[s[0]] = (sum(nums), idle)
    return values

def percent(previous, current, name):
    if name not in previous or name not in current:
        return ''
    delta_total = current[name][0] - previous[name][0]
    delta_idle = current[name][1] - previous[name][1]
    return round(100 * (1 - delta_idle / delta_total), 1) if delta_total > 0 else ''

def memory_mb():
    vals = {}
    for line in read('/proc/meminfo').splitlines():
        if ':' in line:
            k, v = line.split(':', 1)
            try:
                vals[k] = int(v.strip().split()[0]) / 1024
            except (ValueError, IndexError):
                pass
    return (round(vals.get('MemAvailable', 0), 1),
            round(vals.get('MemTotal', 0) - vals.get('MemAvailable', 0), 1))

def temperatures():
    all_temps, cpu_temps, gpu_temps = [], [], []
    for path in glob.glob('/sys/class/thermal/thermal_zone*'):
        typ = read(path + '/type').lower()
        raw = read(path + '/temp')
        try:
            t = float(raw)
            if abs(t) > 1000:
                t /= 1000.0
            if not -30 <= t <= 150:
                continue
        except ValueError:
            continue
        all_temps.append(t)
        if any(x in typ for x in ('cpu', 'silver', 'gold')):
            cpu_temps.append(t)
        if 'gpu' in typ:
            gpu_temps.append(t)
    return [round(max(v), 1) if v else '' for v in (cpu_temps, gpu_temps, all_temps)]

def frequency_mhz(core):
    s = read('/sys/devices/system/cpu/cpu%d/cpufreq/scaling_cur_freq' % core)
    try:
        return round(float(s) / 1000, 1)
    except ValueError:
        return ''

def gpu_freq_mhz():
    s = read('/sys/class/kgsl/kgsl-3d0/devfreq/cur_freq')
    try:
        return round(float(s) / 1e6, 1)
    except ValueError:
        return ''

# PX4 listener generally prints "voltage_v: 15.2" lines, possibly indented.
BATTERY_KEYS = ('voltage_v', 'voltage_filtered_v', 'current_a', 'current_filtered_a',
                'remaining', 'temperature', 'connected', 'warning',
                'discharged_mah', 'timestamp')
FIELD_RE = re.compile(r'^\s*([A-Za-z_][A-Za-z_0-9]*)\s*:\s*(\S+)')

def battery_snapshot(command, timeout):
    try:
        result = subprocess.run(command, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, timeout=timeout,
                                universal_newlines=True)
    except (OSError, subprocess.TimeoutExpired):
        return {}
    if result.returncode != 0:
        return {}
    values = {}
    for line in result.stdout.splitlines():
        match = FIELD_RE.match(line)
        if match and match.group(1) in BATTERY_KEYS:
            values[match.group(1)] = match.group(2)
    if 'voltage_v' not in values and 'voltage_filtered_v' not in values:
        return {}
    return values

class BatterySampler:
    def __init__(self, command, interval, timeout):
        self.command = command
        self.interval = interval
        self.timeout = timeout
        self.lock = threading.Lock()
        self.latest = {}
        self.last_time = None
        self.thread = threading.Thread(target=self.loop, daemon=True)

    def start(self):
        self.thread.start()

    def loop(self):
        while running:
            snapshot = battery_snapshot(self.command, self.timeout)
            with self.lock:
                if snapshot:
                    self.latest = snapshot
                    self.last_time = time.monotonic()
            deadline = time.monotonic() + self.interval
            while running and time.monotonic() < deadline:
                time.sleep(min(0.1, max(0, deadline - time.monotonic())))

    def get(self):
        with self.lock:
            return dict(self.latest), self.last_time

def numeric(s):
    try:
        value = float(s)
        if not (-1e20 < value < 1e20):
            return None
        return value
    except (ValueError, TypeError):
        return None

def battery_columns(snapshot, age, max_age):
    cols = {'battery_age_s': round(age, 2) if age is not None else '',
            'battery_voltage_v': '', 'battery_current_a': '',
            'battery_power_w': '', 'battery_remaining_pct': '',
            'battery_temp_c': '', 'battery_connected': '',
            'battery_warning': '', 'battery_discharged_mah': ''}
    if age is None or age > max_age:
        return cols  # Never present stale readings as current.
    voltage = numeric(snapshot.get('voltage_v', snapshot.get('voltage_filtered_v')))
    current = numeric(snapshot.get('current_a', snapshot.get('current_filtered_a')))
    remaining = numeric(snapshot.get('remaining'))
    temp = numeric(snapshot.get('temperature'))
    discharged = numeric(snapshot.get('discharged_mah'))
    if voltage is not None and voltage >= 0:
        cols['battery_voltage_v'] = round(voltage, 3)
    if current is not None and current >= 0:
        cols['battery_current_a'] = round(current, 3)
    if voltage is not None and voltage >= 0 and current is not None and current >= 0:
        cols['battery_power_w'] = round(voltage * current, 2)
    if remaining is not None and 0 <= remaining <= 1:
        cols['battery_remaining_pct'] = round(remaining * 100, 1)
    if temp is not None and -40 <= temp <= 150:
        cols['battery_temp_c'] = round(temp, 1)
    cols['battery_connected'] = snapshot.get('connected', '')
    cols['battery_warning'] = snapshot.get('warning', '')
    if discharged is not None and discharged >= 0:
        cols['battery_discharged_mah'] = round(discharged, 1)
    return cols

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('-o', '--output', default='voxl_health.csv')
    p.add_argument('-i', '--interval', type=float, default=1.0,
                   help='CPU metrics interval in seconds (default: 1)')
    p.add_argument('--battery-interval', type=float, default=2.0,
                   help='Interval between PX4 battery queries (default: 2)')
    p.add_argument('--battery-timeout', type=float, default=2.0,
                   help='Timeout for each battery query (default: 2)')
    p.add_argument('--battery-command', default='px4-listener battery_status',
                   help='Command to query battery_status')
    p.add_argument('--no-battery', action='store_true',
                   help='Disable PX4 battery queries')
    args = p.parse_args()
    if args.interval < 0.5 or args.battery_interval < 0.5 or args.battery_timeout <= 0:
        p.error('intervals must be >= 0.5 s and battery timeout > 0')
    command = shlex.split(args.battery_command)
    if not command:
        p.error('battery command cannot be empty')
    sampler = None
    if not args.no_battery:
        if shutil.which(command[0]) is None:
            print('WARNING: %s not found; battery columns will be blank.' % command[0], flush=True)
        else:
            sampler = BatterySampler(command, args.battery_interval, args.battery_timeout)
            sampler.start()
    core_count = os.cpu_count() or 8
    cols = (['timestamp_utc', 'uptime_s', 'cpu_total_pct'] +
            [f'cpu{i}_pct' for i in range(core_count)] +
            [f'cpu{i}_mhz' for i in range(core_count)] +
            ['cpu_temp_max_c', 'gpu_temp_max_c', 'thermal_max_c', 'gpu_mhz',
             'ram_available_mb', 'ram_used_mb', 'load_1m', 'load_5m', 'load_15m',
             'battery_age_s', 'battery_voltage_v', 'battery_current_a',
             'battery_power_w', 'battery_remaining_pct', 'battery_temp_c',
             'battery_connected', 'battery_warning', 'battery_discharged_mah'])
    previous = cpu_times()
    with open(args.output, 'w', newline='', buffering=1) as out:
        writer = csv.DictWriter(out, fieldnames=cols)
        writer.writeheader()
        print('Logging to', os.path.abspath(args.output), 'Ctrl+C to stop', flush=True)
        print('Battery source: PX4 battery_status (not VOXL2 regulated 5 V input)', flush=True)
        while running:
            time.sleep(args.interval)
            current = cpu_times()
            cpu_c, gpu_c, max_c = temperatures()
            ram_avail, ram_used = memory_mb()
            loads = os.getloadavg()
            row = {
                'timestamp_utc': datetime.now(timezone.utc).isoformat(),
                'uptime_s': read('/proc/uptime').split()[0],
                'cpu_total_pct': percent(previous, current, 'cpu'),
                'cpu_temp_max_c': cpu_c, 'gpu_temp_max_c': gpu_c,
                'thermal_max_c': max_c, 'gpu_mhz': gpu_freq_mhz(),
                'ram_available_mb': ram_avail, 'ram_used_mb': ram_used,
                'load_1m': round(loads[0], 2), 'load_5m': round(loads[1], 2),
                'load_15m': round(loads[2], 2),
            }
            for i in range(core_count):
                row[f'cpu{i}_pct'] = percent(previous, current, f'cpu{i}')
                row[f'cpu{i}_mhz'] = frequency_mhz(i)
            if sampler:
                snap, sampled_at = sampler.get()
                age = time.monotonic() - sampled_at if sampled_at is not None else None
                row.update(battery_columns(snap, age, args.battery_interval + args.battery_timeout + 2))
            else:
                row.update(battery_columns({}, None, 0))
            writer.writerow(row)
            out.flush()
            os.fsync(out.fileno())
            previous = current
    print('Stopped.', flush=True)

if __name__ == '__main__':
    main()
