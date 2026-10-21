# ELRS Beacon Relay Testing

Logs from bench tests of the beacon-detection relay over the ELRS radio link, plus `evaluate_beacon_tests.py`. The script scores each run on data accuracy, message delivery rate, end-to-end latency, update frequency and system stability, and draws charts comparing the runs.

Each test publishes 100 synthetic beacon detections on the UAV and logs them at every stage of the pipeline:

```
ROS publisher ──► beacon_mavlink_bridge.py ──► ELRS radio ──► elrs_windows_router.py ──► beacon_mavlink_ground.py
 (UAV)             (UAV, MEMORY_VECT)                          (Windows)                   (WSL)
 source log        relay log                                   router log                  ground log
```

The wire format is described in [`utils/beacon_mavlink_protocol.py`](../utils/beacon_mavlink_protocol.py) (see [`utils/README.md`](../utils/README.md)).

---

## Folder Overview

| Path | Contents |
|---|---|
| `NOTES.md` | One line per test describing its settings (publish rate, repeats, telemetry stream changes). The evaluator parses this file. |
| `test1/` … `test7/` | Raw CSV logs from each run (see [Log files](#log-files)). |
| `evaluate_beacon_tests.py` | Evaluation script. Reads every `testN/` folder and writes to `results/`. |
| `results/` | Generated output. Safe to delete; rerun the script to rebuild it. |

---

## Running the Evaluation

Requires Python 3 with `pandas`, `numpy` and `matplotlib`. No `pymavlink` or ROS needed.

```
python elrs_testing/evaluate_beacon_tests.py
python elrs_testing/evaluate_beacon_tests.py --root elrs_testing --out elrs_testing/results
```

| Option | Default | Meaning |
|---|---|---|
| `--root` | the script's folder | Folder holding `NOTES.md` and the `testN/` folders |
| `--out` | `<root>/results` | Where results are written |

The console prints any rows it discarded and any mismatch between `NOTES.md` and the logs, then a one-line summary per test.

### Output

| File | Contents |
|---|---|
| `report.md` | Full write-up: config table, one table per metric category, all charts, caveats |
| `summary_metrics.csv` | One row per test, every metric |
| `per_message.csv` | One row per published detection: timestamps at each stage, per-hop latency, which copy arrived, fate (`delivered`, `delivered_copy2`, `lost_radio`, `lost_ground`, `not_relayed`) |
| `overview.png` | 2×3 dashboard: delivery, latency, update rate, worst outage, accuracy, link health |
| `delivery_timeline.png` | Fate of every detection, per test, in publish order. Shows loss bursts |
| `latency_over_time.png` | End-to-end latency of each detection across each run, with losses marked |
| `latency_by_hop.png` | Latency split into UAV / radio / ground hops |
| `update_intervals.png` | Histogram of time between ground updates vs. the publish period |
| `link_load.png` | MAVLink frames/s by type sharing the ELRS link, including corrupt frames |

---

## Log Files

File names were **not** kept consistent between tests (e.g. `published_uav.csv` is the *source* log in test1 but the *relay* log in tests 2–7). The evaluator therefore ignores file names and identifies each CSV by its columns:

| Role | Identified by | Written by | Key columns |
|---|---|---|---|
| source | has `topic` | ROS topic logger on the UAV | `epoch_time`, `tracking_id`, `color`, `blink`, `latitude`, `longitude` — ground truth |
| relay | has `copy`, `copies` or `ros_timestamp_unix_seconds` | `beacon_mavlink_bridge.py` | send time, `tracking_id`, and (later tests) `msg_id`, `copy` |
| router | has `hex_payload` | `elrs_windows_router.py` | every MAVLink frame forwarded to WSL, with `msg_type`, `mav_seq`, `tracking_id` |
| ground | has `is_blinking` and none of the above | `beacon_mavlink_ground.py` | decoded detections output on WSL |

| Test | source | relay | router | ground |
|---|---|---|---|---|
| test1 | `published_uav.csv` | `uav_mavlink_relay.csv` | `elrs_to_wsl.csv` (raw bytes) | `wsl_output.csv` |
| test2–4, 6–7 | `elrs_uav.csv` | `published_uav.csv` | `elrs_to_wsl.csv` | `wsl_output.csv` |
| test5 | `elrs_uav.csv` | *(missing)* | `elrs_to_wsl.csv` | `wsl_output.csv` |

Quirks the evaluator handles:

- **Appended logs.** The loggers open files in append mode, so some logs contain rows from earlier runs (test4's relay log holds all of test3's; test7's has 2 rows from an aborted start). Rows more than 5 s before the first publish or 15 s after the last are dropped.
- **Test1's router log is raw serial chunks**, not parsed frames. Beacon frames are recovered by scanning the reassembled byte stream for CRC-valid `MEMORY_VECT` frames with address `0xBEAC`. Per-message-type link stats are unavailable for test1.
- **Which copy arrived (tests 6–7).** The bridge's MAVLink component (sysid 1, compid 191) sends only beacons, so its sequence number is `offset + copies × msg_id + (copy − 1)` mod 256. This identifies whether the first or the repeat copy got through.
- **Router `UNKNOWN_*` frames** all come from sysid/compid 0/0. They are corrupted byte runs the parser resynced on, and are counted as corrupt frames.

---

## NOTES.md Format

One line per test:

```
test6 - publish messages at 0.5 Hz and send each message twice - ignore duplicates
```

The evaluator extracts:

| Phrase | Setting |
|---|---|
| `publish(ed) ... at X Hz` | publish rate |
| `twice` / `N times` / `N copies` | copies per detection |
| `change <stream> [and <stream>] frequency ... to X Hz` | telemetry stream rates (`ATTITUDE`, `ATTITUDE_QUATERNION`, `VFR_HUD`) |
| `pymavlink ... windows` | decoding moved to the Windows router |

A setting a line doesn't mention carries over from the previous test (test3 and test4 inherit test2's 1 Hz). The settings are only labels: publish rate and copies are measured from the logs, and the script prints a warning if they disagree with the notes. The notes text appears in the chart captions and the report's config table.

---

## Metrics

All delivery and latency figures are per published detection (by `tracking_id`). Duplicate copies count as one delivery, at the time the first copy arrived.

### Data accuracy
Every ground row (duplicates included) is compared with the published detection with the same `tracking_id`. Position is correct within 5 cm (the wire format quantizes to 1e-7°, ~1 cm). Color and blink state must match exactly. `phantom_rows` counts ground rows whose `tracking_id` was never published.

### Message delivery rate
| Metric | Meaning |
|---|---|
| `e2e_delivery_pct` | Published detections that came out of the ground decoder |
| `relay_pct` | Published detections the bridge sent |
| `radio_delivery_pct` | Sent detections the Windows router received (any copy) |
| `router_to_ground_pct` | Router-received detections the WSL decoder output |
| `radio_frame_delivery_pct` | Individual radio frames received / sent (each copy counts) |
| `radio_delivery_if_independent_pct` | Delivery expected from the frame rate if losses were random. Observed delivery below this means losses are bursty |
| `copy1_delivery_pct`, `saved_by_repeat` | Repeat tests only: how often the first copy alone made it, and how many detections only the repeat delivered |

### End-to-end latency
Ground output time minus publish time, plus the same split into hops: UAV (publish → bridge send), radio (send time of the copy that arrived first → router), and ground (router → WSL output). Reported as median, mean, p95, min, max, σ.

### Update frequency
Built from the times new detections appeared at the ground. `achieved_update_hz` vs. `publish_hz`, median/p95 interval, longest gap, and the coefficient of variation of the intervals.

### System stability
| Metric | Meaning |
|---|---|
| `latency_jitter_s` | σ of end-to-end latency |
| `latency_tail_s` | p95 − median latency |
| `latency_drift_s_per_min` | Linear latency trend over the run (queue build-up shows as positive) |
| `max_gap_publish_periods` | Longest gap expressed in publish periods |
| `longest_loss_streak` | Most consecutive detections lost |
| `rolling_delivery_min_pct` | Worst delivery rate over any 10 consecutive detections |
| `delivery_first_half_pct`, `delivery_second_half_pct` | Delivery in each half of the run |
| `corrupt_frame_pct` | Router frames that failed to parse (`UNKNOWN_*`) |
| `fc_telemetry_loss_pct` | Autopilot (sysid 1, compid 1) frames lost, from MAVLink sequence gaps |
| `link_frames_per_s`, `link_bytes_per_s` | Total MAVLink load received over the link |

---

## Caveats

- **Clocks.** Latencies subtract timestamps from three clocks (UAV, Windows, WSL). Their offsets are included in the absolute numbers. The router → WSL hop currently comes out around −0.1 s, which is that offset showing through. Comparisons between tests are fair if the offsets stayed constant. Sync the clocks (NTP/chrony) before quoting absolute latency.
- **Test5 has no relay log**, so its radio hop is measured from the ROS publish time and its bridge delivery is unknown.
- **Test1 router stats** are limited to beacon frames and raw byte throughput (see [Log Files](#log-files)).

---

## Adding a New Test

1. Create `testN/` and copy in the four logs. Names don't matter; the columns identify each file.
2. Start each logger with a fresh file, or leave the 5 s / 15 s window filter to drop older rows.
3. Add a `testN - ...` line to `NOTES.md` using the phrases above.
4. Rerun `python elrs_testing/evaluate_beacon_tests.py`.

New tests are picked up automatically, and the charts resize to fit them.
