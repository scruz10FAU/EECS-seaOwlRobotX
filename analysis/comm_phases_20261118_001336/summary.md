# Communication metrics, phases 2-4

Generated 2026-11-18 00:13 from 59 ground-GUI sessions and 14 tlogs with flights.

## Phase 2 - flight, UAV -> ground

Sessions: 15. Armed segments in tlogs: 31.

| Metric | Value |
|---|---|
| Total armed time (s) | 1200.2 |
| UAV messages / s (mean of flights) | 29.01 |
| PX4 seq-gap loss %, median of flights (includes rate limiting) | 88.21 |
| Seabird status msgs received | 223 |
| Seabird seq-gap loss % (median of flights) | 0.0 |
| PX4 heartbeat gap max (s) | 9.423 |
| Longest silence, any UAV message (s) | 3.66 |


Per flight:

| tlog | armed at | dur s | msgs/s | PX4 loss % | Seabird loss % | HB gap max s | OCS HB in flight | OCS HB gap max s |
|---|---|---|---|---|---|---|---|---|
| 2026-11-09 10-32-37.tlog | 2026-11-09T07:57:21 | 72.2 | 9.91 | 88.1 |  | 1.277 |  |  |
| 2026-11-09 10-32-37.tlog | 2026-11-09T08:09:42 | 57.1 | 9.72 | 88.18 |  | 1.255 |  |  |
| 2026-11-09 10-32-37.tlog | 2026-11-09T08:11:57 | 146.4 | 9.74 | 88.2 |  | 1.478 |  |  |
| 2026-11-09 10-32-37.tlog | 2026-11-09T08:15:32 | 25.2 | 9.82 | 88.15 |  | 1.279 |  |  |
| 2026-11-09 10-32-37.tlog | 2026-11-09T08:22:31 | 124.5 | 9.82 | 88.44 |  | 2.462 |  |  |
| 2026-11-09 10-32-37.tlog | 2026-11-09T08:27:30 | 73.4 | 9.9 | 88.21 |  | 1.518 |  |  |
| 2026-11-09 11-41-08.tlog | 2026-11-09T08:56:07 | 10.0 | 9.52 | 89.29 |  | 1.168 |  |  |
| 2026-11-09 11-41-08.tlog | 2026-11-09T08:56:31 | 81.3 | 9.94 | 88.25 |  | 1.439 |  |  |
| 2026-11-09 11-41-08.tlog | 2026-11-09T09:37:10 | 7.3 | 100.08 | 11.08 |  | 1.002 | 0 |  |
| 2026-11-09 18-09-55.tlog | 2026-11-09T14:50:38 | 20.0 | 9.81 | 88.6 |  | 1.26 | 0 |  |
| 2026-11-09 18-09-55.tlog | 2026-11-09T14:55:44 | 20.1 | 9.77 | 88.53 |  | 1.264 | 0 |  |
| 2026-11-09 18-09-55.tlog | 2026-11-09T15:12:33 | 13.0 | 9.82 | 88.47 |  | 1.153 |  |  |
| 2026-11-10 09-09-21.tlog | 2026-11-10T07:08:53 | 6.0 | 10.61 | 92.52 |  | 1.026 |  |  |
| 2026-11-10 12-34-16.tlog | 2026-11-10T07:44:19 | 20.0 | 9.81 | 88.04 |  | 1.419 |  |  |
| 2026-11-10 12-34-16.tlog | 2026-11-10T08:15:57 | 38.9 | 10.4 | 87.7 |  | 1.35 |  |  |
| 2026-11-10 12-34-16.tlog | 2026-11-10T10:11:49 | 9.0 | 10.52 | 87.37 |  | 1.061 | 17 | 0.567 |
| 2026-11-11 10-24-53.tlog | 2026-11-10T11:48:23 | 17.0 | 10.66 | 87.89 | 0.0 | 1.239 |  |  |
| 2026-11-11 10-24-53.tlog | 2026-11-10T11:51:52 | 28.0 | 10.45 | 88.34 | 0.0 | 1.106 |  |  |
| 2026-11-11 11-07-41.tlog | 2026-11-11T09:49:32 | 104.1 | 10.13 | 88.26 | 1.92 | 1.961 |  |  |
| 2026-11-12 11-25-58.tlog | 2026-11-12T10:55:08 | 35.1 | 9.88 | 88.21 |  | 1.526 | 70 | 0.729 |
| 2026-11-12 12-19-10.tlog | 2026-11-12T12:10:13 | 11.9 | 10.81 | 88.22 | 0.0 | 1.016 | 24 | 0.606 |
| 2026-11-12 16-29-48.tlog | 2026-11-12T16:16:36 | 20.0 | 76.92 | 14.14 | 0.0 | 1.058 | 39 | 0.751 |
| 2026-11-12 16-29-48.tlog | 2026-11-12T16:26:11 | 13.3 | 133.12 | 4.7 | 0.0 | 1.244 | 26 | 0.65 |
| 2026-11-12 16-29-48.tlog | 2026-11-12T16:29:09 | 27.0 | 134.73 | 8.67 | 0.0 | 1.033 | 54 | 0.753 |
| 2026-11-12 17-47-21.tlog | 2026-11-12T16:43:42 | 10.0 | 137.63 | 10.59 | 0.0 | 1.146 | 20 | 0.731 |
| 2026-11-12 17-47-21.tlog | 2026-11-12T16:59:31 | 23.0 | 76.57 | 46.53 | 0.0 | 0.783 | 45 | 0.776 |
| 2026-11-13 09-03-48.tlog | 2026-11-13T08:38:04 | 18.1 | 10.64 | 88.29 | 0.0 | 1.165 | 35 | 0.791 |
| 2026-11-13 09-03-48.tlog | 2026-11-13T08:56:36 | 61.2 | 8.49 | 91.36 | 22.22 | 9.423 | 121 | 1.159 |
| 2026-11-13 10-08-18.tlog | 2026-11-13T09:57:45 | 30.1 | 10.55 | 88.62 | 0.0 | 1.182 | 55 | 1.068 |
| 2026-11-13 11-53-45.tlog | 2026-11-13T11:18:38 | 42.5 | 2.73 | 94.28 | 78.57 | 5.616 |  |  |
| 2026-11-13 11-53-45.tlog | 2026-11-13T11:33:36 | 34.5 | 6.69 | 92.6 | 29.41 | 3.518 |  |  |


Router, flight sessions:

| Metric | Value |
|---|---|
| Router time logged (s) | 35455.0 |
| Frames received / s | 10.56 |
| Seq-gap loss % (all PX4 traffic, includes onboard rate limiting) | 83.0 |
| Corrupted bytes | 52451 |
| 5 s intervals with no frames after first contact (includes drone powered off) | 1633 |
| Frames via ELRS / WiFi | 29598 / 55970 |
| Cross-link duplicates removed | 7783 |
| Uplink switches ELRS<->WiFi | 15 |


Ground bridge heartbeat gaps (flight sessions, whole session incl. drone powered off/rebooting): 21 recovered, longest 218.0 s. In-flight gaps are the 'HB gap max' column above.


## Phase 3 - bench, UAV -> ground -> OCS/USV

Sessions: 26.

| Metric | Value |
|---|---|
| Status heartbeats to RoboCommand | 32963 |
| Heartbeat rate Hz (median of sessions) | 1.804 |
| Heartbeat gap p95 / max (s) | 1.56 / 208.793 |
| Heartbeat gaps > 2 s | 347 |
| Heartbeats suppressed as stale | 1020 |
| Heartbeats with NaN fields | 116 |
| MQTT (re)connects | 45 |
| RunStart received from RoboCommand | 24 |
| RunStart relayed to UAV / acknowledged | 17 / 11 (64.7 %) |
| Restart requests / acknowledged | 10 / 9 |
| Geofence uploads / accepted / VALID / FAILED | 51 / 48 / 26 / 16 |
| RunDeclarations sent / blocked | 46 / 10 |
| SafePassage reports forwarded to RoboCommand + USV | 15 |
| Task commands forwarded to USV | 0 |
| OCS runs (restarts split at > 300 s silence) | 33 |
| OCS ERROR lines | 11 |
| OCS tracebacks: total / at shutdown (MQTT on_disconnect signature bug) | 41 / 41 |
| Broker client connects / disconnects / timeouts | 18 / 15 / 0 |

| Round trip | requests | matched | min s | median s | p95 s | max s |
|---|---|---|---|---|---|---|
| Restart (ground->UAV->ACK at QGC, 1 s resolution) | 10 | 4 | 11.9 | 12.9 | 14.1 | 14.1 |
| RunStart (RoboCommand->OCS->UAV->ACK at QGC) | 24 | 11 | 0.034 | 0.337 | 8.265 | 8.265 |


## Phase 4 - flight, UAV -> ground -> OCS/USV

Sessions: 15.

| Metric | Value |
|---|---|
| Status heartbeats to RoboCommand | 44005 |
| Heartbeat rate Hz (median of sessions) | 1.834 |
| Heartbeat gap p95 / max (s) | 0.918 / 236.198 |
| Heartbeat gaps > 2 s | 339 |
| Heartbeats suppressed as stale | 440 |
| Heartbeats with NaN fields | 250 |
| MQTT (re)connects | 41 |
| RunStart received from RoboCommand | 37 |
| RunStart relayed to UAV / acknowledged | 37 / 35 (94.6 %) |
| Restart requests / acknowledged | 18 / 18 |
| Geofence uploads / accepted / VALID / FAILED | 41 / 39 / 33 / 6 |
| RunDeclarations sent / blocked | 53 / 14 |
| SafePassage reports forwarded to RoboCommand + USV | 0 |
| Task commands forwarded to USV | 0 |
| OCS runs (restarts split at > 300 s silence) | 23 |
| OCS ERROR lines | 14 |
| OCS tracebacks: total / at shutdown (MQTT on_disconnect signature bug) | 43 / 39 |
| Broker client connects / disconnects / timeouts | 12 / 6 / 0 |

| Round trip | requests | matched | min s | median s | p95 s | max s |
|---|---|---|---|---|---|---|
| Restart (ground->UAV->ACK at QGC, 1 s resolution) | 18 | 17 | 8.0 | 17.1 | 43.2 | 43.2 |
| RunStart (RoboCommand->OCS->UAV->ACK at QGC) | 37 | 15 | 0.065 | 0.312 | 273.993 | 273.993 |


## Limitations

- No beacon messages were sent in flight; phases 2 and 4 measure the link and status/command traffic, not beacon delivery.
- The USV's own receipt is not logged here; measurements stop at the OCS output to RoboCommand MQTT / USV guidance topics.
- Over ELRS the UAV rate-limits PX4 telemetry on purpose, so PX4 sequence-gap "loss" mixes intentional drops with radio loss.
  Seabird component messages (191/192) are the better radio-loss indicator.
- Round-trip times compare OCS timestamps (WSL clock) with QGC tlog arrival times (Windows clock); they assume the clocks agree.
  Restart round trips have 1 s resolution (the request id is HHMMSS).
- tlogs exist only while QGC was running; tlogs larger than --max-tlog-mb were skipped: none


## Competition runs alongside the USV (log package `drone_log_package_20261117`)

Source: per-message router CSVs (UTC epoch arrival times), OCS log, ground.log, QGC tlogs. Airborne = GLOBAL_POSITION_INT relative altitude > 1.0 m.


USV status in `flight_2026-11-11_run2` OCS log (TeensyStatus samples): USV1 auto=2446, USV1 manual=9578, USV1 manual +kill=4417


### Phase 2 - UAV -> ground link (per-message router CSV)

| run | segment | from (UTC) | dur s | msgs/s | bad /100 good | PX4 seq loss % | Seabird loss % | HB Hz | HB gap max s | longest silence s |
|---|---|---|---|---|---|---|---|---|---|---|
| flight_2026-11-11_run2 | whole log | 00:48:16Z | 5266.4 | 2.99 | 0.51 | 87.78 | 0.47 | 0.992 | 2.448 | 2566.455 |
| flight_2026-11-11_run2 | on ground (outside airborne) | 00:48:16Z | 5171.8 | 2.83 | 0.48 | 87.77 | 9.22 | 0.923 | 95.449 |  |
| flight_2026-11-11_run2 | airborne 1 | 01:49:40Z | 94.6 | 11.74 | 0.9 | 88.26 | 0.0 | 0.996 | 1.96 | 0.473 |
| flight_2026-11-13_runs2-3-4 | whole log | 03:03:16Z | 1854.2 | 10.27 | 14.99 | 89.53 | 20.75 | 0.763 | 142.316 | 116.974 |
| flight_2026-11-13_runs2-3-4 | on ground (outside airborne) | 03:03:16Z | 1839.5 | 10.26 | 14.76 | 89.5 | 21.14 | 0.757 | 142.316 |  |
| flight_2026-11-13_runs2-3-4 | airborne 1 | 03:18:45Z | 0.6 | 6.7 | 125.0 | 96.08 |  |  |  | 0.322 |
| flight_2026-11-13_runs2-3-4 | airborne 2 | 03:19:06Z | 9.6 | 12.82 | 43.09 | 95.1 | 0.0 | 0.568 | 3.04 | 0.456 |
| flight_2026-11-13_runs2-3-4 | airborne 3 | 03:28:20Z | 0.0 | 1000000000.0 | 0.0 | 0.0 |  |  |  |  |
| flight_2026-11-13_runs2-3-4 | airborne 4 | 03:33:44Z | 4.5 | 9.34 | 26.19 | 91.51 | 0.0 | 1.006 | 1.389 | 0.486 |
| flight_2026-11-13_runs2-3-4 | airborne 5 | 03:34:08Z | 0.0 | 1000000000.0 | 0.0 | 0.0 |  |  |  |  |
| other_drone_sessions_20261113/20261113T022946Z | whole log | 02:30:00Z | 251.6 | 9.92 | 0.04 | 84.25 |  | 0.785 | 48.273 | 47.497 |
| other_drone_sessions_20261113/20261113T022946Z | on ground (outside airborne) | 02:30:00Z | 251.6 | 9.92 | 0.04 | 84.26 |  | 0.785 | 48.273 |  |
| other_drone_sessions_20261113/20261113T022946Z | airborne 1 | 02:32:05Z | 0.0 | 1000000000.0 | 0.0 | 0.0 |  |  |  |  |


Command path from ground.log (counts only, no timestamps):

| run | RunStart relayed | acked | never acked | max sends w/o ack | link gaps recovered | longest gap s | beacons |
|---|---|---|---|---|---|---|---|
| flight_2026-11-11_run2 | 2 | 1 | 5 | 1575 | 0 | 0 | 0 |
| flight_2026-11-13_runs2-3-4 | 4 | 2 | 1,4 | 7260 | 12 | 23283.0 | 0 |
| other_drone_sessions_20261113/20261113T022946Z | 0 | 0 |  | 0 | 0 | 0 | 0 |


### Phase 4 - OCS to RoboCommand, UAV and USV together

| run | vehicle | scope | heartbeats | Hz | gap p95 s | gap max s | gaps > 2 s | stale suppressed |
|---|---|---|---|---|---|---|---|---|
| flight_2026-11-11_run2 | UAV1 | whole log | 3326 | 1.998 | 0.501 | 2.001 | 1 | 3 |
| flight_2026-11-11_run2 | UAV1 | UAV airborne 1 | 190 | 2.0 | 0.501 | 0.506 | 0 |  |
| flight_2026-11-11_run2 | USV1 | whole log | 3289 | 1.976 | 0.501 | 20.499 | 1 | 40 |
| flight_2026-11-11_run2 | USV1 | UAV airborne 1 | 190 | 2.0 | 0.501 | 0.503 | 0 |  |


Command latency:

| run | path | id | sent (UTC) | targets | latency s |
|---|---|---|---|---|---|
| flight_2026-11-11_run2 | RoboCommand run_start -> forwarded to vehicles | 1 | 2026-11-11 01:49:31Z | USV1,UAV1 | 0.001 |
| flight_2026-11-11_run2 | RunStart OCS -> UAV -> ACK at ground | 2 | 2026-11-11 01:49:31Z | UAV1 | 0.623 |


Package caveats: no beacon messages were sent in these flights (ground counted 0); the SafePassage reports in the
OCS log came from the USV. No onboard .ulg, so latency is measured at the ground (round trip / forwarding only).
The 11-13 runs include the crash flight. Router CSV and OCS ran on the same laptop clock.
