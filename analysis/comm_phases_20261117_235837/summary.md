# Communication metrics, phases 2-4

Generated 2026-11-17 23:59 from 59 ground-GUI sessions and 14 tlogs with flights.

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
| 5 s intervals with no frames (after first contact) | 1633 |
| Frames via ELRS / WiFi | 29598 / 55970 |
| Cross-link duplicates removed | 7783 |
| Uplink switches ELRS<->WiFi | 15 |


Ground bridge link gaps (flight sessions): 21 recovered, longest 218.0 s.


## Phase 3 - bench, UAV -> ground -> OCS/USV

Sessions: 26.

| Metric | Value |
|---|---|
| Status heartbeats to RoboCommand | 32963 |
| Heartbeat rate Hz (median of sessions) | 1.759 |
| Heartbeat gap p95 / max (s) | 1.56 / 7205.81 |
| Heartbeat gaps > 2 s | 354 |
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
| OCS errors / crashes (tracebacks) | 11 / 41 |
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
| Heartbeat rate Hz (median of sessions) | 1.735 |
| Heartbeat gap p95 / max (s) | 0.918 / 44858.474 |
| Heartbeat gaps > 2 s | 347 |
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
| OCS errors / crashes (tracebacks) | 14 / 43 |
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
