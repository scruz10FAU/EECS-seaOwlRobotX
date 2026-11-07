# RoboCommand pre-run checklist: coverage and plan (not implemented yet)

**Status:** deferred, saved 2026-11-07 for later. Nothing in this file is
implemented. It records how far the ground-station GUI
(`seabird_ground_gui.py`), `seaowls_ocs` and `beacon_mavlink_ground.py` cover
the Team Handbook 3.4.16 pre-run checklist, and how to close the gaps.

## Coverage today

| # | Checklist item | Covered? | How / gap |
|---|---|---|---|
| 1 | RoboCommand-facing OCS interface is configured for DHCP | ✅ Yes | Competition preflight checks DHCP on the selected adapter |
| 2 | Network bridging and Internet sharing are disabled | ✅ Yes | Preflight checks bridge binding, ICS and IP forwarding |
| 3 | The OCS received an address in the assigned course subnet | ✅ Yes | Preflight knows Alpha–Delta (`192.168.65–68`) and Validation (`192.168.69`). The Team Village demo subnet isn't published, so it's a warning there. |
| 4 | The default gateway matches the course RoboCommand address | ✅ Yes | Preflight checks it; the OCS uses the gateway as `ROBOCOMMAND_HOST` |
| 5 | The MQTT client is connected | ✅ Yes | OCS status row: "connected to RoboCommand broker", or red when disconnected |
| 6 | The OCS subscribed to `robocommand/robotx/course` | ⚠️ Partly | Subscribed on every connect, but the broker's acceptance (SUBACK) isn't confirmed or shown |
| 7 | The OCS subscribed to `robocommand/robotx/<team_id>/command` | ⚠️ Partly | Same as 6 |
| 8 | The retained RxCourse message was received and decoded | ⚠️ Partly | Visible only indirectly, through the Geofence row's course ID. No "Course" row; an `invalid RxCourse` decode error only appears in the OCS log tab. |
| 9 | The course identifier and boundary were checked | ⚠️ Partly | Boundary: yes (ground check, readback check, OCS re-check). Course ID: only used to match the readback to the current course; never checked itself. |
| 10 | The application is using the approved protobuf schema release | ✅ Yes | Preflight compares `vendor/robocommand` with `robonation/robocommand` HEAD. Needs Internet; on course networks it warns. |
| 11 | The assigned team_id is configured correctly | ⚠️ Partly | GUI field passed to the OCS; its format isn't checked |
| 12 | All participating vehicle IDs are unique and configured consistently | ⚠️ Partly | "Vehicles" is free text: no check for duplicates, unknown IDs, or that `UAV1` (the ground script's `--vehicle-id`) is listed |
| 13 | Independent report sequence counters are operating for every vehicle | ⚠️ Partly | The OCS keeps `self.seq[v]` per vehicle, but it's only visible in the OCS log (`UAV1 heartbeat seq=…`) |
| 14 | The UAV geofence is closed and within the received course boundary | ✅ Yes | Ground + readback + OCS checks; Geofence and Declaration rows |
| 15 | **The intended tier for every task is correct in RunDeclaration** | ❌ **No** | The OCS hard-codes `task1_tier=TIER_CORE` and Tasks 2–4 `TIER_NONE`; the GUI neither shows nor sets tiers |
| 16 | Operators can see the RoboCommand connection and command-response status | ⚠️ Partly | Connection and RunStart are shown. Other RxCommand bodies aren't, and a refused RunDeclaration is silent (RoboCommand sends nothing back), so "sent" isn't confirmation until RunStart. |
| 17 | Logging is enabled for the run | ✅ Yes | Every process logs to `logs/ground_gui/<timestamp>/`, never overwritten |

### ⚠ Before Validation: check the Task 1 tier (item 15)

The RoboCommand protobuf (`rx_reports.proto`, `SafePassageReport`) says
SafePassage is "sent by USV/UUV (Core) or UAV (Advanced/Disruptive)". Our UAV
sends SafePassage reports, but the OCS declares Task 1 as `TIER_CORE`.
Confirm the intended tier with whoever owns that decision. Until plan item 1
is done, it can only be changed in `ocs_node.py` (`maybe_declare()`,
`kwargs = dict(... task1_tier=common_pb2.TIER_CORE ...)`).

---

## Plan to close the gaps

Files:
- **GUI:** `elrs_windows_routing/seabird_ground_gui.py` (this repo).
- **OCS:** `seaowls-robotx-robocommand/ros2_ws/src/seaowls_robocommand/seaowls_robocommand/ocs_node.py`
  (Xavier's repo; leave changes uncommitted for his review). It's a
  `--symlink-install` build: editing a file needs no rebuild, adding one
  does.

### 1. Task tiers (item 15): do this first

- **OCS:** read `TASK1_TIER` … `TASK4_TIER` from the environment (`NONE`,
  `CORE`, `ADVANCED`, `DISRUPTIVE` → `common_pb2.TIER_*`, default today's
  values). Use them in `maybe_declare()`. Refuse to start on an unknown
  value, and never send `TIER_UNKNOWN` (Handbook 3.4.10).
- **OCS:** include the tiers in the `RunDeclaration seq=…` log line.
- **GUI:**
  - a "Tiers" box with four dropdowns, per profile;
  - pass the values as environment variables when starting the OCS;
  - show them on the Declaration row;
  - preflight warning if Task 1 is `CORE` while `UAV1` sends SafePassage.

### 2. Course received / decoded / checked (items 8, 9)

- **OCS:** after decoding RxCourse, log
  `RxCourse received: course=<id> corners=<n> transit=<n>`.
  - Checks on the course ID: not empty, printable, at most 64 characters.
    On failure, log `INVALID course id …` and don't publish the geofence.
  - Keep logging the decode error (`invalid RxCourse: …`) as an error.
- **GUI:** a new **Course** status row, set from those lines: green for
  received/decoded, red for an invalid ID or a decode error.

### 3. Subscription confirmation (items 6, 7)

- **OCS:** keep the MIDs returned by `c.subscribe(...)` in `connected()`, and
  add an `on_subscribe` callback (paho v2 signature
  `(client, userdata, mid, reason_code_list, properties)`). Log
  `subscribed <topic> OK` or `subscribe <topic> FAILED (<reason>)` per topic.
  Re-confirm after every reconnect.
- **GUI:** the OCS row shows "connected; subscribed: course ✓ command ✓",
  red if either failed.

### 4. team_id and vehicle IDs (items 11, 12): GUI only

- **Preflight checks:**
  - team ID is non-empty and uppercase letters/digits;
  - vehicle IDs are comma-separated, from {`USV1`, `UUV1`, `UAV1`}, with
    no duplicates;
  - `UAV1` is listed when the drone/ground script is used.
- Start all asks for confirmation on a failure, like the existing
  callsign/altitude prompt.

### 5. Sequence counters and command status (items 13, 16)

- **OCS:** already logs `<v> heartbeat seq=<n>`. Add `RxCommand` logging of
  every body type (it already logs `RxCommand seq=… body=…`).
- **GUI:**
  - a **Sequences** readout: last heartbeat/report seq per vehicle, red if
    one stops increasing for more than 2 s;
  - a **Last command** row: body, seq and time of the last RxCommand
    (RunStart, and Task 4 AssistanceRequest / KeepOutZone / AllClear).
  - On the Declaration row, note "sent — RoboCommand only answers with
    RunStart; no reply means refused or waiting for AUTO".

### Testing (same approach as the existing work)

- Run the OCS + ground script in one process against the fake PX4, with
  RoboNation's `rc_test/test_server.py` as the broker side. It confirms
  accepted declarations with `[geofence] … OK` and starts the run once every
  declared vehicle reports AUTO.
- Tiers: check the `run_declaration` the test server logs for each tier
  combination.
- GUI: drive `_parse()` with recorded OCS/Ground log lines, and run the
  preflight checks with good and bad team/vehicle IDs.

---

# Handbook 3.4.12 compliance plan (approved 2026-11-07, in progress)

Tasks and tiers attempted aren't decided yet, so every task and tier is built
and chosen per run in the GUI. Done in phases, reviewed after each.

| Phase | Work | Status |
|---|---|---|
| 1 | Per-task tier selector (GUI → OCS `TASK1_TIER`…`TASK4_TIER`, never `TIER_UNKNOWN`). RunStart gate: OCS publishes the verified RunStart, which goes ground → ELRS → drone, and the sweep holds in OFFBOARD until it arrives (switchable for bench/sim). GUI "Commands" row; report sequence numbers in logs. | **done** (2026-11-07): tiers, RunStart gate via the drone ELRS bridge's onboard copy (option A), Commands/Reports rows, report seqs in logs. Needs the drone ELRS bridge redeployed. |
| 2 | Task 4, all tiers. New `seaowls_interfaces` messages. Core: AssistanceRequest → IncidentAck → ReadinessReport → ReadinessConfirm. Advanced: KeepOutZone → IncidentAck → AllClear → IncidentAck. Disruptive: forward MovingObjectAlert. Vehicle behaviour stays with the vehicle autonomy code. | **done** (2026-11-07): Task4Command message, OCS routing + IncidentAck (auto or vehicle) + ReadinessReport/Confirm, GUI Task 4 row + Send ReadinessReport + Rehearse Task 4, tests/mock_robocommand_task4.py. |
| 3 | Task 2 PipelineSurveyReport (UUV1); the UAV's ResourceDeliveryRequest echo (Tasks 2/3, Advanced/Disruptive). | **done** (2026-11-07): PipelineSurvey message + OCS PipelineSurveyReport as UUV1; UAV ResourceDelivery echo topic + GUI 'Echo delivery request (UAV)'. |
| 4 | SafePassage rate limit (~1/s, 3.4.5). README: sync the laptop clock before the course; never restart the OCS mid-run. | **done** (2026-11-07): SafePassage merged to <= 1/s (latest field always sent); GUI preflight WSL/Windows clock checks; GUI confirms before stopping the OCS after RunStart; README procedure notes. |

Gaps this closes (audit of 3.4.12 + 3.4.8/3.4.10/3.4.13): no RunStart gate on
the vehicle; Task 4 chain absent; command receipt/response not shown;
PipelineSurveyReport and the UAV ResourceDelivery echo absent; no SafePassage
rate limit; sequence counters reset if the OCS restarts mid-run; OCS clock
can't sync on course networks.
