#!/usr/bin/env python3
"""
mock_robocommand_task4.py — plays RoboCommand's side of Task 4 (Dynamic
Incident Response) over MQTT, so seaowls_ocs's command-response chains can be
tested and rehearsed without the competition server (whose rc_test server
doesn't issue Task 4 commands).

It publishes real RxCommand protobufs on robocommand/robotx/<team_id>/command
and watches robocommand/robotx/<team_id>/+/report, checking each response
against Team Handbook 3.4.12:

  core:       AssistanceRequest -> IncidentAck -> ReadinessReport -> ReadinessConfirm
  advanced:   KeepOutZone -> IncidentAck -> AllClear -> IncidentAck
  disruptive: MovingObjectAlert (no acknowledgement expected)

For core, it waits for the vehicle's ReadinessReport (the vehicle -- or the
ground GUI's "Send ReadinessReport" button -- publishes std_msgs/UInt32 on
/seaowls/<vehicle>/task4/ready), then answers ReadinessConfirm with that
report's seq and vehicle_id, as RoboCommand does.

Requires: paho-mqtt and the RoboCommand protobufs (ROBOCOMMAND_ROOT, as for
mock_robocommand_course.py). Exit code 0 = every expected response arrived.

Usage (WSL, broker on localhost:1883, seaowls_ocs running):
    python3 tests/mock_robocommand_task4.py --scenario all
    python3 tests/mock_robocommand_task4.py --scenario core --vehicle-type USV --lat 26.3712 --lon -80.1034
    python3 tests/mock_robocommand_task4.py --scenario advanced --vehicle-type UAV --radius 15
"""

import argparse
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mock_robocommand_course import _import_schemas   # noqa: E402  (sets up the protobuf path)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default=os.getenv("ROBOCOMMAND_HOST", "localhost"))
    ap.add_argument("--port", type=int, default=int(os.getenv("ROBOCOMMAND_PORT", "1883")))
    ap.add_argument("--team-id", default=os.getenv("TEAM_ID", "FLAU"))
    ap.add_argument("--scenario", choices=["core", "advanced", "disruptive", "all"], default="all")
    ap.add_argument("--vehicle-type", choices=["USV", "UUV", "UAV"], default="USV",
                    help="Vehicle type the commands address (default: USV)")
    ap.add_argument("--lat", type=float, default=26.3712, help="Incident / zone / object latitude")
    ap.add_argument("--lon", type=float, default=-80.1034, help="Incident / zone / object longitude")
    ap.add_argument("--radius", type=float, default=10.0, help="Keep-out radius, m (default: 10)")
    ap.add_argument("--timeout", type=float, default=60.0,
                    help="Seconds to wait for each expected response (default: 60)")
    args = ap.parse_args()

    common_pb2, _, Timestamp = _import_schemas()
    from robotx import rx_commands_pb2, rx_reports_pb2, rx_common_pb2
    import paho.mqtt.client as mqtt

    vtype = getattr(rx_common_pb2, f"TYPE_{args.vehicle_type}")
    root = f"robocommand/robotx/{args.team_id}"
    reports, cond = [], threading.Condition()
    seq = [int(time.time()) % 100000 * 10]   # well clear of RunStart's seq

    def on_message(client, userdata, msg):
        r = rx_reports_pb2.RxReport()
        try:
            r.ParseFromString(msg.payload)
        except Exception:
            return
        body = r.WhichOneof("body")
        if body in ("incident_ack", "readiness"):
            with cond:
                reports.append(r)
                cond.notify_all()

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_message = on_message
    client.connect(args.host, args.port)
    client.subscribe(f"{root}/+/report")
    client.loop_start()
    time.sleep(1.0)

    def send(**body):
        seq[0] += 1
        ts = Timestamp(); ts.GetCurrentTime()
        cmd = rx_commands_pb2.RxCommand(team_id=args.team_id, seq=seq[0], sent_at=ts, **body)
        client.publish(f"{root}/command", cmd.SerializeToString()).wait_for_publish()
        print(f"RC -> {cmd.WhichOneof('body')} seq={seq[0]}", flush=True)
        return seq[0]

    def expect(kind, command_seq):
        """Wait for an incident_ack / readiness report for command_seq; returns the RxReport."""
        end = time.time() + args.timeout
        with cond:
            while time.time() < end:
                for r in reports:
                    body = getattr(r, r.WhichOneof("body"))
                    if r.WhichOneof("body") == kind and body.command_seq == command_seq:
                        reports.remove(r)
                        print(f"RC <- {kind} from {r.vehicle_id} command_seq={command_seq} "
                              f"(report seq={r.seq})  OK", flush=True)
                        return r
                cond.wait(timeout=0.5)
        print(f"RC !! no {kind} for command_seq={command_seq} within {args.timeout:.0f} s  FAIL", flush=True)
        return None

    pos = common_pb2.LatLng(latitude=args.lat, longitude=args.lon)
    ok = True
    if args.scenario in ("core", "all"):
        print("\n== Task 4 Core", flush=True)
        s = send(assistance_request=rx_commands_pb2.AssistanceRequest(position=pos, vehicle_type=vtype))
        ok &= expect("incident_ack", s) is not None
        print("   (waiting for the vehicle to reach the position and report ready)", flush=True)
        ready = expect("readiness", s)
        ok &= ready is not None
        if ready is not None:
            send(readiness_confirm=rx_commands_pb2.ReadinessConfirm(
                report_seq=ready.seq, vehicle_id=ready.vehicle_id))
    if args.scenario in ("advanced", "all"):
        print("\n== Task 4 Advanced", flush=True)
        s = send(keep_out_zone=rx_commands_pb2.KeepOutZone(center=pos, radius_m=args.radius,
                                                            vehicle_type=vtype))
        ok &= expect("incident_ack", s) is not None
        s = send(all_clear=rx_commands_pb2.AllClear(vehicle_type=vtype))
        ok &= expect("incident_ack", s) is not None
    if args.scenario in ("disruptive", "all"):
        print("\n== Task 4 Disruptive", flush=True)
        send(moving_object_alert=rx_commands_pb2.MovingObjectAlert(
            position=pos, heading_deg=90.0, speed_mps=1.5, affected_vehicle_types=[vtype],
            keep_out_radius_m=5.0))
        print("   (no acknowledgement in this chain -- check the OCS log routed it)", flush=True)
    time.sleep(1.0)
    client.loop_stop()
    client.disconnect()
    print("\nRESULT:", "PASS -- every expected response arrived" if ok else "FAIL", flush=True)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
