#!/usr/bin/env python3
"""
check_drone_pose.py — Minimal, standalone check for whether the drone's AGL
height source is actually reachable from this process, isolated from the
full beacon detector pipeline (no beacon_camera.py, no YOLO, nothing else).

Mirrors get_drone_height_agl()'s actual logic exactly (same topic default,
same message type, same z-negation, same z_valid gate) for whichever source
you select, so a "GOT A MESSAGE" here means get_drone_height_agl() will
really produce a value once the real pipeline runs -- not just that some
topic exists.

IMPORTANT: run this from the exact same environment (container/shell) that
beacon_detector_config.py actually runs in. Running it somewhere else (e.g.
directly on the VOXL2 host when the real detector runs inside the Docker
container) won't tell you anything useful -- what matters is whether THIS
topic is reachable from THAT process's point of view.

Usage:
    python3 check_drone_pose.py                          # px4 source (default)
    python3 check_drone_pose.py --source px4 [topic]      # /fmu/out/vehicle_local_position
    python3 check_drone_pose.py --source vvhub [topic]     # /vvhub_body_wrt_local/pose
"""
import sys
import time
import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

TIMEOUT_SEC = 10.0


class Px4HeightChecker(Node):
    """Mirrors _on_local_position_px4() / get_drone_height_agl() exactly."""

    def __init__(self, topic):
        super().__init__("px4_height_checker")
        self.topic = topic
        self.got_message = False
        self.height_agl = None

        from px4_msgs.msg import VehicleLocalPosition

        _print_publishers(self, topic)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(VehicleLocalPosition, topic, self._on_local_position, qos)
        print(f"[check] Subscribed to {topic} (BEST_EFFORT/KEEP_LAST/depth=1) "
              f"-- waiting up to {TIMEOUT_SEC:.0f}s for a message...")

    def _on_local_position(self, msg):
        # Exact same logic as beacon_detector_config.py's _on_local_position_px4:
        # z is NED (down-positive), so height AGL is -z; z_valid gates against
        # using an estimate that hasn't converged.
        if not getattr(msg, "z_valid", True):
            print(f"[check] Message received but z_valid=False (not converged "
                  f"yet) -- get_drone_height_agl() would still return None "
                  f"for this reading, waiting for a valid one...")
            return
        self.got_message = True
        self.height_agl = -msg.z
        print(f"[check] GOT A VALID MESSAGE: z={msg.z:.3f} (NED down) "
              f"-> height_agl={self.height_agl:.3f} m")


class VvhubPoseChecker(Node):
    """Mirrors _on_drone_pose() / get_drone_pose()[2] exactly."""

    def __init__(self, topic):
        super().__init__("vvhub_pose_checker")
        self.topic = topic
        self.got_message = False
        self.height_agl = None

        from geometry_msgs.msg import PoseStamped

        _print_publishers(self, topic)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(PoseStamped, topic, self._on_pose, qos)
        print(f"[check] Subscribed to {topic} (BEST_EFFORT/KEEP_LAST/depth=1) "
              f"-- waiting up to {TIMEOUT_SEC:.0f}s for a message...")

    def _on_pose(self, msg):
        self.got_message = True
        p = msg.pose.position
        self.height_agl = p.z
        print(f"[check] GOT A MESSAGE: x={p.x:.3f} y={p.y:.3f} z={p.z:.3f} "
              f"-> height_agl={self.height_agl:.3f} m")


def _print_publishers(node, topic):
    # Same check that caught the missing px4_msgs GPS topic earlier -- a
    # topic can "exist" on one machine/container and be completely invisible
    # from another.
    pubs = node.get_publishers_info_by_topic(topic)
    if not pubs:
        print(f"[check] No publishers visible for {topic} from this process.")
        print("[check]   -> either the topic doesn't exist here, or this "
              "process's DDS domain/discovery can't see it at all.")
    else:
        print(f"[check] {len(pubs)} publisher(s) visible for {topic}:")
        for p in pubs:
            print(f"[check]   node={p.node_name}  "
                  f"reliability={p.qos_profile.reliability}  "
                  f"durability={p.qos_profile.durability}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("topic", nargs="?", default=None,
                        help="Topic name override (default depends on --source)")
    parser.add_argument("--source", choices=["px4", "vvhub"], default="px4",
                        help="Which height source to check (default: px4, "
                             "matching topics.drone_height_source=px4_local_position)")
    args = parser.parse_args()

    rclpy.init()

    if args.source == "px4":
        topic = args.topic or "/fmu/out/vehicle_local_position"
        try:
            node = Px4HeightChecker(topic)
        except ImportError:
            print("[check] RESULT: 'px4_msgs' is not installed in this "
                  "environment -- get_drone_height_agl() will fall back to "
                  "vvhub_pose and print a warning at startup, same as the "
                  "real pipeline. Build px4_msgs here first, or re-run this "
                  "check with --source vvhub.")
            try: rclpy.shutdown()
            except Exception: pass
            return
    else:
        topic = args.topic or "/vvhub_body_wrt_local/pose"
        node = VvhubPoseChecker(topic)

    start = time.time()
    try:
        while time.time() - start < TIMEOUT_SEC and not node.got_message:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass

    print()
    if node.got_message:
        print(f"[check] RESULT: height source reachable -- "
              f"get_drone_height_agl() will return ~{node.height_agl:.3f} m "
              f"in the real pipeline too.")
    else:
        print(f"[check] RESULT: NO valid message received in {TIMEOUT_SEC:.0f}s "
              f"-- this confirms get_drone_height_agl() will return None in "
              f"this environment with source={args.source}.")
        print("[check]   Next step depends on what was printed above:")
        print("[check]     - No publishers listed at all -> this process can't "
              "see the topic (DDS domain/discovery/network issue).")
        print("[check]     - Publishers ARE listed but still no valid message "
              "-> suspect a QoS mismatch, or (px4 source) the estimate simply "
              "hasn't converged yet (z_valid=False).")

    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
