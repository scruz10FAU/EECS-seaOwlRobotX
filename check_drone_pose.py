#!/usr/bin/env python3
"""
check_drone_pose.py — Minimal, standalone check for whether the drone_pose
topic is actually reachable from this process, isolated from the full
beacon detector pipeline (no beacon_camera.py, no YOLO, nothing else).

IMPORTANT: run this from the exact same environment (container/shell) that
beacon_detector_config.py actually runs in. Running it somewhere else (e.g.
directly on the VOXL2 host when the real detector runs inside the Docker
container) won't tell you anything useful -- what matters is whether THIS
topic is reachable from THAT process's point of view.

Usage:
    python3 check_drone_pose.py [topic_name]
    (default topic: /vvhub_body_wrt_local/pose)
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped

TOPIC = sys.argv[1] if len(sys.argv) > 1 else "/vvhub_body_wrt_local/pose"
TIMEOUT_SEC = 10.0


class PoseChecker(Node):
    def __init__(self):
        super().__init__("pose_checker")
        self.got_message = False

        # Step 1: who's actually publishing this topic, from THIS process's
        # point of view? This is the same check that caught the missing
        # px4_msgs GPS topic earlier -- a topic can "exist" on one machine/
        # container and be completely invisible from another.
        pubs = self.get_publishers_info_by_topic(TOPIC)
        if not pubs:
            print(f"[check] No publishers visible for {TOPIC} from this process.")
            print("[check]   -> either the topic doesn't exist here, or this "
                  "process's DDS domain/discovery can't see it at all.")
        else:
            print(f"[check] {len(pubs)} publisher(s) visible for {TOPIC}:")
            for p in pubs:
                print(f"[check]   node={p.node_name}  "
                      f"reliability={p.qos_profile.reliability}  "
                      f"durability={p.qos_profile.durability}")

        # Step 2: subscribe with the SAME QoS beacon_camera.py uses, and see
        # if a message actually arrives.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(PoseStamped, TOPIC, self._on_pose, qos)
        print(f"[check] Subscribed to {TOPIC} (BEST_EFFORT/KEEP_LAST/depth=1) "
              f"-- waiting up to {TIMEOUT_SEC:.0f}s for a message...")

    def _on_pose(self, msg):
        self.got_message = True
        p = msg.pose.position
        print(f"[check] GOT A MESSAGE: x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}")


def main():
    rclpy.init()
    node = PoseChecker()
    start = time.time()
    try:
        while time.time() - start < TIMEOUT_SEC and not node.got_message:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass

    print()
    if node.got_message:
        print("[check] RESULT: topic is reachable and publishing -- "
              "get_drone_pose() should work in the real pipeline too.")
    else:
        print(f"[check] RESULT: NO message received in {TIMEOUT_SEC:.0f}s -- "
              "this confirms get_drone_pose() will always return (None, None) "
              "in this environment.")
        print("[check]   Next step depends on what was printed above:")
        print("[check]     - No publishers listed at all -> this process can't "
              "see the topic (DDS domain/discovery/network issue).")
        print("[check]     - Publishers ARE listed but still no message -> "
              "suspect a QoS mismatch (compare reliability/durability above "
              "against what this subscriber uses), or VVHub isn't actually "
              "publishing real data yet (VIO not initialized/locked).")

    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
