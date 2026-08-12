#!/usr/bin/env python3

# HH_260811 - Added a bounded, subscribe-only diagnostic probe for outdoor sensor/localization readiness.
import argparse
import json
import re
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


DIAGNOSTIC_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
RELEVANT = re.compile(
    r"gnss|novatel|imu|locali[sz]|ndt|pose|ekf|gyro|lidar|hesai", re.IGNORECASE
)


class DiagnosticProbe(Node):
    def __init__(self) -> None:
        super().__init__("outdoor_diagnostic_probe")
        self.arrays = 0
        self.latest: dict[str, dict[str, object]] = {}
        self.create_subscription(DiagnosticArray, "/diagnostics", self.on_message, DIAGNOSTIC_QOS)

    def on_message(self, message: DiagnosticArray) -> None:
        self.arrays += 1
        for status in message.status:
            key = f"{status.name}|{status.hardware_id}"
            # HH_260811 - Normalized Humble's one-byte Python representation before filtering diagnostics.
            level = status.level[0] if isinstance(status.level, bytes) else int(status.level)
            if level == 0 and not RELEVANT.search(key):
                continue
            self.latest[key] = {
                "level": level,
                "message": status.message,
                "values": {item.key: item.value for item in status.values[:40]},
            }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()
    node = DiagnosticProbe()
    started = time.monotonic()
    try:
        deadline = started + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.25)
        print(
            json.dumps(
                {
                    "duration_s": round(time.monotonic() - started, 3),
                    "diagnostic_arrays": node.arrays,
                    "statuses": dict(sorted(node.latest.items())),
                },
                indent=2,
                sort_keys=True,
            )
        )
        return 0 if node.arrays else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
