#!/usr/bin/env python3

# HH_260811 - Added a bounded, read-only graph probe for the PC1 SocketCAN bridge contract.
import argparse
import json
import time

import rclpy
from rclpy.node import Node


class BridgeGraphProbe(Node):
    def __init__(self) -> None:
        super().__init__("can_bridge_graph_probe")

    @staticmethod
    def describe(endpoint: object) -> dict[str, object]:
        qos = endpoint.qos_profile
        return {
            "node": f"{endpoint.node_namespace.rstrip('/')}/{endpoint.node_name}",
            "reliability": str(qos.reliability),
            "durability": str(qos.durability),
        }

    def endpoints(self, topic: str) -> dict[str, object]:
        return {
            "publishers": [
                self.describe(endpoint) for endpoint in self.get_publishers_info_by_topic(topic)
            ],
            "subscriptions": [
                self.describe(endpoint)
                for endpoint in self.get_subscriptions_info_by_topic(topic)
            ],
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()
    node = BridgeGraphProbe()
    started = time.monotonic()
    try:
        deadline = started + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.25)

        nodes = sorted(f"{namespace.rstrip('/')}/{name}" for name, namespace in node.get_node_names_and_namespaces())
        topics = {
            name: types
            for name, types in node.get_topic_names_and_types()
            if "can" in name.lower() or "bridge" in name.lower()
        }
        candidate_topics = sorted(
            set(topics)
            | {
                "/from_can_bus",
                "/to_can_bus",
                "/pc1/can/from_can_bus",
                "/pc1/can/to_can_bus",
            }
        )
        result = {
            "duration_s": round(time.monotonic() - started, 3),
            "nodes": [name for name in nodes if "can" in name.lower() or "bridge" in name.lower()],
            "domain_node_count": len(nodes),
            "topics": dict(sorted(topics.items())),
            "endpoints": {topic: node.endpoints(topic) for topic in candidate_topics},
        }
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
