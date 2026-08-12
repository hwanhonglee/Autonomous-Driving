#!/usr/bin/env python3

# HH_260810 - Added a bounded, subscribe-only probe for PC3 inputs and PC2 camera/perception outputs.
import argparse
import json
import time
from collections import defaultdict

import rclpy
from autoware_perception_msgs.msg import PredictedObjects
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, PointCloud2
from tf2_msgs.msg import TFMessage


SENSOR_QOS = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
RELIABLE_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
STATIC_TF_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class FlowProbe(Node):
    def __init__(self) -> None:
        super().__init__("pc2_pc3_flow_probe")
        self.started = time.monotonic()
        self.counts: dict[str, int] = defaultdict(int)
        self.frames: dict[str, set[str]] = defaultdict(set)
        self.first_stamp_ns: dict[str, int] = {}
        self.last_stamp_ns: dict[str, int] = {}
        self.object_lengths: list[int] = []
        self.camera_shapes: set[tuple[int, int]] = set()
        self.camera_fx: set[float] = set()

        self.create_subscription(
            PointCloud2,
            "/sensing/lidar/top/pointcloud_before_sync",
            lambda message: self.on_header_message("before_sync", message),
            SENSOR_QOS,
        )
        self.create_subscription(
            PointCloud2,
            "/sensing/lidar/top/pointcloud_raw_ex",
            lambda message: self.on_header_message("raw_ex", message),
            SENSOR_QOS,
        )
        self.create_subscription(
            CompressedImage,
            "/lucid_vision/camera/image_compressed",
            lambda message: self.on_header_message("camera_compressed", message),
            SENSOR_QOS,
        )
        self.create_subscription(
            CameraInfo,
            "/sensing/camera/camera1/traffic_light/camera_info",
            self.on_camera_info,
            SENSOR_QOS,
        )
        self.create_subscription(
            PredictedObjects,
            "/perception/object_recognition/objects",
            self.on_objects,
            RELIABLE_QOS,
        )
        self.create_subscription(
            TFMessage, "/tf", lambda message: self.on_tf("tf", message), RELIABLE_QOS
        )
        self.create_subscription(
            TFMessage,
            "/tf_static",
            lambda message: self.on_tf("tf_static", message),
            STATIC_TF_QOS,
        )

    @staticmethod
    def stamp_ns(message: object) -> int:
        stamp = message.header.stamp
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def on_header_message(self, name: str, message: object) -> None:
        self.counts[name] += 1
        self.frames[name].add(message.header.frame_id)
        stamp = self.stamp_ns(message)
        self.first_stamp_ns.setdefault(name, stamp)
        self.last_stamp_ns[name] = stamp

    def on_camera_info(self, message: CameraInfo) -> None:
        self.on_header_message("camera_info", message)
        self.camera_shapes.add((message.width, message.height))
        self.camera_fx.add(round(message.k[0], 6))

    def on_objects(self, message: PredictedObjects) -> None:
        self.on_header_message("objects", message)
        self.object_lengths.append(len(message.objects))

    def on_tf(self, name: str, message: TFMessage) -> None:
        self.counts[name] += 1
        for transform in message.transforms:
            self.frames[name].add(
                f"{transform.header.frame_id}->{transform.child_frame_id}"
            )

    def rate_hz(self, name: str) -> float:
        count = self.counts[name]
        first = self.first_stamp_ns.get(name, 0)
        last = self.last_stamp_ns.get(name, 0)
        if count < 2 or last <= first:
            return 0.0
        return (count - 1) * 1_000_000_000.0 / (last - first)

    # HH_260811 - Summarized selected endpoints from the same persistent participant used for flow counts.
    def endpoint_summary(self, topic: str) -> dict[str, object]:
        def describe(endpoint: object) -> dict[str, object]:
            qos = endpoint.qos_profile
            return {
                "node": f"{endpoint.node_namespace.rstrip('/')}/{endpoint.node_name}",
                "reliability": str(qos.reliability),
                "durability": str(qos.durability),
            }

        publishers = self.get_publishers_info_by_topic(topic)
        subscriptions = self.get_subscriptions_info_by_topic(topic)
        return {
            "publishers": [describe(endpoint) for endpoint in publishers],
            "subscriptions": [describe(endpoint) for endpoint in subscriptions],
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init()
    node = FlowProbe()
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.25)

        result = {
            "duration_s": round(time.monotonic() - node.started, 3),
            "counts": dict(sorted(node.counts.items())),
            "rates_hz": {
                name: round(node.rate_hz(name), 3)
                for name in (
                    "before_sync",
                    "raw_ex",
                    "camera_compressed",
                    "camera_info",
                    "objects",
                )
            },
            "frames": {
                name: sorted(values) for name, values in sorted(node.frames.items())
            },
            "camera_shapes": sorted(node.camera_shapes),
            "camera_fx": sorted(node.camera_fx),
            "object_count_min_max": (
                [min(node.object_lengths), max(node.object_lengths)]
                if node.object_lengths
                else []
            ),
            "endpoints": {
                topic: node.endpoint_summary(topic)
                for topic in (
                    "/sensing/lidar/top/pointcloud_before_sync",
                    "/sensing/lidar/top/pointcloud_raw_ex",
                    "/map/vector_map",
                    "/tf",
                    "/tf_static",
                    "/localization/kinematic_state",
                    "/perception/object_recognition/objects",
                    "/perception/traffic_light_recognition/traffic_signals",
                    "/perception/obstacle_segmentation/pointcloud",
                    "/planning/mission_planning/route",
                    "/planning/scenario_planning/trajectory",
                    "/control/command/control_cmd",
                    "/to_can_bus",
                    "/pacmod/to_can_bus",
                )
            },
        }
        print(json.dumps(result, indent=2, sort_keys=True))

        # HH_260810 - Required live PC3 pointclouds plus the live PC2 camera contract; objects may validly be empty.
        passed = all(
            node.counts[name] > 0
            for name in ("before_sync", "raw_ex", "camera_compressed", "camera_info")
        )
        return 0 if passed else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
