#!/usr/bin/env python3

# HH_260812 - Validate the uncalibrated Windshield camera-to-2D-YOLOX contract without requiring fusion.
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tier4_perception_msgs.msg import DetectedObjectsWithFeature


def stamp_ns(message: DetectedObjectsWithFeature) -> int:
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def header_rate(stamps: set[int]) -> float:
    ordered = sorted(stamps)
    if len(ordered) < 2 or ordered[-1] == ordered[0]:
        return 0.0
    return (len(ordered) - 1) * 1_000_000_000.0 / (ordered[-1] - ordered[0])


class YoloXContractProbe(Node):
    def __init__(
        self,
        topic: str,
        input_topic: str,
        width: int,
        height: int,
        yolox_node: str,
        input_publisher_node: str,
        required_output_subscribers: set[str],
    ) -> None:
        super().__init__("pc2_yolox_contract_probe")
        self.topic = topic
        self.input_topic = input_topic
        self.width = width
        self.height = height
        self.yolox_node = yolox_node
        self.input_publisher_node = input_publisher_node
        self.required_output_subscribers = required_output_subscribers
        self.stamps: set[int] = set()
        self.frames: set[str] = set()
        self.object_counts: list[int] = []
        self.invalid_rois = 0
        self.create_subscription(
            DetectedObjectsWithFeature,
            topic,
            self.on_objects,
            QoSProfile(depth=1),
        )

    @staticmethod
    def endpoint_name(endpoint: object) -> str:
        namespace = str(endpoint.node_namespace).rstrip("/")
        return f"{namespace}/{endpoint.node_name}" if namespace else f"/{endpoint.node_name}"

    def graph_contract(self) -> tuple[bool, list[str], list[str], list[str], list[str]]:
        output_publishers = sorted(
            self.endpoint_name(endpoint)
            for endpoint in self.get_publishers_info_by_topic(self.topic)
        )
        output_subscribers = sorted(
            self.endpoint_name(endpoint)
            for endpoint in self.get_subscriptions_info_by_topic(self.topic)
        )
        input_publishers = sorted(
            self.endpoint_name(endpoint)
            for endpoint in self.get_publishers_info_by_topic(self.input_topic)
        )
        input_subscribers = sorted(
            self.endpoint_name(endpoint)
            for endpoint in self.get_subscriptions_info_by_topic(self.input_topic)
        )
        passed = (
            output_publishers == [self.yolox_node]
            and self.required_output_subscribers.issubset(output_subscribers)
            and input_publishers == [self.input_publisher_node]
            and self.yolox_node in input_subscribers
        )
        return (
            passed,
            output_publishers,
            output_subscribers,
            input_publishers,
            input_subscribers,
        )

    def on_objects(self, message: DetectedObjectsWithFeature) -> None:
        self.stamps.add(stamp_ns(message))
        self.frames.add(message.header.frame_id)
        self.object_counts.append(len(message.feature_objects))
        for feature_object in message.feature_objects:
            roi = feature_object.feature.roi
            if (
                roi.width <= 0
                or roi.height <= 0
                or roi.x_offset + roi.width > self.width
                or roi.y_offset + roi.height > self.height
            ):
                self.invalid_rois += 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--topic", default="/perception/object_recognition/detection/rois0"
    )
    parser.add_argument(
        "--input-topic",
        default="/sensing/camera/camera0/image_raw",
    )
    parser.add_argument("--samples", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--min-rate", type=float, default=5.0)
    parser.add_argument("--width", type=int, default=2880)
    parser.add_argument("--height", type=int, default=1860)
    parser.add_argument(
        "--frame-id", default="camera0/camera_optical_link"
    )
    parser.add_argument(
        "--yolox-node",
        default="/tensorrt_yolox",
        help="Expected node that subscribes to the image and publishes the ROI topic.",
    )
    parser.add_argument(
        "--input-publisher-node",
        default="/sensing/camera/camera0/windshield_image_relay",
        help="Sole expected publisher node for the normalized Windshield image.",
    )
    parser.add_argument(
        "--required-output-subscriber",
        action="append",
        default=None,
        help="Required permanent ROI subscriber node; may be repeated.",
    )
    args = parser.parse_args()

    required_output_subscribers = set(
        args.required_output_subscriber
        or ["/topic_state_monitor_windshield_yolox_rois"]
    )

    rclpy.init()
    node = YoloXContractProbe(
        args.topic,
        args.input_topic,
        args.width,
        args.height,
        args.yolox_node,
        args.input_publisher_node,
        required_output_subscribers,
    )
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline and len(node.stamps) < args.samples:
            rclpy.spin_once(node, timeout_sec=0.25)

        rate = header_rate(node.stamps)
        (
            graph_passed,
            output_publishers,
            output_subscribers,
            input_publishers,
            input_subscribers,
        ) = node.graph_contract()
        passed = (
            len(node.stamps) >= args.samples
            and node.frames == {args.frame_id}
            and rate >= args.min_rate
            and node.invalid_rois == 0
            and graph_passed
        )
        total_objects = sum(node.object_counts)
        min_objects = min(node.object_counts, default=0)
        max_objects = max(node.object_counts, default=0)
        print(
            f"result={'PASS' if passed else 'FAIL'} "
            f"samples={len(node.stamps)} "
            f"rate_hz={rate:.3f} "
            f"frames={sorted(node.frames)} "
            f"total_objects={total_objects} "
            f"objects_per_message={min_objects}..{max_objects} "
            f"invalid_rois={node.invalid_rois} "
            f"graph={'PASS' if graph_passed else 'FAIL'} "
            f"required_output_subscribers={sorted(required_output_subscribers)} "
            f"output_publishers={output_publishers} "
            f"output_subscribers={output_subscribers} "
            f"input_publishers={input_publishers} "
            f"input_subscribers={input_subscribers}"
        )
        return 0 if passed else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
