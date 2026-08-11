#!/usr/bin/env python3

# HH_260810 - Added a bounded live contract probe for the normalized PC2 traffic-light camera pair.
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def stamp_ns(message: Image | CameraInfo) -> int:
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def header_rate(stamps: set[int]) -> float:
    ordered = sorted(stamps)
    if len(ordered) < 2 or ordered[-1] == ordered[0]:
        return 0.0
    return (len(ordered) - 1) * 1_000_000_000.0 / (ordered[-1] - ordered[0])


class CameraContractProbe(Node):
    def __init__(self, image_topic: str, info_topic: str) -> None:
        super().__init__("pc2_camera_contract_probe")
        self.image_stamps: set[int] = set()
        self.info_stamps: set[int] = set()
        self.image_shapes: set[tuple[int, int]] = set()
        self.info_shapes: set[tuple[int, int]] = set()
        self.image_frames: set[str] = set()
        self.info_frames: set[str] = set()
        self.image_encodings: set[str] = set()
        self.fx_values: set[float] = set()
        self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, info_topic, self.on_info, qos_profile_sensor_data)

    def on_image(self, message: Image) -> None:
        self.image_stamps.add(stamp_ns(message))
        self.image_shapes.add((message.width, message.height))
        self.image_frames.add(message.header.frame_id)
        self.image_encodings.add(message.encoding)

    def on_info(self, message: CameraInfo) -> None:
        self.info_stamps.add(stamp_ns(message))
        self.info_shapes.add((message.width, message.height))
        self.info_frames.add(message.header.frame_id)
        self.fx_values.add(round(message.k[0], 6))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--image-topic",
        default="/sensing/camera/camera1/traffic_light/image_raw",
    )
    parser.add_argument(
        "--info-topic",
        default="/sensing/camera/camera1/traffic_light/camera_info",
    )
    parser.add_argument("--samples", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1200)
    parser.add_argument("--encoding", default="bgr8")
    parser.add_argument(
        "--frame-id", default="traffic_light_camera/camera_link"
    )
    args = parser.parse_args()

    rclpy.init()
    node = CameraContractProbe(args.image_topic, args.info_topic)
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            if (
                len(node.image_stamps) >= args.samples
                and len(node.info_stamps) >= args.samples
            ):
                break
            rclpy.spin_once(node, timeout_sec=0.25)

        common_stamps = node.image_stamps & node.info_stamps
        expected_shape = {(args.width, args.height)}
        expected_frame = {args.frame_id}
        enough_samples = (
            len(node.image_stamps) >= args.samples
            and len(node.info_stamps) >= args.samples
        )
        metadata_match = (
            node.image_shapes == expected_shape
            and node.info_shapes == expected_shape
            and node.image_frames == expected_frame
            and node.info_frames == expected_frame
            and node.image_encodings == {args.encoding}
        )
        stamps_match = len(common_stamps) >= max(1, args.samples - 2)
        passed = enough_samples and metadata_match and stamps_match

        print(
            f"result={'PASS' if passed else 'FAIL'} "
            f"image_samples={len(node.image_stamps)} "
            f"info_samples={len(node.info_stamps)} "
            f"common_stamps={len(common_stamps)} "
            f"image_rate_hz={header_rate(node.image_stamps):.3f} "
            f"info_rate_hz={header_rate(node.info_stamps):.3f} "
            f"image_shapes={sorted(node.image_shapes)} "
            f"info_shapes={sorted(node.info_shapes)} "
            f"image_encodings={sorted(node.image_encodings)} "
            f"image_frames={sorted(node.image_frames)} "
            f"info_frames={sorted(node.info_frames)} "
            f"fx={sorted(node.fx_values)}"
        )
        return 0 if passed else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
