#!/usr/bin/env python3

import argparse
import json
import math
from pathlib import Path

import cv2
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ImageSequenceRecorder(Node):
    def __init__(self, args):
        super().__init__("autoware_e2e_image_sequence_recorder")
        self.args = args
        self.bridge = CvBridge()
        self.speed_mps = 0.0
        self.motion_started = not args.wait_for_motion
        self.first_stamp_ns = None
        self.last_stamp_ns = None
        self.frame_count = 0
        self.error = None
        self.output_dir = args.output_dir.expanduser().resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.create_subscription(
            Odometry, args.odometry_topic, self._on_odometry, 10
        )
        self.create_subscription(
            Image, args.topic, self._on_image, qos_profile_sensor_data
        )

    def _on_odometry(self, message):
        velocity = message.twist.twist.linear
        self.speed_mps = math.sqrt(
            velocity.x * velocity.x
            + velocity.y * velocity.y
            + velocity.z * velocity.z
        )
        if self.speed_mps >= self.args.motion_threshold:
            self.motion_started = True

    def _on_image(self, message):
        if not self.motion_started or self.frame_count >= self.args.max_frames:
            return
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if self.last_stamp_ns == stamp_ns:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as error:
            self.error = str(error)
            return

        if self.first_stamp_ns is None:
            self.first_stamp_ns = stamp_ns
        self.last_stamp_ns = stamp_ns
        elapsed = (stamp_ns - self.first_stamp_ns) / 1.0e9
        self.frame_count += 1

        label = (
            f"FAST VAD | CAM_FRONT | sim +{elapsed:04.1f}s | "
            f"speed {self.speed_mps:3.1f} m/s"
        )
        cv2.rectangle(image, (0, 0), (image.shape[1], 30), (16, 20, 24), -1)
        cv2.putText(
            image,
            label,
            (10, 21),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (245, 248, 250),
            1,
            cv2.LINE_AA,
        )
        path = self.output_dir / f"frame_{self.frame_count:05d}.png"
        if not cv2.imwrite(str(path), image, [cv2.IMWRITE_PNG_COMPRESSION, 3]):
            self.error = f"failed to write {path}"

    def finished(self):
        return self.error is not None or self.frame_count >= self.args.max_frames

    def write_manifest(self):
        payload = {
            "topic": self.args.topic,
            "odometry_topic": self.args.odometry_topic,
            "wait_for_motion": self.args.wait_for_motion,
            "motion_threshold_mps": self.args.motion_threshold,
            "frame_count": self.frame_count,
            "first_stamp_ns": self.first_stamp_ns,
            "last_stamp_ns": self.last_stamp_ns,
        }
        manifest = self.output_dir / "manifest.json"
        manifest.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        return manifest


def parse_args():
    parser = argparse.ArgumentParser(
        description="Record unique ROS image messages as a simulation-time frame sequence."
    )
    parser.add_argument("output_dir", type=Path)
    parser.add_argument(
        "--topic", default="/sensing/camera/CAM_FRONT/image_raw"
    )
    parser.add_argument(
        "--odometry-topic", default="/localization/kinematic_state"
    )
    parser.add_argument("--max-frames", type=int, default=100)
    parser.add_argument("--motion-threshold", type=float, default=0.05)
    parser.add_argument(
        "--no-wait-for-motion", dest="wait_for_motion", action="store_false"
    )
    parser.set_defaults(wait_for_motion=True)
    args = parser.parse_args()
    if args.max_frames <= 0:
        parser.error("--max-frames must be positive")
    if not math.isfinite(args.motion_threshold) or args.motion_threshold < 0.0:
        parser.error("--motion-threshold must be finite and non-negative")
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = ImageSequenceRecorder(args)
    try:
        while rclpy.ok() and not node.finished():
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        manifest = node.write_manifest()
        error = node.error
        count = node.frame_count
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if error:
        print(f"ERROR: {error}")
        return 1
    print(f"Recorded {count} frames; manifest: {manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
