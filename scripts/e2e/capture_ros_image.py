#!/usr/bin/env python3

import argparse
import os
from pathlib import Path
import tempfile
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ImageCapture(Node):
    def __init__(self, topic):
        super().__init__("autoware_e2e_image_capture")
        self.bridge = CvBridge()
        self.image = None
        self.error = None
        self.subscription = self.create_subscription(
            Image, topic, self._on_image, qos_profile_sensor_data
        )

    def _on_image(self, message):
        if self.image is not None:
            return
        try:
            self.image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as error:
            self.error = str(error)


def parse_args():
    parser = argparse.ArgumentParser(description="Save one ROS Image topic frame.")
    parser.add_argument("output", type=Path)
    parser.add_argument(
        "--topic",
        default="/sensing/camera/all_cameras/image_raw",
        help="sensor_msgs/msg/Image topic",
    )
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()
    if args.timeout <= 0.0:
        parser.error("--timeout must be positive")
    if args.output.suffix.lower() not in (".png", ".jpg", ".jpeg"):
        parser.error("output extension must be .png, .jpg, or .jpeg")
    return args


def write_image(path, image):
    output = path.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    success, encoded = cv2.imencode(output.suffix, image)
    if not success:
        raise RuntimeError(f"failed to encode {output.suffix} image")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output.name}.", suffix=".tmp", dir=str(output.parent)
    )
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(encoded.tobytes())
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, output)
    except Exception:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise
    return output


def main():
    args = parse_args()
    rclpy.init()
    node = ImageCapture(args.topic)
    try:
        deadline = time.monotonic() + args.timeout
        while rclpy.ok() and node.image is None and node.error is None:
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f"timed out waiting {args.timeout:.1f}s for {args.topic}"
                )
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.error is not None:
            raise RuntimeError(f"failed to convert {args.topic}: {node.error}")
        output = write_image(args.output, node.image)
        print(f"ROS image captured: {output}")
        return 0
    except RuntimeError as error:
        print(f"ERROR: {error}")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
