#!/usr/bin/env python3

# HH_260810 - Added a bounded compressed-frame verifier for the physically connected PC2 Lucid camera.
import argparse
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class FrameVerifier(Node):
    def __init__(self, topic: str, output: Path | None, raw: bool) -> None:
        super().__init__("lucid_frame_verifier")
        self.output = output
        self.result: str | None = None
        message_type = Image if raw else CompressedImage
        callback = self.on_raw_frame if raw else self.on_compressed_frame
        self.create_subscription(message_type, topic, callback, qos_profile_sensor_data)

    def on_compressed_frame(self, message: CompressedImage) -> None:
        encoded = np.frombuffer(message.data, dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            self.result = "decode_failed"
            return

        self.finish_frame(
            image,
            message.header.frame_id,
            message.header.stamp.sec,
            message.header.stamp.nanosec,
            message.format,
            len(message.data),
            bytes(message.data),
        )

    def on_raw_frame(self, message: Image) -> None:
        channels_by_encoding = {"bgr8": 3, "rgb8": 3, "mono8": 1}
        channels = channels_by_encoding.get(message.encoding)
        if channels is None:
            self.result = f"unsupported_encoding={message.encoding}"
            return

        row = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.step
        )
        visible = row[:, : message.width * channels]
        if channels == 1:
            image = visible.reshape(message.height, message.width)
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        else:
            image = visible.reshape(message.height, message.width, channels)
            if message.encoding == "rgb8":
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        encoded_ok, encoded = cv2.imencode(".jpg", image)
        if not encoded_ok:
            self.result = "encode_failed"
            return
        self.finish_frame(
            image,
            message.header.frame_id,
            message.header.stamp.sec,
            message.header.stamp.nanosec,
            message.encoding,
            len(message.data),
            encoded.tobytes(),
        )

    def finish_frame(
        self,
        image: np.ndarray,
        frame_id: str,
        stamp_sec: int,
        stamp_nanosec: int,
        image_format: str,
        source_bytes: int,
        output_bytes: bytes,
    ) -> None:

        grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        channel_mean = image.mean(axis=(0, 1))
        channel_std = image.std(axis=(0, 1))
        focus_variance = cv2.Laplacian(grayscale, cv2.CV_64F).var()
        underexposed = float(np.mean(grayscale <= 5) * 100.0)
        overexposed = float(np.mean(grayscale >= 250) * 100.0)

        if self.output is not None:
            self.output.parent.mkdir(parents=True, exist_ok=True)
            self.output.write_bytes(output_bytes)

        self.result = (
            f"frame_id={frame_id} "
            f"stamp={stamp_sec}.{stamp_nanosec:09d} "
            f"format={image_format} bytes={source_bytes} "
            f"shape={image.shape[1]}x{image.shape[0]}x{image.shape[2]} "
            f"bgr_mean={channel_mean.round(2).tolist()} "
            f"bgr_std={channel_std.round(2).tolist()} "
            f"focus_variance={focus_variance:.2f} "
            f"underexposed_pct={underexposed:.3f} "
            f"overexposed_pct={overexposed:.3f} "
            f"output={self.output if self.output is not None else 'none'}"
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--topic", default="/lucid_vision/camera/image_compressed"
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        help="subscribe to sensor_msgs/Image and encode the captured frame as JPEG",
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = FrameVerifier(args.topic, args.output, args.raw)
    deadline = time.monotonic() + args.timeout
    try:
        while node.result is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.25)
        if node.result is None:
            print(f"timeout waiting for {args.topic}", file=sys.stderr)
            return 2
        print(node.result)
        return 0 if node.result != "decode_failed" else 3
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
