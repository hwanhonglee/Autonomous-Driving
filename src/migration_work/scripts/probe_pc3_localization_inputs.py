#!/usr/bin/env python3

# HH_260811 - Added a bounded, subscribe-only probe across the PC3 GNSS-to-localization chain.
import argparse
import json
import time
from collections import defaultdict

import rclpy
from autoware_sensing_msgs.msg import GnssInsOrientationStamped
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from tf2_msgs.msg import TFMessage
from tier4_debug_msgs.msg import BoolStamped
from tier4_map_msgs.msg import MapProjectorInfo


SENSOR_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
RELIABLE_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
TRANSIENT_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class LocalizationInputProbe(Node):
    def __init__(self) -> None:
        super().__init__("pc3_localization_input_probe")
        self.counts: dict[str, int] = defaultdict(int)
        self.last: dict[str, object] = {}
        self.tf_pairs: set[str] = set()

        self.create_subscription(
            MapProjectorInfo, "/map/map_projector_info", self.on_projector, TRANSIENT_QOS
        )
        self.create_subscription(
            NavSatFix, "/sensing/gnss/novatel/oem7/fix", self.on_fix, SENSOR_QOS
        )
        self.create_subscription(
            GnssInsOrientationStamped,
            "/autoware_orientation",
            self.on_orientation,
            RELIABLE_QOS,
        )
        self.create_subscription(
            PoseStamped, "/sensing/gnss/pose", lambda msg: self.on_pose("pose", msg), RELIABLE_QOS
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/sensing/gnss/pose_with_covariance",
            self.on_pose_cov,
            RELIABLE_QOS,
        )
        self.create_subscription(
            BoolStamped, "/sensing/gnss/fixed", self.on_fixed, RELIABLE_QOS
        )
        self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            self.on_kinematic_state,
            RELIABLE_QOS,
        )
        self.create_subscription(TFMessage, "/tf", self.on_tf, RELIABLE_QOS)

    def on_projector(self, msg: MapProjectorInfo) -> None:
        self.counts["map_projector_info"] += 1
        self.last["map_projector_info"] = {
            "projector_type": msg.projector_type,
            "vertical_datum": msg.vertical_datum,
            "mgrs_grid": msg.mgrs_grid,
            "origin": [msg.map_origin.latitude, msg.map_origin.longitude, msg.map_origin.altitude],
        }

    def on_fix(self, msg: NavSatFix) -> None:
        self.counts["novatel_fix"] += 1
        level = msg.status.status[0] if isinstance(msg.status.status, bytes) else int(msg.status.status)
        self.last["novatel_fix"] = {
            "frame": msg.header.frame_id,
            "status": level,
            "service": int(msg.status.service),
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "altitude": msg.altitude,
            "covariance_type": int(msg.position_covariance_type),
        }

    def on_orientation(self, msg: GnssInsOrientationStamped) -> None:
        self.counts["autoware_orientation"] += 1
        q = msg.orientation.orientation
        self.last["autoware_orientation"] = {
            "frame": msg.header.frame_id,
            "quaternion": [q.x, q.y, q.z, q.w],
            "rmse": [
                msg.orientation.rmse_rotation_x,
                msg.orientation.rmse_rotation_y,
                msg.orientation.rmse_rotation_z,
            ],
        }

    def on_pose(self, name: str, msg: PoseStamped) -> None:
        self.counts[name] += 1
        self.last[name] = {"frame": msg.header.frame_id}

    def on_pose_cov(self, msg: PoseWithCovarianceStamped) -> None:
        self.counts["pose_with_covariance"] += 1
        self.last["pose_with_covariance"] = {"frame": msg.header.frame_id}

    def on_fixed(self, msg: BoolStamped) -> None:
        self.counts["fixed"] += 1
        self.last["fixed"] = bool(msg.data)

    def on_kinematic_state(self, msg: Odometry) -> None:
        self.counts["kinematic_state"] += 1
        self.last["kinematic_state"] = {
            "frame": msg.header.frame_id,
            "child_frame": msg.child_frame_id,
        }

    def on_tf(self, msg: TFMessage) -> None:
        self.counts["tf"] += 1
        for transform in msg.transforms:
            self.tf_pairs.add(f"{transform.header.frame_id}->{transform.child_frame_id}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()
    node = LocalizationInputProbe()
    started = time.monotonic()
    try:
        deadline = started + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.25)

        matching_topics = {
            name: types
            for name, types in node.get_topic_names_and_types()
            if any(token in name for token in ("gnss", "orientation", "kinematic", "/tf"))
        }
        print(
            json.dumps(
                {
                    "duration_s": round(time.monotonic() - started, 3),
                    "counts": dict(sorted(node.counts.items())),
                    "last": node.last,
                    "tf_pairs": sorted(node.tf_pairs),
                    "matching_topics": dict(sorted(matching_topics.items())),
                },
                indent=2,
                sort_keys=True,
            )
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
