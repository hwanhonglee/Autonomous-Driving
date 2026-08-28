#!/usr/bin/env python3

import argparse
import time

import rclpy
from autoware_internal_planning_msgs.msg import CandidateTrajectories
from autoware_perception_msgs.msg import PredictedObjects
from autoware_planning_msgs.msg import Trajectory
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray


def parse_args():
    parser = argparse.ArgumentParser(description="Validate the live VAD output structure.")
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("timeout must be positive")
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = Node("autoware_e2e_vad_output_validator")
    messages = {}

    subscriptions = (
        (Trajectory, "/planning/trajectory", "trajectory"),
        (
            CandidateTrajectories,
            "/planning/vad/candidate_trajectories",
            "candidates",
        ),
        (
            PredictedObjects,
            "/perception/object_recognition/objects",
            "objects",
        ),
        (MarkerArray, "/perception/vad/map_points", "markers"),
    )
    for message_type, topic, key in subscriptions:
        node.create_subscription(
            message_type,
            topic,
            lambda message, output=key: messages.setdefault(output, message),
            10,
        )

    deadline = time.monotonic() + args.timeout
    while rclpy.ok() and len(messages) < len(subscriptions) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)

    missing = sorted({entry[2] for entry in subscriptions} - set(messages))
    if missing:
        print(f"[invalid] missing VAD outputs: {', '.join(missing)}")
        return 1

    trajectory = messages["trajectory"]
    candidates = messages["candidates"]
    objects = messages["objects"]
    markers = messages["markers"]
    candidate_points = [
        len(candidate.points) for candidate in candidates.candidate_trajectories
    ]
    marker_points = sum(len(marker.points) for marker in markers.markers)

    print(
        f"[ok] VAD structure: selected={len(trajectory.points)} points, "
        f"candidates={len(candidate_points)} {candidate_points}, "
        f"objects={len(objects.objects)}, "
        f"map_markers={len(markers.markers)}/{marker_points} points"
    )
    if (
        not trajectory.points
        or trajectory.header.frame_id != "map"
        or not candidate_points
        or not all(candidate_points)
        or not markers.markers
        or marker_points == 0
    ):
        print("[invalid] one or more VAD planning/map outputs are structurally empty")
        return 1
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    finally:
        if rclpy.ok():
            rclpy.shutdown()
