#!/usr/bin/env python3
"""Audit six-camera CameraInfo timing in a ROS 2 bag without replaying it."""

from __future__ import annotations

import argparse
from bisect import bisect_left
from collections import Counter
import json
from pathlib import Path
from typing import Iterable


CAMERA_NAMES = (
    "CAM_FRONT",
    "CAM_BACK",
    "CAM_FRONT_LEFT",
    "CAM_BACK_LEFT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK_RIGHT",
)


def gap_histogram(stamps_ns: Iterable[int], quantum_ns: int = 50_000_000) -> dict[str, int]:
    stamps = sorted(stamps_ns)
    buckets = Counter(round((right - left) / quantum_ns) * quantum_ns for left, right in zip(stamps, stamps[1:]))
    return {f"{gap / 1e6:.0f}ms": count for gap, count in sorted(buckets.items())}


def _nearest(stamps: list[int], target: int) -> int | None:
    position = bisect_left(stamps, target)
    candidates = stamps[max(0, position - 1) : min(len(stamps), position + 1)]
    return min(candidates, key=lambda stamp: (abs(stamp - target), stamp)) if candidates else None


def front_anchored_coverage(
    stamps_by_camera: dict[str, list[int]], tolerance_ns: int
) -> dict[str, float | int | None]:
    front = sorted(set(stamps_by_camera["CAM_FRONT"]))
    matched_spans = []
    for anchor in front:
        bundle = [anchor]
        for camera in CAMERA_NAMES[1:]:
            nearest = _nearest(sorted(set(stamps_by_camera[camera])), anchor)
            if nearest is None or abs(nearest - anchor) > tolerance_ns:
                break
            bundle.append(nearest)
        if len(bundle) == len(CAMERA_NAMES):
            matched_spans.append(max(bundle) - min(bundle))

    count = len(matched_spans)
    return {
        "matched": count,
        "front": len(front),
        "coverage_percent": 100.0 * count / len(front) if front else 0.0,
        "maximum_stamp_span_ms": max(matched_spans) / 1e6 if matched_spans else None,
    }


def summarize(stamps_by_camera: dict[str, list[int]]) -> dict:
    missing = [camera for camera in CAMERA_NAMES if camera not in stamps_by_camera]
    if missing:
        raise ValueError(f"missing camera topics: {', '.join(missing)}")

    stamp_sets = {camera: set(stamps_by_camera[camera]) for camera in CAMERA_NAMES}
    common = set.intersection(*stamp_sets.values())
    union = set.union(*stamp_sets.values())
    multiplicity = Counter(sum(stamp in values for values in stamp_sets.values()) for stamp in union)
    return {
        "per_camera": {
            camera: {
                "count": len(stamps_by_camera[camera]),
                "unique_count": len(stamp_sets[camera]),
                "gap_histogram": gap_histogram(stamp_sets[camera]),
            }
            for camera in CAMERA_NAMES
        },
        "union_stamp_count": len(union),
        "exact_six_camera_count": len(common),
        "exact_six_camera_gap_histogram": gap_histogram(common),
        "stamp_multiplicity_histogram": {
            str(camera_count): count for camera_count, count in sorted(multiplicity.items())
        },
        "within_50ms": front_anchored_coverage(stamps_by_camera, 50_000_000),
        "within_200ms": front_anchored_coverage(stamps_by_camera, 200_000_000),
    }


def read_camera_info_stamps(bag: Path) -> dict[str, list[int]]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CameraInfo

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topics = {
        topic.name: topic.type
        for topic in reader.get_all_topics_and_types()
        if topic.name.endswith("/camera_info")
    }
    reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(topics)))
    stamps = {camera: [] for camera in CAMERA_NAMES}
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        camera = topic.split("/")[-2]
        if camera not in stamps:
            continue
        message = deserialize_message(serialized, CameraInfo)
        stamps[camera].append(message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec)
    return stamps


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="+", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    report = {}
    for bag in args.bags:
        report[str(bag)] = summarize(read_camera_info_stamps(bag))
    rendered = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
