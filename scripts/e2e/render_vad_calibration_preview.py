#!/usr/bin/env python3
"""Render a six-camera contact sheet with the base_link ground grid projected by bag calibration."""

from __future__ import annotations

import argparse
from collections import defaultdict, deque
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping

import cv2
import numpy as np

try:
    from vad_training_data_contract import CAMERA_ORDER
    from vad_training_data_contract import load_profile
    from validate_vad_training_bag import read_metadata
    from vad_calibration_contract import analyze_tf_tree
    from vad_calibration_contract import calibration_path_from_profile
    from vad_calibration_contract import expected_camera_pose
    from vad_calibration_contract import load_calibration
    from vad_calibration_contract import pose_error
except ModuleNotFoundError:
    from scripts.e2e.vad_training_data_contract import CAMERA_ORDER
    from scripts.e2e.vad_training_data_contract import load_profile
    from scripts.e2e.validate_vad_training_bag import read_metadata
    from scripts.e2e.vad_calibration_contract import analyze_tf_tree
    from scripts.e2e.vad_calibration_contract import calibration_path_from_profile
    from scripts.e2e.vad_calibration_contract import expected_camera_pose
    from scripts.e2e.vad_calibration_contract import load_calibration
    from scripts.e2e.vad_calibration_contract import pose_error


def quaternion_transform(translation: Iterable[float], quaternion: Iterable[float]) -> np.ndarray:
    x, y, z, w = (float(value) for value in quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 0:
        raise ValueError("invalid zero or non-finite transform quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    matrix[:3, 3] = np.asarray(tuple(translation), dtype=np.float64)
    return matrix


def transform_graph(
    transforms: Iterable[tuple[str, str, np.ndarray]]
) -> dict[str, list[tuple[str, np.ndarray]]]:
    graph: dict[str, list[tuple[str, np.ndarray]]] = defaultdict(list)
    for parent, child, parent_from_child in transforms:
        # TransformStamped stores the child pose in the parent frame.
        graph[parent].append((child, np.linalg.inv(parent_from_child)))
        graph[child].append((parent, parent_from_child))
    return graph


def lookup_transform(
    graph: Mapping[str, list[tuple[str, np.ndarray]]], source: str, target: str
) -> np.ndarray:
    queue = deque([(source, np.eye(4, dtype=np.float64))])
    visited: set[str] = set()
    while queue:
        node, node_from_source = queue.popleft()
        if node == target:
            return node_from_source
        if node in visited:
            continue
        visited.add(node)
        for neighbor, neighbor_from_node in graph.get(node, []):
            if neighbor not in visited:
                queue.append((neighbor, neighbor_from_node @ node_from_source))
    raise ValueError(f"no static transform path from {source} to {target}")


def project_point(
    point_base: tuple[float, float, float], camera_from_base: np.ndarray, intrinsic: np.ndarray
) -> tuple[int, int] | None:
    homogeneous = np.array([*point_base, 1.0], dtype=np.float64)
    point_camera = camera_from_base @ homogeneous
    if not np.all(np.isfinite(point_camera)) or point_camera[2] <= 0.05:
        return None
    pixel = intrinsic @ point_camera[:3]
    if pixel[2] <= 0:
        return None
    return int(round(pixel[0] / pixel[2])), int(round(pixel[1] / pixel[2]))


def read_preview_inputs(
    bag: Path, profile: Mapping[str, Any]
) -> tuple[dict[str, np.ndarray], dict[str, Any], list[tuple[str, str, np.ndarray]]]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    information, metadata_topics = read_metadata(bag)
    camera_by_image = {camera["image_topic"]: camera for camera in profile["cameras"]}
    camera_by_info = {camera["camera_info_topic"]: camera for camera in profile["cameras"]}
    tf_topic = profile["runtime_topics"]["tf_static"]["topic"]
    selected = (set(camera_by_image) | set(camera_by_info) | {tf_topic}) & set(metadata_topics)
    if len(selected) < len(CAMERA_ORDER) * 2 + 1:
        raise ValueError("bag does not contain all camera image/info topics and tf_static")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag), storage_id=information.get("storage_identifier", "sqlite3")
        ),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(selected)))
    message_classes = {
        topic: get_message(str(metadata_topics[topic]["type"])) for topic in selected
    }
    images: dict[str, np.ndarray] = {}
    camera_infos: dict[str, Any] = {}
    transforms: dict[tuple[str, str], np.ndarray] = {}
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic in camera_by_image and camera_by_image[topic]["name"] in images:
            continue
        if topic in camera_by_info and camera_by_info[topic]["name"] in camera_infos:
            continue
        message = deserialize_message(serialized, message_classes[topic])
        if topic in camera_by_image:
            name = camera_by_image[topic]["name"]
            if message.encoding == "bgr8":
                array = np.frombuffer(message.data, dtype=np.uint8).reshape(
                    int(message.height), int(message.step)
                )[:, : int(message.width) * 3]
                images[name] = array.reshape(int(message.height), int(message.width), 3).copy()
            elif message.encoding == "bgra8":
                array = np.frombuffer(message.data, dtype=np.uint8).reshape(
                    int(message.height), int(message.step)
                )[:, : int(message.width) * 4]
                bgra = array.reshape(int(message.height), int(message.width), 4)
                images[name] = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
            else:
                raise ValueError(f"unsupported {name} encoding: {message.encoding}")
        elif topic in camera_by_info:
            camera_infos[camera_by_info[topic]["name"]] = message
        elif topic == tf_topic:
            for transform in message.transforms:
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                key = (
                    str(transform.header.frame_id).lstrip("/"),
                    str(transform.child_frame_id).lstrip("/"),
                )
                transforms[key] = quaternion_transform(
                    (translation.x, translation.y, translation.z),
                    (rotation.x, rotation.y, rotation.z, rotation.w),
                )
    missing_images = sorted(set(CAMERA_ORDER) - set(images))
    missing_info = sorted(set(CAMERA_ORDER) - set(camera_infos))
    if missing_images or missing_info:
        raise ValueError(
            f"missing preview messages: images={missing_images}, camera_info={missing_info}"
        )
    return images, camera_infos, [
        (parent, child, matrix) for (parent, child), matrix in transforms.items()
    ]


def _draw_projected_polyline(
    image: np.ndarray,
    points: Iterable[tuple[float, float, float]],
    camera_from_base: np.ndarray,
    intrinsic: np.ndarray,
    color: tuple[int, int, int],
    thickness: int,
) -> int:
    visible_segments = 0
    previous: tuple[int, int] | None = None
    height, width = image.shape[:2]
    for point in points:
        projected = project_point(point, camera_from_base, intrinsic)
        if projected is None:
            previous = None
            continue
        inside = -width <= projected[0] < 2 * width and -height <= projected[1] < 2 * height
        if previous is not None and inside:
            cv2.line(image, previous, projected, color, thickness, cv2.LINE_AA)
            visible_segments += 1
        previous = projected if inside else None
    return visible_segments


def render_preview(
    bag: Path,
    profile: Mapping[str, Any],
    calibration: Mapping[str, Any],
    output: Path,
) -> dict[str, Any]:
    images, camera_infos, transforms = read_preview_inputs(bag, profile)
    graph = transform_graph(transforms)
    calibration_by_name = {camera["name"]: camera for camera in calibration["cameras"]}
    tf_tree = analyze_tf_tree(
        transforms,
        profile["model_contract"]["base_frame"],
        [camera["optical_frame"] for camera in profile["cameras"]],
    )
    if not tf_tree["valid"]:
        raise ValueError("tf_static is not a valid directed tree for all six optical frames")
    panels: list[np.ndarray] = []
    projection_counts: dict[str, int] = {}
    for camera in profile["cameras"]:
        name = camera["name"]
        panel = images[name].copy()
        info = camera_infos[name]
        intrinsic = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        expected = calibration_by_name[name]
        if not np.allclose(intrinsic.reshape(-1), expected["k"], rtol=0.0, atol=1e-6):
            raise ValueError(f"{name} CameraInfo K does not match the calibration manifest")
        translation_error, rotation_error = pose_error(
            tf_tree["poses_in_root"][camera["optical_frame"]], expected_camera_pose(expected)
        )
        if translation_error > 0.002 or rotation_error > 0.1:
            raise ValueError(f"{name} tf_static extrinsic does not match the calibration manifest")
        camera_from_base = lookup_transform(
            graph, profile["model_contract"]["base_frame"], camera["optical_frame"]
        )
        count = 0
        for lateral in range(-10, 11, 2):
            count += _draw_projected_polyline(
                panel,
                ((distance * 0.5, float(lateral), 0.0) for distance in range(1, 61)),
                camera_from_base,
                intrinsic,
                (70, 190, 255),
                1,
            )
        for longitudinal in range(2, 31, 2):
            count += _draw_projected_polyline(
                panel,
                ((float(longitudinal), lateral * 0.5, 0.0) for lateral in range(-20, 21)),
                camera_from_base,
                intrinsic,
                (80, 220, 100),
                1,
            )
        count += _draw_projected_polyline(
            panel,
            ((distance * 0.25, 0.0, 0.0) for distance in range(1, 121)),
            camera_from_base,
            intrinsic,
            (30, 30, 240),
            2,
        )
        projection_counts[name] = count
        cv2.rectangle(panel, (0, 0), (panel.shape[1], 30), (20, 20, 20), -1)
        cv2.putText(
            panel,
            f"{camera['model_index']}: {name}  grid segments={count}",
            (8, 21),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (245, 245, 245),
            1,
            cv2.LINE_AA,
        )
        panels.append(panel)

    target_height = max(panel.shape[0] for panel in panels)
    target_width = max(panel.shape[1] for panel in panels)
    normalized = [
        cv2.resize(panel, (target_width, target_height), interpolation=cv2.INTER_AREA)
        if panel.shape[:2] != (target_height, target_width)
        else panel
        for panel in panels
    ]
    mosaic = np.vstack((np.hstack(normalized[:3]), np.hstack(normalized[3:])))
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(output.stem + ".tmp" + output.suffix)
    if not cv2.imwrite(str(temporary), mosaic):
        raise OSError(f"failed to write calibration preview: {temporary}")
    temporary.replace(output)
    return {
        "output": str(output),
        "width": int(mosaic.shape[1]),
        "height": int(mosaic.shape[0]),
        "projection_segments": projection_counts,
        "calibration_sha256": calibration.get("_sha256"),
        "all_cameras_have_projection": all(value > 0 for value in projection_counts.values()),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument(
        "--profile",
        type=Path,
        default=Path(__file__).resolve().parents[2]
        / "autoware_e2e_vad_launch/config/vad_real_data_collection.yaml",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--calibration", type=Path)
    args = parser.parse_args()
    try:
        profile = load_profile(args.profile)
        calibration = load_calibration(
            args.calibration.expanduser().resolve()
            if args.calibration
            else calibration_path_from_profile(profile)
        )
        result = render_preview(
            args.bag.expanduser().resolve(), profile, calibration, args.output.expanduser().resolve()
        )
    except (OSError, RuntimeError, ValueError) as error:
        parser.error(str(error))
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["all_cameras_have_projection"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
