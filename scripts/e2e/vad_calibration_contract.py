#!/usr/bin/env python3
"""Calibration manifest schema and transform helpers for real VAD capture."""

from __future__ import annotations

from collections import defaultdict, deque
import hashlib
import math
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np
import yaml

try:
    from vad_training_data_contract import CAMERA_ORDER
except ModuleNotFoundError:
    from scripts.e2e.vad_training_data_contract import CAMERA_ORDER


class CalibrationError(ValueError):
    pass


def _finite_numbers(values: Any, length: int, context: str) -> list[float]:
    if not isinstance(values, list) or len(values) != length:
        raise CalibrationError(f"{context} must contain {length} numbers")
    if any(isinstance(value, bool) or not isinstance(value, (int, float)) for value in values):
        raise CalibrationError(f"{context} must contain only numbers")
    result = [float(value) for value in values]
    if not all(math.isfinite(value) for value in result):
        raise CalibrationError(f"{context} contains a non-finite value")
    return result


def validate_calibration(calibration: Mapping[str, Any]) -> None:
    if calibration.get("schema_version") != 1:
        raise CalibrationError("calibration schema_version must be 1")
    if calibration.get("base_frame") != "base_link":
        raise CalibrationError("calibration base_frame must be base_link")
    if calibration.get("image_width") != 640 or calibration.get("image_height") != 360:
        raise CalibrationError("calibration image size must be 640x360")
    cameras = calibration.get("cameras")
    if not isinstance(cameras, list) or len(cameras) != len(CAMERA_ORDER):
        raise CalibrationError("calibration must contain exactly six cameras")
    serials: set[str] = set()
    frames: set[str] = set()
    poses: set[tuple[float, ...]] = set()
    for index, raw_camera in enumerate(cameras):
        if not isinstance(raw_camera, Mapping):
            raise CalibrationError(f"calibration cameras[{index}] must be a mapping")
        expected_name = CAMERA_ORDER[index]
        if raw_camera.get("name") != expected_name or raw_camera.get("model_index") != index:
            raise CalibrationError(
                f"calibration cameras[{index}] must be {expected_name} with model_index {index}"
            )
        serial = raw_camera.get("serial")
        frame = raw_camera.get("optical_frame")
        if not isinstance(serial, str) or not serial or serial.startswith("REPLACE_WITH_"):
            raise CalibrationError(f"{expected_name}.serial must identify the physical camera")
        if not isinstance(frame, str) or not frame:
            raise CalibrationError(f"{expected_name}.optical_frame must be set")
        if serial in serials or frame in frames:
            raise CalibrationError("calibration camera serials and optical frames must be unique")
        serials.add(serial)
        frames.add(frame)
        if raw_camera.get("distortion_model") != "plumb_bob":
            raise CalibrationError(f"{expected_name}.distortion_model must be plumb_bob")
        if raw_camera.get("timestamp_source") != "hardware_exposure":
            raise CalibrationError(f"{expected_name}.timestamp_source must be hardware_exposure")
        if raw_camera.get("trigger_mode") not in ("hardware", "ptp"):
            raise CalibrationError(f"{expected_name}.trigger_mode must be hardware or ptp")
        distortion = _finite_numbers(raw_camera.get("d"), 5, f"{expected_name}.d")
        if any(abs(value) > 1e-9 for value in distortion):
            raise CalibrationError(f"{expected_name}.d must be zero for the rectified feed")
        intrinsic = _finite_numbers(raw_camera.get("k"), 9, f"{expected_name}.k")
        if intrinsic[0] <= 0 or intrinsic[4] <= 0 or abs(intrinsic[8] - 1.0) > 1e-9:
            raise CalibrationError(f"{expected_name}.k has invalid focal length or homogeneous scale")
        if not 0 <= intrinsic[2] < 640 or not 0 <= intrinsic[5] < 360:
            raise CalibrationError(f"{expected_name}.k principal point is outside the image")
        if abs(intrinsic[1]) > 1e-9 or abs(intrinsic[3]) > 1e-9:
            raise CalibrationError(f"{expected_name}.k skew terms must be zero")
        pose = raw_camera.get("camera_pose_in_base")
        if not isinstance(pose, Mapping):
            raise CalibrationError(f"{expected_name}.camera_pose_in_base must be a mapping")
        translation = _finite_numbers(
            pose.get("translation_xyz"), 3, f"{expected_name}.translation_xyz"
        )
        quaternion = _finite_numbers(
            pose.get("rotation_xyzw"), 4, f"{expected_name}.rotation_xyzw"
        )
        quaternion_norm = math.sqrt(sum(value * value for value in quaternion))
        if abs(quaternion_norm - 1.0) > 1e-4:
            raise CalibrationError(f"{expected_name}.rotation_xyzw must be normalized")
        signature = tuple(round(value, 9) for value in translation + quaternion)
        if signature in poses:
            raise CalibrationError("two cameras have an identical extrinsic pose")
        poses.add(signature)


def load_calibration(path: Path | str) -> dict[str, Any]:
    calibration_path = Path(path).expanduser().resolve()
    try:
        document = yaml.safe_load(calibration_path.read_text(encoding="utf-8"))
    except OSError as error:
        raise CalibrationError(f"cannot read calibration {calibration_path}: {error}") from error
    except yaml.YAMLError as error:
        raise CalibrationError(f"invalid calibration YAML {calibration_path}: {error}") from error
    if not isinstance(document, Mapping):
        raise CalibrationError("calibration document must be a mapping")
    calibration = dict(document)
    validate_calibration(calibration)
    calibration["_path"] = str(calibration_path)
    calibration["_sha256"] = hashlib.sha256(calibration_path.read_bytes()).hexdigest()
    return calibration


def calibration_path_from_profile(profile: Mapping[str, Any]) -> Path:
    raw_path = profile.get("provenance", {}).get("calibration_file")
    if not isinstance(raw_path, str) or not raw_path or raw_path.startswith("REPLACE_WITH_"):
        raise CalibrationError("provenance.calibration_file must point to a completed manifest")
    path = Path(raw_path).expanduser()
    if not path.is_absolute():
        profile_path = profile.get("_profile_path")
        if not profile_path:
            raise CalibrationError("relative calibration path needs a loaded profile path")
        path = Path(profile_path).parent / path
    return path.resolve()


def quaternion_transform(translation: Sequence[float], quaternion: Sequence[float]) -> np.ndarray:
    x, y, z, w = (float(value) for value in quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 0:
        raise CalibrationError("invalid transform quaternion")
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
    matrix[:3, 3] = np.asarray(translation, dtype=np.float64)
    return matrix


def expected_camera_pose(camera: Mapping[str, Any]) -> np.ndarray:
    pose = camera["camera_pose_in_base"]
    return quaternion_transform(pose["translation_xyz"], pose["rotation_xyzw"])


def analyze_tf_tree(
    transforms: Iterable[tuple[str, str, np.ndarray]], root: str, targets: Iterable[str]
) -> dict[str, Any]:
    target_list = list(targets)
    transform_list = list(transforms)
    parents_by_child: dict[str, set[str]] = defaultdict(set)
    for parent, child, _matrix in transform_list:
        parents_by_child[child].add(parent)

    # A bag can contain transforms for the whole vehicle. Only ancestors of the
    # six VAD optical frames can affect this calibration contract.
    relevant_nodes = {root, *target_list}
    pending = deque(target_list)
    while pending:
        child = pending.popleft()
        for parent in parents_by_child.get(child, set()):
            if parent not in relevant_nodes:
                relevant_nodes.add(parent)
                pending.append(parent)
    relevant_transforms = [
        (parent, child, matrix)
        for parent, child, matrix in transform_list
        if parent in relevant_nodes and child in relevant_nodes
    ]

    parent_by_child: dict[str, str] = {}
    matrix_by_edge: dict[tuple[str, str], np.ndarray] = {}
    conflicting_parents: set[str] = set()
    conflicting_values: set[str] = set()
    children: dict[str, set[str]] = defaultdict(set)
    for parent, child, matrix in relevant_transforms:
        previous_parent = parent_by_child.get(child)
        if previous_parent is not None and previous_parent != parent:
            conflicting_parents.add(child)
        parent_by_child[child] = parent
        edge = (parent, child)
        if edge in matrix_by_edge and not np.allclose(matrix_by_edge[edge], matrix, atol=1e-9):
            conflicting_values.add(f"{parent}->{child}")
        matrix_by_edge[edge] = matrix
        children[parent].add(child)

    visiting: set[str] = set()
    visited: set[str] = set()
    cycle = False

    def visit(node: str) -> None:
        nonlocal cycle
        if node in visiting:
            cycle = True
            return
        if node in visited:
            return
        visiting.add(node)
        for child in children.get(node, set()):
            visit(child)
        visiting.remove(node)
        visited.add(node)

    for node in set(children) | set(parent_by_child):
        visit(node)

    poses: dict[str, np.ndarray] = {root: np.eye(4, dtype=np.float64)}
    queue = deque([root])
    expanded: set[str] = set()
    while queue:
        parent = queue.popleft()
        if parent in expanded:
            continue
        expanded.add(parent)
        for child in children.get(parent, set()):
            if child in poses:
                continue
            poses[child] = poses[parent] @ matrix_by_edge[(parent, child)]
            queue.append(child)
    target_paths = {target: target in poses for target in target_list}
    return {
        "valid": not conflicting_parents
        and not conflicting_values
        and not cycle
        and all(target_paths.values()),
        "relevant_nodes": sorted(relevant_nodes),
        "conflicting_parent_children": sorted(conflicting_parents),
        "conflicting_value_edges": sorted(conflicting_values),
        "cycle": cycle,
        "target_paths": target_paths,
        "poses_in_root": poses,
    }


def pose_error(actual: np.ndarray, expected: np.ndarray) -> tuple[float, float]:
    translation_error = float(np.linalg.norm(actual[:3, 3] - expected[:3, 3]))
    relative_rotation = actual[:3, :3] @ expected[:3, :3].T
    cosine = max(-1.0, min(1.0, (float(np.trace(relative_rotation)) - 1.0) / 2.0))
    rotation_error_deg = math.degrees(math.acos(cosine))
    return translation_error, rotation_error_deg
