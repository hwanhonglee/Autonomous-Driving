#!/usr/bin/env python3
"""Shared contract and live preflight for VAD training-data capture."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import re
import subprocess
import time
from typing import Any, Iterable, Mapping

import yaml


CAMERA_ORDER = (
    "CAM_FRONT",
    "CAM_BACK",
    "CAM_FRONT_LEFT",
    "CAM_BACK_LEFT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK_RIGHT",
)

IMAGE_TYPE = "sensor_msgs/msg/Image"
CAMERA_INFO_TYPE = "sensor_msgs/msg/CameraInfo"
DEVICE_INFO_TYPE = "diagnostic_msgs/msg/DiagnosticStatus"
FIXED_MODEL_CONTRACT = {
    "model_name": "vad-carla-tiny",
    "input_width": 640,
    "input_height": 360,
    "target_width": 640,
    "target_height": 384,
    "can_bus_dimension": 18,
    "base_frame": "base_link",
    "map_frame": "map",
    "acceleration_frame": "base_link",
}
RUNTIME_TOPIC_TYPES = {
    "kinematic_state": "nav_msgs/msg/Odometry",
    "acceleration": "geometry_msgs/msg/AccelWithCovarianceStamped",
    "tf_static": "tf2_msgs/msg/TFMessage",
}


class ProfileError(ValueError):
    """Raised when a capture profile cannot represent the current VAD contract."""


@dataclass(frozen=True)
class TopicSpec:
    key: str
    topic: str
    message_type: str
    category: str
    required_for_capture: bool
    required_for_planning: bool
    required_for_multitask: bool

    def to_dict(self) -> dict[str, Any]:
        return {
            "key": self.key,
            "topic": self.topic,
            "type": self.message_type,
            "category": self.category,
            "required_for_capture": self.required_for_capture,
            "required_for_planning": self.required_for_planning,
            "required_for_multitask": self.required_for_multitask,
        }


def _mapping(value: Any, context: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ProfileError(f"{context} must be a mapping")
    return value


def _list(value: Any, context: str) -> list[Any]:
    if not isinstance(value, list):
        raise ProfileError(f"{context} must be a list")
    return value


def _positive_number(value: Any, context: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
        raise ProfileError(f"{context} must be a positive number")
    return float(value)


def _topic_name(value: Any, context: str) -> str:
    if not isinstance(value, str) or not value.startswith("/") or " " in value:
        raise ProfileError(f"{context} must be an absolute ROS topic")
    return value


def _message_type(value: Any, context: str) -> str:
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9_]+/msg/[A-Za-z0-9_]+", value):
        raise ProfileError(f"{context} must use package/msg/Message syntax")
    return value


def load_profile(path: Path | str) -> dict[str, Any]:
    profile_path = Path(path).expanduser().resolve()
    try:
        data = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    except OSError as error:
        raise ProfileError(f"cannot read profile {profile_path}: {error}") from error
    except yaml.YAMLError as error:
        raise ProfileError(f"invalid YAML in {profile_path}: {error}") from error
    profile = dict(_mapping(data, "profile"))
    validate_profile(profile)
    profile["_profile_path"] = str(profile_path)
    profile["_profile_sha256"] = hashlib.sha256(profile_path.read_bytes()).hexdigest()
    return profile


def find_placeholders(value: Any, prefix: str = "") -> list[str]:
    """Return dotted paths whose values still carry an explicit template marker."""
    found: list[str] = []
    if isinstance(value, Mapping):
        for key, child in value.items():
            if str(key).startswith("_"):
                continue
            child_prefix = f"{prefix}.{key}" if prefix else str(key)
            found.extend(find_placeholders(child, child_prefix))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            found.extend(find_placeholders(child, f"{prefix}[{index}]"))
    elif isinstance(value, str) and value.startswith("REPLACE_WITH_"):
        found.append(prefix)
    return found


def sha256_file(path: Path, chunk_size: int = 8 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(chunk_size):
            digest.update(chunk)
    return digest.hexdigest()


def model_artifact_report(profile: Mapping[str, Any]) -> dict[str, Any]:
    workspace = Path(__file__).resolve().parents[2]
    report: dict[str, Any] = {}
    passed = True
    for key, entry in profile["model_contract"]["model_files"].items():
        path = (workspace / entry["path"]).resolve()
        inside_workspace = path == workspace or workspace in path.parents
        actual_hash = sha256_file(path) if inside_workspace and path.is_file() else None
        matches = actual_hash == entry["sha256"]
        report[key] = {
            "path": str(path),
            "expected_sha256": entry["sha256"],
            "actual_sha256": actual_hash,
            "matches": matches,
        }
        passed = passed and matches
    return {"pass": passed, "files": report}


def load_profile_calibration(profile: Mapping[str, Any]) -> dict[str, Any]:
    try:
        from vad_calibration_contract import calibration_path_from_profile
        from vad_calibration_contract import load_calibration
    except ModuleNotFoundError:
        from scripts.e2e.vad_calibration_contract import calibration_path_from_profile
        from scripts.e2e.vad_calibration_contract import load_calibration
    return load_calibration(calibration_path_from_profile(profile))


def load_calibration_file(path: Path | str) -> dict[str, Any]:
    try:
        from vad_calibration_contract import load_calibration
    except ModuleNotFoundError:
        from scripts.e2e.vad_calibration_contract import load_calibration
    return load_calibration(path)


def load_profile_commissioning_contract(
    profile: Mapping[str, Any], calibration: Mapping[str, Any]
) -> dict[str, Any]:
    try:
        from vad_commissioning_contract import load_profile_commissioning
    except ModuleNotFoundError:
        from scripts.e2e.vad_commissioning_contract import load_profile_commissioning
    return load_profile_commissioning(profile, calibration)


def resolve_profile_provenance_file(profile: Mapping[str, Any], key: str) -> Path:
    try:
        from vad_commissioning_contract import resolve_profile_file
    except ModuleNotFoundError:
        from scripts.e2e.vad_commissioning_contract import resolve_profile_file
    return resolve_profile_file(profile, key)


def load_commissioning_file(
    path: Path | str,
    calibration: Mapping[str, Any],
    rectification_config: Path,
    profile: Mapping[str, Any],
) -> dict[str, Any]:
    try:
        from vad_commissioning_contract import load_commissioning
    except ModuleNotFoundError:
        from scripts.e2e.vad_commissioning_contract import load_commissioning
    return load_commissioning(path, calibration, rectification_config, profile)


def validate_profile(profile: Mapping[str, Any]) -> None:
    if profile.get("schema_version") != 1:
        raise ProfileError("schema_version must be 1")

    session = _mapping(profile.get("session"), "session")
    if not isinstance(session.get("source"), str) or not session["source"]:
        raise ProfileError("session.source must be a non-empty string")
    if not isinstance(session.get("vehicle_id"), str) or not session["vehicle_id"]:
        raise ProfileError("session.vehicle_id must be a non-empty string")

    model = _mapping(profile.get("model_contract"), "model_contract")
    if tuple(_list(model.get("camera_order"), "model_contract.camera_order")) != CAMERA_ORDER:
        raise ProfileError("model_contract.camera_order must match the carla_tiny training order")
    for key, expected in FIXED_MODEL_CONTRACT.items():
        if model.get(key) != expected:
            raise ProfileError(f"model_contract.{key} must be {expected!r}")
    if model.get("input_encoding") not in ("bgr8", "bgra8"):
        raise ProfileError("model_contract.input_encoding must be bgr8 or bgra8")
    if model.get("rectified_images") is not True:
        raise ProfileError("model_contract.rectified_images must be true")
    model_files = _mapping(model.get("model_files"), "model_contract.model_files")
    if set(model_files) != {"param_json", "backbone", "head", "head_no_prev"}:
        raise ProfileError("model_contract.model_files must name param_json/backbone/head/head_no_prev")
    for key, entry_value in model_files.items():
        entry = _mapping(entry_value, f"model_contract.model_files.{key}")
        if not isinstance(entry.get("path"), str) or not entry["path"]:
            raise ProfileError(f"model_contract.model_files.{key}.path must be set")
        if not isinstance(entry.get("sha256"), str) or not re.fullmatch(
            r"[0-9a-f]{64}", entry["sha256"]
        ):
            raise ProfileError(f"model_contract.model_files.{key}.sha256 must be lowercase SHA256")

    thresholds = _mapping(profile.get("validation"), "validation")
    for key in (
        "minimum_duration_s",
        "minimum_camera_rate_hz",
        "minimum_state_rate_hz",
        "maximum_p99_gap_ms",
        "maximum_state_delta_ms",
        "runtime_sync_tolerance_ms",
        "dataset_sync_tolerance_ms",
        "minimum_bundle_coverage_percent",
        "maximum_count_imbalance_percent",
        "minimum_motion_distance_m",
        "maximum_command_age_ms",
        "minimum_command_coverage_percent",
        "maximum_intrinsic_error",
        "maximum_extrinsic_translation_error_m",
        "maximum_extrinsic_rotation_error_deg",
        "maximum_ptp_offset_ns",
        "maximum_status_age_ms",
        "maximum_image_transport_delay_ms",
        "maximum_device_status_delta_ms",
        "future_horizon_s",
        "future_step_s",
    ):
        _positive_number(thresholds.get(key), f"validation.{key}")
    if float(thresholds["runtime_sync_tolerance_ms"]) > float(
        thresholds["dataset_sync_tolerance_ms"]
    ):
        raise ProfileError("runtime sync tolerance cannot exceed dataset sync tolerance")
    if float(thresholds["minimum_bundle_coverage_percent"]) > 100.0:
        raise ProfileError("minimum_bundle_coverage_percent cannot exceed 100")
    if float(thresholds["maximum_count_imbalance_percent"]) > 100.0:
        raise ProfileError("maximum_count_imbalance_percent cannot exceed 100")
    if float(thresholds["minimum_duration_s"]) <= float(thresholds["future_horizon_s"]):
        raise ProfileError("minimum_duration_s must exceed future_horizon_s")
    for key in ("require_zero_distortion", "require_reliable_images", "require_transient_local_tf_static"):
        if not isinstance(thresholds.get(key), bool):
            raise ProfileError(f"validation.{key} must be boolean")

    cameras = _list(profile.get("cameras"), "cameras")
    if len(cameras) != len(CAMERA_ORDER):
        raise ProfileError(f"cameras must contain exactly {len(CAMERA_ORDER)} entries")
    image_topics: set[str] = set()
    info_topics: set[str] = set()
    device_topics: set[str] = set()
    frames: set[str] = set()
    for index, camera_value in enumerate(cameras):
        camera = _mapping(camera_value, f"cameras[{index}]")
        expected_name = CAMERA_ORDER[index]
        if camera.get("name") != expected_name or camera.get("model_index") != index:
            raise ProfileError(
                f"cameras[{index}] must be {expected_name} with model_index {index}"
            )
        image_topic = _topic_name(camera.get("image_topic"), f"{expected_name}.image_topic")
        info_topic = _topic_name(
            camera.get("camera_info_topic"), f"{expected_name}.camera_info_topic"
        )
        if camera.get("image_type") != IMAGE_TYPE:
            raise ProfileError(f"{expected_name}.image_type must be {IMAGE_TYPE}")
        if camera.get("camera_info_type") != CAMERA_INFO_TYPE:
            raise ProfileError(f"{expected_name}.camera_info_type must be {CAMERA_INFO_TYPE}")
        device_topic = _topic_name(
            camera.get("device_info_topic"), f"{expected_name}.device_info_topic"
        )
        if camera.get("device_info_type") != DEVICE_INFO_TYPE:
            raise ProfileError(f"{expected_name}.device_info_type must be {DEVICE_INFO_TYPE}")
        frame = camera.get("optical_frame")
        if not isinstance(frame, str) or not frame:
            raise ProfileError(f"{expected_name}.optical_frame must be set")
        if (
            image_topic in image_topics
            or info_topic in info_topics
            or device_topic in device_topics
            or frame in frames
        ):
            raise ProfileError(
                "camera image/info/device topics and optical frames must be unique"
            )
        image_topics.add(image_topic)
        info_topics.add(info_topic)
        device_topics.add(device_topic)
        frames.add(frame)

    runtime_topics = _mapping(profile.get("runtime_topics"), "runtime_topics")
    for key, expected_type in RUNTIME_TOPIC_TYPES.items():
        entry = _mapping(runtime_topics.get(key), f"runtime_topics.{key}")
        _topic_name(entry.get("topic"), f"runtime_topics.{key}.topic")
        if entry.get("type") != expected_type:
            raise ProfileError(f"runtime_topics.{key}.type must be {expected_type}")

    capture_topics = _mapping(profile.get("capture_topics"), "capture_topics")
    time_sync = _mapping(capture_topics.get("time_sync_status"), "capture_topics.time_sync_status")
    _topic_name(time_sync.get("topic"), "capture_topics.time_sync_status.topic")
    if time_sync.get("type") != DEVICE_INFO_TYPE:
        raise ProfileError(f"capture_topics.time_sync_status.type must be {DEVICE_INFO_TYPE}")

    for category in ("context_topics", "label_topics"):
        entries = _mapping(profile.get(category, {}), category)
        for key, entry_value in entries.items():
            entry = _mapping(entry_value, f"{category}.{key}")
            _topic_name(entry.get("topic"), f"{category}.{key}.topic")
            _message_type(entry.get("type"), f"{category}.{key}.type")
            if "record" in entry and not isinstance(entry["record"], bool):
                raise ProfileError(f"{category}.{key}.record must be boolean")

    labels = _mapping(profile.get("label_topics"), "label_topics")
    required_label_keys = {"command", "object_ground_truth", "vector_map_ground_truth"}
    if not required_label_keys.issubset(labels):
        missing = ", ".join(sorted(required_label_keys - set(labels)))
        raise ProfileError(f"label_topics is missing: {missing}")


def iter_topic_specs(profile: Mapping[str, Any]) -> Iterable[TopicSpec]:
    for camera_value in profile["cameras"]:
        camera = _mapping(camera_value, "camera")
        name = str(camera["name"])
        yield TopicSpec(
            key=f"{name}.image",
            topic=str(camera["image_topic"]),
            message_type=str(camera["image_type"]),
            category="camera_image",
            required_for_capture=True,
            required_for_planning=True,
            required_for_multitask=True,
        )
        yield TopicSpec(
            key=f"{name}.device_info",
            topic=str(camera["device_info_topic"]),
            message_type=str(camera["device_info_type"]),
            category="camera_device_info",
            required_for_capture=True,
            required_for_planning=True,
            required_for_multitask=True,
        )
        yield TopicSpec(
            key=f"{name}.camera_info",
            topic=str(camera["camera_info_topic"]),
            message_type=str(camera["camera_info_type"]),
            category="camera_info",
            required_for_capture=True,
            required_for_planning=True,
            required_for_multitask=True,
        )

    for key, entry_value in profile["runtime_topics"].items():
        entry = _mapping(entry_value, f"runtime_topics.{key}")
        yield TopicSpec(
            key=str(key),
            topic=str(entry["topic"]),
            message_type=str(entry["type"]),
            category="runtime",
            required_for_capture=True,
            required_for_planning=True,
            required_for_multitask=True,
        )

    for key, entry_value in profile["capture_topics"].items():
        entry = _mapping(entry_value, f"capture_topics.{key}")
        yield TopicSpec(
            key=str(key),
            topic=str(entry["topic"]),
            message_type=str(entry["type"]),
            category="capture_provenance",
            required_for_capture=True,
            required_for_planning=True,
            required_for_multitask=True,
        )

    for category, profile_key in (("context", "context_topics"), ("label", "label_topics")):
        for key, entry_value in profile.get(profile_key, {}).items():
            entry = _mapping(entry_value, f"{profile_key}.{key}")
            if entry.get("record", True) is False:
                continue
            requirements = set(entry.get("required_for", []))
            yield TopicSpec(
                key=str(key),
                topic=str(entry["topic"]),
                message_type=str(entry["type"]),
                category=category,
                required_for_capture=False,
                required_for_planning="planning" in requirements,
                required_for_multitask="multitask" in requirements,
            )


def unique_topic_specs(profile: Mapping[str, Any]) -> list[TopicSpec]:
    by_topic: dict[str, TopicSpec] = {}
    for spec in iter_topic_specs(profile):
        previous = by_topic.get(spec.topic)
        if previous and previous.message_type != spec.message_type:
            raise ProfileError(
                f"topic {spec.topic} has conflicting types: "
                f"{previous.message_type} and {spec.message_type}"
            )
        by_topic.setdefault(spec.topic, spec)
    return sorted(by_topic.values(), key=lambda item: item.topic)


def parse_ros2_topic_list(output: str) -> dict[str, list[str]]:
    topics: dict[str, list[str]] = {}
    for line in output.splitlines():
        match = re.fullmatch(r"\s*(/\S+)\s+\[([^]]+)]\s*", line)
        if not match:
            continue
        topics[match.group(1)] = [item.strip() for item in match.group(2).split(",")]
    return topics


def evaluate_live_topics(
    profile: Mapping[str, Any], discovered: Mapping[str, list[str]]
) -> dict[str, Any]:
    required_missing: list[str] = []
    required_type_mismatch: list[dict[str, Any]] = []
    optional_present: list[str] = []
    optional_missing: list[str] = []
    for spec in unique_topic_specs(profile):
        types = discovered.get(spec.topic)
        if not types:
            if spec.required_for_capture:
                required_missing.append(spec.topic)
            else:
                optional_missing.append(spec.topic)
            continue
        if spec.message_type not in types:
            mismatch = {"topic": spec.topic, "expected": spec.message_type, "actual": types}
            if spec.required_for_capture:
                required_type_mismatch.append(mismatch)
            else:
                optional_missing.append(spec.topic)
            continue
        if not spec.required_for_capture:
            optional_present.append(spec.topic)
    return {
        "pass": not required_missing and not required_type_mismatch,
        "required_missing": sorted(required_missing),
        "required_type_mismatch": required_type_mismatch,
        "optional_present": sorted(optional_present),
        "optional_missing": sorted(set(optional_missing)),
    }


def query_live_topics() -> dict[str, list[str]]:
    result = subprocess.run(
        ["ros2", "topic", "list", "--show-types", "--no-daemon", "--spin-time", "1"],
        check=False,
        text=True,
        capture_output=True,
    )
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip()
        raise RuntimeError(f"ros2 topic discovery failed: {detail}")
    return parse_ros2_topic_list(result.stdout)


def wait_for_live_topics(profile: Mapping[str, Any], timeout_s: float) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    report: dict[str, Any] = {}
    while True:
        report = evaluate_live_topics(profile, query_live_topics())
        if report["pass"] or time.monotonic() >= deadline:
            return report
        time.sleep(min(1.0, max(0.0, deadline - time.monotonic())))


def inspect_live_activity(profile: Mapping[str, Any], timeout_s: float) -> dict[str, Any]:
    import rclpy
    from rosidl_runtime_py.utilities import get_message

    context = rclpy.context.Context()
    rclpy.init(args=None, context=context)
    node = rclpy.create_node(
        f"vad_capture_preflight_{int(time.time_ns() % 1_000_000_000)}", context=context
    )
    required = [spec for spec in unique_topic_specs(profile) if spec.required_for_capture]
    received: set[str] = set()
    subscriptions = []
    publishers: dict[str, Any] = {}
    discovery_deadline = time.monotonic() + min(timeout_s, 3.0)
    try:
        while time.monotonic() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            publishers = {
                spec.topic: node.get_publishers_info_by_topic(spec.topic) for spec in required
            }
            if all(publishers.values()):
                break

        for spec in required:
            endpoints = publishers.get(spec.topic, [])
            if not endpoints:
                continue
            qos = endpoints[0].qos_profile

            def callback(_message: Any, topic: str = spec.topic) -> None:
                received.add(topic)

            subscriptions.append(
                node.create_subscription(get_message(spec.message_type), spec.topic, callback, qos)
            )

        activity_deadline = time.monotonic() + timeout_s
        while time.monotonic() < activity_deadline and len(received) < len(required):
            rclpy.spin_once(node, timeout_sec=0.1)

        image_topics = {camera["image_topic"] for camera in profile["cameras"]}
        reliable_images = {
            topic: any(
                int(endpoint.qos_profile.reliability) == 1
                for endpoint in publishers.get(topic, [])
            )
            for topic in image_topics
        }
        tf_static_topic = profile["runtime_topics"]["tf_static"]["topic"]
        tf_static_transient = any(
            int(endpoint.qos_profile.durability) == 1
            for endpoint in publishers.get(tf_static_topic, [])
        )
        missing_publishers = sorted(
            spec.topic for spec in required if not publishers.get(spec.topic)
        )
        inactive = sorted(spec.topic for spec in required if spec.topic not in received)
        validation = profile["validation"]
        qos_pass = (
            (not validation["require_reliable_images"] or all(reliable_images.values()))
            and (
                not validation["require_transient_local_tf_static"] or tf_static_transient
            )
        )
        return {
            "pass": not missing_publishers and not inactive and qos_pass,
            "missing_publishers": missing_publishers,
            "inactive_required_topics": inactive,
            "reliable_image_publishers": dict(sorted(reliable_images.items())),
            "tf_static_transient_local": tf_static_transient,
            "qos_pass": qos_pass,
        }
    finally:
        subscriptions.clear()
        node.destroy_node()
        rclpy.shutdown(context=context)


def write_session_manifest(
    profile: Mapping[str, Any],
    output: Path,
    status: str,
    bag: Path | None,
    calibration: Mapping[str, Any] | None,
    commissioning: Mapping[str, Any] | None,
    hash_bag_files: bool,
) -> None:
    destination = output / "session_manifest.json"
    previous: dict[str, Any] = {}
    if destination.is_file():
        try:
            loaded = json.loads(destination.read_text(encoding="utf-8"))
            if isinstance(loaded, dict):
                previous = loaded
        except (OSError, json.JSONDecodeError):
            previous = {}
    now_ns = time.time_ns()
    artifacts = model_artifact_report(profile)
    if status == "validated" and not artifacts["pass"]:
        raise ProfileError("cannot mark a session validated with changed model artifacts")
    if status == "validated" and (
        commissioning is None or commissioning.get("_expired", True)
    ):
        raise ProfileError("cannot mark a session validated with expired commissioning")
    manifest = {
        "schema_version": 1,
        "status": status,
        "created_unix_ns": previous.get("created_unix_ns", now_ns),
        "updated_unix_ns": now_ns,
        "profile_path": profile.get("_profile_path"),
        "profile_sha256": profile.get("_profile_sha256"),
        "collection_profile": {
            "path": profile.get("_profile_path"),
            "sha256": profile.get("_profile_sha256"),
        },
        "source": profile["session"]["source"],
        "vehicle_id": profile["session"]["vehicle_id"],
        "sensor_rig_id": profile["session"].get("sensor_rig_id"),
        "session": profile["session"],
        "provenance": profile.get("provenance", {}),
        "camera_order": list(CAMERA_ORDER),
        "bag": str(bag) if bag else None,
        "model_artifacts": artifacts,
    }
    if calibration:
        manifest["calibration"] = {
            "path": calibration.get("_path"),
            "sha256": calibration.get("_sha256"),
            "camera_serials": {
                camera["name"]: camera["serial"] for camera in calibration["cameras"]
            },
        }
    if commissioning:
        manifest["commissioning"] = {
            "path": commissioning.get("_path"),
            "sha256": commissioning.get("_sha256"),
            "approved_by": commissioning.get("approved_by"),
            "approved_utc": commissioning.get("approved_utc"),
            "valid_until_utc": commissioning.get("valid_until_utc"),
            "expired_at_manifest_write": commissioning.get("_expired"),
            "checkerboard_dataset_sha256": commissioning.get(
                "checkerboard_dataset_sha256"
            ),
            "rectification_config_path": commissioning.get(
                "_rectification_config_path"
            ),
            "rectification_config_sha256": commissioning.get(
                "_rectification_config_sha256"
            ),
            "evidence_files": commissioning.get("_evidence_files", {}),
        }
    workspace = Path(__file__).resolve().parents[2]
    commit = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=workspace, check=False, text=True, capture_output=True
    )
    dirty = subprocess.run(
        ["git", "status", "--porcelain"], cwd=workspace, check=False, text=True, capture_output=True
    )
    manifest["software"] = {
        "git_commit": commit.stdout.strip() if commit.returncode == 0 else None,
        "git_worktree_dirty": bool(dirty.stdout.strip()) if dirty.returncode == 0 else None,
    }
    if bag and (bag / "metadata.yaml").is_file():
        manifest["metadata_sha256"] = hashlib.sha256((bag / "metadata.yaml").read_bytes()).hexdigest()
        metadata = yaml.safe_load((bag / "metadata.yaml").read_text(encoding="utf-8"))
        information = metadata.get("rosbag2_bagfile_information", {})
        previous_bag_files = {
            entry.get("path"): entry
            for entry in previous.get("bag_files", [])
            if isinstance(entry, Mapping) and isinstance(entry.get("path"), str)
        }
        manifest["bag_files"] = []
        for relative_path in information.get("relative_file_paths", []):
            path = bag / relative_path
            previous_hash = previous_bag_files.get(relative_path, {}).get("sha256")
            manifest["bag_files"].append(
                {
                    "path": relative_path,
                    "size_bytes": path.stat().st_size if path.is_file() else None,
                    "sha256": (
                        sha256_file(path)
                        if hash_bag_files and path.is_file()
                        else previous_hash
                    ),
                }
            )
    artifact_hashes = {}
    for name in ("training_data_report.json", "calibration_preview.png"):
        path = output / name
        if path.is_file():
            artifact_hashes[name] = sha256_file(path)
    manifest["session_artifact_sha256"] = artifact_hashes
    temporary = destination.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(destination)


def _print_preflight(report: Mapping[str, Any]) -> None:
    print(json.dumps(report, indent=2, sort_keys=True))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    check_parser = subparsers.add_parser("check", help="validate a collection profile")
    check_parser.add_argument("profile", type=Path)
    check_parser.add_argument("--capture-ready", action="store_true")

    calibration_parser = subparsers.add_parser(
        "calibration-path", help="resolve and validate the profile calibration manifest"
    )
    calibration_parser.add_argument("profile", type=Path)

    provenance_parser = subparsers.add_parser(
        "provenance-path", help="resolve a required profile provenance file"
    )
    provenance_parser.add_argument("profile", type=Path)
    provenance_parser.add_argument(
        "key", choices=("rectification_config_file", "commissioning_file")
    )

    evidence_parser = subparsers.add_parser(
        "commissioning-evidence-paths",
        help="print validated commissioning evidence paths",
    )
    evidence_parser.add_argument("profile", type=Path)

    artifacts_parser = subparsers.add_parser(
        "model-artifacts", help="verify the fixed deployment artifact hashes"
    )
    artifacts_parser.add_argument("profile", type=Path)

    topics_parser = subparsers.add_parser("topics", help="print recorder topics")
    topics_parser.add_argument("profile", type=Path)
    topics_parser.add_argument("--json", action="store_true")

    preflight_parser = subparsers.add_parser("preflight", help="check required live topics")
    preflight_parser.add_argument("profile", type=Path)
    preflight_parser.add_argument("--timeout", type=float, default=15.0)

    manifest_parser = subparsers.add_parser("manifest", help="write a capture session manifest")
    manifest_parser.add_argument("profile", type=Path)
    manifest_parser.add_argument("output", type=Path)
    manifest_parser.add_argument("--status", required=True)
    manifest_parser.add_argument("--bag", type=Path)
    manifest_parser.add_argument("--calibration", type=Path)
    manifest_parser.add_argument("--commissioning", type=Path)
    manifest_parser.add_argument("--rectification-config", type=Path)
    manifest_parser.add_argument("--hash-bag-files", action="store_true")

    args = parser.parse_args()
    try:
        profile = load_profile(args.profile)
        if args.command == "check":
            placeholders = find_placeholders(profile)
            calibration = None
            commissioning = None
            calibration_error = None
            artifacts = None
            if not placeholders:
                try:
                    calibration = load_profile_calibration(profile)
                    commissioning = load_profile_commissioning_contract(profile, calibration)
                    artifacts = model_artifact_report(profile)
                except (OSError, ValueError) as error:
                    calibration_error = str(error)
            capture_ready = (
                not placeholders
                and calibration is not None
                and calibration_error is None
                and commissioning is not None
                and not commissioning.get("_expired", True)
                and artifacts is not None
                and artifacts["pass"]
            )
            print(
                json.dumps(
                    {
                        "pass": capture_ready if args.capture_ready else True,
                        "capture_ready": capture_ready,
                        "placeholders": placeholders,
                        "calibration_error": calibration_error,
                        "calibration_sha256": calibration.get("_sha256") if calibration else None,
                        "commissioning_sha256": (
                            commissioning.get("_sha256") if commissioning else None
                        ),
                        "commissioning_expired": (
                            commissioning.get("_expired") if commissioning else None
                        ),
                        "model_artifacts": artifacts,
                        "profile": profile["_profile_path"],
                        "sha256": profile["_profile_sha256"],
                        "record_topic_count": len(unique_topic_specs(profile)),
                    },
                    indent=2,
                    sort_keys=True,
                )
            )
            return 0 if not args.capture_ready or capture_ready else 1
        if args.command == "calibration-path":
            calibration = load_profile_calibration(profile)
            print(calibration["_path"])
            return 0
        if args.command == "provenance-path":
            print(resolve_profile_provenance_file(profile, args.key))
            return 0
        if args.command == "commissioning-evidence-paths":
            calibration = load_profile_calibration(profile)
            commissioning = load_profile_commissioning_contract(profile, calibration)
            for name in sorted(commissioning["_evidence_files"]):
                print(commissioning["_evidence_files"][name]["path"])
            return 0
        if args.command == "model-artifacts":
            report = model_artifact_report(profile)
            print(json.dumps(report, indent=2, sort_keys=True))
            return 0 if report["pass"] else 1
        if args.command == "topics":
            specs = unique_topic_specs(profile)
            if args.json:
                print(json.dumps([spec.to_dict() for spec in specs], indent=2, sort_keys=True))
            else:
                for spec in specs:
                    print(spec.topic)
            return 0
        if args.command == "preflight":
            if args.timeout <= 0:
                raise ProfileError("preflight timeout must be positive")
            report = wait_for_live_topics(profile, args.timeout)
            if report["pass"]:
                activity = inspect_live_activity(profile, args.timeout)
                report["activity"] = activity
                report["pass"] = bool(activity["pass"])
            _print_preflight(report)
            return 0 if report["pass"] else 1
        if args.command == "manifest":
            args.output.mkdir(parents=True, exist_ok=True)
            calibration = (
                load_profile_calibration(profile)
                if args.calibration is None
                else load_calibration_file(args.calibration)
            )
            if args.commissioning is None and args.rectification_config is None:
                commissioning = load_profile_commissioning_contract(profile, calibration)
            elif args.commissioning is not None and args.rectification_config is not None:
                commissioning = load_commissioning_file(
                    args.commissioning,
                    calibration,
                    args.rectification_config.expanduser().resolve(),
                    profile,
                )
            else:
                raise ProfileError(
                    "--commissioning and --rectification-config must be provided together"
                )
            write_session_manifest(
                profile,
                args.output,
                args.status,
                args.bag,
                calibration,
                commissioning,
                args.hash_bag_files,
            )
            return 0
    except (OSError, ValueError, RuntimeError) as error:
        parser.error(str(error))
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
