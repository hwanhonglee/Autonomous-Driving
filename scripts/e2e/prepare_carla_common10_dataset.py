#!/usr/bin/env python3
"""Convert validated native CARLA expert episodes into one Common10 dataset.

The converter is intentionally fail closed.  It never invents steering angle or
speed-limit telemetry, never retimes camera frames, and only emits anchors with
the complete 6.4 second planning future required by ``common_10hz_v1``.
"""

from __future__ import annotations

import argparse
from bisect import bisect_right
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
from statistics import median
from collections import Counter
from typing import Any, Iterable, Mapping, Sequence

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from portable_e2e import contract as common10  # noqa: E402


IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:-]*$")
SPLITS = ("train", "val", "test")
ADAPTER_ID = "carla_native_expert_to_common10_v1"
CAPTURE_PHASES = ("stationary_warmup", "driving", "stationary_tail")
ANCHOR_PHASES = frozenset(("stationary_warmup", "driving"))


class AdapterError(RuntimeError):
    """Raised when a native episode cannot be converted without fabrication."""


@dataclass(frozen=True)
class EpisodeInput:
    split: str
    path: Path


@dataclass(frozen=True)
class NativeEpisode:
    spec: EpisodeInput
    manifest: Mapping[str, Any]
    route: Mapping[str, Any]
    states: tuple[Mapping[str, Any], ...]
    camera_frames: tuple[Mapping[str, Any], ...]
    source_hashes: Mapping[str, str]
    episode_id: str
    route_id: str
    site_id: str
    weather: str
    seed: int


def _parse_episode(value: str) -> EpisodeInput:
    split, separator, raw_path = value.partition("=")
    if not separator or split not in SPLITS or not raw_path:
        raise argparse.ArgumentTypeError(
            "episode must use train=/path, val=/path, or test=/path"
        )
    return EpisodeInput(split=split, path=Path(raw_path).expanduser().resolve())


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--episode",
        action="append",
        required=True,
        type=_parse_episode,
        help="repeatable split=/absolute/or/relative/native-episode path",
    )
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--dataset-id", required=True)
    parser.add_argument(
        "--source-dataset-version",
        default="carla-0.9.15-native-expert-v1",
    )
    parser.add_argument(
        "--license-id",
        required=True,
        help="reviewed identifier for generated simulator data; never inferred",
    )
    parser.add_argument(
        "--image-mode",
        choices=("copy", "hardlink"),
        default="copy",
        help="hardlink is local-only and fails rather than silently falling back",
    )
    parser.add_argument(
        "--git-commit",
        help="40-hex adapter commit; default requires a clean repository HEAD",
    )
    parser.add_argument(
        "--validate-mode",
        choices=("planning", "runtime"),
        default="planning",
    )
    args = parser.parse_args(argv)
    for label in ("dataset_id", "source_dataset_version", "license_id"):
        value = getattr(args, label)
        if not isinstance(value, str) or not value.strip():
            parser.error(f"--{label.replace('_', '-')} must be non-empty")
    if not IDENTIFIER.fullmatch(args.dataset_id):
        parser.error("--dataset-id contains unsafe characters")
    if len({item.path for item in args.episode}) != len(args.episode):
        parser.error("--episode paths must be unique")
    return args


def _loads_json(text: str, context: str) -> Any:
    def unique_object(pairs: Sequence[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise AdapterError(f"{context}: duplicate JSON key {key!r}")
            result[key] = value
        return result

    try:
        return json.loads(
            text,
            object_pairs_hook=unique_object,
            parse_constant=lambda token: (_ for _ in ()).throw(
                AdapterError(f"{context}: non-standard JSON number {token!r}")
            ),
        )
    except json.JSONDecodeError as error:
        raise AdapterError(f"{context}: invalid JSON: {error}") from error


def _json(path: Path) -> dict[str, Any]:
    try:
        value = _loads_json(path.read_text(encoding="utf-8"), str(path))
    except OSError as error:
        raise AdapterError(f"cannot read JSON {path}: {error}") from error
    if not isinstance(value, dict):
        raise AdapterError(f"{path} must contain a JSON object")
    return value


def _jsonl(path: Path) -> tuple[dict[str, Any], ...]:
    records: list[dict[str, Any]] = []
    try:
        for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            if not line.strip():
                continue
            value = _loads_json(line, f"{path}:{line_number}")
            if not isinstance(value, dict):
                raise AdapterError(f"{path}:{line_number} must contain an object")
            records.append(value)
    except OSError as error:
        raise AdapterError(f"cannot read JSONL {path}: {error}") from error
    if not records:
        raise AdapterError(f"{path} contains no records")
    return tuple(records)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while block := stream.read(1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def _write_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _write_jsonl(path: Path, values: Iterable[Mapping[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        for value in values:
            stream.write(
                json.dumps(value, separators=(",", ":"), ensure_ascii=False, allow_nan=False)
                + "\n"
            )


def _number(value: Any, context: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise AdapterError(f"{context} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise AdapterError(f"{context} must be finite")
    if minimum is not None and result < minimum:
        raise AdapterError(f"{context} must be at least {minimum}")
    return result


def _integer(value: Any, context: str, *, minimum: int | None = None) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise AdapterError(f"{context} must be an integer")
    if minimum is not None and value < minimum:
        raise AdapterError(f"{context} must be at least {minimum}")
    return value


def _timestamp_ns(seconds: Any, context: str) -> int:
    value = _number(seconds, context, minimum=0.0)
    result = int(round(value * 1_000_000_000.0))
    if result < 0:
        raise AdapterError(f"{context} produced a negative timestamp")
    return result


def _safe_source_file(root: Path, value: Any, context: str) -> Path:
    if not isinstance(value, str) or not value:
        raise AdapterError(f"{context} must be a non-empty relative path")
    relative = Path(value)
    if relative.is_absolute() or ".." in relative.parts:
        raise AdapterError(f"{context} must stay inside the native episode")
    path = root / relative
    if path.is_symlink() or not path.is_file() or path.stat().st_size <= 0:
        raise AdapterError(f"{context} is missing, empty, or a symlink: {path}")
    try:
        path.resolve().relative_to(root.resolve())
    except ValueError as error:
        raise AdapterError(f"{context} escapes the native episode") from error
    return path


def _identifier(value: str, fallback: str) -> str:
    normalized = re.sub(r"[^A-Za-z0-9_.:-]+", "_", value).strip("_.:-")
    return normalized or fallback


def _require_close(
    observed: Any,
    expected: float,
    context: str,
    *,
    absolute_tolerance: float = 1.0e-9,
) -> float:
    value = _number(observed, context)
    if not math.isclose(
        value,
        float(expected),
        rel_tol=0.0,
        abs_tol=absolute_tolerance,
    ):
        raise AdapterError(
            f"{context}={value!r} does not match recomputed value {expected!r}"
        )
    return value


def _front_steering_from_carla_degrees(
    front_left_carla_deg: float,
    front_right_carla_deg: float,
) -> tuple[float, float, float]:
    values = (front_left_carla_deg, front_right_carla_deg)
    if not all(math.isfinite(value) for value in values):
        raise AdapterError("measured front-wheel steering values must be finite")
    left_ros = -math.radians(front_left_carla_deg)
    right_ros = -math.radians(front_right_carla_deg)
    if max(abs(left_ros), abs(right_ros)) >= math.pi / 2.0:
        raise AdapterError("measured front-wheel steering must stay within 90 degrees")
    left_tangent = math.tan(left_ros)
    right_tangent = math.tan(right_ros)
    straight_tolerance = math.tan(math.radians(0.1))
    if max(abs(left_tangent), abs(right_tangent)) <= straight_tolerance:
        virtual = 0.5 * (left_ros + right_ros)
    else:
        if left_tangent * right_tangent <= 0.0:
            raise AdapterError("measured front wheels have inconsistent steering directions")
        virtual = math.atan(
            2.0 * left_tangent * right_tangent / (left_tangent + right_tangent)
        )
    return left_ros, right_ros, virtual


def _native_route_id(route: Mapping[str, Any]) -> str:
    town = route.get("town")
    scenario = route.get("scenario")
    alignment = route.get("coordinate_alignment")
    if not isinstance(town, str) or not town or not isinstance(scenario, str) or not scenario:
        raise AdapterError("native route lacks town/scenario identity")
    if not isinstance(alignment, Mapping):
        raise AdapterError("native route lacks coordinate_alignment provenance")
    source_route = alignment.get("source_route")
    source_hash = alignment.get("source_route_sha256")
    if not isinstance(source_route, str) or not source_route:
        raise AdapterError("native route coordinate_alignment lacks source_route")
    if not isinstance(source_hash, str) or re.fullmatch(r"[0-9a-f]{64}", source_hash) is None:
        raise AdapterError("native route coordinate_alignment lacks a source route SHA-256")
    route_id = Path(source_route).stem
    expected = re.compile(
        rf"^{re.escape(town.lower())}_{re.escape(scenario.lower())}_s[0-9]{{4}}_p[0-9]{{2}}$"
    )
    if expected.fullmatch(route_id.lower()) is None:
        raise AdapterError(
            "native catalog route identity does not match town/scenario: "
            f"{route_id!r}"
        )
    return _identifier(route_id, "route")


def _git_commit(explicit: str | None) -> str:
    if explicit is not None:
        if re.fullmatch(r"[0-9a-f]{40}", explicit) is None:
            raise AdapterError("--git-commit must contain exactly 40 lowercase hex digits")
        return explicit
    status = subprocess.run(
        ["git", "status", "--porcelain"],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    if status.returncode != 0:
        raise AdapterError(f"cannot inspect repository status: {status.stderr.strip()}")
    if status.stdout.strip():
        raise AdapterError(
            "repository is dirty; commit the adapter or pass an explicit reviewed --git-commit"
        )
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    commit = result.stdout.strip()
    if result.returncode != 0 or re.fullmatch(r"[0-9a-f]{40}", commit) is None:
        raise AdapterError("cannot resolve a 40-hex repository HEAD")
    return commit


def _load_native(spec: EpisodeInput) -> NativeEpisode:
    root = spec.path
    if root.is_symlink() or not root.is_dir():
        raise AdapterError(f"native episode is not a real directory: {root}")
    paths = {
        name: root / name
        for name in ("manifest.json", "route.json", "states.jsonl", "camera_frames.jsonl")
    }
    if any(path.is_symlink() or not path.is_file() for path in paths.values()):
        raise AdapterError(f"native episode is incomplete or contains symlinked metadata: {root}")
    manifest = _json(paths["manifest.json"])
    route = _json(paths["route.json"])
    states = _jsonl(paths["states.jsonl"])
    frames = _jsonl(paths["camera_frames.jsonl"])
    hashes = {name: _sha256(path) for name, path in paths.items()}
    if manifest.get("status") != "complete":
        raise AdapterError(f"native episode is not complete: {root}")
    cleanup = manifest.get("cleanup")
    if not isinstance(cleanup, Mapping) or cleanup.get("completed") is not True:
        raise AdapterError(f"native episode cleanup did not complete: {root}")
    result = manifest.get("result")
    if not isinstance(result, Mapping) or result.get("goal_reached") is not True:
        raise AdapterError(f"native episode did not reach the catalog goal: {root}")
    if _integer(result.get("collision_event_count"), "collision count", minimum=0):
        raise AdapterError(f"expert episode contains a collision: {root}")
    if _integer(result.get("lane_invasion_event_count"), "lane invasion count", minimum=0):
        raise AdapterError(f"expert episode contains a lane invasion: {root}")
    capture = manifest.get("capture_contract")
    if not isinstance(capture, Mapping):
        raise AdapterError(f"native episode lacks capture_contract: {root}")
    camera_hz = _number(capture.get("camera_hz"), "camera_hz")
    physics_hz = _number(capture.get("physics_hz"), "physics_hz")
    if not math.isclose(camera_hz, 10.0, rel_tol=0.0, abs_tol=1.0e-12):
        raise AdapterError(f"native episode camera_hz is not 10: {root}")
    if physics_hz < 10.0:
        raise AdapterError(f"native episode physics_hz is below 10: {root}")
    ratio = physics_hz / camera_hz
    interval = int(round(ratio))
    if interval < 1 or not math.isclose(ratio, interval, rel_tol=0.0, abs_tol=1.0e-12):
        raise AdapterError("native physics_hz must be an integer multiple of camera_hz")
    if _integer(capture.get("camera_interval_ticks"), "camera_interval_ticks", minimum=1) != interval:
        raise AdapterError("native camera_interval_ticks does not match physics/camera rate")
    if tuple(capture.get("camera_order", ())) != common10.CAMERA_ORDER:
        raise AdapterError(f"native episode camera order is incompatible: {root}")
    if capture.get("image_encoding") != "jpeg_bgr8":
        raise AdapterError("native episode image encoding is not jpeg_bgr8")
    jpeg_quality = _integer(capture.get("jpeg_quality"), "jpeg_quality", minimum=1)
    if jpeg_quality > 100:
        raise AdapterError("native jpeg_quality exceeds 100")
    if capture.get("capture_phase_field") != "capture_phase":
        raise AdapterError(
            f"native episode does not declare the capture_phase field: {root}"
        )
    schedule = capture.get("capture_phase_schedule")
    if not isinstance(schedule, Mapping) or tuple(schedule.get("order", ())) != CAPTURE_PHASES:
        raise AdapterError("native capture phase schedule is absent or out of order")
    measurement_contract = capture.get("measured_vehicle_state")
    expected_measurement_contract = {
        "steering_source": "carla.Vehicle.get_wheel_steer_angle(FL_Wheel,FR_Wheel)",
        "steering_source_unit": "degree",
        "carla_steering_sign": "positive-right",
        "ros_steering_sign": "positive-left",
        "wheel_conversion": "ros_rad=-radians(carla_deg)",
        "virtual_tire_conversion": "Ackermann harmonic mean of wheel tangents",
        "normalized_vehicle_control_steer_used_as_angle": False,
        "speed_limit_source": "carla.Vehicle.get_speed_limit",
        "speed_limit_source_unit": "km/h",
        "speed_limit_output_unit": "m/s",
        "speed_limit_conversion": "m/s=(km/h)/3.6",
    }
    if not isinstance(measurement_contract, Mapping) or any(
        measurement_contract.get(key) != value
        for key, value in expected_measurement_contract.items()
    ):
        raise AdapterError("native measured vehicle-state provenance is incomplete")
    coordinate_contract = manifest.get("coordinate_contract")
    measured_fields = (
        coordinate_contract.get("measured_vehicle_fields")
        if isinstance(coordinate_contract, Mapping)
        else None
    )
    expected_measured_fields = {
        "front_left_wheel_steer_angle_carla_deg",
        "front_right_wheel_steer_angle_carla_deg",
        "front_left_wheel_steer_angle_ros_rad",
        "front_right_wheel_steer_angle_ros_rad",
        "steering_tire_angle_rad",
        "speed_limit_carla_kmh",
        "speed_limit_mps",
    }
    if not isinstance(measured_fields, list) or set(measured_fields) != expected_measured_fields:
        raise AdapterError("native coordinate contract omits measured vehicle fields")
    if _integer(result.get("state_count"), "result.state_count", minimum=1) != len(states):
        raise AdapterError(f"native result.state_count does not match states.jsonl: {root}")
    if (
        _integer(result.get("camera_anchor_count"), "result.camera_anchor_count", minimum=1)
        != len(frames)
    ):
        raise AdapterError(
            f"native result.camera_anchor_count does not match camera_frames.jsonl: {root}"
        )
    town = route.get("town")
    scenario = route.get("scenario")
    route_weather = route.get("weather")
    runtime = manifest.get("runtime")
    if not isinstance(runtime, Mapping):
        raise AdapterError("native manifest lacks runtime metadata")
    weather = runtime.get("weather")
    if not all(
        isinstance(value, str) and value
        for value in (town, scenario, route_weather, weather)
    ):
        raise AdapterError(f"native route lacks town/scenario/weather: {root}")
    if runtime.get("town") != town or weather != route_weather:
        raise AdapterError("native route and runtime town/weather disagree")
    for key in ("server_version", "vehicle_type"):
        if not isinstance(runtime.get(key), str) or not runtime[key]:
            raise AdapterError(f"native runtime.{key} must be a non-empty string")
    world_settings = runtime.get("capture_world_settings")
    if not isinstance(world_settings, Mapping) or world_settings.get("synchronous_mode") is not True:
        raise AdapterError("native capture was not declared synchronous")
    _require_close(
        world_settings.get("fixed_delta_seconds"),
        1.0 / physics_hz,
        "runtime.capture_world_settings.fixed_delta_seconds",
    )
    provenance = manifest.get("provenance")
    if not isinstance(provenance, Mapping):
        raise AdapterError("native manifest lacks provenance")
    if provenance.get("route_sha256") != hashes["route.json"]:
        raise AdapterError("native provenance route SHA-256 does not match route.json")
    route_id = _native_route_id(route)
    seed = _integer(capture.get("seed"), "capture seed", minimum=0)
    episode_token = hashlib.sha256(
        (hashes["manifest.json"] + hashes["states.jsonl"]).encode("ascii")
    ).hexdigest()[:12]
    episode_id = _identifier(
        f"carla_{str(town).lower()}_{route_id}_{str(weather)}_s{seed:04d}_{episode_token}",
        f"carla_{episode_token}",
    )
    return NativeEpisode(
        spec=spec,
        manifest=manifest,
        route=route,
        states=states,
        camera_frames=frames,
        source_hashes=hashes,
        episode_id=episode_id,
        route_id=route_id,
        site_id=str(town),
        weather=str(weather),
        seed=seed,
    )


def _matmul3(left: Sequence[Sequence[float]], right: Sequence[Sequence[float]]) -> list[list[float]]:
    return [
        [sum(left[row][k] * right[k][column] for k in range(3)) for column in range(3)]
        for row in range(3)
    ]


def _optical_transform(extrinsic: Mapping[str, Any]) -> list[float]:
    roll = _number(extrinsic.get("roll"), "camera roll")
    pitch = _number(extrinsic.get("pitch"), "camera pitch")
    yaw = _number(extrinsic.get("yaw"), "camera yaw")
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    base_from_link = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    link_from_optical = [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
    rotation = _matmul3(base_from_link, link_from_optical)
    translation = [
        _number(extrinsic.get("x"), "camera x"),
        _number(extrinsic.get("y"), "camera y"),
        _number(extrinsic.get("z"), "camera z"),
    ]
    return [
        rotation[0][0], rotation[0][1], rotation[0][2], translation[0],
        rotation[1][0], rotation[1][1], rotation[1][2], translation[1],
        rotation[2][0], rotation[2][1], rotation[2][2], translation[2],
        0.0, 0.0, 0.0, 1.0,
    ]


def _rig_document(native: NativeEpisode) -> dict[str, Any]:
    raw_cameras = native.manifest.get("cameras")
    if not isinstance(raw_cameras, list) or len(raw_cameras) != len(common10.CAMERA_ORDER):
        raise AdapterError("native manifest must contain exactly six cameras")
    cameras: list[dict[str, Any]] = []
    for index, (name, raw) in enumerate(zip(common10.CAMERA_ORDER, raw_cameras)):
        if not isinstance(raw, Mapping) or raw.get("name") != name or raw.get("model_index") != index:
            raise AdapterError(f"native camera {index} violates the fixed Common10 order")
        if raw.get("width") != 640 or raw.get("height") != 360:
            raise AdapterError(f"native camera {name} is not 640x360")
        if raw.get("enable_postprocess_effects") is not False:
            raise AdapterError(f"native camera {name} did not disable postprocess effects")
        _require_close(raw.get("sensor_tick"), 0.0, f"{name}.sensor_tick")
        frame_id = raw.get("frame_id")
        if not isinstance(frame_id, str) or not frame_id:
            raise AdapterError(f"native camera {name} lacks a frame_id")
        intrinsic = raw.get("intrinsic")
        extrinsic = raw.get("ros_extrinsic")
        if not isinstance(intrinsic, Mapping) or not isinstance(extrinsic, Mapping):
            raise AdapterError(f"native camera {name} lacks calibration")
        fx = _number(intrinsic.get("fx"), f"{name}.fx", minimum=1.0)
        fy = _number(intrinsic.get("fy"), f"{name}.fy", minimum=1.0)
        cx = _number(intrinsic.get("cx"), f"{name}.cx", minimum=0.0)
        cy_value = _number(intrinsic.get("cy"), f"{name}.cy", minimum=0.0)
        cameras.append(
            {
                "name": name,
                "model_index": index,
                "physical_id": f"virtual:carla:{name}",
                "optical_frame": frame_id,
                "width_px": 640,
                "height_px": 360,
                "source_encoding": "bgr8",
                "storage_codec": "jpeg",
                "K": [fx, 0.0, cx, 0.0, fy, cy_value, 0.0, 0.0, 1.0],
                "D": [],
                "horizontal_fov_rad": 2.0 * math.atan(640.0 / (2.0 * fx)),
                "T_base_from_camera": _optical_transform(extrinsic),
                "timestamp_source": "carla_simulation_frame",
                "trigger_mode": "synchronous_world_tick",
                "firmware": str(native.manifest["runtime"]["server_version"]),
                "exposure_mode": "virtual_postprocess_disabled",
            }
        )
    rig_body = {
        "domain": "carla",
        "base_frame": "base_link",
        "rectified": True,
        "camera_order": list(common10.CAMERA_ORDER),
        "cameras": cameras,
    }
    fingerprint = hashlib.sha256(
        json.dumps(rig_body, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()[:16]
    return {"schema_id": common10.RIG_SCHEMA_ID, "rig_id": f"carla_rig_{fingerprint}", **rig_body}


def _state_timeline(native: NativeEpisode) -> tuple[tuple[int, Mapping[str, Any]], ...]:
    timeline: list[tuple[int, Mapping[str, Any]]] = []
    last = -1
    last_frame = -1
    last_phase_index = 0
    phase_counts: Counter[str] = Counter()
    physics_hz = _number(native.manifest["capture_contract"].get("physics_hz"), "physics_hz")
    expected_gap_ns = 1_000_000_000.0 / physics_hz
    gap_tolerance_ns = max(10.0, expected_gap_ns * 1.0e-5)
    for index, state in enumerate(native.states):
        stamp = _timestamp_ns(state.get("timestamp"), f"states[{index}].timestamp")
        if stamp <= last:
            raise AdapterError("native state timestamps must be strictly increasing")
        frame = _integer(state.get("frame"), f"states[{index}].frame", minimum=0)
        if index and frame != last_frame + 1:
            raise AdapterError("native state frame counters must be contiguous")
        if index and abs((stamp - last) - expected_gap_ns) > gap_tolerance_ns:
            raise AdapterError("native state cadence does not match declared physics_hz")
        for field in ("x", "y", "z", "yaw", "vx", "vy", "ax", "ay", "yaw_rate"):
            _number(state.get(field), f"states[{index}].{field}")
        left_carla = _number(
            state.get("front_left_wheel_steer_angle_carla_deg"),
            f"states[{index}].front_left_wheel_steer_angle_carla_deg",
        )
        right_carla = _number(
            state.get("front_right_wheel_steer_angle_carla_deg"),
            f"states[{index}].front_right_wheel_steer_angle_carla_deg",
        )
        left_ros, right_ros, virtual = _front_steering_from_carla_degrees(
            left_carla, right_carla
        )
        _require_close(
            state.get("front_left_wheel_steer_angle_ros_rad"),
            left_ros,
            f"states[{index}].front_left_wheel_steer_angle_ros_rad",
        )
        _require_close(
            state.get("front_right_wheel_steer_angle_ros_rad"),
            right_ros,
            f"states[{index}].front_right_wheel_steer_angle_ros_rad",
        )
        _require_close(
            state.get("steering_tire_angle_rad"),
            virtual,
            f"states[{index}].steering_tire_angle_rad",
        )
        speed_limit_kmh = _number(
            state.get("speed_limit_carla_kmh"),
            f"states[{index}].speed_limit_carla_kmh",
            minimum=0.0,
        )
        _require_close(
            state.get("speed_limit_mps"),
            speed_limit_kmh / 3.6,
            f"states[{index}].speed_limit_mps",
        )
        command = _integer(state.get("command"), f"states[{index}].command", minimum=0)
        if command > 5:
            raise AdapterError(f"states[{index}].command exceeds 5")
        _number(state.get("route_progress_m"), f"states[{index}].route_progress_m", minimum=0.0)
        _number(state.get("route_cte_m"), f"states[{index}].route_cte_m", minimum=0.0)
        for event_name in ("collision", "lane_invasion"):
            events = state.get(event_name)
            if not isinstance(events, list):
                raise AdapterError(f"states[{index}].{event_name} must be a list")
            if events:
                raise AdapterError(
                    f"states[{index}].{event_name} is non-empty in an expert episode"
                )
        phase = state.get("capture_phase")
        if phase not in CAPTURE_PHASES:
            raise AdapterError(
                f"states[{index}].capture_phase must be one of {CAPTURE_PHASES}"
            )
        phase_index = CAPTURE_PHASES.index(str(phase))
        if phase_index < last_phase_index:
            raise AdapterError("native state capture phases move backwards")
        last_phase_index = phase_index
        phase_counts[str(phase)] += 1
        timeline.append((stamp, state))
        last = stamp
        last_frame = frame

    if any(phase_counts[phase] <= 0 for phase in CAPTURE_PHASES):
        raise AdapterError("native state timeline must contain all three capture phases")
    result = native.manifest["result"]
    ledger = result.get("capture_phases")
    if not isinstance(ledger, Mapping) or tuple(ledger.get("order", ())) != CAPTURE_PHASES:
        raise AdapterError("native result capture phase ledger is absent or out of order")
    frame_phase_counts = Counter(
        str(frame.get("capture_phase")) for frame in native.camera_frames
    )
    schedule = native.manifest["capture_contract"]["capture_phase_schedule"]
    for phase in CAPTURE_PHASES:
        phase_ledger = ledger.get(phase)
        if not isinstance(phase_ledger, Mapping):
            raise AdapterError(f"native result lacks the {phase} phase ledger")
        if _integer(phase_ledger.get("state_count"), f"{phase}.state_count", minimum=0) != phase_counts[phase]:
            raise AdapterError(f"native {phase} state ledger does not match records")
        if _integer(
            phase_ledger.get("camera_anchor_count"),
            f"{phase}.camera_anchor_count",
            minimum=0,
        ) != frame_phase_counts[phase]:
            raise AdapterError(f"native {phase} camera ledger does not match records")
        phase_schedule = schedule.get(phase)
        if not isinstance(phase_schedule, Mapping):
            raise AdapterError(f"native capture schedule lacks {phase}")
        if phase in ("stationary_warmup", "stationary_tail"):
            scheduled_ticks = _integer(
                phase_schedule.get("scheduled_ticks"),
                f"capture schedule {phase}.scheduled_ticks",
                minimum=1,
            )
            if scheduled_ticks != phase_counts[phase]:
                raise AdapterError(f"native {phase} record count differs from schedule")
        else:
            maximum_ticks = _integer(
                phase_schedule.get("maximum_ticks"),
                "capture schedule driving.maximum_ticks",
                minimum=1,
            )
            if phase_counts[phase] > maximum_ticks:
                raise AdapterError("native driving record count exceeds schedule")
    if _integer(ledger.get("observed_total_ticks"), "observed_total_ticks", minimum=1) != len(timeline):
        raise AdapterError("native observed total tick count does not match state records")
    _require_close(
        ledger.get("observed_total_elapsed_sim_sec"),
        len(timeline) / physics_hz,
        "observed_total_elapsed_sim_sec",
        absolute_tolerance=1.0e-8,
    )

    route_length = _number(native.route.get("route_length_m"), "route.route_length_m", minimum=0.0)
    route_points = native.route.get("route")
    if not isinstance(route_points, list) or not route_points:
        raise AdapterError("native route has no route points")
    last_route_point = route_points[-1]
    if not isinstance(last_route_point, Mapping):
        raise AdapterError("native terminal route point is malformed")
    _require_close(
        last_route_point.get("distance_m"),
        route_length,
        "terminal route distance_m",
        absolute_tolerance=1.0e-6,
    )
    _require_close(
        result.get("route_length_m"),
        route_length,
        "result.route_length_m",
        absolute_tolerance=1.0e-6,
    )
    driving_final = next(
        state for _stamp, state in reversed(timeline) if state["capture_phase"] == "driving"
    )
    final_progress = _number(
        result.get("final_route_progress_m"), "result.final_route_progress_m", minimum=0.0
    )
    _require_close(
        driving_final.get("route_progress_m"),
        final_progress,
        "terminal driving route progress",
        absolute_tolerance=1.0e-6,
    )
    post_progress = _number(
        result.get("post_capture_route_progress_m"),
        "result.post_capture_route_progress_m",
        minimum=0.0,
    )
    if final_progress > route_length + 1.0e-6 or post_progress > route_length + 1.0e-6:
        raise AdapterError("native route progress exceeds the declared route length")
    _require_close(
        timeline[-1][1].get("route_progress_m"),
        post_progress,
        "terminal capture route progress",
        absolute_tolerance=1.0e-6,
    )
    remaining = _number(
        result.get("final_remaining_route_m"),
        "result.final_remaining_route_m",
        minimum=0.0,
    )
    _require_close(
        remaining,
        max(0.0, route_length - final_progress),
        "result.final_remaining_route_m",
        absolute_tolerance=1.0e-6,
    )
    goal_tolerance = _number(
        result.get("goal_tolerance_m"), "result.goal_tolerance_m", minimum=0.0
    )
    if goal_tolerance <= 0.0:
        raise AdapterError("native goal tolerance must be positive")
    _require_close(
        native.manifest["capture_contract"].get("goal_tolerance_m"),
        goal_tolerance,
        "capture_contract.goal_tolerance_m",
        absolute_tolerance=1.0e-9,
    )
    goal_distance = _number(
        result.get("final_catalog_goal_distance_m"),
        "result.final_catalog_goal_distance_m",
        minimum=0.0,
    )
    if remaining > goal_tolerance + 1.0e-6 or goal_distance > goal_tolerance + 1.0e-6:
        raise AdapterError("native goal_reached contradicts the configured goal tolerance")
    goal_pose = native.route.get("goal_ros_pose")
    if not isinstance(goal_pose, Mapping):
        raise AdapterError("native route lacks goal_ros_pose")
    _require_close(
        last_route_point.get("x"),
        _number(goal_pose.get("x"), "goal_ros_pose.x"),
        "terminal route goal x",
        absolute_tolerance=1.0e-5,
    )
    _require_close(
        last_route_point.get("y"),
        _number(goal_pose.get("y"), "goal_ros_pose.y"),
        "terminal route goal y",
        absolute_tolerance=1.0e-5,
    )
    recomputed_goal_distance = math.hypot(
        float(driving_final["x"]) - _number(goal_pose.get("x"), "goal_ros_pose.x"),
        float(driving_final["y"]) - _number(goal_pose.get("y"), "goal_ros_pose.y"),
    )
    _require_close(
        goal_distance,
        recomputed_goal_distance,
        "result.final_catalog_goal_distance_m",
        absolute_tolerance=1.0e-5,
    )
    return tuple(timeline)


def _interpolate_state(
    timeline: Sequence[tuple[int, Mapping[str, Any]]], target_ns: int
) -> dict[str, float] | None:
    stamps = [item[0] for item in timeline]
    upper = bisect_right(stamps, target_ns)
    if upper and stamps[upper - 1] == target_ns:
        state = timeline[upper - 1][1]
        return {key: float(state[key]) for key in ("x", "y", "z", "yaw", "vx", "vy")}
    if upper == 0 or upper == len(timeline):
        return None
    first_ns, first = timeline[upper - 1]
    second_ns, second = timeline[upper]
    ratio = (target_ns - first_ns) / (second_ns - first_ns)
    yaw_delta = math.atan2(
        math.sin(float(second["yaw"]) - float(first["yaw"])),
        math.cos(float(second["yaw"]) - float(first["yaw"])),
    )
    result = {
        key: float(first[key]) + ratio * (float(second[key]) - float(first[key]))
        for key in ("x", "y", "z", "vx", "vy")
    }
    result["yaw"] = math.atan2(
        math.sin(float(first["yaw"]) + ratio * yaw_delta),
        math.cos(float(first["yaw"]) + ratio * yaw_delta),
    )
    return result


def _causal_state(
    timeline: Sequence[tuple[int, Mapping[str, Any]]], anchor_ns: int
) -> tuple[int, Mapping[str, Any]]:
    stamps = [item[0] for item in timeline]
    index = bisect_right(stamps, anchor_ns) - 1
    if index < 0:
        raise AdapterError("camera anchor precedes the first native ego state")
    return timeline[index]


def _relative_xy_yaw(
    anchor: Mapping[str, Any], future: Mapping[str, float]
) -> tuple[list[float], float]:
    dx = float(future["x"]) - float(anchor["x"])
    dy = float(future["y"]) - float(anchor["y"])
    yaw = float(anchor["yaw"])
    cosine, sine = math.cos(yaw), math.sin(yaw)
    xy = [cosine * dx + sine * dy, -sine * dx + cosine * dy]
    relative_yaw = math.atan2(
        math.sin(float(future["yaw"]) - yaw), math.cos(float(future["yaw"]) - yaw)
    )
    return xy, relative_yaw


def _copy_image(source: Path, destination: Path, mode: str) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        raise AdapterError(f"prepared image path already exists: {destination}")
    if mode == "hardlink":
        os.link(source, destination, follow_symlinks=False)
    else:
        shutil.copy2(source, destination, follow_symlinks=False)


def _scenario_tags(scenario: str) -> list[str]:
    normalized = scenario.lower()
    if normalized == "straight":
        return ["lane_follow", "straight"]
    if normalized in ("left", "right"):
        return ["lane_follow", "turn", normalized]
    if normalized == "lane_follow":
        return ["lane_follow"]
    raise AdapterError(f"unsupported native route scenario: {scenario!r}")


def _created_at(native: Sequence[NativeEpisode]) -> str:
    values: list[datetime] = []
    for item in native:
        raw = item.manifest.get("created_at")
        if not isinstance(raw, str):
            raise AdapterError("native manifest created_at must be an ISO timestamp")
        try:
            value = datetime.fromisoformat(raw.replace("Z", "+00:00"))
        except ValueError as error:
            raise AdapterError(f"invalid native created_at: {raw!r}") from error
        if value.tzinfo is None:
            raise AdapterError("native created_at must include a timezone")
        values.append(value.astimezone(timezone.utc))
    return min(values).isoformat().replace("+00:00", "Z")


def _prepare_episode(
    native: NativeEpisode,
    output_root: Path,
    rig: Mapping[str, Any],
    contract: Mapping[str, Any],
    *,
    source_dataset_version: str,
    license_id: str,
    image_mode: str,
    git_commit: str,
) -> dict[str, str]:
    episode_root = output_root / "episodes" / native.episode_id
    episode_root.mkdir(parents=True, exist_ok=False)
    route_points = native.route.get("route")
    if not isinstance(route_points, list) or len(route_points) < 2:
        raise AdapterError("native route must contain at least two points")
    polyline: list[list[float]] = []
    for index, point in enumerate(route_points):
        parsed = [
            _number(
                point.get("x") if isinstance(point, Mapping) else None,
                f"route[{index}].x",
            ),
            _number(
                point.get("y") if isinstance(point, Mapping) else None,
                f"route[{index}].y",
            ),
        ]
        if not polyline or math.dist(polyline[-1], parsed) > 1.0e-9:
            polyline.append(parsed)
    if len(polyline) < 2:
        raise AdapterError("native route must span at least one nonzero segment")
    route_geometry_path = episode_root / "route_geometry.json"
    _write_json(
        route_geometry_path,
        {"route_id": native.route_id, "frame_id": "map", "polyline_m": polyline},
    )
    route_hash = _sha256(route_geometry_path)
    timeline = _state_timeline(native)
    trajectory = contract["trajectory"]
    horizon_ns = int(round(float(trajectory["horizon_s"]) * 1_000_000_000.0))
    step_ns = int(round(float(trajectory["step_s"]) * 1_000_000_000.0))
    final_state_ns = timeline[-1][0]
    source_session_id = f"session:{native.episode_id}"
    route_source_ns = min(timeline[0][0], *(
        _timestamp_ns(frame.get("timestamp"), "camera frame timestamp")
        for frame in native.camera_frames
    ))
    raw_camera_hash = native.source_hashes["camera_frames.jsonl"]
    raw_route_hash = native.source_hashes["route.json"]
    raw_state_hash = native.source_hashes["states.jsonl"]
    raw_manifest_hash = native.source_hashes["manifest.json"]
    source_artifacts = [
        {"uri": f"source://{native.episode_id}/{name}", "sha256": digest}
        for name, digest in (
            ("manifest.json", raw_manifest_hash),
            ("route.json", raw_route_hash),
            ("states.jsonl", raw_state_hash),
            ("camera_frames.jsonl", raw_camera_hash),
        )
    ]
    raw_frames: list[dict[str, Any]] = []
    selected_sources: list[dict[str, Any]] = []
    samples: list[dict[str, Any]] = []
    ineligible: dict[str, int] = {}
    last_anchor = -1
    frame_counts = {name: 0 for name in common10.CAMERA_ORDER}
    last_camera_phase_index = 0
    last_carla_frame = -1
    expected_camera_interval = _integer(
        native.manifest["capture_contract"].get("camera_interval_ticks"),
        "camera_interval_ticks",
        minimum=1,
    )
    expected_jpeg_quality = _integer(
        native.manifest["capture_contract"].get("jpeg_quality"),
        "capture jpeg_quality",
        minimum=1,
    )
    state_by_frame = {
        _integer(state.get("frame"), "native state frame", minimum=0): (stamp, state)
        for stamp, state in timeline
    }
    seen_source_uris: set[str] = set()
    seen_source_files: set[tuple[int, int]] = set()
    for raw_index, frame in enumerate(native.camera_frames):
        if tuple(frame.get("camera_order", ())) != common10.CAMERA_ORDER:
            raise AdapterError(f"camera_frames[{raw_index}] violates camera order")
        images = frame.get("images")
        source_times = frame.get("source_timestamps")
        if not isinstance(images, Mapping) or tuple(images) != common10.CAMERA_ORDER:
            raise AdapterError(f"camera_frames[{raw_index}].images violates camera order")
        if not isinstance(source_times, Mapping) or tuple(source_times) != common10.CAMERA_ORDER:
            raise AdapterError(f"camera_frames[{raw_index}].source_timestamps violates order")
        phase = frame.get("capture_phase")
        if phase not in CAPTURE_PHASES:
            raise AdapterError(
                f"camera_frames[{raw_index}].capture_phase must be one of {CAPTURE_PHASES}"
            )
        phase_index = CAPTURE_PHASES.index(str(phase))
        if phase_index < last_camera_phase_index:
            raise AdapterError("native camera capture phases move backwards")
        last_camera_phase_index = phase_index
        camera_times = [
            _timestamp_ns(source_times[name], f"camera_frames[{raw_index}].{name}.timestamp")
            for name in common10.CAMERA_ORDER
        ]
        anchor_ns = int(median(camera_times))
        snapshot_ns = _timestamp_ns(
            frame.get("timestamp"), f"camera_frames[{raw_index}].timestamp"
        )
        if abs(snapshot_ns - anchor_ns) > 1:
            raise AdapterError(
                f"camera_frames[{raw_index}] snapshot time differs from camera median"
            )
        skew_ns = max(camera_times) - min(camera_times)
        maximum_skew_ns = int(
            round(float(contract["capture"]["dataset_bundle_skew_ms"]) * 1_000_000.0)
        )
        if skew_ns > maximum_skew_ns:
            raise AdapterError(f"camera_frames[{raw_index}] exceeds Common10 bundle skew")
        declared_span = _number(
            frame.get("timestamp_span_sec"),
            f"camera_frames[{raw_index}].timestamp_span_sec",
            minimum=0.0,
        )
        if abs(declared_span - skew_ns * 1.0e-9) > 1.0e-9:
            raise AdapterError(
                f"camera_frames[{raw_index}].timestamp_span_sec is inconsistent"
            )
        if anchor_ns <= last_anchor:
            raise AdapterError("native camera anchor timestamps must be strictly increasing")
        last_anchor = anchor_ns
        source_frame_ids: list[str] = []
        bundle: list[dict[str, Any]] = []
        prepared_images: list[tuple[Path, Path, str]] = []
        carla_frame = _integer(frame.get("frame"), f"camera_frames[{raw_index}].frame", minimum=0)
        if last_carla_frame >= 0 and carla_frame != last_carla_frame + expected_camera_interval:
            raise AdapterError("native CARLA camera frame cadence does not match capture interval")
        last_carla_frame = carla_frame
        state_binding = state_by_frame.get(carla_frame)
        if state_binding is None:
            raise AdapterError(f"camera_frames[{raw_index}] has no same-frame ego state")
        state_ns, same_frame_state = state_binding
        if snapshot_ns != state_ns:
            raise AdapterError(f"camera_frames[{raw_index}] timestamp differs from same-frame ego state")
        if same_frame_state.get("capture_phase") != phase:
            raise AdapterError(f"camera_frames[{raw_index}] phase differs from same-frame ego state")
        if any(abs(camera_time - snapshot_ns) > 1 for camera_time in camera_times):
            raise AdapterError(f"camera_frames[{raw_index}] sensor timestamp differs from snapshot")
        if _integer(frame.get("jpeg_quality"), f"camera_frames[{raw_index}].jpeg_quality", minimum=1) != expected_jpeg_quality:
            raise AdapterError(f"camera_frames[{raw_index}] JPEG quality differs from capture contract")
        eligibility_reason: str | None = None
        if phase not in ANCHOR_PHASES:
            eligibility_reason = "stationary_tail_label_context_only"
        elif anchor_ns + horizon_ns > final_state_ns:
            eligibility_reason = "future_horizon_unavailable"
        for camera_index, name in enumerate(common10.CAMERA_ORDER):
            expected_uri = (Path("images") / name / f"{carla_frame:08d}.jpg").as_posix()
            source_uri = images[name]
            if source_uri != expected_uri:
                raise AdapterError(
                    f"camera_frames[{raw_index}] {name} URI is not bound to its CARLA frame"
                )
            if source_uri in seen_source_uris:
                raise AdapterError(f"native camera source URI is reused: {source_uri}")
            source = _safe_source_file(native.spec.path, source_uri, f"{name} image")
            identity = (source.stat().st_dev, source.stat().st_ino)
            if identity in seen_source_files:
                raise AdapterError(f"native camera source file inode is reused: {source_uri}")
            seen_source_uris.add(source_uri)
            seen_source_files.add(identity)
            digest = _sha256(source)
            source_frame_id = f"carla:{carla_frame:010d}:{name}"
            source_frame_ids.append(source_frame_id)
            relative = Path("images") / name / f"{camera_times[camera_index]:019d}.jpg"
            destination = episode_root / relative
            prepared_images.append((source, destination, digest))
            raw_frames.append(
                {
                    "camera_name": name,
                    "source_frame_id": source_frame_id,
                    "source_timestamp_ns": camera_times[camera_index],
                    "source_sequence_index": raw_index,
                    "source_uri": str(source_uri),
                    "source_payload_sha256": digest,
                    "source_artifact_sha256": raw_camera_hash,
                    "prepared_payload_sha256": (
                        digest if eligibility_reason is None else None
                    ),
                    "image_transform_id": "identity_jpeg",
                }
            )
            frame_counts[name] += 1
            bundle.append(
                {
                    "name": name,
                    "model_index": camera_index,
                    "timestamp_ns": camera_times[camera_index],
                    "source_timestamp_ns": camera_times[camera_index],
                    "source_frame_id": source_frame_id,
                    "frame_counter": raw_index,
                    "frame_id": rig["cameras"][camera_index]["optical_frame"],
                    "path": relative.as_posix(),
                    "sha256": digest,
                }
            )
        if eligibility_reason is not None:
            ineligible[eligibility_reason] = ineligible.get(eligibility_reason, 0) + 1
            continue
        ego_ns, ego = _causal_state(timeline, anchor_ns)
        if ego.get("capture_phase") != phase:
            raise AdapterError("camera and causal ego capture phases disagree")
        if anchor_ns - ego_ns > int(contract["capture"]["maximum_state_delta_ms"] * 1_000_000):
            raise AdapterError("native ego state is stale for a camera anchor")
        positions: list[list[float]] = []
        yaws: list[float] = []
        speeds: list[float] = []
        target_times: list[int] = []
        for point_index in range(int(trajectory["point_count"])):
            target_ns = anchor_ns + (point_index + 1) * step_ns
            future = _interpolate_state(timeline, target_ns)
            if future is None:
                raise AdapterError("complete-horizon anchor lacks an interpolatable state")
            xy, relative_yaw = _relative_xy_yaw(ego, future)
            positions.append(xy)
            yaws.append(relative_yaw)
            speeds.append(math.hypot(future["vx"], future["vy"]))
            target_times.append(target_ns)
        yaw = float(ego["yaw"])
        quaternion = [0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)]
        expected_route, expected_goal, anchor_arc = common10._canonical_route_in_base(
            tuple((point[0], point[1]) for point in polyline),
            position_m=(float(ego["x"]), float(ego["y"]), float(ego["z"])),
            orientation_xyzw=quaternion,
            contract=contract,
            context=f"{native.episode_id}:{raw_index}.navigation",
        )
        source_sample_id = f"source:{native.episode_id}:{carla_frame:010d}"
        sample_id = f"{native.episode_id}:{len(samples):06d}"
        samples.append(
            {
                "schema_id": common10.SAMPLE_SCHEMA_ID,
                "sample_id": sample_id,
                "source_sample_id": source_sample_id,
                "episode_id": native.episode_id,
                "sequence_index": len(samples),
                "anchor_timestamp_ns": anchor_ns,
                "rig_id": rig["rig_id"],
                "camera_bundle": bundle,
                "ego": {
                    "timestamp_ns": ego_ns,
                    "frame_id": "map",
                    "child_frame_id": "base_link",
                    "position_m": [float(ego["x"]), float(ego["y"]), float(ego["z"])],
                    "orientation_xyzw": quaternion,
                    "linear_velocity_base_mps": [float(ego["vx"]), float(ego["vy"]), 0.0],
                    "angular_velocity_base_radps": [0.0, 0.0, float(ego["yaw_rate"])],
                    "linear_acceleration_base_mps2": [float(ego["ax"]), float(ego["ay"]), 0.0],
                    "steering_tire_angle_rad": float(ego["steering_tire_angle_rad"]),
                },
                "navigation": {
                    "route_id": native.route_id,
                    "route_source": "episode_route_geometry",
                    "route_source_timestamp_ns": route_source_ns,
                    "route_geometry_sha256": route_hash,
                    "route_anchor_arc_m": anchor_arc,
                    "command": int(ego["command"]),
                    "command_timestamp_ns": ego_ns,
                    "speed_limit_mps": float(ego["speed_limit_mps"]),
                    "route_polyline_base_m": [list(point) for point in expected_route],
                    "goal_base_m": list(expected_goal),
                },
                "labels": {
                    "planning": {
                        "available": True,
                        "dt_s": float(trajectory["step_s"]),
                        "positions_base_xy_m": positions,
                        "yaw_rad": yaws,
                        "speed_mps": speeds,
                        "valid": [True] * int(trajectory["point_count"]),
                        "target_timestamp_ns": target_times,
                        "invalid_reason": [None] * int(trajectory["point_count"]),
                    },
                    "objects": {"available": False},
                    "occupancy": {"available": False},
                },
                "events": {
                    "collision": False,
                    "lane_invasion": False,
                    "manual_intervention": False,
                    "fallback_active": False,
                },
            }
        )
        selected_sources.append(
            {
                "source_sample_id": source_sample_id,
                "anchor_timestamp_ns": anchor_ns,
                "camera_source_frame_ids": source_frame_ids,
            }
        )
        for source, destination, _digest in prepared_images:
            _copy_image(source, destination, image_mode)
    if len(samples) < 2:
        raise AdapterError(f"{native.episode_id} has fewer than two complete-horizon anchors")
    selected_duration = (samples[-1]["anchor_timestamp_ns"] - samples[0]["anchor_timestamp_ns"]) * 1e-9
    minimum_duration = float(contract["capture"]["minimum_episode_duration_s"])
    if selected_duration + 1.0e-9 < minimum_duration:
        raise AdapterError(
            f"{native.episode_id} complete-horizon anchor span {selected_duration:.3f}s "
            f"is below Common10 minimum {minimum_duration:.3f}s; collect explicit "
            "warm-up plus at least 6.4s stationary tail or use a longer route"
        )
    samples_path = episode_root / "samples.jsonl"
    _write_jsonl(samples_path, samples)
    source_manifest_path = episode_root / "source_manifest.json"
    _write_json(
        source_manifest_path,
        {
            "schema_id": common10.SOURCE_MANIFEST_SCHEMA_ID,
            "source_session_id": source_session_id,
            "source_dataset_id": "carla_basicagent_expert",
            "source_dataset_version": source_dataset_version,
            "license_id": license_id,
            "source_artifacts": source_artifacts,
            "route_source": {
                "available_timestamp_ns": route_source_ns,
                "uri": "route.json",
                "source_payload_sha256": raw_route_hash,
                "source_artifact_sha256": raw_route_hash,
                "route_geometry_sha256": route_hash,
            },
            "camera_frames": raw_frames,
            "selected_samples": selected_sources,
        },
    )
    collection_path = episode_root / "collection_config.json"
    _write_json(
        collection_path,
        {
            "adapter_id": ADAPTER_ID,
            "native_capture_contract": native.manifest["capture_contract"],
            "native_runtime": native.manifest.get("runtime"),
            "native_result": native.manifest.get("result"),
            "future_selection_policy": "complete_64_point_prefix_only",
            "image_mode": image_mode,
            "source_hashes": dict(native.source_hashes),
        },
    )
    provisional_episode = {
        "map_id": native.site_id,
        "route_id": native.route_id,
    }
    route_semantic_hash = common10._route_geometry_semantic_hash(
        route_geometry_path,
        episode=provisional_episode,
        contract=contract,
    )
    split_group = f"carla:{native.site_id}:{route_semantic_hash}"
    selected_camera_timestamps = [
        int(camera["timestamp_ns"])
        for sample in samples
        for camera in sample["camera_bundle"]
    ]
    episode_start_ns = min(selected_camera_timestamps)
    episode_end_ns = max(
        final_state_ns,
        max(selected_camera_timestamps),
        max(int(value) for value in samples[-1]["labels"]["planning"]["target_timestamp_ns"]),
    )
    episode = {
        "schema_id": common10.EPISODE_SCHEMA_ID,
        "episode_id": native.episode_id,
        "domain": "carla",
        "split": native.spec.split,
        "split_group_id": split_group,
        "source_session_id": source_session_id,
        "scene_group_id": f"{split_group}:traffic={native.seed}:weather={native.weather}",
        "rig_id": rig["rig_id"],
        "vehicle_id": str((native.manifest.get("runtime") or {}).get("vehicle_type", "carla_vehicle")),
        "site_id": native.site_id,
        "map_id": native.site_id,
        "route_id": native.route_id,
        "scenario_tags": _scenario_tags(str(native.route["scenario"])),
        "weather": native.weather,
        "clock_domain": "carla_sim_time",
        "traffic_seed": native.seed,
        "start_timestamp_ns": episode_start_ns,
        "end_timestamp_ns": episode_end_ns,
        "route_geometry_file": "route_geometry.json",
        "route_geometry_sha256": route_hash,
        "sample_jsonl": "samples.jsonl",
        "sample_count": len(samples),
        "sample_jsonl_sha256": _sha256(samples_path),
        "source_provenance": {
            "adapter_id": ADAPTER_ID,
            "git_commit": git_commit,
            "source_manifest_file": "source_manifest.json",
            "source_manifest_sha256": _sha256(source_manifest_path),
            "collection_config_file": "collection_config.json",
            "collection_config_sha256": _sha256(collection_path),
        },
        "capture_accounting": {
            "raw_anchor_count": len(native.camera_frames),
            "eligible_anchor_count": len(samples),
            "selected_bundle_count": len(samples),
            "dropped_incomplete_bundle_count": 0,
            "ineligible_anchor_reasons": dict(sorted(ineligible.items())),
            "raw_camera_frame_counts": frame_counts,
        },
    }
    episode_path = episode_root / "episode.json"
    _write_json(episode_path, episode)
    return {
        "episode_id": native.episode_id,
        "manifest": episode_path.relative_to(output_root).as_posix(),
        "sha256": _sha256(episode_path),
    }


def prepare_dataset(args: argparse.Namespace) -> tuple[Path, Mapping[str, Any]]:
    output = args.output.expanduser().resolve()
    partial = output.with_name(output.name + ".partial")
    if output.exists() or partial.exists():
        raise AdapterError(f"output and partial output must not already exist: {output}")
    for spec in args.episode:
        try:
            output.relative_to(spec.path)
        except ValueError:
            continue
        raise AdapterError(
            f"output must not be inside a native episode: {spec.path}"
        )
    native = tuple(_load_native(spec) for spec in args.episode)
    episode_ids = [item.episode_id for item in native]
    if len(set(episode_ids)) != len(episode_ids):
        raise AdapterError("native inputs resolve to duplicate episode IDs")
    rig = _rig_document(native[0])
    for item in native[1:]:
        if _rig_document(item) != rig:
            raise AdapterError("native inputs use different camera rigs; convert separately")
    commit = _git_commit(args.git_commit)
    contract = common10.load_contract()
    try:
        partial.mkdir(parents=True, exist_ok=False)
        rig_path = partial / "rigs" / f"{rig['rig_id']}.json"
        _write_json(rig_path, rig)
        references = [
            _prepare_episode(
                item,
                partial,
                rig,
                contract,
                source_dataset_version=args.source_dataset_version,
                license_id=args.license_id,
                image_mode=args.image_mode,
                git_commit=commit,
            )
            for item in native
        ]
        _write_json(
            partial / "dataset.json",
            {
                "schema_id": common10.DATASET_SCHEMA_ID,
                "contract_id": common10.CONTRACT_ID,
                "contract_sha256": common10.contract_fingerprint(contract),
                "dataset_id": args.dataset_id,
                "created_at_utc": _created_at(native),
                "camera_order": list(common10.CAMERA_ORDER),
                "split_policy": contract["split_policy"],
                "rigs": [
                    {
                        "rig_id": rig["rig_id"],
                        "manifest": rig_path.relative_to(partial).as_posix(),
                        "sha256": _sha256(rig_path),
                    }
                ],
                "episodes": references,
            },
        )
        report = common10.validate_dataset(
            partial,
            contract=contract,
            mode=args.validate_mode,
            check_image_hashes=True,
        )
        _write_json(partial / "validation_report.json", report)
        partial.replace(output)
        return output, report
    except BaseException:
        shutil.rmtree(partial, ignore_errors=True)
        raise


def main(argv: Sequence[str] | None = None) -> int:
    try:
        args = _parse_args(argv)
        output, report = prepare_dataset(args)
    except (AdapterError, common10.ContractError, OSError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print(
        f"PASS dataset={report['dataset_id']} episodes={report['episode_count']} "
        f"samples={report['sample_count']} mode={report['mode']} output={output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
