#!/usr/bin/env python3
"""Collect deterministic six-camera CARLA episodes driven by BasicAgent."""

from __future__ import annotations

import argparse
from collections import defaultdict
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
from queue import Empty, SimpleQueue
import random
import shutil
import signal
import sys
import threading
import time
from typing import Any, Mapping, Sequence

import yaml

# HH_260906 - Keep goal-stop collection opt-in and independent of learned actuation.
if __package__:
    from .carla_goal_stop_profile import (
        DevelopmentGoalStopConfig, DevelopmentGoalStopGovernor, GoalStopGovernor,
        annotate_development_control, bounded_route_projection, complete_terminal_plan, configuration_from_args, install_development_control,
        goal_stop_termination_reason, install_normal_brake_cap, measured_goal_completion,
        measured_stop_quality, source_motion_bounds, terminal_overshoot_m, validate_development_route,
    )
else:
    from carla_goal_stop_profile import (
        DevelopmentGoalStopConfig, DevelopmentGoalStopGovernor, GoalStopGovernor,
        annotate_development_control, bounded_route_projection, complete_terminal_plan, configuration_from_args, install_development_control,
        goal_stop_termination_reason, install_normal_brake_cap, measured_goal_completion,
        measured_stop_quality, source_motion_bounds, terminal_overshoot_m, validate_development_route,
    )


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MAPPING = (
    ROOT / "autoware_e2e_vad_launch/config/sensor_mapping_vad_fast_reliable.yaml"
)
DEFAULT_CALIBRATION = (
    ROOT
    / "src/launcher/autoware_launch/sensor_kit/carla_sensor_kit_launch/"
    "carla_sensor_kit_description/config/sensor_kit_calibration.yaml"
)
MODEL_CAMERA_ORDER = (
    "CAM_FRONT",
    "CAM_BACK",
    "CAM_FRONT_LEFT",
    "CAM_BACK_LEFT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK_RIGHT",
)
WHEELBASE_M = 2.850
MANEUVER_COMMANDS = frozenset((0, 1, 2, 4, 5))
CAPTURE_PHASE_ORDER = (
    "stationary_warmup",
    "driving",
    "stationary_tail",
)

# These are the CARLA 0.9.15 LocalPlanner defaults used by the collector before
# the controls below became explicit CLI options.  Keep them stable so existing
# commands retain the same 20 Hz behavior while every new episode discloses the
# exact expert-controller contract.
BASIC_AGENT_CONTROL_DEFAULTS = {
    "base_min_distance_m": 3.0,
    "distance_ratio_s": 0.5,
    "lateral_pid_kp": 1.95,
    "lateral_pid_ki": 0.05,
    "lateral_pid_kd": 0.2,
    "lateral_pid_dt_sec": 1.0 / 20.0,
    "max_steering": 0.8,
    "lane_offset_m": 0.0,
}


class CollectionError(RuntimeError):
    """Raised when a dataset cannot satisfy its capture contract."""


class CollectionInterrupted(CollectionError):
    """Raised after SIGINT or SIGTERM requests a graceful stop."""


@dataclass(frozen=True)
class CameraSpec:
    name: str
    model_index: int
    carla_type: str
    width: int
    height: int
    fov_degrees: float
    sensor_tick: float
    enable_postprocess_effects: bool
    frame_id: str
    ros_extrinsic: dict[str, float]
    carla_extrinsic: dict[str, float]
    intrinsic: dict[str, float]


@dataclass(frozen=True)
class Projection:
    progress_m: float
    cross_track_error_m: float
    segment_index: int


@dataclass(frozen=True)
class CatalogGoalStatus:
    reached: bool
    remaining_route_m: float
    planar_distance_m: float


def basic_agent_control_configuration(
    args: argparse.Namespace, sampling_resolution_m: float
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Resolve and validate the exact BasicAgent/LocalPlanner control options.

    ``getattr`` fallbacks intentionally support older programmatic Namespace
    fixtures.  The fallback values match the pre-option CARLA 0.9.15 behavior.
    """
    physics_hz = float(getattr(args, "physics_hz", 20.0))
    target_speed_kmh = float(getattr(args, "target_speed_kmh", 9.0))
    base_min_distance_m = float(
        getattr(
            args,
            "basic_agent_base_min_distance_m",
            BASIC_AGENT_CONTROL_DEFAULTS["base_min_distance_m"],
        )
    )
    distance_ratio_s = float(
        getattr(
            args,
            "basic_agent_distance_ratio",
            BASIC_AGENT_CONTROL_DEFAULTS["distance_ratio_s"],
        )
    )
    lateral_pid_kp = float(
        getattr(
            args,
            "basic_agent_lateral_kp",
            BASIC_AGENT_CONTROL_DEFAULTS["lateral_pid_kp"],
        )
    )
    lateral_pid_ki = float(
        getattr(
            args,
            "basic_agent_lateral_ki",
            BASIC_AGENT_CONTROL_DEFAULTS["lateral_pid_ki"],
        )
    )
    lateral_pid_kd = float(
        getattr(
            args,
            "basic_agent_lateral_kd",
            BASIC_AGENT_CONTROL_DEFAULTS["lateral_pid_kd"],
        )
    )
    max_steering = float(
        getattr(
            args,
            "basic_agent_max_steering",
            BASIC_AGENT_CONTROL_DEFAULTS["max_steering"],
        )
    )
    lane_offset_m = float(
        getattr(
            args,
            "basic_agent_lane_offset_m",
            BASIC_AGENT_CONTROL_DEFAULTS["lane_offset_m"],
        )
    )
    sampling_resolution_m = float(sampling_resolution_m)

    finite_values = {
        "physics_hz": physics_hz,
        "target_speed_kmh": target_speed_kmh,
        "sampling_resolution_m": sampling_resolution_m,
        "basic_agent_base_min_distance_m": base_min_distance_m,
        "basic_agent_distance_ratio": distance_ratio_s,
        "basic_agent_lateral_kp": lateral_pid_kp,
        "basic_agent_lateral_ki": lateral_pid_ki,
        "basic_agent_lateral_kd": lateral_pid_kd,
        "basic_agent_max_steering": max_steering,
        "basic_agent_lane_offset_m": lane_offset_m,
    }
    invalid_finite = [
        name for name, value in finite_values.items() if not math.isfinite(value)
    ]
    if invalid_finite:
        raise CollectionError(
            "BasicAgent control values must be finite: " + ", ".join(invalid_finite)
        )
    if physics_hz <= 0.0 or sampling_resolution_m <= 0.0 or target_speed_kmh <= 0.0:
        raise CollectionError(
            "BasicAgent physics Hz, sampling resolution, and target speed must be positive"
        )
    if base_min_distance_m < 0.0 or distance_ratio_s < 0.0:
        raise CollectionError(
            "BasicAgent waypoint purge base distance and distance ratio must be "
            "non-negative"
        )
    if lateral_pid_kp <= 0.0:
        raise CollectionError("BasicAgent lateral PID Kp must be positive")
    if lateral_pid_ki < 0.0 or lateral_pid_kd < 0.0:
        raise CollectionError("BasicAgent lateral PID Ki and Kd must be non-negative")
    if not 0.0 < max_steering <= 1.0:
        raise CollectionError(
            "BasicAgent maximum steering must be in the normalized range (0, 1]"
        )

    lateral_pid = {
        "K_P": lateral_pid_kp,
        "K_I": lateral_pid_ki,
        "K_D": lateral_pid_kd,
        # Preserve the upstream LocalPlanner default.  The existing collector's
        # top-level opt_dict dt did not rewrite this nested controller value.
        "dt": BASIC_AGENT_CONTROL_DEFAULTS["lateral_pid_dt_sec"],
    }
    options = {
        "dt": 1.0 / physics_hz,
        "target_speed": target_speed_kmh,
        "sampling_resolution": sampling_resolution_m,
        "base_min_distance": base_min_distance_m,
        "distance_ratio": distance_ratio_s,
        "lateral_control_dict": lateral_pid,
        "max_steering": max_steering,
        "offset": lane_offset_m,
    }
    contract = {
        "provider": "CARLA PythonAPI BasicAgent",
        "option_injection": "BasicAgent(..., opt_dict=effective_opt_dict)",
        "effective_opt_dict": options,
        "waypoint_purge_lookahead": {
            "formula": "base_min_distance_m + distance_ratio_s * speed_mps",
            "base_min_distance_m": base_min_distance_m,
            "distance_ratio_s": distance_ratio_s,
            "upstream_option_keys": ["base_min_distance", "distance_ratio"],
        },
        "lateral_pid": {
            "kp": lateral_pid_kp,
            "ki": lateral_pid_ki,
            "kd": lateral_pid_kd,
            "dt_sec": lateral_pid["dt"],
            "upstream_option_key": "lateral_control_dict",
        },
        "normalized_max_steering": max_steering,
        "lane_offset_m": lane_offset_m,
        "lane_offset_sign_convention": "positive-right, negative-left",
        "collector_cli": {
            "--basic-agent-base-min-distance-m": base_min_distance_m,
            "--basic-agent-distance-ratio": distance_ratio_s,
            "--basic-agent-lateral-kp": lateral_pid_kp,
            "--basic-agent-lateral-ki": lateral_pid_ki,
            "--basic-agent-lateral-kd": lateral_pid_kd,
            "--basic-agent-max-steering": max_steering,
            "--basic-agent-lane-offset-m": lane_offset_m,
        },
        "compatibility_defaults": dict(BASIC_AGENT_CONTROL_DEFAULTS),
    }
    return options, contract


def _python_class_source_provenance(value: Any) -> dict[str, Any]:
    """Record the loaded controller implementation without modifying it."""
    class_value = value if inspect.isclass(value) else type(value)
    result: dict[str, Any] = {
        "python_class": f"{class_value.__module__}.{class_value.__qualname__}",
    }
    source = inspect.getsourcefile(class_value)
    if source is None:
        result["source_file"] = None
        result["source_sha256"] = None
        return result
    source_path = Path(source).expanduser().resolve()
    result["source_file"] = str(source_path)
    result["source_sha256"] = sha256_file(source_path)
    return result


def capture_phase_schedule(
    physics_hz: float,
    maximum_duration_sec: float,
    stationary_warmup_sec: float,
    stationary_tail_sec: float,
) -> dict[str, Any]:
    """Build a tick-exact phase schedule bounded by the total duration."""
    values = (
        physics_hz,
        maximum_duration_sec,
        stationary_warmup_sec,
        stationary_tail_sec,
    )
    if not all(math.isfinite(value) for value in values):
        raise CollectionError("capture phase durations and physics Hz must be finite")
    if physics_hz <= 0.0 or maximum_duration_sec <= 0.0:
        raise CollectionError("physics Hz and maximum duration must be positive")
    if stationary_warmup_sec < 0.0 or stationary_tail_sec < 0.0:
        raise CollectionError("stationary phase durations must be non-negative")

    # A requested stationary duration is rounded up so that it is never silently
    # shortened.  The total limit is rounded down, so the capture never exceeds it.
    def requested_ticks(duration_sec: float) -> int:
        return int(math.ceil(duration_sec * physics_hz - 1.0e-12))

    warmup_ticks = requested_ticks(stationary_warmup_sec)
    tail_ticks = requested_ticks(stationary_tail_sec)
    maximum_total_ticks = int(
        math.floor(maximum_duration_sec * physics_hz + 1.0e-12)
    )
    maximum_driving_ticks = maximum_total_ticks - warmup_ticks - tail_ticks
    if maximum_driving_ticks < 1:
        raise CollectionError(
            "maximum duration must leave at least one physics tick for driving "
            "after stationary warmup and tail"
        )

    return {
        "order": list(CAPTURE_PHASE_ORDER),
        "maximum_total_duration_sec": float(maximum_duration_sec),
        "maximum_total_ticks": maximum_total_ticks,
        "rounding_policy": {
            "stationary_duration": "ceil_to_physics_tick",
            "maximum_total_duration": "floor_to_physics_tick",
        },
        "stationary_warmup": {
            "control_source": "full_service_brake",
            "requested_duration_sec": float(stationary_warmup_sec),
            "scheduled_ticks": warmup_ticks,
            "scheduled_duration_sec": warmup_ticks / physics_hz,
        },
        "driving": {
            "control_source": "carla.BasicAgent",
            "maximum_ticks": maximum_driving_ticks,
            "maximum_duration_sec": maximum_driving_ticks / physics_hz,
        },
        "stationary_tail": {
            "control_source": "full_service_brake",
            "requested_duration_sec": float(stationary_tail_sec),
            "scheduled_ticks": tail_ticks,
            "scheduled_duration_sec": tail_ticks / physics_hz,
        },
    }


def summarize_capture_phases(
    schedule: Mapping[str, Any],
    observations: Mapping[str, Mapping[str, Any]],
    physics_hz: float,
) -> dict[str, Any]:
    """Combine planned phase bounds with observed records for the manifest."""
    result: dict[str, Any] = {
        "order": list(CAPTURE_PHASE_ORDER),
        "maximum_total_duration_sec": float(schedule["maximum_total_duration_sec"]),
        "maximum_total_ticks": int(schedule["maximum_total_ticks"]),
    }
    total_ticks = 0
    for phase in CAPTURE_PHASE_ORDER:
        observation = observations[phase]
        state_count = int(observation["state_count"])
        total_ticks += state_count
        first_timestamp = observation.get("first_timestamp")
        last_timestamp = observation.get("last_timestamp")
        timestamp_span_sec = (
            float(last_timestamp) - float(first_timestamp)
            if state_count > 1
            else 0.0
        )
        result[phase] = {
            **dict(schedule[phase]),
            "state_count": state_count,
            "camera_anchor_count": int(observation["camera_anchor_count"]),
            "first_timestamp": first_timestamp,
            "last_timestamp": last_timestamp,
            "timestamp_span_sec": timestamp_span_sec,
            "elapsed_sim_sec": state_count / physics_hz,
        }
    result["observed_total_ticks"] = total_ticks
    result["observed_total_elapsed_sim_sec"] = total_ticks / physics_hz
    return result


def front_steering_measurement(
    front_left_carla_deg: float, front_right_carla_deg: float
) -> dict[str, float]:
    """Convert measured CARLA front-wheel angles to ROS-positive-left radians.

    CARLA's vehicle control/wheel convention is positive-right.  The two physical
    wheel angles are retained verbatim and converted independently.  The virtual
    center tire angle uses the Ackermann harmonic-tangent relation, avoiding any
    use of the normalized VehicleControl.steer command as an angle.
    """
    if not all(
        math.isfinite(value)
        for value in (front_left_carla_deg, front_right_carla_deg)
    ):
        raise CollectionError("measured front-wheel steer angles must be finite")
    front_left_ros_rad = -math.radians(front_left_carla_deg)
    front_right_ros_rad = -math.radians(front_right_carla_deg)
    if max(abs(front_left_ros_rad), abs(front_right_ros_rad)) >= math.pi / 2.0:
        raise CollectionError("measured front-wheel steer angles must be within 90 degrees")

    left_tangent = math.tan(front_left_ros_rad)
    right_tangent = math.tan(front_right_ros_rad)
    # Treat only sub-resolution disagreement around straight ahead as zero; a
    # material left/right disagreement is a corrupt physical measurement.
    straight_tolerance = math.tan(math.radians(0.1))
    if max(abs(left_tangent), abs(right_tangent)) <= straight_tolerance:
        virtual_angle = 0.5 * (front_left_ros_rad + front_right_ros_rad)
    else:
        if left_tangent * right_tangent <= 0.0:
            raise CollectionError(
                "measured front-wheel steer angles have inconsistent directions"
            )
        virtual_tangent = (
            2.0 * left_tangent * right_tangent / (left_tangent + right_tangent)
        )
        virtual_angle = math.atan(virtual_tangent)

    return {
        "front_left_wheel_steer_angle_carla_deg": float(front_left_carla_deg),
        "front_right_wheel_steer_angle_carla_deg": float(front_right_carla_deg),
        "front_left_wheel_steer_angle_ros_rad": front_left_ros_rad,
        "front_right_wheel_steer_angle_ros_rad": front_right_ros_rad,
        "steering_tire_angle_rad": virtual_angle,
    }


def speed_limit_measurement(speed_limit_kmh: float) -> dict[str, float]:
    """Convert CARLA's measured speed-limit value from km/h to m/s."""
    if not math.isfinite(speed_limit_kmh) or speed_limit_kmh < 0.0:
        raise CollectionError("measured speed limit must be finite and non-negative")
    return {
        "speed_limit_carla_kmh": float(speed_limit_kmh),
        "speed_limit_mps": float(speed_limit_kmh) / 3.6,
    }


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def shift_transform_local_x(
    transform: Mapping[str, float], distance_m: float
) -> dict[str, float]:
    """Shift a CARLA transform along its local forward vector."""
    yaw = math.radians(float(transform.get("yaw", 0.0)))
    pitch = math.radians(float(transform.get("pitch", 0.0)))
    cos_pitch = math.cos(pitch)
    shifted = {
        key: float(transform.get(key, 0.0))
        for key in ("x", "y", "z", "roll", "pitch", "yaw")
    }
    shifted["x"] += distance_m * cos_pitch * math.cos(yaw)
    shifted["y"] += distance_m * cos_pitch * math.sin(yaw)
    shifted["z"] += distance_m * math.sin(pitch)
    return shifted


def apply_spawn_z_offset(
    actor_center_transform: Mapping[str, float], spawn_z_offset_m: float
) -> dict[str, float]:
    """Raise only the actor-center spawn, leaving the map-referenced pose intact."""
    if not math.isfinite(spawn_z_offset_m) or spawn_z_offset_m < 0.0:
        raise CollectionError("spawn z offset must be finite and non-negative")
    spawn = {
        key: float(actor_center_transform.get(key, 0.0))
        for key in ("x", "y", "z", "roll", "pitch", "yaw")
    }
    spawn["z"] += spawn_z_offset_m
    return spawn


def _spawn_runtime_fields(
    map_center: Mapping[str, float],
    actor_spawn: Mapping[str, float],
    spawn_z_offset_m: float,
) -> dict[str, Any]:
    return {
        "spawn_z_offset_m": float(spawn_z_offset_m),
        "actor_center_start_carla": dict(map_center),
        "actor_center_spawn_carla": dict(actor_spawn),
    }


def ros_extrinsic_to_carla_vehicle_center(
    transform: Mapping[str, float], wheelbase_m: float = WHEELBASE_M
) -> dict[str, float]:
    """Convert a ROS rear-axle calibration into a CARLA actor-relative pose."""
    return {
        "x": float(transform["x"]) - wheelbase_m / 2.0,
        "y": -float(transform["y"]),
        "z": float(transform["z"]),
        "roll": math.degrees(float(transform.get("roll", 0.0))),
        "pitch": -math.degrees(float(transform.get("pitch", 0.0))),
        "yaw": -math.degrees(float(transform.get("yaw", 0.0))),
    }


def camera_intrinsic(width: int, height: int, fov_degrees: float) -> dict[str, float]:
    focal = width / (2.0 * math.tan(math.radians(fov_degrees) / 2.0))
    return {
        "fx": focal,
        "fy": focal,
        "cx": width / 2.0,
        "cy": height / 2.0,
    }


def _load_yaml_mapping(path: Path, context: str) -> Mapping[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise CollectionError(f"failed to load {context} {path}: {error}") from error
    if not isinstance(value, Mapping):
        raise CollectionError(f"{context} must contain a YAML mapping: {path}")
    return value


def load_camera_specs(
    mapping_path: Path, calibration_path: Path, wheelbase_m: float = WHEELBASE_M
) -> tuple[CameraSpec, ...]:
    mapping = _load_yaml_mapping(mapping_path, "sensor mapping")
    calibration = _load_yaml_mapping(calibration_path, "sensor calibration")
    mappings = mapping.get("sensor_mappings")
    extrinsics = calibration.get("sensor_kit_base_link")
    if not isinstance(mappings, Mapping) or not isinstance(extrinsics, Mapping):
        raise CollectionError("sensor mapping/calibration is missing its required root mapping")

    specs = []
    for model_index, name in enumerate(MODEL_CAMERA_ORDER):
        key = f"{name}/camera_link"
        camera_mapping = mappings.get(key)
        ros_extrinsic = extrinsics.get(key)
        if not isinstance(camera_mapping, Mapping) or not isinstance(ros_extrinsic, Mapping):
            raise CollectionError(f"camera {key} is missing from mapping or calibration")
        if camera_mapping.get("carla_type") != "sensor.camera.rgb":
            raise CollectionError(f"camera {key} must use sensor.camera.rgb")
        parameters = camera_mapping.get("parameters")
        ros_config = camera_mapping.get("ros_config")
        if not isinstance(parameters, Mapping) or not isinstance(ros_config, Mapping):
            raise CollectionError(f"camera {key} has incomplete configuration")
        width = int(parameters["image_size_x"])
        height = int(parameters["image_size_y"])
        fov = float(parameters["fov"])
        sensor_tick = float(parameters.get("sensor_tick", 0.0))
        if width <= 0 or height <= 0 or not 0.0 < fov < 180.0:
            raise CollectionError(f"camera {key} has invalid image geometry")
        if sensor_tick != 0.0:
            raise CollectionError(
                f"camera {key} sensor_tick must be 0.0 for exact-frame capture"
            )
        raw_extrinsic = {
            field: float(ros_extrinsic.get(field, 0.0))
            for field in ("x", "y", "z", "roll", "pitch", "yaw")
        }
        specs.append(
            CameraSpec(
                name=name,
                model_index=model_index,
                carla_type="sensor.camera.rgb",
                width=width,
                height=height,
                fov_degrees=fov,
                sensor_tick=sensor_tick,
                enable_postprocess_effects=bool(
                    parameters.get("enable_postprocess_effects", True)
                ),
                frame_id=str(ros_config.get("frame_id", f"{name}/camera_optical_link")),
                ros_extrinsic=raw_extrinsic,
                carla_extrinsic=ros_extrinsic_to_carla_vehicle_center(
                    raw_extrinsic, wheelbase_m
                ),
                intrinsic=camera_intrinsic(width, height, fov),
            )
        )
    return tuple(specs)


def load_route(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise CollectionError(f"failed to load route {path}: {error}") from error
    if not isinstance(payload, dict) or payload.get("schema_version") != 1:
        raise CollectionError(f"route must use schema version 1: {path}")
    if payload.get("coordinate_reference", "base_link") != "base_link":
        raise CollectionError("route coordinates must use ROS base_link")
    for key in ("town", "start_carla_transform", "goal_carla_transform", "route"):
        if key not in payload:
            raise CollectionError(f"route is missing {key}: {path}")
    if not isinstance(payload["route"], list) or len(payload["route"]) < 2:
        raise CollectionError("route must contain at least two points")
    return payload


class RouteProjector:
    def __init__(self, route_payload: Mapping[str, Any]):
        self.points = tuple(route_payload["route"])
        self.length_m = float(self.points[-1]["distance_m"])
        previous = -math.inf
        for point in self.points:
            distance = float(point["distance_m"])
            command = int(point["vad_command"])
            if distance < previous or command not in range(6):
                raise CollectionError("route distances or commands are invalid")
            previous = distance

    def project(
        self,
        x: float,
        y: float,
        previous_progress_m: float,
        backtrack_m: float = 3.0,
        forward_m: float = 80.0,
    ) -> Projection:
        start = max(0.0, previous_progress_m - backtrack_m)
        end = min(self.length_m, previous_progress_m + forward_m)
        best = None
        for index, (first, second) in enumerate(zip(self.points, self.points[1:])):
            first_distance = float(first["distance_m"])
            second_distance = float(second["distance_m"])
            if second_distance < start or first_distance > end:
                continue
            dx = float(second["x"]) - float(first["x"])
            dy = float(second["y"]) - float(first["y"])
            squared_length = dx * dx + dy * dy
            ratio = 0.0
            if squared_length > 1.0e-9:
                ratio = max(
                    0.0,
                    min(
                        1.0,
                        ((x - float(first["x"])) * dx + (y - float(first["y"])) * dy)
                        / squared_length,
                    ),
                )
            projected_x = float(first["x"]) + ratio * dx
            projected_y = float(first["y"]) + ratio * dy
            error = math.hypot(x - projected_x, y - projected_y)
            progress = first_distance + ratio * (second_distance - first_distance)
            candidate = (error, -progress, index, progress)
            if best is None or candidate < best:
                best = candidate
        if best is None:
            raise CollectionError("no route segment is available in the projection window")
        return Projection(max(previous_progress_m, best[3]), best[0], best[2])

    def command_at(
        self, progress_m: float, lookahead_m: float, exit_lookahead_m: float
    ) -> int:
        progress_m = max(0.0, min(self.length_m, progress_m))
        current_index = len(self.points) - 1
        for index, point in enumerate(self.points):
            if float(point["distance_m"]) + 1.0e-6 >= progress_m:
                current_index = index
                break
        current_command = int(self.points[current_index]["vad_command"])
        if current_command in MANEUVER_COMMANDS:
            limit = progress_m + max(0.0, exit_lookahead_m)
            for point in self.points[current_index:]:
                if float(point["distance_m"]) > limit:
                    break
                if int(point["vad_command"]) not in MANEUVER_COMMANDS:
                    return int(point["vad_command"])
            return current_command
        limit = progress_m + max(0.0, lookahead_m)
        for point in self.points[current_index:]:
            if float(point["distance_m"]) > limit:
                break
            if int(point["vad_command"]) in MANEUVER_COMMANDS:
                return int(point["vad_command"])
        return 3


def base_link_state(
    center_transform: Mapping[str, float],
    world_velocity: Sequence[float],
    world_acceleration: Sequence[float],
    angular_velocity_z_degrees: float,
    wheelbase_m: float = WHEELBASE_M,
) -> dict[str, float]:
    base = shift_transform_local_x(center_transform, -wheelbase_m / 2.0)
    yaw_carla = math.radians(float(center_transform["yaw"]))
    cos_yaw = math.cos(yaw_carla)
    sin_yaw = math.sin(yaw_carla)

    def local_ros(vector: Sequence[float]) -> tuple[float, float]:
        local_x = cos_yaw * float(vector[0]) + sin_yaw * float(vector[1])
        local_y_carla = -sin_yaw * float(vector[0]) + cos_yaw * float(vector[1])
        return local_x, -local_y_carla

    vx, center_vy = local_ros(world_velocity)
    ax, ay = local_ros(world_acceleration)
    yaw_rate = -math.radians(float(angular_velocity_z_degrees))
    vy = center_vy - wheelbase_m / 2.0 * yaw_rate
    return {
        "x": base["x"],
        "y": -base["y"],
        "z": base["z"],
        "yaw": wrap_angle(-yaw_carla),
        "vx": vx,
        "vy": vy,
        "ax": ax,
        "ay": ay,
        "yaw_rate": yaw_rate,
    }


def catalog_goal_status(
    route_progress_m: float,
    route_length_m: float,
    base_link_x: float,
    base_link_y: float,
    final_route_x: float,
    final_route_y: float,
    tolerance_m: float,
) -> CatalogGoalStatus:
    """Evaluate the serialized route goal using planar base_link coordinates."""
    values = (
        route_progress_m,
        route_length_m,
        base_link_x,
        base_link_y,
        final_route_x,
        final_route_y,
        tolerance_m,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise CollectionError("catalog goal inputs must be finite")
    if route_progress_m < 0.0 or route_length_m < 0.0:
        raise CollectionError("catalog route progress and length must be non-negative")
    if tolerance_m <= 0.0:
        raise CollectionError("catalog goal tolerance must be positive")

    remaining_route_m = max(0.0, route_length_m - route_progress_m)
    planar_distance_m = math.hypot(
        base_link_x - final_route_x, base_link_y - final_route_y
    )
    return CatalogGoalStatus(
        reached=(
            remaining_route_m <= tolerance_m
            and planar_distance_m <= tolerance_m
        ),
        remaining_route_m=remaining_route_m,
        planar_distance_m=planar_distance_m,
    )


def termination_reason(
    *, basic_agent_done: bool, catalog_goal_reached: bool
) -> str | None:
    if catalog_goal_reached:
        return "catalog_goal_tolerance"
    if basic_agent_done:
        return "basic_agent_done_before_catalog_goal"
    return None


def control_dict(control: Any) -> dict[str, Any]:
    return {
        "throttle": float(getattr(control, "throttle", 0.0)),
        "steer": float(getattr(control, "steer", 0.0)),
        "brake": float(getattr(control, "brake", 0.0)),
        "hand_brake": bool(getattr(control, "hand_brake", False)),
        "reverse": bool(getattr(control, "reverse", False)),
        "gear": int(getattr(control, "gear", 0)),
        "manual_gear_shift": bool(getattr(control, "manual_gear_shift", False)),
    }


class AcknowledgedControlTransport:
    """HH_260906 - Acknowledge server command acceptance without claiming physical actuator application."""

    def __init__(self, client: Any, carla: Any, ego: Any, frame_reader: Any, persist_receipt: Any):
        if type(ego.id) is not int or ego.id <= 0:
            raise CollectionError("acknowledged transport requires one valid owned actor ID")
        self.client, self.carla, self.ego = client, carla, ego
        self.frame_reader, self.persist_receipt = frame_reader, persist_receipt
        self.receipts: list[dict[str, Any]] = []
        self.alignment_failures = 0

    @staticmethod
    def checked_control(control: Any) -> dict[str, Any]:
        # HH_260906 - Reject malformed controls before a batch can send them to the owned actor.
        for name in ("throttle", "brake", "steer"):
            value = getattr(control, name, None)
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
                raise CollectionError("acknowledged control has a nonfinite or invalid numeric field")
            if not (-1.0 <= value <= 1.0 if name == "steer" else 0.0 <= value <= 1.0):
                raise CollectionError("acknowledged control is outside normalized limits")
        if any(type(getattr(control, name, None)) is not bool for name in ("hand_brake", "reverse", "manual_gear_shift")):
            raise CollectionError("acknowledged control requires strict boolean flags")
        if type(getattr(control, "gear", None)) is not int:
            raise CollectionError("acknowledged control requires an integer gear")
        return control_dict(control)

    def send(self, control: Any, reason: str, expected_before_frame: int | None = None) -> dict[str, Any]:
        requested = self.checked_control(control)
        receipt = {"sequence": len(self.receipts) + 1, "mode": "acknowledged_batch", "actor_id": self.ego.id,
            "reason": reason, "requested_control": requested, "do_tick": False, "status": "FAILED",
            "server_accepted": False, "physical_actuation_proven": False}
        self.receipts.append(receipt)
        try:
            before = self.frame_reader()
            if type(before) is not int or before < 0:
                raise CollectionError("acknowledged command has an invalid before-frame")
            receipt["before_frame"] = before
            if expected_before_frame is not None:
                receipt["expected_before_frame"] = expected_before_frame
                if type(expected_before_frame) is not int or before != expected_before_frame:
                    raise CollectionError("command source frame changed before acknowledgement")
            responses = self.client.apply_batch_sync(
                [self.carla.command.ApplyVehicleControl(self.ego.id, control)], False)
            if not isinstance(responses, (list, tuple)) or len(responses) != 1:
                raise CollectionError("acknowledged command requires exactly one response")
            response = responses[0]
            if response.has_error() is not False or response.error != "":
                raise CollectionError("owned control batch failed: " + str(response.error))
            if type(response.actor_id) is not int or response.actor_id != self.ego.id:
                raise CollectionError("acknowledged command response actor ID mismatch")
            receipt["response_actor_id"] = response.actor_id
            receipt["server_accepted"] = True
            after = self.frame_reader()
            receipt["after_ack_frame"] = after
            if type(after) is not int or after != before:
                raise CollectionError("world frame changed during a no-tick acknowledged command")
            receipt["status"] = "ACKNOWLEDGED"
        except Exception as error:
            receipt["error"] = f"{type(error).__name__}: {error}"
            raise CollectionError("acknowledged control transport failed; no async fallback: " + str(error)) from error
        finally:
            # HH_260906 - Persist each success/failure receipt before the caller is allowed to advance a physics tick.
            self.persist_receipt(receipt)
        return receipt

    def compare_observation(self, control: Any, expected: Mapping[str, Any], frame: int,
                            before_control_frame: int, after_control_frame: int) -> dict[str, Any]:
        observed = self.checked_control(control)
        request = expected["requested_control"]
        mismatches = [name for name in ("throttle", "brake", "steer")
                      if abs(observed[name] - request[name]) > 1.0e-6]
        mismatches += [name for name in ("hand_brake", "reverse", "manual_gear_shift") if observed[name] != request[name]]
        if request["manual_gear_shift"] and observed["gear"] != request["gear"]:
            mismatches.append("gear")
        frame_bound = (type(frame) is int and type(before_control_frame) is int and type(after_control_frame) is int
                       and before_control_frame == after_control_frame == frame
                       and expected["status"] == "ACKNOWLEDGED" and expected["after_ack_frame"] == frame - 1)
        passed = frame_bound and not mismatches
        self.alignment_failures += int(not passed)
        return {"status": "PASS" if passed else "FAIL", "expected_receipt_sequence": expected["sequence"],
            "expected_command_before_frame": expected.get("before_frame"), "observed_frame": frame,
            "control_read_frame_before": before_control_frame, "control_read_frame_after": after_control_frame,
            "frame_binding_pass": frame_bound, "mismatched_fields": mismatches, "absolute_tolerance": 1.0e-6,
            "automatic_gear_equality_required": False, "lookback_relabeling_allowed": False,
            "physical_actuation_proven": False}


def acknowledged_snapshot_state(snapshot: Any, ego: Any, wheelbase_m: float) -> tuple[dict[str, float], dict[str, Any]]:
    """HH_260906 - Preserve one immutable actor snapshot's raw reference point without redefining legacy acceleration."""
    actor = snapshot.find(ego.id)
    if actor is None or actor.id != ego.id:
        raise CollectionError("owned ego is missing from the immutable frame snapshot")
    transform = _transform_dict(actor.get_transform())
    velocity = _vector_tuple(actor.get_velocity())
    acceleration = _vector_tuple(actor.get_acceleration())
    angular = _vector_tuple(actor.get_angular_velocity())
    if not all(math.isfinite(value) for value in (*transform.values(), *velocity, *acceleration, *angular)):
        raise CollectionError("immutable actor snapshot contains nonfinite measurements")
    raw = {"actor_snapshot_transform_carla": transform, "world_velocity_carla": velocity,
        "world_acceleration_carla": acceleration, "world_angular_velocity_carla_deg_s": angular,
        "raw_state_reference": "CARLA actor snapshot API reference point; physical COM/rear-axle identity is not independently verified."}
    return base_link_state(transform, velocity, acceleration, angular[2], wheelbase_m), raw


def measured_vehicle_fields(ego: Any, carla: Any) -> dict[str, float]:
    """Read physical steering and map speed-limit labels for the current tick."""
    steering = front_steering_measurement(
        float(
            ego.get_wheel_steer_angle(
                carla.VehicleWheelLocation.FL_Wheel
            )
        ),
        float(
            ego.get_wheel_steer_angle(
                carla.VehicleWheelLocation.FR_Wheel
            )
        ),
    )
    return {
        **steering,
        **speed_limit_measurement(float(ego.get_speed_limit())),
    }


def capture_interval(physics_hz: float, camera_hz: float) -> int:
    if not math.isfinite(physics_hz) or physics_hz <= 0.0:
        raise CollectionError("physics Hz must be positive and finite")
    if not math.isfinite(camera_hz) or camera_hz <= 0.0:
        raise CollectionError("camera Hz must be positive and finite")
    ratio = physics_hz / camera_hz
    interval = int(round(ratio))
    if interval < 1 or not math.isclose(ratio, interval, rel_tol=0.0, abs_tol=1.0e-9):
        raise CollectionError("physics Hz must be an integer multiple of camera Hz")
    return interval


def exact_camera_bundle(
    queues: Mapping[str, SimpleQueue], frame: int, timeout_sec: float
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_sec
    bundle = {}
    for name in MODEL_CAMERA_ORDER:
        queue = queues[name]
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise CollectionError(f"timed out waiting for {name} frame {frame}")
            try:
                image = queue.get(timeout=remaining)
            except Empty as error:
                raise CollectionError(f"timed out waiting for {name} frame {frame}") from error
            image_frame = int(image.frame)
            if image_frame < frame:
                continue
            if image_frame > frame:
                raise CollectionError(
                    f"camera {name} skipped frame {frame}; next frame is {image_frame}"
                )
            bundle[name] = image
            break
    timestamps = [float(image.timestamp) for image in bundle.values()]
    if max(timestamps) - min(timestamps) > 1.0e-6:
        raise CollectionError(f"camera frame {frame} is not timestamp-synchronous")
    return bundle


class EventRecorder:
    def __init__(self):
        self._lock = threading.Lock()
        self._collisions: dict[int, list[dict[str, Any]]] = defaultdict(list)
        self._lane_invasions: dict[int, list[dict[str, Any]]] = defaultdict(list)

    def on_collision(self, event: Any) -> None:
        actor = event.other_actor
        impulse = event.normal_impulse
        record = {
            "frame": int(event.frame),
            "timestamp": float(event.timestamp),
            "other_actor_id": int(actor.id),
            "other_actor_type": str(actor.type_id),
            "other_actor_role": str(actor.attributes.get("role_name", "")),
            "normal_impulse": {
                "x": float(impulse.x),
                "y": float(impulse.y),
                "z": float(impulse.z),
            },
            "intensity": math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2),
        }
        with self._lock:
            self._collisions[record["frame"]].append(record)

    def on_lane_invasion(self, event: Any) -> None:
        markings = []
        for marking in event.crossed_lane_markings:
            markings.append(
                {
                    "type": str(marking.type),
                    "color": str(marking.color),
                    "lane_change": str(marking.lane_change),
                }
            )
        record = {
            "frame": int(event.frame),
            "timestamp": float(event.timestamp),
            "crossed_lane_markings": markings,
        }
        with self._lock:
            self._lane_invasions[record["frame"]].append(record)

    def snapshot(self) -> tuple[dict[int, list[dict]], dict[int, list[dict]]]:
        with self._lock:
            return (
                {frame: list(events) for frame, events in self._collisions.items()},
                {frame: list(events) for frame, events in self._lane_invasions.items()},
            )


def _write_json(path: Path, value: Any) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=False) + "\n", encoding="utf-8"
    )
    os.replace(temporary, path)


def _write_jsonl(path: Path, records: Sequence[Mapping[str, Any]]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        for record in records:
            stream.write(json.dumps(record, separators=(",", ":"), sort_keys=False) + "\n")
    os.replace(temporary, path)


def finalize_output(partial: Path, output: Path) -> None:
    """Atomically promote a completed staging directory without overwriting data."""
    if output.exists():
        raise CollectionError(f"output already exists: {output}")
    if not partial.is_dir():
        raise CollectionError(f"partial output does not exist: {partial}")
    os.replace(partial, output)


def _settings_dict(settings: Any) -> dict[str, Any]:
    return {
        "synchronous_mode": bool(settings.synchronous_mode),
        "fixed_delta_seconds": settings.fixed_delta_seconds,
        "no_rendering_mode": bool(settings.no_rendering_mode),
        "substepping": bool(settings.substepping),
        "max_substep_delta_time": float(settings.max_substep_delta_time),
        "max_substeps": int(settings.max_substeps),
    }


def _transform_dict(transform: Any) -> dict[str, float]:
    return {
        "x": float(transform.location.x),
        "y": float(transform.location.y),
        "z": float(transform.location.z),
        "roll": float(transform.rotation.roll),
        "pitch": float(transform.rotation.pitch),
        "yaw": float(transform.rotation.yaw),
    }


def _vector_tuple(vector: Any) -> tuple[float, float, float]:
    return float(vector.x), float(vector.y), float(vector.z)


def _carla_transform(carla: Any, values: Mapping[str, float]) -> Any:
    return carla.Transform(
        carla.Location(x=values["x"], y=values["y"], z=values["z"]),
        carla.Rotation(
            roll=values["roll"], pitch=values["pitch"], yaw=values["yaw"]
        ),
    )


def _carla_location(carla: Any, values: Mapping[str, float]) -> Any:
    """Create an owned Location instead of borrowing it from a temporary Transform."""
    return carla.Location(x=values["x"], y=values["y"], z=values["z"])


def _configure_camera_blueprint(blueprint: Any, spec: CameraSpec) -> None:
    blueprint.set_attribute("image_size_x", str(spec.width))
    blueprint.set_attribute("image_size_y", str(spec.height))
    blueprint.set_attribute("fov", str(spec.fov_degrees))
    blueprint.set_attribute("sensor_tick", str(spec.sensor_tick))
    blueprint.set_attribute(
        "enable_postprocess_effects", str(spec.enable_postprocess_effects).lower()
    )


def _save_jpeg(image: Any, output: Path, quality: int) -> None:
    import cv2
    import numpy as np

    bgra = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(
        int(image.height), int(image.width), 4
    )
    success, encoded = cv2.imencode(
        ".jpg", bgra[:, :, :3], [cv2.IMWRITE_JPEG_QUALITY, quality]
    )
    if not success:
        raise CollectionError(f"failed to encode {output}")
    output.write_bytes(encoded.tobytes())


def _validate_image_geometry(image: Any, spec: CameraSpec) -> None:
    if int(image.width) != spec.width or int(image.height) != spec.height:
        raise CollectionError(
            f"camera {spec.name} frame {image.frame} has {image.width}x{image.height}; "
            f"expected {spec.width}x{spec.height}"
        )


def _preflight_exclusive(world: Any) -> None:
    actors = []
    for pattern in ("vehicle.*", "sensor.*"):
        actors.extend(world.get_actors().filter(pattern))
    if actors:
        details = ", ".join(
            f"id={actor.id},type={actor.type_id},role={actor.attributes.get('role_name', '')}"
            for actor in actors[:20]
        )
        raise CollectionError(f"CARLA world is already in use: {details}")


def _destroy_actor(actor: Any, errors: list[str]) -> bool:
    if actor is None:
        return True
    if str(getattr(actor, "type_id", "")).startswith("sensor."):
        try:
            actor.stop()
        except Exception as error:  # CARLA RPC errors must not prevent later cleanup.
            errors.append(f"stop actor {getattr(actor, 'id', '?')}: {error}")
    try:
        actor.destroy()
    except Exception as error:
        errors.append(f"destroy actor {getattr(actor, 'id', '?')}: {error}")
        return False
    return True


def _server_available_for_cleanup(
    client: Any, timeout_sec: float, errors: list[str]
) -> bool:
    """Bound cleanup latency when the CARLA process has already disappeared."""
    try:
        client.set_timeout(timeout_sec)
        client.get_world()
    except Exception as error:
        errors.append(f"cleanup server unavailable: {error}")
        return False
    return True


def _blank_brake_control(carla: Any) -> Any:
    return carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0)


def suppress_stopped_brake_steering(
    control: Any, speed_mps: float, enabled: bool, speed_threshold_mps: float = 0.3
) -> bool:
    """Prevent BasicAgent's lateral PID from winding up while a red light holds the car."""
    if (
        enabled
        and speed_mps < speed_threshold_mps
        and (float(getattr(control, "brake", 0.0)) > 0.05 or bool(getattr(control, "hand_brake", False)))
    ):
        control.steer = 0.0
        return True
    return False


def collect_episode(
    args: argparse.Namespace,
    route: Mapping[str, Any],
    specs: Sequence[CameraSpec],
    partial: Path,
    state_records: list[dict[str, Any]],
    camera_records: list[dict[str, Any]],
    manifest: dict[str, Any],
) -> None:
    import carla
    from agents.navigation.basic_agent import BasicAgent
    from agents.navigation.global_route_planner import GlobalRoutePlanner

    stationary_warmup_sec = float(getattr(args, "stationary_warmup_sec", 0.0))
    stationary_tail_sec = float(getattr(args, "stationary_tail_sec", 0.0))
    # HH_260906 - Resolve the fixed profile before changing any simulator state.
    goal_stop_config = configuration_from_args(args)
    goal_stop_bounds = source_motion_bounds() if goal_stop_config else None
    goal_stop_governor = (
        (DevelopmentGoalStopGovernor if isinstance(goal_stop_config, DevelopmentGoalStopConfig)
         else GoalStopGovernor)(goal_stop_config, args.target_speed_kmh / 3.6, args.physics_hz)
        if goal_stop_config else None
    )
    phase_schedule = capture_phase_schedule(
        args.physics_hz,
        args.max_duration_sec,
        stationary_warmup_sec,
        stationary_tail_sec,
    )
    phase_observations: dict[str, dict[str, Any]] = {
        phase: {
            "state_count": 0,
            "camera_anchor_count": 0,
            "first_timestamp": None,
            "last_timestamp": None,
        }
        for phase in CAPTURE_PHASE_ORDER
    }

    random.seed(args.seed)
    try:
        import numpy as np

        np.random.seed(args.seed)
    except ImportError:
        pass

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    _preflight_exclusive(world)
    requested_town = str(route["town"])
    current_town = world.get_map().name.rsplit("/", 1)[-1]
    map_load_performed = False
    if current_town != requested_town:
        if not args.allow_map_load:
            raise CollectionError(
                f"current CARLA map {current_town!r} does not match route map "
                f"{requested_town!r}; automatic client map loading is disabled, "
                "so cold-start CARLA on the required map or pass the explicit "
                "--allow-map-load opt-in"
            )
        world = client.load_world(requested_town)
        map_load_performed = True
        time.sleep(2.0)
        _preflight_exclusive(world)

    original_settings = world.get_settings()
    original_weather = world.get_weather()
    actors: list[Any] = []
    cleanup_errors: list[str] = []
    event_recorder = EventRecorder()
    queues = {name: SimpleQueue() for name in MODEL_CAMERA_ORDER}
    stop_requested = False
    previous_handlers: dict[int, Any] = {}
    acknowledged = None

    def persist_control_receipt(receipt: Mapping[str, Any]) -> None:
        # HH_260906 - This append-only journal is closed before any subsequent physics tick.
        with (partial / "control_receipts.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(receipt, sort_keys=True, allow_nan=False) + "\n")

    def send_control(control: Any, reason: str, expected_frame: int | None = None) -> dict[str, Any] | None:
        # HH_260906 - Every owned-ego command uses one selected transport, including bootstrap and abort commands.
        if acknowledged is None:
            ego.apply_control(control)
            return None
        return acknowledged.send(control, reason, expected_frame)

    def request_stop(signum: int, _frame: Any) -> None:
        nonlocal stop_requested
        stop_requested = True
        manifest["stop_signal"] = int(signum)

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.getsignal(signum)
        signal.signal(signum, request_stop)

    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0 / args.physics_hz
        world.apply_settings(settings)
        weather_name = args.weather or str(route.get("weather", "ClearNoon"))
        try:
            weather = getattr(carla.WeatherParameters, weather_name)
        except AttributeError as error:
            raise CollectionError(f"unknown CARLA weather {weather_name!r}") from error
        world.set_weather(weather)
        if hasattr(world, "set_pedestrians_seed"):
            world.set_pedestrians_seed(args.seed)

        blueprint_library = world.get_blueprint_library()
        vehicle_blueprint = blueprint_library.find(args.vehicle_type)
        if vehicle_blueprint.has_attribute("role_name"):
            vehicle_blueprint.set_attribute("role_name", args.role_name)
        if vehicle_blueprint.has_attribute("color"):
            colors = vehicle_blueprint.get_attribute("color").recommended_values
            if colors:
                vehicle_blueprint.set_attribute("color", colors[args.seed % len(colors)])

        center_start = shift_transform_local_x(
            route["start_carla_transform"], args.wheelbase_m / 2.0
        )
        actor_spawn = apply_spawn_z_offset(center_start, args.spawn_z_offset_m)
        ego = world.try_spawn_actor(
            vehicle_blueprint, _carla_transform(carla, actor_spawn)
        )
        if ego is None:
            raise CollectionError(f"failed to spawn {args.vehicle_type} at route start")
        actors.append(ego)
        if getattr(args, "control_transport", "legacy_async") == "acknowledged_batch":
            acknowledged = AcknowledgedControlTransport(
                client, carla, ego, lambda: world.get_snapshot().frame, persist_control_receipt)

        for spec in specs:
            blueprint = blueprint_library.find(spec.carla_type)
            _configure_camera_blueprint(blueprint, spec)
            sensor = world.spawn_actor(
                blueprint,
                _carla_transform(carla, spec.carla_extrinsic),
                attach_to=ego,
                attachment_type=carla.AttachmentType.Rigid,
            )
            sensor.listen(queues[spec.name].put)
            actors.append(sensor)

        collision = world.spawn_actor(
            blueprint_library.find("sensor.other.collision"),
            carla.Transform(),
            attach_to=ego,
            attachment_type=carla.AttachmentType.Rigid,
        )
        collision.listen(event_recorder.on_collision)
        actors.append(collision)
        lane_invasion = world.spawn_actor(
            blueprint_library.find("sensor.other.lane_invasion"),
            carla.Transform(),
            attach_to=ego,
            attachment_type=carla.AttachmentType.Rigid,
        )
        lane_invasion.listen(event_recorder.on_lane_invasion)
        actors.append(lane_invasion)

        carla_map = world.get_map()
        sampling_resolution = float(route.get("sampling_resolution_m", 1.0))
        basic_agent_options, basic_agent_control = basic_agent_control_configuration(
            args, sampling_resolution
        )
        manifest.setdefault("capture_contract", {})[
            "basic_agent_control"
        ] = basic_agent_control
        planner = GlobalRoutePlanner(carla_map, sampling_resolution)
        goal_center = shift_transform_local_x(
            route["goal_carla_transform"], args.wheelbase_m / 2.0
        )
        # This CARLA build reports (0, 0, 0) from ego.get_location() until the
        # first world tick. The route contract already gives the exact spawn pose.
        start_location = _carla_location(carla, center_start)
        goal_location = _carla_location(carla, goal_center)
        plan = planner.trace_route(start_location, goal_location)
        if len(plan) < 2:
            raise CollectionError("BasicAgent global plan contains fewer than two points")
        if goal_stop_config:
            # HH_260906 - GRP can end up to two samples early, before the exact catalog stop window.
            plan, terminal_plan = complete_terminal_plan(
                plan, carla_map.get_waypoint(goal_location),
                {name: float(goal_center[name]) for name in ("x", "y", "z")}, sampling_resolution,
            )
            terminal_plan["global_route_planner_source"] = _python_class_source_provenance(planner)
            manifest["capture_contract"]["goal_stop_profile"]["terminal_plan"] = terminal_plan
        agent = BasicAgent(
            ego,
            target_speed=args.target_speed_kmh,
            opt_dict=basic_agent_options,
            map_inst=carla_map,
            grp_inst=planner,
        )
        agent.set_global_plan(plan)
        local_planner = agent.get_local_planner()
        if goal_stop_config:
            # HH_260906 - Hazard braking remains BasicAgent's original emergency override.
            goal_stop_control = (install_development_control(agent, goal_stop_governor)
                                 if isinstance(goal_stop_governor, DevelopmentGoalStopGovernor)
                                 else install_normal_brake_cap(
                                     local_planner, goal_stop_config.normal_brake_cap,
                                     goal_stop_config.normal_throttle_cap))
            goal_stop_control["unchanged_basic_agent_emergency_brake"] = float(agent._max_brake)
            manifest["capture_contract"]["goal_stop_profile"]["effective_control"] = goal_stop_control

        # CARLA 0.9.15 can report the actor at the origin until its first world
        # tick.  Advance one disclosed, unrecorded setup tick under full brake so
        # every state labelled "driving" was produced by a BasicAgent control.
        bootstrap_control = _blank_brake_control(carla)
        send_control(bootstrap_control, "pre_capture_bootstrap")
        bootstrap_frame = int(world.tick(args.timeout))
        bootstrap_snapshot = world.get_snapshot()
        if int(bootstrap_snapshot.frame) != bootstrap_frame:
            raise CollectionError(
                "world snapshot frame does not match pre-capture bootstrap tick"
            )
        if acknowledged is not None:
            bootstrap_state, bootstrap_raw = acknowledged_snapshot_state(bootstrap_snapshot, ego, args.wheelbase_m)
            before_control = world.get_snapshot().frame
            observed_control = ego.get_control()
            after_control = world.get_snapshot().frame
            bootstrap_alignment = acknowledged.compare_observation(
                observed_control, acknowledged.receipts[-1], bootstrap_frame, before_control, after_control)
            manifest["capture_contract"]["control_transport"]["bootstrap_observation"] = {
                "frame": bootstrap_frame, "timestamp": float(bootstrap_snapshot.timestamp.elapsed_seconds),
                "recorded_training_state": False, **bootstrap_state, **bootstrap_raw,
                "current_control": control_dict(observed_control), "alignment": bootstrap_alignment}
            if bootstrap_alignment["status"] != "PASS":
                raise CollectionError("acknowledged bootstrap control/frame alignment failed")
        capture_origin_timestamp = float(
            bootstrap_snapshot.timestamp.elapsed_seconds
        )

        projector = RouteProjector(route)
        final_route_point = route["route"][-1]
        progress_m = 0.0
        interval = capture_interval(args.physics_hz, args.capture_hz)
        tick_index = 0
        image_root = partial / "images"
        for name in MODEL_CAMERA_ORDER:
            (image_root / name).mkdir(parents=True, exist_ok=False)

        manifest["runtime"] = {
            "carla_version": getattr(carla, "__version__", "unknown"),
            "server_version": client.get_server_version(),
            "client_version": client.get_client_version(),
            "town": carla_map.name.rsplit("/", 1)[-1],
            "weather": weather_name,
            "client_map_loading_allowed": args.allow_map_load,
            "client_map_loading_performed": map_load_performed,
            "original_world_settings": _settings_dict(original_settings),
            "capture_world_settings": _settings_dict(world.get_settings()),
            "vehicle_type": args.vehicle_type,
            "role_name": args.role_name,
            **_spawn_runtime_fields(
                center_start, actor_spawn, args.spawn_z_offset_m
            ),
            "goal_center_carla": goal_center,
            "catalog_goal_base_link": {
                "x": float(final_route_point["x"]),
                "y": float(final_route_point["y"]),
            },
            "basic_agent_plan_points": len(plan),
            "basic_agent_control": {
                "effective_opt_dict": basic_agent_options,
                "implementation_sources": {
                    "basic_agent": _python_class_source_provenance(BasicAgent),
                    "local_planner": _python_class_source_provenance(local_planner),
                    "vehicle_controller": _python_class_source_provenance(
                        local_planner._vehicle_controller
                    ),
                },
            },
            "capture_origin_timestamp": capture_origin_timestamp,
            "pre_capture_bootstrap": {
                "frame": bootstrap_frame,
                "timestamp": capture_origin_timestamp,
                "control": control_dict(bootstrap_control),
                "recorded": False,
                "included_in_maximum_total_duration": False,
                "reason": "initialize CARLA actor state before BasicAgent control",
            },
            "capture_phase_schedule": phase_schedule,
        }
        _write_json(partial / "manifest.json", manifest)

        def tick_and_record(phase: str) -> tuple[CatalogGoalStatus, bool, str | None]:
            nonlocal progress_m, tick_index
            if stop_requested:
                raise CollectionInterrupted("capture interrupted by signal")
            if tick_index >= int(phase_schedule["maximum_total_ticks"]):
                raise CollectionError(
                    "capture reached the tick-exact maximum total duration"
                )
            expected_receipt = acknowledged.receipts[-1] if acknowledged is not None else None
            frame = int(world.tick(args.timeout))
            snapshot = world.get_snapshot()
            if int(snapshot.frame) != frame:
                raise CollectionError(
                    f"world snapshot frame {snapshot.frame} does not match tick {frame}"
                )
            timestamp = float(snapshot.timestamp.elapsed_seconds)

            raw_snapshot_fields = {}
            if acknowledged is not None:
                state, raw_snapshot_fields = acknowledged_snapshot_state(snapshot, ego, args.wheelbase_m)
            else:
                transform = ego.get_transform()
                state = base_link_state(
                    _transform_dict(transform),
                    _vector_tuple(ego.get_velocity()),
                    _vector_tuple(ego.get_acceleration()),
                    float(ego.get_angular_velocity().z),
                    args.wheelbase_m,
                )
            vehicle_fields = measured_vehicle_fields(ego, carla)
            if goal_stop_config:
                # HH_260906 - The legacy 80 m nearest-segment search can jump across close loops.
                projection = Projection(*bounded_route_projection(
                    projector.points, state["x"], state["y"], progress_m,
                    goal_stop_config.maximum_projection_step_m,
                ))
            else:
                projection = projector.project(state["x"], state["y"], progress_m)
            progress_m = projection.progress_m
            command = projector.command_at(
                progress_m, args.command_lookahead_m, args.command_exit_lookahead_m
            )
            before_control = world.get_snapshot().frame if acknowledged is not None else None
            current_control = ego.get_control()
            alignment = None
            if acknowledged is not None:
                alignment = acknowledged.compare_observation(
                    current_control, expected_receipt, frame, before_control, world.get_snapshot().frame)
            goal_status = catalog_goal_status(
                progress_m,
                projector.length_m,
                state["x"],
                state["y"],
                float(final_route_point["x"]),
                float(final_route_point["y"]),
                args.goal_tolerance_m,
            )
            basic_agent_done = bool(agent.done())
            goal_stop_record = None
            stop_reason = (
                termination_reason(
                    basic_agent_done=basic_agent_done,
                    catalog_goal_reached=goal_status.reached,
                )
                if phase == "driving"
                else None
            )
            if goal_stop_governor:
                # HH_260906 - A position-only goal at cruising speed must not trigger the full-brake tail.
                pilot_measurement = ({"timestamp": timestamp, "longitudinal_speed_mps": state["vx"],
                                      "driving": phase == "driving"}
                                     if isinstance(goal_stop_governor, DevelopmentGoalStopGovernor) else {})
                goal_stop_record = goal_stop_governor.update(
                    goal_status.remaining_route_m, goal_status.planar_distance_m,
                    math.hypot(state["vx"], state["vy"]),
                    terminal_overshoot_m(projector.points, state["x"], state["y"]),
                    count_hold=phase == "driving",
                    **pilot_measurement,
                )
                stop_reason = None
                if phase == "driving":
                    agent.set_target_speed(goal_stop_record["target_speed_mps"] * 3.6)
                    stop_reason = goal_stop_termination_reason(
                        goal_stop_record, basic_agent_done, projection.cross_track_error_m, goal_stop_config,
                    )
                else:
                    # HH_260906 - Setup and tail use explicit brake, not a latent speed command.
                    goal_stop_governor.target_speed_mps = 0.0
                    goal_stop_record["target_speed_mps"] = 0.0
                goal_status = CatalogGoalStatus(
                    measured_goal_completion(goal_stop_record, projection.cross_track_error_m, goal_stop_config),
                    goal_status.remaining_route_m, goal_status.planar_distance_m,
                )
            next_control = (
                agent.run_step()
                if phase == "driving" and stop_reason is None
                else _blank_brake_control(carla)
            )
            if goal_stop_record is not None:
                goal_stop_record["basic_agent_done"] = basic_agent_done
                goal_stop_record["termination_reason"] = stop_reason
                goal_stop_record["control_source"] = (
                    "BasicAgent_with_route_arc_target_and_normal_PID_cap_emergency_unchanged"
                    if phase == "driving" and stop_reason is None
                    else "full_brake_measured_stop_tail_or_setup" if phase != "driving" or goal_status.reached
                    else "full_brake_failed_capture_abort_not_comfortable_data"
                )
                if isinstance(goal_stop_governor, DevelopmentGoalStopGovernor):
                    # HH_260906 - A newly requested hazard brake is applied now and observed before next-tick failure.
                    if phase == "driving" and stop_reason is None:
                        # HH_260906 - Keep legacy v3 provenance exact while naming the new brake-only revision correctly.
                        version = goal_stop_governor.config.profile_id.removeprefix("comfortable_")
                        goal_stop_record["control_source"] = f"BasicAgent_lateral_and_emergency_with_development_{version}_normal_longitudinal"
                    annotate_development_control(goal_stop_record, goal_stop_governor, stop_reason)
            if not (isinstance(goal_stop_governor, DevelopmentGoalStopGovernor)
                    and goal_stop_record["next_control_is_unchanged_emergency_override"]):
                # HH_260906 - The development hazard hook promises the unchanged emergency control, including steering.
                suppress_stopped_brake_steering(
                    next_control,
                    math.hypot(state["vx"], state["vy"]),
                    enabled=not args.allow_stopped_steering,
                )
            state_records.append(
                {
                    "frame": frame,
                    "timestamp": timestamp,
                    "capture_phase": phase,
                    **state,
                    **vehicle_fields,
                    "command": command,
                    "route_progress_m": progress_m,
                    "route_cte_m": projection.cross_track_error_m,
                    "current_control": control_dict(current_control),
                    "next_control": control_dict(next_control),
                    "collision": [],
                    "lane_invasion": [],
                }
            )
            if goal_stop_record is not None:
                state_records[-1]["goal_stop"] = goal_stop_record
            if acknowledged is not None:
                state_records[-1].update(raw_snapshot_fields)
                state_records[-1]["control_transport"] = alignment
                if alignment["status"] != "PASS":
                    # HH_260906 - Retain the mismatched observation, never relabel it using an older command.
                    alignment["unapplied_planned_next_control"] = state_records[-1]["next_control"]
                    next_control = _blank_brake_control(carla)
                    state_records[-1]["next_control"] = control_dict(next_control)
                    alignment["abort_full_brake_requested"] = True
            observation = phase_observations[phase]
            observation["state_count"] += 1
            if observation["first_timestamp"] is None:
                observation["first_timestamp"] = timestamp
            observation["last_timestamp"] = timestamp

            if tick_index % interval == 0:
                bundle = exact_camera_bundle(queues, frame, args.sensor_timeout_sec)
                image_paths = {}
                source_timestamps = {}
                for name in MODEL_CAMERA_ORDER:
                    spec = specs[MODEL_CAMERA_ORDER.index(name)]
                    _validate_image_geometry(bundle[name], spec)
                    relative = Path("images") / name / f"{frame:08d}.jpg"
                    _save_jpeg(bundle[name], partial / relative, args.jpeg_quality)
                    image_paths[name] = relative.as_posix()
                    source_timestamps[name] = float(bundle[name].timestamp)
                timestamps = tuple(source_timestamps.values())
                camera_records.append(
                    {
                        "frame": frame,
                        "timestamp": timestamp,
                        "capture_phase": phase,
                        "camera_order": list(MODEL_CAMERA_ORDER),
                        "images": image_paths,
                        "source_timestamps": source_timestamps,
                        "timestamp_span_sec": max(timestamps) - min(timestamps),
                        "jpeg_quality": args.jpeg_quality,
                    }
                )
                observation["camera_anchor_count"] += 1

            try:
                send_control(next_control, "alignment_failure_abort" if alignment is not None and alignment["status"] != "PASS"
                             else "next_" + phase + "_control", frame)
            finally:
                if acknowledged is not None and acknowledged.receipts:
                    state_records[-1]["control_transport"]["next_command_receipt_sequence"] = acknowledged.receipts[-1]["sequence"]
            tick_index += 1
            if alignment is not None and alignment["status"] != "PASS":
                raise CollectionError("acknowledged control/frame alignment failed; mismatched row retained")
            if (isinstance(goal_stop_governor, DevelopmentGoalStopGovernor)
                    and phase != "driving" and goal_stop_governor.failure_reason is not None):
                # HH_260906 - Retain the offending setup/tail measurement and apply the stop command before aborting.
                raise CollectionError(goal_stop_governor.failure_reason)
            return goal_status, basic_agent_done, stop_reason

        if int(phase_schedule["stationary_warmup"]["scheduled_ticks"]):
            send_control(_blank_brake_control(carla), "stationary_warmup_start", bootstrap_frame)
        for _ in range(int(phase_schedule["stationary_warmup"]["scheduled_ticks"])):
            tick_and_record("stationary_warmup")

        if goal_stop_governor:
            # HH_260906 - Ramp the initial command too; do not insert a full-speed first tick.
            goal_stop_governor.target_speed_mps = 0.0
            initial_state = state_records[-1] if state_records else (
                bootstrap_state if acknowledged is not None else base_link_state(
                    _transform_dict(ego.get_transform()), _vector_tuple(ego.get_velocity()),
                    _vector_tuple(ego.get_acceleration()), float(ego.get_angular_velocity().z), args.wheelbase_m,
                ))
            initial_goal_stop = goal_stop_governor.update(
                max(0.0, projector.length_m - progress_m),
                math.hypot(initial_state["x"] - float(final_route_point["x"]),
                           initial_state["y"] - float(final_route_point["y"])),
                math.hypot(initial_state["vx"], initial_state["vy"]),
                terminal_overshoot_m(projector.points, initial_state["x"], initial_state["y"]),
                count_hold=False,
                **({"timestamp": state_records[-1]["timestamp"],
                    "longitudinal_speed_mps": initial_state["vx"], "driving": True}
                   if isinstance(goal_stop_governor, DevelopmentGoalStopGovernor) else {}),
            )
            if isinstance(goal_stop_governor, DevelopmentGoalStopGovernor) and goal_stop_governor.failure_reason is not None:
                send_control(_blank_brake_control(carla), "initial_governor_failure_abort")
                raise CollectionError(goal_stop_governor.failure_reason)
            agent.set_target_speed(initial_goal_stop["target_speed_mps"] * 3.6)
        initial_drive_control = agent.run_step()
        velocity = (state_records[-1]["world_velocity_carla"] if state_records else bootstrap_raw["world_velocity_carla"]
                    ) if acknowledged is not None else _vector_tuple(ego.get_velocity())
        if not (isinstance(goal_stop_governor, DevelopmentGoalStopGovernor)
                and goal_stop_governor.failure_reason == f"{goal_stop_governor.config.profile_id}_emergency_override"):
            # HH_260906 - Initial engagement also preserves the exact emergency return before the following measured tick.
            suppress_stopped_brake_steering(
                initial_drive_control,
                math.sqrt(sum(component * component for component in velocity)),
                enabled=not args.allow_stopped_steering,
            )
        send_control(initial_drive_control, "initial_drive_control", state_records[-1]["frame"] if state_records else bootstrap_frame)
        if state_records and state_records[-1]["capture_phase"] == "stationary_warmup":
            state_records[-1]["next_control"] = control_dict(initial_drive_control)
            if acknowledged is not None:
                state_records[-1]["control_transport"]["next_command_receipt_sequence"] = acknowledged.receipts[-1]["sequence"]
            if goal_stop_governor:
                # HH_260906 - The final warmup state's next command actually starts driving.
                state_records[-1]["goal_stop"].update(initial_goal_stop)
                state_records[-1]["goal_stop"]["control_source"] = (
                    "BasicAgent_initial_drive_command_with_route_arc_target_emergency_unchanged"
                )
                state_records[-1]["goal_stop"]["next_control_starts_driving"] = True
                if isinstance(goal_stop_governor, DevelopmentGoalStopGovernor):
                    annotate_development_control(state_records[-1]["goal_stop"], goal_stop_governor, None)

        goal_status: CatalogGoalStatus | None = None
        basic_agent_done = False
        stop_reason: str | None = None
        while True:
            if (
                int(phase_observations["driving"]["state_count"])
                >= int(phase_schedule["driving"]["maximum_ticks"])
            ):
                raise CollectionError(
                    "BasicAgent did not finish within the driving budget reserved by "
                    "the maximum total duration and stationary tail"
                )
            goal_status, basic_agent_done, stop_reason = tick_and_record("driving")
            if stop_reason is not None:
                break

        send_control(_blank_brake_control(carla), "driving_end_brake", state_records[-1]["frame"] if state_records else None)
        if acknowledged is not None and state_records:
            state_records[-1]["next_control"] = control_dict(_blank_brake_control(carla))
            state_records[-1]["control_transport"]["next_command_receipt_sequence"] = acknowledged.receipts[-1]["sequence"]
        if goal_status is None:
            raise CollectionError("driving phase produced no state")
        termination_route_progress_m = progress_m
        if goal_status.reached:
            for _ in range(int(phase_schedule["stationary_tail"]["scheduled_ticks"])):
                tick_and_record("stationary_tail")

        phase_results = summarize_capture_phases(
            phase_schedule, phase_observations, args.physics_hz
        )
        manifest["result"] = {
            "goal_reached": goal_status.reached,
            "termination_reason": stop_reason,
            "basic_agent_done": basic_agent_done,
            "state_count": len(state_records),
            "camera_anchor_count": len(camera_records),
            "duration_sec": state_records[-1]["timestamp"] - state_records[0]["timestamp"],
            "elapsed_sim_sec": phase_results["observed_total_elapsed_sim_sec"],
            "capture_phases": phase_results,
            "final_route_progress_m": termination_route_progress_m,
            "post_capture_route_progress_m": progress_m,
            "route_length_m": projector.length_m,
            "final_remaining_route_m": goal_status.remaining_route_m,
            "final_catalog_goal_distance_m": goal_status.planar_distance_m,
            "goal_tolerance_m": args.goal_tolerance_m,
        }
        if isinstance(goal_stop_config, DevelopmentGoalStopConfig):
            manifest["result"].update({"training_data_approved": False, "development_only": True})
            # HH_260906 - A failed pilot still reports measured cruise and speed checks without requiring a tail.
            manifest["result"]["goal_stop_quality"] = measured_stop_quality(
                state_records, [r["frame"] for r in camera_records], goal_stop_config,
                goal_stop_bounds, goal_status.reached,
            )
        if not goal_status.reached:
            if isinstance(goal_stop_config, DevelopmentGoalStopConfig):
                raise CollectionError(f"{goal_stop_config.profile_id} development pilot failed: {stop_reason}")
            raise CollectionError(
                "BasicAgent reported done before the catalog goal: "
                f"remaining route {goal_status.remaining_route_m:.3f} m, "
                f"planar base_link distance {goal_status.planar_distance_m:.3f} m "
                f"(tolerance {args.goal_tolerance_m:.3f} m)"
            )
        if goal_stop_config:
            # HH_260906 - Reject measured violations instead of claiming the requested cap worked.
            quality = (manifest["result"]["goal_stop_quality"]
                       if isinstance(goal_stop_config, DevelopmentGoalStopConfig)
                       else measured_stop_quality(
                           state_records, [r["frame"] for r in camera_records], goal_stop_config,
                           goal_stop_bounds, goal_status.reached))
            manifest["result"]["goal_stop_quality"] = quality
            if quality["status"] != "PASS":
                raise CollectionError("comfortable goal-stop measured quality failed; preserve partial evidence")
    finally:
        if acknowledged is not None and sys.exc_info()[0] is not None:
            try:
                send_control(_blank_brake_control(carla), "exception_cleanup_abort")
            except Exception as error:
                # HH_260906 - Failed emergency acceptance is disclosed and must not skip actor cleanup.
                cleanup_errors.append(f"acknowledged abort command failed: {error}")
        server_available = _server_available_for_cleanup(
            client, min(args.timeout, 2.0), cleanup_errors
        )
        if server_available:
            for index, actor in enumerate(reversed(actors)):
                if not _destroy_actor(actor, cleanup_errors):
                    remaining = len(actors) - index - 1
                    if remaining:
                        cleanup_errors.append(
                            f"skipped {remaining} remaining actor(s) after cleanup RPC failure"
                        )
                    server_available = False
                    break
        collisions, lane_invasions = event_recorder.snapshot()
        for state in state_records:
            state["collision"] = collisions.get(state["frame"], [])
            state["lane_invasion"] = lane_invasions.get(state["frame"], [])
        result = manifest.setdefault("result", {})
        if acknowledged is not None:
            # HH_260906 - A missing receipt journal must fail evidence checks without skipping world or signal restoration.
            receipt_journal_sha256 = None
            receipt_journal_error = None
            try:
                receipt_journal_sha256 = sha256_file(partial / "control_receipts.jsonl")
            except Exception as error:
                receipt_journal_error = f"{type(error).__name__}: {error}"
                cleanup_errors.append(f"control receipt journal verification failed: {receipt_journal_error}")
            result["control_transport"] = {
                "mode": "acknowledged_batch", "command_receipt_count": len(acknowledged.receipts),
                "acknowledged_command_count": sum(receipt["status"] == "ACKNOWLEDGED" for receipt in acknowledged.receipts),
                "failed_command_count": sum(receipt["status"] != "ACKNOWLEDGED" for receipt in acknowledged.receipts),
                "control_alignment_failure_count": acknowledged.alignment_failures,
                "receipt_journal_sha256": receipt_journal_sha256,
                "receipt_journal_error": receipt_journal_error,
                "physical_actuation_proven": False,
            }
        result["collision_event_count"] = sum(map(len, collisions.values()))
        result["lane_invasion_event_count"] = sum(map(len, lane_invasions.values()))
        result["capture_phases"] = summarize_capture_phases(
            phase_schedule, phase_observations, args.physics_hz
        )
        if goal_stop_config:
            # HH_260906 - Failed and interrupted captures retain the same measured quality ledger.
            try:
                result["goal_stop_quality"] = measured_stop_quality(
                    state_records, [r["frame"] for r in camera_records], goal_stop_config,
                    goal_stop_bounds, bool(result.get("goal_reached")),
                )
            except Exception as quality_error:
                # HH_260906 - A bad quality record must never skip world restoration or actor cleanup.
                result["goal_stop_quality"] = {"status": "FAIL", "error": str(quality_error)}
                cleanup_errors.append(f"goal-stop quality failed: {quality_error}")
        if server_available:
            try:
                world.set_weather(original_weather)
            except Exception as error:
                cleanup_errors.append(f"restore weather: {error}")
            try:
                world.apply_settings(original_settings)
            except Exception as error:
                cleanup_errors.append(f"restore world settings: {error}")
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        manifest["cleanup"] = {
            "completed": not cleanup_errors,
            "server_available": server_available,
            "errors": cleanup_errors,
        }
        if cleanup_errors and sys.exc_info()[0] is None:
            raise CollectionError("CARLA cleanup failed: " + "; ".join(cleanup_errors))


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Collect a six-camera CARLA VAD dataset driven by BasicAgent.",
        # HH_260906 - Reject abbreviated host/port/map-load flags before an ownership wrapper launches CARLA.
        allow_abbrev=False,
    )
    parser.add_argument("output", type=Path, help="final dataset directory")
    parser.add_argument("route_file", type=Path, help="route JSON from prepare_carla_route.py")
    parser.add_argument("--host", default=os.environ.get("CARLA_HOST", "127.0.0.1"))
    parser.add_argument(
        "--port", type=int, default=int(os.environ.get("CARLA_PORT", "2000"))
    )
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--physics-hz", type=float, default=20.0)
    parser.add_argument("--capture-hz", type=float, default=10.0)
    parser.add_argument("--target-speed-kmh", type=float, default=9.0)
    parser.add_argument("--max-duration-sec", type=float, default=180.0)
    parser.add_argument(
        "--stationary-warmup-sec",
        type=float,
        default=0.0,
        help="simulation seconds recorded under full brake before BasicAgent drives",
    )
    parser.add_argument(
        "--stationary-tail-sec",
        type=float,
        default=0.0,
        help="simulation seconds recorded under full brake after the catalog goal",
    )
    parser.add_argument("--sensor-timeout-sec", type=float, default=5.0)
    parser.add_argument("--jpeg-quality", type=int, default=95)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--weather")
    parser.add_argument(
        "--allow-map-load",
        action="store_true",
        help=(
            "UNSAFE opt-in: permit client.load_world when the running map does not "
            "match the route; the default requires an exact cold-started map"
        ),
    )
    parser.add_argument("--vehicle-type", default="vehicle.toyota.prius")
    parser.add_argument("--role-name", default="autoware_e2e_expert")
    # HH_260906 - The transport experiment leaves all historical commands and output dictionaries unchanged by default.
    parser.add_argument("--control-transport", choices=("legacy_async", "acknowledged_batch"), default="legacy_async",
                        help="opt-in acknowledged no-tick batch commands with immutable snapshot and strict control/frame alignment")
    parser.add_argument("--wheelbase-m", type=float, default=WHEELBASE_M)
    parser.add_argument("--spawn-z-offset-m", type=float, default=0.0)
    parser.add_argument("--command-lookahead-m", type=float, default=2.0)
    parser.add_argument("--command-exit-lookahead-m", type=float, default=2.5)
    parser.add_argument("--goal-tolerance-m", type=float, default=2.5)
    # HH_260906 - This experimental goal-stop governor is never enabled by an old command.
    # HH_260906 - comfortable_v4 isolates zero normal brake under acknowledged transport; default and emergency behavior are unchanged.
    parser.add_argument("--goal-stop-profile", choices=("disabled", "comfortable_v1", "comfortable_v2", "comfortable_v3", "comfortable_v4"), default="disabled",
                        help="opt-in measured goal stop; requires 20/10 Hz, <=30 km/h, 1 m tolerance and >=6.5 s tail")
    parser.add_argument(
        "--basic-agent-base-min-distance-m",
        type=float,
        default=BASIC_AGENT_CONTROL_DEFAULTS["base_min_distance_m"],
        help=(
            "base waypoint-purge lookahead in metres; CARLA LocalPlanner option "
            "base_min_distance"
        ),
    )
    parser.add_argument(
        "--basic-agent-distance-ratio",
        type=float,
        default=BASIC_AGENT_CONTROL_DEFAULTS["distance_ratio_s"],
        help=(
            "speed-scaled waypoint-purge ratio in seconds; lookahead adds this "
            "value times vehicle speed in m/s"
        ),
    )
    parser.add_argument(
        "--basic-agent-lateral-kp",
        type=float,
        default=BASIC_AGENT_CONTROL_DEFAULTS["lateral_pid_kp"],
    )
    parser.add_argument(
        "--basic-agent-lateral-ki",
        type=float,
        default=BASIC_AGENT_CONTROL_DEFAULTS["lateral_pid_ki"],
    )
    parser.add_argument(
        "--basic-agent-lateral-kd",
        type=float,
        default=BASIC_AGENT_CONTROL_DEFAULTS["lateral_pid_kd"],
    )
    parser.add_argument(
        "--basic-agent-max-steering",
        type=float,
        default=BASIC_AGENT_CONTROL_DEFAULTS["max_steering"],
        help="maximum absolute normalized CARLA steering command in (0, 1]",
    )
    parser.add_argument(
        "--basic-agent-lane-offset-m",
        type=float,
        default=BASIC_AGENT_CONTROL_DEFAULTS["lane_offset_m"],
        help="lateral lane offset in metres; positive is right and negative is left",
    )
    parser.add_argument(
        "--allow-stopped-steering",
        action="store_true",
        help="disable the default BasicAgent red-light lateral windup guard",
    )
    parser.add_argument("--mapping", type=Path, default=DEFAULT_MAPPING)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    args = parser.parse_args(argv)
    if args.port <= 0 or args.timeout <= 0.0 or args.sensor_timeout_sec <= 0.0:
        parser.error("port and timeouts must be positive")
    if args.target_speed_kmh <= 0.0 or args.max_duration_sec <= 0.0:
        parser.error("target speed and maximum duration must be positive")
    if args.wheelbase_m <= 0.0:
        parser.error("wheelbase must be positive")
    if not math.isfinite(args.spawn_z_offset_m) or args.spawn_z_offset_m < 0.0:
        parser.error("spawn-z-offset-m must be finite and non-negative")
    if not 1 <= args.jpeg_quality <= 100:
        parser.error("JPEG quality must be in [1, 100]")
    if args.command_lookahead_m < 0.0 or args.command_exit_lookahead_m < 0.0:
        parser.error("command lookaheads must be non-negative")
    if not math.isfinite(args.goal_tolerance_m) or args.goal_tolerance_m <= 0.0:
        parser.error("goal-tolerance-m must be positive and finite")
    try:
        capture_interval(args.physics_hz, args.capture_hz)
        configuration_from_args(args)
        basic_agent_control_configuration(args, sampling_resolution_m=1.0)
        args.capture_phase_schedule = capture_phase_schedule(
            args.physics_hz,
            args.max_duration_sec,
            args.stationary_warmup_sec,
            args.stationary_tail_sec,
        )
    except (CollectionError, ValueError) as error:
        parser.error(str(error))
    return args


def run(args: argparse.Namespace) -> Path:
    output = args.output.expanduser().resolve()
    partial = Path(str(output) + ".partial")
    if output.exists():
        raise CollectionError(f"output already exists: {output}")
    if partial.exists():
        raise CollectionError(f"partial output already exists: {partial}")
    route_path = args.route_file.expanduser().resolve()
    mapping_path = args.mapping.expanduser().resolve()
    calibration_path = args.calibration.expanduser().resolve()
    route = load_route(route_path)
    # HH_260906 - The new pilot cannot be redirected to a different route or weather before source-bound qualification.
    validate_development_route(args, route, route_path)
    specs = load_camera_specs(mapping_path, calibration_path, args.wheelbase_m)
    phase_schedule = capture_phase_schedule(
        args.physics_hz,
        args.max_duration_sec,
        float(getattr(args, "stationary_warmup_sec", 0.0)),
        float(getattr(args, "stationary_tail_sec", 0.0)),
    )
    sampling_resolution = float(route.get("sampling_resolution_m", 1.0))
    _, basic_agent_control = basic_agent_control_configuration(
        args, sampling_resolution
    )
    goal_stop_config = configuration_from_args(args)

    partial.mkdir(parents=True)
    shutil.copy2(route_path, partial / "route.json")
    state_records: list[dict[str, Any]] = []
    camera_records: list[dict[str, Any]] = []
    manifest: dict[str, Any] = {
        "schema_version": 1,
        "status": "collecting",
        "created_at": utc_now(),
        "coordinate_contract": {
            "map_frame": "ROS map: x forward/east, y left, z up",
            "ego_frame": "ROS rear-axle base_link: x forward, y left, z up",
            "pose_fields": ["x", "y", "z", "yaw"],
            "dynamics_fields": ["vx", "vy", "ax", "ay", "yaw_rate"],
            "measured_vehicle_fields": [
                "front_left_wheel_steer_angle_carla_deg",
                "front_right_wheel_steer_angle_carla_deg",
                "front_left_wheel_steer_angle_ros_rad",
                "front_right_wheel_steer_angle_ros_rad",
                "steering_tire_angle_rad",
                "speed_limit_carla_kmh",
                "speed_limit_mps",
            ],
            "wheelbase_m": args.wheelbase_m,
        },
        "capture_contract": {
            "physics_hz": args.physics_hz,
            "camera_hz": args.capture_hz,
            "camera_interval_ticks": capture_interval(args.physics_hz, args.capture_hz),
            "camera_order": list(MODEL_CAMERA_ORDER),
            "image_encoding": "jpeg_bgr8",
            "jpeg_quality": args.jpeg_quality,
            "target_speed_kmh": args.target_speed_kmh,
            "maximum_total_duration_sec": args.max_duration_sec,
            "capture_phase_field": "capture_phase",
            "capture_phase_schedule": phase_schedule,
            "seed": args.seed,
            "spawn_z_offset_m": args.spawn_z_offset_m,
            "command_lookahead_m": args.command_lookahead_m,
            "command_exit_lookahead_m": args.command_exit_lookahead_m,
            "goal_tolerance_m": args.goal_tolerance_m,
            "stopped_brake_steering_suppressed": not args.allow_stopped_steering,
            "client_map_loading_allowed": args.allow_map_load,
            "basic_agent_control": basic_agent_control,
            "measured_vehicle_state": {
                "steering_source": (
                    "carla.Vehicle.get_wheel_steer_angle(FL_Wheel,FR_Wheel)"
                ),
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
            },
        },
        "provenance": {
            "source_route": str(route_path),
            "route_sha256": sha256_file(route_path),
            "mapping_file": str(mapping_path),
            "mapping_sha256": sha256_file(mapping_path),
            "calibration_file": str(calibration_path),
            "calibration_sha256": sha256_file(calibration_path),
            "collector_file": str(Path(__file__).resolve()),
            "collector_sha256": sha256_file(Path(__file__).resolve()),
            "spawn_z_offset_m": args.spawn_z_offset_m,
            "basic_agent_control": {
                "configuration_reference": "capture_contract.basic_agent_control",
                "runtime_source_reference": (
                    "runtime.basic_agent_control.implementation_sources"
                ),
                "external_carla_source_patch_required": False,
            },
            "runtime_measurements": {
                "front_wheel_steering": (
                    "carla.Vehicle.get_wheel_steer_angle(FL_Wheel,FR_Wheel)"
                ),
                "speed_limit": "carla.Vehicle.get_speed_limit",
            },
        },
        "cameras": [asdict(spec) for spec in specs],
        "files": {
            "route": "route.json",
            "states": "states.jsonl",
            "camera_frames": "camera_frames.jsonl",
            "images": "images/<CAMERA>/<CARLA_FRAME>.jpg",
        },
    }
    if goal_stop_config:
        # HH_260906 - Record helper bytes and fixed settings without changing legacy manifests.
        helper_path = Path(__file__).with_name("carla_goal_stop_profile.py")
        manifest["capture_contract"]["goal_stop_profile"] = {
            **asdict(goal_stop_config), "control_source": "CARLA BasicAgent expert, not a learned model",
            "tail_policy": "full brake only after measured stop and continuous dwell",
            "bounds": source_motion_bounds(),
        }
        manifest["provenance"]["goal_stop_helper_sha256"] = sha256_file(helper_path)
        if isinstance(goal_stop_config, DevelopmentGoalStopConfig):
            manifest["capture_contract"]["goal_stop_profile"].update({
                "pilot_scope": "Exact Town07 straight development only; 28.8 km/h nominal 30 km/h class.",
                "attempt_ownership": "The owned execution ledger must allow at most two attempts per frozen revision; the collector never schedules a retry.",
                "full_future_xy_admission": "Pending independent post-capture audit; scalar pilot completion is not dataset approval.",
            })
            manifest["result"] = {"training_data_approved": False, "development_only": True}
    if getattr(args, "control_transport", "legacy_async") == "acknowledged_batch":
        manifest["capture_contract"]["control_transport"] = {
            "schema": "carla.acknowledged_control_transport.v1", "mode": "acknowledged_batch",
            "command_api": "Client.apply_batch_sync([ApplyVehicleControl(owned_actor_id, control)], False)",
            "single_actor_single_response": True, "implicit_tick": False, "async_fallback_allowed": False,
            "receipt_journal": "control_receipts.jsonl", "receipt_persisted_before_next_tick": True,
            "kinematics_source": "tick-exact immutable WorldSnapshot.find(owned_actor_id)",
            "control_source": "Vehicle.get_control cached observation bracketed by equal world snapshot frames",
            "comparison_absolute_tolerance": 1.0e-6, "automatic_gear_equality_required": False,
            "on_alignment_failure": "retain mismatched row, acknowledge full-brake abort without extra tick, fail capture",
            "label_rewrite_or_lookback_pass_allowed": False, "physical_actuation_proven": False,
            "native_acceleration_definition_changed": False,
        }
        manifest["files"]["control_receipts"] = "control_receipts.jsonl"
        _write_jsonl(partial / "control_receipts.jsonl", [])
    _write_json(partial / "manifest.json", manifest)
    error: BaseException | None = None
    try:
        collect_episode(
            args, route, specs, partial, state_records, camera_records, manifest
        )
    except BaseException as caught:
        error = caught
    finally:
        _write_jsonl(partial / "states.jsonl", state_records)
        _write_jsonl(partial / "camera_frames.jsonl", camera_records)
        manifest["status"] = "failed" if error else "complete"
        manifest["finished_at"] = utc_now()
        manifest.setdefault("result", {})["state_count"] = len(state_records)
        manifest.setdefault("result", {})["camera_anchor_count"] = len(camera_records)
        if error:
            manifest["error"] = f"{type(error).__name__}: {error}"
        _write_json(partial / "manifest.json", manifest)

    if error is not None:
        raise error
    finalize_output(partial, output)
    return output


def main(argv: Sequence[str] | None = None) -> int:
    try:
        output = run(parse_args(argv))
    except (CollectionError, OSError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print(f"capture={output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
