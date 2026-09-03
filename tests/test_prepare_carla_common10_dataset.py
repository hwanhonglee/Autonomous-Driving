from __future__ import annotations

import hashlib
import io
import importlib.util
import json
import math
import os
from pathlib import Path
import sys

from PIL import Image
import pytest

from portable_e2e import contract as common10


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/e2e/prepare_carla_common10_dataset.py"


def load_module():
    spec = importlib.util.spec_from_file_location("prepare_carla_common10_dataset", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def _write_jsonl(path: Path, values: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "".join(json.dumps(value, separators=(",", ":")) + "\n" for value in values),
        encoding="utf-8",
    )


def _read_jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]


def _native_camera(name: str, index: int, yaw: float) -> dict:
    return {
        "name": name,
        "model_index": index,
        "width": 640,
        "height": 360,
        "sensor_tick": 0.0,
        "enable_postprocess_effects": False,
        "frame_id": f"{name}/camera_optical_link",
        "ros_extrinsic": {
            "x": 1.0 - index * 0.1,
            "y": 0.0,
            "z": 1.6,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": yaw,
        },
        "intrinsic": {"fx": 457.0, "fy": 457.0, "cx": 320.0, "cy": 180.0},
    }


def write_native_episode(root: Path, *, duration_s: float = 37.0, steering: bool = True) -> Path:
    root.mkdir(parents=True)
    state_count = int(round(duration_s * 20.0)) + 1
    camera_count = int(math.floor(duration_s * 10.0)) + 1
    role_yaws = (0.0, math.pi, math.pi / 4, 3 * math.pi / 4, -math.pi / 4, -3 * math.pi / 4)
    cameras = [
        _native_camera(name, index, role_yaws[index])
        for index, name in enumerate(common10.CAMERA_ORDER)
    ]
    route = {
        "schema_version": 1,
        "coordinate_reference": "base_link",
        "town": "Town07",
        "weather": "ClearNoon",
        "scenario": "straight",
        "route_length_m": 56.0,
        "goal_ros_pose": {"x": 56.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
        "route": [
            {"x": float(index), "y": 0.0, "distance_m": float(index), "vad_command": 3}
            for index in range(57)
        ],
        "coordinate_alignment": {
            "source_route": "/catalog/town07_straight_s0007_p00.json",
            "source_route_sha256": "a" * 64,
        },
    }
    _write_json(root / "route.json", route)

    base_time_s = 1.0
    states = []
    for index in range(state_count):
        elapsed = index / 20.0
        if elapsed < 3.5:
            phase = "stationary_warmup"
            position = 0.0
            speed = 0.0
        elif elapsed <= 30.5:
            phase = "driving"
            position = 2.0 * (elapsed - 3.5)
            speed = 2.0
        else:
            phase = "stationary_tail"
            position = 54.0
            speed = 0.0
        state = {
            "frame": 10_000 + index,
            "timestamp": base_time_s + elapsed,
            "x": position,
            "y": 0.0,
            "z": 0.5,
            "yaw": 0.0,
            "vx": speed,
            "vy": 0.0,
            "ax": 0.0,
            "ay": 0.0,
            "yaw_rate": 0.0,
            "front_left_wheel_steer_angle_carla_deg": 0.0,
            "front_right_wheel_steer_angle_carla_deg": 0.0,
            "front_left_wheel_steer_angle_ros_rad": -0.0,
            "front_right_wheel_steer_angle_ros_rad": -0.0,
            "speed_limit_carla_kmh": 30.0,
            "speed_limit_mps": 30.0 / 3.6,
            "capture_phase": phase,
            "command": 3,
            "route_progress_m": position,
            "route_cte_m": 0.0,
            "collision": [],
            "lane_invasion": [],
        }
        if steering:
            state["steering_tire_angle_rad"] = 0.0
        states.append(state)
    _write_jsonl(root / "states.jsonl", states)

    image_payloads = {}
    for camera_index, name in enumerate(common10.CAMERA_ORDER):
        stream = io.BytesIO()
        Image.new(
            "RGB",
            (640, 360),
            (20 + camera_index * 20, 60, 120),
        ).save(stream, format="JPEG")
        image_payloads[name] = stream.getvalue()
    frames = []
    for index in range(camera_count):
        elapsed = index / 10.0
        stamp = base_time_s + elapsed
        if elapsed < 3.5:
            phase = "stationary_warmup"
        elif elapsed <= 30.5:
            phase = "driving"
        else:
            phase = "stationary_tail"
        frame = 10_000 + index * 2
        images = {}
        for name in common10.CAMERA_ORDER:
            relative = Path("images") / name / f"{frame:08d}.jpg"
            image = root / relative
            image.parent.mkdir(parents=True, exist_ok=True)
            image.write_bytes(image_payloads[name])
            images[name] = relative.as_posix()
        frames.append(
            {
                "frame": frame,
                "timestamp": stamp,
                "camera_order": list(common10.CAMERA_ORDER),
                "images": images,
                "source_timestamps": {
                    name: stamp for name in common10.CAMERA_ORDER
                },
                "timestamp_span_sec": 0.0,
                "jpeg_quality": 95,
                "capture_phase": phase,
            }
        )
    _write_jsonl(root / "camera_frames.jsonl", frames)

    state_phase_counts = {
        phase: sum(state["capture_phase"] == phase for state in states)
        for phase in ("stationary_warmup", "driving", "stationary_tail")
    }
    camera_phase_counts = {
        phase: sum(frame["capture_phase"] == phase for frame in frames)
        for phase in ("stationary_warmup", "driving", "stationary_tail")
    }
    phase_schedule = {
        "order": ["stationary_warmup", "driving", "stationary_tail"],
        "stationary_warmup": {
            "scheduled_ticks": state_phase_counts["stationary_warmup"],
        },
        "driving": {"maximum_ticks": state_phase_counts["driving"]},
        "stationary_tail": {
            "scheduled_ticks": state_phase_counts["stationary_tail"],
        },
    }
    phase_ledger = {
        "order": ["stationary_warmup", "driving", "stationary_tail"],
        **{
            phase: {
                "state_count": state_phase_counts[phase],
                "camera_anchor_count": camera_phase_counts[phase],
            }
            for phase in ("stationary_warmup", "driving", "stationary_tail")
        },
        "observed_total_ticks": state_count,
        "observed_total_elapsed_sim_sec": state_count / 20.0,
    }
    manifest = {
        "schema_version": 1,
        "status": "complete",
        "created_at": "2026-09-04T00:00:00+00:00",
        "coordinate_contract": {
            "measured_vehicle_fields": [
                "front_left_wheel_steer_angle_carla_deg",
                "front_right_wheel_steer_angle_carla_deg",
                "front_left_wheel_steer_angle_ros_rad",
                "front_right_wheel_steer_angle_ros_rad",
                "steering_tire_angle_rad",
                "speed_limit_carla_kmh",
                "speed_limit_mps",
            ]
        },
        "capture_contract": {
            "physics_hz": 20.0,
            "camera_hz": 10.0,
            "camera_interval_ticks": 2,
            "camera_order": list(common10.CAMERA_ORDER),
            "image_encoding": "jpeg_bgr8",
            "jpeg_quality": 95,
            "capture_phase_field": "capture_phase",
            "capture_phase_schedule": phase_schedule,
            "target_speed_kmh": 7.2,
            "goal_tolerance_m": 2.5,
            "seed": 7,
            "measured_vehicle_state": {
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
            },
        },
        "provenance": {
            "source_route": "/catalog/town07_straight_s0007_p00.json",
            "route_sha256": hashlib.sha256((root / "route.json").read_bytes()).hexdigest(),
        },
        "runtime": {
            "town": "Town07",
            "weather": "ClearNoon",
            "server_version": "0.9.15",
            "vehicle_type": "vehicle.toyota.prius",
            "capture_world_settings": {
                "synchronous_mode": True,
                "fixed_delta_seconds": 0.05,
            },
        },
        "cameras": cameras,
        "result": {
            "goal_reached": True,
            "collision_event_count": 0,
            "lane_invasion_event_count": 0,
            "state_count": state_count,
            "camera_anchor_count": camera_count,
            "capture_phases": phase_ledger,
            "route_length_m": 56.0,
            "final_route_progress_m": 54.0,
            "post_capture_route_progress_m": 54.0,
            "final_remaining_route_m": 2.0,
            "final_catalog_goal_distance_m": 2.0,
            "goal_tolerance_m": 2.5,
        },
        "cleanup": {"completed": True, "server_available": True, "errors": []},
    }
    _write_json(root / "manifest.json", manifest)
    return root


def _args(module, source: Path, output: Path):
    return module._parse_args(
        [
            "--episode",
            f"train={source}",
            "--output",
            str(output),
            "--dataset-id",
            "carla_common10_fixture",
            "--license-id",
            "unit-test-generated",
            "--image-mode",
            "hardlink",
            "--git-commit",
            "0123456789abcdef0123456789abcdef01234567",
        ]
    )


def test_optical_transform_uses_base_x_forward_y_left_axes() -> None:
    module = load_module()

    transform = module._optical_transform(
        {"x": 1.0, "y": 2.0, "z": 3.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    )

    assert transform == pytest.approx(
        [
            0.0, 0.0, 1.0, 1.0,
            -1.0, 0.0, 0.0, 2.0,
            0.0, -1.0, 0.0, 3.0,
            0.0, 0.0, 0.0, 1.0,
        ]
    )


def test_missing_measured_steering_fails_without_output(tmp_path: Path) -> None:
    module = load_module()
    source = write_native_episode(tmp_path / "native", steering=False)
    output = tmp_path / "prepared"

    with pytest.raises(module.AdapterError, match="steering_tire_angle_rad"):
        module.prepare_dataset(_args(module, source, output))

    assert not output.exists()
    assert not output.with_name(output.name + ".partial").exists()


def test_complete_native_episode_converts_and_passes_planning(tmp_path: Path) -> None:
    module = load_module()
    source = write_native_episode(tmp_path / "native")
    source_hashes = {
        path.relative_to(source).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in source.rglob("*")
        if path.is_file()
    }
    output = tmp_path / "prepared"

    result, report = module.prepare_dataset(_args(module, source, output))

    assert result == output.resolve()
    assert report["status"] == "PASS"
    assert report["qualification"]["common_10hz_planning"] == "PASS"
    assert report["episode_count"] == 1
    assert report["sample_count"] == 306
    assert report["episodes"][0]["duration_s"] == pytest.approx(30.5)
    assert report["episodes"][0]["effective_rate_hz"] == pytest.approx(10.0)
    assert report["episodes"][0]["valid_future_point_percent"] == 100.0
    assert report["episodes"][0]["motion_distance_m"] == pytest.approx(54.0)
    assert (output / "validation_report.json").is_file()
    episode_path = next((output / "episodes").glob("*/episode.json"))
    prepared_episode = json.loads(episode_path.read_text(encoding="utf-8"))
    assert prepared_episode["route_id"] == "town07_straight_s0007_p00"
    source_manifest = json.loads(
        episode_path.with_name("source_manifest.json").read_text(encoding="utf-8")
    )
    assert len({frame["source_uri"] for frame in source_manifest["camera_frames"]}) == (
        6 * 371
    )
    assert sum(
        frame["prepared_payload_sha256"] is None
        for frame in source_manifest["camera_frames"]
    ) == 6 * 65
    assert source_hashes == {
        path.relative_to(source).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in source.rglob("*")
        if path.is_file()
    }
    with pytest.raises(module.AdapterError, match="must not already exist"):
        module.prepare_dataset(_args(module, source, output))


def test_short_complete_horizon_span_fails_and_removes_partial(tmp_path: Path) -> None:
    module = load_module()
    source = write_native_episode(tmp_path / "native", duration_s=35.0)
    output = tmp_path / "prepared"

    with pytest.raises(module.AdapterError, match="anchor span .* below Common10 minimum"):
        module.prepare_dataset(_args(module, source, output))

    assert not output.exists()
    assert not output.with_name(output.name + ".partial").exists()


def test_duplicate_json_key_fails_closed_before_output(tmp_path: Path) -> None:
    module = load_module()
    source = write_native_episode(tmp_path / "native")
    manifest_path = source / "manifest.json"
    manifest_path.write_text(
        manifest_path.read_text(encoding="utf-8").replace(
            '  "status": "complete",',
            '  "status": "complete",\n  "status": "complete",',
            1,
        ),
        encoding="utf-8",
    )
    output = tmp_path / "prepared"

    with pytest.raises(module.AdapterError, match="duplicate JSON key 'status'"):
        module.prepare_dataset(_args(module, source, output))

    assert not output.exists()
    assert not output.with_name(output.name + ".partial").exists()


def test_output_inside_native_episode_is_rejected_without_mutating_source(
    tmp_path: Path,
) -> None:
    module = load_module()
    source = write_native_episode(tmp_path / "native")
    before = {
        path.relative_to(source).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in source.rglob("*")
        if path.is_file()
    }
    output = source / "prepared"

    with pytest.raises(module.AdapterError, match="must not be inside a native episode"):
        module.prepare_dataset(_args(module, source, output))

    assert not output.exists()
    assert not output.with_name(output.name + ".partial").exists()
    assert before == {
        path.relative_to(source).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in source.rglob("*")
        if path.is_file()
    }


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("steering_tire_angle_rad", 0.25, "does not match recomputed value"),
        ("speed_limit_mps", 30.0, "does not match recomputed value"),
    ],
)
def test_derived_vehicle_measurements_are_recomputed(
    tmp_path: Path,
    field: str,
    value: float,
    message: str,
) -> None:
    module = load_module()
    source = write_native_episode(tmp_path / "native")
    states_path = source / "states.jsonl"
    states = _read_jsonl(states_path)
    states[0][field] = value
    _write_jsonl(states_path, states)
    output = tmp_path / "prepared"

    with pytest.raises(module.AdapterError, match=message):
        module.prepare_dataset(_args(module, source, output))

    assert not output.exists()


def test_reused_camera_file_identity_is_rejected(tmp_path: Path) -> None:
    module = load_module()
    source = write_native_episode(tmp_path / "native")
    frames = _read_jsonl(source / "camera_frames.jsonl")
    first = source / frames[0]["images"]["CAM_FRONT"]
    second = source / frames[1]["images"]["CAM_FRONT"]
    second.unlink()
    os.link(first, second)
    output = tmp_path / "prepared"

    with pytest.raises(module.AdapterError, match="source file inode is reused"):
        module.prepare_dataset(_args(module, source, output))

    assert not output.exists()


def test_sparse_native_state_timeline_is_rejected(tmp_path: Path) -> None:
    module = load_module()
    source = write_native_episode(tmp_path / "native")
    states_path = source / "states.jsonl"
    states = _read_jsonl(states_path)
    states.pop(1)
    _write_jsonl(states_path, states)
    manifest_path = source / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["result"]["state_count"] -= 1
    manifest["result"]["capture_phases"]["stationary_warmup"]["state_count"] -= 1
    manifest["result"]["capture_phases"]["observed_total_ticks"] -= 1
    manifest["result"]["capture_phases"]["observed_total_elapsed_sim_sec"] -= 0.05
    _write_json(manifest_path, manifest)
    output = tmp_path / "prepared"

    with pytest.raises(module.AdapterError, match="state frame counters must be contiguous"):
        module.prepare_dataset(_args(module, source, output))

    assert not output.exists()


def test_camera_frame_must_bind_same_physics_state(tmp_path: Path) -> None:
    module = load_module()
    source = write_native_episode(tmp_path / "native")
    frames_path = source / "camera_frames.jsonl"
    frames = _read_jsonl(frames_path)
    frames[0]["frame"] += 1
    _write_jsonl(frames_path, frames)
    output = tmp_path / "prepared"

    with pytest.raises(module.AdapterError, match="timestamp differs from same-frame ego state"):
        module.prepare_dataset(_args(module, source, output))

    assert not output.exists()


def test_declared_goal_distance_is_recomputed(tmp_path: Path) -> None:
    module = load_module()
    source = write_native_episode(tmp_path / "native")
    manifest_path = source / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["result"]["final_catalog_goal_distance_m"] = 0.0
    _write_json(manifest_path, manifest)
    output = tmp_path / "prepared"

    with pytest.raises(module.AdapterError, match="does not match recomputed value"):
        module.prepare_dataset(_args(module, source, output))

    assert not output.exists()
