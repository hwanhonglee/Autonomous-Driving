from __future__ import annotations

import json
import math
from pathlib import Path
from queue import SimpleQueue
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts/e2e"))

from collect_carla_vad_expert import (  # noqa: E402
    _carla_location,
    _server_available_for_cleanup,
    _spawn_runtime_fields,
    CollectionError,
    DEFAULT_CALIBRATION,
    DEFAULT_MAPPING,
    EventRecorder,
    MODEL_CAMERA_ORDER,
    RouteProjector,
    apply_spawn_z_offset,
    base_link_state,
    catalog_goal_status,
    capture_interval,
    exact_camera_bundle,
    finalize_output,
    load_camera_specs,
    parse_args,
    ros_extrinsic_to_carla_vehicle_center,
    shift_transform_local_x,
    suppress_stopped_brake_steering,
    termination_reason,
)


class Image:
    def __init__(self, frame: int, timestamp: float):
        self.frame = frame
        self.timestamp = timestamp


def test_camera_specs_match_fixed_vad_model_and_carla_rig() -> None:
    specs = load_camera_specs(DEFAULT_MAPPING, DEFAULT_CALIBRATION)

    assert tuple(spec.name for spec in specs) == MODEL_CAMERA_ORDER
    assert tuple(spec.model_index for spec in specs) == tuple(range(6))
    assert all(spec.width == 640 and spec.height == 360 for spec in specs)
    assert all(spec.sensor_tick == 0.0 for spec in specs)
    assert all(spec.enable_postprocess_effects is False for spec in specs)
    by_name = {spec.name: spec for spec in specs}
    assert by_name["CAM_BACK"].fov_degrees == 110.0
    assert all(
        spec.fov_degrees == 70.0
        for spec in specs
        if spec.name != "CAM_BACK"
    )
    assert by_name["CAM_FRONT"].carla_extrinsic["x"] == pytest.approx(0.8)
    assert by_name["CAM_FRONT_LEFT"].carla_extrinsic == pytest.approx(
        {"x": 0.27, "y": -0.55, "z": 1.6, "roll": 0.0, "pitch": 0.0, "yaw": -55.0},
        abs=1.0e-4,
    )
    assert by_name["CAM_FRONT_RIGHT"].carla_extrinsic["y"] == pytest.approx(0.55)
    assert by_name["CAM_BACK_LEFT"].carla_extrinsic["yaw"] == pytest.approx(-110.0, abs=1e-4)
    assert by_name["CAM_BACK_RIGHT"].carla_extrinsic["yaw"] == pytest.approx(110.0, abs=1e-4)
    assert by_name["CAM_FRONT"].intrinsic["fx"] == pytest.approx(457.0073621574767)
    assert by_name["CAM_BACK"].intrinsic["fx"] == pytest.approx(224.06641222710715)


def test_ros_extrinsic_and_spawn_origin_conversions() -> None:
    carla_extrinsic = ros_extrinsic_to_carla_vehicle_center(
        {"x": 1.695, "y": 0.55, "z": 1.6, "roll": 0.0, "pitch": 0.0, "yaw": math.radians(55)}
    )
    assert carla_extrinsic == pytest.approx(
        {"x": 0.27, "y": -0.55, "z": 1.6, "roll": 0.0, "pitch": 0.0, "yaw": -55.0}
    )

    base = {"x": 10.0, "y": 20.0, "z": 0.3, "roll": 0.0, "pitch": 0.0, "yaw": 90.0}
    center = shift_transform_local_x(base, 1.425)
    recovered = shift_transform_local_x(center, -1.425)
    assert center["x"] == pytest.approx(10.0)
    assert center["y"] == pytest.approx(21.425)
    assert recovered == pytest.approx(base)


def test_spawn_z_offset_only_changes_actor_center_spawn_height() -> None:
    route_base = {
        "x": 10.0,
        "y": 20.0,
        "z": 3.764,
        "roll": 0.0,
        "pitch": 10.95,
        "yaw": 90.0,
    }
    map_center = shift_transform_local_x(route_base, 1.425)
    actor_spawn = apply_spawn_z_offset(map_center, 1.4)

    assert actor_spawn["x"] == pytest.approx(map_center["x"])
    assert actor_spawn["y"] == pytest.approx(map_center["y"])
    assert actor_spawn["z"] == pytest.approx(map_center["z"] + 1.4)
    assert actor_spawn["pitch"] == pytest.approx(map_center["pitch"])
    assert route_base["z"] == pytest.approx(3.764)

    runtime = _spawn_runtime_fields(map_center, actor_spawn, 1.4)
    assert runtime["spawn_z_offset_m"] == pytest.approx(1.4)
    assert runtime["actor_center_start_carla"] == pytest.approx(map_center)
    assert runtime["actor_center_spawn_carla"] == pytest.approx(actor_spawn)


def test_spawn_z_offset_cli_defaults_and_validation(tmp_path: Path) -> None:
    positional = [str(tmp_path / "episode"), str(tmp_path / "route.json")]

    assert parse_args(positional).capture_hz == pytest.approx(10.0)
    assert parse_args(positional).spawn_z_offset_m == pytest.approx(0.0)
    explicit = parse_args(positional + ["--spawn-z-offset-m", "1.4"])
    assert explicit.spawn_z_offset_m == pytest.approx(1.4)
    for invalid in ("-0.01", "nan", "inf"):
        with pytest.raises(SystemExit):
            parse_args(positional + ["--spawn-z-offset-m", invalid])


def test_goal_tolerance_cli_defaults_and_requires_positive_finite_value(
    tmp_path: Path,
) -> None:
    positional = [str(tmp_path / "episode"), str(tmp_path / "route.json")]

    assert parse_args(positional).goal_tolerance_m == pytest.approx(2.5)
    explicit = parse_args(positional + ["--goal-tolerance-m", "1.25"])
    assert explicit.goal_tolerance_m == pytest.approx(1.25)
    for invalid in ("0", "-0.01", "nan", "inf"):
        with pytest.raises(SystemExit):
            parse_args(positional + ["--goal-tolerance-m", invalid])


def test_base_link_state_uses_ros_pose_and_ego_dynamics() -> None:
    state = base_link_state(
        {"x": 10.0, "y": 21.425, "z": 0.3, "roll": 0.0, "pitch": 0.0, "yaw": 90.0},
        world_velocity=(0.0, 4.0, 0.0),
        world_acceleration=(-2.0, 0.0, 0.0),
        angular_velocity_z_degrees=10.0,
    )
    assert state["x"] == pytest.approx(10.0)
    assert state["y"] == pytest.approx(-20.0)
    assert state["yaw"] == pytest.approx(-math.pi / 2.0)
    assert state["vx"] == pytest.approx(4.0)
    assert state["vy"] == pytest.approx(1.425 * math.radians(10.0))
    assert state["ax"] == pytest.approx(0.0, abs=1e-12)
    assert state["ay"] == pytest.approx(-2.0)
    assert state["yaw_rate"] == pytest.approx(-math.radians(10.0))


def test_catalog_goal_requires_route_progress_and_planar_distance() -> None:
    progress_only = catalog_goal_status(99.0, 100.0, 90.0, 0.0, 100.0, 0.0, 2.0)
    distance_only = catalog_goal_status(90.0, 100.0, 99.0, 1.0, 100.0, 0.0, 2.0)
    both = catalog_goal_status(98.0, 100.0, 99.0, 1.0, 100.0, 0.0, 2.0)

    assert progress_only.reached is False
    assert progress_only.remaining_route_m == pytest.approx(1.0)
    assert progress_only.planar_distance_m == pytest.approx(10.0)
    assert distance_only.reached is False
    assert distance_only.remaining_route_m == pytest.approx(10.0)
    assert distance_only.planar_distance_m == pytest.approx(math.sqrt(2.0))
    assert both.reached is True
    assert both.remaining_route_m == pytest.approx(2.0)
    assert both.planar_distance_m == pytest.approx(math.sqrt(2.0))


def test_catalog_goal_rejects_invalid_inputs() -> None:
    with pytest.raises(CollectionError, match="tolerance must be positive"):
        catalog_goal_status(10.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    with pytest.raises(CollectionError, match="inputs must be finite"):
        catalog_goal_status(math.nan, 10.0, 0.0, 0.0, 0.0, 0.0, 2.0)
    with pytest.raises(CollectionError, match="progress and length must be non-negative"):
        catalog_goal_status(-1.0, 10.0, 0.0, 0.0, 0.0, 0.0, 2.0)


def test_termination_reason_prioritizes_catalog_goal_and_rejects_early_agent_done() -> None:
    assert (
        termination_reason(basic_agent_done=False, catalog_goal_reached=False) is None
    )
    assert termination_reason(
        basic_agent_done=True, catalog_goal_reached=False
    ) == "basic_agent_done_before_catalog_goal"
    assert termination_reason(
        basic_agent_done=False, catalog_goal_reached=True
    ) == "catalog_goal_tolerance"
    assert termination_reason(
        basic_agent_done=True, catalog_goal_reached=True
    ) == "catalog_goal_tolerance"


def _route_payload() -> dict:
    commands = (3, 3, 1, 1, 3)
    return {
        "route": [
            {
                "x": float(index * 5),
                "y": 0.0,
                "distance_m": float(index * 5),
                "vad_command": command,
            }
            for index, command in enumerate(commands)
        ]
    }


def test_route_projection_and_command_lookahead_match_runtime_policy() -> None:
    route = RouteProjector(_route_payload())
    projection = route.project(4.0, 1.0, 0.0)
    assert projection.progress_m == pytest.approx(4.0)
    assert projection.cross_track_error_m == pytest.approx(1.0)
    assert route.command_at(4.0, lookahead_m=2.0, exit_lookahead_m=2.5) == 3
    assert route.command_at(9.0, lookahead_m=2.0, exit_lookahead_m=2.5) == 1
    assert route.command_at(14.0, lookahead_m=2.0, exit_lookahead_m=2.5) == 1
    assert route.command_at(19.0, lookahead_m=2.0, exit_lookahead_m=2.5) == 3


def test_exact_camera_bundle_discards_stale_frames_and_keeps_model_order() -> None:
    queues = {name: SimpleQueue() for name in MODEL_CAMERA_ORDER}
    for name in MODEL_CAMERA_ORDER:
        queues[name].put(Image(9, 0.45))
        queues[name].put(Image(10, 0.50))

    bundle = exact_camera_bundle(queues, frame=10, timeout_sec=0.1)

    assert tuple(bundle) == MODEL_CAMERA_ORDER
    assert all(image.frame == 10 for image in bundle.values())


def test_exact_camera_bundle_rejects_missing_or_phase_split_frames() -> None:
    queues = {name: SimpleQueue() for name in MODEL_CAMERA_ORDER}
    for name in MODEL_CAMERA_ORDER:
        queues[name].put(Image(11 if name == "CAM_BACK" else 10, 0.5))
    with pytest.raises(CollectionError, match="skipped frame"):
        exact_camera_bundle(queues, frame=10, timeout_sec=0.1)

    queues = {name: SimpleQueue() for name in MODEL_CAMERA_ORDER}
    for index, name in enumerate(MODEL_CAMERA_ORDER):
        queues[name].put(Image(10, 0.5 + index * 1.0e-5))
    with pytest.raises(CollectionError, match="timestamp-synchronous"):
        exact_camera_bundle(queues, frame=10, timeout_sec=0.1)


@pytest.mark.parametrize(
    ("physics_hz", "camera_hz", "expected"),
    ((20.0, 5.0, 4), (20.0, 10.0, 2), (10.0, 10.0, 1)),
)
def test_capture_interval_requires_an_integer_tick_ratio(
    physics_hz: float, camera_hz: float, expected: int
) -> None:
    assert capture_interval(physics_hz, camera_hz) == expected


def test_capture_interval_rejects_non_integral_or_invalid_rates() -> None:
    with pytest.raises(CollectionError, match="integer multiple"):
        capture_interval(20.0, 6.0)
    with pytest.raises(CollectionError, match="positive"):
        capture_interval(0.0, 5.0)


def test_default_route_camera_files_exist_and_are_json_serializable() -> None:
    specs = load_camera_specs(DEFAULT_MAPPING, DEFAULT_CALIBRATION)
    serialized = json.dumps([spec.__dict__ for spec in specs])
    assert "CAM_FRONT" in serialized


def test_finalize_output_atomically_promotes_only_a_staging_directory(
    tmp_path: Path,
) -> None:
    output = tmp_path / "episode"
    partial = tmp_path / "episode.partial"
    partial.mkdir()
    (partial / "manifest.json").write_text("{}\n", encoding="utf-8")

    finalize_output(partial, output)

    assert output.is_dir()
    assert not partial.exists()
    assert (output / "manifest.json").is_file()
    replacement = tmp_path / "replacement.partial"
    replacement.mkdir()
    with pytest.raises(CollectionError, match="output already exists"):
        finalize_output(replacement, output)


def test_cleanup_server_probe_bounds_disconnected_cleanup() -> None:
    class DisconnectedClient:
        def __init__(self):
            self.timeout = None
            self.calls = 0

        def set_timeout(self, value: float) -> None:
            self.timeout = value

        def get_world(self):
            self.calls += 1
            raise RuntimeError("server disappeared")

    client = DisconnectedClient()
    errors: list[str] = []

    assert _server_available_for_cleanup(client, 2.0, errors) is False
    assert client.timeout == pytest.approx(2.0)
    assert client.calls == 1
    assert errors == ["cleanup server unavailable: server disappeared"]


def test_event_recorder_groups_collision_and_lane_events_by_carla_frame() -> None:
    vector = type("Vector", (), {"x": 3.0, "y": 4.0, "z": 0.0})()
    actor = type(
        "Actor",
        (),
        {"id": 7, "type_id": "vehicle.test", "attributes": {"role_name": "npc"}},
    )()
    collision = type(
        "Collision",
        (),
        {
            "frame": 42,
            "timestamp": 2.1,
            "other_actor": actor,
            "normal_impulse": vector,
        },
    )()
    marking = type(
        "Marking", (), {"type": "Solid", "color": "White", "lane_change": "None"}
    )()
    invasion = type(
        "Invasion", (), {"frame": 42, "timestamp": 2.1, "crossed_lane_markings": [marking]}
    )()
    recorder = EventRecorder()

    recorder.on_collision(collision)
    recorder.on_lane_invasion(invasion)
    collisions, invasions = recorder.snapshot()

    assert collisions[42][0]["intensity"] == pytest.approx(5.0)
    assert collisions[42][0]["other_actor_role"] == "npc"
    assert invasions[42][0]["crossed_lane_markings"][0]["type"] == "Solid"


def test_red_light_guard_prevents_stopped_lateral_pid_windup() -> None:
    control = type(
        "Control", (), {"steer": -0.8, "brake": 0.5, "hand_brake": False}
    )()

    assert suppress_stopped_brake_steering(control, speed_mps=0.0, enabled=True) is True
    assert control.steer == 0.0

    moving = type(
        "Control", (), {"steer": -0.4, "brake": 0.5, "hand_brake": False}
    )()
    assert suppress_stopped_brake_steering(moving, speed_mps=1.0, enabled=True) is False
    assert moving.steer == -0.4


def test_goal_location_has_independent_lifetime() -> None:
    class Location:
        def __init__(self, **values):
            self.values = values

    fake_carla = type("Carla", (), {"Location": Location})

    location = _carla_location(fake_carla, {"x": 1.0, "y": 2.0, "z": 3.0})

    assert location.values == {"x": 1.0, "y": 2.0, "z": 3.0}
