from __future__ import annotations

import importlib.util
import math
import sys
from collections import deque
from pathlib import Path

import numpy as np
import pytest
import yaml


pytest.importorskip("rosbag2_py")

MODULE_PATH = Path(__file__).parents[1] / "scripts" / "e2e" / "mpc_replay_sweep.py"
SPEC = importlib.util.spec_from_file_location("mpc_replay_sweep", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
replay = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = replay
SPEC.loader.exec_module(replay)


def _trajectory(yaws: list[float], step: float = 1.0):
    trajectory = replay.Trajectory()
    for index, yaw in enumerate(yaws):
        point = trajectory.points.add() if hasattr(trajectory.points, "add") else None
        if point is None:
            from autoware_planning_msgs.msg import TrajectoryPoint

            point = TrajectoryPoint()
            trajectory.points.append(point)
        point.pose.position.x = index * step
        point.pose.position.y = 0.0
        point.pose.orientation.z = math.sin(0.5 * yaw)
        point.pose.orientation.w = math.cos(0.5 * yaw)
        point.longitudinal_velocity_mps = 2.5
    return trajectory


def test_parse_grid_sorts_deduplicates_and_rejects_invalid_values() -> None:
    assert replay.parse_grid("0.24, 0.09,0.24", "grid") == [0.09, 0.24]
    with pytest.raises(replay.ReplayError):
        replay.parse_grid("0.1,-0.2", "grid")


def test_auto_snapshot_selection_prefers_complete_heading_change() -> None:
    straight = _trajectory([0.0] * 12)
    complete_corner = _trajectory(np.linspace(math.pi / 2, 0.0, 12).tolist())
    noisy_kink = _trajectory([0.0, 0.8, 0.0] + [0.0] * 9)

    selected = replay.choose_trajectory_index(
        [(1, straight), (2, complete_corner), (3, noisy_kink)], None
    )

    assert selected == 1


def test_point_to_polyline_returns_signed_error_and_progress() -> None:
    line = np.asarray([[0.0, 0.0], [10.0, 0.0], [20.0, 0.0]])

    left, left_progress, yaw, segment = replay.point_to_polyline((7.0, 2.0), line)
    right, right_progress, _, _ = replay.point_to_polyline((12.0, -3.0), line)

    assert left == pytest.approx(2.0)
    assert left_progress == pytest.approx(7.0)
    assert yaw == pytest.approx(0.0)
    assert segment == 0
    assert right == pytest.approx(-3.0)
    assert right_progress == pytest.approx(12.0)


def test_actuator_delay_is_independent_from_controller_parameters() -> None:
    state = replay.PlantState(x=0.0, y=0.0, yaw=0.0, speed=0.0, steer=0.0)
    command_buffer = deque([0.0, 0.0])
    measured = []
    for _ in range(3):
        state, _ = replay.advance_plant(
            state,
            command=0.3,
            command_buffer=command_buffer,
            dt=0.1,
            wheel_base=2.79,
            steer_tau=0.2,
            max_steer=0.7,
        )
        measured.append(state.steer)

    assert measured[:2] == pytest.approx([0.0, 0.0])
    assert measured[2] > 0.0


def test_trace_score_penalizes_tracking_error_and_missing_commands() -> None:
    base = [
        {
            "cte_m": value,
            "heading_error_rad": 0.1,
            "steer_command_rad": 0.2,
            "steer_measured_rad": 0.15,
            "progress_m": float(index),
        }
        for index, value in enumerate([0.1, -0.2, 0.3])
    ]
    worse = [dict(sample, cte_m=sample["cte_m"] * 2.0) for sample in base]

    base_metrics = replay.summarize_trace(base, 0)
    worse_metrics = replay.summarize_trace(worse, 0)
    missing_metrics = replay.summarize_trace(base, 1)

    assert worse_metrics["score"] > base_metrics["score"]
    assert missing_metrics["score"] > worse_metrics["score"]


def test_aggregate_results_ranks_by_mean_score() -> None:
    def result(delay: float, tau: float, score: float):
        return {
            "requested_input_delay_sec": delay,
            "effective_input_delay_sec": delay,
            "controller_steer_tau_sec": tau,
            "metrics": {"score": score, "cte_rms_m": score / 2.0},
            "trace": [],
        }

    aggregate = replay.aggregate_results(
        [result(0.24, 0.27, 2.0), result(0.10, 0.20, 1.0), result(0.10, 0.20, 1.2)]
    )

    assert aggregate[0]["requested_input_delay_sec"] == pytest.approx(0.10)
    assert aggregate[0]["metrics_mean"]["score"] == pytest.approx(1.1)
    assert aggregate[0]["metrics_std"]["score"] == pytest.approx(0.1)


def test_zero_delay_overlay_remains_a_ros_double(tmp_path: Path) -> None:
    overlay = tmp_path / "override.yaml"

    replay.write_overlay(overlay, input_delay=0.0, steer_tau=0.03, ctrl_period=0.03)

    assert "input_delay: 0.0" in overlay.read_text(encoding="utf-8")


def test_overlay_serializes_optional_smoothing_parameters(tmp_path: Path) -> None:
    overlay = tmp_path / "override.yaml"

    replay.write_overlay(
        overlay,
        input_delay=0.12,
        steer_tau=0.15,
        ctrl_period=0.03,
        overrides={
            "mpc_weight_heading_error_squared_vel": 0.2,
            "mpc_weight_steer_acc": 4.0e-6,
            "steering_lpf_cutoff_hz": 2.0,
            "steer_rate_lim_dps_list_by_curvature": [40.0] * 3,
            "max_steer_angle": 0.62,
        },
    )

    parameters = yaml.safe_load(overlay.read_text(encoding="utf-8"))["/**"][
        "ros__parameters"
    ]
    assert parameters["mpc_weight_steer_acc"] == pytest.approx(4.0e-6)
    assert parameters["mpc_weight_heading_error_squared_vel"] == pytest.approx(0.2)
    assert parameters["steering_lpf_cutoff_hz"] == pytest.approx(2.0)
    assert parameters["steer_rate_lim_dps_list_by_curvature"] == [40.0] * 3
    assert parameters["max_steer_angle"] == pytest.approx(0.62)


def test_parse_args_rejects_invalid_smoothing_values(tmp_path: Path) -> None:
    with pytest.raises(SystemExit):
        replay.parse_args(
            [str(tmp_path / "bag"), str(tmp_path / "out"), "--steer-rate-limit-dps", "0"]
        )


def test_controller_rate_limit_override_does_not_emit_yaml_aliases() -> None:
    args = replay.parse_args(
        ["bag", "out", "--steer-rate-limit-dps", "40", "--input-delays", "0.12"]
    )

    serialized = yaml.safe_dump(replay.controller_overrides(args))

    assert "&id" not in serialized
    assert "*id" not in serialized


def test_parse_args_accepts_independent_simulation_step() -> None:
    args = replay.parse_args(
        ["bag", "out", "--ctrl-period", "0.03", "--simulation-step", "0.05"]
    )

    assert args.ctrl_period == pytest.approx(0.03)
    assert args.simulation_step == pytest.approx(0.05)


def test_parse_args_accepts_plant_side_steering_cap() -> None:
    args = replay.parse_args(["bag", "out", "--plant-command-steer-cap-rad", "0.65"])

    assert args.plant_command_steer_cap_rad == pytest.approx(0.65)
