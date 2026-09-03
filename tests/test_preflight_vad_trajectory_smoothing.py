from __future__ import annotations

import copy
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest


pytest.importorskip("rosbag2_py")
pytest.importorskip("autoware_internal_planning_msgs")
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint  # noqa: E402


ROOT = Path(__file__).parents[1]
SCRIPT = ROOT / "scripts/analysis/preflight_vad_trajectory_smoothing.py"
SPEC = importlib.util.spec_from_file_location(
    "preflight_vad_trajectory_smoothing", SCRIPT
)
assert SPEC is not None and SPEC.loader is not None
preflight = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = preflight
SPEC.loader.exec_module(preflight)


def _simple_point(x: float, y: float = 0.0, yaw: float = 0.0):
    return SimpleNamespace(
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y, z=0.0),
            orientation=SimpleNamespace(
                x=0.0,
                y=0.0,
                z=math.sin(yaw * 0.5),
                w=math.cos(yaw * 0.5),
            ),
        ),
        longitudinal_velocity_mps=16.0,
        lateral_velocity_mps=0.0,
        acceleration_mps2=0.0,
        heading_rate_rps=0.0,
        front_wheel_angle_rad=0.0,
        rear_wheel_angle_rad=0.0,
        time_from_start=0,
    )


def _simple_trajectory(*points):
    return SimpleNamespace(points=list(points))


def _record(topic: str, storage_ns: int, header_ns: int, sequence: int):
    return preflight.BagRecord(
        topic=topic,
        storage_ns=storage_ns,
        header_ns=header_ns,
        message=SimpleNamespace(points=[]),
        sequence=sequence,
    )


def _variant_summary(*, curvature: float = 0.003, rejection_count: int = 0):
    return {
        "snapshot_count": 1,
        "smoothing_rejection_count": rejection_count,
        "smoothing_deviation_max_m": {"maximum": 0.04},
        "route_cte_max_m": {"maximum": 0.2},
        "fixed_endpoint_shift_m": {"maximum": 0.0},
        "fixed_stop_anchor_shift_m": {"maximum": 0.0},
        "self_intersection_count": 0,
        "minimum_segment_m": {"minimum": 0.25},
        "snapshot_peak_curvature_per_m": {
            "minimum": curvature,
            "median": curvature,
            "p95": curvature,
            "maximum": curvature,
            "count": 1,
        },
        "solve_time_ms": {
            "minimum": 1.0,
            "median": 1.0,
            "p95": 1.0,
            "maximum": 1.0,
            "count": 1,
        },
    }


def _gate_row(curvature: float = 0.003, *, remaining_m: float = 100.0):
    output = _simple_trajectory(_simple_point(0.0), _simple_point(1.0))
    baseline_metrics = {
        "curvature_peak_per_m": 0.02,
        "trajectory_horizon_minimum_mps": 12.0,
        "first_zero_position": None,
    }
    candidate_metrics = {
        "curvature_peak_per_m": curvature,
        "trajectory_horizon_minimum_mps": 12.0,
        "first_zero_position": None,
        "smoothing_applied": True,
    }
    return {
        "snapshot_index": 7,
        "progress_m": 10.0,
        "remaining_m": remaining_m,
        "command": 3,
        "baseline": baseline_metrics,
        "candidate": candidate_metrics,
        "baseline_output": output,
        "candidate_output": copy.deepcopy(output),
        "baseline_candidate_bidirectional_deviation_m": 0.04,
    }


def _evaluate(curvature: float = 0.003, *, oracle_status: str = "PASS"):
    pairing = {
        "active_shaped_pair_count": 1,
        "unpaired_raw_count": 0,
    }
    oracle = {"status": oracle_status}
    baseline = _variant_summary(curvature=0.02)
    candidate = _variant_summary(curvature=curvature)
    return preflight.evaluate_gates(
        pairing,
        oracle,
        baseline,
        candidate,
        [_gate_row(curvature)],
    )


def test_pairing_selects_first_unique_causal_same_stamp_result():
    raw = [_record("raw", 100, 50, 1)]
    final = [
        _record("final", 90, 50, 2),
        _record("final", 101, 50, 3),
        _record("final", 110, 50, 4),
    ]

    pairs, failures, unused = preflight.pair_trajectory_records(raw, final)

    assert failures == []
    assert [pair.final.sequence for pair in pairs] == [3]
    assert unused == 2


def test_pairing_fails_closed_without_causal_result_or_on_tie():
    raw = [_record("raw", 100, 50, 1), _record("raw", 200, 60, 2)]
    final = [
        _record("final", 90, 50, 3),
        _record("final", 201, 60, 4),
        _record("final", 201, 60, 5),
    ]

    pairs, failures, unused = preflight.pair_trajectory_records(raw, final)

    assert pairs == []
    assert any("no causal" in failure for failure in failures)
    assert any("ambiguous" in failure for failure in failures)
    assert unused == 3


def test_oracle_uses_documented_capture_boundary_tolerances():
    expected = _simple_trajectory(_simple_point(0.0), _simple_point(1.0))
    within = copy.deepcopy(expected)
    within.points[1].pose.position.y = 9.9e-4
    within.points[1].pose.orientation.z = math.sin(4.9e-5 * 0.5)
    within.points[1].pose.orientation.w = math.cos(4.9e-5 * 0.5)
    outside = copy.deepcopy(expected)
    outside.points[1].pose.position.y = 1.01e-3

    accepted = preflight.compare_trajectory_payload(within, expected)
    rejected = preflight.compare_trajectory_payload(outside, expected)

    assert accepted["status"] == "PASS"
    assert rejected["status"] == "FAILED"
    assert preflight.ORACLE_XY_TOLERANCE_M == 1.0e-3
    assert preflight.ORACLE_YAW_TOLERANCE_RAD == 5.0e-5


def test_time_from_start_and_scalar_payload_remain_strict():
    expected = _simple_trajectory(_simple_point(0.0), _simple_point(1.0))
    changed_time = copy.deepcopy(expected)
    changed_time.points[1].time_from_start = 1
    changed_scalar = copy.deepcopy(expected)
    changed_scalar.points[1].acceleration_mps2 = 2.0e-6

    assert preflight.compare_trajectory_payload(changed_time, expected)["status"] == "FAILED"
    assert preflight.compare_trajectory_payload(changed_scalar, expected)["status"] == "FAILED"


def test_oracle_separates_all_geometry_from_post_startup_scalar_time():
    expected = _simple_trajectory(_simple_point(0.0), _simple_point(1.0))
    startup_actual = copy.deepcopy(expected)
    startup_actual.points[1].time_from_start = 1
    startup_oracle = preflight.compare_trajectory_payload(startup_actual, expected)
    strict_oracle = preflight.compare_trajectory_payload(expected, expected)
    rows = [
        {
            "snapshot_index": 0,
            "progress_m": 0.0005,
            "baseline_oracle": startup_oracle,
        },
        {
            "snapshot_index": 1,
            "progress_m": 0.0005,
            "baseline_oracle": startup_oracle,
        },
        {
            "snapshot_index": 2,
            "progress_m": 0.2,
            "baseline_oracle": strict_oracle,
        },
    ]

    summary = preflight.summarize_oracle(rows)

    assert summary["status"] == "PASS"
    assert summary["geometry"]["status"] == "PASS"
    assert summary["geometry"]["snapshot_count"] == 3
    assert summary["scalar_time_after_startup"]["status"] == "PASS"
    assert summary["scalar_time_after_startup"]["snapshot_count"] == 1
    assert summary["startup_context_missing"]["excluded_snapshot_indexes"] == [0, 1]
    assert summary["startup_context_missing"]["observed_scalar_time_mismatch_count"] == 2


def test_gate_decision_separates_hard_failure_hold_and_pass():
    passed, passed_gates = _evaluate(0.003)
    hold, hold_gates = _evaluate(0.01)
    failed, failed_gates = _evaluate(0.003, oracle_status="FAILED")

    assert passed == "PASS"
    assert hold == "HOLD"
    assert hold_gates["checks"]["candidate_curvature_p95_supports_15mps"]["status"] == "FAIL"
    assert hold_gates["checks"]["candidate_curvature_max_supports_15mps"]["status"] == "FAIL"
    assert failed == "FAILED"
    assert failed_gates["checks"]["baseline_strength10_oracle"]["severity"] == "hard"
    assert passed_gates["midroute"]["eligible_snapshot_count"] == 1


def test_curvature_gate_excludes_terminal_snapshots_but_not_midroute_peaks():
    eligible = _gate_row(0.01, remaining_m=100.0)
    terminal = _gate_row(0.0001, remaining_m=10.0)
    terminal["snapshot_index"] = 8
    baseline = _variant_summary(curvature=0.02)
    baseline["snapshot_count"] = 2
    candidate = _variant_summary(curvature=0.0001)
    candidate["snapshot_count"] = 2

    status, gates = preflight.evaluate_gates(
        {"active_shaped_pair_count": 2, "unpaired_raw_count": 0},
        {"status": "PASS"},
        baseline,
        candidate,
        [eligible, terminal],
    )

    assert status == "HOLD"
    metric = gates["midroute"][
        "candidate_adjacent_point_snapshot_peak_curvature_per_m"
    ]
    assert metric["count"] == 1
    assert metric["p95"] == pytest.approx(0.01)
    assert gates["midroute"]["candidate_curvature_limit_violation_count"] == 1


def test_self_intersection_check_has_fast_monotone_path_and_crossing_fallback():
    monotone = [_simple_point(float(index), 0.1 * index) for index in range(6)]
    crossing = [
        _simple_point(0.0, 0.0),
        _simple_point(2.0, 2.0),
        _simple_point(0.0, 2.0),
        _simple_point(2.0, 0.0),
    ]

    assert preflight.count_self_intersections(monotone) == 0
    assert preflight.count_self_intersections(crossing) == 1


def test_production_shape_preserves_corridor_endpoints_and_stop_contract():
    route_points = [
        SimpleNamespace(
            x=0.0,
            y=0.0,
            z=0.0,
            yaw=0.0,
            distance_m=0.0,
            vad_command=3,
            road_option="LANEFOLLOW",
        ),
        SimpleNamespace(
            x=200.0,
            y=0.0,
            z=0.0,
            yaw=0.0,
            distance_m=200.0,
            vad_command=3,
            road_option="LANEFOLLOW",
        ),
    ]
    route = preflight.RoutePlan({}, route_points)
    raw = Trajectory()
    raw.header.frame_id = "map"
    for index in range(7):
        point = TrajectoryPoint()
        point.pose.position.x = 10.0 + 4.0 * index
        point.pose.position.y = 0.05 * (-1.0 if index % 2 else 1.0)
        point.pose.orientation.w = 1.0
        point.longitudinal_velocity_mps = 16.0
        point.time_from_start.sec = index
        raw.points.append(point)
    parameters = dict(preflight.PARAMETER_CONTRACT)
    pre_points, stop_distance, lateral_min, lateral_max, _ = (
        preflight.build_pre_smoothing_points(
            raw,
            route,
            progress_m=10.0,
            remaining_m=190.0,
            command=3,
            parameters=parameters,
        )
    )

    output, metrics = preflight.shape_variant(
        raw,
        pre_points,
        route,
        preflight.route_polyline(route),
        progress_m=10.0,
        stop_distance_m=stop_distance,
        lateral_min_m=lateral_min,
        lateral_max_m=lateral_max,
        strength=10000.0,
        parameters=parameters,
    )

    assert metrics["smoothing_applied"] is True
    assert metrics["rejection"] is None
    assert metrics["route_cte_max_m"] <= 0.2 + preflight.CORRIDOR_TOLERANCE_M
    assert metrics["fixed_endpoint_shift_m"] <= preflight.FIXED_POINT_TOLERANCE_M
    assert metrics["self_intersection_count"] == 0
    times_ns = [
        point.time_from_start.sec * 1_000_000_000
        + point.time_from_start.nanosec
        for point in output.points
    ]
    assert all(left < right for left, right in zip(times_ns, times_ns[1:]))


def test_endpoint_tapered_c1_shape_is_explicit_and_keeps_default_isolated():
    route = preflight.RoutePlan(
        {},
        [
            SimpleNamespace(
                x=0.0,
                y=0.0,
                z=0.0,
                yaw=0.0,
                distance_m=0.0,
                vad_command=3,
                road_option="LANEFOLLOW",
            ),
            SimpleNamespace(
                x=50.0,
                y=0.0,
                z=0.0,
                yaw=0.0,
                distance_m=50.0,
                vad_command=3,
                road_option="LANEFOLLOW",
            ),
        ],
    )
    raw = Trajectory()
    raw.header.frame_id = "map"
    for index, offset in enumerate((0.0, 0.1, 0.28, 0.28, 0.1, 0.0)):
        point = TrajectoryPoint()
        point.pose.position.x = 5.0 + 2.0 * index
        point.pose.position.y = offset
        point.pose.orientation.w = 1.0
        point.longitudinal_velocity_mps = 16.0
        point.time_from_start.sec = index
        raw.points.append(point)
    parameters = dict(preflight.PARAMETER_CONTRACT)
    pre_points, stop_distance, lateral_min, lateral_max, _ = (
        preflight.build_pre_smoothing_points(
            raw,
            route,
            progress_m=5.0,
            remaining_m=45.0,
            command=3,
            parameters=parameters,
        )
    )

    _, metrics = preflight.shape_variant(
        raw,
        pre_points,
        route,
        preflight.route_polyline(route),
        progress_m=5.0,
        stop_distance_m=stop_distance,
        lateral_min_m=lateral_min,
        lateral_max_m=lateral_max,
        strength=10000.0,
        parameters=parameters,
        corridor_saturation_mode="endpoint_tapered_c1",
        corridor_transition_width_m=0.0125,
        corridor_endpoint_taper_m=5.0,
    )

    assert metrics["corridor_saturation_mode"] == "endpoint_tapered_c1"
    assert metrics["corridor_transition_width_m"] == pytest.approx(0.0125)
    assert metrics["corridor_endpoint_taper_m"] == pytest.approx(5.0)
    assert metrics["rejection"] is None
    assert metrics["route_cte_max_m"] <= 0.2 + preflight.CORRIDOR_TOLERANCE_M


def test_c1_cli_requires_explicit_mode_and_parameters():
    parsed = preflight.parse_args(
        [
            "--candidate-corridor-saturation",
            "endpoint_tapered_c1",
            "--candidate-transition-width-m",
            "0.0125",
            "--candidate-endpoint-taper-m",
            "5.0",
        ]
    )

    assert parsed.candidate_corridor_saturation == "endpoint_tapered_c1"
    assert parsed.candidate_transition_width_m == pytest.approx(0.0125)
    assert parsed.candidate_endpoint_taper_m == pytest.approx(5.0)


def test_failed_payload_never_authorizes_live_or_real_vehicle_use():
    payload = preflight.failed_payload(Path("/tmp/v4"), "pinned SHA mismatch")

    assert payload["status"] == "FAILED"
    assert payload["real_vehicle_ready"] is False
    assert payload["validation_boundary"]["live_ab_authorized"] is False
    assert payload["validation_boundary"]["real_vehicle_ready"] is False

    c1_payload = preflight.failed_payload(
        Path("/tmp/v4"), "gate mismatch", "endpoint_tapered_c1"
    )
    assert c1_payload["analysis"].endswith("c1_corridor_preflight_v1")
    assert c1_payload["experiment"]["candidate_corridor_saturation_mode"] == (
        "endpoint_tapered_c1"
    )
