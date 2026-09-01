from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest


pytest.importorskip("rosbag2_py")

MODULE_PATH = Path(__file__).parents[1] / "scripts" / "e2e" / "analyze_turn_dynamics.py"
SPEC = importlib.util.spec_from_file_location("analyze_turn_dynamics", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
analyzer = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(analyzer)


def _trajectory(
    xy: np.ndarray, bag_sec: float = 1.0, stamp_sec: float = 1.0
) -> dict[str, object]:
    return {
        "bag_ns": int(round(bag_sec * 1.0e9)),
        "stamp_ns": int(round(stamp_sec * 1.0e9)),
        "xy": np.asarray(xy, dtype=float),
        "relative_time": np.arange(len(xy), dtype=float) * 0.1,
    }


def test_record_times_use_message_stamp_with_receipt_fallback() -> None:
    records = [
        {"bag_ns": 40_000_000_000, "stamp_ns": 2_000_000_000},
        {"bag_ns": 44_000_000_000, "stamp_ns": 0},
    ]

    assert analyzer._record_times(records).tolist() == [2.0, 44.0]
    assert analyzer._receipt_times(records).tolist() == [40.0, 44.0]


def test_predicted_topic_prefers_current_name_and_supports_legacy_bags() -> None:
    current, legacy = analyzer.PREDICTED_TOPICS
    records = {current: [{"value": "current"}], legacy: [{"value": "legacy"}]}

    topic, selected = analyzer._first_topic_with_records(records, analyzer.PREDICTED_TOPICS)
    assert topic == current
    assert selected == [{"value": "current"}]

    topic, selected = analyzer._first_topic_with_records(
        {legacy: [{"value": "legacy"}]}, analyzer.PREDICTED_TOPICS
    )
    assert topic == legacy
    assert selected == [{"value": "legacy"}]


def test_control_topic_prefers_full_then_minimal_then_gated() -> None:
    full, minimal, gated = analyzer.CONTROL_TOPICS
    records = {
        full: [{"value": "full"}],
        minimal: [{"value": "minimal"}],
        gated: [{"value": "gated"}],
    }

    topic, selected = analyzer._first_topic_with_records(records, analyzer.CONTROL_TOPICS)
    assert topic == full
    assert selected == [{"value": "full"}]
    assert analyzer._control_selection_metadata(topic) == {
        "selected_control_stage": "trajectory_follower_raw",
        "quality_warnings": [],
    }

    topic, selected = analyzer._first_topic_with_records(
        {minimal: [{"value": "minimal"}], gated: [{"value": "gated"}]},
        analyzer.CONTROL_TOPICS,
    )
    assert topic == minimal
    assert selected == [{"value": "minimal"}]


def test_gated_control_fallback_has_data_quality_warning() -> None:
    metadata = analyzer._control_selection_metadata(analyzer.GATED_CONTROL_TOPIC)

    assert metadata["selected_control_stage"] == "vehicle_cmd_gate"
    assert len(metadata["quality_warnings"]) == 1
    assert analyzer.GATED_CONTROL_TOPIC in metadata["quality_warnings"][0]
    assert "MRM intervention" in metadata["quality_warnings"][0]


def test_future_prediction_uses_simulation_stamp_not_scaled_receipt_time() -> None:
    odometry = [
        {
            "bag_ns": int(sim_sec * 4.0e9),
            "stamp_ns": int(sim_sec * 1.0e9),
            "x": sim_sec,
            "y": 0.0,
            "speed": 1.0,
        }
        for sim_sec in (0.0, 1.0, 2.0, 3.0)
    ]
    predicted = [
        {
            "bag_ns": 4_000_000_000,
            "stamp_ns": 1_000_000_000,
            "xy": np.asarray([[1.0, 0.0], [2.0, 0.0], [3.0, 0.0]]),
            "relative_time": np.asarray([0.0, 1.0, 2.0]),
        }
    ]

    metrics, _ = analyzer._prediction_future_metrics(predicted, odometry)

    assert metrics["horizon_1.0_sec_error_m"]["max_abs"] == pytest.approx(0.0)
    assert metrics["horizon_2.0_sec_error_m"]["max_abs"] == pytest.approx(0.0)


def test_common_progress_geometry_ignores_density_and_unmatched_extent() -> None:
    route_x = np.linspace(0.0, 20.0, 81)
    route_xy = np.column_stack((route_x, np.zeros_like(route_x)))
    route_progress = route_x.copy()
    sparse_x = np.asarray([0.0, 5.0, 10.0])
    dense_x = np.linspace(2.0, 15.0, 105)
    source = [_trajectory(np.column_stack((sparse_x, np.zeros_like(sparse_x))))]
    target = [
        _trajectory(
            np.column_stack((dense_x, np.full_like(dense_x, 0.5))),
            bag_sec=1.1,
            stamp_sec=1.0,
        )
    ]

    metrics, samples = analyzer._pair_trajectory_geometry(
        source, target, route_xy, route_progress
    )

    assert metrics["pair_count"] == 1
    assert samples[0]["overlap_m"] == pytest.approx(8.0)
    assert samples[0]["common_progress_p95_m"] == pytest.approx(0.5)
    assert samples[0]["common_progress_max_m"] == pytest.approx(0.5)


def test_route_profile_clips_raw_horizon_at_route_end() -> None:
    route_x = np.linspace(0.0, 12.0, 49)
    route_xy = np.column_stack((route_x, np.zeros_like(route_x)))
    raw_xy = np.asarray(
        [
            [10.5, 0.15],
            [14.5, 0.10],
            [18.5, -0.40],
            [22.5, -1.20],
        ]
    )

    progress, offset = analyzer._trajectory_route_profile(
        _trajectory(raw_xy), route_xy, route_x
    )

    assert progress[0] == pytest.approx(10.5)
    assert progress[-1] == pytest.approx(12.0, abs=0.02)
    assert np.max(np.abs(offset)) < 0.2


def test_route_profile_does_not_clip_horizon_before_route_end() -> None:
    route_x = np.linspace(0.0, 30.0, 121)
    route_xy = np.column_stack((route_x, np.zeros_like(route_x)))
    raw_xy = np.asarray([[10.0, 0.1], [14.0, 0.2], [18.0, 0.3]])

    clipped = analyzer._clip_trajectory_to_route_horizon(
        raw_xy, route_xy, route_x
    )

    assert clipped == pytest.approx(raw_xy)


def test_sparse_trajectory_is_densified_without_moving_endpoints() -> None:
    sparse = np.asarray([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])

    dense = analyzer._densify_polyline(sparse, interval_m=0.25)

    assert dense[0] == pytest.approx(sparse[0])
    assert dense[-1] == pytest.approx(sparse[-1])
    assert np.max(np.linalg.norm(np.diff(dense, axis=0), axis=1)) <= 0.25 + 1.0e-12


def test_pair_geometry_ignores_raw_tail_beyond_short_route_goal() -> None:
    route_x = np.linspace(0.0, 12.0, 49)
    route_xy = np.column_stack((route_x, np.zeros_like(route_x)))
    raw_xy = np.asarray(
        [
            [10.7, 0.15],
            [14.7, 0.10],
            [18.7, -0.40],
            [22.7, -1.10],
            [26.7, -1.25],
            [30.7, -1.30],
            [34.1, -1.35],
        ]
    )
    retained = analyzer._clip_trajectory_to_route_horizon(
        raw_xy, route_xy, route_x
    )
    final_xy = analyzer._densify_polyline(retained)
    source = [_trajectory(raw_xy)]
    target = [_trajectory(final_xy, bag_sec=1.01, stamp_sec=1.0)]

    metrics, samples = analyzer._pair_trajectory_geometry(
        source, target, route_xy, route_x
    )

    assert metrics["pair_count"] == 1
    assert samples[0]["common_progress_p95_m"] < 1.0e-9
    assert samples[0]["common_progress_max_m"] < 1.0e-9


def test_route_end_clip_keeps_minimum_nonzero_horizon_at_goal() -> None:
    route_x = np.linspace(0.0, 12.0, 49)
    route_xy = np.column_stack((route_x, np.zeros_like(route_x)))
    raw_xy = np.asarray([[12.0, 0.0], [13.0, 0.1], [14.0, 0.2]])

    clipped = analyzer._clip_trajectory_to_route_horizon(
        raw_xy, route_xy, route_x
    )

    assert len(clipped) == 2
    assert np.linalg.norm(clipped[-1] - clipped[0]) == pytest.approx(0.1)


def test_route_metrics_weight_snapshots_equally_despite_point_density() -> None:
    route_x = np.linspace(0.0, 20.0, 81)
    route_xy = np.column_stack((route_x, np.zeros_like(route_x)))
    sparse_x = np.asarray([0.0, 10.0, 20.0])
    dense_x = np.linspace(0.0, 20.0, 401)
    records = [
        _trajectory(np.column_stack((sparse_x, np.full_like(sparse_x, 0.5)))),
        _trajectory(np.column_stack((dense_x, np.full_like(dense_x, 0.5)))),
    ]

    metrics, errors, _ = analyzer._trajectory_route_metrics(records, route_xy, route_x)

    assert len(errors) == 82
    assert metrics["mean"] == pytest.approx(0.5)
    assert metrics["p95_abs"] == pytest.approx(0.5)


def test_pose_yaw_rate_sorts_deduplicates_unwraps_and_caps_window() -> None:
    times = np.asarray([0.002, 0.0, 0.001, 0.001])
    continuous_yaw = math.pi - 0.0005 + times
    wrapped_yaw = (continuous_yaw + math.pi) % (2.0 * math.pi) - math.pi

    rate = analyzer._pose_derived_yaw_rate(times, wrapped_yaw)

    assert rate.shape == times.shape
    assert np.all(np.isfinite(rate))
    assert rate == pytest.approx(np.ones_like(rate), abs=1.0e-9)


def test_frenet_decomposition_uses_one_route_normal() -> None:
    route_x = np.linspace(0.0, 20.0, 81)
    route_xy = np.column_stack((route_x, np.zeros_like(route_x)))
    path_x = np.linspace(2.0, 18.0, 17)
    snapshot = _trajectory(np.column_stack((path_x, np.full_like(path_x, 0.4))))

    context = analyzer._trajectory_frenet_context(
        snapshot,
        actual_route_offset_m=1.2,
        actual_progress_m=10.0,
        route_xy=route_xy,
        route_progress=route_x,
    )

    assert context["path_offset_at_actual_progress_m"] == pytest.approx(0.4)
    assert context["actual_minus_path_offset_m"] == pytest.approx(0.8)
    assert context["decomposition_residual_m"] == pytest.approx(0.0)
    assert context["path_offset_at_lookahead_m"] == pytest.approx(0.4)


def test_turn_geometry_window_starts_at_route_manager_command_switch() -> None:
    route_progress = np.arange(0.0, 16.0)
    route_command = np.full(len(route_progress), 3, dtype=int)
    route_command[8:13] = 1

    commanded_turn = analyzer._turn_progress_ranges(route_progress, route_command)
    command_switch = analyzer._turn_progress_ranges(
        route_progress,
        route_command,
        margin_m=0.0,
        maneuver_lookahead_m=4.0,
    )
    analysis_window = analyzer._turn_progress_ranges(
        route_progress,
        route_command,
        maneuver_lookahead_m=4.0,
    )

    assert commanded_turn == [(6.0, 14.0)]
    assert command_switch == [(4.0, 12.0)]
    assert analysis_window == [(2.0, 14.0)]


@pytest.mark.parametrize("lookahead", [-1.0, math.inf, math.nan])
def test_turn_geometry_window_rejects_invalid_lookahead(lookahead: float) -> None:
    with pytest.raises(ValueError, match="finite and non-negative"):
        analyzer._turn_progress_ranges(
            np.asarray([0.0, 1.0]),
            np.asarray([3, 1]),
            maneuver_lookahead_m=lookahead,
        )


def test_peak_corner_context_excludes_stopped_and_non_turn_samples() -> None:
    route_x = np.linspace(0.0, 20.0, 21)
    route_xy = np.column_stack((route_x, np.zeros_like(route_x)))
    route_command = np.full(len(route_x), 3, dtype=int)
    route_command[8:13] = 1
    tracking = [
        {
            "bag_sec": 1.0,
            "time_sec": 1.0,
            "route_cte_m": -8.0,
            "route_progress_m": 2.0,
            "route_curvature_per_m": -0.1,
            "speed_mps": 1.0,
            "final_cte_m": -7.5,
            "route_yaw_error_rad": 0.0,
            "final_yaw_error_rad": 0.0,
            "pose_yaw_rate_rps": -0.1,
        },
        {
            "bag_sec": 2.0,
            "time_sec": 2.0,
            "route_cte_m": -5.0,
            "route_progress_m": 10.0,
            "route_curvature_per_m": -0.1,
            "speed_mps": 0.0,
            "final_cte_m": -4.5,
            "route_yaw_error_rad": 0.0,
            "final_yaw_error_rad": 0.0,
            "pose_yaw_rate_rps": 0.0,
        },
        {
            "bag_sec": 3.0,
            "time_sec": 3.0,
            "route_cte_m": -1.2,
            "route_progress_m": 10.0,
            "route_curvature_per_m": -0.1,
            "speed_mps": 2.0,
            "final_cte_m": -0.8,
            "route_yaw_error_rad": 0.1,
            "final_yaw_error_rad": 0.1,
            "pose_yaw_rate_rps": -0.2,
        },
    ]
    odometry = [
        {
            "bag_ns": int(sample["bag_sec"] * 1.0e9),
            "stamp_ns": int(sample["time_sec"] * 1.0e9),
            "x": sample["route_progress_m"],
            "y": sample["route_cte_m"],
        }
        for sample in tracking
    ]
    path_x = np.linspace(0.0, 20.0, 21)
    raw = [_trajectory(np.column_stack((path_x, np.full_like(path_x, -0.6))), 2.9, 2.9)]
    final = [_trajectory(np.column_stack((path_x, np.full_like(path_x, -0.4))), 2.9, 2.9)]

    context = analyzer._peak_corner_context(
        tracking,
        raw,
        final,
        [],
        [],
        [],
        [],
        odometry,
        route_xy,
        route_x,
        route_command,
        0.0,
        "virtual",
        2.79,
        1.64,
    )

    assert context["selection"] == "inward_corner_cut"
    assert context["route_progress_m"] == pytest.approx(10.0)
    assert context["actual_to_route_signed_cte_m"] == pytest.approx(-1.2)
    assert context["route_frenet"]["final"]["decomposition_residual_m"] == pytest.approx(0.0)


def test_predicted_and_steering_topics_are_optional() -> None:
    records = {
        analyzer.RAW_TOPIC: [{"value": 1}],
        analyzer.FINAL_TOPIC: [{"value": 1}],
        analyzer.ODOMETRY_TOPIC: [{"value": 1}],
    }

    assert analyzer._missing_required_topics(records) == []


def test_config_mismatch_alone_does_not_claim_control_dominance() -> None:
    metrics = {
        "steering_tracking": {
            "first_order_fit": {"delay_sec": 0.0, "time_constant_sec": 0.5}
        },
        "tracking": {
            "actual_to_final_cte_m": {"p95_abs": 0.01, "max_abs": 0.02},
            "actual_to_route_cte_m": {"max_abs": 0.02},
        },
    }

    verdict = analyzer._classify(metrics, 0.24, 0.27)

    assert verdict["control_score"] == 0
    assert verdict["classification"] == "weak_evidence"


def test_complete_metrics_below_thresholds_are_not_insufficient_evidence() -> None:
    metrics = {
        "raw_to_final_geometry": {"common_progress_p95_m": {"p95_abs": 0.04}},
        "raw_path": {"p95_abs": 0.15},
        "final_path": {"p95_abs": 0.16, "max_abs": 0.18},
        "tracking": {
            "actual_to_final_cte_m": {"p95_abs": 0.001, "max_abs": 0.002},
            "actual_to_route_cte_m": {"max_abs": 0.16},
        },
    }

    verdict = analyzer._classify(metrics, 0.12, 0.15)

    assert verdict["classification"] == "within_thresholds"
    assert verdict["planning_score"] == 0
    assert verdict["control_score"] == 0
