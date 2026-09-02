from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).parents[1]
MODULE_PATH = ROOT / "scripts/e2e/analyze_longitudinal_response.py"
SPEC = importlib.util.spec_from_file_location(
    "analyze_longitudinal_response", MODULE_PATH
)
assert SPEC is not None and SPEC.loader is not None
analyzer = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(analyzer)


def _record(stamp_sec: float, **values: float) -> dict[str, object]:
    stamp_ns = int(round(stamp_sec * 1.0e9))
    return {
        "bag_ns": stamp_ns + 1_000,
        "stamp_ns": stamp_ns,
        **values,
    }


def _complete_records() -> dict[str, list[dict[str, object]]]:
    raw = [
        _record(1.0, speed=0.0, acceleration=0.0, jerk=0.0),
        _record(2.0, speed=8.0, acceleration=1.8, jerk=0.2),
        _record(3.0, speed=8.333, acceleration=1.8, jerk=0.0),
        _record(4.0, speed=8.333, acceleration=-0.5, jerk=-0.2),
    ]
    gated = [
        _record(1.0, speed=0.0, acceleration=0.0, jerk=0.0),
        _record(2.0, speed=7.5, acceleration=1.5, jerk=0.1),
        _record(3.0, speed=8.333, acceleration=1.5, jerk=0.0),
        _record(4.0, speed=7.0, acceleration=-0.5, jerk=-0.1),
    ]
    odometry = [
        _record(1.0, speed=0.0),
        _record(2.0, speed=4.0),
        _record(3.0, speed=7.6),
        _record(4.0, speed=7.7),
    ]
    measured = [
        _record(1.0, acceleration=0.0),
        _record(2.0, acceleration=1.0),
        _record(3.0, acceleration=1.1),
        _record(4.0, acceleration=-0.3),
    ]
    command = [
        _record(1.0, accel=0.0, brake=0.0),
        _record(2.0, accel=0.4, brake=0.0),
        _record(3.0, accel=0.4, brake=0.0),
        _record(4.0, accel=0.0, brake=0.6),
    ]
    status = [
        _record(1.0, accel=0.0, brake=0.0),
        _record(2.0, accel=0.4, brake=0.0),
        _record(3.0, accel=0.4, brake=0.0),
        _record(4.0, accel=0.0, brake=0.6),
    ]
    return {
        analyzer.RAW_CONTROL_TOPICS[0]: raw,
        analyzer.GATED_CONTROL_TOPIC: gated,
        analyzer.ODOMETRY_TOPIC: odometry,
        analyzer.ACCELERATION_TOPIC: measured,
        analyzer.ACTUATION_COMMAND_TOPIC: command,
        analyzer.ACTUATION_STATUS_TOPIC: status,
    }


def _topic_types() -> dict[str, str]:
    return {
        analyzer.RAW_CONTROL_TOPICS[0]: "autoware_control_msgs/msg/Control",
        analyzer.GATED_CONTROL_TOPIC: "autoware_control_msgs/msg/Control",
        analyzer.ODOMETRY_TOPIC: "nav_msgs/msg/Odometry",
        analyzer.ACCELERATION_TOPIC: (
            "geometry_msgs/msg/AccelWithCovarianceStamped"
        ),
        analyzer.ACTUATION_COMMAND_TOPIC: (
            "tier4_vehicle_msgs/msg/ActuationCommandStamped"
        ),
        analyzer.ACTUATION_STATUS_TOPIC: (
            "tier4_vehicle_msgs/msg/ActuationStatusStamped"
        ),
    }


def _identity(status: str = "PASS") -> dict[str, object]:
    return {
        "route_result": {
            "success": status == "PASS",
            "speed_exposure": {
                "status": status,
                "minimum_sustained_speed_mps": 7.5,
                "minimum_sustained_speed_sec": 1.0,
                "maximum_observed_speed_limit_mps": 9.0,
                "maximum_observed_speed_mps": 7.7,
                "maximum_sustained_speed_duration_sec": (
                    1.0 if status == "PASS" else 0.0
                ),
                "continuity_maximum_gap_sec": 1.1,
            },
        }
    }


def _build(
    records: dict[str, list[dict[str, object]]] | None = None,
    **kwargs: object,
) -> dict[str, object]:
    return analyzer.build_evidence(
        records or _complete_records(),
        _topic_types(),
        bag=Path("/tmp/synthetic_longitudinal_bag"),
        profile_id="carla_vad_30kph_v2",
        target_speed_mps=8.333333333333334,
        longitudinal_speed_source="explicit_simulation_nominal",
        source_identity=_identity(),
        **kwargs,
    )


def _write_bound_trial(
    tmp_path: Path, *, exposure_status: str = "PASS"
) -> tuple[Path, Path, Path]:
    bag = tmp_path / "bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("version: 9\n", encoding="utf-8")
    (bag / "bag_0.db3").write_bytes(b"synthetic-bag")
    route = tmp_path / "aligned_route.json"
    route.write_text(
        json.dumps(
            {
                "town": "Town06",
                "scenario": "straight",
                "route_length_m": 210.0,
            }
        ),
        encoding="utf-8",
    )
    result = tmp_path / "result.json"
    result.write_text(
        json.dumps(
            {
                "success": exposure_status == "PASS",
                "execution_mode": "full_stack",
                "route_file": str(route),
                "profile_context": analyzer.EXPECTED_PROFILE_CONTEXT,
                "limits": {"maximum_speed_sample_gap_sec": 0.25},
                "speed_exposure": {
                    **analyzer.EXPECTED_PROFILE_CONTEXT,
                    "status": exposure_status,
                    "minimum_sustained_speed_mps": 7.5,
                    "minimum_sustained_speed_sec": 1.0,
                    "maximum_observed_speed_limit_mps": 9.0,
                    "maximum_observed_speed_mps": 7.7,
                    "maximum_sustained_speed_duration_sec": (
                        1.2 if exposure_status == "PASS" else 0.0
                    ),
                },
                "reason": (
                    "goal reached"
                    if exposure_status == "PASS"
                    else "speed exposure contract failed"
                ),
            }
        ),
        encoding="utf-8",
    )
    return bag, route, result


def test_module_import_does_not_require_ros_python_modules() -> None:
    assert callable(analyzer.build_evidence)
    assert callable(analyzer._read_bag)


def test_evidence_separates_commands_actuation_and_vehicle_response() -> None:
    evidence = _build()

    assert evidence["status"] == "complete"
    assert evidence["sources"]["raw_control"]["topic"] == (
        analyzer.RAW_CONTROL_TOPICS[0]
    )
    assert evidence["summary"]["raw_target_speed_mps"]["maximum"] == (
        pytest.approx(8.333)
    )
    assert evidence["summary"]["gated_acceleration_mps2"]["maximum"] == 1.5
    assert evidence["summary"]["actual_speed_mps"]["maximum"] == 7.7
    assert evidence["summary"]["accel_command"]["maximum"] == 0.4
    assert evidence["summary"]["brake_status"]["maximum"] == 0.6
    assert evidence["alignment"]["primary_time"].startswith("ROS message/header")
    assert evidence["interpretation"][
        "requested_target_is_converter_lookup_velocity"
    ] is False
    assert evidence["interpretation"]["converter_lookup_velocity_source"] == (
        "absolute_current_odometry_longitudinal_speed_mps"
    )


def test_robust_acceleration_rejects_a_large_isolated_outlier() -> None:
    samples = [
        {
            "time_sec": index * 0.05,
            "time_source": "header_stamp",
            "bag_time_ns": index,
            "header_time_ns": index,
            "acceleration": value,
        }
        for index, value in enumerate((1.0, 1.1, 80.0, 0.9, 1.0))
    ]

    robust, metrics = analyzer._robust_acceleration(samples)

    assert robust[2]["outlier"] is True
    assert robust[2]["robust_acceleration_mps2"] == pytest.approx(1.0)
    assert metrics["outlier_count"] == 1
    assert metrics["robust_summary_mps2"]["maximum"] < 2.0


def test_duty_and_longest_duration_exclude_large_time_gaps() -> None:
    samples = [
        {"time_sec": 0.0, "value": 1.0},
        {"time_sec": 0.1, "value": 1.0},
        {"time_sec": 0.2, "value": 0.0},
        {"time_sec": 2.0, "value": 1.0},
        {"time_sec": 2.1, "value": 1.0},
    ]

    duty = analyzer._duty_metrics(
        samples,
        "value",
        lambda value: value > 0.5,
        maximum_gap_sec=0.25,
        definition="value > 0.5",
    )

    assert duty["active_sample_count"] == 4
    assert duty["sample_fraction_percent"] == pytest.approx(80.0)
    assert duty["active_interval_duration_sec"] == pytest.approx(0.2)
    assert duty["eligible_interval_duration_sec"] == pytest.approx(0.3)
    assert duty["longest_contiguous_duration_sec"] == pytest.approx(0.1)
    assert duty["excluded_gap_duration_sec"] == pytest.approx(1.8)


def test_saturation_and_target_exposure_are_machine_readable() -> None:
    evidence = _build()
    duty = evidence["saturation_and_duty"]

    assert duty["raw_acceleration_above_gate_limit"]["active_sample_count"] == 2
    assert duty["gated_positive_acceleration_limit"]["active_sample_count"] == 2
    assert duty["accel_command_near_saturation"]["active_sample_count"] == 2
    assert duty["brake_command_near_saturation"]["active_sample_count"] == 1
    assert duty["accel_status_active"]["active_sample_count"] == 2
    assert duty["brake_status_active"]["active_sample_count"] == 1
    contract = evidence["target_exposure"]["route_contract_minimum"]
    assert contract["active_sample_count"] == 2
    assert evidence["target_exposure"]["route_result_reported"]["status"] == "PASS"
    cross_check = evidence["target_exposure"]["route_result_cross_check"]
    assert cross_check["maximum_speed_consistent_with_bag"] is True
    assert cross_check["minimum_duration_condition_met_from_bag"] is True
    assert cross_check["duration_consistent_within_one_continuity_gap"] is True


def test_zero_duration_turn_contract_does_not_emit_a_false_duration_warning() -> None:
    identity = _identity()
    exposure = identity["route_result"]["speed_exposure"]
    exposure["minimum_sustained_speed_mps"] = 0.0
    exposure["minimum_sustained_speed_sec"] = 0.0
    exposure["maximum_sustained_speed_duration_sec"] = 40.0

    evidence = analyzer.build_evidence(
        _complete_records(),
        _topic_types(),
        bag=Path("/tmp/synthetic_turn_longitudinal_bag"),
        profile_id="carla_vad_30kph_v2",
        target_speed_mps=8.333333333333334,
        longitudinal_speed_source="explicit_simulation_nominal",
        source_identity=identity,
    )

    cross_check = evidence["target_exposure"]["route_result_cross_check"]
    assert cross_check["duration_comparison_applicable"] is False
    assert cross_check["duration_consistent_within_one_continuity_gap"] is None
    assert not evidence["quality"]["warnings"]


def test_missing_required_actuation_status_is_incomplete() -> None:
    records = _complete_records()
    records[analyzer.ACTUATION_STATUS_TOPIC] = []

    evidence = _build(records)

    assert evidence["status"] == "incomplete"
    assert any(
        analyzer.ACTUATION_STATUS_TOPIC in problem
        for problem in evidence["quality"]["problems"]
    )


def test_legacy_raw_control_topic_is_supported() -> None:
    records = _complete_records()
    records[analyzer.RAW_CONTROL_TOPICS[1]] = records.pop(
        analyzer.RAW_CONTROL_TOPICS[0]
    )

    evidence = _build(records)

    assert evidence["status"] == "complete"
    assert evidence["sources"]["raw_control"]["topic"] == (
        analyzer.RAW_CONTROL_TOPICS[1]
    )


@pytest.mark.parametrize("exposure_status", ["PASS", "FAIL"])
def test_source_identity_accepts_pass_and_speed_exposure_fail(
    tmp_path: Path, exposure_status: str
) -> None:
    bag, route, result = _write_bound_trial(
        tmp_path, exposure_status=exposure_status
    )

    identity = analyzer._source_identity(
        bag,
        route,
        result,
        profile_id="carla_vad_30kph_v2",
        target_speed_mps=8.333333333333334,
        longitudinal_speed_source="explicit_simulation_nominal",
    )

    assert identity["route_result"]["speed_exposure"]["status"] == (
        exposure_status
    )
    assert identity["profile"][
        "requested_target_is_converter_lookup_velocity"
    ] is False
    assert identity["rosbag"]["files"][0]["sha256"]
    assert identity["sha256"] == analyzer._sha256_json(
        {key: value for key, value in identity.items() if key != "sha256"}
    )


def test_source_identity_rejects_profile_target_outside_result_envelope(
    tmp_path: Path,
) -> None:
    bag, route, result = _write_bound_trial(tmp_path)

    with pytest.raises(analyzer.AnalysisError, match="outside.*envelope"):
        analyzer._source_identity(
            bag,
            route,
            result,
            profile_id="wrong_60kph_profile",
            target_speed_mps=16.666666666666668,
            longitudinal_speed_source="explicit_simulation_nominal",
        )


def test_actuation_map_context_keeps_target_and_runtime_lookup_distinct(
    tmp_path: Path,
) -> None:
    coverage_path = tmp_path / "actuation_map_runtime_coverage.json"
    coverage_path.write_text(
        json.dumps(
            {
                "analysis": "raw_vehicle_command_converter_velocity_coverage",
                "status": "EXPLORATORY",
                "profile_id": "carla_vad_60kph_straight_pilot_v1",
                "target_speed_mps": 16.666666666666668,
                "target_envelope_classification": (
                    "TARGET_ENVELOPE_EXCEEDS_MAP_AXIS_CLAMP_IF_REACHED"
                ),
                "target_within_map_velocity_axis": False,
                "map_velocity_axis_minimum_mps": 0.0,
                "map_velocity_axis_maximum_mps": 13.89,
                "runtime_lookup_observation": {
                    "available": True,
                    "classification": "OBSERVED_LOOKUPS_WITHIN_MAP_AXIS",
                    "maximum_absolute_current_speed_mps": 9.77,
                    "velocity_axis_clamping_observed": False,
                },
                "validation_boundary": {
                    "target_speed_is_converter_lookup_velocity": False,
                    "converter_lookup_velocity_source": (
                        "absolute_current_odometry_longitudinal_speed_mps"
                    ),
                },
            }
        ),
        encoding="utf-8",
    )

    context = analyzer._map_coverage_context(
        coverage_path,
        profile_id="carla_vad_60kph_straight_pilot_v1",
        target_speed_mps=16.666666666666668,
    )

    assert context["target_within_map_velocity_axis"] is False
    assert context["runtime_lookup_observation"][
        "velocity_axis_clamping_observed"
    ] is False
    assert context["interpretation"][
        "requested_target_is_converter_lookup_velocity"
    ] is False


def test_build_cross_checks_runtime_map_speed_against_bag() -> None:
    map_context = {
        "provided": True,
        "runtime_lookup_observation": {
            "available": True,
            "maximum_absolute_current_speed_mps": 7.7,
        },
    }

    evidence = _build(map_coverage=map_context)

    assert evidence["status"] == "complete"
    assert evidence["actuation_map_coverage"]["runtime_cross_check"][
        "consistent_with_bag"
    ] is True


def test_run_atomically_writes_png_and_json(tmp_path: Path) -> None:
    bag, route, result = _write_bound_trial(tmp_path)
    output = tmp_path / "evidence"
    args = argparse.Namespace(
        bag=bag,
        route_file=route,
        result=result,
        output_dir=output,
        profile_id="carla_vad_30kph_v2",
        target_speed_mps=8.333333333333334,
        longitudinal_speed_source="explicit_simulation_nominal",
        actuation_map_coverage=None,
        gated_acceleration_limit_mps2=1.5,
        accel_pedal_saturation=0.399,
        brake_pedal_saturation=0.599,
        pedal_activity_epsilon=1.0e-3,
    )

    status = analyzer.run(
        args, read_bag=lambda _bag: (_complete_records(), _topic_types())
    )

    assert status == 0
    payload = json.loads(
        (output / "longitudinal_response.json").read_text(encoding="utf-8")
    )
    assert payload["status"] == "complete"
    assert (output / "longitudinal_response.png").stat().st_size > 10_000
    assert not list(output.glob(".*.staged"))


def test_run_preserves_machine_readable_failure_and_plot(tmp_path: Path) -> None:
    bag, route, result = _write_bound_trial(tmp_path)
    output = tmp_path / "failed_evidence"
    args = argparse.Namespace(
        bag=bag,
        route_file=route,
        result=result,
        output_dir=output,
        profile_id="carla_vad_30kph_v2",
        target_speed_mps=8.333333333333334,
        longitudinal_speed_source="explicit_simulation_nominal",
        actuation_map_coverage=None,
    )

    status = analyzer.run(
        args,
        read_bag=lambda _bag: (_ for _ in ()).throw(RuntimeError("bag failed")),
    )

    assert status == 1
    payload = json.loads(
        (output / "longitudinal_response.json").read_text(encoding="utf-8")
    )
    assert payload["status"] == "incomplete"
    assert "bag failed" in payload["quality"]["problems"][0]
    assert (output / "longitudinal_response.png").stat().st_size > 10_000
    assert not list(output.glob(".*.staged"))
