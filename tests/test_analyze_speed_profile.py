from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest


pytest.importorskip("rosbag2_py")

ROOT = Path(__file__).parents[1]
MODULE_PATH = ROOT / "scripts/e2e/analyze_speed_profile.py"
SPEC = importlib.util.spec_from_file_location("analyze_speed_profile", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
analyzer = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(analyzer)


def _trajectory(stamp_sec: float, speeds: list[float]) -> dict[str, object]:
    stamp_ns = int(round(stamp_sec * 1.0e9))
    return {
        "bag_ns": stamp_ns + 1_000,
        "stamp_ns": stamp_ns,
        "speed": np.asarray(speeds, dtype=float),
    }


def _scalar(stamp_sec: float, speed: float) -> dict[str, object]:
    stamp_ns = int(round(stamp_sec * 1.0e9))
    return {
        "bag_ns": stamp_ns + 1_000,
        "stamp_ns": stamp_ns,
        "speed": speed,
    }


def _complete_records() -> dict[str, list[dict[str, object]]]:
    return {
        analyzer.RAW_TOPIC: [
            _trajectory(1.0, [2.5, 4.0, 6.0]),
            _trajectory(2.0, [2.5, 5.0, 7.0]),
        ],
        analyzer.PLANNING_TOPIC: [
            _trajectory(1.0, [8.333, 8.333, 8.333]),
            _trajectory(2.0, [7.5, 6.0, 0.0]),
        ],
        analyzer.GATED_CONTROL_TOPIC: [
            _scalar(1.0, 0.0),
            _scalar(1.5, 4.0),
            _scalar(2.0, 7.5),
        ],
        analyzer.ODOMETRY_TOPIC: [
            _scalar(1.0, 0.0),
            _scalar(1.5, 2.0),
            _scalar(2.0, 5.0),
        ],
    }


def _topic_types() -> dict[str, str]:
    return {
        analyzer.RAW_TOPIC: "autoware_planning_msgs/msg/Trajectory",
        analyzer.PLANNING_TOPIC: "autoware_planning_msgs/msg/Trajectory",
        analyzer.GATED_CONTROL_TOPIC: "autoware_control_msgs/msg/Control",
        analyzer.ODOMETRY_TOPIC: "nav_msgs/msg/Odometry",
    }


def _build(records: dict[str, list[dict[str, object]]]) -> dict[str, object]:
    return analyzer.build_evidence(
        records,
        _topic_types(),
        bag=Path("/tmp/synthetic_speed_bag"),
        profile_id="carla_vad_30kph_v2",
        target_speed_mps=8.333333333333334,
        longitudinal_speed_source="explicit_simulation_nominal",
    )


def test_evidence_separates_raw_overlay_gate_and_actual_velocity() -> None:
    evidence = _build(_complete_records())

    assert evidence["status"] == "complete"
    assert evidence["interpretation"]["raw_vad_velocity_is_cruise_target"] is False
    assert evidence["interpretation"]["real_vehicle_ready"] is False
    assert evidence["alignment"]["primary_time"].startswith("ROS message/header stamp")
    assert evidence["alignment"]["common_interval_duration_sec"] == pytest.approx(1.0)

    raw = evidence["series"]["raw_selected_vad"]
    planning = evidence["series"]["explicit_overlaid_planning"]
    assert [sample["speed_mps"] for sample in raw] == [2.5, 2.5]
    assert raw[-1]["trajectory_horizon_maximum_mps"] == 7.0
    assert planning[0]["speed_mps"] == pytest.approx(8.333)
    assert evidence["summary"]["gated_control_command"]["maximum_mps"] == 7.5
    assert evidence["summary"]["actual_odometry"]["maximum_mps"] == 5.0
    assert evidence["sources"]["gated_control_command"]["velocity_field"] == (
        "longitudinal.velocity"
    )


def test_missing_required_series_is_machine_readable_and_fail_closed() -> None:
    records = _complete_records()
    records[analyzer.GATED_CONTROL_TOPIC] = []

    evidence = _build(records)

    assert evidence["status"] == "incomplete"
    assert any(
        analyzer.GATED_CONTROL_TOPIC in problem
        for problem in evidence["quality"]["problems"]
    )


def test_zero_header_stamp_is_disclosed_as_receipt_time_fallback() -> None:
    records = _complete_records()
    records[analyzer.GATED_CONTROL_TOPIC][0]["stamp_ns"] = 0
    records[analyzer.GATED_CONTROL_TOPIC][0]["bag_ns"] = 1_000_001_000

    evidence = _build(records)

    assert evidence["status"] == "complete"
    assert (
        evidence["summary"]["gated_control_command"]["bag_receipt_fallback_count"]
        == 1
    )
    assert analyzer.GATED_CONTROL_TOPIC in evidence["quality"]["warnings"][0]


def test_run_writes_png_and_json_even_when_evidence_is_incomplete(
    tmp_path: Path,
) -> None:
    records = _complete_records()
    records[analyzer.RAW_TOPIC] = []
    args = argparse.Namespace(
        bag=tmp_path / "bag",
        output_dir=tmp_path / "evidence",
        profile_id="carla_vad_30kph_v2",
        target_speed_mps=8.333333333333334,
        longitudinal_speed_source="explicit_simulation_nominal",
    )

    status = analyzer.run(args, read_bag=lambda _bag: (records, _topic_types()))

    assert status == 1
    payload = json.loads(
        (args.output_dir / "speed_profile.json").read_text(encoding="utf-8")
    )
    assert payload["status"] == "incomplete"
    assert (args.output_dir / "speed_profile.png").stat().st_size > 10_000


def test_recorded_trial_invokes_speed_analyzer_only_inside_speed_profile() -> None:
    source = (ROOT / "scripts/e2e/run_recorded_route_trial.sh").read_text(
        encoding="utf-8"
    )

    assert "scripts/e2e/analyze_speed_profile.py" in source
    speed_guard = (
        'if [[ "${speed_30kph}" == "true" || '
        '"${speed_60kph_pilot}" == "true" ]]; then'
    )
    analyzer_offset = source.index("scripts/e2e/analyze_speed_profile.py")
    guard_offset = source.rfind(speed_guard, 0, analyzer_offset)
    assert guard_offset >= 0
    assert "--profile-id \"${speed_profile_id}\"" in source[guard_offset:analyzer_offset + 500]
    assert "--target-speed-mps \"${target_speed_mps}\"" in source[
        guard_offset : analyzer_offset + 500
    ]
    assert "--route-file \"${route_file}\"" in source[
        guard_offset : analyzer_offset + 500
    ]
    assert "--result \"${output_dir}/result.json\"" in source[
        guard_offset : analyzer_offset + 500
    ]
    assert "speed_profile_analysis.log" in source[guard_offset:analyzer_offset + 500]


def test_evidence_uses_the_requested_target_instead_of_a_fixed_30kph_label() -> None:
    records = _complete_records()
    evidence = analyzer.build_evidence(
        records,
        _topic_types(),
        bag=Path("/tmp/synthetic_speed_bag_60"),
        profile_id="carla_vad_60kph_straight_pilot_v1",
        target_speed_mps=16.666666666666668,
        longitudinal_speed_source="explicit_simulation_nominal",
    )

    assert evidence["inputs"]["target_speed_kph"] == pytest.approx(60.0)
    assert "60 km/h cruise target" in evidence["interpretation"]["note"]


def test_source_identity_binds_route_result_and_complete_rosbag(
    tmp_path: Path,
) -> None:
    bag = tmp_path / "bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("version: 9\n", encoding="utf-8")
    (bag / "data.db3").write_bytes(b"bag-evidence")
    route = tmp_path / "aligned_route.json"
    route.write_text(
        json.dumps(
            {
                "town": "Town01",
                "scenario": "straight",
                "route_length_m": 210.0,
            }
        ),
        encoding="utf-8",
    )
    context = {
        "longitudinal_velocity_source": "explicit_simulation_nominal",
        "vad_velocity_evaluated": False,
        "vad_geometry_evaluated": True,
    }
    result = tmp_path / "result.json"
    result.write_text(
        json.dumps(
            {
                "success": True,
                "execution_mode": "full_stack",
                "route_file": str(route),
                "profile_context": context,
                "speed_exposure": {"status": "PASS", **context},
            }
        ),
        encoding="utf-8",
    )

    identity = analyzer._source_identity(bag, route, result)

    assert identity["effective_route"]["trial_id"] == "straight"
    assert identity["rosbag"]["root"] == str(bag.resolve())
    assert len(identity["rosbag"]["files"]) == 2
    assert identity["route_result"]["success"] is True
    assert identity["route_result"]["speed_exposure_status"] == "PASS"
    assert identity["sha256"] == analyzer._sha256_json(
        {key: value for key, value in identity.items() if key != "sha256"}
    )


def test_source_identity_preserves_a_failed_speed_exposure_for_analysis(
    tmp_path: Path,
) -> None:
    bag = tmp_path / "bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("version: 9\n", encoding="utf-8")
    route = tmp_path / "route.json"
    route.write_text(
        json.dumps(
            {
                "town": "Town06",
                "scenario": "straight",
                "route_length_m": 445.88,
            }
        ),
        encoding="utf-8",
    )
    context = {
        "longitudinal_velocity_source": "explicit_simulation_nominal",
        "vad_velocity_evaluated": False,
        "vad_geometry_evaluated": True,
    }
    result = tmp_path / "result.json"
    result.write_text(
        json.dumps(
            {
                "success": False,
                "reason": "speed exposure contract failed",
                "execution_mode": "full_stack",
                "route_file": str(route),
                "profile_context": context,
                "speed_exposure": {"status": "FAIL", **context},
            }
        ),
        encoding="utf-8",
    )

    identity = analyzer._source_identity(bag, route, result)

    assert identity["route_result"]["success"] is False
    assert identity["route_result"]["speed_exposure_status"] == "FAIL"
    assert identity["route_result"]["reason"] == "speed exposure contract failed"


def test_source_identity_rejects_result_bound_to_another_route(
    tmp_path: Path,
) -> None:
    bag = tmp_path / "bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("version: 9\n", encoding="utf-8")
    route = tmp_path / "route.json"
    other = tmp_path / "other.json"
    payload = {"town": "Town01", "scenario": "straight", "route_length_m": 210.0}
    route.write_text(json.dumps(payload), encoding="utf-8")
    other.write_text(json.dumps(payload), encoding="utf-8")
    context = {
        "longitudinal_velocity_source": "explicit_simulation_nominal",
        "vad_velocity_evaluated": False,
        "vad_geometry_evaluated": True,
    }
    result = tmp_path / "result.json"
    result.write_text(
        json.dumps(
            {
                "success": True,
                "execution_mode": "full_stack",
                "route_file": str(other),
                "profile_context": context,
                "speed_exposure": {"status": "PASS", **context},
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(RuntimeError, match="different effective route"):
        analyzer._source_identity(bag, route, result)
