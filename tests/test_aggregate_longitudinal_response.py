from __future__ import annotations

import argparse
import csv
import importlib.util
import json
from pathlib import Path
from typing import Any

from PIL import Image
import pytest


ROOT = Path(__file__).parents[1]
MODULE_PATH = ROOT / "scripts/e2e/aggregate_longitudinal_response.py"
SPEC = importlib.util.spec_from_file_location(
    "aggregate_longitudinal_response", MODULE_PATH
)
assert SPEC is not None and SPEC.loader is not None
analyzer = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(analyzer)


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def _summary(maximum: float) -> dict[str, Any]:
    return {
        "count": 20,
        "minimum": 0.0,
        "mean": maximum / 2.0,
        "median": maximum / 2.0,
        "p95": maximum * 0.95,
        "maximum": maximum,
    }


def _duty(sample_percent: float, longest: float) -> dict[str, Any]:
    return {
        "available": True,
        "sample_fraction_percent": sample_percent,
        "time_fraction_percent": sample_percent - 0.1,
        "longest_contiguous_duration_sec": longest,
    }


def _tracking(mean: float, rmse: float) -> dict[str, Any]:
    return {
        "available": True,
        "count": 20,
        "minimum": -0.1,
        "mean": mean,
        "median": mean,
        "p95": mean + 0.2,
        "maximum": mean + 0.3,
        "rmse": rmse,
        "p95_absolute": rmse + 0.2,
        "maximum_absolute": rmse + 0.4,
    }


def _attempt(
    root: Path,
    trial_id: str,
    *,
    maximum_speed: float,
    gate_duty: float,
    clamped: bool = False,
) -> Path:
    attempt = root / "maps/town01/trials" / trial_id / "attempt_001"
    attempt.mkdir(parents=True)
    route_scenario = "straight" if trial_id == "straight" else "left"
    route_path = attempt / "aligned_route.json"
    result_path = attempt / "result.json"
    _write_json(
        route_path,
        {
            "town": "Town01",
            "scenario": route_scenario,
            "route_length_m": 200.0 if trial_id == "straight" else 65.0,
        },
    )
    _write_json(result_path, {"success": True, "trial_id": trial_id})
    bag = attempt / "bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("version: 9\n", encoding="utf-8")
    (bag / "bag_0.db3").write_bytes(b"bag")
    Image.new("RGB", (2400, 1950), "white").save(
        attempt / "longitudinal_response.png"
    )

    duration_required = 1.0 if trial_id == "straight" else 0.0
    bag_files = [
        {
            "path": path.relative_to(bag).as_posix(),
            "size_bytes": path.stat().st_size,
            "sha256": analyzer._sha256_file(path),
        }
        for path in sorted(bag.rglob("*"))
        if path.is_file()
    ]
    bag_identity = {
        "schema_version": 1,
        "root": str(bag.resolve()),
        "files": bag_files,
    }
    bag_identity["sha256"] = analyzer._sha256_json(
        {"schema_version": 1, "files": bag_files}
    )
    source = {
        "schema_version": 1,
        "profile": {
            "profile_id": "carla_vad_30kph_v2",
            "target_speed_mps": 8.333333333333334,
            "longitudinal_speed_source": "explicit_simulation_nominal",
            "requested_target_is_converter_lookup_velocity": False,
            "converter_lookup_velocity_source": analyzer.EXPECTED_LOOKUP_SOURCE,
        },
        "effective_route": {
            "path": str(route_path.resolve()),
            "sha256": analyzer._sha256_file(route_path),
            "town": "Town01",
            "scenario": route_scenario,
            "trial_id": trial_id,
            "route_length_m": 200.0 if trial_id == "straight" else 65.0,
        },
        "route_result": {
            "path": str(result_path.resolve()),
            "sha256": analyzer._sha256_file(result_path),
            "success": True,
            "speed_exposure": {
                "status": "PASS",
                "minimum_sustained_speed_mps": (
                    7.5 if trial_id == "straight" else 0.0
                ),
                "minimum_sustained_speed_sec": duration_required,
                "maximum_observed_speed_limit_mps": 9.0,
                "maximum_observed_speed_mps": maximum_speed,
                "maximum_sustained_speed_duration_sec": (
                    4.0 if trial_id == "straight" else 0.0
                ),
                "continuity_maximum_gap_sec": 0.25,
            },
        },
        "rosbag": bag_identity,
    }
    source["sha256"] = analyzer._sha256_json(source)
    response = {
        "schema_version": 1,
        "analysis": analyzer.INDIVIDUAL_ANALYSIS_ID,
        "status": "complete",
        "inputs": {
            "profile_id": "carla_vad_30kph_v2",
            "target_speed_mps": 8.333333333333334,
            "target_speed_kph": 30.0,
            "longitudinal_speed_source": "explicit_simulation_nominal",
        },
        "source_identity": source,
        "quality": {"problems": [], "warnings": []},
        "interpretation": {
            "requested_target_is_converter_lookup_velocity": False,
            "converter_lookup_velocity_source": analyzer.EXPECTED_LOOKUP_SOURCE,
        },
        "summary": {
            "actual_speed_mps": _summary(maximum_speed),
            "robust_measured_acceleration_mps2": {
                **_summary(1.2),
                "minimum": -2.1,
            },
        },
        "robust_measured_acceleration": {
            "outlier_count": 3,
            "outlier_sample_percent": 0.3,
        },
        "target_exposure": {
            "route_result_reported": source["route_result"]["speed_exposure"],
            "route_result_cross_check": {
                "maximum_speed_consistent_with_bag": True,
            },
        },
        "saturation_and_duty": {
            "gated_positive_acceleration_limit": _duty(gate_duty, 3.0),
            "raw_acceleration_above_gate_limit": _duty(gate_duty - 1.0, 2.8),
            "accel_command_near_saturation": _duty(0.5, 0.1),
            "brake_command_active": _duty(12.0, 1.5),
        },
        "tracking": {
            "gated_target_minus_actual_speed_mps": _tracking(1.1, 1.5),
            "accel_command_minus_status": _tracking(0.001, 0.002),
            "brake_command_minus_status": _tracking(0.001, 0.003),
        },
        "actuation_map_coverage": {
            "provided": True,
            "status": "PASS",
            "target_within_map_velocity_axis": True,
            "map_velocity_axis_maximum_mps": 13.89,
            "runtime_lookup_observation": {
                "classification": (
                    "OBSERVED_SPEED_REACHED_CLAMPED_MAP_REGION"
                    if clamped
                    else "OBSERVED_LOOKUPS_WITHIN_MAP_AXIS"
                ),
                "velocity_axis_clamping_observed": clamped,
            },
            "runtime_cross_check": {"consistent_with_bag": True},
        },
    }
    _write_json(attempt / "longitudinal_response.json", response)
    return attempt


def _matrix_root(tmp_path: Path, *, clamped: bool = False) -> Path:
    root = tmp_path / "matrix"
    straight = _attempt(
        root,
        "straight",
        maximum_speed=7.7,
        gate_duty=31.0,
        clamped=clamped,
    )
    turn = _attempt(
        root,
        "turn",
        maximum_speed=5.1,
        gate_duty=22.0,
    )
    _write_json(
        root / "aggregate.json",
        {
            "schema_version": 1,
            "matrix_id": "synthetic_matrix",
            "status": "COMPLETE",
            "runtime_profile_selector": "speed_30kph_camera_source_5hz",
            "runnable_pass_count": 1,
            "runtime_profile": {
                "speed_contract": {
                    "profile_id": "carla_vad_30kph_v2",
                    "target_speed_mps": 8.333333333333334,
                    "route_manager_parameters": {
                        "longitudinal_velocity_source": (
                            "explicit_simulation_nominal"
                        )
                    },
                }
            },
            "maps": [
                {
                    "map_id": "town01",
                    "canonical_name": "Town01",
                    "status": "PASS",
                    "trials": {
                        "straight": {
                            "status": "PASS",
                            "attempt_directory": str(straight.resolve()),
                        },
                        "turn": {
                            "status": "PASS",
                            "attempt_directory": str(turn.resolve()),
                        },
                    },
                }
            ],
        },
    )
    return root


def test_build_aggregate_binds_selected_straight_and_turn_attempts(
    tmp_path: Path,
) -> None:
    root = _matrix_root(tmp_path)

    evidence = analyzer.build_aggregate(root, expected_trial_count=2)

    assert evidence["status"] == "COMPLETE"
    assert evidence["quality"]["observed_trial_count"] == 2
    assert evidence["quality"]["problem_count"] == 0
    assert [row["trial_id"] for row in evidence["trials"]] == [
        "straight",
        "turn",
    ]
    assert evidence["summary_by_trial"]["straight"]["metrics"][
        "actual_maximum_speed_mps"
    ]["maximum"] == pytest.approx(7.7)
    assert evidence["extremes"]["lowest_actual_maximum_speed_mps"] == {
        "map_id": "town01",
        "trial_id": "turn",
        "value": 5.1,
    }
    assert evidence["profile"][
        "requested_target_is_converter_lookup_velocity"
    ] is False


def test_csv_is_concise_and_has_one_row_per_selected_trial(
    tmp_path: Path,
) -> None:
    evidence = analyzer.build_aggregate(
        _matrix_root(tmp_path), expected_trial_count=2
    )

    rows = list(csv.DictReader(analyzer._csv_text(evidence).splitlines()))

    assert len(rows) == 2
    assert set(rows[0]) == set(analyzer.CSV_COLUMNS)
    assert {row["trial_id"] for row in rows} == {"straight", "turn"}
    assert "series" not in rows[0]


def test_expected_trial_count_is_fail_closed(tmp_path: Path) -> None:
    root = _matrix_root(tmp_path)

    with pytest.raises(analyzer.AggregateError, match="expected 18.*found 2"):
        analyzer.build_aggregate(root, expected_trial_count=18)


def test_only_attempts_selected_by_matrix_aggregate_are_admitted(
    tmp_path: Path,
) -> None:
    root = _matrix_root(tmp_path)
    unselected = root / "maps/town01/trials/straight/attempt_999"
    unselected.mkdir(parents=True)
    _write_json(
        unselected / "longitudinal_response.json",
        {"status": "complete", "unexpected": True},
    )

    evidence = analyzer.build_aggregate(root, expected_trial_count=2)

    assert evidence["quality"]["observed_trial_count"] == 2
    assert all(
        "attempt_999" not in row["attempt_directory"]
        for row in evidence["trials"]
    )


def test_selected_attempt_missing_analysis_is_rejected(tmp_path: Path) -> None:
    root = _matrix_root(tmp_path)
    response = (
        root / "maps/town01/trials/straight/attempt_001/longitudinal_response.json"
    )
    response.unlink()

    with pytest.raises(analyzer.AggregateError, match="longitudinal response"):
        analyzer.build_aggregate(root, expected_trial_count=2)


def test_runtime_map_clamping_is_rejected(tmp_path: Path) -> None:
    root = _matrix_root(tmp_path, clamped=True)

    with pytest.raises(analyzer.AggregateError, match="map coverage is not PASS"):
        analyzer.build_aggregate(root, expected_trial_count=2)


def test_source_identity_tampering_is_rejected(tmp_path: Path) -> None:
    root = _matrix_root(tmp_path)
    response_path = (
        root
        / "maps/town01/trials/straight/attempt_001/longitudinal_response.json"
    )
    response = json.loads(response_path.read_text(encoding="utf-8"))
    response["source_identity"]["effective_route"]["route_length_m"] = 999.0
    _write_json(response_path, response)

    with pytest.raises(analyzer.AggregateError, match="source identity SHA"):
        analyzer.build_aggregate(root, expected_trial_count=2)


def test_attempt_local_route_binding_is_enforced_after_identity_rehash(
    tmp_path: Path,
) -> None:
    root = _matrix_root(tmp_path)
    response_path = (
        root
        / "maps/town01/trials/straight/attempt_001/longitudinal_response.json"
    )
    response = json.loads(response_path.read_text(encoding="utf-8"))
    other_route = root / "other_route.json"
    _write_json(
        other_route,
        {"town": "Town01", "scenario": "straight", "route_length_m": 200.0},
    )
    source = response["source_identity"]
    source["effective_route"]["path"] = str(other_route.resolve())
    source["effective_route"]["sha256"] = analyzer._sha256_file(other_route)
    source["sha256"] = analyzer._sha256_json(
        {key: value for key, value in source.items() if key != "sha256"}
    )
    _write_json(response_path, response)

    with pytest.raises(analyzer.AggregateError, match="not attempt-local"):
        analyzer.build_aggregate(root, expected_trial_count=2)


def test_rosbag_file_corruption_is_rejected(tmp_path: Path) -> None:
    root = _matrix_root(tmp_path)
    bag_file = (
        root / "maps/town01/trials/straight/attempt_001/bag/bag_0.db3"
    )
    bag_file.write_bytes(b"BAD")

    with pytest.raises(analyzer.AggregateError, match="rosbag file SHA-256 changed"):
        analyzer.build_aggregate(root, expected_trial_count=2)


def test_rehashed_rosbag_file_record_replacement_is_rejected(
    tmp_path: Path,
) -> None:
    root = _matrix_root(tmp_path)
    response_path = (
        root
        / "maps/town01/trials/straight/attempt_001/longitudinal_response.json"
    )
    response = json.loads(response_path.read_text(encoding="utf-8"))
    source = response["source_identity"]
    bag = source["rosbag"]
    bag["files"][0]["sha256"] = "0" * 64
    bag["sha256"] = analyzer._sha256_json(
        {"schema_version": bag["schema_version"], "files": bag["files"]}
    )
    source["sha256"] = analyzer._sha256_json(
        {key: value for key, value in source.items() if key != "sha256"}
    )
    _write_json(response_path, response)

    with pytest.raises(analyzer.AggregateError, match="rosbag file SHA-256 changed"):
        analyzer.build_aggregate(root, expected_trial_count=2)


def test_rehashed_rosbag_manifest_sha_replacement_is_rejected(
    tmp_path: Path,
) -> None:
    root = _matrix_root(tmp_path)
    response_path = (
        root / "maps/town01/trials/turn/attempt_001/longitudinal_response.json"
    )
    response = json.loads(response_path.read_text(encoding="utf-8"))
    source = response["source_identity"]
    source["rosbag"]["sha256"] = "0" * 64
    source["sha256"] = analyzer._sha256_json(
        {key: value for key, value in source.items() if key != "sha256"}
    )
    _write_json(response_path, response)

    with pytest.raises(analyzer.AggregateError, match="rosbag manifest SHA-256"):
        analyzer.build_aggregate(root, expected_trial_count=2)


def test_run_atomically_writes_json_csv_and_png(tmp_path: Path) -> None:
    root = _matrix_root(tmp_path)
    output = tmp_path / "output"
    args = argparse.Namespace(
        matrix_root=root,
        output_dir=output,
        expected_trial_count=2,
    )

    status = analyzer.run(args)

    assert status == 0
    payload = json.loads(
        (output / "matrix_longitudinal_response.json").read_text(
            encoding="utf-8"
        )
    )
    assert payload["status"] == "COMPLETE"
    assert len(
        list(
            csv.DictReader(
                (output / "matrix_longitudinal_response.csv")
                .read_text(encoding="utf-8")
                .splitlines()
            )
        )
    ) == 2
    assert b"\r\n" not in (
        output / "matrix_longitudinal_response.csv"
    ).read_bytes()
    with Image.open(output / "matrix_longitudinal_response.png") as image:
        assert image.size == (2700, 1800)
    assert not list(output.glob(".*.staged"))


def test_run_preserves_atomic_error_outputs_when_selection_is_incomplete(
    tmp_path: Path,
) -> None:
    root = _matrix_root(tmp_path)
    output = tmp_path / "failed_output"
    args = argparse.Namespace(
        matrix_root=root,
        output_dir=output,
        expected_trial_count=18,
    )

    status = analyzer.run(args)

    assert status == 1
    payload = json.loads(
        (output / "matrix_longitudinal_response.json").read_text(
            encoding="utf-8"
        )
    )
    assert payload["status"] == "ERROR"
    assert "expected 18" in payload["quality"]["problems"][0]
    assert (output / "matrix_longitudinal_response.csv").read_text(
        encoding="utf-8"
    ).startswith("map_id,")
    with Image.open(output / "matrix_longitudinal_response.png") as image:
        assert image.size == (2100, 1050)
    assert not list(output.glob(".*.staged"))
