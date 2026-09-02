from __future__ import annotations

from copy import deepcopy
import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).parents[1]
MODULE_PATH = ROOT / "scripts/e2e/compare_speed_trials.py"
SPEC = importlib.util.spec_from_file_location("compare_speed_trials", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
comparison = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(comparison)


def _write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


def _bound_identity(
    trial: Path, result: dict, route: dict, *, role: str, longitudinal: bool
) -> dict:
    contract = comparison.TRIAL_CONTRACTS[role]
    result_path = trial / "result.json"
    route_path = trial / "aligned_route.json"
    identity = {
        "schema_version": 1,
        "effective_route": {
            "path": str(route_path),
            "sha256": comparison._sha256(route_path),
            "town": route["town"],
            "scenario": route["scenario"],
            "route_length_m": route["route_length_m"],
        },
        "route_result": {
            "path": str(result_path),
            "sha256": comparison._sha256(result_path),
            "success": result["success"],
            "execution_mode": "full_stack",
            "reason": result["reason"],
            "profile_context": comparison.EXPECTED_PROFILE_CONTEXT,
        },
        "rosbag": comparison._bag_manifest(trial / "bag"),
    }
    if longitudinal:
        identity["profile"] = {
            "profile_id": contract["profile_id"],
            "target_speed_mps": contract["target_speed_mps"],
            "target_speed_kph": contract["target_speed_mps"] * 3.6,
            "longitudinal_speed_source": "explicit_simulation_nominal",
            "requested_target_is_converter_lookup_velocity": False,
            "converter_lookup_velocity_source": (
                "absolute_current_odometry_longitudinal_speed_mps"
            ),
        }
        identity["route_result"]["speed_exposure"] = {
            key: result["speed_exposure"][key]
            for key in (
                "status",
                "minimum_sustained_speed_mps",
                "maximum_observed_speed_limit_mps",
                "maximum_observed_speed_mps",
                "maximum_sustained_speed_duration_sec",
            )
        }
    else:
        identity["route_result"]["speed_exposure_status"] = result[
            "speed_exposure"
        ]["status"]
    identity["sha256"] = comparison._sha256_json(identity)
    return identity


def _write_bound_trial(root: Path, role: str, *, runtime: bool = False) -> Path:
    contract = comparison.TRIAL_CONTRACTS[role]
    trial = root / role
    bag = trial / "bag"
    bag.mkdir(parents=True)
    (bag / "metadata.yaml").write_text("version: 9\n", encoding="utf-8")
    (bag / "bag_0.db3").write_bytes(b"bound bag")
    route = {"town": "Town06", "scenario": "straight", "route_length_m": 445.0}
    _write_json(trial / "aligned_route.json", route)
    maximum = 7.74 if role == "reference" else 9.78
    result = {
        "execution_mode": "full_stack",
        "profile_context": comparison.EXPECTED_PROFILE_CONTEXT,
        "success": contract["result_success"],
        "reason": "goal reached" if role == "reference" else "speed exposure failed",
        "route_file": str(trial / "aligned_route.json"),
        "started_at": "2026-09-02T00:00:00+00:00",
        "finished_at": "2026-09-02T00:01:00+00:00",
        "final": {"goal_reached": True},
        "metrics": {
            "maximum_observed_speed_mps": maximum,
            "maximum_sustained_speed_duration_sec": (
                2.0 if role == "reference" else 0.0
            ),
            "maximum_absolute_cte_m": 0.5,
            "maximum_lateral_acceleration_mps2": 0.4,
            "traveled_distance_m": 444.0,
            "sim_elapsed_sec": 60.0,
            "wall_elapsed_sec": 120.0,
        },
        "speed_exposure": {
            **comparison.EXPECTED_PROFILE_CONTEXT,
            "status": contract["speed_exposure_status"],
            "minimum_sustained_speed_mps": contract["minimum_sustained_speed_mps"],
            "minimum_sustained_speed_sec": 1.0,
            "maximum_observed_speed_limit_mps": contract[
                "maximum_observed_speed_limit_mps"
            ],
            "maximum_observed_speed_mps": maximum,
            "maximum_sustained_speed_duration_sec": (
                2.0 if role == "reference" else 0.0
            ),
        },
    }
    _write_json(trial / "result.json", result)
    inputs = {
        "bag": str(bag),
        "profile_id": contract["profile_id"],
        "target_speed_mps": contract["target_speed_mps"],
        "target_speed_kph": contract["target_speed_mps"] * 3.6,
        "longitudinal_speed_source": "explicit_simulation_nominal",
    }
    speed = {
        "status": "complete",
        "quality": {"problems": []},
        "inputs": inputs,
        "source_identity": _bound_identity(
            trial, result, route, role=role, longitudinal=False
        ),
    }
    _write_json(trial / "speed_profile.json", speed)
    actuation = {
        "profile_id": contract["profile_id"],
        "target_speed_mps": contract["target_speed_mps"],
        "status": contract["actuation_status"],
        "runtime_lookup_observation": {
            "available": True,
            "classification": "OBSERVED_LOOKUPS_WITHIN_MAP_AXIS",
            "maximum_absolute_current_speed_mps": maximum,
            "velocity_axis_clamping_observed": False,
        },
    }
    _write_json(trial / "actuation_map_runtime_coverage.json", actuation)
    duty = {
        key: {
            "available": True,
            "time_fraction_percent": 1.0,
            "sample_fraction_percent": 1.0,
            "longest_contiguous_duration_sec": 1.0,
        }
        for key in (
            "raw_acceleration_above_gate_limit",
            "gated_positive_acceleration_limit",
            "accel_command_near_saturation",
            "brake_command_active",
        )
    }
    longitudinal = {
        "status": "complete",
        "quality": {"problems": []},
        "inputs": inputs,
        "source_identity": _bound_identity(
            trial, result, route, role=role, longitudinal=True
        ),
        "summary": {},
        "saturation_and_duty": duty,
        "actuation_map_coverage": {
            "source": {
                "path": str(trial / "actuation_map_runtime_coverage.json"),
                "sha256": comparison._sha256(
                    trial / "actuation_map_runtime_coverage.json"
                ),
            },
            "runtime_cross_check": {"consistent_with_bag": True},
        },
    }
    _write_json(trial / "longitudinal_response.json", longitudinal)
    rates = {
        f"/sensing/camera/{camera}/camera_info": {
            "stamp_period_sec": {"max": 0.2}
        }
        for camera in comparison.CAMERA_IDS
    }
    latency = {
        "inputs": {"bag": str(bag)},
        "camera_bundle": {
            "available": True,
            "bundle_coverage_percent": 100.0,
            "camera_count": 6,
            "front_frame_count": 10,
        },
        "candidate_front_acceptance": {
            "available": True,
            "acceptance_percent": 100.0,
            "candidate_count": 10,
            "front_count": 10,
        },
        "event_rates": rates,
    }
    _write_json(trial / "latency/e2e_latency.json", latency)
    mapping = trial / "sensor_mapping_provenance/sensor_mapping.yaml"
    mapping.parent.mkdir(parents=True)
    mapping.write_text("mapping: bound\n", encoding="utf-8")
    mapping_sha = comparison._sha256(mapping)
    (mapping.parent / "SHA256SUMS").write_text(
        f"{mapping_sha}  sensor_mapping.yaml\n", encoding="utf-8"
    )
    (trial / "runtime.env").write_text(
        "CAMERA_SOURCE_5HZ=true\n"
        "CAMERA_SOURCE_SENSOR_TICK_SEC=0.2\n"
        f"SENSOR_MAPPING_SHA256={mapping_sha}\n",
        encoding="utf-8",
    )
    (trial / "stack.log").write_text(
        "published_count=10 mailbox_taken=10 coalesced_drops=0\n"
        "VAD frame queued: source_stamp_ns=1 assembled=10 capacity_pruned=0 "
        "superseded=0 mailbox_submitted=10 coalesced_drops=0 "
        "received_images_min=10 received_images_max=10\n",
        encoding="utf-8",
    )
    camera = {
        "status": "PASS",
        "contract": comparison.CAMERA_CONTRACT,
        "mapping_sha256": mapping_sha,
        "bundle_coverage_percent": 100.0,
        "maximum_camera_stamp_gap_sec": 0.2,
        "camera_stamp_gap_sec": {camera: 0.2 for camera in comparison.CAMERA_IDS},
        "candidate_front_acceptance_percent": 100.0,
        "candidate_count": 10,
        "front_count": 10,
        "vad_inference": {
            "published_count": 10,
            "mailbox_taken": 10,
            "mailbox_submitted": 10,
            "capacity_pruned": 0,
            "superseded": 0,
            "coalesced_drops": 0,
        },
    }
    _write_json(trial / "camera_source_5hz_validation.json", camera)
    _write_json(
        trial / "desktop_capture.json",
        {
            "candidate_observed": True,
            "rviz_view_contract": {"vehicle_centered": True},
            "png_dimensions": [1920, 1080],
            "gif_dimensions": [960, 540],
        },
    )
    if runtime:
        bag_manifest = comparison._bag_manifest(bag)
        manifest = {
            key: comparison._file_record(path)
            for key, path in (
                ("result", trial / "result.json"),
                ("latency", trial / "latency/e2e_latency.json"),
                ("stack", trial / "stack.log"),
            )
        }
        manifest["bag"] = {
            "path": str(bag),
            "files": [
                {
                    "name": item["path"],
                    "size_bytes": item["size_bytes"],
                    "sha256": item["sha256"],
                }
                for item in bag_manifest["files"]
            ],
        }
        _write_json(
            trial / "runtime_load_analysis.json",
            {
                "schema_version": 2,
                "analysis": "CARLA/VAD pilot runtime-load phase reconstruction",
                "input_manifest": manifest,
                "trial": {
                    "started_at": result["started_at"],
                    "finished_at": result["finished_at"],
                    "success": result["success"],
                    "reason": result["reason"],
                    **result["metrics"],
                },
                "vad_runtime": {"runtime_pattern": "synthetic"},
                "findings": {"classification": "synthetic"},
            },
        )
    return trial


def _trial(role: str, maximum: float) -> dict:
    contract = comparison.TRIAL_CONTRACTS[role]
    label = contract["label"]
    target = contract["target_speed_mps"]
    success = contract["result_success"]
    return {
        "label": label,
        "trial_directory": f"/evidence/{label}",
        "profile_id": contract["profile_id"],
        "target_speed_mps": target,
        "target_speed_kph": target * 3.6,
        "trial_verdict": "PASS" if success else "FAIL",
        "reason": "goal reached" if success else "speed exposure failed",
        "goal_reached": True,
        "speed_exposure_status": "PASS" if success else "FAIL",
        "route": {
            "sha256": "a" * 64,
            "town": "Town06",
            "scenario": "straight",
            "route_length_m": 445.0,
        },
        "motion": {
            "maximum_observed_speed_mps": maximum,
            "maximum_observed_speed_kph": maximum * 3.6,
            "target_attainment_percent": maximum / target * 100.0,
            "maximum_sustained_speed_duration_sec": 1.0 if success else 0.0,
            "maximum_absolute_cte_m": 0.5,
            "maximum_lateral_acceleration_mps2": 0.4,
            "traveled_distance_m": 444.0,
            "sim_elapsed_sec": 70.0,
            "wall_elapsed_sec": 280.0 if success else 140.0,
            "real_time_factor": 0.25 if success else 0.5,
        },
        "longitudinal": {
            key: {"time_fraction_percent": value}
            for key, value in (
                ("gated_positive_acceleration_limit", 20.0),
                ("accel_command_near_saturation", 3.0),
                ("brake_command_active", 10.0),
            )
        },
        "actuation_map": {"velocity_axis_clamping_observed": False},
        "camera": {"status": "PASS"},
        "rviz": {"vehicle_centered": True},
    }


def test_build_comparison_preserves_failed_pilot_and_bounded_rtf_claim() -> None:
    report = comparison.build_comparison(
        _trial("reference", 7.74),
        _trial("pilot", 9.78),
    )

    assert report["status"] == "complete"
    assert report["same_route"]["verified"] is True
    assert report["comparison"]["pilot_speed_exposure_passed"] is False
    assert report["comparison"]["pilot_goal_reached_despite_speed_failure"] is True
    bounded = report["findings"]["target_speed_alone_does_not_explain_low_rtf"]
    assert bounded["supported"] is True
    assert report["real_vehicle_ready"] is False


def test_build_comparison_rejects_route_substitution() -> None:
    pilot = _trial("pilot", 9.78)
    pilot["route"]["sha256"] = "b" * 64

    with pytest.raises(comparison.ComparisonError, match="byte-identical"):
        comparison.build_comparison(
            _trial("reference", 7.74), pilot
        )


@pytest.mark.parametrize(
    ("role", "field", "value"),
    (
        ("reference", "profile_id", "some_other_30kph_profile"),
        ("reference", "speed_exposure_status", "FAIL"),
        ("pilot", "target_speed_mps", 15.0),
        ("pilot", "trial_verdict", "PASS"),
    ),
)
def test_build_comparison_rejects_identity_or_outcome_substitution(
    role: str, field: str, value: object
) -> None:
    reference = _trial("reference", 7.74)
    pilot = _trial("pilot", 9.78)
    selected = reference if role == "reference" else pilot
    selected[field] = value

    with pytest.raises(comparison.ComparisonError, match="strict identity/outcome|target"):
        comparison.build_comparison(reference, pilot)


def test_atomic_outputs_and_pilot_summary(tmp_path: Path) -> None:
    report = comparison.build_comparison(
        _trial("reference", 7.74),
        _trial("pilot", 9.78),
    )
    json_path = tmp_path / "same_route_30_vs_60.json"
    plot_path = tmp_path / "same_route_30_vs_60.png"
    comparison._atomic_json(json_path, report)
    comparison._atomic_plot(plot_path, report)

    loaded = json.loads(json_path.read_text(encoding="utf-8"))
    assert loaded["findings"]["classification"].startswith("60KPH")
    assert plot_path.stat().st_size > 20_000
    summary = comparison._pilot_summary(report, json_path)
    assert summary["status"] == "EVIDENCE_COMPLETE_TRIAL_FAILED_SPEED_EXPOSURE"
    assert summary["supersedes_for_interpretation"]["original_file_preserved"] is True

    contradictory = deepcopy(report)
    contradictory["comparison"]["pilot_speed_exposure_passed"] = True
    with pytest.raises(comparison.ComparisonError, match="contradicts"):
        comparison._pilot_summary(contradictory, json_path)

    stale = deepcopy(report)
    stale["generated_at"] = "stale"
    with pytest.raises(comparison.ComparisonError, match="does not match"):
        comparison._pilot_summary(stale, json_path)


def test_load_trial_accepts_fully_bound_evidence(tmp_path: Path) -> None:
    trial = _write_bound_trial(tmp_path, "reference")

    loaded = comparison._load_trial(trial, "reference")

    assert loaded["profile_id"] == "carla_vad_30kph_v2"
    assert loaded["rosbag_identity"]["sha256"]


def test_load_trial_rejects_stale_speed_result_binding(tmp_path: Path) -> None:
    trial = _write_bound_trial(tmp_path, "reference")
    path = trial / "speed_profile.json"
    speed = json.loads(path.read_text(encoding="utf-8"))
    speed["source_identity"]["route_result"]["sha256"] = "0" * 64
    identity = speed["source_identity"]
    identity["sha256"] = comparison._sha256_json(
        {key: value for key, value in identity.items() if key != "sha256"}
    )
    _write_json(path, speed)

    with pytest.raises(comparison.ComparisonError, match="route-result binding"):
        comparison._load_trial(trial, "reference")


def test_load_trial_rejects_mixed_camera_latency(tmp_path: Path) -> None:
    trial = _write_bound_trial(tmp_path, "reference")
    path = trial / "camera_source_5hz_validation.json"
    camera = json.loads(path.read_text(encoding="utf-8"))
    camera["candidate_count"] = 9
    _write_json(path, camera)

    with pytest.raises(comparison.ComparisonError, match="camera counts"):
        comparison._load_trial(trial, "reference")


def test_load_trial_rejects_stale_actuation_hash(tmp_path: Path) -> None:
    trial = _write_bound_trial(tmp_path, "reference")
    path = trial / "actuation_map_runtime_coverage.json"
    actuation = json.loads(path.read_text(encoding="utf-8"))
    actuation["new_unbound_field"] = True
    _write_json(path, actuation)

    with pytest.raises(comparison.ComparisonError, match="actuation audit"):
        comparison._load_trial(trial, "reference")


def test_load_trial_rejects_stale_runtime_result_hash(tmp_path: Path) -> None:
    trial = _write_bound_trial(tmp_path, "pilot", runtime=True)
    path = trial / "runtime_load_analysis.json"
    runtime = json.loads(path.read_text(encoding="utf-8"))
    runtime["input_manifest"]["result"]["sha256"] = "0" * 64
    _write_json(path, runtime)

    with pytest.raises(comparison.ComparisonError, match="runtime input digest"):
        comparison._load_trial(trial, "pilot")
