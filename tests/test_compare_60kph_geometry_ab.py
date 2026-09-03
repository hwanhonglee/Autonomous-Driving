from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).parents[1]
MODULE_PATH = ROOT / "scripts/e2e/compare_60kph_geometry_ab.py"
SPEC = importlib.util.spec_from_file_location("compare_60kph_geometry_ab", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
comparison = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(comparison)


def _write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def _write_sums(directory: Path, names: tuple[str, ...]) -> None:
    (directory / "SHA256SUMS").write_text(
        "".join(f"{comparison._sha256(directory / name)}  {name}\n" for name in names),
        encoding="utf-8",
    )


def _source_identity(
    trial: Path,
    result: dict,
    route: dict,
    bag: dict,
) -> dict:
    identity = {
        "schema_version": 1,
        "effective_route": {
            "path": str(trial / "aligned_route.json"),
            "sha256": comparison._sha256(trial / "aligned_route.json"),
            "town": route["town"],
            "scenario": route["scenario"],
            "route_length_m": route["route_length_m"],
        },
        "route_result": {
            "path": str(trial / "result.json"),
            "sha256": comparison._sha256(trial / "result.json"),
            "success": result["success"],
            "reason": result["reason"],
            "execution_mode": "full_stack",
            "profile_context": comparison.EXPECTED_PROFILE_CONTEXT,
            "speed_exposure_status": result["speed_exposure"]["status"],
        },
        "rosbag": bag,
    }
    identity["sha256"] = comparison._sha256_json(identity)
    return identity


def _fixture(
    tmp_path: Path,
    role: str,
    *,
    controller_text: str = "controller: fixed\n",
    extra_param: float | None = None,
    source_marker: str = "same-route",
    candidate_curvature_p95: float = 0.003,
    candidate_curvature_max: float = 0.004,
    candidate_planning_minimum_mps: float = 12.0,
    legacy_baseline: bool = False,
) -> Path:
    candidate = role == "candidate"
    trial = tmp_path / role / "trial/attempt_001"
    trial.mkdir(parents=True)
    source_route = {
        "town": "Town06",
        "scenario": "straight",
        "marker": source_marker,
        "route": [{"x": value, "y": 0.0} for value in range(0, 101, 10)],
    }
    _write_json(trial / "source_route.json", source_route)
    source_sha = comparison._sha256(trial / "source_route.json")
    route_points = [
        {
            "index": index,
            "x": float(value),
            "y": 0.0,
            "z": 0.0,
            "yaw": 0.0,
            "distance_m": float(value),
            "remaining_m": float(100 - value),
            "road_option": "LANEFOLLOW",
            "road_option_value": 4,
            "vad_command": 3,
            "road_id": 1,
            "section_id": 0,
            "lane_id": -1,
            "is_junction": False,
        }
        for index, value in enumerate(range(0, 101, 10))
    ]
    route = {
        "schema_version": 1,
        "coordinate_reference": "base_link",
        "town": "Town06",
        "scenario": "straight",
        "route_length_m": 100.0,
        "route": route_points,
        "physical_straight_preflight": {
            "status": "PASS",
            "route_length_m": 100.0,
            "maximum_chord_deviation_m": 0.0,
            "maximum_absolute_curvature_per_m": 0.0,
            "failure_reasons": [],
        },
        "coordinate_alignment": {
            "schema_version": 1,
            "source_frame": "carla_map",
            "target_frame": "map",
            "source_route": str(tmp_path / role / "catalog/route.json"),
            "source_route_sha256": source_sha,
            "map_bundle_profile": "town06",
            "carla_to_map_transform": {
                "x_m": 0.0,
                "y_m": 0.0,
                "z_m": 0.0,
                "yaw_rad": 0.0,
            },
        },
    }
    _write_json(trial / "aligned_route.json", route)
    _write_json(
        trial / "route_alignment.json",
        {
            "status": "PASS",
            "source_route_sha256": source_sha,
            "aligned_route_sha256": comparison._sha256(trial / "aligned_route.json"),
        },
    )

    actual_path = [
        {
            "sim_time_sec": float(10 + index),
            "x": float(index * 5),
            "y": 0.0,
            "speed_mps": min(14.5, float(index)),
        }
        for index in range(21)
    ]
    result = {
        "execution_mode": "full_stack",
        "profile_context": comparison.EXPECTED_PROFILE_CONTEXT,
        "route_file": str(trial / "aligned_route.json"),
        "success": False,
        "reason": "speed exposure contract failed",
        "actual_path": actual_path,
        "final": {"goal_reached": True},
        "limits": {
            "comfortable_deceleration_mps2": 2.0,
            "maximum_lateral_acceleration_mps2": 1.2,
        },
        "metrics": {
            "sim_elapsed_sec": 80.0,
            "wall_elapsed_sec": 80.2,
            "maximum_observed_speed_mps": 14.5 if candidate else 10.1,
            "maximum_sustained_speed_duration_sec": 0.0,
            "maximum_absolute_cte_m": 0.40 if candidate else 0.52,
            "maximum_lateral_acceleration_mps2": 0.35 if candidate else 0.44,
            "maximum_trajectory_correction_m": 4.55 if candidate else 4.60,
        },
        "speed_exposure": {
            "status": "FAIL",
            "minimum_sustained_speed_mps": 15.0,
            "minimum_sustained_speed_sec": 1.0,
            "maximum_observed_speed_limit_mps": 18.0,
        },
    }
    _write_json(trial / "result.json", result)

    bag_dir = trial / "bag"
    bag_dir.mkdir()
    (bag_dir / "metadata.yaml").write_text("version: 9\n", encoding="utf-8")
    (bag_dir / "bag_0.db3").write_bytes(b"immutable synthetic bag")
    bag = comparison._bag_manifest(bag_dir)
    planning_minimum = candidate_planning_minimum_mps if candidate else 6.0
    planning = [
        {
            "header_time_ns": int((11.0 + index) * 1.0e9),
            "time_sec": float(index),
            "speed_mps": 16.0,
            "trajectory_horizon_minimum_mps": planning_minimum,
            "trajectory_horizon_maximum_mps": 16.666666666666668,
            "trajectory_horizon_p95_mps": 16.666666666666668,
        }
        for index in range(4)
    ]
    speed = {
        "status": "complete",
        "quality": {"problems": [], "warnings": []},
        "inputs": {
            "bag": str(bag_dir),
            "profile_id": comparison.PROFILE_ID,
            "target_speed_mps": comparison.TARGET_SPEED_MPS,
            "target_speed_kph": 60.0,
            "longitudinal_speed_source": "explicit_simulation_nominal",
        },
        "series": {"explicit_overlaid_planning": planning},
        "source_identity": _source_identity(trial, result, route, bag),
    }
    _write_json(trial / "speed_profile.json", speed)
    _write_json(
        trial / "diagnosis.json",
        {
            "inputs": {
                "bag": str(bag_dir),
                "route_file": str(trial / "aligned_route.json"),
                "town": "Town06",
                "scenario": "straight",
            },
            "quality_warnings": [],
            "metrics": {
                "final_path": {
                    "snapshot_peak_curvature_per_m": {
                        "count": 10,
                        "p95_abs": candidate_curvature_p95 if candidate else 0.020,
                        "max_abs": candidate_curvature_max if candidate else 0.060,
                    }
                }
            },
        },
    )
    _write_json(
        trial / "latency/e2e_latency.json",
        {
            "inputs": {"bag": str(bag_dir)},
            "camera_bundle": {
                "bundle_coverage_percent": 100.0,
                "receipt_span_sec": {"p95": 0.008},
            },
        },
    )
    _write_json(
        trial / "camera_source_5hz_validation.json",
        {"status": "PASS", "bundle_coverage_percent": 100.0},
    )
    health = {
        "status": "PASS",
        "sequence": {"winning_window_indexes": [0, 1, 2]},
        "windows": [{"clock": {"rtf": 0.99}} for _ in range(3)],
    }
    _write_json(trial / "runtime_health.json", health)

    speed_provenance = trial / "speed_profile_provenance"
    speed_provenance.mkdir()
    speed_files = {
        "longitudinal_controller.param.yaml": controller_text,
        "longitudinal_controller.param.yaml.metadata.json": "{}\n",
        "vehicle_cmd_gate.param.yaml": "gate: fixed\n",
        "vehicle_cmd_gate.param.yaml.metadata.json": "{}\n",
    }
    for name, content in speed_files.items():
        (speed_provenance / name).write_text(content, encoding="utf-8")
    _write_sums(speed_provenance, tuple(speed_files))

    trajectory_provenance = trial / "trajectory_code_provenance"
    trajectory_provenance.mkdir()
    (trajectory_provenance / "vad_route_logic.py").write_text(
        "LOGIC = 'fixed'\n", encoding="utf-8"
    )
    (trajectory_provenance / "vad_route_manager.py").write_text(
        "MANAGER = 'fixed'\n", encoding="utf-8"
    )
    _write_sums(
        trajectory_provenance, ("vad_route_logic.py", "vad_route_manager.py")
    )

    actuation = trial / "actuation_config_provenance"
    actuation.mkdir()
    actuation_files = {
        "raw_vehicle_cmd_converter.param.yaml": "max_throttle: 0.4\n",
        "accel_map.csv": "0,0\n",
        "brake_map.csv": "0,0\n",
        "steer_map.csv": "0,0\n",
    }
    for name, content in actuation_files.items():
        (actuation / name).write_text(content, encoding="utf-8")
    _write_json(
        actuation / "manifest.json",
        {
            "schema_version": 1,
            "execution": {
                "uses_original_selected_config": True,
                "uses_artifact_copy": False,
            },
            "files": {
                "config": {
                    "artifact": "raw_vehicle_cmd_converter.param.yaml",
                    "sha256": comparison._sha256(
                        actuation / "raw_vehicle_cmd_converter.param.yaml"
                    ),
                },
                **{
                    key: {
                        "artifact": f"{key}.csv",
                        "sha256": comparison._sha256(actuation / f"{key}.csv"),
                    }
                    for key in ("accel_map", "brake_map", "steer_map")
                },
            },
        },
    )

    parameters = {
        "use_sim_time": True,
        "route_file": str(trial / "aligned_route.json"),
        "route_corridor_half_width_m": 0.2 if candidate else 0.5,
        "turn_outward_corridor_half_width_m": 0.2 if candidate else 0.5,
        "turn_inward_corridor_half_width_m": 0.2,
        "maximum_lateral_acceleration_mps2": (
            extra_param if extra_param is not None else 1.0
        ),
        "nominal_cruise_speed_mps": comparison.TARGET_SPEED_MPS,
    }
    (trial / "vad_route_manager.params.yaml").write_text(
        yaml.safe_dump({"/vad_route_manager": {"ros__parameters": parameters}}),
        encoding="utf-8",
    )

    environment = {**comparison.STRING_INVARIANTS}
    environment.update(
        {key: str(value) for key, value in comparison.NUMERIC_INVARIANTS.items()}
    )
    fixed_hash = "a" * 64
    environment.update({key: fixed_hash for key in comparison.EQUAL_HASH_ENV_KEYS})
    environment.update(
        {
            "LONGITUDINAL_CONTROLLER_PARAM_SHA256": comparison._sha256(
                speed_provenance / "longitudinal_controller.param.yaml"
            ),
            "LONGITUDINAL_CONTROLLER_METADATA_SHA256": comparison._sha256(
                speed_provenance / "longitudinal_controller.param.yaml.metadata.json"
            ),
            "VEHICLE_CMD_GATE_PARAM_SHA256": comparison._sha256(
                speed_provenance / "vehicle_cmd_gate.param.yaml"
            ),
            "VEHICLE_CMD_GATE_METADATA_SHA256": comparison._sha256(
                speed_provenance / "vehicle_cmd_gate.param.yaml.metadata.json"
            ),
            "TRAJECTORY_LOGIC_SHA256": comparison._sha256(
                trajectory_provenance / "vad_route_logic.py"
            ),
            "VAD_ROUTE_MANAGER_SHA256": comparison._sha256(
                trajectory_provenance / "vad_route_manager.py"
            ),
            "RUNTIME_HEALTH_GATE_ENABLED": "true",
            "RUNTIME_HEALTH_GATE_STATUS": "PASS",
            "RUNTIME_HEALTH_EVIDENCE_SHA256": comparison._sha256(
                trial / "runtime_health.json"
            ),
        }
    )
    if not (legacy_baseline and not candidate):
        environment.update(
            {
                "GEOMETRY_AB_CANDIDATE_ID": (
                    "route_corridor_0p2" if candidate else "baseline_corridor_0p5"
                ),
                "GEOMETRY_AB_ROUTE_CORRIDOR_0P2": "true" if candidate else "false",
                "GEOMETRY_AB_ROUTE_CORRIDOR_BASELINE_M": "0.50",
                "GEOMETRY_AB_ROUTE_CORRIDOR_CANDIDATE_M": "0.20",
                "GEOMETRY_AB_BEHAVIORAL_SINGLE_KNOB": "true",
                "GEOMETRY_AB_PARAMETER_CHANGE_COUNT": "2",
                "GEOMETRY_AB_COUPLED_PARAMETER_REASON": (
                    "turn_width_must_not_exceed_route_width"
                ),
                "GEOMETRY_AB_ROUTE_SCOPE": "straight_only",
                "ROUTE_CORRIDOR_HALF_WIDTH_M": "0.20" if candidate else "0.50",
                "TURN_OUTWARD_CORRIDOR_HALF_WIDTH_M": (
                    "0.20" if candidate else "0.50"
                ),
            }
        )
    (trial / "runtime.env").write_text(
        "".join(f"{key}={value}\n" for key, value in environment.items()),
        encoding="utf-8",
    )
    return tmp_path / role


def test_valid_geometry_pair_accepts_geometry_but_keeps_speed_separate(
    tmp_path: Path,
) -> None:
    baseline_root = _fixture(tmp_path, "baseline", legacy_baseline=True)
    candidate_root = _fixture(tmp_path, "candidate")

    baseline = comparison.load_trial(baseline_root, "baseline")
    candidate = comparison.load_trial(candidate_root, "candidate")
    payload = comparison.compare(baseline, candidate)

    assert baseline["route"]["aligned_route_file_sha256"] != candidate["route"][
        "aligned_route_file_sha256"
    ]
    assert baseline["route"]["canonical_geometry_identity_sha256"] == candidate[
        "route"
    ]["canonical_geometry_identity_sha256"]
    assert baseline["midroute_4_to_9mps_caps"]["cap_sample_count"] == 4
    assert candidate["midroute_4_to_9mps_caps"]["cap_sample_count"] == 0
    assert payload["geometry_outcome"]["decision"] == "ACCEPT"
    assert set(payload["candidate"]["evidence_sha256"]) >= {
        "source_route.json",
        "aligned_route.json",
        "route_alignment.json",
    }
    assert payload["geometry_outcome"]["checks"][
        "conditioned_curvature_maximum_supports_15mps"
    ]["status"] == "PASS"
    assert payload["speed_contract"]["decision"] == "FAIL"
    assert payload["real_vehicle_ready"] is False


def test_controller_change_is_rejected_as_a_confound(tmp_path: Path) -> None:
    baseline = comparison.load_trial(_fixture(tmp_path, "baseline"), "baseline")
    candidate = comparison.load_trial(
        _fixture(tmp_path, "candidate", controller_text="controller: changed\n"),
        "candidate",
    )

    with pytest.raises(comparison.ComparisonError, match="provenance environment changed"):
        comparison.compare(baseline, candidate)


def test_non_corridor_route_manager_change_is_rejected(tmp_path: Path) -> None:
    baseline = comparison.load_trial(_fixture(tmp_path, "baseline"), "baseline")
    candidate = comparison.load_trial(
        _fixture(tmp_path, "candidate", extra_param=0.9), "candidate"
    )

    with pytest.raises(comparison.ComparisonError, match="non-corridor"):
        comparison.compare(baseline, candidate)


def test_source_route_mismatch_is_rejected(tmp_path: Path) -> None:
    baseline = comparison.load_trial(_fixture(tmp_path, "baseline"), "baseline")
    candidate = comparison.load_trial(
        _fixture(tmp_path, "candidate", source_marker="different-route"), "candidate"
    )

    with pytest.raises(comparison.ComparisonError, match="effective-route identity"):
        comparison.compare(baseline, candidate)


def test_candidate_clipping_kink_forces_geometry_hold(tmp_path: Path) -> None:
    baseline = comparison.load_trial(_fixture(tmp_path, "baseline"), "baseline")
    candidate = comparison.load_trial(
        _fixture(tmp_path, "candidate", candidate_curvature_max=0.070), "candidate"
    )

    payload = comparison.compare(baseline, candidate)

    assert payload["geometry_outcome"]["decision"] == "HOLD"
    assert payload["geometry_outcome"]["checks"][
        "conditioned_curvature_maximum_no_kink_regression"
    ]["status"] == "FAIL"


def test_partial_geometry_improvement_is_reported_without_acceptance(
    tmp_path: Path,
) -> None:
    baseline = comparison.load_trial(_fixture(tmp_path, "baseline"), "baseline")
    candidate = comparison.load_trial(
        _fixture(
            tmp_path,
            "candidate",
            candidate_curvature_p95=0.015,
            candidate_curvature_max=0.020,
            candidate_planning_minimum_mps=6.0,
        ),
        "candidate",
    )

    payload = comparison.compare(baseline, candidate)

    assert payload["geometry_outcome"]["decision"] == "HOLD"
    assert (
        payload["geometry_outcome"]["diagnostic_effect"]
        == "PARTIAL_IMPROVEMENT_INSUFFICIENT"
    )


def test_cap_exposure_uses_full_eligible_sample_hold_timeline(tmp_path: Path) -> None:
    root = _fixture(tmp_path, "baseline")
    trial = root / "trial/attempt_001"
    speed = json.loads((trial / "speed_profile.json").read_text(encoding="utf-8"))
    speed["series"]["explicit_overlaid_planning"][1][
        "trajectory_horizon_minimum_mps"
    ] = 16.0
    for index, sample in enumerate(speed["series"]["explicit_overlaid_planning"]):
        sample["header_time_ns"] = int((11.0 + 0.2 * index) * 1.0e9)
    result = json.loads((trial / "result.json").read_text(encoding="utf-8"))
    result["actual_path"] = [
        {**sample, "sim_time_sec": 11.0 + 0.2 * index}
        for index, sample in enumerate(result["actual_path"][1:5])
    ]
    summary = comparison._midroute_caps(
        speed,
        result,
        json.loads((trial / "aligned_route.json").read_text(encoding="utf-8")),
    )

    assert summary["cap_sample_count"] == 3
    assert summary["estimated_exposure_duration_sec"] == pytest.approx(0.4)
    assert summary["longest_contiguous_exposure_sec"] == pytest.approx(0.2)


def test_rosbag_tampering_is_rejected(tmp_path: Path) -> None:
    candidate_root = _fixture(tmp_path, "candidate")
    (candidate_root / "trial/attempt_001/bag/bag_0.db3").write_bytes(b"tampered")

    with pytest.raises(comparison.ComparisonError, match="rosbag manifest"):
        comparison.load_trial(candidate_root, "candidate")


def test_markdown_and_png_show_both_independent_decisions(tmp_path: Path) -> None:
    baseline = comparison.load_trial(_fixture(tmp_path, "baseline"), "baseline")
    candidate = comparison.load_trial(_fixture(tmp_path, "candidate"), "candidate")
    payload = comparison.compare(baseline, candidate)

    markdown = comparison.render_markdown(payload)
    output = tmp_path / "comparison.png"
    comparison.render_png(payload, output)

    assert "Geometry decision: **ACCEPT**" in markdown
    assert "speed contract: **FAIL**" in markdown
    assert output.is_file() and output.stat().st_size > 1_000
