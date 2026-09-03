from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).parents[1]
SCRIPT = ROOT / "scripts/e2e/validate_60kph_pilot_contract.py"
SPEC = importlib.util.spec_from_file_location("validate_60kph_pilot_contract", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
gate = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(gate)


def _write(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def _manifest(bag: Path) -> dict:
    files = [
        {
            "path": path.name,
            "size_bytes": path.stat().st_size,
            "sha256": gate._sha256_file(path),
        }
        for path in sorted(bag.iterdir())
    ]
    value = {"schema_version": 1, "root": str(bag), "files": files}
    value["sha256"] = gate._sha256_json(
        {"schema_version": 1, "files": files}
    )
    return value


def _identity(value: dict) -> dict:
    value.pop("sha256", None)
    value["sha256"] = gate._sha256_json(value)
    return value


def _checksum_manifest(root: Path, files: dict[str, bytes]) -> dict[str, str]:
    root.mkdir(parents=True, exist_ok=True)
    digests = {}
    for name, content in files.items():
        path = root / name
        path.write_bytes(content)
        digests[name] = gate._sha256_file(path)
    (root / "SHA256SUMS").write_text(
        "".join(f"{digest}  {name}\n" for name, digest in sorted(digests.items())),
        encoding="utf-8",
    )
    return digests


def _actuation_provenance(attempt: Path) -> dict:
    root = attempt / "actuation_config_provenance"
    root.mkdir(parents=True, exist_ok=True)
    artifacts = {
        "config": ("raw_vehicle_cmd_converter.param.yaml", b"config: true\n"),
        "accel_map": ("accel_map.csv", b"velocity,0.0,13.89\n0.0,0.0,1.0\n"),
        "brake_map": ("brake_map.csv", b"velocity,0.0,13.89\n0.0,0.0,-1.0\n"),
        "steer_map": ("steer_map.csv", b"steer,0.0\n0.0,0.0\n"),
    }
    records = {}
    for key, (name, content) in artifacts.items():
        path = root / name
        path.write_bytes(content)
        records[key] = {
            "artifact": name,
            "sha256": gate._sha256_file(path),
            "size_bytes": path.stat().st_size,
        }
    _write(
        root / "manifest.json",
        {
            "schema_version": 1,
            "execution": {
                "uses_original_selected_config": True,
                "uses_artifact_copy": False,
            },
            "files": records,
        },
    )
    return {
        "root": str(root),
        "manifest": {
            "path": str(root / "manifest.json"),
            "sha256": gate._sha256_file(root / "manifest.json"),
        },
        "maps": {
            key: {
                "artifact": records[key]["artifact"],
                "sha256": records[key]["sha256"],
            }
            for key in ("accel_map", "brake_map")
        },
    }


def _fixture(tmp_path: Path, *, accepted: bool = True) -> tuple[Path, Path]:
    source = tmp_path / "catalog/route.json"
    attempt = tmp_path / "trial/attempt_001"
    source_payload = {
        "town": "Town06",
        "scenario": "straight",
        "route_length_m": 445.0,
        "physical_straight_preflight": {"status": "PASS"},
    }
    _write(source, source_payload)
    _write(attempt / "source_route.json", source_payload)
    aligned_payload = {
        **source_payload,
        "start_ros_pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
        "goal_ros_pose": {"x": 445.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
    }
    _write(attempt / "aligned_route.json", aligned_payload)
    source_sha = gate._sha256_file(source)
    aligned_sha = gate._sha256_file(attempt / "aligned_route.json")
    _write(
        attempt / "route_alignment.json",
        {
            "status": "PASS",
            "source_route": str(source),
            "source_route_sha256": source_sha,
            "aligned_route": str(attempt / "aligned_route.json"),
            "aligned_route_sha256": aligned_sha,
        },
    )

    duration = 1.2 if accepted else 0.0
    maximum_speed = 15.5 if accepted else 10.1
    exposure_status = "PASS" if accepted else "FAIL"
    result = {
        "schema_version": 1,
        "execution_mode": "full_stack",
        "route_file": str(attempt / "aligned_route.json"),
        "profile_context": dict(gate.PROFILE_CONTEXT),
        "success": accepted,
        "reason": "goal reached" if accepted else "speed exposure contract failed",
        "limits": {
            "minimum_sustained_speed_mps": 15.0,
            "minimum_sustained_speed_sec": 1.0,
            "maximum_observed_speed_mps": 18.0,
            "maximum_lateral_acceleration_mps2": 1.2,
            "maximum_speed_sample_gap_sec": 0.25,
        },
        "metrics": {
            "maximum_observed_speed_mps": maximum_speed,
            "maximum_sustained_speed_duration_sec": duration,
            "maximum_lateral_acceleration_mps2": 0.4,
            "maximum_speed_sample_gap_sec": 0.05,
            "sim_elapsed_sec": 80.0,
            "wall_elapsed_sec": 80.2,
        },
        "assessment": {
            "route_completion": "PASS" if accepted else "FAIL",
        },
        "final": {"goal_reached": True, "route_status": "goal_reached"},
        "speed_exposure": {
            **gate.PROFILE_CONTEXT,
            "status": exposure_status,
            "minimum_sustained_speed_mps": 15.0,
            "minimum_sustained_speed_sec": 1.0,
            "maximum_observed_speed_limit_mps": 18.0,
            "maximum_lateral_acceleration_limit_mps2": 1.2,
            "maximum_observed_speed_mps": maximum_speed,
            "maximum_sustained_speed_duration_sec": duration,
            "maximum_lateral_acceleration_mps2": 0.4,
            "maximum_speed_sample_gap_sec": 0.05,
        },
    }
    _write(attempt / "result.json", result)
    result_sha = gate._sha256_file(attempt / "result.json")

    bag = attempt / "bag"
    bag.mkdir(parents=True)
    (bag / "bag_0.db3").write_bytes(b"sqlite-evidence")
    (bag / "metadata.yaml").write_text("storage: sqlite3\n", encoding="utf-8")
    bag_manifest = _manifest(bag)
    common_route = {
        "path": str(attempt / "aligned_route.json"),
        "sha256": aligned_sha,
        "town": "Town06",
        "scenario": "straight",
        "trial_id": "straight",
        "route_length_m": 445.0,
    }
    common_result = {
        "path": str(attempt / "result.json"),
        "sha256": result_sha,
        "success": accepted,
        "execution_mode": "full_stack",
    }
    speed_identity = _identity(
        {
            "schema_version": 1,
            "effective_route": dict(common_route),
            "route_result": {
                **common_result,
                "profile_context": dict(gate.PROFILE_CONTEXT),
                "speed_exposure_status": exposure_status,
                "reason": result["reason"],
            },
            "rosbag": bag_manifest,
        }
    )
    _write(
        attempt / "speed_profile.json",
        {
            "schema_version": 1,
            "analysis": "carla_speed_source_evidence",
            "status": "complete",
            "inputs": {
                "profile_id": gate.PROFILE_ID,
                "target_speed_mps": gate.TARGET_SPEED_MPS,
                "longitudinal_speed_source": "explicit_simulation_nominal",
            },
            "quality": {"problems": []},
            "source_identity": speed_identity,
        },
    )
    longitudinal_identity = _identity(
        {
            "schema_version": 1,
            "profile": {
                "profile_id": gate.PROFILE_ID,
                "target_speed_mps": gate.TARGET_SPEED_MPS,
            },
            "effective_route": dict(common_route),
            "route_result": {
                **common_result,
                "profile_context": dict(gate.PROFILE_CONTEXT),
                "reason": result["reason"],
                "speed_exposure": {
                    "status": exposure_status,
                    "minimum_sustained_speed_mps": 15.0,
                    "minimum_sustained_speed_sec": 1.0,
                    "maximum_observed_speed_limit_mps": 18.0,
                    "maximum_observed_speed_mps": maximum_speed,
                    "maximum_sustained_speed_duration_sec": duration,
                    "continuity_maximum_gap_sec": 0.25,
                },
            },
            "rosbag": bag_manifest,
        }
    )
    _write(
        attempt / "longitudinal_response.json",
        {
            "schema_version": 1,
            "analysis": "carla_longitudinal_response",
            "status": "complete",
            "inputs": {
                "profile_id": gate.PROFILE_ID,
                "target_speed_mps": gate.TARGET_SPEED_MPS,
                "longitudinal_speed_source": "explicit_simulation_nominal",
            },
            "quality": {"problems": []},
            "source_identity": longitudinal_identity,
            "target_exposure": {
                "route_result_cross_check": {
                    "maximum_speed_consistent_with_bag": True,
                    "duration_consistent_within_one_continuity_gap": True,
                    "minimum_duration_condition_met_from_bag": accepted,
                }
            },
            "summary": {
                "gated_acceleration_mps2": {"maximum": 1.5},
            },
            "series": {
                "raw_control": [
                    {"jerk": 0.0},
                    {"jerk": 0.0},
                ],
                "gated_control": [
                    {"jerk": 0.0},
                    {"jerk": 0.0},
                ],
            },
        },
    )

    transport = {
        "profile_id": gate.CAMERA_PROFILE_ID,
        "sensor_mapping_sha256": "a" * 64,
        "vad_model_override_sha256": "b" * 64,
        "cyclonedds_config_sha256": "c" * 64,
        "rmw_implementation": "rmw_cyclonedds_cpp",
    }
    window = {
        "index": 0,
        "status": "PASS",
        "clock": {"rtf": 0.99},
        "minimum_observed_camera_wall_rate_hz": 5.0,
        "bundles": {
            "coverage_percent": 100.0,
            "receipt_span_seconds": {"p95": 0.006},
        },
    }
    health = {
        "schema_version": 1,
        "probe_id": "pre_engagement_runtime_health_v1",
        "status": "PASS",
        "source": {"sha256": "d" * 64},
        "contract": {
            "thresholds": {
                "maximum_bundle_receipt_p95_seconds": 0.04,
                "minimum_bundle_coverage_percent": 99.0,
                "minimum_camera_wall_rate_hz": 4.0,
                "minimum_complete_bundle_count": 20,
                "minimum_rtf": 0.9,
            },
            "camera_transport": dict(transport),
        },
        "sequence": {
            "status": "PASS",
            "timed_out": False,
            "maximum_consecutive_passes": 3,
            "winning_window_indexes": [0, 1, 2],
        },
        "camera_image_graph": {"status": "PASS"},
        "runtime": {"transport_environment": {"status": "PASS"}},
        "windows": [
            dict(window, index=0),
            dict(window, index=1),
            dict(window, index=2),
        ],
    }
    _write(attempt / "runtime_health.json", health)
    _write(
        attempt / "camera_source_5hz_validation.json",
        {
            "schema_version": 1,
            "status": "PASS",
            "contract": {
                "profile_id": gate.CAMERA_PROFILE_ID,
                "sensor_count": 6,
                "source_frequency_hz": 5.0,
                "camera_image_publish_qos": "best_effort",
                "camera_image_publish_depth": 1,
                "real_vehicle_ready": False,
            },
            "transport_provenance": dict(transport),
            "bundle_coverage_percent": 100.0,
            "maximum_camera_stamp_gap_sec": 0.2,
            "candidate_front_acceptance_percent": 100.0,
            "raw_six_image_queue_integrity": {"status": "PASS"},
        },
    )
    _write(
        attempt / "runtime_load_analysis.json",
        {
            "schema_version": 3,
            "status": "complete",
            "problems": [],
            "vad_runtime": {
                "phases": {
                    "full_run": {"aggregate_rtf": 0.99, "wall_output_rate_hz": 5.0}
                }
            },
            "camera_delivery": {
                "phases": {"full_run": {"receipt_span_ms": {"p95": 6.0}}}
            },
        },
    )
    _write(
        attempt / "diagnosis.json",
        {
            "schema_version": 2,
            "inputs": {"town": "Town06", "scenario": "straight"},
            "metrics": {
                "final_path": {
                    "snapshot_peak_curvature_per_m": {
                        "p95_abs": 0.004,
                        "max_abs": 0.008,
                    }
                }
            },
        },
    )
    actuation_provenance = _actuation_provenance(attempt)
    coverage = {
        "schema_version": 1,
        "analysis": "raw_vehicle_command_converter_velocity_coverage",
        "status": "EXPLORATORY",
        "profile_id": gate.PROFILE_ID,
        "target_speed_mps": gate.TARGET_SPEED_MPS,
        "target_speed_kph": gate.TARGET_SPEED_MPS * 3.6,
        "map_velocity_axis_minimum_mps": 0.0,
        "map_velocity_axis_minimum_kph": 0.0,
        "map_velocity_axis_maximum_mps": 13.89,
        "map_velocity_axis_maximum_kph": 13.89 * 3.6,
        "target_excess_mps": gate.TARGET_SPEED_MPS - 13.89,
        "target_shortfall_mps": 0.0,
        "target_within_map_velocity_axis": False,
        "target_envelope_classification": (
            "TARGET_ENVELOPE_EXCEEDS_MAP_AXIS_CLAMP_IF_REACHED"
        ),
        "target_envelope_extension_authorized_for_exploratory_simulation": True,
        "runtime_lookup_observation": {
            "available": False,
            "classification": "PREFLIGHT_ONLY_NO_OBSERVED_SPEED",
            "velocity_axis_clamping_observed": None,
        },
        "validation_boundary": {
            "simulation_only": True,
            "map_coverage_above_axis": False,
            "real_vehicle_ready": False,
            "route_result_pass_is_high_speed_actuation_calibration": False,
            "target_speed_is_converter_lookup_velocity": False,
            "converter_lookup_velocity_source": (
                "absolute_current_odometry_longitudinal_speed_mps"
            ),
        },
        "provenance": actuation_provenance,
    }
    _write(attempt / "actuation_map_coverage.json", coverage)
    _write(
        attempt / "actuation_map_runtime_coverage.json",
        {
            **coverage,
            "runtime_lookup_observation": {
                "available": True,
                "maximum_absolute_current_speed_mps": maximum_speed,
                "maximum_absolute_current_speed_kph": maximum_speed * 3.6,
                "within_map_velocity_axis": maximum_speed <= 13.89,
                "velocity_axis_clamping_observed": maximum_speed > 13.89,
                "classification": (
                    "OBSERVED_LOOKUPS_WITHIN_MAP_AXIS"
                    if maximum_speed <= 13.89
                    else "OBSERVED_SPEED_REACHED_CLAMPED_MAP_REGION"
                ),
            },
        },
    )
    generation_id = "town06_60kph_straight_attempt_001"
    owner_pid = 12345
    owner_pgid = 12345
    server_log_path = attempt / "carla_server.log"
    server_log_path.write_text("CARLA owned server\n", encoding="utf-8")
    server_log = {
        "path": str(server_log_path),
        "sha256": gate._sha256_file(server_log_path),
        "size_bytes": server_log_path.stat().st_size,
    }
    for name, stage, mode in (
        ("carla_preflight_health.json", "trial_preflight", "running"),
        ("carla_completion_health.json", "trial_completion", "running"),
        ("carla_cleanup_health.json", "trial_cleanup", "stopped"),
    ):
        _write(
            attempt / name,
            {
                "schema_version": 1,
                "stage": stage,
                "status": "PASS",
                "mode": mode,
                "expected_map": "Town06",
                "active_map_basename": "Town06" if mode == "running" else None,
                "active_map_name": (
                    "Carla/Maps/Town06" if mode == "running" else None
                ),
                "generation_id": generation_id,
                "owner_pid": owner_pid,
                "owner_pgid": owner_pgid,
                "owner_process_state": "S" if mode == "running" else None,
                "port": 2100,
                "read_only": True,
                "port_released": True if mode == "stopped" else None,
                "server_log": server_log,
                "error": None,
            },
        )
    trajectory_digests = _checksum_manifest(
        attempt / "trajectory_code_provenance",
        {
            "vad_route_logic.py": b"# route logic\n",
            "vad_route_manager.py": b"# route manager\n",
        },
    )
    (attempt / "vad_route_manager.params.yaml").write_text(
        "/vad_route_manager:\n"
        "  ros__parameters:\n"
        "    route_corridor_half_width_m: 0.5\n"
        "    turn_outward_corridor_half_width_m: 0.5\n"
        f"    route_file: {attempt / 'aligned_route.json'}\n",
        encoding="utf-8",
    )
    env = {
        "CARLA_HOST": "127.0.0.1",
        "CARLA_PORT": "2100",
        "CARLA_LIFECYCLE": "cold_start_owned_process_group_per_trial",
        "CARLA_GENERATION_ID": generation_id,
        "CARLA_EXPECTED_MAP": "Town06",
        "CARLA_OWNER_PID": str(owner_pid),
        "CARLA_OWNER_PGID": str(owner_pgid),
        "CARLA_SERVER_LOG": str(server_log_path),
        "CARLA_MATRIX_OWNED": "true",
        "SPEED_60KPH_PILOT": "true",
        "SPEED_PROFILE_ID": gate.PROFILE_ID,
        "ROUTE_SCENARIO": "straight",
        "SPEED_EXPOSURE_MODE": "straight_target_required",
        "TARGET_SPEED_MPS": str(gate.TARGET_SPEED_MPS),
        "MINIMUM_SUSTAINED_SPEED_MPS": "15.0",
        "MINIMUM_SUSTAINED_SPEED_SEC": "1.0",
        "MAXIMUM_OBSERVED_SPEED_MPS": "18.0",
        "MAXIMUM_LATERAL_ACCELERATION_LIMIT_MPS2": "1.2",
        "MAXIMUM_SPEED_SAMPLE_GAP_SEC": "0.25",
        "RUNTIME_HEALTH_GATE_ENABLED": "true",
        "RUNTIME_HEALTH_GATE_STATUS": "PASS",
        "RUNTIME_HEALTH_REQUIRED_CONSECUTIVE_PASSES": "3",
        "RUNTIME_HEALTH_EVIDENCE_SHA256": gate._sha256_file(
            attempt / "runtime_health.json"
        ),
        "RUNTIME_HEALTH_PROBE_SHA256": "d" * 64,
        "CAMERA_SOURCE_5HZ": "true",
        "CAMERA_TRANSPORT_PROFILE_ID": gate.CAMERA_PROFILE_ID,
        "REAL_VEHICLE_READY": "false",
        "SIMULATION_ONLY_EXPLORATORY": "true",
        "ROUTE_SCOPE": "straight_only",
        "GEOMETRY_AB_CANDIDATE_ID": "baseline_corridor_0p5",
        "GEOMETRY_AB_ROUTE_CORRIDOR_0P2": "false",
        "GEOMETRY_AB_ROUTE_CORRIDOR_BASELINE_M": "0.50",
        "GEOMETRY_AB_ROUTE_CORRIDOR_CANDIDATE_M": "0.20",
        "GEOMETRY_AB_BEHAVIORAL_SINGLE_KNOB": "true",
        "GEOMETRY_AB_PARAMETER_CHANGE_COUNT": "2",
        "GEOMETRY_AB_COUPLED_PARAMETER_REASON": (
            "turn_width_must_not_exceed_route_width"
        ),
        "GEOMETRY_AB_ROUTE_SCOPE": "straight_only",
        "ROUTE_CORRIDOR_HALF_WIDTH_M": "0.50",
        "TURN_OUTWARD_CORRIDOR_HALF_WIDTH_M": "0.50",
        "TRAJECTORY_LOGIC_SHA256": trajectory_digests["vad_route_logic.py"],
        "VAD_ROUTE_MANAGER_SHA256": trajectory_digests["vad_route_manager.py"],
        "ACTUATION_MAP_COVERAGE_STATUS": "EXPLORATORY",
        "ACTUATION_MAP_TARGET_ENVELOPE_CLASSIFICATION": (
            "TARGET_ENVELOPE_EXCEEDS_MAP_AXIS_CLAMP_IF_REACHED"
        ),
        "ACTUATION_MAP_VELOCITY_AXIS_MAXIMUM_MPS": "13.89",
        "ACTUATION_TARGET_WITHIN_MAP_VELOCITY_AXIS": "false",
        "SOURCE_ROUTE_FILE": str(source),
        "EFFECTIVE_ROUTE_FILE": str(attempt / "aligned_route.json"),
    }
    (attempt / "runtime.env").write_text(
        "".join(f"{key}={value}\n" for key, value in env.items()),
        encoding="utf-8",
    )
    return attempt, source


def _rebind_result_analysis(attempt: Path) -> None:
    result_path = attempt / "result.json"
    result = json.loads(result_path.read_text(encoding="utf-8"))
    result_sha = gate._sha256_file(result_path)
    for name in ("speed_profile.json", "longitudinal_response.json"):
        path = attempt / name
        document = json.loads(path.read_text(encoding="utf-8"))
        identity = document["source_identity"]
        route_result = identity["route_result"]
        route_result["sha256"] = result_sha
        route_result["success"] = result["success"]
        route_result["reason"] = result["reason"]
        route_result["profile_context"] = result["profile_context"]
        if name == "speed_profile.json":
            route_result["speed_exposure_status"] = result["speed_exposure"][
                "status"
            ]
        else:
            route_result["speed_exposure"] = {
                "status": result["speed_exposure"]["status"],
                "minimum_sustained_speed_mps": result["speed_exposure"][
                    "minimum_sustained_speed_mps"
                ],
                "minimum_sustained_speed_sec": result["speed_exposure"][
                    "minimum_sustained_speed_sec"
                ],
                "maximum_observed_speed_limit_mps": result["speed_exposure"][
                    "maximum_observed_speed_limit_mps"
                ],
                "maximum_observed_speed_mps": result["speed_exposure"][
                    "maximum_observed_speed_mps"
                ],
                "maximum_sustained_speed_duration_sec": result["speed_exposure"][
                    "maximum_sustained_speed_duration_sec"
                ],
                "continuity_maximum_gap_sec": 0.25,
            }
        _identity(identity)
        _write(path, document)


def test_complete_evidence_and_speed_exposure_pass_simulation_gate(
    tmp_path: Path,
) -> None:
    attempt, source = _fixture(tmp_path)

    result = gate.evaluate_trial(attempt, source)

    assert result["evidence_integrity_status"] == "PASS"
    assert result["simulation_pilot_acceptance_status"] == "PASS"
    assert result["real_vehicle_readiness_status"] == "BLOCKED"
    assert result["real_vehicle_ready"] is False
    assert result["integrity_failures"] == []
    assert result["acceptance_failures"] == []
    assert result["physical_goal_completion_status"] == "PASS"
    assert result["speed_exposure_contract_status"] == "PASS"
    assert result["full_stack_route_test_status"] == "PASS"
    geometry = result["geometry_variant"]
    assert geometry["provenance_status"] == "PASS"
    assert geometry["candidate_id"] == "baseline_corridor_0p5"
    assert geometry["route_corridor_0p2"] is False
    assert geometry["route_corridor_half_width_m"] == 0.5
    assert geometry["turn_outward_corridor_half_width_m"] == 0.5
    assert geometry["parameter_dump_sha256"] == gate._sha256_file(
        attempt / "vad_route_manager.params.yaml"
    )
    assert geometry["trajectory_checksum_manifest_sha256"] == gate._sha256_file(
        attempt / "trajectory_code_provenance/SHA256SUMS"
    )
    lifecycle = result["sources"]["carla_lifecycle"]
    assert lifecycle["preflight"]["sha256"] == gate._sha256_file(
        attempt / "carla_preflight_health.json"
    )
    assert lifecycle["completion"]["sha256"] == gate._sha256_file(
        attempt / "carla_completion_health.json"
    )
    assert lifecycle["cleanup"]["sha256"] == gate._sha256_file(
        attempt / "carla_cleanup_health.json"
    )
    assert any("jerk" in item for item in result["readiness_blockers"])
    assert any("velocity axis" in item for item in result["readiness_blockers"])


def test_goal_completion_without_speed_exposure_is_consistent_but_rejected(
    tmp_path: Path,
) -> None:
    attempt, source = _fixture(tmp_path, accepted=False)

    result = gate.evaluate_trial(attempt, source)

    assert result["evidence_integrity_status"] == "PASS"
    assert result["simulation_pilot_acceptance_status"] == "FAILED"
    assert result["physical_goal_completion_status"] == "PASS"
    assert result["speed_exposure_contract_status"] == "FAILED"
    assert result["full_stack_route_test_status"] == "FAILED"
    assert result["summary"]["goal_reached"] is True
    assert result["summary"]["route_status"] == "goal_reached"
    assert result["summary"]["speed_exposure_status"] == "FAIL"
    assert "60 kph speed exposure contract status is FAIL" in result[
        "acceptance_failures"
    ]
    assert all(
        "route result success" not in item.lower()
        and "route completion assessment" not in item.lower()
        for item in result["acceptance_failures"]
    )
    assert any("15 m/s" in item for item in result["acceptance_failures"])
    assert any("rosbag" in item for item in result["acceptance_failures"])


def test_goal_and_speed_pass_preserve_non_speed_route_failure(
    tmp_path: Path,
) -> None:
    attempt, source = _fixture(tmp_path)
    result_path = attempt / "result.json"
    result = json.loads(result_path.read_text(encoding="utf-8"))
    result["success"] = False
    result["reason"] = "goal reached, but disengage failed"
    result["assessment"]["route_completion"] = "FAIL"
    _write(result_path, result)
    _rebind_result_analysis(attempt)

    evaluated = gate.evaluate_trial(attempt, source)

    assert evaluated["evidence_integrity_status"] == "PASS"
    assert evaluated["physical_goal_completion_status"] == "PASS"
    assert evaluated["speed_exposure_contract_status"] == "PASS"
    assert evaluated["full_stack_route_test_status"] == "FAILED"
    assert evaluated["acceptance_failures"] == [
        "full-stack route-test verdict failed for a non-speed reason"
    ]


def test_speed_failure_reason_must_name_speed_exposure(tmp_path: Path) -> None:
    attempt, source = _fixture(tmp_path, accepted=False)
    result_path = attempt / "result.json"
    result = json.loads(result_path.read_text(encoding="utf-8"))
    result["reason"] = "goal reached but target condition failed"
    _write(result_path, result)
    _rebind_result_analysis(attempt)

    evaluated = gate.evaluate_trial(attempt, source)

    assert evaluated["evidence_integrity_status"] == "FAILED"
    assert any(
        "failure reason does not identify the speed-exposure failure" in item
        for item in evaluated["integrity_failures"]
    )


def test_legacy_geometry_absence_remains_explicit_and_compatible(
    tmp_path: Path,
) -> None:
    attempt, source = _fixture(tmp_path)
    env_path = attempt / "runtime.env"
    legacy_lines = [
        line
        for line in env_path.read_text(encoding="utf-8").splitlines()
        if not line.startswith("GEOMETRY_AB_")
        and not line.startswith("ROUTE_CORRIDOR_HALF_WIDTH_M=")
        and not line.startswith("TURN_OUTWARD_CORRIDOR_HALF_WIDTH_M=")
    ]
    env_path.write_text("\n".join(legacy_lines) + "\n", encoding="utf-8")

    result = gate.evaluate_trial(attempt, source)

    assert result["evidence_integrity_status"] == "PASS"
    assert result["simulation_pilot_acceptance_status"] == "PASS"
    assert result["geometry_variant"]["provenance_status"] == "LEGACY_NOT_RECORDED"


def test_route_corridor_0p2_geometry_matches_runtime_parameter_dump(
    tmp_path: Path,
) -> None:
    attempt, source = _fixture(tmp_path)
    env_path = attempt / "runtime.env"
    environment = env_path.read_text(encoding="utf-8")
    environment = environment.replace(
        "GEOMETRY_AB_CANDIDATE_ID=baseline_corridor_0p5",
        "GEOMETRY_AB_CANDIDATE_ID=route_corridor_0p2",
    ).replace(
        "GEOMETRY_AB_ROUTE_CORRIDOR_0P2=false",
        "GEOMETRY_AB_ROUTE_CORRIDOR_0P2=true",
    ).replace(
        "ROUTE_CORRIDOR_HALF_WIDTH_M=0.50",
        "ROUTE_CORRIDOR_HALF_WIDTH_M=0.20",
    ).replace(
        "TURN_OUTWARD_CORRIDOR_HALF_WIDTH_M=0.50",
        "TURN_OUTWARD_CORRIDOR_HALF_WIDTH_M=0.20",
    )
    env_path.write_text(environment, encoding="utf-8")
    parameter_path = attempt / "vad_route_manager.params.yaml"
    parameter_path.write_text(
        parameter_path.read_text(encoding="utf-8").replace(
            "corridor_half_width_m: 0.5", "corridor_half_width_m: 0.2"
        ),
        encoding="utf-8",
    )

    result = gate.evaluate_trial(attempt, source)

    assert result["evidence_integrity_status"] == "PASS"
    assert result["geometry_variant"]["provenance_status"] == "PASS"
    assert result["geometry_variant"]["candidate_id"] == "route_corridor_0p2"
    assert result["geometry_variant"]["route_corridor_half_width_m"] == 0.2


def test_geometry_baseline_metadata_tamper_fails_provenance(tmp_path: Path) -> None:
    attempt, source = _fixture(tmp_path)
    env_path = attempt / "runtime.env"
    env_path.write_text(
        env_path.read_text(encoding="utf-8").replace(
            "GEOMETRY_AB_ROUTE_CORRIDOR_BASELINE_M=0.50",
            "GEOMETRY_AB_ROUTE_CORRIDOR_BASELINE_M=0.60",
        ),
        encoding="utf-8",
    )

    result = gate.evaluate_trial(attempt, source)

    assert result["evidence_integrity_status"] == "FAILED"
    assert result["geometry_variant"]["provenance_status"] == "FAILED"
    assert any("geometry A/B provenance" in item for item in result["integrity_failures"])


def test_cleanup_port_release_tamper_fails_lifecycle_integrity(
    tmp_path: Path,
) -> None:
    attempt, source = _fixture(tmp_path)
    path = attempt / "carla_cleanup_health.json"
    cleanup = json.loads(path.read_text(encoding="utf-8"))
    cleanup["port_released"] = False
    _write(path, cleanup)

    result = gate.evaluate_trial(attempt, source)

    assert result["evidence_integrity_status"] == "FAILED"
    assert result["simulation_pilot_acceptance_status"] == "FAILED"
    assert any("RPC port release" in item for item in result["integrity_failures"])


def test_lifecycle_generation_owner_or_missing_preflight_fails_closed(
    tmp_path: Path,
) -> None:
    attempt, source = _fixture(tmp_path)
    completion_path = attempt / "carla_completion_health.json"
    completion = json.loads(completion_path.read_text(encoding="utf-8"))
    completion["owner_pgid"] += 1
    _write(completion_path, completion)
    (attempt / "carla_preflight_health.json").unlink()

    result = gate.evaluate_trial(attempt, source)

    assert result["evidence_integrity_status"] == "FAILED"
    assert any("cannot read carla_preflight_health.json" in item for item in result["integrity_failures"])
    assert any("CARLA completion lifecycle contract mismatch" in item for item in result["integrity_failures"])


def test_carla_server_log_digest_tamper_fails_lifecycle_integrity(
    tmp_path: Path,
) -> None:
    attempt, source = _fixture(tmp_path)
    (attempt / "carla_server.log").write_text(
        "mutated owned server log\n", encoding="utf-8"
    )

    result = gate.evaluate_trial(attempt, source)

    assert result["evidence_integrity_status"] == "FAILED"
    assert any("server-log digest/size mismatch" in item for item in result["integrity_failures"])


def test_geometry_parameter_dump_drift_fails_strict_provenance(
    tmp_path: Path,
) -> None:
    attempt, source = _fixture(tmp_path)
    path = attempt / "vad_route_manager.params.yaml"
    path.write_text(
        path.read_text(encoding="utf-8").replace(
            "route_corridor_half_width_m: 0.5",
            "route_corridor_half_width_m: 0.2",
        ),
        encoding="utf-8",
    )

    result = gate.evaluate_trial(attempt, source)

    assert result["evidence_integrity_status"] == "FAILED"
    assert result["geometry_variant"]["provenance_status"] == "FAILED"
    assert any("runtime geometry" in item for item in result["integrity_failures"])


def test_trajectory_code_digest_drift_fails_strict_provenance(
    tmp_path: Path,
) -> None:
    attempt, source = _fixture(tmp_path)
    (attempt / "trajectory_code_provenance/vad_route_logic.py").write_text(
        "# changed after capture\n", encoding="utf-8"
    )

    result = gate.evaluate_trial(attempt, source)

    assert result["evidence_integrity_status"] == "FAILED"
    assert result["geometry_variant"]["provenance_status"] == "FAILED"
    assert any("checksum mismatch" in item for item in result["integrity_failures"])


def test_actuation_map_digest_drift_fails_provenance(tmp_path: Path) -> None:
    attempt, source = _fixture(tmp_path)
    (attempt / "actuation_config_provenance/accel_map.csv").write_text(
        "changed\n", encoding="utf-8"
    )

    result = gate.evaluate_trial(attempt, source)

    assert result["evidence_integrity_status"] == "FAILED"
    assert any(
        "actuation artifact accel_map digest/size mismatch" in item
        for item in result["integrity_failures"]
    )


def test_actuation_manifest_execution_contract_fails_closed(tmp_path: Path) -> None:
    attempt, source = _fixture(tmp_path)
    manifest_path = attempt / "actuation_config_provenance/manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["execution"]["uses_artifact_copy"] = True
    _write(manifest_path, manifest)

    result = gate.evaluate_trial(attempt, source)

    assert result["evidence_integrity_status"] == "FAILED"
    assert any(
        "actuation configuration manifest contract mismatch" in item
        for item in result["integrity_failures"]
    )


def test_result_tamper_after_analysis_fails_provenance_integrity(
    tmp_path: Path,
) -> None:
    attempt, source = _fixture(tmp_path)
    result_path = attempt / "result.json"
    result = json.loads(result_path.read_text(encoding="utf-8"))
    result["reason"] = "tampered"
    _write(result_path, result)

    evaluated = gate.evaluate_trial(attempt, source)

    assert evaluated["evidence_integrity_status"] == "FAILED"
    assert evaluated["simulation_pilot_acceptance_status"] == "FAILED"
    assert any(
        "route/result provenance mismatch" in item
        for item in evaluated["integrity_failures"]
    )


def test_runtime_health_digest_tamper_is_rejected(tmp_path: Path) -> None:
    attempt, source = _fixture(tmp_path)
    env_path = attempt / "runtime.env"
    env_path.write_text(
        env_path.read_text(encoding="utf-8").replace(
            "RUNTIME_HEALTH_EVIDENCE_SHA256="
            + gate._sha256_file(attempt / "runtime_health.json"),
            "RUNTIME_HEALTH_EVIDENCE_SHA256=" + "0" * 64,
        ),
        encoding="utf-8",
    )

    result = gate.evaluate_trial(attempt, source)

    assert result["evidence_integrity_status"] == "FAILED"
    assert any("runtime-health JSON digest" in item for item in result["integrity_failures"])


def test_full_run_runtime_regression_fails_acceptance_not_integrity(
    tmp_path: Path,
) -> None:
    attempt, source = _fixture(tmp_path)
    path = attempt / "runtime_load_analysis.json"
    runtime = json.loads(path.read_text(encoding="utf-8"))
    runtime["vad_runtime"]["phases"]["full_run"]["aggregate_rtf"] = 0.75
    _write(path, runtime)

    result = gate.evaluate_trial(attempt, source)

    assert result["evidence_integrity_status"] == "PASS"
    assert result["simulation_pilot_acceptance_status"] == "FAILED"
    assert "full-run VAD RTF is below 0.9" in result["acceptance_failures"]


def test_cli_preserves_machine_readable_failed_gate(tmp_path: Path) -> None:
    attempt, source = _fixture(tmp_path, accepted=False)
    output = tmp_path / "pilot_acceptance_gate.json"

    status = gate.run(
        argparse.Namespace(
            attempt_dir=attempt,
            source_route=source,
            output=output,
        )
    )

    payload = json.loads(output.read_text(encoding="utf-8"))
    assert status == 1
    assert payload["evidence_integrity_status"] == "PASS"
    assert payload["simulation_pilot_acceptance_status"] == "FAILED"
