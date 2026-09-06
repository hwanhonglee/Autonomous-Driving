from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).parents[1]
REAL_STRICT_V1_ATTEMPT = (
    ROOT
    / "artifacts/validation/2026-09-07"
    / "autoware_vad_60kph_town06_strict10_transport_v1"
    / "trial/attempt_001"
)
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


def _edge_bounded_integrity(
    retained_bundle_count: int = 100,
    *,
    leading_missing: tuple[str, ...] | None = None,
    trailing_missing: tuple[str, ...] | None = None,
) -> dict:
    """HH_260906 - Build internally consistent strict-v2 recorder-edge evidence."""
    topics = gate.STRICT_CAMERA_INFO_TOPICS
    retained_minimum = 200_000_000
    retained_maximum = retained_minimum + (retained_bundle_count - 1) * 100_000_000
    edge_specs = (
        ("leading", leading_missing, 100_000_000),
        ("trailing", trailing_missing, retained_maximum + 100_000_000),
    )
    descriptors = {}
    trimmed_stamps = []
    trimmed_topic_counts = {topic: 0 for topic in topics}
    for edge, missing_topics, source_stamp_ns in edge_specs:
        if missing_topics is None:
            descriptors[edge] = None
            continue
        per_topic = {
            topic: int(topic not in missing_topics)
            for topic in topics
        }
        present_topics = [topic for topic in topics if per_topic[topic] == 1]
        descriptors[edge] = {
            "source_stamp_ns": source_stamp_ns,
            "record_count": sum(per_topic.values()),
            "per_topic_record_counts": per_topic,
            "present_topics": present_topics,
            "missing_topics": list(missing_topics),
            "duplicate_topics": [],
            "incomplete": True,
            "exact_six_camera_one_to_one": False,
        }
        trimmed_stamps.append(source_stamp_ns)
        for topic in present_topics:
            trimmed_topic_counts[topic] += 1

    topic_reports = {
        topic: {
            "record_count": retained_bundle_count + trimmed_topic_counts[topic],
            "positive_stamp_count": (
                retained_bundle_count + trimmed_topic_counts[topic]
            ),
            "unique_positive_stamp_count": (
                retained_bundle_count + trimmed_topic_counts[topic]
            ),
            "zero_or_negative_stamp_count": 0,
            "duplicate_positive_stamp_count": 0,
            "non_increasing_positive_stamp_delta_count": 0,
            "strictly_increasing_unique_positive_stamps": True,
            "trimmed_boundary_record_count": trimmed_topic_counts[topic],
            "retained_interior_record_count": retained_bundle_count,
        }
        for topic in topics
    }
    trimmed_positive_count = sum(trimmed_topic_counts.values())
    retained_positive_count = retained_bundle_count * len(topics)
    return {
        "schema_version": 1,
        "qualification_id": "camera_source_stamp_edge_bounded_whole_bag_v1",
        "status": "PASS",
        "camera_count": len(topics),
        "expected_topics": list(topics),
        "raw_record_count": retained_positive_count + trimmed_positive_count,
        "positive_record_count": retained_positive_count + trimmed_positive_count,
        "nonpositive_record_count": 0,
        "duplicate_positive_record_count": 0,
        "non_increasing_positive_stamp_delta_count": 0,
        "union_positive_source_stamp_count": (
            retained_bundle_count + len(trimmed_stamps)
        ),
        "boundary_trim": {
            "policy": "at_most_one_incomplete_union_stamp_per_bag_edge_v1",
            "maximum_incomplete_union_stamp_count_per_edge": 1,
            "leading_incomplete_union_stamp_count": int(
                leading_missing is not None
            ),
            "trailing_incomplete_union_stamp_count": int(
                trailing_missing is not None
            ),
            "trimmed_union_stamp_count": len(trimmed_stamps),
            "trimmed_positive_record_count": trimmed_positive_count,
            "trimmed_source_stamps_ns": trimmed_stamps,
            **descriptors,
        },
        "retained_interior": {
            "source_stamp_count": retained_bundle_count,
            "source_stamp_range_ns": {
                "minimum": retained_minimum,
                "maximum": retained_maximum,
            },
            "positive_record_count": retained_positive_count,
            "expected_record_count": retained_positive_count,
            "complete_bundle_count": retained_bundle_count,
            "incomplete_union_stamp_count": 0,
            "non_exact_union_stamp_count": 0,
            "all_source_stamps_exact_six_camera_one_to_one": True,
        },
        "topics": topic_reports,
        "failures": [],
    }


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


def _runtime_load_input_manifest(attempt: Path) -> dict:
    telemetry = attempt.parents[1] / "host_telemetry"
    paths = {
        "result": attempt / "result.json",
        "latency": attempt / "latency/e2e_latency.json",
        "stack": attempt / "stack.log",
        "recorder": attempt / "recorder.log",
        "vmstat": telemetry / "vmstat.log",
        "nvidia_dmon": telemetry / "nvidia_smi_dmon.log",
        "pidstat": telemetry / "pidstat.log",
        "bag": attempt / "bag",
    }
    for label, path in paths.items():
        if label in {"result", "bag"}:
            continue
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(f"{label} evidence\n", encoding="utf-8")
    manifest = {}
    for label, path in paths.items():
        resolved = path.resolve()
        if resolved.is_dir():
            manifest[label] = {
                "path": str(resolved),
                "files": [
                    {
                        "name": item.name,
                        "size_bytes": item.stat().st_size,
                        "sha256": gate._sha256_file(item),
                    }
                    for item in sorted(resolved.iterdir())
                    if item.is_file()
                ],
            }
        else:
            manifest[label] = {
                "path": str(resolved),
                "size_bytes": resolved.stat().st_size,
                "sha256": gate._sha256_file(resolved),
            }
    return manifest


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


def _fixture(
    tmp_path: Path, *, accepted: bool = True, strict_10hz: bool = False
) -> tuple[Path, Path]:
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
        "started_at": "2026-09-07T00:00:01+00:00",
        "finished_at": "2026-09-07T00:01:21+00:00",
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

    camera_profile_id = (
        gate.CAMERA_PROFILE_10HZ_STRICT_ID
        if strict_10hz
        else gate.CAMERA_PROFILE_5HZ_ID
    )
    camera_rate_hz = 10.0 if strict_10hz else 5.0
    minimum_health_rate_hz = 9.0 if strict_10hz else 4.0
    minimum_complete_bundles = 70 if strict_10hz else 20
    transport = {
        "profile_id": camera_profile_id,
        "sensor_mapping_sha256": "a" * 64,
        "vad_model_override_sha256": "b" * 64,
        "cyclonedds_config_sha256": "c" * 64,
        "rmw_implementation": "rmw_cyclonedds_cpp",
    }
    if strict_10hz:
        transport.update(
            {
                "qualification_scope": (
                    "strict_six_camera_10hz_transport_runtime_only"
                ),
                "simulation_only": True,
                "vehicle_behavior_validated": False,
                "portable_model_loaded": False,
                "portable_model_60kph_validated": False,
                "portable_model_60kph_claim_allowed": False,
                "portable_runtime_graph_absence_required": True,
                "portable_shadow_node": gate.PORTABLE_SHADOW_NODE,
                "portable_output_topics": list(gate.PORTABLE_OUTPUT_TOPICS),
                "real_vehicle_ready": False,
            }
        )
    window = {
        "index": 0,
        "status": "PASS",
        "window": {
            "duration_seconds": 8.0,
            "start_elapsed_seconds": 0.0,
            "end_elapsed_seconds": 8.0,
        },
        "clock": {"rtf": 0.99},
        "minimum_observed_camera_wall_rate_hz": camera_rate_hz,
        "bundles": {
            "complete_bundle_count": minimum_complete_bundles,
            "coverage_percent": 100.0,
            "receipt_span_seconds": {"p95": 0.006},
            "stamp_span_seconds": {"maximum": 0.0},
        },
    }
    if strict_10hz:
        window["maximum_observed_camera_wall_rate_hz"] = 10.0
        window["camera_source_stamp_integrity"] = {
            "status": "PASS",
            "record_count_parity": True,
            "all_records_used_exactly_once": True,
            "all_window_records_used_exactly_once": True,
            "matched_bundle_count": 80,
            "topics": {
                f"/sensing/camera/{camera}/camera_info": {
                    "window_record_count": 80,
                    "source_range_record_count": 80,
                    "positive_window_stamp_count": 80,
                    "zero_or_negative_window_stamp_count": 0,
                    "duplicate_positive_source_range_stamp_count": 0,
                    "non_increasing_source_range_arrival_stamp_delta_count": 0,
                    "strictly_increasing_unique_positive_arrival_stamps": True,
                    "source_rate_hz_from_span": 10.0,
                    "minimum_source_gap_seconds": 0.1,
                    "maximum_source_gap_seconds": 0.1,
                    "window_matched_record_count": 80,
                    "window_unmatched_record_count": 0,
                }
                for camera in (
                    "CAM_FRONT",
                    "CAM_BACK",
                    "CAM_FRONT_LEFT",
                    "CAM_BACK_LEFT",
                    "CAM_FRONT_RIGHT",
                    "CAM_BACK_RIGHT",
                )
            },
        }
        window["portable_runtime_absence"] = {
            "status": "PASS",
            "observation_scope": "window_end_graph_snapshot",
            "continuous_window_observation": False,
            "required_absent_node": gate.PORTABLE_SHADOW_NODE,
            "required_absent_output_topics": list(gate.PORTABLE_OUTPUT_TOPICS),
            "observed_portable_nodes": [],
            "observed_output_publishers": {
                topic: [] for topic in gate.PORTABLE_OUTPUT_TOPICS
            },
            "failures": [],
            "observed_elapsed_seconds": 8.1,
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
                "minimum_camera_wall_rate_hz": minimum_health_rate_hz,
                "minimum_complete_bundle_count": minimum_complete_bundles,
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
            {
                **window,
                "index": index,
                "window": {
                    "duration_seconds": 8.0,
                    "start_elapsed_seconds": float(index),
                    "end_elapsed_seconds": float(index + 8),
                },
            }
            for index in range(3)
        ],
    }
    if strict_10hz:
        health["contract"]["strict_camera_source_integrity_required"] = True
        health["contract"]["bundle_match_tolerance_seconds"] = 0.000005
        health["contract"]["thresholds"].update(
            {
                "maximum_bundle_stamp_span_seconds": 0.000005,
                "maximum_camera_wall_rate_hz": 11.0,
                "minimum_camera_source_rate_hz": 9.5,
                "maximum_camera_source_rate_hz": 10.5,
                "minimum_camera_source_gap_seconds": 0.099995,
                "maximum_camera_source_gap_seconds": 0.100005,
            }
        )
    _write(attempt / "runtime_health.json", health)
    delivery_provenance = None
    runtime_parameters_source = None
    strict_runtime_parameters_path = attempt / "strict_camera_runtime_parameters.json"
    if strict_10hz:
        camera_worktree = ROOT / "src/universe/autoware_universe"
        camera_files = {
            "bundle_dispatch_source": camera_worktree
            / "simulator/autoware_carla_interface/src/autoware_carla_interface/modules/carla_wrapper.py",
            "bundle_dispatch_runtime": camera_worktree
            / "simulator/autoware_carla_interface/src/autoware_carla_interface/modules/carla_wrapper.py",
            "bridge_source": camera_worktree
            / "simulator/autoware_carla_interface/src/autoware_carla_interface/carla_ros.py",
            "bridge_runtime": camera_worktree
            / "simulator/autoware_carla_interface/src/autoware_carla_interface/carla_ros.py",
            "entrypoint_source": camera_worktree
            / "simulator/autoware_carla_interface/src/autoware_carla_interface/carla_autoware.py",
            "entrypoint_runtime": camera_worktree
            / "simulator/autoware_carla_interface/src/autoware_carla_interface/carla_autoware.py",
            "publish_worker_source": camera_worktree
            / "simulator/autoware_carla_interface/src/autoware_carla_interface/modules/sensor_publish_worker.py",
            "publish_worker_runtime": camera_worktree
            / "simulator/autoware_carla_interface/src/autoware_carla_interface/modules/sensor_publish_worker.py",
            "interface_launch_source": camera_worktree
            / "simulator/autoware_carla_interface/launch/autoware_carla_interface.launch.xml",
            "interface_launch_runtime": camera_worktree
            / "simulator/autoware_carla_interface/launch/autoware_carla_interface.launch.xml",
            "delivery_patch": ROOT
            / "patches/autoware_carla_interface_camera_delivery_contract.patch",
        }
        delivery_provenance = {
            label: {
                "declared_path": str(path),
                "path": str(path.resolve()),
                "sha256": gate._sha256_file(path),
            }
            for label, path in camera_files.items()
        }
        delivery_provenance["patch_application"] = {
            "status": "PASS",
            "worktree": str(camera_worktree.resolve()),
            "verification": "git_apply_reverse_check",
        }
        runtime_parameters = {
            "schema_version": 1,
            "status": "PASS",
            "node": "/autoware_carla_interface",
            "parameters": {
                "fixed_delta_seconds": 0.05,
                "sync_mode": True,
                "camera_frame_barrier_enabled": True,
                "camera_frame_wait_timeout_sec": 0.25,
                "camera_publish_deadline_sec": 0.25,
                "camera_pending_frame_limit": 8,
            },
            "derived_camera_frame_stride": 2,
            "derivation": (
                "sensor_tick_0.1_sec_divided_by_fixed_delta_0.05_sec"
            ),
            "read_only": True,
        }
        _write(strict_runtime_parameters_path, runtime_parameters)
        runtime_parameters_source = {
            "path": str(strict_runtime_parameters_path.resolve()),
            "sha256": gate._sha256_file(strict_runtime_parameters_path),
            "evidence": runtime_parameters,
        }
    camera_evidence = {
            "schema_version": 1,
            "status": "PASS",
            "contract": {
                "profile_id": camera_profile_id,
                "sensor_count": 6,
                "source_frequency_hz": camera_rate_hz,
                "camera_image_publish_qos": "best_effort",
                "camera_image_publish_depth": 1,
            },
            "transport_provenance": dict(transport),
            "bundle_coverage_percent": 100.0,
            "maximum_camera_stamp_gap_sec": 0.1 if strict_10hz else 0.2,
    }
    if strict_10hz:
        camera_evidence.update(
            {
                "qualification_id": (
                    "carla_six_camera_10hz_strict_transport_runtime_v2"
                ),
                "qualification_boundary": {
                    "scope": "strict_six_camera_10hz_transport_runtime_only",
                    "simulation_only": True,
                    "vehicle_behavior_validated": False,
                    "portable_model_loaded": False,
                    "portable_model_60kph_validated": False,
                    "portable_model_60kph_claim_allowed": False,
                    "real_vehicle_ready": False,
                },
                "barrier_contract": {
                    "dispatch_policy": "exact_due_frame_barrier_fail_closed_v1",
                    "delivery_contract_id": (
                        "strict10_exact_due_frame_fail_closed_v1"
                    ),
                    "scope": "strict_six_camera_10hz_transport_only",
                    "frame_stride": 2,
                    "expected_rgb_count": 6,
                    "wait_timeout_sec": 0.25,
                    "publish_deadline_sec": 0.25,
                    "pending_frame_limit": 8,
                },
                "runtime_health": {
                    "path": str(attempt / "runtime_health.json"),
                    "sha256": gate._sha256_file(attempt / "runtime_health.json"),
                    "status": "PASS",
                    "winning_window_indexes": [0, 1, 2],
                },
                "minimum_camera_stamp_rate_hz": 10.0,
                "maximum_camera_stamp_rate_hz": 10.0,
                "minimum_camera_stamp_gap_sec": 0.1,
                "maximum_camera_bundle_stamp_span_sec": 0.0,
                "portable_runtime_absence": {
                    "status": "PASS",
                    "observation_scope": "winning_window_end_graph_snapshots",
                    "continuous_window_observation": False,
                    "observed_window_indexes": [0, 1, 2],
                    "observations": [
                        {
                            "window_index": index,
                            "status": "PASS",
                            "observed_elapsed_seconds": 8.1,
                        }
                        for index in range(3)
                    ],
                    "portable_shadow_node_observed": False,
                    "portable_output_publisher_observed": False,
                },
                "camera_delivery_contract_provenance": delivery_provenance,
                "runtime_parameters": runtime_parameters_source,
            }
        )
        camera_evidence["contract"].update(
            {
                "minimum_camera_rate_hz": 9.5,
                "maximum_camera_rate_hz": 10.5,
                "minimum_bundle_coverage_percent": 99.0,
                "minimum_camera_stamp_gap_sec": 0.099995,
                "maximum_camera_stamp_gap_sec": 0.100005,
                "maximum_camera_bundle_stamp_span_sec": 0.000005,
                "camera_info_publish_qos": "reliable",
            }
        )
        camera_evidence_name = "camera_source_10hz_strict_validation.json"
    else:
        camera_evidence["contract"]["real_vehicle_ready"] = False
        camera_evidence.update(
            {
                "candidate_front_acceptance_percent": 100.0,
                "raw_six_image_queue_integrity": {"status": "PASS"},
            }
        )
        camera_evidence_name = "camera_source_5hz_validation.json"
    _write(attempt / camera_evidence_name, camera_evidence)
    _write(
        attempt / "runtime_load_analysis.json",
        {
            "schema_version": 3,
            "status": "complete",
            "problems": [],
            "input_manifest": _runtime_load_input_manifest(attempt),
            "vad_runtime": {
                "phases": {
                    "full_run": {"aggregate_rtf": 0.99, "wall_output_rate_hz": 5.0}
                }
            },
            "camera_delivery": {
                "source_rate_hz_from_median_period": camera_rate_hz,
                "source_period_sec": {
                    "min": 0.1 if strict_10hz else 0.2,
                    "max": 0.1 if strict_10hz else 0.2,
                },
                "bundle_coverage_percent": 100.0,
                "matched_bundle_count": 100,
                "phases": {"full_run": {"receipt_span_ms": {"p95": 6.0}}},
            },
        },
    )
    if strict_10hz:
        runtime_load_path = attempt / "runtime_load_analysis.json"
        runtime_load = json.loads(runtime_load_path.read_text(encoding="utf-8"))
        runtime_load["camera_delivery"]["source_stamp_integrity"] = {
            "status": "PASS",
            "record_count_parity": True,
            "all_records_used_exactly_once": True,
            "matched_bundle_count": 100,
            "topics": {
                f"/sensing/camera/{camera}/camera_info": {
                    "record_count": 100,
                    "positive_stamp_count": 100,
                    "unique_positive_stamp_count": 100,
                    "zero_or_negative_stamp_count": 0,
                    "duplicate_positive_stamp_count": 0,
                    "non_increasing_positive_stamp_delta_count": 0,
                    "strictly_increasing_unique_positive_stamps": True,
                    "unused_record_count": 0,
                }
                for camera in (
                    "CAM_FRONT",
                    "CAM_BACK",
                    "CAM_FRONT_LEFT",
                    "CAM_BACK_LEFT",
                    "CAM_FRONT_RIGHT",
                    "CAM_BACK_RIGHT",
                )
            },
        }
        runtime_load["camera_delivery"][
            "edge_bounded_source_stamp_integrity"
        ] = _edge_bounded_integrity()
        _write(runtime_load_path, runtime_load)
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
        "CAMERA_SOURCE_5HZ": "false" if strict_10hz else "true",
        "CAMERA_SOURCE_10HZ_STRICT": "true" if strict_10hz else "false",
        "CAMERA_SOURCE_SENSOR_TICK_SEC": "0.1" if strict_10hz else "0.2",
        "CAMERA_ROS_PUBLISH_HZ": "10.0" if strict_10hz else "5.0",
        "CAMERA_TRANSPORT_PROFILE_ID": camera_profile_id,
        "PORTABLE_SHADOW_ENABLED": "false",
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
    if strict_10hz:
        recorder_measurement = {
            "schema_version": 1,
            "status": "PASS",
            "policy": (
                "paused_until_all_six_camera_info_subscriptions_acknowledged_v1"
            ),
            "recorder_started_paused": True,
            "camera_info_subscription_count": 6,
            "pause_state_proof": "Waiting for recording: Press SPACE to start.",
            "resume_control": "humble_rosbag2_owned_pty_space_key_v1",
            "resume_acknowledgement": "Resuming recording.",
            "resume_requested_at": "2026-09-07T00:00:00.100000Z",
            "resumed_at": "2026-09-07T00:00:00.200000Z",
            "measurement_boundary": (
                "owned_pty_resume_acknowledged_before_route_evaluation"
            ),
        }
        recorder_measurement_path = attempt / "recorder_measurement_start.json"
        _write(recorder_measurement_path, recorder_measurement)
        env.update(
            {
                "CAMERA_TRANSPORT_QUALIFICATION_SCOPE": (
                    "strict_six_camera_10hz_transport_runtime_only"
                ),
                "CAMERA_TRANSPORT_SIMULATION_ONLY": "true",
                "CAMERA_TRANSPORT_VEHICLE_BEHAVIOR_VALIDATED": "false",
                "CAMERA_TRANSPORT_REAL_VEHICLE_READY": "false",
                "CAMERA_TRANSPORT_SENSOR_MAPPING_ROLE": (
                    "six_camera_source_abi_only"
                ),
                "CAMERA_TRANSPORT_SENSOR_MAPPING_REUSE": (
                    "portable_common10_mapping_without_portable_runtime"
                ),
                "PORTABLE_SHADOW_LAUNCH_REQUESTED": "false",
                "PORTABLE_RUNTIME_BUNDLE_PROVIDED": "false",
                "PORTABLE_MODEL_LOADED": "false",
                "PORTABLE_MODEL_60KPH_VALIDATED": "false",
                "PORTABLE_MODEL_60KPH_CLAIM_ALLOWED": "false",
                "CARLA_CAMERA_BUNDLE_DISPATCH_POLICY": (
                    "exact_due_frame_barrier_fail_closed_v1"
                ),
                "CARLA_CAMERA_FRAME_BARRIER_ENABLED": "true",
                "CARLA_CAMERA_FRAME_STRIDE": "2",
                "CARLA_CAMERA_FRAME_WAIT_TIMEOUT_SEC": "0.25",
                "CARLA_CAMERA_PUBLISH_DEADLINE_SEC": "0.25",
                "CARLA_CAMERA_PENDING_FRAME_LIMIT": "8",
                "CARLA_CAMERA_EXPECTED_RGB_COUNT": "6",
                "CARLA_CAMERA_BARRIER_SCOPE": (
                    "strict_six_camera_10hz_transport_only"
                ),
                "CARLA_CAMERA_DELIVERY_CONTRACT_ID": (
                    "strict10_exact_due_frame_fail_closed_v1"
                ),
                "CARLA_FIXED_DELTA_SECONDS_PINNED": "0.05",
                "CARLA_SYNC_MODE_PINNED": "true",
                "CARLA_CAMERA_RUNTIME_PARAMETERS_FILE": str(
                    strict_runtime_parameters_path
                ),
                "CARLA_CAMERA_RUNTIME_PARAMETERS_SHA256": gate._sha256_file(
                    strict_runtime_parameters_path
                ),
                "CARLA_CAMERA_RUNTIME_PARAMETERS_STATUS": "PASS",
                "CARLA_CAMERA_BUNDLE_DISPATCH_SOURCE_FILE": str(
                    camera_files["bundle_dispatch_source"]
                ),
                "CARLA_CAMERA_BUNDLE_DISPATCH_RUNTIME_FILE": str(
                    camera_files["bundle_dispatch_runtime"]
                ),
                "CARLA_CAMERA_BUNDLE_DISPATCH_SHA256": gate._sha256_file(
                    camera_files["bundle_dispatch_source"]
                ),
                "CARLA_CAMERA_BRIDGE_SOURCE_FILE": str(
                    camera_files["bridge_source"]
                ),
                "CARLA_CAMERA_BRIDGE_RUNTIME_FILE": str(
                    camera_files["bridge_runtime"]
                ),
                "CARLA_CAMERA_BRIDGE_SHA256": gate._sha256_file(
                    camera_files["bridge_source"]
                ),
                "CARLA_CAMERA_ENTRYPOINT_SOURCE_FILE": str(
                    camera_files["entrypoint_source"]
                ),
                "CARLA_CAMERA_ENTRYPOINT_RUNTIME_FILE": str(
                    camera_files["entrypoint_runtime"]
                ),
                "CARLA_CAMERA_ENTRYPOINT_SHA256": gate._sha256_file(
                    camera_files["entrypoint_source"]
                ),
                "CARLA_CAMERA_PUBLISH_WORKER_SOURCE_FILE": str(
                    camera_files["publish_worker_source"]
                ),
                "CARLA_CAMERA_PUBLISH_WORKER_RUNTIME_FILE": str(
                    camera_files["publish_worker_runtime"]
                ),
                "CARLA_CAMERA_PUBLISH_WORKER_SHA256": gate._sha256_file(
                    camera_files["publish_worker_source"]
                ),
                "CARLA_CAMERA_INTERFACE_LAUNCH_SOURCE_FILE": str(
                    camera_files["interface_launch_source"]
                ),
                "CARLA_CAMERA_INTERFACE_LAUNCH_RUNTIME_FILE": str(
                    camera_files["interface_launch_runtime"]
                ),
                "CARLA_CAMERA_INTERFACE_LAUNCH_SHA256": gate._sha256_file(
                    camera_files["interface_launch_source"]
                ),
                "CARLA_CAMERA_DELIVERY_PATCH_FILE": str(
                    camera_files["delivery_patch"]
                ),
                "CARLA_CAMERA_DELIVERY_PATCH_SHA256": gate._sha256_file(
                    camera_files["delivery_patch"]
                ),
                "CARLA_CAMERA_DELIVERY_PATCH_WORKTREE": str(camera_worktree),
                "CARLA_CAMERA_DELIVERY_PATCH_REVERSE_CHECK": "PASS",
                "STRICT_RECORDER_START_PAUSED": "true",
                "STRICT_RECORDER_CAMERA_INFO_SUBSCRIPTIONS_ACKNOWLEDGED": "6",
                "STRICT_RECORDER_RESUME_CONTROL": (
                    "humble_rosbag2_owned_pty_space_key_v1"
                ),
                "STRICT_RECORDER_MEASUREMENT_RESUME_STATUS": "pass",
                "STRICT_RECORDER_MEASUREMENT_RESUME_REQUESTED_AT": (
                    "2026-09-07T00:00:00.100000Z"
                ),
                "STRICT_RECORDER_MEASUREMENT_RESUMED_AT": (
                    "2026-09-07T00:00:00.200000Z"
                ),
                "STRICT_RECORDER_MEASUREMENT_EVIDENCE_FILE": (
                    "recorder_measurement_start.json"
                ),
                "STRICT_RECORDER_MEASUREMENT_EVIDENCE_SHA256": (
                    gate._sha256_file(recorder_measurement_path)
                ),
                "ROUTE_EVALUATION_STARTED_AT": (
                    "2026-09-07T00:00:00.300000Z"
                ),
                "ROUTE_EVALUATION_FINISHED_AT": (
                    "2026-09-07T00:01:21.100000Z"
                ),
                "RECORDER_ROUTE_EVALUATION_LIVENESS_STATUS": "pass",
                "RECORDER_ROUTE_COMPLETION_LIVENESS_STATUS": "pass",
                "RECORDER_OWNED_GROUP_CLEANUP_STATUS": "pass",
                "RECORDER_CONTROL_FIFO_REMOVED": "true",
                "STACK_OWNED_GROUP_CLEANUP_STATUS": "pass",
                "STACK_POST_SHUTDOWN_CRITICAL_PROCESS_CHECK": "pass",
            }
        )
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


def _rebind_runtime_health_evidence(attempt: Path) -> None:
    health_path = attempt / "runtime_health.json"
    health_sha = gate._sha256_file(health_path)
    camera_path = attempt / "camera_source_10hz_strict_validation.json"
    camera = json.loads(camera_path.read_text(encoding="utf-8"))
    camera["runtime_health"]["sha256"] = health_sha
    _write(camera_path, camera)
    env_path = attempt / "runtime.env"
    environment = gate._parse_env(env_path)
    environment["RUNTIME_HEALTH_EVIDENCE_SHA256"] = health_sha
    env_path.write_text(
        "".join(f"{key}={value}\n" for key, value in environment.items()),
        encoding="utf-8",
    )


def _convert_strict_fixture_to_legacy_v1(attempt: Path) -> None:
    health_path = attempt / "runtime_health.json"
    health = json.loads(health_path.read_text(encoding="utf-8"))
    health["contract"]["camera_transport"]["profile_id"] = (
        gate.CAMERA_PROFILE_10HZ_STRICT_LEGACY_ID
    )
    health["contract"]["thresholds"].pop(
        "minimum_camera_source_gap_seconds"
    )
    for window in health["windows"]:
        integrity = window["camera_source_stamp_integrity"]
        integrity.pop("all_window_records_used_exactly_once")
        for topic in integrity["topics"].values():
            topic.pop("minimum_source_gap_seconds")
            topic.pop("window_matched_record_count")
            topic.pop("window_unmatched_record_count")
        window["portable_runtime_absence"].pop("observation_scope")
        window["portable_runtime_absence"].pop(
            "continuous_window_observation"
        )
    _write(health_path, health)

    camera_path = attempt / "camera_source_10hz_strict_validation.json"
    camera = json.loads(camera_path.read_text(encoding="utf-8"))
    camera["qualification_id"] = (
        "carla_six_camera_10hz_strict_transport_runtime_v1"
    )
    camera["contract"]["profile_id"] = gate.CAMERA_PROFILE_10HZ_STRICT_LEGACY_ID
    camera["transport_provenance"]["profile_id"] = (
        gate.CAMERA_PROFILE_10HZ_STRICT_LEGACY_ID
    )
    camera["contract"].pop("minimum_camera_stamp_gap_sec")
    camera.pop("minimum_camera_stamp_gap_sec")
    camera["portable_runtime_absence"]["observation_scope"] = (
        "winning_runtime_health_windows"
    )
    camera["portable_runtime_absence"].pop(
        "continuous_window_observation"
    )
    camera["runtime_health"]["sha256"] = gate._sha256_file(health_path)
    _write(camera_path, camera)

    runtime_load_path = attempt / "runtime_load_analysis.json"
    runtime_load = json.loads(runtime_load_path.read_text(encoding="utf-8"))
    runtime_load["camera_delivery"].pop(
        "edge_bounded_source_stamp_integrity"
    )
    _write(runtime_load_path, runtime_load)

    env_path = attempt / "runtime.env"
    environment = gate._parse_env(env_path)
    environment["CAMERA_TRANSPORT_PROFILE_ID"] = (
        gate.CAMERA_PROFILE_10HZ_STRICT_LEGACY_ID
    )
    environment["RUNTIME_HEALTH_EVIDENCE_SHA256"] = gate._sha256_file(health_path)
    obsolete_prefixes = (
        "STRICT_RECORDER_",
        "RECORDER_ROUTE_",
        "RECORDER_OWNED_",
        "RECORDER_CONTROL_",
        "ROUTE_EVALUATION_",
        "STACK_",
    )
    environment = {
        key: value
        for key, value in environment.items()
        if not key.startswith(obsolete_prefixes)
    }
    env_path.write_text(
        "".join(f"{key}={value}\n" for key, value in environment.items()),
        encoding="utf-8",
    )
    (attempt / "recorder_measurement_start.json").unlink()


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


def test_strict_10hz_transport_pass_is_separate_from_portable_model_validity(
    tmp_path: Path,
) -> None:
    attempt, source = _fixture(tmp_path, strict_10hz=True)

    result = gate.evaluate_trial(attempt, source)

    assert result["evidence_integrity_status"] == "PASS"
    assert result["simulation_pilot_acceptance_status"] == "PASS"
    assert result["camera_transport_qualification_status"] == "PASS"
    qualification = result["camera_transport_qualification"]
    assert qualification == {
        "status": "PASS",
        "profile_id": gate.CAMERA_PROFILE_10HZ_STRICT_ID,
        "evidence_file": "camera_source_10hz_strict_validation.json",
        "scope": "strict_six_camera_10hz_transport_runtime_only",
        "simulation_only": True,
        "vehicle_behavior_validated": False,
        "portable_model_loaded": False,
        "portable_model_60kph_validated": False,
        "portable_model_60kph_claim_allowed": False,
        "real_vehicle_ready": False,
        "failures": [],
    }
    assert any(
        "no Portable model was loaded or validated at 60 km/h" in item
        for item in result["readiness_blockers"]
    )
    recorder = result["sources"]["strict_recorder_measurement"]
    assert recorder["evidence"]["status"] == "PASS"
    assert recorder["control_fifo_removed"] is True


def test_legacy_strict_v1_remains_offline_validatable_without_v2_recorder_proof(
    tmp_path: Path,
) -> None:
    attempt, source = _fixture(tmp_path, strict_10hz=True)
    _convert_strict_fixture_to_legacy_v1(attempt)

    result = gate.evaluate_trial(attempt, source)

    assert result["evidence_integrity_status"] == "PASS"
    assert result["simulation_pilot_acceptance_status"] == "PASS"
    assert result["camera_transport_qualification_status"] == "PASS"
    assert result["camera_transport_qualification"]["profile_id"] == (
        gate.CAMERA_PROFILE_10HZ_STRICT_LEGACY_ID
    )
    assert result["sources"]["strict_recorder_measurement"] is None
    assert result["sources"]["legacy_camera_provenance_validation_scope"] == {
        "status": "DECLARATION_CONSISTENCY_ONLY",
        "offline_legacy_gate_semantics": True,
        "archived_source_bytes_available": False,
        "current_source_bytes_rehashed": False,
        "byte_identical_runtime_replay_claim_allowed": False,
    }
    assert any(
        "declaration consistency only" in blocker
        for blocker in result["readiness_blockers"]
    )
    assert all(
        "strict recorder" not in failure
        for failure in result["integrity_failures"]
    )


@pytest.mark.skipif(
    not REAL_STRICT_V1_ATTEMPT.is_dir(),
    reason="raw local strict-v1 artifact is not part of a lightweight clone",
)
def test_real_strict_v1_artifact_preserves_original_key_verdicts() -> None:
    environment = gate._parse_env(REAL_STRICT_V1_ATTEMPT / "runtime.env")
    source = Path(environment["SOURCE_ROUTE_FILE"])

    result = gate.evaluate_trial(REAL_STRICT_V1_ATTEMPT, source)

    assert result["evidence_integrity_status"] == "PASS"
    assert result["simulation_pilot_acceptance_status"] == "FAILED"
    assert result["camera_transport_qualification_status"] == "FAILED"
    assert result["physical_goal_completion_status"] == "PASS"
    assert result["speed_exposure_contract_status"] == "FAILED"
    assert result["camera_transport_qualification"]["failures"] == [
        "full-run six-camera receipt p95 exceeds 40 ms",
        (
            "full-run six-camera stamps are not strictly monotonic, unique, "
            "count-parity, and one-to-one bundled"
        ),
    ]


@pytest.mark.parametrize(
    ("mutation", "expected_failure"),
    (
        ("winning_duplicate", "three consecutive PASS windows"),
        ("duration", "one unique 8-second window"),
        ("bundle_count", "one unique 8-second window"),
        ("duplicate_window", "one unique 8-second window"),
    ),
)
def test_strict_v2_replays_three_unique_complete_health_windows(
    tmp_path: Path,
    mutation: str,
    expected_failure: str,
) -> None:
    attempt, source = _fixture(tmp_path, strict_10hz=True)
    health_path = attempt / "runtime_health.json"
    health = json.loads(health_path.read_text(encoding="utf-8"))
    if mutation == "winning_duplicate":
        health["sequence"]["winning_window_indexes"] = [0, 0, 0]
    elif mutation == "duration":
        health["windows"][1]["window"]["duration_seconds"] = 7.9
    elif mutation == "bundle_count":
        health["windows"][1]["bundles"]["complete_bundle_count"] = 69
    else:
        health["windows"].append(dict(health["windows"][1]))
    _write(health_path, health)
    _rebind_runtime_health_evidence(attempt)

    result = gate.evaluate_trial(attempt, source)

    assert result["camera_transport_qualification_status"] == "FAILED"
    assert any(
        expected_failure in failure
        for failure in result["camera_transport_qualification"]["failures"]
    )


def test_strict_v2_accepts_float_rounding_at_exact_maximum_period(
    tmp_path: Path,
) -> None:
    attempt, source = _fixture(tmp_path, strict_10hz=True)
    rounded_boundary = 0.10000500000000001
    health_path = attempt / "runtime_health.json"
    health = json.loads(health_path.read_text(encoding="utf-8"))
    for window in health["windows"]:
        for topic in window["camera_source_stamp_integrity"]["topics"].values():
            topic["maximum_source_gap_seconds"] = rounded_boundary
    _write(health_path, health)
    _rebind_runtime_health_evidence(attempt)
    camera_path = attempt / "camera_source_10hz_strict_validation.json"
    camera = json.loads(camera_path.read_text(encoding="utf-8"))
    camera["maximum_camera_stamp_gap_sec"] = rounded_boundary
    _write(camera_path, camera)

    result = gate.evaluate_trial(attempt, source)

    assert result["evidence_integrity_status"] == "PASS"
    assert result["camera_transport_qualification_status"] == "PASS"


@pytest.mark.parametrize(
    "mutation", ("delete", "hash", "timestamp_order", "equal_route_start")
)
def test_strict_v2_requires_bound_recorder_measurement_proof(
    tmp_path: Path,
    mutation: str,
) -> None:
    attempt, source = _fixture(tmp_path, strict_10hz=True)
    evidence_path = attempt / "recorder_measurement_start.json"
    env_path = attempt / "runtime.env"
    if mutation == "delete":
        evidence_path.unlink()
    elif mutation == "hash":
        environment = env_path.read_text(encoding="utf-8")
        recorded_hash = gate._sha256_file(evidence_path)
        env_path.write_text(
            environment.replace(recorded_hash, "0" * 64),
            encoding="utf-8",
        )
    else:
        evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
        evidence["resumed_at"] = (
            "2026-09-07T00:00:00.300000Z"
            if mutation == "equal_route_start"
            else "2026-09-07T00:00:00.400000Z"
        )
        _write(evidence_path, evidence)
        environment = gate._parse_env(env_path)
        environment["STRICT_RECORDER_MEASUREMENT_RESUMED_AT"] = evidence[
            "resumed_at"
        ]
        environment["STRICT_RECORDER_MEASUREMENT_EVIDENCE_SHA256"] = (
            gate._sha256_file(evidence_path)
        )
        env_path.write_text(
            "".join(f"{key}={value}\n" for key, value in environment.items()),
            encoding="utf-8",
        )

    result = gate.evaluate_trial(attempt, source)

    assert result["evidence_integrity_status"] == "FAILED"
    assert result["camera_transport_qualification_status"] == "FAILED"
    assert any(
        "strict recorder" in failure
        for failure in result["camera_transport_qualification"]["failures"]
    )


def test_strict_10hz_rejects_portable_60kph_validity_claim(tmp_path: Path) -> None:
    attempt, source = _fixture(tmp_path, strict_10hz=True)
    env_path = attempt / "runtime.env"
    env_path.write_text(
        env_path.read_text(encoding="utf-8").replace(
            "PORTABLE_MODEL_60KPH_VALIDATED=false",
            "PORTABLE_MODEL_60KPH_VALIDATED=true",
        ),
        encoding="utf-8",
    )

    result = gate.evaluate_trial(attempt, source)

    assert result["evidence_integrity_status"] == "FAILED"
    assert result["simulation_pilot_acceptance_status"] == "FAILED"
    assert any(
        "PORTABLE_MODEL_60KPH_VALIDATED" in item
        for item in result["integrity_failures"]
    )


def test_strict_10hz_rejects_missing_fail_closed_barrier(tmp_path: Path) -> None:
    attempt, source = _fixture(tmp_path, strict_10hz=True)
    env_path = attempt / "runtime.env"
    env_path.write_text(
        env_path.read_text(encoding="utf-8").replace(
            "CARLA_CAMERA_FRAME_BARRIER_ENABLED=true",
            "CARLA_CAMERA_FRAME_BARRIER_ENABLED=false",
        ),
        encoding="utf-8",
    )

    result = gate.evaluate_trial(attempt, source)

    assert result["evidence_integrity_status"] == "FAILED"
    assert result["camera_transport_qualification_status"] == "FAILED"
    assert any(
        "CARLA_CAMERA_FRAME_BARRIER_ENABLED" in item
        for item in result["integrity_failures"]
    )


def test_strict_10hz_full_run_camera_rate_fails_transport_qualification(
    tmp_path: Path,
) -> None:
    attempt, source = _fixture(tmp_path, strict_10hz=True)
    path = attempt / "runtime_load_analysis.json"
    runtime = json.loads(path.read_text(encoding="utf-8"))
    runtime["camera_delivery"]["source_rate_hz_from_median_period"] = 8.0
    _write(path, runtime)

    result = gate.evaluate_trial(attempt, source)

    assert result["evidence_integrity_status"] == "PASS"
    assert result["simulation_pilot_acceptance_status"] == "FAILED"
    assert result["camera_transport_qualification_status"] == "FAILED"
    assert (
        "full-run six-camera source cadence is outside [9.5, 10.5] Hz"
        in result["camera_transport_qualification"]["failures"]
    )


@pytest.mark.parametrize("source", ("runtime_load", "postrun_camera"))
def test_strict_v2_rejects_compressed_50ms_source_interval(
    tmp_path: Path,
    source: str,
) -> None:
    attempt, route = _fixture(tmp_path, strict_10hz=True)
    if source == "runtime_load":
        path = attempt / "runtime_load_analysis.json"
        document = json.loads(path.read_text(encoding="utf-8"))
        document["camera_delivery"]["source_period_sec"]["min"] = 0.05
    else:
        path = attempt / "camera_source_10hz_strict_validation.json"
        document = json.loads(path.read_text(encoding="utf-8"))
        document["minimum_camera_stamp_gap_sec"] = 0.05
    _write(path, document)

    result = gate.evaluate_trial(attempt, route)

    assert result["camera_transport_qualification_status"] == "FAILED"
    assert any(
        "below 0.099995 s" in failure
        for failure in result["camera_transport_qualification"]["failures"]
    )


@pytest.mark.parametrize("mutation", ("manifest_hash", "input_after_analysis"))
def test_strict_v2_rehashes_runtime_load_inputs(
    tmp_path: Path,
    mutation: str,
) -> None:
    attempt, source = _fixture(tmp_path, strict_10hz=True)
    runtime_path = attempt / "runtime_load_analysis.json"
    runtime = json.loads(runtime_path.read_text(encoding="utf-8"))
    if mutation == "manifest_hash":
        runtime["input_manifest"]["stack"]["sha256"] = "0" * 64
        _write(runtime_path, runtime)
    else:
        (attempt / "stack.log").write_text(
            "changed after runtime-load analysis\n", encoding="utf-8"
        )

    result = gate.evaluate_trial(attempt, source)

    assert result["evidence_integrity_status"] == "FAILED"
    assert result["camera_transport_qualification_status"] == "FAILED"
    assert any(
        "runtime-load stack digest/size mismatch" in failure
        for failure in result["camera_transport_qualification"]["failures"]
    )


def test_legacy_5hz_accepts_absent_new_camera_and_portable_false_keys(
    tmp_path: Path,
) -> None:
    attempt, source = _fixture(tmp_path)
    env_path = attempt / "runtime.env"
    lines = [
        line
        for line in env_path.read_text(encoding="utf-8").splitlines()
        if not line.startswith("CAMERA_SOURCE_10HZ_STRICT=")
        and not line.startswith("PORTABLE_SHADOW_ENABLED=")
    ]
    env_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    result = gate.evaluate_trial(attempt, source)

    assert result["evidence_integrity_status"] == "PASS"
    assert result["camera_transport_qualification_status"] == "PASS"


def test_strict_10hz_rejects_observed_portable_node_in_health_window(
    tmp_path: Path,
) -> None:
    attempt, source = _fixture(tmp_path, strict_10hz=True)
    health_path = attempt / "runtime_health.json"
    health = json.loads(health_path.read_text(encoding="utf-8"))
    absence = health["windows"][1]["portable_runtime_absence"]
    absence["status"] = "FAIL"
    absence["observed_portable_nodes"] = [gate.PORTABLE_SHADOW_NODE]
    _write(health_path, health)

    result = gate.evaluate_trial(attempt, source)

    assert result["camera_transport_qualification_status"] == "FAILED"
    assert any(
        "Portable node/output absence" in failure
        for failure in result["camera_transport_qualification"]["failures"]
    )


def test_strict_10hz_rejects_health_and_postrun_stamp_span_drift(
    tmp_path: Path,
) -> None:
    attempt, source = _fixture(tmp_path, strict_10hz=True)
    health_path = attempt / "runtime_health.json"
    health = json.loads(health_path.read_text(encoding="utf-8"))
    health["windows"][0]["bundles"]["stamp_span_seconds"]["maximum"] = 0.1
    _write(health_path, health)
    camera_path = attempt / "camera_source_10hz_strict_validation.json"
    camera = json.loads(camera_path.read_text(encoding="utf-8"))
    camera["maximum_camera_bundle_stamp_span_sec"] = 0.1
    _write(camera_path, camera)

    result = gate.evaluate_trial(attempt, source)

    assert result["camera_transport_qualification_status"] == "FAILED"
    failures = result["camera_transport_qualification"]["failures"]
    assert any("source-stamp span exceeds 5 us" in failure for failure in failures)


def test_strict_10hz_rejects_missing_camera_delivery_source(
    tmp_path: Path,
) -> None:
    attempt, source = _fixture(tmp_path, strict_10hz=True)
    env_path = attempt / "runtime.env"
    env_path.write_text(
        env_path.read_text(encoding="utf-8").replace(
            "CARLA_CAMERA_BRIDGE_SOURCE_FILE=",
            "CARLA_CAMERA_BRIDGE_SOURCE_FILE=/missing/",
        ),
        encoding="utf-8",
    )

    result = gate.evaluate_trial(attempt, source)

    assert result["camera_transport_qualification_status"] == "FAILED"
    assert any(
        "bridge source path/hash" in failure
        for failure in result["camera_transport_qualification"]["failures"]
    )


def test_strict_10hz_rejects_missing_camera_entrypoint_source(
    tmp_path: Path,
) -> None:
    attempt, source = _fixture(tmp_path, strict_10hz=True)
    env_path = attempt / "runtime.env"
    env_path.write_text(
        env_path.read_text(encoding="utf-8").replace(
            "CARLA_CAMERA_ENTRYPOINT_SOURCE_FILE=",
            "CARLA_CAMERA_ENTRYPOINT_SOURCE_FILE=/missing/",
        ),
        encoding="utf-8",
    )

    result = gate.evaluate_trial(attempt, source)

    assert result["camera_transport_qualification_status"] == "FAILED"
    assert any(
        "entrypoint source path/hash" in failure
        for failure in result["camera_transport_qualification"]["failures"]
    )


def test_strict_10hz_rejects_tampered_delivery_patch(tmp_path: Path) -> None:
    attempt, source = _fixture(tmp_path, strict_10hz=True)
    original = ROOT / "patches/autoware_carla_interface_camera_delivery_contract.patch"
    tampered = attempt / "tampered_delivery.patch"
    tampered.write_bytes(original.read_bytes() + b"\n# tampered\n")
    env_path = attempt / "runtime.env"
    text = env_path.read_text(encoding="utf-8")
    old_line = next(
        line for line in text.splitlines()
        if line.startswith("CARLA_CAMERA_DELIVERY_PATCH_FILE=")
    )
    env_path.write_text(
        text.replace(old_line, f"CARLA_CAMERA_DELIVERY_PATCH_FILE={tampered}"),
        encoding="utf-8",
    )

    result = gate.evaluate_trial(attempt, source)

    assert result["camera_transport_qualification_status"] == "FAILED"
    assert any(
        "delivery patch" in failure
        for failure in result["camera_transport_qualification"]["failures"]
    )


@pytest.mark.parametrize("mutation", ("delete", "tamper"))
def test_strict_10hz_requires_bound_live_runtime_parameters(
    tmp_path: Path, mutation: str
) -> None:
    attempt, source = _fixture(tmp_path, strict_10hz=True)
    path = attempt / "strict_camera_runtime_parameters.json"
    if mutation == "delete":
        path.unlink()
    else:
        document = json.loads(path.read_text(encoding="utf-8"))
        document["parameters"]["fixed_delta_seconds"] = 0.1
        _write(path, document)

    result = gate.evaluate_trial(attempt, source)

    assert result["camera_transport_qualification_status"] == "FAILED"
    assert any(
        "runtime-parameter evidence" in failure
        for failure in result["camera_transport_qualification"]["failures"]
    )


@pytest.mark.parametrize("mutation", ("delete", "malformed"))
def test_strict_10hz_requires_complete_full_run_runtime_load(
    tmp_path: Path, mutation: str
) -> None:
    attempt, source = _fixture(tmp_path, strict_10hz=True)
    path = attempt / "runtime_load_analysis.json"
    if mutation == "delete":
        path.unlink()
    else:
        _write(path, {"schema_version": 2, "status": "partial", "problems": ["x"]})

    result = gate.evaluate_trial(attempt, source)

    assert result["camera_transport_qualification_status"] == "FAILED"
    assert result["camera_transport_qualification"]["failures"]


def test_strict_10hz_rejects_non_front_duplicate_source_stamp(
    tmp_path: Path,
) -> None:
    attempt, source = _fixture(tmp_path, strict_10hz=True)
    path = attempt / "runtime_load_analysis.json"
    runtime_load = json.loads(path.read_text(encoding="utf-8"))
    topic = "/sensing/camera/CAM_BACK_RIGHT/camera_info"
    integrity = runtime_load["camera_delivery"][
        "edge_bounded_source_stamp_integrity"
    ]
    integrity["status"] = "FAIL"
    integrity["duplicate_positive_record_count"] = 1
    integrity["topics"][topic].update(
        {
            "unique_positive_stamp_count": 99,
            "duplicate_positive_stamp_count": 1,
            "non_increasing_positive_stamp_delta_count": 1,
            "strictly_increasing_unique_positive_stamps": False,
        }
    )
    _write(path, runtime_load)

    result = gate.evaluate_trial(attempt, source)

    assert result["camera_transport_qualification_status"] == "FAILED"
    assert any(
        "recorder edges" in failure
        for failure in result["camera_transport_qualification"]["failures"]
    )


def test_strict_v2_allows_one_partial_union_stamp_at_each_bag_edge(
    tmp_path: Path,
) -> None:
    attempt, source = _fixture(tmp_path, strict_10hz=True)
    path = attempt / "runtime_load_analysis.json"
    runtime_load = json.loads(path.read_text(encoding="utf-8"))
    runtime_load["camera_delivery"][
        "edge_bounded_source_stamp_integrity"
    ] = _edge_bounded_integrity(
        leading_missing=(gate.STRICT_CAMERA_INFO_TOPICS[0],),
        trailing_missing=(gate.STRICT_CAMERA_INFO_TOPICS[-1],),
    )
    _write(path, runtime_load)

    result = gate.evaluate_trial(attempt, source)

    assert result["evidence_integrity_status"] == "PASS"
    assert result["camera_transport_qualification_status"] == "PASS"
    assert result["simulation_pilot_acceptance_status"] == "PASS"


@pytest.mark.parametrize(
    ("path", "replacement"),
    (
        (("status",), "FAIL"),
        (("boundary_trim", "maximum_incomplete_union_stamp_count_per_edge"), 2),
        (("retained_interior", "incomplete_union_stamp_count"), 1),
        (("retained_interior", "positive_record_count"), 599),
        (("failures",), [{"check": "tampered"}]),
    ),
)
def test_strict_v2_rejects_tampered_edge_bounded_whole_bag_proof(
    tmp_path: Path,
    path: tuple[str, ...],
    replacement: object,
) -> None:
    attempt, source = _fixture(tmp_path, strict_10hz=True)
    report_path = attempt / "runtime_load_analysis.json"
    runtime_load = json.loads(report_path.read_text(encoding="utf-8"))
    cursor = runtime_load["camera_delivery"][
        "edge_bounded_source_stamp_integrity"
    ]
    for key in path[:-1]:
        cursor = cursor[key]
    cursor[path[-1]] = replacement
    _write(report_path, runtime_load)

    result = gate.evaluate_trial(attempt, source)

    assert result["camera_transport_qualification_status"] == "FAILED"
    assert any(
        "recorder edges" in failure
        for failure in result["camera_transport_qualification"]["failures"]
    )


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
