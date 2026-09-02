from __future__ import annotations

import json
from pathlib import Path
import shutil
import subprocess

from PIL import Image
import pytest
import yaml

from scripts.e2e import carla_basicagent_sweep_report as sweep_report
from scripts.e2e import publish_validation_assets as publisher


ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "scripts/e2e/carla_expert_suite.yaml"


def _write_json(path: Path, payload) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")


def _write_complete_smoke(artifact_root: Path) -> None:
    job_root = artifact_root / "maps/town01/smoke/job"
    episode = job_root / "episode"
    export = job_root / "export"
    preview = job_root / "preview"
    for directory in (episode, export, preview):
        directory.mkdir(parents=True, exist_ok=True)

    samples = []
    states = []
    for index in range(2):
        frame = 100 + index
        timestamp_ns = index * 1_000_000_000
        states.append(
            {
                "timestamp_ns": timestamp_ns,
                "x": float(index),
                "y": 0.0,
                "z": 0.0,
                "yaw": 0.0,
            }
        )
        cameras = {}
        for camera_index, camera_name in enumerate(publisher.CAMERA_NAMES):
            relative = f"images/{camera_name}/{frame}.jpg"
            image_path = episode / relative
            image_path.parent.mkdir(parents=True, exist_ok=True)
            Image.new(
                "RGB",
                (96, 54),
                (30 + camera_index * 25, 60 + index * 30, 120),
            ).save(image_path)
            cameras[camera_name] = {"path": relative}
        samples.append(
            {
                "frame": frame,
                "timestamp_ns": timestamp_ns,
                "town": "Town01",
                "weather": "ClearNoon",
                "vad_command_0based": 3,
                "route_progress_m": float(index),
                "route_cte_m": 0.05,
                "pose_map_xyz_yaw": [float(index), 0.0, 0.0, 0.0],
                "future_expert": {
                    "positions_xy": [[0.5, 0.0], [1.0, 0.0], [1.5, 0.0]]
                },
                "cameras": cameras,
            }
        )
    _write_json(
        episode / "route.json",
        {
            "town": "Town01",
            "weather": "ClearNoon",
            "route": [{"x": 0.0, "y": 0.0}, {"x": 2.0, "y": 0.0}],
        },
    )
    (episode / "states.jsonl").write_text(
        "".join(json.dumps(item) + "\n" for item in states), encoding="utf-8"
    )
    (export / "samples.jsonl").write_text(
        "".join(json.dumps(item) + "\n" for item in samples), encoding="utf-8"
    )
    _write_json(
        episode / "manifest.json",
        {
            "status": "complete",
            "runtime": {
                "client_map_loading_allowed": False,
                "client_map_loading_performed": False,
            },
            "result": {
                "goal_reached": True,
                "collision_event_count": 0,
                "lane_invasion_event_count": 0,
            },
        },
    )
    _write_json(
        export / "manifest.json",
        {
            "status": "validated",
            "sample_count": 2,
            "maximum_route_cte_m": 0.05,
            "collision_event_count": 0,
            "lane_invasion_event_count": 0,
        },
    )
    Image.new("RGB", (32, 18), "navy").save(preview / "overview.png")
    Image.new("RGB", (32, 18), "navy").save(
        preview / "drive.gif", save_all=True
    )
    _write_json(
        artifact_root / "maps/town01/smoke/collection_plan.json",
        {
            "status": "COMPLETE",
            "jobs": [
                {
                    "status": "COMPLETE",
                    "map_id": "town01",
                    "scenario": "lane_follow",
                    "weather": "ClearNoon",
                    "seed": 0,
                    "route_id": "route_a",
                    "paths": {
                        "episode": str(episode),
                        "export": str(export),
                        "preview_png": str(preview / "overview.png"),
                        "preview_gif": str(preview / "drive.gif"),
                    },
                }
            ],
            "server": {
                "map_load_allowed": False,
                "map_lifecycle_managed": False,
            },
        },
    )
    _write_json(
        artifact_root / "maps/town01/catalog/route_catalog.json",
        {
            "status": "complete",
            "map_id": "town01",
            "server": {
                "map_load_allowed": False,
                "map_load_performed": False,
            },
            "generation": {"seeds": [0]},
            "routes": [
                {"id": "route_a", "scenario": "lane_follow", "seed": 0}
            ],
        },
    )


def _write_labeled_vad_trial(
    directory: Path,
    *,
    scenario: str,
    start_spawn_index: int,
    goal_spawn_index: int,
    town: str = "Town01",
    captured_at: str = "2026-08-31T12:00:00+09:00",
) -> None:
    route = directory / "aligned_route.json"
    route_payload = {
        "town": town,
        "scenario": scenario,
        "route_length_m": 42.5,
        "start_spawn_index": start_spawn_index,
        "goal_spawn_index": goal_spawn_index,
    }
    _write_json(route, route_payload)
    _write_json(directory / "source_route.json", route_payload)
    _write_json(
        directory / "result.json",
        {
            "success": True,
            "execution_mode": "full_stack",
            "route_file": str(route),
            "assessment": {
                "planning_architecture": "vad_route_manager_hybrid",
                "route_completion": "PASS",
            },
            "final": {"goal_reached": True, "route_status": "goal_reached"},
        },
    )
    for name in (
        "route_result.png",
        "path_vs_control.png",
        "steering_tracking.png",
    ):
        Image.new("RGB", (320, 180), "green").save(directory / name)
    Image.new("RGB", (320, 180), "green").save(
        directory / "turn_path_control.gif", save_all=True
    )
    Image.new("RGB", (1920, 1080), "navy").save(
        directory / "autoware_rviz_fullscreen.png"
    )
    Image.new("RGB", (960, 540), "navy").save(
        directory / "autoware_rviz_drive.gif", save_all=True
    )
    _write_json(
        directory / "desktop_capture.json",
        {
            "schema_version": 1,
            "candidate_observed": True,
            "candidate_topic": "/planning/vad/candidate_trajectories",
            "capture_started_after_candidate": True,
            "display": ":99.0",
            "source_dimensions": [1920, 1080],
            "png_dimensions": [1920, 1080],
            "gif_dimensions": [960, 540],
            "png_file": "autoware_rviz_fullscreen.png",
            "gif_file": "autoware_rviz_drive.gif",
            "captured_at": captured_at,
        },
    )
    _write_json(directory / "diagnosis.json", {"status": "complete"})
    _write_json(
        directory / "latency/e2e_latency.json", {"status": "complete"}
    )


def _write_centered_capture_evidence(
    directory: Path,
    *,
    speed_contract: dict | None = None,
    trial_id: str | None = None,
    scenario: str | None = None,
) -> dict[str, str]:
    Image.new("RGB", (1920, 1080), "yellow").save(
        directory / "autoware_rviz_candidate.png"
    )
    (directory / "autoware_rviz_capture.mkv").write_bytes(b"fixture-recording")
    provenance = directory / "rviz_capture_provenance"
    provenance.mkdir(parents=True, exist_ok=True)
    config = provenance / "autoware_vad_carla.rviz"
    shutil.copy2(
        ROOT / "autoware_e2e_vad_launch/rviz/autoware_vad_carla.rviz",
        config,
    )
    config_sha256 = publisher._sha256(config)
    (provenance / "SHA256SUMS").write_text(
        f"{config_sha256}  autoware_vad_carla.rviz\n", encoding="utf-8"
    )
    runtime_values: dict[str, object] = {
        "RECOMMENDED": "true",
        "VISUALIZE": "true",
        "CAPTURE_DESKTOP": "true",
        "RVIZ_CAPTURE_CONFIG": str(config.resolve()),
        "RVIZ_CAPTURE_CONFIG_SHA256": config_sha256,
    }
    if speed_contract is not None:
        assert trial_id in {"straight", "turn"}
        assert scenario in {"straight", "left", "right"}
        trial_contract = speed_contract["trials"][trial_id]
        parameters = speed_contract["route_manager_parameters"]
        gate = speed_contract["vehicle_cmd_gate"]
        controller = speed_contract["longitudinal_controller"]
        speed_provenance = directory / "speed_profile_provenance"
        speed_provenance.mkdir(exist_ok=True)
        source_files = {
            "vehicle_cmd_gate.param.yaml": ROOT
            / "autoware_e2e_vad_launch/config/vehicle_cmd_gate_carla_30kph.param.yaml",
            "vehicle_cmd_gate.param.yaml.metadata.json": ROOT
            / "autoware_e2e_vad_launch/config/vehicle_cmd_gate_carla_30kph.param.yaml.metadata.json",
            "longitudinal_controller.param.yaml": ROOT
            / "autoware_e2e_vad_launch/config/pid_carla_vad_30kph.param.yaml",
            "longitudinal_controller.param.yaml.metadata.json": ROOT
            / "autoware_e2e_vad_launch/config/pid_carla_vad_30kph.param.yaml.metadata.json",
        }
        for name, source in source_files.items():
            shutil.copy2(source, speed_provenance / name)
        speed_sha256 = {
            name: publisher._sha256(speed_provenance / name)
            for name in source_files
        }
        (speed_provenance / "SHA256SUMS").write_text(
            "".join(
                f"{digest}  {name}\n"
                for name, digest in speed_sha256.items()
            ),
            encoding="utf-8",
        )
        (directory / "vad_route_manager.params.yaml").write_text(
            yaml.safe_dump(
                {"/vad_route_manager": {"ros__parameters": parameters}}
            ),
            encoding="utf-8",
        )
        (directory / "vehicle_cmd_gate.params.yaml").write_text(
            "fixture: vehicle_cmd_gate\n", encoding="utf-8"
        )
        (directory / "controller.params.yaml").write_text(
            "fixture: longitudinal_controller\n", encoding="utf-8"
        )
        runtime_values.update(
            {
                "VSCODE_SNAP_GUI_ENV_SANITIZED": "false",
                "SPEED_30KPH": "true",
                "TIGHT_CORRIDOR_CANDIDATE": "false",
                "TRAJECTORY_STABILITY_CANDIDATE": "false",
                "SMART_MPC": "false",
                "FP16_HEADS": "false",
                "SPEED_PROFILE_ID": speed_contract["profile_id"],
                "ROUTE_SCENARIO": scenario,
                "SPEED_EXPOSURE_MODE": trial_contract["exposure_mode"],
                "LONGITUDINAL_SPEED_SOURCE": speed_contract[
                    "longitudinal_speed_source"
                ],
                "LONGITUDINAL_ACCELERATION_ROLE": speed_contract[
                    "longitudinal_acceleration_role"
                ],
                "VAD_GEOMETRY_SOURCE": "true",
                "VAD_VELOCITY_EVALUATED": "false",
                "VAD_GEOMETRY_EVALUATED": "true",
                "VAD_CRUISE_VELOCITY_EVALUATED": "false",
                "VAD_HARD_STOP_SENTINEL_PRESERVED": "true",
                "VAD_IMU_ACCELERATION_ENABLED": "true",
                "CLOSED_LOOP_VALIDATION_STATE": speed_contract[
                    "validation_state"
                ],
                "SPEED_LIMIT_SOURCE": gate["speed_limit_source"],
                "REAL_VEHICLE_READY": "false",
                "TARGET_SPEED_MPS": speed_contract["target_speed_mps"],
                "TARGET_SPEED_KPH": 30.0,
                "MINIMUM_SUSTAINED_SPEED_MPS": trial_contract[
                    "minimum_sustained_speed_mps"
                ],
                "MINIMUM_SUSTAINED_SPEED_SEC": trial_contract[
                    "minimum_sustained_speed_sec"
                ],
                "MAXIMUM_OBSERVED_SPEED_MPS": speed_contract[
                    "maximum_observed_speed_mps"
                ],
                "MAXIMUM_SPEED_SAMPLE_GAP_SEC": speed_contract[
                    "maximum_speed_sample_gap_sec"
                ],
                "MAXIMUM_LATERAL_ACCELERATION_LIMIT_MPS2": trial_contract[
                    "maximum_lateral_acceleration_mps2"
                ],
                "MAXIMUM_LONGITUDINAL_ACCELERATION_MPS2": parameters[
                    "maximum_longitudinal_acceleration_mps2"
                ],
                "COMMAND_GATE_NOMINAL_LONGITUDINAL_ACCELERATION_MPS2": gate[
                    "longitudinal_acceleration_limit_mps2"
                ],
                "MAXIMUM_LATERAL_ACCELERATION_MPS2": parameters[
                    "maximum_lateral_acceleration_mps2"
                ],
                "CONTROLLER_STOP_OFFSET_M": parameters[
                    "controller_stop_offset_m"
                ],
                "MANEUVER_LOOKAHEAD_M": parameters["maneuver_lookahead_m"],
                "MANEUVER_EXIT_LOOKAHEAD_M": parameters[
                    "maneuver_exit_lookahead_m"
                ],
                "ROUTE_CURVATURE_LOOKAHEAD_M": parameters[
                    "route_curvature_lookahead_m"
                ],
                "CURVATURE_SPEED_PREVIEW_M": parameters[
                    "curvature_speed_preview_m"
                ],
                "MAX_ROUTE_DEVIATION_M": parameters["max_route_deviation_m"],
                "MAX_CANDIDATE_AGE_SEC": parameters["max_candidate_age_sec"],
                "CANDIDATE_TIMEOUT_SEC": parameters["candidate_timeout_sec"],
                "COMFORTABLE_DECELERATION_MPS2": parameters[
                    "comfortable_deceleration_mps2"
                ],
                "LONGITUDINAL_PID_MAX_OUT_MPS2": controller[
                    "maximum_output_mps2"
                ],
                "LONGITUDINAL_PID_MAX_P_EFFORT_MPS2": controller[
                    "maximum_proportional_effort_mps2"
                ],
                "VEHICLE_CMD_GATE_PARAM_SHA256": speed_sha256[
                    "vehicle_cmd_gate.param.yaml"
                ],
                "VEHICLE_CMD_GATE_METADATA_SHA256": speed_sha256[
                    "vehicle_cmd_gate.param.yaml.metadata.json"
                ],
                "LONGITUDINAL_CONTROLLER_PARAM_SHA256": speed_sha256[
                    "longitudinal_controller.param.yaml"
                ],
                "LONGITUDINAL_CONTROLLER_METADATA_SHA256": speed_sha256[
                    "longitudinal_controller.param.yaml.metadata.json"
                ],
            }
        )
    (directory / "runtime.env").write_text(
        "".join(f"{key}={value}\n" for key, value in runtime_values.items()),
        encoding="utf-8",
    )
    capture_path = directory / "desktop_capture.json"
    capture = json.loads(capture_path.read_text(encoding="utf-8"))
    capture.update(
        {
            "candidate_observed_at": "2026-08-31T04:59:55+00:00",
            "candidate_png_dimensions": [1920, 1080],
            "candidate_png_file": "autoware_rviz_candidate.png",
            "candidate_still_captured_at": "2026-08-31T04:59:57+00:00",
            "recording_file": "autoware_rviz_capture.mkv",
            "recording_started_at": "2026-08-31T04:59:58+00:00",
            "route_evaluation_started_at": "2026-08-31T05:00:00+00:00",
            "route_evaluation_finished_at": "2026-08-31T05:01:00+00:00",
            "captured_at": "2026-08-31T05:00:30+00:00",
            "representative_frame": {
                "captured_at": "2026-08-31T05:00:30+00:00",
                "offset_sec": 32.0,
                "recording_duration_sec": 64.0,
                "selection": "route_evaluation_midpoint",
                "source": "autoware_rviz_capture.mkv",
            },
            "rviz_view_contract": {
                "angle_rad": 0.0,
                "center_xy_m": [0.0, 0.0],
                "config_file": (
                    "rviz_capture_provenance/autoware_vad_carla.rviz"
                ),
                "config_sha256": config_sha256,
                "controller": "rviz_default_plugins/TopDownOrtho",
                "scale": 10.0,
                "target_frame": "base_link",
                "vehicle_centered": True,
                "visible_path_topics": [
                    "/planning/trajectory",
                    "/planning/vad/candidate_trajectories",
                    "/planning/vad_route/actual_path",
                    "/planning/vad_route/reference_path",
                    "/planning/vad_route/selected_raw_trajectory",
                ],
                "visual_clarity": {
                    "odometry_display": "Kinematic State",
                    "odometry_keep": 1,
                    "odometry_covariance": False,
                    "odometry_orientation": False,
                    "odometry_position": False,
                    "candidate_path_alpha": 0.22,
                    "candidate_path_width": 0.04,
                },
            },
        }
    )
    _write_json(capture_path, capture)
    if speed_contract is None:
        return {}
    return {
        "runtime_env_sha256": publisher._sha256(directory / "runtime.env"),
        "route_manager_parameter_dump_sha256": publisher._sha256(
            directory / "vad_route_manager.params.yaml"
        ),
        "vehicle_cmd_gate_parameter_dump_sha256": publisher._sha256(
            directory / "vehicle_cmd_gate.params.yaml"
        ),
        "longitudinal_controller_parameter_dump_sha256": publisher._sha256(
            directory / "controller.params.yaml"
        ),
        "gate_provenance_sha256": speed_contract["vehicle_cmd_gate"][
            "parameter_sha256"
        ],
        "gate_metadata_sha256": speed_contract["vehicle_cmd_gate"][
            "metadata_sha256"
        ],
        "longitudinal_controller_provenance_sha256": speed_contract[
            "longitudinal_controller"
        ]["parameter_sha256"],
        "longitudinal_controller_metadata_sha256": speed_contract[
            "longitudinal_controller"
        ]["metadata_sha256"],
    }


def _write_centered_vad_visual_refresh(
    directory: Path, selected_directory: Path
) -> None:
    selected_route = json.loads(
        (selected_directory / "source_route.json").read_text(encoding="utf-8")
    )
    _write_labeled_vad_trial(
        directory,
        scenario=selected_route["scenario"],
        start_spawn_index=selected_route["start_spawn_index"],
        goal_spawn_index=selected_route["goal_spawn_index"],
        town=selected_route["town"],
        captured_at="2026-08-31T05:00:30+00:00",
    )
    shutil.copy2(
        selected_directory / "source_route.json", directory / "source_route.json"
    )
    _write_centered_capture_evidence(directory)


def _write_terminal_vad_matrix(
    artifact_root: Path,
    snapshot,
    *,
    aggregate_status: str = "COMPLETE",
    runnable_map_ids: frozenset[str] = frozenset({"town01"}),
) -> Path:
    matrix_root = artifact_root / "autoware_vad_town_matrix"
    matrix_id = "fixture_straight_turn"
    runtime_profile = {
        "id": "fixture_recommended_visualized",
        "wrapper_options": [
            "--recommended",
            "--visualize",
            "--capture-desktop",
        ],
        "client_map_loading_allowed": False,
    }
    route_contract = {
        "trials": [
            {"id": "straight", "catalog_scenarios": ["straight"]},
            {"id": "turn", "catalog_scenarios": ["left", "right"]},
        ]
    }
    plan_maps = []
    statuses = []
    for entry in snapshot["maps"]:
        map_id = entry["map_id"]
        canonical_name = entry["canonical_name"]
        runnable = map_id in runnable_map_ids
        plan_maps.append(
            {
                "map_id": map_id,
                "canonical_name": canonical_name,
                "runnable": runnable,
            }
        )
        trials = {}
        for trial_id, scenario in (("straight", "straight"), ("turn", "left")):
            if runnable:
                attempt = (
                    matrix_root
                    / "maps"
                    / map_id
                    / "trials"
                    / trial_id
                    / "attempt_001"
                )
                _write_labeled_vad_trial(
                    attempt,
                    scenario=scenario,
                    start_spawn_index=1,
                    goal_spawn_index=2,
                    town=canonical_name,
                )
                validation = {
                    "schema_version": 1,
                    "status": "PASS",
                    "matrix_id": matrix_id,
                    "map_id": map_id,
                    "trial_id": trial_id,
                    "catalog_scenario": scenario,
                    "trial_directory": str(attempt.resolve()),
                    "runtime_profile": runtime_profile,
                    "result": {
                        "success": True,
                        "execution_mode": "full_stack",
                        "planning_architecture": "vad_route_manager_hybrid",
                        "route_completion": "PASS",
                        "goal_reached": True,
                    },
                }
                validation_path = attempt / "matrix_validation.json"
                _write_json(validation_path, validation)
                trials[trial_id] = {
                    "status": "PASS",
                    "reason": "fixture passed",
                    "attempt_directory": str(attempt.resolve()),
                    "validation": str(validation_path.resolve()),
                }
            else:
                trials[trial_id] = {
                    "status": "BLOCKED",
                    "reason": "fixture has no admitted bundle",
                    "attempt_directory": None,
                    "validation": None,
                }
        status = {
            "schema_version": 1,
            "matrix_id": matrix_id,
            "map_id": map_id,
            "canonical_name": canonical_name,
            "runnable": runnable,
            "status": "PASS" if runnable else "BLOCKED",
            "stage": "complete" if runnable else "admission",
            "reason": "fixture terminal status",
            "block_code": None if runnable else "fixture_blocked",
            "updated_at": "2026-08-31T03:30:00+00:00",
            "trials": trials,
        }
        _write_json(matrix_root / "maps" / map_id / "status.json", status)
        statuses.append(status)
    _write_json(
        matrix_root / "matrix_plan.json",
        {
            "schema_version": 1,
            "matrix_id": matrix_id,
            "generated_at": "2026-08-31T02:00:00+00:00",
            "canonical_map_count": len(plan_maps),
            "runnable_map_count": len(runnable_map_ids),
            "runtime_profile": runtime_profile,
            "route_contract": route_contract,
            "maps": plan_maps,
        },
    )
    _write_json(
        matrix_root / "aggregate.json",
        {
            "schema_version": 1,
            "matrix_id": matrix_id,
            "generated_at": "2026-08-31T04:00:00+00:00",
            "status": aggregate_status,
            "canonical_map_count": len(statuses),
            "runnable_map_count": len(runnable_map_ids),
            "runnable_pass_count": len(runnable_map_ids),
            "blocked_map_count": len(statuses) - len(runnable_map_ids),
            "status_counts": {
                "BLOCKED": len(statuses) - len(runnable_map_ids),
                "PASS": len(runnable_map_ids),
            },
            "runtime_profile": runtime_profile,
            "route_contract": route_contract,
            "maps": statuses,
        },
    )
    return matrix_root


def _speed_30kph_runtime_profile() -> dict:
    manifest = yaml.safe_load(
        (ROOT / "scripts/e2e/autoware_vad_town_matrix.yaml").read_text(
            encoding="utf-8"
        )
    )
    return dict(manifest["runtime_profiles"]["speed_30kph"])


def _camera_source_5hz_runtime_profile() -> dict:
    manifest = yaml.safe_load(
        (ROOT / "scripts/e2e/autoware_vad_town_matrix.yaml").read_text(
            encoding="utf-8"
        )
    )
    return dict(
        manifest["runtime_profiles"][publisher.CAMERA_SOURCE_5HZ_SELECTOR]
    )


def _write_speed_profile_evidence(directory: Path) -> dict[str, str]:
    result_path = directory / "result.json"
    result = json.loads(result_path.read_text(encoding="utf-8"))
    context = {
        "longitudinal_velocity_source": "explicit_simulation_nominal",
        "vad_geometry_evaluated": True,
        "vad_velocity_evaluated": False,
    }
    result["profile_context"] = context
    result["speed_exposure"] = {"status": "PASS", **context}
    _write_json(result_path, result)
    route_path = Path(result["route_file"]).resolve()
    route = json.loads(route_path.read_text(encoding="utf-8"))
    bag = directory / "bag"
    bag.mkdir(exist_ok=True)
    (bag / "metadata.yaml").write_text(
        "rosbag2_bagfile_information:\n  version: 9\n", encoding="utf-8"
    )
    (bag / "fixture.db3").write_bytes(b"publisher-speed-fixture")
    bag_files = [
        {
            "path": path.relative_to(bag).as_posix(),
            "size_bytes": path.stat().st_size,
            "sha256": publisher._sha256(path),
        }
        for path in sorted(bag.rglob("*"))
        if path.is_file()
    ]
    bag_manifest = {
        "schema_version": 1,
        "root": str(bag.resolve()),
        "files": bag_files,
    }
    bag_manifest["sha256"] = publisher._sha256_json(
        {"schema_version": 1, "files": bag_files}
    )
    trial_id = "straight" if route["scenario"] == "straight" else "turn"
    identity = {
        "schema_version": 1,
        "effective_route": {
            "path": str(route_path),
            "sha256": publisher._sha256(route_path),
            "town": route["town"],
            "scenario": route["scenario"],
            "trial_id": trial_id,
            "route_length_m": route["route_length_m"],
        },
        "route_result": {
            "path": str(result_path.resolve()),
            "sha256": publisher._sha256(result_path),
            "success": True,
            "execution_mode": "full_stack",
            "profile_context": context,
        },
        "rosbag": bag_manifest,
    }
    identity["sha256"] = publisher._sha256_json(identity)
    required_series = [
        "raw_selected_vad",
        "explicit_overlaid_planning",
        "gated_control_command",
        "actual_odometry",
    ]
    _write_json(
        directory / "speed_profile.json",
        {
            "schema_version": 1,
            "analysis": "carla_speed_source_evidence",
            "status": "complete",
            "inputs": {
                "bag": str(bag.resolve()),
                "profile_id": publisher.SPEED_30KPH_PROFILE_ID,
                "longitudinal_speed_source": "explicit_simulation_nominal",
                "target_speed_mps": 8.333333333333334,
            },
            "source_identity": identity,
            "interpretation": {
                "cruise_velocity_source": "explicit CARLA simulation profile",
                "planning_geometry": "VAD route-manager hybrid",
                "raw_vad_velocity_is_cruise_target": False,
                "real_vehicle_ready": False,
            },
            "outputs": {
                "json": "speed_profile.json",
                "plot": "speed_profile.png",
            },
            "quality": {"problems": [], "required_series": required_series},
            "series": {name: [{"time_sec": 0.0, "speed_mps": 0.0}] for name in required_series},
        },
    )
    Image.new("RGB", (640, 360), "purple").save(directory / "speed_profile.png")
    return {
        "speed_profile_json_sha256": publisher._sha256(
            directory / "speed_profile.json"
        ),
        "speed_profile_plot_sha256": publisher._sha256(
            directory / "speed_profile.png"
        ),
        "speed_profile_source_identity_sha256": identity["sha256"],
        "speed_profile_result_sha256": identity["route_result"]["sha256"],
        "speed_profile_route_sha256": identity["effective_route"]["sha256"],
        "speed_profile_bag_manifest_sha256": bag_manifest["sha256"],
    }


def _write_speed_lifecycle_evidence(
    directory: Path, *, map_id: str, trial_id: str, canonical_name: str
) -> dict:
    generation_id = f"{map_id}_{trial_id}_{directory.name}"
    owner_pid = 4242
    server_log = directory / "carla_server.log"
    server_log.write_text("fixture CARLA generation completed\n", encoding="utf-8")
    for file_name, stage, mode in (
        ("carla_preflight_health.json", "trial_preflight", "running"),
        ("carla_completion_health.json", "trial_completion", "running"),
        ("carla_cleanup_health.json", "trial_cleanup", "stopped"),
    ):
        health = {
            "schema_version": 1,
            "status": "PASS",
            "stage": stage,
            "mode": mode,
            "generation_id": generation_id,
            "expected_map": canonical_name,
            "owner_pid": owner_pid,
            "owner_pgid": owner_pid,
            "read_only": True,
        }
        if mode == "stopped":
            health["port_released"] = True
        _write_json(directory / file_name, health)
    return {
        "schema_version": 1,
        "status": "PASS",
        "lifecycle": "cold_start_owned_process_group_per_trial",
        "generation_id": generation_id,
        "expected_map": canonical_name,
        "owner_pid": owner_pid,
        "owner_pgid": owner_pid,
        "server_log": {
            "path": str(server_log.resolve()),
            "size_bytes": server_log.stat().st_size,
            "sha256": publisher._sha256(server_log),
        },
        "preflight_health_sha256": publisher._sha256(
            directory / "carla_preflight_health.json"
        ),
        "completion_health_sha256": publisher._sha256(
            directory / "carla_completion_health.json"
        ),
        "cleanup_health_sha256": publisher._sha256(
            directory / "carla_cleanup_health.json"
        ),
        "post_completion_exit_policy": (
            "completion RPC PASS preserves a completed drive; the next trial "
            "always receives a new cold-start generation"
        ),
    }


def _speed_visual_evidence_binding(directory: Path) -> dict:
    return {
        "schema_version": 1,
        "binding": "matrix_validation_sha256_v1",
        "fullscreen_dimensions": [1920, 1080],
        "candidate_dimensions": [1920, 1080],
        "drive_gif_dimensions": [960, 540],
        "vehicle_centered": True,
        "target_frame": "base_link",
        "files": {
            name: {
                "size_bytes": (directory / name).stat().st_size,
                "sha256": publisher._sha256(directory / name),
            }
            for name in publisher.SPEED_30KPH_VISUAL_EVIDENCE_NAMES
        },
    }


def _convert_terminal_matrix_to_speed_30kph(matrix_root: Path) -> None:
    profile = _speed_30kph_runtime_profile()
    plan_path = matrix_root / "matrix_plan.json"
    aggregate_path = matrix_root / "aggregate.json"
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    aggregate = json.loads(aggregate_path.read_text(encoding="utf-8"))
    plan["runtime_profile_selector"] = "speed_30kph"
    plan["runtime_profile"] = profile
    aggregate["runtime_profile_selector"] = "speed_30kph"
    aggregate["runtime_profile"] = profile
    rviz_path = ROOT / "autoware_e2e_vad_launch/rviz/autoware_vad_carla.rviz"
    campaign_contract = {
        "schema_version": 1,
        "hash_algorithm": "sha256",
        "repository_root": ".",
        "files": [
            {
                "path": "autoware_e2e_vad_launch/rviz/autoware_vad_carla.rviz",
                "sha256": publisher._sha256(rviz_path),
            }
        ],
    }
    campaign_sha256 = publisher._sha256_json(campaign_contract)
    admission_sha256 = "a" * 64
    plan["campaign_execution_contract"] = campaign_contract
    plan["campaign_execution_contract_sha256"] = campaign_sha256
    plan["admission_contract_sha256"] = admission_sha256
    aggregate["campaign_execution_contract_sha256"] = campaign_sha256
    aggregate["admission_contract_sha256"] = admission_sha256
    for status in aggregate["maps"]:
        if status["status"] != "PASS":
            continue
        for trial_id, trial in status["trials"].items():
            attempt = Path(trial["attempt_directory"])
            validation_path = Path(trial["validation"])
            validation = json.loads(validation_path.read_text(encoding="utf-8"))
            validation["runtime_profile_selector"] = "speed_30kph"
            validation["runtime_profile"] = profile
            validation["campaign_execution_contract_sha256"] = campaign_sha256
            validation["admission_contract_sha256"] = admission_sha256
            if trial_id == "straight":
                for name in ("aligned_route.json", "source_route.json"):
                    route_path = attempt / name
                    route = json.loads(route_path.read_text(encoding="utf-8"))
                    route.pop("start_spawn_index")
                    route.pop("goal_spawn_index")
                    route.update(
                        {
                            "endpoint_source": "generated_waypoints",
                            "endpoint_waypoint_spacing_m": 10.0,
                            "start_endpoint_index": 11,
                            "goal_endpoint_index": 22,
                            "spawn_height_contract": {
                                "offset_owner": "autoware_carla_interface_bridge",
                                "bridge_z_offset_m": 2.0,
                                "catalog_z_offset_m": 0.0,
                            },
                        }
                    )
                    _write_json(route_path, route)
            speed_bindings = _write_speed_profile_evidence(attempt)
            route = json.loads(
                (attempt / "aligned_route.json").read_text(encoding="utf-8")
            )
            validation["turn_direction"] = (
                route["scenario"] if trial_id == "turn" else None
            )
            runtime_bindings = _write_centered_capture_evidence(
                attempt,
                speed_contract=profile["speed_contract"],
                trial_id=trial_id,
                scenario=route["scenario"],
            )
            validation["speed_contract"] = {
                "status": "PASS",
                "profile_id": publisher.SPEED_30KPH_PROFILE_ID,
                "maximum_observed_speed_mps": 7.74,
                **speed_bindings,
                **runtime_bindings,
            }
            validation["desktop_capture"] = json.loads(
                (attempt / "desktop_capture.json").read_text(encoding="utf-8")
            )
            validation["carla_lifecycle"] = _write_speed_lifecycle_evidence(
                attempt,
                map_id=status["map_id"],
                trial_id=trial_id,
                canonical_name=status["canonical_name"],
            )
            validation["visual_evidence"] = _speed_visual_evidence_binding(attempt)
            _write_json(validation_path, validation)
    _write_json(plan_path, plan)
    _write_json(aggregate_path, aggregate)


def _mark_terminal_matrix_straight_failed(matrix_root: Path) -> None:
    status_path = matrix_root / "maps/town01/status.json"
    status = json.loads(status_path.read_text(encoding="utf-8"))
    failed_attempt = matrix_root / "maps/town01/trials/straight/attempt_002"
    _write_labeled_vad_trial(
        failed_attempt,
        scenario="straight",
        start_spawn_index=30,
        goal_spawn_index=40,
    )
    failed_result_path = failed_attempt / "result.json"
    failed_result = json.loads(failed_result_path.read_text(encoding="utf-8"))
    failed_result["success"] = False
    failed_result["assessment"]["route_completion"] = "FAIL"
    failed_result["final"] = {"goal_reached": False, "route_status": "ready"}
    _write_json(failed_result_path, failed_result)
    status.update(
        {
            "status": "FAILED",
            "stage": "straight_failed",
            "reason": "The original catalog straight trial failed strict validation.",
        }
    )
    status["trials"]["straight"] = {
        "status": "FAILED",
        "reason": "Original straight attempt failed; retain this matrix result.",
        "attempt_directory": str(failed_attempt.resolve()),
        "validation": None,
    }
    _write_json(status_path, status)

    aggregate_path = matrix_root / "aggregate.json"
    aggregate = json.loads(aggregate_path.read_text(encoding="utf-8"))
    aggregate.update(
        {
            "status": "FAILED",
            "runnable_pass_count": 0,
            "status_counts": {"BLOCKED": 18, "FAILED": 1},
        }
    )
    aggregate["maps"] = [
        status if entry["map_id"] == "town01" else entry
        for entry in aggregate["maps"]
    ]
    _write_json(aggregate_path, aggregate)


def _snapshot(artifact_root: Path, complete: bool = False):
    manifest, _ = sweep_report.load_manifest(MANIFEST)
    sweep_report.initialize_statuses(
        artifact_root, manifest, None, ("town01",), "127.0.0.1", 2100
    )
    if complete:
        _write_complete_smoke(artifact_root)
        sweep_report.update_status(
            artifact_root,
            "town01",
            "PASS",
            "complete",
            "BasicAgent evidence validated; not Autoware VAD evidence.",
            None,
        )
    else:
        sweep_report.update_status(
            artifact_root,
            "town01",
            "FAILED",
            "failed",
            "Synthetic terminal status without publishable evidence.",
            1,
        )
    aggregate, markdown = sweep_report.build_summary(artifact_root, manifest)
    _write_json(artifact_root / "aggregate.json", aggregate)
    (artifact_root / "SUMMARY.md").write_text(markdown, encoding="utf-8")
    return publisher.load_sweep_snapshot(
        artifact_root, expected_selected_map_count=1
    )


def test_dashboard_uses_all_19_matching_aggregate_status_records(tmp_path: Path) -> None:
    snapshot = _snapshot(tmp_path / "artifacts")
    output = tmp_path / "dashboard.png"

    publisher.render_status_dashboard(snapshot, output)

    with Image.open(output) as image:
        assert image.size == (1920, 1080)
    assert len(snapshot["maps"]) == 19
    assert publisher.collect_basicagent_runs(snapshot) == []


def test_stale_aggregate_is_rejected(tmp_path: Path) -> None:
    artifact_root = tmp_path / "artifacts"
    _snapshot(artifact_root)
    status_path = artifact_root / "maps/town01/status.json"
    status = json.loads(status_path.read_text(encoding="utf-8"))
    status["stage"] = "changed_after_summary"
    _write_json(status_path, status)

    with pytest.raises(publisher.PublicationError, match="aggregate/status mismatch"):
        publisher.load_sweep_snapshot(
            artifact_root, expected_selected_map_count=1
        )


def test_verified_source_archive_fails_closed_on_digest_mismatch(
    tmp_path: Path,
) -> None:
    source = tmp_path / "source.json"
    destination = tmp_path / "archive.json"
    source.write_text('{"status":"changed"}\n', encoding="utf-8")

    with pytest.raises(publisher.PublicationError, match="changed during publication"):
        publisher._copy_verified_source(
            source,
            destination,
            "0" * 64,
            "fixture aggregate",
        )


def test_publish_staging_rejects_manifest_and_symlink_path_escape(
    tmp_path: Path,
) -> None:
    destination = tmp_path / "publication"
    destination.mkdir()
    victim = tmp_path / "victim.txt"
    victim.write_text("keep", encoding="utf-8")
    stale = destination / "stale.txt"
    stale.write_text("remove", encoding="utf-8")
    _write_json(
        destination / "publication_manifest.json",
        {
            "generated_files": [
                str(victim),
                "../victim.txt",
                "stale.txt",
            ]
        },
    )
    staging = tmp_path / "staging"
    staging.mkdir()
    (staging / "README.md").write_text("new", encoding="utf-8")

    publisher._publish_staging(staging, destination, [])

    assert victim.read_text(encoding="utf-8") == "keep"
    assert not stale.exists()
    assert (destination / "README.md").read_text(encoding="utf-8") == "new"

    outside = tmp_path / "outside"
    outside.mkdir()
    (destination / "escape").symlink_to(outside, target_is_directory=True)
    escaped_staging = tmp_path / "escaped_staging"
    (escaped_staging / "escape").mkdir(parents=True)
    (escaped_staging / "escape/payload.txt").write_text("blocked", encoding="utf-8")
    with pytest.raises(publisher.PublicationError, match="publish outside"):
        publisher._publish_staging(escaped_staging, destination, [])
    assert not (outside / "payload.txt").exists()


@pytest.mark.skipif(shutil.which("ffmpeg") is None, reason="ffmpeg is required")
def test_publish_renders_only_verified_basicagent_run_and_separates_scope(
    tmp_path: Path,
) -> None:
    artifact_root = tmp_path / "artifacts"
    _snapshot(artifact_root, complete=True)
    docs_root = tmp_path / "docs/assets/validation/2026-08-31"
    stale = docs_root / "town03/expert_overview_1920x1080.png"
    stale.parent.mkdir(parents=True)
    stale.write_bytes(b"stale")
    report = tmp_path / "docs/validation-2026-08-31.md"
    report_preamble = tmp_path / "docs/validation-summary.inc.md"
    report_preamble.write_text(
        "## Campaign summary\n\nReproducible operator-reviewed context.\n",
        encoding="utf-8",
    )

    payload = publisher.publish_assets(
        artifact_root,
        docs_root,
        report_path=report,
        report_preamble_path=report_preamble,
        expected_selected_map_count=1,
        gif_fps=2.0,
        gif_max_frames=2,
    )

    assert len(payload["basicagent_publications"]) == 1
    assert payload["autoware_vad_publications"] == []
    archived_basicagent = docs_root / publisher.BASICAGENT_AGGREGATE_NAME
    assert archived_basicagent.read_bytes() == (artifact_root / "aggregate.json").read_bytes()
    assert payload["published_source_aggregate_file"] == (
        publisher.BASICAGENT_AGGREGATE_NAME
    )
    assert payload["source_aggregate_sha256"] == publisher._sha256(
        archived_basicagent
    )
    with Image.open(docs_root / "town01/expert_overview_1920x1080.png") as image:
        assert image.size == (1920, 1080)
    with Image.open(docs_root / "town01/expert_drive.gif") as image:
        assert image.size == (960, 540)
    assert not stale.exists()
    readme = (docs_root / "README.md").read_text(encoding="utf-8")
    assert "not Autoware VAD inference" in readme
    assert "No Autoware VAD trial was supplied" in readme
    assert "expert_overview_1920x1080.png" in readme
    assert publisher.BASICAGENT_AGGREGATE_NAME in readme
    assert "../../../../artifacts/" not in readme
    assert "Campaign summary" not in readme
    assert f"--report-preamble {report_preamble.resolve()}" in readme
    assert "--expected-map-count 19" in readme
    assert "--expected-selected-map-count 1" in readme
    report_text = report.read_text(encoding="utf-8")
    assert "all_maps_basicagent_status_1920x1080.png" in report_text
    assert report_text.index("## Campaign summary") < report_text.index("Source:")
    assert payload["report_preamble"] == {
        "source": str(report_preamble.resolve()),
        "sha256": publisher._sha256(report_preamble),
        "validation_scope": (
            "UTF-8, non-empty, no level-one heading, SHA256-bound; "
            "operator-reviewed narrative, not semantically generated"
        ),
        "embedding_requested": True,
        "report_target": str(report.resolve()),
    }
    completed = subprocess.run(
        ["sha256sum", "-c", "SHA256SUMS"],
        cwd=docs_root,
        capture_output=True,
        text=True,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr


def test_report_preamble_requires_report_and_cannot_add_level_one_heading(
    tmp_path: Path,
) -> None:
    preamble = tmp_path / "summary.md"
    preamble.write_text("## Valid summary\n", encoding="utf-8")

    with pytest.raises(publisher.PublicationError, match="requires a report path"):
        publisher.publish_assets(
            tmp_path / "artifacts",
            tmp_path / "assets",
            report_preamble_path=preamble,
        )

    preamble.write_text("# Duplicate title\n", encoding="utf-8")
    with pytest.raises(publisher.PublicationError, match="level-one heading"):
        publisher._load_report_preamble(preamble)

    preamble.write_text("## Valid summary\n\n# Late duplicate title\n", encoding="utf-8")
    with pytest.raises(publisher.PublicationError, match="level-one heading"):
        publisher._load_report_preamble(preamble)


def test_owned_window_visual_audit_requires_complete_centered_pass(
    tmp_path: Path,
) -> None:
    audit_dir = tmp_path / "visual_audit"
    audit_dir.mkdir()
    identities = {("town01", "straight"), ("town01", "turn")}
    trial_directories: dict[tuple[str, str], Path] = {}

    def source_record(path: Path, *, dimensions: list[int] | None = None) -> dict:
        record = {
            "sha256": publisher._sha256(path),
            "size_bytes": path.stat().st_size,
        }
        if dimensions is not None:
            record["dimensions"] = dimensions
        return record

    def visual_scene(map_id: str, trial_id: str, png_sha256: str) -> dict:
        return {
            "map_id": map_id,
            "trial_id": trial_id,
            "status": "PASS",
            "vehicle_visible": True,
            "reference_route_visible": True,
            "final_trajectory_visible": True,
            "vad_trajectories_visible": True,
            "viewport_centered": True,
            "representative_png_sha256": png_sha256,
        }

    review_scenes = []
    audit_trials = []
    for identity in sorted(identities):
        map_id, trial_id = identity
        trial = tmp_path / "trials" / map_id / trial_id
        trial.mkdir(parents=True)
        trial_directories[identity] = trial
        _write_json(trial / "result.json", {"success": True})
        _write_json(trial / "source_route.json", {"town": "Town01"})
        _write_json(trial / "desktop_capture.json", {"schema_version": 1})
        Image.new("RGB", (192, 108), "navy").save(
            trial / "autoware_rviz_fullscreen.png"
        )
        Image.new("RGB", (192, 108), "green").save(
            trial / "autoware_rviz_candidate.png"
        )
        Image.new("RGB", (96, 54), "purple").save(
            trial / "autoware_rviz_drive.gif", save_all=True
        )
        (trial / "autoware_rviz_capture.mkv").write_bytes(b"bound-recording")
        representative_sha256 = publisher._sha256(
            trial / "autoware_rviz_fullscreen.png"
        )
        scene = visual_scene(map_id, trial_id, representative_sha256)
        review_scenes.append(scene)
        audit_trials.append({
            "map_id": scene["map_id"],
            "trial_id": scene["trial_id"],
            "status": "PASS",
            "selected_attempt_directory": str(trial),
            "visual_review": scene,
            "result": {
                "sha256": publisher._sha256(trial / "result.json")
            },
            "source_route": {
                "status": "EXACT_MATCH",
                "sha256": publisher._sha256(trial / "source_route.json"),
                "centered_file": str(trial / "source_route.json"),
                "publication_original_file": str(trial / "source_route.json"),
                "publication_source": "fixture_selected_original",
            },
            "capture": {
                "metadata_sha256": publisher._sha256(
                    trial / "desktop_capture.json"
                ),
                "representative_png": source_record(
                    trial / "autoware_rviz_fullscreen.png",
                    dimensions=[192, 108],
                ),
                "candidate_png": source_record(
                    trial / "autoware_rviz_candidate.png",
                    dimensions=[192, 108],
                ),
                "drive_gif": source_record(
                    trial / "autoware_rviz_drive.gif",
                    dimensions=[96, 54],
                ),
                "recording": source_record(
                    trial / "autoware_rviz_capture.mkv"
                ),
                "rviz_view_contract": {
                    "vehicle_centered": True,
                    "target_frame": "base_link",
                    "center_xy_m": [0.0, 0.0],
                },
                "representative_frame": {
                    "frame_verification": {"pixel_exact_match": True}
                },
            },
        })
    contact_sheet = audit_dir / "v16_owned_window_contact_sheet.png"
    contact = Image.new("RGB", (320, 180), "navy")
    contact.paste("green", (160, 0, 320, 180))
    contact.save(contact_sheet)
    selection_records = [
        {
            "map_id": map_id,
            "trial_id": trial_id,
            "trial_directory": str(trial_directories[(map_id, trial_id)].resolve()),
            "source": "fixture_selected_original",
        }
        for map_id, trial_id in sorted(identities)
    ]
    _write_json(
        audit_dir / "v16_owned_window_visual_review.json",
        {"schema_version": 1, "scenes": review_scenes},
    )
    _write_json(
        audit_dir / "v16_owned_window_visual_audit.json",
        {
            "schema_version": 2,
            "status": "PASS",
            "mechanical_status": "PASS",
            "publication_selection": {
                "selection_source_file": str(
                    (tmp_path / "publication_manifest.json").resolve()
                ),
                "scope": publisher.PUBLICATION_SELECTION_SCOPE,
                "canonicalization": (
                    publisher.PUBLICATION_SELECTION_CANONICALIZATION
                ),
                "record_count": len(selection_records),
                "records": selection_records,
                "sha256": publisher._sha256_json(selection_records),
            },
            "counts": {
                "maps": 1,
                "total": 2,
                "mechanical_pass": 2,
                "visual_pass": 2,
                "visual_flag": 0,
                "visual_pending": 0,
            },
            "contact_sheet": {
                "dimensions": [320, 180],
                "scene_count": 2,
                "rows": 1,
                "columns": 2,
                "sha256": publisher._sha256(contact_sheet),
            },
            "trials": audit_trials,
        },
    )
    (audit_dir / "v16_owned_window_visual_audit.md").write_text(
        "# Audit\n\n- Overall status: **PASS**\n", encoding="utf-8"
    )

    record = publisher.collect_owned_window_visual_audit(
        audit_dir, trial_directories
    )

    assert record is not None
    assert record["status"] == "PASS"
    assert record["counts"]["visual_pass"] == 2
    assert record["contact_sheet_dimensions"] == [320, 180]
    assert set(record["files"]) == set(publisher.OWNED_WINDOW_VISUAL_AUDIT_NAMES)
    staging = tmp_path / "staging"
    copied = publisher._copy_owned_window_visual_audit(record, staging)
    assert copied["counts"]["total"] == 2
    for name in publisher.OWNED_WINDOW_VISUAL_AUDIT_NAMES:
        assert (
            staging / publisher.OWNED_WINDOW_VISUAL_AUDIT_DIR / name
        ).read_bytes() == (audit_dir / name).read_bytes()

    audit = json.loads(
        (audit_dir / "v16_owned_window_visual_audit.json").read_text(
            encoding="utf-8"
        )
    )
    audit["publication_selection"]["sha256"] = "0" * 64
    _write_json(audit_dir / "v16_owned_window_visual_audit.json", audit)
    with pytest.raises(
        publisher.PublicationError,
        match="publication-selection binding mismatch",
    ):
        publisher.collect_owned_window_visual_audit(audit_dir, trial_directories)
    audit["publication_selection"]["sha256"] = publisher._sha256_json(
        audit["publication_selection"]["records"]
    )
    audit["trials"][0]["capture"]["representative_png"]["sha256"] = "0" * 64
    _write_json(audit_dir / "v16_owned_window_visual_audit.json", audit)
    with pytest.raises(publisher.PublicationError, match="source binding mismatch"):
        publisher.collect_owned_window_visual_audit(audit_dir, trial_directories)
    audit["trials"][0]["capture"]["representative_png"]["sha256"] = (
        audit["trials"][0]["visual_review"]["representative_png_sha256"]
    )
    _write_json(audit_dir / "v16_owned_window_visual_audit.json", audit)

    review = json.loads(
        (audit_dir / "v16_owned_window_visual_review.json").read_text(
            encoding="utf-8"
        )
    )
    review["scenes"][0]["viewport_centered"] = False
    _write_json(audit_dir / "v16_owned_window_visual_review.json", review)
    with pytest.raises(publisher.PublicationError, match="visible content"):
        publisher.collect_owned_window_visual_audit(audit_dir, trial_directories)


def test_vad_publication_requires_successful_full_stack_result(tmp_path: Path) -> None:
    artifact_root = tmp_path / "artifacts"
    snapshot = _snapshot(artifact_root)
    trial = artifact_root / "vad_trial"
    _write_json(
        trial / "result.json",
        {
            "success": True,
            "execution_mode": "minimal",
            "assessment": {
                "planning_architecture": "vad_route_manager_hybrid",
                "route_completion": "PASS",
            },
            "final": {"goal_reached": True, "route_status": "goal_reached"},
        },
    )
    Image.new("RGB", (32, 18), "green").save(trial / "route_result.png")
    Image.new("RGB", (32, 18), "green").save(
        trial / "turn_path_control.gif", save_all=True
    )

    with pytest.raises(publisher.PublicationError, match="not a successful full-stack"):
        publisher.collect_vad_trials(snapshot, {"town01": trial})

    result = json.loads((trial / "result.json").read_text(encoding="utf-8"))
    result["execution_mode"] = "full_stack"
    _write_json(trial / "result.json", result)
    trials = publisher.collect_vad_trials(snapshot, {"town01": trial})
    assert len(trials) == 1
    assert trials[0].map_id == "town01"


def test_vad_trial_spec_supports_two_labeled_routes_for_one_map() -> None:
    specs = publisher._parse_vad_trial_specs(
        [
            "town01:straight=/tmp/straight",
            "town01:turn_left=/tmp/turn_left",
            "town02=/tmp/legacy",
        ]
    )

    assert [(item.map_id, item.trial_id) for item in specs] == [
        ("town01", "straight"),
        ("town01", "turn_left"),
        ("town02", None),
    ]
    with pytest.raises(publisher.PublicationError, match="duplicate --vad-trial id"):
        publisher._parse_vad_trial_specs(
            ["town01:straight=/tmp/one", "town01:straight=/tmp/two"]
        )


def test_visual_refresh_spec_requires_a_labeled_trial() -> None:
    specs = publisher._parse_vad_visual_refresh_specs(
        ["town01:straight=/tmp/centered"]
    )

    assert len(specs) == 1
    assert specs[0].source == "same_route_vehicle_centered_visual_refresh"
    with pytest.raises(
        publisher.PublicationError,
        match="must use MAP_ID:TRIAL_ID",
    ):
        publisher._parse_vad_visual_refresh_specs(
            ["town01=/tmp/unlabeled"]
        )


def test_same_route_centered_refresh_overlays_only_visual_media_and_is_archived(
    tmp_path: Path,
) -> None:
    artifact_root = tmp_path / "artifacts"
    snapshot = _snapshot(artifact_root)
    matrix_root = _write_terminal_vad_matrix(artifact_root, snapshot)
    selected = matrix_root / "maps/town01/trials/straight/attempt_001"
    refresh = artifact_root / "centered_vad_v2/town01/straight/attempt_001"
    _write_centered_vad_visual_refresh(refresh, selected)
    # Recorder startup/shutdown tails do not have to be symmetric around the
    # route.  The representative image is bound to the route midpoint by its
    # timestamp and exact recording offset, not to the whole MKV midpoint.
    capture_path = refresh / "desktop_capture.json"
    capture = json.loads(capture_path.read_text(encoding="utf-8"))
    capture["representative_frame"]["recording_duration_sec"] = 70.0
    _write_json(capture_path, capture)
    refresh_specs = publisher._parse_vad_visual_refresh_specs(
        [f"town01:straight={refresh}"]
    )
    docs_root = tmp_path / "docs/assets/validation/2026-08-31"
    report = tmp_path / "docs/validation-2026-08-31.md"

    payload = publisher.publish_assets(
        artifact_root,
        docs_root,
        report_path=report,
        expected_selected_map_count=1,
        vad_matrix_root=matrix_root,
        vad_visual_refresh_paths=refresh_specs,
    )

    assert payload["autoware_vad_visual_refresh_count"] == 1
    record = next(
        item
        for item in payload["autoware_vad_publications"]
        if item["map_id"] == "town01" and item["trial_id"] == "straight"
    )
    refresh_record = record["visual_refresh"]
    assert refresh_record["status"] == "PASS"
    assert refresh_record["route_match"] == "exact_payload_and_sha256"
    assert refresh_record["source_route_sha256"] == (
        refresh_record["validation_source_route_sha256"]
    )
    assert refresh_record["rviz_view_contract"]["vehicle_centered"] is True
    assert record["validation_desktop_capture"]["captured_at"] == (
        "2026-08-31T12:00:00+09:00"
    )
    assert record["desktop_capture"]["captured_at"] == (
        "2026-08-31T05:00:30+00:00"
    )
    published = docs_root / "town01/autoware_vad/straight"
    assert (published / "autoware_vad_result.json").read_bytes() == (
        selected / "result.json"
    ).read_bytes()
    assert (published / "autoware_rviz_fullscreen.png").read_bytes() == (
        refresh / "autoware_rviz_fullscreen.png"
    ).read_bytes()
    assert (published / "autoware_rviz_candidate.png").is_file()
    assert (published / "visual_refresh/repeat_result.json").read_bytes() == (
        refresh / "result.json"
    ).read_bytes()
    assert (published / "visual_refresh/source_route.json").is_file()
    assert (published / "visual_refresh/runtime.env").is_file()
    assert (
        published
        / "visual_refresh/rviz_capture_provenance/autoware_vad_carla.rviz"
    ).is_file()
    assert (
        published / "visual_refresh/original_validation_desktop_capture.json"
    ).read_bytes() == (selected / "desktop_capture.json").read_bytes()
    report_text = report.read_text(encoding="utf-8")
    assert "same-route vehicle-centered repeated PASS" in report_text
    assert "original validation provenance preserved" in report_text
    assert f"--vad-visual-refresh town01:straight={refresh.resolve()}" in report_text
    completed = subprocess.run(
        ["sha256sum", "-c", "SHA256SUMS"],
        cwd=docs_root,
        capture_output=True,
        text=True,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr


def test_centered_refresh_rejects_a_different_source_route(tmp_path: Path) -> None:
    artifact_root = tmp_path / "artifacts"
    snapshot = _snapshot(artifact_root)
    matrix_root = _write_terminal_vad_matrix(artifact_root, snapshot)
    specs, _ = publisher.resolve_vad_trial_specs(snapshot, (), matrix_root)
    selected_trials = publisher.collect_vad_trials(snapshot, specs)
    selected = matrix_root / "maps/town01/trials/straight/attempt_001"
    refresh = artifact_root / "centered_vad_v2/town01/straight/attempt_001"
    _write_centered_vad_visual_refresh(refresh, selected)
    source_route = json.loads(
        (refresh / "source_route.json").read_text(encoding="utf-8")
    )
    source_route["route_length_m"] = 43.0
    _write_json(refresh / "source_route.json", source_route)

    with pytest.raises(
        publisher.PublicationError,
        match="does not exactly match",
    ):
        publisher.collect_vad_visual_refreshes(
            snapshot,
            [publisher.VadTrialSpec("town01", "straight", refresh)],
            selected_trials,
        )


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("not_centered", "RViz view must center base_link"),
        ("not_midpoint", "representative frame must be"),
        ("not_recommended", "did not use the recommended"),
        ("odometry_trail", "visual clarity must disable odometry trails"),
    ],
)
def test_centered_refresh_fails_closed_on_capture_contract_changes(
    tmp_path: Path, mutation: str, message: str
) -> None:
    artifact_root = tmp_path / "artifacts"
    snapshot = _snapshot(artifact_root)
    matrix_root = _write_terminal_vad_matrix(artifact_root, snapshot)
    specs, _ = publisher.resolve_vad_trial_specs(snapshot, (), matrix_root)
    selected_trials = publisher.collect_vad_trials(snapshot, specs)
    selected = matrix_root / "maps/town01/trials/straight/attempt_001"
    refresh = artifact_root / "centered_vad_v2/town01/straight/attempt_001"
    _write_centered_vad_visual_refresh(refresh, selected)
    if mutation == "not_recommended":
        runtime_path = refresh / "runtime.env"
        runtime_path.write_text(
            runtime_path.read_text(encoding="utf-8").replace(
                "RECOMMENDED=true", "RECOMMENDED=false"
            ),
            encoding="utf-8",
        )
    else:
        capture_path = refresh / "desktop_capture.json"
        capture = json.loads(capture_path.read_text(encoding="utf-8"))
        if mutation == "not_centered":
            capture["rviz_view_contract"]["vehicle_centered"] = False
        elif mutation == "odometry_trail":
            capture["rviz_view_contract"]["visual_clarity"][
                "odometry_keep"
            ] = 250
        else:
            capture["representative_frame"]["selection"] = "first_frame"
        _write_json(capture_path, capture)

    with pytest.raises(publisher.PublicationError, match=message):
        publisher.collect_vad_visual_refreshes(
            snapshot,
            [publisher.VadTrialSpec("town01", "straight", refresh)],
            selected_trials,
        )


def test_labeled_vad_trials_publish_separately_with_desktop_and_analysis(
    tmp_path: Path,
) -> None:
    artifact_root = tmp_path / "artifacts"
    _snapshot(artifact_root)
    straight = artifact_root / "maps/town01/trials/straight"
    turn_left = artifact_root / "maps/town01/trials/turn_left"
    _write_labeled_vad_trial(
        straight,
        scenario="lane_follow",
        start_spawn_index=394,
        goal_spawn_index=290,
    )
    _write_labeled_vad_trial(
        turn_left,
        scenario="left",
        start_spawn_index=369,
        goal_spawn_index=425,
    )
    specs = publisher._parse_vad_trial_specs(
        [
            f"town01:straight={straight}",
            f"town01:turn_left={turn_left}",
        ]
    )
    docs_root = tmp_path / "docs/assets/validation/2026-08-31"
    report = tmp_path / "docs/validation-2026-08-31.md"

    payload = publisher.publish_assets(
        artifact_root,
        docs_root,
        report_path=report,
        expected_selected_map_count=1,
        vad_trial_paths=specs,
    )

    records = payload["autoware_vad_publications"]
    assert [record["trial_id"] for record in records] == ["straight", "turn_left"]
    assert records[0]["route"]["start_spawn_index"] == 394
    assert records[1]["route"]["goal_spawn_index"] == 425
    for trial_id in ("straight", "turn_left"):
        published = docs_root / f"town01/autoware_vad/{trial_id}"
        assert (published / "autoware_vad_result.json").is_file()
        assert (published / "autoware_vad_route.json").is_file()
        assert (published / "autoware_rviz_fullscreen.png").is_file()
        assert (published / "autoware_rviz_drive.gif").is_file()
        assert (published / "diagnosis.json").is_file()
        assert (published / "latency/e2e_latency.json").is_file()
    report_text = report.read_text(encoding="utf-8")
    assert "`straight`" in report_text
    assert "`turn_left`" in report_text
    assert "`394→290`" in report_text
    assert "`369→425`" in report_text
    assert f"--vad-trial town01:straight={straight.resolve()}" in report_text
    completed = subprocess.run(
        ["sha256sum", "-c", "SHA256SUMS"],
        cwd=docs_root,
        capture_output=True,
        text=True,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr


def test_labeled_vad_trial_rejects_capture_without_candidate(tmp_path: Path) -> None:
    artifact_root = tmp_path / "artifacts"
    snapshot = _snapshot(artifact_root)
    trial = artifact_root / "maps/town01/trials/straight"
    _write_labeled_vad_trial(
        trial,
        scenario="lane_follow",
        start_spawn_index=394,
        goal_spawn_index=290,
    )
    capture_path = trial / "desktop_capture.json"
    capture = json.loads(capture_path.read_text(encoding="utf-8"))
    capture["candidate_observed"] = False
    _write_json(capture_path, capture)

    with pytest.raises(publisher.PublicationError, match="did not observe a VAD candidate"):
        publisher.collect_vad_trials(
            snapshot,
            [publisher.VadTrialSpec("town01", "straight", trial)],
        )


def test_terminal_matrix_auto_discovers_pass_trials_and_keeps_explicit_lane_follow(
    tmp_path: Path,
) -> None:
    artifact_root = tmp_path / "artifacts"
    snapshot = _snapshot(artifact_root)
    matrix_root = _write_terminal_vad_matrix(artifact_root, snapshot)
    lane_follow = artifact_root / "vad_reference/c_track_lane_follow"
    _write_labeled_vad_trial(
        lane_follow,
        scenario="lane_follow",
        start_spawn_index=394,
        goal_spawn_index=290,
        town="C_track_1_0_7",
    )
    explicit = [
        publisher.VadTrialSpec(
            "c_track_1_0_7", "lane_follow", lane_follow
        )
    ]

    specs, matrix_record = publisher.resolve_vad_trial_specs(
        snapshot, explicit, matrix_root
    )

    assert matrix_record is not None
    assert matrix_record["status"] == "COMPLETE"
    assert matrix_record["discovered_pass_trial_count"] == 2
    assert {
        (spec.map_id, spec.trial_id, spec.source) for spec in specs
    } == {
        ("town01", "straight", "matrix_pass_auto_discovery"),
        ("town01", "turn", "matrix_pass_auto_discovery"),
        ("c_track_1_0_7", "lane_follow", "explicit"),
    }

    docs_root = tmp_path / "docs/assets/validation/2026-08-31"
    report = tmp_path / "docs/validation-2026-08-31.md"
    payload = publisher.publish_assets(
        artifact_root,
        docs_root,
        report_path=report,
        expected_selected_map_count=1,
        vad_trial_paths=explicit,
        vad_matrix_root=matrix_root,
    )

    records = payload["autoware_vad_publications"]
    assert len(records) == 3
    assert payload["autoware_vad_matrix_source"][
        "discovered_pass_trial_count"
    ] == 2
    matrix_source = payload["autoware_vad_matrix_source"]
    assert len(matrix_source["maps"]) == 19
    assert matrix_source["published_plan_file"] == publisher.VAD_MATRIX_PLAN_NAME
    assert matrix_source["published_aggregate_file"] == (
        publisher.VAD_MATRIX_AGGREGATE_NAME
    )
    archived_plan = docs_root / publisher.VAD_MATRIX_PLAN_NAME
    archived_aggregate = docs_root / publisher.VAD_MATRIX_AGGREGATE_NAME
    assert archived_plan.read_bytes() == (matrix_root / "matrix_plan.json").read_bytes()
    assert archived_aggregate.read_bytes() == (matrix_root / "aggregate.json").read_bytes()
    assert publisher._sha256(archived_plan) == matrix_source["plan_sha256"]
    assert publisher._sha256(archived_aggregate) == matrix_source["aggregate_sha256"]
    assert (
        docs_root
        / "c_track_1_0_7/autoware_vad/lane_follow/autoware_rviz_fullscreen.png"
    ).is_file()
    report_text = report.read_text(encoding="utf-8")
    assert f"--vad-matrix-root {matrix_root.resolve()}" in report_text
    assert f"--vad-trial c_track_1_0_7:lane_follow={lane_follow.resolve()}" in report_text
    assert "matrix PASS" in report_text
    assert "explicit supplement for matrix-BLOCKED map; block preserved" in report_text
    assert "CARLA BasicAgent" in report_text
    assert "Original terminal matrix: all 19 maps" in report_text
    matrix_table = report_text.split(
        "### Original terminal matrix: all 19 maps", 1
    )[1].split("### Final paired coverage", 1)[0]
    for matrix_map in matrix_source["maps"]:
        assert (
            f"`{matrix_map['map_id']}` ({matrix_map['canonical_name']})"
            in matrix_table
        )
    assert "`town02` (Town02) | no | **BLOCKED**" in report_text
    assert "`town05` (Town05) remains distinct from `town05_opt`" in report_text
    assert "`town10hd` (Town10HD) remains distinct" in report_text
    assert "Final paired coverage across executable variants" in report_text
    assert "`town01` (Town01) | PASS (matrix) | PASS (matrix) | **PASS/PASS**" in report_text
    assert "lane_follow` row is supplemental" in report_text
    assert "route-assisted HYBRID" in report_text


def test_speed_30kph_matrix_is_explicit_external_root_simulation_screening(
    tmp_path: Path,
) -> None:
    artifact_root = tmp_path / "artifacts/validation/2026-08-31"
    snapshot = _snapshot(artifact_root)
    matrix_root = _write_terminal_vad_matrix(
        tmp_path / "artifacts/validation/2026-09-01",
        snapshot,
        runnable_map_ids=publisher.SPEED_30KPH_RUNNABLE_MAP_IDS,
    )
    _convert_terminal_matrix_to_speed_30kph(matrix_root)

    with pytest.raises(
        publisher.PublicationError,
        match="does not match the explicitly requested selector",
    ):
        publisher.resolve_vad_trial_specs(snapshot, (), matrix_root)

    specs, matrix_record = publisher.resolve_vad_trial_specs(
        snapshot, (), matrix_root, "speed_30kph"
    )
    assert matrix_record is not None
    assert matrix_record["runtime_profile_selector"] == "speed_30kph"
    assert matrix_record["status"] == "COMPLETE"
    assert matrix_record["runnable_map_count"] == 9
    assert matrix_record["runnable_pass_count"] == 9
    assert matrix_record["blocked_map_count"] == 10
    assert matrix_record["discovered_pass_trial_count"] == 18
    assert matrix_record["evidence_interpretation"] == (
        publisher.SPEED_30KPH_PUBLICATION_INTERPRETATION
    )
    assert all(spec.evidence_root == matrix_root.resolve() for spec in specs)
    assert all(spec.runtime_profile_selector == "speed_30kph" for spec in specs)

    docs_root = tmp_path / "docs/assets/validation/2026-09-01"
    report = tmp_path / "docs/validation-2026-09-01.md"
    payload = publisher.publish_assets(
        artifact_root,
        docs_root,
        report_path=report,
        expected_selected_map_count=1,
        vad_matrix_root=matrix_root,
        vad_runtime_profile_selector="speed_30kph",
    )

    assert payload["runtime_profile_selector"] == "speed_30kph"
    assert payload["evidence_boundary"]["speed_30kph"] == (
        publisher.SPEED_30KPH_PUBLICATION_INTERPRETATION
    )
    assert payload["source_aggregate"] == str(
        (artifact_root / "aggregate.json").resolve()
    )
    records = payload["autoware_vad_publications"]
    assert len(records) == 18
    assert all(
        record["evidence_interpretation"]
        == publisher.SPEED_30KPH_PUBLICATION_INTERPRETATION
        for record in records
    )
    straight = next(record for record in records if record["trial_id"] == "straight")
    assert straight["route"]["endpoint_source"] == "generated_waypoints"
    assert straight["route"]["start_endpoint_index"] == 11
    assert straight["speed_profile"]["real_vehicle_ready"] is False
    assert straight["centered_capture_provenance"]["status"] == "PASS"
    assert straight["centered_capture_provenance"]["rviz_view_contract"][
        "vehicle_centered"
    ] is True
    for trial_id in ("straight", "turn"):
        published = docs_root / f"town01/autoware_vad/{trial_id}"
        assert (published / "speed_profile.json").is_file()
        assert (published / "speed_profile.png").is_file()
        record = next(
            item
            for item in records
            if item["map_id"] == "town01" and item["trial_id"] == trial_id
        )
        for name in publisher.SPEED_30KPH_CENTERED_CAPTURE_NAMES:
            published_path = published / name
            assert published_path.is_file()
            source_record = record["centered_capture_provenance"][
                "source_files"
            ][name]
            assert publisher._sha256(published_path) == source_record["sha256"]
        assert {
            "candidate",
            "capture_runtime",
            "rviz_config",
            "rviz_checksums",
        }.issubset(record["file_roles"])
    report_text = report.read_text(encoding="utf-8")
    assert "simulation screening only" in report_text
    assert "explicit CARLA simulation speed overlay" in report_text
    assert "`real_vehicle_ready=false`" in report_text
    assert "does not by itself claim that measured speed reached exactly 30 kph" in report_text
    assert "observed max `7.740 m/s` (`27.86 kph`)" in report_text
    assert "`waypoint 11→22`" in report_text
    assert "vehicle-centered candidate still" in report_text
    assert "pinned RViz config" in report_text
    assert "--vad-runtime-profile-selector speed_30kph" in report_text
    assert str(docs_root.resolve()) in report_text
    completed = subprocess.run(
        ["sha256sum", "-c", "SHA256SUMS"],
        cwd=docs_root,
        capture_output=True,
        text=True,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr


def test_camera_source_5hz_runtime_profile_is_explicit_and_pinned() -> None:
    selector = publisher.CAMERA_SOURCE_5HZ_SELECTOR
    profile = _camera_source_5hz_runtime_profile()
    plan = {
        "runtime_profile_selector": selector,
        "runtime_profile": profile,
    }
    aggregate = json.loads(json.dumps(plan))

    runtime_profile, interpretation = publisher._validate_matrix_runtime_profile(
        plan, aggregate, selector
    )

    assert runtime_profile == profile
    assert profile["wrapper_options"] == publisher.VAD_RUNTIME_WRAPPER_OPTIONS[
        selector
    ]
    assert (
        profile["camera_source_contract"]
        == publisher.CAMERA_SOURCE_5HZ_CONTRACT
    )
    assert interpretation == {
        **publisher.SPEED_30KPH_PUBLICATION_INTERPRETATION,
        "camera_source_profile_id": "carla_camera_source_5hz_ab_v1",
        "camera_sensor_count": 6,
        "camera_sensor_tick_sec": 0.2,
        "camera_source_frequency_hz": 5.0,
        "camera_ros_publish_frequency_hz": 5.0,
        "maximum_camera_stamp_gap_sec": 0.25,
    }


def test_camera_source_5hz_runtime_profile_rejects_contract_drift() -> None:
    selector = publisher.CAMERA_SOURCE_5HZ_SELECTOR
    profile = _camera_source_5hz_runtime_profile()
    profile["camera_source_contract"]["maximum_stamp_gap_sec"] = 0.5
    plan = {
        "runtime_profile_selector": selector,
        "runtime_profile": profile,
    }
    aggregate = json.loads(json.dumps(plan))

    with pytest.raises(
        publisher.PublicationError,
        match="camera-source 5 Hz runtime contract is not pinned",
    ):
        publisher._validate_matrix_runtime_profile(plan, aggregate, selector)


def test_baseline_speed_profile_rejects_camera_source_contract() -> None:
    selector = "speed_30kph"
    profile = _speed_30kph_runtime_profile()
    profile["camera_source_contract"] = dict(
        publisher.CAMERA_SOURCE_5HZ_CONTRACT
    )
    plan = {
        "runtime_profile_selector": selector,
        "runtime_profile": profile,
    }
    aggregate = json.loads(json.dumps(plan))

    with pytest.raises(
        publisher.PublicationError,
        match="non-camera-source runtime profile unexpectedly carries",
    ):
        publisher._validate_matrix_runtime_profile(plan, aggregate, selector)


def test_camera_source_5hz_fresh_validation_reads_raw_runtime_and_latency(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    runtime = {
        "CAMERA_SOURCE_5HZ": "true",
        "CAMERA_SOURCE_SENSOR_TICK_SEC": "0.2",
        "CAMERA_ROS_PUBLISH_HZ": "5.0",
    }
    (tmp_path / "runtime.env").write_text(
        "".join(f"{key}={value}\n" for key, value in runtime.items()),
        encoding="utf-8",
    )
    latency = {"status": "complete", "fixture": "raw-latency"}
    _write_json(tmp_path / "latency/e2e_latency.json", latency)
    profile = _camera_source_5hz_runtime_profile()
    expected = {"status": "PASS", "fixture": "freshly-recomputed"}
    calls = []

    def fake_validator(directory, actual_runtime, actual_profile, actual_latency):
        calls.append(
            (directory, actual_runtime, actual_profile, actual_latency)
        )
        return expected

    monkeypatch.setattr(
        publisher, "_matrix_camera_source_5hz_evidence", fake_validator
    )

    assert publisher._fresh_camera_source_5hz_evidence(
        tmp_path, profile, "fixture"
    ) == expected
    assert calls == [(tmp_path, runtime, profile, latency)]

    def failing_validator(*_args):
        raise publisher.MatrixValidationError("camera stamp gap exceeded")

    monkeypatch.setattr(
        publisher, "_matrix_camera_source_5hz_evidence", failing_validator
    )
    with pytest.raises(
        publisher.PublicationError,
        match="raw evidence failed fresh validation: camera stamp gap exceeded",
    ):
        publisher._fresh_camera_source_5hz_evidence(
            tmp_path, profile, "fixture"
        )


def test_camera_source_retry_queue_transition_is_bound_after_recorder(
    tmp_path: Path,
) -> None:
    (tmp_path / "stack.log").write_text(
        "[INFO 10.000000001] VAD frame queued: source_stamp_ns=1 "
        "assembled=2 capacity_pruned=0 superseded=0 mailbox_submitted=2 "
        "coalesced_drops=0 received_images_min=2 received_images_max=2\n"
        "[INFO 12.000000001] VAD frame queued: source_stamp_ns=2 "
        "assembled=3 capacity_pruned=0 superseded=1 mailbox_submitted=3 "
        "coalesced_drops=0 received_images_min=3 received_images_max=4\n"
        "published_count=4 mailbox_taken=4 coalesced_drops=0\n",
        encoding="utf-8",
    )
    (tmp_path / "recorder.log").write_text(
        "[INFO 11.000000001] Recording...\n", encoding="utf-8"
    )

    snapshot = publisher._camera_queue_snapshot(tmp_path, "fixture")

    assert snapshot["first"]["superseded"] == 0
    assert snapshot["first_superseded_transition"]["superseded"] == 1
    assert snapshot["first_superseded_transition"]["line_number"] == 2
    assert snapshot["recorder_start"]["wall_sec"] == "11.000000001"
    assert snapshot["transition_after_recorder"] is True
    assert snapshot["final_inference"] == {
        "line_number": 3,
        "published_count": 4,
        "mailbox_taken": 4,
        "coalesced_drops": 0,
    }


def test_camera_source_host_diagnostics_use_inclusive_whole_second_rows(
    tmp_path: Path,
) -> None:
    matrix_root = tmp_path / "matrix"
    telemetry = matrix_root / "host_telemetry"
    telemetry.mkdir(parents=True)
    (telemetry / "vmstat.log").write_text(
        "header one\nheader two\n"
        "1 0 0 0 0 0 0 0 0 0 0 0 10 5 85 0 0 2026-09-02 10:00:00\n"
        "1 0 0 0 0 0 0 0 0 0 0 0 5 5 90 0 0 2026-09-02 10:00:01\n"
        "1 0 0 0 0 0 0 0 0 0 0 0 4 5 91 0 0 2026-09-02 10:00:01\n"
        "1 0 0 0 0 0 0 0 0 0 0 0 3 5 92 0 0 2026-09-02 10:00:02\n",
        encoding="utf-8",
    )
    (telemetry / "nvidia_smi_dmon.log").write_text(
        "# header one\n# header two\n"
        "20260902 10:00:00 0 0 0 - 10 0\n"
        "20260902 10:00:01 0 0 0 - 20 0\n"
        "20260902 10:00:02 0 0 0 - 30 0\n",
        encoding="utf-8",
    )
    (telemetry / "pidstat.log").write_text(
        "Linux fixture (host) (24 CPU)\nlarge rows omitted\n", encoding="utf-8"
    )
    trial_directory = matrix_root / "maps/town03/trials/turn/attempt_001"
    result = {
        "started_at": "2026-09-02T01:00:00.900000+00:00",
        "finished_at": "2026-09-02T01:00:02.100000+00:00",
        "metrics": {"sim_elapsed_sec": 1.0, "wall_elapsed_sec": 4.0},
    }
    _write_json(trial_directory / "result.json", result)
    trial = publisher.VadTrial(
        map_id="town03",
        trial_id="turn",
        directory=trial_directory,
        result=result,
        route=None,
        desktop_capture=None,
        speed_profile=None,
        centered_capture_provenance=None,
        source="fixture",
    )

    host = publisher._telemetry_window_diagnostics(matrix_root, [trial])

    assert host is not None
    assert host["window"]["start"] == "2026-09-02T10:00:00+09:00"
    assert host["window"]["end"] == "2026-09-02T10:00:02+09:00"
    assert host["measurements"]["vmstat_sample_count"] == 4
    assert host["measurements"]["cpu_idle_sum"] == 358
    assert host["measurements"]["vmstat_timestamp_coverage"] == {
        "raw_row_count": 4,
        "unique_timestamp_count": 3,
        "duplicate_row_count": 1,
        "missing_timestamp_count": 0,
        "unique_timestamp_coverage_percent": 100.0,
        "maximum_timestamp_gap_sec": 1,
    }
    assert host["measurements"]["gpu_sm_sum"] == 60
    assert host["host_cpu_count"] == 24

    diagnostics = {
        "schema_version": 1,
        "status": "PASS",
        "runtime_profile_selector": publisher.CAMERA_SOURCE_5HZ_SELECTOR,
        "matrix_root": str(matrix_root),
        "retry_attempts": [],
        "host_telemetry": host,
        "campaign_console_files": {},
    }
    staging = tmp_path / "staging"
    published = publisher._copy_camera_source_campaign_diagnostics(
        diagnostics, staging
    )
    assert published["retry_attempt_count"] == 0
    assert (staging / "campaign_diagnostics/host_telemetry/vmstat.log").read_bytes() == (
        telemetry / "vmstat.log"
    ).read_bytes()
    assert (
        staging / "campaign_diagnostics/town03_turn_vmstat_window.log"
    ).is_file()
    assert publisher._sha256(
        staging / published["summary_file"]
    ) == published["summary_sha256"]


def test_camera_source_5hz_recorded_evidence_must_equal_fresh_recompute(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    expected = {
        "status": "PASS",
        "bundle_coverage_percent": 100.0,
        "maximum_camera_stamp_gap_sec": 0.2,
    }
    monkeypatch.setattr(
        publisher,
        "_validated_camera_source_5hz_provenance_files",
        lambda *_args: {},
    )
    monkeypatch.setattr(
        publisher,
        "_fresh_camera_source_5hz_evidence",
        lambda *_args: dict(expected),
    )

    fresh, provenance = publisher._validated_camera_source_5hz_evidence(
        tmp_path,
        _camera_source_5hz_runtime_profile(),
        dict(expected),
        "fixture",
    )
    assert fresh == expected
    assert provenance == {}

    stale = dict(expected)
    stale["maximum_camera_stamp_gap_sec"] = 0.25
    with pytest.raises(
        publisher.PublicationError,
        match="does not match freshly recomputed cadence/integrity evidence",
    ):
        publisher._validated_camera_source_5hz_evidence(
            tmp_path,
            _camera_source_5hz_runtime_profile(),
            stale,
            "fixture",
        )


def test_camera_source_5hz_dry_run_requires_all_publication_provenance(
    tmp_path: Path,
) -> None:
    required = (
        *publisher.CAMERA_SOURCE_5HZ_PROVENANCE_NAMES,
        "runtime.env",
        "latency/e2e_latency.json",
    )
    for name in required:
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(f"fixture {name}\n", encoding="utf-8")
    mapping = tmp_path / "sensor_mapping_provenance/sensor_mapping.yaml"
    mapping_sha256 = publisher._sha256(mapping)
    (tmp_path / "sensor_mapping_provenance/SHA256SUMS").write_text(
        f"{mapping_sha256}  sensor_mapping.yaml\n", encoding="utf-8"
    )
    evidence = {"mapping_sha256": mapping_sha256}

    sources = publisher._validated_camera_source_5hz_provenance_files(
        tmp_path, evidence, "fixture"
    )
    assert set(sources) == set(required)

    (tmp_path / "launch_args.txt").unlink()
    with pytest.raises(
        publisher.PublicationError,
        match="raw evidence is missing or symlinked: launch_args.txt",
    ):
        publisher._validated_camera_source_5hz_provenance_files(
            tmp_path, evidence, "fixture"
        )


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("map_roster", "exact 9 runnable maps"),
        ("final_state", "final_state COMPLETE"),
        ("scenario", "route scenario does not match"),
        ("lifecycle", "CARLA lifecycle contract is invalid"),
        ("visual_sha", "matrix visual evidence changed"),
        ("visual_dimensions", "no complete centered visual binding"),
    ],
)
def test_speed_30kph_final_publication_pre_gate_fails_closed(
    tmp_path: Path, mutation: str, message: str
) -> None:
    artifact_root = tmp_path / "artifacts/validation/2026-08-31"
    snapshot = _snapshot(artifact_root)
    matrix_root = _write_terminal_vad_matrix(
        tmp_path / "artifacts/validation/2026-09-01",
        snapshot,
        runnable_map_ids=publisher.SPEED_30KPH_RUNNABLE_MAP_IDS,
    )
    _convert_terminal_matrix_to_speed_30kph(matrix_root)
    plan_path = matrix_root / "matrix_plan.json"
    aggregate_path = matrix_root / "aggregate.json"
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    aggregate = json.loads(aggregate_path.read_text(encoding="utf-8"))

    if mutation == "map_roster":
        next(item for item in plan["maps"] if item["map_id"] == "town02")[
            "runnable"
        ] = True
        _write_json(plan_path, plan)
    elif mutation == "final_state":
        status_path = matrix_root / "maps/town01/status.json"
        status = json.loads(status_path.read_text(encoding="utf-8"))
        status.update(
            {
                "status": "FAILED",
                "stage": "straight_failed",
                "reason": "fixture final campaign failure",
            }
        )
        status["trials"]["straight"].update(
            {"status": "FAILED", "reason": "fixture straight failure"}
        )
        _write_json(status_path, status)
        aggregate.update(
            {
                "status": "FAILED",
                "runnable_pass_count": 8,
                "status_counts": {"BLOCKED": 10, "FAILED": 1, "PASS": 8},
                "maps": [
                    status if item["map_id"] == "town01" else item
                    for item in aggregate["maps"]
                ],
            }
        )
        _write_json(aggregate_path, aggregate)
    else:
        trial_id = "turn" if mutation == "scenario" else "straight"
        trial = next(
            item for item in aggregate["maps"] if item["map_id"] == "town01"
        )["trials"][trial_id]
        validation_path = Path(trial["validation"])
        validation = json.loads(validation_path.read_text(encoding="utf-8"))
        if mutation == "scenario":
            validation["catalog_scenario"] = "straight"
        elif mutation == "lifecycle":
            validation["carla_lifecycle"]["status"] = "FAILED"
        elif mutation == "visual_sha":
            fullscreen = Path(trial["attempt_directory"]) / (
                "autoware_rviz_fullscreen.png"
            )
            fullscreen.write_bytes(fullscreen.read_bytes() + b"drift")
        else:
            validation["visual_evidence"]["fullscreen_dimensions"] = [1280, 720]
        _write_json(validation_path, validation)

    with pytest.raises(publisher.PublicationError, match=message):
        publisher.resolve_vad_trial_specs(
            snapshot, (), matrix_root, "speed_30kph"
        )


def test_speed_30kph_rejects_speed_profile_swapped_between_trials(
    tmp_path: Path,
) -> None:
    artifact_root = tmp_path / "artifacts/validation/2026-08-31"
    snapshot = _snapshot(artifact_root)
    matrix_root = _write_terminal_vad_matrix(
        tmp_path / "artifacts/validation/2026-09-01",
        snapshot,
        runnable_map_ids=publisher.SPEED_30KPH_RUNNABLE_MAP_IDS,
    )
    _convert_terminal_matrix_to_speed_30kph(matrix_root)
    base = matrix_root / "maps/town01/trials"
    shutil.copy2(
        base / "turn/attempt_001/speed_profile.json",
        base / "straight/attempt_001/speed_profile.json",
    )
    specs, _ = publisher.resolve_vad_trial_specs(
        snapshot, (), matrix_root, "speed_30kph"
    )

    with pytest.raises(
        publisher.PublicationError, match="bound to a different route/result"
    ):
        publisher.collect_vad_trials(snapshot, specs)


def test_speed_30kph_rejects_runtime_profile_drift_after_matrix_validation(
    tmp_path: Path,
) -> None:
    artifact_root = tmp_path / "artifacts/validation/2026-08-31"
    snapshot = _snapshot(artifact_root)
    matrix_root = _write_terminal_vad_matrix(
        tmp_path / "artifacts/validation/2026-09-01",
        snapshot,
        runnable_map_ids=publisher.SPEED_30KPH_RUNNABLE_MAP_IDS,
    )
    _convert_terminal_matrix_to_speed_30kph(matrix_root)
    runtime_path = (
        matrix_root
        / "maps/town01/trials/straight/attempt_001/runtime.env"
    )
    runtime_path.write_text(
        runtime_path.read_text(encoding="utf-8").replace(
            "TARGET_SPEED_MPS=8.333333333333334", "TARGET_SPEED_MPS=7.0"
        ),
        encoding="utf-8",
    )
    specs, _ = publisher.resolve_vad_trial_specs(
        snapshot, (), matrix_root, "speed_30kph"
    )

    with pytest.raises(publisher.PublicationError, match="TARGET_SPEED_MPS mismatch"):
        publisher.collect_vad_trials(snapshot, specs)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("center", "matrix visual evidence changed"),
        ("pinned_config", "matrix visual evidence changed"),
        ("recording", "matrix visual evidence changed"),
    ],
)
def test_speed_30kph_publication_revalidates_own_centered_capture(
    tmp_path: Path, mutation: str, message: str
) -> None:
    artifact_root = tmp_path / "artifacts/validation/2026-08-31"
    snapshot = _snapshot(artifact_root)
    matrix_root = _write_terminal_vad_matrix(
        tmp_path / "artifacts/validation/2026-09-01",
        snapshot,
        runnable_map_ids=publisher.SPEED_30KPH_RUNNABLE_MAP_IDS,
    )
    _convert_terminal_matrix_to_speed_30kph(matrix_root)
    aggregate = json.loads((matrix_root / "aggregate.json").read_text(encoding="utf-8"))
    passing_map = next(item for item in aggregate["maps"] if item["status"] == "PASS")
    attempt = Path(passing_map["trials"]["straight"]["attempt_directory"])
    if mutation == "center":
        capture_path = attempt / "desktop_capture.json"
        capture = json.loads(capture_path.read_text(encoding="utf-8"))
        capture["rviz_view_contract"]["center_xy_m"] = [1.0, 0.0]
        _write_json(capture_path, capture)
    elif mutation == "pinned_config":
        (attempt / "rviz_capture_provenance/autoware_vad_carla.rviz").write_text(
            "Panels: [drift]\n", encoding="utf-8"
        )
    else:
        (attempt / "autoware_rviz_capture.mkv").unlink()

    with pytest.raises(publisher.PublicationError, match=message):
        specs, _ = publisher.resolve_vad_trial_specs(
            snapshot, (), matrix_root, "speed_30kph"
        )
        publisher.collect_vad_trials(snapshot, specs)


def test_speed_30kph_centered_source_change_fails_during_copy(
    tmp_path: Path,
) -> None:
    artifact_root = tmp_path / "artifacts/validation/2026-08-31"
    snapshot = _snapshot(artifact_root)
    matrix_root = _write_terminal_vad_matrix(
        tmp_path / "artifacts/validation/2026-09-01",
        snapshot,
        runnable_map_ids=publisher.SPEED_30KPH_RUNNABLE_MAP_IDS,
    )
    _convert_terminal_matrix_to_speed_30kph(matrix_root)
    specs, matrix_record = publisher.resolve_vad_trial_specs(
        snapshot, (), matrix_root, "speed_30kph"
    )
    trials = publisher.collect_vad_trials(snapshot, specs)
    trial = trials[0]
    (trial.directory / "autoware_rviz_candidate.png").write_bytes(b"changed")

    with pytest.raises(publisher.PublicationError, match="changed during publication"):
        publisher._copy_vad_trial(trial, tmp_path / "staging", matrix_record)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("vad_geometry", "vad_geometry_source"),
        ("gate_real_vehicle", "vehicle_cmd_gate"),
        ("controller_real_vehicle", "longitudinal_controller"),
        ("overlay_source", "explicit CARLA simulation speed overlay"),
        ("validation_speed", "lacks passing speed-contract evidence"),
        ("artifact_real_vehicle", "does not prove VAD geometry"),
    ],
)
def test_speed_30kph_publication_fails_closed_on_boundary_drift(
    tmp_path: Path, mutation: str, message: str
) -> None:
    artifact_root = tmp_path / "artifacts/validation/2026-08-31"
    snapshot = _snapshot(artifact_root)
    matrix_root = _write_terminal_vad_matrix(
        tmp_path / "artifacts/validation/2026-09-01",
        snapshot,
        runnable_map_ids=publisher.SPEED_30KPH_RUNNABLE_MAP_IDS,
    )
    _convert_terminal_matrix_to_speed_30kph(matrix_root)
    plan_path = matrix_root / "matrix_plan.json"
    aggregate_path = matrix_root / "aggregate.json"
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    aggregate = json.loads(aggregate_path.read_text(encoding="utf-8"))
    if mutation == "vad_geometry":
        plan["runtime_profile"]["speed_contract"]["vad_geometry_source"] = False
        aggregate["runtime_profile"] = plan["runtime_profile"]
    elif mutation == "gate_real_vehicle":
        plan["runtime_profile"]["speed_contract"]["vehicle_cmd_gate"][
            "real_vehicle_ready"
        ] = True
        aggregate["runtime_profile"] = plan["runtime_profile"]
    elif mutation == "controller_real_vehicle":
        plan["runtime_profile"]["speed_contract"]["longitudinal_controller"][
            "real_vehicle_ready"
        ] = True
        aggregate["runtime_profile"] = plan["runtime_profile"]
    elif mutation == "overlay_source":
        plan["runtime_profile"]["speed_contract"]["route_manager_parameters"][
            "longitudinal_velocity_source"
        ] = "raw_vad"
        aggregate["runtime_profile"] = plan["runtime_profile"]
    elif mutation == "validation_speed":
        passing_map = next(item for item in aggregate["maps"] if item["status"] == "PASS")
        validation_path = Path(
            passing_map["trials"]["straight"]["validation"]
        )
        validation = json.loads(validation_path.read_text(encoding="utf-8"))
        validation["speed_contract"] = None
        _write_json(validation_path, validation)
    else:
        passing_map = next(item for item in aggregate["maps"] if item["status"] == "PASS")
        attempt = Path(
            passing_map["trials"]["straight"]["attempt_directory"]
        )
        profile_path = attempt / "speed_profile.json"
        profile = json.loads(profile_path.read_text(encoding="utf-8"))
        profile["interpretation"]["real_vehicle_ready"] = True
        _write_json(profile_path, profile)
    _write_json(plan_path, plan)
    _write_json(aggregate_path, aggregate)

    with pytest.raises(publisher.PublicationError, match=message):
        specs, _ = publisher.resolve_vad_trial_specs(
            snapshot, (), matrix_root, "speed_30kph"
        )
        publisher.collect_vad_trials(snapshot, specs)


def test_speed_30kph_requires_matrix_and_cannot_replace_recommended_publication(
    tmp_path: Path,
) -> None:
    artifact_root = tmp_path / "artifacts"
    snapshot = _snapshot(artifact_root)
    with pytest.raises(
        publisher.PublicationError, match="requires --vad-matrix-root"
    ):
        publisher.resolve_vad_trial_specs(snapshot, (), None, "speed_30kph")

    docs_root = tmp_path / "docs/assets/validation/2026-08-31"
    previous = {
        "autoware_vad_matrix_source": {
            "runtime_profile": {
                "wrapper_options": list(
                    publisher.VAD_RUNTIME_WRAPPER_OPTIONS["recommended"]
                )
            }
        }
    }
    _write_json(docs_root / "publication_manifest.json", previous)
    original = (docs_root / "publication_manifest.json").read_bytes()
    with pytest.raises(
        publisher.PublicationError, match="use a new dated docs/assets destination"
    ):
        publisher._protect_existing_publication_profile(docs_root, "speed_30kph")
    assert (docs_root / "publication_manifest.json").read_bytes() == original


def test_matrix_auto_discovery_rejects_an_incomplete_campaign(tmp_path: Path) -> None:
    artifact_root = tmp_path / "artifacts"
    snapshot = _snapshot(artifact_root)
    matrix_root = _write_terminal_vad_matrix(
        artifact_root, snapshot, aggregate_status="INCOMPLETE"
    )

    with pytest.raises(publisher.PublicationError, match="matrix is not terminal"):
        publisher.discover_vad_matrix_trial_specs(snapshot, matrix_root)


def test_matrix_auto_discovery_requires_offset_aware_provenance_time(
    tmp_path: Path,
) -> None:
    artifact_root = tmp_path / "artifacts"
    snapshot = _snapshot(artifact_root)
    matrix_root = _write_terminal_vad_matrix(artifact_root, snapshot)
    plan_path = matrix_root / "matrix_plan.json"
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    plan["generated_at"] = "2026-08-31T02:00:00"
    _write_json(plan_path, plan)

    with pytest.raises(publisher.PublicationError, match="must include a UTC offset"):
        publisher.discover_vad_matrix_trial_specs(snapshot, matrix_root)


def test_post_terminal_supplement_preserves_blocked_optimized_map_status(
    tmp_path: Path,
) -> None:
    artifact_root = tmp_path / "artifacts"
    snapshot = _snapshot(artifact_root)
    matrix_root = _write_terminal_vad_matrix(artifact_root, snapshot)
    supplement = artifact_root / "supplemental_vad/town10hd_opt/straight/attempt_001"
    _write_labeled_vad_trial(
        supplement,
        scenario="straight",
        start_spawn_index=130,
        goal_spawn_index=137,
        town="Town10HD_Opt",
        captured_at="2026-08-31T05:00:00+00:00",
    )
    explicit = [
        publisher.VadTrialSpec("town10hd_opt", "straight", supplement)
    ]
    docs_root = tmp_path / "docs/assets/validation/2026-08-31"
    report = tmp_path / "docs/validation-2026-08-31.md"

    payload = publisher.publish_assets(
        artifact_root,
        docs_root,
        report_path=report,
        expected_selected_map_count=1,
        vad_trial_paths=explicit,
        vad_matrix_root=matrix_root,
    )

    record = next(
        item
        for item in payload["autoware_vad_publications"]
        if item["map_id"] == "town10hd_opt"
    )
    context = record["matrix_context"]
    assert context["relation"] == "explicit_supplement_for_blocked_map"
    assert context["timing"] == "after_terminal_matrix"
    assert context["matrix_map_status"] == "BLOCKED"
    assert context["matrix_trial_status"] == "BLOCKED"
    report_text = report.read_text(encoding="utf-8")
    assert "`town10hd` (Town10HD) remains distinct" in report_text
    assert "`town10hd_opt` (Town10HD_Opt)" in report_text
    assert "explicit supplement for matrix-BLOCKED map; block preserved" in report_text
    assert "after terminal matrix" in report_text


def test_terminal_failed_matrix_publishes_only_pass_and_preserves_failure(
    tmp_path: Path,
) -> None:
    artifact_root = tmp_path / "artifacts"
    snapshot = _snapshot(artifact_root)
    matrix_root = _write_terminal_vad_matrix(artifact_root, snapshot)
    _mark_terminal_matrix_straight_failed(matrix_root)
    recovery = artifact_root / "supplemental_vad/town01/straight/attempt_001"
    _write_labeled_vad_trial(
        recovery,
        scenario="straight",
        start_spawn_index=10,
        goal_spawn_index=20,
    )
    explicit = [publisher.VadTrialSpec("town01", "straight", recovery)]

    specs, matrix_record = publisher.resolve_vad_trial_specs(
        snapshot, explicit, matrix_root
    )

    assert matrix_record is not None
    assert matrix_record["status"] == "FAILED"
    assert matrix_record["status_counts"] == {"BLOCKED": 18, "FAILED": 1}
    assert matrix_record["runnable_pass_count"] == 0
    assert matrix_record["runnable_map_count"] == 1
    assert matrix_record["discovered_pass_trial_count"] == 1
    assert matrix_record["failed_map_count"] == 1
    assert matrix_record["failed_trial_count"] == 1
    assert matrix_record["failed_maps"][0]["map_id"] == "town01"
    assert {
        (spec.map_id, spec.trial_id, spec.source) for spec in specs
    } == {
        ("town01", "turn", "matrix_pass_auto_discovery"),
        ("town01", "straight", "explicit_recovery_for_matrix_failure"),
    }

    docs_root = tmp_path / "docs/assets/validation/2026-08-31"
    report = tmp_path / "docs/validation-2026-08-31.md"
    payload = publisher.publish_assets(
        artifact_root,
        docs_root,
        report_path=report,
        expected_selected_map_count=1,
        vad_trial_paths=explicit,
        vad_matrix_root=matrix_root,
    )

    archived_aggregate = docs_root / publisher.VAD_MATRIX_AGGREGATE_NAME
    archived = json.loads(archived_aggregate.read_text(encoding="utf-8"))
    assert archived["status"] == "FAILED"
    assert archived["maps"][0]["trials"]["straight"]["status"] == "FAILED"
    assert payload["autoware_vad_matrix_source"][
        "published_aggregate_file"
    ] == publisher.VAD_MATRIX_AGGREGATE_NAME
    records = payload["autoware_vad_publications"]
    assert {(record["trial_id"], record["source"]) for record in records} == {
        ("straight", "explicit_recovery_for_matrix_failure"),
        ("turn", "matrix_pass_auto_discovery"),
    }
    straight_record = next(
        record for record in records if record["trial_id"] == "straight"
    )
    assert straight_record["matrix_context"]["relation"] == (
        "explicit_recovery_alternate_route"
    )
    assert straight_record["matrix_context"]["timing"] == (
        "during_matrix_window_outside_matrix"
    )
    assert straight_record["matrix_context"]["matrix_failed_route"][
        "start_spawn_index"
    ] == 30
    report_text = report.read_text(encoding="utf-8")
    assert "Matrix campaign status: **FAILED**" in report_text
    assert "Original terminal matrix failures" in report_text
    assert "Original straight attempt failed" in report_text
    assert "explicit alternate-route supplement; matrix failure preserved" in report_text
    assert "failed matrix route 30→40, straight" in report_text
    assert "during matrix window (external)" in report_text
    assert (
        "`town01` (Town01) | PASS (alternate-route supplement; matrix failure "
        "preserved) | PASS (matrix) | **PASS/PASS**"
    ) in report_text
    assert publisher.VAD_MATRIX_AGGREGATE_NAME in report_text
    assert publisher.VAD_MATRIX_PLAN_NAME in report_text


def test_matrix_failed_status_does_not_hide_a_still_running_map(tmp_path: Path) -> None:
    artifact_root = tmp_path / "artifacts"
    snapshot = _snapshot(artifact_root)
    matrix_root = _write_terminal_vad_matrix(artifact_root, snapshot)
    status_path = matrix_root / "maps/town01/status.json"
    status = json.loads(status_path.read_text(encoding="utf-8"))
    status["status"] = "RUNNING"
    status["trials"]["turn"]["status"] = "RUNNING"
    _write_json(status_path, status)
    aggregate_path = matrix_root / "aggregate.json"
    aggregate = json.loads(aggregate_path.read_text(encoding="utf-8"))
    aggregate["status"] = "FAILED"
    aggregate["runnable_pass_count"] = 0
    aggregate["status_counts"] = {"BLOCKED": 18, "RUNNING": 1}
    aggregate["maps"] = [
        status if entry["map_id"] == "town01" else entry
        for entry in aggregate["maps"]
    ]
    _write_json(aggregate_path, aggregate)

    with pytest.raises(publisher.PublicationError, match="non-terminal runnable"):
        publisher.discover_vad_matrix_trial_specs(snapshot, matrix_root)


def test_failed_matrix_report_discloses_failures_without_pass_trials(
    tmp_path: Path,
) -> None:
    """A matrix failure remains visible even when it yields no VAD rows."""
    artifact_root = tmp_path / "artifacts"
    snapshot = _snapshot(artifact_root)
    matrix_root = _write_terminal_vad_matrix(artifact_root, snapshot)
    status_path = matrix_root / "maps/town01/status.json"
    status = json.loads(status_path.read_text(encoding="utf-8"))
    status.update({"status": "FAILED", "stage": "turn_failed"})
    for trial_id in ("straight", "turn"):
        status["trials"][trial_id] = {
            "status": "FAILED",
            "reason": f"Original {trial_id} attempt failed.",
            "attempt_directory": None,
            "validation": None,
        }
    _write_json(status_path, status)
    aggregate_path = matrix_root / "aggregate.json"
    aggregate = json.loads(aggregate_path.read_text(encoding="utf-8"))
    aggregate.update(
        {
            "status": "FAILED",
            "runnable_pass_count": 0,
            "status_counts": {"BLOCKED": 18, "FAILED": 1},
            "maps": [
                status if entry["map_id"] == "town01" else entry
                for entry in aggregate["maps"]
            ],
        }
    )
    _write_json(aggregate_path, aggregate)

    docs_root = tmp_path / "docs/assets/validation/2026-08-31"
    report = tmp_path / "docs/validation-2026-08-31.md"
    payload = publisher.publish_assets(
        artifact_root,
        docs_root,
        report_path=report,
        expected_selected_map_count=1,
        vad_matrix_root=matrix_root,
    )

    assert payload["autoware_vad_publications"] == []
    report_text = report.read_text(encoding="utf-8")
    assert "Matrix campaign status: **FAILED**" in report_text
    assert "Original terminal matrix failures" in report_text
    assert "Original straight attempt failed" in report_text
    assert "Original turn attempt failed" in report_text
