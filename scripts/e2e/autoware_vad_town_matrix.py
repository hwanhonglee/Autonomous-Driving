#!/usr/bin/env python3
"""Plan, validate, and report the straight/turn Autoware VAD town matrix.

The long-running CARLA/ROS processes are owned by
``run_autoware_vad_town_matrix.sh``.  This module is the strict data contract:
it admits only validated full-map bundles, verifies serialized CARLA road
options and their VAD commands, validates completed full-stack trials, and
atomically maintains resumable status/report files.
"""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import sys
import tempfile
from typing import Any, Mapping, Sequence

from PIL import Image
import yaml

try:
    from inventory_carla_training_maps import load_manifest as load_carla_manifest
except ModuleNotFoundError:
    from scripts.e2e.inventory_carla_training_maps import (
        load_manifest as load_carla_manifest,
    )

try:
    from physical_straight_geometry import (
        PhysicalStraightGeometryError,
        SPEED_30KPH_STRAIGHT_GEOMETRY_CONTRACT as COMMON_SPEED_30KPH_STRAIGHT_GEOMETRY_CONTRACT,
        analyze_serialized_physical_straight,
    )
except ModuleNotFoundError:
    from scripts.e2e.physical_straight_geometry import (
        PhysicalStraightGeometryError,
        SPEED_30KPH_STRAIGHT_GEOMETRY_CONTRACT as COMMON_SPEED_30KPH_STRAIGHT_GEOMETRY_CONTRACT,
        analyze_serialized_physical_straight,
    )

try:
    from physical_turn_geometry import (
        PhysicalTurnGeometryError,
        SPEED_30KPH_TURN_CONTRACT_PROVENANCE as COMMON_SPEED_30KPH_TURN_CONTRACT_PROVENANCE,
        SPEED_30KPH_TURN_GEOMETRY_CONTRACT as COMMON_SPEED_30KPH_TURN_GEOMETRY_CONTRACT,
        analyze_serialized_physical_turn,
    )
except ModuleNotFoundError:
    from scripts.e2e.physical_turn_geometry import (
        PhysicalTurnGeometryError,
        SPEED_30KPH_TURN_CONTRACT_PROVENANCE as COMMON_SPEED_30KPH_TURN_CONTRACT_PROVENANCE,
        SPEED_30KPH_TURN_GEOMETRY_CONTRACT as COMMON_SPEED_30KPH_TURN_GEOMETRY_CONTRACT,
        analyze_serialized_physical_turn,
    )

try:
    from serialized_custom_turn_geometry import (
        SerializedCustomTurnGeometryError,
        analyze_serialized_custom_turn,
    )
except ModuleNotFoundError:
    from scripts.e2e.serialized_custom_turn_geometry import (
        SerializedCustomTurnGeometryError,
        analyze_serialized_custom_turn,
    )


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MATRIX = Path(__file__).with_suffix(".yaml")
MAP_ID_RE = re.compile(r"^[a-z0-9_]+$")
TRIAL_IDS = ("straight", "turn")
VAD_COMMANDS = {
    "LEFT": 0,
    "RIGHT": 1,
    "STRAIGHT": 2,
    "LANEFOLLOW": 3,
    "VOID": 3,
    "CHANGELANELEFT": 4,
    "CHANGELANERIGHT": 5,
}
TERMINAL_MAP_STATUSES = {"PASS", "BLOCKED", "FAILED"}
RUNTIME_PROFILE_SELECTORS = ("recommended", "speed_30kph")
RUNTIME_WRAPPER_OPTIONS = {
    "recommended": ["--recommended", "--visualize", "--capture-desktop"],
    "speed_30kph": [
        "--recommended",
        "--speed-30kph",
        "--visualize",
        "--capture-desktop",
    ],
}
CUSTOM_MAP_INITIAL_APPROACH_CONTRACT = {
    "enabled": True,
    "distance_m": 15.0,
    "maximum_lateral_deviation_m": 1.5,
    "maximum_heading_change_deg": 30.0,
}
CUSTOM_MAP_TURN_GEOMETRY_CONTRACT = {
    "enabled": True,
    "require_single_directional_block": True,
    "forbid_additional_maneuvers": True,
    "minimum_arc_length_m": 10.0,
    "maximum_arc_length_m": 30.0,
    "minimum_heading_change_deg": 60.0,
    "maximum_heading_change_deg": 120.0,
    "maximum_heading_excess_deg": 20.0,
    "alignment_heading_margin_deg": 10.0,
    "maximum_command_lead_m": 8.0,
    "maximum_command_tail_m": 8.0,
    "curvature_percentile": 95.0,
    "maximum_p95_abs_curvature_per_m": 0.20,
}
SPEED_30KPH_STRAIGHT_GEOMETRY_CONTRACT = dict(
    COMMON_SPEED_30KPH_STRAIGHT_GEOMETRY_CONTRACT
)
SPEED_30KPH_TURN_GEOMETRY_CONTRACT = dict(
    COMMON_SPEED_30KPH_TURN_GEOMETRY_CONTRACT
)
SPEED_30KPH_TURN_CONTRACT_PROVENANCE = dict(
    COMMON_SPEED_30KPH_TURN_CONTRACT_PROVENANCE
)
SPEED_30KPH_PACKAGED_STRAIGHT_ENDPOINT_SPACING_M = 10.0
TOWN10HD_OPT_30KPH_COMPACT_PROFILE = "town10hd_opt_30kph_compact_v1"
TOWN10HD_OPT_30KPH_STRAIGHT_OVERRIDE = {
    "seeds": [0],
    "pairs_per_seed": 8,
    "minimum_distance_m": 170.0,
    "maximum_distance_m": 182.0,
    "preferred_distance_m": 172.0,
    "maximum_traces_per_scenario": 20000,
    "endpoint_waypoint_spacing_m": 0.5,
    "endpoint_junction_policy": "exclude",
    "candidate_enumeration_policy": "directed_topology_straight_v1",
    "straight_capacity_profile": TOWN10HD_OPT_30KPH_COMPACT_PROFILE,
}
TOWN10HD_OPT_30KPH_CAPACITY_PROVENANCE = {
    "profile_id": TOWN10HD_OPT_30KPH_COMPACT_PROFILE,
    "scope": "CARLA-only Town10HD_Opt 30 kph straight capacity screening",
    "measurement_source": (
        "measured 30 kph threshold-entry, exposure, and stopping distances"
    ),
    "validation_threshold_reuse": False,
    "minimum_route_length_m": 170.0,
    "threshold_entry_distance_m": 145.865,
    "minimum_exposure_distance_m": 7.5,
    "minimum_stop_distance_m": 14.169,
    "derived_required_distance_m": 167.534,
    "minimum_route_margin_m": 2.466,
    "derivation": "145.865 + 7.5 + 14.169 = 167.534; 170.0 - 167.534 = 2.466",
    "unchanged_verdicts": [
        "exact_serialized_physical_straight",
        "30_kph_speed_exposure",
        "AEB",
        "MRM",
    ],
    "postfilter_authority": (
        "exact serialized physical-straight geometry remains mandatory"
    ),
    "real_vehicle_ready": False,
}
TURN_MANEUVER_OPTIONS = frozenset(
    {
        "LEFT",
        "RIGHT",
        "STRAIGHT",
        "CHANGELANELEFT",
        "CHANGELANERIGHT",
    }
)
ROUTE_GENERATION_DISTANCE_FIELDS = (
    "minimum_distance_m",
    "maximum_distance_m",
    "preferred_distance_m",
)
ROUTE_GENERATION_FIELDS = (
    "weather",
    "seeds",
    "pairs_per_seed",
    *ROUTE_GENERATION_DISTANCE_FIELDS,
    "sampling_resolution_m",
    "maximum_endpoint_offset_m",
    "maximum_traces_per_scenario",
)
ROUTE_SELECTION_POLICY = "exact_catalog_scenario_seed_pair_v1"
ROUTE_SELECTION_OVERRIDE_FIELDS = frozenset(
    {"scenario", "seed", "pair_index", "rationale"}
)
SPEED_30KPH_TURN_ROUTE_SELECTION_METHOD = (
    "physical_turn_curvature_deceleration_v1"
)
SPEED_30KPH_TURN_ROUTE_SELECTION_POLICY = {
    "policy_id": SPEED_30KPH_TURN_ROUTE_SELECTION_METHOD,
    "applicability": "speed_30kph_all_turn_candidates",
    "exact_serialized_physical_turn_pass_required": True,
    "common_physical_turn_ranking_authority": True,
    "custom_turn_geometry_preflight_role": (
        "additional_required_provenance_gate"
    ),
    "execution_result_fields_used": False,
    "map_preflight_fallback_allowed": False,
    "candidate_pair_reindexing_allowed": False,
    "candidate_generation_quota_modified": False,
    "curvature_epsilon_per_m": 1.0e-6,
    "ranking_order": [
        "curvature_limited_speed_mps_desc",
        "braking_preview_reserve_m_desc",
        "maximum_command_alignment_skew_m_asc",
        "turn_core_arc_length_m_desc",
        "scenario_asc",
        "seed_asc",
        "pair_index_asc",
        "route_id_asc",
    ],
    "parameter_sources": {
        "target_speed_mps": "speed_contract.target_speed_mps",
        "maximum_lateral_acceleration_mps2": (
            "speed_contract.route_manager_parameters."
            "maximum_lateral_acceleration_mps2"
        ),
        "comfortable_deceleration_mps2": (
            "speed_contract.route_manager_parameters."
            "comfortable_deceleration_mps2"
        ),
        "route_curvature_lookahead_m": (
            "speed_contract.route_manager_parameters."
            "route_curvature_lookahead_m"
        ),
    },
}
SPEED_30KPH_MAP_TRIAL_ROUTE_SELECTION_OVERRIDES = {
    "town03": {
        "straight": {
            "scenario": "straight",
            "seed": 0,
            "pair_index": 1,
            "rationale": "traffic_light_head_clearance_screening_v1",
        },
    },
}
BUNDLE_FILE_NAMES = {
    "metadata": "map_bundle.json",
    "lanelet2": "lanelet2_map.osm",
    "pointcloud": "pointcloud_map.pcd",
    "projector": "map_projector_info.yaml",
}

# These are the in-repository programs and fixed launch/view assets that can
# change route admission, the executed trial, or the PASS evidence contract.
# Keep the allow-list explicit: recursively hashing the repository would bind
# unrelated documentation/artifacts, while omitting a helper would permit the
# campaign semantics to drift after matrix_plan.json was admitted.
CAMPAIGN_EXECUTION_CONTRACT_PATHS = (
    "patches/autoware_mission_planner_lane_only_no_area.patch",
    "patches/autoware_mission_planner_lane_only_no_area.manifest.json",
    "patches/autoware_tensorrt_vad_object_yaw.patch",
    "patches/autoware_aeb_named_timeouts.patch",
    "patches/autoware_vad_object_safety.manifest.json",
    "scripts/e2e/apply_mission_planner_lane_only_patch.sh",
    "scripts/e2e/mission_planner_build_provenance.py",
    "scripts/e2e/apply_vad_object_safety_patches.sh",
    "scripts/e2e/vad_object_safety_build_provenance.py",
    "scripts/e2e/workspace_runtime_lock.sh",
    "scripts/e2e/build.sh",
    "scripts/e2e/build_full.sh",
    "scripts/e2e/autoware_vad_town_matrix.py",
    "scripts/e2e/run_autoware_vad_town_matrix.sh",
    "scripts/e2e/prepare_carla_expert_route_catalog.py",
    "scripts/e2e/prepare_carla_route.py",
    "scripts/e2e/serialized_custom_turn_geometry.py",
    "scripts/e2e/physical_straight_geometry.py",
    "scripts/e2e/physical_turn_geometry.py",
    "scripts/e2e/inventory_carla_training_maps.py",
    "scripts/e2e/run_carla_map.sh",
    "scripts/e2e/probe_carla_server.py",
    "scripts/e2e/run_recorded_route_trial.sh",
    "scripts/e2e/run_route_vad_fast.sh",
    "scripts/e2e/run_route_vad_full.sh",
    "scripts/e2e/route_test.sh",
    "scripts/e2e/route_test.py",
    "scripts/e2e/align_carla_route_to_map.py",
    "scripts/e2e/validate_route_map.py",
    "scripts/e2e/prepare_packaged_town_full_maps.py",
    "scripts/e2e/record_turn_dynamics.sh",
    "scripts/e2e/analyze_speed_profile.py",
    "scripts/e2e/analyze_turn_dynamics.py",
    "scripts/e2e/analyze_e2e_latency.py",
    "scripts/e2e/capture_raw_vehicle_cmd_converter_provenance.py",
    "scripts/e2e/render_route_result.sh",
    "scripts/e2e/render_route_result.py",
    "scripts/e2e/render_turn_animation.py",
    "scripts/e2e/process_group_cleanup.sh",
    "scripts/e2e/env.sh",
    "autoware_e2e_vad_launch/launch/carla_vad_full.launch.xml",
    "autoware_e2e_vad_launch/launch/system_vad_full_shell.launch.xml",
    "autoware_e2e_vad_launch/launch/mission_planning_full_shell.launch.xml",
    "autoware_e2e_vad_launch/launch/vad_visualization.launch.xml",
    "autoware_e2e_vad_launch/launch/vad_carla_tiny_fast.launch.xml",
    "autoware_e2e_vad_launch/config/sensor_mapping_vad_fast_reliable_imu.yaml",
    "autoware_e2e_vad_launch/scripts/vad_aeb_configurator.py",
    "autoware_e2e_vad_launch/scripts/carla_map_aligned_odometry.py",
    "autoware_e2e_vad_launch/scripts/vad_imu_acceleration_adapter.py",
    "autoware_e2e_vad_launch/scripts/vad_standard_route_adapter.py",
    "autoware_e2e_vad_launch/scripts/vad_route_logic.py",
    "autoware_e2e_vad_launch/scripts/vad_route_manager.py",
    "autoware_e2e_vad_launch/config/vad_route_manager.param.yaml",
    "autoware_e2e_vad_launch/config/pid_carla_vad_30kph.param.yaml",
    "autoware_e2e_vad_launch/config/pid_carla_vad_30kph.param.yaml.metadata.json",
    "autoware_e2e_vad_launch/config/vehicle_cmd_gate_carla_30kph.param.yaml",
    "autoware_e2e_vad_launch/config/vehicle_cmd_gate_carla_30kph.param.yaml.metadata.json",
    "autoware_e2e_vad_launch/rviz/autoware_vad_carla.rviz",
    "src/universe/autoware_universe/planning/autoware_mission_planner_universe/src/lanelet2_plugins/default_planner.cpp",
    "src/universe/autoware_universe/planning/autoware_mission_planner_universe/test/test_lanelet2_plugins_default_planner.cpp",
    "src/core/autoware_core/planning/autoware_route_handler/include/autoware/route_handler/route_handler.hpp",
    "src/core/autoware_core/planning/autoware_route_handler/src/route_handler.cpp",
    "src/universe/autoware_universe/e2e/autoware_tensorrt_vad/CMakeLists.txt",
    "src/universe/autoware_universe/e2e/autoware_tensorrt_vad/lib/output_converter/objects_converter.cpp",
    "src/universe/autoware_universe/e2e/autoware_tensorrt_vad/test/test_objects_converter_orientation.cpp",
    "src/universe/autoware_universe/control/autoware_autonomous_emergency_braking/include/autoware/autonomous_emergency_braking/node.hpp",
    "src/universe/autoware_universe/control/autoware_autonomous_emergency_braking/include/autoware/autonomous_emergency_braking/utils.hpp",
    "src/universe/autoware_universe/control/autoware_autonomous_emergency_braking/src/node.cpp",
    "src/universe/autoware_universe/control/autoware_autonomous_emergency_braking/src/utils.cpp",
    "src/universe/autoware_universe/control/autoware_autonomous_emergency_braking/test/test.cpp",
    "build/autoware_mission_planner_universe/libautoware_mission_planner_universe_lanelet2_plugins.so",
    "build/autoware_mission_planner_universe/e2e_lane_only_build_provenance.json",
    "build/autoware_route_handler/libautoware_route_handler.so",
    "build/autoware_tensorrt_vad/libautoware_tensorrt_vad_lib.so",
    "build/autoware_autonomous_emergency_braking/libautoware_autonomous_emergency_braking_node.so",
    "build/autoware_autonomous_emergency_braking/libautoware_autonomous_emergency_braking_helpers.so",
    "build/autoware_tensorrt_vad/e2e_object_safety_build_provenance.json",
)
SPEED_30KPH_VISUAL_EVIDENCE_NAMES = (
    "desktop_capture.json",
    "autoware_rviz_fullscreen.png",
    "autoware_rviz_drive.gif",
    "autoware_rviz_candidate.png",
    "autoware_rviz_capture.mkv",
    "rviz_capture_provenance/autoware_vad_carla.rviz",
    "rviz_capture_provenance/SHA256SUMS",
)
SPEED_30KPH_FULLSCREEN_DIMENSIONS = (1920, 1080)
SPEED_30KPH_DRIVE_GIF_DIMENSIONS = (960, 540)


class MatrixError(RuntimeError):
    """Raised when matrix evidence violates its reproducibility contract."""


class PhysicalTurnCandidateRejection(MatrixError):
    """A non-fatal exact physical-turn rejection for one catalog candidate."""

    def __init__(self, message: str, evidence: Mapping[str, Any]) -> None:
        super().__init__(message)
        self.evidence = dict(evidence)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while block := stream.read(1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def sha256_json(value: Any) -> str:
    serialized = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(serialized).hexdigest()


def _speed_visual_evidence_binding(
    trial_dir: Path,
    desktop: Mapping[str, Any],
    png_size: tuple[int, int],
    gif_size: tuple[int, int],
) -> dict[str, Any]:
    if (
        png_size != SPEED_30KPH_FULLSCREEN_DIMENSIONS
        or gif_size != SPEED_30KPH_DRIVE_GIF_DIMENSIONS
        or desktop.get("source_dimensions")
        != list(SPEED_30KPH_FULLSCREEN_DIMENSIONS)
        or desktop.get("png_dimensions")
        != list(SPEED_30KPH_FULLSCREEN_DIMENSIONS)
        or desktop.get("candidate_png_dimensions")
        != list(SPEED_30KPH_FULLSCREEN_DIMENSIONS)
        or desktop.get("gif_dimensions")
        != list(SPEED_30KPH_DRIVE_GIF_DIMENSIONS)
    ):
        raise MatrixError(
            "speed_30kph requires a 1920x1080 full-screen/candidate capture "
            "and 960x540 drive GIF"
        )
    contract = desktop.get("rviz_view_contract")
    if (
        not isinstance(contract, Mapping)
        or contract.get("vehicle_centered") is not True
        or contract.get("target_frame") != "base_link"
        or contract.get("center_xy_m") != [0.0, 0.0]
    ):
        raise MatrixError(
            "speed_30kph visual evidence is not vehicle-centered on base_link"
        )
    files: dict[str, dict[str, Any]] = {}
    for name in SPEED_30KPH_VISUAL_EVIDENCE_NAMES:
        source = trial_dir / name
        if not source.is_file() or source.is_symlink():
            raise MatrixError(
                f"speed_30kph visual evidence is missing or unsafe: {name}"
            )
        files[name] = {
            "size_bytes": source.stat().st_size,
            "sha256": sha256_file(source),
        }
    campaign_config = ROOT / "autoware_e2e_vad_launch/rviz/autoware_vad_carla.rviz"
    captured_config = trial_dir / "rviz_capture_provenance/autoware_vad_carla.rviz"
    if sha256_file(captured_config) != sha256_file(campaign_config):
        raise MatrixError(
            "speed_30kph capture RViz config differs from the campaign file"
        )
    return {
        "schema_version": 1,
        "binding": "matrix_validation_sha256_v1",
        "fullscreen_dimensions": list(png_size),
        "candidate_dimensions": list(SPEED_30KPH_FULLSCREEN_DIMENSIONS),
        "drive_gif_dimensions": list(gif_size),
        "vehicle_centered": True,
        "target_frame": "base_link",
        "files": files,
    }


def read_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise MatrixError(f"cannot read {label} {path}: {error}") from error
    if not isinstance(value, dict):
        raise MatrixError(f"{label} root must be an object: {path}")
    return value


def atomic_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(value, stream, indent=2, sort_keys=False, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def atomic_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(value)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def _safe_inside(root: Path, candidate: Path, label: str) -> Path:
    root = root.expanduser().resolve()
    candidate = candidate.expanduser().resolve()
    try:
        candidate.relative_to(root)
    except ValueError as error:
        raise MatrixError(f"{label} escapes {root}: {candidate}") from error
    return candidate


def _require_keys(value: Mapping[str, Any], keys: Sequence[str], label: str) -> None:
    missing = [key for key in keys if key not in value]
    if missing:
        raise MatrixError(f"{label} is missing fields: {missing}")


def _campaign_execution_contract() -> dict[str, Any]:
    files: list[dict[str, str]] = []
    if len(set(CAMPAIGN_EXECUTION_CONTRACT_PATHS)) != len(
        CAMPAIGN_EXECUTION_CONTRACT_PATHS
    ):
        raise MatrixError("campaign execution-contract allow-list has duplicates")
    for relative in CAMPAIGN_EXECUTION_CONTRACT_PATHS:
        if not isinstance(relative, str) or not relative or Path(relative).is_absolute():
            raise MatrixError(
                f"invalid campaign execution-contract path: {relative!r}"
            )
        source_path = ROOT / relative
        if source_path.is_symlink():
            raise MatrixError(
                f"campaign execution-contract file must not be a symlink: {relative}"
            )
        resolved = _safe_inside(ROOT, source_path, "campaign execution-contract file")
        if not resolved.is_file():
            raise MatrixError(
                f"campaign execution-contract file is missing: {relative}"
            )
        files.append({"path": relative, "sha256": sha256_file(resolved)})
    return {
        "schema_version": 1,
        "hash_algorithm": "sha256",
        "repository_root": ".",
        "files": files,
    }


def _admission_contract_payload(plan: Mapping[str, Any]) -> dict[str, Any]:
    fields = (
        "matrix_id",
        "matrix_manifest_sha256",
        "canonical_map_manifest_sha256",
        "runtime_profile_selector",
        "runtime_profile",
        "route_contract",
        "route_generation_contracts",
        "route_selection_policy",
        "route_selection_overrides",
        "route_selection_overrides_sha256",
        "turn_route_selection_policy",
        "turn_route_selection_policy_sha256",
        "campaign_execution_contract",
        "campaign_execution_contract_sha256",
        "maps",
    )
    _require_keys(plan, fields, "matrix plan admission contract")
    return {field: plan[field] for field in fields}


def _verify_campaign_execution_contract(
    plan: Mapping[str, Any], label: str
) -> None:
    admitted = plan.get("campaign_execution_contract")
    admitted_sha256 = plan.get("campaign_execution_contract_sha256")
    if not isinstance(admitted, dict) or not isinstance(admitted_sha256, str):
        raise MatrixError(f"{label} lacks the campaign execution-code contract")
    if len(admitted_sha256) != 64 or sha256_json(admitted) != admitted_sha256:
        raise MatrixError(
            f"{label} campaign execution-code contract digest is invalid"
        )
    current = _campaign_execution_contract()
    admitted_files = admitted.get("files")
    current_files = current["files"]
    if not isinstance(admitted_files, list):
        raise MatrixError(f"{label} campaign execution-code file list is invalid")
    admitted_paths = [
        item.get("path") if isinstance(item, dict) else None
        for item in admitted_files
    ]
    current_paths = [item["path"] for item in current_files]
    if admitted_paths != current_paths:
        raise MatrixError(
            f"{label} campaign execution-code allow-list differs from admission"
        )
    for admitted_file, current_file in zip(admitted_files, current_files):
        if admitted_file != current_file:
            raise MatrixError(
                f"{label} campaign execution-code SHA256 changed: "
                f"{current_file['path']}"
            )
    if admitted != current:
        raise MatrixError(
            f"{label} campaign execution-code contract metadata changed"
        )


def _verify_campaign_plan(plan: Mapping[str, Any], label: str) -> None:
    _verify_campaign_execution_contract(plan, label)
    admitted_sha256 = plan.get("admission_contract_sha256")
    if not isinstance(admitted_sha256, str) or len(admitted_sha256) != 64:
        raise MatrixError(f"{label} admission contract digest is missing")
    if sha256_json(_admission_contract_payload(plan)) != admitted_sha256:
        raise MatrixError(f"{label} admission contract digest is invalid")


def _load_verified_campaign_plan(output_root: Path, label: str) -> dict[str, Any]:
    plan = read_object(output_root / "matrix_plan.json", "matrix plan")
    _verify_campaign_plan(plan, label)
    return plan


def _finite_number(value: Any, label: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
    ):
        raise MatrixError(f"{label} must be a finite number, got {value!r}")
    return float(value)


def _require_exact_number(
    value: Mapping[str, Any], field: str, expected: float, label: str
) -> None:
    actual = _finite_number(value.get(field), f"{label}.{field}")
    if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1.0e-9):
        raise MatrixError(
            f"{label}.{field} must be {expected!r}, got {actual!r}"
        )


def _validate_route_selection_overrides(
    value: Any, label: str
) -> dict[str, dict[str, dict[str, Any]]]:
    if not isinstance(value, dict):
        raise MatrixError(f"{label} must be an object")
    result: dict[str, dict[str, dict[str, Any]]] = {}
    for map_id, trial_overrides in value.items():
        if not isinstance(map_id, str) or not MAP_ID_RE.fullmatch(map_id):
            raise MatrixError(f"{label} has an unsafe map id: {map_id!r}")
        if not isinstance(trial_overrides, dict) or not trial_overrides:
            raise MatrixError(f"{label}.{map_id} must be a non-empty object")
        if any(trial_id not in TRIAL_IDS for trial_id in trial_overrides):
            raise MatrixError(f"{label}.{map_id} has an unknown trial id")
        resolved_trials: dict[str, dict[str, Any]] = {}
        for trial_id, override in trial_overrides.items():
            if not isinstance(override, dict) or set(override) != (
                ROUTE_SELECTION_OVERRIDE_FIELDS
            ):
                raise MatrixError(
                    f"{label}.{map_id}.{trial_id} fields are not pinned"
                )
            scenario = override.get("scenario")
            allowed_scenarios = (
                {"straight"} if trial_id == "straight" else {"left", "right"}
            )
            if scenario not in allowed_scenarios:
                raise MatrixError(
                    f"{label}.{map_id}.{trial_id}.scenario is invalid"
                )
            for field in ("seed", "pair_index"):
                number = override.get(field)
                if (
                    isinstance(number, bool)
                    or not isinstance(number, int)
                    or number < 0
                ):
                    raise MatrixError(
                        f"{label}.{map_id}.{trial_id}.{field} must be a "
                        "non-negative integer"
                    )
            rationale = override.get("rationale")
            if (
                not isinstance(rationale, str)
                or not rationale
                or not re.fullmatch(r"[a-z0-9_]+", rationale)
            ):
                raise MatrixError(
                    f"{label}.{map_id}.{trial_id}.rationale is invalid"
                )
            resolved_trials[trial_id] = dict(override)
        result[map_id] = resolved_trials
    return result


def _runtime_route_selection_overrides(
    profile: Mapping[str, Any], selector: str
) -> dict[str, dict[str, dict[str, Any]]]:
    declared = profile.get("map_trial_route_selection_overrides")
    if selector == "recommended":
        if declared is not None:
            raise MatrixError(
                "recommended profile must not define route-selection overrides"
            )
        return {}
    if selector != "speed_30kph":
        raise MatrixError(f"unknown runtime profile selector: {selector!r}")
    resolved = _validate_route_selection_overrides(
        declared,
        "speed_30kph.map_trial_route_selection_overrides",
    )
    if resolved != SPEED_30KPH_MAP_TRIAL_ROUTE_SELECTION_OVERRIDES:
        raise MatrixError(
            "speed_30kph map/trial route-selection overrides are not pinned"
        )
    return resolved


def _resolved_map_route_selection_overrides(
    profile: Mapping[str, Any], selector: str, map_id: str
) -> dict[str, dict[str, Any]]:
    if not MAP_ID_RE.fullmatch(map_id):
        raise MatrixError(f"route-selection override has an unsafe map id: {map_id!r}")
    overrides = _runtime_route_selection_overrides(profile, selector)
    return {
        trial_id: dict(override)
        for trial_id, override in overrides.get(map_id, {}).items()
    }


def _runtime_turn_route_selection_policy(
    profile: Mapping[str, Any], selector: str
) -> dict[str, Any] | None:
    declared = profile.get("turn_route_selection_policy")
    if selector == "recommended":
        if declared is not None:
            raise MatrixError(
                "recommended profile must not define a turn-route ranking policy"
            )
        return None
    if selector != "speed_30kph":
        raise MatrixError(f"unknown runtime profile selector: {selector!r}")
    if declared != SPEED_30KPH_TURN_ROUTE_SELECTION_POLICY:
        raise MatrixError("speed_30kph turn-route ranking policy is not pinned")
    return dict(declared)


def _validate_speed_30kph_contract(profile: Mapping[str, Any]) -> None:
    _runtime_turn_route_selection_policy(profile, "speed_30kph")
    route_generation = profile.get("route_generation_contracts")
    if not isinstance(route_generation, dict) or set(route_generation) != set(
        TRIAL_IDS
    ):
        raise MatrixError(
            "speed_30kph must define separate straight and turn route-generation "
            "contracts"
        )
    expected_route_generation = {
        "straight": {
            "seeds": [0],
            "pairs_per_seed": 8,
            "minimum_distance_m": 170.0,
            "maximum_distance_m": 260.0,
            "preferred_distance_m": 210.0,
            "maximum_traces_per_scenario": 20000,
        },
        "turn": {
            "seeds": [0],
            "pairs_per_seed": 8,
            "minimum_distance_m": 20.0,
            "maximum_distance_m": 120.0,
            "preferred_distance_m": 60.0,
            "maximum_traces_per_scenario": 20000,
        },
    }
    for trial_id, expected_values in expected_route_generation.items():
        values = route_generation.get(trial_id)
        if not isinstance(values, dict) or set(values) != set(expected_values):
            raise MatrixError(
                f"speed_30kph {trial_id} route-generation fields are not pinned"
            )
        if values.get("seeds") != expected_values["seeds"]:
            raise MatrixError(
                f"speed_30kph {trial_id} route-generation seeds are not pinned"
            )
        for field, expected in expected_values.items():
            if field == "seeds":
                continue
            _require_exact_number(
                values,
                field,
                expected,
                f"speed_30kph.route_generation_contracts.{trial_id}",
            )
    expected_map_overrides = {
        "town10hd_opt": {
            "straight": dict(TOWN10HD_OPT_30KPH_STRAIGHT_OVERRIDE),
        }
    }
    if profile.get("map_route_generation_overrides") != expected_map_overrides:
        raise MatrixError(
            "speed_30kph Town10HD_Opt straight route-generation override is not pinned"
        )
    contract = profile.get("speed_contract")
    if not isinstance(contract, dict):
        raise MatrixError("speed_30kph runtime profile lacks speed_contract")
    if contract.get("profile_id") != "carla_vad_30kph_v2":
        raise MatrixError("speed_30kph speed_contract.profile_id is not pinned")
    if contract.get("longitudinal_speed_source") != "explicit_simulation_profile":
        raise MatrixError("speed_30kph longitudinal speed source is not pinned")
    if contract.get("validation_state") != "carla_30kph_v2_screening":
        raise MatrixError("speed_30kph validation state is not pinned")
    if contract.get("longitudinal_acceleration_role") != (
        "trajectory_internal_curve_exit_cap"
    ):
        raise MatrixError("speed_30kph longitudinal acceleration role is not pinned")
    if contract.get("vad_geometry_source") is not True:
        raise MatrixError("speed_30kph must retain VAD as the geometry source")
    if contract.get("vad_velocity_evaluated") is not False:
        raise MatrixError("speed_30kph must not claim VAD velocity evaluation")
    if contract.get("vad_geometry_evaluated") is not True:
        raise MatrixError("speed_30kph must retain VAD geometry evaluation")
    if contract.get("vad_cruise_velocity_evaluated") is not False:
        raise MatrixError("speed_30kph must not evaluate VAD cruise velocity")
    if contract.get("vad_hard_stop_sentinel_preserved") is not True:
        raise MatrixError("speed_30kph must preserve the VAD hard-stop sentinel")
    _require_exact_number(
        contract, "target_speed_mps", 8.333333333333334, "speed_contract"
    )
    _require_exact_number(
        contract, "maximum_observed_speed_mps", 9.0, "speed_contract"
    )
    _require_exact_number(
        contract, "maximum_speed_sample_gap_sec", 0.25, "speed_contract"
    )
    parameters = contract.get("route_manager_parameters")
    if not isinstance(parameters, dict):
        raise MatrixError("speed_contract.route_manager_parameters must be an object")
    if parameters.get("longitudinal_velocity_source") != (
        "explicit_simulation_nominal"
    ):
        raise MatrixError("speed_contract route-manager velocity source is not pinned")
    expected_parameters = {
        "nominal_cruise_speed_mps": 8.333333333333334,
        "maximum_speed_mps": 8.333333333333334,
        "controller_stop_offset_m": 0.60,
        "comfortable_deceleration_mps2": 2.0,
        "maximum_longitudinal_acceleration_mps2": 1.5,
        "maximum_lateral_acceleration_mps2": 1.2,
        "maneuver_lookahead_m": 4.0,
        "maneuver_exit_lookahead_m": 2.5,
        "route_curvature_lookahead_m": 20.0,
        "curvature_speed_preview_m": 3.0,
        "max_route_deviation_m": 1.0,
        "max_candidate_age_sec": 0.5,
        "candidate_timeout_sec": 1.5,
    }
    for field, expected in expected_parameters.items():
        _require_exact_number(
            parameters, field, expected, "speed_contract.route_manager_parameters"
        )
    policy = _runtime_turn_route_selection_policy(profile, "speed_30kph")
    assert policy is not None
    if policy["parameter_sources"] != (
        SPEED_30KPH_TURN_ROUTE_SELECTION_POLICY["parameter_sources"]
    ):
        raise MatrixError("speed_30kph turn-route parameter sources changed")
    gate = contract.get("vehicle_cmd_gate")
    if not isinstance(gate, dict):
        raise MatrixError("speed_contract.vehicle_cmd_gate must be an object")
    if gate.get("profile_id") != "carla_vad_30kph_v2":
        raise MatrixError("speed_contract vehicle-cmd-gate profile is not pinned")
    if gate.get("speed_limit_source") != "explicit_simulation_profile":
        raise MatrixError("speed_contract vehicle-cmd-gate source is not pinned")
    if gate.get("real_vehicle_ready") is not False:
        raise MatrixError("speed_contract must remain explicitly not real-vehicle ready")
    for field in ("parameter_sha256", "metadata_sha256", "source_sha256"):
        if not re.fullmatch(r"[0-9a-f]{64}", str(gate.get(field, ""))):
            raise MatrixError(f"speed_contract vehicle-cmd-gate {field} is invalid")
    if not re.fullmatch(
        r"[0-9a-f]{40}", str(gate.get("source_repository_commit", ""))
    ):
        raise MatrixError("speed_contract vehicle-cmd-gate source commit is invalid")
    _require_exact_number(
        gate, "velocity_limit_mps", 8.333333333333334, "speed_contract.vehicle_cmd_gate"
    )
    _require_exact_number(
        gate,
        "longitudinal_acceleration_limit_mps2",
        1.5,
        "speed_contract.vehicle_cmd_gate",
    )
    _require_exact_number(
        gate,
        "lateral_acceleration_limit_mps2",
        1.8,
        "speed_contract.vehicle_cmd_gate",
    )
    controller = contract.get("longitudinal_controller")
    if not isinstance(controller, dict):
        raise MatrixError(
            "speed_contract.longitudinal_controller must be an object"
        )
    if controller.get("profile_id") != "carla_vad_30kph_v2":
        raise MatrixError("speed_contract longitudinal-controller profile is not pinned")
    if controller.get("speed_limit_source") != "explicit_simulation_profile":
        raise MatrixError("speed_contract longitudinal-controller source is not pinned")
    if controller.get("real_vehicle_ready") is not False:
        raise MatrixError(
            "speed_contract longitudinal controller is not real-vehicle ready"
        )
    for field in ("parameter_sha256", "metadata_sha256", "source_sha256"):
        if not re.fullmatch(r"[0-9a-f]{64}", str(controller.get(field, ""))):
            raise MatrixError(
                f"speed_contract longitudinal-controller {field} is invalid"
            )
    if not re.fullmatch(
        r"[0-9a-f]{40}", str(controller.get("source_repository_commit", ""))
    ):
        raise MatrixError(
            "speed_contract longitudinal-controller source commit is invalid"
        )
    for field in (
        "maximum_output_mps2",
        "maximum_proportional_effort_mps2",
        "command_gate_longitudinal_acceleration_cap_mps2",
    ):
        _require_exact_number(
            controller,
            field,
            1.5,
            "speed_contract.longitudinal_controller",
        )
    trials = contract.get("trials")
    if not isinstance(trials, dict):
        raise MatrixError("speed_contract.trials must be an object")
    straight = trials.get("straight")
    turn = trials.get("turn")
    if not isinstance(straight, dict) or not isinstance(turn, dict):
        raise MatrixError("speed_contract must define straight and turn contracts")
    if straight.get("exposure_mode") != "straight_target_required":
        raise MatrixError("speed_contract straight exposure mode is not pinned")
    _require_exact_number(
        straight, "minimum_sustained_speed_mps", 7.5, "speed_contract.trials.straight"
    )
    _require_exact_number(
        straight, "minimum_sustained_speed_sec", 1.0, "speed_contract.trials.straight"
    )
    geometry = straight.get("physical_geometry")
    if not isinstance(geometry, dict) or set(geometry) != set(
        SPEED_30KPH_STRAIGHT_GEOMETRY_CONTRACT
    ):
        raise MatrixError(
            "speed_contract straight physical-geometry fields are not pinned"
        )
    for field, expected in SPEED_30KPH_STRAIGHT_GEOMETRY_CONTRACT.items():
        _require_exact_number(
            geometry,
            field,
            expected,
            "speed_contract.trials.straight.physical_geometry",
        )
    _require_exact_number(
        straight,
        "maximum_lateral_acceleration_mps2",
        1.8,
        "speed_contract.trials.straight",
    )
    if turn.get("exposure_mode") != "curvature_limited_turn":
        raise MatrixError("speed_contract turn exposure mode is not pinned")
    _require_exact_number(
        turn, "minimum_sustained_speed_mps", 0.0, "speed_contract.trials.turn"
    )
    _require_exact_number(
        turn, "minimum_sustained_speed_sec", 0.0, "speed_contract.trials.turn"
    )
    _require_exact_number(
        turn,
        "maximum_lateral_acceleration_p95_mps2",
        1.5,
        "speed_contract.trials.turn",
    )
    _require_exact_number(
        turn,
        "maximum_lateral_acceleration_mps2",
        1.8,
        "speed_contract.trials.turn",
    )
    if turn.get("require_selected_turn_command") is not True:
        raise MatrixError("speed_contract turn command evidence must remain required")
    turn_geometry = turn.get("physical_geometry")
    if not isinstance(turn_geometry, dict) or turn_geometry != (
        SPEED_30KPH_TURN_GEOMETRY_CONTRACT
    ):
        raise MatrixError(
            "speed_contract turn physical-geometry contract is not pinned"
        )


def _validate_runtime_profile(
    profile: Any, selector: str
) -> Mapping[str, Any]:
    if not isinstance(profile, dict):
        raise MatrixError(f"runtime profile {selector!r} must be an object")
    _require_keys(
        profile,
        (
            "id",
            "trial_wrapper",
            "wrapper_options",
            "ready_timeout_sec",
            "launch_arguments",
            "map_lifecycle",
            "client_map_loading_allowed",
        ),
        f"runtime profile {selector!r}",
    )
    expected_options = RUNTIME_WRAPPER_OPTIONS[selector]
    if profile.get("wrapper_options") != expected_options:
        raise MatrixError(
            f"runtime profile {selector!r} must fix wrapper_options to "
            f"{expected_options}"
        )
    if profile.get("trial_wrapper") != "run_recorded_route_trial.sh":
        raise MatrixError(f"runtime profile {selector!r} has an unknown trial wrapper")
    if profile.get("client_map_loading_allowed") is not False:
        raise MatrixError("client-side CARLA map loading must remain disabled")
    if profile.get("map_lifecycle") != (
        "cold_start_owned_process_group_per_trial"
    ):
        raise MatrixError(
            "matrix requires an owned per-trial cold-start CARLA lifecycle"
        )
    ready_timeout = _finite_number(
        profile.get("ready_timeout_sec"),
        f"runtime profile {selector!r}.ready_timeout_sec",
    )
    if ready_timeout <= 0.0:
        raise MatrixError("runtime-profile ready timeout must be positive")
    launch_arguments = profile.get("launch_arguments")
    if not isinstance(launch_arguments, list) or not all(
        isinstance(item, str) and item for item in launch_arguments
    ):
        raise MatrixError("runtime-profile launch_arguments must be strings")
    _runtime_route_selection_overrides(profile, selector)
    _runtime_turn_route_selection_policy(profile, selector)
    if selector == "speed_30kph":
        _validate_speed_30kph_contract(profile)
    return profile


def select_runtime_profile(
    matrix: Mapping[str, Any], selector: str = "recommended"
) -> Mapping[str, Any]:
    if selector not in RUNTIME_PROFILE_SELECTORS:
        raise MatrixError(f"unknown runtime profile selector: {selector!r}")
    if selector == "recommended":
        profile = matrix.get("runtime_profile")
    else:
        profiles = matrix.get("runtime_profiles")
        if not isinstance(profiles, dict):
            raise MatrixError("matrix manifest has no opt-in runtime_profiles")
        profile = profiles.get(selector)
    return _validate_runtime_profile(profile, selector)


def load_matrix(path: Path = DEFAULT_MATRIX) -> tuple[dict[str, Any], Path]:
    path = path.expanduser().resolve()
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise MatrixError(f"cannot read matrix manifest {path}: {error}") from error
    if not isinstance(value, dict) or value.get("schema_version") != 1:
        raise MatrixError("matrix manifest must be a schema_version 1 object")
    _require_keys(
        value,
        (
            "matrix_id",
            "canonical_map_manifest",
            "runtime_profile",
            "route_contract",
            "validated_full_map_bundles",
        ),
        "matrix manifest",
    )
    select_runtime_profile(value, "recommended")
    select_runtime_profile(value, "speed_30kph")
    trials = value["route_contract"].get("trials")
    if not isinstance(trials, list) or [item.get("id") for item in trials] != list(
        TRIAL_IDS
    ):
        raise MatrixError("route_contract must define straight then turn trials")
    _route_generation_contracts(
        value["route_contract"], value["runtime_profile"], "recommended"
    )
    _route_generation_contracts(
        value["route_contract"],
        value["runtime_profiles"]["speed_30kph"],
        "speed_30kph",
    )
    bundles = value["validated_full_map_bundles"]
    if not isinstance(bundles, dict) or any(
        not MAP_ID_RE.fullmatch(str(map_id)) for map_id in bundles
    ):
        raise MatrixError("validated_full_map_bundles has an unsafe map id")
    return value, path


def _route_generation_contracts(
    route_contract: Any,
    profile: Any,
    selector: str,
) -> dict[str, dict[str, Any]]:
    """Materialize immutable profile-by-scenario catalog-generation inputs."""
    if not isinstance(route_contract, dict):
        raise MatrixError("route_contract must be an object")
    _require_keys(
        route_contract,
        (
            "weather",
            "seed",
            "pairs_per_seed",
            *ROUTE_GENERATION_DISTANCE_FIELDS,
            "sampling_resolution_m",
            "maximum_endpoint_offset_m",
            "maximum_traces_per_scenario",
        ),
        "route_contract",
    )
    if not isinstance(profile, dict):
        raise MatrixError("runtime profile must be an object")

    weather = route_contract.get("weather")
    seed = route_contract.get("seed")
    pairs = route_contract.get("pairs_per_seed")
    max_traces = route_contract.get("maximum_traces_per_scenario")
    if not isinstance(weather, str) or not weather:
        raise MatrixError("route_contract.weather must be a non-empty string")
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise MatrixError("route_contract.seed must be a non-negative integer")
    if isinstance(pairs, bool) or not isinstance(pairs, int) or pairs < 1:
        raise MatrixError("route_contract.pairs_per_seed must be a positive integer")
    if (
        isinstance(max_traces, bool)
        or not isinstance(max_traces, int)
        or max_traces < 1
    ):
        raise MatrixError(
            "route_contract.maximum_traces_per_scenario must be a positive integer"
        )

    base = {
        "weather": route_contract["weather"],
        "seeds": [seed],
        "pairs_per_seed": route_contract["pairs_per_seed"],
        "minimum_distance_m": route_contract["minimum_distance_m"],
        "maximum_distance_m": route_contract["maximum_distance_m"],
        "preferred_distance_m": route_contract["preferred_distance_m"],
        "sampling_resolution_m": route_contract["sampling_resolution_m"],
        "maximum_endpoint_offset_m": route_contract[
            "maximum_endpoint_offset_m"
        ],
        "maximum_traces_per_scenario": route_contract[
            "maximum_traces_per_scenario"
        ],
    }
    overrides = profile.get("route_generation_contracts")
    if selector == "recommended":
        if overrides is not None:
            raise MatrixError(
                "recommended profile must retain the shared baseline route contract"
            )
        overrides = {}
    elif not isinstance(overrides, dict) or set(overrides) != set(TRIAL_IDS):
        raise MatrixError(
            f"runtime profile {selector!r} lacks per-trial route generation contracts"
        )

    result: dict[str, dict[str, Any]] = {}
    for trial_id in TRIAL_IDS:
        contract = dict(base)
        override = overrides.get(trial_id, {})
        override_fields = {
            "seeds",
            "pairs_per_seed",
            *ROUTE_GENERATION_DISTANCE_FIELDS,
            "maximum_traces_per_scenario",
        }
        if not isinstance(override, dict) or any(
            field not in override_fields for field in override
        ):
            raise MatrixError(
                f"runtime profile {selector!r} has invalid {trial_id} route overrides"
            )
        contract.update(override)
        seeds = contract.get("seeds")
        if (
            not isinstance(seeds, list)
            or not seeds
            or any(
                isinstance(item, bool) or not isinstance(item, int) or item < 0
                for item in seeds
            )
            or len(set(seeds)) != len(seeds)
        ):
            raise MatrixError(
                f"{selector} {trial_id} route seeds must be unique non-negative integers"
            )
        scenario_pairs = contract.get("pairs_per_seed")
        scenario_traces = contract.get("maximum_traces_per_scenario")
        if (
            isinstance(scenario_pairs, bool)
            or not isinstance(scenario_pairs, int)
            or scenario_pairs < 1
            or isinstance(scenario_traces, bool)
            or not isinstance(scenario_traces, int)
            or scenario_traces < 1
        ):
            raise MatrixError(
                f"{selector} {trial_id} route search budget is invalid"
            )
        minimum = _finite_number(
            contract["minimum_distance_m"],
            f"{selector}.{trial_id}.minimum_distance_m",
        )
        maximum = _finite_number(
            contract["maximum_distance_m"],
            f"{selector}.{trial_id}.maximum_distance_m",
        )
        preferred = _finite_number(
            contract["preferred_distance_m"],
            f"{selector}.{trial_id}.preferred_distance_m",
        )
        sampling = _finite_number(
            contract["sampling_resolution_m"],
            f"{selector}.{trial_id}.sampling_resolution_m",
        )
        endpoint = _finite_number(
            contract["maximum_endpoint_offset_m"],
            f"{selector}.{trial_id}.maximum_endpoint_offset_m",
        )
        if minimum <= 0.0 or maximum < minimum:
            raise MatrixError(
                f"{selector} {trial_id} route distance window is invalid"
            )
        if preferred < minimum or preferred > maximum:
            raise MatrixError(
                f"{selector} {trial_id} preferred route distance is outside its window"
            )
        if sampling <= 0.0 or endpoint < 0.0:
            raise MatrixError(
                f"{selector} {trial_id} route sampling/endpoint contract is invalid"
            )
        result[trial_id] = contract
    return result


def _town10hd_opt_straight_capacity_contract() -> dict[str, Any]:
    return {
        "enabled": True,
        "profile_id": TOWN10HD_OPT_30KPH_COMPACT_PROFILE,
        "map_id": "town10hd_opt",
        "scenario": "straight",
        "endpoint_waypoint_spacing_m": 0.5,
        "endpoint_junction_policy": "exclude",
        "candidate_enumeration_policy": "directed_topology_straight_v1",
        "admission_policy": (
            "prefilter_then_exact_serialized_physical_postfilter"
        ),
        "provenance": dict(TOWN10HD_OPT_30KPH_CAPACITY_PROVENANCE),
    }


def _resolved_map_route_generation_contracts(
    route_contract: Any,
    profile: Any,
    selector: str,
    map_id: str,
) -> dict[str, dict[str, Any]]:
    contracts = _route_generation_contracts(route_contract, profile, selector)
    if selector != "speed_30kph" or map_id != "town10hd_opt":
        return contracts
    if not isinstance(profile, Mapping):
        raise MatrixError("speed_30kph runtime profile must be an object")
    overrides = profile.get("map_route_generation_overrides")
    expected = {
        "town10hd_opt": {
            "straight": dict(TOWN10HD_OPT_30KPH_STRAIGHT_OVERRIDE),
        }
    }
    if overrides != expected:
        raise MatrixError(
            "Town10HD_Opt map-specific route-generation override changed"
        )
    straight = dict(contracts["straight"])
    straight.update(TOWN10HD_OPT_30KPH_STRAIGHT_OVERRIDE)
    straight["straight_capacity_contract"] = (
        _town10hd_opt_straight_capacity_contract()
    )
    return {"straight": straight, "turn": dict(contracts["turn"])}


def _canonical_manifest_path(matrix: Mapping[str, Any], matrix_path: Path) -> Path:
    value = matrix["canonical_map_manifest"]
    if not isinstance(value, str) or not value:
        raise MatrixError("canonical_map_manifest must be a path")
    path = Path(value)
    if not path.is_absolute():
        path = matrix_path.parent / path
    return path.resolve()


def _bundle_path(spec: Mapping[str, Any], matrix_path: Path) -> Path:
    value = spec.get("path")
    if not isinstance(value, str) or not value:
        raise MatrixError("validated bundle path must be a non-empty string")
    path = Path(value)
    if not path.is_absolute():
        path = matrix_path.parent / path
    return path.resolve()


def _manifest_relative_path(
    spec: Mapping[str, Any], field: str, matrix_path: Path
) -> Path:
    value = spec.get(field)
    if not isinstance(value, str) or not value:
        raise MatrixError(f"{field} must be a non-empty path")
    path = Path(value)
    if not path.is_absolute():
        path = matrix_path.parent / path
    return path.resolve()


def _packaged_town_readiness(
    map_entry: Mapping[str, Any],
    spec: Mapping[str, Any],
    matrix_path: Path,
    bundle_dir: Path,
) -> dict[str, Any]:
    readiness_path = _manifest_relative_path(spec, "readiness_artifact", matrix_path)
    readiness = read_object(readiness_path, "packaged Town readiness")
    records = readiness.get("maps")
    if not isinstance(records, list):
        raise MatrixError("packaged Town readiness has no map records")
    record = next(
        (
            item
            for item in records
            if isinstance(item, dict) and item.get("id") == map_entry["id"]
        ),
        None,
    )
    if record is None:
        raise MatrixError(f"readiness has no record for {map_entry['id']}")
    prefix = spec.get("readiness_status_prefix")
    if not isinstance(prefix, str) or not str(record.get("status", "")).startswith(prefix):
        raise MatrixError(
            f"Town readiness status is not an admission candidate: {record.get('status')!r}"
        )
    bundle = record.get("bundle")
    if not isinstance(bundle, dict) or bundle.get("complete") is not True:
        raise MatrixError("Town readiness does not mark the full-map bundle complete")
    if Path(str(bundle.get("path", ""))).resolve() != bundle_dir:
        raise MatrixError("Town readiness points to a different full-map bundle")
    preflight = record.get("route_preflight")
    if not isinstance(preflight, dict) or preflight.get("status") != "PASS":
        raise MatrixError("Town route/map preflight did not pass")
    cases = preflight.get("cases")
    if not isinstance(cases, list):
        raise MatrixError("Town route/map preflight has no cases")
    admitted_routes: list[dict[str, Any]] = []
    for case in cases:
        if not isinstance(case, dict) or case.get("scenario") not in (
            "straight",
            "left",
            "right",
        ):
            continue
        proximity = case.get("pointcloud_proximity")
        if (
            case.get("status") != "PASS"
            or not isinstance(proximity, dict)
            or proximity.get("status") != "PASS"
        ):
            raise MatrixError(
                f"Town {case.get('scenario')} route lacks a passing map/PCD preflight"
            )
        route_value = case.get("route")
        if not isinstance(route_value, str) or not route_value:
            raise MatrixError("Town readiness case has no route path")
        route_path = Path(route_value)
        if not route_path.is_absolute():
            route_path = ROOT / route_path
        route_path = route_path.resolve()
        if not route_path.is_file():
            raise MatrixError(f"Town readiness route is missing: {route_path}")
        admitted_routes.append(
            {
                "scenario": case["scenario"],
                "route_path": str(route_path),
                "route_sha256": sha256_file(route_path),
                "maximum_lanelet_distance_m": case.get(
                    "maximum_lanelet_distance_m"
                ),
                "maximum_vertical_distance_m": case.get(
                    "maximum_vertical_distance_m"
                ),
                "pointcloud_proximity": proximity,
            }
        )
    scenarios = {item["scenario"] for item in admitted_routes}
    if "straight" not in scenarios or not scenarios.intersection({"left", "right"}):
        raise MatrixError("Town readiness lacks both straight and turn map/PCD preflights")
    return {
        "artifact_path": str(readiness_path),
        "artifact_sha256": sha256_file(readiness_path),
        "status": record["status"],
        "alignment_status": (
            record.get("alignment", {}).get("status")
            if isinstance(record.get("alignment"), dict)
            else None
        ),
        "route_preflight_status": "PASS",
        "admitted_routes": admitted_routes,
        "scope": (
            "structural Lanelet2 + transformed-PCD + exact-route admission only; "
            "not an Autoware VAD execution result"
        ),
    }


def validate_full_map_bundle(
    map_entry: Mapping[str, Any], spec: Mapping[str, Any], matrix_path: Path
) -> dict[str, Any]:
    bundle_dir = _bundle_path(spec, matrix_path)
    if not bundle_dir.is_dir():
        raise MatrixError(f"full-map bundle directory is missing: {bundle_dir}")
    required = {
        "metadata": bundle_dir / "map_bundle.json",
        "lanelet2": bundle_dir / "lanelet2_map.osm",
        "pointcloud": bundle_dir / "pointcloud_map.pcd",
        "projector": bundle_dir / "map_projector_info.yaml",
    }
    for label, path in required.items():
        if not path.is_file():
            raise MatrixError(f"full-map bundle is missing {label}: {path}")
    metadata = read_object(required["metadata"], "map bundle")
    accepted = spec.get("accepted_statuses")
    if not isinstance(accepted, list) or not accepted:
        raise MatrixError("accepted_statuses must be a non-empty list")
    if metadata.get("status") not in accepted:
        raise MatrixError(
            f"bundle status {metadata.get('status')!r} is not accepted for live use"
        )
    for field in ("canonical_carla_map", "profile"):
        if metadata.get(field) != spec.get(field):
            raise MatrixError(
                f"bundle {field} mismatch: expected={spec.get(field)!r} "
                f"actual={metadata.get(field)!r}"
            )
    if metadata.get("canonical_carla_map") != map_entry.get("load_name"):
        raise MatrixError("bundle canonical CARLA map does not match suite load_name")
    schema = spec.get("bundle_schema")
    if schema == "custom_map":
        inspection = metadata.get("structural_inspection")
        if not isinstance(inspection, dict):
            raise MatrixError("custom bundle has no structural inspection")
        lanelet = inspection.get("lanelet2")
        pcd = inspection.get("pcd")
        sources = metadata.get("bundle_sources")
        if not isinstance(sources, dict):
            raise MatrixError("custom bundle source provenance is missing")
        source_records = {
            "lanelet2_map": sources.get("lanelet2_map"),
            "pointcloud_map": sources.get("pointcloud_map"),
        }
        readiness = None
    elif schema == "packaged_town":
        lanelet_record = metadata.get("lanelet2")
        pcd_record = metadata.get("pointcloud_source")
        generated = metadata.get("pointcloud_generated")
        if not isinstance(lanelet_record, dict) or not isinstance(pcd_record, dict):
            raise MatrixError("packaged Town bundle inspection is missing")
        lanelet = lanelet_record.get("inspection")
        pcd = pcd_record.get("inspection")
        source_records = {
            "lanelet2_map": lanelet_record.get("file"),
            "pointcloud_map": generated,
        }
        alignment = metadata.get("alignment")
        accepted_alignment = spec.get("accepted_alignment_statuses")
        if (
            not isinstance(alignment, dict)
            or not isinstance(accepted_alignment, list)
            or alignment.get("status") not in accepted_alignment
        ):
            raise MatrixError(
                f"packaged Town alignment status is not accepted: "
                f"{alignment.get('status') if isinstance(alignment, dict) else None!r}"
            )
        readiness = _packaged_town_readiness(
            map_entry, spec, matrix_path, bundle_dir
        )
    else:
        raise MatrixError(f"unknown full-map bundle schema: {schema!r}")
    if not isinstance(lanelet, dict) or int(lanelet.get("road_lanelets", 0)) <= 0:
        raise MatrixError("bundle has no inspected road lanelets")
    if not isinstance(pcd, dict) or int(pcd.get("points", 0)) <= 0:
        raise MatrixError("bundle has no inspected point-cloud points")
    fresh_hashes: dict[str, str] = {}
    for source_id, local_key in (
        ("lanelet2_map", "lanelet2"),
        ("pointcloud_map", "pointcloud"),
    ):
        source = source_records.get(source_id)
        if not isinstance(source, dict):
            raise MatrixError(f"bundle source {source_id} is missing")
        expected_hash = source.get("sha256") or source.get("expected_sha256")
        expected_size = source.get("size_bytes") or source.get(
            "expected_size_bytes"
        )
        local_path = required[local_key]
        if local_path.stat().st_size != expected_size:
            raise MatrixError(f"bundle source size mismatch: {local_path}")
        actual_hash = sha256_file(local_path)
        if actual_hash != expected_hash:
            raise MatrixError(f"bundle source SHA256 mismatch: {local_path}")
        fresh_hashes[source_id] = actual_hash
    return {
        "path": str(bundle_dir),
        "metadata_path": str(required["metadata"]),
        "metadata_sha256": sha256_file(required["metadata"]),
        "bundle_file_sha256": {
            "metadata": sha256_file(required["metadata"]),
            "lanelet2": fresh_hashes["lanelet2_map"],
            "pointcloud": fresh_hashes["pointcloud_map"],
            "projector": sha256_file(required["projector"]),
        },
        "status": metadata["status"],
        "profile": metadata["profile"],
        "canonical_carla_map": metadata["canonical_carla_map"],
        "bundle_schema": schema,
        "fresh_source_sha256": fresh_hashes,
        "road_lanelets": int(lanelet["road_lanelets"]),
        "pcd_points": int(pcd["points"]),
        "readiness": readiness,
    }


def build_campaign_plan(
    matrix: Mapping[str, Any], matrix_path: Path, runtime_profile: str = "recommended"
) -> dict[str, Any]:
    selected_profile = select_runtime_profile(matrix, runtime_profile)
    execution_contract = _campaign_execution_contract()
    generation_contracts = _route_generation_contracts(
        matrix["route_contract"], selected_profile, runtime_profile
    )
    selection_overrides = _runtime_route_selection_overrides(
        selected_profile, runtime_profile
    )
    turn_selection_policy = _runtime_turn_route_selection_policy(
        selected_profile, runtime_profile
    )
    canonical_path = _canonical_manifest_path(matrix, matrix_path)
    canonical, _ = load_carla_manifest(canonical_path)
    bundles = matrix["validated_full_map_bundles"]
    known_blockers = matrix.get("known_blockers", {})
    if not isinstance(known_blockers, dict):
        raise MatrixError("known_blockers must be an object")
    maps: list[dict[str, Any]] = []
    for map_entry in canonical["maps"]:
        map_id = map_entry["id"]
        base = {
            "map_id": map_id,
            "canonical_name": map_entry["canonical_name"],
            "load_name": map_entry["load_name"],
            "suite_status": map_entry["status"],
            "server_profile": map_entry["server_profile"],
            "route_generation_contracts": (
                _resolved_map_route_generation_contracts(
                    matrix["route_contract"],
                    selected_profile,
                    runtime_profile,
                    map_id,
                )
            ),
            "route_selection_overrides": (
                _resolved_map_route_selection_overrides(
                    selected_profile, runtime_profile, map_id
                )
            ),
        }
        base["route_selection_overrides_sha256"] = sha256_json(
            base["route_selection_overrides"]
        )
        spec = bundles.get(map_id)
        if spec is None:
            blocker = known_blockers.get(map_id)
            blocker_code = (
                blocker.get("code") if isinstance(blocker, dict) else None
            )
            blocker_reason = (
                blocker.get("reason") if isinstance(blocker, dict) else None
            )
            base.update(
                {
                    "runnable": False,
                    "status": "BLOCKED",
                    "block_code": blocker_code
                    or "validated_full_map_bundle_missing",
                    "reason": (
                        blocker_reason
                        or (
                            "No locally validated Lanelet2 + PCD full-map bundle is "
                            "declared; CARLA/BasicAgent assets alone do not authorize "
                            "an Autoware VAD closed-loop claim. "
                            f"Suite status={map_entry['status']}: {map_entry['reason']}"
                        )
                    ),
                    "full_map_bundle": None,
                }
            )
        elif map_entry.get("status") != "ready":
            base.update(
                {
                    "runnable": False,
                    "status": "BLOCKED",
                    "block_code": "carla_map_not_runtime_ready",
                    "reason": (
                        f"CARLA suite status is {map_entry.get('status')}: "
                        f"{map_entry.get('reason')}"
                    ),
                    "full_map_bundle": None,
                }
            )
        else:
            try:
                bundle = validate_full_map_bundle(map_entry, spec, matrix_path)
            except MatrixError as error:
                base.update(
                    {
                        "runnable": False,
                        "status": "BLOCKED",
                        "block_code": "full_map_bundle_validation_failed",
                        "reason": str(error),
                        "full_map_bundle": None,
                    }
                )
            else:
                readiness = bundle.get("readiness")
                if isinstance(readiness, dict):
                    admission_reason = (
                        "Exact straight/turn Lanelet2 + PCD route preflight admitted "
                        "this packaged Town for execution; this is not yet a VAD "
                        f"PASS. Global alignment status={readiness.get('alignment_status')}."
                    )
                else:
                    admission_reason = (
                        "Live-validated custom full-map bundle admitted for matrix execution."
                    )
                base.update(
                    {
                        "runnable": True,
                        "status": "PENDING",
                        "block_code": None,
                        "reason": admission_reason,
                        "full_map_bundle": bundle,
                    }
                )
        maps.append(base)
    plan = {
        "schema_version": 1,
        "matrix_id": matrix["matrix_id"],
        "generated_at": utc_now(),
        "matrix_manifest": str(matrix_path),
        "matrix_manifest_sha256": sha256_file(matrix_path),
        "canonical_map_manifest": str(canonical_path),
        "canonical_map_manifest_sha256": sha256_file(canonical_path),
        "runtime_profile_selector": runtime_profile,
        "runtime_profile": selected_profile,
        "route_contract": matrix["route_contract"],
        "route_generation_contracts": generation_contracts,
        "route_selection_policy": ROUTE_SELECTION_POLICY,
        "route_selection_overrides": selection_overrides,
        "route_selection_overrides_sha256": sha256_json(selection_overrides),
        "turn_route_selection_policy": turn_selection_policy,
        "turn_route_selection_policy_sha256": (
            sha256_json(turn_selection_policy)
            if turn_selection_policy is not None
            else None
        ),
        "campaign_execution_contract": execution_contract,
        "campaign_execution_contract_sha256": sha256_json(
            execution_contract
        ),
        "canonical_map_count": len(maps),
        "runnable_map_count": sum(item["runnable"] for item in maps),
        "maps": maps,
    }
    plan["admission_contract_sha256"] = sha256_json(
        _admission_contract_payload(plan)
    )
    _verify_campaign_plan(plan, "new campaign plan")
    return plan


def _status_path(output_root: Path, map_id: str) -> Path:
    if not MAP_ID_RE.fullmatch(map_id):
        raise MatrixError(f"unsafe map id: {map_id}")
    return output_root / "maps" / map_id / "status.json"


def prepare_output(
    matrix_path: Path,
    output_root: Path,
    resume: bool,
    runtime_profile: str = "recommended",
) -> dict[str, Any]:
    matrix, matrix_path = load_matrix(matrix_path)
    candidate = build_campaign_plan(matrix, matrix_path, runtime_profile)
    output_root = output_root.expanduser().resolve()
    plan_path = output_root / "matrix_plan.json"
    if plan_path.exists():
        if not resume:
            raise MatrixError(f"matrix output exists; pass --resume: {output_root}")
        existing = read_object(plan_path, "existing matrix plan")
        _verify_campaign_plan(existing, "existing matrix plan")
        for field in (
            "matrix_id",
            "matrix_manifest_sha256",
            "canonical_map_manifest_sha256",
            "campaign_execution_contract_sha256",
            "admission_contract_sha256",
        ):
            if existing.get(field) != candidate.get(field):
                raise MatrixError(
                    f"resume contract changed at {field}; use a new output root"
                )
        plan = existing
    else:
        output_root.mkdir(parents=True, exist_ok=True)
        atomic_json(plan_path, candidate)
        plan = _load_verified_campaign_plan(output_root, "written campaign plan")
    for entry in plan["maps"]:
        _verify_campaign_plan(
            plan, f"campaign plan before preparing map {entry['map_id']}"
        )
        status_path = _status_path(output_root, entry["map_id"])
        if status_path.exists():
            continue
        status = {
            "schema_version": 1,
            "matrix_id": plan["matrix_id"],
            "map_id": entry["map_id"],
            "canonical_name": entry["canonical_name"],
            "runnable": entry["runnable"],
            "status": entry["status"],
            "stage": "admission" if not entry["runnable"] else "pending",
            "reason": entry["reason"],
            "block_code": entry["block_code"],
            "updated_at": utc_now(),
            "trials": {
                trial_id: {
                    "status": "BLOCKED" if not entry["runnable"] else "PENDING",
                    "reason": entry["reason"] if not entry["runnable"] else None,
                    "attempt_directory": None,
                    "validation": None,
                }
                for trial_id in TRIAL_IDS
            },
        }
        atomic_json(status_path, status)
    summarize(output_root)
    return plan


def _map_entry(plan: Mapping[str, Any], map_id: str) -> Mapping[str, Any]:
    for entry in plan.get("maps", []):
        if entry.get("map_id") == map_id:
            return entry
    raise MatrixError(f"unknown matrix map id: {map_id}")


def _normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _linear_percentile(values: Sequence[float], percentile: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return 0.0
    position = (len(ordered) - 1) * percentile / 100.0
    lower = int(math.floor(position))
    upper = min(lower + 1, len(ordered) - 1)
    fraction = position - lower
    return ordered[lower] + fraction * (ordered[upper] - ordered[lower])


def _first_progress_crossing(
    samples: Sequence[tuple[float, float]], threshold: float
) -> float | None:
    for (start_distance, start_value), (end_distance, end_value) in zip(
        samples, samples[1:]
    ):
        if start_value >= threshold:
            return start_distance
        if end_value < threshold:
            continue
        if end_value <= start_value:
            return end_distance
        ratio = (threshold - start_value) / (end_value - start_value)
        return start_distance + ratio * (end_distance - start_distance)
    if samples and samples[-1][1] >= threshold:
        return samples[-1][0]
    return None


def _speed_30kph_straight_geometry(
    payload: Mapping[str, Any], required: Mapping[str, Any]
) -> dict[str, Any]:
    """Prove that a STRAIGHT-labeled speed route is physically near-straight."""
    if dict(required) != SPEED_30KPH_STRAIGHT_GEOMETRY_CONTRACT:
        raise MatrixError("30 kph physical-straight geometry contract changed")
    try:
        result = analyze_serialized_physical_straight(payload, required)
    except PhysicalStraightGeometryError as error:
        raise MatrixError(str(error)) from error
    if result["status"] != "PASS":
        raise MatrixError(
            "30 kph physical-straight preflight failed: "
            + "; ".join(result["failure_reasons"])
        )
    return result


def _speed_30kph_turn_geometry(
    payload: Mapping[str, Any], required: Mapping[str, Any]
) -> dict[str, Any]:
    """Prove a packaged-Town turn is isolated and starts with LANEFOLLOW."""
    if dict(required) != SPEED_30KPH_TURN_GEOMETRY_CONTRACT:
        raise MatrixError("30 kph physical-turn geometry contract changed")
    try:
        result = analyze_serialized_physical_turn(payload, required)
    except PhysicalTurnGeometryError as error:
        evidence = error.evidence()
        if error.fatal:
            raise MatrixError(
                "fatal 30 kph physical-turn analysis error "
                f"scope={error.error_scope} code={error.error_code}: {error}"
            ) from error
        raise PhysicalTurnCandidateRejection(str(error), evidence) from error
    if result["status"] != "PASS":
        message = (
            "30 kph physical-turn preflight failed: "
            + "; ".join(result["failure_reasons"])
        )
        raise PhysicalTurnCandidateRejection(
            message,
            {
                "error_type": "PhysicalTurnCandidateRejection",
                "error_scope": "candidate",
                "error_code": "physical_turn_contract_failed",
                "fatal": False,
                "message": message,
                "failure_reasons": list(result["failure_reasons"]),
            },
        )
    return result


def _validate_catalog_initial_approach_contract(
    catalog: Mapping[str, Any]
) -> None:
    generation = catalog.get("generation")
    contract = (
        generation.get("initial_approach_contract")
        if isinstance(generation, dict)
        else None
    )
    if not isinstance(contract, dict) or contract.get("enabled") is not True:
        raise MatrixError("custom-map catalog lacks the enabled initial-approach contract")
    for field, expected in CUSTOM_MAP_INITIAL_APPROACH_CONTRACT.items():
        actual = contract.get(field)
        if isinstance(expected, bool):
            matches = actual is expected
        else:
            matches = isinstance(actual, (int, float)) and math.isclose(
                float(actual), float(expected), abs_tol=1.0e-9
            )
        if not matches:
            raise MatrixError(
                f"custom-map catalog initial-approach {field} mismatch: "
                f"{actual!r} != {expected!r}"
            )


def _validate_catalog_turn_geometry_contract(catalog: Mapping[str, Any]) -> None:
    generation = catalog.get("generation")
    contract = (
        generation.get("turn_geometry_contract")
        if isinstance(generation, dict)
        else None
    )
    if not isinstance(contract, dict) or contract.get("enabled") is not True:
        raise MatrixError("custom-map catalog lacks the enabled turn-geometry contract")
    for field, expected in CUSTOM_MAP_TURN_GEOMETRY_CONTRACT.items():
        actual = contract.get(field)
        if isinstance(expected, bool):
            matches = actual is expected
        else:
            matches = isinstance(actual, (int, float)) and math.isclose(
                float(actual), float(expected), abs_tol=1.0e-9
            )
        if not matches:
            raise MatrixError(
                f"custom-map catalog turn-geometry {field} mismatch: "
                f"{actual!r} != {expected!r}"
            )


def _custom_route_initial_approach(
    payload: Mapping[str, Any]
) -> dict[str, Any]:
    """Recompute the custom-map route-start geometry from serialized ROS poses."""
    points = payload.get("route")
    if not isinstance(points, list) or len(points) < 2:
        raise MatrixError("custom-map initial-approach route needs at least two points")
    required = CUSTOM_MAP_INITIAL_APPROACH_CONTRACT
    distance_limit = float(required["distance_m"])

    def pose(point: Mapping[str, Any], index: int) -> tuple[float, float, float, float]:
        values = tuple(point.get(name) for name in ("x", "y", "yaw", "distance_m"))
        if not all(
            isinstance(value, (int, float)) and math.isfinite(float(value))
            for value in values
        ):
            raise MatrixError(
                f"custom-map route point {index} lacks finite x/y/yaw/distance_m"
            )
        return tuple(float(value) for value in values)  # type: ignore[return-value]

    x0, y0, yaw0, first_distance = pose(points[0], 0)
    if not math.isclose(first_distance, 0.0, abs_tol=1.0e-6):
        raise MatrixError("custom-map route must start at distance_m=0 for preflight")
    normal_x = -math.sin(yaw0)
    normal_y = math.cos(yaw0)
    maximum_lateral = 0.0
    maximum_heading = 0.0
    covered_distance = 0.0

    def measure(x: float, y: float, yaw: float) -> None:
        nonlocal maximum_lateral, maximum_heading
        maximum_lateral = max(
            maximum_lateral,
            abs((x - x0) * normal_x + (y - y0) * normal_y),
        )
        maximum_heading = max(
            maximum_heading, abs(math.degrees(_normalize_angle(yaw - yaw0)))
        )

    previous = (x0, y0, yaw0, first_distance)
    for index, item in enumerate(points[1:], start=1):
        if not isinstance(item, dict):
            raise MatrixError(f"custom-map route point {index} is not an object")
        current = pose(item, index)
        if current[3] < previous[3]:
            raise MatrixError("custom-map route distance_m is not monotonic")
        if current[3] >= distance_limit:
            span = current[3] - previous[3]
            ratio = 0.0 if span <= 1.0e-9 else (distance_limit - previous[3]) / span
            yaw_delta = _normalize_angle(current[2] - previous[2])
            measure(
                previous[0] + ratio * (current[0] - previous[0]),
                previous[1] + ratio * (current[1] - previous[1]),
                previous[2] + ratio * yaw_delta,
            )
            covered_distance = distance_limit
            break
        measure(current[0], current[1], current[2])
        covered_distance = current[3]
        previous = current

    reasons = []
    if covered_distance < distance_limit - 1.0e-6:
        reasons.append(
            f"route covers only {covered_distance:.3f} m of the required "
            f"{distance_limit:.3f} m approach"
        )
    if maximum_lateral > float(required["maximum_lateral_deviation_m"]):
        reasons.append(
            f"initial lateral deviation {maximum_lateral:.3f} m exceeds "
            f"{float(required['maximum_lateral_deviation_m']):.3f} m"
        )
    if maximum_heading > float(required["maximum_heading_change_deg"]):
        reasons.append(
            f"initial heading change {maximum_heading:.3f} deg exceeds "
            f"{float(required['maximum_heading_change_deg']):.3f} deg"
        )
    if reasons:
        raise MatrixError("custom-map initial-approach preflight failed: " + "; ".join(reasons))
    return {
        "status": "PASS",
        "distance_m": distance_limit,
        "covered_distance_m": covered_distance,
        "maximum_lateral_deviation_m": maximum_lateral,
        "maximum_heading_change_deg": maximum_heading,
        "limits": {
            "maximum_lateral_deviation_m": float(
                required["maximum_lateral_deviation_m"]
            ),
            "maximum_heading_change_deg": float(
                required["maximum_heading_change_deg"]
            ),
        },
        "failure_reasons": [],
    }


def _custom_route_turn_geometry(
    payload: Mapping[str, Any], scenario: str
) -> dict[str, Any]:
    """Recompute isolated-turn geometry from the exact serialized ROS route."""
    points = payload.get("route")
    try:
        result = analyze_serialized_custom_turn(
            points, scenario, CUSTOM_MAP_TURN_GEOMETRY_CONTRACT
        )
    except SerializedCustomTurnGeometryError as error:
        raise MatrixError(str(error)) from error
    if result["status"] != "PASS":
        raise MatrixError(
            "custom-map turn-geometry preflight failed: "
            + "; ".join(result["failure_reasons"])
        )
    return result


def _validate_goal_endpoint_provenance(
    payload: Mapping[str, Any],
    endpoint_source: str,
    endpoint_index: int,
    terminal: Mapping[str, Any],
) -> dict[str, Any]:
    """Bind the normalized runtime goal to its untouched endpoint Z."""
    provenance = payload.get("goal_endpoint_provenance")
    expected_provenance_fields = {
        "endpoint_source",
        "endpoint_index",
        "original_goal_carla_transform",
        "original_goal_ros_pose",
        "terminal_z_normalization",
    }
    if not isinstance(provenance, dict) or set(provenance) != (
        expected_provenance_fields
    ):
        raise MatrixError("goal endpoint provenance fields are invalid")
    if (
        provenance.get("endpoint_source") != endpoint_source
        or provenance.get("endpoint_index") != endpoint_index
        or isinstance(provenance.get("endpoint_index"), bool)
    ):
        raise MatrixError(
            "goal endpoint provenance source/index differs from route identity"
        )

    carla_fields = {"x", "y", "z", "roll", "pitch", "yaw"}
    ros_fields = {"x", "y", "z", "yaw"}
    original_carla = provenance.get("original_goal_carla_transform")
    original_ros = provenance.get("original_goal_ros_pose")
    goal_carla = payload.get("goal_carla_transform")
    goal_ros = payload.get("goal_ros_pose")
    if (
        not isinstance(original_carla, dict)
        or set(original_carla) != carla_fields
        or not isinstance(goal_carla, dict)
        or set(goal_carla) != carla_fields
        or not isinstance(original_ros, dict)
        or set(original_ros) != ros_fields
        or not isinstance(goal_ros, dict)
        or set(goal_ros) != ros_fields
    ):
        raise MatrixError("goal endpoint CARLA/ROS pose fields are invalid")
    original_carla_values = {
        field: _finite_number(
            original_carla.get(field), f"original goal CARLA {field}"
        )
        for field in sorted(carla_fields)
    }
    goal_carla_values = {
        field: _finite_number(goal_carla.get(field), f"runtime goal CARLA {field}")
        for field in sorted(carla_fields)
    }
    original_ros_values = {
        field: _finite_number(original_ros.get(field), f"original goal ROS {field}")
        for field in sorted(ros_fields)
    }
    goal_ros_values = {
        field: _finite_number(goal_ros.get(field), f"runtime goal ROS {field}")
        for field in sorted(ros_fields)
    }

    normalization = provenance.get("terminal_z_normalization")
    expected_normalization_fields = {
        "policy",
        "original_endpoint_z_m",
        "last_road_waypoint_z_m",
        "runtime_goal_z_m",
        "serialized_terminal_z_m",
        "applied_offset_m",
    }
    if not isinstance(normalization, dict) or set(normalization) != (
        expected_normalization_fields
    ):
        raise MatrixError("goal terminal-Z normalization fields are invalid")
    if normalization.get("policy") != "last_road_waypoint_z":
        raise MatrixError("goal terminal-Z normalization policy is invalid")
    normalized_z = {
        field: _finite_number(
            normalization.get(field), f"goal terminal-Z normalization {field}"
        )
        for field in expected_normalization_fields - {"policy"}
    }
    terminal_values = {
        field: _finite_number(terminal.get(field), f"route terminal {field}")
        for field in ("x", "y", "z", "yaw")
    }

    def same(first: float, second: float) -> bool:
        return math.isclose(first, second, rel_tol=0.0, abs_tol=1.0e-9)

    original_z = normalized_z["original_endpoint_z_m"]
    runtime_z = normalized_z["runtime_goal_z_m"]
    if not all(
        same(value, original_z)
        for value in (
            original_carla_values["z"],
            original_ros_values["z"],
        )
    ):
        raise MatrixError("goal endpoint original Z provenance is inconsistent")
    if not all(
        same(value, runtime_z)
        for value in (
            normalized_z["last_road_waypoint_z_m"],
            normalized_z["serialized_terminal_z_m"],
            goal_carla_values["z"],
            goal_ros_values["z"],
            terminal_values["z"],
        )
    ):
        raise MatrixError(
            "normalized goal CARLA/ROS and serialized terminal Z differ"
        )
    if not same(normalized_z["applied_offset_m"], runtime_z - original_z):
        raise MatrixError("goal terminal-Z normalization offset is inconsistent")
    for field in carla_fields - {"z"}:
        if not same(original_carla_values[field], goal_carla_values[field]):
            raise MatrixError(
                "runtime goal CARLA pose changed outside terminal-Z normalization"
            )
    for field in ros_fields - {"z"}:
        if not same(original_ros_values[field], goal_ros_values[field]):
            raise MatrixError(
                "runtime goal ROS pose changed outside terminal-Z normalization"
            )
    if (
        not same(goal_carla_values["x"], goal_ros_values["x"])
        or not same(goal_carla_values["y"], -goal_ros_values["y"])
        or not same(
            _normalize_angle(
                goal_ros_values["yaw"]
                + math.radians(goal_carla_values["yaw"])
            ),
            0.0,
        )
        or not same(terminal_values["x"], goal_ros_values["x"])
        or not same(terminal_values["y"], goal_ros_values["y"])
        or not same(
            _normalize_angle(
                terminal_values["yaw"] - goal_ros_values["yaw"]
            ),
            0.0,
        )
    ):
        raise MatrixError(
            "normalized goal CARLA/ROS pose does not bind the route terminal"
        )
    return dict(provenance)


def _validate_route_payload(
    payload: Mapping[str, Any], expected_town: str, scenario: str
) -> dict[str, Any]:
    if payload.get("town") != expected_town or payload.get("scenario") != scenario:
        raise MatrixError(
            f"route identity mismatch: town={payload.get('town')!r} "
            f"scenario={payload.get('scenario')!r}"
        )
    points = payload.get("route")
    if not isinstance(points, list) or len(points) < 2:
        raise MatrixError("route must contain at least two serialized points")
    counts: Counter[str] = Counter()
    commands: Counter[int] = Counter()
    previous_distance = -math.inf
    for index, point in enumerate(points):
        if not isinstance(point, dict):
            raise MatrixError(f"route point {index} is not an object")
        option = point.get("road_option")
        command = point.get("vad_command")
        if option not in VAD_COMMANDS or command != VAD_COMMANDS[option]:
            raise MatrixError(
                f"route point {index} has invalid road-option/VAD-command pair: "
                f"{option!r}/{command!r}"
            )
        distance = point.get("distance_m")
        if not isinstance(distance, (int, float)) or not math.isfinite(distance):
            raise MatrixError(f"route point {index} has invalid distance")
        if float(distance) < previous_distance:
            raise MatrixError("route distance_m is not monotonic")
        previous_distance = float(distance)
        counts[option] += 1
        commands[int(command)] += 1
    declared_counts = payload.get("option_counts")
    if declared_counts != dict(counts):
        raise MatrixError(
            f"route option_counts mismatch: declared={declared_counts!r} "
            f"actual={dict(counts)!r}"
        )
    route_length = payload.get("route_length_m")
    if (
        not isinstance(route_length, (int, float))
        or not math.isfinite(route_length)
        or not math.isclose(float(route_length), previous_distance, abs_tol=1e-6)
    ):
        raise MatrixError("route_length_m does not match its serialized points")
    if scenario == "straight":
        if counts["STRAIGHT"] < 1:
            raise MatrixError("straight trial has no STRAIGHT route command")
        forbidden = {"LEFT", "RIGHT", "CHANGELANELEFT", "CHANGELANERIGHT"}
        if forbidden.intersection(counts):
            raise MatrixError("straight trial contains a turn/lane-change command")
    elif scenario in ("left", "right"):
        expected = scenario.upper()
        opposite = "RIGHT" if expected == "LEFT" else "LEFT"
        if counts[expected] < 1 or counts[opposite] > 0:
            raise MatrixError(
                f"{scenario} turn trial does not contain an unambiguous {expected} command"
            )
        if counts["CHANGELANELEFT"] > 0 or counts["CHANGELANERIGHT"] > 0:
            raise MatrixError(
                f"{scenario} turn trial contains lane-change commands; the matrix "
                "requires an isolated turn maneuver"
            )
    else:
        raise MatrixError(f"unsupported matrix route scenario: {scenario}")
    endpoint_source = payload.get("endpoint_source", "spawn_points")
    endpoint_identity: dict[str, Any]
    if endpoint_source == "spawn_points":
        start_index = payload.get("start_spawn_index")
        goal_index = payload.get("goal_spawn_index")
        if any(
            isinstance(value, bool) or not isinstance(value, int) or value < 0
            for value in (start_index, goal_index)
        ):
            raise MatrixError("spawn-point route endpoint indices are invalid")
        endpoint_identity = {
            "endpoint_source": "spawn_points",
            "start_spawn_index": start_index,
            "goal_spawn_index": goal_index,
        }
    elif endpoint_source == "generated_waypoints":
        start_index = payload.get("start_endpoint_index")
        goal_index = payload.get("goal_endpoint_index")
        spacing = payload.get("endpoint_waypoint_spacing_m")
        if any(
            isinstance(value, bool) or not isinstance(value, int) or value < 0
            for value in (start_index, goal_index)
        ):
            raise MatrixError("generated-waypoint route endpoint indices are invalid")
        spacing_value = _finite_number(
            spacing, "generated-waypoint endpoint spacing"
        )
        if spacing_value <= 0.0:
            raise MatrixError("generated-waypoint endpoint spacing must be positive")
        spawn_height = payload.get("spawn_height_contract")
        if (
            not isinstance(spawn_height, dict)
            or spawn_height.get("offset_owner")
            != "autoware_carla_interface_bridge"
            or not math.isclose(
                _finite_number(
                    spawn_height.get("bridge_z_offset_m"),
                    "generated-waypoint bridge Z offset",
                ),
                2.0,
                rel_tol=0.0,
                abs_tol=1.0e-9,
            )
            or not math.isclose(
                _finite_number(
                    spawn_height.get("catalog_z_offset_m"),
                    "generated-waypoint catalog Z offset",
                ),
                0.0,
                rel_tol=0.0,
                abs_tol=1.0e-9,
            )
        ):
            raise MatrixError(
                "generated-waypoint route lacks the bridge-owned spawn-height contract"
            )
        endpoint_identity = {
            "endpoint_source": "generated_waypoints",
            "endpoint_waypoint_spacing_m": spacing_value,
            "start_endpoint_index": start_index,
            "goal_endpoint_index": goal_index,
            "spawn_height_contract": dict(spawn_height),
        }
    else:
        raise MatrixError(f"unsupported route endpoint source: {endpoint_source!r}")
    goal_endpoint_provenance = _validate_goal_endpoint_provenance(
        payload,
        endpoint_source,
        int(goal_index),
        points[-1],
    )
    return {
        "town": expected_town,
        "scenario": scenario,
        "route_length_m": float(route_length),
        **endpoint_identity,
        "goal_endpoint_provenance": goal_endpoint_provenance,
        "option_counts": dict(counts),
        "vad_command_counts": {str(key): value for key, value in sorted(commands.items())},
        "point_count": len(points),
    }


def _plan_route_generation_contracts(
    plan: Mapping[str, Any],
) -> dict[str, dict[str, Any]]:
    selector = plan.get("runtime_profile_selector", "recommended")
    if selector not in RUNTIME_PROFILE_SELECTORS:
        raise MatrixError("matrix plan has an unknown runtime-profile selector")
    expected = _route_generation_contracts(
        plan.get("route_contract"), plan.get("runtime_profile"), str(selector)
    )
    if plan.get("route_generation_contracts") != expected:
        raise MatrixError(
            "matrix plan route-generation contracts differ from the selected profile"
        )
    return expected


def _plan_map_route_generation_contracts(
    plan: Mapping[str, Any], entry: Mapping[str, Any]
) -> dict[str, dict[str, Any]]:
    _plan_route_generation_contracts(plan)
    selector = str(plan.get("runtime_profile_selector", "recommended"))
    map_id = entry.get("map_id")
    if not isinstance(map_id, str) or not MAP_ID_RE.fullmatch(map_id):
        raise MatrixError("matrix plan map has an invalid route-generation map id")
    expected = _resolved_map_route_generation_contracts(
        plan.get("route_contract"),
        plan.get("runtime_profile"),
        selector,
        map_id,
    )
    if entry.get("route_generation_contracts") != expected:
        raise MatrixError(
            f"matrix plan {map_id} route-generation contracts differ from the "
            "map/profile override"
        )
    return expected


def _plan_route_selection_overrides(
    plan: Mapping[str, Any],
) -> dict[str, dict[str, dict[str, Any]]]:
    selector = plan.get("runtime_profile_selector", "recommended")
    if selector not in RUNTIME_PROFILE_SELECTORS:
        raise MatrixError("matrix plan has an unknown runtime-profile selector")
    profile = plan.get("runtime_profile")
    if not isinstance(profile, Mapping):
        raise MatrixError("matrix plan runtime profile is invalid")
    expected = _runtime_route_selection_overrides(profile, str(selector))
    if (
        plan.get("route_selection_policy") != ROUTE_SELECTION_POLICY
        or plan.get("route_selection_overrides") != expected
        or plan.get("route_selection_overrides_sha256") != sha256_json(expected)
    ):
        raise MatrixError(
            "matrix plan route-selection overrides differ from the selected profile"
        )
    return expected


def _plan_turn_route_selection_policy(
    plan: Mapping[str, Any],
) -> dict[str, Any] | None:
    selector = plan.get("runtime_profile_selector", "recommended")
    if selector not in RUNTIME_PROFILE_SELECTORS:
        raise MatrixError("matrix plan has an unknown runtime-profile selector")
    profile = plan.get("runtime_profile")
    if not isinstance(profile, Mapping):
        raise MatrixError("matrix plan runtime profile is invalid")
    expected = _runtime_turn_route_selection_policy(profile, str(selector))
    expected_sha256 = sha256_json(expected) if expected is not None else None
    if (
        plan.get("turn_route_selection_policy") != expected
        or plan.get("turn_route_selection_policy_sha256") != expected_sha256
    ):
        raise MatrixError(
            "matrix plan turn-route selection policy differs from the "
            "selected profile"
        )
    return expected


def _plan_map_route_selection_overrides(
    plan: Mapping[str, Any], entry: Mapping[str, Any]
) -> dict[str, dict[str, Any]]:
    overrides = _plan_route_selection_overrides(plan)
    map_id = entry.get("map_id")
    if not isinstance(map_id, str) or not MAP_ID_RE.fullmatch(map_id):
        raise MatrixError("matrix plan map has an invalid route-selection map id")
    expected = {
        trial_id: dict(override)
        for trial_id, override in overrides.get(map_id, {}).items()
    }
    if (
        entry.get("route_selection_overrides") != expected
        or entry.get("route_selection_overrides_sha256") != sha256_json(expected)
    ):
        raise MatrixError(
            f"matrix plan {map_id} route-selection overrides differ from the "
            "map/profile contract"
        )
    return expected


def _select_exact_route_override(
    candidates: Sequence[Mapping[str, Any]],
    override: Mapping[str, Any],
    map_id: str,
    trial_id: str,
) -> Mapping[str, Any]:
    matches = []
    available = []
    for candidate in candidates:
        scenario = candidate.get("scenario")
        seed = candidate.get("seed")
        pair_index = candidate.get("pair_index")
        available.append(
            {
                "scenario": scenario,
                "seed": seed,
                "pair_index": pair_index,
                "route_id": candidate.get("id"),
            }
        )
        if (
            scenario == override.get("scenario")
            and not isinstance(seed, bool)
            and isinstance(seed, int)
            and seed == override.get("seed")
            and not isinstance(pair_index, bool)
            and isinstance(pair_index, int)
            and pair_index == override.get("pair_index")
        ):
            matches.append(candidate)
    if len(matches) != 1:
        expected = {
            field: override[field]
            for field in ("scenario", "seed", "pair_index")
        }
        raise MatrixError(
            f"{map_id} {trial_id} exact route-selection override matched "
            f"{len(matches)} candidates; expected={expected}, available={available}"
        )
    return matches[0]


def _turn_candidate_identity(
    candidate: Mapping[str, Any], label: str
) -> dict[str, Any]:
    scenario = candidate.get("scenario")
    seed = candidate.get("seed")
    pair_index = candidate.get("pair_index")
    route_id = candidate.get("id")
    if scenario not in ("left", "right"):
        raise MatrixError(f"{label} has an invalid turn scenario")
    for field, value in (("seed", seed), ("pair_index", pair_index)):
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise MatrixError(f"{label} has an invalid {field}")
    if not isinstance(route_id, str) or not route_id:
        raise MatrixError(f"{label} has an invalid route id")
    return {
        "scenario": scenario,
        "seed": seed,
        "pair_index": pair_index,
        "route_id": route_id,
    }


def _speed_30kph_turn_ranking_vector(
    candidate: Mapping[str, Any],
    physical_turn: Mapping[str, Any],
    speed_contract: Mapping[str, Any],
    physical_preflight_source: str,
) -> dict[str, Any]:
    """Build the declared, outcome-independent turn ranking vector."""
    identity = _turn_candidate_identity(candidate, "turn ranking candidate")
    if physical_turn.get("status") != "PASS":
        raise MatrixError("turn ranking requires an exact physical-turn PASS")
    block = physical_turn.get("selected_block")
    if not isinstance(block, Mapping):
        raise MatrixError("turn ranking physical preflight has no selected block")
    parameters = speed_contract.get("route_manager_parameters")
    if not isinstance(parameters, Mapping):
        raise MatrixError("turn ranking speed contract has no route-manager parameters")
    target_speed = _finite_number(
        speed_contract.get("target_speed_mps"), "turn ranking target speed"
    )
    maximum_lateral_acceleration = _finite_number(
        parameters.get("maximum_lateral_acceleration_mps2"),
        "turn ranking maximum lateral acceleration",
    )
    comfortable_deceleration = _finite_number(
        parameters.get("comfortable_deceleration_mps2"),
        "turn ranking comfortable deceleration",
    )
    curvature_lookahead = _finite_number(
        parameters.get("route_curvature_lookahead_m"),
        "turn ranking route-curvature lookahead",
    )
    if min(
        target_speed,
        maximum_lateral_acceleration,
        comfortable_deceleration,
        curvature_lookahead,
    ) <= 0.0:
        raise MatrixError("turn ranking speed/deceleration parameters must be positive")

    p95_curvature = _finite_number(
        block.get("p95_absolute_curvature_per_m"),
        "turn ranking p95 curvature",
    )
    command_lead = _finite_number(
        block.get("command_lead_distance_m"),
        "turn ranking command lead",
    )
    command_tail = _finite_number(
        block.get("command_tail_distance_m"),
        "turn ranking command tail",
    )
    core_arc = _finite_number(
        block.get("command_arc_length_m"), "turn ranking core arc"
    )
    route_lead_field = (
        "route_lead_distance_m"
        if "route_lead_distance_m" in block
        else "block_start_distance_m"
    )
    route_lead = _finite_number(
        block.get(route_lead_field), "turn ranking route lead"
    )
    if min(p95_curvature, command_lead, command_tail, route_lead) < 0.0:
        raise MatrixError("turn ranking geometry values must be non-negative")
    if core_arc <= 0.0:
        raise MatrixError("turn ranking core arc must be positive")

    epsilon = float(SPEED_30KPH_TURN_ROUTE_SELECTION_POLICY[
        "curvature_epsilon_per_m"
    ])
    curvature_limited_speed = math.sqrt(
        maximum_lateral_acceleration / max(p95_curvature, epsilon)
    )
    braking_curve_speed = min(target_speed, curvature_limited_speed)
    required_braking_distance = max(
        0.0,
        (target_speed**2 - braking_curve_speed**2)
        / (2.0 * comfortable_deceleration),
    )
    available_braking_preview = min(route_lead, curvature_lookahead)
    braking_preview_reserve = (
        available_braking_preview - required_braking_distance
    )
    maximum_command_skew = max(command_lead, command_tail)
    physical_limits = physical_turn.get("limits")
    maximum_p95_curvature = (
        _finite_number(
            physical_limits.get("maximum_p95_abs_curvature_per_m"),
            "turn ranking maximum p95 curvature",
        )
        if isinstance(physical_limits, Mapping)
        else None
    )
    curvature_reserve = (
        maximum_p95_curvature - p95_curvature
        if maximum_p95_curvature is not None
        else None
    )
    scenario_rank = ("left", "right").index(str(identity["scenario"]))
    sort_key: list[Any] = [
        -curvature_limited_speed,
        -braking_preview_reserve,
        maximum_command_skew,
        -core_arc,
        scenario_rank,
        identity["seed"],
        identity["pair_index"],
        identity["route_id"],
    ]
    sort_components = [
        {
            "field": "curvature_limited_speed_mps",
            "direction": "descending",
            "value": curvature_limited_speed,
            "sort_value": -curvature_limited_speed,
            "definition": "sqrt(maximum_lateral_acceleration_mps2 / max(p95_abs_curvature_per_m, curvature_epsilon_per_m))",
        },
        {
            "field": "braking_preview_reserve_m",
            "direction": "descending",
            "value": braking_preview_reserve,
            "sort_value": -braking_preview_reserve,
            "definition": "min(route_lead_m, route_curvature_lookahead_m) - max(0, (target_speed_mps^2 - min(target_speed_mps, curvature_limited_speed_mps)^2) / (2 * comfortable_deceleration_mps2))",
        },
        {
            "field": "maximum_command_alignment_skew_m",
            "direction": "ascending",
            "value": maximum_command_skew,
            "sort_value": maximum_command_skew,
            "definition": "max(command_lead_distance_m, command_tail_distance_m)",
        },
        {
            "field": "turn_core_arc_length_m",
            "direction": "descending",
            "value": core_arc,
            "sort_value": -core_arc,
            "definition": "selected physical-turn command_arc_length_m",
        },
        {
            "field": "scenario",
            "direction": "ascending",
            "value": identity["scenario"],
            "sort_value": scenario_rank,
            "definition": "fixed left-before-right scenario order",
        },
        {
            "field": "seed",
            "direction": "ascending",
            "value": identity["seed"],
            "sort_value": identity["seed"],
            "definition": "catalog seed",
        },
        {
            "field": "pair_index",
            "direction": "ascending",
            "value": identity["pair_index"],
            "sort_value": identity["pair_index"],
            "definition": "catalog pair index",
        },
        {
            "field": "route_id",
            "direction": "ascending",
            "value": identity["route_id"],
            "sort_value": identity["route_id"],
            "definition": "catalog route id",
        },
    ]
    return {
        "schema_version": 1,
        "policy_id": SPEED_30KPH_TURN_ROUTE_SELECTION_METHOD,
        "physical_turn_preflight_status": "PASS",
        "physical_turn_preflight_source": physical_preflight_source,
        "stable_identity": identity,
        "parameter_values": {
            "target_speed_mps": target_speed,
            "maximum_lateral_acceleration_mps2": maximum_lateral_acceleration,
            "comfortable_deceleration_mps2": comfortable_deceleration,
            "route_curvature_lookahead_m": curvature_lookahead,
            "curvature_epsilon_per_m": epsilon,
        },
        "raw_values": {
            "p95_absolute_curvature_per_m": p95_curvature,
            "maximum_p95_absolute_curvature_per_m": maximum_p95_curvature,
            "p95_curvature_reserve_per_m": curvature_reserve,
            "curvature_limited_speed_mps": curvature_limited_speed,
            "braking_curve_speed_mps": braking_curve_speed,
            "route_lead_field": route_lead_field,
            "route_lead_m": route_lead,
            "available_braking_preview_m": available_braking_preview,
            "required_braking_distance_m": required_braking_distance,
            "braking_preview_reserve_m": braking_preview_reserve,
            "command_lead_distance_m": command_lead,
            "command_tail_distance_m": command_tail,
            "maximum_command_alignment_skew_m": maximum_command_skew,
            "turn_core_arc_length_m": core_arc,
        },
        "sort_components": sort_components,
        "sort_key": sort_key,
    }


def _rank_speed_30kph_turn_candidates(
    candidates: Sequence[Mapping[str, Any]],
    catalog_path: Path,
    speed_contract: Mapping[str, Any],
    policy: Mapping[str, Any],
    *,
    custom_map: bool,
) -> tuple[
    Mapping[str, Any],
    dict[str, Any],
    dict[str, Any],
    dict[str, Any] | None,
]:
    """Filter exact physical PASS routes, then rank every remaining candidate."""
    if dict(policy) != SPEED_30KPH_TURN_ROUTE_SELECTION_POLICY:
        raise MatrixError("30 kph turn ranking policy changed")
    required_turn_geometry = speed_contract["trials"]["turn"][
        "physical_geometry"
    ]
    ranked: list[
        tuple[
            tuple[Any, ...],
            Mapping[str, Any],
            dict[str, Any],
            dict[str, Any] | None,
            dict[str, Any],
        ]
    ] = []
    candidate_records: list[dict[str, Any]] = []
    identities: set[tuple[Any, ...]] = set()
    for candidate in candidates:
        identity = _turn_candidate_identity(candidate, "catalog turn candidate")
        identity_key = (
            identity["scenario"],
            identity["seed"],
            identity["pair_index"],
            identity["route_id"],
        )
        if identity_key in identities:
            raise MatrixError(
                f"duplicate turn candidate identity in catalog: {identity}"
            )
        identities.add(identity_key)
        candidate_value = candidate.get("path")
        if not isinstance(candidate_value, str) or not candidate_value:
            raise MatrixError("catalog turn route candidate has no path")
        candidate_path = _safe_inside(
            catalog_path.parent,
            catalog_path.parent / candidate_value,
            "route",
        )
        if not candidate_path.is_file():
            raise MatrixError(f"catalog route is missing: {candidate_path}")
        route_sha256 = sha256_file(candidate_path)
        if route_sha256 != candidate.get("sha256"):
            raise MatrixError(f"catalog route SHA256 mismatch: {candidate_path}")
        candidate_payload = read_object(candidate_path, "turn route candidate")
        custom_turn_geometry = None
        if custom_map:
            try:
                custom_turn_geometry = _custom_route_turn_geometry(
                    candidate_payload, str(identity["scenario"])
                )
            except MatrixError as error:
                raise MatrixError(
                    "saved custom-map turn preflight cannot be reproduced: "
                    f"{error}"
                ) from error
            if (
                candidate.get("turn_geometry_preflight")
                != custom_turn_geometry
                or candidate_payload.get("turn_geometry_preflight")
                != custom_turn_geometry
            ):
                raise MatrixError(
                    "custom-map turn-geometry provenance does not match fresh "
                    "analysis"
                )
        try:
            common_turn_geometry = _speed_30kph_turn_geometry(
                candidate_payload, required_turn_geometry
            )
        except PhysicalTurnCandidateRejection as error:
            candidate_records.append(
                {
                    "identity": identity,
                    "route_sha256": route_sha256,
                    "physical_turn_status": "REJECTED",
                    "physical_turn_preflight_source": (
                        "exact_serialized_common_physical_turn"
                    ),
                    "custom_turn_geometry_preflight_sha256": (
                        sha256_json(custom_turn_geometry)
                        if custom_turn_geometry is not None
                        else None
                    ),
                    "rejection_reason": str(error),
                    "rejection_evidence": error.evidence,
                    "ranking_vector": None,
                }
            )
            continue
        if not custom_map and (
            candidate.get("physical_turn_preflight") != common_turn_geometry
            or candidate_payload.get("physical_turn_preflight")
            != common_turn_geometry
        ):
            raise MatrixError(
                "packaged turn route exact physical-geometry provenance does "
                "not match fresh analysis"
            )
        vector = _speed_30kph_turn_ranking_vector(
            candidate,
            common_turn_geometry,
            speed_contract,
            "exact_serialized_common_physical_turn",
        )
        candidate_records.append(
            {
                "identity": identity,
                "route_sha256": route_sha256,
                "physical_turn_status": "PASS",
                "physical_turn_preflight_source": (
                    "exact_serialized_common_physical_turn"
                ),
                "physical_turn_preflight_sha256": sha256_json(
                    common_turn_geometry
                ),
                "custom_turn_geometry_preflight_sha256": (
                    sha256_json(custom_turn_geometry)
                    if custom_turn_geometry is not None
                    else None
                ),
                "rejection_reason": None,
                "rejection_evidence": None,
                "ranking_vector": vector,
            }
        )
        ranked.append(
            (
                tuple(vector["sort_key"]),
                candidate,
                common_turn_geometry,
                custom_turn_geometry,
                vector,
            )
        )
    candidate_records.sort(
        key=lambda record: (
            record["identity"]["scenario"],
            record["identity"]["seed"],
            record["identity"]["pair_index"],
            record["identity"]["route_id"],
        )
    )
    candidate_set_payload = {
        "schema_version": 1,
        "policy": dict(policy),
        "candidates": candidate_records,
    }
    if not ranked:
        rejected = [
            {
                "identity": record["identity"],
                "reason": record["rejection_reason"],
                "evidence": record.get("rejection_evidence"),
            }
            for record in candidate_records
        ]
        raise MatrixError(
            "30 kph exact physical-turn preflight failed for every catalog "
            f"candidate: {rejected}"
        )
    ranked.sort(key=lambda value: value[0])
    (
        _,
        selected,
        selected_geometry,
        selected_custom_geometry,
        selected_vector,
    ) = ranked[0]
    rejected_candidates = [
        {
            "identity": record["identity"],
            "reason": record["rejection_reason"],
            "evidence": record.get("rejection_evidence"),
        }
        for record in candidate_records
        if record["physical_turn_status"] == "REJECTED"
    ]
    audit = {
        "schema_version": 1,
        "policy": dict(policy),
        "policy_sha256": sha256_json(policy),
        "candidate_count": len(candidate_records),
        "physical_pass_candidate_count": len(ranked),
        "physical_rejected_candidate_count": len(rejected_candidates),
        "candidate_set_sha256": sha256_json(candidate_set_payload),
        "candidate_set_ordering": (
            "scenario_seed_pair_index_route_id_ascending_before_sha256"
        ),
        "selected_identity": _turn_candidate_identity(
            selected, "selected turn candidate"
        ),
        "selected_ranking_vector": selected_vector,
        "selected_ranking_vector_sha256": sha256_json(selected_vector),
        "rejected_candidates": rejected_candidates,
        "execution_result_fields_used": False,
        "map_preflight_fallback_allowed": False,
        "candidate_pair_reindexing_allowed": False,
        "candidate_generation_quota_modified": False,
    }
    return selected, audit, selected_geometry, selected_custom_geometry


def _validate_catalog_generation(
    catalog: Mapping[str, Any],
    expected: Mapping[str, Any],
    trial_id: str,
) -> None:
    generation = catalog.get("generation")
    if not isinstance(generation, dict):
        raise MatrixError(f"{trial_id} route catalog has no generation provenance")
    if generation.get("weather") != expected["weather"]:
        raise MatrixError(f"{trial_id} route catalog weather contract mismatch")
    if generation.get("seeds") != expected["seeds"]:
        raise MatrixError(f"{trial_id} route catalog seed contract mismatch")
    for field, catalog_field in (
        ("pairs_per_seed", "pairs_per_seed"),
        ("maximum_traces_per_scenario", "max_traces_per_scenario"),
    ):
        actual = generation.get(catalog_field)
        if (
            isinstance(actual, bool)
            or not isinstance(actual, int)
            or actual != expected[field]
        ):
            raise MatrixError(
                f"{trial_id} route catalog {field} contract mismatch"
            )
    for field in (
        *ROUTE_GENERATION_DISTANCE_FIELDS,
        "sampling_resolution_m",
        "maximum_endpoint_offset_m",
    ):
        actual = _finite_number(
            generation.get(field), f"{trial_id} route catalog generation.{field}"
        )
        if not math.isclose(
            actual, float(expected[field]), rel_tol=0.0, abs_tol=1.0e-9
        ):
            raise MatrixError(
                f"{trial_id} route catalog {field} contract mismatch: "
                f"expected={expected[field]!r} actual={actual!r}"
            )
    expected_capacity = expected.get(
        "straight_capacity_contract", {"enabled": False}
    )
    if generation.get("straight_capacity_contract") != expected_capacity:
        raise MatrixError(
            f"{trial_id} route catalog straight-capacity contract mismatch"
        )


def _validate_speed_catalog_endpoint_policy(
    catalog: Mapping[str, Any],
    trial_id: str,
    expected_generation: Mapping[str, Any],
    *,
    custom_map: bool,
) -> None:
    generation = catalog.get("generation")
    if not isinstance(generation, dict):
        raise MatrixError(f"{trial_id} speed catalog lacks generation provenance")
    expected_scenarios = ["straight"] if trial_id == "straight" else ["left", "right"]
    if generation.get("scenarios") != expected_scenarios:
        raise MatrixError(
            f"{trial_id} speed catalog scenario filter is not pinned: "
            f"{generation.get('scenarios')!r}"
        )
    use_generated_waypoints = trial_id == "straight" and not custom_map
    expected_source = "generated_waypoints" if use_generated_waypoints else "spawn_points"
    if generation.get("endpoint_source") != expected_source:
        raise MatrixError(
            f"{trial_id} speed catalog endpoint source must be {expected_source!r}"
        )
    use_physical_turn = trial_id == "turn" and not custom_map
    if use_physical_turn:
        expected_physical_turn = {
            "enabled": True,
            "profile_id": "speed_30kph",
            "applicability": "packaged_town_only",
            "measurement_source": "exact_serialized_3d_route_with_terminal_goal",
            "admission_policy": "reject_before_accepted_pair_quota",
            "limits": dict(SPEED_30KPH_TURN_GEOMETRY_CONTRACT),
            "provenance": dict(SPEED_30KPH_TURN_CONTRACT_PROVENANCE),
        }
        if generation.get("physical_turn_contract") != expected_physical_turn:
            raise MatrixError(
                "turn speed catalog physical-geometry generation contract "
                "is not pinned"
            )
    elif generation.get("physical_turn_contract") is not None:
        raise MatrixError(
            f"{trial_id} speed catalog must not enable packaged physical-turn gate"
        )
    spacing = generation.get("endpoint_waypoint_spacing_m")
    expected_junction_policy = expected_generation.get(
        "endpoint_junction_policy", "include"
    )
    expected_enumeration_policy = expected_generation.get(
        "candidate_enumeration_policy", "all_pairs"
    )
    if (
        generation.get("endpoint_junction_policy") != expected_junction_policy
        or generation.get("candidate_enumeration_policy")
        != expected_enumeration_policy
    ):
        raise MatrixError(
            f"{trial_id} speed catalog candidate enumeration policy is not pinned"
        )
    if use_generated_waypoints:
        expected_physical_straight = {
            "enabled": True,
            "profile_id": "speed_30kph",
            "measurement_source": "exact_serialized_route_with_terminal_goal",
            "admission_policy": "reject_before_accepted_pair_quota",
            "limits": dict(SPEED_30KPH_STRAIGHT_GEOMETRY_CONTRACT),
        }
        if generation.get("physical_straight_contract") != (
            expected_physical_straight
        ):
            raise MatrixError(
                "straight speed catalog physical-geometry generation contract "
                "is not pinned"
            )
        spacing_value = _finite_number(
            spacing, f"{trial_id} speed catalog endpoint waypoint spacing"
        )
        expected_spacing = float(
            expected_generation.get(
                "endpoint_waypoint_spacing_m",
                SPEED_30KPH_PACKAGED_STRAIGHT_ENDPOINT_SPACING_M,
            )
        )
        if not math.isclose(
            spacing_value,
            expected_spacing,
            rel_tol=0.0,
            abs_tol=1.0e-9,
        ):
            raise MatrixError(
                f"{trial_id} speed catalog endpoint waypoint spacing is not pinned"
            )
        spawn_height = generation.get("spawn_height_contract")
        if (
            not isinstance(spawn_height, dict)
            or spawn_height.get("offset_owner")
            != "autoware_carla_interface_bridge"
            or not math.isclose(
                _finite_number(
                    spawn_height.get("bridge_z_offset_m"),
                    f"{trial_id} speed catalog bridge Z offset",
                ),
                2.0,
                rel_tol=0.0,
                abs_tol=1.0e-9,
            )
            or not math.isclose(
                _finite_number(
                    spawn_height.get("catalog_z_offset_m"),
                    f"{trial_id} speed catalog catalog Z offset",
                ),
                0.0,
                rel_tol=0.0,
                abs_tol=1.0e-9,
            )
        ):
            raise MatrixError(
                f"{trial_id} speed catalog spawn-height contract is not pinned"
            )
        endpoint_count = generation.get("endpoint_count")
        endpoint_api_count = generation.get("endpoint_api_count")
        eligible_api_count = generation.get("endpoint_eligible_api_count")
        junction_count = generation.get("endpoint_junction_waypoint_count")
        junction_excluded = generation.get("endpoint_junction_excluded_count")
        duplicate_count = generation.get("endpoint_duplicate_transform_count")
        if (
            isinstance(endpoint_count, bool)
            or not isinstance(endpoint_count, int)
            or endpoint_count < 2
            or isinstance(endpoint_api_count, bool)
            or not isinstance(endpoint_api_count, int)
            or endpoint_api_count < endpoint_count
            or isinstance(eligible_api_count, bool)
            or not isinstance(eligible_api_count, int)
            or eligible_api_count < endpoint_count
            or isinstance(junction_count, bool)
            or not isinstance(junction_count, int)
            or junction_count < 0
            or isinstance(junction_excluded, bool)
            or not isinstance(junction_excluded, int)
            or junction_excluded < 0
            or junction_excluded > junction_count
            or eligible_api_count != endpoint_api_count - junction_excluded
            or isinstance(duplicate_count, bool)
            or not isinstance(duplicate_count, int)
            or duplicate_count != eligible_api_count - endpoint_count
            or (
                expected_junction_policy == "exclude"
                and junction_excluded != junction_count
            )
            or (
                expected_junction_policy == "include"
                and junction_excluded != 0
            )
            or generation.get("endpoint_deduplication")
            != "exact_full_transform_keep_first_api_occurrence"
            or generation.get("endpoint_ordering")
            != "deduplicated_lexicographic_transform_x_y_z_roll_pitch_yaw"
        ):
            raise MatrixError(
                f"{trial_id} speed catalog waypoint endpoint provenance is invalid"
            )
        if expected_enumeration_policy == "directed_topology_straight_v1":
            scenario_results = catalog.get("scenario_results")
            result = next(
                (
                    item
                    for item in scenario_results
                    if isinstance(item, dict) and item.get("scenario") == "straight"
                ),
                None,
            ) if isinstance(scenario_results, list) else None
            reports = (
                result.get("candidate_enumeration")
                if isinstance(result, dict)
                else None
            )
            expected_seeds = expected_generation.get("seeds")
            expected_route_count = (
                int(expected_generation["pairs_per_seed"]) * len(expected_seeds)
                if isinstance(expected_seeds, list)
                else -1
            )
            result_traces = result.get("traces") if isinstance(result, dict) else None
            trace_coverage = (
                result.get("trace_coverage") if isinstance(result, dict) else None
            )
            if (
                not isinstance(reports, list)
                or not reports
                or not isinstance(expected_seeds, list)
                or [item.get("seed") for item in reports] != expected_seeds
                or not isinstance(result, dict)
                or result.get("status") != "READY"
                or result.get("route_count") != expected_route_count
                or isinstance(result_traces, bool)
                or not isinstance(result_traces, int)
                or result_traces < expected_route_count
                or result_traces
                > int(expected_generation["maximum_traces_per_scenario"])
                or not isinstance(trace_coverage, dict)
                or trace_coverage.get("accepted") != expected_route_count
                or trace_coverage.get("attempted") != result_traces
                or any(
                    not isinstance(item, dict)
                    or item.get("policy")
                    != "directed_topology_straight_v1"
                    or isinstance(
                        item.get("planar_chord_heading_candidate_count"), bool
                    )
                    or not isinstance(
                        item.get("planar_chord_heading_candidate_count"), int
                    )
                    or isinstance(
                        item.get("directed_reachable_candidate_count"), bool
                    )
                    or not isinstance(
                        item.get("directed_reachable_candidate_count"), int
                    )
                    or item["planar_chord_heading_candidate_count"]
                    < item["directed_reachable_candidate_count"]
                    or item["directed_reachable_candidate_count"]
                    < int(expected_generation["pairs_per_seed"])
                    or item.get("postfilter_authority")
                    != "exact serialized route distance and physical-straight analysis"
                    for item in reports
                )
            ):
                raise MatrixError(
                    "straight speed catalog directed-topology enumeration "
                    "provenance is invalid"
                )
    elif spacing is not None:
        raise MatrixError(
            f"{trial_id} spawn-point speed catalog must not set waypoint spacing"
        )


def _load_e2e_helper(filename: str, module_name: str) -> Any:
    path = Path(__file__).with_name(filename)
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise MatrixError(f"cannot load route/map preflight helper: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
    except Exception as error:
        sys.modules.pop(module_name, None)
        raise MatrixError(f"cannot load route/map preflight helper {path}: {error}") from error
    return module


def _verified_bundle_file_sha256(
    bundle: Mapping[str, Any], label: str
) -> dict[str, str]:
    expected = bundle.get("bundle_file_sha256")
    if not isinstance(expected, dict) or set(expected) != set(BUNDLE_FILE_NAMES):
        raise MatrixError(f"{label} has no complete admitted bundle-file hashes")
    bundle_path = Path(str(bundle.get("path", ""))).expanduser().resolve()
    actual: dict[str, str] = {}
    for file_id, filename in BUNDLE_FILE_NAMES.items():
        path = bundle_path / filename
        if not path.is_file():
            raise MatrixError(f"{label} bundle file is missing: {path}")
        actual[file_id] = sha256_file(path)
        if actual[file_id] != expected.get(file_id):
            raise MatrixError(
                f"{label} {filename} SHA256 differs from campaign admission"
            )
    if actual["metadata"] != bundle.get("metadata_sha256"):
        raise MatrixError(f"{label} map_bundle.json SHA256 is internally inconsistent")
    return actual


def _campaign_route_map_preflight(
    output_root: Path,
    plan: Mapping[str, Any],
    entry: Mapping[str, Any],
    selected: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    """Freshly validate selected speed-profile routes against Lanelet2 and PCD."""
    _verify_campaign_plan(
        plan, f"map {entry.get('map_id')} campaign route/map preflight"
    )
    if (
        len(selected) != len(TRIAL_IDS)
        or {str(item.get("trial_id")) for item in selected} != set(TRIAL_IDS)
    ):
        raise MatrixError(
            "campaign route/map preflight requires exactly straight and turn routes"
        )
    bundle = entry.get("full_map_bundle")
    if not isinstance(bundle, dict):
        raise MatrixError("campaign route/map preflight has no admitted map bundle")
    bundle_path = Path(str(bundle.get("path", ""))).expanduser().resolve()
    bundle_hashes_before = _verified_bundle_file_sha256(
        bundle, "pre-preflight"
    )
    route_paths = [Path(str(item["route_path"])).resolve() for item in selected]
    aligner = _load_e2e_helper(
        "align_carla_route_to_map.py", "matrix_align_carla_route_to_map"
    )
    validator = _load_e2e_helper(
        "validate_route_map.py", "matrix_validate_route_map"
    )
    preparer = _load_e2e_helper(
        "prepare_packaged_town_full_maps.py", "matrix_prepare_town_maps"
    )
    try:
        bundle_payload, bundle_transform = aligner.load_map_bundle(bundle_path)
    except Exception as error:
        raise MatrixError(
            f"cannot load admitted map-bundle alignment: {error}"
        ) from error
    expected_profile = str(bundle.get("profile", ""))
    if (
        not expected_profile
        or bundle_payload.get("profile") != expected_profile
    ):
        raise MatrixError(
            "admitted map-bundle profile differs from map_bundle.json"
        )
    bundle_transform_values = {
        field: _finite_number(
            getattr(bundle_transform, field),
            f"map_bundle.json carla_to_map_transform.{field}",
        )
        for field in ("x_m", "y_m", "z_m", "yaw_rad")
    }
    alignment_helper_path = Path(__file__).with_name(
        "align_carla_route_to_map.py"
    ).resolve()
    alignment_helper_sha256 = sha256_file(alignment_helper_path)
    alignment_root = (
        output_root
        / "maps"
        / str(entry["map_id"])
        / "campaign_route_map_preflight_routes"
    ).resolve()
    cases: list[dict[str, Any]] = []
    aligned_route_paths: list[Path] = []
    aligned_path_by_trial: dict[str, Path] = {}
    for item, route_path in zip(selected, route_paths):
        trial_id = str(item["trial_id"])
        case: dict[str, Any] = {
            "trial_id": trial_id,
            "scenario": item["catalog_scenario"],
            # Preserve the catalog/raw route as the downstream trial identity.
            "route_path": str(route_path),
            "route_sha256": item["route_sha256"],
            "route_length_m": item["analysis"]["route_length_m"],
            "status": "FAIL",
            "lanelet2": None,
            "lanelet2_error": None,
            "pointcloud_proximity": None,
        }
        if sha256_file(route_path) != item["route_sha256"]:
            case["route_alignment"] = {
                "status": "FAIL",
                "error": "raw catalog route SHA256 changed before alignment",
            }
            cases.append(case)
            continue
        aligned_output = alignment_root / f"{trial_id}.aligned.json"
        try:
            alignment = aligner.prepare_aligned_route(
                route_path,
                bundle_path,
                output_path=aligned_output,
            )
            aligned_path = Path(str(alignment["aligned_route"])).resolve()
            if (
                alignment.get("status") != "PASS"
                or alignment.get("already_aligned") is not False
                or Path(str(alignment.get("source_route", ""))).resolve()
                != route_path
                or alignment.get("source_route_sha256") != item["route_sha256"]
                or aligned_path != aligned_output
                or not aligned_path.is_file()
                or alignment.get("aligned_route_sha256")
                != sha256_file(aligned_path)
                or alignment.get("profile") != expected_profile
                or alignment.get("carla_to_map_transform")
                != bundle_transform_values
                or sha256_file(route_path) != item["route_sha256"]
            ):
                raise MatrixError(
                    "route alignment result does not bind raw route, aligned "
                    "route, and admitted map-bundle transform"
                )
            aligned_payload = read_object(
                aligned_path, f"{trial_id} aligned campaign route"
            )
            coordinate_alignment = aligned_payload.get("coordinate_alignment")
            if (
                not isinstance(coordinate_alignment, dict)
                or coordinate_alignment.get("source_frame") != "carla_map"
                or coordinate_alignment.get("target_frame") != "map"
                or Path(
                    str(coordinate_alignment.get("source_route", ""))
                ).resolve()
                != route_path
                or coordinate_alignment.get("source_route_sha256")
                != item["route_sha256"]
                or coordinate_alignment.get("map_bundle_profile")
                != expected_profile
                or coordinate_alignment.get("carla_to_map_transform")
                != bundle_transform_values
            ):
                raise MatrixError(
                    "aligned campaign route has invalid coordinate provenance"
                )
        except Exception as error:
            case["route_alignment"] = {
                "status": "FAIL",
                "error": str(error),
            }
            cases.append(case)
            continue
        case["route_alignment"] = {
            "status": "PASS",
            "method": "align_carla_route_to_map.prepare_aligned_route",
            "source_frame": "carla_map",
            "target_frame": "map",
            "source_route_path": str(route_path),
            "source_route_sha256": item["route_sha256"],
            "aligned_route_path": str(aligned_path),
            "aligned_route_sha256": sha256_file(aligned_path),
            "map_bundle_profile": expected_profile,
            "map_bundle_metadata_sha256": bundle["metadata_sha256"],
            "carla_to_map_transform": bundle_transform_values,
            "input_already_aligned": False,
        }
        cases.append(case)
        aligned_route_paths.append(aligned_path)
        aligned_path_by_trial[trial_id] = aligned_path

    pcd_error: str | None = None
    if aligned_route_paths:
        try:
            pcd_results = preparer.validate_route_pcd_proximity(
                aligned_route_paths, bundle_path
            )
        except Exception as error:
            pcd_error = str(error)
            pcd_results = {
                str(path): {"status": "FAIL", "error": pcd_error}
                for path in aligned_route_paths
            }
    else:
        pcd_results = {}

    for case in cases:
        alignment = case.get("route_alignment")
        if not isinstance(alignment, dict) or alignment.get("status") != "PASS":
            continue
        trial_id = str(case["trial_id"])
        aligned_path = aligned_path_by_trial[trial_id]
        lanelet_result: Mapping[str, Any] | None = None
        lanelet_error: str | None = None
        try:
            lanelet_result = validator.validate_route_map(
                aligned_path,
                bundle_path,
                tolerance_m=0.75,
                vertical_tolerance_m=5.0,
            )
        except Exception as error:
            lanelet_error = str(error)
        pcd_result = pcd_results.get(str(aligned_path))
        case_status = (
            "PASS"
            if isinstance(lanelet_result, dict)
            and lanelet_result.get("status") == "PASS"
            and isinstance(pcd_result, dict)
            and pcd_result.get("status") == "PASS"
            else "FAIL"
        )
        case.update(
            status=case_status,
            lanelet2=dict(lanelet_result) if lanelet_result else None,
            lanelet2_error=lanelet_error,
            pointcloud_proximity=pcd_result,
        )

    for case in cases:
        alignment = case.get("route_alignment")
        if not isinstance(alignment, dict) or alignment.get("status") != "PASS":
            continue
        raw_path = Path(str(case["route_path"])).resolve()
        aligned_path = Path(str(alignment["aligned_route_path"])).resolve()
        if (
            not raw_path.is_file()
            or sha256_file(raw_path) != case["route_sha256"]
            or not aligned_path.is_file()
            or sha256_file(aligned_path)
            != alignment["aligned_route_sha256"]
        ):
            alignment.update(
                status="FAIL",
                error="raw or aligned route changed during campaign preflight",
            )
            case["status"] = "FAIL"

    status = "PASS" if all(case["status"] == "PASS" for case in cases) else "FAIL"
    bundle_hashes_after = _verified_bundle_file_sha256(
        bundle, "post-preflight"
    )
    if bundle_hashes_after != bundle_hashes_before:
        raise MatrixError("map bundle changed during campaign route/map preflight")
    if sha256_file(alignment_helper_path) != alignment_helper_sha256:
        raise MatrixError("route alignment helper changed during campaign preflight")
    _verify_campaign_plan(
        plan, f"map {entry.get('map_id')} campaign preflight completion"
    )
    payload = {
        "schema_version": 1,
        "status": status,
        "scope": (
            "fresh campaign exact-route Lanelet2 + point-cloud structural "
            "preflight; not an Autoware VAD execution result"
        ),
        "generated_at": utc_now(),
        "matrix_id": plan["matrix_id"],
        "runtime_profile_selector": plan.get(
            "runtime_profile_selector", "recommended"
        ),
        "runtime_profile_id": plan["runtime_profile"]["id"],
        "map_id": entry["map_id"],
        "canonical_name": entry["canonical_name"],
        "map_bundle_path": str(bundle_path),
        "map_bundle_metadata_sha256": bundle["metadata_sha256"],
        "map_bundle_file_sha256": bundle_hashes_after,
        "admission_contract_sha256": plan["admission_contract_sha256"],
        "campaign_execution_contract_sha256": plan[
            "campaign_execution_contract_sha256"
        ],
        "route_alignment_contract": {
            "schema_version": 1,
            "method": "align_carla_route_to_map.prepare_aligned_route",
            "source_frame": "carla_map",
            "target_frame": "map",
            "raw_route_identity_preserved": True,
            "validator_input": "aligned_route",
            "map_bundle_profile": expected_profile,
            "carla_to_map_transform": bundle_transform_values,
            "helper_path": str(alignment_helper_path),
            "helper_sha256": alignment_helper_sha256,
        },
        "route_generation_contracts_sha256": sha256_json(
            _plan_map_route_generation_contracts(plan, entry)
        ),
        "pcd_validation_error": pcd_error,
        "cases": cases,
    }
    destination = output_root / "maps" / str(entry["map_id"]) / (
        "campaign_route_map_preflight.json"
    )
    atomic_json(destination, payload)
    artifact = {
        "path": str(destination),
        "sha256": sha256_file(destination),
        "status": status,
        "cases": cases,
    }
    if status != "PASS":
        failures = [
            f"{case['trial_id']}:"
            f"{case.get('route_alignment', {}).get('error') or case.get('lanelet2_error') or case.get('pointcloud_proximity')}"
            for case in cases
            if case["status"] != "PASS"
        ]
        raise MatrixError(
            "fresh campaign route/map preflight failed; artifact retained at "
            f"{destination}: {'; '.join(failures)}"
        )
    return artifact


def _validate_campaign_route_alignment(
    output_root: Path,
    artifact: Mapping[str, Any],
    bundle: Mapping[str, Any],
    route_entry: Mapping[str, Any],
    case: Mapping[str, Any],
) -> dict[str, Any]:
    """Recompute the raw -> map-frame route and verify its immutable chain."""
    contract = artifact.get("route_alignment_contract")
    alignment = case.get("route_alignment")
    if not isinstance(contract, dict) or not isinstance(alignment, dict):
        raise MatrixError("campaign route alignment provenance is missing")
    helper_path = Path(__file__).with_name(
        "align_carla_route_to_map.py"
    ).resolve()
    if (
        contract.get("schema_version") != 1
        or contract.get("method")
        != "align_carla_route_to_map.prepare_aligned_route"
        or contract.get("source_frame") != "carla_map"
        or contract.get("target_frame") != "map"
        or contract.get("raw_route_identity_preserved") is not True
        or contract.get("validator_input") != "aligned_route"
        or Path(str(contract.get("helper_path", ""))).resolve() != helper_path
        or contract.get("helper_sha256") != sha256_file(helper_path)
    ):
        raise MatrixError("campaign route alignment contract is invalid")

    bundle_path = Path(str(bundle.get("path", ""))).expanduser().resolve()
    aligner = _load_e2e_helper(
        "align_carla_route_to_map.py", "matrix_validate_carla_route_alignment"
    )
    try:
        bundle_payload, bundle_transform = aligner.load_map_bundle(bundle_path)
    except Exception as error:
        raise MatrixError(
            f"cannot reload admitted map-bundle alignment: {error}"
        ) from error
    expected_profile = str(bundle.get("profile", ""))
    expected_transform = {
        field: _finite_number(
            getattr(bundle_transform, field),
            f"map_bundle.json carla_to_map_transform.{field}",
        )
        for field in ("x_m", "y_m", "z_m", "yaw_rad")
    }
    if (
        not expected_profile
        or bundle_payload.get("profile") != expected_profile
        or contract.get("map_bundle_profile") != expected_profile
        or contract.get("carla_to_map_transform") != expected_transform
    ):
        raise MatrixError("campaign route alignment bundle transform is invalid")

    raw_route_path = Path(str(route_entry.get("route_path", ""))).resolve()
    raw_route_sha256 = str(route_entry.get("route_sha256", ""))
    aligned_value = alignment.get("aligned_route_path")
    if not isinstance(aligned_value, str) or not aligned_value:
        raise MatrixError("campaign aligned-route path is missing")
    aligned_route_path = _safe_inside(
        output_root,
        Path(aligned_value),
        "campaign aligned route",
    )
    if (
        alignment.get("status") != "PASS"
        or alignment.get("method")
        != "align_carla_route_to_map.prepare_aligned_route"
        or alignment.get("source_frame") != "carla_map"
        or alignment.get("target_frame") != "map"
        or alignment.get("input_already_aligned") is not False
        or Path(str(alignment.get("source_route_path", ""))).resolve()
        != raw_route_path
        or alignment.get("source_route_sha256") != raw_route_sha256
        or alignment.get("map_bundle_profile") != expected_profile
        or alignment.get("map_bundle_metadata_sha256")
        != bundle.get("metadata_sha256")
        or alignment.get("carla_to_map_transform") != expected_transform
        or case.get("route_path") != str(raw_route_path)
        or case.get("route_sha256") != raw_route_sha256
        or not raw_route_path.is_file()
        or sha256_file(raw_route_path) != raw_route_sha256
        or not aligned_route_path.is_file()
        or sha256_file(aligned_route_path)
        != alignment.get("aligned_route_sha256")
    ):
        raise MatrixError(
            "campaign route alignment does not bind raw route, aligned route, "
            "and admitted map bundle"
        )

    raw_payload = read_object(raw_route_path, "raw campaign route")
    aligned_payload = read_object(aligned_route_path, "aligned campaign route")
    try:
        expected_aligned = aligner.align_route_payload(
            raw_payload,
            bundle_transform,
            source_path=raw_route_path,
            source_sha256=raw_route_sha256,
            bundle=bundle_payload,
        )
    except Exception as error:
        raise MatrixError(
            f"cannot recompute campaign route alignment: {error}"
        ) from error
    expected_bytes = (
        json.dumps(
            expected_aligned,
            indent=2,
            sort_keys=False,
            ensure_ascii=True,
        )
        + "\n"
    ).encode("utf-8")
    if (
        aligned_payload != expected_aligned
        or hashlib.sha256(expected_bytes).hexdigest()
        != alignment.get("aligned_route_sha256")
    ):
        raise MatrixError(
            "campaign aligned route differs from the admitted bundle transform"
        )
    return {
        "path": str(aligned_route_path),
        "sha256": alignment["aligned_route_sha256"],
        "source_route_sha256": raw_route_sha256,
        "carla_to_map_transform": expected_transform,
    }


def _validate_route_matrix_provenance(
    output_root: Path,
    plan: Mapping[str, Any],
    entry: Mapping[str, Any],
    route_matrix: Mapping[str, Any],
    route_entry: Mapping[str, Any],
    route_analysis: Mapping[str, Any],
    trial_id: str,
) -> dict[str, Any] | None:
    contracts = _plan_map_route_generation_contracts(plan, entry)
    selection_overrides = _plan_map_route_selection_overrides(plan, entry)
    turn_selection_policy = _plan_turn_route_selection_policy(plan)
    expected = contracts[trial_id]
    expected_selection_override = selection_overrides.get(trial_id)
    selector = plan.get("runtime_profile_selector", "recommended")
    if (
        route_matrix.get("matrix_id") != plan.get("matrix_id")
        or route_matrix.get("map_id") != entry.get("map_id")
        or route_matrix.get("canonical_name") != entry.get("canonical_name")
        or route_matrix.get("runtime_profile_selector") != selector
        or route_matrix.get("admission_contract_sha256")
        != plan.get("admission_contract_sha256")
        or route_matrix.get("campaign_execution_contract_sha256")
        != plan.get("campaign_execution_contract_sha256")
        or route_matrix.get("route_generation_contracts") != contracts
        or route_matrix.get("route_generation_contracts_sha256")
        != sha256_json(contracts)
        or route_matrix.get("route_selection_policy") != ROUTE_SELECTION_POLICY
        or route_matrix.get("route_selection_overrides") != selection_overrides
        or route_matrix.get("route_selection_overrides_sha256")
        != sha256_json(selection_overrides)
        or route_matrix.get("turn_route_selection_policy")
        != turn_selection_policy
        or route_matrix.get("turn_route_selection_policy_sha256")
        != (
            sha256_json(turn_selection_policy)
            if turn_selection_policy is not None
            else None
        )
    ):
        raise MatrixError("route matrix provenance differs from the campaign plan")
    if (
        route_entry.get("route_generation_contract") != expected
        or route_entry.get("route_generation_contract_sha256")
        != sha256_json(expected)
    ):
        raise MatrixError(
            f"route matrix {trial_id} generation contract differs from the plan"
        )
    expected_selection_method = (
        SPEED_30KPH_TURN_ROUTE_SELECTION_METHOD
        if selector == "speed_30kph" and trial_id == "turn"
        else (
            ROUTE_SELECTION_POLICY
            if expected_selection_override is not None
            else "ranked_catalog_candidate"
        )
    )
    expected_selection_sha256 = (
        sha256_json(expected_selection_override)
        if expected_selection_override is not None
        else None
    )
    if (
        route_entry.get("route_selection_method") != expected_selection_method
        or route_entry.get("route_selection_override")
        != expected_selection_override
        or route_entry.get("route_selection_override_sha256")
        != expected_selection_sha256
    ):
        raise MatrixError(
            f"route matrix {trial_id} selection provenance differs from the plan"
        )
    route_length = _finite_number(
        route_analysis.get("route_length_m"), "trial source route length"
    )
    if (
        route_length < float(expected["minimum_distance_m"]) - 1.0e-9
        or route_length > float(expected["maximum_distance_m"]) + 1.0e-9
    ):
        raise MatrixError(
            f"trial source route length violates the {trial_id} generation contract"
        )
    catalog_value = route_entry.get("catalog_path")
    if not isinstance(catalog_value, str) or not catalog_value:
        raise MatrixError(f"route matrix {trial_id} catalog provenance is missing")
    catalog_path = _safe_inside(output_root, Path(catalog_value), "route catalog")
    if (
        not catalog_path.is_file()
        or sha256_file(catalog_path) != route_entry.get("catalog_sha256")
    ):
        raise MatrixError(f"route matrix {trial_id} catalog SHA256 changed")
    catalog = read_object(catalog_path, f"{trial_id} route catalog")
    server = catalog.get("server")
    active_name = (
        str(server.get("active_map_name", "")).rstrip("/").rsplit("/", 1)[-1]
        if isinstance(server, dict)
        else ""
    )
    if (
        catalog.get("status") != "complete"
        or catalog.get("map_id") != entry.get("map_id")
        or not isinstance(server, dict)
        or server.get("map_load_allowed") is not False
        or server.get("map_load_performed") is not False
        or active_name != entry.get("canonical_name")
    ):
        raise MatrixError(f"route matrix {trial_id} catalog provenance is invalid")
    _validate_catalog_generation(catalog, expected, trial_id)
    catalog_routes = catalog.get("routes")
    catalog_route = next(
        (
            item
            for item in catalog_routes
            if isinstance(item, dict)
            and item.get("id") == route_entry.get("route_id")
            and item.get("scenario") == route_entry.get("catalog_scenario")
        ),
        None,
    ) if isinstance(catalog_routes, list) else None
    catalog_route_value = (
        catalog_route.get("path") if isinstance(catalog_route, dict) else None
    )
    if not isinstance(catalog_route_value, str) or not catalog_route_value:
        raise MatrixError(f"route matrix {trial_id} route is absent from its catalog")
    catalog_route_path = _safe_inside(
        catalog_path.parent,
        catalog_path.parent / catalog_route_value,
        "catalog route",
    )
    catalog_seed = catalog_route.get("seed")
    catalog_pair_index = catalog_route.get("pair_index")
    route_seed = route_entry.get("route_seed")
    route_pair_index = route_entry.get("route_pair_index")
    if (
        any(
            isinstance(value, bool) or not isinstance(value, int) or value < 0
            for value in (
                catalog_seed,
                catalog_pair_index,
                route_seed,
                route_pair_index,
            )
        )
        or catalog_route_path
        != Path(str(route_entry.get("route_path", ""))).resolve()
        or catalog_route.get("sha256") != route_entry.get("route_sha256")
        or catalog_seed != route_seed
        or catalog_pair_index != route_pair_index
    ):
        raise MatrixError(
            f"route matrix {trial_id} route differs from its catalog record"
        )
    if expected_selection_override is not None and (
        catalog_route.get("scenario")
        != expected_selection_override.get("scenario")
        or catalog_seed != expected_selection_override.get("seed")
        or catalog_pair_index != expected_selection_override.get("pair_index")
    ):
        raise MatrixError(
            f"route matrix {trial_id} route violates its exact selection override"
        )
    if selector == "speed_30kph" and trial_id == "turn":
        if turn_selection_policy is None or expected_selection_override is not None:
            raise MatrixError("speed turn ranking plan is inconsistent")
        if not isinstance(catalog_routes, list):
            raise MatrixError("speed turn catalog routes are invalid")
        candidates = [
            candidate
            for scenario in ("left", "right")
            for candidate in catalog_routes
            if isinstance(candidate, dict)
            and candidate.get("status") == "ready"
            and candidate.get("scenario") == scenario
        ]
        bundle = entry.get("full_map_bundle")
        custom_map = (
            isinstance(bundle, dict)
            and bundle.get("bundle_schema") == "custom_map"
        )
        (
            expected_route,
            expected_audit,
            expected_common_turn_geometry,
            expected_custom_turn_geometry,
        ) = _rank_speed_30kph_turn_candidates(
            candidates,
            catalog_path,
            plan["runtime_profile"]["speed_contract"],
            turn_selection_policy,
            custom_map=custom_map,
        )
        if (
            route_entry.get("route_id") != expected_route.get("id")
            or route_entry.get("route_sha256") != expected_route.get("sha256")
            or route_entry.get("turn_route_selection_audit") != expected_audit
        ):
            raise MatrixError(
                "route matrix turn ranking vector or candidate-set digest differs "
                "from the exact catalog"
            )
        stored_route_analysis = route_entry.get("analysis")
        if not isinstance(stored_route_analysis, Mapping):
            raise MatrixError("route matrix turn analysis provenance is missing")
        if stored_route_analysis.get("physical_turn_preflight") != (
            expected_common_turn_geometry
        ):
            raise MatrixError(
                "route matrix common physical-turn analysis differs from "
                "fresh exact serialized analysis"
            )
        if custom_map:
            if stored_route_analysis.get("turn_geometry_preflight") != (
                expected_custom_turn_geometry
            ):
                raise MatrixError(
                    "route matrix custom turn-geometry analysis differs from "
                    "fresh exact serialized analysis"
                )
        elif stored_route_analysis.get("turn_geometry_preflight") is not None:
            raise MatrixError(
                "packaged route matrix unexpectedly carries custom "
                "turn-geometry analysis"
            )
    elif route_entry.get("turn_route_selection_audit") is not None:
        raise MatrixError(
            f"route matrix {trial_id} unexpectedly carries a turn ranking audit"
        )

    campaign = route_matrix.get("campaign_route_map_preflight")
    if selector == "recommended":
        if campaign is not None:
            raise MatrixError(
                "recommended route matrix unexpectedly carries a speed preflight"
            )
        return None
    if not isinstance(campaign, dict) or campaign.get("status") != "PASS":
        raise MatrixError("speed route matrix lacks a passing campaign map preflight")
    artifact_value = campaign.get("path")
    if not isinstance(artifact_value, str) or not artifact_value:
        raise MatrixError("campaign route/map preflight path is missing")
    artifact_path = _safe_inside(
        output_root, Path(artifact_value), "campaign route/map preflight"
    )
    if (
        not artifact_path.is_file()
        or sha256_file(artifact_path) != campaign.get("sha256")
    ):
        raise MatrixError("campaign route/map preflight SHA256 changed")
    artifact = read_object(artifact_path, "campaign route/map preflight")
    bundle = entry.get("full_map_bundle")
    live_bundle_hashes = (
        _verified_bundle_file_sha256(bundle, "trial validation")
        if isinstance(bundle, dict)
        else {}
    )
    if (
        artifact.get("schema_version") != 1
        or artifact.get("status") != "PASS"
        or artifact.get("matrix_id") != plan.get("matrix_id")
        or artifact.get("runtime_profile_selector") != selector
        or artifact.get("runtime_profile_id")
        != plan.get("runtime_profile", {}).get("id")
        or artifact.get("admission_contract_sha256")
        != plan.get("admission_contract_sha256")
        or artifact.get("campaign_execution_contract_sha256")
        != plan.get("campaign_execution_contract_sha256")
        or artifact.get("map_id") != entry.get("map_id")
        or artifact.get("canonical_name") != entry.get("canonical_name")
        or not isinstance(bundle, dict)
        or Path(str(artifact.get("map_bundle_path", ""))).resolve()
        != Path(str(bundle.get("path", ""))).resolve()
        or artifact.get("map_bundle_metadata_sha256")
        != bundle.get("metadata_sha256")
        or artifact.get("map_bundle_file_sha256")
        != bundle.get("bundle_file_sha256")
        or live_bundle_hashes != bundle.get("bundle_file_sha256")
        or artifact.get("route_generation_contracts_sha256")
        != sha256_json(contracts)
    ):
        raise MatrixError("campaign route/map preflight provenance is invalid")
    cases = artifact.get("cases")
    if (
        not isinstance(cases, list)
        or len(cases) != len(TRIAL_IDS)
        or not all(isinstance(item, dict) for item in cases)
        or {item.get("trial_id") for item in cases} != set(TRIAL_IDS)
    ):
        raise MatrixError("campaign route/map preflight cases are incomplete")
    case = next(
        (
            item
            for item in cases
            if isinstance(item, dict) and item.get("trial_id") == trial_id
        ),
        None,
    )
    alignment_validation = (
        _validate_campaign_route_alignment(
            output_root,
            artifact,
            bundle,
            route_entry,
            case,
        )
        if isinstance(case, dict) and isinstance(bundle, dict)
        else None
    )
    pcd = case.get("pointcloud_proximity") if isinstance(case, dict) else None
    lanelet = case.get("lanelet2") if isinstance(case, dict) else None
    if (
        not isinstance(case, dict)
        or case.get("status") != "PASS"
        or case.get("scenario") != route_entry.get("catalog_scenario")
        or case.get("route_sha256") != route_entry.get("route_sha256")
        or not math.isclose(
            _finite_number(case.get("route_length_m"), "preflight route length"),
            route_length,
            rel_tol=0.0,
            abs_tol=1.0e-9,
        )
        or not isinstance(lanelet, dict)
        or lanelet.get("status") != "PASS"
        or not isinstance(pcd, dict)
        or pcd.get("status") != "PASS"
    ):
        raise MatrixError(
            f"campaign route/map preflight does not bind the {trial_id} route"
        )
    return {
        "status": "PASS",
        "path": str(artifact_path),
        "sha256": campaign["sha256"],
        "route_sha256": route_entry["route_sha256"],
        "aligned_route": alignment_validation,
        "route_generation_contract_sha256": sha256_json(expected),
    }


def select_routes(
    output_root: Path,
    map_id: str,
    catalog_path: Path | None = None,
    *,
    straight_catalog_path: Path | None = None,
    turn_catalog_path: Path | None = None,
) -> dict[str, Any]:
    output_root = output_root.expanduser().resolve()
    plan = _load_verified_campaign_plan(output_root, f"map {map_id} route selection")
    entry = _map_entry(plan, map_id)
    if entry.get("runnable") is not True:
        raise MatrixError(f"map {map_id} is blocked by full-map admission")
    generation_contracts = _plan_map_route_generation_contracts(plan, entry)
    selection_overrides = _plan_map_route_selection_overrides(plan, entry)
    shared_catalog = catalog_path is not None
    split_catalogs = (
        straight_catalog_path is not None or turn_catalog_path is not None
    )
    if shared_catalog == split_catalogs:
        raise MatrixError(
            "select-routes requires either one shared catalog or both "
            "scenario-specific catalogs"
        )
    if shared_catalog:
        assert catalog_path is not None
        resolved = catalog_path.expanduser().resolve()
        catalog_paths = {trial_id: resolved for trial_id in TRIAL_IDS}
    else:
        if straight_catalog_path is None or turn_catalog_path is None:
            raise MatrixError(
                "select-routes requires both straight and turn catalogs"
            )
        catalog_paths = {
            "straight": straight_catalog_path.expanduser().resolve(),
            "turn": turn_catalog_path.expanduser().resolve(),
        }
    if (
        plan.get("runtime_profile_selector", "recommended") == "speed_30kph"
        and catalog_paths["straight"] == catalog_paths["turn"]
    ):
        raise MatrixError(
            "speed_30kph requires separate straight and turn generation catalogs"
        )

    bundle = entry.get("full_map_bundle")
    custom_map = (
        isinstance(bundle, dict) and bundle.get("bundle_schema") == "custom_map"
    )
    readiness = bundle.get("readiness") if isinstance(bundle, dict) else None
    audited_route_hashes = {
        (str(item.get("scenario")), str(item.get("route_sha256")))
        for item in (
            readiness.get("admitted_routes", [])
            if isinstance(readiness, dict)
            else []
        )
        if isinstance(item, dict)
    }
    catalogs: dict[Path, dict[str, Any]] = {}
    for trial_id, path in catalog_paths.items():
        if path not in catalogs:
            catalog = read_object(path, f"{trial_id} route catalog")
            if catalog.get("status") != "complete" or catalog.get("map_id") != map_id:
                raise MatrixError(
                    f"{trial_id} route catalog is not complete for the requested map"
                )
            server = catalog.get("server")
            if not isinstance(server, dict):
                raise MatrixError("route catalog server provenance is missing")
            if (
                server.get("map_load_allowed") is not False
                or server.get("map_load_performed") is not False
            ):
                raise MatrixError("route catalog used or allowed client-side map loading")
            active_name = (
                str(server.get("active_map_name", ""))
                .rstrip("/")
                .rsplit("/", 1)[-1]
            )
            if active_name != entry["canonical_name"]:
                raise MatrixError(
                    f"route catalog active map mismatch: {active_name!r} != "
                    f"{entry['canonical_name']!r}"
                )
            if custom_map:
                _validate_catalog_initial_approach_contract(catalog)
                _validate_catalog_turn_geometry_contract(catalog)
            if not isinstance(catalog.get("routes"), list):
                raise MatrixError("route catalog routes must be a list")
            catalogs[path] = catalog
        _validate_catalog_generation(
            catalogs[path], generation_contracts[trial_id], trial_id
        )
        if plan.get("runtime_profile_selector") == "speed_30kph":
            _validate_speed_catalog_endpoint_policy(
                catalogs[path],
                trial_id,
                generation_contracts[trial_id],
                custom_map=custom_map,
            )

    selected: list[dict[str, Any]] = []
    for trial_id, scenarios in (("straight", ("straight",)), ("turn", ("left", "right"))):
        current_catalog_path = catalog_paths[trial_id]
        catalog = catalogs[current_catalog_path]
        routes = catalog["routes"]
        candidates = [
            route
            for scenario in scenarios
            for route in routes
            if isinstance(route, dict)
            and route.get("status") == "ready"
            and route.get("scenario") == scenario
        ]
        if not candidates:
            raise MatrixError(
                f"catalog has no route satisfying {trial_id} scenarios {list(scenarios)}"
            )
        turn_selection_audit: dict[str, Any] | None = None
        selected_turn_geometry: dict[str, Any] | None = None
        selected_custom_turn_geometry: dict[str, Any] | None = None
        if (
            plan.get("runtime_profile_selector") == "speed_30kph"
            and trial_id == "straight"
        ):
            physically_admissible = []
            rejected_reasons = []
            required_geometry = plan["runtime_profile"]["speed_contract"][
                "trials"
            ]["straight"]["physical_geometry"]
            for candidate in candidates:
                candidate_value = candidate.get("path")
                if not isinstance(candidate_value, str) or not candidate_value:
                    raise MatrixError("catalog straight route candidate has no path")
                candidate_path = _safe_inside(
                    current_catalog_path.parent,
                    current_catalog_path.parent / candidate_value,
                    "route",
                )
                if not candidate_path.is_file():
                    raise MatrixError(f"catalog route is missing: {candidate_path}")
                if sha256_file(candidate_path) != candidate.get("sha256"):
                    raise MatrixError(
                        f"catalog route SHA256 mismatch: {candidate_path}"
                    )
                candidate_payload = read_object(
                    candidate_path, "straight route candidate"
                )
                try:
                    _speed_30kph_straight_geometry(
                        candidate_payload, required_geometry
                    )
                except (MatrixError, TypeError) as error:
                    rejected_reasons.append(
                        {
                            "route_id": str(candidate.get("id", "")),
                            "reason": str(error),
                        }
                    )
                    continue
                physically_admissible.append(candidate)
            if not physically_admissible:
                raise MatrixError(
                    "30 kph physical-straight preflight failed for every catalog "
                    f"candidate: {rejected_reasons}"
                )
            candidates = physically_admissible
        if (
            plan.get("runtime_profile_selector") == "speed_30kph"
            and trial_id == "turn"
        ):
            turn_policy = _plan_turn_route_selection_policy(plan)
            if turn_policy is None:
                raise MatrixError("30 kph turn route has no ranking policy")
            (
                route,
                turn_selection_audit,
                selected_turn_geometry,
                selected_custom_turn_geometry,
            ) = _rank_speed_30kph_turn_candidates(
                candidates,
                current_catalog_path,
                plan["runtime_profile"]["speed_contract"],
                turn_policy,
                custom_map=custom_map,
            )

        def maneuver_key(route: Mapping[str, Any]) -> tuple[Any, ...]:
            value = route.get("path")
            if not isinstance(value, str) or not value:
                return (2, scenarios.index(str(route.get("scenario"))))
            route_path = _safe_inside(
                current_catalog_path.parent,
                current_catalog_path.parent / value,
                "route",
            )
            payload = read_object(route_path, f"{trial_id} route candidate")
            counts = payload.get("option_counts")
            lane_changes = (
                int(counts.get("CHANGELANELEFT", 0))
                + int(counts.get("CHANGELANERIGHT", 0))
                if isinstance(counts, dict)
                else 1
            )
            return (
                0
                if (str(route.get("scenario")), sha256_file(route_path))
                in audited_route_hashes
                else 1,
                1 if lane_changes else 0,
                scenarios.index(str(route.get("scenario"))),
                int(route.get("seed", 0)),
                int(route.get("pair_index", 0)),
                str(route.get("id", "")),
            )

        selection_override = selection_overrides.get(trial_id)
        if turn_selection_audit is not None:
            if selection_override is not None:
                raise MatrixError(
                    "30 kph turn ranking forbids exact route-selection overrides"
                )
            route_selection_method = SPEED_30KPH_TURN_ROUTE_SELECTION_METHOD
        elif selection_override is None:
            candidates.sort(key=maneuver_key)
            route = candidates[0]
            route_selection_method = "ranked_catalog_candidate"
        else:
            route = _select_exact_route_override(
                candidates, selection_override, map_id, trial_id
            )
            route_selection_method = ROUTE_SELECTION_POLICY
        value = route.get("path")
        if not isinstance(value, str) or not value:
            raise MatrixError(f"catalog {trial_id} route has no path")
        route_path = _safe_inside(
            current_catalog_path.parent,
            current_catalog_path.parent / value,
            "route",
        )
        if not route_path.is_file():
            raise MatrixError(f"catalog route is missing: {route_path}")
        expected_seeds = generation_contracts[trial_id]["seeds"]
        expected_pairs = generation_contracts[trial_id]["pairs_per_seed"]
        if route.get("seed") not in expected_seeds:
            raise MatrixError(f"selected {trial_id} route seed violates its contract")
        pair_index = route.get("pair_index")
        if (
            isinstance(pair_index, bool)
            or not isinstance(pair_index, int)
            or pair_index < 0
            or pair_index >= expected_pairs
        ):
            raise MatrixError(
                f"selected {trial_id} route pair index violates its contract"
            )
        route_hash = sha256_file(route_path)
        if route_hash != route.get("sha256"):
            raise MatrixError(f"catalog route SHA256 mismatch: {route_path}")
        payload = read_object(route_path, f"{trial_id} route")
        analysis = _validate_route_payload(
            payload, entry["canonical_name"], str(route["scenario"])
        )
        minimum_distance = float(
            generation_contracts[trial_id]["minimum_distance_m"]
        )
        maximum_distance = float(
            generation_contracts[trial_id]["maximum_distance_m"]
        )
        if (
            analysis["route_length_m"] < minimum_distance - 1.0e-9
            or analysis["route_length_m"] > maximum_distance + 1.0e-9
        ):
            raise MatrixError(
                f"selected {trial_id} route length {analysis['route_length_m']:.3f} m "
                f"is outside [{minimum_distance:.3f}, {maximum_distance:.3f}] m"
            )
        analysis["route_generation_contract"] = dict(
            generation_contracts[trial_id]
        )
        if (
            plan.get("runtime_profile_selector") == "speed_30kph"
            and trial_id == "straight"
        ):
            analysis["physical_straight_preflight"] = (
                _speed_30kph_straight_geometry(
                    payload,
                    plan["runtime_profile"]["speed_contract"]["trials"][
                        "straight"
                    ]["physical_geometry"],
                )
            )
            if not custom_map and (
                route.get("physical_straight_preflight")
                != analysis["physical_straight_preflight"]
                or payload.get("physical_straight_preflight")
                != analysis["physical_straight_preflight"]
            ):
                raise MatrixError(
                    "packaged straight route physical-geometry provenance "
                    "does not match fresh analysis"
                )
            analysis["distance_capacity_tier"] = (
                "nominal_200_plus"
                if analysis["route_length_m"] >= 200.0
                else "capacity_170_199"
            )
        if (
            plan.get("runtime_profile_selector") == "speed_30kph"
            and trial_id == "turn"
        ):
            if selected_turn_geometry is None:
                raise MatrixError("selected turn has no common ranking geometry")
            analysis["physical_turn_preflight"] = selected_turn_geometry
            if not custom_map and (
                route.get("physical_turn_preflight")
                != analysis["physical_turn_preflight"]
                or payload.get("physical_turn_preflight")
                != analysis["physical_turn_preflight"]
            ):
                raise MatrixError(
                    "packaged turn route physical-geometry provenance "
                    "does not match fresh analysis"
                )
        if custom_map:
            declared = payload.get("initial_approach_preflight")
            catalog_declared = route.get("initial_approach_preflight")
            if (
                not isinstance(declared, dict)
                or declared.get("status") != "PASS"
                or not isinstance(catalog_declared, dict)
                or catalog_declared.get("status") != "PASS"
            ):
                raise MatrixError(
                    f"selected {trial_id} custom-map route lacks a PASS "
                    "initial-approach preflight"
                )
            analysis["initial_approach_preflight"] = (
                _custom_route_initial_approach(payload)
            )
            if trial_id == "turn":
                declared_turn = payload.get("turn_geometry_preflight")
                catalog_declared_turn = route.get("turn_geometry_preflight")
                if (
                    not isinstance(declared_turn, dict)
                    or declared_turn.get("status") != "PASS"
                    or not isinstance(catalog_declared_turn, dict)
                    or catalog_declared_turn.get("status") != "PASS"
                ):
                    raise MatrixError(
                        "selected turn custom-map route lacks a PASS "
                        "turn-geometry preflight"
                    )
                fresh_custom_turn = _custom_route_turn_geometry(
                    payload, str(route["scenario"])
                )
                if (
                    selected_custom_turn_geometry is not None
                    and fresh_custom_turn != selected_custom_turn_geometry
                ):
                    raise MatrixError(
                        "selected custom turn differs from its saved custom "
                        "geometry preflight"
                    )
                analysis["turn_geometry_preflight"] = fresh_custom_turn
        if (
            plan.get("runtime_profile_selector", "recommended") == "recommended"
            and isinstance(readiness, dict)
        ):
            preflight_match = next(
                (
                    item
                    for item in readiness.get("admitted_routes", [])
                    if item.get("scenario") == route["scenario"]
                    and item.get("route_sha256") == route_hash
                ),
                None,
            )
            if preflight_match is None:
                raise MatrixError(
                    f"selected {trial_id} route did not pass the admitted exact-route "
                    "Lanelet2 + point-cloud preflight"
                )
            analysis["map_pointcloud_preflight"] = preflight_match
        selected.append(
            {
                "trial_id": trial_id,
                "catalog_scenario": route["scenario"],
                "turn_direction": route["scenario"] if trial_id == "turn" else None,
                "route_id": route["id"],
                "route_seed": route["seed"],
                "route_pair_index": route["pair_index"],
                "route_path": str(route_path),
                "route_sha256": route_hash,
                "catalog_path": str(current_catalog_path),
                "catalog_sha256": sha256_file(current_catalog_path),
                "route_generation_contract": dict(
                    generation_contracts[trial_id]
                ),
                "route_generation_contract_sha256": sha256_json(
                    generation_contracts[trial_id]
                ),
                "route_selection_method": route_selection_method,
                "route_selection_override": (
                    dict(selection_override)
                    if selection_override is not None
                    else None
                ),
                "route_selection_override_sha256": (
                    sha256_json(selection_override)
                    if selection_override is not None
                    else None
                ),
                "turn_route_selection_audit": (
                    turn_selection_audit if trial_id == "turn" else None
                ),
                "analysis": analysis,
            }
        )
    campaign_preflight = None
    if plan.get("runtime_profile_selector", "recommended") == "speed_30kph":
        campaign_preflight = _campaign_route_map_preflight(
            output_root, plan, entry, selected
        )
        by_trial = {
            str(case["trial_id"]): case for case in campaign_preflight["cases"]
        }
        if set(by_trial) != set(TRIAL_IDS):
            raise MatrixError("campaign route/map preflight cases are incomplete")
        for item in selected:
            case = by_trial[item["trial_id"]]
            if (
                case.get("status") != "PASS"
                or case.get("route_sha256") != item["route_sha256"]
                or case.get("scenario") != item["catalog_scenario"]
            ):
                raise MatrixError(
                    f"campaign route/map preflight does not bind {item['trial_id']}"
                )
            item["analysis"]["map_pointcloud_preflight"] = case
    payload = {
        "schema_version": 1,
        "matrix_id": plan["matrix_id"],
        "admission_contract_sha256": plan["admission_contract_sha256"],
        "campaign_execution_contract_sha256": plan[
            "campaign_execution_contract_sha256"
        ],
        "map_id": map_id,
        "canonical_name": entry["canonical_name"],
        "catalog_path": (
            str(catalog_paths["straight"])
            if catalog_paths["straight"] == catalog_paths["turn"]
            else None
        ),
        "catalog_sha256": (
            sha256_file(catalog_paths["straight"])
            if catalog_paths["straight"] == catalog_paths["turn"]
            else None
        ),
        "catalogs": {
            trial_id: {
                "path": str(path),
                "sha256": sha256_file(path),
                "route_generation_contract": generation_contracts[trial_id],
                "route_generation_contract_sha256": sha256_json(
                    generation_contracts[trial_id]
                ),
            }
            for trial_id, path in catalog_paths.items()
        },
        "runtime_profile_selector": plan.get(
            "runtime_profile_selector", "recommended"
        ),
        "route_generation_contracts": generation_contracts,
        "route_generation_contracts_sha256": sha256_json(
            generation_contracts
        ),
        "route_selection_policy": ROUTE_SELECTION_POLICY,
        "route_selection_overrides": selection_overrides,
        "route_selection_overrides_sha256": sha256_json(selection_overrides),
        "turn_route_selection_policy": _plan_turn_route_selection_policy(plan),
        "turn_route_selection_policy_sha256": plan.get(
            "turn_route_selection_policy_sha256"
        ),
        "campaign_route_map_preflight": (
            {
                "path": campaign_preflight["path"],
                "sha256": campaign_preflight["sha256"],
                "status": campaign_preflight["status"],
            }
            if campaign_preflight is not None
            else None
        ),
        "client_map_loading_allowed": False,
        "custom_map_initial_approach_contract": (
            dict(CUSTOM_MAP_INITIAL_APPROACH_CONTRACT) if custom_map else None
        ),
        "custom_map_turn_geometry_contract": (
            dict(CUSTOM_MAP_TURN_GEOMETRY_CONTRACT) if custom_map else None
        ),
        "selected_at": utc_now(),
        "trials": selected,
    }
    destination = output_root / "maps" / map_id / "route_matrix.json"
    _verify_campaign_plan(plan, f"map {map_id} route-selection completion")
    atomic_json(destination, payload)
    return payload


def select_admitted_routes(output_root: Path, map_id: str) -> dict[str, Any]:
    """Materialize the fixed seed routes from the packaged-Town preflight.

    This is an offline analysis plan only. The runner still cold-starts CARLA,
    regenerates the same seed catalog without client map loading, and replaces
    this file through :func:`select_routes` before any VAD trial starts.
    """
    output_root = output_root.expanduser().resolve()
    plan = _load_verified_campaign_plan(
        output_root, f"map {map_id} admitted-route selection"
    )
    if plan.get("runtime_profile_selector", "recommended") != "recommended":
        raise MatrixError(
            "admitted baseline routes cannot be selected for an opt-in speed profile"
        )
    entry = _map_entry(plan, map_id)
    bundle = entry.get("full_map_bundle")
    readiness = bundle.get("readiness") if isinstance(bundle, dict) else None
    if not isinstance(readiness, dict):
        raise MatrixError(
            f"map {map_id} has no packaged-Town exact-route readiness record"
        )
    seed = int(plan["route_contract"]["seed"])
    seed_token = f"_s{seed:04d}_"
    admitted = [
        item
        for item in readiness.get("admitted_routes", [])
        if isinstance(item, dict) and seed_token in Path(str(item.get("route_path"))).stem
    ]
    selected: list[dict[str, Any]] = []
    for trial_id, scenarios in (("straight", ("straight",)), ("turn", ("left", "right"))):
        candidates: list[tuple[tuple[Any, ...], Mapping[str, Any], dict[str, Any]]] = []
        for item in admitted:
            scenario = item.get("scenario")
            if scenario not in scenarios:
                continue
            route_path = Path(str(item["route_path"])).resolve()
            route_hash = sha256_file(route_path)
            if route_hash != item.get("route_sha256"):
                raise MatrixError(f"admitted route SHA256 changed: {route_path}")
            route_payload = read_object(route_path, f"{trial_id} admitted route")
            counts = route_payload.get("option_counts")
            lane_changes = (
                int(counts.get("CHANGELANELEFT", 0))
                + int(counts.get("CHANGELANERIGHT", 0))
                if isinstance(counts, dict)
                else 1
            )
            key = (
                1 if lane_changes else 0,
                scenarios.index(str(scenario)),
                route_path.name,
            )
            candidates.append((key, item, route_payload))
        if not candidates:
            raise MatrixError(
                f"readiness has no seed-{seed} route for {trial_id} scenarios "
                f"{list(scenarios)}"
            )
        candidates.sort(key=lambda candidate: candidate[0])
        _, preflight, route_payload = candidates[0]
        scenario = str(preflight["scenario"])
        analysis = _validate_route_payload(
            route_payload, entry["canonical_name"], scenario
        )
        analysis["map_pointcloud_preflight"] = dict(preflight)
        route_path = Path(str(preflight["route_path"])).resolve()
        selected.append(
            {
                "trial_id": trial_id,
                "catalog_scenario": scenario,
                "turn_direction": scenario if trial_id == "turn" else None,
                "route_id": route_path.stem,
                "route_path": str(route_path),
                "route_sha256": preflight["route_sha256"],
                "analysis": analysis,
            }
        )
    payload = {
        "schema_version": 1,
        "matrix_id": plan["matrix_id"],
        "admission_contract_sha256": plan["admission_contract_sha256"],
        "campaign_execution_contract_sha256": plan[
            "campaign_execution_contract_sha256"
        ],
        "map_id": map_id,
        "canonical_name": entry["canonical_name"],
        "selection_source": "packaged_town_exact_route_map_pointcloud_preflight",
        "readiness_artifact": readiness["artifact_path"],
        "readiness_artifact_sha256": readiness["artifact_sha256"],
        "route_seed": seed,
        "client_map_loading_allowed": False,
        "execution_note": (
            "Offline route analysis only; the runner must regenerate and verify "
            "this deterministic catalog on a cold-start server before execution."
        ),
        "selected_at": utc_now(),
        "trials": selected,
    }
    destination = output_root / "maps" / map_id / "route_matrix.json"
    _verify_campaign_plan(plan, f"map {map_id} admitted-route completion")
    atomic_json(destination, payload)
    return payload


def _runtime_env(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as error:
        raise MatrixError(f"cannot read runtime provenance {path}: {error}") from error
    for line in lines:
        if "=" in line:
            key, value = line.split("=", 1)
            if not key or key in values:
                raise MatrixError(
                    f"runtime provenance has an empty or duplicate key: {key!r}"
                )
            values[key] = value
    return values


def _runtime_number(
    runtime: Mapping[str, str], field: str, expected: float
) -> float:
    raw = runtime.get(field)
    try:
        actual = float(raw) if raw is not None else math.nan
    except ValueError as error:
        raise MatrixError(f"runtime.env {field} is not numeric: {raw!r}") from error
    if not math.isfinite(actual) or not math.isclose(
        actual, expected, rel_tol=0.0, abs_tol=1.0e-9
    ):
        raise MatrixError(
            f"runtime.env {field} mismatch: expected={expected!r} actual={raw!r}"
        )
    return actual


def _yaml_mapping(path: Path, label: str) -> Mapping[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise MatrixError(f"cannot read {label} {path}: {error}") from error
    if not isinstance(value, dict):
        raise MatrixError(f"{label} root must be an object: {path}")
    return value


def _ros_parameters(path: Path, node_name: str, label: str) -> Mapping[str, Any]:
    document = _yaml_mapping(path, label)
    node = document.get(node_name)
    if not isinstance(node, dict):
        raise MatrixError(f"{label} lacks node {node_name!r}")
    parameters = node.get("ros__parameters")
    if not isinstance(parameters, dict):
        raise MatrixError(f"{label} lacks ros__parameters")
    return parameters


def _parameter_value(
    parameters: Mapping[str, Any], dotted_name: str, label: str
) -> Any:
    if dotted_name in parameters:
        return parameters[dotted_name]
    value: Any = parameters
    for component in dotted_name.split("."):
        if not isinstance(value, dict) or component not in value:
            raise MatrixError(f"{label} lacks parameter {dotted_name!r}")
        value = value[component]
    return value


def _parameter_number(
    parameters: Mapping[str, Any], field: str, expected: float, label: str
) -> float:
    actual = _finite_number(_parameter_value(parameters, field, label), f"{label}.{field}")
    if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1.0e-9):
        raise MatrixError(
            f"{label}.{field} mismatch: expected={expected!r} actual={actual!r}"
        )
    return actual


def _parameter_number_list(
    parameters: Mapping[str, Any], field: str, expected: float, label: str
) -> list[float]:
    raw = _parameter_value(parameters, field, label)
    if not isinstance(raw, list) or not raw:
        raise MatrixError(f"{label}.{field} must be a non-empty list")
    values = [_finite_number(item, f"{label}.{field}") for item in raw]
    if any(
        not math.isclose(item, expected, rel_tol=0.0, abs_tol=1.0e-9)
        for item in values
    ):
        raise MatrixError(
            f"{label}.{field} must contain only {expected!r}, got {values!r}"
        )
    return values


def _flatten_parameters(
    parameters: Mapping[str, Any], prefix: str = ""
) -> dict[str, Any]:
    flattened: dict[str, Any] = {}
    for field, value in parameters.items():
        name = f"{prefix}.{field}" if prefix else str(field)
        if isinstance(value, dict):
            flattened.update(_flatten_parameters(value, name))
        else:
            flattened[name] = value
    return flattened


def _parameter_values_match(actual: Any, expected: Any) -> bool:
    if isinstance(expected, bool):
        return actual is expected
    if isinstance(expected, (int, float)) and not isinstance(expected, bool):
        return (
            isinstance(actual, (int, float))
            and not isinstance(actual, bool)
            and math.isfinite(float(actual))
            and math.isclose(
                float(actual), float(expected), rel_tol=0.0, abs_tol=1.0e-9
            )
        )
    if isinstance(expected, list):
        return isinstance(actual, list) and len(actual) == len(expected) and all(
            _parameter_values_match(actual_item, expected_item)
            for actual_item, expected_item in zip(actual, expected)
        )
    return actual == expected


def _sha256_manifest(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as error:
        raise MatrixError(f"cannot read SHA256 manifest {path}: {error}") from error
    for line in lines:
        parts = line.split(None, 1)
        if len(parts) != 2 or not re.fullmatch(r"[0-9a-f]{64}", parts[0]):
            raise MatrixError(f"malformed SHA256 manifest line: {line!r}")
        name = parts[1].lstrip("*")
        if not name or name in values:
            raise MatrixError(f"unsafe or duplicate SHA256 manifest path: {name!r}")
        values[name] = parts[0]
    return values


def _rosbag_manifest(path: Path, label: str) -> dict[str, Any]:
    root = path.expanduser()
    if root.is_symlink():
        raise MatrixError(f"{label} rosbag root must not be a symlink: {root}")
    root = root.resolve()
    if not root.is_dir():
        raise MatrixError(f"{label} rosbag directory is missing: {root}")
    files: list[dict[str, Any]] = []
    for source in sorted(root.rglob("*")):
        if source.is_symlink():
            raise MatrixError(f"{label} rosbag contains a symlink: {source}")
        if not source.is_file():
            continue
        files.append(
            {
                "path": source.relative_to(root).as_posix(),
                "size_bytes": source.stat().st_size,
                "sha256": sha256_file(source),
            }
        )
    if not files or not any(item["path"] == "metadata.yaml" for item in files):
        raise MatrixError(f"{label} rosbag has no metadata.yaml")
    manifest = {"schema_version": 1, "root": str(root), "files": files}
    manifest["sha256"] = sha256_json(
        {"schema_version": manifest["schema_version"], "files": files}
    )
    return manifest


def _speed_profile_artifact_evidence(
    trial_dir: Path,
    result: Mapping[str, Any],
    trial_id: str,
    scenario: str,
) -> dict[str, str]:
    label = f"{trial_id} speed profile"
    profile_path = trial_dir / "speed_profile.json"
    plot_path = trial_dir / "speed_profile.png"
    profile = read_object(profile_path, label)
    if (
        profile.get("schema_version") != 1
        or profile.get("analysis") != "carla_speed_source_evidence"
        or profile.get("status") != "complete"
        or profile.get("outputs")
        != {"json": "speed_profile.json", "plot": "speed_profile.png"}
    ):
        raise MatrixError(f"{label} has an invalid or incomplete identity")
    quality = profile.get("quality")
    if not isinstance(quality, dict) or quality.get("problems") != []:
        raise MatrixError(f"{label} reports incomplete source evidence")
    _image_size(plot_path, "PNG")

    identity = profile.get("source_identity")
    if not isinstance(identity, dict) or identity.get("schema_version") != 1:
        raise MatrixError(f"{label} has no immutable source identity")
    identity_sha256 = identity.get("sha256")
    if (
        not isinstance(identity_sha256, str)
        or not re.fullmatch(r"[0-9a-f]{64}", identity_sha256)
        or identity_sha256
        != sha256_json(
            {key: value for key, value in identity.items() if key != "sha256"}
        )
    ):
        raise MatrixError(f"{label} source-identity digest mismatch")

    result_path = (trial_dir / "result.json").resolve()
    result_route_value = result.get("route_file")
    if not isinstance(result_route_value, str) or not result_route_value:
        raise MatrixError(f"{label} route result has no effective route")
    result_route_path = Path(result_route_value).expanduser()
    if not result_route_path.is_absolute():
        result_route_path = result_path.parent / result_route_path
    result_route_path = _safe_inside(
        trial_dir, result_route_path, f"{label} effective route"
    )
    if not result_route_path.is_file():
        raise MatrixError(f"{label} effective route is missing")

    route_identity = identity.get("effective_route")
    result_identity = identity.get("route_result")
    bag_identity = identity.get("rosbag")
    if (
        not isinstance(route_identity, dict)
        or not isinstance(result_identity, dict)
        or not isinstance(bag_identity, dict)
    ):
        raise MatrixError(f"{label} source identity is incomplete")
    recorded_route_path = Path(str(route_identity.get("path", ""))).expanduser()
    recorded_result_path = Path(str(result_identity.get("path", ""))).expanduser()
    if (
        recorded_route_path.resolve() != result_route_path
        or recorded_result_path.resolve() != result_path
        or route_identity.get("sha256") != sha256_file(result_route_path)
        or result_identity.get("sha256") != sha256_file(result_path)
        or route_identity.get("scenario") != scenario
        or route_identity.get("trial_id") != trial_id
        or result_identity.get("success") is not True
        or result_identity.get("execution_mode") != "full_stack"
        or result_identity.get("profile_context") != result.get("profile_context")
    ):
        raise MatrixError(f"{label} is bound to a different route/result")

    bag = _rosbag_manifest(trial_dir / "bag", label)
    if bag_identity != bag:
        raise MatrixError(f"{label} rosbag manifest differs from live evidence")
    inputs = profile.get("inputs")
    if (
        not isinstance(inputs, dict)
        or Path(str(inputs.get("bag", ""))).expanduser().resolve()
        != (trial_dir / "bag").resolve()
    ):
        raise MatrixError(f"{label} input bag path differs from source identity")
    return {
        "speed_profile_json_sha256": sha256_file(profile_path),
        "speed_profile_plot_sha256": sha256_file(plot_path),
        "speed_profile_source_identity_sha256": identity_sha256,
        "speed_profile_result_sha256": str(result_identity["sha256"]),
        "speed_profile_route_sha256": str(route_identity["sha256"]),
        "speed_profile_bag_manifest_sha256": str(bag["sha256"]),
    }


def _speed_contract_evidence(
    trial_dir: Path,
    runtime: Mapping[str, str],
    result: Mapping[str, Any],
    profile: Mapping[str, Any],
    trial_id: str,
    scenario: str,
) -> dict[str, Any] | None:
    contract = profile.get("speed_contract")
    if contract is None:
        return None
    if not isinstance(contract, dict):
        raise MatrixError("selected runtime profile has an invalid speed_contract")
    trials = contract.get("trials")
    if not isinstance(trials, dict) or not isinstance(trials.get(trial_id), dict):
        raise MatrixError(f"speed_contract has no {trial_id!r} trial contract")
    trial_contract = trials[trial_id]
    parameters = contract.get("route_manager_parameters")
    gate_contract = contract.get("vehicle_cmd_gate")
    controller_contract = contract.get("longitudinal_controller")
    if (
        not isinstance(parameters, dict)
        or not isinstance(gate_contract, dict)
        or not isinstance(controller_contract, dict)
    ):
        raise MatrixError("selected speed_contract is incomplete")

    if runtime.get("VSCODE_SNAP_GUI_ENV_SANITIZED") not in {"true", "false"}:
        raise MatrixError(
            "runtime.env VSCODE_SNAP_GUI_ENV_SANITIZED must record the GUI "
            "runtime isolation decision"
        )

    string_runtime = {
        "SPEED_30KPH": "true",
        "TIGHT_CORRIDOR_CANDIDATE": "false",
        "TRAJECTORY_STABILITY_CANDIDATE": "false",
        "SMART_MPC": "false",
        "FP16_HEADS": "false",
        "SPEED_PROFILE_ID": str(contract["profile_id"]),
        "ROUTE_SCENARIO": scenario,
        "SPEED_EXPOSURE_MODE": str(trial_contract["exposure_mode"]),
        "LONGITUDINAL_SPEED_SOURCE": str(
            contract["longitudinal_speed_source"]
        ),
        "LONGITUDINAL_ACCELERATION_ROLE": str(
            contract["longitudinal_acceleration_role"]
        ),
        "VAD_GEOMETRY_SOURCE": "true",
        "VAD_VELOCITY_EVALUATED": "false",
        "VAD_GEOMETRY_EVALUATED": "true",
        "VAD_CRUISE_VELOCITY_EVALUATED": "false",
        "VAD_HARD_STOP_SENTINEL_PRESERVED": "true",
        "CLOSED_LOOP_VALIDATION_STATE": str(contract["validation_state"]),
        "SPEED_LIMIT_SOURCE": str(gate_contract["speed_limit_source"]),
        "REAL_VEHICLE_READY": "false",
    }
    for field, expected in string_runtime.items():
        if runtime.get(field) != expected:
            raise MatrixError(
                f"runtime.env {field} mismatch: expected={expected!r} "
                f"actual={runtime.get(field)!r}"
            )
    runtime_numbers = {
        "TARGET_SPEED_MPS": float(contract["target_speed_mps"]),
        "TARGET_SPEED_KPH": 30.0,
        "MINIMUM_SUSTAINED_SPEED_MPS": float(
            trial_contract["minimum_sustained_speed_mps"]
        ),
        "MINIMUM_SUSTAINED_SPEED_SEC": float(
            trial_contract["minimum_sustained_speed_sec"]
        ),
        "MAXIMUM_OBSERVED_SPEED_MPS": float(
            contract["maximum_observed_speed_mps"]
        ),
        "MAXIMUM_SPEED_SAMPLE_GAP_SEC": float(
            contract["maximum_speed_sample_gap_sec"]
        ),
        "MAXIMUM_LATERAL_ACCELERATION_LIMIT_MPS2": float(
            trial_contract["maximum_lateral_acceleration_mps2"]
        ),
        "MAXIMUM_LONGITUDINAL_ACCELERATION_MPS2": float(
            parameters["maximum_longitudinal_acceleration_mps2"]
        ),
        "COMMAND_GATE_NOMINAL_LONGITUDINAL_ACCELERATION_MPS2": float(
            gate_contract["longitudinal_acceleration_limit_mps2"]
        ),
        "MAXIMUM_LATERAL_ACCELERATION_MPS2": float(
            parameters["maximum_lateral_acceleration_mps2"]
        ),
        "CONTROLLER_STOP_OFFSET_M": float(
            parameters["controller_stop_offset_m"]
        ),
        "MANEUVER_LOOKAHEAD_M": float(parameters["maneuver_lookahead_m"]),
        "MANEUVER_EXIT_LOOKAHEAD_M": float(
            parameters["maneuver_exit_lookahead_m"]
        ),
        "ROUTE_CURVATURE_LOOKAHEAD_M": float(
            parameters["route_curvature_lookahead_m"]
        ),
        "CURVATURE_SPEED_PREVIEW_M": float(
            parameters["curvature_speed_preview_m"]
        ),
        "MAX_ROUTE_DEVIATION_M": float(parameters["max_route_deviation_m"]),
        "MAX_CANDIDATE_AGE_SEC": float(parameters["max_candidate_age_sec"]),
        "CANDIDATE_TIMEOUT_SEC": float(parameters["candidate_timeout_sec"]),
        "COMFORTABLE_DECELERATION_MPS2": float(
            parameters["comfortable_deceleration_mps2"]
        ),
        "LONGITUDINAL_PID_MAX_OUT_MPS2": float(
            controller_contract["maximum_output_mps2"]
        ),
        "LONGITUDINAL_PID_MAX_P_EFFORT_MPS2": float(
            controller_contract["maximum_proportional_effort_mps2"]
        ),
    }
    for field, expected in runtime_numbers.items():
        _runtime_number(runtime, field, expected)

    manager_path = trial_dir / "vad_route_manager.params.yaml"
    manager_parameters = _ros_parameters(
        manager_path, "/vad_route_manager", "VAD route-manager parameter dump"
    )
    for field, expected in parameters.items():
        if isinstance(expected, str):
            actual = _parameter_value(
                manager_parameters,
                str(field),
                "VAD route-manager parameter dump",
            )
            if actual != expected:
                raise MatrixError(
                    f"VAD route-manager parameter dump.{field} mismatch: "
                    f"expected={expected!r} actual={actual!r}"
                )
        else:
            _parameter_number(
                manager_parameters,
                str(field),
                float(expected),
                "VAD route-manager parameter dump",
            )

    provenance_root = trial_dir / "speed_profile_provenance"
    gate_path = provenance_root / "vehicle_cmd_gate.param.yaml"
    metadata_path = provenance_root / "vehicle_cmd_gate.param.yaml.metadata.json"
    controller_path = provenance_root / "longitudinal_controller.param.yaml"
    controller_metadata_path = provenance_root / (
        "longitudinal_controller.param.yaml.metadata.json"
    )
    sums_path = provenance_root / "SHA256SUMS"
    for path in (
        gate_path,
        metadata_path,
        controller_path,
        controller_metadata_path,
        sums_path,
    ):
        if not path.is_file():
            raise MatrixError(f"speed profile is missing provenance: {path.name}")
    gate_sha256 = sha256_file(gate_path)
    metadata_sha256 = sha256_file(metadata_path)
    controller_sha256 = sha256_file(controller_path)
    controller_metadata_sha256 = sha256_file(controller_metadata_path)
    if gate_sha256 != gate_contract["parameter_sha256"]:
        raise MatrixError("copied gate config differs from the pinned speed contract")
    if metadata_sha256 != gate_contract["metadata_sha256"]:
        raise MatrixError("copied gate metadata differs from the pinned speed contract")
    if runtime.get("VEHICLE_CMD_GATE_PARAM_SHA256") != gate_sha256:
        raise MatrixError("runtime gate SHA256 differs from copied provenance")
    if runtime.get("VEHICLE_CMD_GATE_METADATA_SHA256") != metadata_sha256:
        raise MatrixError("runtime gate metadata SHA256 differs from copied provenance")
    if not runtime.get("VEHICLE_CMD_GATE_PARAM_FILE"):
        raise MatrixError("runtime gate source path is missing")
    if controller_sha256 != controller_contract["parameter_sha256"]:
        raise MatrixError(
            "copied longitudinal-controller config differs from the pinned speed contract"
        )
    if controller_metadata_sha256 != controller_contract["metadata_sha256"]:
        raise MatrixError(
            "copied longitudinal-controller metadata differs from the pinned speed contract"
        )
    if runtime.get("LONGITUDINAL_CONTROLLER_PARAM_SHA256") != controller_sha256:
        raise MatrixError(
            "runtime longitudinal-controller SHA256 differs from copied provenance"
        )
    if (
        runtime.get("LONGITUDINAL_CONTROLLER_METADATA_SHA256")
        != controller_metadata_sha256
    ):
        raise MatrixError(
            "runtime longitudinal-controller metadata SHA256 differs from copied provenance"
        )
    if not runtime.get("LONGITUDINAL_CONTROLLER_PARAM_FILE"):
        raise MatrixError("runtime longitudinal-controller source path is missing")
    sums = _sha256_manifest(sums_path)
    expected_sums = {
        "vehicle_cmd_gate.param.yaml": gate_sha256,
        "vehicle_cmd_gate.param.yaml.metadata.json": metadata_sha256,
        "longitudinal_controller.param.yaml": controller_sha256,
        "longitudinal_controller.param.yaml.metadata.json": (
            controller_metadata_sha256
        ),
    }
    if sums != expected_sums:
        raise MatrixError("speed-profile SHA256SUMS does not match copied provenance")

    metadata = read_object(metadata_path, "vehicle-cmd-gate metadata")
    if (
        metadata.get("schema_version") != 1
        or metadata.get("profile_id") != gate_contract["profile_id"]
        or metadata.get("speed_limit_source") != gate_contract["speed_limit_source"]
        or metadata.get("real_vehicle_ready") is not False
        or not isinstance(metadata.get("source_file"), str)
        or metadata.get("source_sha256") != gate_contract["source_sha256"]
        or metadata.get("source_repository_commit")
        != gate_contract["source_repository_commit"]
    ):
        raise MatrixError("vehicle-cmd-gate metadata violates the speed-profile contract")

    target_speed = float(gate_contract["velocity_limit_mps"])
    longitudinal_limit = float(
        gate_contract["longitudinal_acceleration_limit_mps2"]
    )
    lateral_limit = float(gate_contract["lateral_acceleration_limit_mps2"])
    allowed_overrides = metadata.get("allowed_overrides")
    if not isinstance(allowed_overrides, dict):
        raise MatrixError("vehicle-cmd-gate metadata lacks allowed_overrides")
    expected_override_fields = {
        "nominal.vel_lim",
        "nominal.lon_acc_lim_for_lon_vel",
        "nominal.lat_acc_lim_for_steer_cmd",
        "on_transition.vel_lim",
        "on_transition.lat_acc_lim_for_steer_cmd",
    }
    if set(allowed_overrides) != expected_override_fields:
        raise MatrixError("vehicle-cmd-gate allowed override set is not pinned")
    gate_parameters = _ros_parameters(
        gate_path, "/**", "vehicle-cmd-gate provenance"
    )
    live_gate_parameters = _ros_parameters(
        trial_dir / "vehicle_cmd_gate.params.yaml",
        "/control/vehicle_cmd_gate",
        "live vehicle-cmd-gate parameter dump",
    )
    for field, expected in _flatten_parameters(gate_parameters).items():
        if isinstance(expected, str) and "$(var " in expected:
            continue
        actual = _parameter_value(
            live_gate_parameters, field, "live vehicle-cmd-gate parameter dump"
        )
        if not _parameter_values_match(actual, expected):
            raise MatrixError(
                f"live vehicle-cmd-gate parameter dump.{field} differs from "
                "the pinned provenance"
            )
    for source, label in (
        (gate_parameters, "vehicle-cmd-gate provenance"),
        (live_gate_parameters, "live vehicle-cmd-gate parameter dump"),
    ):
        _parameter_number(source, "nominal.vel_lim", target_speed, label)
        _parameter_number(source, "on_transition.vel_lim", target_speed, label)
        _parameter_number_list(
            source, "nominal.lon_acc_lim_for_lon_vel", longitudinal_limit, label
        )
        _parameter_number_list(
            source, "nominal.lat_acc_lim_for_steer_cmd", lateral_limit, label
        )
        _parameter_number_list(
            source, "on_transition.lat_acc_lim_for_steer_cmd", lateral_limit, label
        )
    for field, expected in (
        ("nominal.vel_lim", target_speed),
        ("on_transition.vel_lim", target_speed),
    ):
        actual = _finite_number(
            allowed_overrides.get(field), f"vehicle-cmd-gate metadata.{field}"
        )
        if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1.0e-9):
            raise MatrixError(f"vehicle-cmd-gate metadata override mismatch: {field}")
    for field in (
        "nominal.lon_acc_lim_for_lon_vel",
        "nominal.lat_acc_lim_for_steer_cmd",
        "on_transition.lat_acc_lim_for_steer_cmd",
    ):
        raw = allowed_overrides.get(field)
        expected = (
            longitudinal_limit
            if field == "nominal.lon_acc_lim_for_lon_vel"
            else lateral_limit
        )
        if not isinstance(raw, list) or not raw or any(
            not math.isclose(
                _finite_number(item, f"vehicle-cmd-gate metadata.{field}"),
                expected,
                rel_tol=0.0,
                abs_tol=1.0e-9,
            )
            for item in raw
        ):
            raise MatrixError(f"vehicle-cmd-gate metadata override mismatch: {field}")

    controller_metadata = read_object(
        controller_metadata_path, "longitudinal-controller metadata"
    )
    if (
        controller_metadata.get("schema_version") != 1
        or controller_metadata.get("profile_id")
        != controller_contract["profile_id"]
        or controller_metadata.get("speed_limit_source")
        != controller_contract["speed_limit_source"]
        or controller_metadata.get("real_vehicle_ready") is not False
        or not isinstance(controller_metadata.get("source_file"), str)
        or controller_metadata.get("source_sha256")
        != controller_contract["source_sha256"]
        or controller_metadata.get("source_repository_commit")
        != controller_contract["source_repository_commit"]
    ):
        raise MatrixError(
            "longitudinal-controller metadata violates the speed-profile contract"
        )
    controller_overrides = controller_metadata.get("allowed_overrides")
    if not isinstance(controller_overrides, dict) or set(
        controller_overrides
    ) != {"max_out", "max_p_effort"}:
        raise MatrixError(
            "longitudinal-controller allowed override set is not pinned"
        )
    maximum_output = float(controller_contract["maximum_output_mps2"])
    maximum_p_effort = float(
        controller_contract["maximum_proportional_effort_mps2"]
    )
    for field, expected in (
        ("max_out", maximum_output),
        ("max_p_effort", maximum_p_effort),
    ):
        actual = _finite_number(
            controller_overrides.get(field),
            f"longitudinal-controller metadata.{field}",
        )
        if not math.isclose(
            actual, expected, rel_tol=0.0, abs_tol=1.0e-9
        ):
            raise MatrixError(
                f"longitudinal-controller metadata override mismatch: {field}"
            )
    metadata_gate_cap = _finite_number(
        controller_metadata.get(
            "command_gate_longitudinal_acceleration_cap_mps2"
        ),
        "longitudinal-controller metadata.command-gate cap",
    )
    expected_gate_cap = float(
        controller_contract[
            "command_gate_longitudinal_acceleration_cap_mps2"
        ]
    )
    if (
        not math.isclose(
            metadata_gate_cap,
            expected_gate_cap,
            rel_tol=0.0,
            abs_tol=1.0e-9,
        )
        or not math.isclose(
            expected_gate_cap,
            longitudinal_limit,
            rel_tol=0.0,
            abs_tol=1.0e-9,
        )
    ):
        raise MatrixError(
            "longitudinal-controller and command-gate acceleration caps differ"
        )

    controller_parameters = _ros_parameters(
        controller_path, "/**", "longitudinal-controller provenance"
    )
    live_controller_parameters = _ros_parameters(
        trial_dir / "controller.params.yaml",
        "/control/trajectory_follower/controller_node_exe",
        "live longitudinal-controller parameter dump",
    )
    for field, expected in _flatten_parameters(controller_parameters).items():
        actual = _parameter_value(
            live_controller_parameters,
            field,
            "live longitudinal-controller parameter dump",
        )
        if not _parameter_values_match(actual, expected):
            raise MatrixError(
                f"live longitudinal-controller parameter dump.{field} differs "
                "from the pinned provenance"
            )
    for source, label in (
        (controller_parameters, "longitudinal-controller provenance"),
        (
            live_controller_parameters,
            "live longitudinal-controller parameter dump",
        ),
    ):
        _parameter_number(source, "max_out", maximum_output, label)
        _parameter_number(source, "max_p_effort", maximum_p_effort, label)

    exposure = result.get("speed_exposure")
    limits = result.get("limits")
    metrics = result.get("metrics")
    if not isinstance(exposure, dict) or exposure.get("status") != "PASS":
        raise MatrixError("result has no passing speed_exposure contract")
    if not isinstance(limits, dict) or not isinstance(metrics, dict):
        raise MatrixError("result lacks speed-contract limits or metrics")
    expected_profile_context = {
        "longitudinal_velocity_source": parameters[
            "longitudinal_velocity_source"
        ],
        "vad_velocity_evaluated": contract["vad_velocity_evaluated"],
        "vad_geometry_evaluated": contract["vad_geometry_evaluated"],
    }
    if result.get("profile_context") != expected_profile_context:
        raise MatrixError("result profile_context violates the v2 speed contract")
    for field, expected in expected_profile_context.items():
        if exposure.get(field) != expected:
            raise MatrixError(
                f"result.speed_exposure.{field} violates the v2 speed contract"
            )
    declared_limits = {
        "minimum_sustained_speed_mps": float(
            trial_contract["minimum_sustained_speed_mps"]
        ),
        "minimum_sustained_speed_sec": float(
            trial_contract["minimum_sustained_speed_sec"]
        ),
        "maximum_observed_speed_mps": float(
            contract["maximum_observed_speed_mps"]
        ),
        "maximum_lateral_acceleration_mps2": float(
            trial_contract["maximum_lateral_acceleration_mps2"]
        ),
        "maximum_speed_sample_gap_sec": float(
            contract["maximum_speed_sample_gap_sec"]
        ),
    }
    for field, expected in declared_limits.items():
        _require_exact_number(limits, field, expected, "result.limits")
    exposure_fields = {
        "minimum_sustained_speed_mps": declared_limits[
            "minimum_sustained_speed_mps"
        ],
        "minimum_sustained_speed_sec": declared_limits[
            "minimum_sustained_speed_sec"
        ],
        "maximum_observed_speed_limit_mps": declared_limits[
            "maximum_observed_speed_mps"
        ],
        "maximum_lateral_acceleration_limit_mps2": declared_limits[
            "maximum_lateral_acceleration_mps2"
        ],
    }
    for field, expected in exposure_fields.items():
        _require_exact_number(exposure, field, expected, "result.speed_exposure")

    maximum_speed = _finite_number(
        exposure.get("maximum_observed_speed_mps"),
        "result.speed_exposure.maximum_observed_speed_mps",
    )
    maximum_lateral = _finite_number(
        exposure.get("maximum_lateral_acceleration_mps2"),
        "result.speed_exposure.maximum_lateral_acceleration_mps2",
    )
    sustained_duration = _finite_number(
        exposure.get("maximum_sustained_speed_duration_sec"),
        "result.speed_exposure.maximum_sustained_speed_duration_sec",
    )
    maximum_speed_sample_gap = _finite_number(
        exposure.get("maximum_speed_sample_gap_sec"),
        "result.speed_exposure.maximum_speed_sample_gap_sec",
    )
    metric_pairs = {
        "maximum_observed_speed_mps": maximum_speed,
        "maximum_lateral_acceleration_mps2": maximum_lateral,
        "maximum_sustained_speed_duration_sec": sustained_duration,
        "maximum_speed_sample_gap_sec": maximum_speed_sample_gap,
    }
    for field, expected in metric_pairs.items():
        actual = _finite_number(metrics.get(field), f"result.metrics.{field}")
        if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1.0e-9):
            raise MatrixError(
                f"result speed exposure differs from its metric {field}"
            )
    if exposure.get("speed_by_command") != metrics.get("speed_by_command"):
        raise MatrixError("result speed_by_command evidence is internally inconsistent")
    if sustained_duration < 0.0 or maximum_speed_sample_gap < 0.0:
        raise MatrixError("result speed timing metrics must be non-negative")
    if maximum_speed_sample_gap > declared_limits[
        "maximum_speed_sample_gap_sec"
    ] + 1.0e-9:
        raise MatrixError("result speed sample gap violates the 0.25 s contract")
    if maximum_speed < 0.0 or maximum_speed > declared_limits[
        "maximum_observed_speed_mps"
    ] + 1.0e-9:
        raise MatrixError("result maximum observed speed violates the 30-kph contract")
    if maximum_lateral < 0.0 or maximum_lateral > declared_limits[
        "maximum_lateral_acceleration_mps2"
    ] + 1.0e-9:
        raise MatrixError("result maximum lateral acceleration violates the contract")

    evidence: dict[str, Any] = {
        "status": "PASS",
        "profile_id": contract["profile_id"],
        "exposure_mode": trial_contract["exposure_mode"],
        "maximum_observed_speed_mps": maximum_speed,
        "maximum_observed_speed_limit_mps": declared_limits[
            "maximum_observed_speed_mps"
        ],
        "maximum_lateral_acceleration_mps2": maximum_lateral,
        "maximum_lateral_acceleration_limit_mps2": declared_limits[
            "maximum_lateral_acceleration_mps2"
        ],
        "maximum_speed_sample_gap_sec": maximum_speed_sample_gap,
        "maximum_speed_sample_gap_limit_sec": declared_limits[
            "maximum_speed_sample_gap_sec"
        ],
        "route_manager_parameter_dump": str(manager_path),
        "gate_provenance_sha256": gate_sha256,
        "gate_metadata_sha256": metadata_sha256,
        "longitudinal_controller_provenance_sha256": controller_sha256,
        "longitudinal_controller_metadata_sha256": (
            controller_metadata_sha256
        ),
        "longitudinal_controller_parameter_dump": str(
            trial_dir / "controller.params.yaml"
        ),
        "runtime_env_sha256": sha256_file(trial_dir / "runtime.env"),
        "route_manager_parameter_dump_sha256": sha256_file(manager_path),
        "vehicle_cmd_gate_parameter_dump_sha256": sha256_file(
            trial_dir / "vehicle_cmd_gate.params.yaml"
        ),
        "longitudinal_controller_parameter_dump_sha256": sha256_file(
            trial_dir / "controller.params.yaml"
        ),
    }
    if trial_id == "straight":
        if sustained_duration + 1.0e-9 < declared_limits[
            "minimum_sustained_speed_sec"
        ]:
            raise MatrixError("straight trial did not sustain >=7.5 m/s for 1.0 s")
        evidence.update(
            {
                "minimum_sustained_speed_mps": declared_limits[
                    "minimum_sustained_speed_mps"
                ],
                "minimum_sustained_speed_sec": declared_limits[
                    "minimum_sustained_speed_sec"
                ],
                "maximum_sustained_speed_duration_sec": sustained_duration,
            }
        )
    else:
        expected_command = VAD_COMMANDS[scenario.upper()]
        commands_seen = metrics.get("commands_seen")
        speed_by_command = exposure.get("speed_by_command")
        if (
            not isinstance(commands_seen, list)
            or not all(
                isinstance(command, int) and not isinstance(command, bool)
                for command in commands_seen
            )
            or expected_command not in commands_seen
            or not isinstance(speed_by_command, dict)
            or not isinstance(speed_by_command.get(str(expected_command)), dict)
        ):
            raise MatrixError(
                f"turn trial has no observed selected VAD command {expected_command}"
            )
        p95_turn_lateral = _finite_number(
            exposure.get("p95_turn_lateral_acceleration_mps2"),
            "result.speed_exposure.p95_turn_lateral_acceleration_mps2",
        )
        actual_path = result.get("actual_path")
        if not isinstance(actual_path, list):
            raise MatrixError("turn result has no actual_path for lateral p95")
        selected_lateral_samples = [
            float(sample["lateral_acceleration_mps2"])
            for sample in actual_path
            if isinstance(sample, dict)
            and sample.get("command") == expected_command
            and not isinstance(sample.get("lateral_acceleration_mps2"), bool)
            and isinstance(sample.get("lateral_acceleration_mps2"), (int, float))
            and math.isfinite(float(sample["lateral_acceleration_mps2"]))
        ]
        actual_lateral_samples = [
            float(sample["lateral_acceleration_mps2"])
            for sample in actual_path
            if isinstance(sample, dict)
            and sample.get("command") in (0, 1)
            and not isinstance(sample.get("lateral_acceleration_mps2"), bool)
            and isinstance(sample.get("lateral_acceleration_mps2"), (int, float))
            and math.isfinite(float(sample["lateral_acceleration_mps2"]))
        ]
        if not actual_lateral_samples:
            raise MatrixError("turn actual_path has no finite lateral-acceleration samples")
        if not selected_lateral_samples:
            raise MatrixError("turn actual_path has no selected-command lateral samples")
        p95_by_command = exposure.get("p95_lateral_acceleration_mps2_by_command")
        if not isinstance(p95_by_command, dict):
            raise MatrixError("turn result lacks command-specific lateral p95 evidence")
        reported_selected_p95 = _finite_number(
            p95_by_command.get(str(expected_command)),
            "result.speed_exposure.p95_lateral_acceleration_mps2_by_command",
        )
        recomputed_selected_p95 = _linear_percentile(
            selected_lateral_samples, 95.0
        )
        if not math.isclose(
            reported_selected_p95,
            recomputed_selected_p95,
            rel_tol=0.0,
            abs_tol=1.0e-9,
        ):
            raise MatrixError(
                "selected-command lateral p95 differs from actual_path evidence"
            )
        recomputed_p95 = _linear_percentile(actual_lateral_samples, 95.0)
        if not math.isclose(
            p95_turn_lateral, recomputed_p95, rel_tol=0.0, abs_tol=1.0e-9
        ):
            raise MatrixError(
                "turn-command lateral p95 differs from actual_path evidence"
            )
        p95_limit = float(
            trial_contract["maximum_lateral_acceleration_p95_mps2"]
        )
        if p95_turn_lateral < 0.0 or p95_turn_lateral > p95_limit + 1.0e-9:
            raise MatrixError("turn p95 lateral acceleration violates the contract")
        evidence.update(
            {
                "selected_turn_command": expected_command,
                "commands_seen": commands_seen,
                "p95_selected_turn_command_lateral_acceleration_mps2": (
                    reported_selected_p95
                ),
                "p95_turn_lateral_acceleration_mps2": p95_turn_lateral,
                "p95_lateral_acceleration_limit_mps2": p95_limit,
            }
        )
    evidence.update(
        _speed_profile_artifact_evidence(
            trial_dir, result, trial_id, scenario
        )
    )
    return evidence


def _image_size(path: Path, expected_format: str) -> tuple[int, int]:
    try:
        with Image.open(path) as image:
            if image.format != expected_format:
                raise MatrixError(f"unexpected image format for {path}: {image.format}")
            size = image.size
            image.verify()
    except OSError as error:
        raise MatrixError(f"cannot validate image {path}: {error}") from error
    return size


def _carla_lifecycle_evidence(
    trial_dir: Path,
    runtime: Mapping[str, str],
    profile: Mapping[str, Any],
    entry: Mapping[str, Any],
    map_id: str,
    trial_id: str,
) -> dict[str, Any]:
    lifecycle = "cold_start_owned_process_group_per_trial"
    if (
        profile.get("map_lifecycle") != lifecycle
        or runtime.get("CARLA_LIFECYCLE") != lifecycle
        or runtime.get("CARLA_MATRIX_OWNED") != "true"
    ):
        raise MatrixError("trial lacks the owned per-trial CARLA lifecycle")
    expected_generation = f"{map_id}_{trial_id}_{trial_dir.name}"
    generation = runtime.get("CARLA_GENERATION_ID")
    expected_map = str(entry["canonical_name"])
    if (
        generation != expected_generation
        or runtime.get("CARLA_EXPECTED_MAP") != expected_map
    ):
        raise MatrixError("trial CARLA generation/map identity is not pinned")
    try:
        owner_pid = int(runtime["CARLA_OWNER_PID"])
        owner_pgid = int(runtime["CARLA_OWNER_PGID"])
        port = int(runtime["CARLA_PORT"])
    except (KeyError, TypeError, ValueError) as error:
        raise MatrixError("trial CARLA owner/port provenance is invalid") from error
    if owner_pid <= 1 or owner_pgid != owner_pid or not 0 < port <= 65535:
        raise MatrixError("trial CARLA owner/PGID/port provenance is invalid")
    expected_log = (trial_dir / "carla_server.log").resolve()
    recorded_log = Path(runtime.get("CARLA_SERVER_LOG", "")).expanduser().resolve()
    if recorded_log != expected_log or not expected_log.is_file():
        raise MatrixError("trial CARLA generation log is missing or unbound")

    stage_specs = {
        "preflight": ("carla_preflight_health.json", "running", "trial_preflight"),
        "completion": (
            "carla_completion_health.json",
            "running",
            "trial_completion",
        ),
        "cleanup": ("carla_cleanup_health.json", "stopped", "trial_cleanup"),
    }
    health: dict[str, dict[str, Any]] = {}
    timestamps: dict[str, datetime] = {}
    for label, (filename, mode, stage) in stage_specs.items():
        path = trial_dir / filename
        payload = read_object(path, f"CARLA {label} health")
        if (
            payload.get("schema_version") != 1
            or payload.get("status") != "PASS"
            or payload.get("mode") != mode
            or payload.get("stage") != stage
            or payload.get("generation_id") != generation
            or payload.get("expected_map") != expected_map
            or payload.get("owner_pid") != owner_pid
            or payload.get("owner_pgid") != owner_pgid
            or payload.get("host") != runtime.get("CARLA_HOST")
            or payload.get("port") != port
            or payload.get("read_only") is not True
            or payload.get("error") is not None
        ):
            raise MatrixError(f"CARLA {label} health contract mismatch")
        duration = _finite_number(
            payload.get("probe_wall_seconds"),
            f"CARLA {label} health probe_wall_seconds",
        )
        timeout = _finite_number(
            payload.get("timeout_seconds"),
            f"CARLA {label} health timeout_seconds",
        )
        if duration < 0.0 or timeout <= 0.0 or duration > timeout + 1.0:
            raise MatrixError(f"CARLA {label} health timing is invalid")
        try:
            timestamps[label] = datetime.fromisoformat(
                str(payload["checked_at"]).replace("Z", "+00:00")
            )
        except (KeyError, TypeError, ValueError) as error:
            raise MatrixError(
                f"CARLA {label} health timestamp is invalid"
            ) from error
        log_record = payload.get("server_log")
        if (
            not isinstance(log_record, dict)
            or Path(str(log_record.get("path", ""))).expanduser().resolve()
            != expected_log
            or not re.fullmatch(r"[0-9a-f]{64}", str(log_record.get("sha256", "")))
            or isinstance(log_record.get("size_bytes"), bool)
            or not isinstance(log_record.get("size_bytes"), int)
            or log_record["size_bytes"] < 0
        ):
            raise MatrixError(f"CARLA {label} generation-log evidence is invalid")
        if mode == "running":
            if (
                payload.get("active_map_basename") != expected_map
                or payload.get("rpc_sequence")
                != ["get_world", "world.get_map", "world.get_snapshot"]
                or isinstance(payload.get("snapshot_frame"), bool)
                or not isinstance(payload.get("snapshot_frame"), int)
                or payload["snapshot_frame"] < 0
                or payload.get("owner_process_state") in (None, "Z")
            ):
                raise MatrixError(f"CARLA {label} RPC evidence is invalid")
            _finite_number(
                payload.get("snapshot_elapsed_seconds"),
                f"CARLA {label} snapshot elapsed seconds",
            )
        elif (
            payload.get("port_released") is not True
            or payload.get("owner_process_state") is not None
            or payload.get("rpc_sequence") != []
        ):
            raise MatrixError("CARLA cleanup did not prove process/port release")
        health[label] = payload
    if not timestamps["preflight"] < timestamps["completion"] < timestamps["cleanup"]:
        raise MatrixError("CARLA lifecycle timestamps are not strictly ordered")
    cleanup_log = health["cleanup"]["server_log"]
    if (
        cleanup_log["sha256"] != sha256_file(expected_log)
        or cleanup_log["size_bytes"] != expected_log.stat().st_size
    ):
        raise MatrixError("final CARLA generation log differs from cleanup evidence")
    return {
        "schema_version": 1,
        "status": "PASS",
        "lifecycle": lifecycle,
        "generation_id": generation,
        "expected_map": expected_map,
        "owner_pid": owner_pid,
        "owner_pgid": owner_pgid,
        "server_log": {
            "path": str(expected_log),
            "size_bytes": expected_log.stat().st_size,
            "sha256": sha256_file(expected_log),
        },
        "preflight_health_sha256": sha256_file(
            trial_dir / "carla_preflight_health.json"
        ),
        "completion_health_sha256": sha256_file(
            trial_dir / "carla_completion_health.json"
        ),
        "cleanup_health_sha256": sha256_file(
            trial_dir / "carla_cleanup_health.json"
        ),
        "post_completion_exit_policy": (
            "completion RPC PASS preserves a completed drive; the next trial "
            "always receives a new cold-start generation"
        ),
    }


def validate_trial(
    output_root: Path, map_id: str, trial_id: str, trial_dir: Path
) -> dict[str, Any]:
    if trial_id not in TRIAL_IDS:
        raise MatrixError(f"unknown trial id: {trial_id}")
    output_root = output_root.expanduser().resolve()
    trial_dir = _safe_inside(output_root, trial_dir, "trial directory")
    plan = _load_verified_campaign_plan(
        output_root, f"map {map_id} trial {trial_id} validation"
    )
    entry = _map_entry(plan, map_id)
    profile = plan.get("runtime_profile")
    if not isinstance(profile, dict):
        raise MatrixError("matrix plan has no runtime profile")
    route_matrix = read_object(
        output_root / "maps" / map_id / "route_matrix.json", "route matrix"
    )
    route_entry = next(
        (item for item in route_matrix.get("trials", []) if item.get("trial_id") == trial_id),
        None,
    )
    if route_entry is None:
        raise MatrixError(f"route matrix has no {trial_id} trial")
    scenario = str(route_entry["catalog_scenario"])
    required_files = [
        "result.json",
        "runtime.env",
        "source_route.json",
        "map_bundle.json",
        "diagnosis.json",
        "path_vs_control.png",
        "steering_tracking.png",
        "route_result.png",
        "turn_path_control.gif",
        "latency/e2e_latency.json",
        "autoware_rviz_fullscreen.png",
        "autoware_rviz_drive.gif",
        "desktop_capture.json",
        "carla_server.log",
        "carla_preflight_health.json",
        "carla_completion_health.json",
        "carla_cleanup_health.json",
    ]
    if profile.get("speed_contract") is not None:
        required_files.extend(
            (
                "bag/metadata.yaml",
                "speed_profile.json",
                "speed_profile.png",
                "vad_route_manager.params.yaml",
                "vehicle_cmd_gate.params.yaml",
                "controller.params.yaml",
                "autoware_rviz_candidate.png",
                "autoware_rviz_capture.mkv",
                "rviz_capture_provenance/autoware_vad_carla.rviz",
                "rviz_capture_provenance/SHA256SUMS",
            )
        )
    for relative in required_files:
        if not (trial_dir / relative).is_file():
            raise MatrixError(f"trial is missing required evidence: {relative}")
    result = read_object(trial_dir / "result.json", "route result")
    assessment = result.get("assessment")
    final = result.get("final")
    if (
        result.get("success") is not True
        or result.get("execution_mode") != "full_stack"
        or not isinstance(assessment, dict)
        or assessment.get("planning_architecture") != "vad_route_manager_hybrid"
        or assessment.get("route_completion") != "PASS"
        or not isinstance(final, dict)
        or final.get("goal_reached") is not True
        or final.get("route_status") != "goal_reached"
    ):
        raise MatrixError("trial is not a successful full-stack Autoware VAD result")
    runtime = _runtime_env(trial_dir / "runtime.env")
    if (
        runtime.get("RECOMMENDED") != "true"
        or runtime.get("VISUALIZE") != "true"
        or runtime.get("CAPTURE_DESKTOP") != "true"
    ):
        raise MatrixError("trial did not use the recommended visualized profile")
    carla_lifecycle = _carla_lifecycle_evidence(
        trial_dir, runtime, profile, entry, map_id, trial_id
    )
    source_route = read_object(trial_dir / "source_route.json", "trial source route")
    route_analysis = _validate_route_payload(
        source_route, entry["canonical_name"], scenario
    )
    if sha256_file(trial_dir / "source_route.json") != route_entry["route_sha256"]:
        raise MatrixError("trial source route differs from the selected route contract")
    route_preflight = _validate_route_matrix_provenance(
        output_root,
        plan,
        entry,
        route_matrix,
        route_entry,
        route_analysis,
        trial_id,
    )
    copied_bundle = read_object(trial_dir / "map_bundle.json", "trial map bundle")
    admitted_bundle = entry["full_map_bundle"]
    if sha256_file(trial_dir / "map_bundle.json") != admitted_bundle["metadata_sha256"]:
        raise MatrixError("trial map bundle differs from the admitted bundle")
    if copied_bundle.get("canonical_carla_map") != entry["load_name"]:
        raise MatrixError("trial map bundle canonical map mismatch")
    diagnosis = read_object(trial_dir / "diagnosis.json", "diagnosis")
    diagnosis_inputs = diagnosis.get("inputs")
    if not isinstance(diagnosis_inputs, dict) or diagnosis_inputs.get("scenario") != scenario:
        raise MatrixError("route diagnosis does not identify the selected scenario")
    latency = read_object(
        trial_dir / "latency/e2e_latency.json", "latency/e2e_latency"
    )
    selected_topics = latency.get("selected_topics")
    event_rates = latency.get("event_rates")
    candidate_topic = (
        selected_topics.get("vad_output")
        if isinstance(selected_topics, dict)
        else None
    )
    candidate_rate = (
        event_rates.get(candidate_topic)
        if isinstance(event_rates, dict) and isinstance(candidate_topic, str)
        else None
    )
    if (
        str(candidate_topic or "").lstrip("/")
        != "planning/vad/candidate_trajectories"
        or not isinstance(candidate_rate, dict)
        or int(candidate_rate.get("count", 0)) <= 0
    ):
        raise MatrixError("recorded route analysis contains no VAD candidate output")
    png_size = _image_size(trial_dir / "autoware_rviz_fullscreen.png", "PNG")
    gif_size = _image_size(trial_dir / "autoware_rviz_drive.gif", "GIF")
    if gif_size[0] != 960:
        raise MatrixError(f"Autoware/RViz GIF width must be 960, got {gif_size}")
    desktop = read_object(trial_dir / "desktop_capture.json", "desktop capture")
    if (
        desktop.get("candidate_observed") is not True
        or desktop.get("capture_started_after_candidate") is not True
        or str(desktop.get("candidate_topic", "")).lstrip("/")
        != "planning/vad/candidate_trajectories"
        or desktop.get("source_dimensions") != list(png_size)
        or desktop.get("png_dimensions") != list(png_size)
        or desktop.get("gif_dimensions") != list(gif_size)
    ):
        raise MatrixError("desktop capture is not proven to start after a VAD candidate")
    speed_contract = _speed_contract_evidence(
        trial_dir, runtime, result, profile, trial_id, scenario
    )
    visual_evidence = (
        _speed_visual_evidence_binding(trial_dir, desktop, png_size, gif_size)
        if profile.get("speed_contract") is not None
        else None
    )
    validation = {
        "schema_version": 1,
        "status": "PASS",
        "validated_at": utc_now(),
        "matrix_id": plan["matrix_id"],
        "admission_contract_sha256": plan["admission_contract_sha256"],
        "campaign_execution_contract_sha256": plan[
            "campaign_execution_contract_sha256"
        ],
        "map_id": map_id,
        "trial_id": trial_id,
        "catalog_scenario": scenario,
        "turn_direction": route_entry.get("turn_direction"),
        "trial_directory": str(trial_dir),
        "route_analysis": route_analysis,
        "result": {
            "success": True,
            "execution_mode": "full_stack",
            "planning_architecture": "vad_route_manager_hybrid",
            "route_completion": "PASS",
            "goal_reached": True,
        },
        "runtime_profile_selector": plan.get(
            "runtime_profile_selector", "recommended"
        ),
        "runtime_profile": profile,
        "speed_contract": speed_contract,
        "campaign_route_map_preflight": route_preflight,
        "carla_lifecycle": carla_lifecycle,
        "desktop_capture": desktop,
        "visual_evidence": visual_evidence,
    }
    _verify_campaign_plan(
        plan, f"map {map_id} trial {trial_id} validation completion"
    )
    atomic_json(trial_dir / "matrix_validation.json", validation)
    return validation


def _verify_recorded_trial_pass(
    output_root: Path,
    map_id: str,
    trial_id: str,
    trial: Mapping[str, Any],
) -> None:
    attempt_value = trial.get("attempt_directory")
    validation_value = trial.get("validation")
    if not isinstance(attempt_value, str) or not attempt_value:
        raise MatrixError(
            f"map PASS requires a recorded {trial_id} attempt directory"
        )
    if not isinstance(validation_value, str) or not validation_value:
        raise MatrixError(
            f"map PASS requires a recorded {trial_id} validation artifact"
        )
    attempt_path = _safe_inside(
        output_root, Path(attempt_value), f"{trial_id} attempt directory"
    )
    validation_path = _safe_inside(
        output_root, Path(validation_value), f"{trial_id} validation path"
    )
    expected_validation_path = (attempt_path / "matrix_validation.json").resolve()
    if validation_path != expected_validation_path:
        raise MatrixError(
            f"map PASS {trial_id} validation is not bound to its attempt"
        )
    fresh = validate_trial(output_root, map_id, trial_id, attempt_path)
    if (
        fresh.get("status") != "PASS"
        or fresh.get("map_id") != map_id
        or fresh.get("trial_id") != trial_id
        or not isinstance(fresh.get("admission_contract_sha256"), str)
        or not isinstance(
            fresh.get("campaign_execution_contract_sha256"), str
        )
        or Path(str(fresh.get("trial_directory", ""))).resolve() != attempt_path
        or not validation_path.is_file()
    ):
        raise MatrixError(
            f"map PASS requires a freshly validated {trial_id} PASS trial"
        )


def _verify_both_recorded_trial_passes(
    output_root: Path, map_id: str, status: Mapping[str, Any]
) -> None:
    trials = status.get("trials")
    if not isinstance(trials, dict):
        raise MatrixError("map PASS requires straight and turn trial records")
    states = {
        trial_id: (
            trials.get(trial_id, {}).get("status")
            if isinstance(trials.get(trial_id), dict)
            else None
        )
        for trial_id in TRIAL_IDS
    }
    if any(states[trial_id] != "PASS" for trial_id in TRIAL_IDS):
        raise MatrixError(
            "map PASS requires both straight and turn trial status PASS; "
            f"states={states}"
        )
    for trial_id in TRIAL_IDS:
        _verify_recorded_trial_pass(
            output_root, map_id, trial_id, trials[trial_id]
        )


def update_status(
    output_root: Path,
    map_id: str,
    status_value: str | None,
    stage: str | None,
    reason: str | None,
    trial_id: str | None,
    trial_status: str | None,
    attempt_dir: Path | None,
    validation_path: Path | None,
) -> dict[str, Any]:
    output_root = output_root.expanduser().resolve()
    plan = _load_verified_campaign_plan(output_root, f"map {map_id} status update")
    entry = _map_entry(plan, map_id)
    status_path = _status_path(output_root, map_id)
    value = read_object(status_path, f"{map_id} status")
    if (
        entry.get("runnable") is not True
        or value.get("runnable") is not True
        or value.get("matrix_id") != plan.get("matrix_id")
        or value.get("map_id") != map_id
    ):
        raise MatrixError(f"cannot update blocked map {map_id}")
    if trial_id is not None:
        if trial_id not in TRIAL_IDS or trial_status not in {
            "PENDING",
            "RUNNING",
            "PASS",
            "FAILED",
        }:
            raise MatrixError("invalid trial status update")
        trial = value["trials"][trial_id]
        trial["status"] = trial_status
        trial["reason"] = reason
        trial["attempt_directory"] = (
            str(_safe_inside(output_root, attempt_dir, "attempt directory"))
            if attempt_dir is not None
            else trial.get("attempt_directory")
        )
        trial["validation"] = (
            str(_safe_inside(output_root, validation_path, "validation path"))
            if validation_path is not None
            else trial.get("validation")
        )
        states = [value["trials"][name]["status"] for name in TRIAL_IDS]
        if trial_status == "PASS" and not all(
            state == "PASS" for state in states
        ):
            _verify_recorded_trial_pass(
                output_root, map_id, trial_id, trial
            )
        if all(state == "PASS" for state in states):
            _verify_both_recorded_trial_passes(output_root, map_id, value)
            value["status"] = "PASS"
            value["stage"] = "complete"
            value["reason"] = "Straight and turn full-stack trials both passed."
        elif "FAILED" in states:
            value["status"] = "FAILED"
            value["stage"] = stage or f"{trial_id}_failed"
            value["reason"] = reason or f"{trial_id} trial failed"
        else:
            value["status"] = "RUNNING"
            value["stage"] = stage or f"{trial_id}_{trial_status.lower()}"
            value["reason"] = reason
    else:
        if status_value not in {"PENDING", "RUNNING", "PASS", "FAILED"}:
            raise MatrixError("invalid map status update")
        if status_value == "PASS":
            _verify_both_recorded_trial_passes(output_root, map_id, value)
            stage = "complete"
            reason = "Straight and turn full-stack trials both passed."
        elif status_value == "FAILED":
            map_reason = reason or (
                "Map-level prerequisite failed at "
                f"{stage or 'an unspecified stage'}."
            )
            for name in TRIAL_IDS:
                trial = value["trials"][name]
                if trial.get("status") not in {"PENDING", "RUNNING"}:
                    continue
                trial["status"] = "FAILED"
                trial["reason"] = (
                    "Not executed because map-level prerequisite failed: "
                    f"{map_reason}"
                )
                trial["validation"] = None
            reason = map_reason
        value["status"] = status_value
        value["stage"] = stage
        value["reason"] = reason
    value["updated_at"] = utc_now()
    _verify_campaign_plan(plan, f"map {map_id} status-update completion")
    atomic_json(status_path, value)
    summarize(output_root)
    return value


def summarize(output_root: Path) -> dict[str, Any]:
    output_root = output_root.expanduser().resolve()
    plan = _load_verified_campaign_plan(output_root, "matrix summarization")
    maps = []
    for entry in plan["maps"]:
        status = read_object(_status_path(output_root, entry["map_id"]), "map status")
        maps.append(status)
    counts = Counter(item["status"] for item in maps)
    runnable = [item for item in maps if item["runnable"]]
    if any(item["status"] == "FAILED" for item in runnable):
        overall = "FAILED"
    elif runnable and all(item["status"] == "PASS" for item in runnable):
        overall = "COMPLETE"
    else:
        overall = "INCOMPLETE"
    turn_selection_policy = _plan_turn_route_selection_policy(plan)
    turn_route_selection_plans: list[dict[str, Any]] = []
    for entry in (plan["maps"] if turn_selection_policy is not None else []):
        if entry.get("runnable") is not True:
            continue
        map_id = str(entry["map_id"])
        route_matrix_path = output_root / "maps" / map_id / "route_matrix.json"
        if not route_matrix_path.is_file():
            turn_route_selection_plans.append(
                {
                    "map_id": map_id,
                    "status": "NOT_SELECTED",
                    "route_matrix_path": None,
                    "route_matrix_sha256": None,
                    "candidate_set_sha256": None,
                    "selected_identity": None,
                    "selected_ranking_vector": None,
                }
            )
            continue
        route_matrix = read_object(route_matrix_path, f"{map_id} route matrix")
        trials = route_matrix.get("trials")
        turn = next(
            (
                item
                for item in trials
                if isinstance(item, dict) and item.get("trial_id") == "turn"
            ),
            None,
        ) if isinstance(trials, list) else None
        audit = turn.get("turn_route_selection_audit") if isinstance(turn, dict) else None
        if turn_selection_policy is not None:
            if (
                route_matrix.get("admission_contract_sha256")
                != plan.get("admission_contract_sha256")
                or route_matrix.get("turn_route_selection_policy")
                != turn_selection_policy
                or route_matrix.get("turn_route_selection_policy_sha256")
                != sha256_json(turn_selection_policy)
                or not isinstance(audit, dict)
                or audit.get("policy") != turn_selection_policy
                or audit.get("policy_sha256")
                != sha256_json(turn_selection_policy)
                or audit.get("selected_ranking_vector_sha256")
                != sha256_json(audit.get("selected_ranking_vector"))
                or not re.fullmatch(
                    r"[0-9a-f]{64}", str(audit.get("candidate_set_sha256", ""))
                )
            ):
                raise MatrixError(
                    f"{map_id} aggregate turn-route selection plan is invalid"
                )
            turn_route_selection_plans.append(
                {
                    "map_id": map_id,
                    "status": "SELECTED",
                    "route_matrix_path": str(route_matrix_path),
                    "route_matrix_sha256": sha256_file(route_matrix_path),
                    "candidate_count": audit["candidate_count"],
                    "physical_pass_candidate_count": audit[
                        "physical_pass_candidate_count"
                    ],
                    "physical_rejected_candidate_count": audit[
                        "physical_rejected_candidate_count"
                    ],
                    "candidate_set_sha256": audit["candidate_set_sha256"],
                    "selected_identity": audit["selected_identity"],
                    "selected_ranking_vector": audit[
                        "selected_ranking_vector"
                    ],
                    "selected_ranking_vector_sha256": audit[
                        "selected_ranking_vector_sha256"
                    ],
                    "map_preflight_fallback_allowed": audit[
                        "map_preflight_fallback_allowed"
                    ],
                    "candidate_pair_reindexing_allowed": audit[
                        "candidate_pair_reindexing_allowed"
                    ],
                    "candidate_generation_quota_modified": audit[
                        "candidate_generation_quota_modified"
                    ],
                }
            )
    aggregate = {
        "schema_version": 1,
        "matrix_id": plan["matrix_id"],
        "generated_at": utc_now(),
        "status": overall,
        "admission_contract_sha256": plan["admission_contract_sha256"],
        "campaign_execution_contract_sha256": plan[
            "campaign_execution_contract_sha256"
        ],
        "canonical_map_count": len(maps),
        "runnable_map_count": len(runnable),
        "runnable_pass_count": sum(item["status"] == "PASS" for item in runnable),
        "blocked_map_count": sum(item["status"] == "BLOCKED" for item in maps),
        "status_counts": dict(sorted(counts.items())),
        "runtime_profile_selector": plan.get(
            "runtime_profile_selector", "recommended"
        ),
        "runtime_profile": plan["runtime_profile"],
        "route_contract": plan["route_contract"],
        "route_generation_contracts": plan["route_generation_contracts"],
        "turn_route_selection_policy": turn_selection_policy,
        "turn_route_selection_policy_sha256": (
            sha256_json(turn_selection_policy)
            if turn_selection_policy is not None
            else None
        ),
        "turn_route_selection_plans": turn_route_selection_plans,
        "maps": maps,
    }
    atomic_json(output_root / "aggregate.json", aggregate)
    wrapper_options = plan["runtime_profile"].get("wrapper_options", [])
    rendered_options = " ".join(f"`{item}`" for item in wrapper_options)
    lines = [
        "# Autoware VAD straight/turn town matrix",
        "",
        f"Overall: **{overall}** — runnable PASS "
        f"**{aggregate['runnable_pass_count']}/{len(runnable)}**, "
        f"blocked **{aggregate['blocked_map_count']}**.",
        "",
        "All runnable trials use the selected fixed "
        f"`{aggregate['runtime_profile_selector']}` profile ({rendered_options}). "
        "CARLA is cold-started per map and `client.load_world` is forbidden.",
        "",
        "| Map | Admission | Straight | Turn | Detail |",
        "|---|---|---|---|---|",
    ]
    for item in maps:
        trials = item["trials"]
        reason = str(item.get("reason") or item.get("block_code") or "—").replace("|", "\\|")
        lines.append(
            f"| `{item['map_id']}` | **{item['status']}** | "
            f"{trials['straight']['status']} | {trials['turn']['status']} | {reason} |"
        )
    lines.extend(
        [
            "",
            "A BLOCKED row is not an executed VAD failure. It means the map lacks a "
            "freshly validated Autoware Lanelet2 + point-cloud bundle (or a CARLA "
            "runtime prerequisite), so no closed-loop result is claimed.",
            "",
        ]
    )
    atomic_text(output_root / "SUMMARY.md", "\n".join(lines))
    return aggregate


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    prepare = sub.add_parser("prepare")
    prepare.add_argument("--manifest", type=Path, default=DEFAULT_MATRIX)
    prepare.add_argument("--output-root", type=Path, required=True)
    prepare.add_argument(
        "--runtime-profile",
        choices=RUNTIME_PROFILE_SELECTORS,
        default="recommended",
    )
    prepare.add_argument("--resume", action="store_true")
    listing = sub.add_parser("list-runnable")
    listing.add_argument("--output-root", type=Path, required=True)
    select = sub.add_parser("select-routes")
    select.add_argument("--output-root", type=Path, required=True)
    select.add_argument("--map-id", required=True)
    select.add_argument("--catalog", type=Path)
    select.add_argument("--straight-catalog", type=Path)
    select.add_argument("--turn-catalog", type=Path)
    admitted = sub.add_parser("select-admitted-routes")
    admitted.add_argument("--output-root", type=Path, required=True)
    admitted.add_argument("--map-id", required=True)
    trial = sub.add_parser("validate-trial")
    trial.add_argument("--output-root", type=Path, required=True)
    trial.add_argument("--map-id", required=True)
    trial.add_argument("--trial-id", choices=TRIAL_IDS, required=True)
    trial.add_argument("--trial-dir", type=Path, required=True)
    update = sub.add_parser("update")
    update.add_argument("--output-root", type=Path, required=True)
    update.add_argument("--map-id", required=True)
    update.add_argument("--status", choices=("PENDING", "RUNNING", "PASS", "FAILED"))
    update.add_argument("--stage")
    update.add_argument("--reason")
    update.add_argument("--trial-id", choices=TRIAL_IDS)
    update.add_argument("--trial-status", choices=("PENDING", "RUNNING", "PASS", "FAILED"))
    update.add_argument("--attempt-dir", type=Path)
    update.add_argument("--validation", type=Path)
    summary = sub.add_parser("summarize")
    summary.add_argument("--output-root", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    try:
        if args.command == "prepare":
            value = prepare_output(
                args.manifest,
                args.output_root,
                args.resume,
                args.runtime_profile,
            )
            print(
                f"PLAN maps={value['canonical_map_count']} "
                f"runnable={value['runnable_map_count']} "
                f"profile={value['runtime_profile_selector']} "
                f"output={args.output_root.resolve()}"
            )
        elif args.command == "list-runnable":
            value = _load_verified_campaign_plan(
                args.output_root.resolve(), "runnable-map listing"
            )
            for entry in value["maps"]:
                if entry.get("runnable"):
                    bundle = entry["full_map_bundle"]
                    print(
                        "\t".join(
                            (
                                entry["map_id"],
                                entry["canonical_name"],
                                entry["server_profile"],
                                bundle["path"],
                                bundle["bundle_schema"],
                            )
                        )
                    )
        elif args.command == "select-routes":
            value = select_routes(
                args.output_root,
                args.map_id,
                args.catalog,
                straight_catalog_path=args.straight_catalog,
                turn_catalog_path=args.turn_catalog,
            )
            print(
                f"ROUTES map={args.map_id} "
                + " ".join(
                    f"{trial['trial_id']}={trial['catalog_scenario']}"
                    for trial in value["trials"]
                )
            )
        elif args.command == "select-admitted-routes":
            value = select_admitted_routes(args.output_root, args.map_id)
            print(
                f"PREFLIGHT_ROUTES map={args.map_id} seed={value['route_seed']} "
                + " ".join(
                    f"{trial['trial_id']}={trial['catalog_scenario']}"
                    for trial in value["trials"]
                )
            )
        elif args.command == "validate-trial":
            value = validate_trial(
                args.output_root, args.map_id, args.trial_id, args.trial_dir
            )
            print(
                f"PASS map={args.map_id} trial={args.trial_id} "
                f"scenario={value['catalog_scenario']}"
            )
        elif args.command == "update":
            update_status(
                args.output_root,
                args.map_id,
                args.status,
                args.stage,
                args.reason,
                args.trial_id,
                args.trial_status,
                args.attempt_dir,
                args.validation,
            )
        elif args.command == "summarize":
            value = summarize(args.output_root)
            print(
                f"SUMMARY status={value['status']} "
                f"pass={value['runnable_pass_count']}/{value['runnable_map_count']}"
            )
    except (MatrixError, OSError, ValueError, yaml.YAMLError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
