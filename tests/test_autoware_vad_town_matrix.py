from __future__ import annotations

import importlib.util
import json
import math
from copy import deepcopy
from pathlib import Path
import subprocess
from unittest.mock import patch

from PIL import Image
import pytest
import yaml


ROOT = Path(__file__).parents[1]
MODULE_PATH = ROOT / "scripts/e2e/autoware_vad_town_matrix.py"
MATRIX_PATH = ROOT / "scripts/e2e/autoware_vad_town_matrix.yaml"
RUNNER = ROOT / "scripts/e2e/run_autoware_vad_town_matrix.sh"
TOWN01_V4_TURN_RANKING_FIXTURE = (
    ROOT / "tests/fixtures/town01_v4_turn_ranking_metrics.json"
)
C_TRACK_V4_TURN_RANKING_FIXTURE = (
    ROOT / "tests/fixtures/c_track_v4_turn_ranking_metrics.json"
)
SPEC = importlib.util.spec_from_file_location("autoware_vad_town_matrix", MODULE_PATH)
assert SPEC and SPEC.loader
matrix = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(matrix)

BASE_ROUTE_CONTRACT = {
    "weather": "ClearNoon",
    "seed": 0,
    "pairs_per_seed": 1,
    "minimum_distance_m": 20.0,
    "maximum_distance_m": 120.0,
    "preferred_distance_m": 60.0,
    "sampling_resolution_m": 1.0,
    "maximum_endpoint_offset_m": 2.0,
    "maximum_traces_per_scenario": 5000,
}


def write_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


def attach_goal_endpoint_provenance(
    payload: dict,
    *,
    endpoint_source: str = "spawn_points",
    endpoint_index: int | None = None,
    original_z_offset_m: float = 0.5,
) -> None:
    terminal = payload["route"][-1]
    runtime_z = float(terminal["z"])
    original_z = runtime_z + original_z_offset_m
    goal_ros = {
        "x": float(terminal["x"]),
        "y": float(terminal["y"]),
        "z": runtime_z,
        "yaw": float(terminal["yaw"]),
    }
    goal_carla = {
        "x": goal_ros["x"],
        "y": -goal_ros["y"],
        "z": runtime_z,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": -math.degrees(goal_ros["yaw"]),
    }
    original_carla = {**goal_carla, "z": original_z}
    original_ros = {**goal_ros, "z": original_z}
    if endpoint_index is None:
        endpoint_index = (
            int(payload["goal_spawn_index"])
            if endpoint_source == "spawn_points"
            else int(payload["goal_endpoint_index"])
        )
    payload["goal_carla_transform"] = goal_carla
    payload["goal_ros_pose"] = goal_ros
    payload["goal_endpoint_provenance"] = {
        "endpoint_source": endpoint_source,
        "endpoint_index": endpoint_index,
        "original_goal_carla_transform": original_carla,
        "original_goal_ros_pose": original_ros,
        "terminal_z_normalization": {
            "policy": "last_road_waypoint_z",
            "original_endpoint_z_m": original_z,
            "last_road_waypoint_z_m": runtime_z,
            "runtime_goal_z_m": runtime_z,
            "serialized_terminal_z_m": runtime_z,
            "applied_offset_m": runtime_z - original_z,
        },
    }


def route_payload(
    scenario: str, options: list[str], route_length_m: float | None = None
) -> dict:
    command = matrix.VAD_COMMANDS
    if route_length_m is None:
        route_length_m = float(len(options) - 1)
    spacing = route_length_m / float(len(options) - 1)
    points = []
    for index, option in enumerate(options):
        points.append(
            {
                "index": index,
                "distance_m": float(index) * spacing,
                "x": float(index) * spacing,
                "y": 0.0,
                "z": 0.0,
                "yaw": 0.0,
                "road_option": option,
                "vad_command": command[option],
            }
        )
    counts: dict[str, int] = {}
    for option in options:
        counts[option] = counts.get(option, 0) + 1
    payload = {
        "schema_version": 1,
        "town": "TownFixture",
        "scenario": scenario,
        "route_length_m": route_length_m,
        "start_spawn_index": 1,
        "goal_spawn_index": 2,
        "option_counts": counts,
        "route": points,
    }
    attach_goal_endpoint_provenance(payload)
    return payload


def physical_turn_route_payload(
    scenario: str, *, lead_m: int = 30, tail_m: int = 15
) -> dict:
    sign = 1.0 if scenario == "left" else -1.0
    radius_m = 10.0
    points = []
    distance_m = 0.0

    def append(x: float, y: float, yaw: float, option: str) -> None:
        nonlocal distance_m
        if points:
            distance_m += math.hypot(
                x - points[-1]["x"], y - points[-1]["y"]
            )
        points.append(
            {
                "index": len(points),
                "distance_m": distance_m,
                "x": x,
                "y": y,
                "z": 0.0,
                "yaw": yaw,
                "road_option": option,
                "vad_command": matrix.VAD_COMMANDS[option],
            }
        )

    for x in range(lead_m):
        append(float(x), 0.0, 0.0, "LANEFOLLOW")
    center_x = float(lead_m - 1)
    center_y = sign * radius_m
    for sample in range(1, 18):
        fraction = sample / 17.0
        angle = -sign * math.pi / 2.0 + sign * fraction * math.pi / 2.0
        append(
            center_x + radius_m * math.cos(angle),
            center_y + radius_m * math.sin(angle),
            sign * fraction * math.pi / 2.0,
            scenario.upper(),
        )
    for offset in range(1, tail_m + 1):
        append(
            center_x + radius_m,
            center_y + sign * offset,
            sign * math.pi / 2.0,
            "LANEFOLLOW",
        )
    counts: dict[str, int] = {}
    for point in points:
        option = point["road_option"]
        counts[option] = counts.get(option, 0) + 1
    payload = {
        "schema_version": 1,
        "town": "TownFixture",
        "scenario": scenario,
        "route_length_m": points[-1]["distance_m"],
        "start_spawn_index": 1,
        "goal_spawn_index": 2,
        "option_counts": counts,
        "route": points,
    }
    attach_goal_endpoint_provenance(payload)
    return payload


def turn_ranking_vectors_from_fixture(
    fixture_path: Path,
) -> tuple[dict, list[tuple[tuple, dict, dict]]]:
    fixture = json.loads(fixture_path.read_text(encoding="utf-8"))
    manifest, _ = matrix.load_matrix(MATRIX_PATH)
    profile = matrix.select_runtime_profile(manifest, "speed_30kph")
    speed_contract = profile["speed_contract"]
    vectors = []
    ranking_rows = fixture.get(
        "common_pass_ranking_metrics", fixture["candidates"]
    )
    for row in ranking_rows:
        (
            route_id,
            scenario,
            seed,
            pair_index,
            p95_curvature,
            route_lead,
            command_lead,
            command_tail,
            core_arc,
        ) = row
        candidate = {
            "id": route_id,
            "scenario": scenario,
            "seed": seed,
            "pair_index": pair_index,
        }
        lead_field = (
            "block_start_distance_m"
            if fixture["source_map"] == "c_track_1_0_7"
            else "route_lead_distance_m"
        )
        physical_turn = {
            "status": "PASS",
            "selected_block": {
                "p95_absolute_curvature_per_m": p95_curvature,
                lead_field: route_lead,
                "command_lead_distance_m": command_lead,
                "command_tail_distance_m": command_tail,
                "command_arc_length_m": core_arc,
            },
            "limits": {"maximum_p95_abs_curvature_per_m": 0.20},
        }
        vector = matrix._speed_30kph_turn_ranking_vector(
            candidate,
            physical_turn,
            speed_contract,
            fixture["measurement_source"],
        )
        vectors.append((tuple(vector["sort_key"]), candidate, vector))
    return fixture, sorted(vectors, key=lambda item: item[0])


def synthetic_plan(
    output_root: Path,
    bundle_metadata: Path | None = None,
    bundle_schema: str | None = None,
    runtime_profile: dict | None = None,
    runtime_profile_selector: str = "recommended",
) -> dict:
    bundle = {
        "path": str(bundle_metadata.parent) if bundle_metadata else "/bundle",
        "metadata_sha256": (
            matrix.sha256_file(bundle_metadata) if bundle_metadata else "bundle-sha"
        ),
    }
    if bundle_schema is not None:
        bundle["bundle_schema"] = bundle_schema
    if bundle_metadata is not None:
        bundle["profile"] = json.loads(
            bundle_metadata.read_text(encoding="utf-8")
        ).get("profile", "")
        bundle_files = {
            file_id: bundle_metadata.parent / filename
            for file_id, filename in matrix.BUNDLE_FILE_NAMES.items()
        }
        if all(path.is_file() for path in bundle_files.values()):
            bundle["bundle_file_sha256"] = {
                file_id: matrix.sha256_file(path)
                for file_id, path in bundle_files.items()
            }
    if runtime_profile is None:
        runtime_profile = {
            "id": "recommended",
            "map_lifecycle": "cold_start_owned_process_group_per_trial",
            "wrapper_options": [
                "--recommended",
                "--visualize",
                "--capture-desktop",
            ],
        }
    route_contract = dict(BASE_ROUTE_CONTRACT)
    generation_contracts = matrix._route_generation_contracts(
        route_contract, runtime_profile, runtime_profile_selector
    )
    selection_overrides = matrix._runtime_route_selection_overrides(
        runtime_profile, runtime_profile_selector
    )
    turn_selection_policy = matrix._runtime_turn_route_selection_policy(
        runtime_profile, runtime_profile_selector
    )
    map_selection_overrides = matrix._resolved_map_route_selection_overrides(
        runtime_profile, runtime_profile_selector, "town_fixture"
    )
    execution_contract = matrix._campaign_execution_contract()
    plan = {
        "schema_version": 1,
        "matrix_id": "fixture",
        "matrix_manifest_sha256": "0" * 64,
        "canonical_map_manifest_sha256": "1" * 64,
        "runtime_profile_selector": runtime_profile_selector,
        "runtime_profile": runtime_profile,
        "route_contract": route_contract,
        "route_generation_contracts": generation_contracts,
        "route_selection_policy": matrix.ROUTE_SELECTION_POLICY,
        "route_selection_overrides": selection_overrides,
        "route_selection_overrides_sha256": matrix.sha256_json(
            selection_overrides
        ),
        "turn_route_selection_policy": turn_selection_policy,
        "turn_route_selection_policy_sha256": (
            matrix.sha256_json(turn_selection_policy)
            if turn_selection_policy is not None
            else None
        ),
        "campaign_execution_contract": execution_contract,
        "campaign_execution_contract_sha256": matrix.sha256_json(
            execution_contract
        ),
        "maps": [
            {
                "map_id": "town_fixture",
                "canonical_name": "TownFixture",
                "load_name": "/Game/Carla/Maps/TownFixture",
                "runnable": True,
                "full_map_bundle": bundle,
                "route_generation_contracts": generation_contracts,
                "route_selection_overrides": map_selection_overrides,
                "route_selection_overrides_sha256": matrix.sha256_json(
                    map_selection_overrides
                ),
            }
        ],
    }
    plan["admission_contract_sha256"] = matrix.sha256_json(
        matrix._admission_contract_payload(plan)
    )
    write_json(output_root / "matrix_plan.json", plan)
    return plan


def make_catalog(
    output_root: Path,
    corrupt_straight: bool = False,
    generation_contract: dict | None = None,
    catalog_id: str | None = None,
) -> Path:
    catalog_root = output_root / "maps/town_fixture/catalog"
    if catalog_id is not None:
        catalog_root /= catalog_id
    if generation_contract is None:
        generation_contract = matrix._route_generation_contracts(
            BASE_ROUTE_CONTRACT,
            {"id": "recommended", "wrapper_options": []},
            "recommended",
        )["straight"]
    routes = []
    values = {
        "straight": ["LANEFOLLOW", "STRAIGHT", "STRAIGHT", "LANEFOLLOW"],
        "left": ["LANEFOLLOW", "LEFT", "LEFT", "LANEFOLLOW"],
        "right": ["LANEFOLLOW", "RIGHT", "LANEFOLLOW"],
    }
    if catalog_id == "straight":
        values = {"straight": values["straight"]}
    elif catalog_id == "turn":
        values = {name: values[name] for name in ("left", "right")}
    endpoint_source = (
        "generated_waypoints" if catalog_id == "straight" else "spawn_points"
    )
    if corrupt_straight:
        values["straight"].append("LEFT")
    for scenario, options in values.items():
        route_id = f"town_fixture_{scenario}_s0000_p00"
        path = catalog_root / "routes/town_fixture" / scenario / f"{route_id}.json"
        payload = (
            physical_turn_route_payload(scenario)
            if catalog_id == "turn"
            else route_payload(
                scenario,
                options,
                float(generation_contract["preferred_distance_m"]),
            )
        )
        for point in payload["route"]:
            point["z"] = 0.0
        payload.update(
            coordinate_reference="base_link",
            spawn_point_reference="base_link",
            spawn_point="0,0,0,0,0,0",
            start_ros_pose={
                "x": payload["route"][0]["x"],
                "y": payload["route"][0]["y"],
                "z": payload["route"][0]["z"],
                "yaw": payload["route"][0]["yaw"],
            },
            goal_ros_pose={
                "x": payload["route"][-1]["x"],
                "y": payload["route"][-1]["y"],
                "z": payload["route"][-1]["z"],
                "yaw": payload["route"][-1]["yaw"],
            },
        )
        if endpoint_source == "generated_waypoints":
            payload.pop("start_spawn_index")
            payload.pop("goal_spawn_index")
            payload.update(
                endpoint_source="generated_waypoints",
                endpoint_waypoint_spacing_m=10.0,
                start_endpoint_index=0,
                goal_endpoint_index=1,
                spawn_height_contract={
                    "endpoint_transform_z_m": 0.0,
                    "catalog_z_offset_m": 0.0,
                    "bridge_z_offset_m": 2.0,
                    "effective_actor_spawn_z_m": 2.0,
                    "offset_owner": "autoware_carla_interface_bridge",
                    "bridge_source": "fixture",
                },
            )
        attach_goal_endpoint_provenance(
            payload,
            endpoint_source=endpoint_source,
            endpoint_index=(
                int(payload["goal_endpoint_index"])
                if endpoint_source == "generated_waypoints"
                else int(payload["goal_spawn_index"])
            ),
        )
        physical_straight_preflight = (
            matrix._speed_30kph_straight_geometry(
                payload,
                matrix.SPEED_30KPH_STRAIGHT_GEOMETRY_CONTRACT,
            )
            if scenario == "straight" and catalog_id == "straight"
            else None
        )
        if physical_straight_preflight is not None:
            payload["physical_straight_preflight"] = (
                physical_straight_preflight
            )
        physical_turn_preflight = (
            matrix._speed_30kph_turn_geometry(
                payload,
                matrix.SPEED_30KPH_TURN_GEOMETRY_CONTRACT,
            )
            if scenario in ("left", "right") and catalog_id == "turn"
            else None
        )
        if physical_turn_preflight is not None:
            payload["physical_turn_preflight"] = physical_turn_preflight
        write_json(path, payload)
        routes.append(
            {
                "id": route_id,
                "status": "ready",
                "scenario": scenario,
                "seed": 0,
                "pair_index": 0,
                "path": path.relative_to(catalog_root).as_posix(),
                "sha256": matrix.sha256_file(path),
                **(
                    {
                        "physical_straight_preflight": (
                            physical_straight_preflight
                        )
                    }
                    if physical_straight_preflight is not None
                    else {}
                ),
                **(
                    {"physical_turn_preflight": physical_turn_preflight}
                    if physical_turn_preflight is not None
                    else {}
                ),
            }
        )
    catalog_path = catalog_root / "route_catalog.json"
    write_json(
        catalog_path,
        {
            "status": "complete",
            "map_id": "town_fixture",
            "generation": {
                "weather": generation_contract["weather"],
                "scenarios": list(values),
                "seeds": generation_contract["seeds"],
                "pairs_per_seed": generation_contract["pairs_per_seed"],
                "minimum_distance_m": generation_contract[
                    "minimum_distance_m"
                ],
                "maximum_distance_m": generation_contract[
                    "maximum_distance_m"
                ],
                "preferred_distance_m": generation_contract[
                    "preferred_distance_m"
                ],
                "sampling_resolution_m": generation_contract[
                    "sampling_resolution_m"
                ],
                "maximum_endpoint_offset_m": generation_contract[
                    "maximum_endpoint_offset_m"
                ],
                "max_traces_per_scenario": generation_contract[
                    "maximum_traces_per_scenario"
                ],
                "endpoint_source": endpoint_source,
                "endpoint_waypoint_spacing_m": (
                    10.0 if endpoint_source == "generated_waypoints" else None
                ),
                "endpoint_junction_policy": "include",
                "candidate_enumeration_policy": "all_pairs",
                "straight_capacity_contract": {"enabled": False},
                **(
                    {
                        "endpoint_count": 2,
                        "endpoint_api_count": 2,
                        "endpoint_eligible_api_count": 2,
                        "endpoint_junction_waypoint_count": 0,
                        "endpoint_junction_excluded_count": 0,
                        "endpoint_duplicate_transform_count": 0,
                        "endpoint_deduplication": (
                            "exact_full_transform_keep_first_api_occurrence"
                        ),
                        "endpoint_ordering": (
                            "deduplicated_lexicographic_transform_x_y_z_roll_pitch_yaw"
                        ),
                        "spawn_height_contract": {
                            "catalog_z_offset_m": 0.0,
                            "bridge_z_offset_m": 2.0,
                            "offset_owner": "autoware_carla_interface_bridge",
                            "bridge_source": "fixture",
                        },
                        "physical_straight_contract": {
                            "enabled": True,
                            "profile_id": "speed_30kph",
                            "measurement_source": (
                                "exact_serialized_route_with_terminal_goal"
                            ),
                            "admission_policy": (
                                "reject_before_accepted_pair_quota"
                            ),
                            "limits": dict(
                                matrix.SPEED_30KPH_STRAIGHT_GEOMETRY_CONTRACT
                            ),
                        },
                    }
                    if endpoint_source == "generated_waypoints"
                    else {}
                ),
                **(
                    {
                        "physical_turn_contract": {
                            "enabled": True,
                            "profile_id": "speed_30kph",
                            "applicability": "packaged_town_only",
                            "measurement_source": (
                                "exact_serialized_3d_route_with_terminal_goal"
                            ),
                            "admission_policy": (
                                "reject_before_accepted_pair_quota"
                            ),
                            "limits": dict(
                                matrix.SPEED_30KPH_TURN_GEOMETRY_CONTRACT
                            ),
                            "provenance": dict(
                                matrix.SPEED_30KPH_TURN_CONTRACT_PROVENANCE
                            ),
                        }
                    }
                    if catalog_id == "turn"
                    else {}
                ),
            },
            "server": {
                "active_map_name": "/Game/Carla/Maps/TownFixture",
                "map_load_allowed": False,
                "map_load_performed": False,
            },
            "routes": routes,
        },
    )
    return catalog_path


def make_custom_catalog(
    output_root: Path,
    sharp_straight: bool = False,
    compound_turn: bool = False,
) -> Path:
    catalog_root = output_root / "maps/town_fixture/catalog"
    routes = []
    preflight = {
        "status": "PASS",
        "distance_m": 15.0,
        "covered_distance_m": 15.0,
        "maximum_lateral_deviation_m": 0.0,
        "maximum_heading_change_deg": 0.0,
        "limits": {
            "maximum_lateral_deviation_m": 1.5,
            "maximum_heading_change_deg": 30.0,
        },
        "failure_reasons": [],
    }
    for scenario, maneuver in (("straight", "STRAIGHT"), ("left", "LEFT")):
        if scenario == "left":
            options = ["LANEFOLLOW"] * 16 + ["LEFT"] * 13 + ["LANEFOLLOW"] * 4
            if compound_turn:
                options += ["LEFT"] * 3 + ["LANEFOLLOW"]
        else:
            options = ["LANEFOLLOW"] * 16 + [maneuver] * 3 + ["LANEFOLLOW"] * 2
        payload = route_payload(scenario, options)
        for index, point in enumerate(payload["route"]):
            point.update(x=float(index), y=0.0, yaw=0.0)
        if scenario == "left":
            first_turn = 16
            last_turn = 28
            for index, point in enumerate(payload["route"]):
                if first_turn <= index <= last_turn:
                    point["yaw"] = (index - first_turn) * (0.5 * 3.141592653589793 / 12)
                elif index > last_turn:
                    point["yaw"] = 0.5 * 3.141592653589793
        if sharp_straight and scenario == "straight":
            for index, point in enumerate(payload["route"][5:], start=5):
                point.update(x=5.0, y=float(index - 5), yaw=0.5 * 3.141592653589793)
        attach_goal_endpoint_provenance(payload)
        payload["initial_approach_preflight"] = preflight
        turn_preflight = None
        if scenario == "left":
            try:
                turn_preflight = matrix._custom_route_turn_geometry(
                    payload, scenario
                )
            except matrix.MatrixError:
                turn_preflight = {
                    "status": "PASS",
                    "scenario": "left",
                    "directional_command": "LEFT",
                    "directional_block_count": 1,
                    "additional_maneuver_commands": [],
                    "failure_reasons": [],
                }
            payload["turn_geometry_preflight"] = turn_preflight
        route_id = f"town_fixture_{scenario}_s0000_p00"
        path = catalog_root / "routes/town_fixture" / scenario / f"{route_id}.json"
        write_json(path, payload)
        routes.append(
            {
                "id": route_id,
                "status": "ready",
                "scenario": scenario,
                "seed": 0,
                "pair_index": 0,
                "path": path.relative_to(catalog_root).as_posix(),
                "sha256": matrix.sha256_file(path),
                "initial_approach_preflight": preflight,
                **(
                    {"turn_geometry_preflight": turn_preflight}
                    if turn_preflight is not None
                    else {}
                ),
            }
        )
    catalog_path = catalog_root / "route_catalog.json"
    write_json(
        catalog_path,
        {
            "status": "complete",
            "map_id": "town_fixture",
            "generation": {
                "weather": "ClearNoon",
                "seeds": [0],
                "pairs_per_seed": 1,
                "minimum_distance_m": 20.0,
                "maximum_distance_m": 120.0,
                "preferred_distance_m": 60.0,
                "sampling_resolution_m": 1.0,
                "maximum_endpoint_offset_m": 2.0,
                "max_traces_per_scenario": 5000,
                "endpoint_junction_policy": "include",
                "candidate_enumeration_policy": "all_pairs",
                "straight_capacity_contract": {"enabled": False},
                "initial_approach_contract": dict(
                    matrix.CUSTOM_MAP_INITIAL_APPROACH_CONTRACT
                ),
                "turn_geometry_contract": dict(
                    matrix.CUSTOM_MAP_TURN_GEOMETRY_CONTRACT
                ),
            },
            "server": {
                "active_map_name": "/Game/Carla/Maps/TownFixture",
                "map_load_allowed": False,
                "map_load_performed": False,
            },
            "routes": routes,
        },
    )
    return catalog_path


def make_trial_evidence(
    output_root: Path, bundle: Path, route_entry: dict
) -> tuple[Path, dict, dict]:
    trial_id = route_entry["trial_id"]
    scenario = route_entry["catalog_scenario"]
    trial = output_root / (
        f"maps/town_fixture/trials/{trial_id}/attempt_001"
    )
    trial.mkdir(parents=True)
    selected_route = Path(route_entry["route_path"])
    (trial / "source_route.json").write_bytes(selected_route.read_bytes())
    (trial / "map_bundle.json").write_bytes(bundle.read_bytes())
    (trial / "runtime.env").write_text(
        "RECOMMENDED=true\nVISUALIZE=true\nCAPTURE_DESKTOP=true\n",
        encoding="utf-8",
    )
    result = {
        "success": True,
        "execution_mode": "full_stack",
        "assessment": {
            "planning_architecture": "vad_route_manager_hybrid",
            "route_completion": "PASS",
        },
        "final": {"goal_reached": True, "route_status": "goal_reached"},
    }
    write_json(trial / "result.json", result)
    write_json(
        trial / "diagnosis.json",
        {"inputs": {"town": "TownFixture", "scenario": scenario}},
    )
    write_json(
        trial / "latency/e2e_latency.json",
        {
            "selected_topics": {
                "vad_output": "/planning/vad/candidate_trajectories"
            },
            "event_rates": {
                "/planning/vad/candidate_trajectories": {"count": 12}
            },
        },
    )
    for name in (
        "path_vs_control.png",
        "steering_tracking.png",
        "route_result.png",
    ):
        Image.new("RGB", (32, 18), "navy").save(trial / name)
    Image.new("RGB", (32, 18), "navy").save(
        trial / "turn_path_control.gif", save_all=True
    )
    Image.new("RGB", (1200, 800), "navy").save(
        trial / "autoware_rviz_fullscreen.png"
    )
    Image.new("RGB", (960, 640), "navy").save(
        trial / "autoware_rviz_drive.gif", save_all=True
    )
    desktop = {
        "schema_version": 1,
        "candidate_observed": True,
        "candidate_topic": "/planning/vad/candidate_trajectories",
        "capture_started_after_candidate": True,
        "source_dimensions": [1200, 800],
        "png_dimensions": [1200, 800],
        "gif_dimensions": [960, 640],
    }
    write_json(trial / "desktop_capture.json", desktop)
    add_carla_lifecycle_evidence(trial, trial_id)
    return trial, desktop, result


def add_carla_lifecycle_evidence(trial: Path, trial_id: str) -> None:
    generation = f"town_fixture_{trial_id}_{trial.name}"
    owner_pid = 4242
    owner_pgid = 4242
    server_log = trial / "carla_server.log"
    server_log.write_text("CARLA_READY map=Carla/Maps/TownFixture\n", encoding="utf-8")
    log_record = {
        "path": str(server_log.resolve()),
        "size_bytes": server_log.stat().st_size,
        "sha256": matrix.sha256_file(server_log),
    }
    common = {
        "schema_version": 1,
        "status": "PASS",
        "generation_id": generation,
        "host": "127.0.0.1",
        "port": 2100,
        "expected_map": "TownFixture",
        "owner_pid": owner_pid,
        "owner_pgid": owner_pgid,
        "timeout_seconds": 3.0,
        "probe_wall_seconds": 0.01,
        "error": None,
        "read_only": True,
        "server_log": log_record,
    }
    for filename, stage, checked_at, mode in (
        (
            "carla_preflight_health.json",
            "trial_preflight",
            "2026-09-01T00:00:00.100000+00:00",
            "running",
        ),
        (
            "carla_completion_health.json",
            "trial_completion",
            "2026-09-01T00:00:00.200000+00:00",
            "running",
        ),
        (
            "carla_cleanup_health.json",
            "trial_cleanup",
            "2026-09-01T00:00:00.300000+00:00",
            "stopped",
        ),
    ):
        payload = {
            **common,
            "mode": mode,
            "stage": stage,
            "checked_at": checked_at,
        }
        if mode == "running":
            payload.update(
                {
                    "owner_process_state": "S",
                    "active_map_name": "Carla/Maps/TownFixture",
                    "active_map_basename": "TownFixture",
                    "snapshot_frame": 42,
                    "snapshot_elapsed_seconds": 12.5,
                    "rpc_sequence": [
                        "get_world",
                        "world.get_map",
                        "world.get_snapshot",
                    ],
                }
            )
        else:
            payload.update(
                {
                    "owner_process_state": None,
                    "port_released": True,
                    "rpc_sequence": [],
                }
            )
        write_json(trial / filename, payload)
    runtime = trial / "runtime.env"
    with runtime.open("a", encoding="utf-8") as stream:
        stream.write(
            "CARLA_HOST=127.0.0.1\n"
            "CARLA_PORT=2100\n"
            "CARLA_LIFECYCLE=cold_start_owned_process_group_per_trial\n"
            f"CARLA_GENERATION_ID={generation}\n"
            "CARLA_EXPECTED_MAP=TownFixture\n"
            f"CARLA_OWNER_PID={owner_pid}\n"
            f"CARLA_OWNER_PGID={owner_pgid}\n"
            f"CARLA_SERVER_LOG={server_log.resolve()}\n"
            "CARLA_MATRIX_OWNED=true\n"
        )


def add_speed_30kph_evidence(
    trial: Path, profile: dict, scenario: str, result: dict
) -> None:
    contract = profile["speed_contract"]
    trial_id = "straight" if scenario == "straight" else "turn"
    trial_contract = contract["trials"][trial_id]
    parameters = contract["route_manager_parameters"]
    gate_contract = contract["vehicle_cmd_gate"]
    controller_contract = contract["longitudinal_controller"]
    gate_source = (
        ROOT
        / "autoware_e2e_vad_launch/config/vehicle_cmd_gate_carla_30kph.param.yaml"
    )
    metadata_source = Path(f"{gate_source}.metadata.json")
    controller_source = (
        ROOT / "autoware_e2e_vad_launch/config/pid_carla_vad_30kph.param.yaml"
    )
    controller_metadata_source = Path(f"{controller_source}.metadata.json")
    provenance = trial / "speed_profile_provenance"
    provenance.mkdir(parents=True)
    gate_copy = provenance / "vehicle_cmd_gate.param.yaml"
    metadata_copy = provenance / "vehicle_cmd_gate.param.yaml.metadata.json"
    controller_copy = provenance / "longitudinal_controller.param.yaml"
    controller_metadata_copy = provenance / (
        "longitudinal_controller.param.yaml.metadata.json"
    )
    gate_copy.write_bytes(gate_source.read_bytes())
    metadata_copy.write_bytes(metadata_source.read_bytes())
    controller_copy.write_bytes(controller_source.read_bytes())
    controller_metadata_copy.write_bytes(controller_metadata_source.read_bytes())
    gate_sha256 = matrix.sha256_file(gate_copy)
    metadata_sha256 = matrix.sha256_file(metadata_copy)
    controller_sha256 = matrix.sha256_file(controller_copy)
    controller_metadata_sha256 = matrix.sha256_file(controller_metadata_copy)
    (provenance / "SHA256SUMS").write_text(
        f"{gate_sha256}  vehicle_cmd_gate.param.yaml\n"
        f"{metadata_sha256}  vehicle_cmd_gate.param.yaml.metadata.json\n"
        f"{controller_sha256}  longitudinal_controller.param.yaml\n"
        f"{controller_metadata_sha256}  "
        "longitudinal_controller.param.yaml.metadata.json\n",
        encoding="utf-8",
    )

    manager_dump = {
        "/vad_route_manager": {"ros__parameters": dict(parameters)}
    }
    (trial / "vad_route_manager.params.yaml").write_text(
        yaml.safe_dump(manager_dump, sort_keys=True), encoding="utf-8"
    )
    gate_document = yaml.safe_load(gate_copy.read_text(encoding="utf-8"))
    live_gate_dump = {
        "/control/vehicle_cmd_gate": gate_document["/**"]
    }
    (trial / "vehicle_cmd_gate.params.yaml").write_text(
        yaml.safe_dump(live_gate_dump, sort_keys=True), encoding="utf-8"
    )
    controller_document = yaml.safe_load(
        controller_copy.read_text(encoding="utf-8")
    )
    live_controller_dump = {
        "/control/trajectory_follower/controller_node_exe": (
            controller_document["/**"]
        )
    }
    (trial / "controller.params.yaml").write_text(
        yaml.safe_dump(live_controller_dump, sort_keys=True), encoding="utf-8"
    )

    runtime_values = {
        "RECOMMENDED": "true",
        "VISUALIZE": "true",
        "CAPTURE_DESKTOP": "true",
        "VSCODE_SNAP_GUI_ENV_SANITIZED": "false",
        "SPEED_30KPH": "true",
        "TIGHT_CORRIDOR_CANDIDATE": "false",
        "TRAJECTORY_STABILITY_CANDIDATE": "false",
        "SMART_MPC": "false",
        "FP16_HEADS": "false",
        "SPEED_PROFILE_ID": contract["profile_id"],
        "ROUTE_SCENARIO": scenario,
        "SPEED_EXPOSURE_MODE": trial_contract["exposure_mode"],
        "LONGITUDINAL_SPEED_SOURCE": contract["longitudinal_speed_source"],
        "LONGITUDINAL_ACCELERATION_ROLE": contract[
            "longitudinal_acceleration_role"
        ],
        "VAD_GEOMETRY_SOURCE": "true",
        "VAD_VELOCITY_EVALUATED": "false",
        "VAD_GEOMETRY_EVALUATED": "true",
        "VAD_CRUISE_VELOCITY_EVALUATED": "false",
        "VAD_HARD_STOP_SENTINEL_PRESERVED": "true",
        "CLOSED_LOOP_VALIDATION_STATE": contract["validation_state"],
        "TARGET_SPEED_MPS": contract["target_speed_mps"],
        "TARGET_SPEED_KPH": 30.0,
        "MINIMUM_SUSTAINED_SPEED_MPS": trial_contract[
            "minimum_sustained_speed_mps"
        ],
        "MINIMUM_SUSTAINED_SPEED_SEC": trial_contract[
            "minimum_sustained_speed_sec"
        ],
        "MAXIMUM_OBSERVED_SPEED_MPS": contract[
            "maximum_observed_speed_mps"
        ],
        "MAXIMUM_SPEED_SAMPLE_GAP_SEC": contract[
            "maximum_speed_sample_gap_sec"
        ],
        "MAXIMUM_LATERAL_ACCELERATION_LIMIT_MPS2": trial_contract[
            "maximum_lateral_acceleration_mps2"
        ],
        "MAXIMUM_LONGITUDINAL_ACCELERATION_MPS2": parameters[
            "maximum_longitudinal_acceleration_mps2"
        ],
        "COMMAND_GATE_NOMINAL_LONGITUDINAL_ACCELERATION_MPS2": gate_contract[
            "longitudinal_acceleration_limit_mps2"
        ],
        "LONGITUDINAL_PID_MAX_OUT_MPS2": controller_contract[
            "maximum_output_mps2"
        ],
        "LONGITUDINAL_PID_MAX_P_EFFORT_MPS2": controller_contract[
            "maximum_proportional_effort_mps2"
        ],
        "MAXIMUM_LATERAL_ACCELERATION_MPS2": parameters[
            "maximum_lateral_acceleration_mps2"
        ],
        "CONTROLLER_STOP_OFFSET_M": parameters["controller_stop_offset_m"],
        "MANEUVER_LOOKAHEAD_M": parameters["maneuver_lookahead_m"],
        "MANEUVER_EXIT_LOOKAHEAD_M": parameters["maneuver_exit_lookahead_m"],
        "ROUTE_CURVATURE_LOOKAHEAD_M": parameters[
            "route_curvature_lookahead_m"
        ],
        "CURVATURE_SPEED_PREVIEW_M": parameters["curvature_speed_preview_m"],
        "MAX_ROUTE_DEVIATION_M": parameters["max_route_deviation_m"],
        "MAX_CANDIDATE_AGE_SEC": parameters["max_candidate_age_sec"],
        "CANDIDATE_TIMEOUT_SEC": parameters["candidate_timeout_sec"],
        "COMFORTABLE_DECELERATION_MPS2": parameters[
            "comfortable_deceleration_mps2"
        ],
        "SPEED_LIMIT_SOURCE": gate_contract["speed_limit_source"],
        "REAL_VEHICLE_READY": "false",
        "VAD_ROUTE_MANAGER_OPENBLAS_NUM_THREADS": "1",
        "VAD_ROUTE_MANAGER_OMP_NUM_THREADS": "1",
        "VAD_ROUTE_MANAGER_MKL_NUM_THREADS": "1",
        "VAD_ROUTE_MANAGER_NUMEXPR_NUM_THREADS": "1",
        "VEHICLE_CMD_GATE_PARAM_FILE": str(gate_source),
        "VEHICLE_CMD_GATE_PARAM_SHA256": gate_sha256,
        "VEHICLE_CMD_GATE_METADATA_SHA256": metadata_sha256,
        "LONGITUDINAL_CONTROLLER_PARAM_FILE": str(controller_source),
        "LONGITUDINAL_CONTROLLER_PARAM_SHA256": controller_sha256,
        "LONGITUDINAL_CONTROLLER_METADATA_SHA256": (
            controller_metadata_sha256
        ),
    }
    (trial / "runtime.env").write_text(
        "".join(f"{key}={value}\n" for key, value in runtime_values.items()),
        encoding="utf-8",
    )

    expected_command = matrix.VAD_COMMANDS[
        "STRAIGHT" if scenario == "straight" else scenario.upper()
    ]
    maximum_speed = 8.4
    maximum_lateral = 1.2 if scenario == "straight" else 1.7
    sustained_duration = 1.2 if scenario == "straight" else 0.0
    p95_lateral = 1.0 if scenario == "straight" else 1.4
    result.update(
        {
            "route_file": str((trial / "source_route.json").resolve()),
            "profile_context": {
                "longitudinal_velocity_source": parameters[
                    "longitudinal_velocity_source"
                ],
                "vad_velocity_evaluated": contract["vad_velocity_evaluated"],
                "vad_geometry_evaluated": contract["vad_geometry_evaluated"],
            },
            "actual_path": [
                {
                    "command": expected_command,
                    "lateral_acceleration_mps2": p95_lateral,
                },
                {
                    "command": expected_command,
                    "lateral_acceleration_mps2": p95_lateral,
                },
            ],
            "limits": {
                "minimum_sustained_speed_mps": trial_contract[
                    "minimum_sustained_speed_mps"
                ],
                "minimum_sustained_speed_sec": trial_contract[
                    "minimum_sustained_speed_sec"
                ],
                "maximum_observed_speed_mps": contract[
                    "maximum_observed_speed_mps"
                ],
                "maximum_lateral_acceleration_mps2": trial_contract[
                    "maximum_lateral_acceleration_mps2"
                ],
                "maximum_speed_sample_gap_sec": contract[
                    "maximum_speed_sample_gap_sec"
                ],
            },
            "metrics": {
                "commands_seen": [3, expected_command],
                "maximum_observed_speed_mps": maximum_speed,
                "maximum_lateral_acceleration_mps2": maximum_lateral,
                "maximum_sustained_speed_duration_sec": sustained_duration,
                "maximum_speed_sample_gap_sec": 0.1,
                "speed_by_command": {
                    str(expected_command): {
                        "maximum_observed_speed_mps": maximum_speed,
                        "maximum_sustained_speed_duration_sec": sustained_duration,
                    }
                },
            },
            "speed_exposure": {
                "status": "PASS",
                "minimum_sustained_speed_mps": trial_contract[
                    "minimum_sustained_speed_mps"
                ],
                "minimum_sustained_speed_sec": trial_contract[
                    "minimum_sustained_speed_sec"
                ],
                "maximum_observed_speed_limit_mps": contract[
                    "maximum_observed_speed_mps"
                ],
                "maximum_lateral_acceleration_limit_mps2": trial_contract[
                    "maximum_lateral_acceleration_mps2"
                ],
                "maximum_observed_speed_mps": maximum_speed,
                "maximum_sustained_speed_duration_sec": sustained_duration,
                "maximum_speed_sample_gap_sec": 0.1,
                "maximum_lateral_acceleration_mps2": maximum_lateral,
                "p95_lateral_acceleration_mps2": p95_lateral,
                "p95_lateral_acceleration_mps2_by_command": {
                    str(expected_command): p95_lateral
                },
                "p95_turn_lateral_acceleration_mps2": (
                    p95_lateral if scenario != "straight" else None
                ),
                "longitudinal_velocity_source": parameters[
                    "longitudinal_velocity_source"
                ],
                "vad_velocity_evaluated": contract["vad_velocity_evaluated"],
                "vad_geometry_evaluated": contract["vad_geometry_evaluated"],
                "speed_by_command": {
                    str(expected_command): {
                        "maximum_observed_speed_mps": maximum_speed,
                        "maximum_sustained_speed_duration_sec": sustained_duration,
                    }
                },
            },
        }
    )
    write_json(trial / "result.json", result)
    bag = trial / "bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text(
        "rosbag2_bagfile_information:\n  version: 9\n", encoding="utf-8"
    )
    (bag / "fixture.db3").write_bytes(b"speed-profile-fixture")
    bag_files = [
        {
            "path": path.relative_to(bag).as_posix(),
            "size_bytes": path.stat().st_size,
            "sha256": matrix.sha256_file(path),
        }
        for path in sorted(bag.rglob("*"))
        if path.is_file()
    ]
    bag_manifest = {
        "schema_version": 1,
        "root": str(bag.resolve()),
        "files": bag_files,
    }
    bag_manifest["sha256"] = matrix.sha256_json(
        {"schema_version": 1, "files": bag_files}
    )
    route_path = (trial / "source_route.json").resolve()
    result_path = (trial / "result.json").resolve()
    route = json.loads(route_path.read_text(encoding="utf-8"))
    identity = {
        "schema_version": 1,
        "effective_route": {
            "path": str(route_path),
            "sha256": matrix.sha256_file(route_path),
            "town": route["town"],
            "scenario": scenario,
            "trial_id": trial_id,
            "route_length_m": route["route_length_m"],
        },
        "route_result": {
            "path": str(result_path),
            "sha256": matrix.sha256_file(result_path),
            "success": True,
            "execution_mode": "full_stack",
            "profile_context": result["profile_context"],
        },
        "rosbag": bag_manifest,
    }
    identity["sha256"] = matrix.sha256_json(identity)
    required_series = [
        "raw_selected_vad",
        "explicit_overlaid_planning",
        "gated_control_command",
        "actual_odometry",
    ]
    write_json(
        trial / "speed_profile.json",
        {
            "schema_version": 1,
            "analysis": "carla_speed_source_evidence",
            "status": "complete",
            "inputs": {
                "bag": str(bag.resolve()),
                "profile_id": contract["profile_id"],
                "target_speed_mps": contract["target_speed_mps"],
                "longitudinal_speed_source": "explicit_simulation_nominal",
            },
            "source_identity": identity,
            "outputs": {
                "json": "speed_profile.json",
                "plot": "speed_profile.png",
            },
            "quality": {"problems": [], "required_series": required_series},
            "series": {name: [{"time_sec": 0.0}] for name in required_series},
        },
    )
    Image.new("RGB", (640, 360), "purple").save(trial / "speed_profile.png")


def add_speed_30kph_visual_evidence(trial: Path) -> None:
    Image.new("RGB", (1920, 1080), "navy").save(
        trial / "autoware_rviz_fullscreen.png"
    )
    Image.new("RGB", (960, 540), "navy").save(
        trial / "autoware_rviz_drive.gif", save_all=True
    )
    Image.new("RGB", (1920, 1080), "yellow").save(
        trial / "autoware_rviz_candidate.png"
    )
    (trial / "autoware_rviz_capture.mkv").write_bytes(b"fixture recording")
    provenance = trial / "rviz_capture_provenance"
    provenance.mkdir(parents=True)
    config = provenance / "autoware_vad_carla.rviz"
    config.write_bytes(
        (
            ROOT / "autoware_e2e_vad_launch/rviz/autoware_vad_carla.rviz"
        ).read_bytes()
    )
    config_sha256 = matrix.sha256_file(config)
    (provenance / "SHA256SUMS").write_text(
        f"{config_sha256}  autoware_vad_carla.rviz\n", encoding="utf-8"
    )
    desktop_path = trial / "desktop_capture.json"
    desktop = json.loads(desktop_path.read_text(encoding="utf-8"))
    desktop.update(
        {
            "source_dimensions": [1920, 1080],
            "png_dimensions": [1920, 1080],
            "candidate_png_dimensions": [1920, 1080],
            "gif_dimensions": [960, 540],
            "rviz_view_contract": {
                "vehicle_centered": True,
                "target_frame": "base_link",
                "center_xy_m": [0.0, 0.0],
            },
        }
    )
    write_json(desktop_path, desktop)


def fake_campaign_route_map_preflight(
    output_root: Path, plan: dict, entry: dict, selected: list[dict]
) -> dict:
    aligner = matrix._load_e2e_helper(
        "align_carla_route_to_map.py", "test_fake_campaign_route_alignment"
    )
    bundle_path = Path(entry["full_map_bundle"]["path"])
    bundle_payload, bundle_transform = aligner.load_map_bundle(bundle_path)
    transform = {
        field: getattr(bundle_transform, field)
        for field in ("x_m", "y_m", "z_m", "yaw_rad")
    }
    alignment_root = (
        output_root
        / "maps/town_fixture/campaign_route_map_preflight_routes"
    )
    cases = []
    for item in selected:
        source = Path(item["route_path"]).resolve()
        aligned = alignment_root / f"{item['trial_id']}.aligned.json"
        result = aligner.prepare_aligned_route(
            source,
            bundle_path,
            output_path=aligned,
        )
        cases.append(
            {
                "trial_id": item["trial_id"],
                "scenario": item["catalog_scenario"],
                "route_path": str(source),
                "route_sha256": item["route_sha256"],
                "route_length_m": item["analysis"]["route_length_m"],
                "status": "PASS",
                "route_alignment": {
                    "status": "PASS",
                    "method": (
                        "align_carla_route_to_map.prepare_aligned_route"
                    ),
                    "source_frame": "carla_map",
                    "target_frame": "map",
                    "source_route_path": str(source),
                    "source_route_sha256": item["route_sha256"],
                    "aligned_route_path": result["aligned_route"],
                    "aligned_route_sha256": result["aligned_route_sha256"],
                    "map_bundle_profile": bundle_payload["profile"],
                    "map_bundle_metadata_sha256": entry["full_map_bundle"][
                        "metadata_sha256"
                    ],
                    "carla_to_map_transform": transform,
                    "input_already_aligned": False,
                },
                "lanelet2": {"status": "PASS"},
                "lanelet2_error": None,
                "pointcloud_proximity": {"status": "PASS"},
            }
        )
    destination = (
        output_root / "maps/town_fixture/campaign_route_map_preflight.json"
    )
    helper_path = ROOT / "scripts/e2e/align_carla_route_to_map.py"
    write_json(
        destination,
        {
            "schema_version": 1,
            "status": "PASS",
            "matrix_id": plan["matrix_id"],
            "runtime_profile_selector": "speed_30kph",
            "runtime_profile_id": plan["runtime_profile"]["id"],
            "admission_contract_sha256": plan["admission_contract_sha256"],
            "campaign_execution_contract_sha256": plan[
                "campaign_execution_contract_sha256"
            ],
            "map_id": entry["map_id"],
            "canonical_name": entry["canonical_name"],
            "map_bundle_path": entry["full_map_bundle"]["path"],
            "map_bundle_metadata_sha256": entry["full_map_bundle"][
                "metadata_sha256"
            ],
            "map_bundle_file_sha256": entry["full_map_bundle"][
                "bundle_file_sha256"
            ],
            "route_alignment_contract": {
                "schema_version": 1,
                "method": "align_carla_route_to_map.prepare_aligned_route",
                "source_frame": "carla_map",
                "target_frame": "map",
                "raw_route_identity_preserved": True,
                "validator_input": "aligned_route",
                "map_bundle_profile": bundle_payload["profile"],
                "carla_to_map_transform": transform,
                "helper_path": str(helper_path.resolve()),
                "helper_sha256": matrix.sha256_file(helper_path),
            },
            "route_generation_contracts_sha256": matrix.sha256_json(
                entry["route_generation_contracts"]
            ),
            "cases": cases,
        },
    )
    return {
        "path": str(destination),
        "sha256": matrix.sha256_file(destination),
        "status": "PASS",
        "cases": cases,
    }


def make_speed_route_catalogs(
    output_root: Path,
    alignment_z_m: float = 0.0,
) -> tuple[Path, dict, Path, Path]:
    bundle = output_root / "bundle/map_bundle.json"
    write_json(
        bundle,
        {
            "schema_version": 1,
            "profile": "town_fixture",
            "canonical_carla_map": "/Game/Carla/Maps/TownFixture",
            "carla_to_map_transform": {
                "x_m": 0.0,
                "y_m": 0.0,
                "z_m": alignment_z_m,
                "yaw_rad": 0.0,
            },
        },
    )
    for filename, contents in (
        ("lanelet2_map.osm", "<osm version='0.6'/>\n"),
        ("pointcloud_map.pcd", "VERSION 0.7\nDATA ascii\n"),
        ("map_projector_info.yaml", "projector_type: Local\n"),
    ):
        (bundle.parent / filename).write_text(contents, encoding="utf-8")
    manifest, _ = matrix.load_matrix(MATRIX_PATH)
    profile = dict(matrix.select_runtime_profile(manifest, "speed_30kph"))
    synthetic_plan(
        output_root,
        bundle,
        runtime_profile=profile,
        runtime_profile_selector="speed_30kph",
    )
    plan = json.loads((output_root / "matrix_plan.json").read_text(encoding="utf-8"))
    contracts = plan["route_generation_contracts"]
    straight_catalog = make_catalog(
        output_root,
        generation_contract=contracts["straight"],
        catalog_id="straight",
    )
    turn_catalog = make_catalog(
        output_root,
        generation_contract=contracts["turn"],
        catalog_id="turn",
    )
    return bundle, profile, straight_catalog, turn_catalog


def make_speed_custom_route_catalogs(
    output_root: Path,
) -> tuple[Path, dict, Path, Path]:
    bundle, profile, straight_catalog, turn_catalog = make_speed_route_catalogs(
        output_root
    )
    plan_path = output_root / "matrix_plan.json"
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    plan["maps"][0]["full_map_bundle"]["bundle_schema"] = "custom_map"
    plan["admission_contract_sha256"] = matrix.sha256_json(
        matrix._admission_contract_payload(plan)
    )
    write_json(plan_path, plan)

    for trial_id, catalog_path in (
        ("straight", straight_catalog),
        ("turn", turn_catalog),
    ):
        catalog = json.loads(catalog_path.read_text(encoding="utf-8"))
        generation = catalog["generation"]
        generation["endpoint_source"] = "spawn_points"
        generation["endpoint_waypoint_spacing_m"] = None
        generation.pop("physical_straight_contract", None)
        generation.pop("physical_turn_contract", None)
        generation["initial_approach_contract"] = dict(
            matrix.CUSTOM_MAP_INITIAL_APPROACH_CONTRACT
        )
        generation["turn_geometry_contract"] = dict(
            matrix.CUSTOM_MAP_TURN_GEOMETRY_CONTRACT
        )
        for route_entry in catalog["routes"]:
            route_path = catalog_path.parent / route_entry["path"]
            payload = json.loads(route_path.read_text(encoding="utf-8"))
            if trial_id == "straight":
                payload.pop("endpoint_source", None)
                payload.pop("endpoint_waypoint_spacing_m", None)
                payload.pop("start_endpoint_index", None)
                payload.pop("goal_endpoint_index", None)
                payload.pop("spawn_height_contract", None)
                payload["start_spawn_index"] = 1
                payload["goal_spawn_index"] = 2
                attach_goal_endpoint_provenance(payload)
            initial = matrix._custom_route_initial_approach(payload)
            payload["initial_approach_preflight"] = initial
            route_entry["initial_approach_preflight"] = initial
            if trial_id == "turn":
                custom_turn = matrix._custom_route_turn_geometry(
                    payload, str(route_entry["scenario"])
                )
                payload["turn_geometry_preflight"] = custom_turn
                route_entry["turn_geometry_preflight"] = custom_turn
                payload.pop("physical_turn_preflight", None)
                route_entry.pop("physical_turn_preflight", None)
            write_json(route_path, payload)
            route_entry["sha256"] = matrix.sha256_file(route_path)
        write_json(catalog_path, catalog)
    return bundle, profile, straight_catalog, turn_catalog


def make_speed_30kph_trial(
    output_root: Path, trial_index: int
) -> tuple[Path, dict, dict]:
    bundle, profile, straight_catalog, turn_catalog = make_speed_route_catalogs(
        output_root
    )
    with patch.object(
        matrix,
        "_campaign_route_map_preflight",
        fake_campaign_route_map_preflight,
    ):
        route_matrix = matrix.select_routes(
            output_root,
            "town_fixture",
            straight_catalog_path=straight_catalog,
            turn_catalog_path=turn_catalog,
        )
    route_entry = route_matrix["trials"][trial_index]
    trial, _, result = make_trial_evidence(output_root, bundle, route_entry)
    add_speed_30kph_evidence(
        trial, profile, route_entry["catalog_scenario"], result
    )
    add_speed_30kph_visual_evidence(trial)
    add_carla_lifecycle_evidence(trial, route_entry["trial_id"])
    return trial, route_entry, profile


def test_real_matrix_admits_only_freshly_preflighted_full_map_bundles() -> None:
    manifest, path = matrix.load_matrix(MATRIX_PATH)
    plan = matrix.build_campaign_plan(manifest, path)

    assert plan["canonical_map_count"] == 19
    assert plan["runnable_map_count"] == 9
    assert plan["runtime_profile_selector"] == "recommended"
    assert len(plan["admission_contract_sha256"]) == 64
    by_id = {entry["map_id"]: entry for entry in plan["maps"]}
    assert by_id["c_track_1_0_7"]["runnable"] is True
    assert by_id["c_track_1_0_7"]["full_map_bundle"]["road_lanelets"] == 395
    assert set(
        by_id["c_track_1_0_7"]["full_map_bundle"]["bundle_file_sha256"]
    ) == set(matrix.BUNDLE_FILE_NAMES)
    assert by_id["town01"]["status"] == "PENDING"
    assert by_id["town01"]["full_map_bundle"]["readiness"]["status"] == (
        "TEST_ROUTES_MAP_PREFLIGHT_PASS"
    )
    assert by_id["town03"]["full_map_bundle"]["readiness"][
        "alignment_status"
    ] == "PASS_WITH_OUTLIERS"
    assert "not yet a VAD PASS" in by_id["town03"]["reason"]
    assert by_id["town10hd_opt"]["status"] == "PENDING"
    assert by_id["town10hd_opt"]["full_map_bundle"]["pcd_points"] == 6818935
    assert by_id["town10hd_opt"]["full_map_bundle"]["fresh_source_sha256"][
        "pointcloud_map"
    ] == "1f02775a8d18a5a7d566f80a2db0108806201b5d1739daee7c9bab9f6aa67641"
    assert by_id["town10hd_opt"]["full_map_bundle"]["readiness"][
        "alignment_status"
    ] == "PASS"
    assert {
        item["scenario"]
        for item in by_id["town10hd_opt"]["full_map_bundle"]["readiness"][
            "admitted_routes"
        ]
    }.issuperset({"straight", "left", "right"})


def test_campaign_plan_fail_closed_binds_execution_code_sha256() -> None:
    manifest, path = matrix.load_matrix(MATRIX_PATH)
    plan = matrix.build_campaign_plan(manifest, path)

    contract = plan["campaign_execution_contract"]
    files = {item["path"]: item["sha256"] for item in contract["files"]}
    required = {
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
        "scripts/e2e/run_recorded_route_trial.sh",
        "scripts/e2e/route_test.py",
        "scripts/e2e/align_carla_route_to_map.py",
        "scripts/e2e/validate_route_map.py",
        "scripts/e2e/analyze_speed_profile.py",
        "autoware_e2e_vad_launch/config/sensor_mapping_vad_fast_reliable_imu.yaml",
        "autoware_e2e_vad_launch/config/sensor_mapping_vad_fast_reliable_imu_camera_source_5hz.yaml",
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
    }
    assert required.issubset(files)
    assert set(files) == set(matrix.CAMPAIGN_EXECUTION_CONTRACT_PATHS)
    for relative, expected_sha256 in files.items():
        assert expected_sha256 == matrix.sha256_file(ROOT / relative)
    assert plan["campaign_execution_contract_sha256"] == matrix.sha256_json(
        contract
    )
    assert plan["admission_contract_sha256"] == matrix.sha256_json(
        matrix._admission_contract_payload(plan)
    )
    matrix._verify_campaign_plan(plan, "test plan")


def test_speed_campaign_plan_digest_binds_longitudinal_grade_contract() -> None:
    manifest, path = matrix.load_matrix(MATRIX_PATH)
    plan = matrix.build_campaign_plan(manifest, path, "speed_30kph")
    grade_contract = plan["runtime_profile"]["speed_contract"]["trials"][
        "turn"
    ]["physical_geometry"]

    assert grade_contract["longitudinal_grade_window_m"] == pytest.approx(5.0)
    assert grade_contract["maximum_absolute_grade_ratio"] == pytest.approx(
        math.sin(0.1)
    )
    assert grade_contract["controller_maximum_output_mps2"] == pytest.approx(
        1.5
    )

    mutated = json.loads(json.dumps(plan))
    mutated["runtime_profile"]["speed_contract"]["trials"]["turn"][
        "physical_geometry"
    ]["maximum_absolute_grade_ratio"] = 0.36
    assert matrix.sha256_json(
        matrix._admission_contract_payload(mutated)
    ) != plan["admission_contract_sha256"]
    with pytest.raises(matrix.MatrixError, match="admission contract digest"):
        matrix._verify_campaign_plan(mutated, "mutated grade plan")


def test_execution_code_contract_rejects_omission_change_and_missing_file(
    tmp_path: Path,
) -> None:
    first = tmp_path / "first.py"
    second = tmp_path / "second.sh"
    first.write_text("print('first')\n", encoding="utf-8")
    second.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    paths = ("first.py", "second.sh")
    with patch.object(matrix, "ROOT", tmp_path), patch.object(
        matrix, "CAMPAIGN_EXECUTION_CONTRACT_PATHS", paths
    ):
        admitted = matrix._campaign_execution_contract()
        plan = {
            "campaign_execution_contract": admitted,
            "campaign_execution_contract_sha256": matrix.sha256_json(admitted),
        }
        matrix._verify_campaign_execution_contract(plan, "fixture")

        omitted = json.loads(json.dumps(admitted))
        omitted["files"].pop()
        omitted_plan = {
            "campaign_execution_contract": omitted,
            "campaign_execution_contract_sha256": matrix.sha256_json(omitted),
        }
        with pytest.raises(matrix.MatrixError, match="allow-list differs"):
            matrix._verify_campaign_execution_contract(
                omitted_plan, "omitted fixture"
            )

        first.write_text("print('changed')\n", encoding="utf-8")
        with pytest.raises(matrix.MatrixError, match="SHA256 changed: first.py"):
            matrix._verify_campaign_execution_contract(plan, "changed fixture")

    with patch.object(matrix, "ROOT", tmp_path), patch.object(
        matrix,
        "CAMPAIGN_EXECUTION_CONTRACT_PATHS",
        ("first.py", "missing.py"),
    ):
        with pytest.raises(matrix.MatrixError, match="file is missing: missing.py"):
            matrix._campaign_execution_contract()


def test_prepare_carla_route_execution_contract_rejects_source_drift(
    tmp_path: Path,
) -> None:
    relative = "scripts/e2e/prepare_carla_route.py"
    helper = tmp_path / relative
    helper.parent.mkdir(parents=True)
    helper.write_text("VALUE = 1\n", encoding="utf-8")
    with patch.object(matrix, "ROOT", tmp_path), patch.object(
        matrix, "CAMPAIGN_EXECUTION_CONTRACT_PATHS", (relative,)
    ):
        admitted = matrix._campaign_execution_contract()
        plan = {
            "campaign_execution_contract": admitted,
            "campaign_execution_contract_sha256": matrix.sha256_json(admitted),
        }
        helper.write_text("VALUE = 2\n", encoding="utf-8")
        with pytest.raises(
            matrix.MatrixError,
            match=r"execution-code SHA256 changed: scripts/e2e/prepare_carla_route.py",
        ):
            matrix._verify_campaign_execution_contract(
                plan, "prepare_carla_route drift fixture"
            )


def test_map_route_selection_rejects_execution_code_drift(
    tmp_path: Path,
) -> None:
    plan = synthetic_plan(tmp_path)
    catalog = make_catalog(tmp_path)
    plan["campaign_execution_contract"]["files"][0]["sha256"] = "f" * 64
    plan["campaign_execution_contract_sha256"] = matrix.sha256_json(
        plan["campaign_execution_contract"]
    )
    plan["admission_contract_sha256"] = matrix.sha256_json(
        matrix._admission_contract_payload(plan)
    )
    write_json(tmp_path / "matrix_plan.json", plan)

    with pytest.raises(matrix.MatrixError, match="execution-code SHA256 changed"):
        matrix.select_routes(tmp_path, "town_fixture", catalog)


def test_resume_rejects_execution_code_drift_in_existing_plan(
    tmp_path: Path,
) -> None:
    matrix.prepare_output(MATRIX_PATH, tmp_path, False)
    plan_path = tmp_path / "matrix_plan.json"
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    plan["campaign_execution_contract"]["files"][0]["sha256"] = "e" * 64
    plan["campaign_execution_contract_sha256"] = matrix.sha256_json(
        plan["campaign_execution_contract"]
    )
    plan["admission_contract_sha256"] = matrix.sha256_json(
        matrix._admission_contract_payload(plan)
    )
    write_json(plan_path, plan)

    with pytest.raises(matrix.MatrixError, match="execution-code SHA256 changed"):
        matrix.prepare_output(MATRIX_PATH, tmp_path, True)


def test_speed_30kph_plan_is_opt_in_and_preserves_the_default_profile() -> None:
    manifest, path = matrix.load_matrix(MATRIX_PATH)
    baseline = matrix.build_campaign_plan(manifest, path)
    speed = matrix.build_campaign_plan(manifest, path, "speed_30kph")

    assert baseline["runtime_profile_selector"] == "recommended"
    assert baseline["runtime_profile"]["wrapper_options"] == [
        "--recommended",
        "--visualize",
        "--capture-desktop",
    ]
    assert speed["runtime_profile_selector"] == "speed_30kph"
    assert baseline["route_generation_contracts"]["straight"] == baseline[
        "route_generation_contracts"
    ]["turn"]
    assert baseline["route_generation_contracts"]["straight"] == {
        "weather": "ClearNoon",
        "seeds": [0],
        "pairs_per_seed": 1,
        "minimum_distance_m": 20.0,
        "maximum_distance_m": 120.0,
        "preferred_distance_m": 60.0,
        "sampling_resolution_m": 1.0,
        "maximum_endpoint_offset_m": 2.0,
        "maximum_traces_per_scenario": 5000,
    }
    assert speed["route_generation_contracts"]["straight"] == {
        "weather": "ClearNoon",
        "seeds": [0],
        "pairs_per_seed": 8,
        "minimum_distance_m": 170.0,
        "maximum_distance_m": 260.0,
        "preferred_distance_m": 210.0,
        "sampling_resolution_m": 1.0,
        "maximum_endpoint_offset_m": 2.0,
        "maximum_traces_per_scenario": 20000,
    }
    assert speed["route_generation_contracts"]["turn"] == {
        "weather": "ClearNoon",
        "seeds": [0],
        "pairs_per_seed": 8,
        "minimum_distance_m": 20.0,
        "maximum_distance_m": 120.0,
        "preferred_distance_m": 60.0,
        "sampling_resolution_m": 1.0,
        "maximum_endpoint_offset_m": 2.0,
        "maximum_traces_per_scenario": 20000,
    }
    assert speed["runtime_profile"]["wrapper_options"] == [
        "--recommended",
        "--speed-30kph",
        "--visualize",
        "--capture-desktop",
    ]
    assert speed["turn_route_selection_policy"] == (
        matrix.SPEED_30KPH_TURN_ROUTE_SELECTION_POLICY
    )
    assert speed["turn_route_selection_policy_sha256"] == matrix.sha256_json(
        matrix.SPEED_30KPH_TURN_ROUTE_SELECTION_POLICY
    )
    assert baseline["turn_route_selection_policy"] is None
    assert baseline["turn_route_selection_policy_sha256"] is None
    speed_contract = speed["runtime_profile"]["speed_contract"]
    assert speed_contract["profile_id"] == "carla_vad_30kph_v2"
    assert speed_contract["longitudinal_speed_source"] == (
        "explicit_simulation_profile"
    )
    assert speed_contract["vad_geometry_source"] is True
    assert speed_contract["maximum_speed_sample_gap_sec"] == pytest.approx(0.25)
    assert speed_contract["route_manager_parameters"][
        "longitudinal_velocity_source"
    ] == "explicit_simulation_nominal"
    assert speed_contract["route_manager_parameters"][
        "nominal_cruise_speed_mps"
    ] == pytest.approx(8.333333333333334)
    assert speed_contract["longitudinal_controller"][
        "maximum_output_mps2"
    ] == pytest.approx(1.5)
    assert speed_contract["longitudinal_controller"][
        "maximum_proportional_effort_mps2"
    ] == pytest.approx(1.5)
    assert speed["runtime_profile"]["speed_contract"]["trials"]["straight"] == {
        "exposure_mode": "straight_target_required",
        "minimum_sustained_speed_mps": 7.5,
        "minimum_sustained_speed_sec": 1.0,
        "maximum_lateral_acceleration_mps2": 1.8,
        "physical_geometry": dict(
            matrix.SPEED_30KPH_STRAIGHT_GEOMETRY_CONTRACT
        ),
    }
    assert speed["runtime_profile"]["speed_contract"]["trials"]["turn"] == {
        "exposure_mode": "curvature_limited_turn",
        "minimum_sustained_speed_mps": 0.0,
        "minimum_sustained_speed_sec": 0.0,
        "maximum_lateral_acceleration_p95_mps2": 1.5,
        "maximum_lateral_acceleration_mps2": 1.8,
        "require_selected_turn_command": True,
        "physical_geometry": dict(
            matrix.SPEED_30KPH_TURN_GEOMETRY_CONTRACT
        ),
    }
    assert speed["admission_contract_sha256"] != baseline[
        "admission_contract_sha256"
    ]


def test_camera_source_5hz_plan_changes_only_profile_identity_and_mapping_option() -> None:
    manifest, path = matrix.load_matrix(MATRIX_PATH)
    baseline = matrix.build_campaign_plan(manifest, path, "speed_30kph")
    candidate = matrix.build_campaign_plan(
        manifest, path, "speed_30kph_camera_source_5hz"
    )

    assert candidate["route_contract"] == baseline["route_contract"]
    assert candidate["route_generation_contracts"] == baseline[
        "route_generation_contracts"
    ]
    assert candidate["route_selection_overrides"] == baseline[
        "route_selection_overrides"
    ]
    assert candidate["turn_route_selection_policy"] == baseline[
        "turn_route_selection_policy"
    ]
    baseline_profile = deepcopy(baseline["runtime_profile"])
    candidate_profile = deepcopy(candidate["runtime_profile"])
    assert candidate_profile.pop("camera_source_contract") == (
        matrix.CAMERA_SOURCE_5HZ_CONTRACT
    )
    baseline_profile.pop("id")
    candidate_profile.pop("id")
    baseline_profile.pop("wrapper_options")
    candidate_profile.pop("wrapper_options")
    assert candidate_profile == baseline_profile
    assert candidate["runtime_profile"]["wrapper_options"] == [
        "--recommended",
        "--speed-30kph",
        "--camera-source-5hz",
        "--visualize",
        "--capture-desktop",
    ]


def make_camera_source_5hz_evidence(
    tmp_path: Path, *, superseded: int = 0, recorder_wall_sec: float = 1001.0
) -> tuple[Path, dict[str, str], dict, dict]:
    candidate_path = ROOT / (
        "autoware_e2e_vad_launch/config/"
        "sensor_mapping_vad_fast_reliable_imu_camera_source_5hz.yaml"
    )
    provenance = tmp_path / "sensor_mapping_provenance"
    provenance.mkdir(parents=True)
    recorded = provenance / "sensor_mapping.yaml"
    recorded.write_bytes(candidate_path.read_bytes())
    digest = matrix.sha256_file(recorded)
    (provenance / "SHA256SUMS").write_text(
        f"{digest}  sensor_mapping.yaml\n", encoding="utf-8"
    )
    mapping = yaml.safe_load(recorded.read_text(encoding="utf-8"))
    camera_ids = sorted(
        sensor["id"]
        for sensor in mapping["sensor_mappings"].values()
        if sensor.get("carla_type") == "sensor.camera.rgb"
    )
    event_rates = {
        f"/sensing/camera/{camera_id}/camera_info": {
            "count": 100,
            "stamp_rate_hz": 5.0,
            "stamp_period_sec": {
                "available": True,
                "count": 99,
                "mean": 0.2,
                "median": 0.2,
                "max": 0.2,
            },
        }
        for camera_id in camera_ids
    }
    latency = {
        "event_rates": event_rates,
        "camera_bundle": {
            "available": True,
            "camera_count": 6,
            "bundle_coverage_percent": 100.0,
        },
        "candidate_front_acceptance": {
            "available": True,
            "candidate_count": 100,
            "front_count": 100,
            "acceptance_percent": 100.0,
        },
    }
    received_offset = 1 if superseded else 0
    (tmp_path / "stack.log").write_text(
        "\n".join(
            (
                "[INFO 1000.000] VAD frame queued: source_stamp_ns=1 "
                f"assembled=1 capacity_pruned=0 superseded={superseded} "
                "mailbox_submitted=1 coalesced_drops=0 "
                f"received_images_min={1 + received_offset} "
                f"received_images_max={1 + received_offset}",
                "[INFO 1000.010] VAD inference complete: source_stamp_ns=1 "
                "inference_ms=30.0 published=true published_count=1 "
                "mailbox_taken=1 coalesced_drops=0",
                "[INFO 1000.200] VAD frame queued: source_stamp_ns=2 "
                f"assembled=2 capacity_pruned=0 superseded={superseded} "
                "mailbox_submitted=2 coalesced_drops=0 "
                f"received_images_min={2 + received_offset} "
                f"received_images_max={2 + received_offset}",
                "[INFO 1000.210] VAD inference complete: source_stamp_ns=2 "
                "inference_ms=30.0 published=true published_count=2 "
                "mailbox_taken=2 coalesced_drops=0",
                "",
            )
        ),
        encoding="utf-8",
    )
    (tmp_path / "recorder.log").write_text(
        f"[INFO {recorder_wall_sec:.3f}] Recording...\n", encoding="utf-8"
    )
    runtime = {
        "CAMERA_SOURCE_5HZ": "true",
        "CAMERA_SOURCE_SENSOR_TICK_SEC": "0.2",
        "CAMERA_ROS_PUBLISH_HZ": "5.0",
        "SENSOR_MAPPING_FILE": f"/installed/config/{candidate_path.name}",
        "SENSOR_MAPPING_SHA256": digest,
    }
    profile = {"camera_source_contract": dict(matrix.CAMERA_SOURCE_5HZ_CONTRACT)}
    return tmp_path, runtime, profile, latency


def test_camera_source_5hz_evidence_requires_continuous_six_camera_stamps(
    tmp_path: Path,
) -> None:
    trial, runtime, profile, latency = make_camera_source_5hz_evidence(tmp_path)
    evidence = matrix._camera_source_5hz_evidence(
        trial, runtime, profile, latency
    )

    assert evidence is not None
    assert evidence["maximum_camera_stamp_gap_sec"] == pytest.approx(0.2)
    assert evidence["vad_inference"]["superseded_classification"] == "none"

    front = latency["event_rates"]["/sensing/camera/CAM_FRONT/camera_info"]
    front["stamp_period_sec"]["max"] = 0.4
    with pytest.raises(matrix.MatrixError, match="maximum stamp gap exceeds"):
        matrix._camera_source_5hz_evidence(trial, runtime, profile, latency)


def test_camera_source_5hz_allows_only_proven_startup_supersession(
    tmp_path: Path,
) -> None:
    trial, runtime, profile, latency = make_camera_source_5hz_evidence(
        tmp_path, superseded=1
    )
    evidence = matrix._camera_source_5hz_evidence(
        trial, runtime, profile, latency
    )

    assert evidence is not None
    assert evidence["vad_inference"]["superseded"] == 1
    assert evidence["vad_inference"]["superseded_classification"] == (
        "startup_before_recorder"
    )

    (trial / "recorder.log").write_text(
        "[INFO 999.000] Recording...\n", encoding="utf-8"
    )
    with pytest.raises(matrix.MatrixError, match="loss or coalescing"):
        matrix._camera_source_5hz_evidence(trial, runtime, profile, latency)


def test_speed_route_selection_overrides_are_map_scoped_and_digest_bound() -> None:
    manifest, path = matrix.load_matrix(MATRIX_PATH)
    baseline = matrix.build_campaign_plan(manifest, path)
    speed = matrix.build_campaign_plan(manifest, path, "speed_30kph")
    expected = matrix.SPEED_30KPH_MAP_TRIAL_ROUTE_SELECTION_OVERRIDES
    speed_by_id = {entry["map_id"]: entry for entry in speed["maps"]}

    assert speed["route_selection_policy"] == matrix.ROUTE_SELECTION_POLICY
    assert speed["route_selection_overrides"] == expected
    assert speed["route_selection_overrides_sha256"] == matrix.sha256_json(
        expected
    )
    assert speed_by_id["town03"]["route_selection_overrides"] == {
        "straight": expected["town03"]["straight"]
    }
    assert speed_by_id["c_track_1_0_7"]["route_selection_overrides"] == {}
    assert all(
        entry["route_selection_overrides"] == {}
        for map_id, entry in speed_by_id.items()
        if map_id not in expected
    )
    assert baseline["route_selection_overrides"] == {}
    assert all(entry["route_selection_overrides"] == {} for entry in baseline["maps"])

    drifted = json.loads(json.dumps(speed))
    drifted_entry = next(
        entry for entry in drifted["maps"] if entry["map_id"] == "town03"
    )
    drifted_entry["route_selection_overrides"]["straight"]["pair_index"] = 0
    with pytest.raises(matrix.MatrixError, match="admission contract digest"):
        matrix._verify_campaign_plan(drifted, "drifted route selection")

    drifted["admission_contract_sha256"] = matrix.sha256_json(
        matrix._admission_contract_payload(drifted)
    )
    with pytest.raises(matrix.MatrixError, match="map/profile contract"):
        matrix._plan_map_route_selection_overrides(drifted, drifted_entry)


def test_town01_v4_turn_physical_ranking_selects_left_pair_three() -> None:
    fixture, vectors = turn_ranking_vectors_from_fixture(
        TOWN01_V4_TURN_RANKING_FIXTURE
    )
    assert fixture["source_catalog_sha256"] == (
        "0f513f77e28420b1f8b5b40d0ab726c7aa69ff91658e280981e09172611b76c4"
    )
    _, selected, selected_vector = vectors[0]
    assert len(vectors) == 16
    assert selected == {
        "id": "town01_left_s0000_p03",
        "scenario": "left",
        "seed": 0,
        "pair_index": 3,
    }
    assert selected_vector["raw_values"][
        "p95_absolute_curvature_per_m"
    ] == pytest.approx(0.09303106247480829)
    assert [item["direction"] for item in selected_vector["sort_components"]] == [
        "descending",
        "descending",
        "ascending",
        "descending",
        "ascending",
        "ascending",
        "ascending",
        "ascending",
    ]


def test_c_track_v4_general_turn_ranking_preserves_left_pair_seven() -> None:
    fixture, vectors = turn_ranking_vectors_from_fixture(
        C_TRACK_V4_TURN_RANKING_FIXTURE
    )
    assert fixture["source_catalog_sha256"] == (
        "2ae80399af0565df8d8d593817c89a365051b916e49105b2b93cac33f5e37e1e"
    )
    common_audit = fixture[
        "common_physical_turn_audit_after_terminal_z_normalization"
    ]
    common_pass = [row for row in common_audit if row[1] == "PASS"]
    _, selected, selected_vector = vectors[0]
    assert len(common_audit) == 16
    assert common_pass == [["c_track_1_0_7_left_s0000_p07", "PASS", []]]
    assert len(vectors) == 1
    assert selected == {
        "id": "c_track_1_0_7_left_s0000_p07",
        "scenario": "left",
        "seed": 0,
        "pair_index": 7,
    }
    assert selected_vector["raw_values"][
        "p95_absolute_curvature_per_m"
    ] == pytest.approx(0.09779359981586193)
    assert selected_vector["raw_values"]["route_lead_field"] == (
        "block_start_distance_m"
    )


def test_c_track_selector_requires_common_physical_pass_without_reindexing(
    tmp_path: Path,
) -> None:
    fixture = json.loads(
        C_TRACK_V4_TURN_RANKING_FIXTURE.read_text(encoding="utf-8")
    )
    catalog_path = tmp_path / "catalog/turn/route_catalog.json"
    candidates = []
    custom_by_id = {}
    for row in fixture["candidates"]:
        route_id, scenario, seed, pair_index, *_ = row
        custom_geometry = {
            "status": "PASS",
            "scenario": scenario,
            "selected_block": {"fixture": route_id},
        }
        payload = {
            "route_id": route_id,
            "turn_geometry_preflight": custom_geometry,
        }
        route_path = catalog_path.parent / "routes" / f"{route_id}.json"
        write_json(route_path, payload)
        candidate = {
            "id": route_id,
            "status": "ready",
            "scenario": scenario,
            "seed": seed,
            "pair_index": pair_index,
            "path": route_path.relative_to(catalog_path.parent).as_posix(),
            "sha256": matrix.sha256_file(route_path),
            "turn_geometry_preflight": custom_geometry,
        }
        candidates.append(candidate)
        custom_by_id[route_id] = custom_geometry
    write_json(catalog_path, {"routes": candidates})

    common_row = fixture["common_pass_ranking_metrics"][0]
    (
        pass_id,
        _scenario,
        _seed,
        _pair_index,
        p95_curvature,
        route_lead,
        command_lead,
        command_tail,
        core_arc,
    ) = common_row
    common_pass = {
        "status": "PASS",
        "selected_block": {
            "p95_absolute_curvature_per_m": p95_curvature,
            "maximum_absolute_curvature_per_m": p95_curvature,
            "route_lead_distance_m": route_lead,
            "command_lead_distance_m": command_lead,
            "command_tail_distance_m": command_tail,
            "command_arc_length_m": core_arc,
        },
        "limits": {"maximum_p95_abs_curvature_per_m": 0.20},
    }

    def custom_geometry(payload, _scenario):
        return custom_by_id[payload["route_id"]]

    def common_geometry(payload, _required):
        route_id = payload["route_id"]
        if route_id == pass_id:
            return common_pass
        message = f"{route_id} common physical-turn fixture rejected"
        raise matrix.PhysicalTurnCandidateRejection(
            message,
            {
                "error_type": "PhysicalTurnCandidateRejection",
                "error_scope": "candidate",
                "error_code": "c_track_v4_common_physical_rejection",
                "fatal": False,
                "message": message,
            },
        )

    manifest, _ = matrix.load_matrix(MATRIX_PATH)
    profile = matrix.select_runtime_profile(manifest, "speed_30kph")
    with patch.object(
        matrix, "_custom_route_turn_geometry", side_effect=custom_geometry
    ), patch.object(
        matrix, "_speed_30kph_turn_geometry", side_effect=common_geometry
    ):
        selected, audit, physical, custom = (
            matrix._rank_speed_30kph_turn_candidates(
                candidates,
                catalog_path,
                profile["speed_contract"],
                matrix.SPEED_30KPH_TURN_ROUTE_SELECTION_POLICY,
                custom_map=True,
            )
        )

    assert selected["id"] == "c_track_1_0_7_left_s0000_p07"
    assert selected["pair_index"] == 7
    assert audit["candidate_count"] == 16
    assert audit["physical_pass_candidate_count"] == 1
    assert audit["physical_rejected_candidate_count"] == 15
    assert audit["candidate_pair_reindexing_allowed"] is False
    assert audit["candidate_generation_quota_modified"] is False
    assert physical == common_pass
    assert custom == custom_by_id[selected["id"]]


def test_speed_turn_ranking_policy_is_plan_digest_bound() -> None:
    manifest, path = matrix.load_matrix(MATRIX_PATH)
    plan = matrix.build_campaign_plan(manifest, path, "speed_30kph")
    drifted = json.loads(json.dumps(plan))
    drifted["turn_route_selection_policy"]["ranking_order"].reverse()
    with pytest.raises(matrix.MatrixError, match="admission contract digest"):
        matrix._verify_campaign_plan(drifted, "drifted turn ranking")

    drifted["turn_route_selection_policy_sha256"] = matrix.sha256_json(
        drifted["turn_route_selection_policy"]
    )
    drifted["admission_contract_sha256"] = matrix.sha256_json(
        matrix._admission_contract_payload(drifted)
    )
    with pytest.raises(matrix.MatrixError, match="selected profile"):
        matrix._plan_turn_route_selection_policy(drifted)


def test_recommended_profile_rejects_route_selection_override_leakage() -> None:
    manifest, _ = matrix.load_matrix(MATRIX_PATH)
    drifted = json.loads(json.dumps(manifest))
    drifted["runtime_profile"]["map_trial_route_selection_overrides"] = {
        "town03": {
            "straight": {
                "scenario": "straight",
                "seed": 0,
                "pair_index": 1,
                "rationale": "traffic_light_head_clearance_screening_v1",
            }
        }
    }

    with pytest.raises(matrix.MatrixError, match="must not define"):
        matrix.select_runtime_profile(drifted, "recommended")


@pytest.mark.parametrize(
    "override",
    [
        {
            "scenario": "straight",
            "seed": 0,
            "pair_index": 2,
            "rationale": "traffic_light_head_clearance_screening_v1",
        },
        {
            "scenario": "left",
            "seed": 0,
            "pair_index": 1,
            "rationale": "traffic_light_head_clearance_screening_v1",
        },
        {
            "scenario": "straight",
            "seed": 1,
            "pair_index": 1,
            "rationale": "traffic_light_head_clearance_screening_v1",
        },
    ],
)
def test_exact_route_selection_override_fails_closed_on_identity_drift(
    override: dict,
) -> None:
    candidates = [
        {
            "id": "town03_straight_s0000_p01",
            "scenario": "straight",
            "seed": 0,
            "pair_index": 1,
        }
    ]

    with pytest.raises(matrix.MatrixError, match="matched 0 candidates"):
        matrix._select_exact_route_override(
            candidates, override, "town03", "straight"
        )


def test_exact_route_selection_override_rejects_duplicate_candidates() -> None:
    override = dict(
        matrix.SPEED_30KPH_MAP_TRIAL_ROUTE_SELECTION_OVERRIDES["town03"][
            "straight"
        ]
    )
    candidate = {
        "id": "town03_straight_s0000_p01",
        "scenario": "straight",
        "seed": 0,
        "pair_index": 1,
    }

    with pytest.raises(matrix.MatrixError, match="matched 2 candidates"):
        matrix._select_exact_route_override(
            [candidate, {**candidate, "id": "duplicate"}],
            override,
            "town03",
            "straight",
        )


def test_town10_compact_straight_override_is_map_scoped_and_digest_bound() -> None:
    manifest, path = matrix.load_matrix(MATRIX_PATH)
    plan = matrix.build_campaign_plan(manifest, path, "speed_30kph")
    by_id = {entry["map_id"]: entry for entry in plan["maps"]}
    global_contracts = plan["route_generation_contracts"]
    town10 = by_id["town10hd_opt"]["route_generation_contracts"]

    assert town10["straight"] == {
        **global_contracts["straight"],
        **matrix.TOWN10HD_OPT_30KPH_STRAIGHT_OVERRIDE,
        "straight_capacity_contract": (
            matrix._town10hd_opt_straight_capacity_contract()
        ),
    }
    assert town10["turn"] == global_contracts["turn"]
    assert by_id["town05_opt"]["route_generation_contracts"] == global_contracts
    assert by_id["town03"]["route_generation_contracts"] == global_contracts
    provenance = town10["straight"]["straight_capacity_contract"]["provenance"]
    assert provenance["derived_required_distance_m"] == pytest.approx(167.534)
    assert provenance["minimum_route_margin_m"] == pytest.approx(2.466)
    assert provenance["validation_threshold_reuse"] is False
    assert provenance["unchanged_verdicts"] == [
        "exact_serialized_physical_straight",
        "30_kph_speed_exposure",
        "AEB",
        "MRM",
    ]

    drifted = json.loads(json.dumps(plan))
    drifted_entry = next(
        item for item in drifted["maps"] if item["map_id"] == "town10hd_opt"
    )
    drifted_entry["route_generation_contracts"]["straight"][
        "maximum_distance_m"
    ] = 183.0
    with pytest.raises(matrix.MatrixError, match="admission contract digest"):
        matrix._verify_campaign_plan(drifted, "drifted Town10 plan")

    drifted["admission_contract_sha256"] = matrix.sha256_json(
        matrix._admission_contract_payload(drifted)
    )
    with pytest.raises(matrix.MatrixError, match="map/profile override"):
        matrix._plan_map_route_generation_contracts(drifted, drifted_entry)


def test_town10_compact_catalog_policy_and_quota_provenance_are_exact() -> None:
    manifest, path = matrix.load_matrix(MATRIX_PATH)
    plan = matrix.build_campaign_plan(manifest, path, "speed_30kph")
    entry = next(item for item in plan["maps"] if item["map_id"] == "town10hd_opt")
    expected = entry["route_generation_contracts"]["straight"]
    generation = {
        "weather": expected["weather"],
        "scenarios": ["straight"],
        "seeds": expected["seeds"],
        "pairs_per_seed": expected["pairs_per_seed"],
        "minimum_distance_m": expected["minimum_distance_m"],
        "maximum_distance_m": expected["maximum_distance_m"],
        "preferred_distance_m": expected["preferred_distance_m"],
        "sampling_resolution_m": expected["sampling_resolution_m"],
        "maximum_endpoint_offset_m": expected["maximum_endpoint_offset_m"],
        "max_traces_per_scenario": expected["maximum_traces_per_scenario"],
        "endpoint_source": "generated_waypoints",
        "endpoint_waypoint_spacing_m": expected["endpoint_waypoint_spacing_m"],
        "endpoint_junction_policy": expected["endpoint_junction_policy"],
        "candidate_enumeration_policy": expected[
            "candidate_enumeration_policy"
        ],
        "straight_capacity_contract": expected["straight_capacity_contract"],
        "endpoint_count": 10,
        "endpoint_api_count": 12,
        "endpoint_eligible_api_count": 10,
        "endpoint_junction_waypoint_count": 2,
        "endpoint_junction_excluded_count": 2,
        "endpoint_duplicate_transform_count": 0,
        "endpoint_deduplication": (
            "exact_full_transform_keep_first_api_occurrence"
        ),
        "endpoint_ordering": (
            "deduplicated_lexicographic_transform_x_y_z_roll_pitch_yaw"
        ),
        "spawn_height_contract": {
            "catalog_z_offset_m": 0.0,
            "bridge_z_offset_m": 2.0,
            "offset_owner": "autoware_carla_interface_bridge",
        },
        "physical_straight_contract": {
            "enabled": True,
            "profile_id": "speed_30kph",
            "measurement_source": "exact_serialized_route_with_terminal_goal",
            "admission_policy": "reject_before_accepted_pair_quota",
            "limits": dict(matrix.SPEED_30KPH_STRAIGHT_GEOMETRY_CONTRACT),
        },
    }
    catalog = {
        "generation": generation,
        "scenario_results": [
            {
                "scenario": "straight",
                "status": "READY",
                "route_count": 8,
                "traces": 19,
                "trace_coverage": {"accepted": 8, "attempted": 19},
                "candidate_enumeration": [
                    {
                        "seed": 0,
                        "policy": "directed_topology_straight_v1",
                        "planar_chord_heading_candidate_count": 288,
                        "directed_reachable_candidate_count": 33,
                        "postfilter_authority": (
                            "exact serialized route distance and "
                            "physical-straight analysis"
                        ),
                    }
                ],
            }
        ],
    }

    matrix._validate_catalog_generation(catalog, expected, "straight")
    matrix._validate_speed_catalog_endpoint_policy(
        catalog, "straight", expected, custom_map=False
    )

    capacity_drift = json.loads(json.dumps(catalog))
    capacity_drift["generation"]["straight_capacity_contract"]["provenance"][
        "minimum_route_margin_m"
    ] = 3.0
    with pytest.raises(matrix.MatrixError, match="straight-capacity contract"):
        matrix._validate_catalog_generation(
            capacity_drift, expected, "straight"
        )

    quota_drift = json.loads(json.dumps(catalog))
    quota_drift["scenario_results"][0]["route_count"] = 7
    with pytest.raises(matrix.MatrixError, match="enumeration provenance"):
        matrix._validate_speed_catalog_endpoint_policy(
            quota_drift, "straight", expected, custom_map=False
        )


def test_route_selection_separates_true_straight_and_turn_commands(tmp_path: Path) -> None:
    synthetic_plan(tmp_path)
    catalog = make_catalog(tmp_path)

    selected = matrix.select_routes(tmp_path, "town_fixture", catalog)

    straight, turn = selected["trials"]
    assert straight["trial_id"] == "straight"
    assert straight["catalog_scenario"] == "straight"
    assert straight["analysis"]["option_counts"]["STRAIGHT"] == 2
    assert straight["analysis"]["vad_command_counts"]["2"] == 2
    assert turn["trial_id"] == "turn"
    assert turn["catalog_scenario"] == "left"
    assert turn["turn_direction"] == "left"
    assert turn["analysis"]["vad_command_counts"]["0"] == 2


def test_speed_route_selection_applies_exact_override_and_records_provenance(
    tmp_path: Path,
) -> None:
    _, _, straight_catalog, turn_catalog = make_speed_route_catalogs(tmp_path)
    override = {
        "scenario": "straight",
        "seed": 0,
        "pair_index": 1,
        "rationale": "traffic_light_head_clearance_screening_v1",
    }
    overrides = {"town_fixture": {"straight": override}}
    plan_path = tmp_path / "matrix_plan.json"
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    plan["runtime_profile"]["map_trial_route_selection_overrides"] = overrides
    plan["route_selection_overrides"] = overrides
    plan["route_selection_overrides_sha256"] = matrix.sha256_json(overrides)
    entry = plan["maps"][0]
    entry["route_selection_overrides"] = overrides["town_fixture"]
    entry["route_selection_overrides_sha256"] = matrix.sha256_json(
        overrides["town_fixture"]
    )
    plan["admission_contract_sha256"] = matrix.sha256_json(
        matrix._admission_contract_payload(plan)
    )
    write_json(plan_path, plan)

    catalog = json.loads(straight_catalog.read_text(encoding="utf-8"))
    pair_one = dict(catalog["routes"][0])
    pair_one.update(
        id="town_fixture_straight_s0000_p01",
        pair_index=1,
    )
    catalog["routes"].append(pair_one)
    write_json(straight_catalog, catalog)

    with patch.object(
        matrix,
        "SPEED_30KPH_MAP_TRIAL_ROUTE_SELECTION_OVERRIDES",
        overrides,
    ), patch.object(
        matrix,
        "_campaign_route_map_preflight",
        fake_campaign_route_map_preflight,
    ):
        route_matrix = matrix.select_routes(
            tmp_path,
            "town_fixture",
            straight_catalog_path=straight_catalog,
            turn_catalog_path=turn_catalog,
        )
        straight = route_matrix["trials"][0]
        assert straight["route_id"] == pair_one["id"]
        assert straight["route_seed"] == 0
        assert straight["route_pair_index"] == 1
        assert straight["route_selection_method"] == matrix.ROUTE_SELECTION_POLICY
        assert straight["route_selection_override"] == override
        assert straight["route_selection_override_sha256"] == matrix.sha256_json(
            override
        )
        assert route_matrix["route_selection_overrides"] == overrides[
            "town_fixture"
        ]
        assert route_matrix["trials"][1]["route_selection_override"] is None
        matrix._validate_route_matrix_provenance(
            tmp_path,
            plan,
            entry,
            route_matrix,
            straight,
            straight["analysis"],
            "straight",
        )

        drifted = json.loads(json.dumps(route_matrix))
        drifted_straight = drifted["trials"][0]
        drifted_straight["route_selection_override"]["pair_index"] = 0
        with pytest.raises(matrix.MatrixError, match="selection provenance"):
            matrix._validate_route_matrix_provenance(
                tmp_path,
                plan,
                entry,
                drifted,
                drifted_straight,
                drifted_straight["analysis"],
                "straight",
            )


def test_route_selection_rejects_a_straight_route_with_turn_command(tmp_path: Path) -> None:
    synthetic_plan(tmp_path)
    catalog = make_catalog(tmp_path, corrupt_straight=True)

    with pytest.raises(matrix.MatrixError, match="straight trial contains"):
        matrix.select_routes(tmp_path, "town_fixture", catalog)


def test_speed_route_selection_requires_separate_scenario_catalogs(
    tmp_path: Path,
) -> None:
    _, _, straight_catalog, _ = make_speed_route_catalogs(tmp_path)

    with pytest.raises(matrix.MatrixError, match="requires separate straight and turn"):
        matrix.select_routes(tmp_path, "town_fixture", straight_catalog)

    with pytest.raises(matrix.MatrixError, match="baseline routes cannot be selected"):
        matrix.select_admitted_routes(tmp_path, "town_fixture")


def test_speed_route_selection_rejects_swapped_scenario_contracts(
    tmp_path: Path,
) -> None:
    _, _, straight_catalog, turn_catalog = make_speed_route_catalogs(tmp_path)

    with pytest.raises(matrix.MatrixError, match="straight route catalog .* mismatch"):
        matrix.select_routes(
            tmp_path,
            "town_fixture",
            straight_catalog_path=turn_catalog,
            turn_catalog_path=straight_catalog,
        )


def test_speed_route_selection_rejects_generation_provenance_drift(
    tmp_path: Path,
) -> None:
    _, _, straight_catalog, turn_catalog = make_speed_route_catalogs(tmp_path)
    catalog = json.loads(straight_catalog.read_text(encoding="utf-8"))
    catalog["generation"]["preferred_distance_m"] = 129.0
    write_json(straight_catalog, catalog)

    with pytest.raises(matrix.MatrixError, match="preferred_distance_m contract mismatch"):
        matrix.select_routes(
            tmp_path,
            "town_fixture",
            straight_catalog_path=straight_catalog,
            turn_catalog_path=turn_catalog,
        )


def test_speed_route_selection_requires_generation_time_straight_gate(
    tmp_path: Path,
) -> None:
    _, _, straight_catalog, turn_catalog = make_speed_route_catalogs(tmp_path)
    catalog = json.loads(straight_catalog.read_text(encoding="utf-8"))
    catalog["generation"].pop("physical_straight_contract")
    write_json(straight_catalog, catalog)

    with pytest.raises(
        matrix.MatrixError, match="physical-geometry generation contract"
    ):
        matrix.select_routes(
            tmp_path,
            "town_fixture",
            straight_catalog_path=straight_catalog,
            turn_catalog_path=turn_catalog,
        )


def test_speed_route_selection_rejects_short_straight_even_with_claimed_contract(
    tmp_path: Path,
) -> None:
    _, _, straight_catalog, turn_catalog = make_speed_route_catalogs(tmp_path)
    catalog = json.loads(straight_catalog.read_text(encoding="utf-8"))
    route_entry = next(
        item for item in catalog["routes"] if item["scenario"] == "straight"
    )
    route_path = straight_catalog.parent / route_entry["path"]
    route = json.loads(route_path.read_text(encoding="utf-8"))
    original_length = route["route_length_m"]
    for point in route["route"]:
        point["distance_m"] *= 60.0 / original_length
    route["route_length_m"] = 60.0
    physical_straight_preflight = matrix._speed_30kph_straight_geometry(
        route,
        matrix.SPEED_30KPH_STRAIGHT_GEOMETRY_CONTRACT,
    )
    route["physical_straight_preflight"] = physical_straight_preflight
    write_json(route_path, route)
    route_entry["sha256"] = matrix.sha256_file(route_path)
    route_entry["physical_straight_preflight"] = physical_straight_preflight
    write_json(straight_catalog, catalog)

    with pytest.raises(matrix.MatrixError, match=r"outside \[170.000, 260.000\]"):
        matrix.select_routes(
            tmp_path,
            "town_fixture",
            straight_catalog_path=straight_catalog,
            turn_catalog_path=turn_catalog,
        )


def test_speed_route_selection_rejects_straight_label_on_curved_geometry(
    tmp_path: Path,
) -> None:
    _, _, straight_catalog, turn_catalog = make_speed_route_catalogs(tmp_path)
    catalog = json.loads(straight_catalog.read_text(encoding="utf-8"))
    route_entry = next(
        item for item in catalog["routes"] if item["scenario"] == "straight"
    )
    route_path = straight_catalog.parent / route_entry["path"]
    route = json.loads(route_path.read_text(encoding="utf-8"))
    coordinates = (
        (0.0, 0.0, 0.0),
        (70.0, 0.0, 0.0),
        (70.0, 70.0, 1.5707963267948966),
        (70.0, 140.0, 1.5707963267948966),
    )
    for point, (x, y, yaw) in zip(route["route"], coordinates):
        point.update(x=x, y=y, yaw=yaw)
    write_json(route_path, route)
    route_entry["sha256"] = matrix.sha256_file(route_path)
    write_json(straight_catalog, catalog)

    with pytest.raises(matrix.MatrixError, match="physical-straight preflight failed"):
        matrix.select_routes(
            tmp_path,
            "town_fixture",
            straight_catalog_path=straight_catalog,
            turn_catalog_path=turn_catalog,
        )


def test_speed_route_selection_excludes_curved_candidate_before_ranking(
    tmp_path: Path,
) -> None:
    _, _, straight_catalog, turn_catalog = make_speed_route_catalogs(tmp_path)
    catalog = json.loads(straight_catalog.read_text(encoding="utf-8"))
    curved_entry = catalog["routes"][0]
    curved_path = straight_catalog.parent / curved_entry["path"]
    curved_payload = json.loads(curved_path.read_text(encoding="utf-8"))
    good_payload = json.loads(json.dumps(curved_payload))

    coordinates = (
        (0.0, 0.0, 0.0),
        (70.0, 0.0, 0.0),
        (70.0, 70.0, 1.5707963267948966),
        (70.0, 140.0, 1.5707963267948966),
    )
    for point, (x, y, yaw) in zip(curved_payload["route"], coordinates):
        point.update(x=x, y=y, yaw=yaw)
    write_json(curved_path, curved_payload)
    curved_entry["sha256"] = matrix.sha256_file(curved_path)

    good_entry = dict(curved_entry)
    good_entry.update(
        id="town_fixture_straight_s0000_p01",
        pair_index=1,
        start_endpoint_index=2,
        goal_endpoint_index=3,
    )
    good_path = curved_path.with_name(f"{good_entry['id']}.json")
    write_json(good_path, good_payload)
    good_entry["path"] = good_path.relative_to(straight_catalog.parent).as_posix()
    good_entry["sha256"] = matrix.sha256_file(good_path)
    catalog["routes"].append(good_entry)
    write_json(straight_catalog, catalog)

    with patch.object(
        matrix,
        "_campaign_route_map_preflight",
        fake_campaign_route_map_preflight,
    ):
        selected = matrix.select_routes(
            tmp_path,
            "town_fixture",
            straight_catalog_path=straight_catalog,
            turn_catalog_path=turn_catalog,
        )

    assert selected["trials"][0]["route_id"] == good_entry["id"]
    assert selected["trials"][0]["analysis"]["physical_straight_preflight"][
        "status"
    ] == "PASS"


def test_speed_route_selection_requires_generation_time_turn_gate(
    tmp_path: Path,
) -> None:
    _, _, straight_catalog, turn_catalog = make_speed_route_catalogs(tmp_path)
    catalog = json.loads(turn_catalog.read_text(encoding="utf-8"))
    catalog["generation"].pop("physical_turn_contract")
    write_json(turn_catalog, catalog)

    with pytest.raises(
        matrix.MatrixError, match="turn speed catalog physical-geometry"
    ):
        matrix.select_routes(
            tmp_path,
            "town_fixture",
            straight_catalog_path=straight_catalog,
            turn_catalog_path=turn_catalog,
        )


def test_physical_turn_analyzer_preserves_candidate_and_fatal_error_scope() -> None:
    payload = physical_turn_route_payload("left")
    candidate_error = matrix.PhysicalTurnGeometryError(
        "candidate geometry",
        error_scope="candidate",
        error_code="candidate_fixture",
    )
    with patch.object(
        matrix,
        "analyze_serialized_physical_turn",
        side_effect=candidate_error,
    ):
        with pytest.raises(
            matrix.PhysicalTurnCandidateRejection,
            match="candidate geometry",
        ) as rejected:
            matrix._speed_30kph_turn_geometry(
                payload, matrix.SPEED_30KPH_TURN_GEOMETRY_CONTRACT
            )
    assert rejected.value.evidence == {
        "error_type": "PhysicalTurnGeometryError",
        "error_scope": "candidate",
        "error_code": "candidate_fixture",
        "fatal": False,
        "message": "candidate geometry",
    }

    fatal_error = matrix.PhysicalTurnGeometryError(
        "nonfinite route",
        error_scope="nonfinite",
        error_code="nonfinite_fixture",
    )
    with patch.object(
        matrix,
        "analyze_serialized_physical_turn",
        side_effect=fatal_error,
    ):
        with pytest.raises(
            matrix.MatrixError,
            match=(
                "fatal 30 kph physical-turn analysis error "
                "scope=nonfinite code=nonfinite_fixture"
            ),
        ) as fatal:
            matrix._speed_30kph_turn_geometry(
                payload, matrix.SPEED_30KPH_TURN_GEOMETRY_CONTRACT
            )
    assert not isinstance(fatal.value, matrix.PhysicalTurnCandidateRejection)


def test_speed_turn_ranking_aborts_on_fatal_candidate_analysis(
    tmp_path: Path,
) -> None:
    _, _, straight_catalog, turn_catalog = make_speed_route_catalogs(tmp_path)
    catalog = json.loads(turn_catalog.read_text(encoding="utf-8"))
    first = catalog["routes"][0]
    first_path = turn_catalog.parent / first["path"]
    payload = json.loads(first_path.read_text(encoding="utf-8"))
    payload["route"][0]["x"] = math.nan
    write_json(first_path, payload)
    first["sha256"] = matrix.sha256_file(first_path)
    write_json(turn_catalog, catalog)

    with pytest.raises(
        matrix.MatrixError,
        match=r"fatal 30 kph physical-turn analysis error scope=nonfinite",
    ):
        matrix.select_routes(
            tmp_path,
            "town_fixture",
            straight_catalog_path=straight_catalog,
            turn_catalog_path=turn_catalog,
        )
    assert not (tmp_path / "maps/town_fixture/route_matrix.json").exists()


def test_speed_turn_ranking_is_catalog_order_invariant_and_digest_bound(
    tmp_path: Path,
) -> None:
    _, _, straight_catalog, turn_catalog = make_speed_route_catalogs(tmp_path)
    with patch.object(
        matrix,
        "_campaign_route_map_preflight",
        fake_campaign_route_map_preflight,
    ):
        first = matrix.select_routes(
            tmp_path,
            "town_fixture",
            straight_catalog_path=straight_catalog,
            turn_catalog_path=turn_catalog,
        )
        catalog = json.loads(turn_catalog.read_text(encoding="utf-8"))
        catalog["routes"].reverse()
        write_json(turn_catalog, catalog)
        second = matrix.select_routes(
            tmp_path,
            "town_fixture",
            straight_catalog_path=straight_catalog,
            turn_catalog_path=turn_catalog,
        )

    first_turn = first["trials"][1]
    second_turn = second["trials"][1]
    first_audit = first_turn["turn_route_selection_audit"]
    second_audit = second_turn["turn_route_selection_audit"]
    assert first_turn["route_id"] == second_turn["route_id"]
    assert first_turn["route_selection_method"] == (
        matrix.SPEED_30KPH_TURN_ROUTE_SELECTION_METHOD
    )
    assert first_audit["candidate_count"] == 2
    assert first_audit["candidate_set_sha256"] == second_audit[
        "candidate_set_sha256"
    ]
    assert first_audit["selected_ranking_vector"] == second_audit[
        "selected_ranking_vector"
    ]
    assert first_audit["map_preflight_fallback_allowed"] is False


def test_speed_custom_turn_records_common_and_custom_physical_truth(
    tmp_path: Path,
) -> None:
    _, _, straight_catalog, turn_catalog = make_speed_custom_route_catalogs(
        tmp_path
    )
    with patch.object(
        matrix,
        "_campaign_route_map_preflight",
        fake_campaign_route_map_preflight,
    ):
        route_matrix = matrix.select_routes(
            tmp_path,
            "town_fixture",
            straight_catalog_path=straight_catalog,
            turn_catalog_path=turn_catalog,
        )

    turn = route_matrix["trials"][1]
    assert turn["analysis"]["physical_turn_preflight"]["status"] == "PASS"
    assert turn["analysis"]["turn_geometry_preflight"]["status"] == "PASS"
    audit = turn["turn_route_selection_audit"]
    assert audit["selected_ranking_vector"][
        "physical_turn_preflight_source"
    ] == "exact_serialized_common_physical_turn"
    assert audit["policy"]["common_physical_turn_ranking_authority"] is True
    assert audit["policy"]["custom_turn_geometry_preflight_role"] == (
        "additional_required_provenance_gate"
    )
    plan = json.loads((tmp_path / "matrix_plan.json").read_text(encoding="utf-8"))
    entry = plan["maps"][0]
    matrix._validate_route_matrix_provenance(
        tmp_path,
        plan,
        entry,
        route_matrix,
        turn,
        turn["analysis"],
        "turn",
    )

    common_drift = json.loads(json.dumps(route_matrix))
    common_turn = common_drift["trials"][1]
    common_turn["analysis"]["physical_turn_preflight"]["longitudinal_grade"][
        "maximum_absolute_grade_ratio"
    ] += 0.01
    with pytest.raises(matrix.MatrixError, match="common physical-turn analysis"):
        matrix._validate_route_matrix_provenance(
            tmp_path,
            plan,
            entry,
            common_drift,
            common_turn,
            common_turn["analysis"],
            "turn",
        )

    custom_drift = json.loads(json.dumps(route_matrix))
    custom_turn = custom_drift["trials"][1]
    custom_turn["analysis"]["turn_geometry_preflight"][
        "directional_block_count"
    ] += 1
    with pytest.raises(matrix.MatrixError, match="custom turn-geometry analysis"):
        matrix._validate_route_matrix_provenance(
            tmp_path,
            plan,
            entry,
            custom_drift,
            custom_turn,
            custom_turn["analysis"],
            "turn",
        )


def test_speed_custom_turn_rejects_raw_waypoint_float_provenance(
    tmp_path: Path,
) -> None:
    _, _, straight_catalog, turn_catalog = make_speed_custom_route_catalogs(
        tmp_path
    )
    catalog = json.loads(turn_catalog.read_text(encoding="utf-8"))
    route_entry = catalog["routes"][0]
    route_path = turn_catalog.parent / route_entry["path"]
    payload = json.loads(route_path.read_text(encoding="utf-8"))

    # This is the same scale of yaw-derived drift observed when C_track was
    # measured from raw CARLA waypoint degrees instead of serialized ROS yaw.
    for saved in (
        route_entry["turn_geometry_preflight"],
        payload["turn_geometry_preflight"],
    ):
        saved["selected_block"]["absolute_net_heading_change_deg"] += 2.0e-5
        saved["directional_blocks"][0][
            "absolute_net_heading_change_deg"
        ] += 2.0e-5
    write_json(route_path, payload)
    route_entry["sha256"] = matrix.sha256_file(route_path)
    write_json(turn_catalog, catalog)

    with patch.object(
        matrix,
        "_campaign_route_map_preflight",
        fake_campaign_route_map_preflight,
    ), pytest.raises(
        matrix.MatrixError,
        match="custom-map turn-geometry provenance does not match fresh analysis",
    ):
        matrix.select_routes(
            tmp_path,
            "town_fixture",
            straight_catalog_path=straight_catalog,
            turn_catalog_path=turn_catalog,
        )


def test_speed_turn_route_matrix_rejects_ranking_vector_tampering(
    tmp_path: Path,
) -> None:
    _, _, straight_catalog, turn_catalog = make_speed_route_catalogs(tmp_path)
    with patch.object(
        matrix,
        "_campaign_route_map_preflight",
        fake_campaign_route_map_preflight,
    ):
        route_matrix = matrix.select_routes(
            tmp_path,
            "town_fixture",
            straight_catalog_path=straight_catalog,
            turn_catalog_path=turn_catalog,
        )
    plan = json.loads((tmp_path / "matrix_plan.json").read_text(encoding="utf-8"))
    entry = plan["maps"][0]
    turn = route_matrix["trials"][1]
    matrix._validate_route_matrix_provenance(
        tmp_path,
        plan,
        entry,
        route_matrix,
        turn,
        turn["analysis"],
        "turn",
    )

    drifted = json.loads(json.dumps(route_matrix))
    drifted_turn = drifted["trials"][1]
    drifted_turn["turn_route_selection_audit"]["candidate_set_sha256"] = (
        "0" * 64
    )
    with pytest.raises(matrix.MatrixError, match="candidate-set digest"):
        matrix._validate_route_matrix_provenance(
            tmp_path,
            plan,
            entry,
            drifted,
            drifted_turn,
            drifted_turn["analysis"],
            "turn",
        )


def test_speed_route_selection_excludes_short_lead_before_audit_ranking(
    tmp_path: Path,
) -> None:
    _, _, straight_catalog, turn_catalog = make_speed_route_catalogs(tmp_path)
    catalog = json.loads(turn_catalog.read_text(encoding="utf-8"))
    short_entry = next(
        item for item in catalog["routes"] if item["scenario"] == "left"
    )
    short_path = turn_catalog.parent / short_entry["path"]
    short_payload = json.loads(short_path.read_text(encoding="utf-8"))
    for point in short_payload["route"][15:30]:
        point["road_option"] = "LEFT"
        point["vad_command"] = matrix.VAD_COMMANDS["LEFT"]
    write_json(short_path, short_payload)
    short_entry["sha256"] = matrix.sha256_file(short_path)
    write_json(turn_catalog, catalog)

    with patch.object(
        matrix,
        "_campaign_route_map_preflight",
        fake_campaign_route_map_preflight,
    ):
        selected = matrix.select_routes(
            tmp_path,
            "town_fixture",
            straight_catalog_path=straight_catalog,
            turn_catalog_path=turn_catalog,
        )

    turn = selected["trials"][1]
    assert turn["catalog_scenario"] == "right"
    assert turn["analysis"]["physical_turn_preflight"]["status"] == "PASS"
    assert turn["analysis"]["physical_turn_preflight"][
        "initial_command_contract"
    ]["required_command"] == "LANEFOLLOW"
    audit = turn["turn_route_selection_audit"]
    assert audit["candidate_count"] == 2
    assert audit["physical_pass_candidate_count"] == 1
    assert audit["physical_rejected_candidate_count"] == 1
    assert audit["rejected_candidates"][0]["identity"]["scenario"] == "left"
    assert "physical-turn" in audit["rejected_candidates"][0]["reason"]
    assert audit["rejected_candidates"][0]["evidence"]["fatal"] is False
    assert audit["rejected_candidates"][0]["evidence"]["error_scope"] == (
        "candidate"
    )


def test_speed_route_selection_excludes_unsupported_grade_before_ranking(
    tmp_path: Path,
) -> None:
    _, _, straight_catalog, turn_catalog = make_speed_route_catalogs(tmp_path)
    catalog = json.loads(turn_catalog.read_text(encoding="utf-8"))
    steep_entry = next(
        item for item in catalog["routes"] if item["scenario"] == "left"
    )
    steep_path = turn_catalog.parent / steep_entry["path"]
    steep_payload = json.loads(steep_path.read_text(encoding="utf-8"))
    for point in steep_payload["route"]:
        point["z"] = (
            max(0.0, min(float(point["distance_m"]) - 5.0, 5.0)) * 0.36
        )
    write_json(steep_path, steep_payload)
    steep_entry["sha256"] = matrix.sha256_file(steep_path)
    write_json(turn_catalog, catalog)

    with patch.object(
        matrix,
        "_campaign_route_map_preflight",
        fake_campaign_route_map_preflight,
    ):
        selected = matrix.select_routes(
            tmp_path,
            "town_fixture",
            straight_catalog_path=straight_catalog,
            turn_catalog_path=turn_catalog,
        )

    turn = selected["trials"][1]
    assert turn["catalog_scenario"] == "right"
    assert turn["analysis"]["physical_turn_preflight"][
        "longitudinal_grade"
    ]["status"] == "PASS"


def test_speed_route_selection_rejects_catalog_grade_provenance_drift(
    tmp_path: Path,
) -> None:
    _, _, straight_catalog, turn_catalog = make_speed_route_catalogs(tmp_path)
    catalog = json.loads(turn_catalog.read_text(encoding="utf-8"))
    catalog["generation"]["physical_turn_contract"]["provenance"][
        "longitudinal_grade_measurement_source"
    ] = "tampered"
    write_json(turn_catalog, catalog)

    with pytest.raises(
        matrix.MatrixError, match="physical-geometry generation contract"
    ):
        matrix.select_routes(
            tmp_path,
            "town_fixture",
            straight_catalog_path=straight_catalog,
            turn_catalog_path=turn_catalog,
        )


def test_speed_route_selection_rejects_turn_preflight_provenance_drift(
    tmp_path: Path,
) -> None:
    _, _, straight_catalog, turn_catalog = make_speed_route_catalogs(tmp_path)
    catalog = json.loads(turn_catalog.read_text(encoding="utf-8"))
    route_entry = catalog["routes"][0]
    route_path = turn_catalog.parent / route_entry["path"]
    payload = json.loads(route_path.read_text(encoding="utf-8"))
    payload["physical_turn_preflight"]["contract_provenance"]["scope"] = (
        "tampered"
    )
    write_json(route_path, payload)
    route_entry["sha256"] = matrix.sha256_file(route_path)
    write_json(turn_catalog, catalog)

    with pytest.raises(
        matrix.MatrixError, match="physical-geometry provenance"
    ):
        matrix.select_routes(
            tmp_path,
            "town_fixture",
            straight_catalog_path=straight_catalog,
            turn_catalog_path=turn_catalog,
        )


def test_speed_route_selection_requires_fresh_campaign_map_preflight(
    tmp_path: Path,
) -> None:
    _, _, straight_catalog, turn_catalog = make_speed_route_catalogs(tmp_path)

    with patch.object(
        matrix,
        "_campaign_route_map_preflight",
        side_effect=matrix.MatrixError("fresh campaign route/map preflight failed"),
    ):
        with pytest.raises(matrix.MatrixError, match="fresh campaign"):
            matrix.select_routes(
                tmp_path,
                "town_fixture",
                straight_catalog_path=straight_catalog,
                turn_catalog_path=turn_catalog,
            )


def test_speed_turn_map_preflight_failure_does_not_fallback_to_next_route(
    tmp_path: Path,
) -> None:
    _, _, straight_catalog, turn_catalog = make_speed_route_catalogs(tmp_path)
    selected_turn_ids = []

    def fail_selected_route_preflight(_output_root, _plan, _entry, selected):
        selected_turn_ids.append(
            next(item["route_id"] for item in selected if item["trial_id"] == "turn")
        )
        raise matrix.MatrixError("selected turn Lanelet2 preflight failed")

    with patch.object(
        matrix,
        "_campaign_route_map_preflight",
        side_effect=fail_selected_route_preflight,
    ):
        with pytest.raises(matrix.MatrixError, match="Lanelet2 preflight failed"):
            matrix.select_routes(
                tmp_path,
                "town_fixture",
                straight_catalog_path=straight_catalog,
                turn_catalog_path=turn_catalog,
            )

    assert selected_turn_ids == ["town_fixture_left_s0000_p00"]
    assert not (tmp_path / "maps/town_fixture/route_matrix.json").exists()


def test_campaign_preflight_rejects_bundle_drift_during_measurement(
    tmp_path: Path,
) -> None:
    _, _, straight_catalog, turn_catalog = make_speed_route_catalogs(tmp_path)
    with patch.object(
        matrix,
        "_campaign_route_map_preflight",
        fake_campaign_route_map_preflight,
    ):
        route_matrix = matrix.select_routes(
            tmp_path,
            "town_fixture",
            straight_catalog_path=straight_catalog,
            turn_catalog_path=turn_catalog,
        )
    plan = json.loads((tmp_path / "matrix_plan.json").read_text(encoding="utf-8"))
    entry = plan["maps"][0]

    class FakeValidator:
        @staticmethod
        def validate_route_map(*_args, **_kwargs):
            return {"status": "PASS"}

    class MutatingPreparer:
        @staticmethod
        def validate_route_pcd_proximity(route_paths, bundle_path):
            pcd = bundle_path / "pointcloud_map.pcd"
            pcd.write_bytes(pcd.read_bytes() + b"drift")
            return {str(path): {"status": "PASS"} for path in route_paths}

    real_aligner = matrix._load_e2e_helper(
        "align_carla_route_to_map.py", "test_bundle_drift_route_alignment"
    )

    def helper(filename: str, _module_name: str):
        if filename == "align_carla_route_to_map.py":
            return real_aligner
        if filename == "validate_route_map.py":
            return FakeValidator
        return MutatingPreparer

    with patch.object(matrix, "_load_e2e_helper", side_effect=helper):
        with pytest.raises(
            matrix.MatrixError,
            match="post-preflight pointcloud_map.pcd SHA256 differs",
        ):
            matrix._campaign_route_map_preflight(
                tmp_path, plan, entry, route_matrix["trials"]
            )


def test_campaign_preflight_validates_map_aligned_routes_and_preserves_raw_identity(
    tmp_path: Path,
) -> None:
    _, _, straight_catalog, turn_catalog = make_speed_route_catalogs(
        tmp_path, alignment_z_m=-15.0
    )
    with patch.object(
        matrix,
        "_campaign_route_map_preflight",
        fake_campaign_route_map_preflight,
    ):
        route_matrix = matrix.select_routes(
            tmp_path,
            "town_fixture",
            straight_catalog_path=straight_catalog,
            turn_catalog_path=turn_catalog,
        )
    plan = json.loads((tmp_path / "matrix_plan.json").read_text(encoding="utf-8"))
    entry = plan["maps"][0]
    observed_lanelet_inputs = []
    observed_pcd_inputs = []

    class CapturingValidator:
        @staticmethod
        def validate_route_map(route_path, _bundle_path, **kwargs):
            payload = json.loads(Path(route_path).read_text(encoding="utf-8"))
            observed_lanelet_inputs.append((Path(route_path), payload, kwargs))
            assert payload["start_ros_pose"]["z"] == pytest.approx(-15.0)
            assert payload["goal_ros_pose"]["z"] == pytest.approx(-15.0)
            assert all(
                point["z"] == pytest.approx(-15.0)
                for point in payload["route"]
            )
            return {"status": "PASS"}

    class CapturingPreparer:
        @staticmethod
        def validate_route_pcd_proximity(route_paths, _bundle_path):
            observed_pcd_inputs.extend(route_paths)
            return {str(path): {"status": "PASS"} for path in route_paths}

    real_aligner = matrix._load_e2e_helper(
        "align_carla_route_to_map.py", "test_c_track_style_route_alignment"
    )

    def helper(filename: str, _module_name: str):
        if filename == "align_carla_route_to_map.py":
            return real_aligner
        if filename == "validate_route_map.py":
            return CapturingValidator
        return CapturingPreparer

    with patch.object(matrix, "_load_e2e_helper", side_effect=helper):
        result = matrix._campaign_route_map_preflight(
            tmp_path, plan, entry, route_matrix["trials"]
        )

    assert result["status"] == "PASS"
    assert len(observed_lanelet_inputs) == 2
    assert len(observed_pcd_inputs) == 2
    for path, _, kwargs in observed_lanelet_inputs:
        assert "campaign_route_map_preflight_routes" in path.parts
        assert kwargs["vertical_tolerance_m"] == pytest.approx(5.0)
    artifact = json.loads(Path(result["path"]).read_text(encoding="utf-8"))
    assert artifact["route_alignment_contract"]["carla_to_map_transform"][
        "z_m"
    ] == pytest.approx(-15.0)
    for item, case in zip(route_matrix["trials"], artifact["cases"]):
        raw = json.loads(Path(item["route_path"]).read_text(encoding="utf-8"))
        aligned = json.loads(
            Path(case["route_alignment"]["aligned_route_path"]).read_text(
                encoding="utf-8"
            )
        )
        assert raw["start_ros_pose"]["z"] == pytest.approx(0.0)
        assert aligned["start_ros_pose"]["z"] == pytest.approx(-15.0)
        assert case["route_path"] == item["route_path"]
        assert case["route_sha256"] == item["route_sha256"]


def test_route_validation_rejects_wrong_vad_command() -> None:
    route = route_payload("left", ["LANEFOLLOW", "LEFT"])
    route["route"][1]["vad_command"] = 2

    with pytest.raises(matrix.MatrixError, match="road-option/VAD-command pair"):
        matrix._validate_route_payload(route, "TownFixture", "left")


def test_route_validation_binds_generated_endpoint_source_and_index() -> None:
    route = route_payload(
        "straight", ["LANEFOLLOW", "STRAIGHT", "LANEFOLLOW"]
    )
    route.pop("start_spawn_index")
    route.pop("goal_spawn_index")
    route.update(
        endpoint_source="generated_waypoints",
        endpoint_waypoint_spacing_m=10.0,
        start_endpoint_index=4,
        goal_endpoint_index=9,
        spawn_height_contract={
            "endpoint_transform_z_m": 0.0,
            "catalog_z_offset_m": 0.0,
            "bridge_z_offset_m": 2.0,
            "effective_actor_spawn_z_m": 2.0,
            "offset_owner": "autoware_carla_interface_bridge",
            "bridge_source": "fixture",
        },
    )
    attach_goal_endpoint_provenance(
        route,
        endpoint_source="generated_waypoints",
        endpoint_index=9,
    )

    analysis = matrix._validate_route_payload(
        route, "TownFixture", "straight"
    )

    assert analysis["goal_endpoint_provenance"]["endpoint_source"] == (
        "generated_waypoints"
    )
    assert analysis["goal_endpoint_provenance"]["endpoint_index"] == 9


@pytest.mark.parametrize(
    "drift",
    (
        "policy",
        "endpoint_source",
        "endpoint_index",
        "original_carla_z",
        "original_ros_z",
        "original_endpoint_z",
        "runtime_goal_carla_z",
        "runtime_goal_ros_z",
        "serialized_terminal_z",
        "route_terminal_z",
        "applied_offset",
        "runtime_goal_x",
    ),
)
def test_route_validation_rejects_goal_endpoint_provenance_drift(
    drift: str,
) -> None:
    route = route_payload(
        "straight", ["LANEFOLLOW", "STRAIGHT", "LANEFOLLOW"]
    )
    provenance = route["goal_endpoint_provenance"]
    normalization = provenance["terminal_z_normalization"]
    if drift == "policy":
        normalization["policy"] = "raw_endpoint_z"
    elif drift == "endpoint_source":
        provenance["endpoint_source"] = "generated_waypoints"
    elif drift == "endpoint_index":
        provenance["endpoint_index"] += 1
    elif drift == "original_carla_z":
        provenance["original_goal_carla_transform"]["z"] += 0.1
    elif drift == "original_ros_z":
        provenance["original_goal_ros_pose"]["z"] += 0.1
    elif drift == "original_endpoint_z":
        normalization["original_endpoint_z_m"] += 0.1
    elif drift == "runtime_goal_carla_z":
        route["goal_carla_transform"]["z"] += 0.1
    elif drift == "runtime_goal_ros_z":
        route["goal_ros_pose"]["z"] += 0.1
    elif drift == "serialized_terminal_z":
        normalization["serialized_terminal_z_m"] += 0.1
    elif drift == "route_terminal_z":
        route["route"][-1]["z"] += 0.1
    elif drift == "applied_offset":
        normalization["applied_offset_m"] += 0.1
    elif drift == "runtime_goal_x":
        route["goal_carla_transform"]["x"] += 0.1

    with pytest.raises(matrix.MatrixError, match="goal"):
        matrix._validate_route_payload(route, "TownFixture", "straight")


def test_custom_route_selection_recomputes_the_15m_approach_preflight(
    tmp_path: Path,
) -> None:
    synthetic_plan(tmp_path, bundle_schema="custom_map")
    catalog = make_custom_catalog(tmp_path)

    selected = matrix.select_routes(tmp_path, "town_fixture", catalog)

    for trial in selected["trials"]:
        approach = trial["analysis"]["initial_approach_preflight"]
        assert approach["status"] == "PASS"
        assert approach["covered_distance_m"] == pytest.approx(15.0)
    assert selected["custom_map_initial_approach_contract"] == (
        matrix.CUSTOM_MAP_INITIAL_APPROACH_CONTRACT
    )
    assert selected["custom_map_turn_geometry_contract"] == (
        matrix.CUSTOM_MAP_TURN_GEOMETRY_CONTRACT
    )
    turn = selected["trials"][1]["analysis"]["turn_geometry_preflight"]
    assert turn["status"] == "PASS"
    assert turn["directional_block_count"] == 1
    assert turn["selected_block"]["absolute_net_heading_change_deg"] == (
        pytest.approx(90.0)
    )


def test_custom_route_selection_rejects_falsely_declared_stable_approach(
    tmp_path: Path,
) -> None:
    synthetic_plan(tmp_path, bundle_schema="custom_map")
    catalog = make_custom_catalog(tmp_path, sharp_straight=True)

    with pytest.raises(matrix.MatrixError, match="initial-approach preflight failed"):
        matrix.select_routes(tmp_path, "town_fixture", catalog)


def test_custom_route_selection_rejects_two_directional_turn_blocks(
    tmp_path: Path,
) -> None:
    synthetic_plan(tmp_path, bundle_schema="custom_map")
    catalog = make_custom_catalog(tmp_path, compound_turn=True)

    with pytest.raises(matrix.MatrixError, match="exactly one contiguous LEFT block"):
        matrix.select_routes(tmp_path, "town_fixture", catalog)


def test_strict_trial_validation_requires_post_candidate_fullscreen_evidence(
    tmp_path: Path,
) -> None:
    bundle = tmp_path / "bundle/map_bundle.json"
    write_json(
        bundle,
        {"canonical_carla_map": "/Game/Carla/Maps/TownFixture"},
    )
    synthetic_plan(tmp_path, bundle)
    catalog = make_catalog(tmp_path)
    route_matrix = matrix.select_routes(tmp_path, "town_fixture", catalog)
    route_entry = route_matrix["trials"][0]
    trial, desktop, _ = make_trial_evidence(tmp_path, bundle, route_entry)

    validation = matrix.validate_trial(
        tmp_path, "town_fixture", "straight", trial
    )
    assert validation["status"] == "PASS"
    assert validation["desktop_capture"]["candidate_observed"] is True

    desktop["capture_started_after_candidate"] = False
    write_json(trial / "desktop_capture.json", desktop)
    with pytest.raises(matrix.MatrixError, match="after a VAD candidate"):
        matrix.validate_trial(tmp_path, "town_fixture", "straight", trial)


@pytest.mark.parametrize(
    ("mutation", "error_match"),
    (
        ("completion_map", "completion health contract mismatch"),
        ("completion_rpc", "completion RPC evidence is invalid"),
        ("cleanup_port", "cleanup did not prove process/port release"),
        ("timestamp_order", "timestamps are not strictly ordered"),
        ("final_log", "final CARLA generation log differs"),
    ),
)
def test_strict_trial_validation_rejects_carla_lifecycle_evidence_drift(
    tmp_path: Path, mutation: str, error_match: str
) -> None:
    bundle = tmp_path / "bundle/map_bundle.json"
    write_json(
        bundle,
        {"canonical_carla_map": "/Game/Carla/Maps/TownFixture"},
    )
    synthetic_plan(tmp_path, bundle)
    route_entry = matrix.select_routes(
        tmp_path, "town_fixture", make_catalog(tmp_path)
    )["trials"][0]
    trial, _, _ = make_trial_evidence(tmp_path, bundle, route_entry)

    if mutation in {"completion_map", "completion_rpc", "timestamp_order"}:
        path = trial / "carla_completion_health.json"
        payload = json.loads(path.read_text(encoding="utf-8"))
        if mutation == "completion_map":
            payload["expected_map"] = "Town02"
        elif mutation == "completion_rpc":
            payload["rpc_sequence"] = ["get_world"]
        else:
            payload["checked_at"] = "2026-09-01T00:00:00.100000+00:00"
        write_json(path, payload)
    elif mutation == "cleanup_port":
        path = trial / "carla_cleanup_health.json"
        payload = json.loads(path.read_text(encoding="utf-8"))
        payload["port_released"] = False
        write_json(path, payload)
    else:
        with (trial / "carla_server.log").open("a", encoding="utf-8") as stream:
            stream.write("late server output\n")

    with pytest.raises(matrix.MatrixError, match=error_match):
        matrix.validate_trial(tmp_path, "town_fixture", "straight", trial)


@pytest.mark.parametrize(("trial_index", "trial_id"), ((0, "straight"), (1, "turn")))
def test_speed_30kph_trial_validates_runtime_parameters_gate_and_result(
    tmp_path: Path, trial_index: int, trial_id: str
) -> None:
    trial, route_entry, _ = make_speed_30kph_trial(tmp_path, trial_index)

    validation = matrix.validate_trial(
        tmp_path, "town_fixture", trial_id, trial
    )

    evidence = validation["speed_contract"]
    assert evidence["status"] == "PASS"
    assert evidence["profile_id"] == "carla_vad_30kph_v2"
    assert evidence["maximum_observed_speed_mps"] == pytest.approx(8.4)
    assert validation["campaign_route_map_preflight"]["status"] == "PASS"
    assert validation["campaign_route_map_preflight"]["route_sha256"] == (
        route_entry["route_sha256"]
    )
    visual = validation["visual_evidence"]
    assert visual["binding"] == "matrix_validation_sha256_v1"
    assert visual["fullscreen_dimensions"] == [1920, 1080]
    assert visual["candidate_dimensions"] == [1920, 1080]
    assert visual["drive_gif_dimensions"] == [960, 540]
    assert visual["vehicle_centered"] is True
    assert set(visual["files"]) == set(matrix.SPEED_30KPH_VISUAL_EVIDENCE_NAMES)
    for name, record in visual["files"].items():
        assert record["sha256"] == matrix.sha256_file(trial / name)
    assert len(evidence["longitudinal_controller_provenance_sha256"]) == 64
    for field in (
        "runtime_env_sha256",
        "speed_profile_json_sha256",
        "speed_profile_plot_sha256",
        "speed_profile_source_identity_sha256",
        "speed_profile_result_sha256",
        "speed_profile_route_sha256",
        "speed_profile_bag_manifest_sha256",
    ):
        assert len(evidence[field]) == 64
    if trial_id == "straight":
        assert route_entry["analysis"]["route_length_m"] == pytest.approx(210.0)
        assert route_entry["analysis"]["physical_straight_preflight"][
            "status"
        ] == "PASS"
        assert route_entry["analysis"]["distance_capacity_tier"] == (
            "nominal_200_plus"
        )
        assert evidence["maximum_sustained_speed_duration_sec"] == pytest.approx(
            1.2
        )
    else:
        expected_command = matrix.VAD_COMMANDS[
            route_entry["catalog_scenario"].upper()
        ]
        assert evidence["selected_turn_command"] == expected_command
        assert evidence["p95_turn_lateral_acceleration_mps2"] == pytest.approx(
            1.4
        )


def test_speed_30kph_trial_rejects_non_full_hd_centered_capture(
    tmp_path: Path,
) -> None:
    trial, _, _ = make_speed_30kph_trial(tmp_path, 0)
    Image.new("RGB", (1280, 720), "navy").save(
        trial / "autoware_rviz_fullscreen.png"
    )
    desktop_path = trial / "desktop_capture.json"
    desktop = json.loads(desktop_path.read_text(encoding="utf-8"))
    desktop["source_dimensions"] = [1280, 720]
    desktop["png_dimensions"] = [1280, 720]
    write_json(desktop_path, desktop)

    with pytest.raises(matrix.MatrixError, match="1920x1080"):
        matrix.validate_trial(tmp_path, "town_fixture", "straight", trial)


def test_speed_30kph_trial_rejects_rosbag_drift_after_speed_analysis(
    tmp_path: Path,
) -> None:
    trial, _, _ = make_speed_30kph_trial(tmp_path, 0)
    (trial / "bag/fixture.db3").write_bytes(b"changed-after-analysis")

    with pytest.raises(matrix.MatrixError, match="rosbag manifest differs"):
        matrix.validate_trial(tmp_path, "town_fixture", "straight", trial)


def test_speed_30kph_trial_rejects_result_swap_after_speed_analysis(
    tmp_path: Path,
) -> None:
    trial, _, _ = make_speed_30kph_trial(tmp_path, 0)
    result_path = trial / "result.json"
    result = json.loads(result_path.read_text(encoding="utf-8"))
    result["unbound_post_analysis_field"] = "drift"
    write_json(result_path, result)

    with pytest.raises(matrix.MatrixError, match="different route/result"):
        matrix.validate_trial(tmp_path, "town_fixture", "straight", trial)


def test_speed_30kph_turn_rejects_lateral_p95_over_1_5_mps2(
    tmp_path: Path,
) -> None:
    trial, _, _ = make_speed_30kph_trial(tmp_path, 1)
    result_path = trial / "result.json"
    result = json.loads(result_path.read_text(encoding="utf-8"))
    result["speed_exposure"]["p95_turn_lateral_acceleration_mps2"] = 1.5001
    result["speed_exposure"]["p95_lateral_acceleration_mps2_by_command"][
        "0"
    ] = 1.5001
    for sample in result["actual_path"]:
        sample["lateral_acceleration_mps2"] = 1.5001
    write_json(result_path, result)

    with pytest.raises(matrix.MatrixError, match="p95 lateral acceleration"):
        matrix.validate_trial(tmp_path, "town_fixture", "turn", trial)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    (
        ("maximum_observed_speed_mps", 9.0001, "maximum observed speed"),
        (
            "maximum_sustained_speed_duration_sec",
            0.999,
            "did not sustain >=7.5 m/s",
        ),
        (
            "maximum_lateral_acceleration_mps2",
            1.8001,
            "maximum lateral acceleration",
        ),
        (
            "maximum_speed_sample_gap_sec",
            0.2501,
            "speed sample gap",
        ),
    ),
)
def test_speed_30kph_straight_rejects_result_threshold_violations(
    tmp_path: Path, field: str, value: float, message: str
) -> None:
    trial, _, _ = make_speed_30kph_trial(tmp_path, 0)
    result_path = trial / "result.json"
    result = json.loads(result_path.read_text(encoding="utf-8"))
    result["metrics"][field] = value
    result["speed_exposure"][field] = value
    write_json(result_path, result)

    with pytest.raises(matrix.MatrixError, match=message):
        matrix.validate_trial(tmp_path, "town_fixture", "straight", trial)


def test_speed_30kph_turn_requires_the_selected_command(tmp_path: Path) -> None:
    trial, _, _ = make_speed_30kph_trial(tmp_path, 1)
    result_path = trial / "result.json"
    result = json.loads(result_path.read_text(encoding="utf-8"))
    fallback = {
        "3": {
            "maximum_observed_speed_mps": 8.4,
            "maximum_sustained_speed_duration_sec": 0.0,
        }
    }
    result["metrics"]["commands_seen"] = [3]
    result["metrics"]["speed_by_command"] = fallback
    result["speed_exposure"]["speed_by_command"] = fallback
    write_json(result_path, result)

    with pytest.raises(matrix.MatrixError, match="no observed selected VAD command"):
        matrix.validate_trial(tmp_path, "town_fixture", "turn", trial)


def test_speed_30kph_rejects_result_profile_context_drift(tmp_path: Path) -> None:
    trial, _, _ = make_speed_30kph_trial(tmp_path, 0)
    result_path = trial / "result.json"
    result = json.loads(result_path.read_text(encoding="utf-8"))
    result["profile_context"]["vad_velocity_evaluated"] = True
    result["speed_exposure"]["vad_velocity_evaluated"] = True
    write_json(result_path, result)

    with pytest.raises(matrix.MatrixError, match="profile_context"):
        matrix.validate_trial(tmp_path, "town_fixture", "straight", trial)


@pytest.mark.parametrize(
    ("field", "replacement"),
    (
        ("SPEED_PROFILE_ID", "carla_vad_30kph_v1"),
        ("LONGITUDINAL_SPEED_SOURCE", "vad_prediction"),
        ("VAD_GEOMETRY_SOURCE", "false"),
        ("MAXIMUM_SPEED_SAMPLE_GAP_SEC", "0.30"),
        ("VAD_ROUTE_MANAGER_OPENBLAS_NUM_THREADS", "24"),
    ),
)
def test_speed_30kph_rejects_runtime_contract_drift(
    tmp_path: Path, field: str, replacement: str
) -> None:
    trial, _, _ = make_speed_30kph_trial(tmp_path, 0)
    runtime_path = trial / "runtime.env"
    lines = runtime_path.read_text(encoding="utf-8").splitlines()
    changed = [
        f"{field}={replacement}" if line.startswith(f"{field}=") else line
        for line in lines
    ]
    runtime_path.write_text("\n".join(changed) + "\n", encoding="utf-8")

    with pytest.raises(matrix.MatrixError, match=f"runtime.env {field}"):
        matrix.validate_trial(tmp_path, "town_fixture", "straight", trial)


def test_speed_30kph_straight_rejects_parameter_dump_drift(
    tmp_path: Path,
) -> None:
    trial, _, _ = make_speed_30kph_trial(tmp_path, 0)
    dump_path = trial / "vad_route_manager.params.yaml"
    dump = yaml.safe_load(dump_path.read_text(encoding="utf-8"))
    dump["/vad_route_manager"]["ros__parameters"]["maximum_speed_mps"] = 8.0
    dump_path.write_text(yaml.safe_dump(dump), encoding="utf-8")

    with pytest.raises(matrix.MatrixError, match="maximum_speed_mps mismatch"):
        matrix.validate_trial(tmp_path, "town_fixture", "straight", trial)


def test_speed_30kph_rejects_mutated_gate_provenance(tmp_path: Path) -> None:
    trial, _, _ = make_speed_30kph_trial(tmp_path, 0)
    gate_path = (
        trial / "speed_profile_provenance/vehicle_cmd_gate.param.yaml"
    )
    gate_path.write_text(
        gate_path.read_text(encoding="utf-8") + "\n# mutation\n",
        encoding="utf-8",
    )

    with pytest.raises(matrix.MatrixError, match="pinned speed contract"):
        matrix.validate_trial(tmp_path, "town_fixture", "straight", trial)


def test_speed_30kph_rejects_disabled_live_gate_limiter(tmp_path: Path) -> None:
    trial, _, _ = make_speed_30kph_trial(tmp_path, 0)
    dump_path = trial / "vehicle_cmd_gate.params.yaml"
    dump = yaml.safe_load(dump_path.read_text(encoding="utf-8"))
    dump["/control/vehicle_cmd_gate"]["ros__parameters"][
        "enable_cmd_limit_filter"
    ] = False
    dump_path.write_text(yaml.safe_dump(dump), encoding="utf-8")

    with pytest.raises(matrix.MatrixError, match="pinned provenance"):
        matrix.validate_trial(tmp_path, "town_fixture", "straight", trial)


def test_speed_30kph_rejects_mutated_campaign_route_preflight(
    tmp_path: Path,
) -> None:
    trial, _, _ = make_speed_30kph_trial(tmp_path, 0)
    artifact = tmp_path / "maps/town_fixture/campaign_route_map_preflight.json"
    value = json.loads(artifact.read_text(encoding="utf-8"))
    value["cases"][0]["pointcloud_proximity"]["status"] = "FAIL"
    write_json(artifact, value)

    with pytest.raises(matrix.MatrixError, match="preflight SHA256 changed"):
        matrix.validate_trial(tmp_path, "town_fixture", "straight", trial)


def test_speed_30kph_rejects_mutated_campaign_aligned_route(
    tmp_path: Path,
) -> None:
    trial, _, _ = make_speed_30kph_trial(tmp_path, 0)
    artifact = json.loads(
        (
            tmp_path
            / "maps/town_fixture/campaign_route_map_preflight.json"
        ).read_text(encoding="utf-8")
    )
    aligned_path = Path(
        artifact["cases"][0]["route_alignment"]["aligned_route_path"]
    )
    aligned_path.write_bytes(aligned_path.read_bytes() + b"\n")

    with pytest.raises(matrix.MatrixError, match="campaign route alignment"):
        matrix.validate_trial(tmp_path, "town_fixture", "straight", trial)


@pytest.mark.parametrize(
    "filename",
    (
        "map_bundle.json",
        "lanelet2_map.osm",
        "pointcloud_map.pcd",
        "map_projector_info.yaml",
    ),
)
def test_speed_30kph_rejects_map_bundle_file_drift_after_preflight(
    tmp_path: Path, filename: str
) -> None:
    trial, _, _ = make_speed_30kph_trial(tmp_path, 0)
    path = tmp_path / "bundle" / filename
    path.write_bytes(path.read_bytes() + b"drift")

    with pytest.raises(matrix.MatrixError, match="SHA256 differs from campaign admission"):
        matrix.validate_trial(tmp_path, "town_fixture", "straight", trial)


def test_speed_30kph_rejects_mutated_longitudinal_controller_provenance(
    tmp_path: Path,
) -> None:
    trial, _, _ = make_speed_30kph_trial(tmp_path, 0)
    controller = (
        trial / "speed_profile_provenance/longitudinal_controller.param.yaml"
    )
    controller.write_text(
        controller.read_text(encoding="utf-8") + "\n# mutation\n",
        encoding="utf-8",
    )

    with pytest.raises(matrix.MatrixError, match="longitudinal-controller config"):
        matrix.validate_trial(tmp_path, "town_fixture", "straight", trial)


def test_speed_30kph_rejects_live_longitudinal_pid_limit_drift(
    tmp_path: Path,
) -> None:
    trial, _, _ = make_speed_30kph_trial(tmp_path, 0)
    dump_path = trial / "controller.params.yaml"
    dump = yaml.safe_load(dump_path.read_text(encoding="utf-8"))
    dump["/control/trajectory_follower/controller_node_exe"]["ros__parameters"][
        "max_out"
    ] = 2.0
    dump_path.write_text(yaml.safe_dump(dump), encoding="utf-8")

    with pytest.raises(matrix.MatrixError, match="pinned provenance"):
        matrix.validate_trial(tmp_path, "town_fixture", "straight", trial)


def test_speed_30kph_rejects_runtime_longitudinal_pid_limit_drift(
    tmp_path: Path,
) -> None:
    trial, _, _ = make_speed_30kph_trial(tmp_path, 0)
    runtime_path = trial / "runtime.env"
    source = runtime_path.read_text(encoding="utf-8")
    runtime_path.write_text(
        source.replace("LONGITUDINAL_PID_MAX_OUT_MPS2=1.5", "LONGITUDINAL_PID_MAX_OUT_MPS2=2.0"),
        encoding="utf-8",
    )

    with pytest.raises(
        matrix.MatrixError, match="runtime.env LONGITUDINAL_PID_MAX_OUT_MPS2"
    ):
        matrix.validate_trial(tmp_path, "town_fixture", "straight", trial)


def test_speed_30kph_rejects_missing_gui_runtime_isolation_provenance(
    tmp_path: Path,
) -> None:
    trial, _, _ = make_speed_30kph_trial(tmp_path, 0)
    runtime_path = trial / "runtime.env"
    source = runtime_path.read_text(encoding="utf-8")
    runtime_path.write_text(
        source.replace("VSCODE_SNAP_GUI_ENV_SANITIZED=false\n", ""),
        encoding="utf-8",
    )

    with pytest.raises(
        matrix.MatrixError, match="VSCODE_SNAP_GUI_ENV_SANITIZED"
    ):
        matrix.validate_trial(tmp_path, "town_fixture", "straight", trial)


def test_speed_aggregate_records_turn_vector_and_complete_candidate_digest(
    tmp_path: Path,
) -> None:
    _, _, straight_catalog, turn_catalog = make_speed_route_catalogs(tmp_path)
    status = {
        "schema_version": 1,
        "matrix_id": "fixture",
        "map_id": "town_fixture",
        "canonical_name": "TownFixture",
        "runnable": True,
        "status": "PENDING",
        "stage": "admission",
        "reason": None,
        "block_code": None,
        "trials": {
            name: {
                "status": "PENDING",
                "reason": None,
                "attempt_directory": None,
                "validation": None,
            }
            for name in matrix.TRIAL_IDS
        },
    }
    write_json(tmp_path / "maps/town_fixture/status.json", status)
    with patch.object(
        matrix,
        "_campaign_route_map_preflight",
        fake_campaign_route_map_preflight,
    ):
        route_matrix = matrix.select_routes(
            tmp_path,
            "town_fixture",
            straight_catalog_path=straight_catalog,
            turn_catalog_path=turn_catalog,
        )

    aggregate = matrix.summarize(tmp_path)
    selection = aggregate["turn_route_selection_plans"][0]
    audit = route_matrix["trials"][1]["turn_route_selection_audit"]
    assert aggregate["turn_route_selection_policy"] == (
        matrix.SPEED_30KPH_TURN_ROUTE_SELECTION_POLICY
    )
    assert selection["status"] == "SELECTED"
    assert selection["candidate_set_sha256"] == audit["candidate_set_sha256"]
    assert selection["selected_ranking_vector"] == audit[
        "selected_ranking_vector"
    ]
    assert selection["map_preflight_fallback_allowed"] is False


def test_map_level_failed_update_terminalizes_unfinished_trials(
    tmp_path: Path,
) -> None:
    synthetic_plan(tmp_path)
    status = {
        "schema_version": 1,
        "matrix_id": "fixture",
        "map_id": "town_fixture",
        "canonical_name": "TownFixture",
        "runnable": True,
        "status": "PENDING",
        "stage": "admission",
        "reason": None,
        "block_code": None,
        "trials": {
            name: {
                "status": "PENDING",
                "reason": None,
                "attempt_directory": None,
                "validation": None,
            }
            for name in matrix.TRIAL_IDS
        },
    }
    write_json(tmp_path / "maps/town_fixture/status.json", status)

    updated = matrix.update_status(
        tmp_path,
        "town_fixture",
        "FAILED",
        "route_contract_failed",
        "Fresh exact-route map preflight failed.",
        None,
        None,
        None,
        None,
    )

    assert updated["status"] == "FAILED"
    assert updated["stage"] == "route_contract_failed"
    for trial in updated["trials"].values():
        assert trial["status"] == "FAILED"
        assert trial["reason"].startswith(
            "Not executed because map-level prerequisite failed:"
        )
        assert trial["attempt_directory"] is None
        assert trial["validation"] is None
    aggregate = json.loads(
        (tmp_path / "aggregate.json").read_text(encoding="utf-8")
    )
    assert aggregate["status"] == "FAILED"
    assert aggregate["status_counts"] == {"FAILED": 1}


def test_map_level_failed_update_preserves_completed_and_terminalizes_running_trial(
    tmp_path: Path,
) -> None:
    synthetic_plan(tmp_path)
    straight_attempt = tmp_path / "maps/town_fixture/trials/straight/attempt_001"
    turn_attempt = tmp_path / "maps/town_fixture/trials/turn/attempt_001"
    status = {
        "schema_version": 1,
        "matrix_id": "fixture",
        "map_id": "town_fixture",
        "canonical_name": "TownFixture",
        "runnable": True,
        "status": "RUNNING",
        "stage": "turn_running",
        "reason": None,
        "block_code": None,
        "trials": {
            "straight": {
                "status": "PASS",
                "reason": "validated",
                "attempt_directory": str(straight_attempt),
                "validation": str(straight_attempt / "matrix_validation.json"),
            },
            "turn": {
                "status": "RUNNING",
                "reason": "in progress",
                "attempt_directory": str(turn_attempt),
                "validation": None,
            },
        },
    }
    write_json(tmp_path / "maps/town_fixture/status.json", status)

    updated = matrix.update_status(
        tmp_path,
        "town_fixture",
        "FAILED",
        "route_contract_failed",
        "Map prerequisite was invalidated.",
        None,
        None,
        None,
        None,
    )

    assert updated["trials"]["straight"] == status["trials"]["straight"]
    turn = updated["trials"]["turn"]
    assert turn["status"] == "FAILED"
    assert turn["attempt_directory"] == str(turn_attempt)
    assert turn["validation"] is None
    assert "Map prerequisite was invalidated." in turn["reason"]


def test_map_level_pass_rejects_nonpassing_trials(tmp_path: Path) -> None:
    synthetic_plan(tmp_path)
    status = {
        "schema_version": 1,
        "matrix_id": "fixture",
        "map_id": "town_fixture",
        "canonical_name": "TownFixture",
        "runnable": True,
        "status": "RUNNING",
        "stage": "straight_passed",
        "reason": None,
        "block_code": None,
        "trials": {
            "straight": {
                "status": "PASS",
                "reason": "validated",
                "attempt_directory": str(tmp_path / "straight"),
                "validation": str(tmp_path / "straight/matrix_validation.json"),
            },
            "turn": {
                "status": "PENDING",
                "reason": None,
                "attempt_directory": None,
                "validation": None,
            },
        },
    }
    status_path = tmp_path / "maps/town_fixture/status.json"
    write_json(status_path, status)

    with pytest.raises(
        matrix.MatrixError, match="requires both straight and turn trial status PASS"
    ):
        matrix.update_status(
            tmp_path,
            "town_fixture",
            "PASS",
            "complete",
            "unchecked",
            None,
            None,
            None,
            None,
        )
    assert json.loads(status_path.read_text(encoding="utf-8")) == status


def test_map_level_pass_freshly_revalidates_both_trials(tmp_path: Path) -> None:
    synthetic_plan(tmp_path)
    trials = {}
    for trial_id in matrix.TRIAL_IDS:
        attempt = tmp_path / f"maps/town_fixture/trials/{trial_id}/attempt_001"
        validation = attempt / "matrix_validation.json"
        write_json(validation, {"status": "PASS"})
        trials[trial_id] = {
            "status": "PASS",
            "reason": "validated",
            "attempt_directory": str(attempt),
            "validation": str(validation),
        }
    status = {
        "schema_version": 1,
        "matrix_id": "fixture",
        "map_id": "town_fixture",
        "canonical_name": "TownFixture",
        "runnable": True,
        "status": "RUNNING",
        "stage": "turn_passed",
        "reason": None,
        "block_code": None,
        "trials": trials,
    }
    write_json(tmp_path / "maps/town_fixture/status.json", status)

    def fresh_validation(
        _output_root: Path, map_id: str, trial_id: str, trial_dir: Path
    ) -> dict:
        return {
            "status": "PASS",
            "map_id": map_id,
            "trial_id": trial_id,
            "trial_directory": str(trial_dir),
            "admission_contract_sha256": "a" * 64,
            "campaign_execution_contract_sha256": "b" * 64,
        }

    with patch.object(matrix, "validate_trial", side_effect=fresh_validation) as validate:
        updated = matrix.update_status(
            tmp_path,
            "town_fixture",
            "PASS",
            "ignored",
            "ignored",
            None,
            None,
            None,
            None,
        )

    assert validate.call_count == 2
    assert updated["status"] == "PASS"
    assert updated["stage"] == "complete"
    assert updated["reason"] == "Straight and turn full-stack trials both passed."


def test_runner_has_owned_cold_start_and_fixed_profile_contract() -> None:
    subprocess.run(["bash", "-n", str(RUNNER)], check=True)
    source = RUNNER.read_text(encoding="utf-8")

    assert "setsid \"${server_tool}\"" in source
    assert "e2e_stop_owned_process_group" in source
    assert "--runtime-profile PROFILE" in source
    assert "recommended, speed_30kph" in source
    assert "speed_30kph|speed_30kph_camera_source_5hz" in source
    assert '"${trial_wrapper_options[@]}"' in source
    assert "--runtime-profile \"${runtime_profile}\"" in source
    assert "--map-load-settle-sec 0" in source
    assert "--allow-map-load" not in source
    assert "client.load_world" in source
    assert "straight turn" in source
    assert '[[ "${bundle_schema}" == "custom_map" ]]' in source
    assert '"--${catalog_id}-catalog"' in source
    assert '--seeds "${catalog_seeds_csv}"' in source
    assert 'catalog_scenario_args=(--scenarios straight)' in source
    assert "turn)" in source
    assert "catalog_scenario_args=(--scenarios left,right)" in source
    assert '"${catalog_scenario_args[@]}"' in source
    assert '"${catalog_endpoint_source_args[@]}"' in source
    assert '"${speed_30kph_profile}" == "1"' in source
    assert '"${bundle_schema}" == "packaged_town"' in source
    assert 'route_catalog_specs_for_map "${map_id}"' in source
    assert '--endpoint-waypoint-spacing-m "${catalog_waypoint_spacing}"' in source
    assert '--endpoint-junction-policy "${catalog_junction_policy}"' in source
    assert '--candidate-enumeration-policy "${catalog_enumeration_policy}"' in source
    assert '--straight-capacity-profile "${catalog_capacity_profile}"' in source
    assert "--physical-straight-profile speed_30kph" in source
    assert "--physical-turn-profile speed_30kph" in source
    assert "route_generation_contracts" in source
    assert "--initial-approach-distance-m 15.0" in source
    assert "--maximum-initial-lateral-deviation-m 1.5" in source
    assert "--maximum-initial-heading-change-deg 30.0" in source
    assert "--minimum-turn-arc-length-m 10.0" in source
    assert "--maximum-turn-arc-length-m 30.0" in source
    assert "--minimum-turn-heading-change-deg 60.0" in source
    assert "--maximum-turn-heading-change-deg 120.0" in source
    assert "--maximum-turn-heading-excess-deg 20.0" in source
    assert "--turn-alignment-heading-margin-deg 10.0" in source
    assert "--maximum-turn-command-lead-m 8.0" in source
    assert "--maximum-turn-command-tail-m 8.0" in source
    assert "--maximum-turn-p95-curvature-per-m 0.20" in source
    assert "pkill" not in source
    assert "killall" not in source
