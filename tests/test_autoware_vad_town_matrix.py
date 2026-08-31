from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess

from PIL import Image
import pytest


ROOT = Path(__file__).parents[1]
MODULE_PATH = ROOT / "scripts/e2e/autoware_vad_town_matrix.py"
MATRIX_PATH = ROOT / "scripts/e2e/autoware_vad_town_matrix.yaml"
RUNNER = ROOT / "scripts/e2e/run_autoware_vad_town_matrix.sh"
SPEC = importlib.util.spec_from_file_location("autoware_vad_town_matrix", MODULE_PATH)
assert SPEC and SPEC.loader
matrix = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(matrix)


def write_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


def route_payload(scenario: str, options: list[str]) -> dict:
    command = matrix.VAD_COMMANDS
    points = []
    for index, option in enumerate(options):
        points.append(
            {
                "index": index,
                "distance_m": float(index),
                "road_option": option,
                "vad_command": command[option],
            }
        )
    counts: dict[str, int] = {}
    for option in options:
        counts[option] = counts.get(option, 0) + 1
    return {
        "schema_version": 1,
        "town": "TownFixture",
        "scenario": scenario,
        "route_length_m": float(len(options) - 1),
        "start_spawn_index": 1,
        "goal_spawn_index": 2,
        "option_counts": counts,
        "route": points,
    }


def synthetic_plan(
    output_root: Path,
    bundle_metadata: Path | None = None,
    bundle_schema: str | None = None,
) -> dict:
    bundle = {
        "path": str(bundle_metadata.parent) if bundle_metadata else "/bundle",
        "metadata_sha256": (
            matrix.sha256_file(bundle_metadata) if bundle_metadata else "bundle-sha"
        ),
    }
    if bundle_schema is not None:
        bundle["bundle_schema"] = bundle_schema
    plan = {
        "schema_version": 1,
        "matrix_id": "fixture",
        "runtime_profile": {
            "id": "recommended",
            "wrapper_options": [
                "--recommended",
                "--visualize",
                "--capture-desktop",
            ],
        },
        "route_contract": {},
        "maps": [
            {
                "map_id": "town_fixture",
                "canonical_name": "TownFixture",
                "load_name": "/Game/Carla/Maps/TownFixture",
                "runnable": True,
                "full_map_bundle": bundle,
            }
        ],
    }
    write_json(output_root / "matrix_plan.json", plan)
    return plan


def make_catalog(output_root: Path, corrupt_straight: bool = False) -> Path:
    catalog_root = output_root / "maps/town_fixture/catalog"
    routes = []
    values = {
        "straight": ["LANEFOLLOW", "STRAIGHT", "STRAIGHT", "LANEFOLLOW"],
        "left": ["LANEFOLLOW", "LEFT", "LEFT", "LANEFOLLOW"],
        "right": ["LANEFOLLOW", "RIGHT", "LANEFOLLOW"],
    }
    if corrupt_straight:
        values["straight"].append("LEFT")
    for scenario, options in values.items():
        route_id = f"town_fixture_{scenario}_s0000_p00"
        path = catalog_root / "routes/town_fixture" / scenario / f"{route_id}.json"
        write_json(path, route_payload(scenario, options))
        routes.append(
            {
                "id": route_id,
                "status": "ready",
                "scenario": scenario,
                "seed": 0,
                "pair_index": 0,
                "path": path.relative_to(catalog_root).as_posix(),
                "sha256": matrix.sha256_file(path),
            }
        )
    catalog_path = catalog_root / "route_catalog.json"
    write_json(
        catalog_path,
        {
            "status": "complete",
            "map_id": "town_fixture",
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
    turn_preflight = {
        "status": "PASS",
        "scenario": "left",
        "directional_command": "LEFT",
        "directional_block_count": 1,
        "additional_maneuver_commands": [],
        "failure_reasons": [],
    }
    for scenario, maneuver in (("straight", "STRAIGHT"), ("left", "LEFT")):
        if scenario == "left":
            options = ["LANEFOLLOW"] * 16 + ["LEFT"] * 13 + ["LANEFOLLOW"] * 4
            if compound_turn:
                options += ["LEFT"] * 3 + ["LANEFOLLOW"]
        else:
            options = ["LANEFOLLOW"] * 16 + [maneuver] * 3 + ["LANEFOLLOW"]
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
        payload["initial_approach_preflight"] = preflight
        if scenario == "left":
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
                    if scenario == "left"
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


def test_real_matrix_admits_only_freshly_preflighted_full_map_bundles() -> None:
    manifest, path = matrix.load_matrix(MATRIX_PATH)
    plan = matrix.build_campaign_plan(manifest, path)

    assert plan["canonical_map_count"] == 19
    assert plan["runnable_map_count"] == 8
    assert len(plan["admission_contract_sha256"]) == 64
    by_id = {entry["map_id"]: entry for entry in plan["maps"]}
    assert by_id["c_track_1_0_7"]["runnable"] is True
    assert by_id["c_track_1_0_7"]["full_map_bundle"]["road_lanelets"] == 395
    assert by_id["town01"]["status"] == "PENDING"
    assert by_id["town01"]["full_map_bundle"]["readiness"]["status"] == (
        "TEST_ROUTES_MAP_PREFLIGHT_PASS"
    )
    assert by_id["town03"]["full_map_bundle"]["readiness"][
        "alignment_status"
    ] == "PASS_WITH_OUTLIERS"
    assert "not yet a VAD PASS" in by_id["town03"]["reason"]
    assert by_id["town10hd_opt"]["status"] == "BLOCKED"
    assert by_id["town10hd_opt"]["block_code"] == (
        "blocked_missing_pointcloud"
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


def test_route_selection_rejects_a_straight_route_with_turn_command(tmp_path: Path) -> None:
    synthetic_plan(tmp_path)
    catalog = make_catalog(tmp_path, corrupt_straight=True)

    with pytest.raises(matrix.MatrixError, match="straight trial contains"):
        matrix.select_routes(tmp_path, "town_fixture", catalog)


def test_route_validation_rejects_wrong_vad_command() -> None:
    route = route_payload("left", ["LANEFOLLOW", "LEFT"])
    route["route"][1]["vad_command"] = 2

    with pytest.raises(matrix.MatrixError, match="road-option/VAD-command pair"):
        matrix._validate_route_payload(route, "TownFixture", "left")


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
    trial = tmp_path / "maps/town_fixture/trials/straight/attempt_001"
    trial.mkdir(parents=True)
    selected_route = Path(route_entry["route_path"])
    (trial / "source_route.json").write_bytes(selected_route.read_bytes())
    (trial / "map_bundle.json").write_bytes(bundle.read_bytes())
    (trial / "runtime.env").write_text(
        "RECOMMENDED=true\nVISUALIZE=true\nCAPTURE_DESKTOP=true\n",
        encoding="utf-8",
    )
    write_json(
        trial / "result.json",
        {
            "success": True,
            "execution_mode": "full_stack",
            "assessment": {
                "planning_architecture": "vad_route_manager_hybrid",
                "route_completion": "PASS",
            },
            "final": {"goal_reached": True, "route_status": "goal_reached"},
        },
    )
    write_json(
        trial / "diagnosis.json",
        {"inputs": {"town": "TownFixture", "scenario": "straight"}},
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

    validation = matrix.validate_trial(
        tmp_path, "town_fixture", "straight", trial
    )
    assert validation["status"] == "PASS"
    assert validation["desktop_capture"]["candidate_observed"] is True

    desktop["capture_started_after_candidate"] = False
    write_json(trial / "desktop_capture.json", desktop)
    with pytest.raises(matrix.MatrixError, match="after a VAD candidate"):
        matrix.validate_trial(tmp_path, "town_fixture", "straight", trial)


def test_runner_has_owned_cold_start_and_fixed_profile_contract() -> None:
    subprocess.run(["bash", "-n", str(RUNNER)], check=True)
    source = RUNNER.read_text(encoding="utf-8")

    assert "setsid \"${server_tool}\"" in source
    assert "e2e_stop_owned_process_group" in source
    assert "--recommended --visualize --capture-desktop" in source
    assert "--map-load-settle-sec 0" in source
    assert "--allow-map-load" not in source
    assert "client.load_world" in source
    assert "straight turn" in source
    assert '[[ "${bundle_schema}" == "custom_map" ]]' in source
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
