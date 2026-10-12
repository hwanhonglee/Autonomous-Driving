"""HH_260906 - Test TRAIN-only prospective G planning without CARLA, retries, admission, or TEST access."""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path

import pytest

from scripts.e2e import plan_unique_right_turn_collection_g as g


def write_json(path: Path, value: dict) -> str:
    """HH_260906 - Write deterministic synthetic fixture bytes and return their exact SHA."""
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = (json.dumps(value, sort_keys=True, allow_nan=False) + "\n").encode()
    path.write_bytes(payload)
    return hashlib.sha256(payload).hexdigest()


def route_document(canonical: str, map_id: str, pair: int, radius: float, transitions: list[list[int]]) -> dict:
    """HH_260906 - Build a minimal isolated right-turn v16-like payload with an explicit physical group."""
    heading = 90.0
    arc = radius * math.radians(heading)
    points = []
    for index, transition in enumerate(transitions):
        points.append(
            {
                "index": index,
                "x": float(index),
                "y": float(pair * 10 + index),
                "z": 0.0,
                "yaw": -math.radians(heading) * index / max(1, len(transitions) - 1),
                "distance_m": float(index),
                "road_option": "RIGHT",
                "road_id": transition[0],
                "section_id": transition[1],
                "lane_id": transition[2],
            }
        )
    length = float(len(points) - 1)
    block = {
        "start_index": 0,
        "end_index": len(points) - 1,
        "command_arc_length_m": arc,
        "absolute_net_heading_change_deg": heading,
    }
    return {
        "schema_version": 1,
        "coordinate_reference": "base_link",
        "town": canonical,
        "weather": "ClearNoon",
        "scenario": "right",
        "route_length_m": length,
        "start_spawn_index": pair,
        "goal_spawn_index": pair + 100,
        "route": points,
        "physical_turn_preflight": {
            "status": "PASS",
            "scenario": "right",
            "directional_command": "RIGHT",
            "directional_block_count": 1,
            "additional_maneuver_commands": [],
            "selected_block": block,
        },
    }


def make_catalogs(root: Path, aliases: bool = False) -> None:
    """HH_260906 - Create all three allowed catalogs without any DEV or TEST path."""
    for map_index, (map_id, canonical) in enumerate(g.ACTIVE_TRAIN_MAPS):
        directory = root / map_id / "catalog/turn"
        entries = []
        for pair, radius in enumerate((7.0 + map_index, 9.0 + map_index, 11.0 + map_index)):
            if map_id == "town01" and pair == 0:
                transitions = deepcopy(g.EXISTING_TOWN01_P00["turn_group_transitions"])
            elif aliases and pair == 2:
                transitions = [[1000 + map_index, 0, 1], [2000 + map_index, 0, 1]]
            else:
                transitions = [[1000 + map_index + pair * 10, 0, 1], [2000 + map_index + pair * 10, 0, 1]]
            route_id = f"{map_id}_right_s0000_p{pair:02d}"
            relative = Path("routes") / map_id / "right" / f"{route_id}.json"
            document = route_document(canonical, map_id, pair, radius, transitions)
            digest = write_json(directory / relative, document)
            entries.append(
                {
                    "id": route_id,
                    "status": "ready",
                    "scenario": "right",
                    "seed": 0,
                    "pair_index": pair,
                    "start_spawn_index": pair,
                    "goal_spawn_index": pair + 100,
                    "route_length_m": document["route_length_m"],
                    "path": relative.as_posix(),
                    "sha256": digest,
                }
            )
        write_json(
            directory / "route_catalog.json",
            {
                "schema_version": 1,
                "status": "complete",
                "map_id": map_id,
                "canonical_name": canonical,
                "generation": {"weather": "ClearNoon", "seeds": [0]},
                "routes": entries,
            },
        )


def test_real_v16_counts_exclusions_and_recommended_minimum(tmp_path, monkeypatch):
    """HH_260906 - Confirm exact real-catalog capacity while proving no DEV or protected TEST read occurs."""
    opened = []
    original = g.read_bytes

    def guarded(path, *args, **kwargs):
        lowered = {part.lower() for part in Path(path).parts}
        assert lowered.isdisjoint(g.FORBIDDEN_MAPS)
        opened.append(Path(path))
        return original(path, *args, **kwargs)

    monkeypatch.setattr(g, "read_bytes", guarded)
    output = tmp_path / "plan.json"
    raw = tmp_path / "future_raw"
    plan = g.build_plan(g.DEFAULT_CATALOG_ROOT, output, raw, "2026-09-21")
    assert output.is_file() and not raw.exists()
    assert plan["counts"] == {
        "catalog_right_candidates": 22,
        "eligible_route_candidates": 21,
        "excluded_route_candidates": 1,
        "eligible_unique_turn_groups": 11,
        "recommended_minimum_episodes": 6,
        "recommended_weather_counts": {"ClearNoon": 3, "CloudyNoon": 3},
    }
    assert plan["per_map_counts"]["town01"] == {
        "catalog_candidates": 8,
        "eligible_candidates": 7,
        "excluded_candidates": 1,
        "eligible_unique_turn_groups": 6,
        "eligible_radius_range_m": pytest.approx([9.074903166308516, 10.44564848373964]),
        "recommended_episodes": 2,
    }
    assert plan["per_map_counts"]["town07"]["catalog_candidates"] == 6
    assert plan["per_map_counts"]["town07"]["eligible_unique_turn_groups"] == 3
    assert plan["per_map_counts"]["c_track_1_0_7"]["eligible_unique_turn_groups"] == 2
    excluded = plan["excluded_candidates"]
    assert len(excluded) == 1 and excluded[0]["route_id"] == "town01_right_s0000_p00"
    assert excluded[0]["exclusion_reasons"] == [
        "EXISTING_TOWN01_P00_ROUTE_IDENTITY",
        "EXISTING_TOWN01_P00_PHYSICAL_TURN_GROUP",
    ]
    episodes = plan["recommended_minimum_episodes"]
    assert [row["route_id"] for row in episodes] == [
        "town01_right_s0000_p01",
        "town01_right_s0000_p03",
        "town07_right_s0000_p03",
        "town07_right_s0000_p04",
        "c_track_1_0_7_right_s0000_p03",
        "c_track_1_0_7_right_s0000_p01",
    ]
    assert len({row["route_sha256"] for row in episodes}) == len(episodes)
    assert len({row["turn_group_id"] for row in episodes}) == len(episodes)
    assert all(row["split"] == "train" and row["scenario"] == "right" for row in episodes)
    assert all(row["capture"] == g.FIXED_CAPTURE for row in episodes)
    assert all(row["attempt_limit"] == 1 and row["retry_allowed"] is False for row in episodes)
    assert all(row["output_existed_at_planning"] is False for row in episodes)
    assert all(Path(row["fresh_output_path"]).name == "run_001" for row in episodes)
    assert plan["source_and_input_sha256_before"] == plan["source_and_input_sha256_after"]
    assert plan["execution_commands"] == [] and not any(plan["denials"].values())
    assert opened and all("catalog/turn" in path.as_posix() or path == Path(g.__file__) for path in opened)


def test_synthetic_plan_is_fresh_deterministic_and_seals_basic_agent(tmp_path):
    """HH_260906 - Bind fixed ownership, timing, speed, seed, weather, and one-attempt output leaves."""
    catalogs = tmp_path / "catalogs"
    make_catalogs(catalogs)
    output = tmp_path / "evidence/plan.json"
    raw = tmp_path / "raw"
    plan = g.build_plan(catalogs, output, raw, "2026-10-01")
    assert plan["counts"]["catalog_right_candidates"] == 9
    assert plan["counts"]["eligible_route_candidates"] == 8
    assert plan["counts"]["recommended_minimum_episodes"] == 6
    episodes = plan["recommended_minimum_episodes"]
    assert len({row["traffic_seed"] for row in episodes}) == 6
    assert [row["weather"] for row in episodes] == ["ClearNoon", "CloudyNoon"] * 3
    assert all(row["capture"]["physics_hz"] == 20.0 for row in episodes)
    assert all(row["capture"]["camera_hz"] == 10.0 for row in episodes)
    assert all(row["capture"]["speed_ceiling_kph"] == 30.0 for row in episodes)
    assert all(row["capture"]["label_owner"] == "CARLA PythonAPI BasicAgent" for row in episodes)
    assert all(row["capture"]["learned_actuation"] is False for row in episodes)
    assert not raw.exists()
    with pytest.raises(g.PlanError, match="fresh plan output"):
        g.build_plan(catalogs, output, raw, "2026-10-01")


def test_existing_p00_group_is_blocked_even_when_bytes_and_sha_are_new(tmp_path):
    """HH_260906 - Prevent a renamed or reserialized old Town01 intersection from masquerading as unique data."""
    catalogs = tmp_path / "catalogs"
    make_catalogs(catalogs)
    rows, _ = g.load_map_candidates(catalogs, "town01", "Town01", {})
    p00 = next(row for row in rows if row["pair_index"] == 0)
    assert p00["route_sha256"] not in {
        g.EXISTING_TOWN01_P00["source_route_sha256"],
        g.EXISTING_TOWN01_P00["catalog_route_sha256"],
        g.EXISTING_TOWN01_P00["route_geometry_sha256"],
    }
    assert p00["eligible"] is False
    assert "EXISTING_TOWN01_P00_PHYSICAL_TURN_GROUP" in p00["exclusion_reasons"]


def test_group_alias_does_not_create_false_diversity(tmp_path):
    """HH_260906 - Collapse different route files that traverse the same physical turn transition."""
    catalogs = tmp_path / "catalogs"
    make_catalogs(catalogs, aliases=True)
    rows, _ = g.load_map_candidates(catalogs, "town07", "Town07", {})
    rows[2]["turn_group_id"] = rows[1]["turn_group_id"]
    rows[2]["turn_group_transitions"] = rows[1]["turn_group_transitions"]
    by_group = {row["turn_group_id"] for row in rows}
    assert len(by_group) == 2
    selected = g.select_minimum(
        rows
        + g.load_map_candidates(catalogs, "town01", "Town01", {})[0]
        + g.load_map_candidates(catalogs, "c_track_1_0_7", "C_track_1_0_7", {})[0]
    )
    town07 = [row for row in selected if row["map_id"] == "town07"]
    assert len({row["turn_group_id"] for row in town07}) == 2


@pytest.mark.parametrize("forbidden", ["town03", "town04", "town15"])
def test_dev_and_test_catalog_roots_are_rejected_before_read(tmp_path, forbidden):
    """HH_260906 - Reject DEV and protected TEST roots before constructing a catalog payload path."""
    root = tmp_path / forbidden / "maps"
    root.mkdir(parents=True)
    with pytest.raises(g.PlanError, match="DEV/TEST"):
        g.validate_root(root)


@pytest.mark.parametrize(
    "mutation,message",
    [
        ("sha", "SHA256 mismatch"),
        ("scenario", "identity/frame"),
        ("preflight", "isolated PASS"),
        ("path", "escaped"),
        ("duplicate", "duplicate route ID"),
    ],
)
def test_catalog_or_route_tamper_fails_closed(tmp_path, mutation, message):
    """HH_260906 - Reject unpinned bytes, wrong direction, failed geometry, path escape, and duplicate IDs."""
    catalogs = tmp_path / "catalogs"
    make_catalogs(catalogs)
    catalog_path = catalogs / "town07/catalog/turn/route_catalog.json"
    catalog = json.loads(catalog_path.read_text())
    entry = catalog["routes"][0]
    route_path = catalog_path.parent / entry["path"]
    route = json.loads(route_path.read_text())
    if mutation == "sha":
        entry["sha256"] = "0" * 64
    elif mutation == "scenario":
        route["scenario"] = "left"
        entry["sha256"] = write_json(route_path, route)
    elif mutation == "preflight":
        route["physical_turn_preflight"]["status"] = "FAIL"
        entry["sha256"] = write_json(route_path, route)
    elif mutation == "path":
        entry["path"] = "../route.json"
    else:
        catalog["routes"].append(deepcopy(entry))
    write_json(catalog_path, catalog)
    with pytest.raises(g.PlanError, match=message):
        g.load_map_candidates(catalogs, "town07", "Town07", {})


def test_existing_future_output_leaf_stops_entire_plan(tmp_path):
    """HH_260906 - Require fresh raw leaves instead of appending or overwriting a prior attempt."""
    catalogs = tmp_path / "catalogs"
    make_catalogs(catalogs)
    selected = g.select_minimum(
        sum(
            (g.load_map_candidates(catalogs, map_id, canonical, {})[0] for map_id, canonical in g.ACTIVE_TRAIN_MAPS),
            [],
        )
    )
    first = g.planned_episodes(selected, tmp_path / "raw", "2026-10-02")[0]
    Path(first["fresh_output_path"]).mkdir(parents=True)
    with pytest.raises(g.PlanError, match="must be fresh"):
        g.planned_episodes(selected, tmp_path / "raw", "2026-10-02")


def test_cli_writes_plan_only_and_prints_capacity(tmp_path, capsys):
    """HH_260906 - Keep the command surface plan-only with no launch, retry, admission, or promotion action."""
    catalogs = tmp_path / "catalogs"
    make_catalogs(catalogs)
    output = tmp_path / "plan.json"
    g.main(
        [
            "--catalog-maps-root",
            str(catalogs),
            "--output-json",
            str(output),
            "--raw-output-root",
            str(tmp_path / "raw"),
            "--campaign-date",
            "2026-10-03",
        ]
    )
    printed = json.loads(capsys.readouterr().out)
    assert printed["status"] == "PROSPECTIVE_PLAN_ONLY_NOT_EXECUTED"
    assert printed["recommended_minimum_episodes"] == 6
    document = json.loads(output.read_text())
    assert document["execution_commands"] == []
    assert not (tmp_path / "raw").exists()
