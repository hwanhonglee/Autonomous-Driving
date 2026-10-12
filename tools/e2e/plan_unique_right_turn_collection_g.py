#!/usr/bin/env python3
"""HH_260906 - Plan fresh TRAIN-only right-turn collection without launching CARLA or admitting data."""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CATALOG_ROOT = (
    ROOT
    / "storage/experiments/validation/2026-09-01/"
    "autoware_vad_town_matrix_30kph_v16_owned_window_final/maps"
)
# HH_260906 - New collection plans use real storage roots, not legacy symlinks.
DEFAULT_RAW_OUTPUT_ROOT = ROOT / "storage/datasets/raw/carla/unique_right_g_v1"
ACTIVE_TRAIN_MAPS = (
    ("town01", "Town01"),
    ("town07", "Town07"),
    ("c_track_1_0_7", "C_track_1_0_7"),
)
FORBIDDEN_MAPS = {
    "town03": "DEV",
    "town04": "PROTECTED_TEST",
    "town15": "PROTECTED_TEST",
}
MAX_INPUT_BYTES = 32 * 1024 * 1024
CATALOG_ROUTE_ID = re.compile(r"([a-z0-9_]+)_right_s0000_p([0-9]{2})\Z")
SHA256 = re.compile(r"[0-9a-f]{64}\Z")
EXISTING_TOWN01_P00 = {
    "episode_id": "carla_town01_town01_right_s0001_p00_ClearNoon_s0001_1af7c32f1f1f",
    "route_id": "town01_right_s0001_p00",
    "source_route_sha256": "22c33359f618d4901aa627c3c6307b80e2ba2cab29a6f26edf664a937bd50785",
    "catalog_route_sha256": "619e8ca9206b4f42f4e6b417811b043a58d9b2e262ffaf067932d12367809a6f",
    "route_geometry_sha256": "69e2448daff8e6985e9676faa5371c0baba11539e1d0d08cd7727be680f41d7c",
    "turn_group_transitions": [[312, 0, 1], [22, 0, 1]],
}
FIXED_CAPTURE = {
    "split": "train",
    "scenario": "right",
    "physics_hz": 20.0,
    "camera_hz": 10.0,
    "camera_interval_ticks": 2,
    "target_speed_kph": 30.0,
    "speed_ceiling_kph": 30.0,
    "label_owner": "CARLA PythonAPI BasicAgent",
    "control_owner": "CARLA PythonAPI BasicAgent",
    "learned_actuation": False,
    "portable_e2e_controls_vehicle": False,
}
DENIALS = {
    "carla_launched": False,
    "world_rpc_performed": False,
    "collection_executed": False,
    "automatic_retry": False,
    "retry_allowed": False,
    "test_catalogs_opened": False,
    "test_payloads_opened": False,
    "dev_catalogs_opened": False,
    "dev_payloads_opened": False,
    "dataset_admission_approved": False,
    "training_data_approved": False,
    "model_promotion_approved": False,
    "vehicle_release_approved": False,
    "closed_loop_claimed": False,
    "source_data_modified": False,
}


class PlanError(ValueError):
    """HH_260906 - Mark a fail-closed prospective planning contract violation."""


def require(condition: bool, message: str) -> None:
    """HH_260906 - Reject incomplete or ambiguous evidence instead of weakening the plan."""
    if not condition:
        raise PlanError(message)


def canonical_path(path: Path) -> Path:
    """HH_260906 - Refuse parent traversal and symlinked inputs or output parents."""
    path = Path(path).absolute()
    require(".." not in path.parts and path == path.resolve(), "canonical path required")
    current = path
    while not current.exists():
        current = current.parent
    require(not any(item.is_symlink() for item in (current, *current.parents)), "symlinked path refused")
    return path


def sha256_bytes(payload: bytes) -> str:
    """HH_260906 - Hash exact source bytes without JSON normalization."""
    return hashlib.sha256(payload).hexdigest()


def read_bytes(path: Path, ledger: dict[str, str], expected_sha256: str | None = None) -> bytes:
    """HH_260906 - Read one bounded single-link regular file and bind its exact bytes."""
    path = canonical_path(path)
    require(path.is_file() and path.stat().st_nlink == 1, f"single-link regular file required: {path}")
    require(path.stat().st_size <= MAX_INPUT_BYTES, f"input exceeds size bound: {path}")
    payload = path.read_bytes()
    require(len(payload) <= MAX_INPUT_BYTES, f"input grew past size bound: {path}")
    digest = sha256_bytes(payload)
    if expected_sha256 is not None:
        require(SHA256.fullmatch(expected_sha256) is not None, "valid expected SHA256 required")
        require(digest == expected_sha256, f"route SHA256 mismatch: {path}")
    key = str(path)
    require(key not in ledger or ledger[key] == digest, f"input changed during planning: {path}")
    ledger[key] = digest
    return payload


def strict_json(payload: bytes, label: str) -> dict:
    """HH_260906 - Decode finite JSON objects and reject duplicate keys."""
    def pairs(values):
        result = {}
        for key, value in values:
            require(key not in result, f"duplicate JSON key in {label}: {key}")
            result[key] = value
        return result

    try:
        value = json.loads(
            payload,
            object_pairs_hook=pairs,
            parse_constant=lambda token: (_ for _ in ()).throw(PlanError(f"non-finite JSON in {label}: {token}")),
        )
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise PlanError(f"invalid JSON in {label}") from exc
    require(type(value) is dict, f"JSON object required: {label}")
    return value


def finite_number(value, label: str) -> float:
    """HH_260906 - Require a finite non-boolean numeric field."""
    require(type(value) in (int, float) and math.isfinite(value), f"finite numeric {label} required")
    return float(value)


def portable_path(path: Path) -> str:
    """HH_260906 - Preserve an exact repository-relative path when the evidence is local."""
    path = canonical_path(path)
    return path.relative_to(ROOT).as_posix() if path.is_relative_to(ROOT) else str(path)


def validate_root(catalog_root: Path) -> Path:
    """HH_260906 - Permit only the sealed v16 root inside the real repository."""
    catalog_root = canonical_path(catalog_root)
    require(catalog_root.is_dir(), "catalog maps root must exist")
    lower_parts = {part.lower() for part in catalog_root.parts}
    require(lower_parts.isdisjoint(FORBIDDEN_MAPS), "DEV/TEST catalog root refused")
    if catalog_root.is_relative_to(ROOT):
        require(catalog_root == DEFAULT_CATALOG_ROOT, "only the 2026-09-01 v16 catalog root is allowed")
    return catalog_root


def transition_group(map_id: str, points: list[dict], start: int, end: int) -> tuple[str, list[list[int]]]:
    """HH_260906 - Group aliases by the physical turn road-section-lane transition, not file SHA."""
    require(0 <= start <= end < len(points), "turn block index outside route")
    transitions: list[list[int]] = []
    for point in points[start : end + 1]:
        value = [point.get("road_id"), point.get("section_id"), point.get("lane_id")]
        require(all(type(item) is int for item in value), "integer road/section/lane identity required")
        if not transitions or transitions[-1] != value:
            transitions.append(value)
    require(len(transitions) >= 1, "empty physical turn group")
    encoded = json.dumps([map_id, transitions], separators=(",", ":"), allow_nan=False).encode()
    return f"{map_id}:{sha256_bytes(encoded)[:16]}", transitions


def load_map_candidates(
    catalog_root: Path,
    map_id: str,
    canonical_name: str,
    ledger: dict[str, str],
) -> tuple[list[dict], dict]:
    """HH_260906 - Read only one active TRAIN turn catalog and its right-turn payloads."""
    require(map_id in {item[0] for item in ACTIVE_TRAIN_MAPS}, "map is not in active TRAIN allowlist")
    require(map_id not in FORBIDDEN_MAPS, "DEV/TEST map refused before path construction")
    directory = canonical_path(catalog_root / map_id / "catalog/turn")
    catalog_path = canonical_path(directory / "route_catalog.json")
    catalog_payload = read_bytes(catalog_path, ledger)
    catalog = strict_json(catalog_payload, str(catalog_path))
    require(
        catalog.get("schema_version") == 1
        and catalog.get("status") == "complete"
        and catalog.get("map_id") == map_id
        and catalog.get("canonical_name") == canonical_name,
        "v16 catalog identity/status differs",
    )
    generation = catalog.get("generation")
    require(
        type(generation) is dict
        and generation.get("weather") == "ClearNoon"
        and generation.get("seeds") == [0],
        "v16 catalog generation identity differs",
    )
    entries = catalog.get("routes")
    require(type(entries) is list, "catalog route list required")
    candidates = []
    seen_ids: set[str] = set()
    seen_paths: set[str] = set()
    for entry in entries:
        require(type(entry) is dict, "catalog route object required")
        if entry.get("scenario") != "right":
            continue
        route_id = entry.get("id")
        match = CATALOG_ROUTE_ID.fullmatch(route_id) if type(route_id) is str else None
        require(match is not None and match.group(1) == map_id, "sealed v16 right route ID required")
        pair_index = int(match.group(2))
        require(
            entry.get("status") == "ready"
            and entry.get("seed") == 0
            and entry.get("pair_index") == pair_index,
            "ready seed-0 catalog route required",
        )
        require(route_id not in seen_ids, "duplicate route ID in catalog")
        seen_ids.add(route_id)
        relative = Path(entry.get("path", ""))
        require(
            not relative.is_absolute()
            and ".." not in relative.parts
            and relative.parts[:3] == ("routes", map_id, "right")
            and relative.name == f"{route_id}.json",
            "right route reference escaped sealed catalog subtree",
        )
        route_path = canonical_path(directory / relative)
        require(route_path.is_relative_to(directory), "route path escaped catalog")
        require(str(route_path) not in seen_paths, "duplicate route path in catalog")
        seen_paths.add(str(route_path))
        expected_sha = entry.get("sha256")
        route_payload = read_bytes(route_path, ledger, expected_sha)
        route = strict_json(route_payload, str(route_path))
        require(
            route.get("schema_version") == 1
            and route.get("town") == canonical_name
            and route.get("scenario") == "right"
            and route.get("weather") == "ClearNoon"
            and route.get("coordinate_reference") == "base_link",
            "route identity/frame differs",
        )
        route_length = finite_number(route.get("route_length_m"), "route_length_m")
        require(
            route_length > 0.0
            and abs(route_length - finite_number(entry.get("route_length_m"), "catalog route_length_m")) <= 1e-9,
            "route/catalog length mismatch",
        )
        points = route.get("route")
        require(type(points) is list and len(points) >= 2, "route polyline required")
        preflight = route.get("physical_turn_preflight") or route.get("turn_geometry_preflight")
        require(
            type(preflight) is dict
            and preflight.get("status") == "PASS"
            and preflight.get("scenario") == "right"
            and preflight.get("directional_command") == "RIGHT"
            and preflight.get("directional_block_count") == 1
            and preflight.get("additional_maneuver_commands") == [],
            "one isolated PASS right-turn block required",
        )
        block = preflight.get("selected_block")
        require(type(block) is dict, "selected turn block required")
        start, end = block.get("start_index"), block.get("end_index")
        require(type(start) is int and type(end) is int, "integer turn block indices required")
        group_id, transitions = transition_group(map_id, points, start, end)
        arc_m = finite_number(block.get("command_arc_length_m"), "turn arc")
        heading_deg = finite_number(block.get("absolute_net_heading_change_deg"), "turn heading")
        require(arc_m > 0.0 and 1.0 <= heading_deg <= 179.0, "usable right-turn radius inputs required")
        radius_m = arc_m / math.radians(heading_deg)
        candidate = {
            "map_id": map_id,
            "canonical_name": canonical_name,
            "split": "train",
            "scenario": "right",
            "route_id": route_id,
            "catalog_seed": 0,
            "pair_index": pair_index,
            "catalog_path": portable_path(catalog_path),
            "catalog_sha256": ledger[str(catalog_path)],
            "route_path": portable_path(route_path),
            "route_sha256": ledger[str(route_path)],
            "route_length_m": route_length,
            "start_spawn_index": entry.get("start_spawn_index"),
            "goal_spawn_index": entry.get("goal_spawn_index"),
            "turn_group_id": group_id,
            "turn_group_transitions": transitions,
            "turn_arc_length_m": arc_m,
            "turn_heading_change_deg": heading_deg,
            "effective_turn_radius_m": radius_m,
            "source_preflight": "physical_turn_preflight"
            if route.get("physical_turn_preflight") is not None
            else "turn_geometry_preflight",
            "eligible": True,
            "exclusion_reasons": [],
        }
        if map_id == "town01" and pair_index == 0:
            candidate["exclusion_reasons"].append("EXISTING_TOWN01_P00_ROUTE_IDENTITY")
        if map_id == "town01" and transitions == EXISTING_TOWN01_P00["turn_group_transitions"]:
            candidate["exclusion_reasons"].append("EXISTING_TOWN01_P00_PHYSICAL_TURN_GROUP")
        if candidate["route_sha256"] in {
            EXISTING_TOWN01_P00["source_route_sha256"],
            EXISTING_TOWN01_P00["catalog_route_sha256"],
            EXISTING_TOWN01_P00["route_geometry_sha256"],
        }:
            candidate["exclusion_reasons"].append("EXISTING_TOWN01_P00_ROUTE_SHA256")
        candidate["eligible"] = not candidate["exclusion_reasons"]
        candidates.append(candidate)
    require(candidates, f"no right-turn routes in {map_id} catalog")
    return candidates, {
        "map_id": map_id,
        "canonical_name": canonical_name,
        "split": "train",
        "catalog_path": portable_path(catalog_path),
        "catalog_sha256": ledger[str(catalog_path)],
        "right_candidate_count": len(candidates),
    }


def select_minimum(candidates: list[dict]) -> list[dict]:
    """HH_260906 - Select radius extrema from two distinct new physical groups per active TRAIN map."""
    selected = []
    for map_id, _ in ACTIVE_TRAIN_MAPS:
        by_group: dict[str, list[dict]] = defaultdict(list)
        for candidate in candidates:
            if candidate["map_id"] == map_id and candidate["eligible"]:
                by_group[candidate["turn_group_id"]].append(candidate)
        require(len(by_group) >= 2, f"at least two new right-turn groups required for {map_id}")
        representatives = []
        for group_id, rows in sorted(by_group.items()):
            rows.sort(key=lambda row: (-row["route_length_m"], row["route_id"], row["route_sha256"]))
            representatives.append(rows[0])
        representatives.sort(key=lambda row: (row["effective_turn_radius_m"], row["route_id"]))
        chosen = [representatives[0], representatives[-1]]
        require(
            chosen[0]["turn_group_id"] != chosen[1]["turn_group_id"]
            and abs(chosen[0]["effective_turn_radius_m"] - chosen[1]["effective_turn_radius_m"]) >= 0.25,
            f"distinct group and radius coverage unavailable for {map_id}",
        )
        selected.extend(chosen)
    require(
        len(selected) == 6
        and len({row["route_sha256"] for row in selected}) == 6
        and len({row["turn_group_id"] for row in selected}) == 6,
        "recommended minimum must contain six unique route/group selections",
    )
    return selected


def planned_episodes(
    selected: list[dict], raw_output_root: Path, campaign_date: str
) -> list[dict]:
    """HH_260906 - Bind fresh output leaves and fixed label/control ownership without executing them."""
    require(re.fullmatch(r"20[0-9]{2}-[0-9]{2}-[0-9]{2}", campaign_date) is not None, "ISO campaign date required")
    raw_output_root = canonical_path(raw_output_root)
    episodes = []
    seed_base = int(campaign_date.replace("-", "")) * 100
    for index, route in enumerate(selected, start=1):
        radius_role = "compact_radius" if index % 2 else "wide_radius"
        weather = "ClearNoon" if radius_role == "compact_radius" else "CloudyNoon"
        seed = seed_base + index
        leaf = canonical_path(
            raw_output_root
            / campaign_date
            / "30kph"
            / route["map_id"]
            / "right"
            / route["route_id"]
            / weather
            / f"seed_{seed:010d}"
            / "run_001"
        )
        require(not leaf.exists(), f"planned raw output must be fresh: {leaf}")
        episodes.append(
            {
                "plan_episode_id": f"G-R{index:02d}",
                "map_id": route["map_id"],
                "canonical_name": route["canonical_name"],
                "split": "train",
                "scenario": "right",
                "radius_role": radius_role,
                "route_id": route["route_id"],
                "route_path": route["route_path"],
                "route_sha256": route["route_sha256"],
                "catalog_path": route["catalog_path"],
                "catalog_sha256": route["catalog_sha256"],
                "turn_group_id": route["turn_group_id"],
                "turn_group_transitions": route["turn_group_transitions"],
                "effective_turn_radius_m": route["effective_turn_radius_m"],
                "weather": weather,
                "traffic_seed": seed,
                "capture": dict(FIXED_CAPTURE),
                "attempt_limit": 1,
                "retry_allowed": False,
                "fresh_output_path": portable_path(leaf),
                "output_existed_at_planning": False,
                "execution_allowed_by_this_plan": False,
                "admission_allowed_by_this_plan": False,
                "promotion_allowed_by_this_plan": False,
            }
        )
    return episodes


def build_plan(
    catalog_root: Path,
    output_json: Path,
    raw_output_root: Path = DEFAULT_RAW_OUTPUT_ROOT,
    campaign_date: str = "2026-09-21",
) -> dict:
    """HH_260906 - Produce one immutable prospective plan from active TRAIN v16 route bytes."""
    catalog_root = validate_root(catalog_root)
    output_json = canonical_path(output_json)
    raw_output_root = canonical_path(raw_output_root)
    require(not output_json.exists(), "fresh plan output required")
    require(not output_json.is_relative_to(catalog_root), "plan output must be outside route catalogs")
    require(not raw_output_root.is_relative_to(catalog_root), "raw output root must be outside route catalogs")
    ledger: dict[str, str] = {}
    planner_path = canonical_path(Path(__file__))
    read_bytes(planner_path, ledger)
    candidates, catalogs = [], []
    for map_id, canonical_name in ACTIVE_TRAIN_MAPS:
        rows, catalog = load_map_candidates(catalog_root, map_id, canonical_name, ledger)
        candidates.extend(rows)
        catalogs.append(catalog)
    eligible = [row for row in candidates if row["eligible"]]
    blocked = [row for row in candidates if not row["eligible"]]
    selected = select_minimum(candidates)
    episodes = planned_episodes(selected, raw_output_root, campaign_date)
    per_map = {}
    for map_id, _ in ACTIVE_TRAIN_MAPS:
        rows = [row for row in candidates if row["map_id"] == map_id]
        allowed = [row for row in rows if row["eligible"]]
        per_map[map_id] = {
            "catalog_candidates": len(rows),
            "eligible_candidates": len(allowed),
            "excluded_candidates": len(rows) - len(allowed),
            "eligible_unique_turn_groups": len({row["turn_group_id"] for row in allowed}),
            "eligible_radius_range_m": [
                min(row["effective_turn_radius_m"] for row in allowed),
                max(row["effective_turn_radius_m"] for row in allowed),
            ],
            "recommended_episodes": sum(row["map_id"] == map_id for row in episodes),
        }
    require(
        not any(row["map_id"] in FORBIDDEN_MAPS for row in candidates)
        and all(row["split"] == "train" and row["scenario"] == "right" for row in candidates),
        "DEV/TEST or non-right candidate entered plan",
    )
    before = dict(ledger)
    for path, digest in before.items():
        read_bytes(Path(path), ledger, digest)
    require(before == ledger, "source/input ledger changed before publication")
    plan = {
        "schema": "portable_e2e.unique_right_collection_g_plan.v1",
        "status": "PROSPECTIVE_PLAN_ONLY_NOT_EXECUTED",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "campaign_id": "G_unique_right_turn_train_v1",
        "purpose": "Collect unique right-turn TRAIN evidence before any further replay or model-head changes.",
        "catalog_campaign": "2026-09-01_v16_owned_window_final",
        "catalog_root": portable_path(catalog_root),
        "active_map_policy": {
            "train_maps": [item[0] for item in ACTIVE_TRAIN_MAPS],
            "dev_maps_never_opened": ["town03"],
            "protected_test_maps_never_opened": ["town04", "town15"],
        },
        "fixed_capture_contract": dict(FIXED_CAPTURE),
        "existing_town01_p00_exclusion": dict(EXISTING_TOWN01_P00),
        "catalogs": catalogs,
        "counts": {
            "catalog_right_candidates": len(candidates),
            "eligible_route_candidates": len(eligible),
            "excluded_route_candidates": len(blocked),
            "eligible_unique_turn_groups": len({row["turn_group_id"] for row in eligible}),
            "recommended_minimum_episodes": len(episodes),
            "recommended_weather_counts": dict(Counter(row["weather"] for row in episodes)),
        },
        "per_map_counts": per_map,
        "selection_policy": (
            "Exclude existing Town01 p00 identity, physical turn group, and known route hashes; collapse aliases by "
            "road/section/lane transition; within each group prefer the longest route then route ID; select the minimum "
            "and maximum effective-radius groups from each active TRAIN map."
        ),
        "radius_definition": "selected directional command arc length divided by absolute net heading change in radians",
        "recommended_minimum_episodes": episodes,
        "candidates": candidates,
        "excluded_candidates": blocked,
        "execution_commands": [],
        "denials": dict(DENIALS),
        "source_and_input_sha256_before": before,
        "source_and_input_sha256_after": dict(ledger),
        "planner_source": {
            "path": portable_path(planner_path),
            "sha256": ledger[str(planner_path)],
        },
        "limitations": [
            "This file plans collection only; it is not CARLA execution, dataset admission, training, evaluation, or promotion evidence.",
            "Catalog geometry and effective radius do not prove a successful, collision-free, lane-valid, or temporally complete episode.",
            "Each future run remains a single attempt; a failure must be preserved and a new plan must be reviewed rather than retried automatically.",
            "Only active TRAIN maps were read. Town03 DEV and Town04/Town15 protected TEST catalogs and payloads remain unopened.",
            "BasicAgent exclusively owns control and labels; no learned actuation or Portable E2E vehicle control is authorized.",
        ],
    }
    output_json.parent.mkdir(parents=True, exist_ok=True)
    payload = (json.dumps(plan, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()
    with output_json.open("xb") as stream:
        stream.write(payload)
    return plan


def main(argv: list[str] | None = None) -> None:
    """HH_260906 - Expose deterministic plan generation without any execution subcommand."""
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--catalog-maps-root", type=Path, default=DEFAULT_CATALOG_ROOT)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--raw-output-root", type=Path, default=DEFAULT_RAW_OUTPUT_ROOT)
    parser.add_argument("--campaign-date", default="2026-09-21")
    args = parser.parse_args(argv)
    plan = build_plan(args.catalog_maps_root, args.output_json, args.raw_output_root, args.campaign_date)
    print(json.dumps({"status": plan["status"], **plan["counts"]}, sort_keys=True))


if __name__ == "__main__":
    main()
