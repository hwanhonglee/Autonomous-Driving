#!/usr/bin/env python3
"""HH_260906 - Audit a finished Town01 turn catalog without granting capture or training admission."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[2]
REFERENCE = ROOT / (
    "storage/experiments/training/2026-09-07/independent_common10_v1/"
    "town01_right_train_retry_02/route_catalog/routes/town01/right/"
    "town01_right_s0001_p00.json"
)
REFERENCE_SHA256 = "619e8ca9206b4f42f4e6b417811b043a58d9b2e262ffaf067932d12367809a6f"
MAX_BYTES = 32 * 1024 * 1024


class AuditError(ValueError):
    """HH_260906 - Distinguish incomplete evidence from a qualified route."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AuditError(message)


def read_json(path: Path, expected_sha256: str | None = None) -> tuple[dict, str]:
    """HH_260906 - Bind a bounded regular JSON file to its exact source bytes."""
    require(path.is_file() and not path.is_symlink(), f"regular input required: {path}")
    require(path.stat().st_size <= MAX_BYTES, f"oversized input: {path}")
    payload = path.read_bytes()
    digest = hashlib.sha256(payload).hexdigest()
    require(expected_sha256 is None or digest == expected_sha256, f"SHA mismatch: {path}")

    def unique(pairs: list[tuple[str, object]]) -> dict:
        result = {}
        for key, value in pairs:
            require(key not in result, f"duplicate JSON key: {key}")
            result[key] = value
        return result

    document = json.loads(payload, object_pairs_hook=unique,
                          parse_constant=lambda token: (_ for _ in ()).throw(AuditError(f"nonfinite JSON: {token}")))
    require(type(document) is dict, f"JSON object required: {path}")
    return document, digest


def finite(value: object, name: str) -> float:
    require(type(value) in (int, float) and math.isfinite(value), f"finite {name} required")
    return float(value)


def turn_geometry(route: dict) -> dict:
    """HH_260906 - Recheck the serialized turn block rather than trusting a catalog title."""
    points = route.get("route")
    require(type(points) is list and len(points) >= 2, "route points missing")
    length = finite(route.get("route_length_m"), "route length")
    require(200.0 <= length <= 230.0, "route length outside declared 200-230 m")
    preflight = route.get("physical_turn_preflight")
    require(type(preflight) is dict and preflight.get("status") == "PASS"
            and preflight.get("scenario") == "right"
            and preflight.get("directional_block_count") == 1
            and preflight.get("additional_maneuver_commands") == [], "isolated right-turn preflight required")
    block = preflight.get("selected_block")
    require(type(block) is dict, "turn block missing")
    start, end = block.get("start_index"), block.get("end_index")
    require(type(start) is int and type(end) is int and 0 <= start <= end < len(points),
            "turn indices outside route")
    xy = []
    last = -1.0
    for point in points:
        x, y, arc = (finite(point.get(key), key) for key in ("x", "y", "distance_m"))
        require(arc >= last, "unordered route distance")
        xy.append((x, y))
        last = arc
    require(abs(last - length) <= 0.01, "terminal route distance mismatch")
    turn_points = points[start:end + 1]
    require(all(point.get("road_option") == "RIGHT" for point in turn_points),
            "selected turn block is not RIGHT")
    transitions = []
    for point in turn_points:
        identity = tuple(point.get(key) for key in ("road_id", "section_id", "lane_id"))
        require(all(type(item) is int for item in identity), "integer road/lane identity required")
        if not transitions or transitions[-1] != identity:
            transitions.append(identity)
    lead, tail = (finite(block.get(key), key) for key in
                  ("route_lead_distance_m", "route_tail_distance_m"))
    require(abs(lead - finite(points[start]["distance_m"], "turn start arc")) <= 1.0,
            "turn lead disagrees with serialized route")
    require(abs(tail - (length - finite(points[end]["distance_m"], "turn end arc"))) <= 1.0,
            "turn tail disagrees with serialized route")
    center = xy[(start + end) // 2]
    return dict(xy=xy, transitions=transitions, center=center, lead_m=lead, tail_m=tail,
                length_m=length, turn_start_index=start, turn_end_index=end)


def overlap_fraction(candidate_xy: list[tuple[float, float]], reference_xy: list[tuple[float, float]],
                     radius_m: float = 2.0) -> float:
    """HH_260906 - Flag spatial reuse; a low overlap alone does not prove independent sites."""
    require(bool(candidate_xy) and bool(reference_xy), "nonempty route geometry required")
    radius2 = radius_m * radius_m
    return sum(any((x-rx)**2 + (y-ry)**2 <= radius2 for rx, ry in reference_xy)
               for x, y in candidate_xy) / len(candidate_xy)


def audit(owner_root: Path, reference_path: Path = REFERENCE) -> dict:
    """HH_260906 - Require a complete owned TRAIN catalog before reading right-turn candidates."""
    owner_root = owner_root.resolve(strict=True)
    allowed = ROOT / "storage/experiments/training/2026-09-27"
    require(owner_root.parent == allowed and owner_root.name.startswith("drive_long_route_town01_train_catalog_v"),
            "only current Town01 TRAIN owner roots allowed")
    plan, plan_sha = read_json(owner_root / "owner_plan.json")
    owner, owner_sha = read_json(owner_root / "owner_result.json")
    require(plan.get("map_id") == "town01" and plan.get("split_intent") == "TRAIN"
            and plan.get("capture_executed") is False and plan.get("training_executed") is False,
            "Town01 catalog-only plan required")
    require(owner.get("exit_code") == 0 and owner.get("catalog_status") == "complete"
            and owner.get("source_bytes_unchanged_and_archived") is True
            and owner.get("lifecycle_receipts_present") is True,
            "incomplete or failed owner catalog")
    for stage in ("ready", "after_catalog", "stopped"):
        receipt, _ = read_json(owner_root / "lifecycle" / f"{stage}.json")
        require(receipt.get("stage") == stage and receipt.get("status") == "PASS",
                f"invalid {stage} lifecycle receipt")
    expected_catalog_sha = owner.get("catalog_sha256")
    require(type(expected_catalog_sha) is str
            and re.fullmatch(r"[0-9a-f]{64}", expected_catalog_sha) is not None,
            "owner catalog SHA256 required")
    catalog, catalog_sha = read_json(owner_root / "route_catalog.json", expected_catalog_sha)
    require(catalog.get("status") == "complete" and catalog.get("map_id") == "town01"
            and catalog.get("canonical_name") == "Town01"
            and catalog.get("server", {}).get("map_load_performed") is False,
            "catalog identity/completeness mismatch")
    reference, reference_sha = read_json(reference_path, REFERENCE_SHA256)
    reference_geometry = turn_geometry(reference)
    entries = catalog.get("routes")
    require(type(entries) is list, "catalog route list required")
    candidates, seen = [], set()
    for entry in entries:
        require(type(entry) is dict, "catalog entry object required")
        if entry.get("scenario") != "right":
            continue
        identity = entry.get("id")
        require(type(identity) is str and identity.startswith("town01_right_s")
                and identity not in seen, "unique Town01 right ID required")
        seen.add(identity)
        relative = Path(entry.get("path", ""))
        require(not relative.is_absolute() and ".." not in relative.parts
                and relative.parts[:3] == ("routes", "town01", "right")
                and relative.name == identity + ".json", "route path escaped owner root")
        path = owner_root / relative
        expected_sha = entry.get("sha256")
        require(type(expected_sha) is str and re.fullmatch(r"[0-9a-f]{64}", expected_sha) is not None,
                "catalog route SHA256 required")
        route, digest = read_json(path, expected_sha)
        require(entry.get("status") == "ready" and route.get("town") == "Town01"
                and route.get("scenario") == "right"
                and route.get("coordinate_reference") == "base_link", "route identity mismatch")
        geometry = turn_geometry(route)
        require(abs(geometry["length_m"] - finite(entry.get("route_length_m"), "catalog route length")) <= 0.01,
                "catalog/payload route length mismatch")
        center_distance = math.dist(geometry["center"], reference_geometry["center"])
        reused_turn = geometry["transitions"] == reference_geometry["transitions"]
        overlap = overlap_fraction(geometry["xy"], reference_geometry["xy"])
        speed_budget = {}
        for speed in (9, 15):
            speed_mps = speed / 3.6
            minimum_total = 30.0 + speed_mps * 36.4
            minimum_tail = speed_mps * 6.4
            speed_budget[str(speed)] = dict(
                total_minimum_m=minimum_total, turn_tail_minimum_m=minimum_tail,
                total_length_pass=geometry["length_m"] >= minimum_total,
                turn_lead_pass=geometry["lead_m"] >= 30.0,
                turn_tail_pass=geometry["tail_m"] >= minimum_tail,
            )
        flags = []
        if reused_turn:
            flags.append("SAME_PHYSICAL_TURN_TRANSITIONS_AS_TRAIN")
        if center_distance < 30.0:
            flags.append("TURN_CENTER_WITHIN_30M_OF_TRAIN")
        if overlap >= 0.2:
            flags.append("AT_LEAST_20_PERCENT_ROUTE_WITHIN_2M_OF_TRAIN")
        if not all(speed_budget[str(speed)][key] for speed in (9, 15)
                   for key in ("total_length_pass", "turn_lead_pass", "turn_tail_pass")):
            flags.append("STATIC_9_OR_15_KPH_CONTEXT_BUDGET_INCOMPLETE")
        candidates.append(dict(route_id=identity, path=str(path.relative_to(ROOT)), sha256=digest,
                               route_length_m=geometry["length_m"], turn_lead_m=geometry["lead_m"],
                               turn_tail_m=geometry["tail_m"], turn_center_distance_to_train_m=center_distance,
                               same_turn_transitions_as_train=reused_turn,
                               fraction_waypoints_within_2m_of_train=overlap, static_budget=speed_budget,
                               review_flags=flags, capture_authorized=False, training_data_approved=False))
    require(len(candidates) == owner.get("right_candidate_count") and candidates,
            "owner right-turn count mismatch or empty")
    return dict(schema="portable_e2e.long_turn_candidate_audit.v1", status="STATIC_REVIEW_ONLY",
                owner_plan_sha256=plan_sha, owner_result_sha256=owner_sha,
                catalog_sha256=catalog_sha, historical_train_route_sha256=reference_sha,
                candidates=candidates, capture_authorized=False, training_data_approved=False,
                learned_closed_loop_proven=False)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--owner-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    require(not args.output.exists(), "refusing to overwrite audit output")
    result = audit(args.owner_root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")


if __name__ == "__main__":
    main()
