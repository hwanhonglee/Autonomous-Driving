#!/usr/bin/env python3
"""HH_260906 - Fail-closed offline screen of the owned Town01 multi-maneuver catalog."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re

from scripts.e2e import audit_new_long_turn_candidates as source
from scripts.e2e import rank_town01_v5_long_turns as prior
from scripts.e2e import select_turn_centered_subwindow as window


ROOT = Path(__file__).resolve().parents[2]
OWNER = ROOT / "storage/experiments/training/2026-09-27/drive_long_route_town01_multi_train_catalog_v1"
OUTPUT = ROOT / "docs/assets/validation/2026-09-27/drive_town01_multi_rank/rank.json"
CATALOG_SHA256 = "7f60a7d2969fe07499e46e802b2f0ff5a3737f9fca0fd0b3e641295c5413d53e"


def require(condition: bool, reason: str) -> None:
    if not condition:
        raise prior.InvalidEvidence(reason)


def read(path: Path, expected_sha: str | None = None) -> tuple[dict, str]:
    try:
        return source.read_json(path, expected_sha)
    except source.AuditError as error:
        raise prior.InvalidEvidence(str(error)) from error


def completed_owner(owner_root: Path) -> tuple[dict, dict, dict]:
    """HH_260906 - Bind the exact catalog plan, frozen sources, lifecycle and route inventory."""
    require(owner_root.resolve() == OWNER.resolve(), "only the pinned multi-maneuver owner is accepted")
    plan, _ = read(owner_root / "owner_plan.json")
    require(plan.get("schema") == "portable_e2e.owned_turn_route_catalog.v1" and
            plan.get("map_id") == "town01" and plan.get("map") == "Town01" and
            plan.get("split_intent") == "TRAIN_CANDIDATE" and
            plan.get("search_mode") == "multi_maneuver_right" and
            plan.get("quality") == "Epic" and plan.get("allow_map_load") is False and
            plan.get("capture_executed") is False and plan.get("training_executed") is False and
            plan.get("dataset_admission_granted") is False and plan.get("learned_model_control") is False,
            "multi-maneuver catalog-only owner plan changed")
    argv = plan.get("catalog_argv")
    require(isinstance(argv, list) and "--physical-turn-profile" not in argv,
            "multi-maneuver search flags changed")

    def flag(name: str) -> str:
        require(argv.count(name) == 1 and argv.index(name) + 1 < len(argv),
                f"missing or repeated catalog flag: {name}")
        return argv[argv.index(name) + 1]

    for name, expected in (("--map-id", "town01"), ("--scenarios", "right"),
                           ("--active-server-profile", "packaged_0915"),
                           ("--seeds", "6,7"), ("--pairs-per-seed", "8"),
                           ("--min-distance", "240"), ("--max-distance", "300"),
                           ("--preferred-distance", "160"),
                           ("--sampling-resolution", "1"), ("--max-traces", "100000")):
        require(flag(name) == expected, f"catalog search flag changed: {name}")
    require(Path(flag("--output-root")).resolve() == OWNER.resolve(),
            "catalog output root changed")
    result, _ = read(owner_root / "owner_result.json")
    require(result.get("schema") == "portable_e2e.owned_turn_route_catalog_result.v1" and
            result.get("exit_code") == 0 and result.get("catalog_status") == "complete" and
            result.get("source_bytes_unchanged_and_archived") is True and
            result.get("lifecycle_receipts_present") is True and
            result.get("capture_executed") is False and result.get("training_executed") is False and
            result.get("dataset_admission_granted") is False and result.get("learned_model_control") is False,
            "multi-maneuver owner has not completed safely")
    sources, checks = plan.get("source_sha256"), result.get("source_checks")
    require(isinstance(sources, dict) and len(sources) == 13 and
            isinstance(checks, dict) and set(checks) == set(sources) and
            all(value is True for value in checks.values()), "incomplete frozen-source ledger")
    for relative, expected in sources.items():
        name = Path(relative)
        require(not name.is_absolute() and ".." not in name.parts and
                isinstance(expected, str) and len(expected) == 64 and
                prior.digest(owner_root / "provenance" / name) == expected,
                f"archived source changed: {relative}")
    for stage in ("ready", "after_catalog", "stopped"):
        receipt, _ = read(owner_root / "lifecycle" / f"{stage}.json")
        require(receipt.get("stage") == stage and receipt.get("status") == "PASS" and
                receipt.get("expected_map") == "Town01" and
                receipt.get("mode") == ("stopped" if stage == "stopped" else "running"),
                f"CARLA owner lifecycle failed: {stage}")
        if stage == "stopped":
            require(receipt.get("port_released") is True, "CARLA owner port not released")
    catalog, actual_sha = read(owner_root / "route_catalog.json", CATALOG_SHA256)
    require(result.get("catalog_sha256") == actual_sha and
            catalog.get("status") == "complete" and catalog.get("map_id") == "town01" and
            catalog.get("canonical_name") == "Town01" and
            catalog.get("server", {}).get("active_map_name") == "Carla/Maps/Town01" and
            catalog.get("server", {}).get("active_server_profile") == "packaged_0915" and
            catalog.get("server", {}).get("map_load_performed") is False,
            "catalog hash, map identity or active server changed")
    rights = [item for item in catalog.get("routes", []) if item.get("scenario") == "right"]
    require(len(rights) == 16 and len(rights) == result.get("right_candidate_count") and
            len(catalog["routes"]) == 16, "catalog route inventory changed")
    return plan, result, catalog


def fingerprint(route: dict, selected: dict) -> tuple[tuple[tuple[int, int, int], ...], tuple[float, float]]:
    """HH_260906 - Identify the selected physical RIGHT block, not the whole multi-maneuver route."""
    points = route["route"]
    start, end = selected["right_block_start_index"], selected["right_block_end_index"]
    require(type(start) is int and type(end) is int and 0 <= start <= end < len(points) and
            all(point.get("road_option") == "RIGHT" for point in points[start:end + 1]),
            "selected RIGHT block no longer matches serialized route")
    transitions = []
    for point in points[start:end + 1]:
        triple = tuple(point.get(key) for key in ("road_id", "section_id", "lane_id"))
        require(all(type(value) is int for value in triple), "missing selected turn lane ID")
        if not transitions or transitions[-1] != triple:
            transitions.append(triple)
    center = points[(start + end) // 2]
    xy = tuple(float(center[key]) for key in ("x", "y"))
    require(all(math.isfinite(value) for value in xy), "nonfinite turn center")
    return tuple(transitions), xy


def audit(owner_root: Path, output: Path) -> dict:
    plan, owner, catalog = completed_owner(owner_root)
    known, manifest, _ = prior.known_sites()
    import carla

    xodr = (prior.C_EPISODE / "active_map_opendrive.xodr").read_bytes()
    rows = []
    seen = set()
    for entry in catalog["routes"]:
        identity, relative = entry.get("id"), Path(entry.get("path", ""))
        expected_route_sha = entry.get("sha256")
        require(type(identity) is str and identity.startswith("town01_right_s") and
                identity not in seen and entry.get("status") == "ready" and
                not relative.is_absolute() and ".." not in relative.parts and
                relative.parts[:3] == ("routes", "town01", "right") and
                relative.name == identity + ".json" and
                type(expected_route_sha) is str and
                re.fullmatch(r"[0-9a-f]{64}", expected_route_sha) is not None,
                "route identity, path or SHA invalid")
        seen.add(identity)
        route, route_sha = read(owner_root / relative, expected_route_sha)
        require(route.get("town") == "Town01" and route.get("scenario") == "right" and
                route.get("coordinate_reference") == "base_link" and
                abs(float(route["route_length_m"]) - float(entry["route_length_m"])) <= 0.01,
                "serialized route identity/length changed")
        row = {"route_id": identity, "path": str((owner_root / relative).relative_to(ROOT)),
               "route_sha256": route_sha, "route_length_m": route["route_length_m"],
               "window": None, "selected_site": None, "known_site_matches": [],
               "nominal": None, "exclusion_reasons": [], "capture_authorized": False,
               "dataset_admitted": False}
        try:
            screen = window.select_window(route)
        except window.WindowError as error:
            row["window"] = {"status": "UNSUPPORTED", "reason": str(error)}
            row["exclusion_reasons"].append("static_window_unsupported")
        else:
            row["window"] = screen
            if screen["status"] != "STATIC_WINDOW_CANDIDATE":
                row["exclusion_reasons"].append("no_eligible_fixed_turn_window")
            else:
                selected = screen["selected"]
                site = fingerprint(route, selected)
                matches = [{"source": old["source"], "route_id": old["route_id"]}
                           for old in known if prior.same_site(site, old["fingerprint"])]
                row["selected_site"] = {"turn_lane_transitions": [list(part) for part in site[0]],
                                        "turn_center_xy_ros_m": list(site[1])}
                row["known_site_matches"] = matches
                if matches:
                    row["exclusion_reasons"].append("existing_physical_turn_site")
                if math.dist(site[1], known[0]["fingerprint"][1]) < 30.0:
                    row["exclusion_reasons"].append("within_30m_of_historical_TRAIN_turn")
                try:
                    nominal = prior.provisional_nominal_clearance(
                        route, manifest, xodr, carla,
                        start_m=selected["window_start_m"], end_m=selected["window_end_m"])
                    row["nominal"] = nominal
                    if nominal["status"] != "PASS":
                        row["exclusion_reasons"].append("nominal_body_or_sweep_outside_10cm_inner_lane")
                    if not nominal["production_lane_gate_supported"]:
                        row["exclusion_reasons"].append("production_lane_gate_rejects_zero_arc_lane_transition")
                except Exception as error:
                    row["nominal"] = {"status": "UNSUPPORTED", "reason": str(error)}
                    row["exclusion_reasons"].append("nominal_geometry_unsupported")
        rows.append(row)
    groups = []
    for row in rows:
        if row["selected_site"] is None:
            continue
        site = (tuple(tuple(triple) for triple in row["selected_site"]["turn_lane_transitions"]),
                tuple(row["selected_site"]["turn_center_xy_ros_m"]))
        group = next((group for group in groups if prior.same_site(site, group["site"])), None)
        if group is None:
            group = {"site": site, "routes": []}
            groups.append(group)
        group["routes"].append(row)
    ranked_sites = []
    for group in groups:
        members = group["routes"]
        eligible = [row for row in members if not row["exclusion_reasons"]]
        eligible.sort(key=lambda row: (-row["nominal"]["minimum_inset_sweep_clearance_m"],
                                       row["route_id"]))
        ranked_sites.append({"selected_site": members[0]["selected_site"],
                             "route_ids": [row["route_id"] for row in members],
                             "nominal_pass_route_ids": [row["route_id"] for row in eligible],
                             "provisional_top_route_id": eligible[0]["route_id"] if eligible else None,
                             "capture_authorized": False})
    ranked_sites.sort(key=lambda group: group["route_ids"][0])
    provisional = sum(item["provisional_top_route_id"] is not None for item in ranked_sites)
    report = {"schema": "portable_e2e.town01_multi_turn_window_rank.v1",
              "status": "OFFLINE_PREREGISTRATION_CANDIDATES" if provisional else
              "NO_NOMINAL_SAFE_NOVEL_CANDIDATE",
              "owner_plan_sha256": prior.digest(owner_root / "owner_plan.json"),
              "owner_result_sha256": prior.digest(owner_root / "owner_result.json"),
              "catalog_sha256": owner["catalog_sha256"],
              "selector_source_sha256": prior.digest(ROOT / "tools/e2e/select_turn_centered_subwindow.py"),
              "historical_train_route_sha256": source.REFERENCE_SHA256,
              "c_p06_route_sha256": prior.digest(prior.C_EPISODE / "route.json"),
              "live_town01_xodr_sha256": prior.MAP_SHA256,
              "prius_body_manifest_sha256": prior.digest(prior.C_EPISODE / "manifest.json"),
              "right_route_count": len(rows),
              "static_window_candidate_count": sum(row["window"]["status"] == "STATIC_WINDOW_CANDIDATE"
                                                   for row in rows),
              "selected_physical_site_count": len(groups),
              "provisional_site_count": provisional,
              "ranked_sites": ranked_sites, "routes": rows,
              "capture_executed": False, "capture_authorized": False,
              "dataset_admitted": False, "training_executed": False,
              "learned_driving_tested": False}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    try:
        report = audit(OWNER, args.output)
    except (prior.InvalidEvidence, ValueError) as error:
        parser.exit(2, f"STOP_INVALID_EVIDENCE: {error}\n")
    print(json.dumps({key: report[key] for key in
                      ("status", "right_route_count", "static_window_candidate_count",
                       "selected_physical_site_count", "provisional_site_count",
                       "capture_authorized")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
