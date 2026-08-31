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
TURN_MANEUVER_OPTIONS = frozenset(
    {
        "LEFT",
        "RIGHT",
        "STRAIGHT",
        "CHANGELANELEFT",
        "CHANGELANERIGHT",
    }
)


class MatrixError(RuntimeError):
    """Raised when matrix evidence violates its reproducibility contract."""


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
    profile = value["runtime_profile"]
    if not isinstance(profile, dict):
        raise MatrixError("runtime_profile must be an object")
    expected_options = ["--recommended", "--visualize", "--capture-desktop"]
    if profile.get("wrapper_options") != expected_options:
        raise MatrixError(
            f"runtime profile must fix wrapper_options to {expected_options}"
        )
    if profile.get("client_map_loading_allowed") is not False:
        raise MatrixError("client-side CARLA map loading must remain disabled")
    if profile.get("map_lifecycle") != "cold_start_owned_process_group":
        raise MatrixError("matrix requires an owned cold-start CARLA lifecycle")
    trials = value["route_contract"].get("trials")
    if not isinstance(trials, list) or [item.get("id") for item in trials] != list(
        TRIAL_IDS
    ):
        raise MatrixError("route_contract must define straight then turn trials")
    bundles = value["validated_full_map_bundles"]
    if not isinstance(bundles, dict) or any(
        not MAP_ID_RE.fullmatch(str(map_id)) for map_id in bundles
    ):
        raise MatrixError("validated_full_map_bundles has an unsafe map id")
    return value, path


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
    matrix: Mapping[str, Any], matrix_path: Path
) -> dict[str, Any]:
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
        }
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
        "runtime_profile": matrix["runtime_profile"],
        "route_contract": matrix["route_contract"],
        "canonical_map_count": len(maps),
        "runnable_map_count": sum(item["runnable"] for item in maps),
        "maps": maps,
    }
    plan["admission_contract_sha256"] = sha256_json(
        {
            "matrix_id": plan["matrix_id"],
            "matrix_manifest_sha256": plan["matrix_manifest_sha256"],
            "canonical_map_manifest_sha256": plan[
                "canonical_map_manifest_sha256"
            ],
            "runtime_profile": plan["runtime_profile"],
            "route_contract": plan["route_contract"],
            "maps": plan["maps"],
        }
    )
    return plan


def _status_path(output_root: Path, map_id: str) -> Path:
    if not MAP_ID_RE.fullmatch(map_id):
        raise MatrixError(f"unsafe map id: {map_id}")
    return output_root / "maps" / map_id / "status.json"


def prepare_output(
    matrix_path: Path, output_root: Path, resume: bool
) -> dict[str, Any]:
    matrix, matrix_path = load_matrix(matrix_path)
    candidate = build_campaign_plan(matrix, matrix_path)
    output_root = output_root.expanduser().resolve()
    plan_path = output_root / "matrix_plan.json"
    if plan_path.exists():
        if not resume:
            raise MatrixError(f"matrix output exists; pass --resume: {output_root}")
        existing = read_object(plan_path, "existing matrix plan")
        for field in (
            "matrix_id",
            "matrix_manifest_sha256",
            "canonical_map_manifest_sha256",
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
        plan = candidate
    for entry in plan["maps"]:
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
    """Independently recompute isolated-turn geometry from serialized ROS poses."""
    points = payload.get("route")
    if not isinstance(points, list) or len(points) < 2:
        raise MatrixError("custom-map turn geometry needs at least two route points")
    if scenario not in ("left", "right"):
        raise MatrixError("custom-map turn geometry requires left or right scenario")
    required = CUSTOM_MAP_TURN_GEOMETRY_CONTRACT
    target = scenario.upper()
    options = []
    distances = []
    yaws = []
    for index, point in enumerate(points):
        if not isinstance(point, dict):
            raise MatrixError(f"custom-map route point {index} is not an object")
        option = point.get("road_option")
        distance = point.get("distance_m")
        yaw = point.get("yaw")
        if not isinstance(option, str):
            raise MatrixError(f"custom-map route point {index} has no road option")
        if not isinstance(distance, (int, float)) or not math.isfinite(
            float(distance)
        ):
            raise MatrixError(f"custom-map route point {index} has invalid distance")
        if not isinstance(yaw, (int, float)) or not math.isfinite(float(yaw)):
            raise MatrixError(f"custom-map route point {index} has invalid yaw")
        if distances and float(distance) < distances[-1]:
            raise MatrixError("custom-map route distance_m is not monotonic")
        options.append(option)
        distances.append(float(distance))
        yaws.append(float(yaw))

    indices = [index for index, option in enumerate(options) if option == target]
    blocks: list[list[int]] = []
    for index in indices:
        if not blocks or index != blocks[-1][-1] + 1:
            blocks.append([index])
        else:
            blocks[-1].append(index)
    additional = sorted((set(options) & TURN_MANEUVER_OPTIONS) - {target})
    reasons = []
    if len(blocks) != 1:
        reasons.append(
            f"expected exactly one contiguous {target} block, found {len(blocks)}"
        )
    if additional:
        reasons.append("additional maneuver commands present: " + ", ".join(additional))

    def block_geometry(block: Sequence[int]) -> dict[str, Any]:
        start_index = block[0]
        end_index = block[-1]
        support_start = max(0, start_index - 1)
        support_end = min(len(points) - 1, end_index + 1)
        block_start = (
            0.5 * (distances[start_index - 1] + distances[start_index])
            if start_index
            else distances[start_index]
        )
        block_end = (
            0.5 * (distances[end_index] + distances[end_index + 1])
            if end_index + 1 < len(points)
            else distances[end_index]
        )
        unwrapped = [yaws[support_start]]
        for yaw in yaws[support_start + 1 : support_end + 1]:
            unwrapped.append(unwrapped[-1] + _normalize_angle(yaw - unwrapped[-1]))
        net_heading_rad = unwrapped[-1] - unwrapped[0]
        net_heading_deg = abs(math.degrees(net_heading_rad))
        cumulative_heading_rad = sum(
            abs(unwrapped[index] - unwrapped[index - 1])
            for index in range(1, len(unwrapped))
        )
        cumulative_heading_deg = math.degrees(cumulative_heading_rad)
        direction = 1.0 if net_heading_rad >= 0.0 else -1.0
        progress = [
            (
                distances[support_start + index],
                max(0.0, math.degrees(direction * (yaw - unwrapped[0]))),
            )
            for index, yaw in enumerate(unwrapped)
        ]
        curvatures = []
        for index in range(support_start + 1, support_end + 1):
            segment_length = distances[index] - distances[index - 1]
            if segment_length <= 1.0e-6:
                continue
            curvatures.append(
                abs(_normalize_angle(yaws[index] - yaws[index - 1]))
                / segment_length
            )
        arc_length = block_end - block_start
        return {
            "start_index": start_index,
            "end_index": end_index,
            "command_arc_length_m": arc_length,
            "absolute_net_heading_change_deg": net_heading_deg,
            "cumulative_absolute_heading_change_deg": cumulative_heading_deg,
            "heading_excess_deg": max(
                0.0, cumulative_heading_deg - net_heading_deg
            ),
            "mean_absolute_curvature_per_m": (
                cumulative_heading_rad / arc_length if arc_length > 0.0 else math.inf
            ),
            "p95_absolute_curvature_per_m": _linear_percentile(
                curvatures, 95.0
            ),
            "block_start_distance_m": block_start,
            "block_end_distance_m": block_end,
            "_heading_progress": progress,
        }

    block_metrics = [block_geometry(block) for block in blocks]
    selected = block_metrics[0] if len(block_metrics) == 1 else None
    if selected is not None:
        margin = float(required["alignment_heading_margin_deg"])
        net_heading = float(selected["absolute_net_heading_change_deg"])
        lead_crossing = _first_progress_crossing(
            selected["_heading_progress"], margin
        )
        tail_crossing = _first_progress_crossing(
            selected["_heading_progress"], max(0.0, net_heading - margin)
        )
        command_lead = (
            math.inf
            if lead_crossing is None
            else max(
                0.0, lead_crossing - float(selected["block_start_distance_m"])
            )
        )
        command_tail = (
            math.inf
            if tail_crossing is None
            else max(0.0, float(selected["block_end_distance_m"]) - tail_crossing)
        )
        selected["command_lead_distance_m"] = command_lead
        selected["command_tail_distance_m"] = command_tail
        checks = (
            (
                float(selected["command_arc_length_m"])
                < float(required["minimum_arc_length_m"]),
                "turn command arc length is below the minimum",
            ),
            (
                float(selected["command_arc_length_m"])
                > float(required["maximum_arc_length_m"]),
                "turn command arc length exceeds the maximum",
            ),
            (
                net_heading < float(required["minimum_heading_change_deg"]),
                "turn heading change is below the minimum",
            ),
            (
                net_heading > float(required["maximum_heading_change_deg"]),
                "turn heading change exceeds the maximum",
            ),
            (
                float(selected["heading_excess_deg"])
                > float(required["maximum_heading_excess_deg"]),
                "turn heading excess exceeds the maximum",
            ),
            (
                command_lead > float(required["maximum_command_lead_m"]),
                "turn command lead distance exceeds the maximum",
            ),
            (
                command_tail > float(required["maximum_command_tail_m"]),
                "turn command tail distance exceeds the maximum",
            ),
            (
                float(selected["p95_absolute_curvature_per_m"])
                > float(required["maximum_p95_abs_curvature_per_m"]),
                "turn p95 absolute curvature exceeds the maximum",
            ),
        )
        reasons.extend(message for failed, message in checks if failed)

    for metrics in block_metrics:
        metrics.pop("_heading_progress", None)
    if reasons:
        raise MatrixError("custom-map turn-geometry preflight failed: " + "; ".join(reasons))
    return {
        "status": "PASS",
        "scenario": scenario,
        "directional_command": target,
        "directional_block_count": len(blocks),
        "additional_maneuver_commands": additional,
        "selected_block": selected,
        "directional_blocks": block_metrics,
        "limits": dict(required),
        "failure_reasons": [],
    }


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
    return {
        "town": expected_town,
        "scenario": scenario,
        "route_length_m": float(route_length),
        "start_spawn_index": payload.get("start_spawn_index"),
        "goal_spawn_index": payload.get("goal_spawn_index"),
        "option_counts": dict(counts),
        "vad_command_counts": {str(key): value for key, value in sorted(commands.items())},
        "point_count": len(points),
    }


def select_routes(output_root: Path, map_id: str, catalog_path: Path) -> dict[str, Any]:
    output_root = output_root.expanduser().resolve()
    plan = read_object(output_root / "matrix_plan.json", "matrix plan")
    entry = _map_entry(plan, map_id)
    if entry.get("runnable") is not True:
        raise MatrixError(f"map {map_id} is blocked by full-map admission")
    catalog_path = catalog_path.expanduser().resolve()
    catalog = read_object(catalog_path, "route catalog")
    if catalog.get("status") != "complete" or catalog.get("map_id") != map_id:
        raise MatrixError("route catalog is not complete for the requested map")
    server = catalog.get("server")
    if not isinstance(server, dict):
        raise MatrixError("route catalog server provenance is missing")
    if server.get("map_load_allowed") is not False or server.get("map_load_performed") is not False:
        raise MatrixError("route catalog used or allowed client-side map loading")
    active_name = str(server.get("active_map_name", "")).rstrip("/").rsplit("/", 1)[-1]
    if active_name != entry["canonical_name"]:
        raise MatrixError(
            f"route catalog active map mismatch: {active_name!r} != {entry['canonical_name']!r}"
        )
    bundle = entry.get("full_map_bundle")
    custom_map = (
        isinstance(bundle, dict) and bundle.get("bundle_schema") == "custom_map"
    )
    if custom_map:
        _validate_catalog_initial_approach_contract(catalog)
        _validate_catalog_turn_geometry_contract(catalog)
    routes = catalog.get("routes")
    if not isinstance(routes, list):
        raise MatrixError("route catalog routes must be a list")
    selected: list[dict[str, Any]] = []
    for trial_id, scenarios in (("straight", ("straight",)), ("turn", ("left", "right"))):
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

        def maneuver_key(route: Mapping[str, Any]) -> tuple[Any, ...]:
            value = route.get("path")
            if not isinstance(value, str) or not value:
                return (2, scenarios.index(str(route.get("scenario"))))
            route_path = _safe_inside(
                catalog_path.parent, catalog_path.parent / value, "route"
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
                1 if lane_changes else 0,
                scenarios.index(str(route.get("scenario"))),
                int(route.get("seed", 0)),
                int(route.get("pair_index", 0)),
                str(route.get("id", "")),
            )

        candidates.sort(key=maneuver_key)
        route = candidates[0]
        value = route.get("path")
        if not isinstance(value, str) or not value:
            raise MatrixError(f"catalog {trial_id} route has no path")
        route_path = _safe_inside(catalog_path.parent, catalog_path.parent / value, "route")
        if not route_path.is_file():
            raise MatrixError(f"catalog route is missing: {route_path}")
        route_hash = sha256_file(route_path)
        if route_hash != route.get("sha256"):
            raise MatrixError(f"catalog route SHA256 mismatch: {route_path}")
        payload = read_object(route_path, f"{trial_id} route")
        analysis = _validate_route_payload(
            payload, entry["canonical_name"], str(route["scenario"])
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
                analysis["turn_geometry_preflight"] = (
                    _custom_route_turn_geometry(payload, str(route["scenario"]))
                )
        readiness = entry["full_map_bundle"].get("readiness")
        if isinstance(readiness, dict):
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
                "route_path": str(route_path),
                "route_sha256": route_hash,
                "analysis": analysis,
            }
        )
    payload = {
        "schema_version": 1,
        "matrix_id": plan["matrix_id"],
        "map_id": map_id,
        "canonical_name": entry["canonical_name"],
        "catalog_path": str(catalog_path),
        "catalog_sha256": sha256_file(catalog_path),
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
    atomic_json(destination, payload)
    return payload


def select_admitted_routes(output_root: Path, map_id: str) -> dict[str, Any]:
    """Materialize the fixed seed routes from the packaged-Town preflight.

    This is an offline analysis plan only. The runner still cold-starts CARLA,
    regenerates the same seed catalog without client map loading, and replaces
    this file through :func:`select_routes` before any VAD trial starts.
    """
    output_root = output_root.expanduser().resolve()
    plan = read_object(output_root / "matrix_plan.json", "matrix plan")
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
            values[key] = value
    return values


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


def validate_trial(
    output_root: Path, map_id: str, trial_id: str, trial_dir: Path
) -> dict[str, Any]:
    if trial_id not in TRIAL_IDS:
        raise MatrixError(f"unknown trial id: {trial_id}")
    output_root = output_root.expanduser().resolve()
    trial_dir = _safe_inside(output_root, trial_dir, "trial directory")
    plan = read_object(output_root / "matrix_plan.json", "matrix plan")
    entry = _map_entry(plan, map_id)
    route_matrix = read_object(
        output_root / "maps" / map_id / "route_matrix.json", "route matrix"
    )
    route_entry = next(
        (item for item in route_matrix.get("trials", []) if item.get("trial_id") == trial_id),
        None,
    )
    if route_entry is None:
        raise MatrixError(f"route matrix has no {trial_id} trial")
    required_files = (
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
    source_route = read_object(trial_dir / "source_route.json", "trial source route")
    scenario = str(route_entry["catalog_scenario"])
    route_analysis = _validate_route_payload(
        source_route, entry["canonical_name"], scenario
    )
    if sha256_file(trial_dir / "source_route.json") != route_entry["route_sha256"]:
        raise MatrixError("trial source route differs from the selected route contract")
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
    validation = {
        "schema_version": 1,
        "status": "PASS",
        "validated_at": utc_now(),
        "matrix_id": plan["matrix_id"],
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
        "runtime_profile": plan["runtime_profile"],
        "desktop_capture": desktop,
    }
    atomic_json(trial_dir / "matrix_validation.json", validation)
    return validation


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
    status_path = _status_path(output_root, map_id)
    value = read_object(status_path, f"{map_id} status")
    if value.get("runnable") is not True:
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
        if all(state == "PASS" for state in states):
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
        value["status"] = status_value
        value["stage"] = stage
        value["reason"] = reason
    value["updated_at"] = utc_now()
    atomic_json(status_path, value)
    summarize(output_root)
    return value


def summarize(output_root: Path) -> dict[str, Any]:
    output_root = output_root.expanduser().resolve()
    plan = read_object(output_root / "matrix_plan.json", "matrix plan")
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
    aggregate = {
        "schema_version": 1,
        "matrix_id": plan["matrix_id"],
        "generated_at": utc_now(),
        "status": overall,
        "canonical_map_count": len(maps),
        "runnable_map_count": len(runnable),
        "runnable_pass_count": sum(item["status"] == "PASS" for item in runnable),
        "blocked_map_count": sum(item["status"] == "BLOCKED" for item in maps),
        "status_counts": dict(sorted(counts.items())),
        "runtime_profile": plan["runtime_profile"],
        "route_contract": plan["route_contract"],
        "maps": maps,
    }
    atomic_json(output_root / "aggregate.json", aggregate)
    lines = [
        "# Autoware VAD straight/turn town matrix",
        "",
        f"Overall: **{overall}** — runnable PASS "
        f"**{aggregate['runnable_pass_count']}/{len(runnable)}**, "
        f"blocked **{aggregate['blocked_map_count']}**.",
        "",
        "All runnable trials use the same fixed `--recommended --visualize "
        "--capture-desktop` profile. CARLA is cold-started per map and "
        "`client.load_world` is forbidden.",
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
    prepare.add_argument("--resume", action="store_true")
    listing = sub.add_parser("list-runnable")
    listing.add_argument("--output-root", type=Path, required=True)
    select = sub.add_parser("select-routes")
    select.add_argument("--output-root", type=Path, required=True)
    select.add_argument("--map-id", required=True)
    select.add_argument("--catalog", type=Path, required=True)
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
            value = prepare_output(args.manifest, args.output_root, args.resume)
            print(
                f"PLAN maps={value['canonical_map_count']} "
                f"runnable={value['runnable_map_count']} "
                f"output={args.output_root.resolve()}"
            )
        elif args.command == "list-runnable":
            value = read_object(args.output_root.resolve() / "matrix_plan.json", "matrix plan")
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
            value = select_routes(args.output_root, args.map_id, args.catalog)
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
