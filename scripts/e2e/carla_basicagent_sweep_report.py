#!/usr/bin/env python3
"""Maintain durable status and summaries for the packaged CARLA map sweep."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import tempfile
from typing import Any, Mapping, Sequence

try:
    from inventory_carla_training_maps import DEFAULT_MANIFEST, load_manifest
except ModuleNotFoundError:
    from scripts.e2e.inventory_carla_training_maps import (
        DEFAULT_MANIFEST,
        load_manifest,
    )


EVIDENCE_KIND = "CARLA BasicAgent six-camera route smoke"
EVIDENCE_DISCLAIMER = (
    "This is CARLA BasicAgent evidence, not Autoware VAD inference or "
    "closed-loop control evidence."
)
SUCCESS_STATUSES = frozenset(("PASS", "SKIP_RESUME_VALIDATED"))


class ReportError(RuntimeError):
    pass


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def atomic_write_json(path: Path, payload: Mapping[str, Any]) -> Path:
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=False, allow_nan=False)
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
    return path


def atomic_write_text(path: Path, text: str) -> Path:
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise
    return path


def read_json(path: Path, *, required: bool = False) -> dict[str, Any] | None:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        if required:
            raise ReportError(f"required JSON file is missing: {path}")
        return None
    except (OSError, json.JSONDecodeError) as error:
        if required:
            raise ReportError(f"cannot read JSON file {path}: {error}") from error
        return None
    if not isinstance(payload, dict):
        if required:
            raise ReportError(f"JSON root must be an object: {path}")
        return None
    return payload


def parse_map_ids(text: str) -> tuple[str, ...]:
    values = tuple(item.strip() for item in text.split(",") if item.strip())
    if not values or len(set(values)) != len(values):
        raise ReportError("selected maps must be a non-empty unique comma-separated list")
    return values


def _inventory_maps(inventory: Mapping[str, Any] | None) -> dict[str, Mapping[str, Any]]:
    if not inventory:
        return {}
    static = inventory.get("static")
    if not isinstance(static, Mapping):
        return {}
    maps = static.get("maps")
    if not isinstance(maps, list):
        return {}
    return {
        entry["id"]: entry
        for entry in maps
        if isinstance(entry, Mapping) and isinstance(entry.get("id"), str)
    }


def initialize_statuses(
    output_root: Path,
    manifest: Mapping[str, Any],
    inventory: Mapping[str, Any] | None,
    selected_map_ids: Sequence[str],
    host: str,
    port: int,
    invocation_map_ids: Sequence[str] | None = None,
) -> list[Path]:
    output_root = output_root.expanduser().resolve()
    requested = frozenset(selected_map_ids)
    invocation = frozenset(invocation_map_ids or selected_map_ids)
    manifest_ids = {entry["id"] for entry in manifest["maps"]}
    unknown = sorted((requested | invocation) - manifest_ids)
    if unknown:
        raise ReportError(f"selected map ids are not in the manifest: {unknown}")
    outside_campaign = sorted(invocation - requested)
    if outside_campaign:
        raise ReportError(
            f"invocation map ids are not in the selected campaign: {outside_campaign}"
        )
    previous_by_id: dict[str, Mapping[str, Any]] = {}
    for entry in manifest["maps"]:
        map_id = entry["id"]
        status_path = output_root / "maps" / map_id / "status.json"
        previous = read_json(status_path)
        if isinstance(previous, Mapping):
            previous_by_id[map_id] = previous
    selected = requested
    inventory_by_id = _inventory_maps(inventory)
    now = utc_now()
    paths = []
    for entry in manifest["maps"]:
        map_id = entry["id"]
        status_path = output_root / "maps" / map_id / "status.json"
        previous = previous_by_id.get(map_id)
        exit_code = None
        route_seed = 0
        if map_id not in invocation and isinstance(previous, Mapping):
            previous_configuration = previous.get("configuration")
            if isinstance(previous_configuration, Mapping):
                previous_route_seed = previous_configuration.get("route_seed")
                if (
                    isinstance(previous_route_seed, int)
                    and not isinstance(previous_route_seed, bool)
                    and previous_route_seed >= 0
                ):
                    route_seed = previous_route_seed
        if map_id in invocation:
            status = "PENDING"
            stage = "queued"
            reason = "Selected for an Epic-quality packaged CARLA cold-start smoke."
        elif map_id in selected:
            validated, validation_error = validated_job(output_root, map_id)
            plan = read_json(
                output_root / "maps" / map_id / "smoke/collection_plan.json"
            )
            if validation_error is None:
                route_seed = int(validated["route_seed"])
                status = "SKIP_RESUME_VALIDATED"
                stage = "resume_validated"
                reason = (
                    "Existing complete episode, validated export, PNG, and GIF were "
                    "revalidated; this map was not rerun in the current invocation."
                )
            elif isinstance(plan, Mapping) and plan.get("status") == "FAILED":
                status = "FAILED"
                stage = "prior_attempt_failed"
                failed_reasons = [
                    str(job.get("reason"))
                    for job in plan.get("jobs", [])
                    if isinstance(job, Mapping)
                    and job.get("status") == "FAILED"
                    and job.get("reason")
                ]
                reason = (
                    "; ".join(failed_reasons)
                    or "The previous collection plan failed artifact validation."
                )
                exit_code = 1
            elif isinstance(previous, Mapping) and previous.get("status") == "FAILED":
                status = "FAILED"
                stage = str(previous.get("stage") or "prior_attempt_failed")
                reason = str(previous.get("reason") or validation_error)
                exit_code = previous.get("exit_code", 1)
            else:
                status = "PENDING"
                stage = "not_selected_this_invocation"
                reason = (
                    "Previously selected campaign map has no complete validated evidence; "
                    "it was not rerun in the current invocation."
                )
        elif entry["status"] == "unavailable":
            status = "EXCLUDED"
            stage = "not_run"
            reason = entry["reason"]
        elif entry["status"] == "source_editor_required":
            status = "SOURCE_EDITOR_REQUIRED"
            stage = "not_run"
            reason = entry["reason"]
        else:
            status = "BLOCKED"
            stage = "not_run"
            binding = inventory_by_id.get(map_id, {})
            if binding.get("assets_complete") is False:
                reason = "Static inventory reports missing packaged assets. " + entry["reason"]
            else:
                reason = "Not selected for this safe packaged-map sweep invocation."
        payload = {
            "schema_version": 1,
            "evidence_kind": EVIDENCE_KIND,
            "evidence_disclaimer": EVIDENCE_DISCLAIMER,
            "map_id": map_id,
            "canonical_name": entry["canonical_name"],
            "load_name": entry["load_name"],
            "manifest_status": entry["status"],
            "manifest_split": entry["split"],
            "manifest_reason": entry["reason"],
            "static_inventory": dict(inventory_by_id.get(map_id, {})),
            "selected": map_id in selected,
            "selected_this_invocation": map_id in invocation,
            "status": status,
            "stage": stage,
            "reason": reason,
            "configuration": {
                "host": host,
                "port": port,
                "quality": "Epic",
                "cold_start_per_map": True,
                "client_map_loading_allowed": False,
                "route_seed": route_seed,
                "pairs_per_seed": 1,
                "scenario": "lane_follow",
                "weather": "ClearNoon",
                "collection_seed": 0,
            },
            "paths": {
                "status": str(status_path),
                "server_log": str(status_path.parent / "server.log"),
                "live_inventory": str(status_path.parent / "inventory.json"),
                "inventory_log": str(status_path.parent / "inventory.log"),
                "catalog_log": str(status_path.parent / "catalog.log"),
                "route_catalog": str(status_path.parent / "catalog/route_catalog.json"),
                "suite_log": str(status_path.parent / "suite.log"),
                "collection_plan": str(status_path.parent / "smoke/collection_plan.json"),
            },
            "created_at": (
                previous.get("created_at")
                if isinstance(previous, Mapping) and previous.get("created_at")
                else now
            ),
            "updated_at": now,
        }
        if exit_code is not None:
            payload["exit_code"] = exit_code
        atomic_write_json(status_path, payload)
        for log_name in ("server.log", "catalog.log", "suite.log"):
            log_path = status_path.parent / log_name
            if not log_path.exists():
                atomic_write_text(
                    log_path,
                    f"status={status} stage={stage} map={map_id} reason={reason}\n",
                )
        paths.append(status_path)
    return paths


def update_status(
    output_root: Path,
    map_id: str,
    status: str,
    stage: str,
    reason: str | None,
    exit_code: int | None,
    route_seed: int | None = None,
) -> Path:
    path = output_root.expanduser().resolve() / "maps" / map_id / "status.json"
    payload = read_json(path, required=True)
    assert payload is not None
    if not payload.get("selected"):
        raise ReportError(f"refusing to run non-selected map {map_id}")
    if isinstance(route_seed, bool) or (route_seed is not None and route_seed < 0):
        raise ReportError("route seed must be a nonnegative integer")
    payload["status"] = status
    payload["stage"] = stage
    payload["reason"] = reason
    payload["updated_at"] = utc_now()
    if exit_code is not None:
        payload["exit_code"] = exit_code
    elif "exit_code" in payload:
        del payload["exit_code"]
    if route_seed is not None:
        configuration = payload.get("configuration")
        if not isinstance(configuration, dict):
            raise ReportError("status configuration is missing")
        configuration["route_seed"] = route_seed
    return atomic_write_json(path, payload)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while block := stream.read(1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def _contained_path(path_value: Any, allowed_root: Path) -> Path | None:
    if not isinstance(path_value, str):
        return None
    if not path_value.strip():
        return None
    path = Path(path_value).expanduser().resolve()
    try:
        path.relative_to(allowed_root)
    except ValueError:
        return None
    return path


def _artifact(
    path_value: Any,
    output_root: Path,
    allowed_root: Path,
    signatures: Sequence[bytes],
) -> dict[str, Any] | None:
    path = _contained_path(path_value, allowed_root)
    if path is None:
        return None
    if not path.is_file():
        return None
    try:
        size = path.stat().st_size
        if size <= 0:
            return None
        with path.open("rb") as stream:
            header = stream.read(max(map(len, signatures)))
        if not any(header.startswith(signature) for signature in signatures):
            return None
        relative = path.relative_to(output_root).as_posix()
    except OSError:
        return None
    return {
        "path": str(path),
        "relative_path": relative,
        "bytes": size,
        "sha256": _sha256(path),
    }


def validated_job(
    output_root: Path, map_id: str, expected_route_seed: int | None = None
) -> tuple[dict[str, Any], str | None]:
    plan_path = output_root / "maps" / map_id / "smoke/collection_plan.json"
    smoke_root = plan_path.parent.resolve()
    plan = read_json(plan_path)
    if not plan:
        return {}, f"collection plan is missing or invalid: {plan_path}"
    if plan.get("status") != "COMPLETE":
        return {"plan": plan}, f"collection plan status is {plan.get('status')!r}"
    jobs = plan.get("jobs")
    if not isinstance(jobs, list) or len(jobs) != 1 or not isinstance(jobs[0], dict):
        return {"plan": plan}, "lane-follow smoke must contain exactly one suite job"
    job = jobs[0]
    if job.get("status") not in ("COMPLETE", "SKIP_RESUME_VALIDATED"):
        return {"plan": plan, "job": job}, f"suite job status is {job.get('status')!r}"
    expected_contract = {
        "map_id": map_id,
        "scenario": "lane_follow",
        "weather": "ClearNoon",
        "seed": 0,
    }
    mismatches = {
        field: {"expected": expected, "actual": job.get(field)}
        for field, expected in expected_contract.items()
        if job.get(field) != expected
    }
    if mismatches:
        return {"plan": plan, "job": job}, f"suite job contract mismatch: {mismatches}"
    catalog_path = output_root / "maps" / map_id / "catalog/route_catalog.json"
    catalog = read_json(catalog_path)
    if not catalog or catalog.get("status") != "complete":
        return {"plan": plan, "job": job}, "route catalog is missing or incomplete"
    if catalog.get("map_id") != map_id:
        return {"plan": plan, "job": job, "catalog": catalog}, "route catalog map mismatch"
    catalog_server = catalog.get("server")
    if not isinstance(catalog_server, Mapping):
        return {"plan": plan, "job": job, "catalog": catalog}, "route catalog server contract is missing"
    if catalog_server.get("map_load_allowed") is not False:
        return {"plan": plan, "job": job, "catalog": catalog}, "route catalog allowed client-side map loading"
    if catalog_server.get("map_load_performed") is not False:
        return {"plan": plan, "job": job, "catalog": catalog}, "route catalog performed client-side map loading"
    generation = catalog.get("generation")
    route_seeds = generation.get("seeds") if isinstance(generation, Mapping) else None
    if (
        not isinstance(route_seeds, list)
        or len(route_seeds) != 1
        or isinstance(route_seeds[0], bool)
        or not isinstance(route_seeds[0], int)
        or route_seeds[0] < 0
    ):
        return {"plan": plan, "job": job, "catalog": catalog}, "route catalog must contain one nonnegative deterministic seed"
    route_seed = route_seeds[0]
    if expected_route_seed is not None and route_seed != expected_route_seed:
        return {
            "plan": plan,
            "job": job,
            "catalog": catalog,
        }, f"route seed mismatch: expected {expected_route_seed}, actual {route_seed}"
    routes = catalog.get("routes")
    matching_routes = [
        route
        for route in routes
        if isinstance(route, Mapping) and route.get("id") == job.get("route_id")
    ] if isinstance(routes, list) else []
    if len(matching_routes) != 1:
        return {"plan": plan, "job": job, "catalog": catalog}, "suite route is not unique in the route catalog"
    route = matching_routes[0]
    if route.get("scenario") != "lane_follow" or route.get("seed") != route_seed:
        return {"plan": plan, "job": job, "catalog": catalog}, "suite route does not match the deterministic lane-follow contract"
    server = plan.get("server")
    if not isinstance(server, Mapping):
        return {"plan": plan, "job": job}, "suite server contract is missing"
    if server.get("map_load_allowed") is not False:
        return {"plan": plan, "job": job}, "suite allowed client-side map loading"
    if server.get("map_lifecycle_managed") is not False:
        return {"plan": plan, "job": job}, "suite performed client-side map loading"
    paths = job.get("paths")
    if not isinstance(paths, Mapping):
        return {"plan": plan, "job": job}, "suite job paths are missing"
    episode_path = _contained_path(paths.get("episode"), smoke_root)
    export_path = _contained_path(paths.get("export"), smoke_root)
    if episode_path is None or export_path is None:
        return {"plan": plan, "job": job}, "suite episode/export path escapes its map smoke root"
    episode_manifest = read_json(episode_path / "manifest.json")
    export_manifest = read_json(export_path / "manifest.json")
    png = _artifact(
        paths.get("preview_png"),
        output_root,
        smoke_root,
        (b"\x89PNG\r\n\x1a\n",),
    )
    gif = _artifact(
        paths.get("preview_gif"),
        output_root,
        smoke_root,
        (b"GIF87a", b"GIF89a"),
    )
    if not episode_manifest or episode_manifest.get("status") != "complete":
        return {"plan": plan, "job": job}, "episode manifest is not complete"
    result = episode_manifest.get("result")
    if not isinstance(result, Mapping) or result.get("goal_reached") is not True:
        return {"plan": plan, "job": job}, "episode did not reach the catalog goal"
    runtime = episode_manifest.get("runtime")
    if not isinstance(runtime, Mapping):
        return {"plan": plan, "job": job}, "episode runtime contract is missing"
    if runtime.get("client_map_loading_allowed") is not False:
        return {"plan": plan, "job": job}, "collector allowed client-side map loading"
    if runtime.get("client_map_loading_performed") is not False:
        return {"plan": plan, "job": job}, "collector performed client-side map loading"
    if not export_manifest or export_manifest.get("status") != "validated":
        return {"plan": plan, "job": job}, "export manifest is not validated"
    if export_manifest.get("collision_event_count") != 0:
        return {"plan": plan, "job": job}, "validated export contains collision events"
    if export_manifest.get("lane_invasion_event_count") != 0:
        return {"plan": plan, "job": job}, "validated export contains lane-invasion events"
    if png is None or gif is None:
        return {
            "plan": plan,
            "job": job,
        }, "valid in-root PNG and GIF previews are both required"
    return {
        "plan": plan,
        "job": job,
        "catalog": catalog,
        "route_seed": route_seed,
        "episode_manifest": episode_manifest,
        "export_manifest": export_manifest,
        "artifacts": {"png": png, "gif": gif},
    }, None


def map_record(output_root: Path, entry: Mapping[str, Any]) -> dict[str, Any]:
    map_id = entry["id"]
    status_path = output_root / "maps" / map_id / "status.json"
    status = read_json(status_path) or {
        "map_id": map_id,
        "canonical_name": entry["canonical_name"],
        "selected": False,
        "status": "BLOCKED",
        "stage": "not_initialized",
        "reason": "No sweep status was initialized.",
    }
    catalog_path = output_root / "maps" / map_id / "catalog/route_catalog.json"
    catalog = read_json(catalog_path)
    validated, validation_error = validated_job(output_root, map_id)
    record = dict(status)
    record["status_path"] = str(status_path)
    record["catalog"] = {
        "path": str(catalog_path),
        "status": catalog.get("status") if catalog else None,
        "route_count": len(catalog.get("routes", [])) if catalog else 0,
        "lane_follow_route_count": (
            sum(route.get("scenario") == "lane_follow" for route in catalog.get("routes", []))
            if catalog
            else 0
        ),
        "server_map_load_performed": (
            catalog.get("server", {}).get("map_load_performed") if catalog else None
        ),
    }
    if validated:
        plan = validated.get("plan", {})
        job = validated.get("job", {})
        episode = validated.get("episode_manifest", {})
        export = validated.get("export_manifest", {})
        record["smoke"] = {
            "plan_path": str(output_root / "maps" / map_id / "smoke/collection_plan.json"),
            "plan_status": plan.get("status"),
            "job_status": job.get("status"),
            "scenario": job.get("scenario"),
            "weather": job.get("weather"),
            "seed": job.get("seed"),
            "route_id": job.get("route_id"),
            "server": {
                "map_load_allowed": plan.get("server", {}).get("map_load_allowed"),
                "map_lifecycle_managed": plan.get("server", {}).get(
                    "map_lifecycle_managed"
                ),
            },
            "result": episode.get("result"),
            "export": {
                "sample_count": export.get("sample_count"),
                "maximum_route_cte_m": export.get("maximum_route_cte_m"),
                "collision_event_count": export.get("collision_event_count"),
                "lane_invasion_event_count": export.get("lane_invasion_event_count"),
            },
            "artifacts": validated.get("artifacts"),
        }
    else:
        record["smoke"] = None
    record["artifact_validation_error"] = validation_error
    if record.get("status") in SUCCESS_STATUSES and validation_error:
        record["recorded_status"] = record["status"]
        record["status"] = "FAILED"
        record["stage"] = "artifact_revalidation_failed"
        record["reason"] = validation_error
    return record


def build_summary(
    output_root: Path, manifest: Mapping[str, Any]
) -> tuple[dict[str, Any], str]:
    output_root = output_root.expanduser().resolve()
    maps = [map_record(output_root, entry) for entry in manifest["maps"]]
    selected = [entry for entry in maps if entry.get("selected")]
    invocation = [entry for entry in maps if entry.get("selected_this_invocation")]
    counts: dict[str, int] = {}
    for entry in maps:
        value = str(entry.get("status", "UNKNOWN"))
        counts[value] = counts.get(value, 0) + 1
    selected_success = sum(entry.get("status") in SUCCESS_STATUSES for entry in selected)
    if selected and selected_success == len(selected):
        overall = "COMPLETE"
    elif any(entry.get("status") == "FAILED" for entry in selected):
        overall = "FAILED"
    else:
        overall = "INCOMPLETE"
    summary = {
        "schema_version": 1,
        "generated_at": utc_now(),
        "status": overall,
        "evidence_kind": EVIDENCE_KIND,
        "evidence_disclaimer": EVIDENCE_DISCLAIMER,
        "output_root": str(output_root),
        "canonical_map_count": len(maps),
        "selected_map_count": len(selected),
        "selected_success_count": selected_success,
        "invocation_map_count": len(invocation),
        "status_counts": dict(sorted(counts.items())),
        "maps": maps,
    }
    lines = [
        "# CARLA BasicAgent packaged-map evidence",
        "",
        f"> **Scope:** {EVIDENCE_DISCLAIMER}",
        "> 이 결과는 CARLA BasicAgent 6-camera 주행 증거이며 Autoware VAD 추론/폐루프 제어 결과가 아닙니다.",
        "",
        f"Overall: **{overall}** — campaign {selected_success}/{len(selected)} successful; "
        f"current invocation maps {len(invocation)}; canonical maps {len(maps)}.",
        "",
        "| Map | Manifest | Sweep status | Result/artifacts | Reason |",
        "|---|---|---|---|---|",
    ]
    for entry in maps:
        smoke = entry.get("smoke")
        artifacts = smoke.get("artifacts", {}) if isinstance(smoke, Mapping) else {}
        artifact_links = []
        for label in ("png", "gif"):
            artifact = artifacts.get(label) if isinstance(artifacts, Mapping) else None
            if isinstance(artifact, Mapping) and artifact.get("relative_path"):
                artifact_links.append(f"[{label.upper()}]({artifact['relative_path']})")
        result_text = ", ".join(artifact_links) if artifact_links else "—"
        reason = str(entry.get("reason") or "—").replace("|", "\\|").replace("\n", " ")
        lines.append(
            f"| `{entry['map_id']}` ({entry['canonical_name']}) | "
            f"{entry.get('manifest_status', 'unknown')} | **{entry.get('status', 'UNKNOWN')}** | "
            f"{result_text} | {reason} |"
        )
    lines.extend(
        [
            "",
            "Each selected map is cold-started as its own Epic-quality packaged CARLA process. "
            "The orchestrator does not permit client-side map loading.",
            "",
        ]
    )
    return summary, "\n".join(lines)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    initialize = subparsers.add_parser("initialize")
    initialize.add_argument("--output-root", type=Path, required=True)
    initialize.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    initialize.add_argument("--inventory", type=Path)
    initialize.add_argument("--selected-maps", required=True)
    initialize.add_argument(
        "--invocation-maps",
        help=(
            "maps targeted by this invocation; prior selected maps and evidence remain "
            "part of the campaign"
        ),
    )
    initialize.add_argument("--host", default="127.0.0.1")
    initialize.add_argument("--port", type=int, required=True)

    update = subparsers.add_parser("update")
    update.add_argument("--output-root", type=Path, required=True)
    update.add_argument("--map-id", required=True)
    update.add_argument("--status", required=True)
    update.add_argument("--stage", required=True)
    update.add_argument("--reason")
    update.add_argument("--exit-code", type=int)
    update.add_argument("--route-seed", type=int)

    summarize = subparsers.add_parser("summarize")
    summarize.add_argument("--output-root", type=Path, required=True)
    summarize.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)

    validate = subparsers.add_parser("validate-artifacts")
    validate.add_argument("--output-root", type=Path, required=True)
    validate.add_argument("--map-id", required=True)
    validate.add_argument("--route-seed", type=int)

    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        if args.command == "initialize":
            manifest, _ = load_manifest(args.manifest)
            inventory = read_json(args.inventory) if args.inventory else None
            selected = parse_map_ids(args.selected_maps)
            invocation = (
                parse_map_ids(args.invocation_maps)
                if args.invocation_maps is not None
                else selected
            )
            paths = initialize_statuses(
                args.output_root,
                manifest,
                inventory,
                selected,
                args.host,
                args.port,
                invocation,
            )
            print(f"initialized_statuses={len(paths)} output_root={args.output_root.resolve()}")
        elif args.command == "update":
            path = update_status(
                args.output_root,
                args.map_id,
                args.status,
                args.stage,
                args.reason,
                args.exit_code,
                args.route_seed,
            )
            print(f"status={path}")
        elif args.command == "summarize":
            manifest, _ = load_manifest(args.manifest)
            summary, markdown = build_summary(args.output_root, manifest)
            json_path = atomic_write_json(args.output_root / "aggregate.json", summary)
            markdown_path = atomic_write_text(args.output_root / "SUMMARY.md", markdown)
            print(f"aggregate={json_path} markdown={markdown_path} status={summary['status']}")
        elif args.command == "validate-artifacts":
            output_root = args.output_root.expanduser().resolve()
            if args.route_seed is not None and args.route_seed < 0:
                raise ReportError("route seed must be a nonnegative integer")
            _, error = validated_job(output_root, args.map_id, args.route_seed)
            if error:
                raise ReportError(error)
            print(f"validated_map={args.map_id}")
        else:  # pragma: no cover - argparse enforces the command set.
            raise ReportError(f"unsupported command: {args.command}")
    except (OSError, ReportError, ValueError) as error:
        print(f"ERROR: {error}", file=os.sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
