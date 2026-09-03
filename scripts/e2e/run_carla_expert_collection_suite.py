#!/usr/bin/env python3
"""Plan or execute CARLA expert collection without managing the CARLA server."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import time
from typing import Any, Mapping, Sequence

try:
    from inventory_carla_training_maps import DEFAULT_MANIFEST, ManifestError, load_manifest
except ModuleNotFoundError:
    from scripts.e2e.inventory_carla_training_maps import (
        DEFAULT_MANIFEST,
        ManifestError,
        load_manifest,
    )


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_COLLECTOR = Path(__file__).with_name("collect_carla_vad_expert.py")
DEFAULT_EXPORTER = Path(__file__).with_name("export_carla_vad_expert.py")
DEFAULT_RENDERER = Path(__file__).with_name("render_carla_vad_expert.py")
IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
SCENARIOS = ("lane_follow", "straight", "left", "right")


class SuiteError(RuntimeError):
    pass


def parse_integer_list(text: str) -> tuple[int, ...]:
    try:
        values = tuple(int(item.strip()) for item in text.split(",") if item.strip())
    except ValueError as error:
        raise SuiteError("seeds must be comma-separated integers") from error
    if not values or len(set(values)) != len(values) or any(value < 0 for value in values):
        raise SuiteError("seeds must be a non-empty unique list of non-negative integers")
    return values


def parse_string_list(text: str, label: str) -> tuple[str, ...]:
    values = tuple(item.strip() for item in text.split(",") if item.strip())
    if not values or len(set(values)) != len(values):
        raise SuiteError(f"{label} must be a non-empty unique comma-separated list")
    if any(not IDENTIFIER.fullmatch(value) for value in values):
        raise SuiteError(f"{label} contains an unsafe value")
    return values


def parse_scenarios(text: str) -> tuple[str, ...]:
    values = parse_string_list(text, "scenarios")
    unsupported = tuple(value for value in values if value not in SCENARIOS)
    if unsupported:
        raise SuiteError(
            f"scenarios must be a subset of {','.join(SCENARIOS)}; "
            f"unsupported={list(unsupported)}"
        )
    return values


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--catalog", type=Path, action="append", required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--active-server-profile",
        help="explicit profile id of the already-running external CARLA server",
    )
    parser.add_argument("--host", default=os.environ.get("CARLA_HOST", "127.0.0.1"))
    parser.add_argument(
        "--port", type=int, default=int(os.environ.get("CARLA_PORT", "2000"))
    )
    parser.add_argument("--weathers", default="ClearNoon,CloudyNoon,WetNoon,HardRainNoon")
    parser.add_argument("--seeds", default="0,1,2")
    parser.add_argument(
        "--scenarios",
        default=",".join(SCENARIOS),
        help="unique comma-separated subset of lane_follow,straight,left,right",
    )
    parser.add_argument("--max-duration-sec", type=float, default=180.0)
    parser.add_argument("--physics-hz", type=float, default=20.0)
    parser.add_argument("--capture-hz", type=float, default=10.0)
    parser.add_argument("--target-speed-kmh", type=float, default=9.0)
    parser.add_argument("--goal-tolerance-m", type=float, default=2.5)
    parser.add_argument("--server-timeout-sec", type=float, default=30.0)
    parser.add_argument(
        "--map-load-settle-sec",
        type=float,
        default=10.0,
        help=(
            "pre-read delay for the externally started map; also applied after an "
            "explicitly allowed map load"
        ),
    )
    parser.add_argument(
        "--allow-map-load",
        action="store_true",
        help=(
            "UNSAFE opt-in: permit client.load_world between mismatched jobs; some "
            "packaged/custom CARLA maps crash during client map loading"
        ),
    )
    parser.add_argument("--estimated-jpeg-kib", type=float, default=50.0)
    parser.add_argument("--collector", type=Path, default=DEFAULT_COLLECTOR)
    parser.add_argument("--exporter", type=Path, default=DEFAULT_EXPORTER)
    parser.add_argument("--renderer", type=Path, default=DEFAULT_RENDERER)
    parser.add_argument("--fail-fast", action="store_true")
    args = parser.parse_args(argv)
    try:
        args.seeds = parse_integer_list(args.seeds)
        args.weathers = parse_string_list(args.weathers, "weathers")
        args.scenarios = parse_scenarios(args.scenarios)
    except SuiteError as error:
        parser.error(str(error))
    if not 0 < args.port <= 65535:
        parser.error("port must be in [1, 65535]")
    for name in (
        "max_duration_sec",
        "physics_hz",
        "capture_hz",
        "target_speed_kmh",
        "goal_tolerance_m",
        "estimated_jpeg_kib",
        "server_timeout_sec",
    ):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    if not math.isfinite(args.map_load_settle_sec) or args.map_load_settle_sec < 0.0:
        parser.error("--map-load-settle-sec must be finite and non-negative")
    ratio = args.physics_hz / args.capture_hz
    if abs(ratio - round(ratio)) > 1.0e-9:
        parser.error("physics-hz must be an integer multiple of capture-hz")
    return args


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


def _read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise SuiteError(f"failed to read {label} {path}: {error}") from error
    if not isinstance(payload, dict):
        raise SuiteError(f"{label} must contain a JSON object: {path}")
    return payload


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while block := stream.read(1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def _map_by_id(manifest: Mapping[str, Any], map_id: str) -> Mapping[str, Any]:
    for entry in manifest["maps"]:
        if entry["id"] == map_id:
            return entry
    raise SuiteError(f"catalog references unknown map id {map_id!r}")


def load_catalog(path: Path, manifest: Mapping[str, Any]) -> dict[str, Any]:
    path = path.expanduser().resolve()
    catalog = _read_json(path, "route catalog")
    if catalog.get("schema_version") != 1:
        raise SuiteError(f"catalog must use schema version 1: {path}")
    map_id = catalog.get("map_id")
    if not isinstance(map_id, str):
        raise SuiteError(f"catalog is missing map_id: {path}")
    map_entry = _map_by_id(manifest, map_id)
    if catalog.get("server_profile") != map_entry["server_profile"]:
        raise SuiteError(f"catalog/server profile mismatch for {map_id}")
    routes = catalog.get("routes")
    if not isinstance(routes, list):
        raise SuiteError(f"catalog routes must be a list: {path}")
    seen = set()
    resolved_routes = []
    catalog_root = path.parent.resolve()
    for index, route in enumerate(routes):
        if not isinstance(route, dict):
            raise SuiteError(f"catalog route {index} must be an object")
        route_id = route.get("id")
        if not isinstance(route_id, str) or not IDENTIFIER.fullmatch(route_id):
            raise SuiteError(f"catalog route {index} has an unsafe id")
        if route_id in seen:
            raise SuiteError(f"duplicate route id {route_id!r}")
        seen.add(route_id)
        relative = route.get("path")
        if not isinstance(relative, str) or Path(relative).is_absolute():
            raise SuiteError(f"catalog route {route_id} path must be relative")
        route_path = (catalog_root / relative).resolve()
        try:
            route_path.relative_to(catalog_root)
        except ValueError as error:
            raise SuiteError(f"catalog route {route_id} escapes its catalog root") from error
        resolved = dict(route)
        resolved["resolved_path"] = str(route_path)
        resolved_routes.append(resolved)
    result = dict(catalog)
    result["catalog_path"] = str(path)
    result["map_entry"] = dict(map_entry)
    result["routes"] = resolved_routes
    return result


def _manifest_status(path: Path, expected: str) -> bool:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return False
    return isinstance(payload, dict) and payload.get("status") == expected


def _nonempty_file(path: Path) -> bool:
    try:
        return path.is_file() and path.stat().st_size > 0
    except OSError:
        return False


def _preview_status(paths: Mapping[str, str]) -> bool:
    return _nonempty_file(Path(paths["preview_png"])) and _nonempty_file(
        Path(paths["preview_gif"])
    )


def _estimate_job_bytes(args: argparse.Namespace) -> int:
    camera_frames = math.ceil(args.max_duration_sec * args.capture_hz)
    image_bytes = camera_frames * 6 * args.estimated_jpeg_kib * 1024.0
    state_and_metadata = args.max_duration_sec * args.physics_hz * 1024.0
    preview_allowance = 20.0 * 1024.0 * 1024.0
    return int(math.ceil(image_bytes * 1.10 + state_and_metadata + preview_allowance))


def _commands_for_job(args: argparse.Namespace, job: Mapping[str, Any]) -> dict[str, list[str]]:
    python = sys.executable
    episode = job["paths"]["episode"]
    export = job["paths"]["export"]
    preview_png = job["paths"]["preview_png"]
    preview_gif = job["paths"]["preview_gif"]
    commands = {
        "collector": [
            python,
            str(args.collector.expanduser().resolve()),
            episode,
            job["route_path"],
            "--host",
            args.host,
            "--port",
            str(args.port),
            "--physics-hz",
            str(args.physics_hz),
            "--capture-hz",
            str(args.capture_hz),
            "--target-speed-kmh",
            str(args.target_speed_kmh),
            "--goal-tolerance-m",
            str(args.goal_tolerance_m),
            "--max-duration-sec",
            str(args.max_duration_sec),
            "--seed",
            str(job["seed"]),
            "--weather",
            job["weather"],
        ],
        "exporter": [
            python,
            str(args.exporter.expanduser().resolve()),
            "--input",
            episode,
            "--output",
            export,
        ],
        "renderer": [
            python,
            str(args.renderer.expanduser().resolve()),
            "--episode",
            episode,
            "--export",
            export,
            "--output-png",
            preview_png,
            "--output-gif",
            preview_gif,
        ],
    }
    if getattr(args, "allow_map_load", False):
        commands["collector"].append("--allow-map-load")
    return commands


def build_plan(
    args: argparse.Namespace,
    manifest: Mapping[str, Any],
    catalogs: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    output_root = args.output_root.expanduser().resolve()
    allow_map_load = getattr(args, "allow_map_load", False)
    selected_scenarios = tuple(getattr(args, "scenarios", SCENARIOS))
    selected_scenario_set = frozenset(selected_scenarios)
    catalog_route_count = sum(len(catalog["routes"]) for catalog in catalogs)
    selected_route_count = sum(
        route.get("scenario") in selected_scenario_set
        for catalog in catalogs
        for route in catalog["routes"]
    )
    if selected_route_count == 0:
        raise SuiteError(
            "no catalog route matches selected scenarios: "
            + ",".join(selected_scenarios)
        )
    profiles = {
        catalog["server_profile"]
        for catalog in catalogs
        if catalog.get("server_profile") is not None
    }
    active_profile = args.active_server_profile
    if active_profile is not None and active_profile not in manifest["server_profiles"]:
        raise SuiteError(f"unknown active server profile: {active_profile}")
    profile_mismatch = len(profiles) > 1
    estimated_job_bytes = _estimate_job_bytes(args)
    jobs = []
    job_ids: set[str] = set()
    catalog_records = []

    for catalog in catalogs:
        map_entry = catalog["map_entry"]
        selected_routes = [
            route
            for route in catalog["routes"]
            if route.get("scenario") in selected_scenario_set
        ]
        blocked_reason = None
        if map_entry["status"] == "unavailable":
            blocked_reason = f"unavailable map: {map_entry['reason']}"
        elif profile_mismatch:
            blocked_reason = "catalogs use different server profiles"
        elif active_profile is not None and active_profile != catalog["server_profile"]:
            blocked_reason = (
                f"active server profile {active_profile!r} does not match catalog profile "
                f"{catalog['server_profile']!r}"
            )
        elif (
            map_entry["status"] == "source_editor_required"
            and active_profile != catalog["server_profile"]
        ):
            blocked_reason = (
                "source_editor_required map requires explicit matching "
                "--active-server-profile"
            )
        elif catalog.get("status") != "complete":
            blocked_reason = f"catalog status is {catalog.get('status')!r}"
        catalog_records.append(
            {
                "path": catalog["catalog_path"],
                "map_id": catalog["map_id"],
                "server_profile": catalog.get("server_profile"),
                "status": "BLOCKED" if blocked_reason else "READY",
                "reason": blocked_reason,
                "route_count": len(catalog["routes"]),
                "selected_route_count": len(selected_routes),
                "filtered_route_count": len(catalog["routes"]) - len(selected_routes),
            }
        )

        for route in selected_routes:
            route_path = Path(route["resolved_path"])
            route_problem = None
            if not route_path.is_file():
                route_problem = "route file is missing"
            elif route.get("sha256") and _sha256_file(route_path) != route["sha256"]:
                route_problem = "route SHA-256 does not match catalog"
            for weather in args.weathers:
                for seed in args.seeds:
                    job_id = f"{catalog['map_id']}__{route['id']}__{weather}__s{seed:04d}"
                    if not IDENTIFIER.fullmatch(job_id):
                        raise SuiteError(f"generated unsafe job id: {job_id}")
                    if job_id in job_ids:
                        raise SuiteError(f"duplicate job id across catalogs: {job_id}")
                    job_ids.add(job_id)
                    job_root = output_root / catalog["map_id"] / route["id"] / weather / f"seed_{seed:04d}"
                    paths = {
                        "root": str(job_root),
                        "episode": str(job_root / "episode"),
                        "export": str(job_root / "export"),
                        "preview_png": str(job_root / "preview" / "overview.png"),
                        "preview_gif": str(job_root / "preview" / "drive.gif"),
                    }
                    if blocked_reason:
                        status, reason = "BLOCKED", blocked_reason
                    elif route_problem:
                        status, reason = "SKIP", route_problem
                    elif (
                        _manifest_status(
                            Path(paths["episode"]) / "manifest.json", "complete"
                        )
                        and _manifest_status(
                            Path(paths["export"]) / "manifest.json", "validated"
                        )
                        and _preview_status(paths)
                    ):
                        status, reason = "SKIP_RESUME_VALIDATED", "episode complete and export validated"
                    else:
                        status, reason = "PENDING", None
                    job = {
                        "id": job_id,
                        "map_id": catalog["map_id"],
                        "server_profile": catalog.get("server_profile"),
                        "map_load_name": map_entry["load_name"],
                        "canonical_map_name": map_entry["canonical_name"],
                        "route_id": route["id"],
                        "route_path": str(route_path),
                        "scenario": route.get("scenario"),
                        "weather": weather,
                        "seed": seed,
                        "status": status,
                        "reason": reason,
                        "estimated_bytes": estimated_job_bytes,
                        "paths": paths,
                    }
                    job["commands"] = _commands_for_job(args, job)
                    jobs.append(job)

    counts = {
        status: sum(job["status"] == status for job in jobs)
        for status in ("PENDING", "BLOCKED", "SKIP", "SKIP_RESUME_VALIDATED")
    }
    if counts["BLOCKED"] or (not counts["PENDING"] and not counts["SKIP_RESUME_VALIDATED"]):
        status = "BLOCKED"
    elif counts["PENDING"]:
        status = "READY_WITH_SKIPS" if counts["SKIP"] else "READY"
    else:
        status = "COMPLETE"
    return {
        "schema_version": 1,
        "status": status,
        "mode": "execute" if args.execute else "dry-run",
        "execution_performed": False,
        "output_root": str(output_root),
        "plan_path": str(output_root / "collection_plan.json"),
        "server": {
            "host": args.host,
            "port": args.port,
            "server_profiles": sorted(profile for profile in profiles if profile),
            "active_server_profile": active_profile,
            "external_server_required": True,
            "server_lifecycle_managed": False,
            "map_lifecycle_managed": False,
            "map_load_allowed": allow_map_load,
            "map_load_settle_sec": args.map_load_settle_sec,
            "map_load_warning": (
                "client.load_world is explicitly enabled and may crash packaged or "
                "custom CARLA maps"
                if allow_map_load
                else None
            ),
        },
        "matrix": {
            "scenarios": list(selected_scenarios),
            "weathers": list(args.weathers),
            "seeds": list(args.seeds),
            "physics_hz": args.physics_hz,
            "capture_hz": args.capture_hz,
            "maximum_duration_s": args.max_duration_sec,
        },
        "route_selection": {
            "selected_scenarios": list(selected_scenarios),
            "catalog_route_count": catalog_route_count,
            "selected_route_count": selected_route_count,
            "filtered_route_count": catalog_route_count - selected_route_count,
        },
        "catalogs": catalog_records,
        "counts": {"total": len(jobs), **counts},
        "estimated_storage": {
            "bytes_per_pending_job": estimated_job_bytes,
            "pending_bytes": estimated_job_bytes * counts["PENDING"],
            "pending_gib": estimated_job_bytes * counts["PENDING"] / (1024.0**3),
            "assumed_jpeg_kib": args.estimated_jpeg_kib,
        },
        "jobs": jobs,
    }


def _run_logged(command: Sequence[str], log_path: Path) -> int:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8") as stream:
        stream.write("command: " + subprocess.list2cmdline(list(command)) + "\n")
        stream.flush()
        result = subprocess.run(
            list(command),
            cwd=ROOT,
            stdout=stream,
            stderr=subprocess.STDOUT,
            check=False,
        )
    return result.returncode


def _load_carla_runtime() -> Any:
    try:
        import carla
    except ModuleNotFoundError as error:
        raise SuiteError(
            "CARLA Python API is required for --execute; source the environment for "
            "the declared active server profile"
        ) from error
    return carla


def _world_matches_job_map(world: Any, job: Mapping[str, Any]) -> bool:
    name = str(world.get_map().name).rstrip("/")
    return name == job["map_load_name"] or name.rsplit("/", 1)[-1] == job[
        "canonical_map_name"
    ]


def _ensure_idle_world(world: Any) -> None:
    actors = []
    for pattern in ("vehicle.*", "sensor.*"):
        actors.extend(world.get_actors().filter(pattern))
    if actors:
        details = ", ".join(
            f"id={actor.id},type={actor.type_id},role={actor.attributes.get('role_name', '')}"
            for actor in actors[:20]
        )
        raise SuiteError(f"CARLA world is already in use: {details}")


def prepare_server_for_job(
    job: dict[str, Any],
    args: argparse.Namespace,
    *,
    carla_module: Any | None = None,
    sleep: Any = time.sleep,
    pre_read_settle: bool = True,
) -> bool:
    """Verify one job map, loading only after an explicit unsafe opt-in."""
    carla_module = carla_module or _load_carla_runtime()
    client = carla_module.Client(args.host, args.port)
    client.set_timeout(args.server_timeout_sec)
    world = client.get_world()
    if pre_read_settle and args.map_load_settle_sec > 0.0:
        sleep(args.map_load_settle_sec)
    job["server_map_before"] = str(world.get_map().name)
    job["server_map_required"] = job["map_load_name"]
    map_load_performed = False
    if not _world_matches_job_map(world, job):
        if not getattr(args, "allow_map_load", False):
            raise SuiteError(
                f"current CARLA map {world.get_map().name!r} does not match required "
                f"map {job['map_load_name']!r}; automatic map loading is disabled, "
                "so start CARLA on the required map or pass the explicit unsafe "
                "--allow-map-load opt-in"
            )
        _ensure_idle_world(world)
        world = client.load_world(job["map_load_name"])
        map_load_performed = True
        if args.map_load_settle_sec > 0.0:
            sleep(args.map_load_settle_sec)
    else:
        _ensure_idle_world(world)
    if not _world_matches_job_map(world, job):
        raise SuiteError(
            f"server did not activate {job['map_load_name']!r}; "
            f"current={world.get_map().name!r}"
        )
    job["server_map_load_performed"] = map_load_performed
    job["server_map_load_settle_sec"] = (
        args.map_load_settle_sec if map_load_performed else 0.0
    )
    job["server_pre_read_settle_sec"] = (
        args.map_load_settle_sec if pre_read_settle else 0.0
    )
    job["server_map_load_allowed"] = getattr(args, "allow_map_load", False)
    job["server_map_after"] = str(world.get_map().name)
    return map_load_performed


def execute_job(job: dict[str, Any], run_logged=_run_logged) -> None:
    job_root = Path(job["paths"]["root"])
    job_root.mkdir(parents=True, exist_ok=True)
    episode_manifest = Path(job["paths"]["episode"]) / "manifest.json"
    export_manifest = Path(job["paths"]["export"]) / "manifest.json"
    stages = (
        (
            "collector",
            episode_manifest,
            "complete",
            lambda: _manifest_status(episode_manifest, "complete"),
        ),
        (
            "exporter",
            export_manifest,
            "validated",
            lambda: _manifest_status(export_manifest, "validated"),
        ),
        ("renderer", None, None, lambda: _preview_status(job["paths"])),
    )
    upstream_ran = False
    for stage, manifest_path, expected_status, already_complete in stages:
        if not upstream_ran and already_complete():
            continue
        exit_code = run_logged(job["commands"][stage], job_root / "logs" / f"{stage}.log")
        if exit_code != 0:
            raise SuiteError(f"{stage} exited with code {exit_code}")
        if manifest_path is not None and not _manifest_status(manifest_path, expected_status):
            raise SuiteError(f"{stage} did not produce status {expected_status!r}")
        upstream_ran = True
    if not Path(job["paths"]["preview_png"]).is_file() or not Path(
        job["paths"]["preview_gif"]
    ).is_file():
        raise SuiteError("renderer did not produce both preview files")
    job["status"] = "COMPLETE"
    job["reason"] = None


def execute_plan(plan: dict[str, Any], args: argparse.Namespace) -> int:
    plan_path = Path(plan["plan_path"])
    failures = 0
    server_checked = False
    plan["execution_performed"] = True
    for job in plan["jobs"]:
        if job["status"] != "PENDING":
            continue
        job["status"] = "RUNNING"
        atomic_write_json(plan_path, plan)
        try:
            map_load_performed = prepare_server_for_job(
                job, args, pre_read_settle=not server_checked
            )
            server_checked = True
            if map_load_performed:
                plan["server"]["map_lifecycle_managed"] = True
            execute_job(job)
        except (OSError, SuiteError) as error:
            job["status"] = "FAILED"
            job["reason"] = str(error)
            failures += 1
        atomic_write_json(plan_path, plan)
        if failures and args.fail_fast:
            break
    plan["counts"] = {
        "total": len(plan["jobs"]),
        **{
            status: sum(job["status"] == status for job in plan["jobs"])
            for status in (
                "COMPLETE",
                "FAILED",
                "PENDING",
                "BLOCKED",
                "SKIP",
                "SKIP_RESUME_VALIDATED",
            )
        },
    }
    if failures:
        plan["status"] = "FAILED"
    elif any(job["status"] == "BLOCKED" for job in plan["jobs"]):
        plan["status"] = "BLOCKED"
    elif any(job["status"] == "PENDING" for job in plan["jobs"]):
        plan["status"] = "INCOMPLETE"
    else:
        plan["status"] = "COMPLETE"
    atomic_write_json(plan_path, plan)
    return 1 if failures else 0


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        manifest, _ = load_manifest(args.manifest)
        catalogs = [load_catalog(path, manifest) for path in args.catalog]
        plan = build_plan(args, manifest, catalogs)
        plan_path = atomic_write_json(Path(plan["plan_path"]), plan)
        if args.execute:
            if plan["status"] == "BLOCKED":
                print(f"BLOCKED plan={plan_path}", file=sys.stderr)
                return 2
            exit_code = execute_plan(plan, args)
        else:
            exit_code = 0
    except (ManifestError, OSError, SuiteError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    print(
        f"plan={plan_path} mode={plan['mode']} status={plan['status']} "
        f"jobs={plan['counts']['total']} pending_gib={plan['estimated_storage']['pending_gib']:.3f}"
    )
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
