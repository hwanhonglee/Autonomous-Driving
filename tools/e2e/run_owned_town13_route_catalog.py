#!/usr/bin/env python3
"""Own one fixed Town13 source-editor route catalog, with no camera capture."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
from typing import Any, Iterator

try:
    from .audit_town13_source_editor_preflight import (
        EXPECTED_MAP,
        EXPECTED_SHA256,
        inspect_file,
    )
    from .inventory_carla_training_maps import load_manifest
    from .prepare_carla_expert_route_catalog import parse_args as parse_catalog_args
except ImportError:
    from scripts.e2e.audit_town13_source_editor_preflight import (
        EXPECTED_MAP,
        EXPECTED_SHA256,
        inspect_file,
    )
    from scripts.e2e.inventory_carla_training_maps import load_manifest
    from scripts.e2e.prepare_carla_expert_route_catalog import parse_args as parse_catalog_args


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "storage/experiments/validation/2026-09-27/map_town13_route_catalog_v2"
LOCK = ROOT / "storage/assets/locks/autoware_e2e_runtime.lock"
PORTS = (2100, 2101, 2102)
STARTUP_TIMEOUT_SEC = 420
CATALOG_TIMEOUT_SEC = 1200
MAP_NAME = "Town13"
MAP_LOAD_NAME = "/Game/Carla/Maps/Town13/Town13"
# HH_260906 - UE4 reports the same full Town13 asset path without its launch-only /Game prefix.
LIVE_MAP_NAMES = frozenset((MAP_LOAD_NAME, "Carla/Maps/Town13/Town13"))
PROFILE_ID = "source_editor_0915_4ws"
SOURCE_FILES = (
    "tools/e2e/run_owned_town13_route_catalog.py",
    "tools/e2e/audit_town13_source_editor_preflight.py",
    "tools/e2e/carla_expert_suite.yaml",
    "tools/e2e/inventory_carla_training_maps.py",
    "tools/e2e/prepare_carla_expert_route_catalog.py",
    "tools/e2e/prepare_carla_route.py",
    "tools/e2e/physical_straight_geometry.py",
    "tools/e2e/physical_turn_geometry.py",
    "tools/e2e/serialized_custom_turn_geometry.py",
    "tools/e2e/probe_carla_server.py",
    "tools/workspace_python.py",
    "tools/canonical_workspace.py",
)


class CatalogOwnerError(RuntimeError):
    """HH_260906 - Mark any uncertain owner state as a failed catalog attempt."""


def _utc() -> str:
    return datetime.now(timezone.utc).isoformat()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _write_new_json(path: Path, document: dict[str, Any]) -> None:
    with path.open("x", encoding="utf-8") as stream:
        json.dump(document, stream, indent=2, sort_keys=True)
        stream.write("\n")


def _source_environment(profile: dict[str, Any]) -> dict[str, str]:
    """HH_260906 - Put the byte-pinned source CARLA egg ahead of a packaged API."""
    egg = Path(profile["python_api"])
    source_root = egg.parents[3]
    api = source_root / "PythonAPI"
    if not (api / "carla" / "agents").is_dir():
        raise CatalogOwnerError("source CARLA agents directory is absent")
    env = os.environ.copy()
    env["CARLA_SOURCE_ROOT"] = str(source_root)
    env["UE4_ROOT"] = str(Path(profile["executable"]).parents[3])
    preferred = (str(api), str(api / "carla"), str(egg))
    env["PYTHONPATH"] = os.pathsep.join((*preferred, env.get("PYTHONPATH", "")))
    return env


def static_binding() -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    """HH_260906 - Require the exact validation map and five pinned source assets."""
    manifest, manifest_path = load_manifest()
    entry = next((item for item in manifest["maps"] if item["id"] == "town13"), None)
    if entry is None or any(entry.get(key) != value for key, value in EXPECTED_MAP.items()):
        raise CatalogOwnerError("Town13 validation map binding changed")
    profile = manifest["server_profiles"].get(PROFILE_ID)
    if profile is None or profile.get("kind") != "source_editor":
        raise CatalogOwnerError("Town13 source-editor profile changed")
    if profile.get("python_api_sha256") != EXPECTED_SHA256["python_api"]:
        raise CatalogOwnerError("source CARLA Python API pin changed")
    paths = {key: Path(profile[key]) for key in ("executable", "project", "python_api")}
    paths.update({key: Path(entry[key]) for key in ("level_path", "opendrive_path")})
    assets = {
        key: inspect_file(path, EXPECTED_SHA256[key])
        for key, path in paths.items()
    }
    failed = [key for key, record in assets.items() if not record["ok"]]
    if failed:
        raise CatalogOwnerError("Town13 source asset mismatch: " + ", ".join(failed))
    if not os.access(paths["executable"], os.X_OK):
        raise CatalogOwnerError("pinned UE4Editor is not executable")
    content = paths["project"].parent / "Content"
    try:
        relative = paths["level_path"].relative_to(content)
    except ValueError as error:
        raise CatalogOwnerError("Town13 level is outside the pinned project") from error
    if "/Game/" + relative.with_suffix("").as_posix() != MAP_LOAD_NAME:
        raise CatalogOwnerError("Town13 source level does not match the launch URL")
    if Path(manifest_path).is_symlink():
        raise CatalogOwnerError("suite manifest is a symlink")
    return profile, entry, assets


def editor_command(profile: dict[str, Any], entry: dict[str, Any]) -> list[str]:
    """HH_260906 - Use UE4's uncooked game URL, never an editor Play click."""
    if entry["load_name"] != MAP_LOAD_NAME:
        raise CatalogOwnerError("source-editor map URL is not Town13")
    # HH_260928 - Disable unattended Epic crash upload for this owned editor process.
    return [
        profile["executable"], profile["project"], MAP_LOAD_NAME,
        "-game", "-vulkan", "-prefernvidia", "-quality-level=Epic",
        "-carla-port=2100", "-RenderOffScreen", "-nosound", "-unattended",
        "-ini:EditorSettings:[/Script/UnrealEd.CrashReportsPrivacySettings]:bSendUnattendedBugReports=False",
    ]


def catalog_arguments(output: Path = OUTPUT) -> list[str]:
    return [
        "--map-id", "town13", "--active-server-profile", PROFILE_ID,
        "--host", "127.0.0.1", "--port", "2100",
        "--map-load-settle-sec", "0", "--output-root", str(output),
        "--scenarios", "lane_follow,straight,left,right", "--seeds", "1,2",
        "--pairs-per-seed", "4", "--min-distance", "60",
        "--max-distance", "140", "--preferred-distance", "100",
        "--sampling-resolution", "1", "--max-traces", "12000",
    ]


def _assert_catalog_arguments(args: list[str]) -> None:
    parsed = parse_catalog_args(args)
    if (parsed.allow_map_load or parsed.map_id != "town13"
            or parsed.active_server_profile != PROFILE_ID
            or parsed.host != "127.0.0.1" or parsed.port != PORTS[0]
            or parsed.scenarios != ("lane_follow", "straight", "left", "right")
            or parsed.output_root != OUTPUT):
        raise CatalogOwnerError("fixed Town13 catalog contract changed")


def _assert_source_import(env: dict[str, str], profile: dict[str, Any]) -> None:
    script = (
        "import carla, agents, json; "
        "print(json.dumps({'carla': carla.__file__, 'agents': agents.__file__}))"
    )
    result = subprocess.run(
        [sys.executable, "-c", script], env=env, cwd=ROOT,
        text=True, capture_output=True, timeout=20, check=True,
    )
    modules = json.loads(result.stdout)
    egg = profile["python_api"]
    source_agents = str(Path(egg).parents[3] / "PythonAPI/carla/agents")
    if (not modules["carla"].startswith(egg + "/")
            or not modules["agents"].startswith(source_agents + "/")):
        raise CatalogOwnerError("CARLA client or navigation agents came from another profile")


def preflight() -> tuple[dict[str, Any], dict[str, Any], dict[str, Any], dict[str, str]]:
    if sys.version_info[:2] != (3, 10):
        raise CatalogOwnerError("pinned source CARLA egg requires Python 3.10")
    profile, entry, assets = static_binding()
    args = catalog_arguments()
    _assert_catalog_arguments(args)
    for name in SOURCE_FILES:
        source = ROOT / name
        if source.is_symlink() or not source.is_file():
            raise CatalogOwnerError(f"catalog source missing or symlinked: {name}")
    env = _source_environment(profile)
    _assert_source_import(env, profile)
    return profile, entry, assets, env


@contextmanager
def _workspace_lock() -> Iterator[None]:
    """HH_260906 - Reuse the existing nonblocking workspace flock path."""
    LOCK.parent.mkdir(parents=True, exist_ok=True)
    with LOCK.open("a+") as stream:
        try:
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise CatalogOwnerError("another workspace operation owns the runtime lock") from error
        try:
            yield
        finally:
            fcntl.flock(stream.fileno(), fcntl.LOCK_UN)


def _assert_no_other_carla() -> None:
    result = subprocess.run(
        ["ps", "-eo", "pid=,args="], text=True, capture_output=True,
        check=True, timeout=10,
    )
    for line in result.stdout.splitlines():
        parts = line.strip().split(maxsplit=1)
        if len(parts) != 2 or int(parts[0]) == os.getpid():
            continue
        command = parts[1]
        if ("/Engine/Binaries/Linux/UE4Editor" in command
                or "/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping" in command):
            raise CatalogOwnerError("another CARLA or UE4Editor process is already running")


def _assert_ports_free() -> None:
    for port in PORTS:
        with socket.socket() as probe:
            try:
                probe.bind(("127.0.0.1", port))
            except OSError as error:
                raise CatalogOwnerError(f"CARLA port {port} is occupied") from error


def _make_output() -> None:
    if OUTPUT.exists() or OUTPUT.is_symlink():
        raise CatalogOwnerError(f"refusing to reuse an existing output: {OUTPUT}")
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.mkdir()
    (OUTPUT / "provenance").mkdir()
    (OUTPUT / "lifecycle").mkdir()


def _archive_sources() -> dict[str, str]:
    hashes: dict[str, str] = {}
    for name in SOURCE_FILES:
        source = ROOT / name
        if source.is_symlink() or not source.is_file():
            raise CatalogOwnerError(f"source missing or symlinked: {name}")
        data = source.read_bytes()
        archive = OUTPUT / "provenance" / name
        archive.parent.mkdir(parents=True, exist_ok=True)
        with archive.open("xb") as stream:
            stream.write(data)
        hashes[name] = hashlib.sha256(data).hexdigest()
    return hashes


def _verify_sources(hashes: dict[str, str]) -> bool:
    return bool(hashes) and all(
        _sha256(ROOT / name) == expected
        and _sha256(OUTPUT / "provenance" / name) == expected
        for name, expected in hashes.items()
    )


def _verify_source_assets(assets: dict[str, Any]) -> bool:
    return all(
        inspect_file(Path(record["path"]), record["expected_sha256"])["ok"]
        for record in assets.values()
    )


def _probe(stage: str, editor_pid: int, env: dict[str, str], *, stopped: bool = False) -> dict[str, Any]:
    path = OUTPUT / "lifecycle" / f"{stage}.json"
    command = [
        sys.executable, str(ROOT / "tools/workspace_python.py"),
        str(ROOT / "tools/e2e/probe_carla_server.py"),
        "--host", "127.0.0.1", "--port", str(PORTS[0]), "--timeout", "4",
        "--expected-map", MAP_NAME, "--owner-pid", str(editor_pid),
        "--owner-pgid", str(editor_pid), "--generation-id", f"town13_catalog_{editor_pid}",
        "--stage", stage, "--server-log", str(OUTPUT / "server.log"),
        "--output", str(path),
    ]
    if stopped:
        command.append("--expect-stopped")
    result = subprocess.run(command, cwd=ROOT, env=env, text=True,
                            capture_output=True, timeout=12)
    try:
        receipt = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise CatalogOwnerError(f"{stage} probe receipt is missing or invalid") from error
    if result.returncode != 0 or receipt.get("status") != "PASS":
        raise CatalogOwnerError(f"{stage} probe failed: {receipt.get('error')}")
    if not stopped and receipt.get("active_map_name") not in LIVE_MAP_NAMES:
        raise CatalogOwnerError(f"{stage} full map name differs from Town13")
    return receipt


def _wait_ready(editor: subprocess.Popen[bytes], env: dict[str, str]) -> dict[str, Any]:
    deadline = time.monotonic() + STARTUP_TIMEOUT_SEC
    last_error = "source editor has not answered"
    while time.monotonic() < deadline:
        if editor.poll() is not None:
            raise CatalogOwnerError(f"UE4Editor exited during startup: {editor.returncode}")
        try:
            return _probe("ready", editor.pid, env)
        except CatalogOwnerError as error:
            last_error = str(error)
            if "full map name differs" in last_error or "map mismatch" in last_error:
                raise
        time.sleep(2)
    raise CatalogOwnerError(f"Town13 source editor startup timed out: {last_error}")


def _stop_owned_group(process: subprocess.Popen[bytes] | None) -> bool:
    """HH_260906 - Escalate signals only to a process group started by this owner."""
    if process is None:
        return True
    pgid = process.pid
    if pgid <= 1 or pgid == os.getpgrp():
        raise CatalogOwnerError("refusing to signal a non-owned process group")
    if process.poll() is None:
        try:
            if os.getpgid(pgid) != pgid:
                raise CatalogOwnerError("owned process-group identity changed")
        except ProcessLookupError:
            pass
    for sig, duration in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            break
        deadline = time.monotonic() + duration
        group_gone = False
        while time.monotonic() < deadline:
            try:
                os.killpg(pgid, 0)
            except ProcessLookupError:
                group_gone = True
                break
            time.sleep(0.2)
        if group_gone:
            break
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        return False
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return True
    return False


def _validate_catalog_document(catalog: dict[str, Any]) -> int:
    """HH_260906 - Reject a foreign map, unsafe world load, or altered search."""
    server = catalog.get("server", {})
    generation = catalog.get("generation", {})
    if (catalog.get("status") != "complete"
            or catalog.get("map_id") != "town13"
            or catalog.get("load_name") != MAP_LOAD_NAME
            or catalog.get("server_profile") != PROFILE_ID
            or server.get("active_server_profile") != PROFILE_ID
            or server.get("initial_map_name") not in LIVE_MAP_NAMES
            or server.get("active_map_name") not in LIVE_MAP_NAMES
            or server.get("map_load_allowed") is not False
            or server.get("map_load_performed") is not False
            or not server.get("client_version")
            or not server.get("server_version")
            or generation.get("scenarios") != ["lane_follow", "straight", "left", "right"]
            or generation.get("seeds") != [1, 2]
            or generation.get("pairs_per_seed") != 4
            or generation.get("minimum_distance_m") != 60
            or generation.get("maximum_distance_m") != 140
            or generation.get("preferred_distance_m") != 100
            or generation.get("sampling_resolution_m") != 1
            or generation.get("max_traces_per_scenario") != 12000
            or not isinstance(generation.get("spawn_point_count"), int)
            or generation["spawn_point_count"] < 2
            or not isinstance(catalog.get("routes"), list)
            or not catalog["routes"]):
        raise CatalogOwnerError("Town13 catalog identity or route completeness failed")
    for route in catalog["routes"]:
        if (not isinstance(route, dict) or route.get("status") != "ready"
                or route.get("scenario") not in ("lane_follow", "straight", "left", "right")
                or not isinstance(route.get("start_spawn_index"), int)
                or not isinstance(route.get("goal_spawn_index"), int)
                or route["start_spawn_index"] < 0 or route["goal_spawn_index"] < 0
                or not isinstance(route.get("sha256"), str)
                or len(route["sha256"]) != 64
                or any(ch not in "0123456789abcdef" for ch in route["sha256"])):
            raise CatalogOwnerError("Town13 catalog contains an invalid route record")
    return len(catalog["routes"])


def _assess_catalog(plan: dict[str, Any]) -> dict[str, Any]:
    path = OUTPUT / "route_catalog.json"
    if path.is_symlink() or not path.is_file():
        raise CatalogOwnerError("Town13 route catalog is missing or symlinked")
    payload = path.read_bytes()
    catalog = json.loads(payload)
    route_count = _validate_catalog_document(catalog)
    for route in catalog["routes"]:
        name = route.get("path")
        if not isinstance(name, str):
            raise CatalogOwnerError("Town13 route path is missing")
        relative = Path(name)
        if (relative.is_absolute() or ".." in relative.parts
                or relative.parts[:2] != ("routes", "town13")
                or relative.suffix != ".json"):
            raise CatalogOwnerError("Town13 route path escapes its catalog")
        route_path = OUTPUT / relative
        if route_path.is_symlink() or not route_path.is_file():
            raise CatalogOwnerError("Town13 route file is missing or symlinked")
        if _sha256(route_path) != route["sha256"]:
            raise CatalogOwnerError("Town13 route file hash differs from catalog")
        route_data = json.loads(route_path.read_text(encoding="utf-8"))
        if (route_data.get("town") != MAP_NAME
                or route_data.get("coordinate_reference") != "base_link"
                or route_data.get("scenario") != route["scenario"]
                or route_data.get("start_spawn_index") != route["start_spawn_index"]
                or route_data.get("goal_spawn_index") != route["goal_spawn_index"]):
            raise CatalogOwnerError("Town13 route file disagrees with catalog metadata")
    if not _verify_sources(plan["source_sha256"]):
        raise CatalogOwnerError("catalog source bytes changed during ownership")
    return {"sha256": hashlib.sha256(payload).hexdigest(),
            "route_count": route_count, "status": catalog["status"],
            "spawn_point_count": catalog["generation"]["spawn_point_count"]}


def execute(profile: dict[str, Any], entry: dict[str, Any],
            assets: dict[str, Any], env: dict[str, str]) -> int:
    with _workspace_lock():
        locked_profile, locked_entry, locked_assets = static_binding()
        if locked_profile != profile or locked_entry != entry or locked_assets != assets:
            raise CatalogOwnerError("Town13 binding changed while acquiring ownership")
        _assert_no_other_carla()
        _assert_ports_free()
        _make_output()
        hashes: dict[str, str] = {}
        plan: dict[str, Any] = {}
        editor: subprocess.Popen[bytes] | None = None
        worker: subprocess.Popen[bytes] | None = None
        error: str | None = None
        catalog_result: dict[str, Any] | None = None
        cleanup_ok = True
        try:
            hashes = _archive_sources()
            plan = {
                "schema": "portable_e2e.owned_town13_catalog_plan.v1",
                "planned_at_utc": _utc(),
                "map_id": "town13", "split": "validation",
                "source_editor_profile": PROFILE_ID, "map_load_name": MAP_LOAD_NAME,
                "source_assets": assets, "source_sha256": hashes,
                "source_bytes_archived": True,
                "editor_argv": editor_command(profile, entry),
                "catalog_argv": catalog_arguments(),
                "startup_timeout_sec": STARTUP_TIMEOUT_SEC,
                "catalog_timeout_sec": CATALOG_TIMEOUT_SEC,
                "port_triplet": list(PORTS), "allow_map_load": False,
                "camera_capture_executed": False, "training_executed": False,
                "dataset_admission_granted": False,
            }
            _write_new_json(OUTPUT / "owner_plan.json", plan)
            if not _verify_source_assets(assets):
                raise CatalogOwnerError("Town13 source assets changed before launch")
            with (OUTPUT / "server.log").open("xb") as log:
                editor = subprocess.Popen(
                    plan["editor_argv"], cwd=Path(profile["project"]).parent,
                    env=env, stdout=log, stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            _wait_ready(editor, env)
            _write_new_json(OUTPUT / "owner_started.json", {
                "started_at_utc": _utc(), "editor_pid": editor.pid,
                "editor_pgid": editor.pid, "port_triplet": list(PORTS),
                "camera_capture_executed": False,
            })
            if not _verify_sources(hashes):
                raise CatalogOwnerError("catalog source bytes changed before worker start")
            if not _verify_source_assets(assets):
                raise CatalogOwnerError("Town13 source assets changed before catalog start")
            command = [
                sys.executable, str(ROOT / "tools/workspace_python.py"),
                "-m", "scripts.e2e.prepare_carla_expert_route_catalog", "--",
                *plan["catalog_argv"],
            ]
            with (OUTPUT / "catalog.log").open("xb") as log:
                worker = subprocess.Popen(
                    command, cwd=ROOT, env=env, stdout=log,
                    stderr=subprocess.STDOUT, start_new_session=True,
                )
            try:
                worker.wait(timeout=CATALOG_TIMEOUT_SEC)
            except subprocess.TimeoutExpired as exc:
                raise CatalogOwnerError("Town13 catalog worker timed out") from exc
            if worker.returncode != 0:
                raise CatalogOwnerError(f"Town13 catalog worker failed: {worker.returncode}")
            if not _stop_owned_group(worker):
                raise CatalogOwnerError("Town13 catalog worker group did not stop")
            worker = None
            _probe("after_catalog", editor.pid, env)
            catalog_result = _assess_catalog(plan)
        except BaseException as exc:
            error = f"{type(exc).__name__}: {exc}"
        finally:
            if worker is not None:
                try:
                    if not _stop_owned_group(worker):
                        cleanup_ok = False
                except (CatalogOwnerError, OSError):
                    cleanup_ok = False
            if editor is not None:
                try:
                    if not _stop_owned_group(editor):
                        cleanup_ok = False
                except (CatalogOwnerError, OSError):
                    cleanup_ok = False
                try:
                    _probe("stopped", editor.pid, env, stopped=True)
                    _assert_ports_free()
                except (CatalogOwnerError, OSError, subprocess.SubprocessError):
                    cleanup_ok = False
            try:
                sources_ok = _verify_sources(hashes)
            except OSError:
                sources_ok = False
            try:
                assets_ok = _verify_source_assets(assets)
            except (OSError, KeyError, TypeError):
                assets_ok = False
            passed = (error is None and cleanup_ok and sources_ok and assets_ok
                      and catalog_result is not None)
            _write_new_json(OUTPUT / "owner_result.json", {
                "schema": "portable_e2e.owned_town13_catalog_result.v1",
                "completed_at_utc": _utc(), "status": "PASS" if passed else "FAIL",
                "error": error, "catalog": catalog_result,
                "source_bytes_unchanged_and_archived": sources_ok,
                "source_assets_unchanged": assets_ok,
                "owned_processes_stopped_and_ports_released": cleanup_ok,
                "camera_capture_executed": False, "training_executed": False,
                "dataset_admission_granted": False,
            })
        return 0 if passed else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--execute", action="store_true",
                        help="cold-start exactly one owned Town13 catalog attempt")
    args = parser.parse_args(argv)
    try:
        profile, entry, assets, env = preflight()
        if not args.execute:
            output_absent = not (OUTPUT.exists() or OUTPUT.is_symlink())
            print(json.dumps({
                "schema": "portable_e2e.town13_catalog_owner_preflight.v1",
                "status": "STATIC_READY_NOT_LIVE_TESTED" if output_absent else "STOP_OUTPUT_EXISTS",
                "read_only": True,
                "source_editor_argv": editor_command(profile, entry),
                "catalog_argv": catalog_arguments(),
                "source_assets": assets, "output": str(OUTPUT),
                "output_absent": output_absent,
                "simulator_started": False, "dataset_admission_granted": False,
            }, indent=2, sort_keys=True))
            return 0 if output_absent else 2
        return execute(profile, entry, assets, env)
    except (CatalogOwnerError, OSError, ValueError, subprocess.SubprocessError) as error:
        print(f"Town13 owner preflight failed: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
