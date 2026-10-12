#!/usr/bin/env python3
"""HH_260929 - Own one Town12 source-editor identity probe without a driving capture."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
from typing import Any

try:
    from . import run_owned_town13_route_catalog as shared_owner
    from .audit_town13_source_editor_preflight import inspect_file
    from .inventory_carla_training_maps import DEFAULT_MANIFEST, ManifestError, load_manifest
except ImportError:
    from scripts.e2e import run_owned_town13_route_catalog as shared_owner
    from scripts.e2e.audit_town13_source_editor_preflight import inspect_file
    from scripts.e2e.inventory_carla_training_maps import DEFAULT_MANIFEST, ManifestError, load_manifest


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "storage/experiments/validation/2026-09-29/town12_source_editor_probe_v1"
PROFILE_ID = "source_editor_0915_4ws"
MAP_NAME = "Town12"
MAP_LOAD_NAME = "/Game/Carla/Maps/Town12/Town12"
LIVE_MAP_NAMES = frozenset((MAP_LOAD_NAME, "Carla/Maps/Town12/Town12"))
XODR_SHA256 = "26fa53d4902ac3397d35f948969f0c77c1bf023755ede2c67edf00bef817151f"
EXPECTED_MAP = {
    "id": "town12", "canonical_name": MAP_NAME, "load_name": MAP_LOAD_NAME,
    "split": "train", "status": "source_editor_required", "server_profile": PROFILE_ID,
}
EXPECTED_SHA256 = {
    "executable": "331f8ece41d8e09c85e927046a9afa522de2dc0a56affa560849d677b9ccb9e9",
    "project": "11b5ca8a37d550f45fcf5b93d15e450b7df944746ea599518cca02d6e8eddde2",
    "python_api": "ee7209c5570cb9201e9fbd3cbcefc5b176dd4f6b2289df52bb0bd7dc2c56800b",
    "level_path": "a5d427ba29e69a4f7cb13d776bb4b0accdd5b840f01d6f1b7643c0beca2f2f7d",
    "opendrive_path": XODR_SHA256,
}
SOURCE_FILES = (
    "tools/e2e/run_owned_town12_source_editor_probe.py",
    "tools/e2e/run_owned_town13_route_catalog.py",
    "tools/e2e/audit_town13_source_editor_preflight.py",
    "tools/e2e/inventory_carla_training_maps.py",
    "tools/e2e/carla_expert_suite.yaml",
    "tools/e2e/probe_carla_server.py",
    "tools/workspace_python.py",
    "tools/canonical_workspace.py",
)


class Town12OwnerError(RuntimeError):
    """HH_260929 - Treat uncertain map identity or ownership as a failed probe."""


def _utc() -> str:
    return datetime.now(timezone.utc).isoformat()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _write_new_json(path: Path, value: dict[str, Any]) -> None:
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write("\n")


def static_binding() -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    """HH_260929 - Pin the Town12 source level and its exact source-editor profile."""
    if DEFAULT_MANIFEST.is_symlink():
        raise Town12OwnerError("suite manifest is a symlink")
    manifest, _ = load_manifest()
    entry = next((item for item in manifest["maps"] if item["id"] == "town12"), None)
    if entry is None or any(entry.get(key) != value for key, value in EXPECTED_MAP.items()):
        raise Town12OwnerError("Town12 suite binding changed")
    profile = manifest["server_profiles"].get(PROFILE_ID)
    if (profile is None or profile.get("kind") != "source_editor"
            or profile.get("python_api_sha256") != EXPECTED_SHA256["python_api"]):
        raise Town12OwnerError("Town12 source-editor profile changed")
    paths = {key: Path(profile[key]) for key in ("executable", "project", "python_api")}
    paths.update({key: Path(entry[key]) for key in ("level_path", "opendrive_path")})
    assets = {key: inspect_file(path, EXPECTED_SHA256[key]) for key, path in paths.items()}
    failed = [key for key, record in assets.items() if not record["ok"]]
    if failed:
        raise Town12OwnerError("Town12 source asset mismatch: " + ", ".join(failed))
    if not os.access(paths["executable"], os.X_OK):
        raise Town12OwnerError("pinned UE4Editor is not executable")
    content = paths["project"].parent / "Content"
    try:
        relative = paths["level_path"].relative_to(content)
    except ValueError as error:
        raise Town12OwnerError("Town12 level is outside the pinned project") from error
    if "/Game/" + relative.with_suffix("").as_posix() != MAP_LOAD_NAME:
        raise Town12OwnerError("Town12 level differs from its launch URL")
    return profile, entry, assets


def editor_command(profile: dict[str, Any], entry: dict[str, Any]) -> list[str]:
    if entry["load_name"] != MAP_LOAD_NAME:
        raise Town12OwnerError("source-editor map URL is not Town12")
    # HH_260929 - Cold-start the exact uncooked level URL and disable crash upload.
    return [
        profile["executable"], profile["project"], MAP_LOAD_NAME,
        "-game", "-vulkan", "-prefernvidia", "-quality-level=Epic",
        "-carla-port=2100", "-RenderOffScreen", "-nosound", "-unattended",
        "-ini:EditorSettings:[/Script/UnrealEd.CrashReportsPrivacySettings]:bSendUnattendedBugReports=False",
    ]


def _assert_port_triplet_free() -> None:
    """HH_260929 - Require all owned CARLA TCP and UDP ports to be unbound."""
    held: list[socket.socket] = []
    try:
        for port in shared_owner.PORTS:
            for kind in (socket.SOCK_STREAM, socket.SOCK_DGRAM):
                probe = socket.socket(socket.AF_INET, kind)
                held.append(probe)
                try:
                    probe.bind(("127.0.0.1", port))
                except OSError as error:
                    raise Town12OwnerError(f"CARLA port {port} is occupied") from error
    finally:
        for probe in held:
            probe.close()


def preflight() -> tuple[dict[str, Any], dict[str, Any], dict[str, Any], dict[str, str]]:
    if sys.version_info[:2] != (3, 10):
        raise Town12OwnerError("pinned source CARLA egg requires Python 3.10")
    profile, entry, assets = static_binding()
    for name in SOURCE_FILES:
        source = ROOT / name
        if source.is_symlink() or not source.is_file():
            raise Town12OwnerError(f"source missing or symlinked: {name}")
    environment = shared_owner._source_environment(profile)
    shared_owner._assert_source_import(environment, profile)
    editor_command(profile, entry)
    return profile, entry, assets, environment


def _archive_sources() -> dict[str, str]:
    hashes: dict[str, str] = {}
    for name in SOURCE_FILES:
        source = ROOT / name
        if source.is_symlink() or not source.is_file():
            raise Town12OwnerError(f"source missing or symlinked: {name}")
        payload = source.read_bytes()
        archived = OUTPUT / "provenance" / name
        archived.parent.mkdir(parents=True, exist_ok=True)
        with archived.open("xb") as stream:
            stream.write(payload)
        hashes[name] = hashlib.sha256(payload).hexdigest()
    return hashes


def _verify_sources(hashes: dict[str, str]) -> bool:
    try:
        return bool(hashes) and all(
            _sha256(ROOT / name) == expected
            and _sha256(OUTPUT / "provenance" / name) == expected
            for name, expected in hashes.items())
    except OSError:
        return False


def _probe(stage: str, pid: int, environment: dict[str, str],
           *, stopped: bool = False) -> dict[str, Any]:
    path = OUTPUT / "lifecycle" / f"{stage}.json"
    command = [
        sys.executable, str(ROOT / "tools/workspace_python.py"),
        str(ROOT / "tools/e2e/probe_carla_server.py"),
        "--host", "127.0.0.1", "--port", "2100", "--timeout", "4",
        "--expected-map", MAP_NAME, "--owner-pid", str(pid),
        "--owner-pgid", str(pid), "--generation-id", f"town12_source_probe_{pid}",
        "--stage", stage, "--server-log", str(OUTPUT / "server.log"),
        "--output", str(path),
    ]
    if stopped:
        command.append("--expect-stopped")
    result = subprocess.run(command, cwd=ROOT, env=environment, text=True,
                            capture_output=True, timeout=12)
    try:
        receipt = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise Town12OwnerError(f"{stage} probe receipt is missing or invalid") from error
    if result.returncode != 0 or receipt.get("status") != "PASS":
        raise Town12OwnerError(f"{stage} probe failed: {receipt.get('error')}")
    if not stopped and receipt.get("active_map_name") not in LIVE_MAP_NAMES:
        raise Town12OwnerError(f"{stage} full map name differs from Town12 source level")
    return receipt


def _wait_ready(editor: subprocess.Popen[bytes], environment: dict[str, str]) -> None:
    deadline = time.monotonic() + shared_owner.STARTUP_TIMEOUT_SEC
    last_error = "Town12 source editor has not answered"
    while time.monotonic() < deadline:
        if editor.poll() is not None:
            raise Town12OwnerError(f"UE4Editor exited during startup: {editor.returncode}")
        try:
            _probe("ready", editor.pid, environment)
            return
        except Town12OwnerError as error:
            last_error = str(error)
            if "full map name differs" in last_error or "map mismatch" in last_error:
                raise
        time.sleep(2)
    raise Town12OwnerError(f"Town12 source editor startup timed out: {last_error}")


_LIVE_XODR_SCRIPT = """
# HH_260929 - Read exact live Town12 OpenDRIVE bytes without advancing or changing the world.
import hashlib
import json
import sys
import carla
client = carla.Client('127.0.0.1', 2100)
client.set_timeout(60.0)
carla_map = client.get_world().get_map()
name = str(carla_map.name)
if name not in ('/Game/Carla/Maps/Town12/Town12', 'Carla/Maps/Town12/Town12'):
    raise SystemExit('full live Town12 map name mismatch: ' + name)
xml = carla_map.to_opendrive()
if not isinstance(xml, str) or '<OpenDRIVE' not in xml:
    raise SystemExit('live Town12 OpenDRIVE unavailable')
payload = xml.encode('utf-8')
print(json.dumps({'map_name': name, 'sha256': hashlib.sha256(payload).hexdigest(),
                  'size_bytes': len(payload), 'read_only': True}))
"""


def _read_live_xodr(environment: dict[str, str]) -> dict[str, Any]:
    result = subprocess.run(
        [sys.executable, "-c", _LIVE_XODR_SCRIPT], cwd=ROOT, env=environment,
        text=True, capture_output=True, timeout=90,
    )
    if result.returncode != 0:
        raise Town12OwnerError("live Town12 OpenDRIVE read failed: " + result.stderr[-500:])
    try:
        record = json.loads(result.stdout)
    except ValueError as error:
        raise Town12OwnerError("live Town12 OpenDRIVE receipt is invalid") from error
    if (record.get("map_name") not in LIVE_MAP_NAMES
            or record.get("sha256") != XODR_SHA256
            or not isinstance(record.get("size_bytes"), int)
            or record["size_bytes"] < 1000000
            or record.get("read_only") is not True):
        raise Town12OwnerError("live Town12 OpenDRIVE differs from pinned source XODR")
    return record


def execute(profile: dict[str, Any], entry: dict[str, Any], assets: dict[str, Any],
            environment: dict[str, str]) -> int:
    with shared_owner._workspace_lock():
        locked = preflight()
        if locked != (profile, entry, assets, environment):
            raise Town12OwnerError("Town12 binding changed while acquiring the runtime lease")
        shared_owner._assert_no_other_carla()
        _assert_port_triplet_free()
        if OUTPUT.exists() or OUTPUT.is_symlink():
            raise Town12OwnerError(f"refusing to reuse an existing output: {OUTPUT}")
        OUTPUT.parent.mkdir(parents=True, exist_ok=True)
        OUTPUT.mkdir()
        (OUTPUT / "provenance").mkdir()
        (OUTPUT / "lifecycle").mkdir()
        hashes: dict[str, str] = {}
        editor: subprocess.Popen[bytes] | None = None
        live: dict[str, Any] | None = None
        error: str | None = None
        cleanup_ok = True
        try:
            hashes = _archive_sources()
            command = editor_command(profile, entry)
            _write_new_json(OUTPUT / "owner_plan.json", {
                "schema": "portable_e2e.owned_town12_source_editor_probe_plan.v1",
                "planned_at_utc": _utc(), "source_editor_profile": PROFILE_ID,
                "map_load_name": MAP_LOAD_NAME, "expected_source_xodr_sha256": XODR_SHA256,
                "source_assets": assets, "source_sha256": hashes,
                "editor_argv": command, "port_triplet": list(shared_owner.PORTS),
                "startup_timeout_sec": shared_owner.STARTUP_TIMEOUT_SEC,
                "live_xodr_timeout_sec": 90, "capture_enabled": False,
                "actor_spawn_enabled": False, "learned_model_control": False,
                "training_data_admitted": False,
            })
            if not shared_owner._verify_source_assets(assets) or not _verify_sources(hashes):
                raise Town12OwnerError("Town12 source bytes changed before editor launch")
            with (OUTPUT / "server.log").open("xb") as log:
                editor = subprocess.Popen(
                    command, cwd=Path(profile["project"]).parent, env=environment,
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                )
            _wait_ready(editor, environment)
            _write_new_json(OUTPUT / "owner_started.json", {
                "started_at_utc": _utc(), "editor_pid": editor.pid,
                "editor_pgid": editor.pid, "map_load_name": MAP_LOAD_NAME,
            })
            if not shared_owner._verify_source_assets(assets) or not _verify_sources(hashes):
                raise Town12OwnerError("Town12 source bytes changed after editor startup")
            live = _read_live_xodr(environment)
            _write_new_json(OUTPUT / "live_opendrive.json", live)
        except BaseException as exc:
            error = f"{type(exc).__name__}: {exc}"
        finally:
            if editor is not None:
                try:
                    cleanup_ok = shared_owner._stop_owned_group(editor)
                except (shared_owner.CatalogOwnerError, OSError):
                    cleanup_ok = False
                try:
                    _probe("stopped", editor.pid, environment, stopped=True)
                    _assert_port_triplet_free()
                except (Town12OwnerError, shared_owner.CatalogOwnerError, OSError,
                        subprocess.SubprocessError):
                    cleanup_ok = False
            sources_ok = _verify_sources(hashes)
            try:
                assets_ok = shared_owner._verify_source_assets(assets)
            except (OSError, KeyError, TypeError):
                assets_ok = False
            passed = error is None and cleanup_ok and sources_ok and assets_ok and live is not None
            _write_new_json(OUTPUT / "owner_result.json", {
                "schema": "portable_e2e.owned_town12_source_editor_probe_result.v1",
                "completed_at_utc": _utc(), "status": "PASS" if passed else "FAIL",
                "error": error, "live_opendrive": live,
                "source_bytes_unchanged_and_archived": sources_ok,
                "source_assets_unchanged": assets_ok,
                "owned_processes_stopped_and_ports_released": cleanup_ok,
                "capture_performed": False, "actor_spawned": False,
                "training_data_admitted": False, "learned_model_control": False,
            })
        return 0 if passed else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--execute", action="store_true",
                        help="cold-start one owned Town12 identity probe; no capture")
    args = parser.parse_args(argv)
    try:
        profile, entry, assets, environment = preflight()
        if not args.execute:
            output_absent = not (OUTPUT.exists() or OUTPUT.is_symlink())
            print(json.dumps({
                "schema": "portable_e2e.town12_source_editor_owner_preflight.v1",
                "status": "STATIC_READY_NOT_LIVE_TESTED" if output_absent else "STOP_OUTPUT_EXISTS",
                "read_only": True, "source_editor_argv": editor_command(profile, entry),
                "source_assets": assets, "output": str(OUTPUT),
                "output_absent": output_absent, "simulator_started": False,
                "capture_enabled": False, "training_data_admitted": False,
            }, indent=2, sort_keys=True))
            return 0 if output_absent else 2
        return execute(profile, entry, assets, environment)
    except (Town12OwnerError, shared_owner.CatalogOwnerError, ManifestError,
            KeyError, OSError, ValueError,
            TypeError, subprocess.SubprocessError) as error:
        print(f"Town12 source-editor owner preflight failed: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
