#!/usr/bin/env python3
"""Own one byte-pinned Town13 camera diagnostic, never map DEV admission."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Any, Iterator

try:
    from . import run_owned_town13_route_catalog as catalog_owner
    from .collect_carla_mapping_probe import parse_args as parse_collector_args
except ImportError:
    from scripts.e2e import run_owned_town13_route_catalog as catalog_owner
    from scripts.e2e.collect_carla_mapping_probe import parse_args as parse_collector_args


ROOT = Path(__file__).resolve().parents[2]
# HH_260928 - Retain the first source-race failure and use a fresh, immutable retry output.
OUTPUT = ROOT / "storage/experiments/validation/2026-09-28/map_town13_render_sensor_scout_v2"
CATALOG_ROOT = ROOT / "storage/experiments/validation/2026-09-27/map_town13_route_catalog_v2"
CATALOG_SHA256 = "ddd1832dc8456ccd47e1c47206d31e2af84a380fd91bdfb3be2665ae08686c2d"
ROUTE_RELATIVE = Path("routes/town13/right/town13_right_s0001_p00.json")
ROUTE_SHA256 = "6927c0de706e5ffb89afce25a75a892d56f7ee212b577002787605802839520b"
ROUTE_ID = "town13_right_s0001_p00"
FRAMES = 240
CAPTURE_TIMEOUT_SEC = 900
SOURCE_FILES = tuple(dict.fromkeys((
    "tools/e2e/run_owned_town13_mapping_scout.py",
    "tools/e2e/collect_carla_mapping_probe.py",
    "learning/portable_e2e/mapping.py",
    "learning/portable_e2e/mapping_projection.py",
    *catalog_owner.SOURCE_FILES,
)))


class ScoutOwnerError(RuntimeError):
    """HH_260928 - Fail closed when any owner or capture identity is uncertain."""


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


def _regular_unsymlinked(path: Path) -> bool:
    return path.is_file() and not path.is_symlink()


def _catalog_route_binding() -> dict[str, Any]:
    """HH_260928 - Bind exactly one archived route to the successful v2 catalog."""
    if CATALOG_ROOT.is_symlink() or not CATALOG_ROOT.is_dir():
        raise ScoutOwnerError("pinned Town13 catalog directory is absent or symlinked")
    catalog_path = CATALOG_ROOT / "route_catalog.json"
    owner_result_path = CATALOG_ROOT / "owner_result.json"
    route_path = CATALOG_ROOT / ROUTE_RELATIVE
    if not all(_regular_unsymlinked(path) for path in (catalog_path, owner_result_path, route_path)):
        raise ScoutOwnerError("pinned Town13 catalog, owner result, or route is absent or symlinked")
    if any(path.is_symlink() for path in (route_path.parent, route_path.parent.parent, route_path.parent.parent.parent)):
        raise ScoutOwnerError("pinned Town13 route has a symlinked catalog parent")
    if _sha256(catalog_path) != CATALOG_SHA256 or _sha256(route_path) != ROUTE_SHA256:
        raise ScoutOwnerError("pinned Town13 catalog or route bytes changed")
    catalog = json.loads(catalog_path.read_text(encoding="utf-8"))
    owner_result = json.loads(owner_result_path.read_text(encoding="utf-8"))
    if (owner_result.get("status") != "PASS"
            or owner_result.get("catalog", {}).get("sha256") != CATALOG_SHA256
            or owner_result.get("source_assets_unchanged") is not True
            or owner_result.get("source_bytes_unchanged_and_archived") is not True
            or owner_result.get("owned_processes_stopped_and_ports_released") is not True
            or catalog.get("status") != "complete"
            or catalog.get("map_id") != "town13"
            or catalog.get("server_profile") != catalog_owner.PROFILE_ID
            or catalog.get("load_name") != catalog_owner.MAP_LOAD_NAME):
        raise ScoutOwnerError("Town13 v2 catalog owner proof or map identity changed")
    entries = [item for item in catalog.get("routes", []) if item.get("id") == ROUTE_ID]
    if len(entries) != 1:
        raise ScoutOwnerError("exact Town13 diagnostic route missing or duplicated")
    entry = entries[0]
    if (entry.get("status") != "ready" or entry.get("scenario") != "right"
            or entry.get("path") != ROUTE_RELATIVE.as_posix()
            or entry.get("sha256") != ROUTE_SHA256
            or entry.get("start_spawn_index") != 1797
            or entry.get("goal_spawn_index") != 8186):
        raise ScoutOwnerError("Town13 catalog route record differs from preregistration")
    route = json.loads(route_path.read_text(encoding="utf-8"))
    if (route.get("town") != "Town13" or route.get("coordinate_reference") != "base_link"
            or route.get("scenario") != "right"
            or route.get("start_spawn_index") != 1797
            or route.get("goal_spawn_index") != 8186
            or not isinstance(route.get("route"), list) or len(route["route"]) < 2):
        raise ScoutOwnerError("Town13 route contents differ from preregistration")
    return {
        "catalog_path": str(catalog_path), "catalog_sha256": CATALOG_SHA256,
        "catalog_owner_result_path": str(owner_result_path),
        "route_path": str(route_path), "route_sha256": ROUTE_SHA256,
        "route_id": ROUTE_ID, "start_spawn_index": 1797, "goal_spawn_index": 8186,
    }


def collector_arguments(binding: dict[str, Any], output: Path | None = None) -> list[str]:
    if output is None:
        output = OUTPUT
    return [
        str(output / "mapping"), binding["route_path"],
        "--host", "127.0.0.1", "--port", str(catalog_owner.PORTS[0]),
        "--frames", str(FRAMES), "--motion-mode", "basic-agent",
        "--ignore-traffic-lights", "--town13-render-sensor-scout",
    ]


def _assert_collector_arguments(arguments: list[str]) -> None:
    parsed = parse_collector_args(arguments)
    if (parsed.output != OUTPUT / "mapping"
            or parsed.route_file != CATALOG_ROOT / ROUTE_RELATIVE
            or parsed.host != "127.0.0.1" or parsed.port != catalog_owner.PORTS[0]
            or parsed.frames != FRAMES or parsed.motion_mode != "basic-agent"
            or parsed.ignore_traffic_lights is not True
            or parsed.town13_render_sensor_scout is not True
            or parsed.allow_map_load is not False):
        raise ScoutOwnerError("Town13 mapping collector arguments escaped the fixed scope")


def _assert_capture_imports(env: dict[str, str]) -> None:
    """HH_260928 - Check source-runtime image dependencies before launching UE4Editor."""
    result = subprocess.run(
        [sys.executable, "-c", "import numpy, PIL, carla, agents"],
        cwd=ROOT, env=env, text=True, capture_output=True, timeout=30,
    )
    if result.returncode != 0:
        raise ScoutOwnerError("pinned source runtime lacks a camera-capture dependency")


def preflight() -> tuple[dict[str, Any], dict[str, Any], dict[str, Any], dict[str, str], dict[str, Any]]:
    if sys.version_info[:2] != (3, 10):
        raise ScoutOwnerError("pinned source CARLA egg requires Python 3.10")
    profile, map_entry, assets, env = catalog_owner.preflight()
    binding = _catalog_route_binding()
    for name in SOURCE_FILES:
        source = ROOT / name
        if source.is_symlink() or not source.is_file():
            raise ScoutOwnerError(f"camera owner source missing or symlinked: {name}")
    _assert_collector_arguments(collector_arguments(binding))
    _assert_capture_imports(env)
    return profile, map_entry, assets, env, binding


@contextmanager
def _workspace_lock() -> Iterator[None]:
    """HH_260928 - Use the catalog owner's shared nonblocking workspace lease."""
    with catalog_owner._workspace_lock():
        yield


def _make_output() -> None:
    if OUTPUT.exists() or OUTPUT.is_symlink():
        raise ScoutOwnerError(f"refusing to reuse an existing scout output: {OUTPUT}")
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.mkdir()
    (OUTPUT / "provenance").mkdir()
    (OUTPUT / "lifecycle").mkdir()


def _archive_sources_and_route(binding: dict[str, Any]) -> dict[str, str]:
    hashes: dict[str, str] = {}
    for name in SOURCE_FILES:
        source = ROOT / name
        if not _regular_unsymlinked(source):
            raise ScoutOwnerError(f"camera owner source missing or symlinked: {name}")
        data = source.read_bytes()
        archive = OUTPUT / "provenance" / name
        archive.parent.mkdir(parents=True, exist_ok=True)
        with archive.open("xb") as stream:
            stream.write(data)
        hashes[name] = hashlib.sha256(data).hexdigest()
    source = Path(binding["route_path"])
    if not _regular_unsymlinked(source) or _sha256(source) != ROUTE_SHA256:
        raise ScoutOwnerError("Town13 route changed before source archival")
    archive = OUTPUT / "provenance" / "route.json"
    with archive.open("xb") as stream:
        stream.write(source.read_bytes())
    hashes["route"] = ROUTE_SHA256
    # HH_260928 - Retain the exact catalog and its successful owner proof with the camera attempt.
    for key, archive_name in (("catalog_path", "catalog.json"),
                              ("catalog_owner_result_path", "catalog_owner_result.json")):
        source = Path(binding[key])
        if not _regular_unsymlinked(source):
            raise ScoutOwnerError(f"Town13 {key} changed before source archival")
        data = source.read_bytes()
        if key == "catalog_path" and hashlib.sha256(data).hexdigest() != CATALOG_SHA256:
            raise ScoutOwnerError("Town13 catalog changed before source archival")
        archive = OUTPUT / "provenance" / archive_name
        with archive.open("xb") as stream:
            stream.write(data)
        hashes[archive_name] = hashlib.sha256(data).hexdigest()
    return hashes


def _verify_sources_and_route(hashes: dict[str, str], binding: dict[str, Any]) -> bool:
    if (not hashes or hashes.get("route") != ROUTE_SHA256
            or hashes.get("catalog.json") != CATALOG_SHA256
            or not hashes.get("catalog_owner_result.json")):
        return False
    try:
        for name in SOURCE_FILES:
            expected = hashes[name]
            if (_sha256(ROOT / name) != expected
                    or _sha256(OUTPUT / "provenance" / name) != expected):
                return False
        return (_sha256(Path(binding["route_path"])) == ROUTE_SHA256
                and _sha256(OUTPUT / "provenance" / "route.json") == ROUTE_SHA256
                and _sha256(Path(binding["catalog_path"])) == CATALOG_SHA256
                and _sha256(OUTPUT / "provenance" / "catalog.json") == CATALOG_SHA256
                and _sha256(Path(binding["catalog_owner_result_path"]))
                == hashes["catalog_owner_result.json"]
                and _sha256(OUTPUT / "provenance" / "catalog_owner_result.json")
                == hashes["catalog_owner_result.json"])
    except (OSError, KeyError):
        return False


def _probe(stage: str, editor_pid: int, env: dict[str, str], *, stopped: bool = False) -> dict[str, Any]:
    """HH_260928 - Require full Town13 asset identity beyond the generic probe basename."""
    path = OUTPUT / "lifecycle" / f"{stage}.json"
    command = [
        sys.executable, str(ROOT / "tools/workspace_python.py"),
        str(ROOT / "tools/e2e/probe_carla_server.py"),
        "--host", "127.0.0.1", "--port", str(catalog_owner.PORTS[0]),
        "--timeout", "4", "--expected-map", "Town13",
        "--owner-pid", str(editor_pid), "--owner-pgid", str(editor_pid),
        "--generation-id", f"town13_mapping_scout_{editor_pid}",
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
        raise ScoutOwnerError(f"{stage} probe receipt is missing or invalid") from error
    if result.returncode != 0 or receipt.get("status") != "PASS":
        raise ScoutOwnerError(f"{stage} probe failed: {receipt.get('error')}")
    if not stopped and receipt.get("active_map_name") not in catalog_owner.LIVE_MAP_NAMES:
        raise ScoutOwnerError(f"{stage} full map name differs from Town13 source level")
    return receipt


def _wait_ready(editor: subprocess.Popen[bytes], env: dict[str, str]) -> dict[str, Any]:
    deadline = time.monotonic() + catalog_owner.STARTUP_TIMEOUT_SEC
    last_error = "Town13 source editor has not answered"
    while time.monotonic() < deadline:
        if editor.poll() is not None:
            raise ScoutOwnerError(f"UE4Editor exited during startup: {editor.returncode}")
        try:
            return _probe("ready", editor.pid, env)
        except ScoutOwnerError as error:
            last_error = str(error)
            if "full map name differs" in last_error or "map mismatch" in last_error:
                raise
        time.sleep(2)
    raise ScoutOwnerError(f"Town13 source editor startup timed out: {last_error}")


def _validate_frame_files(mapping: Path, stem: str) -> None:
    """HH_260928 - Decode every stored RGB and oracle array without admitting it as a label."""
    import numpy as np
    from PIL import Image

    png_path = mapping / f"{stem}.png"
    npz_path = mapping / f"{stem}.npz"
    if not _regular_unsymlinked(png_path) or not _regular_unsymlinked(npz_path):
        raise ScoutOwnerError(f"missing or symlinked camera pair: {stem}")
    with Image.open(png_path) as image:
        if image.size != (640, 360) or image.mode != "RGB":
            raise ScoutOwnerError(f"incorrect RGB dimensions or mode: {stem}")
        image.verify()
    with np.load(npz_path, allow_pickle=False) as arrays:
        if (set(arrays.files) != {"depth", "semantic", "world_from_camera"}
                or arrays["depth"].shape != (360, 640, 4)
                or arrays["semantic"].shape != (360, 640, 4)
                or arrays["world_from_camera"].shape != (4, 4)
                or arrays["depth"].dtype != np.uint8
                or arrays["semantic"].dtype != np.uint8
                or not np.isfinite(arrays["world_from_camera"]).all()):
            raise ScoutOwnerError(f"incorrect depth, semantic, or pose arrays: {stem}")


def _assess_capture(binding: dict[str, Any]) -> dict[str, Any]:
    """HH_260928 - PASS means only a sound owned sensor diagnostic, not sign DEV."""
    mapping = OUTPUT / "mapping"
    summary_path = mapping / "summary.json"
    records_path = mapping / "records.json"
    map_path = mapping / "map.json"
    if not all(_regular_unsymlinked(path) for path in (summary_path, records_path, map_path)):
        raise ScoutOwnerError("mapping summary, records, or replay map is absent")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    records = json.loads(records_path.read_text(encoding="utf-8"))
    if (summary.get("status") != "COMPLETE_ORACLE_PLUMBING_ONLY"
            or summary.get("town") != "Town13"
            or summary.get("route_sha256") != ROUTE_SHA256
            or summary.get("research_split_intent") != "town13_render_sensor_diagnostic_only"
            or summary.get("frame_count") != FRAMES
            or summary.get("motion_mode") != "basic-agent"
            or summary.get("controller") != "CARLA BasicAgent"
            or summary.get("physics_driving") is not True
            or summary.get("ignores_traffic_lights") is not True
            or summary.get("learned_model_control") is not False
            or summary.get("hd_map_approved") is not False
            or summary.get("oracle_pose") is not True
            or summary.get("oracle_depth") is not True
            or summary.get("oracle_semantics") is not True
            or summary.get("collision_events") != 0
            or summary.get("live_replay_exact_match") is not True
            or summary.get("cleanup_errors") != []
            or not isinstance(records, list) or len(records) != FRAMES):
        raise ScoutOwnerError("Town13 camera diagnostic summary failed its fixed contract")
    if not isinstance(summary.get("travel_m"), (int, float)) or summary["travel_m"] < 3:
        raise ScoutOwnerError("Town13 camera diagnostic did not move at least 3m")
    frames_path = mapping / "frames"
    if frames_path.is_symlink() or not frames_path.is_dir():
        raise ScoutOwnerError("camera frame directory is absent or symlinked")
    expected_png = {f"{index:04d}.png" for index in range(FRAMES)}
    expected_npz = {f"{index:04d}.npz" for index in range(FRAMES)}
    if ({path.name for path in frames_path.glob("*.png")} != expected_png
            or {path.name for path in frames_path.glob("*.npz")} != expected_npz):
        raise ScoutOwnerError("camera frame-pair count or naming differs from 240")
    prior_frame: int | None = None
    prior_timestamp: float | None = None
    for index, row in enumerate(records):
        stem = f"frames/{index:04d}"
        if not isinstance(row, dict) or row.get("stem") != stem:
            raise ScoutOwnerError("camera record order or stem differs")
        frame = row.get("frame")
        timestamp = row.get("timestamp_s")
        if (not isinstance(frame, int) or isinstance(frame, bool)
                or not isinstance(timestamp, (int, float)) or not math.isfinite(timestamp)
                or (prior_frame is not None and frame != prior_frame + 1)
                or (prior_timestamp is not None and timestamp <= prior_timestamp)):
            raise ScoutOwnerError("camera frame or timestamp sequence differs")
        _validate_frame_files(mapping, stem)
        prior_frame, prior_timestamp = frame, float(timestamp)
    if not _verify_sources_and_route(binding["source_sha256"], binding):
        raise ScoutOwnerError("camera owner source or route changed during capture")
    return {
        "status": "CAPTURE_DIAGNOSTIC_ONLY", "frame_count": FRAMES,
        "route_id": ROUTE_ID, "route_sha256": ROUTE_SHA256,
        "rgb_depth_semantic_pairs_decoded": FRAMES,
        "collision_events": 0, "travel_m": summary["travel_m"],
        "live_replay_exact_match": True, "map_dev_admitted": False,
        "training_data_admitted": False,
    }


def execute(profile: dict[str, Any], entry: dict[str, Any], assets: dict[str, Any],
            env: dict[str, str], binding: dict[str, Any]) -> int:
    with _workspace_lock():
        locked_profile, locked_entry, locked_assets, locked_env, locked_binding = preflight()
        if (locked_profile != profile or locked_entry != entry or locked_assets != assets
                or locked_binding != binding or locked_env != env):
            raise ScoutOwnerError("Town13 binding changed while acquiring the runtime lease")
        catalog_owner._assert_no_other_carla()
        catalog_owner._assert_ports_free()
        _make_output()
        hashes: dict[str, str] = {}
        editor: subprocess.Popen[bytes] | None = None
        worker: subprocess.Popen[bytes] | None = None
        error: str | None = None
        capture_result: dict[str, Any] | None = None
        cleanup_ok = True
        try:
            hashes = _archive_sources_and_route(binding)
            binding = {**binding, "source_sha256": hashes}
            command = catalog_owner.editor_command(profile, entry)
            collector_args = collector_arguments(binding)
            _assert_collector_arguments(collector_args)
            _write_new_json(OUTPUT / "owner_plan.json", {
                "schema": "portable_e2e.owned_town13_mapping_scout_plan.v1",
                "planned_at_utc": _utc(), "map_id": "town13",
                "split": "validation_diagnostic_only", "source_editor_profile": catalog_owner.PROFILE_ID,
                "map_load_name": catalog_owner.MAP_LOAD_NAME,
                "accepted_live_map_names": sorted(catalog_owner.LIVE_MAP_NAMES),
                "catalog_sha256": CATALOG_SHA256, "route_id": ROUTE_ID,
                "route_path": binding["route_path"], "route_sha256": ROUTE_SHA256,
                "start_spawn_index": 1797, "goal_spawn_index": 8186,
                "source_assets": assets, "source_sha256": hashes,
                "source_and_route_bytes_archived": True,
                "editor_argv": command, "collector_argv": collector_args,
                "startup_timeout_sec": catalog_owner.STARTUP_TIMEOUT_SEC,
                "collector_timeout_sec": CAPTURE_TIMEOUT_SEC,
                "port_triplet": list(catalog_owner.PORTS), "allow_map_load": False,
                "expected_frames": FRAMES, "capture_intent": "render_sensor_diagnostic_only",
                "learned_model_control": False, "map_dev_admitted": False,
                "training_data_admitted": False,
            })
            if not catalog_owner._verify_source_assets(assets):
                raise ScoutOwnerError("Town13 source assets changed before editor launch")
            with (OUTPUT / "server.log").open("xb") as log:
                editor = subprocess.Popen(
                    command, cwd=Path(profile["project"]).parent, env=env,
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                )
            _wait_ready(editor, env)
            _write_new_json(OUTPUT / "owner_started.json", {
                "started_at_utc": _utc(), "editor_pid": editor.pid,
                "editor_pgid": editor.pid, "port_triplet": list(catalog_owner.PORTS),
                "route_sha256": ROUTE_SHA256, "map_load_name": catalog_owner.MAP_LOAD_NAME,
            })
            if not _verify_sources_and_route(hashes, binding):
                raise ScoutOwnerError("source, route, or catalog changed before collector launch")
            if not catalog_owner._verify_source_assets(assets):
                raise ScoutOwnerError("Town13 source assets changed before collector launch")
            worker_command = [
                sys.executable, str(ROOT / "tools/workspace_python.py"),
                "-m", "scripts.e2e.collect_carla_mapping_probe", "--", *collector_args,
            ]
            with (OUTPUT / "collector.log").open("xb") as log:
                worker = subprocess.Popen(
                    worker_command, cwd=ROOT, env=env, stdout=log,
                    stderr=subprocess.STDOUT, start_new_session=True,
                )
            try:
                worker.wait(timeout=CAPTURE_TIMEOUT_SEC)
            except subprocess.TimeoutExpired as exc:
                raise ScoutOwnerError("Town13 camera diagnostic worker timed out") from exc
            if worker.returncode != 0:
                raise ScoutOwnerError(f"Town13 camera diagnostic worker failed: {worker.returncode}")
            if not catalog_owner._stop_owned_group(worker):
                raise ScoutOwnerError("Town13 camera worker group did not stop")
            worker = None
            _probe("after_capture", editor.pid, env)
            capture_result = _assess_capture(binding)
        except BaseException as exc:
            error = f"{type(exc).__name__}: {exc}"
        finally:
            if worker is not None:
                try:
                    if not catalog_owner._stop_owned_group(worker):
                        cleanup_ok = False
                except (catalog_owner.CatalogOwnerError, OSError):
                    cleanup_ok = False
            if editor is not None:
                try:
                    if not catalog_owner._stop_owned_group(editor):
                        cleanup_ok = False
                except (catalog_owner.CatalogOwnerError, OSError):
                    cleanup_ok = False
                try:
                    _probe("stopped", editor.pid, env, stopped=True)
                    catalog_owner._assert_ports_free()
                except (ScoutOwnerError, catalog_owner.CatalogOwnerError, OSError, subprocess.SubprocessError):
                    cleanup_ok = False
            sources_ok = _verify_sources_and_route(hashes, binding)
            try:
                assets_ok = catalog_owner._verify_source_assets(assets)
            except (OSError, KeyError, TypeError):
                assets_ok = False
            passed = (error is None and cleanup_ok and sources_ok and assets_ok
                      and capture_result is not None)
            _write_new_json(OUTPUT / "owner_result.json", {
                "schema": "portable_e2e.owned_town13_mapping_scout_result.v1",
                "completed_at_utc": _utc(), "status": "PASS" if passed else "FAIL",
                "error": error, "capture": capture_result,
                "source_and_route_bytes_unchanged_and_archived": sources_ok,
                "source_assets_unchanged": assets_ok,
                "owned_processes_stopped_and_ports_released": cleanup_ok,
                "map_dev_admitted": False, "training_data_admitted": False,
                "learned_model_control": False,
            })
        return 0 if passed else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--execute", action="store_true",
                        help="cold-start exactly one owned Town13 render/sensor diagnostic")
    args = parser.parse_args(argv)
    try:
        profile, entry, assets, env, binding = preflight()
        if not args.execute:
            output_absent = not (OUTPUT.exists() or OUTPUT.is_symlink())
            print(json.dumps({
                "schema": "portable_e2e.town13_mapping_scout_owner_preflight.v1",
                "status": "STATIC_READY_NOT_LIVE_TESTED" if output_absent else "STOP_OUTPUT_EXISTS",
                "read_only": True, "source_editor_argv": catalog_owner.editor_command(profile, entry),
                "collector_argv": collector_arguments(binding), "catalog_and_route": binding,
                "source_assets": assets, "output": str(OUTPUT), "output_absent": output_absent,
                "simulator_started": False, "map_dev_admitted": False,
                "training_data_admitted": False,
            }, indent=2, sort_keys=True))
            return 0 if output_absent else 2
        return execute(profile, entry, assets, env, binding)
    except (ScoutOwnerError, catalog_owner.CatalogOwnerError, OSError, ValueError,
            TypeError, subprocess.SubprocessError) as error:
        print(f"Town13 camera owner preflight failed: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
