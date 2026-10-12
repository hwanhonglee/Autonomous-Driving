#!/usr/bin/env python3
"""HH_260928 - Own one byte-pinned stationary Town13 skydome location A-B-A diagnostic."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Any

from PIL import Image

try:
    from . import run_owned_town13_route_catalog as catalog_owner
    from . import capture_town13_skydome_location_aba as capture
except ImportError:
    from scripts.e2e import run_owned_town13_route_catalog as catalog_owner
    from scripts.e2e import capture_town13_skydome_location_aba as capture


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "storage/experiments/validation/2026-09-28/map_town13_skydome_location_aba_v1"
CATALOG_SHA256 = "ddd1832dc8456ccd47e1c47206d31e2af84a380fd91bdfb3be2665ae08686c2d"
SOURCE_FILES = tuple(dict.fromkeys((
    "tools/e2e/run_owned_town13_skydome_location_aba.py",
    "tools/e2e/capture_town13_skydome_location_aba.py",
    *catalog_owner.SOURCE_FILES,
)))
CAPTURE_TIMEOUT_SEC = 300


class OwnedABAFailure(RuntimeError):
    """HH_260928 - Fail closed for uncertain asset, output, camera or lifecycle identity."""


def _utc() -> str:
    return datetime.now(timezone.utc).isoformat()


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_new_json(path: Path, document: dict[str, Any]) -> None:
    with path.open("x", encoding="utf-8") as stream:
        json.dump(document, stream, indent=2, sort_keys=True)
        stream.write("\n")


def _regular(path: Path) -> bool:
    return path.is_file() and not path.is_symlink()


def _bound_inputs() -> dict[str, Path]:
    """HH_260928 - Require v2 baseline proof and both archived route identities."""
    paths = {
        "catalog.json": capture.ARCHIVE / "route_catalog.json",
        "catalog_owner_result.json": capture.ARCHIVE / "owner_result.json",
        "route_a.json": capture.ROUTE_A,
        "route_b.json": capture.ROUTE_B,
        "baseline_camera_frame0000.npz": capture.BASELINE_MATRIX,
        "baseline_owner_result.json": capture.BASELINE / "owner_result.json",
    }
    if any(not _regular(path) for path in paths.values()):
        raise OwnedABAFailure("a pinned Town13 input is missing or symlinked")
    if (_sha(paths["catalog.json"]) != CATALOG_SHA256
            or _sha(paths["route_a.json"]) != capture.ROUTE_SHA256[0]
            or _sha(paths["route_b.json"]) != capture.ROUTE_SHA256[1]
            or _sha(paths["baseline_camera_frame0000.npz"]) != capture.BASELINE_MATRIX_SHA256):
        raise OwnedABAFailure("Town13 catalog, route or baseline camera bytes changed")
    catalog = json.loads(paths["catalog.json"].read_text(encoding="utf-8"))
    proof = json.loads(paths["catalog_owner_result.json"].read_text(encoding="utf-8"))
    baseline = json.loads(paths["baseline_owner_result.json"].read_text(encoding="utf-8"))
    if (catalog.get("status") != "complete" or catalog.get("map_id") != "town13"
            or catalog.get("server_profile") != catalog_owner.PROFILE_ID
            or catalog.get("load_name") != catalog_owner.MAP_LOAD_NAME
            or proof.get("status") != "PASS"
            or proof.get("catalog", {}).get("sha256") != CATALOG_SHA256
            or proof.get("owned_processes_stopped_and_ports_released") is not True
            or baseline.get("status") != "PASS"
            or baseline.get("capture", {}).get("route_sha256") != capture.ROUTE_SHA256[0]
            or baseline.get("owned_processes_stopped_and_ports_released") is not True):
        raise OwnedABAFailure("Town13 catalog or v2 baseline owner proof changed")
    for route_id, path, digest, spawn in (
        ("town13_right_s0001_p00", capture.ROUTE_A, capture.ROUTE_SHA256[0], 1797),
        ("town13_right_s0002_p00", capture.ROUTE_B, capture.ROUTE_SHA256[1], 2198),
    ):
        entries = [item for item in catalog.get("routes", []) if item.get("id") == route_id]
        if (len(entries) != 1 or entries[0].get("status") != "ready"
                or entries[0].get("sha256") != digest
                or entries[0].get("start_spawn_index") != spawn
                or capture.ARCHIVE / entries[0].get("path", "") != path):
            raise OwnedABAFailure(f"pinned catalog route changed: {route_id}")
    capture.pin_inputs()
    return paths


def preflight() -> tuple[dict[str, Any], dict[str, Any], dict[str, Any], dict[str, str], dict[str, Path]]:
    profile, entry, assets, env = catalog_owner.preflight()
    paths = _bound_inputs()
    for name in SOURCE_FILES:
        if not _regular(ROOT / name):
            raise OwnedABAFailure(f"owner source missing or symlinked: {name}")
    result = subprocess.run(
        [sys.executable, "-c", "import numpy, PIL, carla"],
        cwd=ROOT, env=env, capture_output=True, text=True, timeout=30,
    )
    if result.returncode != 0:
        raise OwnedABAFailure("pinned source runtime lacks camera dependencies")
    return profile, entry, assets, env, paths


def _make_output() -> None:
    if OUTPUT.exists() or OUTPUT.is_symlink():
        raise OwnedABAFailure(f"refusing to reuse the existing output: {OUTPUT}")
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.mkdir()
    (OUTPUT / "provenance").mkdir()
    (OUTPUT / "lifecycle").mkdir()


def _archive(paths: dict[str, Path]) -> dict[str, str]:
    hashes = {}
    for name in SOURCE_FILES:
        source = ROOT / name
        if not _regular(source):
            raise OwnedABAFailure(f"owner source changed before archival: {name}")
        data = source.read_bytes()
        archived = OUTPUT / "provenance" / name
        archived.parent.mkdir(parents=True, exist_ok=True)
        with archived.open("xb") as stream:
            stream.write(data)
        hashes[name] = hashlib.sha256(data).hexdigest()
    for name, source in paths.items():
        if not _regular(source):
            raise OwnedABAFailure(f"pinned input changed before archival: {name}")
        data = source.read_bytes()
        with (OUTPUT / "provenance" / name).open("xb") as stream:
            stream.write(data)
        hashes[name] = hashlib.sha256(data).hexdigest()
    return hashes


def _verify_archives(hashes: dict[str, str], paths: dict[str, Path]) -> bool:
    try:
        if not hashes or _sha(paths["catalog.json"]) != CATALOG_SHA256:
            return False
        for name in SOURCE_FILES:
            if _sha(ROOT / name) != hashes[name] or _sha(OUTPUT / "provenance" / name) != hashes[name]:
                return False
        for name, source in paths.items():
            if _sha(source) != hashes[name] or _sha(OUTPUT / "provenance" / name) != hashes[name]:
                return False
        return True
    except (OSError, KeyError):
        return False


def _probe(stage: str, editor_pid: int, env: dict[str, str], *, stopped: bool = False) -> dict[str, Any]:
    """HH_260928 - Bind each RPC receipt to this editor PID and the full Town13 asset."""
    path = OUTPUT / "lifecycle" / f"{stage}.json"
    command = [
        sys.executable, str(ROOT / "tools/workspace_python.py"),
        str(ROOT / "tools/e2e/probe_carla_server.py"),
        "--host", "127.0.0.1", "--port", str(catalog_owner.PORTS[0]),
        "--timeout", "4", "--expected-map", "Town13", "--owner-pid", str(editor_pid),
        "--owner-pgid", str(editor_pid), "--generation-id", f"town13_skydome_aba_{editor_pid}",
        "--stage", stage, "--server-log", str(OUTPUT / "server.log"), "--output", str(path),
    ]
    if stopped:
        command.append("--expect-stopped")
    result = subprocess.run(command, cwd=ROOT, env=env, capture_output=True, text=True, timeout=12)
    try:
        receipt = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise OwnedABAFailure(f"{stage} probe receipt is missing or invalid") from error
    if result.returncode != 0 or receipt.get("status") != "PASS":
        raise OwnedABAFailure(f"{stage} probe failed: {receipt.get('error')}")
    if not stopped and receipt.get("active_map_name") not in catalog_owner.LIVE_MAP_NAMES:
        raise OwnedABAFailure(f"{stage} full map name differs from Town13")
    return receipt


def _wait_ready(editor: subprocess.Popen[bytes], env: dict[str, str]) -> dict[str, Any]:
    deadline = time.monotonic() + catalog_owner.STARTUP_TIMEOUT_SEC
    last_error = "Town13 source editor has not answered"
    while time.monotonic() < deadline:
        if editor.poll() is not None:
            raise OwnedABAFailure(f"UE4Editor exited during startup: {editor.returncode}")
        try:
            return _probe("ready", editor.pid, env)
        except OwnedABAFailure as error:
            last_error = str(error)
            if "full map name differs" in last_error or "map mismatch" in last_error:
                raise
        time.sleep(2)
    raise OwnedABAFailure(f"Town13 source editor startup timed out: {last_error}")


def _assess(hashes: dict[str, str], paths: dict[str, Path]) -> dict[str, Any]:
    """HH_260928 - PASS describes capture integrity, never acceptable map imagery."""
    scan = OUTPUT / "scan"
    summary_path, records_path = scan / "summary.json", scan / "records.json"
    if not _regular(summary_path) or not _regular(records_path):
        raise OwnedABAFailure("stationary camera summary or records are absent")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    records = json.loads(records_path.read_text(encoding="utf-8"))
    if (summary.get("status") != "COMPLETE_DIAGNOSTIC_ONLY"
            or summary.get("map") not in catalog_owner.LIVE_MAP_NAMES
            or summary.get("phase_order") != list(capture.PHASES)
            or summary.get("saved_frame_count") != 30
            or summary.get("warmup_ticks_per_phase") != capture.WARMUP_TICKS
            or summary.get("saved_frames_per_phase") != capture.SAVED_FRAMES_PER_PHASE
            or summary.get("learned_model_control") is not False
            or summary.get("map_dev_admitted") is not False
            or summary.get("training_data_admitted") is not False
            or summary.get("cleanup_errors") != []
            or not isinstance(records, list) or len(records) != 30):
        raise OwnedABAFailure("stationary camera summary differs from the fixed diagnostic scope")
    expected_names = {f"{phase}_{index:02d}.png" for phase in capture.PHASES
                      for index in range(capture.SAVED_FRAMES_PER_PHASE)}
    frames = scan / "frames"
    if frames.is_symlink() or not frames.is_dir() or {p.name for p in frames.glob("*.png")} != expected_names:
        raise OwnedABAFailure("stationary camera PNG count or naming differs")
    for phase_no, phase in enumerate(capture.PHASES):
        for index in range(capture.SAVED_FRAMES_PER_PHASE):
            row = records[phase_no * capture.SAVED_FRAMES_PER_PHASE + index]
            name = f"frames/{phase}_{index:02d}.png"
            path = scan / name
            if (row.get("phase") != phase or row.get("index") != index
                    or row.get("png") != name or not _regular(path)
                    or row.get("png_sha256") != _sha(path)):
                raise OwnedABAFailure("stationary camera record or PNG bytes differ")
            with Image.open(path) as image:
                if image.size != (640, 360) or image.mode != "RGB":
                    raise OwnedABAFailure("stationary camera PNG dimensions or color differ")
                image.verify()
    if not _verify_archives(hashes, paths):
        raise OwnedABAFailure("owner source or pinned input bytes changed during capture")
    return {"status": "CAPTURE_DIAGNOSTIC_ONLY", "frame_count": 30,
            "phase_warning_counts": summary["phase_warning_counts"],
            "map_dev_admitted": False, "training_data_admitted": False}


def execute(profile: dict[str, Any], entry: dict[str, Any], assets: dict[str, Any],
            env: dict[str, str], paths: dict[str, Path]) -> int:
    with catalog_owner._workspace_lock():
        if preflight() != (profile, entry, assets, env, paths):
            raise OwnedABAFailure("Town13 owner binding changed while acquiring runtime lock")
        catalog_owner._assert_no_other_carla()
        catalog_owner._assert_ports_free()
        _make_output()
        hashes: dict[str, str] = {}
        editor = None
        worker = None
        error = None
        capture_result = None
        cleanup_ok = True
        try:
            hashes = _archive(paths)
            editor_command = catalog_owner.editor_command(profile, entry)
            _write_new_json(OUTPUT / "owner_plan.json", {
                "schema": "portable_e2e.town13_skydome_aba_owner_plan.v1",
                "planned_at_utc": _utc(), "map_id": "town13", "map_load_name": catalog_owner.MAP_LOAD_NAME,
                "source_editor_profile": catalog_owner.PROFILE_ID, "source_assets": assets,
                "source_and_input_sha256": hashes, "source_and_inputs_archived": True,
                "editor_argv": editor_command, "port_triplet": list(catalog_owner.PORTS),
                "phase_order": list(capture.PHASES), "warmup_ticks_per_phase": capture.WARMUP_TICKS,
                "saved_frames_per_phase": capture.SAVED_FRAMES_PER_PHASE,
                "collector_timeout_sec": CAPTURE_TIMEOUT_SEC,
                "map_dev_admitted": False, "training_data_admitted": False,
            })
            if not _verify_archives(hashes, paths) or not catalog_owner._verify_source_assets(assets):
                raise OwnedABAFailure("source assets or owner inputs changed before editor launch")
            with (OUTPUT / "server.log").open("xb") as log:
                editor = subprocess.Popen(editor_command, cwd=Path(profile["project"]).parent,
                                          env=env, stdout=log, stderr=subprocess.STDOUT,
                                          start_new_session=True)
            _wait_ready(editor, env)
            if not _verify_archives(hashes, paths) or not catalog_owner._verify_source_assets(assets):
                raise OwnedABAFailure("source assets or owner inputs changed before worker launch")
            worker_command = [sys.executable, str(ROOT / "tools/workspace_python.py"),
                              "-m", "scripts.e2e.capture_town13_skydome_location_aba", "--",
                              str(OUTPUT / "scan"), "--owner-pid", str(os.getpid())]
            with (OUTPUT / "collector.log").open("xb") as log:
                worker = subprocess.Popen(worker_command, cwd=ROOT, env=env, stdout=log,
                                          stderr=subprocess.STDOUT, start_new_session=True)
            try:
                worker.wait(timeout=CAPTURE_TIMEOUT_SEC)
            except subprocess.TimeoutExpired as exc:
                raise OwnedABAFailure("stationary camera worker timed out") from exc
            if worker.returncode != 0:
                raise OwnedABAFailure(f"stationary camera worker failed: {worker.returncode}")
            if not catalog_owner._stop_owned_group(worker):
                raise OwnedABAFailure("stationary camera worker process group did not stop")
            worker = None
            _probe("after_capture", editor.pid, env)
            capture_result = _assess(hashes, paths)
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
                except (OwnedABAFailure, catalog_owner.CatalogOwnerError, OSError,
                        subprocess.SubprocessError):
                    cleanup_ok = False
            source_ok = _verify_archives(hashes, paths)
            try:
                asset_ok = catalog_owner._verify_source_assets(assets)
            except (OSError, KeyError, TypeError):
                asset_ok = False
            passed = error is None and capture_result is not None and cleanup_ok and source_ok and asset_ok
            _write_new_json(OUTPUT / "owner_result.json", {
                "schema": "portable_e2e.town13_skydome_aba_owner_result.v1",
                "completed_at_utc": _utc(), "status": "PASS" if passed else "FAIL",
                "error": error, "capture": capture_result,
                "source_and_input_bytes_unchanged_and_archived": source_ok,
                "source_assets_unchanged": asset_ok,
                "owned_processes_stopped_and_ports_released": cleanup_ok,
                "map_dev_admitted": False, "training_data_admitted": False,
                "learned_model_control": False,
            })
        return 0 if passed else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args(argv)
    try:
        profile, entry, assets, env, paths = preflight()
        if not args.execute:
            output_absent = not (OUTPUT.exists() or OUTPUT.is_symlink())
            print(json.dumps({"status": "STATIC_READY_NOT_LIVE_TESTED" if output_absent else "STOP_OUTPUT_EXISTS",
                              "read_only": True, "output": str(OUTPUT), "output_absent": output_absent,
                              "source_editor_argv": catalog_owner.editor_command(profile, entry),
                              "input_sha256": {name: _sha(path) for name, path in paths.items()},
                              "phase_order": list(capture.PHASES), "simulator_started": False,
                              "map_dev_admitted": False}, indent=2, sort_keys=True))
            return 0 if output_absent else 2
        return execute(profile, entry, assets, env, paths)
    except (OwnedABAFailure, catalog_owner.CatalogOwnerError, OSError, ValueError,
            TypeError, subprocess.SubprocessError) as error:
        print(f"Town13 stationary skydome owner preflight failed: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
