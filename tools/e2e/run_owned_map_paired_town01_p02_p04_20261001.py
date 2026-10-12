#!/usr/bin/env python3
"""HH_261001 - Own one bounded Town01 p02 or p04 four-camera capture."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time

from . import collect_map_paired_town01_p02_p04_20261001 as capture


ROOT = Path(__file__).resolve().parents[2]
CAMPAIGN = ROOT / "storage/experiments/validation/2026-10-01/map_paired_p02_p04_v1"
MIN_START_FREE = 6 * 1024**3
MIN_RUNNING_FREE = 5 * 1024**3
MAX_OWNER_BYTES = 400 * 1024**2
MAX_CAMPAIGN_BYTES = 1024**3
CAPTURE_MODE = "mapping-paired-rgb"
SOURCES = (
    "tools/e2e/run_owned_map_paired_town01_p02_p04_20261001.sh",
    "tools/e2e/run_owned_map_paired_town01_p02_p04_20261001.py",
    "tools/e2e/collect_map_paired_town01_p02_p04_20261001.py",
    "tools/e2e/audit_map_paired_town01_p02_p04_20261001.py",
    "tools/e2e/collect_map_paired_rgb_20261001_v2.py",
    "tools/e2e/audit_map_paired_rgb_20261001_v2.py",
    "tools/e2e/audit_ctrack_isolated_moving_20261001.py",
    "tools/e2e/probe_carla_server.py",
    "tools/e2e/run_carla_map.sh",
    "tools/e2e/env.sh",
    "tools/workspace_python.py",
    "tools/canonical_workspace.py",
    "learning/portable_e2e/mapping.py",
    "learning/portable_e2e/mapping_projection.py",
    "learning/tests/test_map_paired_town01_p02_p04_20261001.py",
    "docs/assets/validation/2026-10-01/map_paired_p02_p04/PLAN.md",
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def write_json(path: Path, payload: dict) -> None:
    with path.open("x") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def tree_bytes(path: Path) -> int:
    """HH_261001 - Count regular evidence bytes without following links."""
    if not path.exists():
        return 0
    total = 0
    for root, dirs, files in os.walk(path, followlinks=False):
        dirs[:] = [name for name in dirs if not (Path(root) / name).is_symlink()]
        for name in files:
            item = Path(root) / name
            if item.is_symlink():
                raise ValueError("evidence tree contains a symlink")
            total += item.stat().st_size
    return total


def check_storage(owner: Path | None = None) -> None:
    if shutil.disk_usage(ROOT).free < (MIN_RUNNING_FREE if owner else MIN_START_FREE):
        raise ValueError("map capture disk reserve is below the 6/5 GiB start/run contract")
    if owner is not None and (tree_bytes(owner) > MAX_OWNER_BYTES or tree_bytes(CAMPAIGN) > MAX_CAMPAIGN_BYTES):
        raise ValueError("map capture exceeded the 400 MiB owner or 1 GiB campaign cap")


def check_no_other_carla(port: int) -> None:
    """HH_261001 - Reject any live CARLA world and all three requested ports."""
    for process in Path("/proc").iterdir():
        if not process.name.isdigit():
            continue
        try:
            argv = (process / "cmdline").read_bytes().split(b"\0")
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        if is_carla_process_argv(argv):
            raise ValueError("another CARLA simulator is already running")
    for requested in range(port, port + 3):
        with socket.socket() as probe:
            try:
                probe.bind(("127.0.0.1", requested))
            except OSError as error:
                raise ValueError(f"CARLA port {requested} is already occupied") from error


def is_carla_process_argv(argv: list[bytes]) -> bool:
    """HH_261001 - Include startup wrappers before the engine binary exists."""
    if not argv or not argv[0]:
        return False
    executable = Path(os.fsdecode(argv[0])).name
    if executable in ("UE4Editor", "CarlaUE4-Linux-Shipping", "CarlaUE4.sh"):
        return True
    return executable in ("bash", "sh") and any(
        Path(os.fsdecode(argument)).name in ("run_carla_map.sh", "CarlaUE4.sh")
        for argument in argv[1:3] if argument
    )


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("output_root", type=Path)
    parser.add_argument("route_file", type=Path)
    parser.add_argument("--port", type=int, default=2100)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args(argv)
    args.output_root = args.output_root.absolute()
    args.route_file = args.route_file.absolute()
    if not 1024 <= args.port <= 65533:
        parser.error("port and its two successors must be valid")
    configured_carla = os.environ.get("CARLA_ROOT")
    if configured_carla and Path(configured_carla).resolve() != capture.base.PACKAGE:
        parser.error("only the pinned packaged CARLA_ROOT is permitted")
    route, route_sha = capture.checked_route(args.route_file)
    args.arm = route["scenario"]
    args.route_sha256 = route_sha
    args.output_name = "p02" if args.arm == "right" else "p04"
    if args.output_root != CAMPAIGN / args.output_name or args.output_root.exists() or args.output_root.is_symlink():
        parser.error("output must be the fresh canonical p02 or p04 directory")
    capture.parse_args([
        str(args.output_root / "mapping"), str(args.route_file), "--host", "127.0.0.1",
        "--port", str(args.port), "--frames", "240",
    ])
    check_storage()
    if args.output_name == "p04":
        prior = CAMPAIGN / "qa/p02.json"
        if not prior.is_file() or prior.is_symlink():
            parser.error("p02 independent QA must finish before p04")
        report = json.loads(prior.read_text())
        if (report.get("status") != "QA_CONDITIONAL" or report.get("dataset_admitted") is not False
                or report.get("route_sha256") != next(iter(capture.ROUTES))):
            parser.error("p02 QA receipt is not scoped and conditional")
    return args


def archive_sources(owner: Path, route_file: Path) -> tuple[dict, dict, dict]:
    """HH_261001 - Preserve the complete Python/shell route and agent source closure."""
    source_hashes = {}
    for logical in SOURCES:
        source = ROOT / logical
        if not source.is_file() or source.is_symlink():
            raise ValueError(f"source is not a regular file: {logical}")
        target = owner / "provenance" / logical
        target.parent.mkdir(parents=True, exist_ok=True)
        payload = source.read_bytes()
        with target.open("xb") as stream:
            stream.write(payload)
        source_hashes[logical] = hashlib.sha256(payload).hexdigest()
    external = {}
    agent_root = capture.base.PACKAGE / "PythonAPI/carla"
    for source in sorted((agent_root / "agents").rglob("*.py")):
        if source.is_symlink():
            raise ValueError("CARLA agent source is a symlink")
        relative = source.relative_to(agent_root)
        target = owner / "provenance/carla_pythonapi" / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        payload = source.read_bytes()
        with target.open("xb") as stream:
            stream.write(payload)
        external[str(relative)] = {
            "path": str(source), "archived_path": str(target.relative_to(owner)),
            "sha256": hashlib.sha256(payload).hexdigest(),
        }
    route_payload = route_file.read_bytes()
    route_archive = owner / "provenance/route.json"
    with route_archive.open("xb") as stream:
        stream.write(route_payload)
    route_ref = {"path": str(route_file), "archived_path": str(route_archive.relative_to(owner)),
                 "sha256": hashlib.sha256(route_payload).hexdigest()}
    return source_hashes, external, route_ref


def source_checks(owner: Path, plan: dict) -> tuple[dict, dict, bool]:
    checks, external_checks = {}, {}
    for logical, sha in plan["source_sha256"].items():
        checks[logical] = all(
            path.is_file() and not path.is_symlink() and capture.base.digest(path) == sha
            for path in (ROOT / logical, owner / "provenance" / logical)
        )
    for logical, ref in plan["external_source_provenance"].items():
        external_checks[logical] = all(
            path.is_file() and not path.is_symlink() and capture.base.digest(path) == ref["sha256"]
            for path in (Path(ref["path"]), owner / ref["archived_path"])
        )
    route = plan["route_archive"]
    route_ok = all(
        path.is_file() and not path.is_symlink() and capture.base.digest(path) == route["sha256"]
        for path in (Path(route["path"]), owner / route["archived_path"])
    )
    return checks, external_checks, route_ok


def probe(owner: Path, stage: str, server_pid: int, port: int, stopped=False) -> bool:
    command = [sys.executable, str(ROOT / "tools/workspace_python.py"),
        str(ROOT / "tools/e2e/probe_carla_server.py"), "--host", "127.0.0.1",
        "--port", str(port), "--timeout", "4", "--expected-map", "Town01",
        "--owner-pid", str(server_pid), "--owner-pgid", str(server_pid),
        "--generation-id", f"paired_{server_pid}", "--stage", stage,
        "--server-log", str(owner / "server.log"),
        "--output", str(owner / "lifecycle" / f"{stage}.json")]
    if stopped:
        command.append("--expect-stopped")
    return subprocess.run(command, cwd=ROOT, check=False, timeout=30).returncode == 0


def process_record(pid: int) -> tuple[str, int, int] | None:
    """HH_261001 - Read the process state, group and immutable kernel start tick."""
    try:
        stat = Path(f"/proc/{pid}/stat").read_text()
    except (FileNotFoundError, ProcessLookupError, PermissionError):
        return None
    fields = stat[stat.rfind(")") + 2:].split()
    if len(fields) < 20:
        raise ValueError("invalid process stat record")
    return fields[0], int(fields[2]), int(fields[19])


def mark_owned(process: subprocess.Popen) -> subprocess.Popen:
    """HH_261001 - Pin a new session before any later cleanup signal."""
    record = process_record(process.pid)
    if record is None or record[1] != process.pid:
        raise RuntimeError("launched process has no isolated owner group")
    process.owned_start_ticks = record[2]
    return process


def exited_without_reap(process: subprocess.Popen) -> bool:
    """HH_261001 - Preserve the leader PID until cleanup verifies its generation."""
    result = os.waitid(os.P_PID, process.pid, os.WEXITED | os.WNOHANG | os.WNOWAIT)
    return result is not None


def live_group_members(pgid: int) -> list[int]:
    members = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        record = process_record(int(entry.name))
        if record is not None and record[1] == pgid and record[0] != "Z":
            members.append(int(entry.name))
    return members


def stop_group(process: subprocess.Popen | None) -> bool:
    """HH_261001 - Signal only a still-pinned owner process generation."""
    if process is None:
        return True
    record = process_record(process.pid)
    if getattr(process, "owned_start_ticks", None) is None:
        # HH_261001 - A failed identity pin permits only the unreaped direct child PID to be stopped.
        if record is not None:
            process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            return False
        return False
    if (process.pid <= 1 or process.pid == os.getpgrp() or record is None
            or record[1] != process.pid
            or record[2] != getattr(process, "owned_start_ticks", None)):
        return False
    for sig, timeout in ((signal.SIGINT, 20), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
        if not live_group_members(process.pid):
            break
        current = process_record(process.pid)
        if current is None or current[1:] != record[1:]:
            return False
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        deadline = time.monotonic() + timeout
        while live_group_members(process.pid) and time.monotonic() < deadline:
            time.sleep(0.2)
    if live_group_members(process.pid):
        return False
    try:
        process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        return False
    return True


def wait_ready(process: subprocess.Popen, log: Path) -> None:
    deadline = time.monotonic() + 195
    while time.monotonic() < deadline:
        if log.is_file() and b"CARLA_READY " in log.read_bytes():
            return
        if exited_without_reap(process):
            break
        time.sleep(1)
    raise RuntimeError("owned Town01 CARLA did not become ready")


def monitor_capture(process: subprocess.Popen, owner: Path) -> None:
    deadline = time.monotonic() + 300
    while not exited_without_reap(process):
        check_storage(owner)
        if time.monotonic() > deadline:
            raise TimeoutError("paired collector exceeded 300 seconds")
        time.sleep(0.5)
    check_storage(owner)


def execute(args) -> int:
    lock_path = ROOT / "storage/assets/locks/autoware_e2e_runtime.lock"
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open("a+b") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError("another workspace owner holds the runtime lock") from error
        check_no_other_carla(args.port)
        check_storage()
        owner = args.output_root
        owner.parent.mkdir(parents=True, exist_ok=True)
        owner.mkdir()
        (owner / "lifecycle").mkdir()
        server = collector = None
        error = None
        server_cleanup = collector_cleanup = True
        after_ok = stopped_ok = False
        try:
            source_sha, external, route_ref = archive_sources(owner, args.route_file)
            plan = {
                "schema": "portable_e2e.map_paired_town01_p02_p04_owner.v1",
                "planned_at_utc": utc_now(), "capture_mode": CAPTURE_MODE,
                "route_path": str(args.route_file), "route_sha256": args.route_sha256,
                "route_archive": route_ref, "map": "Town01", "host": "127.0.0.1",
                "port": args.port, "quality": "Epic", "frames": 240,
                "worker_path": "tools/e2e/collect_map_paired_town01_p02_p04_20261001.py",
                "collector_argv": [str(owner / "mapping"), str(args.route_file),
                                   "--host", "127.0.0.1", "--port", str(args.port), "--frames", "240"],
                "source_sha256": source_sha, "source_bytes_archived": True,
                "external_source_provenance": external, "external_source_bytes_archived": True,
                "source_assets": capture.base.package_assets("Town01"),
                "intended_role": capture.ROLE, "dataset_admitted": False,
                "train_eligible": False, "ctrack_train_eligible": False,
                "independent_dev_approved": False, "qa_status": "PENDING_QA",
                "owner_cap_bytes": MAX_OWNER_BYTES, "campaign_cap_bytes": MAX_CAMPAIGN_BYTES,
                "minimum_running_free_bytes": MIN_RUNNING_FREE,
            }
            write_json(owner / "owner_plan.json", plan)
            check_storage(owner)
            with (owner / "server.log").open("xb") as stream:
                server = subprocess.Popen([
                    "bash", str(ROOT / "tools/e2e/run_carla_map.sh"), "Town01",
                    "--port", str(args.port), "--quality", "Epic",
                    "--startup-timeout-sec", "180", "--", "-RenderOffScreen", "-nosound",
                ], cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
                mark_owned(server)
            wait_ready(server, owner / "server.log")
            if not probe(owner, "ready", server.pid, args.port):
                raise RuntimeError("owned CARLA ready probe failed")
            write_json(owner / "owner_started.json", {
                "started_at_utc": utc_now(), "server_pid": server.pid,
                "server_pgid": server.pid, "route_sha256": args.route_sha256,
                "map": "Town01", "port": args.port, "quality": "Epic",
                "learned_model_control": False,
            })
            check_storage(owner)
            checks, external_checks, route_ok = source_checks(owner, plan)
            if not all(checks.values()) or not all(external_checks.values()) or not route_ok:
                raise RuntimeError("source or route bytes changed before collector launch")
            with (owner / "collector.log").open("xb") as stream:
                collector = subprocess.Popen([
                    sys.executable, str(ROOT / "tools/workspace_python.py"), "-m",
                    "scripts.e2e.collect_map_paired_town01_p02_p04_20261001", "--",
                    *plan["collector_argv"],
                ], cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
                mark_owned(collector)
            monitor_capture(collector, owner)
            collector_cleanup = stop_group(collector)
            collector_code = collector.returncode
            collector = None
            if collector_code != 0:
                raise RuntimeError(f"paired collector exited {collector_code}")
            after_ok = probe(owner, "after_capture", server.pid, args.port)
            if not after_ok:
                raise RuntimeError("owned CARLA after-capture probe failed")
        except BaseException as failure:
            error = repr(failure)
        finally:
            collector_cleanup = stop_group(collector) and collector_cleanup
            server_pid = server.pid if server is not None else None
            server_cleanup = stop_group(server)
            if server_pid is not None:
                stopped_ok = probe(owner, "stopped", server_pid, args.port, stopped=True)
            plan_path = owner / "owner_plan.json"
            checks, external_checks, route_ok = ({}, {}, False)
            if plan_path.is_file():
                try:
                    checks, external_checks, route_ok = source_checks(owner, json.loads(plan_path.read_text()))
                except Exception as failure:
                    error = f"{error}; source closure: {failure!r}"
            success = (error is None and collector_cleanup and server_cleanup and after_ok and stopped_ok
                       and route_ok and all(checks.values()) and all(external_checks.values()))
            try:
                check_storage(owner)
            except Exception as failure:
                success = False
                error = f"{error}; storage: {failure!r}"
            write_json(owner / "owner_result.json", {
                "completed_at_utc": utc_now(), "exit_code": 0 if success else 1,
                "capture_mode": CAPTURE_MODE, "source_bytes_unchanged_and_archived":
                    route_ok and all(checks.values()) and all(external_checks.values()),
                "source_checks": checks, "external_source_checks": external_checks,
                "route_archive_unchanged": route_ok, "collector_cleanup_ok": collector_cleanup,
                "server_cleanup_ok": server_cleanup, "after_capture_ok": after_ok,
                "stopped_ok": stopped_ok, "error": error,
                "intended_role": capture.ROLE, "dataset_admitted": False,
                "train_eligible": False, "ctrack_train_eligible": False,
                "independent_dev_approved": False, "qa_status": "PENDING_QA",
                "learned_model_control": False, "vehicle_control_approved": False,
            })
        return 0 if success else 1


def main(argv=None):
    args = parse_args(argv)
    if not args.execute:
        print(f"TOWN01_P02_P04_PREFLIGHT_PASS route={args.output_name} port={args.port} "
              "frames=240 no_simulator_launched=true")
        return 0
    return execute(args)


if __name__ == "__main__":
    raise SystemExit(main())
