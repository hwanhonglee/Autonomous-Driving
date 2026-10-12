#!/usr/bin/env python3
"""Read-only CARLA RPC and owned-generation health probe.

The probe never loads a world or advances simulation time.  A running check
uses a fresh client for ``get_world()``, exact map identity, and
``get_snapshot()``.  A stopped check proves that the owned process disappeared
and that the configured TCP port was released.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import socket
import tempfile
import time
from typing import Any, Sequence


SAFE_TOKEN = re.compile(r"[A-Za-z0-9_.-]+")


class ProbeError(RuntimeError):
    pass


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, staged_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".staged", dir=path.parent
    )
    staged = Path(staged_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(staged, path)
    finally:
        staged.unlink(missing_ok=True)


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _process_state(pid: int) -> str | None:
    try:
        value = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
    except FileNotFoundError:
        return None
    except OSError as error:
        raise ProbeError(f"cannot inspect owned process {pid}: {error}") from error
    close = value.rfind(")")
    if close < 0:
        raise ProbeError(f"owned process {pid} has an invalid /proc stat record")
    fields = value[close + 2 :].split()
    if not fields:
        raise ProbeError(f"owned process {pid} has an empty /proc stat record")
    return fields[0]


def _require_live_owner(pid: int, pgid: int) -> str:
    state = _process_state(pid)
    if state is None:
        raise ProbeError(f"owned CARLA process {pid} is absent")
    if state == "Z":
        raise ProbeError(f"owned CARLA process {pid} is a zombie")
    try:
        actual_pgid = os.getpgid(pid)
    except ProcessLookupError as error:
        raise ProbeError(f"owned CARLA process {pid} disappeared") from error
    if actual_pgid != pgid:
        raise ProbeError(
            f"owned CARLA process-group mismatch: expected={pgid} actual={actual_pgid}"
        )
    return state


def _normalize_map_name(value: str) -> str:
    token = str(value).rstrip("/").rsplit("/", 1)[-1]
    if not token or SAFE_TOKEN.fullmatch(token) is None:
        raise ProbeError(f"unsafe CARLA map identity: {value!r}")
    return token


def _server_log_record(value: str | None) -> dict[str, Any] | None:
    if value is None:
        return None
    path = Path(value).expanduser().resolve()
    if not path.is_file():
        raise ProbeError(f"CARLA generation log is missing: {path}")
    return {
        "path": str(path),
        "size_bytes": path.stat().st_size,
        "sha256": _sha256_file(path),
    }


def _running_probe(args: argparse.Namespace) -> dict[str, Any]:
    owner_state = (
        _require_live_owner(args.owner_pid, args.owner_pgid)
        if args.owner_pid is not None
        else None
    )
    try:
        import carla
    except Exception as error:  # pragma: no cover - depends on packaged CARLA
        raise ProbeError(f"cannot import packaged CARLA Python API: {error}") from error

    expected = _normalize_map_name(args.expected_map)
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    try:
        world = client.get_world()
        active_map = str(world.get_map().name)
        actual = _normalize_map_name(active_map)
        if actual != expected:
            raise ProbeError(
                f"CARLA map mismatch: expected={expected!r} actual={actual!r}"
            )
        snapshot = world.get_snapshot()
        frame = int(snapshot.frame)
        timestamp = snapshot.timestamp
        elapsed_seconds = float(timestamp.elapsed_seconds)
    except ProbeError:
        raise
    except Exception as error:
        raise ProbeError(f"CARLA read-only RPC failed: {error}") from error
    if frame < 0 or not elapsed_seconds >= 0.0:
        raise ProbeError("CARLA snapshot returned invalid frame/time values")
    return {
        "owner_process_state": owner_state,
        "active_map_name": active_map,
        "active_map_basename": actual,
        "snapshot_frame": frame,
        "snapshot_elapsed_seconds": elapsed_seconds,
        "rpc_sequence": ["get_world", "world.get_map", "world.get_snapshot"],
        "read_only": True,
    }


def _stopped_probe(args: argparse.Namespace) -> dict[str, Any]:
    if args.owner_pid is None or args.owner_pgid is None:
        raise ProbeError("stopped probe requires an owned PID and PGID")
    state = _process_state(args.owner_pid)
    if state is not None:
        raise ProbeError(
            f"owned CARLA process {args.owner_pid} remains alive with state {state}"
        )
    deadline = time.monotonic() + args.timeout
    port_released = False
    while time.monotonic() < deadline:
        try:
            connection = socket.create_connection(
                (args.host, args.port), timeout=min(0.5, args.timeout)
            )
        except OSError:
            port_released = True
            break
        else:
            connection.close()
        time.sleep(0.1)
    if not port_released:
        raise ProbeError(f"CARLA port {args.host}:{args.port} is still accepting TCP")
    return {
        "owner_process_state": state,
        "port_released": True,
        "rpc_sequence": [],
        "read_only": True,
    }


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--expected-map", required=True)
    parser.add_argument("--stage", required=True)
    parser.add_argument("--generation-id", required=True)
    parser.add_argument("--owner-pid", type=int)
    parser.add_argument("--owner-pgid", type=int)
    parser.add_argument("--server-log")
    parser.add_argument("--expect-stopped", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if not 0 < args.port <= 65535:
        parser.error("--port must be in [1, 65535]")
    if args.timeout <= 0.0:
        parser.error("--timeout must be positive")
    if (args.owner_pid is None) != (args.owner_pgid is None):
        parser.error("--owner-pid and --owner-pgid must be supplied together")
    if args.owner_pid is not None and (
        args.owner_pid <= 1 or args.owner_pgid <= 1
    ):
        parser.error("owned PID/PGID must be greater than one")
    if args.expect_stopped and args.owner_pid is None:
        parser.error("--expect-stopped requires owned PID/PGID")
    for label, value in (
        ("stage", args.stage),
        ("generation id", args.generation_id),
    ):
        if SAFE_TOKEN.fullmatch(value) is None:
            parser.error(f"unsafe {label}: {value!r}")
    try:
        _normalize_map_name(args.expected_map)
    except ProbeError as error:
        parser.error(str(error))
    return args


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    started = time.monotonic()
    payload: dict[str, Any] = {
        "schema_version": 1,
        "status": "FAIL",
        "mode": "stopped" if args.expect_stopped else "running",
        "stage": args.stage,
        "generation_id": args.generation_id,
        "host": args.host,
        "port": args.port,
        "expected_map": _normalize_map_name(args.expected_map),
        "owner_pid": args.owner_pid,
        "owner_pgid": args.owner_pgid,
        "checked_at": _utc_now(),
        "timeout_seconds": args.timeout,
        "error": None,
    }
    status = 0
    try:
        details = (
            _stopped_probe(args) if args.expect_stopped else _running_probe(args)
        )
        payload.update(details)
        payload["server_log"] = _server_log_record(args.server_log)
        payload["status"] = "PASS"
    except Exception as error:
        payload["error"] = str(error)
        try:
            payload["server_log"] = _server_log_record(args.server_log)
        except Exception as log_error:
            payload["server_log"] = None
            payload["error"] += f"; server log check failed: {log_error}"
        status = 1
    payload["probe_wall_seconds"] = time.monotonic() - started
    _atomic_json(args.output.expanduser().resolve(), payload)
    if status:
        print(payload["error"], file=os.sys.stderr)
    else:
        print(
            f"CARLA_HEALTH_PASS generation={args.generation_id} "
            f"stage={args.stage} mode={payload['mode']}"
        )
    return status


if __name__ == "__main__":
    raise SystemExit(main())
