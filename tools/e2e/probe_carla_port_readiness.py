#!/usr/bin/env python3
"""HH_260906 - Observe local prelaunch ports without reserving them or retrying a CARLA case."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import socket
import subprocess
from time import monotonic, sleep

HOST = "127.0.0.1"
DEADLINE_SECONDS = 60.0
INTERVAL_SECONDS = 2.0
REQUIRED_CLEAR_CHECKS = 3
SS_TIMEOUT_SECONDS = 2.0
MAX_SS_OUTPUT_BYTES = 8192


def require(condition, message):
    if not condition:
        raise ValueError(message)


def checked_path(path):
    path = Path(path).absolute()
    require(".." not in path.parts and not any(p.is_symlink() for p in (path, *path.parents)),
            "parent traversal and symlink output paths are forbidden")
    return path


def valid_port(port):
    require(type(port) is int and 1024 <= port <= 65533, "port must be an integer in 1024..65533")
    return port


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def source_sha():
    return hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def bind_probe(port):
    """HH_260906 - Hold all successful bindings until this six-socket observation ends, then release every socket."""
    valid_port(port)
    results, owned = [], []
    try:
        for current in range(port, port + 3):
            for protocol, kind in (("TCP", socket.SOCK_STREAM), ("UDP", socket.SOCK_DGRAM)):
                record = dict(host=HOST, port=current, protocol=protocol, status="BLOCKED")
                try:
                    probe = socket.socket(socket.AF_INET, kind)
                    owned.append(probe)
                    # HH_260906 - No SO_REUSEADDR/SO_REUSEPORT, connect, listen, or alternative port selection.
                    probe.bind((HOST, current))
                    record["status"] = "CLEAR"
                except OSError as error:
                    record.update(errno=error.errno, error=str(error)[:512])
                results.append(record)
    finally:
        for probe in owned:
            probe.close()
    return results


def ss_snapshot(port, protocol, deadline):
    """HH_260906 - Capture bounded local socket diagnostics only; timeout applies only to this owned ss child."""
    require(protocol in ("TCP", "UDP"), "unsupported socket protocol")
    valid_port(port)
    remaining = deadline - monotonic()
    # HH_260906 - All TCP states include transient TIME_WAIT; either endpoint may name an owned port.
    fields = ("sport", "dport") if protocol == "TCP" else ("sport",)
    command = ["ss", "-H", "-antp" if protocol == "TCP" else "-aunp",
               "( " + " or ".join(f"{field} = :{p}" for p in range(port, port + 3) for field in fields) + " )"]
    result = dict(protocol=protocol, command=command, timeout_seconds=max(0., min(SS_TIMEOUT_SECONDS, remaining)))
    if remaining <= 0:
        return dict(result, status="SKIPPED_DEADLINE", output="", output_truncated=False)
    try:
        child = subprocess.run(command, capture_output=True, timeout=result["timeout_seconds"], check=False)
        payload = child.stdout + child.stderr
        result.update(status="COMPLETE" if child.returncode == 0 else "COMMAND_FAILED", returncode=child.returncode)
    except subprocess.TimeoutExpired as error:
        payload = (error.stdout or b"") + (error.stderr or b"")
        result.update(status="TIMEOUT", returncode=None)
    except OSError as error:
        payload = str(error).encode()
        result.update(status="UNAVAILABLE", returncode=None)
    decoded = payload[:MAX_SS_OUTPUT_BYTES].decode("utf-8", errors="replace")
    encoded = decoded.encode("utf-8")
    result.update(output=encoded[:MAX_SS_OUTPUT_BYTES].decode("utf-8", errors="ignore"),
                  output_truncated=len(payload) > MAX_SS_OUTPUT_BYTES or len(encoded) > MAX_SS_OUTPUT_BYTES)
    return result


def run(port, output):
    valid_port(port)
    output = checked_path(output)
    journal_path = checked_path(output.with_name(output.name + ".samples.jsonl"))
    require(not output.exists() and not journal_path.exists(), "output and sample journal must both be fresh")
    code_before = source_sha()
    output.parent.mkdir(parents=True, exist_ok=True)
    observations, consecutive, status, failure = [], 0, "NOT_READY_WITHIN_OBSERVATION_BUDGET", None
    started_utc, started = utc_now(), monotonic()
    deadline, next_due = started + DEADLINE_SECONDS, started
    with journal_path.open("x", encoding="utf-8") as journal:
        def persist(record):
            journal.write(json.dumps(record, sort_keys=True, allow_nan=False) + "\n")
            journal.flush()
        persist(dict(kind="started", started_at_utc=started_utc, source_sha256=code_before,
                     host=HOST, ports=list(range(port, port+3)), observation_budget_seconds=DEADLINE_SECONDS))
        try:
            while monotonic() < deadline:
                delay = min(next_due - monotonic(), deadline - monotonic(), INTERVAL_SECONDS)
                if delay > 0:
                    sleep(delay)
                    continue
                if monotonic() >= deadline:
                    break
                observed_at = monotonic()
                bindings = bind_probe(port)
                clear = len(bindings) == 6 and all(row["status"] == "CLEAR" for row in bindings)
                consecutive = consecutive + 1 if clear else 0
                sample = dict(index=len(observations), observed_at_utc=utc_now(), elapsed_seconds=observed_at-started,
                              bindings=bindings, all_six_clear=clear, consecutive_clear_checks=consecutive, ss=[])
                observations.append(sample)
                # HH_260906 - Persist the bind outcome before a diagnostic child can time out or be interrupted.
                persist(dict(kind="bind_sample", **sample))
                if not clear:
                    for protocol in ("TCP", "UDP"):
                        snapshot = ss_snapshot(port, protocol, deadline)
                        sample["ss"].append(snapshot)
                        persist(dict(kind="ss_snapshot", sample_index=sample["index"], **snapshot))
                if consecutive >= REQUIRED_CLEAR_CHECKS and monotonic() <= deadline:
                    status = "READY_OBSERVED_NOT_RESERVED"
                    break
                next_due = observed_at + INTERVAL_SECONDS
        except (KeyboardInterrupt, OSError, ValueError) as error:
            status, failure = "OBSERVATION_FAILED_NOT_READY", f"{type(error).__name__}: {error}"
        code_after = source_sha()
        if code_after != code_before:
            status, failure = "OBSERVATION_FAILED_NOT_READY", "probe source changed during observation"
        report = dict(schema="carla.local_port_readiness.v1", status=status,
            ready_observed=status == "READY_OBSERVED_NOT_RESERVED", error=failure,
            started_at_utc=started_utc, completed_at_utc=utc_now(), elapsed_seconds=monotonic()-started,
            bind_host=HOST, ports=list(range(port, port+3)), protocols=["TCP", "UDP"],
            observation_budget_seconds=DEADLINE_SECONDS, minimum_sample_spacing_seconds=INTERVAL_SECONDS,
            required_consecutive_clear_checks=REQUIRED_CLEAR_CHECKS, final_consecutive_clear_checks=consecutive,
            samples=observations, sample_journal=journal_path.name,
            source_sha256_before=code_before, source_sha256_after=code_after,
            simulator_contacted=False, simulator_launched=False, wrapper_invoked=False,
            port_reserved_after_probe=False, reuse_socket_options_used=False, automatic_trial_retry=False,
            foreign_processes_signaled=False, installations_performed=False,
            interpretation="Prelaunch observations only, not a reservation or future availability guarantee. The owned wrapper remains authoritative; a wrapper failure consumes that case attempt and is never retried by this probe.")
        persist(dict(kind="finished", status=status, elapsed_seconds=report["elapsed_seconds"], error=failure))
    report["sample_journal_sha256"] = hashlib.sha256(journal_path.read_bytes()).hexdigest()
    with output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write("\n")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--port", required=True, type=int)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        report = run(args.port, args.output)
    except (OSError, ValueError) as error:
        parser.exit(2, f"port readiness probe refused: {error}\n")
    print(json.dumps({key:report[key] for key in ("status", "ready_observed", "elapsed_seconds")}))
    return 0 if report["ready_observed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
