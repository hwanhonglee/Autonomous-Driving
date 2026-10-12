#!/usr/bin/env python3
"""HH_261001 - Observe AH PID-generation/PGID extinction and the original exclusive port-bind check."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

from scripts.e2e import run_owned_ah_town07_research_20261001 as original_owner


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha(path):
    require(path.is_file() and not path.is_symlink(), f"regular file required: {path}")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def process(pid):
    try:
        fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        return {"pid": pid, "state": fields[0], "ppid": int(fields[1]), "pgid": int(fields[2]), "start_ticks": int(fields[19])}
    except (FileNotFoundError, ProcessLookupError):
        return None


def socket_rows(ports):
    rows = []
    for protocol in ("tcp", "tcp6", "udp", "udp6"):
        for line in (Path("/proc/net") / protocol).read_text().splitlines()[1:]:
            fields = line.split()
            number = int(fields[1].rsplit(":", 1)[1], 16)
            if number in ports:
                rows.append({"protocol": protocol, "port": number, "local_address_hex": fields[1],
                    "remote_address_hex": fields[2], "state_hex": fields[3], "inode": fields[9],
                    "is_tcp_listener": protocol.startswith("tcp") and fields[3] == "0A",
                    "is_tcp_time_wait": protocol.startswith("tcp") and fields[3] == "06",
                    "is_udp_bound": protocol.startswith("udp")})
    return rows


def observe(started, ports):
    identities = {}
    for role in ("server", "socket_owner"):
        expected = started[role]
        current = process(expected["pid"])
        identities[role] = {"expected": expected, "observed": current,
            "original_generation_absent": current is None or current["start_ticks"] != expected["start_ticks"],
            "numeric_pid_reused": current is not None and current["start_ticks"] != expected["start_ticks"]}
    pgid = int(started["server"]["pgid"])
    members = []
    for entry in Path("/proc").iterdir():
        if entry.name.isdigit():
            current = process(int(entry.name))
            if current and current["pgid"] == pgid:
                members.append(current)
    sockets = socket_rows(ports)
    try:
        original_owner.ports_free(ports[0])
        bind_pass, bind_error = True, None
    except OSError as error:
        bind_pass, bind_error = False, {"errno": error.errno, "message": str(error)}
    listeners_or_udp = any(row["is_tcp_listener"] or row["is_udp_bound"] for row in sockets)
    passed = all(row["original_generation_absent"] for row in identities.values()) and not members and not listeners_or_udp and bind_pass
    return {"checked_at_utc": datetime.now(timezone.utc).isoformat(), "status": "PASS" if passed else "PENDING_RELEASE",
        "original_pid_generations": identities, "remaining_pgid_members": members, "socket_rows": sockets,
        "tcp_listener_or_udp_binding_present": listeners_or_udp,
        "tcp_time_wait_present": any(row["is_tcp_time_wait"] for row in sockets),
        "original_ports_free_exclusive_bind_pass": bind_pass, "exclusive_bind_error": bind_error}


def run(trial, output, max_observations=13, interval_seconds=5.):
    trial, output = trial.resolve(strict=True), output.absolute()
    require(not output.exists() and not output.is_symlink() and not output.resolve().is_relative_to(trial), "fresh receipt directory outside original trial required")
    require(1 <= max_observations <= 13 and 0 < interval_seconds <= 5, "bounded release observation budget exceeded")
    paths = {name: trial / name for name in ("owner_plan.json", "owner_started.json", "owner_result.json")}
    hashes = {name: sha(path) for name, path in paths.items()}
    plan, started, owner = (json.loads(paths[name].read_text()) for name in paths)
    ports = plan["ports"]
    require(ports == [2440, 2441, 2442] and started["server"]["pgid"] == started["socket_owner"]["pgid"], "unsupported port triplet or owner process group")
    require(started["server"]["pid"] == started["server"]["pgid"] > 1, "server is not its own owned group leader")
    source_checks = {}
    for name, expected in plan["source_sha256"].items():
        live, archived = original_owner.ROOT / name, trial / "source" / name
        source_checks[name] = {"expected_sha256": expected, "current_sha256": sha(live), "archived_sha256": sha(archived)}
        require(source_checks[name]["current_sha256"] == source_checks[name]["archived_sha256"] == expected,
                f"execution source/archive differs from plan: {name}")
    pin_checks = {name: sha(Path(name)) == expected for name, expected in plan["source_pins"].items()}
    require(all(pin_checks.values()), "declared source input pin changed")
    observations = []
    for index in range(max_observations):
        observation = observe(started, ports)
        observations.append(observation)
        if observation["status"] == "PASS":
            break
        if index + 1 < max_observations:
            time.sleep(interval_seconds)
    for name, path in paths.items():
        require(sha(path) == hashes[name], "original owner receipt changed during observation")
    for name, expected in plan["source_sha256"].items():
        require(sha(original_owner.ROOT / name) == sha(trial / "source" / name) == expected, "source changed during observation")
    report = {"schema": "ah.post_owner_release_observation.20261001.v1", "status": observations[-1]["status"],
        "trial": str(trial), "original_owner_status": owner["status"], "original_cleanup_errors": owner["cleanup_errors"],
        "original_ports_released_flag": owner["ports_released"], "original_owner_result_rewritten": False,
        "original_receipt_sha256": hashes, "observations": observations, "source_checks": source_checks,
        "source_input_pin_checks": pin_checks, "script_sha256": sha(Path(__file__)),
        "processes_signalled": False, "simulator_contacted": False, "training_or_driving_success_claimed": False,
        "interpretation": "A later release receipt does not erase the earlier owner failure. TCP TIME-WAIT is separate from a process/listener; release requires the unchanged original exclusive IPv4 TCP+UDP bind check to pass for every original port."}
    output.mkdir(parents=True, exist_ok=False)
    with (output / "release_observation.json").open("x") as stream:
        json.dump(report, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--trial", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    report = run(args.trial, args.output)
    print(json.dumps({"status": report["status"], "observations": len(report["observations"]),
        "source_files_verified": len(report["source_checks"]), "input_pins_verified": len(report["source_input_pin_checks"]),
        "original_owner_status_preserved": report["original_owner_status"],
        "exclusive_bind_pass": report["observations"][-1]["original_ports_free_exclusive_bind_pass"]}, sort_keys=True))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
