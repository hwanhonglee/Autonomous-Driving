#!/usr/bin/env python3
"""HH_261001 - Declare or own one source-pinned native-only Town07 low-speed pilot."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys

from scripts.e2e import run_owned_ah_town07_research_20261001 as owner
from scripts.e2e import town07_low_speed_10_30_pilot_contract_20261001 as protocol


def command(plan_sha256: str) -> list[str]:
    return [sys.executable, str(protocol.ROOT / "tools/workspace_python.py"), "-m",
        "scripts.e2e.collect_town07_low_speed_10_30_pilot_20261001", "--",
        "--plan-sha256", plan_sha256, "--output", str(protocol.RUN / "episode")]


def archive_declared(run: Path, saved: dict, plan: dict):
    """HH_261001 - Archive declared local and CARLA agent bytes before any server is launched."""
    saved.setdefault("source_sha256", {})
    for name, expected in plan["source_sha256"].items():
        source, target = protocol.ROOT / name, run / "source" / name
        target.parent.mkdir(parents=True, exist_ok=True)
        if not target.exists():
            with source.open("rb") as read, target.open("xb") as write:
                shutil.copyfileobj(read, write)
        if protocol.sha(target) != expected or protocol.sha(source) != expected:
            raise ValueError("declared local source archive differs: " + name)
        saved["source_sha256"][name] = expected
    for name, expected in plan["external_agent_sha256"].items():
        source = Path(name)
        relative = source.relative_to(protocol.CARLA_ROOT)
        target = run / "source_external" / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        with source.open("rb") as read, target.open("xb") as write:
            shutil.copyfileobj(read, write)
        if protocol.sha(target) != expected or protocol.sha(source) != expected:
            raise ValueError("declared CARLA agent archive differs: " + name)
    with protocol.PLAN.open("rb") as read, (run / "declared_plan.json").open("xb") as write:
        shutil.copyfileobj(read, write)
    saved.update(declared_plan_sha256=protocol.sha(protocol.PLAN),
        external_agent_sha256=dict(plan["external_agent_sha256"]),
        source_archive_before_server=True, training_data_approved=False)


@contextmanager
def scoped_owner(plan: dict, plan_sha256: str):
    saved = (owner.OUTPUT, owner.PINS, owner.collector_command,
             owner.write_json, owner.lifecycle._workspace_lock)
    original_write, original_lock = owner.write_json, owner.lifecycle._workspace_lock
    pins = {path: digest for path, digest in owner.PINS.items()
            if path not in (owner.WEIGHTS, owner.METADATA)}
    pins.update({protocol.ROOT / name: digest for name, digest in plan["source_sha256"].items()})
    pins.update({Path(name): digest for name, digest in plan["external_agent_sha256"].items()})
    pins[protocol.PLAN] = plan_sha256

    def archive_write(path, value):
        if path == protocol.RUN / "owner_plan.json":
            archive_declared(protocol.RUN, value, plan)
        original_write(path, value)

    @contextmanager
    def reserved_lock():
        with original_lock():
            protocol.verify_plan(plan, plan_sha256)
            if protocol.free_bytes() < protocol.FLOOR_BYTES + protocol.STARTUP_RESERVE_BYTES:
                raise ValueError("native pilot lacks fixed startup disk reserve")
            yield

    owner.OUTPUT, owner.PINS, owner.collector_command = protocol.RUN, pins, lambda: command(plan_sha256)
    owner.write_json, owner.lifecycle._workspace_lock = archive_write, reserved_lock
    try:
        yield
    finally:
        (owner.OUTPUT, owner.PINS, owner.collector_command,
         owner.write_json, owner.lifecycle._workspace_lock) = saved


def preflight(plan: dict, plan_sha256: str) -> dict:
    if sys.version_info[:2] != (3, 10):
        raise ValueError("packaged CARLA API requires Python 3.10")
    protocol.verify_plan(plan, plan_sha256)
    if protocol.RUN.exists() or protocol.RUN.is_symlink():
        raise ValueError("pilot output must be create-only")
    available = protocol.free_bytes()
    required = protocol.FLOOR_BYTES + protocol.STARTUP_RESERVE_BYTES
    if available < required:
        raise ValueError(f"native pilot disk guard blocked: free={available}, required={required}")
    if protocol.CARLA_ROOT != owner.CARLA_ROOT or not os.access(protocol.CARLA_ROOT / "CarlaUE4.sh", os.X_OK):
        raise ValueError("packaged CARLA owner installation differs")
    for path, expected in owner.PINS.items():
        if owner.digest(path) != expected:
            raise ValueError("borrowed owner asset changed: " + str(path))
    check = subprocess.run([sys.executable, "-c", "import carla,agents; print(carla.__file__)"],
        cwd=protocol.ROOT, env=owner.environment(), capture_output=True,
        text=True, timeout=30, check=True)
    if not check.stdout.startswith(str(protocol.EGG) + "/"):
        raise ValueError("packaged CARLA API import path differs")
    owner.lifecycle._assert_no_other_carla()
    owner.ports_free(2440)
    from scripts.e2e import collect_town07_low_speed_10_30_pilot_20261001 as worker
    worker.parse_args(command(plan_sha256)[5:])
    return dict(schema="carla.town07_low_speed_10_30_native_owner_plan.v1",
        simulator_started=False, output=str(protocol.RUN), ports=[2440, 2441, 2442],
        declared_plan_sha256=plan_sha256,
        source_pins={str(path): digest for path, digest in owner.PINS.items()},
        collector_command=command(plan_sha256),
        server_command=["bash", "tools/e2e/run_carla_map.sh", "Town07", "--port", "2440",
            "--quality", "Epic", "--startup-timeout-sec", "120", "--", "-RenderOffScreen", "-nosound"],
        wall_timeout_seconds=240, preflight_free_bytes=available,
        required_free_bytes=required, camera_count=0, model_inferences=0,
        training_data_approved=False, vehicle_control_approved=False)


def final_report(plan: dict, plan_sha256: str) -> dict:
    run = protocol.RUN
    original = json.loads((run / "owner_result.json").read_text())
    episode = run / "episode.partial"
    worker = json.loads((episode / "manifest.json").read_text()) if (episode / "manifest.json").is_file() else {}
    replay = dict(status="MISSING_RETAINED")
    try:
        with (episode / "states.jsonl").open() as stream:
            rows = [json.loads(line) for line in stream]
        with (episode / "control_receipts.jsonl").open() as stream:
            receipts = [json.loads(line) for line in stream]
        assessment = protocol.assess_native_window(rows)
        if assessment != worker.get("assessment"):
            raise ValueError("raw fixed-window replay differs from worker assessment")
        if len(rows) != worker.get("native_states") or len(receipts) != len(rows) + 3:
            raise ValueError("native row/ACK count differs")
        if [receipt["sequence"] for receipt in receipts] != list(range(1, len(receipts) + 1)):
            raise ValueError("ACK receipt sequence is not gap-free")
        for row in rows:
            alignment = row["control_alignment"]
            sequence = alignment["expected_receipt_sequence"]
            receipt = receipts[sequence - 1]
            if (alignment["status"] != "PASS" or alignment["frame_binding_pass"] is not True
                    or alignment["observed_frame"] != row["frame"]
                    or receipt["after_ack_frame"] != row["frame"] - 1
                    or receipt["status"] != "ACKNOWLEDGED"):
                raise ValueError("native/ACK causal binding differs")
            for pedal in ("throttle", "brake", "steer"):
                if not math.isclose(row["current_control"][pedal],
                                    receipt["requested_control"][pedal], abs_tol=1e-6, rel_tol=0):
                    raise ValueError("observed control differs from ACK request")
        replay = dict(status="PASS", state_sha256=protocol.sha(episode / "states.jsonl"),
            receipt_sha256=protocol.sha(episode / "control_receipts.jsonl"),
            native_states=len(rows), acknowledged_commands=len(receipts),
            fixed_window_assessment=assessment)
    except (OSError, ValueError, TypeError, KeyError, IndexError) as error:
        replay = dict(status="FAIL", error_type=type(error).__name__, error=str(error))
    try:
        sources_ok = (protocol.sha(run / "declared_plan.json") == plan_sha256
            and all(protocol.sha(protocol.ROOT / name) == digest
                and protocol.sha(run / "source" / name) == digest
                for name, digest in plan["source_sha256"].items())
            and all(protocol.sha(Path(name)) == digest
                and protocol.sha(run / "source_external" / Path(name).relative_to(protocol.CARLA_ROOT)) == digest
                for name, digest in plan["external_agent_sha256"].items()))
    except (OSError, ValueError):
        sources_ok = False
    identity_ok = (worker.get("declared_plan_sha256") == plan_sha256
        and worker.get("camera_bundles") == 0 and worker.get("model_inferences") == 0)
    result = dict(schema="carla.town07_low_speed_10_30_native_owner_result.v1",
        original_owner=original, worker_status=worker.get("status", "MISSING_RETAINED"),
        worker_identity_matches=identity_ok, source_hashes_match=sources_ok,
        independent_raw_replay=replay,
        assessment=worker.get("assessment"), native_states=worker.get("native_states"),
        camera_bundles=0, model_inferences=0,
        final_free_bytes=protocol.free_bytes(), training_data_approved=False,
        vehicle_control_approved=False, full_six_camera_capture_executed=False)
    result["status"] = ("PASS_FEASIBILITY_ONLY" if original.get("collector_exit_code") == 0
        and worker.get("status") == "PASS_FEASIBILITY_ONLY" and identity_ok and sources_ok
        and replay["status"] == "PASS"
        and not original.get("cleanup_errors") and original.get("ports_released") is True
        and result["final_free_bytes"] >= protocol.FLOOR_BYTES else "FAILED_RETAINED")
    owner.write_json(run / "pilot_result.json", result)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--mode", choices=("declare", "preflight", "execute"), required=True)
    parser.add_argument("--plan-sha256")
    args = parser.parse_args(argv)
    if args.mode == "declare":
        if args.plan_sha256 is not None:
            raise ValueError("declaration accepts no caller-selected plan SHA")
        plan = protocol.build_plan()
        protocol.PLAN.parent.mkdir(parents=True, exist_ok=True)
        with protocol.PLAN.open("xb") as stream:
            stream.write(protocol.encoded(plan))
            stream.flush()
            os.fsync(stream.fileno())
        print(json.dumps(dict(status="DECLARED_NO_SIMULATOR", plan=str(protocol.PLAN),
            sha256=protocol.sha(protocol.PLAN), training_data_approved=False)))
        return 0
    if args.plan_sha256 is None:
        raise ValueError("preflight and execution require caller-pinned declared plan SHA")
    plan = json.loads(protocol.PLAN.read_text())
    with scoped_owner(plan, args.plan_sha256):
        check = preflight(plan, args.plan_sha256)
        if args.mode == "preflight":
            print(json.dumps(check, sort_keys=True, indent=2))
            return 0
        owner.execute(check)
        outcome = final_report(plan, args.plan_sha256)
        print(json.dumps(outcome, sort_keys=True, indent=2))
        return 0 if outcome["status"] == "PASS_FEASIBILITY_ONLY" else 1


if __name__ == "__main__":
    raise SystemExit(main())
