#!/usr/bin/env python3
"""HH_261001 - Declare or own one trial of the fixed native-only Town07 ABBA study."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

from scripts.e2e import run_owned_ah_town07_research_20261001 as owner
from scripts.e2e import town07_throttle_isolation_contract_20261001 as protocol


def command(spec, plan_sha256):
    return [sys.executable, str(protocol.ROOT / "tools/workspace_python.py"), "-m",
        "scripts.e2e.collect_town07_throttle_isolation_20261001", "--",
        "--trial", str(spec["index"]), "--plan-sha256", plan_sha256,
        "--output", str(Path(spec["output"]) / "episode")]


def prior_ready(spec):
    """HH_261001 - Continue fixed order after physical failure only when prior ownership and source cleanup passed."""
    if spec["index"] == 0:
        return
    previous = protocol.trial(spec["index"] - 1)
    folder = Path(previous["output"])
    result_path, study_path = folder / "owner_result.json", folder / "throttle_result.json"
    if not result_path.is_file() or not study_path.is_file():
        raise ValueError("previous fixed ABBA trial lacks both retained owner/study receipts")
    result, study = json.loads(result_path.read_text()), json.loads(study_path.read_text())
    if result.get("cleanup_errors") or result.get("ports_released") is not True:
        raise ValueError("previous owned trial did not release source/process/ports")
    if study.get("source_hashes_match") is not True or study.get("trial_id") != previous["trial_id"]:
        raise ValueError("previous fixed ABBA source/identity audit failed")


def archive_declared_sources(run: Path, value: dict, plan: dict):
    """HH_261001 - Keep actual local and external Python source bytes alongside the owner manifest."""
    source_map = value.setdefault("source_sha256", {})
    for name, expected in plan["source_sha256"].items():
        source = protocol.ROOT / name
        target = run / "source" / name
        if not target.exists():
            target.parent.mkdir(parents=True, exist_ok=True)
            with source.open("rb") as read, target.open("xb") as write:
                shutil.copyfileobj(read, write)
        if protocol.sha(target) != expected or protocol.sha(source) != expected:
            raise ValueError("archived local source differs: " + name)
        source_map[name] = expected
    for name, expected in plan["external_agent_sha256"].items():
        source = Path(name)
        relative = source.relative_to(protocol.CARLA_ROOT)
        target = run / "source_external" / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        with source.open("rb") as read, target.open("xb") as write:
            shutil.copyfileobj(read, write)
        if protocol.sha(target) != expected or protocol.sha(source) != expected:
            raise ValueError("archived CARLA agent source differs: " + name)
    value["external_agent_sha256"] = dict(plan["external_agent_sha256"])
    value["declared_plan_sha256"] = protocol.sha(protocol.PLAN)
    with protocol.PLAN.open("rb") as read, (run / "declared_plan.json").open("xb") as write:
        shutil.copyfileobj(read, write)


@contextmanager
def scoped_owner(spec, plan: dict, plan_sha256: str):
    saved = owner.OUTPUT, owner.PINS, owner.collector_command, owner.write_json, owner.lifecycle._workspace_lock
    original_write, original_lock = owner.write_json, owner.lifecycle._workspace_lock
    run = Path(spec["output"])
    pinned = {path: digest for path, digest in owner.PINS.items()
              if path not in (owner.WEIGHTS, owner.METADATA)}
    pinned.update({protocol.ROOT / name: digest for name, digest in plan["source_sha256"].items()})
    pinned.update({Path(name): digest for name, digest in plan["external_agent_sha256"].items()})
    pinned[protocol.PLAN] = plan_sha256

    def archive_write(path, value):
        if path == run / "owner_plan.json":
            archive_declared_sources(run, value, plan)
        original_write(path, value)

    @contextmanager
    def reserved_lock():
        with original_lock():
            prior_ready(spec)
            protocol.verify_plan(plan, expected_sha256=plan_sha256)
            if protocol.free_bytes() < protocol.FLOOR_BYTES + protocol.TRIAL_RESERVE_BYTES:
                raise ValueError("fixed native trial lacks 5 GiB floor plus 128 MiB startup reserve")
            yield

    owner.OUTPUT, owner.PINS = run, pinned
    owner.collector_command = lambda: command(spec, plan_sha256)
    owner.write_json, owner.lifecycle._workspace_lock = archive_write, reserved_lock
    try:
        yield
    finally:
        (owner.OUTPUT, owner.PINS, owner.collector_command,
         owner.write_json, owner.lifecycle._workspace_lock) = saved


def preflight(spec, plan, plan_sha256):
    if sys.version_info[:2] != (3, 10):
        raise ValueError("packaged CARLA Python API requires Python 3.10")
    if protocol.CARLA_ROOT != owner.CARLA_ROOT:
        raise ValueError("packaged CARLA installation differs from owned lifecycle")
    protocol.verify_plan(plan, expected_sha256=plan_sha256)
    prior_ready(spec)
    output = Path(spec["output"])
    if output.exists() or output.is_symlink():
        raise ValueError("fresh native trial output required; no retry or overwrite")
    available = protocol.free_bytes()
    required = protocol.FLOOR_BYTES + protocol.TRIAL_RESERVE_BYTES
    if available < required:
        raise ValueError(f"disk reserve blocked: free={available}, required={required}")
    if not os.access(protocol.CARLA_ROOT / "CarlaUE4.sh", os.X_OK):
        raise ValueError("packaged CARLA executable missing")
    for path, expected in owner.PINS.items():
        if owner.digest(path) != expected:
            raise ValueError("borrowed owned-lifecycle asset changed: " + str(path))
    # HH_261001 - Check the no-GPU Python API import without connecting to or starting a simulator.
    check = subprocess.run([sys.executable, "-c", "import carla,agents; print(carla.__file__)"],
        cwd=protocol.ROOT, env=owner.environment(), text=True, capture_output=True, timeout=30, check=True)
    if not check.stdout.startswith(str(protocol.EGG) + "/"):
        raise ValueError("packaged CARLA API did not import from the pinned egg")
    owner.lifecycle._assert_no_other_carla()
    owner.ports_free(2440)
    from scripts.e2e import collect_town07_throttle_isolation_20261001 as worker
    worker.parse_args(command(spec, plan_sha256)[5:])
    return dict(schema="carla.town07_native_throttle_isolation_preflight.v1",
        trial=spec, plan_sha256=plan_sha256, simulator_started=False,
        collector_command=command(spec, plan_sha256),
        server_command=["bash", "tools/e2e/run_carla_map.sh", "Town07", "--port", "2440",
            "--quality", "Epic", "--startup-timeout-sec", "120", "--", "-RenderOffScreen", "-nosound"],
        ports=[2440, 2441, 2442], source_pins={str(path): digest for path, digest in owner.PINS.items()},
        preflight_free_disk_bytes=available, required_free_disk_bytes=required,
        wall_timeout_seconds=240, camera_count=0, model_inferences=0,
        training_data_approved=False, vehicle_control_approved=False)


def final_report(spec, plan):
    run = Path(spec["output"])
    original = json.loads((run / "owner_result.json").read_text())
    episode = run / "episode.partial"
    worker = json.loads((episode / "manifest.json").read_text()) if (episode / "manifest.json").exists() else {}
    identity_ok = (worker.get("fixed_plan_sha256") == protocol.sha(protocol.PLAN)
        and worker.get("trial") == spec
        and isinstance(worker.get("summary"), dict)
        and worker["summary"].get("trial_id") == spec["trial_id"])
    source_audit_error = None
    try:
        sources_ok = all(protocol.sha(protocol.ROOT / name) == digest
            and protocol.sha(run / "source" / name) == digest for name, digest in plan["source_sha256"].items())
        sources_ok = sources_ok and all(protocol.sha(Path(name)) == digest
            and protocol.sha(run / "source_external" / Path(name).relative_to(protocol.CARLA_ROOT)) == digest
            for name, digest in plan["external_agent_sha256"].items())
        sources_ok = sources_ok and protocol.sha(run / "declared_plan.json") == protocol.sha(protocol.PLAN)
    except (OSError, ValueError) as error:
        sources_ok, source_audit_error = False, f"{type(error).__name__}: {error}"
    result = dict(schema="carla.town07_native_throttle_isolation_owner_result.v1",
        trial_id=spec["trial_id"], fixed_index=spec["index"], original_owner=original,
        worker_status=worker.get("status", "MISSING_RETAINED"), worker_identity_matches=identity_ok,
        summary=worker.get("summary"),
        source_hashes_match=sources_ok, source_audit_error=source_audit_error,
        native_states=worker.get("native_states"),
        camera_bundles=0, model_inferences=0, final_free_bytes=protocol.free_bytes(),
        training_data_approved=False, vehicle_control_approved=False, model_promoted=False)
    result["status"] = ("FINISHED_DIAGNOSTIC_ONLY" if original.get("collector_exit_code") == 0
        and worker.get("status") == "FINISHED_DIAGNOSTIC_ONLY" and sources_ok and identity_ok
        and not original.get("cleanup_errors") and result["final_free_bytes"] >= protocol.FLOOR_BYTES
        else "FAILED_RETAINED")
    owner.write_json(run / "throttle_result.json", result)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--mode", choices=("declare", "preflight", "execute"), required=True)
    parser.add_argument("--trial", type=int)
    parser.add_argument("--plan-sha256")
    args = parser.parse_args(argv)
    if args.mode == "declare":
        if args.trial is not None or args.plan_sha256 is not None:
            raise ValueError("declaration has no runtime trial overrides")
        if protocol.PLAN.exists() or protocol.PLAN.is_symlink() or protocol.RUN_ROOT.exists() or protocol.RUN_ROOT.is_symlink():
            raise ValueError("fresh create-only declared plan and output root required")
        plan = protocol.build_plan()
        protocol.PLAN.parent.mkdir(parents=True, exist_ok=True)
        owner.write_json(protocol.PLAN, plan)
        print(json.dumps(dict(status="DECLARED_NO_SIMULATOR", plan=str(protocol.PLAN),
            sha256=protocol.sha(protocol.PLAN), trials=len(protocol.trial_specs()))))
        return 0
    if args.trial is None or args.plan_sha256 is None:
        raise ValueError("exact trial index and declared plan SHA256 required")
    spec = protocol.trial(args.trial)
    plan = json.loads(protocol.PLAN.read_text())
    with scoped_owner(spec, plan, args.plan_sha256):
        check = preflight(spec, plan, args.plan_sha256)
        if args.mode == "preflight":
            print(json.dumps(check, indent=2, sort_keys=True))
            return 0
        owner.execute(check)
        result = final_report(spec, plan)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0 if result["status"] == "FINISHED_DIAGNOSTIC_ONLY" else 1


if __name__ == "__main__":
    raise SystemExit(main())
