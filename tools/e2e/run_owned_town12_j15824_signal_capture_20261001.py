#!/usr/bin/env python3
"""HH_261001 - Own one 20 km/h Town12 six-camera signal-aware expert trial."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
from pathlib import Path
import subprocess
import sys
from typing import Iterator

from scripts.e2e import run_owned_town12_j15824_signal_scout_20261001 as scout_owner
from scripts.e2e import capture_town12_j15824_signal_expert_20261001 as capture


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "storage/experiments/validation/2026-10-01/town12_j15824_signal_expert20_v1"
SCOUT_RESULT = ROOT / "storage/experiments/validation/2026-10-01/town12_j15824_signal_scout_v1/owner_result.json"
SCOUT_SHA256 = "31a0a268a18c025d6bab656c8528f5b24f3943c5a65f3d16a16f0bd063200ce8"
SOURCE_FILES = (*scout_owner.SOURCE_FILES,
    "tools/e2e/run_owned_town12_j15824_signal_capture_20261001.py",
    "tools/e2e/capture_town12_j15824_signal_expert_20261001.py",
    "learning/tests/test_town12_j15824_signal_capture_20261001.py",
    "tools/e2e/collect_carla_vad_expert.py", "tools/e2e/carla_goal_stop_profile.py",
    "learning/portable_e2e/model.py", "learning/portable_e2e/runtime_contract.py",
    "learning/portable_e2e/workspace_paths.py", "learning/portable_e2e/dataset.py",
    "learning/portable_e2e/contract.py", "learning/portable_e2e/stop_primitive_research.py",
    str(capture.collector.DEFAULT_MAPPING.relative_to(ROOT)),
    str(capture.collector.DEFAULT_CALIBRATION.relative_to(ROOT)),
    str(SCOUT_RESULT.relative_to(ROOT)),
)


def capture_preflight() -> dict:
    capture.scout.require(capture.scout.digest(SCOUT_RESULT) == SCOUT_SHA256,
                          "successful physical-site signal scout proof changed")
    result = json.loads(SCOUT_RESULT.read_text())
    capture.scout.require(result.get("status") == "PASS"
                          and result.get("owned_processes_stopped_and_ports_released") is True,
                          "prior successful signal scout ownership proof is absent")
    args = capture.collector.parse_args(capture.collector_arguments(OUTPUT / "capture", OUTPUT / "scout/route.json"))
    capture.collector.configuration_from_args(args)
    capture.collector.source_motion_bounds()
    return capture.camera_contract()


@contextmanager
def configured_owner() -> Iterator[None]:
    previous = scout_owner.OUTPUT, scout_owner.SOURCE_FILES, scout_owner.EXTERNAL_SOURCES
    try:
        scout_owner.OUTPUT, scout_owner.SOURCE_FILES = OUTPUT, SOURCE_FILES
        scout_owner.EXTERNAL_SOURCES = {**scout_owner.EXTERNAL_SOURCES,
            "external/carla/PythonAPI/carla/agents/navigation/controller.py": capture.scout.SOURCE_PLANNER.with_name("controller.py")}
        with scout_owner.configured_owner():
            owner = scout_owner.diagnostic.owner
            old_preflight, old_ready, old_read, old_write = owner.preflight, owner._wait_ready, owner._read_live_xodr, owner._write_new_json
            process = {}
            capture_receipt = {"capture_performed": False, "actor_spawned": False}

            def preflight():
                capture_preflight()
                binding = old_preflight()
                checked = subprocess.run([sys.executable, "-c", "import numpy, PIL, carla, agents, torch, shapely"],
                    cwd=ROOT, env=binding[3], capture_output=True, text=True, timeout=30)
                if checked.returncode:
                    raise owner.Town12OwnerError("source camera/agent runtime import preflight failed: " + checked.stderr[-500:])
                return binding

            def ready(editor, environment):
                old_ready(editor, environment)
                process["pid"] = editor.pid

            def read_live(environment):
                live = old_read(environment)
                pid = process["pid"]
                directory = OUTPUT / "capture"
                directory.mkdir()
                command = [sys.executable, str(ROOT / "tools/workspace_python.py"),
                    str(ROOT / "tools/e2e/capture_town12_j15824_signal_expert_20261001.py"),
                    "--execute", "--output", str(directory), "--scout", str(OUTPUT / "scout"),
                    "--owner-pid", str(pid), "--owner-pgid", str(pid)]
                capture_receipt.update(capture_performed=None, actor_spawned=None,
                                       status="WORKER_STARTED_NO_RECEIPT_YET")
                with (OUTPUT / "capture_worker.log").open("xb") as log:
                    result = subprocess.run(command, cwd=ROOT, env=environment, stdout=log,
                                            stderr=subprocess.STDOUT, timeout=900)
                capture_receipt.update(json.loads((directory / "capture_result.json").read_text()))
                if result.returncode != 0 or capture_receipt.get("status") != "PASS":
                    raise owner.Town12OwnerError(f"Town12 expert capture failed: {capture_receipt.get('error')}")
                return {**live, "capture": dict(capture_receipt)}

            def write(path, value):
                if path.name == "owner_plan.json":
                    value = {**value, "capture_enabled": True, "actor_spawn_enabled": True,
                        "signal_expert_trial": {"target_speed_kmh": 20.0,
                            "camera_contract": capture.camera_contract(),
                            "collector_arguments": capture.collector_arguments(OUTPUT / "capture", OUTPUT / "scout/route.json"),
                            "capture_requires_same_run_scout_pass": True,
                            "traffic_lights_ignored": False, "training_data_admitted": False}}
                elif path.name == "owner_result.json":
                    value = {**value, "capture_performed": capture_receipt.get("capture_performed"),
                             "actor_spawned": capture_receipt.get("actor_spawned"),
                             "capture": dict(capture_receipt), "training_data_admitted": False}
                old_write(path, value)

            try:
                owner.preflight, owner._wait_ready, owner._read_live_xodr, owner._write_new_json = preflight, ready, read_live, write
                yield
            finally:
                owner.preflight, owner._wait_ready, owner._read_live_xodr, owner._write_new_json = old_preflight, old_ready, old_read, old_write
    finally:
        scout_owner.OUTPUT, scout_owner.SOURCE_FILES, scout_owner.EXTERNAL_SOURCES = previous


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args(argv)
    try:
        with configured_owner():
            owner = scout_owner.diagnostic.owner
            prepared = owner.preflight()
            if not args.execute:
                absent = not (OUTPUT.exists() or OUTPUT.is_symlink())
                print(json.dumps({"status": "STATIC_READY_NOT_LIVE_TESTED" if absent else "STOP_OUTPUT_EXISTS",
                    "output": str(OUTPUT), "output_absent": absent, "simulator_started": False,
                    "capture_on_execute": True, "target_speed_kmh": 20.0,
                    "six_camera_geometry_unchanged": True, "postprocess_effects": True,
                    "signal_scout_repeated_before_capture": True, "training_data_admitted": False}, sort_keys=True))
                return 0 if absent else 2
            return owner.execute(*prepared)
    except (capture.scout.ScoutError, scout_owner.diagnostic.owner.Town12OwnerError,
            scout_owner.diagnostic.owner.shared_owner.CatalogOwnerError,
            OSError, ValueError, KeyError, subprocess.SubprocessError) as error:
        print(f"Town12 signal capture preflight failed: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
