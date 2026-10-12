#!/usr/bin/env python3
"""HH_261001 - Keep proven source/proc/port ownership; four tiny-brake measurements, not expert capture."""

import argparse
from contextlib import contextmanager
import json
from pathlib import Path
import subprocess
import sys

from scripts.e2e import run_owned_town12_j15824_signal_scout_20261001 as inherited
from scripts.e2e import run_owned_town12_j15824_spawn_lift_20261001 as source_closure
from scripts.e2e import calibrate_town12_j15824_brake_pulses_20261001 as worker


ROOT = inherited.ROOT
OUTPUT = ROOT / "storage/experiments/validation/2026-10-01/town12_j15824_fresh_brake_calibration_v1"
WORKER = ROOT / "tools/e2e/calibrate_town12_j15824_brake_pulses_20261001.py"
SOURCE_FILES = (*source_closure.SOURCE_FILES,
    "tools/e2e/collect_town07_low_speed_start_20261001.py",
    "tools/e2e/calibrate_town12_j15824_brake_pulses_20261001.py",
    "tools/e2e/run_owned_town12_j15824_brake_calibration_20261001.py",
    "learning/tests/test_town12_brake_calibration_20261001.py")


@contextmanager
def configured_owner():
    old = inherited.OUTPUT, inherited.SOURCE_FILES, inherited.EXTERNAL_SOURCES
    inherited.OUTPUT, inherited.SOURCE_FILES = OUTPUT, SOURCE_FILES
    inherited.EXTERNAL_SOURCES = {**inherited.EXTERNAL_SOURCES, **source_closure.EXTERNAL_SOURCES,
        "external/carla/PythonAPI/carla/agents/navigation/controller.py": worker.scout.SOURCE_PLANNER.with_name("controller.py")}
    inherited.EXTERNAL_SOURCES.update({f"external/carla/LibCarla/source/carla/{name}":
        Path("/home/a/carla/LibCarla/source/carla") / name for name in
        ("road/Map.cpp", "rpc/VehicleControl.h", "rpc/WheelPhysicsControl.h", "client/World.cpp",
         "client/detail/Episode.cpp", "client/detail/Simulator.cpp")})
    try:
        with inherited.configured_owner():
            owner = inherited.diagnostic.owner
            original = owner.preflight, owner._wait_ready, owner._read_live_xodr, owner._write_new_json
            old_preflight, old_ready, old_read, old_write = original
            process, measured = {}, {"status": "NOT_STARTED", "training_data_admitted": False}

            def preflight():
                worker.lift.nominal_body()
                worker.collector.source_motion_bounds()
                prepared = old_preflight()
                probe = subprocess.run([sys.executable, "-c", "import numpy, carla, agents, torch"],
                    cwd=ROOT, env=prepared[3], capture_output=True, text=True, timeout=30)
                if probe.returncode:
                    raise owner.Town12OwnerError("calibration dependency import failed: " + probe.stderr[-500:])
                return prepared

            def ready(editor, environment):
                old_ready(editor, environment)
                process["pid"] = editor.pid

            def read_live(environment):
                live = old_read(environment)
                directory = OUTPUT / "calibration"
                directory.mkdir()
                pid = process["pid"]
                command = [sys.executable, str(ROOT / "tools/workspace_python.py"), str(WORKER),
                    "--execute", "--output", str(directory), "--scout", str(OUTPUT / "scout"),
                    "--owner-pid", str(pid), "--owner-pgid", str(pid)]
                measured.update(status="WORKER_STARTED_NO_RECEIPT_YET")
                with (OUTPUT / "calibration_worker.log").open("xb") as log:
                    result = subprocess.run(command, cwd=ROOT, env=environment, stdout=log,
                                            stderr=subprocess.STDOUT, timeout=480)
                measured.update(json.loads((directory / "calibration_result.json").read_text()))
                if result.returncode or measured.get("status") != "PASS":
                    raise owner.Town12OwnerError("bounded brake calibration failed: " + str(measured.get("error")))
                return {**live, "actuator_calibration": dict(measured)}

            def write(path, value):
                if path.name == "owner_plan.json":
                    value = {**value, "actor_spawn_enabled": True, "capture_enabled": False,
                        "calibration_enabled": True, "actuator_calibration_protocol": worker.protocol(),
                        "worker_argv_template": [str(WORKER), "--execute", "--output", str(OUTPUT / "calibration"),
                                                 "--scout", str(OUTPUT / "scout"), "--owner-pid", "OWNED", "--owner-pgid", "OWNED"]}
                elif path.name == "owner_result.json":
                    value = {**value, "actuator_calibration": dict(measured),
                        "actor_spawned": any(row.get("actor_id") for row in measured.get("levels", [])),
                        "capture_performed": False, "training_data_admitted": False}
                old_write(path, value)

            try:
                owner.preflight, owner._wait_ready, owner._read_live_xodr, owner._write_new_json = preflight, ready, read_live, write
                yield owner
            finally:
                owner.preflight, owner._wait_ready, owner._read_live_xodr, owner._write_new_json = original
    finally:
        inherited.OUTPUT, inherited.SOURCE_FILES, inherited.EXTERNAL_SOURCES = old


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args(argv)
    try:
        with configured_owner() as owner:
            prepared = owner.preflight()
            if not args.execute:
                absent = not (OUTPUT.exists() or OUTPUT.is_symlink())
                print(json.dumps({"status": "STATIC_READY_NOT_LIVE_TESTED" if absent else "STOP_OUTPUT_EXISTS",
                    "output": str(OUTPUT), "simulator_started": False, "output_absent": absent,
                    "protocol": worker.protocol()}, sort_keys=True))
                return 0 if absent else 2
            return owner.execute(*prepared)
    except (worker.scout.ScoutError, OSError, ValueError, KeyError, subprocess.SubprocessError,
            inherited.diagnostic.owner.Town12OwnerError,
            inherited.diagnostic.owner.shared_owner.CatalogOwnerError) as error:
        print(f"Town12 calibration preflight failed: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
