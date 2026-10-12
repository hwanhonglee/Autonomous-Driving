#!/usr/bin/env python3
"""HH_261001 - Prepare or own one bounded Town07 AH control diagnostic; no model promotion."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

from . import run_owned_town13_route_catalog as lifecycle
from .run_owned_ctrack_isolated_probe_20261001 import (all_processes, descendants,
    listening_socket_owned, ports_free, process_record)


ROOT = Path(__file__).resolve().parents[2]
CARLA_ROOT = Path("/home/a/carla-autoware-universe/CARLA_0.9.15")
EGG = CARLA_ROOT / "PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg"
ROUTE = ROOT / "storage/datasets/raw/carla/common10_v1/2026-09-04/30kph/town07/straight/ClearNoon/seed_0000/run_001/episode/route.json"
WEIGHTS = ROOT / "storage/experiments/validation/2026-10-01/ah_simulation_weights/AH_simulation.npz"
METADATA = WEIGHTS.with_suffix(".json")
OUTPUT = ROOT / "storage/experiments/validation/2026-10-01/ah_town07_straight_research_v1"
PORT = 2440
MAPPING = ROOT / "autonomy/autoware_e2e_vad_launch/config/sensor_mapping_vad_fast_reliable.yaml"
CALIBRATION = ROOT / "autonomy/src/launcher/autoware_launch/sensor_kit/carla_sensor_kit_launch/carla_sensor_kit_description/config/sensor_kit_calibration.yaml"
XODR = CARLA_ROOT / "CarlaUE4/Content/Carla/Maps/OpenDrive/Town07.xodr"
PINS = {
    EGG: "6eb690ef4304b34a919b27a23129273029008c8e95a5f164d52a1bd494a96c3a",
    ROUTE: "8285e70a790d5e8ae75803db1aa538122b56e6fcfc3586e60eaa947d330417c9",
    WEIGHTS: "5fd783def4d67aa8c2abe556187bbfaa06012b1cb2fb5bf04faad760771c9342",
    METADATA: "e1558f43b89fb2e967112692f0f348f8f6e457f4bed39b3d592ba02f3cf02de1",
    MAPPING: "9aaff2befed7ad12376b2e04bbdd51bd1808a3bafe39d87a6f6b241dbcca3136",
    CALIBRATION: "5022cd1de5b48e9c824b6f2f8c59991fa665eeaf7a7bafd084c88acdb65e4bea",
    XODR: "be730652769e993e27f748ba5793dba9e49d299205146a7a8db0bcd17af877bf",
}


def digest(path):
    if path.is_symlink() or not path.is_file():
        raise RuntimeError("regular source required: " + str(path))
    return hashlib.sha256(path.read_bytes()).hexdigest()


def environment():
    env = os.environ.copy()
    env.update(CARLA_ROOT=str(CARLA_ROOT), CUDA_VISIBLE_DEVICES="", OMP_NUM_THREADS="2",
               OPENBLAS_NUM_THREADS="2", PYTHONPATH=os.pathsep.join(map(str,
                   (ROOT / "learning", ROOT, EGG, CARLA_ROOT / "PythonAPI",
                    CARLA_ROOT / "PythonAPI/carla"))))
    return env


def collector_command():
    return [sys.executable, str(ROOT / "tools/workspace_python.py"), "-m",
        "scripts.e2e.collect_ah_simulation_research", "--",
        "--research-weights", str(WEIGHTS), "--research-metadata", str(METADATA),
        "--research-weights-sha256", PINS[WEIGHTS], "--research-metadata-sha256", PINS[METADATA],
        str(OUTPUT / "episode"), str(ROUTE), "--host", "127.0.0.1", "--port", str(PORT),
        "--control-transport", "acknowledged_batch", "--stationary-warmup-sec", "1.2",
        "--max-duration-sec", "20", "--target-speed-kmh", "9", "--seed", "0",
        "--weather", "ClearNoon", "--mapping", str(MAPPING),
        "--calibration", str(CALIBRATION), "--spawn-z-offset-m", "0.5",
        "--expected-live-xodr-sha256", PINS[XODR], "--goal-stop-profile", "disabled"]


def preflight():
    if sys.version_info[:2] != (3, 10):
        raise RuntimeError("packaged CARLA API requires Python 3.10")
    for path, expected in PINS.items():
        if digest(path) != expected:
            raise RuntimeError("source hash changed: " + str(path))
    if not os.access(CARLA_ROOT / "CarlaUE4.sh", os.X_OK):
        raise RuntimeError("packaged CARLA executable missing")
    check = subprocess.run([sys.executable, "-c",
        "import carla,agents,torch,numpy,PIL; print(carla.__file__); print(torch.__version__)"],
        cwd=ROOT, env=environment(), text=True, capture_output=True, timeout=30, check=True)
    if not check.stdout.startswith(str(EGG) + "/"):
        raise RuntimeError("packaged API import is not source-pinned")
    # HH_261001 - Parse the complete worker command without connecting to a simulator.
    from .collect_ah_simulation_research import collector
    argv = collector_command()
    start = argv.index(str(OUTPUT / "episode"))
    collector.parse_args(argv[start:])
    return dict(schema="portable_e2e.owned_ah_town07_plan.v1", simulator_started=False,
        output=str(OUTPUT), ports=[PORT, PORT + 1, PORT + 2],
        source_checkpoint_sha256="4841d4076fb49145643ab2895d87befcae9021ad02f08b8307e75f77be137d91",
        source_pins={str(path): sha for path, sha in PINS.items()},
        collector_command=argv, server_command=["bash", "tools/e2e/run_carla_map.sh", "Town07",
            "--port", str(PORT), "--quality", "Epic", "--startup-timeout-sec", "120",
            "--", "-RenderOffScreen", "-nosound"],
        capture_budget_seconds=20, wall_timeout_seconds=240, research_speed_cap_kph=9,
        training_data_approved=False, vehicle_control_approved=False,
        interpretation="Bounded TRAIN-route control diagnostic. Full 210 m route completion is not expected in 20 seconds.")


def write_json(path, value):
    with path.open("x") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def execute(plan):
    with lifecycle._workspace_lock():
        lifecycle._assert_no_other_carla()
        ports_free(PORT)
        if OUTPUT.exists() or OUTPUT.is_symlink():
            raise RuntimeError("fresh diagnostic output required")
        if shutil.disk_usage(ROOT).free < 5 * 1024**3:
            raise RuntimeError("less than 5 GiB available")
        OUTPUT.mkdir(parents=True)
        imported = {Path(module.__file__).resolve() for module in list(sys.modules.values())
                    if getattr(module, "__file__", None) and Path(module.__file__).is_file()
                    and Path(module.__file__).resolve().is_relative_to(ROOT)}
        sources = sorted(imported | set(ROOT.glob("learning/portable_e2e/*.py"))
            | set(ROOT.glob("learning/portable_e2e/config/*.json"))
            | {ROOT / "tools/e2e/run_carla_map.sh", ROOT / "tools/e2e/env.sh", MAPPING, CALIBRATION, ROUTE})
        hashes = {}
        for path in sources:
            relative = path.relative_to(ROOT)
            target = OUTPUT / "source" / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            with path.open("rb") as src, target.open("xb") as dest:
                shutil.copyfileobj(src, dest)
            hashes[str(relative)] = digest(path)
        plan["source_sha256"] = hashes
        write_json(OUTPUT / "owner_plan.json", plan)
        server = worker = None
        server_identity = None
        error = None
        cleanup_errors = []
        worker_code = None
        prior = {}
        def interrupted(signum, frame):
            raise RuntimeError("owner interrupted by signal " + str(signum))
        try:
            for signum in (signal.SIGINT, signal.SIGTERM):
                prior[signum] = signal.signal(signum, interrupted)
            with (OUTPUT / "server.log").open("xb") as log:
                server = subprocess.Popen(plan["server_command"], cwd=ROOT, env=environment(),
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            server_identity = process_record(server.pid)
            deadline = time.monotonic() + 130
            socket_owner = None
            while time.monotonic() < deadline:
                if server.poll() is not None:
                    raise RuntimeError("owned packaged server exited before readiness")
                if b"CARLA_READY " in (OUTPUT / "server.log").read_bytes():
                    children = descendants(server.pid, all_processes())
                    owners = [r for r in children if r["pgid"] == server.pid
                        and any("CarlaUE4-Linux-Shipping" in a for a in r["argv"])
                        and listening_socket_owned(r["pid"], PORT)]
                    if len(owners) != 1:
                        raise RuntimeError("RPC socket is not uniquely owned by the started shipping server")
                    socket_owner = owners[0]
                    break
                time.sleep(1.)
            if socket_owner is None:
                raise RuntimeError("owned packaged server readiness timeout")
            write_json(OUTPUT / "owner_started.json", dict(server=server_identity, socket_owner=socket_owner))
            with (OUTPUT / "collector.log").open("xb") as log:
                worker = subprocess.Popen(plan["collector_command"], cwd=ROOT, env=environment(),
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                worker_code = worker.wait(timeout=240)
            if worker_code:
                error = "collector returned " + str(worker_code) + "; bounded route noncompletion or other failure retained"
        except BaseException as exc:
            error = f"{type(exc).__name__}: {exc}"
        finally:
            for signum, handler in prior.items():
                signal.signal(signum, handler)
            for process in (worker, server):
                try:
                    if process is server and server_identity:
                        now = process_record(server.pid)
                        if now and now["start_ticks"] != server_identity["start_ticks"]:
                            raise RuntimeError("server PID generation changed; refusing signal")
                    if not lifecycle._stop_owned_group(process):
                        raise RuntimeError("owned process group did not stop")
                except Exception as exc:
                    cleanup_errors.append(str(exc))
            try:
                ports_free(PORT)
                for relative, sha in hashes.items():
                    if digest(ROOT / relative) != sha or digest(OUTPUT / "source" / relative) != sha:
                        raise RuntimeError("source changed during diagnostic: " + relative)
                for path, expected in PINS.items():
                    if digest(path) != expected:
                        raise RuntimeError("pinned asset changed during diagnostic")
            except Exception as exc:
                cleanup_errors.append(str(exc))
            result = dict(status=("OWNER_FAILURE" if cleanup_errors else
                                  "EXECUTION_FAILED_RETAINED" if error else "FINISHED_RESEARCH_ONLY"),
                collector_exit_code=worker_code, error=error, cleanup_errors=cleanup_errors,
                ports_released=not cleanup_errors, training_data_approved=False, vehicle_control_approved=False)
            write_json(OUTPUT / "owner_result.json", result)
        print(json.dumps(result, indent=2))
        return 0 if not cleanup_errors and worker_code == 0 else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    plan = preflight()
    if args.execute:
        return execute(plan)
    print(json.dumps(plan, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
