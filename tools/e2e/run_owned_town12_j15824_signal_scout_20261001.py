#!/usr/bin/env python3
"""HH_261001 - Own the proven Town12 startup and one signal-aware physical-site scout."""

from __future__ import annotations

from contextlib import contextmanager
import json
from pathlib import Path
import subprocess
import sys
from typing import Iterator

try:
    from . import run_owned_town12_distance_field_diagnostic_20261001 as diagnostic
    from . import scout_town12_j15824_live_20261001 as scout
except ImportError:
    from scripts.e2e import run_owned_town12_distance_field_diagnostic_20261001 as diagnostic
    from scripts.e2e import scout_town12_j15824_live_20261001 as scout


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "storage/experiments/validation/2026-10-01/town12_j15824_signal_scout_v1"
STARTUP_RESULT = ROOT / "storage/experiments/validation/2026-10-01/town12_generate_distance_fields_zero_v1/owner_result.json"
STARTUP_SHA256 = "6f31521fa971f3f3746ebf733f4c589e63acd4cfc9d24dbad9113d47099c49f9"
STATIC_ROUTE_SHA256 = "4f6895d36494c3971c987a0a571e421a860b65877fbeced181867ace198cb936"
SOURCE_FILES = (*diagnostic.SOURCE_FILES,
    "tools/e2e/run_owned_town12_j15824_signal_scout_20261001.py",
    "tools/e2e/scout_town12_j15824_live_20261001.py",
    "learning/tests/test_town12_j15824_live_scout_20261001.py",
    str(scout.REPORT.relative_to(ROOT)), str(STARTUP_RESULT.relative_to(ROOT)),
    str((scout.STATIC_OUTPUT / "route.json").relative_to(ROOT)),
    str((scout.STATIC_OUTPUT / "binding.json").relative_to(ROOT)),
)
EXTERNAL_SOURCES = {
    "external/carla/PythonAPI/carla/agents/navigation/global_route_planner.py": scout.SOURCE_PLANNER,
    "external/carla/PythonAPI/carla/agents/navigation/basic_agent.py": scout.SOURCE_PLANNER.with_name("basic_agent.py"),
    "external/carla/PythonAPI/carla/agents/navigation/local_planner.py": scout.SOURCE_PLANNER.with_name("local_planner.py"),
    "external/carla/PythonAPI/carla/agents/tools/misc.py": scout.SOURCE_PLANNER.parents[1] / "tools/misc.py",
    "external/carla/LibCarla/source/carla/client/TrafficLight.cpp": Path("/home/a/carla/LibCarla/source/carla/client/TrafficLight.cpp"),
    "external/carla/LargeMapManager.cpp": Path("/home/a/carla/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/MapGen/LargeMapManager.cpp"),
}


def binding() -> dict:
    scout.require(scout.digest(STARTUP_RESULT) == STARTUP_SHA256, "successful numeric-zero startup proof changed")
    result = json.loads(STARTUP_RESULT.read_text())
    scout.require(result["status"] == "PASS" and result["renderer_diagnostic"]["status"] == "PASS"
                  and result["owned_processes_stopped_and_ports_released"] is True
                  and result["live_opendrive"]["sha256"] == scout.XODR_SHA256,
                  "successful source startup prerequisites absent")
    scout.require(scout.digest(scout.STATIC_OUTPUT / "route.json") == STATIC_ROUTE_SHA256,
                  "exact reconstructed static route changed")
    return scout.static_binding()


@contextmanager
def configured_owner() -> Iterator[None]:
    old_output, old_sources, old_paths = diagnostic.OUTPUT, diagnostic.SOURCE_FILES, diagnostic.source_paths
    try:
        diagnostic.OUTPUT = OUTPUT
        diagnostic.SOURCE_FILES = SOURCE_FILES
        diagnostic.source_paths = lambda: {**old_paths(), **EXTERNAL_SOURCES}
        with diagnostic.configured_owner():
            owner = diagnostic.owner
            old_preflight, old_ready = owner.preflight, owner._wait_ready
            old_read, old_write = owner._read_live_xodr, owner._write_new_json
            process = {}

            def preflight():
                binding()
                return old_preflight()

            def ready(editor, environment):
                old_ready(editor, environment)
                process["pid"] = editor.pid

            def read_live(environment):
                live = old_read(environment)
                pid = process["pid"]
                directory = OUTPUT / "scout"
                directory.mkdir()
                command = [sys.executable, str(ROOT / "tools/workspace_python.py"),
                           str(ROOT / "tools/e2e/scout_town12_j15824_live_20261001.py"),
                           "--execute", "--output", str(directory),
                           "--owner-pid", str(pid), "--owner-pgid", str(pid)]
                with (OUTPUT / "scout_worker.log").open("xb") as log:
                    result = subprocess.run(command, cwd=ROOT, env=environment,
                                            stdout=log, stderr=subprocess.STDOUT, timeout=420)
                receipt = json.loads((directory / "scout_result.json").read_text())
                if result.returncode != 0 or receipt.get("status") != "PASS":
                    raise owner.Town12OwnerError(f"Town12 signal scout failed: {receipt.get('error')}")
                if (receipt.get("route_polyline_sha256") != scout.ROUTE_SHA256
                        or receipt.get("xodr_sha256") != scout.XODR_SHA256
                        or receipt.get("hold_wall_seconds", 0) < scout.HOLD_SECONDS
                        or receipt.get("controlling_signal", {}).get("opendrive_id") != scout.SIGNAL_ID):
                    raise owner.Town12OwnerError("Town12 signal scout proof differs from the fixed scope")
                return {**live, "signal_scout": receipt}

            def write(path, value):
                if path.name == "owner_plan.json":
                    value = {**value, "signal_scout": {
                        "route_polyline_sha256": scout.ROUTE_SHA256,
                        "signal_opendrive_id": scout.SIGNAL_ID, "identity_hold_sec": scout.HOLD_SECONDS,
                        "spectator_relocation_before_sensors": True,
                        "ground_collision_queries_required": True,
                        "traffic_lights_ignored": False, "actor_spawn_enabled": False,
                        "capture_enabled": False, "training_data_admitted": False,
                    }}
                old_write(path, value)

            try:
                owner.preflight, owner._wait_ready = preflight, ready
                owner._read_live_xodr, owner._write_new_json = read_live, write
                yield
            finally:
                owner.preflight, owner._wait_ready = old_preflight, old_ready
                owner._read_live_xodr, owner._write_new_json = old_read, old_write
    finally:
        diagnostic.OUTPUT, diagnostic.SOURCE_FILES, diagnostic.source_paths = old_output, old_sources, old_paths


def main(argv=None) -> int:
    try:
        with configured_owner():
            return diagnostic.owner.main(argv)
    except scout.ScoutError as error:
        print(f"Town12 signal scout preflight failed: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
