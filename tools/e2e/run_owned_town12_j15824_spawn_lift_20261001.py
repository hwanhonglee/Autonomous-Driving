#!/usr/bin/env python3
"""HH_261001 - Versioned one-attempt Town12 capture with source-backed spawn lift."""

from __future__ import annotations

from contextlib import contextmanager
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

from scripts.e2e import run_owned_town12_j15824_signal_capture_20261001 as inherited
from scripts.e2e import capture_town12_j15824_spawn_lift_20261001 as capture


ROOT = inherited.ROOT
OUTPUT = ROOT / "storage/experiments/validation/2026-10-01/town12_j15824_signal_expert20_spawn05_v1"
FAILED_RESULT = ROOT / "storage/experiments/validation/2026-10-01/town12_j15824_signal_expert20_v1/owner_result.json"
FAILED_RESULT_SHA256 = "92be2c8249790eb2cf22a5a4e617fb02afa4f6c9fce6a05de2daf78ddf999454"
WORKER = ROOT / "tools/e2e/capture_town12_j15824_spawn_lift_20261001.py"
SOURCE_FILES = (*inherited.SOURCE_FILES,
    "tools/e2e/run_owned_town12_j15824_spawn_lift_20261001.py",
    "tools/e2e/capture_town12_j15824_spawn_lift_20261001.py",
    "learning/tests/test_town12_j15824_spawn_lift_20261001.py",
    str(capture.NOMINAL_MANIFEST.relative_to(ROOT)), str(FAILED_RESULT.relative_to(ROOT)))
EXTERNAL_SOURCES = {f"external/carla/{name}": Path("/home/a/carla") / name for name in (
    "Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Game/CarlaGameModeBase.cpp",
    "Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Server/CarlaServer.cpp",
    "Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Util/RayTracer.cpp",
    "PythonAPI/carla/source/libcarla/World.cpp")}


def dispatch_worker(command, **kwargs):
    """HH_261001 - Replace only the exact inherited worker argv; keep all other subprocesses intact."""
    old_worker = str(ROOT / "tools/e2e/capture_town12_j15824_signal_expert_20261001.py")
    if isinstance(command, list) and old_worker in command:
        capture.scout.require(len(command) > 3 and command[1] == str(ROOT / "tools/workspace_python.py")
            and command[2] == old_worker and command[3] == "--execute" and command.count(old_worker) == 1,
            "inherited capture worker argv changed")
        command = [*command[:2], str(WORKER), *command[3:]]
    return subprocess.run(command, **kwargs)


@contextmanager
def configured_owner():
    old = inherited.OUTPUT, inherited.SOURCE_FILES, inherited.capture, inherited.subprocess, inherited.capture_preflight
    old_external = inherited.scout_owner.EXTERNAL_SOURCES
    original_preflight = inherited.capture_preflight

    def preflight():
        capture.scout.require(capture.scout.digest(FAILED_RESULT) == FAILED_RESULT_SHA256,
                              "frozen failed zero-offset trial changed")
        failed = json.loads(FAILED_RESULT.read_text())
        capture.scout.require(failed.get("status") == "FAIL"
            and failed.get("owned_processes_stopped_and_ports_released") is True,
            "previous failed trial cleanup is not proven")
        capture.nominal_body()
        return original_preflight()

    try:
        inherited.OUTPUT, inherited.SOURCE_FILES, inherited.capture = OUTPUT, SOURCE_FILES, capture
        inherited.capture_preflight = preflight
        inherited.subprocess = SimpleNamespace(run=dispatch_worker, STDOUT=subprocess.STDOUT,
                                              SubprocessError=subprocess.SubprocessError)
        inherited.scout_owner.EXTERNAL_SOURCES = {**old_external, **EXTERNAL_SOURCES}
        yield
    finally:
        inherited.OUTPUT, inherited.SOURCE_FILES, inherited.capture, inherited.subprocess, inherited.capture_preflight = old
        inherited.scout_owner.EXTERNAL_SOURCES = old_external


def main(argv=None) -> int:
    with configured_owner():
        return inherited.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
