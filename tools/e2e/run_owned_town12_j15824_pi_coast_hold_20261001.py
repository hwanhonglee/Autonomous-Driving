#!/usr/bin/env python3
"""HH_261001 - Own one separately frozen NON-TRAIN controller replacement, retaining every source/process/port gate."""

from contextlib import contextmanager
import json
from pathlib import Path

from scripts.e2e import run_owned_town12_j15824_brake_calibration_20261001 as inherited
from scripts.e2e import run_owned_town12_j15824_brake_calibration_v4_20261001 as prior_owner
from scripts.e2e import calibrate_town12_j15824_pi_coast_hold_20261001 as worker


ROOT = inherited.ROOT
OUTPUT = ROOT / "storage/experiments/validation/2026-10-01/town12_j15824_pi_coast_hold_v1"
WORKER = ROOT / "tools/e2e/calibrate_town12_j15824_pi_coast_hold_20261001.py"
PREREGISTRATION = ROOT / "docs/assets/validation/2026-10-01/town12_pi_coast_hold_v1/PREREGISTRATION.md"
V4_RESULT = prior_owner.OUTPUT / "owner_result.json"
V4_RESULT_SHA256 = "b7d79fb91362f6fcdf04f585408bdf6e1a16684b11ae7f4b4d6d3fee6c8c2de5"
V4_COAST = prior_owner.OUTPUT / "calibration/level_00/result.json"
V4_COAST_SHA256 = "ded00452ca969359e9782364895cb3768a366e16529a10f0e681396fe2b97190"
SOURCE_FILES = tuple(dict.fromkeys((*prior_owner.SOURCE_FILES,
    "tools/e2e/calibrate_town12_j15824_pi_coast_hold_20261001.py",
    "tools/e2e/run_owned_town12_j15824_pi_coast_hold_20261001.py",
    "learning/tests/test_town12_pi_coast_hold_20261001.py",
    str(PREREGISTRATION.relative_to(ROOT)), str(V4_RESULT.relative_to(ROOT)), str(V4_COAST.relative_to(ROOT)))))
EXTERNAL_EVIDENCE = {
    "external/UnrealEngine_4.26/Engine/Source/ThirdParty/PhysX3/PhysX_3.4/Source/PhysXVehicle/src/PxVehicleUpdate.cpp":
        Path("/home/a/UnrealEngine_4.26/Engine/Source/ThirdParty/PhysX3/PhysX_3.4/Source/PhysXVehicle/src/PxVehicleUpdate.cpp"),
    "external/carla/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Vehicle/CarlaWheeledVehicle.cpp":
        Path("/home/a/carla/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Vehicle/CarlaWheeledVehicle.cpp"),
    "external/carla/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Vehicle/MovementComponents/DefaultMovementComponent.cpp":
        Path("/home/a/carla/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Vehicle/MovementComponents/DefaultMovementComponent.cpp"),
}


def validate_prior_evidence():
    worker.scout.require(worker.scout.digest(V4_RESULT) == V4_RESULT_SHA256
        and worker.scout.digest(V4_COAST) == V4_COAST_SHA256, "frozen v4 measurement evidence changed")
    result, coast = json.loads(V4_RESULT.read_text()), json.loads(V4_COAST.read_text())
    worker.scout.require(result.get("status") == "PASS"
        and result.get("owned_processes_stopped_and_ports_released") is True
        and result.get("source_bytes_unchanged_and_archived") is True
        and result.get("source_assets_unchanged") is True
        and coast.get("status") == "PASS_MEASURED_CALIBRATION"
        and coast.get("actor_cleanup", {}).get("status") == "PASS"
        and coast.get("pulse_observed_ticks") == 40 and coast.get("brake_level") == 0.
        and coast.get("training_data_admitted") is False,
        "v4 coast measurement and complete ownership/source cleanup proof required")


@contextmanager
def configured_owner():
    old = inherited.OUTPUT, inherited.WORKER, inherited.SOURCE_FILES, inherited.worker
    scout_owner = inherited.inherited
    old_external = scout_owner.EXTERNAL_SOURCES
    try:
        inherited.OUTPUT, inherited.WORKER, inherited.SOURCE_FILES, inherited.worker = OUTPUT, WORKER, SOURCE_FILES, worker
        scout_owner.EXTERNAL_SOURCES = {**old_external, **EXTERNAL_EVIDENCE}
        yield
    finally:
        inherited.OUTPUT, inherited.WORKER, inherited.SOURCE_FILES, inherited.worker = old
        scout_owner.EXTERNAL_SOURCES = old_external


def main(argv=None):
    worker.require_free_space(ROOT)
    validate_prior_evidence()
    with configured_owner():
        return inherited.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
