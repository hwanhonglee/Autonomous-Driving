#!/usr/bin/env python3
"""HH_261001 - Own a bounded lower-speed calibration with fresh actors and the original source/process/port gates."""

from contextlib import contextmanager
import json

from scripts.e2e import run_owned_town12_j15824_brake_calibration_v2_20261001 as inherited
from scripts.e2e import calibrate_town12_j15824_brake_pulses_v3_20261001 as worker


ROOT = inherited.ROOT
OUTPUT = ROOT / "storage/experiments/validation/2026-10-01/town12_j15824_fresh_brake_calibration_v3"
WORKER = ROOT / "tools/e2e/calibrate_town12_j15824_brake_pulses_v3_20261001.py"
FAILED_V2 = inherited.OUTPUT / "owner_result.json"
FAILED_V2_SHA256 = "8aa40f6a41971aa20ae23e0b46640c2196d2c36499ededec788313bb891fad3c"
V2_LEVEL = inherited.OUTPUT / "calibration/level_00/result.json"
V2_LEVEL_SHA256 = "f077641b4cacab8f8f21b8b49c4fb0746586735c0714d51a11fd37d32e06005b"
SOURCE_FILES = (*inherited.SOURCE_FILES,
    "tools/e2e/calibrate_town12_j15824_brake_pulses_v3_20261001.py",
    "tools/e2e/run_owned_town12_j15824_brake_calibration_v3_20261001.py",
    "learning/tests/test_town12_brake_calibration_v3_20261001.py",
    str(FAILED_V2.relative_to(ROOT)), str(V2_LEVEL.relative_to(ROOT)))


@contextmanager
def configured_owner():
    old = inherited.OUTPUT, inherited.WORKER, inherited.SOURCE_FILES, inherited.worker
    try:
        inherited.OUTPUT, inherited.WORKER, inherited.SOURCE_FILES, inherited.worker = OUTPUT, WORKER, SOURCE_FILES, worker
        yield
    finally:
        inherited.OUTPUT, inherited.WORKER, inherited.SOURCE_FILES, inherited.worker = old


def main(argv=None):
    worker.scout.require(worker.scout.digest(FAILED_V2) == FAILED_V2_SHA256
        and worker.scout.digest(V2_LEVEL) == V2_LEVEL_SHA256, "frozen v2 failure evidence changed")
    failed, level = json.loads(FAILED_V2.read_text()), json.loads(V2_LEVEL.read_text())
    worker.scout.require(failed.get("status") == "FAIL"
        and failed.get("owned_processes_stopped_and_ports_released") is True
        and failed.get("source_bytes_unchanged_and_archived") is True
        and failed.get("source_assets_unchanged") is True
        and level.get("error") == "ScoutError: entry-speed stabilization timed out"
        and level.get("actor_cleanup", {}).get("status") == "PASS",
        "failed v2 speed entry and complete ownership/source cleanup proof required")
    with configured_owner():
        return inherited.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
