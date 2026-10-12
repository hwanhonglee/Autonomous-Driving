#!/usr/bin/env python3
"""HH_261001 - Preserve failed calibrations and own one target-only, explicitly non-training v4 revision."""

from contextlib import contextmanager
import json

from scripts.e2e import run_owned_town12_j15824_brake_calibration_v3_20261001 as inherited
from scripts.e2e import calibrate_town12_j15824_brake_pulses_v4_20261001 as worker


ROOT = inherited.ROOT
OUTPUT = ROOT / "storage/experiments/validation/2026-10-01/town12_j15824_fresh_brake_calibration_v4"
WORKER = ROOT / "tools/e2e/calibrate_town12_j15824_brake_pulses_v4_20261001.py"
FAILED_V3 = inherited.OUTPUT / "owner_result.json"
FAILED_V3_SHA256 = "5711d78ddc91715908a110612b172908d078269d05edb1213d0b24050416c100"
V3_LEVEL = inherited.OUTPUT / "calibration/level_00/result.json"
V3_LEVEL_SHA256 = "a6ead790e0592f6ab368ccb2c7408eddd47009b71819a09ee1fbc88708d3d55a"
SOURCE_FILES = (*inherited.SOURCE_FILES,
    "tools/e2e/calibrate_town12_j15824_brake_pulses_v4_20261001.py",
    "tools/e2e/run_owned_town12_j15824_brake_calibration_v4_20261001.py",
    "learning/tests/test_town12_brake_calibration_v4_20261001.py",
    str(FAILED_V3.relative_to(ROOT)), str(V3_LEVEL.relative_to(ROOT)))


@contextmanager
def configured_owner():
    old = inherited.OUTPUT, inherited.WORKER, inherited.SOURCE_FILES, inherited.worker
    try:
        inherited.OUTPUT, inherited.WORKER, inherited.SOURCE_FILES, inherited.worker = OUTPUT, WORKER, SOURCE_FILES, worker
        yield
    finally:
        inherited.OUTPUT, inherited.WORKER, inherited.SOURCE_FILES, inherited.worker = old


def main(argv=None):
    worker.scout.require(worker.scout.digest(FAILED_V3) == FAILED_V3_SHA256
        and worker.scout.digest(V3_LEVEL) == V3_LEVEL_SHA256, "frozen v3 failure evidence changed")
    failed, level = json.loads(FAILED_V3.read_text()), json.loads(V3_LEVEL.read_text())
    worker.scout.require(failed.get("status") == "FAIL"
        and failed.get("owned_processes_stopped_and_ports_released") is True
        and failed.get("source_bytes_unchanged_and_archived") is True
        and failed.get("source_assets_unchanged") is True
        and level.get("error") == "ScoutError: entry-speed stabilization timed out"
        and level.get("actor_cleanup", {}).get("status") == "PASS"
        and level.get("effective_target_speed_kmh") == 5.58,
        "v3 measured entry failure and complete ownership/source cleanup proof required")
    with configured_owner():
        return inherited.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
