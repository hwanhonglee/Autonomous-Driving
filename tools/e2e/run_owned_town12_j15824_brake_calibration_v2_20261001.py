#!/usr/bin/env python3
"""HH_261001 - Own one separately frozen non-training timed-entry calibration; retain the failed v1 raw data."""

from contextlib import contextmanager
import json

from scripts.e2e import run_owned_town12_j15824_brake_calibration_20261001 as inherited
from scripts.e2e import calibrate_town12_j15824_brake_pulses_v2_20261001 as worker


ROOT = inherited.ROOT
OUTPUT = ROOT / "storage/experiments/validation/2026-10-01/town12_j15824_fresh_brake_calibration_v2"
WORKER = ROOT / "tools/e2e/calibrate_town12_j15824_brake_pulses_v2_20261001.py"
FAILED_V1 = inherited.OUTPUT / "owner_result.json"
FAILED_V1_SHA256 = "f7b46d6097e67698384c0e5cca45817a4b0c301facbd1ca3c7f7accf9a019d89"
V1_LEVEL = inherited.OUTPUT / "calibration/level_00/result.json"
V1_LEVEL_SHA256 = "e3f1892dbabb4022593ceecd3972faa9b208e435dd6e5d9818a09f81e6d30e4e"
SOURCE_FILES = (*inherited.SOURCE_FILES,
    "tools/e2e/calibrate_town12_j15824_brake_pulses_v2_20261001.py",
    "tools/e2e/run_owned_town12_j15824_brake_calibration_v2_20261001.py",
    "learning/tests/test_town12_brake_calibration_v2_20261001.py",
    str(FAILED_V1.relative_to(ROOT)), str(V1_LEVEL.relative_to(ROOT)))


@contextmanager
def configured_owner():
    old = inherited.OUTPUT, inherited.WORKER, inherited.SOURCE_FILES, inherited.worker
    try:
        inherited.OUTPUT, inherited.WORKER, inherited.SOURCE_FILES, inherited.worker = OUTPUT, WORKER, SOURCE_FILES, worker
        yield
    finally:
        inherited.OUTPUT, inherited.WORKER, inherited.SOURCE_FILES, inherited.worker = old


def main(argv=None):
    worker.scout.require(worker.scout.digest(FAILED_V1) == FAILED_V1_SHA256
        and worker.scout.digest(V1_LEVEL) == V1_LEVEL_SHA256, "frozen v1 failure evidence changed")
    failed, level = json.loads(FAILED_V1.read_text()), json.loads(V1_LEVEL.read_text())
    worker.scout.require(failed.get("status") == "FAIL"
        and failed.get("owned_processes_stopped_and_ports_released") is True
        and failed.get("source_bytes_unchanged_and_archived") is True
        and failed.get("source_assets_unchanged") is True
        and level.get("error") == "ValueError: fixed low-speed launch did not reach 0.5 m/s within 8 seconds"
        and level.get("actor_cleanup", {}).get("status") == "PASS",
        "failed launch and complete v1 ownership/source cleanup proof required")
    with configured_owner():
        return inherited.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
