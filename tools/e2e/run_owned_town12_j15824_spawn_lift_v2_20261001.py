#!/usr/bin/env python3
"""HH_261001 - Separate output for the source-validated pre-hook API binding repair."""

from contextlib import contextmanager
import json

from scripts.e2e import run_owned_town12_j15824_spawn_lift_20261001 as inherited


OUTPUT = inherited.ROOT / "storage/experiments/validation/2026-10-01/town12_j15824_signal_expert20_spawn05_v2"
WORKER = inherited.ROOT / "tools/e2e/capture_town12_j15824_spawn_lift_v2_20261001.py"
FAILED_LIFT = inherited.OUTPUT / "owner_result.json"
FAILED_LIFT_SHA256 = "4b3c0e98168a6226562ff7d91e7d8d69c2b6c92d0027852df15b2216c6800642"
SOURCE_FILES = (*inherited.SOURCE_FILES,
    "tools/e2e/run_owned_town12_j15824_spawn_lift_v2_20261001.py",
    "tools/e2e/capture_town12_j15824_spawn_lift_v2_20261001.py",
    "learning/tests/test_town12_j15824_spawn_lift_v2_20261001.py",
    str(FAILED_LIFT.relative_to(inherited.ROOT)))


@contextmanager
def configured_owner():
    previous = inherited.OUTPUT, inherited.WORKER, inherited.SOURCE_FILES
    try:
        inherited.OUTPUT, inherited.WORKER, inherited.SOURCE_FILES = OUTPUT, WORKER, SOURCE_FILES
        yield
    finally:
        inherited.OUTPUT, inherited.WORKER, inherited.SOURCE_FILES = previous


def main(argv=None):
    inherited.capture.scout.require(inherited.capture.scout.digest(FAILED_LIFT) == FAILED_LIFT_SHA256,
                                    "frozen pre-spawn wrapper-order failure changed")
    failed = json.loads(FAILED_LIFT.read_text())
    inherited.capture.scout.require(failed.get("status") == "FAIL"
        and failed.get("owned_processes_stopped_and_ports_released") is True
        and failed.get("capture", {}).get("error") == "ScoutError: wrong CARLA planner import",
        "expected pre-spawn wrapper-order failure and clean shutdown are absent")
    with configured_owner():
        return inherited.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
