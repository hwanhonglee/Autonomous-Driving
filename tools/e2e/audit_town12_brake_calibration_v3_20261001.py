#!/usr/bin/env python3
"""HH_261001 - Replay all v3 native stages with the exact low-speed policy; keep prior raw failures untouched."""

from contextlib import contextmanager
import json

from scripts.e2e import audit_town12_brake_calibration_v2_20261001 as inherited
from scripts.e2e import calibrate_town12_j15824_brake_pulses_v3_20261001 as worker
from scripts.e2e import run_owned_town12_j15824_brake_calibration_v3_20261001 as owner


WORKER_SHA256 = "7694ece619e51139a740f49b37027c959052dccf902dfd5bbf497c5b623f05c8"
ORIGINAL_SOURCE_AUDIT, ORIGINAL_ANALYZE = inherited.source_audit, inherited.analyze_level


def source_audit(trial, plan):
    with owner.configured_owner():
        return ORIGINAL_SOURCE_AUDIT(trial, plan)


def analyze_level(directory):
    result = ORIGINAL_ANALYZE(directory)
    captured = json.loads((directory / "result.json").read_text())
    result["effective_target_speed_kmh"] = captured.get("effective_target_speed_kmh")
    if captured.get("effective_target_speed_kmh") != 5.58:
        result["evidence_failures"].append({"kind": "ACTUAL_PID_TARGET_NOT_VERIFIED"})
    if result.get("pulse_measurement"):
        result["pulse_measurement"]["qualification_speed_scope_mps"] = [1.45, 1.55]
        result["pulse_measurement"]["higher_speed_extrapolation_approved"] = False
    return result


@contextmanager
def configured_audit():
    old = inherited.worker, inherited.WORKER_SHA256, inherited.source_audit, inherited.analyze_level
    old_policy = worker.inherited.PulsePolicy
    try:
        inherited.worker, inherited.WORKER_SHA256 = worker, WORKER_SHA256
        inherited.source_audit, inherited.analyze_level = source_audit, analyze_level
        worker.inherited.PulsePolicy = worker.PulsePolicy
        yield
    finally:
        inherited.worker, inherited.WORKER_SHA256, inherited.source_audit, inherited.analyze_level = old
        worker.inherited.PulsePolicy = old_policy


def main(argv=None):
    with configured_audit():
        return inherited.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
