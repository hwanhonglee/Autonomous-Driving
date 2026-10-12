#!/usr/bin/env python3
"""HH_261001 - Read-only v4 pulse and zero-brake baseline QA, never a higher-speed stopping qualification."""

from contextlib import contextmanager
import json

from scripts.e2e import audit_town12_brake_calibration_v2_20261001 as inherited
from scripts.e2e import calibrate_town12_j15824_brake_pulses_v4_20261001 as worker
from scripts.e2e import run_owned_town12_j15824_brake_calibration_v4_20261001 as owner


WORKER_SHA256 = "b3c50adc083894724fa49ec8ecf9a4d5fd8d3d514ad9c5bd7ad5cf21ff09f34d"
ORIGINAL_SOURCE_AUDIT, ORIGINAL_ANALYZE = inherited.source_audit, inherited.analyze_level


def source_audit(trial, plan):
    with owner.configured_owner(), owner.inherited.configured_owner():
        return ORIGINAL_SOURCE_AUDIT(trial, plan)


def analyze_level(directory):
    summary = ORIGINAL_ANALYZE(directory)
    raw = json.loads((directory / "result.json").read_text())
    summary["effective_target_speed_kmh"] = raw.get("effective_target_speed_kmh")
    if raw.get("effective_target_speed_kmh") != 5.688:
        summary["evidence_failures"].append({"kind": "ACTUAL_PID_TARGET_NOT_VERIFIED"})
    rows = inherited.helpers.rows(directory / "native_frames.jsonl")
    by_frame = {row["frame"]: row for row in rows}
    pulse = summary.get("pulse_measurement")
    if pulse:
        entry_frame = pulse["entry"]["frame"]
        actual_entry = []
        for frame in range(entry_frame - 9, entry_frame + 1):
            row = by_frame.get(frame)
            if row is None:
                summary["evidence_failures"].append({"kind": "ENTRY_FRAME_MISSING", "frame": frame})
                continue
            vx, vy, _ = row["world_velocity_carla"]
            speed = (vx * vx + vy * vy) ** .5
            actual_entry.append({"frame": frame, "actual_actor_planar_speed_mps": speed})
            if not 1.45 <= speed <= 1.55:
                summary["evidence_failures"].append({"kind": "ENTRY_OUTSIDE_FIXED_BAND", "frame": frame})
        pulse.update(actual_ten_tick_entry=actual_entry, qualification_speed_scope_mps=[1.45, 1.55],
            higher_speed_extrapolation_approved=False,
            normal_coast_baseline=summary["brake_level"] == 0.,
            completion_reason=raw.get("completion"))
    return summary


@contextmanager
def configured_audit():
    old = inherited.worker, inherited.WORKER_SHA256, inherited.source_audit, inherited.analyze_level
    try:
        inherited.worker, inherited.WORKER_SHA256 = worker, WORKER_SHA256
        inherited.source_audit, inherited.analyze_level = source_audit, analyze_level
        yield
    finally:
        inherited.worker, inherited.WORKER_SHA256, inherited.source_audit, inherited.analyze_level = old


def main(argv=None):
    with configured_audit():
        return inherited.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
