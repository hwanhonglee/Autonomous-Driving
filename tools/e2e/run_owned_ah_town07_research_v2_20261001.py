#!/usr/bin/env python3
"""HH_261001 - Repeat the fixed AH diagnostic with exact-frame handoff and separate outcomes."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from . import run_owned_ah_town07_research_20261001 as baseline


OUTPUT = baseline.ROOT / "storage/experiments/validation/2026-10-01/ah_town07_straight_research_v2"
BUDGET_ERROR = ("CollectionError: BasicAgent did not finish within the driving budget reserved by "
                "the maximum total duration and stationary tail")


def classify_bounded_capture(manifest, state_count, camera_count):
    """HH_261001 - Recognize the exact 400-tick diagnostic budget without declaring route success."""
    contract = manifest.get("capture_contract", {})
    return (manifest.get("status") == "failed" and manifest.get("error") == BUDGET_ERROR
            and state_count == 400 and camera_count == 200
            and contract.get("physics_hz") == 20.0 and contract.get("camera_hz") == 10.0
            and contract.get("maximum_total_duration_sec") == 20.0)


def final_report(output):
    """HH_261001 - Keep port release, source identity, capture and driving outcomes distinct."""
    owner = json.loads((output / "owner_result.json").read_text())
    plan = json.loads((output / "owner_plan.json").read_text())
    errors = []
    try:
        baseline.ports_free(baseline.PORT)
        released = True
    except OSError as error:
        released = False
        errors.append(str(error))
    source_match = all(baseline.digest(baseline.ROOT / path) == digest
                       and baseline.digest(output / "source" / path) == digest
                       for path, digest in plan["source_sha256"].items())
    episode = output / "episode.partial"
    if not episode.exists():
        episode = output / "episode"
    manifest_path = episode / "manifest.json"
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    state_path, camera_path = episode / "states.jsonl", episode / "camera_frames.jsonl"
    states = sum(1 for _ in state_path.open()) if state_path.exists() else 0
    cameras = sum(1 for _ in camera_path.open()) if camera_path.exists() else 0
    budget = classify_bounded_capture(manifest, states, cameras)
    summary_path = episode / "learned_summary.json"
    learned = json.loads(summary_path.read_text()) if summary_path.exists() else None
    owner_ok = released and source_match and not owner["cleanup_errors"]
    status = ("OWNER_FAILURE" if not owner_ok else "DIAGNOSTIC_BUDGET_REACHED" if budget else
              "CAPTURE_FAILED_RETAINED" if owner.get("error") else "DIAGNOSTIC_CAPTURE_COMPLETE")
    return dict(schema="portable_e2e.owned_ah_town07_outcome.v2", status=status,
        port_triplet_released=released, source_hashes_match=source_match,
        owner_cleanup_errors=owner["cleanup_errors"], port_check_errors=errors,
        original_owner_error=owner.get("error"), original_collector_exit_code=owner["collector_exit_code"],
        original_capture_error=manifest.get("error"), state_count=states, camera_count=cameras,
        learned=learned, goal_reached=manifest.get("result", {}).get("goal_reached", False),
        learned_driving_success_claimed=False, training_data_approved=False, vehicle_control_approved=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    # HH_261001 - Configure this process only; preserve the original v1 owner source and evidence.
    baseline.OUTPUT = OUTPUT
    plan = baseline.preflight()
    plan.update(schema="portable_e2e.owned_ah_town07_plan.v2",
                handoff="first vx>0.5 frame with ten history samples and matching camera frame",
                unchanged_model_tracker_gate=True)
    if not args.execute:
        print(json.dumps(plan, indent=2))
        return 0
    baseline.execute(plan)
    report = final_report(OUTPUT)
    baseline.write_json(OUTPUT / "owner_diagnostic_result.json", report)
    print(json.dumps(report, indent=2))
    return 0 if report["status"] in ("DIAGNOSTIC_BUDGET_REACHED", "DIAGNOSTIC_CAPTURE_COMPLETE") else 1


if __name__ == "__main__":
    raise SystemExit(main())
