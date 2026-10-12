#!/usr/bin/env python3
"""HH_261001 - Own one slower Town12 capture with the explicitly revised stop profile."""

import argparse
from contextlib import contextmanager
import json
from pathlib import Path
import subprocess
import sys

from scripts.e2e import run_owned_town12_j15824_spawn_lift_v2_20261001 as inherited
from scripts.e2e import capture_town12_j15824_expert9_v2stop_20261001 as worker


ROOT = inherited.inherited.ROOT
OUTPUT = ROOT / "storage/experiments/validation/2026-10-01/town12_j15824_signal_expert9_v2stop_v1"
WORKER = ROOT / "tools/e2e/capture_town12_j15824_expert9_v2stop_20261001.py"
FAILED_TWENTY = inherited.OUTPUT / "owner_result.json"
FAILED_TWENTY_SHA256 = "56c1d53092f342c061dfd873bd97e39de58364e35902a662c13b64127bc94498"
CLEANUP_OBSERVATION = ROOT / "storage/experiments/validation/2026-10-01/town12_j15824_signal_expert20_spawn05_v2_offlineqa_v1/independent_cleanup_observation.json"
CLEANUP_SHA256 = "ca966a11a76519ff28107684df3a17e384bc7b6719f228ff24cba28fc9253b09"
SOURCE_FILES = (*inherited.SOURCE_FILES,
    "tools/e2e/run_owned_town12_j15824_expert9_v2stop_20261001.py",
    "tools/e2e/capture_town12_j15824_expert9_v2stop_20261001.py",
    "learning/tests/test_town12_j15824_expert9_v2stop_20261001.py",
    "tools/e2e/audit_town12_j15824_capture_20261001.py",
    "tools/e2e/audit_town12_partial_body_forensics_20261001.py",
    str(FAILED_TWENTY.relative_to(ROOT)), str(CLEANUP_OBSERVATION.relative_to(ROOT)),
    "storage/experiments/validation/2026-10-01/town12_j15824_signal_expert20_spawn05_v2_body_forensics_v1/summary.json")


def protocol_record():
    return {"variant_id": "town12_j15824_expert9_comfortable_v2_180s_v1",
        "target_speed_kmh": 9.0, "max_duration_sec": 180.0, "goal_stop_profile": "comfortable_v2",
        "changed_factors": ["target_speed_kmh:20->9", "goal_stop_profile:comfortable_v1->comfortable_v2", "max_duration_sec:150->180"],
        "single_factor_causal_ab": False,
        "reason": "20 km/h measured lane intrusions/lateral target violations and goal overshoot; original cap0.10 produced about0.96m/s2 deceleration versus v1 assumed1.5.",
        "stop_parameters": {"desired_deceleration_mps2": 1.0, "normal_brake_cap": .15, "normal_throttle_cap": .20},
        "unchanged": ["exact route and full221.76m extent", "20Hz physics/10Hz six-camera geometry", "+0.50m spawn lift", "observed controlling signals and normal emergency braking", "all numeric runtime/body/sweep gates"],
        "thirty_kmh_qualification_claimed": False, "training_data_admitted": False}


@contextmanager
def configured_owner():
    lift_owner = inherited.inherited
    capture_owner = lift_owner.inherited
    owner = capture_owner.scout_owner.diagnostic.owner
    previous = inherited.OUTPUT, inherited.WORKER, inherited.SOURCE_FILES
    old_write = owner._write_new_json

    def write(path, value):
        # HH_261001 - This innermost writer sees the fully decorated parent plan and corrects its literal20 label.
        if path.name == "owner_plan.json":
            trial = {**value["signal_expert_trial"], "target_speed_kmh": 9.0,
                     "combined_protocol_revision": protocol_record()}
            value = {**value, "signal_expert_trial": trial}
        elif path.name == "owner_result.json":
            value = {**value, "combined_protocol_revision": protocol_record()}
        old_write(path, value)

    try:
        inherited.OUTPUT, inherited.WORKER, inherited.SOURCE_FILES = OUTPUT, WORKER, SOURCE_FILES
        owner._write_new_json = write
        with worker.configured_variant(), inherited.configured_owner(), lift_owner.configured_owner(), capture_owner.configured_owner():
            yield owner
    finally:
        owner._write_new_json = old_write
        inherited.OUTPUT, inherited.WORKER, inherited.SOURCE_FILES = previous


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args(argv)
    scout = worker.inherited.inherited.scout
    try:
        scout.require(scout.digest(FAILED_TWENTY) == FAILED_TWENTY_SHA256
            and scout.digest(CLEANUP_OBSERVATION) == CLEANUP_SHA256, "prior failed raw trial or later cleanup proof changed")
        cleanup = json.loads(CLEANUP_OBSERVATION.read_text())
        scout.require(cleanup["status"] == "PASS_CURRENTLY_EXTINCT" and cleanup["remaining_group_members"] == []
            and cleanup["bound_tcp_udp_port_rows"] == [], "later independent process/port extinction proof absent")
        with configured_owner() as owner:
            prepared = owner.preflight()
            if not args.execute:
                absent = not (OUTPUT.exists() or OUTPUT.is_symlink())
                print(json.dumps({"status": "STATIC_READY_NOT_LIVE_TESTED" if absent else "STOP_OUTPUT_EXISTS",
                    "output": str(OUTPUT), "output_absent": absent, "simulator_started": False,
                    "capture_on_execute": True, "protocol": protocol_record()}, sort_keys=True))
                return 0 if absent else 2
            return owner.execute(*prepared)
    except (scout.ScoutError, OSError, ValueError, KeyError, subprocess.SubprocessError,
            inherited.inherited.inherited.scout_owner.diagnostic.owner.Town12OwnerError,
            inherited.inherited.inherited.scout_owner.diagnostic.owner.shared_owner.CatalogOwnerError) as error:
        print(f"Town12 9kmh revised-stop preflight failed: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
