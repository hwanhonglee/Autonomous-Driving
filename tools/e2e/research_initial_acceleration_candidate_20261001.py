#!/usr/bin/env python3
"""HH_261001 - Pin one new acceleration A/B candidate without changing AH evidence."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
from pathlib import Path
import re
import sys

from portable_e2e import simulation_research as research


PARENT_SHA256 = "4841d4076fb49145643ab2895d87befcae9021ad02f08b8307e75f77be137d91"
CHECKPOINT_SCHEMA = "portable_e2e.initial_acceleration_ab_20261001.v1"
ARMS = ("A_ah_continuation", "B_preclamp_acceleration")
LIVE_PLAN_SHA256 = "52223f09c12c56081de8409332497786c3c090f45b1a69ceece4c9bc2e549bcf"


def validate_identity(checkpoint_sha256):
    if (not isinstance(checkpoint_sha256, str)
            or re.fullmatch(r"[0-9a-f]{64}", checkpoint_sha256) is None
            or checkpoint_sha256 == PARENT_SHA256):
        raise ValueError("a distinct exact new checkpoint SHA256 is required")


@contextmanager
def candidate_identity(checkpoint_sha256, collector=None):
    """HH_261001 - Rebind this process only, including every emitted control provenance field."""
    validate_identity(checkpoint_sha256)
    old = research.AH_CHECKPOINT_SHA256, research.CHECKPOINT_SCHEMA
    old_collector = collector.AH_CHECKPOINT_SHA256 if collector is not None else None
    try:
        research.AH_CHECKPOINT_SHA256 = checkpoint_sha256
        research.CHECKPOINT_SCHEMA = CHECKPOINT_SCHEMA
        if collector is not None:
            collector.AH_CHECKPOINT_SHA256 = checkpoint_sha256
        yield
    finally:
        research.AH_CHECKPOINT_SHA256, research.CHECKPOINT_SCHEMA = old
        if collector is not None:
            collector.AH_CHECKPOINT_SHA256 = old_collector


def validate_checkpoint(payload, arm):
    if (arm not in ARMS or payload.get("arm") != arm
            or payload.get("checkpoint_schema") != CHECKPOINT_SCHEMA
            or payload.get("parent_checkpoint_sha256") != PARENT_SHA256
            or payload.get("fit", {}).get("global_step") != 3000):
        raise ValueError("fixed 3000-step AH-parent paired candidate contract differs")


def export_candidate(args):
    import torch
    validate_identity(args.checkpoint_sha256)
    research._pinned_bytes(args.checkpoint, args.checkpoint_sha256)
    payload, actual = research._read_checkpoint_file(args.checkpoint, torch.device("cpu"))
    if actual != args.checkpoint_sha256:
        raise ValueError("candidate checkpoint bytes changed")
    validate_checkpoint(payload, args.arm)
    with candidate_identity(args.checkpoint_sha256):
        result = research.export_ah_weights(args.checkpoint, args.weights)
    return {**result, "arm": args.arm, "parent_checkpoint_sha256": PARENT_SHA256,
            "training_steps": 3000, "vehicle_control_approved": False}


def own_trial(args):
    from scripts.e2e import run_owned_ah_town07_research_20261001 as owner
    from scripts.e2e import run_owned_ah_town07_research_v2_20261001 as repeat
    from scripts.e2e import collect_ah_simulation_research as collector

    # HH_261001 - The simulator, route, tracking gains and safety gate stay identical to AH v2.
    for field in ("metadata", "weights_sha256", "metadata_sha256"):
        if getattr(args, field) is None:
            raise ValueError("exact exported metadata and array hashes are required")
    old_weights, old_metadata = owner.WEIGHTS, owner.METADATA
    owner.WEIGHTS, owner.METADATA = args.weights.resolve(), args.metadata.resolve()
    owner.PINS = {path: sha for path, sha in owner.PINS.items()
                  if path not in (old_weights, old_metadata)}
    owner.PINS.update({owner.WEIGHTS: args.weights_sha256, owner.METADATA: args.metadata_sha256})
    live_plan = owner.ROOT / ("docs/assets/validation/2026-10-01/ah_town07_straight_research_v2/"
                              "CANDIDATE_LIVE_COMPARISON_PLAN.md")
    owner.PINS[live_plan] = LIVE_PLAN_SHA256
    owner.OUTPUT = owner.ROOT / "storage/experiments/validation/2026-10-01" / (
        "initial_acceleration_" + args.arm + "_town07_v1")
    original_command = owner.collector_command

    def command():
        argv = original_command()
        module = argv.index("scripts.e2e.collect_ah_simulation_research")
        argv[module] = "scripts.e2e.research_initial_acceleration_candidate_20261001"
        insertion = module + 2  # After the workspace launcher's literal -- separator.
        return argv[:insertion] + ["--mode", "worker", "--arm", args.arm,
            "--checkpoint-sha256", args.checkpoint_sha256,
            "--weights", str(owner.WEIGHTS)] + argv[insertion:]

    owner.collector_command = command
    with candidate_identity(args.checkpoint_sha256, collector):
        # HH_261001 - Verify data-only arrays before starting any CARLA process.
        research.load_ah_weights(owner.WEIGHTS, owner.METADATA,
            weights_sha256=args.weights_sha256, metadata_sha256=args.metadata_sha256)
        plan = owner.preflight()
        plan.update(schema="portable_e2e.owned_initial_acceleration_town07.v1",
                    arm=args.arm, source_checkpoint_sha256=args.checkpoint_sha256,
                    parent_checkpoint_sha256=PARENT_SHA256,
                    source_checkpoint_schema=CHECKPOINT_SCHEMA,
                    live_comparison_plan_sha256=LIVE_PLAN_SHA256,
                    comparison="same AH v2 route, 20 s budget, 9 km/h cap, tracker and gates",
                    capture_budget_seconds=20, new_training_data_approved=False)
        if args.mode == "preflight":
            if owner.OUTPUT.exists():
                raise ValueError("fresh candidate trial output is required")
            return plan
        owner.execute(plan)
        result = repeat.final_report(owner.OUTPUT)
        result.update(arm=args.arm, source_checkpoint_sha256=args.checkpoint_sha256,
                      parent_checkpoint_sha256=PARENT_SHA256)
        owner.write_json(owner.OUTPUT / "owner_diagnostic_result.json", result)
        return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--mode", choices=("export", "preflight", "execute", "worker"), required=True)
    parser.add_argument("--arm", choices=ARMS, required=True)
    parser.add_argument("--checkpoint-sha256", required=True)
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--weights", type=Path, required=True)
    parser.add_argument("--metadata", type=Path)
    parser.add_argument("--weights-sha256")
    parser.add_argument("--metadata-sha256")
    args, remaining = parser.parse_known_args(argv)
    validate_identity(args.checkpoint_sha256)
    if args.mode == "worker":
        from scripts.e2e import collect_ah_simulation_research as collector
        with candidate_identity(args.checkpoint_sha256, collector):
            return collector.main(remaining)
    if remaining:
        parser.error("unexpected remaining arguments")
    if args.mode == "export" and args.checkpoint is None:
        parser.error("export requires --checkpoint")
    result = export_candidate(args) if args.mode == "export" else own_trial(args)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if args.mode != "execute" or result["status"] in (
        "DIAGNOSTIC_BUDGET_REACHED", "DIAGNOSTIC_CAPTURE_COMPLETE") else 1


if __name__ == "__main__":
    raise SystemExit(main())
