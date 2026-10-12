"""HH_261002 - Print a NEW LEFT20k research binding; never actuate or write exports.

Actual completion/endpoint SHAs must be supplied after ROOT's fixed final20k
publication. Export false flags and the original GO/2.5m/s model stay unchanged.
"""
from __future__ import annotations
import argparse
import json

from portable_e2e import temporal_left_joint_numeric_loader_20261002 as loader

SCHEMA = "portable_e2e.temporal_left_joint_runtime_contract.20261002.v1"


def contract():
    return dict(schema=SCHEMA, loader_schema=loader.SCHEMA, owner_schema=loader.OWNER_SCHEMA,
        completion_authority_schema=loader.AUTHORITY_SCHEMA, training_schema=loader.TRAINING_SCHEMA,
        export_schema=loader.EXPORT_SCHEMA, fixed_endpoint_updates=20000, known_total_updates=32200,
        state_tensors=101, learned_tensors=54, frozen_tensors=47, TRAIN_episodes=19,
        actual_input_exposures=80000, actual_LEFT_anchors=347, LEFT_conversion_sha256=loader.LEFT_CONVERSION_SHA,
        source_count=30, initial_GOOD_parent=loader.PARENT, actual_endpoint_and_journal_SHA_required=True,
        separate_prospective_private_owner_required_for_CARLA=True, current_GO_decoder_2p5mps_unchanged=True,
        export_false_flags_immutable=True, checkpoint_selection=False, optimization_or_inference_performed=False,
        model_promoted=False, vehicle_control_approved=False, official_MAIN9_approved=0, official_sub30_approved=0)


def owner_binding(pins, authority_sha, *, owner_id, purpose="offline_cpu_numeric_validation_only", owner_plan_sha256=None):
    loader.require(type(pins) is dict and set(pins) == set(loader.PARENT) and all(loader.is_sha(v) for v in pins.values())
        and loader.is_sha(authority_sha) and type(owner_id) is str and owner_id.strip(), "real external endpoint/completion pins and owner required")
    loader.require(purpose in ("offline_cpu_numeric_validation_only", "owned_carla_research_only"), "explicit research purpose required")
    control = purpose == "owned_carla_research_only"
    loader.require(loader.is_sha(owner_plan_sha256) if control else owner_plan_sha256 is None,
        "only a separate prospectively sealed private plan can authorize simulator control")
    result = dict(schema=loader.OWNER_SCHEMA, mode="RESEARCH_ONLY", owner_id=owner_id, purpose=purpose,
        fixed_endpoint_updates=20000, completed_run_authority_sha256=authority_sha, **pins,
        simulator_control_authorized=control, simulation_only=True, model_promoted=False, vehicle_control_approved=False)
    if control: result["owner_plan_sha256"] = owner_plan_sha256
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--mode", choices=("contract", "binding"), default="contract")
    for name in ("weights", "metadata", "checkpoint", "declaration", "completed-run-authority"):
        parser.add_argument("--" + name + "-sha256")
    parser.add_argument("--owner-id")
    parser.add_argument("--purpose", choices=("offline_cpu_numeric_validation_only", "owned_carla_research_only"),
        default="offline_cpu_numeric_validation_only")
    parser.add_argument("--owner-plan-sha256")
    args = parser.parse_args(argv)
    if args.mode == "contract":
        loader.require(not any(getattr(args, name + "_sha256") for name in
            ("weights", "metadata", "checkpoint", "declaration", "completed_run_authority"))
            and args.owner_id is None and args.owner_plan_sha256 is None
            and args.purpose == "offline_cpu_numeric_validation_only", "contract query cannot grant owner authority")
        result = contract()
    else:
        pins = {name + "_sha256": getattr(args, name + "_sha256") for name in ("weights", "metadata", "checkpoint", "declaration")}
        result = owner_binding(pins, args.completed_run_authority_sha256, owner_id=args.owner_id,
            purpose=args.purpose, owner_plan_sha256=args.owner_plan_sha256)
    print(json.dumps(result, indent=2, allow_nan=False)); return 0


if __name__ == "__main__": raise SystemExit(main())
