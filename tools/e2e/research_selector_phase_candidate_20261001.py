#!/usr/bin/env python3
"""HH_261001 - Export and bind one selector-only candidate to the unchanged bounded CARLA test."""

import argparse
from contextlib import contextmanager
import hashlib
import json
from pathlib import Path

from portable_e2e import simulation_research as research
from scripts.e2e.research_initial_acceleration_candidate_20261001 import validate_identity


PARENT_SHA256 = '4841d4076fb49145643ab2895d87befcae9021ad02f08b8307e75f77be137d91'
SCHEMA = 'portable_e2e.selector_phase_ab_20261001.v1'
ARMS = ('S0_uniform_selector', 'S1_phase_balanced_selector')
PLAN = Path(__file__).resolve().parents[2] / 'docs/assets/validation/2026-10-01/selector_phase_ab/LIVE_PLAN.md'


@contextmanager
def identity(checkpoint_sha256, collector=None):
    # HH_261001 - Only this process is rebound; old checkpoints and owner source files are immutable.
    validate_identity(checkpoint_sha256)
    original = research.AH_CHECKPOINT_SHA256, research.CHECKPOINT_SCHEMA
    old_collector = collector.AH_CHECKPOINT_SHA256 if collector is not None else None
    try:
        research.AH_CHECKPOINT_SHA256, research.CHECKPOINT_SCHEMA = checkpoint_sha256, SCHEMA
        if collector is not None:
            collector.AH_CHECKPOINT_SHA256 = checkpoint_sha256
        yield
    finally:
        research.AH_CHECKPOINT_SHA256, research.CHECKPOINT_SCHEMA = original
        if collector is not None:
            collector.AH_CHECKPOINT_SHA256 = old_collector


def validate_payload(payload, arm):
    if (arm not in ARMS or payload.get('arm') != arm or payload.get('checkpoint_schema') != SCHEMA
            or payload.get('parent_checkpoint_sha256') != PARENT_SHA256
            or payload.get('fit', {}).get('global_step') != 3000):
        raise ValueError('fixed selector-only checkpoint identity differs')


def assert_frozen_generator(payload, parent):
    # HH_261001 - Recompute tensor bytes against the original AH, not merely a success flag.
    from scripts.e2e.selector_phase_research_20261001 import generator_state
    from scripts.e2e.run_moving_multiroute_recovery_k import tensor_digest
    expected = tensor_digest(generator_state(parent['model_state_dict']))
    actual = tensor_digest(generator_state(payload['model_state_dict']))
    if (actual != expected or payload.get('frozen_generator_state_sha256') != expected
            or payload.get('final_model_state_sha256') != tensor_digest(payload['model_state_dict'])
            or payload.get('fit', {}).get('samples_seen') != 12000):
        raise ValueError('frozen AH generator tensor identity differs')
    return expected


def export(args):
    import torch
    research._pinned_bytes(args.checkpoint, args.checkpoint_sha256)
    payload, actual = research._read_checkpoint_file(args.checkpoint, torch.device('cpu'))
    if actual != args.checkpoint_sha256:
        raise ValueError('candidate checkpoint bytes changed')
    validate_payload(payload, args.arm)
    research._pinned_bytes(args.parent_checkpoint, PARENT_SHA256)
    parent, parent_sha = research._read_checkpoint_file(args.parent_checkpoint, torch.device('cpu'))
    if parent_sha != PARENT_SHA256:
        raise ValueError('original AH parent bytes differ')
    frozen_sha = assert_frozen_generator(payload, parent)
    with identity(actual):
        result = research.export_ah_weights(args.checkpoint, args.weights)
    return dict(result, arm=args.arm, parent_checkpoint_sha256=PARENT_SHA256,
                training_steps=3000, frozen_generator_state_sha256=frozen_sha,
                vehicle_control_approved=False)


def trial(args):
    from scripts.e2e import run_owned_ah_town07_research_20261001 as owner
    from scripts.e2e import run_owned_ah_town07_research_v2_20261001 as outcome
    from scripts.e2e import collect_ah_simulation_research as collector
    for field in ('metadata', 'weights_sha256', 'metadata_sha256', 'live_plan_sha256'):
        if getattr(args, field) is None:
            raise ValueError('exact exported arrays, metadata and preregistered live plan hashes required')
    if PLAN.is_symlink() or hashlib.sha256(PLAN.read_bytes()).hexdigest() != args.live_plan_sha256:
        raise ValueError('live comparison plan differs')
    old_weights, old_metadata = owner.WEIGHTS, owner.METADATA
    owner.WEIGHTS, owner.METADATA = args.weights.resolve(), args.metadata.resolve()
    owner.PINS = {p: sha for p, sha in owner.PINS.items() if p not in (old_weights, old_metadata)}
    owner.PINS.update({owner.WEIGHTS: args.weights_sha256, owner.METADATA: args.metadata_sha256,
                       PLAN: args.live_plan_sha256})
    owner.OUTPUT = owner.ROOT / 'storage/experiments/validation/2026-10-01' / ('selector_phase_' + args.arm + '_town07_v1')
    original_command = owner.collector_command

    def command():
        argv = original_command()
        module = argv.index('scripts.e2e.collect_ah_simulation_research')
        argv[module] = 'scripts.e2e.research_selector_phase_candidate_20261001'
        return argv[:module + 2] + ['--mode', 'worker', '--arm', args.arm,
            '--checkpoint-sha256', args.checkpoint_sha256, '--weights', str(owner.WEIGHTS)] + argv[module + 2:]

    owner.collector_command = command
    with identity(args.checkpoint_sha256, collector):
        research.load_ah_weights(owner.WEIGHTS, owner.METADATA,
            weights_sha256=args.weights_sha256, metadata_sha256=args.metadata_sha256)
        plan = owner.preflight()
        plan.update(schema='portable_e2e.owned_selector_phase_town07.v1', arm=args.arm,
            source_checkpoint_sha256=args.checkpoint_sha256, parent_checkpoint_sha256=PARENT_SHA256,
            source_checkpoint_schema=SCHEMA, live_comparison_plan_sha256=args.live_plan_sha256,
            comparison='same AH v2 route, 20 s budget, 9 km/h cap, raw-argmax tracker and gates')
        if args.mode == 'preflight':
            if owner.OUTPUT.exists():
                raise ValueError('fresh candidate trial output required')
            return plan
        owner.execute(plan)
        result = outcome.final_report(owner.OUTPUT)
        result.update(arm=args.arm, source_checkpoint_sha256=args.checkpoint_sha256,
                      parent_checkpoint_sha256=PARENT_SHA256)
        owner.write_json(owner.OUTPUT / 'owner_diagnostic_result.json', result)
        return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--mode', choices=('export', 'preflight', 'execute', 'worker'), required=True)
    parser.add_argument('--arm', choices=ARMS, required=True)
    parser.add_argument('--checkpoint-sha256', required=True)
    parser.add_argument('--checkpoint', type=Path)
    parser.add_argument('--parent-checkpoint', type=Path)
    parser.add_argument('--weights', type=Path, required=True)
    parser.add_argument('--metadata', type=Path)
    parser.add_argument('--weights-sha256')
    parser.add_argument('--metadata-sha256')
    parser.add_argument('--live-plan-sha256')
    args, remaining = parser.parse_known_args(argv)
    validate_identity(args.checkpoint_sha256)
    if args.mode == 'worker':
        from scripts.e2e import collect_ah_simulation_research as collector
        with identity(args.checkpoint_sha256, collector):
            return collector.main(remaining)
    if remaining:
        parser.error('unexpected arguments')
    if args.mode == 'export' and (args.checkpoint is None or args.parent_checkpoint is None):
        parser.error('export requires candidate and original parent checkpoints')
    result = export(args) if args.mode == 'export' else trial(args)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if args.mode != 'execute' or result['status'] in ('DIAGNOSTIC_BUDGET_REACHED', 'DIAGNOSTIC_CAPTURE_COMPLETE') else 1


if __name__ == '__main__':
    raise SystemExit(main())
