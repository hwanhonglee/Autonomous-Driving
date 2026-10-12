#!/usr/bin/env python3
"""HH_261001 - Bind full-model warmup-supervision endpoints to the unchanged CARLA diagnostic."""

import argparse
from contextlib import contextmanager
import hashlib
import json
from pathlib import Path

from portable_e2e import simulation_research as research
from scripts.e2e.research_initial_acceleration_candidate_20261001 import validate_identity
from scripts.e2e.capture_exact_simulation_inputs_20261001 import exact_tensor_archive


PARENT_SHA256 = '4841d4076fb49145643ab2895d87befcae9021ad02f08b8307e75f77be137d91'
SCHEMA = 'portable_e2e.warmup_supervision_ab_20261001.v1'
ARMS = ('W0_full_supervision', 'W1_warmup_anchor_off')
ROOT = Path(__file__).resolve().parents[2]
PLAN = ROOT / 'docs/assets/validation/2026-10-01/warmup_supervision_ab/LIVE_PLAN.md'
ARCHIVE_PLAN = PLAN.with_name('EXACT_INPUT_CAPTURE_ADDENDUM.md')
ARCHIVE_PLAN_SHA = '55d3638647c8f3d1926531687501ff76a67a4c6434b51e63f2462459893b70bd'


@contextmanager
def identity(checkpoint_sha256, collector=None):
    # HH_261001 - Rebind only this process; original AH sources and evidence stay unchanged.
    validate_identity(checkpoint_sha256)
    original = research.AH_CHECKPOINT_SHA256, research.CHECKPOINT_SCHEMA
    previous = collector.AH_CHECKPOINT_SHA256 if collector is not None else None
    try:
        research.AH_CHECKPOINT_SHA256, research.CHECKPOINT_SCHEMA = checkpoint_sha256, SCHEMA
        if collector is not None:
            collector.AH_CHECKPOINT_SHA256 = checkpoint_sha256
        yield
    finally:
        research.AH_CHECKPOINT_SHA256, research.CHECKPOINT_SCHEMA = original
        if collector is not None:
            collector.AH_CHECKPOINT_SHA256 = previous


def validate_payload(payload, arm, training_plan_sha256):
    # HH_261001 - Scheduled batches and actual updates differ because both arms skip one batch.
    fit = payload.get('fit', {})
    expected = dict(global_step=2999, scheduled_steps=3000, samples_seen=11996,
                    scheduled_samples=12000, skipped_schedule_steps_one_based=[1693],
                    active_supervision_exposures=11996 if arm == ARMS[0] else 11549)
    if (arm not in ARMS or payload.get('arm') != arm or payload.get('checkpoint_schema') != SCHEMA
            or payload.get('parent_checkpoint_sha256') != PARENT_SHA256
            or payload.get('training_plan_sha256') != training_plan_sha256
            or any(fit.get(key) != value for key, value in expected.items())):
        raise ValueError('fixed full-model warmup-supervision checkpoint contract differs')


def validate_tensor_provenance(payload, parent):
    from scripts.e2e.run_moving_multiroute_recovery_k import tensor_digest
    initial = tensor_digest(parent['model_state_dict'])
    final = tensor_digest(payload['model_state_dict'])
    if (payload.get('initial_model_state_sha256') != initial
            or payload.get('final_model_state_sha256') != final or final == initial):
        raise ValueError('actual parent or trained endpoint tensor identity differs')
    return final


def export(args):
    import torch
    research._pinned_bytes(args.checkpoint, args.checkpoint_sha256)
    payload, actual = research._read_checkpoint_file(args.checkpoint, torch.device('cpu'))
    if actual != args.checkpoint_sha256:
        raise ValueError('checkpoint bytes changed')
    validate_payload(payload, args.arm, args.training_plan_sha256)
    research._pinned_bytes(args.parent_checkpoint, PARENT_SHA256)
    parent, parent_sha = research._read_checkpoint_file(args.parent_checkpoint, torch.device('cpu'))
    if parent_sha != PARENT_SHA256:
        raise ValueError('original AH parent bytes differ')
    final = validate_tensor_provenance(payload, parent)
    with identity(actual):
        result = research.export_ah_weights(args.checkpoint, args.weights)
    return dict(result, arm=args.arm, parent_checkpoint_sha256=PARENT_SHA256,
                actual_optimizer_updates=2999, scheduled_steps=3000,
                training_plan_sha256=args.training_plan_sha256, final_model_state_sha256=final,
                vehicle_control_approved=False)


def trial(args):
    from scripts.e2e import run_owned_ah_town07_research_20261001 as owner
    from scripts.e2e import run_owned_ah_town07_research_v2_20261001 as outcome
    from scripts.e2e import collect_ah_simulation_research as collector
    for field in ('metadata', 'weights_sha256', 'metadata_sha256', 'live_plan_sha256'):
        if getattr(args, field) is None:
            raise ValueError('exact exported arrays, metadata and live plan hashes required')
    if PLAN.is_symlink() or hashlib.sha256(PLAN.read_bytes()).hexdigest() != args.live_plan_sha256:
        raise ValueError('preregistered live comparison plan differs')
    old_weights, old_metadata = owner.WEIGHTS, owner.METADATA
    owner.WEIGHTS, owner.METADATA = args.weights.resolve(), args.metadata.resolve()
    owner.PINS = {p: sha for p, sha in owner.PINS.items() if p not in (old_weights, old_metadata)}
    owner.PINS.update({owner.WEIGHTS: args.weights_sha256, owner.METADATA: args.metadata_sha256,
                       PLAN: args.live_plan_sha256, ARCHIVE_PLAN: ARCHIVE_PLAN_SHA})
    owner.OUTPUT = owner.ROOT / 'storage/experiments/validation/2026-10-01' / ('warmup_supervision_' + args.arm + '_town07_v1')
    original_command = owner.collector_command

    def command():
        argv = original_command()
        module = argv.index('scripts.e2e.collect_ah_simulation_research')
        argv[module] = 'scripts.e2e.research_warmup_supervision_candidate_20261001'
        return argv[:module + 2] + ['--mode', 'worker', '--arm', args.arm,
            '--checkpoint-sha256', args.checkpoint_sha256, '--weights', str(owner.WEIGHTS)] + argv[module + 2:]

    owner.collector_command = command
    with identity(args.checkpoint_sha256, collector):
        research.load_ah_weights(owner.WEIGHTS, owner.METADATA,
            weights_sha256=args.weights_sha256, metadata_sha256=args.metadata_sha256)
        plan = owner.preflight()
        plan.update(schema='portable_e2e.owned_warmup_supervision_town07.v1', arm=args.arm,
            source_checkpoint_sha256=args.checkpoint_sha256, parent_checkpoint_sha256=PARENT_SHA256,
            source_checkpoint_schema=SCHEMA, live_comparison_plan_sha256=args.live_plan_sha256,
            exact_input_capture_addendum_sha256=ARCHIVE_PLAN_SHA,
            comparison='same AH v2 route, 20 s budget, 9 km/h cap, raw argmax tracker and gates')
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
    parser.add_argument('--training-plan-sha256')
    parser.add_argument('--weights', type=Path, required=True)
    parser.add_argument('--metadata', type=Path)
    parser.add_argument('--weights-sha256')
    parser.add_argument('--metadata-sha256')
    parser.add_argument('--live-plan-sha256')
    args, remaining = parser.parse_known_args(argv)
    validate_identity(args.checkpoint_sha256)
    if args.mode == 'worker':
        from scripts.e2e import collect_ah_simulation_research as collector
        directory = ROOT / 'storage/experiments/validation/2026-10-01' / (
            'warmup_supervision_' + args.arm + '_town07_v1') / 'exact_runtime_inputs'
        # HH_261001 - Both live arms retain actual forward inputs; the original policy is unchanged.
        with identity(args.checkpoint_sha256, collector), exact_tensor_archive(directory):
            return collector.main(remaining)
    if remaining:
        parser.error('unexpected arguments')
    if args.mode == 'export' and any(getattr(args, field) is None for field in
                                   ('checkpoint', 'parent_checkpoint', 'training_plan_sha256')):
        parser.error('export requires candidate, original parent and exact training plan hash')
    result = export(args) if args.mode == 'export' else trial(args)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if args.mode != 'execute' or result['status'] in ('DIAGNOSTIC_BUDGET_REACHED', 'DIAGNOSTIC_CAPTURE_COMPLETE') else 1


if __name__ == '__main__':
    raise SystemExit(main())
