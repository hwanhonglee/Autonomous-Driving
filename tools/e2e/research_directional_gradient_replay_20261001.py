"""HH_261001 - One fixed A-then-B sparse20 pair using released directional-study endpoints."""

import argparse
from contextlib import contextmanager
import json
from pathlib import Path
import shutil

import torch
from scripts.e2e import research_warmup_duration_w2_replay_20261001 as prior
from scripts.e2e import report_directional_lower_gradient_ab_20261001 as proof

ROOT = prior.ROOT
TRAINING = ROOT / 'storage/experiments/training/2026-10-01/ah_directional_lower_gradient_ab_20261001_v1'
BINDING = TRAINING / 'exports/endpoint_binding.json'
BINDING_SHA = 'a6b674a471f2bec5c1c153d912a61fac5ecffae72ef797f6cbc9a86574efac8e'
PLAN = ROOT / 'docs/assets/validation/2026-10-01/selector_phase_ab/DIRECTIONAL_TOWN07_PAIRED_REPLAY_PREREGISTRATION.md'
PLAN_SHA = '57c0d77539702e49206427f701ba3181ea97b2218bbaf2a45e32068bfde9d0ba'
FREEZE = PLAN.with_name('DIRECTIONAL_REPLAY_SOURCE_FREEZE.json')
FREEZE_SHA = 'd2a8fae7537b45a18d8bc4e29c6a8ae7da9f2d9d61dc535b35a3480b2d071ed2'
TEST = ROOT / 'learning/tests/test_directional_gradient_replay_20261001.py'
NEW_FILES = (Path(__file__), PLAN, FREEZE, TEST)
CHECKPOINTS = {
    'A': '325e188ba0430bd4743d15857f8f44813f6b0de2db6b0fd780cd02bad03522b3',
    'B': 'a49ed9cfe0f4cfca970f21f9ac7cd9e20ebf8365b70330a3104b7714d988a37b',
}
STATES = {'A': prior.W1_STATE, 'B': '6964bc82fbf4eb5742f732aaa05e01967f0b8e69a177652312fa5e251e7cf0ab'}
SCHEMA = 'portable_e2e.directional_paired_sparse20_owner.v1'
PAIR_SCHEMA = 'portable_e2e.directional_paired_sparse20_attempts.v1'
MODULE = 'scripts.e2e.research_directional_gradient_replay_20261001'
require, digest, read_json, write_new = prior.require, prior.digest, prior.read_json, prior.write_new
research, short = prior.research, prior.short


def output(arm):
    require(arm in proof.LABELS, 'only fixed A/B replay arms are declared')
    return ROOT / ('storage/experiments/validation/2026-10-01/directional_gradient_' + arm + '_town07_sparse20_v1')


def source_pins():
    require(digest(PLAN) == PLAN_SHA and digest(FREEZE) == FREEZE_SHA, 'prospective protocol/source manifest differs')
    manifest = read_json(FREEZE)
    require(manifest.get('schema') == 'portable_e2e.directional_replay_runtime_source_freeze.v1',
            'exact runtime source freeze schema required')
    result = {PLAN: PLAN_SHA, FREEZE: FREEZE_SHA}
    for name, expected in manifest['source_sha256'].items():
        path = proof.evidence_relative(ROOT, name)
        require(digest(path) == expected, 'frozen runtime source changed: ' + name)
        result[path] = expected
    return result


def released_endpoints():
    """HH_261001 - Verify both fixed endpoints without consulting DEV/probe metric values."""
    pins = source_pins()
    require(digest(BINDING) == BINDING_SHA, 'fixed dual-export binding bytes differ')
    binding = read_json(BINDING)
    complete = read_json(TRAINING / 'both_final_checkpoints_complete.json')
    summary = read_json(TRAINING / 'summary.json')
    declaration = read_json(TRAINING / 'declaration.json')
    require(summary.get('status') == 'COMPLETE_NOT_PROMOTED' and summary.get('schema') == proof.SCHEMA
        and summary.get('training') == complete and summary.get('source_hashes_match') is True,
        'released completed matched pair required')
    proof.validate_export_binding(binding, complete)
    evidence = binding['evidence_file_sha256']
    require(len(evidence) == 42, 'all 42 restricted-export evidence files required')
    pins[BINDING] = BINDING_SHA
    for name, expected in evidence.items():
        path = proof.evidence_relative(TRAINING, name)
        require(digest(path) == expected, 'released export evidence changed: ' + name)
        pins[path] = expected
    sources = declaration['source_sha256']
    require(len(sources) == 34 and prior.json_digest(sources) == proof.CLOSURE_SHA,
            'fixed 34-source training closure differs')
    for name, expected in dict(sources, **proof.EXPORT_PINS).items():
        path = proof.evidence_relative(ROOT, name)
        require(digest(path) == expected, 'current training/export source changed: ' + name)
        pins[path] = expected
    endpoints = {}
    for label, arm in zip(proof.LABELS, proof.ARMS):
        require(not (TRAINING / arm / 'failure.json').exists(), 'failed endpoint cannot substitute for a final')
        item = dict(binding['endpoints'][arm], checkpoint_schema=proof.SCHEMA, training_arm=arm)
        require(item['checkpoint_sha256'] == CHECKPOINTS[label] and item['model_state_sha256'] == STATES[label],
                'predeclared final endpoint identity differs')
        for field in ('weights', 'metadata'):
            path = proof.evidence_relative(TRAINING, item[field])
            require(digest(path) == item[field + '_sha256'], 'strict numeric endpoint bytes differ')
            item[field] = str(path)
            pins[path] = item[field + '_sha256']
        endpoints[label] = item
    return endpoints, pins


@contextmanager
def identity(arm, checkpoint, collector=None):
    require(arm in CHECKPOINTS and checkpoint == CHECKPOINTS[arm], 'exact arm checkpoint, not W1/parent substitution, required')
    saved = research.AH_CHECKPOINT_SHA256, research.CHECKPOINT_SCHEMA
    old_collector = collector.AH_CHECKPOINT_SHA256 if collector is not None else None
    try:
        research.AH_CHECKPOINT_SHA256, research.CHECKPOINT_SCHEMA = checkpoint, proof.SCHEMA
        if collector is not None:
            collector.AH_CHECKPOINT_SHA256 = checkpoint
        yield
    finally:
        research.AH_CHECKPOINT_SHA256, research.CHECKPOINT_SCHEMA = saved
        if collector is not None:
            collector.AH_CHECKPOINT_SHA256 = old_collector


def load_endpoint(arm, item):
    with identity(arm, item['checkpoint_sha256'], short):
        model = research.load_ah_weights(Path(item['weights']), Path(item['metadata']),
            weights_sha256=item['weights_sha256'], metadata_sha256=item['metadata_sha256'])
        require(prior.tensor_digest(model.state_dict()) == STATES[arm] == item['model_state_sha256'],
                'actual strict NPZ tensor identity differs')
        return model


def validate_capture(args, route, arm):
    # HH_261001 - Use the unchanged W2 capture validator with only its output-directory resolver scoped.
    old = prior.output
    try:
        prior.output = output
        prior.validate_capture(args, route, arm)
    finally:
        prior.output = old


def source_unchanged(pins):
    require(all(digest(path) == expected for path, expected in pins.items()), 'pinned study/runtime/evidence bytes changed')


def prior_a_ready():
    folder = output('A')
    result = read_json(folder / 'owner_diagnostic_result.json')
    plan = read_json(folder / 'owner_plan.json')
    require(result.get('port_triplet_released') is True and result.get('source_hashes_match') is True
        and not result.get('owner_cleanup_errors') and plan.get('binding_sha256') == BINDING_SHA
        and plan.get('replay_plan_sha256') == PLAN_SHA and plan.get('schema') == SCHEMA
        and plan.get('arm') == 'A' and plan.get('source_checkpoint_sha256') == CHECKPOINTS['A']
        and result.get('arm') == 'A' and result.get('source_checkpoint_sha256') == CHECKPOINTS['A'],
        'fixed first A attempt with successful ownership/source cleanup required; progress is not selected')
    for name, expected in plan['source_sha256'].items():
        require(digest(ROOT / name) == expected == digest(folder / 'source' / name), 'A current/archive source changed')
    return result


def worker(args, remaining):
    endpoints, pins = released_endpoints()
    item = endpoints[args.arm]
    capture = short.collector.parse_args(remaining)
    capture.research_speed_cap_mps, capture.research_max_cte_m = 2.5, 1.
    validate_capture(capture, read_json(capture.route_file), args.arm)
    prior.disk_check()
    torch.set_num_threads(2)
    model = load_endpoint(args.arm, item)
    write_new(output(args.arm) / 'worker_endpoint_identity.json', dict(arm=args.arm,
        training_arm=item['training_arm'], checkpoint_sha256=item['checkpoint_sha256'],
        model_state_sha256=item['model_state_sha256'], binding_sha256=BINDING_SHA, replay_plan_sha256=PLAN_SHA,
        all_42_export_files_verified=True, all_34_current_training_sources_verified=True))
    try:
        with identity(args.arm, item['checkpoint_sha256'], short), prior.disk_guard(), \
                prior.w1.sampled_inputs(output(args.arm) / 'sampled_exact_runtime_inputs'):
            short.run(model, capture)
        return 0
    finally:
        source_unchanged(pins)


def pair_receipt():
    """HH_261001 - Two retained attempts are not necessarily two full captures or driving success."""
    _, pins = released_endpoints()
    rows = {}
    for arm in proof.LABELS:
        folder = output(arm)
        plan = read_json(folder / 'owner_plan.json')
        result = read_json(folder / 'owner_diagnostic_result.json')
        require(plan.get('schema') == SCHEMA and plan.get('arm') == arm
            and plan.get('binding_sha256') == BINDING_SHA and plan.get('replay_plan_sha256') == PLAN_SHA
            and plan.get('source_checkpoint_sha256') == CHECKPOINTS[arm]
            and result.get('arm') == arm and result.get('source_checkpoint_sha256') == CHECKPOINTS[arm],
            'both actual fixed-arm attempt receipts required')
        for name, expected in plan['source_sha256'].items():
            require(digest(ROOT / name) == expected == digest(folder / 'source' / name), 'paired source archive differs')
        rows[arm] = dict(status=result['status'], checkpoint_sha256=CHECKPOINTS[arm],
            owner_plan_sha256=digest(folder / 'owner_plan.json'),
            owner_diagnostic_sha256=digest(folder / 'owner_diagnostic_result.json'),
            state_count=result['state_count'], camera_count=result['camera_count'],
            cleanup_ok=result['port_triplet_released'] and result['source_hashes_match'] and not result['owner_cleanup_errors'])
    source_unchanged(pins)
    value = dict(schema=PAIR_SCHEMA, status='BOTH_FIXED_ATTEMPTS_RETAINED_NOT_PROMOTED',
        fixed_arm_order=['A', 'B'], arms=rows, attempt_count=2,
        two_full_20_second_captures=all(r['state_count'] == 400 and r['camera_count'] == 200 for r in rows.values()),
        both_cleanup_ok=all(r['cleanup_ok'] for r in rows.values()), binding_sha256=BINDING_SHA,
        replay_plan_sha256=PLAN_SHA, learned_driving_success_claimed=False,
        training_data_approved=False, vehicle_control_approved=False, model_promoted=False)
    write_new(output('B') / 'paired_attempt_receipt.json', value)
    return value


def trial(args):
    from scripts.e2e import run_owned_ah_town07_research_20261001 as owner
    from scripts.e2e import run_owned_ah_town07_research_v2_20261001 as outcome
    endpoints, pins = released_endpoints()
    item = endpoints[args.arm]
    reserve = prior.disk_check(2 if args.arm == 'A' else 1)
    if args.arm == 'B':
        prior_a_ready()
    require(not output(args.arm).exists() and not output(args.arm).is_symlink(), 'fresh fixed-arm output required; no retry')
    saved = (owner.OUTPUT, owner.WEIGHTS, owner.METADATA, owner.PINS, owner.collector_command,
             owner.write_json, owner.lifecycle._workspace_lock)
    old_weights, old_metadata = owner.WEIGHTS, owner.METADATA
    old_command, old_write, old_lock = owner.collector_command, owner.write_json, owner.lifecycle._workspace_lock
    owner.OUTPUT, owner.WEIGHTS, owner.METADATA = output(args.arm), Path(item['weights']), Path(item['metadata'])
    owner.PINS = {p: sha for p, sha in owner.PINS.items() if p not in (old_weights, old_metadata)}
    owner.PINS.update(pins)

    def command():
        argv = old_command()
        module = argv.index('scripts.e2e.collect_ah_simulation_research')
        rest = argv[argv.index(str(owner.OUTPUT / 'episode')):]
        return argv[:module] + [MODULE, '--', '--mode', 'worker', '--arm', args.arm] + rest

    def archive_write(path, value):
        if path == owner.OUTPUT / 'owner_plan.json':
            # HH_261001 - Archive pinned current code/docs, never duplicate the large training checkpoints.
            pinned_sources = tuple(p for p in pins if p.is_relative_to(ROOT)
                and p.relative_to(ROOT).parts[0] in ('tools', 'learning', 'docs'))
            for source in NEW_FILES + pinned_sources:
                name = str(source.relative_to(ROOT))
                target = owner.OUTPUT / 'source' / name
                if not target.exists():
                    target.parent.mkdir(parents=True, exist_ok=True)
                    with source.open('rb') as src, target.open('xb') as dest:
                        shutil.copyfileobj(src, dest)
                require(digest(target) == digest(source), 'new adapter source archive differs')
                value['source_sha256'][name] = digest(source)
            with BINDING.open('rb') as src, (owner.OUTPUT / 'endpoint_binding.json').open('xb') as dest:
                shutil.copyfileobj(src, dest)
        old_write(path, value)

    @contextmanager
    def reserved_lock():
        with old_lock():
            prior.disk_check(2 if args.arm == 'A' else 1)
            source_unchanged(pins)
            if args.arm == 'B':
                prior_a_ready()
            yield

    owner.collector_command, owner.write_json, owner.lifecycle._workspace_lock = command, archive_write, reserved_lock
    try:
        torch.set_num_threads(2)
        for label, endpoint in endpoints.items():
            load_endpoint(label, endpoint)
        with identity(args.arm, item['checkpoint_sha256'], short):
            plan = owner.preflight()
            argv = command()
            capture = short.collector.parse_args(argv[argv.index(str(owner.OUTPUT / 'episode')):])
            capture.research_speed_cap_mps, capture.research_max_cte_m = 2.5, 1.
            validate_capture(capture, read_json(owner.ROUTE), args.arm)
            plan.update(schema=SCHEMA, arm=args.arm, training_arm=item['training_arm'],
                source_checkpoint_sha256=item['checkpoint_sha256'], source_checkpoint_schema=proof.SCHEMA,
                model_state_sha256=item['model_state_sha256'], parent_checkpoint_sha256=prior.PARENT_SHA,
                binding_sha256=BINDING_SHA, replay_plan_sha256=PLAN_SHA, disk_reservation=reserve,
                sparse_inference_indices='0,10,20,...', all_inferences_archived=False,
                common_teacher_target_kph=9, fixed_arm_order=['A', 'B'],
                all_42_export_files_verified=True, all_34_current_training_sources_verified=True,
                no_metric_based_endpoint_selection=True, this_single_arm_is_not_a_pair=True)
            if args.mode == 'preflight':
                return plan
            owner.execute(plan)
            result = outcome.final_report(owner.OUTPUT)
            result.update(arm=args.arm, source_checkpoint_sha256=item['checkpoint_sha256'],
                binding_sha256=BINDING_SHA, learned_goal_stop_claimed=False,
                exact_inputs_all_inferences_archived=False, replay_plan_sha256=PLAN_SHA,
                this_single_arm_is_not_a_pair=True)
            owner.write_json(owner.OUTPUT / 'owner_diagnostic_result.json', result)
            if args.arm == 'B':
                pair_receipt()
            return result
    finally:
        (owner.OUTPUT, owner.WEIGHTS, owner.METADATA, owner.PINS, owner.collector_command,
         owner.write_json, owner.lifecycle._workspace_lock) = saved


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--mode', choices=('contract', 'verify', 'preflight', 'execute', 'worker'), required=True)
    parser.add_argument('--arm', choices=proof.LABELS)
    args, remaining = parser.parse_known_args(argv)
    if args.mode == 'worker':
        require(args.arm is not None, 'worker requires the exact fixed arm')
        return worker(args, remaining)
    require(not remaining, 'unexpected runtime overrides are forbidden')
    if args.mode in ('contract', 'verify'):
        source_pins()
        result = dict(status='PREREGISTERED_NO_SIMULATOR', replay_plan_sha256=PLAN_SHA,
            fixed_arm_order=['A', 'B'], seconds_per_arm=20, teacher_kph=9, speed_cap_mps=2.5,
            minimum_pair_free_bytes=prior.FLOOR + 2 * prior.ARM_RESERVE + prior.SHARED_RESERVE,
            current_free_bytes=shutil.disk_usage(ROOT).free, checkpoints=CHECKPOINTS,
            no_one_arm_pair_claim=True, no_retry_or_extension=True)
        if args.mode == 'verify':
            endpoints, pins = released_endpoints()
            torch.set_num_threads(2)
            for label, item in endpoints.items():
                load_endpoint(label, item)
            source_unchanged(pins)
            result.update(status='BOTH_ENDPOINTS_VERIFIED_NO_SIMULATOR', binding_sha256=BINDING_SHA,
                          evidence_files=42, current_training_sources=34, local_checkpoint_decode=False)
    else:
        require(args.arm is not None, 'fixed replay arm required')
        result = trial(args)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if args.mode != 'execute' or result['status'] in ('DIAGNOSTIC_BUDGET_REACHED', 'DIAGNOSTIC_CAPTURE_COMPLETE') else 1


if __name__ == '__main__':
    raise SystemExit(main())
