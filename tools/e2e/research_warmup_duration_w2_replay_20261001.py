"""HH_261001 - Strict late-bound W2 export and one prospective matched sparse W1/W2 replay."""

import argparse
from contextlib import contextmanager
import hashlib
import json
from pathlib import Path
import re
import shutil

import torch
from portable_e2e import simulation_research as research
from scripts.e2e import collect_ah_simulation_research as short
from scripts.e2e import collect_w1_town07_long_20261001 as w1
from scripts.e2e.run_moving_multiroute_recovery_k import tensor_digest

ROOT = Path(__file__).resolve().parents[2]
PLAN = ROOT / 'docs/assets/validation/2026-10-01/selector_phase_ab/W2_TOWN07_PAIRED_REPLAY_PREREGISTRATION.md'
PLAN_SHA = '3ab061fb2c5ee0b758cfa48dd173751d44bec99844dc7cc117fa4f8781654307'
TRAIN_PLAN_SHA = '70229908028ac30cb9fd51a64e66d144485099280ebfd7128422e937dc5010b3'
TRAIN_SOURCE_SHA = 'c218dc27e09a94406e7bc2a23d5c7320a1b73eeb63cd09093e578980f5ef29c6'
W2_SCHEMA = 'portable_e2e.warmup_duration_w2_20261001.v1'
W1_SCHEMA = 'portable_e2e.warmup_supervision_ab_20261001.v1'
BINDING_SCHEMA = 'portable_e2e.w2_replay_endpoint_binding.v1'
W2_ARM = 'W2_warmup_anchor_off_4blocks'
PARENT_SHA = '4841d4076fb49145643ab2895d87befcae9021ad02f08b8307e75f77be137d91'
W1_STATE = 'ffcb58cd319bb6d1db412da71f8f9ef8697613482cdd2d760c64b7abbf6f110f'
MASK_SHA = '2c46c730096452291e10eed040c228e31ca2dc6f2ef424a02820e8d04efc3b1e'
ORDER_SHA = 'f0d3d522b3ec4787b84286f82e61017a6aee4619270a99451d3cea08e35110d9'
DATASET_SHA = '4dc6f23b62cc3b65154fb3414b1b509f3ef4696f6aa8bda9f8ee4535143cceda'
W1_EXPORT = ROOT / 'storage/experiments/training/2026-10-01/ah_warmup_supervision_ab_20261001_v1/exports/W1_warmup_anchor_off.npz'
NEW_FILES = (Path(__file__), PLAN, ROOT / 'learning/tests/test_warmup_duration_w2_replay_20261001.py')
FLOOR = 5 * 1024**3
ARM_RESERVE, SHARED_RESERVE = 256 * 1024**2, 64 * 1024**2
FIRST_FIT = dict(scheduled_steps=3000, global_step=2999, samples_seen=11996, active_supervision_exposures=11549)
FINAL_FIT = dict(global_step=11996, scheduled_steps=12000, samples_seen=47984, scheduled_samples=48000,
    active_supervision_exposures=46196, skipped_schedule_steps_one_based=[1693, 4693, 7693, 10693],
    optimizer_state_included=True, optimizer_resumed=False, optimizer_reset_between_blocks=False)


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(path):
    path = Path(path)
    require(path.is_file() and not path.is_symlink(), 'regular non-symlink file required: ' + str(path))
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest() if hasattr(hashlib, 'file_digest') else _stream_digest(stream)


def _stream_digest(stream):
    result = hashlib.sha256()
    for block in iter(lambda: stream.read(1024 * 1024), b''):
        result.update(block)
    return result.hexdigest()


def json_digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def read_json(path):
    digest(path)
    require(Path(path).stat().st_size <= 4 * 1024**2, 'bounded JSON required')
    return json.loads(Path(path).read_text())


def write_new(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')


def exact_sha(value):
    require(isinstance(value, str) and re.fullmatch('[0-9a-f]{64}', value) is not None,
            'explicit lowercase SHA256 required; no wildcard identity')
    return value


def validate_first(value):
    require(value.get('status') == 'PASS_EXACT_W1_STATE'
        and value.get('expected_model_state_sha256') == W1_STATE
        and value.get('actual_model_state_sha256') == W1_STATE
        and value.get('optimizer_same_object') is True and value.get('optimizer_reloaded') is False
        and value.get('checkpoint_used_for_model_selection') is False
        and value.get('checkpoint') == 'first_block_identity_checkpoint.pt'
        and value.get('fit') == FIRST_FIT, 'actual first-block W1 identity/continuity differs')
    exact_sha(value.get('checkpoint_sha256'))


def validate_payload(payload):
    require(payload.get('checkpoint_schema') == W2_SCHEMA and payload.get('arm') == W2_ARM
        and payload.get('parent_checkpoint_sha256') == PARENT_SHA
        and payload.get('training_plan_sha256') == TRAIN_PLAN_SHA
        and payload.get('training_dataset_fingerprint_sha256') == DATASET_SHA
        and payload.get('warmup_mask_sha256') == MASK_SHA and payload.get('exposure_order_sha256') == ORDER_SHA
        and payload.get('fit') == FINAL_FIT and isinstance(payload.get('optimizer_state_dict'), dict)
        and payload.get('optimizer_state_dict', {}).get('state')
        and payload.get('training_data_approved') is False and payload.get('model_promoted') is False
        and payload.get('vehicle_control_approved') is False, 'only the fixed W2 final endpoint is eligible')
    validate_first(payload.get('first_block_identity', {}))


@contextmanager
def identity(arm, checkpoint_sha, collector=None):
    exact_sha(checkpoint_sha)
    require(arm in ('W1', 'W2') and checkpoint_sha != PARENT_SHA
        and (arm != 'W1' or checkpoint_sha == w1.CHECKPOINT)
        and (arm != 'W2' or checkpoint_sha != w1.CHECKPOINT), 'endpoint arm identity differs')
    old = research.AH_CHECKPOINT_SHA256, research.CHECKPOINT_SCHEMA
    old_collector = collector.AH_CHECKPOINT_SHA256 if collector else None
    try:
        research.AH_CHECKPOINT_SHA256 = checkpoint_sha
        research.CHECKPOINT_SCHEMA = W1_SCHEMA if arm == 'W1' else W2_SCHEMA
        if collector:
            collector.AH_CHECKPOINT_SHA256 = checkpoint_sha
        yield
    finally:
        research.AH_CHECKPOINT_SHA256, research.CHECKPOINT_SCHEMA = old
        if collector:
            collector.AH_CHECKPOINT_SHA256 = old_collector


def export(args):
    """HH_261001 - Inspect fixed accounting and identities only, never endpoint metric values."""
    require(digest(PLAN) == PLAN_SHA, 'prospective replay plan differs')
    exact_sha(args.checkpoint_sha256)
    root = args.training_output.resolve()
    checkpoint = root / W2_ARM / 'training/final_checkpoint.pt'
    require(not (root / W2_ARM / 'failure.json').exists(), 'retained failed fit is not a W2 endpoint')
    require(digest(checkpoint) == args.checkpoint_sha256, 'caller-pinned final checkpoint differs')
    payload, actual = research._read_checkpoint_file(checkpoint, torch.device('cpu'))
    require(actual == args.checkpoint_sha256, 'checkpoint changed while safely read')
    validate_payload(payload)
    declaration = read_json(root / 'declaration.json')
    receipt = read_json(root / W2_ARM / 'training_receipt.json')
    complete = read_json(root / 'fixed_final_checkpoint_complete.json')
    summary = read_json(root / 'summary.json')
    first = read_json(root / W2_ARM / 'first_block_identity.json')
    validate_first(first)
    sources = declaration.get('source_sha256', {})
    require(len(sources) == 27 and json_digest(sources) == TRAIN_SOURCE_SHA,
            'exact frozen 27-file W2 training closure differs')
    require(declaration.get('schema') == W2_SCHEMA and declaration.get('arm') == W2_ARM
        and declaration.get('training_plan_sha256') == TRAIN_PLAN_SHA
        and payload.get('research_objective') == declaration, 'checkpoint declaration differs')
    require(receipt == complete and receipt == summary.get('training')
        and receipt.get('fit') == FINAL_FIT and receipt.get('checkpoint_sha256') == actual
        and receipt.get('checkpoint') == W2_ARM + '/training/final_checkpoint.pt'
        and receipt.get('first_block_identity') == first == payload.get('first_block_identity')
        and summary.get('schema') == W2_SCHEMA and summary.get('status') == 'COMPLETE_NOT_PROMOTED'
        and summary.get('source_hashes_match') is True, 'fixed completed endpoint receipt chain differs')
    first_path = checkpoint.with_name(first['checkpoint'])
    first_payload, first_sha = research._read_checkpoint_file(first_path, torch.device('cpu'))
    require(first_sha == first['checkpoint_sha256'] and first_payload.get('checkpoint_schema') == W2_SCHEMA
        and first_payload.get('purpose') == 'FIRST_BLOCK_IDENTITY_ONLY_NOT_SELECTION'
        and first_payload.get('arm') == W2_ARM and first_payload.get('parent_checkpoint_sha256') == PARENT_SHA
        and first_payload.get('training_plan_sha256') == TRAIN_PLAN_SHA
        and first_payload.get('fit') == dict(FIRST_FIT, optimizer_state_included=False, optimizer_resumed=False)
        and tensor_digest(first_payload['model_state_dict']) == W1_STATE
        and first_payload.get('final_model_state_sha256') == W1_STATE, 'actual first-block tensor proof differs')
    require(digest(args.parent_checkpoint) == PARENT_SHA, 'original AH bytes differ')
    parent, parent_sha = research._read_checkpoint_file(args.parent_checkpoint, torch.device('cpu'))
    initial, final = tensor_digest(parent['model_state_dict']), tensor_digest(payload['model_state_dict'])
    require(parent_sha == PARENT_SHA and payload.get('initial_model_state_sha256') == initial
        and first_payload.get('initial_model_state_sha256') == initial
        and payload.get('final_model_state_sha256') == final == receipt.get('final_model_state_sha256'),
        'actual final/parent tensors differ from endpoint provenance')
    evidence = {}
    paths = [checkpoint, first_path, args.parent_checkpoint, root / 'declaration.json',
        root / W2_ARM / 'training_receipt.json', root / W2_ARM / 'first_block_identity.json',
        root / 'fixed_final_checkpoint_complete.json', root / 'summary.json']
    for name, expected in sources.items():
        path = root / 'source' / name
        require(digest(path) == expected, 'archived training source differs: ' + name)
        paths.append(path)
    for path in paths:
        evidence[str(path.resolve())] = digest(path)
    require(not args.binding.exists() and not args.binding.is_symlink(), 'fresh late-binding receipt required')
    require(args.binding.resolve() not in (args.weights.resolve(), args.weights.with_suffix('.json').resolve()),
            'binding, NPZ and metadata must be distinct files')
    args.weights.parent.mkdir(parents=True, exist_ok=True)
    args.binding.parent.mkdir(parents=True, exist_ok=True)
    with identity('W2', actual):
        exported = research.export_ah_weights(checkpoint, args.weights)
        model = research.load_ah_weights(args.weights, args.weights.with_suffix('.json'),
            weights_sha256=exported['weights_sha256'], metadata_sha256=exported['metadata_sha256'])
        require(tensor_digest(model.state_dict()) == final, 'data-only export differs from actual trained tensors')
    binding = dict(schema=BINDING_SCHEMA, arm='W2', training_arm=W2_ARM, checkpoint_schema=W2_SCHEMA,
        checkpoint_sha256=actual, model_state_sha256=final, parent_checkpoint_sha256=PARENT_SHA,
        training_plan_sha256=TRAIN_PLAN_SHA, training_source_sha256=TRAIN_SOURCE_SHA,
        replay_plan_sha256=PLAN_SHA, fit=FINAL_FIT, first_block_identity=first,
        weights=str(args.weights.resolve()), metadata=str(args.weights.with_suffix('.json').resolve()),
        weights_sha256=exported['weights_sha256'], metadata_sha256=exported['metadata_sha256'],
        evidence_file_sha256=evidence, endpoint_selection_used_metric_values=False,
        vehicle_control_approved=False, training_data_approved=False, model_promoted=False)
    write_new(args.binding, binding)
    return dict(status='EXPORTED_NOT_PROMOTED', binding=str(args.binding.resolve()),
                binding_sha256=digest(args.binding), checkpoint_sha256=actual, model_state_sha256=final)


def binding_read(path, expected):
    exact_sha(expected)
    require(digest(path) == expected, 'exact externally pinned late-binding receipt required')
    value = read_json(path)
    require(value.get('schema') == BINDING_SCHEMA and value.get('arm') == 'W2'
        and value.get('training_arm') == W2_ARM and value.get('checkpoint_schema') == W2_SCHEMA
        and value.get('parent_checkpoint_sha256') == PARENT_SHA and value.get('fit') == FINAL_FIT
        and value.get('training_plan_sha256') == TRAIN_PLAN_SHA
        and value.get('training_source_sha256') == TRAIN_SOURCE_SHA
        and value.get('replay_plan_sha256') == PLAN_SHA and digest(PLAN) == PLAN_SHA
        and value.get('endpoint_selection_used_metric_values') is False
        and value.get('vehicle_control_approved') is False and value.get('training_data_approved') is False
        and value.get('model_promoted') is False, 'W2 late-binding contract differs')
    validate_first(value.get('first_block_identity', {}))
    for name in ('checkpoint_sha256', 'model_state_sha256', 'weights_sha256', 'metadata_sha256'):
        exact_sha(value.get(name))
    require(value['checkpoint_sha256'] not in (PARENT_SHA, w1.CHECKPOINT),
            'W2 may not substitute the parent or historical W1 checkpoint')
    evidence = value.get('evidence_file_sha256', {})
    require(len(evidence) == 35, 'complete fixed-end and archived-source evidence required')
    for name, sha in evidence.items():
        require(Path(name).is_absolute() and digest(name) == exact_sha(sha), 'late-bound evidence bytes changed')
    return value


def endpoint(arm, binding):
    require(arm in ('W1', 'W2'), 'undeclared replay arm')
    return (dict(checkpoint_sha256=w1.CHECKPOINT, checkpoint_schema=W1_SCHEMA, model_state_sha256=W1_STATE,
        weights=str(W1_EXPORT), metadata=str(W1_EXPORT.with_suffix('.json')),
        weights_sha256=w1.WEIGHTS_SHA, metadata_sha256=w1.METADATA_SHA) if arm == 'W1' else binding)


def output(arm):
    require(arm in ('W1', 'W2'), 'undeclared replay arm')
    return ROOT / ('storage/experiments/validation/2026-10-01/warmup_duration_' + arm + '_town07_sparse20_v1')


def disk_check(arms=1, free=None):
    require(arms in (1, 2), 'one arm or initial pair reservation required')
    available = shutil.disk_usage(ROOT).free if free is None else free
    needed = FLOOR + arms * ARM_RESERVE + SHARED_RESERVE
    require(available >= needed, 'disk reserve unavailable: need ' + str(needed) + ', have ' + str(available))
    return dict(free_bytes=available, required_bytes=needed, floor_bytes=FLOOR, reserved_arms=arms)


def validate_capture(args, route, arm):
    require(args.host == '127.0.0.1' and args.port == 2440 and not args.allow_map_load
        and args.control_transport == 'acknowledged_batch' and args.goal_stop_profile == 'disabled'
        and args.physics_hz == 20 and args.capture_hz == 10 and args.stationary_warmup_sec == 1.2
        and args.max_duration_sec == 20 and args.research_speed_cap_mps == 2.5
        and args.target_speed_kmh == 9 and args.research_max_cte_m == 1.
        and args.weather == 'ClearNoon' and args.seed == 0 and route['town'] == 'Town07'
        and route['scenario'] == 'straight' and args.output.resolve() == output(arm) / 'episode',
        'fixed paired 20-second teacher9/cap9 capture contract differs')


@contextmanager
def disk_guard():
    original = short.collector.exact_camera_bundle
    def guarded(*args, **kwargs):
        if shutil.disk_usage(ROOT).free < FLOOR + 16 * 1024**2:
            raise short.collector.CollectionError('fixed W2 replay disk reserve floor reached')
        return original(*args, **kwargs)
    short.collector.exact_camera_bundle = guarded
    try:
        yield
    finally:
        short.collector.exact_camera_bundle = original


def load_endpoint(arm, item):
    with identity(arm, item['checkpoint_sha256'], short):
        model = research.load_ah_weights(Path(item['weights']), Path(item['metadata']),
            weights_sha256=item['weights_sha256'], metadata_sha256=item['metadata_sha256'])
        require(tensor_digest(model.state_dict()) == item['model_state_sha256'], 'actual NPZ tensor identity differs')
        return model


def worker(args, remaining):
    binding = binding_read(args.binding, args.binding_sha256)
    item = endpoint(args.arm, binding)
    capture = short.collector.parse_args(remaining)
    capture.research_speed_cap_mps, capture.research_max_cte_m = 2.5, 1.
    validate_capture(capture, read_json(capture.route_file), args.arm)
    disk_check()
    torch.set_num_threads(2)
    model = load_endpoint(args.arm, item)
    write_new(output(args.arm) / 'worker_endpoint_identity.json', dict(arm=args.arm,
        checkpoint_sha256=item['checkpoint_sha256'], model_state_sha256=item['model_state_sha256'],
        binding_sha256=args.binding_sha256, replay_plan_sha256=PLAN_SHA))
    with identity(args.arm, item['checkpoint_sha256'], short), disk_guard(), \
            w1.sampled_inputs(output(args.arm) / 'sampled_exact_runtime_inputs'):
        short.run(model, capture)
    return 0


def prior_w1_ready(binding_sha):
    folder = output('W1')
    result = read_json(folder / 'owner_diagnostic_result.json')
    plan = read_json(folder / 'owner_plan.json')
    require(result.get('port_triplet_released') is True and result.get('source_hashes_match') is True
        and not result.get('owner_cleanup_errors') and plan.get('binding_sha256') == binding_sha
        and plan.get('replay_plan_sha256') == PLAN_SHA and plan.get('arm') == 'W1',
        'first matched W1 attempt with successful ownership/source cleanup required')
    for name, sha in plan['source_sha256'].items():
        require(digest(ROOT / name) == sha == digest(folder / 'source' / name), 'W1 archived/current source changed')


def trial(args):
    from scripts.e2e import run_owned_ah_town07_research_20261001 as owner
    from scripts.e2e import run_owned_ah_town07_research_v2_20261001 as outcome
    binding = binding_read(args.binding, args.binding_sha256)
    item = endpoint(args.arm, binding)
    reserve = disk_check(2 if args.arm == 'W1' else 1)
    if args.arm == 'W2':
        prior_w1_ready(args.binding_sha256)
    require(not output(args.arm).exists() and not output(args.arm).is_symlink(), 'fresh paired arm output required')
    saved = (owner.OUTPUT, owner.WEIGHTS, owner.METADATA, owner.PINS, owner.collector_command,
             owner.write_json, owner.lifecycle._workspace_lock)
    old_weights, old_metadata = owner.WEIGHTS, owner.METADATA
    original_command, original_write, original_lock = owner.collector_command, owner.write_json, owner.lifecycle._workspace_lock
    owner.OUTPUT, owner.WEIGHTS, owner.METADATA = output(args.arm), Path(item['weights']), Path(item['metadata'])
    owner.PINS = {p: sha for p, sha in owner.PINS.items() if p not in (old_weights, old_metadata)}
    owner.PINS.update({owner.WEIGHTS: item['weights_sha256'], owner.METADATA: item['metadata_sha256'],
                      PLAN: PLAN_SHA, args.binding.resolve(): args.binding_sha256})

    def command():
        argv = original_command()
        module = argv.index('scripts.e2e.collect_ah_simulation_research')
        rest = argv[argv.index(str(owner.OUTPUT / 'episode')):]
        return argv[:module] + ['scripts.e2e.research_warmup_duration_w2_replay_20261001', '--',
            '--mode', 'worker', '--arm', args.arm, '--binding', str(args.binding.resolve()),
            '--binding-sha256', args.binding_sha256] + rest

    def archive_write(path, value):
        if path == owner.OUTPUT / 'owner_plan.json':
            for source in NEW_FILES:
                relative = str(source.relative_to(ROOT))
                target = owner.OUTPUT / 'source' / relative
                if not target.exists():
                    target.parent.mkdir(parents=True, exist_ok=True)
                    with source.open('rb') as src, target.open('xb') as dest:
                        shutil.copyfileobj(src, dest)
                require(digest(target) == digest(source), 'new replay source archive differs')
                value['source_sha256'][relative] = digest(source)
            with args.binding.open('rb') as src, (owner.OUTPUT / 'endpoint_binding.json').open('xb') as dest:
                shutil.copyfileobj(src, dest)
        original_write(path, value)

    @contextmanager
    def reserved_lock():
        with original_lock():
            disk_check(2 if args.arm == 'W1' else 1)
            yield

    owner.collector_command, owner.write_json, owner.lifecycle._workspace_lock = command, archive_write, reserved_lock
    try:
        load_endpoint(args.arm, item)
        with identity(args.arm, item['checkpoint_sha256'], short):
            plan = owner.preflight()
            argv = command()
            capture = short.collector.parse_args(argv[argv.index(str(owner.OUTPUT / 'episode')):])
            capture.research_speed_cap_mps, capture.research_max_cte_m = 2.5, 1.
            validate_capture(capture, read_json(owner.ROUTE), args.arm)
            plan.update(schema='portable_e2e.w2_paired_sparse20_owner.v1', arm=args.arm,
                source_checkpoint_sha256=item['checkpoint_sha256'], source_checkpoint_schema=item['checkpoint_schema'],
                model_state_sha256=item['model_state_sha256'], parent_checkpoint_sha256=PARENT_SHA,
                binding_sha256=args.binding_sha256, replay_plan_sha256=PLAN_SHA,
                sparse_inference_indices='0,10,20,...', all_inferences_archived=False, disk_reservation=reserve,
                common_teacher_target_kph=9, no_metric_based_endpoint_selection=True)
            if args.mode == 'preflight':
                return plan
            owner.execute(plan)
            result = outcome.final_report(owner.OUTPUT)
            result.update(arm=args.arm, source_checkpoint_sha256=item['checkpoint_sha256'],
                binding_sha256=args.binding_sha256, learned_goal_stop_claimed=False,
                exact_inputs_all_inferences_archived=False, replay_plan_sha256=PLAN_SHA)
            owner.write_json(owner.OUTPUT / 'owner_diagnostic_result.json', result)
            return result
    finally:
        (owner.OUTPUT, owner.WEIGHTS, owner.METADATA, owner.PINS, owner.collector_command,
         owner.write_json, owner.lifecycle._workspace_lock) = saved


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--mode', required=True, choices=('contract', 'export', 'preflight', 'execute', 'worker'))
    parser.add_argument('--arm', choices=('W1', 'W2'))
    for name in ('training-output', 'parent-checkpoint', 'weights', 'binding'):
        parser.add_argument('--' + name, type=Path)
    parser.add_argument('--checkpoint-sha256')
    parser.add_argument('--binding-sha256')
    args, remaining = parser.parse_known_args(argv)
    if args.mode == 'worker':
        require(args.arm and args.binding and args.binding_sha256, 'worker requires fixed arm and exact binding')
        return worker(args, remaining)
    require(not remaining, 'unexpected arguments')
    if args.mode == 'contract':
        require(digest(PLAN) == PLAN_SHA, 'frozen prospective plan differs')
        result = dict(status='PREREGISTERED_NO_ENDPOINT_OR_SIMULATOR', plan_sha256=PLAN_SHA,
            fixed_arm_order=['W1', 'W2'], total_seconds_per_arm=20, teacher_kph=9, learned_cap_mps=2.5,
            minimum_pair_free_bytes=FLOOR + 2*ARM_RESERVE + SHARED_RESERVE,
            current_free_bytes=shutil.disk_usage(ROOT).free, required_W2_fit=FINAL_FIT)
    elif args.mode == 'export':
        require(all(getattr(args, name) is not None for name in
            ('training_output', 'parent_checkpoint', 'weights', 'binding', 'checkpoint_sha256')),
            'export requires all explicit training/checkpoint/parent/weights/binding inputs')
        result = export(args)
    else:
        require(args.arm and args.binding and args.binding_sha256, 'trial requires fixed arm and exact binding')
        result = trial(args)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if args.mode != 'execute' or result['status'] in ('DIAGNOSTIC_BUDGET_REACHED', 'DIAGNOSTIC_CAPTURE_COMPLETE') else 1


if __name__ == '__main__':
    raise SystemExit(main())
