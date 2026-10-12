#!/usr/bin/env python3
"""HH_260906 - Fit six external research scorers while the original STOPMIX repository and generators remain frozen."""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import ModuleType

WORKSPACE = Path.home() / 'personal/hwanhong/portable_e2e'
SOURCE_COMMIT = 'b478f02e42b94bf04bffec5c8170e05edc33b0f8'
HELPER_PATH = WORKSPACE / 'benchmarks/training_profile_v1/profile_portable_training.py'
HELPER_SHA = 'a995722f1a60f192a059433afb44e6297937d97166877c5cc4de23220d15b436'
ORACLE_SHA = '35d3d0a464fb43c9789f6e24a35e453736c99fdceb36e3292d085c5ceb960d37'
MODEL_PATH = 'portable_e2e/config/perspective_trajectory_physical_stopmix_v1.model.json'
MODEL_SHA = '29e537e8b216cff0216772624f670ece0f0849f0f2ec55c0b726f11c1d74b602'
MODEL_ID = 'portable_e2e.perspective_trajectory.physical_stopmix.v1'
CAMPAIGN = 'hh260909-stopmix-duration-10epochs-v1'
CHECKPOINTS = {
    '20260903': 'bb8844dd77559480bf6c2f1c897c296fb9871731e2cb089d2143fb79d02bb908',
    '20260904': '3ef0acba3ae3ad4ccf7a0c124722f68365cd4a0fbc800266b729de00708cf90b',
    '20260905': 'bb75905bf083fd7d02fd2c63004dcbd1912c00909290afc1b6a4b8f039057de7',
}
SPLITS = {'train': (1147, 'd957e72c1eea755fed5b4ac682e775983861c16104fe083b7cfb6cbb4fed1928'),
    'val': (337, '631be7323f502cafc0dd66766104203bd5e7fee7494436c479bff1e0dddc0285')}
ARMS = ('linear_pair_reset', 'candidate_reset')
HEAD_PARAMETERS = {'linear_pair_reset': 3084, 'candidate_reset': 115201}
INPUTS = ('images', 'calibration', 'ego_history', 'ego_history_mask', 'route_xy', 'route_mask')
CACHE_FIELDS = ('fused', 'candidate_xy', 'candidate_speed', 'target_xy', 'target_speed_mps', 'target_yaw_rad', 'target_valid', 'original_logits')
RENDER_INDICES = (0, 31, 61, 92, 122, 153, 183, 214, 244, 275, 305, 336)
SCHEMA = 'portable_e2e.external_frozen_stopmix_selector.v1'
PLAN_SCHEMA = 'portable_e2e.external_frozen_stopmix_selector_plan.v1'
CACHE_ID = 'portable_e2e.frozen_stopmix_selector_cache.research.v1'
CACHE_TIMEOUT, HEAD_TIMEOUT, AUDIT_TIMEOUT = 120, 240, 90
INTERNAL_TIMEOUT, EXTERNAL_TIMEOUT, RESERVE = 3300, 3600, 180
DENIALS = dict(generator_training=False, candidate_regeneration_during_head_fit=False,
    runtime_checkpoint_export=False, automatic_promotion=False, vehicle_control_approved=False,
    training_data_approved=False, source_or_label_changes=False, test_neural_inference=False, test_used_for_selection=False)


def require(value, message):
    if not value: raise ValueError(message)


def sha(raw): return hashlib.sha256(raw).hexdigest()


def encoded(value): return (json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False) + '\n').encode()


def regular(path):
    path = Path(path).absolute()
    require(path.is_file() and not any(p.is_symlink() for p in (path, *path.parents)), 'regular nonsymlink file required')
    return path


def load_external(path, expected, name):
    # HH_260906 - Execute only reviewed helper/core bytes; package source is never copied into or modified in the frozen repository.
    path = regular(path); raw = path.read_bytes(); require(sha(raw) == expected, 'external module SHA differs')
    require(name not in sys.modules, 'external module name already loaded')
    module = ModuleType(name); module.__file__ = str(path); module.__package__ = name.rpartition('.')[0]
    sys.modules[name] = module
    try: exec(compile(raw, str(path), 'exec'), module.__dict__)
    except BaseException:
        del sys.modules[name]; raise
    require(sha(path.read_bytes()) == expected, 'external module changed while loading')
    return module


def source_paths(base):
    return (*base.SOURCE_PATHS, MODEL_PATH, 'portable_e2e/audit_runtime.py', 'portable_e2e/evaluate.py',
        'portable_e2e/runtime.py', 'portable_e2e/frozen_selector.py', 'portable_e2e/visualize.py',
        'scripts/e2e/audit_portable_stopmix_behavior.py')


def plan_contract(base, oracle, projection):
    return dict(schema=PLAN_SCHEMA, experiment_id='hh260909-frozen-stopmix-selector-six-heads-v1',
        source_commit=SOURCE_COMMIT, gpu_uuid=base.GPU_UUID, dataset=base.DATASET,
        dataset_manifest_sha256=base.MANIFEST_SHA, corpus_fingerprint_sha256=base.CORPUS_SHA,
        split_contract={k: dict(sample_count=v[0], fingerprint_sha256=v[1]) for k, v in SPLITS.items()},
        seeds=list(CHECKPOINTS), parent_checkpoint_sha256=CHECKPOINTS, parent_campaign_id=CAMPAIGN,
        parent_steps=2870, parent_samples_seen=11470, parent_model_id=MODEL_ID,
        model_config=MODEL_PATH, model_config_sha256=MODEL_SHA, parent_parameter_count=1056362,
        original_receipt_sha256=oracle.RECEIPTS, arms=list(ARMS), head_parameter_counts=HEAD_PARAMETERS, expected_head_fit_count=6,
        steps=1540, batch_size=4, sample_exposures_per_head=6155, learning_rate=1e-4, weight_decay=1e-4,
        maximum_gradient_norm=5., candidate_score_weight=.1, loss_config=base.LOSS_CONFIG,
        candidate_count=12, future_points=64, fused_width=256, loss_projection=projection,
        cache_batch_size=4, evaluation_batch_size=4, num_workers=0,
        cache_timeout_seconds=CACHE_TIMEOUT, head_timeout_seconds=HEAD_TIMEOUT, audit_timeout_seconds=AUDIT_TIMEOUT,
        internal_wall_timeout_seconds=INTERNAL_TIMEOUT, external_wall_timeout_seconds=EXTERNAL_TIMEOUT,
        safety_reserve_seconds=RESERVE, finish_before_utc='2026-09-09T01:00:00Z',
        fixed_val_render_indices=list(RENDER_INDICES), owner_helper_sha256=HELPER_SHA, oracle_helper_sha256=ORACLE_SHA,
        baseline_contract='Two original Linear(256,6) heads concatenated DRIVE then STOP; cached logits reproduced bit-exactly in the original batches of four.',
        sampling_contract='Original uniform_without_replacement epoch policy; the same seeded order for both heads of each parent.',
        timing_scope='One process owns the GPU lease. Each cache has 120 seconds, each head fit 240 seconds, and each paired TRAIN/VAL cache audit 90 seconds. Stage alarms preserve the whole-study deadline; external process timeout remains mandatory.',
        integrity_scope='The unchanged full-corpus validator may read held-out test bytes for integrity; no test NN forward, optimization, metrics or model selection.',
        interpretation='Six scorer-only research fits, not six full-model fits. Head capacities differ (3084 versus 115201 parameters), so architecture and capacity are confounded. Future-derived groups are post-fit diagnostics only. No newly selected checkpoint is automatically promoted.',
        **DENIALS)


def validate_plan(plan, base, oracle, projection):
    expected = plan_contract(base, oracle, projection)
    require(set(plan) == set(expected) | {'declared_at_utc', 'source_sha256', 'worker_source_sha256', 'core_source_sha256'}, 'exact plan fields required')
    require(encoded({k: plan[k] for k in expected}) == encoded(expected), 'unreviewed experiment setting')
    require(set(plan['source_sha256']) == set(source_paths(base)), 'complete frozen source list required')
    for value in [*plan['source_sha256'].values(), plan['worker_source_sha256'], plan['core_source_sha256']]:
        require(type(value) is str and len(value) == 64 and all(c in '0123456789abcdef' for c in value), 'SHA256 required')
    now = datetime.now(timezone.utc)
    require(base.timestamp(plan['declared_at_utc']) <= now and
        (base.timestamp(plan['finish_before_utc']) - now).total_seconds() >= EXTERNAL_TIMEOUT + RESERVE, 'full deadline reserve required')


def source_identity(plan, base, repo, core_path, oracle_path):
    environment = dict(os.environ, GIT_NO_LAZY_FETCH='1', GIT_ALLOW_PROTOCOL='', GIT_TERMINAL_PROMPT='0')
    def git(*args): return subprocess.check_output(['git', '-c', 'protocol.allow=never', *args], cwd=repo, env=environment, timeout=10)
    require(git('rev-parse', 'HEAD').decode().strip() == SOURCE_COMMIT and not git('status', '--porcelain', '--untracked-files=all').strip(), 'clean unchanged b478 repository required')
    result = {}
    for name in source_paths(base):
        raw = regular(repo / name).read_bytes()
        require(raw == git('show', SOURCE_COMMIT + ':' + name) and sha(raw) == plan['source_sha256'][name], 'source differs: ' + name)
        result[name] = sha(raw)
    for name, path, value in (('external_worker.py', Path(__file__), plan['worker_source_sha256']),
        ('external_core.py', core_path, plan['core_source_sha256']), ('external_oracle_helper.py', oracle_path, ORACLE_SHA),
        ('external_owner_helper.py', HELPER_PATH, HELPER_SHA)):
        require(base.digest(path) == value, 'external source differs: ' + name); result[name] = value
    require(result[MODEL_PATH] == MODEL_SHA, 'model config differs')
    return result


def checkpoint_path(seed):
    return WORKSPACE / 'runs/campaigns' / CAMPAIGN / f'seed_{seed}/B_drive_stop_mix/training/checkpoints/latest.pt'


def input_receipts(base, oracle, plan):
    pins = {name: base.digest(WORKSPACE / name) for name in oracle.RECEIPTS}
    require(pins == oracle.RECEIPTS, 'original lineage receipts differ')
    state = base.json_read(WORKSPACE / f'runs/campaigns/{CAMPAIGN}/status.json')
    require(state['status'] == 'COMPLETE_NOT_PROMOTED' and state['completed_stage_count'] == len(state['stages']) == 24
        and state['parent_source_and_output_postcheck_pass'] is True, 'completed 24-stage continuation required')
    require(base.timestamp(state['finished_at_utc']) <= base.timestamp(plan['declared_at_utc']), 'new experiment must follow completed parents')
    for seed, expected in CHECKPOINTS.items():
        path = checkpoint_path(seed); require(base.digest(path) == expected, 'continued parent checkpoint differs')
        pins[str(path.relative_to(WORKSPACE))] = expected
        for stage, filename in (('train', 'training/run.json'), ('evaluate', 'evaluation/metrics.json'), ('audit', 'gate_v8.json')):
            rows = [r for r in state['stages'] if r['run'] == f'seed_{seed}/B_drive_stop_mix' and r['stage'] == stage]
            require(len(rows) == 1, 'unique completed parent stage required'); row = rows[0]; report = path.parents[2] / filename
            require(row['status'] == 'COMPLETE' and type(row['returncode']) is int and row['returncode'] == 0
                and row['report']['sha256'] == base.digest(report), 'parent stage report differs')
            if stage != 'train': require(base.json_read(report)['checkpoint_sha256'] == expected, 'checkpoint/report association differs')
            pins[str(report.relative_to(WORKSPACE))] = base.digest(report)
    return pins


@contextmanager
def stage_alarm(seconds):
    # HH_260906 - A stage can shorten but never restart or extend the original whole-study timer.
    remaining, interval = signal.getitimer(signal.ITIMER_REAL); started = time.monotonic()
    require(remaining > 0, 'whole-study timer required')
    signal.setitimer(signal.ITIMER_REAL, min(seconds, remaining))
    try: yield
    finally: signal.setitimer(signal.ITIMER_REAL, max(remaining - (time.monotonic() - started), 1e-6), interval)


def tensor_state_sha(values):
    digest = hashlib.sha256()
    for name, tensor in sorted(values.items()):
        value = tensor.detach().cpu().contiguous(); header = encoded([name, str(value.dtype), list(value.shape)])
        digest.update(len(header).to_bytes(8, 'big')); digest.update(header); digest.update(value.numpy().tobytes())
    return digest.hexdigest()


def frozen_generator_sha(model):
    require(not model.training and all(not p.requires_grad and p.grad is None for p in model.parameters()),
        'generator must remain eval-only with no trainable parameters or accumulated gradients')
    return tensor_state_sha(model.state_dict())


def verify_import_paths(repo, modules=None):
    # HH_260906 - Verified files are insufficient unless imported native modules actually resolve to those frozen files.
    modules = sys.modules if modules is None else modules
    names = ('contract', 'dataset', 'torch_dataset', 'losses', 'model', 'train', 'evaluate',
        'audit_runtime', 'runtime', 'runtime_contract', 'frozen_selector', 'visualize', 'stop_primitive_research')
    expected = {'portable_e2e.' + n: repo / 'portable_e2e' / (n + '.py') for n in names}
    expected['scripts.e2e.audit_portable_stopmix_behavior'] = repo / 'scripts/e2e/audit_portable_stopmix_behavior.py'
    for name, path in expected.items():
        module = modules.get(name)
        require(module is not None and Path(module.__file__).resolve() == path, 'imported native module escaped frozen repository: ' + name)
    return {name: path.relative_to(repo).as_posix() for name, path in expected.items()}


def validate_datasets(datasets, base):
    for split, (count, fingerprint) in SPLITS.items():
        data = datasets[split]
        require(data.split == split and len(data) == count and data.fingerprint_sha256 == fingerprint, 'fixed split identity differs')
        require(len({e.token for e in data.examples}) == count and len({e.episode_id for e in data.examples}) == (3 if split == 'train' else 1), 'split IDs or episode count differs')
    require(not {e.token for e in datasets['train'].examples} & {e.token for e in datasets['val'].examples}
        and not {e.episode_id for e in datasets['train'].examples} & {e.episode_id for e in datasets['val'].examples}, 'train/val leakage')


def validate_parent(payload, config, training_ids, seed, datasets, base):
    require(config.model_id == MODEL_ID and config.candidate_count == 12
        and list(training_ids) == sorted({e.episode_id for e in datasets['train'].examples})
        and payload['dataset_fingerprint_sha256'] == SPLITS['train'][1]
        and payload['state']['global_step'] == 2870 and payload['state']['samples_seen'] == 11470
        and payload['train_config']['seed'] == int(seed) and payload['loss_config'] == base.LOSS_CONFIG, 'exact continued parent TRAIN identity differs')


def extract_cache(model, dataset, core, oracle, base, checkpoint_sha, journal, torch):
    from torch.utils.data import DataLoader
    require(dataset.split in SPLITS and not model.training and not any(p.requires_grad for p in model.parameters()), 'only frozen train/val capture allowed')
    before = frozen_generator_sha(model); arrays = {name: [] for name in CACHE_FIELDS}
    extras = {name: [] for name in ('route_xy', 'route_mask')}; identities = []; observed = []
    hook = model.fusion.register_forward_hook(lambda _module, _inputs, output: observed.append(output.detach().clone()))
    try:
        with journal.open('x') as stream, torch.no_grad(), stage_alarm(CACHE_TIMEOUT):
            for batch in DataLoader(dataset, batch_size=4, shuffle=False, num_workers=0, pin_memory=False):
                observed.clear(); tensors = {k: batch[k].to('cuda:0') for k in INPUTS}
                xy, speed, logits = model(*(tensors[k] for k in INPUTS)); require(len(observed) == 1, 'fusion must run once')
                values = (observed[0], xy, speed, batch['target_xy'], batch['target_speed_mps'], batch['target_yaw_rad'], batch['target_valid'], logits)
                for name, value in zip(CACHE_FIELDS, values): arrays[name].append(value.detach().cpu().clone())
                for name in extras: extras[name].append(batch[name].detach().cpu().clone())
                for i, token in enumerate(batch['sample_id']):
                    index = len(identities); example = dataset.examples[index]
                    require(token == example.token and batch['target_valid'][i].tolist() == [p is not None for p in example.targets_xy], 'original ordered sample or mask differs')
                    row = dict(index=index, sample_id=token, model_input_sha256=oracle.tensor_sha(batch, i, INPUTS),
                        target_sha256=oracle.tensor_sha(batch, i, ('target_xy', 'target_speed_mps', 'target_yaw_rad', 'target_valid')))
                    stream.write(encoded(row).decode()); stream.flush(); identities.append(row)
    finally: hook.remove()
    require(len(identities) == len(dataset) and frozen_generator_sha(model) == before, 'cache incomplete or generator changed')
    data = core.CachedStopmixSelectorData(split=dataset.split, sample_ids=tuple(e.token for e in dataset.examples),
        episode_ids=tuple(e.episode_id for e in dataset.examples), source_checkpoint_sha256=checkpoint_sha,
        corpus_fingerprint_sha256=base.CORPUS_SHA, dataset_fingerprint_sha256=dataset.fingerprint_sha256,
        **{name: torch.cat(values) for name, values in arrays.items()})
    return data, {name: torch.cat(values) for name, values in extras.items()}, identities, before


def audit_cache(data, extras, dataset, identities, logits, output, oracle, behavior, loss_module, projected, torch):
    # HH_260906 - Future-derived groups are diagnostic outputs only; scorers receive fused context and detached candidates, never these labels.
    from portable_e2e.visualize import render_trajectory_png
    output.mkdir(parents=True, exist_ok=False); rows = []; costs = []; renders = []
    require(tuple(logits.shape) == (len(data), 12), 'all cached logits required')
    with (output / 'samples.jsonl').open('x') as stream, torch.no_grad(), stage_alarm(AUDIT_TIMEOUT):
        for start in range(0, len(data), 4):
            index = slice(start, start + 4)
            cost_rows = oracle.candidate_rows(data.candidate_xy[index], data.candidate_speed[index], logits[index].to(data.fused.device),
                data.target_xy[index], data.target_speed_mps[index], data.target_valid[index], data.target_yaw_rad[index],
                loss_module.TrajectoryLossConfig(), loss_module, projected)
            for offset, cost in enumerate(cost_rows):
                i = start + offset; example = dataset.examples[i]
                row = behavior.analyze_sample(index=i, example=example, xy=data.candidate_xy[i].tolist(),
                    speed=data.candidate_speed[i].tolist(), logits=logits[i].tolist(), target_xy=data.target_xy[i].tolist(),
                    target_speed=data.target_speed_mps[i].tolist(), valid=data.target_valid[i].tolist(), model_id=MODEL_ID)
                cost.update(raw_current_vx_mps=float(example.features[1]), target_motion_group=row['target_motion_group'],
                    episode_id=example.episode_id, capture_phase='not_reclassified_by_this_experiment')
                require(row['selected_candidate_index'] == cost['selected_candidate_index'], 'cost/gate selection differs')
                row.update(loss_alignment=cost, model_input_sha256=identities[i]['model_input_sha256'], target_sha256=identities[i]['target_sha256'])
                stream.write(encoded(row).decode()); stream.flush(); rows.append(row); costs.append(cost)
                if data.split == 'val' and i in RENDER_INDICES:
                    name = f'val_{i:03d}.png'
                    render_trajectory_png(output / name, route_xy=extras['route_xy'][i][extras['route_mask'][i]].tolist(),
                        target_xy=data.target_xy[i].tolist(), target_valid=data.target_valid[i].tolist(),
                        candidate_xy=data.candidate_xy[i].tolist(), candidate_logits=logits[i].tolist(),
                        title=f'Frozen STOPMIX selector; {output.parent.name}; val index {i}; research only', width=900, height=700)
                    renders.append(dict(index=i, sample_id=example.token, camera_sha256=list(example.camera_sha256), file=name, sha256=sha(regular(output / name).read_bytes())))
    require(len(rows) == len(data) and (data.split != 'val' or [r['index'] for r in renders] == list(RENDER_INDICES)), 'complete split and fixed renders required')
    invariant = [{k: r[k] for k in ('candidate_composite_costs', 'candidate_ade_m', 'composite_oracle_index', 'exact_minimum_indices', 'original_target_valid')} for r in costs]
    report = dict(schema='portable_e2e.cached_stopmix_selection_diagnostic.v1', split=data.split,
        sample_count=len(rows), cache_digest=data.digest(), oracle_and_cost_sha256=sha(encoded(invariant)),
        oracle_indices=[r['composite_oracle_index'] for r in costs], **behavior.summarize_rows(rows, 12),
        composite_cost_alignment=oracle.summarize_rows(costs), renders=renders,
        samples_sha256=sha(regular(output / 'samples.jsonl').read_bytes()), **DENIALS)
    with (output / 'summary.json').open('x') as stream: stream.write(encoded(report).decode())
    return report


def save_tensors(path, payload, torch):
    require(not path.exists() and not any(p.is_symlink() for p in (path, *path.parents)), 'new research artifact only')
    with path.open('xb') as stream: torch.save(payload, stream)


def output_inventory(output, base):
    return {p.relative_to(output).as_posix(): base.digest(p) for p in sorted(output.rglob('*')) if p.is_file() or p.is_symlink()}


def completed_outputs_intact(output, records, base):
    for record in records:
        for name, value in record.get('completed_files', {}).items():
            if base.digest(output / record['directory'] / name) != value: return False
    return True


def verify_history(path, report):
    rows = []
    for line in regular(path).read_bytes().splitlines():
        rows.append(json.loads(line, parse_constant=lambda _: (_ for _ in ()).throw(ValueError('nonfinite history'))))
    require(rows == report['history'] and len(rows) == 1540
        and [r['global_step'] for r in rows] == list(range(1, 1541))
        and rows[-1]['samples_seen'] == sum(r['batch_samples'] for r in rows) == 6155, 'persisted training history incomplete or changed')


def run(args, base, oracle):
    repo = WORKSPACE / 'autoware_e2e'; base.check_environment(WORKSPACE, repo)
    for path in (Path(__file__).absolute(), args.core.absolute(), args.oracle_helper.absolute()):
        require(path.is_relative_to(WORKSPACE / 'benchmarks'), 'all new execution modules must be outside the frozen repo under personal benchmarks')
    plan_path = regular(args.plan); plan = base.json_read(plan_path); plan_sha = base.digest(plan_path)
    _, projection = oracle.projection_definition(regular(repo / 'portable_e2e/losses.py').read_bytes())
    validate_plan(plan, base, oracle, projection)
    dataset_root = (WORKSPACE / base.DATASET).resolve(strict=True)
    require(dataset_root.is_relative_to((WORKSPACE.parent / 'dataset').resolve()), 'dataset must remain in personal storage')
    output = base.checked_output(args.output_dir, dataset_root, WORKSPACE)
    before = source_identity(plan, base, repo, args.core, args.oracle_helper)
    report = dict(schema=SCHEMA, status='STARTED', started_at_utc=base.utc(), plan_sha256=plan_sha,
        source_sha256=before, loss_projection=projection, seeds=[], **DENIALS)
    model = None; datasets = {}; loaded = {}; loader = None; core = None; records = []; original = {}; caches = {}; cache_sha = {}
    with base.gpu_lease(WORKSPACE), base.bounded_signals(INTERNAL_TIMEOUT):
        report['gpu_preflight'] = base.gpu_idle(repo); original = input_receipts(base, oracle, plan)
        output.mkdir(parents=True, exist_ok=False)
        try:
            with (output / 'plan.json').open('xb') as stream: stream.write(plan_path.read_bytes())
            external = {'external_worker.py': Path(__file__), 'external_core.py': args.core,
                'external_oracle_helper.py': args.oracle_helper, 'external_owner_helper.py': HELPER_PATH}
            for name, value in before.items():
                source = external.get(name, repo / name); target = output / 'source' / name; target.parent.mkdir(parents=True, exist_ok=True)
                with target.open('xb') as stream: stream.write(regular(source).read_bytes())
                require(base.digest(target) == value, 'executed source archive differs')
            sys.path.insert(0, str(repo)); import torch
            from portable_e2e import losses as loss_module
            from portable_e2e.audit_runtime import _read_checkpoint_for_audit
            from portable_e2e.dataset import load_training_examples
            from portable_e2e.model import ModelConfig, PerspectiveTrajectoryModel, parameter_count
            from portable_e2e.torch_dataset import Common10TorchDataset
            from portable_e2e.train import _nested_tensors_are_finite, _seed_everything
            from scripts.e2e import audit_portable_stopmix_behavior as behavior
            require(torch.cuda.device_count() == 1 and torch.get_num_threads() == 16, 'one GPU and fixed16 CPU threads required')
            report['torch_device_uuid'] = base.verify_torch_uuid(torch.cuda.get_device_properties(0)); report['torch_version'] = str(torch.__version__)
            core = load_external(args.core, plan['core_source_sha256'], 'portable_e2e._frozen_stopmix_selector_external')
            require(tuple(core.ARMS) == ARMS, 'reviewed fresh head arms differ')
            report['imported_native_modules'] = verify_import_paths(repo)
            projected, proof = oracle.projected_costs(loss_module); require(proof == projection, 'imported original loss differs')
            loader = load_training_examples; config = ModelConfig.from_mapping(base.json_read(repo / MODEL_PATH))
            for split in SPLITS:
                loaded[split] = loader(dataset_root, split=split, mode='planning', check_image_hashes=True)
                require(loaded[split].validation_report['dataset_fingerprint_sha256'] == base.CORPUS_SHA, 'whole corpus changed')
                datasets[split] = Common10TorchDataset(loaded[split].examples, config, verify_image_sha256=True, split=split)
            validate_datasets(datasets, base)
            for seed, checkpoint_sha in CHECKPOINTS.items():
                require(source_identity(plan, base, repo, args.core, args.oracle_helper) == before and input_receipts(base, oracle, plan) == original, 'frozen inputs changed between parents')
                item = output / f'seed_{seed}'; item.mkdir(); state = dict(seed=seed, directory=item.name, status='STARTED', parent_checkpoint_sha256=checkpoint_sha, arms=[])
                report['seeds'].append(state); records.append(state); _seed_everything(0, torch.device('cuda:0'))
                payload, checkpoint_config, training_ids, provenance = _read_checkpoint_for_audit(checkpoint_path=checkpoint_path(seed), expected_checkpoint_sha256=checkpoint_sha, corpus_fingerprint_sha256=base.CORPUS_SHA)
                require(checkpoint_config == config, 'parent model config differs'); validate_parent(payload, config, training_ids, seed, datasets, base)
                model = PerspectiveTrajectoryModel(config).to('cuda:0'); model.load_state_dict(payload['model_state_dict'], strict=True)
                require(parameter_count(model) == 1056362 and _nested_tensors_are_finite(model.state_dict()), 'parent model shape/state differs')
                model.requires_grad_(False); model.eval(); del payload
                generator_sha = frozen_generator_sha(model); state.update(checkpoint_validation=provenance, generator_state_sha256=generator_sha)
                caches, extras, identities, baselines = {}, {}, {}, {}
                for split in SPLITS:
                    cache, extras[split], identities[split], initial = extract_cache(model, datasets[split], core, oracle, base, checkpoint_sha, item / f'{split}_cache_progress.jsonl', torch)
                    require(initial == generator_sha, 'generator changed during cache')
                    save_tensors(item / f'{split}_cache.pt', dict(artifact_id=CACHE_ID, split=split, digest=cache.digest(),
                        sample_ids=list(cache.sample_ids), episode_ids=list(cache.episode_ids), source_checkpoint_sha256=checkpoint_sha,
                        tensors={n: getattr(cache, n) for n in CACHE_FIELDS}, extra=extras[split], **DENIALS), torch)
                    require(completed_outputs_intact(output, records, base), 'earlier cache artifact changed')
                    state['completed_files'] = output_inventory(item, base)
                    caches[split] = cache.to('cuda:0')
                cache_sha = {k: v.digest() for k, v in caches.items()}; state['cache_digest'] = cache_sha
                with stage_alarm(AUDIT_TIMEOUT):
                    for split in SPLITS:
                        baselines[split] = audit_cache(caches[split], extras[split], datasets[split], identities[split], caches[split].original_logits,
                            item / 'original' / split, oracle, behavior, loss_module, projected, torch)
                require(completed_outputs_intact(output, records, base), 'cache artifact changed during baseline audit')
                state['completed_files'] = output_inventory(item, base)
                sampling_order = None
                for arm in ARMS:
                    require(source_identity(plan, base, repo, args.core, args.oracle_helper) == before
                        and completed_outputs_intact(output, records, base), 'source or completed evidence changed between heads')
                    arm_dir = item / arm; arm_dir.mkdir(); arm_state = dict(arm=arm, status='STARTED'); state['arms'].append(arm_state)
                    with (arm_dir / 'metrics.jsonl').open('x') as history, stage_alarm(HEAD_TIMEOUT):
                        def deadline(): require(datetime.now(timezone.utc) < base.timestamp(plan['finish_before_utc']), 'user deadline reached')
                        def progress(row):
                            deadline(); history.write(encoded(row).decode()); history.flush()
                            if row.get('global_step', row.get('step', 0)) % 154 == 0: print(json.dumps(dict(event='HEAD_PROGRESS', seed=seed, arm=arm, record=row)), flush=True)
                        result = core.fit_frozen_selector(caches['train'], caches['val'], original_drive_head=model.candidate_head,
                            original_stop_head=model.stop_candidate_head, arm=arm, seed=int(seed), device='cuda:0', callback=progress, check_deadline=deadline)
                    require(frozen_generator_sha(model) == generator_sha and {k: v.digest() for k, v in caches.items()} == cache_sha, 'generator/cache mutated during head fitting')
                    verify_import_paths(repo)
                    require(result['report']['final_state']['global_step'] == 1540 and result['report']['final_state']['samples_seen'] == 6155, 'full head exposure required')
                    require(result['report']['head_parameter_count'] == HEAD_PARAMETERS[arm], 'head capacity differs from plan')
                    verify_history(arm_dir / 'metrics.jsonl', result['report'])
                    if sampling_order is None: sampling_order = result['report']['sampling_order_sha256']
                    require(result['report']['sampling_order_sha256'] == sampling_order, 'paired heads used different sample orders')
                    with stage_alarm(AUDIT_TIMEOUT):
                        for split in SPLITS:
                            require(torch.equal(result['baseline_logits_cpu'][split], caches[split].original_logits.cpu()), 'original two-head logits not reproduced exactly')
                            measured = audit_cache(caches[split], extras[split], datasets[split], identities[split], result['final_logits_cpu'][split],
                                arm_dir / 'analysis' / split, oracle, behavior, loss_module, projected, torch)
                            require(measured['oracle_and_cost_sha256'] == baselines[split]['oracle_and_cost_sha256']
                                and measured['oracle_indices'] == result['oracle_indices_cpu'][split].tolist(), 'fixed candidate costs/oracles changed')
                    save_tensors(arm_dir / 'head_only.pt', dict(artifact_id=result['report']['artifact_id'], head_state_dict=result['head_state_dict'],
                        parent_checkpoint_sha256=checkpoint_sha, source_commit=SOURCE_COMMIT, **DENIALS), torch)
                    save_tensors(arm_dir / 'final_logits.pt', dict(artifact_id='portable_e2e.frozen_stopmix_logits.research.v1', logits=result['final_logits_cpu'], **DENIALS), torch)
                    base.write_json(arm_dir / 'report.json', result['report'])
                    arm_state.update(status='HEAD_COMPLETE_NOT_PROMOTED', completed_at_utc=base.utc(), files=output_inventory(arm_dir, base))
                    require(completed_outputs_intact(output, records, base), 'earlier completed evidence changed during head fit')
                    state['completed_files'] = output_inventory(item, base)
                require(frozen_generator_sha(model) == generator_sha and {k: v.digest() for k, v in caches.items()} == cache_sha, 'final frozen cache/model changed')
                state.update(status='TWO_HEADS_COMPLETE_NOT_PROMOTED', generator_state_after_sha256=generator_sha, completed_at_utc=base.utc())
                del result, caches, cache, model; caches = {}; model = None; torch.cuda.empty_cache()
            require(len(report['seeds']) == 3 and all(s['status'] == 'TWO_HEADS_COMPLETE_NOT_PROMOTED' for s in report['seeds']), 'all six head fits required')
            report['status'] = 'SIX_HEAD_FITS_COMPLETE_NOT_PROMOTED'
        except BaseException as error:
            report.update(status='FAILED_OR_PARTIAL_NOT_PROMOTED', error_type=type(error).__name__, error=str(error))
        finally:
            with base.finalization_signals():
                errors = []
                checks = {'source': lambda: source_identity(plan, base, repo, args.core, args.oracle_helper) == before,
                    'original_parent_bytes': lambda: input_receipts(base, oracle, plan) == original,
                    'plan': lambda: base.digest(plan_path) == plan_sha == base.digest(output / 'plan.json'),
                    'source_archives': lambda: all(base.digest(output / 'source' / n) == v for n, v in before.items()),
                    'completed_outputs': lambda: completed_outputs_intact(output, records, base),
                    'dataset_manifest': lambda: base.digest(dataset_root / 'dataset.json') == base.MANIFEST_SHA}
                if loader is not None:
                    for split in loaded:
                        def corpus(split=split):
                            after = loader(dataset_root, split=split, mode='planning', check_image_hashes=True)
                            return after.validation_report['dataset_fingerprint_sha256'] == base.CORPUS_SHA and after.fingerprint_sha256 == loaded[split].fingerprint_sha256
                        checks['full_corpus_' + split] = corpus
                if model is not None:
                    checks['active_generator'] = lambda: frozen_generator_sha(model) == generator_sha
                if caches and cache_sha:
                    checks['active_caches'] = lambda: {k: v.digest() for k, v in caches.items()} == cache_sha
                if 'imported_native_modules' in report:
                    checks['imported_native_modules'] = lambda: verify_import_paths(repo) == report['imported_native_modules']
                for name, operation in checks.items():
                    try: require(operation(), name + ' changed')
                    except BaseException as error: errors.append(dict(check=name, error_type=type(error).__name__, error=str(error)))
                report.update(completed_at_utc=base.utc(), postcheck_errors=errors, original_input_sha256=original,
                    completed_head_fit_count=sum(a['status'] == 'HEAD_COMPLETE_NOT_PROMOTED' for s in report['seeds'] for a in s['arms']),
                    source_and_inputs_unchanged=not errors, deadline_met=datetime.now(timezone.utc) < base.timestamp(plan['finish_before_utc']))
                if errors or not report['deadline_met']: report['status'] = 'FAILED_OR_PARTIAL_NOT_PROMOTED'
                base.write_json(output / 'report.json', report)
                inventory = output_inventory(output, base)
                with (output / 'SHA256SUMS').open('x') as stream:
                    for name, value in inventory.items(): stream.write(value + '  ' + name + '\n')
    return 0 if report['status'] == 'SIX_HEAD_FITS_COMPLETE_NOT_PROMOTED' else 2


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--plan', type=Path, required=True); parser.add_argument('--core', type=Path, required=True)
    parser.add_argument('--oracle-helper', type=Path, required=True); parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args(argv)
    base = load_external(HELPER_PATH, HELPER_SHA, '_frozen_stopmix_owner_helper')
    oracle = load_external(args.oracle_helper, ORACLE_SHA, '_frozen_stopmix_oracle_helper')
    return run(args, base, oracle)


if __name__ == '__main__': raise SystemExit(main())
