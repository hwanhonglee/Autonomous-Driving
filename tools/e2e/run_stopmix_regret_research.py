#!/usr/bin/env python3
"""HH_260906 - Fresh STOPMIX loss A/B on a pinned TRAIN/VAL-only view; never promote."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time

WORKSPACE = Path.home() / 'personal/hwanhong/portable_e2e'
SOURCE_COMMIT = 'b478f02e42b94bf04bffec5c8170e05edc33b0f8'
GPU_UUID = 'GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5'
MODEL_PATH = 'portable_e2e/config/perspective_trajectory_physical_stopmix_v1.model.json'
MODEL_SHA = '29e537e8b216cff0216772624f670ece0f0849f0f2ec55c0b726f11c1d74b602'
MODEL_ID = 'portable_e2e.perspective_trajectory.physical_stopmix.v1'
OWNER_PATH = 'benchmarks/training_profile_v1/profile_portable_training.py'
OWNER_SHA = 'a995722f1a60f192a059433afb44e6297937d97166877c5cc4de23220d15b436'
SOURCE_PATHS = tuple('portable_e2e/' + name + '.py' for name in (
    '__init__', 'contract', 'dataset', 'torch_dataset', 'model', 'losses', 'train',
    'evaluate', 'audit_runtime', 'runtime', 'runtime_contract', 'runtime_weight_bundle',
    'visualize', 'stop_primitive_research')) + (
    'portable_e2e/config/common_10hz_v1.contract.json', MODEL_PATH,
    'scripts/e2e/audit_portable_stopmix_behavior.py', 'scripts/e2e/run_portable_training_campaign.py')
SEEDS = (20260903, 20260904, 20260905)
ARMS = {'A_original_loss': 0.0, 'B_cost_aware': 0.1}
RUNS = tuple(f'seed_{seed}/{arm}' for seed in SEEDS for arm in ARMS)
TIMEOUTS = {'train': 600, 'evaluate': 120, 'audit': 120, 'behavior': 180}
LOSS = dict(xy_weight=1., speed_weight=.2, yaw_weight=.1, kinematic_speed_weight=.05,
            final_displacement_weight=.5, candidate_score_weight=.1)
DENIALS = dict(training_data_approved=False, vehicle_control_approved=False,
    automatic_promotion=False, automatic_retry=False, test_read_or_use=False,
    model_export=False, runtime_gate_changes=False, resume=False)
RENDER_INDICES = (0, 31, 61, 92, 122, 153, 183, 214, 244, 275, 305, 336)


def require(value, message):
    if not value:
        raise ValueError(message)


def encoded(value):
    return (json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False) + '\n').encode()


def regular(path):
    path = Path(path).absolute()
    require('..' not in path.parts and path == path.resolve() and path.is_file()
            and not any(p.is_symlink() for p in (path, *path.parents)), 'regular canonical nonsymlink file required')
    return path


def digest(path):
    value = hashlib.sha256()
    with regular(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def read(path):
    def pairs(items):
        value = {}
        for key, item in items:
            require(key not in value, 'duplicate JSON key')
            value[key] = item
        return value
    return json.loads(regular(path).read_text(), object_pairs_hook=pairs,
        parse_constant=lambda _: require(False, 'nonfinite JSON'))


def write(path, value):
    # HH_260906 - Every receipt is new and flushed; failed runs keep their original partial artifacts.
    with Path(path).open('xb') as stream:
        stream.write(encoded(value)); stream.flush(); os.fsync(stream.fileno())


def utc():
    return datetime.now(timezone.utc).isoformat()


def relative(name):
    require(isinstance(name, str) and name and not Path(name).is_absolute()
            and '..' not in Path(name).parts and Path(name).as_posix() == name, 'unsafe relative path')
    return name


def hash_value(value):
    require(isinstance(value, str) and re.fullmatch('[0-9a-f]{64}', value), 'SHA256 required')


def plan_contract():
    return dict(schema='portable_e2e.stopmix_regret_research_plan.v1',
        campaign_id='hh260914-stopmix-regret-ab-3seeds-v1', source_commit=SOURCE_COMMIT,
        gpu_uuid=GPU_UUID, model_config=MODEL_PATH, model_config_sha256=MODEL_SHA,
        owner_path=OWNER_PATH, owner_sha256=OWNER_SHA, runs=list(RUNS),
        seeds=list(SEEDS), arms=ARMS, steps=1540, samples_seen=6155, batch_size=4,
        learning_rate=1e-4, weight_decay=1e-4, candidate_score_weight=.1,
        checkpoint_interval=154, num_workers=0, maximum_gradient_norm=5.,
        sampling_policy='uniform_without_replacement', stage_timeout_seconds=TIMEOUTS,
        whole_timeout_seconds=7200, fixed_val_render_indices=list(RENDER_INDICES),
        decision=dict(final_checkpoint_only=True, all_three_paired_seeds=True,
            selected_ade_and_fde_strictly_lower=True, geometry_count_nondecreasing=True,
            maximum_speed_mae_ratio=1.05, absolute_limits_m=dict(ade_1p0s_m=.5, ade_3p0s_m=1., ade_6p4s_m=2., fde_6p4s_m=4.),
            retain_all_subgroup_regressions=True, total_loss_not_comparable=True), **DENIALS)


def validate_plan(plan, worker_sha, observed=None):
    fixed = plan_contract()
    variable = {'declared_at_utc', 'worker_sha256', 'source_sha256', 'dataset',
        'dataset_manifest_sha256', 'view_report_path', 'view_report_sha256',
        'view_file_sha256', 'split_contract', 'corpus_fingerprint_sha256'}
    require(set(plan) == set(fixed) | variable, 'plan fields differ')
    require(encoded({k: plan[k] for k in fixed}) == encoded(fixed), 'fixed experiment settings differ')
    stamp = datetime.fromisoformat(plan['declared_at_utc'].replace('Z', '+00:00'))
    require(stamp.tzinfo is not None and stamp.utcoffset().total_seconds() == 0
            and stamp <= (observed or datetime.now(timezone.utc)), 'prospective UTC declaration required')
    require(plan['worker_sha256'] == worker_sha, 'worker SHA differs')
    for key in ('worker_sha256', 'dataset_manifest_sha256', 'view_report_sha256', 'corpus_fingerprint_sha256'):
        hash_value(plan[key])
    require(set(plan['source_sha256']) == set(SOURCE_PATHS), 'native source closure differs')
    for value in plan['source_sha256'].values():
        hash_value(value)
    require(plan['source_sha256'][MODEL_PATH] == MODEL_SHA, 'STOPMIX config pin differs')
    require(relative(plan['dataset']).startswith('datasets/prepared/'), 'view must be in personal prepared datasets')
    relative(plan['view_report_path'])
    require(set(plan['split_contract']) == {'train', 'val'}, 'only TRAIN and VAL permitted')
    for split, count in (('train', 1147), ('val', 337)):
        item = plan['split_contract'][split]
        require(set(item) == {'sample_count', 'fingerprint_sha256'} and type(item['sample_count']) is int
                and item['sample_count'] == count, 'split count differs')
        hash_value(item['fingerprint_sha256'])
    require(isinstance(plan['view_file_sha256'], dict) and 'dataset.json' in plan['view_file_sha256'], 'view file inventory missing')
    require(plan['view_file_sha256']['dataset.json'] == plan['dataset_manifest_sha256'], 'view manifest pins disagree')
    for name, value in plan['view_file_sha256'].items():
        relative(name); hash_value(value)
        require(not any('test' in part.lower() for part in Path(name).parts), 'TEST path forbidden in view')
    return plan


def load_module(path, expected, name):
    require(digest(path) == expected, 'helper SHA differs')
    spec = importlib.util.spec_from_file_location(name, regular(path))
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    require(digest(path) == expected, 'helper changed while importing')
    return module


def verify_sources(plan, repo):
    env = dict(os.environ, GIT_NO_LAZY_FETCH='1', GIT_ALLOW_PROTOCOL='', GIT_TERMINAL_PROMPT='0')
    def git(*args):
        return subprocess.check_output(['git', '-c', 'protocol.allow=never', *args], cwd=repo, env=env, timeout=15)
    require(git('rev-parse', 'HEAD').decode().strip() == SOURCE_COMMIT, 'native repo HEAD differs')
    for name, expected in plan['source_sha256'].items():
        require(digest(repo / name) == expected and hashlib.sha256(git('show', f'{SOURCE_COMMIT}:{name}')).hexdigest() == expected,
                'native source bytes differ: ' + name)


def verify_view(plan, workspace):
    # HH_260906 - Inspect only this physical TRAIN/VAL view, never the parent corpus or a TEST path.
    lexical = workspace / plan['dataset']
    datasets = workspace / 'datasets'
    if datasets.is_symlink():
        # HH_260906 - Preserve the approved personal datasets alias, never arbitrary input symlinks.
        require(datasets.resolve() == workspace.parent / 'dataset', 'unapproved dataset alias')
        current = datasets
        for part in Path(plan['dataset']).parts[1:]:
            current = current / part
            require(not current.is_symlink(), 'symlinks below approved dataset alias forbidden')
        root = lexical.resolve()
        require(root.is_relative_to(workspace.parent / 'dataset/prepared'), 'view escaped approved personal dataset tree')
        require(not any(p.is_symlink() for p in (root, *root.parents)), 'unsafe resolved dataset path')
    else:
        root = lexical
        require(root == root.resolve() and not any(p.is_symlink() for p in (root, *root.parents)), 'unsafe view root')
    require(root.is_dir(), 'view directory required')
    observed = {}
    for parent, directories, files in os.walk(root, followlinks=False):
        for name in [*directories, *files]:
            path = Path(parent) / name
            require(not path.is_symlink(), 'symlink inside view forbidden')
            require('test' not in name.lower(), 'TEST entry inside view forbidden')
        for name in files:
            path = Path(parent) / name
            key = path.relative_to(root).as_posix()
            require(key in plan['view_file_sha256'], 'unplanned view file; refuse to read')
            observed[key] = digest(path)
    require(observed == plan['view_file_sha256'], 'view bytes or complete inventory differ')
    proof_path = workspace / plan['view_report_path']
    require(not proof_path.is_relative_to(root) and not root.is_relative_to(proof_path), 'view proof must be outside dataset')
    require(digest(proof_path) == plan['view_report_sha256'], 'view proof changed')
    proof = read(proof_path)
    require(proof['schema'] == 'portable_e2e.trainval_research_view.v1'
        and proof['status'] == 'VALIDATED_LEGACY_RESEARCH_VIEW_NOT_NEW_ADMISSION'
        and proof['counts'] == {'train': 1147, 'val': 337}
        and proof['copied_payloads_byte_exact'] is True and proof['source_inputs_unchanged'] is True
        and proof['output_file_sha256'] == observed
        and proof['output_dataset_manifest_sha256'] == plan['dataset_manifest_sha256']
        and proof['corpus_fingerprint_sha256'] == plan['corpus_fingerprint_sha256'], 'view proof contract differs')
    require(proof['source_dataset_manifest_sha256'] == '18262e5aa4abbb3e03e35e379b5da1e5ce7fd339a9a8942e02b58ca737f7242c'
        and proof['fresh_fits_only'] is True and proof['optimizer_resume_allowed'] is False
        and all(proof[k] is False for k in ('training_data_approved', 'new_quality_approval',
            'labels_modified', 'split_reassignment', 'test_payloads_opened', 'test_targets_opened',
            'test_images_opened', 'test_provenance_opened')), 'view scope differs')
    for split in ('train', 'val'):
        require(proof['split_tensor_fingerprint_sha256'][split] == plan['split_contract'][split]['fingerprint_sha256'],
                'tensor split fingerprint differs from view proof')
    manifest = read(root / 'dataset.json')
    counts = {'train': 0, 'val': 0}
    require(len(manifest['episodes']) == 4, 'view must contain four TRAIN/VAL episodes')
    for ref in manifest['episodes']:
        relative(ref['manifest'])
        require(ref['manifest'] in observed and observed[ref['manifest']] == ref['sha256'], 'episode ref not pinned')
        episode = read(root / ref['manifest'])
        require(episode['split'] in counts, 'view must not reference TEST')
        counts[episode['split']] += episode['sample_count']
    require(counts == {'train': 1147, 'val': 337}, 'view manifest split counts differ')
    return observed


def command(plan, workspace, repo, output, run, stage, plan_path, script):
    seed_name, arm = run.split('/')
    item = output / run
    checkpoint = item / 'training/checkpoints/latest.pt'
    python = str(workspace / 'venvs/py312/bin/python')
    dataset = str(workspace / plan['dataset'])
    shared = ['--device', 'cuda:0', '--batch-size', '4']
    if stage == 'train':
        return [python, '-m', 'portable_e2e.train', dataset, '--run-dir', str(item / 'training'),
            '--model-config', str(repo / MODEL_PATH), '--split', 'train', *shared,
            '--seed', seed_name.removeprefix('seed_'), '--learning-rate', '0.0001',
            '--weight-decay', '0.0001', '--candidate-score-weight', '0.1',
            '--candidate-regret-weight', str(ARMS[arm]), '--max-steps', '1540',
            '--checkpoint-interval', '154', '--num-workers', '0', '--maximum-gradient-norm', '5',
            '--sampling-policy', 'uniform_without_replacement']
    if stage == 'evaluate':
        return [python, '-m', 'portable_e2e.evaluate', dataset, '--checkpoint', str(checkpoint),
            '--output-dir', str(item / 'evaluation'), '--split', 'val', *shared, '--num-workers', '0', '--render-count', '12']
    if stage == 'audit':
        return [python, '-m', 'portable_e2e.audit_runtime', dataset, '--checkpoint', str(checkpoint),
            '--checkpoint-sha256', digest(checkpoint), '--output-json', str(item / 'gate_v8.json'), '--split', 'val', *shared]
    require(stage == 'behavior', 'unknown stage')
    return [python, str(script), '--plan', str(plan_path), '--output-dir', str(output), '--behavior-run', run]


def verify_stage(plan, output, run, stage):
    item = output / run
    arm = run.split('/')[1]
    path = item / {'train': 'training/run.json', 'evaluate': 'evaluation/metrics.json',
                  'audit': 'gate_v8.json', 'behavior': 'behavior/summary.json'}[stage]
    report = read(path)
    config = read(output / 'source' / MODEL_PATH)
    require(digest(output / 'source' / MODEL_PATH) == MODEL_SHA, 'archived model config differs')
    canonical_model_sha = hashlib.sha256(json.dumps(config, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
    require(report['corpus_fingerprint_sha256'] == plan['corpus_fingerprint_sha256'], 'stage corpus differs')
    if stage in ('train', 'evaluate'):
        require(report['device'] == 'cuda:0' and report['hardware']['device_uuid'] == GPU_UUID.removeprefix('GPU-'), 'stage device differs')
    if stage == 'train':
        require(report.get('warning') == 'RESEARCH CHECKPOINT — NOT APPROVED FOR VEHICLE CONTROL'
            and 'resumed_at_utc' not in report, 'fresh research training required')
        require(report['status'] == 'TRAINING_TARGET_REACHED' and report['dataset_size'] == 1147
                and report['model_parameter_count'] == 1056362, 'training denominator/status/model differs')
        require(report['dataset_fingerprint_sha256'] == plan['split_contract']['train']['fingerprint_sha256'], 'TRAIN fingerprint differs')
        require(report['state']['global_step'] == 1540 and report['state']['samples_seen'] == 6155
                and report['state']['domain_samples_seen'] == {'carla': 6155}, 'training exposure differs')
        loss = dict(LOSS)
        if ARMS[arm]:
            loss['candidate_regret_weight'] = ARMS[arm]
        require(report['loss_config'] == loss, 'loss objective differs')
        expected = dict(seed=int(run.split('/')[0][5:]), batch_size=4, learning_rate=1e-4, weight_decay=1e-4,
            max_steps=1540, checkpoint_interval=154, num_workers=0, maximum_gradient_norm=5.,
            verify_image_sha256=True, sampling_policy='uniform_without_replacement', domain_ratios=[])
        require(report['train_config'] == expected and report['model_config'] == config, 'training configuration differs')
    else:
        require(report.get('vehicle_control_approved') is False, 'stage must deny control')
        require(report['checkpoint_sha256'] == digest(item / 'training/checkpoints/latest.pt'), 'checkpoint changed')
        if stage in ('evaluate', 'audit'):
            count = report['sample_count'] if stage == 'evaluate' else report['geometry']['sample_count']
            require(count == 337 and report['evaluation_split'] == 'val'
                and report['dataset_fingerprint_sha256'] == plan['split_contract']['val']['fingerprint_sha256'], 'VAL denominator differs')
            require(report['training_dataset_fingerprint_sha256'] == plan['split_contract']['train']['fingerprint_sha256']
                and report['model_config_sha256'] == canonical_model_sha, 'evaluation checkpoint provenance differs')
        if stage == 'evaluate':
            require(report['status'] == 'OPEN_LOOP_EVALUATION_COMPLETE' and report['model_parameter_count'] == 1056362, 'evaluation incomplete')
            require(all(type(x) in (int, float) and math.isfinite(x) for x in report['metrics'].values()), 'nonfinite evaluation')
            if ARMS[arm]:
                require(report['loss_config'] == dict(LOSS, candidate_regret_weight=.1)
                    and set(report['auxiliary_loss_metrics']) == {'candidate_regret_loss'}
                    and math.isfinite(report['auxiliary_loss_metrics']['candidate_regret_loss'])
                    and report['auxiliary_loss_metric_counts'] == {'candidate_regret_loss': 337}, 'research auxiliary proof differs')
            else:
                require('auxiliary_loss_metrics' not in report and 'loss_config' not in report, 'baseline auxiliary must remain disabled')
        elif stage == 'audit':
            require(report['status'] == 'RUNTIME_GEOMETRY_AUDIT_COMPLETE'
                and report['gate']['source'] == 'portable_e2e.runtime_geometry_gate.v8'
                and report['gate']['threshold_overrides'] is False and report['gate']['thresholds']['candidate_count'] == 12,
                'runtime gate differs')
            require(report['implementation']['audit_runtime_sha256'] == plan['source_sha256']['portable_e2e/audit_runtime.py']
                and report['implementation']['runtime_contract_sha256'] == plan['source_sha256']['portable_e2e/runtime_contract.py'],
                'runtime implementation SHA differs')
        else:
            require(report['status'] == 'COMPLETE_NOT_PROMOTED' and set(report['splits']) == {'train', 'val'}, 'behavior incomplete')
            for split, count in (('train', 1147), ('val', 337)):
                require(report['splits'][split]['all_samples']['sample_count'] == count, 'behavior denominator differs')
                require(digest(item / 'behavior' / f'{split}_samples.jsonl') == report['splits'][split]['samples_sha256'], 'behavior raw rows changed')
            require([r['index'] for r in report['renders']] == list(RENDER_INDICES), 'behavior render denominator differs')
            for render in report['renders']:
                require(render['path'] == f'trajectories/val_{render["index"]:03d}.png'
                    and digest(item / 'behavior' / render['path']) == render['sha256'], 'behavior PNG changed')
    return {'path': str(path.relative_to(output)), 'sha256': digest(path), 'checkpoint_sha256': digest(item / 'training/checkpoints/latest.pt')}


def behavior(plan, output, run, repo):
    # HH_260906 - Reuse pure fixed subgroup/gate mathematics, not the old CLI's hardcoded parent manifest.
    sys.path.insert(0, str(repo))
    import torch
    require(torch.cuda.device_count() == 1 and str(torch.cuda.get_device_properties(0).uuid) == GPU_UUID.removeprefix('GPU-'),
            'behavior requires exactly the assigned physical GPU0')
    from torch.utils.data import DataLoader
    helper = load_module(repo / 'scripts/e2e/audit_portable_stopmix_behavior.py',
        plan['source_sha256']['scripts/e2e/audit_portable_stopmix_behavior.py'], '_regret_behavior_math')
    dataset_root = WORKSPACE / plan['dataset']
    item = output / run
    checkpoint = item / 'training/checkpoints/latest.pt'
    checkpoint_sha = digest(checkpoint)
    payload, config, train_ids, provenance = helper._read_checkpoint_for_audit(
        checkpoint_path=checkpoint, expected_checkpoint_sha256=checkpoint_sha,
        corpus_fingerprint_sha256=plan['corpus_fingerprint_sha256'])
    require(config.to_dict() == read(repo / MODEL_PATH) and len(train_ids) == 3
        and provenance['training_dataset_fingerprint_sha256'] == plan['split_contract']['train']['fingerprint_sha256'],
        'behavior checkpoint ABI differs')
    target = item / 'behavior'; target.mkdir(exist_ok=False)
    (target / 'trajectories').mkdir()
    result = dict(schema='portable_e2e.stopmix_regret_behavior.v1', status='COMPLETE_NOT_PROMOTED',
        checkpoint_sha256=checkpoint_sha, corpus_fingerprint_sha256=plan['corpus_fingerprint_sha256'],
        splits={}, renders=[], vehicle_control_approved=False, test_read_or_use=False,
        limitations=['TRAIN subgroups are fitted evidence, not held-out stopping success.',
            'VAL has no stationary-hold samples; zero count gives no evidence.',
            'Target groups do not identify traffic intent; overlapping windows are not independent episodes.',
            'Predicted open-loop paths are not CARLA driving or runtime deployment.'])
    device = helper._select_runtime_device('cuda:0'); helper._seed_everything(0, device)
    # HH_260906 - TRAIN must not pass the held-out episode-disjointness validator: load its model via the same secure checkpoint.
    from portable_e2e.model import PerspectiveTrajectoryModel
    model = PerspectiveTrajectoryModel(config).to(device)
    model.load_state_dict(payload['model_state_dict'], strict=True); model.eval()
    for split, count in (('train', 1147), ('val', 337)):
        loaded = helper.load_training_examples(dataset_root, split=split, mode='planning', check_image_hashes=True)
        require(len(loaded.examples) == count
            and loaded.validation_report['dataset_fingerprint_sha256'] == plan['corpus_fingerprint_sha256'], 'behavior split differs')
        dataset = helper.Common10TorchDataset(loaded.examples, config, verify_image_sha256=True, split=split)
        require(dataset.fingerprint_sha256 == plan['split_contract'][split]['fingerprint_sha256'], 'behavior tensor fingerprint differs')
        episode_ids = {e.episode_id for e in loaded.examples}
        require((episode_ids == set(train_ids)) if split == 'train' else not episode_ids.intersection(train_ids),
                'behavior training/held-out episode identities differ')
        rows = []
        with (target / f'{split}_samples.jsonl').open('x') as stream, torch.no_grad():
            for batch in DataLoader(dataset, batch_size=4, shuffle=False, num_workers=0):
                predictions = model(*[batch[k].to(device) for k in helper.INPUT_KEYS])
                xy_rows, speed_rows, logits_rows = [t.detach().cpu().tolist() for t in predictions]
                for pos, token in enumerate(batch['sample_id']):
                    index = len(rows); require(token == dataset.examples[index].token, 'behavior order changed')
                    xy_target = batch['target_xy'][pos].tolist(); valid = batch['target_valid'][pos].tolist()
                    row = helper.analyze_sample(index=index, example=dataset.examples[index], xy=xy_rows[pos],
                        speed=speed_rows[pos], logits=logits_rows[pos], target_xy=xy_target,
                        target_speed=batch['target_speed_mps'][pos].tolist(), valid=valid, model_id=MODEL_ID)
                    stream.write(encoded(row).decode()); stream.flush(); rows.append(row)
                    if split == 'val' and index in RENDER_INDICES:
                        name = f'trajectories/val_{index:03d}.png'
                        helper.render_trajectory_png(target / name,
                            route_xy=batch['route_xy'][pos][batch['route_mask'][pos]].tolist(), target_xy=xy_target,
                            target_valid=valid, candidate_xy=xy_rows[pos], candidate_logits=logits_rows[pos],
                            title=f'OPEN-LOOP RESEARCH ONLY | {run} | val {index} | {row["target_motion_group"]}')
                        result['renders'].append(dict(index=index, path=name, sha256=digest(target / name)))
            os.fsync(stream.fileno())
        require(len(rows) == count, 'behavior rows incomplete')
        result['splits'][split] = helper.summarize_rows(rows, 12)
        result['splits'][split]['samples_sha256'] = digest(target / f'{split}_samples.jsonl')
    require([r['index'] for r in result['renders']] == list(RENDER_INDICES), 'fixed renders incomplete')
    require(digest(checkpoint) == checkpoint_sha, 'checkpoint changed during behavior')
    write(target / 'summary.json', result)


def paired_results(output):
    result = []
    for seed in SEEDS:
        pair = {}
        for arm in ARMS:
            item = output / f'seed_{seed}' / arm
            metrics = read(item / 'evaluation/metrics.json')
            gate = read(item / 'gate_v8.json')
            groups = read(item / 'behavior/summary.json')
            pair[arm] = dict(metrics=metrics['metrics'], metric_counts=metrics['metric_counts'],
                geometry=gate['geometry'], behavior=groups['splits'])
        result.append(dict(seed=seed, arms=pair,
            metric_delta_B_minus_A={k: pair['B_cost_aware']['metrics'][k] - pair['A_original_loss']['metrics'][k]
                for k in pair['A_original_loss']['metrics'] if k != 'loss'},
            unlike_total_loss_not_compared=True))
    return result


def run(plan_path, output_dir, behavior_run=None):
    script = regular(__file__); plan_path = regular(plan_path)
    plan_sha = digest(plan_path); plan = validate_plan(read(plan_path), digest(script))
    workspace = WORKSPACE; repo = workspace / 'autoware_e2e'
    output = Path(output_dir).absolute()
    expected = workspace / 'runs/diagnostics/stopmix_regret_20260914_v1'
    require(output == expected and output.resolve() == output and not any(p.is_symlink() for p in (output, *output.parents)), 'fixed fresh output path required')
    verify_sources(plan, repo); verify_view(plan, workspace)
    require(digest(workspace / OWNER_PATH) == OWNER_SHA, 'GPU ownership helper differs')
    owner = load_module(workspace / OWNER_PATH, OWNER_SHA, '_regret_gpu_owner')
    owner.check_environment(workspace, repo)
    if behavior_run is not None:
        require(behavior_run in RUNS and os.environ.get('CUDA_VISIBLE_DEVICES') == GPU_UUID, 'invalid owned behavior stage')
        require(os.environ.get('PORTABLE_STOPMIX_REGRET_OWNER_PID') == str(os.getppid())
            and read(output / behavior_run / 'behavior_started.json')['status'] == 'RUNNING',
            'behavior must be a directly owned declared stage')
        behavior(plan, output, behavior_run, repo)
        verify_sources(plan, repo); verify_view(plan, workspace)
        require(digest(plan_path) == plan_sha and digest(script) == plan['worker_sha256'], 'behavior source or plan changed')
        return
    require(not output.exists(), 'fresh output required; resume forbidden')
    pipeline = load_module(repo / 'scripts/e2e/run_portable_training_campaign.py',
        plan['source_sha256']['scripts/e2e/run_portable_training_campaign.py'], '_regret_stage_owner')
    with owner.gpu_lease(workspace):
        idle = owner.gpu_idle(repo)
        output.mkdir(parents=True, exist_ok=False)
        write(output / 'plan.json', plan)
        archive = output / 'source'; archive.mkdir()
        for name in SOURCE_PATHS:
            target = archive / name; target.parent.mkdir(parents=True, exist_ok=True)
            with target.open('xb') as stream: stream.write((repo / name).read_bytes())
        with (archive / 'run_stopmix_regret_research.py').open('xb') as stream: stream.write(script.read_bytes())
        with (archive / 'profile_portable_training.py').open('xb') as stream: stream.write((workspace / OWNER_PATH).read_bytes())
        for name, expected_hash in plan['source_sha256'].items():
            require(digest(archive / name) == expected_hash, 'archived source differs before execution')
        require(digest(archive / 'run_stopmix_regret_research.py') == plan['worker_sha256']
            and digest(archive / 'profile_portable_training.py') == OWNER_SHA, 'archived external source differs before execution')
        status = dict(schema='portable_e2e.stopmix_regret_research.v1', status='RUNNING', started_at_utc=utc(),
            plan_sha256=plan_sha, pre_cuda_gpu=idle, stages=[], runs={name: 'NOT_RUN' for name in RUNS}, **DENIALS)
        write(output / 'started.json', status)
        started = time.monotonic()
        try:
            with owner.bounded_signals(7200):
                for name in RUNS:
                    verify_view(plan, workspace)
                    item = output / name; item.mkdir(parents=True)
                    status['runs'][name] = 'RUNNING'
                    checkpoint_sha = None
                    for stage, timeout in TIMEOUTS.items():
                        require(time.monotonic() - started + timeout + 30 < 7200, 'insufficient bounded stage/cleanup time')
                        verify_sources(plan, repo)
                        require(digest(script) == plan['worker_sha256'] and digest(plan_path) == plan_sha, 'worker or prospective plan changed')
                        require(digest(workspace / OWNER_PATH) == OWNER_SHA, 'ownership helper changed')
                        if checkpoint_sha is not None:
                            require(digest(item / 'training/checkpoints/latest.pt') == checkpoint_sha, 'trained checkpoint changed before stage')
                        args = command(plan, workspace, repo, output, name, stage, plan_path, script)
                        receipt = dict(run=name, stage=stage, command=args, timeout_seconds=timeout, started_at_utc=utc(), status='RUNNING')
                        write(item / f'{stage}_started.json', receipt)
                        with (item / f'{stage}.log').open('xb') as log:
                            rc = pipeline.run_owned_stage(args, repo,
                                dict(os.environ, PORTABLE_STOPMIX_REGRET_OWNER_PID=str(os.getpid())), log, timeout=timeout)
                        require(rc == 0, f'{name} {stage} exited {rc}')
                        if checkpoint_sha is not None:
                            require(digest(item / 'training/checkpoints/latest.pt') == checkpoint_sha, 'trained checkpoint changed during stage')
                        receipt.update(status='COMPLETE', ended_at_utc=utc(), **verify_stage(plan, output, name, stage))
                        checkpoint_sha = receipt['checkpoint_sha256']
                        write(item / f'{stage}_completed.json', receipt); status['stages'].append(receipt)
                    verify_view(plan, workspace)
                    status['runs'][name] = 'COMPLETE'
                verify_sources(plan, repo); verify_view(plan, workspace)
                for receipt in status['stages']:
                    require(digest(output / receipt['path']) == receipt['sha256']
                        and digest(output / receipt['run'] / 'training/checkpoints/latest.pt') == receipt['checkpoint_sha256'],
                        'completed stage or checkpoint changed')
                    if receipt['stage'] == 'behavior':
                        verify_stage(plan, output, receipt['run'], 'behavior')
                for name, expected_hash in plan['source_sha256'].items():
                    require(digest(archive / name) == expected_hash, 'archived native source changed')
                require(digest(archive / 'run_stopmix_regret_research.py') == plan['worker_sha256']
                    and digest(archive / 'profile_portable_training.py') == OWNER_SHA, 'archived external source changed')
                require(digest(script) == plan['worker_sha256'] and digest(plan_path) == plan_sha, 'final source or plan changed')
                require(digest(workspace / OWNER_PATH) == OWNER_SHA, 'final owner changed')
                status.update(status='COMPLETE_NOT_PROMOTED', paired_results=paired_results(output),
                    source_and_view_postcheck_pass=True, ended_at_utc=utc(), final_gpu=owner.gpu_idle(repo))
                write(output / 'summary.json', status)
        except BaseException as error:
            for name in RUNS:
                if status['runs'][name] == 'RUNNING': status['runs'][name] = 'FAIL'
            status.update(status='FAIL', error_type=type(error).__name__, error=str(error), ended_at_utc=utc())
            with owner.finalization_signals(): write(output / 'failed.json', status)
            raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--behavior-run', choices=RUNS, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    run(args.plan, args.output_dir, args.behavior_run)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
