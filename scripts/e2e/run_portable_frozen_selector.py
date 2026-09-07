#!/usr/bin/env python3
"""HH_260906 - Isolate scorer learning on immutable GPU0-generated train/val candidates."""

from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import sys
import time

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

import torch
from torch.utils.data import DataLoader

from portable_e2e.audit_runtime import (
    _AuditAccumulator, _atomic_new_json, _read_checkpoint_for_audit,
    _validate_checkpoint_and_model, audit_prediction,
)
from portable_e2e.contract import ContractError, _loads_json
from portable_e2e.dataset import FEATURE_NAMES, load_training_examples
from portable_e2e.model import PHYSICAL_MODEL_ID
from portable_e2e.runtime_contract import RUNTIME_GATE_ID, RuntimeGateConfig
from portable_e2e.torch_dataset import Common10TorchDataset
from portable_e2e.train import _atomic_json, _atomic_torch_save, _seed_everything
from portable_e2e.visualize import render_trajectory_png
from scripts.e2e.diagnose_portable_objective_alignment import (
    CORPUS_SHA256, MANIFEST_SHA256, SPLITS, checkpoint_loss_config,
    file_sha256, require_hash, source_provenance, validate_split,
)
from scripts.e2e.run_portable_training_campaign import (
    GPU_UUID, WORKSPACE, assert_gpu_idle, run_owned_stage, stage_environment, termination_guard,
)


SCHEMA = 'portable_e2e.frozen_selector_run.v1'
CACHE_ID = 'portable_e2e.frozen_selector_cache.v1'
PARENT_CAMPAIGN = 'hh260907-physical-v1-data-expansion-3seeds-v1'
PARENT_SHA256 = {
    20260903: 'cc43adb16d3d2677b5c05235c7563bc71bb267094abf3f1e3378ddfdb79149fd',
    20260904: 'de2d51616b8866275306ec3289e1e7574b7dd290bd83d3cb1721a6ba568cc4d3',
    20260905: '2be796dab237b7b4e15cbb4674ca2cfcfb589c056977a9124a4c7e9a51f26082',
}
ARMS = ('linear_continue', 'linear_reset', 'candidate_reset')
INPUT_NAMES = ('images', 'calibration', 'ego_history', 'ego_history_mask', 'route_xy', 'route_mask')
CACHE_NAMES = ('fused', 'candidate_xy', 'candidate_speed', 'target_xy',
               'target_speed_mps', 'target_yaw_rad', 'target_valid', 'original_logits')
ABSOLUTE_LIMITS = {'ade_1p0s_m': 0.5, 'ade_3p0s_m': 1.0, 'ade_6p4s_m': 2.0, 'fde_6p4s_m': 4.0}
PLAN_PATH = REPO / 'config/portable_e2e_frozen_selector_20260908.json'


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def check_deadline(value):
    # HH_260906 - The requested end time is a hard boundary, not permission to launch unbounded work.
    deadline = datetime.fromisoformat(value.replace('Z', '+00:00'))
    if deadline.tzinfo is None or deadline.utcoffset() is None:
        raise ContractError('deadline must include an explicit timezone')
    if datetime.now(timezone.utc) >= deadline:
        raise ContractError('requested work deadline has been reached')
    return deadline


def load_plan(path=PLAN_PATH):
    # HH_260906 - Bind the readable plan to executable constants instead of publishing an unenforced proposal.
    plan = _loads_json(path.read_text(), 'frozen selector plan')
    fixed = {'schema': 'portable_e2e.frozen_selector_plan.v1',
        'campaign_id': 'hh260908-frozen-selector-3seeds-v1', 'parent_campaign_id': PARENT_CAMPAIGN,
        'dataset': 'datasets/prepared/carla-common10-30kph-five-episodes-20260907-v3',
        'manifest_sha256': MANIFEST_SHA256, 'gpu_uuid': GPU_UUID, 'seeds': list(PARENT_SHA256),
        'arms': list(ARMS), 'train_samples': 1147, 'val_samples': 337,
        'steps': 1540, 'batch_size': 4, 'learning_rate': 0.0001, 'weight_decay': 0.0001,
        'candidate_score_weight': 0.1, 'maximum_gradient_norm': 5.0,
        'deadline_utc': '2026-09-08T01:00:00+00:00', 'absolute_limits_m': ABSOLUTE_LIMITS}
    if any(plan.get(name) != value for name, value in fixed.items()):
        raise ContractError('frozen selector plan differs from the reviewed experiment')
    return plan


def tensor_mapping_sha256(values):
    # HH_260906 - Bind keys, shapes and dtypes as well as bytes so equal raw buffers cannot disguise changed tensors.
    digest = hashlib.sha256()
    for name, tensor in sorted(values.items()):
        if not isinstance(tensor, torch.Tensor):
            raise ContractError('state hash accepts named tensors only')
        value = tensor.detach().cpu().contiguous()
        header = json.dumps([name, str(value.dtype), list(value.shape)], separators=(',', ':')).encode()
        payload = value.numpy().tobytes()
        digest.update(len(header).to_bytes(8, 'big'))
        digest.update(header)
        digest.update(len(payload).to_bytes(8, 'big'))
        digest.update(payload)
    return digest.hexdigest()


def extract_cache(model, dataset, *, device, checkpoint_sha256, deadline_utc):
    # HH_260906 - Observe the existing fusion output without changing model.forward or recomputing a different encoder.
    from portable_e2e.frozen_selector import CachedSelectorData

    if dataset.split not in ('train', 'val'):
        raise ContractError('frozen selection permits train and development val only')
    if model.training or any(parameter.requires_grad for parameter in model.parameters()):
        raise ContractError('the complete generator must be eval-only and frozen')
    state_before = tensor_mapping_sha256(model.state_dict())
    arrays = {name: [] for name in CACHE_NAMES}
    extra = {name: [] for name in ('route_xy', 'route_mask', 'current_speed_mps')}
    sample_ids, observed_context = [], []

    def capture_context(_module, _inputs, output):
        observed_context.append(output.detach().clone())

    hook = model.fusion.register_forward_hook(capture_context)
    try:
        with torch.no_grad():
            for batch in DataLoader(dataset, batch_size=4, shuffle=False, num_workers=0):
                check_deadline(deadline_utc)
                observed_context.clear()
                tensors = {name: batch[name].to(device) for name in INPUT_NAMES}
                xy, speed, logits = model(*(tensors[name] for name in INPUT_NAMES))
                if len(observed_context) != 1:
                    raise ContractError('generator fusion must execute exactly once per batch')
                values = (observed_context[0], xy, speed, batch['target_xy'],
                          batch['target_speed_mps'], batch['target_yaw_rad'], batch['target_valid'], logits)
                for name, value in zip(CACHE_NAMES, values):
                    arrays[name].append(value.detach().cpu().clone())
                for name in ('route_xy', 'route_mask'):
                    extra[name].append(batch[name].detach().cpu().clone())
                extra['current_speed_mps'].append(batch['ego_history'][:, -1,
                    FEATURE_NAMES.index('velocity_x_mps')].detach().cpu().clone())
                sample_ids.extend(batch['sample_id'])
    finally:
        hook.remove()
    expected_ids = tuple(example.token for example in dataset.examples)
    if tuple(sample_ids) != expected_ids or len(set(sample_ids)) != len(dataset):
        raise ContractError('cache must cover the exact ordered split without duplicates')
    if tensor_mapping_sha256(model.state_dict()) != state_before:
        raise ContractError('generator parameters or buffers changed while caching')
    data = CachedSelectorData(split=dataset.split, sample_ids=tuple(sample_ids),
        episode_ids=tuple(example.episode_id for example in dataset.examples),
        source_checkpoint_sha256=checkpoint_sha256, corpus_fingerprint_sha256=CORPUS_SHA256,
        dataset_fingerprint_sha256=dataset.fingerprint_sha256,
        **{name: torch.cat(values) for name, values in arrays.items()})
    return data, {name: torch.cat(values) for name, values in extra.items()}, state_before


def cache_payload(data, extra):
    # HH_260906 - This research cache is explicitly not a normal trainer or runtime checkpoint.
    return {'checkpoint_id': CACHE_ID, 'split': data.split, 'sample_ids': list(data.sample_ids),
        'episode_ids': list(data.episode_ids), 'source_checkpoint_sha256': data.source_checkpoint_sha256,
        'corpus_fingerprint_sha256': data.corpus_fingerprint_sha256,
        'dataset_fingerprint_sha256': data.dataset_fingerprint_sha256, 'cache_digest': data.digest(),
        'tensors': {name: getattr(data, name) for name in CACHE_NAMES}, 'extra': extra,
        'vehicle_control_approved': False}


def audit_and_render(data, extra, logits, output_dir, title):
    # HH_260906 - Frozen candidates still require selected-path checks because a new scorer can choose an unsafe candidate.
    if data.split != 'val' or len(data.sample_ids) != 337:
        raise ContractError('route audit and fixed-phase figures require the complete val337')
    if tuple(logits.shape) != (337, 6) or not bool(torch.isfinite(logits).all()):
        raise ContractError('route audit requires finite six-candidate logits')
    gate = RuntimeGateConfig()
    accumulator = _AuditAccumulator(gate)
    rows = []
    for index in range(337):
        result = audit_prediction(data.candidate_xy[index].tolist(), data.candidate_speed[index].tolist(),
            logits[index].tolist(), gate, current_speed_mps=float(extra['current_speed_mps'][index]))
        accumulator.add(result)
        rows.append(result)
    output_dir.mkdir(parents=True, exist_ok=False)
    rendered = []
    for phase in range(6):
        index = phase * 336 // 5
        path = output_dir / f'val_phase_{index:03d}.png'
        render_trajectory_png(path, route_xy=extra['route_xy'][index][extra['route_mask'][index]].tolist(),
            target_xy=data.target_xy[index].tolist(), target_valid=data.target_valid[index].tolist(),
            candidate_xy=data.candidate_xy[index].tolist(), candidate_logits=logits[index].tolist(),
            title=f'{title}; val337 index {index}; frozen candidates', width=1200, height=900)
        rendered.append({'index': index, 'sample_id': data.sample_ids[index],
            'path': path.name, 'sha256': file_sha256(path)})
    return {'gate': {'source': RUNTIME_GATE_ID, **asdict(gate)}, 'summary': accumulator.report(), 'per_sample': rows,
        'rendered': rendered, 'vehicle_control_approved': False,
        'interpretation': 'Offline cached predictions; not live timing or learned vehicle control.'}


def validate_environment(expected_source_commit, output_dir, deadline_utc):
    check_deadline(deadline_utc)
    if Path(sys.prefix) != WORKSPACE / 'venvs/py312':
        raise ContractError('only the existing personal py312 venv is authorized')
    if os.environ.get('CUDA_VISIBLE_DEVICES') != GPU_UUID:
        raise ContractError('only the explicitly pinned physical GPU0 is authorized')
    if os.environ.get('OMP_NUM_THREADS') != '4' or os.environ.get('MKL_NUM_THREADS') != '4':
        raise ContractError('head-only research requires four OMP/MKL CPU threads')
    output_dir = output_dir.absolute()
    allowed = WORKSPACE / 'runs/campaigns/hh260908-frozen-selector-3seeds-v1'
    if output_dir.parent != allowed or output_dir.exists() or output_dir.is_symlink():
        raise ContractError('output must be a new seed directory in the reviewed personal campaign')
    if allowed.exists() and (allowed.is_symlink() or not allowed.is_dir()):
        raise ContractError('campaign parent must be a regular directory')
    return source_provenance(expected_source_commit)


def run(seed, expected_source_commit, output_dir, deadline_utc):
    from portable_e2e.frozen_selector import fit_frozen_selector

    source = validate_environment(expected_source_commit, output_dir, deadline_utc)
    plan = load_plan()
    if deadline_utc != plan['deadline_utc']:
        raise ContractError('command deadline differs from the predeclared user work boundary')
    plan_sha = file_sha256(PLAN_PATH)
    if seed not in PARENT_SHA256 or output_dir.name != f'seed_{seed}':
        raise ContractError('only the three paired parent checkpoints and seed directory names are reviewed')
    script_sha = file_sha256(Path(__file__))
    core_sha = file_sha256(REPO / 'portable_e2e/frozen_selector.py')
    dataset_root = WORKSPACE / 'datasets/prepared/carla-common10-30kph-five-episodes-20260907-v3'
    parent = WORKSPACE / 'runs/campaigns' / PARENT_CAMPAIGN / f'seed_{seed}/C_expanded_data'
    checkpoint = parent / 'training/checkpoints/latest.pt'
    require_hash(dataset_root / 'dataset.json', MANIFEST_SHA256)
    require_hash(checkpoint, PARENT_SHA256[seed])
    parent_report_sha = {name: file_sha256(parent / name) for name in
        ('training/run.json', 'evaluation/metrics.json', 'gate_v8.json')}
    output_dir.mkdir(parents=True, exist_ok=False)
    started, tick = utc_now(), time.monotonic()
    state = {'schema': SCHEMA, 'status': 'PREFLIGHT', 'seed': seed, 'started_at_utc': started,
        'script_sha256': script_sha, 'core_sha256': core_sha, 'source': source,
        'plan': plan, 'plan_sha256': plan_sha,
        'parent_checkpoint_sha256': PARENT_SHA256[seed], 'parent_report_sha256': parent_report_sha,
        'manifest_sha256': MANIFEST_SHA256, 'deadline_utc': deadline_utc, 'arms': [],
        'vehicle_control_approved': False, 'automatic_promotion': False,
        'test_evaluated': False, 'test_used_for_training_or_selection': False,
        'dataset_integrity_scope': 'The entire declared corpus, including test, is checked for integrity; no test model predictions or outcome analysis.'}
    _atomic_new_json(output_dir / 'started.json', state)
    try:
        # HH_260906 - Use the same cooperative lease as whole-model campaigns and never terminate another GPU user.
        with (WORKSPACE / 'runs/campaigns/.gpu0_training.lock').open('a+') as lease:
            fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
            assert_gpu_idle(REPO)
            torch.set_num_threads(4)
            torch.set_num_interop_threads(4)
            device = torch.device('cuda:0')
            _seed_everything(seed, device)
            loaded_val = load_training_examples(dataset_root, split='val', mode='planning', check_image_hashes=True)
            if loaded_val.validation_report['dataset_fingerprint_sha256'] != CORPUS_SHA256:
                raise ContractError('frozen study requires the exact v3 corpus')
            payload, config, training_ids, parent_provenance = _read_checkpoint_for_audit(
                checkpoint_path=checkpoint, expected_checkpoint_sha256=PARENT_SHA256[seed],
                corpus_fingerprint_sha256=CORPUS_SHA256)
            if config.model_id != PHYSICAL_MODEL_ID or payload['train_config']['seed'] != seed:
                raise ContractError('parent model ID or training seed differs from reviewed C checkpoint')
            loss_config = checkpoint_loss_config(payload)
            from portable_e2e.losses import TrajectoryLossConfig
            if loss_config.to_dict() != TrajectoryLossConfig().to_dict():
                raise ContractError('parent composite loss must exactly match the frozen original loss configuration')
            val = Common10TorchDataset(loaded_val.examples, config, verify_image_sha256=True, split='val')
            validate_split(val, 'val', payload, training_ids)
            model, provenance = _validate_checkpoint_and_model(val, payload=payload, model_config=config,
                training_episode_ids=training_ids, checkpoint_provenance=parent_provenance, device=device)
            loaded_train = load_training_examples(dataset_root, split='train', mode='planning', check_image_hashes=True)
            if loaded_train.validation_report['dataset_fingerprint_sha256'] != CORPUS_SHA256:
                raise ContractError('training corpus differs from the frozen validation corpus')
            train = Common10TorchDataset(loaded_train.examples, config, verify_image_sha256=True, split='train')
            validate_split(train, 'train', payload, training_ids)
            cache, extras = {}, {}
            generator_sha = tensor_mapping_sha256(model.state_dict())
            for dataset in (train, val):
                print(json.dumps({'event': 'CACHE_START', 'seed': seed, 'split': dataset.split}), flush=True)
                data, extra, initial = extract_cache(model, dataset, device=device,
                    checkpoint_sha256=PARENT_SHA256[seed], deadline_utc=deadline_utc)
                if initial != generator_sha:
                    raise ContractError('generator changed between train and val cache extraction')
                cache[dataset.split], extras[dataset.split] = data, extra
                _atomic_torch_save(output_dir / f'{dataset.split}_cache.pt', cache_payload(data, extra))
                print(json.dumps({'event': 'CACHE_COMPLETE', 'seed': seed, 'split': dataset.split,
                    'sample_count': len(data.sample_ids), 'cache_digest': data.digest()}), flush=True)
            baseline_audit = audit_and_render(cache['val'], extras['val'], cache['val'].original_logits,
                output_dir / 'original_c_routes', f'C seed {seed}')
            _atomic_new_json(output_dir / 'original_c_geometry.json', baseline_audit)
            state.update(provenance=provenance, loss_config=loss_config.to_dict(),
                generator_state_sha256=generator_sha, model_config=config.to_dict(),
                torch_version=str(torch.__version__), gpu_uuid=GPU_UUID,
                cache_files={split: {'sha256': file_sha256(output_dir / f'{split}_cache.pt'),
                    'cache_digest': data.digest(), 'sample_count': len(data.sample_ids)} for split, data in cache.items()})
            for arm in ARMS:
                check_deadline(deadline_utc)
                arm_dir = output_dir / arm
                arm_dir.mkdir(exist_ok=False)
                with (arm_dir / 'metrics.jsonl').open('x', encoding='utf-8') as history:
                    def progress(record):
                        check_deadline(deadline_utc)
                        history.write(json.dumps(record, allow_nan=False) + '\n')
                        history.flush()
                        if record.get('step', record.get('global_step', 0)) % 154 == 0:
                            print(json.dumps({'event': 'HEAD_PROGRESS', 'seed': seed, 'arm': arm,
                                'record': record}, allow_nan=False), flush=True)
                    result = fit_frozen_selector(cache['train'], cache['val'], original_head=model.candidate_head,
                        arm=arm, seed=seed, device=str(device), callback=progress)
                if tensor_mapping_sha256(model.state_dict()) != generator_sha:
                    raise ContractError('scorer fitting changed the original generator or head')
                geometry = audit_and_render(cache['val'], extras['val'], result['final_logits_cpu']['val'],
                    arm_dir / 'routes', f'{arm} seed {seed}')
                artifact = {'checkpoint_id': result['report']['artifact_id'],
                    'head_state_dict': result['head_state_dict'], 'report': result['report'],
                    'parent_checkpoint_sha256': PARENT_SHA256[seed], 'source_commit': expected_source_commit,
                    'vehicle_control_approved': False}
                _atomic_torch_save(arm_dir / 'head_only.pt', artifact)
                _atomic_torch_save(arm_dir / 'final_logits.pt', result['final_logits_cpu'])
                _atomic_new_json(arm_dir / 'report.json', result['report'])
                _atomic_new_json(arm_dir / 'geometry.json', geometry)
                record = {'arm': arm, 'status': 'COMPLETE_NOT_PROMOTED',
                    'files': {name: file_sha256(arm_dir / name) for name in
                        ('metrics.jsonl', 'head_only.pt', 'final_logits.pt', 'report.json', 'geometry.json')}}
                state['arms'].append(record)
                print(json.dumps({'event': 'HEAD_COMPLETE', 'seed': seed, **record}), flush=True)
            torch.cuda.synchronize(device)
            generator_after = tensor_mapping_sha256(model.state_dict())
            if generator_after != generator_sha:
                raise ContractError('final generator state differs from the original frozen state')
        # HH_260906 - Recheck immutable private inputs and imported source after every owned seed experiment.
        require_hash(checkpoint, PARENT_SHA256[seed])
        require_hash(dataset_root / 'dataset.json', MANIFEST_SHA256)
        require_hash(Path(__file__), script_sha)
        require_hash(PLAN_PATH, plan_sha)
        require_hash(REPO / 'portable_e2e/frozen_selector.py', core_sha)
        if source_provenance(expected_source_commit) != source:
            raise ContractError('source changed during frozen scorer experiment')
        if {name: file_sha256(parent / name) for name in parent_report_sha} != parent_report_sha:
            raise ContractError('parent training/evaluation evidence changed')
        for split, cache_record in state['cache_files'].items():
            require_hash(output_dir / f'{split}_cache.pt', cache_record['sha256'])
        state.update(status='HEAD_TRAIN_EVAL_AUDIT_COMPLETE_NOT_PROMOTED', completed_at_utc=utc_now(),
            wall_seconds=time.monotonic() - tick, generator_state_after_sha256=generator_after)
        _atomic_new_json(output_dir / 'result.json', state)
        return state
    except BaseException as error:
        # HH_260906 - Preserve partial traces and publish failure without attempting unsafe resume or deletion.
        state.update(status='STOPPED_FAILURE_NO_PROMOTION', failed_at_utc=utc_now(),
            error_type=type(error).__name__, error=str(error))
        _atomic_new_json(output_dir / 'failure.json', state)
        raise


def validate_seed_result(path, seed, source_commit, plan_sha):
    # HH_260906 - A completed child process must also provide all three immutable head result files.
    report = _loads_json(path.read_text(), 'frozen selector seed result')
    expected = {'schema': SCHEMA, 'status': 'HEAD_TRAIN_EVAL_AUDIT_COMPLETE_NOT_PROMOTED',
        'seed': seed, 'parent_checkpoint_sha256': PARENT_SHA256[seed], 'plan_sha256': plan_sha,
        'vehicle_control_approved': False, 'automatic_promotion': False,
        'test_evaluated': False, 'test_used_for_training_or_selection': False}
    if any(report.get(key) != value for key, value in expected.items()):
        raise ContractError('child seed completion differs from the frozen experiment')
    if report.get('source', {}).get('diagnostic_source_commit') != source_commit:
        raise ContractError('child used a different imported source commit')
    generator_sha = report.get('generator_state_sha256', '')
    if not isinstance(generator_sha, str) or re.fullmatch('[0-9a-f]{64}', generator_sha) is None:
        raise ContractError('child generator fingerprint is missing or malformed')
    if generator_sha != report.get('generator_state_after_sha256'):
        raise ContractError('child changed its frozen generator')
    if set(report.get('cache_files', {})) != {'train', 'val'}:
        raise ContractError('child is missing its separate train and val caches')
    for split, count in (('train', 1147), ('val', 337)):
        cache_record = report['cache_files'][split]
        if cache_record.get('sample_count') != count:
            raise ContractError('child cache does not cover the complete reviewed split')
        require_hash(path.parent / f'{split}_cache.pt', cache_record['sha256'])
    if [arm.get('arm') for arm in report.get('arms', [])] != list(ARMS):
        raise ContractError('child completion does not cover all three ordered scorer arms')
    for arm in report['arms']:
        if arm.get('status') != 'COMPLETE_NOT_PROMOTED' or set(arm.get('files', {})) != {
                'metrics.jsonl', 'head_only.pt', 'final_logits.pt', 'report.json', 'geometry.json'}:
            raise ContractError('child arm is missing completed evidence')
        for name, expected_sha in arm['files'].items():
            require_hash(path.parent / arm['arm'] / name, expected_sha)
        head = _loads_json((path.parent / arm['arm'] / 'report.json').read_text(), 'frozen head report')
        expected_head = {'artifact_id': 'portable_e2e.frozen_selector_research_head.v1',
            'status': 'RESEARCH_HEAD_TRAINING_COMPLETE_NOT_PROMOTED', 'arm': arm['arm'], 'seed': seed,
            'source_checkpoint_sha256': PARENT_SHA256[seed], 'steps': 1540, 'batch_size': 4,
            'vehicle_control_approved': False, 'automatic_promotion': False}
        if any(head.get(key) != value for key, value in expected_head.items()):
            raise ContractError('child scorer report differs from the frozen training contract')
        if head.get('final_state', {}).get('global_step') != 1540 or head['final_state'].get('samples_seen') != 6155:
            raise ContractError('child scorer did not finish the reviewed sample exposure budget')
    return report


def run_campaign(expected_source_commit, output_dir, deadline_utc):
    # HH_260906 - Bound and isolate each child process so SSH disconnects do not orphan an untracked GPU run.
    if Path(sys.prefix) != WORKSPACE / 'venvs/py312':
        raise ContractError('campaign requires the existing personal py312 interpreter')
    plan = load_plan()
    if deadline_utc != plan['deadline_utc']:
        raise ContractError('campaign deadline differs from the user boundary')
    check_deadline(deadline_utc)
    source = source_provenance(expected_source_commit)
    expected_root = WORKSPACE / 'runs/campaigns' / plan['campaign_id']
    if output_dir.absolute() != expected_root or output_dir.exists() or output_dir.is_symlink():
        raise ContractError('campaign output must be the new reviewed personal experiment folder')
    if output_dir.parent.resolve() != output_dir.parent:
        raise ContractError('campaign parent must not traverse symlinks')
    plan_sha, script_sha = file_sha256(PLAN_PATH), file_sha256(Path(__file__))
    output_dir.mkdir(exist_ok=False)
    provenance = output_dir / 'provenance'
    provenance.mkdir()
    shutil.copy2(Path(__file__), provenance / 'active_runner.py')
    shutil.copy2(PLAN_PATH, provenance / 'plan.json')
    env = stage_environment()
    env.update(OMP_NUM_THREADS='4', MKL_NUM_THREADS='4')
    state = {'schema': 'portable_e2e.frozen_selector_campaign.v1', 'status': 'PREFLIGHT',
        'created_at_utc': utc_now(), 'source': source, 'source_commit': expected_source_commit,
        'plan': plan, 'plan_sha256': plan_sha, 'runner_sha256': script_sha, 'stages': [],
        'vehicle_control_approved': False, 'automatic_promotion': False,
        'test_evaluated': False, 'test_used_for_training_or_selection': False,
        'dataset_integrity_scope': 'The entire declared corpus, including test, is checked for integrity; no test model predictions or outcome analysis.'}
    _atomic_json(output_dir / 'status.json', state)
    try:
        for seed in plan['seeds']:
            deadline = check_deadline(deadline_utc)
            if source_provenance(expected_source_commit) != source:
                raise ContractError('source changed before the next frozen selector seed')
            assert_gpu_idle(REPO)
            child_output = output_dir / f'seed_{seed}'
            command = [sys.executable, str(Path(__file__).resolve()), '--seed', str(seed),
                '--expected-source-commit', expected_source_commit, '--output-dir', str(child_output),
                '--deadline-utc', deadline_utc]
            record = {'seed': seed, 'status': 'RUNNING', 'started_at_utc': utc_now(), 'command': command}
            state['stages'].append(record)
            state['status'] = 'RUNNING'
            _atomic_json(output_dir / 'status.json', state)
            remaining = (deadline - datetime.now(timezone.utc)).total_seconds()
            with (output_dir / f'seed_{seed}.log').open('x', encoding='utf-8') as log:
                code = run_owned_stage(command, REPO, env, log, timeout=min(1800.0, remaining))
            record['returncode'] = code
            if code:
                raise ContractError(f'owned seed {seed} failed; preserve and inspect its log')
            report_path = child_output / 'result.json'
            validate_seed_result(report_path, seed, expected_source_commit, plan_sha)
            record.update(status='COMPLETE_NOT_PROMOTED', finished_at_utc=utc_now(),
                result_sha256=file_sha256(report_path), completed_head_fits=3)
            _atomic_json(output_dir / 'status.json', state)
            print(json.dumps(record), flush=True)
        if source_provenance(expected_source_commit) != source:
            raise ContractError('source changed before campaign completion')
        require_hash(PLAN_PATH, plan_sha)
        require_hash(Path(__file__), script_sha)
        state.update(status='FROZEN_SELECTOR_CAMPAIGN_COMPLETE_NOT_PROMOTED',
            completed_at_utc=utc_now(), completed_parent_caches=3, completed_head_fits=9)
    except BaseException as error:
        if state['stages'] and state['stages'][-1]['status'] == 'RUNNING':
            state['stages'][-1].update(status='FAILED', finished_at_utc=utc_now())
        state.update(status='STOPPED_FAILURE_NO_PROMOTION', error_type=type(error).__name__, error=str(error))
        raise
    finally:
        state['updated_at_utc'] = utc_now()
        _atomic_json(output_dir / 'status.json', state)
    return state


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument('--seed', type=int, choices=tuple(PARENT_SHA256))
    selection.add_argument('--all-seeds', action='store_true')
    parser.add_argument('--expected-source-commit', required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--deadline-utc', required=True)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    with termination_guard():
        result = (run_campaign(args.expected_source_commit, args.output_dir, args.deadline_utc)
            if args.all_seeds else run(args.seed, args.expected_source_commit, args.output_dir, args.deadline_utc))
    print(json.dumps({'status': result['status'], 'seed': args.seed,
        'completed_arms': result.get('completed_head_fits', len(result.get('arms', [])))}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
