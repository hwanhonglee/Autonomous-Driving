#!/usr/bin/env python3
"""HH_260906 - Run a bounded, predeclared GPU0-only training/validation campaign."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time


# HH_260906 - Derive the approved personal layout without publishing an account-specific home path.
WORKSPACE = Path.home() / 'personal/hwanhong/portable_e2e'
GPU_UUID = 'GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5'
DATA_EXPANSION_SCHEMA = 'portable_e2e.data_expansion_campaign.v1'
SELECTOR_WEIGHT_SCHEMA = 'portable_e2e.selector_weight_campaign.v1'
CANDIDATE_RANK_SCHEMA = 'portable_e2e.candidate_rank_campaign.v1'
NO_ACCEL_DEVELOPMENT_SCHEMA = 'portable_e2e.no_accel_development_campaign.v1'
# HH_260906 - Add one prospective cost-aware selector experiment without changing historical campaign contracts.
CANDIDATE_REGRET_SCHEMA = 'portable_e2e.candidate_regret_campaign.v1'
CANDIDATE_REGRET_ARMS = {'A_original_loss': 0.0001, 'B_cost_aware_selector': 0.0001}
CANDIDATE_REGRET_WEIGHTS = {'A_original_loss': 0.0, 'B_cost_aware_selector': 0.1}
CANDIDATE_REGRET_STAGE_TIMEOUTS = {'train': 600, 'evaluate': 120, 'audit': 120}
CANDIDATE_REGRET_DEADLINE = '2026-09-09T01:00:00Z'
SUCCESSOR_SCHEMAS = (DATA_EXPANSION_SCHEMA, SELECTOR_WEIGHT_SCHEMA, CANDIDATE_RANK_SCHEMA)
NO_ACCEL_MODELS = {
    'A_physical_input': 'portable_e2e/config/perspective_trajectory_physical_v1.model.json',
    'B_no_accel_input': 'portable_e2e/config/perspective_trajectory_physical_no_accel_v1.model.json',
}
NO_ACCEL_MODEL_SHA256 = {
    'A_physical_input': 'e96e31c96cafa41b57b67b9531ae9fff6bf21fd7e418ea78f9062fcb1dcfe74d',
    'B_no_accel_input': '0f941332046854d239d676e83a4ccbc22e60436f3894dcbfe6871002899ed012',
}
# HH_260906 - Compare six DRIVE candidates with six DRIVE plus six STOP candidates; this changes no acceptance gate.
STOPMIX_SCHEMA = 'portable_e2e.stopmix_development_campaign.v1'
STOPMIX_ARMS = {'A_physical_drive': 0.0001, 'B_drive_stop_mix': 0.0001}
STOPMIX_MODELS = {
    'A_physical_drive': 'portable_e2e/config/perspective_trajectory_physical_v1.model.json',
    'B_drive_stop_mix': 'portable_e2e/config/perspective_trajectory_physical_stopmix_v1.model.json',
}
STOPMIX_MODEL_SHA256 = {'A_physical_drive': NO_ACCEL_MODEL_SHA256['A_physical_input'],
    'B_drive_stop_mix': '29e537e8b216cff0216772624f670ece0f0849f0f2ec55c0b726f11c1d74b602'}
STOPMIX_PRIMITIVE_SHA256 = '4021eb2ba6d84260b6b658114fbe31a1e7ef826121e4d07690bfd4154919a92c'
STOPMIX_SOURCE_PATHS = ('portable_e2e/model.py', 'portable_e2e/stop_primitive_research.py',
    'portable_e2e/train.py', 'portable_e2e/losses.py', 'portable_e2e/torch_dataset.py',
    'portable_e2e/evaluate.py', 'portable_e2e/audit_runtime.py', 'portable_e2e/runtime_contract.py',
    'scripts/e2e/run_portable_training_campaign.py', *STOPMIX_MODELS.values())
EXPANDED_DATASET = 'datasets/prepared/carla-common10-30kph-five-episodes-20260907-v3'
EXPANDED_MANIFEST_SHA256 = '18262e5aa4abbb3e03e35e379b5da1e5ce7fd339a9a8942e02b58ca737f7242c'
PREREQUISITE_CAMPAIGN = 'hh260907-physical-v1-lr-ab-3seeds-v1'
DATA_EXPANSION_CAMPAIGN = 'hh260907-physical-v1-data-expansion-3seeds-v1'
HISTORICAL_SOURCE_COMMIT = '081a71f7fc014d2864b790b0b0cae7378ae18e4c'


def now():
    return datetime.now(timezone.utc).isoformat()


def digest(path):
    hasher = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            hasher.update(block)
    return hasher.hexdigest()


def validate_plan(plan):
    # HH_260906 - Accept only reviewed experiments; the new architecture never changes the physical safety gates.
    if plan['schema'] not in ('portable_e2e.lr_ab_campaign.v1', *SUCCESSOR_SCHEMAS, NO_ACCEL_DEVELOPMENT_SCHEMA, CANDIDATE_REGRET_SCHEMA, STOPMIX_SCHEMA):
        raise ValueError('unsupported campaign schema')
    if not re.fullmatch(r'[a-z0-9][a-z0-9_-]{0,100}', plan['campaign_id']):
        raise ValueError('unsafe campaign identifier')
    if plan['gpu_uuid'] != GPU_UUID or plan['seeds'] != [20260903, 20260904, 20260905]:
        raise ValueError('GPU or paired seeds differ from reviewed scope')
    expanded = plan['schema'] == DATA_EXPANSION_SCHEMA
    selector = plan['schema'] == SELECTOR_WEIGHT_SCHEMA
    ranking = plan['schema'] == CANDIDATE_RANK_SCHEMA
    no_accel = plan['schema'] == NO_ACCEL_DEVELOPMENT_SCHEMA
    regret = plan['schema'] == CANDIDATE_REGRET_SCHEMA
    stopmix = plan['schema'] == STOPMIX_SCHEMA
    expected_arms = (STOPMIX_ARMS if stopmix else CANDIDATE_REGRET_ARMS if regret else
        {'A_physical_input': 0.0001, 'B_no_accel_input': 0.0001} if no_accel else
        {'E_candidate_rank': 0.0001} if ranking else {'D_selector_weight': 0.0001} if selector else
        {'C_expanded_data': 0.0001} if expanded else {'A_baseline': 0.0001, 'B_lower_lr': 0.00003})
    if plan['arms'] != expected_arms:
        raise ValueError('unreviewed learning-rate arms')
    if plan['steps'] != 1540 or plan['batch_size'] != 4 or plan['split'] != 'val':
        raise ValueError('unreviewed duration, batch size or evaluation split')
    if not re.fullmatch(r'[0-9a-f]{40}', plan['source_commit']):
        raise ValueError('source commit must be pinned')
    for name in (('dataset',) if no_accel or stopmix else ('dataset', 'model_config')):
        value = Path(plan[name])
        if value.is_absolute() or '..' in value.parts:
            raise ValueError(f'{name} must be a contained relative path')
    if no_accel:
        # HH_260906 - This exploratory v3 ablation trains both arms from scratch; it does not reuse a historical baseline.
        expected = {'campaign_id': 'hh260909-no-accel-development-ab-3seeds-v1',
            'dataset': EXPANDED_DATASET, 'dataset_manifest_sha256': EXPANDED_MANIFEST_SHA256,
            'model_configs': NO_ACCEL_MODELS, 'expected_train_samples': 1147, 'expected_val_samples': 337}
        forbidden = ('model_config', 'prerequisite_campaign_id', 'baseline_campaign_id', 'baseline_source_commit',
            'prerequisite_timeout_seconds', 'resume', 'checkpoint')
        if (any(plan.get(name) != value for name, value in expected.items())
                or any(name in plan for name in forbidden)
                or list(plan['arms']) != list(NO_ACCEL_MODELS)
                or plan.get('candidate_score_weight', 0.1) != 0.1):
            raise ValueError('unreviewed no-accel development dataset, paired models, arm order or fresh-start scope')
    if regret:
        # HH_260906 - Fix all seeds, both fresh arms, the auxiliary weights and the overnight finish budget before dispatch.
        expected = {'campaign_id': 'hh260909-candidate-regret-ab-3seeds-v1',
            'dataset': EXPANDED_DATASET, 'dataset_manifest_sha256': EXPANDED_MANIFEST_SHA256,
            'model_config': NO_ACCEL_MODELS['A_physical_input'],
            'model_config_sha256': NO_ACCEL_MODEL_SHA256['A_physical_input'],
            'candidate_score_weight': 0.1, 'candidate_regret_weights': CANDIDATE_REGRET_WEIGHTS,
            'expected_train_samples': 1147, 'expected_val_samples': 337,
            'finish_before_utc': CANDIDATE_REGRET_DEADLINE,
            'stage_timeout_seconds': CANDIDATE_REGRET_STAGE_TIMEOUTS,
            'finish_reserve_seconds': 300}
        forbidden = ('model_configs', 'prerequisite_campaign_id', 'baseline_campaign_id', 'baseline_source_commit',
            'prerequisite_timeout_seconds', 'resume', 'checkpoint')
        if (any(json.dumps(plan.get(name), sort_keys=True) != json.dumps(value, sort_keys=True)
                for name, value in expected.items())
                or any(name in plan for name in forbidden)
                or list(plan['arms']) != list(CANDIDATE_REGRET_ARMS)):
            raise ValueError('unreviewed candidate-regret dataset, loss, fresh-arm order or deadline')
    if stopmix:
        # HH_260906 - Reserve separate post-campaign behavior diagnostics without pretending they are the eighteen completed stages.
        expected = {'campaign_id': 'hh260909-stopmix-development-ab-3seeds-v1',
            'dataset': EXPANDED_DATASET, 'dataset_manifest_sha256': EXPANDED_MANIFEST_SHA256,
            'model_configs': STOPMIX_MODELS, 'model_config_sha256': STOPMIX_MODEL_SHA256,
            'candidate_score_weight': 0.1, 'stop_primitive_sha256': STOPMIX_PRIMITIVE_SHA256,
            'expected_train_samples': 1147, 'expected_val_samples': 337,
            'finish_before_utc': CANDIDATE_REGRET_DEADLINE,
            'stage_timeout_seconds': CANDIDATE_REGRET_STAGE_TIMEOUTS,
            'finish_reserve_seconds': 300, 'behavior_analysis_count': 6,
            'behavior_analysis_timeout_seconds': 180, 'steps': 1540, 'batch_size': 4,
            'seeds': [20260903, 20260904, 20260905], 'arms': STOPMIX_ARMS}
        forbidden = ('model_config', 'prerequisite_campaign_id', 'baseline_campaign_id', 'baseline_source_commit',
            'prerequisite_timeout_seconds', 'resume', 'checkpoint', 'candidate_regret_weights', 'candidate_regret_weight')
        if (any(json.dumps(plan.get(name), sort_keys=True) != json.dumps(value, sort_keys=True)
                for name, value in expected.items()) or any(name in plan for name in forbidden)
                or list(plan['arms']) != list(STOPMIX_ARMS)):
            raise ValueError('unreviewed STOPMIX models, loss, fresh-arm scope or finish budget')
    if expanded or selector or ranking:
        expected = {
            'campaign_id': ('hh260907-candidate-rank-3seeds-v1' if ranking else
                'hh260907-physical-v1-selector-weight-3seeds-v1' if selector else DATA_EXPANSION_CAMPAIGN),
            'dataset': EXPANDED_DATASET,
            'dataset_manifest_sha256': EXPANDED_MANIFEST_SHA256,
            'model_config': ('portable_e2e/config/perspective_trajectory_candidate_rank_v1.model.json' if ranking else
                'portable_e2e/config/perspective_trajectory_physical_v1.model.json'),
            'expected_train_samples': 1147, 'expected_val_samples': 337,
            'prerequisite_campaign_id': DATA_EXPANSION_CAMPAIGN if selector or ranking else PREREQUISITE_CAMPAIGN,
            'prerequisite_timeout_seconds': 3600,
        }
        if ranking:
            expected.update(candidate_score_weight=0.1, baseline_campaign_id=DATA_EXPANSION_CAMPAIGN,
                baseline_source_commit=HISTORICAL_SOURCE_COMMIT)
        elif selector:
            expected['candidate_score_weight'] = 0.5
        else:
            expected['source_commit'] = HISTORICAL_SOURCE_COMMIT
        if any(plan.get(name) != value for name, value in expected.items()):
            raise ValueError('unreviewed successor dataset, source, counts, score weight or prerequisite')
    return {'train_samples': 1147 if expanded or selector or ranking or no_accel or regret or stopmix else 613, 'val_samples': 337,
        'run_count': 3 if expanded or selector or ranking else 6,
        'stage_count': 9 if expanded or selector or ranking else 18}


def verify_dataset_manifest(plan, dataset):
    # HH_260906 - Bind the new corpus manifest before and after every stage without opening held-out test samples.
    validate_plan(plan)
    if plan['schema'] not in (*SUCCESSOR_SCHEMAS, NO_ACCEL_DEVELOPMENT_SCHEMA, CANDIDATE_REGRET_SCHEMA, STOPMIX_SCHEMA):
        return None
    manifest = dataset / 'dataset.json'
    if manifest.is_symlink() or not manifest.is_file() or digest(manifest) != EXPANDED_MANIFEST_SHA256:
        raise RuntimeError('expanded dataset manifest differs from the frozen SHA-256')
    return EXPANDED_MANIFEST_SHA256


def wait_for_prerequisite(plan, parent):
    # HH_260906 - Wait without a GPU or cooperative lease so the predecessor can finish normally.
    validate_plan(plan)
    if plan['schema'] not in SUCCESSOR_SCHEMAS:
        return None
    selector = plan['schema'] in (SELECTOR_WEIGHT_SCHEMA, CANDIDATE_RANK_SCHEMA)
    predecessor_id = DATA_EXPANSION_CAMPAIGN if selector else PREREQUISITE_CAMPAIGN
    predecessor_source = HISTORICAL_SOURCE_COMMIT if selector else plan['source_commit']
    predecessor_arms = ('C_expanded_data',) if selector else ('A_baseline', 'B_lower_lr')
    status_path = parent / predecessor_id / 'status.json'
    deadline = time.monotonic() + 3600
    expected = [(f'seed_{seed}/{arm}', stage)
        for seed in (20260903, 20260904, 20260905)
        for arm in predecessor_arms for stage in ('train', 'evaluate', 'audit')]
    while True:
        status = None
        if status_path.exists():
            if status_path.is_symlink() or not status_path.is_file():
                raise RuntimeError('prerequisite status is not a regular file')
            payload = status_path.read_bytes()
            state = json.loads(payload)
            status = state.get('status')
            stages = state.get('stages', [])
            if not isinstance(stages, list) or any(not isinstance(stage, dict) for stage in stages):
                raise RuntimeError('prerequisite stages are malformed')
            if status == 'STOPPED_FAILURE_NO_PROMOTION' or any(stage.get('status') == 'FAILED' for stage in stages):
                raise RuntimeError('prerequisite campaign failed; successor will not start')
            if status == 'TRAIN_EVAL_AUDIT_COMPLETE_NOT_PROMOTED':
                if (state.get('plan', {}).get('campaign_id') != predecessor_id or
                        state.get('source_commit') != predecessor_source or
                        state.get('vehicle_control_approved') is not False or
                        [(stage.get('run'), stage.get('stage')) for stage in stages] != expected or
                        any(stage.get('status') != 'COMPLETE' or stage.get('returncode') != 0 for stage in stages)):
                    raise RuntimeError(f'prerequisite completion does not prove all {len(expected)} reviewed stages')
                if selector:
                    # HH_260906 - The new selector CLI commit must not rewrite the completed predecessor's older source identity.
                    predecessor_plan = state.get('plan', {})
                    if predecessor_plan.get('schema') != DATA_EXPANSION_SCHEMA:
                        raise RuntimeError('selector prerequisite is not the reviewed data-expansion campaign')
                    validate_plan(predecessor_plan)
                proof = {'status': status, 'path': str(status_path),
                    'sha256': hashlib.sha256(payload).hexdigest(), 'completed_stages': len(expected)}
                if selector:
                    proof.update(campaign_id=predecessor_id, source_commit=predecessor_source)
                return proof
            if status not in ('PREFLIGHT', 'RUNNING'):
                raise RuntimeError('prerequisite campaign has an unexpected status')
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError('prerequisite campaign did not complete within 3600 seconds')
        print(json.dumps({'status': 'WAITING_PREREQUISITE', 'prerequisite': str(status_path),
            'observed_status': status, 'remaining_seconds': round(remaining, 1),
            'gpu_acquired': False}), flush=True)
        time.sleep(min(5.0, remaining))


def acquire_campaign_lease(lease, plan):
    # HH_260906 - Cover the brief completed-status/lease-release race without overlapping GPU campaigns.
    validate_plan(plan)
    deadline = time.monotonic() + 3600
    while True:
        try:
            fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
            return
        except BlockingIOError:
            if plan['schema'] not in SUCCESSOR_SCHEMAS:
                raise
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError('GPU0 campaign lease remained occupied for 3600 seconds')
            print(json.dumps({'status': 'WAITING_GPU0_CAMPAIGN_LEASE',
                'gpu_acquired': False, 'remaining_seconds': round(remaining, 1)}), flush=True)
            time.sleep(min(5.0, remaining))


def run_inventory(command, repo):
    return subprocess.check_output(command, cwd=repo, text=True, timeout=30).strip()


def assert_gpu_idle(repo):
    # HH_260906 - Pin physical GPU0 by UUID and never stop another user's process.
    rows = run_inventory(['nvidia-smi', '-i', '0', '--query-gpu=index,uuid', '--format=csv,noheader'], repo)
    if f'0, {GPU_UUID}' not in rows.splitlines():
        raise RuntimeError('physical GPU0 UUID changed; manual review required')
    processes = run_inventory(
        ['nvidia-smi', '-i', '0', '--query-compute-apps=gpu_uuid,pid', '--format=csv,noheader'], repo)
    if any(row.split(',')[0].strip() == GPU_UUID for row in processes.splitlines()):
        raise RuntimeError('GPU0 is occupied; leave other jobs untouched')


def stage_environment():
    # HH_260906 - Prevent inherited Python search paths from bypassing the personal venv and pinned repository.
    env = dict(os.environ)
    for name in ('PYTHONPATH', 'PYTHONHOME', 'PYTHONSTARTUP', 'PYTHONUSERBASE'):
        env.pop(name, None)
    env.update(CUDA_VISIBLE_DEVICES=GPU_UUID,
        CUBLAS_WORKSPACE_CONFIG=':4096:8', PYTHONUNBUFFERED='1',
        PYTHONNOUSERSITE='1', PIP_REQUIRE_VIRTUALENV='true',
        PORTABLE_E2E_ROOT=str(WORKSPACE), PIP_CACHE_DIR=str(WORKSPACE / 'cache/pip'),
        OMP_NUM_THREADS='16', MKL_NUM_THREADS='16')
    return env


def verify_source(plan, repo, model_config, expected_model_sha256):
    # HH_260906 - Repeat provenance checks around every stage so later arms cannot silently use edited code.
    source = run_inventory(['git', 'rev-parse', 'HEAD'], repo)
    if source != plan['source_commit']:
        raise RuntimeError('remote source does not match the frozen plan')
    if run_inventory(['git', 'status', '--porcelain', '--untracked-files=all'], repo):
        raise RuntimeError('remote source has tracked or untracked changes')
    if digest(model_config) != expected_model_sha256:
        raise RuntimeError('model config changed during campaign')


def campaign_model_configs(plan, repo):
    # HH_260906 - Bind both arm-specific source files before any training; legacy single-config plans keep their original fields.
    paired = plan['schema'] in (NO_ACCEL_DEVELOPMENT_SCHEMA, STOPMIX_SCHEMA)
    names = plan['model_configs'] if paired else {'shared': plan['model_config']}
    models = {}
    for arm, name in names.items():
        path = (repo / name).resolve(strict=True)
        if not path.is_relative_to(repo):
            raise RuntimeError('model config escapes repository')
        file_sha = digest(path)
        if plan['schema'] == CANDIDATE_REGRET_SCHEMA and (
                (repo / name).is_symlink() or file_sha != plan['model_config_sha256']):
            raise RuntimeError('candidate-regret model differs from the reviewed physical-v1 config')
        record = {'path': name, 'sha256': file_sha}
        if paired:
            expected = STOPMIX_MODEL_SHA256 if plan['schema'] == STOPMIX_SCHEMA else NO_ACCEL_MODEL_SHA256
            if (repo / name).is_symlink() or file_sha != expected[arm]:
                raise RuntimeError('paired model config differs from the reviewed source SHA')
            config = json.loads(path.read_text())
            record.update(model_config=config, canonical_sha256=hashlib.sha256(json.dumps(
                config, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest())
        models[arm] = record
    return models


def campaign_source_files(plan, repo):
    # HH_260906 - Archive exact new-model pipeline bytes; clean pinned Git is still independently checked around every stage.
    if plan['schema'] != STOPMIX_SCHEMA:
        return None
    files = {}
    for name in STOPMIX_SOURCE_PATHS:
        path = repo / name
        if not path.is_file() or any(p.is_symlink() for p in (path, *path.parents)):
            raise RuntimeError('STOPMIX pipeline source must be regular without symlinks')
        files[name] = {'sha256': digest(path), 'archive_path': 'provenance/' +
            ('active_runner.py' if name == 'scripts/e2e/run_portable_training_campaign.py' else name)}
    if files['portable_e2e/stop_primitive_research.py']['sha256'] != STOPMIX_PRIMITIVE_SHA256:
        raise RuntimeError('STOPMIX primitive differs from reviewed source')
    return files


def verify_pipeline_sources(plan, repo, files, archive_root=None):
    if plan['schema'] != STOPMIX_SCHEMA:
        return
    if campaign_source_files(plan, repo) != files:
        raise RuntimeError('STOPMIX pipeline source changed during campaign')
    if archive_root is not None:
        for record in files.values():
            path = archive_root / record['archive_path']
            if not path.is_file() or any(p.is_symlink() for p in (path, *path.parents)) or digest(path) != record['sha256']:
                raise RuntimeError('STOPMIX archived source changed during campaign')


def archive_pipeline_sources(plan, repo, files, root):
    if plan['schema'] != STOPMIX_SCHEMA:
        return
    verify_pipeline_sources(plan, repo, files)
    for name, record in files.items():
        path = root / record['archive_path']
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('xb') as stream:
            stream.write((repo / name).read_bytes())
    verify_pipeline_sources(plan, repo, files, root)


def verify_campaign_models(plan, repo, models):
    # HH_260906 - Recheck every paired config around every stage, including the arm not currently training.
    for record in models.values():
        verify_source(plan, repo, repo / record['path'], record['sha256'])


@contextmanager
def termination_guard():
    # HH_260906 - Translate normal termination into the same cleanup and status path as keyboard interruption.
    def interrupt(signum, _frame):
        raise InterruptedError(f'campaign interrupted by signal {signum}')

    previous = signal.signal(signal.SIGTERM, interrupt)
    try:
        yield
    finally:
        signal.signal(signal.SIGTERM, previous)


def group_has_live_members(pgid):
    # HH_260906 - Ignore already dead zombies while checking the owned Linux process group.
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        if int(fields[2]) == pgid and fields[0] not in ('Z', 'X'):
            return True
    return False


def stop_owned_group(child, grace_seconds=5.0):
    # HH_260906 - Terminate only this stage's session and escalate surviving members after a bounded grace period.
    if child.pid == os.getpgrp():
        raise RuntimeError('refusing to signal the campaign process group')
    for signum in (signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(child.pid, signum)
        except ProcessLookupError:
            pass
        deadline = time.monotonic() + grace_seconds
        while time.monotonic() < deadline:
            child.poll()
            if not group_has_live_members(child.pid):
                child.wait(timeout=grace_seconds)
                return
            time.sleep(0.05)
    raise RuntimeError(f'owned stage process group {child.pid} did not stop')


def run_owned_stage(command, repo, env, log, timeout=1800, cleanup_grace_seconds=5.0):
    child = subprocess.Popen(command, cwd=repo, env=env, stdout=log,
        stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
    try:
        return child.wait(timeout=timeout)
    finally:
        # HH_260906 - Repeated interactive signals must not interrupt cleanup and orphan the GPU stage.
        previous = {sig: signal.signal(sig, signal.SIG_IGN)
            for sig in (signal.SIGINT, signal.SIGTERM)}
        try:
            stop_owned_group(child, cleanup_grace_seconds)
        finally:
            for sig, handler in previous.items():
                signal.signal(sig, handler)


def verify_stage_report(stage, item, checkpoint, state, plan):
    # HH_260906 - Derive immutable sample counts from the reviewed schema and bind all arms to one corpus.
    contract = validate_plan(plan)
    paths = {'train': 'training/run.json', 'evaluate': 'evaluation/metrics.json', 'audit': 'gate_v8.json'}
    statuses = {'train': 'TRAINING_TARGET_REACHED', 'evaluate': 'OPEN_LOOP_EVALUATION_COMPLETE',
        'audit': 'RUNTIME_GEOMETRY_AUDIT_COMPLETE'}
    report_path = item / paths[stage]
    report = json.loads(report_path.read_text())
    if report.get('status') != statuses[stage]:
        raise RuntimeError('stage report does not declare completed work')
    if plan['schema'] == CANDIDATE_REGRET_SCHEMA:
        # HH_260906 - Verify actual per-arm source, parameters and training exposure, not just a successful process exit.
        config = state.get('model_config')
        if not isinstance(config, dict) or config.get('model_id') != 'portable_e2e.perspective_trajectory.physical.v1':
            raise RuntimeError('candidate-regret stage has no pinned physical model config')
        if report.get('model_parameter_count') != 954590:
            raise RuntimeError('candidate-regret stage parameter count differs')
        if stage == 'train':
            if report.get('model_config') != config:
                raise RuntimeError('candidate-regret training model config differs')
            if report.get('state', {}).get('domain_samples_seen') != {'carla': 6155}:
                raise RuntimeError('candidate-regret training exposures differ')
            expected_seed = int(item.parent.name.removeprefix('seed_'))
            if (report.get('train_config', {}).get('seed') != expected_seed
                    or report.get('train_config', {}).get('learning_rate') != 0.0001
                    or report.get('train_config', {}).get('batch_size') != 4):
                raise RuntimeError('candidate-regret training seed, learning rate or batch differs')
        elif report.get('model_config_sha256') != state.get('model_config_canonical_sha256'):
            raise RuntimeError('candidate-regret evaluation model config differs')
    if plan['schema'] in (NO_ACCEL_DEVELOPMENT_SCHEMA, STOPMIX_SCHEMA):
        arm = item.name
        stopmix = plan['schema'] == STOPMIX_SCHEMA
        arm_models = STOPMIX_MODELS if stopmix else NO_ACCEL_MODELS
        arm_hashes = STOPMIX_MODEL_SHA256 if stopmix else NO_ACCEL_MODEL_SHA256
        if arm not in arm_models:
            raise RuntimeError('stage report belongs to an unreviewed model arm')
        model = state.get('model_configs', {}).get(arm, {})
        expected_id = ('portable_e2e.perspective_trajectory.physical.v1' if arm in ('A_physical_input', 'A_physical_drive')
            else 'portable_e2e.perspective_trajectory.physical_stopmix.v1' if stopmix
            else 'portable_e2e.perspective_trajectory.physical_no_accel.v1')
        if (model.get('sha256') != arm_hashes[arm]
                or model.get('path') != arm_models[arm]
                or not isinstance(model.get('model_config'), dict)
                or model['model_config'].get('model_id') != expected_id
                or not re.fullmatch('[0-9a-f]{64}', model.get('canonical_sha256', ''))):
            raise RuntimeError('stage lacks pinned per-arm model provenance')
        if stage == 'train':
            if report.get('model_config') != model.get('model_config'):
                raise RuntimeError('training report model config or model ID differs from its arm')
        elif report.get('model_config_sha256') != model.get('canonical_sha256'):
            raise RuntimeError('evaluation model config fingerprint differs from its arm')
        if stopmix:
            candidates, parameters = (6, 954590) if arm == 'A_physical_drive' else (12, 1056362)
            if model['model_config'].get('candidate_count') != candidates or report.get('model_parameter_count') != parameters:
                raise RuntimeError('STOPMIX candidate or parameter count differs from its arm')
            if stage == 'train':
                config = report.get('train_config', {})
                if (report.get('state', {}).get('domain_samples_seen') != {'carla': 6155}
                        or config.get('seed') != int(item.parent.name.removeprefix('seed_'))
                        or config.get('learning_rate') != 0.0001 or config.get('batch_size') != 4):
                    raise RuntimeError('STOPMIX training exposures, seed, learning rate or batch differ')
            elif any(name in report for name in ('auxiliary_loss_metrics', 'auxiliary_loss_metric_counts', 'loss_config')):
                raise RuntimeError('STOPMIX original-loss evaluation unexpectedly reports an auxiliary loss')
            if stage == 'audit' and (report.get('gate', {}).get('thresholds', {}).get('candidate_count') != candidates
                    or report.get('gate', {}).get('threshold_overrides') is not False):
                raise RuntimeError('STOPMIX audit candidate count or gate override differs')
    corpus = report.get('corpus_fingerprint_sha256', '')
    fingerprint = report.get('dataset_fingerprint_sha256', '')
    if not isinstance(corpus, str) or not re.fullmatch('[0-9a-f]{64}', corpus):
        raise RuntimeError('stage report has no valid corpus fingerprint')
    if not isinstance(fingerprint, str) or not re.fullmatch('[0-9a-f]{64}', fingerprint):
        raise RuntimeError('stage report has no valid split fingerprint')
    expected = state.get('corpus_fingerprint_sha256')
    if expected is not None and corpus != expected:
        raise RuntimeError('corpus fingerprint changed between campaign stages')
    split_key = 'train_fingerprint_sha256' if stage == 'train' else 'val_fingerprint_sha256'
    if split_key in state and fingerprint != state[split_key]:
        raise RuntimeError('split fingerprint changed between campaign stages')
    if stage == 'train':
        if report.get('training_split') != 'train' or report.get('dataset_size') != contract['train_samples']:
            raise RuntimeError('training report does not cover the frozen training split')
        if report.get('state', {}).get('global_step') != plan['steps']:
            raise RuntimeError('training report did not reach the frozen step budget')
        # HH_260906 - A selector-weight experiment changes only the reviewed score loss coefficient.
        expected_loss = {'xy_weight': 1.0, 'speed_weight': 0.2, 'yaw_weight': 0.1,
            'kinematic_speed_weight': 0.05, 'final_displacement_weight': 0.5,
            'candidate_score_weight': 0.5 if plan['schema'] == SELECTOR_WEIGHT_SCHEMA else 0.1}
        if plan['schema'] == CANDIDATE_REGRET_SCHEMA:
            if item.name not in CANDIDATE_REGRET_WEIGHTS:
                raise RuntimeError('unreviewed candidate-regret training arm')
            weight = CANDIDATE_REGRET_WEIGHTS[item.name]
            if weight:
                expected_loss['candidate_regret_weight'] = weight
        if report.get('loss_config') != expected_loss:
            raise RuntimeError('training loss config differs from the frozen campaign weights')
    else:
        if report.get('evaluation_split') != plan['split']:
            raise RuntimeError('evaluation report uses an unreviewed split')
        if report.get('training_dataset_fingerprint_sha256') != state.get('train_fingerprint_sha256'):
            raise RuntimeError('evaluation checkpoint uses another training split')
        if report.get('checkpoint_sha256') != digest(checkpoint):
            raise RuntimeError('evaluation checkpoint hash does not match the retained checkpoint')
        count = report.get('sample_count') if stage == 'evaluate' else report.get('geometry', {}).get('sample_count')
        if count != contract['val_samples'] or report.get('vehicle_control_approved') is not False:
            raise RuntimeError('evaluation report does not cover research-only val337')
        if stage == 'audit' and report.get('gate', {}).get('source') != 'portable_e2e.runtime_geometry_gate.v8':
            raise RuntimeError('runtime audit uses an unexpected geometry gate')
    state['corpus_fingerprint_sha256'] = corpus
    state[split_key] = fingerprint
    return {'path': str(report_path.relative_to(item)), 'sha256': digest(report_path),
        'corpus_fingerprint_sha256': corpus, 'dataset_fingerprint_sha256': fingerprint}


def commands(plan, root, repo):
    validate_plan(plan)
    python = str(WORKSPACE / 'venvs/py312/bin/python')
    dataset = str(WORKSPACE / plan['dataset'])
    for seed in plan['seeds']:
        for arm, learning_rate in plan['arms'].items():
            item = root / f'seed_{seed}' / arm
            checkpoint = item / 'training/checkpoints/latest.pt'
            shared = ['--device', 'cuda:0', '--batch-size', str(plan['batch_size'])]
            training_command = [python, '-m', 'portable_e2e.train', dataset,
                '--run-dir', str(item / 'training'), '--model-config',
                str(repo / (plan['model_configs'][arm] if plan['schema'] in (NO_ACCEL_DEVELOPMENT_SCHEMA, STOPMIX_SCHEMA)
                    else plan['model_config'])), '--split', 'train', *shared,
                '--seed', str(seed), '--learning-rate', str(learning_rate),
                '--weight-decay', '0.0001', '--max-steps', str(plan['steps']),
                '--checkpoint-interval', '154', '--num-workers', '0',
                '--maximum-gradient-norm', '5', '--sampling-policy',
                'uniform_without_replacement']
            if plan['schema'] == SELECTOR_WEIGHT_SCHEMA:
                training_command += ['--candidate-score-weight', '0.5']
            elif plan['schema'] == CANDIDATE_RANK_SCHEMA:
                # HH_260906 - Record the unchanged score coefficient explicitly for architecture-only comparison.
                training_command += ['--candidate-score-weight', '0.1']
            elif plan['schema'] == CANDIDATE_REGRET_SCHEMA:
                # HH_260906 - Explicit zero keeps the original objective; nonzero uses detached composite regret only.
                training_command += ['--candidate-score-weight', '0.1', '--candidate-regret-weight',
                    str(CANDIDATE_REGRET_WEIGHTS[arm])]
            yield item, checkpoint, training_command, 'train'
            yield item, checkpoint, [python, '-m', 'portable_e2e.evaluate', dataset,
                '--checkpoint', str(checkpoint), '--output-dir', str(item / 'evaluation'),
                '--split', plan['split'], *shared, '--num-workers', '0',
                '--render-count', '12'], 'evaluate'
            yield item, checkpoint, [python, '-m', 'portable_e2e.audit_runtime', dataset,
                '--checkpoint', str(checkpoint), '--output-json', str(item / 'gate_v8.json'),
                '--split', plan['split'], *shared], 'audit'


def verify_finish_budget(plan, remaining_stages, *, observed_at=None):
    # HH_260906 - Reserve every remaining stage timeout and owned cleanup before the explicit user deadline.
    if plan['schema'] not in (CANDIDATE_REGRET_SCHEMA, STOPMIX_SCHEMA):
        return
    validate_plan(plan)
    deadline = datetime.fromisoformat(plan['finish_before_utc'].replace('Z', '+00:00'))
    observed = datetime.now(timezone.utc) if observed_at is None else observed_at
    if observed.tzinfo is None or observed.utcoffset() is None:
        raise ValueError('finish-budget observation must be timezone aware')
    if any(stage not in CANDIDATE_REGRET_STAGE_TIMEOUTS for stage in remaining_stages):
        raise ValueError('unreviewed remaining stage in finish budget')
    needed = sum(CANDIDATE_REGRET_STAGE_TIMEOUTS[stage] for stage in remaining_stages) + plan['finish_reserve_seconds']
    if plan['schema'] == STOPMIX_SCHEMA:
        needed += plan['behavior_analysis_count'] * plan['behavior_analysis_timeout_seconds']
    if (deadline - observed).total_seconds() < needed:
        raise TimeoutError(f'insufficient time for remaining campaign and cleanup: {needed}s reserved')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('plan', type=Path)
    parser.add_argument('--validate-only', action='store_true')
    args = parser.parse_args(argv)
    plan_bytes = args.plan.read_bytes()
    plan = json.loads(plan_bytes)
    contract = validate_plan(plan)
    if args.validate_only:
        print('PLAN_VALID; no training started')
        return 0
    remaining_stages = [stage for _seed in plan['seeds'] for _arm in plan['arms']
        for stage in ('train', 'evaluate', 'audit')]
    verify_finish_budget(plan, remaining_stages)
    repo = WORKSPACE / 'autoware_e2e'
    expected_python = WORKSPACE / 'venvs/py312/bin/python'
    if Path(sys.prefix).absolute() != expected_python.parent.parent:
        raise RuntimeError('run only with the personal py312 venv interpreter')
    dataset = (WORKSPACE / plan['dataset']).resolve(strict=True)
    if not dataset.is_relative_to(WORKSPACE.parent / 'dataset'):
        raise RuntimeError('dataset must be inside the personal dataset folder')
    manifest_sha256 = verify_dataset_manifest(plan, dataset)
    models = campaign_model_configs(plan, repo)
    verify_campaign_models(plan, repo, models)
    source_files = campaign_source_files(plan, repo)
    parent = WORKSPACE / 'runs/campaigns'
    parent.mkdir(parents=True, exist_ok=True)
    if parent.resolve() != parent:
        raise RuntimeError('campaign parent must not traverse symlinks')
    root = parent / plan['campaign_id']
    with termination_guard():
        prerequisite = wait_for_prerequisite(plan, parent)
    env = stage_environment()
    state = {'status': 'PREFLIGHT', 'created_at_utc': now(), 'plan': plan,
        'plan_sha256': hashlib.sha256(plan_bytes).hexdigest(), 'runner_sha256': digest(Path(__file__)),
        'source_commit': plan['source_commit'],
        'dataset_manifest_sha256': manifest_sha256, 'reviewed_contract': contract,
        'prerequisite': prerequisite,
        'vehicle_control_approved': False, 'stages': []}
    if plan['schema'] in (NO_ACCEL_DEVELOPMENT_SCHEMA, STOPMIX_SCHEMA):
        state['model_configs'] = models
        if plan['schema'] == STOPMIX_SCHEMA:
            state['source_files'] = source_files
            state['behavior_analysis'] = {'status': 'NOT_RUN_SEPARATE_WORKFLOW', 'expected_count': 6,
                'per_analysis_timeout_seconds': 180, 'included_in_completed_stage_count': False}
    else:
        state['model_config_sha256'] = models['shared']['sha256']
        if plan['schema'] == CANDIDATE_REGRET_SCHEMA:
            state['model_config'] = json.loads((repo / plan['model_config']).read_text())
            state['model_config_canonical_sha256'] = hashlib.sha256(json.dumps(
                state['model_config'], sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
    # HH_260906 - A cooperative lock prevents duplicate campaigns without touching other jobs.
    with termination_guard(), (parent / '.gpu0_training.lock').open('a') as lease:
        acquire_campaign_lease(lease, plan)
        verify_campaign_models(plan, repo, models)
        verify_pipeline_sources(plan, repo, source_files)
        verify_dataset_manifest(plan, dataset)
        assert_gpu_idle(repo)
        verify_finish_budget(plan, remaining_stages)
        root.mkdir(exist_ok=False)
        (root / 'plan.json').write_bytes(plan_bytes)

        def save():
            temporary = root / 'status.json.new'
            temporary.write_text(json.dumps(state, indent=2) + '\n')
            temporary.replace(root / 'status.json')

        save()
        try:
            archive_pipeline_sources(plan, repo, source_files, root)
            for item, checkpoint, command, stage in commands(plan, root, repo):
                verify_campaign_models(plan, repo, models)
                verify_pipeline_sources(plan, repo, source_files, root)
                verify_dataset_manifest(plan, dataset)
                assert_gpu_idle(repo)
                verify_finish_budget(plan, remaining_stages)
                item.mkdir(parents=True, exist_ok=True)
                if stage == 'audit':
                    command += ['--checkpoint-sha256', digest(checkpoint)]
                record = {'run': str(item.relative_to(root)), 'stage': stage,
                    'started_at_utc': now(), 'command': command, 'status': 'RUNNING'}
                state['stages'].append(record)
                state['status'] = 'RUNNING'
                save()
                print(json.dumps(record), flush=True)
                with (item / f'{stage}.log').open('x') as log:
                    if plan['schema'] in (CANDIDATE_REGRET_SCHEMA, STOPMIX_SCHEMA):
                        returncode = run_owned_stage(command, repo, env, log,
                            timeout=CANDIDATE_REGRET_STAGE_TIMEOUTS[stage])
                    else:
                        returncode = run_owned_stage(command, repo, env, log)
                record['returncode'] = returncode
                verify_campaign_models(plan, repo, models)
                verify_pipeline_sources(plan, repo, source_files, root)
                verify_dataset_manifest(plan, dataset)
                if returncode:
                    raise RuntimeError(f'{record["run"]} {stage} failed; inspect preserved log')
                record['report'] = verify_stage_report(stage, item, checkpoint, state, plan)
                record.update(finished_at_utc=now(), status='COMPLETE')
                remaining_stages.pop(0)
                save()
            if len(state['stages']) != contract['stage_count']:
                raise RuntimeError('completed stage count differs from the reviewed campaign')
            state['status'] = 'TRAIN_EVAL_AUDIT_COMPLETE_NOT_PROMOTED'
        except BaseException as error:
            if state['stages'] and state['stages'][-1]['status'] == 'RUNNING':
                state['stages'][-1].update(status='FAILED', finished_at_utc=now())
            state.update(status='STOPPED_FAILURE_NO_PROMOTION', error=f'{type(error).__name__}: {error}')
            raise
        finally:
            state['updated_at_utc'] = now()
            save()
            print(json.dumps({'status': state['status'], 'output': str(root)}), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
