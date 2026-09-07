"""HH_260906 - Verify bounded campaign scope without SSH or GPU execution."""

import copy
import importlib.util
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('training_campaign', ROOT / 'scripts/e2e/run_portable_training_campaign.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


@pytest.fixture
def plan():
    return json.loads((ROOT / 'config/portable_e2e_lr_ab_20260907.json').read_text())


@pytest.fixture
def expansion_plan():
    return json.loads((ROOT / 'config/portable_e2e_data_expansion_20260907.json').read_text())


@pytest.fixture
def selector_plan(expansion_plan):
    # HH_260906 - Use an unfrozen test-only future commit until the root publishes the actual selector CLI commit.
    plan = copy.deepcopy(expansion_plan)
    plan.update(schema=MODULE.SELECTOR_WEIGHT_SCHEMA,
        campaign_id='hh260907-physical-v1-selector-weight-3seeds-v1', source_commit='b' * 40,
        arms={'D_selector_weight': 0.0001}, candidate_score_weight=0.5,
        prerequisite_campaign_id=MODULE.DATA_EXPANSION_CAMPAIGN)
    return plan


def test_expansion_scope_uses_same_step_budget_and_three_seeds(expansion_plan, tmp_path):
    contract = MODULE.validate_plan(expansion_plan)
    assert contract == {'train_samples': 1147, 'val_samples': 337, 'run_count': 3, 'stage_count': 9}
    stages = list(MODULE.commands(expansion_plan, tmp_path, ROOT))
    assert len(stages) == 9
    assert len({str(item) for item, _, _, _ in stages}) == 3
    for _, _, command, stage in stages:
        assert '--candidate-score-weight' not in command
        assert command[command.index('--split') + 1] == ('train' if stage == 'train' else 'val')
        if stage == 'train':
            assert command[command.index('--max-steps') + 1] == '1540'
            assert float(command[command.index('--learning-rate') + 1]) == 0.0001
    assert 'not 10 epochs' in expansion_plan['decision']['budget']


@pytest.mark.parametrize('field,value', [
    ('dataset', 'datasets/prepared/other'), ('dataset_manifest_sha256', 'f' * 64),
    ('expected_train_samples', 613), ('expected_val_samples', 336),
    ('prerequisite_campaign_id', 'other'), ('prerequisite_timeout_seconds', 7200),
    ('source_commit', 'f' * 40), ('arms', {'C_expanded_data': 0.00003}),
    ('model_config', 'portable_e2e/config/perspective_trajectory_v0.model.json'),
])
def test_expansion_cannot_expand_reviewed_scope(expansion_plan, field, value):
    expansion_plan[field] = value
    with pytest.raises(ValueError):
        MODULE.validate_plan(expansion_plan)


def test_expansion_manifest_is_required_and_rechecked(expansion_plan, tmp_path, monkeypatch):
    manifest = tmp_path / 'dataset.json'
    with pytest.raises(RuntimeError, match='manifest'):
        MODULE.verify_dataset_manifest(expansion_plan, tmp_path)
    manifest.write_text('{}')
    monkeypatch.setattr(MODULE, 'digest', lambda _: MODULE.EXPANDED_MANIFEST_SHA256)
    assert MODULE.verify_dataset_manifest(expansion_plan, tmp_path) == MODULE.EXPANDED_MANIFEST_SHA256
    monkeypatch.setattr(MODULE, 'digest', lambda _: 'f' * 64)
    with pytest.raises(RuntimeError, match='manifest'):
        MODULE.verify_dataset_manifest(expansion_plan, tmp_path)


def test_selector_scope_is_three_same_budget_val_only_runs(selector_plan, tmp_path):
    assert MODULE.validate_plan(selector_plan) == {'train_samples': 1147, 'val_samples': 337,
        'run_count': 3, 'stage_count': 9}
    stages = list(MODULE.commands(selector_plan, tmp_path, ROOT))
    assert len(stages) == 9 and len({str(item) for item, _, _, _ in stages}) == 3
    for item, _, command, stage in stages:
        assert item.name == 'D_selector_weight'
        assert command[command.index('--split') + 1] == ('train' if stage == 'train' else 'val')
        assert command[command.index('--device') + 1] == 'cuda:0'
        assert '--resume' not in command
        if stage == 'train':
            assert command.count('--candidate-score-weight') == 1
            assert command[command.index('--candidate-score-weight') + 1] == '0.5'
            assert command[command.index('--max-steps') + 1] == '1540'
            assert command[command.index('--learning-rate') + 1] == '0.0001'
        else:
            assert '--candidate-score-weight' not in command


@pytest.mark.parametrize('field,value', [
    ('candidate_score_weight', 0.1), ('candidate_score_weight', 1.0),
    ('dataset', 'datasets/prepared/other'), ('dataset_manifest_sha256', 'f' * 64),
    ('expected_train_samples', 613), ('expected_val_samples', 336),
    ('arms', {'D_selector_weight': 0.00003}), ('source_commit', 'main'),
    ('prerequisite_campaign_id', MODULE.PREREQUISITE_CAMPAIGN),
    ('prerequisite_timeout_seconds', 7200), ('steps', 3080),
    ('model_config', 'portable_e2e/config/perspective_trajectory_v0.model.json'),
])
def test_selector_rejects_unreviewed_experiment_changes(selector_plan, field, value):
    selector_plan[field] = value
    with pytest.raises(ValueError):
        MODULE.validate_plan(selector_plan)


def test_selector_requires_same_v3_manifest(selector_plan, tmp_path, monkeypatch):
    (tmp_path / 'dataset.json').write_text('{}')
    monkeypatch.setattr(MODULE, 'digest', lambda _: MODULE.EXPANDED_MANIFEST_SHA256)
    assert MODULE.verify_dataset_manifest(selector_plan, tmp_path) == MODULE.EXPANDED_MANIFEST_SHA256
    monkeypatch.setattr(MODULE, 'digest', lambda _: 'f' * 64)
    with pytest.raises(RuntimeError, match='manifest'):
        MODULE.verify_dataset_manifest(selector_plan, tmp_path)


def _prerequisite_status(plan):
    # HH_260906 - The successor may proceed only after all paired predecessor stages complete.
    return {'status': 'TRAIN_EVAL_AUDIT_COMPLETE_NOT_PROMOTED', 'plan': plan,
        'source_commit': plan['source_commit'], 'vehicle_control_approved': False,
        'stages': [{'run': f'seed_{seed}/{arm}', 'stage': stage, 'status': 'COMPLETE', 'returncode': 0}
            for seed in plan['seeds'] for arm in plan['arms'] for stage in ('train', 'evaluate', 'audit')]}


def test_prerequisite_waits_then_completes_without_gpu_or_lease(plan, expansion_plan, tmp_path, monkeypatch, capsys):
    status = tmp_path / MODULE.PREREQUISITE_CAMPAIGN / 'status.json'
    status.parent.mkdir()
    status.write_text(json.dumps({'status': 'RUNNING', 'stages': []}))
    sleeps = []
    def finish_after_sleep(seconds):
        sleeps.append(seconds)
        status.write_text(json.dumps(_prerequisite_status(plan)))
    monkeypatch.setattr(MODULE.time, 'sleep', finish_after_sleep)
    monkeypatch.setattr(MODULE, 'assert_gpu_idle', lambda *_: pytest.fail('waiting must not query/acquire GPU'))
    monkeypatch.setattr(MODULE.fcntl, 'flock', lambda *_: pytest.fail('waiting must not hold cooperative lease'))
    proof = MODULE.wait_for_prerequisite(expansion_plan, tmp_path)
    assert proof['completed_stages'] == 18
    assert proof['sha256'] == MODULE.digest(status)
    assert sleeps == [5.0]
    assert 'WAITING_PREREQUISITE' in capsys.readouterr().out


@pytest.mark.parametrize('mutation', ['failed', 'missing_stage', 'nonzero', 'reordered'])
def test_prerequisite_failure_or_incomplete_completion_blocks(plan, expansion_plan, tmp_path, mutation):
    state = _prerequisite_status(plan)
    if mutation == 'failed':
        state['status'] = 'STOPPED_FAILURE_NO_PROMOTION'
    elif mutation == 'missing_stage':
        state['stages'].pop()
    elif mutation == 'nonzero':
        state['stages'][-1]['returncode'] = 1
    else:
        state['stages'].reverse()
    status = tmp_path / MODULE.PREREQUISITE_CAMPAIGN / 'status.json'
    status.parent.mkdir()
    status.write_text(json.dumps(state))
    with pytest.raises(RuntimeError, match='prerequisite'):
        MODULE.wait_for_prerequisite(expansion_plan, tmp_path)


def test_prerequisite_timeout_does_not_wait_indefinitely(expansion_plan, tmp_path, monkeypatch):
    times = iter([0.0, 3601.0])
    monkeypatch.setattr(MODULE.time, 'monotonic', lambda: next(times))
    monkeypatch.setattr(MODULE.time, 'sleep', lambda *_: pytest.fail('expired wait must not sleep'))
    with pytest.raises(TimeoutError, match='3600 seconds'):
        MODULE.wait_for_prerequisite(expansion_plan, tmp_path)


def test_selector_accepts_old_source_completed_nine_stage_predecessor(selector_plan, expansion_plan, tmp_path):
    state = _prerequisite_status(expansion_plan)
    assert state['source_commit'] != selector_plan['source_commit']
    path = tmp_path / MODULE.DATA_EXPANSION_CAMPAIGN / 'status.json'
    path.parent.mkdir()
    path.write_text(json.dumps(state))
    proof = MODULE.wait_for_prerequisite(selector_plan, tmp_path)
    assert proof['completed_stages'] == 9
    assert proof['source_commit'] == MODULE.HISTORICAL_SOURCE_COMMIT
    assert proof['campaign_id'] == MODULE.DATA_EXPANSION_CAMPAIGN
    assert proof['sha256'] == MODULE.digest(path)


@pytest.mark.parametrize('mutation', ['missing_stage', 'wrong_source', 'wrong_plan_source', 'failed', 'wrong_schema'])
def test_selector_rejects_unproven_predecessor(selector_plan, expansion_plan, tmp_path, mutation):
    state = _prerequisite_status(expansion_plan)
    if mutation == 'missing_stage':
        state['stages'].pop()
    elif mutation == 'wrong_source':
        state['source_commit'] = selector_plan['source_commit']
    elif mutation == 'wrong_plan_source':
        state['plan']['source_commit'] = selector_plan['source_commit']
    elif mutation == 'failed':
        state['stages'][-1]['status'] = 'FAILED'
    else:
        state['plan']['schema'] = 'portable_e2e.lr_ab_campaign.v1'
    path = tmp_path / MODULE.DATA_EXPANSION_CAMPAIGN / 'status.json'
    path.parent.mkdir()
    path.write_text(json.dumps(state))
    with pytest.raises((ValueError, RuntimeError)):
        MODULE.wait_for_prerequisite(selector_plan, tmp_path)


def test_successor_waits_for_completed_predecessor_lease_release(expansion_plan, monkeypatch, capsys):
    attempts = []
    def acquire(*_):
        attempts.append(True)
        if len(attempts) == 1:
            raise BlockingIOError('predecessor releasing lease')
    sleeps = []
    monkeypatch.setattr(MODULE.fcntl, 'flock', acquire)
    monkeypatch.setattr(MODULE.time, 'sleep', sleeps.append)
    MODULE.acquire_campaign_lease(object(), expansion_plan)
    assert len(attempts) == 2 and sleeps == [5.0]
    assert 'WAITING_GPU0_CAMPAIGN_LEASE' in capsys.readouterr().out


def test_frozen_matrix_and_read_only_validation(plan, capsys):
    MODULE.validate_plan(plan)
    assert MODULE.main([str(ROOT / 'config/portable_e2e_lr_ab_20260907.json'), '--validate-only']) == 0
    assert 'no training started' in capsys.readouterr().out


@pytest.mark.parametrize('field,value', [
    ('gpu_uuid', 'GPU-other'), ('seeds', [20260903]), ('steps', 10000000),
    ('batch_size', 8), ('split', 'test'), ('campaign_id', '../../escape'),
    ('dataset', '/tmp/data'), ('dataset', 'datasets/../outside'),
    ('source_commit', 'main'), ('arms', {'A_baseline': 0.0001, 'B_lower_lr': 0.0003}),
])
def test_scope_rejected(plan, field, value):
    changed = copy.deepcopy(plan)
    changed[field] = value
    with pytest.raises(ValueError):
        MODULE.validate_plan(changed)


def test_all_six_runs_are_paired_gpu0_val_only(plan, tmp_path):
    stages = list(MODULE.commands(plan, tmp_path, ROOT))
    assert len(stages) == 18
    assert len({str(item) for item, _, _, _ in stages}) == 6
    for _, _, command, stage in stages:
        assert command[command.index('--device') + 1] == 'cuda:0'
        assert command[command.index('--split') + 1] == ('train' if stage == 'train' else 'val')
        assert '--resume' not in command
        assert '--candidate-score-weight' not in command
    assert [stage for _, _, _, stage in stages] == ['train', 'evaluate', 'audit'] * 6


def test_gpu0_occupied_stops_without_touching_job(monkeypatch):
    replies = iter([f'0, {MODULE.GPU_UUID}\n1, GPU-other', f'{MODULE.GPU_UUID}, 123'])
    monkeypatch.setattr(MODULE, 'run_inventory', lambda *args: next(replies))
    with pytest.raises(RuntimeError, match='occupied'):
        MODULE.assert_gpu_idle(ROOT)


def test_gpu1_job_does_not_select_gpu1(monkeypatch):
    replies = iter([f'0, {MODULE.GPU_UUID}\n1, GPU-other', 'GPU-other, 123'])
    monkeypatch.setattr(MODULE, 'run_inventory', lambda *args: next(replies))
    MODULE.assert_gpu_idle(ROOT)


def test_reordered_physical_gpu_rejected(monkeypatch):
    monkeypatch.setattr(MODULE, 'run_inventory', lambda *args: f'1, {MODULE.GPU_UUID}')
    with pytest.raises(RuntimeError, match='physical GPU0'):
        MODULE.assert_gpu_idle(ROOT)


def test_python_import_environment_is_isolated(monkeypatch):
    for name in ('PYTHONPATH', 'PYTHONHOME', 'PYTHONSTARTUP', 'PYTHONUSERBASE'):
        monkeypatch.setenv(name, '/untrusted/location')
    monkeypatch.setenv('CUDA_VISIBLE_DEVICES', 'GPU-other')
    env = MODULE.stage_environment()
    assert all(name not in env for name in ('PYTHONPATH', 'PYTHONHOME', 'PYTHONSTARTUP', 'PYTHONUSERBASE'))
    assert env['CUDA_VISIBLE_DEVICES'] == MODULE.GPU_UUID
    assert env['PYTHONNOUSERSITE'] == '1'


@pytest.mark.parametrize('mutation', ['head', 'untracked', 'model'])
def test_source_or_model_changes_are_rejected(plan, monkeypatch, tmp_path, mutation):
    model = tmp_path / 'model.json'
    model.write_text('{}')
    expected_hash = MODULE.digest(model)
    if mutation == 'model':
        model.write_text('{"changed":true}')
    replies = iter(['f' * 40 if mutation == 'head' else plan['source_commit'],
        '?? portable_e2e/injected.py' if mutation == 'untracked' else ''])
    monkeypatch.setattr(MODULE, 'run_inventory', lambda *args: next(replies))
    with pytest.raises(RuntimeError, match='frozen plan|untracked changes|model config changed'):
        MODULE.verify_source(plan, ROOT, model, expected_hash)


@pytest.fixture
def stage_reports(tmp_path, plan):
    # HH_260906 - Use small report fixtures to verify cross-run identities without loading a checkpoint.
    item = tmp_path / 'seed_20260903/A_baseline'
    checkpoint = item / 'training/checkpoints/latest.pt'
    checkpoint.parent.mkdir(parents=True)
    checkpoint.write_bytes(b'opaque test checkpoint')
    (item / 'evaluation').mkdir()
    common = {'corpus_fingerprint_sha256': 'a' * 64, 'vehicle_control_approved': False}
    reports = {
        'train': dict(common, status='TRAINING_TARGET_REACHED', dataset_fingerprint_sha256='b' * 64,
            training_split='train', dataset_size=613, state={'global_step': 1540},
            loss_config={'xy_weight': 1.0, 'speed_weight': 0.2, 'yaw_weight': 0.1,
                'kinematic_speed_weight': 0.05, 'final_displacement_weight': 0.5, 'candidate_score_weight': 0.1}),
        'evaluate': dict(common, status='OPEN_LOOP_EVALUATION_COMPLETE',
            dataset_fingerprint_sha256='c' * 64, evaluation_split='val', sample_count=337,
            training_dataset_fingerprint_sha256='b' * 64, checkpoint_sha256=MODULE.digest(checkpoint)),
        'audit': dict(common, status='RUNTIME_GEOMETRY_AUDIT_COMPLETE',
            dataset_fingerprint_sha256='c' * 64, evaluation_split='val', geometry={'sample_count': 337},
            training_dataset_fingerprint_sha256='b' * 64, checkpoint_sha256=MODULE.digest(checkpoint),
            gate={'source': 'portable_e2e.runtime_geometry_gate.v8'}),
    }
    paths = {'train': item / 'training/run.json', 'evaluate': item / 'evaluation/metrics.json', 'audit': item / 'gate_v8.json'}
    for stage, report in reports.items():
        paths[stage].write_text(json.dumps(report))
    return item, checkpoint, reports, paths


def test_first_training_pins_corpus_and_all_stages_match(plan, stage_reports):
    item, checkpoint, _, _ = stage_reports
    state = {}
    for stage in ('train', 'evaluate', 'audit', 'train', 'evaluate', 'audit'):
        proof = MODULE.verify_stage_report(stage, item, checkpoint, state, plan)
        assert proof['corpus_fingerprint_sha256'] == 'a' * 64
        assert len(proof['sha256']) == 64
    assert state == {'corpus_fingerprint_sha256': 'a' * 64,
        'train_fingerprint_sha256': 'b' * 64, 'val_fingerprint_sha256': 'c' * 64}


def test_expansion_report_requires_train1147_and_preserves_val337(expansion_plan, stage_reports):
    item, checkpoint, reports, paths = stage_reports
    reports['train']['dataset_size'] = 1147
    paths['train'].write_text(json.dumps(reports['train']))
    state = {}
    for stage in ('train', 'evaluate', 'audit'):
        MODULE.verify_stage_report(stage, item, checkpoint, state, expansion_plan)
    reports['train']['dataset_size'] = 613
    paths['train'].write_text(json.dumps(reports['train']))
    with pytest.raises(RuntimeError, match='frozen training split'):
        MODULE.verify_stage_report('train', item, checkpoint, state, expansion_plan)


def test_selector_report_requires_score_weight_and_preserves_val_binding(selector_plan, stage_reports):
    item, checkpoint, reports, paths = stage_reports
    reports['train']['dataset_size'] = 1147
    reports['train']['loss_config']['candidate_score_weight'] = 0.5
    paths['train'].write_text(json.dumps(reports['train']))
    state = {}
    for stage in ('train', 'evaluate', 'audit'):
        MODULE.verify_stage_report(stage, item, checkpoint, state, selector_plan)
    assert state['val_fingerprint_sha256'] == 'c' * 64
    reports['train']['loss_config']['candidate_score_weight'] = 0.1
    paths['train'].write_text(json.dumps(reports['train']))
    with pytest.raises(RuntimeError, match='loss config'):
        MODULE.verify_stage_report('train', item, checkpoint, state, selector_plan)


@pytest.mark.parametrize('schema_fixture', ['plan', 'expansion_plan'])
def test_original_campaigns_reject_selector_loss_drift(request, stage_reports, schema_fixture):
    original_plan = request.getfixturevalue(schema_fixture)
    item, checkpoint, reports, paths = stage_reports
    reports['train']['dataset_size'] = MODULE.validate_plan(original_plan)['train_samples']
    reports['train']['loss_config']['candidate_score_weight'] = 0.5
    paths['train'].write_text(json.dumps(reports['train']))
    with pytest.raises(RuntimeError, match='loss config'):
        MODULE.verify_stage_report('train', item, checkpoint, {}, original_plan)


@pytest.mark.parametrize('stage,key,value', [
    ('train', 'corpus_fingerprint_sha256', 'd' * 64),
    ('train', 'dataset_fingerprint_sha256', 'd' * 64),
    ('train', 'dataset_size', 612),
    ('train', 'state', {'global_step': 1539}),
    ('evaluate', 'dataset_fingerprint_sha256', 'd' * 64),
    ('evaluate', 'corpus_fingerprint_sha256', 'd' * 64),
    ('evaluate', 'training_dataset_fingerprint_sha256', 'd' * 64),
    ('evaluate', 'checkpoint_sha256', 'd' * 64),
    ('evaluate', 'evaluation_split', 'test'),
    ('evaluate', 'sample_count', 336),
    ('audit', 'gate', {'source': 'portable_e2e.runtime_geometry_gate.v6'}),
    ('audit', 'geometry', {'sample_count': 336}),
    ('audit', 'vehicle_control_approved', True),
    ('audit', 'status', 'RUNNING'),
])
def test_stage_provenance_mutation_rejected(plan, stage_reports, stage, key, value):
    item, checkpoint, reports, paths = stage_reports
    state = {}
    for initial_stage in ('train', 'evaluate', 'audit'):
        MODULE.verify_stage_report(initial_stage, item, checkpoint, state, plan)
    reports[stage][key] = value
    paths[stage].write_text(json.dumps(reports[stage]))
    with pytest.raises(RuntimeError):
        MODULE.verify_stage_report(stage, item, checkpoint, state, plan)


@pytest.mark.parametrize('interruption', [KeyboardInterrupt, InterruptedError])
def test_interrupted_stage_persists_failed_status(plan, tmp_path, monkeypatch, interruption):
    # HH_260906 - Verify a paused campaign cannot retain a misleading RUNNING status after cleanup.
    workspace = tmp_path / 'personal/project/portable_e2e'
    workspace.mkdir(parents=True)
    dataset_parent = workspace.parent / 'dataset'
    (dataset_parent / Path(plan['dataset']).relative_to('datasets')).mkdir(parents=True)
    (workspace / 'datasets').symlink_to(dataset_parent, target_is_directory=True)
    repo = workspace / 'autoware_e2e'
    model = repo / plan['model_config']
    model.parent.mkdir(parents=True)
    model.write_text('{}')
    plan_path = tmp_path / 'plan.json'
    plan_path.write_text(json.dumps(plan))
    monkeypatch.setattr(MODULE, 'WORKSPACE', workspace)
    monkeypatch.setattr(MODULE.sys, 'prefix', str(workspace / 'venvs/py312'))
    monkeypatch.setattr(MODULE, 'run_inventory',
        lambda command, _repo: plan['source_commit'] if command[1] == 'rev-parse' else '')
    monkeypatch.setattr(MODULE, 'assert_gpu_idle', lambda _repo: None)

    def interrupt_stage(*_args, **_kwargs):
        raise interruption('requested pause')

    monkeypatch.setattr(MODULE, 'run_owned_stage', interrupt_stage)
    with pytest.raises(interruption):
        MODULE.main([str(plan_path)])
    root = workspace / 'runs/campaigns' / plan['campaign_id']
    status = json.loads((root / 'status.json').read_text())
    assert status['status'] == 'STOPPED_FAILURE_NO_PROMOTION'
    assert status['stages'][-1]['status'] == 'FAILED'
    assert status['error'].startswith(interruption.__name__)
    assert (root / 'plan.json').read_bytes() == plan_path.read_bytes()


@pytest.mark.parametrize('ending', ['timeout', 'sigterm', 'keyboard'])
def test_stage_timeout_or_interrupt_cleans_owned_descendants(tmp_path, ending):
    # HH_260906 - Interrupt an isolated harness and verify even signal-ignoring descendants are stopped.
    descendant = 'import signal,time; signal.signal(signal.SIGTERM,signal.SIG_IGN); time.sleep(60)'
    child = ('import os,signal,subprocess,sys,time;'
        'signal.signal(signal.SIGTERM,signal.SIG_IGN);'
        f'p=subprocess.Popen([sys.executable,"-c",{descendant!r}]);'
        'print(f"READY {os.getpid()} {p.pid}",flush=True); time.sleep(60)')
    harness = f'''import importlib.util,sys
spec=importlib.util.spec_from_file_location('campaign',{str(ROOT / 'scripts/e2e/run_portable_training_campaign.py')!r})
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
try:
    with module.termination_guard():
        module.run_owned_stage([sys.executable,'-c',{child!r}],{str(tmp_path)!r},module.stage_environment(),sys.stdout,timeout={0.8 if ending == 'timeout' else 30},cleanup_grace_seconds=0.2)
except BaseException as error:
    print(type(error).__name__,flush=True)
    sys.exit(23)
'''
    process = subprocess.Popen([sys.executable, '-c', harness], stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True)
    pgid = None
    try:
        assert select.select([process.stdout], [], [], 8)[0]
        line = process.stdout.readline().strip()
        assert line.startswith('READY '), line
        pgid, descendant_pid = map(int, line.split()[1:])
        assert pgid != os.getpgrp() and os.getpgid(descendant_pid) == pgid
        if ending != 'timeout':
            process.send_signal(signal.SIGTERM if ending == 'sigterm' else signal.SIGINT)
        output = process.communicate(timeout=8)[0]
        assert process.returncode == 23, output
        assert {'timeout': 'TimeoutExpired', 'sigterm': 'InterruptedError', 'keyboard': 'KeyboardInterrupt'}[ending] in output
        assert not MODULE.group_has_live_members(pgid)
    finally:
        if pgid is not None and MODULE.group_has_live_members(pgid):
            os.killpg(pgid, signal.SIGKILL)
        if process.poll() is None:
            process.kill()
            process.wait(timeout=5)
