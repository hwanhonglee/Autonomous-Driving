"""HH_260906 - Verify the new paired objective and overnight budget without SSH or GPU execution."""

from datetime import datetime, timedelta, timezone
import hashlib
import json
from pathlib import Path

import pytest

from scripts.e2e import run_portable_training_campaign as runner

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def plan():
    # HH_260906 - The synthetic source pin belongs only to unit tests; dispatch requires the committed real source.
    return dict(schema=runner.CANDIDATE_REGRET_SCHEMA,
        campaign_id='hh260909-candidate-regret-ab-3seeds-v1', source_commit='e' * 40,
        gpu_uuid=runner.GPU_UUID, seeds=[20260903, 20260904, 20260905],
        arms=dict(runner.CANDIDATE_REGRET_ARMS), candidate_regret_weights=dict(runner.CANDIDATE_REGRET_WEIGHTS),
        candidate_score_weight=0.1, steps=1540, batch_size=4, split='val',
        dataset=runner.EXPANDED_DATASET, dataset_manifest_sha256=runner.EXPANDED_MANIFEST_SHA256,
        model_config=runner.NO_ACCEL_MODELS['A_physical_input'],
        model_config_sha256=runner.NO_ACCEL_MODEL_SHA256['A_physical_input'],
        expected_train_samples=1147, expected_val_samples=337,
        finish_before_utc=runner.CANDIDATE_REGRET_DEADLINE,
        stage_timeout_seconds=dict(runner.CANDIDATE_REGRET_STAGE_TIMEOUTS), finish_reserve_seconds=300)


def test_six_fresh_runs_change_only_the_regret_weight(plan, tmp_path):
    assert runner.validate_plan(plan) == dict(train_samples=1147, val_samples=337, run_count=6, stage_count=18)
    stages = list(runner.commands(plan, tmp_path, ROOT))
    assert len(stages) == 18
    for item, checkpoint, command, stage in stages:
        assert '--resume' not in command
        assert command[command.index('--device') + 1] == 'cuda:0'
        assert command[command.index('--split') + 1] == ('train' if stage == 'train' else 'val')
        if stage == 'train':
            assert command[command.index('--candidate-regret-weight') + 1] == str(runner.CANDIDATE_REGRET_WEIGHTS[item.name])
            assert command[command.index('--candidate-score-weight') + 1] == '0.1'
            assert command[command.index('--max-steps') + 1] == '1540'
            assert command[command.index('--model-config') + 1] == str(ROOT / plan['model_config'])
            assert command[command.index('--seed') + 1] == item.parent.name[5:]
            assert '--checkpoint' not in command
        else:
            assert command[command.index('--checkpoint') + 1] == str(checkpoint)
            assert '--candidate-regret-weight' not in command
    assert runner.wait_for_prerequisite(plan, tmp_path) is None


@pytest.mark.parametrize('field,value', [
    ('candidate_regret_weights', {'A_original_loss': False, 'B_cost_aware_selector': 0.1}),
    ('candidate_regret_weights', {'A_original_loss': 0.0, 'B_cost_aware_selector': 0.2}),
    ('candidate_regret_weights', {'A_original_loss': 0.0, 'B_cost_aware_selector': float('nan')}),
    ('candidate_score_weight', 0.5), ('steps', 3080), ('batch_size', 8), ('split', 'test'),
    ('expected_train_samples', 1147.0), ('expected_val_samples', 338),
    ('source_commit', 'HEAD'), ('campaign_id', 'unplanned'), ('gpu_uuid', 'GPU1'),
    ('dataset', 'datasets/../outside'), ('dataset_manifest_sha256', 'a' * 64),
    ('model_config', 'portable_e2e/config/perspective_trajectory_physical_no_accel_v1.model.json'),
    ('model_config_sha256', 'a' * 64), ('model_configs', {}), ('resume', 'latest.pt'),
    ('checkpoint', 'old.pt'), ('baseline_campaign_id', 'old'), ('prerequisite_campaign_id', 'old'),
    ('finish_before_utc', '2026-09-10T01:00:00Z'), ('finish_reserve_seconds', 0),
    ('stage_timeout_seconds', {'train': 6000, 'evaluate': 120, 'audit': 120}),
    ('arms', {'B_cost_aware_selector': 0.0001, 'A_original_loss': 0.0001}),
])
def test_unreviewed_scope_is_rejected(plan, field, value):
    plan[field] = value
    with pytest.raises(ValueError):
        runner.validate_plan(plan)


def test_deadline_reserves_all_timeouts_and_cleanup(plan):
    stages = ['train', 'evaluate', 'audit'] * 6
    deadline = datetime(2026, 9, 9, 1, tzinfo=timezone.utc)
    needed = 6 * (600 + 120 + 120) + 300
    runner.verify_finish_budget(plan, stages, observed_at=deadline - timedelta(seconds=needed))
    with pytest.raises(TimeoutError, match='remaining campaign'):
        runner.verify_finish_budget(plan, stages, observed_at=deadline - timedelta(seconds=needed - .001))
    runner.verify_finish_budget(plan, ['audit'], observed_at=deadline - timedelta(seconds=420))
    with pytest.raises(TimeoutError):
        runner.verify_finish_budget(plan, ['audit'], observed_at=deadline)
    with pytest.raises(ValueError, match='timezone'):
        runner.verify_finish_budget(plan, stages, observed_at=datetime(2026, 9, 9))
    with pytest.raises(ValueError, match='unreviewed remaining'):
        runner.verify_finish_budget(plan, ['test'], observed_at=deadline)


def test_expired_plan_validate_only_does_not_launch(plan, tmp_path, capsys):
    path = tmp_path / 'plan.json'; path.write_text(json.dumps(plan))
    assert runner.main([str(path), '--validate-only']) == 0
    assert 'no training started' in capsys.readouterr().out


def test_model_and_manifest_are_pinned(plan, tmp_path, monkeypatch):
    config = tmp_path / plan['model_config']; config.parent.mkdir(parents=True)
    config.write_bytes((ROOT / plan['model_config']).read_bytes())
    assert runner.campaign_model_configs(plan, tmp_path)['shared']['sha256'] == plan['model_config_sha256']
    config.write_text('{}')
    with pytest.raises(RuntimeError, match='reviewed physical'):
        runner.campaign_model_configs(plan, tmp_path)
    (tmp_path / 'dataset.json').write_text('{}')
    with pytest.raises(RuntimeError, match='manifest'):
        runner.verify_dataset_manifest(plan, tmp_path)


@pytest.fixture
def reports(plan, tmp_path):
    item = tmp_path / 'seed_20260903/B_cost_aware_selector'
    checkpoint = item / 'training/checkpoints/latest.pt'
    checkpoint.parent.mkdir(parents=True); checkpoint.write_bytes(b'opaque test bytes, never loaded')
    config = json.loads((ROOT / plan['model_config']).read_text())
    canonical = hashlib.sha256(json.dumps(config, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    state = {'model_config': config, 'model_config_canonical_sha256': canonical}
    common = dict(corpus_fingerprint_sha256='a' * 64, model_parameter_count=954590, vehicle_control_approved=False)
    documents = {
        'train': dict(common, status='TRAINING_TARGET_REACHED', training_split='train', dataset_size=1147,
            dataset_fingerprint_sha256='b' * 64, model_config=config,
            state={'global_step': 1540, 'domain_samples_seen': {'carla': 6155}},
            train_config={'seed': 20260903, 'learning_rate': 0.0001, 'batch_size': 4},
            loss_config={'xy_weight': 1.0, 'speed_weight': 0.2, 'yaw_weight': 0.1,
                'kinematic_speed_weight': 0.05, 'final_displacement_weight': 0.5,
                'candidate_score_weight': 0.1, 'candidate_regret_weight': 0.1}),
        'evaluate': dict(common, status='OPEN_LOOP_EVALUATION_COMPLETE', evaluation_split='val', sample_count=337,
            dataset_fingerprint_sha256='c' * 64, training_dataset_fingerprint_sha256='b' * 64,
            model_config_sha256=canonical, checkpoint_sha256=runner.digest(checkpoint)),
        'audit': dict(common, status='RUNTIME_GEOMETRY_AUDIT_COMPLETE', evaluation_split='val',
            geometry={'sample_count': 337}, gate={'source': 'portable_e2e.runtime_geometry_gate.v8'},
            dataset_fingerprint_sha256='c' * 64, training_dataset_fingerprint_sha256='b' * 64,
            model_config_sha256=canonical, checkpoint_sha256=runner.digest(checkpoint))}
    paths = {'train': 'training/run.json', 'evaluate': 'evaluation/metrics.json', 'audit': 'gate_v8.json'}
    for stage, value in documents.items():
        path = item / paths[stage]; path.parent.mkdir(exist_ok=True)
        path.write_text(json.dumps(value))
    return item, checkpoint, state, documents, paths


def test_b_report_requires_explicit_nonzero_regret_and_bound_model(plan, reports):
    item, checkpoint, state, documents, paths = reports
    for stage in ('train', 'evaluate', 'audit'):
        runner.verify_stage_report(stage, item, checkpoint, state, plan)
    documents['train']['loss_config'].pop('candidate_regret_weight')
    (item / paths['train']).write_text(json.dumps(documents['train']))
    with pytest.raises(RuntimeError, match='loss config'):
        runner.verify_stage_report('train', item, checkpoint, state, plan)


@pytest.mark.parametrize('stage,field,value', [
    ('train', 'model_config', {}), ('train', 'model_parameter_count', 1),
    ('train', 'state', {'global_step': 1540, 'domain_samples_seen': {'carla': 100}}),
    ('train', 'train_config', {'seed': 99, 'learning_rate': 0.0001, 'batch_size': 4}),
    ('evaluate', 'model_config_sha256', 'f' * 64), ('audit', 'model_config_sha256', 'f' * 64),
])
def test_forged_model_or_exposure_rejected(plan, reports, stage, field, value):
    item, checkpoint, state, documents, paths = reports
    documents[stage][field] = value
    (item / paths[stage]).write_text(json.dumps(documents[stage]))
    with pytest.raises(RuntimeError, match='candidate-regret'):
        runner.verify_stage_report(stage, item, checkpoint, state, plan)


def test_baseline_loss_remains_legacy_six_fields(plan, reports):
    item, checkpoint, state, documents, paths = reports
    baseline = item.with_name('A_original_loss'); item.rename(baseline)
    checkpoint = baseline / checkpoint.relative_to(item)
    documents['train']['loss_config'].pop('candidate_regret_weight')
    (baseline / paths['train']).write_text(json.dumps(documents['train']))
    runner.verify_stage_report('train', baseline, checkpoint, state, plan)
    documents['train']['loss_config']['candidate_regret_weight'] = 0.0
    (baseline / paths['train']).write_text(json.dumps(documents['train']))
    with pytest.raises(RuntimeError, match='loss config'):
        runner.verify_stage_report('train', baseline, checkpoint, state, plan)
