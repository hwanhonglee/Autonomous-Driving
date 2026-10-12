"""HH_260906 - Verify frozen-cache extraction and guarded research execution without a live GPU."""

from datetime import datetime, timedelta, timezone
from types import SimpleNamespace
import json

import pytest
import torch
from torch import nn

from portable_e2e.contract import ContractError
from portable_e2e.dataset import FEATURE_NAMES
from scripts.e2e import run_portable_frozen_selector as runner


class TinyDataset:
    # HH_260906 - A synthetic extraction fixture preserves cache shapes, not camera realism.
    def __init__(self, count=5, split='train'):
        self.split = split
        self.fingerprint_sha256 = 'a' * 64
        self.examples = tuple(SimpleNamespace(token=f'{split}-{i}', episode_id=f'{split}-episode') for i in range(count))

    def __len__(self):
        return len(self.examples)

    def __getitem__(self, index):
        return {'sample_id': self.examples[index].token, 'images': torch.tensor([float(index), 1.0]),
            'calibration': torch.zeros(1), 'ego_history': torch.zeros(10, len(FEATURE_NAMES)),
            'ego_history_mask': torch.ones(10, dtype=torch.bool), 'route_xy': torch.zeros(128, 2),
            'route_mask': torch.ones(128, dtype=torch.bool), 'target_xy': torch.zeros(64, 2),
            'target_speed_mps': torch.zeros(64), 'target_yaw_rad': torch.zeros(64),
            'target_valid': torch.ones(64, dtype=torch.bool)}


class TinyGenerator(nn.Module):
    def __init__(self, mutate=False, repeat=False):
        super().__init__()
        self.fusion = nn.Linear(2, 256)
        self.candidate_head = nn.Linear(256, 6)
        self.register_buffer('count', torch.zeros(1))
        self.mutate, self.repeat = mutate, repeat
        self.requires_grad_(False)
        self.eval()

    def forward(self, images, *unused):
        fused = self.fusion(images)
        if self.repeat:
            self.fusion(images)
        if self.mutate:
            self.count.add_(1)
        batch = images.shape[0]
        return torch.zeros(batch, 6, 64, 2), torch.zeros(batch, 6, 64), self.candidate_head(fused)


def future_deadline():
    return (datetime.now(timezone.utc) + timedelta(hours=1)).isoformat()


def extract(model, dataset):
    return runner.extract_cache(model, dataset, device=torch.device('cpu'),
        checkpoint_sha256='b' * 64, deadline_utc=future_deadline())


def test_frozen_plan_matches_implementation():
    plan = runner.load_plan()
    assert plan['arms'] == list(runner.ARMS)
    assert plan['seeds'] == list(runner.PARENT_SHA256)
    assert plan['steps'] == 1540
    assert plan['deadline_utc'] == '2026-09-08T01:00:00+00:00'


@pytest.mark.parametrize('field,value', [('steps', 1), ('gpu_uuid', 'other'), ('train_samples', 337),
    ('arms', ['candidate_reset']), ('deadline_utc', '2099-01-01T00:00:00Z')])
def test_plan_tampering_rejected(tmp_path, field, value):
    import json
    plan = runner.load_plan()
    plan[field] = value
    path = tmp_path / 'plan.json'
    path.write_text(json.dumps(plan))
    with pytest.raises(ContractError, match='reviewed experiment'):
        runner.load_plan(path)


def test_deadline_rejects_missing_timezone_and_expired_boundary():
    with pytest.raises(ContractError, match='timezone'):
        runner.check_deadline('2099-01-01T00:00:00')
    with pytest.raises(ContractError, match='reached'):
        runner.check_deadline('2000-01-01T00:00:00Z')
    assert runner.check_deadline(future_deadline()).tzinfo is not None


def test_tensor_hash_binds_names_shapes_dtypes_and_values():
    tensor = torch.zeros(2, 3)
    original = runner.tensor_mapping_sha256({'weight': tensor})
    variants = ({'bias': tensor}, {'weight': tensor.reshape(3, 2)},
        {'weight': tensor.double()}, {'weight': tensor + 1})
    assert all(runner.tensor_mapping_sha256(value) != original for value in variants)
    assert runner.tensor_mapping_sha256({'weight': tensor.clone()}) == original
    with pytest.raises(ContractError):
        runner.tensor_mapping_sha256({'bad': 'not a tensor'})


def test_cache_preserves_generator_and_exact_order_and_detaches():
    model, dataset = TinyGenerator(), TinyDataset()
    before = runner.tensor_mapping_sha256(model.state_dict())
    cache, extra, observed = extract(model, dataset)
    assert before == observed == runner.tensor_mapping_sha256(model.state_dict())
    assert cache.sample_ids == tuple(item.token for item in dataset.examples)
    assert cache.fused.shape == (5, 256)
    assert cache.candidate_xy.shape == (5, 6, 64, 2)
    assert extra['route_xy'].shape == (5, 128, 2)
    assert all(not getattr(cache, name).requires_grad for name in runner.CACHE_NAMES)
    assert not model.fusion._forward_hooks
    assert not model.training and all(not item.requires_grad for item in model.parameters())
    with torch.no_grad():
        assert torch.allclose(model.candidate_head(cache.fused), cache.original_logits, atol=1e-6)
    payload = runner.cache_payload(cache, extra)
    assert payload['checkpoint_id'] == runner.CACHE_ID
    assert payload['checkpoint_id'] != 'portable_e2e.pytorch_checkpoint.v1'
    assert payload['vehicle_control_approved'] is False


@pytest.mark.parametrize('mode', ['training', 'gradients', 'test'])
def test_cache_refuses_unfrozen_or_test_input(mode):
    model, dataset = TinyGenerator(), TinyDataset()
    if mode == 'training':
        model.train()
    elif mode == 'gradients':
        model.candidate_head.requires_grad_(True)
    else:
        dataset.split = 'test'
    with pytest.raises(ContractError):
        extract(model, dataset)


@pytest.mark.parametrize('mutate,repeat,message', [(True, False, 'buffers changed'), (False, True, 'exactly once')])
def test_cache_detects_mutated_buffers_and_multiple_fusions(mutate, repeat, message):
    model = TinyGenerator(mutate=mutate, repeat=repeat)
    with pytest.raises(ContractError, match=message):
        extract(model, TinyDataset())
    assert not model.fusion._forward_hooks


def test_cache_rejects_changed_sample_order():
    class WrongOrder(TinyDataset):
        def __getitem__(self, index):
            result = super().__getitem__(index)
            result['sample_id'] = f'wrong-{index}'
            return result
    with pytest.raises(ContractError, match='exact ordered split'):
        extract(TinyGenerator(), WrongOrder())


def test_route_audit_rejects_train_or_partial_validation(tmp_path):
    cache, extra, _ = extract(TinyGenerator(), TinyDataset())
    with pytest.raises(ContractError, match='complete val337'):
        runner.audit_and_render(cache, extra, cache.original_logits, tmp_path / 'unused', 'test')
    cache, extra, _ = extract(TinyGenerator(), TinyDataset(split='val'))
    with pytest.raises(ContractError, match='complete val337'):
        runner.audit_and_render(cache, extra, cache.original_logits, tmp_path / 'unused', 'test')
    assert not (tmp_path / 'unused').exists()


def test_environment_rejects_nonpersonal_interpreter_before_output(tmp_path):
    with pytest.raises(ContractError, match='personal py312'):
        runner.validate_environment('a' * 40, tmp_path / 'unused', future_deadline())
    assert not (tmp_path / 'unused').exists()


def seed_result_fixture(tmp_path):
    # HH_260906 - Synthetic byte files exercise completion/hash checks without loading a model or touching CUDA.
    seed, source, plan_sha = 20260903, 'c' * 40, 'd' * 64
    report = {'schema': runner.SCHEMA, 'status': 'HEAD_TRAIN_EVAL_AUDIT_COMPLETE_NOT_PROMOTED',
        'seed': seed, 'parent_checkpoint_sha256': runner.PARENT_SHA256[seed], 'plan_sha256': plan_sha,
        'vehicle_control_approved': False, 'automatic_promotion': False,
        'test_evaluated': False, 'test_used_for_training_or_selection': False,
        'source': {'diagnostic_source_commit': source}, 'generator_state_sha256': 'e' * 64,
        'generator_state_after_sha256': 'e' * 64, 'arms': [], 'cache_files': {}}
    for split, count in (('train', 1147), ('val', 337)):
        path = tmp_path / f'{split}_cache.pt'
        path.write_bytes(b'synthetic cache')
        report['cache_files'][split] = {'sample_count': count, 'sha256': runner.file_sha256(path)}
    for arm in runner.ARMS:
        folder = tmp_path / arm
        folder.mkdir()
        head = {'artifact_id': 'portable_e2e.frozen_selector_research_head.v1',
            'status': 'RESEARCH_HEAD_TRAINING_COMPLETE_NOT_PROMOTED', 'arm': arm, 'seed': seed,
            'source_checkpoint_sha256': runner.PARENT_SHA256[seed], 'steps': 1540, 'batch_size': 4,
            'vehicle_control_approved': False, 'automatic_promotion': False,
            'final_state': {'global_step': 1540, 'samples_seen': 6155}}
        (folder / 'report.json').write_text(json.dumps(head))
        for name in ('head_only.pt', 'final_logits.pt', 'metrics.jsonl', 'geometry.json'):
            (folder / name).write_bytes(b'synthetic retained evidence')
        report['arms'].append({'arm': arm, 'status': 'COMPLETE_NOT_PROMOTED',
            'files': {name: runner.file_sha256(folder / name) for name in
                ('report.json', 'head_only.pt', 'final_logits.pt', 'metrics.jsonl', 'geometry.json')}})
    path = tmp_path / 'result.json'
    path.write_text(json.dumps(report))
    return path, report, seed, source, plan_sha


def test_completed_seed_requires_retained_files(tmp_path):
    path, report, seed, source, plan_sha = seed_result_fixture(tmp_path)
    assert runner.validate_seed_result(path, seed, source, plan_sha) == report
    (tmp_path / runner.ARMS[0] / 'head_only.pt').write_bytes(b'changed')
    with pytest.raises(ContractError, match='SHA-256 mismatch'):
        runner.validate_seed_result(path, seed, source, plan_sha)


@pytest.mark.parametrize('fault', ['promotion', 'test', 'wrong_seed', 'source', 'generator',
    'missing_generator', 'arm_order', 'partial_cache', 'partial_arm', 'cache_bytes'])
def test_seed_result_rejects_false_completion(tmp_path, fault):
    path, report, seed, source, plan_sha = seed_result_fixture(tmp_path)
    if fault == 'promotion': report['automatic_promotion'] = True
    elif fault == 'test': report['test_evaluated'] = True
    elif fault == 'wrong_seed': report['seed'] += 1
    elif fault == 'source': report['source']['diagnostic_source_commit'] = 'f' * 40
    elif fault == 'generator': report['generator_state_after_sha256'] = 'f' * 64
    elif fault == 'missing_generator':
        report.pop('generator_state_sha256')
        report.pop('generator_state_after_sha256')
    elif fault == 'arm_order': report['arms'].reverse()
    elif fault == 'partial_cache': report['cache_files']['val']['sample_count'] = 336
    elif fault == 'partial_arm': report['arms'][0]['status'] = 'RUNNING'
    else: (tmp_path / 'train_cache.pt').write_bytes(b'changed')
    path.write_text(json.dumps(report))
    with pytest.raises(ContractError):
        runner.validate_seed_result(path, seed, source, plan_sha)


def test_seed_result_rejects_short_fit_even_when_hash_updated(tmp_path):
    path, report, seed, source, plan_sha = seed_result_fixture(tmp_path)
    head_path = tmp_path / runner.ARMS[0] / 'report.json'
    head = json.loads(head_path.read_text())
    head['final_state']['samples_seen'] = 6154
    head_path.write_text(json.dumps(head))
    report['arms'][0]['files']['report.json'] = runner.file_sha256(head_path)
    path.write_text(json.dumps(report))
    with pytest.raises(ContractError, match='exposure budget'):
        runner.validate_seed_result(path, seed, source, plan_sha)


def test_campaign_does_not_run_under_nonpersonal_interpreter(tmp_path):
    with pytest.raises(ContractError, match='personal py312'):
        runner.run_campaign('a' * 40, tmp_path / 'unused', future_deadline())
    assert not (tmp_path / 'unused').exists()


def test_cli_requires_exactly_one_seed_selection():
    shared = ['--expected-source-commit', 'a' * 40, '--output-dir', '/unused',
        '--deadline-utc', future_deadline()]
    assert runner.parse_args(['--all-seeds', *shared]).all_seeds
    assert runner.parse_args(['--seed', '20260903', *shared]).seed == 20260903
    with pytest.raises(SystemExit): runner.parse_args(shared)
    with pytest.raises(SystemExit): runner.parse_args(['--all-seeds', '--seed', '20260903', *shared])
