"""HH_260906 - Check isolated K12 scorer fitting, exact two-GEMM reference and unchanged research boundaries."""

from dataclasses import replace
import hashlib
import json

import pytest
import torch
from torch import nn

from portable_e2e import frozen_stopmix_selector as module
from portable_e2e.contract import ContractError
from portable_e2e.losses import TrajectoryLossConfig, trajectory_loss


@pytest.fixture(autouse=True)
def cpu_single_thread():
    previous = torch.get_num_threads()
    torch.set_num_threads(1)
    yield
    torch.set_num_threads(previous)


def originals():
    with torch.random.fork_rng(devices=[]):
        torch.random.default_generator.manual_seed(99)
        return nn.Linear(256, 6), nn.Linear(256, 6)


def cache(split='train', count=8, heads=None):
    # HH_260906 - Synthetic cache fixtures do not assert any real model or driving performance.
    heads = originals() if heads is None else heads
    generator = torch.Generator().manual_seed(19 if split == 'train' else 20)
    fused = torch.randn(count, 256, generator=generator) * .02
    xy = torch.zeros(count, 12, 64, 2)
    xy[..., 0] = torch.arange(1, 65) * .1
    xy[..., 1] = torch.arange(12)[None, :, None] * .1
    with torch.no_grad():
        logits = torch.cat([torch.cat((heads[0](fused[i:i+4]), heads[1](fused[i:i+4])), dim=1)
            for i in range(0, count, 4)])
    return module.CachedStopmixSelectorData(split=split,
        sample_ids=tuple(f'{split}:{i}' for i in range(count)),
        episode_ids=tuple(f'{split}-episode-{i % 3 if split == "train" else 0}' for i in range(count)),
        fused=fused, candidate_xy=xy, candidate_speed=torch.ones(count, 12, 64),
        target_xy=xy[:, 0], target_speed_mps=torch.ones(count, 64), target_yaw_rad=torch.zeros(count, 64),
        target_valid=torch.ones(count, 64, dtype=torch.bool), original_logits=logits,
        source_checkpoint_sha256='a'*64, corpus_fingerprint_sha256='b'*64,
        dataset_fingerprint_sha256=('c' if split == 'train' else 'd')*64)


def small_fit(monkeypatch, arm='linear_pair_reset', callback=None, check_deadline=None):
    # HH_260906 - Short unit budgets are fixture-only; no public fit argument can alter the fixed campaign budget.
    monkeypatch.setattr(module, 'TRAIN_COUNT', 8)
    monkeypatch.setattr(module, 'VAL_COUNT', 5)
    monkeypatch.setattr(module, 'STEPS', 3)
    monkeypatch.setattr(module, 'SAMPLES_SEEN', 12)
    heads = originals()
    train, val = cache(heads=heads), cache('val', 5, heads)
    result = module.fit_frozen_selector(train, val, original_drive_head=heads[0], original_stop_head=heads[1],
        arm=arm, seed=123, callback=callback, check_deadline=check_deadline)
    return result, train, val, heads


def test_fixed_contract_and_unchanged_runtime_support():
    assert (module.STEPS, module.BATCH_SIZE, module.SAMPLES_SEEN) == (1540, 4, 6155)
    assert (module.TRAIN_COUNT, module.VAL_COUNT) == (1147, 337)
    assert (module.LEARNING_RATE, module.WEIGHT_DECAY, module.SCORE_WEIGHT, module.MAX_GRAD_NORM) == (1e-4, 1e-4, .1, 5.)
    assert len(TrajectoryLossConfig().to_dict()) == 6
    from portable_e2e.runtime_weight_bundle import SUPPORTED_MODEL_IDS
    from portable_e2e.model import PHYSICAL_STOPMIX_MODEL_ID
    assert PHYSICAL_STOPMIX_MODEL_ID not in SUPPORTED_MODEL_IDS
    assert module.ARTIFACT_ID not in SUPPORTED_MODEL_IDS


def test_cache_clones_graphs_masks_and_all_k12_candidates():
    data = cache()
    fused = data.fused.clone().requires_grad_()
    xy = data.candidate_xy.clone().requires_grad_()
    valid = data.target_valid.clone()
    valid[0, 31:] = False
    new = replace(data, fused=fused, candidate_xy=xy, target_valid=valid)
    digest = new.digest()
    with torch.no_grad():
        fused.zero_(); xy.zero_(); valid.fill_(True)
    assert new.digest() == digest
    assert not new.fused.requires_grad and not new.candidate_xy.requires_grad
    assert new.target_valid[0].sum() == 31
    assert new.to('cpu') is new


@pytest.mark.parametrize('fault', ['test', 'missing_logits', 'six_candidates', 'wrong_dtype', 'nan', 'inf_logits',
    'duplicate', 'episodes', 'hash', 'integer_mask', 'empty_mask', 'nonprefix'])
def test_invalid_cache_rejected(fault):
    data = cache()
    if fault == 'test': changes = {'split': 'test'}
    elif fault == 'missing_logits': changes = {'original_logits': None}
    elif fault == 'six_candidates': changes = {'candidate_xy': data.candidate_xy[:, :6]}
    elif fault == 'wrong_dtype': changes = {'fused': data.fused.double()}
    elif fault == 'nan': changes = {'candidate_speed': data.candidate_speed * float('nan')}
    elif fault == 'inf_logits': changes = {'original_logits': data.original_logits * float('inf')}
    elif fault == 'duplicate': changes = {'sample_ids': ('one',)*8}
    elif fault == 'episodes': changes = {'episode_ids': ('one',)}
    elif fault == 'hash': changes = {'source_checkpoint_sha256': 'unproven'}
    elif fault == 'integer_mask': changes = {'target_valid': data.target_valid.long()}
    else:
        mask = data.target_valid.clone()
        if fault == 'empty_mask': mask[0] = False
        else: mask[0, 2] = False
        changes = {'target_valid': mask}
    with pytest.raises(ContractError): replace(data, **changes)


@pytest.mark.parametrize('fault', ['size', 'episodes', 'overlap_sample', 'overlap_episode', 'checkpoint', 'corpus', 'fingerprint'])
def test_split_scope_rejects_incomplete_or_leaking_pairs(fault):
    train, val = cache(count=1147), cache('val', 337)
    if fault == 'size': train = cache(count=1146)
    elif fault == 'episodes': train = replace(train, episode_ids=('one',)*1147)
    elif fault == 'overlap_sample': val = replace(val, sample_ids=(train.sample_ids[0],)+val.sample_ids[1:])
    elif fault == 'overlap_episode': val = replace(val, episode_ids=(train.episode_ids[0],)*337)
    elif fault == 'checkpoint': val = replace(val, source_checkpoint_sha256='e'*64)
    elif fault == 'corpus': val = replace(val, corpus_fingerprint_sha256='e'*64)
    else: val = replace(val, dataset_fingerprint_sha256=train.dataset_fingerprint_sha256)
    with pytest.raises(ContractError): module.validate_pair(train, val)


@pytest.mark.parametrize('arm,count', [('linear_pair_reset', 3084), ('candidate_reset', 115201)])
def test_fresh_initialization_reproducible_and_cpu_rng_isolated(arm, count):
    before = torch.get_rng_state().clone()
    a, b = module.make_head(arm, 123), module.make_head(arm, 123)
    assert torch.equal(before, torch.get_rng_state())
    assert sum(p.numel() for p in a.parameters()) == count
    assert all(torch.equal(x, b.state_dict()[n]) for n, x in a.state_dict().items())
    assert all(p.requires_grad and p.grad is None for p in a.parameters())


@pytest.mark.parametrize('arm,seed', [('old_continue', 1), ('candidate_reset', True), ('candidate_reset', -1)])
def test_bad_arm_or_seed_rejected(arm, seed):
    with pytest.raises(ContractError): module.make_head(arm, seed)


@pytest.mark.parametrize('arm', module.ARMS)
def test_initialization_allocations_explicitly_cpu(monkeypatch, arm):
    # HH_260906 - Intercept allocation metadata without creating or querying a CUDA device.
    original = torch.empty
    calls = []
    def allocate(*args, **kwargs):
        calls.append(kwargs)
        assert kwargs['device'] == 'cpu' and kwargs['dtype'] == torch.float32
        return original(*args, **kwargs)
    monkeypatch.setattr(torch, 'empty', allocate)
    head = module.make_head(arm, 123)
    assert len(calls) == 4
    assert all(p.device.type == 'cpu' for p in head.parameters())


def test_explicit_cpu_linear_matches_original_class_rng_and_values():
    with torch.random.fork_rng(devices=[]):
        torch.random.default_generator.manual_seed(55)
        expected = nn.Linear(256, 6)
        expected_rng = torch.get_rng_state().clone()
        torch.random.default_generator.manual_seed(55)
        actual = module._cpu_linear(256, 6)
        assert torch.equal(expected_rng, torch.get_rng_state())
    assert type(actual) is nn.Linear
    assert all(torch.equal(v, actual.state_dict()[n]) for n, v in expected.state_dict().items())


def test_original_reference_two_linear_calls_exact_no_rng_or_original_mutation():
    drive, stop = originals()
    drive.weight.grad = torch.ones_like(drive.weight)
    before = (module._head_digest(drive), module._head_digest(stop), torch.get_rng_state().clone())
    reference = module.clone_original_heads(drive, stop)
    assert tuple(reference._modules) == ('drive', 'stop')
    assert module._head_digest(drive) == before[0] and module._head_digest(stop) == before[1]
    assert torch.equal(torch.get_rng_state(), before[2])
    x = torch.randn(9, 256)
    for i in range(0, 9, 4):
        assert torch.equal(reference(x[i:i+4]), torch.cat((drive(x[i:i+4]), stop(x[i:i+4])), dim=1))
    assert all(not p.requires_grad and p.grad is None for p in reference.parameters())
    assert all(p.data_ptr() != q.data_ptr() for p, q in zip(reference.drive.parameters(), drive.parameters()))
    with pytest.raises(ContractError): module.clone_original_heads(nn.Linear(256, 12), stop)


@pytest.mark.parametrize('arm', module.ARMS)
def test_only_head_receives_gradients(arm):
    data = cache(count=2)
    fused, xy, speed = (v.clone().requires_grad_() for v in (data.fused, data.candidate_xy, data.candidate_speed))
    head = module.make_head(arm, 123)
    module.score_head(head, arm, fused, xy, speed).sum().backward()
    assert fused.grad is xy.grad is speed.grad is None
    assert all(p.grad is not None and torch.isfinite(p.grad).all() for p in head.parameters())


@pytest.mark.parametrize('dtype', [torch.float32, torch.float64])
def test_candidate_geometry_normalization_and_permutation_equivariance(dtype):
    data = cache(count=2)
    fused, xy, speed = (v.to(dtype) for v in (data.fused, data.candidate_xy, data.candidate_speed))
    head = module.make_head('candidate_reset', 123).to(dtype)
    actual = module.score_head(head, 'candidate_reset', fused, xy, speed)
    expected = head(torch.cat((fused[:, None].expand(-1, 12, -1), xy.flatten(2)/120.,
        speed/module.PHYSICAL_MAXIMUM_SPEED_MPS), dim=2)).squeeze(-1)
    assert torch.equal(actual, expected)
    order = [11, 3, 8, 0, 10, 4, 6, 2, 9, 1, 7, 5]
    other = module.score_head(head, 'candidate_reset', fused, xy[:, order], speed[:, order])
    eps = torch.finfo(dtype).eps
    assert torch.allclose(other, actual[:, order], atol=eps, rtol=4*eps)
    changed = module.score_head(head, 'candidate_reset', fused, xy+10, speed)
    assert not torch.equal(actual, changed)


def test_original_composite_teacher_not_ade_and_exact_stop_ties():
    data = cache(count=2)
    speed = data.candidate_speed.clone(); speed[:, 0] = 20
    data = replace(data, candidate_speed=speed)
    target = module.composite_targets(data)
    expected = trajectory_loss(data.candidate_xy, data.candidate_speed, data.original_logits,
        data.target_xy, data.target_speed_mps, data.target_valid, TrajectoryLossConfig(), target_yaw=data.target_yaw_rad)
    assert torch.equal(target, expected['oracle_candidate_index'])
    assert not target.eq(0).any()
    xy = data.candidate_xy.clone(); xy[:, 6:] = 0
    speed = data.candidate_speed.clone(); speed[:, 6:] = 0
    stationary = replace(data, candidate_xy=xy, candidate_speed=speed,
        target_xy=torch.zeros_like(data.target_xy), target_speed_mps=torch.zeros_like(data.target_speed_mps))
    assert module.composite_targets(stationary).tolist() == [6, 6]
    head = module.make_head('candidate_reset', 1)
    logits = module.score_head(head, 'candidate_reset', stationary.fused, xy, speed)
    # HH_260906 - Shared MLP rows can differ by kernel roundoff; this does not relax exact original-head parity or oracle ties.
    eps = torch.finfo(logits.dtype).eps
    assert torch.allclose(logits[:, 6:], logits[:, 6:7].expand(-1, 6), atol=eps, rtol=4*eps)


@pytest.mark.parametrize('arm', module.ARMS)
def test_short_fit_returns_reports_logits_targets_and_immutable_caches(monkeypatch, arm):
    records = []
    result, train, val, heads = small_fit(monkeypatch, arm, records.append)
    report = result['report']
    assert report['status'] == 'RESEARCH_HEAD_TRAINING_COMPLETE_NOT_PROMOTED'
    assert report['final_state']['samples_seen'] == 12
    assert len(records) == len(report['history']) == 3
    assert report['cache_sha256_before'] == report['cache_sha256_after'] == {'train': train.digest(), 'val': val.digest()}
    assert report['original_head_sha256_before'] == report['original_head_sha256_after']
    assert all(x['byte_equal'] and x['max_abs_logit_difference'] == 0 for x in report['original_logit_parity'].values())
    assert all(not report[n] for n in ('generator_parameters_trained', 'training_data_approved', 'vehicle_control_approved', 'automatic_promotion', 'runtime_bundle_supported'))
    assert report['initial_head_sha256'] != report['final_head_sha256']
    assert result['final_logits_cpu']['train'].shape == (8, 12)
    assert torch.equal(result['oracle_indices_cpu']['train'], module.composite_targets(train))
    assert report['post_training_metrics']['val']['sample_count'] == 5
    assert all(p.grad is None for h in heads for p in h.parameters())
    records[0]['global_step'] = -1
    assert report['history'][0]['global_step'] == 1


def test_both_arms_have_exact_same_sampling_and_oracles(monkeypatch):
    a, *_ = small_fit(monkeypatch, 'linear_pair_reset')
    b, *_ = small_fit(monkeypatch, 'candidate_reset')
    assert a['report']['sampling_order_sha256'] == b['report']['sampling_order_sha256']
    assert a['report']['composite_oracle_sha256'] == b['report']['composite_oracle_sha256']


def test_fail_closed_baseline_difference_before_any_optimization(monkeypatch):
    monkeypatch.setattr(module, 'TRAIN_COUNT', 8); monkeypatch.setattr(module, 'VAL_COUNT', 5)
    heads = originals(); train, val = cache(heads=heads), cache('val', 5, heads)
    wrong = train.original_logits.clone(); wrong[0, 0] += 1e-6
    train = replace(train, original_logits=wrong)
    monkeypatch.setattr(torch.optim, 'AdamW', lambda *a, **k: pytest.fail('optimizer created before baseline proof'))
    with pytest.raises(ContractError, match='exact baseline parity'):
        module.fit_frozen_selector(train, val, original_drive_head=heads[0], original_stop_head=heads[1], arm='linear_pair_reset', seed=123)


def test_deadline_interrupts_pretraining_evaluation(monkeypatch):
    def stop(): raise TimeoutError('declared deadline')
    with pytest.raises(TimeoutError, match='declared deadline'): small_fit(monkeypatch, check_deadline=stop)


def test_callback_failure_retains_original_inputs(monkeypatch):
    records = []
    def fail(record): records.append(record); raise TimeoutError('owned writer deadline')
    with pytest.raises(TimeoutError): small_fit(monkeypatch, callback=fail)
    assert len(records) == 1 and records[0]['global_step'] == 1


def test_cache_mutation_detected_even_when_callback_fails(monkeypatch):
    monkeypatch.setattr(module, 'TRAIN_COUNT', 8); monkeypatch.setattr(module, 'VAL_COUNT', 5)
    heads = originals(); train, val = cache(heads=heads), cache('val', 5, heads)
    def corrupt(record): train.candidate_xy[0, 0, 0, 0] += 1; raise RuntimeError('failure')
    with pytest.raises(ContractError, match='cache mutated'):
        module.fit_frozen_selector(train, val, original_drive_head=heads[0], original_stop_head=heads[1],
            arm='linear_pair_reset', seed=123, callback=corrupt)


def test_original_head_mutation_rejected_on_exception(monkeypatch):
    monkeypatch.setattr(module, 'TRAIN_COUNT', 8); monkeypatch.setattr(module, 'VAL_COUNT', 5)
    heads = originals(); train, val = cache(heads=heads), cache('val', 5, heads)
    def corrupt(record):
        with torch.no_grad(): heads[1].weight[0, 0] += 1
        raise RuntimeError('failure')
    with pytest.raises(ContractError, match='two-head reference mutated'):
        module.fit_frozen_selector(train, val, original_drive_head=heads[0], original_stop_head=heads[1],
            arm='linear_pair_reset', seed=123, callback=corrupt)


@pytest.mark.parametrize('fault', ['loss', 'gradient', 'parameters', 'optimizer_state'])
def test_nonfinite_optimization_fails_closed(monkeypatch, fault):
    if fault == 'loss':
        original = module.F.cross_entropy
        def corrupt(*a, **k):
            result = original(*a, **k)
            return result * float('nan') if torch.is_grad_enabled() and a[0].requires_grad else result
        monkeypatch.setattr(module.F, 'cross_entropy', corrupt)
    elif fault == 'gradient':
        original = module.make_head
        def head(*a, **k):
            result = original(*a, **k)
            next(result.parameters()).register_hook(lambda gradient: gradient * float('nan'))
            return result
        monkeypatch.setattr(module, 'make_head', head)
    else:
        original = torch.optim.AdamW.step
        def step(optimizer, *a, **k):
            result = original(optimizer, *a, **k)
            with torch.no_grad():
                parameter = optimizer.param_groups[0]['params'][0]
                if fault == 'optimizer_state': optimizer.state[parameter]['exp_avg'].fill_(float('inf'))
                else: parameter.fill_(float('nan'))
            return result
        monkeypatch.setattr(torch.optim.AdamW, 'step', step)
    with pytest.raises(ContractError, match='nonfinite'): small_fit(monkeypatch)


def test_conflicting_targets_for_identical_context_are_not_filtered():
    data = cache(count=2)
    fused = data.fused.clone(); fused[1] = fused[0]
    xy = data.candidate_xy.clone(); xy[1] = xy[0]
    target = data.target_xy.clone(); target[1, :, 0] += 20
    conflicting = replace(data, fused=fused, candidate_xy=xy, target_xy=target)
    assert len(conflicting) == 2 and conflicting.sample_ids == data.sample_ids
    assert torch.equal(conflicting.fused[0], conflicting.fused[1])
    assert not torch.equal(conflicting.target_xy[0], conflicting.target_xy[1])


def test_complete_fixed_budget_exposures_and_sampling_digest():
    heads = originals(); train, val = cache(count=1147, heads=heads), cache('val', 337, heads)
    result = module.fit_frozen_selector(train, val, original_drive_head=heads[0], original_stop_head=heads[1], arm='linear_pair_reset', seed=123)
    report = result['report']; history = report['history']
    assert len(history) == 1540 and sum(r['batch_samples'] for r in history) == 6155
    assert sum(r['batch_samples'] == 3 for r in history) == 5
    assert report['final_state'] == {'global_step': 1540, 'samples_seen': 6155, 'last_epoch_index': 5}
    expected = []
    for epoch in range(6):
        for indices in module._epoch_batches(1147, batch_size=4, seed=123, epoch=epoch):
            if len(expected) == 1540: break
            expected.append({'epoch': epoch, 'indices': list(indices)})
    assert report['sampling_order_sha256'] == hashlib.sha256(json.dumps(expected, separators=(',', ':')).encode()).hexdigest()
    assert not torch.cuda.is_initialized()
