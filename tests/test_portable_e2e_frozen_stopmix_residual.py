"""HH_260906 - Verify zero-residual parity and fixed-budget immutable-cache information ablations."""

from dataclasses import replace
import hashlib
import json
from pathlib import Path

import pytest
import torch
from torch import nn

from portable_e2e import frozen_stopmix_residual as module
from portable_e2e.contract import ContractError
from portable_e2e.losses import TrajectoryLossConfig, trajectory_loss


@pytest.fixture(autouse=True)
def cpu_single_thread():
    previous = torch.get_num_threads()
    torch.set_num_threads(1)
    yield
    torch.set_num_threads(previous)


def cache(split='train', count=8):
    # HH_260906 - All fixtures are synthetic and do not claim any measured vehicle or model outcome.
    generator = torch.Generator().manual_seed(19 if split == 'train' else 20)
    fused = torch.randn(count, 256, generator=generator) * .1
    xy = torch.zeros(count, 12, 64, 2)
    xy[..., 0] = torch.arange(1, 65) * .1
    xy[..., 1] = torch.arange(12)[None, :, None] * .1
    logits = torch.randn(count, 12, generator=generator)
    return module.CachedStopmixSelectorData(split=split,
        sample_ids=tuple(f'{split}:{i}' for i in range(count)),
        episode_ids=tuple(f'{split}-episode-{i % 3 if split == "train" else 0}' for i in range(count)),
        fused=fused, candidate_xy=xy, candidate_speed=torch.ones(count, 12, 64),
        target_xy=xy[:, 0], target_speed_mps=torch.ones(count, 64), target_yaw_rad=torch.zeros(count, 64),
        target_valid=torch.ones(count, 64, dtype=torch.bool), original_logits=logits,
        source_checkpoint_sha256='a'*64, corpus_fingerprint_sha256='b'*64,
        dataset_fingerprint_sha256=('c' if split == 'train' else 'd')*64)


def small_budget(monkeypatch):
    # HH_260906 - Only unit fixtures alter constants; the public fit has no configurable budget or loss arguments.
    monkeypatch.setattr(module.cache_core, 'TRAIN_COUNT', 8)
    monkeypatch.setattr(module.cache_core, 'VAL_COUNT', 5)
    monkeypatch.setattr(module, 'STEPS', 3)
    monkeypatch.setattr(module, 'SAMPLES_SEEN', 12)


def small_fit(monkeypatch, arm='prior_only_residual', callback=None, check_deadline=None, train=None, val=None):
    small_budget(monkeypatch)
    train = cache() if train is None else train
    val = cache('val', 5) if val is None else val
    result = module.fit_frozen_residual(train, val, arm=arm, seed=123,
        callback=callback, check_deadline=check_deadline)
    return result, train, val


def score(data, head, arm):
    return module.score_head(head, arm, data.fused, data.candidate_xy, data.candidate_speed, data.original_logits)


def test_fixed_contract_reuses_unchanged_core_and_denies_runtime():
    core = module.cache_core
    assert hashlib.sha256(Path(core.__file__).read_bytes()).hexdigest() == '4716c22d29b8e306b86f8835bf2a5ec2bf956bbb6859c6b6f001a261865b8de3'
    assert module.CachedStopmixSelectorData is core.CachedStopmixSelectorData
    assert module.validate_pair is core.validate_pair
    assert module.composite_targets is core.composite_targets and module.evaluate_logits is core.evaluate_logits
    assert (module.STEPS, module.BATCH_SIZE, module.SAMPLES_SEEN) == (1540, 4, 6155)
    assert (module.TRAIN_COUNT, module.VAL_COUNT) == (1147, 337)
    assert (module.LEARNING_RATE, module.WEIGHT_DECAY, module.SCORE_WEIGHT, module.MAX_GRAD_NORM) == (1e-4, 1e-4, .1, 5.)
    assert len(TrajectoryLossConfig().to_dict()) == 6
    from portable_e2e.runtime_weight_bundle import SUPPORTED_MODEL_IDS
    assert module.ARTIFACT_ID not in SUPPORTED_MODEL_IDS
    assert module.ARTIFACT_ID != core.ARTIFACT_ID


def test_same_allocated_architecture_initial_state_rng_and_zero_final_only():
    before = torch.get_rng_state().clone()
    a, b = (module.make_head(arm, 123) for arm in module.ARMS)
    assert torch.equal(before, torch.get_rng_state())
    assert tuple(a._modules) == ('0', '1', '2')
    assert type(a[0]) is nn.Linear and (a[0].in_features, a[0].out_features) == (449, 256)
    assert type(a[1]) is nn.ReLU and type(a[2]) is nn.Linear
    assert (a[2].in_features, a[2].out_features) == (256, 1)
    assert sum(p.numel() for p in a.parameters()) == module.HEAD_PARAMETER_COUNT == 115457
    assert all(torch.equal(v, b.state_dict()[key]) for key, v in a.state_dict().items())
    assert a[0].weight.count_nonzero() and a[0].bias.count_nonzero()
    assert not a[2].weight.count_nonzero() and not a[2].bias.count_nonzero()
    assert all(p.device.type == 'cpu' and p.dtype == torch.float32 and p.requires_grad and p.grad is None for p in a.parameters())


@pytest.mark.parametrize('arm', module.ARMS)
def test_only_explicit_cpu_initialization_allocations(monkeypatch, arm):
    original, calls = torch.empty, []
    def allocate(*args, **kwargs):
        calls.append(kwargs)
        assert kwargs['device'] == 'cpu' and kwargs['dtype'] == torch.float32
        return original(*args, **kwargs)
    monkeypatch.setattr(torch, 'empty', allocate)
    module.make_head(arm, 123)
    assert len(calls) == 4


@pytest.mark.parametrize('arm,seed', [('linear_pair_reset', 123), ('candidate_residual', True),
    ('candidate_residual', -1), ('candidate_residual', 2**63)])
def test_bad_arm_seed_rejected(arm, seed):
    with pytest.raises(ContractError): module.make_head(arm, seed)


@pytest.mark.parametrize('arm', module.ARMS)
def test_zero_residual_exact_original_logits_all_candidates_full_and_partial(arm):
    data, head = cache(count=5), module.make_head(arm, 123)
    actual = module.evaluate_head(data, head, arm)['logits_cpu']
    assert torch.equal(actual, data.original_logits)
    assert module._digest_tensors([('logits', actual)]) == module._digest_tensors([('logits', data.original_logits)])
    assert torch.equal(actual.argmax(1), data.original_logits.argmax(1))


@pytest.mark.parametrize('arm', module.ARMS)
@pytest.mark.parametrize('dtype', [torch.float32, torch.float64])
def test_scoring_formula_permutation_and_geometry_information_boundary(arm, dtype):
    data = cache(count=2)
    fused, xy, speed, prior = (v.to(dtype) for v in
        (data.fused, data.candidate_xy, data.candidate_speed, data.original_logits))
    head = module.make_head(arm, 123).to(dtype)
    with torch.no_grad(): head[-1].weight.fill_(.01)
    actual = module.score_head(head, arm, fused, xy, speed, prior)
    geometry = torch.cat((xy.flatten(2)/120., speed/module.PHYSICAL_MAXIMUM_SPEED_MPS), dim=2)
    if arm == 'prior_only_residual': geometry = torch.zeros_like(geometry)
    expected = prior + head(torch.cat((fused[:, None].expand(-1, 12, -1), prior[..., None], geometry), dim=2)).squeeze(-1)
    assert torch.equal(actual, expected)
    order = [11, 3, 8, 0, 10, 4, 6, 2, 9, 1, 7, 5]
    permuted = module.score_head(head, arm, fused, xy[:, order], speed[:, order], prior[:, order])
    eps = torch.finfo(dtype).eps
    assert torch.allclose(permuted, actual[:, order], atol=eps, rtol=4*eps)
    changed = module.score_head(head, arm, fused, xy+10, speed+2, prior)
    assert torch.equal(actual, changed) if arm == 'prior_only_residual' else not torch.equal(actual, changed)


def test_prior_only_arm_is_not_a_common_shift_null_control():
    data, head = cache(count=2), module.make_head('prior_only_residual', 123)
    with torch.no_grad():
        for parameter in head.parameters(): parameter.zero_()
        head[0].weight[0, 256] = 1
        head[0].bias[0] = 10
        head[2].weight[0, 0] = 2
    actual = score(data, head, 'prior_only_residual')
    expected = data.original_logits + 2 * (data.original_logits + 10)
    assert torch.equal(actual, expected)
    assert not torch.allclose(actual.softmax(1), data.original_logits.softmax(1))


@pytest.mark.parametrize('arm', module.ARMS)
def test_inputs_detached_and_zero_last_layer_permits_later_hidden_gradients(arm):
    data = cache(count=2)
    values = [v.clone().requires_grad_() for v in
        (data.fused, data.candidate_xy, data.candidate_speed, data.original_logits)]
    head = module.make_head(arm, 123)
    loss = module.F.cross_entropy(module.score_head(head, arm, *values), torch.tensor([0, 1]))
    loss.backward()
    assert all(v.grad is None for v in values)
    assert all(p.grad is not None and torch.isfinite(p.grad).all() for p in head.parameters())
    assert not head[0].weight.grad.count_nonzero() and not head[0].bias.grad.count_nonzero()
    assert head[2].weight.grad.count_nonzero()
    with torch.no_grad(): head[2].weight.add_(-.01 * head[2].weight.grad)
    head.zero_grad(set_to_none=True)
    module.F.cross_entropy(module.score_head(head, arm, *values), torch.tensor([0, 1])).backward()
    assert head[0].weight.grad[:, :257].count_nonzero()
    if arm == 'prior_only_residual': assert not head[0].weight.grad[:, 257:].count_nonzero()
    else: assert head[0].weight.grad[:, 257:].count_nonzero()
    assert all(v.grad is None for v in values)


@pytest.mark.parametrize('fault', ['none', 'fused_shape', 'xy_k6', 'speed_shape', 'prior_shape', 'dtype',
    'nan_fused', 'nan_xy', 'inf_speed', 'inf_prior'])
def test_scoring_rejects_invalid_inputs_even_when_geometry_is_ablated(fault):
    data = cache(count=2)
    values = [data.fused, data.candidate_xy, data.candidate_speed, data.original_logits]
    if fault == 'none': values[0] = None
    elif fault == 'fused_shape': values[0] = values[0][:, :255]
    elif fault == 'xy_k6': values[1] = values[1][:, :6]
    elif fault == 'speed_shape': values[2] = values[2][:, :, :63]
    elif fault == 'prior_shape': values[3] = values[3][:, :6]
    elif fault == 'dtype': values[2] = values[2].double()
    else:
        index = {'nan_fused': 0, 'nan_xy': 1, 'inf_speed': 2, 'inf_prior': 3}[fault]
        values[index] = torch.full_like(values[index], float('nan') if fault.startswith('nan') else float('inf'))
    with pytest.raises(ContractError): module.score_head(module.make_head('prior_only_residual', 123), 'prior_only_residual', *values)


@pytest.mark.parametrize('fault', ['size', 'episodes', 'sample_overlap', 'episode_overlap', 'parent', 'corpus', 'fingerprint'])
def test_pair_validation_remains_strict_before_optimizer(monkeypatch, fault):
    small_budget(monkeypatch)
    train, val = cache(), cache('val', 5)
    if fault == 'size': train = cache(count=7)
    elif fault == 'episodes': train = replace(train, episode_ids=('one',)*8)
    elif fault == 'sample_overlap': val = replace(val, sample_ids=(train.sample_ids[0],)+val.sample_ids[1:])
    elif fault == 'episode_overlap': val = replace(val, episode_ids=(train.episode_ids[0],)*5)
    elif fault == 'parent': val = replace(val, source_checkpoint_sha256='e'*64)
    elif fault == 'corpus': val = replace(val, corpus_fingerprint_sha256='e'*64)
    else: val = replace(val, dataset_fingerprint_sha256=train.dataset_fingerprint_sha256)
    monkeypatch.setattr(torch.optim, 'AdamW', lambda *a, **k: pytest.fail('optimizer created before validation'))
    with pytest.raises(ContractError): module.fit_frozen_residual(train, val, arm='candidate_residual', seed=123)


@pytest.mark.parametrize('arm', module.ARMS)
def test_fit_reports_fixed_contract_initial_parity_and_immutable_inputs(monkeypatch, arm):
    records = []
    result, train, val = small_fit(monkeypatch, arm, records.append)
    report = result['report']
    assert report['status'] == 'RESEARCH_HEAD_TRAINING_COMPLETE_NOT_PROMOTED'
    assert report['head_parameter_count'] == 115457
    assert report['cache_sha256_before'] == report['cache_sha256_after'] == {'train': train.digest(), 'val': val.digest()}
    assert report['initial_head_sha256'] != report['final_head_sha256']
    assert report['baseline_metrics'] == report['pre_training_metrics']
    assert all(p['byte_equal'] and p['max_abs_logit_difference'] == 0 and p['selected_index_mismatch_count'] == 0
        for p in report['original_logit_parity'].values())
    assert len(records) == len(report['history']) == 3 and report['final_state']['samples_seen'] == 12
    assert result['final_logits_cpu']['val'].shape == (5, 12)
    assert torch.equal(result['oracle_indices_cpu']['train'], module.composite_targets(train))
    for key in ('generator_parameters_trained', 'original_heads_recomputed_or_trained', 'training_data_approved',
            'vehicle_control_approved', 'automatic_promotion', 'runtime_bundle_supported'):
        assert report[key] is False
    assert report['residual_inputs']['geometry_provided'] == (arm == 'candidate_residual')
    assert 'not equal effective information' in report['interpretation']
    records[0]['global_step'] = -1
    assert report['history'][0]['global_step'] == 1


def test_two_arms_have_exact_same_initial_state_teacher_sampling_and_baseline(monkeypatch):
    a, *_ = small_fit(monkeypatch, 'prior_only_residual')
    b, *_ = small_fit(monkeypatch, 'candidate_residual')
    for key in ('initial_head_sha256', 'sampling_order_sha256', 'composite_oracle_sha256',
            'baseline_metrics', 'pre_training_metrics'):
        assert a['report'][key] == b['report'][key]


def test_val_targets_never_optimize_head(monkeypatch):
    a, train, val = small_fit(monkeypatch, 'candidate_residual')
    altered = replace(val, target_xy=val.target_xy+100, target_speed_mps=val.target_speed_mps+20)
    b, *_ = small_fit(monkeypatch, 'candidate_residual', train=train, val=altered)
    assert a['report']['final_head_sha256'] == b['report']['final_head_sha256']
    assert a['report']['history'] == b['report']['history']
    assert a['report']['post_training_metrics']['val'] != b['report']['post_training_metrics']['val']


@pytest.mark.parametrize('fault', ['nonzero_residual', 'signed_zero_byte_difference'])
def test_initial_parity_failure_cannot_reach_optimizer(monkeypatch, fault):
    train = cache()
    if fault == 'nonzero_residual':
        original = module.make_head
        def make(*args):
            head = original(*args)
            with torch.no_grad(): head[2].bias.fill_(.01)
            return head
        monkeypatch.setattr(module, 'make_head', make)
    else:
        logits = train.original_logits.clone(); logits[0, 0] = -0.0
        train = replace(train, original_logits=logits)
    monkeypatch.setattr(torch.optim, 'AdamW', lambda *a, **k: pytest.fail('optimizer created before zero-residual parity'))
    with pytest.raises(ContractError, match='exact cached-original logit parity'):
        small_fit(monkeypatch, train=train)


def test_deadline_checked_before_training(monkeypatch):
    def stop(): raise TimeoutError('fixed deadline')
    with pytest.raises(TimeoutError): small_fit(monkeypatch, check_deadline=stop)


def test_callback_failure_preserves_original_cache(monkeypatch):
    train, val = cache(), cache('val', 5)
    before, records = (train.digest(), val.digest()), []
    def fail(record): records.append(record); raise TimeoutError('owned log writer')
    with pytest.raises(TimeoutError): small_fit(monkeypatch, callback=fail, train=train, val=val)
    assert (train.digest(), val.digest()) == before
    assert len(records) == 1 and records[0]['global_step'] == 1


@pytest.mark.parametrize('field', ['original_logits', 'fused', 'candidate_xy', 'target_speed_mps'])
def test_original_cache_mutation_detected_even_on_failure(monkeypatch, field):
    train = cache()
    def corrupt(record):
        getattr(train, field).flatten()[0] += 1
        raise RuntimeError('interrupted')
    with pytest.raises(ContractError, match='cache mutated'):
        small_fit(monkeypatch, callback=corrupt, train=train)


@pytest.mark.parametrize('fault', ['loss', 'gradient', 'norm', 'parameters', 'optimizer_state'])
def test_nonfinite_training_state_fails_closed(monkeypatch, fault):
    if fault == 'loss':
        original = module.F.cross_entropy
        def corrupt(*a, **k):
            value = original(*a, **k)
            return value * float('nan') if torch.is_grad_enabled() and a[0].requires_grad else value
        monkeypatch.setattr(module.F, 'cross_entropy', corrupt)
    elif fault == 'gradient':
        original = module.make_head
        def make(*a, **k):
            head = original(*a, **k)
            next(head.parameters()).register_hook(lambda gradient: gradient * float('nan'))
            return head
        monkeypatch.setattr(module, 'make_head', make)
    elif fault == 'norm': monkeypatch.setattr(torch.nn.utils, 'clip_grad_norm_', lambda *a, **k: torch.tensor(float('nan')))
    else:
        original = torch.optim.AdamW.step
        def step(optimizer, *a, **k):
            result = original(optimizer, *a, **k)
            parameter = optimizer.param_groups[0]['params'][0]
            with torch.no_grad():
                if fault == 'parameters': parameter.fill_(float('nan'))
                else: optimizer.state[parameter]['exp_avg'].fill_(float('inf'))
            return result
        monkeypatch.setattr(torch.optim.AdamW, 'step', step)
    with pytest.raises(ContractError, match='nonfinite'): small_fit(monkeypatch)


def test_original_hard_composite_teacher_masks_and_six_stop_ties_remain():
    data = cache(count=2)
    xy, speed, valid = data.candidate_xy.clone(), data.candidate_speed.clone(), data.target_valid.clone()
    xy[:, 6:] = 0; speed[:, 6:] = 0; valid[1, 32:] = False
    stationary = replace(data, candidate_xy=xy, candidate_speed=speed, target_xy=torch.zeros_like(data.target_xy),
        target_speed_mps=torch.zeros_like(data.target_speed_mps), target_valid=valid)
    native = trajectory_loss(xy, speed, stationary.original_logits, stationary.target_xy,
        stationary.target_speed_mps, valid, TrajectoryLossConfig(), target_yaw=stationary.target_yaw_rad)
    assert module.composite_targets(stationary).tolist() == [6, 6]
    assert torch.equal(native['oracle_candidate_index'], module.composite_targets(stationary))


def test_identical_causal_inputs_with_conflicting_targets_are_preserved():
    data = cache(count=2)
    values = {name: getattr(data, name).clone() for name in ('fused', 'candidate_xy', 'candidate_speed', 'original_logits', 'target_xy')}
    for name in ('fused', 'candidate_xy', 'candidate_speed', 'original_logits'): values[name][1] = values[name][0]
    values['target_xy'][1] += 20
    conflict = replace(data, **values)
    for arm in module.ARMS:
        head = module.make_head(arm, 123)
        with torch.no_grad(): head[-1].weight.fill_(.01)
        logits = score(conflict, head, arm)
        eps = torch.finfo(logits.dtype).eps
        assert torch.allclose(logits[0], logits[1], atol=eps, rtol=4*eps)
    assert len(conflict) == 2 and conflict.sample_ids == data.sample_ids
    assert not torch.equal(conflict.target_xy[0], conflict.target_xy[1])


def test_complete_budget_keeps_all_partial_batches_and_original_sampling():
    train, val = cache(count=1147), cache('val', 337)
    result = module.fit_frozen_residual(train, val, arm='candidate_residual', seed=123)
    report, history = result['report'], result['report']['history']
    assert len(history) == 1540 and sum(r['batch_samples'] for r in history) == 6155
    assert sum(r['batch_samples'] == 3 for r in history) == 5
    assert report['final_state'] == {'global_step': 1540, 'samples_seen': 6155, 'last_epoch_index': 5}
    expected = []
    for epoch in range(6):
        for indices in module.cache_core._epoch_batches(1147, batch_size=4, seed=123, epoch=epoch):
            if len(expected) == 1540: break
            expected.append({'epoch': epoch, 'indices': list(indices)})
    assert report['sampling_order_sha256'] == hashlib.sha256(json.dumps(expected, separators=(',', ':')).encode()).hexdigest()
    assert not torch.cuda.is_initialized()
