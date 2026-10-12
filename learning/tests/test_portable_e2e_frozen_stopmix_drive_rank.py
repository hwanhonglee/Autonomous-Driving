"""HH_260906 - Test isolated fixed-parent DRIVE pair objectives with synthetic CPU caches only."""

from copy import deepcopy
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path

import pytest
import torch

from portable_e2e import frozen_stopmix_drive_rank as module
from portable_e2e.contract import ContractError
from scripts.e2e import diagnose_stopmix_training_oracles as oracle


@pytest.fixture(autouse=True)
def cpu_single_thread():
    previous = torch.get_num_threads()
    torch.set_num_threads(1)
    yield
    torch.set_num_threads(previous)


def cache(split='train', count=8):
    # HH_260906 - Synthetic tensors do not represent an actual model, road, traffic state or measured improvement.
    generator = torch.Generator().manual_seed(19 if split == 'train' else 20)
    xy = torch.zeros(count, 12, 64, 2)
    xy[..., 0] = torch.arange(1, 65) * .1
    xy[..., 1] = torch.arange(12)[None, :, None] * .1
    return module.CachedStopmixSelectorData(split=split,
        sample_ids=tuple(f'{split}:{i}' for i in range(count)),
        episode_ids=tuple(f'{split}-episode-{i % 3 if split == "train" else 0}' for i in range(count)),
        fused=torch.randn(count, 256, generator=generator)*.1, candidate_xy=xy,
        candidate_speed=torch.ones(count, 12, 64), target_xy=xy[:, 0],
        target_speed_mps=torch.ones(count, 64), target_yaw_rad=torch.zeros(count, 64),
        target_valid=torch.ones(count, 64, dtype=torch.bool), original_logits=torch.randn(count, 12, generator=generator),
        source_checkpoint_sha256='a'*64, corpus_fingerprint_sha256='b'*64,
        dataset_fingerprint_sha256=('c' if split == 'train' else 'd')*64)


def parent(train, val):
    head = module.residual.make_head('candidate_residual', 17)
    with torch.no_grad():
        head[-1].weight.fill_(.001)
        head[-1].bias.fill_(.02)
    logits = {d.split: module.residual.evaluate_head(d, head, 'candidate_residual')['logits_cpu'] for d in (train, val)}
    state = {k: v.detach().clone() for k, v in head.state_dict().items()}
    return state, logits


def budget(monkeypatch):
    # HH_260906 - Only fixtures shorten the budget; no public API can change the fixed study exposure or coefficients.
    monkeypatch.setattr(module.cache_core, 'TRAIN_COUNT', 8)
    monkeypatch.setattr(module.cache_core, 'VAL_COUNT', 5)
    monkeypatch.setattr(module, 'STEPS', 3)
    monkeypatch.setattr(module, 'SAMPLES_SEEN', 12)


def run_fit(monkeypatch, arm='ce_continue', *, callback=None, check_deadline=None, train=None, val=None,
        state=None, logits=None, projected=None, proof=None):
    budget(monkeypatch)
    train = cache() if train is None else train
    val = cache('val', 5) if val is None else val
    if state is None: state, logits = parent(train, val)
    if projected is None: projected, proof = oracle.projected_costs(module.loss_module)
    result = module.fit_frozen_drive_rank(train, val, parent_head_state_dict=state, parent_logits_cpu=logits,
        projected_costs_fn=projected, projection_proof=proof, arm=arm, seed=123,
        callback=callback, check_deadline=check_deadline)
    return result, train, val, state, logits


def test_fixed_scope_and_unmodified_dependencies():
    assert module.ARMS == ('ce_continue', 'drive_pair_continue')
    assert (module.STEPS, module.BATCH_SIZE, module.SAMPLES_SEEN) == (1540, 4, 6155)
    assert (module.TRAIN_COUNT, module.VAL_COUNT, module.HEAD_PARAMETER_COUNT) == (1147, 337, 115457)
    assert (module.LEARNING_RATE, module.WEIGHT_DECAY, module.SCORE_WEIGHT, module.MAX_GRAD_NORM) == (1e-4, 1e-4, .1, 5.)
    for source, expected in ((module.cache_core, '4716c22d29b8e306b86f8835bf2a5ec2bf956bbb6859c6b6f001a261865b8de3'),
            (module.residual, 'd4bf42fc9f4db930f31ef7b8ccd79900249687642f47bf525dedb240207550b7')):
        assert hashlib.sha256(Path(source.__file__).read_bytes()).hexdigest() == expected
    from portable_e2e.runtime_weight_bundle import SUPPORTED_MODEL_IDS
    assert module.ARTIFACT_ID not in SUPPORTED_MODEL_IDS


@pytest.mark.parametrize('dtype', [torch.float32, torch.float64])
def test_pair_formula_independent_scalar_reference(dtype):
    logits = torch.tensor([[2., -1., 4., 1., 3., 0., 20., 30., 40., 50., 60., 70.]], dtype=dtype)
    costs = torch.tensor([[4., 1., 5., 0., 3., 2., 0., 0., 0., 0., 0., 0.]], dtype=dtype)
    pairs = [(i, j) for i in range(6) for j in range(i+1, 6)]
    denominator = sum(abs(float(costs[0, i]-costs[0, j])) for i, j in pairs)
    expected = 0.
    for i, j in pairs:
        dc = float(costs[0, i]-costs[0, j])
        value = math.copysign(1., dc)*float(logits[0, i]-logits[0, j])
        softplus = max(value, 0.)+math.log1p(math.exp(-abs(value)))
        expected += abs(dc)/denominator*softplus
    actual = module.drive_pair_rank_loss(logits, costs)
    assert float(actual) == pytest.approx(expected, rel=4*torch.finfo(dtype).eps, abs=torch.finfo(dtype).eps)


def test_pair_gradient_direction_detached_costs_and_no_direct_stop_gradient():
    logits = torch.zeros(2, 12, requires_grad=True)
    costs = torch.arange(12.).repeat(2, 1).requires_grad_()
    result = module.drive_pair_rank_loss(logits, costs)
    result.backward()
    assert costs.grad is None
    assert logits.grad[:, 0].lt(0).all() and logits.grad[:, 5].gt(0).all()
    assert logits.grad[:, :6].abs().sum() > 0 and not logits.grad[:, 6:].count_nonzero()
    assert torch.isfinite(logits.grad).all()


def test_exact_ties_zero_and_all_tied_rows_are_retained_in_mean():
    logits = torch.arange(24.).reshape(2, 12).requires_grad_()
    costs = torch.arange(12.).repeat(2, 1)
    costs[0] = 3
    actual = module.drive_pair_rank_loss(logits, costs)
    expected = module.drive_pair_rank_loss(logits[1:], costs[1:])/2
    assert torch.equal(actual, expected)
    actual.backward()
    assert not logits.grad[0].count_nonzero()
    logits2 = torch.zeros(1, 12, requires_grad=True)
    zero = module.drive_pair_rank_loss(logits2, torch.ones_like(logits2))
    zero.backward()
    assert float(zero) == 0 and not logits2.grad.count_nonzero()


def test_pair_permutation_positive_cost_scale_shift_and_stop_values_invariant():
    generator = torch.Generator().manual_seed(123)
    logits = torch.randn(3, 12, generator=generator, dtype=torch.float64)
    costs = torch.arange(12., dtype=torch.float64).repeat(3, 1)
    actual = module.drive_pair_rank_loss(logits, costs)
    assert torch.equal(actual, module.drive_pair_rank_loss(logits, costs*8+16))
    other_logits, other_costs = logits.clone(), costs.clone()
    other_logits[:, 6:] += 100; other_costs[:, 6:] += 200
    assert torch.equal(actual, module.drive_pair_rank_loss(other_logits, other_costs))
    order = [4, 1, 5, 0, 3, 2, 6, 7, 8, 9, 10, 11]
    assert torch.allclose(actual, module.drive_pair_rank_loss(logits[:, order], costs[:, order]), atol=1e-15, rtol=1e-15)


@pytest.mark.parametrize('fault', ['k6', 'count', 'dtype', 'nan_logits', 'inf_costs', 'overflow'])
def test_pair_invalid_or_overflow_inputs_rejected(fault):
    logits, costs = torch.zeros(2, 12), torch.arange(12.).repeat(2, 1)
    if fault == 'k6': logits = logits[:, :6]
    elif fault == 'count': costs = costs[:1]
    elif fault == 'dtype': costs = costs.double()
    elif fault == 'nan_logits': logits[0, 11] = float('nan')
    elif fault == 'inf_costs': costs[0, 11] = float('inf')
    else: costs[0, :3] = torch.finfo(torch.float32).max; costs[0, 3:6] = -torch.finfo(torch.float32).max
    with pytest.raises(ContractError): module.drive_pair_rank_loss(logits, costs)


def test_parent_copy_is_exact_isolated_and_caller_rng_unchanged():
    train, val = cache(), cache('val', 5)
    state, _ = parent(train, val)
    before = torch.get_rng_state().clone()
    head = module.make_parent_head(state, 123)
    assert torch.equal(before, torch.get_rng_state())
    assert module._digest_tensors(head.state_dict().items()) == module._digest_tensors(state.items())
    assert all(head.state_dict()[k].data_ptr() != v.data_ptr() for k, v in state.items())
    assert sum(p.numel() for p in head.parameters()) == 115457


@pytest.mark.parametrize('fault', ['key', 'shape', 'dtype', 'nan', 'grad', 'seed'])
def test_bad_parent_state_rejected(fault):
    state, _ = parent(cache(), cache('val', 5))
    if fault == 'key': state['other'] = torch.zeros(1)
    elif fault == 'shape': state['0.bias'] = torch.zeros(255)
    elif fault == 'dtype': state['0.bias'] = state['0.bias'].double()
    elif fault == 'nan': state['0.bias'][0] = float('nan')
    elif fault == 'grad': state['0.bias'].requires_grad_()
    with pytest.raises(ContractError): module.make_parent_head(state, True if fault == 'seed' else 123)


@pytest.mark.parametrize('fault', ['source', 'boundary', 'hash', 'key', 'callable'])
def test_projection_proof_rejected_before_optimizer(monkeypatch, fault):
    projected, proof = oracle.projected_costs(module.loss_module)
    if fault == 'source': proof['original_loss_sha256'] = 'a'*64
    elif fault == 'boundary': proof['prefix_first_line'] = True
    elif fault == 'hash': proof['derived_ast_sha256'] = 'bad'
    elif fault == 'key': proof['override'] = True
    else: projected = False
    monkeypatch.setattr(torch.optim, 'AdamW', lambda *a, **k: pytest.fail('optimizer before proof'))
    with pytest.raises(ContractError): run_fit(monkeypatch, projected=projected, proof=proof)


def test_projection_native_oracle_regression_masks_and_costs_exact():
    data = cache(count=5)
    valid = data.target_valid.clone(); valid[1, 31:] = False
    data = replace(data, target_valid=valid)
    projected, _ = oracle.projected_costs(module.loss_module)
    actual, targets = module.projected_targets(data, projected)
    for start in range(0, 5, 4):
        index = slice(start, start+4)
        args = (data.candidate_xy[index], data.candidate_speed[index], data.original_logits[index], data.target_xy[index],
            data.target_speed_mps[index], data.target_valid[index], module.loss_module.TrajectoryLossConfig())
        expected = projected(*args, target_yaw=data.target_yaw_rad[index])
        native = module.loss_module.trajectory_loss(*args, target_yaw=data.target_yaw_rad[index])
        assert torch.equal(actual[index], expected) and torch.equal(targets[index], native['oracle_candidate_index'])
    assert not actual.requires_grad and not targets.requires_grad


@pytest.mark.parametrize('fault', ['shape', 'nonfinite', 'oracle', 'regression'])
def test_projection_consistency_rejects_wrong_native_result(fault):
    projected, _ = oracle.projected_costs(module.loss_module)
    def incorrect(*a, **k):
        values = projected(*a, **k)
        if fault == 'shape': return values[:, :6]
        if fault == 'nonfinite': return values*float('nan')
        if fault == 'oracle': values[:, 11] = -100
        else: values += 1
        return values
    with pytest.raises(ContractError): module.projected_targets(cache(count=5), incorrect)


@pytest.mark.parametrize('arm', module.ARMS)
def test_small_fit_reports_parent_parity_and_immutable_all_inputs(monkeypatch, arm):
    records = []
    result, train, val, state, logits = run_fit(monkeypatch, arm, callback=records.append)
    report = result['report']
    assert report['status'] == 'RESEARCH_HEAD_TRAINING_COMPLETE_NOT_PROMOTED'
    assert report['initial_head_sha256'] == report['parent_head_sha256_before'] == report['parent_head_sha256_after']
    assert report['parent_logits_sha256_before'] == report['parent_logits_sha256_after'] == module._digest_tensors(logits.items())
    assert report['cache_sha256_before'] == report['cache_sha256_after'] == {'train': train.digest(), 'val': val.digest()}
    assert all(v['byte_equal'] and v['max_abs_logit_difference'] == 0 for v in report['parent_logit_parity'].values())
    assert report['pre_training_metrics'] == report['baseline_metrics']
    assert len(records) == len(report['history']) == 3 and report['final_state']['samples_seen'] == 12
    assert report['initial_head_sha256'] != report['final_head_sha256']
    assert len(report['loss_config']) == 6 and report['optimizer_resumed'] is False
    assert report['selector_objective']['drive_pair_weight'] == (arm == 'drive_pair_continue')
    assert all(not report[k] for k in ('generator_parameters_trained', 'original_heads_recomputed_or_trained',
        'training_data_approved', 'vehicle_control_approved', 'automatic_promotion', 'runtime_bundle_supported'))
    assert result['final_logits_cpu']['train'].shape == (8, 12)
    assert torch.equal(result['oracle_indices_cpu']['train'], module.cache_core.composite_targets(train))
    records[0]['global_step'] = -1
    assert report['history'][0]['global_step'] == 1


def test_same_parent_sampling_teacher_but_different_objective(monkeypatch):
    a, *_ = run_fit(monkeypatch, 'ce_continue')
    b, *_ = run_fit(monkeypatch, 'drive_pair_continue')
    for name in ('initial_head_sha256', 'baseline_metrics', 'sampling_order_sha256', 'composite_oracle_sha256', 'projected_cost_sha256'):
        assert a['report'][name] == b['report'][name]
    assert a['report']['final_head_sha256'] != b['report']['final_head_sha256']


def test_zero_auxiliary_arm_matches_manual_native_ce_updates_bit_exact(monkeypatch):
    result, train, val, state, _ = run_fit(monkeypatch, 'ce_continue')
    head = module.make_parent_head(state, 123)
    optimizer = torch.optim.AdamW(head.parameters(), lr=1e-4, weight_decay=1e-4)
    targets = module.cache_core.composite_targets(train)
    step = 0
    for epoch in range(2):
        for indices in module.cache_core._epoch_batches(8, batch_size=4, seed=123, epoch=epoch):
            if step == 3: break
            index = torch.tensor(indices)
            scores = module.residual.score_head(head, 'candidate_residual', train.fused[index], train.candidate_xy[index],
                train.candidate_speed[index], train.original_logits[index])
            loss = .1*module.F.cross_entropy(scores, targets[index])
            optimizer.zero_grad(set_to_none=True); loss.backward()
            torch.nn.utils.clip_grad_norm_(head.parameters(), 5.); optimizer.step(); step += 1
    assert module._digest_tensors(head.state_dict().items()) == result['report']['final_head_sha256']


def test_val_targets_do_not_change_optimized_head(monkeypatch):
    a, train, val, state, logits = run_fit(monkeypatch, 'drive_pair_continue')
    changed = replace(val, target_xy=val.target_xy+50, target_speed_mps=val.target_speed_mps+5)
    b, *_ = run_fit(monkeypatch, 'drive_pair_continue', train=train, val=changed, state=state, logits=logits)
    assert a['report']['final_head_sha256'] == b['report']['final_head_sha256']
    assert a['report']['history'] == b['report']['history']
    assert a['report']['post_training_metrics']['val'] != b['report']['post_training_metrics']['val']


@pytest.mark.parametrize('fault', ['saved_logits', 'logit_dtype', 'missing_split', 'parent_dtype', 'split_leakage'])
def test_invalid_parent_or_split_cannot_reach_optimizer(monkeypatch, fault):
    train, val = cache(), cache('val', 5)
    state, logits = parent(train, val)
    if fault == 'saved_logits': logits['val'][0, 0] += .01
    elif fault == 'logit_dtype': logits['train'] = logits['train'].double()
    elif fault == 'missing_split': del logits['train']
    elif fault == 'parent_dtype': state['0.weight'] = state['0.weight'].double()
    else: val = replace(val, sample_ids=(train.sample_ids[0],)+val.sample_ids[1:])
    monkeypatch.setattr(torch.optim, 'AdamW', lambda *a, **k: pytest.fail('optimizer before parent/split proof'))
    with pytest.raises(ContractError): run_fit(monkeypatch, train=train, val=val, state=state, logits=logits)


@pytest.mark.parametrize('fault', ['parent_state', 'parent_logits', 'cache', 'proof'])
def test_mutation_detected_during_callback_failure(monkeypatch, fault):
    train, val = cache(), cache('val', 5)
    state, logits = parent(train, val)
    projected, proof = oracle.projected_costs(module.loss_module)
    def corrupt(record):
        if fault == 'parent_state': state['0.bias'][0] += 1
        elif fault == 'parent_logits': logits['train'][0, 0] += 1
        elif fault == 'cache': train.original_logits[0, 0] += 1
        else: proof['derived_ast_sha256'] = 'a'*64
        raise TimeoutError('owned callback failure')
    with pytest.raises(ContractError, match='mutated'):
        run_fit(monkeypatch, callback=corrupt, train=train, val=val, state=state, logits=logits, projected=projected, proof=proof)


def test_deadline_before_fit_and_after_partial_history(monkeypatch):
    def stop(): raise TimeoutError('deadline')
    with pytest.raises(TimeoutError): run_fit(monkeypatch, check_deadline=stop)
    records = []
    def fail(record): records.append(record); raise TimeoutError('writer deadline')
    with pytest.raises(TimeoutError): run_fit(monkeypatch, callback=fail)
    assert len(records) == 1 and records[0]['global_step'] == 1


@pytest.mark.parametrize('fault', ['loss', 'gradient', 'parameters', 'optimizer_state'])
def test_nonfinite_optimizer_path_fails_closed(monkeypatch, fault):
    if fault == 'loss':
        original = module.F.cross_entropy
        def bad(*a, **k):
            result = original(*a, **k)
            return result*float('nan') if torch.is_grad_enabled() and a[0].requires_grad else result
        monkeypatch.setattr(module.F, 'cross_entropy', bad)
    elif fault == 'gradient':
        original = module.make_parent_head
        def head(*a, **k):
            value = original(*a, **k); next(value.parameters()).register_hook(lambda g: g*float('nan')); return value
        monkeypatch.setattr(module, 'make_parent_head', head)
    else:
        original = torch.optim.AdamW.step
        def step(optimizer, *a, **k):
            result = original(optimizer, *a, **k); p = optimizer.param_groups[0]['params'][0]
            with torch.no_grad():
                if fault == 'parameters': p.fill_(float('nan'))
                else: optimizer.state[p]['exp_avg'].fill_(float('inf'))
            return result
        monkeypatch.setattr(torch.optim.AdamW, 'step', step)
    with pytest.raises(ContractError, match='nonfinite'): run_fit(monkeypatch)


def test_complete_fixed_budget_exposures_and_original_order():
    train, val = cache(count=1147), cache('val', 337)
    state, logits = parent(train, val)
    projected, proof = oracle.projected_costs(module.loss_module)
    result = module.fit_frozen_drive_rank(train, val, parent_head_state_dict=state, parent_logits_cpu=logits,
        projected_costs_fn=projected, projection_proof=proof, arm='drive_pair_continue', seed=123)
    report = result['report']; history = report['history']
    assert len(history) == 1540 and sum(r['batch_samples'] for r in history) == 6155
    assert sum(r['batch_samples'] == 3 for r in history) == 5
    expected = []
    for epoch in range(6):
        for indices in module.cache_core._epoch_batches(1147, batch_size=4, seed=123, epoch=epoch):
            if len(expected) == 1540: break
            expected.append({'epoch': epoch, 'indices': list(indices)})
    assert report['sampling_order_sha256'] == hashlib.sha256(json.dumps(expected, separators=(',', ':')).encode()).hexdigest()
    assert report['final_state'] == {'global_step': 1540, 'samples_seen': 6155, 'last_epoch_index': 5}
    assert not torch.cuda.is_initialized()
