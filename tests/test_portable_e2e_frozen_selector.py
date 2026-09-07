"""HH_260906 - Verify scorer-only optimization, immutable caches, paired scope, and reproducibility."""

from dataclasses import replace

import pytest
import torch
from torch import nn

from portable_e2e.contract import ContractError
from portable_e2e import frozen_selector as module
from portable_e2e.losses import TrajectoryLossConfig, trajectory_loss


@pytest.fixture(autouse=True)
def single_thread():
    previous = torch.get_num_threads()
    torch.set_num_threads(1)
    yield
    torch.set_num_threads(previous)


def original_head():
    with torch.random.fork_rng(devices=[]):
        torch.random.default_generator.manual_seed(10)
        head = nn.Linear(256, 6)
    with torch.no_grad():
        head.weight.zero_()
        head.bias.zero_()
        head.bias[2] = 0.5
    return head


def cache(split='train', count=8, *, head=None):
    generator = torch.Generator().manual_seed(7 if split == 'train' else 8)
    fused = torch.randn(count, 256, generator=generator) * 0.01
    fused[:, 0] = 1
    xy = torch.zeros(count, 6, 64, 2)
    xy[..., 0] = torch.arange(1, 65) * 0.1
    xy[..., 1] = torch.arange(6)[None, :, None]
    speed = torch.ones(count, 6, 64)
    speed[:, 0] = 20
    head = original_head() if head is None else head
    with torch.no_grad():
        logits = head(fused)
    return module.CachedSelectorData(split=split,
        sample_ids=tuple(f'{split}:{index}' for index in range(count)),
        episode_ids=(f'{split}-episode',) * count,
        fused=fused, candidate_xy=xy, candidate_speed=speed,
        target_xy=xy[:, 0].clone(), target_speed_mps=torch.ones(count, 64),
        target_yaw_rad=torch.zeros(count, 64), target_valid=torch.ones(count, 64, dtype=torch.bool),
        source_checkpoint_sha256='a' * 64, corpus_fingerprint_sha256='b' * 64,
        dataset_fingerprint_sha256=('c' if split == 'train' else 'd') * 64, original_logits=logits)


def test_cache_owns_detached_clones_and_preserves_original_masks():
    data = cache()
    fused = data.fused.clone().requires_grad_(True)
    xy = data.candidate_xy.clone().requires_grad_(True)
    valid = data.target_valid.clone()
    valid[0, 30:] = False
    detached = replace(data, fused=fused, candidate_xy=xy, target_valid=valid)
    before = detached.digest()
    with torch.no_grad():
        fused[0, 0] = 999
        xy[0, 0, 0] = 999
    valid[0] = True
    assert detached.digest() == before
    assert not detached.fused.requires_grad and not detached.candidate_xy.requires_grad
    assert detached.fused.data_ptr() != fused.data_ptr()
    assert detached.target_valid[0].sum() == 30


@pytest.mark.parametrize('fault', ['test', 'shape', 'dtype', 'nan_context', 'inf_speed', 'nan_yaw',
    'bad_logits', 'integer_mask', 'empty_mask', 'nonprefix', 'duplicate_ids', 'unaligned_episodes', 'bad_hash'])
def test_bad_cache_rejected(fault):
    data = cache()
    if fault == 'test': kwargs = {'split': 'test'}
    elif fault == 'shape': kwargs = {'candidate_xy': data.candidate_xy[:, :5]}
    elif fault == 'dtype': kwargs = {'fused': data.fused.double()}
    elif fault == 'nan_context':
        value = data.fused.clone(); value[0, 0] = float('nan'); kwargs = {'fused': value}
    elif fault == 'inf_speed':
        value = data.candidate_speed.clone(); value[0, 0, 0] = float('inf'); kwargs = {'candidate_speed': value}
    elif fault == 'nan_yaw':
        value = data.target_yaw_rad.clone(); value[0, 0] = float('nan'); kwargs = {'target_yaw_rad': value}
    elif fault == 'bad_logits': kwargs = {'original_logits': torch.zeros(8, 5)}
    elif fault == 'integer_mask': kwargs = {'target_valid': data.target_valid.long()}
    elif fault in ('empty_mask', 'nonprefix'):
        value = data.target_valid.clone(); value[0] = False if fault == 'empty_mask' else True
        if fault == 'nonprefix': value[0, 1] = False
        kwargs = {'target_valid': value}
    elif fault == 'duplicate_ids': kwargs = {'sample_ids': ('same',) * 8}
    elif fault == 'unaligned_episodes': kwargs = {'episode_ids': ('one',)}
    else: kwargs = {'source_checkpoint_sha256': 'untrusted'}
    with pytest.raises(ContractError):
        replace(data, **kwargs)


@pytest.mark.parametrize('arm', module.ARMS)
def test_head_initialization_does_not_pollute_rng_or_original_head(arm):
    original = original_head()
    original.weight.grad = torch.ones_like(original.weight)
    before, rng = module._head_digest(original), torch.get_rng_state().clone()
    head = module.make_head(original, arm, 123)
    assert module._head_digest(original) == before
    assert torch.equal(torch.get_rng_state(), rng)
    assert all(parameter.grad is None for parameter in head.parameters())
    assert all(parameter.requires_grad for parameter in head.parameters())
    assert set(parameter.data_ptr() for parameter in head.parameters()).isdisjoint(
        parameter.data_ptr() for parameter in original.parameters())
    if arm == 'linear_continue':
        assert all(torch.equal(value, head.state_dict()[name]) for name, value in original.state_dict().items())
    assert all(torch.equal(value, module.make_head(original, arm, 123).state_dict()[name])
        for name, value in head.state_dict().items())


@pytest.mark.parametrize('arm', module.ARMS)
def test_only_new_head_gets_gradients(arm):
    data = cache(count=2)
    fused = data.fused.clone().requires_grad_(True)
    xy = data.candidate_xy.clone().requires_grad_(True)
    speed = data.candidate_speed.clone().requires_grad_(True)
    head = module.make_head(original_head(), arm, 12)
    module.score_head(head, arm, fused, xy, speed).sum().backward()
    assert fused.grad is xy.grad is speed.grad is None
    assert all(parameter.grad is not None for parameter in head.parameters())


def test_candidate_head_matches_e_normalization_and_is_permutation_equivariant():
    data = cache(count=2)
    head = module.make_head(original_head(), 'candidate_reset', 123)
    geometry = torch.cat((data.candidate_xy.flatten(2) / 120.0,
        data.candidate_speed / module.PHYSICAL_MAXIMUM_SPEED_MPS), dim=2)
    expected = head(torch.cat((data.fused[:, None].expand(-1, 6, -1), geometry), dim=2)).squeeze(-1)
    actual = module.score_head(head, 'candidate_reset', data.fused, data.candidate_xy, data.candidate_speed)
    assert torch.equal(actual, expected)
    order = [2, 5, 1, 3, 4, 0]
    permuted = module.score_head(head, 'candidate_reset', data.fused,
        data.candidate_xy[:, order], data.candidate_speed[:, order])
    assert torch.equal(permuted, actual[:, order])


def test_targets_use_actual_composite_loss_not_ade_and_keep_yaw_mask(monkeypatch):
    data = cache(count=2)
    calls = []

    def loss(*args, **kwargs):
        calls.append((args, kwargs))
        return trajectory_loss(*args, **kwargs)

    monkeypatch.setattr(module, 'trajectory_loss', loss)
    targets = module._targets(data)
    assert targets.tolist() == [1, 1]
    assert torch.norm(data.candidate_xy - data.target_xy[:, None], dim=-1).mean(-1).argmin(-1).tolist() == [0, 0]
    assert calls[0][0][5] is data.target_valid
    assert calls[0][1]['target_yaw'] is data.target_yaw_rad
    assert not targets.requires_grad


def test_evaluation_matches_existing_metrics_and_horizon_denominators():
    data = cache(count=2)
    valid = data.target_valid.clone(); valid[0, 10:] = False
    data = replace(data, target_valid=valid)
    result = module.evaluate_logits(data, data.original_logits)
    direct = trajectory_loss(data.candidate_xy, data.candidate_speed, data.original_logits,
        data.target_xy, data.target_speed_mps, data.target_valid, TrajectoryLossConfig(), target_yaw=data.target_yaw_rad)
    assert result['metrics']['selected_ade_m'] == pytest.approx(float(direct['selected_ade_m']))
    assert result['metric_counts']['ade_1p0s_m'] == 2
    assert result['metric_counts']['ade_3p0s_m'] == result['metric_counts']['ade_6p4s_m'] == 1
    assert result['selected_histogram'] == [0, 0, 2, 0, 0, 0]


@pytest.mark.parametrize('fault', ['samples', 'episodes', 'checkpoint', 'corpus', 'fingerprint', 'swapped'])
def test_training_validation_leakage_or_provenance_mismatch_rejected(fault):
    train, val = cache(), cache('val', 4)
    if fault == 'samples': val = replace(val, sample_ids=train.sample_ids[:4])
    elif fault == 'episodes': val = replace(val, episode_ids=train.episode_ids[:4])
    elif fault == 'checkpoint': val = replace(val, source_checkpoint_sha256='e' * 64)
    elif fault == 'corpus': val = replace(val, corpus_fingerprint_sha256='e' * 64)
    elif fault == 'fingerprint': val = replace(val, dataset_fingerprint_sha256=train.dataset_fingerprint_sha256)
    else: train, val = val, train
    with pytest.raises(ContractError):
        module.fit_frozen_selector(train, val, original_head=original_head(), arm='linear_continue', seed=12)


@pytest.mark.parametrize('arm', module.ARMS)
def test_full_fixed_budget_is_reproducible_head_only_and_leak_free(arm):
    train, val, original = cache(), cache('val', 4), original_head()
    cache_before = (train.digest(), val.digest())
    old_head = module._head_digest(original)
    rng = torch.get_rng_state().clone()
    callbacks = []

    def callback(record):
        callbacks.append(record['global_step'])
        record['samples_seen'] = -99

    result = module.fit_frozen_selector(train, val, original_head=original, arm=arm, seed=123, callback=callback)
    report = result['report']
    assert callbacks == list(range(1, 1541))
    assert len(report['history']) == report['final_state']['global_step'] == 1540
    assert report['final_state']['samples_seen'] == 6160
    assert report['cache_sha256_before'] == report['cache_sha256_after']
    assert report['original_head_sha256_before'] == report['original_head_sha256_after'] == old_head
    assert (train.digest(), val.digest()) == cache_before
    assert module._head_digest(original) == old_head
    assert torch.equal(torch.get_rng_state(), rng)
    assert report['artifact_id'] != 'portable_e2e.pytorch_checkpoint.v1'
    assert 'checkpoint_id' not in report
    assert report['vehicle_control_approved'] is report['automatic_promotion'] is False
    assert report['original_logit_parity']['train']['max_abs_logit_difference'] == 0
    assert report['baseline_metrics']['train']['selected_histogram'] == [0, 0, 8, 0, 0, 0]
    for epoch in range(770):
        indices = [index for row in report['history'] if row['epoch'] == epoch for index in row['sample_indices']]
        assert sorted(indices) == list(range(8))
    assert report['train_sample_ids'] == list(train.sample_ids)
    assert set(report['train_sample_ids']).isdisjoint(report['val_sample_ids'])
    assert all(value.grad is None for value in (train.fused, train.candidate_xy, train.candidate_speed))
    assert set(result['final_logits_cpu']) == {'train', 'val'}
    again = module.fit_frozen_selector(train, val, original_head=original, arm=arm, seed=123)
    assert report['sampling_order_sha256'] == again['report']['sampling_order_sha256']
    assert report['history'] == again['report']['history']
    assert all(torch.equal(value, again['head_state_dict'][name]) for name, value in result['head_state_dict'].items())


def test_same_batch_order_is_used_for_every_arm(monkeypatch):
    # HH_260906 - Shorten only this unit-test loop; production exposes no step-budget override.
    monkeypatch.setattr(module, 'STEPS', 6)
    results = [module.fit_frozen_selector(cache(count=5), cache('val', 4),
        original_head=original_head(), arm=arm, seed=20)['report'] for arm in module.ARMS]
    assert len({report['sampling_order_sha256'] for report in results}) == 1
    assert [row['batch_samples'] for row in results[0]['history']] == [4, 1, 4, 1, 4, 1]


def test_changed_validation_labels_cannot_change_training_targets_or_final_head(monkeypatch):
    monkeypatch.setattr(module, 'STEPS', 8)
    train, val, original = cache(), cache('val', 4), original_head()
    changed = replace(val, target_xy=val.target_xy + 100, target_speed_mps=val.target_speed_mps + 10)
    first = module.fit_frozen_selector(train, val, original_head=original, arm='candidate_reset', seed=12)
    second = module.fit_frozen_selector(train, changed, original_head=original, arm='candidate_reset', seed=12)
    assert first['report']['train_composite_targets_sha256'] == second['report']['train_composite_targets_sha256']
    assert first['report']['history'] == second['report']['history']
    assert first['report']['post_training_metrics']['val'] != second['report']['post_training_metrics']['val']
    assert all(torch.equal(value, second['head_state_dict'][name]) for name, value in first['head_state_dict'].items())


def test_measured_original_logits_are_not_silently_replaced(monkeypatch):
    monkeypatch.setattr(module, 'STEPS', 1)
    train, val = cache(), cache('val', 4)
    different = train.original_logits.clone()
    different[:, 1] = 2
    train = replace(train, original_logits=different)
    result = module.fit_frozen_selector(train, val, original_head=original_head(), arm='linear_reset', seed=12)
    assert result['report']['baseline_metrics']['train']['selected_histogram'] == [0, 8, 0, 0, 0, 0]
    assert result['report']['original_logit_parity']['train']['selected_index_mismatch_count'] == 8
    assert torch.equal(result['baseline_logits_cpu']['train'], different)


def test_callback_cancellation_propagates_without_changing_parent_or_cache():
    train, val, original = cache(), cache('val', 4), original_head()
    before = (train.digest(), val.digest(), module._head_digest(original))

    def stop(record):
        raise TimeoutError('owned experiment deadline')

    with pytest.raises(TimeoutError, match='deadline'):
        module.fit_frozen_selector(train, val, original_head=original, arm='linear_continue', seed=1, callback=stop)
    assert (train.digest(), val.digest(), module._head_digest(original)) == before


def test_in_place_cache_mutation_is_detected_even_on_callback_failure():
    train, val = cache(), cache('val', 4)

    def corrupt(record):
        train.fused[0, 0] += 1
        raise TimeoutError('stop')

    with pytest.raises(ContractError, match='cache mutated'):
        module.fit_frozen_selector(train, val, original_head=original_head(), arm='linear_continue', seed=1, callback=corrupt)


@pytest.mark.parametrize('device', ['cuda:1', 'cuda', 'mps'])
def test_other_device_rejected_before_training(device):
    with pytest.raises(ContractError, match='device zero'):
        module.fit_frozen_selector(cache(), cache('val', 4), original_head=original_head(), arm='linear_reset', seed=1, device=device)
