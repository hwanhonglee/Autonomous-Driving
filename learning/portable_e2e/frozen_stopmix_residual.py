"""HH_260906 - Compare information-ablated residual scorers on unchanged K12 research caches."""

from __future__ import annotations

import copy
import hashlib
import json
from typing import Callable

import torch
from torch import Tensor, nn
from torch.nn import functional as F

from . import frozen_stopmix_selector as cache_core
from .losses import TrajectoryLossConfig
from .model import PHYSICAL_MAXIMUM_SPEED_MPS


ARTIFACT_ID = 'portable_e2e.frozen_stopmix_residual_research_head.v1'
ARMS = ('prior_only_residual', 'candidate_residual')
STEPS, BATCH_SIZE, SAMPLES_SEEN = 1540, 4, 6155
TRAIN_COUNT, VAL_COUNT = 1147, 337
LEARNING_RATE, WEIGHT_DECAY, SCORE_WEIGHT, MAX_GRAD_NORM = 1.0e-4, 1.0e-4, 0.1, 5.0
HEAD_PARAMETER_COUNT = 115457
CachedStopmixSelectorData = cache_core.CachedStopmixSelectorData
validate_pair = cache_core.validate_pair
composite_targets = cache_core.composite_targets
evaluate_logits = cache_core.evaluate_logits
_require, _check = cache_core._require, cache_core._check
_device, _digest_tensors = cache_core._device, cache_core._digest_tensors


def make_head(arm, seed):
    _require(arm in ARMS and type(seed) is int and 0 <= seed < 2 ** 63,
        'unknown residual arm or invalid initialization seed')
    _require(torch.get_default_dtype() == torch.float32, 'fixed float32 initialization is required')
    # HH_260906 - Both arms allocate identical CPU parameters without changing caller RNG or touching another GPU.
    with torch.random.fork_rng(devices=[]):
        torch.random.default_generator.manual_seed(seed)
        head = nn.Sequential(cache_core._cpu_linear(449, 256), nn.ReLU(inplace=True),
            cache_core._cpu_linear(256, 1))
        nn.init.zeros_(head[-1].weight)
        nn.init.zeros_(head[-1].bias)
    return head


def score_head(head, arm, fused, candidate_xy, candidate_speed, original_logits):
    values = (fused, candidate_xy, candidate_speed, original_logits)
    _require(arm in ARMS and all(isinstance(v, Tensor) for v in values),
        'a known arm and tensor scoring inputs are required')
    _require(fused.ndim == 2 and fused.shape[1] == 256 and len(fused) > 0
        and tuple(candidate_xy.shape) == (len(fused), 12, 64, 2)
        and tuple(candidate_speed.shape) == (len(fused), 12, 64)
        and tuple(original_logits.shape) == (len(fused), 12), 'fixed K12 residual input shapes required')
    _require(fused.dtype in (torch.float32, torch.float64)
        and all(v.dtype == fused.dtype and v.device == fused.device and bool(torch.isfinite(v).all()) for v in values),
        'finite same-device and same-dtype residual inputs required, including ablated geometry')
    _device(fused.device)
    # HH_260906 - The scalar prior is a frozen causal-model score, never a target or oracle label.
    context, prior = fused.detach(), original_logits.detach()
    geometry = torch.cat((candidate_xy.detach().flatten(2) / 120.0,
        candidate_speed.detach() / PHYSICAL_MAXIMUM_SPEED_MPS), dim=2)
    if arm == 'prior_only_residual':
        geometry = torch.zeros_like(geometry)
    inputs = torch.cat((context.unsqueeze(1).expand(-1, 12, -1), prior.unsqueeze(-1), geometry), dim=2)
    return prior + head(inputs).squeeze(-1)


def evaluate_head(data, head, arm, device='cpu', check_deadline=None):
    data = data.to(device)
    _require(all(p.device == data.fused.device for p in head.parameters()),
        'head must already be on the evaluation device')
    values = []
    with torch.no_grad():
        for start in range(0, len(data), BATCH_SIZE):
            _check(check_deadline)
            index = slice(start, start + BATCH_SIZE)
            values.append(score_head(head, arm, data.fused[index], data.candidate_xy[index],
                data.candidate_speed[index], data.original_logits[index]))
    logits = torch.cat(values)
    return {**evaluate_logits(data, logits, check_deadline), 'logits_cpu': logits.detach().cpu().clone()}


def fit_frozen_residual(train, val, *, arm, seed, device='cpu',
        callback: Callable[[dict], None] | None = None, check_deadline: Callable[[], None] | None = None):
    validate_pair(train, val)
    device = _device(device)
    before = {data.split: data.digest() for data in (train, val)}
    working = {'train': train.to(device), 'val': val.to(device)}
    head = make_head(arm, seed).to(device)
    initial_head_sha256 = _digest_tensors(head.state_dict().items())
    history, order, baseline_metrics, before_metrics, final_metrics = [], [], {}, {}, {}
    baseline_logits, final_logits, oracles, parity = {}, {}, {}, {}
    try:
        for split, data in working.items():
            result = evaluate_head(data, head, arm, device, check_deadline)
            actual, expected = result.pop('logits_cpu'), data.original_logits.detach().cpu()
            expected_sha256 = _digest_tensors([('logits', expected)])
            actual_sha256 = _digest_tensors([('logits', actual)])
            parity[split] = {'cached_original_logits_required': True,
                'byte_equal': actual_sha256 == expected_sha256 and torch.equal(actual, expected),
                'original_logits_sha256': expected_sha256, 'initial_logits_sha256': actual_sha256,
                'max_abs_logit_difference': float((actual - expected).abs().max()),
                'selected_index_mismatch_count': int((actual.argmax(1) != expected.argmax(1)).sum()),
                'operator_shape': 'Cached original logits plus zero-initialized shared residual; batch4 and final partial batch.',
                'scope': 'Residual initialization versus authenticated cache only; no generator or original-head numerical replay.'}
            _require(parity[split]['byte_equal'], 'zero residual failed exact cached-original logit parity')
            baseline_logits[split] = expected.clone()
            baseline_metrics[split] = evaluate_logits(data, data.original_logits, check_deadline)
            before_metrics[split] = result
            oracles[split] = composite_targets(data, check_deadline)
        targets = oracles['train']
        oracle_digests = {split: _digest_tensors([('composite_oracle', values)]) for split, values in oracles.items()}
        optimizer = torch.optim.AdamW(head.parameters(), lr=LEARNING_RATE, weight_decay=WEIGHT_DECAY)
        step, epoch, samples_seen = 0, 0, 0
        head.train()
        while step < STEPS:
            for batch in cache_core._epoch_batches(len(train), batch_size=BATCH_SIZE, seed=seed, epoch=epoch):
                if step == STEPS:
                    break
                _check(check_deadline)
                index = torch.tensor(batch, device=device, dtype=torch.long)
                data = working['train']
                logits = score_head(head, arm, data.fused[index], data.candidate_xy[index],
                    data.candidate_speed[index], data.original_logits[index])
                score_loss = F.cross_entropy(logits, targets[index])
                objective = SCORE_WEIGHT * score_loss
                _require(bool(torch.isfinite(objective)), 'nonfinite residual loss')
                optimizer.zero_grad(set_to_none=True)
                objective.backward()
                # HH_260906 - Zero initial hidden-layer gradients are valid; missing or nonfinite gradients are not.
                _require(all(p.grad is not None and bool(torch.isfinite(p.grad).all()) for p in head.parameters()),
                    'missing/nonfinite residual gradient')
                norm = torch.nn.utils.clip_grad_norm_(head.parameters(), MAX_GRAD_NORM)
                _require(bool(torch.isfinite(norm)), 'nonfinite residual gradient norm')
                optimizer.step()
                _require(all(bool(torch.isfinite(p).all()) for p in head.parameters()), 'nonfinite residual parameters after update')
                _require(cache_core._nested_tensors_are_finite(optimizer.state_dict()), 'nonfinite residual optimizer state after update')
                step += 1
                samples_seen += len(batch)
                order.append({'epoch': epoch, 'indices': list(batch)})
                record = {'global_step': step, 'epoch': epoch, 'batch_samples': len(batch), 'samples_seen': samples_seen,
                    'sample_indices': list(batch), 'candidate_score_loss': float(score_loss.detach()),
                    'optimization_loss': float(objective.detach()), 'gradient_norm': float(norm)}
                history.append(record)
                if callback is not None:
                    callback(copy.deepcopy(record))
            epoch += 1
        _require(step == STEPS and samples_seen == SAMPLES_SEEN, 'fixed residual step/exposure budget differs')
        for split, data in working.items():
            result = evaluate_head(data, head, arm, device, check_deadline)
            final_logits[split] = result.pop('logits_cpu')
            final_metrics[split] = result
            _require(torch.equal(composite_targets(data, check_deadline), oracles[split]), 'frozen composite teacher changed')
        report = {'artifact_id': ARTIFACT_ID, 'status': 'RESEARCH_HEAD_TRAINING_COMPLETE_NOT_PROMOTED',
            'arm': arm, 'seed': seed, 'device': str(device), 'optimizer': 'AdamW', 'steps': step,
            'batch_size': BATCH_SIZE, 'learning_rate': LEARNING_RATE, 'weight_decay': WEIGHT_DECAY,
            'candidate_score_weight': SCORE_WEIGHT, 'maximum_gradient_norm': MAX_GRAD_NORM,
            'loss_config': TrajectoryLossConfig().to_dict(), 'initial_head_sha256': initial_head_sha256,
            'final_head_sha256': _digest_tensors(head.state_dict().items()),
            'initialization': 'Fresh CPU-seeded identical 449-to-256-to-1 residual MLP; only final Linear weight and bias zeroed; fresh optimizer.',
            'optimization_loss_definition': '0.1 * original hard-argmin composite-oracle cross entropy; no regression, tie modification or extra weights.',
            'sampling_policy': 'Uniform without replacement: original train._epoch_batches no-plan branch, CPU seed+epoch randperm.',
            'sampling_order_sha256': hashlib.sha256(json.dumps(order, separators=(',', ':')).encode()).hexdigest(),
            'composite_oracle_sha256': oracle_digests, 'cache_sha256_before': before, 'cache_sha256_after': dict(before),
            'original_logit_parity': parity, 'baseline_metrics': baseline_metrics,
            'pre_training_metrics': before_metrics, 'post_training_metrics': final_metrics, 'history': history,
            'final_state': {'global_step': step, 'samples_seen': samples_seen, 'last_epoch_index': history[-1]['epoch']},
            'train_sample_ids': list(train.sample_ids), 'val_sample_ids': list(val.sample_ids),
            'source_checkpoint_sha256': train.source_checkpoint_sha256, 'corpus_fingerprint_sha256': train.corpus_fingerprint_sha256,
            'split_fingerprints': {split: data.dataset_fingerprint_sha256 for split, data in working.items()},
            'head_parameter_count': sum(p.numel() for p in head.parameters()), 'candidate_count': 12,
            'residual_inputs': {'frozen_context_features': 256, 'original_candidate_score_features': 1,
                'candidate_geometry_features': 192, 'geometry_provided': arm == 'candidate_residual',
                'xy_divisor_m': 120.0, 'speed_divisor_mps': PHYSICAL_MAXIMUM_SPEED_MPS},
            'generator_parameters_trained': False, 'original_heads_recomputed_or_trained': False,
            'training_data_approved': False, 'vehicle_control_approved': False,
            'automatic_promotion': False, 'runtime_bundle_supported': False,
            'interpretation': 'Residual information ablation: identical allocated structure and initialization, not equal effective information or functional capacity. Zero geometry connections are inactive in the prior-only arm. Initial behavior preservation is not a post-training guarantee. Same-input conflicts persist; no traffic-intent, driving or real-time claim.'}
        return {'report': report, 'head_state_dict': {name: value.detach().cpu().clone() for name, value in head.state_dict().items()},
            'final_logits_cpu': final_logits, 'baseline_logits_cpu': baseline_logits,
            'oracle_indices_cpu': {split: values.detach().cpu().clone() for split, values in oracles.items()}}
    finally:
        _require(train.digest() == before['train'] and val.digest() == before['val'], 'original cache mutated during residual fitting')
        _require(all(data.digest() == before[split] for split, data in working.items()), 'working cache mutated during residual fitting')
