"""HH_260906 - Fit isolated research scorers on immutable K12 DRIVE/STOP caches."""

from __future__ import annotations

import copy
from dataclasses import dataclass, fields
import hashlib
import json
import math
import re
from typing import Callable

import torch
from torch import Tensor, nn
from torch.nn import functional as F

from .contract import ContractError
from .evaluate import _accumulate_batch_metrics
from .frozen_selector import _device, _digest_tensors, _head_digest
from .losses import TrajectoryLossConfig, trajectory_loss
from .model import PHYSICAL_MAXIMUM_SPEED_MPS
from .train import _epoch_batches, _nested_tensors_are_finite


ARTIFACT_ID = 'portable_e2e.frozen_stopmix_selector_research_head.v1'
ARMS = ('linear_pair_reset', 'candidate_reset')
STEPS, BATCH_SIZE, LEARNING_RATE = 1540, 4, 1.0e-4
WEIGHT_DECAY, SCORE_WEIGHT, MAX_GRAD_NORM = 1.0e-4, 0.1, 5.0
TRAIN_COUNT, VAL_COUNT, SAMPLES_SEEN = 1147, 337, 6155
_TENSOR_FIELDS = ('fused', 'candidate_xy', 'candidate_speed', 'target_xy',
    'target_speed_mps', 'target_yaw_rad', 'target_valid', 'original_logits')


def _require(value, message):
    if not value:
        raise ContractError(message)


def _check(check_deadline):
    if check_deadline is not None:
        check_deadline()


@dataclass(frozen=True)
class CachedStopmixSelectorData:
    # HH_260906 - Cache construction clones every tensor; no generator graph, label rewrite or sample filter is retained.
    split: str
    sample_ids: tuple[str, ...]
    episode_ids: tuple[str, ...]
    fused: Tensor
    candidate_xy: Tensor
    candidate_speed: Tensor
    target_xy: Tensor
    target_speed_mps: Tensor
    target_yaw_rad: Tensor
    target_valid: Tensor
    source_checkpoint_sha256: str
    corpus_fingerprint_sha256: str
    dataset_fingerprint_sha256: str
    original_logits: Tensor

    def __post_init__(self):
        _require(self.split in ('train', 'val'), 'only TRAIN and development VAL caches are permitted')
        for name in ('source_checkpoint_sha256', 'corpus_fingerprint_sha256', 'dataset_fingerprint_sha256'):
            value = getattr(self, name)
            _require(isinstance(value, str) and re.fullmatch('[0-9a-f]{64}', value), name + ' must be SHA256')
        for name in ('sample_ids', 'episode_ids'):
            values = getattr(self, name)
            _require(isinstance(values, (tuple, list)) and bool(values)
                and all(isinstance(v, str) and bool(v) for v in values), name + ' must contain per-row IDs')
            object.__setattr__(self, name, tuple(values))
        count = len(self.sample_ids)
        _require(len(set(self.sample_ids)) == count and len(self.episode_ids) == count, 'aligned unique sample IDs required')
        shapes = {'fused': (count, 256), 'candidate_xy': (count, 12, 64, 2),
            'candidate_speed': (count, 12, 64), 'target_xy': (count, 64, 2),
            'target_speed_mps': (count, 64), 'target_yaw_rad': (count, 64),
            'target_valid': (count, 64), 'original_logits': (count, 12)}
        device = None
        for name in _TENSOR_FIELDS:
            value = getattr(self, name)
            dtype = torch.bool if name == 'target_valid' else torch.float32
            _require(isinstance(value, Tensor) and tuple(value.shape) == shapes[name] and value.dtype == dtype,
                name + ' violates the fixed float32/bool N/256/12/64 cache ABI')
            if device is None:
                device = _device(value.device)
            _require(value.device == device and bool(torch.isfinite(value).all()), 'cache tensors must be finite on one permitted device')
            object.__setattr__(self, name, value.detach().clone().contiguous())
        _require(bool(self.target_valid.any(dim=1).all())
            and not bool((~self.target_valid[:, :-1] & self.target_valid[:, 1:]).any()),
            'target masks require a nonempty contiguous valid prefix')

    def __len__(self):
        return len(self.sample_ids)

    def digest(self):
        metadata = {f.name: getattr(self, f.name) for f in fields(self) if f.name not in _TENSOR_FIELDS}
        tensors = _digest_tensors((name, getattr(self, name)) for name in _TENSOR_FIELDS)
        return hashlib.sha256((json.dumps(metadata, sort_keys=True, separators=(',', ':')) + '\0' + tensors).encode()).hexdigest()

    def to(self, device):
        device = _device(device)
        if device == self.fused.device:
            return self
        values = {f.name: getattr(self, f.name) for f in fields(self)}
        values.update({name: getattr(self, name).to(device) for name in _TENSOR_FIELDS})
        return CachedStopmixSelectorData(**values)


def validate_pair(train, val):
    _require(type(train) is CachedStopmixSelectorData and type(val) is CachedStopmixSelectorData, 'validated K12 cache objects required')
    _require(train.split == 'train' and val.split == 'val', 'separate TRAIN and VAL caches required')
    _require(len(train) == TRAIN_COUNT and len(val) == VAL_COUNT
        and len(set(train.episode_ids)) == 3 and len(set(val.episode_ids)) == 1,
        'complete TRAIN1147/three episodes and VAL337/one episode required')
    _require(not (set(train.sample_ids) & set(val.sample_ids))
        and not (set(train.episode_ids) & set(val.episode_ids)), 'TRAIN/VAL sample or episode leakage')
    _require(train.source_checkpoint_sha256 == val.source_checkpoint_sha256
        and train.corpus_fingerprint_sha256 == val.corpus_fingerprint_sha256
        and train.dataset_fingerprint_sha256 != val.dataset_fingerprint_sha256,
        'parent checkpoint, corpus or split fingerprints disagree')


def _validate_linear(head):
    _require(type(head) is nn.Linear and head.in_features == 256 and head.out_features == 6 and head.bias is not None,
        'each original head must be Linear(256,6) with bias')
    devices = {_device(p.device) for p in head.parameters()}
    _require(len(devices) == 1 and all(p.dtype == torch.float32 and bool(torch.isfinite(p).all()) for p in head.parameters()),
        'original heads must be finite float32')


def _cpu_linear(inputs, outputs):
    # HH_260906 - Explicit CPU allocation also supports the local older PyTorch without Linear's device keyword.
    result = nn.Linear.__new__(nn.Linear)
    nn.Module.__init__(result)
    result.in_features, result.out_features = inputs, outputs
    result.weight = nn.Parameter(torch.empty((outputs, inputs), device='cpu', dtype=torch.float32))
    result.bias = nn.Parameter(torch.empty(outputs, device='cpu', dtype=torch.float32))
    result.reset_parameters()
    return result


class TwinLinearHeads(nn.Module):
    # HH_260906 - Keep two original six-output GEMMs; a combined twelve-output Linear is not a byte-parity reference.
    def __init__(self):
        super().__init__()
        self.drive = _cpu_linear(256, 6)
        self.stop = _cpu_linear(256, 6)
        nn.init.zeros_(self.drive.bias)
        nn.init.zeros_(self.stop.bias)

    def forward(self, context):
        return torch.cat((self.drive(context), self.stop(context)), dim=1)


def clone_original_heads(drive, stop):
    _validate_linear(drive)
    _validate_linear(stop)
    _require(next(drive.parameters()).device == next(stop.parameters()).device, 'original head devices differ')
    # HH_260906 - Copy modules without constructors or RNG draws, preserving separate original operator shapes.
    result = TwinLinearHeads.__new__(TwinLinearHeads)
    nn.Module.__init__(result)
    result.drive, result.stop = copy.deepcopy(drive), copy.deepcopy(stop)
    result.requires_grad_(False)
    for parameter in result.parameters():
        parameter.grad = None
    return result.eval()


def make_head(arm, seed):
    _require(arm in ARMS and type(seed) is int and 0 <= seed < 2 ** 63, 'unknown arm or invalid initialization seed')
    _require(torch.get_default_dtype() == torch.float32, 'fixed float32 initialization is required')
    # HH_260906 - Fork only CPU RNG; head initialization never touches another CUDA device or caller RNG state.
    with torch.random.fork_rng(devices=[]):
        torch.random.default_generator.manual_seed(seed)
        if arm == 'linear_pair_reset':
            head = TwinLinearHeads()
        else:
            head = nn.Sequential(_cpu_linear(448, 256), nn.ReLU(inplace=True), _cpu_linear(256, 1))
            nn.init.zeros_(head[-1].bias)
    return head


def score_head(head, arm, fused, candidate_xy, candidate_speed):
    _require(arm in ARMS and fused.ndim == 2 and fused.shape[1] == 256
        and tuple(candidate_xy.shape) == (len(fused), 12, 64, 2)
        and tuple(candidate_speed.shape) == (len(fused), 12, 64), 'fixed K12 scoring shapes/arm required')
    context = fused.detach()
    if arm == 'linear_pair_reset':
        _require(type(head) is TwinLinearHeads, 'linear reference requires two separate six-output heads')
        return head(context)
    # HH_260906 - Detach context and geometry: only this independent shared scorer receives optimization gradients.
    geometry = torch.cat((candidate_xy.detach().flatten(2) / 120.0,
        candidate_speed.detach() / PHYSICAL_MAXIMUM_SPEED_MPS), dim=2)
    return head(torch.cat((context.unsqueeze(1).expand(-1, 12, -1), geometry), dim=2)).squeeze(-1)


def composite_targets(data, check_deadline=None):
    targets = []
    with torch.no_grad():
        for start in range(0, len(data), BATCH_SIZE):
            _check(check_deadline)
            index = slice(start, start + BATCH_SIZE)
            targets.append(trajectory_loss(data.candidate_xy[index], data.candidate_speed[index], data.original_logits[index],
                data.target_xy[index], data.target_speed_mps[index], data.target_valid[index], TrajectoryLossConfig(),
                target_yaw=data.target_yaw_rad[index])['oracle_candidate_index'])
    return torch.cat(targets).detach().clone()


def evaluate_logits(data, logits, check_deadline=None):
    _require(isinstance(logits, Tensor) and tuple(logits.shape) == (len(data), 12)
        and logits.dtype == torch.float32 and logits.device == data.fused.device and bool(torch.isfinite(logits).all()),
        'finite float32 K12 logits on the cache device required')
    sums, counts = {}, {}
    with torch.no_grad():
        for start in range(0, len(data), BATCH_SIZE):
            _check(check_deadline)
            index = slice(start, start + BATCH_SIZE)
            _accumulate_batch_metrics(sums, counts, candidate_xy=data.candidate_xy[index],
                candidate_speed=data.candidate_speed[index], candidate_logits=logits[index],
                target_xy=data.target_xy[index], target_speed=data.target_speed_mps[index],
                target_valid=data.target_valid[index], target_yaw=data.target_yaw_rad[index], loss_config=TrajectoryLossConfig())
        oracle, selected = composite_targets(data, check_deadline), logits.argmax(dim=1)
    metrics = {name: sums[name] / counts[name] for name in sums}
    _require(all(math.isfinite(v) for v in metrics.values()), 'nonfinite frozen evaluation metrics')
    return {'split': data.split, 'sample_count': len(data), 'metrics': metrics, 'metric_counts': counts,
        'selected_histogram': torch.bincount(selected, minlength=12).cpu().tolist(),
        'composite_oracle_histogram': torch.bincount(oracle, minlength=12).cpu().tolist(),
        'selected_composite_oracle_agreement_count': int((selected == oracle).sum()),
        'interpretation': 'In-sample TRAIN diagnostic.' if data.split == 'train' else 'Development VAL, not held-out test or driving approval.'}


def evaluate_head(data, head, arm, device='cpu', check_deadline=None):
    data = data.to(device)
    _require(all(p.device == data.fused.device for p in head.parameters()), 'head must already be on the evaluation device')
    values = []
    with torch.no_grad():
        for start in range(0, len(data), BATCH_SIZE):
            _check(check_deadline)
            index = slice(start, start + BATCH_SIZE)
            values.append(score_head(head, arm, data.fused[index], data.candidate_xy[index], data.candidate_speed[index]))
    logits = torch.cat(values)
    return {**evaluate_logits(data, logits, check_deadline), 'logits_cpu': logits.detach().cpu().clone()}


def fit_frozen_selector(train, val, *, original_drive_head, original_stop_head, arm, seed, device='cpu',
        callback: Callable[[dict], None] | None = None, check_deadline: Callable[[], None] | None = None):
    validate_pair(train, val)
    _validate_linear(original_drive_head)
    _validate_linear(original_stop_head)
    device = _device(device)
    before = {data.split: data.digest() for data in (train, val)}
    original = {name: _head_digest(head) for name, head in (('drive', original_drive_head), ('stop', original_stop_head))}
    working = {'train': train.to(device), 'val': val.to(device)}
    head = make_head(arm, seed).to(device)
    initial_head_sha256 = _digest_tensors(head.state_dict().items())
    reference = clone_original_heads(original_drive_head, original_stop_head).to(device)
    history, order, baseline_metrics, before_metrics, final_metrics = [], [], {}, {}, {}
    baseline_logits, final_logits, oracles, parity = {}, {}, {}, {}
    try:
        for split, data in working.items():
            result = evaluate_head(data, reference, 'linear_pair_reset', device, check_deadline)
            actual = result.pop('logits_cpu').to(device)
            expected = data.original_logits
            parity[split] = {'cached_original_logits_required': True, 'byte_equal': torch.equal(actual, expected),
                'max_abs_logit_difference': float((actual - expected).abs().max()),
                'selected_index_mismatch_count': int((actual.argmax(1) != expected.argmax(1)).sum()),
                'operator_shape': 'Two separate Linear(256,6) calls, then concatenation; batch4 and final partial batch.'}
            _require(parity[split]['byte_equal'], 'original two-head cached logits failed exact baseline parity')
            baseline_logits[split] = expected.detach().cpu().clone()
            baseline_metrics[split] = evaluate_logits(data, expected, check_deadline)
            result = evaluate_head(data, head, arm, device, check_deadline)
            result.pop('logits_cpu')
            before_metrics[split] = result
            oracles[split] = composite_targets(data, check_deadline)
        targets = oracles['train']
        oracle_digests = {split: _digest_tensors([('composite_oracle', values)]) for split, values in oracles.items()}
        optimizer = torch.optim.AdamW(head.parameters(), lr=LEARNING_RATE, weight_decay=WEIGHT_DECAY)
        step, epoch, samples_seen = 0, 0, 0
        head.train()
        while step < STEPS:
            for batch in _epoch_batches(len(train), batch_size=BATCH_SIZE, seed=seed, epoch=epoch):
                if step == STEPS:
                    break
                _check(check_deadline)
                index = torch.tensor(batch, device=device, dtype=torch.long)
                data = working['train']
                logits = score_head(head, arm, data.fused[index], data.candidate_xy[index], data.candidate_speed[index])
                score_loss = F.cross_entropy(logits, targets[index])
                objective = SCORE_WEIGHT * score_loss
                _require(bool(torch.isfinite(objective)), 'nonfinite scorer loss')
                optimizer.zero_grad(set_to_none=True)
                objective.backward()
                _require(all(p.grad is not None and bool(torch.isfinite(p.grad).all()) for p in head.parameters()), 'missing/nonfinite head gradient')
                norm = torch.nn.utils.clip_grad_norm_(head.parameters(), MAX_GRAD_NORM)
                _require(bool(torch.isfinite(norm)), 'nonfinite gradient norm')
                optimizer.step()
                _require(all(bool(torch.isfinite(p).all()) for p in head.parameters()), 'nonfinite head parameters after update')
                _require(_nested_tensors_are_finite(optimizer.state_dict()), 'nonfinite head optimizer state after update')
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
        _require(step == STEPS and samples_seen == SAMPLES_SEEN, 'fixed head fit step/exposure budget differs')
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
            'initialization': 'Fresh CPU-seeded head with fresh optimizer; all generator tensors and context detached.',
            'optimization_loss_definition': '0.1 * original hard-argmin composite-oracle cross entropy; no regression, tie modification or extra weights.',
            'sampling_policy': 'Uniform without replacement: original train._epoch_batches no-plan branch, CPU seed+epoch randperm.',
            'sampling_order_sha256': hashlib.sha256(json.dumps(order, separators=(',', ':')).encode()).hexdigest(),
            'composite_oracle_sha256': oracle_digests, 'cache_sha256_before': before, 'cache_sha256_after': dict(before),
            'original_head_sha256_before': original, 'original_head_sha256_after': dict(original),
            'original_logit_parity': parity, 'baseline_metrics': baseline_metrics,
            'pre_training_metrics': before_metrics, 'post_training_metrics': final_metrics, 'history': history,
            'final_state': {'global_step': step, 'samples_seen': samples_seen, 'last_epoch_index': history[-1]['epoch']},
            'train_sample_ids': list(train.sample_ids), 'val_sample_ids': list(val.sample_ids),
            'source_checkpoint_sha256': train.source_checkpoint_sha256, 'corpus_fingerprint_sha256': train.corpus_fingerprint_sha256,
            'split_fingerprints': {split: data.dataset_fingerprint_sha256 for split, data in working.items()},
            'head_parameter_count': sum(p.numel() for p in head.parameters()), 'candidate_count': 12,
            'generator_parameters_trained': False, 'training_data_approved': False, 'vehicle_control_approved': False,
            'automatic_promotion': False, 'runtime_bundle_supported': False,
            'interpretation': 'Head-only research; architecture and capacity differ. Same-input conflicting labels persist; no traffic-intent, driving or real-time claim.'}
        return {'report': report, 'head_state_dict': {name: value.detach().cpu().clone() for name, value in head.state_dict().items()},
            'final_logits_cpu': final_logits, 'baseline_logits_cpu': baseline_logits,
            'oracle_indices_cpu': {split: values.detach().cpu().clone() for split, values in oracles.items()}}
    finally:
        _require(train.digest() == before['train'] and val.digest() == before['val'], 'original cache mutated during head fitting')
        _require(all(data.digest() == before[split] for split, data in working.items()), 'working cache mutated during head fitting')
        _require(_head_digest(original_drive_head) == original['drive'] and _head_digest(original_stop_head) == original['stop'],
            'original two-head reference mutated during fitting')
