"""HH_260906 - Fit research-only scorer heads on immutable cached physical-model outputs."""

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
from .losses import TrajectoryLossConfig, trajectory_loss
from .model import PHYSICAL_MAXIMUM_SPEED_MPS
from .train import _epoch_batches


ARTIFACT_ID = 'portable_e2e.frozen_selector_research_head.v1'
ARMS = ('linear_continue', 'linear_reset', 'candidate_reset')
STEPS, BATCH_SIZE, LEARNING_RATE = 1540, 4, 1.0e-4
WEIGHT_DECAY, SCORE_WEIGHT, MAX_GRAD_NORM = 1.0e-4, 0.1, 5.0
_TENSOR_FIELDS = ('fused', 'candidate_xy', 'candidate_speed', 'target_xy',
    'target_speed_mps', 'target_yaw_rad', 'target_valid', 'original_logits')


def _tensor_bytes(value: Tensor) -> bytes:
    return value.detach().cpu().contiguous().numpy().tobytes()


def _digest_tensors(values) -> str:
    digest = hashlib.sha256()
    for name, value in values:
        digest.update(json.dumps([name, str(value.dtype), list(value.shape)], separators=(',', ':')).encode())
        digest.update(b'\0')
        digest.update(_tensor_bytes(value))
    return digest.hexdigest()


def _device(device) -> torch.device:
    try:
        result = torch.device(device)
    except (TypeError, ValueError, RuntimeError) as error:
        raise ContractError('frozen-selector research supports CPU or visible CUDA device zero only') from error
    if str(result) not in ('cpu', 'cuda:0'):
        raise ContractError('frozen-selector research supports CPU or visible CUDA device zero only')
    return result


@dataclass(frozen=True)
class CachedSelectorData:
    # HH_260906 - Each cache owns detached clones; byte fingerprints detect any later in-place mutation.
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
    original_logits: Tensor | None = None

    def __post_init__(self):
        if self.split not in ('train', 'val'):
            raise ContractError('cache split must be train or val; test is forbidden')
        for name in ('source_checkpoint_sha256', 'corpus_fingerprint_sha256', 'dataset_fingerprint_sha256'):
            value = getattr(self, name)
            if not isinstance(value, str) or re.fullmatch('[0-9a-f]{64}', value) is None:
                raise ContractError(f'{name} must be a SHA-256 fingerprint')
        for name in ('sample_ids', 'episode_ids'):
            value = getattr(self, name)
            if not isinstance(value, (tuple, list)):
                raise ContractError(f'{name} must be a per-row sequence')
            value = tuple(value)
            if not value or any(not isinstance(item, str) or not item for item in value):
                raise ContractError(f'{name} must contain nonempty strings')
            object.__setattr__(self, name, value)
        count = len(self.sample_ids)
        if len(set(self.sample_ids)) != count or len(self.episode_ids) != count:
            raise ContractError('cache IDs must be aligned and sample IDs unique')
        shapes = {'fused': (count, 256), 'candidate_xy': (count, 6, 64, 2),
            'candidate_speed': (count, 6, 64), 'target_xy': (count, 64, 2),
            'target_speed_mps': (count, 64), 'target_yaw_rad': (count, 64),
            'target_valid': (count, 64), 'original_logits': (count, 6)}
        device = None
        for name in _TENSOR_FIELDS:
            value = getattr(self, name)
            if value is None and name == 'original_logits':
                continue
            dtype = torch.bool if name == 'target_valid' else torch.float32
            if not isinstance(value, Tensor) or tuple(value.shape) != shapes[name] or value.dtype != dtype:
                raise ContractError(f'{name} violates the fixed float32/bool N/256/6/64 cache ABI')
            if device is None:
                device = value.device
                _device(device)
            if value.device != device or not bool(torch.isfinite(value).all()):
                raise ContractError('all cache tensors must be finite and on the same permitted device')
            object.__setattr__(self, name, value.detach().clone().contiguous())
        if not bool(self.target_valid.any(dim=1).all()) or bool((~self.target_valid[:, :-1] & self.target_valid[:, 1:]).any()):
            raise ContractError('target_valid must preserve a nonempty contiguous valid prefix')

    def __len__(self):
        return len(self.sample_ids)

    def digest(self) -> str:
        metadata = {field.name: getattr(self, field.name) for field in fields(self)
            if field.name not in _TENSOR_FIELDS}
        tensor_digest = _digest_tensors((name, getattr(self, name)) for name in _TENSOR_FIELDS
            if getattr(self, name) is not None)
        return hashlib.sha256((json.dumps(metadata, sort_keys=True, separators=(',', ':'))
            + '\0' + tensor_digest).encode()).hexdigest()

    def to(self, device):
        device = _device(device)
        if self.fused.device == device:
            return self
        values = {field.name: getattr(self, field.name) for field in fields(self)}
        values.update({name: value.to(device) for name in _TENSOR_FIELDS
            if (value := getattr(self, name)) is not None})
        return CachedSelectorData(**values)


def _validate_pair(train, val):
    if not isinstance(train, CachedSelectorData) or not isinstance(val, CachedSelectorData):
        raise ContractError('fit requires validated CachedSelectorData objects')
    if train.split != 'train' or val.split != 'val':
        raise ContractError('fit requires separately labelled train and val caches')
    if set(train.sample_ids) & set(val.sample_ids) or set(train.episode_ids) & set(val.episode_ids):
        raise ContractError('training/validation sample or episode leakage')
    if (train.source_checkpoint_sha256 != val.source_checkpoint_sha256
            or train.corpus_fingerprint_sha256 != val.corpus_fingerprint_sha256
            or train.dataset_fingerprint_sha256 == val.dataset_fingerprint_sha256):
        raise ContractError('cache parent checkpoint/corpus or split fingerprints disagree')


def _head_digest(head):
    values = list(head.state_dict().items())
    for name, parameter in head.named_parameters():
        if parameter.grad is not None:
            values.append((f'grad:{name}', parameter.grad))
    metadata = [(name, parameter.requires_grad, parameter.grad is None) for name, parameter in head.named_parameters()]
    return hashlib.sha256((_digest_tensors(values) + json.dumps(metadata) + str(head.training)).encode()).hexdigest()


def _validate_original_head(head):
    if (type(head) is not nn.Linear or head.in_features != 256 or head.out_features != 6
            or head.bias is None or any(parameter.dtype != torch.float32 for parameter in head.parameters())
            or not all(bool(torch.isfinite(parameter).all()) for parameter in head.parameters())):
        raise ContractError('original C head must be a finite float32 Linear(256,6) with bias')


def make_head(original_head, arm, seed):
    _validate_original_head(original_head)
    if arm not in ARMS or type(seed) is not int or not 0 <= seed < 2 ** 63:
        raise ContractError('unknown research arm or invalid initialization seed')
    if torch.get_default_dtype() != torch.float32:
        raise ContractError('research head initialization requires the fixed float32 default dtype')
    # HH_260906 - Construct on CPU with a forked CPU RNG; never seed or touch another CUDA device.
    with torch.random.fork_rng(devices=[]):
        torch.random.default_generator.manual_seed(seed)
        if arm == 'linear_continue':
            head = copy.deepcopy(original_head).cpu()
        elif arm == 'linear_reset':
            head = nn.Linear(256, 6)
            nn.init.zeros_(head.bias)
        else:
            head = nn.Sequential(nn.Linear(256 + 64 * 3, 256), nn.ReLU(inplace=True), nn.Linear(256, 1))
            nn.init.zeros_(head[-1].bias)
    head.requires_grad_(True)
    for parameter in head.parameters():
        parameter.grad = None
    return head


def score_head(head, arm, fused, candidate_xy, candidate_speed):
    # HH_260906 - Match E's scorer normalization while detaching context as well as decoded geometry.
    if arm not in ARMS:
        raise ContractError('unknown frozen-selector arm')
    context = fused.detach()
    if arm != 'candidate_reset':
        return head(context)
    geometry = torch.cat((candidate_xy.detach().flatten(2) / 120.0,
        candidate_speed.detach() / PHYSICAL_MAXIMUM_SPEED_MPS), dim=2)
    return head(torch.cat((context.unsqueeze(1).expand(-1, 6, -1), geometry), dim=2)).squeeze(-1)


def _targets(data):
    with torch.no_grad():
        return trajectory_loss(data.candidate_xy, data.candidate_speed,
            torch.zeros((len(data), 6), device=data.fused.device, dtype=torch.float32), data.target_xy,
            data.target_speed_mps, data.target_valid, TrajectoryLossConfig(),
            target_yaw=data.target_yaw_rad)['oracle_candidate_index'].detach().clone()


def evaluate_logits(data, logits):
    if (tuple(logits.shape) != (len(data), 6) or logits.dtype != torch.float32
            or logits.device != data.fused.device or not bool(torch.isfinite(logits).all())):
        raise ContractError('evaluation logits must be finite and match the frozen cache ABI/device')
    sums, counts = {}, {}
    with torch.no_grad():
        for start in range(0, len(data), BATCH_SIZE):
            index = slice(start, start + BATCH_SIZE)
            _accumulate_batch_metrics(sums, counts, candidate_xy=data.candidate_xy[index],
                candidate_speed=data.candidate_speed[index], candidate_logits=logits[index],
                target_xy=data.target_xy[index], target_speed=data.target_speed_mps[index],
                target_valid=data.target_valid[index], target_yaw=data.target_yaw_rad[index],
                loss_config=TrajectoryLossConfig())
        oracle, selected = _targets(data), logits.argmax(dim=1)
    metrics = {name: sums[name] / counts[name] for name in sums}
    if not all(math.isfinite(value) for value in metrics.values()):
        raise ContractError('frozen-selector evaluation produced nonfinite metrics')
    return {'split': data.split, 'sample_count': len(data), 'metrics': metrics, 'metric_counts': counts,
        'selected_histogram': torch.bincount(selected, minlength=6).cpu().tolist(),
        'composite_oracle_histogram': torch.bincount(oracle, minlength=6).cpu().tolist(),
        'selected_composite_oracle_agreement_count': int((selected == oracle).sum()),
        'interpretation': 'In-sample training metrics, not generalization.' if data.split == 'train'
            else 'Development validation, not held-out test or driving approval.'}


def evaluate_head(data, head, arm, device='cpu'):
    data = data.to(device)
    if any(parameter.device != data.fused.device for parameter in head.parameters()):
        raise ContractError('move the independent research head to the evaluation device explicitly')
    logits = []
    with torch.no_grad():
        for start in range(0, len(data), BATCH_SIZE):
            index = slice(start, start + BATCH_SIZE)
            logits.append(score_head(head, arm, data.fused[index], data.candidate_xy[index], data.candidate_speed[index]))
    values = torch.cat(logits)
    return {**evaluate_logits(data, values), 'logits_cpu': values.detach().cpu().clone()}


def fit_frozen_selector(train, val, *, original_head, arm, seed, device='cpu',
        callback: Callable[[dict], None] | None = None):
    _validate_pair(train, val)
    _validate_original_head(original_head)
    device = _device(device)
    caches_before = {'train': train.digest(), 'val': val.digest()}
    original_before = _head_digest(original_head)
    working = {'train': train.to(device), 'val': val.to(device)}
    head = make_head(original_head, arm, seed).to(device)
    reference = copy.deepcopy(original_head).to(device).requires_grad_(False)
    history, order_records, baseline_metrics, before_metrics, final_metrics = [], [], {}, {}, {}
    baseline_logits, final_logits, parity = {}, {}, {}
    try:
        for split, data in working.items():
            reference_result = evaluate_head(data, reference, 'linear_continue', device)
            reference_logits = reference_result.pop('logits_cpu').to(device)
            baseline = reference_logits if data.original_logits is None else data.original_logits
            baseline_logits[split] = baseline.detach().cpu().clone()
            baseline_metrics[split] = evaluate_logits(data, baseline)
            parity[split] = {'cached_original_logits_supplied': data.original_logits is not None,
                'max_abs_logit_difference': float((reference_logits - baseline).abs().max()),
                'selected_index_mismatch_count': int((reference_logits.argmax(1) != baseline.argmax(1)).sum())}
            before = evaluate_head(data, head, arm, device)
            before.pop('logits_cpu')
            before_metrics[split] = before
        targets = _targets(working['train'])
        targets_digest = _digest_tensors([('train_composite_oracle', targets)])
        optimizer = torch.optim.AdamW(head.parameters(), lr=LEARNING_RATE, weight_decay=WEIGHT_DECAY)
        samples_seen, step, epoch = 0, 0, 0
        head.train()
        while step < STEPS:
            for batch in _epoch_batches(len(train), batch_size=BATCH_SIZE, seed=seed, epoch=epoch):
                if step == STEPS:
                    break
                index = torch.tensor(batch, device=device, dtype=torch.long)
                data = working['train']
                logits = score_head(head, arm, data.fused[index], data.candidate_xy[index], data.candidate_speed[index])
                score_loss = F.cross_entropy(logits, targets[index])
                optimization_loss = SCORE_WEIGHT * score_loss
                if not bool(torch.isfinite(optimization_loss)):
                    raise ContractError('nonfinite scorer optimization loss')
                optimizer.zero_grad(set_to_none=True)
                optimization_loss.backward()
                if any(parameter.grad is None or not bool(torch.isfinite(parameter.grad).all()) for parameter in head.parameters()):
                    raise ContractError('scorer gradients are missing or nonfinite')
                norm = torch.nn.utils.clip_grad_norm_(head.parameters(), MAX_GRAD_NORM)
                if not bool(torch.isfinite(norm)):
                    raise ContractError('nonfinite scorer gradient norm')
                optimizer.step()
                if not all(bool(torch.isfinite(parameter).all()) for parameter in head.parameters()):
                    raise ContractError('scorer optimizer produced nonfinite parameters')
                step += 1
                samples_seen += len(batch)
                order_records.append({'epoch': epoch, 'indices': list(batch)})
                record = {'global_step': step, 'epoch': epoch, 'batch_samples': len(batch),
                    'samples_seen': samples_seen, 'sample_indices': list(batch),
                    'candidate_score_loss': float(score_loss.detach()),
                    'optimization_loss': float(optimization_loss.detach()), 'gradient_norm': float(norm)}
                history.append(record)
                if callback is not None:
                    callback(copy.deepcopy(record))
            epoch += 1
        for split, data in working.items():
            result = evaluate_head(data, head, arm, device)
            final_logits[split] = result.pop('logits_cpu')
            final_metrics[split] = result
        report = {'artifact_id': ARTIFACT_ID, 'status': 'RESEARCH_HEAD_TRAINING_COMPLETE_NOT_PROMOTED',
            'arm': arm, 'seed': seed, 'device': str(device), 'optimizer': 'AdamW',
            'initialization': 'Deep-copied C linear head with a fresh optimizer.' if arm == 'linear_continue'
                else 'Fresh seeded head with a fresh optimizer; no generator parameters are trained.',
            'steps': STEPS, 'batch_size': BATCH_SIZE, 'learning_rate': LEARNING_RATE,
            'weight_decay': WEIGHT_DECAY, 'candidate_score_weight': SCORE_WEIGHT,
            'maximum_gradient_norm': MAX_GRAD_NORM, 'loss_config': TrajectoryLossConfig().to_dict(),
            'optimization_loss_definition': '0.1 * cross_entropy(head logits, fixed composite trajectory_loss oracle); no generator parameters or regression gradients.',
            'sampling_policy': 'Uniform without replacement via train._epoch_batches no-plan branch; CPU randperm seeded with seed+epoch. This is not the original domain-balanced order.',
            'sampling_order_sha256': hashlib.sha256(json.dumps(order_records, separators=(',', ':')).encode()).hexdigest(),
            'train_composite_targets_sha256': targets_digest,
            'cache_sha256_before': caches_before, 'cache_sha256_after': dict(caches_before),
            'original_head_sha256_before': original_before, 'original_head_sha256_after': original_before,
            'original_logit_parity': parity, 'baseline_metrics': baseline_metrics,
            'pre_training_metrics': before_metrics, 'post_training_metrics': final_metrics,
            'history': history, 'final_state': {'global_step': step, 'samples_seen': samples_seen,
                'last_epoch_index': history[-1]['epoch'], 'last_epoch_steps': sum(record['epoch'] == history[-1]['epoch'] for record in history)},
            'train_sample_ids': list(train.sample_ids), 'val_sample_ids': list(val.sample_ids),
            'source_checkpoint_sha256': train.source_checkpoint_sha256,
            'corpus_fingerprint_sha256': train.corpus_fingerprint_sha256,
            'split_fingerprints': {split: data.dataset_fingerprint_sha256 for split, data in working.items()},
            'head_parameter_count': sum(parameter.numel() for parameter in head.parameters()),
            'vehicle_control_approved': False, 'automatic_promotion': False,
            'interpretation': 'Scorer-only cached research artifact; not an accepted runtime checkpoint, learned driving, or 10 Hz runtime evidence.'}
        return {'report': report, 'head_state_dict': {name: value.detach().cpu().clone() for name, value in head.state_dict().items()},
            'final_logits_cpu': final_logits, 'baseline_logits_cpu': baseline_logits}
    finally:
        if train.digest() != caches_before['train'] or val.digest() != caches_before['val']:
            raise ContractError('input cache mutated during scorer-only training')
        if any(data.digest() != caches_before[split] for split, data in working.items()):
            raise ContractError('working cache mutated during scorer-only training')
        if _head_digest(original_head) != original_before:
            raise ContractError('original C head mutated during scorer-only training')
