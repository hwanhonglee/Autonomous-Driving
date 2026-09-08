"""HH_260906 - Compare fixed-parent CE continuation with a DRIVE-only pairwise ranking auxiliary."""

from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
from typing import Callable, Mapping

import torch
from torch import Tensor
from torch.nn import functional as F

from . import frozen_stopmix_selector as cache_core
from . import frozen_stopmix_residual as residual
from . import losses as loss_module


ARTIFACT_ID = 'portable_e2e.frozen_stopmix_drive_rank_research_head.v1'
ARMS = ('ce_continue', 'drive_pair_continue')
STEPS, BATCH_SIZE, SAMPLES_SEEN = 1540, 4, 6155
TRAIN_COUNT, VAL_COUNT, HEAD_PARAMETER_COUNT = 1147, 337, 115457
LEARNING_RATE, WEIGHT_DECAY, SCORE_WEIGHT, MAX_GRAD_NORM = 1.e-4, 1.e-4, .1, 5.
LOSS_SHA = 'cdf37262024435c3a04888bf816c24db1b769b87034f08e1541c02d22da80973'
CachedStopmixSelectorData = cache_core.CachedStopmixSelectorData
_require, _check, _device = cache_core._require, cache_core._check, cache_core._device
_digest_tensors = cache_core._digest_tensors
_STATE_SHAPES = {'0.weight': (256, 449), '0.bias': (256,), '2.weight': (1, 256), '2.bias': (1,)}


def drive_pair_rank_loss(logits, costs):
    _require(isinstance(logits, Tensor) and isinstance(costs, Tensor) and logits.ndim == 2
        and len(logits) > 0 and logits.shape[1] == 12 and costs.shape == logits.shape,
        'full Nx12 logits and composite costs required')
    _require(logits.dtype in (torch.float32, torch.float64) and costs.dtype == logits.dtype
        and costs.device == logits.device and bool(torch.isfinite(logits).all()) and bool(torch.isfinite(costs).all()),
        'finite matching logits/cost tensors required')
    _device(logits.device)
    # HH_260906 - All fifteen DRIVE pairs contribute; target-derived groups never select or remove training rows.
    pairs = torch.triu_indices(6, 6, offset=1, device=logits.device)
    cost = costs.detach()
    delta_cost = cost[:, pairs[0]] - cost[:, pairs[1]]
    delta_logit = logits[:, pairs[0]] - logits[:, pairs[1]]
    gap = delta_cost.abs()
    total = gap.sum(dim=1, keepdim=True)
    _require(bool(torch.isfinite(delta_cost).all()) and bool(torch.isfinite(delta_logit).all())
        and bool(torch.isfinite(total).all()), 'nonfinite pair differences or normalization')
    # HH_260906 - Exact cost ties receive zero weight; an all-tied DRIVE set has zero auxiliary loss without epsilon or temperature.
    denominator = torch.where(total == 0, torch.ones_like(total), total)
    weights = gap / denominator
    per_row = (weights * F.softplus(delta_cost.sign() * delta_logit)).sum(dim=1)
    result = per_row.mean()
    _require(bool(torch.isfinite(result)), 'nonfinite DRIVE pair ranking loss')
    return result


def make_parent_head(state, seed):
    _require(isinstance(state, Mapping) and set(state) == set(_STATE_SHAPES), 'exact residual parent parameter keys required')
    for name, shape in _STATE_SHAPES.items():
        value = state[name]
        _require(isinstance(value, Tensor) and value.device.type == 'cpu' and value.dtype == torch.float32
            and tuple(value.shape) == shape and not value.requires_grad and bool(torch.isfinite(value).all()),
            'parent parameters must be detached finite CPU float32 with exact shapes')
    head = residual.make_head('candidate_residual', seed)
    head.load_state_dict({name: value.detach().clone() for name, value in state.items()}, strict=True)
    return head


def _proof_digest(proof):
    return hashlib.sha256(json.dumps(proof, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def _validate_projection(projection, proof):
    _require(callable(projection) and isinstance(proof, Mapping), 'authenticated projection callable and proof required')
    keys = {'original_loss_sha256', 'original_function_ast_sha256', 'prefix_source_sha256',
        'prefix_first_line', 'prefix_last_line', 'derived_ast_sha256', 'modification'}
    _require(set(proof) == keys and proof['original_loss_sha256'] == LOSS_SHA
        and hashlib.sha256(Path(loss_module.__file__).read_bytes()).hexdigest() == LOSS_SHA,
        'unchanged native loss source/projection proof required')
    for name in ('original_function_ast_sha256', 'prefix_source_sha256', 'derived_ast_sha256'):
        value = proof[name]
        _require(type(value) is str and len(value) == 64 and all(c in '0123456789abcdef' for c in value), 'projection SHA256 required')
    _require(type(proof['prefix_first_line']) is int and type(proof['prefix_last_line']) is int
        and 0 < proof['prefix_first_line'] < proof['prefix_last_line']
        and proof['modification'] == 'Return per_candidate immediately before the unchanged detached argmin; all prior statements and signature unchanged.',
        'original projection boundary proof required')


def projected_targets(data, projected_costs_fn, check_deadline=None):
    values, targets = [], []
    with torch.no_grad():
        for start in range(0, len(data), BATCH_SIZE):
            _check(check_deadline)
            index = slice(start, start + BATCH_SIZE)
            args = (data.candidate_xy[index], data.candidate_speed[index], data.original_logits[index],
                data.target_xy[index], data.target_speed_mps[index], data.target_valid[index], loss_module.TrajectoryLossConfig())
            costs = projected_costs_fn(*args, target_yaw=data.target_yaw_rad[index])
            native = loss_module.trajectory_loss(*args, target_yaw=data.target_yaw_rad[index])
            _require(isinstance(costs, Tensor) and costs.shape == data.original_logits[index].shape
                and costs.dtype == torch.float32 and costs.device == data.fused.device and bool(torch.isfinite(costs).all()),
                'finite full native float32 K12 projected costs required')
            oracle = costs.detach().argmin(dim=1)
            _require(torch.equal(oracle, native['oracle_candidate_index']) and torch.equal(
                costs.gather(1, oracle[:, None]).mean().detach(), native['regression_loss']),
                'projected oracle or gathered regression differs from native loss')
            values.append(costs.detach().clone())
            targets.append(oracle.detach().clone())
    return torch.cat(values).contiguous(), torch.cat(targets).contiguous()


def fit_frozen_drive_rank(train, val, *, parent_head_state_dict, parent_logits_cpu, projected_costs_fn,
        projection_proof, arm, seed, device='cpu', callback: Callable[[dict], None] | None = None,
        check_deadline: Callable[[], None] | None = None):
    cache_core.validate_pair(train, val)
    _require(arm in ARMS, 'unknown fixed DRIVE-ranking arm')
    _validate_projection(projected_costs_fn, projection_proof)
    device = _device(device)
    _require(isinstance(parent_logits_cpu, Mapping) and set(parent_logits_cpu) == {'train', 'val'}, 'both complete parent-logit splits required')
    for data in (train, val):
        value = parent_logits_cpu[data.split]
        _require(isinstance(value, Tensor) and tuple(value.shape) == (len(data), 12) and value.device.type == 'cpu'
            and value.dtype == torch.float32 and not value.requires_grad and bool(torch.isfinite(value).all()),
            'complete detached finite CPU float32 parent logits required')
    head = make_parent_head(parent_head_state_dict, seed).to(device)
    parent_head_sha = _digest_tensors(parent_head_state_dict.items())
    parent_logits_sha = _digest_tensors(parent_logits_cpu.items())
    initial_head_sha = _digest_tensors(head.state_dict().items())
    _require(initial_head_sha == parent_head_sha, 'loaded parent parameter bytes differ')
    proof_sha = _proof_digest(projection_proof)
    before = {data.split: data.digest() for data in (train, val)}
    working = {'train': train.to(device), 'val': val.to(device)}
    history, order, baseline_metrics, final_metrics = [], [], {}, {}
    baseline_logits, final_logits, oracles, costs, parity = {}, {}, {}, {}, {}
    cost_digests = {}
    pair_weight = 0 if arm == 'ce_continue' else 1
    try:
        for split, data in working.items():
            result = residual.evaluate_head(data, head, 'candidate_residual', device, check_deadline)
            actual, expected = result.pop('logits_cpu'), parent_logits_cpu[split]
            initial_logits_sha = _digest_tensors([('logits', actual)])
            expected_logits_sha = _digest_tensors([('logits', expected)])
            parity[split] = {'byte_equal': torch.equal(actual, expected) and initial_logits_sha == expected_logits_sha,
                'parent_logits_sha256': expected_logits_sha, 'initial_logits_sha256': initial_logits_sha,
                'max_abs_logit_difference': float((actual-expected).abs().max()),
                'selected_index_mismatch_count': int((actual.argmax(1) != expected.argmax(1)).sum()),
                'scope': 'Same parent residual head over original cached prior/context/geometry; batch4 plus final partial batch, no generator forward.'}
            _require(parity[split]['byte_equal'], 'continued parent head failed exact saved-logit parity')
            baseline_metrics[split] = result
            baseline_logits[split] = expected.detach().clone()
            costs[split], oracles[split] = projected_targets(data, projected_costs_fn, check_deadline)
            cost_digests[split] = _digest_tensors([('projected_costs', costs[split])])
        oracle_digests = {split: _digest_tensors([('composite_oracle', value)]) for split, value in oracles.items()}
        optimizer = torch.optim.AdamW(head.parameters(), lr=LEARNING_RATE, weight_decay=WEIGHT_DECAY)
        step, epoch, samples_seen = 0, 0, 0
        head.train()
        while step < STEPS:
            for batch in cache_core._epoch_batches(len(train), batch_size=BATCH_SIZE, seed=seed, epoch=epoch):
                if step == STEPS: break
                _check(check_deadline)
                index = torch.tensor(batch, device=device, dtype=torch.long)
                data = working['train']
                logits = residual.score_head(head, 'candidate_residual', data.fused[index], data.candidate_xy[index],
                    data.candidate_speed[index], data.original_logits[index])
                score_loss = F.cross_entropy(logits, oracles['train'][index])
                pair_loss = drive_pair_rank_loss(logits, costs['train'][index])
                objective = SCORE_WEIGHT * score_loss
                # HH_260906 - The zero-weight arm keeps the original CE arithmetic exactly; no zero-valued term is added.
                if pair_weight:
                    objective = objective + SCORE_WEIGHT * pair_loss
                _require(bool(torch.isfinite(objective)), 'nonfinite ranking optimization loss')
                optimizer.zero_grad(set_to_none=True)
                objective.backward()
                _require(all(p.grad is not None and bool(torch.isfinite(p.grad).all()) for p in head.parameters()), 'missing/nonfinite ranking gradient')
                norm = torch.nn.utils.clip_grad_norm_(head.parameters(), MAX_GRAD_NORM)
                _require(bool(torch.isfinite(norm)), 'nonfinite ranking gradient norm')
                optimizer.step()
                _require(all(bool(torch.isfinite(p).all()) for p in head.parameters()), 'nonfinite ranking parameters')
                _require(cache_core._nested_tensors_are_finite(optimizer.state_dict()), 'nonfinite ranking optimizer state')
                step += 1
                samples_seen += len(batch)
                order.append({'epoch': epoch, 'indices': list(batch)})
                record = {'global_step': step, 'epoch': epoch, 'batch_samples': len(batch), 'samples_seen': samples_seen,
                    'sample_indices': list(batch), 'candidate_score_loss': float(score_loss.detach()),
                    'drive_pair_rank_loss': float(pair_loss.detach()), 'drive_pair_weight': pair_weight,
                    'optimization_loss': float(objective.detach()), 'gradient_norm': float(norm)}
                history.append(record)
                if callback is not None: callback(copy.deepcopy(record))
            epoch += 1
        _require(step == STEPS and samples_seen == SAMPLES_SEEN, 'fixed continuation step/exposure budget differs')
        for split, data in working.items():
            result = residual.evaluate_head(data, head, 'candidate_residual', device, check_deadline)
            final_logits[split] = result.pop('logits_cpu')
            final_metrics[split] = result
            checked_costs, checked_oracles = projected_targets(data, projected_costs_fn, check_deadline)
            _require(torch.equal(checked_costs, costs[split]) and torch.equal(checked_oracles, oracles[split]), 'frozen projected costs or teacher changed')
        report = {'artifact_id': ARTIFACT_ID, 'status': 'RESEARCH_HEAD_TRAINING_COMPLETE_NOT_PROMOTED',
            'arm': arm, 'seed': seed, 'device': str(device), 'optimizer': 'AdamW', 'optimizer_resumed': False,
            'steps': step, 'batch_size': BATCH_SIZE, 'learning_rate': LEARNING_RATE, 'weight_decay': WEIGHT_DECAY,
            'candidate_score_weight': SCORE_WEIGHT, 'maximum_gradient_norm': MAX_GRAD_NORM,
            'loss_config': loss_module.TrajectoryLossConfig().to_dict(),
            'selector_objective': {'drive_pair_weight': pair_weight, 'auxiliary_coefficient': SCORE_WEIGHT * pair_weight,
                'drive_candidate_indices': list(range(6)), 'unordered_pairs_per_sample': 15,
                'formula': '0.1*hard_composite_CE + 0.1*lambda*mean_rows(sum_pairs(abs(delta_cost)/sum15_abs_gaps * softplus(sign(delta_cost)*delta_logit)))',
                'exact_ties': 'zero weight; all-tied DRIVE row contributes zero; no epsilon, temperature, group filter or sample deletion'},
            'initialization': 'Every arm copies the same fixed parent B residual head; fresh Adam state, not optimizer-state resume or fresh random head.',
            'initial_head_sha256': initial_head_sha, 'final_head_sha256': _digest_tensors(head.state_dict().items()),
            'parent_head_sha256_before': parent_head_sha, 'parent_head_sha256_after': parent_head_sha,
            'parent_logits_sha256_before': parent_logits_sha, 'parent_logits_sha256_after': parent_logits_sha,
            'parent_logit_parity': parity, 'cache_sha256_before': before, 'cache_sha256_after': dict(before),
            'projection_proof': copy.deepcopy(dict(projection_proof)), 'projection_proof_sha256': proof_sha,
            'projection_scope': 'Worker authenticates helper source and exact native AST-prefix derivation. Core separately checks full K12 shape/finite values, native argmin and gathered regression each batch; these consistency checks alone do not prove every off-oracle cost.',
            'projected_cost_sha256': cost_digests, 'composite_oracle_sha256': oracle_digests,
            'sampling_policy': 'Original uniform_without_replacement CPU seed+epoch randperm; same ordered full TRAIN batches in both arms.',
            'sampling_order_sha256': hashlib.sha256(json.dumps(order, separators=(',', ':')).encode()).hexdigest(),
            'baseline_metrics': baseline_metrics, 'pre_training_metrics': copy.deepcopy(baseline_metrics),
            'post_training_metrics': final_metrics, 'history': history,
            'final_state': {'global_step': step, 'samples_seen': samples_seen, 'last_epoch_index': history[-1]['epoch']},
            'train_sample_ids': list(train.sample_ids), 'val_sample_ids': list(val.sample_ids),
            'source_checkpoint_sha256': train.source_checkpoint_sha256, 'corpus_fingerprint_sha256': train.corpus_fingerprint_sha256,
            'split_fingerprints': {split: data.dataset_fingerprint_sha256 for split, data in working.items()},
            'head_parameter_count': sum(p.numel() for p in head.parameters()), 'candidate_count': 12,
            'generator_parameters_trained': False, 'original_heads_recomputed_or_trained': False,
            'training_data_approved': False, 'vehicle_control_approved': False, 'automatic_promotion': False,
            'runtime_bundle_supported': False,
            'interpretation': 'Isolated objective comparison, not a new generator or traffic-intent model. Direct auxiliary gradients use DRIVE logits only; shared weights may still change STOP scores. Stationary behavior is not guaranteed. Future-derived groups remain diagnostics and identical-input conflicts persist.'}
        return {'report': report, 'head_state_dict': {name: value.detach().cpu().clone() for name, value in head.state_dict().items()},
            'final_logits_cpu': final_logits, 'baseline_logits_cpu': baseline_logits,
            'oracle_indices_cpu': {split: value.detach().cpu().clone() for split, value in oracles.items()}}
    finally:
        _require(train.digest() == before['train'] and val.digest() == before['val'], 'original cache mutated during ranking')
        _require(all(data.digest() == before[split] for split, data in working.items()), 'working cache mutated during ranking')
        _require(_digest_tensors(parent_head_state_dict.items()) == parent_head_sha, 'parent head state mutated')
        _require(_digest_tensors(parent_logits_cpu.items()) == parent_logits_sha, 'parent logits mutated')
        _require(_proof_digest(projection_proof) == proof_sha, 'projection proof mutated')
        _require(all(_digest_tensors([('projected_costs', value)]) == cost_digests[split]
            for split, value in costs.items() if split in cost_digests), 'working projected costs mutated')
