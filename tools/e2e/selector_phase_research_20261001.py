"""HH_261001 - Frozen-generator selector phase weighting with full TRAIN coverage and unchanged oracle targets."""

import random

import torch
from torch.nn import functional as F


PHASES = ("stop_or_full_hold", "low_speed_immediate_start", "low_speed_other", "moving")
HEAD_NAMES = ("candidate_head.weight", "candidate_head.bias", "stop_candidate_head.weight", "stop_candidate_head.bias")


def schedule(count, positions=12000, batch_size=4, seed=20260906):
    if count < batch_size or positions < count or positions % batch_size:
        raise ValueError("coverable nonempty fixed-batch schedule required")
    rng, order = random.Random(seed), []
    while len(order) < positions:
        cycle = list(range(count))
        rng.shuffle(cycle)
        remainder = len(order) % batch_size
        if remainder:
            blocked = set(order[-remainder:])
            cycle = [i for i in cycle if i not in blocked] + [i for i in cycle if i in blocked]
        order.extend(cycle[:positions-len(order)])
    batches = [tuple(order[start:start+batch_size]) for start in range(0, positions, batch_size)]
    if len(set(order)) != count or any(len(set(batch)) != batch_size for batch in batches):
        raise ValueError("schedule lost coverage or repeated within a batch")
    return batches


def phase_ids(current, target, valid, oracle):
    if (current.ndim != 1 or target.shape != valid.shape or target.shape != (len(current), 64)
            or valid.dtype != torch.bool or oracle.shape != current.shape or oracle.dtype != torch.long
            or not bool(valid.any(1).all()) or not bool(torch.isfinite(current).all())
            or not bool(torch.isfinite(target).all()) or bool(((oracle < 0) | (oracle >= 12)).any())):
        raise ValueError("finite original K12 TRAIN target/current/mask/oracle tensors required")
    full_hold = ((target <= .1) | ~valid).all(1)
    stop = (oracle >= 6) | full_hold
    low = current.abs() <= .5
    start = low & valid[:, :3].all(1) & (target[:, 2] > current.clamp_min(0.) + .1)
    result = torch.full_like(oracle, 3)
    result[low] = 2
    result[start] = 1
    result[stop] = 0
    return result


def phase_weights(phases, batches):
    if phases.ndim != 1 or phases.dtype != torch.long or bool(((phases < 0) | (phases >= 4)).any()):
        raise ValueError("four fixed phase IDs required")
    indices = torch.tensor([index for batch in batches for index in batch], dtype=torch.long, device=phases.device)
    if set(indices.tolist()) != set(range(len(phases))):
        raise ValueError("all TRAIN identities must receive exposure")
    counts = torch.bincount(phases[indices], minlength=4)
    if not bool((counts > 0).all()):
        raise ValueError("all four predefined phase strata must be present")
    # HH_261001 - Fixed half ordinary mean plus half equal-stratum mean; original STOP losses remain positive.
    weights = .5 + .5 * len(indices) / (4. * counts.to(torch.float32))
    if not bool((weights >= .5).all()) or abs(float(weights[phases[indices]].mean()) - 1.) > 1.e-6:
        raise ValueError("phase mixture mass or minimum weight differs")
    return weights, counts


def selector_loss(logits, costs, weights):
    if (logits.ndim != 2 or logits.shape[1] != 12 or costs.shape != logits.shape
            or weights.shape != logits.shape[:1] or not bool(torch.isfinite(logits).all())
            or not bool(torch.isfinite(costs).all()) or not bool(torch.isfinite(weights).all())
            or not bool((weights > 0).all())):
        raise ValueError("finite weighted full-K12 selector tensors required")
    detached = costs.detach()
    oracle = detached.argmin(1)
    regret = detached - detached.min(1, keepdim=True).values
    per_row = .1 * F.cross_entropy(logits, oracle, reduction="none")
    per_row = per_row + .1 * (logits.softmax(1) * regret).sum(1)
    return (per_row * weights.detach()).mean()


def generator_state(state):
    return {key: value for key, value in state.items() if key not in HEAD_NAMES}
