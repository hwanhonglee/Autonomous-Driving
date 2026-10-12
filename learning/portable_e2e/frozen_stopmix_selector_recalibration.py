"""HH_260906 - Recalibrate only the frozen F selector under paired replay identities."""

from __future__ import annotations

import copy
import hashlib
import json
from typing import Any, Callable, Mapping, Sequence

import torch
from torch import Tensor, nn
from torch.nn import functional as F

from . import frozen_stopmix_drive_rank as projected
from . import frozen_stopmix_selector as cache_core
from . import losses as loss_module
from .contract import ContractError
from .train import (
    DIRECTION_BALANCED_REPLAY_SAMPLING_POLICY,
    TrainConfig,
    _canonical_sha256,
    _cohort_replay_epoch_occurrences,
    _nested_tensors_are_finite,
)


ARTIFACT_ID = "portable_e2e.frozen_f_selector_sampling_recalibration.v1"
ARMS = ("H0_f_right_replay", "H1_train_cyclic_uniform_replay")
SOURCE_CHECKPOINT_SHA256 = (
    "a551570aa6fc1697e9dcfd9fa714b2c1066f95e2194a5fefa9b0d9f3c20baf8c"
)
F_SAMPLING_PLAN_SHA256 = (
    "abb0e655b174844cc20f293bc12c5156fe33b3a00741ac1cfe8b703367e41c7b"
)
TRAIN_COUNT, DEV_COUNT = 1475, 665
EPOCHS, BATCH_SIZE, STEPS, SAMPLES_SEEN = 20, 4, 8280, 33120
BASE_EXPOSURES, EXTRA_EXPOSURES = 29500, 3620
BASE_PER_EPOCH, EXTRA_PER_EPOCH = 1475, 181
LEARNING_RATE, WEIGHT_DECAY = 1.0e-4, 1.0e-4
CE_WEIGHT, EXPECTED_REGRET_WEIGHT, MAX_GRAD_NORM = 0.1, 0.1, 5.0
HEAD_PARAMETER_COUNT = 3084
CachedStopmixSelectorData = cache_core.CachedStopmixSelectorData
_require, _check = cache_core._require, cache_core._check
_device, _digest_tensors = cache_core._device, cache_core._digest_tensors


def validate_pair(train: CachedStopmixSelectorData, dev: CachedStopmixSelectorData) -> None:
    """HH_260906 - Admit only the fixed augmented TRAIN and two-episode Town03 DEV caches."""
    _require(
        type(train) is CachedStopmixSelectorData
        and type(dev) is CachedStopmixSelectorData,
        "validated K12 cache objects required",
    )
    _require(train.split == "train" and dev.split == "val", "TRAIN and DEV caches required")
    _require(
        len(train) == TRAIN_COUNT
        and len(dev) == DEV_COUNT
        and len(set(train.episode_ids)) == 4
        and len(set(dev.episode_ids)) == 2,
        "complete TRAIN1475/four-episode and DEV665/two-episode caches required",
    )
    _require(
        not (set(train.sample_ids) & set(dev.sample_ids))
        and not (set(train.episode_ids) & set(dev.episode_ids)),
        "TRAIN/DEV sample or episode leakage",
    )
    _require(
        train.source_checkpoint_sha256
        == dev.source_checkpoint_sha256
        == SOURCE_CHECKPOINT_SHA256,
        "exact F source checkpoint required",
    )
    _require(
        train.corpus_fingerprint_sha256 == dev.corpus_fingerprint_sha256
        and train.dataset_fingerprint_sha256 != dev.dataset_fingerprint_sha256,
        "corpus or split fingerprints disagree",
    )


def clone_f_heads(drive: nn.Module, stop: nn.Module) -> cache_core.TwinLinearHeads:
    """HH_260906 - Copy F's two six-way Linear heads without consuming RNG state."""
    result = cache_core.clone_original_heads(drive, stop)
    result.requires_grad_(True)
    for parameter in result.parameters():
        parameter.grad = None
    return result.train()


def score_head(head: cache_core.TwinLinearHeads, fused: Tensor) -> Tensor:
    _require(
        type(head) is cache_core.TwinLinearHeads
        and isinstance(fused, Tensor)
        and fused.ndim == 2
        and fused.shape[1] == 256
        and fused.dtype in (torch.float32, torch.float64)
        and bool(torch.isfinite(fused).all()),
        "finite N-by-256 context and exact twin Linear heads required",
    )
    _device(fused.device)
    _require(
        all(parameter.device == fused.device for parameter in head.parameters()),
        "selector and cached context devices differ",
    )
    # HH_260906 - Context is immutable; only the original candidate and stop head parameters receive gradients.
    return head(fused.detach())


def expected_regret_loss(logits: Tensor, costs: Tensor) -> Tensor:
    _require(
        isinstance(logits, Tensor)
        and isinstance(costs, Tensor)
        and logits.ndim == 2
        and logits.shape == costs.shape
        and logits.shape[1] == 12
        and logits.dtype in (torch.float32, torch.float64)
        and costs.dtype == logits.dtype
        and costs.device == logits.device
        and bool(torch.isfinite(logits).all())
        and bool(torch.isfinite(costs).all()),
        "finite same-device K12 logits and composite costs required",
    )
    _device(logits.device)
    # HH_260906 - Exact frozen composite costs provide logit gradients only, matching the F auxiliary definition.
    regret = costs.detach() - costs.detach().min(dim=1, keepdim=True).values
    result = (logits.softmax(dim=1) * regret).sum(dim=1).mean()
    _require(bool(torch.isfinite(result)), "nonfinite expected-regret loss")
    return result


def _validate_sampling_contract(
    sampling_plan: Mapping[str, Any], train_config: TrainConfig
) -> None:
    _require(
        isinstance(sampling_plan, Mapping)
        and _canonical_sha256(sampling_plan) == F_SAMPLING_PLAN_SHA256,
        "exact F direction-balanced sampling plan required",
    )
    expected = {
        "seed": 20260906,
        "batch_size": BATCH_SIZE,
        "learning_rate": LEARNING_RATE,
        "weight_decay": WEIGHT_DECAY,
        "max_steps": STEPS,
        "maximum_gradient_norm": MAX_GRAD_NORM,
        "sampling_policy": DIRECTION_BALANCED_REPLAY_SAMPLING_POLICY,
    }
    _require(
        isinstance(train_config, TrainConfig)
        and all(getattr(train_config, name) == value for name, value in expected.items())
        and train_config.domain_ratios == (),
        "exact F seed/batch/optimizer/replay TrainConfig required",
    )


def _cyclic_epoch_identities(epoch: int) -> list[int]:
    start = (epoch * EXTRA_PER_EPOCH) % TRAIN_COUNT
    return [(start + offset) % TRAIN_COUNT for offset in range(EXTRA_PER_EPOCH)]


def _match_uniform_identities(
    h0: Sequence[Mapping[str, Any]], epoch: int
) -> dict[int, int]:
    """HH_260906 - Preserve the exact cyclic multiset while matching it to duplicate-free replay slots."""
    replay_positions = [
        position
        for position, row in enumerate(h0)
        if row["sampling_stream"] == "replay"
    ]
    identities = _cyclic_epoch_identities(epoch)
    _require(
        len(replay_positions) == len(identities) == EXTRA_PER_EPOCH,
        "paired replay-slot count differs",
    )
    forbidden = {}
    for position in replay_positions:
        start = (position // BATCH_SIZE) * BATCH_SIZE
        forbidden[position] = {
            int(row["dataset_index"])
            for row in h0[start : start + BATCH_SIZE]
            if row["sampling_stream"] == "base"
        }

    slot_for_identity: dict[int, int] = {}

    def assign(position: int, visited: set[int]) -> bool:
        offset = replay_positions.index(position)
        for shift in range(len(identities)):
            identity = identities[(offset + shift) % len(identities)]
            if identity in visited or identity in forbidden[position]:
                continue
            visited.add(identity)
            previous = slot_for_identity.get(identity)
            if previous is None or assign(previous, visited):
                slot_for_identity[identity] = position
                return True
        return False

    for position in replay_positions:
        _require(assign(position, set()), "cyclic-uniform identities cannot satisfy batch uniqueness")
    position_to_identity = {position: identity for identity, position in slot_for_identity.items()}
    _require(
        len(position_to_identity) == EXTRA_PER_EPOCH
        and sorted(position_to_identity.values()) == sorted(identities),
        "cyclic-uniform matching changed its identity multiset",
    )
    return position_to_identity


def paired_epoch_occurrences(
    sampling_plan: Mapping[str, Any], train_config: TrainConfig, *, epoch: int
) -> dict[str, tuple[dict[str, Any], ...]]:
    _validate_sampling_contract(sampling_plan, train_config)
    _require(type(epoch) is int and 0 <= epoch < EPOCHS, "epoch is outside the fixed H study")
    original = tuple(
        dict(row)
        for row in _cohort_replay_epoch_occurrences(
            sampling_plan, train_config, epoch=epoch
        )
    )
    _require(
        len(original) == BASE_PER_EPOCH + EXTRA_PER_EPOCH
        and sum(row["sampling_stream"] == "base" for row in original)
        == BASE_PER_EPOCH
        and sum(row["sampling_stream"] == "replay" for row in original)
        == EXTRA_PER_EPOCH,
        "F epoch occurrence accounting differs",
    )
    uniform_map = _match_uniform_identities(original, epoch)
    uniform = []
    for position, row in enumerate(original):
        value = dict(row)
        if value["sampling_stream"] == "replay":
            value["dataset_index"] = uniform_map[position]
        uniform.append(value)
    for rows in (original, uniform):
        _require(
            all(
                len(
                    {
                        int(row["dataset_index"])
                        for row in rows[start : start + BATCH_SIZE]
                    }
                )
                == len(rows[start : start + BATCH_SIZE])
                for start in range(0, len(rows), BATCH_SIZE)
            ),
            "paired schedule contains a duplicate dataset index within one batch",
        )
    _require(
        [row for row in original if row["sampling_stream"] == "base"]
        == [row for row in uniform if row["sampling_stream"] == "base"]
        and [row["sampling_stream"] for row in original]
        == [row["sampling_stream"] for row in uniform],
        "paired arm changed base order or stream positions",
    )
    return {ARMS[0]: original, ARMS[1]: tuple(uniform)}


def paired_schedule(
    sampling_plan: Mapping[str, Any], train_config: TrainConfig
) -> dict[str, Any]:
    arms = {arm: [] for arm in ARMS}
    for epoch in range(EPOCHS):
        pair = paired_epoch_occurrences(sampling_plan, train_config, epoch=epoch)
        for arm in ARMS:
            arms[arm].append(pair[arm])
    uniform_replay = [
        int(row["dataset_index"])
        for epochs in (arms[ARMS[1]],)
        for rows in epochs
        for row in rows
        if row["sampling_stream"] == "replay"
    ]
    counts = torch.bincount(torch.tensor(uniform_replay), minlength=TRAIN_COUNT)
    _require(
        len(uniform_replay) == EXTRA_EXPOSURES
        and int(counts.min()) == 2
        and int(counts.max()) == 3
        and int((counts == 3).sum()) == EXTRA_EXPOSURES - 2 * TRAIN_COUNT,
        "H1 cyclic-uniform exposure distribution differs",
    )
    base_projection = {
        arm: [
            {
                "epoch": epoch,
                "position": position,
                "dataset_index": int(row["dataset_index"]),
            }
            for epoch, rows in enumerate(arms[arm])
            for position, row in enumerate(rows)
            if row["sampling_stream"] == "base"
        ]
        for arm in ARMS
    }
    _require(
        base_projection[ARMS[0]] == base_projection[ARMS[1]],
        "paired base occurrence positions differ",
    )
    payload = {
        arm: [[dict(row) for row in rows] for rows in epochs]
        for arm, epochs in arms.items()
    }
    return {
        "arms": arms,
        "schedule_sha256": {
            arm: hashlib.sha256(
                json.dumps(payload[arm], sort_keys=True, separators=(",", ":")).encode()
            ).hexdigest()
            for arm in ARMS
        },
        "base_position_order_sha256": hashlib.sha256(
            json.dumps(
                base_projection[ARMS[0]], sort_keys=True, separators=(",", ":")
            ).encode()
        ).hexdigest(),
        "h1_extra_identity_counts": counts.tolist(),
    }


def _head_logits(
    data: CachedStopmixSelectorData,
    head: cache_core.TwinLinearHeads,
    *,
    check_deadline: Callable[[], None] | None,
) -> Tensor:
    rows = []
    with torch.no_grad():
        for start in range(0, len(data), BATCH_SIZE):
            _check(check_deadline)
            rows.append(score_head(head, data.fused[start : start + BATCH_SIZE]))
    return torch.cat(rows)


def fit_recalibrated_selector(
    train: CachedStopmixSelectorData,
    dev: CachedStopmixSelectorData,
    *,
    original_drive_head: nn.Module,
    original_stop_head: nn.Module,
    sampling_plan: Mapping[str, Any],
    train_config: TrainConfig,
    projected_costs_fn: Callable[..., Tensor],
    projection_proof: Mapping[str, Any],
    arm: str,
    device: str | torch.device = "cpu",
    callback: Callable[[dict[str, Any]], None] | None = None,
    check_deadline: Callable[[], None] | None = None,
) -> dict[str, Any]:
    validate_pair(train, dev)
    _require(arm in ARMS, "unknown paired H arm")
    projected._validate_projection(projected_costs_fn, projection_proof)
    schedule = paired_schedule(sampling_plan, train_config)
    device = _device(device)
    before = {data.split: data.digest() for data in (train, dev)}
    working = {"train": train.to(device), "val": dev.to(device)}
    parent = {
        "drive": cache_core._head_digest(original_drive_head),
        "stop": cache_core._head_digest(original_stop_head),
    }
    head = clone_f_heads(original_drive_head, original_stop_head).to(device)
    initial_head_sha256 = _digest_tensors(head.state_dict().items())
    _require(
        sum(parameter.numel() for parameter in head.parameters())
        == HEAD_PARAMETER_COUNT,
        "only the 3084 F selector parameters may be trainable",
    )
    history: list[dict[str, Any]] = []
    costs: dict[str, Tensor] = {}
    oracles: dict[str, Tensor] = {}
    parity: dict[str, Any] = {}
    try:
        # HH_260906 - DEV is checked only for byte-parity here; no pre-fit DEV metric or selection is computed.
        for split, data in working.items():
            actual = _head_logits(data, head, check_deadline=check_deadline)
            expected = data.original_logits
            parity[split] = {
                "byte_equal": torch.equal(actual, expected),
                "max_abs_logit_difference": float((actual - expected).abs().max()),
                "selected_index_mismatch_count": int(
                    (actual.argmax(1) != expected.argmax(1)).sum()
                ),
            }
            _require(parity[split]["byte_equal"], "F head/cached-logit parity failed")
        costs["train"], oracles["train"] = projected.projected_targets(
            working["train"], projected_costs_fn, check_deadline
        )
        cost_sha256 = {
            split: _digest_tensors([("projected_costs", value)])
            for split, value in costs.items()
        }
        oracle_sha256 = {
            split: _digest_tensors([("composite_oracle", value)])
            for split, value in oracles.items()
        }
        optimizer = torch.optim.AdamW(
            head.parameters(), lr=LEARNING_RATE, weight_decay=WEIGHT_DECAY
        )
        step = samples_seen = base_seen = extra_seen = 0
        for epoch, occurrences in enumerate(schedule["arms"][arm]):
            for batch_index, start in enumerate(
                range(0, len(occurrences), BATCH_SIZE)
            ):
                _check(check_deadline)
                batch = occurrences[start : start + BATCH_SIZE]
                indices = [int(row["dataset_index"]) for row in batch]
                _require(
                    len(indices) == BATCH_SIZE
                    and len(set(indices)) == BATCH_SIZE,
                    "fixed H batch is partial or contains duplicate indices",
                )
                index = torch.tensor(indices, device=device, dtype=torch.long)
                logits = score_head(head, working["train"].fused[index])
                ce = F.cross_entropy(logits, oracles["train"][index])
                regret = expected_regret_loss(logits, costs["train"][index])
                objective = CE_WEIGHT * ce + EXPECTED_REGRET_WEIGHT * regret
                _require(bool(torch.isfinite(objective)), "nonfinite H objective")
                optimizer.zero_grad(set_to_none=True)
                objective.backward()
                _require(
                    all(
                        parameter.grad is not None
                        and bool(torch.isfinite(parameter.grad).all())
                        for parameter in head.parameters()
                    ),
                    "missing or nonfinite selector gradient",
                )
                norm = torch.nn.utils.clip_grad_norm_(
                    head.parameters(), MAX_GRAD_NORM
                )
                _require(bool(torch.isfinite(norm)), "nonfinite selector gradient norm")
                optimizer.step()
                _require(
                    all(bool(torch.isfinite(parameter).all()) for parameter in head.parameters())
                    and _nested_tensors_are_finite(optimizer.state_dict()),
                    "nonfinite selector parameter or optimizer state",
                )
                step += 1
                samples_seen += BATCH_SIZE
                base_count = sum(row["sampling_stream"] == "base" for row in batch)
                base_seen += base_count
                extra_seen += BATCH_SIZE - base_count
                record = {
                    "global_step": step,
                    "epoch": epoch,
                    "batch_index": batch_index,
                    "sample_indices": indices,
                    "sample_tokens": [train.sample_ids[index] for index in indices],
                    "sampling_streams": [row["sampling_stream"] for row in batch],
                    "samples_seen": samples_seen,
                    "base_samples_seen": base_seen,
                    "extra_samples_seen": extra_seen,
                    "candidate_score_loss": float(ce.detach()),
                    "candidate_regret_loss": float(regret.detach()),
                    "optimization_loss": float(objective.detach()),
                    "gradient_norm": float(norm),
                }
                history.append(record)
                if callback is not None:
                    callback(copy.deepcopy(record))
        _require(
            (step, samples_seen, base_seen, extra_seen)
            == (STEPS, SAMPLES_SEEN, BASE_EXPOSURES, EXTRA_EXPOSURES),
            "fixed H step or exposure accounting differs",
        )
        head.eval()
        final_train_logits = _head_logits(
            working["train"], head, check_deadline=check_deadline
        )
        final_dev_logits = _head_logits(
            working["val"], head, check_deadline=check_deadline
        )
        # HH_260906 - This is the sole DEV performance evaluation and occurs only after all fixed updates.
        costs["val"], oracles["val"] = projected.projected_targets(
            working["val"], projected_costs_fn, check_deadline
        )
        cost_sha256["val"] = _digest_tensors(
            [("projected_costs", costs["val"])]
        )
        oracle_sha256["val"] = _digest_tensors(
            [("composite_oracle", oracles["val"])]
        )
        final_dev_metrics = cache_core.evaluate_logits(
            working["val"], final_dev_logits, check_deadline
        )
        checked_costs, checked_oracles = projected.projected_targets(
            working["train"], projected_costs_fn, check_deadline
        )
        _require(
            torch.equal(checked_costs, costs["train"])
            and torch.equal(checked_oracles, oracles["train"]),
            "frozen TRAIN costs or oracle changed during H fitting",
        )
        report = {
            "artifact_id": ARTIFACT_ID,
            "status": "HEAD_ONLY_FIT_COMPLETE_NOT_PROMOTED",
            "arm": arm,
            "source_checkpoint_sha256": SOURCE_CHECKPOINT_SHA256,
            "device": str(device),
            "optimizer": "AdamW",
            "optimizer_resumed": False,
            "epochs": EPOCHS,
            "steps": step,
            "batch_size": BATCH_SIZE,
            "samples_seen": samples_seen,
            "base_samples_seen": base_seen,
            "extra_samples_seen": extra_seen,
            "learning_rate": LEARNING_RATE,
            "weight_decay": WEIGHT_DECAY,
            "maximum_gradient_norm": MAX_GRAD_NORM,
            "objective": "0.1 * hard composite-oracle CE + 0.1 * softmax expected detached composite regret",
            "candidate_score_weight": CE_WEIGHT,
            "candidate_regret_weight": EXPECTED_REGRET_WEIGHT,
            "loss_config": loss_module.TrajectoryLossConfig(
                candidate_regret_weight=EXPECTED_REGRET_WEIGHT
            ).to_dict(),
            "initialization": "Exact F candidate_head and stop_candidate_head weights; fresh AdamW state per arm.",
            "initial_head_sha256": initial_head_sha256,
            "final_head_sha256": _digest_tensors(head.state_dict().items()),
            "parent_head_sha256_before": parent,
            "parent_head_sha256_after": dict(parent),
            "head_parameter_count": HEAD_PARAMETER_COUNT,
            "trainable_parameter_names": [
                "candidate_head.weight",
                "candidate_head.bias",
                "stop_candidate_head.weight",
                "stop_candidate_head.bias",
            ],
            "f_head_logit_parity": parity,
            "cache_sha256_before": before,
            "cache_sha256_after": dict(before),
            "projected_cost_sha256": cost_sha256,
            "composite_oracle_sha256": oracle_sha256,
            "f_sampling_plan_sha256": F_SAMPLING_PLAN_SHA256,
            "arm_schedule_sha256": schedule["schedule_sha256"][arm],
            "paired_base_position_order_sha256": schedule[
                "base_position_order_sha256"
            ],
            "h1_extra_identity_histogram": schedule["h1_extra_identity_counts"],
            "train_index_token_sha256": hashlib.sha256(
                json.dumps(
                    [
                        {"dataset_index": index, "sample_token": token}
                        for index, token in enumerate(train.sample_ids)
                    ],
                    sort_keys=True,
                    separators=(",", ":"),
                ).encode()
            ).hexdigest(),
            "core_full_dev_metric_evaluation_count": 1,
            "dev_evaluated_only_after_final_step": True,
            "dev_used_for_training": False,
            "dev_used_for_intermediate_selection": False,
            "test_read_or_used": False,
            "generator_parameters_trained": False,
            "candidate_geometry_recomputed_during_fit": False,
            "new_function_head_added": False,
            "automatic_promotion": False,
            "runtime_checkpoint_exported": False,
            "vehicle_control_approved": False,
            "history": history,
            "final_dev_metrics": final_dev_metrics,
            "final_state": {
                "epoch": EPOCHS,
                "next_batch_index": 0,
                "global_step": step,
                "samples_seen": samples_seen,
            },
            "interpretation": "Paired selector-only sampling diagnostic. It cannot establish closed-loop, TEST, all-map, export, or vehicle readiness.",
        }
        return {
            "report": report,
            "head_state_dict": {
                name: value.detach().cpu().clone()
                for name, value in head.state_dict().items()
            },
            "final_logits_cpu": {
                "train": final_train_logits.detach().cpu().clone(),
                "val": final_dev_logits.detach().cpu().clone(),
            },
            "oracle_indices_cpu": {
                split: value.detach().cpu().clone()
                for split, value in oracles.items()
            },
        }
    finally:
        _require(
            train.digest() == before["train"] and dev.digest() == before["val"],
            "original cache mutated during H fitting",
        )
        _require(
            all(data.digest() == before[split] for split, data in working.items()),
            "working cache mutated during H fitting",
        )
        _require(
            cache_core._head_digest(original_drive_head) == parent["drive"]
            and cache_core._head_digest(original_stop_head) == parent["stop"],
            "original F heads mutated during H fitting",
        )
