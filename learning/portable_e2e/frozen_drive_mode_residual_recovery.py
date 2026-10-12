"""HH_260906 - Diagnose DRIVE-only residual reranking on immutable F caches."""

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
from . import frozen_stopmix_selector_recalibration as h_core
from .model import PHYSICAL_MAXIMUM_SPEED_MPS
from .train import _nested_tensors_are_finite


ARTIFACT_ID = "portable_e2e.frozen_f_drive_mode_residual_recovery.v1"
ARMS = ("I0_prior_context_residual", "I1_candidate_geometry_residual")
SOURCE_CHECKPOINT_SHA256 = h_core.SOURCE_CHECKPOINT_SHA256
H1_HISTORY_SHA256 = (
    "0bdbfeff48187e804c2639f9838674a398461c965bcbfb1ddac635f576f5cf92"
)
SEED = 20260906
TRAIN_COUNT, DEV_COUNT = 1475, 665
EPOCHS, BATCH_SIZE, STEPS, SAMPLES_SEEN = 20, 4, 8280, 33120
BASE_EXPOSURES, EXTRA_EXPOSURES = 29500, 3620
BASE_PER_EPOCH, EXTRA_PER_EPOCH = 1475, 181
LEARNING_RATE, WEIGHT_DECAY = 1.0e-4, 1.0e-4
CE_WEIGHT, EXPECTED_REGRET_WEIGHT, MAX_GRAD_NORM = 0.1, 0.1, 5.0
DRIVE_CANDIDATE_COUNT, TOTAL_CANDIDATE_COUNT = 6, 12
HEAD_PARAMETER_COUNT = 115457
CachedStopmixSelectorData = cache_core.CachedStopmixSelectorData
_require, _check = cache_core._require, cache_core._check
_device, _digest_tensors = cache_core._device, cache_core._digest_tensors


def validate_pair(train: CachedStopmixSelectorData, dev: CachedStopmixSelectorData) -> None:
    """HH_260906 - Reuse the sealed F TRAIN1475 and Town03 DEV665 cache contract."""
    h_core.validate_pair(train, dev)


def make_head(arm: str, seed: int = SEED) -> nn.Sequential:
    """HH_260906 - Initialize paired 449-256-1 residual selectors without caller RNG changes."""
    _require(
        arm in ARMS and type(seed) is int and 0 <= seed < 2**63,
        "known I arm and valid CPU initialization seed required",
    )
    _require(
        torch.get_default_dtype() == torch.float32,
        "fixed float32 initialization is required",
    )
    with torch.random.fork_rng(devices=[]):
        torch.random.default_generator.manual_seed(seed)
        head = nn.Sequential(
            cache_core._cpu_linear(449, 256),
            nn.ReLU(inplace=True),
            cache_core._cpu_linear(256, 1),
        )
        nn.init.zeros_(head[-1].weight)
        nn.init.zeros_(head[-1].bias)
    _require(
        sum(parameter.numel() for parameter in head.parameters())
        == HEAD_PARAMETER_COUNT,
        "fixed 115457-parameter selector architecture required",
    )
    return head


def _validate_score_inputs(
    head: nn.Module,
    arm: str,
    fused: Tensor,
    candidate_xy: Tensor,
    candidate_speed: Tensor,
    original_logits: Tensor,
) -> None:
    values = (fused, candidate_xy, candidate_speed, original_logits)
    _require(
        arm in ARMS
        and isinstance(head, nn.Sequential)
        and all(isinstance(value, Tensor) for value in values),
        "known I arm, residual head and tensors required",
    )
    count = len(fused)
    _require(
        count > 0
        and tuple(fused.shape) == (count, 256)
        and tuple(candidate_xy.shape) == (count, 12, 64, 2)
        and tuple(candidate_speed.shape) == (count, 12, 64)
        and tuple(original_logits.shape) == (count, 12),
        "fixed F cache scoring shapes required",
    )
    _require(
        fused.dtype in (torch.float32, torch.float64)
        and all(
            value.dtype == fused.dtype
            and value.device == fused.device
            and bool(torch.isfinite(value).all())
            for value in values
        ),
        "finite same-device and same-dtype F cache tensors required",
    )
    _device(fused.device)
    _require(
        all(
            parameter.device == fused.device and parameter.dtype == fused.dtype
            for parameter in head.parameters()
        ),
        "residual head and cache device or dtype differ",
    )


def score_drive_logits(
    head: nn.Module,
    arm: str,
    fused: Tensor,
    candidate_xy: Tensor,
    candidate_speed: Tensor,
    original_logits: Tensor,
) -> Tensor:
    """HH_260906 - Score only DRIVE candidates while keeping every F input immutable."""
    _validate_score_inputs(
        head, arm, fused, candidate_xy, candidate_speed, original_logits
    )
    context = fused.detach().unsqueeze(1).expand(-1, DRIVE_CANDIDATE_COUNT, -1)
    prior = original_logits.detach()[:, :DRIVE_CANDIDATE_COUNT]
    geometry = torch.cat(
        (
            candidate_xy.detach()[:, :DRIVE_CANDIDATE_COUNT].flatten(2) / 120.0,
            candidate_speed.detach()[:, :DRIVE_CANDIDATE_COUNT]
            / PHYSICAL_MAXIMUM_SPEED_MPS,
        ),
        dim=2,
    )
    if arm == ARMS[0]:
        geometry = torch.zeros_like(geometry)
    residual = head(torch.cat((context, prior.unsqueeze(-1), geometry), dim=2))
    residual = residual.squeeze(-1)
    result = prior + residual
    _require(
        tuple(result.shape) == (len(fused), DRIVE_CANDIDATE_COUNT)
        and bool(torch.isfinite(result).all()),
        "finite N-by-6 DRIVE scores required",
    )
    return result


def score_mode_preserving_logits(
    head: nn.Module,
    arm: str,
    fused: Tensor,
    candidate_xy: Tensor,
    candidate_speed: Tensor,
    original_logits: Tensor,
) -> Tensor:
    """HH_260906 - Rerank within DRIVE while preserving F's exact mode boundary."""
    raw_drive = score_drive_logits(
        head, arm, fused, candidate_xy, candidate_speed, original_logits
    )
    original = original_logits.detach()
    prior_drive = original[:, :DRIVE_CANDIDATE_COUNT]
    original_selected = original.argmax(dim=1)
    original_stop = original_selected >= DRIVE_CANDIDATE_COUNT
    normalized_drive = (
        raw_drive
        - raw_drive.max(dim=1, keepdim=True).values
        + prior_drive.max(dim=1, keepdim=True).values
    )
    unchanged_drive = (raw_drive == prior_drive).all(dim=1, keepdim=True)
    normalized_drive = torch.where(
        unchanged_drive, prior_drive, normalized_drive
    )
    drive_mode_logits = torch.cat(
        (normalized_drive, original[:, DRIVE_CANDIDATE_COUNT:]), dim=1
    )
    result = torch.where(original_stop.unsqueeze(1), original, drive_mode_logits)
    selected = result.argmax(dim=1)
    _require(
        tuple(result.shape) == tuple(original.shape)
        and bool(torch.isfinite(result).all())
        and not bool(((selected >= DRIVE_CANDIDATE_COUNT) != original_stop).any()),
        "DRIVE/STOP mode preservation failed",
    )
    _require(
        torch.equal(result[original_stop], original[original_stop]),
        "original STOP rows must retain exact F logits",
    )
    return result


def expected_drive_regret_loss(logits: Tensor, costs: Tensor) -> Tensor:
    """HH_260906 - Optimize detached projected regret over the six DRIVE choices only."""
    _require(
        isinstance(logits, Tensor)
        and isinstance(costs, Tensor)
        and logits.ndim == 2
        and logits.shape == costs.shape
        and logits.shape[1] == DRIVE_CANDIDATE_COUNT
        and logits.dtype in (torch.float32, torch.float64)
        and costs.dtype == logits.dtype
        and costs.device == logits.device
        and bool(torch.isfinite(logits).all())
        and bool(torch.isfinite(costs).all()),
        "finite same-device N-by-6 DRIVE logits and costs required",
    )
    _device(logits.device)
    detached = costs.detach()
    regret = detached - detached.min(dim=1, keepdim=True).values
    result = (logits.softmax(dim=1) * regret).sum(dim=1).mean()
    _require(bool(torch.isfinite(result)), "nonfinite expected DRIVE regret")
    return result


def validate_h1_schedule(
    rows: Sequence[Mapping[str, Any]], train: CachedStopmixSelectorData
) -> tuple[dict[str, Any], ...]:
    """HH_260906 - Admit only the complete token-bound H1 cyclic-uniform history."""
    _require(
        isinstance(rows, Sequence) and len(rows) == STEPS,
        "complete 8280-row H1 history required",
    )
    normalized: list[dict[str, Any]] = []
    base_seen = extra_seen = 0
    per_epoch: dict[int, list[dict[str, Any]]] = {epoch: [] for epoch in range(EPOCHS)}
    for position, row in enumerate(rows):
        _require(isinstance(row, Mapping), "H1 history row must be a mapping")
        epoch = position // (SAMPLES_SEEN // BATCH_SIZE // EPOCHS)
        batch_index = position % (SAMPLES_SEEN // BATCH_SIZE // EPOCHS)
        indices = list(row.get("sample_indices", ()))
        tokens = list(row.get("sample_tokens", ()))
        streams = list(row.get("sampling_streams", ()))
        _require(
            row.get("global_step") == position + 1
            and row.get("epoch") == epoch
            and row.get("batch_index") == batch_index,
            "H1 global step, epoch or batch index differs",
        )
        _require(
            len(indices) == len(tokens) == len(streams) == BATCH_SIZE
            and all(type(index) is int and 0 <= index < len(train) for index in indices)
            and len(set(indices)) == BATCH_SIZE
            and all(stream in ("base", "replay") for stream in streams),
            "H1 batch identity or stream contract differs",
        )
        _require(
            tokens == [train.sample_ids[index] for index in indices],
            "H1 dataset index to sample-token binding differs",
        )
        value = {
            "global_step": position + 1,
            "epoch": epoch,
            "batch_index": batch_index,
            "sample_indices": indices,
            "sample_tokens": tokens,
            "sampling_streams": streams,
        }
        normalized.append(value)
        per_epoch[epoch].append(value)
        base_seen += streams.count("base")
        extra_seen += streams.count("replay")
    _require(
        (base_seen, extra_seen) == (BASE_EXPOSURES, EXTRA_EXPOSURES),
        "H1 base or replay exposure accounting differs",
    )
    replay_all = []
    for epoch, epoch_rows in per_epoch.items():
        flattened = [
            (index, stream)
            for row in epoch_rows
            for index, stream in zip(row["sample_indices"], row["sampling_streams"])
        ]
        base = [index for index, stream in flattened if stream == "base"]
        replay = [index for index, stream in flattened if stream == "replay"]
        expected_replay = [
            (epoch * EXTRA_PER_EPOCH + offset) % TRAIN_COUNT
            for offset in range(EXTRA_PER_EPOCH)
        ]
        _require(
            len(flattened) == BASE_PER_EPOCH + EXTRA_PER_EPOCH
            and sorted(base) == list(range(TRAIN_COUNT))
            and sorted(replay) == sorted(expected_replay),
            "H1 per-epoch base permutation or cyclic replay multiset differs",
        )
        replay_all.extend(replay)
    counts = torch.bincount(torch.tensor(replay_all), minlength=TRAIN_COUNT)
    minimum_count, higher_count = divmod(EXTRA_EXPOSURES, TRAIN_COUNT)
    _require(
        int(counts.min()) == minimum_count
        and int(counts.max()) == minimum_count + (1 if higher_count else 0)
        and int((counts == minimum_count + 1).sum()) == higher_count,
        "H1 full-study cyclic replay histogram differs",
    )
    return tuple(normalized)


def schedule_digest(rows: Sequence[Mapping[str, Any]]) -> str:
    """HH_260906 - Bind both I arms to one normalized H1 schedule."""
    return hashlib.sha256(
        json.dumps(rows, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()


def _all_logits(
    data: CachedStopmixSelectorData,
    head: nn.Module,
    arm: str,
    check_deadline: Callable[[], None] | None,
) -> Tensor:
    rows = []
    with torch.no_grad():
        for start in range(0, len(data), BATCH_SIZE):
            _check(check_deadline)
            index = slice(start, start + BATCH_SIZE)
            rows.append(
                score_mode_preserving_logits(
                    head,
                    arm,
                    data.fused[index],
                    data.candidate_xy[index],
                    data.candidate_speed[index],
                    data.original_logits[index],
                )
            )
    return torch.cat(rows)


def mode_preservation_proof(
    original_logits: Tensor, final_logits: Tensor
) -> dict[str, Any]:
    """HH_260906 - Prove exact STOP identity and zero DRIVE/STOP mode changes."""
    _require(
        isinstance(original_logits, Tensor)
        and isinstance(final_logits, Tensor)
        and original_logits.shape == final_logits.shape
        and original_logits.ndim == 2
        and original_logits.shape[1] == TOTAL_CANDIDATE_COUNT
        and original_logits.dtype == final_logits.dtype
        and original_logits.device == final_logits.device
        and bool(torch.isfinite(original_logits).all())
        and bool(torch.isfinite(final_logits).all()),
        "aligned finite K12 logits required for mode proof",
    )
    original_selected = original_logits.argmax(dim=1)
    final_selected = final_logits.argmax(dim=1)
    original_stop = original_selected >= DRIVE_CANDIDATE_COUNT
    final_stop = final_selected >= DRIVE_CANDIDATE_COUNT
    original_drive = ~original_stop
    drive_group_max_mismatch_count = int(
        (
            final_logits[original_drive, :DRIVE_CANDIDATE_COUNT].max(dim=1).values
            != original_logits[original_drive, :DRIVE_CANDIDATE_COUNT]
            .max(dim=1)
            .values
        ).sum()
    )
    stop_row_logit_mismatch_count = sum(
        not torch.equal(final_logits[index], original_logits[index])
        for index in torch.where(original_stop)[0].tolist()
    )
    return {
        "sample_count": len(original_logits),
        "original_drive_count": int((~original_stop).sum()),
        "original_stop_count": int(original_stop.sum()),
        "mode_change_count": int((original_stop != final_stop).sum()),
        "drive_to_stop_count": int((~original_stop & final_stop).sum()),
        "stop_to_drive_count": int((original_stop & ~final_stop).sum()),
        "original_stop_selected_index_mismatch_count": int(
            (final_selected[original_stop] != original_selected[original_stop]).sum()
        ),
        "original_stop_row_logit_mismatch_count": int(
            stop_row_logit_mismatch_count
        ),
        "original_drive_group_max_mismatch_count": drive_group_max_mismatch_count,
        "pass": bool(
            not (original_stop != final_stop).any()
            and not (
                final_selected[original_stop] != original_selected[original_stop]
            ).any()
            and stop_row_logit_mismatch_count == 0
            and drive_group_max_mismatch_count == 0
        ),
    }


def _drive_targets(
    data: CachedStopmixSelectorData,
    projected_costs_fn: Callable[..., Tensor],
    check_deadline: Callable[[], None] | None,
) -> tuple[Tensor, Tensor]:
    costs, _ = projected.projected_targets(
        data, projected_costs_fn, check_deadline
    )
    drive_costs = costs[:, :DRIVE_CANDIDATE_COUNT].detach().clone()
    return drive_costs, drive_costs.argmin(dim=1)


def fit_drive_mode_residual(
    train: CachedStopmixSelectorData,
    dev: CachedStopmixSelectorData,
    *,
    schedule: Sequence[Mapping[str, Any]],
    projected_costs_fn: Callable[..., Tensor],
    projection_proof: Mapping[str, Any],
    arm: str,
    seed: int = SEED,
    device: str | torch.device = "cpu",
    callback: Callable[[dict[str, Any]], None] | None = None,
    check_deadline: Callable[[], None] | None = None,
) -> dict[str, Any]:
    """HH_260906 - Fit one fixed-budget DRIVE-only residual arm without DEV feedback."""
    validate_pair(train, dev)
    _require(arm in ARMS and seed == SEED, "fixed paired I arm and seed required")
    projected._validate_projection(projected_costs_fn, projection_proof)
    schedule = validate_h1_schedule(schedule, train)
    schedule_sha256 = schedule_digest(schedule)
    device = _device(device)
    before = {data.split: data.digest() for data in (train, dev)}
    working = {"train": train.to(device), "val": dev.to(device)}
    head = make_head(arm, seed).to(device)
    initial_head_sha256 = _digest_tensors(head.state_dict().items())
    history: list[dict[str, Any]] = []
    costs: dict[str, Tensor] = {}
    targets: dict[str, Tensor] = {}
    parity: dict[str, Any] = {}
    try:
        for split, data in working.items():
            actual = _all_logits(data, head, arm, check_deadline)
            expected = data.original_logits
            parity[split] = {
                "byte_equal": torch.equal(actual, expected),
                "original_logits_sha256": _digest_tensors(
                    [("logits", expected)]
                ),
                "initial_logits_sha256": _digest_tensors(
                    [("logits", actual)]
                ),
                "max_abs_logit_difference": float((actual - expected).abs().max()),
                "selected_index_mismatch_count": int(
                    (actual.argmax(1) != expected.argmax(1)).sum()
                ),
            }
            _require(
                parity[split]["byte_equal"],
                "zero residual failed exact F logit parity",
            )
        costs["train"], targets["train"] = _drive_targets(
            working["train"], projected_costs_fn, check_deadline
        )
        cost_sha256 = {
            "train": _digest_tensors([("drive_projected_costs", costs["train"])])
        }
        target_sha256 = {
            "train": _digest_tensors([("drive_oracle", targets["train"])])
        }
        optimizer = torch.optim.AdamW(
            head.parameters(), lr=LEARNING_RATE, weight_decay=WEIGHT_DECAY
        )
        head.train()
        base_seen = extra_seen = 0
        for row in schedule:
            _check(check_deadline)
            indices = list(row["sample_indices"])
            index = torch.tensor(indices, dtype=torch.long, device=device)
            data = working["train"]
            logits = score_drive_logits(
                head,
                arm,
                data.fused[index],
                data.candidate_xy[index],
                data.candidate_speed[index],
                data.original_logits[index],
            )
            ce = F.cross_entropy(logits, targets["train"][index])
            regret = expected_drive_regret_loss(logits, costs["train"][index])
            objective = CE_WEIGHT * ce + EXPECTED_REGRET_WEIGHT * regret
            _require(bool(torch.isfinite(objective)), "nonfinite I objective")
            optimizer.zero_grad(set_to_none=True)
            objective.backward()
            _require(
                all(
                    parameter.grad is not None
                    and bool(torch.isfinite(parameter.grad).all())
                    for parameter in head.parameters()
                ),
                "missing or nonfinite residual gradient",
            )
            norm = torch.nn.utils.clip_grad_norm_(
                head.parameters(), MAX_GRAD_NORM
            )
            _require(bool(torch.isfinite(norm)), "nonfinite residual gradient norm")
            optimizer.step()
            _require(
                all(
                    bool(torch.isfinite(parameter).all())
                    for parameter in head.parameters()
                )
                and _nested_tensors_are_finite(optimizer.state_dict()),
                "nonfinite residual parameter or optimizer state",
            )
            base_seen += row["sampling_streams"].count("base")
            extra_seen += row["sampling_streams"].count("replay")
            record = {
                **copy.deepcopy(dict(row)),
                "samples_seen": row["global_step"] * BATCH_SIZE,
                "base_samples_seen": base_seen,
                "extra_samples_seen": extra_seen,
                "drive_candidate_score_loss": float(ce.detach()),
                "drive_candidate_regret_loss": float(regret.detach()),
                "optimization_loss": float(objective.detach()),
                "gradient_norm": float(norm),
            }
            history.append(record)
            if callback is not None:
                callback(copy.deepcopy(record))
        _require(
            len(history) == STEPS
            and history[-1]["samples_seen"] == SAMPLES_SEEN
            and (base_seen, extra_seen) == (BASE_EXPOSURES, EXTRA_EXPOSURES),
            "fixed I step or exposure accounting differs",
        )
        head.eval()
        final_train_logits = _all_logits(
            working["train"], head, arm, check_deadline
        )
        final_dev_logits = _all_logits(
            working["val"], head, arm, check_deadline
        )
        mode_proof = {
            split: mode_preservation_proof(
                data.original_logits,
                final_train_logits if split == "train" else final_dev_logits,
            )
            for split, data in working.items()
        }
        _require(
            all(value["pass"] for value in mode_proof.values()),
            "final I mode preservation proof failed",
        )
        costs["val"], targets["val"] = _drive_targets(
            working["val"], projected_costs_fn, check_deadline
        )
        cost_sha256["val"] = _digest_tensors(
            [("drive_projected_costs", costs["val"])]
        )
        target_sha256["val"] = _digest_tensors(
            [("drive_oracle", targets["val"])]
        )
        final_dev_metrics = cache_core.evaluate_logits(
            working["val"], final_dev_logits, check_deadline
        )
        checked_costs, checked_targets = _drive_targets(
            working["train"], projected_costs_fn, check_deadline
        )
        _require(
            torch.equal(checked_costs, costs["train"])
            and torch.equal(checked_targets, targets["train"]),
            "frozen TRAIN projected costs or DRIVE oracle changed",
        )
        report = {
            "artifact_id": ARTIFACT_ID,
            "status": "DRIVE_MODE_RESIDUAL_FIT_COMPLETE_NOT_PROMOTED",
            "arm": arm,
            "source_checkpoint_sha256": SOURCE_CHECKPOINT_SHA256,
            "h1_history_sha256": H1_HISTORY_SHA256,
            "normalized_h1_schedule_sha256": schedule_sha256,
            "seed": seed,
            "device": str(device),
            "optimizer": "AdamW",
            "optimizer_resumed": False,
            "epochs": EPOCHS,
            "steps": STEPS,
            "batch_size": BATCH_SIZE,
            "samples_seen": SAMPLES_SEEN,
            "base_samples_seen": base_seen,
            "extra_samples_seen": extra_seen,
            "learning_rate": LEARNING_RATE,
            "weight_decay": WEIGHT_DECAY,
            "maximum_gradient_norm": MAX_GRAD_NORM,
            "objective": "0.1 * DRIVE6 projected-cost argmin CE + 0.1 * softmax expected detached DRIVE6 projected regret",
            "candidate_score_weight": CE_WEIGHT,
            "candidate_regret_weight": EXPECTED_REGRET_WEIGHT,
            "initialization": "Identical CPU seed 20260906 449-256-1 MLP in both arms; final Linear weight and bias are exactly zero, with fresh AdamW state.",
            "initial_head_sha256": initial_head_sha256,
            "final_head_sha256": _digest_tensors(head.state_dict().items()),
            "head_parameter_count": HEAD_PARAMETER_COUNT,
            "candidate_count": TOTAL_CANDIDATE_COUNT,
            "trained_candidate_indices": list(range(DRIVE_CANDIDATE_COUNT)),
            "frozen_candidate_indices": list(
                range(DRIVE_CANDIDATE_COUNT, TOTAL_CANDIDATE_COUNT)
            ),
            "residual_inputs": {
                "frozen_context_features": 256,
                "frozen_original_drive_logit_features": 1,
                "frozen_candidate_geometry_features": 192,
                "geometry_provided": arm == ARMS[1],
                "total_features": 449,
            },
            "initial_f_logit_parity": parity,
            "final_mode_preservation": mode_proof,
            "cache_sha256_before": before,
            "cache_sha256_after": dict(before),
            "projected_drive_cost_sha256": cost_sha256,
            "projected_drive_oracle_sha256": target_sha256,
            "core_full_dev_metric_evaluation_count": 1,
            "dev_evaluated_only_after_final_step": True,
            "dev_used_for_training": False,
            "dev_used_for_intermediate_selection": False,
            "test_read_or_used": False,
            "generator_parameters_trained": False,
            "cached_candidates_trained_or_recomputed": False,
            "f_logits_trained_or_recomputed": False,
            "drive_stop_mode_change_allowed": False,
            "selector_replacement_diagnostic_only": True,
            "new_autonomous_function_head_added": False,
            "automatic_promotion": False,
            "runtime_checkpoint_exported": False,
            "closed_loop_evaluation": False,
            "vehicle_control_approved": False,
            "history": history,
            "final_dev_metrics": final_dev_metrics,
            "final_state": {
                "epoch": EPOCHS,
                "next_batch_index": 0,
                "global_step": STEPS,
                "samples_seen": SAMPLES_SEEN,
            },
            "interpretation": "Paired F DRIVE-candidate replacement diagnostic. It cannot establish stationary performance, closed-loop, TEST, all-map, export or vehicle readiness.",
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
            "drive_oracle_indices_cpu": {
                split: value.detach().cpu().clone()
                for split, value in targets.items()
            },
        }
    finally:
        _require(
            train.digest() == before["train"] and dev.digest() == before["val"],
            "original H cache mutated during I fitting",
        )
        _require(
            all(
                data.digest() == before[split]
                for split, data in working.items()
            ),
            "working H cache mutated during I fitting",
        )
