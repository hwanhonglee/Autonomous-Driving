"""Model-neutral evidence generation for a portable trajectory checkpoint."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any, Mapping, Sequence

import torch
from torch import Tensor
from torch.utils.data import DataLoader

from .contract import ContractError
from .dataset import load_training_examples
from .losses import TrajectoryLossConfig, trajectory_loss
from .model import ModelConfig, PerspectiveTrajectoryModel, parameter_count
from .torch_dataset import Common10TorchDataset
from .train import (
    CHECKPOINT_ID,
    DOMAIN_BALANCED_SAMPLING_POLICY,
    KNOWN_DOMAINS,
    TrainConfig,
    UNIFORM_SAMPLING_POLICY,
    _absolute_path,
    _weighted_prefix_domain_counts,
    _canonical_sha256,
    _device_abi,
    _read_checkpoint_file,
    _require_sha256,
    _runtime_abi,
    _sampling_plan_from_counts,
    _seed_everything,
    _select_device,
    _slice_loaded,
)
from .visualize import render_trajectory_png


EVALUATION_ID = "portable_e2e.open_loop_evaluation.v1"
CORE_METRIC_NAMES = (
    "loss",
    "regression_loss",
    "candidate_score_loss",
    "oracle_ade_m",
    "selected_ade_m",
    "selected_fde_m",
    "selected_speed_mae_mps",
    "selected_yaw_mae_rad",
    "selected_kinematic_speed_mae_mps",
)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _atomic_json(path: Path, value: Mapping[str, Any]) -> None:
    temporary = path.with_name(f".{path.name}.tmp.{os.getpid()}")
    payload = json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n"
    try:
        with temporary.open("x", encoding="utf-8") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def _selected(candidate: Tensor, indices: Tensor) -> Tensor:
    view = [candidate.shape[0], 1] + [1] * (candidate.ndim - 2)
    expand = [candidate.shape[0], 1, *candidate.shape[2:]]
    return candidate.gather(1, indices.view(*view).expand(*expand)).squeeze(1)


def _horizon_sums(
    selected_xy: Tensor,
    selected_speed: Tensor,
    target_xy: Tensor,
    target_speed: Tensor,
    valid: Tensor,
) -> dict[str, tuple[float, int]]:
    output: dict[str, tuple[float, int]] = {}
    displacement = torch.linalg.norm(selected_xy - target_xy, dim=-1)
    speed_error = (selected_speed - target_speed).abs()
    for horizon_s, point_index in ((1.0, 9), (3.0, 29), (6.4, 63)):
        eligible = valid[:, point_index]
        count = int(eligible.sum().item())
        if count == 0:
            continue
        prefix = valid[:, : point_index + 1]
        ade_per_sample = (
            (displacement[:, : point_index + 1] * prefix.to(displacement.dtype)).sum(dim=1)
            / prefix.sum(dim=1).clamp_min(1).to(displacement.dtype)
        )
        speed_per_sample = (
            (speed_error[:, : point_index + 1] * prefix.to(speed_error.dtype)).sum(dim=1)
            / prefix.sum(dim=1).clamp_min(1).to(speed_error.dtype)
        )
        label = str(horizon_s).replace(".", "p")
        output[f"ade_{label}s_m"] = (float(ade_per_sample[eligible].sum().item()), count)
        output[f"fde_{label}s_m"] = (
            float(displacement[eligible, point_index].sum().item()),
            count,
        )
        output[f"speed_mae_{label}s_mps"] = (
            float(speed_per_sample[eligible].sum().item()),
            count,
        )
    return output


def _accumulate_batch_metrics(
    metric_sums: dict[str, float],
    metric_counts: dict[str, int],
    *,
    candidate_xy: Tensor,
    candidate_speed: Tensor,
    candidate_logits: Tensor,
    target_xy: Tensor,
    target_speed: Tensor,
    target_valid: Tensor,
    target_yaw: Tensor,
    loss_config: TrajectoryLossConfig,
) -> None:
    losses = trajectory_loss(
        candidate_xy,
        candidate_speed,
        candidate_logits,
        target_xy,
        target_speed,
        target_valid,
        loss_config,
        target_yaw=target_yaw,
    )
    current_batch = int(candidate_xy.shape[0])
    for key in CORE_METRIC_NAMES:
        metric_sums[key] = metric_sums.get(key, 0.0) + float(
            losses[key].detach().cpu().item()
        ) * current_batch
        metric_counts[key] = metric_counts.get(key, 0) + current_batch

    selected_indices = candidate_logits.argmax(dim=1)
    selected_xy = _selected(candidate_xy, selected_indices)
    selected_speed = _selected(candidate_speed, selected_indices)
    for key, (value, count) in _horizon_sums(
        selected_xy,
        selected_speed,
        target_xy,
        target_speed,
        target_valid,
    ).items():
        metric_sums[key] = metric_sums.get(key, 0.0) + value
        metric_counts[key] = metric_counts.get(key, 0) + count


def _evaluation_sample_domains(
    dataset: Common10TorchDataset,
) -> tuple[dict[str, str], dict[str, int]]:
    by_token: dict[str, str] = {}
    counts = {domain: 0 for domain in KNOWN_DOMAINS}
    for example in dataset.examples:
        if example.domain not in KNOWN_DOMAINS:
            raise ContractError(
                f"evaluation example {example.token!r} has unknown domain "
                f"{example.domain!r}; expected one of {KNOWN_DOMAINS}"
            )
        if example.token in by_token:
            raise ContractError(f"duplicate evaluation sample token: {example.token}")
        by_token[example.token] = example.domain
        counts[example.domain] += 1
    return by_token, {domain: counts[domain] for domain in KNOWN_DOMAINS if counts[domain]}


def _checkpoint_sampling_provenance(
    payload: Mapping[str, Any],
) -> tuple[str, str, dict[str, int]]:
    def checked_counts(
        value: Any, context: str, *, positive: bool
    ) -> dict[str, int]:
        minimum = 1 if positive else 0
        if (
            not isinstance(value, dict)
            or not value
            or any(domain not in KNOWN_DOMAINS for domain in value)
            or any(
                isinstance(count, bool)
                or not isinstance(count, int)
                or count < minimum
                for count in value.values()
            )
        ):
            qualifier = "positive" if positive else "nonnegative"
            raise ContractError(
                f"checkpoint {context} must contain {qualifier} known-domain counts"
            )
        return dict(value)

    sampling_plan = payload.get("sampling_plan")
    if not isinstance(sampling_plan, dict) or not sampling_plan:
        raise ContractError("checkpoint sampling plan is missing or invalid")
    sampling_plan_sha256 = payload.get("sampling_plan_sha256")
    _require_sha256(sampling_plan_sha256, "checkpoint sampling plan fingerprint")
    try:
        calculated_sha256 = _canonical_sha256(sampling_plan)
    except (MemoryError, OverflowError, RecursionError, TypeError, ValueError) as error:
        raise ContractError(f"checkpoint sampling plan is not canonical JSON: {error}") from error
    if sampling_plan_sha256 != calculated_sha256:
        raise ContractError("checkpoint sampling plan fingerprint does not match")

    sampling_policy = sampling_plan.get("policy")
    if sampling_policy not in (
        UNIFORM_SAMPLING_POLICY,
        DOMAIN_BALANCED_SAMPLING_POLICY,
    ):
        raise ContractError("checkpoint sampling policy is not supported")
    train_config_value = payload.get("train_config")
    if not isinstance(train_config_value, dict) or set(train_config_value) != set(
        TrainConfig().to_dict()
    ):
        raise ContractError("checkpoint training config fields do not match the v1 schema")
    try:
        train_config = TrainConfig(**train_config_value)
        train_config.validate()
    except (ContractError, TypeError) as error:
        raise ContractError(f"checkpoint training config is invalid: {error}") from error
    if train_config.sampling_policy != sampling_policy:
        raise ContractError("checkpoint training config sampling policy does not match")

    if sampling_plan.get("known_domains") != list(KNOWN_DOMAINS):
        raise ContractError("checkpoint sampling plan known domains are not canonical")
    if sampling_plan.get("replacement") is not False:
        raise ContractError("checkpoint sampling plan must be without replacement")
    dataset_domain_counts = checked_counts(
        sampling_plan.get("dataset_domain_counts"),
        "sampling plan dataset domains",
        positive=True,
    )
    epoch_domain_counts = checked_counts(
        sampling_plan.get("epoch_domain_sample_counts"),
        "sampling plan epoch domains",
        positive=True,
    )
    discarded_domain_counts = checked_counts(
        sampling_plan.get("epoch_discarded_sample_counts"),
        "sampling plan discarded domains",
        positive=False,
    )
    if not (
        set(dataset_domain_counts)
        == set(epoch_domain_counts)
        == set(discarded_domain_counts)
    ) or any(
        dataset_domain_counts[domain]
        != epoch_domain_counts[domain] + discarded_domain_counts[domain]
        for domain in dataset_domain_counts
    ):
        raise ContractError("checkpoint sampling plan domain accounting is inconsistent")
    epoch_sample_count = sampling_plan.get("epoch_sample_count")
    batch_size = sampling_plan.get("batch_size")
    batches_per_epoch = sampling_plan.get("batches_per_epoch")
    if (
        isinstance(epoch_sample_count, bool)
        or not isinstance(epoch_sample_count, int)
        or epoch_sample_count != sum(epoch_domain_counts.values())
        or isinstance(batch_size, bool)
        or not isinstance(batch_size, int)
        or batch_size <= 0
        or train_config.batch_size != batch_size
        or isinstance(batches_per_epoch, bool)
        or not isinstance(batches_per_epoch, int)
        or batches_per_epoch != (epoch_sample_count + batch_size - 1) // batch_size
    ):
        raise ContractError("checkpoint sampling plan epoch accounting is inconsistent")
    domain_ratios = sampling_plan.get("domain_ratios")
    if sampling_policy == UNIFORM_SAMPLING_POLICY:
        if domain_ratios != {} or epoch_domain_counts != dataset_domain_counts:
            raise ContractError("checkpoint uniform sampling plan is inconsistent")
    else:
        ratio_counts = checked_counts(
            domain_ratios,
            "sampling plan domain ratios",
            positive=True,
        )
        if set(ratio_counts) != set(KNOWN_DOMAINS) or set(
            dataset_domain_counts
        ) != set(KNOWN_DOMAINS):
            raise ContractError(
                "checkpoint balanced sampling plan must contain carla and real"
            )
        if any(
            epoch_domain_counts[domain] % ratio_counts[domain]
            for domain in KNOWN_DOMAINS
        ) or len(
            {
                epoch_domain_counts[domain] // ratio_counts[domain]
                for domain in KNOWN_DOMAINS
            }
        ) != 1:
            raise ContractError("checkpoint balanced sampling quotas do not match ratios")

    expected_plan = _sampling_plan_from_counts(dataset_domain_counts, train_config)
    if sampling_plan != expected_plan:
        raise ContractError(
            "checkpoint sampling plan does not match its validated training config"
        )

    sampling_metrics = payload.get("sampling_metrics")
    if not isinstance(sampling_metrics, dict) or set(sampling_metrics) != {
        "samples_seen",
        "domain_samples_seen",
    }:
        raise ContractError("checkpoint sampling metrics are missing or invalid")
    samples_seen = sampling_metrics["samples_seen"]
    domain_samples_seen = sampling_metrics["domain_samples_seen"]
    if (
        isinstance(samples_seen, bool)
        or not isinstance(samples_seen, int)
        or samples_seen <= 0
        or not isinstance(domain_samples_seen, dict)
        or not domain_samples_seen
        or any(domain not in KNOWN_DOMAINS for domain in domain_samples_seen)
        or any(
            isinstance(count, bool) or not isinstance(count, int) or count < 0
            for count in domain_samples_seen.values()
        )
        or sum(domain_samples_seen.values()) != samples_seen
    ):
        raise ContractError("checkpoint domain sampling metrics are inconsistent")
    if set(dataset_domain_counts) != set(domain_samples_seen):
        raise ContractError("checkpoint sampling plan domains do not match its metrics")
    state = payload.get("state")
    state_fields = {
        "epoch",
        "next_batch_index",
        "global_step",
        "samples_seen",
        "domain_samples_seen",
    }
    if not isinstance(state, dict) or set(state) != state_fields:
        raise ContractError("checkpoint training state fields do not match the v1 schema")
    for name in state_fields - {"domain_samples_seen"}:
        value = state[name]
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ContractError("checkpoint training state cursors must be nonnegative integers")
    if (
        state["global_step"] > train_config.max_steps
        or state["epoch"] > state["global_step"]
        or state["next_batch_index"] > state["global_step"]
        or samples_seen > state["global_step"] * train_config.batch_size
        or any(count > samples_seen for count in domain_samples_seen.values())
    ):
        raise ContractError("checkpoint training state exceeds its configured bounds")
    if (
        state["samples_seen"] != samples_seen
        or state["domain_samples_seen"] != domain_samples_seen
    ):
        raise ContractError("checkpoint sampling metrics do not match its state")

    batches_per_epoch = int(sampling_plan["batches_per_epoch"])
    epoch_sample_count = int(sampling_plan["epoch_sample_count"])
    next_batch = state["next_batch_index"]
    if next_batch > batches_per_epoch:
        raise ContractError("checkpoint training state batch cursor exceeds its epoch")
    current_epoch_samples = min(
        next_batch * train_config.batch_size,
        epoch_sample_count,
    )
    expected_steps = state["epoch"] * batches_per_epoch + next_batch
    expected_samples = state["epoch"] * epoch_sample_count + current_epoch_samples
    if state["global_step"] != expected_steps or samples_seen != expected_samples:
        raise ContractError("checkpoint training state cursor accounting is inconsistent")
    expected_domain_samples = {
        domain: state["epoch"]
        * int(sampling_plan["epoch_domain_sample_counts"][domain])
        for domain in dataset_domain_counts
    }
    current_domain_counts = _weighted_prefix_domain_counts(
        sampling_plan["epoch_domain_sample_counts"],
        seed=train_config.seed,
        epoch=state["epoch"],
        prefix_size=current_epoch_samples,
    )
    for domain, count in current_domain_counts.items():
        expected_domain_samples[domain] += count
    if domain_samples_seen != expected_domain_samples:
        raise ContractError("checkpoint domain sample counts do not match its cursor")
    if state["global_step"] > train_config.max_steps or not (
        state["global_step"] == train_config.max_steps
        or state["global_step"] % train_config.checkpoint_interval == 0
    ):
        raise ContractError("checkpoint training state is not at a valid saved step")
    return sampling_plan_sha256, sampling_policy, dict(domain_samples_seen)


def _warm_up_model(
    model: PerspectiveTrajectoryModel,
    dataset: Common10TorchDataset,
    *,
    device: torch.device,
    batch_size: int,
) -> dict[str, float | int]:
    """Run one unmeasured model batch before collecting evaluation latency."""
    loader = DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=False,
        num_workers=0,
        pin_memory=False,
    )
    try:
        raw_batch = next(iter(loader))
    except StopIteration as error:
        raise ContractError("evaluation warm-up dataset produced no samples") from error
    batch = {
        key: value.to(device) if isinstance(value, Tensor) else value
        for key, value in raw_batch.items()
    }
    if device.type == "cuda":
        torch.cuda.synchronize(device)
    started = time.perf_counter()
    with torch.no_grad():
        outputs = model(
            batch["images"],
            batch["calibration"],
            batch["ego_history"],
            batch["ego_history_mask"],
            batch["route_xy"],
            batch["route_mask"],
        )
    if device.type == "cuda":
        torch.cuda.synchronize(device)
    elapsed = time.perf_counter() - started
    if elapsed <= 0.0 or any(
        not bool(torch.isfinite(output).all().item()) for output in outputs
    ):
        raise FloatingPointError("evaluation model warm-up was non-finite or unmeasurable")
    return {
        "warmup_batches": 1,
        "warmup_samples": int(batch["images"].shape[0]),
        "warmup_seconds": elapsed,
    }


def evaluate_model(
    dataset: Common10TorchDataset,
    *,
    checkpoint_path: Path,
    output_dir: Path,
    dataset_fingerprint_sha256: str,
    device_name: str = "cpu",
    batch_size: int = 4,
    num_workers: int = 0,
    render_count: int = 8,
    corpus_fingerprint_sha256: str | None = None,
    evaluation_split: str = "val",
) -> dict[str, Any]:
    integer_counts = {
        "batch_size": (batch_size, 1),
        "num_workers": (num_workers, 0),
        "render_count": (render_count, 0),
    }
    if any(
        isinstance(value, bool) or not isinstance(value, int) or value < minimum
        for value, minimum in integer_counts.values()
    ):
        raise ContractError("evaluation batch/worker/render counts are invalid")
    device = _select_device(device_name)
    _seed_everything(0, device)
    if evaluation_split not in ("val", "test") or dataset.split != evaluation_split:
        raise ContractError("evaluation only accepts an explicit val or test split")
    _require_sha256(dataset_fingerprint_sha256, "evaluation dataset fingerprint")
    if dataset_fingerprint_sha256 != dataset.fingerprint_sha256:
        raise ContractError("caller dataset fingerprint does not match the tensor dataset")
    if not dataset.verify_image_sha256:
        raise ContractError("release evaluation requires image SHA-256 verification")
    sample_domains, expected_domain_sample_counts = _evaluation_sample_domains(dataset)
    # Keep the final path component unresolved so _read_checkpoint_file's
    # O_NOFOLLOW guard can reject a checkpoint symlink.
    checkpoint_path = _absolute_path(checkpoint_path)
    payload, loaded_checkpoint_sha256 = _read_checkpoint_file(checkpoint_path, device)
    _require_sha256(loaded_checkpoint_sha256, "evaluation checkpoint fingerprint")
    if payload.get("checkpoint_id") != CHECKPOINT_ID:
        raise ContractError("checkpoint kind is not supported by this evaluator")
    (
        training_sampling_plan_sha256,
        training_sampling_policy,
        training_domain_samples_seen,
    ) = _checkpoint_sampling_provenance(payload)
    if payload.get("training_split") != "train":
        raise ContractError("evaluation checkpoint was not produced from the train split")
    training_episode_ids = payload.get("training_episode_ids")
    if (
        not isinstance(training_episode_ids, list)
        or not training_episode_ids
        or any(not isinstance(value, str) or not value for value in training_episode_ids)
    ):
        raise ContractError("checkpoint training episode IDs are missing or invalid")
    if training_episode_ids != sorted(training_episode_ids) or len(
        set(training_episode_ids)
    ) != len(training_episode_ids):
        raise ContractError("checkpoint training episode IDs are not canonical")
    evaluation_episode_ids = sorted(
        {example.episode_id for example in dataset.examples}
    )
    episode_overlap = sorted(set(training_episode_ids) & set(evaluation_episode_ids))
    if episode_overlap:
        raise ContractError(
            "train/evaluation episode leakage detected: " + ", ".join(episode_overlap[:5])
        )
    training_dataset_fingerprint = payload.get("dataset_fingerprint_sha256")
    _require_sha256(
        training_dataset_fingerprint, "checkpoint training dataset fingerprint"
    )
    if training_dataset_fingerprint == dataset_fingerprint_sha256:
        raise ContractError("evaluation split fingerprint matches the training split")
    if corpus_fingerprint_sha256 is None:
        corpus_fingerprint_sha256 = dataset_fingerprint_sha256
    _require_sha256(corpus_fingerprint_sha256, "evaluation corpus fingerprint")
    if payload.get("corpus_fingerprint_sha256") != corpus_fingerprint_sha256:
        raise ContractError("evaluation corpus fingerprint does not match the checkpoint")
    model_value = payload.get("model_config")
    if not isinstance(model_value, dict):
        raise ContractError("checkpoint model config is missing")
    model_config = ModelConfig.from_mapping(model_value)
    model_config_sha256 = _canonical_sha256(model_config.to_dict())
    if payload.get("model_config_sha256") != model_config_sha256:
        raise ContractError("checkpoint model config fingerprint does not match")
    if model_config != dataset.config:
        raise ContractError("evaluation dataset tensor config does not match the checkpoint")
    loss_value = payload.get("loss_config")
    if not isinstance(loss_value, dict):
        raise ContractError("checkpoint loss config is missing")
    expected_loss_fields = set(TrajectoryLossConfig().to_dict())
    if set(loss_value) != expected_loss_fields:
        raise ContractError("checkpoint loss config fields do not match")
    try:
        loss_config = TrajectoryLossConfig(**loss_value)
    except TypeError as error:
        raise ContractError(f"checkpoint loss config is invalid: {error}") from error
    loss_config.validate()
    model = PerspectiveTrajectoryModel(model_config).to(device)
    try:
        model.load_state_dict(payload["model_state_dict"], strict=True)
    except (KeyError, RuntimeError, ValueError) as error:
        raise ContractError(f"checkpoint model tensors are incompatible: {error}") from error
    if not all(
        bool(torch.isfinite(parameter).all().item()) for parameter in model.parameters()
    ):
        raise FloatingPointError("checkpoint model contains NaN or Inf")
    model.eval()

    output_dir = output_dir.expanduser().resolve()
    if output_dir.exists():
        raise ContractError(f"evaluation output directory already exists: {output_dir}")
    output_dir.mkdir(parents=True)
    plot_dir = output_dir / "trajectories"
    if render_count:
        plot_dir.mkdir()

    loader = DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=False,
        num_workers=num_workers,
        pin_memory=False,
    )
    warmup_timing = _warm_up_model(
        model,
        dataset,
        device=device,
        batch_size=batch_size,
    )
    metric_sums: dict[str, float] = {}
    metric_counts: dict[str, int] = {}
    per_domain_sums: dict[str, dict[str, float]] = {
        domain: {} for domain in expected_domain_sample_counts
    }
    per_domain_counts: dict[str, dict[str, int]] = {
        domain: {} for domain in expected_domain_sample_counts
    }
    observed_domain_sample_counts = {
        domain: 0 for domain in expected_domain_sample_counts
    }
    rendered = 0
    sample_count = 0
    forward_seconds = 0.0
    wall_start = time.perf_counter()
    with torch.no_grad():
        for batch in loader:
            tensor_batch = {
                key: value.to(device) if isinstance(value, Tensor) else value
                for key, value in batch.items()
            }
            if device.type == "cuda":
                torch.cuda.synchronize(device)
            forward_start = time.perf_counter()
            candidate_xy, candidate_speed, candidate_logits = model(
                tensor_batch["images"],
                tensor_batch["calibration"],
                tensor_batch["ego_history"],
                tensor_batch["ego_history_mask"],
                tensor_batch["route_xy"],
                tensor_batch["route_mask"],
            )
            if device.type == "cuda":
                torch.cuda.synchronize(device)
            forward_seconds += time.perf_counter() - forward_start
            _accumulate_batch_metrics(
                metric_sums,
                metric_counts,
                candidate_xy=candidate_xy,
                candidate_speed=candidate_speed,
                candidate_logits=candidate_logits,
                target_xy=tensor_batch["target_xy"],
                target_speed=tensor_batch["target_speed_mps"],
                target_valid=tensor_batch["target_valid"],
                target_yaw=tensor_batch["target_yaw_rad"],
                loss_config=loss_config,
            )
            current_batch = int(candidate_xy.shape[0])
            sample_count += current_batch

            raw_sample_ids = batch["sample_id"]
            if not isinstance(raw_sample_ids, (list, tuple)) or len(
                raw_sample_ids
            ) != current_batch:
                raise ContractError("evaluation batch sample IDs are invalid")
            positions_by_domain: dict[str, list[int]] = {
                domain: [] for domain in expected_domain_sample_counts
            }
            for position, raw_sample_id in enumerate(raw_sample_ids):
                sample_id = str(raw_sample_id)
                domain = sample_domains.get(sample_id)
                if domain is None:
                    raise ContractError(
                        f"evaluation loader returned an unknown sample ID: {sample_id}"
                    )
                positions_by_domain[domain].append(position)
                observed_domain_sample_counts[domain] += 1
            for domain, positions in positions_by_domain.items():
                if not positions:
                    continue
                indices = torch.tensor(positions, dtype=torch.long, device=device)
                _accumulate_batch_metrics(
                    per_domain_sums[domain],
                    per_domain_counts[domain],
                    candidate_xy=candidate_xy.index_select(0, indices),
                    candidate_speed=candidate_speed.index_select(0, indices),
                    candidate_logits=candidate_logits.index_select(0, indices),
                    target_xy=tensor_batch["target_xy"].index_select(0, indices),
                    target_speed=tensor_batch["target_speed_mps"].index_select(
                        0, indices
                    ),
                    target_valid=tensor_batch["target_valid"].index_select(0, indices),
                    target_yaw=tensor_batch["target_yaw_rad"].index_select(0, indices),
                    loss_config=loss_config,
                )

            for batch_index in range(current_batch):
                if rendered >= render_count:
                    break
                valid_route = batch["route_mask"][batch_index]
                sample_id = str(batch["sample_id"][batch_index])
                safe_name = "".join(
                    character if character.isalnum() or character in "-_." else "_"
                    for character in sample_id
                )[:120]
                render_trajectory_png(
                    plot_dir / f"{rendered:04d}_{safe_name}.png",
                    route_xy=batch["route_xy"][batch_index][valid_route].tolist(),
                    target_xy=batch["target_xy"][batch_index].tolist(),
                    target_valid=batch["target_valid"][batch_index].tolist(),
                    candidate_xy=candidate_xy[batch_index].detach().cpu().tolist(),
                    candidate_logits=candidate_logits[batch_index].detach().cpu().tolist(),
                    title=f"sample={sample_id}",
                )
                rendered += 1

    wall_seconds = time.perf_counter() - wall_start
    if sample_count == 0:
        raise ContractError("evaluation dataset produced no samples")
    if observed_domain_sample_counts != expected_domain_sample_counts:
        raise ContractError("evaluation loader domain sample counts are inconsistent")
    metrics = {
        key: metric_sums[key] / metric_counts[key] for key in sorted(metric_sums)
    }
    per_domain_metrics = {
        domain: {
            "sample_count": observed_domain_sample_counts[domain],
            "metrics": {
                key: per_domain_sums[domain][key] / per_domain_counts[domain][key]
                for key in sorted(per_domain_sums[domain])
            },
            "metric_counts": per_domain_counts[domain],
        }
        for domain in expected_domain_sample_counts
    }
    if any(not math.isfinite(value) for value in metrics.values()) or any(
        not math.isfinite(value)
        for domain_report in per_domain_metrics.values()
        for value in domain_report["metrics"].values()
    ):
        raise FloatingPointError("evaluation produced NaN or Inf")
    report = {
        "evaluation_id": EVALUATION_ID,
        "created_at_utc": _utc_now(),
        "status": "OPEN_LOOP_EVALUATION_COMPLETE",
        "warning": "OPEN-LOOP RESEARCH RESULT — NOT APPROVED FOR VEHICLE CONTROL",
        "checkpoint": str(checkpoint_path),
        "checkpoint_sha256": loaded_checkpoint_sha256,
        "model_config_sha256": model_config_sha256,
        "training_dataset_fingerprint_sha256": training_dataset_fingerprint,
        "training_sampling_plan_sha256": training_sampling_plan_sha256,
        "training_sampling_policy": training_sampling_policy,
        "training_domain_samples_seen": training_domain_samples_seen,
        "dataset_fingerprint_sha256": dataset_fingerprint_sha256,
        "corpus_fingerprint_sha256": corpus_fingerprint_sha256,
        "evaluation_split": evaluation_split,
        "training_episode_count": len(training_episode_ids),
        "evaluation_episode_count": len(evaluation_episode_ids),
        "sample_count": sample_count,
        "domain_sample_counts": observed_domain_sample_counts,
        "per_domain_metrics": per_domain_metrics,
        "rendered_trajectory_png": rendered,
        "model_parameter_count": parameter_count(model),
        "device": str(device),
        "batch_size": batch_size,
        "runtime": _runtime_abi(),
        "hardware": _device_abi(device),
        "timing": {
            **warmup_timing,
            "evaluation_wall_seconds": wall_seconds,
            "model_forward_seconds": forward_seconds,
            "model_forward_ms_per_sample": 1000.0 * forward_seconds / sample_count,
        },
        "metrics": metrics,
        "metric_counts": metric_counts,
        "vehicle_control_approved": False,
    }
    _atomic_json(output_dir / "metrics.json", report)
    return report


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate one portable E2E checkpoint.")
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--split", choices=("val", "test"), default="val")
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--batch-size", type=int, default=4)
    parser.add_argument("--num-workers", type=int, default=0)
    parser.add_argument("--render-count", type=int, default=8)
    parser.add_argument("--limit-samples", type=int)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        device = _select_device(args.device)
        payload, _ = _read_checkpoint_file(
            _absolute_path(args.checkpoint), device
        )
        if payload.get("checkpoint_id") != CHECKPOINT_ID:
            raise ContractError("checkpoint kind is not supported by this evaluator")
        model_value = payload.get("model_config")
        if not isinstance(model_value, dict):
            raise ContractError("checkpoint model config is missing")
        model_config = ModelConfig.from_mapping(model_value)
        loaded = load_training_examples(
            args.dataset,
            split=args.split,
            mode="planning",
            check_image_hashes=True,
        )
        loaded = _slice_loaded(loaded, args.limit_samples)
        dataset = Common10TorchDataset(
            loaded.examples,
            model_config,
            verify_image_sha256=True,
            split=loaded.split,
        )
        report = evaluate_model(
            dataset,
            checkpoint_path=args.checkpoint,
            output_dir=args.output_dir,
            dataset_fingerprint_sha256=dataset.fingerprint_sha256,
            corpus_fingerprint_sha256=str(
                loaded.validation_report["dataset_fingerprint_sha256"]
            ),
            device_name=args.device,
            batch_size=args.batch_size,
            num_workers=args.num_workers,
            render_count=args.render_count,
            evaluation_split=loaded.split,
        )
    except (ContractError, FloatingPointError, OSError, ValueError) as error:
        print(f"EVALUATION_ERROR: {error}", file=sys.stderr)
        return 2
    print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
