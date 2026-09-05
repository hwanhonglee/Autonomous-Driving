"""Masked multi-candidate planning loss and open-loop batch metrics."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any

import torch
from torch import Tensor
from torch.nn import functional as F

from .contract import ContractError


@dataclass(frozen=True)
class TrajectoryLossConfig:
    xy_weight: float = 1.0
    speed_weight: float = 0.2
    yaw_weight: float = 0.1
    kinematic_speed_weight: float = 0.05
    final_displacement_weight: float = 0.5
    candidate_score_weight: float = 0.1

    def validate(self) -> None:
        for name, value in asdict(self).items():
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                or float(value) < 0.0
            ):
                raise ContractError(f"loss.{name} must be a finite nonnegative number")
        if self.xy_weight <= 0.0:
            raise ContractError("loss.xy_weight must be positive")

    def to_dict(self) -> dict[str, Any]:
        self.validate()
        return asdict(self)


def _validate_shapes(
    candidate_xy: Tensor,
    candidate_speed: Tensor,
    candidate_logits: Tensor,
    target_xy: Tensor,
    target_speed: Tensor,
    valid_mask: Tensor,
    target_yaw: Tensor | None,
) -> None:
    if candidate_xy.ndim != 4 or candidate_xy.shape[-1] != 2:
        raise ValueError("candidate_xy must have shape [B,K,N,2]")
    batch, candidates, points, _ = candidate_xy.shape
    if batch <= 0 or candidates <= 0 or points <= 0:
        raise ValueError("planning tensors must have nonempty B, K, and N axes")
    expected = {
        "candidate_speed": (batch, candidates, points),
        "candidate_logits": (batch, candidates),
        "target_xy": (batch, points, 2),
        "target_speed": (batch, points),
        "valid_mask": (batch, points),
    }
    actual = {
        "candidate_speed": tuple(candidate_speed.shape),
        "candidate_logits": tuple(candidate_logits.shape),
        "target_xy": tuple(target_xy.shape),
        "target_speed": tuple(target_speed.shape),
        "valid_mask": tuple(valid_mask.shape),
    }
    for name, expected_shape in expected.items():
        if actual[name] != expected_shape:
            raise ValueError(f"{name} shape {actual[name]} does not match {expected_shape}")
    if valid_mask.dtype != torch.bool:
        raise ValueError("valid_mask must use torch.bool")
    if bool(torch.any(~valid_mask[:, :-1] & valid_mask[:, 1:]).item()):
        raise ValueError("valid_mask must be a contiguous valid prefix")
    if not bool(torch.all(valid_mask.any(dim=1)).item()):
        raise ValueError("every training sample needs at least one valid future point")
    tensors = (candidate_xy, candidate_speed, candidate_logits, target_xy, target_speed)
    reference_device = candidate_xy.device
    if any(value.device != reference_device for value in (*tensors, valid_mask)):
        raise ValueError("all planning-loss tensors must be on one device")
    if not all(torch.is_floating_point(value) for value in tensors):
        raise ValueError("planning-loss value tensors must use floating-point dtypes")
    if not all(bool(torch.isfinite(value).all().item()) for value in tensors):
        raise FloatingPointError("planning loss received NaN or Inf")
    if target_yaw is not None:
        if tuple(target_yaw.shape) != (batch, points):
            raise ValueError("target_yaw shape does not match [B,N]")
        if target_yaw.device != reference_device or not torch.is_floating_point(target_yaw):
            raise ValueError("target_yaw must be floating point on the loss device")
        if not bool(torch.isfinite(target_yaw).all().item()):
            raise FloatingPointError("target_yaw contains NaN or Inf")


def _last_valid_indices(valid_mask: Tensor) -> Tensor:
    point_indices = torch.arange(valid_mask.shape[1], device=valid_mask.device)
    point_indices = point_indices.unsqueeze(0).expand_as(valid_mask)
    masked_indices = torch.where(valid_mask, point_indices, torch.zeros_like(point_indices))
    return masked_indices.max(dim=1).values


def trajectory_loss(
    candidate_xy: Tensor,
    candidate_speed: Tensor,
    candidate_logits: Tensor,
    target_xy: Tensor,
    target_speed: Tensor,
    valid_mask: Tensor,
    config: TrajectoryLossConfig | None = None,
    *,
    target_yaw: Tensor | None = None,
) -> dict[str, Tensor]:
    """Compute best-of-K regression, candidate classification, and metrics."""
    cfg = TrajectoryLossConfig() if config is None else config
    cfg.validate()
    _validate_shapes(
        candidate_xy,
        candidate_speed,
        candidate_logits,
        target_xy,
        target_speed,
        valid_mask,
        target_yaw,
    )
    batch, candidate_count, _, _ = candidate_xy.shape
    point_mask = valid_mask.unsqueeze(1).to(candidate_xy.dtype)
    valid_count = point_mask.sum(dim=2).clamp_min(1.0)

    xy_error = F.smooth_l1_loss(
        candidate_xy,
        target_xy.unsqueeze(1).expand_as(candidate_xy),
        reduction="none",
    ).sum(dim=-1)
    speed_error = F.smooth_l1_loss(
        candidate_speed,
        target_speed.unsqueeze(1).expand_as(candidate_speed),
        reduction="none",
    )
    per_candidate = (
        float(cfg.xy_weight) * (xy_error * point_mask).sum(dim=2) / valid_count
        + float(cfg.speed_weight) * (speed_error * point_mask).sum(dim=2) / valid_count
    )

    origin = torch.zeros_like(candidate_xy[:, :, :1])
    step_xy = torch.diff(torch.cat((origin, candidate_xy), dim=2), dim=2)
    geometric_speed = torch.linalg.norm(step_xy, dim=-1) / 0.1
    kinematic_speed_error = F.smooth_l1_loss(
        candidate_speed, geometric_speed, reduction="none"
    )
    per_candidate = per_candidate + float(cfg.kinematic_speed_weight) * (
        kinematic_speed_error * point_mask
    ).sum(dim=2) / valid_count

    yaw_error: Tensor | None = None
    yaw_point_mask: Tensor | None = None
    yaw_valid_count: Tensor | None = None
    if target_yaw is not None:
        # HH_260906 - Mask stationary steps before atan2 so physical stops have finite gradients.
        moving = step_xy.square().sum(dim=-1) > 1.0e-8
        safe_step_x = torch.where(
            moving, step_xy[..., 0], torch.ones_like(step_xy[..., 0])
        )
        safe_step_y = torch.where(
            moving, step_xy[..., 1], torch.zeros_like(step_xy[..., 1])
        )
        predicted_yaw = torch.atan2(safe_step_y, safe_step_x)
        yaw_delta = predicted_yaw - target_yaw.unsqueeze(1)
        yaw_error = torch.atan2(torch.sin(yaw_delta), torch.cos(yaw_delta)).abs()
        yaw_point_mask = point_mask * moving.to(candidate_xy.dtype)
        yaw_valid_count = yaw_point_mask.sum(dim=2).clamp_min(1.0)
        per_candidate = per_candidate + float(cfg.yaw_weight) * (
            yaw_error * yaw_point_mask
        ).sum(dim=2) / yaw_valid_count

    last_valid = _last_valid_indices(valid_mask)
    gather_xy = last_valid.view(batch, 1, 1, 1).expand(batch, candidate_count, 1, 2)
    candidate_final = candidate_xy.gather(2, gather_xy).squeeze(2)
    target_final = target_xy.gather(
        1, last_valid.view(batch, 1, 1).expand(batch, 1, 2)
    ).squeeze(1)
    final_displacement = torch.linalg.norm(
        candidate_final - target_final.unsqueeze(1), dim=-1
    )
    per_candidate = per_candidate + float(cfg.final_displacement_weight) * final_displacement

    oracle_candidate = per_candidate.detach().argmin(dim=1)
    regression_loss = per_candidate.gather(1, oracle_candidate.unsqueeze(1)).mean()
    score_loss = F.cross_entropy(candidate_logits, oracle_candidate)
    total_loss = regression_loss + float(cfg.candidate_score_weight) * score_loss

    displacement = torch.linalg.norm(candidate_xy - target_xy.unsqueeze(1), dim=-1)
    candidate_ade = (displacement * point_mask).sum(dim=2) / valid_count
    selected_candidate = candidate_logits.detach().argmax(dim=1)
    selected_ade = candidate_ade.gather(1, selected_candidate.unsqueeze(1)).squeeze(1)
    selected_fde = final_displacement.gather(
        1, selected_candidate.unsqueeze(1)
    ).squeeze(1)
    candidate_speed_mae = (
        (candidate_speed - target_speed.unsqueeze(1)).abs() * point_mask
    ).sum(dim=2) / valid_count
    selected_speed_mae = candidate_speed_mae.gather(
        1, selected_candidate.unsqueeze(1)
    ).squeeze(1)

    outputs = (
        total_loss,
        regression_loss,
        score_loss,
        candidate_ade,
        final_displacement,
        candidate_speed_mae,
    )
    if not all(bool(torch.isfinite(value).all().item()) for value in outputs):
        raise FloatingPointError("planning loss or metric overflowed to NaN or Inf")

    result = {
        "loss": total_loss,
        "regression_loss": regression_loss.detach(),
        "candidate_score_loss": score_loss.detach(),
        "oracle_ade_m": candidate_ade.min(dim=1).values.mean().detach(),
        "selected_ade_m": selected_ade.mean().detach(),
        "selected_fde_m": selected_fde.mean().detach(),
        "selected_speed_mae_mps": selected_speed_mae.mean().detach(),
        "selected_kinematic_speed_mae_mps": (
            (geometric_speed - candidate_speed).abs() * point_mask
        ).sum(dim=2).div(valid_count).gather(
            1, selected_candidate.unsqueeze(1)
        ).mean().detach(),
        "oracle_candidate_index": oracle_candidate,
        "selected_candidate_index": selected_candidate,
    }
    if yaw_error is not None:
        result["selected_yaw_mae_rad"] = yaw_error.mul(yaw_point_mask).sum(dim=2).div(
            yaw_valid_count
        ).gather(1, selected_candidate.unsqueeze(1)).mean().detach()
    return result
