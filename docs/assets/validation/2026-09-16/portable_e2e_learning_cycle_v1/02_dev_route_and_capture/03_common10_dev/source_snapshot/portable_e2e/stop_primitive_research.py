"""HH_260906 - Research-only differentiable stop geometry, not a trained policy or deployable model.

HH_260906 - The caller already chooses STOP; no future labels, stop decision,
obstacles, route following, actuator commands or runtime admission are provided.
HH_260906 - Keep this module separate from existing model IDs and checkpoints.
"""

from __future__ import annotations

from dataclasses import dataclass

import torch
from torch import Tensor


RESEARCH_ID = "portable_e2e.stop_primitive.research.v1"
TIME_STEP_S = 0.1
FUTURE_POINTS = 64
CANDIDATE_COUNT = 6
MAX_SPEED_MPS = 30.0 / 3.6
MAX_DECELERATION_MPS2 = 2.9
MAX_CURVATURE_RAD_PER_M = 0.2
MAX_LATERAL_ACCELERATION_MPS2 = 2.8


@dataclass(frozen=True)
class StopPrimitiveResult:
    """HH_260906 - Return nominal discrete geometry without scores, labels or permission to control a vehicle."""

    xy_base_m: Tensor
    speed_mps: Tensor
    heading_rad: Tensor
    stop_duration_s: Tensor


def _check_inputs(current_speed_mps: Tensor, duration_logits: Tensor, curvature_logits: Tensor) -> None:
    """HH_260906 - Reject unsupported inputs rather than clipping invalid motion into an apparently legal stop."""
    inputs = {"current_speed_mps": current_speed_mps, "duration_logits": duration_logits,
              "curvature_logits": curvature_logits}
    for name, tensor in inputs.items():
        if not isinstance(tensor, Tensor):
            raise ValueError(f"{name} must be a torch Tensor")
        if tensor.layout != torch.strided or tensor.dtype not in (torch.float32, torch.float64):
            raise ValueError(f"{name} must use dense float32 or float64")
        if tensor.dtype != current_speed_mps.dtype or tensor.device != current_speed_mps.device:
            raise ValueError("stop inputs must share dtype and device")
        if not bool(torch.isfinite(tensor).all().item()):
            raise ValueError(f"{name} must be finite")
    if current_speed_mps.ndim != 1 or current_speed_mps.shape[0] == 0:
        raise ValueError("current_speed_mps must have a nonempty [batch] shape")
    batch = current_speed_mps.shape[0]
    if tuple(duration_logits.shape) != (batch, CANDIDATE_COUNT):
        raise ValueError("duration_logits must have shape [batch, 6]")
    if tuple(curvature_logits.shape) != (batch, CANDIDATE_COUNT, FUTURE_POINTS):
        raise ValueError("curvature_logits must have shape [batch, 6, 64]")
    # HH_260906 - Negative jitter normalization belongs to an external observed-input contract, not this primitive.
    if bool(torch.any(current_speed_mps < 0.0).item()) or bool(torch.any(current_speed_mps > MAX_SPEED_MPS).item()):
        raise ValueError("current_speed_mps must be between zero and 30 km/h")


def decode_stop_primitive(
    current_speed_mps: Tensor,
    duration_logits: Tensor,
    curvature_logits: Tensor,
) -> StopPrimitiveResult:
    """HH_260906 - Decode an already requested STOP without reacceleration or changing the existing runtime gate.

    HH_260906 - Tmin=max(dt,v0/amax), T=Tmin+sigmoid(z)*(horizon-Tmin).
    HH_260906 - Speed is v0*max(0,1-t/T), with explicit zero at/after T.
    HH_260906 - XY uses end-of-interval speed times dt, matching the existing
    discrete output convention; it is not the continuous braking-distance integral.
    HH_260906 - Curvature is bounded using interval-entry speed, but no road,
    collision, actuator tracking, jerk or learned decision guarantee follows.
    """
    _check_inputs(current_speed_mps, duration_logits, curvature_logits)
    horizon_s = FUTURE_POINTS * TIME_STEP_S
    minimum_duration = torch.clamp(current_speed_mps / MAX_DECELERATION_MPS2, min=TIME_STEP_S)
    duration = minimum_duration[:, None] + torch.sigmoid(duration_logits) * (
        horizon_s - minimum_duration[:, None]
    )
    # HH_260906 - Bound rounding at the fixed horizon, without changing any caller speed or latent tensor.
    duration = torch.clamp(duration, max=horizon_s)
    times = torch.arange(1, FUTURE_POINTS + 1, dtype=current_speed_mps.dtype,
                         device=current_speed_mps.device) * TIME_STEP_S
    unconstrained_speed = current_speed_mps[:, None, None] * (1.0 - times / duration[..., None])
    speed = torch.where(times >= duration[..., None], torch.zeros_like(unconstrained_speed),
                        torch.clamp(unconstrained_speed, min=0.0))
    entry_speed = torch.cat((current_speed_mps[:, None, None].expand(-1, CANDIDATE_COUNT, 1),
                             speed[:, :, :-1]), dim=2)
    curvature_bound = torch.clamp(
        MAX_LATERAL_ACCELERATION_MPS2 / torch.clamp(entry_speed.square(), min=1.0e-6),
        max=MAX_CURVATURE_RAD_PER_M,
    )
    curvature = torch.tanh(curvature_logits) * curvature_bound
    step_distance = speed * TIME_STEP_S
    heading = torch.cumsum(curvature * step_distance, dim=2)
    step_xy = torch.stack((torch.cos(heading), torch.sin(heading)), dim=-1) * step_distance[..., None]
    xy = torch.cumsum(step_xy, dim=2)
    return StopPrimitiveResult(xy_base_m=xy, speed_mps=speed, heading_rad=heading, stop_duration_s=duration)
