"""HH_261001 - Differentiable research-only spatial path and low-speed time decoder.

This module does not approve a learned path, infer support from future labels, or
replace the independent runtime/body/latency gates.  Support is an explicit
caller contract.  Instantaneous speed, arc progress, and XY chord speed are
reported separately; only the latter has the historical trajectory-speed ABI.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import torch
from torch import Tensor, nn
from torch.nn import functional as F

from .contract import ContractError

GO, BRAKE_TO_STOP, HOLD = 0, 1, 2


def _sinc_radians(value: Tensor) -> Tensor:
    """HH_261001 - Keep the true small-angle limit differentiable on older installed Torch versions."""
    near_zero = value.abs() < 1.0e-4
    denominator = torch.where(near_zero, torch.ones_like(value), value)
    polynomial = 1 - value.square() / 6 + value.pow(4) / 120
    return torch.where(near_zero, polynomial, value.sin() / denominator)


@dataclass(frozen=True)
class DecoderConfigV2:
    """HH_261001 - Pin an initial 9 km/h research envelope, not 30 km/h approval."""

    knot_count: int = 33
    spatial_extent_m: float = 64.0
    spatial_step_m: float = 0.25
    future_points: int = 64
    timestep_s: float = 0.1
    maximum_speed_mps: float = 2.5
    maximum_acceleration_mps2: float = 1.5
    maximum_deceleration_mps2: float = 2.0
    maximum_lateral_acceleration_mps2: float = 1.5
    maximum_curvature_inv_m: float = 0.5
    minimum_preferred_speed_mps: float = 1.0e-4
    hold_entry_speed_mps: float = 0.05
    preview_margin_m: float = 0.25
    tolerance: float = 1.0e-6

    def validate(self) -> None:
        # HH_261001 - Increasing the operating envelope requires a distinct tested revision.
        if self.knot_count != 33 or self.future_points != 64:
            raise ContractError("decoder v2 requires 33 spatial knots and 64 future points")
        fixed = (self.spatial_extent_m, self.spatial_step_m, self.timestep_s)
        if fixed != (64.0, 0.25, 0.1):
            raise ContractError("decoder v2 requires 64 m, 0.25 m, and 0.1 s sampling")
        for name, limit in (
            ("maximum_speed_mps", 2.5),
            ("maximum_acceleration_mps2", 1.5),
            ("maximum_deceleration_mps2", 2.0),
            ("maximum_lateral_acceleration_mps2", 1.5),
            ("maximum_curvature_inv_m", 0.5),
            ("minimum_preferred_speed_mps", 1.0e-3),
            ("hold_entry_speed_mps", 0.05),
            ("preview_margin_m", 1.0),
            ("tolerance", 1.0e-5),
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 < value <= limit:
                raise ContractError(f"decoder.{name} must be finite and in (0, {limit}]")
        if self.minimum_preferred_speed_mps < 1.0e-4:
            raise ContractError("decoder minimum_preferred_speed_mps cannot bypass numerical conditioning")
        if self.go_active_distance_m > self.spatial_extent_m:
            raise ContractError("decoder operating horizon and braking preview exceed spatial extent")

    @property
    def go_active_distance_m(self) -> float:
        # HH_261001 - Do not propagate curvature from the unneeded 64 m tail into a low-speed plan.
        horizon = self.maximum_speed_mps * self.future_points * self.timestep_s
        braking = self.maximum_speed_mps**2 / (2 * self.maximum_deceleration_mps2)
        return math.ceil((horizon + braking + self.preview_margin_m) / self.spatial_step_m) * self.spatial_step_m


class PathTimeDecoderV2(nn.Module):
    """HH_261001 - Preserve observed v0 and reject unsupported/infeasible research plans.

    GO interprets preferred speed as a request, not an instantaneous physical
    limit: an observed speed above that request decelerates continuously.
    BRAKE_TO_STOP follows an analytic bounded deceleration to the explicit
    distance. HOLD is only legal near standstill and still accounts for its
    measured residual speed and stopping distance. Positive speed requests
    must retain a separate pre-limit supervision loss in the caller.
    """

    def __init__(self, config: DecoderConfigV2 | None = None) -> None:
        super().__init__()
        self.config = config or DecoderConfigV2()
        self.config.validate()

    def _validate_inputs(self, curvature: Tensor, preferred: Tensor, current: Tensor,
                         intent: Tensor, stop: Tensor, support: Tensor) -> None:
        if curvature.ndim != 2 or curvature.shape[1] != 33 or curvature.shape[0] == 0:
            raise ContractError("curvature_knots must have nonempty shape [B,33]")
        batch = curvature.shape[0]
        if curvature.dtype not in (torch.float32, torch.float64):
            raise ContractError("decoder requires float32 or float64 research tensors")
        for name, value, shape in (
            ("preferred_speed_knots", preferred, (batch, 33)),
            ("current_speed", current, (batch,)),
            ("stop_distance_m", stop, (batch,)),
            ("supported_distance_m", support, (batch,)),
        ):
            if tuple(value.shape) != shape or value.dtype != curvature.dtype or value.device != curvature.device:
                raise ContractError(f"{name} must have shape {shape} and match curvature dtype/device")
        if intent.shape != (batch,) or intent.device != curvature.device or intent.dtype not in (torch.int32, torch.int64):
            raise ContractError("intent must be a same-device integer tensor [B]")
        if any(not bool(torch.isfinite(value).all()) for value in (curvature, preferred, current, stop, support)):
            raise ContractError("decoder inputs must be finite")
        cfg = self.config
        if bool((curvature.abs() > cfg.maximum_curvature_inv_m).any()):
            raise ContractError("curvature exceeds the declared research limit")
        if bool((preferred < cfg.minimum_preferred_speed_mps).any()):
            raise ContractError("preferred speed knots must be strictly positive and >=1e-4 m/s for numerical conditioning; use an explicit stop intent")
        if bool(((current < 0) | (current > cfg.maximum_speed_mps)).any()):
            raise ContractError("observed current speed is outside the declared envelope; it will not be clipped")
        if bool(((intent < GO) | (intent > HOLD)).any()):
            raise ContractError("intent must be GO=0, BRAKE_TO_STOP=1, or HOLD=2")
        if bool(((stop < 0) | (stop > cfg.spatial_extent_m)).any()):
            raise ContractError("stop distance must be in [0,64] m")
        if bool(((support < 0) | (support > cfg.spatial_extent_m)).any()):
            raise ContractError("supported distance must be in [0,64] m")
        if bool(((intent == HOLD) & (current > cfg.hold_entry_speed_mps)).any()):
            raise ContractError("HOLD cannot replace a moving observed state; use BRAKE_TO_STOP")
        if bool(((intent == GO) & (support + cfg.tolerance < cfg.go_active_distance_m)).any()):
            raise ContractError("GO has insufficient supported horizon plus braking preview")
        braking = intent == BRAKE_TO_STOP
        if bool((braking & ((current <= 0) | (stop <= 0))).any()):
            raise ContractError("BRAKE_TO_STOP requires a moving observed state and positive stop distance")
        minimum_stop = current.square() / (2 * cfg.maximum_deceleration_mps2)
        safe_stop = torch.where(stop > 0, stop, torch.ones_like(stop))
        requested_deceleration = current.square() / (2 * safe_stop)
        if bool((braking & (requested_deceleration > cfg.maximum_deceleration_mps2 + cfg.tolerance)).any()):
            raise ContractError("requested stopping distance is physically infeasible at observed speed")
        required = torch.where(braking, stop, minimum_stop)
        if bool(((intent != GO) & (support + cfg.tolerance < required)).any()):
            raise ContractError("stopping the measured state requires more supported distance")

    def _spatial_shape(self, curvature: Tensor) -> tuple[Tensor, Tensor, Tensor]:
        ds = self.config.spatial_step_m
        dense = F.interpolate(curvature[:, None], size=257, mode="linear", align_corners=True)[:, 0]
        segment_curvature = 0.5 * (dense[:, :-1] + dense[:, 1:])
        turn = segment_curvature * ds
        heading = torch.cat((torch.zeros_like(turn[:, :1]), turn.cumsum(-1)), dim=-1)
        mid_heading = heading[:, :-1] + 0.5 * turn
        # HH_261001 - Exact constant-curvature segment integration has finite gradients at zero curvature.
        length = ds * _sinc_radians(0.5 * turn)
        delta = length[..., None] * torch.stack((mid_heading.cos(), mid_heading.sin()), dim=-1)
        xy = torch.cat((torch.zeros_like(delta[:, :1]), delta.cumsum(1)), dim=1)
        return dense, heading, xy

    def _sample_shape(self, distance: Tensor, dense: Tensor, heading: Tensor, xy: Tensor) -> Tensor:
        ds = self.config.spatial_step_m
        index = torch.floor(distance / ds).to(torch.long).clamp(0, 255)
        batch = torch.arange(distance.shape[0], device=distance.device)[:, None]
        elapsed_arc = distance - index * ds
        curvature = 0.5 * (dense[batch, index] + dense[batch, index + 1])
        turn = curvature * elapsed_arc
        angle = heading[batch, index] + 0.5 * turn
        length = elapsed_arc * _sinc_radians(0.5 * turn)
        return xy[batch, index] + length[..., None] * torch.stack((angle.cos(), angle.sin()), dim=-1)

    def _go_speed(self, dense: Tensor, preferred: Tensor, current: Tensor, go: Tensor) -> Tensor:
        cfg = self.config
        count = round(cfg.go_active_distance_m / cfg.spatial_step_m) + 1
        curvature = dense[:, :count]
        request = F.interpolate(preferred[:, None], size=257, mode="linear", align_corners=True)[:, 0, :count]
        # HH_261001 - Bound both ends of every active cell, not only its starting curvature.
        cell_curvature = torch.maximum(curvature[:, :-1].abs(), curvature[:, 1:].abs())
        node_curvature = torch.maximum(F.pad(cell_curvature, (1, 0)), F.pad(cell_curvature, (0, 1)))
        cap = torch.minimum(torch.full_like(curvature, cfg.maximum_speed_mps),
                            torch.sqrt(cfg.maximum_lateral_acceleration_mps2 / node_curvature.clamp_min(1.0e-8)))
        # HH_261001 - Check physical curvature feasibility independently of the learned preference.
        physical = [cap[:, -1]]
        for index in range(count - 2, -1, -1):
            physical.append(torch.minimum(cap[:, index], torch.sqrt(physical[-1].square() + 2 * cfg.maximum_deceleration_mps2 * cfg.spatial_step_m)))
        physical = torch.stack(physical[::-1], dim=-1)
        if bool((go & (current > physical[:, 0] + cfg.tolerance)).any()):
            raise ContractError("observed speed cannot reach the supported curvature envelope with bounded braking")
        arc = torch.arange(count, dtype=current.dtype, device=current.device) * cfg.spatial_step_m
        floor_square = (current[:, None].square() - 2 * cfg.maximum_deceleration_mps2 * arc).clamp_min(0)
        # HH_261001 - sqrt at exactly zero otherwise gives undefined gradients in an inactive branch.
        braking_floor = torch.where(floor_square > 0, torch.sqrt(floor_square.clamp_min(torch.finfo(current.dtype).tiny)), torch.zeros_like(floor_square))
        desired = torch.minimum(torch.maximum(request, braking_floor), physical)
        backward = [desired[:, -1]]
        for index in range(count - 2, -1, -1):
            backward.append(torch.minimum(desired[:, index], torch.sqrt(backward[-1].square() + 2 * cfg.maximum_deceleration_mps2 * cfg.spatial_step_m)))
        backward = torch.stack(backward[::-1], dim=-1)
        if bool((go & (current > backward[:, 0] + cfg.tolerance)).any()):
            raise ContractError("observed speed is inconsistent with the supported speed envelope")
        forward = [current]
        for index in range(1, count):
            forward.append(torch.minimum(backward[:, index], torch.sqrt(forward[-1].square() + 2 * cfg.maximum_acceleration_mps2 * cfg.spatial_step_m)))
        return torch.stack(forward, dim=-1)

    def _retime_go(self, spatial_speed: Tensor, current: Tensor) -> tuple[Tensor, Tensor]:
        cfg = self.config
        # HH_261001 - Float64 clock arithmetic avoids overflow for very small positive GO preferences.
        speeds = spatial_speed.to(torch.float64)
        duration = 2 * cfg.spatial_step_m / (speeds[:, :-1] + speeds[:, 1:])
        boundaries = torch.cat((torch.zeros_like(duration[:, :1]), duration.cumsum(-1)), dim=-1)
        ticks = torch.arange(65, device=current.device, dtype=torch.float64)[None].expand(current.shape[0], -1) * cfg.timestep_s
        index = torch.searchsorted(boundaries.contiguous(), ticks.contiguous(), right=True).sub(1).clamp(0, speeds.shape[1] - 2)
        batch = torch.arange(current.shape[0], device=current.device)[:, None]
        elapsed = ticks - boundaries[batch, index]
        acceleration = (speeds[:, 1:].square() - speeds[:, :-1].square()) / (2 * cfg.spatial_step_m)
        initial = speeds[batch, index]
        accel = acceleration[batch, index]
        distance = index * cfg.spatial_step_m + initial * elapsed + 0.5 * accel * elapsed.square()
        velocity = initial + accel * elapsed
        return distance.to(current.dtype), velocity.to(current.dtype)

    def forward(self, curvature_knots: Tensor, preferred_speed_knots: Tensor,
                current_speed: Tensor, intent: Tensor, stop_distance_m: Tensor,
                supported_distance_m: Tensor) -> dict[str, Tensor]:
        self._validate_inputs(curvature_knots, preferred_speed_knots, current_speed,
                              intent, stop_distance_m, supported_distance_m)
        cfg = self.config
        dense, heading, spatial_xy = self._spatial_shape(curvature_knots)
        go = intent == GO
        go_spatial = self._go_speed(dense, preferred_speed_knots, current_speed, go)
        go_arc, go_velocity = self._retime_go(go_spatial, current_speed)
        times = torch.arange(65, dtype=current_speed.dtype, device=current_speed.device)[None] * cfg.timestep_s
        minimum_stop = current_speed.square() / (2 * cfg.maximum_deceleration_mps2)
        stop_distance = torch.where(intent == BRAKE_TO_STOP, stop_distance_m, minimum_stop)
        denominator = torch.where(stop_distance > 0, stop_distance, torch.ones_like(stop_distance))
        brake_acceleration = current_speed.square() / (2 * denominator)
        # HH_261001 - HOLD with exactly zero speed has zero motion without 0/0 derivatives.
        brake_acceleration = torch.where(current_speed > 0, brake_acceleration, torch.zeros_like(brake_acceleration))
        safe_current = torch.where(current_speed > 0, current_speed, torch.ones_like(current_speed))
        time_to_stop = 2 * stop_distance / safe_current
        elapsed = torch.minimum(times, time_to_stop[:, None])
        brake_arc = current_speed[:, None] * elapsed - 0.5 * brake_acceleration[:, None] * elapsed.square()
        brake_velocity = (current_speed[:, None] - brake_acceleration[:, None] * times).clamp_min(0)
        arc = torch.where(go[:, None], go_arc, brake_arc)
        velocity = torch.where(go[:, None], go_velocity, brake_velocity)
        # HH_261001 - Never replace observed t0 by an envelope or an intent-selected zero.
        velocity = torch.cat((current_speed[:, None], velocity[:, 1:]), dim=-1)
        spatial_arc = torch.arange(257, dtype=current_speed.dtype, device=current_speed.device) * cfg.spatial_step_m
        remaining = (1 - spatial_arc[None] / denominator[:, None]).clamp_min(0)
        brake_spatial = current_speed[:, None] * torch.where(remaining > 0, torch.sqrt(remaining.clamp_min(torch.finfo(current_speed.dtype).tiny)), torch.zeros_like(remaining))
        # HH_261001 - A conservative endpoint product bounds linear curvature over each braking cell.
        interval_k = torch.maximum(dense[:, :-1].abs(), dense[:, 1:].abs())
        lateral_bound = brake_spatial[:, :-1].square() * interval_k
        relevant = spatial_arc[:-1][None] < stop_distance[:, None]
        if bool((((~go)[:, None]) & relevant & (lateral_bound > cfg.maximum_lateral_acceleration_mps2 + cfg.tolerance)).any()):
            raise ContractError("requested stop is infeasible under the supported curvature envelope")
        if not bool(torch.isfinite(arc).all()) or not bool(torch.isfinite(velocity).all()):
            raise ContractError("time retiming produced nonfinite values")
        if bool((arc[:, 1:] < arc[:, :-1] - cfg.tolerance).any()) or bool((velocity < -cfg.tolerance).any()):
            raise ContractError("time retiming reversed progress or produced negative speed")
        if bool((arc[:, -1] > supported_distance_m + cfg.tolerance).any()):
            raise ContractError("retimed plan exceeds explicitly supported distance")
        xy = self._sample_shape(arc, dense, heading, spatial_xy)
        chord_speed = torch.norm(xy[:, 1:] - xy[:, :-1], dim=-1) / cfg.timestep_s
        pad = 257 - go_spatial.shape[1]
        spatial_speed = torch.where(go[:, None], F.pad(go_spatial, (0, pad)), brake_spatial)
        active = torch.where(go, torch.full_like(current_speed, cfg.go_active_distance_m), stop_distance)
        return {
            "xy": xy[:, None, 1:], "speed": chord_speed[:, None],
            "instantaneous_speed": velocity,
            "acceleration": (velocity[:, 1:] - velocity[:, :-1]) / cfg.timestep_s,
            "arc_distance": arc, "spatial_xy": spatial_xy, "spatial_heading": heading,
            "spatial_curvature": dense, "spatial_speed": spatial_speed,
            "spatial_valid": spatial_arc[None] <= supported_distance_m[:, None],
            "spatial_speed_valid": (spatial_arc[None] <= supported_distance_m[:, None]) & (spatial_arc[None] <= active[:, None]),
            "active_distance_m": active, "intent": intent,
            "raw_preferred_speed_knots": preferred_speed_knots,
        }
