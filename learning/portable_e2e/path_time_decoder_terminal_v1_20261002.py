"""HH_261002 - Prepare causal terminal retiming without deploying or training a model.

Ordinary calls delegate unchanged to the frozen V2 decoder.  Only a separately
authenticated COMPLETE-mission witness enables this new B=1 numerical ABI.
Physical D/a0 requests are explicit inputs, not teacher futures or conversions
of the currently unused raw semantic heads.  Path heading is not measured body
yaw, and this module provides no vehicle-control or free-space approval.
"""

from __future__ import annotations

import math

import torch
from torch import Tensor, nn

from .contract import ContractError
from .path_time_decoder_v2 import BRAKE_TO_STOP, GO, HOLD, DecoderConfigV2, PathTimeDecoderV2, _sinc_radians
from .terminal_witness_v1_20261002 import (
    MAXIMUM_INPUT_AGE_NS, ORDINARY, SCHEMA as WITNESS_SCHEMA, VERIFIED_TERMINAL,
    TerminalWitness, TerminalWitnessDecision,
)


SCHEMA = "portable_e2e.causal_terminal_path_time_decoder.20261002.v1"
FROZEN_ORDINARY_DECODER_SHA256 = "d80e370d52c73591df8a593a0483e225cb4cf2595e8389a91400f68f94eea7e7"


class TerminalDecoderError(ContractError):
    """HH_261002 - Preserve explicit numerical rejection rather than permissive fallback."""
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


def _require(condition, code, message):
    if not bool(condition):
        raise TerminalDecoderError(code, message)


def _sha(value):
    return type(value) is str and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def _nonnegative_sqrt(value: Tensor) -> Tensor:
    # HH_261002 - An inactive exact-zero square uses a finite zero subgradient, not an epsilon speed floor.
    positive = value > 0
    return torch.where(positive, torch.sqrt(torch.where(positive, value, torch.ones_like(value))), torch.zeros_like(value))


def _roundoff_leq(left: Tensor, right: Tensor) -> Tensor:
    # HH_261002 - Admit only relative float64 arithmetic roundoff at an exact boundary; returned physical bounds still audit independently.
    magnitude = torch.maximum(left.abs(), right.abs())
    return left <= right + 8 * torch.finfo(torch.float64).eps * magnitude


class PathTimeDecoderTerminalV1(nn.Module):
    """HH_261002 - Bound a requested first stop on actual mission support; no intent relabelling.

Terminal GO may request a future stop while its current intent remains GO.
BRAKE uses the same variable-speed retiming, not constant deceleration except
the explicitly diagnosed sub-100ms residual-stop boundary case.  HOLD requires
measured v0<=.05 and retains positive residual motion.  A predicted stop beyond
6.4 seconds is reported, never presented as an observed/completed goal stop.
"""

    def __init__(self, config: DecoderConfigV2 | None = None):
        super().__init__()
        self.ordinary_decoder = PathTimeDecoderV2(config)
        self.config = self.ordinary_decoder.config

    def _witness(self, decision, support, *, terminal_anchor_timestamp_ns, terminal_now_ns,
                 terminal_observation_sha256, terminal_mission_sha256, terminal_route_artifact_sha256):
        _require(type(decision) is TerminalWitnessDecision and decision.status == "VERIFIED"
                 and type(decision.witness) is TerminalWitness, "WITNESS_REJECTED", "a verified new witness decision is required")
        witness = decision.witness
        _require(witness.schema == WITNESS_SCHEMA and witness.mode == VERIFIED_TERMINAL
                 and witness.terminal_permission is True and witness.provided_prefix_reaches_mission_endpoint is True,
                 "TERMINAL_PERMISSION_MISSING", "a truncated/ordinary route cannot authorize terminal retiming")
        for actual, expected in (
            (witness.observation_sha256, terminal_observation_sha256),
            (witness.mission_declaration_sha256, terminal_mission_sha256),
            (witness.route_artifact_sha256, terminal_route_artifact_sha256),
        ):
            _require(_sha(actual) and _sha(expected) and actual == expected, "WITNESS_SOURCE_MISMATCH", "exact externally bound witness pins required")
        _require(type(terminal_anchor_timestamp_ns) is int and type(terminal_now_ns) is int
                 and terminal_anchor_timestamp_ns == witness.anchor_timestamp_ns
                 and 0 <= terminal_now_ns - terminal_anchor_timestamp_ns <= MAXIMUM_INPUT_AGE_NS,
                 "WITNESS_CLOCK_MISMATCH", "witness must bind the current measured input anchor and remain fresh")
        remaining = witness.remaining_mission_arc_m
        provided = witness.provided_route_arc_m
        _require(type(remaining) in (int, float) and type(provided) in (int, float)
                 and math.isfinite(remaining) and math.isfinite(provided) and remaining >= 0 and provided >= 0,
                 "WITNESS_SUPPORT_INVALID", "actual nonnegative mission/provided arcs required")
        limit = min(remaining, provided, self.config.spatial_extent_m)
        _require(abs(float(support.detach().cpu()[0]) - limit) <= self.config.tolerance,
                 "CAUSAL_SUPPORT_MISMATCH", "support must equal the verified real mission/provided arc bound")
        return witness, limit

    def _inputs(self, curvature, preferred, current, intent, stop, support, a0):
        _require(isinstance(curvature, Tensor) and tuple(curvature.shape) == (1, 33)
                 and curvature.dtype in (torch.float32, torch.float64), "TERMINAL_TENSOR_ABI", "new terminal ABI requires B=1 float32/float64 [1,33]")
        for name, value, shape in (
            ("preferred_speed_knots", preferred, (1, 33)), ("current_speed", current, (1,)),
            ("stop_distance_m", stop, (1,)), ("supported_distance_m", support, (1,)),
            ("initial_acceleration_prediction", a0, (1,)),
        ):
            _require(isinstance(value, Tensor) and tuple(value.shape) == shape
                     and value.dtype == curvature.dtype and value.device == curvature.device,
                     "TERMINAL_TENSOR_ABI", name + " must match the explicit shape/dtype/device")
        _require(isinstance(intent, Tensor) and tuple(intent.shape) == (1,)
                 and intent.dtype in (torch.int32, torch.int64) and intent.device == curvature.device,
                 "TERMINAL_TENSOR_ABI", "intent must be the same-device integer [1] tensor")
        _require(all(bool(torch.isfinite(v).all()) for v in (curvature, preferred, current, stop, support, a0)),
                 "NONFINITE_TERMINAL_INPUT", "all raw knots and physical requests must be finite")
        cfg = self.config
        _require(bool((curvature.abs() <= cfg.maximum_curvature_inv_m).all()), "CURVATURE_LIMIT", "raw curvature exceeds the unchanged bound")
        _require(bool((preferred >= 0).all()), "NEGATIVE_SPEED_REQUEST", "terminal requests may be zero but never negative")
        _require(bool(((current >= 0) & (current <= cfg.maximum_speed_mps)).all()),
                 "OBSERVED_SPEED_LIMIT", "actual v0 is neither clipped nor replaced")
        _require(bool(((intent >= GO) & (intent <= HOLD)).all()), "INTENT_INVALID", "explicit GO/BRAKE/HOLD required")
        _require(bool(((support >= 0) & (support <= cfg.spatial_extent_m)).all()), "SUPPORT_LIMIT", "actual support must be in [0,64]")
        _require(bool(((stop >= 0) & (stop <= support + cfg.tolerance)).all()), "ENDPOINT_OUTSIDE_SUPPORT", "predicted physical D must lie within verified real support")
        _require(bool(((a0 >= -cfg.maximum_deceleration_mps2) & (a0 <= cfg.maximum_acceleration_mps2)).all()),
                 "INITIAL_ACCELERATION_LIMIT", "physical first-100ms mean acceleration request is out of bounds")
        mode = int(intent[0])
        _require(mode != HOLD or bool(current[0] <= cfg.hold_entry_speed_mps), "HOLD_ENTRY_SPEED", "HOLD cannot replace a moving state")
        _require(mode != HOLD or bool(a0[0] <= 0), "HOLD_ACCELERATION_REQUEST", "new terminal HOLD cannot accelerate its measured residual")
        _require(mode != BRAKE_TO_STOP or bool((current[0] > 0) & (stop[0] > 0)), "BRAKE_MOVING_STATE", "BRAKE requires actual positive speed and endpoint")
        if bool(current[0] == 0) and mode == HOLD:
            _require(bool((stop[0] == 0) & (a0[0] == 0)), "STATIONARY_HOLD_REQUEST", "true zero HOLD must request zero D/a0, not invented motion")
        else:
            _require(bool(stop[0] > 0), "ZERO_ENDPOINT_REQUIRES_HOLD", "zero-arc stationary exception is explicit true-zero HOLD only")
        return mode

    def _grid(self, distance, boundary):
        # HH_261002 - Keep original 0..64m raw coordinates; no request stretching, duplicate padding or fake support.
        regular = torch.arange(257, dtype=distance.dtype, device=distance.device) * self.config.spatial_step_m
        # HH_261002 - D scales sampling locations only, enabling real short departure/stop motion without stretching raw fields.
        adaptive = distance * (torch.arange(257, dtype=distance.dtype, device=distance.device) / 256.)
        grid = torch.cat((regular[(regular > 0) & (regular < distance)], adaptive, distance.reshape(1), boundary.reshape(1)))
        grid = torch.sort(grid).values
        keep = torch.cat((torch.ones(1, dtype=torch.bool, device=grid.device), grid[1:] > grid[:-1]))
        grid = grid[keep]
        _require(bool((grid[1:] > grid[:-1]).all()), "ZERO_LENGTH_CELL", "only exact duplicate boundaries may be removed")
        return grid

    def _raw_at(self, raw, grid):
        index = torch.floor(grid / 2.).to(torch.long).clamp(0, 31)
        fraction = (grid - index * 2.) / 2.
        return raw[0, index] + fraction * (raw[0, index + 1] - raw[0, index])

    def _shape(self, curvature, grid):
        dense = self._raw_at(curvature, grid)
        ds = grid[1:] - grid[:-1]
        cell = .5 * (dense[:-1] + dense[1:])
        turn = cell * ds
        heading = torch.cat((turn.new_zeros(1), turn.cumsum(0)))
        middle = heading[:-1] + .5 * turn
        length = ds * _sinc_radians(.5 * turn)
        delta = length[:, None] * torch.stack((middle.cos(), middle.sin()), dim=-1)
        xy = torch.cat((delta.new_zeros((1, 2)), delta.cumsum(0)), dim=0)
        return dense, heading, xy, cell

    def _caps(self, dense):
        cfg = self.config
        cell_k = torch.maximum(dense[:-1].abs(), dense[1:].abs())
        node_k = torch.cat((cell_k[:1], torch.maximum(cell_k[:-1], cell_k[1:]), cell_k[-1:]))
        # HH_261002 - A zero curvature cap is genuinely vmax, not an artificial epsilon curvature.
        safe_k = torch.where(node_k > 0, node_k, torch.ones_like(node_k))
        lateral = torch.where(node_k > 0, torch.sqrt(cfg.maximum_lateral_acceleration_mps2 / safe_k), torch.full_like(node_k, cfg.maximum_speed_mps))
        return torch.minimum(lateral, torch.full_like(node_k, cfg.maximum_speed_mps)), cell_k

    def _envelope(self, grid, request, cap, initial, mode):
        cfg = self.config
        ds = grid[1:] - grid[:-1]
        physical = [cap[-1] * 0]
        for i in range(len(ds) - 1, -1, -1):
            physical.append(torch.minimum(cap[i], _nonnegative_sqrt(physical[-1].square() + 2 * cfg.maximum_deceleration_mps2 * ds[i])))
        physical = torch.stack(physical[::-1])
        _require(bool(initial <= physical[0] + cfg.tolerance), "INITIAL_BOUNDARY_INFEASIBLE", "first100ms velocity cannot reach the remaining physical envelope")
        floor = _nonnegative_sqrt((initial.square() - 2 * cfg.maximum_deceleration_mps2 * (grid - grid[0])).clamp_min(0))
        desired = torch.minimum(torch.maximum(request, floor), physical)
        desired = torch.cat((desired[:-1], desired.new_zeros(1)))
        backward = [desired[-1]]
        for i in range(len(ds) - 1, -1, -1):
            backward.append(torch.minimum(desired[i], _nonnegative_sqrt(backward[-1].square() + 2 * cfg.maximum_deceleration_mps2 * ds[i])))
        backward = torch.stack(backward[::-1])
        _require(bool(initial <= backward[0] + cfg.tolerance), "SPEED_REQUEST_INFEASIBLE", "requests cannot hide an infeasible initial velocity")
        forward = [initial]
        # HH_261002 - Prepared HOLD retains residual motion but never accelerates; old ordinary HOLD remains untouched.
        acceleration_limit = 0. if mode == HOLD else cfg.maximum_acceleration_mps2
        for i in range(len(ds)):
            forward.append(torch.minimum(backward[i + 1], _nonnegative_sqrt(forward[-1].square() + 2 * acceleration_limit * ds[i])))
        speed = torch.stack(forward)
        _require(bool((speed[:-1] + speed[1:] > 0).all()), "UNREACHABLE_SPEED_CELL", "positive-arc zero/zero cells cannot be retimed or replaced by creep")
        _require(bool((speed[1:-1] > 0).all()), "STOP_BEFORE_REQUESTED_ENDPOINT", "an interior stop is not the declared first-stop endpoint")
        return speed

    def _sample_shape(self, distance, grid, dense, heading, xy):
        index = torch.searchsorted(grid.contiguous(), distance.contiguous(), right=True).sub(1).clamp(0, len(grid) - 2)
        elapsed = distance - grid[index]
        cell_k = .5 * (dense[index] + dense[index + 1])
        turn = cell_k * elapsed
        angle = heading[index] + .5 * turn
        length = elapsed * _sinc_radians(.5 * turn)
        return xy[index] + length[:, None] * torch.stack((angle.cos(), angle.sin()), dim=-1), heading[index] + turn

    def _audit(self, arc, velocity, spatial_speed, grid, dense, cell_k, initial, requested_a0, expected_first_arc):
        cfg = self.config
        _require(all(bool(torch.isfinite(v).all()) for v in (arc, velocity, spatial_speed)), "NONFINITE_RETIMING", "retiming must remain finite")
        _require(bool((arc[1:] >= arc[:-1] - cfg.tolerance).all()) and bool((arc >= -cfg.tolerance).all())
                 and bool((arc <= grid[-1] + cfg.tolerance).all()), "ARC_BOUNDARY", "progress must be monotone within the actual endpoint")
        _require(bool(((velocity >= 0) & (velocity <= cfg.maximum_speed_mps + cfg.tolerance)).all()), "RETIME_SPEED_LIMIT", "time velocities violate unchanged bounds")
        acceleration = (velocity[1:] - velocity[:-1]) / cfg.timestep_s
        _require(bool(((acceleration >= -cfg.maximum_deceleration_mps2 - cfg.tolerance)
                       & (acceleration <= cfg.maximum_acceleration_mps2 + cfg.tolerance)).all()), "RETIME_ACCELERATION_LIMIT", "all returned100ms mean rates must satisfy unchanged bounds")
        _require(bool(torch.equal(velocity[:1], initial)) and bool((acceleration[:1] - requested_a0).abs().max() <= cfg.tolerance),
                 "INITIAL_BOUNDARY_MISMATCH", "t0 and real first100ms boundary must match the requests")
        _require(bool(_roundoff_leq(arc[1], expected_first_arc) & _roundoff_leq(expected_first_arc, arc[1])),
                 "INITIAL_ARC_BOUNDARY_MISMATCH", "first100ms displacement must match the exact timed prefix or true earlier stop")
        if len(grid) > 1:
            ds = grid[1:] - grid[:-1]
            rate = (spatial_speed[1:].square() - spatial_speed[:-1].square()) / (2 * ds)
            _require(bool(((rate >= -cfg.maximum_deceleration_mps2 - cfg.tolerance)
                           & (rate <= cfg.maximum_acceleration_mps2 + cfg.tolerance)).all()), "SPATIAL_ACCELERATION_LIMIT", "every positive-length cell requires bounded true acceleration")
            bound = torch.maximum(spatial_speed[:-1].square(), spatial_speed[1:].square()) * cell_k
            _require(bool((bound <= cfg.maximum_lateral_acceleration_mps2 + cfg.tolerance).all()), "LATERAL_ACCELERATION_LIMIT", "full-cell curvature/speed envelope is infeasible")
        _require(bool((dense.abs() <= cfg.maximum_curvature_inv_m + cfg.tolerance).all()), "CURVATURE_LIMIT", "sampled raw curvature violates unchanged bounds")
        return acceleration

    def forward(self, curvature_knots: Tensor, preferred_speed_knots: Tensor, current_speed: Tensor,
                intent: Tensor, stop_distance_m: Tensor, supported_distance_m: Tensor, *,
                terminal_witness: TerminalWitnessDecision | None = None, initial_acceleration_prediction: Tensor | None = None,
                terminal_anchor_timestamp_ns=None, terminal_now_ns=None, terminal_observation_sha256=None,
                terminal_mission_sha256=None, terminal_route_artifact_sha256=None) -> dict[str, Tensor]:
        # HH_261002 - Do not inspect new arguments or alter old outputs/errors on the ordinary branch.
        if terminal_witness is None or (type(terminal_witness) is TerminalWitnessDecision
                and terminal_witness.status == "VERIFIED" and type(terminal_witness.witness) is TerminalWitness
                and terminal_witness.witness.mode == ORDINARY and terminal_witness.witness.terminal_permission is False):
            return self.ordinary_decoder(curvature_knots, preferred_speed_knots, current_speed, intent, stop_distance_m, supported_distance_m)
        mode = self._inputs(curvature_knots, preferred_speed_knots, current_speed, intent, stop_distance_m,
                            supported_distance_m, initial_acceleration_prediction)
        self._witness(terminal_witness, supported_distance_m, terminal_anchor_timestamp_ns=terminal_anchor_timestamp_ns,
                      terminal_now_ns=terminal_now_ns, terminal_observation_sha256=terminal_observation_sha256,
                      terminal_mission_sha256=terminal_mission_sha256, terminal_route_artifact_sha256=terminal_route_artifact_sha256)
        dtype, cfg = current_speed.dtype, self.config
        # HH_261002 - Preserve representable first-boundary arithmetic in the input dtype; only clock/arc work uses float64.
        next_speed_input = current_speed + initial_acceleration_prediction * cfg.timestep_s
        _require(bool(((next_speed_input >= 0) & (next_speed_input <= cfg.maximum_speed_mps)).all()), "INITIAL_VELOCITY_LIMIT", "requested first100ms speed is outside the unchanged range")
        v0, v1 = current_speed[0].to(torch.float64), next_speed_input[0].to(torch.float64)
        distance = stop_distance_m[0].to(torch.float64)
        minimum = v0.square() / (2 * cfg.maximum_deceleration_mps2)
        _require(bool(distance >= minimum), "ENDPOINT_BRAKING_INFEASIBLE", "D cannot be shorter than actual v0's bounded stopping distance")
        ticks = torch.arange(65, dtype=torch.float64, device=current_speed.device) * cfg.timestep_s
        s1 = .5 * (v0 + v1) * cfg.timestep_s
        stationary = mode == HOLD and bool(v0 == 0)
        early_stop = bool(v0 > 0) and bool(v1 == 0) and bool(distance <= s1)
        if stationary:
            grid = distance.new_zeros(1)
            dense = self._raw_at(curvature_knots.to(torch.float64), grid)
            heading, spatial_xy, cell_k = grid * 0, grid.new_zeros((1, 2)), grid.new_zeros(0)
            spatial_speed, arc, velocity, arrival = grid * 0, ticks * 0, ticks * 0, distance * 0
            expected_first_arc = distance * 0
            completed = torch.ones((), dtype=torch.bool, device=current_speed.device)
            regime = 0
        elif early_stop:
            # HH_261002 - A genuine small residual can stop before100ms; never turn measured residual v0 into zero.
            braking = v0.square() / (2 * distance)
            arrival = 2 * distance / v0
            _require(bool(braking <= cfg.maximum_deceleration_mps2 + cfg.tolerance)
                     and bool(arrival <= cfg.timestep_s + cfg.tolerance), "EARLY_STOP_INFEASIBLE", "early-stop physical boundary failed")
            grid = self._grid(distance, distance)
            dense, heading, spatial_xy, cell_k = self._shape(curvature_knots.to(torch.float64), grid)
            elapsed = torch.minimum(ticks, arrival)
            arc = torch.where(ticks >= arrival, distance.expand_as(ticks), v0 * elapsed - .5 * braking * elapsed.square())
            velocity = torch.where(ticks >= arrival, torch.zeros_like(ticks), v0 - braking * ticks)
            spatial_speed = _nonnegative_sqrt((v0.square() * (1 - grid / distance)).clamp_min(0))
            expected_first_arc = distance
            completed = arrival <= ticks[-1]
            regime = 1
        else:
            _require(not (bool(v0 > 0) and bool(v1 == 0)), "STOP_BEFORE_REQUESTED_ENDPOINT", "first100ms stop cannot be relabelled as a farther endpoint")
            _require(bool(_roundoff_leq(s1 + v1.square() / (2 * cfg.maximum_deceleration_mps2), distance)),
                     "INITIAL_BOUNDARY_INFEASIBLE", "D must support the exact first100ms boundary and remaining braking")
            _require(bool(distance > s1), "EMPTY_POST_PREFIX_SUPPORT", "moving terminal retiming requires actual positive remaining arc")
            grid = self._grid(distance, s1)
            dense, heading, spatial_xy, cell_k = self._shape(curvature_knots.to(torch.float64), grid)
            cap, _ = self._caps(dense)
            request = self._raw_at(preferred_speed_knots.to(torch.float64), grid)
            split = int(torch.nonzero(grid == s1, as_tuple=False)[0, 0])
            remaining_speed = self._envelope(grid[split:], request[split:], cap[split:], v1, mode)
            prefix_acceleration = (v1 - v0) / cfg.timestep_s
            prefix_speed = _nonnegative_sqrt((v0.square() + 2 * prefix_acceleration * grid[:split]).clamp_min(0))
            spatial_speed = torch.cat((prefix_speed, remaining_speed))
            ds = grid[split + 1:] - grid[split:-1]
            duration = 2 * ds / (remaining_speed[:-1] + remaining_speed[1:])
            _require(bool(torch.isfinite(duration).all()) and bool((duration > 0).all()),
                     "NONFINITE_CELL_CLOCK", "every real positive-length cell needs a finite positive duration")
            # HH_261002 - Relative clocks preserve tiny real durations and never overwrite the exact .1s prefix through rounded absolute arrival.
            boundaries = torch.cat((duration.new_zeros(1), duration.cumsum(0)))
            relative_ticks = (ticks - cfg.timestep_s).clamp_min(0)
            arrival = cfg.timestep_s + boundaries[-1]
            index = torch.searchsorted(boundaries.contiguous(), relative_ticks.contiguous(), right=True).sub(1).clamp(0, len(duration) - 1)
            elapsed = (relative_ticks - boundaries[index]).clamp_min(0)
            elapsed = torch.minimum(elapsed, duration[index])
            cell_acceleration = (remaining_speed[1:].square() - remaining_speed[:-1].square()) / (2 * ds)
            remaining_arc = grid[split:][index] + remaining_speed[index] * elapsed + .5 * cell_acceleration[index] * elapsed.square()
            remaining_velocity = remaining_speed[index] + cell_acceleration[index] * elapsed
            prefix_arc = v0 * ticks + .5 * prefix_acceleration * ticks.square()
            prefix_velocity = v0 + prefix_acceleration * ticks
            arc = torch.where(ticks <= cfg.timestep_s, prefix_arc, remaining_arc)
            velocity = torch.where(ticks <= cfg.timestep_s, prefix_velocity, remaining_velocity)
            arrived = (ticks > cfg.timestep_s) & (relative_ticks >= boundaries[-1])
            arc = torch.where(arrived, distance.expand_as(ticks), arc)
            velocity = torch.where(arrived, torch.zeros_like(velocity), velocity)
            expected_first_arc = s1
            completed = boundaries[-1] <= ticks[-1] - cfg.timestep_s
            regime = 2
        velocity = torch.cat((current_speed, velocity[1:].to(dtype)))
        acceleration = self._audit(arc, velocity, spatial_speed, grid, dense, cell_k, current_speed,
                                   initial_acceleration_prediction, expected_first_arc)
        if len(grid) == 1:
            sampled_xy, sampled_heading = spatial_xy.expand(65, 2), heading.expand(65)
        else:
            sampled_xy, sampled_heading = self._sample_shape(arc, grid, dense, heading, spatial_xy)
        _require(bool(torch.isfinite(sampled_xy).all()) and bool(torch.isfinite(sampled_heading).all()), "NONFINITE_PATH", "all XY/path-heading samples must be finite")
        xy = sampled_xy.to(dtype)
        chord = torch.norm(xy[1:] - xy[:-1], dim=-1) / cfg.timestep_s
        _require(bool((chord <= cfg.maximum_speed_mps + cfg.tolerance).all()), "CHORD_SPEED_LIMIT", "true XY chord speed exceeds the unchanged bound")
        cell_duration = 2 * (grid[1:] - grid[:-1]) / (spatial_speed[:-1] + spatial_speed[1:])
        _require(bool(torch.isfinite(cell_duration).all()) and bool((cell_duration > 0).all()),
                 "NONFINITE_CELL_CLOCK", "positive cells retain actual finite positive clocks, not artificial epsilon motion")
        return {
            "xy": xy[None, None, 1:], "speed": chord[None, None], "instantaneous_speed": velocity[None],
            "acceleration": acceleration[None], "arc_distance": arc.to(dtype)[None],
            "path_heading": sampled_heading.to(dtype)[None], "spatial_xy": spatial_xy.to(dtype)[None],
            "spatial_heading": heading.to(dtype)[None], "spatial_curvature": dense.to(dtype)[None],
            "spatial_speed": spatial_speed[None], "spatial_arc_m": grid[None],
            "spatial_cell_duration_s": cell_duration[None],
            "spatial_valid": torch.ones((1, len(grid)), dtype=torch.bool, device=current_speed.device),
            "spatial_speed_valid": torch.ones((1, len(grid)), dtype=torch.bool, device=current_speed.device),
            "active_distance_m": stop_distance_m, "intent": intent, "raw_preferred_speed_knots": preferred_speed_knots,
            "terminal_arrival_time_s": arrival[None], "terminal_stop_complete_within_horizon": completed[None],
            "initial_acceleration_request": initial_acceleration_prediction,
            "terminal_numeric_regime": torch.tensor([regime], dtype=torch.int64, device=current_speed.device),
            "terminal_spatial_node_count": torch.tensor([len(grid)], dtype=torch.int64, device=current_speed.device),
            "terminal_max_spatial_cell_m": (grid[1:] - grid[:-1]).max().reshape(1) if len(grid) > 1 else grid.reshape(1),
        }


def prepared_contract():
    """HH_261002 - Describe a new oracle prerequisite, not learned or vehicle performance."""
    return dict(schema=SCHEMA, prepared_not_deployed=True, model_or_weights_changed=False, new_learned_heads=False,
        ordinary_decoder_sha256=FROZEN_ORDINARY_DECODER_SHA256, ordinary_outputs_errors_exact_delegate=True,
        terminal_batch_size=1, raw_knot_coordinates_m="0,2,...,64; truncated, never stretched",
        spatial_sampling_grid="union(D*i/256, original0.25m knots insideD, exactfirst100ms arc); exactduplicate removal only",
        variable_grid_topology_discrete_continuous_values_piecewise_differentiable=True,
        variable_grid_arc_speed_cell_clock_diagnostics_dtype="float64; timed public output preserves input dtype",
        post_prefix_retime_uses_relative_clock=True, first100ms_arc_and_mean_acceleration_audited=True,
        physical_stop_request_not_raw_head_conversion=True, physical_a0_request_not_raw_head_conversion=True,
        support="min(verified actual provided arc, verified remaining mission arc,64)",
        stop_reference_offset_not_stop_distance_label=True, observed_future_arguments_accepted=False,
        first100ms_prefix="exact timed constant acceleration; explicit earlier residual stop and true-zero HOLD",
        zero_zero_positive_arc_cells="REJECTED; no epsilon creep", first_stop_must_be_requested_endpoint=True,
        new_terminal_HOLD_a0_nonpositive_and_residual_speed_nonincreasing=True, ordinary_HOLD_unchanged=True,
        maximum_speed_mps=2.5, maximum_acceleration_mps2=1.5, maximum_deceleration_mps2=2.,
        maximum_curvature_inv_m=.5, maximum_lateral_acceleration_mps2=1.5,
        future_points=64, timestep_s=.1, arrival_after_horizon_not_claimed_completed=True,
        path_heading_is_not_measured_body_yaw=True, support_is_not_body_or_free_space_safety=True,
        vehicle_control_approved=False, main_functions_approved=0, subfunctions_approved=0)
