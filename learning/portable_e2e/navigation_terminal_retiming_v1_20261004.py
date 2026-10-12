"""HH_261004 - Prepare an explicit navigation-constrained terminal controller, not learned STOP.

The ordinary decoder branch delegates unchanged.  A verified COMPLETE-mission
witness alone enables the CPU-only new terminal ABI.  Navigation owns physical
D; the first physical preferred-speed knot requests next speed.  Bounding that
REQUEST never clips measured v0, invents support, interprets untrained semantic
heads, updates weights, or grants actuation/performance approval.  All original
terminal numerical rejection and downstream runtime/body/deadline gates remain.
"""

from __future__ import annotations

import hashlib
import math
from pathlib import Path

import torch
from torch import Tensor, nn

from .path_time_decoder_terminal_v1_20261002 import PathTimeDecoderTerminalV1, TerminalDecoderError
from .path_time_decoder_v2 import BRAKE_TO_STOP, GO, HOLD, DecoderConfigV2
from .terminal_witness_v1_20261002 import ORDINARY, TerminalWitness, TerminalWitnessDecision


SCHEMA = "portable_e2e.navigation_terminal_retiming.prepared.20261004.v1"
FROZEN_SOURCE_SHA256 = {
    "path_time_decoder_v2.py": "d80e370d52c73591df8a593a0483e225cb4cf2595e8389a91400f68f94eea7e7",
    "path_time_decoder_terminal_v1_20261002.py": "df9beb1d7f71c6757907f7b6e7792ba3d9b7e2679d75c165b952cf6f6895943a",
    "terminal_witness_v1_20261002.py": "52704ec0214053f7942a2657a76bf18fed2cbbbf81d729c3d512d66484672ca1",
}


def _require(condition, code, message):
    if not bool(condition):
        raise TerminalDecoderError(code, message)


def _source_sha256(filename):
    return hashlib.sha256((Path(__file__).resolve().parent / filename).read_bytes()).hexdigest()


def verify_frozen_source_closure():
    """HH_261004 - Fail closed if any of the three frozen numerical/proof sources changed."""
    actual = {name: _source_sha256(name) for name in FROZEN_SOURCE_SHA256}
    _require(actual == FROZEN_SOURCE_SHA256, "NAVIGATION_FROZEN_SOURCE_MISMATCH", "frozen ordinary/terminal/witness source closure differs")
    return actual


def _cpu_tensor(value, shape, name, *, integer=False, dtype=None):
    allowed = (torch.int32, torch.int64) if integer else (torch.float32, torch.float64)
    _require(isinstance(value, Tensor) and tuple(value.shape) == shape and value.device.type == "cpu"
             and value.dtype in allowed and (dtype is None or value.dtype == dtype),
             "NAVIGATION_TENSOR_ABI", name + " must be a finite CPU tensor of the declared shape/dtype")
    _require(bool(torch.isfinite(value).all()), "NAVIGATION_NONFINITE_INPUT", name + " must be finite")


class NavigationTerminalRetimingV1(nn.Module):
    """HH_261004 - Declare a new longitudinal request law; no simulator/control ownership here."""

    def __init__(self, config: DecoderConfigV2 | None = None):
        super().__init__()
        self.source_sha256 = verify_frozen_source_closure()
        self.decoder = PathTimeDecoderTerminalV1(config)
        self.config = self.decoder.config

    def forward(self, curvature_knots: Tensor, preferred_speed_knots: Tensor, current_speed: Tensor,
                intent: Tensor, stop_distance_m: Tensor, supported_distance_m: Tensor, *,
                terminal_witness: TerminalWitnessDecision | None = None,
                terminal_anchor_timestamp_ns=None, terminal_now_ns=None,
                terminal_observation_sha256=None, terminal_mission_sha256=None,
                terminal_route_artifact_sha256=None):
        # HH_261004 - No new tensor checks, casts, request law or output fields affect ordinary calls.
        ordinary = terminal_witness is None or (
            type(terminal_witness) is TerminalWitnessDecision and terminal_witness.status == "VERIFIED"
            and type(terminal_witness.witness) is TerminalWitness and terminal_witness.witness.mode == ORDINARY
            and terminal_witness.witness.terminal_permission is False)
        bindings = dict(terminal_witness=terminal_witness,
            terminal_anchor_timestamp_ns=terminal_anchor_timestamp_ns, terminal_now_ns=terminal_now_ns,
            terminal_observation_sha256=terminal_observation_sha256, terminal_mission_sha256=terminal_mission_sha256,
            terminal_route_artifact_sha256=terminal_route_artifact_sha256)
        if ordinary:
            return self.decoder(curvature_knots, preferred_speed_knots, current_speed, intent,
                                stop_distance_m, supported_distance_m, **bindings)

        # HH_261004 - Validate original physical NN values before lossless float64 conversion; never accept semantic/a0 heads.
        _cpu_tensor(curvature_knots, (1, 33), "curvature_knots")
        _cpu_tensor(preferred_speed_knots, (1, 33), "preferred_speed_knots", dtype=curvature_knots.dtype)
        _cpu_tensor(current_speed, (1,), "current_speed", dtype=curvature_knots.dtype)
        _cpu_tensor(intent, (1,), "intent", integer=True)
        _cpu_tensor(stop_distance_m, (1,), "legacy_stop_distance_placeholder")
        _cpu_tensor(supported_distance_m, (1,), "actual_supported_distance_m")
        _require(bool((stop_distance_m == 0).all()), "NAVIGATION_STOP_PLACEHOLDER", "navigation owns D; legacy terminal stop placeholder must be zero")
        cfg = self.config
        v0 = float(current_speed[0])
        mode = int(intent[0])
        _require(0 <= v0 <= cfg.maximum_speed_mps, "OBSERVED_SPEED_LIMIT", "measured v0 is not clipped or replaced")
        _require(mode in (GO, BRAKE_TO_STOP, HOLD), "INTENT_INVALID", "explicit GO/BRAKE/HOLD required")
        _require(bool((curvature_knots.abs() <= cfg.maximum_curvature_inv_m).all()), "CURVATURE_LIMIT", "raw curvature exceeds the unchanged bound")
        _require(bool((preferred_speed_knots >= 0).all()), "NEGATIVE_SPEED_REQUEST", "physical NN speed requests cannot be negative")
        support64 = supported_distance_m.to(torch.float64)
        witness, distance = self.decoder._witness(terminal_witness, support64,
            terminal_anchor_timestamp_ns=terminal_anchor_timestamp_ns, terminal_now_ns=terminal_now_ns,
            terminal_observation_sha256=terminal_observation_sha256, terminal_mission_sha256=terminal_mission_sha256,
            terminal_route_artifact_sha256=terminal_route_artifact_sha256)
        _require(witness.vehicle_control_approved is False and witness.path_support_is_free_space_or_body_safety is False,
                 "NAVIGATION_WITNESS_AUTHORITY", "navigation geometry does not approve control or free space")
        _require(mode != HOLD or v0 <= cfg.hold_entry_speed_mps, "HOLD_ENTRY_SPEED", "HOLD cannot replace a moving state")
        _require(mode != BRAKE_TO_STOP or (v0 > 0 and distance > 0), "BRAKE_MOVING_STATE", "BRAKE requires actual positive speed and distance")
        dt, deceleration = cfg.timestep_s, cfg.maximum_deceleration_mps2
        desired = float(preferred_speed_knots[0, 0])
        minimum_stop = v0 * v0 / (2 * deceleration)
        _require(distance >= minimum_stop, "ENDPOINT_BRAKING_INFEASIBLE", "verified actual D is shorter than measured v0's bounded stop")

        # HH_261004 - Curvature cap concerns only the possible first100ms prefix, not an unrelated complete2m knot.
        maximum_next = min(cfg.maximum_speed_mps, v0 + cfg.maximum_acceleration_mps2 * dt)
        maximum_first_arc = min(distance, .5 * (v0 + maximum_next) * dt)
        k0, k2 = float(curvature_knots[0, 0]), float(curvature_knots[0, 1])
        maximum_prefix_curvature = max(abs(k0), abs(k0 + (k2 - k0) * maximum_first_arc / 2.))
        lateral_cap = cfg.maximum_speed_mps if maximum_prefix_curvature == 0 else min(
            cfg.maximum_speed_mps, math.sqrt(cfg.maximum_lateral_acceleration_mps2 / maximum_prefix_curvature))
        lower = max(0., v0 - deceleration * dt)
        early_stop = False
        if distance == 0:
            _require(mode == HOLD and v0 == 0, "ZERO_ENDPOINT_REQUIRES_HOLD", "true zero D requires explicit HOLD and actual zero v0")
            requested_next, a0, endpoint_cap = 0., 0., 0.
        elif v0 > 0 and distance <= .5 * v0 * dt:
            # HH_261004 - Actual positive residual may finish before100ms; no synthetic zero state or epsilon creep.
            _require(v0 / dt <= deceleration and minimum_stop <= distance,
                     "NAVIGATION_EARLY_STOP_INFEASIBLE", "small actual D must support the complete physical residual stop")
            requested_next, a0, endpoint_cap = 0., -v0 / dt, 0.
            early_stop = True
        else:
            # HH_261004 - Solve v1^2 + d*dt*v1 + d*dt*v0 <= 2*d*D, capping REQUEST only.
            discriminant = (deceleration * dt) ** 2 - 4 * deceleration * dt * v0 + 8 * deceleration * distance
            _require(discriminant >= 0, "NAVIGATION_FIRST_BOUNDARY_INFEASIBLE", "actual D has no real first100ms stopping boundary")
            endpoint_cap = (-deceleration * dt + math.sqrt(discriminant)) / 2
            upper = min(maximum_next, lateral_cap, endpoint_cap, v0 if mode == HOLD else cfg.maximum_speed_mps)
            _require(upper >= lower, "NAVIGATION_FIRST_BOUNDARY_INFEASIBLE", "bounded acceleration, curvature and actual endpoint have no feasible request interval")
            requested_next = max(lower, min(desired, upper))
            _require(not (v0 > 0 and requested_next == 0), "STOP_BEFORE_REQUESTED_ENDPOINT", "first100ms zero cannot stand in for a farther navigation endpoint")
            # HH_261004 - Bound only the requested acceleration against subtraction roundoff; actual state/distance stay unchanged.
            a0 = max(-deceleration, min(cfg.maximum_acceleration_mps2, (requested_next - v0) / dt))

        curve64, preferred64, current64 = (value.to(torch.float64) for value in
                                           (curvature_knots, preferred_speed_knots, current_speed))
        distance64 = torch.tensor([distance], dtype=torch.float64)
        a064 = torch.tensor([a0], dtype=torch.float64)
        # HH_261004 - The frozen decoder remains authoritative for exact timed prefix and all full-cell physical checks.
        result = self.decoder(curve64, preferred64, current64, intent, distance64, distance64,
                              initial_acceleration_prediction=a064, **bindings)
        result.update(
            navigation_requested_next_speed_raw=torch.tensor([desired], dtype=torch.float64),
            navigation_requested_next_speed_bounded=current64 + a064 * dt,
            navigation_initial_acceleration_request=a064,
            navigation_stop_distance_request=distance64,
            navigation_first_speed_lower_bound=torch.tensor([lower], dtype=torch.float64),
            navigation_first_speed_lateral_cap=torch.tensor([lateral_cap], dtype=torch.float64),
            navigation_first_speed_endpoint_cap=torch.tensor([endpoint_cap], dtype=torch.float64),
            navigation_first_prefix_curvature_bound=torch.tensor([maximum_prefix_curvature], dtype=torch.float64),
            navigation_actual_early_stop_request=torch.tensor([early_stop]),
            navigation_controller_intervention=torch.tensor([True]),
            navigation_model_weights_changed=torch.tensor([False]),
            navigation_learned_STOP_claim=torch.tensor([False]),
            navigation_vehicle_control_approved=torch.tensor([False]),
        )
        return result


def prepared_contract():
    """HH_261004 - Describe numerical preparation and an intervention, never completed driving."""
    return dict(schema=SCHEMA, prepared_cpu_only=True, prepared_not_deployed=True,
        source_sha256=verify_frozen_source_closure(), ordinary_outputs_errors_exact_delegate=True,
        terminal_batch_size=1, terminal_dtype="float64 lossless conversion of original float32 physical fields/v0",
        desired_next_speed="current raw preferred_speed_knots[0,0]; physical m/s, not semantic head",
        distance="min(verified actual provided arc, verified remaining mission arc,64)",
        first_boundary="s1=(v0+v1)*.1/2; s1+v1^2/(2*dmax)<=D; explicit genuine early residual stop",
        request_caps="speed, prefix curvature lateral bound, +amax/-dmax first100ms, actual endpoint; never observed-state clipping",
        unchanged_nn_curvature_and_preferred_speed=True, untrained_semantic_head_conversion=False,
        navigation_controller_intervention=True, model_or_weights_changed=False, learned_STOP_claim=False,
        fallback_to_teacher_or_fake_support=False, runtime_body_deadline_gates_unchanged=True,
        maximum_speed_mps=2.5, maximum_acceleration_mps2=1.5, maximum_deceleration_mps2=2.,
        maximum_curvature_inv_m=.5, maximum_lateral_acceleration_mps2=1.5,
        vehicle_control_approved=False, main_functions_approved=0, subfunctions_approved=0)
