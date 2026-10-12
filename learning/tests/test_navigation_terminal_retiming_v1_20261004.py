"""HH_261004 - Test synthetic CPU navigation-retiming prerequisites; no actuation or performance claims."""

from dataclasses import replace
import hashlib
import json
import math

import pytest
import torch

from portable_e2e import navigation_terminal_retiming_v1_20261004 as navigation
from portable_e2e import terminal_witness_v1_20261002 as proof
from portable_e2e.contract import ContractError
from portable_e2e.path_time_decoder_terminal_v1_20261002 import TerminalDecoderError
from portable_e2e.path_time_decoder_v2 import BRAKE_TO_STOP, GO, HOLD, PathTimeDecoderV2
from portable_e2e.runtime_contract import RuntimeGateConfig, validate_and_select_trajectory


def _encoded(document):
    payload = json.dumps(document, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    return payload, hashlib.sha256(payload).hexdigest()


def _witness(support, mode=proof.VERIFIED_TERMINAL):
    # HH_261004 - Authenticated payloads below are explicitly synthetic current geometry, never TEST/teacher future captures.
    stamp = 1_000_000_000
    position = 200. - support
    mission = dict(schema=proof.MISSION_SCHEMA, coordinates=dict(proof.COORDINATES), mission_id="synthetic-nav-terminal",
        map_id="synthetic-map", route_id="synthetic-route", source_route_artifact_sha256="a" * 64,
        source_available_timestamp_ns=0, complete_mission_route=True, polyline_map_xy_m=[[0., 0.], [200., 0.]],
        goal_map_xy_m=[200., 0.], stop_reference_offset_m=0.)
    samples = [float(i) for i in range(math.floor(support) + 1)]
    if support - samples[-1] > 1.e-6:
        samples.append(support)
    observation = dict(schema=proof.OBSERVATION_SCHEMA, coordinates=dict(proof.COORDINATES), mission_id=mission["mission_id"],
        map_id=mission["map_id"], route_id=mission["route_id"], source_route_artifact_sha256="a" * 64, sequence_index=0,
        anchor_timestamp_ns=stamp, ego_timestamp_ns=stamp, pose_timestamp_ns=stamp, navigation_timestamp_ns=stamp,
        camera_timestamp_ns=[stamp] * 6, camera_source_sha256=[str(i) * 64 for i in range(1, 7)],
        ego_source_sha256="b" * 64, pose_source_sha256="c" * 64, position_map_m=[position, 0., .1], orientation_xyzw=[0., 0., 0., 1.],
        provided_route_xy_base_m=[[s, 0.] for s in samples], provided_route_support_m=support,
        route_anchor_arc_m=position, remaining_mission_arc_m=support, goal_base_m=[support, 0.])
    mp, ms = _encoded(mission)
    op, os = _encoded(observation)
    decision = proof.evaluate_terminal_witness(mission_payload=mp, expected_mission_sha256=ms,
        expected_route_artifact_sha256="a" * 64, observation_payload=op, expected_observation_sha256=os,
        now_ns=stamp, requested_mode=mode)
    assert decision.status == "VERIFIED", decision
    return dict(terminal_witness=decision, terminal_anchor_timestamp_ns=stamp, terminal_now_ns=stamp,
        terminal_observation_sha256=os, terminal_mission_sha256=ms, terminal_route_artifact_sha256="a" * 64)


def _inputs(support=10., speed=2.4, preferred=2.4, curvature=0., intent=GO, dtype=torch.float32):
    args = [torch.full((1, 33), curvature, dtype=dtype), torch.full((1, 33), preferred, dtype=dtype),
        torch.tensor([speed], dtype=dtype), torch.tensor([intent]), torch.zeros(1, dtype=dtype),
        torch.tensor([support], dtype=torch.float64)]
    return args, _witness(support)


def _run(**parameters):
    args, kwargs = _inputs(**parameters)
    return navigation.NavigationTerminalRetimingV1()(*args, **kwargs), args


def _assert_gates(result, speed):
    for value in result.values():
        assert bool(torch.isfinite(value).all())
    # HH_261004 - The existing one-candidate runtime gate is configured, not weakened or bypassed.
    gate = RuntimeGateConfig(candidate_count=1)
    selected = validate_and_select_trajectory(result["xy"][0].tolist(), result["speed"][0].tolist(), [0.],
                                             gate, current_speed_mps=speed)
    assert selected.candidate_index == 0
    assert result["instantaneous_speed"][0, 0].item() == speed
    assert result["acceleration"].min().item() >= -2. - 1.e-6
    assert result["acceleration"].max().item() <= 1.5 + 1.e-6
    assert result["instantaneous_speed"].max().item() <= 2.5 + 1.e-6
    assert result["arc_distance"].max().item() <= result["navigation_stop_distance_request"].item() + 1.e-6
    assert result["navigation_controller_intervention"].item() is True
    assert result["navigation_learned_STOP_claim"].item() is False
    assert result["navigation_model_weights_changed"].item() is False
    assert result["navigation_vehicle_control_approved"].item() is False


@pytest.mark.parametrize("support", [17.9, 10., 5., 2.5])
@pytest.mark.parametrize("dtype", [torch.float32, torch.float64])
def test_real_prepared_witness_short_support_decoder_and_original_gates(support, dtype):
    result, args = _run(support=support, dtype=dtype)
    _assert_gates(result, args[2].item())
    assert result["xy"].dtype == torch.float64
    assert torch.equal(result["raw_preferred_speed_knots"], args[1].to(torch.float64))
    assert result["navigation_stop_distance_request"].item() == pytest.approx(support, abs=1.e-12)
    assert result["intent"].item() == GO
    assert result["instantaneous_speed"][0, 1].item() == args[1][0, 0].item()
    with pytest.raises(ContractError, match="GO has insufficient supported horizon"):
        PathTimeDecoderV2()(*[v.to(dtype) if i in (4, 5) else v for i, v in enumerate(args)])


@pytest.mark.parametrize("dtype", [torch.float32, torch.float64])
@pytest.mark.parametrize("mode", [GO, BRAKE_TO_STOP, HOLD])
def test_ordinary_branch_exact_outputs_and_original_errors(dtype, mode):
    args, _ = _inputs(support=64., speed=.01 if mode == HOLD else 1., dtype=dtype, intent=mode)
    args[4] = torch.tensor([3.], dtype=dtype)
    args[5] = args[5].to(dtype)
    old = PathTimeDecoderV2()(*args)
    new = navigation.NavigationTerminalRetimingV1()(*args, terminal_now_ns=True, terminal_mission_sha256="bad")
    assert old.keys() == new.keys()
    assert all(torch.equal(old[key], new[key]) for key in old)
    args[3] = torch.tensor([GO])
    args[5] = torch.tensor([10.], dtype=dtype)
    with pytest.raises(ContractError, match="GO has insufficient supported horizon"):
        navigation.NavigationTerminalRetimingV1()(*args, **_witness(10., proof.ORDINARY))


def test_actual_raw_first_physical_knot_changes_request_not_a_semantic_head():
    slow, args = _run(speed=1., preferred=.7)
    fast, _ = _run(speed=1., preferred=1.3)
    assert slow["navigation_requested_next_speed_raw"].item() == args[1][0, 0].item()
    assert slow["navigation_requested_next_speed_bounded"].item() == pytest.approx(.8)
    assert fast["navigation_requested_next_speed_bounded"].item() == pytest.approx(1.15)
    assert slow["acceleration"][0, 0].item() == pytest.approx(-2.)
    assert fast["acceleration"][0, 0].item() == pytest.approx(1.5)
    _assert_gates(slow, args[2].item())
    _assert_gates(fast, args[2].item())


def test_endpoint_cap_uses_actual_D_without_clipping_observed_v0():
    result, args = _run(support=1.5, preferred=2.5)
    _assert_gates(result, args[2].item())
    v0 = args[2].item()
    v1 = result["instantaneous_speed"][0, 1].item()
    assert v1 < v0
    assert result["instantaneous_speed"][0, 0].item() == v0
    assert .5 * (v0 + v1) * .1 + v1 * v1 / 4 <= 1.5 + 1.e-12


def test_curvature_cap_uses_actual_first_prefix_and_retains_raw_field():
    result, args = _run(speed=1.7, preferred=2.5, curvature=.45, support=2.5)
    cap = math.sqrt(1.5 / args[0][0, 0].item())
    assert result["navigation_first_speed_lateral_cap"].item() == pytest.approx(cap)
    assert result["navigation_requested_next_speed_bounded"].item() <= cap + 1.e-12
    assert torch.equal(result["spatial_curvature"], torch.full_like(result["spatial_curvature"], args[0][0, 0].item()))
    # HH_261004 - Strong curvature during terminal deceleration may fail the unchanged chord-heading gate despite numerical feasibility.
    with pytest.raises(ContractError, match="curvature"):
        _assert_gates(result, args[2].item())
    args, kwargs = _inputs(speed=2.4, preferred=2.5, curvature=.5)
    with pytest.raises(TerminalDecoderError, match="no feasible request interval"):
        navigation.NavigationTerminalRetimingV1()(*args, **kwargs)


def test_moderate_curvature_terminal_plan_must_pass_existing_geometric_gate():
    result, args = _run(speed=2.4, preferred=2.4, curvature=.1, support=2.5)
    _assert_gates(result, args[2].item())


def test_exact_curvature_boundary_keeps_independent_geometric_gate_rejection():
    # HH_261004 - Decoder full-cell physical feasibility never overrides the existing chord-based curvature gate.
    result, args = _run(speed=1.7, preferred=2.5, curvature=.5)
    assert result["navigation_first_speed_lateral_cap"].item() == pytest.approx(math.sqrt(3.))
    with pytest.raises(ContractError, match="curvature"):
        _assert_gates(result, args[2].item())


def test_departure_true_zero_is_not_stationary_zero_endpoint():
    result, args = _run(speed=0., preferred=1., support=2.5)
    _assert_gates(result, args[2].item())
    assert result["instantaneous_speed"][0, 1].item() == pytest.approx(.15)
    assert result["arc_distance"][0, 1].item() > 0
    assert result["intent"].item() == GO


@pytest.mark.parametrize("mode", [GO, BRAKE_TO_STOP, HOLD])
def test_true_positive_residual_early_stop_is_not_fabricated_zero(mode):
    result, args = _run(speed=.049, preferred=2.4, support=.001, intent=mode)
    _assert_gates(result, args[2].item())
    assert result["navigation_actual_early_stop_request"].item() is True
    assert result["terminal_numeric_regime"].item() == 1
    assert result["instantaneous_speed"][0, 0].item() > 0
    assert result["instantaneous_speed"][0, 1:].sum().item() == 0
    assert result["arc_distance"][0, 1].item() == pytest.approx(.001, abs=1.e-12)
    assert result["intent"].item() == mode


def test_stationary_zero_endpoint_requires_explicit_true_zero_HOLD():
    result, args = _run(speed=0., preferred=0., support=0., intent=HOLD)
    _assert_gates(result, args[2].item())
    assert result["instantaneous_speed"].sum().item() == 0
    assert result["spatial_arc_m"].shape == (1, 1)
    with pytest.raises(TerminalDecoderError, match="explicit HOLD"):
        _run(speed=0., preferred=0., support=0., intent=GO)


def test_explicit_HOLD_positive_residual_never_requests_acceleration():
    result, args = _run(speed=.049, preferred=2.4, support=.3, intent=HOLD)
    _assert_gates(result, args[2].item())
    assert result["navigation_initial_acceleration_request"].item() == 0
    assert result["instantaneous_speed"][0, 0].item() == args[2].item() > 0
    assert result["instantaneous_speed"][0, 1:].max().item() <= args[2].item()


def test_lossless_float32_HOLD_boundary_is_not_clipped_into_approval():
    # HH_261004 - Actual float32 .05 is just above the strict .05 limit after lossless casting; retain rejection.
    args, kwargs = _inputs(speed=.05, intent=HOLD)
    assert args[2].item() > .05
    with pytest.raises(TerminalDecoderError, match="HOLD cannot replace"):
        navigation.NavigationTerminalRetimingV1()(*args, **kwargs)
    result, args = _run(speed=.05, preferred=2.4, support=.3, intent=HOLD, dtype=torch.float64)
    _assert_gates(result, args[2].item())


@pytest.mark.parametrize("speed", [-.01, 2.500001, float("nan"), float("inf")])
def test_invalid_observed_speed_rejects_without_clipping(speed):
    with pytest.raises(TerminalDecoderError):
        _run(speed=speed)


def test_impossible_endpoint_and_farther_first_stop_reject_without_fallback():
    with pytest.raises(TerminalDecoderError, match="shorter than measured"):
        _run(speed=2.4, support=1.)
    with pytest.raises(TerminalDecoderError, match="farther navigation endpoint"):
        _run(speed=.05, preferred=0., support=1.)
    with pytest.raises(TerminalDecoderError, match="HOLD cannot replace"):
        _run(speed=.050001, intent=HOLD)


@pytest.mark.parametrize("field,value", [
    ("terminal_now_ns", 1_150_000_001), ("terminal_anchor_timestamp_ns", 999_999_999),
    ("terminal_observation_sha256", "d" * 64), ("terminal_mission_sha256", "d" * 64),
    ("terminal_route_artifact_sha256", "d" * 64), ("terminal_witness", {"status": "VERIFIED"}),
])
def test_unknown_stale_or_wrong_source_witness_rejects(field, value):
    args, kwargs = _inputs()
    kwargs[field] = value
    with pytest.raises(TerminalDecoderError):
        navigation.NavigationTerminalRetimingV1()(*args, **kwargs)


def test_truncated_fake_support_and_permission_forgery_reject():
    args, kwargs = _inputs()
    args[5] = torch.tensor([18.], dtype=torch.float64)
    with pytest.raises(TerminalDecoderError, match="support must equal"):
        navigation.NavigationTerminalRetimingV1()(*args, **kwargs)
    args, kwargs = _inputs()
    decision = kwargs["terminal_witness"]
    kwargs["terminal_witness"] = replace(decision, witness=replace(decision.witness, provided_prefix_reaches_mission_endpoint=False))
    with pytest.raises(TerminalDecoderError, match="truncated/ordinary"):
        navigation.NavigationTerminalRetimingV1()(*args, **kwargs)


def test_no_raw_semantic_stop_or_initial_acceleration_argument():
    args, kwargs = _inputs()
    args[4] = torch.tensor([1.], dtype=torch.float32)
    with pytest.raises(TerminalDecoderError, match="placeholder must be zero"):
        navigation.NavigationTerminalRetimingV1()(*args, **kwargs)
    args[4].zero_()
    with pytest.raises(TypeError, match="initial_acceleration_prediction"):
        navigation.NavigationTerminalRetimingV1()(*args, initial_acceleration_prediction=torch.tensor([0.]), **kwargs)


def test_frozen_three_source_closure_and_prepared_no_actuation_contract(monkeypatch):
    contract = navigation.prepared_contract()
    assert contract["source_sha256"] == navigation.FROZEN_SOURCE_SHA256
    assert contract["prepared_cpu_only"] is True
    assert contract["prepared_not_deployed"] is True
    assert contract["vehicle_control_approved"] is False
    assert contract["main_functions_approved"] == contract["subfunctions_approved"] == 0
    monkeypatch.setattr(navigation, "_source_sha256", lambda _: "0" * 64)
    with pytest.raises(TerminalDecoderError, match="source closure differs"):
        navigation.NavigationTerminalRetimingV1()
