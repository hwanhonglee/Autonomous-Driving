"""HH_261002 - Synthetic numerical/causal prerequisites only; no model, actual future, simulator or actuation."""

from dataclasses import replace
import hashlib
import inspect
import json
import math

import pytest
import torch

from portable_e2e import path_time_decoder_terminal_v1_20261002 as terminal
from portable_e2e import terminal_witness_v1_20261002 as proof
from portable_e2e.contract import ContractError
from portable_e2e.path_time_decoder_v2 import BRAKE_TO_STOP, GO, HOLD, PathTimeDecoderV2


def encoded(document):
    payload = json.dumps(document, sort_keys=True, separators=(",", ":")).encode()
    return payload, hashlib.sha256(payload).hexdigest()


def witness(support=10., mode=proof.VERIFIED_TERMINAL):
    stamp = 1_000_000_000
    position = 200. - support
    mission = dict(schema=proof.MISSION_SCHEMA, coordinates=dict(proof.COORDINATES), mission_id="synthetic-complete",
        map_id="synthetic-map", route_id="synthetic-route", source_route_artifact_sha256="a" * 64,
        source_available_timestamp_ns=0, complete_mission_route=True, polyline_map_xy_m=[[0., 0.], [200., 0.]],
        goal_map_xy_m=[200., 0.], stop_reference_offset_m=.75)
    distances = [float(i) for i in range(int(math.floor(support)) + 1)]
    if support - distances[-1] > 1.e-6:
        distances.append(support)
    observation = dict(schema=proof.OBSERVATION_SCHEMA, coordinates=dict(proof.COORDINATES), mission_id=mission["mission_id"],
        map_id=mission["map_id"], route_id=mission["route_id"], source_route_artifact_sha256="a" * 64, sequence_index=0,
        anchor_timestamp_ns=stamp, ego_timestamp_ns=stamp, pose_timestamp_ns=stamp, navigation_timestamp_ns=stamp,
        camera_timestamp_ns=[stamp] * 6, camera_source_sha256=[str(i) * 64 for i in range(1, 7)],
        ego_source_sha256="b" * 64, pose_source_sha256="c" * 64, position_map_m=[position, 0., .1], orientation_xyzw=[0., 0., 0., 1.],
        provided_route_xy_base_m=[[s, 0.] for s in distances], provided_route_support_m=support,
        route_anchor_arc_m=position, remaining_mission_arc_m=support, goal_base_m=[support, 0.])
    mp, ms = encoded(mission); op, os = encoded(observation)
    decision = proof.evaluate_terminal_witness(mission_payload=mp, expected_mission_sha256=ms,
        expected_route_artifact_sha256="a" * 64, observation_payload=op, expected_observation_sha256=os,
        now_ns=stamp, requested_mode=mode)
    assert decision.status == "VERIFIED", decision
    kwargs = dict(terminal_witness=decision, terminal_anchor_timestamp_ns=stamp, terminal_now_ns=stamp,
        terminal_observation_sha256=os, terminal_mission_sha256=ms, terminal_route_artifact_sha256="a" * 64)
    return kwargs


def inputs(speed=1., preferred=1., curvature=0., intent=GO, stop=3., support=10., a0=-.2, dtype=torch.float64):
    args = [torch.full((1, 33), curvature, dtype=dtype), torch.full((1, 33), preferred, dtype=dtype),
        torch.tensor([speed], dtype=dtype), torch.tensor([intent]), torch.tensor([stop], dtype=dtype), torch.tensor([support], dtype=dtype)]
    kwargs = witness(support)
    kwargs["initial_acceleration_prediction"] = torch.tensor([a0], dtype=dtype)
    return args, kwargs


def run(**parameters):
    args, kwargs = inputs(**parameters)
    return terminal.PathTimeDecoderTerminalV1()(*args, **kwargs)


def assert_physical(result, speed, a0, endpoint):
    for value in result.values():
        assert bool(torch.isfinite(value).all())
    assert result["xy"].shape == (1, 1, 64, 2)
    assert result["instantaneous_speed"].shape == (1, 65)
    assert result["path_heading"].shape == (1, 65)
    assert result["instantaneous_speed"][0, 0].item() == pytest.approx(speed, abs=1.e-7)
    assert result["acceleration"][0, 0].item() == pytest.approx(a0, abs=1.e-6)
    assert result["instantaneous_speed"].min().item() >= 0.
    assert result["instantaneous_speed"].max().item() <= 2.5 + 1.e-6
    assert result["acceleration"].min().item() >= -2. - 1.e-6
    assert result["acceleration"].max().item() <= 1.5 + 1.e-6
    assert result["arc_distance"].max().item() <= endpoint + 1.e-6
    assert bool((result["arc_distance"][0, 1:] >= result["arc_distance"][0, :-1] - 1.e-6).all())
    grid = result["spatial_arc_m"][0]
    assert bool((grid[1:] > grid[:-1]).all())
    assert bool((grid[1:] - grid[:-1] <= .25 + 1.e-6).all())
    speed_cells = result["spatial_speed"][0]
    if len(grid) > 1:
        assert bool((speed_cells[:-1] + speed_cells[1:] > 0).all())
        k = result["spatial_curvature"][0]
        bound = torch.maximum(speed_cells[:-1].square(), speed_cells[1:].square()) * torch.maximum(k[:-1].abs(), k[1:].abs())
        assert bound.max().item() <= 1.5 + 1.e-6
    xy = torch.cat((torch.zeros_like(result["xy"][:, :, :1]), result["xy"]), dim=2)
    chord = torch.norm(xy[:, :, 1:] - xy[:, :, :-1], dim=-1) / .1
    assert torch.equal(result["speed"], chord)
    if bool(result["terminal_stop_complete_within_horizon"][0]):
        assert result["arc_distance"][0, -1].item() == pytest.approx(endpoint, abs=1.e-6)
        assert result["instantaneous_speed"][0, -1].item() == 0.


@pytest.mark.parametrize("dtype", [torch.float32, torch.float64])
@pytest.mark.parametrize("mode", [GO, BRAKE_TO_STOP, HOLD])
def test_ordinary_none_exact_outputs_and_ignores_new_arguments(dtype, mode):
    speed = .01 if mode == HOLD else 1.
    args, kwargs = inputs(speed=speed, dtype=dtype, intent=mode, support=64.)
    extras = dict(initial_acceleration_prediction=torch.tensor([float("nan")]), terminal_now_ns=True,
                  terminal_anchor_timestamp_ns=-1, terminal_observation_sha256="not-a-source")
    old = PathTimeDecoderV2()(*args)
    new = terminal.PathTimeDecoderTerminalV1()(*args, **extras)
    assert old.keys() == new.keys()
    assert all(torch.equal(old[key], new[key]) for key in old)
    assert "terminal_arrival_time_s" not in new


def test_verified_ordinary_exact_delegate_and_original_rejection():
    args, _ = inputs(support=10.)
    ordinary = witness(10., proof.ORDINARY)
    for decoder, kwargs in [(PathTimeDecoderV2(), {}), (terminal.PathTimeDecoderTerminalV1(), ordinary)]:
        with pytest.raises(ContractError, match="GO has insufficient supported horizon plus braking preview"):
            decoder(*args, **kwargs)
    args[5] = torch.tensor([64.], dtype=torch.float64)
    new = terminal.PathTimeDecoderTerminalV1()(*args, **ordinary)
    old = PathTimeDecoderV2()(*args)
    assert all(torch.equal(old[key], new[key]) for key in old)


@pytest.mark.parametrize("dtype", [torch.float32, torch.float64])
@pytest.mark.parametrize("mode", [GO, BRAKE_TO_STOP])
@pytest.mark.parametrize("curvature", [-.1, 0., .1])
def test_short_true_terminal_variable_requests_and_exact_timed_prefix(dtype, mode, curvature):
    result = run(speed=1., preferred=1.4, curvature=curvature, intent=mode, stop=2.137, support=3., a0=-.25, dtype=dtype)
    assert_physical(result, 1., -.25, 2.137)
    assert result["intent"].item() == mode
    assert result["terminal_numeric_regime"].item() == 2
    arc = result["arc_distance"][0, 1:]
    expected = torch.stack((arc, torch.zeros_like(arc)), dim=-1) if curvature == 0 else torch.stack(
        (torch.sin(curvature * arc) / curvature, (1 - torch.cos(curvature * arc)) / curvature), dim=-1)
    assert torch.allclose(result["xy"][0, 0], expected, atol=1.e-6, rtol=1.e-6)
    assert result["arc_distance"][0, 1].item() == pytest.approx(.09875, abs=1.e-7)
    assert result["instantaneous_speed"][0, 1].item() == pytest.approx(.975, abs=1.e-7)
    assert result["path_heading"][0, 1].item() == pytest.approx(curvature * .09875, abs=1.e-7)
    exact_boundary = .5 * (result["instantaneous_speed"][0, 0].to(torch.float64)
                           + result["instantaneous_speed"][0, 1].to(torch.float64)) * .1
    assert bool((result["spatial_arc_m"][0] == exact_boundary).any())


def test_brake_speed_head_requests_genuinely_change_timing_not_constant_deceleration():
    slow = run(intent=BRAKE_TO_STOP, preferred=.5)
    fast = run(intent=BRAKE_TO_STOP, preferred=2.)
    assert not torch.equal(slow["instantaneous_speed"], fast["instantaneous_speed"])
    assert slow["terminal_arrival_time_s"].item() > fast["terminal_arrival_time_s"].item()
    assert slow["acceleration"][0, 0].item() == pytest.approx(-.2)
    assert fast["acceleration"][0, 0].item() == pytest.approx(-.2)
    assert slow["spatial_speed"][0, -1].item() == fast["spatial_speed"][0, -1].item() == 0.


def test_arrival_after64_is_reported_not_claimed_completed():
    result = run(intent=BRAKE_TO_STOP, preferred=.2, stop=8.)
    assert result["terminal_arrival_time_s"].item() > 6.4
    assert result["terminal_stop_complete_within_horizon"].item() is False
    assert result["arc_distance"][0, -1].item() < 8.
    assert result["instantaneous_speed"][0, -1].item() > 0.


@pytest.mark.parametrize("dtype", [torch.float32, torch.float64])
@pytest.mark.parametrize("speed", [1.e-6, .01, .05])
def test_hold_actual_residual_stops_before100ms_without_zeroing_v0(dtype, speed):
    v0 = torch.tensor([speed], dtype=dtype)
    args, kwargs = inputs(speed=speed, intent=HOLD, stop=speed * speed / 3., support=1., a0=0., dtype=dtype)
    kwargs["initial_acceleration_prediction"] = -v0 / .1
    result = terminal.PathTimeDecoderTerminalV1()(*args, **kwargs)
    assert_physical(result, v0.item(), kwargs["initial_acceleration_prediction"].item(), args[4].item())
    assert torch.equal(result["instantaneous_speed"][:, :1], v0[:, None])
    assert result["terminal_numeric_regime"].item() == 1
    assert result["arc_distance"][0, 1].item() > 0.
    assert result["instantaneous_speed"][0, 1:].sum().item() == 0.
    assert torch.equal(result["xy"][0, 0, 0], result["xy"][0, 0, -1])
    assert torch.equal(result["path_heading"][0, 1:2], result["path_heading"][0, -1:])


def test_true_zero_endpoint_hold_onepoint_has_no_fake_cells():
    result = run(speed=0., preferred=0., intent=HOLD, stop=0., support=0., a0=0.)
    assert_physical(result, 0., 0., 0.)
    assert result["spatial_arc_m"].shape == (1, 1)
    assert result["xy"].abs().sum().item() == 0.
    assert result["instantaneous_speed"].abs().sum().item() == 0.
    assert result["terminal_arrival_time_s"].item() == 0.


def test_hold_nonincreasing_without_erasing_positive_residual():
    result = run(speed=.05, preferred=2., intent=HOLD, stop=.3, support=1., a0=-.1)
    assert_physical(result, .05, -.1, .3)
    assert bool((result["instantaneous_speed"][0, 1:] <= result["instantaneous_speed"][0, :-1] + 1.e-12).all())
    assert result["arc_distance"][0, 1].item() > 0.


def test_go_true_zero_can_wait_first100ms_then_accelerate_without_relabel():
    result = run(speed=0., preferred=1., intent=GO, stop=2., support=3., a0=0.)
    assert_physical(result, 0., 0., 2.)
    assert result["instantaneous_speed"][0, 0].item() == result["instantaneous_speed"][0, 1].item() == 0.
    assert result["arc_distance"][0, 1].item() == 0.
    assert result["instantaneous_speed"][0, 2].item() > 0.
    assert result["intent"].item() == GO


@pytest.mark.parametrize("dtype", [torch.float32, torch.float64])
@pytest.mark.parametrize("endpoint", [.001, .01, .1, .249])
def test_short_go_true_zero_has_real_interior_departure_stop_not_epsilon_creep(dtype, endpoint):
    result = run(speed=0., preferred=1., intent=GO, stop=endpoint, support=1., a0=0., dtype=dtype)
    assert_physical(result, 0., 0., endpoint)
    assert result["instantaneous_speed"][0, 0].item() == result["instantaneous_speed"][0, 1].item() == 0.
    assert result["spatial_speed"][0, 1:-1].max().item() > 0.
    assert result["arc_distance"][0, -1].item() == pytest.approx(endpoint, abs=1.e-7)
    assert result["terminal_spatial_node_count"].item() == result["spatial_arc_m"].shape[1]
    assert result["terminal_max_spatial_cell_m"].item() <= .25


def test_exact_minimum_braking_and_first100ms_boundary_is_not_false_rejected():
    result = run(speed=1., preferred=.3, intent=BRAKE_TO_STOP, stop=.25, support=1., a0=-2.)
    assert_physical(result, 1., -2., .25)
    assert result["arc_distance"][0, 1].item() == pytest.approx(.09)
    assert result["terminal_arrival_time_s"].item() == pytest.approx(.5)
    with pytest.raises(terminal.TerminalDecoderError) as error:
        run(speed=1., intent=BRAKE_TO_STOP, stop=.24999, support=1., a0=-2.)
    assert error.value.code == "ENDPOINT_BRAKING_INFEASIBLE"


def test_interior_zero_first_stop_is_not_silently_retimed_as_farther_endpoint():
    args, kwargs = inputs(stop=4., support=5.)
    args[1][0, 1] = 0.
    with pytest.raises(terminal.TerminalDecoderError) as error:
        terminal.PathTimeDecoderTerminalV1()(*args, **kwargs)
    assert error.value.code == "STOP_BEFORE_REQUESTED_ENDPOINT"


@pytest.mark.parametrize("dtype", [torch.float32, torch.float64])
def test_short_adaptive_grid_endpoint_and_raw_field_gradients_finite(dtype):
    args, kwargs = inputs(speed=0., preferred=.04, curvature=.1, stop=.02, support=1., a0=0., dtype=dtype)
    for index in (0, 1, 4): args[index].requires_grad_(True)
    kwargs["initial_acceleration_prediction"].requires_grad_(True)
    result = terminal.PathTimeDecoderTerminalV1()(*args, **kwargs)
    (result["xy"].sum() + result["instantaneous_speed"].sum() + result["terminal_arrival_time_s"].sum()).backward()
    for value in (args[0], args[1], args[4], kwargs["initial_acceleration_prediction"]):
        assert value.grad is not None and bool(torch.isfinite(value.grad).all())
    assert args[1].grad.abs().sum().item() > 0.
    assert args[4].grad.abs().sum().item() > 0.


def test_float32_subnormal_endpoint_retains_real_grid_and_exact_first100ms_prefix():
    endpoint = 1.401298464324817e-45
    args, kwargs = inputs(speed=0., preferred=1., stop=endpoint, support=1., a0=0., dtype=torch.float32)
    result = terminal.PathTimeDecoderTerminalV1()(*args, **kwargs)
    assert result["spatial_arc_m"].dtype == torch.float64
    assert result["spatial_speed"].dtype == torch.float64
    assert result["spatial_cell_duration_s"].dtype == torch.float64
    assert bool((result["spatial_arc_m"][0, 1:] > result["spatial_arc_m"][0, :-1]).all())
    assert bool((result["spatial_cell_duration_s"] > 0).all())
    assert bool(torch.isfinite(result["spatial_cell_duration_s"]).all())
    assert result["spatial_arc_m"][0, -1].item() == args[4].item()
    assert result["arc_distance"][0, 1].item() == 0.
    assert result["arc_distance"][0, 2].item() == args[4].item() > 0.
    assert torch.equal(result["instantaneous_speed"][:, :1], args[2][:, None])
    assert result["xy"].dtype == result["instantaneous_speed"].dtype == torch.float32


@pytest.mark.parametrize("parameters,code", [
    ({"speed": 2.50001}, "OBSERVED_SPEED_LIMIT"), ({"speed": -.01}, "OBSERVED_SPEED_LIMIT"),
    ({"stop": 10.1}, "ENDPOINT_OUTSIDE_SUPPORT"), ({"stop": .1}, "ENDPOINT_BRAKING_INFEASIBLE"),
    ({"a0": 1.50001}, "INITIAL_ACCELERATION_LIMIT"), ({"a0": -2.00001}, "INITIAL_ACCELERATION_LIMIT"),
    ({"speed": 2.45, "a0": 1.}, "INITIAL_VELOCITY_LIMIT"),
    ({"intent": HOLD, "speed": .05001}, "HOLD_ENTRY_SPEED"),
    ({"intent": HOLD, "speed": .01, "a0": .01}, "HOLD_ACCELERATION_REQUEST"),
    ({"intent": HOLD, "speed": 0., "a0": 0., "stop": 1.}, "STATIONARY_HOLD_REQUEST"),
    ({"intent": BRAKE_TO_STOP, "speed": 0., "a0": 0.}, "BRAKE_MOVING_STATE"),
    ({"intent": GO, "speed": 0., "a0": 0., "stop": 0.}, "ZERO_ENDPOINT_REQUIRES_HOLD"),
    ({"speed": .05, "a0": -.5, "stop": .1}, "STOP_BEFORE_REQUESTED_ENDPOINT"),
    ({"speed": 1., "a0": 1.5, "stop": .3}, "INITIAL_BOUNDARY_INFEASIBLE"),
    ({"preferred": 0., "speed": 0., "a0": 0.}, "UNREACHABLE_SPEED_CELL"),
    ({"preferred": -.01}, "NEGATIVE_SPEED_REQUEST"), ({"curvature": .50001}, "CURVATURE_LIMIT"),
])
def test_explicit_semantic_and_physical_rejection(parameters, code):
    with pytest.raises(terminal.TerminalDecoderError) as error:
        run(**parameters)
    assert error.value.code == code


@pytest.mark.parametrize("mutation,code", [
    ("rejected", "WITNESS_REJECTED"), ("permission", "TERMINAL_PERMISSION_MISSING"),
    ("source", "WITNESS_SOURCE_MISMATCH"), ("mission", "WITNESS_SOURCE_MISMATCH"),
    ("artifact", "WITNESS_SOURCE_MISMATCH"), ("anchor", "WITNESS_CLOCK_MISMATCH"),
    ("now_bool", "WITNESS_CLOCK_MISMATCH"), ("stale", "WITNESS_CLOCK_MISMATCH"),
    ("future", "WITNESS_CLOCK_MISMATCH"), ("support", "CAUSAL_SUPPORT_MISMATCH"),
])
def test_witness_pins_clock_and_support_fail_closed(mutation, code):
    args, kwargs = inputs()
    if mutation == "rejected": kwargs["terminal_witness"] = proof.TerminalWitnessDecision("REJECTED", "no", "no", None)
    elif mutation == "permission":
        decision = kwargs["terminal_witness"]
        kwargs["terminal_witness"] = replace(decision, witness=replace(decision.witness, terminal_permission=False))
    elif mutation == "source": kwargs["terminal_observation_sha256"] = "d" * 64
    elif mutation == "mission": kwargs["terminal_mission_sha256"] = "d" * 64
    elif mutation == "artifact": kwargs["terminal_route_artifact_sha256"] = "d" * 64
    elif mutation == "anchor": kwargs["terminal_anchor_timestamp_ns"] += 1
    elif mutation == "now_bool": kwargs["terminal_now_ns"] = True
    elif mutation == "stale": kwargs["terminal_now_ns"] += proof.MAXIMUM_INPUT_AGE_NS + 1
    elif mutation == "future": kwargs["terminal_now_ns"] -= 1
    elif mutation == "support": args[5] -= .1
    with pytest.raises(terminal.TerminalDecoderError) as error:
        terminal.PathTimeDecoderTerminalV1()(*args, **kwargs)
    assert error.value.code == code


@pytest.mark.parametrize("index", [0, 1, 2, 4, 5])
@pytest.mark.parametrize("nonfinite", [float("nan"), float("inf"), -float("inf")])
def test_all_raw_and_physical_tensors_checked_even_unused_tail(index, nonfinite):
    args, kwargs = inputs()
    args[index].reshape(-1)[-1] = nonfinite
    with pytest.raises(terminal.TerminalDecoderError, match="all raw knots"):
        terminal.PathTimeDecoderTerminalV1()(*args, **kwargs)


def test_untrained_raw_a0_head_requires_explicit_valid_physical_request():
    args, kwargs = inputs()
    kwargs["initial_acceleration_prediction"] = None
    with pytest.raises(terminal.TerminalDecoderError) as error:
        terminal.PathTimeDecoderTerminalV1()(*args, **kwargs)
    assert error.value.code == "TERMINAL_TENSOR_ABI"


@pytest.mark.parametrize("dtype", [torch.float32, torch.float64])
@pytest.mark.parametrize("mode", [GO, BRAKE_TO_STOP])
def test_gradients_actual_variable_endpoint_raw_speed_curvature_and_a0(dtype, mode):
    args, kwargs = inputs(speed=.8, preferred=.9, curvature=.08, intent=mode, stop=2.137, support=3., a0=-.3, dtype=dtype)
    for index in (0, 1, 2, 4): args[index].requires_grad_(True)
    kwargs["initial_acceleration_prediction"].requires_grad_(True)
    result = terminal.PathTimeDecoderTerminalV1()(*args, **kwargs)
    loss = result["xy"].square().sum() + result["instantaneous_speed"].sum() + result["terminal_arrival_time_s"].sum()
    loss.backward()
    for value in [args[0], args[1], args[2], args[4], kwargs["initial_acceleration_prediction"]]:
        assert value.grad is not None and bool(torch.isfinite(value.grad).all())
    assert args[0].grad.abs().sum().item() > 0.
    assert args[1].grad.abs().sum().item() > 0.
    assert args[4].grad.abs().sum().item() > 0.
    assert kwargs["initial_acceleration_prediction"].grad.abs().sum().item() > 0.


@pytest.mark.parametrize("speed", [0., 1.e-6, .01])
def test_zero_and_small_hold_gradients_are_finite(speed):
    args, kwargs = inputs(speed=speed, preferred=0., intent=HOLD, stop=speed * speed / 3., support=1., a0=-speed / .1)
    for index in (0, 1, 2, 4): args[index].requires_grad_(True)
    kwargs["initial_acceleration_prediction"].requires_grad_(True)
    result = terminal.PathTimeDecoderTerminalV1()(*args, **kwargs)
    loss = result["xy"].sum() + result["instantaneous_speed"].sum() + result["terminal_arrival_time_s"].sum()
    loss.backward()
    for value in [args[0], args[2], args[4]]:
        # HH_261002 - True-zero HOLD has no moving-path curvature dependency; an unused gradient is not a fake zero label.
        if speed == 0. and value is args[0]:
            assert value.grad is None or bool(torch.isfinite(value.grad).all())
        else:
            assert value.grad is not None and bool(torch.isfinite(value.grad).all())


def test_raw_field_coordinates_not_stretched_to_short_endpoint_or_nav_replaced():
    args, kwargs = inputs(stop=2.137, support=3.)
    args[0][0] = torch.arange(33, dtype=torch.float64) * .002
    result = terminal.PathTimeDecoderTerminalV1()(*args, **kwargs)
    assert result["spatial_curvature"][0, -1].item() == pytest.approx(.002137)
    assert result["xy"][0, 0, -1, 1].item() > 0.
    assert result["raw_preferred_speed_knots"] is args[1]
    assert result["intent"] is args[3]


def test_after_arrival_xy_heading_and_speed_remain_exactly_stationary():
    result = run(curvature=.2, stop=.8, a0=-.5)
    elapsed = torch.arange(65, dtype=torch.float64) * .1
    stopped = elapsed >= result["terminal_arrival_time_s"].item()
    assert stopped.sum().item() > 2
    assert torch.equal(result["arc_distance"][0, stopped], torch.full_like(result["arc_distance"][0, stopped], .8))
    stopped_heading = result["path_heading"][0, stopped]
    assert stopped_heading[0].item() == pytest.approx(.16)
    assert torch.equal(stopped_heading, stopped_heading[0].expand_as(stopped_heading))
    assert result["instantaneous_speed"][0, stopped].abs().sum().item() == 0.


def test_signature_and_contract_accept_no_future_or_teacher_inputs():
    parameters = inspect.signature(terminal.PathTimeDecoderTerminalV1.forward).parameters
    assert not any("future" in name or "teacher" in name or "label" in name for name in parameters)
    contract = terminal.prepared_contract()
    assert contract["prepared_not_deployed"] and contract["ordinary_outputs_errors_exact_delegate"]
    assert not contract["model_or_weights_changed"] and not contract["vehicle_control_approved"]
    assert contract["main_functions_approved"] == contract["subfunctions_approved"] == 0
    assert contract["new_terminal_HOLD_a0_nonpositive_and_residual_speed_nonincreasing"]
