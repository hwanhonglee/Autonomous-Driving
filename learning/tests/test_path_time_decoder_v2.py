"""HH_261001 - Check low-speed continuity, physical rejection and differentiable shape/time research."""

from dataclasses import replace
import math

import pytest
import torch
from torch.nn import functional as F

from portable_e2e.contract import ContractError
from portable_e2e.path_time_decoder_v2 import BRAKE_TO_STOP, GO, HOLD, DecoderConfigV2, PathTimeDecoderV2
from portable_e2e.runtime_contract import RuntimeGateConfig, validate_and_select_trajectory


def inputs(speed=1.0, preferred=1.0, curvature=0.0, intent=GO, stop=3.0, support=64.0, dtype=torch.float64):
    return [torch.full((1, 33), curvature, dtype=dtype),
            torch.full((1, 33), preferred, dtype=dtype),
            torch.tensor([speed], dtype=dtype), torch.tensor([intent]),
            torch.tensor([stop], dtype=dtype), torch.tensor([support], dtype=dtype)]


def run(**kwargs):
    return PathTimeDecoderV2()(*inputs(**kwargs))


def assert_physical(result):
    assert bool(torch.isfinite(result["xy"]).all())
    assert bool(torch.isfinite(result["instantaneous_speed"]).all())
    assert result["acceleration"].max().item() <= 1.5 + 1.0e-5
    assert result["acceleration"].min().item() >= -2.0 - 1.0e-5
    assert result["instantaneous_speed"].max().item() <= 2.5 + 1.0e-6
    assert bool((torch.diff(result["arc_distance"]) >= -1.0e-7).all())
    zero = torch.zeros_like(result["xy"][:, :, :1])
    reconstructed = torch.norm(torch.diff(torch.cat((zero, result["xy"]), dim=2), dim=2), dim=-1) / 0.1
    assert torch.allclose(result["speed"], reconstructed, atol=1.0e-9)


@pytest.mark.parametrize("dtype", [torch.float32, torch.float64])
def test_shapes_and_constant_straight_speed(dtype):
    result = run(dtype=dtype)
    expected = {"xy": (1, 1, 64, 2), "speed": (1, 1, 64), "instantaneous_speed": (1, 65),
                "acceleration": (1, 64), "arc_distance": (1, 65), "spatial_xy": (1, 257, 2),
                "spatial_heading": (1, 257), "spatial_speed": (1, 257), "spatial_valid": (1, 257)}
    for key, shape in expected.items():
        assert result[key].shape == shape
    assert torch.allclose(result["xy"][0, 0, :, 0], torch.arange(1, 65, dtype=dtype) * 0.1, atol=1.0e-6)
    assert torch.allclose(result["instantaneous_speed"], torch.ones(1, 65, dtype=dtype))
    assert torch.equal(result["xy"][..., 1], torch.zeros(1, 1, 64, dtype=dtype))
    assert result["active_distance_m"].item() == 18.0
    assert_physical(result)


@pytest.mark.parametrize("curvature", [-0.1, 0.0, 0.1])
def test_constant_curve_analytic_position_and_gate(curvature):
    result = run(curvature=curvature)
    distance = result["arc_distance"][0, 1:]
    if curvature:
        expected = torch.stack((torch.sin(curvature * distance) / curvature,
                                (1 - torch.cos(curvature * distance)) / curvature), dim=-1)
    else:
        expected = torch.stack((distance, torch.zeros_like(distance)), dim=-1)
    assert torch.allclose(result["xy"][0, 0], expected, atol=1.0e-11)
    validate_and_select_trajectory(result["xy"][0].tolist(), result["speed"][0].tolist(), [0.0], RuntimeGateConfig(candidate_count=1), current_speed_mps=1.0)
    assert_physical(result)


def test_smooth_acceleration_from_observed_standstill():
    result = run(speed=0.0, preferred=2.0)
    assert result["instantaneous_speed"][0, 0].item() == 0.0
    assert result["instantaneous_speed"][0, 1].item() == pytest.approx(0.15)
    assert result["xy"][0, 0, 0, 0].item() == pytest.approx(0.0075)
    assert result["instantaneous_speed"][0, -1].item() == pytest.approx(2.0)
    assert_physical(result)


def test_preferred_speed_below_observed_is_continuous_braking_not_start_replacement():
    result = run(speed=2.0, preferred=0.5)
    assert result["instantaneous_speed"][0, 0].item() == 2.0
    assert result["instantaneous_speed"][0, 1].item() == pytest.approx(1.8)
    assert result["instantaneous_speed"][0, -1].item() == pytest.approx(0.5)
    assert_physical(result)


def test_distant_curve_and_speed_request_do_not_brake_low_speed_prefix():
    args = inputs(speed=0.4, preferred=0.4)
    original = PathTimeDecoderV2()(*args)
    args[0][0, 12:] = 0.5
    args[1][0, 12:] = 0.05
    distant = PathTimeDecoderV2()(*args)
    assert torch.equal(original["xy"], distant["xy"])
    assert torch.equal(original["instantaneous_speed"], distant["instantaneous_speed"])


def test_curvature_cap_and_preview_braking_before_turn():
    args = inputs(speed=2.4, preferred=2.5)
    args[0][0, 2:10] = 0.4
    result = PathTimeDecoderV2()(*args)
    assert result["instantaneous_speed"][0, 0].item() == 2.4
    assert result["spatial_speed"][0, 16].item() <= math.sqrt(1.5 / 0.4) + 1.0e-8
    assert_physical(result)


@pytest.mark.parametrize("stop", [0.5, 1.234, 4.0, 25.0])
def test_explicit_stop_distance_and_continuous_v0(stop):
    result = run(speed=1.0, preferred=2.0, intent=BRAKE_TO_STOP, stop=stop)
    deceleration = 1.0 / (2 * stop)
    times = torch.arange(65, dtype=torch.float64) * 0.1
    elapsed = torch.minimum(times, torch.tensor(2 * stop))
    expected_distance = elapsed - 0.5 * deceleration * elapsed.square()
    assert torch.allclose(result["arc_distance"][0], expected_distance, atol=1.0e-12)
    assert result["instantaneous_speed"][0, 0].item() == 1.0
    if 2 * stop <= 6.4:
        assert result["arc_distance"][0, -1].item() == pytest.approx(stop)
        assert result["instantaneous_speed"][0, -1].item() == 0.0
    assert_physical(result)


@pytest.mark.parametrize("speed", [0.0, 1.0e-5, 0.01, 0.05])
def test_hold_preserves_residual_t0_and_accounts_for_bounded_stop(speed):
    result = run(speed=speed, intent=HOLD, stop=0.0, support=0.25)
    assert result["instantaneous_speed"][0, 0].item() == speed
    assert result["instantaneous_speed"][0, 1:].abs().max().item() == 0.0
    assert result["arc_distance"][0, -1].item() == pytest.approx(speed**2 / 4, abs=1.0e-15)
    assert_physical(result)


@pytest.mark.parametrize("overrides,match", [
    ({"speed": 2.50001}, "will not be clipped"),
    ({"speed": -0.01}, "will not be clipped"),
    ({"speed": 0.051, "intent": HOLD}, "HOLD cannot replace"),
    ({"speed": 2.0, "intent": BRAKE_TO_STOP, "stop": 0.99}, "physically infeasible"),
    ({"speed": 0.0, "intent": BRAKE_TO_STOP}, "moving observed state"),
    ({"speed": 2.4, "curvature": 0.5}, "curvature envelope"),
    ({"support": 17.99}, "insufficient supported"),
    ({"intent": BRAKE_TO_STOP, "stop": 3.0, "support": 2.99}, "more supported distance"),
    ({"intent": HOLD, "speed": 0.05, "support": 0.0}, "more supported distance"),
    ({"preferred": 0.0}, "strictly positive"),
    ({"preferred": 1.0e-200}, "numerical conditioning"),
    ({"preferred": 0.000099}, "numerical conditioning"),
    ({"speed": 0.001, "intent": BRAKE_TO_STOP, "stop": 1.0e-8}, "physically infeasible"),
    ({"curvature": 0.50001}, "curvature exceeds"),
    ({"support": 64.001}, "supported distance"),
    ({"stop": -1.0}, "stop distance"),
    ({"intent": 3}, "intent must"),
])
def test_explicit_contract_rejections(overrides, match):
    with pytest.raises(ContractError, match=match):
        run(**overrides)


@pytest.mark.parametrize("index", [0, 1, 2, 4, 5])
def test_nonfinite_inputs_rejected(index):
    args = inputs()
    args[index].flatten()[0] = float("nan")
    with pytest.raises(ContractError, match="finite"):
        PathTimeDecoderV2()(*args)


@pytest.mark.parametrize("index", [0, 1, 2, 3, 4, 5])
def test_malformed_shapes_rejected(index):
    args = inputs()
    args[index] = args[index][..., None]
    with pytest.raises(ContractError):
        PathTimeDecoderV2()(*args)


@pytest.mark.parametrize("index", [0, 1, 2, 3, 4, 5])
def test_wrong_dtype_rejected(index):
    args = inputs()
    args[index] = args[index].to(torch.float32 if index else torch.float16)
    with pytest.raises(ContractError):
        PathTimeDecoderV2()(*args)


@pytest.mark.parametrize("field,value", [("maximum_speed_mps", 8.333), ("maximum_acceleration_mps2", 3.0),
                                        ("maximum_deceleration_mps2", 6.0), ("hold_entry_speed_mps", 0.1),
                                        ("minimum_preferred_speed_mps", 1.0e-7),
                                        ("knot_count", 32), ("timestep_s", 0.2), ("spatial_step_m", 0.5)])
def test_config_cannot_silently_expand_envelope(field, value):
    with pytest.raises(ContractError):
        PathTimeDecoderV2(replace(DecoderConfigV2(), **{field: value}))


@pytest.mark.parametrize("dtype", [torch.float32, torch.float64])
def test_smallest_allowed_positive_request_has_finite_gradients(dtype):
    args = inputs(speed=0.0, preferred=0.0001, curvature=0.0, dtype=dtype)
    args[0].requires_grad_()
    args[1].requires_grad_()
    result = PathTimeDecoderV2()(*args)
    gradients = torch.autograd.grad(result["xy"].sum(), (args[0], args[1]))
    assert all(bool(torch.isfinite(gradient).all()) for gradient in gradients)
    assert result["arc_distance"][0, -1].item() > 0


@pytest.mark.parametrize("speed,intent,stop", [(0.0, GO, 3.0), (0.05, HOLD, 0.0),
                                             (1.0, BRAKE_TO_STOP, 1.234), (1.0, BRAKE_TO_STOP, 0.5)])
def test_start_hold_and_brake_pass_unchanged_runtime_geometry_gate(speed, intent, stop):
    result = run(speed=speed, intent=intent, stop=stop)
    selected = validate_and_select_trajectory(result["xy"][0].tolist(), result["speed"][0].tolist(),
                                              [0.0], RuntimeGateConfig(candidate_count=1), current_speed_mps=speed)
    assert selected.candidate_index == 0


@pytest.mark.parametrize("intent,speed,stop", [(GO, 0.3, 3.0), (GO, 1.2, 3.0),
                                              (BRAKE_TO_STOP, 1.2, 2.3), (HOLD, 0.03, 0.0)])
def test_shape_gradient_finite_and_directionally_matches_numeric_derivative(intent, speed, stop):
    args = inputs(speed=speed, preferred=1.1, curvature=0.04, intent=intent, stop=stop)
    args[0].requires_grad_()
    decoder = PathTimeDecoderV2()
    result = decoder(*args)
    objective = result["xy"][..., 1].sum() + 0.1 * result["spatial_xy"][:, :20, 1].sum()
    gradient, = torch.autograd.grad(objective, args[0])
    assert bool(torch.isfinite(gradient).all())
    direction = torch.linspace(0.1, 0.9, 33, dtype=torch.float64)[None]
    analytic = (gradient * direction).sum().item()
    values = []
    for sign in (-1, 1):
        shifted = list(args)
        shifted[0] = args[0].detach() + sign * 1.0e-6 * direction
        decoded = decoder(*shifted)
        values.append((decoded["xy"][..., 1].sum() + 0.1 * decoded["spatial_xy"][:, :20, 1].sum()).item())
    numeric = (values[1] - values[0]) / 2.0e-6
    assert analytic == pytest.approx(numeric, rel=2.0e-5, abs=1.0e-7)
    assert abs(analytic) > 0


@pytest.mark.parametrize("current,preferred", [(0.0, 0.73), (0.3, 0.73), (1.2, 0.73), (1.7, 0.93)])
def test_go_preference_gradient_reaches_xy_and_matches_numeric_derivative(current, preferred):
    args = inputs(speed=current, preferred=preferred, curvature=0.02)
    args[1].requires_grad_()
    decoder = PathTimeDecoderV2()
    objective = decoder(*args)["xy"].square().sum()
    gradient, = torch.autograd.grad(objective, args[1])
    assert bool(torch.isfinite(gradient).all())
    direction = torch.linspace(0.1, 0.7, 33, dtype=torch.float64)[None]
    analytic = (gradient * direction).sum().item()
    values = []
    for sign in (-1, 1):
        changed = list(args)
        changed[1] = args[1].detach() + sign * 1.0e-6 * direction
        values.append(decoder(*changed)["xy"].square().sum().item())
    assert analytic == pytest.approx((values[1] - values[0]) / 2.0e-6, rel=2.0e-5, abs=1.0e-5)
    assert analytic > 0


def test_softplus_raw_loss_preserves_gradient_when_physical_cap_is_active():
    args = inputs(speed=1.8, curvature=0.4)
    raw = torch.full((1, 33), 4.0, dtype=torch.float64, requires_grad=True)
    args[1] = F.softplus(raw)
    result = PathTimeDecoderV2()(*args)
    assert result["raw_preferred_speed_knots"] is args[1]
    loss = result["xy"].square().mean() + (result["raw_preferred_speed_knots"] - 1.0).square().mean()
    loss.backward()
    assert bool(torch.isfinite(raw.grad).all())
    assert raw.grad.abs().min().item() > 0


def test_active_curvature_cap_gradient_matches_directional_numeric_derivative():
    args = inputs(speed=1.7, preferred=3.0, curvature=0.4)
    args[0].requires_grad_()
    decoder = PathTimeDecoderV2()
    objective = decoder(*args)["xy"].square().sum()
    gradient, = torch.autograd.grad(objective, args[0])
    direction = torch.full_like(args[0], 0.1)
    analytic = (gradient * direction).sum().item()
    values = []
    for sign in (-1, 1):
        changed = list(args)
        changed[0] = args[0].detach() + sign * 1.0e-6 * direction
        values.append(decoder(*changed)["xy"].square().sum().item())
    assert bool(torch.isfinite(gradient).all())
    assert analytic == pytest.approx((values[1] - values[0]) / 2.0e-6, rel=1.0e-5, abs=1.0e-5)


def test_every_active_cell_has_a_conservative_lateral_acceleration_bound():
    args = inputs(speed=1.7, preferred=4.0)
    args[0][0] = 0.1 + 0.35 * torch.linspace(0, 1, 33, dtype=torch.float64)
    result = PathTimeDecoderV2()(*args)
    count = int(result["active_distance_m"].item() / 0.25) + 1
    k = result["spatial_curvature"][0, :count]
    v = result["spatial_speed"][0, :count]
    bound = torch.maximum(k[:-1].abs(), k[1:].abs()) * torch.maximum(v[:-1].square(), v[1:].square())
    assert bound.max().item() <= 1.5 + 1.0e-8


def test_zero_curvature_and_start_speed_have_finite_gradients():
    args = inputs(speed=0.0, preferred=0.71)
    args[0].requires_grad_()
    args[1].requires_grad_()
    args[2].requires_grad_()
    result = PathTimeDecoderV2()(*args)
    gradients = torch.autograd.grad(result["xy"].sum(), (args[0], args[1], args[2]))
    assert all(bool(torch.isfinite(gradient).all()) for gradient in gradients)


def test_stop_distance_gradient_is_finite_and_matches_numeric_derivative():
    args = inputs(speed=1.2, curvature=0.03, intent=BRAKE_TO_STOP, stop=2.27)
    args[4].requires_grad_()
    decoder = PathTimeDecoderV2()
    objective = decoder(*args)["xy"].square().sum()
    gradient, = torch.autograd.grad(objective, args[4])
    values = []
    for sign in (-1, 1):
        changed = list(args)
        changed[4] = args[4].detach() + sign * 1.0e-6
        values.append(decoder(*changed)["xy"].square().sum().item())
    assert gradient.item() == pytest.approx((values[1] - values[0]) / 2.0e-6, rel=1.0e-5)


def test_mixed_intents_and_prefix_masks_are_independent():
    rows = [inputs(speed=1.0, support=18.0), inputs(speed=1.0, intent=BRAKE_TO_STOP, stop=1.1, support=1.1),
            inputs(speed=0.01, intent=HOLD, support=0.25)]
    batch = [torch.cat([row[index] for row in rows]) for index in range(6)]
    result = PathTimeDecoderV2()(*batch)
    assert result["spatial_valid"].sum(-1).tolist() == [73, 5, 2]
    for index, row in enumerate(rows):
        single = PathTimeDecoderV2()(*row)
        assert torch.allclose(result["xy"][index], single["xy"][0])
        assert torch.equal(result["instantaneous_speed"][index], single["instantaneous_speed"][0])
