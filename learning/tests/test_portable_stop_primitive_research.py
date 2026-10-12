"""HH_260906 - Test synthetic stop primitives without training data, GPU work or deploying a model."""

import inspect

import pytest
import torch

from portable_e2e import stop_primitive_research as primitive
from portable_e2e.model import ModelConfig
from portable_e2e.contract import ContractError
from portable_e2e.runtime_contract import validate_and_select_trajectory


def inputs(speed=1.0, duration=0.0, curvature=0.0, dtype=torch.float64):
    """HH_260906 - Build explicitly synthetic inputs, never substitute these for captured driving evidence."""
    return (torch.tensor([speed], dtype=dtype), torch.full((1, 6), duration, dtype=dtype),
            torch.full((1, 6, 64), curvature, dtype=dtype))


@pytest.mark.parametrize("dtype", [torch.float32, torch.float64])
@pytest.mark.parametrize("speed", [0.0, 0.01, 0.1, 0.4, 1.0, 30.0 / 3.6])
@pytest.mark.parametrize("duration", [-20.0, 0.0, 20.0])
def test_complete_monotonic_stop_with_existing_unmodified_gate(dtype, speed, duration):
    result = primitive.decode_stop_primitive(*inputs(speed, duration, dtype=dtype))
    assert result.xy_base_m.shape == (1, 6, 64, 2)
    assert torch.equal(result.speed_mps[:, :, -1], torch.zeros(1, 6, dtype=dtype))
    entry = torch.cat((torch.full((1, 6, 1), speed, dtype=dtype), result.speed_mps[:, :, :-1]), dim=2)
    assert bool(torch.all(result.speed_mps <= entry))
    eps = torch.finfo(dtype).eps
    assert bool(torch.all((entry - result.speed_mps) / .1 <= primitive.MAX_DECELERATION_MPS2 + 256 * eps))
    assert torch.equal(result.xy_base_m[..., 1], torch.zeros(1, 6, 64, dtype=dtype))
    assert torch.allclose(result.xy_base_m[..., 0], torch.cumsum(result.speed_mps * .1, dim=2))
    selected = validate_and_select_trajectory(result.xy_base_m[0].tolist(), result.speed_mps[0].tolist(),
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], current_speed_mps=float(inputs(speed, dtype=dtype)[0].item()))
    assert selected.candidate_index == 0


def test_zero_motion_is_exact_and_does_not_turn_in_place():
    result = primitive.decode_stop_primitive(*inputs(0.0, -1.0, 20.0))
    assert result.xy_base_m.count_nonzero() == result.speed_mps.count_nonzero() == result.heading_rad.count_nonzero() == 0


def test_continuous_distance_is_not_misrepresented_as_discrete_geometry():
    result = primitive.decode_stop_primitive(*inputs(1.0))
    duration = float(result.stop_duration_s[0, 0])
    expected = sum(max(0.0, 1.0 - (i + 1) * .1 / duration) * .1 for i in range(64))
    assert float(result.xy_base_m[0, 0, -1, 0]) == pytest.approx(expected, abs=1e-12)
    assert expected < .5 * duration
    assert .5 * duration - expected <= .1


def test_left_and_right_are_symmetric_without_caller_mutation():
    raw = inputs(4.0, .2, .4)
    before = tuple(t.clone() for t in raw)
    left = primitive.decode_stop_primitive(*raw)
    right = primitive.decode_stop_primitive(raw[0], raw[1], -raw[2])
    assert all(torch.equal(a, b) for a, b in zip(raw, before))
    assert torch.equal(left.speed_mps, right.speed_mps)
    assert torch.equal(left.xy_base_m[..., 0], right.xy_base_m[..., 0])
    assert torch.equal(left.xy_base_m[..., 1], -right.xy_base_m[..., 1])
    assert bool(torch.all(left.heading_rad >= 0.0))


def test_heading_increments_obey_entry_speed_curvature_and_lateral_bounds():
    speed, duration, curvature = inputs(8.0, 0.0, 20.0)
    result = primitive.decode_stop_primitive(speed, duration, curvature)
    entry = torch.cat((speed[:, None, None].expand(-1, 6, 1), result.speed_mps[:, :, :-1]), dim=2)
    previous_heading = torch.cat((torch.zeros(1, 6, 1, dtype=speed.dtype), result.heading_rad[:, :, :-1]), dim=2)
    ds = result.speed_mps * .1
    assessed = ds > 0
    measured_curvature = (result.heading_rad - previous_heading)[assessed] / ds[assessed]
    assert measured_curvature.max() <= .2 + 1e-12
    assert (measured_curvature * entry[assessed].square()).max() <= 2.8 + 1e-12


def test_gradients_are_finite_and_match_local_finite_difference():
    raw = tuple(t.requires_grad_() for t in inputs(1.0, .3, .2))
    output = primitive.decode_stop_primitive(*raw)
    loss = output.xy_base_m.square().sum() + output.speed_mps.sum()
    loss.backward()
    assert all(t.grad is not None and bool(torch.isfinite(t.grad).all()) for t in raw)
    assert all(bool(t.grad.abs().sum() > 0) for t in raw)
    epsilon = 1e-6
    values = []
    for sign in [-1.0, 1.0]:
        shifted = [t.detach().clone() for t in raw]
        shifted[1][0, 0] += sign * epsilon
        result = primitive.decode_stop_primitive(*shifted)
        values.append(float(result.xy_base_m.square().sum() + result.speed_mps.sum()))
    assert float(raw[1].grad[0, 0]) == pytest.approx((values[1] - values[0]) / (2 * epsilon), rel=1e-6)


@pytest.mark.parametrize("field", [0, 1, 2])
@pytest.mark.parametrize("value", [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_inputs_are_rejected(field, value):
    raw = list(inputs())
    raw[field].reshape(-1)[0] = value
    with pytest.raises(ValueError, match="finite"):
        primitive.decode_stop_primitive(*raw)


@pytest.mark.parametrize("value", [-.001, -.1, 30.0 / 3.6 + .001])
def test_invalid_current_speed_is_not_silently_clamped(value):
    with pytest.raises(ValueError, match="between zero"):
        primitive.decode_stop_primitive(*inputs(value))


@pytest.mark.parametrize("change", ["float16", "integer", "mixed_dtype", "list", "empty", "speed_shape", "duration_shape", "curvature_shape"])
def test_input_contract_rejects_bad_shapes_and_types(change):
    raw = list(inputs())
    if change == "float16": raw = [t.half() for t in raw]
    elif change == "integer": raw = [t.long() for t in raw]
    elif change == "mixed_dtype": raw[2] = raw[2].float()
    elif change == "list": raw[0] = [1.0]
    elif change == "empty": raw = [t[:0] for t in raw]
    elif change == "speed_shape": raw[0] = raw[0].reshape(1, 1)
    elif change == "duration_shape": raw[1] = raw[1][:, :5]
    else: raw[2] = raw[2][:, :, :63]
    with pytest.raises(ValueError): primitive.decode_stop_primitive(*raw)


def test_research_id_is_not_a_trainable_or_deployable_model_id():
    with pytest.raises(ContractError): ModelConfig(model_id=primitive.RESEARCH_ID).validate()
    assert not any(name in inspect.signature(primitive.decode_stop_primitive).parameters
        for name in ("target_xy", "target_speed", "future", "checkpoint", "stop_label"))
