"""HH_260906 - Independently exercise research STOP isolation and tensor edge cases on CPU."""

from dataclasses import fields
import inspect

import pytest
import torch

from portable_e2e import stop_primitive_research as primitive
from portable_e2e.contract import ContractError
from portable_e2e.model import ModelConfig, SUPPORTED_MODEL_IDS
from portable_e2e import runtime_weight_bundle as bundle


def varied_inputs(dtype=torch.float64):
    """HH_260906 - Use fixed synthetic values without reading captured trajectories or future labels."""
    speed = torch.tensor([0.0, 0.07, 0.4, 4.0, 30.0 / 3.6], dtype=dtype)
    duration = torch.linspace(-5.0, 5.0, 30, dtype=dtype).reshape(5, 6)
    curvature = torch.sin(torch.arange(5 * 6 * 64, dtype=dtype)).reshape(5, 6, 64)
    return speed, duration, curvature


def assert_results_equal(left, right):
    """HH_260906 - Require exact tensor equality for transformations that cannot mix candidate arithmetic."""
    for field in fields(primitive.StopPrimitiveResult):
        assert torch.equal(getattr(left, field.name), getattr(right, field.name)), field.name


def assert_numerically_equal(left, right):
    """HH_260906 - Permit only machine-scale tensor-layout arithmetic differences, not gate relaxation."""
    epsilon = torch.finfo(left.dtype).eps
    assert bool(torch.all(torch.abs(left - right) <= 8 * epsilon * (1 + torch.abs(right))))


@pytest.mark.parametrize("dtype", [torch.float32, torch.float64])
def test_batch_and_candidate_permutations_do_not_change_decoded_content(dtype):
    speed, duration, curvature = varied_inputs(dtype)
    original = primitive.decode_stop_primitive(speed, duration, curvature)
    batch_order = torch.tensor([4, 1, 3, 0, 2])
    candidate_order = torch.tensor([5, 2, 0, 4, 1, 3])
    permuted = primitive.decode_stop_primitive(
        speed[batch_order], duration[batch_order][:, candidate_order],
        curvature[batch_order][:, candidate_order],
    )
    for field in fields(primitive.StopPrimitiveResult):
        expected = getattr(original, field.name)[batch_order][:, candidate_order]
        assert_numerically_equal(getattr(permuted, field.name), expected)


@pytest.mark.parametrize("dtype", [torch.float32, torch.float64])
def test_noncontiguous_dense_inputs_preserve_numerical_outputs(dtype):
    original_inputs = varied_inputs(dtype)
    strided_inputs = []
    for value in original_inputs:
        storage = torch.empty((*value.shape, 2), dtype=dtype)
        storage[..., 0] = value
        storage[..., 1] = -123.0
        strided = storage[..., 0]
        assert not strided.is_contiguous()
        strided_inputs.append(strided)
    original = primitive.decode_stop_primitive(*original_inputs)
    strided = primitive.decode_stop_primitive(*strided_inputs)
    for field in fields(primitive.StopPrimitiveResult):
        assert_numerically_equal(getattr(strided, field.name), getattr(original, field.name))


@pytest.mark.parametrize("dtype", [torch.float32, torch.float64])
@pytest.mark.parametrize("speed_value", [0.0, 0.07, 30.0 / 3.6])
def test_finite_extreme_latents_have_finite_forward_and_backward(dtype, speed_value):
    limit = torch.finfo(dtype).max
    speed = torch.tensor([speed_value], dtype=dtype, requires_grad=True)
    duration = torch.tensor([[-limit, limit, -limit, limit, -limit, limit]],
                            dtype=dtype, requires_grad=True)
    curvature = torch.full((1, 6, 64), limit, dtype=dtype)
    curvature[:, ::2, :] = -limit
    curvature.requires_grad_()
    result = primitive.decode_stop_primitive(speed, duration, curvature)
    for field in fields(primitive.StopPrimitiveResult):
        assert bool(torch.isfinite(getattr(result, field.name)).all())
    loss = result.xy_base_m.square().sum() + result.speed_mps.sum() + result.heading_rad.square().sum()
    loss.backward()
    for value in (speed, duration, curvature):
        assert value.grad is not None
        assert bool(torch.isfinite(value.grad).all())
    assert int(duration.grad.count_nonzero()) == 0
    assert int(curvature.grad.count_nonzero()) == 0
    if speed_value == 0.0:
        assert int(result.xy_base_m.count_nonzero()) == 0
        assert int(result.speed_mps.count_nonzero()) == 0
        assert int(result.heading_rad.count_nonzero()) == 0


@pytest.mark.parametrize("dtype", [torch.float32, torch.float64])
def test_smallest_normal_speed_remains_finite_and_ends_at_exact_zero(dtype):
    speed = torch.tensor([torch.finfo(dtype).tiny], dtype=dtype, requires_grad=True)
    duration = torch.zeros((1, 6), dtype=dtype, requires_grad=True)
    curvature = torch.ones((1, 6, 64), dtype=dtype, requires_grad=True)
    result = primitive.decode_stop_primitive(speed, duration, curvature)
    assert bool(torch.isfinite(result.xy_base_m).all())
    assert bool(torch.all(result.speed_mps >= 0))
    assert int(result.speed_mps[:, :, -1].count_nonzero()) == 0
    (result.xy_base_m.sum() + result.speed_mps.sum()).backward()
    assert all(bool(torch.isfinite(value.grad).all()) for value in (speed, duration, curvature))


@pytest.mark.parametrize("dtype", [torch.float32, torch.float64])
def test_post_stop_curvature_cannot_rotate_translate_or_backpropagate(dtype):
    speed = torch.tensor([1.0], dtype=dtype)
    duration = torch.full((1, 6), -3.0, dtype=dtype)
    curvature = torch.linspace(-1.0, 1.0, 384, dtype=dtype).reshape(1, 6, 64).requires_grad_()
    result = primitive.decode_stop_primitive(speed, duration, curvature)
    times = torch.arange(1, 65, dtype=dtype) * .1
    stopped = times >= result.stop_duration_s[..., None]
    assert bool(stopped.any())
    assert int(result.speed_mps[stopped].count_nonzero()) == 0
    for candidate in range(6):
        first = int(torch.nonzero(stopped[0, candidate])[0].item())
        assert first > 0
        for value in (result.xy_base_m, result.heading_rad):
            tail = value[0, candidate, first:]
            previous = value[0, candidate, first - 1].expand_as(tail)
            assert torch.equal(tail, previous)
    changed = curvature.detach().clone()
    changed[stopped] = torch.finfo(dtype).max
    assert_results_equal(result, primitive.decode_stop_primitive(speed, duration, changed))
    (result.xy_base_m.square().sum() + result.heading_rad.square().sum()).backward()
    assert int(curvature.grad[stopped].count_nonzero()) == 0


def test_future_curvature_parameters_cannot_change_earlier_endpoints():
    speed, duration, curvature = varied_inputs()
    original = primitive.decode_stop_primitive(speed, duration, curvature)
    changed = curvature.clone()
    changed[:, :, 20:] = 10.0
    output = primitive.decode_stop_primitive(speed, duration, changed)
    assert torch.equal(output.xy_base_m[:, :, :20], original.xy_base_m[:, :, :20])
    assert torch.equal(output.heading_rad[:, :, :20], original.heading_rad[:, :, :20])
    assert torch.equal(output.speed_mps, original.speed_mps)


def test_candidate_loss_does_not_leak_gradients_to_other_candidates_or_batches():
    raw = tuple(value.requires_grad_() for value in varied_inputs())
    output = primitive.decode_stop_primitive(*raw)
    output.xy_base_m[3, 2].square().sum().backward()
    assert int(raw[0].grad[[0, 1, 2, 4]].count_nonzero()) == 0
    for value in raw[1:]:
        gradients = value.grad.clone()
        assert bool(gradients[3, 2].abs().sum() > 0)
        gradients[3, 2] = 0
        assert int(gradients.count_nonzero()) == 0


@pytest.mark.parametrize("field", [0, 1, 2])
@pytest.mark.parametrize("kind", ["bool", "sparse"])
def test_unsupported_tensor_representation_is_rejected(field, kind):
    raw = list(varied_inputs())
    raw[field] = raw[field].bool() if kind == "bool" else raw[field].to_sparse()
    with pytest.raises(ValueError, match="dense float32 or float64"):
        primitive.decode_stop_primitive(*raw)


def test_research_callable_has_only_declared_causal_state_and_latent_inputs():
    signature = inspect.signature(primitive.decode_stop_primitive)
    assert tuple(signature.parameters) == ("current_speed_mps", "duration_logits", "curvature_logits")
    assert all(parameter.kind == inspect.Parameter.POSITIONAL_OR_KEYWORD
               and parameter.default == inspect.Parameter.empty
               for parameter in signature.parameters.values())
    with pytest.raises(TypeError, match="unexpected keyword argument"):
        primitive.decode_stop_primitive(*varied_inputs(), future_xy=torch.zeros(1))


def test_research_id_is_rejected_by_legacy_runtime_bundle_configuration():
    assert primitive.RESEARCH_ID not in SUPPORTED_MODEL_IDS
    assert primitive.RESEARCH_ID not in bundle.SUPPORTED_MODEL_IDS
    config = ModelConfig().to_dict()
    config["model_id"] = primitive.RESEARCH_ID
    with pytest.raises(ContractError, match="not supported by bundle v1"):
        bundle._validate_model_config(config, "0" * 64)
