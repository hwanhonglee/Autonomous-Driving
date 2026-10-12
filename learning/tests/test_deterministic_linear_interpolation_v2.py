"""HH_261001 - Check numerical equivalence and serial ownership without weakening strict determinism."""

import pytest
import torch
from torch.nn import functional as F

from portable_e2e.contract import ContractError
from portable_e2e.deterministic_linear_interpolation_v2 import (
    DeterministicPathTimeDecoderV2, fixed_linear_interpolation,
    fixed_decoder_interpolation_scope, interpolation_matrix)
from portable_e2e.path_time_decoder_v2 import PathTimeDecoderV2


@pytest.mark.parametrize("dtype", [torch.float32, torch.float64])
@pytest.mark.parametrize("shape", [(1, 1, 33), (4, 1, 33), (2, 3, 33)])
def test_forward_backward_matches_native_cpu_interpolation(dtype, shape):
    torch.manual_seed(103)
    first = torch.randn(*shape, dtype=dtype, requires_grad=True)
    second = first.detach().clone().requires_grad_()
    expected = F.interpolate(first, size=257, mode="linear", align_corners=True)
    result = fixed_linear_interpolation(second)
    weights = torch.randn_like(expected)
    (expected*weights).sum().backward(); (result*weights).sum().backward()
    atol = 1e-6 if dtype == torch.float32 else 1e-12
    assert torch.allclose(result, expected, atol=atol, rtol=atol)
    assert torch.allclose(first.grad, second.grad, atol=atol, rtol=atol)


def test_matrix_is_exact_eighth_fraction_partition_and_keeps_endpoints():
    matrix = interpolation_matrix(dtype=torch.float64)
    assert torch.equal(matrix.sum(0), torch.ones(257, dtype=torch.float64))
    assert torch.equal(matrix*8, (matrix*8).round())
    assert matrix[0, 0] == 1. and matrix[32, 256] == 1.


@pytest.mark.parametrize("shape", [(1, 33), (1, 1, 32), (0, 1, 33), (1, 0, 33)])
def test_wrong_fixed_shape_is_rejected(shape):
    with pytest.raises(ContractError, match="nonempty"):
        fixed_linear_interpolation(torch.zeros(*shape))


def test_nonfinite_values_are_rejected():
    value = torch.zeros(1, 1, 33); value[0, 0, 0] = float("nan")
    with pytest.raises(ContractError, match="finite"):
        fixed_linear_interpolation(value)


def test_dispatch_changes_only_exact_decoder_signature_and_restores():
    original = F.interpolate
    value = torch.randn(2, 1, 33)
    with fixed_decoder_interpolation_scope(interpolation_matrix()) as receipt:
        actual = F.interpolate(value, size=257, mode="linear", align_corners=True)
        other = F.interpolate(value, size=129, mode="linear", align_corners=True)
        assert torch.allclose(actual, original(value, size=257, mode="linear", align_corners=True), atol=1e-6, rtol=1e-6)
        assert torch.equal(other, original(value, size=129, mode="linear", align_corners=True))
    assert F.interpolate is original
    assert receipt["fixed_calls"] == receipt["unrelated_calls_delegated"] == 1


def test_dispatch_restores_after_owned_exception_and_rejects_nested_scope():
    original = F.interpolate
    with pytest.raises(ContractError, match="nest"):
        with fixed_decoder_interpolation_scope(interpolation_matrix()):
            with fixed_decoder_interpolation_scope(interpolation_matrix()):
                pass
    assert F.interpolate is original


@pytest.mark.parametrize("dtype", [torch.float32, torch.float64])
def test_whole_decoder_forward_and_gradient_matches_frozen_math(dtype):
    arguments = [torch.full((2, 33), .005, dtype=dtype, requires_grad=True),
        torch.full((2, 33), 2.3, dtype=dtype, requires_grad=True), torch.full((2,), 2.484, dtype=dtype),
        torch.zeros(2, dtype=torch.long), torch.zeros(2, dtype=dtype), torch.full((2,), 64., dtype=dtype)]
    second = [value.detach().clone().requires_grad_(value.requires_grad) for value in arguments]
    baseline = PathTimeDecoderV2()(*arguments)
    decoder = DeterministicPathTimeDecoderV2()
    result = decoder(*second)
    atol = 1e-6 if dtype == torch.float32 else 1e-12
    for key in baseline:
        if baseline[key].dtype == torch.bool:
            assert torch.equal(baseline[key], result[key])
        else:
            assert torch.allclose(baseline[key], result[key], atol=atol, rtol=atol)
    (baseline["xy"].square().mean()+baseline["instantaneous_speed"].square().mean()).backward()
    (result["xy"].square().mean()+result["instantaneous_speed"].square().mean()).backward()
    assert torch.allclose(arguments[0].grad, second[0].grad, atol=atol, rtol=atol)
    assert torch.allclose(arguments[1].grad, second[1].grad, atol=atol, rtol=atol)
    assert decoder.last_interpolation_receipt["fixed_calls"] == 2
    assert "fixed_interpolation_matrix" not in decoder.state_dict()


def test_revision_scope_preserves_failed_v1_file_and_restores_bindings():
    from scripts.e2e import train_temporal_v2_low_speed_joint_cuda_deterministic_20261001 as revision
    frozen = revision.frozen
    old_decoder, old_schema, old_snapshot = frozen.PathTimeDecoderV2, frozen.SCHEMA, frozen.source_snapshot
    with revision.revised_training_scope(dict(test_only=True)):
        assert frozen.PathTimeDecoderV2 is DeterministicPathTimeDecoderV2
        assert frozen.SCHEMA == revision.SCHEMA
        assert str(revision.Path(revision.__file__).relative_to(revision.ROOT)) in frozen.source_snapshot()
    assert frozen.PathTimeDecoderV2 is old_decoder and frozen.SCHEMA == old_schema
    assert frozen.source_snapshot is old_snapshot
