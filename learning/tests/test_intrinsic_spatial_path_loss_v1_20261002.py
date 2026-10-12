"""HH_261002 - Analytic/source-only geometry and finite-gradient proof, not a trained driving result."""

import inspect
import math
from unittest import mock

import pytest
import torch
from torch.nn import functional as F

from portable_e2e.contract import ContractError
from portable_e2e.intrinsic_spatial_path_loss_v1_20261002 import IntrinsicSpatialPathLossV1
from portable_e2e.path_time_decoder_v2 import PathTimeDecoderV2


def inputs(*, dtype=torch.float64, batch=1):
    curvature = torch.zeros(batch, 33, dtype=dtype, requires_grad=True)
    distance = torch.arange(1, 65, dtype=dtype)[None].expand(batch, -1) * .25
    target = torch.stack((distance, torch.zeros_like(distance)), -1)
    return curvature, target, torch.ones(batch, 64, dtype=torch.bool)


def circle(k, distance):
    return torch.stack((torch.sin(k * distance) / k, (1 - torch.cos(k * distance)) / k), -1)


@pytest.mark.parametrize("dtype", [torch.float32, torch.float64])
def test_straight_exact_geometry_zero_loss_finite_gradient_and_current_origin(dtype):
    curvature, target, valid = inputs(dtype=dtype)
    result = IntrinsicSpatialPathLossV1()(curvature, target, valid)
    assert torch.equal(result["sampled_predicted_xy"], target)
    assert result["true_future_chord_arc_m"][0, 0] == .25  # not the first future point as origin
    assert result["total"] == 0 and result["assessed_point_count"].tolist() == [64]
    result["total"].backward()
    assert torch.isfinite(curvature.grad).all() and torch.count_nonzero(curvature.grad) == 0


@pytest.mark.parametrize("k", [-.15, -.025, .025, .15])
def test_clockwise_counterclockwise_constant_curvature_analytic_chord_floor(k):
    curvature, _, valid = inputs()
    distance = torch.arange(1, 65, dtype=torch.float64)[None] * .25
    target = circle(k, distance)
    predicted_k = torch.full_like(curvature, k, requires_grad=True)
    result = IntrinsicSpatialPathLossV1()(predicted_k, target, valid)
    chord = 2 * math.sin(abs(k) * .25 / 2) / abs(k)
    polygonal_arc = torch.arange(1, 65, dtype=torch.float64)[None] * chord
    assert torch.allclose(result["true_future_chord_arc_m"], polygonal_arc, atol=1e-12, rtol=1e-12)
    assert torch.allclose(result["sampled_predicted_xy"], circle(k, polygonal_arc), atol=1e-11, rtol=1e-11)
    # HH_261002 - Chord arc is shorter than continuous arc; do not advertise an exact-zero perfect-circle floor.
    assert result["total"] > 0 and torch.norm(result["sampled_predicted_xy"] - target, dim=-1).max() < .005
    result["total"].backward(); assert torch.isfinite(predicted_k.grad).all()


@pytest.mark.parametrize("k", [-.08, .08])
def test_zero_curvature_receives_correct_signed_repair_gradient(k):
    curvature, _, valid = inputs()
    target = circle(k, torch.arange(1, 65, dtype=torch.float64)[None] * .25)
    result = IntrinsicSpatialPathLossV1()(curvature, target, valid)
    result["total"].backward()
    assert torch.isfinite(curvature.grad).all() and curvature.grad.sum().item() * k < 0


def test_reversed_true_displacement_is_not_rotated_or_clipped_into_a_forward_label():
    curvature, target, valid = inputs(); target = -target; original = target.clone()
    result = IntrinsicSpatialPathLossV1()(curvature, target, valid)
    assert torch.equal(target, original) and result["total"] > 1.
    assert torch.all(result["sampled_predicted_xy"][..., 0] > 0)
    result["total"].backward(); assert torch.isfinite(curvature.grad).all()


def test_masked_suffix_has_no_fake_bridge_or_label_gradient():
    curvature, target, valid = inputs(batch=2)
    valid[0, 12:] = False; target[0, 12:] = 1e100
    target.requires_grad_(); original = target.detach().clone()
    result = IntrinsicSpatialPathLossV1()(curvature, target, valid)
    assert result["assessed_point_count"].tolist() == [12, 64]
    assert torch.equal(target.detach(), original)
    assert torch.all(result["true_future_chord_arc_m"][0, 12:] == 3.)
    result["total"].backward()
    assert target.grad is None and torch.isfinite(curvature.grad).all()


def test_exact_64m_boundary_assessed_and_outside_original_labels_retained():
    curvature, _, valid = inputs()
    distance = torch.arange(1, 65, dtype=torch.float64)[None] * 2.
    target = torch.stack((distance, torch.zeros_like(distance)), -1); original = target.clone()
    result = IntrinsicSpatialPathLossV1()(curvature, target, valid)
    assert result["assessed_point_count"].tolist() == [32]
    assert result["valid_outside_64m_point_count"].tolist() == [32]
    assert result["sampled_prediction_valid"][0, 31] and not result["sampled_prediction_valid"][0, 32]
    assert result["true_future_chord_arc_m"][0, -1] == 128 and torch.equal(target, original)
    assert result["total"] == 0


@pytest.mark.parametrize("stationary", [False, True])
def test_empty_or_stationary_supervision_cannot_be_reported_as_nonzero_training_gain(stationary):
    curvature, target, valid = inputs()
    if stationary: target.zero_()
    else: valid.zero_()
    result = IntrinsicSpatialPathLossV1()(curvature, target, valid)
    assert result["total"] == 0 and result["nonzero_arc_anchor_count"] == 0
    assert not result["nonzero_supervision_present"] and not result["stationary_curvature_labels_created"]
    assert result["assessed_point_count"].item() == (64 if stationary else 0)
    result["total"].backward(); assert torch.isfinite(curvature.grad).all()


def test_per_anchor_mean_does_not_weight_an_anchor_by_its_longer_prefix():
    curvature, target, valid = inputs(batch=2); target[..., 1] = .2
    valid[0, 8:] = False
    result = IntrinsicSpatialPathLossV1()(curvature, target, valid)
    separate = [IntrinsicSpatialPathLossV1()(curvature[i:i+1], target[i:i+1], valid[i:i+1])["total"] for i in range(2)]
    assert torch.allclose(result["total"], torch.stack(separate).mean(), atol=1e-12, rtol=1e-12)


def test_frozen_spatial_math_is_reused_with_only_one_deterministic_interpolation():
    curvature, target, valid = inputs(); curvature = (curvature + .01).detach().requires_grad_()
    original = F.interpolate; decoder = PathTimeDecoderV2()
    dense, heading, xy = decoder._spatial_shape(curvature)
    with mock.patch.object(PathTimeDecoderV2, "forward", side_effect=AssertionError("temporal retiming forbidden")), \
        mock.patch.object(PathTimeDecoderV2, "_go_speed", side_effect=AssertionError("speed projection forbidden")):
        result = IntrinsicSpatialPathLossV1()(curvature, target, valid)
    expected = decoder._sample_shape(result["true_future_chord_arc_m"].to(curvature.dtype), dense, heading, xy)
    assert torch.allclose(result["sampled_predicted_xy"], expected, atol=1e-12, rtol=1e-12)
    assert result["interpolation_receipt"]["fixed_calls"] == 1 and F.interpolate is original
    assert result["future_arc_is_loss_only_not_causal_support"] and not result["time_supervision"]
    assert not result["dataset_admitted"] and not result["model_promoted"]


def test_no_new_model_parameters_or_speed_v0_time_or_heading_inputs():
    loss = IntrinsicSpatialPathLossV1()
    assert list(loss.parameters()) == [] and loss.state_dict() == {}
    assert list(inspect.signature(loss.forward).parameters) == ["curvature_knots", "target_xy", "target_valid"]
    curvature, target, valid = inputs(); speed = torch.full((1, 33), 8.4, requires_grad=True)
    result = loss(curvature, target, valid); result["total"].backward()
    assert speed.grad is None and not result["speed_or_v0_changed"]


@pytest.mark.parametrize("mutation", ["mask_hole", "mask_dtype", "xy_shape", "curvature_shape", "empty_batch", "dtype", "nan_xy", "nan_k", "bound", "arc_overflow"])
def test_malformed_or_nonfinite_original_data_is_not_repaired(mutation):
    curvature, target, valid = inputs()
    if mutation == "mask_hole": valid[0, 4] = False
    elif mutation == "mask_dtype": valid = valid.float()
    elif mutation == "xy_shape": target = target[..., :1]
    elif mutation == "curvature_shape": curvature = curvature[:, :-1]
    elif mutation == "empty_batch": curvature, target, valid = curvature[:0], target[:0], valid[:0]
    elif mutation == "dtype": target = target.float()
    elif mutation == "nan_xy": target[0, -1, 0] = float("nan")
    elif mutation == "nan_k": curvature = curvature.detach(); curvature[0, 0] = float("nan")
    elif mutation == "bound": curvature = curvature.detach() + .20001
    elif mutation == "arc_overflow": target[0, 0, 0] = 1e308; target[0, 1, 0] = -1e308
    with pytest.raises(ContractError): IntrinsicSpatialPathLossV1()(curvature, target, valid)


def test_analytic_finite_difference_gradient_matches_autograd():
    curvature, target, valid = inputs(); curvature = (curvature + .017).detach().requires_grad_()
    target = circle(.024, torch.arange(1, 65, dtype=torch.float64)[None] * .19)
    assert torch.autograd.gradcheck(lambda value: IntrinsicSpatialPathLossV1()(value, target, valid)["total"],
        (curvature,), eps=1e-6, atol=1e-5, rtol=1e-4)
