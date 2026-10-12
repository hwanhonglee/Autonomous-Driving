"""HH_261002 - Independently check fixed source/image sign loss math on synthetic CPU tensors, never actual TRAIN or GPU data."""

import math

import pytest
import torch

from learning.portable_e2e import map_sign_margin_loss_v1_20261002 as loss


GROUP_A = "Town01_original497"
GROUP_B = "Town02_original232"


def auxiliary(logits, target, groups):
    result, stats = loss.sign_margin_auxiliary(logits, target, groups)
    assert type(stats) is dict
    return result


@pytest.mark.parametrize("count", [1, 3, 64])
def test_hard_negative_uses_every_valid_pixel_when_at_most64(count):
    values = torch.arange(1, count + 1, dtype=torch.float32, requires_grad=True)
    result, _ = loss.deterministic_hard_negative_mean(values)
    assert result.item() == pytest.approx((count + 1) / 2, abs=1e-6)
    result.backward()
    assert torch.allclose(values.grad, torch.full_like(values, 1 / count), atol=1e-7, rtol=0)


def test_hard_negative_empty_has_no_fabricated_pixels_but_retains_gradient_path():
    values = torch.empty(0, dtype=torch.float32, requires_grad=True)
    result, _ = loss.deterministic_hard_negative_mean(values)
    assert result.item() == 0 and result.requires_grad
    result.backward()
    assert values.grad is not None and values.grad.numel() == 0


def test_top64_exact_values_and_gradient_denominator():
    values = torch.arange(100, dtype=torch.float32, requires_grad=True)
    result, _ = loss.deterministic_hard_negative_mean(values)
    assert result.item() == pytest.approx(67.5, abs=1e-6)
    result.backward()
    assert torch.equal(values.grad[:36], torch.zeros(36))
    assert torch.allclose(values.grad[36:], torch.full((64,), 1 / 64), atol=1e-7, rtol=0)
    assert values.grad.sum().item() == pytest.approx(1., abs=1e-6)


def test_fractional_threshold_ties_have_no_index_order_preference():
    # HH_261002 - Thirty-two strict winners leave32 of48 equal threshold pixels: each tie receives2/3 of one slot.
    values = torch.tensor([2.] * 32 + [1.] * 48, requires_grad=True)
    result, _ = loss.deterministic_hard_negative_mean(values)
    assert result.item() == pytest.approx(1.5, abs=1e-6)
    result.backward()
    assert torch.allclose(values.grad[:32], torch.full((32,), 1 / 64), atol=1e-7, rtol=0)
    assert torch.allclose(values.grad[32:], torch.full((48,), 1 / 96), atol=1e-7, rtol=0)
    assert values.grad.sum().item() == pytest.approx(1., abs=1e-6)
    reordered = torch.tensor([1., 2.] * 32 + [1.] * 16, requires_grad=True)
    other, _ = loss.deterministic_hard_negative_mean(reordered)
    other.backward()
    assert other.item() == pytest.approx(result.item(), abs=1e-6)
    assert torch.allclose(reordered.grad[reordered.detach() == 1], torch.full((48,), 1 / 96), atol=1e-7, rtol=0)


def test_all_ties_distribute_exact64_mass_over_all80_valid_pixels():
    values = torch.ones(80, requires_grad=True)
    result, _ = loss.deterministic_hard_negative_mean(values)
    result.backward()
    assert result.item() == 1.
    assert torch.allclose(values.grad, torch.full((80,), 1 / 80), atol=1e-7, rtol=0)


@pytest.mark.parametrize("target_class, expected", [(5, .5 * math.log(7)), (0, .5 * math.log(7 / 6))])
def test_sign_vs_existing_other6_probabilities_and_absent_component_zero(target_class, expected):
    logits = torch.zeros(1, 7, 2, 2, requires_grad=True)
    target = torch.full((1, 2, 2), target_class, dtype=torch.int64)
    result = auxiliary(logits, target, (GROUP_A,))
    assert result.item() == pytest.approx(expected, abs=1e-6)
    result.backward()
    assert torch.isfinite(logits.grad).all()
    assert torch.allclose(logits.grad.sum(1), torch.zeros(1, 2, 2), atol=1e-6, rtol=0)


def test_negative_image_denominator_keeps_all_sign_image_as_zero_not_omitted():
    logits = torch.zeros(2, 7, 2, 2, requires_grad=True)
    target = torch.stack((torch.full((2, 2), 5), torch.zeros(2, 2))).to(torch.int64)
    result = auxiliary(logits, target, (GROUP_A, GROUP_A))
    assert result.item() == pytest.approx(.5 * math.log(7) + .25 * math.log(7 / 6), abs=1e-6)


def test_present_source_groups_are_equal_not_weighted_by_image_count():
    logits = torch.zeros(2, 7, 2, 2, requires_grad=True)
    target = torch.stack((torch.full((2, 2), 5), torch.zeros(2, 2))).to(torch.int64)
    expected = .25 * math.log(7) + .25 * math.log(7 / 6)
    result = auxiliary(logits, target, (GROUP_A, GROUP_B))
    assert result.item() == pytest.approx(expected, abs=1e-6)
    repeated_logits = logits[[0, 0, 0, 1]]
    repeated_target = target[[0, 0, 0, 1]]
    repeated = auxiliary(repeated_logits, repeated_target, (GROUP_A, GROUP_A, GROUP_A, GROUP_B))
    assert repeated.item() == pytest.approx(result.item(), abs=1e-6)


def test_positive_image_means_not_positive_pixel_count_weighted():
    logits = torch.zeros(2, 7, 1, 5)
    logits[0, 5] = -1.
    logits[1, 5] = 1.
    logits.requires_grad_()
    target = torch.tensor([[[5, 0, 0, 0, 0]], [[5, 5, 5, 5, 0]]], dtype=torch.int64)
    result = auxiliary(logits, target, (GROUP_A, GROUP_A))
    positive = (math.log1p(6 * math.exp(1)) + math.log1p(6 * math.exp(-1))) / 2
    negative = (math.log1p(math.exp(-1) / 6) + math.log1p(math.exp(1) / 6)) / 2
    assert result.item() == pytest.approx(.5 * positive + .5 * negative, abs=1e-6)


def test_existing_nonsign_class_permutation_cannot_change_auxiliary():
    logits = torch.arange(42, dtype=torch.float32).reshape(1, 7, 2, 3) / 20
    target = torch.tensor([[[0, 5, 1], [2, 3, 4]]], dtype=torch.int64)
    result = auxiliary(logits, target, (GROUP_A,))
    reordered = logits[:, [1, 0, 2, 3, 4, 5, 6]]
    swapped = target.clone()
    swapped[target == 0] = 1; swapped[target == 1] = 0
    assert auxiliary(reordered, swapped, (GROUP_A,)).item() == pytest.approx(result.item(), abs=1e-6)


def test_candidate_calls_original_CE_once_with_identical_inputs_and_fixed_coefficient():
    logits = torch.zeros(1, 7, 2, 2, requires_grad=True)
    target = torch.full((1, 2, 2), 5, dtype=torch.int64)
    weights = torch.ones(7)
    calls = []
    def original_ce(observed_logits, observed_target, observed_weights):
        assert observed_logits is logits and observed_target is target and observed_weights is weights
        calls.append(True)
        return logits.sum() * 0 + 3.
    combined, stats = loss.candidate_loss(logits, target, weights, (GROUP_A,), original_ce)
    assert type(stats) is dict and calls == [True]
    assert combined.item() == pytest.approx(3. + .10 * .5 * math.log(7), abs=1e-6)


@pytest.mark.parametrize("groups", [(), (GROUP_A, GROUP_B), ("invented_DEV_group",), (True,)])
def test_source_metadata_is_exact_count_and_only_original_TRAIN_groups(groups):
    with pytest.raises((ValueError, TypeError)):
        auxiliary(torch.zeros(1, 7, 2, 2), torch.zeros(1, 2, 2, dtype=torch.int64), groups)
