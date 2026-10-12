"""HH_261001 - Verify raw zero-speed gradients and honest missing-label masking."""

import pytest
import torch

from portable_e2e.contract import ContractError
from portable_e2e.temporal_losses_v2 import temporal_trajectory_loss_v2


def example():
    raw = dict(curvature_knots=torch.zeros(2, 33, requires_grad=True),
               preferred_speed_knots=torch.ones(2, 33, requires_grad=True),
               intent_logits=torch.zeros(2, 3, requires_grad=True),
               stop_distance_raw=torch.ones(2, 1, requires_grad=True))
    targets = dict(spatial_curvature=torch.zeros(2, 33), spatial_preferred_speed=torch.zeros(2, 33),
                   spatial_valid=torch.ones(2, 33, dtype=torch.bool),
                   intent_target=torch.zeros(2, dtype=torch.long), intent_valid=torch.zeros(2, dtype=torch.bool),
                   stop_distance_target=torch.zeros(2), stop_distance_valid=torch.zeros(2, dtype=torch.bool))
    return raw, targets


def test_raw_zero_speed_sends_nonzero_gradient_without_decoder():
    raw, target = example()
    result = temporal_trajectory_loss_v2(raw, target)
    result["total"].backward()
    assert torch.isfinite(raw["preferred_speed_knots"].grad).all()
    assert (raw["preferred_speed_knots"].grad > 0).all()
    assert result["projected_time_supervision"] is False
    assert result["intent"] == result["stop_distance"] == 0
    assert torch.count_nonzero(raw["intent_logits"].grad) == 0


def test_missing_spatial_labels_do_not_teach_false_zero():
    raw, target = example()
    target["spatial_valid"][:] = False
    result = temporal_trajectory_loss_v2(raw, target)
    result["total"].backward()
    assert result["total"] == 0
    assert torch.count_nonzero(raw["preferred_speed_knots"].grad) == 0


def test_verified_intent_is_supervised_only_where_present():
    raw, target = example()
    target["intent_valid"][0] = True
    target["intent_target"][0] = 1
    temporal_trajectory_loss_v2(raw, target)["total"].backward()
    assert torch.count_nonzero(raw["intent_logits"].grad[0]) == 3
    assert torch.count_nonzero(raw["intent_logits"].grad[1]) == 0


def test_temporal_loss_compares_instantaneous_not_chord_speed():
    raw, target = example()
    target.update(target_xy=torch.zeros(2, 64, 2), target_speed_mps=torch.ones(2, 64),
                  target_valid=torch.ones(2, 64, dtype=torch.bool))
    decoded = dict(xy=torch.zeros(2, 1, 64, 2), instantaneous_speed=torch.ones(2, 65),
                   speed=torch.full((2, 1, 64), 42.))
    result = temporal_trajectory_loss_v2(raw, target, decoded=decoded)
    assert result["temporal_speed"] == result["temporal_xy"] == 0
    assert result["projected_time_supervision"] is True


@pytest.mark.parametrize("key", ["spatial_curvature", "spatial_preferred_speed"])
def test_bad_target_values_are_rejected(key):
    raw, target = example()
    target[key][0, 0] = float("nan")
    with pytest.raises(ContractError, match="finite"):
        temporal_trajectory_loss_v2(raw, target)


def test_nonboolean_mask_is_rejected():
    raw, target = example()
    target["spatial_valid"] = target["spatial_valid"].float()
    with pytest.raises(ContractError, match="boolean"):
        temporal_trajectory_loss_v2(raw, target)
