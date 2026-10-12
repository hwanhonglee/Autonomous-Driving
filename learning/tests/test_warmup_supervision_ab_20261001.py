"""HH_261001 - Source-only anchor suppression preserves native loss, targets and matched steps."""

import copy

import pytest
import torch

from scripts.e2e import warmup_supervision_research_20261001 as research
from scripts.e2e import run_ah_warmup_supervision_ab_20261001 as trainer


def tensors():
    torch.manual_seed(12)
    xy = torch.randn(4, 12, 64, 2).cumsum(2) * .05
    speed = torch.rand(4, 12, 64)
    xy[1, 6] = 0.; speed[1, 6] = 0.
    logits = torch.randn(4, 12)
    target_xy = torch.randn(4, 64, 2) * .03
    target_speed = torch.rand(4, 64)
    target_xy[1] = 0.; target_speed[1] = 0.
    batch = dict(target_xy=target_xy, target_speed_mps=target_speed,
                 target_valid=torch.ones(4, 64, dtype=torch.bool), target_yaw_rad=torch.zeros(4, 64))
    batch["target_valid"][0, -17:] = False
    batch["target_valid"][2, -32:] = False
    return xy.requires_grad_(), speed.requires_grad_(), logits.requires_grad_(), batch


def test_unweighted_original_loss_and_gradients_match():
    xy, speed, logits, batch = tensors()
    native = research.previous.base.trajectory_loss(xy, speed, logits, batch["target_xy"], batch["target_speed_mps"],
        batch["target_valid"], research.previous.base.LOSS_CONFIG, target_yaw=batch["target_yaw_rad"])
    near = research.previous.near_horizon_loss(xy, speed, batch["target_xy"], batch["target_speed_mps"],
        batch["target_valid"], native["oracle_candidate_index"])
    expected = native["loss"] + .5 * near
    got, _, _, oracle = research.weighted_original_loss(xy, speed, logits, batch, torch.ones(4))
    assert torch.equal(oracle, native["oracle_candidate_index"])
    assert torch.allclose(got, expected, atol=1e-6, rtol=1e-6)
    wanted_grads = torch.autograd.grad(expected, (xy, speed, logits), retain_graph=True)
    actual_grads = torch.autograd.grad(got, (xy, speed, logits))
    assert all(torch.allclose(a, b, atol=1e-6, rtol=1e-6) for a, b in zip(actual_grads, wanted_grads))


def test_masked_anchor_zero_gradient_stop_still_supervised_no_mutation():
    xy, speed, logits, batch = tensors()
    snapshots = {key: value.clone() for key, value in batch.items()}
    weights = torch.tensor([0., research.ACTIVE_WEIGHT, research.ACTIVE_WEIGHT, research.ACTIVE_WEIGHT])
    total, _, _, oracle = research.weighted_original_loss(xy, speed, logits, batch, weights)
    total.backward()
    assert int(oracle[1]) == 6
    assert all(torch.count_nonzero(value.grad[0]) == 0 for value in (xy, speed, logits))
    assert torch.count_nonzero(logits.grad[1]) > 0
    assert torch.count_nonzero(xy.grad[2]) > 0
    assert all(torch.equal(batch[key], value) for key, value in snapshots.items())


def test_half_open_source_intervals_and_future_not_used():
    phases = dict(stationary_warmup=dict(state_count=70, first_timestamp=.05, last_timestamp=3.5,
                                        control_source="full_service_brake"),
                  driving=dict(first_timestamp=3.55, last_timestamp=40.),
                  stationary_tail=dict(state_count=130, first_timestamp=40.05, last_timestamp=46.5))
    assert research.classify_anchor(3_549_999_999, phases) == "warmup"
    assert research.classify_anchor(3_550_000_000, phases) == "driving"
    assert research.classify_anchor(40_100_000_000, phases) == "tail"
    changed = copy.deepcopy(phases)
    changed["stationary_warmup"]["control_source"] = "unknown"
    with pytest.raises(ValueError):
        research.classify_anchor(3_400_000_000, changed)
    with pytest.raises(ValueError):
        research.classify_anchor(1, phases)


def test_shared_full_coverage_schedule_skip_and_mass():
    batches = research.phase.schedule(2787)
    warmup = set(range(35)) | set(range(309, 344)) | set(range(613, 648))
    skip = [i for i, row in enumerate(batches, 1) if all(j in warmup for j in row)]
    assert skip == [1693]
    assert batches[1692] == (617, 619, 322, 317)
    assert len(set(j for row in batches for j in row)) == 2787
    executed = [j for i, row in enumerate(batches, 1) if i not in skip for j in row]
    assert len(executed) == 11996
    assert sum(j not in warmup for j in executed) == 11549
    assert abs(sum(0. if j in warmup else research.ACTIVE_WEIGHT for j in executed) / len(executed) - 1.) < 1e-12


def test_skipped_step_does_not_mutate_adamw_parameter_or_momentum():
    parameter = torch.nn.Parameter(torch.tensor([2.]))
    optimizer = torch.optim.AdamW([parameter], lr=.1, weight_decay=.1)
    parameter.square().sum().backward(); optimizer.step()
    before = parameter.detach().clone()
    state = copy.deepcopy(optimizer.state_dict())
    # HH_261001 - Match the trainer's branch: no zero_grad, forward, backward, or step for skipped1693.
    for step in research.SKIP_ONE_BASED:
        if step in research.SKIP_ONE_BASED:
            continue
        raise AssertionError("unreachable optimizer step")
    assert torch.equal(parameter, before)
    for key, value in state["state"][0].items():
        actual = optimizer.state_dict()["state"][0][key]
        assert torch.equal(value, actual) if torch.is_tensor(value) else value == actual


def test_protocol_has_actual_update_accounting_and_no_other_intervention():
    contract = trainer.contract()
    assert contract["optimizer_updates"] == 2999
    assert contract["scheduled_steps"] == 3000
    assert contract["full_model_trainable"]
    assert contract["final_dev_after_both_checkpoints"]
    assert not contract["future_validity_masks_changed"]
    assert not contract["driving_futures_crossing_tail_changed"]
    assert not contract["initial_acceleration_auxiliary"]
