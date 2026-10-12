"""HH_261001 - Establish useful pre-clamp gradients without changing decoder behavior or STOP training."""

import torch

from portable_e2e.initial_acceleration_research import initial_acceleration_loss, capture_drive_latents


def inputs():
    raw = torch.full((1, 6, 64, 2), -1., requires_grad=True)
    current = torch.zeros(1)
    target = torch.arange(1, 65, dtype=torch.float32)[None] * .1
    valid = torch.ones(1, 64, dtype=torch.bool)
    return raw, current, target, valid, torch.zeros(1, dtype=torch.long)


def test_preclamp_gradient_escapes_zero_speed_deadzone():
    raw, current, target, valid, oracle = inputs()
    speed = (current[:, None] + torch.tanh(raw[:, 0, :5, 0]) * .29).clamp(0., 30 / 3.6)
    speed_loss = (speed - target[:, :5]).square().mean()
    decoded_gradient = torch.autograd.grad(speed_loss, raw, retain_graph=True)[0]
    assert decoded_gradient.abs().sum() == 0
    before = tuple(t.detach().clone() for t in (raw, current, target, valid, oracle))
    loss, metrics = initial_acceleration_loss(raw, current, target, valid, oracle)
    loss.backward()
    assert bool((raw.grad[0, 0, :5, 0] < 0).all())
    assert raw.grad[:, 1:].abs().sum() == 0
    assert raw.grad[..., 1].abs().sum() == 0
    assert metrics["eligible_pairs"] == 5
    assert all(torch.equal(old, new) for old, new in zip(before, (raw, current, target, valid, oracle)))


def test_stop_oracle_has_zero_auxiliary_and_gradient():
    args = list(inputs())
    args[-1] = torch.tensor([8])
    loss, metrics = initial_acceleration_loss(*args)
    loss.backward()
    assert loss == 0 and args[0].grad.abs().sum() == 0
    assert metrics["eligible_pairs"] == 0


def test_negative_acceleration_retained_and_impossible_pair_excluded():
    raw, current, target, valid, oracle = inputs()
    raw = torch.ones_like(raw, requires_grad=True)
    current = torch.ones_like(current)
    target = torch.full_like(target, .5)
    target[0, :5] = torch.tensor([.9, .8, .7, .6, .5])
    loss, metrics = initial_acceleration_loss(raw, current, target, valid, oracle)
    loss.backward()
    assert bool((raw.grad[0, 0, :5, 0] > 0).all())
    assert metrics["deceleration_pairs"] == 5
    target[0, 0] = 4.
    _, metrics = initial_acceleration_loss(raw, current, target, valid, oracle)
    assert metrics["eligible_pairs"] == 3


def test_masked_invalid_pairs_and_reverse_current_do_not_become_labels():
    raw, current, target, valid, oracle = inputs()
    valid[:, 2:] = False
    target[:, 2:] = float("nan")
    current[:] = -.03
    loss, metrics = initial_acceleration_loss(raw, current, target, valid, oracle)
    assert torch.isfinite(loss) and metrics["eligible_pairs"] == 1


def test_hook_observes_without_modifying_and_is_removed():
    class Model(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.trajectory_head = torch.nn.Linear(2, 768)

        def forward(self, x):
            return self.trajectory_head(x)

    model = Model()
    x = torch.zeros(1, 2)
    before = model(x)
    with capture_drive_latents(model) as captured:
        after = model(x)
        assert torch.equal(before, after)
        assert torch.equal(captured[0].flatten(1), after)
    assert not model.trajectory_head._forward_hooks
