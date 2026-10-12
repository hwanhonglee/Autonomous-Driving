"""HH_260906 - Check the existing twelve-candidate model and regret objective together before the new whole-model campaign."""

import pytest

torch = pytest.importorskip("torch")

from portable_e2e.losses import TrajectoryLossConfig, trajectory_loss
from portable_e2e.model import PerspectiveTrajectoryModel
from test_portable_e2e_stopmix_model import config, inputs


@pytest.fixture(autouse=True)
def cpu_threads():
    previous = torch.get_num_threads()
    torch.set_num_threads(1)
    yield
    torch.set_num_threads(previous)


@pytest.mark.parametrize("speed", [0., 2., 8.])
def test_twelve_candidate_whole_model_loss_has_finite_backward_and_updates_parameters(speed):
    cfg = config()
    torch.manual_seed(20260903)
    model = PerspectiveTrajectoryModel(cfg)
    before = {name: value.detach().clone() for name, value in model.named_parameters()}
    optimizer = torch.optim.AdamW(model.parameters(), lr=1.e-4)
    xy, velocity, logits = model(**inputs(cfg, speed=speed))
    target_speed = torch.full((2, 64), speed)
    target_xy = torch.zeros(2, 64, 2)
    target_xy[..., 0] = torch.arange(1, 65) * .1 * speed
    mask = torch.ones(2, 64, dtype=torch.bool)
    values = trajectory_loss(xy, velocity, logits, target_xy, target_speed, mask,
                             TrajectoryLossConfig(candidate_regret_weight=.1), target_yaw=torch.zeros(2, 64))
    assert xy.shape == (2, 12, 64, 2) and logits.shape == (2, 12)
    assert torch.isfinite(values["loss"]) and torch.isfinite(values["candidate_regret_loss"])
    values["loss"].backward()
    assert all(parameter.requires_grad for parameter in model.parameters())
    assert all(parameter.grad is None or torch.isfinite(parameter.grad).all() for parameter in model.parameters())
    optimizer.step()
    changed = [name for name, value in model.named_parameters() if not torch.equal(value, before[name])]
    assert changed and any("candidate_head" not in name and "stop_candidate_head" not in name for name in changed)


def test_regret_auxiliary_changes_only_direct_logit_gradient_at_fixed_k12_outputs():
    # HH_260906 - Shared-network effects are possible, but detached costs must not add direct trajectory-output gradients.
    batch, candidates, points = 2, 12, 64
    generator = torch.Generator().manual_seed(1147)
    values = [torch.rand(batch, candidates, points, 2, generator=generator),
              torch.rand(batch, candidates, points, generator=generator),
              torch.rand(batch, candidates, generator=generator)]
    target_xy, target_speed = torch.zeros(batch, points, 2), torch.zeros(batch, points)
    mask = torch.ones(batch, points, dtype=torch.bool)
    gradients = []
    for weight in (0., .1):
        supplied = [value.clone().requires_grad_() for value in values]
        loss = trajectory_loss(*supplied, target_xy, target_speed, mask,
                               TrajectoryLossConfig(candidate_regret_weight=weight))["loss"]
        gradients.append(torch.autograd.grad(loss, supplied))
    assert torch.equal(gradients[0][0], gradients[1][0])
    assert torch.equal(gradients[0][1], gradients[1][1])
    assert not torch.equal(gradients[0][2], gradients[1][2])

