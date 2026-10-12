"""HH_260906 - Model shape/gradient checks are not perception performance evidence."""
import torch
from portable_e2e.map_perception import MapPerception,LABELS
from portable_e2e.map_perception import semantic_loss


def test_rgb_only_dense_output_and_backward():
    model=MapPerception()
    y=model(torch.zeros(2,3,18,32))
    assert y.shape==(2,len(LABELS),18,32)
    y.square().mean().backward()
    assert all(p.grad is not None and torch.isfinite(p.grad).all() for p in model.parameters())


def test_odd_input_preserves_image_shape():
    assert MapPerception()(torch.zeros(1,3,19,33)).shape==(1,len(LABELS),19,33)


def test_explicit_loss_matches_weighted_cross_entropy_and_gradient():
    # HH_260906 - CPU reference validates the objective is unchanged by the CUDA compatibility fix.
    torch.manual_seed(3)
    x=torch.randn(2,7,9,13,requires_grad=True)
    y=torch.randint(0,7,(2,9,13));w=torch.arange(1,8).float()
    actual=semantic_loss(x,y,w)
    expected=torch.nn.functional.cross_entropy(x,y,weight=w)
    assert torch.allclose(actual,expected,atol=1e-6)
    g1=torch.autograd.grad(actual,x,retain_graph=True)[0]
    g2=torch.autograd.grad(expected,x)[0]
    assert torch.allclose(g1,g2,atol=1e-7)
