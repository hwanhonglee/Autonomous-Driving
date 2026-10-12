"""HH_260906 - Check route interpolation geometry and progress gradients independently of driving approval."""
import torch
import pytest
from scripts.e2e.research_arc_progress_decoder import interpolate_route

pytestmark=pytest.mark.skipif(not hasattr(torch.linalg,'vector_norm'),reason='Requires training-venv PyTorch; remote numeric checks run separately')


def test_right_angle_progress_and_endpoint():
    route=torch.tensor([[[0.,0.],[2.,0.],[2.,-2.]]])
    mask=torch.ones((1,3),dtype=torch.bool)
    s=torch.tensor([[[1.,3.,5.]]],requires_grad=True)
    xy=interpolate_route(route,mask,s)
    torch.testing.assert_close(xy,torch.tensor([[[[1.,0.],[2.,-1.],[2.,-2.]]]]))
    xy.sum().backward()
    assert torch.isfinite(s.grad).all()


def test_padded_route_is_not_followed():
    route=torch.tensor([[[0.,0.],[2.,0.],[900.,900.]]])
    mask=torch.tensor([[True,True,False]])
    xy=interpolate_route(route,mask,torch.tensor([[[3.]]]))
    torch.testing.assert_close(xy,torch.tensor([[[[2.,0.]]]]))
