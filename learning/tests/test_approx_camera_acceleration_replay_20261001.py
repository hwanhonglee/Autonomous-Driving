"""HH_261001 - Approximate replay changes only copied acceleration channels, not JPEGs or baseline tensors."""

import torch

from scripts.e2e.replay_approx_camera_acceleration_20261001 import tensor_hash, transform_history


def test_acceleration_intervention_preserves_all_other_tensor_bytes():
    tensors=dict(images=torch.randn(1,6,3,2,2),calibration=torch.randn(1,6,16),
        ego_history=torch.randn(1,10,13),ego_history_mask=torch.ones(1,10,dtype=torch.bool),
        route_xy=torch.randn(1,8,2),route_mask=torch.ones(1,8,dtype=torch.bool))
    before=tensor_hash(tensors)
    history=[dict(frame=i) for i in range(10)]
    derived={i:dict(ax=.1*i,ay=.2*i) for i in range(10)}
    changed=transform_history(tensors,history,derived)
    assert tensor_hash(tensors)==before
    assert tensor_hash(changed)!=before
    assert changed['ego_history'].data_ptr()!=tensors['ego_history'].data_ptr()
    assert torch.allclose(changed['ego_history'][0,:,3],torch.arange(10)*.1)
    assert torch.allclose(changed['ego_history'][0,:,4],torch.arange(10)*.2)
    for name in tensors:
        if name!='ego_history':assert torch.equal(tensors[name],changed[name])
