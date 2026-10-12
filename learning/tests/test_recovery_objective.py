"""HH_260906 - Test detached ranking targets and conditional teacher retention."""
import torch
from portable_e2e.recovery_objective import recovery_terms


def tensors():
    xy=torch.zeros(2,3,4,2,requires_grad=True)
    speed=torch.zeros(2,3,4,requires_grad=True)
    logits=torch.zeros(2,3,requires_grad=True)
    target=torch.zeros(2,4,2);ts=torch.zeros(2,4);valid=torch.ones(2,4,dtype=torch.bool)
    teacher=(xy.detach().clone(),speed.detach().clone(),logits.detach().clone())
    return xy,speed,logits,teacher,target,ts,valid


def test_rank_has_no_direct_geometry_gradients():
    a=tensors();rank,_,_=recovery_terms(*a);rank.backward()
    assert a[0].grad is None and a[1].grad is None and a[2].grad is not None


def test_bad_teacher_excluded():
    a=list(tensors());a[3]=(a[3][0]+100,a[3][1],a[3][2])
    _,preserve,count=recovery_terms(*a)
    assert count==0 and preserve==0


def test_good_teacher_preservation_reaches_geometry():
    a=list(tensors());a[3]=(a[3][0]+.1,a[3][1],a[3][2])
    _,preserve,count=recovery_terms(*a);preserve.backward()
    assert count==2 and a[0].grad.abs().sum()>0
