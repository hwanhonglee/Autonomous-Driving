"""HH_260906 - Verify auxiliary gradient scope and paired TRAIN augmentation."""
import numpy as np
import pytest
import torch
from scripts.e2e.dual_recovery_objectives import near_horizon_loss,paired_crop


def test_near_loss_only_updates_original_oracle_near_prefix():
    xy=torch.ones(2,3,64,2,requires_grad=True);speed=torch.ones(2,3,64,requires_grad=True)
    valid=torch.ones(2,64,dtype=torch.bool);valid[1,10:]=False
    loss=near_horizon_loss(xy,speed,torch.zeros(2,64,2),torch.zeros(2,64),valid,torch.tensor([1,2]))
    loss.backward()
    assert loss.item()==pytest.approx(1.1)
    assert xy.grad[0,1,:20].abs().sum()>0
    assert xy.grad[0,0].abs().sum()==0 and xy.grad[0,2].abs().sum()==0
    assert xy.grad[:,:,20:].abs().sum()==0 and xy.grad[1,:,10:].abs().sum()==0
    assert speed.grad[:,:,20:].abs().sum()==0


def test_empty_near_mask_rejected():
    with pytest.raises(ValueError,match='valid near-horizon'):
        near_horizon_loss(torch.zeros(1,1,20,2),torch.zeros(1,1,20),torch.zeros(1,20,2),
                          torch.zeros(1,20),torch.zeros(1,20,dtype=torch.bool),torch.tensor([0]))


def test_crop_repeatable_labels_preserved_and_no_input_mutation():
    y=np.zeros((36,64),dtype=np.uint8);y[10:25,20:40]=5
    x=np.repeat((y*40)[...,None],3,axis=2)
    before=x.copy()
    a,b=paired_crop(x,y,np.random.default_rng(2))
    c,d=paired_crop(x,y,np.random.default_rng(2))
    assert np.array_equal(a,c) and np.array_equal(b,d)
    assert np.array_equal(x,before) and set(np.unique(b))<={0,5}
    assert a.shape==x.shape and b.shape==y.shape
    assert np.mean((a[...,0]>100)==(b==5))>.95


def test_dev_crop_disabled_is_exact_copy():
    x=np.zeros((18,32,3),dtype=np.uint8);y=np.zeros((18,32),dtype=np.uint8)
    a,b=paired_crop(x,y,np.random.default_rng(0),enabled=False)
    assert np.array_equal(a,x) and np.array_equal(b,y)
    assert not np.shares_memory(a,x) and not np.shares_memory(b,y)
