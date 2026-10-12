"""HH_261001 - Mapping, absent-class semantics, no-fit policy and safe NPZ ABI."""
import hashlib

import numpy as np
import pytest

from scripts.e2e import evaluate_ctrack_fixed_maps_20261001 as evaluate


def test_raw_tags_keep_existing_seven_class_order():
    raw = np.zeros((360,640,4),dtype=np.uint8)
    tags = (0,1,2,24,7,8,6,255)
    raw[0,:len(tags),2] = tags
    assert evaluate.targets(raw)[0,:len(tags)].tolist() == [0,1,2,3,4,5,6,0]


def test_absent_gt_iou_null_preserves_false_positives():
    target = np.array([[1,1,3,0]],dtype=np.uint8)
    prediction = np.array([[1,6,3,0]],dtype=np.uint8)
    result = evaluate.metrics(evaluate.confusion(target,prediction))
    assert result["class_metrics"]["pole"]["iou"] is None
    assert result["class_metrics"]["pole"]["false_positive_pixels"] == 1
    assert result["class_metrics"]["road"]["iou"] == .5
    assert result["class_metrics"]["lane_marking"]["iou"] == 1


def test_preregistered_policy_has_no_sampling_or_fit():
    policy = evaluate.policy()
    assert policy["frames_per_route"] == 240 and policy["fit_steps"] == 0
    assert policy["visual_indices"] == [0,119,239]
    assert policy["device"] == "cpu" and policy["torch_threads"] == 2
    assert policy["performance_pass_thresholds"] is None
    assert not policy["train_eligible"] and not policy["model_promotion"]


def test_safe_npz_rejects_dtype_shape_and_nonfinite(tmp_path):
    torch = pytest.importorskip("torch")
    for index,value in enumerate((np.zeros((1,1),dtype=np.float64),
            np.zeros((2,1),dtype=np.float32),np.full((1,1),np.nan,dtype=np.float32))):
        model = torch.nn.Linear(1,1,bias=False)
        path = tmp_path/f"bad{index}.npz"
        np.savez(path,weight=value)
        sha = hashlib.sha256(path.read_bytes()).hexdigest()
        with pytest.raises(ValueError,match="ABI"):
            evaluate.safe_npz_state(model,path,sha,torch)


def test_safe_npz_valid_weights_are_frozen_without_forward(tmp_path):
    torch = pytest.importorskip("torch")
    path = tmp_path/"safe.npz"
    np.savez(path,weight=np.ones((1,1),dtype=np.float32))
    sha = hashlib.sha256(path.read_bytes()).hexdigest()
    model = evaluate.safe_npz_state(torch.nn.Linear(1,1,bias=False),path,sha,torch)
    assert not model.training and not model.weight.requires_grad
