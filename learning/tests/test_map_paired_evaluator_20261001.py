"""HH_261001 - Fixed pair policy and native full-FOV rendering preserve both arms."""
import numpy as np
from scripts.e2e import evaluate_map_paired_rgb_20261001 as evaluate


def test_all_frames_fixed_weights_and_no_fit():
    policy = evaluate.policy()
    assert policy["frames_per_route"] == policy["frames_per_model_condition"] == 240
    assert policy["rgb_conditions"] == ["false","true"] and policy["models"] == ["A","B"]
    assert policy["fit_steps"] == 0 and policy["performance_gates"] is None
    assert not policy["train_eligible"] and not policy["ctrack_train_eligible"]


def test_both_rgb_panels_remain_native_and_shared_gt_identical():
    rgb = {"false":np.full((360,640,3),17,dtype=np.uint8),
           "true":np.full((360,640,3),211,dtype=np.uint8)}
    target = np.ones((360,640),dtype=np.uint8)
    predictions = {key:target.copy() for key in ("A_false","A_true","B_false","B_true")}
    image = np.asarray(evaluate.panels("town01_p01",119,rgb,target,predictions))
    assert image.shape == (1640,1280,3)
    assert np.array_equal(image[57:417,:640],rgb["false"])
    assert np.array_equal(image[57:417,640:],rgb["true"])
    assert np.array_equal(image[442:802,:640],image[442:802,640:])
