"""HH_261001 - Endpoint identity, fixed panels and independent absent-class recount."""
import copy
import numpy as np
import pytest
from scripts.e2e import evaluate_map_appearance_endpoints_20261001 as evaluate
from scripts.e2e import verify_map_appearance_endpoints_20261001 as verify


def test_policy_exact_exposed_sets_and_no_selection():
    policy = evaluate.policy()
    assert policy["scenarios"] == ["ctrack_left","ctrack_straight","train_town01_p01","train_town02_p00"]
    assert policy["frames_per_scenario"] == 240 and policy["visual_indices"] == [0,119,239]
    assert policy["performance_gates"] is None and not policy["model_promotion"]
    assert not policy["ctrack_train_eligible"] and policy["fit_steps_during_evaluation"] == 0
    assert "resubstitution" in policy["paired_role"]
    assert "original live-map equality unavailable" in " ".join(policy["limitations"])


def test_independent_label_mapping_unknown_tags_and_absent_iou():
    semantic = np.zeros((360,640,4),np.uint8)
    semantic[0,:8,2] = [1,2,24,7,8,6,255,0]
    target = verify.independent_target(semantic)
    assert target[0,:8].tolist() == [1,2,3,4,5,6,0,0]
    assert np.array_equal(target,evaluate.frozen.targets(semantic))
    truth = np.zeros((2,2),np.uint8); pred = np.array([[0,5],[0,5]],np.uint8)
    matrix = verify.independent_confusion(truth,pred)
    assert np.array_equal(matrix,evaluate.frozen.confusion(truth,pred))
    metrics = evaluate.frozen.metrics(matrix)
    verify.check_metrics(matrix,metrics)
    assert metrics["class_metrics"]["traffic_sign"]["iou"] is None
    assert metrics["class_metrics"]["traffic_sign"]["false_positive_pixels"] == 2
    metrics["class_metrics"]["traffic_sign"]["iou"] = 0.
    with pytest.raises(ValueError,match="absent-class"):
        verify.check_metrics(matrix,metrics)


@pytest.mark.parametrize("conditions",[("true",),("false","true")])
def test_render_native_all_pixels_and_both_same_architecture_arms(conditions):
    rng = np.random.default_rng(55)
    rgb = {condition:rng.integers(0,256,(360,640,3),np.uint8) for condition in conditions}
    target = rng.integers(0,7,(360,640),np.uint8)
    pred = {arm+"_"+condition:rng.integers(0,7,(360,640),np.uint8) for condition in conditions for arm in evaluate.ARMS}
    image = np.asarray(evaluate.render("test",119,rgb,target,pred,conditions))
    assert image.shape == (1640 if len(conditions)==2 else 860,1280,3)
    for x,y,pixels in evaluate.panel_layout(rgb,target,pred,conditions):
        assert np.array_equal(image[y:y+360,x:x+640],pixels)


def metadata_fixture():
    manifest = evaluate.package_binding()
    info = {"steps":10000,"parameter_count":245831,"initial_state_sha256":evaluate.INITIAL_STATE_SHA,
        "optimizer_initial_state_entries":0,"sample_sequence_sha256":manifest["schedule_contract"]["indices_sha256"],
        "crop_sequence_sha256":manifest["schedule_contract"]["crops_sha256"]}
    common = {"study_id":evaluate.contract.NAME,"data_manifest_sha256":evaluate.DATA_SHA,
        "source_bundle_sha256":evaluate.SOURCE_SHA,"study_registry_sha256":evaluate.REGISTRY_SHA,
        "parent_weights_sha256":evaluate.contract.PARENT_SHA,"steps_per_arm":10000,"batch":8,
        "common_class_weights":manifest["common_class_weights"],"schedule_contract":manifest["schedule_contract"],
        "ctrack_train_eligible":False,"model_promotion":False}
    complete = {"status":"BOTH_FIXED_ENDPOINTS_SAVED_BEFORE_DEV","arms":{arm:copy.deepcopy(info) for arm in evaluate.ARMS}}
    summary = {**common,"status":"MATCHED_PAIR_COMPLETE_DIAGNOSTIC_NOT_PROMOTED","arms":complete["arms"],
        "dev_used_during_optimization":False,"source_and_data_unchanged":True}
    return summary,common,complete,manifest


def test_endpoint_metadata_requires_both_fixed_matched_10000step_models():
    original = metadata_fixture()
    evaluate.validate_endpoint_metadata(*original)
    for key,value in (("steps",9999),("parameter_count",130503),("initial_state_sha256","other"),
            ("sample_sequence_sha256","other"),("crop_sequence_sha256","other"),("optimizer_initial_state_entries",1)):
        summary,started,complete,manifest = copy.deepcopy(original)
        complete["arms"]["appearance"][key] = value
        with pytest.raises(ValueError,match="matched endpoint"):
            evaluate.validate_endpoint_metadata(summary,started,complete,manifest)


def test_endpoint_metadata_rejects_dev_tuning_or_unbound_study():
    original = metadata_fixture()
    for key,value in (("dev_used_during_optimization",True),("source_and_data_unchanged",False),
            ("data_manifest_sha256","unregistered"),("model_promotion",True)):
        summary,started,complete,manifest = copy.deepcopy(original)
        summary[key] = value
        with pytest.raises(ValueError):
            evaluate.validate_endpoint_metadata(summary,started,complete,manifest)
