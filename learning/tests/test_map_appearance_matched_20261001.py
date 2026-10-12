"""HH_261001 - Appearance-only fitting contracts, without simulator or GPU access."""
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from scripts.e2e import map_appearance_matched_contract_20261001 as contract
from scripts.e2e import prepare_map_appearance_matched_20261001 as builder
from scripts.e2e import train_map_appearance_matched_20261001 as trainer


@pytest.fixture(scope="module")
def schedule():
    return contract.schedule()


def test_exact_complete_epochs_and_identical_schedule(schedule):
    ids,crops = schedule
    receipt = contract.validate_schedule(ids,crops)
    assert receipt["full_epochs"] == 66 and receipt["last_partial_epoch"] == 206
    assert set(receipt["sample_exposures"]) == {66,67}
    assert receipt["original_exposures"]+receipt["paired_exposures"] == 80000
    assert len(receipt["sample_exposures"]) == 1209


def test_schedule_rejects_missing_replay_and_altered_crop(schedule):
    ids,crops = schedule
    wrong = ids.copy(); wrong[0,0] = wrong[0,1]
    with pytest.raises(ValueError,match="omitted or duplicated"):
        contract.validate_schedule(wrong,crops)
    wrong_crop = crops.copy(); wrong_crop[0,0] = (0,0,360,639)
    with pytest.raises(ValueError,match="frozen seed"):
        contract.validate_schedule(ids,wrong_crop)


def test_only_new_pair_rgb_changes_and_targets_crops_match(monkeypatch):
    monkeypatch.setattr(contract,"ORIGINAL_COUNT",1)
    original = np.full((1,360,640,3),25,np.uint8)
    false = np.full_like(original,50); true = np.full_like(original,175)
    target = np.indices((360,640))[1].astype(np.uint8)[None]%7
    arrays = (original,target,false,true,target)
    ids = np.array([0,1]); boxes = np.array([[0,0,360,640],[20,40,270,480]])
    a,ya = trainer.batch_arrays(arrays,ids,boxes,"control")
    b,yb = trainer.batch_arrays(arrays,ids,boxes,"appearance")
    assert np.array_equal(a[0],b[0]) and not np.array_equal(a[1],b[1])
    assert np.array_equal(ya,yb) and set(np.unique(ya)) <= set(range(7))
    with pytest.raises(ValueError,match="unknown"):
        trainer.batch_arrays(arrays,ids,boxes,"third")


def test_common_weights_finite_and_rgb_independent():
    values = contract.common_weights([900,100,0,30,0,0,0])
    assert values.dtype == np.float32 and np.isfinite(values).all()
    assert np.isclose(values.mean(),1)


def test_manifest_rejects_widened_roles_counts_optimizer_and_reordered_pairs():
    files = {name:{"sha256":sha} for name,sha in (("original_train729.npz",contract.ORIGINAL_SHA),
        ("parent_weights.npz",contract.PARENT_SHA),("paired_train480.npz","paired"),("training_schedule.npz","schedule"))}
    manifest = {"schema":"portable_e2e.map_appearance_matched_data.v1","study_id":contract.NAME,
        "labels":list(contract.LABELS),"counts":{"original":729,"paired":480,"per_arm":1209},
        "arms":{"control":"original729+paired_false480","appearance":"original729+paired_true480"},
        "ctrack_train_eligible":False,"broad_dataset_admitted":False,"parent_weights_sha256":contract.PARENT_SHA,
        "files":files,"schedule_contract":{},"paired_sample_ids":[f"{route}:frames/{index:04d}"
            for route in ("town01_p01","town02_p00") for index in range(240)]}
    registry = {"schema":"portable_e2e.study_train_registration.v1","study_id":contract.NAME,
        "study_train_authorized":True,"route_sha256":contract.ROUTES,"source_diagnostic_receipts_preserved":True,
        "ctrack_train_eligible":False,"broad_registry_modified":False,"source_diagnostic_train_eligible_remains_false":True,
        "dev_or_test_used_for_fit":False,"counts":manifest["counts"],"parent_weights_sha256":contract.PARENT_SHA,
        "data_files":files,"optimization":{"steps_per_arm":10000,"batch":8,"seed":contract.SEED,
            "learning_rate":3e-5,"weight_decay":1e-4,"optimizer":"fresh AdamW in each arm","common_schedule":{}}}
    contract.validate_manifest(manifest,registry)
    for change in ({"ctrack_train_eligible":True},{"counts":{"original":0,"paired":480,"per_arm":480}},
            {"paired_sample_ids":manifest["paired_sample_ids"][::-1]},
            {"arms":{"control":"false","appearance":"true_plus_architecture"}}):
        with pytest.raises(ValueError):
            contract.validate_manifest({**manifest,**change},registry)
    for change in ({"broad_registry_modified":True},{"source_diagnostic_train_eligible_remains_false":False},
            {"dev_or_test_used_for_fit":True},{"optimization":{**registry["optimization"],"steps_per_arm":10001}}):
        with pytest.raises(ValueError):
            contract.validate_manifest(manifest,{**registry,**change})


def test_review_gate_requires_actual_same_route_qa_and_fixed_images():
    receipt = {"route":"town01_p01","source":"/capture","qa_sha256":"abc",
        "policy":{"visual_indices":[0,119,239],"rgb_conditions":["false","true"]},
        "review_note":"Root inspected all six native PNGs and retained glare and blur."}
    builder.review_contract(receipt,"town01_p01",Path("/capture"),"abc")
    for change in ({"review_note":""},{"qa_sha256":"other"},{"route":"town02_p02"},
            {"policy":{"visual_indices":[0,239],"rgb_conditions":["true"]}}):
        with pytest.raises(ValueError,match="review receipt"):
            builder.review_contract({**receipt,**change},"town01_p01",Path("/capture"),"abc")


def test_metrics_keep_absent_targets_null_and_false_positives():
    confusion = np.zeros((7,7),np.int64); confusion[0,0] = 8; confusion[0,5] = 2
    metrics = trainer.confusion_metrics(confusion)
    assert metrics["iou"]["traffic_sign"] is None
    assert metrics["false_positive_pixels"]["traffic_sign"] == 2
    assert metrics["iou"]["other"] == .8


def test_safe_weights_reject_wrong_hash_keys_dtype_and_nonfinite(tmp_path):
    import torch
    model = torch.nn.Linear(3,2)
    valid = {key:value.detach().numpy() for key,value in model.state_dict().items()}
    for index,alter in enumerate(({}, {"extra":np.zeros(1,np.float32)},
            {"weight":valid["weight"].astype(np.float64)},
            {"weight":np.full((2,3),np.nan,np.float32)})):
        path = tmp_path/f"{index}.npz"
        np.savez(path,**{**valid,**alter})
        if index == 0:
            trainer.load_weights(model,path,contract.digest(path))
            assert all(p.requires_grad for p in model.parameters())
            with pytest.raises(ValueError,match="bytes"):
                trainer.load_weights(model,path,"0"*64)
        else:
            with pytest.raises(ValueError,match="keys|ABI"):
                trainer.load_weights(model,path,contract.digest(path))


def test_both_endpoints_required_before_eval_file_open(monkeypatch):
    monkeypatch.setattr(trainer,"EVAL_DATA",Path("/nonexistent/never-open"))
    with pytest.raises(ValueError,match="both final endpoints"):
        trainer.evaluate_endpoints({"control":{}},None,None)


def test_environment_rejects_local_host_before_gpu():
    with pytest.raises(ValueError,match="approved personal host"):
        trainer.environment_contract()


def test_source_guard_rejects_changed_architecture(tmp_path):
    names = {"train_map_appearance_matched_20261001.py","map_appearance_matched_contract_20261001.py",
        "map_perception.py","map_context_recovery.py","train_map_photometric_pilot.py","plan.md"}
    for name in names:
        (tmp_path/name).write_text("changed")
    bundle = {"schema":"portable_e2e.map_appearance_source.v1","study_id":contract.NAME,
        "files":{name:contract.digest(tmp_path/name) for name in names},
        "data_manifest_sha256":"data","study_registry_sha256":"registry"}
    (tmp_path/"bundle.json").write_text(json.dumps(bundle))
    with pytest.raises(ValueError,match="original architecture/guard"):
        trainer.source_contract(tmp_path,contract.digest(tmp_path/"bundle.json"),"data","registry")


def test_training_source_has_fixed_budget_and_no_pickle_load():
    source = Path(trainer.__file__).read_text()
    assert "torch.load(" not in source and "gpu0_lease()" in source
    assert "PAIR_BUDGET_SECONDS = 1800" in source
    assert source.index('guard.write_new(OUTPUT/"fit_complete.json"') < source.index("evaluated = evaluate_endpoints(")
    assert contract.STEPS == 10000 and contract.BATCH == 8 and contract.LR == 3e-5
