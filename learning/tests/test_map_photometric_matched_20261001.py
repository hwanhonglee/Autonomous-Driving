"""HH_261001 - Pure-photo factor, immutable replay and independent RNG safeguards."""
import copy
from pathlib import Path
import numpy as np
import pytest
from scripts.e2e import map_photometric_matched_contract_20261001 as photo
from scripts.e2e import prepare_map_photometric_matched_20261001 as builder
from scripts.e2e import train_map_photometric_matched_20261001 as trainer
from scripts.e2e import train_map_appearance_matched_20261001 as predecessor


@pytest.fixture(scope="module")
def guard():
    return builder.load_policy()


def test_archive_policy_probability_ranges_and_original_only_exclusion():
    policy = photo.policy()
    assert policy["gate_probability"] == .5 and policy["grayscale_blend_uniform"] == [0.,1.]
    assert policy["brightness_uniform"] == [.35,1.] and policy["seed"] == 20261003
    assert not policy["original729_transformed"] and not policy["target_transformed"]
    assert not policy["ctrack_train_eligible"] and not policy["retuned_after_scores"]


def test_both_new_arms_exactly_predecessor_true_inputs(monkeypatch):
    monkeypatch.setattr(photo.base,"ORIGINAL_COUNT",1)
    original = np.full((1,360,640,3),25,np.uint8)
    false = np.full_like(original,50); true = np.full_like(original,175)
    target = np.indices((360,640))[1].astype(np.uint8)[None]%7
    arrays = (original,target,false,true,target)
    ids = np.array([0,1]); boxes = np.array([[0,0,360,640],[20,40,270,480]])
    expected,y = predecessor.batch_arrays(arrays,ids,boxes,"appearance")
    for arm in photo.ARMS:
        actual,label = trainer.batch_arrays(arrays,ids,boxes,arm)
        assert np.array_equal(actual,expected) and np.array_equal(label,y)
    assert expected[1,0,0,0] == 175


def test_exact_archived_transform_only_true_candidate_rng_isolated(guard):
    rng = np.random.default_rng(321)
    rgb = rng.integers(0,256,(3,360,640,3),dtype=np.uint8)
    original = rgb.copy(); indices = np.array([0,729,728])
    candidate_rng = np.random.default_rng(photo.PHOTO_SEED)
    expected_rng = np.random.default_rng(photo.PHOTO_SEED)
    expected,applied = guard.photometric(rgb[1],expected_rng)
    trace = photo.Trace()
    actual = photo.augment_batch(rgb,indices,"photometric",candidate_rng,guard.photometric,trace)
    assert np.array_equal(actual[1],expected)
    assert np.array_equal(actual[[0,2]],rgb[[0,2]]) and np.array_equal(rgb,original)
    assert candidate_rng.bit_generator.state == expected_rng.bit_generator.state
    assert trace.eligible == 1 and trace.applied == int(applied)


def test_reference_no_photo_draws_or_image_changes(guard):
    rgb = np.full((2,360,640,3),175,np.uint8)
    rng = np.random.default_rng(photo.PHOTO_SEED); before = copy.deepcopy(rng.bit_generator.state)
    trace = photo.Trace()
    value = photo.augment_batch(rgb,np.array([729,1208]),"control",rng,guard.photometric,trace)
    assert np.array_equal(value,rgb) and not np.shares_memory(value,rgb)
    assert rng.bit_generator.state == before and trace.eligible == trace.applied == 0


def test_rng_trace_replays_full_serialized_exposure_schedule(guard):
    with np.load(builder.REFERENCE/"training_schedule.npz",allow_pickle=False) as data:
        indices,crops = data["indices"],data["crops"]
    before_ids,before_crops = indices.copy(),crops.copy()
    result = photo.expected_trace(indices,guard.photometric)
    assert result == photo.expected_trace(indices,guard.photometric)
    assert result["eligible_calls"] == 31769 and 0 < result["applied_calls"] < 31769
    assert np.array_equal(indices,before_ids) and np.array_equal(crops,before_crops)


def study_fixture():
    ref = photo.base.read_json(builder.REFERENCE/"manifest.json")
    common = {"study_id":photo.NAME,"reference_package":{"study_id":photo.REFERENCE_NAME,
        "manifest_sha256":photo.REFERENCE_MANIFEST_SHA,"registry_sha256":photo.REFERENCE_REGISTRY_SHA,
        "source_bundle_sha256":photo.REFERENCE_SOURCE_SHA},"photometric_policy":photo.policy(),
        "counts":{"original":729,"paired_true":480,"per_arm":1209},
        "arms":{"control":"original729+TRUE480; no photo","photometric":"same pool; archived photo on TRUE480 only"},
        "parent_weights_sha256":photo.base.PARENT_SHA,"fresh_fits":2,"ctrack_train_eligible":False,
        "broad_dataset_admitted":False,"preview_review_sha256":"review"}
    registry = {"schema":"portable_e2e.study_train_registration.v1",**copy.deepcopy(common),
        "study_train_authorized":True,"source_diagnostic_receipts_preserved":True,
        "source_study_registry_modified":False,"dev_or_test_used_for_fit":False,
        "optimization":{"steps_per_arm":10000,"batch":8,"seed":photo.base.SEED,"learning_rate":3e-5,
            "weight_decay":1e-4,"optimizer":"fresh AdamW in both arms"}}
    manifest = {"schema":"portable_e2e.map_photometric_reference_data.v1",**common,"local_data_files":{},
        **{"reference_"+key:ref[key] for key in ("files","schedule_contract","common_class_pixels","common_class_weights","paired_samples_sha256")}}
    return manifest,registry,ref


def test_study_guard_rejects_widening_policy_copy_or_unequal_optimization():
    original = study_fixture(); photo.validate_study(*original)
    for target,key,value in ((0,"fresh_fits",1),(0,"local_data_files",{"copied.npz":"x"}),
            (0,"ctrack_train_eligible",True),(1,"source_study_registry_modified",True),
            (1,"dev_or_test_used_for_fit",True),(1,"optimization",{"steps_per_arm":20000}),
            (0,"photometric_policy",{**photo.policy(),"gate_probability":.7})):
        arguments = copy.deepcopy(original); arguments[target][key] = value
        with pytest.raises(ValueError):
            photo.validate_study(*arguments)


def test_preview_conditional_is_exact_declared_archived_policy(guard):
    image = np.full((2,2,3),[100,180,240],np.uint8)
    result,applied = guard.photometric(image,builder.ConditionalPreviewRng())
    rgb = image.astype(np.float32); gray = .299*rgb[...,:1]+.587*rgb[...,1:2]+.114*rgb[...,2:3]
    expected = np.rint(np.clip(.35*(.5*rgb+.5*gray),0,255)).astype(np.uint8)
    assert applied and np.array_equal(result,expected)


def test_tensor_digest_matches_actual_guard_torch_digest(tmp_path,guard):
    import torch
    model = torch.nn.Linear(3,2)
    path = tmp_path/"weights.npz"
    np.savez_compressed(path,**{key:value.detach().numpy() for key,value in model.state_dict().items()})
    assert builder.tensor_digest(path) == guard.state_digest(model)


def test_source_requires_two_fresh_fits_and_reference_reproduction():
    source = Path(trainer.__file__).read_text()
    assert 'for arm in ("control","photometric"):' in source and 'range(contract.STEPS)' in source
    assert 'study["predecessor_appearance_state_sha256"]' in source
    assert 'torch.load(' not in source and 'PAIR_BUDGET_SECONDS = 1800' in source
    assert source.index('guard.write_new(OUTPUT/"fit_complete.json"') < source.index('evaluated = evaluate_endpoints(')
    assert source.index('photo.expected_trace(indices,guard.photometric)') < source.index('if args.validate_only:')


def test_environment_and_incomplete_endpoints_fail_before_cuda_work():
    with pytest.raises(ValueError,match="approved personal host"):
        trainer.environment_contract()
    with pytest.raises(ValueError,match="both final endpoints"):
        trainer.evaluate_endpoints({"control":{}},None,None)


def test_fixed_train_preview_pixels_and_archived_rng_replay(guard):
    import hashlib
    from PIL import Image
    path = builder.PREVIEW/"receipt.json"
    if not path.exists():
        pytest.skip("deterministic TRAIN previews not generated yet")
    receipt = photo.base.read_json(path)
    assert [(row["route"],row["index"]) for row in receipt["samples"]] == [
        (route,index) for route in ("town01_p01","town02_p00") for index in (0,119,239)]
    with np.load(builder.REFERENCE/"paired_train480.npz",allow_pickle=False) as data:
        rgb,target = data["rgb_true"],data["target"]
    rng = np.random.default_rng(photo.PHOTO_SEED)
    colors = np.asarray(((30,30,40),(67,92,123),(141,88,147),(246,222,90),
        (246,100,70),(60,220,180),(179,190,199)),np.uint8)
    for row in receipt["samples"]:
        original = rgb[row["pair_id"]]
        transformed,applied = guard.photometric(original,rng)
        low,_ = guard.photometric(original,builder.ConditionalPreviewRng())
        assert applied == row["seeded_applied"]
        assert hashlib.sha256(original.tobytes()).hexdigest() == row["rgb_sha256"]
        assert hashlib.sha256(target[row["pair_id"]].tobytes()).hexdigest() == row["target_sha256"]
        image = np.asarray(Image.open(builder.PREVIEW/row["png"]))
        assert image.shape == (860,1280,3)
        for x,y,pixels in ((0,57,original),(640,57,transformed),(0,442,low),
                (640,442,colors[target[row["pair_id"]]])):
            assert np.array_equal(image[y:y+360,x:x+640],pixels)
