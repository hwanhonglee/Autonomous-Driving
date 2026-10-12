"""HH_261001 - Thin reference-only photometric study and isolated augmentation RNG."""

import hashlib
import json
from pathlib import Path
import numpy as np
try:
    from . import map_appearance_matched_contract_20261001 as base
except ImportError:
    import map_appearance_matched_contract_20261001 as base

NAME = "map_photometric_matched_20261001_v1"
REFERENCE_NAME = base.NAME
REFERENCE_MANIFEST_SHA = "32ed912505e0b215bfc54e563fcdba7553aa25412ba22d90b45bfa00339e93c8"
REFERENCE_REGISTRY_SHA = "456b8b31ef2a3426d4c732236e7c549792ebe9d9c1cb246e0b055496441718af"
REFERENCE_SOURCE_SHA = "0b6d5b4b394b7223f96701f185054656fb2aca694889b6b336e5f760514dda86"
PREDECESSOR_FIT_SHA = "463741ec65e9fce501bcb06dbaecc6c02d5b46bd566d6510ca0a7050b54e5253"
POLICY_SOURCE_SHA = "4de567323d3964ec877da831ec2acb08e445fe77c9da659f052edcf52ca53484"
PHOTO_SEED = base.SEED+2
ARMS = ("control","photometric")


def policy():
    return {"source_sha256":POLICY_SOURCE_SHA,"function":"photometric",
        "gate_probability":.5,"grayscale_blend_uniform":[0.,1.],"brightness_uniform":[.35,1.],
        "formula":"round_even(clip(scale*((1-blend)*RGB+blend*(.299R+.587G+.114B)),0,255))",
        "apply_after":"shared aligned crop","eligible_ids":"729..1208: paired TRUE480 only",
        "original729_transformed":False,"target_transformed":False,"seed":PHOTO_SEED,
        "rng":"separate PCG64 stream; original729 and control consume no photo draws",
        "architecture_or_exposure_mixture_changed":False,"new_capture":False,
        "ctrack_train_eligible":False,"retuned_after_scores":False}


class Trace:
    def __init__(self):
        self.sha = hashlib.sha256()
        self.eligible = 0
        self.applied = 0

    def add(self,index,applied):
        self.eligible += 1
        self.applied += int(applied)
        self.sha.update(np.asarray([index,int(applied)],dtype=np.int64).tobytes())

    def receipt(self,rng):
        return {"eligible_calls":self.eligible,"applied_calls":self.applied,
            "sample_gate_trace_sha256":self.sha.hexdigest(),"final_photo_rng_state":rng.bit_generator.state}


def expected_trace(indices,photometric):
    rng = np.random.default_rng(PHOTO_SEED)
    trace = Trace()
    tiny = np.zeros((1,1,3),np.uint8)
    for index in indices.ravel():
        if int(index) >= base.ORIGINAL_COUNT:
            _,applied = photometric(tiny,rng)
            trace.add(int(index),applied)
    return trace.receipt(rng)


def augment_batch(rgb,indices,arm,rng,photometric,trace):
    """HH_261001 - No label argument; only candidate TRUE480 crops can change."""
    base.require(arm in ARMS and rgb.dtype == np.uint8 and rgb.shape == (len(indices),360,640,3),
                 "photometric batch ABI differs")
    output = rgb.copy()
    if arm == "photometric":
        for row,index in enumerate(indices):
            if int(index) >= base.ORIGINAL_COUNT:
                output[row],applied = photometric(rgb[row],rng)
                trace.add(int(index),applied)
    return output


def validate_study(manifest,registry,reference):
    base.require(manifest.get("schema") == "portable_e2e.map_photometric_reference_data.v1"
        and manifest.get("study_id") == NAME and registry.get("schema") == "portable_e2e.study_train_registration.v1"
        and registry.get("study_id") == NAME and registry.get("study_train_authorized") is True,"new study-only authorization missing")
    binding = {"study_id":REFERENCE_NAME,"manifest_sha256":REFERENCE_MANIFEST_SHA,
        "registry_sha256":REFERENCE_REGISTRY_SHA,"source_bundle_sha256":REFERENCE_SOURCE_SHA}
    base.require(manifest.get("reference_package") == registry.get("reference_package") == binding,
                 "immutable reference package binding differs")
    base.require(manifest.get("photometric_policy") == registry.get("photometric_policy") == policy(),"archived photo policy changed")
    for item in (manifest,registry):
        base.require(item.get("counts") == {"original":729,"paired_true":480,"per_arm":1209}
            and item.get("arms") == {"control":"original729+TRUE480; no photo","photometric":"same pool; archived photo on TRUE480 only"}
            and item.get("parent_weights_sha256") == base.PARENT_SHA and item.get("fresh_fits") == 2
            and item.get("ctrack_train_eligible") is False and item.get("broad_dataset_admitted") is False,
            "same-data, fresh-fit or admission boundary differs")
    base.require(registry.get("source_diagnostic_receipts_preserved") is True
        and registry.get("source_study_registry_modified") is False and registry.get("dev_or_test_used_for_fit") is False,
        "historical receipts or optimization boundary changed")
    expected_optimization = {"steps_per_arm":base.STEPS,"batch":base.BATCH,"seed":base.SEED,
        "learning_rate":base.LR,"weight_decay":base.WEIGHT_DECAY,"optimizer":"fresh AdamW in both arms"}
    base.require(registry.get("optimization") == expected_optimization,"fixed optimization differs")
    for key in ("files","schedule_contract","common_class_pixels","common_class_weights","paired_samples_sha256"):
        base.require(manifest.get("reference_"+key) == reference[key],"referenced arrays, labels or exposure changed: "+key)
    base.require(set(manifest.get("local_data_files",{})) == set(),"large array copies or alternate local data forbidden")
    base.require(manifest.get("preview_review_sha256") == registry.get("preview_review_sha256"),"TRAIN-only preview review differs")


def study_binding(study,expected_manifest_sha,expected_registry_sha,reference_root):
    base.require(study.is_dir() and study.resolve() == study and reference_root.is_dir()
        and reference_root.resolve() == reference_root,"canonical regular data/reference directories required")
    base.require(base.digest(study/"manifest.json") == expected_manifest_sha
        and base.digest(study/"study_train_registry.json") == expected_registry_sha,"photometric study metadata changed")
    base.require(base.digest(reference_root/"manifest.json") == REFERENCE_MANIFEST_SHA
        and base.digest(reference_root/"study_train_registry.json") == REFERENCE_REGISTRY_SHA
        and base.digest(reference_root/"source/bundle.json") == REFERENCE_SOURCE_SHA,"reference metadata changed")
    manifest,registry,reference = (base.read_json(path) for path in
        (study/"manifest.json",study/"study_train_registry.json",reference_root/"manifest.json"))
    base.validate_manifest(reference,base.read_json(reference_root/"study_train_registry.json"))
    validate_study(manifest,registry,reference)
    base.require(manifest["study_registry_sha256"] == expected_registry_sha
        and base.digest(study/"preview_review.json") == manifest["preview_review_sha256"],"study review/registry binding changed")
    return manifest,registry,reference
