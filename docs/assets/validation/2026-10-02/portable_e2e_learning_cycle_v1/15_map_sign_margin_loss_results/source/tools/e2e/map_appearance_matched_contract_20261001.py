"""HH_261001 - Shared immutable data/sampling contract for appearance-only fitting."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

NAME = "map_appearance_matched_20261001_v1"
LABELS = ("other","road","sidewalk","lane_marking","traffic_light_fixture","traffic_sign","pole")
ORIGINAL_COUNT,PAIRED_COUNT,TOTAL_COUNT = 729,480,1209
STEPS,BATCH,SEED = 10000,8,20261001
LR,WEIGHT_DECAY = 3e-5,1e-4
ORIGINAL_SHA = "d2d7b8b306fcba9672a229b4b1222b208e8d17fe500caf43755687612025e8f3"
PARENT_SHA = "ff5d7d03a43b859d21a0a3f6b01d815054d89d78e850730121f326e5b79c023e"
PARENT_CHECKPOINT_SHA = "410358363d1c9cdc5551a9cdc3f37fa1bd336a1c3fd17ea64e9a71cd11d74abf"
ROUTES = {"town01_p01":"df68ac6046064faefe5d6a19425e0b038e94d2171878741f5c713a35ec0ce8de",
          "town02_p00":"520fc0236e32d0664bc125a15f45601e4ef020cf1e3f1db888332af0e188f08b"}
ARCHITECTURE_SHA = {
    "map_perception.py":"1b0076c8f9d34192ab5aedf6cdf96ce2c14458e2bb49986685a6811376b0750b",
    "map_context_recovery.py":"c8e2a8f0026852cc2b68ff653f78ed4666a1d2d2bad81997d1f5c16798fbc50d"}


def require(ok,message):
    if not ok:
        raise ValueError(message)


def digest(path):
    require(path.is_file() and not path.is_symlink(),f"regular file required: {path}")
    sha = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b""):
            sha.update(chunk)
    return sha.hexdigest()


def read_json(path):
    digest(path)
    return json.loads(path.read_text())


def schedule(seed=SEED):
    """HH_261001 - Complete shuffled epochs; same physical sample IDs in both arms."""
    sampler = np.random.default_rng(seed)
    required = STEPS*BATCH
    epochs = [sampler.permutation(TOTAL_COUNT).astype(np.int64)
              for _ in range((required+TOTAL_COUNT-1)//TOTAL_COUNT)]
    indices = np.concatenate(epochs)[:required].reshape(STEPS,BATCH)
    rng = np.random.default_rng(seed+1)
    crops = np.empty((required,4),dtype=np.int16)
    for index in range(required):
        if rng.random() >= .5:
            crops[index] = (0,0,360,640)
        else:
            scale = float(rng.uniform(.65,1.))
            height,width = max(1,int(360*scale)),max(1,int(640*scale))
            crops[index] = (int(rng.integers(360-height+1)),int(rng.integers(640-width+1)),height,width)
    return indices,crops.reshape(STEPS,BATCH,4)


def validate_schedule(indices,crops):
    require(indices.shape == (STEPS,BATCH) and indices.dtype == np.int64
        and int(indices.min()) >= 0 and int(indices.max()) < TOTAL_COUNT,"sample schedule ABI differs")
    require(crops.shape == (STEPS,BATCH,4) and crops.dtype == np.int16,"crop schedule ABI differs")
    flat = crops.reshape(-1,4)
    require(bool((flat[:,:2] >= 0).all()) and bool((flat[:,2:] > 0).all())
        and bool((flat[:,0]+flat[:,2] <= 360).all()) and bool((flat[:,1]+flat[:,3] <= 640).all()),
        "crop extends outside calibrated source")
    whole = indices.ravel()
    expected_ids = np.arange(TOTAL_COUNT)
    for start in range(0,len(whole)-TOTAL_COUNT+1,TOTAL_COUNT):
        require(np.array_equal(np.sort(whole[start:start+TOTAL_COUNT]),expected_ids),
                "full epoch omitted or duplicated a replay/paired sample")
    generated_ids,generated_crops = schedule()
    require(np.array_equal(indices,generated_ids) and np.array_equal(crops,generated_crops),
            "schedule differs from frozen seed/algorithm")
    return {"indices_sha256":hashlib.sha256(indices.tobytes()).hexdigest(),
        "crops_sha256":hashlib.sha256(crops.tobytes()).hexdigest(),
        "sample_exposures":np.bincount(whole,minlength=TOTAL_COUNT).tolist(),
        "original_exposures":int((whole<ORIGINAL_COUNT).sum()),
        "paired_exposures":int((whole>=ORIGINAL_COUNT).sum()),
        "full_epochs":len(whole)//TOTAL_COUNT,"last_partial_epoch":len(whole)%TOTAL_COUNT}


def apply_crop(rgb,target,box):
    from PIL import Image
    require(rgb.shape == (360,640,3) and target.shape == (360,640)
        and rgb.dtype == target.dtype == np.uint8,"native crop input ABI differs")
    top,left,height,width = (int(value) for value in box)
    require(top>=0 and left>=0 and height>0 and width>0 and top+height<=360 and left+width<=640,
            "invalid shared crop box")
    if (top,left,height,width) == (0,0,360,640):
        return rgb.copy(),target.copy()
    sampling = getattr(Image,"Resampling",Image)
    image = Image.fromarray(rgb[top:top+height,left:left+width]).resize((640,360),sampling.BILINEAR)
    label = Image.fromarray(target[top:top+height,left:left+width]).resize((640,360),sampling.NEAREST)
    return np.asarray(image).copy(),np.asarray(label).copy()


def common_weights(pixel_counts):
    counts = np.asarray(pixel_counts,dtype=np.float64)
    require(counts.shape == (7,) and bool(np.isfinite(counts).all())
        and bool((counts >= 0).all()) and counts.sum()>0,"common TRAIN counts invalid")
    weights = 1/np.sqrt(counts/counts.sum()+.001)
    return (weights/weights.mean()).astype(np.float32)


def validate_manifest(manifest,registry):
    require(manifest.get("schema") == "portable_e2e.map_appearance_matched_data.v1"
        and manifest.get("study_id") == NAME and manifest.get("labels") == list(LABELS),"data schema/labels differ")
    require(manifest.get("counts") == {"original":729,"paired":480,"per_arm":1209},"matched pool counts differ")
    require(manifest.get("arms") == {"control":"original729+paired_false480","appearance":"original729+paired_true480"},
            "appearance is not the sole paired-input factor")
    require(manifest.get("ctrack_train_eligible") is False and manifest.get("broad_dataset_admitted") is False,
            "study expanded to CTrack or broad admission")
    require(registry.get("schema") == "portable_e2e.study_train_registration.v1"
        and registry.get("study_id") == NAME and registry.get("study_train_authorized") is True
        and registry.get("route_sha256") == ROUTES and registry.get("source_diagnostic_receipts_preserved") is True,
            "study-only TRAIN registration missing or widened")
    require(registry.get("ctrack_train_eligible") is False and registry.get("broad_registry_modified") is False,
            "registration changes protected boundaries")
    require(registry.get("source_diagnostic_train_eligible_remains_false") is True
        and registry.get("dev_or_test_used_for_fit") is False
        and registry.get("counts") == manifest["counts"],"source roles or fitting boundary differs")
    optimization = registry.get("optimization",{})
    require({name:optimization.get(name) for name in ("steps_per_arm","batch","seed","learning_rate","weight_decay","optimizer")}
        == {"steps_per_arm":STEPS,"batch":BATCH,"seed":SEED,"learning_rate":LR,"weight_decay":WEIGHT_DECAY,
            "optimizer":"fresh AdamW in each arm"},"registered optimization differs")
    require(optimization.get("common_schedule") == manifest.get("schedule_contract"),"registered common exposure differs")
    require(manifest.get("parent_weights_sha256") == registry.get("parent_weights_sha256") == PARENT_SHA,
            "same existing B initialization required")
    expected_files = {"original_train729.npz","paired_train480.npz","training_schedule.npz","parent_weights.npz"}
    require(set(manifest.get("files",{})) == expected_files,"unexpected data package members")
    require(manifest["files"]["original_train729.npz"]["sha256"] == ORIGINAL_SHA
        and manifest["files"]["parent_weights.npz"]["sha256"] == PARENT_SHA,"original replay/parent changed")
    require(registry.get("data_files") == manifest["files"],"registered data members differ")
    ids = manifest.get("paired_sample_ids",[])
    expected = [f"{route}:frames/{index:04d}" for route in ("town01_p01","town02_p00") for index in range(240)]
    require(ids == expected and len(set(ids)) == 480,"same-frame IDs reordered, filtered or duplicated")


def load_package(root,expected_manifest_sha,expected_registry_sha):
    require(root.is_dir() and root.resolve() == root,"regular canonical package directory required")
    require(digest(root/"manifest.json") == expected_manifest_sha
        and digest(root/"study_train_registry.json") == expected_registry_sha,"registered input bytes changed")
    manifest,registry = read_json(root/"manifest.json"),read_json(root/"study_train_registry.json")
    validate_manifest(manifest,registry)
    require(manifest["study_registry_sha256"] == expected_registry_sha,"registry reference differs")
    require(digest(root/"paired_samples.json") == manifest["paired_samples_sha256"],"same-frame provenance changed")
    rows = read_json(root/"paired_samples.json")
    require([row.get("sample_id") for row in rows] == manifest["paired_sample_ids"],"paired provenance IDs differ")
    for name,ref in manifest["files"].items():
        require(digest(root/name) == ref["sha256"],"registered package file changed: "+name)
    with np.load(root/"original_train729.npz",allow_pickle=False) as data:
        require(set(data.files) == {"rgb","target"},"original replay archive keys differ")
        original_rgb,original_target = data["rgb"].copy(),data["target"].copy()
    with np.load(root/"paired_train480.npz",allow_pickle=False) as data:
        require(set(data.files) == {"rgb_false","rgb_true","target"},"paired archive keys differ")
        pair_false,pair_true,pair_target = (data[name].copy() for name in ("rgb_false","rgb_true","target"))
    for value,shape in ((original_rgb,(729,360,640,3)),(original_target,(729,360,640)),
            (pair_false,(480,360,640,3)),(pair_true,(480,360,640,3)),(pair_target,(480,360,640))):
        require(value.shape == shape and value.dtype == np.uint8,"native data ABI differs")
    require(int(original_target.max())<7 and int(pair_target.max())<7,"class ID out of range")
    for index,row in enumerate(rows):
        for key,value in (("rgb_false_sha256",pair_false[index]),("rgb_true_sha256",pair_true[index]),
                ("shared_target_sha256",pair_target[index])):
            require(hashlib.sha256(value.tobytes()).hexdigest() == row[key],"decoded same-frame data differs")
    pixels = np.bincount(original_target.ravel(),minlength=7)+np.bincount(pair_target.ravel(),minlength=7)
    require(pixels.tolist() == manifest["common_class_pixels"]
        and common_weights(pixels).tolist() == manifest["common_class_weights"],"common target weights differ")
    with np.load(root/"training_schedule.npz",allow_pickle=False) as data:
        require(set(data.files) == {"indices","crops"},"schedule keys differ")
        indices,crops = data["indices"].copy(),data["crops"].copy()
    require(validate_schedule(indices,crops) == manifest["schedule_contract"],"sample/crop exposure differs")
    return manifest,(original_rgb,original_target,pair_false,pair_true,pair_target),indices,crops
