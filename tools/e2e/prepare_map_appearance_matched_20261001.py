#!/usr/bin/env python3
"""HH_261001 - Register only QA/reviewed paired TRAIN-route bytes for one study."""

from datetime import datetime, timezone
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys

sys.dont_write_bytecode = True
try:
    from . import map_appearance_matched_contract_20261001 as contract
    from . import audit_map_paired_rgb_20261001_v2 as qa
    from . import evaluate_ctrack_fixed_maps_20261001 as frozen
except ImportError:
    import map_appearance_matched_contract_20261001 as contract
    import audit_map_paired_rgb_20261001_v2 as qa
    import evaluate_ctrack_fixed_maps_20261001 as frozen

import numpy as np

ROOT = frozen.ROOT
DOCS = ROOT/"docs/assets/validation/2026-10-01/map_paired_appearance"
PLAN = DOCS/"TRAIN_PLAN.md"
OUTPUT = ROOT/"storage/experiments/training/2026-10-01"/contract.NAME
CAPTURES = {name:frozen.OUTPUT_ROOT/f"map_paired_{name}_20261001_v2/mapping" for name in contract.ROUTES}
QA_PATHS = {name:DOCS/f"{name}_qa_v2.json" for name in contract.ROUTES}
REVIEWS = {name:frozen.OUTPUT_ROOT/f"map_paired_{name}_fixed_models_20261001_v2/started.json" for name in contract.ROUTES}
MODEL_SOURCE = frozen.RUN/"source"


def copy_new(source,target):
    target.parent.mkdir(parents=True,exist_ok=True)
    with source.open("rb") as incoming,target.open("xb") as outgoing:
        shutil.copyfileobj(incoming,outgoing,1024*1024)
    contract.require(contract.digest(source) == contract.digest(target),"archival copy differs")


def review_contract(review,scenario,source,qa_sha):
    contract.require(review.get("route") == scenario and review.get("source") == str(source)
        and review.get("qa_sha256") == qa_sha and review.get("policy",{}).get("visual_indices") == [0,119,239]
        and review.get("policy",{}).get("rgb_conditions") == ["false","true"]
        and isinstance(review.get("review_note"),str) and len(review["review_note"].strip()) >= 20,
        "explicit fixed false/true frame review receipt required")


def input_contract():
    frozen.model_source_contract()
    source_manifest = contract.read_json(frozen.DATA/"manifest.json")
    contract.require(source_manifest["files"]["train_b"]["count"] == 729
        and source_manifest["files"]["train_b"]["towns"] == ["Town01","Town02_Opt"],"original TRAIN provenance differs")
    original = frozen.DATA/"train_b.npz"
    contract.require(contract.digest(original) == contract.ORIGINAL_SHA,"original729 bytes changed")
    audited,reviewed = {},{}
    for name,source in CAPTURES.items():
        saved = contract.read_json(QA_PATHS[name])
        fresh = qa.audit(source)
        contract.require(saved == fresh and fresh["status"] == "QA_CONDITIONAL"
            and fresh["frame_count"] == 240 and fresh["route_sha256"] == contract.ROUTES[name]
            and fresh["intended_role"] == qa.capture.ROLE
            and fresh["train_eligible"] is False and fresh["dataset_admitted"] is False,
            "paired diagnostic raw-data QA or source role differs")
        review = contract.read_json(REVIEWS[name])
        review_contract(review,name,source,contract.digest(QA_PATHS[name]))
        audited[name],reviewed[name] = fresh,review
    return original,audited,reviewed


def build():
    contract.require(not OUTPUT.exists() and not OUTPUT.is_symlink(),"fresh study package required")
    contract.require(shutil.disk_usage(ROOT).free >= 5*1024**3,"minimum5GiB disk reserve required")
    original,audited,reviewed = input_contract()
    contract.require(PLAN.is_file(),"approved matched-fit source plan missing")
    source_files = {"train_map_appearance_matched_20261001.py":ROOT/"tools/e2e/train_map_appearance_matched_20261001.py",
        "map_appearance_matched_contract_20261001.py":Path(contract.__file__).resolve(),"plan.md":PLAN}
    for name in ("map_perception.py","map_context_recovery.py","train_map_photometric_pilot.py"):
        source_files[name] = MODEL_SOURCE/name
    source_fingerprints = {name:contract.digest(path) for name,path in source_files.items()}
    # HH_261001 - No output or TRAIN registration exists before both actual QA/review checks pass.
    OUTPUT.mkdir(parents=True)
    copy_new(original,OUTPUT/"original_train729.npz")
    copy_new(frozen.RUN/"B/weights.npz",OUTPUT/"parent_weights.npz")
    false = np.empty((480,360,640,3),dtype=np.uint8)
    true = np.empty_like(false)
    target = np.empty((480,360,640),dtype=np.uint8)
    rows,sample_ids = [],[]
    for route_index,(name,source) in enumerate(CAPTURES.items()):
        records = contract.read_json(source/"records.json")
        for index,row in enumerate(records):
            position = route_index*240+index
            path = source/(row["stem"]+".npz")
            contract.require(contract.digest(path) == row["artifact_sha256"][".npz"],"raw paired bytes changed")
            with np.load(path,allow_pickle=False) as data:
                false[position] = data["rgb_false"][...,[2,1,0]]
                true[position] = data["rgb_true"][...,[2,1,0]]
                target[position] = frozen.targets(data["semantic"])
            sample_id = name+":"+row["stem"]
            sample_ids.append(sample_id)
            rows.append({"sample_id":sample_id,"capture_index":index,"carla_frame":row["frame"],
                "timestamp_s":row["timestamp_s"],"raw_path":str(path),"raw_sha256":row["artifact_sha256"][".npz"],
                "rgb_false_sha256":hashlib.sha256(false[position].tobytes()).hexdigest(),
                "rgb_true_sha256":hashlib.sha256(true[position].tobytes()).hexdigest(),
                "shared_target_sha256":hashlib.sha256(target[position].tobytes()).hexdigest()})
    with (OUTPUT/"paired_train480.npz").open("xb") as stream:
        np.savez_compressed(stream,rgb_false=false,rgb_true=true,target=target)
    with np.load(original,allow_pickle=False) as data:
        original_target = data["target"]
        contract.require(original_target.shape == (729,360,640) and original_target.dtype == np.uint8
            and int(original_target.max()) < 7,"original target ABI differs")
        pixels = np.bincount(original_target.ravel(),minlength=7)+np.bincount(target.ravel(),minlength=7)
    indices,crops = contract.schedule()
    schedule_contract = contract.validate_schedule(indices,crops)
    with (OUTPUT/"training_schedule.npz").open("xb") as stream:
        np.savez_compressed(stream,indices=indices,crops=crops)
    files = {name:{"sha256":contract.digest(OUTPUT/name),"bytes":(OUTPUT/name).stat().st_size}
        for name in ("original_train729.npz","paired_train480.npz","training_schedule.npz","parent_weights.npz")}
    reviews = {}
    for name,source in CAPTURES.items():
        for suffix,label in (("summary.json","capture_summary"),("records.json","capture_records")):
            copy_new(source/suffix,OUTPUT/"provenance"/name/(label+".json"))
        copy_new(QA_PATHS[name],OUTPUT/"provenance"/name/"raw_qa.json")
        copy_new(REVIEWS[name],OUTPUT/"provenance"/name/"render_review_receipt.json")
        reviews[name] = {"qa_sha256":contract.digest(QA_PATHS[name]),
            "review_receipt_sha256":contract.digest(REVIEWS[name]),"review_note":reviewed[name]["review_note"],
            "reviewed_indices":[0,119,239],"source_capture":str(source),
            "fixed_png_sha256":{f"{index:04d}.{condition}":audited[name]["frame_artifact_sha256"][f"frames/{index:04d}.{condition}.png"]
                for index in (0,119,239) for condition in ("false","true")}}
        for filename,sha in audited[name]["frame_artifact_sha256"].items():
            contract.require(contract.digest(source/filename) == sha,"diagnostic capture changed during package build")
        contract.require(contract.digest(QA_PATHS[name]) == reviews[name]["qa_sha256"]
            and contract.digest(REVIEWS[name]) == reviews[name]["review_receipt_sha256"],"review/QA changed while packaging")
    registry = {"schema":"portable_e2e.study_train_registration.v1","study_id":contract.NAME,
        "registered_at_utc":datetime.now(timezone.utc).isoformat(),"study_train_authorized":True,
        "authorization":"Parent approved matched appearance-only10000-step pair from the existing B checkpoint; both v2 QA and fixed RGB review gates satisfied.",
        "source_diagnostic_receipts_preserved":True,"source_diagnostic_train_eligible_remains_false":True,
        "route_sha256":contract.ROUTES,"counts":{"original":729,"paired":480,"per_arm":1209},
        "parent_weights_sha256":contract.PARENT_SHA,"parent_checkpoint_sha256":contract.PARENT_CHECKPOINT_SHA,
        "architecture":"MapContextRecovery in both arms; unchanged",
        "single_factor":"new paired480 RGB exposure false(control) versus true(appearance)",
        "train_use_scope":"this exact study only; no broad dataset/site admission",
        "broad_registry_modified":False,"ctrack_train_eligible":False,"dev_or_test_used_for_fit":False,
        "source_reviews":reviews,"data_files":files,"training_plan_sha256":contract.digest(PLAN),
        "optimization":{"steps_per_arm":10000,"batch":8,"seed":contract.SEED,"learning_rate":3e-5,
            "weight_decay":1e-4,"optimizer":"fresh AdamW in each arm","common_schedule":schedule_contract},
        "exposure_note":"CTrack previously inspected and motivated intervention; future CTrack metrics remain exposed diagnostics, not independent certification."}
    frozen.write_new(OUTPUT/"study_train_registry.json",registry)
    frozen.write_new(OUTPUT/"paired_samples.json",rows)
    manifest = {"schema":"portable_e2e.map_appearance_matched_data.v1","study_id":contract.NAME,
        "counts":{"original":729,"paired":480,"per_arm":1209},"labels":list(contract.LABELS),
        "arms":{"control":"original729+paired_false480","appearance":"original729+paired_true480"},
        "parent_weights_sha256":contract.PARENT_SHA,"files":files,
        "paired_sample_ids":sample_ids,"paired_samples_sha256":contract.digest(OUTPUT/"paired_samples.json"),
        "common_class_pixels":pixels.tolist(),"common_class_weights":contract.common_weights(pixels).tolist(),
        "schedule_contract":schedule_contract,"study_registry_sha256":contract.digest(OUTPUT/"study_train_registry.json"),
        "ctrack_train_eligible":False,"broad_dataset_admitted":False,
        "builder_sha256":contract.digest(Path(__file__).resolve()),"original_train_manifest_sha256":frozen.DATA_SHA,
        "raw_model_inputs":["RGB"],"depth_or_pose_model_input":False,"source_receipts_modified":False}
    contract.validate_manifest(manifest,registry)
    frozen.write_new(OUTPUT/"manifest.json",manifest)
    for name,path in source_files.items():
        contract.require(contract.digest(path) == source_fingerprints[name],"training source changed during build")
        copy_new(path,OUTPUT/"source"/name)
    for path in (Path(__file__).resolve(),Path(qa.__file__).resolve(),Path(qa.capture.__file__).resolve(),
            Path(qa.geometry.__file__).resolve(),Path(frozen.__file__).resolve()):
        copy_new(path,OUTPUT/"provenance/builder_sources"/path.name)
    bundle = {"schema":"portable_e2e.map_appearance_source.v1","study_id":contract.NAME,
        "files":source_fingerprints,
        "data_manifest_sha256":contract.digest(OUTPUT/"manifest.json"),
        "study_registry_sha256":contract.digest(OUTPUT/"study_train_registry.json")}
    frozen.write_new(OUTPUT/"source/bundle.json",bundle)
    result = {"status":"STUDY_TRAIN_PACKAGE_READY_NOT_FITTED","output":str(OUTPUT),
        "manifest_sha256":bundle["data_manifest_sha256"],"registry_sha256":bundle["study_registry_sha256"],
        "source_bundle_sha256":contract.digest(OUTPUT/"source/bundle.json"),
        "source_diagnostic_receipts_preserved":True,"gpu_used":False,"ctrack_train_eligible":False}
    frozen.write_new(OUTPUT/"build_result.json",result)
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--execute",action="store_true")
    args = parser.parse_args()
    if args.execute:
        result = build()
    else:
        input_contract()
        result = {"status":"QA_AND_REVIEW_PREFLIGHT_PASS","registration_created":False,"gpu_used":False}
    print(json.dumps(result,sort_keys=True),flush=True)
