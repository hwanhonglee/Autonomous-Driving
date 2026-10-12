#!/usr/bin/env python3
"""HH_261001 - Preregistered all-frame diagnostics of both fixed matched endpoints."""

from datetime import datetime, timezone
import argparse
import importlib
import json
from pathlib import Path
import sys
import time

sys.dont_write_bytecode = True
try:
    from . import evaluate_ctrack_fixed_maps_20261001 as frozen
    from . import audit_map_paired_rgb_20261001_v2 as paired_qa
    from . import map_appearance_matched_contract_20261001 as contract
    from . import train_map_appearance_matched_20261001 as training
except ImportError:
    import evaluate_ctrack_fixed_maps_20261001 as frozen
    import audit_map_paired_rgb_20261001_v2 as paired_qa
    import map_appearance_matched_contract_20261001 as contract
    import train_map_appearance_matched_20261001 as training
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = frozen.ROOT
PACKAGE = ROOT/"storage/experiments/training/2026-10-01"/contract.NAME
FIT = PACKAGE/"fit"
DOCS = ROOT/"docs/assets/validation/2026-10-01/map_appearance_endpoints"
PREREG = DOCS/"preregistration.json"
DATA_SHA = "32ed912505e0b215bfc54e563fcdba7553aa25412ba22d90b45bfa00339e93c8"
REGISTRY_SHA = "456b8b31ef2a3426d4c732236e7c549792ebe9d9c1cb246e0b055496441718af"
SOURCE_SHA = "0b6d5b4b394b7223f96701f185054656fb2aca694889b6b336e5f760514dda86"
INITIAL_STATE_SHA = "edf12b02e3bf34526ce30de6fb450c2fa91085004e5dbd42c95c9e159d6a2b67"
ARMS = ("control","appearance")
INDICES = (0,119,239)
SCENARIOS = ("ctrack_left","ctrack_straight","train_town01_p01","train_town02_p00")


def locations(scenario):
    contract.require(scenario in SCENARIOS,"unknown preregistered scenario")
    if scenario.startswith("ctrack_"):
        route = scenario.removeprefix("ctrack_")
        return frozen.SOURCE_ROOTS[route],frozen.QA_PATHS[route],("true",),frozen.OUTPUT_ROOT/f"ctrack_{route}_fixed_maps_20261001_v1/started.json"
    route = scenario.removeprefix("train_")
    return (frozen.OUTPUT_ROOT/f"map_paired_{route}_20261001_v2/mapping",
        ROOT/f"docs/assets/validation/2026-10-01/map_paired_appearance/{route}_qa_v2.json",
        ("false","true"),frozen.OUTPUT_ROOT/f"map_paired_{route}_fixed_models_20261001_v2/started.json")


def output_path(scenario):
    return frozen.OUTPUT_ROOT/f"map_appearance_endpoints_{scenario}_20261001_v1"


def policy():
    return {"study_id":contract.NAME,"scenarios":list(SCENARIOS),"arms":list(ARMS),
        "frames_per_scenario":240,"paired_conditions":["false","true"],"ctrack_condition":"true",
        "endpoint_steps":10000,"architecture_both":"MapContextRecovery","parent_weights_sha256":contract.PARENT_SHA,
        "data_manifest_sha256":DATA_SHA,"source_bundle_sha256":SOURCE_SHA,"study_registry_sha256":REGISTRY_SHA,
        "endpoint_binding":"both fixed fit_complete endpoints plus explicit final summary SHA; no intermediate checkpoints",
        "fit_path":str(FIT),"labels":list(frozen.LABELS),"raw_label_table":{str(k):v for k,v in frozen.RAW_LABELS.items()},
        "input":"all240 native BGRA->RGB float32/255; no resize,crop,filter,dedup,depth or pose inputs",
        "visual_indices":list(INDICES),"full_fov_wh":[640,360],"device":"cpu","threads":2,"batch":1,
        "order":"frame, condition false/true where available, control/appearance",
        "metrics":"full per-frame and aggregate confusion, class support, IoU; absent target IoU null, false positives retained",
        "latency":"forward+argmax CPU2 only, first cold included; p50/p95; no unloaded or full-pipeline10Hz claim",
        "fit_steps_during_evaluation":0,"performance_gates":None,"model_promotion":False,"ctrack_train_eligible":False,
        "paired_role":"same exact training examples: resubstitution appearance diagnostic, not validation",
        "ctrack_role":"exposed external_map_diagnostic_evaluation; no learned weights fitted on CTrack",
        "limitations":["CTrack failure and historical RGB exposure motivated this fit; not independent certification.",
            "Camera configured geometry matches prior TRAIN; postprocess, materials and source/editor versus package differ.",
            "LEFT original owner FAIL remains; forensic raw QA only, original live-map equality unavailable.",
            "Paired TRAIN frames belong to this fit and cannot demonstrate held-out generalization.",
            "No DEV tuning, learned thresholds, adaptive visual/frame selection or automatic promotion."]}


def sources():
    paths = [Path(__file__).resolve(),ROOT/"tools/e2e/verify_map_appearance_endpoints_20261001.py",
        ROOT/"learning/tests/test_map_appearance_endpoints_20261001.py",DOCS/"PLAN.md",
        Path(frozen.__file__).resolve(),Path(paired_qa.__file__).resolve(),Path(paired_qa.capture.__file__).resolve(),
        Path(paired_qa.geometry.__file__).resolve(),Path(frozen.capture_qa.__file__).resolve(),
        Path(frozen.capture_qa.glyphs.__file__).resolve(),Path(contract.__file__).resolve(),Path(training.__file__).resolve(),
        ROOT/"learning/portable_e2e/mapping.py",ROOT/"learning/portable_e2e/mapping_projection.py",
        ROOT/"docs/assets/validation/2026-10-01/ctrack_external_maps/camera_rig_correction.json"]
    return {str(path):contract.digest(path) for path in paths}


def raw_receipts():
    result = {}
    for scenario in SCENARIOS:
        source,qa,conditions,review = locations(scenario)
        records = contract.read_json(source/"records.json")
        contract.require(len(records) == 240 and [row["stem"] for row in records] == [f"frames/{i:04d}" for i in range(240)],
                         "exact unfiltered native240 frames required")
        reviewed = contract.read_json(review)
        if scenario.startswith("ctrack_"):
            contract.require(reviewed["render_review"]["indices"] == list(INDICES)
                and len(reviewed["render_review"]["note"]) >= 20,"prior CTrack native review absent")
        else:
            contract.require(reviewed["policy"]["visual_indices"] == list(INDICES)
                and reviewed["policy"]["rgb_conditions"] == list(conditions)
                and len(reviewed["review_note"]) >= 20,"prior paired native review absent")
        contract.require(reviewed["qa_sha256"] == contract.digest(qa),"prior review refers to another raw QA")
        result[scenario] = {"source":str(source),"conditions":list(conditions),
            "qa_sha256":contract.digest(qa),"review_sha256":contract.digest(review),
            "records_sha256":contract.digest(source/"records.json"),"summary_sha256":contract.digest(source/"summary.json"),
            "frame_artifacts":{row["stem"]+suffix:sha for row in records for suffix,sha in row["artifact_sha256"].items()}}
    return result


def package_binding():
    contract.require(contract.digest(PACKAGE/"manifest.json") == DATA_SHA
        and contract.digest(PACKAGE/"study_train_registry.json") == REGISTRY_SHA,"study registry/data changed")
    training.source_contract(PACKAGE/"source",SOURCE_SHA,DATA_SHA,REGISTRY_SHA)
    manifest = contract.read_json(PACKAGE/"manifest.json")
    contract.validate_manifest(manifest,contract.read_json(PACKAGE/"study_train_registry.json"))
    return manifest


def preregister():
    contract.require(not (FIT/"fit_complete.json").exists() and not any(output_path(s).exists() for s in SCENARIOS),
                     "register before imported final endpoints or predictions")
    package_binding()
    record = {"schema":"portable_e2e.map_appearance_endpoint_preregistration.v1",
        "created_at_utc":datetime.now(timezone.utc).isoformat(),"policy":policy(),"sources":sources(),
        "raw_receipts":raw_receipts(),"predictions_made":False,"final_weights_available":False}
    frozen.write_new(PREREG,record)
    return {"status":"PREREGISTERED_BEFORE_ENDPOINTS","sha256":contract.digest(PREREG),"path":str(PREREG)}


def validate_registration():
    value = contract.read_json(PREREG)
    contract.require(value["policy"] == policy() and value["sources"] == sources()
        and value["raw_receipts"] == raw_receipts() and value["predictions_made"] is False
        and value["final_weights_available"] is False,"frozen endpoint evaluation contract changed")
    package_binding()
    return value


def validate_endpoint_metadata(summary,started,complete,manifest):
    contract.require(summary.get("status") == "MATCHED_PAIR_COMPLETE_DIAGNOSTIC_NOT_PROMOTED"
        and complete.get("status") == "BOTH_FIXED_ENDPOINTS_SAVED_BEFORE_DEV"
        and summary.get("arms") == complete.get("arms") and set(complete.get("arms",{})) == set(ARMS),
        "both completed fixed endpoints required")
    for item in (summary,started):
        contract.require(item.get("study_id") == contract.NAME and item.get("data_manifest_sha256") == DATA_SHA
            and item.get("source_bundle_sha256") == SOURCE_SHA and item.get("study_registry_sha256") == REGISTRY_SHA
            and item.get("parent_weights_sha256") == contract.PARENT_SHA and item.get("steps_per_arm") == 10000
            and item.get("batch") == 8 and item.get("common_class_weights") == manifest["common_class_weights"]
            and item.get("schedule_contract") == manifest["schedule_contract"]
            and item.get("ctrack_train_eligible") is False and item.get("model_promotion") is False,"fit binding differs")
    contract.require(summary.get("dev_used_during_optimization") is False and summary.get("source_and_data_unchanged") is True,
                     "fit source/selection contract failed")
    for info in complete["arms"].values():
        contract.require(info.get("steps") == 10000 and info.get("parameter_count") == 245831
            and info.get("initial_state_sha256") == INITIAL_STATE_SHA and info.get("optimizer_initial_state_entries") == 0
            and info.get("sample_sequence_sha256") == manifest["schedule_contract"]["indices_sha256"]
            and info.get("crop_sequence_sha256") == manifest["schedule_contract"]["crops_sha256"],"matched endpoint exposure/initialization differs")


def fit_binding(expected_summary_sha):
    contract.require(contract.digest(FIT/"summary.json") == expected_summary_sha,"explicit final fit summary SHA differs")
    manifest = package_binding()
    summary,started,complete = (contract.read_json(FIT/name) for name in ("summary.json","started.json","fit_complete.json"))
    validate_endpoint_metadata(summary,started,complete,manifest)
    training.source_contract(FIT/"source",SOURCE_SHA,DATA_SHA,REGISTRY_SHA)
    contract.require(contract.digest(FIT/"inputs/manifest.json") == DATA_SHA
        and contract.digest(FIT/"inputs/study_train_registry.json") == REGISTRY_SHA
        and contract.digest(FIT/"inputs/paired_samples.json") == manifest["paired_samples_sha256"],"fit archived inputs differ")
    hashes = {name:contract.digest(FIT/name) for name in ("summary.json","started.json","fit_complete.json")}
    for arm,info in complete["arms"].items():
        contract.require(contract.read_json(FIT/arm/"training.json") == info,"endpoint receipt differs")
        for name,key in (("weights.npz","weights_sha256"),("checkpoint.pt","checkpoint_sha256")):
            contract.require(contract.digest(FIT/arm/name) == info[key],"endpoint artifact changed")
            hashes[arm+"/"+name] = info[key]
        hashes[arm+"/training.json"] = contract.digest(FIT/arm/"training.json")
    return hashes


def capture_binding(scenario,registration):
    source,qa_path,conditions,review = locations(scenario)
    if scenario.startswith("ctrack_"):
        _,records,qa = frozen.validate_capture(scenario.removeprefix("ctrack_"),qa_path)
        distinction = {"source_qa_status":qa["status"],"original_owner_status":qa["owner_provenance"]["original_owner_status"],
            "original_live_replay_exact_match":qa["original_live_replay_exact_match"],"evaluation_role":"exposed_external_map_diagnostic"}
    else:
        qa = paired_qa.audit(source)
        contract.require(qa == contract.read_json(qa_path) and qa["status"] == "QA_CONDITIONAL","paired raw QA differs")
        records = contract.read_json(source/"records.json")
        distinction = {"source_qa_status":qa["status"],"original_owner_status":"PASS",
            "original_live_replay_exact_match":True,"evaluation_role":"training_resubstitution_appearance_diagnostic"}
    verify_raw_hashes(scenario,registration)
    return source,records,conditions,qa,distinction


def verify_raw_hashes(scenario,registration):
    source = locations(scenario)[0]
    for filename,sha in registration["raw_receipts"][scenario]["frame_artifacts"].items():
        contract.require(contract.digest(source/filename) == sha,"registered raw RGB/target bytes changed")


def load_models(bindings):
    import torch
    torch.set_num_threads(2); torch.set_num_interop_threads(2)
    sys.path.insert(0,str(FIT/"source"))
    core,context = (importlib.import_module(name) for name in ("map_perception","map_context_recovery"))
    for module in (core,context):
        contract.require(Path(module.__file__).resolve().parent == FIT/"source","model import escaped frozen fit source")
    contract.require(tuple(core.LABELS) == frozen.LABELS,"class order differs")
    return torch,{arm:frozen.safe_npz_state(context.MapContextRecovery(),FIT/arm/"weights.npz",bindings[arm+"/weights.npz"],torch) for arm in ARMS}


def raw_frame(source,row,conditions):
    with np.load(source/(row["stem"]+".npz"),allow_pickle=False) as data:
        rgb = {condition:data["rgb" if len(conditions)==1 else "rgb_"+condition][...,[2,1,0]].copy() for condition in conditions}
        target = frozen.targets(data["semantic"])
    for value in rgb.values():
        contract.require(value.shape == (360,640,3) and value.dtype == np.uint8,"native RGB ABI differs")
    return rgb,target


def panel_layout(rgb,target,predictions,conditions):
    if len(conditions) == 1:
        return [(0,57,rgb["true"]),(640,57,frozen.COLORS[target]),
            (0,442,frozen.COLORS[predictions["control_true"]]),(640,442,frozen.COLORS[predictions["appearance_true"]])]
    return [(column*640,57+row*385,pixels) for column,condition in enumerate(conditions)
        for row,pixels in enumerate((rgb[condition],frozen.COLORS[target],
            frozen.COLORS[predictions["control_"+condition]],frozen.COLORS[predictions["appearance_"+condition]]))]


def render(scenario,index,rgb,target,predictions,conditions):
    paired = len(conditions)==2
    image = Image.new("RGB",(1280,1640 if paired else 860),(18,27,40))
    draw = ImageDraw.Draw(image)
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",16)
    draw.text((8,7),f"{scenario} | fixed frame {index:03d}/239 | matched10000-step endpoints | no promotion",font=font,fill="white")
    captions = (["RGB false","Shared GT","Control: paired-false training","Appearance: paired-true training",
        "RGB true","Shared GT","Control: paired-false training","Appearance: paired-true training"] if paired else
        ["Original lit RGB true","Semantic GT","Control: paired-false training","Appearance: paired-true training"])
    for (x,y,pixels),caption in zip(panel_layout(rgb,target,predictions,conditions),captions):
        draw.text((x+8,y-22),caption,font=font,fill="white")
        image.paste(Image.fromarray(pixels),(x,y))
    draw.text((8,image.height-44),"TRAIN resubstitution only; not validation." if paired else
        "Exposed CTrack diagnostic; same configured rig, source/material shifts remain.",font=font,fill="white")
    draw.text((8,image.height-21),"LEFT original FAIL and unavailable live equality remain. No independent certification.",font=font,fill="white")
    return image


def execute(scenario,expected_summary_sha):
    registration = validate_registration()
    bindings = fit_binding(expected_summary_sha)
    output = output_path(scenario)
    contract.require(not output.exists() and not output.is_symlink(),"fresh diagnostic output required")
    source,records,conditions,qa,distinction = capture_binding(scenario,registration)
    output.mkdir()
    frozen.write_new(output/"started.json",{"created_at_utc":datetime.now(timezone.utc).isoformat(),"scenario":scenario,
        "policy":registration["policy"],"fit_artifact_sha256":bindings,"preregistration_sha256":contract.digest(PREREG),
        "raw_receipt":registration["raw_receipts"][scenario],**distinction})
    frozen.write_new(output/"source_qa_replayed.json",qa)
    torch,models = load_models(bindings)
    keys = [arm+"_"+condition for condition in conditions for arm in ARMS]
    predictions = {key:np.empty((240,360,640),np.uint8) for key in keys}
    matrices = {key:np.zeros((7,7),np.int64) for key in keys}
    timings,rows,visuals = {key:[] for key in keys},[],{}
    with torch.no_grad():
        for index,row in enumerate(records):
            contract.require(contract.digest(source/(row["stem"]+".npz")) == row["artifact_sha256"][".npz"],"raw frame changed after QA")
            rgb,target = raw_frame(source,row,conditions)
            report = {"index":index,"sensor_frame":row["frame"],"timestamp_s":row["timestamp_s"]}
            for condition in conditions:
                tensor = torch.from_numpy(rgb[condition].transpose(2,0,1).copy()).float().unsqueeze(0)/255
                for arm,model in models.items():
                    key = arm+"_"+condition
                    start = time.perf_counter()
                    pred = model(tensor).argmax(1)[0].numpy().astype(np.uint8)
                    timings[key].append((time.perf_counter()-start)*1000)
                    predictions[key][index] = pred
                    matrix = frozen.confusion(target,pred)
                    matrices[key] += matrix
                    report[key] = {"confusion":matrix.tolist(),"inference_ms":timings[key][-1]}
            rows.append(report)
            if index in INDICES:
                path = output/f"{scenario}_{index:03d}_full_fov.png"
                render(scenario,index,rgb,target,{key:value[index] for key,value in predictions.items()},conditions).save(path)
                visuals[path.name] = contract.digest(path)
            if (index+1)%40 == 0:
                print(json.dumps({"scenario":scenario,"frames_per_model_condition":index+1}),flush=True)
    artifacts = {}
    for key,value in predictions.items():
        path = output/(key+"_all240_predictions.npz")
        with path.open("xb") as stream:
            np.savez_compressed(stream,prediction=value)
        artifacts[path.name] = contract.digest(path)
    verify_raw_hashes(scenario,registration)
    contract.require(fit_binding(expected_summary_sha) == bindings,"endpoint source changed during inference")
    validate_registration()
    results = {key:{**frozen.metrics(matrices[key]),"frames":240,
        "inference_p50_ms":float(np.quantile(timings[key],.5)),"inference_p95_ms":float(np.quantile(timings[key],.95)),
        "first_inference_ms":timings[key][0]} for key in keys}
    delta = {condition:{label:None if results["control_"+condition]["class_metrics"][label]["iou"] is None else
        results["appearance_"+condition]["class_metrics"][label]["iou"]-results["control_"+condition]["class_metrics"][label]["iou"]
        for label in frozen.LABELS} for condition in conditions}
    frozen.write_new(output/"per_frame.json",rows)
    result = {"schema":"portable_e2e.map_appearance_endpoint_diagnostic.v1","status":"DIAGNOSTIC_COMPLETE_NOT_PROMOTED",
        "scenario":scenario,"source":str(source),"model_results":results,"iou_delta_appearance_minus_control":delta,
        "prediction_artifacts":artifacts,"full_fov_visuals":visuals,"per_frame_sha256":contract.digest(output/"per_frame.json"),
        "fit_artifact_sha256":bindings,"preregistration_sha256":contract.digest(PREREG),"source_fingerprints_unchanged":True,
        "frames_per_model_condition":240,"device":"cpu","threads":2,"torch_version":torch.__version__,
        "fit_steps_during_evaluation":0,"ctrack_train_eligible":False,"model_promotion":False,
        "independent_validation":False,"performance_gates":None,"limitations":registration["policy"]["limitations"],**distinction}
    frozen.write_new(output/"summary.json",result)
    return {"status":result["status"],"scenario":scenario,"output":str(output),"iou_delta_appearance_minus_control":delta}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--preregister",action="store_true"); mode.add_argument("--execute",action="store_true")
    parser.add_argument("--scenario",choices=SCENARIOS)
    parser.add_argument("--expected-fit-summary-sha256")
    args = parser.parse_args()
    if args.preregister:
        result = preregister()
    elif args.execute:
        contract.require(args.scenario and args.expected_fit_summary_sha256,"scenario and explicit completed fit SHA required")
        result = execute(args.scenario,args.expected_fit_summary_sha256)
    else:
        validate_registration(); result = {"status":"FROZEN_PREREGISTRATION_PASS","predictions_made":False}
    print(json.dumps(result,sort_keys=True),flush=True)
