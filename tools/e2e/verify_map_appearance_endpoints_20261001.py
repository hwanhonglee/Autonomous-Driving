#!/usr/bin/env python3
"""HH_261001 - Independent raw-target prediction recount; no inference or fitting."""

from datetime import datetime, timezone
import argparse
import json
from pathlib import Path
import sys

sys.dont_write_bytecode = True
try:
    from . import evaluate_map_appearance_endpoints_20261001 as evaluate
except ImportError:
    import evaluate_map_appearance_endpoints_20261001 as evaluate
import numpy as np
from PIL import Image


def independent_target(semantic):
    evaluate.contract.require(semantic.shape == (360,640,4) and semantic.dtype == np.uint8,"semantic ABI differs")
    target = np.zeros((360,640),np.uint8)
    for raw_id,class_id in ((1,1),(2,2),(24,3),(7,4),(8,5),(6,6)):
        target[semantic[...,2] == raw_id] = class_id
    return target


def independent_confusion(target,prediction):
    evaluate.contract.require(target.shape == prediction.shape and target.dtype == prediction.dtype == np.uint8
        and int(target.max()) < 7 and int(prediction.max()) < 7,"target/prediction ABI differs")
    matrix = np.zeros((7,7),np.int64)
    for actual in range(7):
        matrix[actual] = np.bincount(prediction[target == actual],minlength=7)
    return matrix


def check_metrics(matrix,result):
    require = evaluate.contract.require
    require(result["confusion"] == matrix.tolist(),"aggregate confusion differs")
    actual,predicted = matrix.sum(1),matrix.sum(0)
    require(result["pixel_accuracy"] == float(np.trace(matrix)/matrix.sum()),"pixel accuracy differs")
    for index,label in enumerate(evaluate.frozen.LABELS):
        tp = int(matrix[index,index]); gt = int(actual[index]); pred = int(predicted[index]); union = gt+pred-tp
        expected = {"ground_truth_pixels":gt,"predicted_pixels":pred,"true_positive_pixels":tp,
            "false_positive_pixels":pred-tp,"false_negative_pixels":gt-tp,"union_pixels":union,
            "iou":float(tp/union) if gt else None,"ground_truth_absent":not bool(gt)}
        require(result["class_metrics"][label] == expected,"class support, absent-class handling or IoU differs")


def verify(scenario,expected_summary_sha):
    c = evaluate.contract
    registration = evaluate.validate_registration()
    bindings = evaluate.fit_binding(expected_summary_sha)
    source,records,conditions,qa,distinction = evaluate.capture_binding(scenario,registration)
    output = evaluate.output_path(scenario)
    summary,started = (c.read_json(output/name) for name in ("summary.json","started.json"))
    c.require(summary["status"] == "DIAGNOSTIC_COMPLETE_NOT_PROMOTED" and summary["scenario"] == scenario
        and summary["fit_steps_during_evaluation"] == 0 and summary["frames_per_model_condition"] == 240
        and summary["ctrack_train_eligible"] is False and summary["model_promotion"] is False
        and summary["independent_validation"] is False and summary["performance_gates"] is None
        and summary["source_fingerprints_unchanged"] is True,"diagnostic completion or role differs")
    for item in (summary,started):
        c.require(item["fit_artifact_sha256"] == bindings and item["preregistration_sha256"] == c.digest(evaluate.PREREG)
            and all(item[key] == value for key,value in distinction.items()),"endpoint/forensic binding differs")
    c.require(started["policy"] == registration["policy"] and started["raw_receipt"] == registration["raw_receipts"][scenario],
              "started registration differs")
    c.require(c.digest(output/"per_frame.json") == summary["per_frame_sha256"],"per-frame receipt changed")
    rows = c.read_json(output/"per_frame.json")
    c.require(len(rows) == 240,"per-frame count differs")
    keys = [arm+"_"+condition for condition in conditions for arm in evaluate.ARMS]
    predictions,matrices = {},{}
    c.require(set(summary["prediction_artifacts"]) == {key+"_all240_predictions.npz" for key in keys},"unexpected prediction members")
    for key in keys:
        path = output/(key+"_all240_predictions.npz")
        c.require(c.digest(path) == summary["prediction_artifacts"][path.name],"prediction bytes changed")
        with np.load(path,allow_pickle=False) as data:
            c.require(data.files == ["prediction"],"prediction archive keys differ")
            value = data["prediction"].copy()
        c.require(value.shape == (240,360,640) and value.dtype == np.uint8 and int(value.max()) < 7,"all240 prediction ABI differs")
        predictions[key],matrices[key] = value,np.zeros((7,7),np.int64)
    checked_panels = 0
    for index,row in enumerate(records):
        c.require(rows[index]["index"] == index and rows[index]["sensor_frame"] == row["frame"]
            and rows[index]["timestamp_s"] == row["timestamp_s"],"per-frame identity reordered")
        raw = source/(row["stem"]+".npz")
        c.require(c.digest(raw) == row["artifact_sha256"][".npz"],"raw target changed")
        with np.load(raw,allow_pickle=False) as data:
            target = independent_target(data["semantic"])
            rgb = {condition:data["rgb" if len(conditions)==1 else "rgb_"+condition][...,[2,1,0]] for condition in conditions}
        for key in keys:
            matrix = independent_confusion(target,predictions[key][index])
            c.require(matrix.tolist() == rows[index][key]["confusion"],"independent per-frame confusion differs")
            matrices[key] += matrix
        if index in evaluate.INDICES:
            path = output/f"{scenario}_{index:03d}_full_fov.png"
            c.require(c.digest(path) == summary["full_fov_visuals"][path.name],"fixed panel artifact changed")
            canvas = np.asarray(Image.open(path))
            c.require(canvas.shape == (1640 if len(conditions)==2 else 860,1280,3),"native panel canvas ABI differs")
            if len(conditions) == 1:
                panels = ((0,57,rgb["true"]),(640,57,evaluate.frozen.COLORS[target]),
                    (0,442,evaluate.frozen.COLORS[predictions["control_true"][index]]),
                    (640,442,evaluate.frozen.COLORS[predictions["appearance_true"][index]]))
            else:
                panels = [(column*640,y,pixels) for column,condition in enumerate(("false","true"))
                    for y,pixels in ((57,rgb[condition]),(442,evaluate.frozen.COLORS[target]),
                        (827,evaluate.frozen.COLORS[predictions["control_"+condition][index]]),
                        (1212,evaluate.frozen.COLORS[predictions["appearance_"+condition][index]]))]
            for x,y,pixels in panels:
                c.require(np.array_equal(canvas[y:y+360,x:x+640],pixels),"native full-FOV RGB/GT/prediction panel differs")
                checked_panels += 1
    for key,matrix in matrices.items():
        c.require(int(matrix.sum()) == 240*360*640,"all-pixel count differs")
        check_metrics(matrix,summary["model_results"][key])
    for condition in conditions:
        for label in evaluate.frozen.LABELS:
            left = summary["model_results"]["control_"+condition]["class_metrics"][label]["iou"]
            right = summary["model_results"]["appearance_"+condition]["class_metrics"][label]["iou"]
            c.require(summary["iou_delta_appearance_minus_control"][condition][label] == (None if left is None else right-left),"model delta differs")
    evaluate.verify_raw_hashes(scenario,registration)
    c.require(evaluate.fit_binding(expected_summary_sha) == bindings,"fixed endpoints changed during recount")
    result = {"schema":"portable_e2e.map_appearance_endpoint_recount.v1","status":"PASS",
        "created_at_utc":datetime.now(timezone.utc).isoformat(),"scenario":scenario,
        "frames_per_model_condition":240,"model_condition_count":len(keys),"predicted_pixels_recounted":len(keys)*240*360*640,
        "native_full_fov_panels_verified":checked_panels,"all_per_frame_and_aggregate_counts_match":True,
        "summary_sha256":c.digest(output/"summary.json"),"fit_summary_sha256":expected_summary_sha,
        "inference_repeated":False,"gpu_used":False,"model_promotion":False,"ctrack_train_eligible":False,**distinction}
    evaluate.frozen.write_new(output/"prediction_audit.json",result)
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--scenario",choices=evaluate.SCENARIOS,required=True)
    parser.add_argument("--expected-fit-summary-sha256",required=True)
    args = parser.parse_args()
    print(json.dumps(verify(args.scenario,args.expected_fit_summary_sha256),sort_keys=True),flush=True)
