#!/usr/bin/env python3
"""HH_261001 - Frozen A/B on every same-frame false/true RGB pair; never fit."""

from datetime import datetime, timezone
import argparse
import json
from pathlib import Path
import sys
import time

sys.dont_write_bytecode = True
try:
    from . import evaluate_ctrack_fixed_maps_20261001 as frozen
    from . import audit_map_paired_rgb_20261001 as paired_qa
except ImportError:
    import evaluate_ctrack_fixed_maps_20261001 as frozen
    import audit_map_paired_rgb_20261001 as paired_qa

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = frozen.ROOT
DOCS = ROOT/"docs/assets/validation/2026-10-01/map_paired_appearance"
PREREG = DOCS/"preregistration.json"
RUNS = {name:frozen.OUTPUT_ROOT/f"map_paired_{name}_20261001_v1" for name in ("town01_p01","town02_p00")}
INDICES = (0,119,239)
ROLE = "known_train_routes_paired_appearance_diagnostic"


def policy():
    return {"role":ROLE,"routes":{k:str(v) for k,v in RUNS.items()},"frames_per_route":240,
        "rgb_conditions":["false","true"],"models":["A","B"],"weights_sha256":frozen.WEIGHTS_SHA,
        "frames_per_model_condition":240,"visual_indices":list(INDICES),"visual_native_wh":[640,360],
        "target":"one shared raw semantic sensor per frame, existing seven-class mapping",
        "input":"RGB only, float32/255, no crop/resize/filter/dedup",
        "device":"cpu","threads":2,"batch_size":1,"inference_order":"frame: false A/B then true A/B",
        "metrics":"full confusion/support/IoU; absent GT class IoU null; false-positive counts retained",
        "latency":"forward+argmax only, cold first frame included, no unloaded-runtime claim",
        "fit_steps":0,"performance_gates":None,"train_eligible":False,"dataset_admitted":False,
        "model_promotion":False,"ctrack_train_eligible":False,
        "claim_limit":"Composite RGB postprocess appearance effect on familiar TRAIN corridors, not unseen-map generalization"}


def sources():
    return {str(path):frozen.digest(path) for path in (Path(__file__).resolve(),Path(frozen.__file__).resolve(),
        Path(paired_qa.__file__).resolve(),Path(paired_qa.capture.__file__).resolve(),
        Path(paired_qa.geometry.__file__).resolve(),DOCS/"PLAN.md")}


def preregister():
    value = {"schema":"portable_e2e.map_paired_fixed_preregistration.v1",
        "created_at_utc":datetime.now(timezone.utc).isoformat(),"policy":policy(),
        "model_sources":frozen.model_source_contract(),"evaluator_sources":sources(),
        "predictions_made":False}
    frozen.write_new(PREREG,value)
    return {"status":"PREREGISTERED_NO_PREDICTIONS","path":str(PREREG),"sha256":frozen.digest(PREREG)}


def validate_registration():
    value = frozen.read_json(PREREG)
    frozen.require(value["policy"] == policy() and value["evaluator_sources"] == sources()
        and value["model_sources"] == frozen.model_source_contract(),"registered sources or policy changed")
    return value


def panels(route,index,rgb,target,predictions):
    """HH_261001 - Every source pixel retained; paired columns share target and pose."""
    image = Image.new("RGB",(1280,1640),(18,27,40))
    draw = ImageDraw.Draw(image)
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",16)
    draw.text((8,7),f"{route} | fixed frame {index:03d}/239 | same-frame paired RGB | no fitting",fill="white",font=font)
    for col,condition in enumerate(("false","true")):
        content = (("Raw RGB",rgb[condition]),("Shared semantic GT",frozen.COLORS[target]),
            ("Frozen A",frozen.COLORS[predictions["A_"+condition]]),
            ("Frozen B",frozen.COLORS[predictions["B_"+condition]]))
        for row,(label,pixels) in enumerate(content):
            x,y = col*640,35+row*385
            draw.text((x+8,y),f"{label} | postprocess={condition}",fill="white",font=font)
            image.paste(Image.fromarray(pixels),(x,y+22))
    draw.text((8,1580),"Same configured camera and target; postprocess switch includes lighting/gamma effects.",fill="white",font=font)
    draw.text((8,1607),"Known TRAIN routes only. CTrack stays external EVAL. No dataset admission or model promotion.",fill="white",font=font)
    return image


def execute(args):
    registration = validate_registration()
    frozen.require(args.review_note and len(args.review_note.strip()) >= 20,"explicit review note for both RGB conditions at0/119/239 required")
    source = RUNS[args.route]/"mapping"
    qa_path = args.qa.absolute()
    saved = frozen.read_json(qa_path)
    qa = paired_qa.audit(source)
    frozen.require(saved == qa and qa["status"] == "QA_CONDITIONAL","paired QA receipt differs from full replay")
    output = frozen.OUTPUT_ROOT/f"map_paired_{args.route}_fixed_models_20261001_v1"
    frozen.require(not output.exists(),"fresh paired evaluation output required")
    source_hashes = {"qa_sha256":frozen.digest(qa_path),"preregistration_sha256":frozen.digest(PREREG),
        "records_sha256":frozen.digest(source/"records.json"),"summary_sha256":frozen.digest(source/"summary.json")}
    output.mkdir()
    frozen.write_new(output/"started.json",{"created_at_utc":datetime.now(timezone.utc).isoformat(),
        "policy":registration["policy"],"route":args.route,"source":str(source),
        "review_note":args.review_note,**source_hashes})
    frozen.write_new(output/"source_qa_replayed.json",qa)
    records = frozen.read_json(source/"records.json")
    torch,models = frozen.load_models()
    keys = [arm+"_"+condition for condition in ("false","true") for arm in ("A","B")]
    predictions = {key:np.empty((240,360,640),dtype=np.uint8) for key in keys}
    matrices = {key:np.zeros((7,7),dtype=np.int64) for key in keys}
    timings,frame_rows,visuals = {key:[] for key in keys},[],{}
    with torch.no_grad():
        for index,row in enumerate(records):
            raw = source/(row["stem"]+".npz")
            frozen.require(frozen.digest(raw) == row["artifact_sha256"][".npz"],"raw bytes changed after QA")
            with np.load(raw,allow_pickle=False) as archive:
                rgb = {condition:archive["rgb_"+condition][...,[2,1,0]].copy() for condition in ("false","true")}
                target = frozen.targets(archive["semantic"])
            frame = {"index":index,"sensor_frame":row["frame"],"timestamp_s":row["timestamp_s"]}
            for condition in ("false","true"):
                tensor = torch.from_numpy(rgb[condition].transpose(2,0,1).copy()).float().unsqueeze(0)/255
                for arm,model in models.items():
                    key = arm+"_"+condition
                    start = time.perf_counter()
                    value = model(tensor).argmax(1)[0].numpy().astype(np.uint8)
                    timings[key].append((time.perf_counter()-start)*1000)
                    predictions[key][index] = value
                    matrix = frozen.confusion(target,value)
                    matrices[key] += matrix
                    frame[key] = {"confusion":matrix.tolist(),"inference_ms":timings[key][-1]}
            frame_rows.append(frame)
            if index in INDICES:
                path = output/f"{args.route}_{index:03d}_paired_full_fov.png"
                panels(args.route,index,rgb,target,{key:values[index] for key,values in predictions.items()}).save(path)
                visuals[path.name] = frozen.digest(path)
            if (index+1)%40 == 0:
                print(json.dumps({"route":args.route,"frames_per_model_condition":index+1}),flush=True)
    artifacts = {}
    for key,value in predictions.items():
        path = output/(key+"_all240_predictions.npz")
        with path.open("xb") as stream:
            np.savez_compressed(stream,prediction=value)
        artifacts[path.name] = frozen.digest(path)
    for name,sha in qa["frame_artifact_sha256"].items():
        frozen.require(frozen.digest(source/name) == sha,"paired source changed during inference")
    for name,sha in qa["receipt_sha256"].items():
        path = source/name if name in ("summary.json","records.json","map.json") else source.parent/name
        frozen.require(frozen.digest(path) == sha,"owner/source receipt changed during inference")
    frozen.require(frozen.digest(qa_path) == source_hashes["qa_sha256"]
        and frozen.digest(PREREG) == source_hashes["preregistration_sha256"],"QA/registration changed")
    validate_registration()
    results = {key:{**frozen.metrics(matrices[key]),"frames":240,
        "inference_p50_ms":float(np.quantile(timings[key],.5)),
        "inference_p95_ms":float(np.quantile(timings[key],.95))} for key in keys}
    delta = {arm:{label:None if results[arm+"_false"]["class_metrics"][label]["iou"] is None else
        results[arm+"_true"]["class_metrics"][label]["iou"]-results[arm+"_false"]["class_metrics"][label]["iou"]
        for label in frozen.LABELS} for arm in ("A","B")}
    frozen.write_new(output/"per_frame.json",frame_rows)
    result = {"schema":"portable_e2e.map_paired_fixed_models.v1","status":"DIAGNOSTIC_COMPLETE_NOT_PROMOTED",
        "route":args.route,"source":str(source),"intended_role":ROLE,"model_results":results,
        "iou_delta_true_minus_false":delta,"prediction_artifacts":artifacts,"full_fov_visuals":visuals,
        "per_frame_sha256":frozen.digest(output/"per_frame.json"),"frames_per_model_condition":240,
        "device":"cpu","threads":2,"fit_steps":0,"source_fingerprints_unchanged":True,
        "weights_sha256":frozen.WEIGHTS_SHA,**source_hashes,"train_eligible":False,"dataset_admitted":False,
        "model_promotion":False,"ctrack_train_eligible":False,"independent_dev_approved":False,
        "claim_limit":registration["policy"]["claim_limit"]}
    frozen.write_new(output/"summary.json",result)
    return {"status":result["status"],"output":str(output),"iou_delta_true_minus_false":delta}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--preregister",action="store_true")
    parser.add_argument("--execute",action="store_true")
    parser.add_argument("--route",choices=sorted(RUNS))
    parser.add_argument("--qa",type=Path)
    parser.add_argument("--review-note")
    args = parser.parse_args()
    frozen.require(not(args.preregister and args.execute),"choose registration or execution")
    if args.preregister:
        result = preregister()
    elif args.execute:
        frozen.require(args.route is not None and args.qa is not None,"route and completed QA required")
        result = execute(args)
    else:
        validate_registration()
        result = {"status":"FROZEN_PAIRED_EVALUATOR_PREFLIGHT_PASS","predictions_made":False}
    print(json.dumps(result,sort_keys=True),flush=True)
