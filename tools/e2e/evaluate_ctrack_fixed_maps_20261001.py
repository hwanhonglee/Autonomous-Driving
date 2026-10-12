#!/usr/bin/env python3
"""HH_261001 - Preregister and evaluate frozen map A/B on external CTrack RGB.

No fitting, threshold selection, checkpoint selection, CARLA, or promotion.
LEFT permits only the pinned forensic raw recovery; STRAIGHT requires v2 PASS.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib
import json
import os
from pathlib import Path
import sys
import time

sys.dont_write_bytecode = True
os.environ["OMP_NUM_THREADS"] = "2"
os.environ["MKL_NUM_THREADS"] = "2"

import numpy as np
from PIL import Image, ImageDraw, ImageFont

try:
    from . import audit_ctrack_isolated_moving_20261001_v2 as capture_qa
except ImportError:
    import audit_ctrack_isolated_moving_20261001_v2 as capture_qa

ROOT = Path(__file__).resolve().parents[2]
RUN = ROOT / "storage/experiments/training/2026-10-01/map_context_recovery_20261001_v1"
DATA = ROOT / "storage/experiments/training/2026-09-29/map_town02_p00_route_holdout_data_v1"
OUTPUT_ROOT = ROOT / "storage/experiments/validation/2026-10-01"
PREREG = ROOT / "docs/assets/validation/2026-10-01/ctrack_external_maps/preregistration.json"
PLAN = PREREG.with_name("PLAN.md")
ROLE = "external_map_diagnostic_evaluation"
LABELS = ("other","road","sidewalk","lane_marking","traffic_light_fixture","traffic_sign","pole")
RAW_LABELS = {1:"road",24:"lane_marking",2:"sidewalk",7:"traffic_light_fixture",8:"traffic_sign",6:"pole"}
COLORS = np.asarray(((30,30,40),(67,92,123),(141,88,147),(246,222,90),
                     (246,100,70),(60,220,180),(179,190,199)),dtype=np.uint8)
VISUAL_INDICES = (0,119,239)
WEIGHTS_SHA = {"A":"dd212df685f6e46f4347df457c18f559785de5649128033068dd1fc9a770f9a6",
               "B":"ff5d7d03a43b859d21a0a3f6b01d815054d89d78e850730121f326e5b79c023e"}
SUMMARY_SHA = "127c15564bd2180eee4061fd6993f52f00bac0dcbc65aecc6505b5dae425b606"
BUNDLE_SHA = "1ed896e7f46909076553a37f97facde0546ffbe31f03bfba5c7857491e47a7ab"
DATA_SHA = "f4764b9aa9d015f3d136a145f6194dd60bda8900c27dd2423de6145b9f472834"
SOURCE_ROOTS = {
    "left":OUTPUT_ROOT/"ctrack_left_external_diagnostic_20261001/capture",
    "straight":OUTPUT_ROOT/"ctrack_straight_external_diagnostic_20261001_v2/capture"}
QA_PATHS = {
    "left":ROOT/"docs/assets/validation/2026-10-01/ctrack_isolation/left_forensic_raw_v2.json",
    "straight":ROOT/"docs/assets/validation/2026-10-01/ctrack_isolation/straight_raw_v2.json"}
LIMITATIONS = (
    "CTrack has no examples in these learned map weights' Town01+Town02 TRAIN.",
    "Historical human CTrack RGB exposure and source repairs are disclosed; not independent certification.",
    "CTrack RGB postprocess=true differs from TRAIN postprocess=false; camera rig also differs.",
    "This is joint source/render/rig/map-domain diagnosis, not pure map-geometry generalization.",
    "LEFT is forensic raw recovery: original owner FAIL remains, original live-map equality unavailable.",
    "Oracle raw semantics are targets only; depth/pose never enter learned model input.",
    "No TRAIN admission, threshold tuning, retraining, automatic promotion or whole-pipeline 10Hz claim.")


def require(ok, message):
    if not ok:
        raise ValueError(message)


def digest(path):
    require(path.is_file() and not path.is_symlink(),f"regular file required: {path}")
    state = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b""):
            state.update(chunk)
    return state.hexdigest()


def read_json(path):
    digest(path)
    return json.loads(path.read_text())


def write_new(path, value):
    require(not path.exists() and not path.is_symlink(),f"fresh output required: {path}")
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open("x") as stream:
        json.dump(value,stream,indent=2,sort_keys=True,allow_nan=False)
        stream.write("\n")


def model_source_contract():
    require(digest(RUN/"summary.json") == SUMMARY_SHA,"fixed run summary changed")
    require(digest(RUN/"source/bundle.json") == BUNDLE_SHA,"architecture bundle changed")
    require(digest(DATA/"manifest.json") == DATA_SHA,"TRAIN provenance manifest changed")
    summary, bundle, manifest = (read_json(path) for path in
        (RUN/"summary.json",RUN/"source/bundle.json",DATA/"manifest.json"))
    require(manifest["files"]["train_b"]["towns"] == ["Town01","Town02_Opt"]
            and manifest["files"]["train_b"]["count"] == 729,"unexpected TRAIN sites/count")
    hashes = {str(path):digest(path) for path in
              (RUN/"summary.json",RUN/"source/bundle.json",DATA/"manifest.json")}
    for name,sha in bundle["files"].items():
        path = RUN/"source"/name
        require(path.parent == RUN/"source" and digest(path) == sha,"archived model source changed")
        hashes[str(path)] = sha
    for arm,sha in WEIGHTS_SHA.items():
        path = RUN/arm/"weights.npz"
        require(digest(path) == summary[arm]["weights_sha256"] == sha,"frozen weights changed")
        hashes[str(path)] = sha
    return hashes


def evaluator_source_contract():
    return {str(path):digest(path) for path in
        (Path(__file__).resolve(),Path(capture_qa.__file__).resolve(),
         Path(capture_qa.glyphs.__file__).resolve(),PLAN)}


def policy():
    return {"role":ROLE,"frames_per_route":240,"routes":["left","straight"],
        "source_paths":{key:str(path) for key,path in SOURCE_ROOTS.items()},
        "labels":list(LABELS),"raw_label_table":{str(k):v for k,v in RAW_LABELS.items()},
        "unknown_tags_map_to":"other","full_fov_wh":[640,360],"frame_selection":"all240_no_dedup",
        "rgb_processing":"raw BGRA -> RGB -> float32/255 only; no resize/crop/normalization",
        "visual_indices":list(VISUAL_INDICES),"visual_kind":"native full-FOV RGB/GT/A/B PNG only",
        "weights_sha256":WEIGHTS_SHA,"device":"cpu","torch_threads":2,"batch_size":1,
        "inference_order":"frame-major A then B; no warmup or checkpoint selection",
        "latency":"model forward + argmax only; first cold inference included; p50/p95 per arm",
        "iou_absent_ground_truth_class":None,"absent_class_false_positives":"still reported as pixel counts",
        "confusion_orientation":"rows=ground_truth; columns=prediction",
        "fit_steps":0,"learned_thresholds":False,"performance_pass_thresholds":None,
        "train_eligible":False,"dataset_admitted":False,"model_promotion":False,
        "independent_dev_approved":False,"limitations":list(LIMITATIONS)}


def preregister():
    require(not PREREG.exists(),"preregistration already exists; never overwrite it")
    record = {"schema":"portable_e2e.ctrack_fixed_maps_preregistration.v1",
        "created_at_utc":datetime.now(timezone.utc).isoformat(),"policy":policy(),
        "model_sources":model_source_contract(),"evaluator_sources":evaluator_source_contract(),
        "predictions_made":False,"simulator_launched":False}
    write_new(PREREG,record)
    return {"status":"PREREGISTERED_NO_PREDICTIONS","path":str(PREREG),"sha256":digest(PREREG)}


def validate_preregistration():
    record = read_json(PREREG)
    require(record["policy"] == policy(),"frozen evaluation policy changed")
    require(record["model_sources"] == model_source_contract(),"registered model source changed")
    require(record["evaluator_sources"] == evaluator_source_contract(),"registered evaluator changed")
    require(record["predictions_made"] is False,"invalid pre-prediction record")
    return record


def validate_capture(scenario, qa_path):
    source = SOURCE_ROOTS[scenario]
    saved = read_json(qa_path)
    route = capture_qa.ROUTES[scenario]
    fresh = capture_qa.audit_capture(source,route["path"],route["sha256"],route["road_option"])
    expected = "QA_CONDITIONAL_FORENSIC_RAW" if scenario == "left" else "QA_CONDITIONAL"
    require(saved.get("status") == fresh["status"] == expected and not fresh["fail_reasons"],
            "raw capture/owner QA must pass with the specified forensic distinction")
    require(saved.get("input_sha256") == fresh["input_sha256"],"capture differs from QA receipt")
    require(saved.get("immutable_source_hashes_verified_after_replay") is True,"QA end hashes missing")
    for name,sha in saved.get("validator_source_sha256",{}).items():
        require(digest(Path(name)) == sha,"QA validator changed after audit")
    require(saved.get("validator_source_sha256"),"QA validator provenance missing")
    require(fresh.get("intended_role") == ROLE and all(fresh.get(key) is False for key in
        ("train_eligible","dataset_admitted","independent_dev_approved")),"source role changed")
    if scenario == "left":
        require(fresh.get("forensic_raw_recovery") is True
                and fresh.get("original_live_replay_exact_match") is None,"forensic qualification changed")
        rebuilt = saved.get("offline_reconstruction",{})
        require(rebuilt.get("all_saved_projected_point_counts_match") is True
                and rebuilt.get("original_live_map_equality") is None,"offline reconstruction missing")
        require(digest(Path(rebuilt["map_path"])) == rebuilt["map_sha256"],"offline oracle map changed")
    else:
        require(fresh.get("forensic_raw_recovery") is False
                and fresh.get("original_live_replay_exact_match") is True,"straight requires full live/replay equality")
        owner = read_json(source.parent/"owner_plan.json")
        require(owner.get("rgb_warning_detector") == capture_qa.glyphs.VERSION,"straight requires v2 detector owner")
    records = read_json(source/"records.json")
    require(len(records) == 240,"all 240 native frames required")
    return source,records,fresh


def safe_npz_state(model, path, expected_sha, torch):
    """HH_261001 - Data-only loader: exact names, float32, shape, finite values, hash."""
    require(digest(path) == expected_sha,"weights hash mismatch")
    expected, state = model.state_dict(), {}
    with np.load(path,allow_pickle=False) as archive:
        require(len(archive.files) == len(set(archive.files))
                and set(archive.files) == set(expected),"weight key ABI mismatch")
        for name in archive.files:
            value = archive[name]
            require(value.dtype == np.float32 and tuple(value.shape) == tuple(expected[name].shape)
                    and bool(np.isfinite(value).all()),"weight tensor ABI mismatch")
            state[name] = torch.from_numpy(value.copy())
    model.load_state_dict(state,strict=True)
    model.eval()
    for parameter in model.parameters():
        parameter.requires_grad_(False)
    return model


def load_models():
    import torch
    torch.set_num_threads(2)
    torch.set_num_interop_threads(2)
    archived = RUN/"source"
    sys.path.insert(0,str(archived))
    first = importlib.import_module("map_perception")
    second = importlib.import_module("map_context_recovery")
    for module in (first,second):
        require(Path(module.__file__).resolve().parent == archived,"model import escaped frozen archive")
    require(tuple(first.LABELS) == LABELS,"label order ABI differs")
    models = {arm:safe_npz_state(cls(),RUN/arm/"weights.npz",WEIGHTS_SHA[arm],torch)
              for arm,cls in (("A",first.MapPerception),("B",second.MapContextRecovery))}
    return torch,models


def targets(semantic):
    require(semantic.shape == (360,640,4) and semantic.dtype == np.uint8,"raw semantic ABI differs")
    target = np.zeros((360,640),dtype=np.uint8)
    for tag,label in RAW_LABELS.items():
        target[semantic[...,2] == tag] = LABELS.index(label)
    return target


def confusion(target, prediction):
    require(target.shape == prediction.shape and target.dtype == prediction.dtype == np.uint8
            and int(target.max()) < 7 and int(prediction.max()) < 7,"prediction/target ABI differs")
    encoded = target.astype(np.int64)*7+prediction
    return np.bincount(encoded.ravel(),minlength=49).reshape(7,7)


def metrics(matrix):
    actual, predicted, true_positive = matrix.sum(1),matrix.sum(0),np.diag(matrix)
    union = actual+predicted-true_positive
    return {"confusion":matrix.tolist(),"pixel_accuracy":float(true_positive.sum()/matrix.sum()),
        "class_metrics":{label:{"ground_truth_pixels":int(actual[i]),"predicted_pixels":int(predicted[i]),
            "true_positive_pixels":int(true_positive[i]),"false_positive_pixels":int(predicted[i]-true_positive[i]),
            "false_negative_pixels":int(actual[i]-true_positive[i]),"union_pixels":int(union[i]),
            "iou":float(true_positive[i]/union[i]) if actual[i] else None,
            "ground_truth_absent":not bool(actual[i])} for i,label in enumerate(LABELS)}}


def render(scenario, index, rgb, target, prediction_a, prediction_b):
    require(rgb.shape == (360,640,3),"full native RGB required")
    canvas = Image.new("RGB",(1280,860),(18,27,40))
    draw = ImageDraw.Draw(canvas)
    font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    font = ImageFont.truetype(font_path,16)
    small = ImageFont.truetype(font_path,13)
    draw.text((10,7),f"CTrack {scenario} | fixed frame {index:03d}/239 | external diagnostic only",fill="white",font=font)
    panels = (("Original lit RGB (postprocess=true)",rgb),("Oracle semantic target",COLORS[target]),
              ("A: fixed continuation model",COLORS[prediction_a]),("B: fixed spatial-context model",COLORS[prediction_b]))
    for (x,y),(caption,pixels) in zip(((0,35),(640,35),(0,420),(640,420)),panels):
        draw.text((x+8,y),caption,fill="white",font=font)
        canvas.paste(Image.fromarray(pixels),(x,y+22))
    for i,label in enumerate(LABELS):
        x = 8+i*181
        draw.rectangle((x,807,x+12,819),fill=tuple(int(v) for v in COLORS[i]))
        draw.text((x+16,805),label,fill="white",font=small)
    draw.text((8,835),"Different render/rig from TRAIN; not independent certification. LEFT original owner FAIL is preserved.",fill="white",font=small)
    return canvas


def execute(args):
    prereg = validate_preregistration()
    require(args.render_review_note and len(args.render_review_note.strip()) >= 20,
            "explicit full-FOV review note for frames 0,119,239 required before predictions")
    output = (args.output or OUTPUT_ROOT/f"ctrack_{args.scenario}_fixed_maps_20261001_v1").absolute()
    require(not output.exists() and not output.is_symlink(),"fresh evaluation output required")
    require(output.is_relative_to(OUTPUT_ROOT) and not output.is_relative_to(SOURCE_ROOTS[args.scenario].parent),
            "evaluation output must be separate from immutable capture")
    qa_path = (args.qa or QA_PATHS[args.scenario]).absolute()
    source, records, qa = validate_capture(args.scenario,qa_path)
    before = {"qa_sha256":digest(qa_path),"preregistration_sha256":digest(PREREG),
        "summary_sha256":digest(source/"summary.json"),"records_sha256":digest(source/"records.json")}
    output.mkdir(parents=True)
    write_new(output/"started.json",{"created_at_utc":datetime.now(timezone.utc).isoformat(),
        "policy":prereg["policy"],"scenario":args.scenario,"source":str(source),
        "source_qa_status":qa["status"],"original_owner_status":qa["owner_provenance"]["original_owner_status"],
        "render_review":{"indices":list(VISUAL_INDICES),"note":args.render_review_note,
          "png_sha256":{str(i):records[i]["artifact_sha256"][".png"] for i in VISUAL_INDICES}},**before})
    write_new(output/"source_qa_replayed.json",qa)
    torch, models = load_models()
    matrices = {arm:np.zeros((7,7),dtype=np.int64) for arm in models}
    predictions = {arm:np.empty((240,360,640),dtype=np.uint8) for arm in models}
    times, per_frame, visuals = {arm:[] for arm in models}, [], {}
    with torch.no_grad():
        for index,row in enumerate(records):
            raw, png = source/(row["stem"]+".npz"),source/(row["stem"]+".png")
            require(digest(raw) == row["artifact_sha256"][".npz"]
                    and digest(png) == row["artifact_sha256"][".png"],"raw frame changed after QA")
            with np.load(raw,allow_pickle=False) as archive:
                require(set(archive.files) == {"rgb","depth","semantic","world_from_camera"},"capture array keys differ")
                rgb, target = archive["rgb"][...,[2,1,0]].copy(),targets(archive["semantic"])
            require(rgb.dtype == np.uint8 and rgb.shape == (360,640,3),"RGB input ABI differs")
            tensor = torch.from_numpy(rgb.transpose(2,0,1).copy()).float().unsqueeze(0)/255
            frame = {"index":index,"sensor_frame":row["frame"],"timestamp_s":row["timestamp_s"]}
            for arm,model in models.items():
                start = time.perf_counter()
                prediction = model(tensor).argmax(1)[0].numpy().astype(np.uint8)
                times[arm].append((time.perf_counter()-start)*1000)
                predictions[arm][index] = prediction
                matrix = confusion(target,prediction)
                matrices[arm] += matrix
                frame[arm] = {"confusion":matrix.tolist(),"inference_ms":times[arm][-1]}
            per_frame.append(frame)
            if index in VISUAL_INDICES:
                name = f"{args.scenario}_{index:03d}_full_fov.png"
                render(args.scenario,index,rgb,target,predictions["A"][index],predictions["B"][index]).save(output/name)
                visuals[name] = digest(output/name)
            if (index+1)%40 == 0:
                print(json.dumps({"scenario":args.scenario,"frames_per_model":index+1}),flush=True)
    artifacts = {}
    for arm in models:
        path = output/f"{arm}_all240_predictions.npz"
        with path.open("xb") as stream:
            np.savez_compressed(stream,prediction=predictions[arm])
        artifacts[path.name] = digest(path)
        require(digest(RUN/arm/"weights.npz") == WEIGHTS_SHA[arm],"weights changed during inference")
    require(digest(qa_path) == before["qa_sha256"] and digest(PREREG) == before["preregistration_sha256"]
            and digest(source/"summary.json") == before["summary_sha256"]
            and digest(source/"records.json") == before["records_sha256"],"input receipt changed during inference")
    for row in records:
        for suffix,sha in row["artifact_sha256"].items():
            require(digest(source/(row["stem"]+suffix)) == sha,"source frame changed during inference")
    validate_preregistration()
    results = {arm:{**metrics(matrices[arm]),"frames":240,"weights_sha256":WEIGHTS_SHA[arm],
        "inference_p50_ms":float(np.quantile(times[arm],.5)),
        "inference_p95_ms":float(np.quantile(times[arm],.95)),"first_inference_ms":times[arm][0]}
        for arm in models}
    delta = {label:None if results["A"]["class_metrics"][label]["iou"] is None else
        results["B"]["class_metrics"][label]["iou"]-results["A"]["class_metrics"][label]["iou"]
        for label in LABELS}
    write_new(output/"per_frame.json",per_frame)
    result = {"schema":"portable_e2e.ctrack_fixed_maps_external_diagnostic.v1",
        "status":"DIAGNOSTIC_COMPLETE_NOT_PROMOTED","scenario":args.scenario,
        "source":str(source),"source_qa_status":qa["status"],"intended_role":ROLE,
        "original_owner_status":qa["owner_provenance"]["original_owner_status"],
        "original_live_replay_exact_match":qa["original_live_replay_exact_match"],
        "device":"cpu","torch_version":torch.__version__,"threads":2,"frames_per_model":240,
        "model_results":results,"iou_delta_b_minus_a":delta,"prediction_artifacts":artifacts,
        "per_frame_sha256":digest(output/"per_frame.json"),"full_fov_visuals":visuals,
        "source_fingerprints_unchanged":True,"model_sources":prereg["model_sources"],**before,
        "train_eligible":False,"dataset_admitted":False,"model_promotion":False,
        "independent_dev_approved":False,"full_pipeline_10hz_verified":False,
        "fit_steps":0,"limitations":list(LIMITATIONS)}
    write_new(output/"summary.json",result)
    return {"status":result["status"],"scenario":args.scenario,"output":str(output),"iou_delta_b_minus_a":delta}


def main():
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--preregister",action="store_true")
    parser.add_argument("--execute",action="store_true")
    parser.add_argument("--scenario",choices=sorted(SOURCE_ROOTS))
    parser.add_argument("--qa",type=Path)
    parser.add_argument("--output",type=Path)
    parser.add_argument("--render-review-note",help="Attest full-FOV RGB review of fixed frames 0,119,239")
    args = parser.parse_args()
    require(not(args.preregister and args.execute),"choose registration or execution")
    if args.preregister:
        result = preregister()
    elif args.execute:
        require(args.scenario is not None,"scenario required")
        result = execute(args)
    else:
        validate_preregistration()
        result = {"status":"FROZEN_MODEL_SOURCE_PREFLIGHT_PASS","predictions_made":False}
    print(json.dumps(result,sort_keys=True),flush=True)


if __name__ == "__main__":
    main()
