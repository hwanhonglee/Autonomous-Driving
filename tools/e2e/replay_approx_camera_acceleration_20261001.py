"""HH_261001 - Approximate identical-JPEG acceleration sensitivity; never a live or exact-input counterfactual."""

import argparse
from contextlib import nullcontext
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image
import torch

from portable_e2e.contract import _canonical_route_in_base, load_contract
from portable_e2e.runtime import RuntimeCameraFrame, RuntimeInputs, build_runtime_tensors
from portable_e2e import simulation_research as research
from scripts.e2e import research_selector_phase_candidate_20261001 as selector
from scripts.e2e.collect_ah_simulation_research import state_features
from scripts.e2e.prepare_carla_common10_dataset import _optical_transform
from scripts.e2e.causal_camera_acceleration_research_20261001 import camera_acceleration
from scripts.e2e.audit_ah_speed_cap_trace_20261001 import ROOT, RUNS, rows, sha


SPECS = (
    ("AH", RUNS[0], "storage/experiments/validation/2026-10-01/ah_simulation_weights/AH_simulation.npz",
     "4841d4076fb49145643ab2895d87befcae9021ad02f08b8307e75f77be137d91",
     "5fd783def4d67aa8c2abe556187bbfaa06012b1cb2fb5bf04faad760771c9342",
     "e1558f43b89fb2e967112692f0f348f8f6e457f4bed39b3d592ba02f3cf02de1"),
    ("S0", RUNS[1], "storage/experiments/training/2026-10-01/ah_selector_phase_ab_20261001_v1/exports/S0_uniform_selector.npz",
     "e9f6e6e3d70742b1e70fc5bb6f4ac4880a6fe88c3aeea33353516cdd82ae4c6e",
     "aa9936e470a6eead603e9a65d4e1935d5d6a2aa9487391fa821312f5e9851c37",
     "a40d03571de1cab38e562f43ae34d8821f9ec395e34cdbeb4f102b27e1605572"),
    ("S1", RUNS[2], "storage/experiments/training/2026-10-01/ah_selector_phase_ab_20261001_v1/exports/S1_phase_balanced_selector.npz",
     "19c53a23b1f2e57b6d83171818f83f4e30aa267989b4a387a855178892087d8f",
     "05b06433c953d5d1d25c912222cb9b2a2fb3e56720c533d3f4600c2ad9c87d93",
     "146a7e0967703f4536d1d5379e6e36894f75ba52175e8c84f9c08c989c332a94"))


def tensor_hash(tensors):
    digest=hashlib.sha256()
    for name in sorted(tensors):
        array=tensors[name].detach().cpu().numpy()
        digest.update(name.encode());digest.update(str(array.dtype).encode());digest.update(str(array.shape).encode());digest.update(array.tobytes())
    return digest.hexdigest()


def transform_history(tensors, history, derived):
    """HH_261001 - Copy only two channels; baseline and every other tensor remain byte-identical."""
    changed=dict(tensors);changed["ego_history"]=tensors["ego_history"].clone()
    for i,state in enumerate(history):
        value=derived[state["frame"]]
        changed["ego_history"][0,i,3]=value["ax"]
        changed["ego_history"][0,i,4]=value["ay"]
    for name in tensors:
        if name != "ego_history" and not torch.equal(tensors[name],changed[name]):
            raise ValueError("counterfactual changed non-acceleration input")
    for feature in [0,1,2,5,6,7,8,9,10,11,12]:
        if not torch.equal(tensors["ego_history"][...,feature],changed["ego_history"][...,feature]):
            raise ValueError("counterfactual changed non-acceleration history channel")
    return changed


def replay(spec,output):
    label,run,weight_relative,checkpoint_sha,weights_sha,metadata_sha=spec
    directory=ROOT/"storage/experiments/validation/2026-10-01"/run/"episode.partial"
    states,cameras,commands=(rows(directory/name) for name in ("states.jsonl","camera_frames.jsonl","learned_control.jsonl"))
    manifest=json.loads((directory/"manifest.json").read_text());contract=load_contract()
    by_frame={row["frame"]:row for row in states}; camera_index={row["frame"]:i for i,row in enumerate(cameras)}
    decisions=[r for r in commands if r.get("raw_output") is not None and not r["held_between_camera_frames"]]
    if len(decisions)!=183 or any(r["source_checkpoint_sha256"]!=checkpoint_sha for r in decisions):
        raise ValueError("fixed183 raw inference/checkpoint coverage differs")
    wheelbase=manifest["coordinate_contract"]["wheelbase_m"]
    derived={};previous=None
    for camera in cameras:
        state=by_frame[camera["frame"]]
        derived[state["frame"]]=camera_acceleration(state,previous,wheelbase)
        previous=state
    calibration=[]
    for index,s in enumerate(manifest["cameras"]):
        if s["model_index"]!=index:raise ValueError("camera order differs")
        k=s["intrinsic"]
        calibration.append((k["fx"]/s["width"],k["fy"]/s["height"],k["cx"]/s["width"],k["cy"]/s["height"],*_optical_transform(s["ros_extrinsic"])[:12]))
    route=tuple((r["x"],r["y"]) for r in json.loads((directory/"route.json").read_text())["route"])
    weights=ROOT/weight_relative
    with selector.identity(checkpoint_sha) if label!="AH" else nullcontext():
        model=research.load_ah_weights(weights,weights.with_suffix(".json"),weights_sha256=weights_sha,metadata_sha256=metadata_sha)
    before_state=tensor_hash(model.state_dict())
    raw_arrays={};records=[];image_hashes={}
    for count,decision in enumerate(decisions):
        frame=decision["inference_frame"];state=by_frame[frame];ci=camera_index[frame];camera=cameras[ci]
        history=[by_frame[r["frame"]] for r in cameras[max(0,ci-9):ci+1]]
        if len(history)!=10:raise ValueError("exact ten-frame live numeric history required")
        local,_,_=_canonical_route_in_base(route,position_m=(state["x"],state["y"],state["z"]),
            orientation_xyzw=(0.,0.,math.sin(state["yaw"]/2),math.cos(state["yaw"]/2)),contract=contract,context="approximate fixed JPEG replay")
        images=[]
        for name in camera["camera_order"]:
            path=directory/camera["images"][name];image_hashes[str(path.relative_to(ROOT))]=sha(path)
            with Image.open(path) as image:rgb=np.asarray(image.convert("RGB")).copy()
            images.append(RuntimeCameraFrame(int(round(camera["source_timestamps"][name]*1e9)),rgb,"rgb8"))
        inputs=RuntimeInputs(tuple(images),tuple(calibration),tuple(state_features(r) for r in history),(True,)*10,local)
        baseline=build_runtime_tensors(inputs,model.config);original_hash=tensor_hash(baseline)
        intervention=transform_history(baseline,history,derived)
        with torch.no_grad():
            predictions=[model(*(tensors[name] for name in research.FIELDS)) for tensors in (baseline,intervention)]
        if tensor_hash(baseline)!=original_hash:raise ValueError("baseline inputs mutated")
        output_rows=[];differences={}
        for kind,prediction in zip(("jpeg_baseline","jpeg_causal_acceleration"),predictions):
            values={name:tensor[0].numpy() for name,tensor in zip(("xy","speed","logits"),prediction)}
            if not all(np.isfinite(v).all() for v in values.values()):raise ValueError("nonfinite replay output")
            for name,value in values.items():raw_arrays.setdefault(kind+"_"+name,[]).append(value)
            selected=int(values["logits"].argmax())
            output_rows.append(dict(selected=selected,selected_speed_at_0p3_mps=float(values["speed"][selected,2])))
            if kind=="jpeg_baseline":
                for name,key in (("xy","candidate_xy"),("speed","candidate_speed"),("logits","candidate_logits")):
                    error=np.abs(values[name]-np.asarray(decision["raw_output"][key],dtype=np.float32))
                    differences[name]=dict(max_abs=float(error.max()),mean_abs=float(error.mean()))
        recorded=int(np.asarray(decision["raw_output"]["candidate_logits"]).argmax())
        records.append(dict(frame=frame,current_vx_mps=state["vx"],recorded_selected=recorded,
            baseline=output_rows[0],intervention=output_rows[1],baseline_vs_recorded_output_diff=differences,
            baseline_input_tensor_sha256=original_hash,intervention_input_tensor_sha256=tensor_hash(intervention),
            original_current_ax=state["ax"],original_current_ay=state["ay"],derived_current_acceleration=derived[frame]))
        if (count+1)%50==0:print(json.dumps(dict(model=label,replayed=count+1,required=183)),flush=True)
    if before_state!=tensor_hash(model.state_dict()):raise ValueError("frozen checkpoint weights changed")
    npz_path=output/(label+"_all_replay_outputs.npz")
    with npz_path.open("xb") as stream:np.savez_compressed(stream,**{key:np.stack(value) for key,value in raw_arrays.items()})
    groups={}
    for name,group in (("all",records),("low_abs_vx_le0p5",[r for r in records if abs(r["current_vx_mps"])<=.5]),
                       ("speed1p5_to2p5",[r for r in records if 1.5<=r["current_vx_mps"]<=2.5])):
        groups[name]=dict(count=len(group),baseline_recorded_selected_agreement=sum(r["baseline"]["selected"]==r["recorded_selected"] for r in group),
            intervention_changed_selected=sum(r["baseline"]["selected"]!=r["intervention"]["selected"] for r in group),
            baseline_selected_counts={str(i):sum(r["baseline"]["selected"]==i for r in group) for i in range(12)},
            intervention_selected_counts={str(i):sum(r["intervention"]["selected"]==i for r in group) for i in range(12)},
            preview_speed_change_mean=float(np.mean([r["intervention"]["selected_speed_at_0p3_mps"]-r["baseline"]["selected_speed_at_0p3_mps"] for r in group])) if group else None)
    return dict(model=label,checkpoint_sha256=checkpoint_sha,weights_sha256=weights_sha,metadata_sha256=metadata_sha,
        model_tensors_unchanged=True,input_image_sha256=image_hashes,groups=groups,records=records,
        all_replay_outputs_npz_sha256=sha(npz_path),neural_forwards=366,
        source_sha256={name:sha(directory/name) for name in ("states.jsonl","camera_frames.jsonl","learned_control.jsonl","manifest.json","route.json")})


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("--output",type=Path,required=True);args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False);torch.set_num_threads(2)
    results=[replay(spec,args.output) for spec in SPECS]
    report=dict(schema="portable_e2e.approximate_jpeg_causal_acceleration_replay.v1",status="APPROXIMATE_OFFLINE_SENSITIVITY_ONLY",
        models=results,neural_forwards=1098,optimizer_steps=0,live_runs=0,training_data_approved=False,
        exact_raw_input_replay=False,identical_jpeg_inputs_within_each_pair=True,live_cure_established=False,
        future_labels_used_for_input_transform=False,first_observation_fallback="preserve originalax/ay if no previouscamera pose",
        source_sha256={str(path.relative_to(ROOT)):sha(path) for path in (Path(__file__),ROOT/"tools/e2e/causal_camera_acceleration_research_20261001.py")})
    with (args.output/"summary.json").open("x") as stream:json.dump(report,stream,indent=2,sort_keys=True,allow_nan=False);stream.write("\n")
    print(json.dumps(dict(status=report["status"],groups={r["model"]:r["groups"] for r in results}),indent=2))


if __name__=="__main__":main()
