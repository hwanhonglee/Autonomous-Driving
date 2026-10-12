#!/usr/bin/env python3
"""HH_261001 - Frozen paired appearance-only continuation on leased personal GPU0."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import sys
import time

sys.dont_write_bytecode = True
try:
    from . import map_appearance_matched_contract_20261001 as contract
except ImportError:
    import map_appearance_matched_contract_20261001 as contract
import numpy as np

WORKSPACE = Path("/home/pro6000/personal/hwanhong/portable_e2e")
SOURCE = WORKSPACE/"source_worktrees"/contract.NAME
DATA = WORKSPACE.parent/"dataset"/contract.NAME
OUTPUT = WORKSPACE/"runs/diagnostics"/contract.NAME
EVAL_DATA = WORKSPACE.parent/"dataset/map_town02_p00_route_holdout_20260929_v1"
# HH_261001 - Exact preexisting read-only known DEV/holdout; never opened during fitting.
EVAL_MANIFEST_SHA = "f4764b9aa9d015f3d136a145f6194dd60bda8900c27dd2423de6145b9f472834"
EVAL_FILES = {"dev_town01":(118,"417c60f924abb044617e2aba5b1afd70b83f5da7f905e54513e9c220172ba4b3"),
    "holdout_town02_p02":(232,"858f92ccd874c154576d3cf6a85132508801a22671baf6d0c07abe3936eb16cb")}
GPU_UUID = "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
GUARD_SHA = "4de567323d3964ec877da831ec2acb08e445fe77c9da659f052edcf52ca53484"
PAIR_BUDGET_SECONDS = 1800


def environment_contract():
    contract.require(os.uname().nodename == "pro6000" and Path(sys.prefix) == WORKSPACE/"venvs/py312",
                     "approved personal host and existing py312 venv required")
    contract.require(Path.cwd() == SOURCE and Path(__file__).resolve().parent == SOURCE,
                     "exact frozen personal source directory required")
    required = {"CUDA_VISIBLE_DEVICES":GPU_UUID,"CUBLAS_WORKSPACE_CONFIG":":4096:8",
        "PYTHONNOUSERSITE":"1","OMP_NUM_THREADS":"16","MKL_NUM_THREADS":"16"}
    contract.require(all(os.environ.get(key) == value for key,value in required.items()),
                     "deterministic single physical GPU0 environment differs")
    contract.require(not any(os.environ.get(key) for key in
        ("PYTHONPATH","PYTHONHOME","PYTHONSTARTUP","PYTHONUSERBASE")),"Python path override forbidden")
    for path in (WORKSPACE,SOURCE,DATA,EVAL_DATA,OUTPUT.parent):
        contract.require(path.is_dir() and path.resolve() == path,"canonical personal directory required: "+str(path))
    contract.require(not OUTPUT.exists() and not OUTPUT.is_symlink(),"fresh output required; no resume")


def source_contract(root,expected_source_sha,expected_data_sha,expected_registry_sha):
    contract.require(contract.digest(root/"bundle.json") == expected_source_sha,"source bundle changed")
    bundle = contract.read_json(root/"bundle.json")
    expected = {"train_map_appearance_matched_20261001.py","map_appearance_matched_contract_20261001.py",
        "map_perception.py","map_context_recovery.py","train_map_photometric_pilot.py","plan.md"}
    contract.require(bundle.get("schema") == "portable_e2e.map_appearance_source.v1"
        and bundle.get("study_id") == contract.NAME and set(bundle.get("files",{})) == expected
        and bundle.get("data_manifest_sha256") == expected_data_sha
        and bundle.get("study_registry_sha256") == expected_registry_sha,"source/data/registry binding differs")
    for name,sha in bundle["files"].items():
        contract.require(contract.digest(root/name) == sha,"frozen source changed: "+name)
    for name,sha in {**contract.ARCHITECTURE_SHA,"train_map_photometric_pilot.py":GUARD_SHA}.items():
        contract.require(bundle["files"][name] == sha,"original architecture/guard changed")
    return bundle


def load_weights(model,path,expected_sha):
    """HH_261001 - No pickle loading; exact finite float32 learned-tensor ABI."""
    import torch
    contract.require(contract.digest(path) == expected_sha,"parent/endpoint weight bytes changed")
    template = model.state_dict()
    with np.load(path,allow_pickle=False) as data:
        contract.require(set(data.files) == set(template),"learned-tensor keys differ")
        state = {}
        for name,tensor in template.items():
            value = data[name]
            contract.require(value.shape == tuple(tensor.shape) and value.dtype == np.float32
                and bool(np.isfinite(value).all()),"learned-tensor ABI differs: "+name)
            state[name] = torch.from_numpy(value.copy())
    model.load_state_dict(state,strict=True)
    contract.require(all(parameter.requires_grad for parameter in model.parameters()),"frozen parameter in TRAIN model")


def batch_arrays(arrays,indices,boxes,arm):
    contract.require(arm in ("control","appearance"),"unknown appearance arm")
    original_rgb,original_target,pair_false,pair_true,pair_target = arrays
    pairs = []
    for index,box in zip(indices,boxes):
        index = int(index)
        if index < contract.ORIGINAL_COUNT:
            rgb,target = original_rgb[index],original_target[index]
        else:
            index -= contract.ORIGINAL_COUNT
            rgb = (pair_false if arm == "control" else pair_true)[index]
            target = pair_target[index]
        pairs.append(contract.apply_crop(rgb,target,box))
    return np.stack([pair[0] for pair in pairs]),np.stack([pair[1] for pair in pairs])


def confusion_metrics(confusion):
    actual,predicted = confusion.sum(1),confusion.sum(0)
    diagonal = np.diag(confusion)
    union = actual+predicted-diagonal
    return {"confusion":confusion.tolist(),"target_pixels":dict(zip(contract.LABELS,actual.tolist())),
        "predicted_pixels":dict(zip(contract.LABELS,predicted.tolist())),
        "false_positive_pixels":dict(zip(contract.LABELS,(predicted-diagonal).tolist())),
        "iou":{label:float(diagonal[i]/union[i]) if actual[i] else None for i,label in enumerate(contract.LABELS)},
        "pixel_accuracy":float(diagonal.sum()/confusion.sum()),"absent_target_class_iou":"null; false positives retained"}


def evaluate_endpoints(arm_info,model_class,guard):
    """HH_261001 - Both immutable final endpoints must exist before any evaluation arrays open."""
    import torch
    contract.require(set(arm_info) == {"control","appearance"},"both final endpoints required before DEV")
    for arm,info in arm_info.items():
        contract.require(info["steps"] == contract.STEPS
            and contract.digest(OUTPUT/arm/"weights.npz") == info["weights_sha256"],"endpoint incomplete or changed")
    contract.require(contract.digest(EVAL_DATA/"manifest.json") == EVAL_MANIFEST_SHA,"known evaluation manifest changed")
    results = {}
    for split,(count,sha) in EVAL_FILES.items():
        path = EVAL_DATA/(split+".npz")
        contract.require(contract.digest(path) == sha,"known DEV/holdout bytes changed")
        with np.load(path,allow_pickle=False) as data:
            contract.require(set(data.files) == {"rgb","target"},"known evaluation keys differ")
            rgb,target = data["rgb"],data["target"]
        contract.require(rgb.shape == (count,360,640,3) and target.shape == (count,360,640)
            and rgb.dtype == target.dtype == np.uint8 and int(target.max()) < 7,"known evaluation ABI differs")
        results[split] = {}
        for arm,info in arm_info.items():
            model = model_class()
            load_weights(model,OUTPUT/arm/"weights.npz",info["weights_sha256"])
            model.to("cuda:0").eval()
            predictions = np.empty_like(target)
            confusion = np.zeros((7,7),dtype=np.int64)
            with torch.no_grad():
                for begin in range(0,count,contract.BATCH):
                    x = torch.tensor(rgb[begin:begin+contract.BATCH].transpose(0,3,1,2).copy(),dtype=torch.float32,device="cuda:0")/255
                    pred = model(x).argmax(1).cpu().numpy().astype(np.uint8)
                    predictions[begin:begin+len(pred)] = pred
                    encoded = target[begin:begin+len(pred)].astype(np.int64)*7+pred
                    confusion += np.bincount(encoded.ravel(),minlength=49).reshape(7,7)
            with (OUTPUT/arm/(split+"_predictions.npz")).open("xb") as stream:
                np.savez_compressed(stream,prediction=predictions)
            results[split][arm] = {"count":count,"source_sha256":sha,**confusion_metrics(confusion)}
            del model
        contract.require(contract.digest(path) == sha,"evaluation bytes changed during inference")
    guard.write_new(OUTPUT/"known_evaluation.json",results)
    return results


def run(args):
    environment_contract()
    bundle = source_contract(SOURCE,args.expected_source_sha256,args.expected_data_sha256,args.expected_registry_sha256)
    manifest,arrays,indices,crops = contract.load_package(DATA,args.expected_data_sha256,args.expected_registry_sha256)
    contract.require(contract.digest(SOURCE/"plan.md") == contract.read_json(DATA/"study_train_registry.json")["training_plan_sha256"],
                     "registered optimization plan differs")
    receipt = {"study_id":contract.NAME,"data_manifest_sha256":args.expected_data_sha256,
        "source_bundle_sha256":args.expected_source_sha256,"study_registry_sha256":args.expected_registry_sha256,
        "parent_weights_sha256":contract.PARENT_SHA,"schedule_contract":manifest["schedule_contract"],
        "steps_per_arm":contract.STEPS,"batch":contract.BATCH,"common_class_weights":manifest["common_class_weights"],
        "sole_factor":"new480 paired RGB false versus true; original729 unchanged",
        "ctrack_train_eligible":False,"model_promotion":False,"independent_dev_sites":0}
    if args.validate_only:
        print(json.dumps({"status":"CPU_PREFLIGHT_PASS",**receipt}),flush=True)
        return
    # HH_261001 - Import model/guard only after their complete frozen source hashes pass.
    import torch
    import train_map_photometric_pilot as guard
    from map_context_recovery import MapContextRecovery
    from map_perception import semantic_loss
    torch.set_num_threads(16)
    torch.set_num_interop_threads(1)
    with guard.gpu0_lease():
        gpu = guard.preflight_gpu()
        OUTPUT.mkdir()
        for name in (*bundle["files"],"bundle.json"):
            guard.copy_new(SOURCE/name,OUTPUT/"source"/name)
        for name in ("manifest.json","study_train_registry.json","paired_samples.json"):
            guard.copy_new(DATA/name,OUTPUT/"inputs"/name)
        guard.write_new(OUTPUT/"started.json",{**receipt,"gpu":gpu,"created_at_utc":datetime.now(timezone.utc).isoformat()})
        deadline = time.monotonic()+PAIR_BUDGET_SECONDS
        arm_info = {}
        weights = torch.tensor(manifest["common_class_weights"],dtype=torch.float32,device="cuda:0")
        for arm in ("control","appearance"):
            guard.seed_everything(contract.SEED)
            model = MapContextRecovery()
            load_weights(model,DATA/"parent_weights.npz",contract.PARENT_SHA)
            initial_sha = guard.state_digest(model)
            contract.require(sum(p.numel() for p in model.parameters()) == 245831,"same B architecture required")
            model.to("cuda:0").train()
            optimizer = torch.optim.AdamW(model.parameters(),lr=contract.LR,weight_decay=contract.WEIGHT_DECAY)
            contract.require(len(optimizer.state) == 0,"fresh optimizer required")
            arm_dir = OUTPUT/arm
            arm_dir.mkdir()
            started = time.monotonic()
            samples_seen,crops_seen = hashlib.sha256(),hashlib.sha256()
            total = 0.
            with (arm_dir/"history.jsonl").open("x") as log:
                for step in range(contract.STEPS):
                    contract.require(time.monotonic() < deadline,"bounded pair wall-time budget exhausted")
                    ids,boxes = indices[step],crops[step]
                    samples_seen.update(ids.tobytes()); crops_seen.update(boxes.tobytes())
                    rgb,target = batch_arrays(arrays,ids,boxes,arm)
                    x = torch.tensor(rgb.transpose(0,3,1,2).copy(),dtype=torch.float32,device="cuda:0")/255
                    y = torch.tensor(target.astype(np.int64),device="cuda:0")
                    optimizer.zero_grad(set_to_none=True)
                    loss = semantic_loss(model(x),y,weights)
                    contract.require(bool(torch.isfinite(loss)),"nonfinite training loss")
                    loss.backward()
                    norm = torch.nn.utils.clip_grad_norm_(model.parameters(),5.)
                    contract.require(bool(torch.isfinite(norm)),"nonfinite training gradient")
                    optimizer.step()
                    total += float(loss.detach())
                    if (step+1)%100 == 0:
                        row = {"arm":arm,"step":step+1,"mean_loss_100":total/100,"wall_seconds":time.monotonic()-started}
                        log.write(json.dumps(row)+"\n"); log.flush()
                        print(json.dumps(row),flush=True)
                        total = 0.
            torch.cuda.synchronize()
            checkpoint = {"model_state_dict":model.state_dict(),"optimizer_state_dict":optimizer.state_dict(),
                "labels":contract.LABELS,"steps":contract.STEPS,"parent_weights_sha256":contract.PARENT_SHA,
                "study_id":contract.NAME,"arm":arm,"deployable":False}
            with (arm_dir/"checkpoint.pt").open("xb") as stream:
                torch.save(checkpoint,stream)
            with (arm_dir/"weights.npz").open("xb") as stream:
                np.savez_compressed(stream,**{name:value.detach().cpu().numpy() for name,value in model.state_dict().items()})
            info = {"steps":contract.STEPS,"initial_state_sha256":initial_sha,"parameter_count":245831,
                "training_wall_seconds":time.monotonic()-started,"sample_sequence_sha256":samples_seen.hexdigest(),
                "crop_sequence_sha256":crops_seen.hexdigest(),"weights_sha256":contract.digest(arm_dir/"weights.npz"),
                "checkpoint_sha256":contract.digest(arm_dir/"checkpoint.pt"),"optimizer_initial_state_entries":0}
            contract.require(info["sample_sequence_sha256"] == manifest["schedule_contract"]["indices_sha256"]
                and info["crop_sequence_sha256"] == manifest["schedule_contract"]["crops_sha256"],"actual exposure sequence differs")
            guard.write_new(arm_dir/"training.json",info)
            arm_info[arm] = info
            del model,optimizer,checkpoint,x,y,loss
            torch.cuda.empty_cache()
        contract.require(arm_info["control"]["initial_state_sha256"] == arm_info["appearance"]["initial_state_sha256"],
                         "matched initialization differs")
        guard.write_new(OUTPUT/"fit_complete.json",{"status":"BOTH_FIXED_ENDPOINTS_SAVED_BEFORE_DEV","arms":arm_info})
        evaluated = evaluate_endpoints(arm_info,MapContextRecovery,guard)
        source_contract(SOURCE,args.expected_source_sha256,args.expected_data_sha256,args.expected_registry_sha256)
        for name,ref in manifest["files"].items():
            contract.require(contract.digest(DATA/name) == ref["sha256"],"training input changed during fitting")
        contract.require(contract.digest(DATA/"manifest.json") == args.expected_data_sha256
            and contract.digest(DATA/"study_train_registry.json") == args.expected_registry_sha256
            and contract.digest(DATA/"paired_samples.json") == manifest["paired_samples_sha256"],"registration changed during fitting")
        result = {"status":"MATCHED_PAIR_COMPLETE_DIAGNOSTIC_NOT_PROMOTED",**receipt,"arms":arm_info,
            "known_evaluation":evaluated,"dev_used_during_optimization":False,"source_and_data_unchanged":True}
        guard.write_new(OUTPUT/"summary.json",result)
        print(json.dumps({"status":result["status"],"output":str(OUTPUT)}),flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    for name in ("data","source","registry"):
        parser.add_argument("--expected-"+name+"-sha256",required=True)
    parser.add_argument("--validate-only",action="store_true")
    run(parser.parse_args())
