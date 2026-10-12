#!/usr/bin/env python3
"""HH_261002 - Continue the current map model in a fixed TRAIN-sampler-only A/B study.

Code preparation and a successful inventory grant no optimizer permission.  A
separately reviewed, externally pinned authorization is required.  Both fixed
final4000 exports must exist before previously exposed evaluation arrays open.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import sys
import time

import numpy as np

sys.dont_write_bytecode = True
try:
    from . import map_train_hardnegative_inventory_sampler_20261002 as mining
    from . import train_map_paired_balanced_20261002 as previous
except ImportError:
    import map_train_hardnegative_inventory_sampler_20261002 as mining
    import train_map_paired_balanced_20261002 as previous

contract, require = mining.data_contract, mining.require
NAME = "map_hardnegative_continuation_20261002_v1"
SCHEMA = "portable_e2e.map_hardnegative_continuation.20261002.v1"
AUTHORIZATION_SCHEMA = SCHEMA + ".authorization"
WORKSPACE, DATA = mining.WORKSPACE, mining.DATA
SOURCE = WORKSPACE / "source_worktrees" / NAME
OUTPUT = WORKSPACE / "runs/diagnostics" / NAME
MINING_OUTPUT = mining.OUTPUT
MINING_SHA = {
    "inventory.json": "c785ce2954c461ba6373d56123ab2908817a517a4fa121d77235ecdb0ba259dd",
    "started.json": "2c03f3c4c447bfc37f0a6bcf6448404b153c2605076cc528455cd3224c9c0adc",
    "authorization.json": "f85a0687750bcabd88760c42d12111a4558a78e1da3533d3577ac41d4e2f76e3",
    "records.jsonl": "7f6df41b42c057ec0d25b7cedf4aaa277391eefb07b3a7815eb2590868fd8fcc",
    "parent_TRAIN_predictions.npz": "aa4c786f9bfa0cc063683b3098ecd6f5707028e07c9dba9b55f3349b8df7ea93",
    "proposed_schedule.npz": "c019fd402c9886c147d81052bd4448a336b9a8b45f9d4f3f076b00fd23eca93a",
}
MINER_SHA = "bb9f93f047cb3ffc46f60431e4c3c63a93de620e09ddbca2f1bb7900ad4fe0eb"
FROZEN_SOURCE_SHA = {**mining.FROZEN_SOURCE_SHA,
    "map_train_hardnegative_inventory_sampler_20261002.py": MINER_SHA}
SOURCE_NAMES = set(FROZEN_SOURCE_SHA) | {Path(__file__).name}
ARMS = ("control", "candidate")
STEPS, BATCH, SEED = mining.STEPS, mining.BATCH, mining.SEED
PAIR_BUDGET_SECONDS = 1800
EVALUATION_BUDGET_SECONDS = 900
INITIAL_STATE_SHA = "12d647a948008dd61410bf26d3e431118ca46b2d0c3ff65c9289f0865f40916f"


def prepared_contract():
    # HH_261002 - Fixed known-site diagnostic screens are not independent site or HD-map approval.
    return dict(schema=SCHEMA, study_id=NAME, source=str(SOURCE), output=str(OUTPUT),
        mining_output=str(MINING_OUTPUT), mining_artifact_sha256=dict(MINING_SHA),
        initial_model_weights_sha256=mining.INITIAL_MODEL_SHA, initial_model_path=str(mining.INITIAL_MODEL_PATH),
        initial_model_receipt_sha256=dict(mining.INITIAL_RECEIPT_SHA),
        dataset_origin_parent_weights_sha256=mining.PARENT_SHA,
        data_manifest_sha256=mining.DATA_MANIFEST_SHA, data_registry_sha256=mining.DATA_REGISTRY_SHA,
        physical_TRAIN_slots=1209, stratum_frame_counts=dict(small_sign=391, large_sign=193, hard_negative=145),
        arms=list(ARMS), additional_updates_per_arm=STEPS, batch=BATCH, examples_per_arm=STEPS*BATCH,
        initial_completed_updates=10000, total_lineage_updates_per_arm=14000, seed=SEED,
        parameter_count=245831, optimizer="fresh AdamW each arm", learning_rate=3e-5, weight_decay=1e-4,
        gradient_norm_cap=5., paired_fit_wall_seconds=PAIR_BUDGET_SECONDS,
        post_fit_evaluation_wall_seconds=EVALUATION_BUDGET_SECONDS,
        sole_intervention="8global versus 4global+1small_sign+1large_sign+2hard_negative full-cycle exposure",
        shared_crop_law="unchanged frozen first4000 draws; no sign-dependent reroll", RGB_views="FALSE_OR_ORIGINAL only",
        class_pixels=list(mining.CLASS_PIXELS), class_weights=list(mining.CLASS_WEIGHTS),
        architecture_unchanged=True, semantic_loss_unchanged=True, labels_changed=False,
        final_selection="fixed_final4000_each_arm_only; no early or best checkpoint selection",
        known_evaluation_files={k:dict(count=v[0],sha256=v[1]) for k,v in previous.EVAL_FILES.items()},
        evaluation_opens_only_after_both_final4000=True, known_evaluation_not_independent=True,
        exploratory_screen=dict(comparators=["parent","control"],town01_sign_precision_strict_gain=True,
            town01_other_to_sign_FP_strict_reduction=True,town01_sign_recall_max_drop=.02,
            both_known_cohorts_road_lane_light_iou_max_drop=.01,town02_sign_iou_max_drop=.01),
        native_denials_preserved=True, independent_new_dev_sites=0,
        training_authorized=False, model_promoted=False, vehicle_control_approved=False,
        main_capabilities_approved=0, subfunctions_approved=0)


def source_snapshot(root):
    contract.canonical(root)
    result = {name:contract.digest(root/name) for name in sorted(SOURCE_NAMES)}
    require(all(result[name] == sha for name,sha in FROZEN_SOURCE_SHA.items()), "frozen mining/model/helper bytes differ")
    return result


def validate_authorization(value, source):
    require(type(value) is dict and value.get("schema") == AUTHORIZATION_SCHEMA
        and value.get("source_sha256") == source
        and json.dumps(value.get("contract"),sort_keys=True,allow_nan=False)==json.dumps(prepared_contract(),sort_keys=True,allow_nan=False),
        "exact prospective current-parent/inventory/source/sampler/diagnostic declaration required")
    for key,wanted in (("research_fit_authorized",True),("native_denials_preserved",True),
                       ("DEV_or_HOLDOUT_used_for_fit",False),("automatic_model_promotion",False),
                       ("vehicle_control_approved",False)):
        require(value.get(key) is wanted,"authorization scope differs: "+key)
    require(type(value.get("reviewer")) is str and len(value["reviewer"].strip()) >= 3,
        "actual reviewer required")


def environment_contract():
    require(os.uname().nodename == "pro6000" and Path(sys.prefix) == WORKSPACE/"venvs/py312"
        and Path.cwd() == SOURCE and Path(__file__).resolve().parent == SOURCE,
        "canonical private sourceworktree/existing personal py312 required")
    expected={"CUDA_VISIBLE_DEVICES":mining.GPU_UUID,"CUBLAS_WORKSPACE_CONFIG":":4096:8",
        "PYTHONNOUSERSITE":"1","OMP_NUM_THREADS":"16","MKL_NUM_THREADS":"16"}
    require(all(os.environ.get(k)==v for k,v in expected.items())
        and not any(os.environ.get(k) for k in ("PYTHONPATH","PYTHONHOME","PYTHONUSERBASE","PYTHONSTARTUP")),
        "strict single physicalGPU0 environment required")
    for path in (SOURCE,DATA,MINING_OUTPUT,mining.INITIAL_RUN,OUTPUT.parent):
        contract.canonical(path);require(path.is_dir(),"canonical personal directory required")
    contract.canonical(OUTPUT);require(not OUTPUT.exists(),"fresh output only; no overwrite/retry/resume")
    mining.reserve(OUTPUT)


def load_inventory():
    # HH_261002 - Authenticate full successful mining and its unchanged source/parent lineage, never recompute strata on DEV.
    for relative,sha in MINING_SHA.items(): contract.payload(MINING_OUTPUT/relative,sha)
    value=contract.read_json(MINING_OUTPUT/"inventory.json",MINING_SHA["inventory.json"])
    checked=mining.validate_inventory(value)
    require(checked["stratum_frame_counts"] == prepared_contract()["stratum_frame_counts"]
        and value.get("source_and_inputs_unchanged") is True and value.get("no_optimizer") is True
        and type(value.get("physical_gpu_index")) is int and value["physical_gpu_index"] == 0
        and value.get("gpu_uuid") == mining.GPU_UUID
        and value.get("authorization_sha256") == MINING_SHA["authorization.json"]
        and value.get("record_journal_sha256") == MINING_SHA["records.jsonl"]
        and value.get("parent_TRAIN_prediction_file_sha256") == MINING_SHA["parent_TRAIN_predictions.npz"]
        and value.get("actual_parent_state_sha256") == INITIAL_STATE_SHA,"completed immutable mining lineage differs")
    source=value.get("source_sha256")
    require(type(source) is dict and set(source)==mining.SOURCE_NAMES
        and source["map_train_hardnegative_inventory_sampler_20261002.py"]==MINER_SHA
        and all(source[k]==v for k,v in mining.FROZEN_SOURCE_SHA.items()),"mining source closure differs")
    for relative,sha in source.items(): require(contract.digest(MINING_OUTPUT/"source"/relative)==sha,"archived mining source differs")
    mining.validate_authorization(contract.read_json(MINING_OUTPUT/"authorization.json",MINING_SHA["authorization.json"]),source)
    started=contract.read_json(MINING_OUTPUT/"started.json",MINING_SHA["started.json"])
    require(started.get("source_sha256")==source and started.get("no_optimizer") is True
        and started.get("no_DEV_or_HOLDOUT_opened") is True
        and started.get("initial_model_weights_sha256")==mining.INITIAL_MODEL_SHA
        and started.get("initial_parent_state_sha256")==INITIAL_STATE_SHA,"actual mining start differs")
    journal=[json.loads(line) for line in contract.payload(MINING_OUTPUT/"records.jsonl",MINING_SHA["records.jsonl"])[0].splitlines()]
    require(journal==checked["records"],"all1209 mining journal rows differ")
    for relative,sha in mining.INITIAL_RECEIPT_SHA.items():
        require(contract.digest(MINING_OUTPUT/"initial_lineage"/relative)==sha,"mining initial receipt archive differs")
    require(contract.digest(MINING_OUTPUT/"initial_model_weights.npz")==mining.INITIAL_MODEL_SHA,"mining initial numeric model differs")
    arrays=contract.read_npz(MINING_OUTPUT/"proposed_schedule.npz",MINING_SHA["proposed_schedule.npz"])
    proof=mining.schedule_proof(value,arrays)
    require(value.get("sampler_proof")==proof,"actual immutable sampler receipt differs")
    return value,arrays,proof


def verify_train_identity(value,arrays):
    original_rgb,original_target,pair_false,_,pair_target=arrays
    for i,row in enumerate(value["records"]):
        rgb,target=(original_rgb[i],original_target[i]) if i<729 else (pair_false[i-729],pair_target[i-729])
        require(mining.array_sha(rgb)==row["rgb_sha256"] and mining.array_sha(target)==row["target_sha256"],
            "mined TRAIN RGB/true label membership differs: "+str(i))


def verify_exposures(path,arm,schedule):
    require(arm in ARMS,"unknown arm")
    rows=[json.loads(line) for line in path.read_bytes().splitlines()]
    require(len(rows)==STEPS,"all4000 optimizer exposure rows required")
    index_sha,crop_sha,view_sha=hashlib.sha256(),hashlib.sha256(),hashlib.sha256()
    for i,row in enumerate(rows):
        ids,boxes,bits=schedule[arm+"_indices"][i],schedule["shared_crops"][i],schedule["shared_FALSE_view_bits"][i]
        require(type(row.get("step")) is int and row["step"]==i+1
            and row.get("physical_slot_indices")==ids.tolist() and row.get("shared_crop_boxes")==boxes.tolist()
            and row.get("actual_paired_TRUE_bits")==bits.tolist(),"actual exposure/crop/view chronology differs")
        index_sha.update(ids.tobytes());crop_sha.update(boxes.tobytes());view_sha.update(bits.tobytes())
    return dict(steps=STEPS,examples_seen=STEPS*BATCH,sample_sequence_sha256=index_sha.hexdigest(),
        crop_sequence_sha256=crop_sha.hexdigest(),view_sequence_sha256=view_sha.hexdigest(),
        physical_slot_exposures=np.bincount(schedule[arm+"_indices"].ravel(),minlength=1209).tolist(),
        full_exposure_journal_sha256=contract.digest(path))


def metrics(matrix):
    require(isinstance(matrix,np.ndarray) and matrix.shape==(7,7) and matrix.dtype==np.int64
        and bool((matrix>=0).all()) and int(matrix.sum())>0,"full49 integer confusion required")
    actual,predicted,diagonal=matrix.sum(1),matrix.sum(0),np.diag(matrix)
    result=previous.confusion_metrics(matrix)
    result.update(true_positive_pixels=dict(zip(mining.LABELS,diagonal.tolist())),
        false_negative_pixels=dict(zip(mining.LABELS,(actual-diagonal).tolist())),
        precision={k:float(diagonal[i]/predicted[i]) if predicted[i] else None for i,k in enumerate(mining.LABELS)},
        recall={k:float(diagonal[i]/actual[i]) if actual[i] else None for i,k in enumerate(mining.LABELS)},
        other_to_sign_false_positive_pixels=int(matrix[0,5]))
    return result


def diagnostic_screen(evaluation):
    checks={}
    town=evaluation["dev_town01"]
    candidate=town["candidate"]
    # HH_261002 - Compare against BOTH current parent and continuation control to isolate the sampler from extra training.
    for comparator in ("parent","control"):
        reference=town[comparator]
        for key,wanted in (("precision",0.),("recall",-.02)):
            a,b=reference[key]["traffic_sign"],candidate[key]["traffic_sign"]
            checks["Town01_sign_"+key+"_vs_"+comparator]=a is not None and b is not None and (b>a if key=="precision" else b>=a+wanted)
        checks["Town01_other_to_sign_FP_reduction_vs_"+comparator]=candidate["other_to_sign_false_positive_pixels"]<reference["other_to_sign_false_positive_pixels"]
        for split,cohort in evaluation.items():
            for label in ("road","lane_marking","traffic_light_fixture"):
                a,b=cohort[comparator]["iou"][label],cohort["candidate"]["iou"][label]
                checks[split+"_"+label+"_retention_vs_"+comparator]=a is not None and b is not None and b>=a-.01
        cohort=evaluation["holdout_town02_p02"]
        a,b=cohort[comparator]["iou"]["traffic_sign"],cohort["candidate"]["iou"]["traffic_sign"]
        checks["Town02_sign_IoU_retention_vs_"+comparator]=a is not None and b is not None and b>=a-.01
    return dict(checks=checks,passed_exploratory_only=all(checks.values()),independent_validation=False,
        model_promoted=False,vehicle_control_approved=False)


def evaluate_endpoints(arms,model_class,guard):
    import torch
    require(set(arms)==set(ARMS),"BOTH fixed final4000 exports required before known evaluation opens")
    for arm,info in arms.items():
        require(info["steps"]==STEPS and info["examples_seen"]==STEPS*BATCH
            and contract.digest(OUTPUT/arm/"weights.npz")==info["weights_sha256"],"fixed final endpoint incomplete or changed")
    contract.read_json(previous.EVAL_DATA/"manifest.json",previous.EVAL_MANIFEST_SHA)
    result,deadline={},time.monotonic()+EVALUATION_BUDGET_SECONDS
    for split,(count,sha) in previous.EVAL_FILES.items():
        path=previous.EVAL_DATA/(split+".npz");data=contract.read_npz(path,sha)
        require(set(data)=={"rgb","target"},"known diagnostic keys differ")
        rgb,target=data["rgb"],data["target"]
        require(rgb.shape==(count,360,640,3) and target.shape==(count,360,640)
            and rgb.dtype==target.dtype==np.uint8 and int(target.max())<7,"full known diagnostic ABI differs")
        result[split]={}
        for arm in ("parent",*ARMS):
            model=model_class()
            weights_path=OUTPUT/"initial_model_weights.npz" if arm=="parent" else OUTPUT/arm/"weights.npz"
            weights_sha=mining.INITIAL_MODEL_SHA if arm=="parent" else arms[arm]["weights_sha256"]
            previous.load_weights(model,weights_path,weights_sha);model.to("cuda:0").eval()
            prediction,matrix=np.empty_like(target),np.zeros((7,7),dtype=np.int64)
            with torch.no_grad():
                for begin in range(0,count,BATCH):
                    require(time.monotonic()<deadline,"bounded15minute post-fit diagnostic evaluation exhausted")
                    x=torch.tensor(rgb[begin:begin+BATCH].transpose(0,3,1,2).copy(),dtype=torch.float32,device="cuda:0")/255
                    logits=model(x);require(bool(torch.isfinite(logits).all()),"nonfinite diagnostic logits")
                    pred=logits.argmax(1).cpu().numpy().astype(np.uint8)
                    prediction[begin:begin+len(pred)]=pred
                    matrix+=np.bincount((target[begin:begin+len(pred)].astype(np.int64)*7+pred).ravel(),minlength=49).reshape(7,7)
            mining.reserve(OUTPUT)
            directory=OUTPUT/("parent_diagnostic" if arm=="parent" else arm);directory.mkdir(exist_ok=True)
            with (directory/(split+"_predictions.npz")).open("xb") as stream:np.savez_compressed(stream,prediction=prediction)
            result[split][arm]=dict(count=count,source_sha256=sha,weights_sha256=weights_sha,
                known_exposed_diagnostic_not_independent=True,**metrics(matrix))
            del model;torch.cuda.empty_cache()
        require(contract.digest(path)==sha,"known diagnostic bytes changed")
    guard.write_new(OUTPUT/"known_evaluation.json",result)
    return result


def postcheck(source,manifest,authorization,authorization_sha):
    require(source_snapshot(SOURCE)==source,"live source changed during fit")
    for name,sha in source.items():require(contract.digest(OUTPUT/"source"/name)==sha,"archived source changed")
    for name,sha in MINING_SHA.items():
        require(contract.digest(MINING_OUTPUT/name)==sha and contract.digest(OUTPUT/"mining"/name)==sha,"mining evidence changed")
    for name,sha in FROZEN_SOURCE_SHA.items():require(contract.digest(MINING_OUTPUT/"source"/name)==sha,"mining archive changed")
    mining.initial_lineage()
    require(contract.digest(OUTPUT/"initial_model_weights.npz")==mining.INITIAL_MODEL_SHA,"archived initial numeric weights changed")
    for name,sha in mining.INITIAL_RECEIPT_SHA.items():
        require(contract.digest(MINING_OUTPUT/"initial_lineage"/name)==sha
            and contract.digest(OUTPUT/"initial_lineage"/name)==sha,"initial lineage archive changed")
    for name,ref in manifest["files"].items():require(contract.digest(DATA/name)==ref["sha256"],"training payload changed")
    for name,sha in (("manifest.json",mining.DATA_MANIFEST_SHA),("study_train_registry.json",mining.DATA_REGISTRY_SHA),
        ("paired_samples.json",manifest["paired_samples_sha256"])):
        require(contract.digest(DATA/name)==sha and contract.digest(OUTPUT/"inputs"/name)==sha,"TRAIN metadata archive changed")
    registry=contract.read_json(DATA/"study_train_registry.json",mining.DATA_REGISTRY_SHA)
    for name,sha in registry["copied_provenance_files"].items():require(contract.digest(DATA/name)==sha,"source provenance changed")
    require(contract.digest(authorization)==authorization_sha and contract.digest(OUTPUT/"authorization.json")==authorization_sha,
        "fit authorization changed")


def run(args):
    environment_contract();source=source_snapshot(SOURCE)
    authorization=Path(args.authorization);auth_raw,auth_sha=contract.payload(authorization,args.authorization_sha256)
    validate_authorization(json.loads(auth_raw),source)
    value,schedule,proof=load_inventory();mining.initial_lineage()
    manifest,arrays,_,_,_=contract.load_package(DATA,mining.DATA_MANIFEST_SHA,mining.DATA_REGISTRY_SHA)
    verify_train_identity(value,arrays)
    require(manifest["common_class_pixels"]==mining.CLASS_PIXELS and manifest["common_class_weights"]==mining.CLASS_WEIGHTS,
        "unchanged TRAIN loss weights required")
    receipt={**prepared_contract(),"source_sha256":source,"authorization_sha256":auth_sha,
        "sampler_proof":proof,"training_authorized":True,"validation_used_during_optimization":False}
    if args.validate_only:
        print(json.dumps(dict(status="CPU_PREFLIGHT_PASS_NO_CUDA_OR_OUTPUT",**receipt),allow_nan=False),flush=True);return
    import torch
    import train_map_photometric_pilot as guard
    import map_context_recovery as architecture
    import map_perception as perception
    for module in (guard,architecture,perception):require(Path(module.__file__).resolve().parent==SOURCE,"provider escaped pinned source")
    torch.set_num_threads(16);torch.set_num_interop_threads(1);torch.use_deterministic_algorithms(True)
    torch.backends.cuda.matmul.allow_tf32=False;torch.backends.cudnn.allow_tf32=False
    with guard.gpu0_lease():
        gpu=guard.preflight_gpu();mining.reserve(OUTPUT);OUTPUT.mkdir()
        for name in source:guard.copy_new(SOURCE/name,OUTPUT/"source"/name)
        guard.copy_new(authorization,OUTPUT/"authorization.json")
        for name in MINING_SHA:guard.copy_new(MINING_OUTPUT/name,OUTPUT/"mining"/name)
        guard.copy_new(mining.INITIAL_MODEL_PATH,OUTPUT/"initial_model_weights.npz")
        for name in mining.INITIAL_RECEIPT_SHA:guard.copy_new(mining.INITIAL_RUN/name,OUTPUT/"initial_lineage"/name)
        for name in ("manifest.json","study_train_registry.json","paired_samples.json"):guard.copy_new(DATA/name,OUTPUT/"inputs"/name)
        guard.write_new(OUTPUT/"started.json",dict(**receipt,gpu=gpu,created_at_utc=datetime.now(timezone.utc).isoformat()))
        arms,deadline={},time.monotonic()+PAIR_BUDGET_SECONDS
        weights=torch.tensor(mining.CLASS_WEIGHTS,dtype=torch.float32,device="cuda:0")
        for arm in ARMS:
            guard.seed_everything(SEED);model=architecture.MapContextRecovery()
            previous.load_weights(model,OUTPUT/"initial_model_weights.npz",mining.INITIAL_MODEL_SHA)
            require(guard.state_digest(model)==INITIAL_STATE_SHA and sum(p.numel() for p in model.parameters())==245831,
                "same completed10000 parent state/architecture required")
            model.to("cuda:0").train();optimizer=torch.optim.AdamW(model.parameters(),lr=3e-5,weight_decay=1e-4)
            require(len(optimizer.state)==0,"fresh optimizer each arm required")
            directory=OUTPUT/arm;directory.mkdir();start,total=time.monotonic(),0.
            with (directory/"history.jsonl").open("x") as history,(directory/"exposures.jsonl").open("x") as exposures:
                for step in range(STEPS):
                    require(time.monotonic()<deadline,"bounded30minute paired optimizer budget exhausted")
                    ids,boxes,bits=schedule[arm+"_indices"][step],schedule["shared_crops"][step],schedule["shared_FALSE_view_bits"][step]
                    rgb,target,actual_bits=previous.batch_arrays(arrays,ids,boxes,bits,"control")
                    require(np.array_equal(actual_bits,bits),"unchanged FALSE-only input required")
                    x=torch.tensor(rgb.transpose(0,3,1,2).copy(),dtype=torch.float32,device="cuda:0")/255
                    y=torch.tensor(target.astype(np.int64),device="cuda:0")
                    optimizer.zero_grad(set_to_none=True);loss=perception.semantic_loss(model(x),y,weights)
                    require(bool(torch.isfinite(loss)),"nonfinite training loss");loss.backward()
                    require(all(p.grad is None or bool(torch.isfinite(p.grad).all()) for p in model.parameters()),"nonfinite parameter gradient")
                    norm=torch.nn.utils.clip_grad_norm_(model.parameters(),5.)
                    require(bool(torch.isfinite(norm)),"nonfinite aggregate gradient norm");optimizer.step()
                    exposures.write(json.dumps(dict(step=step+1,physical_slot_indices=ids.tolist(),shared_crop_boxes=boxes.tolist(),actual_paired_TRUE_bits=actual_bits.tolist()))+"\n")
                    total+=float(loss.detach())
                    if (step+1)%100==0:
                        row=dict(arm=arm,step=step+1,mean_loss_100=total/100,wall_seconds=time.monotonic()-start)
                        history.write(json.dumps(row,allow_nan=False)+"\n");history.flush();exposures.flush();print(json.dumps(row),flush=True);total=0.
            torch.cuda.synchronize();mining.reserve(OUTPUT)
            state={k:v.detach().cpu().numpy().copy() for k,v in model.state_dict().items()}
            require(all(v.dtype==np.float32 and bool(np.isfinite(v).all()) for v in state.values()),"finite final numeric state required")
            checkpoint=dict(model_state_dict=model.state_dict(),optimizer_state_dict=optimizer.state_dict(),labels=list(mining.LABELS),
                steps=STEPS,initial_completed_updates=10000,total_lineage_updates=14000,study_id=NAME,arm=arm,
                initial_model_weights_sha256=mining.INITIAL_MODEL_SHA,deployment_approved=False)
            with (directory/"checkpoint.pt").open("xb") as stream:torch.save(checkpoint,stream)
            mining.reserve(OUTPUT)
            with (directory/"weights.npz").open("xb") as stream:np.savez_compressed(stream,**state)
            info=dict(**verify_exposures(directory/"exposures.jsonl",arm,schedule),initial_state_sha256=INITIAL_STATE_SHA,
                initial_model_weights_sha256=mining.INITIAL_MODEL_SHA,initial_completed_updates=10000,total_lineage_updates=14000,
                parameter_count=245831,optimizer_initial_state_entries=0,training_wall_seconds=time.monotonic()-start,
                checkpoint_sha256=contract.digest(directory/"checkpoint.pt"),weights_sha256=contract.digest(directory/"weights.npz"))
            require(info["sample_sequence_sha256"]==proof["tensor_sha256"][arm+"_indices"]
                and info["crop_sequence_sha256"]==proof["tensor_sha256"]["shared_crops"]
                and info["view_sequence_sha256"]==proof["tensor_sha256"]["shared_FALSE_view_bits"],"actual complete exposure sequence differs")
            guard.write_new(directory/"training.json",info);arms[arm]=info
            del model,optimizer,checkpoint,state,x,y,loss;torch.cuda.empty_cache()
        guard.write_new(OUTPUT/"fit_complete.json",dict(status="BOTH_FIXED_FINAL4000_ENDPOINTS_SAVED_BEFORE_KNOWN_EVALUATION",arms=arms))
        evaluation=evaluate_endpoints(arms,architecture.MapContextRecovery,guard)
        postcheck(source,manifest,authorization,auth_sha)
        result=dict(**receipt,status="FIXED_CURRENT_PARENT_SAMPLER_AB_COMPLETE_DIAGNOSTIC_NOT_PROMOTED",arms_result=arms,
            known_evaluation=evaluation,exploratory_screen_result=diagnostic_screen(evaluation),source_and_inputs_unchanged=True)
        guard.write_new(OUTPUT/"summary.json",result)
        print(json.dumps(dict(status=result["status"],output=str(OUTPUT))),flush=True)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--mode",choices=("contract","fit"),default="contract")
    parser.add_argument("--authorization");parser.add_argument("--authorization-sha256")
    parser.add_argument("--validate-only",action="store_true")
    args=parser.parse_args(argv)
    if args.mode=="contract":print(json.dumps(prepared_contract(),indent=2,allow_nan=False));return 0
    require(args.authorization is not None and args.authorization_sha256 is not None,"fit requires external exact authorization")
    run(args);return 0


if __name__ == "__main__":raise SystemExit(main())
