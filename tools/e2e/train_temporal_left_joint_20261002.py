"""HH_261002 - Fixed LEFT347 DATA+schedule continuation; unchanged causal model and losses.

One fresh20000-update fit starts at the original GOOD joint final6000, not an
intrinsic candidate. Every B4 contains recovery, old low, raw-only legacy and
actual LEFT. This changes DATA and its exposure schedule together, so it is
not an isolated causal A/B. DEV/HOLDOUT open only after the actual final20000.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import math
import os
from pathlib import Path
import random
import sys
import time

if __name__ == "__mp_main__":
    # HH_261002 - Spawn workers configure canonical imports only; never initialize CUDA.
    import importlib.util
    _entry=Path(__file__).resolve().parents[1]/"workspace_python.py"
    _spec=importlib.util.spec_from_file_location("hh_left_joint_worker",_entry)
    _module=importlib.util.module_from_spec(_spec);_spec.loader.exec_module(_module)
    _module.configure(Path(__file__).resolve().parents[2])

import json
import numpy as np
import torch
from torch.utils.data import ConcatDataset, DataLoader
from torch.utils.data.dataloader import default_collate

from scripts.e2e import train_temporal_intrinsic_curve_pair_v2_20261002 as gpu_revision

pair=gpu_revision.pair
original,replay,previous,joint,raw_parent=pair.original,pair.replay,pair.previous,pair.joint,pair.raw_parent
require,digest,write_new,read,regular,json_sha=pair.require,pair.digest,pair.write_new,pair.read,pair.regular,pair.json_sha
ROOT=Path(__file__).resolve().parents[2]
SCHEMA="portable_e2e.temporal_actual_left_joint_continuation.20261002.v1"
EXPORT_SCHEMA="portable_e2e.temporal_actual_left_joint_export_research.20261002.v1"
AUTH_SCHEMA=SCHEMA+".prospective_authorization"
MAX_UPDATES,CHECKPOINT_INTERVAL,MAX_FIT_SECONDS=20000,1000,180*60
SEED=2026100201
LEFT_BASENAME="town01-low-speed-left45-train-20261002-v3"
LEFT_CONVERSION_SCHEMA="carla.actual_town01_left45_common10_conversion.20261002.v3"
LEFT_QUALIFICATION_SCHEMA="carla.actual_town01_left45_common10_qualification.20261002.v3"
LEFT_ADAPTER="carla_actual_town01_left347_to_common10_20261002_v3"
PARENT_HISTORY_SHA="3c1fe1e4f9e18017a2db31b5371001873dece4a2b8e78bb3cdb21f9aad5ebac3"
BATCH_DOMAINS=["actual_recovery_joint","old_low_joint","legacy_raw_only","actual_LEFT347_joint"]
LEFT_SCOPE=dict(rolling_LEFT_only=True,startup_anchors_used_for_training=False,
    STOP_or_stationary_tail_supervision=False,raw_startup_and_tail_retained_as_context_only=True,
    fixed_driving_relative_anchor_offsets=[80,772],fixed_anchor_driving_seconds=[4.,38.6],
    new_independent_site_or_sealed_TEST_claimed=False,original_V1_failed_capture_not_qualified=True)
_created_output=None


def is_sha(value):
    return type(value) is str and len(value)==64 and all(c in "0123456789abcdef" for c in value)


def left_scope_clear(value):
    return all(value.get(k) is v if type(v) is bool else value.get(k)==v for k,v in LEFT_SCOPE.items())


def source_snapshot():
    old=gpu_revision.source_snapshot()
    paths=set(replay.imported_source_closure(ROOT,[Path(__file__),ROOT/"tools/workspace_python.py",ROOT/"tools/canonical_workspace.py"]))
    paths.add(ROOT/"learning/portable_e2e/config/common_10hz_v1.contract.json")
    result={str(p.relative_to(ROOT)):digest(p) for p in sorted(paths)}
    require(len(old)==29 and len(result)==30 and all(result.get(k)==v for k,v in old.items()),
        "NEW trainer plus all29 immutable dependencies required")
    return result


def learning_rate_config():
    # HH_261002 - Preserve peaks, warmup and final rates; disclose the new fixed20k cosine horizon.
    return dict(original.learning_rate_config(),schedule="fixed20000_linear_warmup_cosine_two_groups")


def learning_rate(update,group):
    require(type(update) is int and 1<=update<=MAX_UPDATES and group in ("shared","heads"),"fixed20k LR update/group required")
    cfg=learning_rate_config();peak,final=cfg[group+"_peak"],cfg[group+"_final"]
    if update<=cfg["warmup_updates"]:return peak*(update/cfg["warmup_updates"])
    fraction=(update-cfg["warmup_updates"])/(MAX_UPDATES-cfg["warmup_updates"])
    return final+.5*(peak-final)*(1+math.cos(math.pi*fraction))


def contract(left_conversion_sha256):
    require(is_sha(left_conversion_sha256),"actual externally pinned LEFT conversion receipt required; never invent input pins")
    return dict(schema=SCHEMA,sources=source_snapshot(),actual_parent_final6000=pair.PARENT,
        actual_parent_history_sha256=PARENT_HISTORY_SHA,parent_optimizer_updates=6000,parent_known_total_optimizer_updates=12200,
        fixed_new_updates=20000,known_final_total_optimizer_updates=32200,effective_batch_size=4,examples_seen=80000,
        exposures=dict(recovery=20000,recovery_focus=10000,recovery_full=10000,old_low=20000,
            legacy=20000,legacy_curved=10000,legacy_straight=10000,LEFT=20000),
        training_episode_count=19,eligible_TRAIN_anchor_count=5879,new_LEFT_count=347,new_LEFT_full_cycles=57,new_LEFT_cycle_remainder=221,
        batch_domains=BATCH_DOMAINS,recovery_sampler="alternate signed-state8-bin focus and full9-profile cycles, independent cursors",
        legacy_sampler="alternating true curved/straight, round-robin maps, full-cycle queues",LEFT_sampler="all347 full-cycle shuffled; no quality filtering",
        training_seed=SEED,optimizer="fresh AdamW; no parent optimizer reuse",learning_rate=learning_rate_config(),
        objective=".25*(joint_recovery+joint_LEFT+joint_oldlow+.25*raw_legacy_curvature+raw_legacy_speed)",
        original_loss_functions_unchanged=True,intrinsic_loss_added=False,legacy_decoded=False,
        trainable_tensor_count=54,frozen_tensor_count=47,trainable_parameter_count=2277827,
        new_LEFT_basename=LEFT_BASENAME,new_LEFT_conversion_sha256=left_conversion_sha256,
        recovery_conversion_sha256=original.TRAIN_RECEIPT_SHA,holdout_conversion_sha256=original.HOLDOUT_RECEIPT_SHA,
        old_low_manifest_sha256=replay.LOW_MANIFEST_SHA,old_low_conversion_sha256=replay.LOW_CONVERSION_SHA,
        legacy_manifest_sha256=raw_parent.TRAIN_MANIFEST_SHA,gpu_preflight_revision=gpu_revision.GPU_REVISION,
        maximum_fit_seconds=MAX_FIT_SECONDS,postfit_evaluation_separately_timed=True,checkpoint_interval=1000,
        no_DEV_or_HOLDOUT_payload_until_verified_final=True,fixed_final_only=True,no_checkpoint_selection=True,
        retained_original_diagnostic_screens=71,LEFT_TRAIN_screens=dict(ADE_m=.1,FDE_m=.25,speed_MAE_mps=.05),
        intervention_scope="DATA+schedule+longer-update-horizon coupled; NOT isolated DATA causality",
        known_DEV_independent=False,new_LEFT_independent_validation=False,runtime_architecture_changed=False,runtime_caps_changed=False,
        labels_clipped=False,temporal_retiming=False,future_model_inputs=False,stop_intent_trained=False,
        training_only=True,simulation_only=True,model_promoted=False,vehicle_control_approved=False,
        inference_loader_authorized=False,official_main_features_approved=0,official_subfeatures_approved=0)


def validate_authorization(path,sha,proposed):
    require(is_sha(sha) and digest(path)==sha,"external prospective ROOT authorization SHA required")
    value=read(path)
    require(value.get("schema")==AUTH_SCHEMA and value.get("authorized") is True and value.get("contract")==proposed
        and value.get("contract_sha256")==json_sha(proposed) and value.get("actual_parent_history_sha256")==PARENT_HISTORY_SHA
        and type(value.get("GPU_physical_index")) is int and value["GPU_physical_index"]==0
        and value.get("model_promoted") is False and value.get("vehicle_control_approved") is False,
        "exact source/data/schedule/parent GPU0 research authority required")
    return value


def verify_LEFT_conversion(value):
    numbers=dict(episode_count=1,train_examples=347,val_examples=0,test_examples=0,true_future_points=22208,
        full_five_camera_history_examples=339,full_ten_ego_history_examples=338,formal_MAIN9_approved=0,formal_sub30_approved=0)
    flags=dict(research_training_authorized=True,raw_native_denials_overridden=False,original_pending_metadata_unchanged=True,
        labels_clipped=False,labels_or_pixels_transformed=False,synthetic_history_created=False,future_labels_used_as_model_input=False,
        exact_native_ego_navigation_and_target_reconstruction=True,actual_hypot_current_speed_no_clipping=True,
        signed_state_diagnostic_not_model_input=True,vehicle_control_approved=False,model_promoted=False,
        unselected_raw_context_preserved_in_source_manifest=True)
    require(value.get("schema")==LEFT_CONVERSION_SCHEMA and value.get("status")=="QUALIFIED_RESEARCH_TRAIN_LEFT347_COMMON10"
        and all(type(value.get(k)) is int and value[k]==v for k,v in numbers.items())
        and all(value.get(k) is v for k,v in flags.items()) and left_scope_clear(value),
        "only exact approved full347 LEFT V3 admission/native V2 TRAIN may enter the objective")
    require(all(is_sha(value.get(k)) for k in ("dataset_manifest_sha256","validation_report_sha256","qualification_receipt_sha256",
        "train_loader_fingerprint_sha256","temporal_dataset_fingerprint_sha256","signed_current_state_audit_sha256","external_ROOT_authority_sha256")),
        "LEFT manifest/validation/qualification/loader/temporal/audit/ROOT authority pins required")
    cases=value.get("per_episode_native_qualification",[])
    require(type(cases) is list and len(cases)==1 and cases[0]["split"]=="train" and type(cases[0]["selected_examples"]) is int and cases[0]["selected_examples"]==347
        and cases[0]["case_id"]=="pilot_00" and cases[0]["qualification_receipt_sha256"]==value["qualification_receipt_sha256"],
        "one exact native LEFT TRAIN identity required")
    require(all(is_sha(cases[0].get(k)) for k in ("native_manifest_sha256","source_route_sha256","case_plan_sha256",
        "source_camera_sha256","source_state_sha256")),"full authentic native LEFT provenance pins required")
    return cases[0]


def load_LEFT(args):
    root=regular(args.left_train,original.PERSONAL)
    require(root.name==LEFT_BASENAME and root.is_relative_to(original.PERSONAL/"dataset/prepared"),"exact personal LEFT prepared root required")
    conversion=root/"moving_conversion_qualification.json"
    require(digest(conversion)==args.left_conversion_sha256,"external actual LEFT conversion differs")
    value=read(conversion);case=verify_LEFT_conversion(value);pins={str(conversion):args.left_conversion_sha256}
    for name,key in (("dataset.json","dataset_manifest_sha256"),("validation_report.json","validation_report_sha256"),
            ("signed_current_state_audit.jsonl","signed_current_state_audit_sha256")):
        path=regular(root/name,original.PERSONAL);require(digest(path)==value[key],"LEFT conversion artifact differs: "+name);pins[str(path)]=value[key]
    manifest=read(root/"dataset.json")
    require(len(manifest["episodes"])==1 and manifest["episodes"][0]["manifest"]==case["manifest"],"exact one LEFT episode manifest required")
    episode=regular(root/case["manifest"],root).parent
    episode_description=read(root/case["manifest"])
    require(episode_description.get("split")=="train" and episode_description.get("episode_id")==case["episode_id"]
        and episode_description.get("source_provenance",{}).get("adapter_id")==LEFT_ADAPTER,"LEFT original TRAIN role/derived adapter identity differs")
    for name,key in (("moving_qualification.json","qualification_receipt_sha256"),("native_manifest.json","native_manifest_sha256"),
            ("native_route.json","source_route_sha256")):
        path=regular(episode/name,root);expected=value[key] if key in value else case[key]
        require(digest(path)==expected,"copied immutable LEFT evidence differs");pins[str(path)]=expected
    qualification=read(episode/"moving_qualification.json")
    require(qualification.get("schema")==LEFT_QUALIFICATION_SCHEMA and qualification.get("status")=="QUALIFIED_RESEARCH_TRAIN_LEFT347"
        and qualification.get("research_training_authorized") is True and qualification.get("raw_native_denials_overridden") is False
        and qualification.get("external_ROOT_authority_sha256")==value["external_ROOT_authority_sha256"]
        and left_scope_clear(qualification),"copied actual LEFT qualification scope differs")
    require(len(qualification.get("cases",[]))==1 and all(qualification["cases"][0][k]==case[k]
        for k in ("case_id","native_episode","native_manifest_sha256","source_route_sha256","case_plan_sha256")),"LEFT native identity substitution prohibited")
    dataset,receipt=raw_parent.load_view(root,"train",value["dataset_manifest_sha256"])
    require(len(dataset)==347 and len({r.episode_id for r in dataset.examples})==1
        and receipt["validated_fingerprint_sha256"]==value["train_loader_fingerprint_sha256"]
        and receipt["temporal_fingerprint_sha256"]==value["temporal_dataset_fingerprint_sha256"],"full actual LEFT loader/temporal fingerprint differs")
    require(all(r.valid_point_count==64 and 0<math.hypot(r.features[1],r.features[2])<=2.5
        and all(0<v<=2.5 for v in r.target_speed_mps) for r in dataset.examples),"all original LEFT v0/future64 values must fit unchanged decoder, without clipping")
    statistics=joint.cohort_statistics(dataset)
    require(statistics["causal_navigation_support_m"]["minimum"]>=18.,"LEFT causal navigation must retain original18m GO support")
    require(all(digest(path)==sha for path,sha in pins.items()),"LEFT artifacts changed while loading")
    return dataset,dict(receipt,conversion_sha256=args.left_conversion_sha256,statistics=statistics),pins


def fixed_batches(groups,strata):
    focus_keys,full_keys=sorted(groups["focus"]),sorted(groups["full"])
    require(len(focus_keys)==8 and len(full_keys)==9 and all(groups["focus"].values())
        and all(len(v)==302 for v in groups["full"].values()),"unchanged actual8 focus bins and full9 recovery profiles required")
    focus={k:replay.CycleQueue(v,SEED+100+i) for i,(k,v) in enumerate(sorted(groups["focus"].items()))}
    full={k:replay.CycleQueue(v,SEED+200+i) for i,(k,v) in enumerate(sorted(groups["full"].items()))}
    low,left=replay.CycleQueue(range(327),SEED+300),replay.CycleQueue(range(347),SEED+700)
    old={kind:{k:replay.CycleQueue(v,SEED+400+20*p+i) for i,(k,v) in enumerate(sorted(strata[kind].items()))}
        for p,kind in enumerate(("curved","straight"))}
    require(all(old.values()) and all(v for maps in old.values() for v in maps.values()),"nonempty original legacy strata required")
    counts=Counter();order=[]
    for step in range(MAX_UPDATES):
        kind="curved" if step%2==0 else "straight";domain="focus" if step%2==0 else "full"
        keys,queues=(focus_keys,focus) if domain=="focus" else (full_keys,full)
        recovery=queues[keys[counts[domain]%len(keys)]].next();counts[domain]+=1
        maps=sorted(old[kind]);legacy=old[kind][maps[counts[kind]%len(maps)]].next();counts[kind]+=1
        order.append([recovery,2718+low.next(),3045+legacy,5832+left.next()])
    return order


def sample_ids(datasets,indices):
    offsets=np.cumsum([0]+[len(d) for d in datasets]).tolist();result=[]
    for index in indices:
        require(type(index) is int and 0<=index<offsets[-1],"declared TRAIN index outside cohorts")
        pos=next(i for i in range(len(datasets)) if index<offsets[i+1]);result.append(datasets[pos].examples[index-offsets[pos]].token)
    return result


def domains(model,decoder,batch):
    # HH_261002 - LEFT substitutes the second joint domain; every frozen masked loss and floating operation order is preserved.
    terms=[joint.joint_forward(model,decoder,replay.slice_batch(batch,p,p+1))[2] for p in (0,3,1)]
    old=original.temporal_trajectory_loss_v2(raw_parent.forward(model,replay.slice_batch(batch,2,3)),replay.slice_batch(batch,2,3))
    loss=original.exact_objective(terms[0]["total"],terms[1]["total"],terms[2]["total"],old["raw_curvature"],old["raw_preferred_speed"])
    values=(terms[0]["total"],terms[2]["total"],.25*old["raw_curvature"]+old["raw_preferred_speed"],terms[1]["total"])
    return loss,terms,old,values


def preflight_gradient(parent,dataset,order,device):
    # HH_261002 - Actual TRAIN-only strict forward/backward; no optimizer, discard model and restore ALL RNG states.
    states=(torch.get_rng_state(),torch.cuda.get_rng_state(device),random.getstate(),np.random.get_state())
    try:
        model=pair.fresh_model(parent,device).train();decoder=original.DeterministicPathTimeDecoderV2().to(device)
        batch=raw_parent.move(default_collate([dataset[i] for i in order[0]]),device)
        loss,_,_,values=domains(model,decoder,batch)
        require(bool(torch.isfinite(loss)),"TRAIN preflight objective nonfinite")
        gradients={name:previous.finite_branch_gradient(value,model) for name,value in zip(BATCH_DOMAINS,values)}
        loss.backward();original.finite_clip_gradients(model)
        require(all(p.grad is not None and bool(torch.isfinite(p.grad).all()) for p in model.parameters() if p.requires_grad)
            and all(p.grad is None for p in model.parameters() if not p.requires_grad),"all54 finite trainable gradients/zero47 frozen gradients required")
        require(all(torch.equal(v.detach().cpu(),parent.state_dict()[k]) for k,v in model.state_dict().items()),"preflight must not update any of101 weights")
        return dict(status="PASS_ALL4_ACTUAL_TRAIN_STRICT_BACKWARD_NO_UPDATE",optimizer_updates=0,per_domain_gradients=gradients)
    finally:
        torch.set_rng_state(states[0]);torch.cuda.set_rng_state(states[1],device);random.setstate(states[2]);np.random.set_state(states[3])


def save_checkpoint(output,model,optimizer,step,declaration,declsha,frozen_sha):
    replay.storage_reserve(output);state=model.state_dict()
    require(len(state)==101 and original.frozen_fingerprint(state)==frozen_sha
        and all(v.dtype==torch.float32 and bool(torch.isfinite(v).all()) for v in state.values()),"full101 finite/47 byte-fixed export required")
    # HH_261002 - The complete80k sample journal stays in the declaration; exports bind its exact digest without copying it20 times.
    metadata=dict({k:v for k,v in declaration.items() if k!="fixed_sample_ids"},schema=EXPORT_SCHEMA,training_schema=SCHEMA,optimizer_updates=step,additional_optimizer_updates=step,
        maximum_optimizer_updates=MAX_UPDATES,known_total_optimizer_updates=12200+step,examples_seen=4*step,
        recovery_examples_seen=step,v4_examples_seen=step,legacy_examples_seen=step,LEFT_examples_seen=step,
        declaration_sha256=declsha,frozen_state_sha256=frozen_sha,runtime_loader_implemented=False,history_sha256=digest(output/"history.jsonl"),
        numerical_revision=dict(kernel_id=original.KERNEL_ID,strict_deterministic_algorithms=True,tf32_matrix_multiplication=False,
            finite_precision_bitwise_equivalence_claimed=False))
    checkpoint=output/"checkpoints"/("step_%06d.pt"%step);checkpoint.parent.mkdir(exist_ok=True)
    with checkpoint.open("xb") as stream:torch.save(dict(metadata,model_state_dict=state,optimizer_state_dict=optimizer.state_dict()),stream)
    export=output/"exports"/("step_%06d"%step);export.mkdir(parents=True,exist_ok=False)
    arrays={k:v.detach().cpu().numpy().copy() for k,v in state.items()};npz=export/"model_state.npz"
    with npz.open("xb") as stream:np.savez(stream,**arrays)
    metadata.update(checkpoint_sha256=digest(checkpoint),npz_sha256=digest(npz),state_tensor_shapes={k:list(v.shape) for k,v in arrays.items()},
        state_tensor_dtypes={k:str(v.dtype) for k,v in arrays.items()})
    write_new(export/"metadata.json",metadata)
    return dict(optimizer_updates=step,checkpoint_sha256=digest(checkpoint),npz_sha256=digest(npz),metadata_sha256=digest(export/"metadata.json"))


def train(args,parent,dataset,order,declaration,sources,device):
    torch.manual_seed(SEED);torch.cuda.manual_seed_all(SEED);random.seed(SEED);np.random.seed(SEED)
    model=pair.fresh_model(parent,device).train();decoder=original.DeterministicPathTimeDecoderV2().to(device)
    groups=original.optimizer_groups(model)
    for group in groups:group["lr"]=learning_rate(1,group["name"])
    optimizer=torch.optim.AdamW(groups,weight_decay=1e-4);frozen=original.frozen_fingerprint(parent.state_dict())
    options=dict(dataset=dataset,batch_sampler=order,num_workers=args.loader_workers,pin_memory=True)
    if args.loader_workers:options.update(prefetch_factor=2,persistent_workers=True,multiprocessing_context="spawn")
    loader=DataLoader(**options);started=time.monotonic();completed=0;checkpoints=[];declsha=digest(args.output/"declaration.json")
    with (args.output/"history.jsonl").open("x") as journal:
        for step,batch in enumerate(loader,1):
            if time.monotonic()-started>=MAX_FIT_SECONDS:break
            require(list(batch["sample_id"])==declaration["fixed_sample_ids"][step-1],"actual collated TRAIN sample order differs")
            batchsha=pair.batch_digest(batch);batch=raw_parent.move(batch,device);optimizer.zero_grad(set_to_none=True)
            loss,terms,old,values=domains(model,decoder,batch)
            gradients={name:previous.finite_branch_gradient(value,model) for name,value in zip(BATCH_DOMAINS,values)}
            require(bool(torch.isfinite(loss)),"joint objective nonfinite");loss.backward()
            require(all(p.grad is not None and bool(torch.isfinite(p.grad).all()) for p in model.parameters() if p.requires_grad)
                and all(p.grad is None for p in model.parameters() if not p.requires_grad),"54 finite gradients and47 no-grad required")
            norm=original.finite_clip_gradients(model)
            for group in optimizer.param_groups:group["lr"]=learning_rate(step,group["name"])
            optimizer.step();completed=step
            row=dict(optimizer_updates=step,examples_seen=4*step,indices=order[step-1],sample_ids=list(batch["sample_id"]),batch_sha256=batchsha,
                loss=float(loss.detach()),recovery=joint.loss_numbers(terms[0]),LEFT=joint.loss_numbers(terms[1]),v4=joint.loss_numbers(terms[2]),
                legacy=joint.loss_numbers(old),legacy_curved=step%2==1,recovery_focus=step%2==1,
                per_domain_gradients=gradients,preclip_gradient_norm=float(norm),learning_rates={g["name"]:g["lr"] for g in optimizer.param_groups},
                elapsed_fit_seconds=time.monotonic()-started)
            journal.write(json.dumps(row,allow_nan=False)+"\n");journal.flush()
            if step%100==0:print(json.dumps(dict(status="ACTUAL_LEFT_JOINT_CONTINUATION",**row)),flush=True)
            if step%CHECKPOINT_INTERVAL==0:checkpoints.append(save_checkpoint(args.output,model,optimizer,step,declaration,declsha,frozen))
    result=dict(optimizer_updates=completed,examples_seen=4*completed,fit_seconds=time.monotonic()-started,checkpoints=checkpoints,
        completed_fixed_endpoint=completed==MAX_UPDATES,DEV_or_HOLDOUT_read=False,model_promoted=False)
    write_new(args.output/"fit_result.json",result);replay.verify_sources(args.output,sources);return model.cpu(),result


def verify_final(output,result,parent,sources):
    require(result.get("optimizer_updates")==MAX_UPDATES and result.get("examples_seen")==80000
        and result.get("completed_fixed_endpoint") is True
        and [c["optimizer_updates"] for c in result.get("checkpoints",[])]==list(range(1000,20001,1000)),"all20 fixed checkpoint receipts required before DEV")
    final=result["checkpoints"][-1];folder=output/"exports/step_020000"
    for path,key in ((output/"checkpoints/step_020000.pt","checkpoint_sha256"),(folder/"model_state.npz","npz_sha256"),(folder/"metadata.json","metadata_sha256")):
        require(digest(path)==final[key],"actual final20000 endpoint bytes differ")
    meta=read(folder/"metadata.json")
    declaration=read(output/"declaration.json")
    require(meta.get("checkpoint_sha256")==final["checkpoint_sha256"] and meta.get("npz_sha256")==final["npz_sha256"]
        and meta.get("history_sha256")==digest(output/"history.jsonl"),"numeric export must bind actual checkpoint, NPZ and completed update journal")
    require(meta.get("schema")==EXPORT_SCHEMA and meta.get("training_schema")==SCHEMA
        and meta.get("optimizer_updates")==20000 and meta.get("additional_optimizer_updates")==20000
        and meta.get("maximum_optimizer_updates")==20000 and meta.get("known_total_optimizer_updates")==32200
        and meta.get("examples_seen")==80000 and all(meta.get(k)==20000 for k in ("recovery_examples_seen","v4_examples_seen","legacy_examples_seen","LEFT_examples_seen"))
        and meta.get("source_sha256")==sources and meta.get("declaration_sha256")==digest(output/"declaration.json")
        and meta.get("learning_rate")==learning_rate_config() and meta.get("runtime_loader_implemented") is False
        and meta.get("model_promoted") is False and meta.get("vehicle_control_approved") is False
        and all(meta.get("parent_"+role+"_sha256")==sha for role,sha in (("npz",pair.PARENT["weights_sha256"]),
            ("metadata",pair.PARENT["metadata_sha256"]),("checkpoint",pair.PARENT["checkpoint_sha256"]),("declaration",pair.PARENT["declaration_sha256"]))),
        "truthful final20k/source/currentparent/scope required")
    npz=folder/"model_state.npz";require(npz.stat().st_size<=32*1024**2,"bounded numeric101 export required")
    state=parent.state_dict()
    require(set(meta["state_tensor_shapes"])==set(meta["state_tensor_dtypes"])==set(state),"complete exact101 metadata shape/dtype keys required")
    with np.load(npz,allow_pickle=False) as arrays:
        require(len(arrays.files)==len(set(arrays.files))==101 and set(arrays.files)==set(state),"all101 exact numeric keys required")
        for name,value in state.items():
            array=arrays[name]
            require(array.dtype==np.float32 and np.isfinite(array).all() and list(array.shape)==list(value.shape)
                ==meta["state_tensor_shapes"][name] and meta["state_tensor_dtypes"][name]=="float32","finite full shape/dtype ABI differs")
            if not name.startswith(original.TRAINABLE_PREFIXES):require(np.array_equal(array,value.cpu().numpy()),"one of47 frozen weights changed")
    replay.verify_sources(output,sources)
    rows=[json.loads(line,object_pairs_hook=pair.unique_json_object) for line in (output/"history.jsonl").read_bytes().splitlines()]
    require(len(rows)==20000 and len(declaration["fixed_sample_ids"])==20000
        and json_sha(declaration["fixed_sample_ids"])==meta.get("fixed_sample_ids_sha256")
        and all(type(row["optimizer_updates"]) is int and row["optimizer_updates"]==i+1 and row["examples_seen"]==4*(i+1)
            and row["sample_ids"]==declaration["fixed_sample_ids"][i] and is_sha(row["batch_sha256"])
            and row["learning_rates"]=={group:learning_rate(i+1,group) for group in ("shared","heads")} for i,row in enumerate(rows))
        and json_sha([r["indices"] for r in rows])==declaration["fixed_schedule_sha256"],"all actual20k updates/80k samples/fixed batches/LRs must precede DEV")
    return dict(final,all101_numeric_finite=True,frozen47_byte_identical=True,all20000_actual_journal_rows_verified=True)


def evaluate(model,device,training,audits,low,legacy,strata,left,holdout,ho_audits,known):
    model=model.to(device);decoder=original.DeterministicPathTimeDecoderV2().to(device)
    result=dict(recovery_train=previous.evaluate_joint(model,decoder,training,device,audits),
        recovery_holdout=previous.evaluate_joint(model,decoder,holdout,device,ho_audits),v4=previous.evaluate_joint(model,decoder,low,device),
        legacy=previous.evaluate_legacy_raw(model,legacy,strata,device),known599=previous.evaluate_legacy_raw(model,known,replay.legacy_strata(known),device),
        LEFT=previous.evaluate_joint(model,decoder,left,device))
    model.cpu();return result


def endpoint_screens(before,after):
    old=original.endpoint_screens(before,after)
    require(before["LEFT"]["full_fixed_denominator"]==after["LEFT"]["full_fixed_denominator"]==347
        and [r["token"] for r in before["LEFT"]["rows"]]==[r["token"] for r in after["LEFT"]["rows"]],"all347 before/final LEFT anchors required")
    checks=list(old["checks"])
    for name,limit in (("mean_ade_6p4s_m",.1),("mean_fde_6p4s_m",.25),("mean_instantaneous_speed_mae_mps",.05)):
        actual=after["LEFT"][name];checks.append(dict(name="LEFT_TRAIN_resubstitution/"+name,actual=actual,threshold=limit,passed=math.isfinite(actual) and actual<=limit))
    return dict(old,fixed_endpoint_updates=20000,diagnostic_screen_count=74,retained_original_joint_screen_count=71,checks=checks,
        all_diagnostic_screens_passed=all(c["passed"] for c in checks),new_LEFT_independent_validation=False)


def parser():
    value=argparse.ArgumentParser(description=__doc__,allow_abbrev=False);value.add_argument("--mode",choices=("contract","validate-only","fit"),required=True)
    for name in ("parent-run","warm-parent-weights","warm-parent-metadata","backbone-parent-weights","backbone-parent-metadata",
            "recovery-train","recovery-conversion","recovery-holdout","holdout-conversion","left-train","output","authorization"):
        value.add_argument("--"+name,type=Path)
    value.add_argument("--left-conversion-sha256",required=True);value.add_argument("--authorization-sha256")
    value.add_argument("--parent-history-sha256",default=PARENT_HISTORY_SHA);value.add_argument("--loader-workers",type=int,choices=(0,1,2),default=2)
    return value


def run(argv=None):
    global _created_output
    args=parser().parse_args(argv);proposed=contract(args.left_conversion_sha256)
    if args.mode=="contract":print(json.dumps(proposed,indent=2,allow_nan=False));return 0
    require(Path(sys.prefix).resolve()==(original.WORKSPACE/"venvs/py312").resolve(),"existing private py312 only; no install")
    require(all(getattr(args,n) is not None for n in ("parent_run","warm_parent_weights","warm_parent_metadata","backbone_parent_weights",
        "backbone_parent_metadata","recovery_train","recovery_conversion","recovery_holdout","holdout_conversion","left_train","output","authorization")),"explicit canonical assets/output required")
    require(args.parent_history_sha256==PARENT_HISTORY_SHA,"original actual GOOD final6000 parent journal required")
    require(not args.output.exists() and ".." not in args.output.parts and not any(p.is_symlink() for p in (args.output,*args.output.parents))
        and args.output.resolve().is_relative_to((original.WORKSPACE/"runs/training").resolve()),"fresh create-only private output required")
    regular(args.authorization,original.PERSONAL);validate_authorization(args.authorization,args.authorization_sha256,proposed)
    replay.storage_reserve(args.output.parent);torch.set_num_threads(4)
    training,audits,low,legacy,strata,receipts=pair.load_train(args);left,left_receipt,left_pins=load_LEFT(args)
    datasets=(training,low,legacy,left);pair.disjoint(datasets);groups=previous.recovery_groups(training,audits)
    parent_order=original.fixed_batches(groups,strata)
    parent,pins,parent_screens=pair.verify_parent(args,parent_order,(training,low,legacy))
    order=fixed_batches(groups,strata);receipts=dict(receipts,LEFT=left_receipt);dataset=ConcatDataset(datasets)
    require(len(dataset)==6179 and len({r.episode_id for d in datasets for r in d.examples})==19,"all19 TRAIN sources/6179 total anchors retained")
    pins.update(left_pins)
    for path in (args.authorization,args.recovery_conversion,args.recovery_train/"dataset.json",replay.LOW_PATH/"dataset.json",
            replay.LOW_PATH/"moving_conversion_qualification.json",replay.LEGACY_PATH/"dataset.json"):
        regular(path,original.PERSONAL);pins[str(path)]=digest(path)
    fields=dict(original.scope_fields(),training_episode_count=19,replay_training_episode_count=10,LEFT_training_episode_count=1,
        current_training_purpose="actual_LEFT347_DATA_and_schedule_joint_continuation",parent_diagnostic_pass_count=sum(c["passed"] for c in parent_screens["checks"]),
        parent_diagnostic_screen_count=71,parent_all_diagnostic_screens_passed=parent_screens["all_diagnostic_screens_passed"])
    declaration=dict(fields,schema=SCHEMA,protocol=proposed,parent_optimizer_updates=6000,parent_known_total_optimizer_updates=12200,
        parent_npz_sha256=pair.PARENT["weights_sha256"],parent_metadata_sha256=pair.PARENT["metadata_sha256"],parent_checkpoint_sha256=pair.PARENT["checkpoint_sha256"],
        parent_declaration_sha256=pair.PARENT["declaration_sha256"],parent_history_sha256=PARENT_HISTORY_SHA,
        model_id=original.MODEL_ID,architecture_schema=original.ARCHITECTURE_SCHEMA,model_config=parent.config.to_dict(),recovery_config=parent.recovery_config.to_dict(),
        source_sha256=proposed["sources"],cohorts=receipts,training_dataset_fingerprint_sha256=json_sha(receipts),
        deferred_holdout_conversion_sha256=original.HOLDOUT_RECEIPT_SHA,holdout_payload_loaded=False,
        trainable_state_names=original.freeze_inputs(parent),frozen_state_names=sorted(set(parent.state_dict())-set(original.trainable_names(parent.state_dict()))),
        trainable_state_tensor_count=54,frozen_state_tensor_count=47,trainable_parameter_count=2277827,
        effective_batch_size=4,seed=SEED,learning_rate=learning_rate_config(),batch_domains=BATCH_DOMAINS,
        fixed_schedule_sha256=json_sha(order),fixed_sample_ids=[sample_ids(datasets,row) for row in order],
        immutable_input_sha256=pins,authorization_sha256=args.authorization_sha256)
    declaration["fixed_sample_ids_sha256"]=json_sha(declaration["fixed_sample_ids"])
    if args.mode=="validate-only":
        require(not torch.cuda.is_initialized(),"CPU validation must not initialize CUDA")
        print(json.dumps(dict(status="VALIDATED_ALL19_TRAIN_CPU_NO_CUDA_NO_OUTPUT",**declaration),allow_nan=False));return 0
    with raw_parent.gpu0_lease():
        require(os.environ.get("CUBLAS_WORKSPACE_CONFIG")==":4096:8","strict CuBLAS workspace required")
        gpu=gpu_revision.gpu_preflight();device=torch.device("cuda:0")
        torch.use_deterministic_algorithms(True);torch.backends.cudnn.deterministic=True;torch.backends.cudnn.benchmark=False
        torch.backends.cuda.matmul.allow_tf32=False;torch.backends.cudnn.allow_tf32=False
        preflight=preflight_gradient(parent,dataset,order,device);pair.verify_inputs(pins,proposed["sources"],[])
        images=pair.verify_training_images(datasets);args.output.mkdir();_created_output=args.output
        write_new(args.output/"declaration.json",declaration);joint.archive_sources(args.output,proposed["sources"])
        write_new(args.output/"started.json",dict(schema=SCHEMA,UTC=datetime.now(timezone.utc).isoformat(),PID=os.getpid(),GPU=gpu,preflight=preflight,TRAIN_images=images))
        final,result=train(args,parent,dataset,order,declaration,proposed["sources"],device)
        pair.verify_inputs(pins,proposed["sources"],[args.output])
        if not result["completed_fixed_endpoint"]:
            write_new(args.output/"summary.json",dict(schema=SCHEMA,status="INCOMPLETE_FIXED20K_NO_EVALUATION_NOT_PROMOTED",fit=result,model_promoted=False));return 2
        endpoint=verify_final(args.output,result,parent,proposed["sources"])
        # HH_261002 - No DEV/HOLDOUT payload above this point. All actual final101 evidence precedes the deferred loader.
        holdout,ho_audits,ho_receipt,known,known_receipt=pair.load_evaluation_only(args,training,low,legacy)
        sets=({r.token for r in left.examples},{r.episode_id for r in left.examples})
        require(all(not sets[0]&{r.token for r in d.examples} and not sets[1]&{r.episode_id for r in d.examples} for d in (holdout,known)),"LEFT evaluation repeated-view leakage")
        pins.update(pair.evaluation_artifact_pins(args,known_receipt,holdout,known));pair.verify_training_images((holdout,known))
        write_new(args.output/"evaluation_opened_after_final.json",dict(endpoint=endpoint,HOLDOUT=ho_receipt,known_DEV=known_receipt,checkpoint_selection=False))
        post=time.monotonic();metrics={}
        for label,model in (("parent",parent),("final",final)):
            metrics[label]=evaluate(model,device,training,audits,low,legacy,strata,left,holdout,ho_audits,known)
            for cohort,value in metrics[label].items():write_new(args.output/(label+"_"+cohort+".json"),value)
        screens=endpoint_screens(metrics["parent"],metrics["final"])
        require(pair.load_train(args)[-1]=={k:v for k,v in receipts.items() if k!="LEFT"} and load_LEFT(args)[1]==left_receipt,"all TRAIN loader/label/source fingerprints changed")
        post_ho,_,post_ho_receipt,post_known,post_known_receipt=pair.load_evaluation_only(args,training,low,legacy)
        require(post_ho_receipt==ho_receipt and post_known_receipt==known_receipt and post_ho.fingerprint_sha256==holdout.fingerprint_sha256
            and post_known.fingerprint_sha256==known.fingerprint_sha256,"full deferred evaluation images/labels changed")
        pair.verify_inputs(pins,proposed["sources"],[args.output]);final_images=pair.verify_training_images((*datasets,holdout,known));verify_final(args.output,result,parent,proposed["sources"])
        write_new(args.output/"summary.json",dict(schema=SCHEMA,status="FIXED20000_ACTUAL_LEFT_JOINT_COMPLETE_NOT_PROMOTED",fit=result,
            endpoint_diagnostic_screens=screens,all71_original_screens_retained=True,full_LEFT347_retained=True,
            postfit_evaluation_seconds=time.monotonic()-post,final_image_postcheck=final_images,model_promoted=False,vehicle_control_approved=False,
            official_main_features_approved=0,official_subfeatures_approved=0))
    return 0


def main(argv=None):
    global _created_output
    require(_created_output is None,"single non-reentrant invocation required")
    try:return run(argv)
    except Exception as error:
        if _created_output is not None:
            require(not _created_output.is_symlink(),"failure output must remain canonical")
            write_new(_created_output/"failure_receipt.json",dict(schema=SCHEMA,status="FAILED_PARTIAL_RETAINED_NOT_PROMOTED",
                exception_type=type(error).__name__,exception_message=str(error),partial_outputs_retained=True,
                GPU_lease_context_exited=True,foreign_processes_killed=False,model_promoted=False,vehicle_control_approved=False,checkpoint_selection=False))
        raise
    finally:_created_output=None


if __name__=="__main__":raise SystemExit(main())
