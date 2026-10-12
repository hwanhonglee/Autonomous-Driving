"""HH_261002 - A fixed TRAIN-only intrinsic-curve hypothesis, not a new runtime/head.

Two fresh6000-update continuations start at the actual joint final6000. They
use identical54/47 tensors, four-domain batches, raw losses, learning rates and
unmodified labels. ONLY candidate legacy-curved anchors add .05 intrinsic XY
to their existing raw domain loss. Known DEV/HOLDOUT are opened only AFTER
BOTH endpoints; neither checkpoints nor coefficients are selected on them.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import random
import sys
import time

if __name__ == "__mp_main__":
    # HH_261002 - Spawn workers reconstruct canonical imports; never initialize CUDA.
    import importlib.util
    _entry = Path(__file__).resolve().parents[1] / "workspace_python.py"
    _spec = importlib.util.spec_from_file_location("hh_intrinsic_pair_worker", _entry)
    _module = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(_module)
    _module.configure(Path(__file__).resolve().parents[2])

import numpy as np
import torch
from torch.utils.data import ConcatDataset, DataLoader
from torch.utils.data.dataloader import default_collate

from portable_e2e.intrinsic_spatial_path_loss_v1_20261002 import IntrinsicSpatialPathLossV1, SCHEMA as LOSS_SCHEMA
from portable_e2e.temporal_route_joint_export_research_20261002 import (
    OWNER_SCHEMA as PARENT_OWNER_SCHEMA, load_route_joint_research_npz, unique_json_object,
)
from scripts.e2e import train_temporal_route_joint_20261002 as original

ROOT = Path(__file__).resolve().parents[2]
replay, previous, joint, raw_parent = original.replay, original.previous, original.joint, original.raw_parent
require, digest, write_new = original.require, original.digest, original.write_new
SCHEMA = "portable_e2e.temporal_intrinsic_curve_paired_continuation.20261002.v1"
EXPORT_SCHEMA = "portable_e2e.temporal_intrinsic_curve_continuation_export_research.20261002.v1"
AUTH_SCHEMA = SCHEMA + ".prospective_authorization"
MAX_UPDATES, COEFFICIENT, MAX_ARM_FIT_SECONDS = 6000, .05, 90 * 60
PARENT = dict(weights_sha256="b24a6ddd151595f8ef2a463dc8307819e9c52dd43aec3af216c76e4d646d8625",
    metadata_sha256="a4b2c35cbc782378a5464ba8daa69ff97cea3ac388567ba260d4f2934381be04",
    checkpoint_sha256="43b861c30ec9971ad5828af5f2c9f7917356ac007446433cd2cf6e627f7b4e88",
    declaration_sha256="0041d4d229d3a9ad777e2e0d4873793b65eeab608cbeb69b83fcda01b98cc3e5")
_created_output = None


def json_sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def read(path):
    return json.loads(Path(path).read_bytes(), object_pairs_hook=unique_json_object)


def regular(path, boundary):
    path, boundary = Path(path).absolute(), Path(boundary).resolve(strict=True)
    require(".." not in path.parts and not any(p.is_symlink() for p in (path, *path.parents)), "symlink/lexical escape prohibited")
    require(path.resolve(strict=True).is_relative_to(boundary), "input escaped its declared personal boundary")
    return path.resolve(strict=True)


def source_snapshot():
    old = original.source_snapshot()
    paths = set(replay.imported_source_closure(ROOT, [Path(__file__), ROOT/"tools/workspace_python.py", ROOT/"tools/canonical_workspace.py"]))
    paths.add(ROOT/"learning/portable_e2e/config/common_10hz_v1.contract.json")
    values = {str(p.relative_to(ROOT)): digest(p) for p in sorted(paths)}
    require(all(values.get(k) == sha for k, sha in old.items()), "all26 original source bytes must be retained")
    return values


def contract():
    return dict(schema=SCHEMA, sources=source_snapshot(), actual_parent_final6000=PARENT,
        parent_optimizer_updates=6000, parent_known_total_optimizer_updates=12200,
        fixed_new_updates_per_arm=6000, effective_batch_size=4, examples_per_arm=24000,
        recovery_exposures_per_arm=12000, old_low_exposures_per_arm=6000, legacy_exposures_per_arm=6000,
        curved_legacy_exposures_per_arm=3000, straight_legacy_exposures_per_arm=3000,
        arm_order=["control", "intrinsic_curve"], training_seed=original.SEED,
        batch_domains=original.BATCH_DOMAINS, learning_rate=original.learning_rate_config(),
        optimizer="fresh AdamW per arm; no parent optimizer reuse", trainable_tensor_count=54,
        frozen_tensor_count=47, trainable_parameter_count=2277827, maximum_fit_seconds_per_arm=MAX_ARM_FIT_SECONDS,
        maximum_pair_fit_seconds=2*MAX_ARM_FIT_SECONDS, postfit_evaluation_separately_timed=True,
        objective=original.exact_objective.__doc__, original_objective=".25*(joint_focus+joint_full+joint_oldlow+.25*raw_legacy_curvature+raw_legacy_speed)",
        candidate_addition=".25*.05*intrinsic_XY, ONLY actual legacy_curved domain anchors",
        coefficient=COEFFICIENT, coefficient_selection="fixed prospective hypothesis; not DEV-tuned",
        loss_schema=LOSS_SCHEMA, loss_denominator="mean actual <=64m valid XY points per anchor, coordinate-mean Huber; then legacy domain weight .25",
        recovery_conversion_sha256=original.TRAIN_RECEIPT_SHA, holdout_conversion_sha256=original.HOLDOUT_RECEIPT_SHA,
        old_low_manifest_sha256=replay.LOW_MANIFEST_SHA, old_low_conversion_sha256=replay.LOW_CONVERSION_SHA,
        legacy_manifest_sha256=raw_parent.TRAIN_MANIFEST_SHA, old_low_temporal_fingerprint=replay.LOW_TEMPORAL_FP,
        legacy_temporal_fingerprint=replay.LEGACY_TEMPORAL_FP, existing_curved_anchor_count=328,
        existing_curved_map_counts={"C_track_1_0_7":260, "Town01":68},
        no_DEV_or_HOLDOUT_payload_until_both_final=True, known_DEV_independent=False,
        endpoint_screens=71, fixed_final_only=True, no_checkpoint_selection=True,
        same_batch_tensor_bytes_required=True, runtime_architecture_changed=False, runtime_caps_changed=False,
        labels_clipped=False, temporal_retiming=False, future_model_inputs=False,
        stop_intent_trained=False, training_only=True, simulation_only=True,
        model_promoted=False, vehicle_control_approved=False, inference_loader_authorized=False,
        official_main_features_approved=0, official_subfeatures_approved=0)


def validate_authorization(path, expected_sha, proposed):
    require(type(expected_sha) is str and len(expected_sha) == 64 and digest(path) == expected_sha, "external prospective authorization SHA required")
    value = read(path)
    require(value.get("schema") == AUTH_SCHEMA and value.get("authorized") is True
        and value.get("contract") == proposed and value.get("contract_sha256") == json_sha(proposed)
        and value.get("GPU_physical_index") == 0 and type(value.get("GPU_physical_index")) is int
        and value.get("model_promoted") is False and value.get("vehicle_control_approved") is False,
        "exact prospectively reviewed GPU0-only research contract required")
    return value


def authorize_parent_history(authorization, expected_sha):
    require(type(expected_sha) is str and len(expected_sha)==64 and all(c in "0123456789abcdef" for c in expected_sha)
        and authorization.get("actual_parent_history_sha256")==expected_sha,
        "the root authorization must bind the caller's actual final6000 update journal")


def disjoint(datasets):
    require(all(d.split == "train" for d in datasets), "loss cohorts must all be TRAIN")
    tokens, episodes = [{r.token for r in d.examples} for d in datasets], [{r.episode_id for r in d.examples} for d in datasets]
    require(all(len(t) == len(d) for t,d in zip(tokens,datasets)) and all(not a&b for i,a in enumerate(tokens) for b in tokens[i+1:])
        and all(not a&b for i,a in enumerate(episodes) for b in episodes[i+1:]), "TRAIN repeated-view/cohort leakage")


def load_train(args):
    # HH_261002 - Do not call original.load_cohorts: it eagerly opens known DEV/HOLDOUT.
    for path in (replay.LOW_PATH,replay.LEGACY_PATH,args.recovery_train,args.recovery_conversion):regular(path,original.PERSONAL)
    low, low_receipt = joint.load_train(replay.LOW_PATH, replay.LOW_MANIFEST_SHA, replay.LOW_CONVERSION_SHA)
    legacy, legacy_receipt = raw_parent.load_view(replay.LEGACY_PATH, "train", raw_parent.TRAIN_MANIFEST_SHA)
    for receipt, loader, temporal in ((low_receipt,replay.LOW_LOADER_FP,replay.LOW_TEMPORAL_FP),
            (legacy_receipt,replay.LEGACY_LOADER_FP,replay.LEGACY_TEMPORAL_FP)):
        require(receipt.get("validated_fingerprint_sha256",receipt.get("validation_fingerprint_sha256")) == loader
            and receipt["temporal_fingerprint_sha256"] == temporal, "same existing TRAIN loader/temporal fingerprint required")
    with original.moving.activated():
        training, audits, recovery_receipt = previous.load_admitted(args.recovery_train,"train",args.recovery_conversion,original.TRAIN_RECEIPT_SHA)
    disjoint((training,low,legacy)); strata = replay.legacy_strata(legacy); receipt = replay.stratum_receipt(legacy,strata)
    require(len(training)==2718 and len(low)==327 and len(legacy)==2787 and len(strata["eligible"])==2487
        and receipt["curved_by_map"] == {"C_track_1_0_7":260,"Town01":68}
        and receipt["straight_by_map"] == {"C_track_1_0_7":662,"Town01":570,"Town07":927}, "unchanged all18 TRAIN membership required")
    return training,audits,low,legacy,strata,dict(recovery=recovery_receipt,v4=low_receipt,legacy=legacy_receipt)


def curved_indices(strata):
    values = sorted(i for group in strata["curved"].values() for i in group)
    require(len(values)==len(set(values))==328 and set(values)<=set(strata["eligible"]), "all328 real curved TRAIN anchors required")
    return values


def spatial_inventory(dataset, strata):
    # HH_261002 - Frozen target construction needs no image decoding or CNN. Count original points beyond64m, never retime them.
    records=[]; loss=IntrinsicSpatialPathLossV1()
    with torch.no_grad():
        for index in curved_indices(strata):
            xy,_,_,valid=dataset._targets(dataset.examples[index])
            result=loss(torch.zeros(1,33),xy[None],valid[None])
            records.append(dict(index=index,token=dataset.examples[index].token,map=dataset.examples[index].map_id,
                original_valid_points=int(result["original_valid_point_count"][0]),assessed_points=int(result["assessed_point_count"][0]),
                outside_64m_points=int(result["valid_outside_64m_point_count"][0]),zero_chord_points=int(result["zero_chord_valid_point_count"][0]),
                nonzero_arc_anchor=result["nonzero_supervision_present"],full_actual_chord_arc_m=float(result["true_future_chord_arc_m"][0,-1])))
    require(all(r["nonzero_arc_anchor"] and r["assessed_points"]>0 for r in records), "empty intrinsic TRAIN supervision cannot advertise improvement")
    return dict(anchor_count=len(records),rows=records,**{k:sum(r[k] for r in records) for k in
        ("original_valid_points","assessed_points","outside_64m_points","zero_chord_points")},future_arc_loss_only=True,labels_clipped=False)


def expected_tokens(datasets, indices):
    examples=[d.examples for d in datasets]
    return [examples[0][i].token if i<2718 else examples[1][i-2718].token if i<3045 else examples[2][i-3045].token for i in indices]


def parent_paths(args):
    return dict(weights=args.parent_run/"exports/step_006000/model_state.npz",metadata=args.parent_run/"exports/step_006000/metadata.json",
        checkpoint=args.parent_run/"checkpoints/step_006000.pt",declaration=args.parent_run/"declaration.json")


def verify_parent(args, order, datasets):
    paths=parent_paths(args)
    for role,path in paths.items(): regular(path,original.PERSONAL); require(digest(path)==PARENT[role+"_sha256"],"actual final6000 parent differs: "+role)
    history=regular(args.parent_run/"history.jsonl",original.PERSONAL)
    require(digest(history)==args.parent_history_sha256,"external actual parent update journal SHA required")
    rows=[json.loads(line,object_pairs_hook=unique_json_object) for line in history.read_bytes().splitlines()]
    require(len(rows)==6000 and all(r["optimizer_updates"]==i+1 and r["examples_seen"]==4*(i+1)
        and r["indices"]==order[i] and r["sample_ids"]==expected_tokens(datasets,order[i]) for i,r in enumerate(rows)),
        "parent must actually complete all6000 unchanged TRAIN updates; no old3000 implicit parent")
    summary=read(args.parent_run/"summary.json")
    require(summary.get("status")=="FIXED6000_JOINT_COMPLETE_NOT_PROMOTED" and summary.get("optimizer_updates")==6000
        and summary.get("examples_seen")==24000 and summary.get("frozen47_tensors_byte_identical") is True,"parent completion journal absent")
    description=read(paths["metadata"])
    require(all(digest(ROOT/k)==sha and digest(args.parent_run/"source"/k)==sha for k,sha in description["source_sha256"].items()),
        "actual parent live/archive source closure differs")
    owner=dict(schema=PARENT_OWNER_SCHEMA,mode="RESEARCH_ONLY",owner_id="HH_261002-intrinsic-pair-parent-numeric-only",
        purpose="offline_cpu_numeric_validation_only",simulator_control_authorized=False,vehicle_control_approved=False,
        model_promoted=False,fixed_endpoint_updates=6000,**PARENT)
    model=load_route_joint_research_npz(paths["weights"],paths["metadata"],weights_sha256=PARENT["weights_sha256"],metadata_sha256=PARENT["metadata_sha256"],
        parent_weights=args.warm_parent_weights,parent_metadata=args.warm_parent_metadata,backbone_parent_weights=args.backbone_parent_weights,
        backbone_parent_metadata=args.backbone_parent_metadata,research_owner_binding=owner,source_root=ROOT)
    for path in (args.warm_parent_weights,args.warm_parent_metadata,args.backbone_parent_weights,args.backbone_parent_metadata):regular(path,original.PERSONAL)
    diagnostics=summary["endpoint_diagnostic_screens"]
    require(len(diagnostics["checks"])==71 and all(type(c["passed"]) is bool for c in diagnostics["checks"]),"all parent71 actual diagnostic outcomes required")
    return model,{str(p):digest(p) for p in (*paths.values(),history,args.parent_run/"summary.json",
        args.warm_parent_weights,args.warm_parent_metadata,args.backbone_parent_weights,args.backbone_parent_metadata)},diagnostics


def intrinsic_domain(raw, batch, is_curved, arm, loss_module):
    require(type(is_curved) is bool and arm in ("control","intrinsic_curve"),"explicit real TRAIN stratum/arm required")
    old=original.temporal_trajectory_loss_v2(raw,batch)
    base=.25*old["raw_curvature"]+old["raw_preferred_speed"]
    spatial=None
    if arm=="intrinsic_curve" and is_curved:
        spatial=loss_module(raw["curvature_knots"],batch["target_xy"],batch["target_valid"])
        require(spatial["nonzero_supervision_present"],"sampled curved anchor lacks actual intrinsic supervision")
    extra=spatial["total"] if spatial is not None else base*0.
    return (base+COEFFICIENT*extra if spatial is not None else base),old,extra,spatial


def batch_digest(batch):
    # HH_261002 - Hash the actual collated CPU tensor bytes, not merely indices or repeated-view labels.
    value=hashlib.sha256()
    for name,item in sorted(batch.items()):
        value.update(name.encode())
        if isinstance(item,torch.Tensor):
            require(item.device.type=="cpu","paired byte hash must precede device movement")
            value.update(str(item.dtype).encode());value.update(str(tuple(item.shape)).encode());value.update(item.contiguous().numpy().tobytes())
        else: value.update(json.dumps(item,sort_keys=True,allow_nan=False).encode())
    return value.hexdigest()


def domains(model,decoder,batch,is_curved,arm,loss_module):
    terms=[]
    for p in range(3): terms.append(joint.joint_forward(model,decoder,replay.slice_batch(batch,p,p+1))[2])
    old_batch=replay.slice_batch(batch,3,4);raw=raw_parent.forward(model,old_batch)
    legacy,old,extra,spatial=intrinsic_domain(raw,old_batch,is_curved,arm,loss_module)
    # HH_261002 - Preserve the frozen objective's exact floating operation order in CONTROL; do not reassociate legacy sums.
    loss=original.exact_objective(terms[0]["total"],terms[1]["total"],terms[2]["total"],old["raw_curvature"],old["raw_preferred_speed"])
    if spatial is not None:loss=loss+.25*COEFFICIENT*extra
    return loss,terms,old,extra,spatial,(terms[0]["total"],terms[1]["total"],terms[2]["total"],legacy)


def fresh_model(parent,device):
    model=type(parent)(parent.config,parent.recovery_config);model.load_state_dict(parent.state_dict(),strict=True)
    original.freeze_inputs(model);return model.to(device)


def preflight_gradient(parent,dataset,order,curves,device):
    # HH_261002 - Only actual TRAIN labels; restore RNG and discard this model before either fresh optimizer is built.
    cpu=torch.get_rng_state();cuda=torch.cuda.get_rng_state(device)
    try:
        model=fresh_model(parent,device);model.train();module=IntrinsicSpatialPathLossV1().to(device)
        batch=raw_parent.move(default_collate([dataset[i] for i in order[0]]),device)
        require(order[0][3]-3045 in curves,"first fixed preflight must be a real curved TRAIN anchor")
        decoder=original.DeterministicPathTimeDecoderV2().to(device)
        for arm in ("control","intrinsic_curve"):
            model.zero_grad(set_to_none=True)
            loss,*_,values=domains(model,decoder,batch,True,arm,module)
            require(bool(torch.isfinite(loss)),"strict CUDA TRAIN preflight loss nonfinite")
            for value in values: previous.finite_branch_gradient(value,model)
            loss.backward();original.finite_clip_gradients(model)
            require(all(p.grad is None or bool(torch.isfinite(p.grad).all()) for p in model.parameters()),"strict CUDA TRAIN preflight gradient nonfinite")
        require(all(torch.equal(value.detach().cpu(),parent.state_dict()[name]) for name,value in model.state_dict().items()),
            "forward/backward-only preflight mutated one of101 parent tensors")
        return dict(status="PASS_BOTH_STRICT_CUDA_ACTUAL_TRAIN_FORWARD_BACKWARD_NO_UPDATE",optimizer_updates=0)
    finally: torch.set_rng_state(cpu);torch.cuda.set_rng_state(cuda,device)


def save_checkpoint(output,model,optimizer,step,declaration,declaration_sha,frozen_sha,arm):
    replay.storage_reserve(output);state=model.state_dict()
    require(len(state)==101 and original.frozen_fingerprint(state)==frozen_sha
        and all(v.dtype==torch.float32 and bool(torch.isfinite(v).all()) for v in state.values()),"full101 finite/47 exact export required")
    metadata=dict(declaration,schema=EXPORT_SCHEMA,training_schema=SCHEMA,arm=arm,optimizer_updates=step,additional_optimizer_updates=step,
        maximum_optimizer_updates=6000,known_total_optimizer_updates=12200+step,examples_seen=4*step,recovery_examples_seen=2*step,
        v4_examples_seen=step,legacy_examples_seen=step,declaration_sha256=declaration_sha,frozen_state_sha256=frozen_sha,
        numerical_revision=dict(kernel_id=original.KERNEL_ID,strict_deterministic_algorithms=True,tf32_matrix_multiplication=False,
            finite_precision_bitwise_equivalence_claimed=False),runtime_loader_implemented=False)
    checkpoint=output/"checkpoints"/("step_%06d.pt"%step);checkpoint.parent.mkdir(exist_ok=True)
    with checkpoint.open("xb") as stream:torch.save(dict(metadata,model_state_dict=state,optimizer_state_dict=optimizer.state_dict()),stream)
    export=output/"exports"/("step_%06d"%step);export.mkdir(parents=True,exist_ok=False)
    arrays={k:v.detach().cpu().numpy().copy() for k,v in state.items()};npz=export/"model_state.npz"
    with npz.open("xb") as stream:np.savez(stream,**arrays)
    metadata.update(checkpoint_sha256=digest(checkpoint),npz_sha256=digest(npz),state_tensor_shapes={k:list(v.shape) for k,v in arrays.items()},
        state_tensor_dtypes={k:str(v.dtype) for k,v in arrays.items()})
    write_new(export/"metadata.json",metadata)
    return dict(optimizer_updates=step,checkpoint_sha256=digest(checkpoint),npz_sha256=digest(npz),metadata_sha256=digest(export/"metadata.json"))


def train_arm(args,arm,parent,dataset,order,curves,declaration,sources,device,control_hashes=None):
    torch.manual_seed(original.SEED);torch.cuda.manual_seed_all(original.SEED);np.random.seed(original.SEED);random.seed(original.SEED)
    output=args.output/arm;output.mkdir();write_new(output/"declaration.json",dict(declaration,arm=arm));declsha=digest(output/"declaration.json")
    joint.archive_sources(output,sources);model=fresh_model(parent,device);model.train()
    frozen_sha=original.frozen_fingerprint(parent.state_dict());decoder=original.DeterministicPathTimeDecoderV2().to(device)
    spatial=IntrinsicSpatialPathLossV1().to(device);optimizer=torch.optim.AdamW(original.optimizer_groups(model),weight_decay=1e-4)
    options=dict(dataset=dataset,batch_sampler=order,num_workers=args.loader_workers,pin_memory=True)
    if args.loader_workers:options.update(prefetch_factor=2,persistent_workers=True,multiprocessing_context="spawn")
    loader=DataLoader(**options);started=time.monotonic();checkpoints=[];hashes=[];completed=0
    with (output/"history.jsonl").open("x") as journal:
        for step,batch in enumerate(loader,1):
            if time.monotonic()-started>=MAX_ARM_FIT_SECONDS:break
            require(list(batch["sample_id"])==declaration["fixed_sample_ids"][step-1],"actual TRAIN sample order differs")
            byte_sha=batch_digest(batch)
            if control_hashes is not None:require(byte_sha==control_hashes[step-1],"paired actual tensor/crop bytes differ")
            hashes.append(byte_sha);batch=raw_parent.move(batch,device);optimizer.zero_grad(set_to_none=True)
            is_curved=order[step-1][3]-3045 in curves
            require(is_curved==(step%2==1),"unchanged alternating curved/straight replay required")
            loss,terms,old,extra,coverage,values=domains(model,decoder,batch,is_curved,arm,spatial)
            gradients={name:previous.finite_branch_gradient(value,model) for name,value in zip(original.BATCH_DOMAINS,values)}
            require(bool(torch.isfinite(loss)),"paired objective nonfinite");loss.backward()
            require(all(p.grad is None or bool(torch.isfinite(p.grad).all()) for p in model.parameters()),"paired gradients nonfinite")
            require(all(p.grad is None for name,p in model.named_parameters() if not name.startswith(original.TRAINABLE_PREFIXES)),"frozen47 gradient forbidden")
            norm=original.finite_clip_gradients(model)
            for group in optimizer.param_groups:group["lr"]=original.learning_rate(step,group["name"])
            optimizer.step();completed=step
            row=dict(optimizer_updates=step,examples_seen=4*step,indices=order[step-1],sample_ids=list(batch["sample_id"]),batch_sha256=byte_sha,
                loss=float(loss.detach()),recovery0=joint.loss_numbers(terms[0]),recovery1=joint.loss_numbers(terms[1]),v4=joint.loss_numbers(terms[2]),
                legacy=joint.loss_numbers(old),legacy_curved=is_curved,intrinsic_XY=float(extra.detach()),intrinsic_coefficient=COEFFICIENT if arm=="intrinsic_curve" and is_curved else 0.,
                intrinsic_assessed_points=int(coverage["assessed_point_count"].sum()) if coverage else 0,
                intrinsic_outside64_points=int(coverage["valid_outside_64m_point_count"].sum()) if coverage else 0,
                per_domain_gradients=gradients,preclip_gradient_norm=float(norm),learning_rates={g["name"]:g["lr"] for g in optimizer.param_groups},
                elapsed_fit_seconds=time.monotonic()-started)
            journal.write(json.dumps(row,allow_nan=False)+"\n");journal.flush()
            if step%100==0:print(json.dumps(dict(status="ACTUAL_PAIRED_CONTINUATION",arm=arm,**row)),flush=True)
            if step%500==0:checkpoints.append(save_checkpoint(output,model,optimizer,step,declaration,declsha,frozen_sha,arm))
    result=dict(arm=arm,optimizer_updates=completed,examples_seen=4*completed,fit_seconds=time.monotonic()-started,
        checkpoints=checkpoints,completed_fixed_endpoint=completed==6000,model_promoted=False,DEV_or_HOLDOUT_read=False)
    write_new(output/"fit_result.json",result);replay.verify_sources(output,sources)
    return model.cpu(),hashes,result


def load_evaluation_only(args,training,low,legacy):
    # HH_261002 - Called only after both final endpoints are independently verified present.
    for path in (args.recovery_holdout,args.holdout_conversion,replay.DEV_PATH):regular(path,original.PERSONAL)
    with original.moving.activated():
        holdout,audits,receipt=previous.load_admitted(args.recovery_holdout,"val",args.holdout_conversion,original.HOLDOUT_RECEIPT_SHA)
    known,known_receipt=raw_parent.load_view(replay.DEV_PATH,"val",raw_parent.DEV_MANIFEST_SHA)
    require(len(known)==665 and len(replay.legacy_strata(known)["eligible"])==599,"known599 full denominator unchanged")
    require(known_receipt["validated_fingerprint_sha256"]==replay.DEV_LOADER_FP and known.fingerprint_sha256==replay.DEV_TEMPORAL_FP,"known DEV identity differs")
    allsets=[{r.token for r in d.examples} for d in (training,low,legacy,holdout,known)]
    episodes=[{r.episode_id for r in d.examples} for d in (training,low,legacy,holdout,known)]
    require(all(not a&b for sets in (allsets,episodes) for i,a in enumerate(sets) for b in sets[i+1:]),"TRAIN/evaluation repeated-view leakage")
    require(previous.read(args.recovery_conversion)["full_cohort_case_provenance"]==previous.read(args.holdout_conversion)["full_cohort_case_provenance"],"same full11 qualified cohort required")
    return holdout,audits,receipt,known,known_receipt


def evaluate(model,device,training,audits,low,legacy,strata,holdout,holdout_audits,known):
    model=model.to(device);decoder=original.DeterministicPathTimeDecoderV2().to(device)
    metrics=dict(recovery_train=previous.evaluate_joint(model,decoder,training,device,audits),
        recovery_holdout=previous.evaluate_joint(model,decoder,holdout,device,holdout_audits),v4=previous.evaluate_joint(model,decoder,low,device),
        legacy=previous.evaluate_legacy_raw(model,legacy,strata,device),known599=previous.evaluate_legacy_raw(model,known,replay.legacy_strata(known),device))
    records=[];loss_module=IntrinsicSpatialPathLossV1().to(device);model.eval()
    with torch.no_grad():
        for index in curved_indices(strata):
            batch=raw_parent.move(default_collate([legacy[index]]),device);raw=raw_parent.forward(model,batch)
            value=loss_module(raw["curvature_knots"],batch["target_xy"],batch["target_valid"])
            records.append(dict(index=index,token=legacy.examples[index].token,loss=float(value["total"]),
                assessed_points=int(value["assessed_point_count"][0]),outside64_points=int(value["valid_outside_64m_point_count"][0])))
    metrics["intrinsic_legacy_TRAIN_only"]=dict(anchor_count=328,mean_XY_huber=sum(r["loss"] for r in records)/328,rows=records,independent_validation=False)
    model.cpu();return metrics


def verify_inputs(pins,sources,outputs):
    require(all(digest(path)==sha for path,sha in pins.items()),"parent/authorization/dataset immutable input changed")
    for output in outputs:replay.verify_sources(output,sources)


def verify_training_images(datasets):
    # HH_261002 - Full selected-view/history-source image integrity is rechecked after fitting, not just the images sampled last.
    pins={}
    for dataset in datasets:
        for row in dataset.examples:
            for path,sha in zip(row.camera_paths,row.camera_sha256):
                name=str(path)
                require(name not in pins or pins[name]==sha,"one image cannot have conflicting original TRAIN identities")
                pins[name]=sha
    require(pins and all(digest(regular(path,original.PERSONAL))==sha for path,sha in pins.items()),"an original TRAIN/history camera source changed")
    return dict(unique_camera_source_count=len(pins),all_source_image_sha256_match=True)


def verify_final_endpoint(output,result,parent,sources):
    # HH_261002 - A boolean fit result is not a completed model. Verify actual immutable final files before opening any DEV.
    require(result.get("optimizer_updates")==6000 and result.get("examples_seen")==24000
        and result.get("completed_fixed_endpoint") is True
        and [c["optimizer_updates"] for c in result.get("checkpoints",[])]==list(range(500,6001,500)),"all12 actual fixed checkpoint receipts required")
    final=result["checkpoints"][-1];export=output/"exports/step_006000"
    checkpoint=output/"checkpoints/step_006000.pt";npz=export/"model_state.npz";metadata=export/"metadata.json"
    require(digest(checkpoint)==final["checkpoint_sha256"] and digest(npz)==final["npz_sha256"]
        and digest(metadata)==final["metadata_sha256"],"actual final6000 endpoint bytes differ")
    description=read(metadata)
    require(description.get("schema")==EXPORT_SCHEMA and description.get("training_schema")==SCHEMA
        and description.get("arm")==result["arm"] and description.get("optimizer_updates")==6000
        and description.get("additional_optimizer_updates")==6000 and description.get("known_total_optimizer_updates")==18200
        and description.get("examples_seen")==24000 and description.get("checkpoint_sha256")==final["checkpoint_sha256"]
        and description.get("npz_sha256")==final["npz_sha256"] and description.get("source_sha256")==sources
        and description.get("declaration_sha256")==digest(output/"declaration.json")
        and description.get("parent_npz_sha256")==PARENT["weights_sha256"]
        and description.get("parent_metadata_sha256")==PARENT["metadata_sha256"]
        and description.get("parent_checkpoint_sha256")==PARENT["checkpoint_sha256"]
        and description.get("parent_declaration_sha256")==PARENT["declaration_sha256"]
        and description.get("model_promoted") is False and description.get("vehicle_control_approved") is False
        and description.get("runtime_loader_implemented") is False,"new final6000 continuation lineage/scope differs")
    expected=parent.state_dict();require(npz.stat().st_size<=32*1024**2,"bounded full101 NPZ required")
    with np.load(npz,allow_pickle=False) as arrays:
        require(len(arrays.files)==len(set(arrays.files))==101 and set(arrays.files)==set(expected),"complete exact101 numeric endpoint required")
        for name,tensor in expected.items():
            value=arrays[name]
            require(value.dtype==np.float32 and np.isfinite(value).all() and list(value.shape)==list(tensor.shape)
                ==description["state_tensor_shapes"][name] and description["state_tensor_dtypes"][name]=="float32","numeric endpoint ABI differs: "+name)
            if not name.startswith(original.TRAINABLE_PREFIXES):
                require(np.array_equal(value,tensor.detach().cpu().numpy()),"one of47 endpoint frozen tensors differs: "+name)
    replay.verify_sources(output,sources)
    return dict(arm=result["arm"],fixed_endpoint_updates=6000,all101_numeric_finite=True,frozen47_exact=True,
        checkpoint_sha256=final["checkpoint_sha256"],npz_sha256=final["npz_sha256"],metadata_sha256=final["metadata_sha256"])


def evaluation_artifact_pins(args,known_receipt,holdout,known):
    # HH_261002 - After deferred loaders have verified all image hashes/fingerprints, retain manifests and each copied/source ledger too.
    values={}
    for root,dataset in ((args.recovery_holdout,holdout),(replay.DEV_PATH,known)):
        # HH_261002 - Never census unrelated TRAIN/TEST label directories under a mixed prepared root.
        for path in (root/"dataset.json",):regular(path,original.PERSONAL);values[str(path)]=digest(path)
        directories=[root/"episodes"/episode for episode in sorted({r.episode_id for r in dataset.examples})]
        for directory in directories:
            regular(directory,original.PERSONAL)
            for suffix in ("*.json","*.jsonl"):
                for path in directory.rglob(suffix):regular(path,original.PERSONAL);values[str(path)]=digest(path)
    for path in (args.holdout_conversion,args.recovery_holdout/"signed_current_state_audit.jsonl"):
        regular(path,original.PERSONAL);values[str(path)]=digest(path)
    require(digest(args.holdout_conversion)==original.HOLDOUT_RECEIPT_SHA
        and known_receipt["validated_fingerprint_sha256"]==replay.DEV_LOADER_FP
        and known_receipt["temporal_fingerprint_sha256"]==replay.DEV_TEMPORAL_FP,"deferred actual evaluation identities differ")
    return values


def run(argv=None):
    global _created_output
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--mode",choices=("contract","validate-only","fit"),required=True)
    for name in ("parent-run","warm-parent-weights","warm-parent-metadata","backbone-parent-weights","backbone-parent-metadata",
        "recovery-train","recovery-conversion","recovery-holdout","holdout-conversion","output","authorization"):
        parser.add_argument("--"+name,type=Path)
    parser.add_argument("--authorization-sha256");parser.add_argument("--parent-history-sha256")
    parser.add_argument("--loader-workers",type=int,choices=(0,1,2),default=2)
    args=parser.parse_args(argv);proposed=contract()
    if args.mode=="contract":print(json.dumps(proposed,indent=2,allow_nan=False));return 0
    require(Path(sys.prefix).resolve()==(original.WORKSPACE/"venvs/py312").resolve(),"CPU validation and fitting require the existing private py312 venv; no install")
    require(all(getattr(args,n.replace("-","_")) is not None for n in ("parent-run","warm-parent-weights","warm-parent-metadata",
        "backbone-parent-weights","backbone-parent-metadata","recovery-train","recovery-conversion","recovery-holdout",
        "holdout-conversion","output","authorization")),"all explicit canonical assets/outputs required")
    require(not args.output.exists() and not any(p.is_symlink() for p in (args.output,*args.output.parents))
        and args.output.resolve().is_relative_to((original.WORKSPACE/"runs/training").resolve()),"fresh create-only personal pair output required")
    regular(args.authorization,original.PERSONAL);authority=validate_authorization(args.authorization,args.authorization_sha256,proposed)
    authorize_parent_history(authority,args.parent_history_sha256)
    replay.storage_reserve(args.output.parent);torch.set_num_threads(4)
    training,audits,low,legacy,strata,receipts=load_train(args)
    groups=previous.recovery_groups(training,audits);order=original.fixed_batches(groups,strata)
    dataset=ConcatDataset([training,low,legacy]);curves=set(curved_indices(strata));coverage=spatial_inventory(legacy,strata)
    parent,pins,parent_diagnostics=verify_parent(args,order,(training,low,legacy))
    for path in (args.authorization,args.recovery_conversion,args.recovery_train/"dataset.json",replay.LOW_PATH/"dataset.json",
        replay.LOW_PATH/"moving_conversion_qualification.json",replay.LEGACY_PATH/"dataset.json"):
        pins[str(path)]=digest(path)
    fields=original.scope_fields();fields.update(parent_diagnostic_pass_count=sum(c["passed"] for c in parent_diagnostics["checks"]),
        parent_diagnostic_screen_count=71,parent_all_diagnostic_screens_passed=parent_diagnostics["all_diagnostic_screens_passed"],
        current_training_purpose="paired_fixed_intrinsic_legacy_curve_geometry_continuation")
    declaration=dict(fields,schema=SCHEMA,protocol=proposed,parent_optimizer_updates=6000,parent_known_total_optimizer_updates=12200,
        parent_npz_sha256=PARENT["weights_sha256"],parent_metadata_sha256=PARENT["metadata_sha256"],parent_checkpoint_sha256=PARENT["checkpoint_sha256"],
        parent_declaration_sha256=PARENT["declaration_sha256"],parent_history_sha256=args.parent_history_sha256,
        model_id=original.MODEL_ID,architecture_schema=original.ARCHITECTURE_SCHEMA,model_config=parent.config.to_dict(),recovery_config=parent.recovery_config.to_dict(),
        source_sha256=proposed["sources"],cohorts=receipts,training_dataset_fingerprint_sha256=json_sha(receipts),
        deferred_holdout_conversion_sha256=original.HOLDOUT_RECEIPT_SHA,holdout_payload_loaded=False,
        trainable_state_names=original.freeze_inputs(parent),frozen_state_names=sorted(set(parent.state_dict())-set(original.trainable_names(parent.state_dict()))),
        trainable_state_tensor_count=54,frozen_state_tensor_count=47,trainable_parameter_count=2277827,
        effective_batch_size=4,seed=original.SEED,learning_rate=original.learning_rate_config(),batch_domains=original.BATCH_DOMAINS,
        fixed_schedule_sha256=json_sha(order),fixed_sample_ids=[expected_tokens((training,low,legacy),indices) for indices in order],
        intrinsic_TRAIN_coverage=coverage,immutable_input_sha256=pins,authorization_sha256=args.authorization_sha256)
    if args.mode=="validate-only":
        require(not torch.cuda.is_initialized(),"CPU validation must not initialize CUDA")
        print(json.dumps(dict(status="VALIDATED_TRAIN_ONLY_CPU_NO_CUDA_NO_OUTPUT",**declaration),allow_nan=False));return 0
    with raw_parent.gpu0_lease():
        require(os.environ.get("CUBLAS_WORKSPACE_CONFIG")==":4096:8","strict deterministic CuBLAS workspace required")
        gpu=raw_parent.gpu_preflight();device=torch.device("cuda:0")
        torch.use_deterministic_algorithms(True);torch.backends.cudnn.deterministic=True;torch.backends.cudnn.benchmark=False
        torch.backends.cuda.matmul.allow_tf32=False;torch.backends.cudnn.allow_tf32=False
        preflight=preflight_gradient(parent,dataset,order,curves,device)
        verify_inputs(pins,proposed["sources"],[]);initial_images=verify_training_images((training,low,legacy));args.output.mkdir();_created_output=args.output
        write_new(args.output/"declaration.json",declaration)
        write_new(args.output/"started.json",dict(schema=SCHEMA,UTC=datetime.now(timezone.utc).isoformat(),PID=os.getpid(),GPU=gpu,preflight=preflight,training_images=initial_images))
        finals={};results={};outputs=[];control_hashes=None;endpoint_receipts=[]
        for arm in ("control","intrinsic_curve"):
            model,hashes,result=train_arm(args,arm,parent,dataset,order,curves,declaration,proposed["sources"],device,control_hashes)
            finals[arm],results[arm]=model,result;outputs.append(args.output/arm)
            if arm=="control":control_hashes=hashes
            verify_inputs(pins,proposed["sources"],outputs)
            if not result["completed_fixed_endpoint"]:
                write_new(args.output/"summary.json",dict(status="INCOMPLETE_PAIR_NO_EVALUATION_NOT_PROMOTED",arms=results,model_promoted=False));return 2
            endpoint_receipts.append(verify_final_endpoint(args.output/arm,result,parent,proposed["sources"]))
        # HH_261002 - Both actual fixed endpoints exist BEFORE the first evaluation-only payload is opened.
        holdout,ho_audits,ho_receipt,known,known_receipt=load_evaluation_only(args,training,low,legacy)
        eval_pins=evaluation_artifact_pins(args,known_receipt,holdout,known);pins.update(eval_pins)
        initial_eval_images=verify_training_images((holdout,known))
        write_new(args.output/"evaluation_opened_after_both_final.json",dict(arms=results,verified_actual_endpoints=endpoint_receipts,
            holdout=ho_receipt,known_DEV=known_receipt,evaluation_source_sha256=eval_pins,evaluation_image_precheck=initial_eval_images,checkpoint_selection=False))
        metrics={};post=time.monotonic()
        for label,model in (("parent",parent),*finals.items()):
            metrics[label]=evaluate(model,device,training,audits,low,legacy,strata,holdout,ho_audits,known)
            for cohort,value in metrics[label].items():write_new(args.output/(label+"_"+cohort+".json"),value)
        screens={arm:original.endpoint_screens(metrics["parent"],metrics[arm]) for arm in finals}
        post_train=load_train(args)
        require(post_train[-1]==receipts,"full TRAIN loader/source/label fingerprints changed after paired fit")
        post_holdout,_,post_ho_receipt,post_known,post_known_receipt=load_evaluation_only(args,training,low,legacy)
        require(post_ho_receipt==ho_receipt and post_known_receipt==known_receipt
            and post_holdout.fingerprint_sha256==holdout.fingerprint_sha256 and post_known.fingerprint_sha256==known.fingerprint_sha256,
            "deferred full HOLDOUT/known loader/image/label fingerprints changed during postfit diagnostics")
        verify_inputs(pins,proposed["sources"],outputs);final_images=verify_training_images((training,low,legacy,holdout,known))
        for arm in finals:verify_final_endpoint(args.output/arm,results[arm],parent,proposed["sources"])
        write_new(args.output/"summary.json",dict(schema=SCHEMA,status="BOTH_FIXED6000_COMPLETE_NOT_PROMOTED",arms=results,
            endpoint_diagnostic_screens=screens,all71_retained_per_arm=True,actual_batch_bytes_equal=True,
            postfit_evaluation_seconds=time.monotonic()-post,final_TRAIN_image_postcheck=final_images,model_promoted=False,vehicle_control_approved=False,
            official_main_features_approved=0,official_subfeatures_approved=0))
    return 0


def main(argv=None):
    global _created_output
    require(_created_output is None,"single non-reentrant pair invocation required")
    try:return run(argv)
    except Exception as error:
        # HH_261002 - The GPU lease context has unwound before this failure journal; retain every partial arm/checkpoint/failed row.
        if _created_output is not None:
            require(not _created_output.is_symlink(),"refuse failure writes through a changed output")
            write_new(_created_output/"failure_receipt.json",dict(schema=SCHEMA,status="FAILED_PARTIAL_PAIR_RETAINED_NOT_PROMOTED",
                exception_type=type(error).__name__,exception_message=str(error),partial_outputs_retained=True,
                GPU_lease_context_exited=True,foreign_processes_killed=False,model_promoted=False,
                vehicle_control_approved=False,DEV_checkpoint_selection=False))
        raise
    finally:_created_output=None


if __name__=="__main__":raise SystemExit(main())
