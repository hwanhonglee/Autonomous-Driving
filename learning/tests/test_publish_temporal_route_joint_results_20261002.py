"""HH_261002 - Synthetic-only final6000/all71 publication checks; no GPU, simulator or actual training claims."""
import copy
from collections import Counter
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest

from scripts.e2e import publish_temporal_route_joint_results_20261002 as publisher

_spec=importlib.util.spec_from_file_location("hh_old_publish_fixtures",Path(__file__).with_name("test_publish_temporal_route_recovery_moving_v4_results_20261001.py"))
old=importlib.util.module_from_spec(_spec);_spec.loader.exec_module(old)


def scope():
    return dict(training_only=True,simulation_only=True,main_training=False,model_promoted=False,vehicle_control_approved=False,
        inference_loader_authorized=False,stop_intent_trained=False,backbone_frozen=False,visual_encoder_frozen=True,
        semantic_head_parameters_frozen=True,semantic_outputs_preserved=False,labels_clipped=False,labels_extrapolated=False,
        legacy_decoded=False,independent_validation=False,checkpoint_selection_by_holdout=False,startup_recovery_fitted=False,
        new_signed_recovery_cohort_moving_only=True,parent_all_diagnostic_screens_passed=False,
        runtime_intent="explicit_cruise_GO_not_head",training_episode_count=18,recovery_training_episode_count=9,replay_training_episode_count=9,
        holdout_episode_count=2,official_main_features_approved=0,official_subfeatures_approved=0,parent_diagnostic_pass_count=20,
        parent_diagnostic_screen_count=48,fixed_recovery_driving_relative_window_seconds=[4.,34.1],fixed_recovery_native_frame_offsets=[80,682])


def shapes(*,tiny=False):
    # HH_261002 - Fake names test byte/count contracts only, never pretend to construct a real neural model.
    values={"fusion.fixture%02d"%i:[1] for i in range(50)}
    if not tiny:values["fusion.fixture00"]=[2276369]
    values.update({"image_encoder.fixture%02d"%i:[1] for i in range(47)})
    values.update(publisher.previous.BRANCH_SHAPES)
    return values


@pytest.fixture
def evidence():
    documents,_,declaration_sha=old.evidence.__wrapped__()
    before={k:documents[k+"_before.json"] for k in publisher.COHORTS}
    after={k:documents[k+"_final.json"] for k in publisher.COHORTS}
    for cohort in ("recovery_train","recovery_holdout"):
        for row in after[cohort]["rows"]:row["instantaneous_speed_mae_mps"] = .02
        after[cohort]["mean_instantaneous_speed_mae_mps"] = .02
        for row in after[cohort]["per_episode"].values():row["instantaneous_speed_mae_mps"] = .02
    for cohort in ("recovery_train","recovery_holdout","v4"):
        for row in before[cohort]["rows"]:row["learned_near_branch_logit_mean"] = .05
    for stage in ("before","final"):
        documents["known599_"+stage+".json"]["role"]="previously_exposed_known_DEV599_raw_diagnostic"
    legacy=before["legacy"]["rows"]
    domains={kind:{m:[r for r in legacy if r["stratum"]==kind and r["map"]==m]
        for m in sorted({r["map"] for r in legacy if r["stratum"]==kind})} for kind in ("curved","straight")}
    history=[];counts=Counter();unique={k:set() for k in ("recovery","v4","legacy")}
    for i in range(6000):
        kind="curved" if i%2==0 else "straight"; maps=sorted(domains[kind]);m=maps[counts[kind]%len(maps)]
        candidates=domains[kind][m];row=candidates[(counts[kind]//len(maps))%len(candidates)];counts[kind]+=1
        indices=[i%30,(i%9)*302+(i//9)%302,2718+i%327,3045+row["index"]]
        tokens=[before["recovery_train"]["rows"][j]["token"] for j in indices[:2]]+[before["v4"]["rows"][i%327]["token"],row["token"]]
        unique["recovery"].update(tokens[:2]);unique["v4"].add(tokens[2]);unique["legacy"].add(tokens[3])
        history.append(dict(optimizer_updates=i+1,examples_seen=4*(i+1),recovery_examples_seen=2*(i+1),v4_examples_seen=i+1,legacy_examples_seen=i+1,
            indices=indices,sample_ids=tokens,loss=.25*(.2+.2+.2+.25*.01+.02),recovery0=dict(total=.2),recovery1=dict(total=.2),v4=dict(total=.2),
            legacy=dict(total=.0225,raw_curvature=.01,raw_preferred_speed=.02,intent=0.,stop_distance=0.,initial_acceleration=0.,temporal_xy=0.,temporal_speed=0.),
            per_domain_trainable_gradients={k:dict(all_present_branch_gradients_finite=True,gradient_l2_norm=.1,unused_branch_tensors=0) for k in publisher.BATCH_DOMAINS},
            learning_rates={g:publisher.learning_rate(i+1,g) for g in ("shared","heads")},preclip_trainable_gradient_norm=.2,elapsed_owned_seconds=i*.05))
    sources={k:"d"*64 for k in publisher.SOURCE_NAMES};sources.update(publisher.FROZEN_SOURCES)
    state_shapes=shapes();names=publisher.trainable_names(state_shapes);frozen=sorted(set(state_shapes)-set(names))
    shared=dict(scope(),parent_checkpoint_sha256=publisher.PARENT_CHECKPOINT_SHA,parent_npz_sha256=publisher.PARENT_NPZ_SHA,
        parent_metadata_sha256=publisher.PARENT_METADATA_SHA,parent_declaration_sha256=publisher.PARENT_DECLARATION_SHA,
        source_sha256=sources,learning_rate=copy.deepcopy(publisher.RATE),batch_domains=publisher.BATCH_DOMAINS,
        trainable_state_names=names,frozen_state_names=frozen,trainable_state_tensor_count=54,frozen_state_tensor_count=47,trainable_parameter_count=2277827,
        frozen_state_sha256="b"*64,model_config=dict(fixture="not_real_model"),recovery_config=dict(fixture="not_real_branch"),
        training_dataset_fingerprint_sha256="f"*64,holdout_dataset_fingerprint_sha256="e"*64)
    declaration=documents["declaration.json"]
    declaration.update(shared,schema=publisher.RUN_SCHEMA,requested_optimizer_updates=6000,maximum_optimizer_updates=6000,effective_batch_size=4,
        seed=2026100201,fair_three_seed_comparison_performed=False,maximum_fit_including_baseline_seconds=5400,
        fixed_schedule_sha256=hashlib.sha256(json.dumps([r["indices"] for r in history],separators=(",",":")).encode()).hexdigest(),
        declared_unique_source_exposure_counts={k:len(v) for k,v in unique.items()})
    metadata=dict(shared,schema=publisher.EXPORT_SCHEMA,training_schema=publisher.RUN_SCHEMA,architecture_schema=publisher.ARCHITECTURE_SCHEMA,
        model_id=publisher.MODEL_ID,optimizer_updates=6000,additional_optimizer_updates=6000,maximum_optimizer_updates=6000,effective_batch_size=4,
        examples_seen=24000,recovery_examples_seen=12000,v4_examples_seen=6000,legacy_examples_seen=6000,parent_optimizer_updates=3000,
        parent_known_total_optimizer_updates=6200,known_total_optimizer_updates=12200,declaration_sha256=declaration_sha,
        checkpoint_sha256="c"*64,npz_sha256="e"*64,state_tensor_shapes=state_shapes,state_tensor_dtypes={k:"float32" for k in state_shapes})
    documents.pop("exports/step_003000/metadata.json");documents["exports/step_006000/metadata.json"]=metadata
    checks=publisher.endpoint_checks(before,after)
    endpoint=dict(fixed_endpoint_updates=6000,diagnostic_screen_count=71,retained_original_screen_count=48,checks=checks,
        all_diagnostic_screens_passed=all(c["passed"] for c in checks),independent_validation=False,feature_approved=False,
        vehicle_control_approved=False,model_promoted=False,absolute_30kph_speed_tracking_claimed=False,fixed_final_only=True,holdout_selected_checkpoint=False)
    documents["endpoint_diagnostic_screens.json"]=endpoint
    checkpoints=[dict(optimizer_updates=step,sha256="c"*64,export_npz_sha256="e"*64,export_metadata_sha256="b"*64,optimizer_state_retained=True) for step in range(500,6001,500)]
    documents["summary.json"].update(scope(),schema=publisher.RUN_SCHEMA,status="FIXED6000_JOINT_COMPLETE_NOT_PROMOTED",optimizer_updates=6000,
        examples_seen=24000,recovery_examples_seen=12000,v4_examples_seen=6000,legacy_examples_seen=6000,wall_budget_stop=False,
        frozen47_tensors_byte_identical=True,full97_frozen_claimed=False,source_hashes_match=True,checkpoints=checkpoints,endpoint_diagnostic_screens=endpoint)
    documents["started.json"]=dict(gpu=dict(physical_index=0,uuid="GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5",idle_before_torch=True),
        actual_parent_all101_loaded=True,trainable_state_tensor_count=54,frozen_state_tensor_count=47,trainable_parameter_count=2277827,frozen_state_sha256="b"*64)
    for cohort,suffix in (("recovery_train","train"),("recovery_holdout","holdout")):
        documents["actual_signed_branch_response_"+suffix+".json"]=old.response(before[cohort],after[cohort])
    return documents,history,declaration_sha


def test_all71_gate_recomputation_keeps_failure_and_full6000_journal(evidence):
    report=publisher.build_report(*evidence)
    assert report["diagnostic_gate_count"]==71 and report["diagnostic_pass_count"]==70
    assert report["actual_journal"]["actual_anchor_exposures"]==24000
    assert report["actual_journal"]["actual_unique_source_anchors"]==dict(recovery=2718,v4=327,legacy=2487)
    assert report["lineage"]["learned_tensor_count"]==54 and report["lineage"]["frozen_tensor_count"]==47
    assert report["official_main_function_approvals"]==report["official_subfunction_approvals"]==0
    assert report["physical_trials_included"] is False


@pytest.mark.parametrize("mutation",["drop_update","holdout_token","speed_loss_missing","wrong_lr","nan_gradient","frozen97","promote","pick5000","known_independent","drop_tail"])
def test_contract_and_journal_mutations_rejected(evidence,mutation):
    documents,history,sha=evidence
    if mutation=="drop_update":history.pop()
    elif mutation=="holdout_token":history[0]["sample_ids"][0]=documents["recovery_holdout_before.json"]["rows"][0]["token"]
    elif mutation=="speed_loss_missing":history[0]["loss"]-=.25*history[0]["legacy"]["raw_preferred_speed"]
    elif mutation=="wrong_lr":history[0]["learning_rates"]["heads"]=3e-4
    elif mutation=="nan_gradient":history[0]["preclip_trainable_gradient_norm"]=float("nan")
    elif mutation=="frozen97":documents["summary.json"]["full97_frozen_claimed"]=True
    elif mutation=="promote":documents["summary.json"]["model_promoted"]=True
    elif mutation=="pick5000":documents["summary.json"]["optimizer_updates"]=5000
    elif mutation=="known_independent":documents["known599_final.json"]["independent_validation"]=True
    else:documents["recovery_train_final.json"]["rows"].pop()
    with pytest.raises(ValueError):publisher.build_report(documents,history,sha)


def test_readable_MF02_MF03_scope_and_all_failed_gates(evidence):
    report=publisher.build_report(*evidence);text=publisher.markdown(report,dict(status="SYNTHETIC_ONLY"),Path("synthetic-run"))
    assert "0/9" in text and "0/30" in text and "MF02" in text and "MF03" in text
    assert "30kph SPD01 검증이 아닙니다" in text and "STOP은 아직 학습·승인하지 않았습니다" in text
    assert all(c["name"] in text for c in report["checks"] if not c["passed"])
    assert "54개 tensor" in text and "47개 tensor" in text


def test_publisher_imports_without_Torch_numpy_GPU_or_model_loader():
    command="import runpy,sys; w=runpy.run_path('tools/workspace_python.py'); w['configure'](); from scripts.e2e import publish_temporal_route_joint_results_20261002; assert 'torch' not in sys.modules and 'numpy' not in sys.modules; print('STDLIB_ONLY_PASS')"
    result=subprocess.run([sys.executable,"-B","-c",command],cwd=publisher.ROOT,capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    assert "STDLIB_ONLY_PASS" in result.stdout


@pytest.fixture
def archives(tmp_path,evidence,monkeypatch):
    # HH_261002 - Tiny artificial101 arrays validate retained byte-boundaries, not actual neural weights.
    import numpy as np
    run,root=tmp_path/"run",tmp_path/"sources";run.mkdir();root.mkdir()
    documents,_,_=evidence;decl=documents["declaration.json"];summary=documents["summary.json"]
    dims=shapes(tiny=True);grand_arrays={k:np.ones(v,dtype=np.float32) for k,v in dims.items() if not k.startswith("route_recovery_branch.")}
    parent_arrays=dict(grand_arrays,**{k:np.zeros(v,dtype=np.float32) for k,v in publisher.previous.BRANCH_SHAPES.items()})
    folders=[tmp_path/"parent",tmp_path/"grandparent"]
    for folder in folders:folder.mkdir()
    parent_weights,parent_metadata=folders[0]/"model.npz",folders[0]/"metadata.json"
    backbone_weights,backbone_metadata=folders[1]/"model.npz",folders[1]/"metadata.json"
    for path,arrays in ((parent_weights,parent_arrays),(backbone_weights,grand_arrays)):
        with path.open("xb") as stream:np.savez(stream,**arrays)
    parent=dict(schema=publisher.previous.EXPORT_SCHEMA,checkpoint_sha256=publisher.PARENT_CHECKPOINT_SHA,
        declaration_sha256=publisher.PARENT_DECLARATION_SHA,optimizer_updates=3000,known_total_optimizer_updates=6200,backbone_frozen=True,
        model_promoted=False,vehicle_control_approved=False,model_config=decl["model_config"],recovery_config=decl["recovery_config"],
        state_tensor_shapes={k:list(v.shape) for k,v in parent_arrays.items()},state_tensor_dtypes={k:"float32" for k in parent_arrays})
    grand=dict(schema=publisher.previous.prior.EXPORT_SCHEMA,checkpoint_sha256=publisher.previous.PARENT_CHECKPOINT_SHA,
        optimizer_updates=3000,npz_sha256=publisher.digest(backbone_weights),state_tensor_shapes={k:list(v.shape) for k,v in grand_arrays.items()},
        state_tensor_dtypes={k:"float32" for k in grand_arrays})
    publisher.write_new(parent_metadata,parent);publisher.write_new(backbone_metadata,grand)
    monkeypatch.setattr(publisher,"PARENT_NPZ_SHA",publisher.digest(parent_weights));monkeypatch.setattr(publisher,"PARENT_METADATA_SHA",publisher.digest(parent_metadata))
    monkeypatch.setattr(publisher,"GRANDPARENT_NPZ_SHA",publisher.digest(backbone_weights));monkeypatch.setattr(publisher,"GRANDPARENT_METADATA_SHA",publisher.digest(backbone_metadata))
    for name in decl["source_sha256"]:
        for directory in (root,run/"source"):
            path=directory/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(b"synthetic immutable source")
        decl["source_sha256"][name]=publisher.digest(root/name)
    for field,name in (("HELPER_SHA",publisher.HELPER_FILE),):
        path=root/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(b"synthetic frozen helper");monkeypatch.setattr(publisher,field,publisher.digest(path))
    prior=root/publisher.previous.PRIOR_FILE;prior.parent.mkdir(parents=True,exist_ok=True);prior.write_bytes(b"synthetic older helper")
    monkeypatch.setattr(publisher.previous,"PRIOR_SHA",publisher.digest(prior))
    publisher.write_new(run/"declaration.json",decl);decl_sha=publisher.digest(run/"declaration.json")
    frozen=sorted(set(parent_arrays)-set(publisher.trainable_names(parent_arrays)))
    frozen_sha=publisher.previous.backbone_fingerprint({k:parent_arrays[k] for k in frozen})
    final=None
    for step in range(500,6001,500):
        folder=run/"exports"/("step_%06d"%step);folder.mkdir(parents=True)
        checkpoint=run/"checkpoints"/("step_%06d.pt"%step);checkpoint.parent.mkdir(exist_ok=True);checkpoint.write_bytes(b"opaque optimizer checkpoint"+str(step).encode())
        arrays={k:v.copy() for k,v in parent_arrays.items()};arrays["fusion.fixture00"]+=step/10000
        with (folder/"model_state.npz").open("xb") as stream:np.savez(stream,**arrays)
        middle=dict(scope(),schema=publisher.EXPORT_SCHEMA,training_schema=publisher.RUN_SCHEMA,source_sha256=decl["source_sha256"],
            learning_rate=copy.deepcopy(publisher.RATE),declaration_sha256=decl_sha,parent_npz_sha256=publisher.PARENT_NPZ_SHA,frozen_state_sha256=frozen_sha,
            trainable_state_names=publisher.trainable_names(arrays),frozen_state_names=frozen,checkpoint_sha256=publisher.digest(checkpoint),
            npz_sha256=publisher.digest(folder/"model_state.npz"),optimizer_updates=step,additional_optimizer_updates=step,examples_seen=4*step,
            recovery_examples_seen=2*step,v4_examples_seen=step,legacy_examples_seen=step,known_total_optimizer_updates=6200+step,
            model_config=decl["model_config"],recovery_config=decl["recovery_config"],
            state_tensor_shapes={k:list(v.shape) for k,v in arrays.items()},state_tensor_dtypes={k:"float32" for k in arrays})
        publisher.write_new(folder/"metadata.json",middle)
        summary["checkpoints"][step//500-1].update(sha256=middle["checkpoint_sha256"],export_npz_sha256=middle["npz_sha256"],export_metadata_sha256=publisher.digest(folder/"metadata.json"))
        if step==6000:final=middle
    documents["exports/step_006000/metadata.json"]=final;publisher.write_new(run/"summary.json",summary)
    kwargs=dict(summary_sha256=publisher.digest(run/"summary.json"),declaration_sha256=decl_sha,checkpoint_sha256=final["checkpoint_sha256"],
        weights_sha256=final["npz_sha256"],metadata_sha256=publisher.digest(run/"exports/step_006000/metadata.json"),
        parent_weights=parent_weights,parent_metadata=parent_metadata,backbone_parent_weights=backbone_weights,backbone_parent_metadata=backbone_metadata)
    return run,root,documents,kwargs


def test_all12_numeric_exports_preserve47_but_fusion_speed_may_change(archives):
    run,root,docs,kwargs=archives;receipt=publisher.verify_evidence_files(run,root,docs,**kwargs)
    assert receipt["all12_exports_full101_finite_frozen47_exact"] is True and receipt["no_Torch_pickle"] is True
    assert receipt["original97_frozen_claimed"] is False and len(receipt["fixed_exports"])==12


@pytest.mark.parametrize("mutation",["frozen_parent","checkpoint","metadata","source","bad_finalpin"])
def test_retained_archive_boundary_mutations_fail(archives,mutation):
    run,root,docs,kwargs=archives
    if mutation=="frozen_parent":kwargs["parent_weights"].write_bytes(b"changed")
    elif mutation=="checkpoint":(run/"checkpoints/step_000500.pt").write_bytes(b"changed")
    elif mutation=="metadata":(run/"exports/step_001000/metadata.json").write_bytes(b"changed")
    elif mutation=="source":(run/"source"/next(iter(publisher.SOURCE_NAMES))).write_bytes(b"changed")
    else:kwargs["weights_sha256"]="0"*64
    with pytest.raises(ValueError):publisher.verify_evidence_files(run,root,docs,**kwargs)


@pytest.mark.parametrize("where",["run","parent","grandparent"])
def test_output_cannot_mutate_any_immutable_run_or_parent_directory(archives,where):
    run,root,docs,kwargs=archives
    parent=run if where=="run" else kwargs["parent_weights"].parent if where=="parent" else kwargs["backbone_parent_weights"].parent
    with pytest.raises(ValueError,match="outside immutable"):publisher.publish(run,parent/"newreport",root=root,**kwargs)


def test_symlink_ancestor_fail_closed_before_any_write(archives,tmp_path):
    run,root,docs,kwargs=archives;alias=tmp_path/"alias";alias.symlink_to(run,target_is_directory=True)
    output=tmp_path/"freshreport"
    with pytest.raises(ValueError,match="canonical path"):publisher.publish(alias,output,root=root,**kwargs)
    assert not output.exists()


def test_first_read_mutation_before_publication_is_detected(evidence,tmp_path,monkeypatch):
    documents,history,sha=evidence;run=tmp_path/"run";run.mkdir();root=tmp_path/"root";root.mkdir()
    for name,doc in documents.items():path=run/name;path.parent.mkdir(parents=True,exist_ok=True);publisher.write_new(path,doc)
    path=run/"history.jsonl";path.write_text("\n".join(json.dumps(r) for r in history)+"\n")
    assets=[]
    for i in range(4):directory=tmp_path/("parent%d"%i);directory.mkdir();asset=directory/"asset";asset.write_bytes(b"synthetic");assets.append(asset)
    monkeypatch.setattr(publisher,"verify_evidence_files",lambda *args,**kwargs:{})
    def mutate(*args,**kwargs):
        path.write_text("changed journal after parsing");return dict(status="SYNTHETIC_ONLY")
    monkeypatch.setattr(publisher,"numeric_receipt",mutate)
    output=tmp_path/"freshreport"
    with pytest.raises(ValueError,match="pinned evidence"):
        publisher.publish(run,output,root=root,summary_sha256="a"*64,declaration_sha256=sha,checkpoint_sha256="b"*64,weights_sha256="c"*64,
            metadata_sha256="d"*64,parent_weights=assets[0],parent_metadata=assets[1],backbone_parent_weights=assets[2],backbone_parent_metadata=assets[3])
    assert not output.exists()
