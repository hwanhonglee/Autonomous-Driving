"""HH_261002 - Synthetic contracts/samplers/numeric endpoints; no actual TRAIN, model inference, CUDA or vehicle jobs."""
import copy
from collections import Counter
import hashlib
import inspect
import json
from pathlib import Path
import shutil
import subprocess
import sys
from types import SimpleNamespace
from unittest import mock

import numpy as np
import pytest
import torch

from scripts.e2e import train_temporal_left_joint_20261002 as t
from portable_e2e.temporal_route_recovery_v3_20261001 import RouteRecoveryTemporalModelV3


def groups():
    profiles=sorted(t.previous.TRAIN_PROFILES)
    full={p:list(range(i*302,(i+1)*302)) for i,p in enumerate(profiles)}
    sizes=[7,19,115,104,63,116,9,7]
    keys=[(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]
    focus={k:full[profiles[i]][:size] for i,(k,size) in enumerate(zip(keys,sizes))}
    return dict(full=full,focus=focus)


def strata():
    return dict(curved={"C_track_1_0_7":list(range(260)),"Town01":list(range(260,328))},
        straight={"C_track_1_0_7":list(range(328,990)),"Town01":list(range(990,1560)),"Town07":list(range(1560,2487))})


@pytest.fixture(scope="module")
def order():return t.fixed_batches(groups(),strata())


def test_actual20k_fullcycle80k_and_all4_exactdomains(order):
    assert len(order)==20000 and sum(map(len,order))==80000
    assert order==t.fixed_batches(groups(),strata())
    assert all(len(set(r))==4 and 0<=r[0]<2718 and 2718<=r[1]<3045
        and 3045<=r[2]<5532 and 5832<=r[3]<6179 for r in order)
    assert {r[0] for r in order}==set(range(2718))
    assert {r[1]-2718 for r in order}==set(range(327))
    assert {r[2]-3045 for r in order}==set(range(2487))
    assert {r[3]-5832 for r in order}==set(range(347))
    assert not {r[2]-3045 for r in order}&set(range(2487,2787))
    counts=Counter(r[3]-5832 for r in order)
    assert set(counts.values())=={57,58} and sum(v==58 for v in counts.values())==221


def test_independent_focus_cursor_does_not_drop_half_signed_bins(order):
    g=groups();by_index={i:k for k,v in g["focus"].items() for i in v}
    selected=[row[0] for step,row in enumerate(order) if step%2==0]
    assert len(selected)==10000 and set(selected)==set(by_index)
    assert Counter(by_index[i] for i in selected)=={key:1250 for key in g["focus"]}
    assert {row[0] for step,row in enumerate(order) if step%2}==set(range(2718))


def test_replay_curve_straight_and_maps_remain_exactly_balanced(order):
    s=strata();curved={i for values in s["curved"].values() for i in values}
    assert sum(row[2]-3045 in curved for row in order)==10000
    assert all((row[2]-3045 in curved)==(step%2==0) for step,row in enumerate(order))
    assert [sum(row[2]-3045 in values for row in order) for values in s["curved"].values()]==[5000,5000]
    assert sorted(sum(row[2]-3045 in values for row in order) for values in s["straight"].values())==[3333,3333,3334]


@pytest.mark.parametrize("bad",["emptyfocus","sevenbins","eightprofiles","lost302","emptycurve","emptystraight"])
def test_no_empty_or_fabricated_recovery_legacy_strata(bad):
    g,s=groups(),strata()
    if bad=="emptyfocus":g["focus"]={}
    elif bad=="sevenbins":g["focus"].pop(next(iter(g["focus"])))
    elif bad=="eightprofiles":g["full"].pop(next(iter(g["full"])))
    elif bad=="lost302":next(iter(g["full"].values())).pop()
    elif bad=="emptycurve":s["curved"]={}
    else:s["straight"]={}
    with pytest.raises(ValueError):t.fixed_batches(g,s)


def test_sampler_never_mutates_original6000_generator_or_learning_globals():
    g,s=groups(),strata();before=t.original.fixed_batches(g,s)
    t.fixed_batches(g,s)
    assert before==t.original.fixed_batches(g,s) and len(before)==6000
    assert t.original.MAX_UPDATES==6000 and t.pair.MAX_UPDATES==6000


@pytest.mark.parametrize("group,peak,final",[("shared",1e-5,1e-6),("heads",3e-5,3e-6)])
def test20k_lr_only_changes_disclosed_cosine_horizon(group,peak,final):
    assert t.learning_rate(1,group)==t.original.learning_rate(1,group)
    assert t.learning_rate(300,group)==peak and t.learning_rate(20000,group)==final
    assert t.learning_rate(6000,group)>final
    assert {k:v for k,v in t.learning_rate_config().items() if k!="schedule"}=={k:v for k,v in t.original.learning_rate_config().items() if k!="schedule"}


@pytest.mark.parametrize("update,group",[(True,"shared"),(0,"shared"),(20001,"shared"),(300.,"heads"),(1,"best_DEV")])
def test_fixed_endpoint_lr_and_typed_group(update,group):
    with pytest.raises(ValueError):t.learning_rate(update,group)


def test_exact_old_loss_order_and_realLEFT_position_no_legacy_decode():
    leaves=[torch.tensor(float(i+1),requires_grad=True) for i in range(5)]
    labels=dict(marker=torch.tensor([0,1,2,3]),sample_id=["rec","low","legacy","LEFT"])
    calls=[]
    def joint(model,decoder,batch):
        index=int(batch["marker"][0]);calls.append(index)
        return None,None,dict(total=leaves[{0:0,3:1,1:2}[index]])
    def raw(model,batch):assert int(batch["marker"][0])==2;return {"marker":2}
    def old(output,batch):assert output["marker"]==2;return dict(raw_curvature=leaves[3],raw_preferred_speed=leaves[4])
    with mock.patch.object(t.joint,"joint_forward",side_effect=joint),mock.patch.object(t.raw_parent,"forward",side_effect=raw),\
            mock.patch.object(t.original,"temporal_trajectory_loss_v2",side_effect=old):
        loss,terms,legacy,values=t.domains(None,None,labels)
    assert calls==[0,3,1] and torch.equal(loss,t.original.exact_objective(*leaves))
    loss.backward();assert [float(v.grad) for v in leaves]==[.25,.25,.25,.0625,.25]
    assert values[0] is terms[0]["total"] and values[1] is terms[2]["total"] and values[3] is terms[1]["total"]


def test_exact54_47_same101_architecture_and_actual_good_parent_only():
    model=RouteRecoveryTemporalModelV3();names=t.original.freeze_inputs(model)
    assert len(model.state_dict())==101 and len(names)==54
    assert sum(p.numel() for p in model.parameters() if p.requires_grad)==2277827
    assert not any(p.requires_grad for p in model.image_encoder.parameters())
    assert model.speed_head.weight.requires_grad and model.curvature_head.weight.requires_grad
    assert not model.intent_head.weight.requires_grad and not model.acceleration_head.weight.requires_grad
    assert t.pair.PARENT["weights_sha256"].startswith("b24a6ddd") and t.PARENT_HISTORY_SHA.startswith("3c1fe1e4")


def receipt():
    result=dict(schema=t.LEFT_CONVERSION_SCHEMA,status="QUALIFIED_RESEARCH_TRAIN_LEFT347_COMMON10",episode_count=1,train_examples=347,
        val_examples=0,test_examples=0,true_future_points=22208,full_five_camera_history_examples=339,full_ten_ego_history_examples=338,
        formal_MAIN9_approved=0,formal_sub30_approved=0,research_training_authorized=True,raw_native_denials_overridden=False,
        original_pending_metadata_unchanged=True,labels_clipped=False,labels_or_pixels_transformed=False,synthetic_history_created=False,
        future_labels_used_as_model_input=False,exact_native_ego_navigation_and_target_reconstruction=True,actual_hypot_current_speed_no_clipping=True,
        signed_state_diagnostic_not_model_input=True,vehicle_control_approved=False,model_promoted=False,
        unselected_raw_context_preserved_in_source_manifest=True,**t.LEFT_SCOPE)
    result.update({key:"a"*64 for key in ("dataset_manifest_sha256","validation_report_sha256","qualification_receipt_sha256",
        "train_loader_fingerprint_sha256","temporal_dataset_fingerprint_sha256","signed_current_state_audit_sha256","external_ROOT_authority_sha256")})
    result["per_episode_native_qualification"]=[dict(split="train",selected_examples=347,case_id="pilot_00",episode_id="LEFTnative",manifest="episodes/LEFTnative/episode.json",
        qualification_receipt_sha256="a"*64,native_episode="/original/V2/pilot_00/episode",native_manifest_sha256="b"*64,
        source_route_sha256="c"*64,case_plan_sha256="d"*64,source_camera_sha256="e"*64,source_state_sha256="f"*64)]
    return result


def test_exact_V3_conversion_nativeV2_scope_not_failed_V2_alias():
    value=receipt();assert t.verify_LEFT_conversion(value)==value["per_episode_native_qualification"][0]
    assert t.LEFT_BASENAME.endswith("-v3") and t.LEFT_ADAPTER.endswith("_v3")


@pytest.mark.parametrize("key,value",[("schema","carla.actual_town01_left45_common10_conversion.20261002.v2"),
    ("train_examples",346),("train_examples",347.),("val_examples",1),("test_examples",1),("true_future_points",22144),
    ("research_training_authorized",False),("raw_native_denials_overridden",True),("labels_clipped",True),
    ("STOP_or_stationary_tail_supervision",True),("rolling_LEFT_only",1),("synthetic_history_created",True),
    ("model_promoted",True),("qualification_receipt_sha256",None),("full_ten_ego_history_examples",347)])
def test_allmembership_denial_label_and_research_flags_strict(key,value):
    r=receipt();r[key]=value
    with pytest.raises(ValueError):t.verify_LEFT_conversion(r)


@pytest.mark.parametrize("key,value",[("split","val"),("selected_examples",347.),("selected_examples",346),
    ("case_id","old_failed_V1"),("qualification_receipt_sha256","b"*64),("source_state_sha256","notSHA")])
def test_one_native_identity_and_split_required(key,value):
    r=receipt();r["per_episode_native_qualification"][0][key]=value
    with pytest.raises(ValueError):t.verify_LEFT_conversion(r)


def fake_dataset(count,episode,split="train"):
    class Dataset:
        def __init__(self):
            self.split=split;self.examples=[SimpleNamespace(token=f"{episode}/{i}",episode_id=episode,
                features=[0.,1.5,0.],valid_point_count=64,target_speed_mps=[1.5]*64) for i in range(count)]
        def __len__(self):return len(self.examples)
    return Dataset()


def test_generically_bound4cohort_tokens_offsets_and_noVAL_repeatview():
    sets=[fake_dataset(n,label) for n,label in [(2718,"rec"),(327,"low"),(2787,"legacy"),(347,"LEFT")]]
    assert t.sample_ids(sets,[0,2718,3045,5832])==["rec/0","low/0","legacy/0","LEFT/0"]
    t.pair.disjoint(sets)
    sets[-1].split="val"
    with pytest.raises(ValueError):t.pair.disjoint(sets)
    sets[-1].split="train";sets[-1].examples[0].token="rec/0"
    with pytest.raises(ValueError):t.pair.disjoint(sets)


@pytest.mark.parametrize("index",[-1,6179,True,0.])
def test_schedule_indices_must_be_typed_actualTRAIN(index):
    sets=[fake_dataset(n,label) for n,label in [(2718,"rec"),(327,"low"),(2787,"legacy"),(347,"LEFT")]]
    with pytest.raises(ValueError):t.sample_ids(sets,[index])


@pytest.fixture
def LEFTfixture(tmp_path,monkeypatch):
    root=tmp_path/"dataset/prepared"/t.LEFT_BASENAME;episode=root/"episodes/LEFTnative";episode.mkdir(parents=True)
    monkeypatch.setattr(t.original,"PERSONAL",tmp_path)
    r=receipt();case=r["per_episode_native_qualification"][0]
    def write(path,value):path.write_text(json.dumps(value));return t.digest(path)
    r["dataset_manifest_sha256"]=write(root/"dataset.json",dict(episodes=[dict(manifest=case["manifest"])]))
    r["validation_report_sha256"]=write(root/"validation_report.json",dict(status="synthetic"))
    r["signed_current_state_audit_sha256"]=write(root/"signed_current_state_audit.jsonl",dict(diagnostic=True))
    write(episode/"episode.json",dict(split="train",episode_id="LEFTnative",source_provenance=dict(adapter_id=t.LEFT_ADAPTER)))
    case["native_manifest_sha256"]=write(episode/"native_manifest.json",dict(native="PENDING_QA_NOT_AUTHORIZED"))
    case["source_route_sha256"]=write(episode/"native_route.json",dict(route="original"))
    q=dict(schema=t.LEFT_QUALIFICATION_SCHEMA,status="QUALIFIED_RESEARCH_TRAIN_LEFT347",research_training_authorized=True,
        raw_native_denials_overridden=False,external_ROOT_authority_sha256=r["external_ROOT_authority_sha256"],cases=[copy.deepcopy(case)],**t.LEFT_SCOPE)
    r["qualification_receipt_sha256"]=write(episode/"moving_qualification.json",q);case["qualification_receipt_sha256"]=r["qualification_receipt_sha256"]
    conversion=root/"moving_conversion_qualification.json";pin=write(conversion,r)
    dataset=fake_dataset(347,"LEFTnative")
    loader=dict(validated_fingerprint_sha256=r["train_loader_fingerprint_sha256"],temporal_fingerprint_sha256=r["temporal_dataset_fingerprint_sha256"])
    monkeypatch.setattr(t.raw_parent,"load_view",lambda *args:(dataset,loader))
    monkeypatch.setattr(t.joint,"cohort_statistics",lambda d:dict(causal_navigation_support_m=dict(minimum=18.)))
    return SimpleNamespace(args=SimpleNamespace(left_train=root,left_conversion_sha256=pin),root=root,episode=episode,dataset=dataset,r=r)


def test_load_exactLEFT_receipt_calls_unchangedTRAINloader_and_no_eval(LEFTfixture):
    dataset,receipt_,pins=t.load_LEFT(LEFTfixture.args)
    assert dataset is LEFTfixture.dataset and len(dataset)==347 and len(pins)==7
    assert receipt_["conversion_sha256"]==LEFTfixture.args.left_conversion_sha256
    assert not torch.cuda.is_initialized()


@pytest.mark.parametrize("mutation",["current_speed_high","zero_future","missing_point","copiednative","episodeVAL","foreignadapter","wrongbasename"])
def test_LEFT_loss_eligibility_never_clips_or_promotes(LEFTfixture,mutation):
    f=LEFTfixture
    if mutation=="current_speed_high":f.dataset.examples[0].features[1]=2.6
    elif mutation=="zero_future":f.dataset.examples[0].target_speed_mps[0]=0.
    elif mutation=="missing_point":f.dataset.examples[0].valid_point_count=63
    elif mutation=="copiednative":(f.episode/"native_manifest.json").write_text("tampered")
    elif mutation in ("episodeVAL","foreignadapter"):
        value=t.read(f.episode/"episode.json")
        if mutation=="episodeVAL":value["split"]="val"
        else:value["source_provenance"]["adapter_id"]="carla_actual_town01_left347_to_common10_20261002_v2"
        (f.episode/"episode.json").write_text(json.dumps(value))
    else:
        replacement=f.root.with_name(f.root.name.replace("v3","v2"));f.root.rename(replacement);f.args.left_train=replacement
    with pytest.raises(ValueError):t.load_LEFT(f.args)


def test_source30_closure_preservesall29_no_runtime_orQA_fabrication(tmp_path):
    sources=t.source_snapshot();old=t.gpu_revision.source_snapshot()
    assert len(sources)==30 and len(old)==29 and all(sources[k]==v for k,v in old.items())
    assert not any("qualify_" in k or "collect_" in k for k in sources)
    for name in sources:
        target=tmp_path/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(t.ROOT/name,target)
    result=subprocess.run([sys.executable,"-B",str(tmp_path/"tools/workspace_python.py"),"-m","scripts.e2e.train_temporal_left_joint_20261002","--","--mode","contract","--left-conversion-sha256","a"*64],
        cwd=tmp_path,capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    value=json.loads(result.stdout);assert len(value["sources"])==30 and value["fixed_new_updates"]==20000


@pytest.mark.parametrize("sha",[None,"a"*63,"A"*64,0,True])
def test_publiccontract_never_invents_actualLEFTpin(sha):
    with pytest.raises(ValueError):t.contract(sha)


def test_researchcontract_fullroles_no_intrinsic_or_independentapproval():
    c=t.contract("a"*64)
    assert c["training_episode_count"]==19 and c["eligible_TRAIN_anchor_count"]==5879
    assert c["examples_seen"]==80000 and sum(c["exposures"][k] for k in ("recovery","old_low","legacy","LEFT"))==80000
    assert c["new_LEFT_full_cycles"]==57 and c["new_LEFT_cycle_remainder"]==221
    assert c["intrinsic_loss_added"] is False and c["legacy_decoded"] is False
    assert c["known_final_total_optimizer_updates"]==32200 and c["no_DEV_or_HOLDOUT_payload_until_verified_final"] is True
    assert c["official_main_features_approved"]==c["official_subfeatures_approved"]==0
    assert all(c[k] is False for k in ("future_model_inputs","labels_clipped","stop_intent_trained","model_promoted","vehicle_control_approved","inference_loader_authorized"))


def test_authority_exact_source_data_parent_only_before_actualfit(tmp_path,monkeypatch):
    proposed=dict(schema=t.SCHEMA,sources={"new.py":"a"*64},new_LEFT_conversion_sha256="b"*64)
    value=dict(schema=t.AUTH_SCHEMA,authorized=True,contract=proposed,contract_sha256=t.json_sha(proposed),
        actual_parent_history_sha256=t.PARENT_HISTORY_SHA,GPU_physical_index=0,model_promoted=False,vehicle_control_approved=False)
    path=tmp_path/"ROOT.json";path.write_text(json.dumps(value))
    assert t.validate_authorization(path,t.digest(path),proposed)==value
    for key,changed in (("GPU_physical_index",False),("actual_parent_history_sha256","c"*64),("model_promoted",True),("contract_sha256","d"*64)):
        wrong=dict(value,**{key:changed});path.write_text(json.dumps(wrong))
        with pytest.raises(ValueError):t.validate_authorization(path,t.digest(path),proposed)


def test_trainonly_preflight_discarded_no_optimizer_and_all_rng_restore_source():
    source=inspect.getsource(t.preflight_gradient)
    assert "AdamW(" not in source and "optimizer.step(" not in source
    assert "random.setstate(states[2])" in source and "np.random.set_state(states[3])" in source
    assert "torch.set_rng_state(states[0])" in source and "torch.cuda.set_rng_state(states[1],device)" in source
    assert "parent.state_dict()" in source and "all54" in source


def test_deferred_eval_requiresactual_final_before_firstpayload_and_fullpostchecks():
    source=inspect.getsource(t.run)
    assert source.index("verify_final(args.output,result,parent")<source.index("pair.load_evaluation_only(")
    assert source.count("pair.load_evaluation_only(")==2
    assert "pair.load_train(args)[-1]" in source and "load_LEFT(args)[1]" in source
    assert "pair.verify_training_images((*datasets,holdout,known))" in source
    assert "parent_order=original.fixed_batches(groups,strata)" in source
    assert "pair.verify_parent(args,parent_order,(training,low,legacy))" in source
    assert "gpu_revision.gpu_preflight()" in source and "with raw_parent.gpu0_lease()" in source


def test_failure_journal_does_not_promote_or_delete_partial(tmp_path,monkeypatch):
    monkeypatch.setattr(t,"_created_output",None)
    def fail(argv):
        t._created_output=tmp_path;raise RuntimeError("synthetic retained failed54")
    monkeypatch.setattr(t,"run",fail)
    with pytest.raises(RuntimeError):t.main([])
    value=t.read(tmp_path/"failure_receipt.json")
    assert value["partial_outputs_retained"] and value["GPU_lease_context_exited"]
    assert value["model_promoted"] is False and value["foreign_processes_killed"] is False
    assert t._created_output is None


def test71_original_screens_plus3LEFT_are_not_checkpoint_selection(monkeypatch):
    monkeypatch.setattr(t.original,"endpoint_screens",lambda before,after:dict(checks=[dict(name=str(i),passed=True) for i in range(71)],fixed_endpoint_updates=6000))
    left=dict(full_fixed_denominator=347,rows=[dict(token="LEFT"+str(i)) for i in range(347)],
        mean_ade_6p4s_m=.08,mean_fde_6p4s_m=.2,mean_instantaneous_speed_mae_mps=.03)
    before={"LEFT":copy.deepcopy(left)};after={"LEFT":copy.deepcopy(left)}
    report=t.endpoint_screens(before,after)
    assert len(report["checks"])==74 and report["fixed_endpoint_updates"]==20000 and report["all_diagnostic_screens_passed"]
    after["LEFT"]["mean_ade_6p4s_m"]=.11
    assert not t.endpoint_screens(before,after)["all_diagnostic_screens_passed"]
    after["LEFT"]["rows"].pop()
    with pytest.raises(ValueError):t.endpoint_screens(before,after)


@pytest.fixture
def numeric_endpoint(tmp_path,monkeypatch):
    # HH_261002 - Tiny101 scalar parameters represent the exact numeric ABI checks, not trained network outputs.
    names=["speed_head.synthetic%02d"%i for i in range(54)]+["frozen.synthetic%02d"%i for i in range(47)]
    state={name:torch.tensor([float(i)],dtype=torch.float32) for i,name in enumerate(names)}
    parent=SimpleNamespace(state_dict=lambda:state)
    output=tmp_path/"run";folder=output/"exports/step_020000";folder.mkdir(parents=True)
    checkpoints=output/"checkpoints";checkpoints.mkdir()
    batch=["rec/0","low/0","legacy/0","LEFT/0"];indices=[0,2718,3045,5832]
    declaration=dict(schema=t.SCHEMA,fixed_sample_ids=[batch]*20000,fixed_schedule_sha256=t.json_sha([indices]*20000))
    (output/"declaration.json").write_text(json.dumps(declaration))
    with (output/"history.jsonl").open("x") as stream:
        for step in range(1,20001):
            stream.write(json.dumps(dict(optimizer_updates=step,examples_seen=4*step,sample_ids=batch,indices=indices,batch_sha256="b"*64,
                learning_rates={group:t.learning_rate(step,group) for group in ("shared","heads")}))+"\n")
    pt=checkpoints/"step_020000.pt";pt.write_bytes(b"HASH_ONLY_NO_PICKLE_LOAD")
    npz=folder/"model_state.npz"
    arrays={k:v.numpy().copy() for k,v in state.items()}
    with npz.open("xb") as stream:np.savez(stream,**arrays)
    sources={"source.py":"a"*64}
    meta=dict(schema=t.EXPORT_SCHEMA,training_schema=t.SCHEMA,optimizer_updates=20000,additional_optimizer_updates=20000,
        maximum_optimizer_updates=20000,known_total_optimizer_updates=32200,examples_seen=80000,recovery_examples_seen=20000,
        v4_examples_seen=20000,legacy_examples_seen=20000,LEFT_examples_seen=20000,source_sha256=sources,
        declaration_sha256=t.digest(output/"declaration.json"),learning_rate=t.learning_rate_config(),runtime_loader_implemented=False,
        history_sha256=t.digest(output/"history.jsonl"),fixed_sample_ids_sha256=t.json_sha(declaration["fixed_sample_ids"]),
        model_promoted=False,vehicle_control_approved=False,parent_npz_sha256=t.pair.PARENT["weights_sha256"],
        parent_metadata_sha256=t.pair.PARENT["metadata_sha256"],parent_checkpoint_sha256=t.pair.PARENT["checkpoint_sha256"],
        parent_declaration_sha256=t.pair.PARENT["declaration_sha256"],checkpoint_sha256=t.digest(pt),npz_sha256=t.digest(npz),
        state_tensor_shapes={k:list(a.shape) for k,a in arrays.items()},state_tensor_dtypes={k:"float32" for k in arrays})
    description=folder/"metadata.json";description.write_text(json.dumps(meta))
    result=dict(optimizer_updates=20000,examples_seen=80000,completed_fixed_endpoint=True,
        checkpoints=[dict(optimizer_updates=step) for step in range(1000,20001,1000)])
    result["checkpoints"][-1].update(checkpoint_sha256=t.digest(pt),npz_sha256=t.digest(npz),metadata_sha256=t.digest(description))
    monkeypatch.setattr(t.replay,"verify_sources",lambda *args:None)
    return SimpleNamespace(output=output,parent=parent,result=result,sources=sources,meta=meta,description=description,npz=npz,arrays=arrays)


def repin_endpoint(f):
    f.description.write_text(json.dumps(f.meta))
    f.result["checkpoints"][-1]["metadata_sha256"]=t.digest(f.description)


def test_actual20k101finite47byteproof_before_deferred_eval_no_pickle(numeric_endpoint,monkeypatch):
    monkeypatch.setattr(torch,"load",lambda *args,**kwargs:pytest.fail("pickle must not be loaded"))
    f=numeric_endpoint;receipt_=t.verify_final(f.output,f.result,f.parent,f.sources)
    assert receipt_["optimizer_updates"]==20000 and receipt_["frozen47_byte_identical"]


@pytest.mark.parametrize("key,value",[("optimizer_updates",6000),("known_total_optimizer_updates",18200),
    ("LEFT_examples_seen",0),("runtime_loader_implemented",True),("model_promoted",True),("parent_npz_sha256","b"*64),
    ("source_sha256",{}),("declaration_sha256","c"*64)])
def test_final_endpoint_newschema_scope_cannot_masquerade(numeric_endpoint,key,value):
    f=numeric_endpoint;f.meta[key]=value;repin_endpoint(f)
    with pytest.raises(ValueError):t.verify_final(f.output,f.result,f.parent,f.sources)


@pytest.mark.parametrize("mutation",["NaN","float64","shape","missing","frozenchanged"])
def test_full_numeric_endpoint_finite_exactkeys_shapes_and47bytes(numeric_endpoint,mutation):
    f=numeric_endpoint
    name=next(k for k in f.arrays if not k.startswith(t.original.TRAINABLE_PREFIXES))
    if mutation=="NaN":f.arrays[name][0]=np.nan
    elif mutation=="float64":f.arrays[name]=f.arrays[name].astype(np.float64)
    elif mutation=="shape":f.arrays[name]=np.repeat(f.arrays[name],2)
    elif mutation=="missing":f.arrays.pop(name)
    else:f.arrays[name][0]+=.25
    with f.npz.open("wb") as stream:np.savez(stream,**f.arrays)
    f.meta["npz_sha256"]=f.result["checkpoints"][-1]["npz_sha256"]=t.digest(f.npz);repin_endpoint(f)
    with pytest.raises(ValueError):t.verify_final(f.output,f.result,f.parent,f.sources)


@pytest.mark.parametrize("mutation",["incomplete","lost_checkpoint","checkpoint_corruption"])
def test_boolean_completion_not_sufficient_to_open_evaluation(numeric_endpoint,mutation):
    f=numeric_endpoint
    if mutation=="incomplete":f.result["optimizer_updates"]=19999
    elif mutation=="lost_checkpoint":f.result["checkpoints"].pop(0)
    else:(f.output/"checkpoints/step_020000.pt").write_bytes(b"changed")
    with pytest.raises(ValueError):t.verify_final(f.output,f.result,f.parent,f.sources)


def test_finite_individual_gradient_norm_overflow_still_blocks_step():
    parameter=torch.nn.Parameter(torch.ones(2));parameter.grad=torch.tensor([1e38,1e38])
    with pytest.raises(ValueError,match="aggregate joint gradient norm"):
        t.original.finite_clip_gradients(SimpleNamespace(parameters=lambda:[parameter]))


@pytest.mark.parametrize("mutation",["history_rows","sample_order","batch_index","LR","batch_sha"])
def test_all20k_actualrows_required_not_fitboolean(numeric_endpoint,mutation):
    f=numeric_endpoint;path=f.output/"history.jsonl"
    rows=[json.loads(line) for line in path.read_text().splitlines()]
    if mutation=="history_rows":rows.pop()
    elif mutation=="sample_order":rows[0]["sample_ids"][3]="knownDEV/0"
    elif mutation=="batch_index":rows[0]["indices"][3]=0
    elif mutation=="LR":rows[0]["learning_rates"]["heads"]=.5
    else:rows[0]["batch_sha256"]="unknown"
    path.write_text("\n".join(json.dumps(row) for row in rows)+"\n")
    f.meta["history_sha256"]=t.digest(path);repin_endpoint(f)
    with pytest.raises(ValueError,match="all actual20k"):t.verify_final(f.output,f.result,f.parent,f.sources)
