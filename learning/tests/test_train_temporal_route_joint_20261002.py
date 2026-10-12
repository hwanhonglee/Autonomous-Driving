"""HH_261002 - Verify actual-domain sampling, joint-speed gradients and honest partial-freeze protocols."""
import copy
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
from types import SimpleNamespace

import pytest
import torch

from scripts.e2e import train_temporal_route_joint_20261002 as trainer
from portable_e2e import temporal_route_joint_export_research_20261002 as loader
from portable_e2e.temporal_route_recovery_v3_20261001 import RouteRecoveryTemporalModelV3
from scripts.e2e import train_temporal_route_recovery_v3_20261001 as previous


def groups():
    full = {p:list(range(i*302,(i+1)*302)) for i,p in enumerate(sorted(previous.TRAIN_PROFILES))}
    focus = {(-1,-1):full[(-.2,-1.5)][:7], (-1,0):full[(-.2,0.)][:19], (-1,1):full[(-.2,1.5)][:115],
        (0,-1):full[(0.,-1.5)][:104], (0,1):full[(0.,1.5)][:63], (1,-1):full[(.2,-1.5)][:116],
        (1,0):full[(.2,0.)][:9], (1,1):full[(.2,1.5)][:7]}
    old = dict(curved={"C_track_1_0_7":list(range(260)), "Town01":list(range(260,328))},
        straight={"C_track_1_0_7":list(range(328,990)), "Town01":list(range(990,1560)), "Town07":list(range(1560,2487))})
    return dict(full=full,focus=focus),old


def test_fixed6000_actual_domains_alltails_no_val_test_or_repeated_view():
    g,old = groups(); rows=trainer.fixed_batches(g,old)
    assert len(rows)==6000 and all(len(r)==4 for r in rows)
    assert rows==trainer.fixed_batches(g,old)
    assert {r[0] for r in rows}=={i for v in g["focus"].values() for i in v}
    assert {r[1] for r in rows}==set(range(2718))
    assert {r[2]-2718 for r in rows}==set(range(327))
    assert {r[3]-3045 for r in rows}==set(range(2487))
    assert all(0<=r[0]<2718 and 0<=r[1]<2718 and 2718<=r[2]<3045 and 3045<=r[3]<5532 for r in rows)
    for step,r in enumerate(rows):
        kind="curved" if step%2==0 else "straight"
        assert r[3]-3045 in {i for v in old[kind].values() for i in v}


def test_actual_focus_empty_fail_closed_not_center_fake():
    g,old=groups(); g["focus"]={}
    with pytest.raises(ValueError): trainer.fixed_batches(g,old)


def test_loss_exact_per_anchor_joint_and_new_legacy_speed_weights():
    values=[torch.tensor(float(i+1),requires_grad=True) for i in range(5)]
    result=trainer.exact_objective(*values); result.backward()
    assert float(result)==3.
    assert [float(v.grad) for v in values]==[.25,.25,.25,.0625,.25]


def test_finite_individual_gradient_cannot_bypass_overflowing_aggregate_norm():
    parameter=torch.nn.Parameter(torch.ones(2)); parameter.grad=torch.full((2,),1e38)
    model=SimpleNamespace(parameters=lambda:[parameter])
    assert bool(torch.isfinite(parameter.grad).all())
    with pytest.raises(ValueError,match="aggregate joint gradient norm"): trainer.finite_clip_gradients(model)
    parameter.grad=torch.tensor([3.,4.]); assert float(trainer.finite_clip_gradients(model))==5.


def test_freeze54_learned47_exact_including_visual_and_semantic_heads():
    model=RouteRecoveryTemporalModelV3(); names=trainer.freeze_inputs(model)
    assert len(model.state_dict())==101 and len(names)==54
    assert sum(p.numel() for p in model.parameters() if p.requires_grad)==2277827
    assert len(set(model.state_dict())-set(names))==47
    assert model.speed_head.weight.requires_grad and model.curvature_head.weight.requires_grad
    assert model.trunk[0].weight.requires_grad and model.fusion[0].context_norm.weight.requires_grad
    assert not model.intent_head.weight.requires_grad and not model.stop_distance_head.weight.requires_grad
    assert not model.acceleration_head.weight.requires_grad and not model.camera_embedding.requires_grad
    assert not any(p.requires_grad for p in model.image_encoder.parameters())


def test_frozen_hash_ignores_only_declared54_and_detects_any_semantic_visual_change():
    model=RouteRecoveryTemporalModelV3(); state=model.state_dict(); initial=loader.frozen_fingerprint(state)
    state["speed_head.bias"][0]+=1
    assert loader.frozen_fingerprint(state)==initial
    state["intent_head.bias"][0]+=1
    assert loader.frozen_fingerprint(state)!=initial


def test_optimizer_groups_partition54_without_duplicate_params():
    model=RouteRecoveryTemporalModelV3(); names=trainer.freeze_inputs(model); groups_=trainer.optimizer_groups(model)
    actual=[id(p) for g in groups_ for p in g["params"]]
    assert len(actual)==len(set(actual))==54
    assert [g["name"] for g in groups_]==["shared","heads"]
    assert [len(g["params"]) for g in groups_]==[46,8]


@pytest.mark.parametrize("group,peak,last",[("shared",1e-5,1e-6),("heads",3e-5,3e-6)])
def test_fixed6000_cosine_no_scheduler_reset(group,peak,last):
    assert trainer.learning_rate(1,group)==pytest.approx(peak/300)
    assert trainer.learning_rate(300,group)==peak
    assert trainer.learning_rate(6000,group)==last
    assert trainer.learning_rate(3000,group)>last


@pytest.mark.parametrize("step",[True,0,6001,300.])
def test_learning_rate_rejects_other_endpoint_or_bool(step):
    with pytest.raises(ValueError): trainer.learning_rate(step,"shared")


def test_source26_closure_full_original24_pins_and_fresh_clone_import(tmp_path):
    sources=trainer.source_snapshot(); originals=trainer.moving.source_snapshot()
    assert len(originals)==24 and len(sources)==26
    assert all(sources[k]==sha for k,sha in originals.items())
    assert not any("collect_" in k or "simulation_research" in k for k in sources)
    for name in sources:
        target=tmp_path/name; target.parent.mkdir(parents=True,exist_ok=True); shutil.copyfile(trainer.ROOT/name,target)
    script="from scripts.e2e import train_temporal_route_joint_20261002 as t; import torch; assert len(t.source_snapshot())==26; assert not torch.cuda.is_initialized(); print('FRESH26_CPU_IMPORT_PASS')"
    result=subprocess.run([sys.executable,"-B",str(tmp_path/"tools/workspace_python.py"),"-m","scripts.e2e.train_temporal_route_joint_20261002","--","--help"],capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    probe=subprocess.run([sys.executable,"-B","-c","import runpy; x=runpy.run_path("+repr(str(tmp_path/"tools/workspace_python.py"))+"); x['configure'](); "+script],cwd=tmp_path,capture_output=True,text=True)
    assert probe.returncode==0,probe.stderr
    assert "FRESH26_CPU_IMPORT_PASS" in probe.stdout


def metric_fixture():
    result={}
    for cohort,episodes,count in (("recovery_train",9,2718),("recovery_holdout",2,604),("legacy",8,2487),("known599",2,599)):
        rows=[dict(index=i,token=cohort+str(i)) for i in range(count)]
        per={cohort+"_e"+str(i):dict(ade_6p4s_m=.08,fde_6p4s_m=.2,instantaneous_speed_mae_mps=.1,
            raw_curvature=.01,raw_spatial_preferred_speed_mae_mps=.2) for i in range(episodes)}
        result[cohort]=dict(rows=rows,per_episode=per,groups={"all_eligible":dict(raw_curvature=.01,raw_spatial_preferred_speed_mae_mps=.2)},
            full_fixed_denominator=count,full_fixed_eligible_denominator=count)
    result["v4"]=dict(full_fixed_denominator=327,rows=[dict(token="v4"+str(i)) for i in range(327)],
        mean_ade_6p4s_m=.08,mean_fde_6p4s_m=.2,mean_instantaneous_speed_mae_mps=.02)
    return result


def test_all71_preserved_absolute_criteria_plus_actual_speed_gain_and_rawspeed_guards():
    before=metric_fixture(); after=copy.deepcopy(before)
    for cohort in ("recovery_train","recovery_holdout"):
        for row in after[cohort]["per_episode"].values(): row["instantaneous_speed_mae_mps"] = .07
    result=trainer.endpoint_screens(before,after)
    assert result["diagnostic_screen_count"]==71 and result["all_diagnostic_screens_passed"]
    assert result["fixed_endpoint_updates"]==6000 and result["retained_original_screen_count"]==48
    after["legacy"]["per_episode"]["legacy_e0"]["raw_spatial_preferred_speed_mae_mps"] = .22
    result=trainer.endpoint_screens(before,after)
    assert len([r for r in result["checks"] if not r["passed"]])==1
    assert not result["model_promoted"] and not result["feature_approved"]


def test_same_speed_not_reported_as_joint_learning_success():
    data=metric_fixture(); result=trainer.endpoint_screens(data,copy.deepcopy(data))
    assert len([r for r in result["checks"] if not r["passed"]])==11


def test_scope_denies_old_branch_only_frozen97_and_stop_output_preservation():
    scope=trainer.scope_fields()
    assert scope["backbone_frozen"] is False and scope["visual_encoder_frozen"] is True
    assert scope["semantic_head_parameters_frozen"] is True and scope["semantic_outputs_preserved"] is False
    assert scope["stop_intent_trained"] is False and scope["official_main_features_approved"]==0
    assert scope["official_subfeatures_approved"]==0 and scope["startup_recovery_fitted"] is False
