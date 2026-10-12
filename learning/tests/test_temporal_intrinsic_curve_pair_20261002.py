"""HH_261002 - Synthetic protocol/math tests only; no actual data/model/CUDA/vehicle jobs."""
import ast
import copy
import hashlib
import inspect
import json
from pathlib import Path
import shutil
import subprocess
import sys
from types import SimpleNamespace
from unittest import mock

import pytest
import numpy as np
import torch

from scripts.e2e import train_temporal_intrinsic_curve_pair_20261002 as pair


def batch():
    xy=torch.zeros(1,64,2);xy[0,:,0]=torch.arange(1,65)*.5
    return dict(target_xy=xy,target_valid=torch.ones(1,64,dtype=torch.bool))


def raw():
    return dict(curvature_knots=torch.zeros(1,33,requires_grad=True))


def old_terms(output,unused):
    value=output["curvature_knots"].sum()*0
    return dict(raw_curvature=value+2.,raw_preferred_speed=value+3.,total=value+5.)


class Spatial:
    def __init__(self):self.calls=[]
    def __call__(self,*args):
        self.calls.append(args)
        return dict(total=args[0].sum()*0+7.,nonzero_supervision_present=True)


@pytest.mark.parametrize("arm,is_curved,expected",[("control",True,3.5),("control",False,3.5),
    ("intrinsic_curve",False,3.5),("intrinsic_curve",True,3.85)])
def test_single_domain_only_adds_fixed_intrinsic_for_candidate_real_curve(arm,is_curved,expected):
    module=Spatial();output=raw();labels=batch();before=labels["target_xy"].clone()
    with mock.patch.object(pair.original,"temporal_trajectory_loss_v2",side_effect=old_terms):
        domain,old,extra,coverage=pair.intrinsic_domain(output,labels,is_curved,arm,module)
    assert float(domain)==pytest.approx(expected)
    assert float(old["raw_curvature"])==2. and float(old["raw_preferred_speed"])==3.
    assert len(module.calls)==int(arm=="intrinsic_curve" and is_curved)
    assert torch.equal(labels["target_xy"],before) and labels["target_valid"].all()
    assert float(extra)==(7. if module.calls else 0.)
    assert (coverage is not None)==bool(module.calls)


@pytest.mark.parametrize("is_curved,arm",[(1,"control"),(False,"best_DEV"),(None,"intrinsic_curve")])
def test_no_untyped_stratum_or_outcome_selected_arm(is_curved,arm):
    with pytest.raises(Exception):pair.intrinsic_domain(raw(),batch(),is_curved,arm,Spatial())


def test_empty_curve_supervision_fails_instead_of_advertising_zero_loss_gain():
    empty=lambda *args:dict(total=args[0].sum()*0,nonzero_supervision_present=False)
    with mock.patch.object(pair.original,"temporal_trajectory_loss_v2",side_effect=old_terms),pytest.raises(Exception,match="lacks actual"):
        pair.intrinsic_domain(raw(),batch(),True,"intrinsic_curve",empty)


def test_pair_objective_matches_original_and_extra_has_explicit_quarter_domain_weight():
    labels={k:v.expand(4,*v.shape[1:]).clone() for k,v in batch().items()};module=Spatial()
    def joint(*args):return None,None,dict(total=torch.tensor(1.))
    with mock.patch.object(pair.joint,"joint_forward",side_effect=joint),mock.patch.object(pair.raw_parent,"forward",return_value=raw()),\
        mock.patch.object(pair.original,"temporal_trajectory_loss_v2",side_effect=old_terms):
        control=pair.domains(None,None,labels,True,"control",module)[0]
        candidate=pair.domains(None,None,labels,True,"intrinsic_curve",module)[0]
    assert float(control)==float(pair.original.exact_objective(1.,1.,1.,2.,3.))
    assert float(candidate-control)==pytest.approx(.25*.05*7.)


def fake_dataset(tokens,episodes,split="train"):
    class Dataset:
        def __init__(self):
            self.split=split;self.examples=[SimpleNamespace(token=t,episode_id=e) for t,e in zip(tokens,episodes)]
        def __len__(self):return len(self.examples)
    return Dataset()


@pytest.mark.parametrize("kind",["val","repeat_token","repeat_episode","duplicate_own"])
def test_loss_dataset_guard_rejects_VAL_and_repeated_views(kind):
    a=fake_dataset(["a"],["one"]);b=fake_dataset(["b"],["two"])
    if kind=="val":b.split="val"
    if kind=="repeat_token":b.examples[0].token="a"
    if kind=="repeat_episode":b.examples[0].episode_id="one"
    if kind=="duplicate_own":b.examples.append(copy.copy(b.examples[0]))
    with pytest.raises(Exception):pair.disjoint((a,b))


def test_actual_batch_hash_binds_tensor_dtype_shape_bytes_and_sample_ids():
    first=dict(images=torch.tensor([[.25,.5]]),sample_id=["source-TRAIN"])
    assert pair.batch_digest(first)==pair.batch_digest(copy.deepcopy(first))
    for changed in (dict(first,images=first["images"].double()),dict(first,images=first["images"].reshape(2,1)),
        dict(first,images=first["images"]+.001),dict(first,sample_id=["repeated-DEV"])):
        assert pair.batch_digest(first)!=pair.batch_digest(changed)


def test_no_gpu_tensor_hash_before_actual_cpu_input_witness():
    # HH_261002 - Meta device is synthetic; constructing this tensor initializes no CUDA.
    with pytest.raises(Exception):pair.batch_digest(dict(images=torch.empty(1,device="meta")))


def strata():
    return dict(curved={"C_track_1_0_7":list(range(260)),"Town01":list(range(260,328))},
        straight={"C_track_1_0_7":list(range(328,990)),"Town01":list(range(990,1560)),"Town07":list(range(1560,2487))},
        eligible=list(range(2487)),excluded=list(range(2487,2787)))


def groups():
    return dict(focus={(-1,-1):list(range(7)),(1,1):list(range(7,14))},
        full={(i//3,i%3):list(range(i*302,(i+1)*302)) for i in range(9)})


def test_exact_frozen6000_sampler_full_denominator_and_pair_identity():
    a=pair.original.fixed_batches(groups(),strata());b=pair.original.fixed_batches(groups(),strata())
    assert a==b and len(a)==6000 and len({tuple(row) for row in a})>5900
    assert sum(len(row) for row in a)==24000
    assert all(len(row)==4 and 0<=row[0]<2718 and 0<=row[1]<2718 and 2718<=row[2]<3045 and 3045<=row[3]<5532 for row in a)
    curved=set(pair.curved_indices(strata()))
    assert sum(row[3]-3045 in curved for row in a)==3000
    assert {row[3]-3045 for row in a if row[3]-3045 in curved}==curved
    assert {row[2]-2718 for row in a}==set(range(327))
    assert {row[1] for row in a}==set(range(2718))
    assert all((row[3]-3045 in curved)==(i%2==0) for i,row in enumerate(a))


def test_protocol_exact_actual_parent6000_not_old3000_and_unchanged_LR54_47():
    value=pair.contract()
    assert value["actual_parent_final6000"]==pair.PARENT
    assert value["parent_optimizer_updates"]==6000 and value["parent_known_total_optimizer_updates"]==12200
    assert value["fixed_new_updates_per_arm"]==6000 and value["coefficient"]==.05
    assert value["trainable_tensor_count"]==54 and value["frozen_tensor_count"]==47
    assert value["learning_rate"]==pair.original.learning_rate_config()
    assert value["no_DEV_or_HOLDOUT_payload_until_both_final"] and not value["model_promoted"]
    assert pair.original.learning_rate(6000,"shared")==pytest.approx(1e-6)
    assert pair.original.learning_rate(6000,"heads")==pytest.approx(3e-6)


def authority(contract,history="a"*64):
    return dict(schema=pair.AUTH_SCHEMA,authorized=True,contract=contract,contract_sha256=pair.json_sha(contract),
        GPU_physical_index=0,model_promoted=False,vehicle_control_approved=False,actual_parent_history_sha256=history)


@pytest.mark.parametrize("change",["GPU1","booleanGPU","parent3000","coefficient","sources","promotion","wrongSHA"])
def test_external_authorization_rejects_scope_parent_coefficient_source_mutations(tmp_path,change):
    proposed=pair.contract();value=authority(proposed);value=copy.deepcopy(value)
    if change=="GPU1":value["GPU_physical_index"]=1
    if change=="booleanGPU":value["GPU_physical_index"]=False
    if change=="parent3000":value["contract"]["parent_optimizer_updates"]=3000
    if change=="coefficient":value["contract"]["coefficient"]=.01
    if change=="sources":value["contract"]["sources"].clear()
    if change=="promotion":value["model_promoted"]=True
    path=tmp_path/"auth.json";path.write_text(json.dumps(value));sha=pair.digest(path)
    if change=="wrongSHA":sha="0"*64
    with pytest.raises(Exception):pair.validate_authorization(path,sha,proposed)


def test_external_authorization_accepts_exact_contract_and_binds_actual_parent_history(tmp_path):
    proposed=pair.contract();value=authority(proposed);path=tmp_path/"auth.json";path.write_text(json.dumps(value))
    result=pair.validate_authorization(path,pair.digest(path),proposed);pair.authorize_parent_history(result,"a"*64)
    with pytest.raises(Exception):pair.authorize_parent_history(result,"b"*64)


def test_regular_rejects_leaf_ancestor_symlink_and_escape(tmp_path):
    inside=tmp_path/"inside";inside.mkdir();leaf=inside/"file";leaf.write_text("original")
    assert pair.regular(leaf,tmp_path)==leaf
    linked=tmp_path/"linked";linked.symlink_to(inside,target_is_directory=True)
    for path in (linked/"file",inside/".."/"inside"/"file"):
        with pytest.raises(Exception):pair.regular(path,tmp_path)
    with pytest.raises(Exception):pair.regular(leaf,inside/"elsewhere")


def test_source_snapshot_retains_all26_and_imports_exact_new_loss_and_runner():
    previous=pair.original.source_snapshot();sources=pair.source_snapshot()
    assert len(previous)==26 and len(sources)==28
    assert all(sources[k]==sha for k,sha in previous.items())
    assert sources["learning/portable_e2e/intrinsic_spatial_path_loss_v1_20261002.py"]=="051a07c032c9335083b2e004711f58460e7e412aecb98e362c061e0a9a8ae314"
    assert "tools/e2e/train_temporal_intrinsic_curve_pair_20261002.py" in sources
    assert "learning/tests/test_temporal_intrinsic_curve_pair_20261002.py" not in sources


def test_fresh_minimal28_source_clone_public_contract_initializes_no_CUDA(tmp_path):
    for name,sha in pair.source_snapshot().items():
        target=tmp_path/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(pair.ROOT/name,target)
        assert hashlib.sha256(target.read_bytes()).hexdigest()==sha
    result=subprocess.run([sys.executable,"-B",str(tmp_path/"tools/workspace_python.py"),"-m",
        "scripts.e2e.train_temporal_intrinsic_curve_pair_20261002","--","--mode","contract"],
        cwd=tmp_path,text=True,capture_output=True,timeout=45)
    assert result.returncode==0,result.stderr
    value=json.loads(result.stdout);assert value["sources"]==pair.source_snapshot() and value["fixed_new_updates_per_arm"]==6000


def test_all_frozen_helpers_are_called_not_copied_or_semantically_rewritten():
    source=inspect.getsource(pair)
    assert "original.exact_objective" in inspect.getsource(pair.contract)
    assert "original.fixed_batches(groups,strata)" in source
    assert "original.learning_rate(step,group[\"name\"])" in source
    assert "original.endpoint_screens(metrics[\"parent\"],metrics[arm])" in source
    assert "original.finite_clip_gradients(model)" in source
    assert "load_route_joint_research_npz" in inspect.getsource(pair.verify_parent)
    assert "torch.load" not in source


def test_no_DEV_HOLDOUT_loader_in_training_or_preflight_closure():
    train_source=inspect.getsource(pair.load_train)+inspect.getsource(pair.preflight_gradient)+inspect.getsource(pair.train_arm)
    assert "DEV_PATH" not in train_source and '"val"' not in train_source and "load_evaluation_only" not in train_source
    assert "replay.load_cohorts(" not in train_source
    main=ast.parse(inspect.getsource(pair.run));body=main.body[0]
    lease=next(n for n in body.body if isinstance(n,ast.With))
    fit_loop=next(n for n in lease.body if isinstance(n,ast.For))
    eval_position=next(i for i,n in enumerate(lease.body) if "load_evaluation_only(" in ast.unparse(n))
    assert lease.body.index(fit_loop)<eval_position
    assert "if not result['completed_fixed_endpoint']" in ast.unparse(fit_loop)
    assert "return 2" in ast.unparse(fit_loop)
    assert "verify_final_endpoint(" in ast.unparse(fit_loop)


def test_new_export_truthful_lineage_denies_old_runtime_loader_and_promotions():
    source=inspect.getsource(pair.save_checkpoint)
    assert "schema=EXPORT_SCHEMA" in source and "known_total_optimizer_updates=12200+step" in source
    assert "runtime_loader_implemented=False" in source
    assert pair.EXPORT_SCHEMA!=pair.original.EXPORT_SCHEMA
    assert "simulator_control_authorized=True" not in inspect.getsource(pair)


def test_coverage_keeps_outside64_and_current_origin_and_requires_nonzero_actual_points():
    row=SimpleNamespace(token="actual-TRAIN",map_id="Town01")
    dataset=SimpleNamespace(examples=[row])
    xy=torch.zeros(64,2);xy[:,0]=torch.arange(1,65)*2.
    dataset._targets=lambda value:(xy,torch.zeros(64),torch.ones(64)*8.,torch.ones(64,dtype=torch.bool))
    with mock.patch.object(pair,"curved_indices",return_value=[0]):
        value=pair.spatial_inventory(dataset,{})
        assert value["anchor_count"]==1 and value["original_valid_points"]==64
        assert value["assessed_points"]==32 and value["outside_64m_points"]==32
        assert value["rows"][0]["full_actual_chord_arc_m"]==128.
        dataset._targets=lambda unused:(torch.zeros(64,2),torch.zeros(64),torch.zeros(64),torch.ones(64,dtype=torch.bool))
        with pytest.raises(Exception,match="empty intrinsic"):pair.spatial_inventory(dataset,{})


def test_control_no_intrinsic_or_legacy_speed_rewrite_before_existing_raw_loss():
    source=inspect.getsource(pair.intrinsic_domain)
    assert '.25*old["raw_curvature"]+old["raw_preferred_speed"]' in source
    assert "speed" not in inspect.getsource(pair.spatial_inventory)
    assert "target_xy" in source and "target_valid" in source


@pytest.mark.parametrize("fails",[True,False])
def test_failure_receipt_retains_partial_and_global_state_restores(tmp_path,fails):
    def fake_run(argv):
        pair._created_output=tmp_path
        if fails:raise RuntimeError("original partial failure")
        return 0
    with mock.patch.object(pair,"run",side_effect=fake_run):
        if fails:
            with pytest.raises(RuntimeError,match="original partial failure"):pair.main([])
            value=json.loads((tmp_path/"failure_receipt.json").read_text())
            assert value["partial_outputs_retained"] and value["GPU_lease_context_exited"] and not value["model_promoted"]
        else:assert pair.main([])==0 and not (tmp_path/"failure_receipt.json").exists()
    assert pair._created_output is None


def test_DEFERRED_evaluation_pins_only_declared_episode_directories_not_other_TEST(tmp_path):
    hold=tmp_path/"hold";known=tmp_path/"known"
    for root,episode in ((hold,"HO"),(known,"DEV")):
        folder=root/"episodes"/episode;folder.mkdir(parents=True);(root/"dataset.json").write_text("{}");(folder/"manifest.json").write_text("{}")
        protected=root/"episodes/PROTECTED_TEST";protected.mkdir();(protected/"labels.json").write_text("NEVER OPEN")
    conversion=hold/"moving_conversion_qualification.json";conversion.write_text("{}");(hold/"signed_current_state_audit.jsonl").write_text("{}")
    receipt=dict(validated_fingerprint_sha256=pair.replay.DEV_LOADER_FP,temporal_fingerprint_sha256=pair.replay.DEV_TEMPORAL_FP)
    args=SimpleNamespace(recovery_holdout=hold,holdout_conversion=conversion)
    with mock.patch.object(pair.original,"PERSONAL",tmp_path),mock.patch.object(pair.replay,"DEV_PATH",known),\
        mock.patch.object(pair.original,"HOLDOUT_RECEIPT_SHA",pair.digest(conversion)):
        values=pair.evaluation_artifact_pins(args,receipt,fake_dataset(["h"],["HO"],"val"),fake_dataset(["d"],["DEV"],"val"))
    assert len(values)==6 and all("PROTECTED_TEST" not in name for name in values)


def test_actual_endpoint_gate_cannot_be_a_fit_boolean_without_files(tmp_path):
    fake=dict(arm="control",optimizer_updates=6000,examples_seen=24000,completed_fixed_endpoint=True,
        checkpoints=[dict(optimizer_updates=step,checkpoint_sha256="a"*64,npz_sha256="b"*64,metadata_sha256="c"*64) for step in range(500,6001,500)])
    with pytest.raises(ValueError,match="regular pinned file required"):pair.verify_final_endpoint(tmp_path,fake,None,{})
    fake["checkpoints"].pop()
    with pytest.raises(Exception,match="all12 actual"):pair.verify_final_endpoint(tmp_path,fake,None,{})


def test_postfit_full_loader_fingerprint_and_all_image_integrity_are_not_TRAIN_only():
    source=inspect.getsource(pair.run)
    assert "post_train=load_train(args)" in source and "post_train[-1]==receipts" in source
    assert "post_ho_receipt==ho_receipt" in source and "post_known_receipt==known_receipt" in source
    assert "verify_training_images((training,low,legacy,holdout,known))" in source


@pytest.mark.parametrize("corruption",[None,"fixed47","dtype","nonfinite","missingkey","forgedcounts"])
def test_actual_final_numeric_gate_full101_and47_parent_bytes(tmp_path,corruption):
    state={"fusion.synthetic_%02d"%i:torch.ones(1) for i in range(54)}
    state.update({"input.synthetic_%02d"%i:torch.ones(1) for i in range(47)})
    parent=SimpleNamespace(state_dict=lambda:state)
    values={k:v.numpy().copy() for k,v in state.items()}
    if corruption=="fixed47":values["input.synthetic_00"][:]=2.
    if corruption=="dtype":values["fusion.synthetic_00"]=values["fusion.synthetic_00"].astype(np.float64)
    if corruption=="nonfinite":values["fusion.synthetic_00"][:]=np.nan
    if corruption=="missingkey":values.pop("fusion.synthetic_00")
    export=tmp_path/"exports/step_006000";export.mkdir(parents=True)
    checkpoints=tmp_path/"checkpoints";checkpoints.mkdir()
    checkpoint=checkpoints/"step_006000.pt";checkpoint.write_bytes(b"HASH ONLY, NEVER PICKLE")
    declaration=tmp_path/"declaration.json";declaration.write_text("{}")
    with (export/"model_state.npz").open("xb") as stream:np.savez(stream,**values)
    metadata=dict(schema=pair.EXPORT_SCHEMA,training_schema=pair.SCHEMA,arm="control",optimizer_updates=6000,
        additional_optimizer_updates=6000,known_total_optimizer_updates=18200,examples_seen=24000,
        checkpoint_sha256=pair.digest(checkpoint),npz_sha256=pair.digest(export/"model_state.npz"),source_sha256={},
        declaration_sha256=pair.digest(declaration),parent_npz_sha256=pair.PARENT["weights_sha256"],
        parent_metadata_sha256=pair.PARENT["metadata_sha256"],parent_checkpoint_sha256=pair.PARENT["checkpoint_sha256"],
        parent_declaration_sha256=pair.PARENT["declaration_sha256"],model_promoted=False,vehicle_control_approved=False,
        runtime_loader_implemented=False,state_tensor_shapes={k:[1] for k in state},state_tensor_dtypes={k:"float32" for k in state})
    if corruption=="forgedcounts":metadata["optimizer_updates"]=3000
    (export/"metadata.json").write_text(json.dumps(metadata))
    receipts=[dict(optimizer_updates=step) for step in range(500,6001,500)]
    receipts[-1].update(checkpoint_sha256=pair.digest(checkpoint),npz_sha256=pair.digest(export/"model_state.npz"),metadata_sha256=pair.digest(export/"metadata.json"))
    result=dict(arm="control",optimizer_updates=6000,examples_seen=24000,completed_fixed_endpoint=True,checkpoints=receipts)
    with mock.patch.object(pair.replay,"verify_sources"):
        if corruption is None:
            receipt=pair.verify_final_endpoint(tmp_path,result,parent,{})
            assert receipt["all101_numeric_finite"] and receipt["frozen47_exact"]
        else:
            with pytest.raises(Exception):pair.verify_final_endpoint(tmp_path,result,parent,{})


def test_control_objective_exact_float32_operation_order_not_reassociated():
    labels={k:v.expand(4,*v.shape[1:]).clone() for k,v in batch().items()}
    scalars=[torch.tensor(.072102576),torch.tensor(.0000031728),torch.tensor(.12374672)]
    curvature,speed=torch.tensor(.03186111),torch.tensor(.0000002142)
    calls=iter(scalars)
    with mock.patch.object(pair.joint,"joint_forward",side_effect=lambda *a:(None,None,dict(total=next(calls)))),\
        mock.patch.object(pair.raw_parent,"forward",return_value=raw()),\
        mock.patch.object(pair.original,"temporal_trajectory_loss_v2",return_value=dict(raw_curvature=curvature,raw_preferred_speed=speed)):
        result=pair.domains(None,None,labels,True,"control",Spatial())[0]
    assert torch.equal(result,pair.original.exact_objective(*scalars,curvature,speed))
