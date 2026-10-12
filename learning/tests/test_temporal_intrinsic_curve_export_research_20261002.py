"""HH_261002 - Synthetic101-state paired files and source pins; no actual weights/data/CUDA/vehicle."""
import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
import torch
from portable_e2e import temporal_intrinsic_curve_export_research_20261002 as loader
from scripts.e2e import train_temporal_intrinsic_curve_pair_v2_20261002 as trainer


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(value,sort_keys=True,allow_nan=False))


class FakeModel(torch.nn.Module):
    def __init__(self,config=None,recovery_config=None):
        super().__init__();self.config=config or SimpleNamespace(to_dict=lambda:{"synthetic101":True})
        self.recovery_config=recovery_config or SimpleNamespace(to_dict=lambda:{"syntheticRecovery":True})
        for prefix,count in (("speed_head",54),("intent_head",47)):
            module=torch.nn.Module()
            for i in range(count):module.register_parameter("p%02d"%i,torch.nn.Parameter(torch.tensor([i/100.],dtype=torch.float32)))
            self.add_module(prefix,module)


def owner(d,metadata_sha,authority_sha,purpose="offline_cpu_numeric_validation_only"):
    return dict(schema=loader.OWNER_SCHEMA,mode="RESEARCH_ONLY",owner_id="synthetic-no-control",purpose=purpose,
        arm=d["arm"],completed_pair_authority_sha256=authority_sha,weights_sha256=d["npz_sha256"],metadata_sha256=metadata_sha,
        checkpoint_sha256=d["checkpoint_sha256"],declaration_sha256=d["declaration_sha256"],fixed_endpoint_updates=6000,
        simulator_control_authorized=False,model_promoted=False,vehicle_control_approved=False)


@pytest.fixture
def pair(tmp_path,monkeypatch,frozen_protocol):
    # HH_261002 - Reuse the same immutable source-only protocol census; each loader still rechecks every pinned live source.
    protocol=copy.deepcopy(frozen_protocol);parent=FakeModel();sources=protocol["sources"]
    old_sources={k:v for k,v in sources.items() if k not in loader.ADDED_TRAINING_SOURCES}
    assets={name:tmp_path/(name+".npz" if name.endswith("weights") else name+".json") for name in
        ("current_parent_weights","current_parent_metadata","warm_parent_weights","warm_parent_metadata","backbone_parent_weights","backbone_parent_metadata")}
    for path in assets.values():path.write_bytes(b"synthetic ancestor pin only")
    write(assets["warm_parent_metadata"],dict(parent_npz_sha256=digest(assets["backbone_parent_weights"]),
        parent_metadata_sha256=digest(assets["backbone_parent_metadata"])))
    parent._research_temporal_export_metadata=dict(source_sha256=old_sources,npz_sha256=digest(assets["current_parent_weights"]),
        parent_npz_sha256=digest(assets["warm_parent_weights"]),parent_metadata_sha256=digest(assets["warm_parent_metadata"]))
    parent._research_temporal_metadata_sha256=digest(assets["current_parent_metadata"])
    monkeypatch.setattr(loader,"load_current_parent",lambda *a,**k:parent)
    monkeypatch.setattr(loader.old,"RouteRecoveryTemporalModelV3",FakeModel)
    monkeypatch.setattr(torch,"load",lambda *a,**k:(_ for _ in ()).throw(AssertionError("Torch pickle forbidden")))
    run=tmp_path/"pair";reference=parent.state_dict();names=loader.trainable_names(reference)
    base=dict(trainer.pair.original.scope_fields(),training_schema=loader.TRAINING_SCHEMA,schema=loader.EXPORT_SCHEMA,
        architecture_schema=loader.old.ARCHITECTURE_SCHEMA,model_id=loader.old.MODEL_ID,protocol=protocol,
        optimizer_updates=6000,additional_optimizer_updates=6000,maximum_optimizer_updates=6000,effective_batch_size=4,
        examples_seen=24000,recovery_examples_seen=12000,v4_examples_seen=6000,legacy_examples_seen=6000,
        parent_optimizer_updates=6000,parent_known_total_optimizer_updates=12200,known_total_optimizer_updates=18200,
        parent_npz_sha256=loader.PARENT["weights_sha256"],parent_metadata_sha256=loader.PARENT["metadata_sha256"],
        parent_checkpoint_sha256=loader.PARENT["checkpoint_sha256"],parent_declaration_sha256=loader.PARENT["declaration_sha256"],
        parent_history_sha256=loader.PARENT_HISTORY_SHA,model_config=parent.config.to_dict(),recovery_config=parent.recovery_config.to_dict(),
        trainable_state_tensor_count=54,frozen_state_tensor_count=47,trainable_parameter_count=2277827,
        trainable_state_names=names,frozen_state_names=sorted(set(reference)-set(names)),
        state_tensor_shapes={k:list(v.shape) for k,v in reference.items()},state_tensor_dtypes={k:"float32" for k in reference},
        learning_rate=loader.old.learning_rate_config(),batch_domains=loader.old.BATCH_DOMAINS,source_sha256=sources,
        frozen_state_sha256=loader.frozen_fingerprint(reference),training_dataset_fingerprint_sha256="f"*64,
        deferred_holdout_conversion_sha256="d90c022464e716ad6b86eec06395df892349869b981b91b91eecd056ae926594",
        authorization_sha256="a"*64,holdout_payload_loaded=False,runtime_loader_implemented=False,
        numerical_revision=dict(kernel_id=loader.old.KERNEL_ID,strict_deterministic_algorithms=True,
            tf32_matrix_multiplication=False,finite_precision_bitwise_equivalence_claimed=False))
    root_decl=dict(protocol=protocol,source_sha256=sources,authorization_sha256="a"*64)
    write(run/"declaration.json",root_decl)
    descriptions={};arrays={};receipts={};authority={}
    for arm in loader.ARMS:
        directory=run/arm/"exports/step_006000";directory.mkdir(parents=True)
        checkpoint=run/arm/"checkpoints/step_006000.pt";checkpoint.parent.mkdir();checkpoint.write_bytes(b"HASH-ONLY-NOT-A-PICKLE-"+arm.encode())
        write(run/arm/"declaration.json",dict(root_decl,arm=arm))
        descriptions[arm]=dict(base,arm=arm,checkpoint_sha256=digest(checkpoint),declaration_sha256=digest(run/arm/"declaration.json"))
        arrays[arm]={k:v.detach().numpy().copy() for k,v in reference.items()}
    summary=dict(schema=loader.TRAINING_SCHEMA,status="BOTH_FIXED6000_COMPLETE_NOT_PROMOTED",actual_batch_bytes_equal=True,
        all71_retained_per_arm=True,model_promoted=False,vehicle_control_approved=False,arms=receipts,
        endpoint_diagnostic_screens={arm:{"all_diagnostic_screens_passed":False} for arm in loader.ARMS})
    def seal():
        endpoints={}
        for arm in loader.ARMS:
            directory=run/arm/"exports/step_006000";d=descriptions[arm]
            with (directory/"model_state.npz").open("wb") as stream:np.savez(stream,**arrays[arm])
            d["npz_sha256"]=digest(directory/"model_state.npz");write(directory/"metadata.json",d)
            checkpoints=[dict(optimizer_updates=step,checkpoint_sha256="b"*64,npz_sha256="c"*64,metadata_sha256="d"*64) for step in range(500,6001,500)]
            checkpoints[-1].update(checkpoint_sha256=d["checkpoint_sha256"],npz_sha256=d["npz_sha256"],metadata_sha256=digest(directory/"metadata.json"))
            receipts.setdefault(arm,dict(arm=arm,optimizer_updates=6000,examples_seen=24000,completed_fixed_endpoint=True))
            receipts[arm]["checkpoints"]=checkpoints
            endpoints[arm]=dict(weights_sha256=d["npz_sha256"],metadata_sha256=digest(directory/"metadata.json"),
                checkpoint_sha256=d["checkpoint_sha256"],declaration_sha256=d["declaration_sha256"])
        write(run/"summary.json",summary)
        authority.update(schema=loader.AUTHORITY_SCHEMA,mode="RESEARCH_ONLY",pair_run=str(run),pair_declaration_sha256=digest(run/"declaration.json"),
            pair_summary_sha256=digest(run/"summary.json"),training_authorization_sha256="a"*64,parent=loader.PARENT,
            parent_history_sha256=loader.PARENT_HISTORY_SHA,source_sha256=sources,protocol_sha256=loader.json_sha(protocol),
            arm_order=loader.ARMS,both_fixed_endpoints_completed=True,model_promoted=False,vehicle_control_approved=False,endpoints=endpoints)
        write(tmp_path/"authority.json",authority)
    def invoke(arm="control",binding_change=None):
        directory=run/arm/"exports/step_006000";d=descriptions[arm];auth_sha=digest(tmp_path/"authority.json")
        binding=owner(d,digest(directory/"metadata.json"),auth_sha)
        if binding_change:binding.update(binding_change)
        return loader.load_intrinsic_curve_research_npz(directory/"model_state.npz",directory/"metadata.json",weights_sha256=d["npz_sha256"],
            metadata_sha256=digest(directory/"metadata.json"),**assets,
            completed_pair_authority=tmp_path/"authority.json",completed_pair_authority_sha256=auth_sha,research_owner_binding=binding,source_root=trainer.ROOT)
    seal()
    return SimpleNamespace(parent=parent,run=run,descriptions=descriptions,arrays=arrays,receipts=receipts,authority=authority,
        authority_path=tmp_path/"authority.json",summary=summary,seal=seal,invoke=invoke,assets=assets)


@pytest.fixture(scope="module")
def frozen_protocol():return trainer.contract()


def test_both_fixed_files_load_numeric_without_rewriting_denied_metadata(pair):
    before={str(p):p.read_bytes() for p in pair.run.rglob("*") if p.is_file()}
    for arm in loader.ARMS:
        model=pair.invoke(arm)
        assert all(torch.equal(model.state_dict()[k],v) for k,v in pair.parent.state_dict().items())
        assert not model.training and not any(p.requires_grad for p in model.parameters())
        assert model._research_recovery_numeric_loader_schema==loader.SCHEMA
        assert model._research_temporal_export_metadata["schema"]==loader.EXPORT_SCHEMA
        assert model._research_temporal_export_metadata["runtime_loader_implemented"] is False
        assert model._research_temporal_export_metadata["inference_loader_authorized"] is False
        assert model._research_recovery_simulator_control_authorized is False
    assert before=={str(p):p.read_bytes() for p in pair.run.rglob("*") if p.is_file()}


def test_speed_tensor_may_change_but_stop_is_not_claimed(pair):
    pair.arrays["intrinsic_curve"]["speed_head.p00"]+=.05;pair.seal()
    model=pair.invoke("intrinsic_curve")
    assert not torch.equal(model.state_dict()["speed_head.p00"],pair.parent.state_dict()["speed_head.p00"])
    assert model._research_temporal_export_metadata["stop_intent_trained"] is False


@pytest.mark.parametrize("arm",loader.ARMS)
@pytest.mark.parametrize("change",["frozen","nan","dtype","shape","missing"])
def test_both_endpoints_must_be_full101_finite_float32_and_frozen47(pair,arm,change):
    values=pair.arrays[arm]
    if change=="frozen":values["intent_head.p00"]+=.01
    elif change=="nan":values["speed_head.p00"][0]=np.nan
    elif change=="dtype":values["speed_head.p00"]=values["speed_head.p00"].astype(np.float64)
    elif change=="shape":values["speed_head.p00"]=np.zeros(2,dtype=np.float32)
    else:values.pop("speed_head.p00")
    pair.seal()
    with pytest.raises(ValueError):pair.invoke("control")


@pytest.mark.parametrize("key,value",[("optimizer_updates",5500),("optimizer_updates",6000.),("known_total_optimizer_updates",12200),
    ("parent_optimizer_updates",3000),("trainable_state_tensor_count",4),("frozen_state_tensor_count",97),
    ("model_promoted",True),("runtime_loader_implemented",True),("inference_loader_authorized",True),
    ("stop_intent_trained",True),("holdout_payload_loaded",True),("parent_history_sha256","0"*64),
    ("schema",loader.old.EXPORT_SCHEMA)])
def test_scope_falsification_in_either_final_is_rejected(pair,key,value):
    pair.descriptions["intrinsic_curve"][key]=value;pair.seal()
    with pytest.raises(ValueError):pair.invoke("control")


def test_false_completed_flag_cannot_replace_actual_second_final_files(pair):
    (pair.run/"intrinsic_curve/exports/step_006000/model_state.npz").unlink()
    with pytest.raises((ValueError,FileNotFoundError)):pair.invoke("control")


def test_insufficient_updates_of_counterpart_block_selected_control(pair):
    pair.receipts["intrinsic_curve"]["optimizer_updates"]=5999;pair.seal()
    with pytest.raises(ValueError,match="completion"):pair.invoke("control")


def test_matched_arm_order_cannot_select_observed_best(pair):
    pair.authority["arm_order"]=["intrinsic_curve","control"];write(pair.authority_path,pair.authority)
    with pytest.raises(ValueError):pair.invoke()


def test_wrong_external_authority_and_cross_arm_owner_rejected(pair):
    with pytest.raises(ValueError):pair.invoke(binding_change={"completed_pair_authority_sha256":"0"*64})
    with pytest.raises(ValueError):pair.invoke(binding_change={"arm":"intrinsic_curve"})


def test_numeric_owner_cannot_actuate_and_physical_requires_separate_plan(pair):
    with pytest.raises(ValueError):pair.invoke(binding_change={"simulator_control_authorized":True})
    with pytest.raises(ValueError):pair.invoke(binding_change={"purpose":"owned_carla_research_only","simulator_control_authorized":True,"simulation_only":True})
    model=pair.invoke(binding_change={"purpose":"owned_carla_research_only","simulator_control_authorized":True,"simulation_only":True,"owner_plan_sha256":"e"*64})
    assert model._research_recovery_simulator_control_authorized is True
    assert model._research_temporal_export_metadata["model_promoted"] is False


def test_source_missing_wrong_wrapper_or_gpu_revision_rejected(pair):
    d=copy.deepcopy(pair.descriptions["control"])
    d["source_sha256"].pop(next(iter(loader.ADDED_TRAINING_SOURCES)))
    with pytest.raises(ValueError):loader.verify_scope(d)
    d=copy.deepcopy(pair.descriptions["control"]);d["protocol"]["gpu_preflight_revision"]["maximum_driver_residual_MiB"]=17
    with pytest.raises(ValueError):loader.verify_scope(d)
    d=copy.deepcopy(pair.descriptions["control"]);d["protocol"]["gpu_preflight_revision"]["physical_index"]=False
    with pytest.raises(ValueError):loader.verify_scope(d)


def test_parent_source_closure_must_include_all26_unchanged(pair):
    pair.parent._research_temporal_export_metadata["source_sha256"].pop(next(iter(pair.parent._research_temporal_export_metadata["source_sha256"])))
    with pytest.raises(ValueError,match="26 parent"):pair.invoke()


def test_mutation_after_numeric_read_is_detected_before_return(pair,monkeypatch):
    original=loader.old.RouteRecoveryTemporalModelV3
    def constructor(*args,**kwargs):
        model=original(*args,**kwargs);(pair.run/"intrinsic_curve/checkpoints/step_006000.pt").write_bytes(b"changed-after-read")
        return model
    monkeypatch.setattr(loader.old,"RouteRecoveryTemporalModelV3",constructor)
    with pytest.raises(ValueError,match="SHA differs"):pair.invoke()


@pytest.mark.parametrize("role",["current_parent_weights","current_parent_metadata","warm_parent_weights","warm_parent_metadata",
    "backbone_parent_weights","backbone_parent_metadata"])
def test_each_ancestor_mutation_during_pair_load_is_detected(pair,monkeypatch,role):
    original=loader.old.RouteRecoveryTemporalModelV3
    def constructor(*args,**kwargs):
        model=original(*args,**kwargs);pair.assets[role].write_bytes(b"changed-mid-call-ancestor")
        return model
    monkeypatch.setattr(loader.old,"RouteRecoveryTemporalModelV3",constructor)
    with pytest.raises(ValueError,match="SHA differs"):pair.invoke()


def test_aliasing_ancestor_directory_rejected(pair,tmp_path):
    linked=tmp_path/"alias";linked.symlink_to(pair.run,target_is_directory=True)
    pair.authority["pair_run"]=str(linked);write(pair.authority_path,pair.authority)
    with pytest.raises(ValueError,match="non-symlink"):pair.invoke()


def test_current_parent_adapter_uses_exact_actual6000_plus_four_ancestor_files(monkeypatch):
    calls=[]
    monkeypatch.setattr(loader.old,"load_route_joint_research_npz",lambda *a,**k:calls.append((a,k)) or "numeric-parent")
    assert loader.load_current_parent("current.npz","current.json",warm_parent_weights="warm.npz",warm_parent_metadata="warm.json",
        backbone_parent_weights="backbone.npz",backbone_parent_metadata="backbone.json",source_root="root")=="numeric-parent"
    kwargs=calls[0][1];assert kwargs["weights_sha256"]==loader.PARENT["weights_sha256"]
    assert kwargs["parent_weights"]=="warm.npz" and kwargs["backbone_parent_weights"]=="backbone.npz"
    assert kwargs["research_owner_binding"]["fixed_endpoint_updates"]==6000
    assert kwargs["research_owner_binding"]["simulator_control_authorized"] is False
