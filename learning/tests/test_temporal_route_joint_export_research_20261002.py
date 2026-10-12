"""HH_261002 - Reject frozen97 masquerades and preserve47 exact tensors in the NEW joint numeric protocol."""
import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from portable_e2e import temporal_route_joint_export_research_20261002 as loader
from portable_e2e.temporal_route_recovery_v3_20261001 import RouteRecoveryTemporalModelV3
from scripts.e2e import train_temporal_route_joint_20261002 as trainer


def description():
    model=RouteRecoveryTemporalModelV3(); state=model.state_dict(); names=loader.trainable_names(state)
    return dict(schema=loader.EXPORT_SCHEMA,training_schema=loader.TRAINING_SCHEMA,
        architecture_schema=loader.ARCHITECTURE_SCHEMA,model_id=loader.MODEL_ID,
        optimizer_updates=6000,additional_optimizer_updates=6000,maximum_optimizer_updates=6000,effective_batch_size=4,
        examples_seen=24000,recovery_examples_seen=12000,v4_examples_seen=6000,legacy_examples_seen=6000,
        parent_optimizer_updates=3000,parent_known_total_optimizer_updates=6200,known_total_optimizer_updates=12200,
        trainable_state_tensor_count=54,frozen_state_tensor_count=47,trainable_parameter_count=2277827,
        parent_checkpoint_sha256=loader.PARENT_CHECKPOINT_SHA,parent_npz_sha256=loader.PARENT_NPZ_SHA,
        parent_metadata_sha256=loader.PARENT_METADATA_SHA,parent_declaration_sha256=loader.PARENT_DECLARATION_SHA,
        model_config=model.config.to_dict(),recovery_config=model.recovery_config.to_dict(),
        trainable_state_names=names,frozen_state_names=sorted(set(state)-set(names)),learning_rate=loader.learning_rate_config(),
        batch_domains=loader.BATCH_DOMAINS,checkpoint_sha256="a"*64,npz_sha256="b"*64,declaration_sha256="c"*64,
        frozen_state_sha256="d"*64,training_dataset_fingerprint_sha256="e"*64,holdout_dataset_fingerprint_sha256="f"*64,
        source_sha256={name:"a"*64 for name in ("tools/e2e/train_temporal_route_joint_20261002.py",
            "learning/portable_e2e/temporal_route_joint_export_research_20261002.py",
            "tools/e2e/train_temporal_route_recovery_moving_v4_20261001.py",
            "learning/portable_e2e/temporal_route_recovery_v3_20261001.py")},
        numerical_revision=dict(kernel_id=loader.KERNEL_ID,strict_deterministic_algorithms=True,
            tf32_matrix_multiplication=False,finite_precision_bitwise_equivalence_claimed=False),**trainer.scope_fields())


def binding(d,metadata_sha="8"*64):
    return dict(schema=loader.OWNER_SCHEMA,mode="RESEARCH_ONLY",owner_id="unit-test-no-control",
        purpose="offline_cpu_numeric_validation_only",simulator_control_authorized=False,
        vehicle_control_approved=False,model_promoted=False,weights_sha256=d["npz_sha256"],metadata_sha256=metadata_sha,
        checkpoint_sha256=d["checkpoint_sha256"],declaration_sha256=d["declaration_sha256"],fixed_endpoint_updates=6000)


def test_exact_new_scope_and_real_cumulative12200_not_frozen97():
    d=description(); loader.verify_scope(d)
    assert d["backbone_frozen"] is False and d["semantic_outputs_preserved"] is False


@pytest.mark.parametrize("key,value",[("optimizer_updates",3000),("optimizer_updates",6000.),("training_episode_count",9),
    ("backbone_frozen",True),("visual_encoder_frozen",False),("semantic_outputs_preserved",True),
    ("stop_intent_trained",True),("model_promoted",True),("inference_loader_authorized",True),
    ("legacy_decoded",True),("trainable_state_tensor_count",4),("frozen_state_tensor_count",97),
    ("known_total_optimizer_updates",6200),("parent_checkpoint_sha256","0"*64),
    ("schema","portable_e2e.temporal_route_recovery_export_research.20261001.v3")])
def test_scope_falsification_rejected(key,value):
    d=description(); d[key]=value
    with pytest.raises(ValueError): loader.verify_scope(d)


def test_numeric_owner_cannot_actuate_and_research_owner_needs_prospective_plan():
    d=description(); owner=binding(d); loader.verify_owner(owner,d,d["npz_sha256"],"8"*64)
    owner["simulator_control_authorized"]=True
    with pytest.raises(ValueError): loader.verify_owner(owner,d,d["npz_sha256"],"8"*64)
    owner.update(purpose="owned_carla_research_only",simulation_only=True)
    with pytest.raises(ValueError): loader.verify_owner(owner,d,d["npz_sha256"],"8"*64)
    owner["owner_plan_sha256"]="7"*64; loader.verify_owner(owner,d,d["npz_sha256"],"8"*64)


def test_fixed6000_owner_rejects_intermediate_checkpoint():
    d=description(); owner=binding(d); owner["fixed_endpoint_updates"]=5500
    with pytest.raises(ValueError): loader.verify_owner(owner,d,d["npz_sha256"],"8"*64)


def test_source_and_schedule_required():
    d=description(); d["source_sha256"].pop("tools/e2e/train_temporal_route_joint_20261002.py")
    with pytest.raises(ValueError): loader.verify_scope(d)
    d=description(); d["learning_rate"]["schedule"]="fixed3000_branch_only"
    with pytest.raises(ValueError): loader.verify_scope(d)


def fixture_export(tmp_path,monkeypatch):
    parent=RouteRecoveryTemporalModelV3(); sources=trainer.source_snapshot()
    parent._research_temporal_export_metadata={"source_sha256":trainer.moving.source_snapshot()}
    monkeypatch.setattr(loader,"load_joint_parent",lambda *a,**k:parent)
    monkeypatch.setattr(trainer.replay,"storage_reserve",lambda p: {})
    model=RouteRecoveryTemporalModelV3(); model.load_state_dict(parent.state_dict()); trainer.freeze_inputs(model)
    optimizer=torch.optim.AdamW(trainer.optimizer_groups(model))
    receipt=trainer.save_checkpoint(tmp_path,model,optimizer,6000,sources,"e"*64,"f"*64,"c"*64,loader.frozen_fingerprint(parent.state_dict()))
    directory=tmp_path/"exports/step_006000"; npz=directory/"model_state.npz"; meta=directory/"metadata.json"
    description_=json.loads(meta.read_text()); return parent,npz,meta,description_


def invoke(npz,meta,d):
    meta_sha=hashlib.sha256(meta.read_bytes()).hexdigest()
    return loader.load_route_joint_research_npz(npz,meta,weights_sha256=d["npz_sha256"],metadata_sha256=meta_sha,
        parent_weights="mock-parent.npz",parent_metadata="mock-parent.json",backbone_parent_weights="mock-grandparent.npz",
        backbone_parent_metadata="mock-grandparent.json",research_owner_binding=binding(d,meta_sha),source_root=trainer.ROOT)


def test_all101_numeric_parent_parity_then_speed_weight_change_allowed(tmp_path,monkeypatch):
    parent,npz,meta,d=fixture_export(tmp_path,monkeypatch); model=invoke(npz,meta,d)
    assert all(torch.equal(model.state_dict()[k],v) for k,v in parent.state_dict().items())
    assert not model.training and not any(p.requires_grad for p in model.parameters())
    with np.load(npz,allow_pickle=False) as original: arrays={k:original[k].copy() for k in original.files}
    arrays["speed_head.bias"][0]+=.05
    with npz.open("wb") as stream: np.savez(stream,**arrays)
    d["npz_sha256"]=hashlib.sha256(npz.read_bytes()).hexdigest(); meta.write_text(json.dumps(d))
    changed=invoke(npz,meta,d)
    assert not torch.equal(changed.speed_head.bias,parent.speed_head.bias)
    assert torch.equal(changed.intent_head.bias,parent.intent_head.bias)


@pytest.mark.parametrize("change",["frozen","nan","dtype","shape","missing"])
def test_numeric_47_frozen_finite_float32_fullshape_guard(tmp_path,monkeypatch,change):
    parent,npz,meta,d=fixture_export(tmp_path,monkeypatch)
    with np.load(npz,allow_pickle=False) as original: arrays={k:original[k].copy() for k in original.files}
    if change=="frozen": arrays["intent_head.bias"][0]+=.01
    elif change=="nan": arrays["speed_head.bias"][0]=np.nan
    elif change=="dtype": arrays["speed_head.bias"]=arrays["speed_head.bias"].astype(np.float64)
    elif change=="shape": arrays["speed_head.bias"]=arrays["speed_head.bias"][:1]
    else: arrays.pop("speed_head.bias")
    with npz.open("wb") as stream: np.savez(stream,**arrays)
    d["npz_sha256"]=hashlib.sha256(npz.read_bytes()).hexdigest(); meta.write_text(json.dumps(d))
    with pytest.raises(ValueError): invoke(npz,meta,d)


def test_new_source_metadata_cannot_omit_an_original24_parent_pin(tmp_path,monkeypatch):
    parent,npz,meta,d=fixture_export(tmp_path,monkeypatch)
    d["source_sha256"].pop("learning/portable_e2e/path_time_decoder_v2.py"); meta.write_text(json.dumps(d))
    with pytest.raises(ValueError,match="all24"): invoke(npz,meta,d)
