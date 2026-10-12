"""HH_261002 - Load BOTH fixed paired endpoints numerically; separate declared research use from promotion.

The frozen export/training schema and false release flags are never rewritten.
The same101-tensor model has54 trainable/47 unchanged tensors; this loader is
not a frozen97 protocol, STOP training, checkpoint selector or vehicle release.
"""
from __future__ import annotations
from io import BytesIO
import hashlib
import json
from pathlib import Path
import zipfile
import numpy as np
import torch
from . import temporal_route_joint_export_research_20261002 as old

ROOT=Path(__file__).resolve().parents[2]
SCHEMA="portable_e2e.temporal_intrinsic_curve_numeric_loader_research.20261002.v1"
OWNER_SCHEMA="portable_e2e.temporal_intrinsic_curve_owner_binding.20261002.v1"
AUTHORITY_SCHEMA="portable_e2e.temporal_intrinsic_curve_completed_pair_authority.20261002.v1"
EXPORT_SCHEMA="portable_e2e.temporal_intrinsic_curve_continuation_export_research.20261002.v1"
TRAINING_SCHEMA="portable_e2e.temporal_intrinsic_curve_paired_continuation.20261002.v1"
ARMS=["control","intrinsic_curve"]
PARENT=dict(weights_sha256="b24a6ddd151595f8ef2a463dc8307819e9c52dd43aec3af216c76e4d646d8625",
    metadata_sha256="a4b2c35cbc782378a5464ba8daa69ff97cea3ac388567ba260d4f2934381be04",
    checkpoint_sha256="43b861c30ec9971ad5828af5f2c9f7917356ac007446433cd2cf6e627f7b4e88",
    declaration_sha256="0041d4d229d3a9ad777e2e0d4873793b65eeab608cbeb69b83fcda01b98cc3e5")
PARENT_HISTORY_SHA="3c1fe1e4f9e18017a2db31b5371001873dece4a2b8e78bb3cdb21f9aad5ebac3"
ADDED_TRAINING_SOURCES={
    "tools/e2e/train_temporal_intrinsic_curve_pair_20261002.py":"3da40c323c69b1d3e1a64b77f68955b4a8c169a895b5698db312f86d4b23c0c8",
    "learning/portable_e2e/intrinsic_spatial_path_loss_v1_20261002.py":"051a07c032c9335083b2e004711f58460e7e412aecb98e362c061e0a9a8ae314",
    "tools/e2e/train_temporal_intrinsic_curve_pair_v2_20261002.py":"d2a585dcc0485f8fe5cde07a66e53ddec4ae5ea7f3a0613b8f32a5c2ce46940a"}
GPU_REVISION=dict(schema="portable_e2e.gpu0_driver_residual_preflight.20261002.v2",physical_index=0,
    uuid="GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5",maximum_driver_residual_MiB=16,required_utilization_percent=0,
    required_GPU0_compute_process_count=0,physical_queries_before_torch_initialization=True,
    original_gpu0_lease_unchanged=True,training_math_or_labels_changed=False)
require,is_sha,unique_json_object=old.require,old.is_sha,old.unique_json_object
trainable_names,frozen_fingerprint=old.trainable_names,old.frozen_fingerprint


def canonical(path):
    path=Path(path).absolute()
    require(".." not in path.parts and not any(p.is_symlink() for p in (path,*path.parents)),"canonical non-symlink asset required")
    return path.resolve(strict=True)


def pinned(path,sha,maximum):
    return old.pinned(canonical(path),sha,maximum)


def json_sha(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(",",":"),allow_nan=False).encode()).hexdigest()


def read_json(path,sha):
    def invalid(value):raise ValueError("nonfinite JSON: "+value)
    value=json.loads(pinned(path,sha,4*1024**2),object_pairs_hook=unique_json_object,parse_constant=invalid)
    require(isinstance(value,dict),"JSON object required")
    return value


def verify_scope(d):
    require(isinstance(d,dict) and d.get("schema")==EXPORT_SCHEMA and d.get("training_schema")==TRAINING_SCHEMA
        and d.get("architecture_schema")==old.ARCHITECTURE_SCHEMA and d.get("model_id")==old.MODEL_ID
        and d.get("arm") in ARMS,"explicit NEW intrinsic continuation schema/arm required")
    numbers=dict(optimizer_updates=6000,additional_optimizer_updates=6000,maximum_optimizer_updates=6000,effective_batch_size=4,
        examples_seen=24000,recovery_examples_seen=12000,v4_examples_seen=6000,legacy_examples_seen=6000,
        parent_optimizer_updates=6000,parent_known_total_optimizer_updates=12200,known_total_optimizer_updates=18200,
        training_episode_count=18,recovery_training_episode_count=9,replay_training_episode_count=9,holdout_episode_count=2,
        trainable_state_tensor_count=54,frozen_state_tensor_count=47,trainable_parameter_count=2277827)
    require(all(type(d.get(k)) is int and d[k]==v for k,v in numbers.items()),"fixed final6000/parent12200/total18200/54-47 counts differ")
    flags=dict(training_only=True,simulation_only=True,main_training=False,model_promoted=False,vehicle_control_approved=False,
        inference_loader_authorized=False,runtime_loader_implemented=False,stop_intent_trained=False,backbone_frozen=False,
        visual_encoder_frozen=True,semantic_head_parameters_frozen=True,semantic_outputs_preserved=False,legacy_decoded=False,
        independent_validation=False,checkpoint_selection_by_holdout=False,labels_clipped=False,labels_extrapolated=False,
        startup_recovery_fitted=False,holdout_payload_loaded=False)
    require(all(d.get(k) is v for k,v in flags.items()) and d.get("runtime_intent")=="explicit_cruise_GO_not_head"
        and d.get("batch_domains")==old.BATCH_DOMAINS and d.get("learning_rate")==old.learning_rate_config(),
        "historical denied export/GO-only objective scope must remain unchanged")
    require(all(d.get("parent_"+k)==v for k,v in (("npz_sha256",PARENT["weights_sha256"]),
        ("metadata_sha256",PARENT["metadata_sha256"]),("checkpoint_sha256",PARENT["checkpoint_sha256"]),
        ("declaration_sha256",PARENT["declaration_sha256"]))) and d.get("parent_history_sha256")==PARENT_HISTORY_SHA,
        "exact actual joint6000 parent and completed update journal required")
    require(all(is_sha(d.get(k)) for k in ("npz_sha256","checkpoint_sha256","declaration_sha256","frozen_state_sha256",
        "training_dataset_fingerprint_sha256","authorization_sha256","deferred_holdout_conversion_sha256")),"actual export/data/authority identities required")
    require(d.get("deferred_holdout_conversion_sha256")=="d90c022464e716ad6b86eec06395df892349869b981b91b91eecd056ae926594"
        and d.get("numerical_revision")==dict(kernel_id=old.KERNEL_ID,strict_deterministic_algorithms=True,
            tf32_matrix_multiplication=False,finite_precision_bitwise_equivalence_claimed=False),"unchanged deferred HOLDOUT/deterministic kernel required")
    sources=d.get("source_sha256",{})
    require(isinstance(sources,dict) and len(sources)==29 and all(is_sha(v) for v in sources.values())
        and all(sources.get(k)==v for k,v in ADDED_TRAINING_SOURCES.items()),"exact NEW29 source closure required")
    p=d.get("protocol",{})
    require(isinstance(p,dict) and p.get("schema")==TRAINING_SCHEMA and p.get("sources")==sources
        and p.get("actual_parent_final6000")==PARENT and p.get("gpu_preflight_revision")==GPU_REVISION
        and p.get("arm_order")==ARMS and type(p.get("fixed_new_updates_per_arm")) is int and p["fixed_new_updates_per_arm"]==6000
        and type(p.get("coefficient")) is float and p["coefficient"]==.05
        and p.get("candidate_addition")==".25*.05*intrinsic_XY, ONLY actual legacy_curved domain anchors"
        and all(p.get(k) is v for k,v in dict(no_DEV_or_HOLDOUT_payload_until_both_final=True,no_checkpoint_selection=True,
            same_batch_tensor_bytes_required=True,temporal_retiming=False,future_model_inputs=False,labels_clipped=False).items()),
        "fixed paired objective, deferred evaluation and disclosed GPU-only revision required")
    gpu=p["gpu_preflight_revision"]
    require(all(type(gpu[k]) is int for k in ("physical_index","maximum_driver_residual_MiB","required_utilization_percent","required_GPU0_compute_process_count"))
        and all(type(gpu[k]) is bool for k in ("physical_queries_before_torch_initialization","original_gpu0_lease_unchanged","training_math_or_labels_changed")),
        "GPU revision requires exact Boolean/integer types")


def verify_owner(binding,d,weights_sha,metadata_sha,authority_sha):
    require(isinstance(binding,dict) and binding.get("schema")==OWNER_SCHEMA and binding.get("mode")=="RESEARCH_ONLY"
        and isinstance(binding.get("owner_id"),str) and binding["owner_id"].strip()
        and binding.get("model_promoted") is False and binding.get("vehicle_control_approved") is False
        and binding.get("arm")==d["arm"] and binding.get("completed_pair_authority_sha256")==authority_sha,
        "explicit arm and externally sealed completed-pair research binding required")
    if binding.get("purpose")=="offline_cpu_numeric_validation_only":
        require(binding.get("simulator_control_authorized") is False,"numeric validation cannot actuate")
    else:
        require(binding.get("purpose")=="owned_carla_research_only" and binding.get("simulator_control_authorized") is True
            and binding.get("simulation_only") is True and is_sha(binding.get("owner_plan_sha256")),"separate prospective private research owner required")
    require(binding.get("weights_sha256")==weights_sha and binding.get("metadata_sha256")==metadata_sha
        and binding.get("checkpoint_sha256")==d["checkpoint_sha256"] and binding.get("declaration_sha256")==d["declaration_sha256"]
        and type(binding.get("fixed_endpoint_updates")) is int and binding["fixed_endpoint_updates"]==6000,
        "owner must bind fixed final6000, not an intermediate or observed-best checkpoint")


def load_current_parent(weights,metadata,*,warm_parent_weights,warm_parent_metadata,backbone_parent_weights,backbone_parent_metadata,source_root):
    owner=dict(schema=old.OWNER_SCHEMA,mode="RESEARCH_ONLY",owner_id="HH_261002-intrinsic-parent-numeric-only",
        purpose="offline_cpu_numeric_validation_only",simulator_control_authorized=False,vehicle_control_approved=False,
        model_promoted=False,fixed_endpoint_updates=6000,**PARENT)
    return old.load_route_joint_research_npz(weights,metadata,weights_sha256=PARENT["weights_sha256"],metadata_sha256=PARENT["metadata_sha256"],
        parent_weights=warm_parent_weights,parent_metadata=warm_parent_metadata,backbone_parent_weights=backbone_parent_weights,
        backbone_parent_metadata=backbone_parent_metadata,research_owner_binding=owner,source_root=source_root)


def numeric_state(weights,d,reference):
    expected=reference;names=trainable_names(expected);shapes,dtypes=d.get("state_tensor_shapes",{}),d.get("state_tensor_dtypes",{})
    require(len(expected)==101 and len(names)==54 and set(shapes)==set(dtypes)==set(expected)
        and d.get("trainable_state_names")==names and d.get("frozen_state_names")==sorted(set(expected)-set(names)),"full exact101/54-47 ABI required")
    payload=pinned(weights,d["npz_sha256"],32*1024**2)
    with zipfile.ZipFile(BytesIO(payload)) as archive:
        entries=archive.infolist()
        require(len(entries)==101 and len({e.filename for e in entries})==101 and {e.filename for e in entries}=={k+".npy" for k in expected}
            and all(not e.is_dir() and not(e.flag_bits&1) for e in entries) and sum(e.file_size for e in entries)<=32*1024**2,"bounded unique101 numeric archive required")
    state={}
    with np.load(BytesIO(payload),allow_pickle=False) as arrays:
        require(set(arrays.files)==set(expected),"full numeric state keys differ")
        for name,target in expected.items():
            value=arrays[name]
            require(value.dtype==np.float32 and np.isfinite(value).all() and isinstance(shapes[name],list)
                and all(type(v) is int for v in shapes[name]) and list(value.shape)==shapes[name]==list(target.shape)
                and dtypes[name]=="float32" and target.dtype==torch.float32 and target.device.type=="cpu","finite float32 CPU shape ABI differs: "+name)
            state[name]=torch.from_numpy(value.copy())
    require(all(torch.equal(state[k],reference[k]) for k in set(expected)-set(names))
        and frozen_fingerprint(state)==frozen_fingerprint(reference)==d["frozen_state_sha256"],"one of47 tensors differs from actual current joint6000 parent")
    return state


def completed_pair(authority_path,authority_sha,selected,root,parent):
    authority=read_json(authority_path,authority_sha)
    require(authority.get("schema")==AUTHORITY_SCHEMA and authority.get("mode")=="RESEARCH_ONLY"
        and authority.get("both_fixed_endpoints_completed") is True and authority.get("model_promoted") is False
        and authority.get("vehicle_control_approved") is False and authority.get("arm_order")==ARMS
        and authority.get("parent")==PARENT and authority.get("parent_history_sha256")==PARENT_HISTORY_SHA,
        "root-sealed BOTH fixed endpoints/current parent research authority required")
    require(isinstance(authority.get("pair_run"),str) and Path(authority["pair_run"]).is_absolute(),"absolute canonical paired run required")
    run=canonical(authority["pair_run"]);summary=read_json(run/"summary.json",authority["pair_summary_sha256"])
    declaration=read_json(run/"declaration.json",authority["pair_declaration_sha256"])
    require(summary.get("schema")==TRAINING_SCHEMA and summary.get("status")=="BOTH_FIXED6000_COMPLETE_NOT_PROMOTED"
        and summary.get("actual_batch_bytes_equal") is True and summary.get("all71_retained_per_arm") is True
        and summary.get("model_promoted") is False and summary.get("vehicle_control_approved") is False
        and declaration.get("source_sha256")==authority.get("source_sha256")
        and declaration.get("authorization_sha256")==authority.get("training_authorization_sha256")
        and json_sha(declaration["protocol"])==authority.get("protocol_sha256"),"actual pair completion/source/protocol receipts differ")
    sources=authority["source_sha256"];reference=parent.state_dict();expected_sources=dict(parent._research_temporal_export_metadata["source_sha256"],**ADDED_TRAINING_SOURCES)
    require(len(expected_sources)==29 and sources==expected_sources and set(authority.get("endpoints",{}))==set(ARMS),"exact26 parent plus3 NEW training pins/BOTH endpoints required")
    for name,sha in sources.items():
        relative=Path(name);require(not relative.is_absolute() and ".." not in relative.parts and str(relative)==name,"canonical source key required")
        pinned(root/relative,sha,16*1024**2)
    result={};evidence={str(authority_path):(authority_sha,4*1024**2),str(run/"summary.json"):(authority["pair_summary_sha256"],4*1024**2),
        str(run/"declaration.json"):(authority["pair_declaration_sha256"],4*1024**2)}
    for arm in ARMS:
        pins=authority["endpoints"][arm];require(set(pins)=={"weights_sha256","metadata_sha256","checkpoint_sha256","declaration_sha256"}
            and all(is_sha(v) for v in pins.values()),"four actual external hashes per fixed arm required")
        export=run/arm/"exports/step_006000";d=read_json(export/"metadata.json",pins["metadata_sha256"]);verify_scope(d)
        receipt=summary["arms"][arm];checkpoints=receipt.get("checkpoints",[])
        require(type(receipt.get("optimizer_updates")) is int and receipt["optimizer_updates"]==6000
            and type(receipt.get("examples_seen")) is int and receipt["examples_seen"]==24000 and receipt.get("completed_fixed_endpoint") is True
            and [r["optimizer_updates"] for r in checkpoints]==list(range(500,6001,500)) and d["arm"]==arm
            and d["source_sha256"]==sources and json_sha(d["protocol"])==authority["protocol_sha256"]
            and d["authorization_sha256"]==authority["training_authorization_sha256"] and d["npz_sha256"]==pins["weights_sha256"]
            and d["checkpoint_sha256"]==pins["checkpoint_sha256"] and d["declaration_sha256"]==pins["declaration_sha256"]
            and all(checkpoints[-1][k]==pins[v] for k,v in (("npz_sha256","weights_sha256"),("metadata_sha256","metadata_sha256"),("checkpoint_sha256","checkpoint_sha256")))
            and d["model_config"]==parent.config.to_dict() and d["recovery_config"]==parent.recovery_config.to_dict(),"actual final6000 arm completion/config/lineage differs")
        pinned(run/arm/"checkpoints/step_006000.pt",pins["checkpoint_sha256"],128*1024**2)  # HH_261002 - Hash only; never deserialize Torch pickle.
        child=read_json(run/arm/"declaration.json",pins["declaration_sha256"])
        require(child.get("arm")==arm and child.get("protocol")==declaration["protocol"] and child.get("source_sha256")==sources,"arm declaration differs")
        state=numeric_state(export/"model_state.npz",d,reference)
        for path,sha,limit in ((export/"model_state.npz",pins["weights_sha256"],32*1024**2),
            (export/"metadata.json",pins["metadata_sha256"],4*1024**2),(run/arm/"checkpoints/step_006000.pt",pins["checkpoint_sha256"],128*1024**2),
            (run/arm/"declaration.json",pins["declaration_sha256"],4*1024**2)):evidence[str(path)]=(sha,limit)
        if arm==selected:result=dict(description=d,state=state,pins=pins,export=export)
    result["evidence"]=evidence
    return authority,result


def load_intrinsic_curve_research_npz(weights,metadata,*,weights_sha256,metadata_sha256,current_parent_weights,current_parent_metadata,
        warm_parent_weights,warm_parent_metadata,backbone_parent_weights,backbone_parent_metadata,completed_pair_authority,
        completed_pair_authority_sha256,research_owner_binding,source_root=ROOT):
    d=read_json(metadata,metadata_sha256);verify_scope(d)
    verify_owner(research_owner_binding,d,weights_sha256,metadata_sha256,completed_pair_authority_sha256)
    require(d["npz_sha256"]==weights_sha256,"selected exact endpoint differs")
    root=canonical(source_root);parent=load_current_parent(current_parent_weights,current_parent_metadata,warm_parent_weights=warm_parent_weights,
        warm_parent_metadata=warm_parent_metadata,backbone_parent_weights=backbone_parent_weights,backbone_parent_metadata=backbone_parent_metadata,source_root=root)
    pd=parent._research_temporal_export_metadata;warm=read_json(warm_parent_metadata,pd["parent_metadata_sha256"])
    ancestors=((current_parent_weights,pd["npz_sha256"],32*1024**2),(current_parent_metadata,parent._research_temporal_metadata_sha256,4*1024**2),
        (warm_parent_weights,pd["parent_npz_sha256"],32*1024**2),(warm_parent_metadata,pd["parent_metadata_sha256"],4*1024**2),
        (backbone_parent_weights,warm["parent_npz_sha256"],32*1024**2),(backbone_parent_metadata,warm["parent_metadata_sha256"],4*1024**2))
    authority,endpoint=completed_pair(completed_pair_authority,completed_pair_authority_sha256,d["arm"],root,parent)
    require(canonical(weights)==endpoint["export"]/"model_state.npz" and canonical(metadata)==endpoint["export"]/"metadata.json"
        and endpoint["description"]==d and endpoint["pins"]["weights_sha256"]==weights_sha256
        and endpoint["pins"]["metadata_sha256"]==metadata_sha256,"selected files must be the root-bound fixed paired endpoint")
    model=old.RouteRecoveryTemporalModelV3(parent.config,parent.recovery_config).eval();model.load_state_dict(endpoint["state"],strict=True)
    model.requires_grad_(False)
    model._research_temporal_checkpoint_sha256=d["checkpoint_sha256"];model._research_temporal_export_npz_sha256=weights_sha256
    model._research_temporal_metadata_sha256=metadata_sha256;model._research_temporal_export_metadata=d
    model._research_recovery_owner_binding=dict(research_owner_binding);model._research_recovery_numeric_loader_schema=SCHEMA
    model._research_recovery_simulator_control_authorized=research_owner_binding["simulator_control_authorized"]
    model._research_intrinsic_completed_pair_authority_sha256=completed_pair_authority_sha256
    model._research_intrinsic_matched_arm_order=tuple(ARMS)
    # HH_261002 - Source/control authority stays separate from immutable export false flags.
    for path,(sha,limit) in endpoint["evidence"].items():pinned(path,sha,limit)
    for path,sha,limit in ancestors:pinned(path,sha,limit)
    for name,sha in authority["source_sha256"].items():pinned(root/name,sha,16*1024**2)
    return model
