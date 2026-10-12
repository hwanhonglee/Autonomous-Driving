"""HH_261002 - Numeric-only research loading for explicitly partially unfrozen V3 joint training.

This is not the frozen97/branch4 export protocol. Architecture and causal inputs
are unchanged, while54 post-input tensors may learn. STOP remains untrained.
"""
from __future__ import annotations

from io import BytesIO
import hashlib
import json
from pathlib import Path
import zipfile

import numpy as np
import torch

from .contract import ContractError
from .deterministic_linear_interpolation_v2 import KERNEL_ID
from .temporal_balanced_export_loader_research_20261001 import is_sha, pinned, unique_json_object
from .temporal_route_recovery_export_loader_research_20261001 import (
    OWNER_SCHEMA as PARENT_OWNER_SCHEMA, load_route_recovery_research_npz,
)
from .temporal_route_recovery_v3_20261001 import MODEL_ID, SCHEMA as ARCHITECTURE_SCHEMA, RouteRecoveryTemporalModelV3

ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "portable_e2e.temporal_route_joint_numeric_loader_research.20261002.v1"
EXPORT_SCHEMA = "portable_e2e.temporal_route_joint_export_research.20261002.v1"
TRAINING_SCHEMA = "portable_e2e.temporal_route_joint_training.20261002.v1"
OWNER_SCHEMA = "portable_e2e.temporal_route_joint_owner_binding.20261002.v1"
PARENT_CHECKPOINT_SHA = "a17544d99fd87467e4afe1c051dd598f72ab9cd0d79794f34d10291e1edce52f"
PARENT_NPZ_SHA = "0bdd747dffa6e37196e0ee6afb25fa9930d6aba6ca6f787c779ccfbeae3d4dc9"
PARENT_METADATA_SHA = "4ace621d8cf1b8d3ce269bb77aabe9e89598f3f750e18ea707a0817086b3dc2d"
PARENT_DECLARATION_SHA = "34c8a55028ea7b4cb2678a73701b16425d811fb10dd77afb5554183dd79ef6c5"
MAX_UPDATES = 6000
TRAINABLE_PREFIXES = ("fusion.", "output_norm.", "trunk.", "curvature_head.", "speed_head.", "route_recovery_branch.")
FAST_PREFIXES = ("curvature_head.", "speed_head.", "route_recovery_branch.")
BATCH_DOMAINS = ["recovery_noncentered_joint", "recovery_all_joint", "v4_joint", "legacy_raw_curvature_and_speed"]


def require(condition, message):
    if not condition: raise ContractError(message)


def trainable_names(state):
    return sorted(name for name in state if name.startswith(TRAINABLE_PREFIXES))


def frozen_fingerprint(state):
    # HH_261002 - Pin all47 unchanged inputs/semantic heads/buffer, not a misleading frozen-backbone assertion.
    result = hashlib.sha256()
    for name in sorted(set(state) - set(trainable_names(state))):
        value = state[name].detach().contiguous().cpu()
        require(value.dtype == torch.float32 and bool(torch.isfinite(value).all()), "finite float32 frozen state required")
        result.update(name.encode()); result.update(str(tuple(value.shape)).encode()); result.update(value.numpy().tobytes())
    return result.hexdigest()


def learning_rate_config():
    return dict(schedule="fixed6000_linear_warmup_cosine_two_groups", warmup_updates=300,
        shared_peak=1e-5, shared_final=1e-6, heads_peak=3e-5, heads_final=3e-6,
        gradient_clip_l2=5., weight_decay=1e-4, units="AdamW optimizer learning rate")


def load_joint_parent(weights, metadata, *, backbone_parent_weights, backbone_parent_metadata, source_root=ROOT):
    owner = dict(schema=PARENT_OWNER_SCHEMA, mode="RESEARCH_ONLY", owner_id="HH_261002-joint-training-parent-numeric-only",
        purpose="offline_cpu_numeric_validation_only", simulator_control_authorized=False,
        vehicle_control_approved=False, model_promoted=False, weights_sha256=PARENT_NPZ_SHA,
        metadata_sha256=PARENT_METADATA_SHA, checkpoint_sha256=PARENT_CHECKPOINT_SHA,
        declaration_sha256=PARENT_DECLARATION_SHA, fixed_endpoint_updates=3000)
    return load_route_recovery_research_npz(weights, metadata, weights_sha256=PARENT_NPZ_SHA,
        metadata_sha256=PARENT_METADATA_SHA, parent_weights=backbone_parent_weights,
        parent_metadata=backbone_parent_metadata, research_owner_binding=owner, source_root=source_root)


def verify_scope(description):
    require(isinstance(description, dict) and description.get("schema") == EXPORT_SCHEMA
        and description.get("training_schema") == TRAINING_SCHEMA
        and description.get("architecture_schema") == ARCHITECTURE_SCHEMA and description.get("model_id") == MODEL_ID,
        "explicit NEW partially unfrozen joint schema required; no frozen97 export masquerade")
    numbers = dict(optimizer_updates=6000, additional_optimizer_updates=6000, maximum_optimizer_updates=6000,
        effective_batch_size=4, examples_seen=24000, recovery_examples_seen=12000, v4_examples_seen=6000,
        legacy_examples_seen=6000, training_episode_count=18, recovery_training_episode_count=9,
        replay_training_episode_count=9, holdout_episode_count=2, parent_optimizer_updates=3000,
        parent_known_total_optimizer_updates=6200, known_total_optimizer_updates=12200,
        trainable_state_tensor_count=54, frozen_state_tensor_count=47, trainable_parameter_count=2277827)
    require(all(type(description.get(k)) is int and description[k] == v for k, v in numbers.items()),
        "fixed6000 actual18TRAIN/2HOLDOUT/54learning/47frozen lineage differs")
    flags = dict(training_only=True, simulation_only=True, model_promoted=False, vehicle_control_approved=False,
        inference_loader_authorized=False, stop_intent_trained=False, backbone_frozen=False,
        visual_encoder_frozen=True, semantic_head_parameters_frozen=True, semantic_outputs_preserved=False,
        legacy_decoded=False, independent_validation=False, checkpoint_selection_by_holdout=False,
        labels_clipped=False, labels_extrapolated=False, startup_recovery_fitted=False)
    require(all(description.get(k) is v for k, v in flags.items())
        and description.get("batch_domains") == BATCH_DOMAINS
        and description.get("runtime_intent") == "explicit_cruise_GO_not_head",
        "truthful partial-backbone learning/denied release/GO-only scope required")
    require(description.get("parent_checkpoint_sha256") == PARENT_CHECKPOINT_SHA
        and description.get("parent_npz_sha256") == PARENT_NPZ_SHA
        and description.get("parent_metadata_sha256") == PARENT_METADATA_SHA
        and description.get("parent_declaration_sha256") == PARENT_DECLARATION_SHA
        and description.get("learning_rate") == learning_rate_config(), "exact actual V3 warmstart and new optimizer schedule required")
    require(all(is_sha(description.get(k)) for k in ("checkpoint_sha256", "npz_sha256", "declaration_sha256",
        "frozen_state_sha256", "training_dataset_fingerprint_sha256", "holdout_dataset_fingerprint_sha256")),
        "all actual run/state/dataset identities required")
    numerical = description.get("numerical_revision", {})
    require(numerical == dict(kernel_id=KERNEL_ID, strict_deterministic_algorithms=True,
        tf32_matrix_multiplication=False, finite_precision_bitwise_equivalence_claimed=False), "strict unchanged decoder kernel required")
    sources = description.get("source_sha256", {})
    minimum = {"tools/e2e/train_temporal_route_joint_20261002.py",
        "learning/portable_e2e/temporal_route_joint_export_research_20261002.py",
        "tools/e2e/train_temporal_route_recovery_moving_v4_20261001.py",
        "learning/portable_e2e/temporal_route_recovery_v3_20261001.py"}
    require(isinstance(sources, dict) and minimum <= set(sources) and all(is_sha(v) for v in sources.values()),
        "complete imported NEW training/loader and unchanged causal model closure required")


def verify_owner(binding, description, weights_sha, metadata_sha):
    require(isinstance(binding, dict) and binding.get("schema") == OWNER_SCHEMA
        and binding.get("mode") == "RESEARCH_ONLY" and isinstance(binding.get("owner_id"), str) and binding["owner_id"].strip()
        and binding.get("vehicle_control_approved") is False and binding.get("model_promoted") is False,
        "explicit denied-release research owner required")
    if binding.get("purpose") == "offline_cpu_numeric_validation_only":
        require(binding.get("simulator_control_authorized") is False, "numeric validation cannot actuate")
    else:
        require(binding.get("purpose") == "owned_carla_research_only" and binding.get("simulator_control_authorized") is True
            and binding.get("simulation_only") is True and is_sha(binding.get("owner_plan_sha256")),
            "physical research needs a separate prospectively declared private owner")
    require(binding.get("weights_sha256") == weights_sha and binding.get("metadata_sha256") == metadata_sha
        and binding.get("checkpoint_sha256") == description["checkpoint_sha256"]
        and binding.get("declaration_sha256") == description["declaration_sha256"]
        and type(binding.get("fixed_endpoint_updates")) is int and binding["fixed_endpoint_updates"] == MAX_UPDATES,
        "owner must bind exact final6000; no intermediate checkpoint selection")


def load_route_joint_research_npz(weights, metadata, *, weights_sha256, metadata_sha256,
        parent_weights, parent_metadata, backbone_parent_weights, backbone_parent_metadata,
        research_owner_binding, source_root=ROOT):
    """HH_261002 - Float32 NPZ only: original47 exact,54 explicitly permitted to change; never torch pickle."""
    description = json.loads(pinned(metadata, metadata_sha256, 4 * 1024**2), object_pairs_hook=unique_json_object)
    verify_scope(description); verify_owner(research_owner_binding, description, weights_sha256, metadata_sha256)
    require(description["npz_sha256"] == weights_sha256, "exact joint export identity differs")
    root = Path(source_root).resolve(strict=True)
    for name, sha in description["source_sha256"].items():
        path = root / name
        require(not Path(name).is_absolute() and ".." not in Path(name).parts and path.resolve(strict=True).is_relative_to(root),
            "source path escaped canonical root")
        pinned(path, sha, 16 * 1024**2)
    parent = load_joint_parent(parent_weights, parent_metadata, backbone_parent_weights=backbone_parent_weights,
        backbone_parent_metadata=backbone_parent_metadata, source_root=root)
    require(all(description["source_sha256"].get(name) == sha
        for name, sha in parent._research_temporal_export_metadata["source_sha256"].items()),
        "all24 original actual parent training source pins must be inherited unchanged")
    model = RouteRecoveryTemporalModelV3(parent.config, parent.recovery_config).eval()
    expected, reference = model.state_dict(), parent.state_dict()
    require(len(expected) == len(reference) == 101 and len(trainable_names(expected)) == 54,
        "unchanged101 V3 tensor architecture required")
    require(description.get("model_config") == parent.config.to_dict()
        and description.get("recovery_config") == parent.recovery_config.to_dict()
        and description.get("trainable_state_names") == trainable_names(expected)
        and description.get("frozen_state_names") == sorted(set(expected) - set(trainable_names(expected))),
        "exact current101 architecture and54/47 partition required")
    shapes, dtypes = description.get("state_tensor_shapes", {}), description.get("state_tensor_dtypes", {})
    require(set(shapes) == set(dtypes) == set(expected), "complete shape/dtype ABI required")
    payload = pinned(weights, weights_sha256, 32 * 1024**2)
    with zipfile.ZipFile(BytesIO(payload)) as archive:
        entries = archive.infolist()
        require(len(entries) == 101 and len({e.filename for e in entries}) == 101
            and {e.filename for e in entries} == {k + ".npy" for k in expected}
            and all(not e.is_dir() and not (e.flag_bits & 1) for e in entries)
            and sum(e.file_size for e in entries) <= 32 * 1024**2, "bounded complete101 numeric archive required")
    state = {}
    with np.load(BytesIO(payload), allow_pickle=False) as arrays:
        require(set(arrays.files) == set(expected), "joint numeric keys differ")
        for name, target in expected.items():
            array = arrays[name]
            require(array.dtype == np.float32 and np.isfinite(array).all() and list(array.shape) == shapes[name] == list(target.shape)
                and isinstance(shapes[name], list) and all(type(v) is int for v in shapes[name]) and dtypes[name] == "float32",
                "strict finite float32 ABI differs: " + name)
            state[name] = torch.from_numpy(array.copy())
    frozen = set(expected) - set(trainable_names(expected))
    require(all(torch.equal(state[k], reference[k]) for k in frozen)
        and frozen_fingerprint(state) == frozen_fingerprint(reference) == description["frozen_state_sha256"],
        "one of47 frozen input/semantic tensors changed")
    model.load_state_dict(state, strict=True); model.requires_grad_(False); model.eval()
    model._research_temporal_checkpoint_sha256 = description["checkpoint_sha256"]
    model._research_temporal_export_npz_sha256 = weights_sha256
    model._research_temporal_metadata_sha256 = metadata_sha256
    model._research_temporal_export_metadata = description
    model._research_recovery_owner_binding = dict(research_owner_binding)
    model._research_recovery_numeric_loader_schema = SCHEMA
    model._research_recovery_simulator_control_authorized = research_owner_binding["simulator_control_authorized"]
    return model
