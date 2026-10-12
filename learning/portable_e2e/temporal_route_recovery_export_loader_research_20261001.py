"""HH_261001 - Load the NEW learned recovery export with immutable parent weights and explicit research ownership."""
from __future__ import annotations

from io import BytesIO
import hashlib
import json
from pathlib import Path
from typing import Mapping
import zipfile

import numpy as np
import torch

from .contract import ContractError
from .deterministic_linear_interpolation_v2 import KERNEL_ID
from .temporal_balanced_export_loader_research_20261001 import (
    TRAINING_SOURCE_PINS, OWNER_SCHEMA as PARENT_OWNER_SCHEMA, load_balanced_research_npz,
    is_sha, pinned, unique_json_object,
)
from .temporal_model_v2 import TemporalModelConfigV2
from .temporal_route_recovery_v3_20261001 import (
    BRANCH_PREFIX, MODEL_ID, SCHEMA as ARCHITECTURE_SCHEMA, EXPORT_SCHEMA,
    RouteRecoveryConfigV3, RouteRecoveryTemporalModelV3,
)

SCHEMA = "portable_e2e.temporal_route_recovery_numeric_loader_research.20261001.v1"
OWNER_SCHEMA = "portable_e2e.temporal_route_recovery_owner_binding.20261001.v1"
ROOT = Path(__file__).resolve().parents[2]
PARENT_CHECKPOINT_SHA = "c87c582fc4823d8ef3a00faf103c7aaa18481f2e1bfb7d38fea2f4d7136a99e9"
PARENT_NPZ_SHA = "4e1623e5156b2a19b546f1889d7a70e9c7b312f72fbf7a0e5a99ccda7d8e5735"
PARENT_METADATA_SHA = "88ffb8dcf9d21d0f756cad2c4ff478a74deee3b8e817f72a039a8885484bb4ec"
PARENT_DECLARATION_SHA = "b6165e816ca20ae5dd37446afbed7b0e674efd76a77d99eb0cb3114b99d02e93"
BRANCH_KEYS = frozenset(BRANCH_PREFIX + suffix for suffix in ("0.weight", "0.bias", "2.weight", "2.bias"))
TRAINER_FILE = "tools/e2e/train_temporal_route_recovery_v3_20261001.py"
MODEL_FILE = "learning/portable_e2e/temporal_route_recovery_v3_20261001.py"
LOADER_FILE = "learning/portable_e2e/temporal_route_recovery_export_loader_research_20261001.py"
MINIMUM_SOURCE_KEYS = frozenset(TRAINING_SOURCE_PINS) | {TRAINER_FILE, MODEL_FILE, LOADER_FILE,
    "learning/portable_e2e/temporal_cached_encoder_v2.py",
    "learning/portable_e2e/temporal_cpu_finite_validation_v2_20261001.py",
    "learning/portable_e2e/temporal_balanced_export_loader_research_20261001.py"}
BATCH_DOMAINS = ["recovery_noncentered_joint", "recovery_all_joint", "v4_joint", "legacy_raw_curvature"]


def require(condition, message):
    if not condition:
        raise ContractError(message)


def backbone_fingerprint(state):
    """HH_261001 - Fingerprint every unchanged original tensor, not just a promise that parameters were frozen."""
    digest = hashlib.sha256()
    for name in sorted(k for k in state if not k.startswith(BRANCH_PREFIX)):
        value = state[name].detach().contiguous().cpu()
        require(value.dtype == torch.float32 and bool(torch.isfinite(value).all()), "finite float32 parent backbone required")
        digest.update(name.encode()); digest.update(str(tuple(value.shape)).encode()); digest.update(value.numpy().tobytes())
    return digest.hexdigest()


def verify_scope(description):
    require(isinstance(description, dict) and description.get("schema") == EXPORT_SCHEMA
            and description.get("architecture_schema") == ARCHITECTURE_SCHEMA and description.get("model_id") == MODEL_ID,
            "explicit NEW recovery architecture/export schema required")
    integers = dict(optimizer_updates=3000, maximum_optimizer_updates=3000, additional_optimizer_updates=3000,
        effective_batch_size=4, examples_seen=12000, recovery_examples_seen=6000, v4_examples_seen=3000,
        legacy_examples_seen=3000, training_episode_count=18, recovery_training_episode_count=9,
        replay_training_episode_count=9, holdout_episode_count=2, parent_optimizer_updates=3000,
        parent_known_total_optimizer_updates=3200, known_total_optimizer_updates=6200)
    require(all(type(description.get(k)) is int and description[k] == value for k, value in integers.items()),
            "fixed final3000 actual18TRAIN/9recovery/9replay/2HOLDOUT lineage differs")
    flags = dict(training_only=True, simulation_only=True, main_training=False, model_promoted=False,
        vehicle_control_approved=False, inference_loader_authorized=False, stop_intent_trained=False,
        labels_clipped=False, labels_extrapolated=False, backbone_frozen=True, independent_validation=False,
        legacy_decoded=False, checkpoint_selection_by_holdout=False, parent_all_diagnostic_screens_passed=False)
    require(all(description.get(k) is value for k, value in flags.items())
            and description.get("runtime_intent") == "explicit_cruise_GO_not_head"
            and type(description.get("parent_diagnostic_pass_count")) is int and description["parent_diagnostic_pass_count"] == 7
            and description.get("trainable_state_names") == sorted(BRANCH_KEYS)
            and description.get("batch_domains") == BATCH_DOMAINS, "truthful new recovery scope/trainable branch differs")
    require(description.get("parent_checkpoint_sha256") == PARENT_CHECKPOINT_SHA
            and description.get("parent_npz_sha256") == PARENT_NPZ_SHA
            and description.get("parent_metadata_sha256") == PARENT_METADATA_SHA
            and all(is_sha(description.get(k)) for k in ("npz_sha256", "checkpoint_sha256", "declaration_sha256",
                "backbone_state_sha256", "training_dataset_fingerprint_sha256", "holdout_dataset_fingerprint_sha256")),
            "exact unpromoted mixed parent/new endpoint identities required")
    numerical = description.get("numerical_revision")
    require(isinstance(numerical, dict) and numerical.get("kernel_id") == KERNEL_ID
            and numerical.get("strict_deterministic_algorithms") is True
            and numerical.get("tf32_matrix_multiplication") is False
            and numerical.get("finite_precision_bitwise_equivalence_claimed") is False,
            "explicit strict deterministic unchanged decoder geometry numerical revision required")
    rate = description.get("learning_rate")
    require(isinstance(rate, dict) and set(rate) == {"peak", "final", "warmup_updates", "schedule", "units",
                "gradient_clip_l2", "weight_decay", "inherited_joint_schedule_reused"}
            and type(rate.get("peak")) is float and rate["peak"] == 3e-4
            and type(rate.get("final")) is float and rate["final"] == 3e-5
            and type(rate.get("warmup_updates")) is int and rate["warmup_updates"] == 100
            and type(rate.get("gradient_clip_l2")) is float and rate["gradient_clip_l2"] == 5.
            and type(rate.get("weight_decay")) is float and rate["weight_decay"] == 1e-4
            and rate.get("schedule") == "fixed3000_linear_warmup_cosine_branch_only"
            and rate.get("units") == "AdamW optimizer learning rate" and rate.get("inherited_joint_schedule_reused") is False,
            "exact prospectively declared fresh-branch optimizer schedule required")
    configuration = TemporalModelConfigV2.from_mapping(description["model_config"])
    require(configuration.to_dict() == TemporalModelConfigV2().to_dict(), "exact full parent model ABI required")
    recovery = RouteRecoveryConfigV3.from_mapping(description["recovery_config"])
    sources = description.get("source_sha256")
    require(isinstance(sources, dict) and MINIMUM_SOURCE_KEYS <= set(sources)
            and all(isinstance(k, str) and is_sha(v) for k, v in sources.items())
            and all(sources[k] == sha for k, sha in TRAINING_SOURCE_PINS.items()),
            "recursive imported source closure and all frozen17 parent source pins required")
    return configuration, recovery


def verify_owner(binding, description, weights_sha, metadata_sha):
    require(isinstance(binding, Mapping) and binding.get("schema") == OWNER_SCHEMA and binding.get("mode") == "RESEARCH_ONLY"
            and isinstance(binding.get("owner_id"), str) and binding["owner_id"].strip()
            and binding.get("vehicle_control_approved") is False and binding.get("model_promoted") is False,
            "explicit external research owner required; no vehicle approval")
    purpose = binding.get("purpose")
    if purpose == "offline_cpu_numeric_validation_only":
        require(binding.get("simulator_control_authorized") is False, "numeric-only owner cannot control a simulator")
    elif purpose == "owned_carla_research_only":
        require(binding.get("simulator_control_authorized") is True and binding.get("simulation_only") is True
                and is_sha(binding.get("owner_plan_sha256")), "owned CARLA research needs a separate prospectively pinned owner plan")
    else:
        raise ContractError("unsupported recovery owner purpose")
    require(binding.get("weights_sha256") == weights_sha and binding.get("metadata_sha256") == metadata_sha
            and binding.get("checkpoint_sha256") == description["checkpoint_sha256"]
            and binding.get("declaration_sha256") == description["declaration_sha256"]
            and type(binding.get("fixed_endpoint_updates")) is int and binding["fixed_endpoint_updates"] == 3000,
            "owner must bind exact final3000 export/declaration; no checkpoint selection")


def load_route_recovery_research_npz(weights, metadata, *, weights_sha256, metadata_sha256,
        parent_weights, parent_metadata, research_owner_binding, source_root=ROOT):
    """HH_261001 - Strict numeric CPU loading; external experimental ownership never edits denied export flags."""
    description = json.loads(pinned(metadata, metadata_sha256, 4 * 1024**2), object_pairs_hook=unique_json_object)
    config, recovery = verify_scope(description)
    require(description["npz_sha256"] == weights_sha256, "exact new recovery NPZ identity differs")
    verify_owner(research_owner_binding, description, weights_sha256, metadata_sha256)
    root = Path(source_root).resolve(strict=True)
    for name, sha in description["source_sha256"].items():
        path = root / name
        require(not Path(name).is_absolute() and ".." not in Path(name).parts
                and path.resolve(strict=True).is_relative_to(root), "source closure escaped canonical root")
        pinned(path, sha, 16 * 1024**2)
    parent_binding = dict(schema=PARENT_OWNER_SCHEMA, mode="RESEARCH_ONLY", owner_id=research_owner_binding["owner_id"],
        purpose="offline_cpu_numeric_validation_only", simulator_control_authorized=False,
        vehicle_control_approved=False, model_promoted=False, weights_sha256=PARENT_NPZ_SHA,
        metadata_sha256=PARENT_METADATA_SHA, checkpoint_sha256=PARENT_CHECKPOINT_SHA,
        declaration_sha256=PARENT_DECLARATION_SHA, fixed_endpoint_updates=3000)
    parent = load_balanced_research_npz(parent_weights, parent_metadata, weights_sha256=PARENT_NPZ_SHA,
        metadata_sha256=PARENT_METADATA_SHA, research_owner_binding=parent_binding, source_root=root)
    parent_state = parent.state_dict()
    require(len(parent_state) == 97 and backbone_fingerprint(parent_state) == description["backbone_state_sha256"],
            "actual97 original parent tensors differ from frozen backbone declaration")
    model = RouteRecoveryTemporalModelV3(config, recovery).eval()
    expected = model.state_dict(); shapes, dtypes = description.get("state_tensor_shapes"), description.get("state_tensor_dtypes")
    require(isinstance(shapes, dict) and isinstance(dtypes, dict) and set(shapes) == set(dtypes) == set(expected)
            and set(expected) - set(parent_state) == BRANCH_KEYS, "new101 tensor/key ABI differs")
    payload = pinned(weights, weights_sha256, 32 * 1024**2)
    with zipfile.ZipFile(BytesIO(payload)) as archive:
        entries = archive.infolist()
        require(len(entries) == len(expected) and len({e.filename for e in entries}) == len(entries)
                and {e.filename for e in entries} == {k + ".npy" for k in expected}
                and all(not e.is_dir() and not (e.flag_bits & 1) for e in entries)
                and sum(e.file_size for e in entries) <= 32 * 1024**2, "bounded complete numeric recovery archive required")
    state = {}
    with np.load(BytesIO(payload), allow_pickle=False) as arrays:
        require(set(arrays.files) == set(expected), "full numeric recovery tensor keys differ")
        for name, target in expected.items():
            array = arrays[name]
            require(array.dtype == np.float32 and np.isfinite(array).all()
                    and isinstance(shapes[name], list) and all(type(size) is int for size in shapes[name])
                    and list(array.shape) == shapes[name] == list(target.shape) and dtypes[name] == "float32",
                    "finite float32 full recovery tensor ABI differs: " + name)
            state[name] = torch.from_numpy(array.copy())
    require(all(torch.equal(state[name], value) for name, value in parent_state.items())
            and backbone_fingerprint(state) == description["backbone_state_sha256"],
            "one of the97 frozen original tensors changed during branch training")
    model.load_state_dict(state, strict=True); model.requires_grad_(False); model.eval()
    model._research_temporal_checkpoint_sha256 = description["checkpoint_sha256"]
    model._research_temporal_export_npz_sha256 = weights_sha256
    model._research_temporal_metadata_sha256 = metadata_sha256
    model._research_temporal_export_metadata = description
    model._research_recovery_owner_binding = dict(research_owner_binding)
    model._research_recovery_numeric_loader_schema = SCHEMA
    model._research_recovery_simulator_control_authorized = research_owner_binding["simulator_control_authorized"]
    return model
