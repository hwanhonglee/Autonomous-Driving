"""HH_261001 - Load the mixed nine-episode endpoint without falsifying single-episode authorization.

Numeric CPU validation only: a root-owned, exact-hash RESEARCH_ONLY binding is
mandatory. This module does not authorize simulator control, promote a model,
load Torch pickle, alter original metadata, or change the frozen live loader.
"""
from __future__ import annotations

from io import BytesIO
import hashlib
import json
from pathlib import Path
from typing import Mapping
import zipfile

import numpy as np
import torch

from .contract import ContractError, _read_regular_file_bounded
from .temporal_cached_encoder_v2 import CachedTemporalModelV2
from .temporal_model_v2 import TemporalModelConfigV2


SCHEMA = "portable_e2e.temporal_v2_balanced_numeric_loader_research.20261001.v1"
EXPORT_SCHEMA = "portable_e2e.temporal_v2_balanced_replay_export.20261001.v1"
OWNER_SCHEMA = "portable_e2e.temporal_v2_balanced_numeric_owner_binding.20261001.v1"
ROOT = Path(__file__).resolve().parents[2]
MAX_WEIGHTS_BYTES, MAX_METADATA_BYTES = 32 * 1024**2, 4 * 1024**2
PARENT_SHA256 = "6c8bfceee5b3a2a0e332666b225769af236499baac9414294c18ccc6e9833ece"
TRAINER_SHA256 = "a1e3ad4ed9302b02701c7e3aa56de88afacf6681b275c1bc6986f79ebfd79103"
TRAINING_SOURCE_PINS = {
    "learning/portable_e2e/__init__.py": "c5c53d6d0650bd67d4aeb3a828d55fae978987b61bbe6842234739b07431e8ef",
    "learning/portable_e2e/config/common_10hz_v1.contract.json": "6f1a7a82abe39a0cc16b42df2b191d2cdae735243a22e8f8cb47bf11faaa4642",
    "learning/portable_e2e/contract.py": "e2e3a2669fa94aa98cb7e50f20ffa7db433b99eead9a12c654392e4b293a5c09",
    "learning/portable_e2e/dataset.py": "3b30a7b4db1907a10ad94002bb880deaa4bb545a978bad3beec1b0aacf3b9c95",
    "learning/portable_e2e/deterministic_linear_interpolation_v2.py": "778cda87276ee9330ce1a41c6a60f7f8793f4b61cf0c779eefc84c6e4cd006e6",
    "learning/portable_e2e/model.py": "fbbedef10b97add830e0d887400107b2bcd6dfb5f0c5c4901e399d2e4717fd6e",
    "learning/portable_e2e/path_time_decoder_v2.py": "d80e370d52c73591df8a593a0483e225cb4cf2595e8389a91400f68f94eea7e7",
    "learning/portable_e2e/stop_primitive_research.py": "4021eb2ba6d84260b6b658114fbe31a1e7ef826121e4d07690bfd4154919a92c",
    "learning/portable_e2e/temporal_dataset_v2.py": "9f27fd12f7302ed654df4c2c956697d794bb15243c9576681e0300a226cefafe",
    "learning/portable_e2e/temporal_losses_v2.py": "53d4d61cea15c136ec2c65ffbabc849d16499596f995d9ae3bf55ba175074fe7",
    "learning/portable_e2e/temporal_model_v2.py": "93f709c6c229e705899fcf1778f9dd253ea867e88a6f237ad3f0fb8efdbbf435",
    "learning/portable_e2e/torch_dataset.py": "a81d26b1c9aaf67db56cf993a41857133dccac78e42047754e106cff4c8987c9",
    "tools/canonical_workspace.py": "5d5ee618dba8dfdd84cc6da2877fb2dce8b5ba0017e4a503eb3c55d28401d0ed",
    "tools/e2e/train_temporal_v2_balanced_replay_20261001.py": TRAINER_SHA256,
    "tools/e2e/train_temporal_v2_low_speed_joint_20261001.py": "f047739b639586c5998d8d117bd6d0330940ee28c9c5eadad34ce0a57340178d",
    "tools/e2e/train_temporal_v2_raw_bootstrap_20261001.py": "85b24d472112364e8c3a468bbe7849c10f4a802a3f7c2ea391c1049ddda5648e",
    "tools/workspace_python.py": "2221cdf71e4ec840566fb19c180329887e53035ef1f319b02221c547141a5b21",
}
NUMERICAL_SOURCE_KEYS = frozenset((
    "tools/e2e/train_temporal_v2_raw_bootstrap_20261001.py",
    "tools/e2e/train_temporal_v2_low_speed_joint_20261001.py",
    "learning/portable_e2e/temporal_model_v2.py", "learning/portable_e2e/temporal_losses_v2.py",
    "learning/portable_e2e/path_time_decoder_v2.py", "learning/portable_e2e/deterministic_linear_interpolation_v2.py",
))
BATCH_DOMAINS = ["v4_joint", "v4_joint", "legacy_curved_raw", "legacy_straight_raw"]


def require(condition, message):
    if not condition:
        raise ContractError(message)


def is_sha(value):
    return isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def pinned(path, sha, maximum):
    require(is_sha(sha), "exact caller SHA256 required")
    raw = _read_regular_file_bounded(Path(path), maximum, "balanced numeric research input")
    require(hashlib.sha256(raw).hexdigest() == sha, "balanced numeric input SHA differs")
    return raw


def unique_json_object(pairs):
    # HH_261001 - Reject duplicate metadata keys instead of silently accepting their final overwritten value.
    result = {}
    for key, value in pairs:
        require(key not in result, "duplicate balanced metadata JSON key")
        result[key] = value
    return result


def verify_export_scope(description, weights_sha):
    require(isinstance(description, dict), "balanced metadata object required")
    require(description.get("schema") == EXPORT_SCHEMA
            and description.get("npz_sha256") == weights_sha
            and is_sha(description.get("checkpoint_sha256"))
            and is_sha(description.get("declaration_sha256"))
            and is_sha(description.get("training_dataset_fingerprint_sha256")), "explicit mixed export/hash identity differs")
    integer_fields = dict(optimizer_updates=3000, examples_seen=12000, v4_examples_seen=6000,
        legacy_examples_seen=6000, parent_source_optimizer_updates=200, known_total_optimizer_updates=3200,
        maximum_optimizer_updates=3000, training_episode_count=9)
    require(all(type(description.get(k)) is int and description[k] == v for k, v in integer_fields.items())
            and description.get("parent_checkpoint_sha256") == PARENT_SHA256
            and description.get("batch_domains") == BATCH_DOMAINS, "fixed final3000 mixed research lineage/exposure differs")
    flags = dict(training_only=True, simulation_only=True, one_episode_screening=False,
        research_screening=True, mixed_raw_joint_training=True, main_training=False, model_promoted=False,
        vehicle_control_approved=False, independent_validation=False, stop_intent_trained=False,
        raw_spatial_only=False, decoder_integrated=True, legacy_decoded=False, labels_clipped=False,
        labels_extrapolated=False, inference_loader_authorized=False, single_episode_export_compatibility_claimed=False)
    require(all(description.get(k) is v for k, v in flags.items())
            and description.get("runtime_intent") == "explicit_cruise_GO_not_head", "truthful mixed research flags differ")
    sources = description.get("source_sha256")
    require(isinstance(sources, dict) and sources == TRAINING_SOURCE_PINS, "exact frozen17 training source closure differs")
    revision = description.get("numerical_revision")
    require(isinstance(revision, dict)
            and revision.get("kernel_id") == "fixed33_to257_align_corners_dense_gemm.20261001.v1"
            and revision.get("strict_deterministic_algorithms") is True
            and revision.get("tf32_matrix_multiplication") is False
            and revision.get("finite_precision_bitwise_equivalence_claimed") is False
            and revision.get("frozen_decoder_sha256") == TRAINING_SOURCE_PINS["learning/portable_e2e/path_time_decoder_v2.py"]
            and revision.get("source_sha256") == {k: TRAINING_SOURCE_PINS[k] for k in NUMERICAL_SOURCE_KEYS},
            "mixed export deterministic numerical revision differs")
    configuration = TemporalModelConfigV2.from_mapping(description["model_config"])
    require(configuration.to_dict() == TemporalModelConfigV2().to_dict(), "exact raw200/mixed model configuration required")
    return configuration


def verify_owner_binding(binding, description, weights_sha, metadata_sha):
    """HH_261001 - This binding permits numeric validation, never a simulator or real-vehicle control trial."""
    require(isinstance(binding, Mapping) and binding.get("schema") == OWNER_SCHEMA
            and binding.get("mode") == "RESEARCH_ONLY"
            and isinstance(binding.get("owner_id"), str) and bool(binding["owner_id"].strip())
            and binding.get("purpose") == "offline_cpu_numeric_validation_only"
            and binding.get("simulator_control_authorized") is False
            and binding.get("vehicle_control_approved") is False and binding.get("model_promoted") is False,
            "explicit RESEARCH_ONLY numeric owner binding required; no control authority")
    require(binding.get("weights_sha256") == weights_sha and binding.get("metadata_sha256") == metadata_sha
            and binding.get("checkpoint_sha256") == description["checkpoint_sha256"]
            and binding.get("declaration_sha256") == description["declaration_sha256"]
            and binding.get("fixed_endpoint_updates") == 3000
            and type(binding.get("fixed_endpoint_updates")) is int,
            "owner binding must pin exact endpoint weights/metadata/checkpoint/declaration")


def load_balanced_research_npz(weights, metadata, *, weights_sha256, metadata_sha256,
                               research_owner_binding, source_root=ROOT):
    """HH_261001 - Strict float32 CPU ABI; source metadata stays false-authorized and no Torch pickle is loaded."""
    metadata_bytes = pinned(metadata, metadata_sha256, MAX_METADATA_BYTES)
    description = json.loads(metadata_bytes, object_pairs_hook=unique_json_object)
    configuration = verify_export_scope(description, weights_sha256)
    verify_owner_binding(research_owner_binding, description, weights_sha256, metadata_sha256)
    root = Path(source_root).resolve(strict=True)
    for name, sha in TRAINING_SOURCE_PINS.items():
        path = root / name
        require(path.resolve(strict=True).is_relative_to(root), "exported training source escapes canonical root")
        pinned(path, sha, 16 * 1024**2)
    model = CachedTemporalModelV2(configuration)
    expected = model.state_dict()
    shapes, dtypes = description.get("state_tensor_shapes"), description.get("state_tensor_dtypes")
    require(isinstance(shapes, dict) and isinstance(dtypes, dict)
            and set(shapes) == set(dtypes) == set(expected), "full mixed state key ABI differs")
    require(all(isinstance(value, list) and all(type(size) is int and size >= 0 for size in value)
                for value in shapes.values()), "integer-only full state shapes required")
    payload = pinned(weights, weights_sha256, MAX_WEIGHTS_BYTES)
    with zipfile.ZipFile(BytesIO(payload)) as archive:
        entries = archive.infolist()
        require(len(entries) == len(expected) and len({x.filename for x in entries}) == len(entries)
                and {x.filename for x in entries} == {k + ".npy" for k in expected}
                and all(not x.is_dir() and not (x.flag_bits & 1) for x in entries)
                and sum(x.file_size for x in entries) <= MAX_WEIGHTS_BYTES,
                "bounded numeric NPZ members differ or duplicate/encrypted/oversized entries present")
    state = {}
    with np.load(BytesIO(payload), allow_pickle=False) as arrays:
        require(set(arrays.files) == set(expected), "full numeric mixed tensor keys differ")
        for name, target in expected.items():
            array = arrays[name]
            require(array.dtype == np.float32 and np.isfinite(array).all()
                    and list(array.shape) == shapes[name] == list(target.shape)
                    and dtypes[name] == "float32" and target.dtype == torch.float32
                    and target.device.type == "cpu", "finite float32 CPU tensor shape/dtype ABI differs: " + name)
            state[name] = torch.from_numpy(array.copy())
    model.load_state_dict(state, strict=True)
    model.requires_grad_(False); model.eval()
    model._research_temporal_checkpoint_sha256 = description["checkpoint_sha256"]
    model._research_temporal_export_npz_sha256 = weights_sha256
    model._research_temporal_metadata_sha256 = metadata_sha256
    model._research_temporal_export_metadata = description
    model._research_balanced_numeric_owner_binding = dict(research_owner_binding)
    model._research_balanced_numeric_loader_schema = SCHEMA
    model._research_balanced_simulator_control_authorized = False
    return model
