# HH_260906 - Export and load a fail-closed NumPy runtime weight bundle without pickle.
"""Versioned, non-executable model weights for legacy PyTorch runtimes."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from io import BytesIO
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import secrets
import stat
import sys
from types import MappingProxyType
from typing import Any, Mapping, Sequence
import zipfile

import numpy as np

from .contract import ContractError, _loads_json


BUNDLE_ID = "portable_e2e.numpy_runtime_weight_bundle.v1"
BUNDLE_SCHEMA_VERSION = 1
SOURCE_CHECKPOINT_ID = "portable_e2e.pytorch_checkpoint.v1"
SUPPORTED_MODEL_IDS = frozenset(
    (
        "portable_e2e.perspective_trajectory.v0",
        "portable_e2e.perspective_trajectory.physical.v1",
    )
)
MANIFEST_MEMBER = "manifest.json"
MAX_BUNDLE_FILE_BYTES = 512 * 1024 * 1024
MAX_MANIFEST_BYTES = 4 * 1024 * 1024
MAX_TENSOR_COUNT = 4096
MAX_TENSOR_BYTES = 256 * 1024 * 1024
MAX_TOTAL_TENSOR_BYTES = 512 * 1024 * 1024
MAX_NPY_OVERHEAD_BYTES = 64 * 1024
READ_CHUNK_BYTES = 8 * 1024 * 1024

_SHA256_PATTERN = re.compile(r"[0-9a-f]{64}\Z")
_STATE_KEY_PATTERN = re.compile(r"[A-Za-z0-9_]+(?:\.[A-Za-z0-9_]+)*\Z")
_ALLOWED_DTYPES = frozenset(
    {
        "|b1",
        "|i1",
        "|u1",
        "<i2",
        "<i4",
        "<i8",
        "<u2",
        "<u4",
        "<u8",
        "<f2",
        "<f4",
        "<f8",
        "<c8",
        "<c16",
    }
)
_MODEL_CONFIG_FIELDS = frozenset(
    {
        "model_id",
        "camera_count",
        "calibration_features",
        "ego_features",
        "route_points",
        "future_points",
        "candidate_count",
        "image_width",
        "image_height",
        "image_channels",
        "image_grid_width",
        "image_grid_height",
        "image_embedding",
        "camera_fusion_width",
        "ego_embedding",
        "route_embedding",
        "hidden_width",
        "encoder_base_channels",
        "ego_history_frames",
        "maximum_step_m",
        "route_scale_m",
    }
)
_TRAIN_CONFIG_FIELDS = frozenset(
    {
        "seed",
        "batch_size",
        "learning_rate",
        "weight_decay",
        "max_steps",
        "checkpoint_interval",
        "num_workers",
        "maximum_gradient_norm",
        "verify_image_sha256",
        "sampling_policy",
        "domain_ratios",
    }
)
_LOSS_CONFIG_FIELDS = frozenset(
    {
        "xy_weight",
        "speed_weight",
        "yaw_weight",
        "kinematic_speed_weight",
        "final_displacement_weight",
        "candidate_score_weight",
    }
)
_STATE_FIELDS = frozenset(
    {
        "epoch",
        "next_batch_index",
        "global_step",
        "samples_seen",
        "domain_samples_seen",
    }
)
_RUNTIME_ABI_FIELDS = frozenset(
    {
        "python",
        "torch",
        "numpy",
        "pillow",
        "torch_cuda",
        "cudnn",
        "deterministic_algorithms",
        "cudnn_benchmark",
        "cudnn_deterministic",
        "cublas_workspace_config",
        "torch_num_threads",
        "torch_num_interop_threads",
        "omp_num_threads",
        "mkl_num_threads",
        "cuda_matmul_allow_tf32",
        "cudnn_allow_tf32",
    }
)
_PROVENANCE_FIELDS = frozenset(
    {
        "created_at_utc",
        "dataset_fingerprint_sha256",
        "training_split",
        "training_episode_ids",
        "sampling_plan",
        "sampling_plan_sha256",
        "sampling_metrics",
        "train_config",
        "loss_config",
        "state",
        "runtime_abi",
        "device_abi",
    }
)
_TENSOR_RECORD_FIELDS = frozenset(
    {
        "index",
        "key",
        "member",
        "dtype",
        "shape",
        "nbytes",
        "npy_nbytes",
        "tensor_sha256",
        "npy_sha256",
    }
)
_MANIFEST_FIELDS = frozenset(
    {
        "bundle_id",
        "schema_version",
        "serialization",
        "source_checkpoint",
        "model_config",
        "model_config_sha256",
        "corpus_fingerprint_sha256",
        "training_provenance",
        "training_provenance_sha256",
        "tensor_count",
        "total_tensor_bytes",
        "total_npy_bytes",
        "model_state_sha256",
        "tensors",
    }
)
_SERIALIZATION_CONTRACT = {
    "archive": "zip",
    "compression": "stored",
    "tensor_format": "npy",
    "allow_pickle": False,
    "byte_order": "little_endian",
}


@dataclass(frozen=True)
class ExportedRuntimeWeightBundle:
    """Identity returned after an atomic runtime bundle export."""

    path: Path
    bundle_sha256: str
    source_checkpoint_sha256: str
    model_config_sha256: str
    corpus_fingerprint_sha256: str
    tensor_count: int
    total_tensor_bytes: int


@dataclass(frozen=True)
class RuntimeWeightBundle:
    """Verified metadata and immutable NumPy views from one runtime bundle."""

    path: Path
    bundle_sha256: str
    source_checkpoint_sha256: str
    model_config_sha256: str
    corpus_fingerprint_sha256: str
    model_config: Mapping[str, Any]
    training_provenance: Mapping[str, Any]
    numpy_state_dict: Mapping[str, np.ndarray]

    def to_torch_state_dict(self, torch_module: Any | None = None) -> dict[str, Any]:
        """Create an ordinary state dict through the legacy-safe from_numpy API."""
        if torch_module is None:
            try:
                import torch as torch_module
            except ImportError as error:
                raise ContractError(
                    "PyTorch is required only when reconstructing the state dict"
                ) from error
        from_numpy = getattr(torch_module, "from_numpy", None)
        if not callable(from_numpy):
            raise ContractError("torch_module must provide a callable from_numpy")
        state_dict: dict[str, Any] = {}
        for key, array in self.numpy_state_dict.items():
            try:
                tensor = from_numpy(np.array(array, copy=True, order="C"))
            except (RuntimeError, TypeError, ValueError) as error:
                raise ContractError(
                    f"cannot reconstruct runtime tensor {key!r}: {error}"
                ) from error
            state_dict[key] = tensor
        return state_dict


# HH_260906 - Enforce the model's exact tensor dtype and shape before PyTorch can coerce values.
def _validate_torch_state_dict_abi(
    actual_state: Any,
    expected_state: Mapping[str, Any],
    torch_module: Any,
    *,
    context: str,
) -> None:
    if not isinstance(actual_state, Mapping) or not actual_state:
        raise ContractError(f"{context} must be a nonempty mapping")
    tensor_type = getattr(torch_module, "Tensor", None)
    if not isinstance(tensor_type, type):
        raise ContractError("torch_module must expose the Tensor type")
    actual_keys = tuple(
        _validate_state_key(key, f"{context} key {index}")
        for index, key in enumerate(actual_state)
    )
    expected_keys = tuple(expected_state)
    if set(actual_keys) != set(expected_keys):
        missing = sorted(set(expected_keys) - set(actual_keys))
        unknown = sorted(set(actual_keys) - set(expected_keys))
        raise ContractError(
            f"{context} keys differ from the model ABI; "
            f"missing={missing}, unknown={unknown}"
        )
    for key in expected_keys:
        actual = actual_state[key]
        expected = expected_state[key]
        if not isinstance(actual, tensor_type):
            raise ContractError(f"{context} tensor {key!r} is not a torch.Tensor")
        if tuple(actual.shape) != tuple(expected.shape):
            raise ContractError(
                f"{context} tensor {key!r} shape {tuple(actual.shape)} differs "
                f"from {tuple(expected.shape)}"
            )
        if actual.dtype != expected.dtype:
            raise ContractError(
                f"{context} tensor {key!r} dtype {actual.dtype} differs "
                f"from {expected.dtype}"
            )


def _require_exact_fields(
    value: Any, expected: frozenset[str], context: str
) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ContractError(f"{context} must be an object")
    actual = set(value)
    if actual != set(expected):
        missing = sorted(set(expected) - actual)
        unknown = sorted(actual - set(expected))
        raise ContractError(
            f"{context} schema fields differ; missing={missing}, unknown={unknown}"
        )
    return value


def _require_sha256(value: Any, context: str) -> str:
    if not isinstance(value, str) or _SHA256_PATTERN.fullmatch(value) is None:
        raise ContractError(f"{context} must be a lowercase SHA-256")
    return value


def _canonical_json_bytes(value: Mapping[str, Any]) -> bytes:
    try:
        return json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=True,
            allow_nan=False,
        ).encode("utf-8")
    except (MemoryError, OverflowError, RecursionError, TypeError, ValueError) as error:
        raise ContractError(f"value is not bounded canonical JSON: {error}") from error


def _canonical_sha256(value: Mapping[str, Any]) -> str:
    return hashlib.sha256(_canonical_json_bytes(value)).hexdigest()


def _json_clone(value: Mapping[str, Any], context: str) -> dict[str, Any]:
    encoded = _canonical_json_bytes(value)
    try:
        text = encoded.decode("ascii")
    except UnicodeDecodeError as error:
        raise ContractError(f"{context} is not canonical ASCII JSON") from error
    return _loads_json(text, context)


def _normalized_path(path: Path, context: str) -> Path:
    expanded = Path(path).expanduser()
    if ".." in expanded.parts:
        raise ContractError(f"{context} must not contain path traversal")
    return Path(os.path.abspath(os.fspath(expanded)))


def _reject_symlink_components(
    path: Path, context: str, *, allow_missing_leaf: bool
) -> None:
    current = Path(path.anchor)
    parts = path.parts[1:] if path.is_absolute() else path.parts
    for index, part in enumerate(parts):
        current = current / part
        is_leaf = index == len(parts) - 1
        try:
            metadata = os.lstat(current)
        except FileNotFoundError:
            if allow_missing_leaf and is_leaf:
                return
            raise ContractError(f"{context} component does not exist: {current}")
        except OSError as error:
            raise ContractError(f"cannot inspect {context} component {current}: {error}") from error
        if stat.S_ISLNK(metadata.st_mode):
            raise ContractError(f"{context} must not contain a symlink: {current}")


def _require_regular_path(
    path: Path,
    context: str,
    *,
    maximum_bytes: int,
    required_suffix: str | None = None,
) -> tuple[Path, os.stat_result]:
    normalized = _normalized_path(path, context)
    _reject_symlink_components(normalized, context, allow_missing_leaf=False)
    if required_suffix is not None and normalized.suffix != required_suffix:
        raise ContractError(f"{context} must use the {required_suffix} suffix")
    try:
        metadata = os.stat(normalized, follow_symlinks=False)
    except OSError as error:
        raise ContractError(f"cannot inspect {context} {normalized}: {error}") from error
    if not stat.S_ISREG(metadata.st_mode) or metadata.st_size <= 0:
        raise ContractError(f"{context} must be a nonempty regular file")
    if metadata.st_size > maximum_bytes:
        raise ContractError(
            f"{context} size {metadata.st_size} exceeds {maximum_bytes} bytes"
        )
    return normalized, metadata


def _identity(metadata: os.stat_result) -> tuple[int, int, int, int, int]:
    return (
        metadata.st_dev,
        metadata.st_ino,
        metadata.st_size,
        metadata.st_mtime_ns,
        metadata.st_ctime_ns,
    )


def _hash_open_file(stream: Any) -> str:
    stream.seek(0)
    digest = hashlib.sha256()
    for block in iter(lambda: stream.read(READ_CHUNK_BYTES), b""):
        digest.update(block)
    stream.seek(0)
    return digest.hexdigest()


def _validate_state_key(value: Any, context: str) -> str:
    if (
        not isinstance(value, str)
        or not value
        or len(value) > 512
        or _STATE_KEY_PATTERN.fullmatch(value) is None
    ):
        raise ContractError(f"{context} is not a canonical model state key")
    return value


def _canonical_array(value: Any, key: str) -> np.ndarray:
    try:
        array = value.detach().cpu().contiguous().numpy()
    except (AttributeError, RuntimeError, TypeError, ValueError) as error:
        raise ContractError(f"model state tensor {key!r} is not exportable: {error}") from error
    if array.dtype.hasobject or array.dtype.fields is not None or array.dtype.subdtype is not None:
        raise ContractError(f"model state tensor {key!r} uses an unsafe dtype")
    if array.dtype.itemsize > 1:
        if array.dtype.byteorder == ">" or (
            array.dtype.byteorder == "=" and sys.byteorder == "big"
        ):
            array = array.byteswap().newbyteorder("<")
        else:
            array = array.astype(array.dtype.newbyteorder("<"), copy=False)
    array = np.ascontiguousarray(array)
    if array.dtype.str not in _ALLOWED_DTYPES:
        raise ContractError(
            f"model state tensor {key!r} dtype {array.dtype.str!r} is not supported"
        )
    if array.ndim > 8 or any(dimension <= 0 for dimension in array.shape):
        raise ContractError(f"model state tensor {key!r} has an unsupported shape")
    if array.nbytes <= 0 or array.nbytes > MAX_TENSOR_BYTES:
        raise ContractError(f"model state tensor {key!r} exceeds its size gate")
    if array.dtype.kind in "fc" and not bool(np.isfinite(array).all()):
        raise ContractError(f"model state tensor {key!r} contains NaN or Inf")
    return array


def _npy_payload(array: np.ndarray) -> bytes:
    stream = BytesIO()
    try:
        np.save(stream, array, allow_pickle=False)
    except (MemoryError, TypeError, ValueError) as error:
        raise ContractError(f"cannot serialize runtime tensor as NPY: {error}") from error
    payload = stream.getvalue()
    if len(payload) > array.nbytes + MAX_NPY_OVERHEAD_BYTES:
        raise ContractError("runtime tensor NPY header exceeds the bounded overhead")
    return payload


def _validate_model_config(value: Any, expected_sha256: Any) -> dict[str, Any]:
    config = _require_exact_fields(value, _MODEL_CONFIG_FIELDS, "model_config")
    cloned = _json_clone(config, "model_config")
    # HH_260906 - Keep v0 and physical v1 identities distinct inside bundle v1.
    if cloned["model_id"] not in SUPPORTED_MODEL_IDS:
        raise ContractError("model_config.model_id is not supported by bundle v1")
    integer_limits = {
        "camera_count": (1, 16),
        "calibration_features": (1, 128),
        "ego_features": (1, 256),
        "route_points": (2, 1024),
        "future_points": (2, 256),
        "candidate_count": (1, 32),
        "image_width": (16, 2048),
        "image_height": (16, 2048),
        "image_channels": (1, 8),
        "image_grid_width": (1, 64),
        "image_grid_height": (1, 64),
        "image_embedding": (8, 2048),
        "camera_fusion_width": (8, 4096),
        "ego_embedding": (8, 1024),
        "route_embedding": (8, 2048),
        "hidden_width": (16, 8192),
        "encoder_base_channels": (4, 512),
        "ego_history_frames": (1, 100),
    }
    for name, (minimum, maximum) in integer_limits.items():
        number = cloned[name]
        if (
            isinstance(number, bool)
            or not isinstance(number, int)
            or not minimum <= number <= maximum
        ):
            raise ContractError(f"model_config.{name} is outside the v1 bounds")
    if (
        cloned["camera_count"] != 6
        or cloned["calibration_features"] != 16
        or cloned["ego_features"] != 13
        or cloned["future_points"] != 64
        or cloned["image_channels"] != 3
    ):
        raise ContractError("model_config does not preserve the portable E2E tensor ABI")
    for name, minimum, maximum in (
        ("maximum_step_m", 0.1, 10.0),
        ("route_scale_m", 1.0, 10_000.0),
    ):
        number = cloned[name]
        if (
            isinstance(number, bool)
            or not isinstance(number, (int, float))
            or not math.isfinite(float(number))
            or not minimum <= float(number) <= maximum
        ):
            raise ContractError(f"model_config.{name} is outside the v1 bounds")
    if (
        cloned["model_id"] == "portable_e2e.perspective_trajectory.physical.v1"
        and not math.isclose(
            float(cloned["maximum_step_m"]),
            1.0,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        )
    ):
        raise ContractError("physical v1 maximum_step_m must be exactly 1.0")
    feature_height = cloned["image_height"]
    feature_width = cloned["image_width"]
    for _ in range(4):
        feature_height = (feature_height + 1) // 2
        feature_width = (feature_width + 1) // 2
    if (
        feature_height % cloned["image_grid_height"]
        or feature_width % cloned["image_grid_width"]
    ):
        raise ContractError("model_config image grid does not divide the encoder output")
    expected = _require_sha256(expected_sha256, "model_config_sha256")
    if _canonical_sha256(cloned) != expected:
        raise ContractError("model_config_sha256 does not match model_config")
    return cloned


def _validate_training_provenance(value: Any) -> dict[str, Any]:
    provenance = _require_exact_fields(
        value, _PROVENANCE_FIELDS, "training_provenance"
    )
    created = provenance["created_at_utc"]
    if not isinstance(created, str) or not created.endswith("Z") or len(created) > 64:
        raise ContractError("training_provenance.created_at_utc is invalid")
    _require_sha256(
        provenance["dataset_fingerprint_sha256"],
        "training_provenance.dataset_fingerprint_sha256",
    )
    if provenance["training_split"] != "train":
        raise ContractError("training_provenance.training_split must be train")
    episodes = provenance["training_episode_ids"]
    if (
        not isinstance(episodes, list)
        or not episodes
        or len(episodes) > 100_000
        or any(not isinstance(item, str) or not item or len(item) > 1024 for item in episodes)
        or episodes != sorted(set(episodes))
    ):
        raise ContractError("training_provenance.training_episode_ids are not canonical")
    _require_sha256(
        provenance["sampling_plan_sha256"],
        "training_provenance.sampling_plan_sha256",
    )
    sampling_plan = provenance["sampling_plan"]
    if not isinstance(sampling_plan, Mapping) or not sampling_plan:
        raise ContractError("training_provenance.sampling_plan is invalid")
    if _canonical_sha256(sampling_plan) != provenance["sampling_plan_sha256"]:
        raise ContractError("training_provenance sampling plan hash does not match")
    _require_exact_fields(
        provenance["sampling_metrics"],
        frozenset({"samples_seen", "domain_samples_seen"}),
        "training_provenance.sampling_metrics",
    )
    train_config = _require_exact_fields(
        provenance["train_config"], _TRAIN_CONFIG_FIELDS, "training_provenance.train_config"
    )
    loss_config = _require_exact_fields(
        provenance["loss_config"], _LOSS_CONFIG_FIELDS, "training_provenance.loss_config"
    )
    state = _require_exact_fields(
        provenance["state"], _STATE_FIELDS, "training_provenance.state"
    )
    _require_exact_fields(
        provenance["runtime_abi"],
        _RUNTIME_ABI_FIELDS,
        "training_provenance.runtime_abi",
    )
    device = provenance["device_abi"]
    if not isinstance(device, Mapping) or device.get("device_type") not in ("cpu", "cuda"):
        raise ContractError("training_provenance.device_abi is invalid")
    expected_device_fields = (
        {"device", "device_type", "device_name"}
        if device.get("device_type") == "cpu"
        else {
            "device",
            "device_type",
            "device_name",
            "compute_capability",
            "total_memory_bytes",
            "device_uuid",
        }
    )
    _require_exact_fields(
        device, frozenset(expected_device_fields), "training_provenance.device_abi"
    )
    for name in (
        "seed",
        "batch_size",
        "max_steps",
        "checkpoint_interval",
        "num_workers",
    ):
        number = train_config[name]
        minimum = 0 if name in ("seed", "num_workers") else 1
        if isinstance(number, bool) or not isinstance(number, int) or number < minimum:
            raise ContractError(f"training_provenance.train_config.{name} is invalid")
    for name in ("learning_rate", "weight_decay", "maximum_gradient_norm"):
        number = train_config[name]
        if (
            isinstance(number, bool)
            or not isinstance(number, (int, float))
            or not math.isfinite(float(number))
            or float(number) < 0.0
        ):
            raise ContractError(f"training_provenance.train_config.{name} is invalid")
    if (
        train_config["learning_rate"] <= 0.0
        or train_config["maximum_gradient_norm"] <= 0.0
        or not isinstance(train_config["verify_image_sha256"], bool)
        or train_config["sampling_policy"]
        not in ("uniform_without_replacement", "domain_balanced_without_replacement")
        or not isinstance(train_config["domain_ratios"], list)
    ):
        raise ContractError("training_provenance.train_config values are invalid")
    for name, number in loss_config.items():
        if (
            isinstance(number, bool)
            or not isinstance(number, (int, float))
            or not math.isfinite(float(number))
            or float(number) < 0.0
        ):
            raise ContractError(f"training_provenance.loss_config.{name} is invalid")
    if loss_config["xy_weight"] <= 0.0:
        raise ContractError("training_provenance.loss_config.xy_weight must be positive")
    for name in _STATE_FIELDS - {"domain_samples_seen"}:
        number = state[name]
        if isinstance(number, bool) or not isinstance(number, int) or number < 0:
            raise ContractError(f"training_provenance.state.{name} is invalid")
    state_domains = state["domain_samples_seen"]
    metrics = provenance["sampling_metrics"]
    if (
        not isinstance(state_domains, Mapping)
        or not state_domains
        or any(
            not isinstance(name, str)
            or isinstance(number, bool)
            or not isinstance(number, int)
            or number < 0
            for name, number in state_domains.items()
        )
        or metrics["samples_seen"] != state["samples_seen"]
        or metrics["domain_samples_seen"] != state_domains
        or sum(state_domains.values()) != state["samples_seen"]
    ):
        raise ContractError("training provenance sampling state is inconsistent")
    global_step = state["global_step"]
    max_steps = train_config["max_steps"]
    if (
        isinstance(global_step, bool)
        or not isinstance(global_step, int)
        or isinstance(max_steps, bool)
        or not isinstance(max_steps, int)
        or global_step <= 0
        or global_step != max_steps
    ):
        raise ContractError("training provenance is not a completed checkpoint")
    return _json_clone(provenance, "training_provenance")


def _extract_training_provenance(payload: Mapping[str, Any]) -> dict[str, Any]:
    missing = sorted(_PROVENANCE_FIELDS - set(payload))
    if missing:
        raise ContractError(f"source checkpoint training provenance is missing {missing}")
    selected = {name: payload[name] for name in sorted(_PROVENANCE_FIELDS)}
    return _validate_training_provenance(_json_clone(selected, "source checkpoint provenance"))


def _secure_checkpoint_decode(path: Path) -> tuple[Mapping[str, Any], str]:
    try:
        import torch
    except ImportError as error:
        raise ContractError("secure export requires modern PyTorch") from error
    from .train import _read_checkpoint_file

    return _read_checkpoint_file(path, torch.device("cpu"))


def _validated_checkpoint_for_export(
    source_path: Path, expected_source_sha256: str
) -> tuple[Mapping[str, Any], str, dict[str, Any], dict[str, Any]]:
    payload, actual_sha256 = _secure_checkpoint_decode(source_path)
    if actual_sha256 != expected_source_sha256:
        raise ContractError("source checkpoint SHA-256 does not match the operator pin")
    if payload.get("checkpoint_id") != SOURCE_CHECKPOINT_ID:
        raise ContractError("source checkpoint ID is not supported")
    corpus_sha256 = _require_sha256(
        payload.get("corpus_fingerprint_sha256"),
        "source checkpoint corpus_fingerprint_sha256",
    )
    model_config_sha256 = _require_sha256(
        payload.get("model_config_sha256"), "source checkpoint model_config_sha256"
    )
    model_config = _validate_model_config(
        payload.get("model_config"), model_config_sha256
    )
    provenance = _extract_training_provenance(payload)
    state_dict = payload.get("model_state_dict")
    if not isinstance(state_dict, Mapping) or not state_dict:
        raise ContractError("source checkpoint model_state_dict is missing")
    try:
        from .evaluate import _checkpoint_sampling_provenance
        from .model import ModelConfig, PerspectiveTrajectoryModel
        import torch

        _checkpoint_sampling_provenance(payload)
        parsed_config = ModelConfig.from_mapping(model_config)
        expected_model = PerspectiveTrajectoryModel(parsed_config)
        _validate_torch_state_dict_abi(
            state_dict,
            expected_model.state_dict(),
            torch,
            context="source checkpoint model state",
        )
        expected_model.load_state_dict(state_dict, strict=True)
    except KeyError as error:
        raise ContractError(f"source checkpoint model state is missing: {error}") from error
    except (RuntimeError, TypeError, ValueError) as error:
        if isinstance(error, ContractError):
            raise
        raise ContractError(f"source checkpoint validation failed: {error}") from error
    return state_dict, corpus_sha256, model_config, provenance


def _state_index_sha256(records: Sequence[Mapping[str, Any]]) -> str:
    index = {
        "tensors": [
            {
                "key": record["key"],
                "dtype": record["dtype"],
                "shape": record["shape"],
                "nbytes": record["nbytes"],
                "tensor_sha256": record["tensor_sha256"],
            }
            for record in records
        ]
    }
    return _canonical_sha256(index)


def _zip_info(name: str) -> zipfile.ZipInfo:
    info = zipfile.ZipInfo(filename=name, date_time=(1980, 1, 1, 0, 0, 0))
    info.compress_type = zipfile.ZIP_STORED
    info.create_system = 3
    info.external_attr = 0o100600 << 16
    return info


def _write_atomic_no_overwrite(
    output_path: Path, members: Sequence[tuple[str, bytes]]
) -> str:
    output = _normalized_path(output_path, "runtime bundle output")
    if output.suffix != ".npz":
        raise ContractError("runtime bundle output must use the .npz suffix")
    _reject_symlink_components(output.parent, "runtime bundle output parent", allow_missing_leaf=False)
    _reject_symlink_components(output, "runtime bundle output", allow_missing_leaf=True)
    try:
        parent_descriptor = os.open(
            output.parent,
            os.O_RDONLY
            | getattr(os, "O_DIRECTORY", 0)
            | getattr(os, "O_CLOEXEC", 0)
            | getattr(os, "O_NOFOLLOW", 0),
        )
    except OSError as error:
        raise ContractError(f"cannot open runtime bundle output directory: {error}") from error
    temporary_name = f".{output.name}.tmp.{os.getpid()}.{secrets.token_hex(8)}"
    descriptor = -1
    linked = False
    completed = False
    try:
        try:
            os.stat(output.name, dir_fd=parent_descriptor, follow_symlinks=False)
        except FileNotFoundError:
            pass
        else:
            raise ContractError(f"refusing to overwrite runtime bundle {output}")
        flags = os.O_RDWR | os.O_CREAT | os.O_EXCL | getattr(os, "O_CLOEXEC", 0)
        flags |= getattr(os, "O_NOFOLLOW", 0)
        descriptor = os.open(temporary_name, flags, 0o600, dir_fd=parent_descriptor)
        with os.fdopen(descriptor, "w+b", closefd=False) as stream:
            with zipfile.ZipFile(stream, mode="w", allowZip64=False) as archive:
                for member_name, payload in members:
                    archive.writestr(_zip_info(member_name), payload)
            stream.flush()
            os.fsync(descriptor)
            before = os.fstat(descriptor)
            if not stat.S_ISREG(before.st_mode) or before.st_size <= 0:
                raise ContractError("generated runtime bundle is not a regular file")
            if before.st_size > MAX_BUNDLE_FILE_BYTES:
                raise ContractError("generated runtime bundle exceeds its size gate")
            digest = _hash_open_file(stream)
            after = os.fstat(descriptor)
            if _identity(before) != _identity(after):
                raise ContractError("generated runtime bundle changed while hashing")
        os.link(
            temporary_name,
            output.name,
            src_dir_fd=parent_descriptor,
            dst_dir_fd=parent_descriptor,
            follow_symlinks=False,
        )
        linked = True
        os.fsync(parent_descriptor)
        os.unlink(temporary_name, dir_fd=parent_descriptor)
        temporary_name = ""
        os.fsync(parent_descriptor)
        completed = True
        return digest
    except FileExistsError as error:
        raise ContractError(f"refusing to overwrite runtime bundle {output}") from error
    except ContractError:
        raise
    except (OSError, RuntimeError, ValueError, zipfile.LargeZipFile) as error:
        raise ContractError(f"cannot atomically write runtime bundle {output}: {error}") from error
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        if temporary_name:
            try:
                os.unlink(temporary_name, dir_fd=parent_descriptor)
            except FileNotFoundError:
                pass
        if linked and not completed:
            try:
                os.unlink(output.name, dir_fd=parent_descriptor)
            except FileNotFoundError:
                pass
        os.close(parent_descriptor)


def export_runtime_weight_bundle(
    source_checkpoint_path: Path,
    output_path: Path,
    *,
    expected_source_checkpoint_sha256: str,
) -> ExportedRuntimeWeightBundle:
    """Securely decode a pinned checkpoint and atomically export inference weights."""
    expected_source_sha256 = _require_sha256(
        expected_source_checkpoint_sha256, "expected source checkpoint SHA-256"
    )
    source, _ = _require_regular_path(
        source_checkpoint_path,
        "source checkpoint",
        maximum_bytes=8 * 1024 * 1024 * 1024,
        required_suffix=".pt",
    )
    state_dict, corpus_sha256, model_config, provenance = _validated_checkpoint_for_export(
        source, expected_source_sha256
    )
    records: list[dict[str, Any]] = []
    tensor_members: list[tuple[str, bytes]] = []
    seen_keys: set[str] = set()
    total_tensor_bytes = 0
    total_npy_bytes = 0
    raw_items = list(state_dict.items())
    if not raw_items or len(raw_items) > MAX_TENSOR_COUNT:
        raise ContractError("source checkpoint tensor count exceeds the bundle gate")
    for index, (raw_key, tensor) in enumerate(sorted(raw_items, key=lambda item: str(item[0]))):
        key = _validate_state_key(raw_key, f"model state key {index}")
        if key in seen_keys:
            raise ContractError(f"duplicate model state key {key!r}")
        seen_keys.add(key)
        array = _canonical_array(tensor, key)
        total_tensor_bytes += int(array.nbytes)
        if total_tensor_bytes > MAX_TOTAL_TENSOR_BYTES:
            raise ContractError("model state exceeds the total tensor byte gate")
        payload = _npy_payload(array)
        total_npy_bytes += len(payload)
        member = f"tensors/{index:06d}.npy"
        raw_bytes = array.tobytes(order="C")
        record = {
            "index": index,
            "key": key,
            "member": member,
            "dtype": array.dtype.str,
            "shape": list(array.shape),
            "nbytes": int(array.nbytes),
            "npy_nbytes": len(payload),
            "tensor_sha256": hashlib.sha256(raw_bytes).hexdigest(),
            "npy_sha256": hashlib.sha256(payload).hexdigest(),
        }
        records.append(record)
        tensor_members.append((member, payload))
    model_config_sha256 = _canonical_sha256(model_config)
    manifest = {
        "bundle_id": BUNDLE_ID,
        "schema_version": BUNDLE_SCHEMA_VERSION,
        "serialization": dict(_SERIALIZATION_CONTRACT),
        "source_checkpoint": {
            "checkpoint_id": SOURCE_CHECKPOINT_ID,
            "sha256": expected_source_sha256,
        },
        "model_config": model_config,
        "model_config_sha256": model_config_sha256,
        "corpus_fingerprint_sha256": corpus_sha256,
        "training_provenance": provenance,
        "training_provenance_sha256": _canonical_sha256(provenance),
        "tensor_count": len(records),
        "total_tensor_bytes": total_tensor_bytes,
        "total_npy_bytes": total_npy_bytes,
        "model_state_sha256": _state_index_sha256(records),
        "tensors": records,
    }
    manifest_payload = _canonical_json_bytes(manifest) + b"\n"
    if len(manifest_payload) > MAX_MANIFEST_BYTES:
        raise ContractError("runtime bundle manifest exceeds its size gate")
    output = _normalized_path(output_path, "runtime bundle output")
    bundle_sha256 = _write_atomic_no_overwrite(
        output,
        [(MANIFEST_MEMBER, manifest_payload), *tensor_members],
    )
    return ExportedRuntimeWeightBundle(
        path=output,
        bundle_sha256=bundle_sha256,
        source_checkpoint_sha256=expected_source_sha256,
        model_config_sha256=model_config_sha256,
        corpus_fingerprint_sha256=corpus_sha256,
        tensor_count=len(records),
        total_tensor_bytes=total_tensor_bytes,
    )


def _validate_member_name(value: str) -> None:
    raw_parts = value.split("/")
    if (
        not value
        or "\\" in value
        or "\x00" in value
        or value.startswith("/")
        or PurePosixPath(value).is_absolute()
        or any(part in ("", ".", "..") for part in raw_parts)
    ):
        raise ContractError(f"runtime bundle member contains path traversal: {value!r}")


def _read_member(archive: zipfile.ZipFile, info: zipfile.ZipInfo, limit: int) -> bytes:
    if info.file_size <= 0 or info.file_size > limit:
        raise ContractError(f"runtime bundle member {info.filename!r} exceeds its size gate")
    try:
        with archive.open(info, mode="r") as stream:
            payload = stream.read(limit + 1)
    except (MemoryError, OSError, RuntimeError, zipfile.BadZipFile) as error:
        raise ContractError(
            f"cannot read runtime bundle member {info.filename!r}: {error}"
        ) from error
    if len(payload) != info.file_size or len(payload) > limit:
        raise ContractError(f"runtime bundle member {info.filename!r} size is inconsistent")
    return payload


def _validate_archive_members(archive: zipfile.ZipFile) -> dict[str, zipfile.ZipInfo]:
    if archive.comment:
        raise ContractError("runtime bundle archive comments are not allowed")
    infos = archive.infolist()
    if not infos or len(infos) > MAX_TENSOR_COUNT + 1:
        raise ContractError("runtime bundle member count exceeds its gate")
    names = [info.filename for info in infos]
    if len(names) != len(set(names)):
        raise ContractError("runtime bundle contains duplicate member names")
    total = 0
    by_name: dict[str, zipfile.ZipInfo] = {}
    for info in infos:
        _validate_member_name(info.filename)
        mode = (info.external_attr >> 16) & 0o170000
        if info.is_dir() or mode == stat.S_IFLNK:
            raise ContractError("runtime bundle directories and symlinks are not allowed")
        if info.flag_bits & 0x1:
            raise ContractError("runtime bundle encrypted members are not allowed")
        if info.compress_type != zipfile.ZIP_STORED or info.compress_size != info.file_size:
            raise ContractError("runtime bundle compression is forbidden by the zip-bomb gate")
        if info.comment or info.extra:
            raise ContractError("runtime bundle member comments and extra fields are forbidden")
        total += info.file_size
        if total > MAX_MANIFEST_BYTES + MAX_TOTAL_TENSOR_BYTES + (
            MAX_TENSOR_COUNT * MAX_NPY_OVERHEAD_BYTES
        ):
            raise ContractError("runtime bundle uncompressed content exceeds its byte gate")
        by_name[info.filename] = info
    return by_name


def _validate_tensor_record(value: Any, index: int) -> dict[str, Any]:
    record = _require_exact_fields(
        value, _TENSOR_RECORD_FIELDS, f"tensor record {index}"
    )
    if record["index"] != index:
        raise ContractError("runtime bundle tensor indices are not contiguous")
    key = _validate_state_key(record["key"], f"tensor record {index} key")
    expected_member = f"tensors/{index:06d}.npy"
    if record["member"] != expected_member:
        raise ContractError("runtime bundle tensor member order is not canonical")
    _validate_member_name(expected_member)
    dtype = record["dtype"]
    if not isinstance(dtype, str) or dtype not in _ALLOWED_DTYPES:
        raise ContractError(f"runtime bundle tensor {key!r} dtype is not supported")
    shape = record["shape"]
    if (
        not isinstance(shape, list)
        or len(shape) > 8
        or any(isinstance(item, bool) or not isinstance(item, int) or item <= 0 for item in shape)
    ):
        raise ContractError(f"runtime bundle tensor {key!r} shape is invalid")
    nbytes = record["nbytes"]
    npy_nbytes = record["npy_nbytes"]
    if (
        isinstance(nbytes, bool)
        or not isinstance(nbytes, int)
        or nbytes <= 0
        or nbytes > MAX_TENSOR_BYTES
        or isinstance(npy_nbytes, bool)
        or not isinstance(npy_nbytes, int)
        or npy_nbytes <= nbytes
        or npy_nbytes > nbytes + MAX_NPY_OVERHEAD_BYTES
    ):
        raise ContractError(f"runtime bundle tensor {key!r} byte counts are invalid")
    try:
        expected_nbytes = math.prod(shape) * np.dtype(dtype).itemsize
    except (MemoryError, TypeError, ValueError) as error:
        raise ContractError(f"runtime bundle tensor {key!r} dtype is invalid") from error
    if expected_nbytes != nbytes:
        raise ContractError(f"runtime bundle tensor {key!r} shape byte count differs")
    _require_sha256(record["tensor_sha256"], f"runtime tensor {key!r} SHA-256")
    _require_sha256(record["npy_sha256"], f"runtime tensor {key!r} NPY SHA-256")
    return dict(record)


def _parse_manifest(payload: bytes) -> dict[str, Any]:
    try:
        manifest = _loads_json(payload.decode("utf-8"), "runtime bundle manifest")
    except UnicodeDecodeError as error:
        raise ContractError("runtime bundle manifest must be UTF-8") from error
    _require_exact_fields(manifest, _MANIFEST_FIELDS, "runtime bundle manifest")
    if manifest["bundle_id"] != BUNDLE_ID or manifest["schema_version"] != BUNDLE_SCHEMA_VERSION:
        raise ContractError("runtime bundle version is not supported")
    if manifest["serialization"] != _SERIALIZATION_CONTRACT:
        raise ContractError("runtime bundle serialization contract has drifted")
    source = _require_exact_fields(
        manifest["source_checkpoint"],
        frozenset({"checkpoint_id", "sha256"}),
        "runtime bundle source_checkpoint",
    )
    if source["checkpoint_id"] != SOURCE_CHECKPOINT_ID:
        raise ContractError("runtime bundle source checkpoint ID is not supported")
    _require_sha256(source["sha256"], "runtime bundle source checkpoint SHA-256")
    model_config = _validate_model_config(
        manifest["model_config"], manifest["model_config_sha256"]
    )
    manifest["model_config"] = model_config
    _require_sha256(
        manifest["corpus_fingerprint_sha256"],
        "runtime bundle corpus fingerprint",
    )
    provenance = _validate_training_provenance(manifest["training_provenance"])
    if _canonical_sha256(provenance) != _require_sha256(
        manifest["training_provenance_sha256"], "training provenance SHA-256"
    ):
        raise ContractError("runtime bundle training provenance hash does not match")
    manifest["training_provenance"] = provenance
    records = manifest["tensors"]
    count = manifest["tensor_count"]
    if (
        isinstance(count, bool)
        or not isinstance(count, int)
        or count <= 0
        or count > MAX_TENSOR_COUNT
        or not isinstance(records, list)
        or len(records) != count
    ):
        raise ContractError("runtime bundle tensor count is invalid")
    parsed_records = [_validate_tensor_record(record, index) for index, record in enumerate(records)]
    keys = [record["key"] for record in parsed_records]
    if keys != sorted(keys) or len(keys) != len(set(keys)):
        raise ContractError("runtime bundle tensor keys are duplicate or not canonical")
    total_tensor_bytes = sum(record["nbytes"] for record in parsed_records)
    total_npy_bytes = sum(record["npy_nbytes"] for record in parsed_records)
    if (
        manifest["total_tensor_bytes"] != total_tensor_bytes
        or total_tensor_bytes > MAX_TOTAL_TENSOR_BYTES
        or manifest["total_npy_bytes"] != total_npy_bytes
    ):
        raise ContractError("runtime bundle tensor totals are inconsistent")
    if _state_index_sha256(parsed_records) != _require_sha256(
        manifest["model_state_sha256"], "runtime bundle model state SHA-256"
    ):
        raise ContractError("runtime bundle model state index hash does not match")
    manifest["tensors"] = parsed_records
    return manifest


def _decode_tensor(payload: bytes, record: Mapping[str, Any]) -> np.ndarray:
    key = record["key"]
    if len(payload) != record["npy_nbytes"]:
        raise ContractError(f"runtime tensor {key!r} NPY size does not match")
    if hashlib.sha256(payload).hexdigest() != record["npy_sha256"]:
        raise ContractError(f"runtime tensor {key!r} NPY SHA-256 does not match")
    try:
        array = np.load(BytesIO(payload), allow_pickle=False)
    except (MemoryError, OSError, TypeError, ValueError) as error:
        raise ContractError(f"cannot safely decode runtime tensor {key!r}: {error}") from error
    if not isinstance(array, np.ndarray):
        raise ContractError(f"runtime tensor {key!r} did not decode as an array")
    if array.dtype.hasobject or array.dtype.fields is not None or array.dtype.subdtype is not None:
        raise ContractError(f"runtime tensor {key!r} uses an unsafe dtype")
    if array.dtype.str != record["dtype"] or list(array.shape) != record["shape"]:
        raise ContractError(f"runtime tensor {key!r} ABI does not match its manifest")
    array = np.ascontiguousarray(array)
    if array.nbytes != record["nbytes"]:
        raise ContractError(f"runtime tensor {key!r} byte count does not match")
    if hashlib.sha256(array.tobytes(order="C")).hexdigest() != record["tensor_sha256"]:
        raise ContractError(f"runtime tensor {key!r} SHA-256 does not match")
    if array.dtype.kind in "fc" and not bool(np.isfinite(array).all()):
        raise ContractError(f"runtime tensor {key!r} contains NaN or Inf")
    array.setflags(write=False)
    return array


def load_runtime_weight_bundle(
    bundle_path: Path,
    *,
    expected_bundle_sha256: str,
    expected_source_checkpoint_sha256: str,
    expected_model_config_sha256: str,
    expected_corpus_fingerprint_sha256: str,
) -> RuntimeWeightBundle:
    """Load a fully pinned NumPy bundle without invoking torch.load or pickle."""
    expected_bundle = _require_sha256(expected_bundle_sha256, "expected bundle SHA-256")
    expected_source = _require_sha256(
        expected_source_checkpoint_sha256, "expected source checkpoint SHA-256"
    )
    expected_model = _require_sha256(
        expected_model_config_sha256, "expected model config SHA-256"
    )
    expected_corpus = _require_sha256(
        expected_corpus_fingerprint_sha256, "expected corpus fingerprint"
    )
    path, inspected = _require_regular_path(
        bundle_path,
        "runtime weight bundle",
        maximum_bytes=MAX_BUNDLE_FILE_BYTES,
        required_suffix=".npz",
    )
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(path, flags)
    except OSError as error:
        raise ContractError(f"cannot safely open runtime weight bundle {path}: {error}") from error
    try:
        before = os.fstat(descriptor)
        if _identity(before) != _identity(inspected):
            raise ContractError("runtime weight bundle changed before it was opened")
        with os.fdopen(descriptor, "rb", closefd=False) as stream:
            actual_bundle = _hash_open_file(stream)
            if actual_bundle != expected_bundle:
                raise ContractError("runtime weight bundle SHA-256 does not match")
            try:
                with zipfile.ZipFile(stream, mode="r", allowZip64=False) as archive:
                    members = _validate_archive_members(archive)
                    manifest_info = members.get(MANIFEST_MEMBER)
                    if manifest_info is None:
                        raise ContractError("runtime bundle manifest is missing")
                    manifest_payload = _read_member(
                        archive, manifest_info, MAX_MANIFEST_BYTES
                    )
                    manifest = _parse_manifest(manifest_payload)
                    expected_names = {MANIFEST_MEMBER}
                    expected_names.update(record["member"] for record in manifest["tensors"])
                    if set(members) != expected_names:
                        raise ContractError("runtime bundle members differ from the manifest")
                    arrays: dict[str, np.ndarray] = {}
                    for record in manifest["tensors"]:
                        info = members[record["member"]]
                        payload = _read_member(archive, info, record["npy_nbytes"])
                        arrays[record["key"]] = _decode_tensor(payload, record)
            except ContractError:
                raise
            except (MemoryError, OSError, RuntimeError, ValueError, zipfile.BadZipFile) as error:
                raise ContractError(f"cannot parse runtime weight bundle: {error}") from error
        after = os.fstat(descriptor)
        if _identity(before) != _identity(after):
            raise ContractError("runtime weight bundle changed while it was loaded")
    finally:
        os.close(descriptor)
    source_sha256 = manifest["source_checkpoint"]["sha256"]
    model_sha256 = manifest["model_config_sha256"]
    corpus_sha256 = manifest["corpus_fingerprint_sha256"]
    if source_sha256 != expected_source:
        raise ContractError("runtime bundle source checkpoint pin does not match")
    if model_sha256 != expected_model:
        raise ContractError("runtime bundle model config pin does not match")
    if corpus_sha256 != expected_corpus:
        raise ContractError("runtime bundle corpus pin does not match")
    return RuntimeWeightBundle(
        path=path,
        bundle_sha256=actual_bundle,
        source_checkpoint_sha256=source_sha256,
        model_config_sha256=model_sha256,
        corpus_fingerprint_sha256=corpus_sha256,
        model_config=MappingProxyType(dict(manifest["model_config"])),
        training_provenance=MappingProxyType(dict(manifest["training_provenance"])),
        numpy_state_dict=MappingProxyType(arrays),
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    export = subparsers.add_parser("export")
    export.add_argument("--source-checkpoint", type=Path, required=True)
    export.add_argument("--source-checkpoint-sha256", required=True)
    export.add_argument("--output", type=Path, required=True)
    verify = subparsers.add_parser("verify")
    verify.add_argument("--bundle", type=Path, required=True)
    verify.add_argument("--bundle-sha256", required=True)
    verify.add_argument("--source-checkpoint-sha256", required=True)
    verify.add_argument("--model-config-sha256", required=True)
    verify.add_argument("--corpus-fingerprint-sha256", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "export":
            result = export_runtime_weight_bundle(
                args.source_checkpoint,
                args.output,
                expected_source_checkpoint_sha256=args.source_checkpoint_sha256,
            )
            report = {
                "bundle_id": BUNDLE_ID,
                "bundle_sha256": result.bundle_sha256,
                "source_checkpoint_sha256": result.source_checkpoint_sha256,
                "model_config_sha256": result.model_config_sha256,
                "corpus_fingerprint_sha256": result.corpus_fingerprint_sha256,
                "tensor_count": result.tensor_count,
                "total_tensor_bytes": result.total_tensor_bytes,
                "vehicle_control_approved": False,
            }
        else:
            loaded = load_runtime_weight_bundle(
                args.bundle,
                expected_bundle_sha256=args.bundle_sha256,
                expected_source_checkpoint_sha256=args.source_checkpoint_sha256,
                expected_model_config_sha256=args.model_config_sha256,
                expected_corpus_fingerprint_sha256=args.corpus_fingerprint_sha256,
            )
            report = {
                "bundle_id": BUNDLE_ID,
                "bundle_sha256": loaded.bundle_sha256,
                "source_checkpoint_sha256": loaded.source_checkpoint_sha256,
                "model_config_sha256": loaded.model_config_sha256,
                "corpus_fingerprint_sha256": loaded.corpus_fingerprint_sha256,
                "tensor_count": len(loaded.numpy_state_dict),
                "vehicle_control_approved": False,
            }
    except ContractError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
