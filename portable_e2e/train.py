"""Real PyTorch training entry point for the portable trajectory baseline.

Unlike ``control_flow_smoke``, this module decodes camera images, creates a
neural network, performs backpropagation, and writes PyTorch checkpoints.  GPU
use is never automatic: a CUDA device must be selected explicitly.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
import fcntl
from functools import wraps
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import platform
import random
import stat
import sys
from typing import Any, Mapping, Sequence

import numpy as np
from PIL import __version__ as pillow_version
import torch
from torch import Tensor
from torch.nn.utils import clip_grad_norm_
from torch.optim import AdamW
from torch.utils.data import DataLoader

from .contract import ContractError, _loads_json
from .dataset import LoadedDataset, fingerprint_examples, load_training_examples
from .losses import TrajectoryLossConfig, trajectory_loss
from .model import ModelConfig, PerspectiveTrajectoryModel, parameter_count
from .torch_dataset import Common10TorchDataset


TRAINER_ID = "portable_e2e.pytorch_trainer.v1"
CHECKPOINT_ID = "portable_e2e.pytorch_checkpoint.v1"
KNOWN_DOMAINS = ("carla", "real")
UNIFORM_SAMPLING_POLICY = "uniform_without_replacement"
DOMAIN_BALANCED_SAMPLING_POLICY = "domain_balanced_without_replacement"
UNIFORM_ORDER_ALGORITHM = "proportional_weighted_interleave_v1"
BALANCED_ORDER_ALGORITHM = "ratio_weighted_interleave_v1"
# Public checkpoint resource contract. The portable baseline is intentionally
# much smaller than this ceiling; the headroom preserves existing research
# checkpoints while rejecting oversized files before hashing or deserialization.
MAX_CHECKPOINT_FILE_BYTES = 8 * 1024 * 1024 * 1024


@dataclass(frozen=True)
class TrainConfig:
    seed: int = 20260903
    batch_size: int = 4
    learning_rate: float = 1.0e-4
    weight_decay: float = 1.0e-4
    max_steps: int = 1000
    checkpoint_interval: int = 100
    num_workers: int = 0
    maximum_gradient_norm: float = 5.0
    verify_image_sha256: bool = True
    sampling_policy: str = UNIFORM_SAMPLING_POLICY
    domain_ratios: tuple[tuple[str, int], ...] = ()

    def validate(self) -> None:
        integer_limits = {
            "seed": (self.seed, 0, 2**63 - 1),
            "batch_size": (self.batch_size, 1, 1024),
            "max_steps": (self.max_steps, 1, 10**9),
            "checkpoint_interval": (self.checkpoint_interval, 1, 10**9),
            "num_workers": (self.num_workers, 0, 256),
        }
        for name, (value, minimum, maximum) in integer_limits.items():
            if isinstance(value, bool) or not isinstance(value, int):
                raise ContractError(f"train.{name} must be an integer")
            if not minimum <= value <= maximum:
                raise ContractError(f"train.{name} must be in [{minimum}, {maximum}]")
        numeric_limits = {
            "learning_rate": (self.learning_rate, 1.0e-9, 1.0),
            "weight_decay": (self.weight_decay, 0.0, 1.0),
            "maximum_gradient_norm": (self.maximum_gradient_norm, 1.0e-6, 10_000.0),
        }
        for name, (value, minimum, maximum) in numeric_limits.items():
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                or not minimum <= float(value) <= maximum
            ):
                raise ContractError(
                    f"train.{name} must be finite and in [{minimum}, {maximum}]"
                )
        if not isinstance(self.verify_image_sha256, bool):
            raise ContractError("train.verify_image_sha256 must be boolean")
        if self.sampling_policy not in (
            UNIFORM_SAMPLING_POLICY,
            DOMAIN_BALANCED_SAMPLING_POLICY,
        ):
            raise ContractError("train.sampling_policy is not supported")
        if not isinstance(self.domain_ratios, tuple) or any(
            not isinstance(item, tuple) or len(item) != 2
            for item in self.domain_ratios
        ):
            raise ContractError("train.domain_ratios must be canonical domain/weight pairs")
        if self.sampling_policy == UNIFORM_SAMPLING_POLICY:
            if self.domain_ratios:
                raise ContractError(
                    "uniform sampling must not define domain ratios"
                )
        else:
            if tuple(domain for domain, _ in self.domain_ratios) != KNOWN_DOMAINS:
                raise ContractError(
                    "domain-balanced sampling requires exactly carla and real ratios "
                    "in canonical order"
                )
            weights = tuple(weight for _, weight in self.domain_ratios)
            if any(
                isinstance(weight, bool)
                or not isinstance(weight, int)
                or not 1 <= weight <= 1_000_000
                for weight in weights
            ):
                raise ContractError("domain ratio weights must be positive integers")
            if math.gcd(*weights) != 1:
                raise ContractError("domain ratio weights must be reduced to lowest terms")

    def to_dict(self) -> dict[str, Any]:
        self.validate()
        return asdict(self)


@dataclass
class TrainState:
    epoch: int = 0
    next_batch_index: int = 0
    global_step: int = 0
    samples_seen: int = 0
    domain_samples_seen: dict[str, int] = field(default_factory=dict)


def _canonical_sha256(value: Mapping[str, Any]) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _require_sha256(value: str, context: str) -> str:
    if not isinstance(value, str) or len(value) != 64 or any(
        character not in "0123456789abcdef" for character in value
    ):
        raise ContractError(f"{context} must be a lowercase SHA-256")
    return value


def _dataset_episode_ids(dataset: Common10TorchDataset) -> tuple[str, ...]:
    episode_ids = tuple(sorted({example.episode_id for example in dataset.examples}))
    if not episode_ids or any(not value for value in episode_ids):
        raise ContractError("dataset episode IDs are missing")
    return episode_ids


def _atomic_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp.{os.getpid()}")
    if temporary.exists():
        raise ContractError(f"refusing to reuse temporary output {temporary}")
    payload = json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n"
    try:
        with temporary.open("x", encoding="utf-8") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def _atomic_torch_save(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp.{os.getpid()}")
    if temporary.exists():
        raise ContractError(f"refusing to reuse temporary checkpoint {temporary}")
    try:
        with temporary.open("xb") as stream:
            torch.save(dict(value), stream)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def _absolute_path(path: Path) -> Path:
    return Path(os.path.abspath(os.fspath(path.expanduser())))


@contextmanager
def _exclusive_run_lock(run_dir: Path):
    run_dir = _absolute_path(run_dir)
    run_dir.parent.mkdir(parents=True, exist_ok=True)
    lock_path = run_dir.parent / f".{run_dir.name}.portable-e2e.lock"
    flags = os.O_RDWR | os.O_CREAT
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        descriptor = os.open(lock_path, flags, 0o600)
    except OSError as error:
        raise ContractError(f"cannot open run lock safely {lock_path}: {error}") from error
    try:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise ContractError(f"another process is already using run {run_dir}") from error
        yield
    finally:
        os.close(descriptor)


def _serialize_run(function):
    @wraps(function)
    def wrapped(*args: Any, **kwargs: Any):
        if "run_dir" not in kwargs:
            raise ContractError("run_dir must be supplied as a keyword argument")
        run_dir = _absolute_path(Path(kwargs["run_dir"]))
        kwargs["run_dir"] = run_dir
        with _exclusive_run_lock(run_dir):
            return function(*args, **kwargs)

    return wrapped


def _append_jsonl_no_follow(path: Path, value: Mapping[str, Any]) -> None:
    payload = (
        json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n"
    ).encode("utf-8")
    flags = os.O_WRONLY | os.O_APPEND | os.O_CREAT
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        descriptor = os.open(path, flags, 0o600)
    except OSError as error:
        raise ContractError(f"cannot append metrics safely {path}: {error}") from error
    try:
        offset = 0
        while offset < len(payload):
            written = os.write(descriptor, payload[offset:])
            if written <= 0:
                raise OSError("metrics append made no forward progress")
            offset += written
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _reconcile_metrics(path: Path, state: TrainState) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise ContractError(f"resume metrics are not a regular file: {path}")
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as error:
        raise ContractError(f"cannot read resume metrics {path}: {error}") from error
    parsed: list[dict[str, Any]] = []
    for index, line in enumerate(lines, start=1):
        try:
            value = _loads_json(line, f"{path}:{index}")
        except ContractError:
            if index > state.global_step and index == len(lines):
                break
            raise
        if value.get("global_step") != index:
            raise ContractError("metrics log has a duplicate or non-contiguous global step")
        parsed.append(value)
    if len(parsed) < state.global_step:
        raise ContractError("metrics log ends before the latest checkpoint")
    if len(parsed) != len(lines) or len(parsed) > state.global_step:
        payload = "".join(
            json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)
            + "\n"
            for value in parsed[: state.global_step]
        )
        temporary = path.with_name(f".{path.name}.reconcile.{os.getpid()}")
        try:
            with temporary.open("x", encoding="utf-8") as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
        finally:
            if temporary.exists():
                temporary.unlink()
    return parsed[state.global_step - 1] if state.global_step else {}


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _select_device(requested: str) -> torch.device:
    if requested == "cpu":
        return torch.device("cpu")
    if not requested.startswith("cuda:"):
        raise ContractError("--device must be cpu or an explicit cuda:<index>")
    suffix = requested.split(":", 1)[1]
    if not suffix.isdigit():
        raise ContractError("CUDA selection needs an explicit numeric device index")
    if not torch.cuda.is_available():
        raise ContractError("CUDA was explicitly requested but is unavailable")
    index = int(suffix)
    if index >= torch.cuda.device_count():
        raise ContractError(f"CUDA device index {index} is unavailable")
    return torch.device(requested)


def _runtime_abi() -> dict[str, Any]:
    deterministic_enabled = (
        bool(torch.are_deterministic_algorithms_enabled())
        if hasattr(torch, "are_deterministic_algorithms_enabled")
        else None
    )
    return {
        "python": platform.python_version(),
        "torch": str(torch.__version__),
        "numpy": np.__version__,
        "pillow": pillow_version,
        "torch_cuda": (
            str(torch.version.cuda) if getattr(torch.version, "cuda", None) else None
        ),
        "cudnn": (
            torch.backends.cudnn.version()
            if hasattr(torch.backends, "cudnn") and torch.backends.cudnn.is_available()
            else None
        ),
        "deterministic_algorithms": deterministic_enabled,
        "cudnn_benchmark": bool(torch.backends.cudnn.benchmark),
        "cudnn_deterministic": bool(torch.backends.cudnn.deterministic),
        "cublas_workspace_config": os.environ.get("CUBLAS_WORKSPACE_CONFIG"),
        "torch_num_threads": int(torch.get_num_threads()),
        "torch_num_interop_threads": int(torch.get_num_interop_threads()),
        "omp_num_threads": os.environ.get("OMP_NUM_THREADS"),
        "mkl_num_threads": os.environ.get("MKL_NUM_THREADS"),
        "cuda_matmul_allow_tf32": bool(torch.backends.cuda.matmul.allow_tf32),
        "cudnn_allow_tf32": bool(torch.backends.cudnn.allow_tf32),
    }


def _device_abi(device: torch.device) -> dict[str, Any]:
    value: dict[str, Any] = {"device": str(device), "device_type": device.type}
    if device.type == "cuda":
        properties = torch.cuda.get_device_properties(device)
        value.update(
            {
                "device_name": properties.name,
                "compute_capability": list(torch.cuda.get_device_capability(device)),
                "total_memory_bytes": int(properties.total_memory),
                "device_uuid": (
                    str(properties.uuid) if getattr(properties, "uuid", None) else None
                ),
            }
        )
    else:
        value["device_name"] = platform.processor() or platform.machine() or "cpu"
    return value


def _read_checkpoint_file(
    path: Path, device: torch.device
) -> tuple[Mapping[str, Any], str]:
    """Safely open and weights-only decode one same-inode checkpoint."""
    if "weights_only" not in inspect.signature(torch.load).parameters:
        raise ContractError(
            "secure checkpoint loading requires PyTorch with weights_only support"
        )
    flags = os.O_RDONLY
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        descriptor = os.open(path, flags)
    except OSError as error:
        raise ContractError(f"cannot open checkpoint safely {path}: {error}") from error
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode) or before.st_size <= 0:
            raise ContractError(f"checkpoint is not a nonempty regular file: {path}")
        if before.st_size > MAX_CHECKPOINT_FILE_BYTES:
            raise ContractError(
                f"checkpoint size {before.st_size} exceeds limit "
                f"{MAX_CHECKPOINT_FILE_BYTES} bytes: {path}"
            )
        with os.fdopen(descriptor, "rb", closefd=False) as stream:
            hasher = hashlib.sha256()
            for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
                hasher.update(block)
            digest = hasher.hexdigest()
            stream.seek(0)
            try:
                payload = torch.load(stream, map_location=device, weights_only=True)
            except Exception as error:
                raise ContractError(f"cannot safely load checkpoint {path}: {error}") from error
        after = os.fstat(descriptor)
        identity_before = (
            before.st_dev,
            before.st_ino,
            before.st_size,
            before.st_mtime_ns,
            before.st_ctime_ns,
        )
        identity_after = (
            after.st_dev,
            after.st_ino,
            after.st_size,
            after.st_mtime_ns,
            after.st_ctime_ns,
        )
        if identity_before != identity_after:
            raise ContractError("checkpoint changed while it was being loaded")
    finally:
        os.close(descriptor)
    if not isinstance(payload, Mapping):
        raise ContractError("checkpoint root must be a mapping")
    return payload, digest


def _configure_determinism(device: torch.device) -> None:
    if device.type == "cuda" and os.environ.get("CUBLAS_WORKSPACE_CONFIG") not in (
        ":16:8",
        ":4096:8",
    ):
        raise ContractError(
            "CUDA runs require CUBLAS_WORKSPACE_CONFIG=:4096:8 (or :16:8) "
            "to make exact-resume determinism explicit"
        )
    if hasattr(torch, "use_deterministic_algorithms"):
        torch.use_deterministic_algorithms(True)
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True


def _seed_everything(seed: int, device: torch.device) -> None:
    random.seed(seed)
    np.random.seed(seed % (2**32))
    torch.random.default_generator.manual_seed(seed)
    if device.type == "cuda":
        with torch.cuda.device(device):
            torch.cuda.manual_seed(seed)
    _configure_determinism(device)


def _balanced_epoch_generator(seed: int, epoch: int, stream: str) -> torch.Generator:
    """Create a stable, independent CPU RNG stream for balanced sampling."""
    digest = hashlib.sha256(
        f"portable-e2e-balanced-v1\0{seed}\0{epoch}\0{stream}".encode("utf-8")
    ).digest()
    derived_seed = int.from_bytes(digest[:8], "big") % (2**63 - 1)
    generator = torch.Generator(device="cpu")
    generator.manual_seed(derived_seed)
    return generator


def _epoch_batches(
    size: int,
    *,
    batch_size: int,
    seed: int,
    epoch: int,
    domain_indices: Mapping[str, Sequence[int]] | None = None,
    sampling_plan: Mapping[str, Any] | None = None,
) -> tuple[tuple[int, ...], ...]:
    if size <= 0:
        raise ContractError("training dataset is empty")
    if sampling_plan is None:
        if domain_indices is not None:
            raise ContractError("domain indices require an explicit sampling plan")
        generator = torch.Generator(device="cpu")
        generator.manual_seed((seed + epoch) % (2**63 - 1))
        order = torch.randperm(size, generator=generator).tolist()
    else:
        if domain_indices is None:
            raise ContractError("sampling plan requires domain indices")
        available = {domain: len(indices) for domain, indices in domain_indices.items()}
        if sampling_plan.get("dataset_domain_counts") != available:
            raise ContractError("sampling plan no longer matches dataset domains")
        policy = sampling_plan.get("policy")
        expected_algorithm = {
            UNIFORM_SAMPLING_POLICY: UNIFORM_ORDER_ALGORITHM,
            DOMAIN_BALANCED_SAMPLING_POLICY: BALANCED_ORDER_ALGORITHM,
        }.get(policy)
        if sampling_plan.get("order_algorithm") != expected_algorithm:
            raise ContractError("sampling plan order algorithm is not supported")
        if policy not in (UNIFORM_SAMPLING_POLICY, DOMAIN_BALANCED_SAMPLING_POLICY):
            raise ContractError("sampling plan policy is not supported")
        quotas = sampling_plan.get("epoch_domain_sample_counts")
        if not isinstance(quotas, Mapping):
            raise ContractError("sampling plan domain quotas are missing")
        active_domains = tuple(
            domain for domain in KNOWN_DOMAINS if domain in domain_indices
        )
        if set(quotas) != set(active_domains):
            raise ContractError("sampling plan domain quotas do not match the dataset")
        if policy == UNIFORM_SAMPLING_POLICY and dict(quotas) != available:
            raise ContractError("uniform sampling must use every domain sample")
        selected_by_domain: dict[str, list[int]] = {}
        for domain in active_domains:
            indices = tuple(domain_indices[domain])
            quota = quotas.get(domain)
            if (
                isinstance(quota, bool)
                or not isinstance(quota, int)
                or not 0 < quota <= len(indices)
            ):
                raise ContractError("sampling plan domain quota is impossible")
            domain_generator = _balanced_epoch_generator(
                seed, epoch, f"domain:{domain}"
            )
            permutation = torch.randperm(
                len(indices), generator=domain_generator
            ).tolist()
            selected_by_domain[domain] = [
                indices[index] for index in permutation[:quota]
            ]

        # Randomize only equal-deficit tie priority once per epoch.  Smooth
        # weighted interleaving prevents an early max_steps stop from exposing
        # an arbitrarily one-sided domain prefix under either v1 policy.
        tie_generator = _balanced_epoch_generator(seed, epoch, "tie-priority")
        tie_permutation = torch.randperm(
            len(active_domains), generator=tie_generator
        ).tolist()
        tie_rank = {
            active_domains[domain_index]: rank
            for rank, domain_index in enumerate(tie_permutation)
        }
        used = {domain: 0 for domain in active_domains}
        total = sum(int(quotas[domain]) for domain in active_domains)
        order = []
        while len(order) < total:
            next_position = len(order) + 1
            candidates = [
                domain
                for domain in active_domains
                if used[domain] < int(quotas[domain])
            ]
            if not candidates:
                raise ContractError("weighted domain interleave ended early")
            domain = max(
                candidates,
                key=lambda name: (
                    next_position * int(quotas[name]) - used[name] * total,
                    -tie_rank[name],
                ),
            )
            order.append(selected_by_domain[domain][used[domain]])
            used[domain] += 1
        if used != {domain: int(quotas[domain]) for domain in active_domains}:
            raise ContractError("weighted domain interleave quota mismatch")
        if len(order) != sampling_plan.get("epoch_sample_count"):
            raise ContractError("sampling plan epoch size is inconsistent")
    return tuple(
        tuple(order[offset : offset + batch_size])
        for offset in range(0, len(order), batch_size)
    )


def _dataset_domain_indices(
    dataset: Common10TorchDataset,
) -> dict[str, tuple[int, ...]]:
    grouped: dict[str, list[int]] = {domain: [] for domain in KNOWN_DOMAINS}
    for index, example in enumerate(dataset.examples):
        domain = example.domain
        if domain not in KNOWN_DOMAINS:
            raise ContractError(
                f"training example {example.token!r} has unknown domain {domain!r}; "
                f"expected one of {KNOWN_DOMAINS}"
            )
        grouped[domain].append(index)
    return {
        domain: tuple(grouped[domain])
        for domain in KNOWN_DOMAINS
        if grouped[domain]
    }


def _sampling_plan(
    domain_indices: Mapping[str, Sequence[int]],
    train_config: TrainConfig,
) -> dict[str, Any]:
    """Return the immutable, finite epoch contract used by the sampler."""
    unknown = sorted(set(domain_indices) - set(KNOWN_DOMAINS))
    if unknown:
        raise ContractError("sampling plan contains unknown domains: " + ", ".join(unknown))
    counts = {
        domain: len(domain_indices[domain])
        for domain in KNOWN_DOMAINS
        if domain in domain_indices
    }
    if sum(counts.values()) != sum(len(indices) for indices in domain_indices.values()):
        raise ContractError("sampling plan domain index accounting is inconsistent")
    return _sampling_plan_from_counts(counts, train_config)


def _sampling_plan_from_counts(
    counts: Mapping[str, int],
    train_config: TrainConfig,
) -> dict[str, Any]:
    """Build a sampling plan without allocating index arrays for claimed counts."""
    train_config.validate()
    unknown = sorted(set(counts) - set(KNOWN_DOMAINS))
    if unknown:
        raise ContractError("sampling plan contains unknown domains: " + ", ".join(unknown))
    canonical_counts = {
        domain: counts[domain] for domain in KNOWN_DOMAINS if domain in counts
    }
    if not canonical_counts or any(
        isinstance(count, bool) or not isinstance(count, int) or count <= 0
        for count in canonical_counts.values()
    ):
        raise ContractError("sampling plan requires positive known-domain counts")

    if train_config.sampling_policy == UNIFORM_SAMPLING_POLICY:
        epoch_counts = dict(canonical_counts)
        ratios: dict[str, int] = {}
    else:
        if tuple(canonical_counts) != KNOWN_DOMAINS:
            raise ContractError(
                "domain-balanced sampling requires both carla and real samples"
            )
        ratios = dict(train_config.domain_ratios)
        rounds = min(
            canonical_counts[domain] // ratios[domain] for domain in KNOWN_DOMAINS
        )
        if rounds <= 0:
            raise ContractError(
                "domain ratio cannot form one without-replacement epoch from this dataset"
            )
        epoch_counts = {
            domain: rounds * ratios[domain] for domain in KNOWN_DOMAINS
        }

    epoch_sample_count = sum(epoch_counts.values())
    return {
        "policy": train_config.sampling_policy,
        "order_algorithm": (
            UNIFORM_ORDER_ALGORITHM
            if train_config.sampling_policy == UNIFORM_SAMPLING_POLICY
            else BALANCED_ORDER_ALGORITHM
        ),
        "seed_derivation": "sha256_seed_epoch_named_stream_v1",
        "known_domains": list(KNOWN_DOMAINS),
        "domain_ratios": ratios,
        "replacement": False,
        "dataset_domain_counts": canonical_counts,
        "epoch_domain_sample_counts": epoch_counts,
        "epoch_discarded_sample_counts": {
            domain: canonical_counts[domain] - epoch_counts[domain]
            for domain in canonical_counts
        },
        "epoch_sample_count": epoch_sample_count,
        "batch_size": train_config.batch_size,
        "batches_per_epoch": (
            epoch_sample_count + train_config.batch_size - 1
        )
        // train_config.batch_size,
    }


def _weighted_prefix_domain_counts(
    quotas: Mapping[str, int],
    *,
    seed: int,
    epoch: int,
    prefix_size: int,
) -> dict[str, int]:
    """Return exact v1 weighted-interleave domain counts for one prefix."""
    active_domains = tuple(domain for domain in KNOWN_DOMAINS if domain in quotas)
    if not active_domains or set(quotas) != set(active_domains) or any(
        isinstance(value, bool) or not isinstance(value, int) or value <= 0
        for value in quotas.values()
    ):
        raise ContractError("weighted prefix quotas must contain positive known domains")
    total = sum(quotas.values())
    if (
        isinstance(prefix_size, bool)
        or not isinstance(prefix_size, int)
        or not 0 <= prefix_size <= total
    ):
        raise ContractError("weighted prefix size is outside the epoch quota")
    if len(active_domains) == 1:
        return {active_domains[0]: prefix_size}
    tie_generator = _balanced_epoch_generator(seed, epoch, "tie-priority")
    tie_permutation = torch.randperm(
        len(active_domains), generator=tie_generator
    ).tolist()
    tie_rank = {
        active_domains[domain_index]: rank
        for rank, domain_index in enumerate(tie_permutation)
    }
    first, second = active_domains
    quotient, remainder = divmod(prefix_size * quotas[first], total)
    first_count = quotient
    if 2 * remainder > total or (
        2 * remainder == total and tie_rank[first] < tie_rank[second]
    ):
        first_count += 1
    return {first: first_count, second: prefix_size - first_count}


def _parse_domain_ratios(values: Sequence[str]) -> tuple[tuple[str, int], ...]:
    parsed: dict[str, int] = {}
    for value in values:
        if not isinstance(value, str) or value.count("=") != 1:
            raise ContractError("--domain-ratio must use DOMAIN=INTEGER")
        domain, raw_weight = value.split("=", 1)
        if domain not in KNOWN_DOMAINS:
            raise ContractError(f"unknown --domain-ratio domain: {domain!r}")
        if domain in parsed:
            raise ContractError(f"duplicate --domain-ratio domain: {domain}")
        if not raw_weight.isdigit():
            raise ContractError("--domain-ratio weights must be positive integers")
        parsed[domain] = int(raw_weight)
    return tuple((domain, parsed[domain]) for domain in KNOWN_DOMAINS if domain in parsed)


def _move_batch(batch: Mapping[str, Any], device: torch.device) -> dict[str, Any]:
    return {
        key: value.to(device, non_blocking=False) if isinstance(value, Tensor) else value
        for key, value in batch.items()
    }


def _nested_tensors_are_finite(value: Any) -> bool:
    if isinstance(value, Tensor):
        if torch.is_floating_point(value) or torch.is_complex(value):
            return bool(torch.isfinite(value).all().item())
        return True
    if isinstance(value, Mapping):
        return all(_nested_tensors_are_finite(child) for child in value.values())
    if isinstance(value, (list, tuple)):
        return all(_nested_tensors_are_finite(child) for child in value)
    return True


def _checkpoint_payload(
    *,
    model: PerspectiveTrajectoryModel,
    optimizer: AdamW,
    state: TrainState,
    dataset_fingerprint_sha256: str,
    corpus_fingerprint_sha256: str,
    model_config: ModelConfig,
    train_config: TrainConfig,
    loss_config: TrajectoryLossConfig,
    device: torch.device,
    training_split: str,
    training_episode_ids: Sequence[str],
    sampling_plan: Mapping[str, Any],
) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "checkpoint_id": CHECKPOINT_ID,
        "created_at_utc": _utc_now(),
        "dataset_fingerprint_sha256": dataset_fingerprint_sha256,
        "corpus_fingerprint_sha256": corpus_fingerprint_sha256,
        "training_split": training_split,
        "training_episode_ids": list(training_episode_ids),
        "sampling_plan": dict(sampling_plan),
        "sampling_plan_sha256": _canonical_sha256(sampling_plan),
        "sampling_metrics": {
            "samples_seen": state.samples_seen,
            "domain_samples_seen": dict(state.domain_samples_seen),
        },
        "model_config": model_config.to_dict(),
        "model_config_sha256": _canonical_sha256(model_config.to_dict()),
        "train_config": train_config.to_dict(),
        "loss_config": loss_config.to_dict(),
        "state": asdict(state),
        "model_state_dict": model.state_dict(),
        "optimizer_state_dict": optimizer.state_dict(),
        "torch_rng_state": torch.get_rng_state(),
        "runtime_abi": _runtime_abi(),
        "device_abi": _device_abi(device),
    }
    if device.type == "cuda":
        payload["cuda_rng_state"] = torch.cuda.get_rng_state(device)
    return payload


def _load_checkpoint(
    path: Path,
    *,
    model: PerspectiveTrajectoryModel,
    optimizer: AdamW,
    dataset_fingerprint_sha256: str,
    corpus_fingerprint_sha256: str,
    model_config: ModelConfig,
    train_config: TrainConfig,
    loss_config: TrajectoryLossConfig,
    device: torch.device,
    training_split: str,
    training_episode_ids: Sequence[str],
    dataset_size: int,
    domain_indices: Mapping[str, Sequence[int]],
    sampling_plan: Mapping[str, Any],
) -> TrainState:
    payload, _ = _read_checkpoint_file(path, device)
    if payload.get("checkpoint_id") != CHECKPOINT_ID:
        raise ContractError("checkpoint kind is not compatible with this trainer")
    if payload.get("dataset_fingerprint_sha256") != dataset_fingerprint_sha256:
        raise ContractError("checkpoint dataset fingerprint does not match")
    if payload.get("corpus_fingerprint_sha256") != corpus_fingerprint_sha256:
        raise ContractError("checkpoint corpus fingerprint does not match")
    if payload.get("model_config") != model_config.to_dict():
        raise ContractError("checkpoint model config does not match")
    if payload.get("model_config_sha256") != _canonical_sha256(model_config.to_dict()):
        raise ContractError("checkpoint model config fingerprint does not match")
    if payload.get("training_split") != training_split:
        raise ContractError("checkpoint training split does not match")
    if payload.get("training_episode_ids") != list(training_episode_ids):
        raise ContractError("checkpoint training episode IDs do not match")
    if payload.get("sampling_plan") != dict(sampling_plan):
        raise ContractError("checkpoint sampling plan does not match")
    if payload.get("sampling_plan_sha256") != _canonical_sha256(sampling_plan):
        raise ContractError("checkpoint sampling plan fingerprint does not match")
    if payload.get("runtime_abi") != _runtime_abi():
        raise ContractError("checkpoint runtime ABI does not match exact-resume runtime")
    if payload.get("device_abi") != _device_abi(device):
        raise ContractError("checkpoint device ABI does not match exact-resume device")
    stored_train = payload.get("train_config")
    if not isinstance(stored_train, dict):
        raise ContractError("checkpoint training config is missing")
    current_train = train_config.to_dict()
    if set(stored_train) != set(current_train):
        raise ContractError("checkpoint training config fields do not match")
    for key, value in current_train.items():
        if key == "max_steps":
            continue
        if stored_train[key] != value:
            raise ContractError(f"checkpoint immutable training field changed: {key}")
    stored_steps = stored_train.get("max_steps")
    if isinstance(stored_steps, bool) or not isinstance(stored_steps, int):
        raise ContractError("checkpoint max_steps is invalid")
    if train_config.max_steps < stored_steps:
        raise ContractError("resume max_steps cannot be reduced")
    if payload.get("loss_config") != loss_config.to_dict():
        raise ContractError("checkpoint loss config does not match")
    state_value = payload.get("state")
    if not isinstance(state_value, dict):
        raise ContractError("checkpoint state is missing")
    expected_state_fields = {
        "epoch",
        "next_batch_index",
        "global_step",
        "samples_seen",
        "domain_samples_seen",
    }
    if set(state_value) != expected_state_fields:
        raise ContractError("checkpoint state fields do not match")
    integer_state_fields = expected_state_fields - {"domain_samples_seen"}
    if any(
        isinstance(state_value[name], bool)
        or not isinstance(state_value[name], int)
        or state_value[name] < 0
        for name in integer_state_fields
    ):
        raise ContractError("checkpoint state must contain nonnegative integer cursors")
    state_domains = state_value["domain_samples_seen"]
    if (
        not isinstance(state_domains, dict)
        or tuple(state_domains) != tuple(domain_indices)
        or any(
            isinstance(value, bool) or not isinstance(value, int) or value < 0
            for value in state_domains.values()
        )
    ):
        raise ContractError("checkpoint domain sample counts are invalid")
    state = TrainState(**state_value)
    if payload.get("sampling_metrics") != {
        "samples_seen": state.samples_seen,
        "domain_samples_seen": state.domain_samples_seen,
    }:
        raise ContractError("checkpoint sampling metrics do not match its state")
    epoch_batches = _epoch_batches(
        dataset_size,
        batch_size=train_config.batch_size,
        seed=train_config.seed,
        epoch=state.epoch,
        domain_indices=domain_indices,
        sampling_plan=sampling_plan,
    )
    if state.next_batch_index > len(epoch_batches):
        raise ContractError("checkpoint batch cursor exceeds its epoch")
    expected_global_step = state.epoch * len(epoch_batches) + state.next_batch_index
    epoch_sample_count = int(sampling_plan["epoch_sample_count"])
    expected_samples_seen = state.epoch * epoch_sample_count + sum(
        len(batch) for batch in epoch_batches[: state.next_batch_index]
    )
    if state.global_step != expected_global_step:
        raise ContractError("checkpoint global step is inconsistent with its cursor")
    if state.samples_seen != expected_samples_seen:
        raise ContractError("checkpoint sample count is inconsistent with its cursor")
    index_domains = {
        index: domain for domain, indices in domain_indices.items() for index in indices
    }
    expected_domain_samples = {
        domain: state.epoch
        * int(sampling_plan["epoch_domain_sample_counts"][domain])
        for domain in domain_indices
    }
    for batch in epoch_batches[: state.next_batch_index]:
        for index in batch:
            domain = index_domains[index]
            expected_domain_samples[domain] += 1
    if state.domain_samples_seen != expected_domain_samples:
        raise ContractError(
            "checkpoint domain sample counts are inconsistent with its cursor"
        )
    if state.global_step > stored_steps:
        raise ContractError("checkpoint global step exceeds its configured target")
    try:
        model.load_state_dict(payload["model_state_dict"], strict=True)
        optimizer.load_state_dict(payload["optimizer_state_dict"])
        torch.set_rng_state(payload["torch_rng_state"].cpu())
        if device.type == "cuda":
            torch.cuda.set_rng_state(payload["cuda_rng_state"].cpu(), device)
    except (KeyError, RuntimeError, ValueError, TypeError) as error:
        raise ContractError(f"checkpoint tensors are incompatible: {error}") from error
    if not _nested_tensors_are_finite(model.state_dict()):
        raise FloatingPointError("checkpoint model contains NaN or Inf")
    if not _nested_tensors_are_finite(optimizer.state_dict()):
        raise FloatingPointError("checkpoint optimizer contains NaN or Inf")
    return state


def _loader_for_batches(
    dataset: Common10TorchDataset,
    batches: Sequence[Sequence[int]],
    *,
    num_workers: int,
    generator_seed: int,
) -> DataLoader:
    loader_generator = torch.Generator(device="cpu")
    loader_generator.manual_seed(generator_seed % (2**63 - 1))
    return DataLoader(
        dataset,
        batch_sampler=list(batches),
        num_workers=num_workers,
        pin_memory=False,
        generator=loader_generator,
    )


@_serialize_run
def train_model(
    dataset: Common10TorchDataset,
    *,
    run_dir: Path,
    dataset_fingerprint_sha256: str,
    model_config: ModelConfig,
    train_config: TrainConfig,
    loss_config: TrajectoryLossConfig,
    corpus_fingerprint_sha256: str | None = None,
    device_name: str = "cpu",
    resume: bool = False,
    training_split: str = "train",
) -> dict[str, Any]:
    """Train to ``max_steps`` and atomically checkpoint every configured interval."""
    model_config.validate()
    train_config.validate()
    loss_config.validate()
    if training_split != "train" or dataset.split != "train":
        raise ContractError("the training runner only accepts the train split")
    _require_sha256(dataset_fingerprint_sha256, "training dataset fingerprint")
    if dataset_fingerprint_sha256 != dataset.fingerprint_sha256:
        raise ContractError("caller dataset fingerprint does not match the tensor dataset")
    if not dataset.verify_image_sha256 or not train_config.verify_image_sha256:
        raise ContractError("real training requires image SHA-256 verification")
    device = _select_device(device_name)
    if corpus_fingerprint_sha256 is None:
        corpus_fingerprint_sha256 = dataset_fingerprint_sha256
    _require_sha256(corpus_fingerprint_sha256, "training corpus fingerprint")
    training_episode_ids = _dataset_episode_ids(dataset)
    domain_indices = _dataset_domain_indices(dataset)
    sampling_plan = _sampling_plan(domain_indices, train_config)
    sampling_plan_sha256 = _canonical_sha256(sampling_plan)
    _seed_everything(train_config.seed, device)
    run_dir = _absolute_path(run_dir)
    if run_dir.is_symlink():
        raise ContractError(f"run directory must not be a symlink: {run_dir}")
    checkpoint_path = run_dir / "checkpoints" / "latest.pt"
    manifest_path = run_dir / "run.json"
    metrics_path = run_dir / "metrics.jsonl"

    if resume:
        if not run_dir.is_dir() or run_dir.is_symlink():
            raise ContractError(f"resume run directory does not exist: {run_dir}")
        if manifest_path.is_symlink() or not manifest_path.is_file():
            raise ContractError("resume run manifest is not a regular file")
        checkpoint_dir = checkpoint_path.parent
        if checkpoint_dir.is_symlink() or not checkpoint_dir.is_dir():
            raise ContractError("resume checkpoint directory is not a regular directory")
        try:
            resume_manifest = _loads_json(
                manifest_path.read_text(encoding="utf-8"), str(manifest_path)
            )
        except OSError as error:
            raise ContractError(f"cannot read resume run manifest: {error}") from error
        if resume_manifest.get("trainer_id") != TRAINER_ID:
            raise ContractError("resume run manifest has the wrong trainer ID")
        if resume_manifest.get("dataset_fingerprint_sha256") != dataset_fingerprint_sha256:
            raise ContractError("resume run manifest dataset fingerprint does not match")
        if resume_manifest.get("corpus_fingerprint_sha256") != corpus_fingerprint_sha256:
            raise ContractError("resume run manifest corpus fingerprint does not match")
        if resume_manifest.get("training_episode_ids") != list(training_episode_ids):
            raise ContractError("resume run manifest episode IDs do not match")
        if resume_manifest.get("sampling_plan") != sampling_plan:
            raise ContractError("resume run manifest sampling plan does not match")
        if resume_manifest.get("sampling_plan_sha256") != sampling_plan_sha256:
            raise ContractError("resume run manifest sampling fingerprint does not match")
    else:
        if run_dir.exists():
            raise ContractError(f"run directory already exists: {run_dir}")
        run_dir.mkdir(parents=True)

    model = PerspectiveTrajectoryModel(model_config).to(device)
    optimizer = AdamW(
        model.parameters(),
        lr=float(train_config.learning_rate),
        weight_decay=float(train_config.weight_decay),
    )
    state = TrainState(
        domain_samples_seen={domain: 0 for domain in domain_indices}
    )
    last_metrics: dict[str, Any] = {}
    if resume:
        state = _load_checkpoint(
            checkpoint_path,
            model=model,
            optimizer=optimizer,
            dataset_fingerprint_sha256=dataset_fingerprint_sha256,
            corpus_fingerprint_sha256=corpus_fingerprint_sha256,
            model_config=model_config,
            train_config=train_config,
            loss_config=loss_config,
            device=device,
            training_split=training_split,
            training_episode_ids=training_episode_ids,
            dataset_size=len(dataset),
            domain_indices=domain_indices,
            sampling_plan=sampling_plan,
        )
        last_metrics = _reconcile_metrics(metrics_path, state)
        if state.global_step and last_metrics.get(
            "domain_samples_seen"
        ) != state.domain_samples_seen:
            raise ContractError(
                "metrics domain sample counts do not match the latest checkpoint"
            )
        resume_manifest["status"] = "RUNNING"
        resume_manifest["resumed_at_utc"] = _utc_now()
        resume_manifest["train_config"] = train_config.to_dict()
        _atomic_json(manifest_path, resume_manifest)
    else:
        manifest = {
            "trainer_id": TRAINER_ID,
            "created_at_utc": _utc_now(),
            "status": "RUNNING",
            "device": str(device),
            "dataset_fingerprint_sha256": dataset_fingerprint_sha256,
            "corpus_fingerprint_sha256": corpus_fingerprint_sha256,
            "training_split": training_split,
            "training_episode_ids": list(training_episode_ids),
            "dataset_size": len(dataset),
            "sampling_plan": sampling_plan,
            "sampling_plan_sha256": sampling_plan_sha256,
            "model_parameter_count": parameter_count(model),
            "model_config": model_config.to_dict(),
            "train_config": train_config.to_dict(),
            "loss_config": loss_config.to_dict(),
            "runtime": _runtime_abi(),
            "hardware": _device_abi(device),
            "warning": "RESEARCH CHECKPOINT — NOT APPROVED FOR VEHICLE CONTROL",
        }
        _atomic_json(manifest_path, manifest)

    index_domains = {
        index: domain for domain, indices in domain_indices.items() for index in indices
    }
    while state.global_step < train_config.max_steps:
        batches = _epoch_batches(
            len(dataset),
            batch_size=train_config.batch_size,
            seed=train_config.seed,
            epoch=state.epoch,
            domain_indices=domain_indices,
            sampling_plan=sampling_plan,
        )
        if state.next_batch_index >= len(batches):
            state.epoch += 1
            state.next_batch_index = 0
            continue
        remaining_batches = batches[state.next_batch_index :]
        start_batch_index = state.next_batch_index
        loader = _loader_for_batches(
            dataset,
            remaining_batches,
            num_workers=train_config.num_workers,
            generator_seed=(
                train_config.seed
                + state.epoch * 1_000_003
                + state.next_batch_index * 10_007
            ),
        )
        for batch_offset, raw_batch in enumerate(loader):
            batch_index = start_batch_index + batch_offset
            batch = _move_batch(raw_batch, device)
            model.train()
            optimizer.zero_grad()
            candidate_xy, candidate_speed, candidate_logits = model(
                batch["images"],
                batch["calibration"],
                batch["ego_history"],
                batch["ego_history_mask"],
                batch["route_xy"],
                batch["route_mask"],
            )
            losses = trajectory_loss(
                candidate_xy,
                candidate_speed,
                candidate_logits,
                batch["target_xy"],
                batch["target_speed_mps"],
                batch["target_valid"],
                loss_config,
                target_yaw=batch["target_yaw_rad"],
            )
            total_loss = losses["loss"]
            if not bool(torch.isfinite(total_loss).item()):
                raise FloatingPointError("training loss became NaN or Inf")
            total_loss.backward()
            gradient_norm = clip_grad_norm_(
                model.parameters(), float(train_config.maximum_gradient_norm)
            )
            if not bool(torch.isfinite(gradient_norm).item()):
                raise FloatingPointError("training gradient norm became NaN or Inf")
            optimizer.step()
            if not _nested_tensors_are_finite(model.state_dict()):
                raise FloatingPointError("training model parameters became NaN or Inf")
            if not _nested_tensors_are_finite(optimizer.state_dict()):
                raise FloatingPointError("training optimizer state became NaN or Inf")

            state.global_step += 1
            current_batch_size = int(batch["images"].shape[0])
            state.samples_seen += current_batch_size
            batch_domain_counts = {domain: 0 for domain in domain_indices}
            for index in batches[batch_index]:
                batch_domain_counts[index_domains[index]] += 1
            for domain, count in batch_domain_counts.items():
                state.domain_samples_seen[domain] += count
            state.next_batch_index = batch_index + 1
            last_metrics = {
                "global_step": state.global_step,
                "epoch": state.epoch,
                "samples_seen": state.samples_seen,
                "batch_domain_sample_counts": batch_domain_counts,
                "domain_samples_seen": dict(state.domain_samples_seen),
                "loss": float(total_loss.detach().cpu().item()),
                "regression_loss": float(losses["regression_loss"].cpu().item()),
                "candidate_score_loss": float(
                    losses["candidate_score_loss"].cpu().item()
                ),
                "selected_ade_m": float(losses["selected_ade_m"].cpu().item()),
                "selected_fde_m": float(losses["selected_fde_m"].cpu().item()),
                "selected_speed_mae_mps": float(
                    losses["selected_speed_mae_mps"].cpu().item()
                ),
                "selected_yaw_mae_rad": float(
                    losses["selected_yaw_mae_rad"].cpu().item()
                ),
                "selected_kinematic_speed_mae_mps": float(
                    losses["selected_kinematic_speed_mae_mps"].cpu().item()
                ),
                "gradient_norm": float(gradient_norm.detach().cpu().item()),
            }
            _append_jsonl_no_follow(metrics_path, last_metrics)
            if (
                state.global_step % train_config.checkpoint_interval == 0
                or state.global_step >= train_config.max_steps
            ):
                _atomic_torch_save(
                    checkpoint_path,
                    _checkpoint_payload(
                        model=model,
                        optimizer=optimizer,
                        state=state,
                        dataset_fingerprint_sha256=dataset_fingerprint_sha256,
                        corpus_fingerprint_sha256=corpus_fingerprint_sha256,
                        model_config=model_config,
                        train_config=train_config,
                        loss_config=loss_config,
                        device=device,
                        training_split=training_split,
                        training_episode_ids=training_episode_ids,
                        sampling_plan=sampling_plan,
                    ),
                )
            if state.global_step >= train_config.max_steps:
                break
        if state.next_batch_index >= len(batches):
            state.epoch += 1
            state.next_batch_index = 0

    manifest = _loads_json(manifest_path.read_text(encoding="utf-8"), str(manifest_path))
    manifest["status"] = "TRAINING_TARGET_REACHED"
    manifest["completed_at_utc"] = _utc_now()
    manifest["state"] = asdict(state)
    manifest["last_metrics"] = last_metrics
    _atomic_json(manifest_path, manifest)
    return {
        "status": "TRAINING_TARGET_REACHED",
        "run_dir": str(run_dir),
        "checkpoint": str(checkpoint_path),
        "state": asdict(state),
        "last_metrics": last_metrics,
        "vehicle_control_approved": False,
    }


def _load_model_config(path: Path) -> ModelConfig:
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as error:
        raise ContractError(f"cannot read model config {path}: {error}") from error
    return ModelConfig.from_mapping(_loads_json(text, str(path)))


def _slice_loaded(loaded: LoadedDataset, limit: int | None) -> LoadedDataset:
    if limit is None:
        return loaded
    if limit <= 0:
        raise ContractError("--limit-samples must be positive")
    examples = loaded.examples[:limit]
    if not examples:
        raise ContractError("sample limit selected no examples")
    digest = hashlib.sha256()
    digest.update(loaded.fingerprint_sha256.encode("ascii"))
    digest.update(b"\0limited\0")
    digest.update(str(limit).encode("ascii"))
    digest.update(b"\0")
    digest.update(fingerprint_examples(examples).encode("ascii"))
    return LoadedDataset(examples, digest.hexdigest(), loaded.validation_report, loaded.split)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Train the real image-based portable E2E trajectory baseline."
    )
    parser.add_argument("dataset", type=Path, help="validated common10 dataset root")
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument(
        "--model-config",
        type=Path,
        default=Path(__file__).with_name("config")
        / "perspective_trajectory_v0.model.json",
    )
    parser.add_argument("--split", choices=("train",), default="train")
    parser.add_argument("--device", default="cpu", help="cpu or explicit cuda:<index>")
    parser.add_argument("--seed", type=int, default=20260903)
    parser.add_argument("--batch-size", type=int, default=4)
    parser.add_argument("--learning-rate", type=float, default=1.0e-4)
    parser.add_argument("--weight-decay", type=float, default=1.0e-4)
    parser.add_argument("--max-steps", type=int, default=1000)
    parser.add_argument("--checkpoint-interval", type=int, default=100)
    parser.add_argument("--num-workers", type=int, default=0)
    parser.add_argument("--maximum-gradient-norm", type=float, default=5.0)
    parser.add_argument(
        "--sampling-policy",
        choices=(UNIFORM_SAMPLING_POLICY, DOMAIN_BALANCED_SAMPLING_POLICY),
        default=UNIFORM_SAMPLING_POLICY,
        help=(
            "finite epoch policy; balanced mode is without replacement and "
            "requires explicit carla and real ratios"
        ),
    )
    parser.add_argument(
        "--domain-ratio",
        action="append",
        default=[],
        metavar="DOMAIN=INTEGER",
        help="repeat exactly once for carla and real in domain-balanced mode",
    )
    parser.add_argument("--limit-samples", type=int)
    parser.add_argument("--resume", action="store_true")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        model_config = _load_model_config(args.model_config.expanduser().resolve())
        loaded = load_training_examples(
            args.dataset,
            split=args.split,
            mode="planning",
            check_image_hashes=True,
        )
        loaded = _slice_loaded(loaded, args.limit_samples)
        train_config = TrainConfig(
            seed=args.seed,
            batch_size=args.batch_size,
            learning_rate=args.learning_rate,
            weight_decay=args.weight_decay,
            max_steps=args.max_steps,
            checkpoint_interval=args.checkpoint_interval,
            num_workers=args.num_workers,
            maximum_gradient_norm=args.maximum_gradient_norm,
            verify_image_sha256=True,
            sampling_policy=args.sampling_policy,
            domain_ratios=_parse_domain_ratios(args.domain_ratio),
        )
        dataset = Common10TorchDataset(
            loaded.examples,
            model_config,
            verify_image_sha256=train_config.verify_image_sha256,
            split=loaded.split,
        )
        report = train_model(
            dataset,
            run_dir=args.run_dir,
            dataset_fingerprint_sha256=dataset.fingerprint_sha256,
            corpus_fingerprint_sha256=str(
                loaded.validation_report["dataset_fingerprint_sha256"]
            ),
            model_config=model_config,
            train_config=train_config,
            loss_config=TrajectoryLossConfig(),
            device_name=args.device,
            resume=args.resume,
            training_split=loaded.split,
        )
    except (ContractError, FloatingPointError, OSError, ValueError) as error:
        print(f"TRAINING_ERROR: {error}", file=sys.stderr)
        return 2
    print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
