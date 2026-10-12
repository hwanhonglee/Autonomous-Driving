"""HH_260906 - Run the sealed fresh-fit K0/K1 multiroute recovery experiment."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
from functools import wraps
import hashlib
import json
import os
from pathlib import Path
import random
import shutil
import subprocess
import sys
import time
from typing import Any, Callable, Mapping, Sequence

import torch
from torch import Tensor
from torch.nn.utils import clip_grad_norm_
from torch.optim import AdamW
from torch.utils.data import DataLoader

SOURCE_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(SOURCE_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_REPO_ROOT))

from portable_e2e.audit_runtime import _AuditAccumulator, audit_prediction
from portable_e2e.contract import ContractError
from portable_e2e.dataset import FEATURE_NAMES, TrainingExample, load_training_examples
from portable_e2e.evaluate import _accumulate_batch_metrics
from portable_e2e.losses import TrajectoryLossConfig, trajectory_loss
from portable_e2e.model import ModelConfig, PerspectiveTrajectoryModel, parameter_count
from portable_e2e.runtime_contract import RuntimeGateConfig
from portable_e2e.torch_dataset import Common10TorchDataset
from portable_e2e.train import (
    _canonical_sha256,
    _move_batch,
    _nested_tensors_are_finite,
    _seed_everything,
)


SCHEMA = "portable_e2e.moving_multiroute_recovery_k_plan.v1"
CHECKPOINT_SCHEMA = "portable_e2e.moving_multiroute_recovery_k_checkpoint.v1"
REPORT_SCHEMA = "portable_e2e.moving_multiroute_recovery_k_report.v1"
SEED = 20260906
BATCH_SIZE = 4
LOGICAL_EPOCHS = 20
BATCHES_PER_EPOCH = 750
MAX_STEPS = LOGICAL_EPOCHS * BATCHES_PER_EPOCH
SAMPLES_SEEN = MAX_STEPS * BATCH_SIZE
LEARNING_RATE = 1.0e-4
WEIGHT_DECAY = 1.0e-4
MAXIMUM_GRADIENT_NORM = 5.0
EXPECTED_PARAMETER_COUNT = 1_056_362
EXPECTED_K0_COUNT = 1475
EXPECTED_NEW_EPISODE_COUNT = 328
EXPECTED_K1_COUNT = EXPECTED_K0_COUNT + 4 * EXPECTED_NEW_EPISODE_COUNT
EXPECTED_DEV_COUNT = 665
GPU_UUID = "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
ARMS = ("K0_current1475", "K1_current_plus_successful4")
MODEL_RELATIVE = "portable_e2e/config/perspective_trajectory_physical_stopmix_v1.model.json"
SOURCE_RELATIVES = (
    "portable_e2e/audit_runtime.py",
    "portable_e2e/contract.py",
    "portable_e2e/dataset.py",
    "portable_e2e/evaluate.py",
    "portable_e2e/losses.py",
    "portable_e2e/model.py",
    "portable_e2e/runtime_contract.py",
    "portable_e2e/stop_primitive_research.py",
    "portable_e2e/torch_dataset.py",
    "portable_e2e/train.py",
    "scripts/e2e/run_moving_multiroute_recovery_k.py",
)
T07_30_EPISODE = (
    "carla_town07_town07_straight_s0000_p00_ClearNoon_s0000_dfc36d890f2a"
)
T07_15_EPISODE = (
    "carla_town07_town07_straight_s0000_p00_ClearNoon_s0000_f3f1a4d7b240"
)
LEFT_EPISODE = (
    "carla_c_track_1_0_7_c_track_1_0_7_left_s0000_p01_ClearNoon_s0000_9b5bfc42d8e7"
)
RIGHT_EPISODE = (
    "carla_town01_town01_right_s0001_p00_ClearNoon_s0001_1af7c32f1f1f"
)
K0_EPISODE_COUNTS = {
    T07_30_EPISODE: 309,
    T07_15_EPISODE: 328,
    LEFT_EPISODE: 304,
    RIGHT_EPISODE: 534,
}
NEW_ROUTE_IDS = (
    "town07_straight_s0000_p02",
    "c_track_1_0_7_straight_s0000_p06",
    "c_track_1_0_7_straight_s0000_p03",
    "town01_straight_s0000_p02",
)
EXCLUDED_ROUTE_ID = "town01_straight_s0000_p01"
EXCLUDED_ROUTE_SHA256 = (
    "913b44d56755d226f3e2957814924b021ad6913400a880c8aac19bb934fa4194"
)
DEV_EPISODE_COUNTS = {
    "carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9": 337,
    "carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320": 328,
}
RIGHT_MAXIMUM = {
    "ade_6p4s_m": 2.9353143748791353,
    "fde_6p4s_m": 7.793866114022117,
    "speed_mae_6p4s_mps": 1.1076154233086357,
}
STRAIGHT_MAXIMUM = {
    "ade_6p4s_m": 3.764740187947343,
    "fde_6p4s_m": 11.027441243572932,
    "speed_mae_6p4s_mps": 1.7877504960065935,
}
COMBINED_MAXIMUM = {
    "ade_6p4s_m": 3.5632455477589056,
    "fde_6p4s_m": 10.353536791729749,
    "speed_mae_6p4s_mps": 1.6350220532345592,
}
GEOMETRY_MINIMUM = {
    "selected_geometry_pass_count": 664,
    "any_candidate_geometry_pass_count": 664,
    "all_candidates_geometry_pass_count": 664,
}
DENIALS = {
    "automatic_retry": False,
    "optimizer_resume": False,
    "intermediate_checkpoint": False,
    "intermediate_dev_evaluation": False,
    "early_stopping": False,
    "test_read_or_use": False,
    "stationary_read_or_use": False,
    "automatic_promotion": False,
    "runtime_checkpoint_exported": False,
    "closed_loop_evaluation": False,
    "vehicle_control_approved": False,
    "deployment_approved": False,
    "new_autonomous_function_head_added": False,
}
LOSS_CONFIG = TrajectoryLossConfig(candidate_regret_weight=0.1)


def require(value: Any, message: str) -> None:
    if not value:
        raise ValueError(message)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def encoded(value: Any) -> bytes:
    return (
        json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n"
    ).encode("utf-8")


def compact(value: Any) -> bytes:
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")


def read_json(path: Path) -> Any:
    def pairs(rows: Sequence[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in rows:
            require(key not in result, f"duplicate JSON key: {key}")
            result[key] = value
        return result

    source = regular(path)
    return json.loads(
        source.read_text(encoding="utf-8"),
        object_pairs_hook=pairs,
        parse_constant=lambda value: require(False, f"nonfinite JSON: {value}"),
    )


def regular(path: Path) -> Path:
    source = Path(path).absolute()
    require(source.is_file() and not source.is_symlink(), f"regular file required: {source}")
    require(
        not any(parent.is_symlink() for parent in source.parents),
        f"symlink parent forbidden: {source}",
    )
    return source


def digest(path: Path) -> str:
    result = hashlib.sha256()
    with regular(path).open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def write_new(path: Path, value: Any) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("xb") as stream:
        stream.write(encoded(value))
        stream.flush()
        os.fsync(stream.fileno())


def save_new(path: Path, value: Mapping[str, Any]) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("xb") as stream:
        torch.save(dict(value), stream)
        stream.flush()
        os.fsync(stream.fileno())


def copy_new(source: Path, target: Path) -> None:
    source = regular(source)
    target = Path(target)
    target.parent.mkdir(parents=True, exist_ok=True)
    with source.open("rb") as input_stream, target.open("xb") as output_stream:
        shutil.copyfileobj(input_stream, output_stream, 1024 * 1024)
        output_stream.flush()
        os.fsync(output_stream.fileno())
    require(digest(source) == digest(target), "copied bytes differ")


def tensor_digest(values: Mapping[str, Tensor]) -> str:
    result = hashlib.sha256()
    for name, tensor in sorted(values.items()):
        value = tensor.detach().cpu().contiguous()
        header = compact([name, str(value.dtype), list(value.shape)])
        result.update(len(header).to_bytes(8, "big"))
        result.update(header)
        result.update(value.numpy().tobytes())
    return result.hexdigest()


def _derived_seed(*parts: Any) -> int:
    payload = "\0".join(str(part) for part in (SEED, *parts)).encode("utf-8")
    return int.from_bytes(hashlib.sha256(payload).digest()[:8], "big")


class _CyclicSelector:
    def __init__(self, values: Sequence[Any], stream: str, epoch: int) -> None:
        require(values, f"empty cyclic selector: {stream}")
        self.values = tuple(values)
        self.stream = stream
        self.epoch = epoch
        self.cycle = 0
        self.offset = 0
        self.order: list[Any] = []

    def take(self) -> Any:
        if self.offset >= len(self.order):
            self.order = list(self.values)
            random.Random(
                _derived_seed("cyclic", self.stream, self.epoch, self.cycle)
            ).shuffle(self.order)
            self.cycle += 1
            self.offset = 0
        value = self.order[self.offset]
        self.offset += 1
        return value


def episode_inventory(examples: Sequence[TrainingExample]) -> dict[str, int]:
    return dict(sorted(Counter(example.episode_id for example in examples).items()))


def _route_inventory(examples: Sequence[TrainingExample]) -> dict[str, tuple[str, ...]]:
    rows: dict[str, set[str]] = defaultdict(set)
    for example in examples:
        rows[example.route_id].add(example.episode_id)
    return {key: tuple(sorted(value)) for key, value in sorted(rows.items())}


def validate_examples(
    k0_examples: Sequence[TrainingExample],
    k1_examples: Sequence[TrainingExample],
    dev_examples: Sequence[TrainingExample],
) -> dict[str, Any]:
    for name, examples in (("K0", k0_examples), ("K1", k1_examples), ("DEV", dev_examples)):
        require(examples, f"{name} examples are empty")
        require(
            all(example.domain == "carla" for example in examples),
            f"{name} must contain only CARLA examples",
        )
        require(
            all(example.route_id != EXCLUDED_ROUTE_ID for example in examples),
            f"{name} contains excluded failed T01-S02 route",
        )
        require(
            all(EXCLUDED_ROUTE_SHA256 not in example.source_manifest_sha256 for example in examples),
            f"{name} contains excluded T01-S02 provenance",
        )
        tokens = [example.token for example in examples]
        require(len(tokens) == len(set(tokens)), f"{name} contains duplicate tokens")

    k0_counts = episode_inventory(k0_examples)
    k1_counts = episode_inventory(k1_examples)
    dev_counts = episode_inventory(dev_examples)
    require(k0_counts == K0_EPISODE_COUNTS, "K0 is not the exact current1475 cohort")
    require(len(k0_examples) == EXPECTED_K0_COUNT, "K0 sample count differs")
    require(dev_counts == DEV_EPISODE_COUNTS, "DEV episode identities or counts differ")
    require(len(dev_examples) == EXPECTED_DEV_COUNT, "DEV sample count differs")
    require(len(k1_examples) == EXPECTED_K1_COUNT, "K1 sample count differs")
    for episode, count in K0_EPISODE_COUNTS.items():
        require(k1_counts.get(episode) == count, f"K1 changed current episode {episode}")
    new_episode_ids = sorted(set(k1_counts) - set(k0_counts))
    require(len(new_episode_ids) == 4, "K1 must add exactly four successful episodes")
    require(
        all(k1_counts[episode] == EXPECTED_NEW_EPISODE_COUNT for episode in new_episode_ids),
        "each successful new K1 episode must contain exactly 328 samples",
    )
    routes = _route_inventory(k1_examples)
    for route_id in NEW_ROUTE_IDS:
        require(
            route_id in routes and len(routes[route_id]) == 1,
            f"K1 route identity differs: {route_id}",
        )
        require(routes[route_id][0] in new_episode_ids, f"K1 route is not a new episode: {route_id}")
    new_routes = {
        example.route_id for example in k1_examples if example.episode_id in new_episode_ids
    }
    require(new_routes == set(NEW_ROUTE_IDS), "K1 new route set differs")
    k0_tokens = {example.token for example in k0_examples}
    k1_current_tokens = {
        example.token for example in k1_examples if example.episode_id in K0_EPISODE_COUNTS
    }
    require(k0_tokens == k1_current_tokens, "K1 changed current1475 token identities")
    require(
        not (set(k0_counts) & set(dev_counts)) and not (set(k1_counts) & set(dev_counts)),
        "TRAIN and DEV episode leakage detected",
    )
    tags_by_episode: dict[str, set[str]] = defaultdict(set)
    for example in k1_examples:
        tags_by_episode[example.episode_id].update(example.scenario_tags)
    require("left" in tags_by_episode[LEFT_EPISODE], "left episode tag differs")
    require("right" in tags_by_episode[RIGHT_EPISODE], "right episode tag differs")
    straight_ids = set(k1_counts) - {LEFT_EPISODE, RIGHT_EPISODE}
    require(
        all("straight" in tags_by_episode[episode] for episode in straight_ids),
        "straight episode tag differs",
    )
    return {
        "K0_episode_counts": k0_counts,
        "K1_episode_counts": k1_counts,
        "DEV_episode_counts": dev_counts,
        "new_episode_ids": new_episode_ids,
        "new_route_episode_ids": {
            route_id: routes[route_id][0] for route_id in NEW_ROUTE_IDS
        },
    }


def arm_groups(examples: Sequence[TrainingExample], arm: str) -> dict[str, Any]:
    require(arm in ARMS, "unknown K arm")
    inventory = episode_inventory(examples)
    by_route = _route_inventory(examples)
    if arm == ARMS[0]:
        straight = {"town07_p00_15_30": (T07_30_EPISODE, T07_15_EPISODE)}
    else:
        straight = {
            "town07_p00_15_30": (T07_30_EPISODE, T07_15_EPISODE),
            "town07_p02": (by_route[NEW_ROUTE_IDS[0]][0],),
            "c_track_p06_p03": (
                by_route[NEW_ROUTE_IDS[1]][0],
                by_route[NEW_ROUTE_IDS[2]][0],
            ),
            "town01_p02": (by_route[NEW_ROUTE_IDS[3]][0],),
        }
    expected_ids = {LEFT_EPISODE, RIGHT_EPISODE}
    expected_ids.update(episode for group in straight.values() for episode in group)
    require(expected_ids == set(inventory), f"{arm} group coverage differs")
    return {"straight": straight, "left": (LEFT_EPISODE,), "right": (RIGHT_EPISODE,)}


def build_epoch_batches(
    examples: Sequence[TrainingExample], arm: str, epoch: int
) -> tuple[list[tuple[int, int, int, int]], dict[str, Any]]:
    require(0 <= epoch < LOGICAL_EPOCHS, "logical epoch out of range")
    groups = arm_groups(examples, arm)
    episode_indices: dict[str, list[int]] = defaultdict(list)
    for index, example in enumerate(examples):
        episode_indices[example.episode_id].append(index)
    sample_selectors = {
        episode: _CyclicSelector(indices, f"{arm}:sample:{episode}", epoch)
        for episode, indices in episode_indices.items()
    }
    episode_selectors = {
        name: _CyclicSelector(episodes, f"{arm}:episode:{name}", epoch)
        for name, episodes in groups["straight"].items()
    }
    left_selector = _CyclicSelector(
        episode_indices[LEFT_EPISODE], f"{arm}:left", epoch
    )
    right_selector = _CyclicSelector(
        episode_indices[RIGHT_EPISODE], f"{arm}:right", epoch
    )
    group_names = tuple(groups["straight"])
    group_stream = _CyclicSelector(group_names, f"{arm}:straight_groups", epoch)
    batches: list[tuple[int, int, int, int]] = []
    group_counts: Counter[str] = Counter()
    episode_counts: Counter[str] = Counter()
    for _batch_index in range(BATCHES_PER_EPOCH):
        straight_rows: list[int] = []
        selected_groups: list[str] = []
        for _straight_slot in range(2):
            group_name = group_stream.take()
            episode = episode_selectors[group_name].take()
            index = sample_selectors[episode].take()
            straight_rows.append(index)
            selected_groups.append(group_name)
            group_counts[group_name] += 1
            episode_counts[episode] += 1
        require(
            len(group_names) == 1 or selected_groups[0] != selected_groups[1],
            "two straight slots reused a physical group",
        )
        left_index = left_selector.take()
        right_index = right_selector.take()
        episode_counts[LEFT_EPISODE] += 1
        episode_counts[RIGHT_EPISODE] += 1
        batch = (straight_rows[0], straight_rows[1], left_index, right_index)
        require(len(set(batch)) == BATCH_SIZE, "within-batch sample reuse detected")
        batches.append(batch)
    expected_group_exposures = 1500 // len(group_names)
    require(
        set(group_counts.values()) == {expected_group_exposures},
        "straight physical groups are not cyclic-uniform",
    )
    token_batches = [
        [examples[index].token for index in batch] for batch in batches
    ]
    proof = {
        "epoch": epoch,
        "batch_count": len(batches),
        "sample_count": len(batches) * BATCH_SIZE,
        "direction_exposures": {"straight": 1500, "left": 750, "right": 750},
        "straight_group_exposures": dict(sorted(group_counts.items())),
        "episode_exposures": dict(sorted(episode_counts.items())),
        "token_batch_schedule_sha256": hashlib.sha256(compact(token_batches)).hexdigest(),
        "within_batch_duplicate_count": 0,
    }
    return batches, proof


def schedule_contract(examples: Sequence[TrainingExample], arm: str) -> dict[str, Any]:
    proofs = [build_epoch_batches(examples, arm, epoch)[1] for epoch in range(LOGICAL_EPOCHS)]
    totals: Counter[str] = Counter()
    group_totals: Counter[str] = Counter()
    for proof in proofs:
        totals.update(proof["episode_exposures"])
        group_totals.update(proof["straight_group_exposures"])
    require(sum(totals.values()) == SAMPLES_SEEN, "schedule total exposure differs")
    return {
        "algorithm": "hh260906_nested_sha256_cyclic_uniform_2straight_1left_1right_v1",
        "seed": SEED,
        "logical_epochs": LOGICAL_EPOCHS,
        "batches_per_epoch": BATCHES_PER_EPOCH,
        "steps": MAX_STEPS,
        "batch_size": BATCH_SIZE,
        "samples_seen": SAMPLES_SEEN,
        "replacement_between_cycles": True,
        "within_batch_reuse": False,
        "epoch_proofs": proofs,
        "total_episode_exposures": dict(sorted(totals.items())),
        "total_straight_group_exposures": dict(sorted(group_totals.items())),
    }


def manifest_scope_precheck(root: Path, role: str) -> dict[str, Any]:
    require(role in (*ARMS, "Town03_DEV"), "unknown K view role")
    source = Path(root).absolute()
    manifest = read_json(source / "dataset.json")
    require(isinstance(manifest, Mapping), "dataset manifest root differs")
    references = manifest.get("episodes")
    require(isinstance(references, list) and references, "dataset episode references missing")
    episode_ids: list[str] = []
    for reference in references:
        require(isinstance(reference, Mapping), "dataset episode reference differs")
        episode_id = reference.get("episode_id")
        manifest_path = reference.get("manifest")
        require(isinstance(episode_id, str) and episode_id, "dataset episode ID missing")
        require(isinstance(manifest_path, str) and manifest_path, "dataset episode path missing")
        lowered = (episode_id + "\0" + manifest_path).lower()
        require("test" not in lowered, "TEST reference forbidden before dataset loading")
        require("stationary" not in lowered, "stationary reference forbidden before dataset loading")
        episode_ids.append(episode_id)
    require(len(episode_ids) == len(set(episode_ids)), "duplicate manifest episode ID")
    observed = set(episode_ids)
    k0_ids = set(K0_EPISODE_COUNTS)
    dev_ids = set(DEV_EPISODE_COUNTS)
    unknown = observed - k0_ids - dev_ids
    if role == ARMS[0]:
        require(observed in (k0_ids, k0_ids | dev_ids), "K0 manifest scope differs")
    elif role == ARMS[1]:
        require(k0_ids <= observed and len(unknown) == 4, "K1/new-route manifest scope differs")
        require(observed in (k0_ids | unknown, k0_ids | unknown | dev_ids), "K1 manifest scope differs")
        for route_id in NEW_ROUTE_IDS:
            require(
                sum(route_id in episode_id for episode_id in unknown) == 1,
                f"new manifest episode identity does not bind route: {route_id}",
            )
    else:
        require(dev_ids <= observed, "DEV manifest episodes missing")
        require(
            observed == dev_ids
            or observed == k0_ids | dev_ids
            or (k0_ids <= observed and len(unknown) == 4 and observed == k0_ids | unknown | dev_ids),
            "DEV view manifest scope differs",
        )
        if unknown:
            for route_id in NEW_ROUTE_IDS:
                require(
                    sum(route_id in episode_id for episode_id in unknown) == 1,
                    f"DEV view new episode does not bind route: {route_id}",
                )
    return {
        "status": "NO_TEST_OR_STATIONARY_REFERENCES_BEFORE_LOADER",
        "episode_ids": sorted(episode_ids),
        "episode_reference_count": len(episode_ids),
    }


def _load_view(root: Path, split: str, role: str) -> tuple[Any, tuple[TrainingExample, ...], dict[str, Any]]:
    require(split in ("train", "val"), "K may read only TRAIN or DEV/val")
    scope = manifest_scope_precheck(root, role)
    loaded = load_training_examples(
        Path(root), split=split, mode="planning", check_image_hashes=True
    )
    return loaded, tuple(loaded.examples), scope


def _examples_sha256(examples: Sequence[TrainingExample]) -> str:
    rows = [
        {
            "token": example.token,
            "episode_id": example.episode_id,
            "route_id": example.route_id,
            "source_manifest_sha256": example.source_manifest_sha256,
        }
        for example in examples
    ]
    return hashlib.sha256(compact(rows)).hexdigest()


def _view_contract(
    root: Path,
    loaded: Any,
    examples: Sequence[TrainingExample],
    scope: Mapping[str, Any],
) -> dict[str, Any]:
    source = Path(root).absolute()
    manifest = source / "dataset.json"
    require(manifest.is_file() and not manifest.is_symlink(), "dataset manifest missing")
    return {
        "path": str(source),
        "dataset_manifest_sha256": digest(manifest),
        "loader_fingerprint_sha256": loaded.fingerprint_sha256,
        "corpus_fingerprint_sha256": loaded.validation_report.get(
            "dataset_fingerprint_sha256"
        ),
        "examples_sha256": _examples_sha256(examples),
        "sample_count": len(examples),
        "episode_counts": episode_inventory(examples),
        "scope_precheck": dict(scope),
    }


def build_plan(
    repo_root: Path,
    k0_root: Path,
    k1_root: Path,
    dev_root: Path,
    output_root: Path,
) -> dict[str, Any]:
    repo = Path(repo_root).absolute()
    require(repo.is_dir() and not repo.is_symlink(), "regular repository root required")
    require(repo.resolve() == SOURCE_REPO_ROOT, "runner must execute from the declared repository")
    k0_loaded, k0_examples, k0_scope = _load_view(k0_root, "train", ARMS[0])
    k1_loaded, k1_examples, k1_scope = _load_view(k1_root, "train", ARMS[1])
    dev_loaded, dev_examples, dev_scope = _load_view(dev_root, "val", "Town03_DEV")
    inventory = validate_examples(k0_examples, k1_examples, dev_examples)
    model_path = repo / MODEL_RELATIVE
    model_value = read_json(model_path)
    model_config = ModelConfig.from_mapping(model_value)
    _seed_everything(SEED, torch.device("cpu"))
    model = PerspectiveTrajectoryModel(model_config)
    require(parameter_count(model) == EXPECTED_PARAMETER_COUNT, "model parameter count differs")
    initial_model_state_sha256 = tensor_digest(model.state_dict())
    source_sha256 = {name: digest(repo / name) for name in SOURCE_RELATIVES}
    plan = {
        "schema": SCHEMA,
        "declared_at_utc": utc_now(),
        "repo_root": str(repo),
        "output_root": str(Path(output_root).absolute()),
        "runner_sha256": source_sha256[SOURCE_RELATIVES[-1]],
        "source_sha256": source_sha256,
        "model": {
            "path": MODEL_RELATIVE,
            "file_sha256": digest(model_path),
            "canonical_sha256": _canonical_sha256(model_config.to_dict()),
            "model_id": model_config.model_id,
            "parameter_count": EXPECTED_PARAMETER_COUNT,
            "fresh_full_parameter_fit": True,
            "seeded_initial_model_state_sha256": initial_model_state_sha256,
        },
        "gpu": {"physical_index": 0, "uuid": GPU_UUID, "visible_count": 1},
        "views": {
            ARMS[0]: _view_contract(k0_root, k0_loaded, k0_examples, k0_scope),
            ARMS[1]: _view_contract(k1_root, k1_loaded, k1_examples, k1_scope),
            "Town03_DEV": _view_contract(dev_root, dev_loaded, dev_examples, dev_scope),
        },
        "inventory": inventory,
        "fit": {
            "seed": SEED,
            "optimizer": "AdamW",
            "learning_rate": LEARNING_RATE,
            "weight_decay": WEIGHT_DECAY,
            "maximum_gradient_norm": MAXIMUM_GRADIENT_NORM,
            "batch_size": BATCH_SIZE,
            "logical_epochs": LOGICAL_EPOCHS,
            "batches_per_epoch": BATCHES_PER_EPOCH,
            "steps": MAX_STEPS,
            "samples_seen": SAMPLES_SEEN,
            "loss_config": LOSS_CONFIG.to_dict(),
            "same_initial_model_state_required": True,
            "optimizer_state_loaded": False,
        },
        "sampling": {
            ARMS[0]: schedule_contract(k0_examples, ARMS[0]),
            ARMS[1]: schedule_contract(k1_examples, ARMS[1]),
        },
        "dev": {
            "split": "val",
            "episodes": DEV_EPISODE_COUNTS,
            "sample_count": EXPECTED_DEV_COUNT,
            "final_checkpoint_only": True,
            "right_C_maximum": RIGHT_MAXIMUM,
            "straight_D_maximum": STRAIGHT_MAXIMUM,
            "combined_D_maximum": COMBINED_MAXIMUM,
            "geometry_minimum": GEOMETRY_MINIMUM,
            "K1_vs_K0_nonregression_count": 9,
        },
        "interpretation": {
            "j_status": "FAIL_4_OF_9",
            "j_combined_ade_excess_m": 0.00035987527213054915,
            "j_right_regressed": True,
            "town03_is_repeatedly_observed_dev": True,
            "single_seed_exploratory_only": True,
        },
        **DENIALS,
    }
    validate_plan(plan)
    return plan


def validate_plan(plan: Mapping[str, Any]) -> None:
    def valid_sha256(value: Any) -> bool:
        return (
            isinstance(value, str)
            and len(value) == 64
            and all(character in "0123456789abcdef" for character in value)
        )

    expected_fields = {
        "schema",
        "declared_at_utc",
        "repo_root",
        "output_root",
        "runner_sha256",
        "source_sha256",
        "model",
        "gpu",
        "views",
        "inventory",
        "fit",
        "sampling",
        "dev",
        "interpretation",
        *DENIALS,
    }
    require(set(plan) == expected_fields, "K top-level plan fields differ")
    require(plan.get("schema") == SCHEMA, "K plan schema differs")
    require(tuple(plan.get("sampling", {})) == ARMS, "K arm order differs")
    fit = plan.get("fit")
    require(isinstance(fit, Mapping), "K fit contract missing")
    require(
        fit
        == {
            "seed": SEED,
            "optimizer": "AdamW",
            "learning_rate": LEARNING_RATE,
            "weight_decay": WEIGHT_DECAY,
            "maximum_gradient_norm": MAXIMUM_GRADIENT_NORM,
            "batch_size": BATCH_SIZE,
            "logical_epochs": LOGICAL_EPOCHS,
            "batches_per_epoch": BATCHES_PER_EPOCH,
            "steps": MAX_STEPS,
            "samples_seen": SAMPLES_SEEN,
            "loss_config": LOSS_CONFIG.to_dict(),
            "same_initial_model_state_required": True,
            "optimizer_state_loaded": False,
        },
        "K fit contract differs",
    )
    model = plan.get("model", {})
    require(
        set(model)
        == {
            "path",
            "file_sha256",
            "canonical_sha256",
            "model_id",
            "parameter_count",
            "fresh_full_parameter_fit",
            "seeded_initial_model_state_sha256",
        }
        and model.get("path") == MODEL_RELATIVE
        and model.get("parameter_count") == EXPECTED_PARAMETER_COUNT
        and model.get("fresh_full_parameter_fit") is True,
        "K model contract differs",
    )
    for context, value in (
        ("runner", plan.get("runner_sha256")),
        ("model file", model.get("file_sha256")),
        ("model canonical", model.get("canonical_sha256")),
        ("initial model state", model.get("seeded_initial_model_state_sha256")),
    ):
        require(valid_sha256(value), f"{context} SHA-256 differs")
    require(
        set(plan.get("source_sha256", {})) == set(SOURCE_RELATIVES),
        "K source closure differs",
    )
    require(
        all(valid_sha256(value) for value in plan["source_sha256"].values()),
        "K source fingerprint differs",
    )
    require(
        plan.get("gpu") == {"physical_index": 0, "uuid": GPU_UUID, "visible_count": 1},
        "K GPU contract differs",
    )
    views = plan.get("views", {})
    require(
        views.get(ARMS[0], {}).get("sample_count") == EXPECTED_K0_COUNT
        and views.get(ARMS[1], {}).get("sample_count") == EXPECTED_K1_COUNT
        and views.get("Town03_DEV", {}).get("sample_count") == EXPECTED_DEV_COUNT,
        "K view denominators differ",
    )
    for role in (*ARMS, "Town03_DEV"):
        contract = views.get(role, {})
        require(
            set(contract)
            == {
                "path",
                "dataset_manifest_sha256",
                "loader_fingerprint_sha256",
                "corpus_fingerprint_sha256",
                "examples_sha256",
                "sample_count",
                "episode_counts",
                "scope_precheck",
            },
            f"{role} view contract fields differ",
        )
        require(
            contract["scope_precheck"].get("status")
            == "NO_TEST_OR_STATIONARY_REFERENCES_BEFORE_LOADER",
            f"{role} scope precheck differs",
        )
        require(
            all(
                valid_sha256(contract[name])
                for name in (
                    "dataset_manifest_sha256",
                    "loader_fingerprint_sha256",
                    "corpus_fingerprint_sha256",
                    "examples_sha256",
                )
            ),
            f"{role} view fingerprint differs",
        )
        scope_ids = contract["scope_precheck"].get("episode_ids")
        require(
            isinstance(scope_ids, list)
            and scope_ids == sorted(scope_ids)
            and len(scope_ids) == contract["scope_precheck"].get("episode_reference_count")
            and set(contract["episode_counts"]) <= set(scope_ids),
            f"{role} manifest scope identities differ",
        )
    inventory = plan.get("inventory", {})
    require(
        set(inventory)
        == {
            "K0_episode_counts",
            "K1_episode_counts",
            "DEV_episode_counts",
            "new_episode_ids",
            "new_route_episode_ids",
        }
        and inventory["K0_episode_counts"] == K0_EPISODE_COUNTS
        and inventory["DEV_episode_counts"] == DEV_EPISODE_COUNTS
        and len(inventory["new_episode_ids"]) == 4
        and inventory["new_episode_ids"] == sorted(inventory["new_episode_ids"])
        and set(inventory["new_route_episode_ids"]) == set(NEW_ROUTE_IDS)
        and set(inventory["new_route_episode_ids"].values())
        == set(inventory["new_episode_ids"]),
        "K inventory contract differs",
    )
    expected_k1_counts = dict(K0_EPISODE_COUNTS)
    expected_k1_counts.update(
        {episode: EXPECTED_NEW_EPISODE_COUNT for episode in inventory["new_episode_ids"]}
    )
    require(inventory["K1_episode_counts"] == expected_k1_counts, "K1 inventory counts differ")
    dev = plan.get("dev", {})
    require(
        dev.get("split") == "val"
        and dev.get("episodes") == DEV_EPISODE_COUNTS
        and dev.get("right_C_maximum") == RIGHT_MAXIMUM
        and dev.get("straight_D_maximum") == STRAIGHT_MAXIMUM
        and dev.get("combined_D_maximum") == COMBINED_MAXIMUM
        and dev.get("geometry_minimum") == GEOMETRY_MINIMUM
        and dev.get("K1_vs_K0_nonregression_count") == 9
        and dev.get("final_checkpoint_only") is True,
        "K DEV gate contract differs",
    )
    for arm in ARMS:
        sampling = plan["sampling"][arm]
        require(
            set(sampling)
            == {
                "algorithm",
                "seed",
                "logical_epochs",
                "batches_per_epoch",
                "steps",
                "batch_size",
                "samples_seen",
                "replacement_between_cycles",
                "within_batch_reuse",
                "epoch_proofs",
                "total_episode_exposures",
                "total_straight_group_exposures",
            }
            and sampling.get("seed") == SEED
            and sampling.get("logical_epochs") == LOGICAL_EPOCHS
            and sampling.get("batches_per_epoch") == BATCHES_PER_EPOCH
            and sampling.get("steps") == MAX_STEPS
            and sampling.get("samples_seen") == SAMPLES_SEEN
            and len(sampling.get("epoch_proofs", [])) == LOGICAL_EPOCHS,
            f"{arm} schedule contract differs",
        )
        for proof in sampling["epoch_proofs"]:
            require(
                set(proof)
                == {
                    "epoch",
                    "batch_count",
                    "sample_count",
                    "direction_exposures",
                    "straight_group_exposures",
                    "episode_exposures",
                    "token_batch_schedule_sha256",
                    "within_batch_duplicate_count",
                }
                and proof.get("batch_count") == BATCHES_PER_EPOCH
                and proof.get("direction_exposures")
                == {"straight": 1500, "left": 750, "right": 750}
                and proof.get("within_batch_duplicate_count") == 0,
                f"{arm} per-epoch exposure contract differs",
            )
            require(
                valid_sha256(proof.get("token_batch_schedule_sha256")),
                f"{arm} schedule fingerprint differs",
            )
        expected_group_totals = (
            {"town07_p00_15_30": 30000}
            if arm == ARMS[0]
            else {
                "c_track_p06_p03": 7500,
                "town01_p02": 7500,
                "town07_p00_15_30": 7500,
                "town07_p02": 7500,
            }
        )
        require(
            sampling.get("total_straight_group_exposures") == expected_group_totals,
            f"{arm} physical-group exposure totals differ",
        )
    for key, expected in DENIALS.items():
        require(plan.get(key) is expected, f"K denial changed: {key}")
    text = json.dumps(plan, sort_keys=True)
    require(EXCLUDED_ROUTE_ID not in text, "excluded failed T01-S02 entered K plan")
    require(EXCLUDED_ROUTE_SHA256 not in text, "excluded failed T01-S02 SHA entered K plan")


def verify_sources(plan: Mapping[str, Any]) -> None:
    repo = Path(plan["repo_root"])
    require(repo.resolve() == SOURCE_REPO_ROOT, "loaded runner repository differs")
    expected = plan.get("source_sha256")
    require(isinstance(expected, Mapping), "source fingerprints missing")
    require(set(expected) == set(SOURCE_RELATIVES), "source fingerprint set differs")
    for name in SOURCE_RELATIVES:
        require(digest(repo / name) == expected[name], f"source bytes differ: {name}")
    require(plan.get("runner_sha256") == digest(Path(__file__)), "runner bytes differ")
    model = plan["model"]
    require(digest(repo / MODEL_RELATIVE) == model["file_sha256"], "model config bytes differ")


def _verify_loaded_view(
    contract: Mapping[str, Any], loaded: Any, examples: Sequence[TrainingExample]
) -> None:
    require(len(examples) == contract["sample_count"], "view sample count differs")
    require(episode_inventory(examples) == contract["episode_counts"], "view episodes differ")
    require(loaded.fingerprint_sha256 == contract["loader_fingerprint_sha256"], "loader fingerprint differs")
    require(
        loaded.validation_report.get("dataset_fingerprint_sha256")
        == contract["corpus_fingerprint_sha256"],
        "corpus fingerprint differs",
    )
    require(_examples_sha256(examples) == contract["examples_sha256"], "example identities differ")
    require(
        digest(Path(contract["path"]) / "dataset.json")
        == contract["dataset_manifest_sha256"],
        "dataset manifest differs",
    )


def verify_views(plan: Mapping[str, Any]) -> tuple[Any, Any, dict[str, Any]]:
    loaded: dict[str, Any] = {}
    examples: dict[str, tuple[TrainingExample, ...]] = {}
    for arm in ARMS:
        loaded[arm], examples[arm], scope = _load_view(
            Path(plan["views"][arm]["path"]), "train", arm
        )
        require(scope == plan["views"][arm]["scope_precheck"], f"{arm} manifest scope differs")
        _verify_loaded_view(plan["views"][arm], loaded[arm], examples[arm])
    dev_loaded, dev_examples, dev_scope = _load_view(
        Path(plan["views"]["Town03_DEV"]["path"]), "val", "Town03_DEV"
    )
    require(
        dev_scope == plan["views"]["Town03_DEV"]["scope_precheck"],
        "DEV manifest scope differs",
    )
    _verify_loaded_view(plan["views"]["Town03_DEV"], dev_loaded, dev_examples)
    validate_examples(examples[ARMS[0]], examples[ARMS[1]], dev_examples)
    for arm in ARMS:
        observed = schedule_contract(examples[arm], arm)
        require(observed == plan["sampling"][arm], f"{arm} schedule proof differs")
    return loaded[ARMS[0]], loaded[ARMS[1]], {
        "loaded": dev_loaded,
        "examples": dev_examples,
    }


def preflight_gpu() -> dict[str, Any]:
    require(
        os.environ.get("CUDA_VISIBLE_DEVICES") == GPU_UUID,
        "CUDA_VISIBLE_DEVICES must bind only approved physical GPU0 by UUID",
    )
    rows = subprocess.check_output(
        [
            "nvidia-smi",
            "--query-gpu=index,uuid,memory.used,utilization.gpu",
            "--format=csv,noheader,nounits",
        ],
        timeout=20,
        text=True,
    ).splitlines()
    matching = [row for row in rows if GPU_UUID in row]
    require(len(matching) == 1, "approved GPU0 UUID missing")
    fields = [value.strip() for value in matching[0].split(",")]
    require(fields[0] == "0", "approved UUID is not physical GPU0")
    require(int(fields[2]) == 0 and int(fields[3]) == 0, "GPU0 must be idle before K")
    require(torch.cuda.is_available() and torch.cuda.device_count() == 1, "exactly one CUDA GPU must be visible")
    device = torch.device("cuda:0")
    properties = torch.cuda.get_device_properties(device)
    observed_uuid = str(getattr(properties, "uuid", ""))
    require(observed_uuid in (GPU_UUID, GPU_UUID.removeprefix("GPU-")), "PyTorch GPU UUID differs")
    return {
        "physical_index": 0,
        "uuid": GPU_UUID,
        "memory_used_mib_before_torch": int(fields[2]),
        "utilization_percent_before_torch": int(fields[3]),
        "name": properties.name,
    }


def check_run_environment(plan: Mapping[str, Any]) -> Path:
    workspace = Path.home() / "personal/hwanhong/portable_e2e"
    repo = Path(plan["repo_root"])
    output = Path(plan["output_root"])
    require(Path(sys.prefix).absolute() == workspace / "venvs/py312", "personal py312 venv required")
    require(repo.resolve().is_relative_to(workspace.resolve()), "K repository escaped personal workspace")
    require(Path.cwd().resolve() == repo.resolve(), "K must run from its declared repository")
    require(
        output.absolute().is_relative_to(workspace / "runs/diagnostics")
        and output.absolute() != workspace / "runs/diagnostics",
        "K output must be a fresh personal diagnostic directory",
    )
    expected_environment = {
        "CUDA_VISIBLE_DEVICES": GPU_UUID,
        "CUBLAS_WORKSPACE_CONFIG": ":4096:8",
        "PYTHONNOUSERSITE": "1",
        "OMP_NUM_THREADS": "16",
        "MKL_NUM_THREADS": "16",
    }
    require(
        all(os.environ.get(key) == value for key, value in expected_environment.items()),
        "K deterministic GPU environment differs",
    )
    require(
        not any(
            os.environ.get(key)
            for key in ("PYTHONPATH", "PYTHONHOME", "PYTHONSTARTUP", "PYTHONUSERBASE")
        ),
        "inherited Python search overrides are forbidden",
    )
    return workspace


@contextmanager
def gpu0_lease(workspace: Path):
    path = workspace / "runs/campaigns/.gpu0_training.lock"
    require(path.parent.is_dir() and not path.parent.is_symlink(), "safe GPU lease parent required")
    flags = os.O_RDWR | os.O_CREAT | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(path, flags, 0o600)
    try:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise ValueError("GPU0 is already leased by another training run") from error
        yield
    finally:
        os.close(descriptor)


def leased_gpu0(function: Callable[[Path], dict[str, Any]]) -> Callable[[Path], dict[str, Any]]:
    @wraps(function)
    def wrapped(plan_path: Path) -> dict[str, Any]:
        plan = read_json(plan_path)
        validate_plan(plan)
        workspace = check_run_environment(plan)
        with gpu0_lease(workspace):
            return function(plan_path)

    return wrapped


def _clone_state(state: Mapping[str, Tensor]) -> dict[str, Tensor]:
    return {name: tensor.detach().cpu().clone() for name, tensor in state.items()}


def _checkpoint_payload(
    arm: str,
    model: PerspectiveTrajectoryModel,
    model_config: ModelConfig,
    initial_sha256: str,
    dataset: Common10TorchDataset,
    schedule: Mapping[str, Any],
    last_metrics: Mapping[str, Any],
) -> dict[str, Any]:
    return {
        "checkpoint_schema": CHECKPOINT_SCHEMA,
        "arm": arm,
        "created_at_utc": utc_now(),
        "model_config": model_config.to_dict(),
        "model_parameter_count": parameter_count(model),
        "model_state_dict": model.state_dict(),
        "initial_model_state_sha256": initial_sha256,
        "final_model_state_sha256": tensor_digest(model.state_dict()),
        "training_split": "train",
        "training_dataset_fingerprint_sha256": dataset.fingerprint_sha256,
        "training_episode_ids": sorted({example.episode_id for example in dataset.examples}),
        "fit": {
            "seed": SEED,
            "optimizer": "AdamW",
            "optimizer_state_included": False,
            "optimizer_resumed": False,
            "learning_rate": LEARNING_RATE,
            "weight_decay": WEIGHT_DECAY,
            "maximum_gradient_norm": MAXIMUM_GRADIENT_NORM,
            "batch_size": BATCH_SIZE,
            "logical_epochs": LOGICAL_EPOCHS,
            "batches_per_epoch": BATCHES_PER_EPOCH,
            "global_step": MAX_STEPS,
            "samples_seen": SAMPLES_SEEN,
            "loss_config": LOSS_CONFIG.to_dict(),
        },
        "sampling": dict(schedule),
        "last_training_metrics": dict(last_metrics),
        **DENIALS,
    }


def fit_arm(
    arm: str,
    loaded: Any,
    model_config: ModelConfig,
    initial_state: Mapping[str, Tensor],
    initial_sha256: str,
    output: Path,
    device: torch.device,
) -> dict[str, Any]:
    _seed_everything(SEED, device)
    dataset = Common10TorchDataset(
        loaded.examples, model_config, verify_image_sha256=True, split="train"
    )
    model = PerspectiveTrajectoryModel(model_config).to(device)
    model.load_state_dict(initial_state, strict=True)
    require(tensor_digest(model.state_dict()) == initial_sha256, "paired initial state differs")
    require(parameter_count(model) == EXPECTED_PARAMETER_COUNT, "trainable parameter count differs")
    require(all(parameter.requires_grad for parameter in model.parameters()), "full model is not trainable")
    optimizer = AdamW(
        model.parameters(), lr=LEARNING_RATE, weight_decay=WEIGHT_DECAY
    )
    started = time.perf_counter()
    last_metrics: dict[str, Any] = {}
    global_step = 0
    model.train()
    history_path = output / arm / "training/epoch_history.jsonl"
    history_path.parent.mkdir(parents=True, exist_ok=False)
    with history_path.open("x", encoding="utf-8") as history:
        for epoch in range(LOGICAL_EPOCHS):
            batches, proof = build_epoch_batches(dataset.examples, arm, epoch)
            loader = DataLoader(
                dataset,
                batch_sampler=batches,
                num_workers=0,
                pin_memory=False,
            )
            epoch_loss = 0.0
            for batch_index, raw_batch in enumerate(loader):
                batch = _move_batch(raw_batch, device)
                optimizer.zero_grad(set_to_none=True)
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
                    LOSS_CONFIG,
                    target_yaw=batch["target_yaw_rad"],
                )
                total_loss = losses["loss"]
                require(bool(torch.isfinite(total_loss).item()), "training loss is nonfinite")
                total_loss.backward()
                gradient_norm = clip_grad_norm_(model.parameters(), MAXIMUM_GRADIENT_NORM)
                require(bool(torch.isfinite(gradient_norm).item()), "gradient norm is nonfinite")
                optimizer.step()
                require(_nested_tensors_are_finite(model.state_dict()), "model state is nonfinite")
                require(_nested_tensors_are_finite(optimizer.state_dict()), "optimizer state is nonfinite")
                global_step += 1
                epoch_loss += float(total_loss.detach().cpu().item())
                last_metrics = {
                    "global_step": global_step,
                    "logical_epoch": epoch,
                    "batch_index": batch_index,
                    "loss": float(total_loss.detach().cpu().item()),
                    "regression_loss": float(losses["regression_loss"].detach().cpu().item()),
                    "candidate_score_loss": float(losses["candidate_score_loss"].detach().cpu().item()),
                    "candidate_regret_loss": float(losses["candidate_regret_loss"].detach().cpu().item()),
                    "gradient_norm": float(gradient_norm.detach().cpu().item()),
                }
                if global_step % 250 == 0:
                    print(
                        json.dumps(
                            {
                                "event": "K_TRAIN_PROGRESS",
                                "arm": arm,
                                "global_step": global_step,
                                "samples_seen": global_step * BATCH_SIZE,
                            },
                            sort_keys=True,
                        ),
                        flush=True,
                    )
            row = {
                "arm": arm,
                "logical_epoch": epoch,
                "completed_steps": global_step,
                "mean_training_loss": epoch_loss / BATCHES_PER_EPOCH,
                "schedule_sha256": proof["token_batch_schedule_sha256"],
                "dev_observed": False,
            }
            history.write(json.dumps(row, sort_keys=True, allow_nan=False) + "\n")
            history.flush()
    require(global_step == MAX_STEPS, "final optimizer step count differs")
    checkpoint = output / arm / "training/final_checkpoint.pt"
    payload = _checkpoint_payload(
        arm,
        model,
        model_config,
        initial_sha256,
        dataset,
        schedule_contract(dataset.examples, arm),
        last_metrics,
    )
    save_new(checkpoint, payload)
    report = {
        "schema": REPORT_SCHEMA,
        "status": "FINAL_CHECKPOINT_COMPLETE_NOT_EVALUATED",
        "arm": arm,
        "completed_at_utc": utc_now(),
        "checkpoint": str(checkpoint.relative_to(output)),
        "checkpoint_sha256": digest(checkpoint),
        "initial_model_state_sha256": initial_sha256,
        "final_model_state_sha256": payload["final_model_state_sha256"],
        "global_step": global_step,
        "samples_seen": global_step * BATCH_SIZE,
        "wall_seconds": time.perf_counter() - started,
        "last_training_metrics": last_metrics,
        "dev_evaluation_count_during_fit": 0,
        **DENIALS,
    }
    write_new(output / arm / "training/run.json", report)
    del optimizer, model
    torch.cuda.empty_cache()
    return report


def _metric_report(sums: Mapping[str, float], counts: Mapping[str, int]) -> dict[str, Any]:
    metrics = {key: sums[key] / counts[key] for key in sorted(sums)}
    return {"metrics": metrics, "metric_counts": dict(sorted(counts.items()))}


def evaluate_final(
    arm: str,
    checkpoint_path: Path,
    dev_loaded: Any,
    model_config: ModelConfig,
    output: Path,
    device: torch.device,
) -> dict[str, Any]:
    payload = torch.load(regular(checkpoint_path), map_location="cpu")
    require(payload.get("checkpoint_schema") == CHECKPOINT_SCHEMA, "K checkpoint schema differs")
    require(payload.get("arm") == arm, "K checkpoint arm differs")
    require(payload["fit"]["global_step"] == MAX_STEPS, "intermediate checkpoint rejected")
    require(payload["fit"]["optimizer_state_included"] is False, "optimizer state was persisted")
    model = PerspectiveTrajectoryModel(model_config).to(device)
    model.load_state_dict(payload["model_state_dict"], strict=True)
    require(tensor_digest(model.state_dict()) == payload["final_model_state_sha256"], "final model state differs")
    model.requires_grad_(False)
    model.eval()
    dataset = Common10TorchDataset(
        dev_loaded.examples, model_config, verify_image_sha256=True, split="val"
    )
    require(episode_inventory(dataset.examples) == DEV_EPISODE_COUNTS, "final DEV differs")
    sums = {name: {} for name in ("town03_right_turn_dev", "town03_straight_dev", "combined_val")}
    counts = {name: {} for name in sums}
    gate_config = RuntimeGateConfig(
        candidate_count=model_config.candidate_count,
        future_points=model_config.future_points,
    )
    geometry = _AuditAccumulator(gate_config)
    loader = DataLoader(dataset, batch_size=BATCH_SIZE, shuffle=False, num_workers=0, pin_memory=False)
    right_id, straight_id = tuple(DEV_EPISODE_COUNTS)
    with torch.no_grad():
        for raw_batch in loader:
            batch = _move_batch(raw_batch, device)
            candidate_xy, candidate_speed, candidate_logits = model(
                batch["images"],
                batch["calibration"],
                batch["ego_history"],
                batch["ego_history_mask"],
                batch["route_xy"],
                batch["route_mask"],
            )
            output_tensors = {
                "candidate_xy": candidate_xy,
                "candidate_speed": candidate_speed,
                "candidate_logits": candidate_logits,
                "target_xy": batch["target_xy"],
                "target_speed": batch["target_speed_mps"],
                "target_valid": batch["target_valid"],
                "target_yaw": batch["target_yaw_rad"],
            }
            _accumulate_batch_metrics(
                sums["combined_val"], counts["combined_val"], loss_config=LOSS_CONFIG, **output_tensors
            )
            episode_rows = list(raw_batch["episode_id"])
            for group, episode_id in (
                ("town03_right_turn_dev", right_id),
                ("town03_straight_dev", straight_id),
            ):
                positions = [index for index, value in enumerate(episode_rows) if value == episode_id]
                if positions:
                    selected = torch.tensor(positions, dtype=torch.long, device=device)
                    subset = {key: value.index_select(0, selected) for key, value in output_tensors.items()}
                    _accumulate_batch_metrics(
                        sums[group], counts[group], loss_config=LOSS_CONFIG, **subset
                    )
            xy_rows = candidate_xy.detach().cpu().tolist()
            speed_rows = candidate_speed.detach().cpu().tolist()
            logit_rows = candidate_logits.detach().cpu().tolist()
            current_speed_rows = (
                batch["ego_history"][:, -1, FEATURE_NAMES.index("velocity_x_mps")]
                .detach()
                .cpu()
                .tolist()
            )
            for xy, speed, logits, current_speed in zip(
                xy_rows, speed_rows, logit_rows, current_speed_rows
            ):
                geometry.add(
                    audit_prediction(
                        xy,
                        speed,
                        logits,
                        gate_config,
                        current_speed_mps=float(current_speed),
                    )
                )
    reports = {name: _metric_report(sums[name], counts[name]) for name in sums}
    require(reports["town03_right_turn_dev"]["metric_counts"]["selected_ade_m"] == 337, "right DEV count differs")
    require(reports["town03_straight_dev"]["metric_counts"]["selected_ade_m"] == 328, "straight DEV count differs")
    require(reports["combined_val"]["metric_counts"]["selected_ade_m"] == 665, "combined DEV count differs")
    report = {
        "schema": REPORT_SCHEMA,
        "status": "FINAL_CHECKPOINT_DEV_EVALUATION_COMPLETE_NOT_PROMOTED",
        "arm": arm,
        "checkpoint_sha256": digest(checkpoint_path),
        "initial_model_state_sha256": payload["initial_model_state_sha256"],
        "final_model_state_sha256": payload["final_model_state_sha256"],
        "evaluation_split": "val",
        "evaluation_pass_index": 1,
        "sample_count": len(dataset),
        "subgroups": reports,
        "geometry": geometry.report(),
        **DENIALS,
    }
    write_new(output / arm / "final_dev_evaluation.json", report)
    del model
    torch.cuda.empty_cache()
    return report


def strict_gate(report: Mapping[str, Any]) -> dict[str, Any]:
    thresholds = {
        "town03_right_turn_dev": RIGHT_MAXIMUM,
        "town03_straight_dev": STRAIGHT_MAXIMUM,
        "combined_val": COMBINED_MAXIMUM,
    }
    metric_checks = []
    for group, maximums in thresholds.items():
        metrics = report["subgroups"][group]["metrics"]
        for metric, maximum in maximums.items():
            observed = metrics[metric]
            metric_checks.append(
                {
                    "group": group,
                    "metric": metric,
                    "observed": observed,
                    "maximum": maximum,
                    "pass": observed <= maximum,
                }
            )
    geometry = report["geometry"]
    observed_geometry = {
        "selected_geometry_pass_count": geometry["selected_result"]["geometry_pass_count"],
        "any_candidate_geometry_pass_count": geometry["any_candidate_geometry_pass_count"],
        "all_candidates_geometry_pass_count": geometry["all_candidates_geometry_pass_count"],
    }
    geometry_checks = [
        {
            "metric": metric,
            "observed": observed_geometry[metric],
            "minimum": minimum,
            "pass": observed_geometry[metric] >= minimum,
        }
        for metric, minimum in GEOMETRY_MINIMUM.items()
    ]
    metric_pass_count = sum(row["pass"] for row in metric_checks)
    geometry_pass_count = sum(row["pass"] for row in geometry_checks)
    return {
        "pass": metric_pass_count == 9 and geometry_pass_count == 3,
        "metric_pass_count": metric_pass_count,
        "metric_check_count": 9,
        "geometry_pass_count": geometry_pass_count,
        "geometry_check_count": 3,
        "metric_checks": metric_checks,
        "geometry_checks": geometry_checks,
    }


def paired_nonregression(k0: Mapping[str, Any], k1: Mapping[str, Any]) -> dict[str, Any]:
    checks = []
    for group in ("town03_right_turn_dev", "town03_straight_dev", "combined_val"):
        for metric in ("ade_6p4s_m", "fde_6p4s_m", "speed_mae_6p4s_mps"):
            reference = k0["subgroups"][group]["metrics"][metric]
            observed = k1["subgroups"][group]["metrics"][metric]
            checks.append(
                {
                    "group": group,
                    "metric": metric,
                    "K0": reference,
                    "K1": observed,
                    "delta_K1_minus_K0": observed - reference,
                    "pass": observed <= reference,
                }
            )
    pass_count = sum(row["pass"] for row in checks)
    return {
        "pass": pass_count == 9,
        "pass_count": pass_count,
        "check_count": 9,
        "checks": checks,
    }


def paired_execution(
    train_callable: Callable[[str], Any], evaluate_callable: Callable[[str, Any], Any]
) -> tuple[dict[str, Any], dict[str, Any]]:
    trained = {arm: train_callable(arm) for arm in ARMS}
    evaluated = {arm: evaluate_callable(arm, trained[arm]) for arm in ARMS}
    return trained, evaluated


@leased_gpu0
def run(plan_path: Path) -> dict[str, Any]:
    plan = read_json(plan_path)
    validate_plan(plan)
    verify_sources(plan)
    output = Path(plan["output_root"])
    require(not output.exists(), "create-only K output already exists")
    gpu = preflight_gpu()
    k0_loaded, k1_loaded, dev_bundle = verify_views(plan)
    repo = Path(plan["repo_root"])
    model_config = ModelConfig.from_mapping(read_json(repo / MODEL_RELATIVE))
    output.mkdir(parents=True, exist_ok=False)
    copy_new(plan_path, output / "plan.json")
    for name in SOURCE_RELATIVES:
        copy_new(repo / name, output / "source" / name)
    write_new(
        output / "started.json",
        {
            "schema": REPORT_SCHEMA,
            "status": "K_FRESH_FIT_STARTED",
            "started_at_utc": utc_now(),
            "plan_sha256": digest(output / "plan.json"),
            "gpu": gpu,
            **DENIALS,
        },
    )
    device = torch.device("cuda:0")
    _seed_everything(SEED, device)
    initial_model = PerspectiveTrajectoryModel(model_config)
    require(parameter_count(initial_model) == EXPECTED_PARAMETER_COUNT, "initial model capacity differs")
    initial_state = _clone_state(initial_model.state_dict())
    initial_sha256 = tensor_digest(initial_state)
    require(
        initial_sha256 == plan["model"]["seeded_initial_model_state_sha256"],
        "seeded initial model state differs from the sealed plan",
    )
    del initial_model
    loaded_by_arm = {ARMS[0]: k0_loaded, ARMS[1]: k1_loaded}

    def train_one(arm: str) -> Mapping[str, Any]:
        return fit_arm(
            arm,
            loaded_by_arm[arm],
            model_config,
            _clone_state(initial_state),
            initial_sha256,
            output,
            device,
        )

    def evaluate_one(arm: str, training: Mapping[str, Any]) -> Mapping[str, Any]:
        require(training["initial_model_state_sha256"] == initial_sha256, "paired initialization differs")
        return evaluate_final(
            arm,
            output / training["checkpoint"],
            dev_bundle["loaded"],
            model_config,
            output,
            device,
        )

    try:
        training, evaluations = paired_execution(train_one, evaluate_one)
        initial_hashes = {training[arm]["initial_model_state_sha256"] for arm in ARMS}
        require(initial_hashes == {initial_sha256}, "K arms did not share an initial model state")
        strict = {arm: strict_gate(evaluations[arm]) for arm in ARMS}
        nonregression = paired_nonregression(evaluations[ARMS[0]], evaluations[ARMS[1]])
        passed = strict[ARMS[1]]["pass"] and strict[ARMS[0]]["geometry_pass_count"] == 3 and nonregression["pass"]
        decision = {
            "schema": REPORT_SCHEMA,
            "status": (
                "PASS_K_RECOVERY_SIGNAL_EXPLORATORY_NOT_PROMOTED"
                if passed
                else "FAIL_K_RECOVERY_SIGNAL_NOT_PROMOTED"
            ),
            "pass": passed,
            "same_initial_model_state_sha256": initial_sha256,
            "strict_gates": strict,
            "K1_vs_K0_nonregression": nonregression,
            "promotion_eligible": False,
            "reason": "Town03 DEV is repeatedly observed and TEST, stationary, export, and closed-loop work are prohibited in K.",
            **DENIALS,
        }
        write_new(output / "decision.json", decision)
        summary = {
            "schema": REPORT_SCHEMA,
            "status": "COMPLETE_NOT_PROMOTED",
            "completed_at_utc": utc_now(),
            "plan_sha256": digest(output / "plan.json"),
            "training": training,
            "evaluations": {
                arm: {
                    "path": f"{arm}/final_dev_evaluation.json",
                    "sha256": digest(output / arm / "final_dev_evaluation.json"),
                }
                for arm in ARMS
            },
            "decision_sha256": digest(output / "decision.json"),
            **DENIALS,
        }
        write_new(output / "summary.json", summary)
        return summary
    except Exception as error:
        failed = output / "failed.json"
        if not failed.exists():
            write_new(
                failed,
                {
                    "schema": REPORT_SCHEMA,
                    "status": "FAILED_RETAINED_NO_RETRY",
                    "failed_at_utc": utc_now(),
                    "error_type": type(error).__name__,
                    "error": str(error),
                    **DENIALS,
                },
            )
        raise


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Declare or run the sealed K0/K1 performance-recovery fresh fits."
    )
    commands = parser.add_subparsers(dest="command", required=True)
    preflight = commands.add_parser("preflight")
    preflight.add_argument("--repo-root", type=Path, required=True)
    preflight.add_argument("--k0-root", type=Path, required=True)
    preflight.add_argument("--k1-root", type=Path, required=True)
    preflight.add_argument("--dev-root", type=Path, required=True)
    preflight.add_argument("--output-root", type=Path, required=True)
    preflight.add_argument("--plan", type=Path, required=True)
    execute = commands.add_parser("run")
    execute.add_argument("--plan", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == "preflight":
            plan = build_plan(
                args.repo_root,
                args.k0_root,
                args.k1_root,
                args.dev_root,
                args.output_root,
            )
            write_new(args.plan, plan)
            print(
                json.dumps(
                    {
                        "status": "K_PLAN_DECLARED_NO_TRAINING",
                        "plan": str(args.plan.absolute()),
                        "plan_sha256": digest(args.plan),
                    },
                    sort_keys=True,
                )
            )
        else:
            result = run(args.plan)
            print(json.dumps({"status": result["status"]}, sort_keys=True))
    except (ContractError, OSError, RuntimeError, TypeError, ValueError) as error:
        print(f"K recovery error: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
