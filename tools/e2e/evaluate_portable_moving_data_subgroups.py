#!/usr/bin/env python3
"""HH_260906 - Evaluate the two pinned moving-data VAL episodes separately."""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import importlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
from types import SimpleNamespace
from typing import Any, Mapping, Sequence


PLAN_SCHEMA = "portable_e2e.moving_data_ab_seed_pilot_plan.v1"
SUMMARY_SCHEMA = "portable_e2e.moving_data_ab_subgroup_evaluation.v1"
SOURCE_COMMIT = "b478f02e42b94bf04bffec5c8170e05edc33b0f8"
GPU_UUID = "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
ARMS = ("A_baseline_train", "B_town07_augmented_train")
ARM_COUNTS = {
    "A_baseline_train": {"train": 1147, "val": 665},
    "B_town07_augmented_train": {"train": 1475, "val": 665},
}
EPISODES = (
    (
        "town03_right_turn_dev",
        "carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9",
        337,
    ),
    (
        "town03_straight_dev",
        "carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320",
        328,
    ),
)
CORE_SOURCE_PATHS = (
    "portable_e2e/__init__.py",
    "portable_e2e/contract.py",
    "portable_e2e/dataset.py",
    "portable_e2e/torch_dataset.py",
    "portable_e2e/model.py",
    "portable_e2e/losses.py",
    "portable_e2e/train.py",
    "portable_e2e/evaluate.py",
    "portable_e2e/visualize.py",
    "portable_e2e/config/common_10hz_v1.contract.json",
)
DENIALS = {
    "test_read_or_use": False,
    "training_executed_by_evaluator": False,
    "closed_loop_evaluation": False,
    "vehicle_control_approved": False,
    "automatic_promotion": False,
    "model_exported": False,
}


class EvaluationError(ValueError):
    pass


@dataclass(frozen=True)
class EpisodeSpec:
    label: str
    episode_id: str
    count: int


@dataclass(frozen=True)
class DatasetSpec:
    path: str
    manifest_sha256: str
    corpus_fingerprint_sha256: str
    train_fingerprint_sha256: str
    val_fingerprint_sha256: str
    split_examples_sha256: Mapping[str, str]
    counts: Mapping[str, int]
    episode_ids: Mapping[str, tuple[str, ...]]


@dataclass(frozen=True)
class PlanSpec:
    raw: Mapping[str, Any]
    schema: str
    source_commit: str
    gpu_uuid: str
    evaluator_sha256: str
    source_sha256: Mapping[str, str]
    arms: Mapping[str, DatasetSpec]
    validation_count: int
    validation_fingerprint_sha256: str
    validation_episodes: tuple[EpisodeSpec, ...]
    output_path: str
    workspace_root: str | None


def require(value: Any, message: str) -> None:
    if not value:
        raise EvaluationError(message)


def _mapping(value: Any, context: str) -> Mapping[str, Any]:
    require(isinstance(value, dict), f"{context} must be an object")
    return value


def _sequence(value: Any, context: str) -> Sequence[Any]:
    require(isinstance(value, list), f"{context} must be an array")
    return value


def _string(value: Any, context: str) -> str:
    require(isinstance(value, str) and value, f"{context} must be a nonempty string")
    return value


def _integer(value: Any, context: str, minimum: int = 0) -> int:
    require(
        not isinstance(value, bool) and isinstance(value, int) and value >= minimum,
        f"{context} must be an integer >= {minimum}",
    )
    return value


def _sha256(value: Any, context: str) -> str:
    text = _string(value, context)
    require(re.fullmatch(r"[0-9a-f]{64}", text) is not None, f"{context} must be SHA-256")
    return text


def _safe_relative(value: Any, context: str) -> str:
    text = _string(value, context)
    path = Path(text)
    require(
        not path.is_absolute()
        and ".." not in path.parts
        and "\\" not in text
        and path.as_posix() == text,
        f"{context} must be a safe POSIX relative path",
    )
    return text


def _json_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        require(key not in result, f"duplicate JSON key: {key!r}")
        result[key] = value
    return result


def _read_json(path: Path) -> Mapping[str, Any]:
    try:
        return _mapping(
            json.loads(
                _regular_file(path, "JSON input").read_text(encoding="utf-8"),
                object_pairs_hook=_json_pairs,
                parse_constant=lambda value: require(False, f"nonfinite JSON value: {value}"),
            ),
            str(path),
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise EvaluationError(f"cannot read strict JSON {path}: {error}") from error


def _encoded(value: Mapping[str, Any]) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()


def _path_without_symlinks(path: Path, context: str, *, must_exist: bool) -> Path:
    lexical = Path(path).expanduser().absolute()
    require(".." not in lexical.parts, f"{context} must not contain traversal")
    if must_exist:
        require(lexical.exists(), f"{context} does not exist: {lexical}")
    current = lexical if lexical.exists() else lexical.parent
    require(
        not any(item.is_symlink() for item in (current, *current.parents)),
        f"{context} must not traverse a symbolic link",
    )
    resolved = lexical.resolve(strict=must_exist)
    require(resolved == lexical, f"{context} must be canonical")
    return resolved


def _regular_file(path: Path, context: str) -> Path:
    resolved = _path_without_symlinks(path, context, must_exist=True)
    require(resolved.is_file(), f"{context} must be a regular file: {resolved}")
    return resolved


def _directory(path: Path, context: str) -> Path:
    resolved = _path_without_symlinks(path, context, must_exist=True)
    require(resolved.is_dir(), f"{context} must be a directory: {resolved}")
    return resolved


def _fresh_output(path: Path) -> Path:
    lexical = Path(path).expanduser().absolute()
    require(".." not in lexical.parts and not lexical.exists(), "fresh output directory required")
    parent = _directory(lexical.parent, "output parent")
    resolved = parent / lexical.name
    require(not resolved.exists() and not resolved.is_symlink(), "fresh nonsymlink output required")
    return resolved


def digest(path: Path) -> str:
    result = hashlib.sha256()
    with _regular_file(path, "hash input").open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def _dataset_spec(raw: Any, arm: str) -> DatasetSpec:
    value = _mapping(raw, f"plan.arms.{arm}.dataset")
    required = {
        "path",
        "manifest_sha256",
        "corpus_fingerprint_sha256",
        "split_fingerprint_sha256",
        "counts",
        "episode_ids",
    }
    require(required.issubset(value), f"plan.arms.{arm}.dataset fields are incomplete")
    split_fingerprints = _mapping(
        value["split_fingerprint_sha256"], f"plan.arms.{arm}.dataset.split_fingerprint_sha256"
    )
    counts = _mapping(value["counts"], f"plan.arms.{arm}.dataset.counts")
    episode_ids = _mapping(value["episode_ids"], f"plan.arms.{arm}.dataset.episode_ids")
    examples_fingerprints = _mapping(
        value.get("split_examples_sha256"), f"plan.arms.{arm}.dataset.split_examples_sha256"
    )
    require(set(split_fingerprints) == {"train", "val"}, f"{arm} split fingerprints differ")
    require(set(counts) == {"train", "val"}, f"{arm} split counts differ")
    require(set(episode_ids) == {"train", "val"}, f"{arm} episode ID splits differ")
    require(set(examples_fingerprints) == {"train", "val"}, f"{arm} example fingerprints differ")
    parsed_counts = {
        split: _integer(counts[split], f"{arm}.{split} count", 1) for split in ("train", "val")
    }
    require(parsed_counts == ARM_COUNTS[arm], f"{arm} exact split counts differ")
    parsed_ids: dict[str, tuple[str, ...]] = {}
    for split in ("train", "val"):
        ids = tuple(
            _string(item, f"{arm}.{split} episode ID")
            for item in _sequence(episode_ids[split], f"{arm}.{split} episode IDs")
        )
        require(ids and len(ids) == len(set(ids)), f"{arm}.{split} episode IDs must be unique")
        parsed_ids[split] = ids
    require(len(parsed_ids["train"]) == (3 if arm == ARMS[0] else 4), f"{arm} TRAIN episodes differ")
    require(len(parsed_ids["val"]) == 2, f"{arm} VAL episodes differ")
    path = _string(value["path"], f"plan.arms.{arm}.dataset.path")
    if not Path(path).is_absolute():
        _safe_relative(path, f"plan.arms.{arm}.dataset.path")
    return DatasetSpec(
        path=path,
        manifest_sha256=_sha256(value["manifest_sha256"], f"{arm} manifest SHA-256"),
        corpus_fingerprint_sha256=_sha256(
            value["corpus_fingerprint_sha256"], f"{arm} corpus fingerprint"
        ),
        train_fingerprint_sha256=_sha256(split_fingerprints["train"], f"{arm} TRAIN fingerprint"),
        val_fingerprint_sha256=_sha256(split_fingerprints["val"], f"{arm} VAL fingerprint"),
        split_examples_sha256={
            split: _sha256(examples_fingerprints[split], f"{arm} {split} example fingerprint")
            for split in ("train", "val")
        },
        counts=parsed_counts,
        episode_ids=parsed_ids,
    )


def parse_plan(raw: Mapping[str, Any]) -> PlanSpec:
    """HH_260906 - Validate only the evaluator's strict, documented plan projection."""
    schema = _string(raw.get("schema"), "plan.schema")
    require(schema == PLAN_SCHEMA, "campaign plan schema differs")
    source_commit = _string(raw.get("source_commit"), "plan.source_commit")
    require(source_commit == SOURCE_COMMIT, "campaign source commit differs")
    gpu = _mapping(raw.get("gpu"), "plan.gpu")
    gpu_uuid = _string(gpu.get("uuid"), "plan.gpu.uuid")
    require(gpu_uuid == GPU_UUID, "campaign GPU UUID differs")
    evaluator_sha256 = _sha256(raw.get("evaluator_sha256"), "plan.evaluator_sha256")
    source_values = _mapping(raw.get("source_sha256"), "plan.source_sha256")
    source_sha256: dict[str, str] = {}
    for name, expected in source_values.items():
        safe = _safe_relative(name, "plan.source_sha256 key")
        source_sha256[safe] = _sha256(expected, f"plan.source_sha256[{safe!r}]")
    require(
        set(CORE_SOURCE_PATHS).issubset(source_sha256),
        "campaign plan omits required evaluator core source pins",
    )
    arm_values = _mapping(raw.get("arms"), "plan.arms")
    require(set(arm_values) == set(ARMS), "campaign plan arms differ")
    arms = {
        arm: _dataset_spec(_mapping(arm_values[arm], f"plan.arms.{arm}").get("dataset"), arm)
        for arm in ARMS
    }
    validation = _mapping(raw.get("validation"), "plan.validation")
    require(
        {"count", "fingerprint_sha256", "episodes"}.issubset(validation),
        "campaign validation fields are incomplete",
    )
    validation_count = _integer(validation["count"], "plan.validation.count", 1)
    require(validation_count == 665, "combined VAL count differs")
    validation_fingerprint = _sha256(
        validation["fingerprint_sha256"], "plan.validation.fingerprint_sha256"
    )
    episode_values = _sequence(validation["episodes"], "plan.validation.episodes")
    require(len(episode_values) == len(EPISODES), "combined VAL episode count differs")
    parsed_episodes: list[EpisodeSpec] = []
    for index, (value, expected) in enumerate(zip(episode_values, EPISODES)):
        item = _mapping(value, f"plan.validation.episodes[{index}]")
        require(set(item) == {"label", "episode_id", "count"}, "VAL episode fields differ")
        actual = EpisodeSpec(
            label=_string(item["label"], f"VAL episode {index} label"),
            episode_id=_string(item["episode_id"], f"VAL episode {index} ID"),
            count=_integer(item["count"], f"VAL episode {index} count", 1),
        )
        require(
            (actual.label, actual.episode_id, actual.count) == expected,
            f"VAL episode {index} identity/count/order differs",
        )
        parsed_episodes.append(actual)
    expected_val_ids = tuple(item.episode_id for item in parsed_episodes)
    for arm, item in arms.items():
        require(item.episode_ids["val"] == expected_val_ids, f"{arm} VAL episode order differs")
        require(item.val_fingerprint_sha256 == arms[ARMS[0]].val_fingerprint_sha256,
                "arms do not share one full VAL tensor fingerprint")
        require(
            item.split_examples_sha256["val"] == validation_fingerprint,
            f"{arm} shared VAL example fingerprint differs",
        )
    output_path = _safe_relative(raw.get("output"), "plan.output")
    workspace_root = raw.get("workspace_root")
    if workspace_root is not None:
        workspace_root = _string(workspace_root, "plan.workspace_root")
        require(Path(workspace_root).is_absolute(), "plan.workspace_root must be absolute")
    require(raw.get("test_read_or_use", False) is False, "campaign plan permits TEST read/use")
    return PlanSpec(
        raw=raw,
        schema=schema,
        source_commit=source_commit,
        gpu_uuid=gpu_uuid,
        evaluator_sha256=evaluator_sha256,
        source_sha256=source_sha256,
        arms=arms,
        validation_count=validation_count,
        validation_fingerprint_sha256=validation_fingerprint,
        validation_episodes=tuple(parsed_episodes),
        output_path=output_path,
        workspace_root=workspace_root,
    )


def _git(repo_root: Path, *arguments: str) -> bytes:
    environment = dict(
        os.environ,
        GIT_NO_LAZY_FETCH="1",
        GIT_ALLOW_PROTOCOL="",
        GIT_TERMINAL_PROMPT="0",
    )
    try:
        return subprocess.check_output(
            ["git", "-c", "protocol.allow=never", "-C", str(repo_root), *arguments],
            env=environment,
            timeout=20,
            stderr=subprocess.STDOUT,
        )
    except (OSError, subprocess.SubprocessError) as error:
        raise EvaluationError(f"cannot verify pinned repository source: {error}") from error


def verify_repo_sources(plan: PlanSpec, repo_root: Path) -> None:
    """HH_260906 - Dirty unrelated files are allowed; every imported core byte is pinned."""
    head = _git(repo_root, "rev-parse", "HEAD").decode("ascii").strip()
    require(head == plan.source_commit, "repository HEAD differs from campaign source commit")
    for name, expected in plan.source_sha256.items():
        path = _regular_file(repo_root / name, f"pinned source {name}")
        require(path.is_relative_to(repo_root), f"pinned source escapes repo: {name}")
        require(digest(path) == expected, f"working source SHA-256 differs: {name}")
        committed = _git(repo_root, "show", f"{plan.source_commit}:{name}")
        require(hashlib.sha256(committed).hexdigest() == expected, f"commit source SHA-256 differs: {name}")


def load_core(repo_root: Path) -> SimpleNamespace:
    """HH_260906 - Import only after the verified clean-source root wins sys.path."""
    text = str(repo_root)
    sys.path[:] = [item for item in sys.path if item != text]
    sys.path.insert(0, text)
    importlib.invalidate_caches()
    contract = importlib.import_module("portable_e2e.contract")
    dataset = importlib.import_module("portable_e2e.dataset")
    evaluate = importlib.import_module("portable_e2e.evaluate")
    model = importlib.import_module("portable_e2e.model")
    torch_dataset = importlib.import_module("portable_e2e.torch_dataset")
    train = importlib.import_module("portable_e2e.train")
    for module in (contract, dataset, evaluate, model, torch_dataset, train):
        module_path = Path(module.__file__).resolve(strict=True)
        require(module_path.is_relative_to(repo_root), f"import escaped pinned repo: {module.__name__}")
    return SimpleNamespace(
        ContractError=contract.ContractError,
        load_training_examples=dataset.load_training_examples,
        fingerprint_examples=dataset.fingerprint_examples,
        evaluate_model=evaluate.evaluate_model,
        ModelConfig=model.ModelConfig,
        Common10TorchDataset=torch_dataset.Common10TorchDataset,
        CHECKPOINT_ID=train.CHECKPOINT_ID,
        read_checkpoint_file=train._read_checkpoint_file,
        select_device=train._select_device,
        canonical_sha256=train._canonical_sha256,
    )


def _workspace_from_plan(plan_path: Path, plan: PlanSpec) -> Path:
    if plan.workspace_root is not None:
        workspace = _directory(Path(plan.workspace_root), "plan workspace root")
    else:
        workspace = plan_path.parent
        for _ in Path(plan.output_path).parts:
            workspace = workspace.parent
        workspace = _directory(workspace, "workspace inferred from plan output")
        require(workspace / plan.output_path == plan_path.parent, "plan path does not match planned output root")
    return workspace


def _resolve_dataset_path(spec: DatasetSpec, plan: PlanSpec, plan_path: Path) -> Path:
    value = Path(spec.path).expanduser()
    if not value.is_absolute():
        relative = Path(_safe_relative(spec.path, "dataset path"))
        workspace = _workspace_from_plan(plan_path, plan)
        value = workspace / relative
        datasets = workspace / "datasets"
        if datasets.is_symlink():
            require(datasets.resolve() == workspace.parent / "dataset", "unapproved datasets alias")
            current = datasets
            for part in relative.parts[1:]:
                current /= part
                require(not current.is_symlink(), "symlink below approved datasets alias")
            resolved = value.resolve(strict=True)
            prepared = (workspace.parent / "dataset/prepared").resolve(strict=True)
            require(resolved.is_relative_to(prepared), "dataset escaped approved prepared storage")
            return _directory(resolved, "arm self-contained dataset")
    return _directory(value, "arm self-contained dataset")


def _safe_child_file(root: Path, value: Any, context: str) -> Path:
    relative = Path(_safe_relative(value, context))
    candidate = _regular_file(root / relative, context)
    require(candidate.is_relative_to(root), f"{context} escaped dataset root")
    return candidate


def preflight_view_metadata(root: Path, spec: DatasetSpec) -> Mapping[Path, str]:
    """HH_260906 - Reject TEST from metadata before the Common10 loader can open payloads."""
    manifest_path = _regular_file(root / "dataset.json", "dataset manifest")
    manifest_hash = digest(manifest_path)
    require(manifest_hash == spec.manifest_sha256, "dataset manifest SHA-256 differs from plan")
    manifest = _read_json(manifest_path)
    references = _sequence(manifest.get("episodes"), "dataset.episodes")
    require(references, "dataset has no episodes")
    observed_ids: dict[str, list[str]] = {"train": [], "val": []}
    observed_counts = {"train": 0, "val": 0}
    hashes: dict[Path, str] = {manifest_path: manifest_hash}
    seen: set[str] = set()
    for index, raw_reference in enumerate(references):
        reference = _mapping(raw_reference, f"dataset.episodes[{index}]")
        episode_id = _string(reference.get("episode_id"), f"dataset.episodes[{index}].episode_id")
        require(episode_id not in seen, "duplicate episode ID in dataset manifest")
        expected_path = f"episodes/{episode_id}/episode.json"
        require(reference.get("manifest") == expected_path, "episode metadata path/layout differs")
        episode_path = _safe_child_file(root, expected_path, "episode manifest")
        episode_hash = digest(episode_path)
        require(
            episode_hash == _sha256(reference.get("sha256"), "episode reference SHA-256"),
            "episode manifest SHA-256 differs",
        )
        episode = _read_json(episode_path)
        require(episode.get("episode_id") == episode_id, "episode identity differs")
        split = episode.get("split")
        require(split in observed_counts, "TEST/unknown split forbidden before payload loading")
        observed_ids[split].append(episode_id)
        observed_counts[split] += _integer(episode.get("sample_count"), "episode sample count", 1)
        hashes[episode_path] = episode_hash
        seen.add(episode_id)
    require(observed_counts == dict(spec.counts), "dataset metadata split counts differ from plan")
    require(
        {key: tuple(value) for key, value in observed_ids.items()} == dict(spec.episode_ids),
        "dataset metadata episode identity/order differs from plan",
    )
    return hashes


def _verify_metadata_unchanged(hashes: Mapping[Path, str]) -> None:
    for path, expected in hashes.items():
        require(digest(path) == expected, f"dataset metadata changed during evaluation: {path}")


def _checkpoint_model_config(core: SimpleNamespace, checkpoint: Path, expected: str, spec: DatasetSpec):
    device = core.select_device("cuda:0")
    payload, observed = core.read_checkpoint_file(checkpoint, device)
    require(observed == expected, "secure checkpoint read SHA-256 differs")
    require(payload.get("checkpoint_id") == core.CHECKPOINT_ID, "unsupported checkpoint kind")
    require(payload.get("training_split") == "train", "checkpoint was not trained on TRAIN")
    require(
        payload.get("dataset_fingerprint_sha256") == spec.train_fingerprint_sha256,
        "checkpoint TRAIN fingerprint differs from selected arm",
    )
    require(
        payload.get("corpus_fingerprint_sha256") == spec.corpus_fingerprint_sha256,
        "checkpoint corpus fingerprint differs from selected arm",
    )
    require(
        payload.get("training_episode_ids") == sorted(spec.episode_ids["train"]),
        "checkpoint TRAIN episode provenance differs from selected arm",
    )
    model_value = _mapping(payload.get("model_config"), "checkpoint.model_config")
    model_config = core.ModelConfig.from_mapping(model_value)
    require(
        payload.get("model_config_sha256") == core.canonical_sha256(model_config.to_dict()),
        "checkpoint model-config fingerprint differs",
    )
    return model_config, payload


def _episode_order(examples: Sequence[Any]) -> tuple[tuple[str, ...], Mapping[str, int]]:
    order: list[str] = []
    counts: dict[str, int] = {}
    for example in examples:
        episode_id = _string(getattr(example, "episode_id", None), "loaded example episode ID")
        if not order or order[-1] != episode_id:
            require(episode_id not in counts, "VAL episodes are interleaved or repeated")
            order.append(episode_id)
            counts[episode_id] = 0
        counts[episode_id] += 1
    return tuple(order), counts


def _write_new_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        stream.write(_encoded(value))
        stream.flush()
        os.fsync(stream.fileno())


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _verify_episode_output(
    output: Path,
    report: Mapping[str, Any],
    *,
    episode: EpisodeSpec,
    checkpoint_sha256: str,
    subset_fingerprint: str,
    corpus_fingerprint: str,
    train_fingerprint: str,
) -> Mapping[str, Any]:
    metrics_path = _regular_file(output / "metrics.json", "episode metrics")
    stored = _read_json(metrics_path)
    require(stored == report, "returned and persisted episode reports differ")
    require(report.get("status") == "OPEN_LOOP_EVALUATION_COMPLETE", "episode evaluation incomplete")
    require(report.get("checkpoint_sha256") == checkpoint_sha256, "episode checkpoint hash differs")
    require(
        report.get("training_dataset_fingerprint_sha256") == train_fingerprint,
        "episode checkpoint TRAIN fingerprint differs",
    )
    require(report.get("dataset_fingerprint_sha256") == subset_fingerprint, "subset fingerprint differs")
    require(report.get("corpus_fingerprint_sha256") == corpus_fingerprint, "full corpus fingerprint differs")
    require(report.get("evaluation_split") == "val", "episode evaluation split differs")
    require(report.get("evaluation_episode_count") == 1, "episode subset contains multiple episodes")
    require(report.get("sample_count") == episode.count, "episode evaluated sample count differs")
    require(report.get("rendered_trajectory_png") == 6, "episode render count differs")
    require(report.get("vehicle_control_approved") is False, "episode report approves vehicle control")
    trajectory_root = _directory(output / "trajectories", "episode trajectory directory")
    pngs = sorted(trajectory_root.glob("*.png"))
    require(len(pngs) == 6, "episode trajectory PNG inventory differs")
    png_sha256: dict[str, str] = {}
    for path in pngs:
        regular = _regular_file(path, "episode trajectory PNG")
        require(regular.parent == trajectory_root, "nested trajectory output forbidden")
        png_sha256[regular.name] = digest(regular)
    return {
        "episode_id": episode.episode_id,
        "count": episode.count,
        "subset_fingerprint_sha256": subset_fingerprint,
        "metrics": report.get("metrics"),
        "metric_counts": report.get("metric_counts"),
        "report_path": f"episodes/{episode.label}/metrics.json",
        "report_sha256": digest(metrics_path),
        "trajectory_png_sha256": png_sha256,
    }


def run(args: argparse.Namespace, *, core: SimpleNamespace | None = None) -> Mapping[str, Any]:
    plan_path = _regular_file(args.plan, "campaign plan")
    plan_sha256 = _sha256(args.plan_sha256, "--plan-sha256")
    require(digest(plan_path) == plan_sha256, "campaign plan SHA-256 differs")
    plan = parse_plan(_read_json(plan_path))
    evaluator_path = _regular_file(Path(__file__), "archived evaluator")
    require(digest(evaluator_path) == plan.evaluator_sha256, "archived evaluator SHA-256 differs")
    arm = _string(args.arm, "--arm")
    require(arm in ARMS, "unknown moving-data A/B arm")
    repo_root = _directory(args.repo_root, "pinned repository root")
    output = _fresh_output(args.output_dir)
    checkpoint = _regular_file(args.checkpoint, "checkpoint")
    checkpoint_sha256 = _sha256(args.checkpoint_sha256, "--checkpoint-sha256")
    require(digest(checkpoint) == checkpoint_sha256, "checkpoint SHA-256 differs")
    require(os.environ.get("CUDA_VISIBLE_DEVICES") == plan.gpu_uuid, "CUDA_VISIBLE_DEVICES must be the exact pinned GPU UUID")
    require(not output.is_relative_to(repo_root), "evaluation output must stay outside the pinned repo")
    verify_repo_sources(plan, repo_root)
    core = core or load_core(repo_root)
    dataset_spec = plan.arms[arm]
    dataset_root = _resolve_dataset_path(dataset_spec, plan, plan_path)
    require(not output.is_relative_to(dataset_root), "evaluation output must stay outside the dataset")
    metadata_hashes = preflight_view_metadata(dataset_root, dataset_spec)
    model_config, checkpoint_payload = _checkpoint_model_config(
        core, checkpoint, checkpoint_sha256, dataset_spec
    )
    loaded = core.load_training_examples(
        dataset_root,
        split="val",
        mode="planning",
        check_image_hashes=True,
    )
    examples = tuple(loaded.examples)
    require(len(examples) == plan.validation_count, "loaded combined VAL count differs")
    require(
        loaded.validation_report.get("dataset_manifest_sha256") == dataset_spec.manifest_sha256,
        "loader dataset manifest fingerprint differs",
    )
    require(
        loaded.validation_report.get("dataset_fingerprint_sha256")
        == dataset_spec.corpus_fingerprint_sha256,
        "loader corpus fingerprint differs",
    )
    require(
        core.fingerprint_examples(examples) == plan.validation_fingerprint_sha256,
        "shared combined VAL example-content fingerprint differs",
    )
    order, counts = _episode_order(examples)
    expected_order = tuple(item.episode_id for item in plan.validation_episodes)
    expected_counts = {item.episode_id: item.count for item in plan.validation_episodes}
    require(order == expected_order and counts == expected_counts, "loaded VAL episode count/order differs")
    full_dataset = core.Common10TorchDataset(
        examples,
        model_config,
        verify_image_sha256=True,
        split="val",
    )
    require(
        full_dataset.fingerprint_sha256 == dataset_spec.val_fingerprint_sha256,
        "full combined VAL tensor fingerprint differs",
    )
    episode_results: dict[str, Mapping[str, Any]] = {}
    for episode in plan.validation_episodes:
        subset_examples = tuple(item for item in examples if item.episode_id == episode.episode_id)
        require(len(subset_examples) == episode.count, f"{episode.label} subset count differs")
        subset = core.Common10TorchDataset(
            subset_examples,
            model_config,
            verify_image_sha256=True,
            split="val",
        )
        child = output / "episodes" / episode.label
        report = core.evaluate_model(
            subset,
            checkpoint_path=checkpoint,
            output_dir=child,
            dataset_fingerprint_sha256=subset.fingerprint_sha256,
            corpus_fingerprint_sha256=dataset_spec.corpus_fingerprint_sha256,
            device_name="cuda:0",
            batch_size=4,
            num_workers=0,
            render_count=6,
            evaluation_split="val",
        )
        episode_results[episode.label] = _verify_episode_output(
            child,
            report,
            episode=episode,
            checkpoint_sha256=checkpoint_sha256,
            subset_fingerprint=subset.fingerprint_sha256,
            corpus_fingerprint=dataset_spec.corpus_fingerprint_sha256,
            train_fingerprint=dataset_spec.train_fingerprint_sha256,
        )
    _verify_metadata_unchanged(metadata_hashes)
    require(digest(checkpoint) == checkpoint_sha256, "checkpoint changed during subgroup evaluation")
    require(digest(plan_path) == plan_sha256, "campaign plan changed during subgroup evaluation")
    require(digest(evaluator_path) == plan.evaluator_sha256, "evaluator changed during subgroup evaluation")
    summary = {
        "schema": SUMMARY_SCHEMA,
        "status": "COMPLETE_NOT_PROMOTED",
        "created_at_utc": _utc_now(),
        "arm": arm,
        "plan_path": str(plan_path),
        "plan_sha256": plan_sha256,
        "source_commit": plan.source_commit,
        "gpu_uuid": plan.gpu_uuid,
        "evaluator_sha256": plan.evaluator_sha256,
        "source_sha256": dict(plan.source_sha256),
        "dataset": {
            "path": str(dataset_root),
            "manifest_sha256": dataset_spec.manifest_sha256,
            "corpus_fingerprint_sha256": dataset_spec.corpus_fingerprint_sha256,
            "train_fingerprint_sha256": dataset_spec.train_fingerprint_sha256,
            "combined_val_fingerprint_sha256": dataset_spec.val_fingerprint_sha256,
            "combined_val_example_fingerprint_sha256": plan.validation_fingerprint_sha256,
            "counts": dict(dataset_spec.counts),
        },
        "checkpoint": {
            "path": str(checkpoint),
            "sha256": checkpoint_sha256,
            "training_split": checkpoint_payload["training_split"],
            "training_episode_ids": checkpoint_payload["training_episode_ids"],
            "training_dataset_fingerprint_sha256": checkpoint_payload[
                "dataset_fingerprint_sha256"
            ],
            "corpus_fingerprint_sha256": checkpoint_payload["corpus_fingerprint_sha256"],
            "model_config_sha256": checkpoint_payload["model_config_sha256"],
        },
        "checkpoint_sha256": checkpoint_sha256,
        "corpus_fingerprint_sha256": dataset_spec.corpus_fingerprint_sha256,
        "combined_val_count": plan.validation_count,
        "episodes": episode_results,
        "interpretation": (
            "Open-loop subgroup metrics isolate the old Town03 right-turn and new Town03 "
            "straight validation episodes under one arm corpus/checkpoint. They do not establish "
            "closed-loop driving, TEST performance, or vehicle readiness."
        ),
        **DENIALS,
    }
    _write_new_json(output / "summary.json", summary)
    require(_read_json(output / "summary.json") == summary, "persisted summary differs")
    return summary


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--plan-sha256", required=True)
    parser.add_argument("--arm", choices=ARMS, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--checkpoint-sha256", required=True)
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        summary = run(args)
    except (EvaluationError, OSError, RuntimeError, subprocess.SubprocessError) as error:
        print(f"MOVING_SUBGROUP_EVALUATION_ERROR: {error}", file=sys.stderr)
        return 2
    print(
        json.dumps(
            {
                "status": summary["status"],
                "arm": summary["arm"],
                "combined_val_count": summary["combined_val_count"],
                "output_dir": str(args.output_dir),
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
