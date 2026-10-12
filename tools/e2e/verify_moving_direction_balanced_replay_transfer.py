#!/usr/bin/env python3
"""HH_260906 - Authenticate the exact successful F transfer without Torch, GPU, or gate recomputation."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import stat
from typing import Any


SCHEMA = "portable_e2e.moving_direction_balanced_replay_transfer_verification.v1"
PLAN_SCHEMA = "portable_e2e.moving_direction_balanced_replay_seed_pilot_plan.v1"
SUMMARY_SCHEMA = "portable_e2e.moving_direction_balanced_replay_seed_pilot_summary.v1"
SUBGROUP_SCHEMA = "portable_e2e.moving_direction_balanced_replay_subgroup_evaluation.v1"
CAMPAIGN_ID = "hh260921-moving-direction-balanced-replay-seed20260906-v1"
BASE_COMMIT = "b478f02e42b94bf04bffec5c8170e05edc33b0f8"
ARM = "F_direction_balanced_replay_train"
STAGES = ("train", "evaluate", "audit", "subgroup")
EXPECTED_TRAIN_STATE = {"epoch": 20, "next_batch_index": 0, "global_step": 8280,
                        "samples_seen": 33120, "domain_samples_seen": {"carla": 33120}}
EXPECTED_STREAM_SAMPLES = {"base": 29500, "replay": 3620}
EXPECTED_SAMPLING_PLAN_SHA256 = "abb0e655b174844cc20f293bc12c5156fe33b3a00741ac1cfe8b703367e41c7b"
EXPECTED_EXECUTION_SCHEDULE_SHA256 = "d340a10c42f2a733988b69ae6571ec3c37b7e8fa1601c43e6111192cd35c55b0"
EXPECTED_EXECUTION_SCHEDULE_ROWS = 33120
EXPECTED_HISTORY_ROWS = 8280
EXPECTED_EPISODES = {
    "town03_right_turn_dev": ("carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9", 337),
    "town03_straight_dev": ("carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320", 328),
}
RUN_DENIALS = ("resume", "automatic_retry", "test_read_or_use", "model_export",
               "automatic_promotion", "training_data_approved", "vehicle_control_approved",
               "closed_loop_claim", "all_map_claim", "deployment_approved")
SUBGROUP_DENIALS = ("test_read_or_use", "training_executed_by_evaluator",
                    "closed_loop_evaluation", "vehicle_control_approved",
                    "automatic_promotion", "model_exported")
SOURCE_BASE_PATHS = tuple("portable_e2e/" + name + ".py" for name in (
    "__init__", "contract", "dataset", "torch_dataset", "model", "losses", "train",
    "evaluate", "audit_runtime", "runtime", "runtime_contract", "runtime_weight_bundle",
    "visualize", "stop_primitive_research")) + (
    "portable_e2e/config/common_10hz_v1.contract.json",
    "portable_e2e/config/perspective_trajectory_physical_stopmix_v1.model.json",
    "scripts/e2e/audit_portable_stopmix_behavior.py",
    "scripts/e2e/run_portable_training_campaign.py",
)
SOURCE_ADDED_PATHS = ("scripts/e2e/evaluate_portable_moving_data_subgroups.py",)
SOURCE_OVERLAYS = ("portable_e2e/evaluate.py", "portable_e2e/train.py")
RUNNER_ARCHIVE = "source/run_moving_direction_balanced_replay_seed_pilot.py"
EVALUATOR_ARCHIVE = "source/evaluate_portable_moving_direction_balanced_replay_subgroups.py"
OWNER_ARCHIVE = "source/profile_portable_training.py"
STAGE_REPORTS = {
    "train": "training/run.json",
    "evaluate": "evaluation/metrics.json",
    "audit": "gate_v8.json",
    "subgroup": "subgroup/summary.json",
}
REFERENCE_C_PATHS = {
    "plan": "plan.json",
    "summary": "summary.json",
    "training_run": "A_baseline_train/training/run.json",
    "subgroup": "A_baseline_train/subgroup/summary.json",
    "gate": "A_baseline_train/gate_v8.json",
}
REFERENCE_D_PATHS = {
    "checkpoint": "B_town07_augmented_train/training/checkpoints/latest.pt",
    "training_run": "B_town07_augmented_train/training/run.json",
    "evaluation": "B_town07_augmented_train/evaluation/metrics.json",
    "subgroup": "B_town07_augmented_train/subgroup/summary.json",
    "gate": "B_town07_augmented_train/gate_v8.json",
}


class VerificationError(ValueError):
    """HH_260906 - Signal a closed transfer-verification failure."""


def require(value: Any, message: str) -> None:
    if not value:
        raise VerificationError(message)


def _pairs(items: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in items:
        require(key not in result, f"duplicate JSON key: {key!r}")
        result[key] = value
    return result


def _sha(value: Any, context: str) -> str:
    require(type(value) is str and re.fullmatch(r"[0-9a-f]{64}", value) is not None,
            f"{context} must be a lowercase SHA-256")
    return value


def _relative(value: Any, context: str) -> str:
    require(type(value) is str and value and "\\" not in value, f"{context} must be a POSIX path")
    path = Path(value)
    require(not path.is_absolute() and ".." not in path.parts and path.as_posix() == value and value != ".",
            f"{context} must stay below its root")
    return value


def _inventory_relative(value: Any, context: str) -> str:
    # HH_260906 - Accept one sha256sum-style './' prefix while keeping one canonical internal key.
    require(type(value) is str, f"{context} must be a POSIX path")
    normalized = value
    if value.startswith("./"):
        normalized = value[2:]
        require(normalized and not normalized.startswith("./"),
                f"{context} permits exactly one './' prefix")
    return _relative(normalized, context)


def _directory(path: Path, context: str) -> Path:
    lexical = Path(path).expanduser().absolute()
    require(".." not in lexical.parts and lexical.exists() and lexical.resolve() == lexical,
            f"{context} must be an existing canonical path")
    mode = lexical.lstat().st_mode
    require(stat.S_ISDIR(mode) and not lexical.is_symlink(), f"{context} must be a nonsymlink directory")
    return lexical


def _regular(path: Path, context: str) -> Path:
    lexical = Path(path).expanduser().absolute()
    require(".." not in lexical.parts and lexical.exists() and lexical.resolve() == lexical,
            f"{context} does not exist or is noncanonical: {lexical}")
    info = lexical.lstat()
    require(stat.S_ISREG(info.st_mode) and not lexical.is_symlink() and info.st_nlink == 1,
            f"{context} must be a single-link regular file")
    return lexical


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with _regular(path, "hash input").open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(
            _regular(path, "JSON input").read_text(encoding="utf-8"),
            object_pairs_hook=_pairs,
            parse_constant=lambda item: require(False, f"nonfinite JSON value: {item}"),
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise VerificationError(f"cannot read strict JSON {path}: {error}") from error
    require(type(value) is dict, f"JSON object required: {path}")
    try:
        json.dumps(value, allow_nan=False)
    except (TypeError, ValueError) as error:
        raise VerificationError(f"nonfinite JSON value in {path}") from error
    return value


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                 allow_nan=False).encode()).hexdigest()


def read_inventory(path: Path) -> dict[str, str]:
    payload = _regular(path, "inventory").read_bytes()
    require(payload.endswith(b"\n"), "inventory must end with a complete line")
    try:
        lines = payload.decode("utf-8").splitlines()
    except UnicodeDecodeError as error:
        raise VerificationError("inventory must be UTF-8") from error
    result: dict[str, str] = {}
    names: list[str] = []
    for number, line in enumerate(lines, 1):
        match = re.fullmatch(r"([0-9a-f]{64})  (.+)", line)
        require(match is not None, f"invalid inventory line {number}")
        expected, raw_name = match.groups()
        name = _inventory_relative(raw_name, f"inventory path at line {number}")
        require(name not in result, f"duplicate inventory path: {name}")
        result[name] = expected
        names.append(name)
    require(result and names == sorted(names), "inventory must be nonempty and path-sorted")
    return result


def _scan_tree(root: Path) -> tuple[set[str], set[str]]:
    files: set[str] = set()
    directories: set[str] = set()
    pending = [root]
    while pending:
        parent = pending.pop()
        with os.scandir(parent) as entries:
            for entry in entries:
                path = Path(entry.path)
                relative = path.relative_to(root).as_posix()
                info = entry.stat(follow_symlinks=False)
                require(not entry.is_symlink(), f"symlink forbidden in copied results: {relative}")
                if stat.S_ISDIR(info.st_mode):
                    require(relative not in directories, f"duplicate directory: {relative}")
                    directories.add(relative)
                    pending.append(path)
                else:
                    require(stat.S_ISREG(info.st_mode) and info.st_nlink == 1,
                            f"special or hard-linked file forbidden: {relative}")
                    require(relative not in files, f"duplicate file: {relative}")
                    files.add(relative)
    return files, directories


def verify_inventory(root: Path, inventory: dict[str, str]) -> tuple[int, int]:
    observed, directories = _scan_tree(root)
    require(observed == set(inventory), "copied result file set differs from the pinned remote inventory")
    expected_directories = {parent.as_posix() for name in inventory for parent in Path(name).parents
                            if parent.as_posix() != "."}
    require(directories == expected_directories, "copied result directory closure differs")
    require(not any(Path(name).name == "failed.json" for name in observed),
            "successful F transfer must contain no failed.json")
    total = 0
    for name, expected in inventory.items():
        path = root / name
        require(digest(path) == expected, f"copied result SHA differs: {name}")
        total += path.stat().st_size
    return len(observed), total


def _false_fields(value: dict[str, Any], fields: tuple[str, ...], context: str) -> None:
    for field in fields:
        require(value.get(field) is False, f"{context}.{field} must be false")


def _source_files(root: Path, plan: dict[str, Any]) -> set[str]:
    source = plan.get("source")
    require(type(source) is dict and set(source) == {
        "base_commit", "base_sha256", "current_sha256", "overlay_paths", "added_paths", "added_sha256"
    }, "plan source closure differs")
    require(source.get("base_commit") == BASE_COMMIT and source.get("overlay_paths") == list(SOURCE_OVERLAYS)
            and source.get("added_paths") == list(SOURCE_ADDED_PATHS), "source identity differs")
    base, current, added = source["base_sha256"], source["current_sha256"], source["added_sha256"]
    require(type(base) is dict and type(current) is dict and type(added) is dict,
            "source SHA maps required")
    require(set(base) == set(current) == set(SOURCE_BASE_PATHS) and set(added) == set(SOURCE_ADDED_PATHS),
            "source path closure differs")
    names: set[str] = set()
    for name in SOURCE_BASE_PATHS:
        current_sha = _sha(current[name], f"current source {name}")
        base_sha = _sha(base[name], f"base source {name}")
        require((current_sha != base_sha) == (name in SOURCE_OVERLAYS),
                f"source overlay byte distinction differs: {name}")
        archive = "source/" + name
        require(digest(root / archive) == current_sha, f"archived current source differs: {name}")
        names.add(archive)
    for name in SOURCE_ADDED_PATHS:
        archive = "source/" + name
        require(digest(root / archive) == _sha(added[name], f"added source {name}"),
                f"archived added source differs: {name}")
        names.add(archive)
    extras = ((RUNNER_ARCHIVE, plan.get("runner_sha256")),
              (EVALUATOR_ARCHIVE, plan.get("evaluator_sha256")),
              (OWNER_ARCHIVE, plan.get("owner", {}).get("sha256")))
    for name, expected in extras:
        require(digest(root / name) == _sha(expected, name), f"archived helper differs: {name}")
        names.add(name)
    require(digest(root / "view_report.json") == _sha(plan.get("view_report_sha256"), "view report"),
            "archived view report differs")
    require(len(names) == 22, "source archive count differs")
    return names


def _reference_files(root: Path, plan: dict[str, Any]) -> set[str]:
    result: set[str] = set()
    for key, prefix, expected_paths, copied_labels in (
        ("reference_C", "reference_C", REFERENCE_C_PATHS, set(REFERENCE_C_PATHS)),
        ("reference_D", "reference_D", REFERENCE_D_PATHS, set(REFERENCE_D_PATHS)),
    ):
        reference = plan.get(key)
        require(type(reference) is dict and type(reference.get("files")) is dict,
                f"{key} bindings missing")
        expected_labels = set(expected_paths) | ({"checkpoint"} if key == "reference_C" else set())
        require(set(reference["files"]) == expected_labels, f"{key} file binding set differs")
        for label in copied_labels:
            row = reference["files"][label]
            relative = expected_paths[label]
            archive = f"{prefix}/{relative}"
            require(type(row) is dict and row.get("path") == relative
                    and digest(root / archive) == _sha(row.get("sha256"), f"{key}.{label}"),
                    f"{key} copied binding differs: {label}")
            result.add(archive)
        if key == "reference_C":
            checkpoint = reference["files"]["checkpoint"]
            require(type(checkpoint) is dict and checkpoint.get("path") ==
                    "A_baseline_train/training/checkpoints/latest.pt",
                    "reference C checkpoint path differs")
            _sha(checkpoint.get("sha256"), "reference C checkpoint")
    require(len(result) == 10, "reference archive count differs")
    return result


def _png_files(root: Path, inventory: dict[str, str], subgroup: dict[str, Any]) -> tuple[set[str], dict[str, int]]:
    pngs = {name for name in inventory if Path(name).suffix.lower() == ".png"}
    expected = {f"{ARM}/evaluation/trajectories": 12}
    expected.update({f"{ARM}/subgroup/episodes/{label}/trajectories": 6 for label in EXPECTED_EPISODES})
    counts: dict[str, int] = {}
    for directory, count in expected.items():
        names = {name for name in pngs if Path(name).parent.as_posix() == directory}
        require(len(names) == count and all(re.fullmatch(r"[0-9]{4}_.+\.png", Path(name).name)
                                                for name in names),
                f"PNG path/count differs: {directory}")
        counts[directory] = len(names)
    require(sum(counts.values()) == len(pngs) == 24, "unexpected PNG path/count")
    episodes = subgroup.get("episodes")
    require(type(episodes) is dict, "subgroup episode map required")
    for label in EXPECTED_EPISODES:
        hashes = episodes[label].get("trajectory_png_sha256")
        require(type(hashes) is dict and len(hashes) == 6, f"subgroup PNG binding differs: {label}")
        for filename, expected_sha in hashes.items():
            _relative(filename, f"subgroup PNG filename {label}")
            require(Path(filename).name == filename and digest(root / ARM / "subgroup/episodes" / label /
                    "trajectories" / filename) == _sha(expected_sha, f"subgroup PNG {label}/{filename}"),
                    f"subgroup PNG SHA differs: {label}/{filename}")
    return pngs, counts


def _history(path: Path) -> tuple[str, int]:
    count = 0
    last: dict[str, Any] | None = None
    with _regular(path, "training history").open("r", encoding="utf-8") as stream:
        for line in stream:
            count += 1
            require(line.endswith("\n"), "training history contains an incomplete line")
            try:
                last = json.loads(line, object_pairs_hook=_pairs,
                                  parse_constant=lambda item: require(False, f"nonfinite history value: {item}"))
            except json.JSONDecodeError as error:
                raise VerificationError(f"invalid training history line {count}: {error}") from error
            require(type(last) is dict and last.get("global_step") == count,
                    f"training history step differs at line {count}")
            try:
                json.dumps(last, allow_nan=False)
            except (TypeError, ValueError) as error:
                raise VerificationError(f"nonfinite training history line {count}") from error
    require(count == EXPECTED_HISTORY_ROWS and last is not None
            and last.get("samples_seen") == EXPECTED_TRAIN_STATE["samples_seen"]
            and last.get("domain_samples_seen") == EXPECTED_TRAIN_STATE["domain_samples_seen"]
            and last.get("sampling_stream_samples_seen") == EXPECTED_STREAM_SAMPLES,
            "training history endpoint/count differs")
    return digest(path), count


def _reports(root: Path, plan: dict[str, Any], checkpoint_sha: str) -> tuple[dict[str, Any], str, int]:
    item = root / ARM
    train = read_json(item / "training/run.json")
    evaluation = read_json(item / "evaluation/metrics.json")
    audit = read_json(item / "gate_v8.json")
    subgroup = read_json(item / "subgroup/summary.json")
    require(train.get("status") == "TRAINING_TARGET_REACHED"
            and train.get("warning") == "RESEARCH CHECKPOINT — NOT APPROVED FOR VEHICLE CONTROL"
            and "resumed_at_utc" not in train and train.get("state") == EXPECTED_TRAIN_STATE,
            "fresh F training endpoint differs")
    require(train.get("train_config", {}).get("sampling_policy") == "direction_balanced_cohort_replay"
            and train.get("sampling_plan_sha256") == EXPECTED_SAMPLING_PLAN_SHA256
            and train.get("sampling_execution_schedule_sha256") == EXPECTED_EXECUTION_SCHEDULE_SHA256
            and train.get("sampling_execution_schedule_row_count") == EXPECTED_EXECUTION_SCHEDULE_ROWS
            and train.get("sampling_metrics", {}).get("sampling_stream_samples_seen") == EXPECTED_STREAM_SAMPLES
            and train.get("last_metrics", {}).get("sampling_stream_samples_seen") == EXPECTED_STREAM_SAMPLES,
            "F replay plan or accounting differs")
    for report, status_name, context in (
        (evaluation, "OPEN_LOOP_EVALUATION_COMPLETE", "evaluation"),
        (audit, "RUNTIME_GEOMETRY_AUDIT_COMPLETE", "audit"),
    ):
        require(report.get("status") == status_name and report.get("checkpoint_sha256") == checkpoint_sha
                and report.get("evaluation_split") == "val" and report.get("evaluation_episode_count") == 2
                and report.get("training_sampling_plan_sha256") == EXPECTED_SAMPLING_PLAN_SHA256
                and report.get("training_sampling_policy") == "direction_balanced_cohort_replay"
                and report.get("vehicle_control_approved") is False,
                f"{context} provenance/scope differs")
    require(evaluation.get("sample_count") == 665 and evaluation.get("rendered_trajectory_png") == 12,
            "combined VAL evaluation count/render count differs")
    require(subgroup.get("schema") == SUBGROUP_SCHEMA and subgroup.get("status") == "COMPLETE_NOT_PROMOTED"
            and subgroup.get("arm") == ARM and subgroup.get("plan_sha256") == digest(root / "plan.json")
            and subgroup.get("runner_sha256") == plan.get("runner_sha256")
            and subgroup.get("evaluator_sha256") == plan.get("evaluator_sha256")
            and subgroup.get("checkpoint_sha256") == checkpoint_sha
            and subgroup.get("checkpoint", {}).get("sha256") == checkpoint_sha
            and subgroup.get("checkpoint", {}).get("sampling_plan_sha256") == EXPECTED_SAMPLING_PLAN_SHA256
            and subgroup.get("combined_val_count") == 665,
            "F subgroup identity/checkpoint differs")
    _false_fields(subgroup, SUBGROUP_DENIALS, "subgroup")
    episodes = subgroup.get("episodes")
    require(type(episodes) is dict and set(episodes) == set(EXPECTED_EPISODES), "subgroup episode set differs")
    for label, (episode_id, count) in EXPECTED_EPISODES.items():
        row = episodes[label]
        report_relative = f"episodes/{label}/metrics.json"
        require(type(row) is dict and row.get("episode_id") == episode_id and row.get("count") == count
                and row.get("report_path") == report_relative
                and row.get("report_sha256") == digest(item / "subgroup" / report_relative),
                f"subgroup episode binding/count differs: {label}")
    history_sha, history_rows = _history(item / "training/metrics.jsonl")
    return subgroup, history_sha, history_rows


def _stage_receipts(root: Path, plan: dict[str, Any], summary: dict[str, Any],
                    checkpoint_sha: str, history_sha: str) -> None:
    item = root / ARM
    plan_sha = digest(root / "plan.json")
    rows = summary.get("stages")
    require(type(rows) is list and [row.get("stage") for row in rows if type(row) is dict] == list(STAGES),
            "summary stage order/count differs")
    for stage, summary_row in zip(STAGES, rows, strict=True):
        started = read_json(item / f"{stage}_started.json")
        completed = read_json(item / f"{stage}_completed.json")
        require(completed == summary_row, f"summary/completed receipt differs: {stage}")
        for key in ("arm", "stage", "command", "timeout_seconds", "started_at_utc", "owner_pid",
                    "plan_sha256", "input_checkpoint_sha256", "pre_cuda_gpu"):
            require(started.get(key) == completed.get(key), f"started/completed receipt differs: {stage}/{key}")
        expected_input = None if stage == "train" else checkpoint_sha
        report = f"{ARM}/{STAGE_REPORTS[stage]}"
        require(started.get("status") == "RUNNING" and completed.get("status") == "COMPLETE"
                and completed.get("arm") == ARM and completed.get("stage") == stage
                and completed.get("plan_sha256") == plan_sha
                and completed.get("input_checkpoint_sha256") == expected_input
                and completed.get("checkpoint_sha256") == checkpoint_sha
                and completed.get("path") == report and completed.get("sha256") == digest(root / report),
                f"stage receipt/checkpoint binding differs: {stage}")
        command = completed.get("command")
        require(type(command) is list and all(type(value) is str for value in command),
                f"stage command differs: {stage}")
        if "--split" in command:
            position = command.index("--split")
            require(position + 1 < len(command) and command[position + 1] == ("train" if stage == "train" else "val"),
                    f"stage split differs: {stage}")
        require(not any(value.lower() == "test" for value in command), f"TEST command forbidden: {stage}")
        if stage == "train":
            require(completed.get("history_sha256") == history_sha
                    and completed.get("sampling_execution_schedule_row_count") == EXPECTED_EXECUTION_SCHEDULE_ROWS
                    and completed.get("sampling_execution_schedule_sha256") == EXPECTED_EXECUTION_SCHEDULE_SHA256,
                    "train receipt replay schedule differs")


def _expected_fixed_files() -> set[str]:
    files = {"plan.json", "started.json", "summary.json", "view_report.json",
             f"{ARM}/.training.portable-e2e.lock", f"{ARM}/training/checkpoints/latest.pt",
             f"{ARM}/training/metrics.jsonl", f"{ARM}/training/run.json",
             f"{ARM}/evaluation/metrics.json", f"{ARM}/gate_v8.json",
             f"{ARM}/subgroup/summary.json"}
    for stage in STAGES:
        files.update({f"{ARM}/{stage}.log", f"{ARM}/{stage}_started.json", f"{ARM}/{stage}_completed.json"})
    for label in EXPECTED_EPISODES:
        files.add(f"{ARM}/subgroup/episodes/{label}/metrics.json")
    return files


def verify(*, artifact_root: Path, remote_inventory_sha256: str,
           remote_inventory: Path | None = None, local_inventory: Path | None = None) -> dict[str, Any]:
    artifact = _directory(artifact_root, "artifact root")
    remote_inventory = _regular(remote_inventory or artifact / "remote_inventory.sha256", "remote inventory")
    local_inventory = _regular(local_inventory or artifact / "local_inventory.sha256", "local inventory")
    results_lexical = artifact / "remote_results"
    require(not remote_inventory.is_relative_to(results_lexical)
            and not local_inventory.is_relative_to(results_lexical),
            "inventories must remain outside copied results")
    external_pin = _sha(remote_inventory_sha256, "external remote inventory SHA")
    # HH_260906 - Authenticate the remote inventory before parsing any copied run payload.
    require(digest(remote_inventory) == external_pin, "external remote inventory SHA differs")
    remote_bytes = remote_inventory.read_bytes()
    require(remote_bytes == local_inventory.read_bytes(), "remote/local inventory files are not byte-identical")
    inventory = read_inventory(remote_inventory)
    require(inventory == read_inventory(local_inventory), "remote/local inventory mappings differ")
    results = _directory(results_lexical, "copied remote results")
    file_count, total_size = verify_inventory(results, inventory)
    plan = read_json(results / "plan.json")
    plan_sha = digest(results / "plan.json")
    require(plan.get("schema") == PLAN_SCHEMA and plan.get("campaign_id") == CAMPAIGN_ID
            and plan.get("base_commit") == BASE_COMMIT and plan.get("stage_order") == list(STAGES)
            and plan.get("arm", {}).get("label") == ARM, "F plan identity differs")
    _false_fields(plan, RUN_DENIALS, "plan")
    require(plan.get("decision", {}).get("adoption_allowed") is False, "F plan must deny adoption")
    arm_plan = plan.get("arm", {})
    require(arm_plan.get("expected_train_state") == EXPECTED_TRAIN_STATE
            and arm_plan.get("expected_sampling_stream_samples_seen") == EXPECTED_STREAM_SAMPLES
            and arm_plan.get("sampling_plan_sha256") == EXPECTED_SAMPLING_PLAN_SHA256
            and canonical_sha256(arm_plan.get("sampling_plan")) == EXPECTED_SAMPLING_PLAN_SHA256
            and arm_plan.get("execution_schedule", {}).get("row_count") == EXPECTED_EXECUTION_SCHEDULE_ROWS
            and arm_plan.get("execution_schedule", {}).get("sha256") == EXPECTED_EXECUTION_SCHEDULE_SHA256,
            "F prospective replay plan differs")
    require(plan.get("validation", {}).get("count") == 665, "F validation count differs")
    source_names = _source_files(results, plan)
    reference_names = _reference_files(results, plan)
    started = read_json(results / "started.json")
    summary = read_json(results / "summary.json")
    require(started.get("schema") == summary.get("schema") == SUMMARY_SCHEMA
            and started.get("status") == "RUNNING" and started.get("plan_sha256") == plan_sha
            and started.get("arm") == "NOT_RUN" and started.get("stages") == []
            and summary.get("status") == "COMPLETE_NOT_PROMOTED" and summary.get("plan_sha256") == plan_sha
            and summary.get("arm") == "COMPLETE" and summary.get("started_at_utc") == started.get("started_at_utc")
            and summary.get("source_dataset_reference_postcheck_pass") is True
            and summary.get("sampling_execution_schedule") == {
                "row_count": EXPECTED_EXECUTION_SCHEDULE_ROWS,
                "sha256": EXPECTED_EXECUTION_SCHEDULE_SHA256,
            }, "F start/summary completion receipt differs")
    _false_fields(started, RUN_DENIALS, "started")
    _false_fields(summary, RUN_DENIALS, "summary")
    gate = summary.get("diagnostic_gate")
    require(type(gate) is dict and gate.get("status") in {
        "PASS_SINGLE_SEED_DIAGNOSTIC", "FAIL_SINGLE_SEED_DIAGNOSTIC"}
            and gate.get("pass") is (gate.get("status") == "PASS_SINGLE_SEED_DIAGNOSTIC")
            and gate.get("adoption_allowed") is False and gate.get("automatic_promotion") is False,
            "reported diagnostic gate state/scope differs")
    checkpoint = results / ARM / "training/checkpoints/latest.pt"
    checkpoint_sha = digest(checkpoint)
    subgroup, history_sha, history_rows = _reports(results, plan, checkpoint_sha)
    _stage_receipts(results, plan, summary, checkpoint_sha, history_sha)
    png_names, png_counts = _png_files(results, inventory, subgroup)
    expected_names = (_expected_fixed_files() | source_names | reference_names | png_names)
    require(set(inventory) == expected_names and file_count == len(expected_names) == 81,
            "sealed successful F result file closure/count differs")
    stable_count, stable_size = verify_inventory(results, inventory)
    require((stable_count, stable_size) == (file_count, total_size)
            and digest(remote_inventory) == external_pin
            and remote_inventory.read_bytes() == local_inventory.read_bytes()
            and digest(results / "plan.json") == plan_sha and digest(checkpoint) == checkpoint_sha,
            "artifact or inventory changed during verification")
    return {
        "schema": SCHEMA,
        "status": "PASS_EXACT_SUCCESSFUL_F_TRANSFER_NOT_PROMOTED",
        "verified_at_utc": datetime.now(timezone.utc).isoformat(),
        "artifact_root": str(artifact),
        "remote_results": str(results),
        "remote_inventory_sha256": external_pin,
        "local_inventory_sha256": digest(local_inventory),
        "inventories_byte_identical": True,
        "file_count": file_count,
        "total_size_bytes": total_size,
        "source_file_count": len(source_names),
        "reference_file_count": len(reference_names),
        "png_count": len(png_names),
        "png_counts": png_counts,
        "plan_sha256": plan_sha,
        "runner_sha256": plan["runner_sha256"],
        "checkpoint_sha256": checkpoint_sha,
        "history_sha256": history_sha,
        "history_rows": history_rows,
        "completed_stage_count": 4,
        "diagnostic_gate_as_reported": gate,
        "diagnostic_gate_recomputed_by_transfer_verifier": False,
        "checkpoint_payload_deserialized": False,
        "torch_imported": False,
        "gpu_used": False,
        "test_read_or_use": False,
        "closed_loop_evaluation": False,
        "model_export": False,
        "automatic_promotion": False,
        "training_data_approved": False,
        "vehicle_control_approved": False,
    }


def write_new(path: Path, value: dict[str, Any], results: Path) -> None:
    target = Path(path).expanduser().absolute()
    require(".." not in target.parts and not target.exists() and not target.is_symlink(),
            "verification output must be fresh")
    require(not target.is_relative_to(results), "verification output must remain outside copied results")
    parent = _directory(target.parent, "verification output parent")
    payload = (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()
    with (parent / target.name).open("xb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--artifact-root", type=Path, required=True)
    parser.add_argument("--remote-inventory-sha256", required=True)
    parser.add_argument("--remote-inventory", type=Path)
    parser.add_argument("--local-inventory", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    result = verify(artifact_root=args.artifact_root,
                    remote_inventory_sha256=args.remote_inventory_sha256,
                    remote_inventory=args.remote_inventory,
                    local_inventory=args.local_inventory)
    write_new(args.output, result, Path(result["remote_results"]))
    print(json.dumps({"status": result["status"], "output": str(args.output),
                      "checkpoint_sha256": result["checkpoint_sha256"],
                      "file_count": result["file_count"], "png_count": result["png_count"]},
                     sort_keys=True, separators=(",", ":"), allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
