#!/usr/bin/env python3
"""HH_260906 - Verify the exact F postrun provenance and preserve its failed performance decision."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import sys
from typing import Any


SCHEMA = "portable_e2e.moving_direction_balanced_replay_postrun_verification.v1"
PLAN_SCHEMA = "portable_e2e.moving_direction_balanced_replay_seed_pilot_plan.v1"
SUMMARY_SCHEMA = "portable_e2e.moving_direction_balanced_replay_seed_pilot_summary.v1"
SUBGROUP_SCHEMA = "portable_e2e.moving_direction_balanced_replay_subgroup_evaluation.v1"
CAMPAIGN_ID = "hh260921-moving-direction-balanced-replay-seed20260906-v1"
BASE_COMMIT = "b478f02e42b94bf04bffec5c8170e05edc33b0f8"
ARM = "F_direction_balanced_replay_train"
STAGES = ("train", "evaluate", "audit", "subgroup")
EXPECTED_PLAN_SHA256 = "a2fdf6dcf66234c252d4289f2f979313902b8bbeb92f9796d07675e1b1eedc8d"
EXPECTED_SAMPLING_PLAN_SHA256 = "abb0e655b174844cc20f293bc12c5156fe33b3a00741ac1cfe8b703367e41c7b"
EXPECTED_SCHEDULE_SHA256 = "d340a10c42f2a733988b69ae6571ec3c37b7e8fa1601c43e6111192cd35c55b0"
EXPECTED_SCHEDULE_ROWS = 33120
EXPECTED_TRAIN_STATE = {
    "epoch": 20,
    "next_batch_index": 0,
    "global_step": 8280,
    "samples_seen": 33120,
    "domain_samples_seen": {"carla": 33120},
}
EXPECTED_STREAM_SAMPLES = {"base": 29500, "replay": 3620}
EXPECTED_EPISODES = {
    "town03_right_turn_dev": (
        "carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9", 337
    ),
    "town03_straight_dev": (
        "carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320", 328
    ),
}
RUN_DENIALS = (
    "resume", "automatic_retry", "test_read_or_use", "model_export",
    "automatic_promotion", "training_data_approved", "vehicle_control_approved",
    "closed_loop_claim", "all_map_claim", "deployment_approved",
)
SUBGROUP_DENIALS = (
    "test_read_or_use", "training_executed_by_evaluator", "closed_loop_evaluation",
    "vehicle_control_approved", "automatic_promotion", "model_exported",
)
SOURCE_CURRENT_PATHS = tuple(
    "portable_e2e/" + name + ".py"
    for name in (
        "__init__", "contract", "dataset", "torch_dataset", "model", "losses", "train",
        "evaluate", "audit_runtime", "runtime", "runtime_contract", "runtime_weight_bundle",
        "visualize", "stop_primitive_research",
    )
) + (
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
REFERENCE_LAYOUT = {
    "reference_C": {
        "checkpoint": "A_baseline_train/training/checkpoints/latest.pt",
        "plan": "plan.json",
        "summary": "summary.json",
        "training_run": "A_baseline_train/training/run.json",
        "subgroup": "A_baseline_train/subgroup/summary.json",
        "gate": "A_baseline_train/gate_v8.json",
    },
    "reference_D": {
        "checkpoint": "B_town07_augmented_train/training/checkpoints/latest.pt",
        "training_run": "B_town07_augmented_train/training/run.json",
        "evaluation": "B_town07_augmented_train/evaluation/metrics.json",
        "subgroup": "B_town07_augmented_train/subgroup/summary.json",
        "gate": "B_town07_augmented_train/gate_v8.json",
    },
}
REFERENCE_CHECKPOINT_COPIED = {"reference_C": False, "reference_D": True}
STAGE_REPORTS = {
    "train": "training/run.json",
    "evaluate": "evaluation/metrics.json",
    "audit": "gate_v8.json",
    "subgroup": "subgroup/summary.json",
}


class VerificationError(ValueError):
    """HH_260906 - Signal one fail-closed provenance or output-contract violation."""


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


def _directory(path: Path, context: str) -> Path:
    lexical = Path(path).expanduser().absolute()
    require(".." not in lexical.parts and lexical.exists() and lexical.resolve() == lexical,
            f"{context} must be an existing canonical path")
    info = lexical.lstat()
    require(stat.S_ISDIR(info.st_mode) and not lexical.is_symlink(),
            f"{context} must be a nonsymlink directory")
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


def canonical_sha256(value: Any) -> str:
    try:
        payload = json.dumps(value, sort_keys=True, separators=(",", ":"),
                             allow_nan=False).encode()
    except (TypeError, ValueError) as error:
        raise VerificationError("canonical SHA input is not finite JSON") from error
    return hashlib.sha256(payload).hexdigest()


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


def _relative(value: Any, context: str) -> str:
    require(type(value) is str and value and "\\" not in value, f"{context} must be a POSIX path")
    path = Path(value)
    require(not path.is_absolute() and ".." not in path.parts and path.as_posix() == value
            and value != ".", f"{context} must remain below its root")
    return value


def _scan_tree(root: Path) -> set[str]:
    files: set[str] = set()
    pending = [root]
    while pending:
        parent = pending.pop()
        with os.scandir(parent) as entries:
            for entry in entries:
                path = Path(entry.path)
                relative = path.relative_to(root).as_posix()
                info = entry.stat(follow_symlinks=False)
                require(not entry.is_symlink(), f"symlink forbidden in F campaign: {relative}")
                if stat.S_ISDIR(info.st_mode):
                    pending.append(path)
                else:
                    require(stat.S_ISREG(info.st_mode) and info.st_nlink == 1,
                            f"special or hard-linked file forbidden in F campaign: {relative}")
                    require(relative not in files, f"duplicate campaign file: {relative}")
                    files.add(relative)
    require(not any(Path(name).name == "failed.json" for name in files),
            "successful F campaign must contain no failed.json")
    return files


def _false_fields(value: dict[str, Any], fields: tuple[str, ...], context: str) -> None:
    for field in fields:
        require(value.get(field) is False, f"{context}.{field} must be false")


def _verify_plan(root: Path, plan: dict[str, Any]) -> tuple[str, str]:
    plan_sha = digest(root / "plan.json")
    require(plan_sha == EXPECTED_PLAN_SHA256, "exact F plan SHA differs")
    require(plan.get("schema") == PLAN_SCHEMA and plan.get("campaign_id") == CAMPAIGN_ID
            and plan.get("base_commit") == BASE_COMMIT and plan.get("stage_order") == list(STAGES),
            "F plan identity or stage order differs")
    _false_fields(plan, RUN_DENIALS, "plan")
    require(plan.get("decision", {}).get("adoption_allowed") is False,
            "F plan must deny automatic adoption")
    arm = plan.get("arm")
    require(type(arm) is dict and arm.get("label") == ARM, "F plan arm differs")
    sampling_plan = arm.get("sampling_plan")
    sampling_sha = _sha(arm.get("sampling_plan_sha256"), "sampling plan SHA")
    require(sampling_sha == EXPECTED_SAMPLING_PLAN_SHA256
            and canonical_sha256(sampling_plan) == sampling_sha,
            "sealed sampling plan SHA differs")
    schedule = arm.get("execution_schedule")
    require(type(schedule) is dict and schedule.get("row_count") == EXPECTED_SCHEDULE_ROWS
            and type(schedule.get("rows")) is list
            and len(schedule["rows"]) == EXPECTED_SCHEDULE_ROWS,
            "execution schedule row accounting differs")
    schedule_sha = _sha(schedule.get("sha256"), "execution schedule SHA")
    require(schedule_sha == EXPECTED_SCHEDULE_SHA256
            and canonical_sha256(schedule["rows"]) == schedule_sha,
            "sealed execution schedule SHA differs")
    require(arm.get("expected_train_state") == EXPECTED_TRAIN_STATE
            and arm.get("expected_sampling_stream_samples_seen") == EXPECTED_STREAM_SAMPLES,
            "prospective training endpoint or stream totals differ")
    validation = plan.get("validation")
    expected_validation = [
        {"label": label, "episode_id": episode_id, "count": count}
        for label, (episode_id, count) in EXPECTED_EPISODES.items()
    ]
    require(type(validation) is dict and validation.get("count") == 665
            and validation.get("episodes") == expected_validation,
            "fixed validation population differs")
    return plan_sha, sampling_sha


def _verify_source_archive(root: Path, plan: dict[str, Any]) -> int:
    source = plan.get("source")
    require(type(source) is dict and set(source) == {
        "base_commit", "base_sha256", "current_sha256", "overlay_paths",
        "added_paths", "added_sha256",
    }, "plan source closure differs")
    base = source.get("base_sha256")
    current = source.get("current_sha256")
    added = source.get("added_sha256")
    require(source.get("base_commit") == BASE_COMMIT
            and source.get("overlay_paths") == list(SOURCE_OVERLAYS)
            and source.get("added_paths") == list(SOURCE_ADDED_PATHS)
            and type(base) is dict and type(current) is dict and type(added) is dict,
            "plan source identity differs")
    require(set(base) == set(current) == set(SOURCE_CURRENT_PATHS)
            and set(added) == set(SOURCE_ADDED_PATHS), "plan source path closure differs")
    expected_archives: set[str] = set()
    for name in SOURCE_CURRENT_PATHS:
        base_sha = _sha(base[name], f"base source {name}")
        current_sha = _sha(current[name], f"current source {name}")
        require((base_sha != current_sha) == (name in SOURCE_OVERLAYS),
                f"source overlay distinction differs: {name}")
        archive = "source/" + name
        require(digest(root / archive) == current_sha, f"source archive SHA differs: {name}")
        expected_archives.add(archive)
    for name in SOURCE_ADDED_PATHS:
        archive = "source/" + name
        require(digest(root / archive) == _sha(added[name], f"added source {name}"),
                f"added source archive SHA differs: {name}")
        expected_archives.add(archive)
    for archive, expected in (
        (RUNNER_ARCHIVE, plan.get("runner_sha256")),
        (EVALUATOR_ARCHIVE, plan.get("evaluator_sha256")),
        (OWNER_ARCHIVE, plan.get("owner", {}).get("sha256")),
    ):
        require(digest(root / archive) == _sha(expected, archive),
                f"archived helper SHA differs: {archive}")
        expected_archives.add(archive)
    require(digest(root / "view_report.json") ==
            _sha(plan.get("view_report_sha256"), "view report SHA"),
            "view report SHA differs")
    observed = {
        path.relative_to(root).as_posix()
        for path in (root / "source").rglob("*") if path.is_file()
    }
    require(observed == expected_archives, "source archive file closure differs")
    return len(observed)


def _verify_references(root: Path, plan: dict[str, Any]) -> tuple[int, dict[str, str]]:
    copied_count = 0
    checkpoint_shas: dict[str, str] = {}
    for key, layout in REFERENCE_LAYOUT.items():
        reference = plan.get(key)
        require(type(reference) is dict and type(reference.get("files")) is dict
                and set(reference["files"]) == set(layout), f"{key} plan file closure differs")
        checkpoint_row = reference["files"]["checkpoint"]
        require(type(checkpoint_row) is dict
                and checkpoint_row.get("path") == layout["checkpoint"],
                f"{key} checkpoint declaration differs")
        checkpoint_sha = _sha(checkpoint_row.get("sha256"), f"{key} checkpoint SHA")
        checkpoint_shas[key] = checkpoint_sha
        expected_copied: set[str] = set()
        for label, relative in layout.items():
            if label == "checkpoint" and not REFERENCE_CHECKPOINT_COPIED[key]:
                continue
            row = reference["files"].get(label)
            require(type(row) is dict and row.get("path") == relative,
                    f"{key}.{label} plan binding differs")
            archive = f"{key}/{relative}"
            require(digest(root / archive) == _sha(row.get("sha256"), f"{key}.{label} SHA"),
                    f"{key}.{label} archive SHA differs")
            expected_copied.add(archive)
        if REFERENCE_CHECKPOINT_COPIED[key]:
            require(digest(root / key / layout["checkpoint"]) == checkpoint_sha,
                    f"{key} copied checkpoint SHA differs")
        observed = {
            path.relative_to(root).as_posix()
            for path in (root / key).rglob("*") if path.is_file()
        }
        require(observed == expected_copied, f"{key} archive file closure differs")
        copied_count += len(observed)
    c_subgroup = read_json(root / "reference_C/A_baseline_train/subgroup/summary.json")
    c_gate = read_json(root / "reference_C/A_baseline_train/gate_v8.json")
    require(c_subgroup.get("checkpoint_sha256") == checkpoint_shas["reference_C"]
            and c_subgroup.get("checkpoint", {}).get("sha256") == checkpoint_shas["reference_C"]
            and c_gate.get("checkpoint_sha256") == checkpoint_shas["reference_C"],
            "reference_C declared checkpoint SHA is not bound to its archived reports")
    d_evaluation = read_json(root / "reference_D/B_town07_augmented_train/evaluation/metrics.json")
    d_subgroup = read_json(root / "reference_D/B_town07_augmented_train/subgroup/summary.json")
    d_gate = read_json(root / "reference_D/B_town07_augmented_train/gate_v8.json")
    require(d_evaluation.get("checkpoint_sha256") == checkpoint_shas["reference_D"]
            and d_subgroup.get("checkpoint_sha256") == checkpoint_shas["reference_D"]
            and d_subgroup.get("checkpoint", {}).get("sha256") == checkpoint_shas["reference_D"]
            and d_gate.get("checkpoint_sha256") == checkpoint_shas["reference_D"],
            "reference_D declared checkpoint SHA is not bound to its archived reports")
    return copied_count, checkpoint_shas


def _verify_history(path: Path) -> tuple[str, int]:
    count = 0
    last: dict[str, Any] | None = None
    with _regular(path, "training history").open("r", encoding="utf-8") as stream:
        for line in stream:
            count += 1
            require(line.endswith("\n"), "training history contains an incomplete line")
            try:
                last = json.loads(
                    line,
                    object_pairs_hook=_pairs,
                    parse_constant=lambda item: require(False, f"nonfinite history value: {item}"),
                )
            except json.JSONDecodeError as error:
                raise VerificationError(f"invalid training history line {count}: {error}") from error
            require(type(last) is dict and last.get("global_step") == count,
                    f"training history step differs at line {count}")
    require(count == EXPECTED_TRAIN_STATE["global_step"] and last is not None
            and last.get("samples_seen") == EXPECTED_TRAIN_STATE["samples_seen"]
            and last.get("domain_samples_seen") == EXPECTED_TRAIN_STATE["domain_samples_seen"]
            and last.get("sampling_stream_samples_seen") == EXPECTED_STREAM_SAMPLES,
            "training history endpoint or stream totals differ")
    return digest(path), count


def _verify_reports(root: Path, plan: dict[str, Any], plan_sha: str,
                    sampling_sha: str) -> tuple[str, str, dict[str, Any]]:
    item = root / ARM
    checkpoint_sha = digest(item / "training/checkpoints/latest.pt")
    train = read_json(item / "training/run.json")
    evaluation = read_json(item / "evaluation/metrics.json")
    audit = read_json(item / "gate_v8.json")
    subgroup = read_json(item / "subgroup/summary.json")
    require(train.get("status") == "TRAINING_TARGET_REACHED"
            and train.get("warning") == "RESEARCH CHECKPOINT — NOT APPROVED FOR VEHICLE CONTROL"
            and "resumed_at_utc" not in train and train.get("state") == EXPECTED_TRAIN_STATE,
            "fresh F training endpoint differs")
    require(train.get("train_config", {}).get("sampling_policy") ==
            "direction_balanced_cohort_replay"
            and train.get("sampling_plan_sha256") == sampling_sha
            and canonical_sha256(train.get("sampling_plan")) == sampling_sha
            and train.get("sampling_execution_schedule_sha256") == EXPECTED_SCHEDULE_SHA256
            and train.get("sampling_execution_schedule_row_count") == EXPECTED_SCHEDULE_ROWS,
            "training sampling plan or schedule differs")
    for name in ("sampling_metrics", "last_metrics"):
        value = train.get(name)
        require(type(value) is dict
                and value.get("sampling_stream_samples_seen") == EXPECTED_STREAM_SAMPLES
                and value.get("samples_seen", EXPECTED_TRAIN_STATE["samples_seen"])
                == EXPECTED_TRAIN_STATE["samples_seen"]
                and value.get("domain_samples_seen") == EXPECTED_TRAIN_STATE["domain_samples_seen"],
                f"training {name} accounting differs")
    require(evaluation.get("status") == "OPEN_LOOP_EVALUATION_COMPLETE"
            and evaluation.get("checkpoint_sha256") == checkpoint_sha
            and evaluation.get("evaluation_split") == "val"
            and evaluation.get("evaluation_episode_count") == 2
            and evaluation.get("sample_count") == 665
            and evaluation.get("rendered_trajectory_png") == 12
            and evaluation.get("training_sampling_plan_sha256") == sampling_sha
            and evaluation.get("training_sampling_policy") == "direction_balanced_cohort_replay"
            and evaluation.get("training_sampling_stream_samples_seen") == EXPECTED_STREAM_SAMPLES
            and evaluation.get("vehicle_control_approved") is False,
            "combined VAL evaluation provenance/count differs")
    require(audit.get("status") == "RUNTIME_GEOMETRY_AUDIT_COMPLETE"
            and audit.get("checkpoint_sha256") == checkpoint_sha
            and audit.get("evaluation_split") == "val"
            and audit.get("evaluation_episode_count") == 2
            and audit.get("training_sampling_plan_sha256") == sampling_sha
            and audit.get("training_sampling_policy") == "direction_balanced_cohort_replay"
            and audit.get("vehicle_control_approved") is False,
            "geometry audit provenance differs")
    geometry = audit.get("geometry")
    require(type(geometry) is dict and geometry.get("sample_count") == 665
            and geometry.get("selected_result", {}).get("sample_count") == 665
            and geometry.get("selected_result", {}).get("geometry_pass_count") == 664
            and geometry.get("any_candidate_geometry_pass_count") == 664
            and geometry.get("all_candidates_geometry_pass_count") == 664,
            "geometry audit must remain exactly 664/665")
    require(subgroup.get("schema") == SUBGROUP_SCHEMA
            and subgroup.get("status") == "COMPLETE_NOT_PROMOTED"
            and subgroup.get("arm") == ARM and subgroup.get("plan_sha256") == plan_sha
            and subgroup.get("runner_sha256") == plan.get("runner_sha256")
            and subgroup.get("evaluator_sha256") == plan.get("evaluator_sha256")
            and subgroup.get("checkpoint_sha256") == checkpoint_sha
            and subgroup.get("checkpoint", {}).get("sha256") == checkpoint_sha
            and subgroup.get("checkpoint", {}).get("sampling_plan_sha256") == sampling_sha
            and subgroup.get("combined_val_count") == 665,
            "subgroup provenance or combined count differs")
    _false_fields(subgroup, SUBGROUP_DENIALS, "subgroup")
    episodes = subgroup.get("episodes")
    require(type(episodes) is dict and set(episodes) == set(EXPECTED_EPISODES),
            "subgroup episode closure differs")
    for label, (episode_id, count) in EXPECTED_EPISODES.items():
        row = episodes[label]
        report_path = f"episodes/{label}/metrics.json"
        report = read_json(item / "subgroup" / report_path)
        require(type(row) is dict and row.get("episode_id") == episode_id
                and row.get("count") == count and row.get("report_path") == report_path
                and row.get("report_sha256") == digest(item / "subgroup" / report_path)
                and report.get("checkpoint_sha256") == checkpoint_sha
                and report.get("sample_count") == count,
                f"subgroup episode provenance/count differs: {label}")
    history_sha, _ = _verify_history(item / "training/metrics.jsonl")
    return checkpoint_sha, history_sha, subgroup


def _verify_stage_receipts(root: Path, summary: dict[str, Any], plan_sha: str,
                           checkpoint_sha: str, history_sha: str) -> None:
    item = root / ARM
    rows = summary.get("stages")
    require(type(rows) is list and len(rows) == len(STAGES)
            and all(type(row) is dict for row in rows)
            and [row.get("stage") for row in rows] == list(STAGES),
            "summary stage order/count differs")
    observed_started = {path.name for path in item.glob("*_started.json")}
    observed_completed = {path.name for path in item.glob("*_completed.json")}
    require(observed_started == {f"{stage}_started.json" for stage in STAGES}
            and observed_completed == {f"{stage}_completed.json" for stage in STAGES},
            "stage receipt file closure differs")
    for stage, summary_row in zip(STAGES, rows, strict=True):
        started = read_json(item / f"{stage}_started.json")
        completed = read_json(item / f"{stage}_completed.json")
        require(completed == summary_row, f"summary/completed receipt differs: {stage}")
        for key in (
            "arm", "stage", "command", "timeout_seconds", "started_at_utc", "owner_pid",
            "plan_sha256", "input_checkpoint_sha256", "pre_cuda_gpu",
        ):
            require(started.get(key) == completed.get(key),
                    f"started/completed receipt differs: {stage}/{key}")
        expected_input = None if stage == "train" else checkpoint_sha
        report_relative = f"{ARM}/{STAGE_REPORTS[stage]}"
        require(started.get("status") == "RUNNING" and completed.get("status") == "COMPLETE"
                and completed.get("arm") == ARM and completed.get("stage") == stage
                and completed.get("plan_sha256") == plan_sha
                and completed.get("input_checkpoint_sha256") == expected_input
                and completed.get("checkpoint_sha256") == checkpoint_sha
                and completed.get("path") == report_relative
                and completed.get("sha256") == digest(root / report_relative),
                f"stage receipt/report/checkpoint binding differs: {stage}")
        command = completed.get("command")
        require(type(command) is list and command and all(type(value) is str for value in command)
                and not any(value.lower() == "test" for value in command),
                f"stage command is invalid or reads TEST: {stage}")
        if stage == "train":
            require(completed.get("history_sha256") == history_sha
                    and completed.get("sampling_execution_schedule_row_count") ==
                    EXPECTED_SCHEDULE_ROWS
                    and completed.get("sampling_execution_schedule_sha256") ==
                    EXPECTED_SCHEDULE_SHA256,
                    "train receipt history or schedule differs")


def _verify_pngs(root: Path, files: set[str], subgroup: dict[str, Any]) -> dict[str, int]:
    pngs = {name for name in files if Path(name).suffix.lower() == ".png"}
    expected_directories = {f"{ARM}/evaluation/trajectories": 12}
    expected_directories.update(
        {f"{ARM}/subgroup/episodes/{label}/trajectories": 6 for label in EXPECTED_EPISODES}
    )
    counts: dict[str, int] = {}
    for directory, expected_count in expected_directories.items():
        names = {name for name in pngs if Path(name).parent.as_posix() == directory}
        require(len(names) == expected_count, f"PNG count differs: {directory}")
        for name in names:
            with _regular(root / name, "PNG evidence").open("rb") as stream:
                require(stream.read(8) == b"\x89PNG\r\n\x1a\n", f"invalid PNG signature: {name}")
        counts[directory] = len(names)
    require(sum(counts.values()) == len(pngs) == 24, "F evidence must contain exactly 24 PNGs")
    for label in EXPECTED_EPISODES:
        hashes = subgroup["episodes"][label].get("trajectory_png_sha256")
        require(type(hashes) is dict and len(hashes) == 6,
                f"subgroup PNG hash closure differs: {label}")
        for filename, expected_sha in hashes.items():
            _relative(filename, f"subgroup PNG filename {label}")
            require(Path(filename).name == filename
                    and digest(root / ARM / "subgroup/episodes" / label / "trajectories" / filename)
                    == _sha(expected_sha, f"subgroup PNG SHA {label}/{filename}"),
                    f"subgroup PNG SHA differs: {label}/{filename}")
    return counts


def _verify_run_envelope(root: Path, plan_sha: str) -> tuple[dict[str, Any], dict[str, Any]]:
    started = read_json(root / "started.json")
    summary = read_json(root / "summary.json")
    require(started.get("schema") == summary.get("schema") == SUMMARY_SCHEMA
            and started.get("status") == "RUNNING" and started.get("plan_sha256") == plan_sha
            and started.get("arm") == "NOT_RUN" and started.get("stages") == []
            and summary.get("status") == "COMPLETE_NOT_PROMOTED"
            and summary.get("plan_sha256") == plan_sha and summary.get("arm") == "COMPLETE"
            and summary.get("started_at_utc") == started.get("started_at_utc")
            and summary.get("source_dataset_reference_postcheck_pass") is True
            and summary.get("sampling_execution_schedule") == {
                "row_count": EXPECTED_SCHEDULE_ROWS, "sha256": EXPECTED_SCHEDULE_SHA256,
            }, "F started/summary envelope differs")
    _false_fields(started, RUN_DENIALS, "started")
    _false_fields(summary, RUN_DENIALS, "summary")
    gate = summary.get("diagnostic_gate")
    require(type(gate) is dict and gate.get("status") == "FAIL_SINGLE_SEED_DIAGNOSTIC"
            and gate.get("pass") is False and gate.get("adoption_allowed") is False
            and gate.get("automatic_promotion") is False,
            "F diagnostic decision must remain FAIL_SINGLE_SEED_DIAGNOSTIC")
    geometry = gate.get("geometry")
    require(type(geometry) is dict and geometry.get("sample_count") == 665
            and geometry.get("pass") is True
            and all(geometry.get("checks", {}).get(name, {}).get("observed_F") == 664
                    for name in ("selected", "any_candidate", "all_candidates")),
            "diagnostic gate geometry summary must remain exactly 664/665")
    return started, summary


def verify(campaign_root: Path) -> dict[str, Any]:
    root = _directory(campaign_root, "F campaign root")
    files = _scan_tree(root)
    plan = read_json(root / "plan.json")
    plan_sha, sampling_sha = _verify_plan(root, plan)
    source_count = _verify_source_archive(root, plan)
    reference_count, reference_checkpoint_shas = _verify_references(root, plan)
    _, summary = _verify_run_envelope(root, plan_sha)
    checkpoint_sha, history_sha, subgroup = _verify_reports(root, plan, plan_sha, sampling_sha)
    _verify_stage_receipts(root, summary, plan_sha, checkpoint_sha, history_sha)
    png_counts = _verify_pngs(root, files, subgroup)
    stable_checkpoint_sha = digest(root / ARM / "training/checkpoints/latest.pt")
    require(digest(root / "plan.json") == plan_sha and stable_checkpoint_sha == checkpoint_sha
            and not any(Path(name).name == "failed.json" for name in _scan_tree(root)),
            "F campaign changed during postrun verification")
    return {
        "schema": SCHEMA,
        "status": "VERIFIED_NOT_PROMOTED",
        "verified_at_utc": datetime.now(timezone.utc).isoformat(),
        "campaign_root": str(root),
        "campaign_id": CAMPAIGN_ID,
        "arm": ARM,
        "plan_sha256": plan_sha,
        "sampling_plan_sha256": sampling_sha,
        "sampling_execution_schedule_sha256": EXPECTED_SCHEDULE_SHA256,
        "sampling_execution_schedule_row_count": EXPECTED_SCHEDULE_ROWS,
        "checkpoint_sha256": checkpoint_sha,
        "training_state": EXPECTED_TRAIN_STATE,
        "sampling_stream_samples_seen": EXPECTED_STREAM_SAMPLES,
        "evaluation_sample_count": 665,
        "subgroup_sample_counts": {
            label: count for label, (_, count) in EXPECTED_EPISODES.items()
        },
        "geometry_pass_count": 664,
        "geometry_sample_count": 665,
        "completed_stage_count": len(STAGES),
        "source_archive_file_count": source_count,
        "reference_archive_file_count": reference_count,
        "reference_checkpoint_sha256": reference_checkpoint_shas,
        "png_count": sum(png_counts.values()),
        "png_counts": png_counts,
        "failed_json_count": 0,
        "diagnostic_gate_status": "FAIL_SINGLE_SEED_DIAGNOSTIC",
        "model_performance_gate_passed": False,
        "provenance_verified": True,
        "test_read_or_use": False,
        "closed_loop_evaluation": False,
        "model_exported": False,
        "automatic_promotion": False,
        "training_data_approved": False,
        "vehicle_control_approved": False,
        "deployment_approved": False,
        "torch_imported": False,
        "gpu_used": False,
    }


def _fresh_output(path: Path, campaign_root: Path) -> Path:
    target = Path(path).expanduser().absolute()
    require(".." not in target.parts and not target.exists() and not target.is_symlink(),
            "output JSON must be a fresh create-only path")
    require(not target.is_relative_to(campaign_root),
            "output JSON must remain outside the immutable F campaign root")
    _directory(target.parent, "output JSON parent")
    return target


def write_new(path: Path, value: dict[str, Any], campaign_root: Path) -> None:
    target = _fresh_output(path, campaign_root)
    payload = (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()
    with target.open("xb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())


def execute(campaign_root: Path, output_json: Path) -> dict[str, Any]:
    root = _directory(campaign_root, "F campaign root")
    _fresh_output(output_json, root)
    result = verify(root)
    write_new(output_json, result, root)
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--campaign-root", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        result = execute(args.campaign_root, args.output_json)
    except (VerificationError, OSError, UnicodeError, KeyError, TypeError,
            AttributeError, IndexError) as error:
        print(json.dumps({"status": "PROVENANCE_VERIFICATION_FAILED", "error": str(error)},
                         sort_keys=True, separators=(",", ":")), file=sys.stderr)
        return 2
    print(json.dumps({
        "status": result["status"],
        "output_json": str(Path(args.output_json).expanduser().absolute()),
        "checkpoint_sha256": result["checkpoint_sha256"],
        "diagnostic_gate_status": result["diagnostic_gate_status"],
        "png_count": result["png_count"],
    }, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
