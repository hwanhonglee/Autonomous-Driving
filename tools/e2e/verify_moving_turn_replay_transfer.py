#!/usr/bin/env python3
"""HH_260906 - Verify the copied E turn-replay artifact without Torch, GPU, or gate recomputation."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
from typing import Any


SCHEMA = "portable_e2e.moving_turn_replay_transfer_verification.v1"
PLAN_SCHEMA = "portable_e2e.moving_turn_replay_seed_pilot_plan.v1"
SUMMARY_SCHEMA = "portable_e2e.moving_turn_replay_seed_pilot_summary.v1"
COMPARISON_SCHEMA = "portable_e2e.moving_turn_replay_comparison.v1"
POSTRUN_SCHEMA = "portable_e2e.moving_turn_replay_postrun_verification.v1"
RECOVERY_PLAN_SCHEMA = "portable_e2e.moving_turn_replay_subgroup_recovery_plan.v1"
RECOVERY_SUMMARY_SCHEMA = "portable_e2e.moving_turn_replay_subgroup_recovery_summary.v1"
RECOVERY_ID = "hh260921-moving-turn-replay-subgroup-recovery-v1"
CAMPAIGN_ID = "hh260921-moving-turn-replay-seed20260906-v1"
BASE_COMMIT = "b478f02e42b94bf04bffec5c8170e05edc33b0f8"
ARM = "E_turn_replay_train"
REFERENCE_C_CHECKPOINT_SHA256 = "01b6764df96639336ba00cc11bca67406e1aced3829d471d741a1fe60ed8b061"
EXPECTED_E_CHECKPOINT_SHA256 = "c7739926f25a5c3db7519f33fce236b70d8108e912ec4009bd876afa2a666f78"
EXECUTED_RECOVERY_WRAPPER_SHA256 = "ffe5588a254543418159efe6533098b214d9ea5916fdaaa8a34337350b388f4d"
SUPPLEMENTAL_C_CHECKPOINT = "supplemental/reference_C_checkpoint/latest.pt"
EXECUTED_RECOVERY_WRAPPER = "supplemental/source/run_moving_turn_replay_subgroup_recovery.py"
POSTRUN_VERIFICATION = "derived/postrun_verification.json"
RECOVERY_RESULTS = "recovery_results"
STAGES = ("train", "evaluate", "audit", "subgroup")
STAGE_REPORTS = {
    "train": "training/run.json",
    "evaluate": "evaluation/metrics.json",
    "audit": "gate_v8.json",
    "subgroup": "subgroup/summary.json",
}
RUNNER_ARCHIVE = "source/run_moving_turn_replay_seed_pilot.py"
EVALUATOR_ARCHIVE = "source/evaluate_portable_moving_turn_replay_subgroups.py"
OWNER_ARCHIVE = "source/profile_portable_training.py"
EXPECTED_CURRENT_SOURCE_PATHS = (
    "portable_e2e/__init__.py",
    "portable_e2e/contract.py",
    "portable_e2e/dataset.py",
    "portable_e2e/torch_dataset.py",
    "portable_e2e/model.py",
    "portable_e2e/losses.py",
    "portable_e2e/train.py",
    "portable_e2e/evaluate.py",
    "portable_e2e/audit_runtime.py",
    "portable_e2e/runtime.py",
    "portable_e2e/runtime_contract.py",
    "portable_e2e/runtime_weight_bundle.py",
    "portable_e2e/visualize.py",
    "portable_e2e/stop_primitive_research.py",
    "portable_e2e/config/common_10hz_v1.contract.json",
    "portable_e2e/config/perspective_trajectory_physical_stopmix_v1.model.json",
    "scripts/e2e/audit_portable_stopmix_behavior.py",
    "scripts/e2e/run_portable_training_campaign.py",
)
EXPECTED_ADDED_SOURCE_PATHS = (
    "scripts/e2e/evaluate_portable_moving_data_subgroups.py",
)
EXPECTED_OVERLAY_PATHS = ("portable_e2e/evaluate.py", "portable_e2e/train.py")
RUN_SCOPE_DENIALS = (
    "resume",
    "automatic_retry",
    "test_read_or_use",
    "model_export",
    "automatic_promotion",
    "training_data_approved",
    "vehicle_control_approved",
    "closed_loop_claim",
    "all_map_claim",
    "deployment_approved",
)
COMPARISON_SCOPE_DENIALS = (
    "test_data_read_or_used",
    "closed_loop_evaluation",
    "model_promotion",
    "model_export",
    "training_data_approved",
    "vehicle_control_approved",
    "deployment_approved",
    "all_map_claim",
)
RECOVERY_DENIALS = (
    "automatic_retry",
    "training_executed",
    "evaluate_stage_executed",
    "audit_executed",
    "test_read_or_use",
    "model_export",
    "automatic_promotion",
    "training_data_approved",
    "vehicle_control_approved",
    "closed_loop_claim",
    "all_map_claim",
    "deployment_approved",
)
EXPECTED_FIXED_FILES = {
    "plan.json",
    "started.json",
    "failed.json",
    "view_report.json",
    "reference_C/plan.json",
    "reference_C/summary.json",
    f"reference_C/{'A_baseline_train/training/run.json'}",
    f"reference_C/{'A_baseline_train/subgroup/summary.json'}",
    f"reference_C/{'A_baseline_train/gate_v8.json'}",
    f"{ARM}/.training.portable-e2e.lock",
    f"{ARM}/audit.log",
    f"{ARM}/audit_started.json",
    f"{ARM}/audit_completed.json",
    f"{ARM}/evaluate.log",
    f"{ARM}/evaluate_started.json",
    f"{ARM}/evaluate_completed.json",
    f"{ARM}/subgroup.log",
    f"{ARM}/subgroup_started.json",
    f"{ARM}/train.log",
    f"{ARM}/train_started.json",
    f"{ARM}/train_completed.json",
    f"{ARM}/training/checkpoints/latest.pt",
    f"{ARM}/training/metrics.jsonl",
    f"{ARM}/training/run.json",
    f"{ARM}/evaluation/metrics.json",
    f"{ARM}/gate_v8.json",
}
PNG_DIRECTORIES = {
    f"{ARM}/evaluation/trajectories": 12,
}
RECOVERY_FIXED_FILES = {
    "recovery_plan.json",
    "execution_plan.json",
    "started.json",
    "subgroup_started.json",
    "subgroup.log",
    "completed.json",
    "recovery_summary.json",
    "source/original_runner.py",
    "source/patched_evaluator.py",
    "subgroup/summary.json",
    "subgroup/episodes/town03_right_turn_dev/metrics.json",
    "subgroup/episodes/town03_straight_dev/metrics.json",
}
RECOVERY_PNG_DIRECTORIES = {
    "subgroup/episodes/town03_right_turn_dev/trajectories": 6,
    "subgroup/episodes/town03_straight_dev/trajectories": 6,
}


class VerificationError(ValueError):
    """HH_260906 - Signal a closed verification failure without changing copied evidence."""


def require(value: Any, message: str) -> None:
    if not value:
        raise VerificationError(message)


def _pairs(items: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in items:
        require(key not in result, f"duplicate JSON key: {key!r}")
        result[key] = value
    return result


def _sha_text(value: Any, context: str) -> str:
    require(type(value) is str and re.fullmatch(r"[0-9a-f]{64}", value) is not None,
            f"{context} must be a lowercase SHA-256")
    return value


def _encoded(value: Any) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()


def _safe_relative(value: Any, context: str) -> str:
    require(type(value) is str and value and "\\" not in value, f"{context} must be a POSIX path")
    path = Path(value)
    require(not path.is_absolute() and ".." not in path.parts and path.as_posix() == value and value != ".",
            f"{context} must stay below its declared root")
    return value


def _directory(path: Path, context: str) -> Path:
    lexical = Path(path).expanduser().absolute()
    require(".." not in lexical.parts and lexical.exists(), f"{context} does not exist")
    require(lexical.resolve() == lexical and lexical.is_dir() and not lexical.is_symlink(),
            f"{context} must be a canonical nonsymlink directory")
    return lexical


def _regular(path: Path, context: str) -> Path:
    lexical = Path(path).expanduser().absolute()
    require(".." not in lexical.parts and lexical.exists(), f"{context} does not exist: {lexical}")
    require(lexical.resolve() == lexical and lexical.is_file() and not lexical.is_symlink() and
            lexical.stat().st_nlink == 1, f"{context} must be a canonical single-link regular file")
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


def read_inventory(path: Path) -> dict[str, str]:
    payload = _regular(path, "inventory").read_bytes()
    require(payload.endswith(b"\n"), "inventory must end with one complete line")
    try:
        lines = payload.decode("utf-8").splitlines()
    except UnicodeDecodeError as error:
        raise VerificationError("inventory must be UTF-8") from error
    result: dict[str, str] = {}
    for number, line in enumerate(lines, 1):
        match = re.fullmatch(r"([0-9a-f]{64})  (.+)", line)
        require(match is not None, f"invalid sha256sum inventory line {number}")
        expected, name = match.groups()
        name = _safe_relative(name, f"inventory path at line {number}")
        require(name not in result, f"duplicate inventory path: {name}")
        result[name] = expected
    require(result, "inventory must not be empty")
    return result


def _scan_names(root: Path) -> tuple[set[str], set[str]]:
    names: set[str] = set()
    directory_names: set[str] = set()
    for parent, directories, files in os.walk(root, followlinks=False):
        for name in directories:
            path = Path(parent) / name
            require(not path.is_symlink(), f"symlink directory forbidden: {path}")
            directory_names.add(path.relative_to(root).as_posix())
        for name in files:
            path = Path(parent) / name
            require(not path.is_symlink() and path.is_file() and path.stat().st_nlink == 1,
                    f"non-regular result file forbidden: {path}")
            relative = path.relative_to(root).as_posix()
            require(relative not in names, f"duplicate result path: {relative}")
            names.add(relative)
    return names, directory_names


def verify_inventory(root: Path, inventory: dict[str, str]) -> tuple[int, int]:
    # HH_260906 - Compare names before hashing so an unpinned extra file is never treated as evidence.
    observed, directories = _scan_names(root)
    require(observed == set(inventory), "local result file set differs from the pinned remote inventory")
    expected_directories = {
        parent.as_posix()
        for name in inventory
        for parent in Path(name).parents
        if parent.as_posix() != "."
    }
    require(directories == expected_directories,
            "local result directory set differs from the pinned file-tree closure")
    require({name for name in observed if Path(name).name == "failed.json"} == {"failed.json"},
            "exactly one top-level failed.json must preserve the original run failure")
    total = 0
    for name, expected in inventory.items():
        path = root / name
        require(path.is_relative_to(root), f"inventory entry escaped result root: {name}")
        require(digest(path) == expected, f"local result SHA differs: {name}")
        total += path.stat().st_size
    return len(observed), total


def _false_fields(value: dict[str, Any], fields: tuple[str, ...], context: str) -> None:
    for field in fields:
        require(value.get(field) is False, f"{context}.{field} must be false")


def _option(command: Any, name: str) -> str:
    require(type(command) is list and all(type(item) is str for item in command), "stage command must be strings")
    positions = [index for index, item in enumerate(command) if item == name]
    require(len(positions) == 1 and positions[0] + 1 < len(command), f"stage command option differs: {name}")
    return command[positions[0] + 1]


def _source_files(root: Path, plan: dict[str, Any]) -> set[str]:
    source = plan.get("source")
    require(type(source) is dict and set(source) == {
        "base_commit", "base_sha256", "current_sha256", "overlay_paths", "added_paths", "added_sha256"
    }, "plan source closure differs")
    require(source.get("base_commit") == BASE_COMMIT, "plan source base commit differs")
    base, current, added = source["base_sha256"], source["current_sha256"], source["added_sha256"]
    require(type(base) is dict and type(current) is dict and type(added) is dict,
            "plan source hash maps required")
    require(set(current) == set(EXPECTED_CURRENT_SOURCE_PATHS) and set(base) == set(current),
            "current/base source path closure differs")
    require(set(added) == set(EXPECTED_ADDED_SOURCE_PATHS) and
            source.get("added_paths") == list(EXPECTED_ADDED_SOURCE_PATHS) and
            source.get("overlay_paths") == list(EXPECTED_OVERLAY_PATHS),
            "added/overlay source path closure differs")
    for name in current:
        expected = _sha_text(current[name], f"current source {name}")
        prior = _sha_text(base[name], f"base source {name}")
        require((expected != prior) == (name in EXPECTED_OVERLAY_PATHS),
                f"source overlay byte distinction differs: {name}")
        require(digest(root / "source" / name) == expected, f"archived current source differs: {name}")
    for name in added:
        require(digest(root / "source" / name) == _sha_text(added[name], f"added source {name}"),
                f"archived added source differs: {name}")
    runner_sha = _sha_text(plan.get("runner_sha256"), "plan runner")
    evaluator_sha = _sha_text(plan.get("evaluator_sha256"), "plan evaluator")
    require(digest(root / RUNNER_ARCHIVE) == runner_sha, "archived runner differs from the plan")
    require(digest(root / EVALUATOR_ARCHIVE) == evaluator_sha, "archived evaluator differs from the plan")
    owner = plan.get("owner")
    require(type(owner) is dict and digest(root / OWNER_ARCHIVE) == _sha_text(owner.get("sha256"), "owner"),
            "archived GPU owner differs from the plan")
    require(digest(root / "view_report.json") == _sha_text(plan.get("view_report_sha256"), "view report"),
            "archived view report differs from the plan")
    names = {f"source/{name}" for name in (*EXPECTED_CURRENT_SOURCE_PATHS, *EXPECTED_ADDED_SOURCE_PATHS)}
    names.update({RUNNER_ARCHIVE, EVALUATOR_ARCHIVE, OWNER_ARCHIVE})
    require(len(names) == 22, "source archive count differs")
    return names


def _reference_files(root: Path, plan: dict[str, Any]) -> None:
    reference = plan.get("reference_C")
    require(type(reference) is dict and type(reference.get("files")) is dict,
            "reference C bindings missing")
    expected = {
        "plan": "plan.json",
        "summary": "summary.json",
        "training_run": "A_baseline_train/training/run.json",
        "subgroup": "A_baseline_train/subgroup/summary.json",
        "gate": "A_baseline_train/gate_v8.json",
    }
    require(set(reference["files"]) == set(expected) | {"checkpoint"}, "reference C file binding set differs")
    for label, relative in expected.items():
        row = reference["files"][label]
        require(type(row) is dict and row.get("path") == relative and
                digest(root / "reference_C" / relative) == _sha_text(row.get("sha256"), f"reference C {label}"),
                f"copied reference C binding differs: {label}")
    checkpoint = reference["files"]["checkpoint"]
    require(type(checkpoint) is dict and checkpoint.get("path") ==
            "A_baseline_train/training/checkpoints/latest.pt" and
            checkpoint.get("sha256") == REFERENCE_C_CHECKPOINT_SHA256,
            "reference C checkpoint binding differs")
    _sha_text(checkpoint.get("sha256"), "reference C checkpoint")


def _history(path: Path) -> tuple[str, int]:
    line_count, last = 0, None
    with _regular(path, "training history").open("r", encoding="utf-8") as stream:
        for line in stream:
            line_count += 1
            require(line.endswith("\n"), "training history contains an incomplete line")
            try:
                last = json.loads(line, object_pairs_hook=_pairs,
                                  parse_constant=lambda item: require(False, f"nonfinite history value: {item}"))
            except json.JSONDecodeError as error:
                raise VerificationError(f"invalid training history line {line_count}: {error}") from error
            try:
                json.dumps(last, allow_nan=False)
            except (TypeError, ValueError) as error:
                raise VerificationError(f"nonfinite training history line {line_count}") from error
            require(type(last) is dict and last.get("global_step") == line_count,
                    f"training history step differs at line {line_count}")
    require(line_count == 9020 and last is not None and last.get("samples_seen") == 36040 and
            last.get("domain_samples_seen") == {"carla": 36040} and
            last.get("sampling_stream_samples_seen") == {"base": 29500, "replay": 6540},
            "training history endpoint/count differs")
    return digest(path), line_count


def _png_files(inventory: dict[str, str]) -> tuple[set[str], dict[str, int]]:
    pngs = {name for name in inventory if Path(name).suffix.lower() == ".png"}
    counts: dict[str, int] = {}
    for directory, expected in PNG_DIRECTORIES.items():
        rows = {name for name in pngs if Path(name).parent.as_posix() == directory}
        require(len(rows) == expected and all(re.fullmatch(r"[0-9]{4}_.+\.png", Path(name).name) for name in rows),
                f"PNG count/name differs: {directory}")
        counts[directory] = len(rows)
    require(sum(counts.values()) == len(pngs) == 12, "unexpected original-run PNG path/count")
    return pngs, counts


def _stage_receipts(root: Path, plan: dict[str, Any], failure: dict[str, Any],
                    checkpoint_sha: str, history_sha: str) -> None:
    plan_sha = digest(root / "plan.json")
    completed_stages = STAGES[:3]
    rows = failure.get("stages")
    require(type(rows) is list and [row.get("stage") for row in rows if type(row) is dict] ==
            list(completed_stages), "failed receipt completed-stage order/count differs")
    for stage, completed in zip(completed_stages, rows, strict=True):
        started = read_json(root / ARM / f"{stage}_started.json")
        receipt = read_json(root / ARM / f"{stage}_completed.json")
        require(receipt == completed, f"failed/completed receipt differs: {stage}")
        for key in ("arm", "stage", "command", "timeout_seconds", "started_at_utc", "owner_pid",
                    "plan_sha256", "input_checkpoint_sha256", "pre_cuda_gpu"):
            require(started.get(key) == receipt.get(key), f"started/completed receipt differs: {stage}/{key}")
        require(started.get("status") == "RUNNING" and receipt.get("status") == "COMPLETE" and
                receipt.get("arm") == ARM and receipt.get("stage") == stage and
                receipt.get("plan_sha256") == plan_sha,
                f"stage receipt state differs: {stage}")
        expected_input = None if stage == "train" else checkpoint_sha
        require(receipt.get("input_checkpoint_sha256") == expected_input and
                receipt.get("checkpoint_sha256") == checkpoint_sha,
                f"stage checkpoint binding differs: {stage}")
        relative = f"{ARM}/{STAGE_REPORTS[stage]}"
        require(receipt.get("path") == relative and receipt.get("sha256") == digest(root / relative),
                f"stage report binding differs: {stage}")
        if stage == "train":
            schedule = plan.get("arm", {}).get("execution_schedule", {})
            require(receipt.get("history_sha256") == history_sha and
                    receipt.get("sampling_execution_schedule_row_count") == 36040 and
                    receipt.get("sampling_execution_schedule_sha256") == schedule.get("sha256"),
                    "train history/schedule receipt differs")
    subgroup = read_json(root / ARM / "subgroup_started.json")
    require(subgroup.get("status") == "RUNNING" and subgroup.get("arm") == ARM and
            subgroup.get("stage") == "subgroup" and subgroup.get("plan_sha256") == plan_sha and
            subgroup.get("input_checkpoint_sha256") == checkpoint_sha,
            "failed subgroup start receipt differs")
    command = subgroup.get("command")
    require(_option(command, "--runner-sha256") == plan["runner_sha256"] and
            Path(_option(command, "--runner")).name == Path(RUNNER_ARCHIVE).name and
            _option(command, "--plan-sha256") == plan_sha and
            _option(command, "--checkpoint-sha256") == checkpoint_sha,
            "executed subgroup runner/plan/checkpoint identity differs")


def _reports(root: Path, plan: dict[str, Any], checkpoint_sha: str) -> None:
    train = read_json(root / ARM / "training/run.json")
    evaluation = read_json(root / ARM / "evaluation/metrics.json")
    audit = read_json(root / ARM / "gate_v8.json")
    model_name = _safe_relative(plan.get("model_config"), "plan model config")
    model_path = root / "source" / model_name
    model_config = read_json(model_path)
    require(digest(model_path) == _sha_text(plan.get("model_config_sha256"), "raw model config") and
            hashlib.sha256(json.dumps(model_config, sort_keys=True, separators=(",", ":"),
                                      allow_nan=False).encode()).hexdigest() ==
            _sha_text(plan.get("model_config_canonical_sha256"), "canonical model config"),
            "raw/canonical model config binding differs")
    require(train.get("status") == "TRAINING_TARGET_REACHED" and "resumed_at_utc" not in train and
            train.get("warning") == "RESEARCH CHECKPOINT — NOT APPROVED FOR VEHICLE CONTROL" and
            train.get("model_config") == model_config,
            "fresh non-vehicle training/model report required")
    require(evaluation.get("status") == "OPEN_LOOP_EVALUATION_COMPLETE" and
            evaluation.get("checkpoint_sha256") == checkpoint_sha and
            evaluation.get("vehicle_control_approved") is False and
            evaluation.get("loss_config") == plan.get("loss_config") and
            evaluation.get("auxiliary_loss_metric_counts") == {"candidate_regret_loss": 665},
            "evaluation checkpoint/loss/scope differs")
    require(audit.get("status") == "RUNTIME_GEOMETRY_AUDIT_COMPLETE" and
            audit.get("checkpoint_sha256") == checkpoint_sha and audit.get("vehicle_control_approved") is False,
            "audit checkpoint/scope differs")


def _tree_rows(root: Path) -> tuple[list[dict[str, Any]], set[str]]:
    names, directories = _scan_names(root)
    expected_directories = {
        parent.as_posix()
        for name in names
        for parent in Path(name).parents
        if parent.as_posix() != "."
    }
    require(directories == expected_directories, "recovery directory closure differs")
    rows = [{"path": name, "size": (root / name).stat().st_size, "sha256": digest(root / name)}
            for name in sorted(names)]
    return rows, names


def _recovery_pngs(names: set[str]) -> tuple[set[str], dict[str, int]]:
    pngs = {name for name in names if Path(name).suffix.lower() == ".png"}
    counts: dict[str, int] = {}
    for directory, expected in RECOVERY_PNG_DIRECTORIES.items():
        rows = {name for name in pngs if Path(name).parent.as_posix() == directory}
        require(len(rows) == expected and all(re.fullmatch(r"[0-9]{4}_.+\.png", Path(name).name) for name in rows),
                f"recovery PNG count/name differs: {directory}")
        counts[directory] = len(rows)
    require(sum(counts.values()) == len(pngs) == 12, "unexpected recovery PNG path/count")
    return pngs, counts


def _verify_recovery(artifact: Path, original_root: Path, original_plan: dict[str, Any],
                     original_inventory: dict[str, str], recovery_inventory: dict[str, str],
                     recovery_inventory_sha: str, plan_sha: str, failed_sha: str,
                     checkpoint_sha: str) -> dict[str, Any]:
    root = _directory(artifact / RECOVERY_RESULTS, "post-hoc subgroup recovery results")
    rows, names = _tree_rows(root)
    require(names == set(recovery_inventory) and
            {row["path"]: row["sha256"] for row in rows} == recovery_inventory,
            "local recovery result tree differs from the pinned remote recovery inventory")
    require("failed.json" not in names and "summary.json" not in names,
            "recovery root has a failure or ambiguous legacy summary")
    pngs, png_counts = _recovery_pngs(names)
    require(names == RECOVERY_FIXED_FILES | pngs and len(names) == 24,
            "sealed subgroup recovery file closure/count differs")
    recovery_plan = read_json(root / "recovery_plan.json")
    recovery_plan_sha = digest(root / "recovery_plan.json")
    execution_plan = read_json(root / "execution_plan.json")
    execution_plan_sha = digest(root / "execution_plan.json")
    require(recovery_plan.get("schema") == RECOVERY_PLAN_SCHEMA and
            recovery_plan.get("recovery_id") == RECOVERY_ID and
            recovery_plan.get("output") ==
                "runs/diagnostics/moving_turn_replay_seed20260906_20260921_v1_subgroup_recovery_v1" and
            recovery_plan.get("stage_order") == ["subgroup"] and
            recovery_plan.get("subgroup_evaluation_executed") is True,
            "recovery plan identity/scope differs")
    _false_fields(recovery_plan, RECOVERY_DENIALS, "recovery plan")
    original = recovery_plan.get("original_run")
    require(type(original) is dict and type(original.get("tree")) is dict and
            type(original.get("required")) is dict,
            "recovery original-run binding missing")
    require(original.get("root") == "runs/diagnostics/moving_turn_replay_seed20260906_20260921_v1" and
            original.get("known_failure") == {
                "error_type": "ValueError",
                "error": f"{ARM} subgroup exited 2",
                "subgroup_log_exact":
                    "MOVING_TURN_REPLAY_SUBGROUP_ERROR: checkpoint replay contract/accounting differs\n",
            }, "recovery known original failure differs")
    original_rows = [{"path": name, "size": (original_root / name).stat().st_size,
                      "sha256": original_inventory[name]} for name in sorted(original_inventory)]
    tree = original["tree"]
    require(tree.get("file_count") == 60 and tree.get("files") == original_rows and
            tree.get("total_size_bytes") == sum(row["size"] for row in original_rows) and
            tree.get("inventory_sha256") == hashlib.sha256(_encoded(original_rows)).hexdigest(),
            "recovery plan did not preserve the exact original failed tree")
    required = original["required"]
    required_paths = {
        "plan": "plan.json",
        "failed": "failed.json",
        "subgroup_log": f"{ARM}/subgroup.log",
        "checkpoint": f"{ARM}/training/checkpoints/latest.pt",
        "training_run": f"{ARM}/training/run.json",
        "evaluation": f"{ARM}/evaluation/metrics.json",
        "audit": f"{ARM}/gate_v8.json",
        "runner": RUNNER_ARCHIVE,
        "old_evaluator": EVALUATOR_ARCHIVE,
    }
    require(set(required) == set(required_paths), "recovery required original bindings differ")
    for label, relative in required_paths.items():
        row = required[label]
        require(type(row) is dict and row.get("path") == relative and
                row.get("size") == (original_root / relative).stat().st_size and
                row.get("sha256") == digest(original_root / relative),
                f"recovery required original binding differs: {label}")
    require(required["plan"]["sha256"] == plan_sha and
            required["failed"]["sha256"] == failed_sha and
            required["checkpoint"]["sha256"] == checkpoint_sha and
            required["runner"]["sha256"] == original_plan.get("runner_sha256") and
            required["old_evaluator"]["sha256"] == original_plan.get("evaluator_sha256"),
            "recovery original plan/failure/checkpoint SHA differs")
    execution = recovery_plan.get("execution_plan")
    require(type(execution) is dict and execution.get("path") == "execution_plan.json" and
            execution.get("sha256") == execution_plan_sha and
            execution.get("original_plan_sha256") == plan_sha and
            execution.get("changed_fields") == ["evaluator_sha256"],
            "recovery execution-plan binding differs")
    require(set(execution_plan) == set(original_plan), "recovery execution plan field set differs")
    for key in original_plan:
        if key == "evaluator_sha256":
            require(execution_plan[key] == recovery_plan.get("patched_evaluator", {}).get("sha256") and
                    execution_plan[key] != original_plan[key], "recovery evaluator amendment differs")
        else:
            require(execution_plan[key] == original_plan[key],
                    f"recovery execution plan changed forbidden field: {key}")
    runner = recovery_plan.get("runner")
    patched = recovery_plan.get("patched_evaluator")
    require(type(runner) is dict and runner.get("path") == RUNNER_ARCHIVE and
            runner.get("sha256") == original_plan.get("runner_sha256") and
            digest(root / "source/original_runner.py") == runner["sha256"],
            "recovery archived original runner differs")
    require(type(patched) is dict and patched.get("name") ==
            "evaluate_portable_moving_turn_replay_subgroups.py" and
            digest(root / "source/patched_evaluator.py") == _sha_text(patched.get("sha256"), "patched evaluator"),
            "recovery archived patched evaluator differs")
    fix = recovery_plan.get("allowed_fix")
    require(type(fix) is dict and fix.get("scope") ==
            "subgroup_evaluator_train_config_representation_only" and
            fix.get("old_evaluator_sha256") == original_plan.get("evaluator_sha256") and
            fix.get("patched_evaluator_sha256") == patched["sha256"] and
            fix.get("changed_execution_plan_fields") == ["evaluator_sha256"],
            "recovery allowed-fix boundary differs")
    started = read_json(root / "started.json")
    stage_started = read_json(root / "subgroup_started.json")
    completed = read_json(root / "completed.json")
    recovery_summary = read_json(root / "recovery_summary.json")
    require(started.get("schema") == recovery_summary.get("schema") == RECOVERY_SUMMARY_SCHEMA and
            started.get("status") == "RUNNING" and
            started.get("recovery_plan_sha256") == recovery_plan_sha and
            started.get("execution_plan_sha256") == execution_plan_sha and
            started.get("original_plan_sha256") == plan_sha and
            started.get("checkpoint_sha256") == checkpoint_sha and started.get("stage") == "subgroup",
            "recovery prospective start differs")
    _false_fields(started, RECOVERY_DENIALS, "recovery started")
    require(stage_started.get("status") == "RUNNING" and stage_started.get("stage") == "subgroup" and
            stage_started.get("input_checkpoint_sha256") == checkpoint_sha,
            "recovery subgroup start differs")
    _false_fields(stage_started, RECOVERY_DENIALS, "recovery subgroup started")
    for key in ("stage", "command", "started_at_utc", "timeout_seconds", "input_checkpoint_sha256",
                *RECOVERY_DENIALS):
        require(completed.get(key) == stage_started.get(key), f"recovery receipt differs: {key}")
    subgroup_path = root / "subgroup/summary.json"
    subgroup_sha = digest(subgroup_path)
    require(completed.get("status") == "COMPLETE" and completed.get("path") == "subgroup/summary.json" and
            completed.get("sha256") == subgroup_sha and completed.get("checkpoint_sha256") == checkpoint_sha,
            "recovery completed subgroup proof differs")
    command = completed.get("command")
    require(_option(command, "--runner-sha256") == original_plan["runner_sha256"] and
            Path(_option(command, "--runner")).name == Path(RUNNER_ARCHIVE).name and
            _option(command, "--plan-sha256") == execution_plan_sha and
            _option(command, "--checkpoint-sha256") == checkpoint_sha,
            "recovery executed runner/plan/checkpoint identity differs")
    require(recovery_summary.get("status") == "COMPLETE_NOT_PROMOTED" and
            recovery_summary.get("recovery_plan_sha256") == recovery_plan_sha and
            recovery_summary.get("execution_plan_sha256") == execution_plan_sha and
            recovery_summary.get("original_plan_sha256") == plan_sha and
            recovery_summary.get("original_failed_run_unchanged") is True and
            recovery_summary.get("stage_order") == ["subgroup"] and
            recovery_summary.get("subgroup") == {"path": "subgroup/summary.json",
                "sha256": subgroup_sha, "checkpoint_sha256": checkpoint_sha} and
            recovery_summary.get("subgroup_evaluation_executed") is True,
            "recovery authoritative summary differs")
    _false_fields(recovery_summary, RECOVERY_DENIALS, "recovery summary")
    subgroup = read_json(subgroup_path)
    require(subgroup.get("status") == "COMPLETE_NOT_PROMOTED" and subgroup.get("arm") == ARM and
            subgroup.get("plan_sha256") == execution_plan_sha and
            subgroup.get("runner_sha256") == original_plan["runner_sha256"] and
            subgroup.get("evaluator_sha256") == patched["sha256"] and
            subgroup.get("checkpoint_sha256") == checkpoint_sha and
            subgroup.get("combined_val_count") == 665,
            "recovered subgroup completion/provenance differs")
    _false_fields(subgroup, ("test_read_or_use", "closed_loop_evaluation", "automatic_promotion",
                             "model_exported", "training_executed_by_evaluator",
                             "vehicle_control_approved"), "recovered subgroup")
    episodes = subgroup.get("episodes")
    require(type(episodes) is dict and set(episodes) ==
            {"town03_right_turn_dev", "town03_straight_dev"} and
            episodes["town03_right_turn_dev"].get("count") == 337 and
            episodes["town03_straight_dev"].get("count") == 328,
            "recovered subgroup episode identity/count differs")
    for label, row in episodes.items():
        report = _safe_relative(row.get("report_path"), f"recovery {label} report")
        require(row.get("report_sha256") == digest(root / "subgroup" / report),
                f"recovery {label} report SHA differs")
    require(digest(original_root / f"{ARM}/training/checkpoints/latest.pt") == checkpoint_sha,
            "post-hoc subgroup recovery changed the original checkpoint")
    stable_rows, stable_names = _tree_rows(root)
    require(stable_rows == rows and stable_names == names, "recovery results changed during verification")
    return {
        "root": root,
        "file_count": len(rows),
        "total_size_bytes": sum(row["size"] for row in rows),
        "inventory_sha256": recovery_inventory_sha,
        "tree_sha256": hashlib.sha256(_encoded(rows)).hexdigest(),
        "plan_sha256": recovery_plan_sha,
        "execution_plan_sha256": execution_plan_sha,
        "summary_sha256": digest(root / "recovery_summary.json"),
        "subgroup_sha256": subgroup_sha,
        "patched_evaluator_sha256": patched["sha256"],
        "png_count": len(pngs),
        "png_counts": png_counts,
    }


def _comparison(path: Path, expected_sha: str, root: Path, plan_sha: str,
                failed_sha: str, checkpoint_sha: str, history_sha: str,
                recovery: dict[str, Any]) -> dict[str, Any]:
    require(digest(path) == _sha_text(expected_sha, "external comparison SHA"),
            "independent comparison SHA differs")
    value = read_json(path)
    require(value.get("schema") == COMPARISON_SCHEMA and value.get("status") == "COMPLETE_NOT_PROMOTED",
            "independent comparison completion differs")
    experiment = value.get("inputs", {}).get("E_experiment", {})
    expected_recovery = {
        "plan_sha256": recovery["plan_sha256"],
        "execution_plan_sha256": recovery["execution_plan_sha256"],
        "summary_sha256": recovery["summary_sha256"],
        "subgroup_sha256": recovery["subgroup_sha256"],
    }
    require(experiment.get("campaign_id") == CAMPAIGN_ID and experiment.get("plan_sha256") == plan_sha and
            experiment.get("failed_sha256") == failed_sha and
            experiment.get("original_status") == "FAIL_SUBGROUP" and
            experiment.get("posthoc_subgroup_recovery") == expected_recovery,
            "independent comparison E input binding differs")
    provenance = value.get("arm_provenance", {}).get("E", {})
    require(provenance.get("checkpoint_sha256") == checkpoint_sha and
            provenance.get("history_sha256") == history_sha and
            provenance.get("training_run_sha256") == digest(root / ARM / "training/run.json") and
            provenance.get("evaluation_sha256") == digest(root / ARM / "evaluation/metrics.json") and
            provenance.get("subgroup_sha256") == recovery["subgroup_sha256"] and
            provenance.get("subgroup_source") == "posthoc_recovery" and
            provenance.get("geometry_gate_sha256") == digest(root / ARM / "gate_v8.json"),
            "independent comparison E provenance differs")
    scope = value.get("scope")
    require(type(scope) is dict, "independent comparison scope missing")
    _false_fields(scope, COMPARISON_SCOPE_DENIALS, "comparison.scope")
    require(scope.get("single_seed_only") is True and scope.get("three_seed_confirmation_required") is True,
            "independent comparison single-seed scope differs")
    gate = value.get("preregistered_C_E_gate", {})
    result = gate.get("result", {})
    require(gate.get("runner_summary_exact_match") is False and
            gate.get("posthoc_recovery_subgroup_used") is True and
            result.get("status") in ("PASS_SINGLE_SEED_DIAGNOSTIC", "FAIL_SINGLE_SEED_DIAGNOSTIC") and
            type(result.get("pass")) is bool and
            result.get("pass") == (result.get("status") == "PASS_SINGLE_SEED_DIAGNOSTIC") and
            result.get("adoption_allowed") is False and result.get("automatic_promotion") is False,
            "independent comparison reported gate is malformed")
    # HH_260906 - Preserve the independent report verbatim; transport verification never recomputes model quality.
    return {"status": result["status"], "pass": result["pass"]}


def _postrun(path: Path, *, plan: dict[str, Any], remote_inventory_sha: str,
             plan_sha: str, failed_sha: str, checkpoint_sha: str, file_count: int,
             total_size: int, source_file_count: int, recovery: dict[str, Any],
             executed_recovery_wrapper_sha: str) -> str:
    value = read_json(path)
    require(value.get("schema") == POSTRUN_SCHEMA and
            value.get("status") == "VERIFIED_PARTIAL_FAILURE_WITH_POSTHOC_RECOVERY",
            "postrun verification completion differs")
    require(value.get("remote_inventory_sha256") == remote_inventory_sha and
            value.get("plan_sha256") == plan_sha and value.get("failed_sha256") == failed_sha and
            value.get("runner_sha256") == plan["runner_sha256"] and
            value.get("recovery_plan_sha256") == recovery["plan_sha256"] and
            value.get("recovery_execution_plan_sha256") == recovery["execution_plan_sha256"] and
            value.get("recovery_summary_sha256") == recovery["summary_sha256"] and
            value.get("recovery_subgroup_sha256") == recovery["subgroup_sha256"] and
            value.get("recovery_inventory_sha256") == recovery["inventory_sha256"] and
            value.get("recovery_tree_sha256") == recovery["tree_sha256"] and
            value.get("executed_recovery_wrapper_sha256") == executed_recovery_wrapper_sha and
            value.get("reference_C_checkpoint_supplemental_sha256") == REFERENCE_C_CHECKPOINT_SHA256,
            "postrun input/provenance binding differs")
    require(value.get("original_run_failure_preserved") is True and
            value.get("original_completed_stages") == 3 and
            value.get("recovery_subgroup_pass") is True,
            "postrun original-failure/recovery state differs")
    boolean_fields = (
        "full_tree_verification_pass",
        "source_verification_pass",
        "reference_C_verification_pass",
        "model_config_verification_pass",
        "evaluation_loss_config_verification_pass",
        "candidate_regret_count_verification_pass",
    )
    require(all(value.get(field) is True for field in boolean_fields),
            "postrun verification pass fields differ")
    checks = value.get("checks")
    reference_hashes = {
        label: row["sha256"] for label, row in plan["reference_C"]["files"].items()
    }
    # HH_260906 - Bind each postrun PASS to the exact independently re-observed value.
    expected_checks = {
        "full_tree": {
            "original_file_count": file_count,
            "original_total_size_bytes": total_size,
            "original_inventory_sha256": remote_inventory_sha,
            "recovery_file_count": recovery["file_count"],
            "recovery_total_size_bytes": recovery["total_size_bytes"],
            "recovery_inventory_sha256": recovery["inventory_sha256"],
            "recovery_tree_sha256": recovery["tree_sha256"],
        },
        "source_archive": {
            "source_file_count": source_file_count,
            "runner_sha256": plan["runner_sha256"],
            "original_evaluator_sha256": plan["evaluator_sha256"],
            "patched_evaluator_sha256": recovery["patched_evaluator_sha256"],
        },
        "reference_C": reference_hashes,
        "model_config": {
            "raw_sha256": plan["model_config_sha256"],
            "canonical_sha256": plan["model_config_canonical_sha256"],
            "training_run_matches_source": True,
        },
        "evaluation_loss_config": plan["loss_config"],
        "candidate_regret_count": {"candidate_regret_loss": 665},
        "reference_C_checkpoint_supplemental": REFERENCE_C_CHECKPOINT_SHA256,
        "recovery_inventory": recovery["inventory_sha256"],
        "executed_recovery_wrapper": executed_recovery_wrapper_sha,
        "original_failure_preservation": {
            "status": "FAIL",
            "failed_sha256": failed_sha,
            "completed_stages": 3,
        },
        "recovery_subgroup": {
            "status": "COMPLETE_NOT_PROMOTED",
            "sha256": recovery["subgroup_sha256"],
            "checkpoint_sha256": checkpoint_sha,
            "combined_val_count": 665,
        },
    }
    require(type(checks) is dict and set(checks) == set(expected_checks),
            "postrun detailed check set differs")
    for name, expected in expected_checks.items():
        row = checks[name]
        require(row == {"expected": expected, "observed": expected, "pass": True},
                f"postrun detailed check failed: {name}")
    scope = value.get("scope")
    require(type(scope) is dict, "postrun scope missing")
    _false_fields(scope, COMPARISON_SCOPE_DENIALS, "postrun.scope")
    return digest(path)


def verify(*, artifact_root: Path, remote_inventory_sha256: str,
           recovery_inventory_sha256: str, comparison_sha256: str,
           remote_inventory: Path | None = None, local_inventory: Path | None = None,
           recovery_remote_inventory: Path | None = None,
           recovery_local_inventory: Path | None = None,
           comparison: Path | None = None) -> dict[str, Any]:
    artifact = _directory(artifact_root, "artifact root")
    remote_inventory = _regular(remote_inventory or artifact / "remote_inventory.sha256", "remote inventory")
    local_inventory = _regular(local_inventory or artifact / "local_inventory.sha256", "local inventory")
    recovery_remote_inventory = _regular(
        recovery_remote_inventory or artifact / "recovery_remote_inventory.sha256",
        "remote recovery inventory",
    )
    recovery_local_inventory = _regular(
        recovery_local_inventory or artifact / "recovery_local_inventory.sha256",
        "local recovery inventory",
    )
    comparison = _regular(comparison or artifact / "derived/comparison_C_E.json", "independent comparison")
    require(not remote_inventory.is_relative_to(artifact / "remote_results") and
            not local_inventory.is_relative_to(artifact / "remote_results") and
            not comparison.is_relative_to(artifact / "remote_results"),
            "inventory/comparison must remain outside copied remote results")
    require(not recovery_remote_inventory.is_relative_to(artifact / RECOVERY_RESULTS) and
            not recovery_local_inventory.is_relative_to(artifact / RECOVERY_RESULTS),
            "recovery inventories must remain outside copied recovery results")
    remote_pin = _sha_text(remote_inventory_sha256, "external remote inventory SHA")
    # HH_260906 - Authenticate the remotely captured inventory before any copied run payload is parsed.
    require(digest(remote_inventory) == remote_pin, "external remote inventory SHA differs")
    require(remote_inventory.read_bytes() == local_inventory.read_bytes(),
            "remote/local inventory files are not byte-identical")
    inventory = read_inventory(remote_inventory)
    require(inventory == read_inventory(local_inventory), "remote/local inventory mappings differ")
    recovery_pin = _sha_text(recovery_inventory_sha256, "external recovery inventory SHA")
    require(digest(recovery_remote_inventory) == recovery_pin,
            "external recovery inventory SHA differs")
    require(recovery_remote_inventory.read_bytes() == recovery_local_inventory.read_bytes(),
            "remote/local recovery inventory files are not byte-identical")
    recovery_inventory = read_inventory(recovery_remote_inventory)
    require(recovery_inventory == read_inventory(recovery_local_inventory),
            "remote/local recovery inventory mappings differ")
    results = _directory(artifact / "remote_results", "copied remote results")
    file_count, total_size = verify_inventory(results, inventory)
    plan = read_json(results / "plan.json")
    plan_sha = digest(results / "plan.json")
    failed_sha = digest(results / "failed.json")
    require(plan.get("schema") == PLAN_SCHEMA and plan.get("campaign_id") == CAMPAIGN_ID and
            plan.get("stage_order") == list(STAGES) and plan.get("arm", {}).get("label") == ARM,
            "E plan identity differs")
    _false_fields(plan, RUN_SCOPE_DENIALS, "plan")
    require(plan.get("decision", {}).get("adoption_allowed") is False,
            "plan must deny adoption")
    source_names = _source_files(results, plan)
    _reference_files(results, plan)
    png_names, png_counts = _png_files(inventory)
    expected_names = EXPECTED_FIXED_FILES | source_names | png_names
    require(set(inventory) == expected_names and file_count == len(expected_names) == 60,
            "sealed original failed E result file closure/count differs")
    started = read_json(results / "started.json")
    failure = read_json(results / "failed.json")
    require(started.get("schema") == failure.get("schema") == SUMMARY_SCHEMA and
            started.get("status") == "RUNNING" and started.get("plan_sha256") == plan_sha and
            started.get("arm") == "NOT_RUN" and started.get("stages") == [],
            "prospective start receipt differs")
    require(failure.get("status") == "FAIL" and failure.get("plan_sha256") == plan_sha and
            failure.get("arm") == "FAIL" and failure.get("started_at_utc") == started.get("started_at_utc") and
            failure.get("error_type") == "ValueError" and
            failure.get("error") == f"{ARM} subgroup exited 2",
            "preserved original subgroup failure receipt differs")
    _false_fields(started, RUN_SCOPE_DENIALS, "started")
    _false_fields(failure, RUN_SCOPE_DENIALS, "failed")
    checkpoint = results / ARM / "training/checkpoints/latest.pt"
    checkpoint_sha = digest(checkpoint)
    require(checkpoint_sha == EXPECTED_E_CHECKPOINT_SHA256,
            "original E checkpoint SHA differs from the preserved failed run")
    history_sha, history_rows = _history(results / ARM / "training/metrics.jsonl")
    _stage_receipts(results, plan, failure, checkpoint_sha, history_sha)
    _reports(results, plan, checkpoint_sha)
    recovery = _verify_recovery(artifact, results, plan, inventory, recovery_inventory,
                                recovery_pin, plan_sha, failed_sha, checkpoint_sha)
    supplemental = _regular(artifact / SUPPLEMENTAL_C_CHECKPOINT, "supplemental C checkpoint")
    require(not supplemental.is_relative_to(results) and digest(supplemental) == REFERENCE_C_CHECKPOINT_SHA256,
            "supplemental C checkpoint SHA differs")
    executed_wrapper = _regular(artifact / EXECUTED_RECOVERY_WRAPPER,
                                "supplemental executed recovery wrapper")
    require(not executed_wrapper.is_relative_to(results) and
            not executed_wrapper.is_relative_to(recovery["root"]) and
            digest(executed_wrapper) == EXECUTED_RECOVERY_WRAPPER_SHA256,
            "supplemental executed recovery wrapper SHA differs")
    postrun_path = _regular(artifact / POSTRUN_VERIFICATION, "postrun verification")
    postrun_sha = _postrun(postrun_path, plan=plan,
                           remote_inventory_sha=remote_pin, plan_sha=plan_sha,
                           failed_sha=failed_sha, checkpoint_sha=checkpoint_sha,
                           file_count=file_count, total_size=total_size,
                           source_file_count=len(source_names), recovery=recovery,
                           executed_recovery_wrapper_sha=EXECUTED_RECOVERY_WRAPPER_SHA256)
    gate = _comparison(comparison, comparison_sha256, results, plan_sha,
                       failed_sha, checkpoint_sha, history_sha, recovery)
    stable_count, stable_size = verify_inventory(results, inventory)
    stable_recovery_rows, _ = _tree_rows(recovery["root"])
    require((stable_count, stable_size) == (file_count, total_size) and
            digest(remote_inventory) == remote_pin and
            remote_inventory.read_bytes() == local_inventory.read_bytes() and
            digest(recovery_remote_inventory) == recovery_pin and
            recovery_remote_inventory.read_bytes() == recovery_local_inventory.read_bytes() and
            digest(results / "plan.json") == plan_sha and
            digest(results / "failed.json") == failed_sha and
            digest(supplemental) == REFERENCE_C_CHECKPOINT_SHA256 and
            digest(executed_wrapper) == EXECUTED_RECOVERY_WRAPPER_SHA256 and
            hashlib.sha256(_encoded(stable_recovery_rows)).hexdigest() == recovery["tree_sha256"] and
            digest(postrun_path) == postrun_sha and digest(comparison) == comparison_sha256,
            "artifact or verification inputs changed during verification")
    return {
        "schema": SCHEMA,
        "status": "PASS_EXACT_FAILED_TRANSFER_WITH_POSTHOC_RECOVERY_NOT_PROMOTED",
        "verified_at_utc": datetime.now(timezone.utc).isoformat(),
        "artifact_root": str(artifact),
        "remote_results": str(results),
        "remote_inventory_sha256": remote_pin,
        "local_inventory_sha256": digest(local_inventory),
        "inventories_byte_identical": True,
        "recovery_remote_inventory_sha256": recovery_pin,
        "recovery_local_inventory_sha256": digest(recovery_local_inventory),
        "recovery_inventories_byte_identical": True,
        "file_count": file_count,
        "total_size_bytes": total_size,
        "source_file_count": len(source_names),
        "png_count": len(png_names),
        "png_counts": png_counts,
        "plan_sha256": plan_sha,
        "original_failed_sha256": failed_sha,
        "runner_sha256": plan["runner_sha256"],
        "checkpoint_sha256": checkpoint_sha,
        "history_sha256": history_sha,
        "history_rows": history_rows,
        "reference_C_checkpoint_supplemental_sha256": REFERENCE_C_CHECKPOINT_SHA256,
        "executed_recovery_wrapper_sha256": EXECUTED_RECOVERY_WRAPPER_SHA256,
        "postrun_verification_sha256": postrun_sha,
        "recovery": {key: value for key, value in recovery.items() if key != "root"},
        "original_completed_stage_count": 3,
        "original_failed_receipt_preserved": True,
        "comparison_sha256": digest(comparison),
        "diagnostic_gate_as_reported_by_independent_comparison": gate,
        "diagnostic_gate_recomputed_by_transfer_verifier": False,
        "checkpoint_payload_deserialized": False,
        "torch_imported": False,
        "gpu_used": False,
        "test_read_or_use": False,
        "closed_loop_evaluation": False,
        "automatic_promotion": False,
        "model_export": False,
        "training_data_approved": False,
        "vehicle_control_approved": False,
        "all_map_claim": False,
        "deployment_approved": False,
    }


def write_new(path: Path, value: dict[str, Any], results: Path) -> None:
    target = Path(path).expanduser().absolute()
    require(".." not in target.parts and not target.exists() and not target.is_symlink(),
            "verification output must be fresh")
    require(not target.is_relative_to(results), "verification output must remain outside copied remote results")
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
    parser.add_argument("--recovery-inventory-sha256", required=True)
    parser.add_argument("--comparison-sha256", required=True)
    parser.add_argument("--remote-inventory", type=Path)
    parser.add_argument("--local-inventory", type=Path)
    parser.add_argument("--recovery-remote-inventory", type=Path)
    parser.add_argument("--recovery-local-inventory", type=Path)
    parser.add_argument("--comparison", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    result = verify(
        artifact_root=args.artifact_root,
        remote_inventory_sha256=args.remote_inventory_sha256,
        recovery_inventory_sha256=args.recovery_inventory_sha256,
        comparison_sha256=args.comparison_sha256,
        remote_inventory=args.remote_inventory,
        local_inventory=args.local_inventory,
        recovery_remote_inventory=args.recovery_remote_inventory,
        recovery_local_inventory=args.recovery_local_inventory,
        comparison=args.comparison,
    )
    write_new(args.output, result, _directory(args.artifact_root / "remote_results", "copied remote results"))
    print(json.dumps({"status": result["status"], "output": str(args.output),
                      "file_count": result["file_count"], "gate_recomputed": False}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
