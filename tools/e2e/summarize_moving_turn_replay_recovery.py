#!/usr/bin/env python3
"""HH_260906 - Verify C versus failed-E plus its subgroup-only recovery."""
from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import tempfile
from typing import Any

try:
    from scripts.e2e import summarize_moving_turn_replay as base
except ModuleNotFoundError:  # pragma: no cover - supports direct script execution.
    import summarize_moving_turn_replay as base  # type: ignore[no-redef]


SCHEMA = base.SCHEMA
RECOVERY_PLAN_SCHEMA = "portable_e2e.moving_turn_replay_subgroup_recovery_plan.v1"
RECOVERY_SUMMARY_SCHEMA = "portable_e2e.moving_turn_replay_subgroup_recovery_summary.v1"
RECOVERY_ID = "hh260921-moving-turn-replay-subgroup-recovery-v1"
ARM = base.EXPERIMENT_ARM
GPU = {"physical_index": 0, "uuid": "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"}

ORIGINAL_PLAN_SHA256 = "fda9836d51538ad66e51fb4250e53ef94aadf23b90bd77525bb38edbe811d906"
ORIGINAL_FAILED_SHA256 = "12b1d98ccfe2849734e38d8c9b3598eac24ffb0ff29fbd7447a8f04a2de79208"
ORIGINAL_CHECKPOINT_SHA256 = "c7739926f25a5c3db7519f33fce236b70d8108e912ec4009bd876afa2a666f78"
ORIGINAL_TRAIN_SHA256 = "790d276f0b123431b0b092427a4969a813b7659b1b9abdbc4a90b2934ee930dc"
ORIGINAL_HISTORY_SHA256 = "a04d26398ca610fadd0e38d72495dd68f60591e4501f08a10ba15b2f8b913d9a"
ORIGINAL_EVALUATION_SHA256 = "d3cb56647711df0f3a69a365a97188e1ae2dc44c5e3933b4a894df48c6d82a1e"
ORIGINAL_AUDIT_SHA256 = "dad62b58793a038dda3ae32a583a89ecf29cff6c48eaeebc6b8c3554a0938caf"
ORIGINAL_SCHEDULE_SHA256 = "33d9318687c150e590d98e6228c7a9608284f65cb2d6a1f8c597685fbae44861"
ORIGINAL_TREE_FILE_COUNT = 60
ORIGINAL_TREE_TOTAL_SIZE = 51_406_952
ORIGINAL_TREE_INVENTORY_SHA256 = "7010bccf0a0884b0b1a11e50350dfd44fdcb8dcd53c372bae264a67ec0cba116"

RECOVERY_PLAN_SHA256 = "6e6c02b2c9d0918e3519f2491556b8e65e59eaa5ce174e2bed5981bc2ca77faf"
RECOVERY_EXECUTION_PLAN_SHA256 = "96fc788de2da6acb88f142c565bfff66b09f24210065c9c458bcead7d74dc2dd"
RECOVERY_SUMMARY_SHA256 = "5b6aeaedff116c5aea7edfa363d4b531d513756d5e94192a14660eb2373af6ab"
RECOVERY_SUBGROUP_SHA256 = "86527ca4cd6776b5b69d444437c0d29a0fa15a9e5375860db762b63138beb9e8"
RECOVERY_COMPLETED_SHA256 = "45c65514df2af0739349ce9b39fd9957f1815b1af67c94eb2e04b9b0230f4396"
RECOVERY_FILE_COUNT = 24
RECOVERY_TOTAL_SIZE = 10_356_238

OLD_EVALUATOR_SHA256 = base.EXPECTED_EVALUATOR_SHA256
PATCHED_EVALUATOR_SHA256 = "4720a9207d588c91ad1704defef21d4e07bb0d5f81781ffbf597414287af5962"
EXECUTED_RECOVERY_WRAPPER_SHA256 = "ffe5588a254543418159efe6533098b214d9ea5916fdaaa8a34337350b388f4d"
RUNNER_SHA256 = base.EXPECTED_RUNNER_SHA256

REFERENCE_PLAN_SHA256 = "dda31c7ff03b959575d8483bfed8506bbf4e2c6c5bb4597c6285517a3c991cdc"
REFERENCE_SUMMARY_SHA256 = "12c34fde7c64bd47767191139a729b11e625538065f0e7fbb2d0230e4a46b369"
REFERENCE_CHECKPOINT_SHA256 = "01b6764df96639336ba00cc11bca67406e1aced3829d471d741a1fe60ed8b061"
REFERENCE_TRAIN_SHA256 = "3b566482cce05774e428529e8e2990dabc48195ac37fdfa3844072e194fecb6c"
REFERENCE_SUBGROUP_SHA256 = "f79f2ff7d689af978d7fcb34c72cbcce24262ff12e32cb6b53174f0c3920d925"
REFERENCE_AUDIT_SHA256 = "35a5c01e654232ae27534683e102ca4cbe443d84439e01e36458025de346ff37"

KNOWN_ERROR = {"error_type": "ValueError", "error": f"{ARM} subgroup exited 2"}
KNOWN_LOG = "MOVING_TURN_REPLAY_SUBGROUP_ERROR: checkpoint replay contract/accounting differs\n"
EXPECTED_MODEL_CONFIG = "portable_e2e/config/perspective_trajectory_physical_stopmix_v1.model.json"
EXPECTED_MODEL_CONFIG_CANONICAL_SHA256 = (
    "c17b49ea680b5cae0d9fc35dc6c457af8cc32d1b7171863534f70e9b158cfb7a"
)
EXPECTED_LOSS_CONFIG = {
    "candidate_regret_weight": 0.1,
    "candidate_score_weight": 0.1,
    "final_displacement_weight": 0.5,
    "kinematic_speed_weight": 0.05,
    "speed_weight": 0.2,
    "xy_weight": 1.0,
    "yaw_weight": 0.1,
}
RECOVERY_DENIAL_FIELDS = (
    "automatic_retry", "training_executed", "evaluate_stage_executed", "audit_executed",
    "test_read_or_use", "model_export", "automatic_promotion", "training_data_approved",
    "vehicle_control_approved", "closed_loop_claim", "all_map_claim", "deployment_approved",
)


def _encoded(value: Any) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode("utf-8")


def _write_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(_encoded(value))


def _inventory(root: Path) -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    total = 0
    for parent, directories, files in os.walk(root, followlinks=False):
        for name in (*directories, *files):
            base.require(not (Path(parent) / name).is_symlink(), "evidence tree contains a symlink")
        for name in files:
            path = Path(parent) / name
            size = path.stat().st_size
            total += size
            rows.append({
                "path": path.relative_to(root).as_posix(),
                "size": size,
                "sha256": base.digest(path),
            })
    rows.sort(key=lambda row: row["path"])
    return {
        "file_count": len(rows),
        "total_size_bytes": total,
        "files": rows,
        "inventory_sha256": hashlib.sha256(_encoded(rows)).hexdigest(),
    }


def _require_hash(path: Path, expected: str, context: str) -> None:
    base.require(base.digest(path) == expected, f"{context} SHA256 differs")


def _gpu_idle(value: Any, context: str) -> None:
    row = base._mapping(value, context)
    base.require(row == {**GPU, "pre_cuda_compute_processes": 0}, f"{context} differs")


def _recovery_denials(value: dict[str, Any], context: str) -> None:
    for field in RECOVERY_DENIAL_FIELDS:
        base.require(value.get(field) is False, f"{context}.{field} must be false")


def _command_value(command: list[Any], option: str, context: str) -> str:
    base.require(command.count(option) == 1, f"{context} must contain one {option}")
    offset = command.index(option)
    base.require(offset + 1 < len(command) and type(command[offset + 1]) is str,
                 f"{context} {option} value differs")
    return command[offset + 1]


def _verify_evaluation_contract(plan: dict[str, Any], experiment_root: Path) -> dict[str, Any]:
    item = experiment_root / ARM
    train = base.read_json(item / "training/run.json")
    evaluation = base.read_json(item / "evaluation/metrics.json")
    base.require(plan.get("model_config") == EXPECTED_MODEL_CONFIG and
                 plan.get("model_config_canonical_sha256") == EXPECTED_MODEL_CONFIG_CANONICAL_SHA256,
                 "E model config declaration differs")
    base.require(plan.get("loss_config") == EXPECTED_LOSS_CONFIG and
                 train.get("loss_config") == EXPECTED_LOSS_CONFIG and
                 evaluation.get("loss_config") == EXPECTED_LOSS_CONFIG,
                 "E training/evaluation loss config differs")
    auxiliary = base._mapping(evaluation.get("auxiliary_loss_metrics"),
                              "E evaluation auxiliary loss metrics")
    counts = base._mapping(evaluation.get("auxiliary_loss_metric_counts"),
                           "E evaluation auxiliary loss counts")
    base.require(set(auxiliary) == set(counts) == {"candidate_regret_loss"} and
                 counts["candidate_regret_loss"] == 665 and
                 math.isfinite(base._number(auxiliary["candidate_regret_loss"],
                                             "candidate_regret_loss")),
                 "E candidate-regret evaluation accounting differs")
    return {
        "model_config_path": EXPECTED_MODEL_CONFIG,
        "model_config_canonical_sha256": EXPECTED_MODEL_CONFIG_CANONICAL_SHA256,
        "loss_config": EXPECTED_LOSS_CONFIG,
        "candidate_regret_loss": auxiliary["candidate_regret_loss"],
        "candidate_regret_count": counts["candidate_regret_loss"],
        "pass": True,
    }


def _verify_original(experiment_root: Path) -> tuple[dict[str, Any], dict[str, Any], list[dict[str, Any]]]:
    plan_path = experiment_root / "plan.json"
    failed_path = experiment_root / "failed.json"
    _require_hash(plan_path, ORIGINAL_PLAN_SHA256, "original E plan")
    _require_hash(failed_path, ORIGINAL_FAILED_SHA256, "original E failed receipt")
    plan = base.read_json(plan_path)
    failed = base.read_json(failed_path)
    base.require(plan.get("schema") == base.EXPERIMENT_PLAN_SCHEMA and
                 plan.get("campaign_id") == base.EXPERIMENT_CAMPAIGN and
                 plan.get("runner_sha256") == RUNNER_SHA256 and
                 plan.get("evaluator_sha256") == OLD_EVALUATOR_SHA256 and
                 plan.get("gpu") == GPU,
                 "original E plan identity differs")
    base._verify_denials(plan, "original E plan")
    base._verify_denials(failed, "original E failure")
    base.require(failed.get("schema") == base.EXPERIMENT_SUMMARY_SCHEMA and
                 failed.get("status") == "FAIL" and failed.get("arm") == "FAIL" and
                 failed.get("plan_sha256") == ORIGINAL_PLAN_SHA256 and
                 {key: failed.get(key) for key in KNOWN_ERROR} == KNOWN_ERROR,
                 "original E failure is not the exact subgroup-only failure")
    _gpu_idle(failed.get("pre_cuda_gpu"), "original E pre-CUDA GPU")
    stages = base._list(failed.get("stages"), "original E failed stages")
    base.require([row.get("stage") for row in stages] == ["train", "evaluate", "audit"],
                 "original E must contain exactly the completed 3/4 prefix")
    expected_files = {
        "train": ("training/run.json", ORIGINAL_TRAIN_SHA256),
        "evaluate": ("evaluation/metrics.json", ORIGINAL_EVALUATION_SHA256),
        "audit": ("gate_v8.json", ORIGINAL_AUDIT_SHA256),
    }
    for raw in stages:
        row = base._mapping(raw, "original E failed stage")
        stage = row["stage"]
        relative, expected_sha = expected_files[stage]
        base.require(row.get("arm") == ARM and row.get("status") == "COMPLETE" and
                     row.get("plan_sha256") == ORIGINAL_PLAN_SHA256 and
                     row.get("path") == f"{ARM}/{relative}" and
                     row.get("sha256") == expected_sha and
                     row.get("checkpoint_sha256") == ORIGINAL_CHECKPOINT_SHA256 and
                     base.read_json(experiment_root / ARM / f"{stage}_completed.json") == row,
                     f"original E {stage} receipt differs")
        _require_hash(experiment_root / ARM / relative, expected_sha, f"original E {stage}")
    train_receipt = stages[0]
    base.require(train_receipt.get("history_sha256") == ORIGINAL_HISTORY_SHA256 and
                 train_receipt.get("sampling_execution_schedule_sha256") == ORIGINAL_SCHEDULE_SHA256 and
                 train_receipt.get("sampling_execution_schedule_row_count") == 36_040,
                 "original E training schedule receipt differs")
    _require_hash(experiment_root / ARM / "training/metrics.jsonl", ORIGINAL_HISTORY_SHA256,
                  "original E training history")
    _require_hash(experiment_root / ARM / "training/checkpoints/latest.pt",
                  ORIGINAL_CHECKPOINT_SHA256, "original E checkpoint")
    _require_hash(experiment_root / "source/run_moving_turn_replay_seed_pilot.py",
                  RUNNER_SHA256, "original E runner")
    _require_hash(experiment_root / "source/evaluate_portable_moving_turn_replay_subgroups.py",
                  OLD_EVALUATOR_SHA256, "original E evaluator")
    forbidden = ("summary.json", f"{ARM}/subgroup/summary.json", f"{ARM}/subgroup_completed.json")
    for relative in forbidden:
        base.require(not (experiment_root / relative).exists(),
                     f"original failed E has impossible completion evidence: {relative}")
    started = base.read_json(experiment_root / ARM / "subgroup_started.json")
    base.require(started.get("status") == "RUNNING" and started.get("stage") == "subgroup" and
                 started.get("input_checkpoint_sha256") == ORIGINAL_CHECKPOINT_SHA256,
                 "original E subgroup-start receipt differs")
    log_path = experiment_root / ARM / "subgroup.log"
    base.require(log_path.read_text(encoding="utf-8") == KNOWN_LOG,
                 "original E subgroup failure log differs")
    contract = _verify_evaluation_contract(plan, experiment_root)
    return plan, contract, stages


def _only_evaluator_changed(original: dict[str, Any], execution: dict[str, Any]) -> None:
    base.require(set(original) == set(execution), "recovery execution plan fields differ")
    for key, value in original.items():
        if key == "evaluator_sha256":
            base.require(value == OLD_EVALUATOR_SHA256 and
                         execution.get(key) == PATCHED_EVALUATOR_SHA256,
                         "recovery evaluator amendment differs")
        else:
            base.require(execution.get(key) == value,
                         f"recovery changed forbidden execution-plan field: {key}")


def _subgroup_metrics(subgroup: dict[str, Any], subgroup_root: Path,
                      context: str) -> dict[str, dict[str, float | int]]:
    metrics: dict[str, dict[str, float | int]] = {}
    rows = base._episode_rows(subgroup, context)
    for spec in base.EPISODES:
        row = rows[spec["label"]]
        base.require(row.get("count") == spec["count"], f"{context} count differs")
        values = base._finite_metrics(row.get("metrics"), row.get("metric_counts"), spec["count"],
                                      f"{context} {spec['label']}")
        report = row.get("report_path")
        base.require(type(report) is str and report and not Path(report).is_absolute() and
                     ".." not in Path(report).parts,
                     f"{context} report path differs")
        report_path = subgroup_root / report
        base.require(row.get("report_sha256") == base.digest(report_path),
                     f"{context} report SHA differs")
        report_json = base.read_json(report_path)
        base.require(report_json.get("sample_count") == spec["count"] and
                     report_json.get("metrics") == row.get("metrics"),
                     f"{context} report content differs")
        pngs = base._mapping(row.get("trajectory_png_sha256"), f"{context} trajectory PNGs")
        base.require(len(pngs) == 6, f"{context} trajectory PNG count differs")
        for name, expected in pngs.items():
            base.require(type(name) is str and Path(name).name == name and
                         expected == base.digest(subgroup_root / "episodes" / spec["label"] /
                                                 "trajectories" / name),
                         f"{context} trajectory PNG differs: {name}")
        metrics[spec["label"]] = values
    return metrics


def _verify_recovery(recovery_root: Path, experiment_root: Path,
                     original_plan: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    files = {
        "plan": ("recovery_plan.json", RECOVERY_PLAN_SHA256),
        "execution": ("execution_plan.json", RECOVERY_EXECUTION_PLAN_SHA256),
        "summary": ("recovery_summary.json", RECOVERY_SUMMARY_SHA256),
        "completed": ("completed.json", RECOVERY_COMPLETED_SHA256),
        "subgroup": ("subgroup/summary.json", RECOVERY_SUBGROUP_SHA256),
    }
    for context, (relative, expected) in files.items():
        _require_hash(recovery_root / relative, expected, f"recovery {context}")
    base.require(not (recovery_root / "failed.json").exists(), "recovery contains failed.json")
    recovery_tree = _inventory(recovery_root)
    base.require(recovery_tree["file_count"] == RECOVERY_FILE_COUNT and
                 recovery_tree["total_size_bytes"] == RECOVERY_TOTAL_SIZE,
                 "recovery tree size/count differs")
    plan = base.read_json(recovery_root / "recovery_plan.json")
    execution = base.read_json(recovery_root / "execution_plan.json")
    summary = base.read_json(recovery_root / "recovery_summary.json")
    completed = base.read_json(recovery_root / "completed.json")
    started = base.read_json(recovery_root / "started.json")
    subgroup_started = base.read_json(recovery_root / "subgroup_started.json")
    subgroup = base.read_json(recovery_root / "subgroup/summary.json")
    base.require(plan.get("schema") == RECOVERY_PLAN_SCHEMA and
                 plan.get("recovery_id") == RECOVERY_ID and plan.get("gpu") == GPU and
                 plan.get("stage_order") == ["subgroup"] and
                 plan.get("subgroup_evaluation_executed") is True,
                 "recovery plan identity/scope differs")
    _recovery_denials(plan, "recovery plan")
    binding = base._mapping(plan.get("execution_plan"), "recovery execution-plan binding")
    base.require(binding == {
        "path": "execution_plan.json",
        "sha256": RECOVERY_EXECUTION_PLAN_SHA256,
        "original_plan_sha256": ORIGINAL_PLAN_SHA256,
        "changed_fields": ["evaluator_sha256"],
    }, "recovery execution-plan binding differs")
    fix = base._mapping(plan.get("allowed_fix"), "recovery allowed fix")
    base.require(fix.get("scope") == "subgroup_evaluator_train_config_representation_only" and
                 fix.get("changed_execution_plan_fields") == ["evaluator_sha256"] and
                 fix.get("old_evaluator_sha256") == OLD_EVALUATOR_SHA256 and
                 fix.get("patched_evaluator_sha256") == PATCHED_EVALUATOR_SHA256 and
                 "tuple" in fix.get("reason", "") and "list" in fix.get("reason", ""),
                 "recovery allowed-fix declaration differs")
    _only_evaluator_changed(original_plan, execution)
    _require_hash(recovery_root / "source/original_runner.py", RUNNER_SHA256,
                  "recovery archived original runner")
    _require_hash(recovery_root / "source/patched_evaluator.py", PATCHED_EVALUATOR_SHA256,
                  "recovery archived patched evaluator")
    original_tree = base._mapping(plan.get("original_run"), "recovery original-run binding").get("tree")
    observed_original_tree = _inventory(experiment_root)
    base.require(original_tree == observed_original_tree and
                 observed_original_tree["file_count"] == ORIGINAL_TREE_FILE_COUNT and
                 observed_original_tree["total_size_bytes"] == ORIGINAL_TREE_TOTAL_SIZE and
                 observed_original_tree["inventory_sha256"] == ORIGINAL_TREE_INVENTORY_SHA256,
                 "original failed E full tree changed or differs from the sealed recovery plan")
    required = base._mapping(plan["original_run"].get("required"), "recovery required evidence")
    for name, row_value in required.items():
        row = base._mapping(row_value, f"recovery required evidence {name}")
        relative = row.get("path")
        base.require(type(relative) is str and not Path(relative).is_absolute() and
                     ".." not in Path(relative).parts and
                     row.get("sha256") == base.digest(experiment_root / relative) and
                     row.get("size") == (experiment_root / relative).stat().st_size,
                     f"recovery required original binding differs: {name}")
    for value, context in ((started, "recovery start"), (summary, "recovery summary")):
        _recovery_denials(value, context)
        base.require(value.get("recovery_plan_sha256") == RECOVERY_PLAN_SHA256 and
                     value.get("execution_plan_sha256") == RECOVERY_EXECUTION_PLAN_SHA256 and
                     value.get("original_plan_sha256") == ORIGINAL_PLAN_SHA256,
                     f"{context} plan binding differs")
    base.require(started.get("schema") == RECOVERY_SUMMARY_SCHEMA and
                 started.get("status") == "RUNNING" and started.get("stage") == "subgroup" and
                 started.get("checkpoint_sha256") == ORIGINAL_CHECKPOINT_SHA256,
                 "recovery start receipt differs")
    _gpu_idle(started.get("pre_cuda_gpu"), "recovery pre-CUDA GPU")
    base.require(summary.get("schema") == RECOVERY_SUMMARY_SCHEMA and
                 summary.get("status") == "COMPLETE_NOT_PROMOTED" and
                 summary.get("original_failed_run_unchanged") is True and
                 summary.get("stage_order") == ["subgroup"] and
                 summary.get("subgroup_evaluation_executed") is True and
                 summary.get("subgroup") == {
                     "path": "subgroup/summary.json",
                     "sha256": RECOVERY_SUBGROUP_SHA256,
                     "checkpoint_sha256": ORIGINAL_CHECKPOINT_SHA256,
                 }, "recovery summary differs")
    _gpu_idle(summary.get("final_gpu"), "recovery final GPU")
    for value, context, status in ((subgroup_started, "recovery subgroup start", "RUNNING"),
                                   (completed, "recovery completion", "COMPLETE")):
        _recovery_denials(value, context)
        base.require(value.get("stage") == "subgroup" and value.get("status") == status and
                     value.get("input_checkpoint_sha256") == ORIGINAL_CHECKPOINT_SHA256,
                     f"{context} differs")
        command = base._list(value.get("command"), f"{context} command")
        base.require(_command_value(command, "--runner-sha256", context) == RUNNER_SHA256 and
                     _command_value(command, "--plan-sha256", context) == RECOVERY_EXECUTION_PLAN_SHA256 and
                     _command_value(command, "--checkpoint-sha256", context) == ORIGINAL_CHECKPOINT_SHA256 and
                     all(token not in command for token in
                         ("portable_e2e.train", "portable_e2e.evaluate",
                          "portable_e2e.audit_runtime", "--resume")),
                     f"{context} escaped subgroup-only scope")
    base.require(subgroup_started.get("command") == completed.get("command") and
                 completed.get("path") == "subgroup/summary.json" and
                 completed.get("sha256") == RECOVERY_SUBGROUP_SHA256 and
                 completed.get("checkpoint_sha256") == ORIGINAL_CHECKPOINT_SHA256,
                 "recovery completion evidence differs")
    base.require(subgroup.get("schema") == base.EXPERIMENT_SUBGROUP_SCHEMA and
                 subgroup.get("status") == "COMPLETE_NOT_PROMOTED" and subgroup.get("arm") == ARM and
                 subgroup.get("plan_sha256") == RECOVERY_EXECUTION_PLAN_SHA256 and
                 subgroup.get("runner_sha256") == RUNNER_SHA256 and
                 subgroup.get("evaluator_sha256") == PATCHED_EVALUATOR_SHA256 and
                 subgroup.get("checkpoint_sha256") == ORIGINAL_CHECKPOINT_SHA256 and
                 subgroup.get("combined_val_count") == 665 and
                 subgroup.get("source") == execution.get("source") and
                 subgroup.get("base_commit") == execution.get("base_commit") and
                 subgroup.get("test_read_or_use") is False and
                 subgroup.get("training_executed_by_evaluator") is False and
                 subgroup.get("automatic_promotion") is False and
                 subgroup.get("model_exported") is False and
                 subgroup.get("vehicle_control_approved") is False and
                 subgroup.get("closed_loop_evaluation") is False,
                 "recovered subgroup provenance/scope differs")
    arm_plan = base._mapping(execution.get("arm"), "recovery E arm plan")
    data = base._dataset(arm_plan.get("dataset"), 1475, "recovery E dataset")
    subgroup_data = base._mapping(subgroup.get("dataset"), "recovered subgroup dataset")
    base.require(subgroup_data.get("combined_val_example_fingerprint_sha256") ==
                 data["split_examples_sha256"]["val"] and
                 subgroup_data.get("combined_val_fingerprint_sha256") ==
                 data["split_fingerprint_sha256"]["val"] and
                 subgroup_data.get("train_fingerprint_sha256") ==
                 data["split_fingerprint_sha256"]["train"] and
                 subgroup_data.get("manifest_sha256") == data["manifest_sha256"] and
                 subgroup_data.get("counts") == data["counts"],
                 "recovered subgroup dataset differs")
    checkpoint = base._mapping(subgroup.get("checkpoint"), "recovered subgroup checkpoint")
    base.require(checkpoint.get("sha256") == ORIGINAL_CHECKPOINT_SHA256 and
                 checkpoint.get("model_config_sha256") == EXPECTED_MODEL_CONFIG_CANONICAL_SHA256 and
                 checkpoint.get("sampling_plan_sha256") == arm_plan.get("sampling_plan_sha256") and
                 checkpoint.get("training_dataset_fingerprint_sha256") ==
                 data["split_fingerprint_sha256"]["train"] and
                 checkpoint.get("corpus_fingerprint_sha256") == data["corpus_fingerprint_sha256"],
                 "recovered subgroup checkpoint contract differs")
    episode_metrics = _subgroup_metrics(subgroup, recovery_root / "subgroup", "recovered subgroup")
    evaluation = base.read_json(experiment_root / ARM / "evaluation/metrics.json")
    combined = base._finite_metrics(evaluation.get("metrics"), evaluation.get("metric_counts"),
                                    665, "original E combined VAL")
    base.require(set(combined) == set.intersection(*(set(row) for row in episode_metrics.values())),
                 "recovered subgroup/combined metric keys differ")
    for metric, observed in combined.items():
        weighted = sum(episode_metrics[spec["label"]][metric] * spec["count"]
                       for spec in base.EPISODES) / 665
        base.require(math.isclose(observed, weighted, rel_tol=0.0, abs_tol=1e-6),
                     f"recovered subgroup aggregation differs: {metric}")
    base.require(not (recovery_root / "training").exists() and
                 not (recovery_root / "evaluation").exists(),
                 "recovery illegally contains training/evaluation stages")
    return subgroup, {
        "recovery_file_count": recovery_tree["file_count"],
        "recovery_total_size_bytes": recovery_tree["total_size_bytes"],
        "original_full_tree_inventory_sha256": observed_original_tree["inventory_sha256"],
        "execution_plan_only_evaluator_changed": True,
        "executed_recovery_wrapper_sha256": EXECUTED_RECOVERY_WRAPPER_SHA256,
        "patched_evaluator_sha256": PATCHED_EVALUATOR_SHA256,
        "checkpoint_sha256": ORIGINAL_CHECKPOINT_SHA256,
    }


def _geometry_stub(audit: dict[str, Any], context: str) -> dict[str, Any]:
    geometry, _ = base._geometry(audit, context)
    return {"geometry": geometry}


def _reference_stub(reference_root: Path) -> dict[str, Any]:
    subgroup = base.read_json(reference_root / base.REFERENCE_ARM / "subgroup/summary.json")
    metrics = _subgroup_metrics(subgroup, reference_root / base.REFERENCE_ARM / "subgroup",
                                "C reference subgroup")
    return {
        "metrics": metrics,
        **_geometry_stub(base.read_json(reference_root / base.REFERENCE_ARM / "gate_v8.json"),
                         "C reference audit"),
    }


def _verify_reference_hashes(reference_root: Path) -> None:
    rows = {
        "plan.json": REFERENCE_PLAN_SHA256,
        "summary.json": REFERENCE_SUMMARY_SHA256,
        f"{base.REFERENCE_ARM}/training/checkpoints/latest.pt": REFERENCE_CHECKPOINT_SHA256,
        f"{base.REFERENCE_ARM}/training/run.json": REFERENCE_TRAIN_SHA256,
        f"{base.REFERENCE_ARM}/subgroup/summary.json": REFERENCE_SUBGROUP_SHA256,
        f"{base.REFERENCE_ARM}/gate_v8.json": REFERENCE_AUDIT_SHA256,
    }
    for relative, expected in rows.items():
        _require_hash(reference_root / relative, expected, f"C reference {relative}")


def _create_composite(experiment_root: Path, recovery_root: Path, destination: Path,
                      original_plan: dict[str, Any], stages: list[dict[str, Any]],
                      diagnostic_gate: dict[str, Any]) -> None:
    shutil.copytree(experiment_root, destination, copy_function=shutil.copyfile)
    (destination / "failed.json").unlink()
    shutil.copytree(recovery_root / "subgroup", destination / ARM / "subgroup",
                    copy_function=shutil.copyfile)
    subgroup_path = destination / ARM / "subgroup/summary.json"
    subgroup = base.read_json(subgroup_path)
    # HH_260906 - Normalize only the synthetic validation view; immutable inputs remain untouched.
    subgroup["plan_sha256"] = ORIGINAL_PLAN_SHA256
    subgroup["evaluator_sha256"] = OLD_EVALUATOR_SHA256
    _write_json(subgroup_path, subgroup)
    subgroup_receipt = {
        "arm": ARM,
        "stage": "subgroup",
        "status": "COMPLETE",
        "plan_sha256": ORIGINAL_PLAN_SHA256,
        "path": f"{ARM}/subgroup/summary.json",
        "sha256": base.digest(subgroup_path),
        "checkpoint_sha256": ORIGINAL_CHECKPOINT_SHA256,
    }
    _write_json(destination / ARM / "subgroup_completed.json", subgroup_receipt)
    summary = {
        "schema": base.EXPERIMENT_SUMMARY_SCHEMA,
        "status": "COMPLETE_NOT_PROMOTED",
        "plan_sha256": ORIGINAL_PLAN_SHA256,
        "stages": [*deepcopy(stages), subgroup_receipt],
        "source_dataset_reference_postcheck_pass": True,
        "arm": "COMPLETE",
        "diagnostic_gate": diagnostic_gate,
        **{field: False for field in base.DENIAL_FIELDS},
    }
    _write_json(destination / "summary.json", summary)
    base.require(base.read_json(destination / "plan.json") == original_plan,
                 "synthetic validation view changed the E plan")


def summarize(*, reference_root: Path, experiment_root: Path,
              recovery_root: Path) -> dict[str, Any]:
    reference_root = base._canonical_directory(reference_root, "C reference root")
    experiment_root = base._canonical_directory(experiment_root, "original failed E root")
    recovery_root = base._canonical_directory(recovery_root, "E subgroup recovery root")
    _verify_reference_hashes(reference_root)
    original_plan, evaluation_contract, stages = _verify_original(experiment_root)
    recovered_subgroup, recovery_proof = _verify_recovery(
        recovery_root, experiment_root, original_plan)
    reference_stub = _reference_stub(reference_root)
    experiment_stub = {
        "metrics": _subgroup_metrics(recovered_subgroup, recovery_root / "subgroup",
                                     "recovered E subgroup gate input"),
        **_geometry_stub(base.read_json(experiment_root / ARM / "gate_v8.json"),
                         "original E audit"),
    }
    computed_gate = base._gate(reference_stub, experiment_stub)
    base.require(computed_gate["pass"] is False and
                 computed_gate["status"] == "FAIL_SINGLE_SEED_DIAGNOSTIC",
                 "sealed recovered E unexpectedly passed; do not authorize confirmation")
    with tempfile.TemporaryDirectory(prefix="moving-turn-replay-recovery-summary-") as temporary:
        composite = Path(temporary) / "E_composite_read_only_view"
        _create_composite(experiment_root, recovery_root, composite, original_plan,
                          stages, computed_gate)
        result = base.summarize(reference_root=reference_root, experiment_root=composite)
    base.require(result["preregistered_C_E_gate"]["result"] == computed_gate,
                 "full verification and independently recomputed recovery gate differ")
    result["schema"] = SCHEMA
    result["created_at_utc"] = datetime.now(timezone.utc).isoformat()
    result["inputs"] = {
        "C_reference": {
            "campaign_id": base.REFERENCE_CAMPAIGN,
            "plan_sha256": REFERENCE_PLAN_SHA256,
            "summary_sha256": REFERENCE_SUMMARY_SHA256,
        },
        "E_experiment": {
            "campaign_id": base.EXPERIMENT_CAMPAIGN,
            "plan_sha256": ORIGINAL_PLAN_SHA256,
            "failed_sha256": ORIGINAL_FAILED_SHA256,
            "original_status": "FAIL_SUBGROUP",
            "posthoc_subgroup_recovery": {
                "plan_sha256": RECOVERY_PLAN_SHA256,
                "execution_plan_sha256": RECOVERY_EXECUTION_PLAN_SHA256,
                "summary_sha256": RECOVERY_SUMMARY_SHA256,
                "subgroup_sha256": RECOVERY_SUBGROUP_SHA256,
            },
        },
    }
    e_provenance = result["arm_provenance"]["E"]
    e_provenance.update({
        "subgroup_source": "posthoc_recovery",
        "subgroup_sha256": RECOVERY_SUBGROUP_SHA256,
        "original_failed_sha256": ORIGINAL_FAILED_SHA256,
        "recovery_plan_sha256": RECOVERY_PLAN_SHA256,
        "recovery_execution_plan_sha256": RECOVERY_EXECUTION_PLAN_SHA256,
    })
    result["preregistered_C_E_gate"].update({
        "runner_summary_exact_match": False,
        "posthoc_recovery_subgroup_used": True,
    })
    result["verification"] = {
        "original_exact_3_of_4_failure": True,
        "original_completed_stages": ["train", "evaluate", "audit"],
        "original_failed_stage": "subgroup",
        "original_full_tree_unchanged": True,
        "recovery_subgroup_only": True,
        "gpu0_only": True,
        "source_dataset_checkpoint_and_episode_bindings": True,
        "evaluation_contract": evaluation_contract,
        **recovery_proof,
    }
    result["scope"].update({
        "unseen_confirmation_seeds_authorized": False,
        "posthoc_recovery_is_original_runner_completion": False,
    })
    base.require(result["scope"]["model_promotion"] is False and
                 result["scope"]["model_export"] is False and
                 result["scope"]["unseen_confirmation_seeds_authorized"] is False,
                 "failed gate must not authorize export, promotion, or unseen seeds")
    result["interpretation"] = (
        "The original E runner remains FAIL_SUBGROUP (3/4 stages). A separately sealed, "
        "subgroup-only posthoc recovery supplies the two fixed-VAL episode slices. The "
        "independently recomputed preregistered C-versus-E gate fails, so no unseen seed, "
        "promotion, export, closed-loop, all-map, deployment, or vehicle-readiness claim is allowed."
    )
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--c-reference-root", "--reference-root", dest="reference_root",
                        type=Path, required=True)
    parser.add_argument("--e-experiment-root", "--experiment-root", dest="experiment_root",
                        type=Path, required=True)
    parser.add_argument("--recovery-root", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    args = parser.parse_args(argv)
    result = summarize(reference_root=args.reference_root,
                       experiment_root=args.experiment_root,
                       recovery_root=args.recovery_root)
    base.write_new(args.output_json, result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
