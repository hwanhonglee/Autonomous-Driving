#!/usr/bin/env python3
"""HH_260906 - Recover only the failed E subgroup stage under a create-only amended plan."""
from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys


SCHEMA = "portable_e2e.moving_turn_replay_subgroup_recovery_plan.v1"
SUMMARY_SCHEMA = "portable_e2e.moving_turn_replay_subgroup_recovery_summary.v1"
RECOVERY_ID = "hh260921-moving-turn-replay-subgroup-recovery-v1"
ORIGINAL_CAMPAIGN_ID = "hh260921-moving-turn-replay-seed20260906-v1"
ORIGINAL_ROOT = "runs/diagnostics/moving_turn_replay_seed20260906_20260921_v1"
OUTPUT_PATH = "runs/diagnostics/moving_turn_replay_seed20260906_20260921_v1_subgroup_recovery_v1"
ARM = "E_turn_replay_train"
GPU = {"physical_index": 0, "uuid": "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"}
ORIGINAL_PLAN_SHA256 = "fda9836d51538ad66e51fb4250e53ef94aadf23b90bd77525bb38edbe811d906"
CHECKPOINT_SHA256 = "c7739926f25a5c3db7519f33fce236b70d8108e912ec4009bd876afa2a666f78"
RUNNER_SHA256 = "46ce391a5c50712bc799c0cc43e83c185e425048b5d9ff07f80a1358bc2ed129"
OLD_EVALUATOR_SHA256 = "a1285542bcde1d404fe500dcdb5ef8996637c9e4bf4ec89d672e2117751d162b"
PATCHED_EVALUATOR_SHA256 = "4720a9207d588c91ad1704defef21d4e07bb0d5f81781ffbf597414287af5962"
OWNER_PATH = "benchmarks/training_profile_v1/profile_portable_training.py"
OWNER_SHA256 = "a995722f1a60f192a059433afb44e6297937d97166877c5cc4de23220d15b436"
PIPELINE_PATH = "scripts/e2e/run_portable_training_campaign.py"
ORIGINAL_RUNNER_PATH = "source/run_moving_turn_replay_seed_pilot.py"
ORIGINAL_EVALUATOR_PATH = "source/evaluate_portable_moving_turn_replay_subgroups.py"
SUBGROUP_SUMMARY_PATH = "subgroup/summary.json"
TIMEOUT_SECONDS = 1200
WHOLE_TIMEOUT_SECONDS = 1500
KNOWN_FAILURE = {"error_type": "ValueError", "error": "E_turn_replay_train subgroup exited 2",
    "subgroup_log_exact": "MOVING_TURN_REPLAY_SUBGROUP_ERROR: checkpoint replay contract/accounting differs\n"}
REQUIRED_FILES = {
    "plan": "plan.json", "failed": "failed.json", "subgroup_log": f"{ARM}/subgroup.log",
    "checkpoint": f"{ARM}/training/checkpoints/latest.pt", "training_run": f"{ARM}/training/run.json",
    "evaluation": f"{ARM}/evaluation/metrics.json", "audit": f"{ARM}/gate_v8.json",
    "runner": ORIGINAL_RUNNER_PATH, "old_evaluator": ORIGINAL_EVALUATOR_PATH,
}
FORBIDDEN_ORIGINAL_FILES = ("summary.json", f"{ARM}/subgroup/summary.json", f"{ARM}/subgroup_completed.json")
ALLOWED_FIX = {"scope": "subgroup_evaluator_train_config_representation_only",
    "reason": "Checkpoint TrainConfig stores domain_ratios as a tuple while the sealed JSON plan stores the equivalent value as a list.",
    "old_evaluator_sha256": OLD_EVALUATOR_SHA256, "patched_evaluator_sha256": PATCHED_EVALUATOR_SHA256,
    "behavior": "Canonicalize checkpoint train_config through strict JSON before equality with the plan; no model, checkpoint, dataset, metric, gate or stage change.",
    "changed_execution_plan_fields": ["evaluator_sha256"]}
DENIALS = {"automatic_retry": False, "training_executed": False, "evaluate_stage_executed": False,
    "audit_executed": False, "test_read_or_use": False, "model_export": False,
    "automatic_promotion": False, "training_data_approved": False, "vehicle_control_approved": False,
    "closed_loop_claim": False, "all_map_claim": False, "deployment_approved": False}


def require(value, message):
    if not value:
        raise ValueError(message)


def utc():
    return datetime.now(timezone.utc).isoformat()


def encoded(value):
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()


def decode(payload):
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, "duplicate JSON key")
            result[key] = value
        return result
    return json.loads(payload, object_pairs_hook=pairs,
        parse_constant=lambda value: require(False, "nonfinite JSON: " + value))


def canonical(path, *, must_exist=True):
    path = Path(path).absolute(); require(".." not in path.parts, "parent traversal is forbidden")
    if must_exist:
        require(path.exists() and path == path.resolve(), "canonical existing path required: " + str(path))
    return path


def regular(path):
    path = canonical(path)
    require(path.is_file() and path.stat().st_nlink == 1
            and not any(parent.is_symlink() for parent in (path, *path.parents)),
            "regular single-link nonsymlink file required: " + str(path))
    return path


def directory(path):
    path = canonical(path)
    require(path.is_dir() and not path.is_symlink(), "canonical nonsymlink directory required: " + str(path))
    return path


def digest(path):
    value = hashlib.sha256()
    with regular(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def read(path):
    return decode(regular(path).read_text())


def write_new(path, value):
    # HH_260906 - Recovery declarations and receipts are immutable create-only evidence.
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        stream.write(encoded(value)); stream.flush(); os.fsync(stream.fileno())


def copy_new(source, target, expected):
    source = regular(source); require(digest(source) == expected, "copy source SHA differs")
    target = Path(target); target.parent.mkdir(parents=True, exist_ok=True)
    with source.open("rb") as src, target.open("xb") as dst:
        shutil.copyfileobj(src, dst, 1024 * 1024); dst.flush(); os.fsync(dst.fileno())
    require(digest(target) == expected and source.stat().st_ino != target.stat().st_ino,
            "independent exact copy failed")


def sha256(value, name="SHA256"):
    require(type(value) is str and re.fullmatch(r"[0-9a-f]{64}", value), name + " required")
    return value


def relative(value, name="relative path"):
    require(type(value) is str and value and not Path(value).is_absolute()
            and ".." not in Path(value).parts and Path(value).as_posix() == value,
            name + " required")
    return value


def timestamp(value):
    require(type(value) is str, "timestamp string required")
    parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    require(parsed.tzinfo is not None and parsed.utcoffset().total_seconds() == 0, "UTC timestamp required")
    return parsed


def load_module(path, expected, name):
    path = regular(path); require(digest(path) == expected, "module SHA differs")
    spec = importlib.util.spec_from_file_location(name, path); require(spec and spec.loader, "module spec failed")
    module = importlib.util.module_from_spec(spec); sys.modules[name] = module
    prior = sys.dont_write_bytecode
    try:
        # HH_260906 - Import sealed evidence without creating a pycache inside the immutable failed run.
        sys.dont_write_bytecode = True; spec.loader.exec_module(module)
    finally:
        sys.dont_write_bytecode = prior
    require(digest(path) == expected, "module changed during import"); return module


def tree_inventory(root):
    root = directory(root); rows = []; total = 0
    for parent, directories, files in os.walk(root, followlinks=False):
        for name in [*directories, *files]:
            require(not (Path(parent) / name).is_symlink(), "original run contains a symlink")
        for name in files:
            path = regular(Path(parent) / name); size = path.stat().st_size; total += size
            rows.append({"path": path.relative_to(root).as_posix(), "size": size, "sha256": digest(path)})
    rows.sort(key=lambda row: row["path"])
    require(rows and len({row["path"] for row in rows}) == len(rows), "original inventory differs")
    return {"file_count": len(rows), "total_size_bytes": total, "files": rows,
        "inventory_sha256": hashlib.sha256(encoded(rows)).hexdigest()}


def _required_bindings(root):
    result = {}
    for key, name in REQUIRED_FILES.items():
        path = regular(root / name); result[key] = {"path": name, "size": path.stat().st_size, "sha256": digest(path)}
    return result


def _only_evaluator_changed(original, execution):
    require(set(original) == set(execution), "execution plan fields differ")
    for key in original:
        if key == "evaluator_sha256":
            require(original[key] == OLD_EVALUATOR_SHA256 and execution[key] == PATCHED_EVALUATOR_SHA256,
                    "evaluator amendment differs")
        else:
            require(original[key] == execution[key], "execution plan changed forbidden field: " + key)


def inspect_original(*, workspace, original_root, repo_root, runner_module=None):
    workspace = directory(workspace); root = directory(original_root); repo = directory(repo_root)
    require(root == workspace / ORIGINAL_ROOT and root.is_relative_to(workspace / "runs/diagnostics"),
            "original failed run root differs")
    bindings = _required_bindings(root)
    require(bindings["plan"]["sha256"] == ORIGINAL_PLAN_SHA256
            and bindings["checkpoint"]["sha256"] == CHECKPOINT_SHA256
            and bindings["runner"]["sha256"] == RUNNER_SHA256
            and bindings["old_evaluator"]["sha256"] == OLD_EVALUATOR_SHA256,
            "known original evidence SHA differs")
    for name in FORBIDDEN_ORIGINAL_FILES: require(not (root / name).exists(), "failed run has impossible completion file")
    original = read(root / REQUIRED_FILES["plan"]); failed = read(root / REQUIRED_FILES["failed"])
    train = read(root / REQUIRED_FILES["training_run"]); evaluation = read(root / REQUIRED_FILES["evaluation"])
    audit = read(root / REQUIRED_FILES["audit"])
    require(original.get("campaign_id") == ORIGINAL_CAMPAIGN_ID and original.get("output") == ORIGINAL_ROOT
            and original.get("runner_sha256") == RUNNER_SHA256
            and original.get("evaluator_sha256") == OLD_EVALUATOR_SHA256, "original plan identity differs")
    require(failed.get("status") == "FAIL" and failed.get("plan_sha256") == ORIGINAL_PLAN_SHA256
            and failed.get("error_type") == KNOWN_FAILURE["error_type"]
            and failed.get("error") == KNOWN_FAILURE["error"] and failed.get("arm") == "FAIL",
            "failed receipt is not the known subgroup-only failure")
    require((root / REQUIRED_FILES["subgroup_log"]).read_text() == KNOWN_FAILURE["subgroup_log_exact"],
            "subgroup failure log differs")
    stages = failed.get("stages")
    require(type(stages) is list and [row.get("stage") for row in stages] == ["train", "evaluate", "audit"]
            and all(row.get("status") == "COMPLETE" and row.get("checkpoint_sha256") == CHECKPOINT_SHA256
                    for row in stages), "pre-failure completed stage receipts differ")
    require(train.get("status") == "TRAINING_TARGET_REACHED"
            and train.get("state") == {"epoch": 20, "next_batch_index": 0, "global_step": 9020,
                "samples_seen": 36040, "domain_samples_seen": {"carla": 36040}}
            and train.get("sampling_execution_schedule_row_count") == 36040
            and sha256(train.get("sampling_execution_schedule_sha256")) is not None,
            "completed training evidence differs")
    require(evaluation.get("status") == "OPEN_LOOP_EVALUATION_COMPLETE"
            and evaluation.get("checkpoint_sha256") == CHECKPOINT_SHA256
            and evaluation.get("evaluation_split") == "val" and evaluation.get("sample_count") == 665,
            "completed evaluation evidence differs")
    require(audit.get("status") == "RUNTIME_GEOMETRY_AUDIT_COMPLETE"
            and audit.get("checkpoint_sha256") == CHECKPOINT_SHA256
            and audit.get("evaluation_split") == "val" and audit.get("geometry", {}).get("sample_count") == 665,
            "completed audit evidence differs")
    runner = runner_module or load_module(root / ORIGINAL_RUNNER_PATH, RUNNER_SHA256, "_recovery_original_runner")
    runner.validate_plan(original, runner_sha256=RUNNER_SHA256)
    runner.verify_sources(original, repo)
    return {"root": ORIGINAL_ROOT, "tree": tree_inventory(root), "required": bindings,
        "known_failure": KNOWN_FAILURE}, original, runner


def build_execution_plan(original, patched_evaluator_sha256):
    require(patched_evaluator_sha256 == PATCHED_EVALUATOR_SHA256, "unreviewed evaluator patch")
    execution = deepcopy(original); execution["evaluator_sha256"] = patched_evaluator_sha256
    _only_evaluator_changed(original, execution); return execution


def declare_plan(*, workspace, repo_root, original_run_root, patched_evaluator,
                 output_plan, output_execution_plan, runner_module=None):
    workspace = directory(workspace); patched_evaluator = regular(patched_evaluator)
    require(digest(patched_evaluator) == PATCHED_EVALUATOR_SHA256, "patched evaluator SHA differs")
    evidence, original, runner = inspect_original(workspace=workspace, original_root=original_run_root,
        repo_root=repo_root, runner_module=runner_module)
    execution = build_execution_plan(original, digest(patched_evaluator))
    runner.validate_plan(execution, runner_sha256=RUNNER_SHA256)
    execution_sha = hashlib.sha256(encoded(execution)).hexdigest()
    plan = {"schema": SCHEMA, "recovery_id": RECOVERY_ID, "declared_at_utc": utc(),
        "gpu": GPU, "output": OUTPUT_PATH, "original_run": evidence,
        "execution_plan": {"path": "execution_plan.json", "sha256": execution_sha,
            "original_plan_sha256": ORIGINAL_PLAN_SHA256,
            "changed_fields": ["evaluator_sha256"]},
        "runner": {"path": ORIGINAL_RUNNER_PATH, "sha256": RUNNER_SHA256},
        "patched_evaluator": {"name": Path(patched_evaluator).name, "sha256": PATCHED_EVALUATOR_SHA256},
        "allowed_fix": ALLOWED_FIX, "stage_order": ["subgroup"],
        "timeout_seconds": TIMEOUT_SECONDS, "whole_timeout_seconds": WHOLE_TIMEOUT_SECONDS,
        "subgroup_evaluation_executed": True, **DENIALS}
    validate_plan(plan); output_plan = Path(output_plan); output_execution_plan = Path(output_execution_plan)
    require(output_plan.absolute() != output_execution_plan.absolute(), "declaration output files must differ")
    write_new(output_execution_plan, execution); write_new(output_plan, plan)
    require(digest(output_execution_plan) == execution_sha, "persisted execution plan SHA differs")
    return plan, execution


def validate_plan(plan, *, observed=None):
    fixed = {"schema": SCHEMA, "recovery_id": RECOVERY_ID, "gpu": GPU, "output": OUTPUT_PATH,
        "runner": {"path": ORIGINAL_RUNNER_PATH, "sha256": RUNNER_SHA256},
        "allowed_fix": ALLOWED_FIX, "stage_order": ["subgroup"], "timeout_seconds": TIMEOUT_SECONDS,
        "whole_timeout_seconds": WHOLE_TIMEOUT_SECONDS, "subgroup_evaluation_executed": True, **DENIALS}
    variable = {"declared_at_utc", "original_run", "execution_plan", "patched_evaluator"}
    require(type(plan) is dict and set(plan) == set(fixed) | variable, "recovery plan fields differ")
    for key, value in fixed.items(): require(plan.get(key) == value, "fixed recovery field differs: " + key)
    require(timestamp(plan["declared_at_utc"]) <= (observed or datetime.now(timezone.utc)), "future declaration forbidden")
    original = plan["original_run"]
    require(type(original) is dict and set(original) == {"root", "tree", "required", "known_failure"}
            and original["root"] == ORIGINAL_ROOT and original["known_failure"] == KNOWN_FAILURE,
            "original evidence contract differs")
    tree = original["tree"]
    require(type(tree) is dict and set(tree) == {"file_count", "total_size_bytes", "files", "inventory_sha256"}
            and type(tree["files"]) is list and tree["file_count"] == len(tree["files"])
            and tree["inventory_sha256"] == hashlib.sha256(encoded(tree["files"])).hexdigest(),
            "original tree inventory contract differs")
    require(set(original["required"]) == set(REQUIRED_FILES), "required evidence bindings differ")
    for key, name in REQUIRED_FILES.items():
        row = original["required"][key]
        require(row.get("path") == name and type(row.get("size")) is int and row["size"] >= 0, "evidence binding differs")
        sha256(row.get("sha256"), "evidence SHA")
    require(original["required"]["plan"]["sha256"] == ORIGINAL_PLAN_SHA256
            and original["required"]["checkpoint"]["sha256"] == CHECKPOINT_SHA256, "known evidence binding differs")
    execution = plan["execution_plan"]
    require(execution == {"path": "execution_plan.json", "sha256": execution.get("sha256"),
            "original_plan_sha256": ORIGINAL_PLAN_SHA256, "changed_fields": ["evaluator_sha256"]},
            "execution plan binding differs")
    sha256(execution["sha256"], "execution plan SHA")
    require(plan["patched_evaluator"] == {"name": "evaluate_portable_moving_turn_replay_subgroups.py",
            "sha256": PATCHED_EVALUATOR_SHA256}, "patched evaluator binding differs")
    return plan


def verify_original_tree(plan, workspace):
    root = directory(workspace / ORIGINAL_ROOT); observed = tree_inventory(root)
    require(observed == plan["original_run"]["tree"], "original failed run changed")
    for key, row in plan["original_run"]["required"].items():
        require(digest(root / row["path"]) == row["sha256"], "required original file changed: " + key)
    return root


def verify_gpu_idle(value):
    # HH_260906 - Bind every recovery execution to the preregistered idle physical GPU0 UUID.
    require(type(value) is dict and value.get("physical_index") == GPU["physical_index"]
            and value.get("uuid") == GPU["uuid"]
            and value.get("pre_cuda_compute_processes") == 0,
            "physical GPU0 identity or idle state differs")
    return value


def verify_execution(plan, execution, original, runner):
    require(hashlib.sha256(encoded(execution)).hexdigest() == plan["execution_plan"]["sha256"],
            "execution plan bytes differ")
    _only_evaluator_changed(original, execution)
    runner.validate_plan(original, runner_sha256=RUNNER_SHA256)
    runner.validate_plan(execution, runner_sha256=RUNNER_SHA256)


def stage_command(*, workspace, repo, evaluator, runner, execution_plan, checkpoint, output):
    python = workspace / "venvs/py312/bin/python"
    return [str(python), str(evaluator), "--repo-root", str(repo), "--workspace", str(workspace),
        "--runner", str(runner), "--runner-sha256", RUNNER_SHA256,
        "--plan", str(execution_plan), "--plan-sha256", digest(execution_plan),
        "--checkpoint", str(checkpoint), "--checkpoint-sha256", CHECKPOINT_SHA256,
        "--output-dir", str(output / "subgroup")]


def verify_subgroup(output, execution_sha):
    summary_path = output / SUBGROUP_SUMMARY_PATH; summary = read(summary_path)
    require(summary.get("status") == "COMPLETE_NOT_PROMOTED" and summary.get("arm") == ARM
            and summary.get("plan_sha256") == execution_sha and summary.get("runner_sha256") == RUNNER_SHA256
            and summary.get("evaluator_sha256") == PATCHED_EVALUATOR_SHA256
            and summary.get("checkpoint_sha256") == CHECKPOINT_SHA256
            and summary.get("combined_val_count") == 665
            and summary.get("test_read_or_use") is False
            and summary.get("training_executed_by_evaluator") is False
            and summary.get("automatic_promotion") is False
            and summary.get("vehicle_control_approved") is False
            and summary.get("closed_loop_evaluation") is False,
            "recovered subgroup summary differs")
    episodes = summary.get("episodes")
    require(type(episodes) is dict and set(episodes) == {"town03_right_turn_dev", "town03_straight_dev"}
            and episodes["town03_right_turn_dev"].get("count") == 337
            and episodes["town03_straight_dev"].get("count") == 328,
            "recovered subgroup episode evidence differs")
    return {"path": SUBGROUP_SUMMARY_PATH, "sha256": digest(summary_path),
        "checkpoint_sha256": CHECKPOINT_SHA256}


def run(*, recovery_plan, execution_plan, workspace, repo_root, patched_evaluator, runner_module=None,
        owner_module=None, pipeline_module=None):
    plan_path = regular(recovery_plan); execution_path = regular(execution_plan)
    workspace = directory(workspace); repo = directory(repo_root); evaluator = regular(patched_evaluator)
    plan_sha = digest(plan_path); plan = validate_plan(read(plan_path)); execution = read(execution_path)
    require(digest(execution_path) == plan["execution_plan"]["sha256"]
            and digest(evaluator) == PATCHED_EVALUATOR_SHA256, "recovery execution input differs")
    original_root = verify_original_tree(plan, workspace); original = read(original_root / "plan.json")
    runner_path = regular(original_root / ORIGINAL_RUNNER_PATH)
    require(digest(runner_path) == RUNNER_SHA256, "archived runner differs")
    runner = runner_module or load_module(runner_path, RUNNER_SHA256, "_recovery_execution_runner")
    verify_execution(plan, execution, original, runner); runner.verify_sources(execution, repo)
    checkpoint = regular(original_root / REQUIRED_FILES["checkpoint"])
    require(digest(checkpoint) == CHECKPOINT_SHA256, "recovery checkpoint differs")
    output = workspace / OUTPUT_PATH
    require(output == workspace / plan["output"] and not output.exists() and not output.is_symlink(), "fresh recovery output required")
    owner_path = regular(workspace / OWNER_PATH); require(digest(owner_path) == OWNER_SHA256, "GPU owner differs")
    pipeline_path = regular(repo / PIPELINE_PATH)
    pipeline_sha = execution["source"]["current_sha256"][PIPELINE_PATH]
    owner = owner_module or load_module(owner_path, OWNER_SHA256, "_recovery_gpu_owner")
    pipeline = pipeline_module or load_module(pipeline_path, pipeline_sha, "_recovery_stage_owner")
    owner.check_environment(workspace, repo)
    with owner.gpu_lease(workspace):
        idle = verify_gpu_idle(owner.gpu_idle(repo)); output.mkdir(parents=True, exist_ok=False)
        copy_new(plan_path, output / "recovery_plan.json", plan_sha)
        copy_new(execution_path, output / "execution_plan.json", plan["execution_plan"]["sha256"])
        copy_new(runner_path, output / "source/original_runner.py", RUNNER_SHA256)
        copy_new(evaluator, output / "source/patched_evaluator.py", PATCHED_EVALUATOR_SHA256)
        started = {"schema": SUMMARY_SCHEMA, "status": "RUNNING", "started_at_utc": utc(),
            "recovery_plan_sha256": plan_sha, "execution_plan_sha256": plan["execution_plan"]["sha256"],
            "original_plan_sha256": ORIGINAL_PLAN_SHA256, "checkpoint_sha256": CHECKPOINT_SHA256,
            "pre_cuda_gpu": idle, "stage": "subgroup", **DENIALS}
        write_new(output / "started.json", started)
        try:
            with owner.bounded_signals(WHOLE_TIMEOUT_SECONDS):
                verify_original_tree(plan, workspace); require(digest(checkpoint) == CHECKPOINT_SHA256, "checkpoint changed")
                command = stage_command(workspace=workspace, repo=repo, evaluator=evaluator,
                    runner=runner_path, execution_plan=output / "execution_plan.json", checkpoint=checkpoint, output=output)
                require("portable_e2e.train" not in command and "portable_e2e.evaluate" not in command
                        and "portable_e2e.audit_runtime" not in command and "--resume" not in command,
                        "recovery command escaped subgroup-only scope")
                receipt = {"status": "RUNNING", "stage": "subgroup", "command": command,
                    "started_at_utc": utc(), "timeout_seconds": TIMEOUT_SECONDS,
                    "input_checkpoint_sha256": CHECKPOINT_SHA256, **DENIALS}
                write_new(output / "subgroup_started.json", receipt)
                # HH_260906 - The subgroup child may read the archived runner but must never add bytecode to its tree.
                env = dict(os.environ, PORTABLE_MOVING_TURN_REPLAY_RECOVERY_OWNER_PID=str(os.getpid()),
                    PYTHONDONTWRITEBYTECODE="1")
                with (output / "subgroup.log").open("xb") as log:
                    rc = pipeline.run_owned_stage(command, repo, env, log, timeout=TIMEOUT_SECONDS)
                require(rc == 0, "recovery subgroup exited " + str(rc))
                require(digest(checkpoint) == CHECKPOINT_SHA256, "subgroup changed checkpoint")
                proof = verify_subgroup(output, plan["execution_plan"]["sha256"])
                verify_original_tree(plan, workspace)
                receipt.update(status="COMPLETE", ended_at_utc=utc(), **proof)
                write_new(output / "completed.json", receipt)
                summary = {"schema": SUMMARY_SCHEMA, "status": "COMPLETE_NOT_PROMOTED",
                    "completed_at_utc": utc(), "recovery_plan_sha256": plan_sha,
                    "execution_plan_sha256": plan["execution_plan"]["sha256"],
                    "original_plan_sha256": ORIGINAL_PLAN_SHA256,
                    "original_failed_run_unchanged": True, "stage_order": ["subgroup"],
                    "subgroup": proof, "final_gpu": verify_gpu_idle(owner.gpu_idle(repo)),
                    "subgroup_evaluation_executed": True, **DENIALS}
                write_new(output / "recovery_summary.json", summary)
        except BaseException as error:
            failed = {**started, "status": "FAIL", "error_type": type(error).__name__,
                "error": str(error), "ended_at_utc": utc()}
            with owner.finalization_signals(): write_new(output / "failed.json", failed)
            raise
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    sub = parser.add_subparsers(dest="mode", required=True)
    declare = sub.add_parser("declare", allow_abbrev=False)
    declare.add_argument("--workspace", type=Path, required=True); declare.add_argument("--repo-root", type=Path, required=True)
    declare.add_argument("--original-run-root", type=Path, required=True)
    declare.add_argument("--patched-evaluator", type=Path, required=True)
    declare.add_argument("--output-plan", type=Path, required=True)
    declare.add_argument("--output-execution-plan", type=Path, required=True)
    execute = sub.add_parser("run", allow_abbrev=False)
    execute.add_argument("--workspace", type=Path, required=True); execute.add_argument("--repo-root", type=Path, required=True)
    execute.add_argument("--recovery-plan", type=Path, required=True)
    execute.add_argument("--execution-plan", type=Path, required=True)
    execute.add_argument("--patched-evaluator", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.mode == "declare":
        declare_plan(workspace=args.workspace, repo_root=args.repo_root,
            original_run_root=args.original_run_root, patched_evaluator=args.patched_evaluator,
            output_plan=args.output_plan, output_execution_plan=args.output_execution_plan)
    else:
        run(recovery_plan=args.recovery_plan, execution_plan=args.execution_plan,
            workspace=args.workspace, repo_root=args.repo_root, patched_evaluator=args.patched_evaluator)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
