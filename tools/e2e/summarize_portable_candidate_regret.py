#!/usr/bin/env python3
"""HH_260906 - Verify a fresh three-seed composite-regret experiment without model or data promotion."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import re
import subprocess

from portable_e2e.contract import ContractError, _loads_json
from scripts.e2e import summarize_portable_no_accel_development as shared
from scripts.e2e import summarize_portable_training_campaign as base
from scripts.e2e import summarize_portable_data_expansion as expansion

SCHEMA = "portable_e2e.candidate_regret_summary.v1"
PLAN_SCHEMA = "portable_e2e.candidate_regret_campaign.v1"
CAMPAIGN_ID = "hh260909-candidate-regret-ab-3seeds-v1"
ARMS = {"A_original_loss": .0001, "B_cost_aware_selector": .0001}
WEIGHTS = {"A_original_loss": 0.0, "B_cost_aware_selector": .1}
MODEL = shared.MODELS["A_physical_input"]
MODEL_SHA256 = shared.MODEL_SHA256["A_physical_input"]
DATASET, WORKER, ARCHIVE = shared.DATASET, shared.WORKER, shared.ARCHIVE
CORE_FILES = tuple(name for name in shared.CORE_FILES if name != shared.MODELS["B_no_accel_input"])
STAGE_TIMEOUTS = {"train": 600, "evaluate": 120, "audit": 120}
DEADLINE = "2026-09-09T01:00:00Z"
LOSS = {"xy_weight": 1.0, "speed_weight": .2, "yaw_weight": .1, "kinematic_speed_weight": .05,
        "final_displacement_weight": .5, "candidate_score_weight": .1}
canonical, sha_file = shared.canonical, shared.sha_file


def git_bytes(commit, path):
    # HH_260906 - Historical source verification must never fetch missing objects or contact a remote.
    env = dict(os.environ, GIT_NO_LAZY_FETCH="1", GIT_ALLOW_PROTOCOL="", GIT_TERMINAL_PROMPT="0")
    try:
        return subprocess.run(["git", "-c", "protocol.allow=never", "show", f"{commit}:{path}"],
            cwd=base.REPO, env=env, check=True, capture_output=True, timeout=15).stdout
    except (subprocess.SubprocessError, OSError) as error:
        raise ContractError("pinned historical source unavailable locally") from error


def validate_plan(plan, state, payload):
    fixed = {"schema": PLAN_SCHEMA, "campaign_id": CAMPAIGN_ID, "gpu_uuid": "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5",
        "dataset": DATASET, "dataset_manifest_sha256": expansion.MANIFEST_SHA256, "model_config": MODEL,
        "model_config_sha256": MODEL_SHA256, "seeds": base.SEEDS, "arms": ARMS, "steps": 1540, "batch_size": 4,
        "split": "val", "expected_train_samples": 1147, "expected_val_samples": 337, "candidate_score_weight": .1,
        "candidate_regret_weights": WEIGHTS, "finish_before_utc": DEADLINE, "stage_timeout_seconds": STAGE_TIMEOUTS,
        "finish_reserve_seconds": 300}
    base._require(all(canonical(plan.get(k)) == canonical(v) for k, v in fixed.items())
        and list(plan["arms"]) == list(ARMS), "candidate-regret plan differs from fixed prospective protocol")
    base._require(not any(k in plan for k in ("model_configs", "prerequisite_campaign_id", "baseline_campaign_id",
        "baseline_source_commit", "prerequisite_timeout_seconds", "resume", "checkpoint")), "not six fresh training runs")
    commit = plan.get("source_commit")
    base._require(isinstance(commit, str) and re.fullmatch(r"[0-9a-f]{40}", commit) is not None, "source commit must be pinned")
    base._require(state.get("source_commit") == commit and state.get("plan") == plan
        and state.get("plan_sha256") == base._sha(payload), "frozen plan/source/hash mismatch")
    base._require(plan.get("decision", {}).get("absolute_limits_m") ==
        {k.removesuffix("_m"): v for k, v in base.ABSOLUTE_LIMITS.items()}, "absolute gates changed")
    base._require(state.get("dataset_manifest_sha256") == expansion.MANIFEST_SHA256
        and state.get("reviewed_contract") == {"train_samples": 1147, "val_samples": 337, "run_count": 6, "stage_count": 18}
        and state.get("prerequisite") is None and state.get("vehicle_control_approved") is False,
        "dataset/stage budget or nonpromotion proof mismatch")
    sources = {name: git_bytes(commit, name) for name in CORE_FILES}
    config = _loads_json(sources[MODEL].decode(), "pinned physical config")
    base._require(base._sha(sources[MODEL]) == MODEL_SHA256 == state.get("model_config_sha256")
        and state.get("model_config") == config and config.get("model_id") == "portable_e2e.perspective_trajectory.physical.v1"
        and state.get("model_config_canonical_sha256") == base._sha(canonical(config).encode()), "physical model config proof mismatch")
    base._require(state.get("runner_sha256") == base._sha(sources[WORKER]), "executed runner differs from pinned source")
    return config, sources


def expected_command(record, checkpoint_sha):
    command = record.get("command")
    base._require(isinstance(command, list) and command and all(isinstance(v, str) for v in command), "invalid recorded command")
    python = Path(command[0])
    base._require(python.is_absolute() and ".." not in python.parts and python.parts[-4:] == ("venvs", "py312", "bin", "python"),
        "interpreter must be the personal py312 venv")
    workspace = python.parents[3]
    base._require(workspace.parts[-3:] == ("personal", "hwanhong", "portable_e2e"), "unreviewed workspace")
    seed, arm = record["run"].split("/")
    item = workspace / "runs/campaigns" / CAMPAIGN_ID / record["run"]
    common = ["--device", "cuda:0", "--batch-size", "4"]
    dataset = str(workspace / DATASET)
    if record["stage"] == "train":
        expected = [str(python), "-m", "portable_e2e.train", dataset, "--run-dir", str(item / "training"),
            "--model-config", str(workspace / "autoware_e2e" / MODEL), "--split", "train", *common,
            "--seed", seed.removeprefix("seed_"), "--learning-rate", "0.0001", "--weight-decay", "0.0001",
            "--max-steps", "1540", "--checkpoint-interval", "154", "--num-workers", "0", "--maximum-gradient-norm", "5",
            "--sampling-policy", "uniform_without_replacement", "--candidate-score-weight", "0.1",
            "--candidate-regret-weight", str(WEIGHTS[arm])]
    elif record["stage"] == "evaluate":
        expected = [str(python), "-m", "portable_e2e.evaluate", dataset, "--checkpoint", str(item / "training/checkpoints/latest.pt"),
            "--output-dir", str(item / "evaluation"), "--split", "val", *common, "--num-workers", "0", "--render-count", "12"]
    else:
        base._require(isinstance(checkpoint_sha, str) and re.fullmatch(r"[0-9a-f]{64}", checkpoint_sha) is not None, "missing checkpoint SHA")
        expected = [str(python), "-m", "portable_e2e.audit_runtime", dataset, "--checkpoint", str(item / "training/checkpoints/latest.pt"),
            "--output-json", str(item / "gate_v8.json"), "--split", "val", *common, "--checkpoint-sha256", checkpoint_sha]
    base._require(command == expected, "exact fresh train/val command mismatch")
    return [v.replace(str(workspace), "<PERSONAL_WORKSPACE>") for v in command]


def auxiliary_diagnostics(training, evaluation, arm):
    # HH_260906 - Composite-cost expected regret is not selected ADE minus ADE oracle and never replaces standard metrics.
    names = ("auxiliary_loss_metrics", "auxiliary_loss_metric_counts", "loss_config")
    last = training.get("last_metrics", {})
    if WEIGHTS[arm] == 0:
        base._require(not any(k in evaluation for k in names) and "candidate_regret_loss" not in last,
            "zero arm must preserve legacy loss/report keys")
        return {"enabled": False, "weight": 0.0}
    metrics = evaluation.get("auxiliary_loss_metrics")
    base._require(isinstance(metrics, dict) and set(metrics) == {"candidate_regret_loss"}
        and evaluation.get("auxiliary_loss_metric_counts") == {"candidate_regret_loss": 337}
        and evaluation.get("loss_config") == dict(LOSS, candidate_regret_weight=.1), "auxiliary loss config/count proof mismatch")
    values = (metrics["candidate_regret_loss"], last.get("candidate_regret_loss"))
    base._require(all(type(v) in (int, float) and math.isfinite(v) and v >= 0 for v in values), "nonfinite/negative auxiliary loss")
    base._require("candidate_regret_loss" not in evaluation["metrics"], "auxiliary loss must not alter standard metric namespace")
    return {"enabled": True, "weight": .1, "evaluation_mean": values[0], "evaluation_sample_count": 337,
            "last_training_batch_mean": values[1], "definition": "Mean softmax-weighted detached composite-cost regret; not ADE selection regret."}


def summarize_campaign(root: Path) -> dict:
    root = root.absolute()
    base._require(all(not p.is_symlink() for p in (root, *root.parents)), "symlink campaign root")
    report = {"schema": SCHEMA, "status": "INCOMPLETE", "candidate_screen": "NOT_EVALUATED", "absolute_quality": "NOT_EVALUATED",
        "automatic_promotion": False, "vehicle_control_approved": False, "training_data_approved_by_this_report": False,
        "test_evaluated": None, "test_used_for_training_or_selection": None, "pairs": [], "stages": [], "expected_stage_count": 18,
        "limitations": ["Six fresh development fits on unchanged historical v3 data; no new dataset admission or learned closed-loop evidence.",
            "Train1147 and val337 only for optimization/predictions/selection; whole-corpus integrity checks may read held-out test files.",
            "Composite-cost auxiliary regret is not ADE selection regret; only original ADE/FDE/speed/geometry criteria decide the screen.",
            "B total loss includes the new auxiliary term while A does not; raw total losses are not the same objective and cannot establish improvement.",
            "No model/checkpoint tensors are loaded here. Optional local checkpoint bytes are hashed, not decoded.",
            "Frozen plan hash/source bindings are verified; independent preregistration chronology is not established by this result-only reader."]}
    headers = ("plan.json", "status.json")
    missing = [n for n in headers if not (root / n).exists()]
    if missing:
        report["missing_artifacts"] = missing
        return report
    for name in headers:
        path = root / name
        base._require(path.is_file() and all(not p.is_symlink() for p in (path, *path.parents)), "nonregular/symlink header")
    initial = {n: sha_file(root / n) for n in headers}
    plan, state = (base._read(root / n) for n in headers)
    config, sources = validate_plan(plan, state, (root / "plan.json").read_bytes())
    expected = [(f"seed_{seed}/{arm}", stage) for seed in base.SEEDS for arm in ARMS for stage in expansion.STAGE_FILES]
    required = [f"{run}/{expansion.STAGE_FILES[stage]}" for run, stage in expected] + [ARCHIVE]
    report["missing_artifacts"] = [n for n in required if not (root / n).exists()]
    checkpoints = [f"seed_{seed}/{arm}/training/checkpoints/latest.pt" for seed in base.SEEDS for arm in ARMS]
    for name in [*headers, *required, *checkpoints]:
        path = root / name
        if not path.exists() and not path.is_symlink():
            continue
        base._require(path.is_file() and all(not p.is_symlink() for p in (path, *path.parents)), "nonregular/symlink artifact")
        initial.setdefault(name, sha_file(path))
    if ARCHIVE not in report["missing_artifacts"]:
        base._require(initial[ARCHIVE] == state["runner_sha256"], "active runner archive mismatch")
    report.update(campaign_id=CAMPAIGN_ID, source_commit=plan["source_commit"], plan_sha256=initial["plan.json"],
        dataset_manifest_sha256=expansion.MANIFEST_SHA256, corpus_fingerprint_sha256=expansion.CORPUS_SHA256,
        budget={"steps": 1540, "batch_size": 4, "train_samples": 1147, "val_samples": 337, "samples_seen_per_run": 6155,
            "batches_per_epoch": 287, "full_epochs": 5, "partial_epoch_batches": 105, "partial_epoch_samples": 420, "fresh_run_count": 6},
        source_proof={"source_commit": plan["source_commit"], "files": {n: base._sha(v) for n, v in sources.items()},
            "shared_model_config": config, "initial_tensor_hashes_available": False}, runner_status=state.get("status"))
    stages = state.get("stages")
    base._require(isinstance(stages, list) and all(isinstance(r, dict) for r in stages)
        and len(stages) <= 18 and [(r.get("run"), r.get("stage")) for r in stages] == expected[:len(stages)], "18-stage order/scope mismatch")
    previous, deadline = shared._time(state["created_at_utc"]), shared._time(DEADLINE.replace("Z", "+00:00"))
    for index, record in enumerate(stages):
        status = record.get("status")
        base._require(status in ("COMPLETE", "RUNNING", "FAILED"), "unknown stage status")
        base._require(status == "COMPLETE" or index == len(stages) - 1, "later stage follows an unfinished stage")
        started = shared._time(record.get("started_at_utc"))
        reserve = sum(STAGE_TIMEOUTS[s] for _, s in expected[index:]) + 300
        base._require(started >= previous and (deadline - started).total_seconds() >= reserve, "stage chronology/deadline budget mismatch")
        if status != "RUNNING":
            previous = shared._time(record.get("finished_at_utc"))
            base._require(previous >= started, "stage end precedes start")
        if status == "COMPLETE":
            base._require(type(record.get("returncode")) is int and record["returncode"] == 0 and previous <= deadline,
                "completed stage exit/deadline mismatch")
        evaluation = root / record["run"] / "evaluation/metrics.json"
        sha = base._read(evaluation).get("checkpoint_sha256") if record["stage"] == "audit" and evaluation.exists() else None
        if record["stage"] == "audit" and sha is None:
            sha = base._option(record.get("command", []), "--checkpoint-sha256")
        command = expected_command(record, sha)
        expansion._validate_stage(record, root, state, complete=status == "COMPLETE")
        report["stages"].append({"run": record["run"], "stage": record["stage"], "status": status,
            "returncode": record.get("returncode"), "command": command, "report": record.get("report"),
            "recorded_command_sha256": base._sha(canonical(record["command"]).encode()),
            "started_at_utc": record["started_at_utc"], "finished_at_utc": record.get("finished_at_utc")})
    report["completed_stage_count"] = sum(r["status"] == "COMPLETE" for r in stages)
    report["not_started_stages"] = [{"run": run, "stage": stage, "status": "NOT_RUN"} for run, stage in expected[len(stages):]]
    complete = not report["missing_artifacts"] and len(stages) == 18 and all(r["status"] == "COMPLETE" for r in stages)
    if complete and state.get("status") == "TRAIN_EVAL_AUDIT_COMPLETE_NOT_PROMOTED":
        comparison = base.compare_reports([root / f"seed_{seed}/{arm}/evaluation/metrics.json" for seed in base.SEEDS for arm in ARMS])
        episode_sets, sampling_plans = set(), set()
        for index, seed in enumerate(base.SEEDS):
            paired = []
            for offset, arm in enumerate(ARMS):
                relative = f"seed_{seed}/{arm}"
                training = base._read(root / relative / "training/run.json")
                evaluation = base._read(root / relative / "evaluation/metrics.json")
                row = comparison["reports"][2 * index + offset]
                row.update(report=relative + "/evaluation/metrics.json", checkpoint=relative + "/training/checkpoints/latest.pt", label=relative)
                run = base._validate_run(root, relative, plan, config, seed, arm, source_bytes=sources)
                base._require(row["model_parameter_count"] == 954590 and row["training_domain_samples_seen"] == {"carla": 6155},
                    "parameter count or 6155-exposure mismatch")
                episode_sets.add(tuple(sorted(training["training_episode_ids"])))
                sampling_plans.add(row["training_sampling_plan_sha256"])
                run.update(metrics=row["metrics"], auxiliary_diagnostics=auxiliary_diagnostics(training, evaluation, arm))
                run["absolute_checks"] = {k: k in row["metrics"] and row["metrics"][k] <= v for k, v in base.ABSOLUTE_LIMITS.items()}
                run["absolute_quality"] = "PASS" if all(run["absolute_checks"].values()) else "FAIL"
                paired.append(run)
            a, b = paired
            checks = {"selected_ade_improves": b["metrics"]["selected_ade_m"] < a["metrics"]["selected_ade_m"],
                "selected_fde_improves": b["metrics"]["selected_fde_m"] < a["metrics"]["selected_fde_m"],
                "geometry_does_not_regress": b["geometry"]["selected_pass_count"] >= a["geometry"]["selected_pass_count"],
                "speed_mae_within_5_percent": b["metrics"]["selected_speed_mae_mps"] <= 1.05 * a["metrics"]["selected_speed_mae_mps"]}
            report["pairs"].append({"seed": seed, "baseline": a, "candidate": b, "relative_checks": checks,
                "absolute_checks": b["absolute_checks"], "candidate_screen": "PASS" if all(checks.values()) else "FAIL", "absolute_quality": b["absolute_quality"]})
        base._require(len(episode_sets) == len(sampling_plans) == 1, "paired episode/sampling plans differ")
        report.update(status="COMPLETE_NOT_PROMOTED", test_evaluated=False, test_used_for_training_or_selection=False,
            candidate_screen="PASS" if all(p["candidate_screen"] == "PASS" for p in report["pairs"]) else "FAIL",
            absolute_quality="PASS" if all(p["absolute_quality"] == "PASS" for p in report["pairs"]) else "FAIL",
            absolute_limits_m=base.ABSOLUTE_LIMITS, comparison=comparison)
    base._require(all(sha_file(root / name) == digest for name, digest in initial.items()), "campaign inputs changed during summary")
    report["input_manifest"] = [{"path": n, "sha256": v} for n, v in sorted(initial.items())]
    return report


def render_markdown(report):
    return base.render_markdown(report).replace("학습·검증 3-seed A/B", "Composite-regret 보조 손실 · 새 학습 3-seed A/B").replace(
        "이번 캠페인은 validation만 사용하며 독립 test는 열지 않습니다.",
        "Train1147 학습·val337 평가만 사용합니다. 전체 corpus 무결성 검사에서 test 파일을 읽을 수 있지만 test 추론·최적화·모델 선택은 금지됩니다.") + (
        "\nA λ=0 / B λ=0.1, 각각 새 학습 1540 step·6155 sample 노출입니다. 보조 손실은 composite-cost regret이며 ADE regret과 다릅니다. "
        "B total loss에만 새 항이 있으므로 A/B total loss 감소를 같은 목적함수의 개선으로 해석하면 안 됩니다. "
        "기존 기준으로 세 seed를 모두 비교하고 자동 승격·새 데이터 승인·자율주행 완료를 주장하지 않습니다.\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("campaign_root", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        output = args.output_dir.absolute()
        base._require(not output.exists() and all(not p.is_symlink() for p in (output, *output.parents)), "output must be fresh without symlinks")
        base._require(not any(output.resolve().is_relative_to(p) for p in
            (args.campaign_root.resolve(), base.REPO / "datasets", (base.REPO / "datasets").resolve())), "output overlaps input/datasets")
        report = summarize_campaign(args.campaign_root)
        payload = json.dumps(report, indent=2, allow_nan=False) + "\n"
        base._require(not any(token in payload for token in ("/home/", "/root/", "/tmp/", "ssh://", "-----BEGIN ")), "private path/key token in output")
        output.mkdir(parents=True, exist_ok=False)
        for name, value in (("summary.json", payload), ("README.md", render_markdown(report))):
            with (output / name).open("x") as stream:
                stream.write(value)
        with (output / "SHA256SUMS").open("x") as stream:
            stream.write("".join(f"{sha_file(output / n)}  {n}\n" for n in ("summary.json", "README.md")))
    except (ContractError, ValueError, OSError, TypeError, KeyError, IndexError) as error:
        print(f"CANDIDATE_REGRET_SUMMARY_REJECTED: {type(error).__name__}; no completion asserted")
        return 2
    print(json.dumps({"status": report["status"], "candidate_screen": report["candidate_screen"]}))
    return 0 if report["status"] == "COMPLETE_NOT_PROMOTED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
