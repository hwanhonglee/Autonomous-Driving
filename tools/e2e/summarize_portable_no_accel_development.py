#!/usr/bin/env python3
"""HH_260906 - Verify a fresh paired input-ablation study without approving data, a model or vehicle control."""

from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re

from scripts.e2e import summarize_portable_data_expansion as expansion
from scripts.e2e import summarize_portable_training_campaign as base
from portable_e2e.contract import ContractError, _loads_json

SCHEMA = "portable_e2e.no_accel_development_summary.v1"
PLAN_SCHEMA = "portable_e2e.no_accel_development_campaign.v1"
CAMPAIGN_ID = "hh260909-no-accel-development-ab-3seeds-v1"
SOURCE_COMMIT = "213cfc69e3551b315dc3a2f618387027ff5948f6"
ARMS = {"A_physical_input": 0.0001, "B_no_accel_input": 0.0001}
MODELS = {"A_physical_input": "portable_e2e/config/perspective_trajectory_physical_v1.model.json",
    "B_no_accel_input": "portable_e2e/config/perspective_trajectory_physical_no_accel_v1.model.json"}
MODEL_IDS = {"A_physical_input": "portable_e2e.perspective_trajectory.physical.v1",
    "B_no_accel_input": "portable_e2e.perspective_trajectory.physical_no_accel.v1"}
MODEL_SHA256 = {"A_physical_input": "e96e31c96cafa41b57b67b9531ae9fff6bf21fd7e418ea78f9062fcb1dcfe74d",
    "B_no_accel_input": "0f941332046854d239d676e83a4ccbc22e60436f3894dcbfe6871002899ed012"}
DATASET = "datasets/prepared/carla-common10-30kph-five-episodes-20260907-v3"
WORKER = "scripts/e2e/run_portable_training_campaign.py"
ARCHIVE = "provenance/active_runner.py"
CORE_FILES = ("portable_e2e/model.py", "portable_e2e/losses.py", "portable_e2e/train.py",
    "portable_e2e/evaluate.py", "portable_e2e/audit_runtime.py", "portable_e2e/runtime_contract.py",
    "portable_e2e/contract.py", "portable_e2e/dataset.py", "portable_e2e/torch_dataset.py",
    "portable_e2e/runtime_weight_bundle.py", WORKER, *MODELS.values())


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def sha_file(path):
    # HH_260906 - Hash optional checkpoints in chunks without loading a checkpoint or importing a model.
    hasher = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            hasher.update(chunk)
    return hasher.hexdigest()


def validate_plan(plan, state, payload):
    fixed = {"schema": PLAN_SCHEMA, "campaign_id": CAMPAIGN_ID, "source_commit": SOURCE_COMMIT,
        "gpu_uuid": "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5", "dataset": DATASET,
        "dataset_manifest_sha256": expansion.MANIFEST_SHA256, "model_configs": MODELS,
        "seeds": base.SEEDS, "arms": ARMS, "steps": 1540, "batch_size": 4, "split": "val",
        "expected_train_samples": 1147, "expected_val_samples": 337}
    base._require(all(canonical(plan.get(key)) == canonical(value) for key, value in fixed.items())
        and list(plan["arms"]) == list(ARMS), "no-accel plan differs from reviewed matrix")
    base._require(plan.get("candidate_score_weight", .1) == .1 and not any(key in plan for key in
        ("model_config", "prerequisite_campaign_id", "baseline_campaign_id", "baseline_source_commit",
         "prerequisite_timeout_seconds", "resume", "checkpoint")), "not the fresh unchanged-loss experiment")
    commit = plan.get("source_commit", "")
    base._require(isinstance(commit, str) and re.fullmatch(r"[0-9a-f]{40}", commit) is not None,
        "source commit must be pinned")
    base._require(state.get("source_commit") == commit and state.get("plan") == plan
        and state.get("plan_sha256") == base._sha(payload), "status frozen-plan/source mismatch")
    base._require(plan.get("decision", {}).get("absolute_limits_m") ==
        {key.removesuffix("_m"): value for key, value in base.ABSOLUTE_LIMITS.items()}, "absolute criteria changed")
    base._require(state.get("dataset_manifest_sha256") == expansion.MANIFEST_SHA256
        and state.get("reviewed_contract") == {"train_samples": 1147, "val_samples": 337, "run_count": 6, "stage_count": 18},
        "dataset/denominator proof mismatch")
    base._require(state.get("vehicle_control_approved") is False and state.get("prerequisite") is None,
        "unexpected control approval or predecessor")
    configs, sources = {}, {}
    for name in CORE_FILES:
        sources[name] = base._git_bytes(commit, name)
    base._require(state.get("runner_sha256") == base._sha(sources[WORKER]), "recorded runner is not the pinned Git source")
    base._require(set(state.get("model_configs", {})) == set(ARMS), "per-arm model provenance missing")
    for arm, name in MODELS.items():
        raw = sources[name]
        config = _loads_json(raw.decode(), "pinned model config")
        configs[arm] = config
        base._require(base._sha(raw) == MODEL_SHA256[arm] and config.get("model_id") == MODEL_IDS[arm],
            "unreviewed per-arm model config/ID")
        base._require(state["model_configs"][arm] == {"path": name, "sha256": base._sha(raw),
            "canonical_sha256": base._sha(canonical(config).encode()), "model_config": config}, "per-arm config proof mismatch")
    a, b = (configs[arm] for arm in ARMS)
    base._require(b == {**a, "model_id": MODEL_IDS["B_no_accel_input"]}, "model configs differ beyond model ID")
    return configs, sources


def expected_command(record, plan, checkpoint_sha):
    # HH_260906 - Normalize only the declared personal workspace prefix; every remaining argument is exact.
    command = record.get("command")
    base._require(isinstance(command, list) and all(isinstance(item, str) for item in command) and command, "invalid recorded command")
    python = Path(command[0])
    base._require(python.is_absolute() and ".." not in python.parts
        and python.parts[-4:] == ("venvs", "py312", "bin", "python"),
        "recorded interpreter is not the personal py312 venv")
    workspace = python.parents[3]
    base._require(workspace.parts[-3:] == ("personal", "hwanhong", "portable_e2e"), "unreviewed personal workspace layout")
    item = workspace / "runs/campaigns" / CAMPAIGN_ID / record["run"]
    dataset = str(workspace / DATASET)
    arm = record["run"].split("/")[1]
    seed = record["run"].split("/")[0].removeprefix("seed_")
    common = ["--device", "cuda:0", "--batch-size", "4"]
    if record["stage"] == "train":
        expected = [str(python), "-m", "portable_e2e.train", dataset, "--run-dir", str(item / "training"),
            "--model-config", str(workspace / "autoware_e2e" / MODELS[arm]), "--split", "train", *common,
            "--seed", seed, "--learning-rate", "0.0001", "--weight-decay", "0.0001", "--max-steps", "1540",
            "--checkpoint-interval", "154", "--num-workers", "0", "--maximum-gradient-norm", "5",
            "--sampling-policy", "uniform_without_replacement"]
    elif record["stage"] == "evaluate":
        expected = [str(python), "-m", "portable_e2e.evaluate", dataset, "--checkpoint", str(item / "training/checkpoints/latest.pt"),
            "--output-dir", str(item / "evaluation"), "--split", "val", *common, "--num-workers", "0", "--render-count", "12"]
    else:
        base._require(isinstance(checkpoint_sha, str) and re.fullmatch(r"[0-9a-f]{64}", checkpoint_sha) is not None,
            "audit command checkpoint hash missing")
        expected = [str(python), "-m", "portable_e2e.audit_runtime", dataset, "--checkpoint", str(item / "training/checkpoints/latest.pt"),
            "--output-json", str(item / "gate_v8.json"), "--split", "val", *common, "--checkpoint-sha256", checkpoint_sha]
    base._require(command == expected, "recorded command differs from exact fresh train/val protocol")
    return [value.replace(str(workspace), "<PERSONAL_WORKSPACE>") for value in command]


def _time(value):
    base._require(isinstance(value, str), "missing stage timestamp")
    parsed = datetime.fromisoformat(value)
    base._require(parsed.utcoffset() is not None, "timestamp must include timezone")
    return parsed


def summarize_campaign(root: Path) -> dict:
    root = root.absolute()
    base._require(all(not path.is_symlink() for path in (root, *root.parents)), "symlink campaign root")
    report = {"schema": SCHEMA, "status": "INCOMPLETE", "candidate_screen": "NOT_EVALUATED",
        "absolute_quality": "NOT_EVALUATED", "automatic_promotion": False, "vehicle_control_approved": False,
        "training_data_approved_by_this_report": False, "test_evaluated": None,
        "test_used_for_training_or_selection": None, "pairs": [], "stages": [], "expected_stage_count": 18,
        "limitations": ["Exploratory development on historical v3 data, not qualification or admission of new captures.",
            "Full-corpus contract/hash validation may inspect held-out test files; training, inference, metrics and selection use train/val only.",
            "A uses supplied acceleration; B zeros channels 3/4 at every ego-history time inside the model. This is not a physical acceleration correction.",
            "Equal config shapes/parameter counts and pinned shared source are checked; no stored initialization tensors are available to hash.",
            "The no-accel research ID is not added to the secure runtime-bundle allowlist. No learned driving or live 10 Hz performance is established.",
            "This summarizer reads result artifacts only; it does not rehash the underlying corpus or evaluate the test split."]}
    header = ("plan.json", "status.json")
    missing = [name for name in header if not (root / name).exists()]
    if missing:
        report["missing_artifacts"] = missing
        return report
    initial = {name: sha_file(root / name) for name in header}
    plan, state = (base._read(root / name) for name in header)
    configs, sources = validate_plan(plan, state, (root / "plan.json").read_bytes())
    expected = [(f"seed_{seed}/{arm}", stage) for seed in base.SEEDS for arm in ARMS for stage in expansion.STAGE_FILES]
    required = [f"{relative}/{expansion.STAGE_FILES[stage]}" for relative, stage in expected]
    report.update(campaign_id=CAMPAIGN_ID, source_commit=plan["source_commit"], plan_sha256=initial["plan.json"],
        dataset_manifest_sha256=expansion.MANIFEST_SHA256, corpus_fingerprint_sha256=expansion.CORPUS_SHA256,
        budget={"steps": 1540, "batch_size": 4, "train_samples": 1147, "val_samples": 337,
            "batches_per_epoch": 287, "approximate_epochs": 1540 / 287, "fresh_initialization_planned": True},
        model_configs=configs, source_proof={"source_commit": plan["source_commit"],
            "files": {name: base._sha(raw) for name, raw in sources.items()},
            "shared_config_except_model_id": True, "initial_tensor_hashes_available": False})
    report["missing_artifacts"] = [name for name in [*required, ARCHIVE] if not (root / name).exists()]
    tracked = [*header, *[name for name in [*required, ARCHIVE] if (root / name).exists()]]
    checkpoints = [f"seed_{seed}/{arm}/training/checkpoints/latest.pt" for seed in base.SEEDS for arm in ARMS]
    tracked.extend(name for name in checkpoints if (root / name).exists() or (root / name).is_symlink())
    for name in tracked:
        path = root / name
        base._require(path.is_file() and all(not p.is_symlink() for p in (path, *path.parents)), "nonregular/symlink campaign artifact")
        initial.setdefault(name, sha_file(path))
    if ARCHIVE not in report["missing_artifacts"]:
        base._require(initial[ARCHIVE] == state["runner_sha256"], "archived worker differs from executed/source pin")
    stages = state.get("stages")
    base._require(isinstance(stages, list) and all(isinstance(record, dict) for record in stages), "invalid stage ledger")
    base._require(len(stages) <= 18 and [(r.get("run"), r.get("stage")) for r in stages] == expected[:len(stages)],
        "18-stage order/scope mismatch")
    report["not_started_stages"] = [{"run": relative, "stage": stage, "status": "NOT_RUN"}
        for relative, stage in expected[len(stages):]]
    report["completed_stage_count"] = sum(record.get("status") == "COMPLETE" for record in stages)
    previous = _time(state["created_at_utc"])
    for record in stages:
        status = record.get("status")
        base._require(status in ("RUNNING", "FAILED", "COMPLETE"), "invalid stage status")
        started = _time(record.get("started_at_utc"))
        base._require(started >= previous, "stage chronology regressed")
        if status != "RUNNING":
            previous = _time(record.get("finished_at_utc"))
            base._require(previous >= started, "stage ended before starting")
        else:
            previous = started
        if status == "COMPLETE":
            base._require(type(record.get("returncode")) is int and record["returncode"] == 0, "completed stage has failed exit status")
        metrics_path = root / record["run"] / "evaluation/metrics.json"
        checkpoint_sha = base._read(metrics_path).get("checkpoint_sha256") if record["stage"] == "audit" and metrics_path.exists() else None
        if record["stage"] == "audit" and checkpoint_sha is None:
            checkpoint_sha = base._option(record.get("command", []), "--checkpoint-sha256")
        normalized = expected_command(record, plan, checkpoint_sha)
        expansion._validate_stage(record, root, state, complete=status == "COMPLETE")
        report["stages"].append({"run": record["run"], "stage": record["stage"], "status": status,
            "returncode": record.get("returncode"), "command": normalized,
            "recorded_command_sha256": base._sha(canonical(record["command"]).encode()),
            "report": record.get("report"), "started_at_utc": record["started_at_utc"], "finished_at_utc": record.get("finished_at_utc")})
    complete = not report["missing_artifacts"] and len(stages) == 18 and state.get("status") == "TRAIN_EVAL_AUDIT_COMPLETE_NOT_PROMOTED"
    complete = complete and all(r["status"] == "COMPLETE" for r in stages)
    report["runner_status"] = state.get("status")
    if complete:
        evaluations = [root / f"seed_{seed}/{arm}/evaluation/metrics.json" for seed in base.SEEDS for arm in ARMS]
        comparison = base.compare_reports(evaluations)
        for index, (seed, arm) in enumerate((seed, arm) for seed in base.SEEDS for arm in ARMS):
            row = comparison["reports"][index]
            relative = f"seed_{seed}/{arm}"
            row["report"] = relative + "/evaluation/metrics.json"
            row["checkpoint"] = relative + "/training/checkpoints/latest.pt"
            row["label"] = relative
        counts, episode_sets, sampling_plans = set(), set(), set()
        for index, seed in enumerate(base.SEEDS):
            paired = []
            for offset, arm in enumerate(ARMS):
                run = base._validate_run(root, f"seed_{seed}/{arm}", plan, configs[arm], seed, arm)
                row = comparison["reports"][2 * index + offset]
                counts.add(row["model_parameter_count"])
                training = base._read(root / f"seed_{seed}/{arm}/training/run.json")
                episode_sets.add(tuple(sorted(training["training_episode_ids"])))
                sampling_plans.add(row["training_sampling_plan_sha256"])
                base._require(row["training_domain_samples_seen"] == {"carla": 6155}, "not the full fixed 6155-sample training exposure")
                run["metrics"] = row["metrics"]
                run["absolute_checks"] = {key: key in row["metrics"] and row["metrics"][key] <= value for key, value in base.ABSOLUTE_LIMITS.items()}
                run["absolute_quality"] = "PASS" if all(run["absolute_checks"].values()) else "FAIL"
                paired.append(run)
            a, b = paired
            checks = {"selected_ade_improves": b["metrics"]["selected_ade_m"] < a["metrics"]["selected_ade_m"],
                "selected_fde_improves": b["metrics"]["selected_fde_m"] < a["metrics"]["selected_fde_m"],
                "geometry_does_not_regress": b["geometry"]["selected_pass_count"] >= a["geometry"]["selected_pass_count"],
                "speed_mae_within_5_percent": b["metrics"]["selected_speed_mae_mps"] <= 1.05 * a["metrics"]["selected_speed_mae_mps"]}
            report["pairs"].append({"seed": seed, "baseline": a, "candidate": b, "relative_checks": checks,
                "absolute_checks": b["absolute_checks"], "candidate_screen": "PASS" if all(checks.values()) else "FAIL",
                "absolute_quality": b["absolute_quality"]})
        base._require(len(counts) == 1 and type(next(iter(counts))) is int and next(iter(counts)) > 0,
            "paired parameter counts differ")
        base._require(len(episode_sets) == len(sampling_plans) == 1, "paired training episodes or sampling plan differ")
        report["source_proof"]["shared_parameter_count"] = next(iter(counts))
        report.update(status="COMPLETE_NOT_PROMOTED", test_evaluated=False, test_used_for_training_or_selection=False,
            candidate_screen="PASS" if all(pair["candidate_screen"] == "PASS" for pair in report["pairs"]) else "FAIL",
            absolute_quality="PASS" if all(pair["absolute_quality"] == "PASS" for pair in report["pairs"]) else "FAIL",
            absolute_limits_m=base.ABSOLUTE_LIMITS, comparison=comparison)
    base._require(all(sha_file(root / name) == digest for name, digest in initial.items()), "campaign artifacts changed during summary")
    report["input_manifest"] = [{"path": name, "sha256": digest} for name, digest in sorted(initial.items())]
    return report


def render_markdown(report):
    return base.render_markdown(report).replace("학습·검증 3-seed A/B", "제공 가속도 입력 유지/제거 · 새 학습 3-seed A/B").replace(
        "이번 캠페인은 validation만 사용하며 독립 test는 열지 않습니다.",
        "학습은 train1147, 평가는 val337입니다. 전체 corpus 무결성 검사에서 test 파일을 읽을 수 있으나 test 추론·학습·모델 선택은 하지 않습니다.") + (
        "\nID 외 설정 일치와 실제 보고된 파라미터 수의 검증 결과는 JSON source_proof에 기록합니다. 실제 초기 텐서 해시는 저장되지 않았습니다. "
        "기존 v3 개발용 데이터에서 같은 1540 step(약 5.37 epoch)으로 새로 학습하는 탐색적 비교이며, 새 데이터 승인이나 실차 준비 완료를 뜻하지 않습니다.\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("campaign_root", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        output = args.output_dir.absolute()
        base._require(not output.exists() and all(not p.is_symlink() for p in (output, *output.parents)), "output must be fresh without symlinks")
        base._require(not any(output.resolve().is_relative_to(path) for path in
            (args.campaign_root.resolve(), base.REPO / "datasets", (base.REPO / "datasets").resolve())), "output overlaps inputs/datasets")
        report = summarize_campaign(args.campaign_root)
        payload = json.dumps(report, indent=2, allow_nan=False) + "\n"
        base._require(not any(token in payload for token in ("/home/", "/root/", "/tmp/", "ssh://", "-----BEGIN ")),
            "public summary contains a private path/key token")
        output.mkdir(parents=True, exist_ok=False)
        for name, value in (("summary.json", payload), ("README.md", render_markdown(report))):
            with (output / name).open("x") as stream:
                stream.write(value)
        with (output / "SHA256SUMS").open("x") as stream:
            stream.write("".join(f"{sha_file(output / name)}  {name}\n" for name in ("summary.json", "README.md")))
    except (ContractError, ValueError, OSError, TypeError, KeyError, IndexError) as error:
        print(f"NO_ACCEL_SUMMARY_REJECTED: {type(error).__name__}; no completion is asserted")
        return 2
    print(json.dumps({"status": report["status"], "candidate_screen": report["candidate_screen"]}))
    return 0 if report["status"] == "COMPLETE_NOT_PROMOTED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
