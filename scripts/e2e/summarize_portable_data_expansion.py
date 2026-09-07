#!/usr/bin/env python3
"""HH_260906 - Verify the three-seed expanded-corpus campaign without cross-corpus promotion."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from scripts.e2e import summarize_portable_training_campaign as base
from portable_e2e.contract import ContractError, _loads_json

SCHEMA = "portable_e2e.data_expansion_summary.v1"
MANIFEST_SHA256 = "18262e5aa4abbb3e03e35e379b5da1e5ce7fd339a9a8942e02b58ca737f7242c"
CORPUS_SHA256 = "56d9ff663612a090cd61698b53cf0cf39b0a7ae0de6b42d096957a94b6c92047"
ARM = "C_expanded_data"
STAGE_FILES = {"train": "training/run.json", "evaluate": "evaluation/metrics.json", "audit": "gate_v8.json"}


def _validate_plan(plan: dict, state: dict, payload: bytes) -> dict:
    # HH_260906 - Freeze the second corpus and budget independently of the original LR study.
    fixed = {
        "schema": "portable_e2e.data_expansion_campaign.v1",
        "campaign_id": "hh260907-physical-v1-data-expansion-3seeds-v1",
        "source_commit": "081a71f7fc014d2864b790b0b0cae7378ae18e4c",
        "gpu_uuid": "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5",
        "dataset": "datasets/prepared/carla-common10-30kph-five-episodes-20260907-v3",
        "dataset_manifest_sha256": MANIFEST_SHA256,
        "model_config": "portable_e2e/config/perspective_trajectory_physical_v1.model.json",
        "seeds": base.SEEDS, "arms": {ARM: 0.0001}, "steps": 1540, "batch_size": 4,
        "split": "val", "expected_train_samples": 1147, "expected_val_samples": 337,
        "prerequisite_campaign_id": "hh260907-physical-v1-lr-ab-3seeds-v1",
        "prerequisite_timeout_seconds": 3600,
    }
    base._require(all(plan.get(name) == value for name, value in fixed.items()), "expanded plan differs from reviewed scope")
    base._require(state.get("plan") == plan and state.get("source_commit") == fixed["source_commit"], "status plan/source mismatch")
    base._require(state.get("plan_sha256") == base._sha(payload), "frozen plan SHA-256 mismatch")
    base._require(state.get("dataset_manifest_sha256") == MANIFEST_SHA256, "dataset manifest proof mismatch")
    base._require(state.get("reviewed_contract") == {"train_samples": 1147, "val_samples": 337, "run_count": 3, "stage_count": 9},
                  "reviewed sample/run/stage contract mismatch")
    base._require(state.get("vehicle_control_approved") is False, "campaign cannot approve control")
    runner_sha = state.get("runner_sha256", "")
    base._require(isinstance(runner_sha, str) and re.fullmatch(r"[0-9a-f]{64}", runner_sha) is not None, "missing runner hash declaration")
    prerequisite = state.get("prerequisite", {})
    base._require(prerequisite.get("status") == "TRAIN_EVAL_AUDIT_COMPLETE_NOT_PROMOTED" and prerequisite.get("completed_stages") == 18,
                  "predecessor completion proof missing")
    prerequisite_sha = prerequisite.get("sha256", "")
    base._require(isinstance(prerequisite_sha, str) and re.fullmatch(r"[0-9a-f]{64}", prerequisite_sha) is not None,
                  "predecessor status hash missing")
    config_bytes = base._git_bytes(plan["source_commit"], plan["model_config"])
    base._require(state.get("model_config_sha256") == base._sha(config_bytes), "model-config byte proof mismatch")
    return _loads_json(config_bytes.decode("utf-8"), "pinned model config")


def _validate_stage(record: dict, root: Path, state: dict, *, complete: bool) -> Path:
    relative, stage = record["run"], record["stage"]
    command = record.get("command", [])
    base._require(isinstance(command, list) and all(isinstance(value, str) for value in command), "invalid recorded command")
    module = "audit_runtime" if stage == "audit" else stage
    base._require(base._option(command, "-m") == f"portable_e2e.{module}", "unexpected stage module")
    base._require(base._option(command, "--split") == ("train" if stage == "train" else "val"), "test or unreviewed split opened")
    base._require(base._option(command, "--device") == "cuda:0" and base._option(command, "--batch-size") == "4", "unreviewed device/batch")
    if stage == "train":
        seed = relative.split("/")[0].removeprefix("seed_")
        base._require(base._option(command, "--seed") == seed and float(base._option(command, "--learning-rate")) == 0.0001
                      and base._option(command, "--max-steps") == "1540" and "--resume" not in command,
                      "unreviewed seed/LR/step budget")
    path = root / relative / STAGE_FILES[stage]
    if not complete or not path.exists():
        return path
    proof = record.get("report", {})
    base._require(proof.get("path") == STAGE_FILES[stage] and proof.get("sha256") == base._sha(path.read_bytes()),
                  "per-stage report SHA-256 proof mismatch")
    report = base._read(path)
    split_key = "train_fingerprint_sha256" if stage == "train" else "val_fingerprint_sha256"
    base._require(report.get("corpus_fingerprint_sha256") == proof.get("corpus_fingerprint_sha256") == state.get("corpus_fingerprint_sha256") == CORPUS_SHA256,
                  "expanded corpus fingerprint proof mismatch")
    base._require(report.get("dataset_fingerprint_sha256") == proof.get("dataset_fingerprint_sha256") == state.get(split_key),
                  "stage split fingerprint proof mismatch")
    return path


def summarize_campaign(root: Path) -> dict:
    root = root.resolve(strict=True)
    report = {"schema": SCHEMA, "status": "INCOMPLETE", "absolute_quality": "NOT_EVALUATED",
              "automatic_promotion": False, "vehicle_control_approved": False,
              "test_opened_by_this_campaign": None, "runs": [],
              "cross_corpus_comparison": {"status": "NOT_PERFORMED", "automatic_pass": False,
                  "reason": "Old v2 and expanded v3 have different corpora; compare.py is used only among the three v3 runs."},
              "limitations": ["Validation development only; independent test, shadow and closed-loop gates remain outstanding.",
                  "Runner/predecessor hashes and dataset-manifest SHA are recorded worker proofs; their underlying files are not rehashed here."]}
    missing_header = [name for name in ("plan.json", "status.json") if not (root / name).exists()]
    if missing_header:
        report["missing_artifacts"] = missing_header
        return report
    plan, state = base._read(root / "plan.json"), base._read(root / "status.json")
    config = _validate_plan(plan, state, (root / "plan.json").read_bytes())
    report.update(campaign_id=plan["campaign_id"], source_commit=plan["source_commit"],
                  dataset_manifest_sha256=MANIFEST_SHA256, corpus_fingerprint_sha256=CORPUS_SHA256,
                  budget={"optimizer_steps": 1540, "batch_size": 4, "training_samples": 1147,
                          "batches_per_epoch": 287, "approximate_epochs": 1540 / 287,
                          "description": "Same optimizer-step budget, approximately 5.37 epochs; not 10 epochs."})
    expected = [(f"seed_{seed}/{ARM}", stage) for seed in base.SEEDS for stage in STAGE_FILES]
    stages = state.get("stages", [])
    base._require(isinstance(stages, list) and all(isinstance(record, dict) for record in stages), "invalid stage list")
    base._require([(record.get("run"), record.get("stage")) for record in stages] == expected[:len(stages)], "nine-stage order/scope mismatch")
    required = [root / relative / STAGE_FILES[stage] for relative, stage in expected]
    report["missing_artifacts"] = [str(path.relative_to(root)) for path in required if not path.exists()]
    for record in stages:
        _validate_stage(record, root, state, complete=record.get("status") == "COMPLETE")
    if (report["missing_artifacts"] or len(stages) != 9 or state.get("status") != "TRAIN_EVAL_AUDIT_COMPLETE_NOT_PROMOTED"
            or any(record.get("status") != "COMPLETE" or record.get("returncode") != 0 for record in stages)):
        report["runner_status"] = state.get("status")
        return report
    evaluations = [root / f"seed_{seed}/{ARM}/evaluation/metrics.json" for seed in base.SEEDS]
    comparison = base.compare_reports(evaluations)
    for seed, row in zip(base.SEEDS, comparison["reports"]):
        run = base._validate_run(root, f"seed_{seed}/{ARM}", plan, config, seed, ARM)
        run["metrics"] = row["metrics"]
        run["absolute_checks"] = {name: name in row["metrics"] and row["metrics"][name] <= value for name, value in base.ABSOLUTE_LIMITS.items()}
        run["absolute_quality"] = "PASS" if all(run["absolute_checks"].values()) else "FAIL"
        report["runs"].append(run)
    report.update(status="COMPLETE_NOT_PROMOTED", test_opened_by_this_campaign=False,
                  absolute_quality="PASS" if all(run["absolute_quality"] == "PASS" for run in report["runs"]) else "FAIL",
                  absolute_limits_m=base.ABSOLUTE_LIMITS, comparison=comparison,
                  input_manifest=[{"path": str(path.relative_to(root)), "sha256": base._sha(path.read_bytes())}
                                  for path in [root / "plan.json", root / "status.json", *required]])
    return report


def render_markdown(report: dict) -> str:
    lines = ["# Portable E2E 추가 우회전 데이터 학습 · 3개 seed", "", f"상태: `{report['status']}` · 절대 품질: `{report['absolute_quality']}`", "",
             "Train 1147 / Town03 val337, 1540 optimizer step: 약 5.37 epoch입니다. 모델 자동 승격·차량 제어 승인은 없습니다.",
             "Town04 test는 이번 캠페인에서 평가하지 않습니다. 기존 v2와 v3 간 자동 PASS 판정이나 공정 비교 승인은 하지 않습니다.", ""]
    if report.get("missing_artifacts"):
        lines.extend(["미수집 자료:", "", *[f"- `{name}`" for name in report["missing_artifacts"]], ""])
    if report["runs"]:
        lines.extend(["| Seed | ADE m | FDE m | Oracle ADE m | Selection regret m | Speed MAE m/s | Geometry pass | c0–c5 선택수 | 절대 품질 |",
                      "| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |"])
    for run in report["runs"]:
        metrics, geometry = run["metrics"], run["geometry"]
        values = [f"{metrics[name]:.5f}" for name in ("selected_ade_m", "selected_fde_m", "oracle_ade_m", "selection_regret_ade_m", "selected_speed_mae_mps")]
        histogram = ", ".join(str(geometry["selection_histogram"][str(index)]) for index in range(6))
        lines.append(f"| {run['seed']} | " + " | ".join(values) + f" | {geometry['selected_pass_count']}/337 | {histogram} | {run['absolute_quality']} |")
    for run in report["runs"]:
        lines.extend(["", f"Seed {run['seed']} 절대 기준:", ""])
        lines.extend(f"- {name}: {'PASS' if passed else 'FAIL'}" for name, passed in run["absolute_checks"].items())
    lines.extend(["", "절대 기준: ADE 1/3/6.4초 ≤ 0.5/1/2 m, FDE 6.4초 ≤ 4 m. Selection regret는 경로 다양성 지표가 아닙니다.",
                  "Checkpoint byte 검증 여부는 JSON에 명시합니다. 미다운로드 checkpoint는 평가·감사 간 hash 일치까지만 확인합니다.", ""])
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("campaign_root", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        report = summarize_campaign(args.campaign_root)
        args.output_dir.mkdir(parents=True, exist_ok=False)
        for name, payload in (("summary.json", json.dumps(report, indent=2, allow_nan=False) + "\n"), ("README.md", render_markdown(report))):
            with (args.output_dir / name).open("x", encoding="utf-8") as stream:
                stream.write(payload)
    except (ContractError, OSError, ValueError, TypeError, KeyError, IndexError) as error:
        print(f"EXPANSION_SUMMARY_ERROR: {error}", file=sys.stderr)
        return 2
    print(json.dumps({"status": report["status"], "absolute_quality": report["absolute_quality"], "output": str(args.output_dir)}))
    return 0 if report["status"] == "COMPLETE_NOT_PROMOTED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
