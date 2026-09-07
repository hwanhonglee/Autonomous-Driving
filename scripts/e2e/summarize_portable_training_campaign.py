#!/usr/bin/env python3
"""HH_260906 - Summarize a frozen six-run development campaign without promoting a model."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
from typing import Any

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from portable_e2e.compare import compare_reports
from portable_e2e.contract import ContractError, _loads_json
from portable_e2e.runtime_contract import RUNTIME_GATE_ID, RuntimeGateConfig

SCHEMA = "portable_e2e.training_campaign_summary.v1"
SEEDS = [20260903, 20260904, 20260905]
ARMS = {"A_baseline": 0.0001, "B_lower_lr": 0.00003}
ABSOLUTE_LIMITS = {
    "ade_1p0s_m": 0.5, "ade_3p0s_m": 1.0,
    "ade_6p4s_m": 2.0, "fde_6p4s_m": 4.0,
}


def _sha(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ContractError(message)


def _read(path: Path) -> dict[str, Any]:
    _require(path.is_file() and not path.is_symlink(), f"not a regular file: {path}")
    value = _loads_json(path.read_text(encoding="utf-8"), str(path))
    _require(isinstance(value, dict), f"not a JSON object: {path}")
    return value


def _git_bytes(commit: str, path: str) -> bytes:
    # HH_260906 - Verify source provenance against the pinned commit rather than mutable working files.
    try:
        return subprocess.check_output(
            ["git", "show", f"{commit}:{path}"], cwd=REPO, stderr=subprocess.PIPE,
        )
    except subprocess.CalledProcessError as error:
        raise ContractError(f"cannot read pinned source {commit}:{path}") from error


def _validate_plan(plan: dict[str, Any], state: dict[str, Any], payload: bytes) -> dict[str, Any]:
    _require(plan.get("schema") == "portable_e2e.lr_ab_campaign.v1", "unsupported plan schema")
    _require(plan.get("seeds") == SEEDS and plan.get("arms") == ARMS, "unexpected six-run matrix")
    _require(plan.get("steps") == 1540 and plan.get("batch_size") == 4, "unexpected training budget")
    _require(plan.get("split") == "val", "campaign must not open test")
    _require(plan.get("gpu_uuid") == "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5", "unreviewed GPU UUID")
    commit = plan.get("source_commit", "")
    _require(isinstance(commit, str) and re.fullmatch(r"[0-9a-f]{40}", commit) is not None,
             "source commit must be pinned")
    _require(state.get("source_commit") == commit and state.get("plan") == plan,
             "status does not match frozen plan/source")
    _require(state.get("plan_sha256") == _sha(payload), "frozen plan SHA-256 mismatch")
    _require(state.get("vehicle_control_approved") is False, "campaign cannot approve control")
    limits = plan.get("decision", {}).get("absolute_limits_m")
    _require(limits == {key.removesuffix("_m"): value for key, value in ABSOLUTE_LIMITS.items()},
             "absolute gates differ from predeclared limits")
    config_path = plan.get("model_config")
    _require(config_path == "portable_e2e/config/perspective_trajectory_physical_v1.model.json",
             "campaign model config is not the reviewed physical-v1 config")
    config_payload = _git_bytes(commit, config_path)
    _require(state.get("model_config_sha256") == _sha(config_payload),
             "pinned model-config bytes do not match status")
    return _loads_json(config_payload.decode("utf-8"), config_path)


def _positive_integer(value: Any, context: str, *, zero: bool = False) -> int:
    _require(type(value) is int and value >= (0 if zero else 1), f"invalid count: {context}")
    return value


def _option(command: list[str], name: str) -> str:
    """HH_260906 - Reject missing, repeated or truncated recorded command arguments."""
    _require(command.count(name) == 1, f"recorded command needs exactly one {name}")
    index = command.index(name)
    _require(index + 1 < len(command), f"recorded command has truncated {name}")
    return command[index + 1]


def _geometry(audit: dict[str, Any], count: int) -> dict[str, Any]:
    _require(audit.get("audit_id") == "portable_e2e.runtime_geometry_audit.v8", "audit must use v8")
    _require(audit.get("status") == "RUNTIME_GEOMETRY_AUDIT_COMPLETE", "audit incomplete")
    _require(audit.get("gate") == {
        "source": RUNTIME_GATE_ID, "thresholds": asdict(RuntimeGateConfig()),
        "threshold_overrides": False,
    }, "runtime geometry gate changed")
    geometry = audit.get("geometry", {})
    _require(geometry.get("sample_count") == count, "geometry denominator mismatch")
    selected = geometry.get("selected_result", {})
    passed = _positive_integer(selected.get("geometry_pass_count"), "selected pass", zero=True)
    rejected = _positive_integer(selected.get("geometry_reject_count"), "selected reject", zero=True)
    _require(selected.get("sample_count") == count and passed + rejected == count,
             "selected geometry counts are inconsistent")
    _require(abs(selected.get("geometry_pass_rate", -1) - passed / count) < 1e-12,
             "selected geometry pass rate is inconsistent")
    selector = geometry.get("selector", {})
    histogram = selector.get("selection_counts", {})
    _require(set(histogram) == {str(index) for index in range(6)}, "selector histogram keys invalid")
    observed = sum(_positive_integer(value, "selection count", zero=True) for value in histogram.values())
    _require(observed == count and selector.get("invalid_selection_count") == 0,
             "selector histogram denominator mismatch")
    return {"selected_pass_count": passed, "sample_count": count,
            "selection_histogram": histogram, "selected_failure_counts": selected.get("failure_counts", {})}


def _validate_run(root: Path, relative: str, plan: dict[str, Any], config: dict[str, Any],
                  seed: int, arm: str) -> dict[str, Any]:
    item = root / relative
    training = _read(item / "training/run.json")
    evaluation = _read(item / "evaluation/metrics.json")
    audit = _read(item / "gate_v8.json")
    # HH_260906 - Only the reviewed selector study changes score weight; historical studies remain frozen.
    selector = plan.get("schema") == "portable_e2e.selector_weight_campaign.v1"
    expanded = selector or plan.get("schema") == "portable_e2e.data_expansion_campaign.v1"
    _require(plan.get("schema") in ("portable_e2e.lr_ab_campaign.v1", "portable_e2e.data_expansion_campaign.v1",
                                    "portable_e2e.selector_weight_campaign.v1"),
             f"{relative}: unsupported campaign schema")
    allowed_arms = {"D_selector_weight": 0.0001} if selector else {"C_expanded_data": 0.0001} if expanded else ARMS
    _require(arm in allowed_arms, f"{relative}: unreviewed campaign arm")
    train_samples, train_episode_count = (1147, 3) if expanded else (613, 2)
    _require(training.get("dataset_size") == train_samples,
             f"{relative}: expected full train{train_samples} dataset")
    episode_ids = training.get("training_episode_ids")
    _require(isinstance(episode_ids, list) and all(isinstance(value, str) and value for value in episode_ids)
             and len(episode_ids) == len(set(episode_ids)) == train_episode_count,
             f"{relative}: expected {train_episode_count} unique training episodes")
    _require(evaluation.get("sample_count") == 337 and evaluation.get("domain_sample_counts") == {"carla": 337},
             f"{relative}: expected full CARLA val337 evaluation")
    _require(evaluation.get("training_episode_count") == train_episode_count and
             evaluation.get("evaluation_episode_count") == 1,
             f"{relative}: training/evaluation episode counts differ from reviewed corpus")
    expected_train = {
        "seed": seed, "learning_rate": allowed_arms[arm], "batch_size": 4,
        "max_steps": 1540, "weight_decay": 0.0001, "checkpoint_interval": 154,
        "num_workers": 0, "maximum_gradient_norm": 5.0, "verify_image_sha256": True,
        "sampling_policy": "uniform_without_replacement", "domain_ratios": [],
    }
    _require(training.get("train_config") == expected_train, f"{relative}: train config mismatch")
    _require(training.get("status") == "TRAINING_TARGET_REACHED", f"{relative}: training incomplete")
    _require(training.get("training_split") == "train", f"{relative}: not a train-only run")
    _require(training.get("state", {}).get("global_step") == plan["steps"], f"{relative}: step mismatch")
    _require(training.get("model_config") == config, f"{relative}: model config mismatch")
    _require(training.get("loss_config") == {
        "xy_weight": 1.0, "speed_weight": 0.2, "yaw_weight": 0.1,
        "kinematic_speed_weight": 0.05, "final_displacement_weight": 0.5,
        "candidate_score_weight": 0.5 if selector else 0.1,
    }, f"{relative}: loss config mismatch")
    _require(training.get("model_parameter_count") == evaluation.get("model_parameter_count"),
             f"{relative}: model parameter count mismatch")
    canonical_config_sha = _sha(json.dumps(config, sort_keys=True, separators=(",", ":"), allow_nan=False).encode())
    _require(evaluation.get("model_config_sha256") == canonical_config_sha, f"{relative}: config hash mismatch")
    for name in ("checkpoint_sha256", "model_config_sha256", "corpus_fingerprint_sha256",
                 "dataset_fingerprint_sha256", "training_dataset_fingerprint_sha256",
                 "training_episode_count", "evaluation_episode_count", "training_sampling_policy",
                 "training_sampling_plan_sha256", "training_domain_samples_seen", "domain_sample_counts",
                 "runtime", "device", "batch_size", "model_parameter_count"):
        _require(evaluation.get(name) == audit.get(name), f"{relative}: evaluation/audit {name} mismatch")
    _require(training.get("corpus_fingerprint_sha256") == evaluation.get("corpus_fingerprint_sha256"),
             f"{relative}: training corpus mismatch")
    _require(training.get("dataset_fingerprint_sha256") == evaluation.get("training_dataset_fingerprint_sha256"),
             f"{relative}: training dataset mismatch")
    _require(training.get("sampling_plan_sha256") == evaluation.get("training_sampling_plan_sha256"),
             f"{relative}: sampling plan mismatch")
    _require(training.get("state", {}).get("domain_samples_seen") == evaluation.get("training_domain_samples_seen"),
             f"{relative}: training sample exposure mismatch")
    _require(training.get("runtime") == evaluation.get("runtime"), f"{relative}: training runtime mismatch")
    _require(training.get("hardware") == evaluation.get("hardware"), f"{relative}: training hardware mismatch")
    _require(training.get("device") == evaluation.get("device") == "cuda:0", f"{relative}: device mismatch")
    _require(evaluation.get("hardware", {}).get("device_uuid") == plan["gpu_uuid"].removeprefix("GPU-"),
             f"{relative}: GPU0 UUID mismatch")
    _require(evaluation.get("evaluation_split") == audit.get("evaluation_split") == "val",
             f"{relative}: test must not be opened")
    _require(evaluation.get("vehicle_control_approved") is False and audit.get("vehicle_control_approved") is False,
             f"{relative}: control approval is forbidden")
    for name in ("audit_runtime", "runtime_contract"):
        _require(audit.get("implementation", {}).get(f"{name}_sha256") ==
                 _sha(_git_bytes(plan["source_commit"], f"portable_e2e/{name}.py")),
                 f"{relative}: audit implementation differs from pinned source")
    checkpoint = item / "training/checkpoints/latest.pt"
    checkpoint_present = checkpoint.is_file()
    if checkpoint_present:
        _require(not checkpoint.is_symlink() and _sha(checkpoint.read_bytes()) == evaluation["checkpoint_sha256"],
                 f"{relative}: downloaded checkpoint hash mismatch")
    return {
        "run": relative, "seed": seed, "arm": arm,
        "training_dataset_size": train_samples, "training_episode_count": train_episode_count,
        "validation_sample_count": 337, "validation_episode_count": 1,
        "checkpoint_sha256": evaluation["checkpoint_sha256"],
        "checkpoint_bytes_locally_verified": checkpoint_present,
        "geometry": _geometry(audit, evaluation["sample_count"]),
    }


def summarize_campaign(root: Path) -> dict[str, Any]:
    root = root.resolve(strict=True)
    candidates = [root / name for name in ("plan.json", "frozenplan.json") if (root / name).exists()]
    report: dict[str, Any] = {
        "schema": SCHEMA, "status": "INCOMPLETE", "candidate_screen": "NOT_EVALUATED",
        "automatic_promotion": False, "vehicle_control_approved": False,
        "test_opened_by_this_campaign": None, "pairs": [],
        "limitations": [
            "Validation development only; independent test and closed-loop gates remain outstanding.",
            "Candidate selection counts and ADE regret do not measure geometric trajectory diversity.",
            "Completed artifacts and recorded commands establish this campaign's scope, not activity outside it.",
            "Dataset fingerprints are checked for agreement; the dataset itself is not rehashed by this summarizer.",
        ],
    }
    if not candidates or not (root / "status.json").exists():
        report["missing_artifacts"] = [name for name in ("plan.json", "status.json") if not (root / name).exists()]
        return report
    _require(len(candidates) == 1, "ambiguous frozen plan files")
    plan_path = candidates[0]
    plan, state = _read(plan_path), _read(root / "status.json")
    config = _validate_plan(plan, state, plan_path.read_bytes())
    report.update(campaign_id=plan["campaign_id"], source_commit=plan["source_commit"],
                  plan_sha256=_sha(plan_path.read_bytes()))
    expected = [(f"seed_{seed}/{arm}", seed, arm) for seed in SEEDS for arm in ARMS]
    required = [root / relative / name for relative, _, _ in expected
                for name in ("training/run.json", "evaluation/metrics.json", "gate_v8.json")]
    report["missing_artifacts"] = [str(path.relative_to(root)) for path in required if not path.exists()]
    stages = state.get("stages", [])
    expected_stages = [(relative, stage) for relative, _, _ in expected for stage in ("train", "evaluate", "audit")]
    _require(isinstance(stages, list), "campaign stages must be a list")
    seen_stages = [(stage.get("run"), stage.get("stage")) for stage in stages]
    _require(seen_stages == expected_stages[:len(stages)], "stage order or six-run scope mismatch")
    for record in stages:
        command = record.get("command", [])
        split = "train" if record["stage"] == "train" else "val"
        _require(isinstance(command, list) and all(isinstance(value, str) for value in command),
                 "recorded command must be a string list")
        _require(_option(command, "--split") == split,
                 "recorded command opened a different split")
        _require(_option(command, "--device") == "cuda:0" and _option(command, "--batch-size") == "4",
                 "recorded command device or batch differs from plan")
        module = "audit_runtime" if record["stage"] == "audit" else record["stage"]
        _require(_option(command, "-m") == f"portable_e2e.{module}", "unexpected recorded module")
        if record["stage"] == "train":
            relative_seed, arm = record["run"].split("/")
            _require(_option(command, "--seed") == relative_seed.removeprefix("seed_") and
                     float(_option(command, "--learning-rate")) == ARMS[arm] and
                     _option(command, "--max-steps") == "1540" and "--resume" not in command,
                     "recorded training command differs from frozen seed/LR/budget")
    if (report["missing_artifacts"] or len(stages) != 18 or
            state.get("status") != "TRAIN_EVAL_AUDIT_COMPLETE_NOT_PROMOTED" or
            any(stage.get("status") != "COMPLETE" or stage.get("returncode") != 0 for stage in stages)):
        report["runner_status"] = state.get("status")
        return report
    report["test_opened_by_this_campaign"] = False
    evaluations = [root / relative / "evaluation/metrics.json" for relative, _, _ in expected]
    comparison = compare_reports(evaluations)
    runs = [_validate_run(root, relative, plan, config, seed, arm) for relative, seed, arm in expected]
    for run, row in zip(runs, comparison["reports"]):
        run["metrics"] = row["metrics"]
    for index, seed in enumerate(SEEDS):
        baseline, candidate = runs[2 * index:2 * index + 2]
        a, b = baseline["metrics"], candidate["metrics"]
        checks = {
            "selected_ade_improves": b["selected_ade_m"] < a["selected_ade_m"],
            "selected_fde_improves": b["selected_fde_m"] < a["selected_fde_m"],
            "geometry_does_not_regress": candidate["geometry"]["selected_pass_count"] >= baseline["geometry"]["selected_pass_count"],
            "speed_mae_within_5_percent": b["selected_speed_mae_mps"] <= 1.05 * a["selected_speed_mae_mps"],
        }
        absolute = {name: name in b and b[name] <= limit for name, limit in ABSOLUTE_LIMITS.items()}
        report["pairs"].append({"seed": seed, "baseline": baseline, "candidate": candidate,
                                "relative_checks": checks, "absolute_checks": absolute,
                                "candidate_screen": "PASS" if all(checks.values()) else "FAIL",
                                "absolute_quality": "PASS" if all(absolute.values()) else "FAIL"})
    report.update(
        status="COMPLETE_NOT_PROMOTED",
        candidate_screen="PASS" if all(pair["candidate_screen"] == "PASS" for pair in report["pairs"]) else "FAIL",
        absolute_quality="PASS" if all(pair["absolute_quality"] == "PASS" for pair in report["pairs"]) else "FAIL",
        absolute_limits_m=ABSOLUTE_LIMITS,
        comparison=comparison,
        input_manifest=[{"path": str(path.relative_to(root)), "sha256": _sha(path.read_bytes())}
                        for path in [plan_path, root / "status.json", *required]],
    )
    return report


def render_markdown(report: dict[str, Any]) -> str:
    lines = ["# Portable E2E 학습·검증 3-seed A/B", "", f"상태: `{report['status']}`",
             f"후보 상대 비교: `{report['candidate_screen']}` · 절대 품질: `{report.get('absolute_quality', 'NOT_EVALUATED')}`",
             "", "모델 자동 승격·차량 제어 승인 없음. 이번 캠페인은 validation만 사용하며 독립 test는 열지 않습니다.", ""]
    if report.get("missing_artifacts"):
        lines.extend(["미수집 자료:", "", *[f"- `{name}`" for name in report["missing_artifacts"]], ""])
    if report["pairs"]:
        lines.extend(["| Seed | Arm | ADE m | FDE m | Oracle ADE m | Selection regret m | Speed MAE m/s | Geometry pass | c0–c5 선택수 |",
                      "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |"])
    for pair in report["pairs"]:
        for run in (pair["baseline"], pair["candidate"]):
            metrics, geometry = run["metrics"], run["geometry"]
            values = [f"{metrics[name]:.5f}" for name in ("selected_ade_m", "selected_fde_m", "oracle_ade_m", "selection_regret_ade_m", "selected_speed_mae_mps")]
            histogram = ", ".join(str(geometry["selection_histogram"][str(index)]) for index in range(6))
            lines.append(f"| {pair['seed']} | {run['arm']} | " + " | ".join(values) +
                         f" | {geometry['selected_pass_count']}/{geometry['sample_count']} | {histogram} |")
    for pair in report["pairs"]:
        lines.extend(["", f"Seed {pair['seed']}: 상대 비교 `{pair['candidate_screen']}`, 절대 품질 `{pair['absolute_quality']}`.", ""])
        lines.extend(f"- {name}: {'PASS' if passed else 'FAIL'}" for name, passed in {**pair["relative_checks"], **pair["absolute_checks"]}.items())
    lines.extend(["", "Selection regret는 동일 분모의 selected ADE − oracle ADE입니다. 후보의 기하학적 다양성 판정은 아닙니다.",
                  "절대 기준: ADE 1/3/6.4초 ≤ 0.5/1/2 m, FDE 6.4초 ≤ 4 m.",
                  "Checkpoint가 내려받아지지 않은 경우 평가·감사 간 hash 일치만 확인하며 로컬 byte 검증 여부는 JSON에 기록합니다.", ""])
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("campaign_root", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        report = summarize_campaign(args.campaign_root)
        args.output_dir.mkdir(parents=True, exist_ok=False)
        for name, payload in (("summary.json", json.dumps(report, indent=2, allow_nan=False) + "\n"),
                              ("README.md", render_markdown(report))):
            with (args.output_dir / name).open("x", encoding="utf-8") as stream:
                stream.write(payload)
    except (ContractError, OSError, ValueError, TypeError, KeyError, IndexError) as error:
        print(f"CAMPAIGN_SUMMARY_ERROR: {error}", file=sys.stderr)
        return 2
    print(json.dumps({"status": report["status"], "candidate_screen": report["candidate_screen"], "output": str(args.output_dir)}))
    return 0 if report["status"] == "COMPLETE_NOT_PROMOTED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
