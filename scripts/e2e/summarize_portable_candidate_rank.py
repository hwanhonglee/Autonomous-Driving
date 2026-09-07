#!/usr/bin/env python3
"""HH_260906 - Verify a paired candidate-aware scorer architecture experiment without promotion."""

from __future__ import annotations

import argparse
import ast
import json
from pathlib import Path
import re
import sys

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from scripts.e2e import summarize_portable_data_expansion as expansion
from scripts.e2e import summarize_portable_training_campaign as base
from portable_e2e.contract import ContractError, _loads_json

SCHEMA = "portable_e2e.candidate_rank_summary.v1"
ARM = "E_candidate_rank"
BASELINE_COMMIT = "081a71f7fc014d2864b790b0b0cae7378ae18e4c"
BASELINE_ID = "hh260907-physical-v1-data-expansion-3seeds-v1"
BASELINE_CONFIG = "portable_e2e/config/perspective_trajectory_physical_v1.model.json"
MODEL_CONFIG = "portable_e2e/config/perspective_trajectory_candidate_rank_v1.model.json"
MODEL_ID = "portable_e2e.perspective_trajectory.candidate_rank.v1"
UNCHANGED_FILES = ("portable_e2e/losses.py", "portable_e2e/evaluate.py",
                   "portable_e2e/audit_runtime.py", "portable_e2e/runtime_contract.py")
DECODER_CONSTANTS = ("PHYSICAL_TIME_STEP_S", "PHYSICAL_MAXIMUM_SPEED_MPS",
                     "PHYSICAL_MAXIMUM_ACCELERATION_MPS2", "PHYSICAL_MAXIMUM_ROUTE_SLIP_RAD",
                     "PHYSICAL_MAXIMUM_CURVATURE_RAD_PER_M", "PHYSICAL_MAXIMUM_LATERAL_ACCELERATION_MPS2")


def _decoder_segments(payload: bytes) -> dict[str, bytes]:
    # HH_260906 - AST selects exact decoder source spans while allowing the surrounding scorer architecture to change.
    source = payload.decode("utf-8")
    try:
        tree = ast.parse(source)
    except SyntaxError as error:
        raise ContractError("pinned model source is not valid Python") from error
    classes = [node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "PerspectiveTrajectoryModel"]
    base._require(len(classes) == 1, "pinned model must contain exactly one PerspectiveTrajectoryModel")
    methods = [node for node in classes[0].body if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
               and node.name == "_decode_physical_v1"]
    base._require(len(methods) == 1 and not methods[0].decorator_list, "physical decoder method missing, repeated or decorated")
    result = {"PerspectiveTrajectoryModel._decode_physical_v1": ast.get_source_segment(source, methods[0]).encode("utf-8")}
    for name in DECODER_CONSTANTS:
        nodes = [node for node in tree.body if isinstance(node, ast.Assign)
                 and any(isinstance(target, ast.Name) and target.id == name for target in node.targets)]
        base._require(len(nodes) == 1, f"physical decoder constant missing or repeated: {name}")
        result[name] = ast.get_source_segment(source, nodes[0]).encode("utf-8")
    return result


def _validate_plan(plan: dict, state: dict, payload: bytes, baseline_root: Path) -> dict:
    # HH_260906 - Keep all reviewed data, optimizer, seed and gate fields frozen across the architecture change.
    fixed = {
        "schema": "portable_e2e.candidate_rank_campaign.v1", "campaign_id": "hh260907-candidate-rank-3seeds-v1",
        "gpu_uuid": "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5",
        "dataset": "datasets/prepared/carla-common10-30kph-five-episodes-20260907-v3",
        "dataset_manifest_sha256": expansion.MANIFEST_SHA256, "model_config": MODEL_CONFIG,
        "seeds": base.SEEDS, "arms": {ARM: 0.0001}, "candidate_score_weight": 0.1,
        "steps": 1540, "batch_size": 4, "split": "val", "expected_train_samples": 1147,
        "expected_val_samples": 337, "prerequisite_campaign_id": BASELINE_ID,
        "prerequisite_timeout_seconds": 3600, "baseline_campaign_id": BASELINE_ID,
        "baseline_source_commit": BASELINE_COMMIT,
    }
    base._require(all(plan.get(key) == value for key, value in fixed.items()), "candidate-rank plan differs from reviewed scope")
    commit = plan.get("source_commit", "")
    base._require(isinstance(commit, str) and re.fullmatch(r"[0-9a-f]{40}", commit) is not None,
                  "source commit must be pinned")
    base._require(state.get("source_commit") == commit and state.get("plan") == plan, "status plan/source mismatch")
    base._require(state.get("plan_sha256") == base._sha(payload), "frozen plan SHA-256 mismatch")
    base._require(plan.get("decision", {}).get("absolute_limits_m") ==
                  {key.removesuffix("_m"): value for key, value in base.ABSOLUTE_LIMITS.items()},
                  "absolute gates differ from predeclared limits")
    base._require(state.get("dataset_manifest_sha256") == expansion.MANIFEST_SHA256, "dataset manifest proof mismatch")
    base._require(state.get("reviewed_contract") == {"train_samples": 1147, "val_samples": 337, "run_count": 3, "stage_count": 9},
                  "reviewed sample/run/stage contract mismatch")
    base._require(state.get("vehicle_control_approved") is False, "campaign cannot approve control")
    runner_sha = state.get("runner_sha256", "")
    base._require(isinstance(runner_sha, str) and re.fullmatch(r"[0-9a-f]{64}", runner_sha) is not None,
                  "missing runner hash declaration")
    prerequisite = state.get("prerequisite", {})
    base._require(prerequisite.get("status") == "TRAIN_EVAL_AUDIT_COMPLETE_NOT_PROMOTED" and
                  prerequisite.get("completed_stages") == 9, "predecessor completion proof missing")
    base._require(prerequisite.get("sha256") == base._sha((baseline_root / "status.json").read_bytes()),
                  "predecessor status byte hash mismatch")
    config_bytes = base._git_bytes(commit, MODEL_CONFIG)
    base._require(state.get("model_config_sha256") == base._sha(config_bytes), "model-config byte proof mismatch")
    config = _loads_json(config_bytes.decode("utf-8"), "pinned candidate model config")
    baseline_config = _loads_json(base._git_bytes(BASELINE_COMMIT, BASELINE_CONFIG).decode("utf-8"), "pinned baseline config")
    base._require(config == {**baseline_config, "model_id": MODEL_ID}, "candidate configuration changes more than the reviewed model ID")
    return config


def _source_proof(baseline_root: Path, candidate_root: Path, plan: dict, state: dict) -> dict:
    # HH_260906 - A different model implementation is intentional; loss, evaluation, gate and physical decoder are not.
    files = []
    for name in UNCHANGED_FILES:
        before, after = [base._git_bytes(commit, name) for commit in (BASELINE_COMMIT, plan["source_commit"])]
        base._require(before == after, f"frozen implementation changed between C and E: {name}")
        files.append({"path": name, "baseline_sha256": base._sha(before), "candidate_sha256": base._sha(after)})
    model_bytes = [base._git_bytes(commit, "portable_e2e/model.py") for commit in (BASELINE_COMMIT, plan["source_commit"])]
    before, after = [_decoder_segments(value) for value in model_bytes]
    decoder = []
    for name in before:
        base._require(before[name] == after[name], f"physical decoder source changed between C and E: {name}")
        decoder.append({"symbol": name, "baseline_sha256": base._sha(before[name]), "candidate_sha256": base._sha(after[name])})
    workers = []
    for label, root, worker in (("baseline", baseline_root, base._read(baseline_root / "status.json")),
                                ("candidate", candidate_root, state)):
        path = root / "provenance/active_runner.py"
        present = path.is_file()
        if present:
            base._require(not path.is_symlink() and base._sha(path.read_bytes()) == worker["runner_sha256"],
                          f"{label} archived worker SHA-256 mismatch")
        workers.append({"arm": label, "path": "provenance/active_runner.py", "sha256": worker["runner_sha256"],
                        "bytes_locally_verified": present})
    return {"status": "FROZEN_LOSS_EVALUATION_GATE_AND_DECODER_IDENTICAL", "baseline_commit": BASELINE_COMMIT,
            "candidate_commit": plan["source_commit"], "unchanged_files": files, "decoder_segments": decoder,
            "model_file_sha256": {"baseline": base._sha(model_bytes[0]), "candidate": base._sha(model_bytes[1])},
            "workers": workers, "scope": "Exact declared method/constants and whole loss/evaluator/audit/gate files; not whole-model equality."}


def summarize_campaign(baseline_root: Path, candidate_root: Path) -> dict:
    baseline_root, candidate_root = baseline_root.resolve(strict=True), candidate_root.resolve(strict=True)
    base._require(baseline_root != candidate_root, "baseline and candidate must be separate campaigns")
    report = {"schema": SCHEMA, "status": "INCOMPLETE", "candidate_screen": "NOT_EVALUATED",
              "absolute_quality": "NOT_EVALUATED", "automatic_promotion": False, "vehicle_control_approved": False,
              "test_opened_by_this_campaign": None, "pairs": [],
              "experiment": {"kind": "PAIRED_DIFFERENT_ARCHITECTURE_SAME_CORPUS", "same_model_architecture": False,
                  "changed_variable": "Candidate-aware scorer architecture/model ID; parameter counts may differ.",
                  "comparison_policy": "Unmodified compare_reports validates all six reports under its shared-data/runtime/denominator contract."},
              "limitations": ["Validation development only; independent test, shadow, scenario coverage and closed-loop gates remain outstanding.",
                  "Same seeds and step budget do not mean identical initial weights, gradient trajectories or training compute across architectures.",
                  "Source cleanliness is a worker proof; missing local worker/checkpoint bytes are explicitly marked unverified.",
                  "Corpus metadata is cross-checked, not rehashed here; this summarizer never opens held-out test samples."]}
    missing = [f"{label}/{name}" for label, root in (("baseline", baseline_root), ("candidate", candidate_root))
               for name in ("plan.json", "status.json") if not (root / name).exists()]
    if missing:
        report["missing_artifacts"] = missing
        return report
    baseline = expansion.summarize_campaign(baseline_root)
    if baseline["status"] != "COMPLETE_NOT_PROMOTED":
        report["missing_artifacts"] = ["baseline/" + name for name in baseline.get("missing_artifacts", [])]
        report["baseline_status"] = baseline["status"]
        return report
    plan, state = base._read(candidate_root / "plan.json"), base._read(candidate_root / "status.json")
    config = _validate_plan(plan, state, (candidate_root / "plan.json").read_bytes(), baseline_root)
    report.update(campaign_id=plan["campaign_id"], source_commit=plan["source_commit"], baseline_campaign_id=BASELINE_ID,
                  dataset_manifest_sha256=expansion.MANIFEST_SHA256, corpus_fingerprint_sha256=expansion.CORPUS_SHA256,
                  candidate_score_weights={expansion.ARM: 0.1, ARM: 0.1},
                  source_proof=_source_proof(baseline_root, candidate_root, plan, state))
    expected = [(f"seed_{seed}/{ARM}", stage) for seed in base.SEEDS for stage in expansion.STAGE_FILES]
    stages = state.get("stages", [])
    base._require(isinstance(stages, list) and all(isinstance(record, dict) for record in stages), "invalid stage list")
    base._require([(record.get("run"), record.get("stage")) for record in stages] == expected[:len(stages)],
                  "nine-stage order/scope mismatch")
    required = [candidate_root / relative / expansion.STAGE_FILES[stage] for relative, stage in expected]
    report["missing_artifacts"] = ["candidate/" + str(path.relative_to(candidate_root)) for path in required if not path.exists()]
    for record in stages:
        expansion._validate_stage(record, candidate_root, state, complete=record.get("status") == "COMPLETE")
        if record["stage"] == "train":
            base._require(float(base._option(record["command"], "--candidate-score-weight")) == 0.1,
                          "recorded candidate-score weight differs from reviewed 0.1")
    if (report["missing_artifacts"] or len(stages) != 9 or state.get("status") != "TRAIN_EVAL_AUDIT_COMPLETE_NOT_PROMOTED"
            or any(record.get("status") != "COMPLETE" or record.get("returncode") != 0 for record in stages)):
        report["runner_status"] = state.get("status")
        return report
    evaluations = [root / f"seed_{seed}/{arm}/evaluation/metrics.json" for seed in base.SEEDS
                   for root, arm in ((baseline_root, expansion.ARM), (candidate_root, ARM))]
    comparison = base.compare_reports(evaluations)
    for index, seed in enumerate(base.SEEDS):
        before = dict(baseline["runs"][index])
        after = base._validate_run(candidate_root, f"seed_{seed}/{ARM}", plan, config, seed, ARM)
        rows = comparison["reports"][2 * index:2 * index + 2]
        for run, row in zip((before, after), rows):
            run.update(metrics=row["metrics"], model_config_sha256=row["model_config_sha256"],
                       model_parameter_count=row["model_parameter_count"])
        a, b = before["metrics"], after["metrics"]
        checks = {"selected_ade_improves": b["selected_ade_m"] < a["selected_ade_m"],
                  "selected_fde_improves": b["selected_fde_m"] < a["selected_fde_m"],
                  "geometry_does_not_regress": after["geometry"]["selected_pass_count"] >= before["geometry"]["selected_pass_count"],
                  "speed_mae_within_5_percent": b["selected_speed_mae_mps"] <= 1.05 * a["selected_speed_mae_mps"]}
        absolute = {name: name in b and b[name] <= limit for name, limit in base.ABSOLUTE_LIMITS.items()}
        report["pairs"].append({"seed": seed, "baseline": before, "candidate": after,
            "relative_checks": checks, "absolute_checks": absolute,
            "candidate_screen": "PASS" if all(checks.values()) else "FAIL",
            "absolute_quality": "PASS" if all(absolute.values()) else "FAIL"})
    manifest = [{"path": "baseline/" + item["path"], "sha256": item["sha256"]} for item in baseline["input_manifest"]]
    manifest.extend({"path": "candidate/" + str(path.relative_to(candidate_root)), "sha256": base._sha(path.read_bytes())}
                    for path in [candidate_root / "plan.json", candidate_root / "status.json", *required])
    report.update(status="COMPLETE_NOT_PROMOTED", test_opened_by_this_campaign=False,
                  candidate_screen="PASS" if all(pair["candidate_screen"] == "PASS" for pair in report["pairs"]) else "FAIL",
                  absolute_quality="PASS" if all(pair["absolute_quality"] == "PASS" for pair in report["pairs"]) else "FAIL",
                  absolute_limits_m=base.ABSOLUTE_LIMITS, comparison=comparison, input_manifest=manifest)
    return report


def render_markdown(report: dict) -> str:
    content = base.render_markdown(report).replace("학습·검증 3-seed A/B", "후보 경로 인지 점수 모델 · 동일 v3 · 3-seed C/E")
    return content + "\n서로 다른 모델 구조의 비교입니다. 모델 ID·parameter 수가 다르며, 같은 seed·step이 같은 초기 가중치나 학습 연산량을 의미하지 않습니다.\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline_root", type=Path)
    parser.add_argument("candidate_root", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        report = summarize_campaign(args.baseline_root, args.candidate_root)
        args.output_dir.mkdir(parents=True, exist_ok=False)
        for name, payload in (("summary.json", json.dumps(report, indent=2, allow_nan=False) + "\n"),
                              ("README.md", render_markdown(report))):
            with (args.output_dir / name).open("x", encoding="utf-8") as stream:
                stream.write(payload)
    except (ContractError, OSError, ValueError, TypeError, KeyError, IndexError) as error:
        print(f"CANDIDATE_RANK_SUMMARY_ERROR: {error}", file=sys.stderr)
        return 2
    print(json.dumps({"status": report["status"], "candidate_screen": report["candidate_screen"], "output": str(args.output_dir)}))
    return 0 if report["status"] == "COMPLETE_NOT_PROMOTED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
