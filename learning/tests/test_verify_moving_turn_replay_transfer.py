# HH_260906 - Locate the repository independently of the relocated test directory.
"""HH_260906 - Exercise failed-run transfer plus sealed subgroup recovery using CPU-only fixtures."""
from __future__ import annotations

import ast
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import pytest

from scripts.e2e import verify_moving_turn_replay_transfer as verifier

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path


ROOT = next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())
SCRIPT = _hh_source_path(ROOT, "scripts/e2e/verify_moving_turn_replay_transfer.py")


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def put(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if isinstance(value, bytes):
        path.write_bytes(value)
    else:
        path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n")


def canonical(value) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                 allow_nan=False).encode()).hexdigest()


def seal_inventory(artifact: Path) -> str:
    results = artifact / "remote_results"
    payload = "".join(
        f"{sha(path)}  {path.relative_to(results).as_posix()}\n"
        for path in sorted(item for item in results.rglob("*") if item.is_file())
    )
    (artifact / "remote_inventory.sha256").write_text(payload)
    (artifact / "local_inventory.sha256").write_text(payload)
    return sha(artifact / "remote_inventory.sha256")


def seal_recovery_inventory(artifact: Path) -> str:
    # HH_260906 - Model the separately captured recovery inventory and its byte-identical local copy.
    results = artifact / verifier.RECOVERY_RESULTS
    payload = "".join(
        f"{sha(path)}  {path.relative_to(results).as_posix()}\n"
        for path in sorted(item for item in results.rglob("*") if item.is_file())
    )
    (artifact / "recovery_remote_inventory.sha256").write_text(payload)
    (artifact / "recovery_local_inventory.sha256").write_text(payload)
    return sha(artifact / "recovery_remote_inventory.sha256")


def tree_rows(root: Path) -> list[dict]:
    rows = [{"path": path.relative_to(root).as_posix(), "size": path.stat().st_size,
             "sha256": sha(path)} for path in root.rglob("*") if path.is_file()]
    return sorted(rows, key=lambda row: row["path"])


def make_artifact(tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
                  *, gate_pass: bool = True) -> tuple[Path, str, str, str]:
    artifact = tmp_path / "artifact"
    results = artifact / "remote_results"
    results.mkdir(parents=True)
    supplemental = artifact / verifier.SUPPLEMENTAL_C_CHECKPOINT
    put(supplemental, b"synthetic supplemental C checkpoint")
    monkeypatch.setattr(verifier, "REFERENCE_C_CHECKPOINT_SHA256", sha(supplemental))
    executed_wrapper = artifact / verifier.EXECUTED_RECOVERY_WRAPPER
    put(executed_wrapper, b"synthetic remotely executed recovery wrapper")
    monkeypatch.setattr(verifier, "EXECUTED_RECOVERY_WRAPPER_SHA256", sha(executed_wrapper))
    model_config = {"model_id": "synthetic.turn-replay", "candidate_count": 12}
    current: dict[str, str] = {}
    base: dict[str, str] = {}
    for name in verifier.EXPECTED_CURRENT_SOURCE_PATHS:
        payload = ((json.dumps(model_config, sort_keys=True) + "\n").encode()
                   if name.endswith(".model.json") else ("current:" + name).encode())
        put(results / "source" / name, payload)
        current[name] = sha(results / "source" / name)
        base[name] = ("b" * 64 if name in verifier.EXPECTED_OVERLAY_PATHS else current[name])
    added = {}
    for name in verifier.EXPECTED_ADDED_SOURCE_PATHS:
        put(results / "source" / name, ("added:" + name).encode())
        added[name] = sha(results / "source" / name)
    put(results / verifier.RUNNER_ARCHIVE, b"synthetic archived runner")
    put(results / verifier.EVALUATOR_ARCHIVE, b"synthetic archived evaluator")
    put(results / verifier.OWNER_ARCHIVE, b"synthetic archived owner")
    put(results / "view_report.json", {"synthetic": "view"})
    reference_paths = {"plan": "plan.json", "summary": "summary.json",
        "training_run": "A_baseline_train/training/run.json",
        "subgroup": "A_baseline_train/subgroup/summary.json", "gate": "A_baseline_train/gate_v8.json"}
    reference_files = {}
    for label, name in reference_paths.items():
        put(results / "reference_C" / name, {"synthetic_reference": label})
        reference_files[label] = {"path": name, "sha256": sha(results / "reference_C" / name)}
    reference_files["checkpoint"] = {"path": "A_baseline_train/training/checkpoints/latest.pt",
                                     "sha256": verifier.REFERENCE_C_CHECKPOINT_SHA256}
    run_denials = {field: False for field in verifier.RUN_SCOPE_DENIALS}
    loss_config = {"xy_weight": 1.0, "candidate_regret_weight": 0.1}
    plan = {"schema": verifier.PLAN_SCHEMA, "campaign_id": verifier.CAMPAIGN_ID,
        "source": {"base_commit": verifier.BASE_COMMIT, "base_sha256": base,
            "current_sha256": current, "overlay_paths": list(verifier.EXPECTED_OVERLAY_PATHS),
            "added_paths": list(verifier.EXPECTED_ADDED_SOURCE_PATHS), "added_sha256": added},
        "runner_sha256": sha(results / verifier.RUNNER_ARCHIVE),
        "evaluator_sha256": sha(results / verifier.EVALUATOR_ARCHIVE),
        "owner": {"path": "fixture", "sha256": sha(results / verifier.OWNER_ARCHIVE)},
        "model_config": "portable_e2e/config/perspective_trajectory_physical_stopmix_v1.model.json",
        "model_config_sha256": sha(results / "source/portable_e2e/config/perspective_trajectory_physical_stopmix_v1.model.json"),
        "model_config_canonical_sha256": canonical(model_config),
        "view_report_sha256": sha(results / "view_report.json"), "stage_order": list(verifier.STAGES),
        "arm": {"label": verifier.ARM, "execution_schedule": {"row_count": 36040, "sha256": "e" * 64}},
        "loss_config": loss_config, "decision": {"adoption_allowed": False},
        "reference_C": {"files": reference_files}, **run_denials}
    put(results / "plan.json", plan)
    plan_sha = sha(results / "plan.json")
    item = results / verifier.ARM
    put(item / ".training.portable-e2e.lock", b"")
    put(item / "training/checkpoints/latest.pt", b"opaque E checkpoint bytes")
    checkpoint_sha = sha(item / "training/checkpoints/latest.pt")
    monkeypatch.setattr(verifier, "EXPECTED_E_CHECKPOINT_SHA256", checkpoint_sha)
    history = item / "training/metrics.jsonl"
    history.parent.mkdir(parents=True, exist_ok=True)
    with history.open("w") as stream:
        for step in range(1, 9021):
            row = {"global_step": step}
            if step == 9020:
                row.update(samples_seen=36040, domain_samples_seen={"carla": 36040},
                           sampling_stream_samples_seen={"base": 29500, "replay": 6540})
            stream.write(json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n")
    put(item / "training/run.json", {"status": "TRAINING_TARGET_REACHED",
        "warning": "RESEARCH CHECKPOINT — NOT APPROVED FOR VEHICLE CONTROL",
        "model_config": model_config})
    put(item / "evaluation/metrics.json", {"status": "OPEN_LOOP_EVALUATION_COMPLETE",
        "checkpoint_sha256": checkpoint_sha, "vehicle_control_approved": False,
        "loss_config": loss_config, "auxiliary_loss_metric_counts": {"candidate_regret_loss": 665}})
    put(item / "gate_v8.json", {"status": "RUNTIME_GEOMETRY_AUDIT_COMPLETE",
        "checkpoint_sha256": checkpoint_sha, "vehicle_control_approved": False})
    for index in range(12):
        put(item / "evaluation/trajectories" / f"{index:04d}_synthetic.png",
            b"\x89PNG\r\n\x1a\n" + str(index).encode())
    completed_rows = []
    for index, stage in enumerate(verifier.STAGES):
        command = ["/venv/bin/python", "-m", f"portable_e2e.{stage}"]
        if stage == "subgroup":
            command = ["/venv/bin/python", "/bench/evaluate_portable_moving_turn_replay_subgroups.py",
                       "--runner", "/bench/run_moving_turn_replay_seed_pilot.py",
                       "--runner-sha256", plan["runner_sha256"], "--plan-sha256", plan_sha,
                       "--checkpoint-sha256", checkpoint_sha]
        started = {"arm": verifier.ARM, "stage": stage, "command": command,
            "timeout_seconds": 100, "started_at_utc": f"2026-09-21T00:0{index}:00+00:00",
            "status": "RUNNING", "owner_pid": 1234, "plan_sha256": plan_sha,
            "input_checkpoint_sha256": None if stage == "train" else checkpoint_sha,
            "pre_cuda_gpu": {"physical_index": 0, "pre_cuda_compute_processes": 0, "uuid": "GPU-fixture"}}
        put(item / f"{stage}_started.json", started)
        if stage == "subgroup":
            put(item / "subgroup.log",
                b"MOVING_TURN_REPLAY_SUBGROUP_ERROR: checkpoint replay contract/accounting differs\n")
            continue
        report = item / verifier.STAGE_REPORTS[stage]
        completed = dict(started)
        completed.update(status="COMPLETE", ended_at_utc="2026-09-21T00:10:00+00:00",
                         path=f"{verifier.ARM}/{verifier.STAGE_REPORTS[stage]}",
                         sha256=sha(report), checkpoint_sha256=checkpoint_sha)
        if stage == "train":
            completed.update(history_sha256=sha(history), sampling_execution_schedule_row_count=36040,
                             sampling_execution_schedule_sha256="e" * 64)
        put(item / f"{stage}_completed.json", completed)
        put(item / f"{stage}.log", ("log:" + stage).encode())
        completed_rows.append(completed)
    started_run = {"schema": verifier.SUMMARY_SCHEMA, "status": "RUNNING",
        "started_at_utc": "2026-09-21T00:00:00+00:00", "plan_sha256": plan_sha,
        "arm": "NOT_RUN", "stages": [], **run_denials}
    put(results / "started.json", started_run)
    put(results / "failed.json", {**started_run, "status": "FAIL", "arm": "FAIL",
        "stages": completed_rows, "error_type": "ValueError", "error": f"{verifier.ARM} subgroup exited 2",
        "ended_at_utc": "2026-09-21T00:11:00+00:00"})
    remote_inventory_sha = seal_inventory(artifact)
    recovery = artifact / verifier.RECOVERY_RESULTS
    put(recovery / "source/original_runner.py", (results / verifier.RUNNER_ARCHIVE).read_bytes())
    put(recovery / "source/patched_evaluator.py", b"synthetic patched evaluator")
    execution_plan = deepcopy(plan)
    execution_plan["evaluator_sha256"] = sha(recovery / "source/patched_evaluator.py")
    put(recovery / "execution_plan.json", execution_plan)
    recovery_denials = {field: False for field in verifier.RECOVERY_DENIALS}
    original_rows = tree_rows(results)
    required_paths = {"plan": "plan.json", "failed": "failed.json",
        "subgroup_log": f"{verifier.ARM}/subgroup.log",
        "checkpoint": f"{verifier.ARM}/training/checkpoints/latest.pt",
        "training_run": f"{verifier.ARM}/training/run.json",
        "evaluation": f"{verifier.ARM}/evaluation/metrics.json", "audit": f"{verifier.ARM}/gate_v8.json",
        "runner": verifier.RUNNER_ARCHIVE, "old_evaluator": verifier.EVALUATOR_ARCHIVE}
    required = {label: {"path": name, "size": (results / name).stat().st_size,
                        "sha256": sha(results / name)} for label, name in required_paths.items()}
    recovery_plan = {"schema": verifier.RECOVERY_PLAN_SCHEMA, "recovery_id": verifier.RECOVERY_ID,
        "output": "runs/diagnostics/moving_turn_replay_seed20260906_20260921_v1_subgroup_recovery_v1",
        "stage_order": ["subgroup"], "subgroup_evaluation_executed": True,
        "original_run": {"root": "runs/diagnostics/moving_turn_replay_seed20260906_20260921_v1",
            "tree": {"file_count": 60, "total_size_bytes": sum(row["size"] for row in original_rows),
                     "files": original_rows,
                     "inventory_sha256": hashlib.sha256(verifier._encoded(original_rows)).hexdigest()},
            "required": required, "known_failure": {"error_type": "ValueError",
                "error": f"{verifier.ARM} subgroup exited 2",
                "subgroup_log_exact":
                    "MOVING_TURN_REPLAY_SUBGROUP_ERROR: checkpoint replay contract/accounting differs\n"}},
        "execution_plan": {"path": "execution_plan.json", "sha256": sha(recovery / "execution_plan.json"),
                           "original_plan_sha256": plan_sha, "changed_fields": ["evaluator_sha256"]},
        "runner": {"path": verifier.RUNNER_ARCHIVE, "sha256": plan["runner_sha256"]},
        "patched_evaluator": {"name": "evaluate_portable_moving_turn_replay_subgroups.py",
                              "sha256": execution_plan["evaluator_sha256"]},
        "allowed_fix": {"scope": "subgroup_evaluator_train_config_representation_only",
            "old_evaluator_sha256": plan["evaluator_sha256"],
            "patched_evaluator_sha256": execution_plan["evaluator_sha256"],
            "changed_execution_plan_fields": ["evaluator_sha256"]}, **recovery_denials}
    put(recovery / "recovery_plan.json", recovery_plan)
    recovery_plan_sha, execution_plan_sha = sha(recovery / "recovery_plan.json"), sha(recovery / "execution_plan.json")
    subgroup_denials = {"test_read_or_use": False, "closed_loop_evaluation": False,
        "automatic_promotion": False, "model_exported": False,
        "training_executed_by_evaluator": False, "vehicle_control_approved": False}
    episode_rows = {}
    for label, count in (("town03_right_turn_dev", 337), ("town03_straight_dev", 328)):
        report = recovery / "subgroup/episodes" / label / "metrics.json"
        put(report, {"label": label})
        for index in range(6):
            put(report.parent / "trajectories" / f"{index:04d}_synthetic.png",
                b"\x89PNG\r\n\x1a\n" + label.encode() + str(index).encode())
        episode_rows[label] = {"count": count, "report_path": f"episodes/{label}/metrics.json",
                               "report_sha256": sha(report)}
    put(recovery / "subgroup/summary.json", {"status": "COMPLETE_NOT_PROMOTED",
        "arm": verifier.ARM, "plan_sha256": execution_plan_sha,
        "runner_sha256": plan["runner_sha256"], "evaluator_sha256": execution_plan["evaluator_sha256"],
        "checkpoint_sha256": checkpoint_sha, "combined_val_count": 665,
        "episodes": episode_rows, **subgroup_denials})
    put(recovery / "started.json", {"schema": verifier.RECOVERY_SUMMARY_SCHEMA, "status": "RUNNING",
        "recovery_plan_sha256": recovery_plan_sha, "execution_plan_sha256": execution_plan_sha,
        "original_plan_sha256": plan_sha, "checkpoint_sha256": checkpoint_sha,
        "stage": "subgroup", **recovery_denials})
    command = ["/venv/bin/python", "/bench/evaluate_portable_moving_turn_replay_subgroups.py",
               "--runner", "/original/source/run_moving_turn_replay_seed_pilot.py",
               "--runner-sha256", plan["runner_sha256"], "--plan-sha256", execution_plan_sha,
               "--checkpoint-sha256", checkpoint_sha]
    recovery_stage = {"status": "RUNNING", "stage": "subgroup", "command": command,
        "started_at_utc": "2026-09-21T01:00:00+00:00", "timeout_seconds": 1200,
        "input_checkpoint_sha256": checkpoint_sha, **recovery_denials}
    put(recovery / "subgroup_started.json", recovery_stage)
    put(recovery / "subgroup.log", b"recovery subgroup complete\n")
    completed = dict(recovery_stage)
    completed.update(status="COMPLETE", ended_at_utc="2026-09-21T01:01:00+00:00",
                     path="subgroup/summary.json", sha256=sha(recovery / "subgroup/summary.json"),
                     checkpoint_sha256=checkpoint_sha)
    put(recovery / "completed.json", completed)
    subgroup_proof = {"path": "subgroup/summary.json", "sha256": sha(recovery / "subgroup/summary.json"),
                      "checkpoint_sha256": checkpoint_sha}
    put(recovery / "recovery_summary.json", {"schema": verifier.RECOVERY_SUMMARY_SCHEMA,
        "status": "COMPLETE_NOT_PROMOTED", "recovery_plan_sha256": recovery_plan_sha,
        "execution_plan_sha256": execution_plan_sha, "original_plan_sha256": plan_sha,
        "original_failed_run_unchanged": True, "stage_order": ["subgroup"],
        "subgroup": subgroup_proof, "subgroup_evaluation_executed": True, **recovery_denials})
    recovery_rows = tree_rows(recovery)
    recovery_proof = {"plan_sha256": recovery_plan_sha, "execution_plan_sha256": execution_plan_sha,
        "summary_sha256": sha(recovery / "recovery_summary.json"),
        "subgroup_sha256": sha(recovery / "subgroup/summary.json"),
        "tree_sha256": hashlib.sha256(verifier._encoded(recovery_rows)).hexdigest()}
    recovery_inventory_sha = seal_recovery_inventory(artifact)
    recovery_proof["inventory_sha256"] = recovery_inventory_sha
    false_scope = {field: False for field in verifier.COMPARISON_SCOPE_DENIALS}
    reference_hashes = {label: row["sha256"] for label, row in reference_files.items()}
    check_values = {
        "full_tree": {"original_file_count": 60,
            "original_total_size_bytes": sum(row["size"] for row in original_rows),
            "original_inventory_sha256": remote_inventory_sha,
            "recovery_file_count": 24,
            "recovery_total_size_bytes": sum(row["size"] for row in recovery_rows),
            "recovery_inventory_sha256": recovery_inventory_sha,
            "recovery_tree_sha256": recovery_proof["tree_sha256"]},
        "source_archive": {"source_file_count": 22, "runner_sha256": plan["runner_sha256"],
            "original_evaluator_sha256": plan["evaluator_sha256"],
            "patched_evaluator_sha256": execution_plan["evaluator_sha256"]},
        "reference_C": reference_hashes,
        "model_config": {"raw_sha256": plan["model_config_sha256"],
            "canonical_sha256": plan["model_config_canonical_sha256"],
            "training_run_matches_source": True},
        "evaluation_loss_config": loss_config,
        "candidate_regret_count": {"candidate_regret_loss": 665},
        "reference_C_checkpoint_supplemental": verifier.REFERENCE_C_CHECKPOINT_SHA256,
        "recovery_inventory": recovery_inventory_sha,
        "executed_recovery_wrapper": verifier.EXECUTED_RECOVERY_WRAPPER_SHA256,
        "original_failure_preservation": {"status": "FAIL",
            "failed_sha256": sha(results / "failed.json"), "completed_stages": 3},
        "recovery_subgroup": {"status": "COMPLETE_NOT_PROMOTED",
            "sha256": recovery_proof["subgroup_sha256"], "checkpoint_sha256": checkpoint_sha,
            "combined_val_count": 665},
    }
    checks = {name: {"expected": value, "observed": deepcopy(value), "pass": True}
              for name, value in check_values.items()}
    put(artifact / verifier.POSTRUN_VERIFICATION, {"schema": verifier.POSTRUN_SCHEMA,
        "status": "VERIFIED_PARTIAL_FAILURE_WITH_POSTHOC_RECOVERY",
        "remote_inventory_sha256": remote_inventory_sha, "plan_sha256": plan_sha,
        "failed_sha256": sha(results / "failed.json"), "runner_sha256": plan["runner_sha256"],
        "recovery_plan_sha256": recovery_proof["plan_sha256"],
        "recovery_execution_plan_sha256": recovery_proof["execution_plan_sha256"],
        "recovery_summary_sha256": recovery_proof["summary_sha256"],
        "recovery_subgroup_sha256": recovery_proof["subgroup_sha256"],
        "recovery_inventory_sha256": recovery_inventory_sha,
        "recovery_tree_sha256": recovery_proof["tree_sha256"],
        "executed_recovery_wrapper_sha256": verifier.EXECUTED_RECOVERY_WRAPPER_SHA256,
        "reference_C_checkpoint_supplemental_sha256": verifier.REFERENCE_C_CHECKPOINT_SHA256,
        "original_run_failure_preserved": True, "original_completed_stages": 3,
        "recovery_subgroup_pass": True, "full_tree_verification_pass": True,
        "source_verification_pass": True, "reference_C_verification_pass": True,
        "model_config_verification_pass": True, "evaluation_loss_config_verification_pass": True,
        "candidate_regret_count_verification_pass": True, "checks": checks, "scope": false_scope})
    gate_status = "PASS_SINGLE_SEED_DIAGNOSTIC" if gate_pass else "FAIL_SINGLE_SEED_DIAGNOSTIC"
    comparison = {"schema": verifier.COMPARISON_SCHEMA, "status": "COMPLETE_NOT_PROMOTED",
        "inputs": {"E_experiment": {"campaign_id": verifier.CAMPAIGN_ID, "plan_sha256": plan_sha,
            "failed_sha256": sha(results / "failed.json"), "original_status": "FAIL_SUBGROUP",
            "posthoc_subgroup_recovery": {key: recovery_proof[key] for key in
                ("plan_sha256", "execution_plan_sha256", "summary_sha256", "subgroup_sha256")}}},
        "arm_provenance": {"E": {"checkpoint_sha256": checkpoint_sha, "history_sha256": sha(history),
            "training_run_sha256": sha(item / "training/run.json"),
            "evaluation_sha256": sha(item / "evaluation/metrics.json"),
            "subgroup_sha256": recovery_proof["subgroup_sha256"], "subgroup_source": "posthoc_recovery",
            "geometry_gate_sha256": sha(item / "gate_v8.json")}},
        "scope": {**false_scope, "single_seed_only": True, "three_seed_confirmation_required": True},
        "preregistered_C_E_gate": {"runner_summary_exact_match": False,
            "posthoc_recovery_subgroup_used": True,
            "result": {"status": gate_status, "pass": gate_pass,
                       "adoption_allowed": False, "automatic_promotion": False}}}
    comparison_path = artifact / "derived/comparison_C_E.json"
    put(comparison_path, comparison)
    return artifact, remote_inventory_sha, recovery_inventory_sha, sha(comparison_path)


@pytest.mark.parametrize("gate_pass", [True, False])
def test_failed_transfer_and_recovery_are_verified_without_recomputing_gate(tmp_path, monkeypatch, gate_pass):
    artifact, inventory_sha, recovery_inventory_sha, comparison_sha = make_artifact(
        tmp_path, monkeypatch, gate_pass=gate_pass)
    result = verifier.verify(artifact_root=artifact, remote_inventory_sha256=inventory_sha,
                             recovery_inventory_sha256=recovery_inventory_sha,
                             comparison_sha256=comparison_sha)
    assert result["status"] == "PASS_EXACT_FAILED_TRANSFER_WITH_POSTHOC_RECOVERY_NOT_PROMOTED"
    assert result["file_count"] == 60 and result["source_file_count"] == 22
    assert result["png_count"] == 12 and result["history_rows"] == 9020
    assert result["recovery"]["file_count"] == 24 and result["recovery"]["png_count"] == 12
    assert result["original_failed_receipt_preserved"] is True
    assert result["diagnostic_gate_as_reported_by_independent_comparison"]["pass"] is gate_pass
    assert result["diagnostic_gate_recomputed_by_transfer_verifier"] is False
    assert result["checkpoint_payload_deserialized"] is False
    assert result["torch_imported"] is False and result["gpu_used"] is False


def test_external_inventory_pin_precedes_copied_run_access(tmp_path, monkeypatch):
    artifact, _inventory_sha, recovery_inventory_sha, comparison_sha = make_artifact(
        tmp_path, monkeypatch)
    monkeypatch.setattr(verifier, "verify_inventory",
                        lambda *_: pytest.fail("copied result opened before remote inventory authentication"))
    with pytest.raises(verifier.VerificationError, match="external remote inventory SHA"):
        verifier.verify(artifact_root=artifact, remote_inventory_sha256="f" * 64,
                        recovery_inventory_sha256=recovery_inventory_sha,
                        comparison_sha256=comparison_sha)


@pytest.mark.parametrize("mutation", [
    "inventory_mismatch", "payload", "extra", "empty_directory", "failed_receipt", "source",
    "runner_command", "history", "png", "scope", "model_config", "loss_config",
    "candidate_count", "supplemental_C", "postrun", "comparison_binding",
    "recovery_inventory_mismatch", "executed_wrapper", "postrun_wrapper",
    "recovery_extra", "recovery_plan", "recovery_scope", "recovery_checkpoint",
])
def test_tampering_is_fail_closed(tmp_path, monkeypatch, mutation):
    artifact, inventory_sha, recovery_inventory_sha, comparison_sha = make_artifact(tmp_path, monkeypatch)
    results, item = artifact / "remote_results", artifact / "remote_results" / verifier.ARM
    recovery = artifact / verifier.RECOVERY_RESULTS
    reseal = True
    if mutation == "inventory_mismatch":
        (artifact / "local_inventory.sha256").write_text("0" * 64 + "  wrong\n"); reseal = False
    elif mutation == "payload":
        (results / "view_report.json").write_text("changed"); reseal = False
    elif mutation == "extra": put(results / "unplanned.json", {})
    elif mutation == "empty_directory": (results / "unplanned_empty_directory").mkdir()
    elif mutation == "failed_receipt":
        row = json.loads((results / "failed.json").read_text()); row["status"] = "COMPLETE"; put(results / "failed.json", row)
    elif mutation == "source": (results / verifier.RUNNER_ARCHIVE).write_bytes(b"different runner")
    elif mutation == "runner_command":
        row = json.loads((item / "subgroup_started.json").read_text())
        row["command"][row["command"].index("--runner-sha256") + 1] = "f" * 64; put(item / "subgroup_started.json", row)
    elif mutation == "history":
        with (item / "training/metrics.jsonl").open("a") as stream: stream.write('{"global_step":9021}\n')
    elif mutation == "png": next((item / "evaluation/trajectories").glob("*.png")).unlink()
    elif mutation == "scope":
        row = json.loads((results / "failed.json").read_text()); row["automatic_promotion"] = True; put(results / "failed.json", row)
    elif mutation == "model_config":
        row = json.loads((item / "training/run.json").read_text()); row["model_config"]["candidate_count"] = 99; put(item / "training/run.json", row)
    elif mutation == "loss_config":
        row = json.loads((item / "evaluation/metrics.json").read_text()); row["loss_config"] = {}; put(item / "evaluation/metrics.json", row)
    elif mutation == "candidate_count":
        row = json.loads((item / "evaluation/metrics.json").read_text()); row["auxiliary_loss_metric_counts"] = {"candidate_regret_loss": 664}; put(item / "evaluation/metrics.json", row)
    elif mutation == "supplemental_C":
        (artifact / verifier.SUPPLEMENTAL_C_CHECKPOINT).write_bytes(b"changed C"); reseal = False
    elif mutation == "postrun":
        row = json.loads((artifact / verifier.POSTRUN_VERIFICATION).read_text()); row["source_verification_pass"] = False; put(artifact / verifier.POSTRUN_VERIFICATION, row); reseal = False
    elif mutation == "comparison_binding":
        path = artifact / "derived/comparison_C_E.json"; row = json.loads(path.read_text())
        row["inputs"]["E_experiment"]["plan_sha256"] = "f" * 64; put(path, row); comparison_sha = sha(path); reseal = False
    elif mutation == "recovery_inventory_mismatch":
        (artifact / "recovery_local_inventory.sha256").write_text("0" * 64 + "  wrong\n"); reseal = False
    elif mutation == "executed_wrapper":
        (artifact / verifier.EXECUTED_RECOVERY_WRAPPER).write_bytes(b"different executed wrapper"); reseal = False
    elif mutation == "postrun_wrapper":
        path = artifact / verifier.POSTRUN_VERIFICATION; row = json.loads(path.read_text())
        row["executed_recovery_wrapper_sha256"] = "f" * 64; put(path, row); reseal = False
    elif mutation == "recovery_extra": put(recovery / "unplanned.json", {}); reseal = False
    elif mutation == "recovery_plan":
        row = json.loads((recovery / "execution_plan.json").read_text()); row["loss_config"] = {}; put(recovery / "execution_plan.json", row); reseal = False
    elif mutation == "recovery_scope":
        row = json.loads((recovery / "recovery_summary.json").read_text()); row["automatic_retry"] = True; put(recovery / "recovery_summary.json", row); reseal = False
    else:
        row = json.loads((recovery / "completed.json").read_text()); row["checkpoint_sha256"] = "f" * 64; put(recovery / "completed.json", row); reseal = False
    if reseal: inventory_sha = seal_inventory(artifact)
    with pytest.raises(verifier.VerificationError):
        verifier.verify(artifact_root=artifact, remote_inventory_sha256=inventory_sha,
                        recovery_inventory_sha256=recovery_inventory_sha,
                        comparison_sha256=comparison_sha)


def test_cli_creates_one_report_and_never_overwrites_or_writes_inside_run(tmp_path, monkeypatch):
    artifact, inventory_sha, recovery_inventory_sha, comparison_sha = make_artifact(tmp_path, monkeypatch)
    output = artifact / "verification.json"
    args = ["--artifact-root", str(artifact), "--remote-inventory-sha256", inventory_sha,
            "--recovery-inventory-sha256", recovery_inventory_sha,
            "--comparison-sha256", comparison_sha, "--output", str(output)]
    assert verifier.main(args) == 0 and output.exists()
    with pytest.raises(verifier.VerificationError, match="fresh"): verifier.main(args)
    inside = artifact / "remote_results/verification.json"
    with pytest.raises(verifier.VerificationError, match="outside copied remote results"):
        verifier.main(args[:-1] + [str(inside)])
    assert not inside.exists()


def test_verifier_has_no_torch_gpu_or_process_imports():
    imports = [node for node in ast.walk(ast.parse(SCRIPT.read_text()))
               if isinstance(node, (ast.Import, ast.ImportFrom))]
    names = {alias.name for node in imports for alias in node.names}
    assert "torch" not in names and not any(name.startswith("torch.") for name in names)
    assert "subprocess" not in names
