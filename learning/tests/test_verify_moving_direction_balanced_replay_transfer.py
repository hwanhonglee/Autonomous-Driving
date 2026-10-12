"""HH_260906 - Exercise the successful F transfer verifier with CPU-only synthetic evidence."""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path

import pytest

from scripts.e2e import verify_moving_direction_balanced_replay_transfer as verifier


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def put(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if isinstance(value, bytes):
        path.write_bytes(value)
    else:
        path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n")


def seal(artifact: Path) -> str:
    root = artifact / "remote_results"
    rows = []
    for path in root.rglob("*"):
        if path.is_file() and not path.is_symlink():
            rows.append((path.relative_to(root).as_posix(), sha(path)))
    payload = "".join(f"{digest}  ./{name}\n" for name, digest in sorted(rows))
    (artifact / "remote_inventory.sha256").write_text(payload)
    (artifact / "local_inventory.sha256").write_text(payload)
    return sha(artifact / "remote_inventory.sha256")


def make_artifact(tmp_path: Path, monkeypatch: pytest.MonkeyPatch, *, gate_pass: bool = True):
    artifact = tmp_path / "artifact"
    root = artifact / "remote_results"
    root.mkdir(parents=True)
    sampling_plan = {"policy": "synthetic_direction_balanced", "seed": 20260906}
    sampling_sha = verifier.canonical_sha256(sampling_plan)
    schedule_sha = "d" * 64
    monkeypatch.setattr(verifier, "EXPECTED_SAMPLING_PLAN_SHA256", sampling_sha)
    monkeypatch.setattr(verifier, "EXPECTED_EXECUTION_SCHEDULE_SHA256", schedule_sha)

    current, base = {}, {}
    for name in verifier.SOURCE_BASE_PATHS:
        put(root / "source" / name, ("current:" + name).encode())
        current[name] = sha(root / "source" / name)
        base[name] = "b" * 64 if name in verifier.SOURCE_OVERLAYS else current[name]
    added = {}
    for name in verifier.SOURCE_ADDED_PATHS:
        put(root / "source" / name, ("added:" + name).encode())
        added[name] = sha(root / "source" / name)
    put(root / verifier.RUNNER_ARCHIVE, b"synthetic F runner")
    put(root / verifier.EVALUATOR_ARCHIVE, b"synthetic F evaluator")
    put(root / verifier.OWNER_ARCHIVE, b"synthetic GPU owner")
    put(root / "view_report.json", {"status": "synthetic sealed view"})

    reference_c = {}
    for label, relative in verifier.REFERENCE_C_PATHS.items():
        put(root / "reference_C" / relative, {"reference": "C", "label": label})
        reference_c[label] = {"path": relative, "sha256": sha(root / "reference_C" / relative)}
    reference_c["checkpoint"] = {
        "path": "A_baseline_train/training/checkpoints/latest.pt", "sha256": "c" * 64}
    reference_d = {}
    for label, relative in verifier.REFERENCE_D_PATHS.items():
        put(root / "reference_D" / relative,
            b"synthetic D checkpoint" if label == "checkpoint" else
            (json.dumps({"reference": "D", "label": label}) + "\n").encode())
        reference_d[label] = {"path": relative, "sha256": sha(root / "reference_D" / relative)}

    denials = {name: False for name in verifier.RUN_DENIALS}
    plan = {
        "schema": verifier.PLAN_SCHEMA,
        "campaign_id": verifier.CAMPAIGN_ID,
        "base_commit": verifier.BASE_COMMIT,
        "stage_order": list(verifier.STAGES),
        "source": {"base_commit": verifier.BASE_COMMIT, "base_sha256": base,
                   "current_sha256": current, "overlay_paths": list(verifier.SOURCE_OVERLAYS),
                   "added_paths": list(verifier.SOURCE_ADDED_PATHS), "added_sha256": added},
        "runner_sha256": sha(root / verifier.RUNNER_ARCHIVE),
        "evaluator_sha256": sha(root / verifier.EVALUATOR_ARCHIVE),
        "owner": {"path": "fixture", "sha256": sha(root / verifier.OWNER_ARCHIVE)},
        "view_report_sha256": sha(root / "view_report.json"),
        "arm": {"label": verifier.ARM,
                "expected_train_state": verifier.EXPECTED_TRAIN_STATE,
                "expected_sampling_stream_samples_seen": verifier.EXPECTED_STREAM_SAMPLES,
                "sampling_plan": sampling_plan, "sampling_plan_sha256": sampling_sha,
                "execution_schedule": {"row_count": 33120, "sha256": schedule_sha}},
        "validation": {"count": 665},
        "reference_C": {"files": reference_c},
        "reference_D": {"files": reference_d},
        "decision": {"adoption_allowed": False},
        **denials,
    }
    put(root / "plan.json", plan)
    plan_sha = sha(root / "plan.json")
    item = root / verifier.ARM
    put(item / ".training.portable-e2e.lock", b"")
    put(item / "training/checkpoints/latest.pt", b"opaque synthetic F checkpoint")
    checkpoint_sha = sha(item / "training/checkpoints/latest.pt")
    history = item / "training/metrics.jsonl"
    history.parent.mkdir(parents=True, exist_ok=True)
    with history.open("w") as stream:
        for step in range(1, verifier.EXPECTED_HISTORY_ROWS + 1):
            row = {"global_step": step}
            if step == verifier.EXPECTED_HISTORY_ROWS:
                row.update(samples_seen=33120, domain_samples_seen={"carla": 33120},
                           sampling_stream_samples_seen=verifier.EXPECTED_STREAM_SAMPLES)
            stream.write(json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n")
    put(item / "training/run.json", {
        "status": "TRAINING_TARGET_REACHED",
        "warning": "RESEARCH CHECKPOINT — NOT APPROVED FOR VEHICLE CONTROL",
        "state": verifier.EXPECTED_TRAIN_STATE,
        "train_config": {"sampling_policy": "direction_balanced_cohort_replay"},
        "sampling_plan_sha256": sampling_sha,
        "sampling_execution_schedule_sha256": schedule_sha,
        "sampling_execution_schedule_row_count": 33120,
        "sampling_metrics": {"sampling_stream_samples_seen": verifier.EXPECTED_STREAM_SAMPLES},
        "last_metrics": {"sampling_stream_samples_seen": verifier.EXPECTED_STREAM_SAMPLES},
    })
    common_eval = {
        "checkpoint_sha256": checkpoint_sha, "evaluation_split": "val",
        "evaluation_episode_count": 2, "training_sampling_plan_sha256": sampling_sha,
        "training_sampling_policy": "direction_balanced_cohort_replay",
        "vehicle_control_approved": False,
    }
    put(item / "evaluation/metrics.json", {
        "status": "OPEN_LOOP_EVALUATION_COMPLETE", "sample_count": 665,
        "rendered_trajectory_png": 12, **common_eval})
    put(item / "gate_v8.json", {"status": "RUNTIME_GEOMETRY_AUDIT_COMPLETE", **common_eval})
    for index in range(12):
        put(item / "evaluation/trajectories" / f"{index:04d}_synthetic.png",
            b"\x89PNG\r\n\x1a\n" + str(index).encode())

    subgroup_denials = {name: False for name in verifier.SUBGROUP_DENIALS}
    episode_rows = {}
    for label, (episode_id, count) in verifier.EXPECTED_EPISODES.items():
        report = item / "subgroup/episodes" / label / "metrics.json"
        put(report, {"episode_id": episode_id, "sample_count": count})
        png_hashes = {}
        for index in range(6):
            filename = f"{index:04d}_{label}_synthetic.png"
            path = report.parent / "trajectories" / filename
            put(path, b"\x89PNG\r\n\x1a\n" + label.encode() + str(index).encode())
            png_hashes[filename] = sha(path)
        episode_rows[label] = {
            "episode_id": episode_id, "count": count,
            "report_path": f"episodes/{label}/metrics.json", "report_sha256": sha(report),
            "trajectory_png_sha256": png_hashes,
        }
    subgroup = {
        "schema": verifier.SUBGROUP_SCHEMA, "status": "COMPLETE_NOT_PROMOTED",
        "arm": verifier.ARM, "plan_sha256": plan_sha,
        "runner_sha256": plan["runner_sha256"], "evaluator_sha256": plan["evaluator_sha256"],
        "checkpoint_sha256": checkpoint_sha,
        "checkpoint": {"sha256": checkpoint_sha, "sampling_plan_sha256": sampling_sha},
        "combined_val_count": 665, "episodes": episode_rows, **subgroup_denials,
    }
    put(item / "subgroup/summary.json", subgroup)

    stage_rows = []
    for index, stage in enumerate(verifier.STAGES):
        command = ["/venv/bin/python", "-m", f"portable_e2e.{stage}"]
        if stage == "train":
            command += ["--split", "train"]
        elif stage in ("evaluate", "audit"):
            command += ["--split", "val"]
        started = {
            "arm": verifier.ARM, "stage": stage, "command": command,
            "timeout_seconds": 100, "started_at_utc": f"2026-09-21T00:0{index}:00+00:00",
            "status": "RUNNING", "owner_pid": 1234, "plan_sha256": plan_sha,
            "input_checkpoint_sha256": None if stage == "train" else checkpoint_sha,
            "pre_cuda_gpu": {"physical_index": 0, "uuid": "GPU-fixture"},
        }
        put(item / f"{stage}_started.json", started)
        put(item / f"{stage}.log", ("log:" + stage).encode())
        completed = dict(started)
        report = item / verifier.STAGE_REPORTS[stage]
        completed.update(status="COMPLETE", ended_at_utc="2026-09-21T00:10:00+00:00",
                         path=f"{verifier.ARM}/{verifier.STAGE_REPORTS[stage]}",
                         sha256=sha(report), checkpoint_sha256=checkpoint_sha)
        if stage == "train":
            completed.update(history_sha256=sha(history),
                             sampling_execution_schedule_row_count=33120,
                             sampling_execution_schedule_sha256=schedule_sha)
        put(item / f"{stage}_completed.json", completed)
        stage_rows.append(completed)
    started_at = "2026-09-21T00:00:00+00:00"
    put(root / "started.json", {"schema": verifier.SUMMARY_SCHEMA, "status": "RUNNING",
        "started_at_utc": started_at, "plan_sha256": plan_sha, "arm": "NOT_RUN", "stages": [], **denials})
    gate_status = "PASS_SINGLE_SEED_DIAGNOSTIC" if gate_pass else "FAIL_SINGLE_SEED_DIAGNOSTIC"
    put(root / "summary.json", {"schema": verifier.SUMMARY_SCHEMA, "status": "COMPLETE_NOT_PROMOTED",
        "started_at_utc": started_at, "ended_at_utc": "2026-09-21T00:20:00+00:00",
        "plan_sha256": plan_sha, "arm": "COMPLETE", "stages": stage_rows,
        "source_dataset_reference_postcheck_pass": True,
        "sampling_execution_schedule": {"row_count": 33120, "sha256": schedule_sha},
        "diagnostic_gate": {"status": gate_status, "pass": gate_pass,
                            "adoption_allowed": False, "automatic_promotion": False}, **denials})
    return artifact, seal(artifact)


@pytest.mark.parametrize("gate_pass", [True, False])
def test_exact_successful_f_transfer_is_verified_without_gate_recomputation(tmp_path, monkeypatch, gate_pass):
    artifact, inventory_sha = make_artifact(tmp_path, monkeypatch, gate_pass=gate_pass)
    result = verifier.verify(artifact_root=artifact, remote_inventory_sha256=inventory_sha)
    assert result["status"] == "PASS_EXACT_SUCCESSFUL_F_TRANSFER_NOT_PROMOTED"
    assert result["file_count"] == 81 and result["source_file_count"] == 22
    assert result["reference_file_count"] == 10 and result["png_count"] == 24
    assert result["history_rows"] == 8280 and result["completed_stage_count"] == 4
    assert result["diagnostic_gate_as_reported"]["pass"] is gate_pass
    assert result["diagnostic_gate_recomputed_by_transfer_verifier"] is False
    assert result["checkpoint_payload_deserialized"] is result["torch_imported"] is result["gpu_used"] is False


def test_external_inventory_pin_precedes_copied_result_access(tmp_path, monkeypatch):
    artifact, _ = make_artifact(tmp_path, monkeypatch)
    monkeypatch.setattr(verifier, "verify_inventory",
                        lambda *_: pytest.fail("copied result opened before inventory authentication"))
    with pytest.raises(verifier.VerificationError, match="external remote inventory SHA"):
        verifier.verify(artifact_root=artifact, remote_inventory_sha256="a" * 64)


def test_remote_and_local_inventory_must_be_byte_identical(tmp_path, monkeypatch):
    artifact, inventory_sha = make_artifact(tmp_path, monkeypatch)
    with (artifact / "local_inventory.sha256").open("a") as stream:
        stream.write("\n")
    with pytest.raises(verifier.VerificationError, match="not byte-identical"):
        verifier.verify(artifact_root=artifact, remote_inventory_sha256=inventory_sha)


@pytest.mark.parametrize("name", ["./", "././path", "./../path", "../path", "/absolute/path"])
def test_inventory_rejects_noncanonical_or_repeated_prefix(tmp_path, name):
    inventory = tmp_path / "inventory.sha256"
    inventory.write_text(f"{'a' * 64}  {name}\n")
    with pytest.raises(verifier.VerificationError):
        verifier.read_inventory(inventory)


def test_inventory_normalization_still_rejects_duplicate_paths(tmp_path):
    inventory = tmp_path / "inventory.sha256"
    inventory.write_text(f"{'a' * 64}  ./same/path\n{'b' * 64}  same/path\n")
    with pytest.raises(verifier.VerificationError, match="duplicate inventory path"):
        verifier.read_inventory(inventory)


@pytest.mark.parametrize("mutation,pattern", [
    ("checkpoint", "evaluation provenance/scope differs"),
    ("closed_loop", "subgroup.closed_loop_evaluation must be false"),
    ("failed_receipt", "no failed.json"),
])
def test_checkpoint_scope_and_success_receipt_are_fail_closed(tmp_path, monkeypatch, mutation, pattern):
    artifact, _ = make_artifact(tmp_path, monkeypatch)
    root = artifact / "remote_results"
    if mutation == "checkpoint":
        path = root / verifier.ARM / "evaluation/metrics.json"
        value = json.loads(path.read_text()); value["checkpoint_sha256"] = "0" * 64; put(path, value)
    elif mutation == "closed_loop":
        path = root / verifier.ARM / "subgroup/summary.json"
        value = json.loads(path.read_text()); value["closed_loop_evaluation"] = True; put(path, value)
    else:
        put(root / "failed.json", {"status": "FAIL"})
    inventory_sha = seal(artifact)
    with pytest.raises(verifier.VerificationError, match=pattern):
        verifier.verify(artifact_root=artifact, remote_inventory_sha256=inventory_sha)


def test_symlink_and_special_file_are_rejected(tmp_path, monkeypatch):
    artifact, inventory_sha = make_artifact(tmp_path, monkeypatch)
    root = artifact / "remote_results"
    target = root / verifier.ARM / "train.log"
    payload = target.read_bytes(); target.unlink()
    outside = artifact / "outside.log"; outside.write_bytes(payload)
    target.symlink_to(outside)
    with pytest.raises(verifier.VerificationError, match="symlink forbidden"):
        verifier.verify(artifact_root=artifact, remote_inventory_sha256=inventory_sha)
    target.unlink(); target.write_bytes(payload)
    fifo = root / "unexpected.fifo"; os.mkfifo(fifo)
    with pytest.raises(verifier.VerificationError, match="special or hard-linked"):
        verifier.verify(artifact_root=artifact, remote_inventory_sha256=inventory_sha)


def test_constants_and_source_do_not_import_torch_or_deserialize_checkpoint():
    assert verifier.EXPECTED_SAMPLING_PLAN_SHA256 == \
        "abb0e655b174844cc20f293bc12c5156fe33b3a00741ac1cfe8b703367e41c7b"
    assert verifier.EXPECTED_EXECUTION_SCHEDULE_SHA256 == \
        "d340a10c42f2a733988b69ae6571ec3c37b7e8fa1601c43e6111192cd35c55b0"
    source = Path(verifier.__file__).read_text()
    assert "import torch" not in source and "torch.load" not in source
    assert "checkpoint_payload_deserialized\": False" in source
