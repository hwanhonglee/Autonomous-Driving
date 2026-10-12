"""HH_260906 - Exercise the exact F postrun verifier with CPU-only synthetic evidence."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest

from scripts.e2e import verify_moving_direction_balanced_replay_postrun as verifier


def put(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if isinstance(value, bytes):
        path.write_bytes(value)
    else:
        path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n")


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def canonical(value) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                 allow_nan=False).encode()).hexdigest()


def denials(fields: tuple[str, ...]) -> dict[str, bool]:
    return {field: False for field in fields}


def replace_json(path: Path, update) -> None:
    value = json.loads(path.read_text())
    update(value)
    put(path, value)


def make_campaign(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    root = tmp_path / "campaign"
    root.mkdir()
    state = {
        "epoch": 20,
        "next_batch_index": 0,
        "global_step": 2,
        "samples_seen": 8,
        "domain_samples_seen": {"carla": 8},
    }
    stream_samples = {"base": 6, "replay": 2}
    sampling_plan = {"schema": "portable_e2e.synthetic_direction_balanced_replay.v1",
                     "right_replay_count": 2}
    schedule_rows = [
        {"epoch": 0, "position": index, "stream": "replay" if index in (3, 7) else "base",
         "occurrence_index": index, "dataset_index": index}
        for index in range(8)
    ]
    sampling_sha = canonical(sampling_plan)
    schedule_sha = canonical(schedule_rows)
    monkeypatch.setattr(verifier, "EXPECTED_TRAIN_STATE", state)
    monkeypatch.setattr(verifier, "EXPECTED_STREAM_SAMPLES", stream_samples)
    monkeypatch.setattr(verifier, "EXPECTED_SAMPLING_PLAN_SHA256", sampling_sha)
    monkeypatch.setattr(verifier, "EXPECTED_SCHEDULE_SHA256", schedule_sha)
    monkeypatch.setattr(verifier, "EXPECTED_SCHEDULE_ROWS", len(schedule_rows))

    current: dict[str, str] = {}
    base: dict[str, str] = {}
    for name in verifier.SOURCE_CURRENT_PATHS:
        put(root / "source" / name, ("current:" + name).encode())
        current[name] = sha(root / "source" / name)
        base[name] = "0" * 64 if name in verifier.SOURCE_OVERLAYS else current[name]
    added: dict[str, str] = {}
    for name in verifier.SOURCE_ADDED_PATHS:
        put(root / "source" / name, ("added:" + name).encode())
        added[name] = sha(root / "source" / name)
    put(root / verifier.RUNNER_ARCHIVE, b"synthetic F runner")
    put(root / verifier.EVALUATOR_ARCHIVE, b"synthetic F subgroup evaluator")
    put(root / verifier.OWNER_ARCHIVE, b"synthetic training owner")
    put(root / "view_report.json", {"status": "SYNTHETIC_VIEW"})

    c_checkpoint_sha = "c" * 64
    d_checkpoint = root / "reference_D" / verifier.REFERENCE_LAYOUT["reference_D"]["checkpoint"]
    put(d_checkpoint, b"synthetic D checkpoint")
    d_checkpoint_sha = sha(d_checkpoint)
    reference_payloads = {
        "reference_C": {
            "plan": {"kind": "C plan"},
            "summary": {"kind": "C summary"},
            "training_run": {"kind": "C training"},
            "subgroup": {"checkpoint_sha256": c_checkpoint_sha,
                         "checkpoint": {"sha256": c_checkpoint_sha}},
            "gate": {"checkpoint_sha256": c_checkpoint_sha},
        },
        "reference_D": {
            "training_run": {"kind": "D training"},
            "evaluation": {"checkpoint_sha256": d_checkpoint_sha},
            "subgroup": {"checkpoint_sha256": d_checkpoint_sha,
                         "checkpoint": {"sha256": d_checkpoint_sha}},
            "gate": {"checkpoint_sha256": d_checkpoint_sha},
        },
    }
    reference_plan: dict[str, dict] = {}
    for key, payloads in reference_payloads.items():
        files = {
            "checkpoint": {
                "path": verifier.REFERENCE_LAYOUT[key]["checkpoint"],
                "sha256": c_checkpoint_sha if key == "reference_C" else d_checkpoint_sha,
            }
        }
        for label, payload in payloads.items():
            relative = verifier.REFERENCE_LAYOUT[key][label]
            put(root / key / relative, payload)
            files[label] = {"path": relative, "sha256": sha(root / key / relative)}
        reference_plan[key] = {"files": files}

    validation = [
        {"label": label, "episode_id": episode_id, "count": count}
        for label, (episode_id, count) in verifier.EXPECTED_EPISODES.items()
    ]
    plan = {
        "schema": verifier.PLAN_SCHEMA,
        "campaign_id": verifier.CAMPAIGN_ID,
        "base_commit": verifier.BASE_COMMIT,
        "stage_order": list(verifier.STAGES),
        "source": {
            "base_commit": verifier.BASE_COMMIT,
            "base_sha256": base,
            "current_sha256": current,
            "overlay_paths": list(verifier.SOURCE_OVERLAYS),
            "added_paths": list(verifier.SOURCE_ADDED_PATHS),
            "added_sha256": added,
        },
        "runner_sha256": sha(root / verifier.RUNNER_ARCHIVE),
        "evaluator_sha256": sha(root / verifier.EVALUATOR_ARCHIVE),
        "owner": {"path": "fixture", "sha256": sha(root / verifier.OWNER_ARCHIVE)},
        "view_report_sha256": sha(root / "view_report.json"),
        "arm": {
            "label": verifier.ARM,
            "sampling_plan": sampling_plan,
            "sampling_plan_sha256": sampling_sha,
            "execution_schedule": {
                "row_count": len(schedule_rows), "rows": schedule_rows, "sha256": schedule_sha,
            },
            "expected_train_state": state,
            "expected_sampling_stream_samples_seen": stream_samples,
        },
        "validation": {"count": 665, "episodes": validation},
        "decision": {"adoption_allowed": False},
        **reference_plan,
        **denials(verifier.RUN_DENIALS),
    }
    put(root / "plan.json", plan)
    plan_sha = sha(root / "plan.json")
    monkeypatch.setattr(verifier, "EXPECTED_PLAN_SHA256", plan_sha)

    item = root / verifier.ARM
    checkpoint = item / "training/checkpoints/latest.pt"
    put(checkpoint, b"synthetic F checkpoint")
    checkpoint_sha = sha(checkpoint)
    history = item / "training/metrics.jsonl"
    history.parent.mkdir(parents=True, exist_ok=True)
    history.write_text(
        json.dumps({"global_step": 1}, sort_keys=True, separators=(",", ":")) + "\n"
        + json.dumps({"global_step": 2, "samples_seen": 8,
                      "domain_samples_seen": {"carla": 8},
                      "sampling_stream_samples_seen": stream_samples},
                     sort_keys=True, separators=(",", ":")) + "\n"
    )
    train = {
        "status": "TRAINING_TARGET_REACHED",
        "warning": "RESEARCH CHECKPOINT — NOT APPROVED FOR VEHICLE CONTROL",
        "state": state,
        "train_config": {"sampling_policy": "direction_balanced_cohort_replay"},
        "sampling_plan": sampling_plan,
        "sampling_plan_sha256": sampling_sha,
        "sampling_execution_schedule_sha256": schedule_sha,
        "sampling_execution_schedule_row_count": len(schedule_rows),
        "sampling_metrics": {"samples_seen": 8, "domain_samples_seen": {"carla": 8},
                             "sampling_stream_samples_seen": stream_samples},
        "last_metrics": {"samples_seen": 8, "domain_samples_seen": {"carla": 8},
                         "sampling_stream_samples_seen": stream_samples},
    }
    put(item / "training/run.json", train)
    evaluation = {
        "status": "OPEN_LOOP_EVALUATION_COMPLETE",
        "checkpoint_sha256": checkpoint_sha,
        "evaluation_split": "val",
        "evaluation_episode_count": 2,
        "sample_count": 665,
        "rendered_trajectory_png": 12,
        "training_sampling_plan_sha256": sampling_sha,
        "training_sampling_policy": "direction_balanced_cohort_replay",
        "training_sampling_stream_samples_seen": stream_samples,
        "vehicle_control_approved": False,
    }
    put(item / "evaluation/metrics.json", evaluation)
    audit = {
        "status": "RUNTIME_GEOMETRY_AUDIT_COMPLETE",
        "checkpoint_sha256": checkpoint_sha,
        "evaluation_split": "val",
        "evaluation_episode_count": 2,
        "training_sampling_plan_sha256": sampling_sha,
        "training_sampling_policy": "direction_balanced_cohort_replay",
        "vehicle_control_approved": False,
        "geometry": {
            "sample_count": 665,
            "selected_result": {"sample_count": 665, "geometry_pass_count": 664},
            "any_candidate_geometry_pass_count": 664,
            "all_candidates_geometry_pass_count": 664,
        },
    }
    put(item / "gate_v8.json", audit)
    for index in range(12):
        put(item / "evaluation/trajectories" / f"{index:04d}_synthetic.png",
            b"\x89PNG\r\n\x1a\n" + str(index).encode())

    episode_rows: dict[str, dict] = {}
    for label, (episode_id, count) in verifier.EXPECTED_EPISODES.items():
        report_path = f"episodes/{label}/metrics.json"
        put(item / "subgroup" / report_path,
            {"checkpoint_sha256": checkpoint_sha, "sample_count": count})
        png_hashes: dict[str, str] = {}
        for index in range(6):
            filename = f"{index:04d}_{label}.png"
            path = item / "subgroup/episodes" / label / "trajectories" / filename
            put(path, b"\x89PNG\r\n\x1a\n" + label.encode() + str(index).encode())
            png_hashes[filename] = sha(path)
        episode_rows[label] = {
            "episode_id": episode_id,
            "count": count,
            "report_path": report_path,
            "report_sha256": sha(item / "subgroup" / report_path),
            "trajectory_png_sha256": png_hashes,
        }
    subgroup = {
        "schema": verifier.SUBGROUP_SCHEMA,
        "status": "COMPLETE_NOT_PROMOTED",
        "arm": verifier.ARM,
        "plan_sha256": plan_sha,
        "runner_sha256": plan["runner_sha256"],
        "evaluator_sha256": plan["evaluator_sha256"],
        "checkpoint_sha256": checkpoint_sha,
        "checkpoint": {"sha256": checkpoint_sha, "sampling_plan_sha256": sampling_sha},
        "combined_val_count": 665,
        "episodes": episode_rows,
        **denials(verifier.SUBGROUP_DENIALS),
    }
    put(item / "subgroup/summary.json", subgroup)

    completed_rows = []
    for stage in verifier.STAGES:
        command = ["python", stage]
        common = {
            "arm": verifier.ARM,
            "stage": stage,
            "command": command,
            "timeout_seconds": 10,
            "started_at_utc": f"2026-09-21T00:00:0{len(completed_rows)}+00:00",
            "owner_pid": 123,
            "plan_sha256": plan_sha,
            "input_checkpoint_sha256": None if stage == "train" else checkpoint_sha,
            "pre_cuda_gpu": {"uuid": "synthetic-gpu0"},
        }
        put(item / f"{stage}_started.json", {**common, "status": "RUNNING"})
        report_relative = f"{verifier.ARM}/{verifier.STAGE_REPORTS[stage]}"
        completed = {
            **common,
            "status": "COMPLETE",
            "ended_at_utc": f"2026-09-21T00:01:0{len(completed_rows)}+00:00",
            "checkpoint_sha256": checkpoint_sha,
            "path": report_relative,
            "sha256": sha(root / report_relative),
        }
        if stage == "train":
            completed.update({
                "history_sha256": sha(history),
                "sampling_execution_schedule_row_count": len(schedule_rows),
                "sampling_execution_schedule_sha256": schedule_sha,
            })
        put(item / f"{stage}_completed.json", completed)
        completed_rows.append(completed)

    started_at = "2026-09-21T00:00:00+00:00"
    put(root / "started.json", {
        "schema": verifier.SUMMARY_SCHEMA,
        "status": "RUNNING",
        "plan_sha256": plan_sha,
        "arm": "NOT_RUN",
        "stages": [],
        "started_at_utc": started_at,
        **denials(verifier.RUN_DENIALS),
    })
    gate_checks = {
        name: {"observed_F": 664, "minimum": 664, "pass": True}
        for name in ("selected", "any_candidate", "all_candidates")
    }
    put(root / "summary.json", {
        "schema": verifier.SUMMARY_SCHEMA,
        "status": "COMPLETE_NOT_PROMOTED",
        "plan_sha256": plan_sha,
        "arm": "COMPLETE",
        "stages": completed_rows,
        "started_at_utc": started_at,
        "source_dataset_reference_postcheck_pass": True,
        "sampling_execution_schedule": {"row_count": len(schedule_rows), "sha256": schedule_sha},
        "diagnostic_gate": {
            "status": "FAIL_SINGLE_SEED_DIAGNOSTIC",
            "pass": False,
            "adoption_allowed": False,
            "automatic_promotion": False,
            "geometry": {"sample_count": 665, "pass": True, "checks": gate_checks},
        },
        **denials(verifier.RUN_DENIALS),
    })
    return root


def test_failed_performance_gate_is_verified_not_promoted_and_output_is_create_only(
        tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    root = make_campaign(tmp_path, monkeypatch)
    output = tmp_path / "postrun.json"
    assert verifier.main(["--campaign-root", str(root), "--output-json", str(output)]) == 0
    result = json.loads(output.read_text())
    assert result["status"] == "VERIFIED_NOT_PROMOTED"
    assert result["diagnostic_gate_status"] == "FAIL_SINGLE_SEED_DIAGNOSTIC"
    assert result["model_performance_gate_passed"] is False
    assert result["provenance_verified"] is True
    assert result["completed_stage_count"] == 4
    assert result["evaluation_sample_count"] == 665
    assert result["subgroup_sample_counts"] == {
        "town03_right_turn_dev": 337, "town03_straight_dev": 328,
    }
    assert result["geometry_pass_count"] == 664 and result["geometry_sample_count"] == 665
    assert result["png_count"] == 24 and result["failed_json_count"] == 0
    preserved = output.read_bytes()
    assert verifier.main(["--campaign-root", str(root), "--output-json", str(output)]) == 2
    assert output.read_bytes() == preserved


@pytest.mark.parametrize("mutation", (
    "checkpoint", "source", "reference", "evaluation_count", "stage_checkpoint",
    "failed_json", "png_missing", "geometry", "gate_promoted", "stream_accounting",
))
def test_provenance_mutations_fail_closed_without_output(
        tmp_path: Path, monkeypatch: pytest.MonkeyPatch, mutation: str) -> None:
    root = make_campaign(tmp_path, monkeypatch)
    item = root / verifier.ARM
    if mutation == "checkpoint":
        (item / "training/checkpoints/latest.pt").write_bytes(b"changed checkpoint")
    elif mutation == "source":
        (root / "source/portable_e2e/train.py").write_bytes(b"changed source")
    elif mutation == "reference":
        (root / "reference_C/plan.json").write_bytes(b"changed reference")
    elif mutation == "evaluation_count":
        replace_json(item / "evaluation/metrics.json", lambda value: value.update(sample_count=664))
    elif mutation == "stage_checkpoint":
        replace_json(item / "evaluate_completed.json",
                     lambda value: value.update(checkpoint_sha256="f" * 64))
    elif mutation == "failed_json":
        put(item / "failed.json", {"status": "FAILED"})
    elif mutation == "png_missing":
        next((item / "evaluation/trajectories").glob("*.png")).unlink()
    elif mutation == "geometry":
        replace_json(item / "gate_v8.json",
                     lambda value: value["geometry"]["selected_result"].update(
                         geometry_pass_count=663))
    elif mutation == "gate_promoted":
        replace_json(root / "summary.json",
                     lambda value: value["diagnostic_gate"].update(
                         status="PASS_SINGLE_SEED_DIAGNOSTIC", automatic_promotion=True))
    elif mutation == "stream_accounting":
        replace_json(item / "training/run.json",
                     lambda value: value["sampling_metrics"].update(
                         sampling_stream_samples_seen={"base": 8, "replay": 0}))
    output = tmp_path / "rejected.json"
    assert verifier.main(["--campaign-root", str(root), "--output-json", str(output)]) == 2
    assert not output.exists()


@pytest.mark.parametrize("bad", ('{"x":NaN}', '{"x":1e999}', '{"x":0,"x":1}'))
def test_strict_json_rejects_nonfinite_and_duplicate_values(tmp_path: Path, bad: str) -> None:
    path = tmp_path / "bad.json"
    path.write_text(bad)
    with pytest.raises(verifier.VerificationError):
        verifier.read_json(path)


def test_output_inside_campaign_is_rejected_before_verification(
        tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    root = make_campaign(tmp_path, monkeypatch)
    output = root / "postrun.json"
    assert verifier.main(["--campaign-root", str(root), "--output-json", str(output)]) == 2
    assert not output.exists()
