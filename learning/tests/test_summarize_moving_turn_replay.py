# HH_260906 - Locate the repository independently of the relocated test directory.
"""HH_260906 - CPU tests seal the independent C-versus-E replay verifier."""
from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path

import pytest

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path


ROOT = next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())
PATH = _hh_source_path(ROOT, "scripts/e2e/summarize_moving_turn_replay.py")
SPEC = importlib.util.spec_from_file_location("summarize_moving_turn_replay_test", PATH)
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)

RUNNER_BYTES = b"fixture turn replay runner\n"
EVALUATOR_BYTES = b"fixture turn replay evaluator\n"
OWNER_BYTES = b"fixture GPU owner\n"
VIEW_BYTES = b"fixture view report\n"
TRAIN_OVERLAY_BYTES = b"fixture train overlay\n"
EVALUATE_OVERLAY_BYTES = b"fixture evaluate overlay\n"
ADDED_HELPER_BYTES = b"fixture added subgroup helper\n"


def byte_sha(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def put(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(m._encoded(value))


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def data_spec(name: str, train_count: int) -> dict:
    digit = "1" if train_count == 1147 else "2"
    train_episodes = (["town07", "c-track", "town01-right"] if train_count == 1147 else
                      ["town07-old", "c-track", "town01-right", "town07-new"])
    return {
        "path": f"datasets/prepared/{name}",
        "manifest_sha256": digit * 64,
        "corpus_fingerprint_sha256": ("3" if train_count == 1147 else "4") * 64,
        "split_fingerprint_sha256": {"train": ("5" if train_count == 1147 else "6") * 64,
                                     "val": "7" * 64},
        "split_loader_sha256": {"train": ("8" if train_count == 1147 else "9") * 64,
                                "val": "a" * 64},
        "split_examples_sha256": {"train": ("b" if train_count == 1147 else "c") * 64,
                                  "val": "d" * 64},
        "split_tokens_sha256": {"train": ("e" if train_count == 1147 else "f") * 64,
                                "val": "0" * 64},
        "counts": {"train": train_count, "val": 665},
        "episode_ids": {
            "train": train_episodes,
            "val": [item["episode_id"] for item in m.EPISODES],
        },
    }


def sampling_plan(monkeypatch) -> dict:
    base = [
        {"base_occurrence_index": index, "dataset_index": index,
         "sample_token": f"token-{index:04d}", "domain": "carla"}
        for index in range(1475)
    ]
    replay = [
        {"replay_occurrence_index": position, "dataset_index": index,
         "sample_token": f"token-{index:04d}", "domain": "carla"}
        for position, index in enumerate(range(327))
    ]
    token_sha = m._canonical_sha256([row["sample_token"] for row in replay])
    index_token_sha = m._canonical_sha256([
        {"dataset_index": row["dataset_index"], "token": row["sample_token"]}
        for row in replay
    ])
    monkeypatch.setattr(m, "EXPECTED_MEMBERSHIP_TOKEN_SHA256", token_sha)
    monkeypatch.setattr(m, "EXPECTED_MEMBERSHIP_INDEX_TOKEN_SHA256", index_token_sha)
    cohort = {
        "schema": "portable_e2e.cohort_replay.v1",
        "criterion": m.EXPECTED_COHORT_CRITERION,
        "occurrence_order_hash_encoding": "sha256_canonical_json_array_v1",
        "replay_copies_per_epoch": 1,
        "base_sample_count": 1475,
        "membership_sample_count": 327,
        "membership_sample_tokens_sha256": token_sha,
        "membership_index_token_sha256": index_token_sha,
        "replay_occurrence_count": 327,
        "base_occurrences_sha256": m._canonical_sha256(base),
        "base_occurrences": base,
        "replay_occurrences_sha256": m._canonical_sha256(replay),
        "replay_occurrences": replay,
    }
    return {
        "batch_size": 4,
        "batches_per_epoch": 451,
        "dataset_domain_counts": {"carla": 1475},
        "domain_ratios": {},
        "epoch_domain_sample_counts": {"carla": 1802},
        "epoch_replayed_domain_sample_counts": {"carla": 327},
        "epoch_discarded_sample_counts": {"carla": 0},
        "epoch_sample_count": 1802,
        "known_domains": ["carla", "real"],
        "order_algorithm": "smooth_proportional_cohort_replay_interleave_v1",
        "policy": "cohort_replay",
        "replacement": False,
        "cross_stream_sample_reuse": True,
        "seed_derivation": "sha256_seed_epoch_named_stream_v1",
        "cohort_replay": cohort,
    }


def execution_schedule(sampling: dict) -> dict:
    rows = []
    base = sampling["cohort_replay"]["base_occurrences"]
    replay = sampling["cohort_replay"]["replay_occurrences"]
    for epoch in range(20):
        position = 0
        for stream, occurrences in (("base", base), ("replay", replay)):
            for occurrence_index, occurrence in enumerate(occurrences):
                rows.append({"epoch": epoch, "position": position, "stream": stream,
                             "occurrence_index": occurrence_index,
                             "dataset_index": occurrence["dataset_index"]})
                position += 1
    return {
        "schema": "portable_e2e.cohort_replay_execution_schedule.v1",
        "epoch_count": 20,
        "epoch_sample_count": 1802,
        "row_count": 36040,
        "hash_encoding": "sha256_canonical_json_v1",
        "sha256": m._canonical_sha256(rows),
        "rows": rows,
    }


def configs() -> tuple[dict, dict]:
    common = {
        "seed": 20260906,
        "batch_size": 4,
        "learning_rate": 0.0001,
        "weight_decay": 0.0001,
        "num_workers": 0,
        "maximum_gradient_norm": 5.0,
        "verify_image_sha256": True,
        "domain_ratios": [],
    }
    return (
        {**common, "max_steps": 7380, "checkpoint_interval": 154,
         "sampling_policy": "uniform_without_replacement"},
        {**common, "max_steps": 9020, "checkpoint_interval": 451,
         "sampling_policy": "cohort_replay"},
    )


def common_plan(schema: str, campaign: str) -> dict:
    result = {
        "schema": schema,
        "campaign_id": campaign,
        "evaluator_sha256": ("3" * 64 if schema == m.REFERENCE_PLAN_SCHEMA else
                             byte_sha(EVALUATOR_BYTES)),
        "owner": {"path": "owner.py", "sha256": byte_sha(OWNER_BYTES)},
        "gpu": {"physical_index": 0, "uuid": "GPU-fixture"},
        "model_config": "model.json",
        "model_config_sha256": "5" * 64,
        "model_config_canonical_sha256": "6" * 64,
        "model_id": "fixture.model",
        "loss_config": {"xy_weight": 1.0},
        "view_report_sha256": byte_sha(VIEW_BYTES),
        "resume": False,
        "automatic_retry": False,
        "test_read_or_use": False,
        "model_export": False,
        "automatic_promotion": False,
        "training_data_approved": False,
        "vehicle_control_approved": False,
        "closed_loop_claim": False,
        "all_map_claim": False,
        "deployment_approved": False,
    }
    if schema == m.REFERENCE_PLAN_SCHEMA:
        result.update({
            "source_commit": "1" * 40,
            "source_sha256": {
                "portable_e2e/train.py": "2" * 64,
                "portable_e2e/evaluate.py": "8" * 64,
            },
        })
    else:
        result.update({
            "base_commit": "1" * 40,
            "runner_sha256": byte_sha(RUNNER_BYTES),
            "source": {
                "base_commit": "1" * 40,
                "base_sha256": {
                    "portable_e2e/train.py": "2" * 64,
                    "portable_e2e/evaluate.py": "8" * 64,
                },
                "current_sha256": {
                    "portable_e2e/train.py": byte_sha(TRAIN_OVERLAY_BYTES),
                    "portable_e2e/evaluate.py": byte_sha(EVALUATE_OVERLAY_BYTES),
                },
                "overlay_paths": m.EXPECTED_OVERLAY_PATHS,
                "added_paths": m.EXPECTED_ADDED_SOURCE_PATHS,
                "added_sha256": {
                    m.EXPECTED_ADDED_SOURCE_PATHS[0]: byte_sha(ADDED_HELPER_BYTES),
                },
            },
        })
    return result


def metric_values(label: str, arm: str) -> dict[str, float]:
    c = {
        "town03_right_turn_dev": {"ade_6p4s_m": 10.0, "fde_6p4s_m": 20.0,
                                  "speed_mae_6p4s_mps": 4.0, "loss": 3.0},
        "town03_straight_dev": {"ade_6p4s_m": 20.0, "fde_6p4s_m": 40.0,
                                "speed_mae_6p4s_mps": 8.0, "loss": 6.0},
    }
    e = {
        "town03_right_turn_dev": {"ade_6p4s_m": 9.0, "fde_6p4s_m": 19.0,
                                  "speed_mae_6p4s_mps": 4.1, "loss": 2.8},
        "town03_straight_dev": {"ade_6p4s_m": 18.0, "fde_6p4s_m": 35.0,
                                "speed_mae_6p4s_mps": 7.0, "loss": 5.0},
    }
    return (c if arm == "C" else e)[label]


def write_history(path: Path, arm: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as stream:
        if arm == "C":
            for step in range(1, 7381):
                seen = 29495 if step == 7380 else step
                row = {"global_step": step, "epoch": min(step // 287, 25),
                       "samples_seen": seen, "domain_samples_seen": {"carla": seen}}
                stream.write((json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n").encode())
            return
        totals = {"base": 0, "replay": 0}
        step = 0
        for epoch in range(20):
            labels = ["base"] * 1475 + ["replay"] * 327
            for offset in range(0, 1802, 4):
                step += 1
                batch = {"base": 0, "replay": 0}
                for label in labels[offset:offset + 4]:
                    batch[label] += 1
                    totals[label] += 1
                seen = sum(totals.values())
                row = {
                    "global_step": step,
                    "epoch": epoch,
                    "samples_seen": seen,
                    "domain_samples_seen": {"carla": seen},
                    "batch_sampling_stream_counts": batch,
                    "sampling_stream_samples_seen": dict(totals),
                }
                stream.write((json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n").encode())


def geometry(count: int = 664) -> dict:
    return {
        "sample_count": 665,
        "selected_result": {"sample_count": 665, "geometry_pass_count": count},
        "any_candidate_geometry_pass_count": count,
        "all_candidates_geometry_pass_count": count,
    }


def write_arm(root: Path, plan: dict, arm: str, label: str, data: dict,
              config: dict, state: dict, sampling: dict | None, geometry_count: int = 664) -> dict:
    item = root / arm
    checkpoint = item / "training/checkpoints/latest.pt"
    checkpoint.parent.mkdir(parents=True, exist_ok=True)
    checkpoint.write_bytes(f"checkpoint-{label}".encode())
    checkpoint_sha = sha(checkpoint)
    write_history(item / "training/metrics.jsonl", label)
    train = {
        "status": "TRAINING_TARGET_REACHED",
        "warning": "RESEARCH CHECKPOINT — NOT APPROVED FOR VEHICLE CONTROL",
        "state": state,
        "dataset_size": data["counts"]["train"],
        "dataset_fingerprint_sha256": data["split_fingerprint_sha256"]["train"],
        "corpus_fingerprint_sha256": data["corpus_fingerprint_sha256"],
        "train_config": config,
        "loss_config": plan["loss_config"],
        "device": "cuda:0",
        "training_episode_ids": sorted(data["episode_ids"]["train"]),
    }
    if sampling is not None:
        train.update({
            "sampling_plan": sampling,
            "sampling_plan_sha256": m._canonical_sha256(sampling),
            "sampling_metrics": {"sampling_stream_samples_seen": {"base": 29500, "replay": 6540}},
            "last_metrics": {"sampling_stream_samples_seen": {"base": 29500, "replay": 6540}},
            "sampling_execution_schedule_sha256": plan["arm"]["execution_schedule"]["sha256"],
            "sampling_execution_schedule_row_count": 36040,
        })
    put(item / "training/run.json", train)
    episode_rows = {}
    combined = {}
    for spec in m.EPISODES:
        metrics = metric_values(spec["label"], label)
        report = {"sample_count": spec["count"], "metrics": metrics}
        report_path = item / "subgroup/episodes" / spec["label"] / "metrics.json"
        put(report_path, report)
        episode_rows[spec["label"]] = {
            "episode_id": spec["episode_id"],
            "count": spec["count"],
            "metrics": metrics,
            "metric_counts": {key: spec["count"] for key in metrics},
            "report_path": f"episodes/{spec['label']}/metrics.json",
            "report_sha256": sha(report_path),
        }
    for key in next(iter(episode_rows.values()))["metrics"]:
        combined[key] = sum(row["metrics"][key] * row["count"]
                            for row in episode_rows.values()) / 665
    evaluation = {
        "status": "OPEN_LOOP_EVALUATION_COMPLETE",
        "evaluation_split": "val",
        "sample_count": 665,
        "evaluation_episode_count": 2,
        "checkpoint_sha256": checkpoint_sha,
        "corpus_fingerprint_sha256": data["corpus_fingerprint_sha256"],
        "dataset_fingerprint_sha256": data["split_fingerprint_sha256"]["val"],
        "training_dataset_fingerprint_sha256": data["split_fingerprint_sha256"]["train"],
        "model_config_sha256": plan["model_config_canonical_sha256"],
        "loss_config": plan["loss_config"],
        "vehicle_control_approved": False,
        "metrics": combined,
        "metric_counts": {key: 665 for key in combined},
    }
    if sampling is not None:
        evaluation.update({
            "training_sampling_plan_sha256": m._canonical_sha256(sampling),
            "training_sampling_policy": "cohort_replay",
        })
    put(item / "evaluation/metrics.json", evaluation)
    subgroup = {
        "schema": (m.EXPERIMENT_SUBGROUP_SCHEMA if sampling is not None else
                   m.REFERENCE_SUBGROUP_SCHEMA),
        "status": "COMPLETE_NOT_PROMOTED",
        "arm": arm,
        "plan_sha256": sha(root / "plan.json"),
        "evaluator_sha256": plan["evaluator_sha256"],
        "combined_val_count": 665,
        "checkpoint_sha256": checkpoint_sha,
        "corpus_fingerprint_sha256": data["corpus_fingerprint_sha256"],
        "test_read_or_use": False,
        "closed_loop_evaluation": False,
        "automatic_promotion": False,
        "model_exported": False,
        "training_executed_by_evaluator": False,
        "vehicle_control_approved": False,
        "dataset": {
            "combined_val_example_fingerprint_sha256": data["split_examples_sha256"]["val"],
            "combined_val_fingerprint_sha256": data["split_fingerprint_sha256"]["val"],
            "train_fingerprint_sha256": data["split_fingerprint_sha256"]["train"],
            "manifest_sha256": data["manifest_sha256"],
            "counts": data["counts"],
        },
        "checkpoint": {
            "sha256": checkpoint_sha,
            "training_dataset_fingerprint_sha256": data["split_fingerprint_sha256"]["train"],
            "corpus_fingerprint_sha256": data["corpus_fingerprint_sha256"],
            "model_config_sha256": plan["model_config_canonical_sha256"],
        },
        "episodes": episode_rows,
    }
    if sampling is not None:
        subgroup.update({"base_commit": plan["base_commit"], "source": plan["source"],
                         "runner_sha256": plan["runner_sha256"]})
        subgroup["checkpoint"]["sampling_plan_sha256"] = m._canonical_sha256(sampling)
    else:
        subgroup.update({"source_commit": plan["source_commit"],
                         "source_sha256": plan["source_sha256"]})
    put(item / "subgroup/summary.json", subgroup)
    audit = {
        "status": "RUNTIME_GEOMETRY_AUDIT_COMPLETE",
        "evaluation_split": "val",
        "evaluation_episode_count": 2,
        "checkpoint_sha256": checkpoint_sha,
        "corpus_fingerprint_sha256": data["corpus_fingerprint_sha256"],
        "dataset_fingerprint_sha256": data["split_fingerprint_sha256"]["val"],
        "training_dataset_fingerprint_sha256": data["split_fingerprint_sha256"]["train"],
        "model_config_sha256": plan["model_config_canonical_sha256"],
        "vehicle_control_approved": False,
        "gate": {"source": "portable_e2e.runtime_geometry_gate.v8", "threshold_overrides": False},
        "geometry": geometry(geometry_count),
    }
    if sampling is not None:
        audit.update({"training_sampling_plan_sha256": m._canonical_sha256(sampling),
                      "training_sampling_policy": "cohort_replay"})
    put(item / "gate_v8.json", audit)
    return {
        "metrics": {"combined_val": combined,
                    **{spec["label"]: metric_values(spec["label"], label) for spec in m.EPISODES}},
        "geometry": {name: {"pass_count": geometry_count} for name in
                     ("selected", "any_candidate", "all_candidates")},
    }


def stage_receipts(root: Path, plan: dict, arms: tuple[str, ...]) -> list[dict]:
    rows = []
    plan_sha = sha(root / "plan.json")
    for arm in arms:
        for stage, relative in m.STAGE_PATHS.items():
            path = root / arm / relative
            if not path.exists():
                put(path, {"fixture": True})
            row = {
                "arm": arm,
                "stage": stage,
                "status": "COMPLETE",
                "plan_sha256": plan_sha,
                "path": f"{arm}/{relative}",
                "sha256": sha(path),
            }
            checkpoint = root / arm / "training/checkpoints/latest.pt"
            if checkpoint.exists():
                row["checkpoint_sha256"] = sha(checkpoint)
            history = root / arm / "training/metrics.jsonl"
            if stage == "train" and history.exists():
                row["history_sha256"] = sha(history)
            put(root / arm / f"{stage}_completed.json", row)
            rows.append(row)
    return rows


def denial_summary(schema: str, plan_sha: str, stages: list[dict]) -> dict:
    result = {
        "schema": schema,
        "status": "COMPLETE_NOT_PROMOTED",
        "plan_sha256": plan_sha,
        "stages": stages,
        "resume": False,
        "automatic_retry": False,
        "test_read_or_use": False,
        "model_export": False,
        "automatic_promotion": False,
        "training_data_approved": False,
        "vehicle_control_approved": False,
        "closed_loop_claim": False,
        "all_map_claim": False,
        "deployment_approved": False,
    }
    if schema == m.REFERENCE_SUMMARY_SCHEMA:
        result.update({"source_and_views_postcheck_pass": True,
                       "arms": {m.REFERENCE_ARM: "COMPLETE", m.BASE_DATA_ARM: "COMPLETE"}})
    else:
        result.update({"source_dataset_reference_postcheck_pass": True, "arm": "COMPLETE"})
    return result


def make_pair(tmp_path: Path, monkeypatch) -> tuple[Path, Path]:
    reference, experiment = tmp_path / "reference", tmp_path / "experiment"
    reference.mkdir()
    experiment.mkdir()
    archived = {
        "portable_e2e/train.py": TRAIN_OVERLAY_BYTES,
        "portable_e2e/evaluate.py": EVALUATE_OVERLAY_BYTES,
        m.EXPECTED_ADDED_SOURCE_PATHS[0]: ADDED_HELPER_BYTES,
        "run_moving_turn_replay_seed_pilot.py": RUNNER_BYTES,
        "evaluate_portable_moving_turn_replay_subgroups.py": EVALUATOR_BYTES,
        "profile_portable_training.py": OWNER_BYTES,
    }
    for name, payload in archived.items():
        path = experiment / "source" / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)
    (experiment / "view_report.json").write_bytes(VIEW_BYTES)
    monkeypatch.setattr(m, "EXPECTED_RUNNER_SHA256", byte_sha(RUNNER_BYTES))
    monkeypatch.setattr(m, "EXPECTED_EVALUATOR_SHA256", byte_sha(EVALUATOR_BYTES))
    monkeypatch.setattr(m, "EXPECTED_OVERLAY_SHA256", {
        "portable_e2e/train.py": byte_sha(TRAIN_OVERLAY_BYTES),
        "portable_e2e/evaluate.py": byte_sha(EVALUATE_OVERLAY_BYTES),
    })
    monkeypatch.setattr(m, "EXPECTED_ADDED_SOURCE_SHA256", {
        m.EXPECTED_ADDED_SOURCE_PATHS[0]: byte_sha(ADDED_HELPER_BYTES),
    })
    c_data = data_spec("old", 1147)
    d_data = data_spec("augmented", 1475)
    c_config, e_config = configs()
    sampling = sampling_plan(monkeypatch)
    schedule = execution_schedule(sampling)
    reference_plan = common_plan(m.REFERENCE_PLAN_SCHEMA, m.REFERENCE_CAMPAIGN)
    reference_plan.update({
        "arms": {
            m.REFERENCE_ARM: {"dataset": c_data, "expected_train_state": m.EXPECTED_REFERENCE_STATE,
                              "train_config": c_config},
            m.BASE_DATA_ARM: {"dataset": d_data, "expected_train_state": {}, "train_config": c_config},
        },
        "validation": {"episodes": list(m.EPISODES), "count": 665,
                       "fingerprint_sha256": "d" * 64, "tokens_sha256": "0" * 64},
    })
    put(reference / "plan.json", reference_plan)
    c_result = write_arm(reference, reference_plan, m.REFERENCE_ARM, "C", c_data, c_config,
                         m.EXPECTED_REFERENCE_STATE, None)
    reference_stages = stage_receipts(reference, reference_plan,
                                      (m.REFERENCE_ARM, m.BASE_DATA_ARM))
    reference_summary = denial_summary(
        m.REFERENCE_SUMMARY_SCHEMA, sha(reference / "plan.json"), reference_stages)
    reference_summary["comparison"] = {}
    put(reference / "summary.json", reference_summary)
    experiment_plan = common_plan(m.EXPERIMENT_PLAN_SCHEMA, m.EXPERIMENT_CAMPAIGN)
    thresholds = {
        label: {key: c_result["metrics"][label][key] * multiplier
                for key, multiplier in metrics.items()}
        for label, metrics in m.EXPECTED_GATE_MULTIPLIERS.items()
    }
    reference_metrics = {
        label: {key: c_result["metrics"][label][key]
                for key in m.EXPECTED_GATE_MULTIPLIERS[label]}
        for label in m.EXPECTED_GATE_MULTIPLIERS
    }
    monkeypatch.setattr(m, "EXPECTED_REFERENCE_METRICS", reference_metrics)
    monkeypatch.setattr(m, "EXPECTED_ABSOLUTE_MAXIMUM_E", thresholds)
    experiment_plan.update({
        "arm": {
            "label": m.EXPERIMENT_ARM,
            "dataset_role": m.BASE_DATA_ARM,
            "dataset": d_data,
            "expected_train_state": m.EXPECTED_EXPERIMENT_STATE,
            "expected_sampling_stream_samples_seen": {"base": 29500, "replay": 6540},
            "train_config": e_config,
            "sampling_plan": sampling,
            "sampling_plan_sha256": m._canonical_sha256(sampling),
            "execution_schedule": schedule,
        },
        "train_config": e_config,
        "validation": reference_plan["validation"],
        "reference_C": {
            "root": "runs/diagnostics/reference",
            "campaign_id": m.REFERENCE_CAMPAIGN,
            "arm": m.REFERENCE_ARM,
            "files": {
                "plan": {"path": "plan.json", "sha256": sha(reference / "plan.json")},
                "summary": {"path": "summary.json", "sha256": sha(reference / "summary.json")},
                "checkpoint": {"path": f"{m.REFERENCE_ARM}/training/checkpoints/latest.pt",
                               "sha256": sha(reference / m.REFERENCE_ARM /
                                             "training/checkpoints/latest.pt")},
                "training_run": {"path": f"{m.REFERENCE_ARM}/training/run.json",
                                 "sha256": sha(reference / m.REFERENCE_ARM / "training/run.json")},
                "subgroup": {"path": f"{m.REFERENCE_ARM}/subgroup/summary.json",
                             "sha256": sha(reference / m.REFERENCE_ARM / "subgroup/summary.json")},
                "gate": {"path": f"{m.REFERENCE_ARM}/gate_v8.json",
                         "sha256": sha(reference / m.REFERENCE_ARM / "gate_v8.json")},
            },
            "metrics": reference_metrics,
            "geometry_pass_counts": m.EXPECTED_GEOMETRY_MINIMUM,
        },
        "decision": {
            "single_seed_exploratory_only": True,
            "final_checkpoint_only": True,
            "fresh_initialization": True,
            "no_early_stopping_or_checkpoint_selection": True,
            "reference_arm": "C_old_train_7380_steps",
            "experimental_arm": m.EXPERIMENT_ARM,
            "maximum_E": thresholds,
            "minimum_geometry_pass_counts": m.EXPECTED_GEOMETRY_MINIMUM,
            "diagnostic_hypothesis": "fixture",
            "unseen_confirmation_seeds": [20260907, 20260908, 20260909],
            "confirmation_runs_allowed_only_after_seed20260906_pass": True,
            "adoption_allowed": False,
        },
    })
    put(experiment / "plan.json", experiment_plan)
    e_result = write_arm(experiment, experiment_plan, m.EXPERIMENT_ARM, "E", d_data, e_config,
                         m.EXPECTED_EXPERIMENT_STATE, sampling)
    experiment_stages = stage_receipts(experiment, experiment_plan, (m.EXPERIMENT_ARM,))
    experiment_summary = denial_summary(
        m.EXPERIMENT_SUMMARY_SCHEMA, sha(experiment / "plan.json"), experiment_stages)
    experiment_summary["diagnostic_gate"] = m._gate(c_result, e_result)
    put(experiment / "summary.json", experiment_summary)
    return reference, experiment


def refresh_stage(root: Path, arm: str, stage: str) -> None:
    summary = json.loads((root / "summary.json").read_text())
    relative = m.STAGE_PATHS[stage]
    for row in summary["stages"]:
        if row["arm"] == arm and row["stage"] == stage:
            row["sha256"] = sha(root / arm / relative)
            put(root / arm / f"{stage}_completed.json", row)
    put(root / "summary.json", summary)


def refresh_plan_binding(root: Path) -> None:
    plan_sha = sha(root / "plan.json")
    summary = json.loads((root / "summary.json").read_text())
    summary["plan_sha256"] = plan_sha
    subgroup_path = root / m.EXPERIMENT_ARM / "subgroup/summary.json"
    subgroup = json.loads(subgroup_path.read_text())
    subgroup["plan_sha256"] = plan_sha
    put(subgroup_path, subgroup)
    for row in summary["stages"]:
        row["plan_sha256"] = plan_sha
        if row["stage"] == "subgroup":
            row["sha256"] = sha(subgroup_path)
        put(root / row["arm"] / f"{row['stage']}_completed.json", row)
    put(root / "summary.json", summary)


def test_complete_pass_recomputes_gate_and_exact_exposure_accounting(tmp_path, monkeypatch):
    reference, experiment = make_pair(tmp_path, monkeypatch)
    result = m.summarize(reference_root=reference, experiment_root=experiment)
    assert result["schema"] == m.SCHEMA
    assert result["status"] == "COMPLETE_NOT_PROMOTED"
    assert result["preregistered_C_E_gate"]["result"]["pass"] is True
    exposure = result["cohort_and_exposure_verification"]
    assert exposure["history_line_count"] == 9020
    assert exposure["history_sampling_stream_samples_seen"] == {"base": 29500, "replay": 6540}
    assert exposure["exact_exposure_accounting_pass"] is True
    assert exposure["execution_schedule"]["row_count"] == 36040
    assert exposure["execution_schedule"]["canonical_sha256_recomputed"] is True
    assert exposure["execution_schedule"]["epoch_occurrence_bijection_verified"] is True
    assert result["validation"]["identical_across_C_and_E"] is True
    assert all(value is False for key, value in result["scope"].items()
               if key not in ("single_seed_only", "three_seed_confirmation_required"))


def test_metric_gate_failure_is_reported_without_promotion(tmp_path, monkeypatch):
    reference, experiment = make_pair(tmp_path, monkeypatch)
    subgroup_path = experiment / m.EXPERIMENT_ARM / "subgroup/summary.json"
    subgroup = json.loads(subgroup_path.read_text())
    subgroup["episodes"]["town03_right_turn_dev"]["metrics"]["ade_6p4s_m"] = 11.0
    put(subgroup_path, subgroup)
    report_path = experiment / m.EXPERIMENT_ARM / "subgroup/episodes/town03_right_turn_dev/metrics.json"
    report = json.loads(report_path.read_text())
    report["metrics"]["ade_6p4s_m"] = 11.0
    put(report_path, report)
    subgroup = json.loads(subgroup_path.read_text())
    subgroup["episodes"]["town03_right_turn_dev"]["report_sha256"] = sha(report_path)
    put(subgroup_path, subgroup)
    evaluation_path = experiment / m.EXPERIMENT_ARM / "evaluation/metrics.json"
    evaluation = json.loads(evaluation_path.read_text())
    evaluation["metrics"]["ade_6p4s_m"] = (11.0 * 337 + 18.0 * 328) / 665
    put(evaluation_path, evaluation)
    refresh_stage(experiment, m.EXPERIMENT_ARM, "evaluate")
    refresh_stage(experiment, m.EXPERIMENT_ARM, "subgroup")
    summary = json.loads((experiment / "summary.json").read_text())
    summary["diagnostic_gate"]["episode_metric_checks"][
        "town03_right_turn_dev"]["ade_6p4s_m"].update({"observed_E": 11.0, "pass": False})
    summary["diagnostic_gate"]["pass"] = False
    summary["diagnostic_gate"]["status"] = "FAIL_SINGLE_SEED_DIAGNOSTIC"
    summary["diagnostic_gate"]["next_action"] = (
        "do_not_confirm_or_promote_revise_training_data_or_sampling")
    put(experiment / "summary.json", summary)
    result = m.summarize(reference_root=reference, experiment_root=experiment)
    assert result["preregistered_C_E_gate"]["result"]["status"] == "FAIL_SINGLE_SEED_DIAGNOSTIC"
    assert result["scope"]["model_promotion"] is False


def test_rejects_shared_validation_fingerprint_mismatch(tmp_path, monkeypatch):
    reference, experiment = make_pair(tmp_path, monkeypatch)
    plan = json.loads((experiment / "plan.json").read_text())
    plan["validation"]["fingerprint_sha256"] = "a" * 64
    put(experiment / "plan.json", plan)
    refresh_plan_binding(experiment)
    with pytest.raises(m.SummaryError, match="validation declaration"):
        m.summarize(reference_root=reference, experiment_root=experiment)


def test_rejects_tampered_replay_occurrence_even_if_declared_hash_is_refreshed(tmp_path, monkeypatch):
    reference, experiment = make_pair(tmp_path, monkeypatch)
    plan_path = experiment / "plan.json"
    plan = json.loads(plan_path.read_text())
    sampling = plan["arm"]["sampling_plan"]
    sampling["cohort_replay"]["replay_occurrences"][0]["sample_token"] = "wrong-token"
    sampling["cohort_replay"]["replay_occurrences_sha256"] = m._canonical_sha256(
        sampling["cohort_replay"]["replay_occurrences"])
    plan["arm"]["sampling_plan_sha256"] = m._canonical_sha256(sampling)
    put(plan_path, plan)
    refresh_plan_binding(experiment)
    with pytest.raises(m.SummaryError, match="replay occurrence differs"):
        m.summarize(reference_root=reference, experiment_root=experiment)


def test_rejects_history_with_wrong_per_epoch_replay_exposure(tmp_path, monkeypatch):
    reference, experiment = make_pair(tmp_path, monkeypatch)
    history = experiment / m.EXPERIMENT_ARM / "training/metrics.jsonl"
    lines = history.read_text().splitlines()
    row = json.loads(lines[0])
    row["batch_sampling_stream_counts"] = {"base": 3, "replay": 1}
    row["sampling_stream_samples_seen"] = {"base": 3, "replay": 1}
    lines[0] = json.dumps(row, sort_keys=True, separators=(",", ":"))
    history.write_text("\n".join(lines) + "\n")
    receipt = json.loads((experiment / m.EXPERIMENT_ARM / "train_completed.json").read_text())
    receipt["history_sha256"] = sha(history)
    put(experiment / m.EXPERIMENT_ARM / "train_completed.json", receipt)
    summary = json.loads((experiment / "summary.json").read_text())
    for stage in summary["stages"]:
        if stage["stage"] == "train":
            stage["history_sha256"] = sha(history)
    put(experiment / "summary.json", summary)
    with pytest.raises(m.SummaryError, match="stream counts"):
        m.summarize(reference_root=reference, experiment_root=experiment)


def test_rejects_tampered_full_execution_schedule_even_with_refreshed_hash(tmp_path, monkeypatch):
    reference, experiment = make_pair(tmp_path, monkeypatch)
    plan_path = experiment / "plan.json"
    plan = json.loads(plan_path.read_text())
    schedule = plan["arm"]["execution_schedule"]
    schedule["rows"][0]["dataset_index"] = 1
    schedule["sha256"] = m._canonical_sha256(schedule["rows"])
    put(plan_path, plan)
    refresh_plan_binding(experiment)
    with pytest.raises(m.SummaryError, match="occurrence binding"):
        m.summarize(reference_root=reference, experiment_root=experiment)


def test_rejects_any_test_or_promotion_authorization(tmp_path, monkeypatch):
    reference, experiment = make_pair(tmp_path, monkeypatch)
    plan_path = experiment / "plan.json"
    plan = json.loads(plan_path.read_text())
    plan["test_read_or_use"] = True
    put(plan_path, plan)
    refresh_plan_binding(experiment)
    with pytest.raises(m.SummaryError, match="test_read_or_use must be false"):
        m.summarize(reference_root=reference, experiment_root=experiment)


def test_rejects_runner_gate_disagreement_and_output_replacement(tmp_path, monkeypatch):
    reference, experiment = make_pair(tmp_path, monkeypatch)
    summary = json.loads((experiment / "summary.json").read_text())
    summary["diagnostic_gate"]["pass"] = False
    put(experiment / "summary.json", summary)
    with pytest.raises(m.SummaryError, match="recomputed E/C gate"):
        m.summarize(reference_root=reference, experiment_root=experiment)
    output = tmp_path / "already.json"
    output.write_text("do not replace\n")
    with pytest.raises(m.SummaryError, match="must be fresh"):
        m.write_new(output, {"value": 1})
    assert output.read_text() == "do not replace\n"
