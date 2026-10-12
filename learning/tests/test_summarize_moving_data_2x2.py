# HH_260906 - Locate the repository independently of the relocated test directory.
"""HH_260906 - CPU fixtures verify the immutable moving-data A/B/C/D summarizer."""
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path

import pytest

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path


ROOT = next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())
PATH = _hh_source_path(ROOT, "scripts/e2e/summarize_moving_data_2x2.py")
SPEC = importlib.util.spec_from_file_location("summarize_moving_data_2x2_test", PATH)
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)


def put(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(m._encoded(value))


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def dataset(physical: str) -> dict:
    old = physical == m.PHYSICAL_ARMS[0]
    digit = "1" if old else "2"
    return {
        "path": f"datasets/prepared/{physical}",
        "manifest_sha256": digit * 64,
        "corpus_fingerprint_sha256": ("3" if old else "4") * 64,
        "split_fingerprint_sha256": {"train": ("5" if old else "6") * 64, "val": "7" * 64},
        "split_loader_sha256": {"train": ("8" if old else "9") * 64, "val": "a" * 64},
        "split_examples_sha256": {"train": ("b" if old else "c") * 64, "val": "d" * 64},
        "split_tokens_sha256": {"train": ("e" if old else "f") * 64, "val": "0" * 64},
        "counts": {"train": 1147 if old else 1475, "val": 665},
        "episode_ids": {
            "train": [f"train-{physical}"],
            "val": [spec["episode_id"] for spec in m.EPISODES],
        },
        "tree": {"manifest_sha256": "1" * 64, "file_count": 0,
                 "directory_count": 0, "total_size_bytes": 0, "files": []},
    }


def metric_values(label: str, episode: str) -> dict[str, float]:
    values = {
        "A": {"town03_right_turn_dev": (10.0, 20.0, 4.0, 3.0),
              "town03_straight_dev": (20.0, 40.0, 8.0, 6.0)},
        "B": {"town03_right_turn_dev": (11.0, 23.0, 5.0, 4.0),
              "town03_straight_dev": (10.0, 20.0, 4.0, 3.0)},
        "C": {"town03_right_turn_dev": (10.0, 20.0, 4.0, 3.0),
              "town03_straight_dev": (20.0, 40.0, 8.0, 6.0)},
        "D": {"town03_right_turn_dev": (9.5, 19.0, 4.1, 2.8),
              "town03_straight_dev": (16.0, 32.0, 7.0, 5.0)},
    }
    ade, fde, speed, loss = values[label][episode]
    return {"ade_6p4s_m": ade, "fde_6p4s_m": fde,
            "speed_mae_6p4s_mps": speed, "loss": loss}


def geometry(label: str) -> dict:
    counts = {
        "A": (660, 664, 600), "B": (659, 664, 590),
        "C": (660, 663, 600), "D": (661, 664, 601),
    }[label]
    return {
        "sample_count": 665,
        "selected_result": {"sample_count": 665, "geometry_pass_count": counts[0]},
        "any_candidate_geometry_pass_count": counts[1],
        "all_candidates_geometry_pass_count": counts[2],
    }


def history_line(physical: str, step: int, final_samples: int) -> bytes:
    low_final = 22940 if physical == m.PHYSICAL_ARMS[0] else 22945
    samples = final_samples if step > 5740 else (low_final if step == 5740 else step * 4)
    return (json.dumps({"global_step": step, "loss": step / 1000.0, "samples_seen": samples},
                       sort_keys=True, separators=(",", ":")) + "\n").encode()


def make_campaign(root: Path, budget: str) -> tuple[dict, dict]:
    root.mkdir(parents=True)
    labels = m.LOW_LABELS if budget == "low" else m.HIGH_LABELS
    common = {
        "schema": m.PLAN_SCHEMA,
        "campaign_id": m.CAMPAIGNS[budget],
        "source_commit": "1" * 40,
        "source_sha256": {"source.py": "2" * 64},
        "evaluator_sha256": "3" * 64,
        "runner_sha256": ("4" if budget == "low" else "5") * 64,
        "owner": {"path": "owner.py", "sha256": "6" * 64},
        "gpu": {"physical_index": 0, "uuid": "GPU-fixture"},
        "model_config": "model.json",
        "model_config_sha256": "7" * 64,
        "model_config_canonical_sha256": "8" * 64,
        "model_id": "fixture.model",
        "loss_config": {"xy_weight": 1.0},
        "view_report_sha256": "9" * 64,
        "output": f"runs/{budget}",
        "declared_at_utc": "2026-09-21T00:00:00+00:00",
        "stage_order": ["train", "evaluate", "audit", "subgroup"],
        "stage_timeout_seconds": {"train": 1, "evaluate": 1, "audit": 1, "subgroup": 1},
        "whole_timeout_seconds": 10,
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
    plan = dict(common)
    plan["train_config"] = ({"seed": 20260906, "max_steps": 5740} if budget == "low"
                            else {"seed": 20260906})
    plan["arms"] = {}
    for physical in m.PHYSICAL_ARMS:
        label = labels[physical]
        arm = {"dataset": dataset(physical), "expected_train_state": m.EXPECTED_STATES[label]}
        if budget == "high":
            arm["train_config"] = {"seed": 20260906, "max_steps": 7380}
        plan["arms"][physical] = arm
    plan["validation"] = {
        "episodes": list(m.EPISODES), "count": 665,
        "fingerprint_sha256": "d" * 64, "tokens_sha256": "0" * 64,
    }
    plan["decision"] = ({"adoption_allowed": False} if budget == "low" else {
        "experimental_arm_labels": m.EXPERIMENTAL_LABELS,
        "maximum_D_over_C": m.EXPECTED_CD_THRESHOLDS,
        "optimizer_steps_per_arm": 7380,
        "three_seed_confirmation_required": True,
        "adoption_allowed": False,
    })
    put(root / "plan.json", plan)
    plan_sha = sha(root / "plan.json")
    stages = []
    arm_results = {}
    for physical in m.PHYSICAL_ARMS:
        label = labels[physical]
        state, data = m.EXPECTED_STATES[label], plan["arms"][physical]["dataset"]
        item = root / physical
        checkpoint = item / "training/checkpoints/latest.pt"
        checkpoint.parent.mkdir(parents=True)
        checkpoint.write_bytes(f"checkpoint-{label}".encode())
        checkpoint_sha = sha(checkpoint)
        history = item / "training/metrics.jsonl"
        with history.open("wb") as stream:
            for step in range(1, state["global_step"] + 1):
                stream.write(history_line(physical, step, state["samples_seen"]))
        train = {
            "status": "TRAINING_TARGET_REACHED", "state": state,
            "dataset_size": data["counts"]["train"], "device": "cuda:0",
            "dataset_fingerprint_sha256": data["split_fingerprint_sha256"]["train"],
            "corpus_fingerprint_sha256": data["corpus_fingerprint_sha256"],
            "train_config": plan["arms"][physical].get("train_config", plan["train_config"]),
            "loss_config": plan["loss_config"],
        }
        put(item / "training/run.json", train)
        episode_rows, combined = {}, {}
        for spec in m.EPISODES:
            values = metric_values(label, spec["label"])
            report = {"metrics": values, "sample_count": spec["count"]}
            report_path = item / "subgroup/episodes" / spec["label"] / "metrics.json"
            put(report_path, report)
            episode_rows[spec["label"]] = {
                "episode_id": spec["episode_id"], "count": spec["count"],
                "metrics": values, "metric_counts": {key: spec["count"] for key in values},
                "report_path": f"episodes/{spec['label']}/metrics.json",
                "report_sha256": sha(report_path),
            }
        for key in next(iter(episode_rows.values()))["metrics"]:
            combined[key] = sum(row["metrics"][key] * row["count"] for row in episode_rows.values()) / 665
        evaluation = {
            "status": "OPEN_LOOP_EVALUATION_COMPLETE", "evaluation_split": "val",
            "sample_count": 665, "evaluation_episode_count": 2,
            "checkpoint_sha256": checkpoint_sha,
            "corpus_fingerprint_sha256": data["corpus_fingerprint_sha256"],
            "dataset_fingerprint_sha256": data["split_fingerprint_sha256"]["val"],
            "training_dataset_fingerprint_sha256": data["split_fingerprint_sha256"]["train"],
            "model_config_sha256": plan["model_config_canonical_sha256"],
            "loss_config": plan["loss_config"],
            "vehicle_control_approved": False,
            "metrics": combined, "metric_counts": {key: 665 for key in combined},
        }
        put(item / "evaluation/metrics.json", evaluation)
        subgroup = {
            "schema": m.SUBGROUP_SCHEMA, "status": "COMPLETE_NOT_PROMOTED", "arm": physical,
            "plan_sha256": plan_sha, "source_commit": plan["source_commit"],
            "source_sha256": plan["source_sha256"], "evaluator_sha256": plan["evaluator_sha256"],
            "combined_val_count": 665, "checkpoint_sha256": checkpoint_sha,
            "corpus_fingerprint_sha256": data["corpus_fingerprint_sha256"],
            "test_read_or_use": False, "closed_loop_evaluation": False,
            "automatic_promotion": False, "vehicle_control_approved": False,
            "training_executed_by_evaluator": False, "model_exported": False,
            "dataset": {
                "combined_val_example_fingerprint_sha256": data["split_examples_sha256"]["val"],
                "combined_val_fingerprint_sha256": data["split_fingerprint_sha256"]["val"],
                "train_fingerprint_sha256": data["split_fingerprint_sha256"]["train"],
                "manifest_sha256": data["manifest_sha256"], "counts": data["counts"],
            },
            "checkpoint": {
                "sha256": checkpoint_sha,
                "training_dataset_fingerprint_sha256": data["split_fingerprint_sha256"]["train"],
                "corpus_fingerprint_sha256": data["corpus_fingerprint_sha256"],
                "model_config_sha256": plan["model_config_canonical_sha256"],
            },
            "episodes": episode_rows,
        }
        put(item / "subgroup/summary.json", subgroup)
        audit = {
            "status": "RUNTIME_GEOMETRY_AUDIT_COMPLETE", "evaluation_split": "val",
            "evaluation_episode_count": 2, "checkpoint_sha256": checkpoint_sha,
            "corpus_fingerprint_sha256": data["corpus_fingerprint_sha256"],
            "dataset_fingerprint_sha256": data["split_fingerprint_sha256"]["val"],
            "training_dataset_fingerprint_sha256": data["split_fingerprint_sha256"]["train"],
            "model_config_sha256": plan["model_config_canonical_sha256"],
            "gate": {"source": "portable_e2e.runtime_geometry_gate.v8", "threshold_overrides": False},
            "vehicle_control_approved": False, "geometry": geometry(label),
        }
        put(item / "gate_v8.json", audit)
        stage_paths = {
            "train": "training/run.json", "evaluate": "evaluation/metrics.json",
            "audit": "gate_v8.json", "subgroup": "subgroup/summary.json",
        }
        for stage, relative in stage_paths.items():
            row = {"arm": physical, "stage": stage, "status": "COMPLETE",
                   "plan_sha256": plan_sha, "path": f"{physical}/{relative}",
                   "sha256": sha(item / relative), "checkpoint_sha256": checkpoint_sha}
            if stage == "train":
                row["history_sha256"] = sha(history)
            put(item / f"{stage}_completed.json", row)
            stages.append(row)
        arm_results[label] = {
            "metrics": {spec["label"]: metric_values(label, spec["label"]) for spec in m.EPISODES},
            "geometry": {kind: {"pass_count": value} for kind, value in zip(
                ("selected", "any_candidate", "all_candidates"),
                (geometry(label)["selected_result"]["geometry_pass_count"],
                 geometry(label)["any_candidate_geometry_pass_count"],
                 geometry(label)["all_candidates_geometry_pass_count"]))},
        }
    first, second = (("A", "B") if budget == "low" else ("C", "D"))
    delta_name = "metric_delta_B_minus_A_by_episode" if budget == "low" else "metric_delta_D_minus_C_by_episode"
    comparison = {
        "arms": {physical: {"subgroup_summary_sha256": sha(root / physical / "subgroup/summary.json")}
                 for physical in m.PHYSICAL_ARMS},
        delta_name: {
            spec["label"]: {key: arm_results[second]["metrics"][spec["label"]][key] -
                                 arm_results[first]["metrics"][spec["label"]][key]
                            for key in arm_results[first]["metrics"][spec["label"]]}
            for spec in m.EPISODES
        },
    }
    if budget == "high":
        comparison["experimental_arm_labels"] = m.EXPERIMENTAL_LABELS
        comparison["diagnostic_gate"] = m._cd_gate({"arms": arm_results})
    summary = {
        "schema": m.SUMMARY_SCHEMAS[budget], "status": "COMPLETE_NOT_PROMOTED",
        "plan_sha256": plan_sha, "source_and_views_postcheck_pass": True,
        "arms": {arm: "COMPLETE" for arm in m.PHYSICAL_ARMS}, "stages": stages,
        "comparison": comparison, "test_read_or_use": False, "automatic_promotion": False,
        "vehicle_control_approved": False, "deployment_approved": False,
        "resume": False, "automatic_retry": False, "training_data_approved": False,
        "closed_loop_claim": False, "all_map_claim": False, "model_export": False,
    }
    put(root / "summary.json", summary)
    return plan, summary


def make_pair(tmp_path: Path) -> tuple[Path, Path]:
    low, high = tmp_path / "low", tmp_path / "high"
    make_campaign(low, "low")
    make_campaign(high, "high")
    return low, high


def refresh_history_receipt(root: Path, physical: str) -> None:
    summary = json.loads((root / "summary.json").read_text())
    history_sha = sha(root / physical / "training/metrics.jsonl")
    for row in summary["stages"]:
        if row["arm"] == physical and row["stage"] == "train":
            row["history_sha256"] = history_sha
            put(root / physical / "train_completed.json", row)
    put(root / "summary.json", summary)


def test_complete_four_arm_summary_has_metrics_contrasts_geometry_and_scope(tmp_path):
    low, high = make_pair(tmp_path)
    result = m.summarize(low_root=low, high_root=high)
    assert result["schema"] == m.SCHEMA
    assert result["status"] == "COMPLETE_NOT_PROMOTED"
    assert set(result["arm_provenance"]) == set("ABCD")
    assert result["validation"]["identical_across_all_four_arms"] is True
    row = result["contrasts"]["by_slice"]["town03_straight_dev"]["ade_6p4s_m"]
    assert row["B_minus_A"] == {"absolute": -10.0, "percent_of_A": -50.0}
    assert row["D_minus_C"] == {"absolute": -4.0, "percent_of_C": -20.0}
    assert row["interaction"]["absolute_D_minus_C_minus_B_minus_A"] == 6.0
    assert result["geometry"]["arms"]["D"]["selected"]["pass_count"] == 661
    assert result["preregistered_C_D_gate"]["runner_summary_exact_match"] is True
    assert result["preregistered_C_D_gate"]["result"]["pass"] is True
    assert all(value is False for key, value in result["scope"].items()
               if key not in ("single_seed_only", "three_seed_confirmation_required"))


def test_histories_are_compared_as_bytes_for_exact_5740_line_prefix(tmp_path):
    low, high = make_pair(tmp_path)
    result = m.summarize(low_root=low, high_root=high)
    for row in result["history_prefix_replay"].values():
        assert row["compared_line_count"] == 5740
        assert row["byte_identical"] is True
        assert row["first_mismatch"] is None
        assert row["historical_prefix_sha256"] == row["higher_budget_prefix_sha256"]


def test_history_byte_mismatch_is_reported_without_hiding_first_record(tmp_path):
    low, high = make_pair(tmp_path)
    path = high / m.PHYSICAL_ARMS[0] / "training/metrics.jsonl"
    lines = path.read_bytes().splitlines(keepends=True)
    record = json.loads(lines[1]); record["loss"] = 999.0
    lines[1] = (json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n").encode()
    path.write_bytes(b"".join(lines))
    refresh_history_receipt(high, m.PHYSICAL_ARMS[0])
    result = m.summarize(low_root=low, high_root=high)
    row = result["history_prefix_replay"]["A_vs_C_same_dataset_seed_prefix"]
    assert row["byte_identical"] is False
    assert row["first_mismatch"]["line_number"] == 2
    assert row["first_mismatch"]["historical_record"]["loss"] != 999.0
    assert row["first_mismatch"]["higher_budget_record"]["loss"] == 999.0


def test_rejects_runner_gate_that_does_not_match_independent_recalculation(tmp_path):
    low, high = make_pair(tmp_path)
    summary = json.loads((high / "summary.json").read_text())
    summary["comparison"]["diagnostic_gate"]["pass"] = False
    put(high / "summary.json", summary)
    with pytest.raises(m.SummaryError, match="recomputed C/D gate"):
        m.summarize(low_root=low, high_root=high)


def test_rejects_evaluation_provenance_mutation(tmp_path):
    low, high = make_pair(tmp_path)
    path = high / m.PHYSICAL_ARMS[1] / "evaluation/metrics.json"
    report = json.loads(path.read_text()); report["dataset_fingerprint_sha256"] = "f" * 64
    put(path, report)
    # HH_260906 - Refresh only transport receipts so the semantic fingerprint check is reached.
    summary = json.loads((high / "summary.json").read_text())
    for row in summary["stages"]:
        if row["arm"] == m.PHYSICAL_ARMS[1] and row["stage"] == "evaluate":
            row["sha256"] = sha(path)
            put(high / m.PHYSICAL_ARMS[1] / "evaluate_completed.json", row)
    put(high / "summary.json", summary)
    with pytest.raises(m.SummaryError, match="evaluation provenance"):
        m.summarize(low_root=low, high_root=high)


def test_create_only_cli_writes_strict_json_and_refuses_replacement(tmp_path):
    low, high = make_pair(tmp_path)
    output = tmp_path / "result.json"
    assert m.main(["--low-root", str(low), "--high-root", str(high),
                   "--output-json", str(output)]) == 0
    parsed = json.loads(output.read_text(), parse_constant=lambda value: pytest.fail(value))
    assert parsed["schema"] == m.SCHEMA
    with pytest.raises(m.SummaryError, match="fresh"):
        m.main(["--low-root", str(low), "--high-root", str(high),
                "--output-json", str(output)])


def test_strict_json_rejects_duplicate_keys(tmp_path):
    low, high = make_pair(tmp_path)
    (low / "plan.json").write_text('{"schema":"one","schema":"two"}\n')
    with pytest.raises(m.SummaryError, match="duplicate JSON key"):
        m.summarize(low_root=low, high_root=high)


@pytest.mark.parametrize("field", ["test_read_or_use", "closed_loop_claim", "automatic_promotion"])
def test_scope_claim_cannot_be_enabled(tmp_path, field):
    low, high = make_pair(tmp_path)
    plan = json.loads((low / "plan.json").read_text()); plan[field] = True
    put(low / "plan.json", plan)
    with pytest.raises(m.SummaryError):
        m.summarize(low_root=low, high_root=high)
