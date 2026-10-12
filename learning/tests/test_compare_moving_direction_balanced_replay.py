# HH_260906 - Locate the repository independently of the relocated test directory.
"""HH_260906 - Seal the independent C, D, E, and F fixed-DEV comparator."""
from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path


ROOT = next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())
SCRIPT = _hh_source_path(ROOT, "scripts/e2e/compare_moving_direction_balanced_replay.py")
SPEC = importlib.util.spec_from_file_location("compare_moving_direction_balanced_replay_test", SCRIPT)
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)


def put(path: Path, value: dict) -> None:
    """HH_260906 - Write one canonical fixture JSON file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def combined(right: dict[str, float], straight: dict[str, float]) -> dict[str, float]:
    """HH_260906 - Reconstruct a compatible combined fixture metric map."""
    return {name: (right[name] * 337 + straight[name] * 328) / 665 for name in m.METRICS}


def write_model(root: Path, label: str, metrics: dict[str, dict[str, float]],
                checkpoint: str, geometry: dict[str, int] | None = None) -> Path:
    """HH_260906 - Build the minimum complete immutable evidence triplet."""
    arm = root / (m.F_ARM_NAME if label == "F" else label)
    loss_config = {
        "xy_weight": 1.0,
        "final_displacement_weight": 0.5,
        "speed_weight": 0.2,
        "yaw_weight": 0.1,
        "kinematic_speed_weight": 0.05,
        "candidate_score_weight": 0.1,
        "candidate_regret_weight": 0.1,
    }
    evaluation = {
        "evaluation_id": "portable_e2e.open_loop_evaluation.v1",
        "status": "OPEN_LOOP_EVALUATION_COMPLETE",
        "evaluation_split": "val",
        "sample_count": 665,
        "evaluation_episode_count": 2,
        "vehicle_control_approved": False,
        "checkpoint_sha256": checkpoint,
        "dataset_fingerprint_sha256": m.SHARED_VAL_FINGERPRINT,
        "model_config_sha256": m.MODEL_CONFIG_SHA256,
        "model_parameter_count": m.MODEL_PARAMETER_COUNT,
        "metric_counts": {name: 665 for name in m.METRICS},
        "metrics": metrics["combined"],
        "loss_config": loss_config,
    }
    if label == "F":
        evaluation.update({
            "training_sampling_policy": "direction_balanced_cohort_replay",
            "training_domain_samples_seen": {"carla": 33120},
            "training_sampling_stream_samples_seen": {"base": 29500, "replay": 3620},
        })
    put(arm / "evaluation/metrics.json", evaluation)
    episodes = {}
    for scope, (episode_label, count, episode_id) in m.EPISODES.items():
        episodes[episode_label] = {
            "count": count,
            "episode_id": episode_id,
            "metric_counts": {name: count for name in m.METRICS},
            "metrics": metrics[scope],
        }
    schema = {
        "C": "portable_e2e.moving_data_ab_subgroup_evaluation.v1",
        "D": "portable_e2e.moving_data_ab_subgroup_evaluation.v1",
        "E": "portable_e2e.moving_turn_replay_subgroup_evaluation.v1",
        "F": "portable_e2e.moving_direction_balanced_replay_subgroup_evaluation.v1",
    }[label]
    subgroup = {
        "schema": schema,
        "status": "COMPLETE_NOT_PROMOTED",
        "checkpoint_sha256": checkpoint,
        "checkpoint": {"sha256": checkpoint, "model_config_sha256": m.MODEL_CONFIG_SHA256},
        "dataset": {
            "combined_val_fingerprint_sha256": m.SHARED_VAL_FINGERPRINT,
            "counts": {"train": 1475, "val": 665},
        },
        "combined_val_count": 665,
        "episodes": episodes,
        **m.SUBGROUP_DENIALS,
    }
    put(arm / "subgroup/summary.json", subgroup)
    counts = geometry or {"selected": 664, "any_candidate": 664, "all_candidates": 664}
    gate = {
        "audit_id": "portable_e2e.runtime_geometry_audit.v8",
        "status": "RUNTIME_GEOMETRY_AUDIT_COMPLETE",
        "evaluation_split": "val",
        "vehicle_control_approved": False,
        "checkpoint_sha256": checkpoint,
        "dataset_fingerprint_sha256": m.SHARED_VAL_FINGERPRINT,
        "model_config_sha256": m.MODEL_CONFIG_SHA256,
        "model_parameter_count": m.MODEL_PARAMETER_COUNT,
        "geometry": {
            "sample_count": 665,
            "selected_result": {"sample_count": 665,
                                "geometry_pass_count": counts["selected"]},
            "any_candidate_geometry_pass_count": counts["any_candidate"],
            "all_candidates_geometry_pass_count": counts["all_candidates"],
        },
    }
    put(arm / "gate_v8.json", gate)
    return arm


def fixture_models(tmp_path: Path, f_metrics: dict[str, dict[str, float]] | None = None,
                   geometry: dict[str, int] | None = None) -> dict[str, Path]:
    """HH_260906 - Create anchored references and one configurable F candidate."""
    roots: dict[str, Path] = {}
    for label in ("C", "D", "E"):
        roots[label] = write_model(tmp_path / f"{label}_campaign", label,
                                   m.REFERENCE_METRICS[label], m.REFERENCE_CHECKPOINTS[label])
    if f_metrics is None:
        right = {"ade_6p4s_m": 2.8, "fde_6p4s_m": 7.5, "speed_mae_6p4s_mps": 1.0}
        straight = {"ade_6p4s_m": 3.5, "fde_6p4s_m": 10.5,
                    "speed_mae_6p4s_mps": 1.6}
        f_metrics = {"right_turn": right, "straight": straight,
                     "combined": combined(right, straight)}
    roots["F_CAMPAIGN"] = tmp_path / "F_campaign"
    roots["F"] = write_model(roots["F_CAMPAIGN"], "F", f_metrics, "f" * 64, geometry)
    return roots


def build(paths: dict[str, Path]) -> dict:
    """HH_260906 - Invoke the comparison with fixture-local evidence paths."""
    return m.build_report(paths["C"], paths["D"], paths["E"], paths["F_CAMPAIGN"],
                          paths["E"] / "subgroup/summary.json",
                          paths["F"] / "subgroup/summary.json",
                          created_at_utc="2026-09-21T00:00:00Z")


def test_pass_report_contains_exact_pareto_and_all_reference_deltas(tmp_path: Path) -> None:
    paths = fixture_models(tmp_path)
    report = build(paths)
    assert report["status"] == "PASS_OPEN_LOOP_DIAGNOSTIC_GATE"
    assert report["strict_pareto_gate"]["passed"] is True
    assert set(report["comparisons"]) == {"F_vs_C", "F_vs_D", "F_vs_E"}
    row = report["comparisons"]["F_vs_C"]["right_turn"]["ade_6p4s_m"]
    assert row["candidate_f"] == 2.8
    assert row["reference"] == m.REFERENCE_METRICS["C"]["right_turn"]["ade_6p4s_m"]
    assert row["absolute_delta_f_minus_reference"] == pytest.approx(2.8 - row["reference"])
    assert row["relative_change_percent"] == pytest.approx(
        (2.8 - row["reference"]) / row["reference"] * 100.0)
    assert report["limitations"]["head_addition_approved"] is False
    assert all(item["passed"] for item in report["strict_pareto_gate"]["geometry"].values())


def test_gate_failure_is_reported_without_suppressing_outputs(tmp_path: Path) -> None:
    right = {"ade_6p4s_m": 3.0, "fde_6p4s_m": 7.5, "speed_mae_6p4s_mps": 1.0}
    straight = {"ade_6p4s_m": 3.5, "fde_6p4s_m": 10.5,
                "speed_mae_6p4s_mps": 1.6}
    paths = fixture_models(tmp_path, {"right_turn": right, "straight": straight,
                                      "combined": combined(right, straight)})
    output_json = tmp_path / "comparison.json"
    output_markdown = tmp_path / "comparison.md"
    rc = m.main([
        "--c-arm-dir", str(paths["C"]),
        "--d-arm-dir", str(paths["D"]),
        "--e-arm-dir", str(paths["E"]),
        "--e-subgroup-summary", str(paths["E"] / "subgroup/summary.json"),
        "--f-arm-dir", str(paths["F_CAMPAIGN"]),
        "--f-subgroup-summary", str(paths["F"] / "subgroup/summary.json"),
        "--output-json", str(output_json),
        "--output-markdown", str(output_markdown),
    ])
    assert rc == 1
    saved = json.loads(output_json.read_text(encoding="utf-8"))
    assert saved["status"] == "FAIL_OPEN_LOOP_DIAGNOSTIC_GATE"
    assert saved["strict_pareto_gate"]["benchmarks"]["right_turn"]["passed"] is False
    assert "No TEST, closed-loop, export, promotion" in output_markdown.read_text(encoding="utf-8")


@pytest.mark.parametrize(
    ("relative", "field", "value", "match"),
    [
        ("subgroup/summary.json", "closed_loop_evaluation", True,
         "closed_loop_evaluation must be False"),
        ("evaluation/metrics.json", "training_sampling_policy", "cohort_replay",
         "sampling policy differs"),
        ("gate_v8.json", "checkpoint_sha256", "0" * 64,
         "checkpoint differs"),
    ],
)
def test_fails_closed_on_f_evidence_drift(tmp_path: Path, relative: str, field: str,
                                          value, match: str) -> None:
    paths = fixture_models(tmp_path)
    target = paths["F"] / relative
    payload = json.loads(target.read_text(encoding="utf-8"))
    payload[field] = value
    put(target, payload)
    with pytest.raises(m.ComparisonError, match=match):
        build(paths)


def test_fails_closed_when_frozen_model_contract_changes(tmp_path: Path) -> None:
    paths = fixture_models(tmp_path)
    for relative in ("evaluation/metrics.json", "gate_v8.json"):
        target = paths["F"] / relative
        payload = json.loads(target.read_text(encoding="utf-8"))
        payload["model_config_sha256"] = "9" * 64
        put(target, payload)
    target = paths["F"] / "subgroup/summary.json"
    payload = json.loads(target.read_text(encoding="utf-8"))
    payload["checkpoint"]["model_config_sha256"] = "9" * 64
    put(target, payload)
    with pytest.raises(m.ComparisonError, match="frozen model config"):
        build(paths)


def test_geometry_below_664_fails_only_the_diagnostic_gate(tmp_path: Path) -> None:
    paths = fixture_models(tmp_path, geometry={"selected": 663,
                                               "any_candidate": 664,
                                               "all_candidates": 663})
    report = build(paths)
    assert report["strict_pareto_gate"]["performance_passed"] is True
    assert report["strict_pareto_gate"]["geometry_passed"] is False
    assert report["strict_pareto_gate"]["geometry"]["selected"]["passed"] is False
    assert report["status"] == "FAIL_OPEN_LOOP_DIAGNOSTIC_GATE"


def test_outputs_are_create_only_and_markdown_covers_every_comparator(tmp_path: Path) -> None:
    report = build(fixture_models(tmp_path))
    output_json = tmp_path / "report.json"
    output_markdown = tmp_path / "report.md"
    m.write_outputs(report, output_json, output_markdown)
    markdown = output_markdown.read_text(encoding="utf-8")
    assert "## F versus C" in markdown
    assert "## F versus D" in markdown
    assert "## F versus E" in markdown
    assert "head-addition" in markdown
    with pytest.raises(m.ComparisonError, match="fresh path"):
        m.write_outputs(report, output_json, tmp_path / "second.md")


def test_duplicate_json_key_is_rejected(tmp_path: Path) -> None:
    paths = fixture_models(tmp_path)
    target = paths["F"] / "evaluation/metrics.json"
    target.write_text('{"status":"one","status":"two"}\n', encoding="utf-8")
    with pytest.raises(m.ComparisonError, match="duplicate JSON key"):
        build(paths)
