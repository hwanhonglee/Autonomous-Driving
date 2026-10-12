#!/usr/bin/env python3
"""HH_260906 - Compare sealed C, D, E, and F fixed-DEV open-loop evidence."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
from typing import Any


SCHEMA = "portable_e2e.moving_direction_balanced_replay_comparison.v1"
ROOT = Path(__file__).resolve().parents[2]
DEFAULT_C_ARM = (ROOT / "artifacts/training/2026-09-21/moving_data_high_budget_ab_seed_pilot_v1"
                 / "remote_results/A_baseline_train")
DEFAULT_D_ARM = (ROOT / "artifacts/training/2026-09-21/moving_data_high_budget_ab_seed_pilot_v1"
                 / "remote_results/B_town07_augmented_train")
DEFAULT_E_ARM = (ROOT / "artifacts/training/2026-09-21/moving_turn_replay_seed_pilot_v1"
                 / "remote_results/E_turn_replay_train")
DEFAULT_E_SUBGROUP = (ROOT / "artifacts/training/2026-09-21/moving_turn_replay_seed_pilot_v1"
                      / "recovery_results/subgroup/summary.json")
F_ARM_NAME = "F_direction_balanced_replay_train"
METRICS = ("ade_6p4s_m", "fde_6p4s_m", "speed_mae_6p4s_mps")
SCOPES = ("right_turn", "straight", "combined")
EPISODES = {
    "right_turn": ("town03_right_turn_dev", 337,
                   "carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9"),
    "straight": ("town03_straight_dev", 328,
                 "carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320"),
}
SHARED_VAL_FINGERPRINT = "9d293b60f0eefc61c924e48fa629ef6f4f43ce0b22171bc0ca78f9fe74c9d2ed"
MODEL_CONFIG_SHA256 = "c17b49ea680b5cae0d9fc35dc6c457af8cc32d1b7171863534f70e9b158cfb7a"
MODEL_PARAMETER_COUNT = 1_056_362
REFERENCE_CHECKPOINTS = {
    "C": "01b6764df96639336ba00cc11bca67406e1aced3829d471d741a1fe60ed8b061",
    "D": "aba44cb524e852aa37e1342ef179a27f471ec1f2507ca1f317648b4ea80f0757",
    "E": "c7739926f25a5c3db7519f33fce236b70d8108e912ec4009bd876afa2a666f78",
}
REFERENCE_METRICS = {
    "C": {
        "right_turn": {"ade_6p4s_m": 2.9353143748791353,
                       "fde_6p4s_m": 7.793866114022117,
                       "speed_mae_6p4s_mps": 1.1076154233086357},
        "straight": {"ade_6p4s_m": 8.090865013076037,
                     "fde_6p4s_m": 19.32532976894844,
                     "speed_mae_6p4s_mps": 3.0487949964476795},
        "combined": {"ade_6p4s_m": 5.4782026726948585,
                     "fde_6p4s_m": 13.48156572445891,
                     "speed_mae_6p4s_mps": 2.065069449485693},
    },
    "D": {
        "right_turn": {"ade_6p4s_m": 3.367132111101547,
                       "fde_6p4s_m": 9.697629961486387,
                       "speed_mae_6p4s_mps": 1.4863724666108717},
        "straight": {"ade_6p4s_m": 3.764740187947343,
                     "fde_6p4s_m": 11.027441243572932,
                     "speed_mae_6p4s_mps": 1.7877504960065935},
        "combined": {"ade_6p4s_m": 3.5632455477589056,
                     "fde_6p4s_m": 10.353536791729749,
                     "speed_mae_6p4s_mps": 1.6350220532345592},
    },
    "E": {
        "right_turn": {"ade_6p4s_m": 3.523327853807002,
                       "fde_6p4s_m": 8.968759292753232,
                       "speed_mae_6p4s_mps": 1.224753898046137},
        "straight": {"ade_6p4s_m": 5.608372367009884,
                     "fde_6p4s_m": 15.109218253958517,
                     "speed_mae_6p4s_mps": 2.4759536138394984},
        "combined": {"ade_6p4s_m": 4.5517407237138965,
                     "fde_6p4s_m": 11.997436759839381,
                     "speed_mae_6p4s_mps": 1.8418869932791344},
    },
}
PARETO_REFERENCES = {"right_turn": "C", "straight": "D", "combined": "D"}
GEOMETRY_MINIMUM = {"selected": 664, "any_candidate": 664, "all_candidates": 664}
SUBGROUP_SCHEMAS = {
    "portable_e2e.moving_data_ab_subgroup_evaluation.v1",
    "portable_e2e.moving_turn_replay_subgroup_evaluation.v1",
    "portable_e2e.moving_direction_balanced_replay_subgroup_evaluation.v1",
}
SUBGROUP_DENIALS = {
    "test_read_or_use": False,
    "training_executed_by_evaluator": False,
    "closed_loop_evaluation": False,
    "vehicle_control_approved": False,
    "automatic_promotion": False,
    "model_exported": False,
}


class ComparisonError(ValueError):
    """HH_260906 - Reject incomplete, mutable, or incomparable evidence."""


def require(value: Any, message: str) -> None:
    """HH_260906 - Raise one fail-closed comparison error for a false condition."""
    if not value:
        raise ComparisonError(message)


def _pairs(items: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in items:
        require(key not in result, f"duplicate JSON key: {key!r}")
        result[key] = value
    return result


def _mapping(value: Any, context: str) -> dict[str, Any]:
    require(type(value) is dict, f"{context} must be an object")
    return value


def _integer(value: Any, context: str, expected: int | None = None) -> int:
    require(type(value) is int and value >= 0, f"{context} must be a nonnegative integer")
    if expected is not None:
        require(value == expected, f"{context} must equal {expected}")
    return value


def _number(value: Any, context: str) -> float:
    require(type(value) in (int, float) and math.isfinite(value) and value >= 0,
            f"{context} must be a finite nonnegative number")
    return float(value)


def _sha256(value: Any, context: str) -> str:
    require(type(value) is str and re.fullmatch(r"[0-9a-f]{64}", value) is not None,
            f"{context} must be a lowercase SHA-256")
    return value


def _canonical_directory(path: Path, context: str) -> Path:
    lexical = Path(path).expanduser().absolute()
    require(".." not in lexical.parts and lexical.exists(), f"{context} does not exist: {lexical}")
    require(lexical.is_dir() and not lexical.is_symlink() and lexical.resolve() == lexical,
            f"{context} must be a canonical nonsymlink directory")
    return lexical


def _canonical_file(path: Path, context: str) -> Path:
    lexical = Path(path).expanduser().absolute()
    require(".." not in lexical.parts and lexical.exists(), f"{context} does not exist: {lexical}")
    require(lexical.is_file() and not lexical.is_symlink() and lexical.resolve() == lexical,
            f"{context} must be a canonical nonsymlink regular file")
    require(lexical.stat().st_nlink == 1, f"{context} must have one hard link")
    return lexical


def _read_json(path: Path, context: str) -> tuple[dict[str, Any], str, Path]:
    source = _canonical_file(path, context)
    try:
        payload = source.read_bytes()
        result = json.loads(payload.decode("utf-8"), object_pairs_hook=_pairs,
                            parse_constant=lambda value: require(
                                False, f"{context} contains nonfinite JSON value {value}"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ComparisonError(f"cannot read strict JSON {source}: {error}") from error
    return _mapping(result, context), hashlib.sha256(payload).hexdigest(), source


def _resolve_arm(path: Path, label: str) -> Path:
    root = _canonical_directory(path, f"{label} path")
    if (root / "evaluation/metrics.json").is_file():
        return root
    candidate = root / (F_ARM_NAME if label == "F" else "")
    require(label == "F" and candidate.is_dir()
            and (candidate / "evaluation/metrics.json").is_file(),
            f"{label} path is neither an arm directory nor a campaign root containing {F_ARM_NAME}")
    return _canonical_directory(candidate, "F arm directory")


def _metric_map(value: Any, context: str) -> dict[str, float]:
    source = _mapping(value, context)
    return {name: _number(source.get(name), f"{context}.{name}") for name in METRICS}


def _verify_metric_counts(value: Any, context: str, expected: int) -> None:
    counts = _mapping(value, context)
    for name in METRICS:
        _integer(counts.get(name), f"{context}.{name}", expected)


def _verify_subgroup(report: dict[str, Any], label: str, checkpoint: str,
                     dataset_fingerprint: str, model_config: str) -> dict[str, dict[str, float]]:
    context = f"{label} subgroup"
    require(report.get("schema") in SUBGROUP_SCHEMAS, f"{context} schema is not accepted")
    require(report.get("status") == "COMPLETE_NOT_PROMOTED", f"{context} status differs")
    for key, expected in SUBGROUP_DENIALS.items():
        require(report.get(key) is expected, f"{context}.{key} must be {expected}")
    require(_sha256(report.get("checkpoint_sha256"), f"{context}.checkpoint_sha256") == checkpoint,
            f"{context} checkpoint differs from evaluation")
    checkpoint_record = _mapping(report.get("checkpoint"), f"{context}.checkpoint")
    require(_sha256(checkpoint_record.get("sha256"), f"{context}.checkpoint.sha256") == checkpoint,
            f"{context} checkpoint record differs")
    require(_sha256(checkpoint_record.get("model_config_sha256"),
                    f"{context}.checkpoint.model_config_sha256") == model_config,
            f"{context} model config differs")
    dataset = _mapping(report.get("dataset"), f"{context}.dataset")
    require(_sha256(dataset.get("combined_val_fingerprint_sha256"),
                    f"{context}.dataset.combined_val_fingerprint_sha256") == dataset_fingerprint,
            f"{context} fixed VAL fingerprint differs")
    counts = _mapping(dataset.get("counts"), f"{context}.dataset.counts")
    _integer(counts.get("val"), f"{context}.dataset.counts.val", 665)
    _integer(report.get("combined_val_count"), f"{context}.combined_val_count", 665)
    episodes = _mapping(report.get("episodes"), f"{context}.episodes")
    metrics: dict[str, dict[str, float]] = {}
    for scope, (episode_label, expected_count, expected_id) in EPISODES.items():
        episode = _mapping(episodes.get(episode_label), f"{context}.episodes.{episode_label}")
        _integer(episode.get("count"), f"{context}.{episode_label}.count", expected_count)
        require(episode.get("episode_id") == expected_id, f"{context}.{episode_label} ID differs")
        _verify_metric_counts(episode.get("metric_counts"),
                              f"{context}.{episode_label}.metric_counts", expected_count)
        metrics[scope] = _metric_map(episode.get("metrics"),
                                     f"{context}.{episode_label}.metrics")
    require(set(episodes) == {value[0] for value in EPISODES.values()},
            f"{context} must contain exactly the two fixed DEV episodes")
    return metrics


def _verify_evaluation(report: dict[str, Any], label: str) -> tuple[str, str, str, dict[str, float]]:
    context = f"{label} evaluation"
    require(report.get("evaluation_id") == "portable_e2e.open_loop_evaluation.v1",
            f"{context} ID differs")
    require(report.get("status") == "OPEN_LOOP_EVALUATION_COMPLETE", f"{context} status differs")
    require(report.get("evaluation_split") == "val", f"{context} must use VAL")
    _integer(report.get("sample_count"), f"{context}.sample_count", 665)
    _integer(report.get("evaluation_episode_count"), f"{context}.evaluation_episode_count", 2)
    require(report.get("vehicle_control_approved") is False,
            f"{context}.vehicle_control_approved must be false")
    checkpoint = _sha256(report.get("checkpoint_sha256"), f"{context}.checkpoint_sha256")
    dataset = _sha256(report.get("dataset_fingerprint_sha256"),
                      f"{context}.dataset_fingerprint_sha256")
    model_config = _sha256(report.get("model_config_sha256"), f"{context}.model_config_sha256")
    _integer(report.get("model_parameter_count"), f"{context}.model_parameter_count",
             MODEL_PARAMETER_COUNT)
    _verify_metric_counts(report.get("metric_counts"), f"{context}.metric_counts", 665)
    metrics = _metric_map(report.get("metrics"), f"{context}.metrics")
    loss_config = _mapping(report.get("loss_config"), f"{context}.loss_config")
    require(bool(loss_config), f"{context}.loss_config cannot be empty")
    return checkpoint, dataset, model_config, metrics


def _verify_gate(report: dict[str, Any], label: str, checkpoint: str,
                 dataset_fingerprint: str, model_config: str) -> dict[str, int]:
    context = f"{label} geometry audit"
    require(report.get("audit_id") == "portable_e2e.runtime_geometry_audit.v8",
            f"{context} ID differs")
    require(report.get("status") == "RUNTIME_GEOMETRY_AUDIT_COMPLETE", f"{context} status differs")
    require(report.get("evaluation_split") == "val", f"{context} must use VAL")
    require(report.get("vehicle_control_approved") is False,
            f"{context}.vehicle_control_approved must be false")
    require(_sha256(report.get("checkpoint_sha256"), f"{context}.checkpoint_sha256") == checkpoint,
            f"{context} checkpoint differs")
    require(_sha256(report.get("dataset_fingerprint_sha256"),
                    f"{context}.dataset_fingerprint_sha256") == dataset_fingerprint,
            f"{context} fixed VAL fingerprint differs")
    require(_sha256(report.get("model_config_sha256"), f"{context}.model_config_sha256")
            == model_config, f"{context} model config differs")
    _integer(report.get("model_parameter_count"), f"{context}.model_parameter_count",
             MODEL_PARAMETER_COUNT)
    geometry = _mapping(report.get("geometry"), f"{context}.geometry")
    _integer(geometry.get("sample_count"), f"{context}.geometry.sample_count", 665)
    selected = _mapping(geometry.get("selected_result"), f"{context}.geometry.selected_result")
    _integer(selected.get("sample_count"), f"{context}.geometry.selected_result.sample_count", 665)
    return {
        "selected": _integer(selected.get("geometry_pass_count"),
                             f"{context}.geometry.selected_result.geometry_pass_count"),
        "any_candidate": _integer(geometry.get("any_candidate_geometry_pass_count"),
                                  f"{context}.geometry.any_candidate_geometry_pass_count"),
        "all_candidates": _integer(geometry.get("all_candidates_geometry_pass_count"),
                                   f"{context}.geometry.all_candidates_geometry_pass_count"),
        "sample_count": 665,
    }


def _reconstructed_combined(metrics: dict[str, dict[str, float]], name: str) -> float:
    return (metrics["right_turn"][name] * 337 + metrics["straight"][name] * 328) / 665


def _load_model(label: str, arm_path: Path, subgroup_path: Path | None = None) -> dict[str, Any]:
    arm = _resolve_arm(arm_path, label)
    evaluation, evaluation_sha, evaluation_path = _read_json(
        arm / "evaluation/metrics.json", f"{label} evaluation")
    gate, gate_sha, gate_path = _read_json(arm / "gate_v8.json", f"{label} geometry audit")
    subgroup_source = subgroup_path if subgroup_path is not None else arm / "subgroup/summary.json"
    subgroup, subgroup_sha, subgroup_file = _read_json(subgroup_source, f"{label} subgroup")
    checkpoint, dataset_fingerprint, model_config, combined = _verify_evaluation(evaluation, label)
    subgroup_metrics = _verify_subgroup(subgroup, label, checkpoint, dataset_fingerprint, model_config)
    geometry = _verify_gate(gate, label, checkpoint, dataset_fingerprint, model_config)
    metrics = {**subgroup_metrics, "combined": combined}
    for name in METRICS:
        require(abs(_reconstructed_combined(metrics, name) - combined[name]) <= 1e-5,
                f"{label} combined {name} is inconsistent with fixed subgroup evidence")
    if label in REFERENCE_CHECKPOINTS:
        require(checkpoint == REFERENCE_CHECKPOINTS[label], f"{label} checkpoint anchor differs")
        for scope in SCOPES:
            for name in METRICS:
                require(metrics[scope][name] == REFERENCE_METRICS[label][scope][name],
                        f"{label} anchored {scope}.{name} differs")
        require(all(geometry[key] == 664 for key in GEOMETRY_MINIMUM),
                f"{label} anchored geometry counts differ")
    if label == "F":
        require(evaluation.get("training_sampling_policy") == "direction_balanced_cohort_replay",
                "F evaluation sampling policy differs")
        require(evaluation.get("training_domain_samples_seen") == {"carla": 33120},
                "F evaluation exposure total differs")
        require(evaluation.get("training_sampling_stream_samples_seen")
                == {"base": 29500, "replay": 3620}, "F replay accounting differs")
    return {
        "checkpoint_sha256": checkpoint,
        "dataset_fingerprint_sha256": dataset_fingerprint,
        "model_config_sha256": model_config,
        "model_parameter_count": MODEL_PARAMETER_COUNT,
        "loss_config": evaluation["loss_config"],
        "metrics": metrics,
        "geometry": geometry,
        "evidence": {
            "evaluation": {"path": str(evaluation_path), "sha256": evaluation_sha},
            "subgroup": {"path": str(subgroup_file), "sha256": subgroup_sha},
            "geometry_audit": {"path": str(gate_path), "sha256": gate_sha},
        },
    }


def _comparison(candidate: float, reference: float) -> dict[str, Any]:
    require(reference > 0, "reference metric must be positive for percentage comparison")
    delta = candidate - reference
    if delta < 0:
        direction = "improved"
    elif delta > 0:
        direction = "regressed"
    else:
        direction = "equal"
    return {
        "reference": reference,
        "candidate_f": candidate,
        "absolute_delta_f_minus_reference": delta,
        "relative_change_percent": delta / reference * 100.0,
        "lower_is_better": True,
        "direction": direction,
    }


def build_report(c_arm: Path, d_arm: Path, e_arm: Path, f_arm: Path,
                 e_subgroup: Path | None = None, f_subgroup: Path | None = None,
                 created_at_utc: str | None = None) -> dict[str, Any]:
    """HH_260906 - Validate all evidence and calculate the independent F verdict."""
    models = {
        "C": _load_model("C", c_arm),
        "D": _load_model("D", d_arm),
        "E": _load_model("E", e_arm, e_subgroup),
        "F": _load_model("F", f_arm, f_subgroup),
    }
    require(len({models[label]["checkpoint_sha256"] for label in models}) == 4,
            "C, D, E, and F checkpoints must be distinct")
    require({models[label]["dataset_fingerprint_sha256"] for label in models}
            == {SHARED_VAL_FINGERPRINT}, "all models must use the anchored fixed VAL fingerprint")
    require({models[label]["model_config_sha256"] for label in models}
            == {MODEL_CONFIG_SHA256}, "all models must use the frozen model config")
    require(all(models[label]["loss_config"] == models["C"]["loss_config"] for label in models),
            "all models must use the frozen loss configuration")
    comparisons: dict[str, Any] = {}
    for reference_label in ("C", "D", "E"):
        comparisons[f"F_vs_{reference_label}"] = {
            scope: {
                name: _comparison(models["F"]["metrics"][scope][name],
                                  models[reference_label]["metrics"][scope][name])
                for name in METRICS
            }
            for scope in SCOPES
        }
    benchmark_results: dict[str, Any] = {}
    performance_pass = True
    for scope, reference_label in PARETO_REFERENCES.items():
        metric_results = {}
        for name in METRICS:
            candidate = models["F"]["metrics"][scope][name]
            maximum = models[reference_label]["metrics"][scope][name]
            passed = candidate <= maximum
            performance_pass = performance_pass and passed
            metric_results[name] = {"candidate_f": candidate, "maximum": maximum, "passed": passed}
        benchmark_results[scope] = {"reference_model": reference_label,
                                    "metrics": metric_results,
                                    "passed": all(row["passed"] for row in metric_results.values())}
    geometry_results = {}
    geometry_pass = True
    for name, minimum in GEOMETRY_MINIMUM.items():
        observed = models["F"]["geometry"][name]
        passed = observed >= minimum
        geometry_pass = geometry_pass and passed
        geometry_results[name] = {"observed": observed, "minimum": minimum,
                                  "denominator": 665, "passed": passed}
    passed = performance_pass and geometry_pass
    return {
        "schema": SCHEMA,
        "created_at_utc": created_at_utc or datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "status": "PASS_OPEN_LOOP_DIAGNOSTIC_GATE" if passed else "FAIL_OPEN_LOOP_DIAGNOSTIC_GATE",
        "scope": "FIXED_DEV_OPEN_LOOP_ONLY",
        "models": models,
        "comparisons": comparisons,
        "strict_pareto_gate": {
            "passed": passed,
            "performance_passed": performance_pass,
            "geometry_passed": geometry_pass,
            "benchmarks": benchmark_results,
            "geometry": geometry_results,
            "rule": "Every F metric must be no greater than its fixed reference and every geometry count must meet its minimum.",
        },
        "limitations": {
            "test_read_or_use": False,
            "closed_loop_evaluation": False,
            "vehicle_control_approved": False,
            "model_exported": False,
            "automatic_promotion": False,
            "head_addition_approved": False,
            "all_map_claim": False,
        },
        "interpretation": (
            "This report compares fixed-DEV open-loop trajectory metrics only. A passing gate does not "
            "authorize TEST use, closed-loop driving, export, promotion, head addition, or vehicle control."
        ),
    }


def render_markdown(report: dict[str, Any]) -> str:
    """HH_260906 - Render the machine verdict as a compact human review document."""
    gate = report["strict_pareto_gate"]
    lines = [
        "# Moving direction-balanced replay F comparison",
        "",
        f"- Verdict: `{report['status']}`",
        "- Scope: fixed DEV, open-loop trajectory evaluation only",
        "- Lower is better for ADE, FDE, and speed MAE",
        "- No TEST, closed-loop, export, promotion, head-addition, all-map, or vehicle claim",
        "",
        "## Strict Pareto gate",
        "",
        "| Scope | Reference | Metric | F | Maximum | Result |",
        "|---|---:|---|---:|---:|---|",
    ]
    for scope in SCOPES:
        row = gate["benchmarks"][scope]
        for name in METRICS:
            metric = row["metrics"][name]
            lines.append(f"| {scope} | {row['reference_model']} | {name} | "
                         f"{metric['candidate_f']:.12g} | {metric['maximum']:.12g} | "
                         f"{'PASS' if metric['passed'] else 'FAIL'} |")
    lines.extend([
        "",
        "## Geometry gate",
        "",
        "| Measure | F count | Required | Result |",
        "|---|---:|---:|---|",
    ])
    for name in ("selected", "any_candidate", "all_candidates"):
        row = gate["geometry"][name]
        lines.append(f"| {name} | {row['observed']}/665 | >= {row['minimum']}/665 | "
                     f"{'PASS' if row['passed'] else 'FAIL'} |")
    for reference_label in ("C", "D", "E"):
        lines.extend([
            "",
            f"## F versus {reference_label}",
            "",
            "| Scope | Metric | Reference | F | F-reference | Change | Direction |",
            "|---|---|---:|---:|---:|---:|---|",
        ])
        for scope in SCOPES:
            for name in METRICS:
                row = report["comparisons"][f"F_vs_{reference_label}"][scope][name]
                lines.append(f"| {scope} | {name} | {row['reference']:.12g} | "
                             f"{row['candidate_f']:.12g} | "
                             f"{row['absolute_delta_f_minus_reference']:+.12g} | "
                             f"{row['relative_change_percent']:+.6f}% | {row['direction']} |")
    lines.extend([
        "",
        "## Interpretation boundary",
        "",
        report["interpretation"],
        "",
    ])
    return "\n".join(lines)


def _validate_output(path: Path, context: str) -> Path:
    target = Path(path).expanduser().absolute()
    require(".." not in target.parts and not target.exists() and not target.is_symlink(),
            f"{context} must be a fresh path")
    _canonical_directory(target.parent, f"{context} parent")
    return target


def write_outputs(report: dict[str, Any], json_output: Path, markdown_output: Path) -> None:
    """HH_260906 - Create both evidence files without replacing a prior verdict."""
    json_path = _validate_output(json_output, "JSON output")
    markdown_path = _validate_output(markdown_output, "Markdown output")
    require(json_path != markdown_path, "JSON and Markdown outputs must differ")
    json_bytes = (json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()
    markdown_bytes = render_markdown(report).encode("utf-8")
    created: list[Path] = []
    try:
        for target, payload in ((json_path, json_bytes), (markdown_path, markdown_bytes)):
            with target.open("xb") as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
            created.append(target)
    except OSError:
        for target in created:
            target.unlink(missing_ok=True)
        raise


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """HH_260906 - Parse explicit evidence and create-only output paths."""
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--f-arm-dir", type=Path, required=True)
    parser.add_argument("--f-subgroup-summary", type=Path)
    parser.add_argument("--c-arm-dir", type=Path, default=DEFAULT_C_ARM)
    parser.add_argument("--d-arm-dir", type=Path, default=DEFAULT_D_ARM)
    parser.add_argument("--e-arm-dir", type=Path, default=DEFAULT_E_ARM)
    parser.add_argument("--e-subgroup-summary", type=Path, default=DEFAULT_E_SUBGROUP)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-markdown", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """HH_260906 - Write a report and return one only when the diagnostic gate fails."""
    args = parse_args(argv)
    try:
        report = build_report(args.c_arm_dir, args.d_arm_dir, args.e_arm_dir, args.f_arm_dir,
                              args.e_subgroup_summary, args.f_subgroup_summary)
        write_outputs(report, args.output_json, args.output_markdown)
    except (ComparisonError, OSError) as error:
        print(f"MOVING_DIRECTION_BALANCED_REPLAY_COMPARISON_ERROR: {error}", file=sys.stderr)
        return 2
    print(json.dumps({"status": report["status"],
                      "passed": report["strict_pareto_gate"]["passed"],
                      "output_json": str(Path(args.output_json).absolute()),
                      "output_markdown": str(Path(args.output_markdown).absolute())},
                     sort_keys=True, separators=(",", ":"), allow_nan=False))
    return 0 if report["strict_pareto_gate"]["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
