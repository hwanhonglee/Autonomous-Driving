#!/usr/bin/env python3
"""HH_260906 - Build one immutable A/B/C/D summary from the two sealed moving-data runs."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
from typing import Any


SCHEMA = "portable_e2e.moving_data_2x2_summary.v1"
PLAN_SCHEMA = "portable_e2e.moving_data_ab_seed_pilot_plan.v1"
SUBGROUP_SCHEMA = "portable_e2e.moving_data_ab_subgroup_evaluation.v1"
PHYSICAL_ARMS = ("A_baseline_train", "B_town07_augmented_train")
LOW_LABELS = {PHYSICAL_ARMS[0]: "A", PHYSICAL_ARMS[1]: "B"}
HIGH_LABELS = {PHYSICAL_ARMS[0]: "C", PHYSICAL_ARMS[1]: "D"}
CAMPAIGNS = {
    "low": "hh260921-moving-data-ab-seed20260906-v1",
    "high": "hh260921-moving-data-high-budget-ab-seed20260906-v1",
}
SUMMARY_SCHEMAS = {
    "low": "portable_e2e.moving_data_ab_seed_pilot.v1",
    "high": "portable_e2e.moving_data_high_budget_ab_pilot.v1",
}
EXPERIMENTAL_LABELS = {
    PHYSICAL_ARMS[0]: "C_old_train_7380_steps",
    PHYSICAL_ARMS[1]: "D_augmented_train_7380_steps",
}
EXPECTED_STATES = {
    "A": {"epoch": 20, "next_batch_index": 0, "global_step": 5740,
          "samples_seen": 22940, "domain_samples_seen": {"carla": 22940}},
    "B": {"epoch": 15, "next_batch_index": 205, "global_step": 5740,
          "samples_seen": 22945, "domain_samples_seen": {"carla": 22945}},
    "C": {"epoch": 25, "next_batch_index": 205, "global_step": 7380,
          "samples_seen": 29495, "domain_samples_seen": {"carla": 29495}},
    "D": {"epoch": 20, "next_batch_index": 0, "global_step": 7380,
          "samples_seen": 29500, "domain_samples_seen": {"carla": 29500}},
}
EPISODES = (
    {"label": "town03_right_turn_dev",
     "episode_id": "carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9",
     "count": 337},
    {"label": "town03_straight_dev",
     "episode_id": "carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320",
     "count": 328},
)
EXPECTED_CD_THRESHOLDS = {
    "town03_straight_dev": {"ade_6p4s_m": 0.90, "fde_6p4s_m": 0.90},
    "town03_right_turn_dev": {
        "ade_6p4s_m": 1.0,
        "fde_6p4s_m": 1.0,
        "speed_mae_6p4s_mps": 1.05,
    },
}
DENIAL_FIELDS = (
    "resume",
    "automatic_retry",
    "test_read_or_use",
    "automatic_promotion",
    "training_data_approved",
    "vehicle_control_approved",
    "deployment_approved",
)


class SummaryError(ValueError):
    """HH_260906 - Mark an input as unfit for the sealed four-arm comparison."""


def require(value: Any, message: str) -> None:
    if not value:
        raise SummaryError(message)


def _pairs(items: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in items:
        require(key not in result, f"duplicate JSON key: {key!r}")
        result[key] = value
    return result


def _mapping(value: Any, context: str) -> dict[str, Any]:
    require(type(value) is dict, f"{context} must be an object")
    return value


def _number(value: Any, context: str) -> float | int:
    require(type(value) in (int, float) and math.isfinite(value),
            f"{context} must be a finite JSON number")
    return value


def _sha256_text(value: Any, context: str) -> str:
    require(type(value) is str and re.fullmatch(r"[0-9a-f]{64}", value) is not None,
            f"{context} must be a SHA-256")
    return value


def _canonical_directory(path: Path, context: str) -> Path:
    lexical = Path(path).expanduser().absolute()
    require(".." not in lexical.parts and lexical.exists(), f"{context} does not exist")
    require(lexical.resolve() == lexical and lexical.is_dir() and not lexical.is_symlink(),
            f"{context} must be a canonical nonsymlink directory")
    return lexical


def _regular(path: Path, context: str) -> Path:
    lexical = Path(path).expanduser().absolute()
    require(".." not in lexical.parts and lexical.exists(), f"{context} does not exist: {lexical}")
    require(lexical.resolve() == lexical and lexical.is_file() and not lexical.is_symlink(),
            f"{context} must be a canonical regular file")
    return lexical


def digest(path: Path) -> str:
    result = hashlib.sha256()
    with _regular(path, "hash input").open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def read_json(path: Path) -> dict[str, Any]:
    try:
        return _mapping(json.loads(
            _regular(path, "JSON input").read_text(encoding="utf-8"),
            object_pairs_hook=_pairs,
            parse_constant=lambda value: require(False, f"nonfinite JSON value: {value}"),
        ), str(path))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise SummaryError(f"cannot read strict JSON {path}: {error}") from error


def _encoded(value: dict[str, Any]) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()


def write_new(path: Path, value: dict[str, Any]) -> None:
    # HH_260906 - The comparison is evidence, so replacement and partial overwrite are forbidden.
    target = Path(path).expanduser().absolute()
    require(".." not in target.parts and not target.exists() and not target.is_symlink(),
            "output JSON must be fresh")
    parent = _canonical_directory(target.parent, "output parent")
    target = parent / target.name
    with target.open("xb") as stream:
        stream.write(_encoded(value))
        stream.flush()
        os.fsync(stream.fileno())


def _finite_metrics(value: Any, count_value: Any, expected_count: int, context: str) -> dict[str, float | int]:
    metrics = _mapping(value, context + ".metrics")
    counts = _mapping(count_value, context + ".metric_counts")
    require(metrics and set(metrics) == set(counts), f"{context} metric/count keys differ")
    result: dict[str, float | int] = {}
    for key in sorted(metrics):
        require(type(key) is str and key, f"{context} metric name is invalid")
        result[key] = _number(metrics[key], f"{context}.{key}")
        require(counts[key] == expected_count, f"{context}.{key} denominator differs")
    return result


def _episode_rows(summary: dict[str, Any], context: str) -> dict[str, dict[str, Any]]:
    raw = summary.get("episodes")
    rows = list(raw.values()) if type(raw) is dict else raw
    require(type(rows) is list, f"{context}.episodes must be an object or array")
    result: dict[str, dict[str, Any]] = {}
    for row in rows:
        item = _mapping(row, f"{context}.episode")
        label = next((spec["label"] for spec in EPISODES
                      if spec["episode_id"] == item.get("episode_id")), None)
        require(label is not None and label not in result, f"{context} episode identity differs")
        result[label] = item
    require(set(result) == {item["label"] for item in EPISODES},
            f"{context} fixed episode set differs")
    return result


def _geometry(audit: dict[str, Any], context: str) -> dict[str, dict[str, float | int]]:
    geometry = _mapping(audit.get("geometry"), context + ".geometry")
    selected = _mapping(geometry.get("selected_result"), context + ".selected_result")
    sample_count = geometry.get("sample_count")
    require(sample_count == selected.get("sample_count") == 665,
            f"{context} geometry denominator differs")
    counts = {
        "selected": selected.get("geometry_pass_count"),
        "any_candidate": geometry.get("any_candidate_geometry_pass_count"),
        "all_candidates": geometry.get("all_candidates_geometry_pass_count"),
    }
    result: dict[str, dict[str, float | int]] = {}
    for name, count in counts.items():
        require(type(count) is int and 0 <= count <= sample_count,
                f"{context}.{name} pass count differs")
        result[name] = {"pass_count": count, "sample_count": sample_count,
                        "pass_rate": count / sample_count}
    return result


def _history(path: Path, expected: dict[str, Any], context: str) -> tuple[list[bytes], list[dict[str, Any]], str]:
    lines: list[bytes] = []
    records: list[dict[str, Any]] = []
    with _regular(path, context).open("rb") as stream:
        for line_number, line in enumerate(stream, 1):
            require(line.endswith(b"\n") and line.strip(), f"{context} line framing differs")
            try:
                record = _mapping(json.loads(
                    line.decode("utf-8"), object_pairs_hook=_pairs,
                    parse_constant=lambda value: require(False, f"nonfinite JSON value: {value}"),
                ), f"{context} line {line_number}")
            except (UnicodeError, json.JSONDecodeError) as error:
                raise SummaryError(f"invalid strict JSONL at {context} line {line_number}: {error}") from error
            require(record.get("global_step") == line_number,
                    f"{context} global_step is not sequential")
            lines.append(line)
            records.append(record)
    require(len(lines) == expected["global_step"] and
            records[-1].get("samples_seen") == expected["samples_seen"],
            f"{context} endpoint differs")
    return lines, records, digest(path)


def _stage_index(root: Path, summary: dict[str, Any], plan_sha: str) -> dict[tuple[str, str], dict[str, Any]]:
    rows = summary.get("stages")
    require(type(rows) is list and len(rows) == 8, "summary must contain eight completed stages")
    result: dict[tuple[str, str], dict[str, Any]] = {}
    expected_paths = {
        "train": "training/run.json",
        "evaluate": "evaluation/metrics.json",
        "audit": "gate_v8.json",
        "subgroup": "subgroup/summary.json",
    }
    for raw in rows:
        row = _mapping(raw, "summary stage")
        arm, stage = row.get("arm"), row.get("stage")
        require(arm in PHYSICAL_ARMS and stage in expected_paths and (arm, stage) not in result,
                "summary stage identity differs")
        require(row.get("status") == "COMPLETE" and row.get("plan_sha256") == plan_sha,
                "summary stage completion/provenance differs")
        expected_path = f"{arm}/{expected_paths[stage]}"
        require(row.get("path") == expected_path and row.get("sha256") == digest(root / expected_path),
                "summary stage report digest differs")
        receipt = read_json(root / arm / f"{stage}_completed.json")
        require(receipt == row, "summary stage differs from its immutable receipt")
        result[(arm, stage)] = row
    return result


def _load_campaign(root_path: Path, budget: str) -> dict[str, Any]:
    root = _canonical_directory(root_path, f"{budget}-budget root")
    require(not (root / "failed.json").exists(), f"{budget}-budget run has failed.json")
    plan_path, summary_path = root / "plan.json", root / "summary.json"
    plan, summary = read_json(plan_path), read_json(summary_path)
    plan_sha = digest(plan_path)
    require(plan.get("schema") == PLAN_SCHEMA and plan.get("campaign_id") == CAMPAIGNS[budget],
            f"{budget}-budget plan identity differs")
    require(summary.get("schema") == SUMMARY_SCHEMAS[budget] and
            summary.get("status") == "COMPLETE_NOT_PROMOTED" and
            summary.get("plan_sha256") == plan_sha and
            summary.get("source_and_views_postcheck_pass") is True,
            f"{budget}-budget completion provenance differs")
    require(summary.get("arms") == {arm: "COMPLETE" for arm in PHYSICAL_ARMS},
            f"{budget}-budget arms are incomplete")
    for field in DENIAL_FIELDS:
        require(plan.get(field) is False and summary.get(field) is False,
                f"{budget}-budget {field} must remain false")
    require(plan.get("closed_loop_claim") is False and summary.get("closed_loop_claim") is False,
            f"{budget}-budget closed-loop claim must remain false")
    require(plan.get("all_map_claim") is False and summary.get("all_map_claim") is False,
            f"{budget}-budget all-map claim must remain false")
    require(plan.get("model_export") is False and summary.get("model_export") is False,
            f"{budget}-budget model export must remain false")
    require(set(plan.get("arms", {})) == set(PHYSICAL_ARMS), f"{budget}-budget plan arms differ")
    require(plan.get("validation") == {
        "episodes": list(EPISODES),
        "count": 665,
        "fingerprint_sha256": plan["arms"][PHYSICAL_ARMS[0]]["dataset"]["split_examples_sha256"]["val"],
        "tokens_sha256": plan["arms"][PHYSICAL_ARMS[0]]["dataset"]["split_tokens_sha256"]["val"],
    }, f"{budget}-budget validation declaration differs")
    if budget == "high":
        decision = _mapping(plan.get("decision"), "high-budget decision")
        require(decision.get("experimental_arm_labels") == EXPERIMENTAL_LABELS and
                decision.get("maximum_D_over_C") == EXPECTED_CD_THRESHOLDS and
                decision.get("optimizer_steps_per_arm") == 7380 and
                decision.get("three_seed_confirmation_required") is True and
                decision.get("adoption_allowed") is False,
                "high-budget preregistered decision differs")
        require(summary.get("comparison", {}).get("experimental_arm_labels") == EXPERIMENTAL_LABELS,
                "high-budget summary arm labels differ")
    stages = _stage_index(root, summary, plan_sha)
    labels = LOW_LABELS if budget == "low" else HIGH_LABELS
    result_arms: dict[str, Any] = {}
    for physical_arm in PHYSICAL_ARMS:
        label = labels[physical_arm]
        state = EXPECTED_STATES[label]
        plan_arm = _mapping(plan["arms"][physical_arm], f"plan arm {label}")
        data = _mapping(plan_arm.get("dataset"), f"plan dataset {label}")
        require(plan_arm.get("expected_train_state") == state,
                f"arm {label} expected train state differs")
        if budget == "high":
            require(plan_arm.get("train_config", {}).get("max_steps") == 7380,
                    f"arm {label} high-budget train config differs")
        else:
            require(plan.get("train_config", {}).get("max_steps") == 5740,
                    f"arm {label} low-budget train config differs")
        require(data.get("counts") == ({"train": 1147, "val": 665} if physical_arm == PHYSICAL_ARMS[0]
                                       else {"train": 1475, "val": 665}),
                f"arm {label} dataset counts differ")
        for field in ("manifest_sha256", "corpus_fingerprint_sha256"):
            _sha256_text(data.get(field), f"arm {label} {field}")
        for field in ("split_fingerprint_sha256", "split_loader_sha256",
                      "split_examples_sha256", "split_tokens_sha256"):
            require(set(data.get(field, {})) == {"train", "val"}, f"arm {label} {field} differs")
            for split in ("train", "val"):
                _sha256_text(data[field][split], f"arm {label} {field}.{split}")
        item = root / physical_arm
        train = read_json(item / "training/run.json")
        checkpoint_path = item / "training/checkpoints/latest.pt"
        checkpoint_sha = digest(checkpoint_path)
        expected_train_config = (plan_arm["train_config"] if budget == "high" else plan["train_config"])
        require(train.get("status") == "TRAINING_TARGET_REACHED" and train.get("state") == state and
                train.get("dataset_size") == data["counts"]["train"] and
                train.get("dataset_fingerprint_sha256") == data["split_fingerprint_sha256"]["train"] and
                train.get("corpus_fingerprint_sha256") == data["corpus_fingerprint_sha256"] and
                train.get("train_config") == expected_train_config and
                train.get("loss_config") == plan.get("loss_config") and
                train.get("device") == "cuda:0" and
                train.get("vehicle_control_approved", False) is False,
                f"arm {label} training provenance differs")
        history_lines, history_records, history_sha = _history(
            item / "training/metrics.jsonl", state, f"arm {label} history")
        require(stages[(physical_arm, "train")].get("checkpoint_sha256") == checkpoint_sha and
                stages[(physical_arm, "train")].get("history_sha256") == history_sha,
                f"arm {label} training receipt differs")
        evaluation = read_json(item / "evaluation/metrics.json")
        eval_metrics = _finite_metrics(evaluation.get("metrics"), evaluation.get("metric_counts"),
                                       665, f"arm {label} combined VAL")
        require(evaluation.get("status") == "OPEN_LOOP_EVALUATION_COMPLETE" and
                evaluation.get("evaluation_split") == "val" and evaluation.get("sample_count") == 665 and
                evaluation.get("evaluation_episode_count") == 2 and
                evaluation.get("checkpoint_sha256") == checkpoint_sha and
                evaluation.get("corpus_fingerprint_sha256") == data["corpus_fingerprint_sha256"] and
                evaluation.get("dataset_fingerprint_sha256") == data["split_fingerprint_sha256"]["val"] and
                evaluation.get("training_dataset_fingerprint_sha256") == data["split_fingerprint_sha256"]["train"] and
                evaluation.get("model_config_sha256") == plan.get("model_config_canonical_sha256") and
                evaluation.get("loss_config") == plan.get("loss_config") and
                evaluation.get("vehicle_control_approved") is False,
                f"arm {label} evaluation provenance differs")
        subgroup_path = item / "subgroup/summary.json"
        subgroup = read_json(subgroup_path)
        require(subgroup.get("schema") == SUBGROUP_SCHEMA and subgroup.get("status") == "COMPLETE_NOT_PROMOTED" and
                subgroup.get("arm") == physical_arm and subgroup.get("plan_sha256") == plan_sha and
                subgroup.get("source_commit") == plan.get("source_commit") and
                subgroup.get("evaluator_sha256") == plan.get("evaluator_sha256") and
                subgroup.get("source_sha256") == plan.get("source_sha256") and
                subgroup.get("combined_val_count") == 665 and
                subgroup.get("checkpoint_sha256") == checkpoint_sha and
                subgroup.get("corpus_fingerprint_sha256") == data["corpus_fingerprint_sha256"] and
                subgroup.get("test_read_or_use") is False and
                subgroup.get("closed_loop_evaluation") is False and
                subgroup.get("automatic_promotion") is False and
                subgroup.get("training_executed_by_evaluator") is False and
                subgroup.get("model_exported") is False and
                subgroup.get("vehicle_control_approved") is False,
                f"arm {label} subgroup provenance differs")
        subgroup_data = _mapping(subgroup.get("dataset"), f"arm {label} subgroup dataset")
        require(subgroup_data.get("combined_val_example_fingerprint_sha256") ==
                data["split_examples_sha256"]["val"] and
                subgroup_data.get("combined_val_fingerprint_sha256") ==
                data["split_fingerprint_sha256"]["val"] and
                subgroup_data.get("train_fingerprint_sha256") ==
                data["split_fingerprint_sha256"]["train"] and
                subgroup_data.get("manifest_sha256") == data["manifest_sha256"] and
                subgroup_data.get("counts") == data["counts"],
                f"arm {label} subgroup dataset provenance differs")
        subgroup_checkpoint = _mapping(subgroup.get("checkpoint"), f"arm {label} subgroup checkpoint")
        require(subgroup_checkpoint.get("sha256") == checkpoint_sha and
                subgroup_checkpoint.get("training_dataset_fingerprint_sha256") ==
                data["split_fingerprint_sha256"]["train"] and
                subgroup_checkpoint.get("corpus_fingerprint_sha256") == data["corpus_fingerprint_sha256"] and
                subgroup_checkpoint.get("model_config_sha256") == plan.get("model_config_canonical_sha256"),
                f"arm {label} subgroup checkpoint provenance differs")
        episode_metrics: dict[str, dict[str, float | int]] = {}
        for spec in EPISODES:
            row = _episode_rows(subgroup, f"arm {label} subgroup")[spec["label"]]
            require(row.get("episode_id") == spec["episode_id"] and row.get("count") == spec["count"],
                    f"arm {label} subgroup episode differs")
            episode_metrics[spec["label"]] = _finite_metrics(
                row.get("metrics"), row.get("metric_counts"), spec["count"],
                f"arm {label} {spec['label']}")
            report_path = row.get("report_path")
            require(type(report_path) is str and not Path(report_path).is_absolute() and
                    ".." not in Path(report_path).parts and
                    row.get("report_sha256") == digest(item / "subgroup" / report_path),
                    f"arm {label} subgroup episode report provenance differs")
        require(set(eval_metrics) == set.intersection(*(set(value) for value in episode_metrics.values())),
                f"arm {label} combined/subgroup metric keys differ")
        for key, observed in eval_metrics.items():
            weighted = sum(episode_metrics[spec["label"]][key] * spec["count"] for spec in EPISODES) / 665
            require(math.isclose(observed, weighted, rel_tol=0.0, abs_tol=1e-6),
                    f"arm {label} combined/subgroup metric aggregation differs: {key}")
        audit = read_json(item / "gate_v8.json")
        require(audit.get("status") == "RUNTIME_GEOMETRY_AUDIT_COMPLETE" and
                audit.get("evaluation_split") == "val" and audit.get("evaluation_episode_count") == 2 and
                audit.get("checkpoint_sha256") == checkpoint_sha and
                audit.get("corpus_fingerprint_sha256") == data["corpus_fingerprint_sha256"] and
                audit.get("dataset_fingerprint_sha256") == data["split_fingerprint_sha256"]["val"] and
                audit.get("training_dataset_fingerprint_sha256") == data["split_fingerprint_sha256"]["train"] and
                audit.get("model_config_sha256") == plan.get("model_config_canonical_sha256") and
                audit.get("gate", {}).get("source") == "portable_e2e.runtime_geometry_gate.v8" and
                audit.get("gate", {}).get("threshold_overrides") is False and
                audit.get("vehicle_control_approved") is False,
                f"arm {label} geometry gate provenance differs")
        geometry = _geometry(audit, f"arm {label}")
        require(summary.get("comparison", {}).get("arms", {}).get(physical_arm, {}).get(
            "subgroup_summary_sha256") == digest(subgroup_path),
            f"arm {label} runner comparison subgroup digest differs")
        result_arms[label] = {
            "physical_arm": physical_arm,
            "dataset": data,
            "state": state,
            "metrics": {"combined_val": eval_metrics, **episode_metrics},
            "geometry": geometry,
            "geometry_gate_contract": audit["gate"],
            "history_lines": history_lines,
            "history_records": history_records,
            "provenance": {
                "checkpoint_sha256": checkpoint_sha,
                "history_sha256": history_sha,
                "evaluation_sha256": digest(item / "evaluation/metrics.json"),
                "subgroup_sha256": digest(subgroup_path),
                "geometry_gate_sha256": digest(item / "gate_v8.json"),
            },
        }
    delta_name = "metric_delta_B_minus_A_by_episode" if budget == "low" else "metric_delta_D_minus_C_by_episode"
    left, right = (("A", "B") if budget == "low" else ("C", "D"))
    expected_deltas = {
        spec["label"]: {key: result_arms[right]["metrics"][spec["label"]][key] -
                             result_arms[left]["metrics"][spec["label"]][key]
                        for key in result_arms[left]["metrics"][spec["label"]]}
        for spec in EPISODES
    }
    require(summary.get("comparison", {}).get(delta_name) == expected_deltas,
            f"{budget}-budget runner metric deltas differ")
    return {
        "root": root,
        "plan": plan,
        "summary": summary,
        "plan_sha256": plan_sha,
        "summary_sha256": digest(summary_path),
        "arms": result_arms,
    }


def _percent(delta: float | int, denominator: float | int) -> float | None:
    return None if denominator == 0 else delta / denominator * 100.0


def _metric_contrasts(values: dict[str, float | int]) -> dict[str, Any]:
    a, b, c, d = (values[label] for label in "ABCD")
    ba, dc, ca, db = b - a, d - c, c - a, d - b
    ba_percent, dc_percent = _percent(ba, a), _percent(dc, c)
    return {
        "B_minus_A": {"absolute": ba, "percent_of_A": ba_percent},
        "D_minus_C": {"absolute": dc, "percent_of_C": dc_percent},
        "C_minus_A": {"absolute": ca, "percent_of_A": _percent(ca, a)},
        "D_minus_B": {"absolute": db, "percent_of_B": _percent(db, b)},
        "interaction": {
            "absolute_D_minus_C_minus_B_minus_A": dc - ba,
            "percentage_points_D_over_C_minus_B_over_A": (
                None if ba_percent is None or dc_percent is None else dc_percent - ba_percent
            ),
        },
    }


def _compare_histories(low: dict[str, Any], high: dict[str, Any], old: str, new: str) -> dict[str, Any]:
    historical = low["arms"][old]
    extended = high["arms"][new]
    old_lines, new_lines = historical["history_lines"], extended["history_lines"]
    require(len(old_lines) == 5740 and len(new_lines) == 7380,
            f"{old}/{new} history lengths differ from the declared budgets")
    first = next((index for index, pair in enumerate(zip(old_lines, new_lines), 1)
                  if pair[0] != pair[1]), None)
    mismatch = None
    if first is not None:
        mismatch = {
            "line_number": first,
            "historical_line_sha256": hashlib.sha256(old_lines[first - 1]).hexdigest(),
            "higher_budget_line_sha256": hashlib.sha256(new_lines[first - 1]).hexdigest(),
            "historical_record": historical["history_records"][first - 1],
            "higher_budget_record": extended["history_records"][first - 1],
        }
    old_prefix = b"".join(old_lines)
    new_prefix = b"".join(new_lines[:5740])
    return {
        "historical_arm": old,
        "higher_budget_arm": new,
        "compared_line_count": 5740,
        "historical_total_line_count": len(old_lines),
        "higher_budget_total_line_count": len(new_lines),
        "byte_identical": first is None,
        "historical_prefix_sha256": hashlib.sha256(old_prefix).hexdigest(),
        "higher_budget_prefix_sha256": hashlib.sha256(new_prefix).hexdigest(),
        "first_mismatch": mismatch,
    }


def _cd_gate(high: dict[str, Any]) -> dict[str, Any]:
    arms = high["arms"]
    checks: dict[str, Any] = {}
    for spec in EPISODES:
        label = spec["label"]
        checks[label] = {}
        for key, multiplier in EXPECTED_CD_THRESHOLDS[label].items():
            c, d = arms["C"]["metrics"][label][key], arms["D"]["metrics"][label][key]
            require(c >= 0 and d >= 0, "preregistered gate requires nonnegative metrics")
            checks[label][key] = {
                "C": c,
                "D": d,
                "maximum_D_over_C": multiplier,
                "maximum_D": multiplier * c,
                "observed_D_over_C": d / c if c else None,
                "pass": d <= multiplier * c,
            }
    geometry_checks = {
        kind: {
            "C_pass_count": arms["C"]["geometry"][kind]["pass_count"],
            "D_pass_count": arms["D"]["geometry"][kind]["pass_count"],
            "pass": (arms["D"]["geometry"][kind]["pass_count"] >=
                     arms["C"]["geometry"][kind]["pass_count"]),
        }
        for kind in ("selected", "any_candidate", "all_candidates")
    }
    geometry_pass = all(item["pass"] for item in geometry_checks.values())
    passed = geometry_pass and all(item["pass"] for group in checks.values() for item in group.values())
    return {
        "status": "PASS_SINGLE_SEED_DIAGNOSTIC" if passed else "FAIL_SINGLE_SEED_DIAGNOSTIC",
        "pass": passed,
        "episode_metric_checks": checks,
        "geometry": {"sample_count": 665, "checks": geometry_checks, "pass": geometry_pass},
        "three_seed_confirmation_required": True,
        "adoption_allowed": False,
        "automatic_promotion": False,
    }


def summarize(*, low_root: Path, high_root: Path) -> dict[str, Any]:
    low, high = _load_campaign(low_root, "low"), _load_campaign(high_root, "high")
    # HH_260906 - A two-budget comparison is meaningful only when both runs bind the same source, data and DEV bytes.
    for key in ("source_commit", "source_sha256", "evaluator_sha256", "owner", "gpu",
                "model_config", "model_config_sha256", "model_config_canonical_sha256",
                "model_id", "loss_config", "view_report_sha256", "validation"):
        require(low["plan"].get(key) == high["plan"].get(key), f"cross-budget {key} differs")
    for physical_arm in PHYSICAL_ARMS:
        require(low["plan"]["arms"][physical_arm]["dataset"] ==
                high["plan"]["arms"][physical_arm]["dataset"],
                f"cross-budget dataset differs: {physical_arm}")
    low_train_config = dict(low["plan"]["train_config"])
    require(low_train_config.pop("max_steps", None) == 5740 and
            low_train_config == high["plan"]["train_config"],
            "cross-budget base train config differs")
    val = low["plan"]["validation"]
    for campaign in (low, high):
        for arm in campaign["arms"].values():
            data = arm["dataset"]
            require(data["split_examples_sha256"]["val"] == val["fingerprint_sha256"] and
                    data["split_tokens_sha256"]["val"] == val["tokens_sha256"],
                    "arm validation content/order fingerprint differs")
    arms = {**low["arms"], **high["arms"]}
    require(all(arms[label]["geometry_gate_contract"] == arms["A"]["geometry_gate_contract"]
                for label in "BCD"), "four-arm geometry gate contracts differ")
    slices = ("combined_val", *(spec["label"] for spec in EPISODES))
    raw: dict[str, Any] = {}
    contrasts: dict[str, Any] = {}
    for slice_name in slices:
        metric_keys = set(arms["A"]["metrics"][slice_name])
        require(all(set(arms[label]["metrics"][slice_name]) == metric_keys for label in "BCD"),
                f"four-arm metric keys differ: {slice_name}")
        count = 665 if slice_name == "combined_val" else next(
            spec["count"] for spec in EPISODES if spec["label"] == slice_name)
        raw[slice_name] = {"sample_count": count, "arms": {
            label: arms[label]["metrics"][slice_name] for label in "ABCD"}}
        contrasts[slice_name] = {
            key: _metric_contrasts({label: arms[label]["metrics"][slice_name][key]
                                    for label in "ABCD"})
            for key in sorted(metric_keys)
        }
    geometry_arms = {label: arms[label]["geometry"] for label in "ABCD"}
    geometry_contrasts: dict[str, Any] = {}
    for kind in ("selected", "any_candidate", "all_candidates"):
        values = {label: geometry_arms[label][kind]["pass_count"] for label in "ABCD"}
        rows = _metric_contrasts(values)
        geometry_contrasts[kind] = {
            name: {
                **value,
                "pass_rate_percentage_points": (
                    value["absolute_D_minus_C_minus_B_minus_A"] / 665 * 100.0
                    if name == "interaction" else value["absolute"] / 665 * 100.0
                ),
            }
            for name, value in rows.items()
        }
    recomputed_gate = _cd_gate(high)
    runner_gate = high["summary"].get("comparison", {}).get("diagnostic_gate")
    require(recomputed_gate == runner_gate, "recomputed C/D gate differs from runner summary")
    history = {
        "A_vs_C_same_dataset_seed_prefix": _compare_histories(low, high, "A", "C"),
        "B_vs_D_same_dataset_seed_prefix": _compare_histories(low, high, "B", "D"),
    }
    provenance = {
        label: {
            "budget": "low" if label in "AB" else "high",
            "physical_arm": arms[label]["physical_arm"],
            "expected_train_state": arms[label]["state"],
            "dataset_manifest_sha256": arms[label]["dataset"]["manifest_sha256"],
            "train_fingerprint_sha256": arms[label]["dataset"]["split_fingerprint_sha256"]["train"],
            "val_fingerprint_sha256": arms[label]["dataset"]["split_fingerprint_sha256"]["val"],
            **arms[label]["provenance"],
        }
        for label in "ABCD"
    }
    return {
        "schema": SCHEMA,
        "status": "COMPLETE_NOT_PROMOTED",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "inputs": {
            "low_budget": {"root": str(low["root"]), "campaign_id": CAMPAIGNS["low"],
                           "plan_sha256": low["plan_sha256"], "summary_sha256": low["summary_sha256"]},
            "high_budget": {"root": str(high["root"]), "campaign_id": CAMPAIGNS["high"],
                            "plan_sha256": high["plan_sha256"], "summary_sha256": high["summary_sha256"]},
        },
        "validation": {
            "sample_count": 665,
            "episodes": list(EPISODES),
            "examples_sha256": val["fingerprint_sha256"],
            "tokens_sha256": val["tokens_sha256"],
            "tensor_fingerprint_sha256": arms["A"]["dataset"]["split_fingerprint_sha256"]["val"],
            "identical_across_all_four_arms": True,
        },
        "arm_provenance": provenance,
        "raw_metrics": raw,
        "contrasts": {
            "definition": {
                "absolute_interaction": "(D-C)-(B-A)",
                "percent": "100*(right-left)/left; null when the left value is zero",
                "percentage_point_interaction": "percent(D versus C)-percent(B versus A)",
                "metric_direction": "These are raw signed differences; lower is generally better for listed error/loss metrics.",
            },
            "by_slice": contrasts,
        },
        "geometry": {
            "sample_count": 665,
            "arms": geometry_arms,
            "contrasts": geometry_contrasts,
            "direction": "Higher pass count/rate is better; geometry plausibility is not driving success.",
        },
        "preregistered_C_D_gate": {
            "runner_summary_exact_match": True,
            "thresholds": EXPECTED_CD_THRESHOLDS,
            "result": recomputed_gate,
        },
        "history_prefix_replay": history,
        "scope": {
            "test_data_read_or_used": False,
            "closed_loop_evaluation": False,
            "model_promotion": False,
            "model_export": False,
            "training_data_approved": False,
            "vehicle_control_approved": False,
            "deployment_approved": False,
            "all_map_claim": False,
            "single_seed_only": True,
            "three_seed_confirmation_required": True,
        },
        "interpretation": (
            "Offline fixed-DEV, single-seed A/B/C/D diagnostic only. TEST was not read or used; "
            "no closed-loop behavior, promotion, deployment, all-map coverage, or vehicle readiness is established."
        ),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--low-root", "--low-budget-root", dest="low_root", type=Path, required=True)
    parser.add_argument("--high-root", "--high-budget-root", dest="high_root", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    args = parser.parse_args(argv)
    result = summarize(low_root=args.low_root, high_root=args.high_root)
    write_new(args.output_json, result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
