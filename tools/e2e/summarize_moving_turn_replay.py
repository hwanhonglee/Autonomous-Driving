#!/usr/bin/env python3
"""HH_260906 - Independently verify the sealed C-versus-E turn-replay experiment."""
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


SCHEMA = "portable_e2e.moving_turn_replay_comparison.v1"
REFERENCE_PLAN_SCHEMA = "portable_e2e.moving_data_ab_seed_pilot_plan.v1"
REFERENCE_SUMMARY_SCHEMA = "portable_e2e.moving_data_high_budget_ab_pilot.v1"
EXPERIMENT_PLAN_SCHEMA = "portable_e2e.moving_turn_replay_seed_pilot_plan.v1"
EXPERIMENT_SUMMARY_SCHEMA = "portable_e2e.moving_turn_replay_seed_pilot_summary.v1"
REFERENCE_SUBGROUP_SCHEMA = "portable_e2e.moving_data_ab_subgroup_evaluation.v1"
EXPERIMENT_SUBGROUP_SCHEMA = "portable_e2e.moving_turn_replay_subgroup_evaluation.v1"
REFERENCE_CAMPAIGN = "hh260921-moving-data-high-budget-ab-seed20260906-v1"
EXPERIMENT_CAMPAIGN = "hh260921-moving-turn-replay-seed20260906-v1"
REFERENCE_ARM = "A_baseline_train"
BASE_DATA_ARM = "B_town07_augmented_train"
EXPERIMENT_ARM = "E_turn_replay_train"
STAGES = ("train", "evaluate", "audit", "subgroup")
STAGE_PATHS = {
    "train": "training/run.json",
    "evaluate": "evaluation/metrics.json",
    "audit": "gate_v8.json",
    "subgroup": "subgroup/summary.json",
}
EPISODES = (
    {
        "label": "town03_right_turn_dev",
        "episode_id": "carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9",
        "count": 337,
    },
    {
        "label": "town03_straight_dev",
        "episode_id": "carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320",
        "count": 328,
    },
)
EXPECTED_REFERENCE_STATE = {
    "epoch": 25,
    "next_batch_index": 205,
    "global_step": 7380,
    "samples_seen": 29495,
    "domain_samples_seen": {"carla": 29495},
}
EXPECTED_EXPERIMENT_STATE = {
    "epoch": 20,
    "next_batch_index": 0,
    "global_step": 9020,
    "samples_seen": 36040,
    "domain_samples_seen": {"carla": 36040},
}
EXPECTED_COHORT_CRITERION = {
    "split": "train",
    "field": "target_yaw_rad",
    "reduction": "max_abs_non_null",
    "comparison": "strictly_greater_than",
    "threshold_rad": 0.2,
}
EXPECTED_OVERLAY_PATHS = ["portable_e2e/evaluate.py", "portable_e2e/train.py"]
EXPECTED_ADDED_SOURCE_PATHS = ["scripts/e2e/evaluate_portable_moving_data_subgroups.py"]
EXPECTED_ADDED_SOURCE_SHA256 = {
    "scripts/e2e/evaluate_portable_moving_data_subgroups.py":
        "eba4f7ef276f9a70a1b4857b4ccabccdc9889dea563ba898bb4713563f7a17b0",
}
EXPECTED_RUNNER_SHA256 = "46ce391a5c50712bc799c0cc43e83c185e425048b5d9ff07f80a1358bc2ed129"
EXPECTED_EVALUATOR_SHA256 = "a1285542bcde1d404fe500dcdb5ef8996637c9e4bf4ec89d672e2117751d162b"
EXPECTED_OVERLAY_SHA256 = {
    "portable_e2e/train.py": "d524f0b85346e071f332d9da9e86092b2c534a2f750cfb37d28267afaae0e89b",
    "portable_e2e/evaluate.py": "0ad07ed6715bbb3884a31716293197ad807aabd07ae447a843f854d4fab589df",
}
EXPECTED_MEMBERSHIP_TOKEN_SHA256 = (
    "eb59110e78718fe6c1d79aa233b550d1bea487ba929264ad30d9eb929575be2f"
)
EXPECTED_MEMBERSHIP_INDEX_TOKEN_SHA256 = (
    "d8f80fa3d19cb46fb97b17d1480371996ef11071e0005824c6cd0512d3ea7bde"
)
EXPECTED_COHORT_COUNTS = {
    "base_sample_count": 1475,
    "membership_sample_count": 327,
    "replay_occurrence_count": 327,
    "epoch_sample_count": 1802,
    "batches_per_epoch": 451,
    "completed_epochs": 20,
    "optimizer_steps": 9020,
    "base_exposures": 29500,
    "replay_exposures": 6540,
    "total_exposures": 36040,
}
EXPECTED_GATE_MULTIPLIERS = {
    "town03_right_turn_dev": {
        "ade_6p4s_m": 1.0,
        "fde_6p4s_m": 1.0,
        "speed_mae_6p4s_mps": 1.05,
    },
    "town03_straight_dev": {
        "ade_6p4s_m": 0.90,
        "fde_6p4s_m": 0.90,
    },
}
EXPECTED_REFERENCE_METRICS = {
    "town03_right_turn_dev": {
        "ade_6p4s_m": 2.9353143748791353,
        "fde_6p4s_m": 7.793866114022117,
        "speed_mae_6p4s_mps": 1.1076154233086357,
    },
    "town03_straight_dev": {
        "ade_6p4s_m": 8.090865013076037,
        "fde_6p4s_m": 19.32532976894844,
    },
}
EXPECTED_ABSOLUTE_MAXIMUM_E = {
    "town03_right_turn_dev": {
        "ade_6p4s_m": 2.9353143748791353,
        "fde_6p4s_m": 7.793866114022117,
        "speed_mae_6p4s_mps": 1.1629961944740677,
    },
    "town03_straight_dev": {
        "ade_6p4s_m": 7.281778511768434,
        "fde_6p4s_m": 17.392796792053595,
    },
}
EXPECTED_GEOMETRY_MINIMUM = {
    "selected": 664,
    "any_candidate": 664,
    "all_candidates": 664,
}
DENIAL_FIELDS = (
    "resume",
    "automatic_retry",
    "test_read_or_use",
    "model_export",
    "automatic_promotion",
    "training_data_approved",
    "vehicle_control_approved",
    "closed_loop_claim",
    "all_map_claim",
    "deployment_approved",
)


class SummaryError(ValueError):
    """HH_260906 - Reject incomplete, mutable, or provenance-incompatible evidence."""


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


def _list(value: Any, context: str) -> list[Any]:
    require(type(value) is list, f"{context} must be an array")
    return value


def _integer(value: Any, context: str, minimum: int = 0) -> int:
    require(type(value) is int and value >= minimum, f"{context} must be an integer >= {minimum}")
    return value


def _number(value: Any, context: str) -> float | int:
    require(type(value) in (int, float) and math.isfinite(value),
            f"{context} must be a finite JSON number")
    return value


def _sha256_text(value: Any, context: str) -> str:
    require(type(value) is str and re.fullmatch(r"[0-9a-f]{64}", value) is not None,
            f"{context} must be a lowercase SHA-256")
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


def _canonical_sha256(value: Any) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    return hashlib.sha256(payload).hexdigest()


def write_new(path: Path, value: dict[str, Any]) -> None:
    # HH_260906 - Evidence output is create-only so a second invocation cannot replace a verdict.
    target = Path(path).expanduser().absolute()
    require(".." not in target.parts and not target.exists() and not target.is_symlink(),
            "output JSON must be fresh")
    parent = _canonical_directory(target.parent, "output parent")
    with (parent / target.name).open("xb") as stream:
        stream.write(_encoded(value))
        stream.flush()
        os.fsync(stream.fileno())


def _verify_denials(value: dict[str, Any], context: str) -> None:
    for field in DENIAL_FIELDS:
        require(value.get(field) is False, f"{context}.{field} must be false")


def _dataset(value: Any, expected_train: int, context: str) -> dict[str, Any]:
    data = _mapping(value, context)
    require(data.get("counts") == {"train": expected_train, "val": 665},
            f"{context} counts differ")
    for field in ("manifest_sha256", "corpus_fingerprint_sha256"):
        _sha256_text(data.get(field), f"{context}.{field}")
    for field in ("split_fingerprint_sha256", "split_loader_sha256",
                  "split_examples_sha256", "split_tokens_sha256"):
        rows = _mapping(data.get(field), f"{context}.{field}")
        require(set(rows) == {"train", "val"}, f"{context}.{field} split set differs")
        for split in ("train", "val"):
            _sha256_text(rows[split], f"{context}.{field}.{split}")
    episodes = _mapping(data.get("episode_ids"), f"{context}.episode_ids")
    require(set(episodes) == {"train", "val"} and
            episodes["val"] == [item["episode_id"] for item in EPISODES] and
            type(episodes["train"]) is list and episodes["train"],
            f"{context} episode identities differ")
    require(not any("test" in str(key).lower() for key in data),
            f"{context} must not contain a TEST split field")
    return data


def _stage_index(root: Path, summary: dict[str, Any], arms: tuple[str, ...]) -> dict[tuple[str, str], dict[str, Any]]:
    rows = _list(summary.get("stages"), "summary.stages")
    require(len(rows) == len(arms) * len(STAGES), "summary stage count differs")
    plan_sha = digest(root / "plan.json")
    result: dict[tuple[str, str], dict[str, Any]] = {}
    for raw in rows:
        row = _mapping(raw, "summary stage")
        arm, stage = row.get("arm"), row.get("stage")
        require(arm in arms and stage in STAGES and (arm, stage) not in result,
                "summary stage identity differs")
        relative = f"{arm}/{STAGE_PATHS[stage]}"
        require(row.get("status") == "COMPLETE" and row.get("plan_sha256") == plan_sha and
                row.get("path") == relative and row.get("sha256") == digest(root / relative),
                f"stage receipt provenance differs: {arm}/{stage}")
        require(read_json(root / arm / f"{stage}_completed.json") == row,
                f"stage completion receipt differs: {arm}/{stage}")
        result[(arm, stage)] = row
    return result


def _finite_metrics(value: Any, counts_value: Any, expected_count: int,
                    context: str) -> dict[str, float | int]:
    metrics = _mapping(value, context + ".metrics")
    counts = _mapping(counts_value, context + ".metric_counts")
    require(metrics and set(metrics) == set(counts), f"{context} metric/count keys differ")
    result: dict[str, float | int] = {}
    for key in sorted(metrics):
        require(type(key) is str and key, f"{context} metric name differs")
        result[key] = _number(metrics[key], f"{context}.{key}")
        require(counts[key] == expected_count, f"{context}.{key} denominator differs")
    return result


def _episode_rows(summary: dict[str, Any], context: str) -> dict[str, dict[str, Any]]:
    raw = summary.get("episodes")
    rows = list(raw.values()) if type(raw) is dict else raw
    require(type(rows) is list, f"{context}.episodes must be an object or array")
    result: dict[str, dict[str, Any]] = {}
    for raw_row in rows:
        row = _mapping(raw_row, f"{context}.episode")
        label = next((item["label"] for item in EPISODES
                      if item["episode_id"] == row.get("episode_id")), None)
        require(label is not None and label not in result, f"{context} episode identity differs")
        result[label] = row
    require(set(result) == {item["label"] for item in EPISODES},
            f"{context} fixed episode set differs")
    return result


def _geometry(audit: dict[str, Any], context: str) -> tuple[dict[str, dict[str, float | int]], dict[str, Any]]:
    geometry = _mapping(audit.get("geometry"), context + ".geometry")
    selected = _mapping(geometry.get("selected_result"), context + ".selected_result")
    count = geometry.get("sample_count")
    require(count == selected.get("sample_count") == 665, f"{context} denominator differs")
    values = {
        "selected": selected.get("geometry_pass_count"),
        "any_candidate": geometry.get("any_candidate_geometry_pass_count"),
        "all_candidates": geometry.get("all_candidates_geometry_pass_count"),
    }
    result: dict[str, dict[str, float | int]] = {}
    for name, passed in values.items():
        require(type(passed) is int and 0 <= passed <= 665, f"{context}.{name} count differs")
        result[name] = {"pass_count": passed, "sample_count": 665, "pass_rate": passed / 665}
    return result, _mapping(audit.get("gate"), context + ".gate")


def _read_history(path: Path, expected_state: dict[str, Any], *, verify_streams: bool,
                  execution_schedule: dict[str, Any] | None = None) -> dict[str, Any]:
    line_count = 0
    last: dict[str, Any] | None = None
    stream_totals = {"base": 0, "replay": 0}
    per_epoch: dict[int, dict[str, int]] = {}
    with _regular(path, "training history").open("rb") as stream:
        for line_number, line in enumerate(stream, 1):
            require(line.endswith(b"\n") and line.strip(), "history line framing differs")
            try:
                row = _mapping(json.loads(
                    line.decode("utf-8"), object_pairs_hook=_pairs,
                    parse_constant=lambda value: require(False, f"nonfinite JSON value: {value}"),
                ), f"history line {line_number}")
            except (UnicodeError, json.JSONDecodeError) as error:
                raise SummaryError(f"invalid strict JSONL at line {line_number}: {error}") from error
            require(row.get("global_step") == line_number, "history global_step is not contiguous")
            require(row.get("samples_seen") == sum(
                _mapping(row.get("domain_samples_seen"), "history domain samples").values()),
                "history domain/sample totals differ")
            if verify_streams:
                batch = _mapping(row.get("batch_sampling_stream_counts"), "history batch stream counts")
                cumulative = _mapping(row.get("sampling_stream_samples_seen"),
                                      "history cumulative stream counts")
                require(set(batch) == set(cumulative) == {"base", "replay"},
                        "history stream keys differ")
                for name in stream_totals:
                    stream_totals[name] += _integer(batch[name], f"history batch {name}")
                require(cumulative == stream_totals and row["samples_seen"] == sum(stream_totals.values()),
                        "history stream exposure accounting differs")
                epoch = _integer(row.get("epoch"), "history epoch")
                epoch_counts = per_epoch.setdefault(epoch, {"base": 0, "replay": 0, "steps": 0})
                batch_index = epoch_counts["steps"]
                require(execution_schedule is not None, "history execution schedule is missing")
                schedule_rows = execution_schedule["rows"]
                start = epoch * 1802 + batch_index * 4
                stop = min(epoch * 1802 + 1802, start + 4)
                expected_batch = {"base": 0, "replay": 0}
                for occurrence in schedule_rows[start:stop]:
                    expected_batch[occurrence["stream"]] += 1
                require(batch == expected_batch,
                        "history batch stream counts differ from the concrete execution schedule")
                epoch_counts["steps"] += 1
                for name in stream_totals:
                    epoch_counts[name] += batch[name]
            line_count = line_number
            last = row
    require(line_count == expected_state["global_step"] and last is not None and
            last.get("samples_seen") == expected_state["samples_seen"] and
            last.get("domain_samples_seen") == expected_state["domain_samples_seen"],
            "training history endpoint differs")
    if verify_streams:
        require(set(per_epoch) == set(range(EXPECTED_COHORT_COUNTS["completed_epochs"])),
                "history epoch identities differ")
        expected_epoch = {
            "base": EXPECTED_COHORT_COUNTS["base_sample_count"],
            "replay": EXPECTED_COHORT_COUNTS["replay_occurrence_count"],
            "steps": EXPECTED_COHORT_COUNTS["batches_per_epoch"],
        }
        require(all(row == expected_epoch for row in per_epoch.values()),
                "history per-epoch stream exposure differs")
        require(stream_totals == {
            "base": EXPECTED_COHORT_COUNTS["base_exposures"],
            "replay": EXPECTED_COHORT_COUNTS["replay_exposures"],
        }, "history final stream exposure differs")
    return {
        "sha256": digest(path),
        "line_count": line_count,
        "terminal_record": last,
        "sampling_stream_samples_seen": stream_totals if verify_streams else None,
        "per_epoch_exact": verify_streams,
    }


def _verify_cohort_plan(plan: Any, context: str) -> dict[str, Any]:
    sampling = _mapping(plan, context)
    expected_top_fields = {
        "policy", "order_algorithm", "seed_derivation", "known_domains", "domain_ratios",
        "replacement", "cross_stream_sample_reuse", "dataset_domain_counts",
        "epoch_domain_sample_counts", "epoch_replayed_domain_sample_counts",
        "epoch_discarded_sample_counts", "epoch_sample_count", "batch_size",
        "batches_per_epoch", "cohort_replay",
    }
    require(set(sampling) == expected_top_fields, f"{context} fields differ")
    require(sampling.get("policy") == "cohort_replay" and
            sampling.get("order_algorithm") == "smooth_proportional_cohort_replay_interleave_v1" and
            sampling.get("seed_derivation") == "sha256_seed_epoch_named_stream_v1" and
            sampling.get("known_domains") == ["carla", "real"] and
            sampling.get("domain_ratios") == {} and
            sampling.get("replacement") is False and
            sampling.get("cross_stream_sample_reuse") is True,
            f"{context} policy differs")
    require(sampling.get("dataset_domain_counts") == {"carla": 1475} and
            sampling.get("epoch_domain_sample_counts") == {"carla": 1802} and
            sampling.get("epoch_replayed_domain_sample_counts") == {"carla": 327} and
            sampling.get("epoch_discarded_sample_counts") == {"carla": 0} and
            sampling.get("epoch_sample_count") == EXPECTED_COHORT_COUNTS["epoch_sample_count"] and
            sampling.get("batch_size") == 4 and
            sampling.get("batches_per_epoch") == EXPECTED_COHORT_COUNTS["batches_per_epoch"],
            f"{context} domain/epoch accounting differs")
    cohort = _mapping(sampling.get("cohort_replay"), context + ".cohort_replay")
    require(set(cohort) == {
        "schema", "criterion", "occurrence_order_hash_encoding", "replay_copies_per_epoch",
        "base_sample_count", "membership_sample_count", "membership_sample_tokens_sha256",
        "membership_index_token_sha256", "replay_occurrence_count", "base_occurrences_sha256",
        "base_occurrences", "replay_occurrences_sha256", "replay_occurrences",
    }, f"{context} cohort fields differ")
    require(cohort.get("schema") == "portable_e2e.cohort_replay.v1" and
            cohort.get("criterion") == EXPECTED_COHORT_CRITERION and
            cohort.get("occurrence_order_hash_encoding") ==
            "sha256_canonical_json_array_v1" and
            cohort.get("replay_copies_per_epoch") == 1,
            f"{context} cohort criterion differs")
    for key in ("base_sample_count", "membership_sample_count", "replay_occurrence_count"):
        require(cohort.get(key) == EXPECTED_COHORT_COUNTS[key], f"{context}.{key} differs")
    base = _list(cohort.get("base_occurrences"), context + ".base_occurrences")
    replay = _list(cohort.get("replay_occurrences"), context + ".replay_occurrences")
    require(len(base) == 1475 and len(replay) == 327, f"{context} occurrence row count differs")
    base_by_index: dict[int, dict[str, Any]] = {}
    for position, raw in enumerate(base):
        row = _mapping(raw, f"{context}.base_occurrences[{position}]")
        require(set(row) == {"base_occurrence_index", "dataset_index", "sample_token", "domain"} and
                row["base_occurrence_index"] == position and row["dataset_index"] == position and
                type(row["sample_token"]) is str and row["sample_token"] and row["domain"] == "carla",
                f"{context} base occurrence differs at {position}")
        require(row["dataset_index"] not in base_by_index, f"{context} duplicate base dataset index")
        base_by_index[row["dataset_index"]] = row
    require(len({row["sample_token"] for row in base}) == 1475,
            f"{context} base tokens are not unique")
    for position, raw in enumerate(replay):
        row = _mapping(raw, f"{context}.replay_occurrences[{position}]")
        require(set(row) == {"replay_occurrence_index", "dataset_index", "sample_token", "domain"} and
                row["replay_occurrence_index"] == position and row["dataset_index"] in base_by_index and
                row["sample_token"] == base_by_index[row["dataset_index"]]["sample_token"] and
                row["domain"] == base_by_index[row["dataset_index"]]["domain"],
                f"{context} replay occurrence differs at {position}")
    require(len({row["dataset_index"] for row in replay}) == 327 and
            len({row["sample_token"] for row in replay}) == 327,
            f"{context} replay membership is not unique")
    token_payload = [row["sample_token"] for row in replay]
    index_token_payload = [
        {"dataset_index": row["dataset_index"], "token": row["sample_token"]}
        for row in replay
    ]
    computed = {
        "membership_sample_tokens_sha256": _canonical_sha256(token_payload),
        "membership_index_token_sha256": _canonical_sha256(index_token_payload),
        "base_occurrences_sha256": _canonical_sha256(base),
        "replay_occurrences_sha256": _canonical_sha256(replay),
    }
    require(cohort.get("membership_sample_tokens_sha256") ==
            computed["membership_sample_tokens_sha256"] == EXPECTED_MEMBERSHIP_TOKEN_SHA256,
            f"{context} membership token digest differs")
    require(cohort.get("membership_index_token_sha256") ==
            computed["membership_index_token_sha256"] == EXPECTED_MEMBERSHIP_INDEX_TOKEN_SHA256,
            f"{context} membership index/token digest differs")
    for key in ("base_occurrences_sha256", "replay_occurrences_sha256"):
        require(cohort.get(key) == computed[key], f"{context} {key} differs")
    return {
        "sampling_plan_sha256": _canonical_sha256(sampling),
        "criterion": EXPECTED_COHORT_CRITERION,
        "counts": dict(EXPECTED_COHORT_COUNTS),
        "membership_sample_tokens_sha256": computed["membership_sample_tokens_sha256"],
        "membership_index_token_sha256": computed["membership_index_token_sha256"],
        "base_occurrences_sha256": computed["base_occurrences_sha256"],
        "replay_occurrences_sha256": computed["replay_occurrences_sha256"],
    }


def _verify_execution_schedule(value: Any, sampling_plan: dict[str, Any],
                               context: str) -> dict[str, Any]:
    schedule = _mapping(value, context)
    require(set(schedule) == {"schema", "epoch_count", "epoch_sample_count", "row_count",
                              "hash_encoding", "sha256", "rows"} and
            schedule.get("schema") == "portable_e2e.cohort_replay_execution_schedule.v1" and
            schedule.get("epoch_count") == 20 and schedule.get("epoch_sample_count") == 1802 and
            schedule.get("row_count") == 36040 and
            schedule.get("hash_encoding") == "sha256_canonical_json_v1",
            f"{context} contract differs")
    rows = _list(schedule.get("rows"), context + ".rows")
    require(len(rows) == 36040 and schedule.get("sha256") == _canonical_sha256(rows),
            f"{context} row count/digest differs")
    cohort = _mapping(sampling_plan.get("cohort_replay"), context + ".cohort_replay")
    occurrences = {
        "base": _list(cohort.get("base_occurrences"), context + ".base occurrences"),
        "replay": _list(cohort.get("replay_occurrences"), context + ".replay occurrences"),
    }
    per_epoch = {
        epoch: {"base": set(), "replay": set()}
        for epoch in range(EXPECTED_COHORT_COUNTS["completed_epochs"])
    }
    for offset, raw in enumerate(rows):
        row = _mapping(raw, f"{context}.rows[{offset}]")
        epoch, position = divmod(offset, EXPECTED_COHORT_COUNTS["epoch_sample_count"])
        require(set(row) == {"epoch", "position", "stream", "occurrence_index", "dataset_index"} and
                row.get("epoch") == epoch and row.get("position") == position and
                row.get("stream") in occurrences,
                f"{context} row identity differs at {offset}")
        stream = row["stream"]
        occurrence_index = row.get("occurrence_index")
        require(type(occurrence_index) is int and 0 <= occurrence_index < len(occurrences[stream]),
                f"{context} occurrence index differs at {offset}")
        occurrence = occurrences[stream][occurrence_index]
        require(row.get("dataset_index") == occurrence.get("dataset_index") and
                occurrence_index not in per_epoch[epoch][stream],
                f"{context} occurrence binding/uniqueness differs at {offset}")
        per_epoch[epoch][stream].add(occurrence_index)
    require(all(len(streams["base"]) == 1475 and len(streams["replay"]) == 327
                for streams in per_epoch.values()),
            f"{context} does not contain each base/replay occurrence exactly once per epoch")
    return {
        "schema": schedule["schema"],
        "row_count": schedule["row_count"],
        "sha256": schedule["sha256"],
        "canonical_sha256_recomputed": True,
        "epoch_occurrence_bijection_verified": True,
    }


def _load_arm(*, root: Path, plan: dict[str, Any], arm: str, data: dict[str, Any],
              expected_state: dict[str, Any], stages: dict[tuple[str, str], dict[str, Any]],
              expected_train_config: dict[str, Any], verify_streams: bool,
              execution_schedule: dict[str, Any] | None = None) -> dict[str, Any]:
    item = root / arm
    checkpoint = item / "training/checkpoints/latest.pt"
    checkpoint_sha = digest(checkpoint)
    train = read_json(item / "training/run.json")
    require(train.get("status") == "TRAINING_TARGET_REACHED" and
            "resumed_at_utc" not in train and
            train.get("state") == expected_state and train.get("dataset_size") == data["counts"]["train"] and
            train.get("dataset_fingerprint_sha256") == data["split_fingerprint_sha256"]["train"] and
            train.get("corpus_fingerprint_sha256") == data["corpus_fingerprint_sha256"] and
            train.get("train_config") == expected_train_config and
            train.get("loss_config") == plan.get("loss_config") and train.get("device") == "cuda:0" and
            train.get("vehicle_control_approved", False) is False and
            train.get("warning") == "RESEARCH CHECKPOINT — NOT APPROVED FOR VEHICLE CONTROL",
            f"arm {arm} training provenance differs")
    require(train.get("training_episode_ids") == sorted(data["episode_ids"]["train"]),
            f"arm {arm} training episode identities differ")
    history = _read_history(item / "training/metrics.jsonl", expected_state,
                            verify_streams=verify_streams,
                            execution_schedule=execution_schedule)
    train_receipt = stages[(arm, "train")]
    require(train_receipt.get("checkpoint_sha256") == checkpoint_sha and
            train_receipt.get("history_sha256") == history["sha256"],
            f"arm {arm} training receipt differs")
    if verify_streams:
        cohort = _verify_cohort_plan(train.get("sampling_plan"), f"arm {arm} sampling plan")
        require(train.get("sampling_plan_sha256") == cohort["sampling_plan_sha256"] and
                train.get("sampling_metrics", {}).get("sampling_stream_samples_seen") ==
                history["sampling_stream_samples_seen"] and
                train.get("last_metrics", {}).get("sampling_stream_samples_seen") ==
                history["sampling_stream_samples_seen"] and
                execution_schedule is not None and
                train.get("sampling_execution_schedule_row_count") == 36040 and
                train.get("sampling_execution_schedule_sha256") == execution_schedule["sha256"],
                f"arm {arm} sampling plan/metrics binding differs")
    else:
        cohort = None
    evaluation = read_json(item / "evaluation/metrics.json")
    combined = _finite_metrics(evaluation.get("metrics"), evaluation.get("metric_counts"),
                               665, f"arm {arm} combined VAL")
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
            f"arm {arm} evaluation provenance differs")
    if verify_streams:
        require(evaluation.get("training_sampling_plan_sha256") == train["sampling_plan_sha256"] and
                evaluation.get("training_sampling_policy") == "cohort_replay",
                f"arm {arm} evaluation sampling provenance differs")
    subgroup_path = item / "subgroup/summary.json"
    subgroup = read_json(subgroup_path)
    require(subgroup.get("schema") == (EXPERIMENT_SUBGROUP_SCHEMA if verify_streams
                                       else REFERENCE_SUBGROUP_SCHEMA) and
            subgroup.get("status") == "COMPLETE_NOT_PROMOTED" and subgroup.get("arm") == arm and
            subgroup.get("plan_sha256") == digest(root / "plan.json") and
            subgroup.get("evaluator_sha256") == plan.get("evaluator_sha256") and
            subgroup.get("combined_val_count") == 665 and
            subgroup.get("checkpoint_sha256") == checkpoint_sha and
            subgroup.get("corpus_fingerprint_sha256") == data["corpus_fingerprint_sha256"] and
            subgroup.get("test_read_or_use") is False and subgroup.get("closed_loop_evaluation") is False and
            subgroup.get("automatic_promotion") is False and subgroup.get("model_exported") is False and
            subgroup.get("training_executed_by_evaluator") is False and
            subgroup.get("vehicle_control_approved") is False,
            f"arm {arm} subgroup provenance differs")
    if verify_streams:
        require(subgroup.get("base_commit") == plan.get("base_commit") and
                subgroup.get("source") == plan.get("source") and
                subgroup.get("runner_sha256") == plan.get("runner_sha256"),
                f"arm {arm} subgroup overlay provenance differs")
    else:
        require(subgroup.get("source_commit") == plan.get("source_commit") and
                subgroup.get("source_sha256") == plan.get("source_sha256"),
                f"arm {arm} subgroup historical source provenance differs")
    subgroup_data = _mapping(subgroup.get("dataset"), f"arm {arm} subgroup dataset")
    require(subgroup_data.get("combined_val_example_fingerprint_sha256") ==
            data["split_examples_sha256"]["val"] and
            subgroup_data.get("combined_val_fingerprint_sha256") ==
            data["split_fingerprint_sha256"]["val"] and
            subgroup_data.get("train_fingerprint_sha256") ==
            data["split_fingerprint_sha256"]["train"] and
            subgroup_data.get("manifest_sha256") == data["manifest_sha256"] and
            subgroup_data.get("counts") == data["counts"],
            f"arm {arm} subgroup dataset differs")
    subgroup_checkpoint = _mapping(subgroup.get("checkpoint"), f"arm {arm} subgroup checkpoint")
    require(subgroup_checkpoint.get("sha256") == checkpoint_sha and
            subgroup_checkpoint.get("training_dataset_fingerprint_sha256") ==
            data["split_fingerprint_sha256"]["train"] and
            subgroup_checkpoint.get("corpus_fingerprint_sha256") == data["corpus_fingerprint_sha256"] and
            subgroup_checkpoint.get("model_config_sha256") == plan.get("model_config_canonical_sha256"),
            f"arm {arm} subgroup checkpoint differs")
    if verify_streams:
        require(subgroup_checkpoint.get("sampling_plan_sha256") == train["sampling_plan_sha256"],
                f"arm {arm} subgroup sampling-plan binding differs")
    episode_metrics: dict[str, dict[str, float | int]] = {}
    rows = _episode_rows(subgroup, f"arm {arm} subgroup")
    for spec in EPISODES:
        row = rows[spec["label"]]
        require(row.get("count") == spec["count"], f"arm {arm} subgroup count differs")
        episode_metrics[spec["label"]] = _finite_metrics(
            row.get("metrics"), row.get("metric_counts"), spec["count"],
            f"arm {arm} {spec['label']}")
        report_path = row.get("report_path")
        require(type(report_path) is str and not Path(report_path).is_absolute() and
                ".." not in Path(report_path).parts and
                row.get("report_sha256") == digest(item / "subgroup" / report_path),
                f"arm {arm} subgroup report binding differs")
    require(set(combined) == set.intersection(*(set(value) for value in episode_metrics.values())),
            f"arm {arm} combined/subgroup metric keys differ")
    for key, observed in combined.items():
        weighted = sum(episode_metrics[item["label"]][key] * item["count"] for item in EPISODES) / 665
        require(math.isclose(observed, weighted, rel_tol=0.0, abs_tol=1e-6),
                f"arm {arm} combined metric aggregation differs: {key}")
    audit = read_json(item / "gate_v8.json")
    require(audit.get("status") == "RUNTIME_GEOMETRY_AUDIT_COMPLETE" and
            audit.get("evaluation_split") == "val" and audit.get("evaluation_episode_count") == 2 and
            audit.get("checkpoint_sha256") == checkpoint_sha and
            audit.get("corpus_fingerprint_sha256") == data["corpus_fingerprint_sha256"] and
            audit.get("dataset_fingerprint_sha256") == data["split_fingerprint_sha256"]["val"] and
            audit.get("training_dataset_fingerprint_sha256") == data["split_fingerprint_sha256"]["train"] and
            audit.get("model_config_sha256") == plan.get("model_config_canonical_sha256") and
            audit.get("vehicle_control_approved") is False,
            f"arm {arm} geometry audit provenance differs")
    if verify_streams:
        require(audit.get("training_sampling_plan_sha256") == train["sampling_plan_sha256"] and
                audit.get("training_sampling_policy") == "cohort_replay",
                f"arm {arm} audit sampling provenance differs")
    geometry, gate_contract = _geometry(audit, f"arm {arm}")
    require(gate_contract.get("source") == "portable_e2e.runtime_geometry_gate.v8" and
            gate_contract.get("threshold_overrides") is False,
            f"arm {arm} geometry gate contract differs")
    return {
        "metrics": {"combined_val": combined, **episode_metrics},
        "geometry": geometry,
        "geometry_gate_contract": gate_contract,
        "history": history,
        "cohort": cohort,
        "run_sampling_stream_samples_seen": (
            train.get("sampling_metrics", {}).get("sampling_stream_samples_seen")
            if verify_streams else None
        ),
        "run_sampling_execution_schedule_sha256": (
            train.get("sampling_execution_schedule_sha256") if verify_streams else None
        ),
        "provenance": {
            "checkpoint_sha256": checkpoint_sha,
            "training_run_sha256": digest(item / "training/run.json"),
            "history_sha256": history["sha256"],
            "evaluation_sha256": digest(item / "evaluation/metrics.json"),
            "subgroup_sha256": digest(subgroup_path),
            "geometry_gate_sha256": digest(item / "gate_v8.json"),
        },
    }


def _gate(reference: dict[str, Any], experiment: dict[str, Any]) -> dict[str, Any]:
    checks: dict[str, Any] = {}
    for spec in EPISODES:
        label = spec["label"]
        checks[label] = {}
        for metric, multiplier in EXPECTED_GATE_MULTIPLIERS[label].items():
            c = _number(reference["metrics"][label][metric], f"C {label} {metric}")
            e = _number(experiment["metrics"][label][metric], f"E {label} {metric}")
            require(c >= 0 and e >= 0, "diagnostic gate metrics must be nonnegative")
            maximum = EXPECTED_ABSOLUTE_MAXIMUM_E[label][metric]
            require(math.isclose(maximum, c * multiplier, rel_tol=0.0, abs_tol=5e-15),
                    "absolute gate threshold is inconsistent with the declared C-relative multiplier")
            checks[label][metric] = {
                "C_reference": c,
                "maximum_E": maximum,
                "observed_E": e,
                "pass": e <= maximum,
            }
    geometry_checks: dict[str, Any] = {}
    for name, minimum in EXPECTED_GEOMETRY_MINIMUM.items():
        c = reference["geometry"][name]["pass_count"]
        e = experiment["geometry"][name]["pass_count"]
        require(c == minimum, f"C {name} geometry reference count differs")
        geometry_checks[name] = {
            "minimum": minimum,
            "observed_E": e,
            "pass": e >= minimum,
        }
    metric_pass = all(row["pass"] for group in checks.values() for row in group.values())
    geometry_pass = all(row["pass"] for row in geometry_checks.values())
    passed = metric_pass and geometry_pass
    return {
        "status": "PASS_SINGLE_SEED_DIAGNOSTIC" if passed else "FAIL_SINGLE_SEED_DIAGNOSTIC",
        "pass": passed,
        "episode_metric_checks": checks,
        "geometry": {"sample_count": 665, "checks": geometry_checks, "pass": geometry_pass},
        "next_action": ("run_unseen_confirmation_seeds_20260907_20260908_20260909" if passed else
                        "do_not_confirm_or_promote_revise_training_data_or_sampling"),
        "adoption_allowed": False,
        "automatic_promotion": False,
    }


def _delta(reference: dict[str, Any], experiment: dict[str, Any]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for slice_name in ("combined_val", *(item["label"] for item in EPISODES)):
        c_values, e_values = reference["metrics"][slice_name], experiment["metrics"][slice_name]
        require(set(c_values) == set(e_values), f"C/E metric keys differ: {slice_name}")
        result[slice_name] = {
            key: {
                "C": c_values[key],
                "E": e_values[key],
                "absolute_E_minus_C": e_values[key] - c_values[key],
                "percent_of_C": (None if c_values[key] == 0 else
                                 (e_values[key] - c_values[key]) / c_values[key] * 100.0),
            }
            for key in sorted(c_values)
        }
    return result


def summarize(*, reference_root: Path, experiment_root: Path) -> dict[str, Any]:
    reference_root = _canonical_directory(reference_root, "C reference root")
    experiment_root = _canonical_directory(experiment_root, "E experiment root")
    require(not (reference_root / "failed.json").exists(), "C reference root has failed.json")
    require(not (experiment_root / "failed.json").exists(), "E experiment root has failed.json")
    reference_plan = read_json(reference_root / "plan.json")
    reference_summary = read_json(reference_root / "summary.json")
    experiment_plan = read_json(experiment_root / "plan.json")
    experiment_summary = read_json(experiment_root / "summary.json")
    require(reference_plan.get("schema") == REFERENCE_PLAN_SCHEMA and
            reference_plan.get("campaign_id") == REFERENCE_CAMPAIGN and
            reference_summary.get("schema") == REFERENCE_SUMMARY_SCHEMA and
            reference_summary.get("status") == "COMPLETE_NOT_PROMOTED" and
            reference_summary.get("plan_sha256") == digest(reference_root / "plan.json") and
            reference_summary.get("source_and_views_postcheck_pass") is True,
            "C reference campaign identity/completion differs")
    require(experiment_plan.get("schema") == EXPERIMENT_PLAN_SCHEMA and
            experiment_plan.get("campaign_id") == EXPERIMENT_CAMPAIGN and
            experiment_summary.get("schema") == EXPERIMENT_SUMMARY_SCHEMA and
            experiment_summary.get("status") == "COMPLETE_NOT_PROMOTED" and
            experiment_summary.get("plan_sha256") == digest(experiment_root / "plan.json") and
            experiment_summary.get("source_dataset_reference_postcheck_pass") is True,
            "E experiment campaign identity/completion differs")
    _verify_denials(reference_plan, "C plan")
    _verify_denials(reference_summary, "C summary")
    _verify_denials(experiment_plan, "E plan")
    _verify_denials(experiment_summary, "E summary")
    reference_arms = (REFERENCE_ARM, BASE_DATA_ARM)
    require(reference_summary.get("arms") == {arm: "COMPLETE" for arm in reference_arms},
            "C reference campaign arms are incomplete")
    require(experiment_summary.get("arm") == "COMPLETE",
            "E experiment arm is incomplete")
    reference_stages = _stage_index(reference_root, reference_summary, reference_arms)
    experiment_stages = _stage_index(experiment_root, experiment_summary, (EXPERIMENT_ARM,))
    reference_c_plan_arm = _mapping(reference_plan.get("arms", {}).get(REFERENCE_ARM), "C plan arm")
    reference_d_plan_arm = _mapping(reference_plan.get("arms", {}).get(BASE_DATA_ARM), "D plan arm")
    experiment_plan_arm = _mapping(experiment_plan.get("arm"), "E plan arm")
    require(experiment_plan_arm.get("label") == EXPERIMENT_ARM and
            experiment_plan_arm.get("dataset_role") == BASE_DATA_ARM,
            "E arm identity/dataset role differs")
    c_data = _dataset(reference_c_plan_arm.get("dataset"), 1147, "C dataset")
    d_data = _dataset(reference_d_plan_arm.get("dataset"), 1475, "D base dataset")
    e_data = _dataset(experiment_plan_arm.get("dataset"), 1475, "E dataset")
    require(e_data == d_data, "E base dataset differs byte-for-byte from the prior D plan declaration")
    require(reference_c_plan_arm.get("expected_train_state") == EXPECTED_REFERENCE_STATE,
            "C expected train state differs")
    require(experiment_plan_arm.get("expected_train_state") == EXPECTED_EXPERIMENT_STATE,
            "E expected train state differs")
    require(experiment_plan_arm.get("expected_sampling_stream_samples_seen") ==
            {"base": 29500, "replay": 6540},
            "E expected stream exposures differ")
    validation = reference_plan.get("validation")
    require(validation == experiment_plan.get("validation") == {
        "episodes": list(EPISODES),
        "count": 665,
        "fingerprint_sha256": c_data["split_examples_sha256"]["val"],
        "tokens_sha256": c_data["split_tokens_sha256"]["val"],
    }, "C/E validation declaration differs")
    for data in (c_data, d_data, e_data):
        require(data["split_examples_sha256"]["val"] == validation["fingerprint_sha256"] and
                data["split_tokens_sha256"]["val"] == validation["tokens_sha256"] and
                data["split_fingerprint_sha256"]["val"] == c_data["split_fingerprint_sha256"]["val"],
                "C/D/E validation bytes/order/tensor fingerprint differ")
    for key in ("owner", "gpu", "model_config", "model_config_sha256",
                "model_config_canonical_sha256", "model_id", "loss_config",
                "view_report_sha256"):
        require(reference_plan.get(key) == experiment_plan.get(key), f"C/E {key} differs")
    require(experiment_plan.get("base_commit") == reference_plan.get("source_commit"),
            "E base commit differs from the C source commit")
    source = _mapping(experiment_plan.get("source"), "E source")
    base_source = _mapping(source.get("base_sha256"), "E base source hashes")
    current_source = _mapping(source.get("current_sha256"), "E current source hashes")
    added_source = _mapping(source.get("added_sha256"), "E added source hashes")
    require(set(source) == {"base_commit", "base_sha256", "current_sha256", "overlay_paths",
                            "added_paths", "added_sha256"} and
            source.get("base_commit") == experiment_plan["base_commit"] and
            source.get("overlay_paths") == EXPECTED_OVERLAY_PATHS and
            source.get("added_paths") == EXPECTED_ADDED_SOURCE_PATHS and
            set(base_source) == set(current_source) and
            set(added_source) == set(EXPECTED_ADDED_SOURCE_PATHS),
            "E source overlay declaration differs")
    _sha256_text(experiment_plan.get("runner_sha256"), "E runner SHA")
    _sha256_text(experiment_plan.get("evaluator_sha256"), "E evaluator SHA")
    require(experiment_plan["runner_sha256"] == EXPECTED_RUNNER_SHA256 and
            experiment_plan["evaluator_sha256"] == EXPECTED_EVALUATOR_SHA256,
            "E runner/evaluator bytes differ from the reviewed implementation")
    for name in base_source:
        _sha256_text(base_source[name], f"E base source {name}")
        _sha256_text(current_source[name], f"E current source {name}")
        require((base_source[name] != current_source[name]) == (name in EXPECTED_OVERLAY_PATHS),
                f"E source overlay byte distinction differs: {name}")
    for name, value in added_source.items():
        _sha256_text(value, f"E added source {name}")
    require(added_source == EXPECTED_ADDED_SOURCE_SHA256 and
            all(current_source[name] == value for name, value in EXPECTED_OVERLAY_SHA256.items()),
            "E reviewed overlay/added source bytes differ")
    archive = experiment_root / "source"
    for name, expected in {**current_source, **added_source}.items():
        require(digest(archive / name) == expected, f"E archived source differs: {name}")
    require(digest(archive / "run_moving_turn_replay_seed_pilot.py") ==
            experiment_plan["runner_sha256"] and
            digest(archive / "evaluate_portable_moving_turn_replay_subgroups.py") ==
            experiment_plan["evaluator_sha256"] and
            digest(archive / "profile_portable_training.py") ==
            experiment_plan.get("owner", {}).get("sha256") and
            digest(experiment_root / "view_report.json") == experiment_plan["view_report_sha256"],
            "E archived runner/evaluator/owner/view report provenance differs")
    c_sources = _mapping(reference_plan.get("source_sha256"), "C source hashes")
    require(set(c_sources) == set(base_source) and
            all(c_sources[name] == base_source[name] for name in c_sources),
            "E base source bytes differ from the C historical source closure")
    c_train_config = _mapping(reference_c_plan_arm.get("train_config"), "C train config")
    e_train_config = _mapping(experiment_plan_arm.get("train_config"), "E train config")
    comparable_c = dict(c_train_config)
    comparable_e = dict(e_train_config)
    require(comparable_c.pop("max_steps", None) == 7380 and
            comparable_e.pop("max_steps", None) == 9020 and
            comparable_c.pop("sampling_policy", None) == "uniform_without_replacement" and
            comparable_e.pop("sampling_policy", None) == "cohort_replay" and
            comparable_c.pop("checkpoint_interval", None) == 154 and
            comparable_e.pop("checkpoint_interval", None) == 451 and
            comparable_c == comparable_e,
            "C/E train config differs beyond the preregistered budget/sampler/checkpoint interval")
    require(experiment_plan.get("train_config") == e_train_config,
            "E top-level and arm train configs differ")
    expected_sampling_plan = experiment_plan_arm.get("sampling_plan")
    planned_cohort = _verify_cohort_plan(expected_sampling_plan, "E planned sampling plan")
    require(experiment_plan_arm.get("sampling_plan_sha256") ==
            planned_cohort["sampling_plan_sha256"], "E planned sampling-plan digest differs")
    schedule = _mapping(experiment_plan_arm.get("execution_schedule"), "E execution schedule")
    schedule_proof = _verify_execution_schedule(schedule, expected_sampling_plan,
                                                "E execution schedule")
    reference = _load_arm(
        root=reference_root, plan=reference_plan, arm=REFERENCE_ARM, data=c_data,
        expected_state=EXPECTED_REFERENCE_STATE, stages=reference_stages,
        expected_train_config=c_train_config, verify_streams=False,
    )
    experiment = _load_arm(
        root=experiment_root, plan=experiment_plan, arm=EXPERIMENT_ARM, data=e_data,
        expected_state=EXPECTED_EXPERIMENT_STATE, stages=experiment_stages,
        expected_train_config=e_train_config, verify_streams=True,
        execution_schedule=schedule,
    )
    require(experiment["cohort"]["sampling_plan_sha256"] == planned_cohort["sampling_plan_sha256"] and
            read_json(experiment_root / EXPERIMENT_ARM / "training/run.json").get("sampling_plan") ==
            expected_sampling_plan,
            "E planned and executed sampling plans differ")
    require(reference["geometry_gate_contract"] == experiment["geometry_gate_contract"],
            "C/E geometry gate contracts differ")
    computed_gate = _gate(reference, experiment)
    decision = _mapping(experiment_plan.get("decision"), "E decision")
    expected_thresholds = EXPECTED_ABSOLUTE_MAXIMUM_E
    reference_binding = _mapping(experiment_plan.get("reference_C"), "E reference C binding")
    reference_root_text = reference_binding.get("root")
    require(type(reference_root_text) is str and reference_root_text and
            not Path(reference_root_text).is_absolute() and
            ".." not in Path(reference_root_text).parts,
            "E reference C root must be a safe relative path")
    reference_metric_subset = {
        label: {key: reference["metrics"][label][key]
                for key in EXPECTED_GATE_MULTIPLIERS[label]}
        for label in EXPECTED_GATE_MULTIPLIERS
    }
    require(reference_metric_subset == EXPECTED_REFERENCE_METRICS,
            "C reference gate metrics differ from the sealed high-budget result")
    require(reference_binding.get("campaign_id") == REFERENCE_CAMPAIGN and
            reference_binding.get("arm") == REFERENCE_ARM and
            reference_binding.get("metrics") == reference_metric_subset and
            reference_binding.get("geometry_pass_counts") == EXPECTED_GEOMETRY_MINIMUM,
            "E plan reference-C metrics/geometry binding differs")
    expected_reference_paths = {
        "plan": "plan.json",
        "summary": "summary.json",
        "checkpoint": f"{REFERENCE_ARM}/training/checkpoints/latest.pt",
        "training_run": f"{REFERENCE_ARM}/training/run.json",
        "subgroup": f"{REFERENCE_ARM}/subgroup/summary.json",
        "gate": f"{REFERENCE_ARM}/gate_v8.json",
    }
    files = _mapping(reference_binding.get("files"), "E reference C files")
    require(set(files) == set(expected_reference_paths), "E reference C file set differs")
    for name, relative in expected_reference_paths.items():
        row = _mapping(files[name], f"E reference C file {name}")
        require(row == {"path": relative, "sha256": digest(reference_root / relative)},
                f"E reference C file binding differs: {name}")
    require(decision.get("reference_arm") == "C_old_train_7380_steps" and
            decision.get("experimental_arm") == EXPERIMENT_ARM and
            decision.get("single_seed_exploratory_only") is True and
            decision.get("final_checkpoint_only") is True and
            decision.get("fresh_initialization") is True and
            decision.get("no_early_stopping_or_checkpoint_selection") is True and
            decision.get("maximum_E") == expected_thresholds and
            decision.get("minimum_geometry_pass_counts") == EXPECTED_GEOMETRY_MINIMUM and
            decision.get("unseen_confirmation_seeds") == [20260907, 20260908, 20260909] and
            decision.get("confirmation_runs_allowed_only_after_seed20260906_pass") is True and
            decision.get("adoption_allowed") is False,
            "E preregistered decision differs")
    runner_gate = experiment_summary.get("diagnostic_gate")
    require(runner_gate == computed_gate, "independently recomputed E/C gate differs from runner summary")
    return {
        "schema": SCHEMA,
        "status": "COMPLETE_NOT_PROMOTED",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "inputs": {
            "C_reference": {
                "root": str(reference_root),
                "campaign_id": REFERENCE_CAMPAIGN,
                "plan_sha256": digest(reference_root / "plan.json"),
                "summary_sha256": digest(reference_root / "summary.json"),
            },
            "E_experiment": {
                "root": str(experiment_root),
                "campaign_id": EXPERIMENT_CAMPAIGN,
                "plan_sha256": digest(experiment_root / "plan.json"),
                "summary_sha256": digest(experiment_root / "summary.json"),
            },
        },
        "validation": {
            "sample_count": 665,
            "episodes": list(EPISODES),
            "examples_sha256": validation["fingerprint_sha256"],
            "tokens_sha256": validation["tokens_sha256"],
            "tensor_fingerprint_sha256": c_data["split_fingerprint_sha256"]["val"],
            "identical_across_C_and_E": True,
        },
        "cohort_and_exposure_verification": {
            **planned_cohort,
            "execution_schedule": schedule_proof,
            "plan_equals_executed_sampling_plan": True,
            "history_line_count": experiment["history"]["line_count"],
            "history_per_epoch_exact": experiment["history"]["per_epoch_exact"],
            "history_sampling_stream_samples_seen":
                experiment["history"]["sampling_stream_samples_seen"],
            "run_manifest_sampling_stream_samples_seen":
                experiment["run_sampling_stream_samples_seen"],
            "run_manifest_execution_schedule_sha256":
                experiment["run_sampling_execution_schedule_sha256"],
            "exact_exposure_accounting_pass": True,
        },
        "arm_provenance": {
            "C": {
                "label": "C_old_train_7380_steps",
                "dataset_manifest_sha256": c_data["manifest_sha256"],
                "train_fingerprint_sha256": c_data["split_fingerprint_sha256"]["train"],
                **reference["provenance"],
            },
            "E": {
                "label": "E_turn_replay_train_9020_steps",
                "dataset_manifest_sha256": e_data["manifest_sha256"],
                "train_fingerprint_sha256": e_data["split_fingerprint_sha256"]["train"],
                **experiment["provenance"],
            },
        },
        "metrics_and_deltas": _delta(reference, experiment),
        "geometry": {
            "sample_count": 665,
            "C": reference["geometry"],
            "E": experiment["geometry"],
        },
        "preregistered_C_E_gate": {
            "runner_summary_exact_match": True,
            "multipliers": EXPECTED_GATE_MULTIPLIERS,
            "absolute_thresholds_from_C": expected_thresholds,
            "result": computed_gate,
        },
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
            "Offline fixed-DEV, single-seed C-versus-E diagnostic only. TEST was not read or used; "
            "no closed-loop behavior, promotion, export, deployment, all-map coverage, or vehicle "
            "readiness is established even when the diagnostic gate passes."
        ),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--reference-root", "--c-reference-root", dest="reference_root",
                        type=Path, required=True)
    parser.add_argument("--experiment-root", "--e-experiment-root", dest="experiment_root",
                        type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    args = parser.parse_args(argv)
    result = summarize(reference_root=args.reference_root, experiment_root=args.experiment_root)
    write_new(args.output_json, result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
