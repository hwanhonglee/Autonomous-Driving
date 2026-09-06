#!/usr/bin/env python3
"""Fail-closed aggregation for repeated owned CARLA route trials."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import statistics
import tempfile
from typing import Any, Mapping, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


SCHEMA_ID = "autoware-e2e.owned-route-repeat-summary.v1"
SHADOW_SCHEMA_ID = "autoware-e2e.portable-shadow-evidence-analysis.v2"
TEN_HZ_STATUS = "ESTABLISHED_FOR_SHADOW_ONLY"
SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")

LIMITS = {
    "maximum_cte_m": 1.0,
    "maximum_trajectory_correction_m": 15.0,
    "maximum_lateral_acceleration_mps2": 1.8,
    "minimum_rate_hz": 9.5,
    "maximum_rate_hz": 10.5,
    "maximum_inference_p99_ms": 100.0,
    "maximum_status_wall_gap_s": 0.2,
}

CHECK_REQUIREMENTS = {
    "owner_pass": "owned_trial_summary must be schema-1 PASS",
    "selected_attempt_unambiguous": (
        "selected_attempt must identify exactly one contained attempt directory"
    ),
    "selected_attempt_pass": (
        "selected attempt must record process exit 0 and runtime-health PASS"
    ),
    "exact_route_sha256": (
        "owner and selected source route SHA-256 must equal the expected route SHA-256"
    ),
    "goal_reached": "result success must be true and reason must be goal reached",
    "cte_limit": "maximum absolute CTE must be <= 1.0 m",
    "trajectory_correction_limit": (
        "maximum trajectory correction must be <= 15.0 m"
    ),
    "lateral_acceleration_limit": (
        "maximum lateral acceleration must be <= 1.8 m/s^2"
    ),
    "runtime_health_pass": "runtime_health must be schema-1 PASS",
    "diagnosis_valid": "diagnosis must be a complete schema-2 JSON object",
    "shadow_evidence_valid": (
        "Portable shadow analysis must use the expected schema and be EVIDENCE_VALID"
    ),
    "shadow_ten_hz_pass": (
        "Portable shadow ten-Hz claim must pass for shadow-only scope"
    ),
    "source_rate": "source anchor rate must be within [9.5, 10.5] Hz",
    "path_rate": "shadow path receipt rate must be within [9.5, 10.5] Hz",
    "trajectory_rate": (
        "shadow trajectory receipt rate must be within [9.5, 10.5] Hz"
    ),
    "source_period_continuity": (
        "source period gate must pass with zero adjacent-period violations"
    ),
    "inference_latency": "accepted inference p99 must be <= 100 ms",
    "status_wall_gap": "maximum accepted status wall gap must be <= 0.2 s",
    "zero_runtime_losses": (
        "rejection, drop, stale, eviction, expiry, pending, and timeout counters must be zero"
    ),
}

STATISTIC_METRICS = (
    "maximum_cte_m",
    "maximum_trajectory_correction_m",
    "maximum_lateral_acceleration_mps2",
    "source_rate_hz",
    "path_rate_hz",
    "trajectory_rate_hz",
    "source_period_violation_count",
    "inference_p99_ms",
    "maximum_status_wall_gap_s",
    "actual_to_final_cte_p95_m",
    "actual_to_route_cte_p95_m",
    "raw_to_final_geometry_p95_m",
    "raw_route_cte_p95_m",
)


class EvidenceError(RuntimeError):
    """Raised internally when one evidence field cannot be trusted."""


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    output: dict[str, Any] = {}
    for key, value in pairs:
        if key in output:
            raise EvidenceError(f"duplicate JSON key: {key}")
        output[key] = value
    return output


def _reject_nonfinite_constant(value: str) -> Any:
    raise EvidenceError(f"non-finite JSON constant: {value}")


def _validate_finite_tree(value: Any, label: str) -> None:
    if isinstance(value, float) and not math.isfinite(value):
        raise EvidenceError(f"{label} contains a non-finite number")
    if isinstance(value, list):
        for item in value:
            _validate_finite_tree(item, label)
    elif isinstance(value, dict):
        for item in value.values():
            _validate_finite_tree(item, label)


def _read_json(path: Path, label: str) -> dict[str, Any]:
    if path.is_symlink():
        raise EvidenceError(f"{label} may not be a symlink: {path}")
    if not path.is_file():
        raise EvidenceError(f"missing {label}: {path}")
    try:
        payload = json.loads(
            path.read_text(encoding="utf-8"),
            object_pairs_hook=_reject_duplicate_keys,
            parse_constant=_reject_nonfinite_constant,
        )
        _validate_finite_tree(payload, label)
    except EvidenceError:
        raise
    except (
        OSError,
        RecursionError,
        UnicodeDecodeError,
        json.JSONDecodeError,
    ) as error:
        raise EvidenceError(f"cannot read {label}: {error}") from error
    if not isinstance(payload, dict):
        raise EvidenceError(f"{label} root must be a JSON object")
    return payload


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as error:
        raise EvidenceError(f"cannot hash evidence file {path}: {error}") from error
    return digest.hexdigest()


def _nested(payload: Mapping[str, Any], *keys: str) -> Any:
    value: Any = payload
    for key in keys:
        if not isinstance(value, Mapping) or key not in value:
            raise EvidenceError(f"missing field: {'.'.join(keys)}")
        value = value[key]
    return value


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise EvidenceError(f"{label} must be numeric")
    output = float(value)
    if not math.isfinite(output):
        raise EvidenceError(f"{label} must be finite")
    return output


def _integer(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise EvidenceError(f"{label} must be an integer")
    return value


def _contained_regular_file(root: Path, relative: str, label: str) -> Path:
    candidate = root / relative
    current = root
    for part in Path(relative).parts:
        current = current / part
        if current.is_symlink():
            raise EvidenceError(f"{label} path may not contain symlinks: {candidate}")
    if not candidate.is_file():
        raise EvidenceError(f"missing {label}: {candidate}")
    try:
        resolved = candidate.resolve(strict=True)
    except OSError as error:
        raise EvidenceError(f"cannot resolve {label}: {error}") from error
    if root != resolved and root not in resolved.parents:
        raise EvidenceError(f"{label} escapes selected attempt: {candidate}")
    return resolved


def _check(
    row: dict[str, Any],
    name: str,
    passed: bool,
    actual: Any,
    reason: str | None = None,
) -> None:
    check = {
        "status": "PASS" if passed else "FAIL",
        "requirement": CHECK_REQUIREMENTS[name],
        "actual": actual,
    }
    if not passed:
        failure_reason = reason or f"observed value violates {name}"
        check["reason"] = failure_reason
        row["reasons"].append(f"{name}: {failure_reason}")
    row["checks"][name] = check


def _unavailable_check(row: dict[str, Any], name: str, reason: str) -> None:
    _check(row, name, False, None, f"not evaluated: {reason}")


def _load_selected_evidence(
    row: dict[str, Any], attempt_root: Path, relative: str, label: str
) -> dict[str, Any] | None:
    # HH_260906 - Preserve malformed evidence as a failed run instead of aborting.
    try:
        path = _contained_regular_file(attempt_root, relative, label)
        payload = _read_json(path, label)
        digest = _sha256(path)
        size_bytes = path.stat().st_size
    except (EvidenceError, OSError, RuntimeError, ValueError) as error:
        row["evidence_errors"].append(str(error))
        return None
    row["evidence"][label] = {
        "path": str(path),
        "sha256": digest,
        "size_bytes": size_bytes,
    }
    return payload


def _extract_diagnosis(
    row: dict[str, Any], diagnosis: dict[str, Any] | None
) -> None:
    if diagnosis is None:
        _unavailable_check(row, "diagnosis_valid", "diagnosis evidence is unavailable")
        return
    try:
        schema_version = _integer(diagnosis.get("schema_version"), "diagnosis schema")
        classification = _nested(diagnosis, "verdict", "classification")
        if not isinstance(classification, str) or not classification:
            raise EvidenceError("diagnosis verdict classification must be a string")
        values = {
            "actual_to_final_cte_p95_m": _number(
                _nested(
                    diagnosis,
                    "metrics",
                    "tracking",
                    "actual_to_final_cte_m",
                    "p95_abs",
                ),
                "diagnosis actual-to-final CTE p95",
            ),
            "actual_to_route_cte_p95_m": _number(
                _nested(
                    diagnosis,
                    "metrics",
                    "tracking",
                    "actual_to_route_cte_m",
                    "p95_abs",
                ),
                "diagnosis actual-to-route CTE p95",
            ),
            "raw_to_final_geometry_p95_m": _number(
                _nested(
                    diagnosis,
                    "metrics",
                    "raw_to_final_geometry",
                    "common_progress_p95_m",
                    "p95_abs",
                ),
                "diagnosis raw-to-final geometry p95",
            ),
            "raw_route_cte_p95_m": _number(
                _nested(diagnosis, "metrics", "raw_path", "p95_abs"),
                "diagnosis raw route CTE p95",
            ),
        }
        if any(value < 0.0 for value in values.values()):
            raise EvidenceError("diagnosis distance metrics must be non-negative")
        if schema_version != 2:
            raise EvidenceError(f"diagnosis schema must be 2; got {schema_version}")
    except EvidenceError as error:
        _check(row, "diagnosis_valid", False, None, str(error))
        return
    row["metrics"].update(values)
    row["diagnosis"] = {
        "schema_version": schema_version,
        "classification": classification,
    }
    _check(
        row,
        "diagnosis_valid",
        True,
        {"schema_version": schema_version, "classification": classification},
    )


def _extract_result(row: dict[str, Any], result: dict[str, Any] | None) -> None:
    result_checks = (
        "goal_reached",
        "cte_limit",
        "trajectory_correction_limit",
        "lateral_acceleration_limit",
    )
    if result is None:
        for name in result_checks:
            _unavailable_check(row, name, "result evidence is unavailable")
        return
    try:
        success = result.get("success")
        reason = result.get("reason")
        if not isinstance(success, bool):
            raise EvidenceError("result success must be boolean")
        if not isinstance(reason, str):
            raise EvidenceError("result reason must be a string")
        cte = _number(
            _nested(result, "metrics", "maximum_absolute_cte_m"),
            "maximum absolute CTE",
        )
        correction = _number(
            _nested(result, "metrics", "maximum_trajectory_correction_m"),
            "maximum trajectory correction",
        )
        lateral_acceleration = _number(
            _nested(result, "metrics", "maximum_lateral_acceleration_mps2"),
            "maximum lateral acceleration",
        )
        if min(cte, correction, lateral_acceleration) < 0.0:
            raise EvidenceError("result magnitude metrics must be non-negative")
    except EvidenceError as error:
        for name in result_checks:
            _unavailable_check(row, name, str(error))
        return
    row["metrics"].update(
        {
            "maximum_cte_m": cte,
            "maximum_trajectory_correction_m": correction,
            "maximum_lateral_acceleration_mps2": lateral_acceleration,
        }
    )
    _check(
        row,
        "goal_reached",
        success is True and reason == "goal reached",
        {"success": success, "reason": reason},
        "result did not finish with success=true and reason=goal reached",
    )
    _check(row, "cte_limit", cte <= LIMITS["maximum_cte_m"], cte)
    _check(
        row,
        "trajectory_correction_limit",
        correction <= LIMITS["maximum_trajectory_correction_m"],
        correction,
    )
    _check(
        row,
        "lateral_acceleration_limit",
        lateral_acceleration <= LIMITS["maximum_lateral_acceleration_mps2"],
        lateral_acceleration,
    )


def _extract_runtime_health(
    row: dict[str, Any], health: dict[str, Any] | None
) -> None:
    if health is None:
        _unavailable_check(
            row, "runtime_health_pass", "runtime health evidence is unavailable"
        )
        return
    try:
        schema_version = _integer(health.get("schema_version"), "runtime health schema")
        status = health.get("status")
        if not isinstance(status, str):
            raise EvidenceError("runtime health status must be a string")
    except EvidenceError as error:
        _check(row, "runtime_health_pass", False, None, str(error))
        return
    _check(
        row,
        "runtime_health_pass",
        schema_version == 1 and status == "PASS",
        {"schema_version": schema_version, "status": status},
        "runtime health is not schema-1 PASS",
    )


def _zero_loss_counters(shadow: Mapping[str, Any]) -> dict[str, int]:
    # HH_260906 - Audit raw counters as well as the analyzer's aggregate zero-loss claims.
    counters = {
        "anchor_rejected_count": _integer(
            _nested(
                shadow,
                "status_accounting",
                "denominators",
                "anchor_rejected_count",
            ),
            "anchor rejected count",
        ),
        "input_event_rejected_count": _integer(
            _nested(
                shadow,
                "status_accounting",
                "denominators",
                "input_event_rejected_count",
            ),
            "input event rejected count",
        ),
        "rejected_status_count": _integer(
            _nested(
                shadow,
                "status_accounting",
                "health_snapshot_counts",
                "rejected",
            ),
            "rejected status count",
        ),
        "camera_bundle_timeout_count": _integer(
            _nested(
                shadow,
                "status_accounting",
                "health_snapshot_counts",
                "camera_bundle_timeout",
            ),
            "camera bundle timeout count",
        ),
        "dropped_stale_count": _integer(
            _nested(
                shadow,
                "status_accounting",
                "camera_bundle_cumulative_delta",
                "dropped_stale_count",
            ),
            "dropped stale count",
        ),
        "evicted_capacity_count": _integer(
            _nested(
                shadow,
                "status_accounting",
                "camera_bundle_cumulative_delta",
                "evicted_capacity_count",
            ),
            "evicted capacity count",
        ),
        "expired_pending_count": _integer(
            _nested(
                shadow,
                "status_accounting",
                "camera_bundle_cumulative_delta",
                "expired_pending_count",
            ),
            "expired pending count",
        ),
        "input_settle_timeout_count": _integer(
            _nested(
                shadow,
                "status_accounting",
                "input_settle",
                "input_settle_timeout_count",
            ),
            "input settle timeout count",
        ),
        "observed_drop_count": _integer(
            _nested(
                shadow,
                "claims",
                "ten_hz",
                "requirements",
                "pre_startup_only_initial_waiting_and_zero_losses",
                "observed_drop_count",
            ),
            "observed drop count",
        ),
        "observed_rejection_count": _integer(
            _nested(
                shadow,
                "claims",
                "ten_hz",
                "requirements",
                "pre_startup_only_initial_waiting_and_zero_losses",
                "observed_rejection_count",
            ),
            "observed rejection count",
        ),
        "observed_timeout_count": _integer(
            _nested(
                shadow,
                "claims",
                "ten_hz",
                "requirements",
                "pre_startup_only_initial_waiting_and_zero_losses",
                "observed_timeout_count",
            ),
            "observed timeout count",
        ),
        "terminal_pending_bundle_count": _integer(
            _nested(
                shadow,
                "claims",
                "ten_hz",
                "requirements",
                "zero_silent_runtime_losses_arm_to_seal",
                "observed_terminal_pending_bundle_count",
            ),
            "terminal pending bundle count",
        ),
    }
    stages = _nested(shadow, "status_accounting", "rejection_counts_by_stage")
    if not isinstance(stages, Mapping) or not stages:
        raise EvidenceError("rejection counts by stage must be a non-empty object")
    for stage, value in stages.items():
        if not isinstance(stage, str) or not stage:
            raise EvidenceError("rejection stage names must be non-empty strings")
        counters[f"rejection_stage:{stage}"] = _integer(
            value, f"rejection stage {stage} count"
        )
    if any(value < 0 for value in counters.values()):
        raise EvidenceError("runtime loss counters must be non-negative")
    return counters


def _extract_shadow(row: dict[str, Any], shadow: dict[str, Any] | None) -> None:
    shadow_checks = (
        "shadow_evidence_valid",
        "shadow_ten_hz_pass",
        "source_rate",
        "path_rate",
        "trajectory_rate",
        "source_period_continuity",
        "inference_latency",
        "status_wall_gap",
        "zero_runtime_losses",
    )
    if shadow is None:
        for name in shadow_checks:
            _unavailable_check(row, name, "shadow evidence is unavailable")
        return
    try:
        schema_id = shadow.get("schema_id")
        analysis_status = shadow.get("analysis_status")
        if not isinstance(schema_id, str) or not isinstance(analysis_status, str):
            raise EvidenceError("shadow schema and analysis status must be strings")
        ten_hz_pass = _nested(shadow, "claims", "ten_hz", "pass")
        ten_hz_status = _nested(shadow, "claims", "ten_hz", "status")
        if not isinstance(ten_hz_pass, bool) or not isinstance(ten_hz_status, str):
            raise EvidenceError("shadow ten-Hz claim fields have invalid types")
        requirements = _nested(shadow, "claims", "ten_hz", "requirements")
        if not isinstance(requirements, Mapping):
            raise EvidenceError("shadow ten-Hz requirements must be an object")
        source_rate = _number(
            _nested(requirements, "anchor_rate_hz", "observed"),
            "source anchor rate",
        )
        path_rate = _number(
            _nested(requirements, "path_receipt_rate_hz", "observed"),
            "path receipt rate",
        )
        trajectory_rate = _number(
            _nested(requirements, "trajectory_receipt_rate_hz", "observed"),
            "trajectory receipt rate",
        )
        period_met = _nested(requirements, "continuous_source_anchor_period", "met")
        if not isinstance(period_met, bool):
            raise EvidenceError("source period met flag must be boolean")
        period_violations = _integer(
            _nested(
                requirements,
                "continuous_source_anchor_period",
                "observed_violation_count",
            ),
            "source period violation count",
        )
        inference_p99_ms = _number(
            _nested(shadow, "accepted_latency", "p99_ms"),
            "accepted inference p99",
        )
        maximum_wall_gap_s = _number(
            _nested(shadow, "accepted_status_wall_timing", "maximum_gap_s"),
            "maximum accepted status wall gap",
        )
        counters = _zero_loss_counters(shadow)
        zero_rejections_met = _nested(
            requirements, "zero_rejections_arm_to_seal", "met"
        )
        zero_losses_met = _nested(
            requirements, "zero_silent_runtime_losses_arm_to_seal", "met"
        )
        pre_startup_losses_met = _nested(
            requirements,
            "pre_startup_only_initial_waiting_and_zero_losses",
            "met",
        )
        if not all(
            isinstance(value, bool)
            for value in (
                zero_rejections_met,
                zero_losses_met,
                pre_startup_losses_met,
            )
        ):
            raise EvidenceError("shadow zero-loss requirement flags must be boolean")
        if min(
            source_rate,
            path_rate,
            trajectory_rate,
            period_violations,
            inference_p99_ms,
            maximum_wall_gap_s,
        ) < 0.0:
            raise EvidenceError("shadow rate, count, latency, and gap values must be non-negative")
    except EvidenceError as error:
        for name in shadow_checks:
            _unavailable_check(row, name, str(error))
        return
    row["metrics"].update(
        {
            "source_rate_hz": source_rate,
            "path_rate_hz": path_rate,
            "trajectory_rate_hz": trajectory_rate,
            "source_period_violation_count": period_violations,
            "inference_p99_ms": inference_p99_ms,
            "maximum_status_wall_gap_s": maximum_wall_gap_s,
        }
    )
    row["runtime_loss_counters"] = counters
    _check(
        row,
        "shadow_evidence_valid",
        schema_id == SHADOW_SCHEMA_ID and analysis_status == "EVIDENCE_VALID",
        {"schema_id": schema_id, "analysis_status": analysis_status},
        "shadow schema or evidence-valid status changed",
    )
    _check(
        row,
        "shadow_ten_hz_pass",
        ten_hz_pass is True and ten_hz_status == TEN_HZ_STATUS,
        {"pass": ten_hz_pass, "status": ten_hz_status},
        "ten-Hz claim is not established for shadow-only scope",
    )
    for name, value in (
        ("source_rate", source_rate),
        ("path_rate", path_rate),
        ("trajectory_rate", trajectory_rate),
    ):
        _check(
            row,
            name,
            LIMITS["minimum_rate_hz"] <= value <= LIMITS["maximum_rate_hz"],
            value,
        )
    _check(
        row,
        "source_period_continuity",
        period_met is True and period_violations == 0,
        {"met": period_met, "violation_count": period_violations},
        "source period continuity is false or has non-zero violations",
    )
    _check(
        row,
        "inference_latency",
        inference_p99_ms <= LIMITS["maximum_inference_p99_ms"],
        inference_p99_ms,
    )
    _check(
        row,
        "status_wall_gap",
        maximum_wall_gap_s <= LIMITS["maximum_status_wall_gap_s"],
        maximum_wall_gap_s,
    )
    zero_loss_flags = (
        zero_rejections_met is True
        and zero_losses_met is True
        and pre_startup_losses_met is True
    )
    _check(
        row,
        "zero_runtime_losses",
        zero_loss_flags and all(value == 0 for value in counters.values()),
        {
            "requirement_flags": {
                "zero_rejections_arm_to_seal": zero_rejections_met,
                "zero_silent_runtime_losses_arm_to_seal": zero_losses_met,
                "pre_startup_only_initial_waiting_and_zero_losses": (
                    pre_startup_losses_met
                ),
            },
            "counters": counters,
        },
        "one or more zero-loss flags are false or counters are non-zero",
    )


def _finalize_run(row: dict[str, Any]) -> dict[str, Any]:
    blocker = (
        row["evidence_errors"][0]
        if row["evidence_errors"]
        else "an earlier evidence prerequisite failed"
    )
    for name in CHECK_REQUIREMENTS:
        if name not in row["checks"]:
            _unavailable_check(row, name, blocker)
    row["status"] = (
        "PASS"
        if all(check["status"] == "PASS" for check in row["checks"].values())
        else "FAIL"
    )
    row["reasons"] = list(dict.fromkeys(row["reasons"]))
    return row


def evaluate_run(run_directory: Path, expected_route_sha256: str) -> dict[str, Any]:
    raw_input_path = Path(run_directory)
    row: dict[str, Any] = {
        "run_id": raw_input_path.name or str(raw_input_path),
        "path": str(raw_input_path.absolute()),
        "status": "FAIL",
        "selected_attempt": None,
        "selected_attempt_path": None,
        "checks": {},
        "metrics": {},
        "runtime_loss_counters": {},
        "diagnosis": None,
        "evidence": {},
        "evidence_errors": [],
        "reasons": [],
    }
    try:
        input_path = raw_input_path.expanduser()
    except (OSError, RuntimeError, ValueError) as error:
        row["evidence_errors"].append(f"cannot expand run directory: {error}")
        return _finalize_run(row)
    if input_path.is_symlink():
        error = f"run directory may not be a symlink: {input_path}"
        row["evidence_errors"].append(error)
        return _finalize_run(row)
    try:
        run_root = input_path.resolve(strict=True)
    except (OSError, RuntimeError, ValueError) as error:
        row["evidence_errors"].append(f"cannot resolve run directory: {error}")
        return _finalize_run(row)
    row["path"] = str(run_root)
    if not run_root.is_dir():
        row["evidence_errors"].append(f"run path is not a directory: {run_root}")
        return _finalize_run(row)

    owner_path = run_root / "owned_trial_summary.json"
    try:
        owner = _read_json(owner_path, "owned trial summary")
        row["evidence"]["owned_trial_summary"] = {
            "path": str(owner_path.resolve()),
            "sha256": _sha256(owner_path),
            "size_bytes": owner_path.stat().st_size,
        }
    except (EvidenceError, OSError, RuntimeError, ValueError) as error:
        row["evidence_errors"].append(str(error))
        return _finalize_run(row)

    schema_version = owner.get("schema_version")
    owner_status = owner.get("status")
    owner_pass = (
        isinstance(schema_version, int)
        and not isinstance(schema_version, bool)
        and schema_version == 1
        and owner_status == "PASS"
    )
    _check(
        row,
        "owner_pass",
        owner_pass,
        {"schema_version": schema_version, "status": owner_status},
        "owner summary is not schema-1 PASS",
    )

    selected_id = owner.get("selected_attempt")
    attempts = owner.get("attempts")
    selected_rows: list[dict[str, Any]] = []
    selection_error = ""
    if not isinstance(selected_id, str) or not selected_id:
        selection_error = "selected_attempt must be a non-empty string"
    elif not re.fullmatch(r"attempt_[0-9]{3}", selected_id):
        selection_error = "selected_attempt must match attempt_NNN"
    elif not isinstance(attempts, list):
        selection_error = "attempts must be a list"
    elif any(not isinstance(attempt, dict) for attempt in attempts):
        selection_error = "every attempts entry must be an object"
    else:
        selected_rows = [
            attempt
            for attempt in attempts
            if isinstance(attempt, dict) and attempt.get("attempt_id") == selected_id
        ]
        if len(selected_rows) != 1:
            selection_error = (
                f"selected attempt must match exactly one row; got {len(selected_rows)}"
            )

    attempt_root: Path | None = None
    selected_row: dict[str, Any] | None = None
    if not selection_error:
        selected_row = selected_rows[0]
        try:
            candidate = run_root / "attempts" / selected_id
            if candidate.is_symlink():
                selection_error = "selected attempt directory may not be a symlink"
            elif not candidate.is_dir():
                selection_error = f"selected attempt directory is missing: {candidate}"
            else:
                attempt_root = candidate.resolve(strict=True)
                if run_root not in attempt_root.parents:
                    selection_error = "selected attempt directory escapes run root"
                recorded_path = selected_row.get("path")
                if not isinstance(recorded_path, str):
                    selection_error = "selected attempt recorded path must be a string"
                elif (
                    not selection_error
                    and Path(recorded_path).expanduser().resolve() != attempt_root
                ):
                    selection_error = (
                        "selected attempt recorded path does not match directory"
                    )
        except (OSError, RuntimeError, ValueError) as error:
            selection_error = f"cannot resolve selected attempt path: {error}"
            attempt_root = None

    selection_pass = not selection_error and attempt_root is not None
    _check(
        row,
        "selected_attempt_unambiguous",
        selection_pass,
        {
            "selected_attempt": selected_id,
            "matching_rows": len(selected_rows),
            "attempt_count": len(attempts) if isinstance(attempts, list) else None,
        },
        selection_error or "selected attempt could not be resolved",
    )
    if not selection_pass or selected_row is None or attempt_root is None:
        return _finalize_run(row)
    row["selected_attempt"] = selected_id
    row["selected_attempt_path"] = str(attempt_root)

    process_exit_status = selected_row.get("process_exit_status")
    selected_health_status = selected_row.get("runtime_health_status")
    selected_attempt_pass = (
        isinstance(process_exit_status, int)
        and not isinstance(process_exit_status, bool)
        and process_exit_status == 0
        and selected_health_status == "PASS"
    )
    _check(
        row,
        "selected_attempt_pass",
        selected_attempt_pass,
        {
            "process_exit_status": process_exit_status,
            "runtime_health_status": selected_health_status,
        },
        "selected owner attempt did not record exit 0 and runtime-health PASS",
    )

    source_route = _load_selected_evidence(
        row, attempt_root, "source_route.json", "source route"
    )
    source_route_sha256 = None
    if source_route is not None:
        source_route_sha256 = row["evidence"]["source route"]["sha256"]
    owner_route_sha256 = owner.get("route_sha256")
    expected_valid = (
        isinstance(expected_route_sha256, str)
        and SHA256_PATTERN.fullmatch(expected_route_sha256) is not None
    )
    exact_route = (
        expected_valid
        and owner_route_sha256 == expected_route_sha256
        and source_route_sha256 == expected_route_sha256
    )
    _check(
        row,
        "exact_route_sha256",
        exact_route,
        {
            "expected": expected_route_sha256,
            "owner": owner_route_sha256,
            "source_route": source_route_sha256,
        },
        "expected, owner, and selected source route SHA-256 values do not all match",
    )

    result = _load_selected_evidence(row, attempt_root, "result.json", "route result")
    health = _load_selected_evidence(
        row, attempt_root, "runtime_health.json", "runtime health"
    )
    diagnosis = _load_selected_evidence(
        row, attempt_root, "diagnosis.json", "turn diagnosis"
    )
    shadow = _load_selected_evidence(
        row,
        attempt_root,
        "portable_shadow_provenance/shadow_evidence_analysis.json",
        "Portable shadow evidence",
    )
    _extract_result(row, result)
    _extract_runtime_health(row, health)
    _extract_diagnosis(row, diagnosis)
    _extract_shadow(row, shadow)
    return _finalize_run(row)


def _metric_statistics(runs: Sequence[dict[str, Any]]) -> dict[str, Any]:
    output: dict[str, Any] = {}
    for name in STATISTIC_METRICS:
        values = [
            float(run["metrics"][name])
            for run in runs
            if name in run["metrics"]
            and isinstance(run["metrics"][name], (int, float))
            and not isinstance(run["metrics"][name], bool)
            and math.isfinite(float(run["metrics"][name]))
        ]
        output[name] = {
            "count": len(values),
            "minimum": min(values) if values else None,
            "median": statistics.median(values) if values else None,
            "maximum": max(values) if values else None,
            "values": values,
        }
    return output


def build_summary(
    run_directories: Sequence[Path], expected_route_sha256: str
) -> dict[str, Any]:
    runs = [evaluate_run(path, expected_route_sha256) for path in run_directories]
    resolved_inputs = [run["path"] for run in runs]
    route_sha_valid = (
        isinstance(expected_route_sha256, str)
        and SHA256_PATTERN.fullmatch(expected_route_sha256) is not None
    )
    unique_inputs = len(set(resolved_inputs)) == len(resolved_inputs)
    campaign_checks = {
        "nonempty_run_set": {
            "status": "PASS" if runs else "FAIL",
            "actual": len(runs),
            "requirement": "at least one owned run directory is required",
        },
        "valid_expected_route_sha256": {
            "status": "PASS" if route_sha_valid else "FAIL",
            "actual": expected_route_sha256,
            "requirement": "expected route SHA-256 must be 64 lowercase hex characters",
        },
        "unique_run_directories": {
            "status": "PASS" if unique_inputs else "FAIL",
            "actual": resolved_inputs,
            "requirement": "each resolved run directory may appear only once",
        },
        "all_runs_pass": {
            "status": (
                "PASS" if runs and all(run["status"] == "PASS" for run in runs) else "FAIL"
            ),
            "actual": {
                "pass_count": sum(run["status"] == "PASS" for run in runs),
                "run_count": len(runs),
            },
            "requirement": "every supplied owned run must pass every fail-closed gate",
        },
    }
    status = (
        "PASS"
        if all(check["status"] == "PASS" for check in campaign_checks.values())
        else "FAIL"
    )
    failed_runs = [run["run_id"] for run in runs if run["status"] != "PASS"]
    reasons = []
    if not runs:
        reasons.append("no run directories were supplied")
    if not route_sha_valid:
        reasons.append("expected route SHA-256 format is invalid")
    if not unique_inputs:
        reasons.append("one or more run directories were supplied more than once")
    if failed_runs:
        reasons.append(f"failed runs: {', '.join(failed_runs)}")
    gate_pass_counts = {
        name: {
            "pass_count": sum(
                run["checks"].get(name, {}).get("status") == "PASS" for run in runs
            ),
            "run_count": len(runs),
        }
        for name in CHECK_REQUIREMENTS
    }
    campaign_gate_checks = {
        name: {
            "status": (
                "PASS"
                if runs and counts["pass_count"] == counts["run_count"]
                else "FAIL"
            ),
            "actual": counts,
            "requirement": f"every run must satisfy: {CHECK_REQUIREMENTS[name]}",
        }
        for name, counts in gate_pass_counts.items()
    }
    return {
        "schema_id": SCHEMA_ID,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "status": status,
        "expected_route_sha256": expected_route_sha256,
        "limits": LIMITS,
        "run_count": len(runs),
        "pass_count": sum(run["status"] == "PASS" for run in runs),
        "fail_count": sum(run["status"] != "PASS" for run in runs),
        "campaign_checks": campaign_checks,
        "campaign_gate_checks": campaign_gate_checks,
        "gate_pass_counts": gate_pass_counts,
        "statistics": _metric_statistics(runs),
        "runs": runs,
        "reasons": reasons,
        "scope": "CARLA simulation screening; not approved for real-vehicle control",
    }


def _plot_metric(
    axis: Any,
    runs: Sequence[dict[str, Any]],
    metric: str,
    title: str,
    unit: str,
    limit: float,
) -> None:
    positions = list(range(len(runs)))
    values = [run["metrics"].get(metric) for run in runs]
    bars = [float(value) if isinstance(value, (int, float)) else 0.0 for value in values]
    colors = ["#16836f" if run["status"] == "PASS" else "#c4483d" for run in runs]
    rectangles = axis.bar(positions, bars, color=colors, alpha=0.88)
    for rectangle, value in zip(rectangles, values):
        if not isinstance(value, (int, float)):
            rectangle.set_hatch("//")
            axis.text(
                rectangle.get_x() + rectangle.get_width() / 2,
                0.01 * max(limit, 1.0),
                "N/A",
                ha="center",
                va="bottom",
                fontsize=8,
                rotation=90,
            )
    axis.axhline(limit, color="#111820", linestyle="--", linewidth=1.1)
    axis.set_title(title, weight="bold")
    axis.set_ylabel(unit)
    axis.grid(axis="y", alpha=0.25)


def render_summary(summary: Mapping[str, Any], output: Path) -> None:
    runs = summary["runs"]
    labels = [run["run_id"] for run in runs]
    positions = list(range(len(runs)))
    figure, axes = plt.subplots(2, 3, figsize=(16, 9.5), constrained_layout=True)
    _plot_metric(
        axes[0, 0],
        runs,
        "maximum_cte_m",
        "Maximum route CTE",
        "m",
        LIMITS["maximum_cte_m"],
    )
    _plot_metric(
        axes[0, 1],
        runs,
        "maximum_trajectory_correction_m",
        "Maximum trajectory correction",
        "m",
        LIMITS["maximum_trajectory_correction_m"],
    )
    _plot_metric(
        axes[0, 2],
        runs,
        "maximum_lateral_acceleration_mps2",
        "Maximum lateral acceleration",
        "m/s^2",
        LIMITS["maximum_lateral_acceleration_mps2"],
    )

    rate_axis = axes[1, 0]
    for offset, metric, label, color in (
        (-0.22, "source_rate_hz", "source", "#2464a6"),
        (0.0, "path_rate_hz", "path", "#16836f"),
        (0.22, "trajectory_rate_hz", "trajectory", "#b46b18"),
    ):
        xs = []
        ys = []
        for index, run in enumerate(runs):
            value = run["metrics"].get(metric)
            if isinstance(value, (int, float)):
                xs.append(index + offset)
                ys.append(float(value))
        rate_axis.scatter(xs, ys, label=label, color=color, s=45, zorder=3)
    rate_axis.axhspan(
        LIMITS["minimum_rate_hz"],
        LIMITS["maximum_rate_hz"],
        color="#8ecf9f",
        alpha=0.22,
    )
    rate_axis.set_title("Portable shadow publication rates", weight="bold")
    rate_axis.set_ylabel("Hz")
    rate_axis.legend(loc="best", fontsize=8)
    rate_axis.grid(axis="y", alpha=0.25)
    _plot_metric(
        axes[1, 1],
        runs,
        "inference_p99_ms",
        "Accepted inference p99",
        "ms",
        LIMITS["maximum_inference_p99_ms"],
    )
    _plot_metric(
        axes[1, 2],
        runs,
        "maximum_status_wall_gap_s",
        "Maximum accepted-status wall gap",
        "s",
        LIMITS["maximum_status_wall_gap_s"],
    )
    for axis in axes.flat:
        axis.set_xticks(positions, labels, rotation=25, ha="right")
        axis.spines[["top", "right"]].set_visible(False)
    figure.suptitle(
        "Owned CARLA route repeatability | "
        f"{summary['status']} | {summary['pass_count']}/{summary['run_count']} PASS",
        fontsize=16,
        weight="bold",
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        figure.savefig(output, dpi=170, facecolor="white", format="png")
    finally:
        plt.close(figure)


def _stage_json(path: Path, payload: Mapping[str, Any]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    content = json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n"
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".staged", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
    return temporary


def _stage_png(summary: Mapping[str, Any], path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".png", dir=path.parent
    )
    os.close(descriptor)
    temporary = Path(temporary_name)
    try:
        render_summary(summary, temporary)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
    return temporary


def _publish_output_pair(
    staged_json: Path,
    output_json: Path,
    staged_png: Path,
    output_png: Path,
) -> None:
    published: list[tuple[Path, Path]] = []
    try:
        for staged, output in (
            (staged_json, output_json),
            (staged_png, output_png),
        ):
            os.link(staged, output)
            published.append((staged, output))
    except OSError:
        for staged, output in reversed(published):
            try:
                if os.path.samestat(os.lstat(staged), os.lstat(output)):
                    output.unlink()
            except OSError:
                pass
        raise


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--run",
        action="append",
        type=Path,
        required=True,
        dest="runs",
        help="owned route-trial root; repeat for every run",
    )
    parser.add_argument("--expected-route-sha256", required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-png", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        output_json = args.output_json.expanduser().absolute()
        output_png = args.output_png.expanduser().absolute()
    except (OSError, RuntimeError, ValueError) as error:
        raise SystemExit(f"cannot resolve output path: {error}") from error
    if output_json == output_png:
        raise SystemExit("JSON and PNG outputs must be different paths")
    # HH_260906 - Refuse existing destinations so a later summary cannot overwrite prior evidence.
    if output_json.exists() or output_json.is_symlink():
        raise SystemExit(f"output already exists: {output_json}")
    if output_png.exists() or output_png.is_symlink():
        raise SystemExit(f"output already exists: {output_png}")
    summary = build_summary(args.runs, args.expected_route_sha256)
    staged_json: Path | None = None
    staged_png: Path | None = None
    try:
        staged_json = _stage_json(output_json, summary)
        staged_png = _stage_png(summary, output_png)
        try:
            _publish_output_pair(
                staged_json,
                output_json,
                staged_png,
                output_png,
            )
        except FileExistsError as error:
            raise SystemExit("one or more output paths already exist") from error
    finally:
        if staged_json is not None:
            staged_json.unlink(missing_ok=True)
        if staged_png is not None:
            staged_png.unlink(missing_ok=True)
    print(
        f"OWNED_ROUTE_REPEAT_SUMMARY status={summary['status']} "
        f"pass={summary['pass_count']}/{summary['run_count']} "
        f"json={output_json} png={output_png}",
        flush=True,
    )
    return 0 if summary["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
