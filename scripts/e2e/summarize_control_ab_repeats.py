#!/usr/bin/env python3
"""Summarize repeated owned 30 km/h control A/B pairs fail-closed."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import statistics
import sys
from typing import Any, Callable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import compare_control_ab as control_ab  # noqa: E402


BASELINE_DIRECTORY_NAME = "A_baseline"
PAIRED_METRICS = (
    "route_rtf",
    "sim_elapsed_sec",
    "maximum_speed_mps",
    "sustained_speed_sec",
    "maximum_cte_m",
    "maximum_lateral_acceleration_mps2",
    "maximum_trajectory_correction_m",
    "target_tracking_rmse_mps",
    "raw_gated_target_rmse_mps",
    "robust_acceleration_p95_mps2",
    "gate_positive_cap_time_percent",
    "camera_bundle_coverage_percent",
    "camera_bundle_receipt_p95_sec",
    "path_cte_p95_m",
)
SAFE_DIRECTORY_NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]*")
SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")
GEOMETRY_FAILURE_PATTERNS = (
    re.compile(r"^(?:initial )?cross-track error (?:exceeds|limit exceeded)"),
    re.compile(r"^(?:initial )?trajectory correction (?:exceeds|limit exceeded)"),
    re.compile(
        r"^VAD route manager reported fault:(?:trajectory_correction|route_deviation):"
    ),
)


class SummaryError(RuntimeError):
    pass


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise SummaryError(f"cannot read JSON {path}: {error}") from error
    if not isinstance(value, dict):
        raise SummaryError(f"JSON root must be an object: {path}")
    return value


def _sha256(path: Path) -> str:
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError as error:
        raise SummaryError(f"cannot hash {path}: {error}") from error


def _finite_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _safe_attempt_path(root: Path, attempt: dict[str, Any]) -> Path | None:
    attempt_id = attempt.get("attempt_id")
    if not isinstance(attempt_id, str) or not SAFE_DIRECTORY_NAME.fullmatch(attempt_id):
        return None
    path = (root / "attempts" / attempt_id).resolve()
    if root != path and root not in path.parents:
        return None
    recorded = attempt.get("path")
    if not isinstance(recorded, str) or Path(recorded).expanduser().resolve() != path:
        return None
    return path


def _result_has_assessed_route_execution(result: dict[str, Any]) -> bool:
    assessment = result.get("assessment")
    metrics = result.get("metrics")
    if not isinstance(assessment, dict) or not isinstance(metrics, dict):
        return False
    geometry = assessment.get("trajectory_geometry")
    sim_elapsed = metrics.get("sim_elapsed_sec")
    return (
        isinstance(geometry, str)
        and geometry not in {"", "not_assessed"}
        and _finite_number(sim_elapsed)
        and float(sim_elapsed) > 0.0
    )


def _explicit_geometry_failure(reason: Any) -> bool:
    return isinstance(reason, str) and any(
        pattern.search(reason) for pattern in GEOMETRY_FAILURE_PATTERNS
    )


def inspect_arm(
    root: Path,
    role: str,
    expected_route_sha256: str,
) -> dict[str, Any]:
    resolved = root.expanduser().resolve()
    output: dict[str, Any] = {
        "role": role,
        "owner_root": str(resolved),
        "owner_summary_status": None,
        "selected_attempt": None,
        "selected_owner_pass": False,
        "runtime_health_status": None,
        "process_exit_status": None,
        "route_contract_status": "FAIL",
        "route_status": "NOT_ASSESSED",
        "infrastructure_status": "FAIL",
        "geometry_status": "NOT_ASSESSED",
        "disposition": "INVALID_INFRA",
        "result_reason": None,
        "failures": [],
    }
    summary_path = resolved / "owned_trial_summary.json"
    try:
        summary = _read_json(summary_path)
    except SummaryError as error:
        output["failures"].append(str(error))
        return output
    output["owner_summary_status"] = summary.get("status")
    output["selected_attempt"] = summary.get("selected_attempt")
    owner_route_sha256 = summary.get("route_sha256")
    output["owner_route_sha256"] = owner_route_sha256
    if summary.get("schema_version") != 1:
        output["failures"].append("owned summary schema_version must equal 1")
    attempts = summary.get("attempts")
    if not isinstance(attempts, list) or len(attempts) != 1:
        output["failures"].append(
            "owned summary must contain exactly one authoritative attempt"
        )
        return output
    attempt = attempts[0]
    if not isinstance(attempt, dict):
        output["failures"].append("owned attempt must be an object")
        return output
    attempt_path = _safe_attempt_path(resolved, attempt)
    if attempt_path is None:
        output["failures"].append("owned attempt path or ID is invalid")
        return output
    output["attempt_path"] = str(attempt_path)
    health_status = attempt.get("runtime_health_status")
    process_status = attempt.get("process_exit_status")
    output["runtime_health_status"] = health_status
    output["process_exit_status"] = process_status
    if health_status != "PASS":
        output["failures"].append("runtime health did not PASS")
    if not isinstance(process_status, int) or isinstance(process_status, bool):
        output["failures"].append("process exit status is invalid")

    health_evidence_pass = False
    health_path = attempt_path / "runtime_health.json"
    runtime_env_path = attempt_path / "runtime.env"
    if health_path.is_file() and runtime_env_path.is_file():
        try:
            health = _read_json(health_path)
            environment = control_ab.parse_env(runtime_env_path)
            health_sha256 = _sha256(health_path)
        except (SummaryError, control_ab.ComparisonError, OSError) as error:
            output["failures"].append(f"runtime health evidence is invalid: {error}")
        else:
            health_evidence_pass = (
                health.get("status") == "PASS"
                and environment.get("RUNTIME_HEALTH_GATE_ENABLED") == "true"
                and environment.get("RUNTIME_HEALTH_GATE_STATUS") == "PASS"
                and environment.get("RUNTIME_HEALTH_EVIDENCE_SHA256")
                == health_sha256
            )
            output["runtime_health_evidence_sha256"] = health_sha256
    if not health_evidence_pass:
        output["failures"].append(
            "runtime health JSON and runtime.env provenance did not PASS"
        )

    source_route = attempt_path / "source_route.json"
    source_route_sha256 = _sha256(source_route) if source_route.is_file() else None
    output["source_route_sha256"] = source_route_sha256
    route_contract_pass = (
        owner_route_sha256 == expected_route_sha256
        and source_route_sha256 == expected_route_sha256
    )
    output["route_contract_status"] = "PASS" if route_contract_pass else "FAIL"
    if not route_contract_pass:
        output["failures"].append(
            "owner and source route SHA-256 must match the expected route"
        )

    result_path = attempt_path / "result.json"
    result: dict[str, Any] | None = None
    if result_path.is_file():
        try:
            result = _read_json(result_path)
        except SummaryError as error:
            output["failures"].append(str(error))
    else:
        output["failures"].append("route result evidence is missing")
    result_success = result.get("success") if result is not None else None
    if isinstance(result_success, bool):
        output["route_status"] = "PASS" if result_success else "FAIL"
        output["result_reason"] = result.get("reason")
        if result_success:
            output["geometry_status"] = "PASS"
        elif _explicit_geometry_failure(result.get("reason")):
            output["geometry_status"] = "FAIL"
    else:
        output["failures"].append("route result has no boolean success field")

    attempt_id = attempt.get("attempt_id")
    selected_owner_pass = (
        summary.get("status") == "PASS"
        and summary.get("selected_attempt") == attempt_id
        and process_status == 0
        and health_status == "PASS"
        and health_evidence_pass
        and route_contract_pass
        and result_success is True
    )
    output["selected_owner_pass"] = selected_owner_pass
    geometry_failure = (
        summary.get("status") == "FAIL"
        and summary.get("selected_attempt") is None
        and isinstance(process_status, int)
        and not isinstance(process_status, bool)
        and process_status != 0
        and health_status == "PASS"
        and health_evidence_pass
        and route_contract_pass
        and result_success is False
        and output["geometry_status"] == "FAIL"
    )
    route_failure = (
        summary.get("status") == "FAIL"
        and summary.get("selected_attempt") is None
        and isinstance(process_status, int)
        and not isinstance(process_status, bool)
        and process_status != 0
        and health_status == "PASS"
        and health_evidence_pass
        and route_contract_pass
        and result_success is False
        and result is not None
        and _result_has_assessed_route_execution(result)
        and not geometry_failure
    )
    if selected_owner_pass:
        output["infrastructure_status"] = "PASS"
        output["disposition"] = "PASS"
    elif geometry_failure:
        output["infrastructure_status"] = "PASS"
        output["disposition"] = "GEOMETRY_FAIL"
    elif route_failure:
        output["infrastructure_status"] = "PASS"
        output["disposition"] = "ROUTE_FAIL"
    else:
        output["failures"].append("arm lacks one selected owned PASS attempt")
    return output


def _pair_classification(baseline: dict[str, Any], candidate: dict[str, Any]) -> str:
    invalid = [
        role
        for role, arm in (("BASELINE", baseline), ("CANDIDATE", candidate))
        if arm["disposition"] == "INVALID_INFRA"
    ]
    if invalid:
        return "INVALID_BOTH_INFRA" if len(invalid) == 2 else f"INVALID_{invalid[0]}_INFRA"
    geometry = [
        role
        for role, arm in (("BASELINE", baseline), ("CANDIDATE", candidate))
        if arm["disposition"] == "GEOMETRY_FAIL"
    ]
    if geometry:
        return (
            "GEOMETRY_FAIL_BOTH"
            if len(geometry) == 2
            else f"GEOMETRY_FAIL_{geometry[0]}"
        )
    route_failures = [
        role
        for role, arm in (("BASELINE", baseline), ("CANDIDATE", candidate))
        if arm["disposition"] == "ROUTE_FAIL"
    ]
    if route_failures:
        return (
            "ROUTE_FAIL_BOTH"
            if len(route_failures) == 2
            else f"ROUTE_FAIL_{route_failures[0]}"
        )
    return "READY_FOR_COMPARATOR"


def _arm_gate_summary(pairs: list[dict[str, Any]], role: str) -> dict[str, Any]:
    arms = [pair[role] for pair in pairs]
    total = len(arms)

    def gate(name: str, statuses: tuple[str, ...]) -> dict[str, Any]:
        counts = Counter(arm[name] for arm in arms)
        passed = counts.get("PASS", 0)
        assessed = sum(counts.get(status, 0) for status in statuses)
        return {
            "counts": {status: counts.get(status, 0) for status in statuses},
            "pass_rate_percent_of_all": 100.0 * passed / total if total else None,
            "pass_rate_percent_of_assessed": (
                100.0 * passed / assessed if assessed else None
            ),
            "assessed_count": assessed,
            "total_count": total,
        }

    geometry = gate("geometry_status", ("PASS", "FAIL", "NOT_ASSESSED"))
    geometry_failures = geometry["counts"]["FAIL"]
    geometry_assessed = (
        geometry["counts"]["PASS"] + geometry["counts"]["FAIL"]
    )
    geometry["failure_rate_percent_of_assessed"] = (
        100.0 * geometry_failures / geometry_assessed
        if geometry_assessed
        else None
    )
    return {
        "infrastructure": gate("infrastructure_status", ("PASS", "FAIL")),
        "route": gate("route_status", ("PASS", "FAIL", "NOT_ASSESSED")),
        "geometry": geometry,
    }


def _paired_delta_summary(valid_pairs: list[dict[str, Any]]) -> dict[str, Any]:
    output: dict[str, Any] = {}
    for metric in PAIRED_METRICS:
        deltas = [
            float(pair["candidate_metrics"][metric])
            - float(pair["baseline_metrics"][metric])
            for pair in valid_pairs
        ]
        output[metric] = {
            "definition": "candidate_minus_baseline",
            "count": len(deltas),
            "minimum": min(deltas),
            "median": statistics.median(deltas),
            "maximum": max(deltas),
        }
    return output


def _aggregate_comparator_checks(
    valid_pairs: list[dict[str, Any]],
) -> dict[str, Any]:
    check_names = sorted(
        {
            name
            for pair in valid_pairs
            for name in pair["comparison"].get("checks", {})
        }
    )
    total = len(valid_pairs)
    checks: dict[str, Any] = {}
    for name in check_names:
        passed = sum(
            pair["comparison"].get("checks", {}).get(name, {}).get("status")
            == "PASS"
            for pair in valid_pairs
        )
        checks[name] = {
            "pass_count": passed,
            "valid_pair_count": total,
            "pass_rate_percent": 100.0 * passed / total if total else None,
            "status": "PASS" if total and passed == total else "FAIL",
        }
    return checks


def summarize(
    pair_paths: list[Path],
    scenario: str,
    candidate_id: str,
    candidate_directory_name: str,
    expected_route_sha256: str,
    minimum_valid_pairs: int = 3,
    loader: Callable[[Path, str], dict[str, Any]] = control_ab.load_trial,
    comparator: Callable[
        [dict[str, Any], dict[str, Any], str, str], dict[str, Any]
    ] = control_ab.compare,
) -> dict[str, Any]:
    if scenario not in control_ab.SCENARIOS:
        raise SummaryError(f"unsupported scenario: {scenario}")
    if candidate_id not in control_ab.CANDIDATES:
        raise SummaryError(f"unsupported candidate: {candidate_id}")
    if (
        not SAFE_DIRECTORY_NAME.fullmatch(candidate_directory_name)
        or candidate_directory_name in {".", "..", BASELINE_DIRECTORY_NAME}
    ):
        raise SummaryError("candidate directory name must be one safe explicit basename")
    if not SHA256_PATTERN.fullmatch(expected_route_sha256):
        raise SummaryError("expected route SHA-256 must be 64 lowercase hex characters")
    if minimum_valid_pairs < 1:
        raise SummaryError("minimum valid pair count must be positive")
    if not pair_paths:
        raise SummaryError("at least one ordered pair directory is required")
    resolved_pairs = [path.expanduser().resolve() for path in pair_paths]
    if len(set(resolved_pairs)) != len(resolved_pairs):
        raise SummaryError("ordered pair directories must be unique")

    pairs: list[dict[str, Any]] = []
    valid_pairs: list[dict[str, Any]] = []
    for index, pair_root in enumerate(resolved_pairs, start=1):
        baseline_root = pair_root / BASELINE_DIRECTORY_NAME
        candidate_root = pair_root / candidate_directory_name
        baseline = inspect_arm(baseline_root, "baseline", expected_route_sha256)
        candidate = inspect_arm(candidate_root, "candidate", expected_route_sha256)
        row: dict[str, Any] = {
            "index": index,
            "pair_root": str(pair_root),
            "baseline": baseline,
            "candidate": candidate,
            "classification": _pair_classification(baseline, candidate),
            "valid_for_performance_statistics": False,
            "comparison_decision": None,
            "comparison": None,
        }
        # HH_260906 - Admit performance metrics only after both owned arms have selected PASS evidence.
        if row["classification"] == "READY_FOR_COMPARATOR":
            try:
                loaded_baseline = loader(baseline_root, "baseline")
                loaded_candidate = loader(candidate_root, "candidate")
                if any(
                    trial.get("route_sha256") != expected_route_sha256
                    for trial in (loaded_baseline, loaded_candidate)
                ):
                    raise SummaryError(
                        "authoritative loader route SHA-256 changed after arm admission"
                    )
                for role, trial in (
                    ("baseline", loaded_baseline),
                    ("candidate", loaded_candidate),
                ):
                    metrics = trial.get("metrics")
                    if not isinstance(metrics, dict) or any(
                        metric not in metrics or not _finite_number(metrics[metric])
                        for metric in PAIRED_METRICS
                    ):
                        raise SummaryError(
                            f"{role} lacks finite authoritative paired metrics"
                        )
                comparison = comparator(
                    loaded_baseline, loaded_candidate, scenario, candidate_id
                )
                if (
                    comparison.get("decision") not in {"ACCEPT", "HOLD"}
                    or not isinstance(comparison.get("checks"), dict)
                ):
                    raise SummaryError("authoritative comparator payload is invalid")
            except (control_ab.ComparisonError, SummaryError, ZeroDivisionError) as error:
                row["classification"] = "INVALID_COMPARATOR_CONTRACT"
                row["comparison_error"] = str(error)
            else:
                row["classification"] = f"VALID_{comparison['decision']}"
                row["valid_for_performance_statistics"] = True
                row["comparison_decision"] = comparison["decision"]
                row["comparison"] = comparison
                row["baseline_metrics"] = loaded_baseline["metrics"]
                row["candidate_metrics"] = loaded_candidate["metrics"]
                valid_pairs.append(row)
        pairs.append(row)

    arm_gates = {
        "baseline": _arm_gate_summary(pairs, "baseline"),
        "candidate": _arm_gate_summary(pairs, "candidate"),
    }
    baseline_geometry_failure_rate = arm_gates["baseline"]["geometry"][
        "failure_rate_percent_of_assessed"
    ]
    candidate_geometry_failure_rate = arm_gates["candidate"]["geometry"][
        "failure_rate_percent_of_assessed"
    ]
    geometry_nonregression = (
        baseline_geometry_failure_rate is not None
        and candidate_geometry_failure_rate is not None
        and candidate_geometry_failure_rate <= baseline_geometry_failure_rate
    )
    # HH_260906 - Keep every comparator check mandatory across every fully valid pair.
    aggregate_checks = _aggregate_comparator_checks(valid_pairs)
    comparator_each_accept = bool(valid_pairs) and all(
        pair["comparison_decision"] == "ACCEPT" for pair in valid_pairs
    )
    comparator_aggregate_accept = bool(aggregate_checks) and all(
        check["status"] == "PASS" for check in aggregate_checks.values()
    )
    gates = {
        "minimum_valid_pair_count": {
            "status": "PASS" if len(valid_pairs) >= minimum_valid_pairs else "FAIL",
            "actual": len(valid_pairs),
            "required": minimum_valid_pairs,
        },
        "all_infrastructure_gates": {
            "status": "PASS"
            if all(
                pair[role]["infrastructure_status"] == "PASS"
                for pair in pairs
                for role in ("baseline", "candidate")
            )
            else "FAIL",
            "requirement": "every baseline and candidate infrastructure gate must PASS",
        },
        "all_route_contracts": {
            "status": "PASS"
            if all(
                pair[role]["route_contract_status"] == "PASS"
                for pair in pairs
                for role in ("baseline", "candidate")
            )
            else "FAIL",
            "requirement": "every arm must bind to the expected route SHA-256",
        },
        "all_route_completions": {
            "status": "PASS"
            if all(
                pair[role]["route_status"] == "PASS"
                for pair in pairs
                for role in ("baseline", "candidate")
            )
            else "FAIL",
            "requirement": "every baseline and candidate route must PASS",
        },
        "all_geometry_assessments": {
            "status": "PASS"
            if all(
                pair[role]["geometry_status"] == "PASS"
                for pair in pairs
                for role in ("baseline", "candidate")
            )
            else "FAIL",
            "requirement": "every arm must have an assessed geometry PASS",
        },
        "candidate_geometry_failure_rate_nonregression": {
            "status": "PASS" if geometry_nonregression else "FAIL",
            "baseline_percent": baseline_geometry_failure_rate,
            "candidate_percent": candidate_geometry_failure_rate,
            "requirement": "candidate assessed geometry-failure rate must not exceed baseline",
        },
        "each_pair_comparator_acceptance": {
            "status": "PASS" if comparator_each_accept else "FAIL",
            "accept_count": sum(
                pair["comparison_decision"] == "ACCEPT" for pair in valid_pairs
            ),
            "valid_pair_count": len(valid_pairs),
            "requirement": "every fully valid pair must receive authoritative ACCEPT",
        },
        "aggregate_comparator_acceptance": {
            "status": "PASS" if comparator_aggregate_accept else "FAIL",
            "check_count": len(aggregate_checks),
            "requirement": "every authoritative comparator check must PASS in every valid pair",
        },
        "all_pairs_valid_for_statistics": {
            "status": "PASS" if len(valid_pairs) == len(pairs) else "FAIL",
            "valid_pair_count": len(valid_pairs),
            "input_pair_count": len(pairs),
            "requirement": "no invalid or geometry-failed pair may be overridden by aggregation",
        },
    }
    decision = (
        "ACCEPT"
        if all(gate["status"] == "PASS" for gate in gates.values())
        else "HOLD"
    )
    return {
        "schema_id": "autoware-e2e.repeated-control-ab-summary.v1",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "decision": decision,
        "real_vehicle_ready": False,
        "campaign_contract": {
            "scenario": scenario,
            "candidate_id": candidate_id,
            "baseline_directory_name": BASELINE_DIRECTORY_NAME,
            "candidate_directory_name": candidate_directory_name,
            "expected_route_sha256": expected_route_sha256,
            "minimum_valid_pair_count": minimum_valid_pairs,
            "ordered_pair_count": len(pairs),
        },
        "acceptance_policy": {
            "invalid_pair_policy": (
                "Arms without a selected owned PASS are classified INVALID_* or "
                "GEOMETRY_FAIL/ROUTE_FAIL and never enter paired performance statistics."
            ),
            "each_pair_comparator_policy": (
                "Every fully valid pair must receive ACCEPT from compare_control_ab.compare."
            ),
            "aggregate_comparator_policy": (
                "Every authoritative comparator check must PASS in every valid pair; "
                "median or majority results cannot override a failed pair."
            ),
            "geometry_failure_policy": (
                "Candidate assessed geometry-failure rate must not exceed baseline, "
                "and every arm must still pass route and geometry gates."
            ),
            "minimum_evidence_policy": (
                "At least the predeclared minimum number of fully valid ordered pairs is required."
            ),
        },
        "counts": {
            "input_pairs": len(pairs),
            "valid_pairs": len(valid_pairs),
            "excluded_pairs_from_performance_statistics": len(pairs)
            - len(valid_pairs),
            "classifications": dict(
                sorted(Counter(pair["classification"] for pair in pairs).items())
            ),
        },
        "arm_gate_summary": arm_gates,
        "acceptance_gates": gates,
        "aggregate_comparator_checks": aggregate_checks,
        "paired_delta_statistics": _paired_delta_summary(valid_pairs)
        if valid_pairs
        else {},
        "pairs": pairs,
    }


def render(payload: dict[str, Any], output: Path) -> None:
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.8), constrained_layout=True)
    classifications = payload["counts"]["classifications"]
    labels = list(classifications) or ["none"]
    values = [classifications[label] for label in labels] if classifications else [0]
    axes[0].barh(labels, values, color="#4c78a8")
    axes[0].set_title("Pair classifications", weight="bold")
    axes[0].set_xlabel("ordered pairs")
    axes[0].grid(axis="x", alpha=0.25)

    gate_names = ("infrastructure", "route", "geometry")
    positions = range(len(gate_names))
    baseline_rates = [
        payload["arm_gate_summary"]["baseline"][name][
            "pass_rate_percent_of_all"
        ]
        or 0.0
        for name in gate_names
    ]
    candidate_rates = [
        payload["arm_gate_summary"]["candidate"][name][
            "pass_rate_percent_of_all"
        ]
        or 0.0
        for name in gate_names
    ]
    axes[1].bar(
        [position - 0.18 for position in positions],
        baseline_rates,
        width=0.36,
        label="A baseline",
        color="#69747f",
    )
    axes[1].bar(
        [position + 0.18 for position in positions],
        candidate_rates,
        width=0.36,
        label="B candidate",
        color="#14837d",
    )
    axes[1].set_xticks(list(positions), gate_names)
    axes[1].set_ylim(0.0, 105.0)
    axes[1].set_ylabel("PASS rate across all arms (%)")
    axes[1].set_title("Arm evidence gates", weight="bold")
    axes[1].legend()
    axes[1].grid(axis="y", alpha=0.25)

    axes[2].axis("off")
    gate_lines = [
        f"{name}: {gate['status']}"
        for name, gate in payload["acceptance_gates"].items()
    ]
    text = "\n".join(
        [
            f"Decision: {payload['decision']}",
            (
                f"Valid pairs: {payload['counts']['valid_pairs']} / "
                f"{payload['counts']['input_pairs']}"
            ),
            "",
            *gate_lines,
        ]
    )
    axes[2].text(
        0.0,
        1.0,
        text,
        va="top",
        family="monospace",
        fontsize=9.2,
        bbox={"boxstyle": "round", "facecolor": "#f4f4f4", "edgecolor": "#cccccc"},
    )
    fig.suptitle(
        (
            f"Repeated 30 km/h control A/B | "
            f"{payload['campaign_contract']['scenario']} | "
            f"{payload['campaign_contract']['candidate_id']} | "
            f"{payload['decision']}"
        ),
        fontsize=15,
        weight="bold",
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=170, facecolor="white")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=control_ab.SCENARIOS, required=True)
    parser.add_argument("--candidate-id", choices=control_ab.CANDIDATES, required=True)
    parser.add_argument("--candidate-directory-name", required=True)
    parser.add_argument("--expected-route-sha256", required=True)
    parser.add_argument("--minimum-valid-pairs", type=int, default=3)
    parser.add_argument("--pair", action="append", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-png", type=Path, required=True)
    args = parser.parse_args()
    try:
        payload = summarize(
            args.pair,
            args.scenario,
            args.candidate_id,
            args.candidate_directory_name,
            args.expected_route_sha256,
            args.minimum_valid_pairs,
        )
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(
            json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        render(payload, args.output_png)
    except SummaryError as error:
        parser.error(str(error))
    print(f"REPEATED_CONTROL_AB_DECISION {payload['decision']} {args.output_json}")
    return 0 if payload["decision"] == "ACCEPT" else 1


if __name__ == "__main__":
    raise SystemExit(main())
