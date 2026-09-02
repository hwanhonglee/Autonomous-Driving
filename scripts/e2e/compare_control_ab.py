#!/usr/bin/env python3
"""Compare one health-admitted 30 km/h control A/B pair fail-closed."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


SCENARIOS = ("town07_straight", "c_track_turn", "town03_turn")
CANDIDATES = ("pid_i40", "turn_preview_5m")


class ComparisonError(RuntimeError):
    pass


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise ComparisonError(f"cannot read JSON {path}: {error}") from error
    if not isinstance(value, dict):
        raise ComparisonError(f"JSON root must be an object: {path}")
    return value


def _nested(value: dict[str, Any], *keys: str) -> Any:
    current: Any = value
    try:
        for key in keys:
            current = current[key]
    except (KeyError, TypeError) as error:
        raise ComparisonError(f"missing field {'.'.join(keys)}") from error
    return current


def _number(value: Any, label: str) -> float:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise ComparisonError(f"{label} is not numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ComparisonError(f"{label} is not finite")
    return result


def resolve_trial(path: Path, role: str) -> tuple[Path, dict[str, Any]]:
    if role not in {"baseline", "candidate"}:
        raise ComparisonError(f"unsupported trial role: {role}")
    root = path.expanduser().resolve()
    owner_summary = root / "owned_trial_summary.json"
    if not owner_summary.is_file():
        raise ComparisonError(f"{role} must be an owned trial root: {root}")
    summary = _read_json(owner_summary)
    if summary.get("schema_version") != 1:
        raise ComparisonError(f"{role} owner summary schema is unsupported: {root}")
    attempts = summary.get("attempts")
    if not isinstance(attempts, list) or len(attempts) != 1:
        raise ComparisonError(
            f"{role} owner summary must contain exactly one attempt: {root}"
        )
    attempt = attempts[0]
    if not isinstance(attempt, dict):
        raise ComparisonError(f"{role} owner attempt is not an object: {root}")
    attempt_id = attempt.get("attempt_id")
    if not isinstance(attempt_id, str) or not attempt_id:
        raise ComparisonError(f"{role} owner attempt has no ID: {root}")
    if attempt.get("runtime_health_status") != "PASS":
        raise ComparisonError(f"{role} owner attempt did not pass runtime health: {root}")
    process_status = attempt.get("process_exit_status")
    if not isinstance(process_status, int) or isinstance(process_status, bool):
        raise ComparisonError(f"{role} owner attempt exit status is invalid: {root}")
    owner_status = summary.get("status")
    selected = summary.get("selected_attempt")
    if owner_status == "PASS":
        if selected != attempt_id or process_status != 0:
            raise ComparisonError(
                f"{role} PASS owner summary has inconsistent attempt selection: {root}"
            )
    elif owner_status == "FAIL" and role == "candidate":
        if selected is not None or process_status == 0:
            raise ComparisonError(
                f"candidate FAIL owner summary is not a route-level failure: {root}"
            )
    else:
        raise ComparisonError(f"{role} owned trial root is not PASS: {root}")
    trial = (root / "attempts" / attempt_id).resolve()
    if not trial.is_dir() or root not in trial.parents:
        raise ComparisonError(f"{role} attempt escapes its owner root: {trial}")
    recorded_path = attempt.get("path")
    if not isinstance(recorded_path, str) or Path(recorded_path).resolve() != trial:
        raise ComparisonError(f"{role} owner attempt path does not match its ID: {root}")
    return trial, summary


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_env(path: Path) -> dict[str, str]:
    output: dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        if not raw_line or raw_line.startswith("#") or "=" not in raw_line:
            continue
        key, value = raw_line.split("=", 1)
        if key in output:
            raise ComparisonError(f"duplicate runtime.env key {key}: {path}")
        output[key] = value
    return output


def load_trial(path: Path, role: str) -> dict[str, Any]:
    trial, owner = resolve_trial(path, role)
    required = (
        "result.json",
        "runtime.env",
        "runtime_health.json",
        "source_route.json",
        "speed_profile.json",
        "longitudinal_response.json",
        "diagnosis.json",
        "latency/e2e_latency.json",
    )
    missing = [name for name in required if not (trial / name).is_file()]
    if missing:
        raise ComparisonError(f"trial is missing required evidence: {missing}: {trial}")
    result = _read_json(trial / "result.json")
    health = _read_json(trial / "runtime_health.json")
    speed = _read_json(trial / "speed_profile.json")
    longitudinal = _read_json(trial / "longitudinal_response.json")
    diagnosis = _read_json(trial / "diagnosis.json")
    latency = _read_json(trial / "latency/e2e_latency.json")
    source_route = _read_json(trial / "source_route.json")
    environment = parse_env(trial / "runtime.env")
    result_success = result.get("success")
    if not isinstance(result_success, bool):
        raise ComparisonError(f"{role} route result has no boolean success field")
    if role == "baseline" and result_success is not True:
        raise ComparisonError("baseline route result must be PASS")
    if owner["status"] == "PASS" and result_success is not True:
        raise ComparisonError(f"{role} PASS owner contradicts its route result")
    if owner["status"] == "FAIL" and result_success is not False:
        raise ComparisonError(
            "candidate FAIL owner is not attributable to a route-level failure"
        )
    metrics = _nested(result, "metrics")
    sim_elapsed = _number(metrics.get("sim_elapsed_sec"), "sim elapsed")
    wall_elapsed = _number(metrics.get("wall_elapsed_sec"), "wall elapsed")
    if wall_elapsed <= 0.0:
        raise ComparisonError("wall elapsed must be positive")
    winning_indexes = health.get("sequence", {}).get("winning_window_indexes", [])
    windows = health.get("windows", [])
    if not isinstance(winning_indexes, list) or not winning_indexes:
        raise ComparisonError("runtime health has no winning window sequence")
    try:
        winning_rtfs = [windows[int(index)]["clock"]["rtf"] for index in winning_indexes]
    except (IndexError, KeyError, TypeError, ValueError) as error:
        raise ComparisonError("runtime health winning-window evidence is invalid") from error
    health_rtf = min(_number(value, "health window RTF") for value in winning_rtfs)
    route_sha256 = _sha256(trial / "source_route.json")
    owner_route_sha256 = owner.get("route_sha256") if owner is not None else None
    health_sha256 = _sha256(trial / "runtime_health.json")
    if health.get("status") != "PASS":
        raise ComparisonError(f"{role} runtime health evidence is not PASS")
    if environment.get("RUNTIME_HEALTH_GATE_ENABLED") != "true":
        raise ComparisonError(f"{role} runtime health gate was not enabled")
    if environment.get("RUNTIME_HEALTH_GATE_STATUS") != "PASS":
        raise ComparisonError(f"{role} runtime health provenance is not PASS")
    if environment.get("RUNTIME_HEALTH_EVIDENCE_SHA256") != health_sha256:
        raise ComparisonError(f"{role} runtime health evidence digest mismatch")
    if owner_route_sha256 != route_sha256:
        raise ComparisonError(f"{role} owner/source route digest mismatch")
    return {
        "path": str(trial),
        "owner_root": str(path.expanduser().resolve()),
        "owner_summary": owner,
        "owner_status": owner["status"],
        "route_outcome": "PASS" if result_success else "ROUTE_FAIL",
        "success": result_success,
        "reason": result.get("reason"),
        "health_status": health.get("status"),
        "health_rtf": _number(health_rtf, "health real-time factor"),
        "route_sha256": route_sha256,
        "owner_route_sha256": owner_route_sha256,
        "route_town": source_route.get("town"),
        "route_scenario": source_route.get("scenario"),
        "profile_id": environment.get("SPEED_PROFILE_ID"),
        "control_ab_candidate": environment.get(
            "CONTROL_AB_CANDIDATE_ID", "baseline"
        ),
        "control_ab_pid_i40": environment.get("CONTROL_AB_PID_I40"),
        "control_ab_turn_preview_5m": environment.get(
            "CONTROL_AB_TURN_PREVIEW_5M"
        ),
        "control_ab_isolated_single_knob": environment.get(
            "CONTROL_AB_ISOLATED_SINGLE_KNOB"
        ),
        "runtime_health_gate_enabled": environment.get(
            "RUNTIME_HEALTH_GATE_ENABLED"
        ),
        "runtime_health_gate_status": environment.get(
            "RUNTIME_HEALTH_GATE_STATUS"
        ),
        "runtime_health_evidence_sha256": health_sha256,
        "runtime_health_recorded_sha256": environment.get(
            "RUNTIME_HEALTH_EVIDENCE_SHA256"
        ),
        "quality_problems": list(longitudinal.get("quality", {}).get("problems", []))
        + list(speed.get("quality", {}).get("problems", [])),
        "metrics": {
            "route_rtf": sim_elapsed / wall_elapsed,
            "sim_elapsed_sec": sim_elapsed,
            "maximum_speed_mps": _number(
                metrics.get("maximum_observed_speed_mps"), "maximum speed"
            ),
            "sustained_speed_sec": _number(
                metrics.get("maximum_sustained_speed_duration_sec"),
                "sustained speed duration",
            ),
            "maximum_cte_m": _number(
                metrics.get("maximum_absolute_cte_m"), "maximum CTE"
            ),
            "maximum_lateral_acceleration_mps2": _number(
                metrics.get("maximum_lateral_acceleration_mps2"),
                "maximum lateral acceleration",
            ),
            "maximum_trajectory_correction_m": _number(
                metrics.get("maximum_trajectory_correction_m"),
                "maximum trajectory correction",
            ),
            "target_tracking_rmse_mps": _number(
                _nested(
                    longitudinal,
                    "tracking",
                    "gated_target_minus_actual_speed_mps",
                    "rmse",
                ),
                "target tracking RMSE",
            ),
            "raw_gated_target_rmse_mps": _number(
                _nested(
                    longitudinal,
                    "tracking",
                    "raw_minus_gated_target_speed_mps",
                    "rmse",
                ),
                "raw-gated target RMSE",
            ),
            "robust_acceleration_p95_mps2": _number(
                _nested(
                    longitudinal,
                    "robust_measured_acceleration",
                    "robust_summary_mps2",
                    "p95",
                ),
                "robust acceleration p95",
            ),
            "gate_positive_cap_time_percent": _number(
                _nested(
                    longitudinal,
                    "saturation_and_duty",
                    "gated_positive_acceleration_limit",
                    "time_fraction_percent",
                ),
                "gate positive cap time",
            ),
            "camera_bundle_coverage_percent": _number(
                _nested(latency, "camera_bundle", "bundle_coverage_percent"),
                "camera bundle coverage",
            ),
            "camera_bundle_receipt_p95_sec": _number(
                _nested(
                    latency,
                    "camera_bundle",
                    "receipt_span_sec",
                    "p95",
                ),
                "camera bundle receipt p95",
            ),
            "path_cte_p95_m": _number(
                _nested(
                    diagnosis,
                    "metrics",
                    "tracking",
                    "actual_to_route_cte_m",
                    "p95_abs",
                ),
                "path CTE p95",
            ),
        },
    }


def compare(
    baseline: dict[str, Any], candidate: dict[str, Any], scenario: str, candidate_id: str
) -> dict[str, Any]:
    if scenario not in SCENARIOS or candidate_id not in CANDIDATES:
        raise ComparisonError("unsupported scenario or candidate")
    a = baseline["metrics"]
    b = candidate["metrics"]
    checks: dict[str, dict[str, Any]] = {}

    def check(name: str, passed: bool, actual: Any, requirement: str) -> None:
        checks[name] = {
            "status": "PASS" if passed else "FAIL",
            "actual": actual,
            "requirement": requirement,
        }

    expected_routes = {
        "town07_straight": ("Town07", {"straight"}),
        "c_track_turn": ("C_track_1_0_7", {"left", "right"}),
        "town03_turn": ("Town03", {"left", "right"}),
    }
    expected_town, expected_route_scenarios = expected_routes[scenario]
    check("same_route", baseline["route_sha256"] == candidate["route_sha256"],
          {"baseline": baseline["route_sha256"], "candidate": candidate["route_sha256"]},
          "source_route SHA-256 must be identical")
    check(
        "scenario_route_identity",
        all(
            trial["route_town"] == expected_town
            and trial["route_scenario"] in expected_route_scenarios
            for trial in (baseline, candidate)
        ),
        {
            "expected_town": expected_town,
            "expected_route_scenarios": sorted(expected_route_scenarios),
            "baseline": {
                "town": baseline["route_town"],
                "scenario": baseline["route_scenario"],
            },
            "candidate": {
                "town": candidate["route_town"],
                "scenario": candidate["route_scenario"],
            },
        },
        "both source routes must match the named A/B scenario",
    )
    expected_baseline_control = ("baseline", "false", "false", "true")
    expected_candidate_control = (
        candidate_id,
        "true" if candidate_id == "pid_i40" else "false",
        "true" if candidate_id == "turn_preview_5m" else "false",
        "true",
    )
    for label, trial, expected_control in (
        ("baseline", baseline, expected_baseline_control),
        ("candidate", candidate, expected_candidate_control),
    ):
        actual_control = (
            trial["control_ab_candidate"],
            trial["control_ab_pid_i40"],
            trial["control_ab_turn_preview_5m"],
            trial["control_ab_isolated_single_knob"],
        )
        check(
            f"{label}_profile",
            trial["profile_id"] == "carla_vad_30kph_v2",
            trial["profile_id"],
            "SPEED_PROFILE_ID must be carla_vad_30kph_v2",
        )
        check(
            f"{label}_isolated_control",
            actual_control == expected_control,
            {
                "candidate_id": actual_control[0],
                "pid_i40": actual_control[1],
                "turn_preview_5m": actual_control[2],
                "isolated_single_knob": actual_control[3],
            },
            f"isolated control tuple must equal {expected_control}",
        )
        check(
            f"{label}_health_integrity",
            trial["runtime_health_gate_enabled"] == "true"
            and trial["runtime_health_gate_status"] == "PASS"
            and trial["runtime_health_recorded_sha256"]
            == trial["runtime_health_evidence_sha256"]
            and (
                trial["owner_route_sha256"] is None
                or trial["owner_route_sha256"] == trial["route_sha256"]
            ),
            {
                "gate_enabled": trial["runtime_health_gate_enabled"],
                "gate_status": trial["runtime_health_gate_status"],
                "recorded_health_sha256": trial["runtime_health_recorded_sha256"],
                "actual_health_sha256": trial["runtime_health_evidence_sha256"],
                "owner_route_sha256": trial["owner_route_sha256"],
                "source_route_sha256": trial["route_sha256"],
            },
            "health evidence digest and owned-route digest must match recorded provenance",
        )
    for label, trial in (("baseline", baseline), ("candidate", candidate)):
        m = trial["metrics"]
        check(f"{label}_goal", trial["success"], trial["reason"], "goal/success required")
        check(f"{label}_health", trial["health_status"] == "PASS" and trial["health_rtf"] >= 0.9,
              {"status": trial["health_status"], "rtf": trial["health_rtf"]},
              "pre-engagement health PASS and RTF >= 0.9")
        check(f"{label}_route_rtf", m["route_rtf"] >= 0.9, m["route_rtf"], "route RTF >= 0.9")
        check(f"{label}_camera", m["camera_bundle_coverage_percent"] >= 99.0 and m["camera_bundle_receipt_p95_sec"] <= 0.04,
              {"coverage_percent": m["camera_bundle_coverage_percent"], "receipt_p95_sec": m["camera_bundle_receipt_p95_sec"]},
              "coverage >= 99% and six-camera receipt p95 <= 40 ms")
        check(f"{label}_quality", not trial["quality_problems"], trial["quality_problems"], "analysis problems must be empty")
    check("candidate_speed_ceiling", b["maximum_speed_mps"] <= 9.0,
          b["maximum_speed_mps"], "maximum speed <= 9.0 m/s")
    check("candidate_cte", b["maximum_cte_m"] <= 0.65, b["maximum_cte_m"], "maximum CTE <= 0.65 m")
    check("candidate_lateral_acceleration", b["maximum_lateral_acceleration_mps2"] <= 1.5,
          b["maximum_lateral_acceleration_mps2"], "maximum lateral acceleration <= 1.5 m/s^2")
    check("candidate_robust_acceleration", b["robust_acceleration_p95_mps2"] <= 1.10,
          b["robust_acceleration_p95_mps2"], "robust longitudinal acceleration p95 <= 1.10 m/s^2")
    check("candidate_correction_delta", b["maximum_trajectory_correction_m"] <= a["maximum_trajectory_correction_m"] + 0.25,
          b["maximum_trajectory_correction_m"] - a["maximum_trajectory_correction_m"],
          "trajectory correction increase <= 0.25 m")

    if scenario == "town07_straight" and candidate_id == "pid_i40":
        exposure = b["maximum_speed_mps"] >= 7.78 or b["sustained_speed_sec"] >= 3.5
        check("straight_speed_exposure", exposure,
              {"maximum_speed_mps": b["maximum_speed_mps"], "sustained_speed_sec": b["sustained_speed_sec"]},
              "maximum >= 7.78 m/s or >=7.5 m/s sustained >=3.5 s")
        check("straight_tracking_improvement", b["target_tracking_rmse_mps"] <= 0.95 * a["target_tracking_rmse_mps"],
              b["target_tracking_rmse_mps"] / a["target_tracking_rmse_mps"],
              "target tracking RMSE <= 95% of paired baseline")
        check("gate_cap_growth", b["gate_positive_cap_time_percent"] <= min(46.5, a["gate_positive_cap_time_percent"] + 5.0),
              b["gate_positive_cap_time_percent"], "gate positive-cap time <= 46.5% and baseline + 5 points")
    elif candidate_id == "turn_preview_5m":
        check("turn_raw_gated_improvement", b["raw_gated_target_rmse_mps"] <= 0.90 * a["raw_gated_target_rmse_mps"],
              b["raw_gated_target_rmse_mps"] / a["raw_gated_target_rmse_mps"],
              "raw-to-gated target RMSE <= 90% of paired baseline")
        check("turn_tracking_nonworsening", b["target_tracking_rmse_mps"] <= 1.05 * a["target_tracking_rmse_mps"],
              b["target_tracking_rmse_mps"] / a["target_tracking_rmse_mps"],
              "target tracking RMSE <= 105% of baseline")
        check("turn_speed_retention", b["maximum_speed_mps"] >= 0.95 * a["maximum_speed_mps"],
              b["maximum_speed_mps"] / a["maximum_speed_mps"], "maximum speed >= 95% of baseline")
        check("turn_completion_time", b["sim_elapsed_sec"] <= 1.10 * a["sim_elapsed_sec"],
              b["sim_elapsed_sec"] / a["sim_elapsed_sec"], "sim completion time <= 110% of baseline")
    else:
        check("turn_pid_tracking", b["target_tracking_rmse_mps"] <= 1.05 * a["target_tracking_rmse_mps"],
              b["target_tracking_rmse_mps"] / a["target_tracking_rmse_mps"], "turn tracking RMSE <= 105% of baseline")

    accepted = all(row["status"] == "PASS" for row in checks.values())
    return {
        "schema_version": 1,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "scenario": scenario,
        "candidate_id": candidate_id,
        "baseline": baseline,
        "candidate": candidate,
        "checks": checks,
        "decision": "ACCEPT" if accepted else "HOLD",
        "real_vehicle_ready": False,
    }


def render(payload: dict[str, Any], output: Path) -> None:
    a = payload["baseline"]["metrics"]
    b = payload["candidate"]["metrics"]
    fields = (
        ("maximum_speed_mps", "Maximum speed", "m/s", True),
        ("target_tracking_rmse_mps", "Target tracking RMSE", "m/s", False),
        ("raw_gated_target_rmse_mps", "Raw-to-gated RMSE", "m/s", False),
        ("maximum_cte_m", "Maximum CTE", "m", False),
        ("maximum_lateral_acceleration_mps2", "Maximum lateral accel", "m/s²", False),
        ("camera_bundle_receipt_p95_sec", "Camera-bundle receipt p95", "s", False),
    )
    fig, axes = plt.subplots(2, 3, figsize=(13.5, 7.7), constrained_layout=True)
    for axis, (key, title, unit, higher_better) in zip(axes.flat, fields):
        values = [a[key], b[key]]
        better = values[1] >= values[0] if higher_better else values[1] <= values[0]
        axis.bar((0, 1), values, color=("#69747f", "#14837d" if better else "#c44e3b"))
        axis.set_xticks((0, 1), ("A baseline", "B candidate"))
        axis.set_title(title, weight="bold")
        axis.set_ylabel(unit)
        axis.grid(axis="y", alpha=0.25)
        axis.spines[["top", "right"]].set_visible(False)
    fig.suptitle(
        f"30 km/h control A/B | {payload['scenario']} | {payload['candidate_id']} | {payload['decision']}",
        fontsize=15,
        weight="bold",
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=170, facecolor="white")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=SCENARIOS, required=True)
    parser.add_argument("--candidate-id", choices=CANDIDATES, required=True)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-png", type=Path, required=True)
    args = parser.parse_args()
    try:
        payload = compare(
            load_trial(args.baseline, "baseline"),
            load_trial(args.candidate, "candidate"),
            args.scenario,
            args.candidate_id,
        )
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        render(payload, args.output_png)
    except ComparisonError as error:
        parser.error(str(error))
    print(f"CONTROL_AB_DECISION {payload['decision']} {args.output_json}")
    return 0 if payload["decision"] == "ACCEPT" else 1


if __name__ == "__main__":
    raise SystemExit(main())
