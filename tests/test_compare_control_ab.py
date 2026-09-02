from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path

import pytest


MODULE_PATH = Path(__file__).parents[1] / "scripts/e2e/compare_control_ab.py"
SPEC = importlib.util.spec_from_file_location("compare_control_ab", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload) + "\n", encoding="utf-8")


def _owned_fixture(
    tmp_path: Path, *, candidate: bool, route_success: bool
) -> Path:
    root = tmp_path / ("candidate" if candidate else "baseline")
    trial = root / "attempts/attempt_001"
    trial.mkdir(parents=True)
    source_route = {"town": "Town07", "scenario": "straight"}
    _write_json(trial / "source_route.json", source_route)
    route_sha256 = hashlib.sha256((trial / "source_route.json").read_bytes()).hexdigest()
    _write_json(
        trial / "result.json",
        {
            "success": route_success,
            "reason": "goal reached" if route_success else "speed exposure contract failed",
            "metrics": {
                "sim_elapsed_sec": 40.0,
                "wall_elapsed_sec": 40.1,
                "maximum_observed_speed_mps": 7.8,
                "maximum_sustained_speed_duration_sec": 3.6,
                "maximum_absolute_cte_m": 0.5,
                "maximum_lateral_acceleration_mps2": 0.8,
                "maximum_trajectory_correction_m": 8.0,
            },
        },
    )
    health = {
        "status": "PASS",
        "sequence": {"winning_window_indexes": [0, 1, 2]},
        "windows": [{"clock": {"rtf": 0.99}} for _ in range(3)],
    }
    _write_json(trial / "runtime_health.json", health)
    health_sha256 = hashlib.sha256(
        (trial / "runtime_health.json").read_bytes()
    ).hexdigest()
    _write_json(trial / "speed_profile.json", {"quality": {"problems": []}})
    _write_json(
        trial / "longitudinal_response.json",
        {
            "quality": {"problems": []},
            "tracking": {
                "gated_target_minus_actual_speed_mps": {"rmse": 1.8},
                "raw_minus_gated_target_speed_mps": {"rmse": 1.3},
            },
            "robust_measured_acceleration": {
                "robust_summary_mps2": {"p95": 1.0}
            },
            "saturation_and_duty": {
                "gated_positive_acceleration_limit": {
                    "time_fraction_percent": 40.0
                }
            },
        },
    )
    _write_json(
        trial / "diagnosis.json",
        {
            "metrics": {
                "tracking": {"actual_to_route_cte_m": {"p95_abs": 0.4}}
            }
        },
    )
    _write_json(
        trial / "latency/e2e_latency.json",
        {
            "camera_bundle": {
                "bundle_coverage_percent": 100.0,
                "receipt_span_sec": {"p95": 0.01},
            }
        },
    )
    candidate_id = "pid_i40" if candidate else "baseline"
    (trial / "runtime.env").write_text(
        "\n".join(
            (
                "SPEED_PROFILE_ID=carla_vad_30kph_v2",
                f"CONTROL_AB_CANDIDATE_ID={candidate_id}",
                f"CONTROL_AB_PID_I40={'true' if candidate else 'false'}",
                "CONTROL_AB_TURN_PREVIEW_5M=false",
                "CONTROL_AB_ISOLATED_SINGLE_KNOB=true",
                "RUNTIME_HEALTH_GATE_ENABLED=true",
                "RUNTIME_HEALTH_GATE_STATUS=PASS",
                f"RUNTIME_HEALTH_EVIDENCE_SHA256={health_sha256}",
            )
        )
        + "\n",
        encoding="utf-8",
    )
    owner_status = "PASS" if route_success else "FAIL"
    _write_json(
        root / "owned_trial_summary.json",
        {
            "schema_version": 1,
            "status": owner_status,
            "route_sha256": route_sha256,
            "selected_attempt": "attempt_001" if route_success else None,
            "attempts": [
                {
                    "attempt_id": "attempt_001",
                    "path": str(trial.resolve()),
                    "runtime_health_status": "PASS",
                    "process_exit_status": 0 if route_success else 1,
                }
            ],
        },
    )
    return root


def _trial(*, candidate: bool = False):
    return {
        "path": "/tmp/trial",
        "owner_root": "/tmp/root",
        "owner_summary": None,
        "success": True,
        "reason": "goal reached",
        "health_status": "PASS",
        "health_rtf": 0.99,
        "route_sha256": "a" * 64,
        "owner_route_sha256": "a" * 64,
        "route_town": "Town07",
        "route_scenario": "straight",
        "profile_id": "carla_vad_30kph_v2",
        "control_ab_candidate": "pid_i40" if candidate else "baseline",
        "control_ab_pid_i40": "true" if candidate else "false",
        "control_ab_turn_preview_5m": "false",
        "control_ab_isolated_single_knob": "true",
        "runtime_health_gate_enabled": "true",
        "runtime_health_gate_status": "PASS",
        "runtime_health_evidence_sha256": "b" * 64,
        "runtime_health_recorded_sha256": "b" * 64,
        "quality_problems": [],
        "metrics": {
            "route_rtf": 0.99,
            "sim_elapsed_sec": 40.0,
            "maximum_speed_mps": 7.7 if not candidate else 7.8,
            "sustained_speed_sec": 3.0 if not candidate else 3.6,
            "maximum_cte_m": 0.5,
            "maximum_lateral_acceleration_mps2": 0.8,
            "maximum_trajectory_correction_m": 8.0,
            "target_tracking_rmse_mps": 2.0 if not candidate else 1.8,
            "raw_gated_target_rmse_mps": 1.5 if not candidate else 1.3,
            "robust_acceleration_p95_mps2": 1.0,
            "gate_positive_cap_time_percent": 40.0,
            "camera_bundle_coverage_percent": 100.0,
            "camera_bundle_receipt_p95_sec": 0.01,
            "path_cte_p95_m": 0.4,
        },
    }


def test_town07_pid_candidate_is_accepted_when_all_gates_pass() -> None:
    payload = module.compare(_trial(), _trial(candidate=True), "town07_straight", "pid_i40")

    assert payload["decision"] == "ACCEPT"
    assert all(row["status"] == "PASS" for row in payload["checks"].values())


def test_health_confound_forces_hold() -> None:
    candidate = _trial(candidate=True)
    for trial in (candidate,):
        trial["route_town"] = "Town03"
        trial["route_scenario"] = "left"
        trial["control_ab_candidate"] = "turn_preview_5m"
        trial["control_ab_pid_i40"] = "false"
        trial["control_ab_turn_preview_5m"] = "true"
    baseline = _trial()
    baseline["route_town"] = "Town03"
    baseline["route_scenario"] = "left"
    candidate["health_status"] = "FAIL"
    candidate["health_rtf"] = 0.25

    payload = module.compare(baseline, candidate, "town03_turn", "turn_preview_5m")

    assert payload["decision"] == "HOLD"
    assert payload["checks"]["candidate_health"]["status"] == "FAIL"


def test_route_identity_mismatch_forces_hold() -> None:
    candidate = _trial(candidate=True)
    candidate["route_town"] = "C_track_1_0_7"
    candidate["route_scenario"] = "left"
    candidate["control_ab_candidate"] = "turn_preview_5m"
    candidate["control_ab_pid_i40"] = "false"
    candidate["control_ab_turn_preview_5m"] = "true"
    candidate["route_sha256"] = "b" * 64
    baseline = _trial()
    baseline["route_town"] = "C_track_1_0_7"
    baseline["route_scenario"] = "left"

    payload = module.compare(baseline, candidate, "c_track_turn", "turn_preview_5m")

    assert payload["decision"] == "HOLD"
    assert payload["checks"]["same_route"]["status"] == "FAIL"


def test_wrong_candidate_flag_forces_hold() -> None:
    candidate = _trial(candidate=True)
    candidate["control_ab_candidate"] = "baseline"
    candidate["control_ab_pid_i40"] = "false"

    payload = module.compare(_trial(), candidate, "town07_straight", "pid_i40")

    assert payload["decision"] == "HOLD"
    assert payload["checks"]["candidate_isolated_control"]["status"] == "FAIL"


def test_named_scenario_route_mismatch_forces_hold() -> None:
    payload = module.compare(_trial(), _trial(candidate=True), "town03_turn", "pid_i40")

    assert payload["decision"] == "HOLD"
    assert payload["checks"]["scenario_route_identity"]["status"] == "FAIL"


def test_candidate_route_failure_with_one_healthy_attempt_is_loadable(
    tmp_path: Path,
) -> None:
    root = _owned_fixture(tmp_path, candidate=True, route_success=False)

    trial = module.load_trial(root, "candidate")

    assert trial["owner_status"] == "FAIL"
    assert trial["route_outcome"] == "ROUTE_FAIL"
    assert trial["success"] is False


def test_failed_baseline_is_rejected(tmp_path: Path) -> None:
    root = _owned_fixture(tmp_path, candidate=False, route_success=False)

    with pytest.raises(module.ComparisonError, match="baseline owned trial root is not PASS"):
        module.load_trial(root, "baseline")


def test_multiple_candidate_attempts_are_rejected(tmp_path: Path) -> None:
    root = _owned_fixture(tmp_path, candidate=True, route_success=False)
    summary_path = root / "owned_trial_summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    summary["attempts"].append(dict(summary["attempts"][0]))
    _write_json(summary_path, summary)

    with pytest.raises(module.ComparisonError, match="exactly one attempt"):
        module.load_trial(root, "candidate")


def test_candidate_health_failure_is_rejected(tmp_path: Path) -> None:
    root = _owned_fixture(tmp_path, candidate=True, route_success=False)
    summary_path = root / "owned_trial_summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    summary["attempts"][0]["runtime_health_status"] = "FAIL"
    _write_json(summary_path, summary)

    with pytest.raises(module.ComparisonError, match="did not pass runtime health"):
        module.load_trial(root, "candidate")


def test_missing_candidate_evidence_is_rejected(tmp_path: Path) -> None:
    root = _owned_fixture(tmp_path, candidate=True, route_success=False)
    (root / "attempts/attempt_001/speed_profile.json").unlink()

    with pytest.raises(module.ComparisonError, match="missing required evidence"):
        module.load_trial(root, "candidate")


def test_runtime_health_evidence_tamper_is_rejected(tmp_path: Path) -> None:
    root = _owned_fixture(tmp_path, candidate=True, route_success=False)
    health_path = root / "attempts/attempt_001/runtime_health.json"
    health = json.loads(health_path.read_text(encoding="utf-8"))
    health["tampered"] = True
    _write_json(health_path, health)

    with pytest.raises(module.ComparisonError, match="evidence digest mismatch"):
        module.load_trial(root, "candidate")
