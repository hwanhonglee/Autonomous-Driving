from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
from typing import Any

import pytest


MODULE_PATH = (
    Path(__file__).parents[1]
    / "scripts"
    / "e2e"
    / "summarize_owned_route_repeats.py"
)
SPEC = importlib.util.spec_from_file_location(
    "summarize_owned_route_repeats", MODULE_PATH
)
assert SPEC is not None and SPEC.loader is not None
summary_tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(summary_tool)


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def _shadow_fixture() -> dict[str, Any]:
    return {
        "schema_id": "autoware-e2e.portable-shadow-evidence-analysis.v2",
        "analysis_status": "EVIDENCE_VALID",
        "claims": {
            "ten_hz": {
                "pass": True,
                "status": "ESTABLISHED_FOR_SHADOW_ONLY",
                "requirements": {
                    "anchor_rate_hz": {"observed": 10.0},
                    "path_receipt_rate_hz": {"observed": 9.99},
                    "trajectory_receipt_rate_hz": {"observed": 9.98},
                    "continuous_source_anchor_period": {
                        "met": True,
                        "observed_violation_count": 0,
                    },
                    "zero_rejections_arm_to_seal": {"met": True},
                    "zero_silent_runtime_losses_arm_to_seal": {
                        "met": True,
                        "observed_terminal_pending_bundle_count": 0,
                    },
                    "pre_startup_only_initial_waiting_and_zero_losses": {
                        "met": True,
                        "observed_drop_count": 0,
                        "observed_rejection_count": 0,
                        "observed_timeout_count": 0,
                    },
                },
            }
        },
        "accepted_latency": {"p99_ms": 40.0},
        "accepted_status_wall_timing": {"maximum_gap_s": 0.15},
        "status_accounting": {
            "denominators": {
                "anchor_rejected_count": 0,
                "input_event_rejected_count": 0,
            },
            "health_snapshot_counts": {
                "rejected": 0,
                "camera_bundle_timeout": 0,
            },
            "camera_bundle_cumulative_delta": {
                "dropped_stale_count": 0,
                "evicted_capacity_count": 0,
                "expired_pending_count": 0,
            },
            "input_settle": {"input_settle_timeout_count": 0},
            "rejection_counts_by_stage": {
                "acceleration": 0,
                "camera_info": 0,
                "image": 0,
                "inference": 0,
                "odometry": 0,
                "steering": 0,
            },
        },
    }


def _create_run(
    root: Path,
    name: str,
    *,
    cte: float = 0.5,
    correction: float = 12.0,
    lateral_acceleration: float = 0.7,
) -> tuple[Path, str]:
    run = root / name
    attempt = run / "attempts" / "attempt_001"
    attempt.mkdir(parents=True)
    source_route = {
        "town": "C_track_1_0_7",
        "scenario": "left",
        "route": [{"x": 1.0, "y": 2.0}, {"x": 3.0, "y": 4.0}],
    }
    _write_json(attempt / "source_route.json", source_route)
    route_sha256 = hashlib.sha256(
        (attempt / "source_route.json").read_bytes()
    ).hexdigest()
    _write_json(
        attempt / "result.json",
        {
            "schema_version": 1,
            "success": True,
            "reason": "goal reached",
            "metrics": {
                "maximum_absolute_cte_m": cte,
                "maximum_trajectory_correction_m": correction,
                "maximum_lateral_acceleration_mps2": lateral_acceleration,
            },
        },
    )
    _write_json(
        attempt / "runtime_health.json",
        {"schema_version": 1, "status": "PASS"},
    )
    _write_json(
        attempt / "diagnosis.json",
        {
            "schema_version": 2,
            "verdict": {"classification": "path_dominant"},
            "metrics": {
                "tracking": {
                    "actual_to_final_cte_m": {"p95_abs": 0.2},
                    "actual_to_route_cte_m": {"p95_abs": 0.5},
                },
                "raw_to_final_geometry": {
                    "common_progress_p95_m": {"p95_abs": 8.4}
                },
                "raw_path": {"p95_abs": 6.3},
            },
        },
    )
    _write_json(
        attempt
        / "portable_shadow_provenance"
        / "shadow_evidence_analysis.json",
        _shadow_fixture(),
    )
    _write_json(
        run / "owned_trial_summary.json",
        {
            "schema_version": 1,
            "status": "PASS",
            "route_sha256": route_sha256,
            "selected_attempt": "attempt_001",
            "attempts": [
                {
                    "attempt_id": "attempt_001",
                    "path": str(attempt.resolve()),
                    "runtime_health_status": "PASS",
                    "process_exit_status": 0,
                }
            ],
        },
    )
    return run, route_sha256


def _mutate_json(path: Path, keys: tuple[str, ...], value: Any) -> None:
    payload = json.loads(path.read_text(encoding="utf-8"))
    target = payload
    for key in keys[:-1]:
        target = target[key]
    target[keys[-1]] = value
    _write_json(path, payload)


def test_pass_summary_contains_min_median_max_statistics(tmp_path: Path) -> None:
    run_a, route_sha256 = _create_run(tmp_path, "baseline_repeat_01", cte=0.4)
    run_b, second_sha256 = _create_run(tmp_path, "baseline_repeat_02", cte=0.6)
    assert second_sha256 == route_sha256

    summary = summary_tool.build_summary([run_a, run_b], route_sha256)

    assert summary["status"] == "PASS"
    assert summary["pass_count"] == 2
    assert summary["fail_count"] == 0
    assert summary["statistics"]["maximum_cte_m"] == {
        "count": 2,
        "minimum": pytest.approx(0.4),
        "median": pytest.approx(0.5),
        "maximum": pytest.approx(0.6),
        "values": [pytest.approx(0.4), pytest.approx(0.6)],
    }
    assert all(check["status"] == "PASS" for check in summary["campaign_checks"].values())
    assert all(
        check["status"] == "PASS"
        for check in summary["campaign_gate_checks"].values()
    )


def test_inclusive_threshold_values_pass(tmp_path: Path) -> None:
    run, route_sha256 = _create_run(
        tmp_path,
        "baseline_repeat_01",
        cte=1.0,
        correction=15.0,
        lateral_acceleration=1.8,
    )
    shadow_path = (
        run
        / "attempts/attempt_001/portable_shadow_provenance"
        / "shadow_evidence_analysis.json"
    )
    shadow = json.loads(shadow_path.read_text(encoding="utf-8"))
    requirements = shadow["claims"]["ten_hz"]["requirements"]
    requirements["anchor_rate_hz"]["observed"] = 9.5
    requirements["path_receipt_rate_hz"]["observed"] = 10.5
    requirements["trajectory_receipt_rate_hz"]["observed"] = 9.5
    shadow["accepted_latency"]["p99_ms"] = 100.0
    shadow["accepted_status_wall_timing"]["maximum_gap_s"] = 0.2
    _write_json(shadow_path, shadow)

    summary = summary_tool.build_summary([run], route_sha256)

    assert summary["status"] == "PASS"


@pytest.mark.parametrize(
    ("relative", "keys", "value", "failed_check"),
    (
        (
            "result.json",
            ("metrics", "maximum_absolute_cte_m"),
            1.000001,
            "cte_limit",
        ),
        (
            "result.json",
            ("metrics", "maximum_trajectory_correction_m"),
            15.000001,
            "trajectory_correction_limit",
        ),
        (
            "result.json",
            ("metrics", "maximum_lateral_acceleration_mps2"),
            1.800001,
            "lateral_acceleration_limit",
        ),
        (
            "portable_shadow_provenance/shadow_evidence_analysis.json",
            ("claims", "ten_hz", "requirements", "anchor_rate_hz", "observed"),
            9.499999,
            "source_rate",
        ),
        (
            "portable_shadow_provenance/shadow_evidence_analysis.json",
            ("claims", "ten_hz", "requirements", "path_receipt_rate_hz", "observed"),
            10.500001,
            "path_rate",
        ),
        (
            "portable_shadow_provenance/shadow_evidence_analysis.json",
            (
                "claims",
                "ten_hz",
                "requirements",
                "trajectory_receipt_rate_hz",
                "observed",
            ),
            10.500001,
            "trajectory_rate",
        ),
        (
            "portable_shadow_provenance/shadow_evidence_analysis.json",
            (
                "claims",
                "ten_hz",
                "requirements",
                "continuous_source_anchor_period",
                "observed_violation_count",
            ),
            1,
            "source_period_continuity",
        ),
        (
            "portable_shadow_provenance/shadow_evidence_analysis.json",
            ("accepted_latency", "p99_ms"),
            100.000001,
            "inference_latency",
        ),
        (
            "portable_shadow_provenance/shadow_evidence_analysis.json",
            ("accepted_status_wall_timing", "maximum_gap_s"),
            0.200001,
            "status_wall_gap",
        ),
        (
            "portable_shadow_provenance/shadow_evidence_analysis.json",
            (
                "status_accounting",
                "camera_bundle_cumulative_delta",
                "dropped_stale_count",
            ),
            1,
            "zero_runtime_losses",
        ),
        (
            "portable_shadow_provenance/shadow_evidence_analysis.json",
            ("analysis_status",),
            "EVIDENCE_INVALID",
            "shadow_evidence_valid",
        ),
        (
            "portable_shadow_provenance/shadow_evidence_analysis.json",
            ("claims", "ten_hz", "pass"),
            False,
            "shadow_ten_hz_pass",
        ),
    ),
)
def test_threshold_violation_is_an_explicit_campaign_failure(
    tmp_path: Path,
    relative: str,
    keys: tuple[str, ...],
    value: Any,
    failed_check: str,
) -> None:
    run, route_sha256 = _create_run(tmp_path, "baseline_repeat_01")
    _mutate_json(run / "attempts/attempt_001" / relative, keys, value)

    summary = summary_tool.build_summary([run], route_sha256)

    assert summary["status"] == "FAIL"
    assert summary["runs"][0]["checks"][failed_check]["status"] == "FAIL"
    assert summary["gate_pass_counts"][failed_check]["pass_count"] == 0
    assert summary["campaign_gate_checks"][failed_check]["status"] == "FAIL"


def test_missing_evidence_is_recorded_without_raising(tmp_path: Path) -> None:
    run, route_sha256 = _create_run(tmp_path, "baseline_repeat_01")
    (run / "attempts/attempt_001/diagnosis.json").unlink()

    summary = summary_tool.build_summary([run], route_sha256)

    trial = summary["runs"][0]
    assert summary["status"] == "FAIL"
    assert trial["checks"]["diagnosis_valid"]["status"] == "FAIL"
    assert any("missing turn diagnosis" in reason for reason in trial["evidence_errors"])


def test_malformed_evidence_is_recorded_without_raising(tmp_path: Path) -> None:
    run, route_sha256 = _create_run(tmp_path, "baseline_repeat_01")
    (run / "attempts/attempt_001/result.json").write_text(
        '{"success": true,', encoding="utf-8"
    )

    summary = summary_tool.build_summary([run], route_sha256)

    trial = summary["runs"][0]
    assert summary["status"] == "FAIL"
    assert trial["checks"]["goal_reached"]["status"] == "FAIL"
    assert any("cannot read route result" in reason for reason in trial["evidence_errors"])


def test_wrong_metric_type_is_recorded_without_raising(tmp_path: Path) -> None:
    run, route_sha256 = _create_run(tmp_path, "baseline_repeat_01")
    _mutate_json(
        run / "attempts/attempt_001/result.json",
        ("metrics", "maximum_absolute_cte_m"),
        True,
    )

    summary = summary_tool.build_summary([run], route_sha256)

    trial = summary["runs"][0]
    assert summary["status"] == "FAIL"
    assert trial["checks"]["cte_limit"]["status"] == "FAIL"
    assert any("maximum absolute CTE must be numeric" in reason for reason in trial["reasons"])


def test_route_mismatch_fails_only_the_exact_route_gate(tmp_path: Path) -> None:
    run, route_sha256 = _create_run(tmp_path, "baseline_repeat_01")
    wrong_sha256 = "0" * 64 if route_sha256 != "0" * 64 else "1" * 64

    summary = summary_tool.build_summary([run], wrong_sha256)

    trial = summary["runs"][0]
    assert summary["status"] == "FAIL"
    assert trial["checks"]["exact_route_sha256"]["status"] == "FAIL"
    assert trial["checks"]["goal_reached"]["status"] == "PASS"


def test_duplicate_selected_attempt_rows_are_ambiguous(tmp_path: Path) -> None:
    run, route_sha256 = _create_run(tmp_path, "baseline_repeat_01")
    owner_path = run / "owned_trial_summary.json"
    owner = json.loads(owner_path.read_text(encoding="utf-8"))
    owner["attempts"].append(copy.deepcopy(owner["attempts"][0]))
    _write_json(owner_path, owner)

    summary = summary_tool.build_summary([run], route_sha256)

    trial = summary["runs"][0]
    assert summary["status"] == "FAIL"
    assert trial["checks"]["selected_attempt_unambiguous"]["status"] == "FAIL"
    assert trial["checks"]["goal_reached"]["status"] == "FAIL"


@pytest.mark.parametrize(
    "recorded_path",
    ("~definitely_no_such_user_260906/attempt_001", "bad\x00attempt"),
)
def test_malformed_selected_attempt_path_is_a_failed_verdict(
    tmp_path: Path, recorded_path: str
) -> None:
    run, route_sha256 = _create_run(tmp_path, "baseline_repeat_01")
    owner_path = run / "owned_trial_summary.json"
    owner = json.loads(owner_path.read_text(encoding="utf-8"))
    owner["attempts"][0]["path"] = recorded_path
    _write_json(owner_path, owner)

    summary = summary_tool.build_summary([run], route_sha256)

    trial = summary["runs"][0]
    assert summary["status"] == "FAIL"
    assert trial["checks"]["selected_attempt_unambiguous"]["status"] == "FAIL"
    assert any("cannot resolve selected attempt path" in reason for reason in trial["reasons"])


def test_owner_failure_remains_fail_closed_while_evidence_is_readable(
    tmp_path: Path,
) -> None:
    run, route_sha256 = _create_run(tmp_path, "baseline_repeat_01")
    _mutate_json(run / "owned_trial_summary.json", ("status",), "FAIL")

    summary = summary_tool.build_summary([run], route_sha256)

    trial = summary["runs"][0]
    assert summary["status"] == "FAIL"
    assert trial["checks"]["owner_pass"]["status"] == "FAIL"
    assert trial["checks"]["goal_reached"]["status"] == "PASS"


def test_cli_writes_pass_and_fail_reports_with_png(tmp_path: Path) -> None:
    pass_run, route_sha256 = _create_run(tmp_path, "baseline_repeat_01")
    pass_json = tmp_path / "reports/pass.json"
    pass_png = tmp_path / "reports/pass.png"
    pass_process = subprocess.run(
        [
            sys.executable,
            str(MODULE_PATH),
            "--run",
            str(pass_run),
            "--expected-route-sha256",
            route_sha256,
            "--output-json",
            str(pass_json),
            "--output-png",
            str(pass_png),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert pass_process.returncode == 0, pass_process.stderr
    assert json.loads(pass_json.read_text(encoding="utf-8"))["status"] == "PASS"
    assert pass_png.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert pass_png.stat().st_size > 1000

    fail_run, fail_sha256 = _create_run(tmp_path, "baseline_repeat_02")
    (fail_run / "attempts/attempt_001/runtime_health.json").unlink()
    fail_json = tmp_path / "reports/fail.json"
    fail_png = tmp_path / "reports/fail.png"
    fail_process = subprocess.run(
        [
            sys.executable,
            str(MODULE_PATH),
            "--run",
            str(fail_run),
            "--expected-route-sha256",
            fail_sha256,
            "--output-json",
            str(fail_json),
            "--output-png",
            str(fail_png),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert fail_process.returncode == 1, fail_process.stderr
    assert json.loads(fail_json.read_text(encoding="utf-8"))["status"] == "FAIL"
    assert fail_png.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert fail_png.stat().st_size > 1000


def test_cli_refuses_to_overwrite_an_existing_report(tmp_path: Path) -> None:
    run, route_sha256 = _create_run(tmp_path, "baseline_repeat_01")
    output_json = tmp_path / "reports/existing.json"
    output_png = tmp_path / "reports/new.png"
    output_json.parent.mkdir(parents=True)
    output_json.write_text("preserve-this-evidence\n", encoding="utf-8")

    process = subprocess.run(
        [
            sys.executable,
            str(MODULE_PATH),
            "--run",
            str(run),
            "--expected-route-sha256",
            route_sha256,
            "--output-json",
            str(output_json),
            "--output-png",
            str(output_png),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert process.returncode != 0
    assert output_json.read_text(encoding="utf-8") == "preserve-this-evidence\n"
    assert not output_png.exists()


def test_cli_refuses_to_overwrite_an_existing_png(tmp_path: Path) -> None:
    run, route_sha256 = _create_run(tmp_path, "baseline_repeat_01")
    output_json = tmp_path / "reports/new.json"
    output_png = tmp_path / "reports/existing.png"
    output_png.parent.mkdir(parents=True)
    output_png.write_bytes(b"preserve-this-png")

    process = subprocess.run(
        [
            sys.executable,
            str(MODULE_PATH),
            "--run",
            str(run),
            "--expected-route-sha256",
            route_sha256,
            "--output-json",
            str(output_json),
            "--output-png",
            str(output_png),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert process.returncode != 0
    assert output_png.read_bytes() == b"preserve-this-png"
    assert not output_json.exists()


def test_cli_writes_png_to_the_exact_suffixless_path(tmp_path: Path) -> None:
    run, route_sha256 = _create_run(tmp_path, "baseline_repeat_01")
    output_json = tmp_path / "reports/summary.json"
    output_png = tmp_path / "reports/summary-image"
    implicit_png = output_png.with_suffix(".png")
    implicit_png.parent.mkdir(parents=True)
    implicit_png.write_bytes(b"preserve-implicit-sibling")

    process = subprocess.run(
        [
            sys.executable,
            str(MODULE_PATH),
            "--run",
            str(run),
            "--expected-route-sha256",
            route_sha256,
            "--output-json",
            str(output_json),
            "--output-png",
            str(output_png),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert process.returncode == 0, process.stderr
    assert output_png.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert implicit_png.read_bytes() == b"preserve-implicit-sibling"


def test_exclusive_pair_publish_rolls_back_without_overwriting(tmp_path: Path) -> None:
    staged_json = tmp_path / "staged.json"
    staged_png = tmp_path / "staged.png"
    output_json = tmp_path / "output.json"
    output_png = tmp_path / "output.png"
    staged_json.write_bytes(b"new-json")
    staged_png.write_bytes(b"new-png")
    output_png.write_bytes(b"existing-png")

    with pytest.raises(FileExistsError):
        summary_tool._publish_output_pair(
            staged_json,
            output_json,
            staged_png,
            output_png,
        )

    assert not output_json.exists()
    assert output_png.read_bytes() == b"existing-png"
