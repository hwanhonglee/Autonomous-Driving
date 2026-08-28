from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest


MODULE_PATH = (
    Path(__file__).parents[1] / "scripts" / "e2e" / "summarize_optimization_v2.py"
)
SPEC = importlib.util.spec_from_file_location("summarize_optimization_v2", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
summary_tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(summary_tool)


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")


def _complete_trial(path: Path, *, success: bool = True, reason: str = "goal reached") -> None:
    _write_json(
        path / "result.json",
        {
            "success": success,
            "reason": reason,
            "final": {"remaining_distance_m": 0.75},
            "metrics": {
                "minimum_remaining_distance_m": 0.50,
                "maximum_trajectory_correction_m": 12.25,
            },
        },
    )
    _write_json(
        path / "diagnosis.json",
        {
            "metrics": {
                "tracking": {
                    "actual_to_route_cte_m": {"max_abs": 1.1, "p95_abs": 0.9},
                    "actual_to_final_cte_m": {"max_abs": 0.6, "p95_abs": 0.4},
                    "final_trajectory_age_sec": {"p95_abs": 0.8},
                },
                "mpc_diagnostic": {
                    "final_steer_command_rad": {"max_abs": 0.51}
                },
                "steering_tracking": {
                    "command_peak_abs_rad": 0.61,
                    "measured_virtual_peak_abs_rad": 0.55,
                },
            }
        },
    )
    _write_json(
        path / "latency" / "e2e_latency.json",
        {
            "event_rates": {
                "/planning/vad/candidate_trajectories": {
                    "effective_receipt_rate_hz": 2.04,
                    "receipt_rate_hz": 2.49,
                }
            },
            "stages": {
                "trajectory_age_at_control": {
                    "stamp_delta_sec": {"available": True, "p95": 0.32}
                }
            }
        },
    )


def test_read_trial_extracts_requested_metrics_and_prefers_primary_sources(
    tmp_path: Path,
) -> None:
    trial_path = tmp_path / "mpc_delay012"
    _complete_trial(trial_path)

    trial = summary_tool.read_trial(trial_path)

    assert trial["status"] == "passed"
    assert trial["success"] is True
    assert trial["reason"] == "goal reached"
    assert trial["remaining_distance_m"] == pytest.approx(0.75)
    assert trial["route_cte_max_m"] == pytest.approx(1.1)
    assert trial["route_cte_p95_m"] == pytest.approx(0.9)
    assert trial["final_cte_max_m"] == pytest.approx(0.6)
    assert trial["final_cte_p95_m"] == pytest.approx(0.4)
    assert trial["steer_command_peak_rad"] == pytest.approx(0.51)
    assert trial["steer_measured_peak_rad"] == pytest.approx(0.55)
    assert trial["trajectory_age_p95_sec"] == pytest.approx(0.32)
    assert trial["vad_output_rate_hz"] == pytest.approx(2.04)
    assert trial["metric_sources"]["vad_output_rate_hz"].endswith(
        "effective_receipt_rate_hz"
    )
    assert trial["trajectory_correction_max_m"] == pytest.approx(12.25)
    assert trial["metric_sources"]["trajectory_age_p95_sec"].startswith("latency:")
    assert trial["missing_metrics"] == []


def test_failed_trial_uses_documented_fallbacks_when_optional_files_are_missing(
    tmp_path: Path,
) -> None:
    trial_path = tmp_path / "failed_smart_mpc"
    _write_json(
        trial_path / "result.json",
        {
            "success": False,
            "reason": "route progress stalled",
            "metrics": {"minimum_remaining_distance_m": 2.5},
        },
    )
    _write_json(
        trial_path / "diagnosis.json",
        {
            "metrics": {
                "tracking": {"final_trajectory_age_sec": {"p95_abs": 0.7}},
                "mpc_diagnostic": {"final_steer_command_rad": None},
                "steering_tracking": {
                    "command_peak_abs_rad": 0.42,
                    "report_angle_rad": {"max_abs": 0.39},
                },
            }
        },
    )

    trial = summary_tool.read_trial(trial_path)

    assert trial["status"] == "failed"
    assert trial["reason"] == "route progress stalled"
    assert trial["remaining_distance_m"] == pytest.approx(2.5)
    assert trial["steer_command_peak_rad"] == pytest.approx(0.42)
    assert trial["steer_measured_peak_rad"] == pytest.approx(0.39)
    assert trial["trajectory_age_p95_sec"] == pytest.approx(0.7)
    assert trial["metric_sources"]["trajectory_age_p95_sec"].startswith("diagnosis:")
    assert trial["input_files"]["latency"]["status"] == "missing"
    assert "route_cte_max_m" in trial["missing_metrics"]


def test_vad_rate_falls_back_to_legacy_median_period_metric(tmp_path: Path) -> None:
    trial_path = tmp_path / "legacy_latency"
    _complete_trial(trial_path)
    latency_path = trial_path / "latency" / "e2e_latency.json"
    latency = json.loads(latency_path.read_text(encoding="utf-8"))
    candidate_rate = latency["event_rates"]["/planning/vad/candidate_trajectories"]
    candidate_rate.pop("effective_receipt_rate_hz")
    latency_path.write_text(json.dumps(latency), encoding="utf-8")

    trial = summary_tool.read_trial(trial_path)

    assert trial["vad_output_rate_hz"] == pytest.approx(2.49)
    assert trial["metric_sources"]["vad_output_rate_hz"].endswith("receipt_rate_hz")


def test_summary_retains_missing_and_malformed_trials_in_sorted_order(
    tmp_path: Path,
) -> None:
    root = tmp_path / "optimization_v2"
    _complete_trial(root / "z_pass")
    (root / "a_empty").mkdir(parents=True)
    malformed = root / "m_malformed"
    malformed.mkdir()
    (malformed / "result.json").write_text("{not-json", encoding="utf-8")
    _write_json(malformed / "diagnosis.json", ["not", "an", "object"])

    summary = summary_tool.build_summary(root)

    assert [trial["name"] for trial in summary["trials"]] == [
        "a_empty",
        "m_malformed",
        "z_pass",
    ]
    assert summary["counts"] == {
        "total": 3,
        "passed": 1,
        "failed": 0,
        "incomplete": 2,
        "complete_input_sets": 1,
        "complete_metric_rows": 1,
    }
    empty = summary["trials"][0]
    malformed_row = summary["trials"][1]
    assert empty["reason"] == "result.json missing"
    assert len(empty["missing_metrics"]) == 10
    assert malformed_row["reason"] == "result.json invalid"
    assert malformed_row["input_files"]["result"]["status"] == "invalid"
    assert malformed_row["input_files"]["diagnosis"]["status"] == "invalid"


def test_build_summary_rejects_a_nonexistent_root(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="does not exist"):
        summary_tool.build_summary(tmp_path / "not-there")


def test_trial_name_shortening_keeps_the_repeat_suffix() -> None:
    name = "mixed_fp16_lnfp32_mpc012_tau015_stop060_repeat03"

    shortened = summary_tool._shorten_trial_name(name, width=36)

    assert len(shortened) == 36
    assert shortened.endswith("op060_repeat03")


def test_cli_writes_json_and_png_table_to_requested_paths(tmp_path: Path) -> None:
    root = tmp_path / "optimization_v2"
    _complete_trial(root / "trial01")
    output_json = tmp_path / "reports" / "summary.json"
    output_png = tmp_path / "reports" / "summary.png"

    completed = subprocess.run(
        [
            sys.executable,
            str(MODULE_PATH),
            str(root),
            "--output-json",
            str(output_json),
            "--output-png",
            str(output_png),
            "--title",
            "Generalization trial summary",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    assert json.loads(output_json.read_text(encoding="utf-8"))["counts"]["passed"] == 1
    assert output_png.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert output_png.stat().st_size > 10_000
