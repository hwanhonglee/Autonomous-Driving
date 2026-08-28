from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest


MODULE_PATH = (
    Path(__file__).parents[1] / "scripts" / "e2e" / "summarize_path_tracking_ab.py"
)
SPEC = importlib.util.spec_from_file_location("summarize_path_tracking_ab", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
summary_tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(summary_tool)


def _write_trial(path: Path, scale: float) -> None:
    path.mkdir(parents=True)
    (path / "result.json").write_text(
        json.dumps({"success": True, "reason": "goal reached"}),
        encoding="utf-8",
    )
    (path / "diagnosis.json").write_text(
        json.dumps(
            {
                "metrics": {
                    "tracking": {
                        "actual_to_route_cte_m": {
                            "max_abs": 1.0 * scale,
                            "p95_abs": 0.7 * scale,
                        },
                        "actual_to_final_cte_m": {"p95_abs": 0.5 * scale},
                        "actual_to_final_yaw_error_rad": {
                            "p95_abs": 0.2 * scale
                        },
                    },
                    "final_path": {
                        "snapshot_peak_curvature_per_m": {
                            "p95_abs": 0.3 * scale
                        }
                    },
                    "mpc_diagnostic": {
                        "final_steer_command_rad": {"p95_abs": 0.25 * scale}
                    },
                }
            }
        ),
        encoding="utf-8",
    )


def test_build_summary_holds_a_candidate_with_one_worsening_metric(
    tmp_path: Path,
) -> None:
    _write_trial(tmp_path / "right_baseline_1", 1.0)
    _write_trial(tmp_path / "right_baseline_2", 1.0)
    _write_trial(tmp_path / "right_candidate_1", 0.9)
    _write_trial(tmp_path / "right_candidate_2", 0.9)
    candidate_diagnosis = tmp_path / "right_candidate_2" / "diagnosis.json"
    payload = json.loads(candidate_diagnosis.read_text(encoding="utf-8"))
    payload["metrics"]["final_path"]["snapshot_peak_curvature_per_m"][
        "p95_abs"
    ] = 0.4
    candidate_diagnosis.write_text(json.dumps(payload), encoding="utf-8")

    summary = summary_tool.build_summary(
        tmp_path,
        "right_baseline_",
        "Recommended",
        "right_candidate_",
        "Candidate",
    )

    assert summary["decision"] == "HOLD"
    assert summary["acceptance_gates"]["all_candidate_runs_pass"] is True
    assert (
        summary["acceptance_gates"][
            "path_curvature_p95_per_m_non_worsening"
        ]
        is False
    )
    assert summary["candidate_mean_change_percent"]["route_cte_max_m"] == (
        pytest.approx(-10.0)
    )


def test_cli_writes_decision_json_and_png(tmp_path: Path) -> None:
    _write_trial(tmp_path / "right_baseline_1", 1.0)
    _write_trial(tmp_path / "right_outlier_gate_1", 0.9)
    output_json = tmp_path / "report" / "decision.json"
    output_png = tmp_path / "report" / "decision.png"

    completed = subprocess.run(
        [
            sys.executable,
            str(MODULE_PATH),
            str(tmp_path),
            "--output-json",
            str(output_json),
            "--output-png",
            str(output_png),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    assert json.loads(output_json.read_text(encoding="utf-8"))["decision"] == (
        "ACCEPT"
    )
    assert output_png.stat().st_size > 1000
