#!/usr/bin/env python3
"""Build a fault-tolerant JSON and PNG table for optimization trial folders."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import textwrap
from typing import Any, Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


RESULT_PATH = Path("result.json")
DIAGNOSIS_PATH = Path("diagnosis.json")
LATENCY_PATH = Path("latency/e2e_latency.json")

METRIC_COLUMNS = (
    {
        "key": "remaining_distance_m",
        "label": "Remaining",
        "unit": "m",
    },
    {
        "key": "route_cte_max_m",
        "label": "Route CTE max",
        "unit": "m",
    },
    {
        "key": "route_cte_p95_m",
        "label": "Route CTE p95",
        "unit": "m",
    },
    {
        "key": "final_cte_max_m",
        "label": "Final trajectory CTE max",
        "unit": "m",
    },
    {
        "key": "final_cte_p95_m",
        "label": "Final trajectory CTE p95",
        "unit": "m",
    },
    {
        "key": "steer_command_peak_rad",
        "label": "Steer command peak",
        "unit": "rad",
    },
    {
        "key": "steer_measured_peak_rad",
        "label": "Steer measured peak",
        "unit": "rad",
    },
    {
        "key": "trajectory_age_p95_sec",
        "label": "Trajectory age p95",
        "unit": "s",
    },
    {
        "key": "vad_output_rate_hz",
        "label": "VAD effective output rate",
        "unit": "Hz",
    },
    {
        "key": "trajectory_correction_max_m",
        "label": "Trajectory correction max",
        "unit": "m",
    },
)


def nested(payload: object, *keys: str) -> object | None:
    value = payload
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            return None
        value = value[key]
    return value


def finite_number(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    converted = float(value)
    return converted if math.isfinite(converted) else None


def first_number(
    payload: object,
    candidates: Iterable[tuple[str, ...]],
) -> tuple[float | None, str | None]:
    for path in candidates:
        value = finite_number(nested(payload, *path))
        if value is not None:
            return value, ".".join(path)
    return None, None


def read_json_object(path: Path, relative_path: Path) -> tuple[dict[str, Any] | None, dict]:
    file_info = {"path": relative_path.as_posix()}
    if not path.exists():
        return None, {**file_info, "status": "missing", "error": "file not found"}
    if not path.is_file():
        return None, {**file_info, "status": "invalid", "error": "not a regular file"}
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        return None, {
            **file_info,
            "status": "invalid",
            "error": f"{type(exc).__name__}: {exc}",
        }
    if not isinstance(payload, dict):
        return None, {
            **file_info,
            "status": "invalid",
            "error": "top-level JSON value is not an object",
        }
    return payload, {**file_info, "status": "ok"}


def _file_warning(label: str, info: dict) -> str | None:
    if info["status"] == "ok":
        return None
    return f"{label}: {info['status']} ({info['error']})"


def read_trial(path: Path) -> dict[str, object]:
    result, result_info = read_json_object(path / RESULT_PATH, RESULT_PATH)
    diagnosis, diagnosis_info = read_json_object(
        path / DIAGNOSIS_PATH, DIAGNOSIS_PATH
    )
    latency, latency_info = read_json_object(path / LATENCY_PATH, LATENCY_PATH)

    warnings = [
        warning
        for warning in (
            _file_warning("result", result_info),
            _file_warning("diagnosis", diagnosis_info),
            _file_warning("latency", latency_info),
        )
        if warning is not None
    ]

    raw_success = nested(result, "success")
    success = raw_success if isinstance(raw_success, bool) else None
    if result is not None and success is None:
        warnings.append("result.success: missing or not a boolean")
    status = "passed" if success is True else "failed" if success is False else "incomplete"

    raw_reason = nested(result, "reason")
    reason = raw_reason.strip() if isinstance(raw_reason, str) and raw_reason.strip() else None
    if reason is None and result_info["status"] != "ok":
        reason = f"result.json {result_info['status']}"
    elif reason is None:
        warnings.append("result.reason: missing or empty")

    remaining, remaining_source = first_number(
        result,
        (
            ("final", "remaining_distance_m"),
            ("metrics", "minimum_remaining_distance_m"),
        ),
    )
    route_max, route_max_source = first_number(
        diagnosis,
        (("metrics", "tracking", "actual_to_route_cte_m", "max_abs"),),
    )
    route_p95, route_p95_source = first_number(
        diagnosis,
        (("metrics", "tracking", "actual_to_route_cte_m", "p95_abs"),),
    )
    final_max, final_max_source = first_number(
        diagnosis,
        (("metrics", "tracking", "actual_to_final_cte_m", "max_abs"),),
    )
    final_p95, final_p95_source = first_number(
        diagnosis,
        (("metrics", "tracking", "actual_to_final_cte_m", "p95_abs"),),
    )
    command_peak, command_source = first_number(
        diagnosis,
        (
            ("metrics", "mpc_diagnostic", "final_steer_command_rad", "max_abs"),
            ("metrics", "steering_tracking", "command_peak_abs_rad"),
        ),
    )
    measured_peak, measured_source = first_number(
        diagnosis,
        (
            ("metrics", "steering_tracking", "measured_virtual_peak_abs_rad"),
            ("metrics", "steering_tracking", "report_angle_rad", "max_abs"),
        ),
    )
    trajectory_age, trajectory_age_source = first_number(
        latency,
        (
            ("stages", "trajectory_age_at_control", "stamp_delta_sec", "p95"),
            ("trajectory_age_at_control", "stamp_delta_sec", "p95"),
        ),
    )
    if trajectory_age is None:
        trajectory_age, diagnosis_age_source = first_number(
            diagnosis,
            (("metrics", "tracking", "final_trajectory_age_sec", "p95_abs"),),
        )
        if diagnosis_age_source is not None:
            trajectory_age_source = f"diagnosis:{diagnosis_age_source}"
    elif trajectory_age_source is not None:
        trajectory_age_source = f"latency:{trajectory_age_source}"

    vad_output_rate, vad_output_rate_source = first_number(
        latency,
        (
            (
                "event_rates",
                "/planning/vad/candidate_trajectories",
                "effective_receipt_rate_hz",
            ),
            (
                "event_rates",
                "/planning/trajectory",
                "effective_receipt_rate_hz",
            ),
            (
                "event_rates",
                "/planning/vad/candidate_trajectories",
                "receipt_rate_hz",
            ),
            ("event_rates", "/planning/trajectory", "receipt_rate_hz"),
        ),
    )
    correction_max, correction_max_source = first_number(
        result,
        (("metrics", "maximum_trajectory_correction_m"),),
    )

    values = {
        "remaining_distance_m": remaining,
        "route_cte_max_m": route_max,
        "route_cte_p95_m": route_p95,
        "final_cte_max_m": final_max,
        "final_cte_p95_m": final_p95,
        "steer_command_peak_rad": command_peak,
        "steer_measured_peak_rad": measured_peak,
        "trajectory_age_p95_sec": trajectory_age,
        "vad_output_rate_hz": vad_output_rate,
        "trajectory_correction_max_m": correction_max,
    }
    sources = {
        "remaining_distance_m": (
            f"result:{remaining_source}" if remaining_source is not None else None
        ),
        "route_cte_max_m": (
            f"diagnosis:{route_max_source}" if route_max_source is not None else None
        ),
        "route_cte_p95_m": (
            f"diagnosis:{route_p95_source}" if route_p95_source is not None else None
        ),
        "final_cte_max_m": (
            f"diagnosis:{final_max_source}" if final_max_source is not None else None
        ),
        "final_cte_p95_m": (
            f"diagnosis:{final_p95_source}" if final_p95_source is not None else None
        ),
        "steer_command_peak_rad": (
            f"diagnosis:{command_source}" if command_source is not None else None
        ),
        "steer_measured_peak_rad": (
            f"diagnosis:{measured_source}" if measured_source is not None else None
        ),
        "trajectory_age_p95_sec": trajectory_age_source,
        "vad_output_rate_hz": (
            f"latency:{vad_output_rate_source}"
            if vad_output_rate_source is not None
            else None
        ),
        "trajectory_correction_max_m": (
            f"result:{correction_max_source}"
            if correction_max_source is not None
            else None
        ),
    }
    missing_metrics = [key for key, value in values.items() if value is None]

    return {
        "name": path.name,
        "path": str(path.resolve()),
        "status": status,
        "success": success,
        "reason": reason,
        **values,
        "metric_sources": sources,
        "missing_metrics": missing_metrics,
        "input_files": {
            "result": result_info,
            "diagnosis": diagnosis_info,
            "latency": latency_info,
        },
        "warnings": warnings,
    }


def discover_trial_directories(root: Path) -> list[Path]:
    if not root.exists():
        raise ValueError(f"optimization directory does not exist: {root}")
    if not root.is_dir():
        raise ValueError(f"optimization path is not a directory: {root}")
    return sorted((path for path in root.iterdir() if path.is_dir()), key=lambda path: path.name)


def build_summary(root: Path) -> dict[str, object]:
    root = root.expanduser().resolve()
    trials = [read_trial(path) for path in discover_trial_directories(root)]
    counts = {
        "total": len(trials),
        "passed": sum(trial["status"] == "passed" for trial in trials),
        "failed": sum(trial["status"] == "failed" for trial in trials),
        "incomplete": sum(trial["status"] == "incomplete" for trial in trials),
        "complete_input_sets": sum(
            all(info["status"] == "ok" for info in trial["input_files"].values())
            for trial in trials
        ),
        "complete_metric_rows": sum(not trial["missing_metrics"] for trial in trials),
    }
    return {
        "schema_version": 1,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "optimization_dir": str(root),
        "counts": counts,
        "columns": [
            {"key": "success", "label": "Success", "unit": None},
            {"key": "reason", "label": "Reason", "unit": None},
            *METRIC_COLUMNS,
        ],
        "metric_policy": {
            "remaining": "final.remaining_distance_m; minimum observed distance fallback",
            "steer_command": "MPC final command peak; steering tracking command peak fallback",
            "steer_measured": "virtual tire-angle peak; raw steering-report angle fallback",
            "trajectory_age": (
                "latency stamp-based trajectory age at control; diagnosis trajectory age fallback"
            ),
            "vad_output_rate": (
                "candidate trajectory effective receipt rate (1 / mean period); "
                "final trajectory fallback, then legacy median-period rate"
            ),
            "trajectory_correction": "maximum raw-to-route correction reported by evaluator",
            "missing_values": None,
        },
        "trials": trials,
    }


def _format_number(value: object) -> str:
    number = finite_number(value)
    return "--" if number is None else f"{number:.3f}"


def _metric_pair(trial: dict[str, object], first: str, second: str) -> str:
    return f"{_format_number(trial.get(first))} / {_format_number(trial.get(second))}"


def _status_label(status: object) -> str:
    return {
        "passed": "PASS",
        "failed": "FAIL",
        "incomplete": "INCOMPLETE",
    }.get(status, "UNKNOWN")


def _shorten_trial_name(value: object, width: int = 48) -> str:
    name = str(value)
    if len(name) <= width:
        return name
    suffix_width = 16
    prefix_width = width - suffix_width - 3
    return f"{name[:prefix_width]}...{name[-suffix_width:]}"


def render_table(
    summary: dict[str, object],
    output: Path,
    *,
    title_prefix: str = "Optimization v2 trial summary",
) -> None:
    trials = summary.get("trials")
    if not isinstance(trials, list):
        raise ValueError("summary.trials must be a list")

    headers = (
        "Trial",
        "Status",
        "Reason",
        "Remaining\n[m]",
        "Route CTE\nmax / p95 [m]",
        "Final CTE\nmax / p95 [m]",
        "Steer peak\ncmd / measured [rad]",
        "Trajectory age\np95 [s]",
        "VAD effective\noutput [Hz]",
        "Correction\nmax [m]",
    )
    rows = []
    for trial in trials:
        reason = trial.get("reason") if isinstance(trial, dict) else None
        rows.append(
            [
                _shorten_trial_name(trial.get("name", "--")),
                _status_label(trial.get("status")),
                textwrap.shorten(str(reason), width=46, placeholder="...")
                if reason
                else "--",
                _format_number(trial.get("remaining_distance_m")),
                _metric_pair(trial, "route_cte_max_m", "route_cte_p95_m"),
                _metric_pair(trial, "final_cte_max_m", "final_cte_p95_m"),
                _metric_pair(
                    trial, "steer_command_peak_rad", "steer_measured_peak_rad"
                ),
                _format_number(trial.get("trajectory_age_p95_sec")),
                _format_number(trial.get("vad_output_rate_hz")),
                _format_number(trial.get("trajectory_correction_max_m")),
            ]
        )

    figure_height = max(3.2, 2.25 + 0.48 * max(len(rows), 1))
    fig, axis = plt.subplots(figsize=(20.0, figure_height))
    axis.axis("off")
    counts = summary.get("counts", {})
    title = (
        title_prefix
        + f"  |  {counts.get('passed', 0)} pass, {counts.get('failed', 0)} fail, "
        f"{counts.get('incomplete', 0)} incomplete"
    )
    axis.set_title(title, fontsize=15, fontweight="bold", loc="left", pad=16)

    display_rows = rows or [
        ["No trial directories found", "--", "--", "--", "--", "--", "--", "--", "--", "--"]
    ]
    table = axis.table(
        cellText=display_rows,
        colLabels=headers,
        cellLoc="center",
        colLoc="center",
        colWidths=(0.18, 0.06, 0.195, 0.06, 0.09, 0.09, 0.105, 0.07, 0.07, 0.08),
        bbox=(0.0, 0.12, 1.0, 0.82),
    )
    table.auto_set_font_size(False)
    table.set_fontsize(8.6)
    table.scale(1.0, 1.45)

    for (row_index, column_index), cell in table.get_celld().items():
        cell.set_edgecolor("#c7ccd1")
        cell.set_linewidth(0.7)
        if row_index == 0:
            cell.set_facecolor("#27313a")
            cell.get_text().set_color("white")
            cell.get_text().set_weight("bold")
        else:
            cell.set_facecolor("#f6f7f8" if row_index % 2 == 0 else "white")
            if column_index in (0, 2):
                cell.get_text().set_ha("left")

    status_colors = {
        "passed": ("#dff3e8", "#12643e"),
        "failed": ("#fde3e1", "#982018"),
        "incomplete": ("#eceff1", "#4d5963"),
    }
    for row_index, trial in enumerate(trials, start=1):
        background, foreground = status_colors.get(
            trial.get("status"), ("#eceff1", "#4d5963")
        )
        cell = table[(row_index, 1)]
        cell.set_facecolor(background)
        cell.get_text().set_color(foreground)
        cell.get_text().set_weight("bold")

    fig.text(
        0.01,
        0.045,
        "-- = unavailable. VAD effective rate is 1 / mean bag receipt period; "
        "fallbacks and exact sources are recorded in the JSON report.",
        fontsize=8.5,
        color="#4d5963",
        ha="left",
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=170, facecolor="white", bbox_inches="tight")
    plt.close(fig)


def write_summary(
    summary: dict[str, object],
    output_json: Path,
    output_png: Path,
    *,
    title_prefix: str = "Optimization v2 trial summary",
) -> None:
    output_json.parent.mkdir(parents=True, exist_ok=True)
    output_json.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    render_table(summary, output_png, title_prefix=title_prefix)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("optimization_dir", type=Path)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-png", type=Path)
    parser.add_argument(
        "--title",
        default="Optimization v2 trial summary",
        help="title prefix for the PNG table",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = args.optimization_dir.expanduser().resolve()
    output_json = args.output_json or root / "optimization_v2_summary.json"
    output_png = args.output_png or root / "optimization_v2_summary.png"
    try:
        summary = build_summary(root)
        write_summary(summary, output_json, output_png, title_prefix=args.title)
    except (OSError, ValueError) as exc:
        raise SystemExit(f"error: {exc}") from exc
    print(f"summary JSON: {output_json}")
    print(f"summary PNG:  {output_png}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
