#!/usr/bin/env python3
"""Render fail-closed CARLA speed-source evidence from a recorded ROS bag.

The plot intentionally separates the selected VAD trajectory's recorded velocity
from the explicit CARLA simulation cruise overlay, the vehicle-command gate, and
the measured vehicle response.  It never treats the requested simulation target
as a velocity predicted by VAD.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Callable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import analyze_turn_dynamics as dynamics  # noqa: E402


RAW_TOPIC = dynamics.RAW_TOPIC
PLANNING_TOPIC = dynamics.FINAL_TOPIC
GATED_CONTROL_TOPIC = dynamics.GATED_CONTROL_TOPIC
ODOMETRY_TOPIC = dynamics.ODOMETRY_TOPIC

SERIES_ORDER = (
    "raw_selected_vad",
    "explicit_overlaid_planning",
    "gated_control_command",
    "actual_odometry",
)
SERIES_LABELS = {
    "raw_selected_vad": "Selected raw VAD: first trajectory point",
    "explicit_overlaid_planning": "Planning after explicit CARLA cruise overlay",
    "gated_control_command": "Vehicle command gate: velocity command",
    "actual_odometry": "Actual odometry speed",
}
SERIES_TOPICS = {
    "raw_selected_vad": RAW_TOPIC,
    "explicit_overlaid_planning": PLANNING_TOPIC,
    "gated_control_command": GATED_CONTROL_TOPIC,
    "actual_odometry": ODOMETRY_TOPIC,
}
SERIES_COLORS = {
    "raw_selected_vad": "#e67e22",
    "explicit_overlaid_planning": "#2980b9",
    "gated_control_command": "#8e44ad",
    "actual_odometry": "#111111",
}


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _sha256_json(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _read_json_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise RuntimeError(f"cannot read {label} {path}: {error}") from error
    if not isinstance(value, dict):
        raise RuntimeError(f"{label} must contain a JSON object: {path}")
    return value


def _bag_manifest(bag: Path) -> dict[str, Any]:
    bag = bag.expanduser()
    if bag.is_symlink():
        raise RuntimeError(f"rosbag evidence root must not be a symlink: {bag}")
    bag = bag.resolve()
    if not bag.is_dir():
        raise RuntimeError(f"rosbag directory does not exist: {bag}")
    files: list[dict[str, Any]] = []
    for path in sorted(bag.rglob("*")):
        if path.is_symlink():
            raise RuntimeError(f"rosbag evidence must not contain a symlink: {path}")
        if not path.is_file():
            continue
        relative = path.relative_to(bag).as_posix()
        files.append(
            {
                "path": relative,
                "size_bytes": path.stat().st_size,
                "sha256": _sha256_file(path),
            }
        )
    if not files or not any(item["path"] == "metadata.yaml" for item in files):
        raise RuntimeError(f"rosbag evidence has no metadata.yaml: {bag}")
    manifest = {
        "schema_version": 1,
        "root": str(bag),
        "files": files,
    }
    manifest["sha256"] = _sha256_json(
        {"schema_version": manifest["schema_version"], "files": files}
    )
    return manifest


def _resolved_result_route(result_path: Path, result: dict[str, Any]) -> Path:
    route_value = result.get("route_file")
    if not isinstance(route_value, str) or not route_value:
        raise RuntimeError("route result has no route_file")
    route_path = Path(route_value).expanduser()
    if not route_path.is_absolute():
        route_path = result_path.parent / route_path
    return route_path.resolve()


def _source_identity(
    bag: Path, route_path: Path, result_path: Path
) -> dict[str, Any]:
    route_path = route_path.expanduser().resolve()
    result_path = result_path.expanduser().resolve()
    if not route_path.is_file():
        raise RuntimeError(f"effective route does not exist: {route_path}")
    if not result_path.is_file():
        raise RuntimeError(f"route result does not exist: {result_path}")
    route = _read_json_object(route_path, "effective route")
    result = _read_json_object(result_path, "route result")
    result_route = _resolved_result_route(result_path, result)
    if result_route != route_path:
        raise RuntimeError(
            "route result is bound to a different effective route: "
            f"{result_route} != {route_path}"
        )
    profile_context = result.get("profile_context")
    exposure = result.get("speed_exposure")
    expected_context = {
        "longitudinal_velocity_source": "explicit_simulation_nominal",
        "vad_velocity_evaluated": False,
        "vad_geometry_evaluated": True,
    }
    result_success = result.get("success")
    exposure_status = exposure.get("status") if isinstance(exposure, dict) else None
    if (
        not isinstance(result_success, bool)
        or result.get("execution_mode") != "full_stack"
        or profile_context != expected_context
        or not isinstance(exposure, dict)
        or exposure_status not in {"PASS", "FAIL"}
        or any(exposure.get(key) != value for key, value in expected_context.items())
    ):
        raise RuntimeError("route result is not an explicit-simulation speed trial")
    town = route.get("town")
    scenario = route.get("scenario")
    if not isinstance(town, str) or not town or scenario not in {
        "straight",
        "left",
        "right",
    }:
        raise RuntimeError("effective route has invalid town/scenario identity")
    identity = {
        "schema_version": 1,
        "effective_route": {
            "path": str(route_path),
            "sha256": _sha256_file(route_path),
            "town": town,
            "scenario": scenario,
            "trial_id": "straight" if scenario == "straight" else "turn",
            "route_length_m": route.get("route_length_m"),
        },
        "route_result": {
            "path": str(result_path),
            "sha256": _sha256_file(result_path),
            "success": result_success,
            "execution_mode": "full_stack",
            "profile_context": expected_context,
            "speed_exposure_status": exposure_status,
            "reason": result.get("reason"),
        },
        "rosbag": _bag_manifest(bag),
    }
    identity["sha256"] = _sha256_json(
        {key: value for key, value in identity.items() if key != "sha256"}
    )
    return identity


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path, help="rosbag2 directory")
    parser.add_argument(
        "--route-file",
        required=True,
        type=Path,
        help="effective aligned route used by route_test",
    )
    parser.add_argument(
        "--result",
        required=True,
        type=Path,
        help="route_test result.json bound to this analysis, including failed trials",
    )
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--profile-id", required=True)
    parser.add_argument("--target-speed-mps", required=True, type=float)
    parser.add_argument(
        "--longitudinal-speed-source",
        required=True,
        choices=("explicit_simulation_nominal",),
    )
    args = parser.parse_args()
    if not math.isfinite(args.target_speed_mps) or args.target_speed_mps <= 0.0:
        parser.error("--target-speed-mps must be finite and positive")
    return args


def _physics_ns(record: dict[str, Any]) -> int:
    stamp_ns = int(record.get("stamp_ns", 0))
    return stamp_ns if stamp_ns > 0 else int(record["bag_ns"])


def _finite_summary(values: list[float]) -> dict[str, float | int | None]:
    finite = np.asarray([value for value in values if math.isfinite(value)], dtype=float)
    if not len(finite):
        return {
            "count": 0,
            "minimum_mps": None,
            "mean_mps": None,
            "p95_mps": None,
            "maximum_mps": None,
        }
    return {
        "count": int(len(finite)),
        "minimum_mps": float(np.min(finite)),
        "mean_mps": float(np.mean(finite)),
        "p95_mps": float(np.percentile(finite, 95)),
        "maximum_mps": float(np.max(finite)),
    }


def _sample_time(record: dict[str, Any], origin_ns: int) -> dict[str, Any]:
    stamp_ns = int(record.get("stamp_ns", 0))
    physics_ns = _physics_ns(record)
    return {
        "time_sec": (physics_ns - origin_ns) * 1.0e-9,
        "header_time_ns": stamp_ns if stamp_ns > 0 else None,
        "bag_time_ns": int(record["bag_ns"]),
        "time_source": "header_stamp" if stamp_ns > 0 else "bag_receipt_fallback",
    }


def _trajectory_samples(
    records: list[dict[str, Any]], origin_ns: int
) -> list[dict[str, Any]]:
    samples: list[dict[str, Any]] = []
    for record in records:
        speeds = np.asarray(record.get("speed", []), dtype=float)
        finite = speeds[np.isfinite(speeds)]
        if not len(speeds) or not len(finite) or not math.isfinite(float(speeds[0])):
            continue
        samples.append(
            {
                **_sample_time(record, origin_ns),
                "speed_mps": float(speeds[0]),
                "trajectory_point_count": int(len(speeds)),
                "trajectory_horizon_minimum_mps": float(np.min(finite)),
                "trajectory_horizon_p95_mps": float(np.percentile(finite, 95)),
                "trajectory_horizon_maximum_mps": float(np.max(finite)),
            }
        )
    return sorted(samples, key=lambda sample: (sample["time_sec"], sample["bag_time_ns"]))


def _scalar_samples(
    records: list[dict[str, Any]], origin_ns: int
) -> list[dict[str, Any]]:
    samples: list[dict[str, Any]] = []
    for record in records:
        speed = float(record.get("speed", math.nan))
        if not math.isfinite(speed):
            continue
        samples.append(
            {
                **_sample_time(record, origin_ns),
                "speed_mps": speed,
            }
        )
    return sorted(samples, key=lambda sample: (sample["time_sec"], sample["bag_time_ns"]))


def _series_summary(samples: list[dict[str, Any]]) -> dict[str, Any]:
    times = np.asarray([sample["time_sec"] for sample in samples], dtype=float)
    speed = [float(sample["speed_mps"]) for sample in samples]
    positive_gaps = np.diff(np.unique(times)) if len(times) > 1 else np.empty(0)
    return {
        **_finite_summary(speed),
        "first_time_sec": float(times[0]) if len(times) else None,
        "last_time_sec": float(times[-1]) if len(times) else None,
        "duration_sec": float(times[-1] - times[0]) if len(times) else None,
        "maximum_sample_gap_sec": (
            float(np.max(positive_gaps)) if len(positive_gaps) else None
        ),
        "header_stamp_count": sum(
            sample["time_source"] == "header_stamp" for sample in samples
        ),
        "bag_receipt_fallback_count": sum(
            sample["time_source"] == "bag_receipt_fallback" for sample in samples
        ),
    }


def _aligned_delta_summary(
    minuend: list[dict[str, Any]], subtrahend: list[dict[str, Any]]
) -> dict[str, Any]:
    if len(minuend) < 2 or len(subtrahend) < 2:
        return {**_finite_summary([]), "overlap_sec": None}
    left_time, left_speed = _deduplicate_samples(minuend)
    right_time, right_speed = _deduplicate_samples(subtrahend)
    begin = max(float(left_time[0]), float(right_time[0]))
    end = min(float(left_time[-1]), float(right_time[-1]))
    grid = left_time[(left_time >= begin) & (left_time <= end)]
    if end <= begin or not len(grid):
        return {**_finite_summary([]), "overlap_sec": max(0.0, end - begin)}
    delta = left_speed[(left_time >= begin) & (left_time <= end)] - np.interp(
        grid, right_time, right_speed
    )
    return {**_finite_summary(delta.tolist()), "overlap_sec": end - begin}


def _deduplicate_samples(
    samples: list[dict[str, Any]],
) -> tuple[np.ndarray, np.ndarray]:
    times = np.asarray([sample["time_sec"] for sample in samples], dtype=float)
    speed = np.asarray([sample["speed_mps"] for sample in samples], dtype=float)
    order = np.argsort(times, kind="stable")
    times = times[order]
    speed = speed[order]
    if len(times) > 1:
        keep = np.r_[times[1:] != times[:-1], True]
        times = times[keep]
        speed = speed[keep]
    return times, speed


def build_evidence(
    records: dict[str, list[dict[str, Any]]],
    topic_types: dict[str, str],
    *,
    bag: Path,
    profile_id: str,
    target_speed_mps: float,
    longitudinal_speed_source: str,
    source_identity: dict[str, Any] | None = None,
) -> dict[str, Any]:
    target_speed_kph = target_speed_mps * 3.6
    required_records = {
        key: records.get(topic, []) for key, topic in SERIES_TOPICS.items()
    }
    physics_times = [
        _physics_ns(record)
        for values in required_records.values()
        for record in values
    ]
    if not physics_times:
        raise RuntimeError("bag has no messages for any required speed-evidence topic")
    origin_ns = min(physics_times)
    series = {
        "raw_selected_vad": _trajectory_samples(
            required_records["raw_selected_vad"], origin_ns
        ),
        "explicit_overlaid_planning": _trajectory_samples(
            required_records["explicit_overlaid_planning"], origin_ns
        ),
        "gated_control_command": _scalar_samples(
            required_records["gated_control_command"], origin_ns
        ),
        "actual_odometry": _scalar_samples(
            required_records["actual_odometry"], origin_ns
        ),
    }
    problems: list[str] = []
    warnings: list[str] = []
    for key in SERIES_ORDER:
        count = len(series[key])
        if count < 2:
            problems.append(
                f"{SERIES_TOPICS[key]} has {count} finite sample(s); at least 2 are required"
            )
        fallback_count = sum(
            sample["time_source"] == "bag_receipt_fallback" for sample in series[key]
        )
        if fallback_count:
            warnings.append(
                f"{SERIES_TOPICS[key]} used bag receipt time for {fallback_count} sample(s)"
            )

    intervals = [
        (values[0]["time_sec"], values[-1]["time_sec"])
        for values in series.values()
        if len(values) >= 2
    ]
    common_begin = max((interval[0] for interval in intervals), default=math.nan)
    common_end = min((interval[1] for interval in intervals), default=math.nan)
    common_overlap = (
        max(0.0, common_end - common_begin)
        if math.isfinite(common_begin) and math.isfinite(common_end)
        else None
    )
    if len(intervals) == len(SERIES_ORDER) and not (common_overlap and common_overlap > 0.0):
        problems.append("the four required speed series have no common simulation-time interval")

    summaries = {key: _series_summary(series[key]) for key in SERIES_ORDER}
    deltas = {
        "planning_minus_raw_vad_mps": _aligned_delta_summary(
            series["explicit_overlaid_planning"], series["raw_selected_vad"]
        ),
        "gated_command_minus_planning_mps": _aligned_delta_summary(
            series["gated_control_command"], series["explicit_overlaid_planning"]
        ),
        "actual_minus_gated_command_mps": _aligned_delta_summary(
            series["actual_odometry"], series["gated_control_command"]
        ),
    }
    return {
        "schema_version": 1,
        "analysis": "carla_speed_source_evidence",
        "status": "complete" if not problems else "incomplete",
        "inputs": {
            "bag": str(bag.resolve()),
            "profile_id": profile_id,
            "target_speed_mps": target_speed_mps,
            "target_speed_kph": target_speed_kph,
            "longitudinal_speed_source": longitudinal_speed_source,
        },
        "source_identity": source_identity,
        "interpretation": {
            "planning_geometry": "VAD route-manager hybrid",
            "cruise_velocity_source": "explicit CARLA simulation profile",
            "raw_vad_velocity_is_cruise_target": False,
            "real_vehicle_ready": False,
            "note": (
                "The selected raw VAD velocity is preserved as evidence but is not the "
                f"{target_speed_kph:.6g} km/h cruise target. The planning series "
                "includes the explicit simulation-only velocity overlay."
            ),
        },
        "alignment": {
            "primary_time": "ROS message/header stamp (CARLA simulation clock)",
            "fallback_time": "rosbag receipt timestamp when header stamp is zero",
            "origin_time_ns": origin_ns,
            "reported_time": "seconds relative to the earliest required source sample",
            "common_interval_start_sec": (
                common_begin if math.isfinite(common_begin) else None
            ),
            "common_interval_end_sec": common_end if math.isfinite(common_end) else None,
            "common_interval_duration_sec": common_overlap,
        },
        "sources": {
            key: {
                "topic": SERIES_TOPICS[key],
                "topic_type": topic_types.get(SERIES_TOPICS[key]),
                "label": SERIES_LABELS[key],
                "velocity_field": (
                    "points[0].longitudinal_velocity_mps; horizon min/p95/max also retained"
                    if key in {"raw_selected_vad", "explicit_overlaid_planning"}
                    else (
                        "longitudinal.velocity"
                        if key == "gated_control_command"
                        else "twist.twist.linear.x"
                    )
                ),
            }
            for key in SERIES_ORDER
        },
        "message_counts": {
            topic: len(records.get(topic, [])) for topic in SERIES_TOPICS.values()
        },
        "quality": {
            "required_series": list(SERIES_ORDER),
            "problems": problems,
            "warnings": warnings,
        },
        "summary": summaries,
        "aligned_delta_summary": deltas,
        "series": series,
        "outputs": {
            "json": "speed_profile.json",
            "plot": "speed_profile.png",
        },
    }


def _plot_evidence(evidence: dict[str, Any], output: Path) -> None:
    series = evidence.get("series", {})
    figure, axes = plt.subplots(2, 1, figsize=(16, 10), constrained_layout=True)
    profile = evidence.get("inputs", {}).get("profile_id", "unknown")
    target_kph = evidence.get("inputs", {}).get("target_speed_kph")
    target_label = (
        f"{float(target_kph):.6g} km/h"
        if isinstance(target_kph, (int, float)) and math.isfinite(float(target_kph))
        else "unknown-speed"
    )
    status = evidence.get("status", "incomplete").upper()
    figure.suptitle(
        f"{target_label} CARLA speed-source evidence | {profile} | EVIDENCE {status}",
        fontsize=17,
        fontweight="bold",
    )

    axis = axes[0]
    for key in SERIES_ORDER:
        samples = series.get(key, [])
        if not samples:
            continue
        times = np.asarray([sample["time_sec"] for sample in samples], dtype=float)
        speeds = np.asarray([sample["speed_mps"] for sample in samples], dtype=float)
        linewidth = 2.5 if key == "actual_odometry" else 1.8
        axis.plot(
            times,
            speeds,
            color=SERIES_COLORS[key],
            linewidth=linewidth,
            alpha=0.95,
            label=SERIES_LABELS[key],
        )
    target = evidence.get("inputs", {}).get("target_speed_mps")
    if isinstance(target, (int, float)) and math.isfinite(float(target)):
        axis.axhline(
            float(target),
            color="#16a085",
            linestyle="--",
            linewidth=1.8,
            label=f"Explicit simulation target ({float(target):.3f} m/s)",
        )
    axis.axhline(0.0, color="#555555", linewidth=0.8)
    axis.set_title("Immediate velocity carried through the planning/control pipeline")
    axis.set_xlabel("simulation/header time from first evidence sample [s]")
    axis.set_ylabel("longitudinal velocity [m/s]")
    axis.grid(alpha=0.25)
    axis.legend(fontsize=9, loc="best")

    axis = axes[1]
    for key, alpha in (
        ("raw_selected_vad", 0.22),
        ("explicit_overlaid_planning", 0.18),
    ):
        samples = series.get(key, [])
        if not samples:
            continue
        times = np.asarray([sample["time_sec"] for sample in samples], dtype=float)
        minimum = np.asarray(
            [sample["trajectory_horizon_minimum_mps"] for sample in samples], dtype=float
        )
        maximum = np.asarray(
            [sample["trajectory_horizon_maximum_mps"] for sample in samples], dtype=float
        )
        p95 = np.asarray(
            [sample["trajectory_horizon_p95_mps"] for sample in samples], dtype=float
        )
        axis.fill_between(
            times,
            minimum,
            maximum,
            color=SERIES_COLORS[key],
            alpha=alpha,
            label=f"{SERIES_LABELS[key]}: trajectory horizon min–max",
        )
        axis.plot(
            times,
            p95,
            color=SERIES_COLORS[key],
            linestyle=":" if key == "raw_selected_vad" else "--",
            linewidth=1.5,
            label=f"{SERIES_LABELS[key]}: horizon p95",
        )
    axis.set_title(
        "Trajectory-horizon velocity evidence (raw VAD vs explicit simulation overlay)"
    )
    axis.set_xlabel("simulation/header time from first evidence sample [s]")
    axis.set_ylabel("trajectory velocity [m/s]")
    axis.grid(alpha=0.25)
    axis.legend(fontsize=8, loc="best")

    problems = evidence.get("quality", {}).get("problems", [])
    if problems:
        figure.text(
            0.01,
            0.018,
            "INCOMPLETE: " + " | ".join(str(problem) for problem in problems),
            color="#c0392b",
            fontsize=9,
        )
    figure.text(
        0.99,
        0.005,
        "Evidence completeness is not the route verdict; use result.json for PASS/FAIL.",
        ha="right",
        color="#555555",
        fontsize=8,
    )
    figure.savefig(output, dpi=150)
    plt.close(figure)


def _failure_evidence(
    args: argparse.Namespace,
    error: BaseException,
    source_identity: dict[str, Any] | None = None,
) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "analysis": "carla_speed_source_evidence",
        "status": "incomplete",
        "inputs": {
            "bag": str(args.bag.resolve()),
            "profile_id": args.profile_id,
            "target_speed_mps": args.target_speed_mps,
            "target_speed_kph": args.target_speed_mps * 3.6,
            "longitudinal_speed_source": args.longitudinal_speed_source,
        },
        "source_identity": source_identity,
        "quality": {
            "required_series": list(SERIES_ORDER),
            "problems": [f"{type(error).__name__}: {error}"],
            "warnings": [],
        },
        "series": {key: [] for key in SERIES_ORDER},
        "outputs": {
            "json": "speed_profile.json",
            "plot": "speed_profile.png",
        },
    }


def _write_json(payload: dict[str, Any], path: Path) -> None:
    with path.open("w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def run(
    args: argparse.Namespace,
    read_bag: Callable[
        [Path], tuple[dict[str, list[dict[str, Any]]], dict[str, str]]
    ] = dynamics._read_bag,
) -> int:
    args.output_dir.mkdir(parents=True, exist_ok=True)
    json_path = args.output_dir / "speed_profile.json"
    plot_path = args.output_dir / "speed_profile.png"
    source_identity = None
    try:
        route_file = getattr(args, "route_file", None)
        result_file = getattr(args, "result", None)
        if (route_file is None) != (result_file is None):
            raise RuntimeError("route_file and result must be supplied together")
        if route_file is not None:
            source_identity = _source_identity(args.bag, route_file, result_file)
        records, topic_types = read_bag(args.bag)
        evidence = build_evidence(
            records,
            topic_types,
            bag=args.bag,
            profile_id=args.profile_id,
            target_speed_mps=args.target_speed_mps,
            longitudinal_speed_source=args.longitudinal_speed_source,
            source_identity=source_identity,
        )
    except Exception as error:  # Preserve machine-readable failure evidence.
        evidence = _failure_evidence(args, error, source_identity)
    _write_json(evidence, json_path)
    _plot_evidence(evidence, plot_path)
    print(f"speed evidence status: {evidence['status']}")
    print(f"speed evidence JSON: {json_path}")
    print(f"speed evidence plot: {plot_path}")
    if evidence["status"] != "complete":
        for problem in evidence.get("quality", {}).get("problems", []):
            print(f"speed evidence problem: {problem}", file=sys.stderr)
        return 1
    return 0


def main() -> int:
    return run(_parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
