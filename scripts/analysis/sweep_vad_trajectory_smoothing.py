#!/usr/bin/env python3
"""Offline endpoint-fixed smoothing sweep for frozen VAD trajectories.

The geometry pass is ROS-graph free.  ``--run-mpc`` additionally runs the
existing deterministic Autoware MPC plant-in-loop harness on an isolated ROS
domain; it never plays a bag or starts CARLA/the full Autoware stack.
"""

from __future__ import annotations

import argparse
import copy
import csv
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
from typing import Any, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402


REPO_ROOT = Path(__file__).resolve().parents[2]
VAD_SCRIPTS = REPO_ROOT / "autoware_e2e_vad_launch/scripts"
if str(VAD_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(VAD_SCRIPTS))

from vad_route_logic import resample_trajectory_points  # noqa: E402


def _load_replay_module():
    path = REPO_ROOT / "scripts/e2e/mpc_replay_sweep.py"
    spec = importlib.util.spec_from_file_location("vad_smoothing_mpc_replay", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load MPC replay helper: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


MPC_REPLAY = _load_replay_module()


def endpoint_fixed_whittaker(xy: np.ndarray, strength: float) -> np.ndarray:
    """Minimize ||z-x||^2 + strength*||D2 z||^2 with fixed endpoints."""
    xy = np.asarray(xy, dtype=float)
    if xy.ndim != 2 or xy.shape[1] != 2 or len(xy) < 3:
        raise ValueError("xy must contain at least three planar points")
    if not math.isfinite(strength) or strength < 0.0:
        raise ValueError("smoothing strength must be finite and non-negative")
    if strength == 0.0:
        return xy.copy()

    count = len(xy)
    second_difference = np.zeros((count - 2, count), dtype=float)
    for index in range(count - 2):
        second_difference[index, index : index + 3] = (1.0, -2.0, 1.0)
    system = np.eye(count) + strength * (
        second_difference.T @ second_difference
    )
    fixed = np.asarray((0, count - 1), dtype=int)
    free = np.arange(1, count - 1, dtype=int)
    result = np.empty_like(xy)
    result[fixed] = xy[fixed]
    right_hand_side = xy[free] - system[np.ix_(free, fixed)] @ result[fixed]
    result[free] = np.linalg.solve(system[np.ix_(free, free)], right_hand_side)
    return result


def recompute_tangent_orientations(points: Sequence[Any], xy: np.ndarray) -> None:
    """Set yaw from centered path tangents while retaining all non-pose fields."""
    if len(points) != len(xy) or len(points) < 2:
        raise ValueError("points and xy must have matching non-trivial lengths")
    for index, point in enumerate(points):
        left = max(0, index - 1)
        right = min(len(xy) - 1, index + 1)
        delta = xy[right] - xy[left]
        if np.linalg.norm(delta) <= 1.0e-9:
            raise ValueError(f"zero tangent at trajectory point {index}")
        yaw = math.atan2(float(delta[1]), float(delta[0]))
        point.pose.orientation.x = 0.0
        point.pose.orientation.y = 0.0
        point.pose.orientation.z = math.sin(0.5 * yaw)
        point.pose.orientation.w = math.cos(0.5 * yaw)


def _trajectory_xy(points: Sequence[Any]) -> np.ndarray:
    return np.asarray(
        [[point.pose.position.x, point.pose.position.y] for point in points],
        dtype=float,
    )


def build_smoothed_points(
    source_points: Sequence[Any], interval_m: float, strength: float
) -> list[Any]:
    points = resample_trajectory_points(source_points, interval_m)
    xy = _trajectory_xy(points)
    smoothed = endpoint_fixed_whittaker(xy, strength)
    for point, position in zip(points, smoothed):
        point.pose.position.x = float(position[0])
        point.pose.position.y = float(position[1])
    if strength > 0.0:
        recompute_tangent_orientations(points, smoothed)
    return points


def _project_distance(points: np.ndarray, line: np.ndarray) -> np.ndarray:
    points = np.atleast_2d(np.asarray(points, dtype=float))
    line = np.asarray(line, dtype=float)
    starts = line[:-1]
    vectors = line[1:] - starts
    length_squared = np.einsum("ij,ij->i", vectors, vectors)
    valid = length_squared > 1.0e-12
    relative = points[:, None, :] - starts[None, :, :]
    fractions = np.zeros((len(points), len(vectors)), dtype=float)
    fractions[:, valid] = np.clip(
        np.einsum("pij,ij->pi", relative[:, valid], vectors[valid])
        / length_squared[valid][None, :],
        0.0,
        1.0,
    )
    nearest = starts[None, :, :] + fractions[:, :, None] * vectors[None, :, :]
    squared = np.einsum("pij,pij->pi", points[:, None, :] - nearest, points[:, None, :] - nearest)
    squared[:, ~valid] = np.inf
    return np.sqrt(np.min(squared, axis=1))


def _curvature(xy: np.ndarray) -> np.ndarray:
    distance = np.concatenate(
        ([0.0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
    )
    dx = np.gradient(xy[:, 0], distance)
    dy = np.gradient(xy[:, 1], distance)
    ddx = np.gradient(dx, distance)
    ddy = np.gradient(dy, distance)
    return (dx * ddy - dy * ddx) / np.maximum(
        (dx * dx + dy * dy) ** 1.5, 1.0e-9
    )


def geometry_metrics(
    points: Sequence[Any], baseline_dense_xy: np.ndarray, route_xy: np.ndarray, wheel_base_m: float
) -> dict[str, float]:
    xy = _trajectory_xy(points)
    curvature = _curvature(xy)
    required_steer = np.arctan(wheel_base_m * curvature)
    route_error = _project_distance(xy, route_xy)
    forward_deviation = _project_distance(xy, baseline_dense_xy)
    reverse_deviation = _project_distance(baseline_dense_xy, xy)
    segment = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    return {
        "point_count": int(len(xy)),
        "length_m": float(segment.sum()),
        "route_cte_p95_m": float(np.percentile(route_error, 95)),
        "route_cte_max_m": float(np.max(route_error)),
        "baseline_deviation_p95_m": float(
            max(np.percentile(forward_deviation, 95), np.percentile(reverse_deviation, 95))
        ),
        "baseline_deviation_max_m": float(
            max(np.max(forward_deviation), np.max(reverse_deviation))
        ),
        "required_steer_p95_rad": float(np.percentile(np.abs(required_steer), 95)),
        "required_steer_peak_rad": float(np.max(np.abs(required_steer))),
        "required_steer_total_variation_rad": float(np.abs(np.diff(required_steer)).sum()),
        "curvature_p95_inv_m": float(np.percentile(np.abs(curvature), 95)),
        "curvature_peak_inv_m": float(np.max(np.abs(curvature))),
        "endpoint_shift_m": float(
            max(
                np.linalg.norm(xy[0] - baseline_dense_xy[0]),
                np.linalg.norm(xy[-1] - baseline_dense_xy[-1]),
            )
        ),
    }


def _parse_strengths(raw: str) -> list[float]:
    values = sorted({float(value) for value in raw.split(",") if value.strip()})
    if not values or any(not math.isfinite(value) or value <= 0.0 for value in values):
        raise argparse.ArgumentTypeError("strengths must be positive finite values")
    return values


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", action="append", required=True, type=Path)
    parser.add_argument("--route-file", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--strengths", type=_parse_strengths, default=_parse_strengths("30,100,300"))
    parser.add_argument("--interval-m", type=float, default=0.25)
    parser.add_argument("--corridor-half-width-m", type=float, default=0.5)
    parser.add_argument("--wheel-base-m", type=float, default=2.79)
    parser.add_argument("--run-mpc", action="store_true")
    parser.add_argument("--ros-domain-id", type=int, default=167)
    parser.add_argument("--mpc-input-delay-sec", type=float, default=0.12)
    parser.add_argument("--mpc-steer-tau-sec", type=float, default=0.15)
    args = parser.parse_args(argv)
    if args.interval_m <= 0.0 or args.corridor_half_width_m <= 0.0 or args.wheel_base_m <= 0.0:
        parser.error("interval, corridor width, and wheel base must be positive")
    if not 0 <= args.ros_domain_id <= 232:
        parser.error("ROS domain ID must be in [0, 232]")
    return args


def _route_xy(path: Path) -> np.ndarray:
    payload = json.loads(path.read_text(encoding="utf-8"))
    return np.asarray([[point["x"], point["y"]] for point in payload["route"]], dtype=float)


def _snapshot_with_points(snapshot: Any, points: Sequence[Any]) -> Any:
    result = copy.deepcopy(snapshot)
    result.trajectory.points = list(points)
    result.trajectory_hash = MPC_REPLAY.trajectory_digest(result.trajectory)
    result.trajectory_length_m, result.net_heading_change_rad = MPC_REPLAY.trajectory_geometry(
        result.trajectory
    )
    return result


def _run_mpc(snapshot: Any, bag: Path, output_dir: Path, args: argparse.Namespace) -> dict[str, Any]:
    replay_args = MPC_REPLAY.parse_args(
        [
            str(bag),
            str(output_dir),
            "--input-delays",
            str(args.mpc_input_delay_sec),
            "--steer-taus",
            str(args.mpc_steer_tau_sec),
            "--duration",
            "6.0",
            "--simulation-step",
            "0.05",
            "--ros-domain-id",
            str(args.ros_domain_id),
        ]
    )
    result = MPC_REPLAY.run_candidate(
        snapshot,
        output_dir,
        args.mpc_input_delay_sec,
        args.mpc_steer_tau_sec,
        replay_args,
        1,
    )
    return result["metrics"]


def _aggregate(rows: Sequence[dict[str, Any]]) -> dict[str, Any]:
    candidates = sorted({row["candidate"] for row in rows})
    output = {}
    for candidate in candidates:
        selected = [row for row in rows if row["candidate"] == candidate]
        metrics = {}
        for section in ("geometry", "mpc"):
            keys = sorted({key for row in selected for key in row.get(section, {})})
            metrics[section] = {
                key: float(np.median([row[section][key] for row in selected if key in row.get(section, {})]))
                for key in keys
            }
        output[candidate] = metrics
    return output


def _write_csv(path: Path, rows: Sequence[dict[str, Any]]) -> None:
    flattened = []
    for row in rows:
        record = {"bag": row["bag"], "candidate": row["candidate"]}
        record.update({f"geometry_{key}": value for key, value in row["geometry"].items()})
        record.update({f"mpc_{key}": value for key, value in row.get("mpc", {}).items()})
        flattened.append(record)
    keys = sorted({key for row in flattened for key in row})
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(flattened)


def _render(path: Path, rows: Sequence[dict[str, Any]], aggregate: dict[str, Any]) -> None:
    labels = list(aggregate)
    x = np.arange(len(labels))
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
    geometry = [aggregate[label]["geometry"] for label in labels]
    axes[0, 0].bar(x, [item["required_steer_peak_rad"] for item in geometry])
    axes[0, 0].set_ylabel("reference steer peak [rad]")
    axes[0, 1].bar(x, [item["required_steer_total_variation_rad"] for item in geometry])
    axes[0, 1].set_ylabel("reference steer TV [rad]")
    axes[1, 0].bar(x, [item["route_cte_max_m"] for item in geometry])
    axes[1, 0].axhline(0.5, color="black", linestyle="--", linewidth=1)
    axes[1, 0].set_ylabel("route CTE max [m]")
    if all(aggregate[label]["mpc"] for label in labels):
        axes[1, 1].bar(x - 0.18, [aggregate[label]["mpc"]["cte_p95_m"] for label in labels], 0.36, label="CTE p95")
        axes[1, 1].bar(x + 0.18, [aggregate[label]["mpc"]["steer_command_peak_rad"] for label in labels], 0.36, label="steer peak")
        axes[1, 1].legend()
    axes[1, 1].set_ylabel("MPC replay metric")
    for axis in axes.flat:
        axis.set_xticks(x, labels, rotation=25, ha="right")
        axis.grid(axis="y", alpha=0.25)
    fig.suptitle("Frozen VAD trajectory smoothing: median across bags")
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    output_dir = args.output_dir.expanduser().resolve()
    if output_dir.exists() and any(output_dir.iterdir()):
        raise SystemExit(f"output directory is not empty: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    route_xy = _route_xy(args.route_file)
    rows = []

    if args.run_mpc:
        os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
        rclpy.init(args=None)
    try:
        for bag_index, bag in enumerate(args.bag, start=1):
            bag = bag.expanduser().resolve()
            snapshot = MPC_REPLAY.load_frozen_snapshot(bag)
            dense_baseline = build_smoothed_points(snapshot.trajectory.points, args.interval_m, 0.0)
            baseline_dense_xy = _trajectory_xy(dense_baseline)
            candidates = [("baseline", copy.deepcopy(snapshot.trajectory.points))]
            candidates.append(("resample_only", dense_baseline))
            candidates.extend(
                (
                    f"smooth_{strength:g}",
                    build_smoothed_points(snapshot.trajectory.points, args.interval_m, strength),
                )
                for strength in args.strengths
            )
            for candidate_name, points in candidates:
                geometry_points = (
                    build_smoothed_points(points, args.interval_m, 0.0)
                    if candidate_name == "baseline"
                    else points
                )
                geometry = geometry_metrics(
                    geometry_points, baseline_dense_xy, route_xy, args.wheel_base_m
                )
                if geometry["route_cte_max_m"] > args.corridor_half_width_m + 1.0e-6:
                    raise RuntimeError(
                        f"{bag.name}/{candidate_name} leaves corridor: "
                        f"{geometry['route_cte_max_m']:.3f} m"
                    )
                row = {
                    "bag": str(bag),
                    "trajectory_index": snapshot.trajectory_index,
                    "candidate": candidate_name,
                    "geometry": geometry,
                }
                if args.run_mpc:
                    print(
                        f"[{bag_index}/{len(args.bag)}] {candidate_name}: isolated MPC replay",
                        flush=True,
                    )
                    candidate_snapshot = _snapshot_with_points(snapshot, points)
                    candidate_dir = output_dir / f"bag_{bag_index:02d}" / candidate_name
                    row["mpc"] = _run_mpc(
                        candidate_snapshot, bag, candidate_dir, args
                    )
                rows.append(row)
    finally:
        if args.run_mpc:
            rclpy.shutdown()

    aggregate = _aggregate(rows)
    summary = {
        "schema_version": 1,
        "method": "endpoint_fixed_whittaker_on_uniform_arc_length_samples",
        "objective": "min ||z-x||^2 + lambda ||D2 z||^2",
        "configuration": {
            "interval_m": args.interval_m,
            "strengths": args.strengths,
            "corridor_half_width_m": args.corridor_half_width_m,
            "wheel_base_m": args.wheel_base_m,
            "mpc_replay": args.run_mpc,
            "ros_domain_id": args.ros_domain_id if args.run_mpc else None,
            "mpc_input_delay_sec": args.mpc_input_delay_sec,
            "mpc_steer_tau_sec": args.mpc_steer_tau_sec,
        },
        "aggregate_median": aggregate,
        "runs": rows,
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    _write_csv(output_dir / "per_bag_metrics.csv", rows)
    _render(output_dir / "comparison.png", rows, aggregate)
    print(json.dumps(aggregate, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
