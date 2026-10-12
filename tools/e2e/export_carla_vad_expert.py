#!/usr/bin/env python3
"""Validate a CARLA expert episode and export VAD future-trajectory samples."""

from __future__ import annotations

import argparse
from bisect import bisect_left
import hashlib
import json
import math
from pathlib import Path
import shutil
from typing import Any, Iterable, Mapping, Sequence

from PIL import Image


PRIVATE_TINY_CAMERA_ORDER = (
    "CAM_FRONT",
    "CAM_BACK",
    "CAM_FRONT_LEFT",
    "CAM_BACK_LEFT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK_RIGHT",
)
PUBLIC_B2D_CAMERA_ORDER = (
    "CAM_FRONT",
    "CAM_FRONT_LEFT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK",
    "CAM_BACK_LEFT",
    "CAM_BACK_RIGHT",
)
DEFAULT_HORIZONS_S = (0.5, 1.0, 1.5, 2.0, 2.5, 3.0)


class DatasetError(ValueError):
    """Raised when an episode cannot satisfy the training-data contract."""


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path, help="finalized collector episode")
    parser.add_argument("--output", required=True, type=Path, help="new exported sample directory")
    parser.add_argument(
        "--horizons",
        default=",".join(str(value) for value in DEFAULT_HORIZONS_S),
        help="comma-separated future horizons in seconds",
    )
    parser.add_argument("--camera-state-tolerance-ms", type=float, default=10.0)
    parser.add_argument("--maximum-state-gap-ms", type=float, default=80.0)
    parser.add_argument("--maximum-route-cte-m", type=float, default=2.0)
    parser.add_argument(
        "--maximum-lane-invasions",
        type=parse_nonnegative_integer,
        default=0,
        help="maximum lane-invasion events accepted in one episode (default: 0)",
    )
    parser.add_argument("--allow-collisions", action="store_true")
    parser.add_argument("--skip-image-decode", action="store_true")
    args = parser.parse_args()
    try:
        args.horizons = parse_horizons(args.horizons)
    except DatasetError as error:
        parser.error(str(error))
    for name in (
        "camera_state_tolerance_ms",
        "maximum_state_gap_ms",
        "maximum_route_cte_m",
    ):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    return args


def parse_nonnegative_integer(text: str) -> int:
    try:
        value = int(text)
    except ValueError as error:
        raise argparse.ArgumentTypeError("must be a finite nonnegative integer") from error
    if value < 0:
        raise argparse.ArgumentTypeError("must be a finite nonnegative integer")
    return value


def parse_horizons(text: str) -> tuple[float, ...]:
    try:
        values = tuple(float(item.strip()) for item in text.split(",") if item.strip())
    except ValueError as error:
        raise DatasetError("future horizons must be comma-separated numbers") from error
    if not values or any(not math.isfinite(value) or value <= 0.0 for value in values):
        raise DatasetError("future horizons must be finite and positive")
    if any(second <= first for first, second in zip(values, values[1:])):
        raise DatasetError("future horizons must be strictly increasing")
    return values


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as error:
        raise DatasetError(f"cannot read {path}: {error}") from error
    records: list[dict[str, Any]] = []
    for line_number, line in enumerate(lines, start=1):
        if not line.strip():
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError as error:
            raise DatasetError(f"invalid JSON in {path}:{line_number}: {error}") from error
        if not isinstance(value, dict):
            raise DatasetError(f"{path}:{line_number} must contain a JSON object")
        records.append(value)
    if not records:
        raise DatasetError(f"{path} contains no records")
    return records


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while block := stream.read(1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def _number(record: Mapping[str, Any], *keys: str, default: float | None = None) -> float:
    for key in keys:
        value: Any = record
        for part in key.split("."):
            if not isinstance(value, Mapping) or part not in value:
                break
            value = value[part]
        else:
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise DatasetError(f"state field {key!r} must be numeric")
            result = float(value)
            if not math.isfinite(result):
                raise DatasetError(f"state field {key!r} is not finite")
            return result
    if default is not None:
        return float(default)
    raise DatasetError(f"state is missing one of: {', '.join(keys)}")


def state_time(record: Mapping[str, Any]) -> float:
    if "timestamp_ns" in record:
        return _number(record, "timestamp_ns") * 1.0e-9
    return _number(record, "timestamp_s", "timestamp", "elapsed_s")


def state_pose(record: Mapping[str, Any]) -> tuple[float, float, float, float]:
    return (
        _number(record, "x", "pose.x", "base_pose.x"),
        _number(record, "y", "pose.y", "base_pose.y"),
        _number(record, "z", "pose.z", "base_pose.z", default=0.0),
        _number(record, "yaw", "pose.yaw", "base_pose.yaw"),
    )


def state_command(record: Mapping[str, Any]) -> int:
    value = _number(record, "command", "vad_command", "route.command")
    command = int(value)
    if value != command or not 0 <= command <= 5:
        raise DatasetError(f"VAD command must be an integer in [0, 5], got {value!r}")
    return command


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def interpolate_state(
    states: Sequence[Mapping[str, Any]], times: Sequence[float], target_time: float
) -> dict[str, float] | None:
    """Interpolate a state without extrapolating outside an episode."""
    if target_time < times[0] - 1.0e-9 or target_time > times[-1] + 1.0e-9:
        return None
    upper = bisect_left(times, target_time)
    if upper < len(times) and abs(times[upper] - target_time) <= 1.0e-9:
        first = second = states[upper]
        ratio = 0.0
    elif upper == 0 or upper == len(times):
        return None
    else:
        first, second = states[upper - 1], states[upper]
        span = times[upper] - times[upper - 1]
        if span <= 0.0:
            raise DatasetError("state timestamps must be strictly increasing")
        ratio = (target_time - times[upper - 1]) / span
    first_pose = state_pose(first)
    second_pose = state_pose(second)
    yaw_delta = wrap_angle(second_pose[3] - first_pose[3])
    return {
        "x": first_pose[0] + ratio * (second_pose[0] - first_pose[0]),
        "y": first_pose[1] + ratio * (second_pose[1] - first_pose[1]),
        "z": first_pose[2] + ratio * (second_pose[2] - first_pose[2]),
        "yaw": wrap_angle(first_pose[3] + ratio * yaw_delta),
    }


def relative_pose(
    anchor: tuple[float, float, float, float], future: Mapping[str, float]
) -> list[float]:
    dx = float(future["x"]) - anchor[0]
    dy = float(future["y"]) - anchor[1]
    cosine = math.cos(anchor[3])
    sine = math.sin(anchor[3])
    return [
        cosine * dx + sine * dy,
        -sine * dx + cosine * dy,
        float(future["z"]) - anchor[2],
        wrap_angle(float(future["yaw"]) - anchor[3]),
    ]


def future_labels(
    states: Sequence[Mapping[str, Any]],
    times: Sequence[float],
    anchor: Mapping[str, Any],
    horizons: Sequence[float],
) -> dict[str, Any] | None:
    anchor_time = state_time(anchor)
    anchor_pose = state_pose(anchor)
    poses: list[list[float]] = []
    for horizon in horizons:
        future = interpolate_state(states, times, anchor_time + horizon)
        if future is None:
            return None
        poses.append(relative_pose(anchor_pose, future))
    previous_xy = (0.0, 0.0)
    deltas: list[list[float]] = []
    for pose in poses:
        deltas.append([pose[0] - previous_xy[0], pose[1] - previous_xy[1]])
        previous_xy = (pose[0], pose[1])
    return {
        "horizons_s": [float(value) for value in horizons],
        "poses_xyz_yaw": poses,
        "positions_xy": [[pose[0], pose[1]] for pose in poses],
        "deltas_xy": deltas,
        "valid_mask": [1] * len(horizons),
    }


def _nearest_state(
    states: Sequence[Mapping[str, Any]], times: Sequence[float], target: float
) -> tuple[Mapping[str, Any], float]:
    upper = bisect_left(times, target)
    candidates = []
    if upper < len(times):
        candidates.append((abs(times[upper] - target), upper))
    if upper > 0:
        candidates.append((abs(times[upper - 1] - target), upper - 1))
    if not candidates:
        raise DatasetError("cannot match a camera frame to an empty state series")
    delta, index = min(candidates)
    return states[index], delta


def _camera_timestamp(record: Mapping[str, Any]) -> float:
    if "timestamp_ns" in record:
        return _number(record, "timestamp_ns") * 1.0e-9
    return _number(record, "timestamp_s", "timestamp", "elapsed_s")


def _camera_paths(record: Mapping[str, Any]) -> Mapping[str, Any]:
    value = record.get("images", record.get("camera_paths"))
    if not isinstance(value, Mapping):
        raise DatasetError("camera frame must contain an images mapping")
    if tuple(value.keys()) != PRIVATE_TINY_CAMERA_ORDER:
        raise DatasetError(
            "camera image keys/order must be " + ",".join(PRIVATE_TINY_CAMERA_ORDER)
        )
    return value


def _validate_images(
    dataset: Path, camera_frame: Mapping[str, Any], decode: bool
) -> tuple[dict[str, str], tuple[int, int] | None]:
    relative_paths: dict[str, str] = {}
    dimensions: tuple[int, int] | None = None
    for camera, raw_path in _camera_paths(camera_frame).items():
        if not isinstance(raw_path, str) or not raw_path:
            raise DatasetError(f"{camera} image path must be a non-empty string")
        relative = Path(raw_path)
        if relative.is_absolute() or ".." in relative.parts:
            raise DatasetError(f"{camera} image path must stay inside the episode")
        path = dataset / relative
        if not path.is_file() or path.stat().st_size == 0:
            raise DatasetError(f"missing or empty image: {path}")
        if decode:
            try:
                with Image.open(path) as image:
                    image.load()
                    current_dimensions = image.size
                    if image.mode not in ("RGB", "RGBA"):
                        raise DatasetError(f"{path} has unsupported image mode {image.mode}")
            except OSError as error:
                raise DatasetError(f"cannot decode image {path}: {error}") from error
            if dimensions is None:
                dimensions = current_dimensions
            elif current_dimensions != dimensions:
                raise DatasetError("six-camera bundle has inconsistent image dimensions")
        relative_paths[camera] = relative.as_posix()
    return relative_paths, dimensions


def _one_hot(command: int) -> list[int]:
    return [int(index == command) for index in range(6)]


def _can_bus_18(state: Mapping[str, Any], previous_yaw: float | None) -> list[float]:
    x, y, z, yaw = state_pose(state)
    half_yaw = 0.5 * yaw
    yaw_delta_deg = 0.0 if previous_yaw is None else math.degrees(wrap_angle(yaw - previous_yaw))
    return [
        x,
        y,
        z,
        0.0,
        0.0,
        math.sin(half_yaw),
        math.cos(half_yaw),
        _number(state, "acceleration.x", "accel.x", "ax", default=0.0),
        _number(state, "acceleration.y", "accel.y", "ay", default=0.0),
        _number(state, "acceleration.z", "accel.z", "az", default=0.0),
        0.0,
        0.0,
        _number(state, "angular_velocity.z", "yaw_rate", default=0.0),
        _number(state, "velocity.x", "linear_velocity.x", "vx", "speed_mps", default=0.0),
        _number(state, "velocity.y", "linear_velocity.y", "vy", default=0.0),
        0.0,
        yaw,
        yaw_delta_deg,
    ]


def validate_state_series(
    states: Sequence[Mapping[str, Any]], maximum_gap_s: float
) -> tuple[list[float], dict[str, Any]]:
    times = [state_time(state) for state in states]
    if any(second <= first for first, second in zip(times, times[1:])):
        raise DatasetError("state timestamps must be strictly increasing")
    frames = [int(_number(state, "frame")) for state in states]
    if len(frames) != len(set(frames)):
        raise DatasetError("state frame numbers must be unique")
    for state in states:
        state_pose(state)
        state_command(state)
    gaps = [second - first for first, second in zip(times, times[1:])]
    maximum_gap = max(gaps, default=0.0)
    if maximum_gap > maximum_gap_s + 1.0e-9:
        raise DatasetError(
            f"maximum state gap {maximum_gap * 1000.0:.3f} ms exceeds "
            f"{maximum_gap_s * 1000.0:.3f} ms"
        )
    return times, {
        "state_count": len(states),
        "duration_s": times[-1] - times[0],
        "maximum_state_gap_ms": maximum_gap * 1000.0,
        "state_rate_hz": (len(states) - 1) / (times[-1] - times[0]) if len(states) > 1 else 0.0,
    }


def _event_count(states: Iterable[Mapping[str, Any]], *keys: str) -> int:
    count = 0
    previous = False
    for state in states:
        current = any(bool(state.get(key, False)) for key in keys)
        if current and not previous:
            count += 1
        previous = current
    return count


def export_episode(
    dataset: Path,
    output: Path,
    horizons: Sequence[float] = DEFAULT_HORIZONS_S,
    camera_state_tolerance_s: float = 0.010,
    maximum_state_gap_s: float = 0.080,
    maximum_route_cte_m: float = 2.0,
    maximum_lane_invasions: int = 0,
    allow_collisions: bool = False,
    decode_images: bool = True,
) -> dict[str, Any]:
    dataset = dataset.expanduser().resolve()
    output = output.expanduser().resolve()
    if not dataset.is_dir():
        raise DatasetError(f"input episode is not a directory: {dataset}")
    if (
        isinstance(maximum_lane_invasions, bool)
        or not isinstance(maximum_lane_invasions, int)
        or maximum_lane_invasions < 0
    ):
        raise DatasetError("maximum_lane_invasions must be a nonnegative integer")
    partial = output.with_name(output.name + ".partial")
    if output.exists() or partial.exists():
        raise DatasetError(f"output and partial output must not already exist: {output}")
    manifest_path = dataset / "manifest.json"
    route_path = dataset / "route.json"
    for required in (manifest_path, route_path, dataset / "states.jsonl", dataset / "camera_frames.jsonl"):
        if not required.is_file():
            raise DatasetError(f"episode is missing {required.name}")
    try:
        source_manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        route = json.loads(route_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise DatasetError(f"cannot load episode metadata: {error}") from error
    if not isinstance(source_manifest, dict) or not isinstance(route, dict):
        raise DatasetError("manifest.json and route.json must contain objects")
    states = read_jsonl(dataset / "states.jsonl")
    camera_frames = read_jsonl(dataset / "camera_frames.jsonl")
    times, state_metrics = validate_state_series(states, maximum_state_gap_s)
    collision_count = _event_count(states, "collision", "collision_event")
    lane_invasion_count = _event_count(states, "lane_invasion", "lane_invasion_event")
    if collision_count and not allow_collisions:
        raise DatasetError(f"episode contains {collision_count} collision event(s)")
    if lane_invasion_count > maximum_lane_invasions:
        raise DatasetError(
            f"episode contains {lane_invasion_count} lane invasion event(s), exceeding "
            f"configured maximum {maximum_lane_invasions}"
        )
    ctes = [
        abs(_number(state, "route_cte_m", "cross_track_error_m", default=0.0)) for state in states
    ]
    maximum_cte = max(ctes, default=0.0)
    if maximum_cte > maximum_route_cte_m:
        raise DatasetError(
            f"expert route CTE {maximum_cte:.3f} m exceeds {maximum_route_cte_m:.3f} m"
        )

    partial.mkdir(parents=True)
    sample_count = 0
    dropped_tail_count = 0
    dimensions: tuple[int, int] | None = None
    maximum_camera_state_delta_s = 0.0
    command_counts = {str(index): 0 for index in range(6)}
    previous_yaw: float | None = None
    previous_token = ""
    samples_path = partial / "samples.jsonl"
    try:
        with samples_path.open("w", encoding="utf-8") as stream:
            for camera_frame in camera_frames:
                camera_time = _camera_timestamp(camera_frame)
                state, delta = _nearest_state(states, times, camera_time)
                maximum_camera_state_delta_s = max(maximum_camera_state_delta_s, delta)
                if delta > camera_state_tolerance_s + 1.0e-9:
                    raise DatasetError(
                        f"camera/state delta {delta * 1000.0:.3f} ms exceeds "
                        f"{camera_state_tolerance_s * 1000.0:.3f} ms"
                    )
                image_paths, current_dimensions = _validate_images(
                    dataset, camera_frame, decode_images
                )
                if current_dimensions is not None:
                    if dimensions is None:
                        dimensions = current_dimensions
                    elif current_dimensions != dimensions:
                        raise DatasetError("camera resolution changes within the episode")
                labels = future_labels(states, times, state, horizons)
                if labels is None:
                    dropped_tail_count += 1
                    continue
                frame = int(_number(camera_frame, "frame"))
                command = state_command(state)
                token = f"{source_manifest.get('episode_id', dataset.name)}_{frame:08d}"
                x, y, z, yaw = state_pose(state)
                can_bus = _can_bus_18(state, previous_yaw)
                previous_yaw = yaw
                cameras = {
                    name: {
                        "path": image_paths[name],
                        "private_tiny_index": PRIVATE_TINY_CAMERA_ORDER.index(name),
                        "public_b2d_index": PUBLIC_B2D_CAMERA_ORDER.index(name),
                    }
                    for name in PRIVATE_TINY_CAMERA_ORDER
                }
                sample = {
                    "schema_version": 1,
                    "token": token,
                    "prev": previous_token,
                    "frame": frame,
                    "timestamp_ns": int(round(camera_time * 1.0e9)),
                    "episode_id": source_manifest.get("episode_id", dataset.name),
                    "town": source_manifest.get("town", route.get("town")),
                    "weather": source_manifest.get("weather", route.get("weather")),
                    "route_scenario": route.get("scenario"),
                    "expert": source_manifest.get("expert", "carla.BasicAgent"),
                    "pose_map_xyz_yaw": [x, y, z, yaw],
                    "can_bus_18": can_bus,
                    "vad_command_0based": command,
                    "b2d_command_1based": command + 1,
                    "command_one_hot": _one_hot(command),
                    "route_progress_m": _number(
                        state, "route_progress_m", "progress_m", default=0.0
                    ),
                    "route_cte_m": _number(
                        state, "route_cte_m", "cross_track_error_m", default=0.0
                    ),
                    "camera_order_private_tiny": list(PRIVATE_TINY_CAMERA_ORDER),
                    "camera_order_public_b2d": list(PUBLIC_B2D_CAMERA_ORDER),
                    "cameras": cameras,
                    "future_expert": labels,
                }
                stream.write(json.dumps(sample, separators=(",", ":")) + "\n")
                command_counts[str(command)] += 1
                sample_count += 1
                previous_token = token
        if sample_count == 0:
            raise DatasetError("no camera anchor has a complete future horizon")
        exported_manifest = {
            "schema_version": 1,
            "status": "validated",
            "source_episode": str(dataset),
            "source_episode_id": source_manifest.get("episode_id", dataset.name),
            "source_hashes": {
                path.name: sha256_file(path)
                for path in (
                    manifest_path,
                    route_path,
                    dataset / "states.jsonl",
                    dataset / "camera_frames.jsonl",
                )
            },
            "sample_file": "samples.jsonl",
            "sample_count": sample_count,
            "camera_anchor_count": len(camera_frames),
            "dropped_tail_anchor_count": dropped_tail_count,
            "camera_order_private_tiny": list(PRIVATE_TINY_CAMERA_ORDER),
            "camera_order_public_b2d": list(PUBLIC_B2D_CAMERA_ORDER),
            "image_dimensions": list(dimensions) if dimensions is not None else None,
            "future_horizons_s": [float(value) for value in horizons],
            "future_tail_policy": "drop_anchor_without_full_horizon",
            "state_metrics": state_metrics,
            "maximum_camera_state_delta_ms": maximum_camera_state_delta_s * 1000.0,
            "maximum_route_cte_m": maximum_cte,
            "collision_event_count": collision_count,
            "lane_invasion_event_count": lane_invasion_count,
            "maximum_lane_invasions": maximum_lane_invasions,
            "command_sample_counts": command_counts,
            "coordinate_reference": "ROS map x-forward/y-left; future in anchor base_link",
            "can_bus_contract": "autoware_tensorrt_vad InputCanBusConverter 18D",
        }
        (partial / "manifest.json").write_text(
            json.dumps(exported_manifest, indent=2) + "\n", encoding="utf-8"
        )
        shutil.copy2(route_path, partial / "route.json")
        partial.replace(output)
        return exported_manifest
    except BaseException:
        shutil.rmtree(partial, ignore_errors=True)
        raise


def main() -> int:
    args = _parse_args()
    report = export_episode(
        args.input,
        args.output,
        horizons=args.horizons,
        camera_state_tolerance_s=args.camera_state_tolerance_ms * 1.0e-3,
        maximum_state_gap_s=args.maximum_state_gap_ms * 1.0e-3,
        maximum_route_cte_m=args.maximum_route_cte_m,
        maximum_lane_invasions=args.maximum_lane_invasions,
        allow_collisions=args.allow_collisions,
        decode_images=not args.skip_image_decode,
    )
    print(
        f"PASS samples={report['sample_count']} anchors={report['camera_anchor_count']} "
        f"tail_dropped={report['dropped_tail_anchor_count']} "
        f"max_cte={report['maximum_route_cte_m']:.3f}m output={args.output}"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except DatasetError as error:
        raise SystemExit(f"ERROR: {error}") from error
