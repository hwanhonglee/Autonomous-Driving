#!/usr/bin/env python3

"""Create a map-frame route from a raw CARLA route and a pinned map bundle."""

from __future__ import annotations

import argparse
from copy import deepcopy
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile


class AlignmentError(RuntimeError):
    """Raised when a route or map-bundle alignment contract is invalid."""


@dataclass(frozen=True)
class MapTransform:
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float

    @property
    def enabled(self) -> bool:
        return any(abs(value) > 1.0e-12 for value in asdict(self).values())


def _finite_number(value, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise AlignmentError(f"{label} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise AlignmentError(f"{label} must be finite")
    return result


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def format_ros_float(value: float) -> str:
    """Keep ROS launch substitutions typed as DOUBLE even for integral values."""
    text = f"{value:.17g}"
    return text if any(character in text for character in ".eE") else f"{text}.0"


def load_map_bundle(path: Path) -> tuple[dict, MapTransform]:
    bundle_path = path / "map_bundle.json" if path.is_dir() else path
    try:
        payload = json.loads(bundle_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise AlignmentError(f"cannot read map bundle {bundle_path}: {error}") from error
    if not isinstance(payload, dict):
        raise AlignmentError("map bundle root must be an object")
    if payload.get("schema_version") != 1:
        raise AlignmentError("map bundle schema_version must be 1")
    values = payload.get("carla_to_map_transform")
    if not isinstance(values, dict):
        raise AlignmentError("map bundle has no carla_to_map_transform object")
    transform = MapTransform(
        x_m=_finite_number(values.get("x_m"), "carla_to_map_transform.x_m"),
        y_m=_finite_number(values.get("y_m"), "carla_to_map_transform.y_m"),
        z_m=_finite_number(values.get("z_m"), "carla_to_map_transform.z_m"),
        yaw_rad=_finite_number(values.get("yaw_rad"), "carla_to_map_transform.yaw_rad"),
    )
    return payload, transform


def transform_pose(pose: dict, transform: MapTransform, label: str) -> dict:
    if not isinstance(pose, dict):
        raise AlignmentError(f"{label} must be an object")
    values = {
        key: _finite_number(pose.get(key), f"{label}.{key}")
        for key in ("x", "y", "z", "yaw")
    }
    cosine = math.cos(transform.yaw_rad)
    sine = math.sin(transform.yaw_rad)
    output = deepcopy(pose)
    output["x"] = transform.x_m + cosine * values["x"] - sine * values["y"]
    output["y"] = transform.y_m + sine * values["x"] + cosine * values["y"]
    output["z"] = transform.z_m + values["z"]
    output["yaw"] = normalize_angle(values["yaw"] + transform.yaw_rad)
    return output


def align_route_payload(
    route: dict,
    transform: MapTransform,
    *,
    source_path: Path,
    source_sha256: str,
    bundle: dict,
) -> dict:
    if not isinstance(route, dict):
        raise AlignmentError("route JSON root must be an object")
    if "coordinate_alignment" in route:
        raise AlignmentError(
            "route already contains coordinate_alignment; refusing a possible double transform"
        )
    if not isinstance(route.get("route"), list) or not route["route"]:
        raise AlignmentError("route must be a non-empty array")
    if not isinstance(route.get("spawn_point"), str) or not route["spawn_point"].strip():
        raise AlignmentError("spawn_point must remain a non-empty raw CARLA transform string")

    output = deepcopy(route)
    output["start_ros_pose"] = transform_pose(
        route.get("start_ros_pose"), transform, "start_ros_pose"
    )
    output["goal_ros_pose"] = transform_pose(
        route.get("goal_ros_pose"), transform, "goal_ros_pose"
    )
    output["route"] = [
        transform_pose(point, transform, f"route[{index}]")
        for index, point in enumerate(route["route"])
    ]
    output["coordinate_alignment"] = {
        "schema_version": 1,
        "source_frame": "carla_map",
        "target_frame": "map",
        "source_route": str(source_path.resolve()),
        "source_route_sha256": source_sha256,
        "map_bundle_profile": bundle.get("profile", ""),
        "carla_to_map_transform": asdict(transform),
    }
    return output


def validate_existing_alignment(route: dict, bundle: dict, transform: MapTransform) -> None:
    metadata = route.get("coordinate_alignment")
    if not isinstance(metadata, dict):
        raise AlignmentError("coordinate_alignment must be an object")
    if metadata.get("schema_version") != 1:
        raise AlignmentError("coordinate_alignment.schema_version must be 1")
    if metadata.get("source_frame") != "carla_map" or metadata.get("target_frame") != "map":
        raise AlignmentError("existing route alignment must be carla_map -> map")
    if metadata.get("map_bundle_profile") != bundle.get("profile", ""):
        raise AlignmentError("existing route alignment belongs to a different map bundle profile")
    existing = metadata.get("carla_to_map_transform")
    if not isinstance(existing, dict):
        raise AlignmentError("existing route alignment has no transform object")
    for field, expected in asdict(transform).items():
        actual = _finite_number(existing.get(field), f"coordinate_alignment.{field}")
        if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1.0e-12):
            raise AlignmentError(
                f"existing route alignment {field}={actual:.17g} does not match "
                f"map bundle value {expected:.17g}"
            )
    for key in ("start_ros_pose", "goal_ros_pose"):
        transform_pose(route.get(key), MapTransform(0.0, 0.0, 0.0, 0.0), key)
    points = route.get("route")
    if not isinstance(points, list) or not points:
        raise AlignmentError("route must be a non-empty array")
    for index, point in enumerate(points):
        transform_pose(point, MapTransform(0.0, 0.0, 0.0, 0.0), f"route[{index}]")


def _sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _canonical_json_bytes(payload: dict) -> bytes:
    return (json.dumps(payload, indent=2, sort_keys=False, ensure_ascii=True) + "\n").encode(
        "utf-8"
    )


def _write_atomic(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.is_file() and path.read_bytes() == data:
        return
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, path)
    finally:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass


def prepare_aligned_route(
    route_path: Path,
    bundle_path: Path,
    *,
    output_path: Path | None = None,
    runtime_dir: Path | None = None,
) -> dict:
    route_path = route_path.expanduser().resolve()
    try:
        source_bytes = route_path.read_bytes()
        route = json.loads(source_bytes.decode("utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise AlignmentError(f"cannot read route JSON {route_path}: {error}") from error
    source_sha256 = _sha256_bytes(source_bytes)
    bundle, transform = load_map_bundle(bundle_path.expanduser().resolve())
    if isinstance(route, dict) and "coordinate_alignment" in route:
        validate_existing_alignment(route, bundle, transform)
        if output_path is not None:
            output_path = output_path.expanduser().resolve()
            output_bytes = _canonical_json_bytes(route)
            _write_atomic(output_path, output_bytes)
            aligned_route = output_path
            aligned_sha256 = _sha256_bytes(output_bytes)
        else:
            aligned_route = route_path
            aligned_sha256 = source_sha256
        return {
            "status": "PASS",
            "source_route": str(route_path),
            "source_route_sha256": source_sha256,
            "aligned_route": str(aligned_route.resolve()),
            "aligned_route_sha256": aligned_sha256,
            "profile": bundle.get("profile", ""),
            "alignment_enabled": transform.enabled,
            "already_aligned": True,
            "carla_to_map_transform": asdict(transform),
        }
    aligned = align_route_payload(
        route,
        transform,
        source_path=route_path,
        source_sha256=source_sha256,
        bundle=bundle,
    )
    output_bytes = _canonical_json_bytes(aligned)
    digest = _sha256_bytes(
        source_bytes + json.dumps(asdict(transform), sort_keys=True).encode("ascii")
    )[:16]
    if output_path is None:
        runtime_dir = runtime_dir or Path(
            os.environ.get("XDG_RUNTIME_DIR", f"/tmp/autoware_e2e_{os.getuid()}")
        ) / "autoware_e2e" / "aligned_routes"
        output_path = runtime_dir / f"{route_path.stem}.{digest}.json"
    else:
        output_path = output_path.expanduser().resolve()
    _write_atomic(output_path, output_bytes)
    return {
        "status": "PASS",
        "source_route": str(route_path),
        "source_route_sha256": source_sha256,
        "aligned_route": str(output_path.resolve()),
        "aligned_route_sha256": _sha256_bytes(output_bytes),
        "profile": bundle.get("profile", ""),
        "alignment_enabled": transform.enabled,
        "already_aligned": False,
        "carla_to_map_transform": asdict(transform),
    }


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Apply a pinned CARLA-to-map transform to all ROS poses in a route JSON."
    )
    parser.add_argument("route_json", type=Path)
    parser.add_argument("map_bundle", type=Path, help="map_bundle.json or its parent directory")
    output_group = parser.add_mutually_exclusive_group()
    output_group.add_argument("--output", type=Path)
    output_group.add_argument("--runtime-dir", type=Path)
    parser.add_argument(
        "--shell-values",
        action="store_true",
        help="print aligned path, enable flag, x, y, z, and yaw on separate lines",
    )
    parser.add_argument("--json", action="store_true", help="print preparation metadata as JSON")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.shell_values and args.json:
        raise SystemExit("--shell-values and --json are mutually exclusive")
    try:
        result = prepare_aligned_route(
            args.route_json,
            args.map_bundle,
            output_path=args.output,
            runtime_dir=args.runtime_dir,
        )
    except AlignmentError as error:
        raise SystemExit(f"Route alignment failed: {error}") from error
    if args.shell_values:
        transform = result["carla_to_map_transform"]
        print(result["aligned_route"])
        print("true" if result["alignment_enabled"] else "false")
        print(format_ros_float(transform["x_m"]))
        print(format_ros_float(transform["y_m"]))
        print(format_ros_float(transform["z_m"]))
        print(format_ros_float(transform["yaw_rad"]))
    elif args.json:
        print(json.dumps(result, indent=2))
    else:
        print(
            "Route alignment: PASS "
            f"(profile={result['profile']}, output={result['aligned_route']})"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
