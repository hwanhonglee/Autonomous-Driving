#!/usr/bin/env python3
"""Generate a reproducible ROS-frame PCD from a running CARLA world.

The collector deliberately uses CARLA semantic-LiDAR physics ray-cast returns
from map collision geometry.  It never synthesizes localization points from
OpenDRIVE or Lanelet2 lines.  The server must already be running the expected
map; this program never calls ``load_world``.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
from queue import Empty, Queue
import shutil
import signal
import struct
import subprocess
import sys
import tempfile
import time
from typing import Any, Callable, Iterable, Mapping, Sequence
import xml.etree.ElementTree as ET

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_EXPECTED_MAP = "/Game/Carla/Maps/Town10HD_Opt"
DEFAULT_EXPECTED_OPENDRIVE_SHA256 = (
    "5d883b799f634030af92be1e9d79d107845540ba04338e8c60e095be1aef7be7"
)
EXPECTED_CARLA_VERSION = "0.9.15"
SEMANTIC_DTYPE = np.dtype(
    [
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("cos_incidence", "<f4"),
        ("object_index", "<u4"),
        ("object_tag", "<u4"),
    ]
)

# CARLA 0.9.15 CityObjectLabel values from rpc/ObjectLabel.h.  Untagged, Sky,
# all vulnerable-road-user/vehicle classes, and Dynamic are intentionally
# absent.  Parked vehicle meshes therefore cannot leak into the localization
# map merely because no vehicle actor is moving.
STATIC_TAG_NAMES = {
    1: "Roads",
    2: "Sidewalks",
    3: "Buildings",
    4: "Walls",
    5: "Fences",
    6: "Poles",
    7: "TrafficLight",
    8: "TrafficSigns",
    9: "Vegetation",
    10: "Terrain",
    20: "Static",
    22: "Other",
    23: "Water",
    24: "RoadLines",
    25: "Ground",
    26: "Bridge",
    27: "RailTrack",
    28: "GuardRail",
}
DEFAULT_STATIC_TAGS = frozenset(STATIC_TAG_NAMES)
EXCLUDED_DYNAMIC_TAG_NAMES = {
    12: "Pedestrians",
    13: "Rider",
    14: "Car",
    15: "Truck",
    16: "Bus",
    17: "Train",
    18: "Motorcycle",
    19: "Bicycle",
    21: "Dynamic",
}
EXPECTED_CITY_OBJECT_LABELS = {
    "NONE": 0,
    **{name: value for value, name in STATIC_TAG_NAMES.items()},
    "Sky": 11,
    **{name: value for value, name in EXCLUDED_DYNAMIC_TAG_NAMES.items()},
    "Any": 255,
}
PCD_HEADER_KEYS = frozenset(
    ("VERSION", "FIELDS", "SIZE", "TYPE", "COUNT", "WIDTH", "HEIGHT", "POINTS", "DATA")
)
MAX_SOURCE_TRANSLATION_ERROR_M = 0.05
MAX_SOURCE_ROTATION_ERROR_RAD = math.radians(0.1)


class MapGenerationError(RuntimeError):
    """Raised when a generated map would violate the evidence contract."""


class InterruptedGeneration(MapGenerationError):
    """Raised after SIGINT or SIGTERM so the current checkpoint stays resumable."""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while block := stream.read(8 * 1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def atomic_write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def atomic_save_npy(path: Path, points: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "wb") as stream:
            np.save(stream, np.asarray(points, dtype="<f4"), allow_pickle=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def atomic_copy_file(source: Path, target: Path) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.is_file():
        if sha256_file(target) == sha256_file(source):
            return
        raise MapGenerationError(f"immutable input snapshot differs from source: {target}")
    if target.exists() or target.is_symlink():
        raise MapGenerationError(f"input snapshot target is unsafe: {target}")
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{target.name}.", suffix=".tmp", dir=str(target.parent)
    )
    try:
        with source.open("rb") as input_stream, os.fdopen(descriptor, "wb") as output_stream:
            shutil.copyfileobj(input_stream, output_stream, length=8 * 1024 * 1024)
            output_stream.flush()
            os.fsync(output_stream.fileno())
        os.replace(temporary, target)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def _stable_plan_payload(plan: Mapping[str, Any]) -> dict[str, Any]:
    stable = dict(plan)
    stable.pop("created_at_utc", None)
    stable.pop("fingerprint", None)
    return stable


def plan_fingerprint(plan: Mapping[str, Any]) -> str:
    payload = json.dumps(
        _stable_plan_payload(plan), sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return sha256_bytes(payload)


def validate_resume_plan(existing: Mapping[str, Any], proposed: Mapping[str, Any]) -> None:
    stored = existing.get("fingerprint")
    actual_existing = plan_fingerprint(existing)
    actual_proposed = plan_fingerprint(proposed)
    if stored != actual_existing:
        raise MapGenerationError("existing scan plan fingerprint is corrupt")
    if actual_existing != actual_proposed:
        raise MapGenerationError(
            "resume scan plan differs from the current map, waypoint graph, or parameters"
        )


def _finite_positive(value: float, label: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise MapGenerationError(f"{label} must be finite and positive")
    return number


def _waypoint_record(waypoint: Any) -> dict[str, Any]:
    transform = waypoint.transform
    location = transform.location
    rotation = transform.rotation
    return {
        "road_id": int(waypoint.road_id),
        "section_id": int(waypoint.section_id),
        "lane_id": int(waypoint.lane_id),
        "s_m": round(float(waypoint.s), 6),
        "x_m": round(float(location.x), 6),
        "y_m": round(float(location.y), 6),
        "z_m": round(float(location.z), 6),
        "yaw_deg": round(float(rotation.yaw), 6),
    }


def _lane_key(record: Mapping[str, Any]) -> tuple[int, int, int]:
    return int(record["road_id"]), int(record["section_id"]), int(record["lane_id"])


def _pose_key(record: Mapping[str, Any]) -> tuple[float, float, float]:
    return float(record["x_m"]), float(record["y_m"]), float(record["z_m"])


def _nearest_distances(candidates: np.ndarray, selected: np.ndarray) -> np.ndarray:
    if candidates.shape[0] == 0 or selected.shape[0] == 0:
        raise MapGenerationError("scan coverage cannot be computed for an empty waypoint set")
    try:
        from scipy.spatial import cKDTree
    except ImportError as error:
        raise MapGenerationError(f"scan planning requires scipy: {error}") from error
    return cKDTree(selected).query(candidates, k=1, workers=-1)[0]


def build_scan_plan(
    carla_map: Any,
    spacing_m: float,
    tile_size_m: float,
    z_band_m: float,
    sensor_height_m: float,
) -> dict[str, Any]:
    """Select deterministic tile poses while preserving every lane segment."""
    spacing_m = _finite_positive(spacing_m, "spacing_m")
    tile_size_m = _finite_positive(tile_size_m, "tile_size_m")
    z_band_m = _finite_positive(z_band_m, "z_band_m")
    sensor_height_m = _finite_positive(sensor_height_m, "sensor_height_m")
    waypoints = list(carla_map.generate_waypoints(spacing_m))
    records = sorted(
        (_waypoint_record(waypoint) for waypoint in waypoints),
        key=lambda item: (
            item["road_id"],
            item["section_id"],
            item["lane_id"],
            item["s_m"],
            item["x_m"],
            item["y_m"],
            item["z_m"],
        ),
    )
    if not records:
        raise MapGenerationError("CARLA generated no driving waypoints")

    tile_candidates: dict[tuple[int, int, int], list[dict[str, Any]]] = defaultdict(list)
    for record in records:
        tile = (
            math.floor(float(record["x_m"]) / tile_size_m),
            math.floor(float(record["y_m"]) / tile_size_m),
            math.floor(float(record["z_m"]) / z_band_m),
        )
        enriched = dict(record)
        enriched["tile"] = list(tile)
        tile_candidates[tile].append(enriched)

    selected: list[dict[str, Any]] = []
    selected_keys: set[tuple[float, float, float]] = set()
    for tile in sorted(tile_candidates):
        center_x = (tile[0] + 0.5) * tile_size_m
        center_y = (tile[1] + 0.5) * tile_size_m
        center_z = (tile[2] + 0.5) * z_band_m
        candidate = min(
            tile_candidates[tile],
            key=lambda item: (
                (float(item["x_m"]) - center_x) ** 2
                + (float(item["y_m"]) - center_y) ** 2
                + (float(item["z_m"]) - center_z) ** 2,
                _lane_key(item),
                float(item["s_m"]),
            ),
        )
        selected.append(candidate)
        selected_keys.add(_pose_key(candidate))

    # Tile deduplication may hide a short adjacent lane or a vertically stacked
    # road.  Add one median-s waypoint for every missing road/section/lane key.
    lanes: dict[tuple[int, int, int], list[dict[str, Any]]] = defaultdict(list)
    for record in records:
        lanes[_lane_key(record)].append(record)
    covered_lanes = {_lane_key(record) for record in selected}
    for lane_key in sorted(set(lanes) - covered_lanes):
        lane_records = lanes[lane_key]
        candidate = lane_records[len(lane_records) // 2]
        if _pose_key(candidate) not in selected_keys:
            tile = (
                math.floor(float(candidate["x_m"]) / tile_size_m),
                math.floor(float(candidate["y_m"]) / tile_size_m),
                math.floor(float(candidate["z_m"]) / z_band_m),
            )
            candidate = {**candidate, "tile": list(tile)}
            selected.append(candidate)
            selected_keys.add(_pose_key(candidate))

    # A physical location can represent multiple lane keys.  Preserve all of
    # that provenance while scanning the location only once.
    source_lanes_by_pose: dict[tuple[float, float, float], set[tuple[int, int, int]]] = (
        defaultdict(set)
    )
    for record in records:
        source_lanes_by_pose[_pose_key(record)].add(_lane_key(record))
    unique = {_pose_key(record): record for record in selected}

    def scan_order(record: Mapping[str, Any]) -> tuple[Any, ...]:
        tile_x, tile_y, tile_z = record["tile"]
        serpentine_x = tile_x if tile_y % 2 == 0 else -tile_x
        return tile_z, tile_y, serpentine_x, _lane_key(record), float(record["s_m"])

    ordered = sorted(unique.values(), key=scan_order)
    poses = []
    for scan_index, record in enumerate(ordered):
        pose_key = _pose_key(record)
        poses.append(
            {
                "scan_index": scan_index,
                "x_m": float(record["x_m"]),
                "y_m": float(record["y_m"]),
                "road_z_m": float(record["z_m"]),
                "sensor_z_m": round(float(record["z_m"]) + sensor_height_m, 6),
                "yaw_deg": float(record["yaw_deg"]),
                "tile": list(record["tile"]),
                "representative_lane": list(_lane_key(record)),
                "source_lanes_at_exact_pose": [
                    list(key) for key in sorted(source_lanes_by_pose[pose_key])
                ],
            }
        )

    candidate_xyz = np.asarray([_pose_key(record) for record in records], dtype=np.float64)
    selected_xyz = np.asarray(
        [(pose["x_m"], pose["y_m"], pose["road_z_m"]) for pose in poses],
        dtype=np.float64,
    )
    distances = _nearest_distances(candidate_xyz, selected_xyz)
    candidate_lane_keys = {_lane_key(record) for record in records}
    selected_lane_keys = {
        tuple(lane_key)
        for pose in poses
        for lane_key in pose["source_lanes_at_exact_pose"]
    }
    missing_lane_keys = sorted(candidate_lane_keys - selected_lane_keys)
    if missing_lane_keys:
        raise MapGenerationError(
            f"internal error: {len(missing_lane_keys)} lane segments have no scan pose"
        )
    return {
        "method": "CARLA generate_waypoints + XYZ tile representatives + every lane key",
        "parameters": {
            "waypoint_spacing_m": spacing_m,
            "tile_size_m": tile_size_m,
            "z_band_m": z_band_m,
            "sensor_height_m": sensor_height_m,
        },
        "coverage": {
            "candidate_waypoints": len(records),
            "scan_poses": len(poses),
            "road_count": len({record["road_id"] for record in records}),
            "lane_segment_count": len(candidate_lane_keys),
            "lane_segments_with_pose": len(selected_lane_keys),
            "nearest_pose_3d_m": {
                "median": float(np.median(distances)),
                "p95": float(np.percentile(distances, 95)),
                "max": float(np.max(distances)),
            },
        },
        "poses": poses,
    }


def transform_semantic_points_to_ros(
    records: np.ndarray,
    sensor_to_carla_world: Sequence[Sequence[float]],
    static_tags: Iterable[int] = DEFAULT_STATIC_TAGS,
) -> tuple[np.ndarray, dict[str, Any]]:
    if records.dtype.names is None or not {
        "x",
        "y",
        "z",
        "object_tag",
    }.issubset(records.dtype.names):
        raise MapGenerationError("semantic records have an incompatible dtype")
    matrix = np.asarray(sensor_to_carla_world, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise MapGenerationError("sensor transform must be a finite 4x4 matrix")
    source = np.column_stack((records["x"], records["y"], records["z"])).astype(
        np.float64, copy=False
    )
    finite = np.isfinite(source).all(axis=1)
    tag_values = records["object_tag"].astype(np.int64, copy=False)
    observed_tags = {int(value) for value in np.unique(tag_values)}
    audited_measurement_tags = set(EXPECTED_CITY_OBJECT_LABELS.values()) - {255}
    unexpected_tags = sorted(observed_tags - audited_measurement_tags)
    if unexpected_tags:
        raise MapGenerationError(
            f"semantic LiDAR returned unaudited object tags: {unexpected_tags}"
        )
    static_tag_set = frozenset(int(value) for value in static_tags)
    keep = finite & np.isin(tag_values, tuple(sorted(static_tag_set)))
    retained = source[keep]
    world = retained @ matrix[:3, :3].T + matrix[:3, 3]
    world[:, 1] *= -1.0  # CARLA/Unreal left-handed world -> ROS right-handed map.
    counts = Counter(int(value) for value in tag_values.tolist())
    stats = {
        "raw_points": int(records.shape[0]),
        "finite_points": int(np.count_nonzero(finite)),
        "retained_static_points": int(retained.shape[0]),
        "filtered_points": int(records.shape[0] - retained.shape[0]),
        "raw_tag_counts": {str(key): counts[key] for key in sorted(counts)},
        "retained_tags": {
            str(key): STATIC_TAG_NAMES.get(key, "explicit-user-static-tag")
            for key in sorted(static_tag_set)
        },
        "explicitly_excluded_dynamic_tags": {
            str(key): value for key, value in EXCLUDED_DYNAMIC_TAG_NAMES.items()
        },
    }
    return np.asarray(world, dtype=np.float32), stats


def voxel_centroids(points: np.ndarray, voxel_size: float) -> np.ndarray:
    voxel_size = _finite_positive(voxel_size, "voxel_size")
    values = np.asarray(points, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 3:
        raise MapGenerationError("point array must have shape (N, 3)")
    if values.shape[0] == 0:
        return np.empty((0, 3), dtype=np.float32)
    if not np.isfinite(values).all():
        raise MapGenerationError("point array contains non-finite coordinates")
    keys = np.floor(values / voxel_size).astype(np.int64)
    # Sorting coordinates inside each key makes floating-point reduction stable
    # even if checkpoint chunks are presented in a different order.
    order = np.lexsort(
        (values[:, 2], values[:, 1], values[:, 0], keys[:, 2], keys[:, 1], keys[:, 0])
    )
    keys = keys[order]
    values = values[order]
    starts = np.concatenate(([0], np.flatnonzero(np.any(keys[1:] != keys[:-1], axis=1)) + 1))
    counts = np.diff(np.concatenate((starts, [values.shape[0]]))).astype(np.float64)
    sums = np.add.reduceat(values, starts, axis=0)
    return np.asarray(sums / counts[:, None], dtype=np.float32)


def _pcd_header(point_count: int, encoding: str = "binary") -> bytes:
    if point_count <= 0:
        raise MapGenerationError("PCD must contain at least one point")
    if encoding != "binary":
        raise MapGenerationError("internal PCD writer supports binary payloads only")
    return (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {point_count}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {point_count}\n"
        "DATA binary\n"
    ).encode("ascii")


def write_binary_pcd(path: Path, points: np.ndarray) -> None:
    values = np.asarray(points, dtype="<f4")
    if values.ndim != 2 or values.shape[1] != 3 or values.shape[0] == 0:
        raise MapGenerationError("PCD point array must have non-empty shape (N, 3)")
    if not np.isfinite(values).all():
        raise MapGenerationError("PCD point array contains non-finite coordinates")
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(_pcd_header(int(values.shape[0])))
            stream.write(values.tobytes(order="C"))
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def inspect_pcd(path: Path) -> dict[str, Any]:
    header: dict[str, str] = {}
    data_offset = 0
    with path.open("rb") as stream:
        for _ in range(100):
            raw = stream.readline(4096)
            if not raw:
                break
            try:
                line = raw.decode("ascii").strip()
            except UnicodeDecodeError as error:
                raise MapGenerationError("PCD payload begins before DATA header") from error
            if not line or line.startswith("#"):
                continue
            key, _, value = line.partition(" ")
            normalized_key = key.upper()
            if normalized_key in header:
                raise MapGenerationError(f"PCD header contains duplicate {normalized_key}")
            header[normalized_key] = value.strip()
            if normalized_key == "DATA":
                data_offset = stream.tell()
                break
    missing = PCD_HEADER_KEYS - set(header)
    if missing:
        raise MapGenerationError(f"PCD header is missing {sorted(missing)}")
    if header["VERSION"] != "0.7":
        raise MapGenerationError("PCD VERSION must be 0.7")
    if header["FIELDS"].split() != ["x", "y", "z"]:
        raise MapGenerationError("PCD must contain exactly x y z fields")
    if header["SIZE"].split() != ["4", "4", "4"] or header["TYPE"].split() != [
        "F",
        "F",
        "F",
    ]:
        raise MapGenerationError("PCD x y z must be float32")
    if header["COUNT"].split() != ["1", "1", "1"]:
        raise MapGenerationError("PCD x y z COUNT must be 1 1 1")
    try:
        points = int(header["POINTS"])
        width = int(header["WIDTH"])
        height = int(header["HEIGHT"])
    except ValueError as error:
        raise MapGenerationError("PCD dimensions must be integers") from error
    if points <= 0 or width <= 0 or height <= 0 or width * height != points:
        raise MapGenerationError("PCD dimensions are invalid")
    encoding = header["DATA"].lower()
    size = path.stat().st_size
    if encoding == "binary" and size != data_offset + points * 12:
        raise MapGenerationError("binary PCD payload size is inconsistent")
    trailing_padding = 0
    if encoding == "binary_compressed":
        with path.open("rb") as stream:
            stream.seek(data_offset)
            preamble = stream.read(8)
            if len(preamble) != 8:
                raise MapGenerationError(
                    "binary_compressed PCD has a truncated size preamble"
                )
            compressed_size, uncompressed_size = struct.unpack("<II", preamble)
            if compressed_size <= 0 or uncompressed_size <= 0:
                raise MapGenerationError(
                    "binary_compressed PCD sizes must be positive"
                )
            payload_end = data_offset + 8 + compressed_size
            if size >= payload_end:
                stream.seek(payload_end)
                trailing = stream.read()
            else:
                trailing = b""
        if uncompressed_size != points * 12 or size < payload_end:
            raise MapGenerationError("binary_compressed PCD payload size is inconsistent")
        # PCL 1.12 pads compressed output to its file-buffer boundary.  Admit
        # only zero padding so unrelated/truncated bytes cannot masquerade as a
        # valid compressed payload.
        if any(trailing):
            raise MapGenerationError("binary_compressed PCD has non-zero trailing data")
        trailing_padding = len(trailing)
    if encoding not in {"binary", "binary_compressed"}:
        raise MapGenerationError(f"unsupported PCD encoding: {encoding}")
    return {
        "points": points,
        "encoding": encoding,
        "header_bytes": data_offset,
        "trailing_padding_bytes": trailing_padding,
        "size_bytes": size,
        "sha256": sha256_file(path),
    }


def _actor_record(actor: Any) -> dict[str, Any]:
    attributes = getattr(actor, "attributes", {})
    return {
        "id": int(getattr(actor, "id", -1)),
        "type_id": str(getattr(actor, "type_id", "")),
        "role_name": str(attributes.get("role_name", ""))
        if isinstance(attributes, Mapping)
        else "",
    }


def _actor_inventory_sha256(records: Sequence[Mapping[str, Any]], include_ids: bool) -> str:
    payload = [
        {
            **({"id": int(record["id"])} if include_ids else {}),
            "type_id": str(record["type_id"]),
            "role_name": str(record["role_name"]),
        }
        for record in records
    ]
    if not include_ids:
        payload.sort(key=lambda item: (item["type_id"], item["role_name"]))
    serialized = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return sha256_bytes(serialized)


def static_scene_actor_audit(
    actors: Iterable[Any],
    expected_baseline: Sequence[Mapping[str, Any]] | None = None,
    collector_sensor: Any | None = None,
) -> dict[str, Any]:
    """Reject runtime actors and pin the exact actor set during every scan.

    A cold packaged map is allowed to expose only its spectator and traffic
    infrastructure actors before collection.  In particular, ``static.prop``
    actors are not assumed safe: CARLA semantic tags describe mesh assets, not
    whether an actor was spawned at runtime.  After the collector sensor is
    created, every observed actor id/type/role must equal that baseline plus
    the one owned semantic-LiDAR sensor.
    """
    records = sorted(
        (_actor_record(actor) for actor in actors),
        key=lambda item: (item["id"], item["type_id"], item["role_name"]),
    )
    ids = [record["id"] for record in records]
    duplicate_ids = sorted({actor_id for actor_id in ids if ids.count(actor_id) > 1})
    blocking: list[dict[str, Any]] = []
    missing: list[dict[str, Any]] = []
    changed: list[dict[str, Any]] = []

    if expected_baseline is None:
        for record in records:
            if record["type_id"] != "spectator" and not record["type_id"].startswith(
                "traffic."
            ):
                blocking.append(record)
        expected_count = len(records)
        status = (
            "PASS_MAP_BASELINE_ONLY"
            if not blocking and not duplicate_ids
            else "FAIL_RUNTIME_ACTORS"
        )
    else:
        expected = [dict(record) for record in expected_baseline]
        if collector_sensor is not None:
            expected.append(_actor_record(collector_sensor))
        expected_by_id = {int(record["id"]): record for record in expected}
        observed_by_id = {int(record["id"]): record for record in records}
        blocking = [
            record
            for actor_id, record in observed_by_id.items()
            if actor_id not in expected_by_id
        ]
        missing = [
            record
            for actor_id, record in expected_by_id.items()
            if actor_id not in observed_by_id
        ]
        changed = [
            {
                "expected": expected_by_id[actor_id],
                "observed": observed_by_id[actor_id],
            }
            for actor_id in sorted(expected_by_id.keys() & observed_by_id.keys())
            if expected_by_id[actor_id] != observed_by_id[actor_id]
        ]
        expected_count = len(expected)
        status = (
            "PASS_PINNED_ACTOR_SET"
            if not blocking and not missing and not changed and not duplicate_ids
            else "FAIL_ACTOR_SET_CHANGED"
        )

    return {
        "status": status,
        "actor_count_inspected": len(records),
        "expected_actor_count": expected_count,
        "actor_set_sha256": _actor_inventory_sha256(records, include_ids=True),
        "actor_type_multiset_sha256": _actor_inventory_sha256(records, include_ids=False),
        "actor_inventory": records if expected_baseline is None else None,
        "blocking_actors": blocking,
        "missing_actors": missing,
        "changed_actors": changed,
        "duplicate_actor_ids": duplicate_ids,
    }


def _parse_static_tags(text: str) -> frozenset[int]:
    try:
        tags = frozenset(int(value.strip()) for value in text.split(",") if value.strip())
    except ValueError as error:
        raise MapGenerationError("static tags must be comma-separated integers") from error
    if not tags or any(value < 0 for value in tags):
        raise MapGenerationError("static tags must be non-empty non-negative integers")
    unsafe = tags - DEFAULT_STATIC_TAGS
    if unsafe:
        raise MapGenerationError(
            f"static tags include non-static/unknown CARLA 0.9.15 labels: {sorted(unsafe)}"
        )
    return tags


def validate_city_object_labels(carla: Any) -> dict[str, int]:
    label_type = getattr(carla, "CityObjectLabel", None)
    if label_type is None:
        raise MapGenerationError("CARLA Python API has no CityObjectLabel enum")
    actual = {}
    for name, expected_value in EXPECTED_CITY_OBJECT_LABELS.items():
        value = getattr(label_type, name, None)
        try:
            actual_value = int(value)
        except (TypeError, ValueError) as error:
            raise MapGenerationError(f"CARLA CityObjectLabel lacks {name}") from error
        actual[name] = actual_value
        if actual_value != expected_value:
            raise MapGenerationError(
                f"CARLA CityObjectLabel.{name}={actual_value}, expected {expected_value}"
            )
    return actual


def _canonical_map_leaf(name: str) -> str:
    return str(name).rstrip("/").split("/")[-1]


def verify_world_contract(
    world: Any, expected_map: str, expected_opendrive_sha256: str
) -> tuple[Any, str]:
    carla_map = world.get_map()
    actual_name = str(carla_map.name)
    if _canonical_map_leaf(actual_name) != _canonical_map_leaf(expected_map):
        raise MapGenerationError(
            f"running map mismatch: expected {expected_map!r}, got {actual_name!r}; "
            "the collector refuses to call load_world"
        )
    opendrive = carla_map.to_opendrive().encode("utf-8")
    actual_digest = sha256_bytes(opendrive)
    if actual_digest != expected_opendrive_sha256:
        raise MapGenerationError(
            "running OpenDRIVE differs from the pinned digest: "
            f"expected={expected_opendrive_sha256}, actual={actual_digest}"
        )
    return carla_map, actual_digest


def validate_carla_version_pair(client_version: str, server_version: str) -> None:
    def is_0915(value: str) -> bool:
        return value == EXPECTED_CARLA_VERSION or value.startswith(
            (f"{EXPECTED_CARLA_VERSION}-", f"{EXPECTED_CARLA_VERSION}+")
        )

    if not is_0915(client_version) or not is_0915(server_version):
        raise MapGenerationError(
            "semantic-LiDAR collection requires a CARLA 0.9.15 client and server: "
            f"client={client_version!r}, server={server_version!r}"
        )
    if client_version != server_version:
        raise MapGenerationError(
            "CARLA client/server version strings differ: "
            f"client={client_version!r}, server={server_version!r}"
        )


def carla_runtime_binary_inventory(carla: Any, content_root: Path) -> dict[str, Any]:
    origin = str(getattr(carla, "__file__", ""))
    marker = ".egg/"
    if marker not in origin:
        raise MapGenerationError(
            "CARLA Python API is not loaded from a hashable packaged .egg distribution"
        )
    python_archive = Path(origin.split(marker, 1)[0] + ".egg").resolve()
    server_binary = (
        content_root.expanduser().resolve().parent
        / "Binaries/Linux/CarlaUE4-Linux-Shipping"
    )
    records = {}
    for label, path in (
        ("python_api_distribution", python_archive),
        ("server_executable", server_binary),
    ):
        if not path.is_file():
            raise MapGenerationError(f"CARLA {label} is absent: {path}")
        records[label] = {
            "path": str(path),
            "size_bytes": path.stat().st_size,
            "sha256": sha256_file(path),
        }
    return records


def _listener_pids(port: int) -> list[int]:
    socket_inodes: set[str] = set()
    expected_port = f"{port:04X}"
    for table in (Path("/proc/net/tcp"), Path("/proc/net/tcp6")):
        try:
            lines = table.read_text(encoding="ascii").splitlines()[1:]
        except OSError as error:
            raise MapGenerationError(f"cannot inspect local TCP listeners: {error}") from error
        for line in lines:
            fields = line.split()
            if (
                len(fields) > 9
                and fields[1].rsplit(":", 1)[-1].upper() == expected_port
                and fields[3] == "0A"
            ):
                socket_inodes.add(fields[9])
    if not socket_inodes:
        raise MapGenerationError(f"no local TCP listener owns CARLA port {port}")
    pids: set[int] = set()
    for process_dir in Path("/proc").iterdir():
        if not process_dir.name.isdigit():
            continue
        try:
            descriptors = list((process_dir / "fd").iterdir())
        except (FileNotFoundError, PermissionError):
            continue
        for descriptor in descriptors:
            try:
                target = os.readlink(descriptor)
            except (FileNotFoundError, PermissionError, OSError):
                continue
            if target.startswith("socket:[") and target[8:-1] in socket_inodes:
                pids.add(int(process_dir.name))
                break
    if not pids:
        raise MapGenerationError(
            f"cannot resolve the process that owns local CARLA port {port}"
        )
    return sorted(pids)


def _process_start_time_ticks(pid: int) -> int:
    try:
        stat = (Path("/proc") / str(pid) / "stat").read_text(encoding="ascii")
        fields_after_name = stat[stat.rindex(")") + 2 :].split()
        return int(fields_after_name[19])
    except (OSError, ValueError, IndexError) as error:
        raise MapGenerationError(f"cannot inspect CARLA listener process {pid}") from error


def attest_local_carla_server(
    host: str, port: int, server_binary_record: Mapping[str, Any]
) -> dict[str, Any]:
    if host not in {"127.0.0.1", "localhost", "::1"}:
        raise MapGenerationError(
            "audited semantic-LiDAR collection is restricted to a local CARLA server"
        )
    pids = _listener_pids(port)
    if len(pids) != 1:
        raise MapGenerationError(f"CARLA port {port} has ambiguous listener PIDs: {pids}")
    pid = pids[0]
    process_dir = Path("/proc") / str(pid)
    if process_dir.stat().st_uid != os.getuid():
        raise MapGenerationError("CARLA listener is not owned by the collector user")
    executable = Path(os.readlink(process_dir / "exe")).resolve()
    expected_executable = Path(server_binary_record["path"]).resolve()
    if executable != expected_executable:
        raise MapGenerationError(
            "CARLA port listener is not the pinned packaged server executable: "
            f"{executable} != {expected_executable}"
        )
    if (
        executable.stat().st_size != server_binary_record["size_bytes"]
        or sha256_file(executable) != server_binary_record["sha256"]
    ):
        raise MapGenerationError("live CARLA server executable differs from its binary pin")
    try:
        command_line = [
            value.decode("utf-8", errors="surrogateescape")
            for value in (process_dir / "cmdline").read_bytes().split(b"\0")
            if value
        ]
    except OSError as error:
        raise MapGenerationError("cannot read live CARLA server command line") from error
    if not any(
        value in {f"-carla-port={port}", f"-carla-rpc-port={port}"}
        for value in command_line
    ):
        raise MapGenerationError("live CARLA server command line does not pin the RPC port")
    return {
        "scope": "local /proc TCP-listener-to-executable attestation",
        "host": host,
        "port": port,
        "pid": pid,
        "process_start_time_ticks": _process_start_time_ticks(pid),
        "executable_path": str(executable),
        "executable_size_bytes": executable.stat().st_size,
        "executable_sha256": sha256_file(executable),
        "command_line": command_line,
        "command_line_sha256": sha256_bytes(b"\0".join(value.encode() for value in command_line)),
    }


def verify_live_server_attestation(
    attestation: Mapping[str, Any], *, full_hash: bool = False
) -> None:
    pid = int(attestation["pid"])
    process_dir = Path("/proc") / str(pid)
    try:
        executable = Path(os.readlink(process_dir / "exe")).resolve()
    except OSError as error:
        raise MapGenerationError("attested CARLA server process is no longer live") from error
    try:
        differs = (
            _process_start_time_ticks(pid) != attestation["process_start_time_ticks"]
            or str(executable) != attestation["executable_path"]
            or executable.stat().st_size != attestation["executable_size_bytes"]
            or (
                full_hash
                and sha256_file(executable) != attestation["executable_sha256"]
            )
        )
    except OSError as error:
        raise MapGenerationError(
            "cannot re-verify the live CARLA server process"
        ) from error
    if differs:
        raise MapGenerationError(
            "live CARLA server process differs from its session attestation"
        )


def _settings_snapshot(settings: Any) -> dict[str, Any]:
    fields = (
        "synchronous_mode",
        "no_rendering_mode",
        "fixed_delta_seconds",
        "substepping",
        "max_substep_delta_time",
        "max_substeps",
        "max_culling_distance",
        "deterministic_ragdolls",
        "tile_stream_distance",
        "actor_active_distance",
        "spectator_as_ego",
    )
    return {field: getattr(settings, field) for field in fields if hasattr(settings, field)}


def _settings_snapshots_match(
    expected: Mapping[str, Any], actual: Mapping[str, Any]
) -> bool:
    if set(expected) != set(actual):
        return False
    for key, expected_value in expected.items():
        actual_value = actual[key]
        if isinstance(expected_value, float) or isinstance(actual_value, float):
            if expected_value is None or actual_value is None:
                if expected_value is not actual_value:
                    return False
            elif not math.isclose(
                float(expected_value), float(actual_value), rel_tol=0.0, abs_tol=1e-9
            ):
                return False
        elif expected_value != actual_value:
            return False
    return True


def _measurement_matrix(measurement: Any) -> np.ndarray:
    transform = getattr(measurement, "transform", None)
    if transform is None:
        raise MapGenerationError("semantic LiDAR measurement has no source transform")
    matrix = np.asarray(transform.get_matrix(), dtype=np.float64)
    if matrix.shape != (4, 4):
        raise MapGenerationError("CARLA returned an invalid semantic LiDAR transform")
    return matrix


def transform_matrix_errors(
    actual: Sequence[Sequence[float]], expected: Sequence[Sequence[float]]
) -> tuple[float, float]:
    actual_matrix = np.asarray(actual, dtype=np.float64)
    expected_matrix = np.asarray(expected, dtype=np.float64)
    if (
        actual_matrix.shape != (4, 4)
        or expected_matrix.shape != (4, 4)
        or not np.isfinite(actual_matrix).all()
        or not np.isfinite(expected_matrix).all()
    ):
        raise MapGenerationError("source and expected transforms must be finite 4x4 matrices")
    translation_error = float(
        np.linalg.norm(actual_matrix[:3, 3] - expected_matrix[:3, 3])
    )
    relative_rotation = expected_matrix[:3, :3].T @ actual_matrix[:3, :3]
    cosine = float(np.clip((np.trace(relative_rotation) - 1.0) / 2.0, -1.0, 1.0))
    rotation_error = math.acos(cosine)
    return translation_error, rotation_error


def _wait_for_frame(queue: Queue[Any], expected_frame: int, timeout: float) -> Any:
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise MapGenerationError(
                f"semantic LiDAR timed out waiting for frame {expected_frame}"
            )
        try:
            measurement = queue.get(timeout=remaining)
        except Empty as error:
            raise MapGenerationError(
                f"semantic LiDAR timed out waiting for frame {expected_frame}"
            ) from error
        frame = int(measurement.frame)
        if frame == expected_frame:
            return measurement
        if frame > expected_frame:
            raise MapGenerationError(
                f"semantic LiDAR skipped frame {expected_frame} and returned {frame}"
            )


def capture_transform_settled_measurement(
    world: Any,
    queue: Queue[Any],
    expected_matrix: Sequence[Sequence[float]],
    timeout: float,
    max_extra_frames: int,
    frame_audit: Callable[[], Mapping[str, Any]] | None = None,
) -> tuple[Any, np.ndarray, dict[str, Any]]:
    """Tick until the measurement itself reports the commanded world transform.

    CARLA may occasionally publish one semantic-LiDAR frame at the actor's
    previous transform immediately after ``set_transform``.  Such a frame must
    never be accumulated.  The original 5 cm / 0.1 degree contract remains
    unchanged; this helper discards only explicitly audited stale frames and
    fails after a bounded number of additional synchronous ticks.
    """
    if isinstance(max_extra_frames, bool) or not isinstance(max_extra_frames, int):
        raise MapGenerationError("max_extra_frames must be an integer")
    if max_extra_frames < 0:
        raise MapGenerationError("max_extra_frames must be non-negative")
    attempts: list[dict[str, Any]] = []
    for attempt_index in range(max_extra_frames + 1):
        frame = int(world.tick(timeout))
        measurement = _wait_for_frame(queue, frame, timeout)
        matrix = _measurement_matrix(measurement)
        translation_error, rotation_error = transform_matrix_errors(
            matrix, expected_matrix
        )
        audit = dict(frame_audit()) if frame_audit is not None else None
        attempt = {
            "attempt_index": attempt_index,
            "carla_frame": frame,
            "source_translation_error_m": translation_error,
            "source_rotation_error_rad": rotation_error,
            "actor_and_server_audit": audit,
            "accepted": (
                translation_error <= MAX_SOURCE_TRANSLATION_ERROR_M
                and rotation_error <= MAX_SOURCE_ROTATION_ERROR_RAD
            ),
        }
        attempts.append(attempt)
        if attempt["accepted"]:
            return measurement, matrix, {
                "status": "PASS_MEASUREMENT_TRANSFORM_SETTLED",
                "max_extra_frames": max_extra_frames,
                "capture_attempt_count": len(attempts),
                "discarded_stale_frame_count": len(attempts) - 1,
                "attempts": attempts,
            }
    summary = ", ".join(
        f"frame {attempt['carla_frame']}: "
        f"{attempt['source_translation_error_m']:.3f} m/"
        f"{math.degrees(attempt['source_rotation_error_rad']):.3f} deg"
        for attempt in attempts
    )
    raise MapGenerationError(
        "semantic LiDAR measurement transform did not settle within "
        f"{max_extra_frames} extra frame(s); {summary}"
    )


def _carla_transform(carla: Any, pose: Mapping[str, Any]) -> Any:
    return carla.Transform(
        carla.Location(x=pose["x_m"], y=pose["y_m"], z=pose["sensor_z_m"]),
        carla.Rotation(pitch=0.0, yaw=pose["yaw_deg"], roll=0.0),
    )


def _configure_blueprint(blueprint: Any, attributes: Mapping[str, str]) -> None:
    for key, value in attributes.items():
        if hasattr(blueprint, "has_attribute") and not blueprint.has_attribute(key):
            raise MapGenerationError(f"CARLA semantic LiDAR lacks required attribute {key}")
        blueprint.set_attribute(key, str(value))


def _checkpoint_tag_count_mapping(raw_counts: Any, label: str) -> dict[str, int]:
    if not isinstance(raw_counts, dict):
        raise MapGenerationError(f"{label} is invalid")
    validated: dict[str, int] = {}
    for raw_tag, raw_count in raw_counts.items():
        if not isinstance(raw_tag, str):
            raise MapGenerationError(f"{label} has a non-string tag key")
        try:
            tag = int(raw_tag)
        except ValueError as error:
            raise MapGenerationError(f"{label} has a non-integer tag key") from error
        if raw_tag != str(tag) or tag not in range(29):
            raise MapGenerationError(
                f"{label} has an unaudited CARLA 0.9.15 tag: {raw_tag!r}"
            )
        if type(raw_count) is not int or raw_count < 0:
            raise MapGenerationError(
                f"{label}[{raw_tag}] must be a non-negative integer"
            )
        validated[raw_tag] = raw_count
    return validated


def _checkpoint_retained_tags(plan: Mapping[str, Any]) -> frozenset[int]:
    contract = plan.get("semantic_contract")
    raw_tags = contract.get("retained_static_tags") if isinstance(contract, Mapping) else None
    if not isinstance(raw_tags, Mapping) or not raw_tags:
        raise MapGenerationError("scan plan lacks retained static semantic tags")
    tags: set[int] = set()
    for raw_tag in raw_tags:
        if not isinstance(raw_tag, str):
            raise MapGenerationError("scan plan retained tag key is not a string")
        try:
            tag = int(raw_tag)
        except ValueError as error:
            raise MapGenerationError("scan plan retained tag key is not an integer") from error
        if raw_tag != str(tag) or tag not in DEFAULT_STATIC_TAGS:
            raise MapGenerationError(
                f"scan plan has a non-static/unaudited retained tag: {raw_tag!r}"
            )
        tags.add(tag)
    return frozenset(tags)


def _checkpoint_sha256(value: Any, label: str) -> str:
    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(character not in "0123456789abcdef" for character in value)
    ):
        raise MapGenerationError(f"{label} is not a lowercase SHA-256 digest")
    return value


def _load_checkpoint(path: Path, plan: Mapping[str, Any], chunks_dir: Path) -> dict[str, Any]:
    if not path.is_file():
        checkpoint = {
            "schema_version": 1,
            "plan_fingerprint": plan["fingerprint"],
            "status": "COLLECTING",
            "completed_scans": [],
            "tag_totals": {},
            "sessions": [],
            "updated_at_utc": utc_now(),
        }
    else:
        try:
            checkpoint = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError) as error:
            raise MapGenerationError(f"cannot read checkpoint {path}: {error}") from error
    if type(checkpoint.get("schema_version")) is not int or checkpoint["schema_version"] != 1:
        raise MapGenerationError("checkpoint schema_version must be 1")
    if checkpoint.get("plan_fingerprint") != plan["fingerprint"]:
        raise MapGenerationError("checkpoint belongs to a different scan plan")
    completed = checkpoint.get("completed_scans")
    if not isinstance(completed, list):
        raise MapGenerationError("checkpoint completed_scans is invalid")
    poses = plan.get("scan_plan", {}).get("poses")
    if not isinstance(poses, list) or len(completed) > len(poses):
        raise MapGenerationError("checkpoint contains more scans than the scan plan")
    if checkpoint.get("status") not in {"COLLECTING", "COLLECTED"}:
        raise MapGenerationError("checkpoint status is invalid")
    sessions = checkpoint.get("sessions", [])
    if not isinstance(sessions, list):
        raise MapGenerationError("checkpoint sessions is invalid")
    if completed and not sessions:
        raise MapGenerationError("checkpoint scans have no audited collection session")
    if completed or sessions:
        _checkpoint_sha256(
            checkpoint.get("baseline_actor_type_multiset_sha256"),
            "checkpoint actor baseline pin",
        )
    session_scan_cursor = 0
    for expected_index, session in enumerate(sessions):
        if (
            not isinstance(session, dict)
            or type(session.get("session_index")) is not int
            or session["session_index"] != expected_index
            or session.get("world_settings_restored") is not True
            or session.get("server_attestation_final_verified") is not True
            or session.get("cleanup_errors") != []
            or session.get("outcome") not in {"PASS", "INTERRUPTED", "FAIL"}
        ):
            raise MapGenerationError(
                "checkpoint has an unfinalized or cleanup-tainted collection session; "
                "it is not safe to resume or certify these chunks"
            )
        starting_scans = session.get("starting_completed_scans")
        ending_scans = session.get("ending_completed_scans")
        if (
            type(starting_scans) is not int
            or type(ending_scans) is not int
            or starting_scans != session_scan_cursor
            or ending_scans < starting_scans
            or ending_scans > len(completed)
        ):
            raise MapGenerationError(
                "checkpoint collection sessions do not form a contiguous scan prefix"
            )
        session_scan_cursor = ending_scans
    if session_scan_cursor != len(completed):
        raise MapGenerationError(
            "checkpoint collection sessions do not cover the completed scan prefix"
        )
    retained_tags = _checkpoint_retained_tags(plan) if completed else frozenset()
    expected_chunks = set()
    for expected_index, item in enumerate(completed):
        if not isinstance(item, dict) or item.get("scan_index") != expected_index:
            raise MapGenerationError("checkpoint scans are not a contiguous prefix")
        expected_name = f"scan_{expected_index:06d}.npy"
        if item.get("chunk_file") != expected_name:
            raise MapGenerationError("checkpoint chunk name differs from its scan index")
        voxel_points = item.get("voxel_points")
        raw_points = item.get("raw_points")
        retained_static_points = item.get("retained_static_points")
        if any(
            type(value) is not int or value <= 0
            for value in (voxel_points, raw_points, retained_static_points)
        ):
            raise MapGenerationError("checkpoint scan counters are invalid")
        raw_tag_counts = _checkpoint_tag_count_mapping(
            item.get("raw_tag_counts"), "checkpoint scan raw_tag_counts"
        )
        if sum(raw_tag_counts.values()) != raw_points:
            raise MapGenerationError(
                "checkpoint scan raw_tag_counts does not sum to raw_points"
            )
        expected_retained = sum(
            count for tag, count in raw_tag_counts.items() if int(tag) in retained_tags
        )
        if retained_static_points != expected_retained:
            raise MapGenerationError(
                "checkpoint retained_static_points differs from retained tag counts"
            )
        if voxel_points > retained_static_points:
            raise MapGenerationError(
                "checkpoint voxel_points exceeds retained_static_points"
            )
        carla_frame = item.get("carla_frame")
        if type(carla_frame) is not int or carla_frame < 0:
            raise MapGenerationError("checkpoint CARLA frame is invalid")
        translation_error = item.get("source_translation_error_m")
        rotation_error = item.get("source_rotation_error_rad")
        if (
            isinstance(translation_error, bool)
            or not isinstance(translation_error, (int, float))
            or not math.isfinite(float(translation_error))
            or float(translation_error) < 0.0
            or float(translation_error) > MAX_SOURCE_TRANSLATION_ERROR_M
            or isinstance(rotation_error, bool)
            or not isinstance(rotation_error, (int, float))
            or not math.isfinite(float(rotation_error))
            or float(rotation_error) < 0.0
            or float(rotation_error) > MAX_SOURCE_ROTATION_ERROR_RAD
        ):
            raise MapGenerationError("checkpoint source transform errors are invalid")
        sensor_contract = plan.get("sensor")
        settle_limit = (
            sensor_contract.get("transform_settle_max_extra_frames")
            if isinstance(sensor_contract, Mapping)
            else None
        )
        if settle_limit is not None:
            readback_translation = item.get("commanded_transform_readback_error_m")
            readback_rotation = item.get(
                "commanded_transform_readback_rotation_error_rad"
            )
            if any(
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                or float(value) < 0.0
                for value in (readback_translation, readback_rotation)
            ):
                raise MapGenerationError(
                    "checkpoint commanded transform readback errors are invalid"
                )
            settle = item.get("transform_settle")
            attempts = settle.get("attempts") if isinstance(settle, dict) else None
            recorded_settle_limit = (
                settle.get("max_extra_frames")
                if isinstance(settle, dict)
                else None
            )
            capture_attempt_count = (
                settle.get("capture_attempt_count")
                if isinstance(settle, dict)
                else None
            )
            discarded_stale_frame_count = (
                settle.get("discarded_stale_frame_count")
                if isinstance(settle, dict)
                else None
            )
            if (
                type(settle_limit) is not int
                or not isinstance(settle, dict)
                or settle.get("status") != "PASS_MEASUREMENT_TRANSFORM_SETTLED"
                or type(recorded_settle_limit) is not int
                or recorded_settle_limit != settle_limit
                or not isinstance(attempts, list)
                or not 1 <= len(attempts) <= settle_limit + 1
                or type(capture_attempt_count) is not int
                or capture_attempt_count != len(attempts)
                or type(discarded_stale_frame_count) is not int
                or discarded_stale_frame_count != len(attempts) - 1
            ):
                raise MapGenerationError("checkpoint transform-settle record is invalid")
            attempt_frames: list[int] = []
            for attempt_index, attempt in enumerate(attempts):
                attempt_translation = (
                    attempt.get("source_translation_error_m")
                    if isinstance(attempt, dict)
                    else None
                )
                attempt_rotation = (
                    attempt.get("source_rotation_error_rad")
                    if isinstance(attempt, dict)
                    else None
                )
                attempt_frame = attempt.get("carla_frame") if isinstance(attempt, dict) else None
                frame_audit = (
                    attempt.get("actor_and_server_audit")
                    if isinstance(attempt, dict)
                    else None
                )
                frame_actor_audit = (
                    frame_audit.get("actor_audit")
                    if isinstance(frame_audit, dict)
                    else None
                )
                within_contract = (
                    isinstance(attempt_translation, (int, float))
                    and not isinstance(attempt_translation, bool)
                    and math.isfinite(float(attempt_translation))
                    and 0.0
                    <= float(attempt_translation)
                    <= MAX_SOURCE_TRANSLATION_ERROR_M
                    and isinstance(attempt_rotation, (int, float))
                    and not isinstance(attempt_rotation, bool)
                    and math.isfinite(float(attempt_rotation))
                    and 0.0
                    <= float(attempt_rotation)
                    <= MAX_SOURCE_ROTATION_ERROR_RAD
                )
                if (
                    not isinstance(attempt, dict)
                    or type(attempt.get("attempt_index")) is not int
                    or attempt.get("attempt_index") != attempt_index
                    or type(attempt_frame) is not int
                    or attempt_frame < 0
                    or attempt.get("accepted") is not within_contract
                    or (attempt_index < len(attempts) - 1 and within_contract)
                    or not isinstance(frame_audit, dict)
                    or frame_audit.get("server_attestation_status") != "PASS"
                    or not isinstance(frame_actor_audit, dict)
                    or frame_actor_audit.get("status") != "PASS_PINNED_ACTOR_SET"
                ):
                    raise MapGenerationError(
                        "checkpoint transform-settle attempt is invalid"
                    )
                attempt_frames.append(attempt_frame)
            if (
                attempt_frames
                != list(range(attempt_frames[0], attempt_frames[0] + len(attempt_frames)))
                or attempt_frames[-1] != carla_frame
                or attempts[-1].get("accepted") is not True
                or not math.isclose(
                    float(attempts[-1]["source_translation_error_m"]),
                    float(translation_error),
                    rel_tol=0.0,
                    abs_tol=1e-12,
                )
                or not math.isclose(
                    float(attempts[-1]["source_rotation_error_rad"]),
                    float(rotation_error),
                    rel_tol=0.0,
                    abs_tol=1e-12,
                )
            ):
                raise MapGenerationError(
                    "checkpoint final transform-settle measurement is invalid"
                )
        actor_audit = item.get("actor_audit")
        if not isinstance(actor_audit, dict) or any(
            not isinstance(actor_audit.get(stage), dict)
            or actor_audit[stage].get("status") != "PASS_PINNED_ACTOR_SET"
            for stage in ("before", "after")
        ):
            raise MapGenerationError(
                "checkpoint scan lacks passing before/after pinned-actor-set audits"
            )
        digest = _checkpoint_sha256(item.get("sha256"), "checkpoint chunk digest")
        chunk = chunks_dir / expected_name
        expected_chunks.add(chunk.name)
        if not chunk.is_file() or sha256_file(chunk) != digest:
            raise MapGenerationError(f"checkpoint chunk is absent or corrupt: {chunk}")
        points = np.load(chunk, mmap_mode="r", allow_pickle=False)
        if points.shape != (item.get("voxel_points"), 3) or points.dtype != np.dtype("<f4"):
            raise MapGenerationError(f"checkpoint chunk has an invalid array contract: {chunk}")
    recorded_tag_totals = _checkpoint_tag_count_mapping(
        checkpoint.get("tag_totals"), "checkpoint tag_totals"
    )
    if recorded_tag_totals != _checkpoint_tag_totals(completed):
        raise MapGenerationError("checkpoint tag_totals differs from completed scans")
    if checkpoint.get("status") == "COLLECTED" and len(completed) != len(poses):
        raise MapGenerationError("checkpoint claims COLLECTED before all scan poses")
    if checkpoint.get("status") == "COLLECTED":
        validity = checkpoint.get("collection_validity")
        if (
            not isinstance(validity, dict)
            or validity.get("status") != "PASS"
            or not sessions
            or any(session.get("world_settings_restored") is not True for session in sessions)
            or any(
                session.get("server_attestation_final_verified") is not True
                for session in sessions
            )
            or any(session.get("cleanup_errors") != [] for session in sessions)
        ):
            raise MapGenerationError("COLLECTED checkpoint lacks valid audited sessions")
    unexpected = [
        candidate
        for candidate in sorted(chunks_dir.glob("scan_*.npy"))
        if candidate.name not in expected_chunks
    ]
    unexpected.extend(sorted(chunks_dir.glob(".scan_*.tmp")))
    if unexpected:
        quarantine = chunks_dir / "orphaned"
        quarantine.mkdir(exist_ok=True)
        recovered = checkpoint.setdefault("recovered_orphans", [])
        for source in unexpected:
            digest = sha256_file(source)
            destination = quarantine / f"{source.name.lstrip('.')}.{digest[:12]}.orphan"
            suffix = 1
            while destination.exists():
                destination = quarantine / (
                    f"{source.name.lstrip('.')}.{digest[:12]}.{suffix}.orphan"
                )
                suffix += 1
            os.replace(source, destination)
            recovered.append(
                {
                    "original_name": source.name,
                    "quarantined_path": str(destination),
                    "sha256": digest,
                    "reason": "chunk rename completed without a committed checkpoint",
                    "recovered_at_utc": utc_now(),
                }
            )
        checkpoint["updated_at_utc"] = utc_now()
        atomic_write_json(path, checkpoint)
    return checkpoint


def _checkpoint_tag_totals(completed: Sequence[Mapping[str, Any]]) -> dict[str, int]:
    totals: Counter[str] = Counter()
    for item in completed:
        for tag, count in item.get("raw_tag_counts", {}).items():
            totals[str(tag)] += int(count)
    return dict(sorted(totals.items(), key=lambda item: int(item[0])))


def collect_scans(
    carla: Any,
    world: Any,
    plan: Mapping[str, Any],
    output_root: Path,
    args: argparse.Namespace,
    stop_requested: Any,
    server_attestation: Mapping[str, Any],
) -> tuple[dict[str, Any], dict[str, Any]]:
    chunks_dir = output_root / "chunks"
    chunks_dir.mkdir(parents=True, exist_ok=True)
    checkpoint_path = output_root / "checkpoint.json"
    checkpoint = _load_checkpoint(checkpoint_path, plan, chunks_dir)
    verify_live_server_attestation(server_attestation)
    initial_audit = static_scene_actor_audit(world.get_actors())
    if initial_audit["status"] != "PASS_MAP_BASELINE_ONLY":
        raise MapGenerationError(
            "the cold world contains a runtime/non-map actor; only spectator and "
            "traffic infrastructure are allowed before collection: "
            f"{initial_audit['blocking_actors'][:5]}"
        )
    baseline_actors = initial_audit["actor_inventory"]
    baseline_signature = initial_audit["actor_type_multiset_sha256"]
    recorded_baseline = checkpoint.get("baseline_actor_type_multiset_sha256")
    if checkpoint.get("sessions") and recorded_baseline is None:
        raise MapGenerationError("checkpoint has no cross-session baseline actor pin")
    if recorded_baseline is not None and recorded_baseline != baseline_signature:
        raise MapGenerationError(
            "cold-world actor baseline differs from earlier collection sessions"
        )
    poses = plan["scan_plan"]["poses"]
    if len(checkpoint["completed_scans"]) == len(poses):
        validity = checkpoint.get("collection_validity")
        if (
            checkpoint.get("status") != "COLLECTED"
            or not isinstance(validity, dict)
            or validity.get("status") != "PASS"
        ):
            raise MapGenerationError(
                "all chunks exist but collection validity was not committed; "
                "audit the last session before retrying"
            )
        sessions = checkpoint.get("sessions", [])
        return checkpoint, {
            "resume_no_collection_required": True,
            "sessions": sessions,
            "session_count": len(sessions),
            "total_wall_seconds": sum(
                float(session.get("wall_seconds", 0.0)) for session in sessions
            ),
            "collection_validity": validity,
        }

    original_settings = world.get_settings()
    original_snapshot = _settings_snapshot(original_settings)
    applied_settings = world.get_settings()
    applied_settings.synchronous_mode = True
    applied_settings.fixed_delta_seconds = args.fixed_delta_seconds
    if hasattr(applied_settings, "substepping"):
        applied_settings.substepping = True
        applied_settings.max_substep_delta_time = 0.01
        applied_settings.max_substeps = max(
            5, int(math.ceil(args.fixed_delta_seconds / 0.01))
        )
    if hasattr(applied_settings, "deterministic_ragdolls"):
        applied_settings.deterministic_ragdolls = True
    queue: Queue[Any] = Queue()
    sensor = None
    restored = False
    applied_readback: dict[str, Any] | None = None
    restored_readback: dict[str, Any] | None = None
    start = time.monotonic()
    failure: BaseException | None = None
    cleanup_errors: list[str] = []
    sessions = checkpoint.setdefault("sessions", [])
    session = {
        "session_index": len(sessions),
        "started_at_utc": utc_now(),
        "starting_completed_scans": len(checkpoint["completed_scans"]),
        "initial_actor_audit": initial_audit,
        "original_world_settings": original_snapshot,
        "local_server_attestation": dict(server_attestation),
    }
    sessions.append(session)
    checkpoint.update(
        {
            "status": "COLLECTING",
            "collection_validity": {"status": "PENDING"},
            "baseline_actor_type_multiset_sha256": baseline_signature,
            "sessions": sessions,
            "updated_at_utc": utc_now(),
        }
    )
    atomic_write_json(checkpoint_path, checkpoint)
    blueprint_attributes = {
        "channels": str(args.channels),
        "range": str(args.range_m),
        "points_per_second": str(args.points_per_second),
        "rotation_frequency": str(1.0 / args.fixed_delta_seconds),
        "upper_fov": str(args.upper_fov_deg),
        "lower_fov": str(args.lower_fov_deg),
        "horizontal_fov": "360.0",
        "sensor_tick": "0.0",
    }
    try:
        world.apply_settings(applied_settings)
        applied_readback = _settings_snapshot(world.get_settings())
        if not _settings_snapshots_match(
            _settings_snapshot(applied_settings), applied_readback
        ):
            raise MapGenerationError(
                "CARLA synchronous settings readback differs from the requested settings"
            )
        blueprint = world.get_blueprint_library().find("sensor.lidar.ray_cast_semantic")
        _configure_blueprint(blueprint, blueprint_attributes)
        sensor = world.spawn_actor(blueprint, _carla_transform(carla, poses[0]))
        sensor.listen(queue.put)
        for _ in range(args.warmup_frames):
            frame = int(world.tick(args.timeout))
            _wait_for_frame(queue, frame, args.timeout)

        completed = checkpoint["completed_scans"]
        for pose in poses[len(completed) :]:
            if stop_requested():
                raise InterruptedGeneration("signal received; checkpoint is safe to resume")
            verify_live_server_attestation(server_attestation)
            pre_scan_audit = static_scene_actor_audit(
                world.get_actors(), baseline_actors, sensor
            )
            if pre_scan_audit["status"] != "PASS_PINNED_ACTOR_SET":
                raise MapGenerationError(
                    f"actor set changed before scan {pose['scan_index']}"
                )
            planned_transform = _carla_transform(carla, pose)
            expected_matrix = np.asarray(
                planned_transform.get_matrix(), dtype=np.float64
            )
            sensor.set_transform(planned_transform)
            commanded_readback = np.asarray(
                sensor.get_transform().get_matrix(), dtype=np.float64
            )
            readback_translation_error, readback_rotation_error = (
                transform_matrix_errors(commanded_readback, expected_matrix)
            )

            def audit_capture_frame() -> dict[str, Any]:
                actor_audit = static_scene_actor_audit(
                    world.get_actors(), baseline_actors, sensor
                )
                verify_live_server_attestation(server_attestation)
                if actor_audit["status"] != "PASS_PINNED_ACTOR_SET":
                    raise MapGenerationError(
                        f"actor set changed while settling scan {pose['scan_index']}"
                    )
                return {
                    "actor_audit": actor_audit,
                    "server_attestation_status": "PASS",
                }

            measurement, matrix, transform_settle = (
                capture_transform_settled_measurement(
                    world,
                    queue,
                    expected_matrix,
                    args.timeout,
                    args.transform_settle_max_extra_frames,
                    audit_capture_frame,
                )
            )
            frame = int(measurement.frame)
            records = np.frombuffer(measurement.raw_data, dtype=SEMANTIC_DTYPE)
            final_attempt = transform_settle["attempts"][-1]
            translation_error = final_attempt["source_translation_error_m"]
            rotation_error = final_attempt["source_rotation_error_rad"]
            ros_points, stats = transform_semantic_points_to_ros(
                records, matrix, args.static_tags
            )
            retained_from_tags = sum(
                count
                for tag, count in stats["raw_tag_counts"].items()
                if int(tag) in args.static_tags
            )
            if retained_from_tags != stats["retained_static_points"]:
                raise MapGenerationError(
                    f"scan {pose['scan_index']} contains non-finite retained static hits"
                )
            post_scan_audit = final_attempt["actor_and_server_audit"][
                "actor_audit"
            ]
            voxel_points = voxel_centroids(ros_points, args.voxel_size_m)
            if voxel_points.shape[0] == 0:
                raise MapGenerationError(f"scan {pose['scan_index']} retained no static points")
            chunk_name = f"scan_{pose['scan_index']:06d}.npy"
            chunk_path = chunks_dir / chunk_name
            if chunk_path.exists():
                raise MapGenerationError(f"refusing to overwrite uncheckpointed chunk {chunk_path}")
            atomic_save_npy(chunk_path, voxel_points)
            completed.append(
                {
                    "scan_index": pose["scan_index"],
                    "carla_frame": frame,
                    "chunk_file": chunk_name,
                    "sha256": sha256_file(chunk_path),
                    "raw_points": stats["raw_points"],
                    "retained_static_points": stats["retained_static_points"],
                    "voxel_points": int(voxel_points.shape[0]),
                    "raw_tag_counts": stats["raw_tag_counts"],
                    "source_translation_error_m": translation_error,
                    "source_rotation_error_rad": rotation_error,
                    "commanded_transform_readback_error_m": (
                        readback_translation_error
                    ),
                    "commanded_transform_readback_rotation_error_rad": (
                        readback_rotation_error
                    ),
                    "transform_settle": transform_settle,
                    "actor_audit": {
                        "before": pre_scan_audit,
                        "after": post_scan_audit,
                    },
                }
            )
            checkpoint.update(
                {
                    "status": "COLLECTING",
                    "completed_scans": completed,
                    "tag_totals": _checkpoint_tag_totals(completed),
                    "updated_at_utc": utc_now(),
                }
            )
            atomic_write_json(checkpoint_path, checkpoint)
            print(
                f"[{len(completed)}/{len(poses)}] scan={pose['scan_index']} "
                f"raw={records.shape[0]} static={ros_points.shape[0]} "
                f"voxel={voxel_points.shape[0]}",
                flush=True,
            )
    except BaseException as error:
        failure = error
    finally:
        if sensor is not None:
            try:
                sensor.stop()
            except RuntimeError as error:
                cleanup_errors.append(f"sensor.stop: {error}")
            try:
                if sensor.destroy() is False:
                    cleanup_errors.append("sensor.destroy returned false")
            except RuntimeError as error:
                cleanup_errors.append(f"sensor.destroy: {error}")
        try:
            world.apply_settings(original_settings)
            restored_readback = _settings_snapshot(world.get_settings())
            restored = _settings_snapshots_match(original_snapshot, restored_readback)
            if not restored:
                cleanup_errors.append("world settings restore readback mismatch")
        except (RuntimeError, MapGenerationError) as error:
            restored = False
            cleanup_errors.append(f"world settings restore: {error}")
        try:
            verify_live_server_attestation(server_attestation, full_hash=True)
            server_attestation_final_verified = True
        except MapGenerationError as error:
            server_attestation_final_verified = False
            cleanup_errors.append(f"CARLA server attestation: {error}")
        session.update(
            {
                "ended_at_utc": utc_now(),
                "ending_completed_scans": len(checkpoint["completed_scans"]),
                "wall_seconds": time.monotonic() - start,
                "requested_world_settings": _settings_snapshot(applied_settings),
                "applied_world_settings": applied_readback,
                "restored_world_settings": restored_readback,
                "world_settings_restored": restored,
                "server_attestation_final_verified": server_attestation_final_verified,
                "semantic_lidar_blueprint": "sensor.lidar.ray_cast_semantic",
                "semantic_lidar_attributes": blueprint_attributes,
                "cleanup_errors": cleanup_errors,
                "outcome": (
                    "INTERRUPTED"
                    if isinstance(failure, InterruptedGeneration)
                    else "FAIL" if failure is not None or cleanup_errors else "PENDING_FINAL_AUDIT"
                ),
                "failure": str(failure) if failure is not None else None,
            }
        )
        checkpoint.update(
            {
                "status": "COLLECTING",
                "sessions": sessions,
                "updated_at_utc": utc_now(),
            }
        )
        atomic_write_json(checkpoint_path, checkpoint)

    if not restored or cleanup_errors:
        raise MapGenerationError(
            "collection cleanup/settings restoration failed: " + "; ".join(cleanup_errors)
        ) from failure
    if failure is not None:
        raise failure

    final_audit = static_scene_actor_audit(world.get_actors(), baseline_actors)
    verify_live_server_attestation(server_attestation)
    if final_audit["status"] != "PASS_PINNED_ACTOR_SET":
        session["outcome"] = "FAIL_FINAL_ACTOR_AUDIT"
        session["final_actor_audit"] = final_audit
        checkpoint["updated_at_utc"] = utc_now()
        atomic_write_json(checkpoint_path, checkpoint)
        raise MapGenerationError("the pinned actor set changed; evidence is invalid")
    session["outcome"] = "PASS"
    session["final_actor_audit"] = final_audit
    all_sessions_restored = all(
        item.get("world_settings_restored") is True for item in sessions
    )
    all_sessions_cleanup_error_free = all(
        item.get("cleanup_errors") == [] for item in sessions
    )
    all_sessions_finalized = all(
        item.get("outcome") in {"PASS", "INTERRUPTED", "FAIL"}
        for item in sessions
    )
    if not (
        all_sessions_restored
        and all_sessions_cleanup_error_free
        and all_sessions_finalized
    ):
        checkpoint.update(
            {
                "status": "COLLECTING",
                "collection_validity": {
                    "status": "FAIL_TAINTED_SESSION",
                    "all_sessions_restored_world_settings": all_sessions_restored,
                    "all_sessions_cleanup_error_free": all_sessions_cleanup_error_free,
                    "all_sessions_finalized": all_sessions_finalized,
                    "final_actor_audit": final_audit,
                },
                "updated_at_utc": utc_now(),
            }
        )
        atomic_write_json(checkpoint_path, checkpoint)
        raise MapGenerationError(
            "collection contains an unfinalized or cleanup-tainted prior session; "
            "refusing COMPLETE_QA_PASS"
        )
    checkpoint.update(
        {
            "status": "COLLECTED",
            "collection_validity": {
                "status": "PASS",
                "every_scan_has_before_and_after_pinned_actor_set_audits": True,
                "cross_session_baseline_actor_type_multiset_sha256": baseline_signature,
                "all_sessions_restored_world_settings": all_sessions_restored,
                "all_sessions_cleanup_error_free": all_sessions_cleanup_error_free,
                "all_sessions_finalized": all_sessions_finalized,
                "final_actor_audit": final_audit,
            },
            "updated_at_utc": utc_now(),
        }
    )
    atomic_write_json(checkpoint_path, checkpoint)
    checkpoint = _load_checkpoint(checkpoint_path, plan, chunks_dir)
    runtime = {
        "sessions": sessions,
        "session_count": len(sessions),
        "total_wall_seconds": sum(float(item.get("wall_seconds", 0.0)) for item in sessions),
        "collection_validity": checkpoint["collection_validity"],
    }
    return checkpoint, runtime


def _write_pcd_from_payload(path: Path, payload_path: Path, points: int) -> None:
    if path.exists() or path.is_symlink():
        raise MapGenerationError(f"refusing to overwrite existing output {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "wb") as output, payload_path.open("rb") as source:
            output.write(_pcd_header(points))
            shutil.copyfileobj(source, output, length=16 * 1024 * 1024)
            output.flush()
            os.fsync(output.fileno())
        try:
            os.link(temporary, path)
        except FileExistsError as error:
            raise MapGenerationError(
                f"refusing to overwrite concurrently created output {path}"
            ) from error
        os.unlink(temporary)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def merge_chunks_to_pcd(
    checkpoint: Mapping[str, Any],
    output_root: Path,
    output_pcd: Path,
    voxel_size_m: float,
    bucket_size_m: float,
    encoding: str,
) -> dict[str, Any]:
    voxel_size_m = _finite_positive(voxel_size_m, "voxel_size_m")
    bucket_size_m = _finite_positive(bucket_size_m, "bucket_size_m")
    if output_pcd.exists() or output_pcd.is_symlink():
        raise MapGenerationError(f"refusing to overwrite existing output {output_pcd}")
    bucket_voxels = max(1, int(round(bucket_size_m / voxel_size_m)))
    work = Path(tempfile.mkdtemp(prefix=".pcd-merge-", dir=str(output_root)))
    buckets = work / "buckets"
    buckets.mkdir()
    try:
        for item in checkpoint["completed_scans"]:
            points = np.load(output_root / "chunks" / item["chunk_file"], allow_pickle=False)
            keys = np.floor(points.astype(np.float64) / voxel_size_m).astype(np.int64)
            bucket_keys = np.floor_divide(keys[:, :2], bucket_voxels)
            unique_buckets = np.unique(bucket_keys, axis=0)
            for bucket_x, bucket_y in unique_buckets:
                mask = (bucket_keys[:, 0] == bucket_x) & (bucket_keys[:, 1] == bucket_y)
                bucket_path = buckets / f"b_{bucket_x:+08d}_{bucket_y:+08d}.bin"
                with bucket_path.open("ab") as stream:
                    stream.write(np.asarray(points[mask], dtype="<f4").tobytes(order="C"))

        payload_path = work / "xyz_payload.bin"
        point_count = 0
        bounds_min = np.full(3, np.inf, dtype=np.float64)
        bounds_max = np.full(3, -np.inf, dtype=np.float64)

        def bucket_sort_key(path: Path) -> tuple[int, int]:
            _, bucket_x, bucket_y = path.stem.split("_", 2)
            return int(bucket_x), int(bucket_y)

        with payload_path.open("wb") as payload:
            for bucket_path in sorted(buckets.glob("b_*.bin"), key=bucket_sort_key):
                values = np.fromfile(bucket_path, dtype="<f4")
                if values.size % 3:
                    raise MapGenerationError(f"partition payload is corrupt: {bucket_path}")
                points = voxel_centroids(values.reshape(-1, 3), voxel_size_m)
                if points.shape[0] == 0:
                    continue
                payload.write(np.asarray(points, dtype="<f4").tobytes(order="C"))
                point_count += int(points.shape[0])
                bounds_min = np.minimum(bounds_min, points.min(axis=0))
                bounds_max = np.maximum(bounds_max, points.max(axis=0))
            payload.flush()
            os.fsync(payload.fileno())
        if point_count == 0:
            raise MapGenerationError("global voxel merge produced an empty point cloud")

        binary_path = output_pcd if encoding == "binary" else work / "uncompressed.pcd"
        _write_pcd_from_payload(binary_path, payload_path, point_count)
        if encoding == "binary_compressed":
            converter = shutil.which("pcl_convert_pcd_ascii_binary")
            if converter is None:
                raise MapGenerationError(
                    "binary_compressed requested but pcl_convert_pcd_ascii_binary is absent"
                )
            temporary_output = work / "compressed.pcd"
            subprocess.run(
                [converter, str(binary_path), str(temporary_output), "2"],
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            inspected = inspect_pcd(temporary_output)
            if inspected["points"] != point_count or inspected["encoding"] != encoding:
                raise MapGenerationError("PCL compressed output differs from merge contract")
            output_pcd.parent.mkdir(parents=True, exist_ok=True)
            try:
                os.link(temporary_output, output_pcd)
            except FileExistsError as error:
                raise MapGenerationError(
                    f"refusing to overwrite concurrently created output {output_pcd}"
                ) from error
            temporary_output.unlink()
        return {
            **inspect_pcd(output_pcd),
            "voxel_size_m": voxel_size_m,
            "bucket_size_m": bucket_size_m,
            "bounds_ros_xyz_m": {
                "min": [float(value) for value in bounds_min],
                "max": [float(value) for value in bounds_max],
            },
        }
    finally:
        shutil.rmtree(work, ignore_errors=True)


def _read_lanelet_nodes(path: Path) -> np.ndarray:
    points = []
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError) as error:
        raise MapGenerationError(f"cannot parse Lanelet2 map {path}: {error}") from error
    for node in root.findall("node"):
        tags = {child.get("k"): child.get("v") for child in node.findall("tag")}
        if {"local_x", "local_y", "ele"}.issubset(tags):
            points.append((float(tags["local_x"]), float(tags["local_y"]), float(tags["ele"])))
    result = np.asarray(points, dtype=np.float64)
    if result.ndim != 2 or result.shape[0] == 0 or not np.isfinite(result).all():
        raise MapGenerationError(f"Lanelet2 contains no finite local XYZ nodes: {path}")
    return result


def _read_cloud_points(path: Path, downsample_m: float = 0.25) -> np.ndarray:
    try:
        import open3d as o3d
    except ImportError as error:
        raise MapGenerationError(f"point-cloud QA requires open3d: {error}") from error
    cloud = o3d.io.read_point_cloud(str(path))
    if downsample_m > 0.0:
        cloud = cloud.voxel_down_sample(downsample_m)
    points = np.asarray(cloud.points, dtype=np.float64)
    if points.ndim != 2 or points.shape[0] == 0 or not np.isfinite(points).all():
        raise MapGenerationError(f"generated PCD is empty or non-finite: {path}")
    return points


def _nearest_qa(reference_xyz: np.ndarray, cloud_xyz: np.ndarray) -> dict[str, Any]:
    try:
        from scipy.spatial import cKDTree
    except ImportError as error:
        raise MapGenerationError(f"alignment QA requires scipy: {error}") from error
    # Full XYZ selection is intentional.  An XY-only tree has ambiguous ties
    # on bridges/underpasses and can select the wrong elevation according to
    # input array order.  Report planar and vertical components of the stable
    # nearest-3D correspondence separately for the admission thresholds.
    _distances_3d, indices = cKDTree(cloud_xyz).query(reference_xyz, workers=-1)
    deltas = cloud_xyz[indices] - reference_xyz
    distances = np.linalg.norm(deltas[:, :2], axis=1)
    z_distances = np.abs(deltas[:, 2])
    return {
        "reference_points": int(reference_xyz.shape[0]),
        "nearest_xy_m": {
            "median": float(np.median(distances)),
            "p95": float(np.percentile(distances, 95)),
            "max": float(np.max(distances)),
            "within_1m_fraction": float(np.mean(distances <= 1.0)),
            "within_2m_fraction": float(np.mean(distances <= 2.0)),
        },
        "nearest_abs_z_m": {
            "median": float(np.median(z_distances)),
            "p95": float(np.percentile(z_distances, 95)),
            "max": float(np.max(z_distances)),
        },
    }


def _alignment_pass(metrics: Mapping[str, Any]) -> bool:
    xy = metrics["nearest_xy_m"]
    z = metrics["nearest_abs_z_m"]
    return (
        xy["median"] <= 0.75
        and xy["p95"] <= 1.5
        and xy["within_2m_fraction"] >= 0.99
        and z["p95"] <= 1.5
    )


def _route_xyz(path: Path) -> np.ndarray:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise MapGenerationError(f"cannot read route {path}: {error}") from error
    raw_poses = [payload.get("start_ros_pose"), *payload.get("route", [])]
    poses = [
        (float(pose["x"]), float(pose["y"]), float(pose["z"]))
        for pose in raw_poses
        if isinstance(pose, Mapping)
    ]
    result = np.asarray(poses, dtype=np.float64)
    if result.ndim != 2 or result.shape[0] < 2 or not np.isfinite(result).all():
        raise MapGenerationError(f"route has fewer than two finite ROS poses: {path}")
    return result


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def analyze_route_contract(path: Path, expected_town: str | None = None) -> dict[str, Any]:
    """Verify scenario metadata, road options, and actual planar turn geometry."""
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise MapGenerationError(f"cannot read route contract {path}: {error}") from error
    if not isinstance(payload, dict):
        raise MapGenerationError(f"route contract root is not an object: {path}")
    scenario = payload.get("scenario")
    if scenario not in {"lane_follow", "straight", "left", "right"}:
        raise MapGenerationError(f"route has unsupported scenario {scenario!r}: {path}")
    if path.parent.name != scenario:
        raise MapGenerationError(
            f"route scenario {scenario!r} differs from directory {path.parent.name!r}: {path}"
        )
    if expected_town is not None:
        town = payload.get("town")
        normalized_actual = "".join(character.lower() for character in str(town) if character.isalnum())
        normalized_expected = "".join(
            character.lower() for character in _canonical_map_leaf(expected_town) if character.isalnum()
        )
        if not town or normalized_actual != normalized_expected:
            raise MapGenerationError(
                f"route town {town!r} differs from expected map {expected_town!r}: {path}"
            )
    route = payload.get("route")
    if not isinstance(route, list) or len(route) < 3:
        raise MapGenerationError(f"route has fewer than three serialized poses: {path}")
    points = []
    option_indices: dict[str, list[int]] = defaultdict(list)
    for index, pose in enumerate(route):
        if not isinstance(pose, dict):
            raise MapGenerationError(f"route[{index}] is not an object: {path}")
        try:
            point = float(pose["x"]), float(pose["y"])
        except (KeyError, TypeError, ValueError) as error:
            raise MapGenerationError(f"route[{index}] has invalid XY: {path}") from error
        if not all(math.isfinite(value) for value in point):
            raise MapGenerationError(f"route[{index}] has non-finite XY: {path}")
        option = pose.get("road_option")
        if not isinstance(option, str) or not option:
            raise MapGenerationError(f"route[{index}] has no serialized road_option: {path}")
        points.append(point)
        option_indices[option].append(index)
    computed_counts = {key: len(value) for key, value in sorted(option_indices.items())}
    if payload.get("option_counts") != computed_counts:
        raise MapGenerationError(f"route option_counts differs from serialized poses: {path}")

    segment_lengths = [
        math.hypot(second[0] - first[0], second[1] - first[1])
        for first, second in zip(points, points[1:])
    ]
    geometric_length = sum(segment_lengths)
    endpoint_displacement = math.hypot(
        points[-1][0] - points[0][0], points[-1][1] - points[0][1]
    )
    try:
        declared_length = float(payload["route_length_m"])
    except (KeyError, TypeError, ValueError) as error:
        raise MapGenerationError(f"route has no finite route_length_m: {path}") from error
    if not math.isfinite(declared_length) or not math.isclose(
        declared_length,
        geometric_length,
        rel_tol=1e-3,
        abs_tol=0.05,
    ):
        raise MapGenerationError(
            f"route_length_m differs from planar serialized geometry: {path}"
        )
    if geometric_length < 20.0 or endpoint_displacement < 10.0:
        raise MapGenerationError(
            f"route geometry is too short for driving evidence: length={geometric_length:.3f} m, "
            f"displacement={endpoint_displacement:.3f} m: {path}"
        )
    if endpoint_displacement / geometric_length < 0.25:
        raise MapGenerationError(f"route geometry is dominated by backtracking: {path}")

    forbidden_change = {"CHANGELANELEFT", "CHANGELANERIGHT"}
    if forbidden_change.intersection(option_indices):
        raise MapGenerationError(f"route scenario contains a lane-change command: {path}")
    target_by_scenario = {"straight": "STRAIGHT", "left": "LEFT", "right": "RIGHT"}
    target = target_by_scenario.get(scenario)
    if scenario == "lane_follow":
        unexpected = set(option_indices) - {"LANEFOLLOW"}
        if unexpected:
            raise MapGenerationError(
                f"lane_follow route contains non-LANEFOLLOW commands {sorted(unexpected)}: {path}"
            )
        return {
            "scenario": scenario,
            "option_counts": computed_counts,
            "geometric_length_m": geometric_length,
            "endpoint_displacement_m": endpoint_displacement,
            "maneuver_heading_change_rad": 0.0,
            "geometry_status": "PASS_NO_JUNCTION_COMMAND",
        }
    if target not in option_indices:
        raise MapGenerationError(f"{scenario} route has no {target} road option: {path}")
    unexpected = set(option_indices) - {"LANEFOLLOW", target}
    if unexpected:
        raise MapGenerationError(
            f"{scenario} route contains foreign commands {sorted(unexpected)}: {path}"
        )

    indices = option_indices[target]
    if indices != list(range(indices[0], indices[-1] + 1)):
        raise MapGenerationError(f"{scenario} maneuver command is split into multiple blocks: {path}")
    first = max(0, min(indices) - 5)
    last = min(len(points) - 1, max(indices) + 5)
    if last - first < 4:
        raise MapGenerationError(f"route has too little geometry around {target}: {path}")
    before_end = min(first + 3, last)
    after_start = max(first, last - 3)
    entry_leg_length = math.hypot(
        points[before_end][0] - points[first][0],
        points[before_end][1] - points[first][1],
    )
    exit_leg_length = math.hypot(
        points[last][0] - points[after_start][0],
        points[last][1] - points[after_start][1],
    )
    if entry_leg_length < 2.0 or exit_leg_length < 2.0:
        raise MapGenerationError(
            f"route has degenerate maneuver heading legs: entry={entry_leg_length:.3f} m, "
            f"exit={exit_leg_length:.3f} m: {path}"
        )
    before = math.atan2(
        points[before_end][1] - points[first][1],
        points[before_end][0] - points[first][0],
    )
    after = math.atan2(
        points[last][1] - points[after_start][1],
        points[last][0] - points[after_start][0],
    )
    heading_change = _wrap_angle(after - before)
    minimum_turn_rad = math.radians(20.0)
    maximum_straight_rad = math.radians(20.0)
    geometry_pass = (
        (scenario == "left" and heading_change >= minimum_turn_rad)
        or (scenario == "right" and heading_change <= -minimum_turn_rad)
        or (scenario == "straight" and abs(heading_change) <= maximum_straight_rad)
    )
    if not geometry_pass:
        raise MapGenerationError(
            f"{scenario} route geometry changes heading by {heading_change:.3f} rad: {path}"
        )
    return {
        "scenario": scenario,
        "option_counts": computed_counts,
        "geometric_length_m": geometric_length,
        "endpoint_displacement_m": endpoint_displacement,
        "maneuver_entry_leg_m": entry_leg_length,
        "maneuver_exit_leg_m": exit_leg_length,
        "maneuver_heading_change_rad": heading_change,
        "maneuver_window_indices": [first, last],
        "geometry_status": "PASS_OPTION_AND_PLANAR_HEADING",
    }


def route_inventory(route_root: Path, expected_town: str | None = None) -> dict[str, Any]:
    paths = sorted(route_root.rglob("*.json")) if route_root.is_dir() else []
    if not paths:
        raise MapGenerationError(f"route root contains no JSON routes: {route_root}")
    files = []
    for path in paths:
        contract = analyze_route_contract(path, expected_town)
        files.append(
            {
                "relative_path": str(path.relative_to(route_root)),
                "sha256": sha256_file(path),
                **contract,
            }
        )
    scenarios = {item["scenario"] for item in files}
    if "straight" not in scenarios or not scenarios.intersection({"left", "right"}):
        raise MapGenerationError("route inventory lacks a truthful straight and turn route")
    return {
        "root": str(route_root.resolve()),
        "files": files,
        "straight_present": True,
        "turn_present": True,
    }


def cooked_asset_inventory(content_root: Path, token: str) -> dict[str, Any]:
    content_root = content_root.expanduser().resolve()
    if not content_root.is_dir() or not token:
        raise MapGenerationError("CARLA content root and cooked asset token must be valid")
    files = []
    lowered = token.lower()
    for path in sorted(candidate for candidate in content_root.rglob("*") if candidate.is_file()):
        relative = path.relative_to(content_root)
        if lowered not in str(relative).lower():
            continue
        files.append(
            {
                "relative_path": str(relative),
                "size_bytes": path.stat().st_size,
                "sha256": sha256_file(path),
            }
        )
    if not files:
        raise MapGenerationError(
            f"no cooked CARLA assets containing {token!r} found below {content_root}"
        )
    serialized = json.dumps(files, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return {
        "content_root": str(content_root),
        "selection": f"case-insensitive relative path contains {token!r}",
        "token": token,
        "file_count": len(files),
        "total_size_bytes": sum(item["size_bytes"] for item in files),
        "inventory_sha256": sha256_bytes(serialized),
        "scope_limit": (
            "Pins Town-named cooked assets; shared generic CARLA materials/meshes are "
            "outside this dependency inventory."
        ),
        "files": files,
    }


def snapshot_inputs(
    output_root: Path,
    lanelet_path: Path,
    route_root: Path,
    route_source_inventory: Mapping[str, Any],
    cooked_inventory: Mapping[str, Any],
) -> dict[str, Any]:
    inputs = output_root / "inputs"
    snapshot_lanelet = inputs / "lanelet2_map.osm"
    atomic_copy_file(lanelet_path, snapshot_lanelet)
    snapshot_routes = inputs / "routes"
    for record in route_source_inventory["files"]:
        relative = Path(record["relative_path"])
        source = route_root / relative
        target = snapshot_routes / relative
        atomic_copy_file(source, target)
        if sha256_file(target) != record["sha256"]:
            raise MapGenerationError(f"route snapshot hash mismatch: {target}")
    generator_source = Path(__file__).resolve()
    validator_source = Path(__file__).with_name("validate_route_map.py").resolve()
    generator_snapshot = inputs / "implementation/generate_carla_semantic_lidar_map.py"
    validator_snapshot = inputs / "implementation/validate_route_map.py"
    atomic_copy_file(generator_source, generator_snapshot)
    atomic_copy_file(validator_source, validator_snapshot)
    inventory_path = inputs / "cooked_asset_inventory.json"
    inventory_text = json.dumps(
        cooked_inventory, indent=2, sort_keys=True, allow_nan=False
    ) + "\n"
    if inventory_path.is_file():
        if inventory_path.read_text(encoding="utf-8") != inventory_text:
            raise MapGenerationError("immutable cooked-asset inventory snapshot differs")
    else:
        atomic_write_json(inventory_path, cooked_inventory)
    snapshot_route_inventory = route_inventory(snapshot_routes)
    if snapshot_route_inventory["files"] != route_source_inventory["files"]:
        raise MapGenerationError(
            "route snapshot inventory differs from the exact source inventory; "
            "refusing stale or additional route evidence"
        )
    return {
        "lanelet2": {
            "source_path": str(lanelet_path),
            "snapshot_path": str(snapshot_lanelet),
            "sha256": sha256_file(snapshot_lanelet),
        },
        "routes": {
            **route_source_inventory,
            "snapshot_root": str(snapshot_routes),
            "snapshot_files": snapshot_route_inventory["files"],
        },
        "implementation": {
            "generator_path": str(generator_source),
            "generator_sha256": sha256_file(generator_source),
            "generator_snapshot_path": str(generator_snapshot),
            "validator_path": str(validator_source),
            "validator_sha256": sha256_file(validator_source),
            "validator_snapshot_path": str(validator_snapshot),
        },
        "cooked_assets": {
            key: value for key, value in cooked_inventory.items() if key != "files"
        }
        | {
            "inventory_path": str(inventory_path),
            "inventory_file_sha256": sha256_file(inventory_path),
        },
    }


def verify_pinned_inputs(plan: Mapping[str, Any]) -> None:
    lanelet = plan["lanelet2"]
    if sha256_file(Path(lanelet["snapshot_path"])) != lanelet["sha256"]:
        raise MapGenerationError("Lanelet2 input snapshot changed during collection")
    routes = plan["routes"]
    snapshot_root = Path(routes["snapshot_root"])
    current_routes = route_inventory(
        snapshot_root, plan.get("map", {}).get("expected_name")
    )
    if (
        current_routes["files"] != routes.get("snapshot_files")
        or routes.get("files") != routes.get("snapshot_files")
    ):
        raise MapGenerationError(
            "route input snapshot inventory changed during collection"
        )
    implementation = plan["implementation"]
    for kind in ("generator", "validator"):
        if sha256_file(Path(implementation[f"{kind}_snapshot_path"])) != implementation[
            f"{kind}_sha256"
        ]:
            raise MapGenerationError(f"{kind} implementation snapshot changed")
    cooked = plan["cooked_assets"]
    inventory_path = Path(cooked["inventory_path"])
    if (
        not inventory_path.is_file()
        or sha256_file(inventory_path) != cooked["inventory_file_sha256"]
    ):
        raise MapGenerationError("cooked CARLA asset inventory snapshot changed")
    try:
        snapshot_inventory = json.loads(inventory_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise MapGenerationError(
            "cannot read cooked CARLA asset inventory snapshot"
        ) from error
    current = cooked_asset_inventory(Path(cooked["content_root"]), cooked["token"])
    if (
        current["inventory_sha256"] != cooked["inventory_sha256"]
        or snapshot_inventory != current
    ):
        raise MapGenerationError("Town-named cooked CARLA assets changed during collection")
    for record in plan["carla_runtime_binaries"].values():
        path = Path(record["path"])
        if (
            not path.is_file()
            or path.stat().st_size != record["size_bytes"]
            or sha256_file(path) != record["sha256"]
        ):
            raise MapGenerationError(f"pinned CARLA runtime binary changed: {path}")


def _load_route_validator(path: Path | None = None) -> Any:
    path = path or Path(__file__).with_name("validate_route_map.py")
    spec = importlib.util.spec_from_file_location(
        "_semantic_lidar_route_map_validator", path
    )
    if spec is None or spec.loader is None:
        raise MapGenerationError(f"cannot load route/Lanelet validator: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    try:
        spec.loader.exec_module(module)
    except Exception:
        sys.modules.pop(spec.name, None)
        raise
    return module


def run_global_qa(
    pcd_path: Path,
    lanelet_path: Path,
    route_root: Path,
    expected_town: str | None = None,
    validator_path: Path | None = None,
) -> dict[str, Any]:
    cloud = _read_cloud_points(pcd_path)
    lanelet_nodes = _read_lanelet_nodes(lanelet_path)
    lanelet_metrics = _nearest_qa(lanelet_nodes, cloud)
    lanelet_metrics["status"] = "PASS" if _alignment_pass(lanelet_metrics) else "FAIL"
    inventory = route_inventory(route_root, expected_town)
    route_paths = [route_root / item["relative_path"] for item in inventory["files"]]
    contracts = {item["relative_path"]: item for item in inventory["files"]}
    route_validator = _load_route_validator(validator_path)
    try:
        lanelet_polygons, _, _ = route_validator.load_lanelet_polygons(lanelet_path)
    except route_validator.ValidationError as error:
        raise MapGenerationError(f"Lanelet polygon validation failed: {error}") from error
    route_cases = []
    for route_path in route_paths:
        contract = contracts[str(route_path.relative_to(route_root))]
        metrics = _nearest_qa(_route_xyz(route_path), cloud)
        try:
            _town, route_points = route_validator.load_route(route_path)
            maximum_distance, maximum_vertical_distance, matched_lanelets = (
                route_validator.validate_route_points(
                    route_points,
                    lanelet_polygons,
                    tolerance_m=0.75,
                    vertical_tolerance_m=1.5,
                )
            )
        except route_validator.ValidationError as error:
            lanelet_route = {"status": "FAIL", "errors": list(error.errors)}
        else:
            lanelet_route = {
                "status": "PASS",
                "pose_count": len(route_points),
                "matched_lanelet_count": matched_lanelets,
                "maximum_outside_lanelet_distance_m": maximum_distance,
                "maximum_vertical_distance_m": maximum_vertical_distance,
                "tolerance_m": 0.75,
                "vertical_tolerance_m": 1.5,
            }
        status = (
            "PASS"
            if _alignment_pass(metrics) and lanelet_route["status"] == "PASS"
            else "FAIL"
        )
        route_cases.append(
            {
                "route": str(route_path.resolve()),
                "route_sha256": contract["sha256"],
                "scenario": contract["scenario"],
                "route_contract": contract,
                "status": status,
                "lanelet_route": lanelet_route,
                "pointcloud_route": metrics,
                **metrics,
            }
        )
    scenario_status = {
        scenario: "PASS"
        if any(case["scenario"] == scenario and case["status"] == "PASS" for case in route_cases)
        else "MISSING_OR_FAIL"
        for scenario in ("straight", "left", "right")
    }
    route_status = (
        "PASS"
        if all(case["status"] == "PASS" for case in route_cases)
        and scenario_status["straight"] == "PASS"
        and any(scenario_status[name] == "PASS" for name in ("left", "right"))
        else "FAIL"
    )
    return {
        "status": "PASS"
        if lanelet_metrics["status"] == "PASS" and route_status == "PASS"
        else "FAIL",
        "method": (
            "global Lanelet-node to 0.25 m QA cloud; each route to Lanelet polygons "
            "and the QA cloud"
        ),
        "thresholds": {
            "xy_median_max_m": 0.75,
            "xy_p95_max_m": 1.5,
            "xy_within_2m_min_fraction": 0.99,
            "abs_z_p95_max_m": 1.5,
        },
        "cloud_geometry": {
            "qa_downsampled_points": int(cloud.shape[0]),
            "bounds_xyz_m": {
                "min": [float(value) for value in cloud.min(axis=0)],
                "max": [float(value) for value in cloud.max(axis=0)],
            },
            "extent_xyz_m": [
                float(value) for value in (cloud.max(axis=0) - cloud.min(axis=0))
            ],
        },
        "lanelet2": {
            "path": str(lanelet_path.resolve()),
            "sha256": sha256_file(lanelet_path),
            **lanelet_metrics,
        },
        "routes": {
            "root": str(route_root.resolve()),
            "status": route_status,
            "scenario_status": scenario_status,
            "case_count": len(route_cases),
            "cases": route_cases,
        },
    }


def semantic_surface_coverage_qa(
    checkpoint: Mapping[str, Any],
    plan: Mapping[str, Any],
    cloud_geometry: Mapping[str, Any],
) -> dict[str, Any]:
    structural_tags = {3, 4, 5, 6, 7, 8, 9, 20, 22, 26, 28}
    retained_tags = {
        int(value) for value in plan["semantic_contract"]["retained_static_tags"]
    }
    completed = checkpoint["completed_scans"]
    totals: Counter[int] = Counter()
    scans_with_structural = 0
    scans_with_dense_chunk = 0
    for scan in completed:
        counts = {
            int(tag): int(count) for tag, count in scan["raw_tag_counts"].items()
        }
        totals.update(
            {tag: count for tag, count in counts.items() if tag in retained_tags}
        )
        if sum(counts.get(tag, 0) for tag in structural_tags & retained_tags) > 0:
            scans_with_structural += 1
        if int(scan["voxel_points"]) >= 100:
            scans_with_dense_chunk += 1
    retained_hits = sum(totals.values())
    structural_hits = sum(totals[tag] for tag in structural_tags)
    retained_classes = sum(count > 0 for count in totals.values())
    structural_classes = sum(totals[tag] > 0 for tag in structural_tags)
    scan_count = len(completed)
    structural_scan_fraction = scans_with_structural / scan_count if scan_count else 0.0
    dense_scan_fraction = scans_with_dense_chunk / scan_count if scan_count else 0.0
    z_extent = float(cloud_geometry["extent_xyz_m"][2])
    downsampled_points = int(cloud_geometry["qa_downsampled_points"])
    thresholds = {
        "minimum_qa_downsampled_points": 50_000,
        "minimum_structural_semantic_hits": 10_000,
        "minimum_retained_semantic_classes": 5,
        "minimum_structural_semantic_classes": 2,
        "minimum_scans_with_structural_hits_fraction": 0.20,
        "minimum_scans_with_at_least_100_voxels_fraction": 0.99,
        "minimum_cloud_z_extent_m": 5.0,
    }
    passed = (
        downsampled_points >= thresholds["minimum_qa_downsampled_points"]
        and structural_hits >= thresholds["minimum_structural_semantic_hits"]
        and retained_classes >= thresholds["minimum_retained_semantic_classes"]
        and structural_classes >= thresholds["minimum_structural_semantic_classes"]
        and structural_scan_fraction
        >= thresholds["minimum_scans_with_structural_hits_fraction"]
        and dense_scan_fraction
        >= thresholds["minimum_scans_with_at_least_100_voxels_fraction"]
        and z_extent >= thresholds["minimum_cloud_z_extent_m"]
    )
    return {
        "status": "PASS" if passed else "FAIL",
        "scope": (
            "anti-sparse/anti-road-only construction gate; closed-loop Autoware "
            "localization remains the usability test"
        ),
        "thresholds": thresholds,
        "metrics": {
            "qa_downsampled_points": downsampled_points,
            "retained_semantic_hits": retained_hits,
            "structural_semantic_hits": structural_hits,
            "retained_semantic_class_count": retained_classes,
            "structural_semantic_class_count": structural_classes,
            "scans_with_structural_hits_fraction": structural_scan_fraction,
            "scans_with_at_least_100_voxels_fraction": dense_scan_fraction,
            "cloud_z_extent_m": z_extent,
            "retained_tag_counts": {
                str(tag): totals[tag] for tag in sorted(totals) if totals[tag] > 0
            },
        },
    }


def run_complete_qa(
    pcd_path: Path, plan: Mapping[str, Any], checkpoint: Mapping[str, Any]
) -> dict[str, Any]:
    qa = run_global_qa(
        pcd_path,
        Path(plan["lanelet2"]["snapshot_path"]),
        Path(plan["routes"]["snapshot_root"]),
        expected_town=plan["map"]["expected_name"],
        validator_path=Path(plan["implementation"]["validator_snapshot_path"]),
    )
    semantic_coverage = semantic_surface_coverage_qa(
        checkpoint, plan, qa["cloud_geometry"]
    )
    qa["semantic_surface_coverage"] = semantic_coverage
    if semantic_coverage["status"] != "PASS":
        qa["status"] = "FAIL"
    return qa


def validate_existing_complete_provenance(
    provenance: Mapping[str, Any], output_root: Path, plan: Mapping[str, Any]
) -> None:
    if provenance.get("schema_version") != 1 or provenance.get("status") != "COMPLETE_QA_PASS":
        raise MapGenerationError("existing provenance is not COMPLETE_QA_PASS")
    plan_path = output_root / "scan_plan.json"
    checkpoint_path = output_root / "checkpoint.json"
    pointcloud_path = output_root / "pointcloud_map.pcd"
    plan_record = provenance.get("scan_plan")
    checkpoint_record = provenance.get("checkpoint")
    pointcloud_record = provenance.get("pointcloud")
    if not all(
        isinstance(record, Mapping)
        for record in (plan_record, checkpoint_record, pointcloud_record)
    ):
        raise MapGenerationError("existing provenance lacks required artifact records")
    if (
        Path(str(plan_record.get("path", ""))).expanduser().resolve() != plan_path
        or plan_record.get("sha256") != sha256_file(plan_path)
        or plan_record.get("fingerprint") != plan["fingerprint"]
    ):
        raise MapGenerationError("existing provenance scan-plan record is corrupt")
    if (
        Path(str(checkpoint_record.get("path", ""))).expanduser().resolve()
        != checkpoint_path
        or not checkpoint_path.is_file()
        or checkpoint_record.get("sha256") != sha256_file(checkpoint_path)
    ):
        raise MapGenerationError("existing provenance checkpoint record is corrupt")
    checkpoint = _load_checkpoint(checkpoint_path, plan, output_root / "chunks")
    if (
        checkpoint.get("status") != "COLLECTED"
        or checkpoint_record.get("completed_scans")
        != len(plan["scan_plan"]["poses"])
        or len(checkpoint["completed_scans"]) != len(plan["scan_plan"]["poses"])
        or checkpoint_record.get("sha256") != sha256_file(checkpoint_path)
    ):
        raise MapGenerationError("existing provenance checkpoint is incomplete or changed")
    if Path(str(pointcloud_record.get("path", ""))).expanduser().resolve() != pointcloud_path:
        raise MapGenerationError("existing provenance points to a different PCD")
    inspection = inspect_pcd(pointcloud_path)
    if any(
        inspection[key] != pointcloud_record.get(key)
        for key in ("sha256", "size_bytes", "points", "encoding")
    ):
        raise MapGenerationError("existing COMPLETE_QA_PASS pointcloud is absent or corrupt")

    recorded_qa = provenance.get("qa")
    if (
        not isinstance(recorded_qa, Mapping)
        or recorded_qa.get("status") != "PASS"
        or recorded_qa.get("lanelet2", {}).get("status") != "PASS"
        or recorded_qa.get("routes", {}).get("status") != "PASS"
    ):
        raise MapGenerationError("existing provenance QA record is not PASS")
    expected_routes = {
        str((Path(plan["routes"]["snapshot_root"]) / item["relative_path"]).resolve()): item
        for item in plan["routes"]["snapshot_files"]
    }
    recorded_cases = recorded_qa["routes"].get("cases")
    if not isinstance(recorded_cases, list) or len(recorded_cases) != len(expected_routes):
        raise MapGenerationError("existing provenance route QA inventory is incomplete")
    for case in recorded_cases:
        expected = expected_routes.get(str(case.get("route"))) if isinstance(case, dict) else None
        if (
            expected is None
            or case.get("status") != "PASS"
            or case.get("route_sha256") != expected["sha256"]
            or case.get("scenario") != expected["scenario"]
        ):
            raise MapGenerationError("existing provenance route QA case is corrupt")
    if (
        Path(str(recorded_qa["lanelet2"].get("path", ""))).resolve()
        != Path(plan["lanelet2"]["snapshot_path"])
        or recorded_qa["lanelet2"].get("sha256") != plan["lanelet2"]["sha256"]
    ):
        raise MapGenerationError("existing provenance Lanelet QA record is corrupt")
    qa_recheck = run_complete_qa(pointcloud_path, plan, checkpoint)
    if qa_recheck != recorded_qa:
        raise MapGenerationError("existing provenance QA does not match a fresh deterministic QA")
    runtime = provenance.get("runtime")
    if (
        not isinstance(runtime, Mapping)
        or runtime.get("collection_validity") != checkpoint.get("collection_validity")
        or runtime.get("session_count") != len(checkpoint.get("sessions", []))
    ):
        raise MapGenerationError("existing provenance runtime audit differs from checkpoint")


def _estimate(
    scan_count: int,
    points_per_second: int,
    fixed_delta_seconds: float,
    transform_settle_max_extra_frames: int,
) -> dict[str, Any]:
    raw_points = int(math.ceil(scan_count * points_per_second * fixed_delta_seconds))
    chunk_upper = raw_points * 12
    return {
        "raw_points_upper_bound": raw_points,
        "semantic_simulation_seconds": scan_count * fixed_delta_seconds,
        "maximum_discarded_settle_frames": (
            scan_count * transform_settle_max_extra_frames
        ),
        "maximum_semantic_simulation_seconds_with_settle": (
            scan_count
            * (1 + transform_settle_max_extra_frames)
            * fixed_delta_seconds
        ),
        "filtered_chunk_upper_bound_gib": chunk_upper / 1024**3,
        "recommended_free_working_disk_gib": max(5.0, chunk_upper * 4.0 / 1024**3),
        "wall_time_note": (
            "hardware/map dependent; measure a 50-pose pilot. Typical off-screen semantic ray "
            "casts take substantially longer than the simulated seconds."
        ),
    }


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--lanelet-map", type=Path, required=True)
    parser.add_argument("--route-root", type=Path, required=True)
    parser.add_argument("--carla-content-root", type=Path, required=True)
    parser.add_argument("--cooked-asset-token", default="Town10")
    parser.add_argument("--host", default=os.environ.get("CARLA_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("CARLA_PORT", "2000")))
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--expected-map", default=DEFAULT_EXPECTED_MAP)
    parser.add_argument(
        "--expected-opendrive-sha256", default=DEFAULT_EXPECTED_OPENDRIVE_SHA256
    )
    parser.add_argument("--waypoint-spacing-m", type=float, default=10.0)
    parser.add_argument("--tile-size-m", type=float, default=12.0)
    parser.add_argument("--z-band-m", type=float, default=4.0)
    parser.add_argument("--sensor-height-m", type=float, default=2.5)
    parser.add_argument("--fixed-delta-seconds", type=float, default=0.05)
    parser.add_argument("--channels", type=int, default=128)
    parser.add_argument("--range-m", type=float, default=80.0)
    parser.add_argument("--points-per-second", type=int, default=2_000_000)
    parser.add_argument("--upper-fov-deg", type=float, default=30.0)
    parser.add_argument("--lower-fov-deg", type=float, default=-80.0)
    parser.add_argument("--voxel-size-m", type=float, default=0.10)
    parser.add_argument("--bucket-size-m", type=float, default=50.0)
    parser.add_argument("--warmup-frames", type=int, default=2)
    parser.add_argument(
        "--transform-settle-max-extra-frames",
        type=int,
        default=2,
        help=(
            "bounded extra synchronous ticks used only to discard measurements "
            "whose own transform is still the previous scan pose"
        ),
    )
    parser.add_argument(
        "--static-tags",
        default=",".join(str(value) for value in sorted(DEFAULT_STATIC_TAGS)),
    )
    parser.add_argument(
        "--pcd-encoding", choices=("binary", "binary_compressed"), default="binary_compressed"
    )
    parser.add_argument("--resume", action="store_true")
    parser.add_argument(
        "--plan-only",
        action="store_true",
        help="write/verify the deterministic scan plan without changing world settings",
    )
    args = parser.parse_args(argv)
    try:
        args.static_tags = _parse_static_tags(args.static_tags)
        for name in (
            "timeout",
            "waypoint_spacing_m",
            "tile_size_m",
            "z_band_m",
            "sensor_height_m",
            "fixed_delta_seconds",
            "range_m",
            "voxel_size_m",
            "bucket_size_m",
        ):
            _finite_positive(getattr(args, name), name)
    except MapGenerationError as error:
        parser.error(str(error))
    if not 0 < args.port <= 65535:
        parser.error("port must be in 1..65535")
    if args.host not in {"127.0.0.1", "localhost", "::1"}:
        parser.error("host must be a loopback address for local server attestation")
    if args.channels <= 0 or args.points_per_second <= 0 or args.warmup_frames < 1:
        parser.error("channels, points-per-second, and warmup-frames must be positive")
    if not 0 <= args.transform_settle_max_extra_frames <= 10:
        parser.error("transform-settle-max-extra-frames must be in 0..10")
    if args.fixed_delta_seconds > 0.16:
        parser.error("fixed-delta-seconds must be <= 0.16 for CARLA's 16 substep limit")
    if not -90.0 <= args.lower_fov_deg < args.upper_fov_deg <= 90.0:
        parser.error("LiDAR vertical FOV is invalid")
    if len(args.expected_opendrive_sha256) != 64 or any(
        value not in "0123456789abcdef" for value in args.expected_opendrive_sha256
    ):
        parser.error("expected-opendrive-sha256 must be a lowercase SHA-256 digest")
    return args


def _prepare_output_root(args: argparse.Namespace) -> Path:
    root = args.output_root.expanduser().resolve()
    if root.exists() and not root.is_dir():
        raise MapGenerationError(f"output root is not a directory: {root}")
    existing = list(root.iterdir()) if root.is_dir() else []
    if existing and not args.resume:
        raise MapGenerationError(
            f"output root is non-empty; pass --resume only for this exact scan: {root}"
        )
    if existing and args.resume and not (root / "scan_plan.json").is_file():
        allowed_partial = {".semantic_lidar_collector.lock", "inputs"}
        unexpected = sorted(path.name for path in existing if path.name not in allowed_partial)
        if unexpected:
            raise MapGenerationError(
                "resume output has no scan_plan.json and contains unrelated files: "
                f"{unexpected}"
            )
    root.mkdir(parents=True, exist_ok=True)
    return root


def _load_carla() -> Any:
    try:
        return importlib.import_module("carla")
    except ModuleNotFoundError as error:
        raise MapGenerationError(
            "CARLA Python API is unavailable; source scripts/e2e/env.sh first"
        ) from error


@contextmanager
def _exclusive_collection_locks(output_root: Path, host: str, port: int):
    lock_path = output_root / ".semantic_lidar_collector.lock"
    if host not in {"127.0.0.1", "localhost", "::1"}:
        raise MapGenerationError("CARLA endpoint lock requires an audited loopback host")
    endpoint_digest = sha256_bytes(f"local:{port}".encode("utf-8"))[:16]
    world_lock_path = Path(tempfile.gettempdir()) / (
        f"autoware_e2e_semantic_lidar_world_{endpoint_digest}.lock"
    )
    with lock_path.open("a+", encoding="utf-8") as stream, world_lock_path.open(
        "a+", encoding="utf-8"
    ) as world_stream:
        try:
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise MapGenerationError(
                f"another semantic-LiDAR collector owns output root {output_root}"
            ) from error
        try:
            try:
                fcntl.flock(world_stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error:
                raise MapGenerationError(
                    f"another semantic-LiDAR collector owns CARLA endpoint {host}:{port}"
                ) from error
            try:
                yield
            finally:
                fcntl.flock(world_stream.fileno(), fcntl.LOCK_UN)
        finally:
            fcntl.flock(stream.fileno(), fcntl.LOCK_UN)


def run(args: argparse.Namespace) -> dict[str, Any]:
    output_root = _prepare_output_root(args)
    with _exclusive_collection_locks(output_root, args.host, args.port):
        return _run_locked(args, output_root)


def _run_locked(args: argparse.Namespace, output_root: Path) -> dict[str, Any]:
    lanelet_map = args.lanelet_map.expanduser().resolve()
    route_root = args.route_root.expanduser().resolve()
    carla_content_root = args.carla_content_root.expanduser().resolve()
    if not lanelet_map.is_file():
        raise MapGenerationError(f"Lanelet2 map is absent: {lanelet_map}")
    if not route_root.is_dir():
        raise MapGenerationError(f"route root is absent: {route_root}")
    if not carla_content_root.is_dir():
        raise MapGenerationError(f"CARLA content root is absent: {carla_content_root}")

    source_routes = route_inventory(route_root, args.expected_map)
    cooked_inventory = cooked_asset_inventory(
        carla_content_root, args.cooked_asset_token
    )
    snapshots = snapshot_inputs(
        output_root,
        lanelet_map,
        route_root,
        source_routes,
        cooked_inventory,
    )

    carla = _load_carla()
    city_object_labels = validate_city_object_labels(carla)
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    client_version = str(client.get_client_version())
    server_version = str(client.get_server_version())
    validate_carla_version_pair(client_version, server_version)
    runtime_binaries = carla_runtime_binary_inventory(carla, carla_content_root)
    server_attestation = attest_local_carla_server(
        args.host, args.port, runtime_binaries["server_executable"]
    )
    world = client.get_world()
    carla_map, opendrive_digest = verify_world_contract(
        world, args.expected_map, args.expected_opendrive_sha256
    )
    actor_preflight = static_scene_actor_audit(world.get_actors())
    if actor_preflight["status"] != "PASS_MAP_BASELINE_ONLY":
        raise MapGenerationError(
            "CARLA world preflight found a runtime/non-map actor; use a dedicated "
            f"cold server: {actor_preflight['blocking_actors'][:5]}"
        )
    scan_plan = build_scan_plan(
        carla_map,
        args.waypoint_spacing_m,
        args.tile_size_m,
        args.z_band_m,
        args.sensor_height_m,
    )
    plan = {
        "schema_version": 1,
        "kind": "carla_semantic_lidar_static_surface_map_scan",
        "created_at_utc": utc_now(),
        "server": {
            "host": args.host,
            "port": args.port,
            "required_version": EXPECTED_CARLA_VERSION,
            "client_version": client_version,
            "server_version": server_version,
        },
        "map": {
            "expected_name": args.expected_map,
            "actual_name": str(carla_map.name),
            "opendrive_sha256": opendrive_digest,
        },
        "coordinate_contract": {
            "collection": "CARLA world (x forward/east, y right/south, z up)",
            "output": "ROS Local map",
            "carla_world_to_ros": "(x, y, z) -> (x, -y, z)",
        },
        "semantic_contract": {
            "carla_0915_city_object_labels": city_object_labels,
            "retained_static_tags": {
                str(value): STATIC_TAG_NAMES.get(value, "explicit-user-static-tag")
                for value in sorted(args.static_tags)
            },
            "excluded_dynamic_tags": {
                str(key): value for key, value in EXCLUDED_DYNAMIC_TAG_NAMES.items()
            },
            "source_geometry": "CARLA semantic-LiDAR ray hits; never Lanelet-synthesized",
        },
        "sensor": {
            "blueprint": "sensor.lidar.ray_cast_semantic",
            "channels": args.channels,
            "range_m": args.range_m,
            "points_per_second": args.points_per_second,
            "rotation_frequency_hz": 1.0 / args.fixed_delta_seconds,
            "horizontal_fov_deg": 360.0,
            "upper_fov_deg": args.upper_fov_deg,
            "lower_fov_deg": args.lower_fov_deg,
            "fixed_delta_seconds": args.fixed_delta_seconds,
            "transform_settle_max_extra_frames": (
                args.transform_settle_max_extra_frames
            ),
            "measurement_transform_acceptance": {
                "translation_max_m": MAX_SOURCE_TRANSLATION_ERROR_M,
                "rotation_max_rad": MAX_SOURCE_ROTATION_ERROR_RAD,
            },
            "determinism": "synchronous noiseless semantic ray cast; no global RNG mutation",
        },
        "output": {
            "voxel_size_m": args.voxel_size_m,
            "bucket_size_m": args.bucket_size_m,
            "pcd_encoding": args.pcd_encoding,
        },
        "lanelet2": snapshots["lanelet2"],
        "routes": snapshots["routes"],
        "implementation": snapshots["implementation"],
        "carla_runtime_binaries": runtime_binaries,
        "cooked_assets": snapshots["cooked_assets"],
        "scan_plan": scan_plan,
        "estimate": _estimate(
            scan_plan["coverage"]["scan_poses"],
            args.points_per_second,
            args.fixed_delta_seconds,
            args.transform_settle_max_extra_frames,
        ),
    }
    plan["fingerprint"] = plan_fingerprint(plan)
    plan_path = output_root / "scan_plan.json"
    if plan_path.is_file():
        try:
            existing = json.loads(plan_path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError) as error:
            raise MapGenerationError(f"cannot read scan plan {plan_path}: {error}") from error
        validate_resume_plan(existing, plan)
        plan = existing
    else:
        atomic_write_json(plan_path, plan)
    verify_pinned_inputs(plan)
    if args.plan_only:
        return {
            "status": "PLAN_ONLY",
            "scan_plan": str(plan_path),
            "fingerprint": plan["fingerprint"],
            "coverage": plan["scan_plan"]["coverage"],
            "estimate": plan["estimate"],
            "local_server_attestation": server_attestation,
        }

    provenance_path = output_root / "provenance.json"
    if provenance_path.is_file():
        try:
            completed_provenance = json.loads(provenance_path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError) as error:
            raise MapGenerationError(f"cannot read provenance {provenance_path}: {error}") from error
        validate_existing_complete_provenance(completed_provenance, output_root, plan)
        return completed_provenance

    stop = {"requested": False}

    def request_stop(_signum: int, _frame: Any) -> None:
        stop["requested"] = True

    previous_handlers = {
        signum: signal.signal(signum, request_stop) for signum in (signal.SIGINT, signal.SIGTERM)
    }
    runtime = None
    try:
        checkpoint, runtime = collect_scans(
            carla,
            world,
            plan,
            output_root,
            args,
            lambda: stop["requested"],
            server_attestation,
        )
    finally:
        for signum, previous in previous_handlers.items():
            signal.signal(signum, previous)

    verify_pinned_inputs(plan)
    output_pcd = output_root / "pointcloud_map.pcd"
    if output_pcd.exists():
        quarantine = output_root / "orphaned_outputs"
        quarantine.mkdir(exist_ok=True)
        digest = sha256_file(output_pcd)
        destination = quarantine / f"pointcloud_map.{digest[:12]}.orphan.pcd"
        suffix = 1
        while destination.exists():
            destination = quarantine / f"pointcloud_map.{digest[:12]}.{suffix}.orphan.pcd"
            suffix += 1
        os.replace(output_pcd, destination)
    pcd = merge_chunks_to_pcd(
        checkpoint,
        output_root,
        output_pcd,
        args.voxel_size_m,
        args.bucket_size_m,
        args.pcd_encoding,
    )
    verify_pinned_inputs(plan)
    qa = run_complete_qa(output_pcd, plan, checkpoint)
    verify_pinned_inputs(plan)
    status = "COMPLETE_QA_PASS" if qa["status"] == "PASS" else "COMPLETE_QA_FAIL"
    provenance = {
        "schema_version": 1,
        "status": status,
        "completed_at_utc": utc_now(),
        "truth_claim": (
            "The PCD contains voxelized static semantic-LiDAR ray hits from the verified map, "
            "OpenDRIVE, and Town-named cooked-asset inventory. Shared generic CARLA assets "
            "are recorded as outside that dependency inventory. This proves map generation/"
            "alignment only, not an Autoware VAD drive."
        ),
        "scan_plan": {
            "path": str(plan_path),
            "sha256": sha256_file(plan_path),
            "fingerprint": plan["fingerprint"],
        },
        "checkpoint": {
            "path": str(output_root / "checkpoint.json"),
            "sha256": sha256_file(output_root / "checkpoint.json"),
            "completed_scans": len(checkpoint["completed_scans"]),
        },
        "runtime": runtime,
        "pointcloud": {"path": str(output_pcd), **pcd},
        "qa": qa,
    }
    atomic_write_json(provenance_path, provenance)
    if qa["status"] != "PASS":
        raise MapGenerationError(
            f"PCD was preserved for audit but global Lanelet/route QA failed: {output_pcd}"
        )
    return provenance


def main(argv: Sequence[str] | None = None) -> int:
    try:
        args = parse_args(argv)
        result = run(args)
    except InterruptedGeneration as error:
        print(f"INTERRUPTED: {error}", file=sys.stderr)
        return 130
    except (MapGenerationError, OSError, subprocess.CalledProcessError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
