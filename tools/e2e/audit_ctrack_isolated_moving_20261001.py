#!/usr/bin/env python3
"""HH_261001 - Audit isolated CTrack external diagnostic captures; never admit TRAIN."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re

import numpy as np
from PIL import Image


ROOT = Path(__file__).resolve().parents[2]
ROUTES = {
    "straight": {
        "path": ROOT / ("storage/experiments/validation/2026-09-01/"
                        "autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/"
                        "c_track_1_0_7/catalog/straight/routes/c_track_1_0_7/straight/"
                        "c_track_1_0_7_straight_s0000_p03.json"),
        "sha256": "cb8632fa48598389a948fe552826ab05a0d2c737126dd3f39e4dde12d5e3b70e",
        "road_option": "STRAIGHT",
    },
    "left": {
        "path": ROOT / ("storage/experiments/training/2026-09-08/turn_launch_matrix_v1/"
                        "c_track_left/turn_launch_013_v1/run_002/episode/route.json"),
        "sha256": "2299e5bf2bc86789650da72336fdcfe7c11c585df0b9e0a0fe29a44730bae79b",
        "road_option": "LEFT",
    },
}
TOWN = "C_track_1_0_7"
LABEL_TABLE = {1: "road", 24: "lane_marking", 2: "sidewalk",
               7: "traffic_light_fixture", 8: "traffic_sign", 6: "pole"}
CAPTURE_MODE = "mapping-ctrack-v2"
CAPTURE_OPERATION = "owned_train_capture_pending_qa"
WORKER_SOURCE = "scripts/e2e/collect_carla_mapping_ctrack_v2.py"
V2_REGISTRY_SOURCE = "config/site_split_guard_20260929_v2.json"
V1_COLLECTOR_SOURCE = "scripts/e2e/collect_carla_mapping_probe.py"
PINNED_V1_COLLECTOR_SHA256 = (
    "2fdfa553329f0224f49da7fc32996ea2fe5b3210f70df8dc0f68caeb3d679e4a"
)
PINNED_CTRACK_XODR_SHA256 = (
    "3f760b8ec8cb117a975c9631a74948b620c991aeaef5c1b07497ebb37ec7be27"
)
PINNED_CTRACK_UMAP_SHA256 = (
    "d44c26a3e7fae4e05cd01cc12022e96daf7da5b4650797718442b143743e72a7"
)
REQUIRED_OWNER_SOURCES = frozenset({
    "scripts/e2e/run_owned_carla_expert_trial.sh", WORKER_SOURCE,
    V2_REGISTRY_SOURCE, V1_COLLECTOR_SOURCE,
    "scripts/e2e/site_split_guard_v2.py", "scripts/e2e/site_split_guard.py",
    "config/site_split_guard_20260929.json",
    "portable_e2e/mapping.py", "portable_e2e/mapping_projection.py",
})


@dataclass(frozen=True)
class Policy:
    """HH_261001 - Conservative source-integrity gates, not model quality targets."""

    expected_frames: int = 240
    frame_wh: tuple[int, int] = (640, 360)
    minimum_retained_0p25m: int = 120
    minimum_lane_positive_frames: int = 24
    minimum_road_fraction: float = 0.01
    minimum_lane_fraction: float = 0.0001
    minimum_valid_depth_fraction_per_frame: float = 0.05
    maximum_route_offset_m: float = 5.0
    minimum_window_frames: int = 3
    window_margin_m: float = 2.0
    maximum_station_regression_m_per_frame: float = 2.0
    maximum_station_drawdown_m: float = 3.0
    minimum_window_distinct_positions_0p25m: int = 3
    minimum_window_station_span_m: float = 5.0


def digest(path: Path) -> str:
    if not path.is_file() or path.is_symlink():
        raise ValueError(f"regular file required: {path}")
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


class OwnerEvidenceError(ValueError):
    """HH_261001 - Keep post-capture owner failures machine-readable."""

    def __init__(self, code: str, detail: str):
        super().__init__(detail)
        self.code = code


def _read_owner_object(path: Path, hashes: dict, label: str) -> dict:
    """HH_261001 - Reject missing, symlinked or malformed owner evidence."""
    if path.parent.is_symlink():
        raise OwnerEvidenceError("OWNER_EVIDENCE_MISSING_OR_UNSAFE",
                                 f"{label}: symlinked parent directory")
    try:
        hashes[label] = digest(path)
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError) as error:
        raise OwnerEvidenceError("OWNER_EVIDENCE_MISSING_OR_UNSAFE",
                                 f"{label}: {error}") from error
    if not isinstance(value, dict):
        raise OwnerEvidenceError("OWNER_EVIDENCE_MISSING_OR_UNSAFE",
                                 f"{label}: JSON object required")
    return value


def _owner_provenance(source, route_path, route_sha256, route, summary, town, policy, hashes):
    """HH_261001 - Verify the isolated source editor, archive, role and completed shutdown."""
    root = source.parent
    objects = {name: _read_owner_object(root / filename, hashes, name) for name,filename in (
        ("owner_plan","owner_plan.json"),("owner_started","owner_started.json"),
        ("owner_ready","ready.json"),("owner_result","owner_result.json"),
        ("owner_cleanup","cleanup_result.json"),("pre_actor","capture/pre_actor_provenance.json"))}
    plan,started,ready,result,cleanup,pre = (objects[name] for name in (
        "owner_plan","owner_started","owner_ready","owner_result","owner_cleanup","pre_actor"))
    role = "external_map_diagnostic_evaluation"
    if (plan.get("scenario") != route["scenario"] or plan.get("route_path") != str(route_path)
            or plan.get("route_sha256") != route_sha256 or plan.get("intended_role") != role
            or summary.get("intended_role") != role or result.get("intended_role") != role
            or any(item.get("dataset_admitted") is not False or item.get("train_eligible") is not False
                   for item in (plan,summary,result))):
        raise OwnerEvidenceError("OWNER_ROLE_ROUTE", "external diagnostic role or pinned route differs")
    if (result.get("status") != "PASS" or result.get("cleanup_errors") != []
            or result.get("source_fingerprints_unchanged") is not True
            or cleanup.get("status") != "PASS"
            or cleanup.get("owned_processes_stopped_and_ports_released") is not True):
        raise OwnerEvidenceError("OWNER_LIFECYCLE", "owner or drained sandbox cleanup did not pass")
    expected_map = "/Game/map_package/Maps/C_track_1_0_7/C_track_1_0_7"
    if (ready.get("status") != "PASS" or ready.get("map_name") not in
            (expected_map,expected_map.removeprefix("/Game/"))
            or ready.get("xodr_sha256") != PINNED_CTRACK_XODR_SHA256
            or pre.get("live_map_name") != ready["map_name"]
            or pre.get("live_xodr_sha256") != ready["xodr_sha256"]
            or pre.get("owner_pid") != ready["editor"]["pid"]
            or pre.get("owner_pgid") != started["server_pgid"]
            or pre.get("owner_generation_start_ticks") != ready["editor"]["start_ticks"]):
        raise OwnerEvidenceError("OWNER_MAP_GENERATION", "source-map identity or owned generation differs")
    sources = plan.get("source_provenance", {})
    required = {"run_owned_ctrack_moving_20261001.py","collect_ctrack_isolated_moving_20261001.py",
                "prepare_ctrack_isolated_probe_20261001.py","ctrack_sandbox_cleanup_20261001.py",
                "mapping.py","mapping_projection.py","basic_agent.py","local_planner.py","controller.py"}
    if not sources or not required <= {Path(path).name for path in sources}:
        raise OwnerEvidenceError("OWNER_SOURCES", "required source archive is incomplete")
    for name,entry in sources.items():
        archive = Path(entry["archived_path"])
        if not archive.is_relative_to(root / "provenance") or digest(archive) != entry["sha256"]:
            raise OwnerEvidenceError("OWNER_SOURCES", "archived source hash differs: " + name)
        if digest(Path(name)) != entry["sha256"]:
            raise OwnerEvidenceError("OWNER_SOURCES", "current source hash differs: " + name)
    if summary.get("rgb_enable_postprocess_effects") is not True:
        raise OwnerEvidenceError("RGB_RENDER_MODE", "lit RGB postprocess must be enabled")
    assets = summary.get("source_assets", {})
    expected_source_map = "/home/a/carla/Unreal/CarlaUE4/Content/map_package/Maps/C_track_1_0_7/C_track_1_0_7.umap"
    if assets.get(expected_source_map) != "775739b54fe5b6eecf2b2cae6f463d3c9583336e89911b882b9a9281d27407fe":
        raise OwnerEvidenceError("OWNER_MAP_BYTES", "staging map is not pinned")
    for name,sha in assets.items():
        if digest(Path(name)) != sha:
            raise OwnerEvidenceError("OWNER_MAP_BYTES", "source or preserved package asset changed")
    return {"owner_generation_id":f"source_{ready['editor']['pid']}_{ready['editor']['start_ticks']}",
            "archived_source_count":len(sources),"live_xodr_sha256":ready["xodr_sha256"],
            "intended_role":role,"independent_dev_approved":False}


def _fail(report: dict, code: str, detail: str) -> None:
    report["fail_reasons"].append({"code": code, "detail": detail})


def _finish(report: dict) -> dict:
    report["status"] = "FAIL" if report["fail_reasons"] else "QA_CONDITIONAL"
    return report


def _route_geometry(route: dict, option: str) -> tuple[np.ndarray, np.ndarray, tuple[float, float]]:
    """HH_261001 - Convert ROS route Y to the collector's CARLA UE ego XY frame."""
    rows = route.get("route")
    if not isinstance(rows, list) or len(rows) < 2:
        raise ValueError("route must contain at least two waypoints")
    xy = np.asarray([(row["x"], -row["y"]) for row in rows], dtype=np.float64)
    distance = np.asarray([row["distance_m"] for row in rows], dtype=np.float64)
    chosen = [float(row["distance_m"]) for row in rows if row.get("road_option") == option]
    if not np.isfinite(xy).all() or not np.isfinite(distance).all() or (np.diff(distance) < 0).any():
        raise ValueError("finite monotonic route geometry required")
    if len(chosen) < 2 or max(chosen) - min(chosen) < 5:
        raise ValueError(f"route has no usable {option} window")
    return xy, distance, (min(chosen), max(chosen))


def _nearest_route_s(point: tuple[float, float], xy: np.ndarray,
                     distance: np.ndarray) -> tuple[float, float]:
    """HH_261001 - Project a recorded ego position onto the pinned route polyline."""
    start, delta = xy[:-1], np.diff(xy, axis=0)
    length2 = np.einsum("ij,ij->i", delta, delta)
    valid = length2 > 1e-10
    fraction = np.zeros(len(delta), dtype=np.float64)
    fraction[valid] = np.clip(
        np.einsum("ij,ij->i", np.asarray(point) - start, delta)[valid] / length2[valid],
        0.0, 1.0)
    projected = start + fraction[:, None] * delta
    error2 = np.einsum("ij,ij->i", projected - point, projected - point)
    index = int(np.argmin(error2))
    station = distance[index] + fraction[index] * (distance[index + 1] - distance[index])
    return float(station), float(math.sqrt(error2[index]))


def _depth_m(depth_bgra: np.ndarray) -> np.ndarray:
    """HH_261001 - Decode CARLA packed optical-axis depth in metres."""
    value = depth_bgra.astype(np.float64)
    return (value[..., 2] + 256.0 * value[..., 1] + 65536.0 * value[..., 0]) * (1000.0 / 16777215.0)


def _pose_errors(matrix: np.ndarray, record: dict) -> tuple[float, float] | None:
    """HH_261001 - Check rigid camera pose against the collector's 1.5 m front mount."""
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        return None
    rotation = matrix[:3, :3]
    if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-5)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-4)):
        return None
    yaw = math.radians(record["ego_yaw_deg"])
    expected = np.array([record["ego_x"] + 1.5 * math.cos(yaw),
                         record["ego_y"] + 1.5 * math.sin(yaw)])
    xy_error = float(np.linalg.norm(matrix[:2, 3] - expected))
    camera_yaw = math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]))
    yaw_error = abs((camera_yaw - record["ego_yaw_deg"] + 180.0) % 360.0 - 180.0)
    return xy_error, yaw_error


def _retained_count(positions: list[tuple[float, float]]) -> int:
    """HH_261001 - Retain a position only if every prior retained point is >=0.25 m away."""
    kept: list[tuple[float, float]] = []
    for xy in positions:
        if all(math.dist(xy, old) >= 0.25 for old in kept):
            kept.append(xy)
    return len(kept)


def _station_regression(stations: list[float]) -> tuple[float, float]:
    """HH_261001 - Catch one large backward step and sustained small backward steps."""
    maximum_step = maximum_drawdown = 0.0
    peak = stations[0] if stations else 0.0
    for previous, current in zip(stations, stations[1:]):
        maximum_step = max(maximum_step, previous - current)
        peak = max(peak, current)
        maximum_drawdown = max(maximum_drawdown, peak - current)
    return maximum_step, maximum_drawdown


def verify_sensor_metadata(record, rgb, depth, semantic, matrix):
    """HH_261001 - Replay each sensor's frame, stamp, transform and raw-byte digest."""
    metadata = record.get("sensors", {})
    if set(metadata) != {"rgb","depth","semantic_segmentation"}:
        raise ValueError("three per-sensor metadata records required")
    actor_ids = set()
    for kind,raw in (("rgb",rgb),("depth",depth),("semantic_segmentation",semantic)):
        item = metadata[kind]
        actor_ids.add(item["actor_id"])
        pose = np.asarray(item["world_from_camera"],dtype=float)
        if (item["frame"] != record["frame"] or
                abs(item["timestamp_s"] - record["timestamp_s"]) > 1e-6 or
                (item["width"],item["height"]) != (640,360) or pose.shape != (4,4) or
                not np.isfinite(pose).all() or not np.allclose(pose,matrix,atol=1e-5,rtol=0) or
                hashlib.sha256(raw.tobytes()).hexdigest() != item["raw_bgra_sha256"]):
            raise ValueError("per-sensor archived identity mismatch: " + kind)
    if len(actor_ids) != 3:
        raise ValueError("sensor actor IDs must be distinct")


def audit_capture(source: Path, route_path: Path, route_sha256: str, option: str, *,
                  town: str = TOWN, policy: Policy = Policy()) -> dict:
    """HH_261001 - Report technical QA only; source role and human render review stay pending."""
    report = {
        "schema": "portable_e2e.ctrack_isolated_external_capture_qa.v1",
        "status": "FAIL", "source": str(source), "route": str(route_path),
        "expected_route_sha256": route_sha256, "town": town, "road_option": option,
        "fail_reasons": [], "metrics": {}, "input_sha256": {},
        "independent_per_sensor_timestamp_pose_replay": True,
        "intended_role": "external_map_diagnostic_evaluation",
        "manual_full_fov_render_review_required": True,
        "capture_authorized": False, "dataset_admitted": False,
        "train_eligible": False,
        "independent_dev_approved": False, "hd_map_approved": False,
        "model_predictions_made": False, "gpu_used": False,
    }
    if not source.is_dir() or source.is_symlink():
        _fail(report, "SOURCE_MISSING_OR_UNSAFE", "regular capture directory required")
        return _finish(report)
    # HH_261001 - C-track branding is restricted to the two reviewed route byte identities.
    if town == TOWN and not any(
            route_sha256 == spec["sha256"] and option == spec["road_option"]
            for spec in ROUTES.values()):
        _fail(report, "CTRACK_ROUTE_NOT_PINNED", "C-track route is outside the two reviewed candidates")
        return _finish(report)
    if town == TOWN and policy != Policy():
        _fail(report, "CTRACK_POLICY_OVERRIDE", "C-track QA thresholds and 240-frame contract are fixed")
        return _finish(report)
    try:
        actual_route_sha = digest(route_path)
        report["input_sha256"]["route"] = actual_route_sha
        route = json.loads(route_path.read_text(encoding="utf-8"))
        if actual_route_sha != route_sha256 or route.get("town") != town or option != (
                "STRAIGHT" if route.get("scenario") == "straight" else "LEFT") or (
                route.get("coordinate_reference") != "base_link"):
            raise ValueError("route bytes, town, scenario or coordinate frame differ")
        xy, route_distance, window = _route_geometry(route, option)
    except (OSError, ValueError, KeyError, TypeError) as error:
        _fail(report, "ROUTE_CONTRACT", str(error))
        return _finish(report)
    try:
        summary_path, records_path = source / "summary.json", source / "records.json"
        report["input_sha256"]["summary"] = digest(summary_path)
        report["input_sha256"]["records"] = digest(records_path)
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        records = json.loads(records_path.read_text(encoding="utf-8"))
        if not isinstance(summary, dict) or not isinstance(records, list):
            raise ValueError("summary object and records array required")
    except (OSError, ValueError, TypeError) as error:
        _fail(report, "CAPTURE_MANIFEST", str(error))
        return _finish(report)
    # HH_261001 - Collector completion is necessary because per-sensor sync bytes were not saved.
    checks = {
        "CAPTURE_INCOMPLETE": summary.get("status") == "COMPLETE_ORACLE_PLUMBING_ONLY",
        "LIVE_REPLAY_MISMATCH": summary.get("live_replay_exact_match") is True,
        "COLLISION_EVENTS": summary.get("collision_events") == 0,
        "CAPTURE_FRAME_COUNT": summary.get("frame_count") == policy.expected_frames == len(records),
        "CAPTURE_WORLD_ROUTE": summary.get("town") == town
                               and summary.get("route_sha256") == route_sha256,
        "CAPTURE_NOT_PHYSICAL_AGENT": summary.get("physics_driving") is True
                                      and summary.get("motion_mode") == "basic-agent"
                                      and summary.get("controller") == "CARLA BasicAgent"
                                      and summary.get("learned_model_control") is False,
        "SENSOR_PROVENANCE": all(summary.get(name) is True for name in
                                 ("oracle_pose", "oracle_depth", "oracle_semantics"))
                             and summary.get("label_table") == {str(k): v for k, v in LABEL_TABLE.items()},
        "CAPTURE_CLEANUP": summary.get("cleanup_errors") == [],
        "TRAFFIC_POLICY_UNDECLARED": isinstance(summary.get("ignores_traffic_lights"), bool),
    }
    for code, passed in checks.items():
        if not passed:
            _fail(report, code, "collector summary fails required source contract")
    try:
        report["owner_provenance"] = _owner_provenance(
            source, route_path, route_sha256, route, summary, town, policy,
            report["input_sha256"])
    except OwnerEvidenceError as error:
        _fail(report, error.code, str(error))
        return _finish(report)
    if len(records) != policy.expected_frames:
        return _finish(report)
    frame_dir = source / "frames"
    if not frame_dir.is_dir() or frame_dir.is_symlink():
        _fail(report, "FRAME_DIR_MISSING_OR_UNSAFE", "regular frames directory required")
        return _finish(report)
    if (len(list(frame_dir.glob("*.png"))) != policy.expected_frames
            or len(list(frame_dir.glob("*.npz"))) != policy.expected_frames):
        _fail(report, "FRAME_PAIR_COUNT", "exact native PNG/NPZ pair count required")
    total_road = total_lane = lane_positive = total_valid_depth = 0
    positions, stations, route_offsets, camera_xy_errors, camera_yaw_errors = [], [], [], [], []
    frame_hashes = {}
    previous_time = previous_frame = None
    previous_position = None
    traveled = 0.0
    width, height = policy.frame_wh
    for index, record in enumerate(records):
        stem = f"frames/{index:04d}"
        if not isinstance(record, dict) or record.get("stem") != stem:
            _fail(report, "RECORD_STEM", f"record {index}: sequential stem required")
            continue
        values = [record.get(name) for name in
                  ("timestamp_s", "ego_x", "ego_y", "ego_yaw_deg", "speed_mps")]
        if (not all(isinstance(value, (int, float)) and math.isfinite(value) for value in values)
                or values[-1] < 0 or not isinstance(record.get("frame"), int)
                or isinstance(record.get("frame"), bool)):
            _fail(report, "RECORD_NUMERIC", f"record {index}: finite timestamp/ego/frame/speed required")
            continue
        stamp, ego_x, ego_y, _, _ = values
        if previous_frame is not None and record["frame"] != previous_frame + 1:
            _fail(report, "FRAME_ID_CADENCE", f"record {index}: consecutive CARLA frame IDs required")
        if previous_time is not None and abs((stamp - previous_time) - .1) > .001:
            _fail(report, "TIMESTAMP_CADENCE", f"record {index}: 10 Hz timestamp interval required")
        previous_time, previous_frame = stamp, record["frame"]
        position = (float(ego_x), float(ego_y))
        if previous_position is not None:
            step = math.dist(previous_position, position)
            traveled += step
            if step > 5.0:
                _fail(report, "MOTION_TELEPORT", f"record {index}: {step:.3f} m per 0.1 s")
        previous_position = position
        positions.append(position)
        station, offset = _nearest_route_s(position, xy, route_distance)
        stations.append(station)
        route_offsets.append(offset)
        if offset > policy.maximum_route_offset_m:
            _fail(report, "OFF_ROUTE", f"record {index}: {offset:.3f} m from pinned route")
        png_path, npz_path = source / (stem + ".png"), source / (stem + ".npz")
        try:
            frame_hashes[stem + ".png"] = digest(png_path)
            frame_hashes[stem + ".npz"] = digest(npz_path)
            with Image.open(png_path) as image:
                image.load()
                decoded_rgb = np.asarray(image)
                if image.mode != "RGB" or image.size != (width, height):
                    raise ValueError("RGB native shape/mode differs")
            with np.load(npz_path, allow_pickle=False) as sensor:
                if set(sensor.files) != {"rgb", "depth", "semantic", "world_from_camera"}:
                    raise ValueError("three raw BGRA sensors and camera matrix required")
                rgb = sensor["rgb"]
                depth, semantic, matrix = (sensor[name] for name in
                                           ("depth", "semantic", "world_from_camera"))
            if (depth.shape != (height, width, 4) or semantic.shape != depth.shape
                    or depth.dtype != np.uint8 or semantic.dtype != np.uint8):
                raise ValueError("native uint8 BGRA depth/semantic required")
            if rgb.shape != depth.shape or rgb.dtype != np.uint8 or not np.array_equal(
                    decoded_rgb, rgb[..., [2, 1, 0]]):
                raise ValueError("PNG differs from raw RGB BGRA")
            verify_sensor_metadata(record, rgb, depth, semantic, matrix)
            if any(record.get("artifact_sha256", {}).get(suffix) != frame_hashes[stem + suffix]
                   for suffix in (".png", ".npz")):
                raise ValueError("per-frame artifact hash differs from record")
            top = rgb[:32]
            warning_pixels = int(np.count_nonzero((top[...,2] >= 130) &
                                                  (top[...,1] >= 120) & (top[...,0] < 100)))
            if warning_pixels >= 100:
                _fail(report, "RGB_SKY_WARNING", f"record {index}: {warning_pixels} yellow warning pixels")
            if record.get("rgb_quality",{}).get("yellow_top32_pixels") != warning_pixels:
                raise ValueError("stored RGB warning count differs from pixels")
            pose_errors = _pose_errors(matrix, record)
            if pose_errors is None:
                _fail(report, "POSE_NOT_RIGID", f"record {index}: invalid camera transform")
            else:
                camera_xy_errors.append(pose_errors[0])
                camera_yaw_errors.append(pose_errors[1])
                if pose_errors[0] > .3 or pose_errors[1] > 5.0:
                    _fail(report, "POSE_MOUNT_MISMATCH", f"record {index}: camera/ego mount differs")
            meters = _depth_m(depth)
            valid_depth = np.isfinite(meters) & (meters > .1) & (meters < 40.0)
            valid_count = int(np.count_nonzero(valid_depth))
            total_valid_depth += valid_count
            if valid_count / (width * height) < policy.minimum_valid_depth_fraction_per_frame:
                _fail(report, "DEPTH_INVALID", f"record {index}: too few finite 0.1-40 m pixels")
            tags = semantic[..., 2]
            road = int(np.count_nonzero(tags == 1))
            lane = int(np.count_nonzero(tags == 24))
            total_road += road
            total_lane += lane
            lane_positive += lane > 0
            sampled_tags = tags[::3, ::3]
            sampled_depth = valid_depth[::3, ::3]
            projected_count = int(np.count_nonzero(
                sampled_depth & np.isin(sampled_tags, tuple(LABEL_TABLE))))
            if record.get("points") != projected_count:
                _fail(report, "SENSOR_RECORD_POINTS", f"record {index}: saved sensor point count differs")
        except (OSError, ValueError, KeyError, TypeError) as error:
            _fail(report, "FRAME_DECODE", f"record {index}: {error}")
    report["input_sha256"]["frame_pairs"] = frame_hashes
    if len(positions) != policy.expected_frames:
        _fail(report, "MOTION_RECORDS_INCOMPLETE", "not all ego positions were valid")
        return _finish(report)
    retained = _retained_count(positions)
    road_fraction = total_road / (policy.expected_frames * width * height)
    lane_fraction = total_lane / (policy.expected_frames * width * height)
    valid_depth_fraction = total_valid_depth / (policy.expected_frames * width * height)
    start, end = window
    before = [i for i, station in enumerate(stations) if station < start - policy.window_margin_m]
    inside = [i for i, station in enumerate(stations) if start <= station <= end]
    after = [i for i, station in enumerate(stations) if station > end + policy.window_margin_m]
    ordered = bool(before and inside and after and before[0] < inside[0] < after[0])
    # HH_261001 - First passage is insufficient if the drive later reverses along the route.
    maximum_regression, maximum_drawdown = _station_regression(stations)
    inside_distinct = _retained_count([positions[index] for index in inside])
    inside_span = max((stations[index] for index in inside), default=0.0) - min(
        (stations[index] for index in inside), default=0.0)
    report["metrics"] = {
        "raw_frames": len(records), "retained_positions_0p25m": retained,
        "travel_from_ego_xy_m": traveled,
        "summary_travel_m": summary.get("travel_m"),
        "road_pixels": total_road, "road_fraction": road_fraction,
        "lane_pixels": total_lane, "lane_fraction": lane_fraction,
        "lane_positive_frames": lane_positive,
        "finite_depth_0p1_to_40m_fraction": valid_depth_fraction,
        "route_option_window_m": [start, end],
        "before_window_frames": len(before), "inside_window_frames": len(inside),
        "after_window_frames": len(after), "ordered_window_coverage": ordered,
        "inside_window_distinct_positions_0p25m": inside_distinct,
        "inside_window_station_span_m": inside_span,
        "maximum_backward_route_station_step_m": maximum_regression,
        "maximum_route_station_drawdown_m": maximum_drawdown,
        "maximum_route_offset_m": max(route_offsets),
        "maximum_camera_ego_xy_error_m": max(camera_xy_errors, default=None),
        "maximum_camera_ego_yaw_error_deg": max(camera_yaw_errors, default=None),
    }
    if retained < policy.minimum_retained_0p25m:
        _fail(report, "INSUFFICIENT_DISTINCT_POSITIONS", f"{retained} retained positions")
    if not isinstance(summary.get("travel_m"), (int, float)) or not math.isfinite(summary["travel_m"]) or (
            abs(traveled - summary["travel_m"]) > max(.1, .001 * traveled)):
        _fail(report, "TRAVEL_SUMMARY_MISMATCH", "collector travel differs from recorded ego XY")
    if road_fraction < policy.minimum_road_fraction:
        _fail(report, "ROAD_LABEL_COVERAGE", f"road fraction {road_fraction:.6f}")
    if (lane_fraction < policy.minimum_lane_fraction
            or lane_positive < policy.minimum_lane_positive_frames):
        _fail(report, "LANE_LABEL_COVERAGE", f"lane fraction {lane_fraction:.6f}; frames {lane_positive}")
    if not before:
        _fail(report, "ROUTE_WINDOW_MISSING_BEFORE", "no actual ego position before route option")
    if len(inside) < policy.minimum_window_frames:
        _fail(report, "ROUTE_WINDOW_MISSING_INSIDE", "too few actual ego positions in route option")
    if inside_distinct < policy.minimum_window_distinct_positions_0p25m:
        _fail(report, "ROUTE_WINDOW_STATIONARY", "too few distinct ego positions inside route option")
    if inside_span < policy.minimum_window_station_span_m:
        _fail(report, "ROUTE_WINDOW_STATION_SPAN", "actual route-option travel spans less than 5 m")
    if not after:
        _fail(report, "ROUTE_WINDOW_MISSING_AFTER", "no actual ego position after route option")
    if before and inside and after and not ordered:
        _fail(report, "ROUTE_WINDOW_WRONG_ORDER", "ego positions do not traverse option in order")
    if maximum_regression > policy.maximum_station_regression_m_per_frame:
        _fail(report, "ROUTE_STATION_REGRESSION",
              f"backward route station step {maximum_regression:.3f} m exceeds 2 m")
    if maximum_drawdown > policy.maximum_station_drawdown_m:
        _fail(report, "ROUTE_STATION_DRAWDOWN",
              f"route station falls {maximum_drawdown:.3f} m behind its prior peak")
    return _finish(report)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--scenario", choices=sorted(ROUTES), required=True)
    parser.add_argument("--output", type=Path, help="Fresh JSON report path outside the source")
    args = parser.parse_args()
    spec = ROUTES[args.scenario]
    result = audit_capture(args.source, spec["path"], spec["sha256"], spec["road_option"])
    if args.output:
        if args.output.exists() or args.output.is_symlink() or (
                args.source.resolve() in args.output.resolve().parents):
            parser.error("output must be fresh and outside the capture source")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            json.dump(result, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
    print(json.dumps({"status": result["status"], "fail_reasons": result["fail_reasons"],
                      "metrics": result["metrics"], "dataset_admitted": False}))
    raise SystemExit(0 if result["status"] == "QA_CONDITIONAL" else 2)


if __name__ == "__main__":
    main()
