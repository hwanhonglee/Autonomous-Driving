from __future__ import annotations

from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import shutil
import struct
import sys

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/e2e/generate_carla_semantic_lidar_map.py"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "generate_carla_semantic_lidar_map", SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class Location:
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z


class Rotation:
    def __init__(self, yaw: float):
        self.yaw = yaw


class Transform:
    def __init__(self, x: float, y: float, z: float, yaw: float):
        self.location = Location(x, y, z)
        self.rotation = Rotation(yaw)


class Waypoint:
    def __init__(
        self,
        road_id: int,
        section_id: int,
        lane_id: int,
        s: float,
        x: float,
        y: float,
        z: float,
        yaw: float = 0.0,
    ):
        self.road_id = road_id
        self.section_id = section_id
        self.lane_id = lane_id
        self.s = s
        self.transform = Transform(x, y, z, yaw)


class CarlaMap:
    def __init__(self, waypoints):
        self.waypoints = list(waypoints)
        self.requested_spacing = None

    def generate_waypoints(self, spacing):
        self.requested_spacing = spacing
        return list(self.waypoints)


class Actor:
    def __init__(self, actor_id: int, type_id: str, role_name: str = ""):
        self.id = actor_id
        self.type_id = type_id
        self.attributes = {"role_name": role_name}


def test_scan_plan_is_deterministic_and_covers_every_lane_segment():
    module = load_module()
    waypoints = [
        Waypoint(1, 0, 1, 0.0, 0.0, 0.0, 2.0, 10.0),
        Waypoint(1, 0, 1, 9.0, 9.0, 0.0, 2.0, 20.0),
        Waypoint(1, 0, 1, 15.0, 15.0, 0.0, 2.0, 30.0),
        Waypoint(2, 0, -1, 0.0, 1.0, 0.0, 2.0, 180.0),
    ]
    forward_map = CarlaMap(waypoints)
    reverse_map = CarlaMap(reversed(waypoints))

    expected = module.build_scan_plan(forward_map, 2.0, 10.0, 5.0, 3.5)
    actual = module.build_scan_plan(reverse_map, 2.0, 10.0, 5.0, 3.5)

    assert actual == expected
    assert forward_map.requested_spacing == pytest.approx(2.0)
    assert expected["coverage"]["candidate_waypoints"] == 4
    assert expected["coverage"]["lane_segment_count"] == 2
    assert expected["coverage"]["lane_segments_with_pose"] == 2
    assert {
        tuple(pose["representative_lane"]) for pose in expected["poses"]
    } == {(1, 0, 1), (2, 0, -1)}
    assert [pose["scan_index"] for pose in expected["poses"]] == list(
        range(len(expected["poses"]))
    )
    assert all(
        pose["sensor_z_m"] == pytest.approx(pose["road_z_m"] + 3.5)
        for pose in expected["poses"]
    )


def test_scan_plan_deduplicates_a_physical_pose_without_losing_lane_provenance():
    module = load_module()
    carla_map = CarlaMap(
        [
            Waypoint(1, 0, 1, 5.0, 5.0, 5.0, 0.0),
            Waypoint(2, 0, -1, 5.0, 5.0, 5.0, 0.0),
        ]
    )

    plan = module.build_scan_plan(carla_map, 2.0, 10.0, 5.0, 2.0)

    assert plan["coverage"]["scan_poses"] == 1
    assert plan["coverage"]["lane_segments_with_pose"] == 2
    assert plan["poses"][0]["source_lanes_at_exact_pose"] == [
        [1, 0, 1],
        [2, 0, -1],
    ]


def test_scan_plan_rejects_empty_map_and_invalid_geometry_parameters():
    module = load_module()
    with pytest.raises(module.MapGenerationError, match="no driving waypoints"):
        module.build_scan_plan(CarlaMap([]), 2.0, 10.0, 5.0, 2.0)
    with pytest.raises(module.MapGenerationError, match="tile_size_m"):
        module.build_scan_plan(CarlaMap([Waypoint(1, 0, 1, 0, 0, 0, 0)]), 2, 0, 5, 2)


def test_semantic_points_are_transformed_filtered_and_reflected_into_ros():
    module = load_module()
    records = np.zeros(4, dtype=module.SEMANTIC_DTYPE)
    records["x"] = [1.0, 10.0, np.nan, -1.0]
    records["y"] = [2.0, 10.0, 0.0, 0.0]
    records["z"] = [3.0, 10.0, 0.0, 2.0]
    records["object_tag"] = [7, 10, 1, 14]
    sensor_to_world = np.asarray(
        [
            [0.0, -1.0, 0.0, 10.0],
            [1.0, 0.0, 0.0, 20.0],
            [0.0, 0.0, 1.0, 30.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    points, stats = module.transform_semantic_points_to_ros(records, sensor_to_world)

    np.testing.assert_allclose(points, [[8.0, -21.0, 33.0], [0.0, -30.0, 40.0]])
    assert points.dtype == np.dtype("float32")
    assert stats["raw_points"] == 4
    assert stats["finite_points"] == 3
    assert stats["retained_static_points"] == 2
    assert stats["filtered_points"] == 2
    assert stats["raw_tag_counts"] == {"1": 1, "7": 1, "10": 1, "14": 1}
    assert stats["explicitly_excluded_dynamic_tags"]["14"] == "Car"


def test_carla_0915_city_object_label_table_is_fail_closed():
    module = load_module()
    expected = {
        "NONE": 0,
        "Roads": 1,
        "Sidewalks": 2,
        "Buildings": 3,
        "Walls": 4,
        "Fences": 5,
        "Poles": 6,
        "TrafficLight": 7,
        "TrafficSigns": 8,
        "Vegetation": 9,
        "Terrain": 10,
        "Sky": 11,
        "Pedestrians": 12,
        "Rider": 13,
        "Car": 14,
        "Truck": 15,
        "Bus": 16,
        "Train": 17,
        "Motorcycle": 18,
        "Bicycle": 19,
        "Static": 20,
        "Dynamic": 21,
        "Other": 22,
        "Water": 23,
        "RoadLines": 24,
        "Ground": 25,
        "Bridge": 26,
        "RailTrack": 27,
        "GuardRail": 28,
        "Any": 255,
    }
    label_type = type("CityObjectLabel", (), expected)
    fake_carla = type("Carla", (), {"CityObjectLabel": label_type})

    assert module.validate_city_object_labels(fake_carla) == expected
    assert module.DEFAULT_STATIC_TAGS == frozenset(
        {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 20, 22, 23, 24, 25, 26, 27, 28}
    )
    assert set(module.EXCLUDED_DYNAMIC_TAG_NAMES) == {
        12,
        13,
        14,
        15,
        16,
        17,
        18,
        19,
        21,
    }
    with pytest.raises(module.MapGenerationError, match="expected 14"):
        bad_type = type("BadCityObjectLabel", (), {**expected, "Car": 99})
        module.validate_city_object_labels(
            type("BadCarla", (), {"CityObjectLabel": bad_type})
        )
    with pytest.raises(module.MapGenerationError, match="non-static"):
        module._parse_static_tags("1,14")


def test_semantic_transform_rejects_bad_records_and_bad_matrix():
    module = load_module()
    with pytest.raises(module.MapGenerationError, match="incompatible dtype"):
        module.transform_semantic_points_to_ros(
            np.zeros((1, 3), dtype=np.float32), np.eye(4)
        )
    records = np.zeros(1, dtype=module.SEMANTIC_DTYPE)
    matrix = np.eye(4)
    matrix[0, 0] = np.nan
    with pytest.raises(module.MapGenerationError, match="finite 4x4"):
        module.transform_semantic_points_to_ros(records, matrix)


def test_transform_matrix_errors_detect_stale_sensor_rotation():
    module = load_module()
    expected = np.eye(4)
    actual = np.eye(4)
    actual[:3, :3] = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]

    translation, rotation = module.transform_matrix_errors(actual, expected)

    assert translation == 0.0
    assert rotation == pytest.approx(np.pi / 2.0)


class MatrixTransform:
    def __init__(self, matrix):
        self._matrix = matrix

    def get_matrix(self):
        return self._matrix


class SemanticMeasurement:
    def __init__(self, frame, matrix):
        self.frame = frame
        self.transform = MatrixTransform(matrix)


class TickWorld:
    def __init__(self, queue, measurements):
        self.queue = queue
        self.measurements = list(measurements)
        self.tick_count = 0

    def tick(self, _timeout):
        measurement = self.measurements.pop(0)
        self.queue.put(measurement)
        self.tick_count += 1
        return measurement.frame


def _pose_matrix(x, y, z, yaw_deg):
    yaw = np.radians(yaw_deg)
    return np.asarray(
        [
            [np.cos(yaw), -np.sin(yaw), 0.0, x],
            [np.sin(yaw), np.cos(yaw), 0.0, y],
            [0.0, 0.0, 1.0, z],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def test_capture_discards_one_frame_stale_pose_without_relaxing_thresholds():
    module = load_module()
    queue = module.Queue()
    pose38 = _pose_matrix(-52.616287, -50.447510, 2.5, -288.035767)
    pose39 = _pose_matrix(-67.154648, -58.193745, 2.5, -359.403259)
    world = TickWorld(
        queue,
        [
            SemanticMeasurement(14515, pose38),
            SemanticMeasurement(14516, pose39),
        ],
    )
    audits = []

    def audit():
        audits.append(world.tick_count)
        return {"status": "PASS"}

    measurement, matrix, settle = module.capture_transform_settled_measurement(
        world, queue, pose39, 1.0, 2, audit
    )

    assert measurement.frame == 14516
    np.testing.assert_array_equal(matrix, pose39)
    assert settle["status"] == "PASS_MEASUREMENT_TRANSFORM_SETTLED"
    assert settle["capture_attempt_count"] == 2
    assert settle["discarded_stale_frame_count"] == 1
    assert settle["attempts"][0]["source_translation_error_m"] == pytest.approx(
        16.4732539968
    )
    assert np.degrees(
        settle["attempts"][0]["source_rotation_error_rad"]
    ) == pytest.approx(71.367492)
    assert settle["attempts"][0]["accepted"] is False
    assert settle["attempts"][1]["accepted"] is True
    assert audits == [1, 2]


def test_capture_fails_closed_when_measurement_never_settles():
    module = load_module()
    queue = module.Queue()
    expected = np.eye(4)
    outside_contract = np.eye(4)
    outside_contract[0, 3] = module.MAX_SOURCE_TRANSLATION_ERROR_M + 0.001
    world = TickWorld(
        queue,
        [
            SemanticMeasurement(10, outside_contract),
            SemanticMeasurement(11, outside_contract),
        ],
    )

    with pytest.raises(module.MapGenerationError, match="did not settle within 1 extra"):
        module.capture_transform_settled_measurement(
            world, queue, expected, 1.0, 1
        )

    assert world.tick_count == 2


def test_voxel_centroids_are_stable_for_shuffled_input_and_negative_cells():
    module = load_module()
    points = np.asarray(
        [
            [0.1, 0.1, 0.1],
            [0.9, 0.3, 0.5],
            [1.1, 0.0, 0.0],
            [-0.1, -0.2, -0.3],
        ],
        dtype=np.float64,
    )

    expected = module.voxel_centroids(points, 1.0)
    actual = module.voxel_centroids(points[[2, 0, 3, 1]], 1.0)

    np.testing.assert_array_equal(actual, expected)
    np.testing.assert_allclose(
        expected,
        [[-0.1, -0.2, -0.3], [0.5, 0.2, 0.3], [1.1, 0.0, 0.0]],
    )
    assert expected.dtype == np.dtype("float32")
    assert module.voxel_centroids(np.empty((0, 3)), 1.0).shape == (0, 3)
    with pytest.raises(module.MapGenerationError, match="non-finite"):
        module.voxel_centroids([[np.nan, 0.0, 0.0]], 1.0)


def test_binary_pcd_round_trips_exact_little_endian_float32_payload(tmp_path):
    module = load_module()
    path = tmp_path / "nested/map.pcd"
    points = np.asarray([[1.25, -2.5, 3.75], [4.0, 5.0, -6.0]], dtype=np.float64)

    module.write_binary_pcd(path, points)

    payload = path.read_bytes()
    header, binary = payload.split(b"DATA binary\n", 1)
    assert b"FIELDS x y z\n" in header
    assert b"WIDTH 2\n" in header
    assert b"POINTS 2\n" in header
    np.testing.assert_array_equal(
        np.frombuffer(binary, dtype="<f4").reshape(-1, 3), points.astype("<f4")
    )
    inspection = module.inspect_pcd(path)
    assert inspection["points"] == 2
    assert inspection["encoding"] == "binary"
    assert inspection["size_bytes"] == len(payload)
    assert len(inspection["sha256"]) == 64

    with pytest.raises(module.MapGenerationError, match="non-empty shape"):
        module.write_binary_pcd(tmp_path / "empty.pcd", np.empty((0, 3)))


def test_pcd_inspector_rejects_ambiguous_headers_and_bad_compressed_preambles(
    tmp_path,
):
    module = load_module()
    valid = tmp_path / "valid.pcd"
    module.write_binary_pcd(valid, np.asarray([[1.0, 2.0, 3.0]], dtype="<f4"))
    payload = valid.read_bytes()
    compressed_header = payload.split(b"DATA binary\n", 1)[0] + b"DATA binary_compressed\n"
    cases = [
        (
            payload.replace(
                b"VERSION 0.7\n", b"VERSION 0.7\nVERSION 0.7\n", 1
            ),
            "duplicate VERSION",
        ),
        (payload.replace(b"VERSION 0.7\n", b"VERSION 0.6\n", 1), "VERSION must"),
        (payload.replace(b"COUNT 1 1 1\n", b"COUNT 1 1 2\n", 1), "COUNT must"),
        (compressed_header + b"1234567", "truncated size preamble"),
        (compressed_header + struct.pack("<II", 0, 12), "sizes must be positive"),
        (compressed_header + struct.pack("<II", 1, 0) + b"x", "sizes must be positive"),
    ]

    for index, (malformed, message) in enumerate(cases):
        path = tmp_path / f"malformed_{index}.pcd"
        path.write_bytes(malformed)
        with pytest.raises(module.MapGenerationError, match=message):
            module.inspect_pcd(path)


def test_plan_fingerprint_is_canonical_and_resume_validation_is_strict():
    module = load_module()
    proposed = {
        "schema_version": 1,
        "created_at_utc": "later",
        "map": {"name": "Town10HD_Opt", "digest": "abc"},
        "parameters": {"tile_size_m": 10.0, "spacing_m": 2.0},
    }
    reordered = {
        "parameters": {"spacing_m": 2.0, "tile_size_m": 10.0},
        "map": {"digest": "abc", "name": "Town10HD_Opt"},
        "created_at_utc": "earlier",
        "schema_version": 1,
    }
    fingerprint = module.plan_fingerprint(proposed)
    existing = {**reordered, "fingerprint": fingerprint}

    assert fingerprint == module.plan_fingerprint(reordered)
    assert len(fingerprint) == 64
    module.validate_resume_plan(existing, proposed)

    changed = {**proposed, "parameters": {**proposed["parameters"], "tile_size_m": 20.0}}
    with pytest.raises(module.MapGenerationError, match="resume scan plan differs"):
        module.validate_resume_plan(existing, changed)
    with pytest.raises(module.MapGenerationError, match="fingerprint is corrupt"):
        module.validate_resume_plan({**existing, "fingerprint": "0" * 64}, proposed)


def test_checkpoint_resume_quarantines_an_uncommitted_chunk(tmp_path):
    module = load_module()
    chunks = tmp_path / "chunks"
    chunks.mkdir()
    orphan = chunks / "scan_000000.npy"
    np.save(orphan, np.asarray([[1.0, 2.0, 3.0]], dtype="<f4"), allow_pickle=False)
    plan = {"fingerprint": "f" * 64, "scan_plan": {"poses": [{}]}}
    checkpoint_path = tmp_path / "checkpoint.json"

    checkpoint = module._load_checkpoint(checkpoint_path, plan, chunks)

    assert not orphan.exists()
    assert len(checkpoint["recovered_orphans"]) == 1
    recovered = Path(checkpoint["recovered_orphans"][0]["quarantined_path"])
    assert recovered.is_file()
    assert json.loads(checkpoint_path.read_text())["recovered_orphans"]


def _valid_checkpoint_fixture(module, tmp_path):
    chunks = tmp_path / "chunks"
    chunks.mkdir()
    scans = []
    scan_contracts = [
        ({"1": 5, "14": 2}, 7, 5),
        ({"1": 3, "3": 2, "21": 1}, 6, 5),
    ]
    for index, (tag_counts, raw_points, retained_points) in enumerate(scan_contracts):
        chunk_name = f"scan_{index:06d}.npy"
        chunk = chunks / chunk_name
        np.save(
            chunk,
            np.asarray([[index, 0.0, 0.0], [index, 1.0, 0.0]], dtype="<f4"),
            allow_pickle=False,
        )
        settle_attempts = []
        if index == 0:
            settle_attempts.append(
                {
                    "attempt_index": 0,
                    "carla_frame": 99,
                    "source_translation_error_m": 16.4732539968,
                    "source_rotation_error_rad": float(np.radians(71.367492)),
                    "actor_and_server_audit": {
                        "actor_audit": {"status": "PASS_PINNED_ACTOR_SET"},
                        "server_attestation_status": "PASS",
                    },
                    "accepted": False,
                }
            )
        settle_attempts.append(
            {
                "attempt_index": len(settle_attempts),
                "carla_frame": 100 + index,
                "source_translation_error_m": 0.0,
                "source_rotation_error_rad": 0.0,
                "actor_and_server_audit": {
                    "actor_audit": {"status": "PASS_PINNED_ACTOR_SET"},
                    "server_attestation_status": "PASS",
                },
                "accepted": True,
            }
        )
        scans.append(
            {
                "scan_index": index,
                "carla_frame": 100 + index,
                "chunk_file": chunk_name,
                "sha256": module.sha256_file(chunk),
                "raw_points": raw_points,
                "retained_static_points": retained_points,
                "voxel_points": 2,
                "raw_tag_counts": tag_counts,
                "source_translation_error_m": 0.0,
                "source_rotation_error_rad": 0.0,
                "commanded_transform_readback_error_m": 0.0,
                "commanded_transform_readback_rotation_error_rad": 0.0,
                "transform_settle": {
                    "status": "PASS_MEASUREMENT_TRANSFORM_SETTLED",
                    "max_extra_frames": 2,
                    "capture_attempt_count": len(settle_attempts),
                    "discarded_stale_frame_count": len(settle_attempts) - 1,
                    "attempts": settle_attempts,
                },
                "actor_audit": {
                    "before": {"status": "PASS_PINNED_ACTOR_SET"},
                    "after": {"status": "PASS_PINNED_ACTOR_SET"},
                },
            }
        )
    plan = {
        "fingerprint": "f" * 64,
        "scan_plan": {"poses": [{}, {}]},
        "sensor": {"transform_settle_max_extra_frames": 2},
        "semantic_contract": {
            "retained_static_tags": {"1": "Roads", "3": "Buildings"}
        },
    }
    checkpoint = {
        "schema_version": 1,
        "plan_fingerprint": plan["fingerprint"],
        "status": "COLLECTED",
        "completed_scans": scans,
        "tag_totals": {"1": 8, "3": 2, "14": 2, "21": 1},
        "baseline_actor_type_multiset_sha256": "b" * 64,
        "sessions": [
            {
                "session_index": 0,
                "starting_completed_scans": 0,
                "ending_completed_scans": 1,
                "world_settings_restored": True,
                "server_attestation_final_verified": True,
                "cleanup_errors": [],
                "outcome": "INTERRUPTED",
            },
            {
                "session_index": 1,
                "starting_completed_scans": 1,
                "ending_completed_scans": 2,
                "world_settings_restored": True,
                "server_attestation_final_verified": True,
                "cleanup_errors": [],
                "outcome": "PASS",
            },
        ],
        "collection_validity": {"status": "PASS"},
    }
    checkpoint_path = tmp_path / "checkpoint.json"
    checkpoint_path.write_text(json.dumps(checkpoint), encoding="utf-8")
    return checkpoint_path, chunks, plan, checkpoint


def test_checkpoint_accepts_audited_contiguous_sessions_and_semantic_counts(tmp_path):
    module = load_module()
    checkpoint_path, chunks, plan, checkpoint = _valid_checkpoint_fixture(
        module, tmp_path
    )

    loaded = module._load_checkpoint(checkpoint_path, plan, chunks)

    assert loaded == checkpoint


def test_checkpoint_rejects_structural_semantic_and_frame_corruption(tmp_path):
    module = load_module()
    checkpoint_path, chunks, plan, checkpoint = _valid_checkpoint_fixture(
        module, tmp_path
    )
    mutations = [
        ("schema_version", lambda value: value.__setitem__("schema_version", 2)),
        ("audited collection session", lambda value: value["sessions"].clear()),
        (
            "actor baseline pin",
            lambda value: value.pop("baseline_actor_type_multiset_sha256"),
        ),
        (
            "contiguous scan prefix",
            lambda value: value["sessions"][1].__setitem__(
                "starting_completed_scans", 0
            ),
        ),
        (
            "completed scan prefix",
            lambda value: value["sessions"][1].__setitem__(
                "ending_completed_scans", 1
            ),
        ),
        (
            "unaudited CARLA",
            lambda value: value["completed_scans"][0]["raw_tag_counts"].__setitem__(
                "29", 0
            ),
        ),
        (
            "non-integer tag key",
            lambda value: value["completed_scans"][0]["raw_tag_counts"].__setitem__(
                "1.0", 0
            ),
        ),
        (
            "non-negative integer",
            lambda value: value["completed_scans"][0]["raw_tag_counts"].__setitem__(
                "1", True
            ),
        ),
        (
            "non-negative integer",
            lambda value: value["completed_scans"][0]["raw_tag_counts"].__setitem__(
                "1", -1
            ),
        ),
        (
            "sum to raw_points",
            lambda value: value["completed_scans"][0]["raw_tag_counts"].__setitem__(
                "1", 4
            ),
        ),
        (
            "retained_static_points differs",
            lambda value: value["completed_scans"][0].__setitem__(
                "retained_static_points", 4
            ),
        ),
        (
            "voxel_points exceeds",
            lambda value: value["completed_scans"][0].__setitem__(
                "voxel_points", 6
            ),
        ),
        (
            "tag_totals differs",
            lambda value: value["tag_totals"].__setitem__("1", 7),
        ),
        (
            "CARLA frame",
            lambda value: value["completed_scans"][0].__setitem__(
                "carla_frame", True
            ),
        ),
        (
            "source transform errors",
            lambda value: value["completed_scans"][0].__setitem__(
                "source_translation_error_m", float("nan")
            ),
        ),
        (
            "source transform errors",
            lambda value: value["completed_scans"][0].__setitem__(
                "source_rotation_error_rad", 0.1
            ),
        ),
        (
            "commanded transform readback errors",
            lambda value: value["completed_scans"][0].__setitem__(
                "commanded_transform_readback_error_m", float("nan")
            ),
        ),
        (
            "transform-settle record",
            lambda value: value["completed_scans"][0].pop("transform_settle"),
        ),
        (
            "transform-settle record",
            lambda value: value["completed_scans"][1]["transform_settle"].__setitem__(
                "capture_attempt_count", True
            ),
        ),
        (
            "transform-settle attempt",
            lambda value: value["completed_scans"][0]["transform_settle"][
                "attempts"
            ][0].__setitem__("accepted", True),
        ),
        (
            "transform-settle attempt",
            lambda value: value["completed_scans"][0]["transform_settle"][
                "attempts"
            ][0].__setitem__("attempt_index", False),
        ),
        (
            "transform-settle attempt",
            lambda value: value["completed_scans"][0]["transform_settle"][
                "attempts"
            ][0].pop("actor_and_server_audit"),
        ),
        (
            "final transform-settle measurement",
            lambda value: value["completed_scans"][0]["transform_settle"][
                "attempts"
            ][-1].__setitem__("source_translation_error_m", 0.001),
        ),
        (
            "final transform-settle measurement",
            lambda value: value["completed_scans"][0]["transform_settle"][
                "attempts"
            ][0].__setitem__("carla_frame", 98),
        ),
    ]

    for message, mutate in mutations:
        corrupted = deepcopy(checkpoint)
        mutate(corrupted)
        checkpoint_path.write_text(json.dumps(corrupted), encoding="utf-8")
        with pytest.raises(module.MapGenerationError, match=message):
            module._load_checkpoint(checkpoint_path, plan, chunks)


def test_static_scene_actor_audit_pins_only_map_baseline_and_owned_sensor():
    module = load_module()
    baseline_actors = [
        Actor(1, "spectator"),
        Actor(6, "traffic.traffic_light"),
    ]
    baseline = module.static_scene_actor_audit(baseline_actors)

    assert baseline["status"] == "PASS_MAP_BASELINE_ONLY"
    assert baseline["blocking_actors"] == []
    sensor = Actor(7, "sensor.lidar.ray_cast_semantic")
    pinned = module.static_scene_actor_audit(
        [*baseline_actors, sensor], baseline["actor_inventory"], sensor
    )
    assert pinned["status"] == "PASS_PINNED_ACTOR_SET"

    runtime_actors = [
        *baseline_actors,
        Actor(4, "vehicle.tesla.model3", "hero"),
        Actor(3, "walker.pedestrian.0001"),
        Actor(2, "controller.ai.walker"),
        Actor(8, "static.prop.vendingmachine"),
    ]
    audit = module.static_scene_actor_audit(runtime_actors)

    assert audit["status"] == "FAIL_RUNTIME_ACTORS"
    assert audit["actor_count_inspected"] == 6
    assert audit["blocking_actors"] == [
        {"id": 2, "type_id": "controller.ai.walker", "role_name": ""},
        {"id": 3, "type_id": "walker.pedestrian.0001", "role_name": ""},
        {"id": 4, "type_id": "vehicle.tesla.model3", "role_name": "hero"},
        {"id": 8, "type_id": "static.prop.vendingmachine", "role_name": ""},
    ]
    changed = module.static_scene_actor_audit(
        [baseline_actors[0], sensor], baseline["actor_inventory"], sensor
    )
    assert changed["status"] == "FAIL_ACTOR_SET_CHANGED"
    assert changed["missing_actors"] == [
        {"id": 6, "type_id": "traffic.traffic_light", "role_name": ""}
    ]


def test_disk_partitioned_merge_deduplicates_voxels_across_scan_chunks(tmp_path):
    module = load_module()
    chunks = tmp_path / "chunks"
    chunks.mkdir()
    first = np.asarray([[0.01, 0.01, 0.0], [10.01, -10.01, 1.0]], dtype="<f4")
    second = np.asarray([[0.09, 0.09, 0.0], [-0.01, -0.01, 0.0]], dtype="<f4")
    np.save(chunks / "scan_000000.npy", first, allow_pickle=False)
    np.save(chunks / "scan_000001.npy", second, allow_pickle=False)
    checkpoint = {
        "completed_scans": [
            {"chunk_file": "scan_000000.npy"},
            {"chunk_file": "scan_000001.npy"},
        ]
    }
    output = tmp_path / "pointcloud_map.pcd"

    result = module.merge_chunks_to_pcd(
        checkpoint, tmp_path, output, 0.1, 5.0, "binary"
    )

    assert result["points"] == 3
    assert result["encoding"] == "binary"
    payload = output.read_bytes().split(b"DATA binary\n", 1)[1]
    points = np.frombuffer(payload, dtype="<f4").reshape(-1, 3)
    np.testing.assert_allclose(
        points,
        [[-0.01, -0.01, 0.0], [0.05, 0.05, 0.0], [10.01, -10.01, 1.0]],
        atol=1e-6,
    )
    assert not list(tmp_path.glob(".pcd-merge-*"))

    original = output.read_bytes()
    with pytest.raises(module.MapGenerationError, match="refusing to overwrite"):
        module.merge_chunks_to_pcd(
            checkpoint, tmp_path, output, 0.1, 5.0, "binary"
        )
    assert output.read_bytes() == original


def test_nearest_qa_uses_xyz_to_disambiguate_stacked_roads():
    module = load_module()
    reference = np.asarray([[0.0, 0.0, 10.0]])
    low_first = np.asarray([[0.0, 0.0, 0.0], [0.0, 0.0, 10.0]])
    high_first = low_first[::-1].copy()

    first = module._nearest_qa(reference, low_first)
    second = module._nearest_qa(reference, high_first)

    assert first == second
    assert first["nearest_xy_m"]["max"] == 0.0
    assert first["nearest_abs_z_m"]["max"] == 0.0


def test_global_qa_rejects_a_straight_fixture_mislabeled_as_a_turn(tmp_path):
    module = load_module()
    fixture = ROOT / "autoware_e2e_vad_launch/test/fixtures/route_map"
    lanelet = fixture / "Town99_full/lanelet2_map.osm"
    route = fixture / "town99_route.json"
    route_root = tmp_path / "routes"
    for scenario in ("straight", "left"):
        scenario_dir = route_root / scenario
        scenario_dir.mkdir(parents=True)
        shutil.copyfile(route, scenario_dir / f"{scenario}.json")
    points = np.vstack(
        [module._read_lanelet_nodes(lanelet), module._route_xyz(route)]
    )
    pcd = tmp_path / "pointcloud_map.pcd"
    module.write_binary_pcd(pcd, points)

    with pytest.raises(module.MapGenerationError, match="unsupported scenario"):
        module.run_global_qa(pcd, lanelet, route_root)


def _write_maneuver_route(path, scenario):
    if scenario == "straight":
        xy = [(float(index), 0.0) for index in range(30)]
        target = "STRAIGHT"
    elif scenario == "left":
        xy = [(float(index), 0.0) for index in range(12)]
        xy += [(11.0, float(index)) for index in range(1, 13)]
        target = "LEFT"
    else:
        xy = [(float(index), 0.0) for index in range(12)]
        xy += [(11.0, -float(index)) for index in range(1, 13)]
        target = "RIGHT"
    route = []
    distance = 0.0
    for index, (x, y) in enumerate(xy):
        if index:
            distance += float(
                np.hypot(x - xy[index - 1][0], y - xy[index - 1][1])
            )
        option = target if 7 <= index <= 15 else "LANEFOLLOW"
        route.append(
            {
                "x": x,
                "y": y,
                "z": 0.0,
                "yaw": 0.0,
                "distance_m": distance,
                "road_option": option,
            }
        )
    payload = {
        "scenario": scenario,
        "route_length_m": distance,
        "option_counts": {
            "LANEFOLLOW": len(route) - 9,
            target: 9,
        },
        "route": route,
    }
    path.parent.mkdir(parents=True)
    path.write_text(json.dumps(payload), encoding="utf-8")


def test_route_contract_cross_checks_options_and_planar_turn_direction(tmp_path):
    module = load_module()
    results = {}
    for scenario in ("straight", "left", "right"):
        path = tmp_path / scenario / f"{scenario}.json"
        _write_maneuver_route(path, scenario)
        results[scenario] = module.analyze_route_contract(path)

    assert abs(results["straight"]["maneuver_heading_change_rad"]) < 1e-9
    assert results["left"]["maneuver_heading_change_rad"] == pytest.approx(
        np.pi / 2.0
    )
    assert results["right"]["maneuver_heading_change_rad"] == pytest.approx(
        -np.pi / 2.0
    )

    mislabeled = tmp_path / "wrong/left/wrong.json"
    _write_maneuver_route(mislabeled, "left")
    payload = json.loads(mislabeled.read_text(encoding="utf-8"))
    payload["scenario"] = "right"
    mislabeled.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(module.MapGenerationError, match="differs from directory"):
        module.analyze_route_contract(mislabeled)
