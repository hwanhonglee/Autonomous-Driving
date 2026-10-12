from __future__ import annotations

import hashlib
from functools import lru_cache
from io import BytesIO
import json
import math
from pathlib import Path

from PIL import Image
import pytest

from portable_e2e import contract as common10
from portable_e2e import validate as validate_cli
from portable_e2e.dataset import FEATURE_NAMES
from portable_e2e.dataset import load_training_examples


def _write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


@lru_cache(maxsize=None)
def _jpeg_fixture(sequence_index: int, camera_index: int) -> bytes:
    stream = BytesIO()
    value = sequence_index * len(common10.CAMERA_ORDER) + camera_index
    color = (value & 0xFF, (value >> 8) & 0xFF, (value >> 16) & 0xFF)
    Image.new("RGB", (640, 360), color).save(stream, format="JPEG")
    return stream.getvalue()


def _camera_transform(view_yaw: float, translation_x: float) -> list[float]:
    forward_x = math.cos(view_yaw)
    forward_y = math.sin(view_yaw)
    right_x = forward_y
    right_y = -forward_x
    return [
        right_x,
        0.0,
        forward_x,
        translation_x,
        right_y,
        0.0,
        forward_y,
        0.0,
        0.0,
        -1.0,
        0.0,
        1.5,
        0.0,
        0.0,
        0.0,
        1.0,
    ]


def _rig() -> dict[str, object]:
    cameras = []
    role_yaws = (0.0, math.pi, math.pi / 4, 3 * math.pi / 4, -math.pi / 4, -3 * math.pi / 4)
    for index, name in enumerate(common10.CAMERA_ORDER):
        cameras.append(
            {
                "name": name,
                "model_index": index,
                "physical_id": f"virtual:carla:{name}",
                "optical_frame": f"{name}/camera_optical_link",
                "width_px": 640,
                "height_px": 360,
                "source_encoding": "bgr8",
                "storage_codec": "jpeg",
                "K": [457.0, 0.0, 319.5, 0.0, 457.0, 179.5, 0.0, 0.0, 1.0],
                "D": [],
                "horizontal_fov_rad": 2.0 * math.atan(640.0 / (2.0 * 457.0)),
                "T_base_from_camera": _camera_transform(
                    role_yaws[index], 1.0 + index * 0.01
                ),
                "timestamp_source": "carla_simulation_frame",
                "trigger_mode": "synchronous_tick",
                "firmware": "virtual-carla",
                "exposure_mode": "virtual-fixed",
            }
        )
    return {
        "schema_id": common10.RIG_SCHEMA_ID,
        "rig_id": "carla_rig_01",
        "domain": "carla",
        "base_frame": "base_link",
        "rectified": True,
        "camera_order": list(common10.CAMERA_ORDER),
        "cameras": cameras,
    }


def _sample(
    episode_root: Path,
    episode_id: str,
    sequence_index: int,
    *,
    route_geometry_sha256: str,
    command: int = 3,
    bundle_skew_ns: int = 0,
    invalid_zero_fill: bool = False,
    fake_image: bool = False,
    header_only_image: bool = False,
    repeat_images: bool = False,
    repeat_source_frames: bool = False,
) -> dict[str, object]:
    anchor = 1_000_000_000 + sequence_index * 100_000_000
    bundle = []
    for camera_index, name in enumerate(common10.CAMERA_ORDER):
        relative = Path("images") / name / f"{anchor}.jpg"
        image = episode_root / relative
        image.parent.mkdir(parents=True, exist_ok=True)
        image_payload = _jpeg_fixture(
            0 if repeat_images else sequence_index,
            0 if repeat_images else camera_index,
        )
        if fake_image:
            image_payload = b"not-a-jpeg"
        elif header_only_image:
            image_payload = bytes.fromhex("ffd8ffc00008080168028003ffd9")
        image.write_bytes(image_payload)
        timestamp = anchor + (bundle_skew_ns if camera_index == 0 else 0)
        source_index = sequence_index // 2 if repeat_source_frames else sequence_index
        bundle.append(
            {
                "name": name,
                "model_index": camera_index,
                "timestamp_ns": timestamp,
                "source_timestamp_ns": timestamp,
                "source_frame_id": f"{name}:{source_index:06d}",
                "frame_counter": sequence_index,
                "frame_id": f"{name}/camera_optical_link",
                "path": relative.as_posix(),
                "sha256": _sha(image),
            }
        )
    positions: list[list[float | None]] = []
    yaws: list[float | None] = []
    speeds: list[float | None] = []
    valid: list[bool] = []
    target_times: list[int | None] = []
    invalid_reasons: list[str | None] = []
    for point_index in range(64):
        horizon = (point_index + 1) * 0.1
        positions.append([2.0 * horizon, 0.0])
        yaws.append(0.0)
        speeds.append(2.0)
        valid.append(True)
        target_times.append(anchor + (point_index + 1) * 100_000_000)
        invalid_reasons.append(None)
    if invalid_zero_fill:
        positions[-1] = [0.0, 0.0]
        yaws[-1] = 0.0
        speeds[-1] = 0.0
        valid[-1] = False
        target_times[-1] = None
        invalid_reasons[-1] = "episode_end"
    ego_x = sequence_index * 0.2
    route_remaining = 100.0 - ego_x
    route_polyline = [
        [float(index), 0.0]
        for index in range(int(math.floor(route_remaining)) + 1)
    ]
    if route_remaining - route_polyline[-1][0] > 1.0e-6:
        route_polyline.append([route_remaining, 0.0])
    return {
        "schema_id": common10.SAMPLE_SCHEMA_ID,
        "sample_id": f"{episode_id}:{sequence_index:06d}",
        "source_sample_id": f"source:{episode_id}:{sequence_index:06d}",
        "episode_id": episode_id,
        "sequence_index": sequence_index,
        "anchor_timestamp_ns": anchor,
        "rig_id": "carla_rig_01",
        "camera_bundle": bundle,
        "ego": {
            "timestamp_ns": anchor,
            "frame_id": "map",
            "child_frame_id": "base_link",
            "position_m": [ego_x, 0.0, 0.0],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            "linear_velocity_base_mps": [2.0, 0.0, 0.0],
            "angular_velocity_base_radps": [0.0, 0.0, 0.0],
            "linear_acceleration_base_mps2": [0.0, 0.0, 0.0],
            "steering_tire_angle_rad": 0.0,
        },
        "navigation": {
            "route_id": "route_01",
            "route_source": "episode_route_geometry",
            "route_source_timestamp_ns": 1_000_000_000,
            "route_geometry_sha256": route_geometry_sha256,
            "route_anchor_arc_m": ego_x,
            "command": command,
            "command_timestamp_ns": anchor,
            "speed_limit_mps": 8.333333,
            "route_polyline_base_m": route_polyline,
            "goal_base_m": [route_remaining, 0.0],
        },
        "labels": {
            "planning": {
                "available": True,
                "dt_s": 0.1,
                "positions_base_xy_m": positions,
                "yaw_rad": yaws,
                "speed_mps": speeds,
                "valid": valid,
                "target_timestamp_ns": target_times,
                "invalid_reason": invalid_reasons,
            },
            "objects": {"available": False},
            "occupancy": {"available": False},
        },
        "events": {
            "collision": False,
            "lane_invasion": False,
            "manual_intervention": False,
            "fallback_active": False,
        },
    }


def _write_dataset(
    root: Path,
    episodes: tuple[tuple[str, str], ...] = (("episode_train", "train"),),
    *,
    bundle_skew_ns: int = 0,
    invalid_zero_fill: bool = False,
    contract: dict[str, object] | None = None,
    fake_image: bool = False,
    header_only_image: bool = False,
    repeat_images: bool = False,
    repeat_source_frames: bool = False,
    sample_count: int = 11,
) -> None:
    root.mkdir()
    rig_path = root / "rigs" / "carla_rig_01.json"
    _write_json(rig_path, _rig())
    episode_references = []
    for episode_index, (episode_id, split) in enumerate(episodes):
        episode_root = root / "episodes" / episode_id
        episode_root.mkdir(parents=True, exist_ok=True)
        route_geometry_path = episode_root / "route_geometry.json"
        route_geometry = {
            "route_id": "route_01",
            "frame_id": "map",
            "polyline_m": (
                [[0.0, 0.0], [100.0, 0.0]]
                if episode_index == 0
                else [[0.0, 0.0], [50.0, 0.0], [100.0, 0.0]]
            ),
        }
        if episode_index == 0:
            _write_json(route_geometry_path, route_geometry)
        else:
            route_geometry_path.write_text(
                json.dumps(route_geometry, separators=(",", ":")), encoding="utf-8"
            )
        route_geometry_hash = _sha(route_geometry_path)
        samples = [
            _sample(
                episode_root,
                episode_id,
                index,
                route_geometry_sha256=route_geometry_hash,
                bundle_skew_ns=bundle_skew_ns,
                invalid_zero_fill=invalid_zero_fill and index == 0,
                fake_image=fake_image,
                header_only_image=header_only_image,
                repeat_images=repeat_images,
                repeat_source_frames=repeat_source_frames,
            )
            for index in range(sample_count)
        ]
        samples_path = episode_root / "samples.jsonl"
        samples_path.write_text(
            "".join(json.dumps(sample, separators=(",", ":")) + "\n" for sample in samples),
            encoding="utf-8",
        )
        route_semantic_hash = common10._route_geometry_semantic_hash(
            route_geometry_path,
            episode={"map_id": "Town07", "route_id": "route_01"},
            contract=common10.load_contract(),
        )
        source_manifest_path = episode_root / "source_manifest.json"
        camera_frames = []
        seen_source_frames = set()
        selected_samples = []
        for sample in samples:
            selected_samples.append(
                {
                    "source_sample_id": sample["source_sample_id"],
                    "anchor_timestamp_ns": sample["anchor_timestamp_ns"],
                    "camera_source_frame_ids": [
                        camera["source_frame_id"] for camera in sample["camera_bundle"]
                    ],
                }
            )
            for camera in sample["camera_bundle"]:
                key = (camera["name"], camera["source_frame_id"])
                if key in seen_source_frames:
                    continue
                seen_source_frames.add(key)
                source_sequence_index = int(
                    str(camera["source_frame_id"]).rsplit(":", 1)[1]
                )
                camera_frames.append(
                    {
                        "camera_name": camera["name"],
                        "source_frame_id": camera["source_frame_id"],
                        "source_timestamp_ns": camera["source_timestamp_ns"],
                        "source_sequence_index": source_sequence_index,
                        "source_uri": camera["path"],
                        "source_payload_sha256": camera["sha256"],
                        "source_artifact_sha256": route_geometry_hash,
                        "prepared_payload_sha256": camera["sha256"],
                        "image_transform_id": "identity_jpeg",
                    }
                )
        _write_json(
            source_manifest_path,
            {
                "schema_id": common10.SOURCE_MANIFEST_SCHEMA_ID,
                "source_session_id": f"session:{episode_id}",
                "source_dataset_id": "unit_fixture",
                "source_dataset_version": "v1",
                "license_id": "unit-test-only",
                "source_artifacts": [
                    {
                        "uri": f"unit://{episode_id}",
                        "sha256": route_geometry_hash,
                    }
                ],
                "route_source": {
                    "available_timestamp_ns": 1_000_000_000,
                    "uri": "route_geometry.json",
                    "source_payload_sha256": route_geometry_hash,
                    "source_artifact_sha256": route_geometry_hash,
                    "route_geometry_sha256": route_geometry_hash,
                },
                "camera_frames": camera_frames,
                "selected_samples": selected_samples,
            },
        )
        collection_config_path = episode_root / "collection_config.json"
        _write_json(collection_config_path, {"capture_hz": 10.0, "physics_hz": 20.0})
        derived_split_group = f"carla:Town07:{route_semantic_hash}"
        episode = {
            "schema_id": common10.EPISODE_SCHEMA_ID,
            "episode_id": episode_id,
            "domain": "carla",
            "split": split,
            "split_group_id": derived_split_group,
            "source_session_id": f"session:{episode_id}",
            "scene_group_id": f"{derived_split_group}:traffic=17:weather=ClearNoon",
            "rig_id": "carla_rig_01",
            "vehicle_id": "carla_prius_01",
            "site_id": "Town07",
            "map_id": "Town07",
            "route_id": "route_01",
            "scenario_tags": ["lane_follow", "straight"],
            "weather": "ClearNoon",
            "clock_domain": "carla_sim_time",
            "traffic_seed": 17,
            "start_timestamp_ns": 1_000_000_000,
            "end_timestamp_ns": 1_000_000_000 + (sample_count - 1) * 100_000_000 + 6_400_000_000,
            "route_geometry_file": "route_geometry.json",
            "route_geometry_sha256": route_geometry_hash,
            "sample_jsonl": "samples.jsonl",
            "sample_count": len(samples),
            "sample_jsonl_sha256": _sha(samples_path),
            "source_provenance": {
                "adapter_id": "native_common10_v1",
                "git_commit": "0123456789abcdef0123456789abcdef01234567",
                "source_manifest_file": "source_manifest.json",
                "source_manifest_sha256": _sha(source_manifest_path),
                "collection_config_file": "collection_config.json",
                "collection_config_sha256": _sha(collection_config_path),
            },
            "capture_accounting": {
                "raw_anchor_count": sample_count,
                "eligible_anchor_count": sample_count,
                "selected_bundle_count": sample_count,
                "dropped_incomplete_bundle_count": 0,
                "ineligible_anchor_reasons": {},
                "raw_camera_frame_counts": {
                    name: sample_count for name in common10.CAMERA_ORDER
                },
            },
        }
        episode_path = episode_root / "episode.json"
        _write_json(episode_path, episode)
        episode_references.append(
            {
                "episode_id": episode_id,
                "manifest": episode_path.relative_to(root).as_posix(),
                "sha256": _sha(episode_path),
            }
        )
    _write_json(
        root / "dataset.json",
        {
            "schema_id": common10.DATASET_SCHEMA_ID,
            "contract_id": common10.CONTRACT_ID,
            "contract_sha256": common10.contract_fingerprint(
                common10.load_contract() if contract is None else contract
            ),
            "dataset_id": "unit_common10",
            "created_at_utc": "2026-09-03T00:00:00Z",
            "camera_order": list(common10.CAMERA_ORDER),
            "split_policy": "route_site_day_holdout_v1",
            "rigs": [
                {
                    "rig_id": "carla_rig_01",
                    "manifest": rig_path.relative_to(root).as_posix(),
                    "sha256": _sha(rig_path),
                }
            ],
            "episodes": episode_references,
        },
    )


def _mutate_first_sample(root: Path, mutation) -> None:
    dataset_path = root / "dataset.json"
    dataset = json.loads(dataset_path.read_text(encoding="utf-8"))
    episode_path = root / dataset["episodes"][0]["manifest"]
    episode = json.loads(episode_path.read_text(encoding="utf-8"))
    samples_path = episode_path.parent / episode["sample_jsonl"]
    samples = [json.loads(line) for line in samples_path.read_text(encoding="utf-8").splitlines()]
    mutation(samples[0])
    samples_path.write_text(
        "".join(json.dumps(sample, separators=(",", ":")) + "\n" for sample in samples),
        encoding="utf-8",
    )
    episode["sample_jsonl_sha256"] = _sha(samples_path)
    _write_json(episode_path, episode)
    dataset["episodes"][0]["sha256"] = _sha(episode_path)
    _write_json(dataset_path, dataset)


def _mutate_first_episode(root: Path, mutation) -> None:
    dataset_path = root / "dataset.json"
    dataset = json.loads(dataset_path.read_text(encoding="utf-8"))
    episode_path = root / dataset["episodes"][0]["manifest"]
    episode = json.loads(episode_path.read_text(encoding="utf-8"))
    mutation(episode)
    _write_json(episode_path, episode)
    dataset["episodes"][0]["sha256"] = _sha(episode_path)
    _write_json(dataset_path, dataset)


def _mutate_first_source_manifest(root: Path, mutation) -> None:
    dataset_path = root / "dataset.json"
    dataset = json.loads(dataset_path.read_text(encoding="utf-8"))
    episode_path = root / dataset["episodes"][0]["manifest"]
    episode = json.loads(episode_path.read_text(encoding="utf-8"))
    source_path = episode_path.parent / episode["source_provenance"][
        "source_manifest_file"
    ]
    source_manifest = json.loads(source_path.read_text(encoding="utf-8"))
    mutation(source_manifest)
    _write_json(source_path, source_manifest)
    episode["source_provenance"]["source_manifest_sha256"] = _sha(source_path)
    _write_json(episode_path, episode)
    dataset["episodes"][0]["sha256"] = _sha(episode_path)
    _write_json(dataset_path, dataset)


def _convert_fixture_to_real(
    root: Path,
    episode_fields: list[tuple[str, str, str]],
) -> None:
    dataset_path = root / "dataset.json"
    dataset = json.loads(dataset_path.read_text(encoding="utf-8"))
    rig_path = root / dataset["rigs"][0]["manifest"]
    rig = json.loads(rig_path.read_text(encoding="utf-8"))
    rig["domain"] = "real"
    for camera_index, camera in enumerate(rig["cameras"]):
        camera["physical_id"] = f"real:test-camera:{camera_index}"
    evidence_files = {}
    for name in ("calibration", "rectification", "source_sensor_metadata"):
        evidence_path = rig_path.parent / f"recorded_{name}.json"
        _write_json(evidence_path, {"kind": name, "source": "unit_fixture"})
        evidence_files[f"{name}_file"] = evidence_path.name
        evidence_files[f"{name}_sha256"] = _sha(evidence_path)
    rig["real_data_kind"] = "recorded_dataset"
    rig["recorded_dataset"] = evidence_files
    _write_json(rig_path, rig)
    dataset["rigs"][0]["sha256"] = _sha(rig_path)

    for reference, (site_id, collection_date, drive_id) in zip(
        dataset["episodes"], episode_fields
    ):
        episode_path = root / reference["manifest"]
        episode = json.loads(episode_path.read_text(encoding="utf-8"))
        episode["domain"] = "real"
        episode["site_id"] = site_id
        episode["collection_date"] = collection_date
        episode["continuous_drive_id"] = drive_id
        episode.pop("traffic_seed")
        route_path = episode_path.parent / episode["route_geometry_file"]
        route_semantic_hash = common10._route_geometry_semantic_hash(
            route_path,
            episode=episode,
            contract=common10.load_contract(),
            expected_sha256=episode["route_geometry_sha256"],
        )
        episode["split_group_id"] = (
            f"real:{site_id}:{collection_date}:{drive_id}"
        )
        episode["scene_group_id"] = (
            f"{episode['split_group_id']}:route={route_semantic_hash}"
        )
        _write_json(episode_path, episode)
        reference["sha256"] = _sha(episode_path)
    _write_json(dataset_path, dataset)


def test_default_contract_is_10hz_and_uses_deployed_command_enum() -> None:
    contract = common10.load_contract()

    assert contract["capture"]["nominal_rate_hz"] == 10.0
    assert contract["camera_order"] == list(common10.CAMERA_ORDER)
    assert contract["commands"] == {
        "0": "left",
        "1": "right",
        "2": "straight",
        "3": "lane_follow",
        "4": "change_left",
        "5": "change_right",
    }
    assert contract["trajectory"]["point_count"] == 64
    assert contract["trajectory"]["horizon_s"] == 6.4


def test_schema_validator_and_framework_neutral_loader_pass(tmp_path: Path) -> None:
    dataset = tmp_path / "dataset"
    _write_dataset(dataset)

    report = common10.validate_dataset(
        dataset, mode="schema", check_image_hashes=True
    )
    loaded = load_training_examples(dataset, split="train", mode="schema")

    assert report["status"] == "PASS"
    assert report["sample_count"] == 11
    assert report["episodes"][0]["effective_rate_hz"] == pytest.approx(10.0)
    assert report["qualification"]["common_10hz_planning"] == "NOT_RUN"
    assert report["qualification"]["image_payload_sha256"] == "PASS"
    assert (
        report["qualification"]["image_pixel_decode"]
        == "NOT_RUN_REQUIRES_APPROVED_IMAGE_LIBRARY"
    )
    assert report["qualification"]["offline_1ms_bundle_readiness"] == "NOT_RUN"
    assert report["resource_limits"] == {
        "max_json_file_bytes": common10.MAX_JSON_FILE_BYTES,
        "max_jsonl_line_bytes": common10.MAX_JSONL_LINE_BYTES,
        "max_jpeg_file_bytes": common10.MAX_JPEG_FILE_BYTES,
        "max_source_manifest_frames": common10.MAX_SOURCE_MANIFEST_FRAMES,
    }
    assert common10.load_contract()["_resource_limits"] == report["resource_limits"]
    assert len(loaded.examples) == 11
    assert len(loaded.examples[0].features) == len(FEATURE_NAMES)
    assert len(loaded.examples[0].targets_xy) == 64
    assert len(loaded.examples[0].camera_paths) == 6
    assert len(loaded.examples[0].camera_calibration) == 6
    assert all(len(values) == 16 for values in loaded.examples[0].camera_calibration)
    assert loaded.examples[0].target_yaw_rad == (0.0,) * 64
    assert loaded.examples[0].target_speed_mps == (2.0,) * 64
    assert loaded.examples[0].sequence_index == 0
    assert loaded.examples[0].anchor_timestamp_ns == 1_000_000_000
    assert all(path.is_file() for path in loaded.examples[0].camera_paths)
    assert len(loaded.examples[0].route_points_base_xy_m) == 101
    assert loaded.examples[0].route_points_base_xy_m[:2] == (
        (0.0, 0.0),
        (1.0, 0.0),
    )
    assert loaded.examples[0].route_points_base_xy_m[-1] == (100.0, 0.0)
    assert loaded.fingerprint_sha256 == load_training_examples(
        dataset, split="train", mode="schema"
    ).fingerprint_sha256


def test_full_planning_qualification_passes_30_seconds_at_10hz(tmp_path: Path) -> None:
    dataset = tmp_path / "dataset"
    _write_dataset(dataset, sample_count=301)

    report = common10.validate_dataset(dataset, mode="planning")

    assert report["qualification"]["common_10hz_planning"] == "PASS"
    assert report["episodes"][0]["duration_s"] == pytest.approx(30.0)
    assert report["episodes"][0]["bundle_coverage_percent"] == 100.0
    assert report["episodes"][0]["minimum_unique_jpeg_scan_percent"] > 99.0
    assert report["episodes"][0]["cross_camera_duplicate_bundle_count"] >= 0
    assert report["qualification"]["native_source_frame_identity"] == "PASS"
    assert report["qualification"]["native_source_manifest_binding"] == "PASS"
    assert report["qualification"]["camera_timestamp_equals_native_source"] == "PASS"
    assert (
        report["qualification"]["raw_source_artifact_content"]
        == "NOT_RUN_REQUIRES_RAW_SOURCE_REVIEW"
    )
    assert report["qualification"]["canonical_causal_route_reconstruction"] == "PASS"
    assert (
        report["qualification"]["jpeg_scan_payload_reuse_diagnostic"]
        == "MEASURED_NOT_A_QUALIFICATION_GATE"
    )


def test_planning_allows_identical_jpeg_when_native_source_frames_are_unique(
    tmp_path: Path,
) -> None:
    dataset = tmp_path / "dataset"
    _write_dataset(dataset, sample_count=301, repeat_images=True)

    report = common10.validate_dataset(dataset, mode="planning")

    assert report["episodes"][0]["minimum_unique_jpeg_scan_percent"] < 1.0
    assert report["episodes"][0]["maximum_consecutive_identical_jpeg_scans"] == 301
    assert report["qualification"]["native_source_frame_identity"] == "PASS"


def test_planning_rejects_retimed_duplicate_native_source_frames(
    tmp_path: Path,
) -> None:
    dataset = tmp_path / "dataset"
    _write_dataset(dataset, sample_count=301, repeat_source_frames=True)

    with pytest.raises(common10.ContractError, match="source manifest"):
        common10.validate_dataset(dataset, mode="planning")


def test_validator_rejects_camera_timestamp_retimed_from_native_source(
    tmp_path: Path,
) -> None:
    dataset = tmp_path / "dataset"
    _write_dataset(dataset)

    def retime(sample: dict) -> None:
        sample["camera_bundle"][0]["source_timestamp_ns"] += 1

    _mutate_first_sample(dataset, retime)

    with pytest.raises(common10.ContractError, match="must equal the native source"):
        common10.validate_dataset(dataset, mode="schema")


def test_validator_binds_selected_frame_to_hashed_source_manifest(
    tmp_path: Path,
) -> None:
    dataset = tmp_path / "dataset"
    _write_dataset(dataset)

    def replace_frame_id(sample: dict) -> None:
        sample["camera_bundle"][0]["source_frame_id"] = "CAM_FRONT:fabricated"

    _mutate_first_sample(dataset, replace_frame_id)

    with pytest.raises(common10.ContractError, match="absent from the source manifest"):
        common10.validate_dataset(dataset, mode="schema")


def test_validator_rejects_tampered_source_manifest_derivation(
    tmp_path: Path,
) -> None:
    dataset = tmp_path / "dataset"
    _write_dataset(dataset)

    def tamper(source_manifest: dict) -> None:
        source_manifest["camera_frames"][0]["prepared_payload_sha256"] = "0" * 64

    _mutate_first_source_manifest(dataset, tamper)

    with pytest.raises(common10.ContractError, match="identity_jpeg hashes"):
        common10.validate_dataset(dataset, mode="schema")


@pytest.mark.parametrize(
    ("field", "message"),
    [
        ("route_source_timestamp_ns", "route source timestamp is in the future"),
        ("route_geometry_sha256", "does not match the episode"),
        ("route_anchor_arc_m", "does not match the anchor projection"),
        ("goal_base_m", "does not match the mission route endpoint"),
        ("route_polyline_base_m", "is not the canonical causal projection"),
    ],
)
def test_validator_rejects_noncausal_or_noncanonical_route_input(
    tmp_path: Path,
    field: str,
    message: str,
) -> None:
    dataset = tmp_path / "dataset"
    _write_dataset(dataset)

    def corrupt_route(sample: dict) -> None:
        navigation = sample["navigation"]
        if field == "route_source_timestamp_ns":
            navigation[field] = sample["anchor_timestamp_ns"] + 1
        elif field == "route_geometry_sha256":
            navigation[field] = "0" * 64
        elif field == "route_anchor_arc_m":
            navigation[field] = 10.0
        elif field == "goal_base_m":
            navigation[field] = [50.0, 10.0]
        else:
            navigation[field] = list(reversed(navigation[field]))

    _mutate_first_sample(dataset, corrupt_route)

    with pytest.raises(common10.ContractError, match=message):
        common10.validate_dataset(dataset, mode="schema")


def test_route_projection_uses_heading_to_resolve_self_intersection() -> None:
    route, goal, anchor_arc = common10._canonical_route_in_base(
        [(-10.0, 0.0), (10.0, 0.0), (0.0, -10.0), (0.0, 10.0)],
        position_m=(0.0, 0.0, 0.0),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        contract=common10.load_contract(),
        context="unit.route",
    )

    assert anchor_arc == pytest.approx(10.0)
    assert route[0] == pytest.approx((0.0, 0.0))
    assert route[1][0] > 0.0
    assert goal == pytest.approx((0.0, 10.0))


def test_route_projection_deduplicates_zero_length_endpoint_segments() -> None:
    route, goal, anchor_arc = common10._canonical_route_in_base(
        [(0.0, 0.0), (0.0, 0.0), (10.0, 0.0), (10.0, 0.0)],
        position_m=(0.0, 0.0, 0.0),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        contract=common10.load_contract(),
        context="unit.duplicate_route",
    )

    assert anchor_arc == pytest.approx(0.0)
    assert route[0] == pytest.approx((0.0, 0.0))
    assert goal == pytest.approx((10.0, 0.0))


def test_validator_rejects_bundle_over_dataset_tolerance(tmp_path: Path) -> None:
    dataset = tmp_path / "dataset"
    _write_dataset(dataset, bundle_skew_ns=21_000_000)

    with pytest.raises(common10.ContractError, match="bundle exceeds dataset skew"):
        common10.validate_dataset(dataset, mode="schema")


def test_validator_rejects_zero_filled_invalid_future_label(tmp_path: Path) -> None:
    dataset = tmp_path / "dataset"
    _write_dataset(dataset, invalid_zero_fill=True)

    with pytest.raises(common10.ContractError, match="invalid targets must be null"):
        common10.validate_dataset(dataset, mode="schema")


def test_validator_rejects_episode_group_split_leakage(tmp_path: Path) -> None:
    dataset = tmp_path / "dataset"
    _write_dataset(
        dataset,
        episodes=(
            ("episode_train", "train"),
            ("episode_test", "test"),
        ),
    )
    manifest = json.loads((dataset / "dataset.json").read_text(encoding="utf-8"))
    route_payloads = []
    for reference in manifest["episodes"]:
        episode_path = dataset / reference["manifest"]
        episode = json.loads(episode_path.read_text(encoding="utf-8"))
        route_payloads.append((episode_path.parent / episode["route_geometry_file"]).read_bytes())
    assert route_payloads[0] != route_payloads[1]

    with pytest.raises(common10.ContractError, match="carla_route_geometry.*both train and test"):
        common10.validate_dataset(dataset, mode="schema")


def test_route_id_alias_cannot_hide_same_geometry_across_splits(tmp_path: Path) -> None:
    dataset = tmp_path / "dataset"
    _write_dataset(
        dataset,
        episodes=(
            ("episode_train", "train"),
            ("episode_test", "test"),
        ),
    )
    dataset_path = dataset / "dataset.json"
    manifest = json.loads(dataset_path.read_text(encoding="utf-8"))
    reference = manifest["episodes"][1]
    episode_path = dataset / reference["manifest"]
    episode = json.loads(episode_path.read_text(encoding="utf-8"))
    episode["route_id"] = "route_alias"

    route_path = episode_path.parent / episode["route_geometry_file"]
    route = json.loads(route_path.read_text(encoding="utf-8"))
    route["route_id"] = "route_alias"
    _write_json(route_path, route)
    episode["route_geometry_sha256"] = _sha(route_path)

    source_manifest_path = (
        episode_path.parent / episode["source_provenance"]["source_manifest_file"]
    )
    source_manifest = json.loads(source_manifest_path.read_text(encoding="utf-8"))
    source_manifest["route_source"]["source_payload_sha256"] = _sha(route_path)
    source_manifest["route_source"]["route_geometry_sha256"] = _sha(route_path)
    _write_json(source_manifest_path, source_manifest)
    episode["source_provenance"]["source_manifest_sha256"] = _sha(
        source_manifest_path
    )

    samples_path = episode_path.parent / episode["sample_jsonl"]
    samples = [
        json.loads(line)
        for line in samples_path.read_text(encoding="utf-8").splitlines()
    ]
    for sample in samples:
        sample["navigation"]["route_id"] = "route_alias"
        sample["navigation"]["route_geometry_sha256"] = _sha(route_path)
    samples_path.write_text(
        "".join(json.dumps(sample, separators=(",", ":")) + "\n" for sample in samples),
        encoding="utf-8",
    )
    episode["sample_jsonl_sha256"] = _sha(samples_path)
    _write_json(episode_path, episode)
    reference["sha256"] = _sha(episode_path)
    _write_json(dataset_path, manifest)

    with pytest.raises(common10.ContractError, match="carla_route_geometry.*both train and test"):
        common10.validate_dataset(dataset, mode="schema")


def test_real_site_day_cannot_cross_splits_even_for_different_drives(
    tmp_path: Path,
) -> None:
    dataset = tmp_path / "dataset"
    _write_dataset(
        dataset,
        episodes=(("episode_train", "train"), ("episode_test", "test")),
    )
    _convert_fixture_to_real(
        dataset,
        [
            ("seoul_site", "2026-08-01", "drive_a"),
            ("seoul_site", "2026-08-01", "drive_b"),
        ],
    )

    with pytest.raises(common10.ContractError, match="real_site_day.*both train and test"):
        common10.validate_dataset(dataset, mode="schema")


def test_real_route_geometry_cannot_cross_splits_across_site_days(
    tmp_path: Path,
) -> None:
    dataset = tmp_path / "dataset"
    _write_dataset(
        dataset,
        episodes=(("episode_train", "train"), ("episode_test", "test")),
    )
    _convert_fixture_to_real(
        dataset,
        [
            ("site_a", "2026-08-01", "drive_a"),
            ("site_b", "2026-08-02", "drive_b"),
        ],
    )

    with pytest.raises(
        common10.ContractError,
        match="real_route_geometry.*both train and test",
    ):
        common10.validate_dataset(dataset, mode="schema")


def test_validator_rejects_non_jpeg_payload_even_when_hash_matches(tmp_path: Path) -> None:
    dataset = tmp_path / "dataset"
    _write_dataset(dataset, fake_image=True)

    with pytest.raises(common10.ContractError, match="not a JPEG"):
        common10.validate_dataset(dataset, mode="schema")


def test_validator_rejects_header_only_fake_jpeg_even_when_hash_matches(
    tmp_path: Path,
) -> None:
    dataset = tmp_path / "dataset"
    _write_dataset(dataset, header_only_image=True)

    with pytest.raises(common10.ContractError, match="not a JPEG"):
        common10.validate_dataset(dataset, mode="schema")


def test_route_semantic_hash_rejects_excessive_arc_length(tmp_path: Path) -> None:
    route_path = tmp_path / "route.json"
    _write_json(
        route_path,
        {
            "route_id": "route_01",
            "frame_id": "map",
            "polyline_m": [[0.0, 0.0], [common10.MAX_ROUTE_ARC_LENGTH_M + 1.0, 0.0]],
        },
    )

    with pytest.raises(common10.ContractError, match="arc-length limit"):
        common10._route_geometry_semantic_hash(
            route_path,
            episode={"map_id": "Town07", "route_id": "route_01"},
            contract=common10.load_contract(),
        )


def test_route_semantic_hash_rejects_non_finite_computed_length(tmp_path: Path) -> None:
    route_path = tmp_path / "route.json"
    _write_json(
        route_path,
        {
            "route_id": "route_01",
            "frame_id": "map",
            "polyline_m": [[-1.0e308, 0.0], [1.0e308, 0.0]],
        },
    )

    with pytest.raises(common10.ContractError, match="non-finite arc length"):
        common10._route_geometry_semantic_hash(
            route_path,
            episode={"map_id": "Town07", "route_id": "route_01"},
            contract=common10.load_contract(),
        )


def test_default_validation_hashes_image_payloads(tmp_path: Path) -> None:
    dataset = tmp_path / "dataset"
    _write_dataset(dataset)
    image = next((dataset / "episodes" / "episode_train" / "images").rglob("*.jpg"))
    Image.new("RGB", (640, 360), (200, 10, 10)).save(image, format="JPEG")

    with pytest.raises(common10.ContractError, match="sha256 mismatch"):
        common10.validate_dataset(dataset, mode="schema")


@pytest.mark.parametrize("field", ["ego", "command"])
def test_validator_rejects_future_input_timestamp(tmp_path: Path, field: str) -> None:
    dataset = tmp_path / "dataset"
    _write_dataset(dataset)

    def make_future(sample: dict) -> None:
        if field == "ego":
            sample["ego"]["timestamp_ns"] = sample["anchor_timestamp_ns"] + 1
        else:
            sample["navigation"]["command_timestamp_ns"] = sample["anchor_timestamp_ns"] + 1

    _mutate_first_sample(dataset, make_future)

    with pytest.raises(common10.ContractError, match="timestamp is in the future"):
        common10.validate_dataset(dataset, mode="schema")


def test_validator_rejects_future_mask_hole(tmp_path: Path) -> None:
    dataset = tmp_path / "dataset"
    _write_dataset(dataset)

    def add_hole(sample: dict) -> None:
        planning = sample["labels"]["planning"]
        planning["positions_base_xy_m"][1] = [None, None]
        planning["yaw_rad"][1] = None
        planning["speed_mps"][1] = None
        planning["valid"][1] = False
        planning["target_timestamp_ns"][1] = None
        planning["invalid_reason"][1] = "sensor_gap"

    _mutate_first_sample(dataset, add_hole)

    with pytest.raises(common10.ContractError, match="valid mask must be a prefix"):
        common10.validate_dataset(dataset, mode="schema")


def test_validator_rejects_unimplemented_object_payload_claim(tmp_path: Path) -> None:
    dataset = tmp_path / "dataset"
    _write_dataset(dataset)

    def claim_objects(sample: dict) -> None:
        sample["labels"]["objects"]["available"] = True

    _mutate_first_sample(dataset, claim_objects)

    with pytest.raises(common10.ContractError, match="payload schema"):
        common10.validate_dataset(dataset, mode="schema")


def test_runtime_mode_requires_offline_one_millisecond_bundle_readiness(
    tmp_path: Path,
) -> None:
    dataset = tmp_path / "dataset"
    _write_dataset(dataset, bundle_skew_ns=2_000_000, sample_count=301)

    planning = common10.validate_dataset(dataset, mode="planning")
    assert planning["status"] == "PASS"
    assert planning["qualification"]["offline_1ms_bundle_readiness"] == "FAIL"
    with pytest.raises(common10.ContractError, match="offline 1 ms bundle readiness"):
        common10.validate_dataset(dataset, mode="runtime")


def test_contract_id_cannot_hide_weakened_timing_thresholds() -> None:
    contract = json.loads(json.dumps(common10.load_contract()))
    contract["capture"]["maximum_p99_gap_ms"] = 999.0

    with pytest.raises(common10.ContractError, match="thresholds are not"):
        common10.validate_contract(contract)


def test_validator_rejects_wrong_facing_camera_extrinsic(tmp_path: Path) -> None:
    dataset = tmp_path / "dataset"
    _write_dataset(dataset)
    dataset_path = dataset / "dataset.json"
    document = json.loads(dataset_path.read_text(encoding="utf-8"))
    rig_path = dataset / document["rigs"][0]["manifest"]
    rig = json.loads(rig_path.read_text(encoding="utf-8"))
    rig["cameras"][0]["T_base_from_camera"] = [
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        1.5,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
    _write_json(rig_path, rig)
    document["rigs"][0]["sha256"] = _sha(rig_path)
    _write_json(dataset_path, document)

    with pytest.raises(common10.ContractError, match="invalid pitch"):
        common10.validate_dataset(dataset, mode="schema")


def test_planning_mode_checks_raw_bundle_accounting(tmp_path: Path) -> None:
    dataset = tmp_path / "dataset"
    _write_dataset(dataset, sample_count=301)

    def hide_drops(episode: dict) -> None:
        episode["capture_accounting"]["raw_anchor_count"] = 400
        episode["capture_accounting"]["eligible_anchor_count"] = 400
        episode["capture_accounting"]["dropped_incomplete_bundle_count"] = 99
        episode["capture_accounting"]["raw_camera_frame_counts"] = {
            name: 400 for name in common10.CAMERA_ORDER
        }

    _mutate_first_episode(dataset, hide_drops)

    with pytest.raises(common10.ContractError, match="source manifest"):
        common10.validate_dataset(dataset, mode="planning")


def test_validator_rejects_unexplained_ineligible_raw_anchors(tmp_path: Path) -> None:
    dataset = tmp_path / "dataset"
    _write_dataset(dataset, sample_count=301)

    def hide_raw_anchors(episode: dict) -> None:
        episode["capture_accounting"]["raw_anchor_count"] = 1000

    _mutate_first_episode(dataset, hide_raw_anchors)

    with pytest.raises(common10.ContractError, match="ineligible reason ledger"):
        common10.validate_dataset(dataset, mode="planning")


def test_validator_rejects_wrong_rig_reference_id(tmp_path: Path) -> None:
    dataset = tmp_path / "dataset"
    _write_dataset(dataset)
    dataset_path = dataset / "dataset.json"
    manifest = json.loads(dataset_path.read_text(encoding="utf-8"))
    manifest["rigs"][0]["rig_id"] = "wrong-reference-id"
    _write_json(dataset_path, manifest)

    with pytest.raises(common10.ContractError, match="rig_id does not match"):
        common10.validate_dataset(dataset, mode="schema")


def test_cli_normalizes_embedded_nul_path_to_json_failure(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    dataset = tmp_path / "dataset"
    _write_dataset(dataset)
    dataset_path = dataset / "dataset.json"
    manifest = json.loads(dataset_path.read_text(encoding="utf-8"))
    manifest["rigs"][0]["manifest"] = "bad\0name.json"
    _write_json(dataset_path, manifest)

    result = validate_cli.main([str(dataset), "--mode", "schema"])
    captured = capsys.readouterr()
    failure = json.loads(captured.err)

    assert result == 2
    assert failure["status"] == "FAIL"
    assert "NUL byte" in failure["error"]
    assert "Traceback" not in captured.err


def test_oversized_sparse_json_returns_cli_json_failure_before_reading(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    dataset = tmp_path / "dataset"
    dataset.mkdir()
    manifest = dataset / "dataset.json"
    with manifest.open("wb") as stream:
        stream.truncate(common10.MAX_JSON_FILE_BYTES + 1)

    result = validate_cli.main([str(dataset), "--mode", "schema"])
    captured = capsys.readouterr()
    failure = json.loads(captured.err)

    assert result == 2
    assert failure["status"] == "FAIL"
    assert str(common10.MAX_JSON_FILE_BYTES) in failure["error"]
    assert "exceeds limit" in failure["error"]
    assert "Traceback" not in captured.err


def test_source_manifest_frame_cap_covers_nuplan_camera_group_zero_scale() -> None:
    # The audited official archive has 242,385 total members, so its camera-frame
    # records are strictly fewer than this conservative upper bound.
    assert common10.MAX_SOURCE_MANIFEST_FRAMES == 500_000
    assert common10.MAX_SOURCE_MANIFEST_FRAMES > 242_385


def test_oversized_jsonl_line_is_bounded_before_json_parsing(tmp_path: Path) -> None:
    samples = tmp_path / "samples.jsonl"
    with samples.open("wb") as stream:
        stream.truncate(common10.MAX_JSONL_LINE_BYTES + 1)

    with pytest.raises(common10.ContractError, match="JSONL line exceeds limit"):
        list(common10._iter_jsonl(samples))


def test_deeply_nested_json_fails_as_contract_error() -> None:
    payload = '{"value":' + ("[" * 2_000) + ("[0]" + "]" * 2_000) + "}"

    with pytest.raises(common10.ContractError, match="safe parser depth"):
        common10._loads_json(payload, "nested fixture")


def test_oversized_sparse_jpeg_is_rejected_before_payload_read(tmp_path: Path) -> None:
    dataset = tmp_path / "dataset"
    _write_dataset(dataset)
    image = next(dataset.glob("episodes/**/images/**/*.jpg"))
    with image.open("r+b") as stream:
        stream.truncate(common10.MAX_JPEG_FILE_BYTES + 1)

    with pytest.raises(common10.ContractError, match="JPEG file size .* exceeds limit"):
        common10.validate_dataset(dataset, mode="schema")


def test_bounded_reader_converts_memory_error_to_contract_error(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    document = tmp_path / "small.json"
    document.write_text("{}\n", encoding="utf-8")

    def fail_allocation(_size: int) -> bytearray:
        raise MemoryError("simulated allocation failure")

    monkeypatch.setattr(common10, "bytearray", fail_allocation, raising=False)
    with pytest.raises(common10.ContractError, match="bounded memory budget"):
        common10._read_json_and_sha256(document)


def test_bounded_reader_rejects_path_replacement_during_read(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    document = tmp_path / "document.json"
    document.write_text("{}\n", encoding="utf-8")
    replacement = tmp_path / "replacement.json"
    replacement.write_text("[]\n", encoding="utf-8")
    original_read = common10.os.read
    replaced = False

    def read_then_replace(descriptor: int, size: int) -> bytes:
        nonlocal replaced
        payload = original_read(descriptor, size)
        if payload and not replaced:
            replaced = True
            common10.os.replace(replacement, document)
        return payload

    monkeypatch.setattr(common10.os, "read", read_then_replace)
    with pytest.raises(
        common10.ContractError,
        match="changed while it was read|different object after reading",
    ):
        common10._read_json_and_sha256(document)
