from __future__ import annotations

from dataclasses import replace
import hashlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tarfile
from typing import Callable
import warnings
import zipfile

import pytest

from scripts.e2e import inspect_nuscenes_mini_adapter as inspector
from scripts.e2e.verify_tar_archive import verify_tar_archive
from scripts.e2e.verify_zip_archive import verify_zip_archive


SCENE_NAME = "scene-0001"
SCRIPT = Path(__file__).parents[1] / "scripts/e2e/inspect_nuscenes_mini_adapter.py"


def _token(number: int) -> str:
    return f"{number:032x}"


@pytest.fixture
def tiny_expectations() -> inspector.MiniExpectations:
    return replace(
        inspector.DEFAULT_EXPECTATIONS,
        scene_samples=((SCENE_NAME, 3),),
        sample_count=3,
        sample_data_count=18,
        sensor_count=6,
        calibrated_sensor_count=6,
        ego_pose_count=18,
        log_count=1,
        camera_frame_count=18,
        camera_frames_by_channel=tuple(
            (channel, 3) for channel in inspector.CAMERA_CHANNELS
        ),
        camera_calibrations_per_channel=1,
    )


def _metadata(*, wrong_camera_channel: bool = False) -> tuple[dict[str, list[dict]], list[str]]:
    scene_token = _token(1)
    log_token = _token(2)
    sample_tokens = [_token(100 + index) for index in range(3)]
    sensors = []
    calibrations = []
    for index, channel in enumerate(inspector.CAMERA_CHANNELS):
        if wrong_camera_channel and index == len(inspector.CAMERA_CHANNELS) - 1:
            channel = "CAM_UNKNOWN"
        sensors.append(
            {
                "token": _token(200 + index),
                "channel": channel,
                "modality": "camera",
            }
        )
        calibrations.append(
            {
                "token": _token(300 + index),
                "sensor_token": _token(200 + index),
                "translation": [1.0, 0.0, 1.5],
                "rotation": [1.0, 0.0, 0.0, 0.0],
                "camera_intrinsic": [
                    [1000.0, 0.0, 800.0],
                    [0.0, 1000.0, 450.0],
                    [0.0, 0.0, 1.0],
                ],
            }
        )

    samples = []
    sample_data = []
    ego_poses = []
    image_paths = []
    datum_index = 0
    for sample_index, sample_token in enumerate(sample_tokens):
        timestamp = sample_index * 500_000
        samples.append(
            {
                "token": sample_token,
                "scene_token": scene_token,
                "timestamp": timestamp,
                "prev": "" if sample_index == 0 else sample_tokens[sample_index - 1],
                "next": "" if sample_index == 2 else sample_tokens[sample_index + 1],
            }
        )
        for channel_index, channel in enumerate(inspector.CAMERA_CHANNELS):
            camera_timestamp = timestamp + channel_index * 5_000
            ego_token = _token(400 + datum_index)
            filename = f"samples/{channel}/{_token(500 + datum_index)}.jpg"
            image_paths.append(filename)
            ego_poses.append(
                {
                    "token": ego_token,
                    "timestamp": camera_timestamp,
                    "translation": [float(sample_index), 0.0, 0.0],
                    "rotation": [1.0, 0.0, 0.0, 0.0],
                }
            )
            sample_data.append(
                {
                    "token": _token(500 + datum_index),
                    "sample_token": sample_token,
                    "ego_pose_token": ego_token,
                    "calibrated_sensor_token": _token(300 + channel_index),
                    "timestamp": camera_timestamp,
                    "filename": filename,
                    "fileformat": "jpg",
                    "width": 1600,
                    "height": 900,
                    "is_key_frame": True,
                    "prev": (
                        ""
                        if sample_index == 0
                        else _token(500 + (sample_index - 1) * 6 + channel_index)
                    ),
                    "next": (
                        ""
                        if sample_index == 2
                        else _token(500 + (sample_index + 1) * 6 + channel_index)
                    ),
                }
            )
            datum_index += 1

    tables = {
        "scene": [
            {
                "token": scene_token,
                "name": SCENE_NAME,
                "description": "tiny fixture",
                "log_token": log_token,
                "nbr_samples": 3,
                "first_sample_token": sample_tokens[0],
                "last_sample_token": sample_tokens[-1],
            }
        ],
        "sample": samples,
        "sample_data": sample_data,
        "sensor": sensors,
        "calibrated_sensor": calibrations,
        "ego_pose": ego_poses,
        "log": [
            {
                "token": log_token,
                "logfile": "tiny",
                "vehicle": "test",
                "date_captured": "2026-09-03",
                "location": "test",
            }
        ],
    }
    return tables, image_paths


def _tar_bytes(payload: bytes) -> io.BytesIO:
    stream = io.BytesIO(payload)
    stream.seek(0)
    return stream


def _add_tar_file(archive: tarfile.TarFile, name: str, payload: bytes) -> None:
    info = tarfile.TarInfo(name)
    info.size = len(payload)
    info.mtime = 0
    archive.addfile(info, _tar_bytes(payload))


def _add_tar_directory(archive: tarfile.TarFile, name: str) -> None:
    info = tarfile.TarInfo(name.rstrip("/") + "/")
    info.type = tarfile.DIRTYPE
    info.mtime = 0
    archive.addfile(info)


def _write_nuscenes_tar(
    path: Path,
    *,
    wrong_camera_channel: bool = False,
    add_symlink: bool = False,
    metadata_mutator: Callable[[dict[str, list[dict]], list[str]], None] | None = None,
    extra_regular_files: tuple[tuple[str, bytes], ...] = (),
    empty_image_paths: frozenset[str] = frozenset(),
) -> None:
    tables, image_paths = _metadata(wrong_camera_channel=wrong_camera_channel)
    if metadata_mutator is not None:
        metadata_mutator(tables, image_paths)
    with tarfile.open(path, "w:gz", format=tarfile.PAX_FORMAT) as archive:
        _add_tar_directory(archive, "v1.0-mini")
        for table_name, member_name in inspector.METADATA_MEMBERS.items():
            payload = json.dumps(tables[table_name], separators=(",", ":")).encode()
            _add_tar_file(archive, member_name, payload)
        _add_tar_directory(archive, "samples")
        for channel in inspector.CAMERA_CHANNELS:
            _add_tar_directory(archive, f"samples/{channel}")
        for image_path in image_paths:
            payload = b"" if image_path in empty_image_paths else b"not-decoded-image-payload"
            _add_tar_file(archive, image_path, payload)
        for member_name, payload in extra_regular_files:
            _add_tar_file(archive, member_name, payload)
        if add_symlink:
            link = tarfile.TarInfo("samples/link.jpg")
            link.type = tarfile.SYMTYPE
            link.linkname = "../../outside"
            archive.addfile(link)


def _write_can_zip(path: Path, *, include_route: bool = True) -> None:
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_STORED) as archive:
        archive.writestr("can_bus/", b"")
        archive.writestr(f"can_bus/{SCENE_NAME}_meta.json", b"{}")
        archive.writestr(f"can_bus/{SCENE_NAME}_pose.json", b"[]")
        if include_route:
            archive.writestr(
                f"can_bus/{SCENE_NAME}_route.json",
                json.dumps([[0.0, 0.0], [10.0, 0.0]]).encode(),
            )
        archive.writestr("LICENSE", b"test-license")


def _fixtures(
    tmp_path: Path,
    *,
    include_route: bool = True,
    wrong_camera_channel: bool = False,
    add_symlink: bool = False,
    metadata_mutator: Callable[[dict[str, list[dict]], list[str]], None] | None = None,
    extra_regular_files: tuple[tuple[str, bytes], ...] = (),
    empty_image_paths: frozenset[str] = frozenset(),
) -> tuple[Path, Path]:
    nuscenes = tmp_path / "v1.0-mini.tgz"
    can_bus = tmp_path / "can_bus.zip"
    _write_nuscenes_tar(
        nuscenes,
        wrong_camera_channel=wrong_camera_channel,
        add_symlink=add_symlink,
        metadata_mutator=metadata_mutator,
        extra_regular_files=extra_regular_files,
        empty_image_paths=empty_image_paths,
    )
    _write_can_zip(can_bus, include_route=include_route)
    return nuscenes, can_bus


def _write_generic_reports(
    tmp_path: Path,
    nuscenes: Path,
    can_bus: Path,
) -> tuple[Path, Path]:
    nuscenes_digest = hashlib.sha256(nuscenes.read_bytes()).hexdigest()
    can_digest = hashlib.sha256(can_bus.read_bytes()).hexdigest()
    tar_report = verify_tar_archive(
        nuscenes,
        expected_size=nuscenes.stat().st_size,
        expected_sha256=nuscenes_digest,
    )
    zip_report = verify_zip_archive(
        can_bus,
        expected_size=can_bus.stat().st_size,
        expected_sha256=can_digest,
        verify_payload_crc=True,
    )
    assert tar_report["valid"] is True
    assert zip_report["valid"] is True
    tar_path = tmp_path / "tar-audit.json"
    zip_path = tmp_path / "zip-audit.json"
    tar_path.write_text(json.dumps(tar_report), encoding="utf-8")
    zip_path.write_text(json.dumps(zip_report), encoding="utf-8")
    return tar_path, zip_path


def _inspect_verified(
    tmp_path: Path,
    nuscenes: Path,
    can_bus: Path,
    expectations: inspector.MiniExpectations,
    *,
    limits: inspector.AuditLimits = inspector.DEFAULT_LIMITS,
) -> dict:
    tar_report, zip_report = _write_generic_reports(tmp_path, nuscenes, can_bus)
    return inspector.inspect_nuscenes_mini_adapter(
        nuscenes,
        can_bus,
        nuscenes_audit_report=tar_report,
        can_bus_audit_report=zip_report,
        limits=limits,
        expectations=expectations,
    )


def _mutate_stored_license_without_changing_size(path: Path) -> None:
    with zipfile.ZipFile(path) as archive:
        info = archive.getinfo("LICENSE")
    payload = bytearray(path.read_bytes())
    offset = info.header_offset
    assert payload[offset:offset + 4] == b"PK\x03\x04"
    filename_size = int.from_bytes(payload[offset + 26:offset + 28], "little")
    extra_size = int.from_bytes(payload[offset + 28:offset + 30], "little")
    payload_offset = offset + 30 + filename_size + extra_size
    payload[payload_offset] ^= 0x01
    original_size = path.stat().st_size
    path.write_bytes(payload)
    assert path.stat().st_size == original_size


def test_structural_pass_is_separate_from_common10_not_qualified(
    tmp_path: Path,
    tiny_expectations: inspector.MiniExpectations,
) -> None:
    nuscenes, can_bus = _fixtures(tmp_path)
    before = {
        path: (path.read_bytes(), path.stat().st_mtime_ns)
        for path in (nuscenes, can_bus)
    }

    report = _inspect_verified(tmp_path, nuscenes, can_bus, tiny_expectations)

    assert report["valid"] is True
    assert report["status"] == "PASS"
    assert report["structural_validation"]["status"] == "PASS"
    assert report["common_10hz_v1"]["status"] == "NOT_QUALIFIED"
    assert report["common_10hz_v1"]["qualified"] is False
    assert report["common_10hz_v1"]["policy"] == {
        "retimed_frames": 0,
        "invented_or_duplicated_frames": 0,
        "cross_scene_concatenation": False,
        "selection_only_10hz": {
            "id": "first_timestamp_nearest_unused_v1",
            "grid_period_us": 100_000,
            "grid_anchor": "first_source_timestamp_per_scene_camera_stream",
            "selection": "nearest_unused_native_frame_at_or_after_previous_choice",
            "equal_distance_tie_break": "earlier_native_timestamp",
            "percentile_method": "nearest_rank_ceiling",
        },
    }
    assert report["dataset"]["scene_count"] == 1
    assert report["dataset"]["sample_count"] == 3
    assert report["dataset"]["camera"]["frame_count"] == 18
    assert report["dataset"]["camera"]["keyframe_bundle_count"] == 3
    assert report["dataset"]["camera"]["keyframe_bundle_skew_ms"]["max"] == 25.0
    stream_p99 = report["dataset"]["camera"][
        "selection_only_10hz_stream_p99_ms"
    ]
    assert stream_p99["count"] == 6
    cadence_gate = report["common_10hz_v1"]["gates"][
        "selection_only_max_stream_p99_gap_150ms"
    ]
    assert cadence_gate["observed_max_stream_p99_ms"] == stream_p99["max"]
    assert report["image_payloads_read"] == 0
    assert report["nuscenes_tar"]["image_payloads_read"] == 0
    assert report["can_bus_zip"]["other_payloads_read"] == 0
    assert report["can_correspondence"]["route_available_count"] == 1
    assert report["scope"]["camera_sample_data_stream_semantics"] == "PASS"
    assert report["scope"]["lidar_radar_sample_data_stream_semantics"]["status"] == (
        "NOT_EVALUATED_OUT_OF_SCOPE"
    )
    assert report["scope"]["can_zip_container_profile"][
        "raw_eocd_bounded_before_zipfile"
    ] is True
    for path, (payload, mtime_ns) in before.items():
        assert path.read_bytes() == payload
        assert path.stat().st_mtime_ns == mtime_ns


def test_common10_absolute_gap_gate_uses_actual_max_not_stream_p99() -> None:
    summary = {
        "scenes": {"scene-0001": {"sample_duration_s": 31.0}},
        "camera": {
            "source_effective_rate_hz": {
                "count": 6,
                "min": 10.0,
                "max": 10.0,
            },
            "selection_only_10hz_stream_p99_ms": {
                "count": 6,
                "min": 100.0,
                "median": 100.0,
                "p95": 100.0,
                "p99": 100.0,
                "max": 100.0,
            },
            "selection_only_10hz_gap_ms_pooled": {
                "count": 1000,
                "min": 100.0,
                "median": 100.0,
                "p95": 100.0,
                "p99": 100.0,
                "max": 300.0,
            },
            "keyframe_bundle_skew_ms": {
                "count": 1,
                "min": 0.0,
                "median": 0.0,
                "p95": 0.0,
                "p99": 0.0,
                "max": 0.0,
            },
        },
    }
    correspondence = {"route_missing_count": 0, "route_missing_scenes": []}

    qualification = inspector._common10_qualification(summary, correspondence)

    assert qualification["gates"][
        "selection_only_max_stream_p99_gap_150ms"
    ]["status"] == "PASS"
    absolute_gate = qualification["gates"]["selection_only_absolute_gap_250ms"]
    assert absolute_gate == {"status": "FAIL", "observed_max_ms": 300.0}
    assert qualification["qualified"] is False


def test_exact_six_camera_mapping_is_name_based_and_ordered(
    tmp_path: Path,
    tiny_expectations: inspector.MiniExpectations,
) -> None:
    nuscenes, can_bus = _fixtures(tmp_path)
    report = _inspect_verified(tmp_path, nuscenes, can_bus, tiny_expectations)

    mapping = report["dataset"]["camera"]["mapping"]
    assert [entry["nuscenes_channel"] for entry in mapping] == list(
        inspector.CAMERA_CHANNELS
    )
    assert [entry["common_10hz_channel"] for entry in mapping] == list(
        inspector.CAMERA_CHANNELS
    )


def test_generic_reports_bind_current_archive_bytes_and_reject_stale_report(
    tmp_path: Path,
    tiny_expectations: inspector.MiniExpectations,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    nuscenes, can_bus = _fixtures(tmp_path)
    tar_report, zip_report = _write_generic_reports(tmp_path, nuscenes, can_bus)

    passing = inspector.inspect_nuscenes_mini_adapter(
        nuscenes,
        can_bus,
        nuscenes_audit_report=tar_report,
        can_bus_audit_report=zip_report,
        expectations=tiny_expectations,
    )
    assert passing["valid"] is True
    assert passing["archive_audits"]["nuscenes"]["status"] == "PASS"
    assert passing["archive_audits"]["can_bus"]["status"] == "PASS"
    assert passing["archive_audits"]["nuscenes"]["archive_raw_bytes_hashed"] is True

    _mutate_stored_license_without_changing_size(can_bus)

    def forbidden_scan(*_args: object, **_kwargs: object) -> object:
        raise AssertionError("metadata scan must not start after stale preflight")

    monkeypatch.setattr(inspector, "_scan_nuscenes_tar", forbidden_scan)
    stale = inspector.inspect_nuscenes_mini_adapter(
        nuscenes,
        can_bus,
        nuscenes_audit_report=tar_report,
        can_bus_audit_report=zip_report,
        expectations=tiny_expectations,
    )
    assert stale["valid"] is False
    assert "actual_sha256 does not match" in stale["errors"][0]


def test_missing_optional_can_route_is_reported_without_structural_fabrication(
    tmp_path: Path,
    tiny_expectations: inspector.MiniExpectations,
) -> None:
    nuscenes, can_bus = _fixtures(tmp_path, include_route=False)

    report = _inspect_verified(tmp_path, nuscenes, can_bus, tiny_expectations)

    assert report["valid"] is True
    assert report["can_correspondence"]["route_available_count"] == 0
    assert report["can_correspondence"]["route_missing_scenes"] == [SCENE_NAME]
    route_gate = report["common_10hz_v1"]["gates"][
        "can_route_available_for_each_scene"
    ]
    assert route_gate["status"] == "FAIL"
    assert report["common_10hz_v1"]["status"] == "NOT_QUALIFIED"


def test_wrong_camera_channel_fails_closed(
    tmp_path: Path,
    tiny_expectations: inspector.MiniExpectations,
) -> None:
    nuscenes, can_bus = _fixtures(tmp_path, wrong_camera_channel=True)

    report = _inspect_verified(tmp_path, nuscenes, can_bus, tiny_expectations)

    assert report["valid"] is False
    assert report["structural_validation"]["status"] == "FAIL"
    assert "unexpected nuScenes camera channel" in report["errors"][0]


def test_camera_filename_must_resolve_to_a_regular_member(
    tmp_path: Path,
    tiny_expectations: inspector.MiniExpectations,
) -> None:
    def point_at_directory(tables: dict[str, list[dict]], _paths: list[str]) -> None:
        tables["sample_data"][0]["filename"] = "samples/CAM_FRONT"

    nuscenes, can_bus = _fixtures(tmp_path, metadata_mutator=point_at_directory)
    report = _inspect_verified(tmp_path, nuscenes, can_bus, tiny_expectations)

    assert report["valid"] is False
    assert "is not a regular TAR member" in report["errors"][0]


def test_camera_filename_cannot_be_referenced_more_than_once(
    tmp_path: Path,
    tiny_expectations: inspector.MiniExpectations,
) -> None:
    def duplicate_front_image(tables: dict[str, list[dict]], _paths: list[str]) -> None:
        tables["sample_data"][6]["filename"] = tables["sample_data"][0]["filename"]

    nuscenes, can_bus = _fixtures(tmp_path, metadata_mutator=duplicate_front_image)
    report = _inspect_verified(tmp_path, nuscenes, can_bus, tiny_expectations)

    assert report["valid"] is False
    assert "referenced more than once" in report["errors"][0]


def test_camera_filename_channel_must_match_calibration_channel(
    tmp_path: Path,
    tiny_expectations: inspector.MiniExpectations,
) -> None:
    def cross_channel(tables: dict[str, list[dict]], _paths: list[str]) -> None:
        tables["sample_data"][0]["filename"] = tables["sample_data"][1]["filename"]

    nuscenes, can_bus = _fixtures(tmp_path, metadata_mutator=cross_channel)
    report = _inspect_verified(tmp_path, nuscenes, can_bus, tiny_expectations)

    assert report["valid"] is False
    assert "samples/CAM_FRONT/<unique>.jpg" in report["errors"][0]


def test_keyframe_camera_filename_cannot_point_into_sweeps(
    tmp_path: Path,
    tiny_expectations: inspector.MiniExpectations,
) -> None:
    moved = "sweeps/CAM_FRONT/moved.jpg"

    def wrong_kind(tables: dict[str, list[dict]], _paths: list[str]) -> None:
        tables["sample_data"][0]["filename"] = moved

    nuscenes, can_bus = _fixtures(
        tmp_path,
        metadata_mutator=wrong_kind,
        extra_regular_files=((moved, b"image"),),
    )
    report = _inspect_verified(tmp_path, nuscenes, can_bus, tiny_expectations)

    assert report["valid"] is False
    assert "samples/CAM_FRONT/<unique>.jpg" in report["errors"][0]


def test_unreferenced_camera_image_member_fails_exact_accounting(
    tmp_path: Path,
    tiny_expectations: inspector.MiniExpectations,
) -> None:
    nuscenes, can_bus = _fixtures(
        tmp_path,
        extra_regular_files=(("samples/CAM_FRONT/unreferenced.jpg", b"image"),),
    )
    report = _inspect_verified(tmp_path, nuscenes, can_bus, tiny_expectations)

    assert report["valid"] is False
    assert "camera sample_data/image-member bijection failed" in report["errors"][0]
    assert "unreferenced_member_count=1" in report["errors"][0]


def test_unknown_camera_directory_member_is_not_hidden_from_accounting(
    tmp_path: Path,
    tiny_expectations: inspector.MiniExpectations,
) -> None:
    nuscenes, can_bus = _fixtures(
        tmp_path,
        extra_regular_files=(("samples/CAM_UNKNOWN/extra.jpg", b"image"),),
    )
    report = _inspect_verified(tmp_path, nuscenes, can_bus, tiny_expectations)

    assert report["valid"] is False
    assert "camera image directory contains malformed" in report["errors"][0]
    assert "CAM_UNKNOWN" in report["errors"][0]


def test_referenced_camera_image_member_must_be_nonempty(
    tmp_path: Path,
    tiny_expectations: inspector.MiniExpectations,
) -> None:
    front_image = f"samples/CAM_FRONT/{_token(500)}.jpg"
    nuscenes, can_bus = _fixtures(
        tmp_path,
        empty_image_paths=frozenset({front_image}),
    )
    report = _inspect_verified(tmp_path, nuscenes, can_bus, tiny_expectations)

    assert report["valid"] is False
    assert "image member must be non-empty" in report["errors"][0]


def test_camera_sample_data_prev_chain_must_match_timestamp_order(
    tmp_path: Path,
    tiny_expectations: inspector.MiniExpectations,
) -> None:
    def break_front_chain(tables: dict[str, list[dict]], _paths: list[str]) -> None:
        tables["sample_data"][6]["prev"] = ""

    nuscenes, can_bus = _fixtures(tmp_path, metadata_mutator=break_front_chain)
    report = _inspect_verified(tmp_path, nuscenes, can_bus, tiny_expectations)

    assert report["valid"] is False
    assert "CAM_FRONT has a broken sample_data.prev chain" in report["errors"][0]


def test_camera_sample_data_next_cannot_link_to_another_channel(
    tmp_path: Path,
    tiny_expectations: inspector.MiniExpectations,
) -> None:
    def link_front_to_back(tables: dict[str, list[dict]], _paths: list[str]) -> None:
        tables["sample_data"][0]["next"] = tables["sample_data"][1]["token"]

    nuscenes, can_bus = _fixtures(tmp_path, metadata_mutator=link_front_to_back)
    report = _inspect_verified(tmp_path, nuscenes, can_bus, tiny_expectations)

    assert report["valid"] is False
    assert "CAM_FRONT has a broken sample_data.next chain" in report["errors"][0]


def test_per_channel_camera_frame_counts_are_enforced(
    tmp_path: Path,
    tiny_expectations: inspector.MiniExpectations,
) -> None:
    moved = "sweeps/CAM_BACK/moved-front-record.jpg"

    def move_front_to_back(tables: dict[str, list[dict]], _paths: list[str]) -> None:
        tables["sample_data"][0]["calibrated_sensor_token"] = _token(301)
        tables["sample_data"][0]["filename"] = moved
        tables["sample_data"][0]["is_key_frame"] = False

    nuscenes, can_bus = _fixtures(
        tmp_path,
        metadata_mutator=move_front_to_back,
        extra_regular_files=((moved, b"image"),),
    )
    report = _inspect_verified(tmp_path, nuscenes, can_bus, tiny_expectations)

    assert report["valid"] is False
    assert "CAM_FRONT' has 2 frames, expected 3" in report["errors"][0]


def test_every_declared_camera_calibration_must_be_referenced(
    tmp_path: Path,
    tiny_expectations: inspector.MiniExpectations,
) -> None:
    def add_unused_calibrations(
        tables: dict[str, list[dict]],
        _paths: list[str],
    ) -> None:
        for index, calibration in enumerate(list(tables["calibrated_sensor"])):
            extra = dict(calibration)
            extra["token"] = _token(900 + index)
            tables["calibrated_sensor"].append(extra)

    nuscenes, can_bus = _fixtures(
        tmp_path,
        metadata_mutator=add_unused_calibrations,
    )
    expectations = replace(
        tiny_expectations,
        calibrated_sensor_count=12,
        camera_calibrations_per_channel=2,
    )
    report = _inspect_verified(tmp_path, nuscenes, can_bus, expectations)

    assert report["valid"] is False
    assert "does not reference every declared calibration" in report["errors"][0]


def test_both_full_archive_audit_reports_are_mandatory_before_scan(
    tmp_path: Path,
    tiny_expectations: inspector.MiniExpectations,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    nuscenes, can_bus = _fixtures(tmp_path)

    def forbidden_scan(*_args: object, **_kwargs: object) -> object:
        raise AssertionError("archive scan must not start without both reports")

    monkeypatch.setattr(inspector, "_scan_nuscenes_tar", forbidden_scan)
    missing_both = inspector.inspect_nuscenes_mini_adapter(
        nuscenes,
        can_bus,
        expectations=tiny_expectations,
    )
    tar_report, _zip_report = _write_generic_reports(tmp_path, nuscenes, can_bus)
    missing_one = inspector.inspect_nuscenes_mini_adapter(
        nuscenes,
        can_bus,
        nuscenes_audit_report=tar_report,
        expectations=tiny_expectations,
    )

    for report in (missing_both, missing_one):
        assert report["valid"] is False
        assert "both passing generic TAR" in report["errors"][0]


def test_tar_symlink_is_rejected_without_following_it(
    tmp_path: Path,
    tiny_expectations: inspector.MiniExpectations,
) -> None:
    nuscenes, can_bus = _fixtures(tmp_path, add_symlink=True)

    del can_bus, tiny_expectations
    with pytest.raises(inspector.InspectionFailure, match="forbidden link/special member"):
        inspector._scan_nuscenes_tar(nuscenes, inspector.DEFAULT_LIMITS)


def test_member_resource_cap_fails_closed(
    tmp_path: Path,
    tiny_expectations: inspector.MiniExpectations,
) -> None:
    nuscenes, can_bus = _fixtures(tmp_path)
    limits = replace(inspector.DEFAULT_LIMITS, max_tar_members=2)

    report = _inspect_verified(
        tmp_path,
        nuscenes,
        can_bus,
        tiny_expectations,
        limits=limits,
    )

    assert report["valid"] is False
    assert "member count exceeds adapter limits" in report["errors"][0]


def test_zip_eocd_resource_cap_is_checked_before_zipfile_constructor(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    can_bus = tmp_path / "can_bus.zip"
    _write_can_zip(can_bus)

    def forbidden_constructor(*_args: object, **_kwargs: object) -> object:
        raise AssertionError("ZipFile constructor must not run before the raw EOCD bound")

    monkeypatch.setattr(inspector.zipfile, "ZipFile", forbidden_constructor)
    limits = replace(inspector.DEFAULT_LIMITS, max_zip_central_directory_bytes=1)
    with pytest.raises(inspector.InspectionFailure, match="EOCD central directory"):
        inspector._scan_can_zip(can_bus, frozenset({SCENE_NAME}), limits)


def test_native_selection_keeps_source_timestamps_and_exposes_long_gaps() -> None:
    source = [index * 83_333 for index in range(25)]

    selected = inspector._selection_only_10hz(source)
    gaps_ms = [
        (later - earlier) / 1_000.0
        for earlier, later in zip(selected, selected[1:])
    ]

    assert set(selected).issubset(source)
    assert len(selected) == len(set(selected))
    assert max(gaps_ms) > 150.0


def test_cli_failure_is_json_stdout_without_traceback(capsys: pytest.CaptureFixture[str]) -> None:
    exit_code = inspector.main(
        [
            "/missing-mini.tgz",
            "/missing-can.zip",
            "--nuscenes-audit-report",
            "/missing-tar-audit.json",
            "--can-bus-audit-report",
            "/missing-zip-audit.json",
        ]
    )
    captured = capsys.readouterr()
    report = json.loads(captured.out)

    assert exit_code == 2
    assert report["valid"] is False
    assert report["status"] == "FAIL"
    assert "Traceback" not in captured.out
    assert captured.err == ""


def test_direct_script_help_works_from_repository_root() -> None:
    completed = subprocess.run(
        [sys.executable, str(SCRIPT), "--help"],
        cwd=SCRIPT.parents[2],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    assert "nuScenes" in completed.stdout


def test_cli_argument_error_is_json_stdout(capsys: pytest.CaptureFixture[str]) -> None:
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        exit_code = inspector.main([])
    captured = capsys.readouterr()
    report = json.loads(captured.out)

    assert exit_code == 2
    assert report["valid"] is False
    assert "argument error" in report["errors"][0]
    assert captured.err == ""
