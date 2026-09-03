import gzip
import hashlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tarfile

import pytest

from scripts.e2e import inspect_bench2drive_mini_archives as inspector
from scripts.e2e.inspect_bench2drive_mini_archives import inspect_archive_set
from scripts.e2e.verify_bench2drive_mini import ArchiveSpec


SCENE = "VehicleTurningRoute_Town15_Route443_Weather1"
ARCHIVE_NAME = f"{SCENE}.tar.gz"
SCRIPT = Path(__file__).parents[1] / "scripts/e2e/inspect_bench2drive_mini_archives.py"


def _jpeg(width: int = 1600, height: int = 900) -> bytes:
    return (
        b"\xff\xd8"
        b"\xff\xe0\x00\x04\x00\x00"
        + b"\xff\xc0\x00\x11\x08"
        + height.to_bytes(2, "big")
        + width.to_bytes(2, "big")
        + b"\x03\x01\x11\x00\x02\x11\x00\x03\x11\x00"
        + b"\xff\xd9"
    )


def _matrix(size: int) -> list[list[float]]:
    return [
        [1.0 if row == column else 0.0 for column in range(size)]
        for row in range(size)
    ]


def _annotation(
    command: int = 2,
    timestamp_ns: int | None = None,
    nonfinite_values: int = 0,
) -> bytes:
    camera = {
        "cam2ego": _matrix(4),
        "fov": 90.0,
        "image_size_x": 1600,
        "image_size_y": 900,
        "intrinsic": _matrix(3),
        "location": [0.0, 0.0, 0.0],
        "rotation": [0.0, 0.0, 0.0],
        "world2cam": _matrix(4),
    }
    value = {
        "acceleration": [0.0, 0.0, 0.0],
        "angular_velocity": [0.0, 0.0, 0.0],
        "bounding_boxes": [
            {
                "quality": (
                    float("nan")
                    if index % 3 == 0
                    else float("inf") if index % 3 == 1 else float("-inf")
                )
            }
            for index in range(nonfinite_values)
        ],
        "brake": 0.0,
        "command_far": command,
        "command_near": command,
        "next_command": command,
        "only_ap_brake": False,
        "reverse": False,
        "sensors": {
            sensor: dict(camera)
            for sensor in inspector.ALL_RGB_DIRECTORIES.values()
        },
        "should_brake": False,
        "speed": 1.0,
        "steer": 0.0,
        "theta": 0.0,
        "throttle": 0.2,
        "weather": {},
        "x": 0.0,
        "x_command_far": 10.0,
        "x_command_near": 5.0,
        "x_target": 10.0,
        "y": 0.0,
        "y_command_far": 0.0,
        "y_command_near": 0.0,
        "y_target": 0.0,
    }
    if timestamp_ns is not None:
        value["timestamp_ns"] = timestamp_ns
    return gzip.compress(json.dumps(value, sort_keys=True).encode("utf-8"))


def _add_payload(archive: tarfile.TarFile, name: str, payload: bytes) -> None:
    member = tarfile.TarInfo(name)
    member.size = len(payload)
    archive.addfile(member, io.BytesIO(payload))


def _write_archive(
    directory: Path,
    *,
    frames: tuple[int, ...] = (0, 1),
    camera_frames: dict[str, tuple[int, ...]] | None = None,
    dimensions: dict[str, tuple[int, int]] | None = None,
    missing_calibration: str | None = None,
    timestamps: bool = False,
    nonfinite_values_per_frame: int = 0,
    annotation_payloads: dict[int, bytes] | None = None,
    extra_members: tuple[tuple[str, bytes], ...] = (),
) -> dict[str, ArchiveSpec]:
    archive_path = directory / ARCHIVE_NAME
    camera_frames = camera_frames or {}
    dimensions = dimensions or {}
    with tarfile.open(archive_path, "w:gz", format=tarfile.PAX_FORMAT) as archive:
        for camera in inspector.ALL_RGB_DIRECTORIES:
            selected_frames = camera_frames.get(camera, frames)
            width, height = dimensions.get(camera, (1600, 900))
            for frame in selected_frames:
                _add_payload(
                    archive,
                    f"./{SCENE}/camera/{camera}/{frame:05d}.jpg",
                    _jpeg(width, height),
                )
        for frame in frames:
            payload = (annotation_payloads or {}).get(frame)
            if payload is None:
                payload = _annotation(
                    timestamp_ns=frame * 100_000_000 if timestamps else None,
                    nonfinite_values=nonfinite_values_per_frame,
                )
            if missing_calibration is not None:
                decoded = json.loads(gzip.decompress(payload))
                decoded["sensors"].pop(missing_calibration)
                payload = gzip.compress(json.dumps(decoded).encode("utf-8"))
            _add_payload(
                archive,
                f"./{SCENE}/anno/{frame:05d}.json.gz",
                payload,
            )
        for name, payload in extra_members:
            _add_payload(archive, name, payload)
    payload = archive_path.read_bytes()
    return {
        ARCHIVE_NAME: ArchiveSpec(
            size=len(payload),
            sha256=hashlib.sha256(payload).hexdigest(),
        )
    }


def test_valid_structure_passes_but_absent_timestamps_do_not_qualify_10hz(
    tmp_path: Path,
) -> None:
    manifest = _write_archive(tmp_path)

    report = inspect_archive_set(tmp_path, manifest)
    archive = report["archives"][0]

    assert report["status"] == "PASS"
    assert report["valid"] is True
    assert report["common_10hz_qualification"]["status"] == (
        "NOT_QUALIFIED_COMMON10"
    )
    assert archive["frames"] == {
        "continuous_from_zero": True,
        "count": 2,
        "first": "00000",
        "last": "00001",
        "missing": [],
        "missing_count": 0,
    }
    assert len(archive["rgb_surround"]) == 6
    assert archive["rgb_surround"][0]["jpeg_dimensions"] == [
        {"height": 900, "width": 1600}
    ]
    assert archive["annotations"]["native_timestamp_fields_union"] == []
    assert archive["annotations"]["non_standard_json_constants"]["count"] == 0
    assert archive["maneuver_hint"]["value"] == "turn"
    assert archive["maneuver_hint"]["command_values"] == [2]
    assert archive["conversion_readiness"]["status"] == "BLOCKED_FAIL_CLOSED"
    assert archive["conversion_readiness"]["blocking_reasons"] == [
        "NATIVE_TIMESTAMP_10HZ_EVIDENCE_MISSING",
        "PREPARED_DATASET_CANONICAL_VALIDATION_NOT_RUN",
    ]


def test_two_timestamped_frames_do_not_prove_cadence_or_common10(
    tmp_path: Path,
) -> None:
    manifest = _write_archive(tmp_path, timestamps=True)

    report = inspect_archive_set(tmp_path, manifest)
    archive = report["archives"][0]
    qualification = archive["common_10hz_qualification"]

    assert report["valid"] is True
    assert qualification["status"] == "NOT_QUALIFIED_COMMON10"
    assert qualification["qualified"] is False
    assert qualification["native_timestamp_cadence_evidence"]["status"] == (
        "NOT_PROVEN"
    )
    assert archive["conversion_readiness"]["ready"] is False


def test_long_native_10hz_cadence_is_only_evidence_until_canonical_validation(
    tmp_path: Path,
) -> None:
    manifest = _write_archive(
        tmp_path,
        frames=tuple(range(301)),
        timestamps=True,
    )

    report = inspect_archive_set(tmp_path, manifest)
    archive = report["archives"][0]
    qualification = archive["common_10hz_qualification"]
    cadence = qualification["native_timestamp_cadence_evidence"]

    assert report["valid"] is True
    assert cadence["status"] == "PASS"
    assert cadence["evidence_field"] == "timestamp_ns"
    assert cadence["passing_explicit_unit_field"]["duration_s"] == pytest.approx(30.0)
    assert qualification["status"] == "NOT_QUALIFIED_COMMON10"
    assert qualification["canonical_validator_required"] is True
    assert archive["conversion_readiness"]["status"] == "BLOCKED_FAIL_CLOSED"
    assert archive["conversion_readiness"]["blocking_reasons"] == [
        "PREPARED_DATASET_CANONICAL_VALIDATION_NOT_RUN"
    ]
    assert report["conversion_readiness"]["ready"] is False


def test_nonfinite_json_constants_are_counted_without_hiding_structure_pass(
    tmp_path: Path,
) -> None:
    manifest = _write_archive(
        tmp_path,
        timestamps=True,
        nonfinite_values_per_frame=3,
    )

    report = inspect_archive_set(tmp_path, manifest)
    archive = report["archives"][0]
    constants = archive["annotations"]["non_standard_json_constants"]

    assert report["status"] == "PASS"
    assert archive["status"] == "PASS"
    assert archive["common_10hz_qualification"]["status"] == (
        "NOT_QUALIFIED_COMMON10"
    )
    assert constants["status"] == "PRESENT_BLOCKING_CONVERSION"
    assert constants["count"] == 6
    assert constants["annotation_frame_count"] == 2
    assert constants["by_token"] == {
        "-Infinity": 2,
        "Infinity": 2,
        "NaN": 2,
    }
    assert constants["samples"][0] == {
        "frame_id": "00000",
        "json_pointer": "$/bounding_boxes/0/quality",
        "token": "NaN",
    }
    assert archive["conversion_readiness"]["status"] == "BLOCKED_FAIL_CLOSED"
    assert archive["conversion_readiness"]["blocking_reasons"] == [
        "NATIVE_TIMESTAMP_10HZ_EVIDENCE_MISSING",
        "NON_STANDARD_JSON_NUMERIC_CONSTANTS_REQUIRE_POLICY",
        "PREPARED_DATASET_CANONICAL_VALIDATION_NOT_RUN",
    ]
    assert report["totals"]["non_standard_json_constant_count"] == 6
    assert report["totals"]["archives_with_non_standard_json_constants"] == 1
    assert report["non_standard_json_constants"]["count"] == 6
    assert report["non_standard_json_constants"]["annotation_frame_count"] == 2
    assert report["non_standard_json_constants"]["by_token"] == {
        "-Infinity": 2,
        "Infinity": 2,
        "NaN": 2,
    }
    assert report["non_standard_json_constants"]["samples"][0]["archive"] == (
        ARCHIVE_NAME
    )
    assert report["conversion_readiness"]["ready"] is False
    assert report["conversion_readiness"]["affected_archives"] == [ARCHIVE_NAME]


def test_nonfinite_json_path_samples_are_bounded_but_count_is_exact(
    tmp_path: Path,
) -> None:
    constants_per_frame = inspector.MAX_NONFINITE_SAMPLES_PER_ARCHIVE + 7
    manifest = _write_archive(
        tmp_path,
        timestamps=True,
        nonfinite_values_per_frame=constants_per_frame,
    )

    report = inspect_archive_set(tmp_path, manifest)
    constants = report["archives"][0]["annotations"][
        "non_standard_json_constants"
    ]

    assert report["valid"] is True
    assert constants["count"] == constants_per_frame * 2
    assert len(constants["samples"]) <= inspector.MAX_NONFINITE_SAMPLES_PER_ARCHIVE
    assert constants["samples_truncated"] is True


def test_one_camera_frame_set_mismatch_fails_closed(tmp_path: Path) -> None:
    manifest = _write_archive(tmp_path, camera_frames={"rgb_front": (0,)})

    report = inspect_archive_set(tmp_path, manifest)

    assert report["valid"] is False
    errors = report["archives"][0]["errors"]
    assert any("rgb_front frame set differs" in error for error in errors)
    assert report["totals"]["annotation_frame_count_observed"] == 2
    assert report["totals"]["surround_rgb_image_count_observed"] == 11


def test_common_gap_in_all_frame_sets_still_fails_continuity(tmp_path: Path) -> None:
    manifest = _write_archive(tmp_path, frames=(0, 2))

    report = inspect_archive_set(tmp_path, manifest)

    assert report["valid"] is False
    assert any(
        "annotation frame IDs are not continuous" in error
        for error in report["archives"][0]["errors"]
    )


def test_inconsistent_jpeg_dimensions_fail_closed(tmp_path: Path) -> None:
    manifest = _write_archive(tmp_path, dimensions={"rgb_front": (800, 600)})

    report = inspect_archive_set(tmp_path, manifest)

    assert report["valid"] is False
    assert any(
        "rgb_front JPEG dimensions" in error
        for error in report["archives"][0]["errors"]
    )


def test_missing_camera_calibration_fails_closed(tmp_path: Path) -> None:
    manifest = _write_archive(tmp_path, missing_calibration="CAM_FRONT")

    report = inspect_archive_set(tmp_path, manifest)

    assert report["valid"] is False
    assert any(
        "lacks calibration for CAM_FRONT" in error
        for error in report["archives"][0]["errors"]
    )


def test_annotation_parser_rejects_duplicate_json_keys() -> None:
    payload = gzip.compress(b'{"speed": 1, "speed": 2}')

    with pytest.raises(inspector.InspectionFailure, match="repeats object key"):
        inspector._read_annotation(io.BytesIO(payload), len(payload))


def test_deep_annotation_json_fails_closed_in_archive_report(tmp_path: Path) -> None:
    deeply_nested = gzip.compress(b"[" * 2_000 + b"0" + b"]" * 2_000)
    manifest = _write_archive(tmp_path, annotation_payloads={0: deeply_nested})

    report = inspect_archive_set(tmp_path, manifest)

    assert report["valid"] is False
    assert any(
        "annotation gzip/JSON is invalid" in error
        for error in report["archives"][0]["errors"]
    )


def test_huge_calibration_integer_fails_closed_without_overflow(tmp_path: Path) -> None:
    payload = json.loads(gzip.decompress(_annotation()))
    payload["sensors"]["CAM_FRONT"]["fov"] = 10**400
    malformed = gzip.compress(json.dumps(payload).encode("utf-8"))
    manifest = _write_archive(tmp_path, annotation_payloads={0: malformed})

    report = inspect_archive_set(tmp_path, manifest)

    assert report["valid"] is False
    assert any(
        "calibration CAM_FRONT has invalid fov" in error
        for error in report["archives"][0]["errors"]
    )


def test_empty_timestamp_inventory_has_no_cadence_evidence() -> None:
    report = inspector._native_timestamp_cadence_evidence(set(), {}, {})

    assert report["status"] == "NOT_PROVEN"
    assert report["native_timestamp_availability"] == "NONE"


@pytest.mark.parametrize(
    "extra_members",
    [
        ((f"./{SCENE}/anno/00000.json.gz", _annotation()),),
        (("../outside.jpg", _jpeg()),),
        ((f"./{SCENE}/camera/rgb_unknown/00000.jpg", _jpeg()),),
    ],
)
def test_duplicate_unsafe_and_unexpected_rgb_members_fail_closed(
    tmp_path: Path,
    extra_members: tuple[tuple[str, bytes], ...],
) -> None:
    manifest = _write_archive(tmp_path, extra_members=extra_members)

    report = inspect_archive_set(tmp_path, manifest)

    assert report["valid"] is False


def test_exact_archive_directory_rejects_missing_and_extra_entries(tmp_path: Path) -> None:
    manifest = _write_archive(tmp_path)
    (tmp_path / "stale.part").write_bytes(b"stale")

    report = inspect_archive_set(tmp_path, manifest)

    assert report["valid"] is False
    assert any("unexpected archive-directory entries" in error for error in report["errors"])


def test_cli_returns_zero_for_structural_pass_even_without_native_timestamps(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    manifest = _write_archive(tmp_path)
    monkeypatch.setattr(inspector, "OFFICIAL_LEGACY_MINI_ARCHIVES", manifest)

    result = inspector.main([str(tmp_path)])
    report = json.loads(capsys.readouterr().out)

    assert result == 0
    assert report["status"] == "PASS"
    assert report["common_10hz_qualification"]["status"] == (
        "NOT_QUALIFIED_COMMON10"
    )


def test_cli_returns_two_for_structural_failure(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    manifest = _write_archive(tmp_path, camera_frames={"rgb_front": (0,)})
    monkeypatch.setattr(inspector, "OFFICIAL_LEGACY_MINI_ARCHIVES", manifest)

    result = inspector.main([str(tmp_path)])
    report = json.loads(capsys.readouterr().out)

    assert result == 2
    assert report["status"] == "FAIL"


def test_direct_script_help_works_from_repository_root() -> None:
    completed = subprocess.run(
        [sys.executable, str(SCRIPT), "--help"],
        cwd=SCRIPT.parents[2],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    assert "Bench2Drive" in completed.stdout
