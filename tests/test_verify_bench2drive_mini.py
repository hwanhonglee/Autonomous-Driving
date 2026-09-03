import hashlib
import json
from pathlib import Path

import pytest

from scripts.e2e import verify_bench2drive_mini as verifier
from scripts.e2e.verify_bench2drive_mini import ArchiveSpec
from scripts.e2e.verify_bench2drive_mini import OFFICIAL_LEGACY_MINI_ARCHIVES
from scripts.e2e.verify_bench2drive_mini import verify_archive_directory


def _spec(payload: bytes) -> ArchiveSpec:
    return ArchiveSpec(size=len(payload), sha256=hashlib.sha256(payload).hexdigest())


def _tiny_manifest() -> dict[str, ArchiveSpec]:
    return {
        "FirstScenario_Town01_Route1_Weather1.tar.gz": _spec(b"first archive"),
        "SecondScenario_Town02_Route2_Weather2.tar.gz": _spec(b"second archive"),
    }


def _write_valid_set(directory: Path, manifest: dict[str, ArchiveSpec]) -> None:
    payloads = {
        "FirstScenario_Town01_Route1_Weather1.tar.gz": b"first archive",
        "SecondScenario_Town02_Route2_Weather2.tar.gz": b"second archive",
    }
    for name in manifest:
        (directory / name).write_bytes(payloads[name])


def test_embedded_manifest_matches_official_legacy_mini_contract() -> None:
    expected = {
        "AccidentTwoWays_Town12_Route1444_Weather0.tar.gz": (
            341_845_960,
            "2690cb0053a2e9208f50f9d48c4a1c5befae4179c07e6fb8f06dc4434fb82178",
        ),
        "Accident_Town03_Route156_Weather0.tar.gz": (
            264_497_958,
            "09780cfaae07b13f65f11addfffbda8dea7723a6dfbbe0b49f493303805d24fa",
        ),
        "ConstructionObstacle_Town05_Route68_Weather8.tar.gz": (
            287_017_092,
            "7a1e93168780f5dc4d38b6b5b357e74a62736c935adc3fa74b1ed35028c61e76",
        ),
        "ControlLoss_Town11_Route401_Weather11.tar.gz": (
            99_078_817,
            "9e3abe0c9b25f0e50e597ac0f62a5cf681acd9631e439b23cabac77b85355043",
        ),
        "DynamicObjectCrossing_Town02_Route13_Weather6.tar.gz": (
            280_176_944,
            "b040f5993726dfb3a1dc1aaf5e2e4f93404a0124272d6f47f2469fca53ca25f2",
        ),
        "HardBreakRoute_Town01_Route30_Weather3.tar.gz": (
            426_576_889,
            "2f6593d05e288a88cf37d8043d1e448842a4b1ceaed5544fe13100c174a4cb04",
        ),
        "OppositeVehicleTakingPriority_Town13_Route600_Weather2.tar.gz": (
            177_697_783,
            "757a8e7415081447c2d172592723de194b05ad3fb9416e9fc215844e1dac58dc",
        ),
        "ParkedObstacle_Town10HD_Route371_Weather7.tar.gz": (
            264_725_091,
            "78464d659875f8b2bc5901c763bc9ba154c3eaf021196f4d6077d9ef17c705e4",
        ),
        "VehicleTurningRoute_Town15_Route443_Weather1.tar.gz": (
            487_312_838,
            "97a066963571c10fd91ec0ee45bd5fc598e79ea2f1cf153e8617887ad83b2b5a",
        ),
        "YieldToEmergencyVehicle_Town04_Route165_Weather7.tar.gz": (
            183_804_370,
            "feb52345f09a5358728dd2ed32db93cf64a5d2b0d04d4719a38d408c60c212df",
        ),
    }

    assert {
        name: (item.size, item.sha256)
        for name, item in OFFICIAL_LEGACY_MINI_ARCHIVES.items()
    } == expected
    assert sum(item.size for item in OFFICIAL_LEGACY_MINI_ARCHIVES.values()) == (
        2_812_733_742
    )


def test_valid_directory_passes_with_exact_sizes_and_hashes(tmp_path: Path) -> None:
    manifest = _tiny_manifest()
    _write_valid_set(tmp_path, manifest)

    report = verify_archive_directory(tmp_path, manifest)

    assert report["valid"] is True
    assert report["status"] == "PASS"
    assert report["verified_archive_count"] == 2
    assert report["verified_total_bytes"] == sum(item.size for item in manifest.values())
    assert report["missing"] == []
    assert report["extra"] == []
    assert report["errors"] == []


@pytest.mark.parametrize(
    ("extra_name", "expected_field"),
    [
        ("unexpected.tar.gz", "extra"),
        ("download.tar.gz.part", "partial"),
        ("download.tar.gz.part.001", "partial"),
    ],
)
def test_unexpected_and_partial_entries_fail_closed(
    tmp_path: Path, extra_name: str, expected_field: str
) -> None:
    manifest = _tiny_manifest()
    _write_valid_set(tmp_path, manifest)
    (tmp_path / extra_name).write_bytes(b"unexpected")

    report = verify_archive_directory(tmp_path, manifest)

    assert report["valid"] is False
    assert extra_name in report["extra"]
    assert extra_name in report[expected_field]


def test_missing_archive_fails_closed(tmp_path: Path) -> None:
    manifest = _tiny_manifest()
    missing = "SecondScenario_Town02_Route2_Weather2.tar.gz"
    (tmp_path / "FirstScenario_Town01_Route1_Weather1.tar.gz").write_bytes(
        b"first archive"
    )

    report = verify_archive_directory(tmp_path, manifest)

    assert report["valid"] is False
    assert report["missing"] == [missing]
    assert report["verified_archive_count"] == 1


def test_expected_archive_symlink_is_rejected_without_following_it(tmp_path: Path) -> None:
    manifest = _tiny_manifest()
    _write_valid_set(tmp_path, manifest)
    name = "FirstScenario_Town01_Route1_Weather1.tar.gz"
    target = tmp_path / "target-outside-directory"
    target.write_bytes(b"first archive")
    (tmp_path / name).unlink()
    (tmp_path / name).symlink_to(target)

    report = verify_archive_directory(tmp_path, manifest)

    assert report["valid"] is False
    assert name in report["symlinks"]
    failed = next(item for item in report["files"] if item["name"] == name)
    assert failed["actual_sha256"] is None
    assert "symlink is forbidden" in failed["errors"]


def test_extra_symlink_is_rejected(tmp_path: Path) -> None:
    manifest = _tiny_manifest()
    _write_valid_set(tmp_path, manifest)
    (tmp_path / "unexpected-link").symlink_to(tmp_path / next(iter(manifest)))

    report = verify_archive_directory(tmp_path, manifest)

    assert report["valid"] is False
    assert report["symlinks"] == ["unexpected-link"]
    assert report["extra"] == ["unexpected-link"]


def test_size_mismatch_is_reported(tmp_path: Path) -> None:
    manifest = _tiny_manifest()
    _write_valid_set(tmp_path, manifest)
    name = "FirstScenario_Town01_Route1_Weather1.tar.gz"
    (tmp_path / name).write_bytes(b"wrong length")

    report = verify_archive_directory(tmp_path, manifest)
    failed = next(item for item in report["files"] if item["name"] == name)

    assert report["valid"] is False
    assert "size mismatch" in failed["errors"]
    assert "SHA-256 mismatch" in failed["errors"]


def test_hash_mismatch_is_reported_when_size_matches(tmp_path: Path) -> None:
    manifest = _tiny_manifest()
    _write_valid_set(tmp_path, manifest)
    name = "FirstScenario_Town01_Route1_Weather1.tar.gz"
    (tmp_path / name).write_bytes(b"xxxxxxxxxxxxx")

    report = verify_archive_directory(tmp_path, manifest)
    failed = next(item for item in report["files"] if item["name"] == name)

    assert len(b"xxxxxxxxxxxxx") == manifest[name].size
    assert report["valid"] is False
    assert "size mismatch" not in failed["errors"]
    assert "SHA-256 mismatch" in failed["errors"]


def test_missing_path_regular_file_and_directory_symlink_are_rejected(
    tmp_path: Path,
) -> None:
    manifest = _tiny_manifest()
    missing = verify_archive_directory(tmp_path / "missing", manifest)
    regular = tmp_path / "regular-file"
    regular.write_bytes(b"not a directory")
    regular_report = verify_archive_directory(regular, manifest)
    real_directory = tmp_path / "archives"
    real_directory.mkdir()
    linked_directory = tmp_path / "archives-link"
    linked_directory.symlink_to(real_directory, target_is_directory=True)
    linked_report = verify_archive_directory(linked_directory, manifest)

    assert missing["valid"] is False
    assert "cannot inspect archive directory" in missing["errors"][0]
    assert regular_report["errors"] == ["archive directory is not a directory"]
    assert linked_report["errors"] == ["archive directory must not be a symlink"]


def test_json_cli_is_machine_readable_and_returns_zero_for_a_valid_set(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    manifest = _tiny_manifest()
    _write_valid_set(tmp_path, manifest)
    monkeypatch.setattr(verifier, "OFFICIAL_LEGACY_MINI_ARCHIVES", manifest)

    result = verifier.main([str(tmp_path), "--json"])
    output = json.loads(capsys.readouterr().out)

    assert result == 0
    assert output["valid"] is True
    assert output["read_only"] is True


def test_json_cli_returns_two_for_failure(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    manifest = _tiny_manifest()
    monkeypatch.setattr(verifier, "OFFICIAL_LEGACY_MINI_ARCHIVES", manifest)

    result = verifier.main([str(tmp_path), "--json"])
    output = json.loads(capsys.readouterr().out)

    assert result == 2
    assert output["status"] == "FAIL"
    assert sorted(output["missing"]) == sorted(manifest)
