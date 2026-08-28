import hashlib
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from scripts.e2e.vad_calibration_contract import load_calibration
from scripts.e2e.vad_commissioning_contract import CommissioningError
from scripts.e2e.vad_commissioning_contract import load_commissioning
from scripts.e2e.vad_training_data_contract import load_profile
from vad_training_test_utils import calibration_document
from vad_training_test_utils import complete_profile


ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / "autoware_e2e_vad_launch/config/vad_real_data_collection.yaml"


def write_commissioning(tmp_path: Path) -> tuple[Path, Path, dict, dict]:
    profile_data = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
    calibration_path = tmp_path / "calibration.yaml"
    complete_profile(profile_data, calibration_path)
    calibration_path.write_text(
        yaml.safe_dump(calibration_document(profile_data), sort_keys=False),
        encoding="utf-8",
    )
    profile_path = tmp_path / "profile.yaml"
    profile_path.write_text(yaml.safe_dump(profile_data, sort_keys=False), encoding="utf-8")
    profile = load_profile(profile_path)
    calibration = load_calibration(calibration_path)
    commissioning = Path(profile["provenance"]["commissioning_file"])
    rectification = Path(profile["provenance"]["rectification_config_file"])
    return commissioning, rectification, calibration, profile


def test_commissioning_binds_calibration_rectification_and_evidence(
    tmp_path: Path,
) -> None:
    commissioning, rectification, calibration, profile = write_commissioning(tmp_path)

    loaded = load_commissioning(commissioning, calibration, rectification, profile)

    assert loaded["approved"] is True
    assert loaded["_rectification_config_sha256"] == hashlib.sha256(
        rectification.read_bytes()
    ).hexdigest()
    assert set(loaded["_evidence_files"]) == {
        "checkerboard_evidence",
        "reprojection_report",
        "projection_preview",
        "device_inventory",
        "device_status_adapter",
        "device_status_adapter_config",
    }

    checkerboard = Path(loaded["_evidence_files"]["checkerboard_evidence"]["path"])
    checkerboard.write_bytes(b"tampered")
    with pytest.raises(CommissioningError, match="checkerboard_dataset_sha256"):
        load_commissioning(commissioning, calibration, rectification, profile)


def test_commissioning_rejects_more_than_one_pixel_error(tmp_path: Path) -> None:
    commissioning, rectification, calibration, profile = write_commissioning(tmp_path)
    document = yaml.safe_load(commissioning.read_text(encoding="utf-8"))
    report_path = commissioning.parent / document["reprojection_report_file"]
    report = yaml.safe_load(report_path.read_text(encoding="utf-8"))
    report["maximum_reprojection_error_px"] = 1.1
    report["cameras"][0]["maximum_reprojection_error_px"] = 1.1
    report_path.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
    document["maximum_reprojection_error_px"] = 1.1
    document["reprojection_report_sha256"] = hashlib.sha256(
        report_path.read_bytes()
    ).hexdigest()
    commissioning.write_text(yaml.safe_dump(document), encoding="utf-8")

    with pytest.raises(CommissioningError, match="one-pixel limit"):
        load_commissioning(commissioning, calibration, rectification, profile)


def test_commissioning_requires_explicit_utc_approval_time(tmp_path: Path) -> None:
    commissioning, rectification, calibration, profile = write_commissioning(tmp_path)
    document = yaml.safe_load(commissioning.read_text(encoding="utf-8"))
    document["approved_utc"] = "2026-08-27T00:00:00"
    commissioning.write_text(yaml.safe_dump(document), encoding="utf-8")

    with pytest.raises(CommissioningError, match="explicit UTC offset"):
        load_commissioning(commissioning, calibration, rectification, profile)


def test_commissioning_inventory_serials_match_calibration(tmp_path: Path) -> None:
    commissioning, rectification, calibration, profile = write_commissioning(tmp_path)
    document = yaml.safe_load(commissioning.read_text(encoding="utf-8"))
    inventory_path = commissioning.parent / document["device_inventory_file"]
    inventory = yaml.safe_load(inventory_path.read_text(encoding="utf-8"))
    inventory["cameras"][0]["serial"] = "wrong_camera"
    inventory_path.write_text(yaml.safe_dump(inventory, sort_keys=False), encoding="utf-8")
    document["device_inventory_sha256"] = hashlib.sha256(
        inventory_path.read_bytes()
    ).hexdigest()
    commissioning.write_text(yaml.safe_dump(document), encoding="utf-8")

    with pytest.raises(CommissioningError, match="CAM_FRONT inventory serial"):
        load_commissioning(commissioning, calibration, rectification, profile)


def test_commissioning_rejects_duplicate_inventory_device_ids(tmp_path: Path) -> None:
    commissioning, rectification, calibration, profile = write_commissioning(tmp_path)
    document = yaml.safe_load(commissioning.read_text(encoding="utf-8"))
    inventory_path = commissioning.parent / document["device_inventory_file"]
    inventory = yaml.safe_load(inventory_path.read_text(encoding="utf-8"))
    inventory["cameras"][1]["sdk_device_id"] = inventory["cameras"][0]["sdk_device_id"]
    inventory_path.write_text(yaml.safe_dump(inventory, sort_keys=False), encoding="utf-8")
    document["device_inventory_sha256"] = hashlib.sha256(
        inventory_path.read_bytes()
    ).hexdigest()
    commissioning.write_text(yaml.safe_dump(document), encoding="utf-8")

    with pytest.raises(CommissioningError, match="SDK device IDs must be unique"):
        load_commissioning(commissioning, calibration, rectification, profile)


def test_commissioning_reprojection_aggregates_must_be_consistent(
    tmp_path: Path,
) -> None:
    commissioning, rectification, calibration, profile = write_commissioning(tmp_path)
    document = yaml.safe_load(commissioning.read_text(encoding="utf-8"))
    report_path = commissioning.parent / document["reprojection_report_file"]
    report = yaml.safe_load(report_path.read_text(encoding="utf-8"))
    report["sample_count"] += 1
    report_path.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
    document["reprojection_report_sha256"] = hashlib.sha256(
        report_path.read_bytes()
    ).hexdigest()
    commissioning.write_text(yaml.safe_dump(document), encoding="utf-8")

    with pytest.raises(CommissioningError, match="sample_count must equal"):
        load_commissioning(commissioning, calibration, rectification, profile)


def test_commissioning_preview_must_be_decodable_3x2_mosaic(tmp_path: Path) -> None:
    commissioning, rectification, calibration, profile = write_commissioning(tmp_path)
    document = yaml.safe_load(commissioning.read_text(encoding="utf-8"))
    preview_path = commissioning.parent / document["projection_preview_file"]
    assert cv2.imwrite(str(preview_path), np.zeros((1, 1, 3), dtype=np.uint8))
    document["projection_preview_sha256"] = hashlib.sha256(
        preview_path.read_bytes()
    ).hexdigest()
    commissioning.write_text(yaml.safe_dump(document), encoding="utf-8")

    with pytest.raises(CommissioningError, match="1920x720"):
        load_commissioning(commissioning, calibration, rectification, profile)
