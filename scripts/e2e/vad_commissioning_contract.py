#!/usr/bin/env python3
"""Commissioning approval contract for real-camera rectification."""

from __future__ import annotations

from datetime import datetime, timedelta, timezone
import hashlib
import math
from pathlib import Path
import re
import struct
from typing import Any, Mapping

import yaml

try:
    from vad_training_data_contract import CAMERA_ORDER
except ModuleNotFoundError:
    from scripts.e2e.vad_training_data_contract import CAMERA_ORDER


EVIDENCE_SPECS = {
    "checkerboard_evidence": (
        "checkerboard_evidence_file",
        "checkerboard_dataset_sha256",
    ),
    "reprojection_report": (
        "reprojection_report_file",
        "reprojection_report_sha256",
    ),
    "projection_preview": (
        "projection_preview_file",
        "projection_preview_sha256",
    ),
    "device_inventory": (
        "device_inventory_file",
        "device_inventory_sha256",
    ),
    "device_status_adapter": (
        "device_status_adapter_file",
        "device_status_adapter_sha256",
    ),
    "device_status_adapter_config": (
        "device_status_adapter_config_file",
        "device_status_adapter_config_sha256",
    ),
}


class CommissioningError(ValueError):
    pass


def sha256_file(path: Path, chunk_size: int = 8 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(chunk_size):
            digest.update(chunk)
    return digest.hexdigest()


def _sha(value: Any, context: str) -> str:
    if not isinstance(value, str) or not re.fullmatch(r"[0-9a-f]{64}", value):
        raise CommissioningError(f"{context} must be a lowercase SHA256")
    return value


def _metric(value: Any, context: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CommissioningError(f"{context} must be a number")
    result = float(value)
    if not math.isfinite(result) or result < 0:
        raise CommissioningError(f"{context} must be finite and non-negative")
    return result


def _positive_integer(value: Any, context: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise CommissioningError(f"{context} must be a positive integer")
    return value


def _utc_timestamp(value: Any, context: str) -> datetime:
    if not isinstance(value, str) or value.startswith("REPLACE_WITH_"):
        raise CommissioningError(f"{context} must be an ISO8601 UTC timestamp")
    try:
        timestamp = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as error:
        raise CommissioningError(f"{context} must be an ISO8601 UTC timestamp") from error
    if timestamp.tzinfo is None or timestamp.utcoffset() != timedelta(0):
        raise CommissioningError(f"{context} must carry an explicit UTC offset")
    return timestamp


def _mapping_file(path: Path, context: str) -> dict[str, Any]:
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError) as error:
        raise CommissioningError(f"cannot read {context} {path}: {error}") from error
    except yaml.YAMLError as error:
        raise CommissioningError(f"invalid YAML in {context} {path}: {error}") from error
    if not isinstance(document, Mapping):
        raise CommissioningError(f"{context} must be a YAML mapping")
    return dict(document)


def _evidence_files(
    commissioning: Mapping[str, Any], commissioning_path: Path
) -> dict[str, dict[str, str]]:
    evidence: dict[str, dict[str, str]] = {}
    used_names: set[str] = set()
    for name, (file_field, hash_field) in EVIDENCE_SPECS.items():
        filename = commissioning.get(file_field)
        if (
            not isinstance(filename, str)
            or not filename
            or filename.startswith("REPLACE_WITH_")
            or Path(filename).name != filename
        ):
            raise CommissioningError(
                f"{file_field} must be a simple filename beside the commissioning file"
            )
        if filename in used_names:
            raise CommissioningError("commissioning evidence filenames must be unique")
        if filename == commissioning_path.name:
            raise CommissioningError("commissioning evidence cannot replace its manifest")
        used_names.add(filename)
        path = (commissioning_path.parent / filename).resolve()
        if not path.is_file() or path.stat().st_size <= 0:
            raise CommissioningError(f"commissioning evidence not found or empty: {path}")
        expected_hash = _sha(commissioning.get(hash_field), hash_field)
        actual_hash = sha256_file(path)
        if actual_hash != expected_hash:
            raise CommissioningError(f"{hash_field} does not match {path}")
        evidence[name] = {
            "path": str(path),
            "filename": filename,
            "sha256": actual_hash,
        }
    return evidence


def _validate_preview(path: Path) -> dict[str, int]:
    import cv2

    header = path.read_bytes()[:24]
    if (
        len(header) != 24
        or header[:8] != b"\x89PNG\r\n\x1a\n"
        or header[12:16] != b"IHDR"
    ):
        raise CommissioningError("projection_preview_file must be a valid PNG header")
    width, height = struct.unpack(">II", header[16:24])
    if width <= 0 or height <= 0:
        raise CommissioningError("projection preview dimensions must be positive")
    decoded = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if decoded is None:
        raise CommissioningError("projection_preview_file cannot be decoded")
    decoded_height, decoded_width = decoded.shape[:2]
    if (decoded_width, decoded_height) != (1920, 720) or (width, height) != (1920, 720):
        raise CommissioningError("projection preview must be the 1920x720 VAD 3x2 mosaic")
    return {"width": width, "height": height}


def _validate_reprojection_report(
    path: Path, commissioning: Mapping[str, Any]
) -> dict[str, Any]:
    report = _mapping_file(path, "reprojection report")
    if report.get("schema_version") != 1:
        raise CommissioningError("reprojection report schema_version must be 1")
    if tuple(report.get("camera_order", ())) != CAMERA_ORDER:
        raise CommissioningError("reprojection report camera_order must match VAD")
    _positive_integer(report.get("sample_count"), "reprojection report sample_count")
    allowed = _metric(
        commissioning.get("maximum_allowed_reprojection_error_px"),
        "maximum_allowed_reprojection_error_px",
    )
    report_mean = _metric(
        report.get("mean_reprojection_error_px"),
        "reprojection report mean_reprojection_error_px",
    )
    report_maximum = _metric(
        report.get("maximum_reprojection_error_px"),
        "reprojection report maximum_reprojection_error_px",
    )
    commissioning_mean = _metric(
        commissioning.get("mean_reprojection_error_px"),
        "mean_reprojection_error_px",
    )
    commissioning_maximum = _metric(
        commissioning.get("maximum_reprojection_error_px"),
        "maximum_reprojection_error_px",
    )
    if not math.isclose(report_mean, commissioning_mean, abs_tol=1e-12):
        raise CommissioningError("commissioning mean error does not match report")
    if not math.isclose(report_maximum, commissioning_maximum, abs_tol=1e-12):
        raise CommissioningError("commissioning maximum error does not match report")
    if allowed > 1.0 or report_mean > report_maximum or report_maximum > allowed:
        raise CommissioningError("reprojection error exceeds the approved one-pixel limit")
    cameras = report.get("cameras")
    if not isinstance(cameras, list) or len(cameras) != len(CAMERA_ORDER):
        raise CommissioningError("reprojection report must contain all six cameras")
    camera_counts: list[int] = []
    camera_means: list[float] = []
    camera_maxima: list[float] = []
    for index, camera in enumerate(cameras):
        if not isinstance(camera, Mapping) or camera.get("name") != CAMERA_ORDER[index]:
            raise CommissioningError("reprojection report camera order is invalid")
        camera_count = _positive_integer(
            camera.get("image_count"), f"{CAMERA_ORDER[index]}.image_count"
        )
        camera_mean = _metric(
            camera.get("mean_reprojection_error_px"),
            f"{CAMERA_ORDER[index]}.mean_reprojection_error_px",
        )
        camera_maximum = _metric(
            camera.get("maximum_reprojection_error_px"),
            f"{CAMERA_ORDER[index]}.maximum_reprojection_error_px",
        )
        if camera_mean > camera_maximum or camera_maximum > allowed:
            raise CommissioningError(
                f"{CAMERA_ORDER[index]} reprojection error exceeds the approved limit"
            )
        camera_counts.append(camera_count)
        camera_means.append(camera_mean)
        camera_maxima.append(camera_maximum)
    if report["sample_count"] != sum(camera_counts):
        raise CommissioningError("reprojection sample_count must equal camera image counts")
    weighted_mean = sum(
        count * mean for count, mean in zip(camera_counts, camera_means)
    ) / sum(camera_counts)
    if not math.isclose(report_mean, weighted_mean, abs_tol=1e-9):
        raise CommissioningError("reprojection global mean must equal the weighted camera mean")
    if not math.isclose(report_maximum, max(camera_maxima), abs_tol=1e-12):
        raise CommissioningError("reprojection global maximum must equal the camera maximum")
    return report


def _validate_device_inventory(
    path: Path,
    calibration: Mapping[str, Any],
    profile: Mapping[str, Any],
) -> dict[str, Any]:
    inventory = _mapping_file(path, "device inventory")
    if inventory.get("schema_version") != 1:
        raise CommissioningError("device inventory schema_version must be 1")
    _utc_timestamp(inventory.get("captured_utc"), "device inventory captured_utc")
    sensor_rig_id = profile["session"].get("sensor_rig_id")
    if inventory.get("sensor_rig_id") != sensor_rig_id:
        raise CommissioningError("device inventory sensor_rig_id does not match profile")
    if inventory.get("ptp_clock_source") != profile["provenance"].get("ptp_clock_source"):
        raise CommissioningError("device inventory PTP clock source does not match profile")
    if tuple(inventory.get("camera_order", ())) != CAMERA_ORDER:
        raise CommissioningError("device inventory camera_order must match VAD")
    cameras = inventory.get("cameras")
    calibration_cameras = calibration.get("cameras")
    if not isinstance(cameras, list) or len(cameras) != len(CAMERA_ORDER):
        raise CommissioningError("device inventory must contain all six cameras")
    if not isinstance(calibration_cameras, list):
        raise CommissioningError("calibration cameras are missing")
    sdk_device_ids: set[str] = set()
    device_nodes: set[str] = set()
    for index, camera in enumerate(cameras):
        expected = calibration_cameras[index]
        if not isinstance(camera, Mapping) or camera.get("name") != CAMERA_ORDER[index]:
            raise CommissioningError("device inventory camera order is invalid")
        if camera.get("serial") != expected.get("serial"):
            raise CommissioningError(f"{CAMERA_ORDER[index]} inventory serial does not match")
        if camera.get("firmware") != profile["provenance"].get("camera_firmware"):
            raise CommissioningError(f"{CAMERA_ORDER[index]} inventory firmware does not match")
        for key in ("sdk_device_id", "device_node"):
            value = camera.get(key)
            if not isinstance(value, str) or not value or value.startswith("REPLACE_WITH_"):
                raise CommissioningError(f"{CAMERA_ORDER[index]}.{key} must identify hardware")
        sdk_device_ids.add(str(camera["sdk_device_id"]))
        device_nodes.add(str(camera["device_node"]))
    if len(sdk_device_ids) != len(CAMERA_ORDER):
        raise CommissioningError("device inventory SDK device IDs must be unique")
    if len(device_nodes) != len(CAMERA_ORDER):
        raise CommissioningError("device inventory device nodes must be unique")
    return inventory


def resolve_profile_file(profile: Mapping[str, Any], key: str) -> Path:
    value = profile.get("provenance", {}).get(key)
    if not isinstance(value, str) or not value or value.startswith("REPLACE_WITH_"):
        raise CommissioningError(f"provenance.{key} must point to a completed file")
    path = Path(value).expanduser()
    if not path.is_absolute():
        profile_path = profile.get("_profile_path")
        if not profile_path:
            raise CommissioningError(f"relative provenance.{key} needs a loaded profile path")
        path = Path(profile_path).parent / path
    path = path.resolve()
    if not path.is_file():
        raise CommissioningError(f"provenance.{key} not found: {path}")
    return path


def load_commissioning(
    path: Path | str,
    calibration: Mapping[str, Any],
    rectification_config: Path,
    profile: Mapping[str, Any],
) -> dict[str, Any]:
    commissioning_path = Path(path).expanduser().resolve()
    result = _mapping_file(commissioning_path, "commissioning file")
    if result.get("schema_version") != 1:
        raise CommissioningError("commissioning schema_version must be 1")
    if result.get("approved") is not True or result.get("projection_preview_approved") is not True:
        raise CommissioningError("commissioning and projection preview must both be approved")
    approved_by = result.get("approved_by")
    if not isinstance(approved_by, str) or not approved_by or approved_by.startswith("REPLACE_WITH_"):
        raise CommissioningError("approved_by must identify a commissioning review")
    approved_time = _utc_timestamp(result.get("approved_utc"), "approved_utc")
    valid_until = _utc_timestamp(result.get("valid_until_utc"), "valid_until_utc")
    if valid_until <= approved_time:
        raise CommissioningError("valid_until_utc must be later than approved_utc")
    if valid_until - approved_time > timedelta(days=366):
        raise CommissioningError("commissioning validity cannot exceed 366 days")
    if approved_time > datetime.now(timezone.utc) + timedelta(minutes=5):
        raise CommissioningError("approved_utc cannot be in the future")

    calibration_sha256 = str(calibration.get("_sha256", ""))
    if _sha(result.get("calibration_sha256"), "calibration_sha256") != calibration_sha256:
        raise CommissioningError("commissioning calibration SHA256 does not match")
    rectification_sha = sha256_file(rectification_config)
    if (
        _sha(result.get("rectification_config_sha256"), "rectification_config_sha256")
        != rectification_sha
    ):
        raise CommissioningError("commissioning rectification config SHA256 does not match")
    for key in ("sensor_rig_id", "camera_firmware", "exposure_mode"):
        expected = (
            profile["session"].get(key)
            if key == "sensor_rig_id"
            else profile["provenance"].get(key)
        )
        if result.get(key) != expected:
            raise CommissioningError(f"commissioning {key} does not match profile")

    evidence = _evidence_files(result, commissioning_path)
    report = _validate_reprojection_report(
        Path(evidence["reprojection_report"]["path"]), result
    )
    preview = _validate_preview(Path(evidence["projection_preview"]["path"]))
    inventory = _validate_device_inventory(
        Path(evidence["device_inventory"]["path"]), calibration, profile
    )
    result["_path"] = str(commissioning_path)
    result["_sha256"] = sha256_file(commissioning_path)
    result["_rectification_config_path"] = str(rectification_config)
    result["_rectification_config_sha256"] = rectification_sha
    result["_evidence_files"] = evidence
    result["_reprojection_report"] = report
    result["_projection_preview"] = preview
    result["_device_inventory"] = inventory
    result["_expired"] = datetime.now(timezone.utc) > valid_until
    return result


def load_profile_commissioning(
    profile: Mapping[str, Any], calibration: Mapping[str, Any]
) -> dict[str, Any]:
    return load_commissioning(
        resolve_profile_file(profile, "commissioning_file"),
        calibration,
        resolve_profile_file(profile, "rectification_config_file"),
        profile,
    )
