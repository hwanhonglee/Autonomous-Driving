from datetime import datetime, timedelta, timezone
import hashlib
from pathlib import Path

import cv2
import numpy as np
import yaml


def complete_profile(profile: dict, calibration_path: Path) -> dict:
    approved_time = datetime.now(timezone.utc).replace(microsecond=0)
    valid_until = approved_time + timedelta(days=365)
    approved_utc = approved_time.isoformat().replace("+00:00", "Z")
    valid_until_utc = valid_until.isoformat().replace("+00:00", "Z")
    profile["session"]["vehicle_id"] = "test_vehicle"
    profile["session"]["sensor_rig_id"] = "test_rig"
    for key in profile["provenance"]:
        profile["provenance"][key] = f"test_{key}"

    rectification_path = calibration_path.with_name("rectification.yaml")
    rectification_path.write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "input_width": 640,
                "input_height": 360,
                "camera_order": [camera["name"] for camera in profile["cameras"]],
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    profile["provenance"]["calibration_file"] = str(calibration_path)
    profile["provenance"]["rectification_config_file"] = str(rectification_path)
    commissioning_path = calibration_path.with_name("commissioning.yaml")
    profile["provenance"]["commissioning_file"] = str(commissioning_path)

    checkerboard_path = calibration_path.with_name("checkerboard_evidence.tar.zst")
    checkerboard_path.write_bytes(b"synthetic checkerboard archive")
    reprojection_path = calibration_path.with_name("reprojection_report.yaml")
    reprojection_path.write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "camera_order": [camera["name"] for camera in profile["cameras"]],
                "sample_count": 60,
                "mean_reprojection_error_px": 0.1,
                "maximum_reprojection_error_px": 0.2,
                "cameras": [
                    {
                        "name": camera["name"],
                        "image_count": 10,
                        "mean_reprojection_error_px": 0.1,
                        "maximum_reprojection_error_px": 0.2,
                    }
                    for camera in profile["cameras"]
                ],
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    preview_path = calibration_path.with_name("commissioning_projection.png")
    preview = np.zeros((720, 1920, 3), dtype=np.uint8)
    cv2.line(preview, (0, 360), (1919, 360), (0, 255, 0), 2)
    if not cv2.imwrite(str(preview_path), preview):
        raise RuntimeError("failed to create synthetic commissioning preview")
    inventory_path = calibration_path.with_name("device_inventory.yaml")
    inventory_path.write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "captured_utc": approved_utc,
                "sensor_rig_id": profile["session"]["sensor_rig_id"],
                "ptp_clock_source": profile["provenance"]["ptp_clock_source"],
                "camera_order": [camera["name"] for camera in profile["cameras"]],
                "cameras": [
                    {
                        "name": camera["name"],
                        "serial": f"synthetic_serial_{index}",
                        "firmware": profile["provenance"]["camera_firmware"],
                        "sdk_device_id": f"sdk_camera_{index}",
                        "device_node": f"/dev/video{index}",
                    }
                    for index, camera in enumerate(profile["cameras"])
                ],
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    adapter_path = calibration_path.with_name("device_status_adapter.py")
    adapter_path.write_text("# synthetic status adapter\n", encoding="utf-8")
    adapter_config_path = calibration_path.with_name("device_status_adapter.yaml")
    adapter_config_path.write_text("schema_version: 1\n", encoding="utf-8")

    calibration_bytes = yaml.safe_dump(
        calibration_document(profile), sort_keys=False
    ).encode("utf-8")
    commissioning_path.write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "approved": True,
                "approved_by": "synthetic_test_review",
                "approved_utc": approved_utc,
                "valid_until_utc": valid_until_utc,
                "sensor_rig_id": profile["session"]["sensor_rig_id"],
                "camera_firmware": profile["provenance"]["camera_firmware"],
                "exposure_mode": profile["provenance"]["exposure_mode"],
                "calibration_sha256": hashlib.sha256(calibration_bytes).hexdigest(),
                "rectification_config_sha256": hashlib.sha256(
                    rectification_path.read_bytes()
                ).hexdigest(),
                "checkerboard_dataset_sha256": hashlib.sha256(
                    checkerboard_path.read_bytes()
                ).hexdigest(),
                "checkerboard_evidence_file": checkerboard_path.name,
                "reprojection_report_file": reprojection_path.name,
                "reprojection_report_sha256": hashlib.sha256(
                    reprojection_path.read_bytes()
                ).hexdigest(),
                "projection_preview_file": preview_path.name,
                "projection_preview_sha256": hashlib.sha256(
                    preview_path.read_bytes()
                ).hexdigest(),
                "device_inventory_file": inventory_path.name,
                "device_inventory_sha256": hashlib.sha256(
                    inventory_path.read_bytes()
                ).hexdigest(),
                "device_status_adapter_file": adapter_path.name,
                "device_status_adapter_sha256": hashlib.sha256(
                    adapter_path.read_bytes()
                ).hexdigest(),
                "device_status_adapter_config_file": adapter_config_path.name,
                "device_status_adapter_config_sha256": hashlib.sha256(
                    adapter_config_path.read_bytes()
                ).hexdigest(),
                "mean_reprojection_error_px": 0.1,
                "maximum_reprojection_error_px": 0.2,
                "maximum_allowed_reprojection_error_px": 1.0,
                "projection_preview_approved": True,
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    return profile


def calibration_document(profile: dict) -> dict:
    cameras = []
    for index, camera in enumerate(profile["cameras"]):
        cameras.append(
            {
                "name": camera["name"],
                "model_index": index,
                "serial": f"synthetic_serial_{index}",
                "optical_frame": camera["optical_frame"],
                "timestamp_source": "hardware_exposure",
                "trigger_mode": "hardware",
                "distortion_model": "plumb_bob",
                "d": [0.0] * 5,
                "k": [300.0, 0.0, 320.0, 0.0, 300.0, 180.0, 0.0, 0.0, 1.0],
                "camera_pose_in_base": {
                    "translation_xyz": [1.0 + index, 0.0, 1.2],
                    "rotation_xyzw": [-0.5, 0.5, -0.5, 0.5],
                },
            }
        )
    return {
        "schema_version": 1,
        "base_frame": "base_link",
        "image_width": 640,
        "image_height": 360,
        "cameras": cameras,
    }
