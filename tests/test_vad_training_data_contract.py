from copy import deepcopy
import hashlib
import json
from pathlib import Path

import pytest
import yaml

from scripts.e2e.vad_calibration_contract import load_calibration
from scripts.e2e.vad_commissioning_contract import load_profile_commissioning
from scripts.e2e.vad_training_data_contract import CAMERA_ORDER
from scripts.e2e.vad_training_data_contract import ProfileError
from scripts.e2e.vad_training_data_contract import evaluate_live_topics
from scripts.e2e.vad_training_data_contract import find_placeholders
from scripts.e2e.vad_training_data_contract import load_profile
from scripts.e2e.vad_training_data_contract import parse_ros2_topic_list
from scripts.e2e.vad_training_data_contract import unique_topic_specs
from scripts.e2e.vad_training_data_contract import validate_profile
from scripts.e2e.vad_training_data_contract import write_session_manifest
from vad_training_test_utils import calibration_document
from vad_training_test_utils import complete_profile


ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / "autoware_e2e_vad_launch/config/vad_real_data_collection.yaml"


def loaded_profile():
    profile = load_profile(PROFILE)
    profile["session"]["vehicle_id"] = "test_vehicle"
    profile["session"]["sensor_rig_id"] = "test_rig"
    for key in profile["provenance"]:
        profile["provenance"][key] = f"test_{key}"
    return profile


def test_default_profile_keeps_camera_training_order_and_explicit_placeholders() -> None:
    profile = load_profile(PROFILE)

    assert tuple(camera["name"] for camera in profile["cameras"]) == CAMERA_ORDER
    assert profile["model_contract"]["input_width"] == 640
    assert profile["model_contract"]["input_height"] == 360
    assert "session.vehicle_id" in find_placeholders(profile)
    assert "provenance.calibration_file" in find_placeholders(profile)


def test_profile_rejects_a_camera_order_swap() -> None:
    profile = loaded_profile()
    broken = deepcopy(profile)
    broken["cameras"][0], broken["cameras"][1] = broken["cameras"][1], broken["cameras"][0]

    with pytest.raises(ProfileError, match=r"cameras\[0\]"):
        validate_profile(broken)


def test_profile_rejects_non_runtime_image_shape() -> None:
    profile = loaded_profile()
    profile["model_contract"]["input_width"] = 64

    with pytest.raises(ProfileError, match="input_width"):
        validate_profile(profile)


def test_topic_specs_include_all_required_runtime_inputs() -> None:
    profile = loaded_profile()
    specs = unique_topic_specs(profile)
    required = {spec.topic: spec.message_type for spec in specs if spec.required_for_capture}

    assert len(required) == 22
    assert required["/sensing/camera/CAM_FRONT/image_rect"] == "sensor_msgs/msg/Image"
    assert required["/sensing/camera/CAM_FRONT/device_info"] == (
        "diagnostic_msgs/msg/DiagnosticStatus"
    )
    assert required["/sensing/time_sync/status"] == "diagnostic_msgs/msg/DiagnosticStatus"
    assert required["/localization/kinematic_state"] == "nav_msgs/msg/Odometry"
    assert required["/tf_static"] == "tf2_msgs/msg/TFMessage"


def test_live_preflight_separates_required_and_optional_topics() -> None:
    profile = loaded_profile()
    specs = unique_topic_specs(profile)
    discovered = {
        spec.topic: [spec.message_type] for spec in specs if spec.required_for_capture
    }
    report = evaluate_live_topics(profile, discovered)

    assert report["pass"] is True
    assert report["required_missing"] == []
    assert "/planning/vad_route/command" in report["optional_missing"]

    discovered["/localization/kinematic_state"] = ["std_msgs/msg/String"]
    report = evaluate_live_topics(profile, discovered)
    assert report["pass"] is False
    assert report["required_type_mismatch"][0]["topic"] == "/localization/kinematic_state"


def test_parse_ros2_topic_list_supports_multiple_types() -> None:
    parsed = parse_ros2_topic_list(
        "/camera [sensor_msgs/msg/Image]\n/mixed [std_msgs/msg/String, std_msgs/msg/Bool]\n"
    )

    assert parsed["/camera"] == ["sensor_msgs/msg/Image"]
    assert parsed["/mixed"] == ["std_msgs/msg/String", "std_msgs/msg/Bool"]


def test_manifest_preserves_bag_hash_across_validation_status(tmp_path: Path) -> None:
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
    commissioning = load_profile_commissioning(profile, calibration)

    output = tmp_path / "capture"
    bag = output / "bag"
    bag.mkdir(parents=True)
    bag_file = bag / "data_0.db3"
    bag_file.write_bytes(b"finalized bag")
    (bag / "metadata.yaml").write_text(
        yaml.safe_dump(
            {
                "rosbag2_bagfile_information": {
                    "relative_file_paths": [bag_file.name]
                }
            }
        ),
        encoding="utf-8",
    )

    write_session_manifest(
        profile,
        output,
        "recorded_unvalidated",
        bag,
        calibration,
        commissioning,
        hash_bag_files=True,
    )
    first = json.loads((output / "session_manifest.json").read_text(encoding="utf-8"))
    (output / "training_data_report.json").write_text("{}\n", encoding="utf-8")
    write_session_manifest(
        profile,
        output,
        "validation_failed",
        bag,
        calibration,
        commissioning,
        hash_bag_files=False,
    )
    final = json.loads((output / "session_manifest.json").read_text(encoding="utf-8"))

    expected_bag_sha = hashlib.sha256(bag_file.read_bytes()).hexdigest()
    assert first["bag_files"][0]["sha256"] == expected_bag_sha
    assert final["status"] == "validation_failed"
    assert final["bag_files"][0]["sha256"] == expected_bag_sha
    assert final["created_unix_ns"] == first["created_unix_ns"]
    assert final["session_artifact_sha256"]["training_data_report.json"] == hashlib.sha256(
        b"{}\n"
    ).hexdigest()
