from copy import deepcopy
from pathlib import Path

import yaml

from scripts.e2e.vad_training_data_contract import load_profile
from scripts.e2e.vad_calibration_contract import load_calibration
from scripts.e2e.vad_commissioning_contract import load_profile_commissioning
from scripts.e2e.render_vad_calibration_preview import render_preview
from scripts.e2e.validate_vad_training_bag import alignment_summary
from scripts.e2e.validate_vad_training_bag import anchored_bundle_coverage
from scripts.e2e.validate_vad_training_bag import build_report
from scripts.e2e.validate_vad_training_bag import count_imbalance_percent
from scripts.e2e.validate_vad_training_bag import command_alignment_summary
from scripts.e2e.validate_vad_training_bag import exposure_binding_summary
from scripts.e2e.validate_vad_training_bag import future_window_count
from scripts.e2e.validate_vad_training_bag import graph_has_path
from scripts.e2e.validate_vad_training_bag import sequence_timing
from vad_training_test_utils import calibration_document
from vad_training_test_utils import complete_profile


CAMERAS = (
    "CAM_FRONT",
    "CAM_BACK",
    "CAM_FRONT_LEFT",
    "CAM_BACK_LEFT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK_RIGHT",
)

ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / "autoware_e2e_vad_launch/config/vad_real_data_collection.yaml"


def test_bundle_coverage_distinguishes_runtime_and_dataset_tolerances() -> None:
    stamps = {
        camera: [0, 100_000_000, 200_000_000] for camera in CAMERAS
    }
    stamps["CAM_BACK_RIGHT"] = [5_000_000, 105_000_000, 205_000_000]

    strict = anchored_bundle_coverage(stamps, tolerance_ms=1.0)
    dataset = anchored_bundle_coverage(stamps, tolerance_ms=20.0)

    assert strict["coverage_percent"] == 0.0
    assert dataset["coverage_percent"] == 100.0
    assert dataset["maximum_skew_ms"] == 5.0


def test_sequence_timing_reports_duplicates_rate_and_gap() -> None:
    result = sequence_timing([1_000_000_000, 1_100_000_000, 1_100_000_000, 1_200_000_000])

    assert result["duplicate_or_nonmonotonic_count"] == 1
    assert result["unique_count"] == 3
    assert result["rate_hz"] == 10.0
    assert result["maximum_gap_ms"] == 100.0


def test_state_alignment_uses_nearest_stamp() -> None:
    report = alignment_summary(
        [100_000_000, 200_000_000],
        [90_000_000, 212_000_000],
        tolerance_ms=15.0,
    )

    assert report["coverage_percent"] == 100.0
    assert report["maximum_delta_ms"] == 12.0


def test_future_windows_require_every_half_second_target() -> None:
    odometry = [index * 100_000_000 for index in range(41)]

    assert future_window_count([0, 1_000_000_000], odometry, horizon_s=3.0, step_s=0.5) == 2
    assert future_window_count([2_000_000_000], odometry, horizon_s=3.0, step_s=0.5) == 0


def test_tf_graph_accepts_a_static_transform_chain() -> None:
    edges = {("base_link", "sensor_kit_base_link"), ("sensor_kit_base_link", "camera")}

    assert graph_has_path(edges, "base_link", "camera") is True
    assert graph_has_path(edges, "base_link", "missing") is False


def test_camera_count_imbalance_is_normalized_to_largest_camera() -> None:
    assert count_imbalance_percent([100, 100, 99, 100, 100, 100]) == 1.0
    assert count_imbalance_percent([0, 0, 0]) == 100.0


def test_command_alignment_requires_valid_causal_labels() -> None:
    report = command_alignment_summary(
        [1_000_000_000, 1_100_000_000, 1_200_000_000],
        [(950_000_000, 3), (1_050_000_000, 3), (1_150_000_000, 127)],
        maximum_age_ms=100.0,
    )

    assert report["coverage_percent"] == 100.0 * 2 / 3
    assert report["invalid_values"] == [127]


def test_exposure_binding_requires_ordered_one_to_one_image_stamps() -> None:
    valid = exposure_binding_summary(
        [10, 11],
        [1_000_000_000, 1_100_000_000],
        [1_001_000_000, 1_101_000_000],
        [1_000_000_000, 1_100_000_000],
        [1_000_500_000, 1_100_500_000],
        maximum_status_delta_ms=10.0,
    )
    swapped = exposure_binding_summary(
        [10, 11],
        [1_100_000_000, 1_000_000_000],
        [1_001_000_000, 1_101_000_000],
        [1_000_000_000, 1_100_000_000],
        [1_000_500_000, 1_100_500_000],
        maximum_status_delta_ms=10.0,
    )

    assert valid["pass"] is True
    assert swapped["pass"] is False
    assert swapped["image_header_stamp_order_match"] is False


def test_build_report_accepts_a_small_complete_synthetic_capture(tmp_path: Path) -> None:
    import rosbag2_py
    from diagnostic_msgs.msg import DiagnosticStatus
    from diagnostic_msgs.msg import KeyValue
    from geometry_msgs.msg import AccelWithCovarianceStamped
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import CameraInfo
    from sensor_msgs.msg import Image
    from std_msgs.msg import Int8
    from tf2_msgs.msg import TFMessage

    profile_data = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
    calibration_path = tmp_path / "calibration.yaml"
    complete_profile(profile_data, calibration_path)
    calibration_path.write_text(
        yaml.safe_dump(calibration_document(profile_data), sort_keys=False), encoding="utf-8"
    )
    profile_data["validation"].update(
        {
            "minimum_duration_s": 1.0,
            "minimum_camera_rate_hz": 5.0,
            "minimum_state_rate_hz": 5.0,
            "minimum_motion_distance_m": 0.1,
            "future_horizon_s": 0.5,
            "future_step_s": 0.5,
        }
    )
    profile_path = tmp_path / "profile.yaml"
    profile_path.write_text(yaml.safe_dump(profile_data, sort_keys=False), encoding="utf-8")
    profile = load_profile(profile_path)
    calibration = load_calibration(calibration_path)
    commissioning = load_profile_commissioning(profile, calibration)
    # Keep the production schema fixed at 640x360; shrink only this in-memory fixture.
    profile["model_contract"]["input_width"] = 64
    profile["model_contract"]["input_height"] = 36
    calibration["image_width"] = 64
    calibration["image_height"] = 36
    for camera in calibration["cameras"]:
        camera["k"] = [30.0, 0.0, 32.0, 0.0, 30.0, 18.0, 0.0, 0.0, 1.0]

    bag = tmp_path / "bag"
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topics = []
    for camera in profile["cameras"]:
        topics.extend(
            [
                (camera["image_topic"], "sensor_msgs/msg/Image"),
                (camera["camera_info_topic"], "sensor_msgs/msg/CameraInfo"),
                (camera["device_info_topic"], "diagnostic_msgs/msg/DiagnosticStatus"),
            ]
        )
    topics.extend(
        [
            ("/localization/kinematic_state", "nav_msgs/msg/Odometry"),
            ("/localization/acceleration", "geometry_msgs/msg/AccelWithCovarianceStamped"),
            ("/tf_static", "tf2_msgs/msg/TFMessage"),
            ("/planning/vad_route/command", "std_msgs/msg/Int8"),
            ("/sensing/time_sync/status", "diagnostic_msgs/msg/DiagnosticStatus"),
        ]
    )
    for topic, message_type in topics:
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=topic,
                type=message_type,
                serialization_format="cdr",
                offered_qos_profiles="",
            )
        )

    start_ns = 1_000_000_000
    calibration_by_name = {camera["name"]: camera for camera in calibration["cameras"]}
    static_transforms = []
    for index, camera in enumerate(profile["cameras"]):
        transform = TransformStamped()
        transform.header.stamp.sec = 1
        transform.header.frame_id = "base_link"
        transform.child_frame_id = camera["optical_frame"]
        transform.transform.translation.x = 1.0 + index
        transform.transform.translation.z = 1.2
        transform.transform.rotation.x = -0.5
        transform.transform.rotation.y = 0.5
        transform.transform.rotation.z = -0.5
        transform.transform.rotation.w = 0.5
        static_transforms.append(transform)
    unrelated_invalid = TransformStamped()
    unrelated_invalid.header.stamp.sec = 1
    unrelated_invalid.header.frame_id = "unrelated_parent"
    unrelated_invalid.child_frame_id = "unrelated_child"
    static_transforms.append(unrelated_invalid)
    writer.write("/tf_static", serialize_message(TFMessage(transforms=static_transforms)), start_ns)

    for index in range(20):
        stamp_ns = start_ns + index * 100_000_000
        seconds, nanoseconds = divmod(stamp_ns, 1_000_000_000)
        for camera in profile["cameras"]:
            image = Image()
            image.header.stamp.sec = seconds
            image.header.stamp.nanosec = nanoseconds
            image.header.frame_id = camera["optical_frame"]
            image.width = 64
            image.height = 36
            image.encoding = "bgr8"
            image.step = 192
            image.data = bytes(64 * 36 * 3)
            writer.write(camera["image_topic"], serialize_message(image), stamp_ns)
            if index == 0:
                info = CameraInfo()
                info.header = image.header
                info.width = 64
                info.height = 36
                info.distortion_model = "plumb_bob"
                info.d = [0.0] * 5
                info.k = [30.0, 0.0, 32.0, 0.0, 30.0, 18.0, 0.0, 0.0, 1.0]
                info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                info.p = [30.0, 0.0, 32.0, 0.0, 0.0, 30.0, 18.0, 0.0, 0.0, 0.0, 1.0, 0.0]
                writer.write(camera["camera_info_topic"], serialize_message(info), stamp_ns)

            expected = calibration_by_name[camera["name"]]
            device_info = DiagnosticStatus()
            device_info.level = DiagnosticStatus.OK
            device_info.name = f"vad_training/camera_device_info/{camera['name']}"
            device_info.message = "ready"
            device_info.hardware_id = expected["serial"]
            device_info.values = [
                KeyValue(key="camera_name", value=camera["name"]),
                KeyValue(key="image_topic", value=camera["image_topic"]),
                KeyValue(key="optical_frame", value=camera["optical_frame"]),
                KeyValue(key="timestamp_source", value=expected["timestamp_source"]),
                KeyValue(key="trigger_mode", value=expected["trigger_mode"]),
                KeyValue(key="serial", value=expected["serial"]),
                KeyValue(
                    key="firmware", value=profile["provenance"]["camera_firmware"]
                ),
                KeyValue(
                    key="exposure_mode", value=profile["provenance"]["exposure_mode"]
                ),
                KeyValue(
                    key="phc_clock_source",
                    value=profile["provenance"]["ptp_clock_source"],
                ),
                KeyValue(key="exposure_counter", value=str(index)),
                KeyValue(key="exposure_stamp_ns", value=str(stamp_ns)),
            ]
            writer.write(
                camera["device_info_topic"], serialize_message(device_info), stamp_ns
            )

        odometry = Odometry()
        odometry.header.stamp.sec = seconds
        odometry.header.stamp.nanosec = nanoseconds
        odometry.header.frame_id = "map"
        odometry.child_frame_id = "base_link"
        odometry.pose.pose.position.x = index * 0.1
        odometry.pose.pose.orientation.w = 1.0
        writer.write("/localization/kinematic_state", serialize_message(odometry), stamp_ns)

        acceleration = AccelWithCovarianceStamped()
        acceleration.header = odometry.header
        acceleration.header.frame_id = "base_link"
        writer.write("/localization/acceleration", serialize_message(acceleration), stamp_ns)
        writer.write("/planning/vad_route/command", serialize_message(Int8(data=3)), stamp_ns)

        time_sync = DiagnosticStatus()
        time_sync.level = DiagnosticStatus.OK
        time_sync.name = "vad_training/time_sync"
        time_sync.message = "locked"
        time_sync.hardware_id = profile["provenance"]["ptp_clock_source"]
        time_sync.values = [
            KeyValue(key="locked", value="true"),
            KeyValue(key="offset_ns", value="100"),
            KeyValue(key="timestamp_source", value="hardware_exposure"),
            KeyValue(
                key="clock_source", value=profile["provenance"]["ptp_clock_source"]
            ),
        ]
        writer.write("/sensing/time_sync/status", serialize_message(time_sync), stamp_ns)

    del writer
    report = build_report(bag, profile, calibration, commissioning)
    preview_path = tmp_path / "calibration_preview.png"
    preview = render_preview(bag, profile, calibration, preview_path)

    assert report["gates"]["raw_capture"]["pass"] is True
    assert report["gates"]["planning_finetune_labels_present"]["pass"] is True
    assert report["gates"]["runtime_replay"]["pass"] is False  # Synthetic metadata has no QoS offers.
    assert report["labels"]["future_ego_window_count"] > 0
    assert "/planning/vad_route/command" in report["required_topics"]["optional_present"]
    assert (
        "/training/ground_truth/objects"
        in report["required_topics"]["optional_missing"]
    )
    assert preview["all_cameras_have_projection"] is True
    assert preview_path.is_file()

    wrong_calibration = deepcopy(calibration)
    wrong_calibration["cameras"][0]["k"][0] += 1.0
    wrong_report = build_report(bag, profile, wrong_calibration, commissioning)
    assert wrong_report["gates"]["raw_capture"]["pass"] is False
    assert wrong_report["cameras"]["CAM_FRONT"]["camera_info"]["intrinsic_manifest_match"] is False
