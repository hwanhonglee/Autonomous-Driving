import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from nav_msgs.msg import Odometry
import pytest


ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "autoware_e2e_vad_launch"
ADAPTER = PACKAGE / "scripts/carla_map_aligned_odometry.py"
FULL_LAUNCH = PACKAGE / "launch/carla_vad_full.launch.xml"


def load_adapter():
    spec = importlib.util.spec_from_file_location("carla_map_aligned_odometry", ADAPTER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def make_odometry(x=3.0, y=4.0, z=1.5, yaw_rad=0.0):
    message = Odometry()
    message.header.stamp.sec = 12
    message.header.stamp.nanosec = 345
    message.header.frame_id = "carla_map"
    message.child_frame_id = "carla_base_link"
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.position.z = z
    message.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    message.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
    message.twist.twist.linear.x = 2.5
    message.twist.twist.angular.z = -0.25
    message.twist.covariance = [float(value) for value in range(36)]
    return message


def test_transform_applies_left_composed_planar_pose_and_preserves_child_twist():
    adapter = load_adapter()
    message = make_odometry(yaw_rad=math.radians(30.0))

    output = adapter.transform_odometry(
        message,
        translation_x_m=10.0,
        translation_y_m=-2.0,
        translation_z_m=-0.75,
        yaw_rad=math.pi / 2.0,
    )

    assert output.header.stamp == message.header.stamp
    assert output.header.frame_id == "map"
    assert output.child_frame_id == "base_link"
    assert output.pose.pose.position.x == pytest.approx(6.0)
    assert output.pose.pose.position.y == pytest.approx(1.0)
    assert output.pose.pose.position.z == pytest.approx(0.75)
    assert yaw_from_quaternion(output.pose.pose.orientation) == pytest.approx(
        math.radians(120.0)
    )
    assert output.twist == message.twist
    assert output.twist is not message.twist


def test_pose_covariance_is_rotated_in_position_and_orientation_blocks():
    adapter = load_adapter()
    covariance = [0.0] * 36
    covariance[0] = 4.0
    covariance[7] = 1.0
    covariance[14] = 9.0
    covariance[21] = 16.0
    covariance[28] = 25.0
    covariance[35] = 36.0
    covariance[1] = covariance[6] = 0.5
    covariance[22] = covariance[27] = 1.5

    rotated = adapter.rotate_pose_covariance(covariance, math.pi / 2.0)

    assert rotated[0] == pytest.approx(1.0)
    assert rotated[7] == pytest.approx(4.0)
    assert rotated[1] == pytest.approx(-0.5)
    assert rotated[6] == pytest.approx(-0.5)
    assert rotated[14] == pytest.approx(9.0)
    assert rotated[21] == pytest.approx(25.0)
    assert rotated[28] == pytest.approx(16.0)
    assert rotated[22] == pytest.approx(-1.5)
    assert rotated[27] == pytest.approx(-1.5)
    assert rotated[35] == pytest.approx(36.0)


def test_invalid_alignment_and_orientation_are_rejected():
    adapter = load_adapter()
    message = make_odometry()

    with pytest.raises(ValueError, match="finite"):
        adapter.transform_odometry(message, float("nan"), 0.0, 0.0, 0.0)

    message.pose.pose.orientation.w = 0.0
    with pytest.raises(ValueError, match="zero-norm"):
        adapter.transform_odometry(message, 0.0, 0.0, 0.0, 0.0)


def test_transform_message_matches_aligned_odometry_frames_and_pose():
    adapter = load_adapter()
    output = adapter.transform_odometry(make_odometry(), 1.0, 2.0, 3.0, 0.25)

    transform = adapter.odometry_to_transform(output)

    assert transform.header == output.header
    assert transform.child_frame_id == output.child_frame_id
    assert transform.transform.translation.x == output.pose.pose.position.x
    assert transform.transform.translation.y == output.pose.pose.position.y
    assert transform.transform.rotation == output.pose.pose.orientation


def test_full_launch_switches_tf_and_odometry_ownership_atomically():
    root = ET.parse(FULL_LAUNCH).getroot()
    args = {element.get("name"): element for element in root.findall("./arg")}

    assert args["use_carla_map_alignment"].get("default") == "false"
    assert args["carla_map_alignment_x_m"].get("default") == "0.0"
    assert args["carla_map_alignment_y_m"].get("default") == "0.0"
    assert args["carla_map_alignment_z_m"].get("default") == "0.0"
    assert args["carla_map_alignment_yaw_rad"].get("default") == "0.0"

    state_publisher = next(
        node for node in root.iter("node") if node.get("name") == "carla_state_publisher"
    )
    state_params = {
        element.get("name"): element.get("value")
        for element in state_publisher.findall("./param")
    }
    state_remaps = {
        element.get("from"): element.get("to")
        for element in state_publisher.findall("./remap")
    }
    assert state_params["frame_id"] == "$(var carla_truth_frame_id)"
    assert state_params["publish_tf"] == "$(var carla_truth_publish_tf)"
    assert state_remaps["~/output/odometry"] == "$(var carla_truth_odometry_topic)"

    lets = list(root.findall("./let"))
    odometry_lets = [
        element for element in lets if element.get("name") == "carla_truth_odometry_topic"
    ]
    tf_lets = [
        element for element in lets if element.get("name") == "carla_truth_publish_tf"
    ]
    frame_lets = [
        element for element in lets if element.get("name") == "carla_truth_frame_id"
    ]
    assert {
        (element.get("value"), element.get("if"), element.get("unless"))
        for element in odometry_lets
    } == {
        ("/localization/kinematic_state_unaligned", "$(var use_carla_map_alignment)", None),
        ("/localization/kinematic_state", None, "$(var use_carla_map_alignment)"),
    }
    assert {
        (element.get("value"), element.get("if"), element.get("unless"))
        for element in tf_lets
    } == {
        ("false", "$(var use_carla_map_alignment)", None),
        ("true", None, "$(var use_carla_map_alignment)"),
    }
    assert {
        (element.get("value"), element.get("if"), element.get("unless"))
        for element in frame_lets
    } == {
        ("carla_map", "$(var use_carla_map_alignment)", None),
        ("map", None, "$(var use_carla_map_alignment)"),
    }

    aligner = next(
        node for node in root.iter("node") if node.get("name") == "carla_map_aligned_odometry"
    )
    assert aligner.get("if") == "$(var use_carla_map_alignment)"
    aligner_params = {
        element.get("name"): element.get("value") for element in aligner.findall("./param")
    }
    aligner_remaps = {
        element.get("from"): element.get("to") for element in aligner.findall("./remap")
    }
    assert aligner_params["translation_x_m"] == "$(var carla_map_alignment_x_m)"
    assert aligner_params["translation_y_m"] == "$(var carla_map_alignment_y_m)"
    assert aligner_params["translation_z_m"] == "$(var carla_map_alignment_z_m)"
    assert aligner_params["yaw_rad"] == "$(var carla_map_alignment_yaw_rad)"
    assert aligner_params["publish_tf"] == "true"
    assert aligner_remaps == {
        "~/input/odometry": "/localization/kinematic_state_unaligned",
        "~/output/odometry": "/localization/kinematic_state",
    }
