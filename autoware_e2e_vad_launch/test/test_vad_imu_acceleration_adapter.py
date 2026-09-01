from copy import deepcopy
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
from sensor_msgs.msg import Imu
import yaml


ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "autoware_e2e_vad_launch"
ADAPTER = PACKAGE / "scripts/vad_imu_acceleration_adapter.py"
BASE_MAPPING = PACKAGE / "config/sensor_mapping_vad_fast_reliable.yaml"
IMU_MAPPING = PACKAGE / "config/sensor_mapping_vad_fast_reliable_imu.yaml"
CALIBRATION = (
    ROOT
    / "src/launcher/autoware_launch/sensor_kit/carla_sensor_kit_launch"
    / "carla_sensor_kit_description/config/sensor_kit_calibration.yaml"
)
LAUNCHES = (
    PACKAGE / "launch/carla_vad.launch.xml",
    PACKAGE / "launch/carla_vad_full.launch.xml",
)
FAST_WRAPPER = ROOT / "scripts/e2e/run_route_vad_fast.sh"


def load_yaml(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def load_adapter():
    spec = importlib.util.spec_from_file_location("vad_imu_acceleration_adapter", ADAPTER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def direct_args(root):
    return {element.get("name"): element for element in root.findall("./arg")}


def include_args(include):
    return {element.get("name"): element.get("value") for element in include.findall("./arg")}


def test_conversion_preserves_stamp_gravity_and_linear_covariance():
    message = Imu()
    message.header.stamp.sec = 42
    message.header.stamp.nanosec = 123456789
    message.header.frame_id = "tamagawa/imu_link"
    message.linear_acceleration.x = 1.25
    message.linear_acceleration.y = -0.75
    message.linear_acceleration.z = 9.80665
    message.linear_acceleration_covariance = [float(value) for value in range(9)]

    output = load_adapter().imu_to_acceleration(message)

    assert output.header.stamp == message.header.stamp
    assert output.header.frame_id == "base_link"
    assert output.accel.accel.linear.x == pytest.approx(1.25)
    assert output.accel.accel.linear.y == pytest.approx(-0.75)
    assert output.accel.accel.linear.z == pytest.approx(9.80665)
    assert [
        output.accel.covariance[row * 6 + column]
        for row in range(3)
        for column in range(3)
    ] == list(message.linear_acceleration_covariance)
    assert output.accel.accel.angular.x == 0.0
    assert output.accel.accel.angular.y == 0.0
    assert output.accel.accel.angular.z == 0.0


def test_empty_output_frame_preserves_source_frame():
    message = Imu()
    message.header.frame_id = "custom_aligned_imu"

    output = load_adapter().imu_to_acceleration(message, output_frame_id="")

    assert output.header.frame_id == message.header.frame_id


def test_imu_mapping_only_extends_the_stable_reliable_profile():
    baseline = load_yaml(BASE_MAPPING)
    candidate = load_yaml(IMU_MAPPING)
    imu_key = "tamagawa/imu_link"
    imu = candidate["sensor_mappings"][imu_key]

    without_imu = deepcopy(candidate)
    del without_imu["sensor_mappings"][imu_key]
    without_imu["enabled_sensors"].remove(imu_key)
    assert without_imu == baseline

    assert imu["carla_type"] == "sensor.other.imu"
    assert imu["id"] == "imu"
    assert imu["ros_config"] == {
        "frame_id": "tamagawa/imu_link",
        "topic": "/sensing/imu/tamagawa/imu_raw",
        "frequency_hz": 50,
        "qos_profile": "reliable",
    }
    assert imu["parameters"]["sensor_tick"] == 0.0
    assert all(
        value == 0.0
        for name, value in imu["parameters"].items()
        if name.startswith("noise_")
    )


def test_recommended_profile_uses_and_processes_the_dedicated_imu_mapping():
    wrapper = FAST_WRAPPER.read_text(encoding="utf-8")

    assert "sensor_mapping_vad_fast_reliable_imu.yaml" in wrapper
    assert '"use_vad_imu_acceleration:=true"' in wrapper


def test_dedicated_imu_is_rotation_aligned_with_model_base_frame():
    calibration = load_yaml(CALIBRATION)["sensor_kit_base_link"]["tamagawa/imu_link"]

    assert calibration["roll"] == pytest.approx(0.0)
    assert calibration["pitch"] == pytest.approx(0.0)
    assert calibration["yaw"] == pytest.approx(0.0)
    assert calibration["x"] == pytest.approx(0.025)
    assert calibration["y"] == pytest.approx(0.0)
    assert calibration["z"] == pytest.approx(0.0)


@pytest.mark.parametrize("launch_path", LAUNCHES)
def test_imu_acceleration_is_an_opt_in_vad_only_input(launch_path):
    root = ET.parse(launch_path).getroot()
    args = direct_args(root)

    assert args["use_vad_imu_acceleration"].get("default") == "false"
    assert args["vad_imu_topic"].get("default") == "/sensing/imu/tamagawa/imu_raw"
    assert args["vad_imu_acceleration_topic"].get("default") == (
        "/sensing/imu/vad_acceleration"
    )
    assert args["vad_imu_acceleration_frame"].get("default") == "base_link"

    adapters = [
        node
        for node in root.iter("node")
        if node.get("exec") == "vad_imu_acceleration_adapter.py"
    ]
    assert len(adapters) == 1
    adapter = adapters[0]
    assert adapter.get("if") == "$(var use_vad_imu_acceleration)"
    remaps = {element.get("from"): element.get("to") for element in adapter.findall("./remap")}
    assert remaps == {
        "~/input/imu": "$(var vad_imu_topic)",
        "~/output/acceleration": "$(var vad_imu_acceleration_topic)",
    }

    acceleration_lets = [
        element
        for element in root.findall("./let")
        if element.get("name") == "vad_acceleration_input"
    ]
    assert len(acceleration_lets) == 2
    imu_input = next(element for element in acceleration_lets if element.get("if"))
    default_input = next(element for element in acceleration_lets if element.get("unless"))
    assert imu_input.get("if") == "$(var use_vad_imu_acceleration)"
    assert imu_input.get("value") == "$(var vad_imu_acceleration_topic)"
    assert default_input.get("unless") == "$(var use_vad_imu_acceleration)"
    assert default_input.get("value") == "/localization/acceleration"

    vad_includes = [
        include
        for include in root.iter("include")
        if include.get("file", "").endswith(
            ("vad_carla_tiny.launch.xml", "vad_carla_tiny_fast.launch.xml")
        )
    ]
    assert len(vad_includes) == 2
    assert all(
        include_args(include)["acceleration"] == "$(var vad_acceleration_input)"
        for include in vad_includes
    )


def test_full_launch_keeps_controller_twist2accel_topic_unchanged():
    root = ET.parse(LAUNCHES[1]).getroot()
    twist2accel = next(
        node for node in root.iter("node") if node.get("name") == "twist2accel"
    )
    remaps = {element.get("from"): element.get("to") for element in twist2accel.findall("./remap")}

    assert remaps["output/accel"] == "/localization/acceleration"
    assert "/sensing/imu/vad_acceleration" not in remaps.values()
