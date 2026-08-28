import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


ROOT = Path(__file__).resolve().parents[2]
CARLA_PACKAGE = (
    ROOT
    / "src/universe/autoware_universe/simulator/autoware_carla_interface"
)
CONVERSION_PATH = (
    CARLA_PACKAGE
    / "src/autoware_carla_interface/vehicle_status_conversion.py"
)
LAUNCH_PATH = CARLA_PACKAGE / "launch/autoware_carla_interface.launch.xml"
BRIDGE_PATH = CARLA_PACKAGE / "src/autoware_carla_interface/carla_ros.py"


def load_conversion_module():
    spec = importlib.util.spec_from_file_location("vehicle_status_conversion", CONVERSION_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def fl_angle_from_virtual(virtual_angle, wheel_base, wheel_tread):
    curvature = math.tan(virtual_angle) / wheel_base
    return math.atan(wheel_base * curvature / (1.0 - 0.5 * wheel_tread * curvature))


@pytest.mark.parametrize("virtual_angle", [0.3, -0.3, 0.0])
def test_front_left_ackermann_angle_round_trip(virtual_angle):
    conversion = load_conversion_module()
    wheel_base = 2.79
    wheel_tread = 1.64
    fl_angle = fl_angle_from_virtual(virtual_angle, wheel_base, wheel_tread)

    actual = conversion.fl_wheel_to_virtual_tire_angle(
        fl_angle, wheel_base, wheel_tread
    )

    assert actual == pytest.approx(virtual_angle, abs=1.0e-12)


def test_front_left_bag_regression_value():
    conversion = load_conversion_module()

    actual = conversion.fl_wheel_to_virtual_tire_angle(-0.3341076478, 2.79, 1.64)

    assert actual == pytest.approx(-0.3688650520, abs=1.0e-9)


@pytest.mark.parametrize(
    ("carla_deg_per_sec", "expected_ros_rad_per_sec"),
    [(57.2957795131, -1.0), (-57.2957795131, 1.0), (0.0, 0.0)],
)
def test_carla_yaw_rate_contract(carla_deg_per_sec, expected_ros_rad_per_sec):
    conversion = load_conversion_module()

    assert conversion.carla_yaw_rate_to_ros(carla_deg_per_sec) == pytest.approx(
        expected_ros_rad_per_sec, abs=1.0e-12
    )


@pytest.mark.parametrize("heading_rate", [1.0, -1.0, 0.0])
def test_actor_center_velocity_translates_to_rear_axle(heading_rate):
    conversion = load_conversion_module()
    offset = 1.425
    center_lateral_velocity = offset * heading_rate

    assert conversion.rear_axle_lateral_velocity(
        center_lateral_velocity, heading_rate, offset
    ) == pytest.approx(0.0, abs=1.0e-12)


@pytest.mark.parametrize(
    ("center_lateral_velocity", "heading_rate", "offset"),
    [(math.nan, 0.0, 1.425), (0.0, math.inf, 1.425), (0.0, 0.0, -1.0)],
)
def test_invalid_rear_axle_velocity_inputs_are_rejected(
    center_lateral_velocity, heading_rate, offset
):
    conversion = load_conversion_module()

    with pytest.raises(ValueError):
        conversion.rear_axle_lateral_velocity(
            center_lateral_velocity, heading_rate, offset
        )


@pytest.mark.parametrize(
    ("wheel_base", "wheel_tread"),
    [(0.0, 1.64), (-1.0, 1.64), (2.79, -0.1), (math.nan, 1.64)],
)
def test_invalid_vehicle_geometry_is_rejected(wheel_base, wheel_tread):
    conversion = load_conversion_module()

    with pytest.raises(ValueError):
        conversion.fl_wheel_to_virtual_tire_angle(0.1, wheel_base, wheel_tread)


def test_launch_loads_vehicle_geometry_into_carla_bridge():
    root = ET.parse(LAUNCH_PATH).getroot()
    bridge = next(
        node
        for node in root.findall(".//node")
        if node.attrib.get("name") == "autoware_carla_interface"
    )
    parameter_files = [
        parameter.attrib.get("from", "") for parameter in bridge.findall("param")
    ]

    assert any("vehicle_info.param.yaml" in path for path in parameter_files)


def test_bridge_rotates_world_angular_velocity_into_vehicle_frame():
    source = BRIDGE_PATH.read_text()

    assert "ego_angular_velocity_local = (inv_rot_mat @ angular_vel_vec)" in source
    assert "carla_yaw_rate_to_ros(ego_angular_velocity_local[2])" in source
    assert "rear_axle_lateral_velocity(" in source
