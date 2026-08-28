import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import carla
import pytest
from geometry_msgs.msg import PoseWithCovarianceStamped
from transforms3d.euler import euler2quat


ROOT = Path(__file__).resolve().parents[1]
CARLA_INTERFACE_SRC = (
    ROOT
    / "src/universe/autoware_universe/simulator/autoware_carla_interface/src"
)


@pytest.fixture()
def interface_module(monkeypatch):
    monkeypatch.syspath_prepend(str(CARLA_INTERFACE_SRC))
    for module_name in list(sys.modules):
        if module_name == "autoware_carla_interface" or module_name.startswith(
            "autoware_carla_interface."
        ):
            del sys.modules[module_name]
    import autoware_carla_interface.carla_ros as module

    return module


def _set_ros_yaw(pose, yaw_rad: float) -> None:
    w, x, y, z = euler2quat(0.0, 0.0, yaw_rad)
    pose.orientation.w = w
    pose.orientation.x = x
    pose.orientation.y = y
    pose.orientation.z = z


def test_longitudinal_shift_follows_carla_actor_heading(interface_module) -> None:
    transformer = interface_module.CoordinateTransformer
    center = carla.Transform(
        carla.Location(x=10.0, y=20.0, z=3.0),
        carla.Rotation(pitch=30.0, yaw=90.0),
    )

    rear_axle = transformer.shift_carla_transform_along_local_x(center, -1.425)

    assert rear_axle.location.x == pytest.approx(10.0, abs=1e-6)
    assert rear_axle.location.y == pytest.approx(20.0 - 1.425 * math.cos(math.radians(30)))
    assert rear_axle.location.z == pytest.approx(3.0 - 1.425 * math.sin(math.radians(30)))
    assert rear_axle.rotation.yaw == pytest.approx(90.0)


def test_initialpose_places_actor_center_ahead_of_requested_base_link(
    interface_module,
) -> None:
    bridge = interface_module.carla_ros2_interface.__new__(
        interface_module.carla_ros2_interface
    )
    received = []
    bridge.sensor_loader = SimpleNamespace(wheelbase=2.85)
    bridge._state_lock = threading.Lock()
    bridge.ego_actor = SimpleNamespace(set_transform=received.append)
    bridge.logger = SimpleNamespace(warning=lambda *_: None)

    message = PoseWithCovarianceStamped()
    message.pose.pose.position.x = 10.0
    message.pose.pose.position.y = -20.0
    message.pose.pose.position.z = 1.0
    _set_ros_yaw(message.pose.pose, -math.pi / 2.0)

    bridge.initialpose_callback(message)

    assert len(received) == 1
    actor_transform = received[0]
    assert actor_transform.location.x == pytest.approx(10.0, abs=1e-6)
    assert actor_transform.location.y == pytest.approx(21.425, abs=1e-6)
    assert actor_transform.location.z == pytest.approx(3.0, abs=1e-6)
    assert actor_transform.rotation.yaw == pytest.approx(90.0, abs=1e-6)
    assert message.pose.pose.position.z == pytest.approx(1.0)


def test_published_pose_is_rear_axle_base_link(interface_module) -> None:
    bridge = interface_module.carla_ros2_interface.__new__(
        interface_module.carla_ros2_interface
    )
    published = []
    config = SimpleNamespace(
        publisher=SimpleNamespace(publish=published.append),
        sensor_id="gnss",
        covariance={},
    )
    bridge.sensor_loader = SimpleNamespace(wheelbase=2.85)
    bridge._state_lock = threading.Lock()
    bridge.ego_actor = SimpleNamespace(
        get_transform=lambda: carla.Transform(
            carla.Location(x=10.0, y=20.0, z=3.0),
            carla.Rotation(yaw=90.0),
        )
    )
    bridge.sensor_registry = SimpleNamespace(
        get_sensor=lambda sensor_id: config if sensor_id == "gnss" else None,
        update_sensor_timestamp=lambda *_: None,
    )
    bridge.logger = SimpleNamespace(warning=lambda *_: None)
    bridge.timestamp = 5.0
    bridge.checkFrequency = lambda *_: False

    bridge.pose()

    assert len(published) == 1
    pose = published[0].pose.pose
    assert pose.position.x == pytest.approx(10.0, abs=1e-6)
    assert pose.position.y == pytest.approx(-18.575, abs=1e-6)
    assert pose.position.z == pytest.approx(3.0, abs=1e-6)
    assert pose.orientation.z == pytest.approx(-math.sqrt(0.5), abs=1e-6)
    assert pose.orientation.w == pytest.approx(math.sqrt(0.5), abs=1e-6)


@pytest.mark.parametrize("yaw_deg", [0.0, 90.0, -90.0])
def test_base_link_route_spawn_round_trips_through_actor_center(
    interface_module, yaw_deg: float
) -> None:
    from autoware_carla_interface.carla_autoware import InitializeInterface

    launcher = InitializeInterface.__new__(InitializeInterface)
    launcher.spawn_point = f"10.0,20.0,1.0,0.0,0.0,{yaw_deg}"
    launcher.spawn_point_reference = "base_link"
    launcher.interface = SimpleNamespace(sensor_loader=SimpleNamespace(wheelbase=2.85))

    actor_transform, randomize = launcher._parse_spawn_point()
    published = []
    config = SimpleNamespace(
        publisher=SimpleNamespace(publish=published.append),
        sensor_id="gnss",
        covariance={},
    )
    bridge = interface_module.carla_ros2_interface.__new__(
        interface_module.carla_ros2_interface
    )
    bridge.sensor_loader = SimpleNamespace(wheelbase=2.85)
    bridge._state_lock = threading.Lock()
    bridge.ego_actor = SimpleNamespace(get_transform=lambda: actor_transform)
    bridge.sensor_registry = SimpleNamespace(
        get_sensor=lambda sensor_id: config if sensor_id == "gnss" else None,
        update_sensor_timestamp=lambda *_: None,
    )
    bridge.logger = SimpleNamespace(warning=lambda *_: None)
    bridge.timestamp = 5.0
    bridge.checkFrequency = lambda *_: False

    bridge.pose()

    assert not randomize
    assert len(published) == 1
    pose = published[0].pose.pose
    assert pose.position.x == pytest.approx(10.0, abs=1e-6)
    assert pose.position.y == pytest.approx(-20.0, abs=1e-6)
    assert pose.position.z == pytest.approx(3.0, abs=1e-6)


def test_vehicle_center_spawn_reference_remains_backward_compatible(
    interface_module,
) -> None:
    from autoware_carla_interface.carla_autoware import InitializeInterface

    launcher = InitializeInterface.__new__(InitializeInterface)
    launcher.spawn_point = "10.0,20.0,1.0,0.0,0.0,90.0"
    launcher.spawn_point_reference = "vehicle_center"
    launcher.interface = SimpleNamespace(sensor_loader=SimpleNamespace(wheelbase=2.85))

    actor_transform, randomize = launcher._parse_spawn_point()

    assert not randomize
    assert actor_transform.location.x == pytest.approx(10.0)
    assert actor_transform.location.y == pytest.approx(20.0)
    assert actor_transform.location.z == pytest.approx(3.0)


def test_route_launches_select_base_link_spawn_at_the_boundary() -> None:
    interface_launch = ET.parse(
        CARLA_INTERFACE_SRC.parent / "launch/autoware_carla_interface.launch.xml"
    ).getroot()
    reference_arg = next(
        arg
        for arg in interface_launch.findall(".//arg")
        if arg.attrib.get("name") == "spawn_point_reference"
    )
    assert reference_arg.attrib["default"] == "vehicle_center"

    for name in ("run_route_vad.sh", "run_route_vad_full.sh"):
        source = (ROOT / "scripts/e2e" / name).read_text(encoding="utf-8")
        assert 'route.get("spawn_point_reference", "base_link")' in source
        assert "spawn_point_reference:=base_link" in source


def test_route_generator_declares_base_link_coordinate_contract() -> None:
    source = (ROOT / "scripts/e2e/prepare_carla_route.py").read_text(encoding="utf-8")
    assert '"coordinate_reference": "base_link"' in source
    assert '"spawn_point_reference": "base_link"' in source


def test_builds_apply_base_link_pose_patch_after_dependencies() -> None:
    for name in ("build.sh", "build_full.sh"):
        source = (ROOT / "scripts/e2e" / name).read_text(encoding="utf-8")
        fast_sensor = source.index("apply_carla_fast_sensor_patch.sh")
        sensor_frame = source.index("apply_carla_sensor_frame_patch.sh")
        vehicle_status = source.index("apply_carla_vehicle_status_patch.sh")
        base_link_pose = source.index("apply_carla_base_link_pose_patch.sh")
        route_contract = source.index("apply_carla_base_link_route_contract_patch.sh")
        imu_timestamp = source.index("apply_carla_imu_source_timestamp_patch.sh")
        assert max(fast_sensor, sensor_frame, vehicle_status) < base_link_pose
        assert base_link_pose < route_contract
        assert route_contract < imu_timestamp
