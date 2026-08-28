import math
from pathlib import Path
import sys

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
CARLA_INTERFACE_SRC = (
    ROOT
    / "src/universe/autoware_universe/simulator/autoware_carla_interface/src"
)
CALIBRATION = (
    ROOT
    / "src/launcher/autoware_launch/sensor_kit/carla_sensor_kit_launch/"
    "carla_sensor_kit_description/config/sensor_kit_calibration.yaml"
)


def _load_calibration() -> dict:
    return yaml.safe_load(CALIBRATION.read_text(encoding="utf-8"))[
        "sensor_kit_base_link"
    ]


def test_calibration_uses_ros_base_link_handedness() -> None:
    sensors = _load_calibration()
    assert sensors["CAM_FRONT_LEFT/camera_link"]["y"] == pytest.approx(0.55)
    assert sensors["CAM_FRONT_LEFT/camera_link"]["yaw"] == pytest.approx(
        math.radians(55), abs=1e-6
    )
    assert sensors["CAM_FRONT_RIGHT/camera_link"]["y"] == pytest.approx(-0.55)
    assert sensors["CAM_FRONT_RIGHT/camera_link"]["yaw"] == pytest.approx(
        math.radians(-55), abs=1e-6
    )
    assert sensors["CAM_BACK_LEFT/camera_link"]["y"] == pytest.approx(0.55)
    assert sensors["CAM_BACK_RIGHT/camera_link"]["y"] == pytest.approx(-0.55)


def test_loader_converts_ros_calibration_back_to_training_carla_rig(
    monkeypatch,
) -> None:
    monkeypatch.syspath_prepend(str(CARLA_INTERFACE_SRC))
    for module_name in list(sys.modules):
        if module_name == "autoware_carla_interface" or module_name.startswith(
            "autoware_carla_interface."
        ):
            del sys.modules[module_name]

    from autoware_carla_interface.modules.sensor_kit_loader import SensorKitLoader

    sensors = _load_calibration()
    loader = SensorKitLoader()
    front_left = loader._ros_baselink_to_vehicle_center_transform(
        sensors["CAM_FRONT_LEFT/camera_link"]
    )
    front_right = loader._ros_baselink_to_vehicle_center_transform(
        sensors["CAM_FRONT_RIGHT/camera_link"]
    )

    assert front_left["x"] == pytest.approx(0.27, abs=1e-6)
    assert front_left["y"] == pytest.approx(-0.55, abs=1e-6)
    assert front_left["yaw"] == pytest.approx(-55.0, abs=1e-5)
    assert front_right["x"] == pytest.approx(0.27, abs=1e-6)
    assert front_right["y"] == pytest.approx(0.55, abs=1e-6)
    assert front_right["yaw"] == pytest.approx(55.0, abs=1e-5)


def test_builds_apply_sensor_frame_patches_in_dependency_order() -> None:
    for name in ("build.sh", "build_full.sh"):
        source = (ROOT / "scripts/e2e" / name).read_text(encoding="utf-8")
        frame = source.index("apply_vad_frame_assembly_patch.sh")
        carla_fast = source.index("apply_carla_fast_sensor_patch.sh")
        sensor_frame = source.index("apply_carla_sensor_frame_patch.sh")
        assert frame < carla_fast
        assert carla_fast < sensor_frame
