from pathlib import Path
import sys

from rcl_interfaces.msg import ParameterType


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from vad_aeb_configurator import aeb_parameter_overrides, bool_parameter


def test_aeb_uses_vad_objects_without_unsegmented_lidar():
    overrides = dict(aeb_parameter_overrides())
    assert overrides == {
        "use_pointcloud_data": False,
        "use_predicted_object_data": True,
    }


def test_bool_parameter_has_explicit_ros_type():
    parameter = bool_parameter("enabled", True)
    assert parameter.name == "enabled"
    assert parameter.value.bool_value is True
    assert parameter.value.type == ParameterType.PARAMETER_BOOL
