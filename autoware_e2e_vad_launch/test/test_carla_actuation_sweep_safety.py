import importlib.util
from pathlib import Path
import sys

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parent
LOGIC_PATH = PACKAGE_ROOT / "scripts/carla_actuation_sweep_logic.py"
PROFILE_PATH = PACKAGE_ROOT / "config/carla_actuation_sweep.param.yaml"
LAUNCH_PATH = PACKAGE_ROOT / "launch/carla_actuation_calibration.launch.xml"
INTERFACE_PATH = (
    WORKSPACE_ROOT
    / "src/universe/autoware_universe/simulator/autoware_carla_interface"
    / "src/autoware_carla_interface/carla_ros.py"
)

SPEC = importlib.util.spec_from_file_location("carla_actuation_sweep_logic", LOGIC_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_initialpose_z_cancels_interface_spawn_offset():
    assert MODULE.initialpose_input_z(0.036, 2.0) == pytest.approx(-1.964)
    with pytest.raises(ValueError):
        MODULE.initialpose_input_z(0.0, -1.0)


def test_reverse_monitor_stays_active_with_phase_specific_settling_limit():
    arguments = {
        "maximum_reverse_velocity_mps": 0.1,
        "maximum_settling_reverse_velocity_mps": 0.5,
    }
    assert not MODULE.reverse_velocity_is_safe(
        phase="sample", velocity_mps=-0.11, **arguments
    )
    assert MODULE.reverse_velocity_is_safe(
        phase="reset_settle", velocity_mps=-0.3, **arguments
    )
    assert not MODULE.reverse_velocity_is_safe(
        phase="reset_settle", velocity_mps=-0.51, **arguments
    )


def test_ground_reference_requires_a_continuous_stability_window():
    anchor, since, stable = MODULE.update_stability_window(
        anchor_value=None,
        stable_since_s=None,
        current_value=2.4,
        current_time_s=0.1,
        maximum_delta=0.01,
        required_duration_s=0.75,
    )
    assert not stable
    anchor, since, stable = MODULE.update_stability_window(
        anchor_value=anchor,
        stable_since_s=since,
        current_value=1.0,
        current_time_s=0.5,
        maximum_delta=0.01,
        required_duration_s=0.75,
    )
    assert (anchor, since, stable) == (1.0, 0.5, False)
    anchor, since, stable = MODULE.update_stability_window(
        anchor_value=anchor,
        stable_since_s=since,
        current_value=1.006,
        current_time_s=1.25,
        maximum_delta=0.01,
        required_duration_s=0.75,
    )
    assert stable


def test_profile_measures_ground_pose_and_requires_stationary_reset_hold():
    profile = yaml.safe_load(PROFILE_PATH.read_text())["/**"]["ros__parameters"]
    assert profile["initialpose_interface_z_offset_m"] == 2.0
    assert profile["reset_reference_stability_s"] > profile["stop_hold_s"]
    assert profile["maximum_reset_reference_z_delta_m"] <= 0.01
    assert profile["reset_stop_hold_s"] >= profile["stop_hold_s"]
    assert (
        profile["maximum_settling_reverse_velocity_mps"]
        > profile["maximum_reverse_velocity_mps"]
    )
    assert "reset_pose_z_m" not in profile


def test_calibration_only_enables_interface_stopped_hand_brake():
    launch = LAUNCH_PATH.read_text()
    interface = INTERFACE_PATH.read_text()
    assert '<param name="enable_stopped_hand_brake" value="true"/>' in launch
    assert '"enable_stopped_hand_brake": (rclpy.Parameter.Type.BOOL, False)' in interface
    assert 'out_cmd.hand_brake = bool(' in interface
