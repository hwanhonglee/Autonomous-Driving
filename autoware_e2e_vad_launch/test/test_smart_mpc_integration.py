from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[2]
LAUNCH_REPOSITORY = ROOT / "src/launcher/autoware_launch"
UNIVERSE = ROOT / "src/universe/autoware_universe"
DEFAULT_PRESET = (
    LAUNCH_REPOSITORY
    / "autoware_launch/config/control/preset/default_preset.yaml"
)
SMART_PRESET = (
    LAUNCH_REPOSITORY
    / "autoware_launch/config/control/preset/smart_mpc_nominal_ilqr_preset.yaml"
)
SMART_PACKAGE = (
    UNIVERSE / "control/autoware_smart_mpc_trajectory_follower"
)
sys.path.insert(0, str(SMART_PACKAGE))

from autoware_smart_mpc_trajectory_follower.reference_horizon import (  # noqa: E402
    DEFAULT_GOAL_STOP_PRESERVE_DISTANCE_M,
    condition_distant_goal_stop_reference,
)


FULL_LAUNCH = ROOT / "autoware_e2e_vad_launch/launch/carla_vad_full.launch.xml"
SMART_WRAPPER = ROOT / "scripts/e2e/run_route_vad_smart_mpc.sh"
SMART_SMOKE = ROOT / "scripts/e2e/smoke_smart_mpc.py"
SMART_RUNTIME_DEFAULT = (
    SMART_PACKAGE
    / "autoware_smart_mpc_trajectory_follower/param/runtime.param.yaml"
)
SMART_RUNTIME_GUARD = (
    ROOT / "autoware_e2e_vad_launch/config/smart_mpc_goal_stop_guard.param.yaml"
)
CONTROL_COMPONENT = (
    LAUNCH_REPOSITORY
    / "autoware_launch/launch/components/tier4_control_component.launch.xml"
)
AUTOWARE_LAUNCH = LAUNCH_REPOSITORY / "autoware_launch/launch/autoware.launch.xml"
TIER4_CONTROL_LAUNCH = (
    LAUNCH_REPOSITORY
    / "tier4_universe_launch/tier4_control_launch/launch/control.launch.xml"
)


def load_yaml(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def direct_xml_args(path):
    return {
        arg.get("name"): arg.get("default")
        for arg in ET.parse(path).getroot().findall("./arg")
    }


def include_xml_args(path, target_suffix):
    root = ET.parse(path).getroot()
    include = next(
        item for item in root.iter("include") if item.get("file", "").endswith(target_suffix)
    )
    return {arg.get("name"): arg.get("value") for arg in include.findall("./arg")}


def preset_args(path):
    return {
        item["arg"]["name"]: item["arg"].get("default")
        for item in load_yaml(path)["launch"]
    }


def test_smart_mpc_is_strictly_opt_in():
    default = preset_args(DEFAULT_PRESET)
    smart = preset_args(SMART_PRESET)

    assert default["trajectory_follower_mode"] == "trajectory_follower_node"
    assert smart["trajectory_follower_mode"] == "smart_mpc_trajectory_follower"
    assert {
        key: value for key, value in default.items() if key != "trajectory_follower_mode"
    } == {
        key: value for key, value in smart.items() if key != "trajectory_follower_mode"
    }

    launch_args = {
        arg.get("name"): arg.get("default")
        for arg in ET.parse(FULL_LAUNCH).getroot().findall("./arg")
    }
    assert launch_args["control_module_preset"] == "default"
    assert "control_module_preset:=smart_mpc_nominal_ilqr" in SMART_WRAPPER.read_text(
        encoding="utf-8"
    )
    smoke_source = SMART_SMOKE.read_text(encoding="utf-8")
    assert 'default=142' in smoke_source
    assert "Refusing to use active project ROS domain" in smoke_source


def test_vehicle_command_gate_profile_is_plumbed_without_changing_default():
    stock = (
        "$(find-pkg-share autoware_launch)/config/control/vehicle_cmd_gate/"
        "vehicle_cmd_gate.param.yaml"
    )

    assert direct_xml_args(AUTOWARE_LAUNCH)["vehicle_cmd_gate_param_path"] == stock
    assert direct_xml_args(CONTROL_COMPONENT)["vehicle_cmd_gate_param_path"] == stock
    assert include_xml_args(AUTOWARE_LAUNCH, "tier4_control_component.launch.xml")[
        "vehicle_cmd_gate_param_path"
    ] == "$(var vehicle_cmd_gate_param_path)"
    assert include_xml_args(CONTROL_COMPONENT, "control.launch.xml")[
        "vehicle_cmd_gate_param_path"
    ] == "$(var vehicle_cmd_gate_param_path)"


def test_smart_mpc_pinned_profile_is_nominal_ilqr():
    params = SMART_PACKAGE / "autoware_smart_mpc_trajectory_follower/param"
    mpc = load_yaml(params / "mpc_param.yaml")
    nominal = load_yaml(params / "nominal_param.yaml")
    trained = load_yaml(params / "trained_model_param.yaml")

    assert mpc["mpc_parameter"]["system"]["mode"] == "ilqr"
    assert not trained["trained_model_parameter"]["control_application"][
        "use_trained_model"
    ]
    steering = nominal["nominal_parameter"]["steering"]
    assert steering["steer_time_delay"] == 0.27
    assert steering["steer_time_constant"] == 0.24


def test_runtime_patch_waits_for_every_required_input():
    source = (
        SMART_PACKAGE / "scripts/pympc_trajectory_follower.py"
    ).read_text(encoding="utf-8")

    assert "self._present_acceleration," in source
    assert "self._present_steering_status," in source
    assert "if len(self._present_trajectory.points) < 2:" in source
    assert "cmd_msg.longitudinal.is_defined_acceleration = True" in source
    assert "np.abs(orientation_deviation) > orientation_deviation_threshold" in source


def runtime_params(path):
    return load_yaml(path)["/**"]["ros__parameters"]


def guard_profile():
    positions = [(index * 0.5, 0.0) for index in range(9)]
    velocities = [2.5, 2.33, 2.06, 1.74, 1.35, 0.79, 0.0, 0.0, 0.0]
    return positions, velocities


def apply_guard(positions, velocities, nearest_index=0, goal_position=(4.0, 0.0), **kwargs):
    settings = {
        "enabled": True,
        "zero_speed_threshold_mps": 0.05,
        "preserve_distance_m": DEFAULT_GOAL_STOP_PRESERVE_DISTANCE_M,
        "minimum_speed_mps": 1.5,
        "goal_match_tolerance_m": 2.0,
    }
    settings.update(kwargs)
    return condition_distant_goal_stop_reference(
        positions,
        velocities,
        nearest_index,
        goal_position,
        **settings,
    )


def test_goal_stop_guard_profiles_are_strictly_opt_in():
    default = runtime_params(SMART_RUNTIME_DEFAULT)
    guard = runtime_params(SMART_RUNTIME_GUARD)

    assert not default["enable_reference_horizon_goal_stop_guard"]
    assert guard["enable_reference_horizon_goal_stop_guard"]
    preserve_parameter = "reference_horizon_goal_stop_preserve_distance_m"
    assert DEFAULT_GOAL_STOP_PRESERVE_DISTANCE_M == 0.5
    assert default[preserve_parameter] == DEFAULT_GOAL_STOP_PRESERVE_DISTANCE_M
    assert guard[preserve_parameter] == DEFAULT_GOAL_STOP_PRESERVE_DISTANCE_M
    assert {
        key: value
        for key, value in default.items()
        if key != "enable_reference_horizon_goal_stop_guard"
    } == {
        key: value
        for key, value in guard.items()
        if key != "enable_reference_horizon_goal_stop_guard"
    }


def test_disabled_goal_stop_guard_returns_reference_unchanged():
    positions, velocities = guard_profile()
    result = apply_guard(positions, velocities, enabled=False)

    assert not result.active
    assert result.reason == "disabled"
    assert result.velocities == tuple(velocities)


def test_distant_matching_terminal_goal_stop_is_guarded():
    positions, velocities = guard_profile()
    result = apply_guard(positions, velocities)

    assert result.active
    assert result.reason == "distant_goal_stop_guarded"
    assert result.stop_distance_m == 3.0
    assert result.tail_start_index == 6
    assert result.floored_points == 5
    assert result.velocities[:4] == tuple(velocities[:4])
    assert result.velocities[4:] == (1.5, 1.5, 1.5, 1.5, 1.5)


def test_goal_stop_guard_restores_original_reference_inside_preserve_distance():
    positions, velocities = guard_profile()
    result = apply_guard(positions, velocities, nearest_index=5)

    assert not result.active
    assert result.reason == "inside_preserve_distance"
    assert result.stop_distance_m == DEFAULT_GOAL_STOP_PRESERVE_DISTANCE_M
    assert result.velocities == tuple(velocities)


def test_goal_stop_guard_remains_active_just_outside_deployed_preserve_distance():
    positions, velocities = guard_profile()
    result = apply_guard(positions, velocities, nearest_index=4)

    assert result.active
    assert result.stop_distance_m == 1.0
    assert result.velocities[4:] == (1.5, 1.5, 1.5, 1.5, 1.5)


def test_goal_stop_guard_rejects_non_goal_and_non_terminal_stops():
    positions, velocities = guard_profile()
    mismatch = apply_guard(positions, velocities, goal_position=(10.0, 0.0))
    mid_route = apply_guard(
        positions[:4],
        [2.0, 0.0, 2.0, 2.0],
        goal_position=(1.5, 0.0),
    )

    assert not mismatch.active
    assert mismatch.reason == "stop_point_goal_mismatch"
    assert mismatch.velocities == tuple(velocities)
    assert not mid_route.active
    assert mid_route.reason == "no_terminal_zero_tail"
    assert mid_route.velocities == (2.0, 0.0, 2.0, 2.0)


def test_goal_stop_guard_preserves_intermediate_stop_before_terminal_goal_stop():
    positions = [(index * 0.5, 0.0) for index in range(7)]
    velocities = [2.0, 0.0, 2.0, 1.5, 0.8, 0.0, 0.0]
    result = apply_guard(positions, velocities, goal_position=(3.0, 0.0))

    assert not result.active
    assert result.reason == "non_terminal_zero_speed"
    assert result.velocities == tuple(velocities)


def test_goal_stop_guard_preserves_stop_tail_away_from_mission_goal():
    positions = [(index * 0.5, 0.0) for index in range(9)]
    velocities = [2.5, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    result = apply_guard(positions, velocities, goal_position=(4.0, 0.0))

    assert not result.active
    assert result.reason == "stop_point_goal_mismatch"
    assert result.velocities == tuple(velocities)


def test_goal_stop_guard_does_not_restart_inside_zero_tail():
    positions, velocities = guard_profile()
    result = apply_guard(positions, velocities, nearest_index=6)

    assert not result.active
    assert result.reason == "at_terminal_stop"
    assert result.velocities == tuple(velocities)


def test_smart_mpc_runtime_param_path_reaches_only_the_smart_node():
    launch_paths = (FULL_LAUNCH, AUTOWARE_LAUNCH, CONTROL_COMPONENT, TIER4_CONTROL_LAUNCH)
    for path in launch_paths:
        root = ET.parse(path).getroot()
        args = {arg.get("name"): arg.get("default") for arg in root.findall("./arg")}
        assert "smart_mpc_runtime_param_path" in args

    full_default = {
        arg.get("name"): arg.get("default")
        for arg in ET.parse(FULL_LAUNCH).getroot().findall("./arg")
    }["smart_mpc_runtime_param_path"]
    assert full_default.endswith("/param/runtime.param.yaml")

    for path in (FULL_LAUNCH, AUTOWARE_LAUNCH, CONTROL_COMPONENT):
        source = path.read_text(encoding="utf-8")
        forwarded_arg = (
            'name="smart_mpc_runtime_param_path" '
            'value="$(var smart_mpc_runtime_param_path)"'
        )
        assert forwarded_arg in source

    tier4_root = ET.parse(TIER4_CONTROL_LAUNCH).getroot()
    smart_nodes = [
        node
        for node in tier4_root.findall(".//node")
        if node.get("pkg") == "autoware_smart_mpc_trajectory_follower"
    ]
    assert len(smart_nodes) == 1
    params = smart_nodes[0].findall("./param")
    assert [param.get("from") for param in params] == ["$(var smart_mpc_runtime_param_path)"]


def test_guard_changes_only_internal_velocity_reference():
    source = (SMART_PACKAGE / "scripts/pympc_trajectory_follower.py").read_text(
        encoding="utf-8"
    )

    assert (
        "raw_trajectory_longitudinal_velocity = trajectory_longitudinal_velocity.copy()"
        in source
    )
    assert (
        "cmd_msg.longitudinal.velocity = raw_trajectory_longitudinal_velocity[nearestIndex]"
        in source
    )
