import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
from copy import deepcopy
import xml.etree.ElementTree as ET

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "autoware_e2e_vad_launch"
UNIVERSE = ROOT / "src/universe/autoware_universe"
CARLA_INTERFACE = UNIVERSE / "simulator/autoware_carla_interface"
VAD_PACKAGE = UNIVERSE / "e2e/autoware_tensorrt_vad"

BASELINE_MAPPING = PACKAGE / "config/sensor_mapping_full.yaml"
FAST_MAPPING = PACKAGE / "config/sensor_mapping_vad_fast.yaml"
RELIABLE_FAST_MAPPING = PACKAGE / "config/sensor_mapping_vad_fast_reliable.yaml"
SYNC_QUEUE32_PARAMS = ROOT / "data/generated/vad/sync_queue32.param.yaml"
RELIABLE_SYNC_QUEUE32_PARAMS = PACKAGE / "config/vad_carla_tiny_recommended.param.yaml"
RECOMMENDED_MPC_PARAMS = PACKAGE / "config/mpc_carla_recommended.param.yaml"
CARLA_PID_PARAMS = PACKAGE / "config/pid_carla_vad_no_steer_convergence.param.yaml"
STOCK_PID_PARAMS = (
    ROOT
    / "src/launcher/autoware_launch/autoware_launch/config/control/trajectory_follower/longitudinal/pid.param.yaml"
)
FAST_VAD_PARAMS = PACKAGE / "config/vad_carla_tiny_fast.param.yaml"
FP16_VAD_PARAMS = PACKAGE / "config/vad_carla_tiny_fast_fp16_heads.param.yaml"
MODEL_PARAMS = ROOT / "data/ml_models/vad/v0.1/vad-carla-tiny.param.json"
MINIMAL_LAUNCH = PACKAGE / "launch/carla_vad.launch.xml"
FULL_LAUNCH = PACKAGE / "launch/carla_vad_full.launch.xml"
SYSTEM_VAD_LAUNCH = PACKAGE / "launch/system_vad_full_shell.launch.xml"
FAST_VAD_LAUNCH = PACKAGE / "launch/vad_carla_tiny_fast.launch.xml"
VAD_COMPONENT_MONITOR = PACKAGE / "config/component_state_monitor_vad.yaml"
VAD_DIAGNOSTIC_GRAPH = PACKAGE / "config/diagnostic_graph_vad.yaml"
VAD_ROUTE_MANAGER_PARAMS = PACKAGE / "config/vad_route_manager.param.yaml"
STANDARD_DIAGNOSTIC_DIR = (
    ROOT / "src/launcher/autoware_launch/autoware_launch/config/system/diagnostics"
)
STANDARD_COMPONENT_MONITOR = (
    ROOT
    / "src/launcher/autoware_launch/autoware_launch/config/system/component_state_monitor/topics.yaml"
)
OFFICIAL_VAD_CONFIG = VAD_PACKAGE / "config/vad_carla_tiny.param.yaml"
CARLA_BRIDGE_LAUNCH = CARLA_INTERFACE / "launch/autoware_carla_interface.launch.xml"
CARLA_WRAPPER_SOURCE = (
    CARLA_INTERFACE
    / "src/autoware_carla_interface/modules/carla_wrapper.py"
)
FAST_WRAPPER = ROOT / "scripts/e2e/run_route_vad_fast.sh"
RECORDED_WRAPPER = ROOT / "scripts/e2e/run_recorded_route_trial.sh"
SMART_WRAPPER = ROOT / "scripts/e2e/run_route_vad_smart_mpc.sh"

CAMERAS = (
    "CAM_FRONT",
    "CAM_BACK",
    "CAM_FRONT_LEFT",
    "CAM_BACK_LEFT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK_RIGHT",
)
CAMERA_KEYS = {f"{camera}/camera_link" for camera in CAMERAS}
RAW_FLAGS = [True] * len(CAMERAS)


def load_yaml(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def ros_parameters(path):
    return load_yaml(path)["/**"]["ros__parameters"]


def camera_mappings(mapping):
    return {
        value["id"]: value
        for value in mapping["sensor_mappings"].values()
        if value.get("carla_type") == "sensor.camera.rgb"
    }


def parse_xml(path):
    return ET.parse(path).getroot()


def direct_args(root):
    return {element.get("name"): element for element in root.findall("./arg")}


def find_include(root, filename):
    matches = [
        element
        for element in root.iter("include")
        if element.get("file", "").endswith(filename)
    ]
    assert len(matches) == 1, f"expected one include for {filename}, got {len(matches)}"
    return matches[0]


def include_args(include):
    return {element.get("name"): element.get("value") for element in include.findall("./arg")}


def inherited_conditions(root, element):
    parents = {child: parent for parent in root.iter() for child in parent}
    conditions = []
    current = element
    while current in parents:
        current = parents[current]
        for kind in ("if", "unless"):
            if current.get(kind) is not None:
                conditions.append((kind, current.get(kind)))
    return conditions


def resolve_remap_target(root, target):
    prefix = "$(var "
    if target.startswith(prefix) and target.endswith(")"):
        name = target[len(prefix) : -1]
        return direct_args(root)[name].get("default")
    return target


def camera_intrinsics(width, height, fov_degrees):
    focal = width / (2.0 * math.tan(math.radians(fov_degrees) / 2.0))
    return (focal, focal, width / 2.0, height / 2.0)


def scaled_intrinsics(width, height, fov_degrees, target_width, target_height):
    fx, fy, cx, cy = camera_intrinsics(width, height, fov_degrees)
    return (
        fx * target_width / width,
        fy * target_height / height,
        cx * target_width / width,
        cy * target_height / height,
    )


def test_verified_baseline_sensor_contract_is_unchanged():
    mapping = load_yaml(BASELINE_MAPPING)
    cameras = camera_mappings(mapping)

    assert set(cameras) == set(CAMERAS)
    assert CAMERA_KEYS <= set(mapping["enabled_sensors"])
    for camera, config in cameras.items():
        parameters = config["parameters"]
        ros = config["ros_config"]
        assert parameters["image_size_x"] == 1600
        assert parameters["image_size_y"] == 900
        assert parameters["fov"] == (110.0 if camera == "CAM_BACK" else 70.0)
        assert "sensor_tick" not in parameters
        assert "enable_postprocess_effects" not in parameters
        assert ros["frequency_hz"] == 11
        assert ros["qos_profile"] == "reliable"


def test_fast_sensor_mapping_is_six_camera_raw_profile():
    mapping = load_yaml(FAST_MAPPING)
    cameras = camera_mappings(mapping)

    assert set(cameras) == set(CAMERAS)
    assert set(mapping["enabled_sensors"]) == CAMERA_KEYS | {"gnss_link"}
    image_topics = set()
    info_topics = set()
    for camera, config in cameras.items():
        parameters = config["parameters"]
        ros = config["ros_config"]
        assert parameters["image_size_x"] == 640
        assert parameters["image_size_y"] == 360
        assert parameters["fov"] == (110.0 if camera == "CAM_BACK" else 70.0)
        assert parameters["sensor_tick"] == pytest.approx(0.2)
        assert parameters["enable_postprocess_effects"] is False
        assert ros["frequency_hz"] == 5
        assert ros["frequency_hz"] * parameters["sensor_tick"] == pytest.approx(1.0)
        assert ros["qos_profile"] == "best_effort"
        assert ros["topic_image"] == f"/sensing/camera/{camera}/image_raw"
        assert ros["topic_info"] == f"/sensing/camera/{camera}/camera_info"
        image_topics.add(ros["topic_image"])
        info_topics.add(ros["topic_info"])

    assert len(image_topics) == len(CAMERAS)
    assert len(info_topics) == len(CAMERAS)
    gnss = mapping["sensor_mappings"]["gnss_link"]
    assert gnss["ros_config"]["qos_profile"] == "reliable"


def test_reliable_fast_mapping_uses_exact_frame_capture_and_reliable_qos():
    baseline = load_yaml(FAST_MAPPING)
    candidate = load_yaml(RELIABLE_FAST_MAPPING)

    expected = deepcopy(baseline)
    for config in camera_mappings(expected).values():
        config["ros_config"]["qos_profile"] = "reliable"
        config["parameters"]["sensor_tick"] = 0.0

    assert candidate == expected
    assert set(camera_mappings(candidate)) == set(CAMERAS)
    assert all(
        config["ros_config"]["qos_profile"] == "reliable"
        for config in camera_mappings(candidate).values()
    )
    assert all(
        config["parameters"]["sensor_tick"] == pytest.approx(0.0)
        for config in camera_mappings(candidate).values()
    )


def test_reliable_reader_override_extends_queue32_without_changing_base_candidate():
    baseline = ros_parameters(SYNC_QUEUE32_PARAMS)
    candidate = ros_parameters(RELIABLE_SYNC_QUEUE32_PARAMS)

    assert baseline["sync_params"] == {
        "frame_buffer_size": 32,
        "image_queue_depth": 32,
    }
    expected = deepcopy(baseline)
    expected["sync_params"]["image_reliability"] = "reliable"
    assert candidate == expected


def test_fast_vad_params_match_raw_camera_dimensions_without_changing_networks():
    params = ros_parameters(FAST_VAD_PARAMS)
    official = ros_parameters(OFFICIAL_VAD_CONFIG)
    model = json.loads(MODEL_PARAMS.read_text(encoding="utf-8"))

    assert params["node_params"]["use_raw"] == RAW_FLAGS
    assert params["interface_params"] == {
        "input_image_width": 640,
        "input_image_height": 360,
    }
    assert "model_params" not in params
    assert official["node_params"]["num_cameras"] == len(CAMERAS)
    assert official["model_params"]["nets"]["backbone"]["precision"] == "fp16"
    assert official["model_params"]["nets"]["head"]["precision"] == "fp32"
    assert official["model_params"]["nets"]["head_no_prev"]["precision"] == "fp32"
    assert model["input_specs"] == {
        "target_image_width": 640,
        "target_image_height": 384,
        "num_cameras": len(CAMERAS),
        "camera_order": [name.removeprefix("CAM_") for name in CAMERAS],
    }


def test_experimental_fp16_heads_use_unique_engine_cache_paths():
    official_nets = ros_parameters(OFFICIAL_VAD_CONFIG)["model_params"]["nets"]
    fp16_nets = ros_parameters(FP16_VAD_PARAMS)["model_params"]["nets"]

    assert ros_parameters(FP16_VAD_PARAMS)["node_params"]["use_raw"] == RAW_FLAGS
    for name in ("head", "head_no_prev"):
        override = fp16_nets[name]
        assert override["precision"] == "fp16"
        assert override["engine_path"] != official_nets[name]["engine_path"]
        assert "fp16" in override["engine_path"]
        assert override["engine_path"].endswith(".engine")
    assert fp16_nets["head"]["engine_path"] != fp16_nets["head_no_prev"]["engine_path"]


@pytest.mark.parametrize("fov", [70.0, 110.0])
def test_fast_camera_geometry_matches_verified_baseline_after_vad_resize(fov):
    baseline = scaled_intrinsics(1600, 900, fov, 640, 384)
    fast = scaled_intrinsics(640, 360, fov, 640, 384)
    assert fast == pytest.approx(baseline, rel=1e-12, abs=1e-12)


def test_fast_launch_loads_overrides_after_official_config_and_preserves_camera_order():
    root = parse_xml(FAST_VAD_LAUNCH)
    args = direct_args(root)
    assert args["use_fp16_heads"].get("default").lower() == "false"

    nodes = [node for node in root.iter("node") if node.get("name") == "vad_carla_tiny"]
    assert len(nodes) == 1
    node = nodes[0]
    parameter_files = [param for param in node.findall("./param") if param.get("from")]
    sources = [param.get("from") for param in parameter_files]
    official_index = next(i for i, source in enumerate(sources) if "vad_carla_tiny.param.yaml" in source)
    fast_index = sources.index("$(var fast_config)")
    model_override_index = sources.index("$(var model_override_file)")
    remapper_index = next(
        i for i, source in enumerate(sources) if "object_class_remapper_carla_tiny.param.yaml" in source
    )
    assert official_index < fast_index < model_override_index < remapper_index
    assert parameter_files[fast_index].get("allow_substs") == "true"
    assert parameter_files[model_override_index].get("allow_substs") == "true"

    assert args["model_override_file"].get("default").endswith(
        "/config/vad_carla_tiny_fast.param.yaml"
    )

    fast_config_lets = [
        element for element in root.findall("./let") if element.get("name") == "fast_config"
    ]
    assert len(fast_config_lets) == 2
    standard = next(element for element in fast_config_lets if element.get("unless"))
    experimental = next(element for element in fast_config_lets if element.get("if"))
    assert standard.get("unless") == "$(var use_fp16_heads)"
    assert standard.get("value").endswith("/config/vad_carla_tiny_fast.param.yaml")
    assert experimental.get("if") == "$(var use_fp16_heads)"
    assert experimental.get("value").endswith(
        "/config/vad_carla_tiny_fast_fp16_heads.param.yaml"
    )

    remaps = {remap.get("from"): remap.get("to") for remap in node.findall("./remap")}
    for index, camera in enumerate(CAMERAS):
        image_target = resolve_remap_target(root, remaps[f"~/input/image{index}"])
        info_target = resolve_remap_target(root, remaps[f"~/input/camera_info{index}"])
        assert image_target == f"/sensing/camera/{camera}/image_raw"
        assert info_target == f"/sensing/camera/{camera}/camera_info"


@pytest.mark.parametrize("launch_path", [MINIMAL_LAUNCH, FULL_LAUNCH])
def test_project_launches_keep_baseline_defaults_and_propagate_fast_profile(launch_path):
    root = parse_xml(launch_path)
    args = direct_args(root)
    for name in ("use_fast_vad", "vad_use_fp16_heads", "use_light_weight_sensor_mapping"):
        assert args[name].get("default").lower() == "false"
    assert args["vad_model_override_file"].get("default").endswith(
        "/config/vad_carla_tiny_fast.param.yaml"
    )
    if launch_path == FULL_LAUNCH:
        assert args["launch_fast_camera_view"].get("default").lower() == "false"
    else:
        assert "launch_fast_camera_view" not in args

    bridge = find_include(root, "autoware_carla_interface.launch.xml")
    assert include_args(bridge)["use_light_weight_sensor_mapping"] == (
        "$(var use_light_weight_sensor_mapping)"
    )
    assert include_args(bridge)["sensor_mapping_file"] == "$(var sensor_mapping_file)"

    baseline_vad = find_include(root, "vad_carla_tiny.launch.xml")
    fast_vad = find_include(root, "vad_carla_tiny_fast.launch.xml")
    assert ("unless", "$(var use_fast_vad)") in inherited_conditions(root, baseline_vad)
    assert ("if", "$(var use_fast_vad)") in inherited_conditions(root, fast_vad)
    assert include_args(fast_vad)["use_fp16_heads"] == "$(var vad_use_fp16_heads)"
    assert include_args(fast_vad)["model_override_file"] == "$(var vad_model_override_file)"

    if launch_path == FULL_LAUNCH:
        camera_view = next(
            node for node in root.iter("node") if node.get("name") == "vad_fast_front_camera"
        )
        assert camera_view.get("if") == "$(var launch_fast_camera_view)"
        assert ("if", "$(var rviz)") in inherited_conditions(root, camera_view)


@pytest.mark.parametrize("launch_path", [MINIMAL_LAUNCH, FULL_LAUNCH])
def test_route_postprocessing_is_opt_in_and_launch_parameters_are_wired(launch_path):
    root = parse_xml(launch_path)
    args = direct_args(root)
    expected_defaults = {
        "controller_stop_offset_m": "0.49",
        "comfortable_deceleration_mps2": "1.2",
        "maximum_speed_mps": "2.5",
        "turn_inward_corridor_half_width_m": "0.5",
        "turn_outward_corridor_half_width_m": "0.5",
        "left_turn_outward_corridor_half_width_m": "0.0",
        "right_turn_outward_corridor_half_width_m": "0.0",
        "route_corridor_mode": "hard",
        "route_corridor_entry_distance_m": "0.0",
        "trajectory_lateral_filter_gain": "1.0",
        "left_turn_trajectory_lateral_filter_gain": "0.0",
        "right_turn_trajectory_lateral_filter_gain": "0.0",
        "trajectory_lateral_filter_activation_threshold_m": "0.0",
        "trajectory_lateral_filter_anchor_m": "2.0",
        "trajectory_lateral_filter_timeout_sec": "4.0",
        "maximum_lateral_acceleration_mps2": "0.0",
        "curvature_speed_preview_m": "3.0",
    }
    for name, default in expected_defaults.items():
        assert args[name].get("default") == default

    manager = next(
        node for node in root.iter("node") if node.get("exec") == "vad_route_manager.py"
    )
    parameters = {
        parameter.get("name"): parameter.get("value")
        for parameter in manager.findall("./param")
        if parameter.get("name")
    }
    for name in expected_defaults:
        assert parameters[name] == f"$(var {name})"


def test_full_launch_standard_mpc_parameter_override_is_opt_in():
    root = parse_xml(FULL_LAUNCH)
    args = direct_args(root)
    autoware = find_include(root, "autoware.launch.xml")
    propagated = include_args(autoware)

    assert args["use_lateral_controller_param_override"].get("default") == "false"
    assert args["lateral_controller_param_path"].get("default") == ""
    assert propagated["use_lateral_controller_param_override"] == (
        "$(var use_lateral_controller_param_override)"
    )
    assert propagated["lateral_controller_param_path"] == (
        "$(var lateral_controller_param_path)"
    )
    assert args["use_longitudinal_controller_param_override"].get("default") == "false"
    assert args["longitudinal_controller_param_path"].get("default") == (
        "$(find-pkg-share autoware_e2e_vad_launch)/config/"
        "pid_carla_vad_no_steer_convergence.param.yaml"
    )
    assert propagated["use_longitudinal_controller_param_override"] == (
        "$(var use_longitudinal_controller_param_override)"
    )
    assert propagated["longitudinal_controller_param_path"] == (
        "$(var longitudinal_controller_param_path)"
    )
    stock_gate = (
        "$(find-pkg-share autoware_launch)/config/control/vehicle_cmd_gate/"
        "vehicle_cmd_gate.param.yaml"
    )
    assert args["vehicle_cmd_gate_param_path"].get("default") == stock_gate
    assert propagated["vehicle_cmd_gate_param_path"] == (
        "$(var vehicle_cmd_gate_param_path)"
    )


def test_carla_pid_profile_changes_only_the_stopped_steer_gate():
    stock = ros_parameters(STOCK_PID_PARAMS)
    carla = ros_parameters(CARLA_PID_PARAMS)
    gate = "enable_keep_stopped_until_steer_convergence"

    assert stock[gate] is True
    assert carla[gate] is False
    expected = deepcopy(stock)
    expected[gate] = False
    assert carla == expected


def test_full_shell_uses_standard_system_with_vad_rate_aware_topic_gates():
    root = parse_xml(FULL_LAUNCH)
    args = direct_args(root)
    autoware = find_include(root, "autoware.launch.xml")
    system = find_include(root, "system_vad_full_shell.launch.xml")

    assert include_args(autoware)["launch_system"] == "false"
    assert args["component_state_monitor_topic_path"].get("default").endswith(
        "/config/component_state_monitor_vad.yaml"
    )
    assert include_args(system)["component_state_monitor_topic_path"] == (
        "$(var component_state_monitor_topic_path)"
    )
    assert include_args(system)["diagnostic_graph_aggregator_graph_path"].endswith(
        "/config/diagnostic_graph_vad.yaml"
    )

    system_root = parse_xml(SYSTEM_VAD_LAUNCH)
    standard_system = find_include(system_root, "system.launch.xml")
    assert include_args(standard_system)["component_state_monitor_topic_path"] == (
        "$(var component_state_monitor_topic_path)"
    )
    assert include_args(standard_system)["diagnostic_graph_aggregator_graph_path"] == (
        "$(var diagnostic_graph_aggregator_graph_path)"
    )


def test_full_shell_vad_dummy_diagnostics_use_a_project_owned_launch_arg():
    root = parse_xml(FULL_LAUNCH)
    args = direct_args(root)
    vad_dummy = next(
        node for node in root.iter("node") if node.get("name") == "vad_dummy_diag_publisher"
    )

    assert args["launch_vad_dummy_diag_publisher"].get("default") == "true"
    assert "launch_dummy_diag_publisher" not in args
    assert vad_dummy.get("if") == "$(var launch_vad_dummy_diag_publisher)"


def test_vad_diagnostic_graph_only_widens_three_adapi_heartbeats():
    graph = load_yaml(VAD_DIAGNOSTIC_GRAPH)
    standard_units = []
    for filename in ("autoware-main.yaml", "localization.yaml", "planning.yaml"):
        standard_units.extend(load_yaml(STANDARD_DIAGNOSTIC_DIR / filename)["units"])

    expected_paths = {unit["path"]: unit for unit in standard_units}
    actual_paths = {unit["path"]: unit for unit in graph["units"]}
    assert actual_paths.keys() == expected_paths.keys()

    widened = {
        "/adapi/mrm_request/delegate",
        "/autoware/localization/state",
        "/autoware/planning/routing/state",
    }
    for path, expected in expected_paths.items():
        actual = actual_paths[path].copy()
        if path in widened:
            assert actual.pop("timeout") == pytest.approx(3.0)
        assert actual == expected

    assert graph["files"] == [
        {
            "path": "$(find-pkg-share autoware_launch)"
            f"/config/system/diagnostics/{name}.yaml"
        }
        for name in ("control", "map", "perception", "system", "vehicle")
    ]


def test_vad_component_monitor_only_relaxes_vad_owned_output_rates():
    standard = load_yaml(STANDARD_COMPONENT_MONITOR)
    vad = load_yaml(VAD_COMPONENT_MONITOR)
    assert len(vad) == len(standard)

    relaxed_topics = {
        "/perception/object_recognition/objects",
        "/planning/trajectory",
    }
    for standard_entry, vad_entry in zip(standard, vad):
        standard_args = standard_entry["args"]
        vad_args = vad_entry["args"]
        assert standard_entry.keys() == vad_entry.keys()
        assert standard_entry["module"] == vad_entry["module"]
        assert standard_entry["mode"] == vad_entry["mode"]
        assert standard_entry["type"] == vad_entry["type"]
        if standard_args.get("topic") in relaxed_topics:
            assert vad_args["warn_rate"] == pytest.approx(0.3)
            assert vad_args["error_rate"] == pytest.approx(0.15)
            assert vad_args["timeout"] == pytest.approx(4.0)
            unchanged = {"warn_rate", "error_rate", "timeout"}
            assert {
                key: value for key, value in standard_args.items() if key not in unchanged
            } == {key: value for key, value in vad_args.items() if key not in unchanged}
        else:
            assert vad_entry == standard_entry


def test_route_manager_watchdog_allows_two_verified_vad_intervals():
    params = load_yaml(VAD_ROUTE_MANAGER_PARAMS)["/**"]["ros__parameters"]

    assert params["candidate_timeout_sec"] == pytest.approx(6.0)
    assert params["candidate_timeout_sec"] > 2.0 / 0.4


def test_light_weight_bridge_condition_disables_jpeg_relays_and_combiner():
    root = parse_xml(CARLA_BRIDGE_LAUNCH)
    helpers = []
    for node in root.iter("node"):
        executable = node.get("exec", "")
        name = node.get("name", "")
        if executable == "republish" or executable == "multi_camera_combiner":
            helpers.append(node)
        elif executable == "relay" and name.startswith("traffic_light_"):
            helpers.append(node)

    assert len(helpers) == 10
    assert all(node.get("unless") == "$(var use_light_weight_sensor_mapping)" for node in helpers)


def test_carla_camera_blueprint_fast_options_are_optional(monkeypatch):
    source_path = str(CARLA_INTERFACE / "src")
    monkeypatch.syspath_prepend(source_path)
    for module_name in list(sys.modules):
        if module_name == "autoware_carla_interface" or module_name.startswith(
            "autoware_carla_interface."
        ):
            del sys.modules[module_name]

    try:
        from autoware_carla_interface.modules.carla_wrapper import SensorWrapper
    except ImportError as error:
        pytest.fail(f"CARLA Python environment is required for the bridge unit test: {error}")

    class Blueprint:
        def __init__(self):
            self.calls = []

        def set_attribute(self, name, value):
            self.calls.append((name, value))

    wrapper = SensorWrapper.__new__(SensorWrapper)
    baseline = Blueprint()
    wrapper._configure_camera_attributes(
        baseline,
        {"image_size_x": 1600, "image_size_y": 900, "fov": 70.0},
    )
    assert baseline.calls == [
        ("image_size_x", "1600"),
        ("image_size_y", "900"),
        ("fov", "70.0"),
    ]

    fast = Blueprint()
    wrapper._configure_camera_attributes(
        fast,
        {
            "image_size_x": 640,
            "image_size_y": 360,
            "fov": 70.0,
            "sensor_tick": 0.2,
            "enable_postprocess_effects": False,
        },
    )
    assert fast.calls == [
        ("image_size_x", "640"),
        ("image_size_y", "360"),
        ("fov", "70.0"),
        ("sensor_tick", "0.2"),
        ("enable_postprocess_effects", "false"),
    ]


def make_fake_runtime(tmp_path):
    cuda_root = tmp_path / "cuda"
    bin_dir = cuda_root / "bin"
    bin_dir.mkdir(parents=True)
    nvcc = bin_dir / "nvcc"
    nvcc.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
    nvcc.chmod(0o755)
    prefix = tmp_path / "install"
    package_share = prefix / "share/autoware_e2e_vad_launch"
    (package_share / "config").mkdir(parents=True)
    (package_share / "launch").mkdir(parents=True)
    shutil.copy2(FAST_MAPPING, package_share / "config" / FAST_MAPPING.name)
    shutil.copy2(FAST_VAD_PARAMS, package_share / "config" / FAST_VAD_PARAMS.name)
    shutil.copy2(
        RELIABLE_FAST_MAPPING, package_share / "config" / RELIABLE_FAST_MAPPING.name
    )
    shutil.copy2(
        RELIABLE_SYNC_QUEUE32_PARAMS,
        package_share / "config" / RELIABLE_SYNC_QUEUE32_PARAMS.name,
    )
    shutil.copy2(
        RECOMMENDED_MPC_PARAMS,
        package_share / "config" / RECOMMENDED_MPC_PARAMS.name,
    )
    shutil.copy2(FAST_VAD_LAUNCH, package_share / "launch" / FAST_VAD_LAUNCH.name)

    ros2 = bin_dir / "ros2"
    ros2.write_text(
        "#!/usr/bin/env bash\n"
        "if [[ \"$1\" == pkg && \"$2\" == prefix && \"$3\" == autoware_e2e_vad_launch ]]; then\n"
        f"  printf '%s\\n' {prefix!s}\n"
        "  exit 0\n"
        "fi\n"
        "printf '%s\\n' \"$@\"\n",
        encoding="utf-8",
    )
    ros2.chmod(0o755)
    return cuda_root


def make_route(tmp_path):
    route = json.loads(
        (PACKAGE / "test/fixtures/route_map/town99_route.json").read_text(encoding="utf-8")
    )
    route["spawn_point"] = "1.0,0.0,0.5,0.0,0.0,0.0"
    path = tmp_path / "route.json"
    path.write_text(json.dumps(route), encoding="utf-8")
    return path


def wrapper_environment(tmp_path):
    environment = os.environ.copy()
    environment.update(
        {
            "AUTOWARE_E2E_CUDA_ROOT": str(make_fake_runtime(tmp_path)),
            "AUTOWARE_E2E_SKIP_INSTALL": "1",
            "AUTOWARE_E2E_FULL_MAP_PATH": str(
                PACKAGE / "test/fixtures/route_map/Town99_full"
            ),
        }
    )
    return environment


@pytest.mark.parametrize(
    "protected",
    [
        "use_fast_vad:=false",
        "vad_use_fp16_heads:=true",
        "vad_model_override_file:=/tmp/not-allowed.yaml",
        "use_light_weight_sensor_mapping:=False",
        "sensor_mapping_file:=/tmp/not-fast.yaml",
        "rviz:=true",
        "launch_fast_camera_view:=true",
    ],
)
def test_fast_wrapper_rejects_protected_launch_arguments(tmp_path, protected):
    route = make_route(tmp_path)
    completed = subprocess.run(
        [str(FAST_WRAPPER), str(route), protected],
        cwd=ROOT,
        env=wrapper_environment(tmp_path),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    assert completed.returncode == 2
    assert any(
        marker in completed.stderr.lower() for marker in ("protected", "override", "controlled")
    )


def test_fast_wrapper_rejects_visualization_without_full_stack(tmp_path):
    route = make_route(tmp_path)
    completed = subprocess.run(
        [str(FAST_WRAPPER), "--visualize", str(route)],
        cwd=ROOT,
        env=wrapper_environment(tmp_path),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    assert completed.returncode == 2
    assert "--full" in completed.stderr


def test_fast_wrapper_builds_minimal_raw_profile_command(tmp_path):
    route = make_route(tmp_path)
    completed = subprocess.run(
        [str(FAST_WRAPPER), str(route), "vad_log_level:=warn"],
        cwd=ROOT,
        env=wrapper_environment(tmp_path),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr
    arguments = completed.stdout.splitlines()
    assert "carla_vad.launch.xml" in arguments
    assert "use_fast_vad:=true" in arguments
    assert "vad_use_fp16_heads:=false" in arguments
    assert "use_light_weight_sensor_mapping:=True" in arguments
    installed_mapping = tmp_path / "install/share/autoware_e2e_vad_launch/config" / FAST_MAPPING.name
    installed_params = tmp_path / "install/share/autoware_e2e_vad_launch/config" / FAST_VAD_PARAMS.name
    assert f"sensor_mapping_file:={installed_mapping}" in arguments
    assert f"vad_model_override_file:={installed_params}" in arguments
    assert "vad_log_level:=warn" in arguments
    assert not any(argument.startswith("rviz:=") for argument in arguments)
    assert not any(argument.startswith("launch_fast_camera_view:=") for argument in arguments)


def test_fast_wrapper_builds_full_visual_fp16_profile_command(tmp_path):
    route = make_route(tmp_path)
    completed = subprocess.run(
        [str(FAST_WRAPPER), "--full", "--visualize", "--fp16-heads", str(route)],
        cwd=ROOT,
        env=wrapper_environment(tmp_path),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr
    arguments = completed.stdout.splitlines()
    assert "carla_vad_full.launch.xml" in arguments
    assert "use_fast_vad:=true" in arguments
    assert "vad_use_fp16_heads:=true" in arguments
    assert "use_light_weight_sensor_mapping:=True" in arguments
    installed_mapping = tmp_path / "install/share/autoware_e2e_vad_launch/config" / FAST_MAPPING.name
    installed_params = tmp_path / "install/share/autoware_e2e_vad_launch/config" / FAST_VAD_PARAMS.name
    assert f"sensor_mapping_file:={installed_mapping}" in arguments
    assert f"vad_model_override_file:={installed_params}" in arguments
    assert "rviz:=true" in arguments
    assert "launch_fast_camera_view:=true" in arguments


def test_fast_wrapper_builds_validated_recommended_profile(tmp_path):
    route = make_route(tmp_path)
    completed = subprocess.run(
        [str(FAST_WRAPPER), "--recommended", "--visualize", str(route)],
        cwd=ROOT,
        env=wrapper_environment(tmp_path),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr
    arguments = completed.stdout.splitlines()
    package_config = tmp_path / "install/share/autoware_e2e_vad_launch/config"
    assert "carla_vad_full.launch.xml" in arguments
    assert f"sensor_mapping_file:={package_config / RELIABLE_FAST_MAPPING.name}" in arguments
    assert (
        f"vad_model_override_file:={package_config / RELIABLE_SYNC_QUEUE32_PARAMS.name}"
        in arguments
    )
    assert f"lateral_controller_param_path:={package_config / RECOMMENDED_MPC_PARAMS.name}" in arguments
    assert "use_lateral_controller_param_override:=true" in arguments
    assert "use_longitudinal_controller_param_override:=true" in arguments
    assert "controller_stop_offset_m:=0.60" in arguments
    assert "comfortable_deceleration_mps2:=0.60" in arguments
    assert "maneuver_lookahead_m:=3.0" in arguments
    assert "turn_inward_corridor_half_width_m:=0.20" in arguments
    assert "trajectory_geometry_smoothing_strength:=10.0" in arguments
    assert "maximum_speed_mps:=2.5" in arguments
    assert "trajectory_lateral_filter_gain:=0.75" not in arguments
    assert "right_turn_outward_corridor_half_width_m:=0.30" not in arguments
    assert "right_turn_trajectory_lateral_filter_gain:=0.75" not in arguments
    assert "rviz:=true" in arguments
    assert "launch_fast_camera_view:=true" in arguments


def test_fast_wrapper_adds_trajectory_stability_candidate(tmp_path):
    route = make_route(tmp_path)
    completed = subprocess.run(
        [str(FAST_WRAPPER), "--trajectory-stability", str(route)],
        cwd=ROOT,
        env=wrapper_environment(tmp_path),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr
    arguments = completed.stdout.splitlines()
    assert "carla_vad_full.launch.xml" in arguments
    assert "right_turn_trajectory_lateral_filter_gain:=0.75" in arguments
    assert "trajectory_lateral_filter_activation_threshold_m:=0.20" in arguments
    assert "left_turn_trajectory_lateral_filter_gain:=0.75" not in arguments
    assert "right_turn_outward_corridor_half_width_m:=0.30" not in arguments
    assert "maneuver_exit_lookahead_m:=0.5" not in arguments
    assert "turn_outward_corridor_half_width_m:=0.30" not in arguments
    assert "route_corridor_entry_distance_m:=0.0" not in arguments


def test_fast_wrapper_adds_tight_corridor_candidate(tmp_path):
    route = make_route(tmp_path)
    completed = subprocess.run(
        [str(FAST_WRAPPER), "--recommended", "--tight-corridor", str(route)],
        cwd=ROOT,
        env=wrapper_environment(tmp_path),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr
    arguments = completed.stdout.splitlines()
    assert "route_corridor_half_width_m:=0.20" in arguments
    assert "turn_inward_corridor_half_width_m:=0.20" in arguments
    assert "turn_outward_corridor_half_width_m:=0.20" in arguments
    assert "trajectory_lateral_filter_gain:=0.75" not in arguments
    assert "maximum_lateral_acceleration_mps2:=1.2" not in arguments


@pytest.mark.parametrize(
    "conflict",
    [
        ("--fp16-heads",),
        ("--model-override", "custom_model"),
        ("--sensor-mapping", "custom_mapping"),
    ],
)
def test_fast_wrapper_rejects_recommended_deployment_conflicts(tmp_path, conflict):
    route = make_route(tmp_path)
    command = [str(FAST_WRAPPER), "--recommended"]
    if len(conflict) == 1:
        command.extend(conflict)
    else:
        custom = tmp_path / conflict[1]
        custom.write_text("/**:\n  ros__parameters: {}\n", encoding="utf-8")
        command.extend((conflict[0], str(custom)))
    command.append(str(route))
    completed = subprocess.run(
        command,
        cwd=ROOT,
        env=wrapper_environment(tmp_path),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    assert completed.returncode == 2
    assert "--recommended controls" in completed.stderr


def test_fast_wrapper_rejects_recommended_launch_override(tmp_path):
    route = make_route(tmp_path)
    completed = subprocess.run(
        [
            str(FAST_WRAPPER),
            "--recommended",
            str(route),
            "turn_inward_corridor_half_width_m:=0.5",
        ],
        cwd=ROOT,
        env=wrapper_environment(tmp_path),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    assert completed.returncode == 2
    assert "controlled by this wrapper" in completed.stderr


def test_fast_wrapper_passes_custom_model_override_last(tmp_path):
    route = make_route(tmp_path)
    custom = tmp_path / "custom-vad.param.yaml"
    custom.write_text(
        "/**:\n  ros__parameters:\n    model_params:\n      object_confidence_thresholds: "
        "[0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.4]\n",
        encoding="utf-8",
    )
    completed = subprocess.run(
        [str(FAST_WRAPPER), "--model-override", str(custom), str(route)],
        cwd=ROOT,
        env=wrapper_environment(tmp_path),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr
    arguments = completed.stdout.splitlines()
    assert f"vad_model_override_file:={custom.resolve()}" in arguments
    assert "vad_use_fp16_heads:=false" in arguments


@pytest.mark.parametrize("deployment_option", ["fp16", "model_override"])
def test_fast_wrapper_combines_custom_sensor_mapping_with_deployment_options(
    tmp_path, deployment_option
):
    route = make_route(tmp_path)
    mapping = tmp_path / "custom-sensor-mapping.yaml"
    shutil.copy2(RELIABLE_FAST_MAPPING, mapping)
    command = [str(FAST_WRAPPER)]
    custom_model = tmp_path / "custom-vad.param.yaml"
    if deployment_option == "fp16":
        command.append("--fp16-heads")
    else:
        custom_model.write_text("/**:\n  ros__parameters: {}\n", encoding="utf-8")
        command.extend(("--model-override", str(custom_model)))
    command.extend(("--sensor-mapping", str(mapping), str(route)))

    completed = subprocess.run(
        command,
        cwd=ROOT,
        env=wrapper_environment(tmp_path),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr
    arguments = completed.stdout.splitlines()
    assert f"sensor_mapping_file:={mapping.resolve()}" in arguments
    assert (
        "vad_use_fp16_heads:=true" in arguments
        if deployment_option == "fp16"
        else f"vad_model_override_file:={custom_model.resolve()}" in arguments
    )


def test_fast_wrapper_rejects_missing_custom_sensor_mapping(tmp_path):
    route = make_route(tmp_path)
    missing = tmp_path / "missing-sensor-mapping.yaml"

    completed = subprocess.run(
        [str(FAST_WRAPPER), "--sensor-mapping", str(missing), str(route)],
        cwd=ROOT,
        env=wrapper_environment(tmp_path),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )

    assert completed.returncode == 2
    assert "not found" in completed.stderr.lower()


def test_fast_wrapper_builds_complete_reliable_publisher_and_reader_profile(tmp_path):
    route = make_route(tmp_path)
    completed = subprocess.run(
        [
            str(FAST_WRAPPER),
            "--model-override",
            str(RELIABLE_SYNC_QUEUE32_PARAMS),
            "--sensor-mapping",
            str(RELIABLE_FAST_MAPPING),
            str(route),
        ],
        cwd=ROOT,
        env=wrapper_environment(tmp_path),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr
    arguments = completed.stdout.splitlines()
    assert f"sensor_mapping_file:={RELIABLE_FAST_MAPPING.resolve()}" in arguments
    assert f"vad_model_override_file:={RELIABLE_SYNC_QUEUE32_PARAMS.resolve()}" in arguments


def test_fast_wrapper_rejects_fp16_with_custom_model_override(tmp_path):
    route = make_route(tmp_path)
    custom = tmp_path / "custom-vad.param.yaml"
    custom.write_text("/**:\n  ros__parameters: {}\n", encoding="utf-8")
    completed = subprocess.run(
        [
            str(FAST_WRAPPER),
            "--fp16-heads",
            "--model-override",
            str(custom),
            str(route),
        ],
        cwd=ROOT,
        env=wrapper_environment(tmp_path),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    assert completed.returncode == 2
    assert "either" in completed.stderr.lower()


def test_recorded_wrapper_rejects_fp16_with_custom_model_override_before_runtime(tmp_path):
    route = make_route(tmp_path)
    custom = tmp_path / "custom-vad.param.yaml"
    custom.write_text("/**:\n  ros__parameters: {}\n", encoding="utf-8")

    completed = subprocess.run(
        [
            str(RECORDED_WRAPPER),
            "--fp16-heads",
            "--model-override",
            str(custom),
            str(tmp_path / "artifact"),
            str(route),
        ],
        cwd=ROOT,
        env=wrapper_environment(tmp_path),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )

    assert completed.returncode == 2
    assert "mutually exclusive" in completed.stderr.lower()
    assert not (tmp_path / "artifact").exists()


def test_recorded_wrapper_rejects_direct_sensor_mapping_launch_argument(tmp_path):
    route = make_route(tmp_path)
    completed = subprocess.run(
        [
            str(RECORDED_WRAPPER),
            "--sensor-mapping",
            str(RELIABLE_FAST_MAPPING),
            str(tmp_path / "artifact"),
            str(route),
            "sensor_mapping_file:=/tmp/not-allowed.yaml",
        ],
        cwd=ROOT,
        env=wrapper_environment(tmp_path),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )

    assert completed.returncode == 2
    assert "protected wrapper option" in completed.stderr
    assert not (tmp_path / "artifact").exists()


def test_recorded_wrapper_preserves_sensor_mapping_provenance_and_passes_option():
    wrapper = RECORDED_WRAPPER.read_text(encoding="utf-8")

    assert "SENSOR_MAPPING_FILE=%s\\nSENSOR_MAPPING_SHA256=%s\\n" in wrapper
    assert '"${output_dir}/sensor_mapping_provenance/sensor_mapping.yaml"' in wrapper
    assert '"${output_dir}/sensor_mapping_provenance/SHA256SUMS"' in wrapper
    assert 'stack_command+=(--sensor-mapping "${sensor_mapping}")' in wrapper


def test_smart_mpc_wrapper_forwards_protected_sensor_mapping_option_before_route():
    wrapper = SMART_WRAPPER.read_text(encoding="utf-8")

    option = wrapper.index('sensor_mapping_options=(--sensor-mapping "$2")')
    forwarding = wrapper.index('"${sensor_mapping_options[@]}"')
    route = wrapper.index('"${route_file}" \\', forwarding)
    assert option < forwarding < route
