from importlib.util import module_from_spec
from importlib.util import spec_from_file_location
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parent
LOGIC_PATH = PACKAGE / "scripts/carla_actuation_sweep_logic.py"
LAUNCH_PATH = PACKAGE / "launch/carla_actuation_calibration.launch.xml"
SENSOR_MAPPING_PATH = PACKAGE / "config/sensor_mapping_actuation_calibration.yaml"
PARAM_PATH = PACKAGE / "config/carla_actuation_sweep.param.yaml"
WRAPPER_PATH = WORKSPACE / "scripts/e2e/run_carla_actuation_calibration.sh"
FULL_LAUNCH_PATH = PACKAGE / "launch/carla_vad_full.launch.xml"
RECORDED_WRAPPER_PATH = WORKSPACE / "scripts/e2e/run_recorded_route_trial.sh"


def load_logic():
    spec = spec_from_file_location("carla_actuation_sweep_logic", LOGIC_PATH)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_sweep_plan_is_deterministic_and_complete():
    logic = load_logic()
    cases = logic.build_sweep_cases([0.1, 0.2], [0.1, 0.3], 2)
    assert [case.case_id for case in cases] == [
        "r01_coast",
        "r01_accel_0.100",
        "r01_accel_0.200",
        "r01_brake_0.100",
        "r01_brake_0.300",
        "r02_coast",
        "r02_accel_0.100",
        "r02_accel_0.200",
        "r02_brake_0.100",
        "r02_brake_0.300",
    ]


def test_fit_eligibility_rejects_watchdog_and_command_mismatch():
    logic = load_logic()
    base = {
        "phase": "sample",
        "sample_elapsed_s": 0.5,
        "response_settle_s": 0.35,
        "requested_accel": 0.1,
        "requested_brake": 0.0,
        "applied_accel": 0.1,
        "applied_brake": 0.0,
        "command_tolerance": 0.025,
        "velocity_mps": 1.0,
        "minimum_velocity_mps": 0.1,
        "maximum_velocity_mps": 3.0,
        "steering_rad": 0.0,
        "maximum_steering_rad": 0.12,
        "status_age_wall_s": 0.01,
        "status_timeout_wall_s": 0.5,
        "watchdog_active": False,
        "safety_ok": True,
    }
    assert logic.sample_is_fit_eligible(**base)
    assert not logic.sample_is_fit_eligible(**{**base, "watchdog_active": True})
    assert not logic.sample_is_fit_eligible(**{**base, "applied_brake": 0.1})
    assert not logic.sample_is_fit_eligible(**{**base, "sample_elapsed_s": 0.2})


def test_launch_is_interface_only_and_requires_new_outputs():
    root = ET.parse(LAUNCH_PATH).getroot()
    nodes = root.findall("node")
    executables = {(node.get("pkg"), node.get("exec")) for node in nodes}
    assert executables == {
        ("autoware_carla_interface", "autoware_carla_interface"),
        ("autoware_e2e_vad_launch", "carla_actuation_sweep.py"),
    }
    assert root.findall("include") == []
    assert "autoware_raw_vehicle_cmd_converter" not in LAUNCH_PATH.read_text()
    args = {argument.get("name"): argument for argument in root.findall("arg")}
    assert args["output_csv"].get("default") is None
    assert args["summary_json"].get("default") is None
    assert args["carla_port"].get("default") == "2100"
    assert args["ego_vehicle_role_name"].get("default") == "autoware_e2e_calibration"


def test_calibration_sensor_mapping_has_only_imu_and_gnss():
    mapping = yaml.safe_load(SENSOR_MAPPING_PATH.read_text())
    assert mapping["enabled_sensors"] == ["tamagawa/imu_link", "gnss_link"]
    sensor_types = {
        item["carla_type"] for item in mapping["sensor_mappings"].values()
    }
    assert sensor_types == {"sensor.other.imu", "sensor.other.gnss"}


def test_profile_has_watchdog_and_safe_limits():
    params = yaml.safe_load(PARAM_PATH.read_text())["/**"]["ros__parameters"]
    assert params["status_timeout_wall_s"] <= 0.5
    assert params["maximum_velocity_mps"] > params["sample_max_velocity_mps"]
    assert params["maximum_displacement_m"] > params["maximum_sample_displacement_m"]
    assert params["accel_levels"][-1] <= 0.4
    assert params["brake_levels"][-1] <= 0.8
    assert params["shutdown_brake_wall_s"] >= 1.0


def test_wrapper_owns_only_its_process_group_and_actor_role():
    wrapper = WRAPPER_PATH.read_text()
    assert "setsid ros2 launch" in wrapper
    assert 'kill -INT -- "-${pid}"' in wrapper
    assert "pkill" not in wrapper
    assert "run_carla.sh" not in wrapper
    assert 'role_name\", \"\") == \"autoware_e2e_calibration\"' in wrapper
    assert "actor_guard cleanup" in wrapper
    assert "/calibration/actuation/emergency_stop" in wrapper
    assert 'dynamic_actors.extend(world.get_actors().filter(pattern))' in wrapper
    assert 'if [[ "${sweep_status}" == "completed" ]]' in wrapper
    assert '--sweep-summary "${sweep_summary}"' in wrapper
    assert 'analysis_args+=(--candidate-dir "${output_dir}/candidate")' in wrapper
    assert 'settings.synchronous_mode = original["synchronous_mode"]' in wrapper
    assert 'settings.fixed_delta_seconds = original["fixed_delta_seconds"]' in wrapper


def test_full_launch_exposes_stock_default_converter_config_and_passes_it_through():
    root = ET.parse(FULL_LAUNCH_PATH).getroot()
    arguments = {argument.get("name"): argument for argument in root.findall("arg")}
    stock = "$(find-pkg-share autoware_carla_interface)/config/raw_vehicle_cmd_converter.param.yaml"
    assert arguments["raw_vehicle_cmd_converter_config"].get("default") == stock
    interface_include = next(
        include
        for include in root.iter("include")
        if "autoware_carla_interface.launch.xml" in include.get("file", "")
    )
    passed = {argument.get("name"): argument.get("value") for argument in interface_include}
    assert passed["raw_vehicle_cmd_converter_config"] == "$(var raw_vehicle_cmd_converter_config)"


def test_recorded_trial_copies_selected_converter_config_for_provenance():
    wrapper = RECORDED_WRAPPER_PATH.read_text()
    assert "raw_vehicle_cmd_converter_config:=*" in wrapper
    assert 'raw_vehicle_cmd_converter_config="${argument#*:=}"' in wrapper
    assert "capture_raw_vehicle_cmd_converter_provenance.py" in wrapper
    assert '"${output_dir}/actuation_config_provenance"' in wrapper
    assert "RAW_VEHICLE_CMD_CONVERTER_CONFIG=%s" in wrapper


def test_recorded_trial_passes_model_override_through_fast_wrapper_with_provenance():
    wrapper = RECORDED_WRAPPER_PATH.read_text()

    assert "--model-override)" in wrapper
    assert '--model-override "${model_override}"' in wrapper
    assert "--fp16-heads and --model-override are mutually exclusive" in wrapper
    assert "vad_model_override_file:=*" in wrapper
    assert "VAD_MODEL_OVERRIDE_SHA256=%s" in wrapper
    assert '"${output_dir}/vad_model_override_provenance/model_override.param.yaml"' in wrapper
