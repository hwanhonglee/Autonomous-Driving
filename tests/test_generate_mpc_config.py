import importlib.util
import json
from pathlib import Path

import pytest
import yaml


MODULE_PATH = Path(__file__).parents[1] / "scripts/e2e/generate_mpc_config.py"
SPEC = importlib.util.spec_from_file_location("generate_mpc_config", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_generate_changes_only_timing_values(tmp_path):
    output = tmp_path / "mpc.yaml"
    metadata = MODULE.generate(MODULE.DEFAULT_BASE, output, 0.09, 0.15, 0.03)
    base = yaml.safe_load(MODULE.DEFAULT_BASE.read_text(encoding="utf-8"))
    generated = yaml.safe_load(output.read_text(encoding="utf-8"))
    base_parameters = base["/**"]["ros__parameters"]
    generated_parameters = generated["/**"]["ros__parameters"]

    assert generated_parameters["input_delay"] == pytest.approx(0.09)
    assert generated_parameters["vehicle_model_steer_tau"] == pytest.approx(0.15)
    generated_parameters["input_delay"] = base_parameters["input_delay"]
    generated_parameters["vehicle_model_steer_tau"] = base_parameters[
        "vehicle_model_steer_tau"
    ]
    assert generated == base
    assert metadata["effective_input_delay_sec"] == pytest.approx(0.09)
    assert json.loads(
        output.with_suffix(".yaml.metadata.json").read_text(encoding="utf-8")
    )["vehicle_model_steer_tau_sec"] == pytest.approx(0.15)


def test_generate_applies_explicit_smoothing_overrides(tmp_path):
    output = tmp_path / "mpc_smooth.yaml"
    metadata = MODULE.generate(
        MODULE.DEFAULT_BASE,
        output,
        0.12,
        0.15,
        0.03,
        steering_input_weight=1.5,
        heading_error_squared_vel_weight=0.2,
        steer_rate_weight=0.0,
        steer_acc_weight=4.0e-6,
        steering_lpf_cutoff_hz=2.0,
        steer_rate_limit_dps=40.0,
    )
    generated = yaml.safe_load(output.read_text(encoding="utf-8"))
    parameters = generated["/**"]["ros__parameters"]

    assert parameters["mpc_weight_steering_input"] == pytest.approx(1.5)
    assert parameters["mpc_low_curvature_weight_steering_input"] == pytest.approx(1.5)
    assert parameters["mpc_weight_heading_error_squared_vel"] == pytest.approx(0.2)
    assert parameters["mpc_low_curvature_weight_heading_error_squared_vel"] == pytest.approx(
        0.2
    )
    assert parameters["mpc_weight_steer_rate"] == pytest.approx(0.0)
    assert parameters["mpc_low_curvature_weight_steer_rate"] == pytest.approx(0.0)
    assert parameters["mpc_weight_steer_acc"] == pytest.approx(4.0e-6)
    assert parameters["mpc_low_curvature_weight_steer_acc"] == pytest.approx(4.0e-6)
    assert parameters["steering_lpf_cutoff_hz"] == pytest.approx(2.0)
    assert parameters["steer_rate_lim_dps_list_by_curvature"] == [40.0] * 3
    assert parameters["steer_rate_lim_dps_list_by_velocity"] == [40.0] * 3
    assert metadata["parameter_overrides"]["mpc_weight_steering_input"] == pytest.approx(
        1.5
    )


@pytest.mark.parametrize(
    "overrides",
    [
        {"steering_input_weight": -1.0},
        {"heading_error_squared_vel_weight": -0.1},
        {"steer_rate_weight": float("nan")},
        {"steer_acc_weight": -1.0e-6},
        {"steering_lpf_cutoff_hz": 0.0},
        {"steer_rate_limit_dps": float("inf")},
    ],
)
def test_generate_rejects_invalid_smoothing_overrides(tmp_path, overrides):
    with pytest.raises(ValueError):
        MODULE.generate(
            MODULE.DEFAULT_BASE,
            tmp_path / "mpc.yaml",
            0.12,
            0.15,
            0.03,
            **overrides,
        )


@pytest.mark.parametrize("input_delay", [0.1, -0.03, float("nan")])
def test_generate_rejects_ambiguous_or_invalid_delay(tmp_path, input_delay):
    with pytest.raises(ValueError):
        MODULE.generate(MODULE.DEFAULT_BASE, tmp_path / "mpc.yaml", input_delay, 0.15, 0.03)
