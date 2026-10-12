"""HH_261001 - Test the independent raw v2 model, not vehicle or training approval."""

from __future__ import annotations

import copy
from dataclasses import replace
import json

import pytest

torch = pytest.importorskip("torch")

from portable_e2e.contract import ContractError
from portable_e2e.temporal_model_v2 import (
    MODEL_ID_V2,
    NOMINAL_TIME_OFFSETS_S,
    TemporalModelConfigV2,
    TemporalModelV2,
    parameter_count,
)


@pytest.fixture(scope="module", autouse=True)
def _one_cpu_thread():
    previous = torch.get_num_threads()
    torch.set_num_threads(1)
    yield
    torch.set_num_threads(previous)


@pytest.fixture(scope="module")
def model():
    # HH_261001 - Smaller internal widths preserve the exact production input and token ABI.
    torch.manual_seed(17)
    result = TemporalModelV2(TemporalModelConfigV2(encoder_base_channels=4, hidden_width=128))
    return result.eval()


def _inputs(batch=1):
    generator = torch.Generator().manual_seed(29)
    calibration = torch.zeros(batch, 6, 16)
    calibration[..., :4] = torch.tensor((0.7, 1.2, 0.5, 0.5))
    calibration[..., (4, 9, 14)] = 1.0
    pose = torch.zeros(batch, 5, 3)
    pose[..., 0] = torch.tensor((-1.0, -0.75, -0.5, -0.25, 0.0))
    pose[..., 2] = torch.tensor((0.04, 0.03, 0.02, 0.01, 0.0))
    route = torch.zeros(batch, 128, 2)
    route[..., 0] = torch.arange(128)
    return {
        "images": torch.rand(batch, 5, 6, 3, 180, 320, generator=generator),
        "calibration": calibration,
        "ego_history": torch.zeros(batch, 10, 13),
        "ego_history_mask": torch.ones(batch, 10, dtype=torch.bool),
        "route_xy": route,
        "route_mask": torch.ones(batch, 128, dtype=torch.bool),
        "time_valid": torch.ones(batch, 5, dtype=torch.bool),
        "relative_pose": pose,
        "time_offsets_s": torch.tensor(NOMINAL_TIME_OFFSETS_S).expand(batch, -1).clone(),
    }


def _maximum_difference(a, b):
    return max(float((a[key] - b[key]).abs().max()) for key in a)


def test_default_config_json_roundtrip_has_an_independent_identifier():
    config = TemporalModelConfigV2()
    restored = TemporalModelConfigV2.from_mapping(json.loads(json.dumps(config.to_dict(), sort_keys=True)))
    assert restored == config
    assert config.model_id == MODEL_ID_V2
    assert "research" in MODEL_ID_V2
    assert config.time_offsets_s == NOMINAL_TIME_OFFSETS_S


@pytest.mark.parametrize("change", (
    {"camera_count": 5}, {"image_history_frames": 1}, {"image_height": 16},
    {"image_grid_height": 3}, {"token_width": 64}, {"query_count": 16},
    {"cross_attention_layers": 1}, {"spatial_knots": 64},
    {"camera_count": True}, {"encoder_base_channels": True},
    {"hidden_width": 32}, {"route_scale_m": float("nan")},
    {"maximum_curvature_rad_per_m": 0.3}, {"model_id": "existing.v1"},
    {"time_offsets_s": (-0.8, -0.6, -0.4, -0.2, 0.1)},
    {"time_offsets_s": [-0.8, -0.6, -0.4, -0.2, 0.0]},
    {"time_offsets_s": (-0.8, -0.6, False, -0.2, 0.0)},
    {"relative_pose_convention": "world_pose"}, {"image_normalization": "unscaled_bytes"},
))
def test_config_rejects_shape_and_semantic_drift(change):
    with pytest.raises(ContractError):
        replace(TemporalModelConfigV2(), **change).validate()


def test_config_rejects_missing_unknown_and_non_mapping_fields():
    payload = TemporalModelConfigV2().to_dict()
    del payload["image_width"]
    with pytest.raises(ContractError, match="missing"):
        TemporalModelConfigV2.from_mapping(payload)
    payload = TemporalModelConfigV2().to_dict()
    payload["candidate_count"] = 12
    with pytest.raises(ContractError, match="unknown"):
        TemporalModelConfigV2.from_mapping(payload)
    with pytest.raises(ContractError, match="object"):
        TemporalModelConfigV2.from_mapping([])
    with pytest.raises(ContractError, match="field names"):
        TemporalModelConfigV2.from_mapping({1: "wrong"})


def test_default_parameter_count_is_in_proposed_capacity_band():
    default = TemporalModelV2()
    assert 2_000_000 <= parameter_count(default) <= 5_000_000


def test_production_input_output_shapes_and_finite_gradients(model):
    inputs = _inputs(batch=2)
    model.zero_grad(set_to_none=True)
    output = model(**inputs)
    assert {key: tuple(value.shape) for key, value in output.items()} == {
        "curvature_knots": (2, 33), "preferred_speed_knots": (2, 33),
        "intent_logits": (2, 3), "stop_distance_raw": (2, 1),
        "acceleration_prediction": (2, 1),
    }
    assert all(torch.isfinite(value).all() for value in output.values())
    assert (output["curvature_knots"].abs() <= 0.2).all()
    assert (output["preferred_speed_knots"] > 0.0).all()
    sum(value.square().mean() + value.mean() for value in output.values()).backward()
    assert all(parameter.grad is not None and torch.isfinite(parameter.grad).all() for parameter in model.parameters())
    assert model.image_encoder.features[0].weight.grad.abs().sum() > 0
    assert model.speed_head.weight.grad.abs().sum() > 0
    assert model.pose_time_embedding[0].weight.grad.abs().sum() > 0


def test_negative_raw_speed_retains_gradient_not_hard_clamped(model):
    independent = copy.deepcopy(model)
    with torch.no_grad():
        independent.speed_head.weight.zero_()
        independent.speed_head.bias.fill_(-5.0)
    output = independent(**_inputs())
    output["preferred_speed_knots"].sum().backward()
    assert (independent.speed_head.bias.grad > 0).all()


@pytest.mark.parametrize("name,shape", (
    ("images", (1, 6, 3, 180, 320)), ("calibration", (1, 6, 15)),
    ("ego_history", (1, 9, 13)), ("route_xy", (1, 127, 2)),
    ("time_valid", (1, 4)), ("relative_pose", (1, 5, 2)),
    ("time_offsets_s", (1, 4)),
))
def test_forward_rejects_wrong_shape(model, name, shape):
    inputs = _inputs()
    inputs[name] = torch.zeros(shape, dtype=torch.bool if name == "time_valid" else torch.float32)
    with pytest.raises(ContractError, match="shape|images must"):
        model(**inputs)


@pytest.mark.parametrize("name", ("time_valid", "route_mask", "ego_history_mask"))
def test_numeric_validity_masks_are_rejected(model, name):
    inputs = _inputs()
    inputs[name] = inputs[name].float()
    with pytest.raises(ContractError, match="boolean"):
        model(**inputs)


@pytest.mark.parametrize("name", ("images", "calibration", "relative_pose", "route_xy", "ego_history", "time_offsets_s"))
def test_nonfinite_even_masked_slots_rejected(model, name):
    inputs = _inputs()
    inputs["time_valid"][:, 0] = False
    inputs[name].reshape(-1)[0] = float("nan")
    with pytest.raises(ContractError, match="finite"):
        model(**inputs)


def test_forward_rejects_future_and_misordered_causal_times(model):
    inputs = _inputs()
    inputs["time_offsets_s"][0, -1] = 0.1
    with pytest.raises(ContractError, match="future"):
        model(**inputs)
    inputs = _inputs()
    inputs["time_offsets_s"][0, 1] = -0.9
    with pytest.raises(ContractError, match="increase strictly"):
        model(**inputs)
    inputs = _inputs()
    inputs["time_offsets_s"][0, 1] = -0.59
    with pytest.raises(ContractError, match="pinned five"):
        model(**inputs)


@pytest.mark.parametrize("name", ("time_valid", "ego_history_mask"))
def test_current_camera_or_ego_cannot_be_invalid(model, name):
    inputs = _inputs()
    inputs[name][:, -1] = False
    with pytest.raises(ContractError, match="current"):
        model(**inputs)


def test_current_relative_pose_zero_and_valid_yaw_radians(model):
    inputs = _inputs()
    inputs["relative_pose"][:, -1, 0] = 1.0
    with pytest.raises(ContractError, match="current relative pose"):
        model(**inputs)
    inputs = _inputs()
    inputs["relative_pose"][:, 0, 2] = 4.0
    with pytest.raises(ContractError, match="wrapped"):
        model(**inputs)


def test_dense_navigation_requires_contiguous_prefix_and_two_points(model):
    inputs = _inputs()
    inputs["route_mask"][:, 20] = False
    with pytest.raises(ContractError, match="prefix"):
        model(**inputs)
    inputs = _inputs()
    inputs["route_mask"][:, 1:] = False
    with pytest.raises(ContractError, match="at least two"):
        model(**inputs)


def test_masked_camera_pose_ego_and_navigation_values_do_not_change_outputs(model):
    inputs = _inputs()
    inputs["time_valid"][:, :2] = False
    inputs["ego_history_mask"][:, :4] = False
    inputs["route_mask"][:, 80:] = False
    changed = {key: value.clone() for key, value in inputs.items()}
    changed["images"][:, :2] = 1234.0
    changed["relative_pose"][:, :2] = 456.0
    changed["ego_history"][:, :4] = 789.0
    changed["route_xy"][:, 80:] = 1024.0
    with torch.no_grad():
        a = model(**inputs)
        b = model(**changed)
    assert _maximum_difference(a, b) == 0.0


def test_current_only_masks_past_images_and_preserves_exact_same_parameters(model):
    inputs = _inputs()
    changed = {key: value.clone() for key, value in inputs.items()}
    changed["images"][:, :-1] = -100.0
    changed["relative_pose"][:, :-1, :2] = 100.0
    before = {key: value.clone() for key, value in model.state_dict().items()}
    with torch.no_grad():
        a = model(**inputs, history_mode="current_only")
        b = model(**changed, history_mode="current_only")
        inputs["time_valid"][:, :-1] = False
        c = model(**inputs, history_mode="temporal")
    assert _maximum_difference(a, b) == 0.0
    assert _maximum_difference(a, c) == 0.0
    assert all(torch.equal(before[key], value) for key, value in model.state_dict().items())


def test_two_arms_can_share_exact_initial_tensor_state():
    torch.manual_seed(701)
    current_only = TemporalModelV2(TemporalModelConfigV2(encoder_base_channels=4, hidden_width=128))
    torch.manual_seed(701)
    temporal = TemporalModelV2(TemporalModelConfigV2(encoder_base_channels=4, hidden_width=128))
    assert all(torch.equal(value, temporal.state_dict()[key]) for key, value in current_only.state_dict().items())
    assert parameter_count(current_only) == parameter_count(temporal)


@pytest.mark.parametrize("kind", ("calibration", "past_pose", "past_image", "current_spatial", "navigation", "ego"))
def test_valid_sensor_geometry_and_spatial_content_condition_prediction(model, kind):
    inputs = _inputs()
    changed = {key: value.clone() for key, value in inputs.items()}
    if kind == "calibration":
        changed["calibration"][:, 0, 0] += 1.0
    elif kind == "past_pose":
        changed["relative_pose"][:, 0, 0] += 8.0
    elif kind == "past_image":
        changed["images"][:, 0, 0] = 2.0
    elif kind == "current_spatial":
        changed["images"][:, -1, 0] = torch.flip(changed["images"][:, -1, 0], dims=(-1,))
    elif kind == "navigation":
        changed["route_xy"][..., 1] = 10.0
    elif kind == "ego":
        changed["ego_history"][:, -1, 0] = 5.0
    with torch.no_grad():
        a = model(**inputs)
        b = model(**changed)
    assert _maximum_difference(a, b) > 1.0e-6


def test_full_resolution_encoder_retains_sixty_grid_tokens(model):
    with torch.no_grad():
        features = model.image_encoder(_inputs()["images"][0, -1])
    assert features.shape == (6, 60, 128)
    assert torch.isfinite(features).all()
    assert features.std(dim=1).sum() > 0


def test_unknown_history_mode_and_mismatched_dtype_rejected(model):
    with pytest.raises(ContractError, match="history_mode"):
        model(**_inputs(), history_mode="future_only")
    with pytest.raises(ContractError, match="history_mode"):
        model(**_inputs(), history_mode=[])
    inputs = _inputs()
    inputs["calibration"] = inputs["calibration"].double()
    with pytest.raises(ContractError, match="dtype"):
        model(**inputs)
