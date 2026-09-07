from __future__ import annotations

from dataclasses import replace
import hashlib
import inspect
import json
from pathlib import Path

import pytest

torch = pytest.importorskip("torch")

# HH_260906 - Research scoring must not alter legacy model or physical decoder behavior.
from portable_e2e import ContractError
from portable_e2e.dataset import FEATURE_NAMES
from portable_e2e.losses import trajectory_loss
from portable_e2e.model import (
    CANDIDATE_RANK_MODEL_ID,
    ConvImageEncoder,
    MODEL_ID,
    ModelConfig,
    PHYSICAL_MAXIMUM_ACCELERATION_MPS2,
    PHYSICAL_MAXIMUM_CURVATURE_RAD_PER_M,
    PHYSICAL_MAXIMUM_LATERAL_ACCELERATION_MPS2,
    PHYSICAL_MAXIMUM_SPEED_MPS,
    PHYSICAL_MODEL_ID,
    PHYSICAL_TIME_STEP_S,
    PerspectiveTrajectoryModel,
    parameter_count,
)
from portable_e2e.runtime_weight_bundle import _validate_model_config
from portable_e2e.runtime_contract import RuntimeGateConfig, validate_and_select_trajectory


ROOT = Path(__file__).resolve().parents[1]
CONFIG_ROOT = ROOT / "portable_e2e/config"


def _config(model_id: str = CANDIDATE_RANK_MODEL_ID) -> ModelConfig:
    return ModelConfig(
        model_id=model_id,
        route_points=8,
        candidate_count=3,
        image_width=32,
        image_height=16,
        image_grid_width=2,
        image_grid_height=1,
        image_embedding=8,
        camera_fusion_width=16,
        ego_embedding=8,
        route_embedding=8,
        hidden_width=16,
        encoder_base_channels=4,
        ego_history_frames=3,
        maximum_step_m=1.0,
    )


def _inputs(config: ModelConfig) -> dict[str, torch.Tensor]:
    batch = 2
    ego = torch.zeros(batch, config.ego_history_frames, config.ego_features)
    ego[:, -1, FEATURE_NAMES.index("velocity_x_mps")] = 4.0
    route = torch.zeros(batch, config.route_points, 2)
    route[:, :, 0] = torch.arange(config.route_points) * 10.0
    return {
        "images": torch.zeros(batch, 6, 3, config.image_height, config.image_width),
        "calibration": torch.zeros(batch, 6, config.calibration_features),
        "ego_history": ego,
        "ego_history_mask": torch.ones(batch, config.ego_history_frames, dtype=torch.bool),
        "route_xy": route,
        "route_mask": torch.ones(batch, config.route_points, dtype=torch.bool),
    }


def _hash_source(source: str) -> str:
    return hashlib.sha256(source.rstrip().encode()).hexdigest()


def test_legacy_initialization_and_encoder_source_are_frozen() -> None:
    # HH_260906 - Pin the executed pre-research source independently of PyTorch RNG versions.
    initializer = inspect.getsource(PerspectiveTrajectoryModel.__init__).split(
        "        # HH_260906 - Replace only the research scorer"
    )[0]
    forward_prefix = inspect.getsource(PerspectiveTrajectoryModel.forward).split(
        "        if cfg.model_id in (PHYSICAL_MODEL_ID, CANDIDATE_RANK_MODEL_ID):"
    )[0]
    assert _hash_source(initializer) == (
        "bd9b6eb8c596cc117c5ed4faca85e4cc002416b7cf11f4215cf9fc75df32ebdd"
    )
    assert _hash_source(forward_prefix) == (
        "aabd1165128c2239ed7d9b0f17943bc4e34e39d98dc87401398a4025b0d030b3"
    )
    assert _hash_source(inspect.getsource(ConvImageEncoder)) == (
        "a70ca7680ce1be1e34b13cbdb773862c45043f18633cefb09189ad1bdddae8bb"
    )
    assert _hash_source(inspect.getsource(PerspectiveTrajectoryModel._decode_physical_v1)) == (
        "ffa4930c7624432a68df3ce9cf2c4585d4f98f00fc11ec6394a13fcfaceac420"
    )


@pytest.mark.parametrize(
    ("name", "model_id", "parameters", "channels"),
    (
        ("v0", MODEL_ID, 1_053_278, 3),
        ("physical_v1", PHYSICAL_MODEL_ID, 954_590, 2),
    ),
)
def test_old_config_schema_state_and_same_seed_outputs_remain_unchanged(
    name: str, model_id: str, parameters: int, channels: int
) -> None:
    mapping = json.loads((CONFIG_ROOT / f"perspective_trajectory_{name}.model.json").read_text())
    config = ModelConfig.from_mapping(mapping)
    assert config.to_dict() == mapping
    assert parameter_count(PerspectiveTrajectoryModel(config)) == parameters
    small = _config(model_id)
    torch.manual_seed(20260903)
    first = PerspectiveTrajectoryModel(small).eval()
    torch.manual_seed(20260903)
    second = PerspectiveTrajectoryModel(small).eval()
    assert isinstance(first.candidate_head, torch.nn.Linear)
    assert first.trajectory_head.out_features == small.candidate_count * 64 * channels
    assert set(key for key in first.state_dict() if key.startswith("candidate_head.")) == {
        "candidate_head.weight", "candidate_head.bias"
    }
    for key, value in first.state_dict().items():
        assert torch.equal(value, second.state_dict()[key])
    second.load_state_dict(first.state_dict(), strict=True)
    with torch.no_grad():
        for actual, expected in zip(first(**_inputs(small)), second(**_inputs(small))):
            assert torch.equal(actual, expected)


def test_new_config_changes_only_explicit_model_id_and_stays_outside_runtime_bundle() -> None:
    legacy = json.loads((CONFIG_ROOT / "perspective_trajectory_physical_v1.model.json").read_text())
    mapping = json.loads((CONFIG_ROOT / "perspective_trajectory_candidate_rank_v1.model.json").read_text())
    assert mapping == {**legacy, "model_id": CANDIDATE_RANK_MODEL_ID}
    config = ModelConfig.from_mapping(mapping)
    assert config.to_dict() == mapping
    assert parameter_count(PerspectiveTrajectoryModel(config)) == 1_068_249
    with pytest.raises(ContractError, match="not supported by bundle v1"):
        _validate_model_config(mapping, "0" * 64)


@pytest.mark.parametrize("value", (2.0, 3.0, float("nan"), float("inf"), True))
def test_new_config_rejects_invalid_physical_step_envelope(value: object) -> None:
    with pytest.raises(ContractError):
        replace(_config(), maximum_step_m=value).validate()


@pytest.mark.parametrize(
    "changes", ({"model_id": "candidate_rank"}, {"candidate_count": 0}, {"candidate_count": True})
)
def test_new_config_rejects_unknown_id_or_invalid_candidate_count(changes: dict) -> None:
    with pytest.raises(ContractError):
        ModelConfig.from_mapping({**_config().to_dict(), **changes})


def test_new_config_preserves_exact_field_validation() -> None:
    mapping = _config().to_dict()
    with pytest.raises(ContractError, match="unknown model config fields"):
        ModelConfig.from_mapping({**mapping, "candidate_rank_width": 16})
    del mapping["maximum_step_m"]
    with pytest.raises(ContractError, match="missing model config fields"):
        ModelConfig.from_mapping(mapping)


def test_same_seed_research_model_preserves_shared_weights_and_decoded_outputs() -> None:
    torch.manual_seed(20260903)
    physical = PerspectiveTrajectoryModel(_config(PHYSICAL_MODEL_ID)).eval()
    torch.manual_seed(20260903)
    research = PerspectiveTrajectoryModel(_config()).eval()
    for key, value in physical.state_dict().items():
        if not key.startswith("candidate_head."):
            assert torch.equal(value, research.state_dict()[key]), key
    with torch.no_grad():
        expected = physical(**_inputs(physical.config))
        actual = research(**_inputs(research.config))
    assert tuple(value.shape for value in actual) == ((2, 3, 64, 2), (2, 3, 64), (2, 3))
    assert torch.equal(actual[0], expected[0])
    assert torch.equal(actual[1], expected[1])
    assert all(torch.isfinite(value).all() for value in actual)
    research.reset_parameters()
    assert torch.equal(research.candidate_head[-1].bias, torch.zeros(1))


def test_scores_use_normalized_xy_and_speed_without_candidate_index_bias() -> None:
    config = _config()
    model = PerspectiveTrajectoryModel(config)
    with torch.no_grad():
        for parameter in model.candidate_head.parameters():
            parameter.zero_()
        model.candidate_head[0].weight[0, config.hidden_width] = 1.0
        model.candidate_head[0].weight[0, config.hidden_width + 128] = 1.0
        model.candidate_head[2].weight[0, 0] = 1.0
    context = torch.zeros(1, config.hidden_width)
    xy = torch.zeros(1, 3, 64, 2)
    speed = torch.zeros(1, 3, 64)
    xy[0, 1, 0, 0] = config.route_scale_m
    speed[0, 2, 0] = PHYSICAL_MAXIMUM_SPEED_MPS
    assert torch.equal(model._score_candidates(context, xy, speed), torch.tensor([[0.0, 1.0, 1.0]]))


def test_shared_scorer_is_permutation_equivariant_and_duplicates_have_equal_scores() -> None:
    torch.manual_seed(20260904)
    config = _config()
    model = PerspectiveTrajectoryModel(config)
    context = torch.randn(2, config.hidden_width)
    xy = torch.randn(2, 3, 64, 2)
    speed = torch.rand(2, 3, 64)
    permutation = torch.tensor([2, 0, 1])
    scores = model._score_candidates(context, xy, speed)
    permuted = model._score_candidates(context, xy[:, permutation], speed[:, permutation])
    # HH_260906 - GEMM layout can change float32 rounding without changing candidate equivariance.
    assert torch.allclose(permuted, scores[:, permutation], atol=1.0e-7, rtol=1.0e-6)
    duplicates = model._score_candidates(
        context, xy[:, :1].expand(-1, 3, -1, -1), speed[:, :1].expand(-1, 3, -1)
    )
    assert torch.allclose(
        duplicates, duplicates[:, :1].expand_as(duplicates), atol=1.0e-7, rtol=1.0e-6
    )


def test_score_gradients_detach_geometry_but_retain_shared_context_learning() -> None:
    torch.manual_seed(20260905)
    config = _config()
    model = PerspectiveTrajectoryModel(config)
    context = torch.ones(2, config.hidden_width, requires_grad=True)
    xy = torch.ones(2, 3, 64, 2, requires_grad=True)
    speed = torch.ones(2, 3, 64, requires_grad=True)
    model._score_candidates(context, xy, speed).sum().backward()
    assert xy.grad is None
    assert speed.grad is None
    assert context.grad is not None and torch.isfinite(context.grad).all()
    assert float(context.grad.abs().sum()) > 0.0
    assert all(parameter.grad is not None for parameter in model.candidate_head.parameters())


def test_classification_only_backward_does_not_directly_train_trajectory_head() -> None:
    torch.manual_seed(20260904)
    model = PerspectiveTrajectoryModel(_config())
    xy, speed, logits = model(**_inputs(model.config))
    xy.retain_grad()
    speed.retain_grad()
    torch.nn.functional.cross_entropy(logits, torch.tensor([0, 2])).backward()
    assert xy.grad is None and speed.grad is None
    assert model.trajectory_head.weight.grad is None
    assert model.trajectory_head.bias.grad is None
    assert model.fusion[0].weight.grad is not None
    assert torch.isfinite(model.fusion[0].weight.grad).all()
    assert float(model.fusion[0].weight.grad.abs().sum()) > 0.0


@pytest.mark.parametrize("raw_acceleration", (-100.0, 0.0, 100.0))
def test_research_decoder_preserves_speed_acceleration_curvature_and_lateral_bounds(
    raw_acceleration: float,
) -> None:
    config = _config()
    model = PerspectiveTrajectoryModel(config).eval()
    inputs = _inputs(config)
    inputs["route_xy"] = inputs["route_xy"].flip(-1)
    with torch.no_grad():
        model.trajectory_head.weight.zero_()
        raw = model.trajectory_head.bias.reshape(3, 64, 2)
        raw[..., 0] = raw_acceleration
        raw[..., 1] = 100.0
        xy, speed, logits = model(**inputs)
    steps = torch.diff(torch.cat((torch.zeros_like(xy[:, :, :1]), xy), dim=2), dim=2)
    distances = torch.linalg.norm(steps, dim=-1)
    entry_speed = torch.cat((torch.full_like(speed[:, :, :1], 4.0), speed[:, :, :-1]), dim=2)
    acceleration = (speed - entry_speed) / PHYSICAL_TIME_STEP_S
    headings = torch.atan2(steps[..., 1], steps[..., 0])
    heading_delta = torch.diff(
        torch.cat((torch.zeros_like(headings[:, :, :1]), headings), dim=2), dim=2
    )
    moving = distances > 1.0e-4
    curvature = heading_delta[moving].abs() / distances[moving]
    lateral = curvature * torch.maximum(entry_speed, speed)[moving].square()
    assert float(speed.min()) >= 0.0
    assert float(speed.max()) <= PHYSICAL_MAXIMUM_SPEED_MPS + 1.0e-6
    assert float(acceleration.abs().max()) <= PHYSICAL_MAXIMUM_ACCELERATION_MPS2 + 1.0e-5
    assert torch.allclose(distances / PHYSICAL_TIME_STEP_S, speed, atol=2.0e-5, rtol=1.0e-5)
    assert float(curvature.max()) <= PHYSICAL_MAXIMUM_CURVATURE_RAD_PER_M + 1.0e-4
    # HH_260906 - Account for cancellation when reconstructing curvature from float32 cumulative XY.
    assert float(lateral.max()) <= PHYSICAL_MAXIMUM_LATERAL_ACCELERATION_MPS2 + 5.0e-4
    selected = validate_and_select_trajectory(
        xy[0].tolist(), speed[0].tolist(), logits[0].tolist(),
        RuntimeGateConfig(candidate_count=config.candidate_count), current_speed_mps=4.0,
    )
    assert 0 <= selected.candidate_index < config.candidate_count


def test_existing_loss_and_optimizer_backpropagate_finite_research_gradients() -> None:
    torch.manual_seed(20260903)
    model = PerspectiveTrajectoryModel(_config())
    optimizer = torch.optim.Adam(model.parameters(), lr=1.0e-4)
    xy, speed, logits = model(**_inputs(model.config))
    target_xy = torch.zeros(2, 64, 2)
    target_xy[:, :, 0] = torch.arange(1, 65) * 0.3
    result = trajectory_loss(
        xy, speed, logits, target_xy, torch.full((2, 64), 3.0), torch.ones(2, 64, dtype=torch.bool)
    )
    result["loss"].backward()
    for head in (model.trajectory_head, model.candidate_head):
        assert all(parameter.grad is not None for parameter in head.parameters())
        assert all(torch.isfinite(parameter.grad).all() for parameter in head.parameters())
        assert sum(float(parameter.grad.abs().sum()) for parameter in head.parameters()) > 0.0
    optimizer.step()
    assert all(torch.isfinite(parameter).all() for parameter in model.parameters())
