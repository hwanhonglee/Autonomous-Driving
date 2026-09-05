from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

torch = pytest.importorskip("torch")

# HH_260906 - Verify physical v1 constraints and the unchanged legacy v0 ABI.
from portable_e2e import ContractError
from portable_e2e.dataset import FEATURE_NAMES
from portable_e2e.losses import trajectory_loss
from portable_e2e.model import (
    MODEL_ID,
    PHYSICAL_MAXIMUM_ACCELERATION_MPS2,
    PHYSICAL_MAXIMUM_CURVATURE_RAD_PER_M,
    PHYSICAL_MAXIMUM_LATERAL_ACCELERATION_MPS2,
    PHYSICAL_MAXIMUM_ROUTE_SLIP_RAD,
    PHYSICAL_MAXIMUM_SPEED_MPS,
    PHYSICAL_MODEL_ID,
    PHYSICAL_TIME_STEP_S,
    ModelConfig,
    PerspectiveTrajectoryModel,
    parameter_count,
)
from portable_e2e.runtime_contract import RuntimeGateConfig
from portable_e2e.runtime_contract import validate_and_select_trajectory


ROOT = Path(__file__).resolve().parents[1]


def _config(*, model_id: str = PHYSICAL_MODEL_ID) -> ModelConfig:
    return ModelConfig(
        model_id=model_id,
        route_points=8,
        candidate_count=2,
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


def _inputs(config: ModelConfig, *, velocity_x_mps: float = 0.0) -> dict[str, torch.Tensor]:
    ego_history = torch.zeros(1, config.ego_history_frames, len(FEATURE_NAMES))
    ego_history[:, -1, FEATURE_NAMES.index("velocity_x_mps")] = velocity_x_mps
    route_xy = torch.zeros(1, config.route_points, 2)
    route_xy[0, :, 0] = torch.arange(config.route_points, dtype=torch.float32)
    return {
        "images": torch.zeros(
            1,
            config.camera_count,
            config.image_channels,
            config.image_height,
            config.image_width,
        ),
        "calibration": torch.zeros(
            1, config.camera_count, config.calibration_features
        ),
        "ego_history": ego_history,
        "ego_history_mask": torch.ones(
            1, config.ego_history_frames, dtype=torch.bool
        ),
        "route_xy": route_xy,
        "route_mask": torch.ones(1, config.route_points, dtype=torch.bool),
    }


def _zero_model(model: PerspectiveTrajectoryModel) -> None:
    with torch.no_grad():
        for parameter in model.parameters():
            parameter.zero_()


def _select_with_runtime_gate(
    candidate_xy: torch.Tensor,
    candidate_speed: torch.Tensor,
    candidate_logits: torch.Tensor,
    *,
    current_speed_mps: float,
):
    # HH_260906 - Cross-check float32 physical outputs at the real runtime boundary.
    return validate_and_select_trajectory(
        candidate_xy[0].tolist(),
        candidate_speed[0].tolist(),
        candidate_logits[0].tolist(),
        RuntimeGateConfig(candidate_count=2),
        current_speed_mps=current_speed_mps,
    )


def test_v0_config_and_state_abi_are_unchanged() -> None:
    config_path = ROOT / "portable_e2e/config/perspective_trajectory_v0.model.json"
    expected = json.loads(config_path.read_text(encoding="utf-8"))
    config = ModelConfig.from_mapping(expected)
    model = PerspectiveTrajectoryModel(config)

    assert config.model_id == MODEL_ID
    assert config.to_dict() == expected
    assert model.trajectory_head.out_features == 6 * 64 * 3
    assert parameter_count(model) == 1_053_278


def test_v1_config_uses_separate_model_id_and_two_channel_decoder() -> None:
    config_path = (
        ROOT / "portable_e2e/config/perspective_trajectory_physical_v1.model.json"
    )
    expected = json.loads(config_path.read_text(encoding="utf-8"))
    config = ModelConfig.from_mapping(expected)
    model = PerspectiveTrajectoryModel(config)

    assert config.model_id == PHYSICAL_MODEL_ID
    assert config.to_dict() == expected
    assert model.trajectory_head.out_features == 6 * 64 * 2


def test_v1_config_rejects_a_misleading_step_envelope() -> None:
    with pytest.raises(ContractError, match="maximum_step_m must be exactly 1.0"):
        ModelConfig.from_mapping(
            {**_config().to_dict(), "maximum_step_m": 2.0}
        )


def test_v1_zero_acceleration_supports_exact_stationary_trajectory() -> None:
    config = _config()
    model = PerspectiveTrajectoryModel(config).eval()
    _zero_model(model)

    with torch.no_grad():
        candidate_xy, candidate_speed, candidate_logits = model(**_inputs(config))

    assert candidate_xy.shape == (1, 2, 64, 2)
    assert candidate_speed.shape == (1, 2, 64)
    assert candidate_logits.shape == (1, 2)
    assert torch.equal(candidate_xy, torch.zeros_like(candidate_xy))
    assert torch.equal(candidate_speed, torch.zeros_like(candidate_speed))


@pytest.mark.parametrize("current_speed_mps", (0.0, 0.01, 0.05, 0.099, 0.1, 0.11))
@pytest.mark.parametrize(
    "raw_acceleration",
    (-100.0, math.atanh(-0.1 / 2.9), 0.0, math.atanh(0.01 / 2.9), 100.0),
)
def test_v1_low_speed_motion_remains_inside_runtime_gate(
    current_speed_mps: float,
    raw_acceleration: float,
) -> None:
    config = _config()
    model = PerspectiveTrajectoryModel(config).eval()
    _zero_model(model)
    with torch.no_grad():
        model.trajectory_head.bias.reshape(2, 64, 2)[..., 0].fill_(
            raw_acceleration
        )
        candidate_xy, candidate_speed, candidate_logits = model(
            **_inputs(config, velocity_x_mps=current_speed_mps)
        )

    selected = _select_with_runtime_gate(
        candidate_xy,
        candidate_speed,
        candidate_logits,
        current_speed_mps=current_speed_mps,
    )

    assert selected.candidate_index == 0


@pytest.mark.parametrize(
    ("velocity_x_mps", "expected_speed_mps"),
    ((-0.1, 0.0), (PHYSICAL_MAXIMUM_SPEED_MPS + 0.1, PHYSICAL_MAXIMUM_SPEED_MPS)),
)
def test_v1_bounds_recorded_current_speed_noise(
    velocity_x_mps: float,
    expected_speed_mps: float,
) -> None:
    # HH_260906 - Keep training usable with measured noise without weakening the live raw-speed gate.
    config = _config()
    model = PerspectiveTrajectoryModel(config).eval()
    _zero_model(model)

    with torch.no_grad():
        candidate_xy, candidate_speed, _ = model(
            **_inputs(config, velocity_x_mps=velocity_x_mps)
        )

    assert torch.allclose(
        candidate_speed,
        torch.full_like(candidate_speed, expected_speed_mps),
    )
    assert torch.isfinite(candidate_xy).all()


@pytest.mark.parametrize("raw_acceleration", (-100.0, 100.0))
def test_v1_speed_integration_is_acceleration_bounded_and_step_consistent(
    raw_acceleration: float,
) -> None:
    config = _config()
    model = PerspectiveTrajectoryModel(config).eval()
    _zero_model(model)
    with torch.no_grad():
        model.trajectory_head.bias.reshape(2, 64, 2)[..., 0].fill_(
            raw_acceleration
        )

    inputs = _inputs(config, velocity_x_mps=4.0)
    with torch.no_grad():
        candidate_xy, candidate_speed, _ = model(**inputs)

    origin = torch.zeros_like(candidate_xy[:, :, :1])
    step_xy = torch.diff(torch.cat((origin, candidate_xy), dim=2), dim=2)
    geometric_speed = torch.linalg.norm(step_xy, dim=-1) / PHYSICAL_TIME_STEP_S
    initial_speed = torch.full_like(candidate_speed[:, :, :1], 4.0)
    acceleration = torch.diff(
        torch.cat((initial_speed, candidate_speed), dim=2), dim=2
    ) / PHYSICAL_TIME_STEP_S

    assert float(candidate_speed.min()) >= 0.0
    assert float(candidate_speed.max()) <= PHYSICAL_MAXIMUM_SPEED_MPS + 1.0e-6
    assert (
        float(acceleration.abs().max())
        <= PHYSICAL_MAXIMUM_ACCELERATION_MPS2 + 1.0e-5
    )
    assert torch.allclose(geometric_speed, candidate_speed, atol=2.0e-5, rtol=1.0e-5)


def test_v1_saturated_acceleration_remains_inside_runtime_gate() -> None:
    config = _config()
    model = PerspectiveTrajectoryModel(config).eval()
    _zero_model(model)
    with torch.no_grad():
        model.trajectory_head.bias.reshape(2, 64, 2)[..., 0].fill_(100.0)
        candidate_xy, candidate_speed, candidate_logits = model(
            **_inputs(config, velocity_x_mps=4.0)
        )

    selected = _select_with_runtime_gate(
        candidate_xy,
        candidate_speed,
        candidate_logits,
        current_speed_mps=4.0,
    )

    assert selected.candidate_index == 0


def test_v1_short_low_speed_braking_stop_remains_inside_runtime_gate() -> None:
    # HH_260906 - Keep a saturated legal stop compatible with the live minimum-extent gate.
    config = _config()
    model = PerspectiveTrajectoryModel(config).eval()
    _zero_model(model)
    with torch.no_grad():
        model.trajectory_head.bias.reshape(2, 64, 2)[..., 0].fill_(-100.0)
        candidate_xy, candidate_speed, candidate_logits = model(
            **_inputs(config, velocity_x_mps=0.4)
        )

    selected = _select_with_runtime_gate(
        candidate_xy,
        candidate_speed,
        candidate_logits,
        current_speed_mps=0.4,
    )

    assert selected.candidate_index == 0
    assert selected.planar_extent_m == pytest.approx(0.011, abs=1.0e-6)
    assert float(candidate_speed[0, 0, -1]) == 0.0


def test_v1_high_speed_decelerating_turn_remains_inside_lateral_gate() -> None:
    # HH_260906 - Exercise segment-entry speed during a saturated high-speed turn.
    config = _config()
    model = PerspectiveTrajectoryModel(config).eval()
    _zero_model(model)
    with torch.no_grad():
        raw = model.trajectory_head.bias.reshape(2, 64, 2)
        raw[..., 0] = -100.0
        raw[..., 1] = 100.0
    inputs = _inputs(config, velocity_x_mps=PHYSICAL_MAXIMUM_SPEED_MPS)
    inputs["route_xy"][0, :, 0] = 0.0
    inputs["route_xy"][0, :, 1] = torch.arange(config.route_points)

    with torch.no_grad():
        candidate_xy, candidate_speed, candidate_logits = model(**inputs)

    selected = _select_with_runtime_gate(
        candidate_xy,
        candidate_speed,
        candidate_logits,
        current_speed_mps=PHYSICAL_MAXIMUM_SPEED_MPS,
    )

    assert selected.candidate_index == 0


def test_v1_follows_route_tangent_with_bounded_learned_slip() -> None:
    config = _config()
    model = PerspectiveTrajectoryModel(config).eval()
    _zero_model(model)
    with torch.no_grad():
        raw = model.trajectory_head.bias.reshape(2, 64, 2)
        raw[..., 1] = 100.0

    inputs = _inputs(config, velocity_x_mps=2.0)
    inputs["route_xy"][0, :, 0] = 0.0
    inputs["route_xy"][0, :, 1] = torch.arange(config.route_points)
    with torch.no_grad():
        candidate_xy, candidate_speed, candidate_logits = model(**inputs)

    origin = torch.zeros_like(candidate_xy[:, :, :1])
    step_xy = torch.diff(torch.cat((origin, candidate_xy), dim=2), dim=2)
    heading = torch.atan2(step_xy[..., 1], step_xy[..., 0])
    deviation = (math.pi / 2.0) - heading

    heading_delta = torch.diff(
        torch.cat((torch.zeros_like(heading[:, :, :1]), heading), dim=2), dim=2
    )
    step_distance = torch.linalg.norm(step_xy, dim=-1)

    assert float(deviation.abs()[-1, -1, -1]) < math.pi / 2.0
    assert torch.all(
        heading_delta.abs()
        <= step_distance * PHYSICAL_MAXIMUM_CURVATURE_RAD_PER_M + 1.0e-5
    )
    entry_speed = torch.cat(
        (
            torch.full_like(candidate_speed[:, :, :1], 2.0),
            candidate_speed[:, :, :-1],
        ),
        dim=2,
    )
    implied_lateral_acceleration = (
        heading_delta.abs()
        / step_distance.clamp_min(1.0e-6)
        * torch.maximum(entry_speed, candidate_speed).square()
    )
    assert (
        float(implied_lateral_acceleration.max())
        <= PHYSICAL_MAXIMUM_LATERAL_ACCELERATION_MPS2 + 1.0e-5
    )

    selected = _select_with_runtime_gate(
        candidate_xy,
        candidate_speed,
        candidate_logits,
        current_speed_mps=2.0,
    )
    assert selected.candidate_index == 0


def test_v1_random_curved_routes_remain_inside_runtime_gate() -> None:
    # HH_260906 - Exercise low-speed heading deadbands and changing turn geometry deterministically.
    config = _config()
    model = PerspectiveTrajectoryModel(config).eval()
    velocity_index = FEATURE_NAMES.index("velocity_x_mps")

    for seed_offset in range(512):
        generator = torch.Generator().manual_seed(260_906_000 + seed_offset)
        if seed_offset % 2 == 0:
            current_speed_mps = float(torch.rand((), generator=generator)) * 0.6
        else:
            current_speed_mps = 0.6 + float(torch.rand((), generator=generator)) * 7.7
        raw = torch.rand((1, 2, 64, 2), generator=generator) * 8.0 - 4.0
        turns = torch.rand(7, generator=generator) * 1.8 - 0.9
        turns[0] *= 0.2
        headings = torch.cumsum(turns, dim=0).clamp(-2.8, 2.8)
        lengths = 0.25 + torch.rand(7, generator=generator) * 4.75
        segments = torch.stack((torch.cos(headings), torch.sin(headings)), dim=1)
        segments *= lengths.unsqueeze(1)
        route_xy = torch.cat(
            (torch.zeros(1, 2), torch.cumsum(segments, dim=0)),
            dim=0,
        ).unsqueeze(0)
        ego_history = torch.zeros(1, config.ego_history_frames, len(FEATURE_NAMES))
        ego_history[0, -1, velocity_index] = current_speed_mps

        with torch.no_grad():
            candidate_xy, candidate_speed = model._decode_physical_v1(
                raw,
                ego_history,
                route_xy,
                torch.ones(1, config.route_points, dtype=torch.bool),
            )

        selected = _select_with_runtime_gate(
            candidate_xy,
            candidate_speed,
            torch.zeros(1, config.candidate_count),
            current_speed_mps=current_speed_mps,
        )
        assert selected.candidate_index == 0


def test_v1_forward_backpropagates_through_physical_decoder() -> None:
    config = _config()
    model = PerspectiveTrajectoryModel(config)
    candidate_xy, candidate_speed, candidate_logits = model(
        **_inputs(config, velocity_x_mps=1.0)
    )
    loss = (
        candidate_xy.square().mean()
        + candidate_speed.mean()
        + candidate_logits.mean()
    )

    loss.backward()

    assert model.trajectory_head.weight.grad is not None
    assert torch.isfinite(model.trajectory_head.weight.grad).all()
    assert float(model.trajectory_head.weight.grad.abs().sum()) > 0.0


def test_v1_outputs_work_with_existing_training_loss_and_optimizer() -> None:
    config = _config()
    model = PerspectiveTrajectoryModel(config)
    optimizer = torch.optim.Adam(model.parameters(), lr=1.0e-3)
    candidate_xy, candidate_speed, candidate_logits = model(
        **_inputs(config, velocity_x_mps=1.0)
    )
    valid = torch.ones(1, config.future_points, dtype=torch.bool)
    result = trajectory_loss(
        candidate_xy,
        candidate_speed,
        candidate_logits,
        candidate_xy[:, 0].detach(),
        candidate_speed[:, 0].detach(),
        valid,
    )

    optimizer.zero_grad(set_to_none=True)
    result["loss"].backward()
    optimizer.step()

    assert torch.isfinite(result["loss"])
