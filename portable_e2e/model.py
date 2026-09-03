"""Small VAD-independent planning model used for the first real training gate.

This module intentionally depends only on PyTorch.  It predicts candidate
trajectories, not throttle, brake, or steering commands.  Safety validation and
vehicle control remain outside the learned model.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, fields
import math
from typing import Any, Mapping

import torch
from torch import Tensor, nn
from torch.nn import functional as F
from torch.nn.utils.rnn import pack_padded_sequence

from .contract import ContractError
from .dataset import CALIBRATION_FEATURE_NAMES, FEATURE_NAMES


MODEL_ID = "portable_e2e.perspective_trajectory.v0"
IMAGE_ENCODER_DOWNSAMPLE_STAGES = 4


def _downsampled_image_dimension(value: int) -> int:
    """Return the spatial size after the encoder's stride-two convolutions."""
    for _ in range(IMAGE_ENCODER_DOWNSAMPLE_STAGES):
        value = (value + 1) // 2
    return value


@dataclass(frozen=True)
class ModelConfig:
    """Shape and capacity parameters that form part of a checkpoint ABI."""

    model_id: str = MODEL_ID
    camera_count: int = 6
    calibration_features: int = len(CALIBRATION_FEATURE_NAMES)
    ego_features: int = len(FEATURE_NAMES)
    route_points: int = 128
    future_points: int = 64
    candidate_count: int = 6
    image_width: int = 320
    image_height: int = 180
    image_channels: int = 3
    image_grid_width: int = 5
    image_grid_height: int = 3
    image_embedding: int = 96
    camera_fusion_width: int = 192
    ego_embedding: int = 64
    route_embedding: int = 96
    hidden_width: int = 256
    encoder_base_channels: int = 24
    ego_history_frames: int = 10
    maximum_step_m: float = 3.0
    route_scale_m: float = 120.0

    def validate(self) -> None:
        if self.model_id != MODEL_ID:
            raise ContractError(f"model_id must be {MODEL_ID}")
        integer_limits = {
            "camera_count": (self.camera_count, 1, 16),
            "calibration_features": (self.calibration_features, 1, 128),
            "ego_features": (self.ego_features, 1, 256),
            "route_points": (self.route_points, 2, 1024),
            "future_points": (self.future_points, 2, 256),
            "candidate_count": (self.candidate_count, 1, 32),
            "image_width": (self.image_width, 16, 2048),
            "image_height": (self.image_height, 16, 2048),
            "image_channels": (self.image_channels, 1, 8),
            "image_grid_width": (self.image_grid_width, 1, 64),
            "image_grid_height": (self.image_grid_height, 1, 64),
            "image_embedding": (self.image_embedding, 8, 2048),
            "camera_fusion_width": (self.camera_fusion_width, 8, 4096),
            "ego_embedding": (self.ego_embedding, 8, 1024),
            "route_embedding": (self.route_embedding, 8, 2048),
            "hidden_width": (self.hidden_width, 16, 8192),
            "encoder_base_channels": (self.encoder_base_channels, 4, 512),
            "ego_history_frames": (self.ego_history_frames, 1, 100),
        }
        for name, (value, minimum, maximum) in integer_limits.items():
            if isinstance(value, bool) or not isinstance(value, int):
                raise ContractError(f"model.{name} must be an integer")
            if not minimum <= value <= maximum:
                raise ContractError(f"model.{name} must be in [{minimum}, {maximum}]")
        if self.camera_count != 6:
            raise ContractError("model.camera_count must preserve the six-camera ABI")
        if self.calibration_features != len(CALIBRATION_FEATURE_NAMES):
            raise ContractError("model.calibration_features does not match the dataset ABI")
        if self.ego_features != len(FEATURE_NAMES):
            raise ContractError("model.ego_features does not match the dataset ABI")
        if self.future_points != 64:
            raise ContractError("model.future_points must preserve the 64-point ABI")
        numeric_limits = {
            "maximum_step_m": (self.maximum_step_m, 0.1, 10.0),
            "route_scale_m": (self.route_scale_m, 1.0, 10_000.0),
        }
        for name, (value, minimum, maximum) in numeric_limits.items():
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                or not minimum <= float(value) <= maximum
            ):
                raise ContractError(
                    f"model.{name} must be finite and in [{minimum}, {maximum}]"
                )
        feature_height = _downsampled_image_dimension(self.image_height)
        feature_width = _downsampled_image_dimension(self.image_width)
        if feature_height % self.image_grid_height != 0:
            raise ContractError(
                "model.image_grid_height must exactly divide the downsampled "
                f"image height {feature_height}"
            )
        if feature_width % self.image_grid_width != 0:
            raise ContractError(
                "model.image_grid_width must exactly divide the downsampled "
                f"image width {feature_width}"
            )

    def to_dict(self) -> dict[str, Any]:
        self.validate()
        return asdict(self)

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "ModelConfig":
        known = {field.name for field in fields(cls)}
        unknown = set(value) - known
        missing = known - set(value)
        if unknown:
            raise ContractError(f"unknown model config fields: {sorted(unknown)}")
        if missing:
            raise ContractError(f"missing model config fields: {sorted(missing)}")
        try:
            config = cls(**dict(value))
        except TypeError as error:
            raise ContractError(f"invalid model config: {error}") from error
        config.validate()
        return config


class ConvImageEncoder(nn.Module):
    """A compact shared-weight encoder with no torchvision dependency."""

    def __init__(
        self,
        input_channels: int,
        base_channels: int,
        output_width: int,
        grid_height: int,
        grid_width: int,
        input_height: int,
        input_width: int,
    ) -> None:
        super().__init__()
        widths = (base_channels, base_channels * 2, base_channels * 4, base_channels * 6)
        layers: list[nn.Module] = []
        previous = input_channels
        for width in widths:
            groups = max(1, min(8, width // 4))
            while width % groups != 0:
                groups -= 1
            layers.extend(
                (
                    nn.Conv2d(previous, width, kernel_size=3, stride=2, padding=1, bias=False),
                    nn.GroupNorm(groups, width),
                    nn.ReLU(inplace=True),
                )
            )
            previous = width
        self.features = nn.Sequential(*layers)
        feature_height = _downsampled_image_dimension(input_height)
        feature_width = _downsampled_image_dimension(input_width)
        kernel_size = (
            feature_height // grid_height,
            feature_width // grid_width,
        )
        # Non-overlapping fixed average pooling has a deterministic CUDA
        # backward path. AdaptiveAvgPool2d does not under PyTorch's strict
        # deterministic-algorithm mode, even for this divisible default grid.
        self.pool = nn.AvgPool2d(kernel_size=kernel_size, stride=kernel_size)
        self.projection = nn.Linear(previous * grid_height * grid_width, output_width)

    def forward(self, images: Tensor) -> Tensor:
        features = self.features(images)
        return self.projection(self.pool(features).flatten(1))


class PerspectiveTrajectoryModel(nn.Module):
    """Fuse perspective views, calibration, ego history, and a local route."""

    def __init__(self, config: ModelConfig | None = None) -> None:
        super().__init__()
        self.config = ModelConfig() if config is None else config
        self.config.validate()
        cfg = self.config

        self.image_encoder = ConvImageEncoder(
            cfg.image_channels,
            cfg.encoder_base_channels,
            cfg.image_embedding,
            cfg.image_grid_height,
            cfg.image_grid_width,
            cfg.image_height,
            cfg.image_width,
        )
        self.camera_embedding = nn.Parameter(
            torch.empty(cfg.camera_count, cfg.image_embedding)
        )
        self.calibration_encoder = nn.Sequential(
            nn.Linear(cfg.calibration_features, cfg.image_embedding),
            nn.ReLU(inplace=True),
            nn.Linear(cfg.image_embedding, cfg.image_embedding),
        )
        self.camera_fusion = nn.Sequential(
            nn.Linear(cfg.camera_count * cfg.image_embedding, cfg.camera_fusion_width),
            nn.LayerNorm(cfg.camera_fusion_width),
            nn.ReLU(inplace=True),
        )
        self.ego_input = nn.Sequential(
            nn.Linear(cfg.ego_features, cfg.ego_embedding),
            nn.ReLU(inplace=True),
        )
        self.ego_history = nn.GRU(
            input_size=cfg.ego_embedding,
            hidden_size=cfg.ego_embedding,
            batch_first=True,
        )
        self.route_point_encoder = nn.Sequential(
            nn.Linear(2, cfg.route_embedding),
            nn.ReLU(inplace=True),
            nn.Linear(cfg.route_embedding, cfg.route_embedding),
        )
        self.route_history = nn.GRU(
            input_size=cfg.route_embedding,
            hidden_size=cfg.route_embedding,
            batch_first=True,
        )
        fused_width = cfg.camera_fusion_width + cfg.ego_embedding + cfg.route_embedding
        self.fusion = nn.Sequential(
            nn.Linear(fused_width, cfg.hidden_width),
            nn.LayerNorm(cfg.hidden_width),
            nn.ReLU(inplace=True),
            nn.Linear(cfg.hidden_width, cfg.hidden_width),
            nn.ReLU(inplace=True),
        )
        self.trajectory_head = nn.Linear(
            cfg.hidden_width, cfg.candidate_count * cfg.future_points * 3
        )
        self.candidate_head = nn.Linear(cfg.hidden_width, cfg.candidate_count)
        self.reset_parameters()

    def reset_parameters(self) -> None:
        nn.init.normal_(self.camera_embedding, mean=0.0, std=0.02)
        nn.init.zeros_(self.trajectory_head.bias)
        nn.init.zeros_(self.candidate_head.bias)

    def _check_inputs(
        self,
        images: Tensor,
        calibration: Tensor,
        ego_history: Tensor,
        ego_history_mask: Tensor,
        route_xy: Tensor,
        route_mask: Tensor,
    ) -> None:
        cfg = self.config
        if images.ndim != 5:
            raise ValueError("images must have shape [B, camera, channel, height, width]")
        if tuple(images.shape[1:]) != (
            cfg.camera_count,
            cfg.image_channels,
            cfg.image_height,
            cfg.image_width,
        ):
            raise ValueError("images do not match the model camera/image ABI")
        batch = images.shape[0]
        if batch <= 0:
            raise ValueError("model inputs need a nonempty batch")
        expected_shapes = {
            "calibration": (batch, cfg.camera_count, cfg.calibration_features),
            "ego_history": (batch, cfg.ego_history_frames, cfg.ego_features),
            "ego_history_mask": (batch, cfg.ego_history_frames),
            "route_xy": (batch, cfg.route_points, 2),
            "route_mask": (batch, cfg.route_points),
        }
        actual = {
            "calibration": tuple(calibration.shape),
            "ego_history": tuple(ego_history.shape),
            "ego_history_mask": tuple(ego_history_mask.shape),
            "route_xy": tuple(route_xy.shape),
            "route_mask": tuple(route_mask.shape),
        }
        for name, expected in expected_shapes.items():
            if actual[name] != expected:
                raise ValueError(f"{name} shape {actual[name]} does not match {expected}")
        floating_inputs = {
            "images": images,
            "calibration": calibration,
            "ego_history": ego_history,
            "route_xy": route_xy,
        }
        for name, value in floating_inputs.items():
            if not torch.is_floating_point(value):
                raise ValueError(f"{name} must use a floating-point dtype")
            if value.device != images.device:
                raise ValueError(f"{name} must be on the same device as images")
            if value.dtype != images.dtype:
                raise ValueError(f"{name} must use the same dtype as images")
            if not bool(torch.isfinite(value).all().item()):
                raise FloatingPointError(f"{name} contains NaN or Inf")
        for name, value in {
            "ego_history_mask": ego_history_mask,
            "route_mask": route_mask,
        }.items():
            if value.dtype != torch.bool:
                raise ValueError(f"{name} must use torch.bool")
            if value.device != images.device:
                raise ValueError(f"{name} must be on the same device as images")
        if not bool(torch.all(ego_history_mask[:, -1]).item()):
            raise ValueError("the current ego state must always be valid")
        if bool(torch.any(ego_history_mask[:, :-1] & ~ego_history_mask[:, 1:]).item()):
            raise ValueError("ego_history_mask must be a contiguous valid suffix")
        if not bool(torch.all(route_mask.sum(dim=1) >= 2).item()):
            raise ValueError("every sample needs at least two valid route points")
        if bool(torch.any(~route_mask[:, :-1] & route_mask[:, 1:]).item()):
            raise ValueError("route_mask must be a contiguous valid prefix")

    def forward(
        self,
        images: Tensor,
        calibration: Tensor,
        ego_history: Tensor,
        ego_history_mask: Tensor,
        route_xy: Tensor,
        route_mask: Tensor,
    ) -> tuple[Tensor, Tensor, Tensor]:
        """Return candidate XY, candidate speed, and candidate logits."""
        self._check_inputs(
            images, calibration, ego_history, ego_history_mask, route_xy, route_mask
        )
        cfg = self.config
        batch = images.shape[0]

        flat_images = images.reshape(
            batch * cfg.camera_count,
            cfg.image_channels,
            cfg.image_height,
            cfg.image_width,
        )
        image_features = self.image_encoder(flat_images).reshape(
            batch, cfg.camera_count, cfg.image_embedding
        )
        camera_features = (
            image_features
            + self.camera_embedding.unsqueeze(0)
            + self.calibration_encoder(calibration)
        )
        camera_features = self.camera_fusion(camera_features.flatten(1))

        encoded_ego = self.ego_input(ego_history)
        ego_lengths = ego_history_mask.sum(dim=1, dtype=torch.long)
        history_positions = torch.arange(
            cfg.ego_history_frames, device=images.device
        ).unsqueeze(0)
        history_start = cfg.ego_history_frames - ego_lengths
        history_source = (history_start.unsqueeze(1) + history_positions).clamp_max(
            cfg.ego_history_frames - 1
        )
        compact_ego = encoded_ego.gather(
            1,
            history_source.unsqueeze(-1).expand(
                batch, cfg.ego_history_frames, cfg.ego_embedding
            ),
        )
        compact_ego = compact_ego * (history_positions < ego_lengths.unsqueeze(1)).unsqueeze(
            -1
        ).to(compact_ego.dtype)
        packed_ego = pack_padded_sequence(
            compact_ego,
            ego_lengths.detach().cpu(),
            batch_first=True,
            enforce_sorted=False,
        )
        _, ego_hidden = self.ego_history(packed_ego)
        ego_features = ego_hidden[-1]

        route_features = self.route_point_encoder(route_xy / float(cfg.route_scale_m))
        route_lengths = route_mask.sum(dim=1, dtype=torch.long)
        packed_route = pack_padded_sequence(
            route_features,
            route_lengths.detach().cpu(),
            batch_first=True,
            enforce_sorted=False,
        )
        _, route_hidden = self.route_history(packed_route)
        route_features = route_hidden[-1]

        fused = self.fusion(torch.cat((camera_features, ego_features, route_features), dim=1))
        raw = self.trajectory_head(fused).reshape(
            batch, cfg.candidate_count, cfg.future_points, 3
        )
        step_xy = torch.tanh(raw[..., :2]) * float(cfg.maximum_step_m)
        trajectory_xy = torch.cumsum(step_xy, dim=2)
        trajectory_speed = F.softplus(raw[..., 2])
        return trajectory_xy, trajectory_speed, self.candidate_head(fused)


def parameter_count(model: nn.Module) -> int:
    return sum(parameter.numel() for parameter in model.parameters())
