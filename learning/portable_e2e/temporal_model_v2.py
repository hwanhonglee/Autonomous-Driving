"""HH_261001 - Research-only spatial/temporal single-path network, not a vehicle policy.

This module implements the raw neural outputs only. It does not decode a path,
apply physical limits, select HOLD, or command a vehicle. Features remain image
grid tokens; calibration conditioning is not metric BEV reconstruction. The
initial implementation recomputes the full window. Cached online encoding and
its reset/rig-change contract are separate work, not a claimed 10 Hz capability.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, fields
import math
from typing import Any, Mapping

import torch
from torch import Tensor, nn
from torch.nn import functional as F

from .contract import ContractError


MODEL_ID_V2 = "portable_e2e.dense_nav.spatial_temporal.v2.research"
NOMINAL_TIME_OFFSETS_S = (-0.8, -0.6, -0.4, -0.2, 0.0)
HISTORY_MODES = frozenset(("temporal", "current_only"))
INTENT_NAMES_V2 = ("GO", "BRAKE_TO_STOP", "HOLD")


@dataclass(frozen=True)
class TemporalModelConfigV2:
    """HH_261001 - Serialize an independent, strict checkpoint shape/semantic ABI."""

    model_id: str = MODEL_ID_V2
    camera_count: int = 6
    image_history_frames: int = 5
    image_channels: int = 3
    image_height: int = 180
    image_width: int = 320
    image_grid_height: int = 6
    image_grid_width: int = 10
    calibration_features: int = 16
    ego_history_frames: int = 10
    ego_features: int = 13
    route_points: int = 128
    spatial_knots: int = 33
    token_width: int = 128
    query_count: int = 32
    attention_heads: int = 4
    cross_attention_layers: int = 2
    encoder_base_channels: int = 32
    hidden_width: int = 384
    maximum_curvature_rad_per_m: float = 0.2
    route_scale_m: float = 120.0
    pose_scale_m: float = 20.0
    image_normalization: str = "imagenet_rgb_v1"
    relative_pose_convention: str = "past_base_in_current_base_dx_dy_yaw_rad"
    time_offsets_s: tuple[float, ...] = NOMINAL_TIME_OFFSETS_S

    def validate(self) -> None:
        if self.model_id != MODEL_ID_V2:
            raise ContractError("temporal v2 requires its independent research model_id")
        pinned = {
            "camera_count": 6, "image_history_frames": 5, "image_channels": 3,
            "image_height": 180, "image_width": 320, "image_grid_height": 6,
            "image_grid_width": 10, "calibration_features": 16,
            "ego_history_frames": 10, "ego_features": 13, "route_points": 128,
            "spatial_knots": 33, "token_width": 128, "query_count": 32,
            "attention_heads": 4, "cross_attention_layers": 2,
        }
        for name, expected in pinned.items():
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value != expected:
                raise ContractError(f"temporal v2 {name} must be exactly {expected}")
        for name, minimum, maximum in (("encoder_base_channels", 4, 128), ("hidden_width", 128, 1024)):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
                raise ContractError(f"temporal v2 {name} must be an integer in [{minimum}, {maximum}]")
        for name, expected in (("maximum_curvature_rad_per_m", 0.2), ("route_scale_m", 120.0), ("pose_scale_m", 20.0)):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value != expected:
                raise ContractError(f"temporal v2 {name} must be exactly {expected}")
        if self.image_normalization != "imagenet_rgb_v1":
            raise ContractError("temporal v2 expects ImageNet-normalized RGB inputs")
        if self.relative_pose_convention != "past_base_in_current_base_dx_dy_yaw_rad":
            raise ContractError("temporal v2 relative pose convention is pinned")
        if not isinstance(self.time_offsets_s, tuple) or len(self.time_offsets_s) != 5:
            raise ContractError("temporal v2 time_offsets_s must be the fixed five-slot tuple")
        if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in self.time_offsets_s):
            raise ContractError("temporal v2 time_offsets_s must be finite numbers")
        if tuple(self.time_offsets_s) != NOMINAL_TIME_OFFSETS_S:
            raise ContractError("temporal v2 requires [-0.8, -0.6, -0.4, -0.2, 0.0] seconds")

    def to_dict(self) -> dict[str, Any]:
        self.validate()
        result = asdict(self)
        result["time_offsets_s"] = list(self.time_offsets_s)
        return result

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "TemporalModelConfigV2":
        if not isinstance(value, Mapping):
            raise ContractError("temporal v2 config must be an object")
        if not all(isinstance(key, str) for key in value):
            raise ContractError("temporal v2 config field names must be strings")
        known = {field.name for field in fields(cls)}
        if set(value) != known:
            raise ContractError(f"temporal v2 config fields differ: missing={sorted(known - set(value))}, unknown={sorted(set(value) - known)}")
        payload = dict(value)
        offsets = payload["time_offsets_s"]
        if not isinstance(offsets, (list, tuple)):
            raise ContractError("temporal v2 time_offsets_s must be a sequence")
        payload["time_offsets_s"] = tuple(offsets)
        result = cls(**payload)
        result.validate()
        return result


class SpatialImageEncoderV2(nn.Module):
    """HH_261001 - Preserve 6x10 per-view features, with no cross-image statistics."""

    def __init__(self, base_channels: int, token_width: int) -> None:
        super().__init__()
        widths = (base_channels, base_channels * 2, base_channels * 4, base_channels * 6)
        layers: list[nn.Module] = []
        previous = 3
        for width in widths:
            groups = min(8, width)
            while width % groups:
                groups -= 1
            layers.extend((nn.Conv2d(previous, width, 3, stride=2, padding=1, bias=False), nn.GroupNorm(groups, width), nn.SiLU()))
            previous = width
        self.features = nn.Sequential(*layers)
        # HH_261001 - 180x320 -> 12x20 after four strides; fixed pooling supports deterministic backward.
        self.pool = nn.AvgPool2d(2, stride=2)
        self.projection = nn.Conv2d(previous, token_width, 1)

    def forward(self, images: Tensor) -> Tensor:
        features = self.projection(self.pool(self.features(images)))
        return features.flatten(2).transpose(1, 2)


class QueryFusionBlockV2(nn.Module):
    """HH_261001 - Small queries attend to image/context tokens, not dense token self-attention."""

    def __init__(self, width: int, heads: int) -> None:
        super().__init__()
        self.query_norm = nn.LayerNorm(width)
        self.context_norm = nn.LayerNorm(width)
        # HH_261001 - Use sequence-first attention for the already-installed local Torch 1.8 ABI.
        self.cross_attention = nn.MultiheadAttention(width, heads, dropout=0.0)
        self.self_norm = nn.LayerNorm(width)
        self.self_attention = nn.MultiheadAttention(width, heads, dropout=0.0)
        self.ff_norm = nn.LayerNorm(width)
        self.ff = nn.Sequential(nn.Linear(width, width * 4), nn.SiLU(), nn.Linear(width * 4, width))

    def forward(self, queries: Tensor, context: Tensor, key_padding_mask: Tensor) -> Tensor:
        normalized = self.context_norm(context)
        attended, _ = self.cross_attention(self.query_norm(queries).transpose(0, 1), normalized.transpose(0, 1), normalized.transpose(0, 1), key_padding_mask=key_padding_mask, need_weights=False)
        queries = queries + attended.transpose(0, 1)
        normalized_queries = self.self_norm(queries)
        sequence_queries = normalized_queries.transpose(0, 1)
        attended, _ = self.self_attention(sequence_queries, sequence_queries, sequence_queries, need_weights=False)
        queries = queries + attended.transpose(0, 1)
        return queries + self.ff(self.ff_norm(queries))


def _embedding(input_width: int, output_width: int) -> nn.Sequential:
    return nn.Sequential(nn.Linear(input_width, output_width), nn.SiLU(), nn.Linear(output_width, output_width))


class TemporalModelV2(nn.Module):
    """HH_261001 - One identical parameterization serves current-only and causal temporal arms.

    ``time_valid`` masks absent images, rather than filling slots with copied
    current images. ``current_only`` removes past image tokens but intentionally
    preserves common 10-frame ego input and dense navigation in both arms. This
    isolates camera history, not all temporal information. Relative pose uses
    the pose of each past vehicle frame expressed in the current vehicle frame.
    """

    def __init__(self, config: TemporalModelConfigV2 | None = None) -> None:
        super().__init__()
        self.config = TemporalModelConfigV2() if config is None else config
        self.config.validate()
        cfg = self.config
        width = cfg.token_width
        self.image_encoder = SpatialImageEncoderV2(cfg.encoder_base_channels, width)
        self.camera_embedding = nn.Parameter(torch.empty(cfg.camera_count, width))
        self.image_time_embedding = nn.Parameter(torch.empty(cfg.image_history_frames, width))
        self.grid_embedding = _embedding(2, width)
        self.calibration_embedding = _embedding(cfg.calibration_features, width)
        self.pose_time_embedding = _embedding(5, width)
        self.ego_embedding = _embedding(cfg.ego_features, width)
        self.ego_time_embedding = nn.Parameter(torch.empty(cfg.ego_history_frames, width))
        self.route_embedding = _embedding(2, width)
        self.route_position_embedding = nn.Parameter(torch.empty(cfg.route_points, width))
        self.modality_embedding = nn.Parameter(torch.empty(3, width))
        self.queries = nn.Parameter(torch.empty(cfg.query_count, width))
        self.fusion = nn.ModuleList(QueryFusionBlockV2(width, cfg.attention_heads) for _ in range(cfg.cross_attention_layers))
        self.output_norm = nn.LayerNorm(width)
        self.trunk = nn.Sequential(nn.Linear(cfg.query_count * width, cfg.hidden_width), nn.SiLU(), nn.Linear(cfg.hidden_width, cfg.hidden_width), nn.SiLU())
        self.curvature_head = nn.Linear(cfg.hidden_width, cfg.spatial_knots)
        self.speed_head = nn.Linear(cfg.hidden_width, cfg.spatial_knots)
        self.intent_head = nn.Linear(cfg.hidden_width, len(INTENT_NAMES_V2))
        self.stop_distance_head = nn.Linear(cfg.hidden_width, 1)
        self.acceleration_head = nn.Linear(cfg.hidden_width, 1)
        y = (torch.arange(cfg.image_grid_height, dtype=torch.float32) + 0.5) / cfg.image_grid_height
        x = (torch.arange(cfg.image_grid_width, dtype=torch.float32) + 0.5) / cfg.image_grid_width
        grid_y = y[:, None].expand(cfg.image_grid_height, cfg.image_grid_width)
        grid_x = x[None, :].expand(cfg.image_grid_height, cfg.image_grid_width)
        self.register_buffer("image_grid_xy", torch.stack((grid_x, grid_y), dim=-1).reshape(-1, 2), persistent=True)
        for parameter in (self.camera_embedding, self.image_time_embedding, self.ego_time_embedding, self.route_position_embedding, self.modality_embedding, self.queries):
            nn.init.normal_(parameter, mean=0.0, std=0.02)

    def _validate_inputs(self, inputs: Mapping[str, Tensor], history_mode: str) -> int:
        if not isinstance(history_mode, str) or history_mode not in HISTORY_MODES:
            raise ContractError(f"history_mode must be one of {sorted(HISTORY_MODES)}")
        images = inputs["images"]
        if not isinstance(images, Tensor) or images.ndim != 6 or images.shape[0] < 1:
            raise ContractError("images must have [B,5,6,3,180,320] with B >= 1")
        batch = images.shape[0]
        cfg = self.config
        shapes = {
            "images": (batch, 5, 6, 3, 180, 320), "calibration": (batch, 6, 16),
            "ego_history": (batch, 10, 13), "ego_history_mask": (batch, 10),
            "route_xy": (batch, 128, 2), "route_mask": (batch, 128),
            "time_valid": (batch, 5), "relative_pose": (batch, 5, 3),
            "time_offsets_s": (batch, 5),
        }
        masks = frozenset(("ego_history_mask", "route_mask", "time_valid"))
        reference = self.queries
        for name, value in inputs.items():
            if not isinstance(value, Tensor) or tuple(value.shape) != shapes[name]:
                raise ContractError(f"{name} must have shape {shapes[name]}")
            if value.device != reference.device:
                raise ContractError(f"{name} must be on the model device")
            if name in masks:
                if value.dtype != torch.bool:
                    raise ContractError(f"{name} must use boolean validity, not numeric weights")
            else:
                if not value.is_floating_point() or value.dtype != reference.dtype:
                    raise ContractError(f"{name} must have the model floating dtype")
                if not bool(torch.isfinite(value).all().item()):
                    raise ContractError(f"{name} must contain only finite values, including masked slots")
        if not bool(inputs["time_valid"][:, -1].all().item()):
            raise ContractError("current camera time must be valid")
        if not bool(inputs["ego_history_mask"][:, -1].all().item()):
            raise ContractError("current ego state must be valid")
        route_mask = inputs["route_mask"]
        if not bool(route_mask[:, :2].all().item()) or bool((route_mask[:, 1:] & ~route_mask[:, :-1]).any().item()):
            raise ContractError("dense route validity must be a prefix containing at least two points")
        offsets = inputs["time_offsets_s"]
        if bool((offsets > 0.0).any().item()):
            raise ContractError("future camera times are forbidden")
        if not bool((offsets[:, 1:] > offsets[:, :-1]).all().item()):
            raise ContractError("camera times must increase strictly to the current frame")
        expected = offsets.new_tensor(cfg.time_offsets_s).expand_as(offsets)
        if not bool(torch.isclose(offsets, expected, atol=3.0e-6, rtol=0.0).all().item()):
            raise ContractError("camera time offsets do not match the pinned five causal slots")
        if not bool((inputs["relative_pose"][:, -1].abs() <= 1.0e-6).all().item()):
            raise ContractError("current relative pose must be zero in the current vehicle frame")
        valid_yaw = inputs["relative_pose"][..., 2][inputs["time_valid"]]
        if bool((valid_yaw.abs() > math.pi + 1.0e-6).any().item()):
            raise ContractError("valid relative yaw must be radians wrapped to [-pi, pi]")
        return batch

    def forward(
        self,
        images: Tensor,
        calibration: Tensor,
        ego_history: Tensor,
        ego_history_mask: Tensor,
        route_xy: Tensor,
        route_mask: Tensor,
        time_valid: Tensor,
        relative_pose: Tensor,
        time_offsets_s: Tensor,
        *,
        history_mode: str = "temporal",
    ) -> dict[str, Tensor]:
        inputs = {"images": images, "calibration": calibration, "ego_history": ego_history,
                  "ego_history_mask": ego_history_mask, "route_xy": route_xy, "route_mask": route_mask,
                  "time_valid": time_valid, "relative_pose": relative_pose, "time_offsets_s": time_offsets_s}
        batch = self._validate_inputs(inputs, history_mode)
        cfg = self.config
        effective_valid = time_valid.clone()
        if history_mode == "current_only":
            effective_valid[:, :-1] = False
        # HH_261001 - Zero absent images before encoding, and mask their keys; no artificial copied history.
        masked_images = images.masked_fill(~effective_valid[:, :, None, None, None, None], 0.0)
        encoded = self.image_encoder(masked_images.reshape(-1, 3, 180, 320))
        spatial = encoded.reshape(batch, 5, 6, 60, cfg.token_width)
        grid_features = self.grid_embedding(self.image_grid_xy)
        pose = relative_pose.masked_fill(~effective_valid[..., None], 0.0)
        pose_features = torch.cat((pose[..., :2] / cfg.pose_scale_m, torch.sin(pose[..., 2:3]), torch.cos(pose[..., 2:3]), time_offsets_s[..., None]), dim=-1)
        spatial = (spatial + grid_features[None, None, None, :, :] + self.camera_embedding[None, None, :, None, :]
                   + self.image_time_embedding[None, :, None, None, :] + self.calibration_embedding(calibration)[:, None, :, None, :]
                   + self.pose_time_embedding(pose_features)[:, :, None, None, :] + self.modality_embedding[0])
        camera_valid = effective_valid[:, :, None, None].expand(batch, 5, 6, 60).reshape(batch, -1)
        camera_tokens = spatial.reshape(batch, -1, cfg.token_width).masked_fill(~camera_valid[..., None], 0.0)
        ego_tokens = self.ego_embedding(ego_history.masked_fill(~ego_history_mask[..., None], 0.0)) + self.ego_time_embedding[None] + self.modality_embedding[1]
        ego_tokens = ego_tokens.masked_fill(~ego_history_mask[..., None], 0.0)
        route_tokens = self.route_embedding(route_xy.masked_fill(~route_mask[..., None], 0.0) / cfg.route_scale_m) + self.route_position_embedding[None] + self.modality_embedding[2]
        route_tokens = route_tokens.masked_fill(~route_mask[..., None], 0.0)
        context = torch.cat((camera_tokens, ego_tokens, route_tokens), dim=1)
        validity = torch.cat((camera_valid, ego_history_mask, route_mask), dim=1)
        queries = self.queries[None].expand(batch, -1, -1)
        for block in self.fusion:
            queries = block(queries, context, ~validity)
        hidden = self.trunk(self.output_norm(queries).flatten(1))
        # HH_261001 - These are raw training outputs, not a physically validated runtime trajectory.
        result = {
            "curvature_knots": cfg.maximum_curvature_rad_per_m * torch.tanh(self.curvature_head(hidden)),
            "preferred_speed_knots": F.softplus(self.speed_head(hidden)),
            "intent_logits": self.intent_head(hidden),
            "stop_distance_raw": self.stop_distance_head(hidden),
            "acceleration_prediction": self.acceleration_head(hidden),
        }
        if not all(bool(torch.isfinite(value).all().item()) for value in result.values()):
            raise ContractError("temporal v2 raw neural output contains NaN or Inf")
        return result


def parameter_count(model: nn.Module) -> int:
    """HH_261001 - Count learned tensors without treating token buffers as parameters."""
    return sum(parameter.numel() for parameter in model.parameters())
