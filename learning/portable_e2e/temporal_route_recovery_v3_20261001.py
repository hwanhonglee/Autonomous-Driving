"""HH_261001 - Add a learned causal navigation-to-curvature bypass without replacing neural control.

This NEW research architecture retains the complete frozen temporal backbone
state ABI and adds four route_recovery_branch tensors. It requires actual
off-center recovery TRAIN data; construction, tests, or zero initialization do
not prove a driving capability. No teacher future, map/world oracle, selected
trajectory substitution, extra sensor input, or runtime promotion is included.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
from pathlib import Path
from typing import Mapping

import torch
from torch import Tensor, nn

from .contract import ContractError
from .temporal_cached_encoder_v2 import CachedTemporalModelV2, TemporalCacheWindowV2
from .temporal_cpu_finite_validation_v2_20261001 import CPUFiniteCachedTemporalModelV2
from .temporal_model_v2 import TemporalModelConfigV2

MODEL_ID = "portable_e2e.dense_nav.learned_route_recovery.v3.research"
SCHEMA = "portable_e2e.temporal_route_recovery_model.20261001.v3"
EXPORT_SCHEMA = "portable_e2e.temporal_route_recovery_export_research.20261001.v3"
PARENT_SOURCE_PINS = {
    "temporal_model_v2.py": "93f709c6c229e705899fcf1778f9dd253ea867e88a6f237ad3f0fb8efdbbf435",
    "temporal_cached_encoder_v2.py": "55ef7371e0a1f4ec2b9cb854282e4d51b6c406b055ec5eceb8e286737f03adff",
    "temporal_cpu_finite_validation_v2_20261001.py": "ecc6f7af5e8ee8dcbbc3a656c50e7de474f7a7bfa1e1eca630af40faa59e16c5",
}
BRANCH_PREFIX = "route_recovery_branch."


def require(condition, message):
    if not condition:
        raise ContractError(message)


@dataclass(frozen=True)
class RouteRecoveryConfigV3:
    """HH_261001 - Explicit strong near-route scale, not the backbone's 120 m token normalization."""
    stations_forward_x_m: tuple[float, ...] = (2.0, 4.0, 8.0)
    lateral_scale_m: float = 1.0
    tangent_scale_rad: float = 0.1
    hidden_width: int = 32
    feature_count: int = 9
    minimum_forward_segment_dx_m: float = 1e-6
    support_policy: str = "first_valid_forward_x_crossing_no_extrapolation"
    feature_order: str = "station_major_signed_y_tangent_support"
    residual_site: str = "curvature_linear_logit_before_original_point2_tanh"

    def validate(self):
        require(isinstance(self.stations_forward_x_m, tuple) and len(self.stations_forward_x_m) == 3
                and all(type(value) is float for value in self.stations_forward_x_m)
                and all(type(getattr(self, name)) is float for name in
                        ("lateral_scale_m", "tangent_scale_rad", "minimum_forward_segment_dx_m")),
                "strict float recovery station/scale values required, never booleans")
        require(all(type(getattr(self, name)) is str for name in ("support_policy", "feature_order", "residual_site")),
                "strict string recovery semantics required")
        require(self == RouteRecoveryConfigV3(), "exact declared route recovery architecture required")
        require(type(self.hidden_width) is int and type(self.feature_count) is int,
                "strict integer recovery widths required")

    def to_dict(self):
        self.validate()
        result = asdict(self); result["stations_forward_x_m"] = list(self.stations_forward_x_m)
        return result

    @classmethod
    def from_mapping(cls, value):
        require(isinstance(value, Mapping) and set(value) == set(asdict(cls())), "complete versioned recovery config required")
        values = dict(value); values["stations_forward_x_m"] = tuple(values["stations_forward_x_m"])
        result = cls(**values); result.validate(); return result


def causal_near_route_features(route_xy: Tensor, route_mask: Tensor, config: RouteRecoveryConfigV3 | None = None):
    """HH_261001 - Use ego-frame navigation only: signed y, tangent and honest support at x=2/4/8 m.

    For each station use the FIRST valid forward segment bracketing its x. A
    missing crossing is zero-valued with support=0; never extrapolate, invent a
    future waypoint, recenter y, or inspect true trajectory/CTE labels. Tangent
    is atan2(dy,dx), not a world heading or simulator ground truth.
    """
    config = RouteRecoveryConfigV3() if config is None else config; config.validate()
    require(isinstance(route_xy, Tensor) and route_xy.ndim == 3 and tuple(route_xy.shape[1:]) == (128, 2)
            and route_xy.shape[0] > 0 and route_xy.dtype == torch.float32,
            "recovery route must be float32 [B,128,2]")
    require(isinstance(route_mask, Tensor) and route_mask.dtype == torch.bool
            and tuple(route_mask.shape) == tuple(route_xy.shape[:2]) and route_mask.device == route_xy.device,
            "recovery route mask/device ABI differs")
    require(bool(torch.isfinite(route_xy).all().item()), "recovery route must be finite including masked values")
    require(bool(route_mask[:, :2].all().item()) and not bool((route_mask[:, 1:] & ~route_mask[:, :-1]).any().item()),
            "recovery route support must be a prefix containing at least two points")
    start, end = route_xy[:, :-1], route_xy[:, 1:]
    dx, dy = end[..., 0] - start[..., 0], end[..., 1] - start[..., 1]
    valid_segments = route_mask[:, :-1] & route_mask[:, 1:] & (dx > config.minimum_forward_segment_dx_m)
    features = []
    for station in config.stations_forward_x_m:
        crossing = valid_segments & (start[..., 0] <= station) & (end[..., 0] >= station)
        supported = crossing.any(dim=1)
        index = crossing.to(torch.int64).argmax(dim=1, keepdim=True)
        x0 = start[..., 0].gather(1, index).squeeze(1)
        y0 = start[..., 1].gather(1, index).squeeze(1)
        sx = dx.gather(1, index).squeeze(1); sy = dy.gather(1, index).squeeze(1)
        # HH_261001 - A false station uses safe arithmetic and a false mask, never a short-path extrapolation.
        safe_dx = torch.where(supported, sx, torch.ones_like(sx))
        safe_dy = torch.where(supported, sy, torch.zeros_like(sy))
        alpha = torch.where(supported, (station - x0) / safe_dx, torch.zeros_like(sx))
        lateral = torch.where(supported, y0 + alpha * safe_dy, torch.zeros_like(y0))
        tangent = torch.where(supported, torch.atan2(safe_dy, safe_dx), torch.zeros_like(safe_dx))
        features.extend((lateral / config.lateral_scale_m, tangent / config.tangent_scale_rad, supported.to(route_xy.dtype)))
    result = torch.stack(features, dim=1)
    require(tuple(result.shape) == (route_xy.shape[0], 9) and bool(torch.isfinite(result).all().item()),
            "causal recovery features contain invalid arithmetic")
    return result


class RouteRecoveryTemporalModelV3(CachedTemporalModelV2):
    """HH_261001 - New explicit model: one backbone pass plus a zero-initialized learned logit correction.

    The permanent hook belongs ONLY to this new architecture. It never mutates
    an existing V2 instance. Both ordinary and cached APIs own a serial context;
    the frozen cache calls its own super.forward, so both APIs must set/clear
    that context. Entire added weights, .data changes, hook identities, camera
    bytes and all provenance remain covered by the inherited cache safeguards.
    """
    def __init__(self, config: TemporalModelConfigV2 | None = None,
                 recovery_config: RouteRecoveryConfigV3 | None = None):
        folder = Path(__file__).parent
        require(all(hashlib.sha256((folder / name).read_bytes()).hexdigest() == sha
                    for name, sha in PARENT_SOURCE_PINS.items()), "recovery requires frozen parent/cache/CPU validator sources")
        super().__init__(config)
        self.recovery_config = RouteRecoveryConfigV3() if recovery_config is None else recovery_config
        self.recovery_config.validate()
        self.model_id, self.architecture_schema = MODEL_ID, SCHEMA
        self._backbone_state_keys = frozenset(self.state_dict())
        self.route_recovery_branch = nn.Sequential(nn.Linear(9, 32), nn.SiLU(), nn.Linear(32, self.config.spatial_knots))
        nn.init.zeros_(self.route_recovery_branch[-1].weight); nn.init.zeros_(self.route_recovery_branch[-1].bias)
        self._recovery_active = False
        self._recovery_features = None
        self._recovery_hook_function = self._add_route_recovery_logits
        self._recovery_hook_handle = self.curvature_head.register_forward_hook(self._recovery_hook_function)

    def _validate_inputs(self, inputs: Mapping[str, Tensor], history_mode: str):
        require(self.queries.dtype == torch.float32, "recovery model supports the declared float32 ABI only")
        if self.queries.device.type == "cpu":
            batch = CPUFiniteCachedTemporalModelV2._validate_inputs(self, inputs, history_mode)
        else:
            batch = super()._validate_inputs(inputs, history_mode)
        if self._recovery_active:
            # HH_261001 - Only this 1 KiB route branch is checked here; no third full image finite validation is added.
            self._recovery_features = causal_near_route_features(inputs["route_xy"], inputs["route_mask"], self.recovery_config)
        return batch

    def _add_route_recovery_logits(self, module, arguments, logits):
        require(self._recovery_active and isinstance(self._recovery_features, Tensor),
                "learned recovery curvature head requires a validated serial model call")
        features = self._recovery_features
        require(tuple(features.shape) == (logits.shape[0], 9) and features.device == logits.device,
                "recovery context/logit batch differs")
        residual = self.route_recovery_branch(features)
        supported = (features[:, 2::3] > .5).any(dim=1)
        residual = residual * supported[:, None].to(residual.dtype)
        require(tuple(residual.shape) == tuple(logits.shape) and bool(torch.isfinite(residual).all().item()),
                "learned recovery residual must be finite")
        return logits + residual

    def _begin_recovery_call(self):
        require(not self._recovery_active, "recovery model calls are serial and not reentrant")
        require(set(self.curvature_head._forward_hooks) == {self._recovery_hook_handle.id}
                and self.curvature_head._forward_hooks[self._recovery_hook_handle.id] is self._recovery_hook_function,
                "own permanent recovery logit hook is missing/replaced or extra head hooks were added")
        self._recovery_active = True
        self._recovery_features = None

    def _end_recovery_call(self):
        self._recovery_features = None
        self._recovery_active = False

    def forward(self, images, calibration, ego_history, ego_history_mask, route_xy, route_mask,
                time_valid, relative_pose, time_offsets_s, *, history_mode="temporal"):
        self._begin_recovery_call()
        try:
            return super().forward(images, calibration, ego_history, ego_history_mask, route_xy, route_mask,
                                   time_valid, relative_pose, time_offsets_s, history_mode=history_mode)
        finally:
            self._end_recovery_call()

    def forward_cached(self, inputs, *, window: TemporalCacheWindowV2, history_mode="temporal"):
        self._begin_recovery_call()
        try:
            return super().forward_cached(inputs, window=window, history_mode=history_mode)
        finally:
            self._end_recovery_call()

    def load_backbone_state(self, state):
        """HH_261001 - Explicit warm-start ABI; a full V3 checkpoint instead uses strict inherited load_state_dict."""
        require(not self._recovery_active and isinstance(state, Mapping) and set(state) == self._backbone_state_keys,
                "complete original backbone-only warm start required")
        require(bool((self.route_recovery_branch[-1].weight == 0).all().item())
                and bool((self.route_recovery_branch[-1].bias == 0).all().item()),
                "backbone warm start is allowed only with a zero-output new branch")
        target = self.state_dict()
        for name, value in state.items():
            require(isinstance(value, Tensor) and value.dtype == torch.float32
                    and tuple(value.shape) == tuple(target[name].shape) and bool(torch.isfinite(value).all().item()),
                    "finite original backbone tensor ABI differs: " + name)
        target.update(state)
        return self.load_state_dict(target, strict=True)

    def training_interface(self):
        """HH_261001 - Trainer/export contract, not permission to invent a recovery cohort or run a fit."""
        return dict(schema=SCHEMA, model_id=MODEL_ID, export_schema=EXPORT_SCHEMA,
            backbone_config=self.config.to_dict(), recovery_config=self.recovery_config.to_dict(),
            input_abi="unchanged nine causal TemporalModelV2 fields", required_recovery_training_data=True,
            actual_measured_v0_required=True, true_future_labels_loss_only=True,
            recovery_strata="signed lateral displacement x signed heading error, both signs plus centered legacy replay",
            recovery_scenario_GO_not_STOP_training=True, existing_legacy_true_speed_raw_only=True,
            current_speed_not_clipped=True, zero_initialized_branch_is_not_a_trained_capability=True,
            curvature_bound_inv_m=self.config.maximum_curvature_rad_per_m,
            original_speed_intent_stop_acceleration_heads_unchanged=True,
            V2_numeric_loader_compatible=False, inference_loader_authorized=False,
            simulator_control_authorized=False, vehicle_control_approved=False, model_promoted=False)
