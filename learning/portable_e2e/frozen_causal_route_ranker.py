"""HH_260906 - Fit a minimal causal route-cost ranker on immutable F candidates."""

from __future__ import annotations

import copy
from dataclasses import dataclass, fields
import hashlib
import json
import math
from typing import Any, Callable, Mapping, Sequence

import torch
from torch import Tensor, nn
from torch.nn import functional as F

from . import frozen_stopmix_drive_rank as projected
from . import frozen_stopmix_selector as cache_core
from . import frozen_stopmix_selector_recalibration as h_core
from .model import PHYSICAL_MAXIMUM_SPEED_MPS
from .torch_dataset import _linear_route_samples
from .train import _nested_tensors_are_finite


ARTIFACT_ID = "portable_e2e.frozen_f_causal_route_ranker.v1"
SOURCE_CHECKPOINT_SHA256 = h_core.SOURCE_CHECKPOINT_SHA256
RAW_DATASET_SHA256 = (
    "70fceaf0a6fea1b6ac455154bdc7b1113cfef15abc1a26ca7327ac2a186799e6"
)
SEED = 20260906
TRAIN_COUNT, DEV_COUNT, TOTAL_COUNT = 1475, 665, 2140
EXPECTED_TRAIN_DRIVE_COUNT, EXPECTED_TRAIN_STOP_COUNT = 1232, 243
DRIVE_CANDIDATE_COUNT, TOTAL_CANDIDATE_COUNT = 6, 12
ROUTE_POINT_COUNT, FUTURE_POINT_COUNT = 128, 64
FEATURE_NAMES = ("lateral", "heading", "speed", "curvature")
ROUTE_GROUPS = ("town07_same_physical_route", "c_track", "town01")
COMMAND_NAMES = (
    "left",
    "right",
    "straight",
    "lane_follow",
    "change_left",
    "change_right",
)
TURN_COMMANDS = (0, 1, 4, 5)
STRAIGHT_COMMANDS = (2, 3)
STRAIGHT_HEADING_MAXIMUM_RAD = 0.10
TURN_LATCH_HEADING_MINIMUM_RAD = 0.35
FIT_STEPS, LEARNING_RATE, MAX_GRAD_NORM = 256, 0.025, 10.0
WEIGHT_INITIAL_VALUE = 1.0
WEIGHT_COUNT = 4
CachedStopmixSelectorData = cache_core.CachedStopmixSelectorData
_TENSOR_FIELDS = (
    "route_xy",
    "route_mask",
    "ego_speed_mps",
    "ego_acceleration_mps2",
    "command",
    "straight_ranker_mask",
    "route_heading_change_rad",
)
_require, _check = cache_core._require, cache_core._check
_device, _digest_tensors = cache_core._device, cache_core._digest_tensors


def _finite_number(value: Any, context: str) -> float:
    """HH_260906 - Normalize one finite non-boolean scalar or fail closed."""
    _require(
        not isinstance(value, bool) and isinstance(value, (int, float)),
        context + " must be numeric",
    )
    try:
        result = float(value)
    except (OverflowError, TypeError, ValueError) as error:
        raise cache_core.ContractError(context + " must be finite") from error
    _require(math.isfinite(result), context + " must be finite")
    return result


def _finite_vector(value: Any, count: int, context: str) -> tuple[float, ...]:
    """HH_260906 - Admit one exact-length finite vector without coercing mappings."""
    _require(
        isinstance(value, (list, tuple)) and len(value) == count,
        context + " has the wrong vector length",
    )
    return tuple(
        _finite_number(item, f"{context}[{index}]")
        for index, item in enumerate(value)
    )


def extract_causal_row(sample: Mapping[str, Any]) -> dict[str, Any]:
    """HH_260906 - Extract only current causal ego and navigation fields from one raw row."""
    _require(isinstance(sample, Mapping), "raw sample mapping required")
    sample_id = sample.get("sample_id")
    anchor = sample.get("anchor_timestamp_ns")
    ego = sample.get("ego")
    navigation = sample.get("navigation")
    _require(
        isinstance(sample_id, str)
        and bool(sample_id)
        and type(anchor) is int
        and anchor >= 0
        and isinstance(ego, Mapping)
        and isinstance(navigation, Mapping),
        "causal sample identity, anchor, ego and navigation required",
    )
    ego_time = ego.get("timestamp_ns")
    command_time = navigation.get("command_timestamp_ns")
    route_time = navigation.get("route_source_timestamp_ns")
    command = navigation.get("command")
    _require(
        type(ego_time) is int
        and 0 <= ego_time <= anchor
        and type(command_time) is int
        and 0 <= command_time <= anchor
        and type(route_time) is int
        and 0 <= route_time <= anchor,
        "future or invalid ego/navigation/route timestamp",
    )
    _require(
        type(command) is int and 0 <= command < len(COMMAND_NAMES),
        "navigation command must follow the fixed six-command contract",
    )
    velocity = _finite_vector(
        ego.get("linear_velocity_base_mps"),
        3,
        "ego.linear_velocity_base_mps",
    )
    acceleration = _finite_vector(
        ego.get("linear_acceleration_base_mps2"),
        3,
        "ego.linear_acceleration_base_mps2",
    )
    raw_route = navigation.get("route_polyline_base_m")
    _require(
        isinstance(raw_route, (list, tuple)) and len(raw_route) >= 2,
        "causal route polyline with at least two points required",
    )
    raw_points = tuple(
        _finite_vector(point, 2, f"navigation.route_polyline_base_m[{index}]")
        for index, point in enumerate(raw_route)
    )
    route = _linear_route_samples(raw_points, limit=ROUTE_POINT_COUNT)
    _require(2 <= len(route) <= ROUTE_POINT_COUNT, "resampled route length differs")
    first_heading = math.atan2(
        route[1][1] - route[0][1], route[1][0] - route[0][0]
    )
    last_heading = math.atan2(
        route[-1][1] - route[-2][1], route[-1][0] - route[-2][0]
    )
    heading_change = abs(
        math.atan2(
            math.sin(last_heading - first_heading),
            math.cos(last_heading - first_heading),
        )
    )
    return {
        "sample_id": sample_id,
        "route_xy": route,
        "ego_speed_mps": math.hypot(velocity[0], velocity[1]),
        "ego_acceleration_mps2": acceleration[0],
        "command": command,
        "route_heading_change_rad": heading_change,
        "causality": {
            "anchor_timestamp_ns": anchor,
            "ego_timestamp_ns": ego_time,
            "command_timestamp_ns": command_time,
            "route_source_timestamp_ns": route_time,
        },
    }


def causal_straight_decision(
    command: int, route_heading_change_rad: float, turn_latched_before: bool
) -> tuple[bool, bool, str]:
    """HH_260906 - Apply the predeclared confidence margin and causal turn latch."""
    _require(
        type(command) is int
        and 0 <= command < len(COMMAND_NAMES)
        and type(turn_latched_before) is bool,
        "fixed command and boolean causal latch required",
    )
    change = _finite_number(route_heading_change_rad, "route heading change")
    _require(0.0 <= change <= math.pi, "route heading change is outside [0,pi]")
    turn_evidence = command in TURN_COMMANDS or change >= TURN_LATCH_HEADING_MINIMUM_RAD
    turn_latched_after = turn_latched_before or turn_evidence
    if turn_latched_after:
        return False, True, "TURN_EVIDENCE_OR_CAUSAL_LATCH_F_FALLBACK"
    if command not in STRAIGHT_COMMANDS:
        return False, False, "NON_STRAIGHT_COMMAND_F_FALLBACK"
    if change > STRAIGHT_HEADING_MAXIMUM_RAD:
        return False, False, "ROUTE_INTENT_CONFIDENCE_MARGIN_F_FALLBACK"
    return True, False, "CONFIDENT_CAUSAL_STRAIGHT_ROUTE_RANKER"


@dataclass(frozen=True)
class CausalRouteSidecar:
    """HH_260906 - Hold immutable input-only route features plus non-input fold metadata."""

    split: str
    sample_ids: tuple[str, ...]
    route_groups: tuple[str, ...]
    route_xy: Tensor
    route_mask: Tensor
    ego_speed_mps: Tensor
    ego_acceleration_mps2: Tensor
    command: Tensor
    straight_ranker_mask: Tensor
    route_heading_change_rad: Tensor
    source_dataset_sha256: str = RAW_DATASET_SHA256

    def __post_init__(self) -> None:
        """HH_260906 - Clone every tensor and enforce the no-label fixed sidecar ABI."""
        _require(self.split in ("train", "val"), "TRAIN or DEV sidecar required")
        _require(
            isinstance(self.source_dataset_sha256, str)
            and len(self.source_dataset_sha256) == 64
            and all(
                character in "0123456789abcdef"
                for character in self.source_dataset_sha256
            ),
            "sidecar source dataset SHA256 required",
        )
        count = len(self.sample_ids)
        _require(
            count > 0
            and len(set(self.sample_ids)) == count
            and len(self.route_groups) == count
            and all(
                isinstance(value, str) and bool(value) for value in self.sample_ids
            )
            and all(
                isinstance(value, str) and bool(value) for value in self.route_groups
            ),
            "aligned unique sample identities and route-group metadata required",
        )
        object.__setattr__(self, "sample_ids", tuple(self.sample_ids))
        object.__setattr__(self, "route_groups", tuple(self.route_groups))
        shapes = {
            "route_xy": (count, ROUTE_POINT_COUNT, 2),
            "route_mask": (count, ROUTE_POINT_COUNT),
            "ego_speed_mps": (count,),
            "ego_acceleration_mps2": (count,),
            "command": (count,),
            "straight_ranker_mask": (count,),
            "route_heading_change_rad": (count,),
        }
        dtypes = {
            "route_xy": torch.float32,
            "route_mask": torch.bool,
            "ego_speed_mps": torch.float32,
            "ego_acceleration_mps2": torch.float32,
            "command": torch.int64,
            "straight_ranker_mask": torch.bool,
            "route_heading_change_rad": torch.float32,
        }
        device = None
        for name in _TENSOR_FIELDS:
            value = getattr(self, name)
            _require(
                isinstance(value, Tensor)
                and tuple(value.shape) == shapes[name]
                and value.dtype == dtypes[name]
                and bool(torch.isfinite(value).all()),
                name + " violates the fixed causal sidecar ABI",
            )
            if device is None:
                device = _device(value.device)
            _require(value.device == device, "sidecar tensors must share one device")
            object.__setattr__(self, name, value.detach().clone().contiguous())
        _require(
            bool((self.ego_speed_mps >= 0.0).all())
            and bool((self.command >= 0).all())
            and bool((self.command < len(COMMAND_NAMES)).all())
            and bool((self.route_heading_change_rad >= 0.0).all())
            and bool((self.route_heading_change_rad <= math.pi).all()),
            "sidecar physical or command range differs",
        )
        _require(
            bool((self.route_mask.sum(dim=1) >= 2).all())
            and not bool((~self.route_mask[:, :-1] & self.route_mask[:, 1:]).any()),
            "route mask must be a contiguous valid prefix of at least two points",
        )

    def __len__(self) -> int:
        """HH_260906 - Return the aligned sidecar row count."""
        return len(self.sample_ids)

    def digest(self) -> str:
        """HH_260906 - Bind sidecar metadata and all input tensors to one digest."""
        metadata = {
            item.name: getattr(self, item.name)
            for item in fields(self)
            if item.name not in _TENSOR_FIELDS
        }
        tensor_sha = _digest_tensors(
            (name, getattr(self, name)) for name in _TENSOR_FIELDS
        )
        encoded = json.dumps(metadata, sort_keys=True, separators=(",", ":"))
        return hashlib.sha256((encoded + "\0" + tensor_sha).encode()).hexdigest()

    def to(self, device: str | torch.device) -> CausalRouteSidecar:
        """HH_260906 - Copy only sidecar tensors to one permitted device."""
        device = _device(device)
        if self.route_xy.device == device:
            return self
        values = {item.name: getattr(self, item.name) for item in fields(self)}
        values.update(
            {name: getattr(self, name).to(device) for name in _TENSOR_FIELDS}
        )
        return CausalRouteSidecar(**values)


def make_sidecar(
    split: str,
    sample_ids: Sequence[str],
    rows_by_id: Mapping[str, Mapping[str, Any]],
    route_group_by_id: Mapping[str, str],
    straight_ranker_by_id: Mapping[str, bool],
) -> CausalRouteSidecar:
    """HH_260906 - Align prevalidated causal rows exactly to immutable H cache IDs."""
    ids = tuple(sample_ids)
    _require(
        len(ids) > 0
        and len(set(ids)) == len(ids)
        and set(rows_by_id) == set(ids)
        and set(route_group_by_id) == set(ids)
        and set(straight_ranker_by_id) == set(ids),
        "raw causal rows must cover each cache sample exactly once",
    )
    rows = [rows_by_id[sample_id] for sample_id in ids]
    allowed = {
        "sample_id",
        "route_xy",
        "ego_speed_mps",
        "ego_acceleration_mps2",
        "command",
        "route_heading_change_rad",
        "causality",
    }
    _require(
        all(set(row) == allowed and row["sample_id"] == sample_id for row, sample_id in zip(rows, ids)),
        "causal row field allowlist or sample alignment differs",
    )
    route_xy = torch.zeros(
        (len(rows), ROUTE_POINT_COUNT, 2), dtype=torch.float32
    )
    route_mask = torch.zeros(
        (len(rows), ROUTE_POINT_COUNT), dtype=torch.bool
    )
    for index, row in enumerate(rows):
        count = len(row["route_xy"])
        _require(2 <= count <= ROUTE_POINT_COUNT, "resampled route length differs")
        route_xy[index, :count] = torch.tensor(row["route_xy"], dtype=torch.float32)
        route_mask[index, :count] = True
    return CausalRouteSidecar(
        split=split,
        sample_ids=ids,
        route_groups=tuple(route_group_by_id[sample_id] for sample_id in ids),
        route_xy=route_xy,
        route_mask=route_mask,
        ego_speed_mps=torch.tensor(
            [row["ego_speed_mps"] for row in rows], dtype=torch.float32
        ),
        ego_acceleration_mps2=torch.tensor(
            [row["ego_acceleration_mps2"] for row in rows], dtype=torch.float32
        ),
        command=torch.tensor(
            [row["command"] for row in rows], dtype=torch.int64
        ),
        straight_ranker_mask=torch.tensor(
            [straight_ranker_by_id[sample_id] for sample_id in ids],
            dtype=torch.bool,
        ),
        route_heading_change_rad=torch.tensor(
            [row["route_heading_change_rad"] for row in rows],
            dtype=torch.float32,
        ),
    )


def validate_pair_and_sidecars(
    train: CachedStopmixSelectorData,
    dev: CachedStopmixSelectorData,
    train_sidecar: CausalRouteSidecar,
    dev_sidecar: CausalRouteSidecar,
) -> None:
    """HH_260906 - Bind the exact H cache pair to complete input-only raw sidecars."""
    h_core.validate_pair(train, dev)
    _require(
        type(train_sidecar) is CausalRouteSidecar
        and type(dev_sidecar) is CausalRouteSidecar
        and train_sidecar.split == "train"
        and dev_sidecar.split == "val",
        "TRAIN and DEV causal sidecars required",
    )
    _require(
        tuple(train.sample_ids) == train_sidecar.sample_ids
        and tuple(dev.sample_ids) == dev_sidecar.sample_ids
        and len(train_sidecar) == TRAIN_COUNT
        and len(dev_sidecar) == DEV_COUNT
        and len(train_sidecar) + len(dev_sidecar) == TOTAL_COUNT,
        "sidecar/cache sample ID order or 2140-row coverage differs",
    )
    _require(
        train_sidecar.source_dataset_sha256
        == dev_sidecar.source_dataset_sha256
        == RAW_DATASET_SHA256,
        "exact raw prepared corpus dataset.json SHA256 required",
    )
    _require(
        set(train_sidecar.route_groups) == set(ROUTE_GROUPS)
        and not (set(train_sidecar.sample_ids) & set(dev_sidecar.sample_ids)),
        "three fixed TRAIN route groups and disjoint DEV identities required",
    )


def _wrap_angle(value: Tensor) -> Tensor:
    """HH_260906 - Wrap tensor angles into the principal interval deterministically."""
    return torch.atan2(torch.sin(value), torch.cos(value))


def candidate_route_costs(
    route_xy: Tensor,
    route_mask: Tensor,
    ego_speed_mps: Tensor,
    ego_acceleration_mps2: Tensor,
    candidate_xy: Tensor,
    candidate_speed: Tensor,
) -> Tensor:
    """HH_260906 - Compute four shared causal alignment costs without labels or logits."""
    values = (
        route_xy,
        route_mask,
        ego_speed_mps,
        ego_acceleration_mps2,
        candidate_xy,
        candidate_speed,
    )
    count = len(route_xy)
    _require(
        count > 0
        and all(isinstance(value, Tensor) for value in values)
        and tuple(route_xy.shape) == (count, ROUTE_POINT_COUNT, 2)
        and tuple(route_mask.shape) == (count, ROUTE_POINT_COUNT)
        and tuple(ego_speed_mps.shape) == (count,)
        and tuple(ego_acceleration_mps2.shape) == (count,)
        and tuple(candidate_xy.shape)
        == (count, DRIVE_CANDIDATE_COUNT, FUTURE_POINT_COUNT, 2)
        and tuple(candidate_speed.shape)
        == (count, DRIVE_CANDIDATE_COUNT, FUTURE_POINT_COUNT),
        "fixed route, ego and DRIVE6 candidate shapes required",
    )
    dtype, device = route_xy.dtype, route_xy.device
    _require(
        dtype in (torch.float32, torch.float64)
        and route_mask.dtype == torch.bool
        and route_mask.device == device
        and all(
            value.dtype == dtype
            and value.device == device
            and bool(torch.isfinite(value).all())
            for value in (
                route_xy,
                ego_speed_mps,
                ego_acceleration_mps2,
                candidate_xy,
                candidate_speed,
            )
        )
        and bool((ego_speed_mps >= 0.0).all())
        and bool((candidate_speed >= 0.0).all()),
        "finite same-device route geometry, ego state and candidates required",
    )
    _require(
        bool((route_mask.sum(dim=1) >= 2).all())
        and not bool((~route_mask[:, :-1] & route_mask[:, 1:]).any()),
        "route mask must be a contiguous valid prefix",
    )
    _device(device)
    route = route_xy.detach()
    xy = candidate_xy.detach()
    speed = candidate_speed.detach()
    route_delta = route[:, 1:] - route[:, :-1]
    route_step = torch.norm(route_delta, dim=-1)
    segment_valid = route_mask[:, 1:] & route_mask[:, :-1]
    _require(
        bool((route_step[segment_valid] > 1.0e-6).all()),
        "valid route contains a zero-length segment",
    )
    valid_step = torch.where(segment_valid, route_step, torch.zeros_like(route_step))
    route_arc = torch.cat(
        (
            torch.zeros((count, 1), dtype=dtype, device=device),
            valid_step.cumsum(dim=1),
        ),
        dim=1,
    )
    origin = torch.zeros(
        (count, DRIVE_CANDIDATE_COUNT, 1, 2), dtype=dtype, device=device
    )
    candidate_delta = torch.diff(torch.cat((origin, xy), dim=2), dim=2)
    candidate_step = torch.norm(candidate_delta, dim=-1)
    candidate_arc = candidate_step.cumsum(dim=2)
    query = candidate_arc.flatten(1)
    segment = torch.searchsorted(route_arc, query, right=True) - 1
    segment = segment.clamp(0, ROUTE_POINT_COUNT - 2)
    final_segment = (route_mask.sum(dim=1, dtype=torch.long) - 2).clamp_min(0)
    segment = torch.minimum(segment, final_segment.unsqueeze(1))
    flat_segment = segment.reshape(count, -1)
    p0 = torch.gather(
        route[:, :-1], 1, flat_segment.unsqueeze(-1).expand(-1, -1, 2)
    )
    p1 = torch.gather(
        route[:, 1:], 1, flat_segment.unsqueeze(-1).expand(-1, -1, 2)
    )
    s0 = torch.gather(route_arc[:, :-1], 1, flat_segment)
    length = torch.gather(route_step, 1, flat_segment)
    ratio = ((query - s0) / length).clamp(0.0, 1.0)
    matched = p0 + ratio.unsqueeze(-1) * (p1 - p0)
    matched = matched.reshape(
        count, DRIVE_CANDIDATE_COUNT, FUTURE_POINT_COUNT, 2
    )
    lateral = torch.norm(xy - matched, dim=-1).mean(dim=2) / 10.0
    route_heading_all = torch.atan2(route_delta[..., 1], route_delta[..., 0])
    route_heading = torch.gather(route_heading_all, 1, flat_segment).reshape(
        count, DRIVE_CANDIDATE_COUNT, FUTURE_POINT_COUNT
    )
    candidate_heading = torch.atan2(
        candidate_delta[..., 1], candidate_delta[..., 0]
    )
    heading = _wrap_angle(candidate_heading - route_heading).abs().mean(dim=2) / math.pi
    times = (
        torch.arange(1, FUTURE_POINT_COUNT + 1, dtype=dtype, device=device)
        * 0.1
    )
    causal_speed = (
        ego_speed_mps.detach().unsqueeze(1)
        + ego_acceleration_mps2.detach().unsqueeze(1) * times.unsqueeze(0)
    ).clamp(0.0, PHYSICAL_MAXIMUM_SPEED_MPS)
    speed_cost = (
        (speed - causal_speed.unsqueeze(1)).abs().mean(dim=2)
        / PHYSICAL_MAXIMUM_SPEED_MPS
    )
    candidate_heading_delta = _wrap_angle(
        torch.diff(candidate_heading, dim=2, prepend=candidate_heading[:, :, :1])
    )
    candidate_curvature = candidate_heading_delta / candidate_step.clamp_min(0.01)
    route_heading_delta = _wrap_angle(
        torch.diff(route_heading_all, dim=1, prepend=route_heading_all[:, :1])
    )
    route_curvature_all = torch.where(
        segment_valid,
        route_heading_delta / route_step.clamp_min(0.01),
        torch.zeros_like(route_heading_delta),
    )
    route_curvature = torch.gather(
        route_curvature_all, 1, flat_segment
    ).reshape(count, DRIVE_CANDIDATE_COUNT, FUTURE_POINT_COUNT)
    curvature = (
        (candidate_curvature - route_curvature).abs().clamp_max(0.5).mean(dim=2)
        / 0.5
    )
    result = torch.stack((lateral, heading, speed_cost, curvature), dim=2)
    _require(
        tuple(result.shape) == (count, DRIVE_CANDIDATE_COUNT, WEIGHT_COUNT)
        and bool(torch.isfinite(result).all())
        and bool((result >= 0.0).all()),
        "finite nonnegative N-by-6-by-4 causal costs required",
    )
    return result.contiguous()


class PositiveRouteCostScorer(nn.Module):
    """HH_260906 - Learn four shared nonnegative costs with no candidate-index parameters."""

    def __init__(self) -> None:
        """HH_260906 - Initialize each effective cost weight to one without RNG use."""
        super().__init__()
        raw = math.log(math.expm1(WEIGHT_INITIAL_VALUE))
        self.raw_weights = nn.Parameter(
            torch.full((WEIGHT_COUNT,), raw, dtype=torch.float32)
        )

    def effective_weights(self) -> Tensor:
        """HH_260906 - Map unconstrained storage to strictly nonnegative weights."""
        return F.softplus(self.raw_weights)

    def forward(self, costs: Tensor) -> Tensor:
        """HH_260906 - Score every candidate with the same four physical weights."""
        _require(
            isinstance(costs, Tensor)
            and costs.ndim == 3
            and costs.shape[1:] == (DRIVE_CANDIDATE_COUNT, WEIGHT_COUNT)
            and costs.dtype == self.raw_weights.dtype
            and costs.device == self.raw_weights.device
            and bool(torch.isfinite(costs).all())
            and bool((costs >= 0.0).all()),
            "finite nonnegative causal candidate costs required",
        )
        result = -(costs.detach() * self.effective_weights()).sum(dim=2)
        _require(bool(torch.isfinite(result).all()), "nonfinite route-cost scores")
        return result


def make_scorer(device: str | torch.device = "cpu") -> PositiveRouteCostScorer:
    """HH_260906 - Construct the deterministic four-parameter scorer on one device."""
    scorer = PositiveRouteCostScorer().to(_device(device))
    _require(
        set(scorer.state_dict()) == {"raw_weights"}
        and sum(parameter.numel() for parameter in scorer.parameters()) == WEIGHT_COUNT,
        "route scorer must contain only four shared weight parameters",
    )
    return scorer


def mode_and_turn_preserving_logits(
    original_logits: Tensor,
    route_scores: Tensor,
    straight_ranker_mask: Tensor,
) -> Tensor:
    """HH_260906 - Rerank confident straight DRIVE rows and byte-preserve every fallback."""
    count = len(original_logits)
    _require(
        isinstance(original_logits, Tensor)
        and isinstance(route_scores, Tensor)
        and isinstance(straight_ranker_mask, Tensor)
        and tuple(original_logits.shape) == (count, TOTAL_CANDIDATE_COUNT)
        and tuple(route_scores.shape) == (count, DRIVE_CANDIDATE_COUNT)
        and tuple(straight_ranker_mask.shape) == (count,)
        and straight_ranker_mask.dtype == torch.bool
        and original_logits.dtype == route_scores.dtype
        and original_logits.device
        == route_scores.device
        == straight_ranker_mask.device
        and bool(torch.isfinite(original_logits).all())
        and bool(torch.isfinite(route_scores).all()),
        "aligned finite logits, route scores and policy mask required",
    )
    original = original_logits.detach()
    original_selected = original.argmax(dim=1)
    original_drive = original_selected < DRIVE_CANDIDATE_COUNT
    apply = original_drive & straight_ranker_mask
    drive_maximum = original[:, :DRIVE_CANDIDATE_COUNT].max(
        dim=1, keepdim=True
    ).values
    normalized = (
        route_scores
        - route_scores.max(dim=1, keepdim=True).values
        + drive_maximum
    )
    candidate = torch.cat(
        (normalized, original[:, DRIVE_CANDIDATE_COUNT:]), dim=1
    )
    result = torch.where(apply.unsqueeze(1), candidate, original)
    final_selected = result.argmax(dim=1)
    _require(
        not bool(
            ((final_selected < DRIVE_CANDIDATE_COUNT) != original_drive).any()
        )
        and torch.equal(result[~apply], original[~apply])
        and torch.equal(
            result[apply, :DRIVE_CANDIDATE_COUNT].max(dim=1).values,
            drive_maximum[apply, 0],
        ),
        "F mode/group maximum or byte-exact fallback preservation failed",
    )
    return result


def preservation_proof(
    original_logits: Tensor,
    final_logits: Tensor,
    straight_ranker_mask: Tensor,
) -> dict[str, Any]:
    """HH_260906 - Count mode, STOP, turn and group-maximum preservation exactly."""
    original_selected = original_logits.argmax(dim=1)
    final_selected = final_logits.argmax(dim=1)
    original_drive = original_selected < DRIVE_CANDIDATE_COUNT
    apply = original_drive & straight_ranker_mask
    fallback = ~apply
    drive_maximum_mismatch = int(
        (
            final_logits[apply, :DRIVE_CANDIDATE_COUNT].max(dim=1).values
            != original_logits[apply, :DRIVE_CANDIDATE_COUNT].max(dim=1).values
        ).sum()
    )
    fallback_row_mismatch = sum(
        not torch.equal(final_logits[index], original_logits[index])
        for index in torch.where(fallback)[0].tolist()
    )
    mode_change = int(
        (
            (original_selected < DRIVE_CANDIDATE_COUNT)
            != (final_selected < DRIVE_CANDIDATE_COUNT)
        ).sum()
    )
    return {
        "sample_count": len(original_logits),
        "original_drive_count": int(original_drive.sum()),
        "original_stop_count": int((~original_drive).sum()),
        "straight_ranker_policy_count": int(straight_ranker_mask.sum()),
        "straight_drive_reranked_count": int(apply.sum()),
        "fallback_row_count": int(fallback.sum()),
        "mode_change_count": mode_change,
        "fallback_row_logit_mismatch_count": int(fallback_row_mismatch),
        "reranked_drive_group_maximum_mismatch_count": drive_maximum_mismatch,
        "pass": bool(
            mode_change == 0
            and fallback_row_mismatch == 0
            and drive_maximum_mismatch == 0
        ),
    }


def _fit_weights(
    costs: Tensor,
    targets: Tensor,
    route_groups: Sequence[str],
    selected: Tensor,
    *,
    callback: Callable[[dict[str, Any]], None] | None = None,
) -> tuple[PositiveRouteCostScorer, list[dict[str, Any]]]:
    """HH_260906 - Fit fixed-budget route-balanced weights on selected TRAIN DRIVE rows."""
    _require(
        tuple(costs.shape[1:]) == (DRIVE_CANDIDATE_COUNT, WEIGHT_COUNT)
        and tuple(targets.shape) == (len(costs),)
        and tuple(selected.shape) == (len(costs),)
        and selected.dtype == torch.bool
        and len(route_groups) == len(costs)
        and bool(selected.any()),
        "aligned costs, targets, groups and nonempty training selection required",
    )
    selected_groups = tuple(
        sorted({route_groups[index] for index in torch.where(selected)[0].tolist()})
    )
    _require(
        selected_groups
        and set(selected_groups).issubset(set(ROUTE_GROUPS)),
        "selected TRAIN rows contain an unknown route group",
    )
    scorer = make_scorer(costs.device)
    optimizer = torch.optim.Adam(scorer.parameters(), lr=LEARNING_RATE)
    history: list[dict[str, Any]] = []
    scorer.train()
    for step in range(FIT_STEPS):
        logits = scorer(costs)
        losses = []
        group_counts = {}
        for group in selected_groups:
            mask = selected & torch.tensor(
                [value == group for value in route_groups],
                dtype=torch.bool,
                device=costs.device,
            )
            group_counts[group] = int(mask.sum())
            _require(group_counts[group] > 0, "empty route-balanced group")
            losses.append(F.cross_entropy(logits[mask], targets[mask]))
        objective = torch.stack(losses).mean()
        _require(bool(torch.isfinite(objective)), "nonfinite route-ranker objective")
        optimizer.zero_grad(set_to_none=True)
        objective.backward()
        _require(
            scorer.raw_weights.grad is not None
            and bool(torch.isfinite(scorer.raw_weights.grad).all()),
            "missing or nonfinite route-weight gradient",
        )
        norm = torch.nn.utils.clip_grad_norm_(scorer.parameters(), MAX_GRAD_NORM)
        _require(bool(torch.isfinite(norm)), "nonfinite route-weight gradient norm")
        optimizer.step()
        _require(
            bool(torch.isfinite(scorer.raw_weights).all())
            and bool(torch.isfinite(scorer.effective_weights()).all())
            and bool((scorer.effective_weights() >= 0.0).all())
            and _nested_tensors_are_finite(optimizer.state_dict()),
            "nonfinite or negative route-ranker state",
        )
        record = {
            "step": step + 1,
            "optimization_loss": float(objective.detach()),
            "gradient_norm": float(norm),
            "effective_weights": scorer.effective_weights().detach().cpu().tolist(),
            "route_group_counts": group_counts,
        }
        history.append(record)
        if callback is not None:
            callback(copy.deepcopy(record))
    return scorer.eval(), history


def _selection_metrics(
    scores: Tensor,
    costs: Tensor,
    targets: Tensor,
    selected: Tensor,
    effective_weights: Tensor,
) -> dict[str, Any]:
    """HH_260906 - Summarize learned weighted selection and regret for one row subset."""
    _require(
        tuple(scores.shape) == (len(costs), DRIVE_CANDIDATE_COUNT)
        and tuple(costs.shape)
        == (len(costs), DRIVE_CANDIDATE_COUNT, WEIGHT_COUNT)
        and tuple(targets.shape) == (len(costs),)
        and tuple(selected.shape) == (len(costs),)
        and tuple(effective_weights.shape) == (WEIGHT_COUNT,)
        and selected.dtype == torch.bool
        and bool(selected.any())
        and bool(torch.isfinite(scores).all())
        and bool(torch.isfinite(effective_weights).all())
        and bool((effective_weights >= 0.0).all()),
        "aligned learned scores, costs, targets, weights and selection required",
    )
    chosen = scores.argmax(dim=1)
    rows = torch.where(selected)[0]
    weighted = (costs * effective_weights.view(1, 1, -1)).sum(dim=2)
    chosen_cost = weighted[rows, chosen[rows]]
    target_cost = weighted[rows, targets[rows]]
    best_cost = weighted[rows].min(dim=1).values
    return {
        "sample_count": len(rows),
        "oracle_agreement_count": int((chosen[rows] == targets[rows]).sum()),
        "oracle_agreement_percent": float(
            (chosen[rows] == targets[rows]).float().mean() * 100.0
        ),
        "mean_weighted_alignment_cost": float(chosen_cost.mean()),
        "mean_weighted_target_alignment_cost": float(target_cost.mean()),
        "mean_weighted_alignment_regret": float((chosen_cost - best_cost).mean()),
        "mean_weighted_selected_minus_target_cost": float(
            (chosen_cost - target_cost).mean()
        ),
    }


def fit_causal_route_ranker(
    train: CachedStopmixSelectorData,
    dev: CachedStopmixSelectorData,
    train_sidecar: CausalRouteSidecar,
    dev_sidecar: CausalRouteSidecar,
    *,
    projected_costs_fn: Callable[..., Tensor],
    projection_proof: Mapping[str, Any],
    device: str | torch.device = "cpu",
    callback: Callable[[str, dict[str, Any]], None] | None = None,
    check_deadline: Callable[[], None] | None = None,
) -> dict[str, Any]:
    """HH_260906 - Fit on TRAIN then enter one post-freeze DEV phase and combined reduction."""
    validate_pair_and_sidecars(train, dev, train_sidecar, dev_sidecar)
    projected._validate_projection(projected_costs_fn, projection_proof)
    device = _device(device)
    before = {
        "cache": {value.split: value.digest() for value in (train, dev)},
        "sidecar": {
            value.split: value.digest() for value in (train_sidecar, dev_sidecar)
        },
    }
    try:
        train_data = train.to(device)
        train_inputs = train_sidecar.to(device)
        train_alignment_costs = candidate_route_costs(
            train_inputs.route_xy,
            train_inputs.route_mask,
            train_inputs.ego_speed_mps,
            train_inputs.ego_acceleration_mps2,
            train_data.candidate_xy[:, :DRIVE_CANDIDATE_COUNT],
            train_data.candidate_speed[:, :DRIVE_CANDIDATE_COUNT],
        )
        projected_costs, _ = projected.projected_targets(
            train_data, projected_costs_fn, check_deadline
        )
        train_targets = projected_costs[:, :DRIVE_CANDIDATE_COUNT].argmin(dim=1)
        original_drive = (
            train_data.original_logits.argmax(dim=1) < DRIVE_CANDIDATE_COUNT
        )
        _require(
            int(original_drive.sum()) == EXPECTED_TRAIN_DRIVE_COUNT
            and int((~original_drive).sum()) == EXPECTED_TRAIN_STOP_COUNT,
            "exact F-original TRAIN DRIVE1232/STOP243 partition required",
        )
        loo = []
        for held_group in ROUTE_GROUPS:
            _check(check_deadline)
            train_selection = original_drive & torch.tensor(
                [value != held_group for value in train_inputs.route_groups],
                dtype=torch.bool,
                device=device,
            )
            held_selection = original_drive & torch.tensor(
                [value == held_group for value in train_inputs.route_groups],
                dtype=torch.bool,
                device=device,
            )
            fold_callback = None
            if callback is not None:
                fold_callback = lambda row, group=held_group: callback(
                    "loo_" + group, row
                )
            scorer, history = _fit_weights(
                train_alignment_costs,
                train_targets,
                train_inputs.route_groups,
                train_selection,
                callback=fold_callback,
            )
            with torch.no_grad():
                scores = scorer(train_alignment_costs)
            policy_logits = mode_and_turn_preserving_logits(
                train_data.original_logits,
                scores,
                train_inputs.straight_ranker_mask,
            )
            loo.append(
                {
                    "held_route_group": held_group,
                    "fit_drive_count": int(train_selection.sum()),
                    "optimization_row_exposures": int(train_selection.sum())
                    * FIT_STEPS,
                    "fit_policy_straight_drive_count": int(
                        (
                            train_selection
                            & train_inputs.straight_ranker_mask
                        ).sum()
                    ),
                    "held_drive_count": int(held_selection.sum()),
                    "held_policy_straight_drive_count": int(
                        (
                            held_selection
                            & train_inputs.straight_ranker_mask
                        ).sum()
                    ),
                    "fit_steps": FIT_STEPS,
                    "final_effective_weights": scorer.effective_weights()
                    .detach()
                    .cpu()
                    .tolist(),
                    "held_scorer_alignment": _selection_metrics(
                        scores,
                        train_alignment_costs,
                        train_targets,
                        held_selection,
                        scorer.effective_weights().detach(),
                    ),
                    "held_policy_preservation": preservation_proof(
                        train_data.original_logits[held_selection],
                        policy_logits[held_selection],
                        train_inputs.straight_ranker_mask[held_selection],
                    ),
                    "history_first": history[0],
                    "history_last": history[-1],
                }
            )
        final_callback = None
        if callback is not None:
            final_callback = lambda row: callback("final_all_train", row)
        scorer, final_history = _fit_weights(
            train_alignment_costs,
            train_targets,
            train_inputs.route_groups,
            original_drive,
            callback=final_callback,
        )
        with torch.no_grad():
            final_train_scores = scorer(train_alignment_costs)
            final_train_logits = mode_and_turn_preserving_logits(
                train_data.original_logits,
                final_train_scores,
                train_inputs.straight_ranker_mask,
            )
        train_proof = preservation_proof(
            train_data.original_logits,
            final_train_logits,
            train_inputs.straight_ranker_mask,
        )
        _require(train_proof["pass"], "final TRAIN preservation proof failed")
        _require(
            scorer.training is False,
            "final TRAIN scorer must be frozen before any DEV tensor transfer",
        )
        dev_data = dev.to(device)
        dev_inputs = dev_sidecar.to(device)
        dev_alignment_costs = candidate_route_costs(
            dev_inputs.route_xy,
            dev_inputs.route_mask,
            dev_inputs.ego_speed_mps,
            dev_inputs.ego_acceleration_mps2,
            dev_data.candidate_xy[:, :DRIVE_CANDIDATE_COUNT],
            dev_data.candidate_speed[:, :DRIVE_CANDIDATE_COUNT],
        )
        with torch.no_grad():
            final_dev_scores = scorer(dev_alignment_costs)
            final_dev_logits = mode_and_turn_preserving_logits(
                dev_data.original_logits,
                final_dev_scores,
                dev_inputs.straight_ranker_mask,
            )
        dev_proof = preservation_proof(
            dev_data.original_logits,
            final_dev_logits,
            dev_inputs.straight_ranker_mask,
        )
        _require(dev_proof["pass"], "final DEV preservation proof failed")
        final_dev_metrics = cache_core.evaluate_logits(
            dev_data, final_dev_logits, check_deadline
        )
        report = {
            "artifact_id": ARTIFACT_ID,
            "status": "CAUSAL_ROUTE_RANKER_FIT_COMPLETE_NOT_PROMOTED",
            "source_checkpoint_sha256": SOURCE_CHECKPOINT_SHA256,
            "raw_dataset_sha256": RAW_DATASET_SHA256,
            "seed": SEED,
            "device": str(device),
            "input_contract": {
                "ranker_inputs": [
                    "raw.navigation.route_polyline_base_m",
                    "raw.ego.current_planar_speed_mps",
                    "raw.ego.current_longitudinal_acceleration_mps2",
                    "frozen_candidate_xy",
                    "frozen_candidate_speed",
                ],
                "policy_inputs": [
                    "raw.navigation.current_command",
                    "raw.navigation.current_route_heading_change",
                    "causal_turn_latch",
                    "original_F_mode_and_DRIVE_group_maximum_only",
                ],
                "forbidden_ranker_inputs": [
                    "target",
                    "label",
                    "future_ego",
                    "image",
                    "fused_context",
                    "map_id",
                    "episode_id",
                    "original_F_candidate_ranking_logits",
                ],
            },
            "command_contract": {
                "names": list(COMMAND_NAMES),
                "turn_fallback_commands": list(TURN_COMMANDS),
                "straight_candidate_commands": list(STRAIGHT_COMMANDS),
                "straight_heading_maximum_rad": STRAIGHT_HEADING_MAXIMUM_RAD,
                "turn_latch_heading_minimum_rad": TURN_LATCH_HEADING_MINIMUM_RAD,
                "source": "Command names come from common_10hz_v1. The margin and latch are predeclared exploratory conservative heuristics informed only by TRAIN input structure; they are not contract guarantees and no DEV outcome selected them.",
                "limitation": "A causally indistinguishable straight prefix before any turn evidence can be reranked. Exact F fallback is guaranteed from the first current command or route-geometry turn evidence through the rest of that sequence, not retrospectively for the prefix.",
            },
            "scorer": {
                "features": list(FEATURE_NAMES),
                "parameter_count": WEIGHT_COUNT,
                "parameter_keys": list(scorer.state_dict()),
                "candidate_index_parameters": False,
                "nonnegative_weights": True,
                "final_effective_weights": scorer.effective_weights()
                .detach()
                .cpu()
                .tolist(),
                "fit_steps": FIT_STEPS,
                "learning_rate": LEARNING_RATE,
                "maximum_gradient_norm": MAX_GRAD_NORM,
                "route_group_balanced_objective": True,
            },
            "train_partition": {
                "sample_count": len(train_data),
                "f_original_drive_count": int(original_drive.sum()),
                "f_original_stop_count": int((~original_drive).sum()),
                "stop_rows_used_for_loss": 0,
                "drive_rows_used_for_loss": int(original_drive.sum()),
                "final_fit_optimization_row_exposures": int(
                    original_drive.sum()
                )
                * FIT_STEPS,
                "all_fold_and_final_optimization_row_exposures": sum(
                    value["optimization_row_exposures"] for value in loo
                )
                + int(original_drive.sum()) * FIT_STEPS,
                "stop_optimization_row_exposures": 0,
                "drive_count_by_physical_route_group": {
                    group: int(
                        (
                            original_drive
                            & torch.tensor(
                                [
                                    value == group
                                    for value in train_inputs.route_groups
                                ],
                                dtype=torch.bool,
                                device=device,
                            )
                        ).sum()
                    )
                    for group in ROUTE_GROUPS
                },
            },
            "train_only_leave_one_route_group_out": loo,
            "structure_and_budget_fixed_before_dev": True,
            "dev_post_freeze_phase_count": 1,
            "core_combined_full_split_metric_reduction_count": 1,
            "dev_tensors_transferred_only_after_final_train_scorer_frozen": True,
            "dev_used_for_training": False,
            "dev_used_for_structure_or_threshold_selection": False,
            "test_read_or_used": False,
            "final_preservation": {"train": train_proof, "val": dev_proof},
            "final_dev_metrics": final_dev_metrics,
            "final_history": {
                "row_count": len(final_history),
                "first": final_history[0],
                "last": final_history[-1],
            },
            "cache_sha256_before": before["cache"],
            "cache_sha256_after": dict(before["cache"]),
            "sidecar_sha256_before": before["sidecar"],
            "sidecar_sha256_after": dict(before["sidecar"]),
            "automatic_promotion": False,
            "runtime_checkpoint_exported": False,
            "closed_loop_evaluation": False,
            "vehicle_control_approved": False,
            "deployment_approved": False,
            "new_autonomous_function_head_added": False,
            "interpretation": "Exploratory TRAIN-fold route-conditioned reranker. Town03 is repeatedly seen DEV, not TEST or deployment evidence.",
        }
        return {
            "report": report,
            "scorer_state_dict": {
                name: value.detach().cpu().clone()
                for name, value in scorer.state_dict().items()
            },
            "final_logits_cpu": {
                "train": final_train_logits.detach().cpu().clone(),
                "val": final_dev_logits.detach().cpu().clone(),
            },
        }
    finally:
        _require(
            train.digest() == before["cache"]["train"]
            and dev.digest() == before["cache"]["val"],
            "original H cache mutated during J fitting",
        )
        _require(
            train_sidecar.digest() == before["sidecar"]["train"]
            and dev_sidecar.digest() == before["sidecar"]["val"],
            "original causal sidecar mutated during J fitting",
        )
