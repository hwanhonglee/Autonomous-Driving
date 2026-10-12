"""HH_261001 - Separate raw spatial supervision from optional physically retimed supervision."""

from __future__ import annotations

import torch
from torch import Tensor
from torch.nn import functional as F

from .contract import ContractError


def _masked_mean(values: Tensor, mask: Tensor) -> Tensor:
    # HH_261001 - Missing labels are not negative labels; an empty task contributes zero.
    if mask.dtype != torch.bool or values.shape != mask.shape:
        raise ContractError("loss mask must be boolean and have the value shape")
    if not bool(torch.isfinite(values).all()):
        raise ContractError("nonfinite supervised loss")
    return (values * mask.to(values.dtype)).sum() / mask.sum().clamp_min(1)


def _label(targets: dict, name: str, reference: Tensor) -> Tensor:
    value = targets[name]
    if not isinstance(value, Tensor) or value.shape != reference.shape:
        raise ContractError("target shape differs: " + name)
    if not bool(torch.isfinite(value).all()):
        raise ContractError("target must be finite: " + name)
    return value


def temporal_trajectory_loss_v2(raw: dict, targets: dict, *, decoded: dict | None = None) -> dict:
    """HH_261001 - Bootstrap raw geometry/speed without inventing stop labels or bypassing decoding.

    ``decoded=None`` explicitly selects the raw-spatial wiring diagnostic. It is
    not the complete future-trajectory training objective or a deployment test.
    True zero-speed labels can still send a gradient through positive raw speed.
    A caller may provide decoded output only when observed speed and preview are
    compatible with the declared decoder; original target labels stay unchanged.
    """
    curvature, preferred = raw["curvature_knots"], raw["preferred_speed_knots"]
    if curvature.ndim != 2 or curvature.shape[1] != 33 or preferred.shape != curvature.shape:
        raise ContractError("raw spatial outputs must have [B,33]")
    curvature_target = _label(targets, "spatial_curvature", curvature)
    speed_target = _label(targets, "spatial_preferred_speed", preferred)
    spatial_mask = targets["spatial_valid"]
    curvature_mask = targets.get("spatial_curvature_valid", spatial_mask)
    raw_curvature = _masked_mean(F.smooth_l1_loss(curvature / .1, curvature_target / .1,
                                                 reduction="none"), curvature_mask)
    speed_mask = targets.get("spatial_speed_valid", spatial_mask)
    raw_speed = _masked_mean(F.smooth_l1_loss(preferred, speed_target, reduction="none"), speed_mask)
    zero = (curvature.sum() + preferred.sum()) * 0.
    terms = dict(raw_curvature=raw_curvature, raw_preferred_speed=raw_speed,
                 intent=zero, stop_distance=zero, initial_acceleration=zero,
                 temporal_xy=zero, temporal_speed=zero)
    # HH_261001 - Semantics are learned only from verified intent/stop annotations.
    if "intent_valid" in targets:
        intent_target, intent_valid = targets["intent_target"], targets["intent_valid"]
        logits = raw["intent_logits"]
        if (logits.shape != (curvature.shape[0], 3) or intent_target.shape != (curvature.shape[0],)
                or intent_target.dtype != torch.long or bool(((intent_target < 0) | (intent_target > 2)).any())):
            raise ContractError("intent target/shape differs")
        terms["intent"] = _masked_mean(F.cross_entropy(logits, intent_target, reduction="none"), intent_valid)
    if "stop_distance_valid" in targets:
        predicted = F.softplus(raw["stop_distance_raw"].reshape(-1))
        target = _label(targets, "stop_distance_target", predicted)
        terms["stop_distance"] = _masked_mean(F.smooth_l1_loss(predicted, target, reduction="none"),
                                               targets["stop_distance_valid"])
    if "initial_acceleration_valid" in targets:
        predicted = raw["acceleration_prediction"].reshape(-1)
        target = _label(targets, "initial_acceleration_target", predicted)
        terms["initial_acceleration"] = _masked_mean(F.smooth_l1_loss(predicted, target, reduction="none"),
                                                      targets["initial_acceleration_valid"])
    if decoded is not None:
        xy = decoded["xy"][:, 0]
        # HH_261001 - Compare instantaneous expert labels with instantaneous speed, not XY chord speed.
        speed = decoded["instantaneous_speed"][:, 1:]
        xy_target = _label(targets, "target_xy", xy)
        speed_target = _label(targets, "target_speed_mps", speed)
        valid = targets["target_valid"]
        if xy.shape[1:] != (64, 2) or speed.shape != valid.shape or valid.dtype != torch.bool:
            raise ContractError("64-step time supervision ABI differs")
        # HH_261001 - Normalize each horizon independently so frame counts do not change task weights.
        for begin, end, weight in ((0, 5, .4), (5, 20, .3), (20, 64, .3)):
            mask = valid[:, begin:end]
            terms["temporal_xy"] = terms["temporal_xy"] + weight * _masked_mean(
                F.smooth_l1_loss(xy[:, begin:end], xy_target[:, begin:end], reduction="none").mean(-1), mask)
            terms["temporal_speed"] = terms["temporal_speed"] + weight * _masked_mean(
                F.smooth_l1_loss(speed[:, begin:end], speed_target[:, begin:end], reduction="none"), mask)
    total = (.25 * terms["raw_curvature"] + terms["raw_preferred_speed"] + .2 * terms["intent"]
             + .1 * terms["stop_distance"] + .1 * terms["initial_acceleration"]
             + terms["temporal_xy"] + terms["temporal_speed"])
    if not bool(torch.isfinite(total)):
        raise ContractError("nonfinite total loss")
    return dict(total=total, **terms, projected_time_supervision=decoded is not None)
