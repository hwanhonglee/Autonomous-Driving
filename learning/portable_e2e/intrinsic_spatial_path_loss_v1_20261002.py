"""HH_261002 - Supervise intrinsic path shape without retiming true high-speed futures.

This LOSS-ONLY module reuses the frozen decoder's signed rear-base ROS curvature
knots at 0..64 m, spacing 2 m, and its 0.25 m geometry integration. It introduces
no runtime ABI, head, speed limit, future input, or dataset admission. A caller
must separately authenticate qualified TRAIN membership and preserve its old
raw curvature/speed losses and straight/offset replay.

Target indices are cumulative lengths of the actual XY chords from CURRENT
rear-base (0,0), not from the first future point. These polygonal lengths are
not native continuous arc or timestamps. On a curved analytic path they have a
small discretization floor; on a reversed/slipping path the fixed forward,
zero-initial-heading representation can have an irreducible floor. No label is
rotated, clipped, extrapolated, or replaced to remove either discrepancy.
"""

from __future__ import annotations

import torch
from torch import Tensor, nn
from torch.nn import functional as F

from .contract import ContractError
from .deterministic_linear_interpolation_v2 import (
    KERNEL_ID, fixed_decoder_interpolation_scope, interpolation_matrix,
)
from .path_time_decoder_v2 import PathTimeDecoderV2


SCHEMA = "portable_e2e.intrinsic_spatial_path_loss.20261002.v1"
CURVATURE_LIMIT_INV_M = .2


class IntrinsicSpatialPathLossV1(nn.Module):
    """HH_261002 - Unweighted per-anchor XY Huber loss; never call temporal decoding.

Arguments match existing raw ``curvature_knots``, ``target_xy`` and
``target_valid``. Original finite labels must use the same floating dtype/device
as curvature and a contiguous valid prefix. Only real valid points whose chord
arc is <=64 m are assessed. All exclusions and stationary anchors are counted;
an empty/stationary batch is not evidence of a learned geometric improvement.

Each anchor averages coordinate-mean Huber errors over its assessed points
(empty anchors contribute graph-connected zero), then all B anchors are averaged.
The prospective trainer must declare an additive coefficient before fitting and
require nonzero-arc TRAIN coverage; this module changes no existing objective.
"""

    def __init__(self):
        super().__init__()
        self.geometry = PathTimeDecoderV2()
        self.register_buffer("fixed_interpolation_matrix", interpolation_matrix(), persistent=False)

    def forward(self, curvature_knots: Tensor, target_xy: Tensor, target_valid: Tensor) -> dict:
        if (not isinstance(curvature_knots, Tensor) or curvature_knots.ndim != 2
                or curvature_knots.shape[0] < 1 or curvature_knots.shape[1] != 33
                or curvature_knots.dtype not in (torch.float32, torch.float64)):
            raise ContractError("intrinsic curvature requires nonempty [B,33] float32/64")
        batch = curvature_knots.shape[0]
        if (not isinstance(target_xy, Tensor) or target_xy.shape != (batch, 64, 2)
                or target_xy.dtype != curvature_knots.dtype or target_xy.device != curvature_knots.device):
            raise ContractError("true target XY must match [B,64,2] curvature dtype/device")
        if (not isinstance(target_valid, Tensor) or target_valid.shape != (batch, 64)
                or target_valid.dtype != torch.bool or target_valid.device != curvature_knots.device):
            raise ContractError("true target validity requires same-device boolean [B,64]")
        if not bool(torch.isfinite(curvature_knots).all()) or not bool(torch.isfinite(target_xy).all()):
            raise ContractError("intrinsic inputs must retain finite original values")
        if bool((curvature_knots.abs() > CURVATURE_LIMIT_INV_M).any()):
            raise ContractError("original model curvature limit .2 inverse metres unchanged")
        if bool(((~target_valid[:, :-1]) & target_valid[:, 1:]).any()):
            raise ContractError("true future validity must remain a prefix; no missing-point bridges")

        # HH_261002 - Future geometry is detached supervision, never a gradient-bearing model/runtime input.
        truth = target_xy.detach()
        origin = torch.zeros((batch, 1, 2), dtype=torch.float64, device=truth.device)
        positions = torch.cat((origin, truth.to(torch.float64)), dim=1)
        chords = torch.norm(positions[:, 1:] - positions[:, :-1], dim=-1)
        chords = torch.where(target_valid, chords, torch.zeros_like(chords))
        arc = chords.cumsum(-1)
        if not bool(torch.isfinite(arc).all()):
            raise ContractError("actual valid XY chord arc is nonfinite; no saturation")
        assessed = target_valid & (arc <= 64.)
        # HH_261002 - Zero query/error placeholders for UNASSESSED points are not clipped or rewritten labels.
        query = torch.where(assessed, arc, torch.zeros_like(arc)).to(curvature_knots.dtype)
        matrix = self.fixed_interpolation_matrix.to(device=curvature_knots.device, dtype=curvature_knots.dtype)
        with fixed_decoder_interpolation_scope(matrix) as receipt:
            dense, heading, spatial_xy = self.geometry._spatial_shape(curvature_knots)
        if receipt["fixed_calls"] != 1 or receipt["unrelated_calls_delegated"] != 0:
            raise ContractError("exact one frozen spatial interpolation required")
        predicted = self.geometry._sample_shape(query, dense, heading, spatial_xy)
        residual = torch.where(assessed[..., None], predicted - truth, torch.zeros_like(predicted))
        errors = F.smooth_l1_loss(residual, torch.zeros_like(residual), reduction="none").mean(-1)
        count = assessed.sum(-1)
        per_anchor = errors.sum(-1) / count.clamp_min(1).to(errors.dtype)
        loss = per_anchor.mean()
        if not bool(torch.isfinite(predicted).all()) or not bool(torch.isfinite(loss)):
            raise ContractError("intrinsic path shape/loss is nonfinite")
        nonzero = (assessed & (arc > 0)).any(-1)
        return dict(schema=SCHEMA, total=loss, loss_per_anchor=per_anchor,
            sampled_predicted_xy=predicted, sampled_prediction_valid=assessed,
            true_future_chord_arc_m=arc, original_target_valid=target_valid.detach().clone(),
            original_valid_point_count=target_valid.sum(-1), assessed_point_count=count,
            valid_outside_64m_point_count=(target_valid & ~assessed).sum(-1),
            zero_chord_valid_point_count=(target_valid & (chords == 0)).sum(-1),
            nonzero_arc_anchor_count=nonzero.sum(), anchor_count=batch,
            nonzero_supervision_present=bool(nonzero.any()),
            future_arc_is_loss_only_not_causal_support=True, time_supervision=False,
            speed_or_v0_changed=False, stationary_curvature_labels_created=False,
            dataset_admitted=False, model_promoted=False, feature_approved=False,
            interpolation_kernel_id=KERNEL_ID, interpolation_receipt=dict(receipt))
