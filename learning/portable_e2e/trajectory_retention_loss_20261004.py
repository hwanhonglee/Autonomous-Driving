"""HH_261004 - Research-only masked neural-parent retention on admitted old TRAIN replay.

This pure loss performs no I/O, dataset admission, decoding, optimizer updates,
runtime changes or capability approval. The caller must authenticate the sealed
dataset/admission/qualification hashes and actual sample membership first, run
the same causal inputs through a frozen eval-mode parent, and declare the added
coefficient before fitting. The metadata below checks explicit binding/alignment;
hash-shaped strings are not cryptographic authentication of dataset contents.

Only the original observed spatial masks may be used. Missing future curvature
and stationary curvature remain invalid; parent predictions are retention
targets, not fabricated expert labels. Decoded retention is intentionally absent:
matching raw outputs does not require reinterpreting measured speed, causal
preview support or the separate low-speed decoder contract.
"""
from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import re

import torch
from torch import Tensor
from torch.nn import functional as F

from .contract import ContractError

SCHEMA = "portable_e2e.trajectory_retention_loss.20261004.v1"
OLD_REPLAY_ROLES = frozenset(("OLD_RECOVERY_REPLAY", "OLD_LOW_SPEED_REPLAY", "OLD_LEGACY_REPLAY"))


@dataclass(frozen=True)
class ReplayProvenance:
    """HH_261004 - Explicit caller-authenticated binding for one old replay anchor, never an admission authority."""

    sample_id: str
    split: str
    role: str
    dataset_manifest_sha256: str
    admission_authority_sha256: str
    qualification_sha256: str
    qualified: bool
    training_denied: bool


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ContractError(message)


def _sha(value, name: str) -> None:
    _require(type(value) is str and re.fullmatch(r"[0-9a-f]{64}", value) is not None,
             name + " must be an explicit lowercase SHA256 binding")


def _provenance(rows, sample_ids, batch: int) -> None:
    _require(type(rows) in (tuple, list) and type(sample_ids) in (tuple, list)
             and len(rows) == len(sample_ids) == batch, "one provenance and actual sample ID per replay anchor required")
    for row, actual_id in zip(rows, sample_ids):
        _require(type(row) is ReplayProvenance, "explicit ReplayProvenance required")
        _require(type(actual_id) is str and bool(actual_id.strip())
                 and type(row.sample_id) is str and row.sample_id == actual_id,
                 "provenance must match the actual aligned sample ID")
        _require(type(row.split) is str and row.split == "TRAIN", "only TRAIN replay; DEV/TEST/HOLDOUT forbidden")
        _require(type(row.role) is str and row.role in OLD_REPLAY_ROLES, "only the three declared OLD replay roles allowed")
        _require(type(row.qualified) is bool and row.qualified is True
                 and type(row.training_denied) is bool and row.training_denied is False,
                 "unqualified, failed or explicitly denied captures cannot enter retention")
        for name in ("dataset_manifest_sha256", "admission_authority_sha256", "qualification_sha256"):
            _sha(getattr(row, name), name)


def _raw(outputs, reference: Tensor | None, label: str) -> tuple[Tensor, Tensor]:
    _require(isinstance(outputs, Mapping) and all(name in outputs for name in
             ("curvature_knots", "preferred_speed_knots")), label + " raw spatial outputs required")
    curvature, speed = outputs["curvature_knots"], outputs["preferred_speed_knots"]
    _require(isinstance(curvature, Tensor) and curvature.ndim == 2 and curvature.shape[0] > 0
             and curvature.shape[1] == 33 and curvature.dtype in (torch.float32, torch.float64),
             label + " curvature must be nonempty [B,33] float32/64")
    _require(isinstance(speed, Tensor) and speed.shape == curvature.shape
             and speed.dtype == curvature.dtype and speed.device == curvature.device,
             label + " preferred speed must match curvature shape/dtype/device")
    if reference is not None:
        _require(curvature.shape == reference.shape and curvature.dtype == reference.dtype
                 and curvature.device == reference.device, "parent/student raw ABI must match exactly")
    _require(bool(torch.isfinite(curvature).all()) and bool(torch.isfinite(speed).all()),
             label + " must be finite even at masked knots")
    _require(not bool((curvature.abs() > .2).any()) and not bool((speed < 0).any()),
             label + " original .2/m curvature and nonnegative raw speed ABI required")
    return curvature, speed


def _mask(mask, reference: Tensor, name: str) -> Tensor:
    _require(isinstance(mask, Tensor) and mask.shape == reference.shape
             and mask.dtype == torch.bool and mask.device == reference.device,
             name + " must be same-device boolean [B,33]")
    return mask


def _masked_per_anchor(errors: Tensor, mask: Tensor) -> Tensor:
    _require(bool(torch.isfinite(errors).all()), "nonfinite retention residual or loss")
    # HH_261004 - Average actual valid knots per anchor; an empty anchor stays graph-connected exact zero.
    return torch.where(mask, errors, torch.zeros_like(errors)).sum(-1) / mask.sum(-1).clamp_min(1).to(errors.dtype)


def trajectory_retention_loss(
    student_raw: Mapping[str, Tensor], parent_raw: Mapping[str, Tensor], *,
    observed_spatial_valid: Tensor, curvature_valid: Tensor, preferred_speed_valid: Tensor,
    provenance: tuple[ReplayProvenance, ...] | list[ReplayProvenance],
    sample_ids: tuple[str, ...] | list[str], parent_checkpoint_sha256: str,
) -> dict:
    """HH_261004 - Retain old raw behavior with .25*Huber(curvature/.1)+Huber(preferred speed).

    Each term averages valid knots within each anchor, then all B anchors.
    Empty masks are valid zero contributions, not evidence of old behavior.
    Curvature and speed masks remain separate subsets of the actual observed
    spatial prefix; no mask is inferred from a parent's confident predictions.
    No speed upper cap is imposed: old legacy TRAIN includes faster raw replay.
    The parent is unconditionally detached, even if a caller supplied a graph.
    """
    curvature, speed = _raw(student_raw, None, "student")
    parent_curvature, parent_speed = _raw(parent_raw, curvature, "parent")
    _sha(parent_checkpoint_sha256, "parent_checkpoint_sha256")
    _provenance(provenance, sample_ids, curvature.shape[0])
    observed = _mask(observed_spatial_valid, curvature, "observed_spatial_valid")
    cmask = _mask(curvature_valid, curvature, "curvature_valid")
    smask = _mask(preferred_speed_valid, curvature, "preferred_speed_valid")
    _require(not bool(((~observed[:, :-1]) & observed[:, 1:]).any()), "observed spatial validity must remain a prefix")
    _require(not bool(((cmask | smask) & ~observed).any()), "retention masks cannot extend the true observed spatial support")
    # HH_261004 - Preserve the frozen supervised raw scaling without backpropagating into the neural parent.
    cerrors = F.smooth_l1_loss(curvature / .1, parent_curvature.detach() / .1, reduction="none")
    serrors = F.smooth_l1_loss(speed, parent_speed.detach(), reduction="none")
    cper, sper = _masked_per_anchor(cerrors, cmask), _masked_per_anchor(serrors, smask)
    cterm, sterm = cper.mean(), sper.mean()
    total = .25 * cterm + sterm
    _require(bool(torch.isfinite(total)), "nonfinite total retention loss")
    return dict(schema=SCHEMA, total=total, raw_curvature=cterm, raw_preferred_speed=sterm,
                curvature_per_anchor=cper, preferred_speed_per_anchor=sper,
                curvature_valid_count=cmask.sum(-1), preferred_speed_valid_count=smask.sum(-1),
                observed_spatial_valid_count=observed.sum(-1), anchor_count=curvature.shape[0],
                parent_checkpoint_sha256=parent_checkpoint_sha256, parent_detached=True,
                decoded_path_retention=False, dataset_admitted=False, expert_labels_created=False,
                runtime_changed=False, model_promoted=False, feature_approved=False)
