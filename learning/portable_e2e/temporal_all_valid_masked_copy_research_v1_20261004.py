"""HH_261004 - Prepare a source-pinned cached-only masked-copy candidate, never activate it.

Only the input allocation changes, and only for a trusted read-only encoder
inside the original explicit cached API with every history slot valid. Both
original finite validations, all provenance/state/pixel hashes, cache behavior,
conditioning, fusion and raw outputs remain in their original methods. This
source-only candidate is not a timing result or a driving capability.
"""

from __future__ import annotations

import ast
import copy
import hashlib
from pathlib import Path

import torch
from torch import Tensor, nn
from torch.nn.modules import module as torch_module

from .contract import ContractError
from .temporal_cached_encoder_v2 import CachedTemporalModelV2, _CachedSpatialImageEncoderV2
from .temporal_cpu_finite_validation_v2_20261001 import CPUFiniteCachedTemporalModelV2
from .temporal_model_v2 import SpatialImageEncoderV2, TemporalModelV2
from .temporal_route_recovery_v3_20261001 import RouteRecoveryTemporalModelV3


SCHEMA = "portable_e2e.cached_all_valid_masked_copy.20261004.v1.research"
PARENT_SOURCE_PINS = {
    "temporal_model_v2.py": "93f709c6c229e705899fcf1778f9dd253ea867e88a6f237ad3f0fb8efdbbf435",
    "temporal_cached_encoder_v2.py": "55ef7371e0a1f4ec2b9cb854282e4d51b6c406b055ec5eceb8e286737f03adff",
    "temporal_cpu_finite_validation_v2_20261001.py": "ecc6f7af5e8ee8dcbbc3a656c50e7de474f7a7bfa1e1eca630af40faa59e16c5",
    "temporal_route_recovery_v3_20261001.py": "41494aeb6175c7e8bb054b0c099b121e1897144681a6e3591919bc0740925a25",
}


def require_frozen_parent_sources() -> None:
    """HH_261004 - Reject source drift before compilation or constructing any new model."""
    folder = Path(__file__).parent
    for name, expected in PARENT_SOURCE_PINS.items():
        path = folder / name
        if path.is_symlink() or not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ContractError("masked-copy candidate requires frozen parent source: " + name)


# HH_261004 - Keep exact installed read-only implementations; later monkeypatches force allocation.
_TRUSTED_FORWARDS = {kind: kind.forward for kind in
    (_CachedSpatialImageEncoderV2, SpatialImageEncoderV2, nn.Sequential, nn.Conv2d,
     nn.GroupNorm, nn.SiLU, nn.AvgPool2d)}
_TRUSTED_MODULE_CALL = nn.Module.__call__
_TRUSTED_MODULE_CALL_IMPL = nn.Module._call_impl
_TRUSTED_CONV_FORWARD = nn.Conv2d._conv_forward


def _trusted_readonly_encoder(encoder) -> bool:
    """HH_261004 - Preserve the original allocated path for hooks or image-mutating substitutions."""
    if type(encoder) is not _CachedSpatialImageEncoderV2:
        return False
    if nn.Module.__call__ is not _TRUSTED_MODULE_CALL or nn.Module._call_impl is not _TRUSTED_MODULE_CALL_IMPL:
        return False
    if SpatialImageEncoderV2.forward is not _TRUSTED_FORWARDS[SpatialImageEncoderV2]:
        return False
    # HH_261004 - Global hooks can see/mutate the encoder input even when local hook dictionaries are empty.
    for name in ("_global_forward_pre_hooks", "_global_forward_hooks", "_global_backward_hooks"):
        if getattr(torch_module, name, {}):
            return False
    expected = {"": _CachedSpatialImageEncoderV2, "features": nn.Sequential,
                "pool": nn.AvgPool2d, "projection": nn.Conv2d}
    expected.update({"features." + str(index): (nn.Conv2d, nn.GroupNorm, nn.SiLU)[index % 3]
                     for index in range(12)})
    modules = dict(encoder.named_modules())
    if set(modules) != set(expected):
        return False
    for name, child in modules.items():
        kind = expected[name]
        if type(child) is not kind or child.training:
            return False
        if getattr(child.forward, "__func__", None) is not _TRUSTED_FORWARDS[kind]:
            return False
        if any(key in vars(child) for key in ("forward", "_call_impl", "__call__", "_conv_forward")):
            return False
        if child._forward_hooks or child._forward_pre_hooks or child._backward_hooks:
            return False
        if getattr(child, "inplace", False):
            return False
        if kind is nn.Conv2d and nn.Conv2d._conv_forward is not _TRUSTED_CONV_FORWARD:
            return False
    return True


def select_masked_images(model, images: Tensor, effective_valid: Tensor) -> Tensor:
    """HH_261004 - Reuse only active all-valid trusted cached inputs; otherwise execute the old expression."""
    encoder = model.image_encoder
    # HH_261004 - Original masked_fill materializes standard contiguous layout; preserve that CNN layout as well as bytes.
    if (type(images) is Tensor and images.is_contiguous() and not torch.is_grad_enabled()
            and type(encoder) is _CachedSpatialImageEncoderV2 and encoder.request is not None
            and bool(effective_valid.all().item()) and _trusted_readonly_encoder(encoder)):
        return images
    return images.masked_fill(~effective_valid[:, :, None, None, None, None], 0.0)


def _forward_tree() -> ast.Module:
    """HH_261004 - Read the pinned original method rather than copying altered in-memory methods."""
    require_frozen_parent_sources()
    source = ast.parse(Path(__file__).with_name("temporal_model_v2.py").read_text(encoding="utf-8"))
    model_class = next(node for node in source.body if isinstance(node, ast.ClassDef) and node.name == "TemporalModelV2")
    method = next(node for node in model_class.body if isinstance(node, ast.FunctionDef) and node.name == "forward")
    return ast.Module(body=[copy.deepcopy(method)], type_ignores=[])


def build_candidate_forward():
    """HH_261004 - Prove an invertible one-assignment edit before compiling identical remaining math."""
    original = _forward_tree()
    changed = copy.deepcopy(original)
    old = ast.parse("masked_images = images.masked_fill(~effective_valid[:, :, None, None, None, None], 0.0)").body[0]
    new = ast.parse("masked_images = select_masked_images(self, images, effective_valid)").body[0]
    original_dump = ast.dump(original, include_attributes=False)
    matches = [index for index, node in enumerate(changed.body[0].body)
               if ast.dump(node, include_attributes=False) == ast.dump(old, include_attributes=False)]
    if len(matches) != 1:
        raise ContractError("masked-copy candidate requires exactly one original masked_images assignment")
    changed.body[0].body[matches[0]] = copy.deepcopy(new)
    restored = copy.deepcopy(changed)
    restored.body[0].body[matches[0]] = copy.deepcopy(old)
    if ast.dump(restored, include_attributes=False) != original_dump:
        raise ContractError("masked-copy inverse AST differs from the complete original forward")
    namespace = dict(TemporalModelV2.forward.__globals__)
    namespace["select_masked_images"] = select_masked_images
    exec(compile(ast.fix_missing_locations(changed), str(Path(__file__)), "exec"), namespace)
    result = namespace["forward"]
    result.__doc__ = "HH_261004 - Original forward with only the guarded masked_images assignment changed."
    result._research_original_ast_sha256 = hashlib.sha256(original_dump.encode()).hexdigest()
    result._research_inverse_ast_equal = True
    return result


class AllValidMaskedCopyTemporalModelV2(TemporalModelV2):
    """HH_261004 - Insert the new forward after cached lexical super, without changing original classes."""
    forward = build_candidate_forward()

    def __init__(self, *args, **kwargs):
        require_frozen_parent_sources()
        super().__init__(*args, **kwargs)


class AllValidMaskedCopyCachedTemporalModelV2(CachedTemporalModelV2, AllValidMaskedCopyTemporalModelV2):
    """HH_261004 - Keep the original cached API, hashes, finite scans and cache lifecycle unmodified."""


class AllValidMaskedCopyCPUFiniteCachedTemporalModelV2(CPUFiniteCachedTemporalModelV2, AllValidMaskedCopyTemporalModelV2):
    """HH_261004 - Preserve both CPU finite validations via the original validator MRO."""


class AllValidMaskedCopyRouteRecoveryTemporalModelV3(RouteRecoveryTemporalModelV3, AllValidMaskedCopyTemporalModelV2):
    """HH_261004 - Preserve the recovery serial context/permanent head hook and cached lexical super."""
