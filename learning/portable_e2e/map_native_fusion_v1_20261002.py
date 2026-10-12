"""HH_261002 - Research-only native-feature fusion; no new input, class or head.

The shared quarter-resolution context model remains byte-identical at loading.
The new half-resolution residual ends in an exactly zero transpose projection.
This is prepared architecture code, not measured perception or runtime approval.
"""
from __future__ import annotations

import torch
from torch import nn

try:
    from .map_context_recovery import MapContextRecovery
except ImportError:
    from map_context_recovery import MapContextRecovery

SCHEMA = "portable_e2e.map_native_feature_fusion.20261002.v1"
PARENT_PARAMETERS, NEW_PARAMETERS, PARAMETERS = 245831, 151344, 397175


class MapNativeFusion(MapContextRecovery):
    def __init__(self):
        super().__init__()
        layers = [nn.Conv2d(48, 64, 3, stride=2, padding=1), nn.GroupNorm(4, 64), nn.ReLU()]
        # HH_261002 - Fuse native skip features nonlinearly at half resolution, not new labels or a second classifier.
        for dilation in (1, 2, 4):
            layers.extend((nn.Conv2d(64, 64, 3, padding=dilation, dilation=dilation),
                           nn.GroupNorm(4, 64), nn.ReLU()))
        projection = nn.ConvTranspose2d(64, 48, 2, stride=2)
        nn.init.zeros_(projection.weight)
        nn.init.zeros_(projection.bias)
        layers.append(projection)
        self.native_fusion = nn.Sequential(*layers)
        if sum(p.numel() for p in self.native_fusion.parameters()) != NEW_PARAMETERS:
            raise ValueError("new residual parameter ABI differs")
        if sum(p.numel() for p in self.parameters()) != PARAMETERS:
            raise ValueError("full fusion parameter ABI differs")

    def load_current_parent(self, state):
        # HH_261002 - Only the new branch is absent; forbid replacing the shared parent or loading another architecture.
        template = self.state_dict()
        shared = {k for k in template if not k.startswith("native_fusion.")}
        if set(state) != shared:
            raise ValueError("exact current context-parent keyset required")
        for key, tensor in state.items():
            if (not isinstance(tensor, torch.Tensor) or tensor.dtype != torch.float32
                    or tensor.shape != template[key].shape or not bool(torch.isfinite(tensor).all())):
                raise ValueError("finite float32 current-parent tensor required: " + key)
        result = self.load_state_dict(state, strict=False)
        if set(result.missing_keys) != set(template) - shared or result.unexpected_keys:
            raise ValueError("only the new fusion branch may be missing")
        if not all(torch.equal(self.state_dict()[k].cpu(), v.detach().cpu()) for k, v in state.items()):
            raise ValueError("shared current-parent values changed")
        projection = self.native_fusion[-1]
        if bool(projection.weight.detach().count_nonzero()) or bool(projection.bias.detach().count_nonzero()):
            raise ValueError("initial residual output must be exactly zero")

    def forward(self, images):
        a = self.shallow(images)
        b = self.middle(a)
        c = self.deep(b)
        c = c + self.context(c)
        d = self.decode(torch.cat((b, self.up_deep(c)[..., :b.shape[-2], :b.shape[-1]]), 1))
        native = torch.cat((a, self.up_middle(d)[..., :a.shape[-2], :a.shape[-1]]), 1)
        residual = self.native_fusion(native)[..., :native.shape[-2], :native.shape[-1]]
        return self.output(native + residual)
