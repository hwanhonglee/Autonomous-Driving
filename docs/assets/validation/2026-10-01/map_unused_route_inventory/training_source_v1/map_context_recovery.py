"""HH_261001 - Add identity-initialized spatial context to the research map model."""

import torch
from torch import nn

try:
    from .map_perception import MapPerception
except ImportError:
    from map_perception import MapPerception


class MapContextRecovery(MapPerception):
    """Expand image context without adding classes or changing the input contract."""

    def __init__(self):
        super().__init__()
        layers = []
        # HH_261001 - Work at quarter resolution; preserve native-resolution skip features.
        for dilation in (2, 4, 8):
            layers.extend((nn.Conv2d(64, 64, 3, padding=dilation, dilation=dilation),
                           nn.GroupNorm(4, 64), nn.ReLU()))
        projection = nn.Conv2d(64, 64, 1)
        nn.init.zeros_(projection.weight)
        nn.init.zeros_(projection.bias)
        layers.append(projection)
        self.context = nn.Sequential(*layers)

    def load_baseline(self, state):
        # HH_261001 - Only the new residual branch may be absent from a baseline checkpoint.
        result = self.load_state_dict(state, strict=False)
        expected = {name for name in self.state_dict() if name.startswith("context.")}
        if set(result.missing_keys) != expected or result.unexpected_keys:
            raise ValueError("baseline checkpoint does not match the seven-class model")

    def forward(self, images):
        a = self.shallow(images)
        b = self.middle(a)
        c = self.deep(b)
        c = c + self.context(c)
        d = self.decode(torch.cat((b, self.up_deep(c)[..., :b.shape[-2], :b.shape[-1]]), 1))
        return self.output(torch.cat((a, self.up_middle(d)[..., :a.shape[-2], :a.shape[-1]]), 1))
