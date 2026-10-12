"""HH_260906 - Research-only shared candidate-geometry residual for the existing STOPMIX scorer."""
import torch
from torch import nn
from .model import PerspectiveTrajectoryModel, PHYSICAL_STOPMIX_MODEL_ID


class CandidateGeometryRecoveryModel(PerspectiveTrajectoryModel):
    """HH_260906 - Preserve initial predictions and all DRIVE/STOP candidates; refine existing logits only."""
    def __init__(self, config):
        super().__init__(config)
        if config.model_id != PHYSICAL_STOPMIX_MODEL_ID:
            raise ValueError('L research wrapper requires the unchanged STOPMIX base configuration')
        self.geometry_ranker = nn.Sequential(
            nn.Linear(config.hidden_width + config.future_points * 3, 128), nn.ReLU(), nn.Linear(128, 1))
        nn.init.zeros_(self.geometry_ranker[-1].weight)
        nn.init.zeros_(self.geometry_ranker[-1].bias)
        self._rank_context = None
        self.fusion.register_forward_hook(self._capture_context)

    def _capture_context(self, module, inputs, output):
        """HH_260906 - Capture only current/past sensor and navigation context from the unchanged forward pass."""
        self._rank_context = output

    def geometry_residual(self, context, xy, speed):
        """HH_260906 - Share weights across candidate slots and stop direct rank gradients into decoded geometry."""
        geometry = torch.cat((xy.detach()/120.0, speed.detach().unsqueeze(-1)/(30.0/3.6)),dim=-1).flatten(2)
        context = context[:,None].expand(-1,xy.shape[1],-1)
        return self.geometry_ranker(torch.cat((context,geometry),dim=-1)).squeeze(-1)

    def forward(self, *args, **kwargs):
        """HH_260906 - Add no ground truth, episode identifiers or future sensor inputs to inference."""
        self._rank_context = None
        xy, speed, logits = super().forward(*args, **kwargs)
        context = self._rank_context
        self._rank_context = None
        if context is None:
            raise RuntimeError('base fusion context was not produced')
        return xy, speed, logits + self.geometry_residual(context,xy,speed)
