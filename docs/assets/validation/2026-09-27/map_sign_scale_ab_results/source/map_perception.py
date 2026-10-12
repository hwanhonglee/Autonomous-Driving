"""HH_260906 - Small RGB-only semantic baseline, not a complete HD mapping model."""
import torch
from torch import nn
from torch.nn import functional as F

LABELS=("other", "road", "sidewalk", "lane_marking", "traffic_light_fixture", "traffic_sign", "pole")


def semantic_loss(logits,target,weights):
    """HH_260906 - Equivalent weighted CE without CUDA's nondeterministic spatial NLL kernel."""
    log_prob=F.log_softmax(logits,dim=1)
    selected=log_prob.gather(1,target.unsqueeze(1)).squeeze(1)
    pixel_weight=weights[target]
    return -(selected*pixel_weight).sum()/pixel_weight.sum()


class MapPerception(nn.Module):
    def __init__(self):
        super().__init__()
        def block(cin,cout,stride=1):
            return nn.Sequential(nn.Conv2d(cin,cout,3,stride,1),nn.GroupNorm(4,cout),nn.ReLU(),
                                 nn.Conv2d(cout,cout,3,1,1),nn.GroupNorm(4,cout),nn.ReLU())
        self.shallow=block(3,16)
        self.middle=block(16,32,2)
        self.deep=block(32,64,2)
        # HH_260906 - Avoid CUDA bilinear-backward incompatibility with strict deterministic training.
        self.up_deep=nn.ConvTranspose2d(64,64,2,2)
        self.up_middle=nn.ConvTranspose2d(32,32,2,2)
        self.decode=block(96,32)
        self.output=nn.Conv2d(48,len(LABELS),1)

    def forward(self,images):
        a=self.shallow(images);b=self.middle(a);c=self.deep(b)
        d=self.decode(torch.cat([b,self.up_deep(c)[...,:b.shape[-2],:b.shape[-1]]],1))
        return self.output(torch.cat([a,self.up_middle(d)[...,:a.shape[-2],:a.shape[-1]]],1))
