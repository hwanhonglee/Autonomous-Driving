"""HH_260906 - Research-only path/progress decoder; every output still requires the unchanged runtime gate."""
import torch
from portable_e2e.model import PerspectiveTrajectoryModel


def interpolate_route(route, mask, distance):
    """HH_260906 - Sample the supplied route by arc progress without reading future targets."""
    delta=route[:,1:]-route[:,:-1]
    length=torch.linalg.vector_norm(delta,dim=-1)
    valid=mask[:,1:] & mask[:,:-1]
    length=torch.where(valid,length,torch.zeros_like(length))
    arc=torch.cat((torch.zeros_like(length[:,:1]),length.cumsum(-1)),dim=-1)
    count=(mask.sum(-1)-2).clamp(min=0)
    s=distance.clamp(min=0)
    s=torch.minimum(s,arc[:,-1,None,None])
    index=(s[...,None]>arc[:,None,None,1:]).sum(-1)
    index=torch.minimum(index,count[:,None,None])
    batch=torch.arange(route.shape[0],device=route.device)[:,None,None]
    start=route[batch,index];end=route[batch,index+1]
    ratio=((s-arc[batch,index])/length[batch,index].clamp(min=1e-6)).clamp(0,1)
    return start+(end-start)*ratio[...,None]


class ArcProgressResearchModel(PerspectiveTrajectoryModel):
    """HH_260906 - Retain existing bounded speed integration; isolate route-position construction only."""
    def _decode_physical_v1(self,raw,ego_history,route_xy,route_mask):
        # HH_260906 - This ablation deliberately ignores lateral latent; it is not a deployable replacement.
        _,speed=super()._decode_physical_v1(raw,ego_history,route_xy,route_mask)
        progress=(speed*.1).cumsum(-1)
        xy=interpolate_route(route_xy,route_mask,progress)
        return xy,speed
