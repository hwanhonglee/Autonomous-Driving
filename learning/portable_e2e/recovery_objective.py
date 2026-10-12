"""HH_260906 - TRAIN-only soft ADE ranking and conditional good-teacher retention."""
import torch
from torch.nn import functional as F


def recovery_terms(xy, speed, logits, teacher, target_xy, target_speed, valid):
    """Never use DEV or runtime future truth; teacher is frozen original K1."""
    mask = valid.to(xy.dtype)
    denom = mask.sum(-1).clamp_min(1)
    ade = (torch.norm(xy-target_xy[:, None], dim=-1)*mask[:, None]).sum(-1)/denom[:, None]
    speed_mae = ((speed-target_speed[:, None]).abs()*mask[:, None]).sum(-1)/denom[:, None]
    # HH_260906 - Detach targets so ranking cannot improve by worsening unselected candidates.
    cost = (ade + .2*speed_mae).detach()
    soft_target = torch.softmax(-(cost-cost.min(-1, keepdim=True).values)/.5, -1)
    rank = -(soft_target*F.log_softmax(logits, -1)).sum(-1).mean()
    tx, ts, tl = (t.detach() for t in teacher)
    idx = tl.argmax(-1)
    row = torch.arange(xy.shape[0], device=xy.device)
    teacher_xy, teacher_speed = tx[row, idx], ts[row, idx]
    tade = (torch.norm(teacher_xy-target_xy,dim=-1)*mask).sum(-1)/denom
    tspeed = ((teacher_speed-target_speed).abs()*mask).sum(-1)/denom
    eligible = ((tade < .75) & (tspeed < .5)).to(xy.dtype)
    per_xy = (F.smooth_l1_loss(xy[row, idx], teacher_xy, reduction="none").sum(-1)*mask).sum(-1)/denom
    per_speed = (F.smooth_l1_loss(speed[row, idx], teacher_speed, reduction="none")*mask).sum(-1)/denom
    per_logits = -(F.softmax(tl,-1)*F.log_softmax(logits,-1)).sum(-1)
    preserve = ((per_xy+.2*per_speed+.1*per_logits)*eligible).sum()/eligible.sum().clamp_min(1)
    return rank, preserve, eligible.sum().detach()
