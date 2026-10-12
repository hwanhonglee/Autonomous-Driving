"""HH_260906 - TRAIN-only near-horizon loss and paired image/label augmentation."""
import numpy as np
import torch
from torch.nn import functional as F


def near_horizon_loss(xy, speed, target_xy, target_speed, valid, oracle, points=20):
    """HH_260906 - Refine the full-horizon winner, never choose a conflicting short-horizon mode."""
    if not 1 <= points <= xy.shape[2] or valid.dtype != torch.bool:
        raise ValueError('valid horizon and boolean mask required')
    rows = torch.arange(xy.shape[0], device=xy.device)
    mask = valid[:, :points].to(xy.dtype)
    if not bool((mask.sum(1) > 0).all()):
        raise ValueError('every sample needs a valid near-horizon target')
    error = F.smooth_l1_loss(xy[rows, oracle.detach(), :points], target_xy[:, :points], reduction='none').sum(-1)
    error = error + .2 * F.smooth_l1_loss(speed[rows, oracle.detach(), :points], target_speed[:, :points], reduction='none')
    return ((error * mask).sum(1) / mask.sum(1)).mean()


def paired_crop(rgb, target, rng, enabled=True):
    """HH_260906 - TRAIN-only aligned zoom; no flip or fabricated camera geometry."""
    from PIL import Image
    if rgb.shape[:2] != target.shape or rgb.dtype != np.uint8 or target.dtype != np.uint8:
        raise ValueError('paired uint8 RGB/label arrays required')
    if not enabled or rng.random() >= .5:
        return rgb.copy(), target.copy()
    height, width = target.shape
    scale = float(rng.uniform(.65, 1.))
    h, w = max(1, int(height*scale)), max(1, int(width*scale))
    top, left = int(rng.integers(height-h+1)), int(rng.integers(width-w+1))
    resample = getattr(Image, 'Resampling', Image)
    x = Image.fromarray(rgb[top:top+h,left:left+w]).resize((width,height),resample.BILINEAR)
    y = Image.fromarray(target[top:top+h,left:left+w]).resize((width,height),resample.NEAREST)
    return np.asarray(x).copy(), np.asarray(y).copy()
