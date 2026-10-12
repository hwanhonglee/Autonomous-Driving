"""HH_261001 - TRAIN-only pre-clamp acceleration gradients; physical decoding remains unchanged."""

from contextlib import contextmanager

import torch
from torch.nn import functional as F

from .model import PHYSICAL_MAXIMUM_ACCELERATION_MPS2, PHYSICAL_MAXIMUM_SPEED_MPS


POINTS = 5
WEIGHT = 0.05


def initial_acceleration_loss(raw_drive, current_speed, target_speed, valid, oracle):
    """HH_261001 - Fit representable adjacent speed differences only for the full-horizon DRIVE oracle."""
    batch = raw_drive.shape[0]
    if (tuple(raw_drive.shape) != (batch, 6, 64, 2)
            or tuple(current_speed.shape) != (batch,)
            or tuple(target_speed.shape) != (batch, 64)
            or tuple(valid.shape) != (batch, 64)
            or tuple(oracle.shape) != (batch,)
            or valid.dtype != torch.bool or oracle.dtype != torch.long):
        raise ValueError("fixed STOPMIX raw/target/oracle ABI required")
    if any(value.device != raw_drive.device for value in (current_speed, target_speed, valid, oracle)):
        raise ValueError("all acceleration supervision tensors must share a device")
    if (not bool(torch.isfinite(raw_drive).all()) or not bool(torch.isfinite(current_speed).all())
            or not bool(torch.isfinite(target_speed[valid]).all())
            or bool(((oracle < 0) | (oracle >= 12)).any())
            or bool((~valid[:, :-1] & valid[:, 1:]).any())):
        raise ValueError("finite inputs, valid oracle and contiguous target masks required")
    # HH_261001 - No pseudo-label clipping: impossible or reverse-current pairs receive no new auxiliary.
    previous = torch.cat((current_speed[:, None], target_speed[:, :POINTS - 1]), dim=1).detach()
    future = target_speed[:, :POINTS].detach()
    adjacent_valid = valid[:, :POINTS] & torch.cat((torch.ones_like(valid[:, :1]), valid[:, :POINTS - 1]), dim=1)
    acceleration = (future - previous) / .1
    eligible = (adjacent_valid & torch.isfinite(acceleration)
                & (previous >= 0) & (previous <= PHYSICAL_MAXIMUM_SPEED_MPS)
                & (future >= 0) & (future <= PHYSICAL_MAXIMUM_SPEED_MPS)
                & (acceleration.abs() <= PHYSICAL_MAXIMUM_ACCELERATION_MPS2)
                & (oracle[:, None] < 6))
    row = torch.arange(batch, device=raw_drive.device)
    selected = raw_drive[row, oracle.detach().clamp(max=5), :POINTS, 0]
    predicted = torch.tanh(selected) * PHYSICAL_MAXIMUM_ACCELERATION_MPS2
    # HH_261001 - Invalid annotation payloads cannot contaminate the zero-masked loss.
    safe_target = torch.where(eligible, acceleration, torch.zeros_like(acceleration))
    error = F.smooth_l1_loss(predicted, safe_target, reduction="none")
    mask = eligible.to(error.dtype)
    loss = (error * mask).sum() / mask.sum().clamp_min(1.)
    metrics = dict(eligible_pairs=eligible.sum().detach(),
        acceleration_pairs=(eligible & (acceleration > .1)).sum().detach(),
        deceleration_pairs=(eligible & (acceleration < -.1)).sum().detach(),
        hold_pairs=(eligible & (acceleration.abs() <= .1)).sum().detach(),
        drive_oracle_rows=(oracle < 6).sum().detach(),
        clamp_dead_pairs=(eligible & (previous + predicted.detach() * .1 < 0)).sum().detach())
    return loss, metrics


@contextmanager
def capture_drive_latents(model):
    """HH_261001 - Observe the existing head without replacing modules or changing forward outputs."""
    outputs = []

    def record(module, inputs, output):
        if output.ndim != 2 or output.shape[1] != 6 * 64 * 2:
            raise ValueError("the unchanged six-DRIVE head ABI is required")
        outputs.append(output.reshape(output.shape[0], 6, 64, 2))

    handle = model.trajectory_head.register_forward_hook(record)
    try:
        yield outputs
    finally:
        handle.remove()
