#!/usr/bin/env python3

from dataclasses import dataclass
import math
from typing import Iterable


@dataclass(frozen=True)
class SweepCase:
    case_id: str
    kind: str
    level: float
    repeat: int


def _validated_levels(values: Iterable[float], label: str) -> tuple[float, ...]:
    levels = tuple(float(value) for value in values)
    if not levels:
        raise ValueError(f"{label} levels must not be empty")
    if any(not math.isfinite(value) or value <= 0.0 or value > 1.0 for value in levels):
        raise ValueError(f"{label} levels must be finite and within (0, 1]")
    if len(set(levels)) != len(levels):
        raise ValueError(f"{label} levels must be unique")
    return levels


def build_sweep_cases(
    accel_levels: Iterable[float], brake_levels: Iterable[float], repeats: int
) -> tuple[SweepCase, ...]:
    if repeats < 1:
        raise ValueError("repeats must be positive")
    accel = _validated_levels(accel_levels, "accel")
    brake = _validated_levels(brake_levels, "brake")

    cases = []
    for repeat in range(1, repeats + 1):
        cases.append(SweepCase(f"r{repeat:02d}_coast", "coast", 0.0, repeat))
        cases.extend(
            SweepCase(f"r{repeat:02d}_accel_{level:.3f}", "accel", level, repeat)
            for level in accel
        )
        cases.extend(
            SweepCase(f"r{repeat:02d}_brake_{level:.3f}", "brake", level, repeat)
            for level in brake
        )
    return tuple(cases)


def sample_is_fit_eligible(
    *,
    phase: str,
    sample_elapsed_s: float,
    response_settle_s: float,
    requested_accel: float,
    requested_brake: float,
    applied_accel: float | None,
    applied_brake: float | None,
    command_tolerance: float,
    velocity_mps: float | None,
    minimum_velocity_mps: float,
    maximum_velocity_mps: float,
    steering_rad: float | None,
    maximum_steering_rad: float,
    status_age_wall_s: float,
    status_timeout_wall_s: float,
    watchdog_active: bool,
    safety_ok: bool,
) -> bool:
    if phase != "sample" or sample_elapsed_s < response_settle_s:
        return False
    if applied_accel is None or applied_brake is None or velocity_mps is None:
        return False
    if steering_rad is None or abs(steering_rad) > maximum_steering_rad:
        return False
    if not minimum_velocity_mps <= velocity_mps <= maximum_velocity_mps:
        return False
    if status_age_wall_s > status_timeout_wall_s or watchdog_active or not safety_ok:
        return False
    return (
        abs(applied_accel - requested_accel) <= command_tolerance
        and abs(applied_brake - requested_brake) <= command_tolerance
    )


def reverse_velocity_is_safe(
    *,
    phase: str,
    velocity_mps: float,
    maximum_reverse_velocity_mps: float,
    maximum_settling_reverse_velocity_mps: float,
) -> bool:
    if not math.isfinite(velocity_mps):
        return False
    limit = (
        maximum_settling_reverse_velocity_mps
        if phase in {"stop_before_reset", "reset_settle"}
        else maximum_reverse_velocity_mps
    )
    if not math.isfinite(limit) or limit < 0.0:
        raise ValueError("reverse velocity limits must be finite and non-negative")
    return velocity_mps >= -limit


def initialpose_input_z(reference_z_m: float, interface_z_offset_m: float) -> float:
    if not math.isfinite(reference_z_m):
        raise ValueError("reference z must be finite")
    if not math.isfinite(interface_z_offset_m) or interface_z_offset_m < 0.0:
        raise ValueError("interface z offset must be finite and non-negative")
    return reference_z_m - interface_z_offset_m


def update_stability_window(
    *,
    anchor_value: float | None,
    stable_since_s: float | None,
    current_value: float,
    current_time_s: float,
    maximum_delta: float,
    required_duration_s: float,
) -> tuple[float, float, bool]:
    values = (current_value, current_time_s, maximum_delta, required_duration_s)
    if any(not math.isfinite(value) for value in values):
        raise ValueError("stability window values must be finite")
    if maximum_delta <= 0.0 or required_duration_s <= 0.0:
        raise ValueError("stability window limits must be positive")
    if anchor_value is None or stable_since_s is None:
        return current_value, current_time_s, False
    if not math.isfinite(anchor_value) or not math.isfinite(stable_since_s):
        raise ValueError("stability window state must be finite")
    if current_time_s < stable_since_s or abs(current_value - anchor_value) > maximum_delta:
        return current_value, current_time_s, False
    return (
        anchor_value,
        stable_since_s,
        current_time_s - stable_since_s >= required_duration_s,
    )
