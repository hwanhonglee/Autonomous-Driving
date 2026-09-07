"""HH_260906 - Freeze the second low-speed identification matrix before simulator execution."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math


MATRIX_ID = "low_speed_v2"
MATRIX_SCHEMA = "carla.low_speed_response_matrix.v1"
PHYSICS_HZ = 20.0


@dataclass(frozen=True)
class IdentificationCase:
    """HH_260906 - Repeats share an initial condition and are not independent routes or training episodes."""

    case_id: str
    kind: str
    level: float
    repeat: int
    hold_seconds: float
    ramp_rate_per_second: float | None = None


def identification_matrix() -> tuple[IdentificationCase, ...]:
    """HH_260906 - Prioritize missing coast baselines, then narrow-margin launch repeats and declared ramps."""
    cases = []
    for repeat in (1, 2):
        cases.append(IdentificationCase(f"{len(cases) + 1:02d}_coast_repeat_{repeat:02d}",
                                        "coast", 0.0, repeat, 20.0))
    for repeat in (1, 2, 3):
        cases.append(IdentificationCase(f"{len(cases) + 1:02d}_throttle_0.15_repeat_{repeat:02d}",
                                        "throttle", 0.15, repeat, 8.0))
    for rate in (0.01, 0.025, 0.05, 0.10):
        cases.append(IdentificationCase(f"{len(cases) + 1:02d}_throttle_ramp_{rate:.3f}_per_second",
                                        "throttle_ramp", 0.40, 1, 8.0, rate))
    return tuple(cases)


def hold_control(case: IdentificationCase, tick_index: int) -> tuple[float, float]:
    """HH_260906 - Advance the normalized throttle ramp by one disclosed 50 ms interval per command."""
    if (isinstance(tick_index, bool) or not isinstance(tick_index, int)
            or not 0 <= tick_index < round(case.hold_seconds * PHYSICS_HZ)):
        raise ValueError("hold tick index lies outside the declared case duration")
    if case.kind == "coast":
        return 0.0, 0.0
    if case.kind == "throttle":
        return case.level, 0.0
    if case.kind == "throttle_ramp":
        if case.ramp_rate_per_second is None or not math.isfinite(case.ramp_rate_per_second) or case.ramp_rate_per_second <= 0:
            raise ValueError("ramp case needs a positive finite rate")
        return min(case.level, case.ramp_rate_per_second * (tick_index + 1) / PHYSICS_HZ), 0.0
    raise ValueError("unknown identification case kind")


def matrix_contract() -> dict:
    """HH_260906 - Separate identification completion from motion quality and autonomous-driving claims."""
    return {
        "schema": MATRIX_SCHEMA, "matrix_id": MATRIX_ID,
        "cases": [asdict(case) for case in identification_matrix()],
        "coast_prepare": {"throttle": 0.30, "measured_speed_threshold_mps": 3.0,
                          "maximum_seconds": 15.0, "entry_speed_is_recorded_not_assumed_exact": True},
        "settle_seconds": 3.5, "physics_hz": PHYSICS_HZ,
        "ramp_policy": {
            "initial_throttle": 0.0,
            "command_formula": "min(0.40, rate_per_second * (zero_based_hold_tick + 1) / 20.0)",
            "hold_ticks": 160, "rates_per_second": [0.01, 0.025, 0.05, 0.10],
            "final_commanded_throttles": [0.08, 0.20, 0.40, 0.40],
            "all_ramps_reach_cap": False,
        },
        "constant_launch_repeat_count": 3, "coast_repeat_count": 2,
        "fresh_vehicle_each_case": True, "independent_route_claim": False,
        "manual_gear_changes": False, "physics_parameter_changes": False,
        "simultaneous_throttle_and_brake": False, "training_data": False,
        "automatic_quality_promotion": False,
        "notice": "Repeated initial conditions identify launch and zero-pedal behavior; no chosen profile is declared safe by this matrix alone.",
    }
