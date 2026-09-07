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


@dataclass(frozen=True)
class TwoStageCase(IdentificationCase):
    """HH_260906 - Keep extra handoff settings out of the immutable v2 case serialization."""

    handoff_speed_mps: float = 0.5
    launch_throttle: float = 0.15
    launch_maximum_seconds: float = 8.0
    measured_speed_stop_mps: float = 30.0 / 3.6


def progressive_identification_matrix() -> tuple[IdentificationCase, ...]:
    """HH_260906 - Freeze two longer coast repeats and four measured-handoff experiments as v3."""
    cases = [IdentificationCase(f"{repeat:02d}_coast_45s_repeat_{repeat:02d}",
                                "coast", 0.0, repeat, 45.0) for repeat in (1, 2)]
    for threshold in (0.5, 1.0):
        for rate in (0.05, 0.10):
            cases.append(TwoStageCase(
                f"{len(cases) + 1:02d}_handoff_{threshold:.1f}_ramp_{rate:.2f}",
                "two_stage_ramp", 0.40, 1, 12.0, rate, threshold))
    return tuple(cases)


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
    if case.kind == "two_stage_ramp" and isinstance(case, TwoStageCase):
        if case.ramp_rate_per_second is None or not math.isfinite(case.ramp_rate_per_second) or case.ramp_rate_per_second <= 0:
            raise ValueError("two-stage case needs a positive finite rate")
        return min(case.level, case.launch_throttle + case.ramp_rate_per_second * (tick_index + 1) / PHYSICS_HZ), 0.0
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


def progressive_matrix_contract() -> dict:
    """HH_260906 - Declare identification completion separately from stop dwell and driving-quality success."""
    return {
        "schema": MATRIX_SCHEMA, "matrix_id": "low_speed_v3",
        "cases": [asdict(case) for case in progressive_identification_matrix()],
        "settle_seconds": 3.5, "physics_hz": PHYSICS_HZ,
        "coast_prepare": {"throttle": 0.30, "measured_speed_threshold_mps": 3.0,
                          "maximum_seconds": 15.0, "entry_speed_is_recorded_not_assumed_exact": True},
        "coast_hold_seconds": 45.0, "coast_hold_ticks": 900,
        "coast_stop_observation": {"maximum_speed_mps": 0.10, "continuous_seconds": 2.0,
                                   "brake_after_threshold": False, "hold_stops_early": False,
                                   "failure_to_stop_does_not_discard_measurements": True},
        "two_stage_policy": {
            "launch_throttle": 0.15, "launch_maximum_seconds": 8.0, "launch_maximum_ticks": 160,
            "handoff_speed_thresholds_mps": [0.5, 1.0], "handoff_rule": "first measured speed >= threshold; fail retained case if absent within 160 launch ticks",
            "ramp_rates_per_second": [0.05, 0.10], "throttle_cap": 0.40,
            "first_command_formula": "0.15 + rate_per_second * 0.05",
            "command_formula": "min(0.40, 0.15 + rate_per_second * (zero_based_post_handoff_tick + 1) / 20.0)",
            "maximum_post_handoff_seconds": 12.0, "maximum_post_handoff_ticks": 240,
            "measured_speed_stop_mps": 30.0 / 3.6,
            "completion_rule": "first post-handoff measured speed >= 30/3.6 m/s or 240 post-handoff ticks, whichever occurs first",
            "time_limit_without_30kph_is_measurement_complete_not_cruise_success": True,
            "termination_actuation": "destroy owned actor; no synthetic brake, velocity reset, or unrecorded cruise",
        },
        "fresh_vehicle_each_case": True, "independent_route_claim": False,
        "manual_gear_changes": False, "physics_parameter_changes": False,
        "simultaneous_throttle_and_brake": False, "training_data": False,
        "automatic_quality_promotion": False,
        "notice": "Keep every startup/transition/stop sample and unchanged scalar bounds; six identification cases do not select a comfortable driving profile.",
    }
