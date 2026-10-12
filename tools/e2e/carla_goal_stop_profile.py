"""HH_260906 - Pure opt-in goal-stop planning and measured capture quality checks."""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import hashlib
import math
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence


@dataclass(frozen=True)
class ComfortableGoalStopConfig:
    """HH_260906 - Freeze this first trial; commanded comfort is not measured feasibility."""

    profile_id: str = "comfortable_v1"
    desired_deceleration_mps2: float = 1.5
    target_acceleration_limit_mps2: float = 1.5
    stop_buffer_m: float = 0.75
    goal_tolerance_m: float = 1.0
    stopped_speed_mps: float = 0.1
    hold_seconds: float = 2.0
    normal_brake_cap: float = 0.10
    normal_throttle_cap: float | None = None
    minimum_tail_seconds: float = 6.5
    maximum_projection_step_m: float = 1.0
    maximum_projection_error_m: float = 3.0


@dataclass(frozen=True)
class DevelopmentGoalStopConfig(ComfortableGoalStopConfig):
    """HH_260906 - Freeze a Town07-only development pilot, not an admitted training profile."""

    profile_id: str = "comfortable_v3"
    desired_deceleration_mps2: float = 0.6
    target_acceleration_limit_mps2: float = 1.0
    normal_brake_cap: float = 0.10
    normal_throttle_cap: float | None = 0.40
    nominal_cruise_speed_mps: float = 8.0
    maximum_actual_speed_mps: float = 8.333333333333334
    launch_throttle: float = 0.15
    launch_handoff_speed_mps: float = 0.5
    maximum_launch_seconds: float = 8.0
    post_handoff_throttle_ramp_per_second: float = 0.05
    approach_reference_speed_mps: float = 3.0396464855480536
    approach_reference_distance_m: float = 35.0
    coast_reference_seconds: float = 6.93
    minimum_coast_entry_speed_mps: float = 2.8
    maximum_coast_entry_speed_mps: float = 3.2
    maximum_coast_seconds: float = 45.0
    cruise_minimum_speed_mps: float = 7.8
    cruise_maximum_speed_mps: float = 8.2
    minimum_cruise_seconds: float = 5.0
    route_sha256: str = "8285e70a790d5e8ae75803db1aa538122b56e6fcfc3586e60eaa947d330417c9"
    route_length_m: float = 210.5975914062836
    maximum_attempts_per_revision: int = 2
    development_only: bool = True
    training_data_approved: bool = False
    empirical_reference_notice: str = "Single-condition coast reference; not a robust stopping-distance guarantee."


@dataclass(frozen=True)
class BrakeFreeDevelopmentGoalStopConfig(DevelopmentGoalStopConfig):
    """HH_260906 - Isolate zero normal braking; emergency and stopped-tail braking stay unchanged."""

    profile_id: str = "comfortable_v4"
    normal_brake_cap: float = 0.0


@dataclass(frozen=True)
class BrakeHoldDevelopmentGoalStopConfig(BrakeFreeDevelopmentGoalStopConfig):
    """HH_260906 - Apply service braking after a measured near-goal low-speed entry, before declaring stopped dwell."""

    profile_id: str = "comfortable_v5_brake_hold"
    normal_hold_brake: float = 1.0
    maximum_attempts_per_revision: int = 1


@dataclass(frozen=True)
class LowSpeedSteeringDevelopmentGoalStopConfig(BrakeHoldDevelopmentGoalStopConfig):
    """HH_260906 - Compare emitted steering only; retain v5 physics, hold, route and quality bounds."""

    profile_id: str = "comfortable_v6_low_speed_steering"
    low_speed_steering_mode: str = "native"
    low_speed_steering_threshold_mps: float = 0.75
    maximum_attempts_per_revision: int = 4
    research_only: bool = True


@dataclass(frozen=True)
class HoldBrakeRampDevelopmentGoalStopConfig(LowSpeedSteeringDevelopmentGoalStopConfig):
    """HH_260906 - Isolate immediate versus 0.5-second normal hold braking with v6 zero steering fixed."""

    profile_id: str = "comfortable_v7_hold_brake_ramp"
    low_speed_steering_mode: str = "zero"
    hold_brake_mode: str = "immediate"
    hold_brake_ramp_seconds: float = 0.5


@dataclass(frozen=True)
class TerminalApproachDevelopmentGoalStopConfig(HoldBrakeRampDevelopmentGoalStopConfig):
    """HH_260906 - Isolate a bounded terminal throttle request; retain all measured stop and quality gates."""

    profile_id: str = "comfortable_v9_terminal_feedback"
    terminal_approach_mode: str = "coast"
    terminal_activation_speed_mps: float = 0.75
    terminal_minimum_target_speed_mps: float = 0.12
    terminal_maximum_target_speed_mps: float = 0.35
    terminal_distance_gain_per_second: float = 0.25
    terminal_throttle_bias: float = 0.08
    terminal_feedforward_taper_mps: float = 0.10
    terminal_speed_error_gain: float = 0.5
    terminal_maximum_throttle: float = 0.15
    terminal_throttle_increase_per_second: float = 0.10
    maximum_attempts_per_revision: int = 2


@dataclass(frozen=True)
class TurnLowDevelopmentGoalStopConfig(BrakeFreeDevelopmentGoalStopConfig):
    """HH_260906 - Freeze one low-speed C-track left diagnostic, not a 30 km/h qualification or admitted corpus."""

    profile_id: str = "turn_low_v1"
    nominal_cruise_speed_mps: float = 4.0
    maximum_actual_speed_mps: float = 4.3
    cruise_minimum_speed_mps: float = 3.8
    cruise_maximum_speed_mps: float = 4.2
    route_sha256: str = "2299e5bf2bc86789650da72336fdcfe7c11c585df0b9e0a0fe29a44730bae79b"
    route_length_m: float = 206.31622010469437


@dataclass(frozen=True)
class TurnLaunchDevelopmentGoalStopConfig(TurnLowDevelopmentGoalStopConfig):
    """HH_260906 - Compare fixed launch pedals while preserving a common post-handoff ramp origin and every full-route check."""

    profile_id: str = "turn_launch_015_v1"
    post_handoff_initial_throttle: float = 0.15


# HH_260906 - Four named prospective cases are not an arbitrary throttle override or an automatic retry policy.
TURN_LAUNCH_PEDALS = {"turn_launch_012_v1": 0.12, "turn_launch_013_v1": 0.13,
                      "turn_launch_014_v1": 0.14, "turn_launch_015_v1": 0.15}


def turn_launch_configuration(profile: str) -> TurnLaunchDevelopmentGoalStopConfig:
    """HH_260906 - Resolve only a frozen named launch case; unknown revisions fail before simulator access."""
    if profile not in TURN_LAUNCH_PEDALS:
        raise ValueError("unknown fixed turn launch case")
    return replace(TurnLaunchDevelopmentGoalStopConfig(), profile_id=profile,
                   launch_throttle=TURN_LAUNCH_PEDALS[profile])


def is_low_turn_config(config: Any) -> bool:
    """HH_260906 - Preserve exact-type admission for the original low-speed and separately named launch families."""
    return type(config) in (TurnLowDevelopmentGoalStopConfig, TurnLaunchDevelopmentGoalStopConfig)


def low_speed_steering_mode_from_args(args: Any) -> str | None:
    """HH_260906 - The new mode must be explicit for v6 and absent for every historical/default profile."""
    mode = getattr(args, "low_speed_steering_mode", None)
    profile = getattr(args, "goal_stop_profile", "disabled")
    if profile in ("comfortable_v7_hold_brake_ramp", "comfortable_v9_terminal_feedback"):
        if type(mode) is not str or mode != "zero":
            raise ValueError(f"{profile} requires explicit low_speed_steering_mode=zero")
    elif profile == "comfortable_v6_low_speed_steering":
        if type(mode) is not str or mode not in ("native", "zero"):
            raise ValueError("comfortable_v6_low_speed_steering requires explicit low_speed_steering_mode=native|zero")
    elif mode is not None:
        raise ValueError("low_speed_steering_mode is valid only for comfortable_v6_low_speed_steering")
    return mode


def hold_brake_mode_from_args(args: Any) -> str | None:
    """HH_260906 - Require the declared v7 arm and reject this option for every older/default profile."""
    mode = getattr(args, "hold_brake_mode", None)
    profile = getattr(args, "goal_stop_profile", "disabled")
    if profile == "comfortable_v9_terminal_feedback":
        if type(mode) is not str or mode != "immediate":
            raise ValueError("comfortable_v9_terminal_feedback requires explicit hold_brake_mode=immediate")
    elif profile == "comfortable_v7_hold_brake_ramp":
        if type(mode) is not str or mode not in ("immediate", "ramp"):
            raise ValueError("comfortable_v7_hold_brake_ramp requires explicit hold_brake_mode=immediate|ramp")
    elif mode is not None:
        raise ValueError("hold_brake_mode is valid only for comfortable_v7_hold_brake_ramp")
    return mode


def terminal_approach_mode_from_args(args: Any) -> str | None:
    """HH_260906 - Do not silently enable the new intervention or inherit an unreviewed substep profile."""
    mode = getattr(args, "terminal_approach_mode", None)
    if getattr(args, "goal_stop_profile", "disabled") == "comfortable_v9_terminal_feedback":
        if type(mode) is not str or mode not in ("coast", "feedback"):
            raise ValueError("comfortable_v9_terminal_feedback requires explicit terminal_approach_mode=coast|feedback")
        if getattr(args, "physics_substep_profile", None) != "reference_10ms":
            raise ValueError("comfortable_v9_terminal_feedback requires explicit physics_substep_profile=reference_10ms")
    elif mode is not None:
        raise ValueError("terminal_approach_mode is valid only for comfortable_v9_terminal_feedback")
    return mode


def configuration_from_args(args: Any) -> ComfortableGoalStopConfig | None:
    """HH_260906 - Require explicit compatible settings without changing legacy defaults."""
    steering_mode = low_speed_steering_mode_from_args(args)
    hold_mode = hold_brake_mode_from_args(args)
    terminal_mode = terminal_approach_mode_from_args(args)
    profile = getattr(args, "goal_stop_profile", "disabled")
    if profile == "disabled":
        return None
    if profile not in ("comfortable_v1", "comfortable_v2", "comfortable_v3", "comfortable_v4", "comfortable_v5_brake_hold", "comfortable_v6_low_speed_steering", "comfortable_v7_hold_brake_ramp", "comfortable_v9_terminal_feedback", "turn_low_v1", *TURN_LAUNCH_PEDALS):
        raise ValueError("unknown goal-stop profile")
    config = ComfortableGoalStopConfig()
    if profile == "comfortable_v2":
        # HH_260906 - Joint calibration responds to measured launch spikes and braking lag, not looser QA.
        config = replace(config, profile_id=profile, desired_deceleration_mps2=1.0,
                         normal_throttle_cap=0.20, normal_brake_cap=0.15)
    if profile in ("comfortable_v3", "comfortable_v4", "comfortable_v5_brake_hold", "comfortable_v6_low_speed_steering", "comfortable_v7_hold_brake_ramp", "comfortable_v9_terminal_feedback", "turn_low_v1", *TURN_LAUNCH_PEDALS):
        config = (turn_launch_configuration(profile) if profile in TURN_LAUNCH_PEDALS else
                  {"comfortable_v3": DevelopmentGoalStopConfig, "comfortable_v4": BrakeFreeDevelopmentGoalStopConfig,
                   "comfortable_v5_brake_hold": BrakeHoldDevelopmentGoalStopConfig,
                   "comfortable_v6_low_speed_steering": LowSpeedSteeringDevelopmentGoalStopConfig,
                   "comfortable_v7_hold_brake_ramp": HoldBrakeRampDevelopmentGoalStopConfig,
                   "comfortable_v9_terminal_feedback": TerminalApproachDevelopmentGoalStopConfig,
                   "turn_low_v1": TurnLowDevelopmentGoalStopConfig}[profile]())
        if profile == "comfortable_v6_low_speed_steering":
            config = replace(config, low_speed_steering_mode=steering_mode)
        if profile == "comfortable_v7_hold_brake_ramp":
            config = replace(config, hold_brake_mode=hold_mode)
        if profile == "comfortable_v9_terminal_feedback":
            config = replace(config, terminal_approach_mode=terminal_mode)
        if (profile in TURN_LAUNCH_PEDALS or profile in ("comfortable_v5_brake_hold", "comfortable_v6_low_speed_steering", "comfortable_v7_hold_brake_ramp", "comfortable_v9_terminal_feedback")) and getattr(args, "agent_initialization", "before_bootstrap") != "after_bootstrap":
            raise ValueError(f"{profile} requires the independently observed after_bootstrap initialization")
        # HH_260906 - The brake-only comparison must use the reviewed acknowledged transport, not the legacy async path.
        if profile in ("comfortable_v4", "comfortable_v5_brake_hold", "comfortable_v6_low_speed_steering", "comfortable_v7_hold_brake_ramp", "comfortable_v9_terminal_feedback", "turn_low_v1", *TURN_LAUNCH_PEDALS) and getattr(args, "control_transport", "legacy_async") != "acknowledged_batch":
            raise ValueError(f"{profile} requires control_transport=acknowledged_batch")
        if is_low_turn_config(config) and float(args.target_speed_kmh) != 14.4:
            raise ValueError(f"{profile} requires target_speed_kmh=14.4 for a separate low-speed turn diagnostic")
        if not is_low_turn_config(config) and float(args.target_speed_kmh) != 28.8:
            raise ValueError(f"{profile} requires target_speed_kmh=28.8 (8 m/s nominal 30 km/h class)")
        if float(getattr(args, "stationary_warmup_sec", 0.0)) != 3.5:
            raise ValueError(f"{profile} requires stationary_warmup_sec=3.5")
        if getattr(args, "vehicle_type", "vehicle.toyota.prius") != "vehicle.toyota.prius":
            raise ValueError(f"{profile} requires vehicle.toyota.prius")
        duration = float(getattr(args, "max_duration_sec", 180.0))
        if not math.isfinite(duration) or not 0.0 < duration <= 180.0:
            raise ValueError(f"{profile} requires max_duration_sec in (0, 180]")
    expected = {"physics_hz": 20.0, "capture_hz": 10.0, "goal_tolerance_m": 1.0}
    for name, value in expected.items():
        if float(getattr(args, name)) != value:
            raise ValueError(f"{profile} requires {name}={value}")
    speed = float(args.target_speed_kmh)
    tail = float(getattr(args, "stationary_tail_sec", 0.0))
    if not math.isfinite(speed) or not 0.0 < speed <= 30.0:
        raise ValueError(f"{profile} requires target speed in (0, 30] km/h")
    if not math.isfinite(tail) or tail < config.minimum_tail_seconds:
        raise ValueError(f"{profile} requires at least 6.5 seconds of tail")
    return config


def validate_development_route(args: Any, route: Mapping[str, Any], route_path: Path) -> None:
    """HH_260906 - Reject unapproved pilot maps or route bytes before creating output or accessing CARLA."""
    config = configuration_from_args(args)
    if not isinstance(config, DevelopmentGoalStopConfig):
        return
    # HH_260906 - The original unaligned CARLA route is a separate exact input; historical Town07 guards stay intact.
    low_turn = is_low_turn_config(config)
    town, scenario = ("C_track_1_0_7", "left") if low_turn else ("Town07", "straight")
    if (hashlib.sha256(route_path.read_bytes()).hexdigest() != config.route_sha256
            or route.get("town") != town or route.get("scenario") != scenario
            or float(route.get("route_length_m", -1.0)) != config.route_length_m):
        raise ValueError(f"{config.profile_id} is restricted to the exact approved {town} {scenario} route")
    if low_turn and route.get("coordinate_alignment") is not None:
        raise ValueError("turn_low_v1 native CARLA input must not contain the downstream z-minus-15 map alignment")
    if (getattr(args, "weather", None) not in (None, "ClearNoon")
            or route.get("weather") != "ClearNoon"):
        raise ValueError(f"{config.profile_id} requires ClearNoon")


def source_motion_bounds() -> dict[str, Any]:
    """HH_260906 - Import real asymmetric gate limits only for the opt-in profile."""
    root = Path(__file__).resolve().parents[2]
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    from portable_e2e.model import PHYSICAL_MAXIMUM_ACCELERATION_MPS2
    from portable_e2e.runtime_contract import RuntimeGateConfig
    # HH_260906 - Preserve logical source keys while opening the canonical model files.
    from portable_e2e.workspace_paths import workspace_path

    gate = RuntimeGateConfig()
    return {
        "physical_decoder": {
            "maximum_acceleration_mps2": PHYSICAL_MAXIMUM_ACCELERATION_MPS2,
            "maximum_deceleration_mps2": PHYSICAL_MAXIMUM_ACCELERATION_MPS2,
        },
        "runtime_speed_rate_gate": {
            "maximum_acceleration_mps2": gate.maximum_acceleration_mps2,
            "maximum_deceleration_mps2": gate.maximum_deceleration_mps2,
        },
        "source_sha256": {
            name: hashlib.sha256(workspace_path(root, name).read_bytes()).hexdigest()
            for name in ("portable_e2e/model.py", "portable_e2e/runtime_contract.py")
        },
        "notice": "Speed-rate checks alone do not establish complete decoder or runtime feasibility.",
    }


def bounded_route_projection(
    points: Sequence[Mapping[str, Any]], x: float, y: float,
    previous_progress_m: float, maximum_step_m: float,
) -> tuple[float, float, int]:
    """HH_260906 - Clip segment projections to a contiguous one-step arc window, not nearby loops."""
    if not all(math.isfinite(value) for value in (x, y, previous_progress_m, maximum_step_m)):
        raise ValueError("route projection inputs must be finite")
    if previous_progress_m < 0.0 or maximum_step_m <= 0.0:
        raise ValueError("route projection progress and step are invalid")
    start = previous_progress_m
    end = min(float(points[-1]["distance_m"]), start + maximum_step_m)
    best = None
    for index, (first, second) in enumerate(zip(points, points[1:])):
        first_arc, second_arc = float(first["distance_m"]), float(second["distance_m"])
        if second_arc < start or first_arc > end or second_arc <= first_arc:
            continue
        dx, dy = float(second["x"]) - float(first["x"]), float(second["y"]) - float(first["y"])
        squared_length = dx * dx + dy * dy
        if squared_length <= 1.0e-12:
            continue
        ratio = ((x - float(first["x"])) * dx + (y - float(first["y"])) * dy) / squared_length
        ratio = max(max(0.0, (start - first_arc) / (second_arc - first_arc)),
                    min(min(1.0, (end - first_arc) / (second_arc - first_arc)), ratio))
        progress = first_arc + ratio * (second_arc - first_arc)
        error = math.hypot(x - float(first["x"]) - ratio * dx, y - float(first["y"]) - ratio * dy)
        candidate = (error, progress, index)
        if best is None or candidate < best:
            best = candidate
    if best is None:
        raise ValueError("no contiguous route segment is available")
    return best[1], best[0], best[2]


def terminal_overshoot_m(points: Sequence[Mapping[str, Any]], x: float, y: float) -> float:
    """HH_260906 - Retain signed endpoint overshoot that clamped route progress cannot reveal."""
    final = points[-1]
    for first in reversed(points[:-1]):
        dx, dy = float(final["x"]) - float(first["x"]), float(final["y"]) - float(first["y"])
        length = math.hypot(dx, dy)
        if length > 1.0e-9:
            return ((x - float(final["x"])) * dx + (y - float(final["y"])) * dy) / length
    raise ValueError("route has no terminal tangent")


class GoalStopGovernor:
    """HH_260906 - Base completion on measured stop dwell, never BasicAgent.done alone."""

    def __init__(self, config: ComfortableGoalStopConfig, cruise_speed_mps: float, physics_hz: float):
        self.config = config
        self.cruise_speed_mps = cruise_speed_mps
        self.dt = 1.0 / physics_hz
        self.required_hold_ticks = math.ceil(config.hold_seconds * physics_hz)
        self.hold_ticks = 0
        self.was_stopped = False
        self.target_speed_mps = 0.0
        self.complete = False

    def update(self, remaining_arc_m: float, planar_distance_m: float, speed_mps: float,
               terminal_overshoot: float, *, count_hold: bool = True) -> dict[str, Any]:
        if not all(math.isfinite(v) for v in (remaining_arc_m, planar_distance_m, speed_mps, terminal_overshoot)):
            raise ValueError("goal-stop measurements must be finite")
        if min(remaining_arc_m, planar_distance_m, speed_mps) < 0.0:
            raise ValueError("goal-stop distances and speed must be nonnegative")
        cfg = self.config
        in_window = remaining_arc_m <= cfg.goal_tolerance_m and planar_distance_m <= cfg.goal_tolerance_m
        upstream = remaining_arc_m > 1.0e-6
        stopped = in_window and upstream and terminal_overshoot <= 0.0 and speed_mps <= cfg.stopped_speed_mps
        if count_hold:
            # HH_260906 - Count elapsed intervals, not the first stopped observation itself.
            self.hold_ticks = self.hold_ticks + 1 if stopped and self.was_stopped else 0
            self.was_stopped = stopped
        self.complete = self.hold_ticks >= self.required_hold_ticks
        envelope = min(self.cruise_speed_mps, math.sqrt(
            2.0 * cfg.desired_deceleration_mps2 * max(remaining_arc_m - cfg.stop_buffer_m, 0.0)))
        if stopped:
            envelope = 0.0
        self.target_speed_mps = min(envelope, self.target_speed_mps + cfg.target_acceleration_limit_mps2 * self.dt)
        phase = "complete" if self.complete else "holding" if stopped else "approach" if envelope < self.cruise_speed_mps else "cruise"
        return {
            "profile_id": cfg.profile_id, "phase": phase,
            "remaining_route_arc_m": remaining_arc_m, "planar_goal_distance_m": planar_distance_m,
            "terminal_overshoot_m": terminal_overshoot, "measured_speed_mps": speed_mps,
            "distance_speed_envelope_mps": envelope, "target_speed_mps": self.target_speed_mps,
            "goal_window": in_window, "upstream_of_endpoint": upstream,
            "hold_ticks": self.hold_ticks, "required_hold_ticks": self.required_hold_ticks,
            "hold_duration_sec": self.hold_ticks * self.dt, "complete": self.complete,
        }


class DevelopmentGoalStopGovernor(GoalStopGovernor):
    """HH_260906 - Keep the empirical coast pilot latched and its elapsed clocks independent of warmup."""

    def __init__(self, config: DevelopmentGoalStopConfig, cruise_speed_mps: float, physics_hz: float):
        # HH_260906 - Each named pilot has one frozen configuration; subclasses cannot smuggle alternative bounds.
        expected = {DevelopmentGoalStopConfig: DevelopmentGoalStopConfig(),
                    BrakeFreeDevelopmentGoalStopConfig: BrakeFreeDevelopmentGoalStopConfig(),
                    BrakeHoldDevelopmentGoalStopConfig: BrakeHoldDevelopmentGoalStopConfig(),
                    TurnLowDevelopmentGoalStopConfig: TurnLowDevelopmentGoalStopConfig()}.get(type(config))
        if type(config) is TurnLaunchDevelopmentGoalStopConfig:
            expected = turn_launch_configuration(config.profile_id)
        if type(config) is LowSpeedSteeringDevelopmentGoalStopConfig:
            if type(config.low_speed_steering_mode) is not str or config.low_speed_steering_mode not in ("native", "zero"):
                raise ValueError("v6 requires the frozen native or zero steering mode")
            expected = replace(LowSpeedSteeringDevelopmentGoalStopConfig(),
                               low_speed_steering_mode=config.low_speed_steering_mode)
        if type(config) is HoldBrakeRampDevelopmentGoalStopConfig:
            if type(config.hold_brake_mode) is not str or config.hold_brake_mode not in ("immediate", "ramp"):
                raise ValueError("v7 requires the frozen immediate or ramp hold-brake mode")
            expected = replace(HoldBrakeRampDevelopmentGoalStopConfig(), hold_brake_mode=config.hold_brake_mode)
        if type(config) is TerminalApproachDevelopmentGoalStopConfig:
            if type(config.terminal_approach_mode) is not str or config.terminal_approach_mode not in ("coast", "feedback"):
                raise ValueError("v9 requires the frozen coast or feedback terminal-approach mode")
            expected = replace(TerminalApproachDevelopmentGoalStopConfig(), terminal_approach_mode=config.terminal_approach_mode)
        frozen_nominal = 4.0 if is_low_turn_config(config) else 8.0
        if expected is None or config != expected or cruise_speed_mps != frozen_nominal or physics_hz != 20.0:
            raise ValueError("development pilot requires its exact frozen configuration and 20 Hz clock")
        if (is_low_turn_config(config) or type(config) in (BrakeHoldDevelopmentGoalStopConfig, LowSpeedSteeringDevelopmentGoalStopConfig, HoldBrakeRampDevelopmentGoalStopConfig, TerminalApproachDevelopmentGoalStopConfig)) and any(type(value) is not type(asdict(expected)[key])
                                                                  for key, value in asdict(config).items()):
            raise ValueError(f"{config.profile_id} frozen configuration field types cannot be substituted")
        super().__init__(config, cruise_speed_mps, physics_hz)
        self.driving_started_at: float | None = None
        self.last_driving_timestamp: float | None = None
        self.handoff_at: float | None = None
        self.coast_started_at: float | None = None
        self.last_timestamp: float | None = None
        self.last_record: dict[str, Any] | None = None
        self.failure_reason: str | None = None
        self.emergency_override_count = 0
        # HH_260906 - The new hold latch is unused by historical profiles and never fabricates an elapsed world tick.
        self.hold_brake_started_at: float | None = None
        if type(config) in (HoldBrakeRampDevelopmentGoalStopConfig, TerminalApproachDevelopmentGoalStopConfig):
            self.last_hold_brake_control: dict[str, Any] | None = None
        if type(config) is TerminalApproachDevelopmentGoalStopConfig:
            self.terminal_feedback_started_at: float | None = None
            self.terminal_previous_request_timestamp: float | None = None
            self.terminal_previous_requested_throttle = 0.0
            self.last_terminal_approach_control: dict[str, Any] | None = None

    def fail(self, reason: str) -> None:
        if self.failure_reason is None:
            self.failure_reason = reason
        if type(self.config) is TerminalApproachDevelopmentGoalStopConfig and self.last_record is not None:
            # HH_260906 - Invalidate the live decision, but retain any earlier normal proposal as superseded evidence.
            decision = self.last_record.get("terminal_approach")
            if decision is not None:
                decision.update(eligible=False, target_speed_mps=0.0, positive_speed_error_mps=0.0, raw_throttle=0.0,
                                mathematical_throttle=0.0, decision="governor_failure_unchanged",
                                governor_failure_reason=self.failure_reason)
            if self.last_terminal_approach_control is not None:
                self.last_terminal_approach_control["superseded_by_governor_failure"] = True

    def update(self, remaining_arc_m: float, planar_distance_m: float, speed_mps: float,
               terminal_overshoot: float, *, count_hold: bool = True,
               timestamp: float, longitudinal_speed_mps: float, driving: bool) -> dict[str, Any]:
        cfg = self.config
        if type(cfg) in (HoldBrakeRampDevelopmentGoalStopConfig, TerminalApproachDevelopmentGoalStopConfig):
            # HH_260906 - A setup/tail/early-emergency command cannot inherit the preceding observation's normal request.
            self.last_hold_brake_control = None
        if type(cfg) is TerminalApproachDevelopmentGoalStopConfig:
            self.last_terminal_approach_control = None
            # HH_260906 - Failures discovered in a new update must not rewrite the previous stored observation.
            if self.last_record is not None:
                self.last_record["terminal_approach"] = dict(self.last_record["terminal_approach"])
            values = (remaining_arc_m, planar_distance_m, speed_mps, terminal_overshoot, timestamp, longitudinal_speed_mps)
            if any(type(value) not in (int, float) or not math.isfinite(value) for value in values):
                raise ValueError("v9 measurements require finite non-boolean numbers")
            if type(driving) is not bool or type(count_hold) is not bool:
                raise ValueError("v9 driving and dwell accounting flags must be boolean")
        if not math.isfinite(timestamp) or not math.isfinite(longitudinal_speed_mps):
            raise ValueError(f"{cfg.profile_id} needs finite measured time and longitudinal speed")
        if self.last_timestamp is not None:
            elapsed = timestamp - self.last_timestamp
            # HH_260906 - Initial engagement reuses the last warmup observation without adding an elapsed interval.
            duplicate_engagement = elapsed == 0.0 and driving and not count_hold and self.driving_started_at is None
            if not duplicate_engagement and abs(elapsed - self.dt) > 1.0e-4:
                raise ValueError(f"{cfg.profile_id} measured clock must advance one 20 Hz tick")
        self.last_timestamp = timestamp
        record = super().update(remaining_arc_m, planar_distance_m, speed_mps,
                                terminal_overshoot, count_hold=count_hold)
        if speed_mps > cfg.maximum_actual_speed_mps or longitudinal_speed_mps > cfg.maximum_actual_speed_mps:
            self.fail(f"{cfg.profile_id}_actual_speed_exceeded")
        if driving and self.driving_started_at is None:
            self.driving_started_at = timestamp
        if driving:
            self.last_driving_timestamp = timestamp
        # HH_260906 - Tail ticks remain measured but do not extend driving or zero-pedal coast duration fields.
        timer_timestamp = timestamp if self.last_driving_timestamp is None else self.last_driving_timestamp
        driving_elapsed = 0.0 if self.driving_started_at is None else timer_timestamp - self.driving_started_at
        transition = None
        if driving and self.handoff_at is None:
            if speed_mps >= cfg.launch_handoff_speed_mps:
                self.handoff_at = timestamp
                transition = "launch_to_normal_pid"
            elif driving_elapsed >= cfg.maximum_launch_seconds:
                self.fail(f"{cfg.profile_id}_launch_timeout")
        release_distance = cfg.stop_buffer_m + cfg.coast_reference_seconds * speed_mps
        if driving and self.handoff_at is not None and self.coast_started_at is None and remaining_arc_m <= release_distance:
            if not cfg.minimum_coast_entry_speed_mps <= speed_mps <= cfg.maximum_coast_entry_speed_mps:
                self.fail(f"{cfg.profile_id}_coast_entry_speed_outside_band")
            elif self.failure_reason is None:
                self.coast_started_at = timestamp
                transition = "normal_pid_to_zero_pedal_coast"
        coast_elapsed = None if self.coast_started_at is None else timer_timestamp - self.coast_started_at
        if driving and coast_elapsed is not None and coast_elapsed >= cfg.maximum_coast_seconds and not record["complete"]:
            self.fail(f"{cfg.profile_id}_coast_timeout")
        # HH_260906 - Observe the actual near-goal coast state first; the collector sends normal_control only for the following tick.
        if (type(cfg) in (BrakeHoldDevelopmentGoalStopConfig, LowSpeedSteeringDevelopmentGoalStopConfig, HoldBrakeRampDevelopmentGoalStopConfig, TerminalApproachDevelopmentGoalStopConfig) and driving and self.coast_started_at is not None
                and self.hold_brake_started_at is None and self.failure_reason is None
                and record["goal_window"] and record["upstream_of_endpoint"]
                and record["terminal_overshoot_m"] <= 0.0 and speed_mps <= cfg.stopped_speed_mps):
            self.hold_brake_started_at = timestamp
            transition = "zero_pedal_coast_to_service_brake_hold"
        terminal = None
        if type(cfg) is TerminalApproachDevelopmentGoalStopConfig:
            if (cfg.terminal_approach_mode == "feedback" and driving and self.coast_started_at is not None
                    and self.hold_brake_started_at is None and self.failure_reason is None
                    and self.emergency_override_count == 0 and self.terminal_feedback_started_at is None
                    and speed_mps <= cfg.terminal_activation_speed_mps and not record["complete"]):
                self.terminal_feedback_started_at = timestamp
                transition = "zero_pedal_coast_to_terminal_feedback"
            terminal = self._terminal_decision(record, timestamp, driving)
        envelope = min(cfg.nominal_cruise_speed_mps, math.sqrt(
            cfg.approach_reference_speed_mps ** 2 + 2 * cfg.desired_deceleration_mps2
            * max(remaining_arc_m - cfg.approach_reference_distance_m, 0.0)))
        # HH_260906 - Superclass stop/dwell accounting is retained, but its constant-to-zero envelope is not used by this pilot.
        previous_target = 0.0 if self.last_record is None else self.last_record["target_speed_mps"]
        if not driving or self.coast_started_at is not None or record["complete"]:
            self.target_speed_mps = 0.0
        else:
            self.target_speed_mps = min(envelope, previous_target + cfg.target_acceleration_limit_mps2 * self.dt)
        if terminal is not None and terminal["eligible"]:
            self.target_speed_mps = terminal["target_speed_mps"]
        handoff_elapsed = None if self.handoff_at is None else timer_timestamp - self.handoff_at
        # HH_260906 - New launch cases share the original .15 post-handoff origin; changing the launch pedal cannot silently change the later ramp.
        ramp_origin = cfg.post_handoff_initial_throttle if type(cfg) is TurnLaunchDevelopmentGoalStopConfig else cfg.launch_throttle
        cap = cfg.launch_throttle if handoff_elapsed is None else min(
            float(cfg.normal_throttle_cap), ramp_origin
            + cfg.post_handoff_throttle_ramp_per_second * (handoff_elapsed + self.dt))
        state = ("complete" if record["complete"] else "coast_low" if self.coast_started_at is not None
                 else "launch_low" if self.handoff_at is None else "normal_pid") if driving else "setup_or_tail"
        if terminal is not None and terminal["eligible"]:
            state = "terminal_feedback"
        record.update({
            "phase": state, "pilot_state": state, "pilot_transition": transition,
            "measured_timestamp": timestamp, "measured_longitudinal_speed_mps": longitudinal_speed_mps,
            "driving_elapsed_sec": driving_elapsed, "handoff_elapsed_sec": handoff_elapsed,
            "coast_elapsed_sec": coast_elapsed, "coast_release_distance_m": release_distance,
            "coast_entry_latched": self.coast_started_at is not None,
            "distance_speed_envelope_mps": envelope, "target_speed_mps": self.target_speed_mps,
            "normal_throttle_cap": cap, "normal_brake_cap": cfg.normal_brake_cap,
            "pilot_failure_reason": self.failure_reason, "emergency_override_count": self.emergency_override_count,
            "training_data_approved": False,
        })
        if type(cfg) in (BrakeHoldDevelopmentGoalStopConfig, LowSpeedSteeringDevelopmentGoalStopConfig, HoldBrakeRampDevelopmentGoalStopConfig, TerminalApproachDevelopmentGoalStopConfig):
            # HH_260906 - Preserve the source observation and original dwell counters; a brake command is not stop completion.
            if driving and self.hold_brake_started_at is not None and not record["complete"]:
                record["phase"] = record["pilot_state"] = "service_brake_hold"
            record.update(hold_brake_latched=self.hold_brake_started_at is not None,
                          hold_brake_source_timestamp=self.hold_brake_started_at,
                          normal_hold_brake=cfg.normal_hold_brake,
                          hold_brake_notice="Applied only by the next normal-control command; physical substep timing is not proven.")
        if type(cfg) in (LowSpeedSteeringDevelopmentGoalStopConfig, HoldBrakeRampDevelopmentGoalStopConfig, TerminalApproachDevelopmentGoalStopConfig):
            # HH_260906 - This is the governor/command phase; initial engagement can reuse a warmup observation.
            if type(driving) is not bool:
                raise ValueError("v6 driving state must be boolean")
            record["low_speed_steering_driving_enabled"] = driving
        if terminal is not None:
            record["terminal_approach"] = terminal
        self.last_record = dict(record)
        return record

    def _terminal_decision(self, record: Mapping[str, Any], timestamp: float, driving: bool) -> dict[str, Any]:
        """HH_260906 - Compute the next normal request once from this observation; never advance a clock in a control call."""
        cfg = self.config
        eligible = (cfg.terminal_approach_mode == "feedback" and driving
                    and self.terminal_feedback_started_at is not None and self.hold_brake_started_at is None
                    and self.failure_reason is None and self.emergency_override_count == 0 and not record["complete"])
        target = raw = error = mathematical = 0.0
        interval = self.dt if self.terminal_previous_request_timestamp is None else timestamp - self.terminal_previous_request_timestamp
        if not math.isfinite(interval) or interval < 0.0:
            raise ValueError("v9 previous terminal request must not follow its source observation")
        if eligible and not record["goal_window"]:
            target = min(cfg.terminal_maximum_target_speed_mps, max(cfg.terminal_minimum_target_speed_mps,
                         cfg.terminal_distance_gain_per_second * max(record["remaining_route_arc_m"] - cfg.stop_buffer_m, 0.0)))
            error = max(target - record["measured_speed_mps"], 0.0)
            raw = min(cfg.terminal_maximum_throttle, cfg.terminal_throttle_bias
                      * min(1.0, error / cfg.terminal_feedforward_taper_mps) + cfg.terminal_speed_error_gain * error)
            mathematical = min(raw, self.terminal_previous_requested_throttle + cfg.terminal_throttle_increase_per_second * interval)
        decision = ("governor_failure_unchanged" if self.failure_reason is not None
                    else "coast_mode_unchanged" if cfg.terminal_approach_mode == "coast"
                    else "not_driving_command_unchanged" if not driving
                    else "original_hold_priority" if self.hold_brake_started_at is not None
                    else "feedback_not_latched" if not eligible
                    else "goal_window_zero_pedal" if record["goal_window"] else "bounded_terminal_feedback")
        return {
            "schema": "carla.terminal_approach_decision.v1", "mode": cfg.terminal_approach_mode,
            "source_timestamp": timestamp, "source_measured_speed_mps": record["measured_speed_mps"],
            "source_remaining_arc_m": record["remaining_route_arc_m"],
            "source_planar_goal_distance_m": record["planar_goal_distance_m"],
            "source_terminal_overshoot_m": record["terminal_overshoot_m"], "goal_window": record["goal_window"],
            "upstream_of_endpoint": record["upstream_of_endpoint"], "driving_enabled": driving,
            "coast_latched": self.coast_started_at is not None, "coast_source_timestamp": self.coast_started_at,
            "feedback_latched": self.terminal_feedback_started_at is not None,
            "feedback_source_timestamp": self.terminal_feedback_started_at,
            "hold_latched": self.hold_brake_started_at is not None, "eligible": eligible,
            "target_speed_mps": target, "positive_speed_error_mps": error, "raw_throttle": raw,
            "previous_requested_throttle": self.terminal_previous_requested_throttle,
            "previous_request_timestamp": self.terminal_previous_request_timestamp,
            "rate_interval_seconds": interval, "rate_limit_per_second": cfg.terminal_throttle_increase_per_second,
            "mathematical_throttle": mathematical, "next_native_dt_seconds": self.dt,
            "decision": decision, "governor_failure_reason": self.failure_reason,
            "physical_application_proven": False, "training_data_approved": False, "research_only": True,
        }

    def normal_control(self, control: Any) -> Any:
        """HH_260906 - Change normal pedals only; retain the planner's steering and later emergency authority."""
        if self.last_record is None:
            raise ValueError(f"{self.config.profile_id} normal control requires a measured governor state")
        v7 = type(self.config) in (HoldBrakeRampDevelopmentGoalStopConfig, TerminalApproachDevelopmentGoalStopConfig)
        v9 = type(self.config) is TerminalApproachDevelopmentGoalStopConfig
        if v7:
            before = low_speed_steering_control_projection(control)
            elapsed, mathematical_brake = None, None
        if type(self.config) in (BrakeHoldDevelopmentGoalStopConfig, LowSpeedSteeringDevelopmentGoalStopConfig, HoldBrakeRampDevelopmentGoalStopConfig, TerminalApproachDevelopmentGoalStopConfig) and self.hold_brake_started_at is not None:
            # HH_260906 - Keep service braking latched through measured dwell without changing steering or emergency authority.
            brake = self.config.normal_hold_brake
            if v7:
                elapsed = self.last_record["measured_timestamp"] - self.hold_brake_started_at
                if not math.isfinite(elapsed) or elapsed < 0.0:
                    raise ValueError("v7 hold source must not follow its current measured timestamp")
                # HH_260906 - The first following-interval request is .1; retain actual timestamps, not rounded tick counts.
                brake = (min(1.0, (elapsed + self.dt) / self.config.hold_brake_ramp_seconds)
                         if self.config.hold_brake_mode == "ramp" else 1.0)
                mathematical_brake = brake
            control.throttle, control.brake = 0.0, brake
        elif v9 and self.last_record["terminal_approach"]["eligible"] and self.failure_reason is None:
            control.throttle, control.brake = self.last_record["terminal_approach"]["mathematical_throttle"], 0.0
            self.terminal_previous_requested_throttle = self.last_record["terminal_approach"]["mathematical_throttle"]
            self.terminal_previous_request_timestamp = self.last_record["measured_timestamp"]
        elif self.coast_started_at is not None:
            control.throttle, control.brake = 0.0, 0.0
        elif self.handoff_at is None:
            control.throttle, control.brake = self.config.launch_throttle, 0.0
        else:
            control.throttle = min(float(control.throttle), self.last_record["normal_throttle_cap"])
            control.brake = min(float(control.brake), self.config.normal_brake_cap)
        if v7:
            self.last_hold_brake_control = {
                "schema": "carla.hold_brake_control_request.v1", "mode": self.config.hold_brake_mode,
                "source_timestamp": self.last_record["measured_timestamp"],
                "hold_source_timestamp": self.hold_brake_started_at,
                "hold_latched": self.hold_brake_started_at is not None,
                "hold_elapsed_seconds": elapsed,
                "next_interval_end_elapsed_seconds": None if elapsed is None else elapsed + self.dt,
                "ramp_seconds": self.config.hold_brake_ramp_seconds, "native_dt_seconds": self.dt,
                "normal_control_executed": True, "mathematical_hold_brake": mathematical_brake,
                "input_control": before, "returned_control": low_speed_steering_control_projection(control),
                "control_stage": "LocalPlanner_normal_control_before_BasicAgent_emergency_and_output_steering",
                "physical_application_proven": False, "training_data_approved": False, "research_only": True,
            }
        if v9:
            self.last_terminal_approach_control = {
                "schema": "carla.terminal_approach_control_request.v1", "mode": self.config.terminal_approach_mode,
                "source_timestamp": self.last_record["measured_timestamp"], "normal_control_executed": True,
                "decision_at_normal_control": dict(self.last_record["terminal_approach"]),
                "input_control": before, "returned_control": low_speed_steering_control_projection(control),
                "control_stage": "LocalPlanner_normal_control_before_BasicAgent_emergency_and_output_steering",
                "superseded_by_governor_failure": self.failure_reason is not None,
                "physical_application_proven": False, "training_data_approved": False, "research_only": True,
            }
        return control


def install_development_control(agent: Any, governor: DevelopmentGoalStopGovernor) -> dict[str, Any]:
    """HH_260906 - Observe the real emergency method without delaying or changing its returned control."""
    planner = agent.get_local_planner()
    metadata = install_normal_brake_cap(planner, governor.config.normal_brake_cap,
                                        governor.config.normal_throttle_cap)
    original_step = planner.run_step
    original_emergency = agent.add_emergency_stop

    def normal_step(*args: Any, **kwargs: Any) -> Any:
        return governor.normal_control(original_step(*args, **kwargs))

    def emergency_step(*args: Any, **kwargs: Any) -> Any:
        control = original_emergency(*args, **kwargs)
        governor.emergency_override_count += 1
        governor.fail(f"{governor.config.profile_id}_emergency_override")
        return control

    planner.run_step = normal_step
    agent.add_emergency_stop = emergency_step
    metadata.update({"normal_control_insertion": "LocalPlanner.run_step before BasicAgent.add_emergency_stop",
                     "lateral_control_modified": False, "emergency_return_control_modified": False,
                     "hazard_abort_policy": "Apply original emergency control, observe the following native tick, then retain a failed capture.",
                     "training_data_approved": False})
    if type(governor.config) in (LowSpeedSteeringDevelopmentGoalStopConfig, HoldBrakeRampDevelopmentGoalStopConfig, TerminalApproachDevelopmentGoalStopConfig):
        # HH_260906 - No extra agent/PID hook: the collector applies this only after its original proposal checks.
        metadata.update({
            "lateral_control_modified": governor.config.low_speed_steering_mode == "zero",
            "low_speed_steering_mode": governor.config.low_speed_steering_mode,
            "low_speed_steering_threshold_mps": governor.config.low_speed_steering_threshold_mps,
            "low_speed_steering_insertion": "collector_after_original_agent_proposal_checks_before_existing_stopped_brake_guard",
            "low_speed_steering_scope": "Driving commands only, measured speed <= threshold, no governor failure or emergency; native arm unchanged.",
            "basic_agent_and_pid_internal_updates_modified": False,
            "research_only": True,
        })
    if type(governor.config) in (HoldBrakeRampDevelopmentGoalStopConfig, TerminalApproachDevelopmentGoalStopConfig):
        metadata.update(hold_brake_mode=governor.config.hold_brake_mode,
                        hold_brake_ramp_seconds=governor.config.hold_brake_ramp_seconds,
                        hold_brake_formula="immediate:1; ramp:min(1,(source_timestamp-hold_source_timestamp+native_dt)/0.5)",
                        hold_brake_scope="Normal latched hold only; existing emergency return and measured stop/dwell remain unchanged.")
    if type(governor.config) is TerminalApproachDevelopmentGoalStopConfig:
        metadata.update(terminal_approach_mode=governor.config.terminal_approach_mode,
                        terminal_approach_insertion="LocalPlanner normal pedals before unchanged BasicAgent emergency and output steering",
                        terminal_approach_formula="Outside goal: target=min(.35,max(.12,.25*max(remaining-.75,0))); error=max(target-speed,0); desired=min(.15,.08*min(1,error/.1)+.5*error); upward slew .10/s, immediate reductions. Inside goal: target/throttle zero; original hold has priority.",
                        terminal_approach_target_is_actual_speed_cap=False,
                        terminal_approach_scope="Feedback arm only, latched after coast at measured speed <= .75; no failure/emergency, driving, before original hold.")
    return metadata


def low_speed_steering_control_projection(control: Any) -> dict[str, Any]:
    """HH_260906 - Match collector control_dict's seven fields/defaults without importing the collector."""
    values = {name: getattr(control, name, default) for name, default in (
        ("throttle", 0.0), ("steer", 0.0), ("brake", 0.0), ("hand_brake", False),
        ("reverse", False), ("gear", 0), ("manual_gear_shift", False))}
    for name in ("throttle", "steer", "brake"):
        value = values[name]
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
            raise ValueError("v6 control projection requires finite numeric controls")
        if not (-1.0 <= value <= 1.0 if name == "steer" else 0.0 <= value <= 1.0):
            raise ValueError("v6 control projection is outside normalized control limits")
        values[name] = float(value)
    if any(type(values[name]) is not bool for name in ("hand_brake", "reverse", "manual_gear_shift")):
        raise ValueError("v6 control projection requires strict boolean flags")
    if type(values["gear"]) is not int:
        raise ValueError("v6 control projection requires an integer gear")
    return values


def apply_low_speed_steering(control: Any, governor: Any, record: dict[str, Any] | None,
                             *, driving_command: bool) -> Any:
    """HH_260906 - Preserve original proposal checks/PID memory; optionally change only the next requested steer."""
    if not isinstance(governor, DevelopmentGoalStopGovernor) or type(governor.config) not in (LowSpeedSteeringDevelopmentGoalStopConfig, HoldBrakeRampDevelopmentGoalStopConfig, TerminalApproachDevelopmentGoalStopConfig):
        return control
    state = governor.last_record
    if state is None or record is None or type(driving_command) is not bool:
        raise ValueError("v6 steering requires a measured state, its record and a boolean command phase")
    # HH_260906 - Refuse stale evidence rather than associating a command with another observation.
    for key in ("profile_id", "measured_timestamp", "measured_speed_mps", "low_speed_steering_driving_enabled"):
        if key not in record or record[key] != state[key] or type(record[key]) is not type(state[key]):
            raise ValueError("v6 steering record must match the current measured governor state")
    speed, stamp = float(state["measured_speed_mps"]), float(state["measured_timestamp"])
    input_control = low_speed_steering_control_projection(control)
    original_steer = input_control["steer"]
    if not all(math.isfinite(value) for value in (speed, stamp, original_steer)) or speed < 0.0:
        raise ValueError("v6 steering needs finite measured speed/time and original steering")
    emergency = (governor.emergency_override_count > 0
                 or governor.failure_reason == f"{governor.config.profile_id}_emergency_override")
    failure = governor.failure_reason is not None
    driving = driving_command and state["low_speed_steering_driving_enabled"]
    eligible = driving and not emergency and not failure and speed <= governor.config.low_speed_steering_threshold_mps
    overridden = eligible and governor.config.low_speed_steering_mode == "zero"
    if overridden:
        control.steer = 0.0
    returned_control = low_speed_steering_control_projection(control)
    reason = ("emergency_unchanged" if emergency else "governor_failure_unchanged" if failure
              else "not_driving_command_unchanged" if not driving
              else "native_mode_unchanged" if governor.config.low_speed_steering_mode == "native"
              else "above_threshold_unchanged" if not eligible else "zero_steering_requested")
    record["low_speed_steering"] = {
        "schema": "carla.low_speed_steering_request.v1", "mode": governor.config.low_speed_steering_mode,
        "source_timestamp": stamp, "source_measured_speed_mps": speed,
        "speed_definition": "hypot(measured base_link vx, measured base_link vy), unchanged governor input",
        "governor_driving_enabled": state["low_speed_steering_driving_enabled"],
        "driving_command": driving_command, "threshold_mps": governor.config.low_speed_steering_threshold_mps,
        "emergency_override_present": emergency, "governor_failure_reason": governor.failure_reason,
        "eligible": eligible, "override_requested": overridden,
        "input_steer": original_steer, "returned_steer": returned_control["steer"],
        "input_control": input_control, "returned_control": returned_control,
        "steering_value_changed": original_steer != returned_control["steer"], "decision": reason,
        "control_stage": "after_original_agent_proposal_checks_before_existing_stopped_brake_guard",
        "physical_application_proven": False, "training_data_approved": False, "research_only": True,
        "notice": "Intermediate requested control only; final next_control and ACK/readback remain authoritative. No PID state, label or gate modification.",
    }
    if type(governor.config) in (HoldBrakeRampDevelopmentGoalStopConfig, TerminalApproachDevelopmentGoalStopConfig):
        cached = governor.last_hold_brake_control
        if cached is not None and cached["source_timestamp"] != stamp:
            raise ValueError("v7 normal hold request belongs to another measured observation")
        # HH_260906 - Preserve upstream normal output even when BasicAgent later supersedes it with its original emergency.
        hold = dict(cached) if cached is not None else {
            "schema": "carla.hold_brake_control_request.v1", "mode": governor.config.hold_brake_mode,
            "source_timestamp": stamp, "hold_source_timestamp": governor.hold_brake_started_at,
            "hold_latched": governor.hold_brake_started_at is not None,
            "hold_elapsed_seconds": None, "next_interval_end_elapsed_seconds": None,
            "ramp_seconds": governor.config.hold_brake_ramp_seconds, "native_dt_seconds": governor.dt,
            "normal_control_executed": False, "mathematical_hold_brake": None,
            "input_control": None, "returned_control": None,
            "control_stage": "LocalPlanner_normal_control_before_BasicAgent_emergency_and_output_steering",
            "physical_application_proven": False, "training_data_approved": False, "research_only": True,
        }
        hold.update(driving_command=driving_command, superseded_by_emergency=emergency,
                    notice="Upstream normal request only, not physical actuation. No normal call means no ramp command; collector setup/tail/stop controls are separate.")
        record["hold_brake_control"] = hold
    if type(governor.config) is TerminalApproachDevelopmentGoalStopConfig:
        cached = governor.last_terminal_approach_control
        if cached is not None and cached["source_timestamp"] != stamp:
            raise ValueError("v9 terminal normal request belongs to another measured observation")
        terminal = dict(cached) if cached is not None else {
            "schema": "carla.terminal_approach_control_request.v1", "mode": governor.config.terminal_approach_mode,
            "source_timestamp": stamp, "normal_control_executed": False,
            "decision_at_normal_control": None, "input_control": None, "returned_control": None,
            "control_stage": "LocalPlanner_normal_control_before_BasicAgent_emergency_and_output_steering",
            "superseded_by_governor_failure": failure,
            "physical_application_proven": False, "training_data_approved": False, "research_only": True,
        }
        terminal.update(driving_command=driving_command, superseded_by_emergency=emergency,
                        output_steering_stage_input_control=input_control,
                        output_steering_stage_returned_control=returned_control,
                        notice="Normal proposal and later output stage only; final next_control and ACK/readback remain authoritative. No current actuation or label timing is inferred.")
        record["terminal_approach_control"] = terminal
    return control


def annotate_development_control(record: dict[str, Any], governor: DevelopmentGoalStopGovernor,
                                 stop_reason: str | None) -> None:
    """HH_260906 - Record a hook-triggered emergency after agent.run_step without claiming the next command is applied yet."""
    pending = stop_reason is None and governor.failure_reason == f"{governor.config.profile_id}_emergency_override"
    record.update({"pilot_failure_reason": governor.failure_reason,
                   "emergency_override_count": governor.emergency_override_count,
                   "emergency_failure_pending_next_tick": pending,
                   "next_control_is_unchanged_emergency_override": pending})
    if pending:
        record["control_source"] = "BasicAgent_unchanged_emergency_override_requested_observe_next_tick_then_abort"


def goal_stop_termination_reason(record: Mapping[str, Any], basic_agent_done: bool,
                                 cross_track_error_m: float,
                                 config: ComfortableGoalStopConfig) -> str | None:
    """HH_260906 - Fail early on incomplete planner endpoints; permit only measured goal dwell."""
    if record.get("pilot_failure_reason") is not None:
        return str(record["pilot_failure_reason"])
    if cross_track_error_m > config.maximum_projection_error_m:
        return "comfortable_goal_route_projection_failure"
    if not record["upstream_of_endpoint"] or (record["goal_window"] and record["terminal_overshoot_m"] > 0.0):
        return "comfortable_goal_endpoint_overshoot"
    if record["complete"]:
        return "comfortable_goal_measured_stop_and_dwell"
    if basic_agent_done and not record["goal_window"]:
        return "basic_agent_done_before_comfortable_goal_window"
    return None


def measured_goal_completion(record: Mapping[str, Any], cross_track_error_m: float,
                             config: ComfortableGoalStopConfig) -> bool:
    """HH_260906 - Keep verified dwell meaningful in tail ticks only while the vehicle remains stopped."""
    return bool(record["complete"] and record["goal_window"] and record["upstream_of_endpoint"]
                and record["measured_speed_mps"] <= config.stopped_speed_mps
                and record["terminal_overshoot_m"] <= 0.0
                and cross_track_error_m <= config.maximum_projection_error_m)


def complete_terminal_plan(plan: Sequence[Any], goal_waypoint: Any,
                           requested_goal_xyz: Mapping[str, float],
                           sampling_resolution_m: float) -> tuple[list[Any], dict[str, Any]]:
    """HH_260906 - Append the real requested endpoint omitted by the planner's two-sample early exit."""
    if len(plan) < 2 or goal_waypoint is None or not math.isfinite(sampling_resolution_m) or sampling_resolution_m <= 0:
        raise ValueError("comfortable terminal plan requires a real goal waypoint and a nonempty sampled route")
    previous, road_option = plan[-1]
    identity = lambda wp: (int(wp.road_id), int(wp.section_id), int(wp.lane_id))
    if identity(previous) != identity(goal_waypoint):
        raise ValueError("comfortable terminal goal must stay on the same terminal road/section/lane")
    def position(wp: Any) -> dict[str, float]:
        loc = wp.transform.location
        return {name: float(getattr(loc, name)) for name in ("x", "y", "z")}
    old, goal = position(previous), position(goal_waypoint)
    values = [*old.values(), *goal.values(), *requested_goal_xyz.values()]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("comfortable terminal positions must be finite")
    projection_error = math.hypot(goal["x"] - requested_goal_xyz["x"], goal["y"] - requested_goal_xyz["y"])
    if projection_error > 0.25:
        raise ValueError("comfortable terminal map projection differs from the requested goal by more than 0.25 m")
    gap = math.sqrt(sum((goal[name] - old[name]) ** 2 for name in ("x", "y", "z")))
    if gap > 2.0 * sampling_resolution_m + 0.25:
        raise ValueError("comfortable terminal gap exceeds the documented two-sample planner endpoint window")
    yaw = math.radians(float(previous.transform.rotation.yaw))
    longitudinal_gap = (goal["x"] - old["x"]) * math.cos(yaw) + (goal["y"] - old["y"]) * math.sin(yaw)
    if longitudinal_gap < -1.0e-3:
        raise ValueError("comfortable terminal goal lies behind the existing final waypoint")
    updated = list(plan)
    appended = gap > 1.0e-6
    if appended:
        updated.append((goal_waypoint, road_option))
    return updated, {
        "policy": "Append actual map-projected requested goal-center waypoint; never extend the catalog route or invent a waypoint.",
        "appended_goal_waypoint": appended, "original_plan_points": len(plan), "effective_plan_points": len(updated),
        "original_terminal_carla_xyz": old, "requested_goal_center_carla_xyz": dict(requested_goal_xyz),
        "effective_terminal_carla_xyz": goal, "terminal_road_section_lane": list(identity(goal_waypoint)),
        "original_to_goal_gap_m": gap, "map_projection_planar_error_m": projection_error,
        "maximum_map_projection_error_m": 0.25, "maximum_terminal_gap_m": 2.0 * sampling_resolution_m + 0.25,
        "catalog_route_changed": False, "goal_tolerance_changed": False,
    }


def install_normal_brake_cap(local_planner: Any, cap: float,
                             throttle_cap: float | None = None) -> dict[str, Any]:
    """HH_260906 - Cap planner braking BEFORE BasicAgent adds its untouched emergency override."""
    controller = local_planner._vehicle_controller
    original_cap = float(controller.max_brake)
    original_throttle = float(controller.max_throt) if throttle_cap is not None else None
    original_run_step = local_planner.run_step
    controller.max_brake = cap
    if throttle_cap is not None:
        controller.max_throt = throttle_cap

    def normal_step(*args: Any, **kwargs: Any) -> Any:
        control = original_run_step(*args, **kwargs)
        # HH_260906 - Empty-queue braking bypasses PID; cap it here, not at the agent output.
        control.brake = min(float(control.brake), cap)
        if throttle_cap is not None:
            control.throttle = min(float(control.throttle), throttle_cap)
        return control

    local_planner.run_step = normal_step
    return {"original_pid_max_brake": original_cap, "normal_brake_cap": cap,
            "original_pid_max_throttle": original_throttle, "normal_throttle_cap": throttle_cap,
            "empty_queue_brake_capped_before_hazard_override": True,
            "basic_agent_emergency_brake_modified": False}


def measured_stop_quality(records: Sequence[Mapping[str, Any]], camera_frames: Sequence[int],
                          config: ComfortableGoalStopConfig, bounds: Mapping[str, Any],
                          completed: bool) -> dict[str, Any]:
    """HH_260906 - Include moving-tail boundary intervals at native and actual camera cadences."""
    frame_set = set(camera_frames)
    # HH_260906 - A short suffix can have perfect cadence while silently omitting almost all images.
    expected_frames = {r["frame"] for r in records[::2]}
    coverage_ok = frame_set == expected_frames and len(camera_frames) == len(frame_set)
    measurements = {}
    for cadence, selected in (("native_20hz", list(records)),
                              ("camera_10hz", [r for r in records if r["frame"] in frame_set])):
        intervals = []
        for previous, current in zip(selected, selected[1:]):
            dt = float(current["timestamp"]) - float(previous["timestamp"])
            if not math.isfinite(dt) or dt <= 0.0:
                raise ValueError("measured stop quality requires increasing finite timestamps")
            first_speed = math.hypot(float(previous["vx"]), float(previous["vy"]))
            speed = math.hypot(float(current["vx"]), float(current["vy"]))
            rate = (speed - first_speed) / dt
            if not math.isfinite(rate):
                raise ValueError("measured stop quality requires finite speed rates")
            intervals.append({"from_frame": previous["frame"], "to_frame": current["frame"],
                              "from_phase": previous["capture_phase"], "to_phase": current["capture_phase"],
                              "from_timestamp": previous["timestamp"], "to_timestamp": current["timestamp"],
                              "from_speed_mps": first_speed, "to_speed_mps": speed,
                              "dt_sec": dt, "speed_rate_mps2": rate})
        cadence_report = {}
        expected_dt = 0.05 if cadence == "native_20hz" else 0.1
        cadence_report["cadence_violation_count"] = sum(abs(r["dt_sec"] - expected_dt) > 1.0e-4 for r in intervals)
        for phase in ("all", "stationary_warmup", "driving", "stationary_tail"):
            subset = intervals if phase == "all" else [r for r in intervals if r["to_phase"] == phase]
            rates = [r["speed_rate_mps2"] for r in subset]
            checks = {}
            for name in ("physical_decoder", "runtime_speed_rate_gate"):
                limits = bounds[name]
                violations = [r for r in subset if r["speed_rate_mps2"] > limits["maximum_acceleration_mps2"] + 1.0e-9
                              or r["speed_rate_mps2"] < -limits["maximum_deceleration_mps2"] - 1.0e-9]
                checks[name] = {"violation_count": len(violations), "violation_intervals": violations,
                                "speed_rate_check_pass": bool(subset) and not violations}
            cadence_report[phase] = {"interval_count": len(subset),
                                     "first_interval": dict(subset[0]) if subset else None,
                                     "phase_boundary_intervals": [r for r in subset if r["from_phase"] != r["to_phase"]],
                                     "minimum_speed_rate_mps2": min(rates) if rates else None,
                                     "maximum_speed_rate_mps2": max(rates) if rates else None,
                                     **checks}
        measurements[cadence] = cadence_report
    tail = [r for r in records if r["capture_phase"] == "stationary_tail"]
    tail_speeds = [math.hypot(float(r["vx"]), float(r["vy"])) for r in tail]
    driving = [r for r in records if r["capture_phase"] == "driving"]
    stop = driving[-1].get("goal_stop", {}) if driving else {}
    camera_tail = [r for r in tail if r["frame"] in frame_set]
    tail_ready = len(tail) >= math.ceil(config.minimum_tail_seconds * 20.0)
    tail_still = bool(tail_speeds) and max(tail_speeds) <= config.stopped_speed_mps
    tail_goal_held = bool(tail) and all(measured_goal_completion(
        r.get("goal_stop", {}), float(r.get("route_cte_m", math.inf)), config) for r in tail)
    geometry_ok = bool(driving) and all(
        float(r.get("route_cte_m", math.inf)) <= config.maximum_projection_error_m
        and (float(r.get("goal_stop", {}).get("remaining_route_arc_m", 0.0)) > config.goal_tolerance_m
             or float(r.get("goal_stop", {}).get("terminal_overshoot_m", math.inf)) <= 0.0)
        and bool(r.get("goal_stop", {}).get("upstream_of_endpoint")) for r in driving + tail)
    rates_ok = all(measurements[cadence]["all"]["physical_decoder"]["speed_rate_check_pass"]
                   and measurements[cadence]["cadence_violation_count"] == 0
                   for cadence in measurements)
    passed = bool(completed and stop.get("complete") and tail_ready and tail_still
                  and geometry_ok and rates_ok and camera_tail and coverage_ok and tail_goal_held)
    report = {
        "schema": "carla_expert.comfortable_goal_stop_quality.v1", "status": "PASS" if passed else "FAIL",
        "config": asdict(config), "bounds": dict(bounds), "completed_stop_before_tail": bool(completed and stop.get("complete")),
        "driving_final_stop": dict(stop), "tail_state_count": len(tail), "tail_camera_count": len(camera_tail),
        "tail_minimum_duration_met": tail_ready, "tail_all_speeds_stopped": tail_still,
        "tail_measured_goal_held": tail_goal_held,
        "tail_maximum_speed_mps": max(tail_speeds) if tail_speeds else None,
        "moving_tail_frame_count": sum(speed > config.stopped_speed_mps for speed in tail_speeds),
        "route_geometry_check_pass": geometry_ok, "measurements": measurements,
        "camera_frame_coverage": {
            "complete": coverage_ok, "expected_count": len(expected_frames), "actual_count": len(camera_frames),
            "missing_frames": sorted(expected_frames - frame_set), "unexpected_frames": sorted(frame_set - expected_frames),
            "duplicate_count": len(camera_frames) - len(frame_set),
            "policy": "Every second recorded native state starting with the first state, exactly as the collector schedules cameras.",
        },
        "speed_definition": "hypot(measured base_link vx, measured base_link vy); finite difference over actual timestamps",
        "interval_phase_policy": "Assign each interval to its destination phase, retaining the previous-phase prefix state.",
        "notice": "Measured scalar speed-rate QA only; no learned actuation, target clipping, full runtime validation, or decoder feasibility guarantee.",
    }
    if isinstance(config, DevelopmentGoalStopConfig):
        # HH_260906 - Recompute actual cruise and speed admission independently of governor-reported timer fields.
        speed_violations = []
        cruise_started_at = None
        longest_cruise = 0.0
        cruise_intervals = []
        previous_timestamp = None
        for record in records:
            speed = math.hypot(float(record["vx"]), float(record["vy"]))
            vx, timestamp = float(record["vx"]), float(record["timestamp"])
            if not all(math.isfinite(v) for v in (speed, vx, timestamp)):
                raise ValueError(f"{config.profile_id} quality requires finite actual speed and timestamp")
            if speed > config.maximum_actual_speed_mps or vx > config.maximum_actual_speed_mps:
                speed_violations.append({"frame": record["frame"], "timestamp": timestamp,
                                         "planar_speed_mps": speed, "longitudinal_speed_mps": vx})
            in_cruise = (record["capture_phase"] == "driving"
                         and config.cruise_minimum_speed_mps <= speed <= config.cruise_maximum_speed_mps)
            if previous_timestamp is not None and abs(timestamp - previous_timestamp - 0.05) > 1.0e-4:
                cruise_started_at = None
            previous_timestamp = timestamp
            if in_cruise:
                if cruise_started_at is None:
                    cruise_started_at = timestamp
                duration = timestamp - cruise_started_at
                if duration > longest_cruise:
                    longest_cruise = duration
                    cruise_intervals = [{"start_timestamp": cruise_started_at, "end_timestamp": timestamp}]
            else:
                cruise_started_at = None
        pilot_failures = sorted({str(r["goal_stop"]["pilot_failure_reason"]) for r in records
                                 if r.get("goal_stop", {}).get("pilot_failure_reason") is not None})
        cruise_pass = longest_cruise + 1.0e-9 >= config.minimum_cruise_seconds
        report["development_pilot"] = {
            "actual_speed_limit_mps": config.maximum_actual_speed_mps,
            "actual_speed_violation_count": len(speed_violations), "actual_speed_violations": speed_violations,
            "cruise_band_mps": [config.cruise_minimum_speed_mps, config.cruise_maximum_speed_mps],
            "cruise_speed_definition": "Measured planar hypot(vx,vy) in driving phase; all native states remain in speed admission.",
            "required_continuous_cruise_seconds": config.minimum_cruise_seconds,
            "longest_continuous_cruise_seconds": longest_cruise, "longest_cruise_interval": cruise_intervals,
            "cruise_requirement_met": cruise_pass, "pilot_failure_reasons": pilot_failures,
            "training_data_approved": False,
            "notice": "Development pilot only; full future XY feasibility and independent dataset admission remain separate.",
        }
        if is_low_turn_config(config):
            # HH_260906 - Low-speed stability cannot be reinterpreted as satisfying the old 30 km/h cruise criterion.
            report["development_pilot"].update({
                "cruise_interpretation": "Low-speed turn stability: measured 3.8..4.2 m/s continuously for 5 seconds.",
                "qualification_30_kph": "NOT_CLAIMED",
                "old_30_kph_cruise_criterion": {"band_mps": [7.8, 8.2], "seconds": 5.0, "claimed_met": False},
                "future_dataset_split_if_separately_admitted": "train",
                "notice": "Separate low-speed C-track turn diagnostic only. No 30 km/h qualification, full future XY approval or training admission.",
            })
        if speed_violations or not cruise_pass or pilot_failures:
            report["status"] = "FAIL"
    return report
