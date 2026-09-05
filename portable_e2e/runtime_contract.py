# HH_260906 - Define the fail-closed 10 Hz runtime contract before enabling learned control.
"""ROS-independent validation primitives for live portable E2E inference."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Any, Iterable, Sequence

from .contract import ContractError
from .dataset import FEATURE_NAMES


RUNTIME_CONTRACT_ID = "portable_e2e.runtime_contract.v1"
RUNTIME_GATE_ID = "portable_e2e.runtime_geometry_gate.v6"
CAMERA_ORDER = (
    "CAM_FRONT",
    "CAM_BACK",
    "CAM_FRONT_LEFT",
    "CAM_BACK_LEFT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK_RIGHT",
)
COMMAND_NAMES = (
    "LEFT",
    "RIGHT",
    "STRAIGHT",
    "LANE_FOLLOW",
    "CHANGE_LANE_LEFT",
    "CHANGE_LANE_RIGHT",
)


def _finite_number(value: Any, context: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ContractError(f"{context} must be a number")
    # HH_260906 - Convert unrepresentable Python numerics into a fail-closed contract error.
    try:
        parsed = float(value)
    except (OverflowError, TypeError, ValueError) as error:
        raise ContractError(f"{context} must be finite") from error
    if not math.isfinite(parsed):
        raise ContractError(f"{context} must be finite")
    return parsed


def _positive_integer(value: Any, context: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ContractError(f"{context} must be a positive integer")
    return value


@dataclass(frozen=True)
class RuntimeGateConfig:
    """Physical and tensor limits applied to every model output."""

    candidate_count: int = 6
    future_points: int = 64
    timestep_s: float = 0.1
    maximum_speed_mps: float = 8.333333333333334
    maximum_step_m: float = 1.0
    maximum_abs_x_m: float = 60.0
    maximum_abs_y_m: float = 60.0
    maximum_heading_step_rad: float = 0.75
    maximum_curvature_rad_per_m: float = 0.5
    maximum_lateral_acceleration_mps2: float = 3.0
    heading_minimum_step_m: float = 0.01
    maximum_backward_step_m: float = 0.002
    maximum_speed_disagreement_mps: float = 0.25
    maximum_integrated_distance_disagreement_m: float = 0.02
    maximum_integrated_distance_disagreement_ratio: float = 0.05
    maximum_acceleration_mps2: float = 3.0
    maximum_deceleration_mps2: float = 6.0
    minimum_planar_extent_m: float = 0.05
    stationary_speed_tolerance_mps: float = 0.1
    stationary_claim_speed_epsilon_mps: float = 1.0e-4
    maximum_stationary_radius_m: float = 0.05
    maximum_stationary_extent_m: float = 0.15
    minimum_low_speed_progress_ratio: float = 0.5
    maximum_first_point_distance_m: float = 1.0
    minimum_first_point_x_m: float = -0.25

    def validate(self) -> None:
        _positive_integer(self.candidate_count, "runtime.candidate_count")
        _positive_integer(self.future_points, "runtime.future_points")
        positive = {
            "timestep_s": self.timestep_s,
            "maximum_speed_mps": self.maximum_speed_mps,
            "maximum_step_m": self.maximum_step_m,
            "maximum_abs_x_m": self.maximum_abs_x_m,
            "maximum_abs_y_m": self.maximum_abs_y_m,
            "maximum_heading_step_rad": self.maximum_heading_step_rad,
            "maximum_curvature_rad_per_m": self.maximum_curvature_rad_per_m,
            "maximum_lateral_acceleration_mps2": (
                self.maximum_lateral_acceleration_mps2
            ),
            "heading_minimum_step_m": self.heading_minimum_step_m,
            "maximum_backward_step_m": self.maximum_backward_step_m,
            "maximum_speed_disagreement_mps": self.maximum_speed_disagreement_mps,
            "maximum_integrated_distance_disagreement_m": (
                self.maximum_integrated_distance_disagreement_m
            ),
            "maximum_integrated_distance_disagreement_ratio": (
                self.maximum_integrated_distance_disagreement_ratio
            ),
            "maximum_acceleration_mps2": self.maximum_acceleration_mps2,
            "maximum_deceleration_mps2": self.maximum_deceleration_mps2,
            "minimum_planar_extent_m": self.minimum_planar_extent_m,
            "stationary_speed_tolerance_mps": self.stationary_speed_tolerance_mps,
            "stationary_claim_speed_epsilon_mps": self.stationary_claim_speed_epsilon_mps,
            "maximum_stationary_radius_m": self.maximum_stationary_radius_m,
            "maximum_stationary_extent_m": self.maximum_stationary_extent_m,
            "minimum_low_speed_progress_ratio": (
                self.minimum_low_speed_progress_ratio
            ),
            "maximum_first_point_distance_m": self.maximum_first_point_distance_m,
        }
        for name, raw_value in positive.items():
            value = _finite_number(raw_value, f"runtime.{name}")
            if value <= 0.0:
                raise ContractError(f"runtime.{name} must be positive")
        _finite_number(self.minimum_first_point_x_m, "runtime.minimum_first_point_x_m")
        if self.maximum_heading_step_rad > math.pi:
            raise ContractError("runtime.maximum_heading_step_rad must not exceed pi")
        if self.maximum_first_point_distance_m > self.maximum_step_m:
            raise ContractError(
                "runtime.maximum_first_point_distance_m must not exceed maximum_step_m"
            )
        if self.heading_minimum_step_m > self.maximum_step_m:
            raise ContractError(
                "runtime.heading_minimum_step_m must not exceed maximum_step_m"
            )
        if self.maximum_backward_step_m > self.heading_minimum_step_m:
            raise ContractError(
                "runtime.maximum_backward_step_m must not exceed "
                "heading_minimum_step_m"
            )
        if self.stationary_claim_speed_epsilon_mps > self.stationary_speed_tolerance_mps:
            raise ContractError(
                "runtime.stationary_claim_speed_epsilon_mps must not exceed "
                "stationary_speed_tolerance_mps"
            )
        if self.maximum_integrated_distance_disagreement_ratio > 1.0:
            raise ContractError(
                "runtime.maximum_integrated_distance_disagreement_ratio must not "
                "exceed 1.0"
            )
        if self.minimum_low_speed_progress_ratio > 1.0:
            raise ContractError(
                "runtime.minimum_low_speed_progress_ratio must not exceed 1.0"
            )
        # HH_260906 - Keep both axes large enough for a full-horizon legal turn.
        maximum_horizon_distance_m = (
            self.maximum_speed_mps * self.timestep_s * self.future_points
        )
        if min(self.maximum_abs_x_m, self.maximum_abs_y_m) < maximum_horizon_distance_m:
            raise ContractError(
                "runtime spatial gates must cover the speed-limited prediction horizon"
            )


@dataclass(frozen=True)
class SelectedTrajectory:
    """One validated base-frame trajectory selected by the model logits."""

    candidate_index: int
    xy_base_m: tuple[tuple[float, float], ...]
    speed_mps: tuple[float, ...]
    timestep_s: float
    logit_margin: float
    planar_extent_m: float
    heading_rad: tuple[float, ...] = ()


@dataclass(frozen=True)
class RuntimeHealth:
    """The explicit inputs used to decide whether inference may run."""

    now_ns: int
    image_stamp_ns: int
    odometry_stamp_ns: int
    route_ready: bool
    calibration_ready: bool
    inference_busy: bool = False


def build_ego_features(
    *,
    velocity_x_mps: float,
    velocity_y_mps: float,
    acceleration_x_mps2: float,
    acceleration_y_mps2: float,
    yaw_rate_radps: float,
    steering_tire_angle_rad: float,
    command: int,
) -> tuple[float, ...]:
    """Build the exact 13-value feature vector used during training."""
    if isinstance(command, bool) or not isinstance(command, int):
        raise ContractError("runtime command must be an integer")
    if not 0 <= command < len(COMMAND_NAMES):
        raise ContractError("runtime command must be in [0, 5]")
    physical = tuple(
        _finite_number(value, f"runtime ego feature {name}")
        for name, value in (
            ("velocity_x_mps", velocity_x_mps),
            ("velocity_y_mps", velocity_y_mps),
            ("acceleration_x_mps2", acceleration_x_mps2),
            ("acceleration_y_mps2", acceleration_y_mps2),
            ("yaw_rate_radps", yaw_rate_radps),
            ("steering_tire_angle_rad", steering_tire_angle_rad),
        )
    )
    one_hot = tuple(float(index == command) for index in range(len(COMMAND_NAMES)))
    features = (1.0, *physical, *one_hot)
    if len(features) != len(FEATURE_NAMES):
        raise ContractError("runtime ego feature ABI does not match the training ABI")
    return features


class EgoHistory:
    """Keep a causal 10 Hz suffix and reset it after timestamp discontinuities."""

    def __init__(self, frames: int = 10, maximum_gap_s: float = 0.15) -> None:
        self.frames = _positive_integer(frames, "runtime history frames")
        self.maximum_gap_s = _finite_number(
            maximum_gap_s, "runtime history maximum gap"
        )
        if self.maximum_gap_s <= 0.0:
            raise ContractError("runtime history maximum gap must be positive")
        self._values: deque[tuple[int, tuple[float, ...]]] = deque(maxlen=self.frames)

    def append(self, timestamp_ns: int, features: Sequence[float]) -> None:
        if (
            isinstance(timestamp_ns, bool)
            or not isinstance(timestamp_ns, int)
            or timestamp_ns < 0
        ):
            raise ContractError("runtime ego timestamp must be a nonnegative integer")
        parsed = tuple(
            _finite_number(value, f"runtime ego history feature {index}")
            for index, value in enumerate(features)
        )
        if len(parsed) != len(FEATURE_NAMES):
            raise ContractError("runtime ego history has the wrong feature width")
        if self._values:
            previous_stamp = self._values[-1][0]
            if timestamp_ns <= previous_stamp:
                raise ContractError("runtime ego timestamps must increase")
            gap_s = (timestamp_ns - previous_stamp) * 1.0e-9
            if gap_s > self.maximum_gap_s:
                self._values.clear()
        self._values.append((timestamp_ns, parsed))

    def padded(self) -> tuple[tuple[tuple[float, ...], ...], tuple[bool, ...]]:
        if not self._values:
            raise ContractError("runtime ego history is empty")
        zero = (0.0,) * len(FEATURE_NAMES)
        missing = self.frames - len(self._values)
        values = (zero,) * missing + tuple(value for _, value in self._values)
        mask = (False,) * missing + (True,) * len(self._values)
        return values, mask

    @property
    def sample_count(self) -> int:
        return len(self._values)


def runtime_health_reasons(
    health: RuntimeHealth,
    *,
    maximum_image_age_s: float = 0.25,
    maximum_odometry_age_s: float = 0.2,
    maximum_sensor_skew_s: float = 0.05,
) -> tuple[str, ...]:
    """Return every fail-closed reason that blocks one inference cycle."""
    limits = {
        "maximum_image_age_s": maximum_image_age_s,
        "maximum_odometry_age_s": maximum_odometry_age_s,
        "maximum_sensor_skew_s": maximum_sensor_skew_s,
    }
    for name, raw_value in limits.items():
        value = _finite_number(raw_value, f"runtime health {name}")
        if value <= 0.0:
            raise ContractError(f"runtime health {name} must be positive")
    timestamps = {
        "now_ns": health.now_ns,
        "image_stamp_ns": health.image_stamp_ns,
        "odometry_stamp_ns": health.odometry_stamp_ns,
    }
    if any(
        isinstance(value, bool) or not isinstance(value, int) or value < 0
        for value in timestamps.values()
    ):
        raise ContractError("runtime health timestamps must be nonnegative integers")
    reasons: list[str] = []
    image_age_s = (health.now_ns - health.image_stamp_ns) * 1.0e-9
    odometry_age_s = (health.now_ns - health.odometry_stamp_ns) * 1.0e-9
    sensor_skew_s = abs(health.image_stamp_ns - health.odometry_stamp_ns) * 1.0e-9
    if image_age_s < -1.0e-6 or image_age_s > maximum_image_age_s:
        reasons.append("image_age")
    if odometry_age_s < -1.0e-6 or odometry_age_s > maximum_odometry_age_s:
        reasons.append("odometry_age")
    if sensor_skew_s > maximum_sensor_skew_s:
        reasons.append("sensor_skew")
    if not health.route_ready:
        reasons.append("route_missing")
    if not health.calibration_ready:
        reasons.append("calibration_missing")
    if health.inference_busy:
        reasons.append("inference_busy")
    return tuple(reasons)


def _prediction_rows(
    value: Any,
    *,
    outer: int,
    inner: int,
    width: int | None,
    context: str,
) -> tuple[Any, ...]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ContractError(f"{context} must be a sequence")
    if len(value) != outer:
        raise ContractError(f"{context} must contain {outer} candidates")
    parsed_outer: list[Any] = []
    for candidate_index, candidate in enumerate(value):
        if isinstance(candidate, (str, bytes)) or not isinstance(candidate, Sequence):
            raise ContractError(f"{context}[{candidate_index}] must be a sequence")
        if len(candidate) != inner:
            raise ContractError(
                f"{context}[{candidate_index}] must contain {inner} future points"
            )
        if width is None:
            parsed_outer.append(
                tuple(
                    _finite_number(
                        item, f"{context}[{candidate_index}][{point_index}]"
                    )
                    for point_index, item in enumerate(candidate)
                )
            )
            continue
        points: list[tuple[float, ...]] = []
        for point_index, point in enumerate(candidate):
            if isinstance(point, (str, bytes)) or not isinstance(point, Sequence):
                raise ContractError(
                    f"{context}[{candidate_index}][{point_index}] must be a sequence"
                )
            if len(point) != width:
                raise ContractError(
                    f"{context}[{candidate_index}][{point_index}] must contain {width} values"
                )
            points.append(
                tuple(
                    _finite_number(
                        item,
                        f"{context}[{candidate_index}][{point_index}][{axis_index}]",
                    )
                    for axis_index, item in enumerate(point)
                )
            )
        parsed_outer.append(tuple(points))
    return tuple(parsed_outer)


def validate_and_select_trajectory(
    candidate_xy: Any,
    candidate_speed: Any,
    candidate_logits: Any,
    config: RuntimeGateConfig | None = None,
    *,
    current_speed_mps: float | None = None,
) -> SelectedTrajectory:
    """Parse every candidate and physically validate the highest-logit trajectory."""
    cfg = RuntimeGateConfig() if config is None else config
    cfg.validate()
    xy = _prediction_rows(
        candidate_xy,
        outer=cfg.candidate_count,
        inner=cfg.future_points,
        width=2,
        context="runtime candidate_xy",
    )
    speeds = _prediction_rows(
        candidate_speed,
        outer=cfg.candidate_count,
        inner=cfg.future_points,
        width=None,
        context="runtime candidate_speed",
    )
    if isinstance(candidate_logits, (str, bytes)) or not isinstance(
        candidate_logits, Sequence
    ):
        raise ContractError("runtime candidate_logits must be a sequence")
    if len(candidate_logits) != cfg.candidate_count:
        raise ContractError(
            f"runtime candidate_logits must contain {cfg.candidate_count} values"
        )
    logits = tuple(
        _finite_number(value, f"runtime candidate_logits[{index}]")
        for index, value in enumerate(candidate_logits)
    )
    selected_index = max(range(cfg.candidate_count), key=logits.__getitem__)
    selected_xy = xy[selected_index]
    selected_speed = speeds[selected_index]

    # HH_260906 - Bound the first and subsequent speed changes before ROS derives acceleration.
    previous_speed_mps: float | None = None
    if current_speed_mps is not None:
        previous_speed_mps = _finite_number(
            current_speed_mps, "runtime current_speed_mps"
        )
        if not 0.0 <= previous_speed_mps <= cfg.maximum_speed_mps:
            raise ContractError("runtime current speed exceeds the speed gate")
    previous_geometric_speed_mps = previous_speed_mps

    previous = (0.0, 0.0)
    heading_anchor = (0.0, 0.0)
    previous_heading = 0.0
    heading_path_m = 0.0
    heading_block_max_speed_mps = 0.0
    heading_block_max_geometric_speed_mps = 0.0
    heading_block_max_step_m = 0.0
    previous_heading_uncertainty_m = 0.0
    selected_heading: list[float] = []
    extent_m = 0.0
    integrated_speed_distance_m = 0.0
    accumulated_distance_disagreement_m = 0.0
    maximum_radius_m = 0.0
    monotonic_nonincreasing_speed = True
    for index, ((x_m, y_m), speed_mps) in enumerate(
        zip(selected_xy, selected_speed)
    ):
        if abs(x_m) > cfg.maximum_abs_x_m or abs(y_m) > cfg.maximum_abs_y_m:
            raise ContractError(f"selected trajectory point {index} exceeds the spatial gate")
        if speed_mps < 0.0 or speed_mps > cfg.maximum_speed_mps:
            raise ContractError(f"selected trajectory speed {index} exceeds the speed gate")
        segment_entry_speed_mps = previous_speed_mps
        if previous_speed_mps is not None:
            # HH_260906 - Track a strict braking profile for the bounded-stop exemption.
            if speed_mps > previous_speed_mps + 1.0e-9:
                monotonic_nonincreasing_speed = False
            speed_rate_mps2 = (
                speed_mps - previous_speed_mps
            ) / cfg.timestep_s
            if (
                speed_rate_mps2 > cfg.maximum_acceleration_mps2 + 1.0e-9
                or speed_rate_mps2 < -cfg.maximum_deceleration_mps2 - 1.0e-9
            ):
                raise ContractError(
                    f"selected trajectory speed rate {index} exceeds the acceleration gate"
                )
        previous_speed_mps = speed_mps
        dx = x_m - previous[0]
        dy = y_m - previous[1]
        step_m = math.hypot(dx, dy)
        forward_step_m = (
            dx * math.cos(previous_heading) + dy * math.sin(previous_heading)
        )
        if index == 0:
            if step_m > cfg.maximum_first_point_distance_m:
                raise ContractError("selected trajectory first point is too far from ego")
            if x_m < cfg.minimum_first_point_x_m:
                raise ContractError("selected trajectory begins behind ego")
        # HH_260906 - Reject hidden reverse motion below the heading deadband.
        if forward_step_m < -cfg.maximum_backward_step_m - 1.0e-9:
            raise ContractError(
                f"selected trajectory step {index} moves backward"
            )
        if step_m > cfg.maximum_step_m:
            raise ContractError(f"selected trajectory step {index} exceeds the distance gate")
        # HH_260906 - Reject trajectories whose reported speed contradicts their motion.
        geometric_speed_mps = step_m / float(cfg.timestep_s)
        if geometric_speed_mps > cfg.maximum_speed_mps + 1.0e-4:
            raise ContractError(
                f"selected trajectory geometric speed {index} exceeds the speed gate"
            )
        if abs(geometric_speed_mps - speed_mps) > cfg.maximum_speed_disagreement_mps:
            raise ContractError(
                f"selected trajectory speed {index} disagrees with its displacement"
            )
        segment_entry_geometric_speed_mps = previous_geometric_speed_mps
        # HH_260906 - Gate displacement-derived acceleration so speed errors cannot alternate.
        if previous_geometric_speed_mps is not None:
            geometric_speed_rate_mps2 = (
                geometric_speed_mps - previous_geometric_speed_mps
            ) / cfg.timestep_s
            if (
                geometric_speed_rate_mps2 > cfg.maximum_acceleration_mps2 + 1.0e-4
                or geometric_speed_rate_mps2
                < -cfg.maximum_deceleration_mps2 - 1.0e-4
            ):
                raise ContractError(
                    f"selected trajectory geometric speed rate {index} exceeds "
                    "the acceleration gate"
                )
        previous_geometric_speed_mps = geometric_speed_mps
        extent_m += step_m
        integrated_speed_distance_m += speed_mps * cfg.timestep_s
        accumulated_distance_disagreement_m += abs(
            step_m - speed_mps * cfg.timestep_s
        )
        maximum_radius_m = max(maximum_radius_m, math.hypot(x_m, y_m))
        heading_path_m += step_m
        if segment_entry_speed_mps is not None:
            heading_block_max_speed_mps = max(
                heading_block_max_speed_mps, segment_entry_speed_mps
            )
        heading_block_max_speed_mps = max(
            heading_block_max_speed_mps, speed_mps
        )
        if segment_entry_geometric_speed_mps is not None:
            heading_block_max_geometric_speed_mps = max(
                heading_block_max_geometric_speed_mps,
                segment_entry_geometric_speed_mps,
            )
        heading_block_max_geometric_speed_mps = max(
            heading_block_max_geometric_speed_mps, geometric_speed_mps
        )
        heading_block_max_step_m = max(heading_block_max_step_m, step_m)
        # HH_260906 - Bound path travel inside the heading deadband before accepting jitter.
        heading_dx = x_m - heading_anchor[0]
        heading_dy = y_m - heading_anchor[1]
        heading_displacement_m = math.hypot(heading_dx, heading_dy)
        if heading_displacement_m >= cfg.heading_minimum_step_m:
            heading = math.atan2(heading_dy, heading_dx)
            heading_delta = math.atan2(
                math.sin(heading - previous_heading),
                math.cos(heading - previous_heading),
            )
            if abs(heading_delta) > cfg.maximum_heading_step_rad:
                raise ContractError(
                    f"selected trajectory heading step {index} exceeds the gate"
                )
            # HH_260906 - Reject smooth-looking loops that imply impossible path curvature.
            curvature_path_m = heading_path_m + previous_heading_uncertainty_m
            implied_curvature_rad_per_m = abs(heading_delta) / curvature_path_m
            if implied_curvature_rad_per_m > (
                cfg.maximum_curvature_rad_per_m + 1.0e-9
            ):
                raise ContractError(
                    f"selected trajectory curvature at point {index} exceeds the gate"
                )
            lateral_speed_mps = max(
                heading_block_max_speed_mps,
                heading_block_max_geometric_speed_mps,
            )
            # HH_260906 - Couple curvature to speed to reject unsafe lateral acceleration.
            lateral_speed_limit_mps = math.sqrt(
                (cfg.maximum_lateral_acceleration_mps2 + 1.0e-6)
                / implied_curvature_rad_per_m
            ) if implied_curvature_rad_per_m > 0.0 else math.inf
            if lateral_speed_mps > lateral_speed_limit_mps:
                raise ContractError(
                    f"selected trajectory lateral acceleration at point {index} "
                    "exceeds the gate"
                )
            previous_heading = heading
            heading_anchor = (x_m, y_m)
            # HH_260906 - Carry only aggregation uncertainty from sub-deadband steps.
            previous_heading_uncertainty_m = max(
                0.0, heading_path_m - heading_block_max_step_m
            )
            heading_path_m = 0.0
            heading_block_max_speed_mps = 0.0
            heading_block_max_geometric_speed_mps = 0.0
            heading_block_max_step_m = 0.0
        elif heading_path_m >= cfg.heading_minimum_step_m:
            unresolved_motion_speed_mps = max(
                heading_block_max_speed_mps,
                heading_block_max_geometric_speed_mps,
            )
            if unresolved_motion_speed_mps > cfg.stationary_speed_tolerance_mps:
                raise ContractError(
                    f"selected trajectory moving heading {index} is unresolved"
                )
            # HH_260906 - Retain low-speed unresolved travel until its direction resolves.
            if heading_path_m > cfg.maximum_stationary_extent_m:
                raise ContractError(
                    f"selected trajectory moving heading {index} is unresolved"
                )
        selected_heading.append(previous_heading)
        previous = (x_m, y_m)
    maximum_selected_speed_mps = max(selected_speed)
    # HH_260906 - Separate an explicit zero-speed claim from valid measurable crawl motion.
    claimed_stationary_drift = (
        maximum_selected_speed_mps <= cfg.stationary_claim_speed_epsilon_mps
        and (
            maximum_radius_m > cfg.maximum_stationary_radius_m
            or extent_m > cfg.maximum_stationary_extent_m
        )
    )
    confined_oscillation = (
        maximum_radius_m <= cfg.maximum_stationary_radius_m
        and extent_m > cfg.maximum_stationary_extent_m
    )
    final_radius_m = math.hypot(*previous)
    low_speed_oscillation = (
        maximum_selected_speed_mps <= cfg.stationary_speed_tolerance_mps
        and extent_m > cfg.maximum_stationary_extent_m
        and final_radius_m
        < cfg.minimum_low_speed_progress_ratio * extent_m
    )
    # HH_260906 - Reject shifted low-speed shuttling as well as origin-confined jitter.
    if claimed_stationary_drift or confined_oscillation or low_speed_oscillation:
        raise ContractError("selected stationary trajectory exceeds the drift gate")
    integrated_distance_tolerance_m = max(
        cfg.maximum_integrated_distance_disagreement_m,
        cfg.maximum_integrated_distance_disagreement_ratio
        * max(extent_m, integrated_speed_distance_m),
    )
    # HH_260906 - Reject repeated speed lies outside absolute and relative horizon slack.
    if accumulated_distance_disagreement_m > integrated_distance_tolerance_m:
        raise ContractError(
            "selected trajectory distance disagrees with integrated speed"
        )
    dynamically_consistent_stop = (
        monotonic_nonincreasing_speed
        and selected_speed[-1] <= cfg.stationary_claim_speed_epsilon_mps
    )
    # HH_260906 - Permit a bounded low-speed stop while requiring extent for faster motion.
    if (
        extent_m < cfg.minimum_planar_extent_m
        and maximum_selected_speed_mps > cfg.stationary_speed_tolerance_mps
        and not dynamically_consistent_stop
    ):
        raise ContractError("selected trajectory has insufficient planar extent")
    ranked = sorted(logits, reverse=True)
    return SelectedTrajectory(
        candidate_index=selected_index,
        xy_base_m=selected_xy,
        speed_mps=selected_speed,
        timestep_s=float(cfg.timestep_s),
        logit_margin=ranked[0] - ranked[1] if len(ranked) > 1 else math.inf,
        planar_extent_m=extent_m,
        heading_rad=tuple(selected_heading),
    )


def transform_base_trajectory_to_map(
    trajectory: SelectedTrajectory,
    *,
    ego_x_m: float,
    ego_y_m: float,
    ego_yaw_rad: float,
) -> tuple[tuple[float, float, float, float], ...]:
    """Return map-frame x, y, yaw, and speed for each selected point."""
    ego_x = _finite_number(ego_x_m, "runtime ego_x_m")
    ego_y = _finite_number(ego_y_m, "runtime ego_y_m")
    ego_yaw = _finite_number(ego_yaw_rad, "runtime ego_yaw_rad")
    cosine = math.cos(ego_yaw)
    sine = math.sin(ego_yaw)
    positions = tuple(
        (
            ego_x + cosine * x_m - sine * y_m,
            ego_y + sine * x_m + cosine * y_m,
        )
        for x_m, y_m in trajectory.xy_base_m
    )
    if len(trajectory.heading_rad) != len(positions):
        raise ContractError("validated trajectory heading count does not match its points")
    output: list[tuple[float, float, float, float]] = []
    # HH_260906 - Publish exactly the base headings accepted by the geometry gate.
    for index, ((x_m, y_m), speed_mps, base_heading) in enumerate(
        zip(positions, trajectory.speed_mps, trajectory.heading_rad)
    ):
        parsed_heading = _finite_number(
            base_heading, f"validated trajectory heading[{index}]"
        )
        map_heading = math.atan2(
            math.sin(ego_yaw + parsed_heading),
            math.cos(ego_yaw + parsed_heading),
        )
        output.append((x_m, y_m, map_heading, speed_mps))
    return tuple(output)


class ExactCameraBundle:
    """Accept a six-camera bundle only when every source timestamp is identical."""

    def __init__(
        self,
        camera_order: Sequence[str] = CAMERA_ORDER,
        *,
        retention_ns: int = 1_000_000_000,
        maximum_pending_bundles: int = 32,
    ) -> None:
        parsed = tuple(camera_order)
        if not parsed or any(not isinstance(name, str) or not name for name in parsed):
            raise ContractError("runtime camera order must contain nonempty names")
        if len(set(parsed)) != len(parsed):
            raise ContractError("runtime camera order contains duplicates")
        _positive_integer(retention_ns, "runtime camera retention_ns")
        _positive_integer(
            maximum_pending_bundles, "runtime camera maximum_pending_bundles"
        )
        self.camera_order = parsed
        self._retention_ns = retention_ns
        self._maximum_pending_bundles = maximum_pending_bundles
        self._by_stamp: dict[int, dict[str, Any]] = {}
        self._last_emitted_stamp_ns = -1
        self._maximum_seen_stamp_ns = -1
        self._dropped_stale_count = 0
        self._expired_pending_count = 0
        self._evicted_capacity_count = 0

    @property
    def pending_bundle_count(self) -> int:
        return len(self._by_stamp)

    def counters(self) -> dict[str, int]:
        return {
            "pending_bundle_count": len(self._by_stamp),
            "dropped_stale_count": self._dropped_stale_count,
            "expired_pending_count": self._expired_pending_count,
            "evicted_capacity_count": self._evicted_capacity_count,
        }

    def push(self, camera: str, stamp_ns: int, payload: Any) -> None:
        if camera not in self.camera_order:
            raise ContractError(f"unexpected runtime camera {camera!r}")
        if isinstance(stamp_ns, bool) or not isinstance(stamp_ns, int) or stamp_ns < 0:
            raise ContractError("runtime camera timestamp must be nonnegative")
        if stamp_ns <= self._last_emitted_stamp_ns:
            self._dropped_stale_count += 1
            return
        self._maximum_seen_stamp_ns = max(self._maximum_seen_stamp_ns, stamp_ns)
        minimum_kept = max(
            self._last_emitted_stamp_ns + 1,
            self._maximum_seen_stamp_ns - self._retention_ns,
        )
        if stamp_ns < minimum_kept:
            self._dropped_stale_count += 1
            return
        bundle = self._by_stamp.setdefault(stamp_ns, {})
        if camera in bundle:
            raise ContractError(
                f"duplicate runtime camera frame {camera!r} at {stamp_ns}"
            )
        bundle[camera] = payload
        # HH_260906 - Prune against a monotonic watermark and enforce a hard memory bound.
        for existing_stamp in tuple(self._by_stamp):
            if existing_stamp < minimum_kept:
                del self._by_stamp[existing_stamp]
                self._expired_pending_count += 1
        excess = len(self._by_stamp) - self._maximum_pending_bundles
        if excess > 0:
            for existing_stamp in sorted(self._by_stamp)[:excess]:
                del self._by_stamp[existing_stamp]
                self._evicted_capacity_count += 1

    def pop_latest(self) -> tuple[int, tuple[Any, ...]] | None:
        complete = [
            stamp
            for stamp, bundle in self._by_stamp.items()
            if all(camera in bundle for camera in self.camera_order)
        ]
        if not complete:
            return None
        stamp_ns = max(complete)
        bundle = self._by_stamp[stamp_ns]
        payloads = tuple(bundle[camera] for camera in self.camera_order)
        self._last_emitted_stamp_ns = stamp_ns
        self._by_stamp = {
            stamp: value for stamp, value in self._by_stamp.items() if stamp > stamp_ns
        }
        return stamp_ns, payloads


def validate_calibration_features(
    calibration: Iterable[Sequence[float]],
    *,
    camera_count: int = len(CAMERA_ORDER),
    feature_count: int = 16,
) -> tuple[tuple[float, ...], ...]:
    """Validate the normalized K and T_base_from_camera runtime tensor."""
    parsed_rows = tuple(tuple(row) for row in calibration)
    if len(parsed_rows) != camera_count:
        raise ContractError(f"runtime calibration must contain {camera_count} cameras")
    output: list[tuple[float, ...]] = []
    for camera_index, row in enumerate(parsed_rows):
        if len(row) != feature_count:
            raise ContractError(
                f"runtime calibration camera {camera_index} must contain {feature_count} values"
            )
        output.append(
            tuple(
                _finite_number(
                    value, f"runtime calibration[{camera_index}][{feature_index}]"
                )
                for feature_index, value in enumerate(row)
            )
        )
    return tuple(output)
