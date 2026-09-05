# HH_260906 - Verify the fail-closed live inference contract without ROS, CARLA, or PyTorch.

import math

import pytest

from portable_e2e import ContractError
from portable_e2e.dataset import FEATURE_NAMES
from portable_e2e.runtime_contract import CAMERA_ORDER
from portable_e2e.runtime_contract import EgoHistory
from portable_e2e.runtime_contract import ExactCameraBundle
from portable_e2e.runtime_contract import RuntimeGateConfig
from portable_e2e.runtime_contract import RuntimeHealth
from portable_e2e.runtime_contract import build_ego_features
from portable_e2e.runtime_contract import runtime_health_reasons
from portable_e2e.runtime_contract import transform_base_trajectory_to_map
from portable_e2e.runtime_contract import validate_and_select_trajectory
from portable_e2e.runtime_contract import validate_calibration_features


def _valid_predictions():
    xy = []
    speed = []
    for candidate in range(6):
        lateral = 0.002 * candidate
        xy.append([(0.1 * (index + 1), lateral * index) for index in range(64)])
        consistent_speed_mps = math.hypot(0.1, lateral) / 0.1
        speed.append([consistent_speed_mps] * 64)
    return xy, speed, [-2.0, -1.0, 3.0, 0.5, -0.5, -0.25]


def test_build_ego_features_preserves_training_abi():
    features = build_ego_features(
        velocity_x_mps=8.0,
        velocity_y_mps=0.1,
        acceleration_x_mps2=0.2,
        acceleration_y_mps2=-0.1,
        yaw_rate_radps=0.03,
        steering_tire_angle_rad=0.04,
        command=2,
    )

    assert len(features) == len(FEATURE_NAMES) == 13
    assert features[:7] == pytest.approx((1.0, 8.0, 0.1, 0.2, -0.1, 0.03, 0.04))
    assert features[7:] == (0.0, 0.0, 1.0, 0.0, 0.0, 0.0)


@pytest.mark.parametrize("command", (-1, 6, True, 1.5))
def test_build_ego_features_rejects_invalid_command(command):
    with pytest.raises(ContractError, match="command"):
        build_ego_features(
            velocity_x_mps=0.0,
            velocity_y_mps=0.0,
            acceleration_x_mps2=0.0,
            acceleration_y_mps2=0.0,
            yaw_rate_radps=0.0,
            steering_tire_angle_rad=0.0,
            command=command,
        )


def test_ego_history_returns_contiguous_suffix():
    history = EgoHistory(frames=3, maximum_gap_s=0.15)
    first = build_ego_features(
        velocity_x_mps=1.0,
        velocity_y_mps=0.0,
        acceleration_x_mps2=0.0,
        acceleration_y_mps2=0.0,
        yaw_rate_radps=0.0,
        steering_tire_angle_rad=0.0,
        command=3,
    )
    second = tuple(value + 1.0 for value in first)
    history.append(1_000_000_000, first)
    history.append(1_100_000_000, second)

    values, mask = history.padded()

    assert mask == (False, True, True)
    assert values[0] == (0.0,) * len(FEATURE_NAMES)
    assert values[1:] == (first, second)


def test_ego_history_resets_after_gap_and_rejects_time_reversal():
    history = EgoHistory(frames=3, maximum_gap_s=0.15)
    features = (0.0,) * len(FEATURE_NAMES)
    history.append(1_000_000_000, features)
    history.append(1_300_000_000, features)
    assert history.sample_count == 1
    with pytest.raises(ContractError, match="increase"):
        history.append(1_200_000_000, features)


def test_runtime_health_passes_fresh_synchronized_inputs():
    reasons = runtime_health_reasons(
        RuntimeHealth(
            now_ns=1_200_000_000,
            image_stamp_ns=1_100_000_000,
            odometry_stamp_ns=1_090_000_000,
            route_ready=True,
            calibration_ready=True,
        )
    )

    assert reasons == ()


def test_runtime_health_reports_all_independent_blockers():
    reasons = runtime_health_reasons(
        RuntimeHealth(
            now_ns=2_000_000_000,
            image_stamp_ns=1_000_000_000,
            odometry_stamp_ns=1_700_000_000,
            route_ready=False,
            calibration_ready=False,
            inference_busy=True,
        )
    )

    assert reasons == (
        "image_age",
        "odometry_age",
        "sensor_skew",
        "route_missing",
        "calibration_missing",
        "inference_busy",
    )


def test_selects_highest_logit_and_reports_geometry():
    xy, speed, logits = _valid_predictions()

    selected = validate_and_select_trajectory(xy, speed, logits)

    assert selected.candidate_index == 2
    assert selected.logit_margin == pytest.approx(2.5)
    assert len(selected.xy_base_m) == 64
    assert selected.planar_extent_m > 6.0


@pytest.mark.parametrize(
    ("mutation", "message"),
    (
        (lambda xy, speed, logits: xy[2].__setitem__(0, (math.nan, 0.0)), "finite"),
        (lambda xy, speed, logits: speed[2].__setitem__(4, 9.0), "speed gate"),
        (lambda xy, speed, logits: xy[2].__setitem__(4, (8.0, 0.0)), "distance gate"),
        (lambda xy, speed, logits: xy[2].__setitem__(0, (-0.3, 0.0)), "behind ego"),
        (lambda xy, speed, logits: logits.__setitem__(2, math.inf), "finite"),
    ),
)
def test_selected_trajectory_rejects_unsafe_output(mutation, message):
    xy, speed, logits = _valid_predictions()
    mutation(xy, speed, logits)

    with pytest.raises(ContractError, match=message):
        validate_and_select_trajectory(xy, speed, logits)


def test_rejects_wrong_prediction_shape_before_selection():
    xy, speed, logits = _valid_predictions()
    xy.pop()
    with pytest.raises(ContractError, match="6 candidates"):
        validate_and_select_trajectory(xy, speed, logits)


def test_rejects_excessive_heading_discontinuity():
    xy, speed, logits = _valid_predictions()
    xy[2][1] = (0.1, 0.1)
    with pytest.raises(ContractError, match="heading step"):
        validate_and_select_trajectory(xy, speed, logits)


def test_stationary_trajectory_is_a_valid_fail_safe_plan():
    xy, speed, logits = _valid_predictions()
    xy[2] = [(0.0, 0.0)] * 64
    speed[2] = [0.0] * 64

    selected = validate_and_select_trajectory(xy, speed, logits)

    assert selected.candidate_index == 2
    assert selected.planar_extent_m == 0.0


def test_stationary_geometry_rejects_claimed_motion():
    xy, speed, logits = _valid_predictions()
    xy[2] = [(0.0, 0.0)] * 64
    speed[2] = [3.0] * 64

    with pytest.raises(ContractError, match="disagrees with its displacement"):
        validate_and_select_trajectory(xy, speed, logits)


def test_heading_gate_ignores_subcentimeter_stationary_jitter():
    xy, speed, logits = _valid_predictions()
    xy[2] = [
        ((-1.0 if index % 2 else 1.0) * 0.001, 0.0) for index in range(64)
    ]
    speed[2] = [0.02] * 64

    selected = validate_and_select_trajectory(xy, speed, logits)

    assert selected.planar_extent_m > 0.05


def test_stationary_gate_rejects_accumulated_drift():
    xy, speed, logits = _valid_predictions()
    xy[2] = [(0.009 * (index + 1), 0.0) for index in range(64)]
    speed[2] = [0.0] * 64

    with pytest.raises(ContractError, match="stationary trajectory exceeds"):
        validate_and_select_trajectory(xy, speed, logits)


def test_stationary_gate_accepts_geometry_consistent_crawl_motion():
    # HH_260906 - Do not reinterpret a measurable low-speed trajectory as a zero-speed claim.
    xy, speed, logits = _valid_predictions()
    xy[2] = [(0.005 * (index + 1), 0.0) for index in range(64)]
    speed[2] = [0.05] * 64

    selected = validate_and_select_trajectory(
        xy,
        speed,
        logits,
        current_speed_mps=0.05,
    )

    assert selected.planar_extent_m == pytest.approx(0.32)


@pytest.mark.parametrize("claimed_speed_mps", (0.0001001, 0.0002, 0.01, 0.05))
def test_integrated_distance_gate_rejects_low_speed_drift_bypass(
    claimed_speed_mps,
):
    # HH_260906 - Close the epsilon-boundary bypass across the complete horizon.
    xy, speed, logits = _valid_predictions()
    xy[2] = [(0.1 * (index + 1), 0.0) for index in range(64)]
    speed[2] = [claimed_speed_mps] * 64

    with pytest.raises(ContractError, match="disagrees with its displacement"):
        validate_and_select_trajectory(
            xy,
            speed,
            logits,
        )


def test_integrated_distance_gate_rejects_cancelling_step_errors():
    # HH_260906 - Sum absolute step errors so alternating over and under motion cannot cancel.
    xy, speed, logits = _valid_predictions()
    x_m = 0.0
    points = []
    for index in range(64):
        x_m += 0.08 if index % 2 == 0 else 0.12
        points.append((x_m, 0.0))
    xy[2] = points
    speed[2] = [1.0] * 64

    with pytest.raises(ContractError, match="integrated speed"):
        validate_and_select_trajectory(
            xy,
            speed,
            logits,
            current_speed_mps=1.0,
            config=RuntimeGateConfig(
                maximum_acceleration_mps2=5.0,
                maximum_deceleration_mps2=5.0,
            ),
        )


def test_heading_gate_rejects_fast_confined_subthreshold_oscillation():
    xy, speed, logits = _valid_predictions()
    xy[2] = [((index % 2) * 0.009, 0.0) for index in range(64)]
    speed[2] = [0.101] * 64

    with pytest.raises(ContractError, match="moving heading"):
        validate_and_select_trajectory(
            xy,
            speed,
            logits,
            config=RuntimeGateConfig(maximum_backward_step_m=0.01),
        )


def test_heading_gate_rejects_low_speed_pure_lateral_deadband_shuttle():
    # HH_260906 - Preserve unresolved travel so subcentimeter side slip cannot reset heading.
    xy, speed, logits = _valid_predictions()
    y_m = 0.0
    points = []
    selected_speed = []
    for index in range(64):
        dy_m = (0.0099, -0.0002)[index % 2]
        y_m += dy_m
        points.append((0.0, y_m))
        selected_speed.append(abs(dy_m) / 0.1)
    xy[2] = points
    speed[2] = selected_speed

    with pytest.raises(ContractError, match="heading step"):
        validate_and_select_trajectory(
            xy,
            speed,
            logits,
            current_speed_mps=0.099,
        )


def test_heading_gate_rejects_oscillation_just_outside_stationary_radius():
    xy, speed, logits = _valid_predictions()
    xy[2] = [(0.009 * (index + 1), 0.0) for index in range(6)]
    xy[2].extend(
        (0.045 if index % 2 == 0 else 0.054, 0.0) for index in range(58)
    )
    speed[2] = [0.09] * 6 + [0.101] * 58

    with pytest.raises(ContractError, match="moving heading"):
        validate_and_select_trajectory(
            xy,
            speed,
            logits,
            config=RuntimeGateConfig(maximum_backward_step_m=0.01),
        )


def test_curvature_gate_rejects_smooth_confined_loop_bypass():
    # HH_260906 - Regress the sub-decimeter circular loop that bypassed radius-only guards.
    xy, speed, logits = _valid_predictions()
    radius_m = 0.0255
    angle_step_rad = 0.6
    xy[2] = [
        (
            radius_m * math.sin((index + 1) * angle_step_rad),
            radius_m * (1.0 - math.cos((index + 1) * angle_step_rad)),
        )
        for index in range(64)
    ]
    step_m = 2.0 * radius_m * math.sin(angle_step_rad / 2.0)
    speed[2] = [step_m / 0.1] * 64

    with pytest.raises(ContractError, match="curvature"):
        validate_and_select_trajectory(
            xy,
            speed,
            logits,
            current_speed_mps=speed[2][0],
        )


def test_lateral_acceleration_gate_rejects_high_speed_tight_arc():
    # HH_260906 - Regress a curvature-valid arc whose speed implies unsafe lateral force.
    xy, speed, logits = _valid_predictions()
    step_m = 0.8
    curvature_rad_per_m = 0.49
    x_m = 0.0
    y_m = 0.0
    points = []
    for index in range(64):
        heading_rad = curvature_rad_per_m * step_m * index
        x_m += step_m * math.cos(heading_rad)
        y_m += step_m * math.sin(heading_rad)
        points.append((x_m, y_m))
    xy[2] = points
    speed[2] = [8.0] * 64

    with pytest.raises(ContractError, match="lateral acceleration"):
        validate_and_select_trajectory(
            xy,
            speed,
            logits,
            current_speed_mps=8.0,
        )


def test_geometric_speed_rate_rejects_alternating_displacement_acceleration():
    # HH_260906 - Reject alternating XY speed errors even when reported speed is smooth.
    xy, speed, logits = _valid_predictions()
    x_m = 0.0
    points = []
    for index in range(64):
        x_m += (4.7 if index % 2 == 0 else 5.1) * 0.1
        points.append((x_m, 0.0))
    xy[2] = points
    speed[2] = [4.9] * 64

    with pytest.raises(ContractError, match="geometric speed rate"):
        validate_and_select_trajectory(xy, speed, logits)


def test_geometric_speed_gate_rejects_motion_above_reported_speed_limit():
    # HH_260906 - Apply the 30 km/h cap to XY-derived speed as well as reported speed.
    xy, speed, logits = _valid_predictions()
    geometric_speed_mps = 8.0
    reported_speed_mps = 8.0
    x_m = 0.0
    points = []
    reported = []
    for _ in range(64):
        reported_speed_mps = min(8.333333333333334, reported_speed_mps + 0.29)
        geometric_speed_mps = min(9.74, geometric_speed_mps + 0.29)
        x_m += geometric_speed_mps * 0.1
        points.append((x_m, 0.0))
        reported.append(reported_speed_mps)
    xy[2] = points
    speed[2] = reported

    with pytest.raises(ContractError, match="geometric speed"):
        validate_and_select_trajectory(
            xy,
            speed,
            logits,
            current_speed_mps=8.0,
        )


def test_low_speed_progress_gate_rejects_shifted_shuttle_motion():
    # HH_260906 - Reject crawl-speed shuttling that stays just outside the origin radius.
    xy, speed, logits = _valid_predictions()
    xy[2] = [(0.009 * (index + 1), 0.0) for index in range(6)]
    xy[2].extend(
        (0.045 if index % 2 == 0 else 0.054, 0.0) for index in range(58)
    )
    speed[2] = [0.09] * 64

    with pytest.raises(ContractError, match="moving heading"):
        validate_and_select_trajectory(
            xy,
            speed,
            logits,
            config=RuntimeGateConfig(maximum_backward_step_m=0.01),
        )


def test_backward_step_gate_rejects_hidden_low_speed_heading_reversal():
    # HH_260906 - Reject a reverse step even when every displacement is under 1 cm.
    xy, speed, logits = _valid_predictions()
    x_m = 0.0
    points = []
    for index in range(64):
        direction = -1.0 if index == 60 else 1.0
        x_m += direction * 0.0099
        points.append((x_m, 0.0))
    xy[2] = points
    speed[2] = [0.099] * 64

    with pytest.raises(ContractError, match="moves backward"):
        validate_and_select_trajectory(
            xy,
            speed,
            logits,
            current_speed_mps=0.099,
        )


def test_rejects_speed_that_disagrees_with_geometry():
    xy, speed, logits = _valid_predictions()
    speed[2] = [5.0] * 64

    with pytest.raises(ContractError, match="disagrees with its displacement"):
        validate_and_select_trajectory(xy, speed, logits)


def test_rejects_future_speed_rate_outside_acceleration_gate():
    xy, speed, logits = _valid_predictions()
    speed[2][1] = 2.0

    with pytest.raises(ContractError, match="speed rate"):
        validate_and_select_trajectory(xy, speed, logits)


def test_rejects_first_speed_rate_against_current_ego_speed():
    xy, speed, logits = _valid_predictions()

    with pytest.raises(ContractError, match="speed rate"):
        validate_and_select_trajectory(
            xy,
            speed,
            logits,
            current_speed_mps=8.0,
        )


def test_accepts_first_speed_rate_near_current_ego_speed():
    xy, speed, logits = _valid_predictions()

    selected = validate_and_select_trajectory(
        xy,
        speed,
        logits,
        current_speed_mps=1.2,
    )

    assert selected.speed_mps[0] == pytest.approx(math.hypot(0.1, 0.004) / 0.1)


def test_rejects_negative_current_ego_speed_without_clamping():
    xy, speed, logits = _valid_predictions()

    with pytest.raises(ContractError, match="current speed"):
        validate_and_select_trajectory(
            xy,
            speed,
            logits,
            current_speed_mps=-0.01,
        )


def test_accepts_short_dynamically_consistent_braking_stop():
    # HH_260906 - Permit a legal monotonic stop even when its total distance is under 5 cm.
    xy, speed, logits = _valid_predictions()
    selected_speed = [0.11] + [0.0] * 63
    x_m = 0.0
    selected_xy = []
    for speed_mps in selected_speed:
        x_m += speed_mps * 0.1
        selected_xy.append((x_m, 0.0))
    xy[2] = selected_xy
    speed[2] = selected_speed

    selected = validate_and_select_trajectory(
        xy,
        speed,
        logits,
        current_speed_mps=0.4,
    )

    assert selected.planar_extent_m == pytest.approx(0.011)


@pytest.mark.parametrize(
    "selected_speed",
    (
        [0.11] + [0.0002] * 63,
        [0.11, 0.0, 0.01] + [0.0] * 61,
    ),
)
def test_short_stop_exemption_rejects_nonstop_or_reaccelerating_profiles(
    selected_speed,
):
    # HH_260906 - Keep the short-stop exemption limited to monotonic profiles ending at zero.
    xy, speed, logits = _valid_predictions()
    x_m = 0.0
    selected_xy = []
    for speed_mps in selected_speed:
        x_m += speed_mps * 0.1
        selected_xy.append((x_m, 0.0))
    xy[2] = selected_xy
    speed[2] = selected_speed

    with pytest.raises(ContractError, match="insufficient planar extent"):
        validate_and_select_trajectory(
            xy,
            speed,
            logits,
            current_speed_mps=0.4,
        )


def test_short_stop_exemption_rejects_initial_acceleration_from_current_speed():
    # HH_260906 - Include measured current speed when deciding whether a profile only brakes.
    xy, speed, logits = _valid_predictions()
    selected_speed = [0.11] + [0.0] * 63
    x_m = 0.0
    selected_xy = []
    for speed_mps in selected_speed:
        x_m += speed_mps * 0.1
        selected_xy.append((x_m, 0.0))
    xy[2] = selected_xy
    speed[2] = selected_speed

    with pytest.raises(ContractError, match="insufficient planar extent"):
        validate_and_select_trajectory(
            xy,
            speed,
            logits,
            current_speed_mps=0.0,
        )


def test_spatial_gate_covers_a_full_speed_limited_turn_horizon():
    config = RuntimeGateConfig()

    assert config.maximum_abs_x_m >= (
        config.maximum_speed_mps * config.timestep_s * config.future_points
    )
    assert config.maximum_abs_y_m >= (
        config.maximum_speed_mps * config.timestep_s * config.future_points
    )


def test_transforms_base_trajectory_to_map():
    xy, speed, logits = _valid_predictions()
    selected = validate_and_select_trajectory(xy, speed, logits)

    mapped = transform_base_trajectory_to_map(
        selected,
        ego_x_m=10.0,
        ego_y_m=20.0,
        ego_yaw_rad=math.pi / 2.0,
    )

    assert mapped[0][0] == pytest.approx(10.0)
    assert mapped[0][1] == pytest.approx(20.1)
    assert mapped[0][2] == pytest.approx(math.pi / 2.0)
    assert mapped[0][3] == pytest.approx(math.hypot(0.1, 0.004) / 0.1)


def test_transform_holds_yaw_for_subcentimeter_stationary_jitter():
    xy, speed, logits = _valid_predictions()
    xy[2] = [
        ((-1.0 if index % 2 else 1.0) * 0.001, 0.0) for index in range(64)
    ]
    speed[2] = [0.02] * 64
    selected = validate_and_select_trajectory(xy, speed, logits)

    mapped = transform_base_trajectory_to_map(
        selected,
        ego_x_m=10.0,
        ego_y_m=20.0,
        ego_yaw_rad=0.0,
    )

    assert {point[2] for point in mapped} == {0.0}


def test_transform_reuses_gate_heading_across_stationary_to_moving_transition():
    xy, speed, logits = _valid_predictions()
    heading = math.atan2(0.0001, 0.02)
    points = [(0.009, 0.0), (0.02, 0.0001)]
    x_m = 0.02
    y_m = 0.0001
    for step_m in (0.04, 0.07, *([0.1] * 60)):
        x_m += step_m * math.cos(heading)
        y_m += step_m * math.sin(heading)
        points.append((x_m, y_m))
    xy[2] = points
    speed[2] = [
        0.09,
        math.hypot(0.011, 0.0001) / 0.1,
        0.4,
        0.7,
        1.0,
    ] + [1.0] * 59

    selected = validate_and_select_trajectory(xy, speed, logits)
    mapped = transform_base_trajectory_to_map(
        selected,
        ego_x_m=0.0,
        ego_y_m=0.0,
        ego_yaw_rad=0.0,
    )

    assert mapped[0][2] == 0.0
    assert mapped[1][2] == pytest.approx(heading)
    assert abs(mapped[1][2] - mapped[0][2]) <= 0.75


def test_exact_camera_bundle_uses_model_order_and_latest_complete_stamp():
    synchronizer = ExactCameraBundle()
    for camera in reversed(CAMERA_ORDER):
        synchronizer.push(camera, 100, f"old:{camera}")
    for camera in CAMERA_ORDER:
        synchronizer.push(camera, 200, f"new:{camera}")

    stamp, payloads = synchronizer.pop_latest()

    assert stamp == 200
    assert payloads == tuple(f"new:{camera}" for camera in CAMERA_ORDER)
    assert synchronizer.pop_latest() is None


def test_exact_camera_bundle_rejects_duplicate_frame():
    synchronizer = ExactCameraBundle()
    synchronizer.push("CAM_FRONT", 100, object())
    with pytest.raises(ContractError, match="duplicate"):
        synchronizer.push("CAM_FRONT", 100, object())


def test_exact_camera_bundle_bounds_descending_timestamp_memory():
    synchronizer = ExactCameraBundle(maximum_pending_bundles=32)

    for index in range(5_000):
        synchronizer.push("CAM_FRONT", 20_000_000_000 - index * 2_000_000, index)

    assert synchronizer.pending_bundle_count <= 32
    assert synchronizer.counters()["evicted_capacity_count"] > 0


def test_exact_camera_bundle_far_future_watermark_prunes_old_bundles():
    synchronizer = ExactCameraBundle(maximum_pending_bundles=32)
    synchronizer.push("CAM_FRONT", 100, "old")
    synchronizer.push("CAM_FRONT", 2_000_000_000, "future")
    synchronizer.push("CAM_BACK", 200, "stale")

    assert synchronizer.pending_bundle_count == 1
    assert set(synchronizer._by_stamp) == {2_000_000_000}
    assert synchronizer.counters()["dropped_stale_count"] == 1
    assert synchronizer.counters()["expired_pending_count"] == 1


def test_exact_camera_bundle_retains_valid_out_of_order_completion():
    synchronizer = ExactCameraBundle(maximum_pending_bundles=32)
    synchronizer.push("CAM_FRONT", 200, "new-front")
    for camera in CAMERA_ORDER:
        if camera != "CAM_FRONT":
            synchronizer.push(camera, 100, f"old:{camera}")
    synchronizer.push("CAM_FRONT", 100, "old:CAM_FRONT")

    stamp, payloads = synchronizer.pop_latest()

    assert stamp == 100
    assert payloads == tuple(f"old:{camera}" for camera in CAMERA_ORDER)


def test_calibration_requires_six_finite_rows():
    calibration = [[float(index)] * 16 for index in range(6)]
    assert validate_calibration_features(calibration)[5][0] == 5.0
    calibration[3][2] = math.nan
    with pytest.raises(ContractError, match="finite"):
        validate_calibration_features(calibration)


def test_runtime_gate_configuration_fails_closed():
    with pytest.raises(ContractError, match="must not exceed maximum_step_m"):
        RuntimeGateConfig(
            maximum_step_m=1.0,
            maximum_first_point_distance_m=1.1,
        ).validate()
    with pytest.raises(ContractError, match="stationary_claim_speed_epsilon_mps"):
        RuntimeGateConfig(
            stationary_claim_speed_epsilon_mps=0.2,
            stationary_speed_tolerance_mps=0.1,
        ).validate()
    with pytest.raises(ContractError, match="disagreement_ratio"):
        RuntimeGateConfig(
            maximum_integrated_distance_disagreement_ratio=1.01,
        ).validate()
    with pytest.raises(ContractError, match="progress_ratio"):
        RuntimeGateConfig(minimum_low_speed_progress_ratio=1.01).validate()
    with pytest.raises(ContractError, match="maximum_backward_step_m"):
        RuntimeGateConfig(maximum_backward_step_m=0.011).validate()
