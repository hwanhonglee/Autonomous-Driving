"""HH_260906 - Test opt-in comfort collection without launching or importing CARLA."""

from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts/e2e"))

from carla_goal_stop_profile import (
    ComfortableGoalStopConfig, DevelopmentGoalStopConfig, DevelopmentGoalStopGovernor,
    GoalStopGovernor, annotate_development_control, bounded_route_projection, complete_terminal_plan,
    configuration_from_args, goal_stop_termination_reason, install_normal_brake_cap,
    install_development_control, measured_goal_completion, measured_stop_quality,
    source_motion_bounds, terminal_overshoot_m, validate_development_route,
)
from collect_carla_vad_expert import parse_args


CFG = ComfortableGoalStopConfig()
BOUNDS = {
    "physical_decoder": {"maximum_acceleration_mps2": 2.9, "maximum_deceleration_mps2": 2.9},
    "runtime_speed_rate_gate": {"maximum_acceleration_mps2": 3.0, "maximum_deceleration_mps2": 6.0},
}


def arguments(**changes):
    values = dict(goal_stop_profile="comfortable_v1", physics_hz=20.0, capture_hz=10.0,
                  goal_tolerance_m=1.0, target_speed_kmh=30.0, stationary_tail_sec=6.5)
    values.update(changes)
    return SimpleNamespace(**values)


def point(x, y, distance):
    return {"x": x, "y": y, "distance_m": distance}


def test_default_is_disabled_and_cli_does_not_change_legacy_settings():
    args = parse_args(["unused-output", "unused-route"])
    assert args.goal_stop_profile == "disabled"
    assert args.goal_tolerance_m == 2.5
    assert args.stationary_tail_sec == 0.0
    assert configuration_from_args(args) is None
    assert configuration_from_args(SimpleNamespace()) is None


def test_exact_profile_configuration_and_cli():
    args = parse_args(["unused-output", "unused-route", "--goal-stop-profile", "comfortable_v1",
                       "--goal-tolerance-m", "1.0", "--stationary-tail-sec", "6.5", "--target-speed-kmh", "30"])
    assert asdict(configuration_from_args(args)) == asdict(CFG)


def test_v2_changes_only_declared_control_parameters_not_stop_or_quality_criteria():
    original = asdict(configuration_from_args(arguments()))
    revised = asdict(configuration_from_args(arguments(goal_stop_profile="comfortable_v2")))
    changed = {key: value for key, value in revised.items() if original[key] != value}
    assert changed == {"profile_id": "comfortable_v2", "desired_deceleration_mps2": 1.0,
                       "normal_throttle_cap": 0.20, "normal_brake_cap": 0.15}
    assert original["normal_throttle_cap"] is None
    assert original["desired_deceleration_mps2"] == 1.5


@pytest.mark.parametrize("flags", [["--ho", "127.0.0.2"], ["--po", "2000"], ["--allow-m"]])
def test_collector_rejects_abbreviated_ownership_sensitive_options(flags):
    with pytest.raises(SystemExit):
        parse_args(["unused-output", "unused-route", *flags])


def test_full_spelling_arguments_still_parse_explicitly():
    args = parse_args(["unused-output", "unused-route", "--host", "127.0.0.1", "--port", "2100"])
    assert args.host == "127.0.0.1" and args.port == 2100 and not args.allow_map_load


@pytest.mark.parametrize("changes", [
    {"physics_hz": 10}, {"capture_hz": 5}, {"goal_tolerance_m": 2.5},
    {"stationary_tail_sec": 6.4}, {"stationary_tail_sec": math.nan},
    {"target_speed_kmh": 60}, {"target_speed_kmh": math.inf},
    {"target_speed_kmh": 0}, {"goal_stop_profile": "unknown"},
])
def test_profile_refuses_incompatible_settings(changes):
    with pytest.raises(ValueError):
        configuration_from_args(arguments(**changes))


def pilot_arguments(**changes):
    # HH_260906 - Keep the pilot opt-in and distinguish nominal 28.8 km/h from the hard 30 km/h cap.
    values = dict(goal_stop_profile="comfortable_v3", target_speed_kmh=28.8,
                  stationary_warmup_sec=3.5, vehicle_type="vehicle.toyota.prius")
    values.update(changes)
    return arguments(**values)


def pilot():
    return DevelopmentGoalStopGovernor(DevelopmentGoalStopConfig(), 8.0, 20.0)


def pilot_tick(governor, timestamp, speed=0.0, remaining=100.0, **changes):
    options = dict(timestamp=timestamp, longitudinal_speed_mps=speed, driving=True, count_hold=True)
    options.update(changes)
    return governor.update(remaining, remaining, speed, -remaining, **options)


def test_v3_exact_config_and_route_preflight_do_not_change_legacy_profiles():
    cfg = configuration_from_args(pilot_arguments())
    assert cfg == DevelopmentGoalStopConfig()
    assert cfg.maximum_actual_speed_mps == 30 / 3.6
    assert cfg.stop_buffer_m == CFG.stop_buffer_m
    assert cfg.goal_tolerance_m == CFG.goal_tolerance_m
    assert not cfg.training_data_approved and cfg.development_only
    path = ROOT / "docs/assets/validation/2026-09-01/town07/autoware_vad/straight/autoware_vad_route.json"
    validate_development_route(pilot_arguments(), json.loads(path.read_text()), path)
    # HH_260906 - Legacy profiles do not start reading or pinning route files through the new preflight.
    validate_development_route(arguments(), {}, Path("unused-missing-route"))


@pytest.mark.parametrize("changes", [
    {"target_speed_kmh": 30.0}, {"target_speed_kmh": 28.7}, {"stationary_warmup_sec": 0},
    {"vehicle_type": "vehicle.other"}, {"max_duration_sec": 180.01}, {"max_duration_sec": math.inf},
])
def test_v3_rejects_unfrozen_pilot_settings(changes):
    with pytest.raises(ValueError):
        configuration_from_args(pilot_arguments(**changes))


@pytest.mark.parametrize("kind", ["bytes", "town", "scenario", "length", "weather"])
def test_v3_rejects_route_or_weather_drift_before_simulator_access(tmp_path, kind):
    source = ROOT / "docs/assets/validation/2026-09-01/town07/autoware_vad/straight/autoware_vad_route.json"
    route = json.loads(source.read_text())
    path = source
    args = pilot_arguments()
    if kind == "bytes":
        path = tmp_path / "modified.json"
        path.write_bytes(source.read_bytes() + b"\n")
    if kind == "town": route["town"] = "Town03"
    if kind == "scenario": route["scenario"] = "left"
    if kind == "length": route["route_length_m"] += 0.1
    if kind == "weather": args.weather = "WetNoon"
    with pytest.raises(ValueError):
        validate_development_route(args, route, path)


def test_v3_launch_is_fixed_then_pid_cap_rises_per_elapsed_second():
    g = pilot()
    pilot_tick(g, 4.0, driving=False, count_hold=False)
    initial = pilot_tick(g, 4.0, count_hold=False)
    control = SimpleNamespace(throttle=0.0, brake=1.0, steer=0.37)
    assert initial["driving_elapsed_sec"] == 0.0
    assert initial["target_speed_mps"] == pytest.approx(0.05)
    assert g.normal_control(control) is control
    assert (control.throttle, control.brake, control.steer) == (0.15, 0.0, 0.37)
    transition = pilot_tick(g, 4.05, speed=0.5)
    assert transition["pilot_transition"] == "launch_to_normal_pid"
    assert transition["normal_throttle_cap"] == pytest.approx(0.1525)
    for tick in range(2, 103):
        record = pilot_tick(g, 4.0 + tick * 0.05, speed=2.0)
    assert record["normal_throttle_cap"] == 0.4
    controlled = g.normal_control(SimpleNamespace(throttle=0.75, brake=0.0, steer=-0.28))
    assert (controlled.throttle, controlled.brake, controlled.steer) == (0.4, 0.0, -0.28)


def test_v3_high_approach_envelope_keeps_reference_speed_until_coast():
    g = pilot()
    pilot_tick(g, 0.0, speed=0.5)
    g.last_record["target_speed_mps"] = 8.0
    record = pilot_tick(g, 0.05, speed=4.0, remaining=50.0)
    expected = math.sqrt(g.config.approach_reference_speed_mps ** 2 + 2 * 0.6 * 15)
    assert record["target_speed_mps"] == pytest.approx(expected)
    record = pilot_tick(g, 0.1, speed=3.0, remaining=30.0)
    assert record["target_speed_mps"] == pytest.approx(g.config.approach_reference_speed_mps)
    assert not record["coast_entry_latched"]


def test_v3_launch_timeout_uses_eight_seconds_not_warmup_ticks():
    g = pilot()
    pilot_tick(g, 100.0, count_hold=False)
    for tick in range(1, 160):
        record = pilot_tick(g, 100 + tick * 0.05)
        assert record["pilot_failure_reason"] is None
    record = pilot_tick(g, 108.0)
    assert record["pilot_failure_reason"] == "comfortable_v3_launch_timeout"


@pytest.mark.parametrize("timestamp", [0.0, -0.05, 0.1, math.nan, math.inf])
def test_v3_rejects_duplicate_backward_missing_or_nonfinite_native_clock(timestamp):
    g = pilot()
    pilot_tick(g, 0.0)
    with pytest.raises(ValueError):
        pilot_tick(g, timestamp)


@pytest.mark.parametrize("speed,vx", [(8.333333333333336, 8.0), (8.0, 8.333333333333336)])
def test_v3_actual_planar_and_longitudinal_speed_are_independent_hard_failures(speed, vx):
    record = pilot_tick(pilot(), 0.0, speed=speed, longitudinal_speed_mps=vx)
    assert record["pilot_failure_reason"] == "comfortable_v3_actual_speed_exceeded"


@pytest.mark.parametrize("speed", [2.7, 3.3])
def test_v3_refuses_out_of_band_coast_entry_without_latching_or_rescue(speed):
    g = pilot()
    pilot_tick(g, 0.0, speed=0.5)
    record = pilot_tick(g, 0.05, speed=speed, remaining=0.75 + 6.93 * speed)
    assert record["pilot_failure_reason"] == "comfortable_v3_coast_entry_speed_outside_band"
    assert not record["coast_entry_latched"]


def test_v3_coast_latches_zero_pedals_with_lateral_control_and_no_relaunch():
    g = pilot()
    pilot_tick(g, 0.0, speed=0.5)
    record = pilot_tick(g, 0.05, speed=3.0, remaining=20.0)
    assert record["pilot_transition"] == "normal_pid_to_zero_pedal_coast"
    assert record["target_speed_mps"] == 0.0
    for tick in range(2, 6):
        record = pilot_tick(g, tick * 0.05, speed=0.2, remaining=5.0)
        control = g.normal_control(SimpleNamespace(throttle=0.75, brake=1.0, steer=0.31))
        assert (control.throttle, control.brake, control.steer) == (0.0, 0.0, 0.31)
        assert record["pilot_transition"] is None
    assert goal_stop_termination_reason(record, True, 0.0, g.config) == "basic_agent_done_before_comfortable_goal_window"


def test_v3_coast_timeout_is_retained_without_forcing_positive_brake():
    g = pilot()
    pilot_tick(g, 0.0, speed=0.5)
    pilot_tick(g, 0.05, speed=3.0, remaining=20.0)
    for tick in range(2, 902):
        record = pilot_tick(g, tick * 0.05, speed=0.2, remaining=5.0)
    assert record["coast_elapsed_sec"] == pytest.approx(45.0)
    assert record["pilot_failure_reason"] == "comfortable_v3_coast_timeout"
    control = g.normal_control(SimpleNamespace(throttle=0.7, brake=1.0, steer=0.2))
    assert control.brake == 0.0


def test_v3_inside_window_empty_queue_still_requires_measured_two_second_dwell():
    g = pilot()
    pilot_tick(g, 0.0, speed=0.5)
    pilot_tick(g, 0.05, speed=3.0, remaining=20.0)
    for tick in range(2, 42):
        record = pilot_tick(g, tick * 0.05, speed=0.05, remaining=0.75)
        assert not record["complete"]
        assert goal_stop_termination_reason(record, True, 0.0, g.config) is None
    record = pilot_tick(g, 2.1, speed=0.05, remaining=0.75)
    assert record["complete"] and record["hold_duration_sec"] == 2.0
    assert goal_stop_termination_reason(record, True, 0.0, g.config) == "comfortable_goal_measured_stop_and_dwell"
    tail = pilot_tick(g, 2.15, speed=0.0, remaining=0.75, driving=False, count_hold=False)
    assert tail["coast_elapsed_sec"] == record["coast_elapsed_sec"]
    assert tail["driving_elapsed_sec"] == record["driving_elapsed_sec"]
    assert tail["target_speed_mps"] == 0.0 and tail["complete"]


def test_v3_emergency_return_identity_order_and_pending_next_tick_failure():
    g = pilot()
    pilot_tick(g, 0.0, speed=0.5)
    pilot_tick(g, 0.05, speed=3.0, remaining=20.0)
    planner = SimpleNamespace(_vehicle_controller=SimpleNamespace(max_brake=0.3, max_throt=0.75))
    planner.run_step = lambda: SimpleNamespace(throttle=0.5, brake=0.0, steer=0.29)
    returned = []
    def emergency(control):
        assert control.throttle == control.brake == 0.0
        control.brake = 0.5
        returned.append(control)
        return control
    agent = SimpleNamespace(get_local_planner=lambda: planner, add_emergency_stop=emergency)
    metadata = install_development_control(agent, g)
    record = dict(g.last_record)
    control = agent.add_emergency_stop(planner.run_step())
    assert control is returned[0] and control.brake == 0.5 and control.steer == 0.29
    annotate_development_control(record, g, None)
    assert record["next_control_is_unchanged_emergency_override"]
    assert record["emergency_failure_pending_next_tick"]
    assert record["emergency_override_count"] == 1
    assert "emergency_override_requested" in record["control_source"]
    # HH_260906 - Model the next measured native tick after the caller applies the returned emergency brake.
    observed = pilot_tick(g, 0.1, speed=2.8, remaining=19.85)
    reason = goal_stop_termination_reason(observed, False, 0.0, g.config)
    assert reason == "comfortable_v3_emergency_override"
    annotate_development_control(observed, g, reason)
    assert not observed["emergency_failure_pending_next_tick"]
    assert not metadata["emergency_return_control_modified"]


def test_cli_rejects_implicit_legacy_tolerance_for_new_profile():
    with pytest.raises(SystemExit):
        parse_args(["unused-output", "unused-route", "--goal-stop-profile", "comfortable_v1"])


def test_route_arc_speed_envelope_is_not_the_euclidean_distance():
    governor = GoalStopGovernor(CFG, 30 / 3.6, 20)
    governor.target_speed_mps = 30 / 3.6
    result = governor.update(20.0, 0.2, 8.0, -0.2)
    assert result["distance_speed_envelope_mps"] == pytest.approx(math.sqrt(3 * (20 - 0.75)))
    assert not result["goal_window"]
    assert result["hold_ticks"] == 0


def test_target_acceleration_is_slewed_but_reduction_uses_distance_immediately():
    governor = GoalStopGovernor(CFG, 30 / 3.6, 20)
    assert governor.update(100, 100, 0, -100)["target_speed_mps"] == pytest.approx(0.075)
    assert governor.update(99, 99, 0.1, -99)["target_speed_mps"] == pytest.approx(0.150)
    governor.target_speed_mps = 8.0
    assert governor.update(1.0, 1.0, 8.0, -1.0)["target_speed_mps"] == pytest.approx(math.sqrt(0.75))


def test_position_only_completion_at_eight_mps_is_forbidden():
    governor = GoalStopGovernor(CFG, 30 / 3.6, 20)
    for _ in range(100):
        record = governor.update(0.75, 0.75, 8.0, -0.75)
    assert record["goal_window"]
    assert not record["complete"]
    assert record["hold_ticks"] == 0
    assert goal_stop_termination_reason(record, True, 0.0, CFG) is None


def test_dwell_uses_full_two_seconds_and_resets_on_speed_motion():
    governor = GoalStopGovernor(CFG, 30 / 3.6, 20)
    for _ in range(40):
        record = governor.update(0.75, 0.75, 0.0, -0.75)
    assert record["hold_duration_sec"] == pytest.approx(1.95)
    assert not record["complete"]
    record = governor.update(0.75, 0.75, 0.0, -0.75)
    assert record["complete"] and record["hold_duration_sec"] == 2.0
    assert goal_stop_termination_reason(record, True, 0.0, CFG) == "comfortable_goal_measured_stop_and_dwell"
    assert not governor.update(0.75, 0.75, 0.100001, -0.75)["complete"]


@pytest.mark.parametrize("remaining, planar, overshoot", [(1.01, 0.75, -0.75), (0.75, 1.01, -0.75), (0.0, 0.0, 0.0), (0.01, 0.01, 0.01)])
def test_stop_requires_arc_planar_and_upstream_endpoint(remaining, planar, overshoot):
    governor = GoalStopGovernor(CFG, 30 / 3.6, 20)
    for _ in range(50):
        record = governor.update(remaining, planar, 0.0, overshoot)
    assert not record["complete"]


def test_agent_done_outside_goal_window_is_failed_not_waited_or_success():
    record = GoalStopGovernor(CFG, 30 / 3.6, 20).update(5, 5, 8, -5)
    assert goal_stop_termination_reason(record, True, 0, CFG) == "basic_agent_done_before_comfortable_goal_window"
    assert goal_stop_termination_reason(record, False, 3.01, CFG) == "comfortable_goal_route_projection_failure"


def test_projection_cannot_jump_to_nearby_late_route_segment():
    route = [point(0, 0, 0), point(10, 0, 10), point(10, 10, 20), point(0, 0.1, 34)]
    progress, error, index = bounded_route_projection(route, 0.2, 0.1, 0, 1)
    assert progress == pytest.approx(0.2)
    assert error == pytest.approx(0.1)
    assert index == 0


def test_projection_clips_long_segment_to_actual_arc_window_and_stays_monotonic():
    route = [point(0, 0, 0), point(100, 0, 100)]
    assert bounded_route_projection(route, 80, 0, 2, 1)[0] == 3
    assert bounded_route_projection(route, 0, 0, 2, 1)[0] == 2
    assert terminal_overshoot_m(route, 100.2, 0) == pytest.approx(0.2)
    assert terminal_overshoot_m(route, 99.25, 0) == pytest.approx(-0.75)


def test_emergency_override_remains_above_normal_brake_cap():
    planner = SimpleNamespace(_vehicle_controller=SimpleNamespace(max_brake=0.3))
    planner.run_step = lambda: SimpleNamespace(brake=1.0, throttle=0.0)
    metadata = install_normal_brake_cap(planner, 0.10)
    assert planner._vehicle_controller.max_brake == 0.10
    assert planner.run_step().brake == 0.10
    # HH_260906 - Match the upstream order: local planner first, BasicAgent hazard override second.
    def agent_step(hazard):
        control = planner.run_step()
        if hazard:
            control.brake = 0.5
        return control
    assert agent_step(True).brake == 0.5
    assert not metadata["basic_agent_emergency_brake_modified"]
    assert metadata["original_pid_max_brake"] == 0.3


def test_v2_caps_normal_throttle_and_brake_before_unchanged_hazard_override():
    planner = SimpleNamespace(_vehicle_controller=SimpleNamespace(max_brake=0.3, max_throt=0.75))
    planner.run_step = lambda: SimpleNamespace(brake=1.0, throttle=0.75)
    metadata = install_normal_brake_cap(planner, 0.15, 0.20)
    assert planner._vehicle_controller.max_throt == 0.20
    assert planner._vehicle_controller.max_brake == 0.15
    control = planner.run_step()
    assert control.brake == 0.15 and control.throttle == 0.20
    # HH_260906 - Emergency override remains later in the upstream BasicAgent call order.
    control.throttle, control.brake = 0.0, 0.5
    assert control.brake == 0.5
    assert metadata["original_pid_max_throttle"] == 0.75
    assert not metadata["basic_agent_emergency_brake_modified"]


def waypoint(x, y=0.0, road=1, section=0, lane=-1, yaw=0.0):
    return SimpleNamespace(road_id=road, section_id=section, lane_id=lane,
                           transform=SimpleNamespace(location=SimpleNamespace(x=x, y=y, z=0.0),
                                                     rotation=SimpleNamespace(yaw=yaw)))


def test_terminal_plan_appends_actual_goal_waypoint_without_mutating_original_route():
    plan = [(waypoint(0), "LANEFOLLOW"), (waypoint(8.6), "LANEFOLLOW")]
    goal = waypoint(10)
    updated, report = complete_terminal_plan(plan, goal, {"x": 10, "y": 0, "z": 0}, 1.0)
    assert len(plan) == 2 and len(updated) == 3
    assert updated[-1][0] is goal
    assert report["original_to_goal_gap_m"] == pytest.approx(1.4)
    assert not report["catalog_route_changed"] and not report["goal_tolerance_changed"]


def test_terminal_plan_does_not_duplicate_an_already_exact_endpoint():
    goal = waypoint(10)
    plan = [(waypoint(0), "LANEFOLLOW"), (goal, "LANEFOLLOW")]
    updated, report = complete_terminal_plan(plan, goal, {"x": 10, "y": 0, "z": 0}, 1.0)
    assert len(updated) == 2
    assert not report["appended_goal_waypoint"]


@pytest.mark.parametrize("case", ["wrong_lane", "wrong_road", "wrong_section", "large_gap", "bad_projection", "behind", "missing"])
def test_terminal_plan_refuses_disconnected_or_invented_endpoint(case):
    plan = [(waypoint(0), "LANEFOLLOW"), (waypoint(8.6), "LANEFOLLOW")]
    goal = waypoint(10)
    requested = {"x": 10, "y": 0, "z": 0}
    if case == "wrong_lane": goal.lane_id = 1
    if case == "wrong_road": goal.road_id = 2
    if case == "wrong_section": goal.section_id = 1
    if case == "large_gap": plan[-1] = (waypoint(6), "LANEFOLLOW")
    if case == "bad_projection": requested["y"] = 0.26
    if case == "behind": plan[-1] = (waypoint(11), "LANEFOLLOW")
    if case == "missing": goal = None
    with pytest.raises(ValueError):
        complete_terminal_plan(plan, goal, requested, 1.0)


def test_tail_completion_retains_dwell_but_requires_current_goal_and_speed():
    governor = GoalStopGovernor(CFG, 30 / 3.6, 20)
    for _ in range(41):
        record = governor.update(0.75, 0.75, 0.0, -0.75)
    assert measured_goal_completion(record, 0.0, CFG)
    tail = governor.update(0.75, 0.75, 0.0, -0.75, count_hold=False)
    assert measured_goal_completion(tail, 0.0, CFG)
    moving_tail = governor.update(0.75, 0.75, 0.2, -0.75, count_hold=False)
    assert not measured_goal_completion(moving_tail, 0.0, CFG)
    outside_tail = governor.update(1.1, 1.1, 0.0, -1.1, count_hold=False)
    assert not measured_goal_completion(outside_tail, 0.0, CFG)


def test_collector_package_style_import_is_preserved():
    import importlib
    module = importlib.import_module("scripts.e2e.collect_carla_vad_expert")
    assert module.parse_args(["unused-output", "unused-route"]).goal_stop_profile == "disabled"


def stopped_records():
    governor = GoalStopGovernor(CFG, 30 / 3.6, 20)
    records = []
    for index in range(171):
        phase = "driving" if index < 41 else "stationary_tail"
        record = governor.update(0.75, 0.75, 0.0, -0.75, count_hold=phase == "driving")
        records.append(dict(frame=index, timestamp=index * 0.05, vx=0.0, vy=0.0,
                            route_cte_m=0.0, capture_phase=phase, goal_stop=record))
    return records


def quality(records, completed=True):
    return measured_stop_quality(records, [r["frame"] for r in records if r["frame"] % 2 == 0], CFG, BOUNDS, completed)


def test_quality_measures_both_cadences_and_full_stopped_tail():
    report = quality(stopped_records())
    assert report["status"] == "PASS"
    assert report["tail_state_count"] == 130
    assert report["moving_tail_frame_count"] == 0
    assert report["measurements"]["native_20hz"]["stationary_tail"]["interval_count"] == 130
    assert report["measurements"]["camera_10hz"]["stationary_tail"]["interval_count"] == 65


def pilot_quality_records():
    # HH_260906 - Synthetic scalar QA fixtures do not claim position/XY feasibility or actual simulator motion.
    speeds = ([index * 0.05 for index in range(160)] + [8.0] * 101
              + [8.0 - index * 0.05 for index in range(1, 161)] + [0.0] * 41)
    governor = GoalStopGovernor(CFG, 8.0, 20.0)
    records = []
    driving_count = len(speeds)
    for index, speed in enumerate(speeds + [0.0] * 130):
        phase = "driving" if index < driving_count else "stationary_tail"
        goal_stop = governor.update(0.75, 0.75, speed, -0.75, count_hold=phase == "driving")
        records.append(dict(frame=index, timestamp=index * 0.05, vx=speed, vy=0.0,
                            route_cte_m=0.0, capture_phase=phase, goal_stop=goal_stop))
    return records


def pilot_quality(records):
    return measured_stop_quality(records, [r["frame"] for r in records[::2]],
                                 DevelopmentGoalStopConfig(), BOUNDS, True)


def test_v3_quality_recomputes_continuous_cruise_and_keeps_admission_false():
    report = pilot_quality(pilot_quality_records())
    assert report["status"] == "PASS"
    detail = report["development_pilot"]
    assert detail["cruise_requirement_met"]
    assert detail["longest_continuous_cruise_seconds"] >= 5.0
    assert detail["actual_speed_violation_count"] == 0
    assert not detail["training_data_approved"]
    assert "development_pilot" not in quality(stopped_records())


def test_v3_missing_cruise_fails_even_with_a_forged_governor_timer():
    records = stopped_records()
    for row in records:
        row["goal_stop"]["longest_continuous_cruise_seconds"] = 1000.0
    report = pilot_quality(records)
    assert report["status"] == "FAIL"
    assert not report["development_pilot"]["cruise_requirement_met"]
    assert report["development_pilot"]["longest_continuous_cruise_seconds"] == 0.0


def test_v3_cruise_segments_cannot_be_added_across_out_of_band_samples_or_clock_gaps():
    records = pilot_quality_records()
    cruising = [i for i, r in enumerate(records) if r["vx"] == 8.0]
    middle = cruising[len(cruising) // 2]
    records[middle]["vx"] = 7.79
    report = pilot_quality(records)
    assert not report["development_pilot"]["cruise_requirement_met"]
    records = pilot_quality_records()
    for row in records[middle:]:
        row["timestamp"] += 0.05
    report = pilot_quality(records)
    assert report["status"] == "FAIL"
    assert not report["development_pilot"]["cruise_requirement_met"]


def test_v3_actual_maximum_speed_is_not_replaced_by_nominal_target_or_cruise_success():
    records = pilot_quality_records()
    records[10]["vx"] = 8.333333333333336
    report = pilot_quality(records)
    assert report["status"] == "FAIL"
    assert report["development_pilot"]["actual_speed_violation_count"] == 1
    assert report["development_pilot"]["cruise_requirement_met"]


def test_v3_hazard_record_keeps_quality_failed_even_if_scalar_curve_would_pass():
    records = pilot_quality_records()
    records[10]["goal_stop"]["pilot_failure_reason"] = "comfortable_v3_emergency_override"
    report = pilot_quality(records)
    assert report["status"] == "FAIL"
    assert report["development_pilot"]["pilot_failure_reasons"] == ["comfortable_v3_emergency_override"]


def test_quality_never_labels_2_95_as_decoder_feasible_or_runtime_decel_3():
    records = stopped_records()
    records[1]["vx"] = 2.95 * 0.05
    report = quality(records)
    native = report["measurements"]["native_20hz"]["driving"]
    assert native["physical_decoder"]["violation_count"] == 2
    assert native["runtime_speed_rate_gate"]["violation_count"] == 0
    assert report["status"] == "FAIL"


def test_quality_retains_moving_tail_transition_prefix_at_both_cadences():
    records = stopped_records()
    records[41]["vx"] = 0.4
    records[42]["vx"] = 0.4
    report = quality(records)
    assert report["status"] == "FAIL"
    assert report["moving_tail_frame_count"] == 2
    for cadence in ("native_20hz", "camera_10hz"):
        violations = report["measurements"][cadence]["stationary_tail"]["physical_decoder"]["violation_intervals"]
        prefix = next(item for item in violations if item["from_phase"] == "driving")
        assert prefix["to_phase"] == "stationary_tail"
        assert prefix["to_speed_mps"] == 0.4


@pytest.mark.parametrize("case", ["incomplete", "short_tail", "overshoot", "missing_camera", "cadence_gap", "camera_suffix_only", "duplicate_camera"])
def test_quality_fails_closed_for_incomplete_or_invalid_evidence(case):
    records = stopped_records()
    cameras = [r["frame"] for r in records if r["frame"] % 2 == 0]
    if case == "short_tail":
        records.pop()
    if case == "overshoot":
        records[-1]["goal_stop"]["terminal_overshoot_m"] = 0.01
    if case == "missing_camera":
        cameras = []
    if case == "cadence_gap":
        cameras.pop(5)
    if case == "camera_suffix_only":
        cameras = [168, 170]
    if case == "duplicate_camera":
        cameras.append(cameras[-1])
    report = measured_stop_quality(records, cameras, CFG, BOUNDS, case != "incomplete")
    assert report["status"] == "FAIL"


def test_real_source_bounds_are_asymmetric_and_hash_pinned():
    bounds = source_motion_bounds()
    assert bounds["physical_decoder"] == BOUNDS["physical_decoder"]
    assert bounds["runtime_speed_rate_gate"] == BOUNDS["runtime_speed_rate_gate"]
    assert len(bounds["source_sha256"]) == 2
    assert all(len(value) == 64 for value in bounds["source_sha256"].values())


def test_failed_opt_in_capture_preserves_raw_state_profile_and_helper_hash(tmp_path, monkeypatch):
    import collect_carla_vad_expert as collector
    route_path = tmp_path / "route.json"
    route_path.write_text(json.dumps({
        "schema_version": 1, "coordinate_reference": "base_link", "town": "Town07",
        "start_carla_transform": {"x": 0, "y": 0, "z": 0},
        "goal_carla_transform": {"x": 10, "y": 0, "z": 0},
        "route": [{**point(0, 0, 0), "vad_command": 3}, {**point(10, 0, 10), "vad_command": 3}],
    }))
    output = tmp_path / "new_episode"
    args = parse_args([str(output), str(route_path), "--goal-stop-profile", "comfortable_v1",
                       "--goal-tolerance-m", "1.0", "--stationary-tail-sec", "6.5"])

    def failed_collect(_args, _route, _specs, _partial, states, _cameras, manifest):
        # HH_260906 - A simulator failure must retain evidence without promoting the partial folder.
        states.append({"frame": 42, "timestamp": 0.05, "capture_phase": "driving", "vx": 8.0})
        manifest["result"] = {"goal_reached": False, "goal_stop_quality": {"status": "FAIL"}}
        raise collector.CollectionError("measured stop failed")

    monkeypatch.setattr(collector, "collect_episode", failed_collect)
    with pytest.raises(collector.CollectionError, match="measured stop failed"):
        collector.run(args)
    assert not output.exists()
    partial = Path(str(output) + ".partial")
    manifest = json.loads((partial / "manifest.json").read_text())
    assert manifest["status"] == "failed"
    assert manifest["result"]["goal_stop_quality"]["status"] == "FAIL"
    assert manifest["capture_contract"]["goal_stop_profile"]["normal_brake_cap"] == 0.10
    assert manifest["provenance"]["goal_stop_helper_sha256"] == hashlib.sha256(
        (ROOT / "scripts/e2e/carla_goal_stop_profile.py").read_bytes()).hexdigest()
    assert json.loads((partial / "states.jsonl").read_text())["vx"] == 8.0
    with pytest.raises(collector.CollectionError, match="partial output already exists"):
        collector.run(args)
