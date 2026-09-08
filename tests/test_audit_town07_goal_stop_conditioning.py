"""HH_260906 - Test immutable low-speed conditioning diagnostics with synthetic native states only."""

from __future__ import annotations

import copy
import math
import struct

import pytest

from scripts.e2e import audit_town07_goal_stop_conditioning as module


def f32(value):
    return struct.unpack("!f", struct.pack("!f", value))[0]


def states():
    rows = []
    for i in range(131):
        row = dict(frame=i, timestamp=i * .05, x=i * .05, y=0., z=0., yaw=0., vx=1., vy=0., yaw_rate=0.,
            capture_phase="driving", steering_tire_angle_rad=0., current_control={"steer": 0., "throttle": .1, "brake": 0., "gear": 1},
            actor_snapshot_transform_carla={"x": f32(i * .05 + 1.425), "y": 0., "z": 0., "yaw": 0., "pitch": 0., "roll": 0.},
            world_velocity_carla=[1., 0., 0.], world_acceleration_carla=[0., 0., 0.],
            world_angular_velocity_carla_deg_s=[0., 0., 0.],
            goal_stop={"pilot_state": "normal_pid", "remaining_route_arc_m": 50. - i * .05})
        rows.append(row)
    return rows


def future(rows):
    steps, previous, previous_heading = [], (0., 0.), 0.
    for i in range(64):
        row = rows[(i + 1) * 2]
        vector = (row["x"] - previous[0], row["y"] - previous[1])
        distance = math.hypot(*vector)
        heading = math.atan2(vector[1], vector[0])
        curvature = abs(module.wrap(heading - previous_heading)) / distance
        steps.append({"index": i, "target_timestamp_ns": (i + 1) * 100000000,
            "episode_relative_100ms_tick": i + 1,
            "metrics": {"xy_speed_limit": [distance * 10., False], "xy_curvature": [curvature, curvature > .200001]}})
        previous, previous_heading = (row["x"], row["y"]), heading
    return dict(frame=0, anchor_timestamp_ns=0, capture_phase="driving", disposition="full_64_point_anchor",
        valid_points=64, valid_mask=[True] * 64, diagnostic={"steps": steps})


def test_binary32_spacing_is_not_double_precision_or_a_cause_claim():
    result = module.f32_spacing(198.)
    assert result["observed_value_exactly_float32_representable"] is True
    assert result["float32_spacing_upper_bound"] == 2 ** -16
    assert module.f32_spacing(.1)["observed_value_exactly_float32_representable"] is False
    assert module.f32_spacing(0.)["float32_spacing_upper_bound"] == 2 ** -149
    with pytest.raises(ValueError): module.f32_spacing(float("nan"))


def test_conditional_pose_bound_contains_translation_and_rotation_terms():
    row = states()[0]
    row["actor_snapshot_transform_carla"].update(x=198., y=-140., yaw=90.)
    result = module.pose_rounding_bound(row)
    assert result["rear_planar_position_error_bound_m"] > math.hypot(2 ** -17, 2 ** -17)
    assert result["body_yaw_error_bound_rad"] > 0
    row["actor_snapshot_transform_carla"]["pitch"] = .1
    with pytest.raises(ValueError): module.pose_rounding_bound(row)


def test_complete_64_future_reconstruction_and_no_mutation():
    rows = states(); times = module.timeline(rows); anchor = future(rows)
    before = copy.deepcopy((rows, anchor))
    assert module.check_future(anchor, rows, times) == []
    assert (rows, anchor) == before


def test_later_index_63_violation_is_never_dropped():
    rows = states(); rows[128]["y"] = 1.
    times = module.timeline(rows); anchor = future(rows)
    result = module.check_future(anchor, rows, times)
    assert len(result) == 1 and result[0]["future_index"] == 63
    assert result[0]["recomputed_original_violation"] is True
    assert result[0]["conditional_bound_over_allowed_heading_change"] >= 0


@pytest.mark.parametrize("change", ["flag", "curvature", "speed", "timestamp", "count", "mask"])
def test_published_diagnostic_corruption_fails_closed(change):
    rows = states(); anchor = future(rows)
    step = anchor["diagnostic"]["steps"][-1]
    if change == "flag": step["metrics"]["xy_curvature"][1] = True
    elif change == "curvature": step["metrics"]["xy_curvature"][0] = 1.
    elif change == "speed": step["metrics"]["xy_speed_limit"][0] = 2.
    elif change == "timestamp": step["target_timestamp_ns"] += 1
    elif change == "count": anchor["diagnostic"]["steps"].pop()
    else: anchor["valid_mask"][-1] = False
    with pytest.raises(ValueError): module.check_future(anchor, rows, module.timeline(rows))


def test_native_gaps_and_out_of_range_interpolation_reject():
    rows = states(); times = module.timeline(rows)
    with pytest.raises(ValueError): module.interpolate(rows, times, -1)
    with pytest.raises(ValueError): module.interpolate(rows, times, times[-1] + 1)
    rows[2]["frame"] += 1
    with pytest.raises(ValueError): module.timeline(rows)


def test_wrapped_yaw_interpolation():
    rows = states(); rows[0]["yaw"] = math.radians(179); rows[1]["yaw"] = math.radians(-179)
    value = module.interpolate(rows, module.timeline(rows), 25000000)
    assert math.isclose(abs(value["yaw"]), math.pi, abs_tol=1e-12)


def test_nearest_association_boundary_does_not_retime_input():
    times = [0, 50000000, 100000000]
    assert module.nearest_index(times, 100000000 - 74) == 2
    with pytest.raises(ValueError): module.nearest_index(times, 100000000 - 75)
    assert times == [0, 50000000, 100000000]


def test_native_witness_distinguishes_endpoint_and_interval_quantities():
    rows = states(); rows[128]["y"] = .1; rows[128]["vy"] = .5
    times = module.timeline(rows)
    finding = module.check_future(future(rows), rows, times)[0]
    value = module.native_witness(finding, rows, times)
    assert value["native_from_frame"] == 126 and value["native_to_frame"] == 128
    assert value["api_steering_zero_across_interval"] is True
    assert value["raw_body_velocity_endpoint_mps"] == [1., .5]
    assert not math.isclose(value["native_interval_average_xy_speed_mps"], value["raw_body_planar_endpoint_speeds_mps"][-1])
    assert value["endpoint_velocity_vs_interval_direction_delta_rad"] != 0


def test_direction_bound_becomes_uninformative_when_error_exceeds_distance():
    assert module.direction_bound(.01, .005) == math.pi
    assert module.direction_bound(0., .005) == 0


def test_publish_refuses_dataset_alias_or_existing_input(tmp_path, monkeypatch):
    root = tmp_path / "repo"; root.mkdir()
    data = tmp_path / "actual_data"; data.mkdir()
    (root / "datasets").symlink_to(data, target_is_directory=True)
    monkeypatch.setattr(module, "ROOT", root)
    for path in (data / "new", root / "datasets/new", root):
        with pytest.raises(ValueError): module.publish(path)


def test_reviewed_inputs_include_both_failed_geometry_trials_without_overrides():
    values = module.pins()
    assert len(values) == 12
    assert values[f"{module.DIAG}/summary.json"] == module.SUMMARY_SHA
    assert values[f"{module.DIAG}/future_anchor_audit.jsonl"] == module.FUTURES_SHA
    assert set(module.EXPECTED) == {"run_001", "run_002"}
    assert module.EXPECTED["run_001"] == (671, 145, 576, 60)
    assert module.EXPECTED["run_002"] == (666, 154, 695, 59)
