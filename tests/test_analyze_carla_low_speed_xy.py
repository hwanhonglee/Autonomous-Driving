"""HH_260906 - Distinguish decoder geometry bounds from sensitivity hypotheses without changing failures."""

import copy
import math
from pathlib import Path

import pytest

from scripts.e2e import analyze_carla_low_speed_xy as audit


def pose(x=0., y=0., yaw=0., vx=.2, vy=0.):
    return dict(x=x, y=y, z=0., yaw=yaw, vx=vx, vy=vy)


def test_discrete_constant_curvature_at_bound_matches_heading_budget():
    points, x, y, heading = [], 0., 0., 0.
    for _ in range(64):
        heading += .2 * .02
        x += .02 * math.cos(heading); y += .02 * math.sin(heading)
        points.append(pose(x, y, heading))
    rows = audit.geometry_steps(pose(), points)
    assert all(not row["curvature_failed"] for row in rows)
    assert all(row["curvature_rad_per_m"] == pytest.approx(.2) for row in rows)


def test_existing_small_step_skip_can_flag_a_valid_decoder_but_budget_explanation_is_separate():
    # HH_260906 - This synthetic counterexample validates the diagnostic, not a change to the historical metric.
    points, x, y, heading = [], 0., 0., 0.
    for distance in [.00009] * 10 + [.0002]:
        heading += .2 * distance
        x += distance * math.cos(heading); y += distance * math.sin(heading)
        points.append(pose(x, y, heading))
    last = audit.geometry_steps(pose(), points)[-1]
    assert last["curvature_failed"]
    assert last["preceding_unassessed_distance_m"] == pytest.approx(.0009)
    assert last["skip_threshold_budget_can_explain_failure"]


def test_small_lateral_position_change_causes_large_curvature_at_low_speed():
    rows = audit.geometry_steps(pose(), [pose(.004, 0.), pose(.008, .0006)])
    second = rows[1]
    assert second["curvature_rad_per_m"] > 30
    assert second["xy_speed_mps"] < .05
    assert second["curvature_rad_per_m"] * second["xy_speed_mps"] ** 2 < 2.8
    assert second["body_yaw_change_over_xy_distance"] == 0.
    assert not second["skip_threshold_budget_can_explain_failure"]


def test_center_proxy_is_labelled_sensitivity_and_does_not_mutate_rear_points():
    ego = pose(-200., -70., math.pi / 2)
    points = [pose(-200.0002, -69.996, math.pi / 2 + .00015)]
    before = copy.deepcopy((ego, points))
    row = audit.geometry_steps(ego, points)[0]
    assert row["planar_yaw_reference_offset_delta_xy_m"][0] == pytest.approx(-1.425 * .00015)
    assert (ego, points) == before
    assert "zero_pitch_actor_center_proxy_curvature" in row


def test_float32_ulp_uses_coordinate_scale_not_double_precision():
    assert audit.float32_spacing(200.) == pytest.approx(2 ** -16)
    assert audit.float32_spacing(-200.) == audit.float32_spacing(200.)
    assert audit.float32_spacing(1.) == pytest.approx(2 ** -23)


def test_endpoint_velocity_is_not_silently_replaced_with_interval_xy_speed():
    row = audit.geometry_steps(pose(vx=.07), [pose(.004, 0., vx=.00006)])[0]
    assert row["xy_speed_mps"] == .04
    assert row["instantaneous_speed_label_mps"] == .00006
    assert row["end_speed_vs_interval_xy_speed_error_mps"] == pytest.approx(-.03994)


def test_render_writes_actual_chart_data_with_expected_resolution(tmp_path):
    Image = pytest.importorskip("PIL.Image")
    report = {}
    witnesses = []
    traces = {}
    for name in ("run_001", "run_002"):
        witnesses.append({"trial": name, "xy_speed_mps": .04, "curvature_rad_per_m": 1., "distance_m": .004, "heading_change_rad": .004})
        traces[name] = [dict(time_from_goal_stop_s=float(i), rear_lateral_displacement_mm=.2 * i,
            zero_pitch_center_proxy_lateral_displacement_mm=.1 * i, yaw_change_from_reference_deg=.01 * i,
            reported_yaw_rate_radps=.001, pose_finite_difference_yaw_rate_radps=.002) for i in range(-3, 7)]
    audit.render(report, witnesses, traces, tmp_path)
    paths = sorted(tmp_path.glob("*.png"))
    assert len(paths) == 2
    for path in paths:
        with Image.open(path) as image:
            assert image.width == 1920


def test_requires_new_output_and_pinned_input_before_raw_access(tmp_path):
    source = tmp_path / "diagnostic"; source.mkdir()
    (source / "summary.json").write_text("{}\n")
    with pytest.raises(audit.raw.scalar.EvidenceError, match="summary SHA"):
        audit.analyze(source, tmp_path / "raw", tmp_path / "new", "0" * 64)
    with pytest.raises(audit.raw.scalar.EvidenceError, match="fresh output"):
        audit.analyze(source, tmp_path / "raw", source / "inside", "0" * 64)
