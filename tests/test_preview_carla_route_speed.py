"""HH_260906 - Test offline geometry constraints without simulator access or recorded-data modifications."""

from dataclasses import replace
import copy
import math

import pytest

from scripts.e2e import preview_carla_route_speed as preview


def route(points, scale=1.):
    result, distance = [], 0.
    for index, (x, y) in enumerate(points):
        if index:
            distance += math.dist(points[index - 1], (x, y)) * scale
        result.append(dict(x=x, y=y, distance_m=distance))
    return {"route": result}


def straight(length=200):
    return route([(float(i), 0.) for i in range(length + 1)])


def test_straight_route_has_zero_curvature_preserves_default_nonzero_coast_target():
    source = straight()
    original = copy.deepcopy(source)
    result = preview.preview(source, 1.)
    assert result["maximum_abs_curvature_per_m"] == 0
    assert result["minimum_curvature_cap_mps"] == 8
    assert result["samples"][-1]["backward_speed_cap_mps"] == preview.PreviewConfig().approach_reference_speed_mps
    assert result["policy_implemented"] is False and result["training_data_approved"] is False
    assert source == original


def test_signed_circle_curvature_is_exact_for_both_turn_directions():
    radius = 10.
    points = [dict(x=radius * math.cos(a), y=radius * math.sin(a)) for a in (0., .1, .2)]
    assert preview.signed_curvature(*points) == pytest.approx(.1)
    assert preview.signed_curvature(*reversed(points)) == pytest.approx(-.1)


def test_resampling_retains_short_final_segment_and_endpoint():
    geometry, _ = preview.route_geometry(straight(11))
    rows = preview.resample(geometry, 2.)
    assert [r["planar_s_m"] for r in rows] == [0, 2, 4, 6, 8, 10, 11]
    assert rows[-1]["x"] == 11


def test_catalog_and_planar_arc_are_distinct_and_mappable():
    geometry, _ = preview.route_geometry(route([(0., 0.), (10., 0.), (20., 0.)], scale=1.02))
    assert geometry[-1]["planar_s_m"] == 20 and geometry[-1]["catalog_s_m"] == pytest.approx(20.4)
    point = preview.interpolate(geometry, 10.2, field="catalog_s_m")
    assert point["planar_s_m"] == pytest.approx(10.)


def test_backward_cap_propagates_future_curve_before_entry():
    result = preview.backward_envelope([0., 10., 20.], [8., 8., 4.], .6)
    assert result == pytest.approx([math.sqrt(40), math.sqrt(28), 4.])
    for a, b in zip(result, result[1:]):
        assert a*a <= b*b + 12 + 1e-12


def test_preview_includes_unlabelled_bends_and_obeys_lateral_backward_and_launch_caps():
    points = [(float(i), 0.) for i in range(50)]
    points += [(49 + 15 * math.sin(i / 30), 15 * (1 - math.cos(i / 30))) for i in range(1, 48)]
    last = points[-1]
    points += [(last[0], last[1] + i) for i in range(1, 100)]
    result = preview.preview(route(points), 1.)
    assert result["minimum_curvature_cap_mps"] < 8
    rows = result["samples"]
    for r in rows:
        assert r["backward_speed_cap_mps"] <= r["local_speed_cap_mps"] + 1e-12
        assert r["curvature_cap_mps"]**2 * r["forward_max_abs_curvature_per_m"] <= 2 + 1e-12
    for a, b in zip(rows, rows[1:]):
        ds = b["planar_s_m"] - a["planar_s_m"]
        assert a["backward_speed_cap_mps"]**2 <= b["backward_speed_cap_mps"]**2 + 1.2 * ds + 1e-10
        assert b["planned_from_rest_speed_mps"]**2 <= a["planned_from_rest_speed_mps"]**2 + 2 * ds + 1e-10


def test_single_low_cap_breaks_continuous_cruise_instead_of_summing_separated_spans():
    rows = [dict(planar_s_m=i, planned_from_rest_speed_mps=v) for i, v in enumerate([8., 8., 7., 8., 8.])]
    assert preview.qualifying_spans(rows, 7.8) == [dict(start_planar_s_m=0, end_planar_s_m=1, length_m=1),
                                                 dict(start_planar_s_m=3, end_planar_s_m=4, length_m=1)]


def test_identical_duplicate_is_disclosed_not_silently_changed_in_source():
    source = route([(0., 0.), (1., 0.), (1., 0.), (2., 0.)])
    rows, count = preview.route_geometry(source)
    assert count == 1 and len(rows) == 3 and len(source["route"]) == 4


@pytest.mark.parametrize("value", [True, False, float("nan"), float("inf"), "1", None])
def test_nonfinite_or_bool_geometry_refused(value):
    source = straight()
    source["route"][5]["x"] = value
    with pytest.raises(ValueError):
        preview.preview(source, 1.)


@pytest.mark.parametrize("spacing", [0, -1, True, float("nan"), 200])
def test_invalid_resampling_spacing_refused(spacing):
    with pytest.raises(ValueError):
        preview.preview(straight(), spacing)


@pytest.mark.parametrize("change", ["decreasing", "moving_zero_arc", "vertical_only", "nonzero_start"])
def test_invalid_arc_mapping_never_flattened(change):
    source = straight()
    if change == "decreasing": source["route"][10]["distance_m"] = 1
    elif change == "moving_zero_arc": source["route"][10]["distance_m"] = 9
    elif change == "vertical_only": source["route"][10]["x"] = 9
    else: source["route"][0]["distance_m"] = 1
    with pytest.raises(ValueError):
        preview.route_geometry(source)


def test_reversal_is_not_mislabeled_as_zero_curvature():
    with pytest.raises(ValueError):
        preview.signed_curvature(dict(x=0., y=0.), dict(x=2., y=0.), dict(x=1., y=0.))


@pytest.mark.parametrize("config", [replace(preview.PreviewConfig(), nominal_speed_mps=9.),
    replace(preview.PreviewConfig(), planned_lateral_acceleration_mps2=2.9),
    replace(preview.PreviewConfig(), launch_acceleration_plan_mps2=True)])
def test_config_changes_need_a_new_reviewed_proposal(config):
    with pytest.raises(ValueError):
        preview.preview(straight(), 1., config)


def test_existing_output_is_refused_before_plan_read(tmp_path):
    output = tmp_path / "existing.json"
    output.write_text("preserved")
    with pytest.raises(ValueError, match="create-only"):
        preview.main(["--plan", str(tmp_path / "missing"), "--output", str(output)])
    assert output.read_text() == "preserved"
