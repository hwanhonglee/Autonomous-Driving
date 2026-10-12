"""HH_260906 - Check exact continuous near-arc calculations independently of CARLA and frame sampling."""
import math
import random

import pytest

from scripts.e2e import analyze_carla_dev_route_overlap as analysis


@pytest.mark.parametrize("a,b,c,d,radius,expected", [
    ((0, 0), (20, 0), (10, -10), (10, 10), 5, [[.25, .75]]),
    ((0, 0), (20, 0), (0, 3), (20, 3), 5, [[0, 1]]),
    ((0, 0), (20, 0), (0, 6), (20, 6), 5, []),
    ((0, 0), (20, 0), (10, 0), (10, 0), 5, [[.25, .75]]),
    ((0, 0), (20, 0), (10, 5), (10, 5), 5, [[.5, .5]]),
    ((0, 0), (20, 0), (10, 3), (10, 3), 5, [[.3, .7]]),
    ((0, 0), (0, 0), (0, 0), (10, 0), 5, [[0, 1]]),
    ((0, 0), (0, 0), (10, 0), (20, 0), 5, []),
])
def test_exact_capsule_intersections(a, b, c, d, radius, expected):
    intervals = analysis.capsule_intervals(a, b, c, d, radius)
    assert len(intervals) == len(expected)
    for actual, target in zip(intervals, expected):
        assert actual == pytest.approx(target)


def test_dense_points_agree_with_independent_point_segment_distance():
    rng = random.Random(20260916)
    for _ in range(200):
        a, b, c, d = [tuple(rng.uniform(-200, 200) for _ in range(2)) for _ in range(4)]
        radius = rng.uniform(.1, 20)
        intervals = analysis.capsule_intervals(a, b, c, d, radius)
        for index in range(101):
            t = index/100
            p = tuple(a[i]+t*(b[i]-a[i]) for i in range(2))
            distance = analysis.preflight.point_segment_distance(p, c, d)
            if abs(distance-radius) < 1e-7:
                continue
            assert any(lo <= t <= hi for lo, hi in intervals) == (distance < radius)


def test_interval_union_does_not_double_count_overlapping_reference_segments():
    assert analysis.merge_intervals([[3, 8], [0, 4], [8, 10], [20, 21]]) == [[0, 10], [20, 21]]
    result = analysis.summarize_region([[0, 10], [20, 21]], 5, 20.5)
    assert result["proximity_length_m"] == 5.5
    assert result["proximity_intervals_m"] == [[5, 10], [20, 20.5]]


def route(points, road=1, lane=1):
    return dict(route=[dict(x=x, y=y, road_id=road, section_id=0, lane_id=lane) for x, y in points])


def test_same_road_opposite_lane_classification_and_scored_window():
    result = analysis.arc_analysis(route([(0, 0), (210, 0)], lane=-1),
                                   route([(180, 3.5), (100, 3.5)], lane=1))
    expansion = math.sqrt(25-3.5**2)
    assert result["full_route"]["proximity_intervals_m"][0] == pytest.approx([100-expansion, 180+expansion])
    assert result["selected_window"]["proximity_length_m"] == pytest.approx(65+expansion)
    assert result["nominal_future_support"]["proximity_length_m"] == pytest.approx(15+expansion)
    assert result["proximity_segment_pair_counts"] == {"same_road_other_lane": 1}
    assert result["proximity_heading_difference_deg_range"] == pytest.approx([180, 180])
    assert result["minimum_polyline_distance_m"] == 3.5
    assert result["shared_road_sections"] == [(1, 0)]


def test_tessellation_does_not_change_union_length():
    prior = route([(180, 3.5), (100, 3.5)])
    coarse = analysis.arc_analysis(route([(0, 0), (210, 0)]), prior)
    dense = analysis.arc_analysis(route([(x, 0) for x in range(211)]), prior)
    assert coarse["full_route"]["proximity_length_m"] == pytest.approx(dense["full_route"]["proximity_length_m"])
    assert coarse["selected_window"]["proximity_length_m"] == pytest.approx(dense["selected_window"]["proximity_length_m"])


def test_short_route_does_not_invent_selected_or_future_context():
    result = analysis.arc_analysis(route([(0, 0), (10, 0)]), route([(0, 0), (10, 0)]))
    assert result["selected_window"]["region_length_m"] == 0
    assert result["nominal_future_support"] is None


@pytest.mark.parametrize("radius", [-1, float("nan"), float("inf"), True])
def test_invalid_radius_rejected(radius):
    with pytest.raises(ValueError):
        analysis.capsule_intervals((0, 0), (1, 0), (0, 0), (1, 1), radius)


def test_crossings_visible_even_with_no_near_vertices():
    result = analysis.arc_analysis(route([(0, 0), (210, 0)]), route([(100, -100), (100, 100)], road=2))
    assert result["full_route"]["proximity_length_m"] == 10
    assert result["selected_window"]["proximity_length_m"] == 10
    assert result["minimum_polyline_distance_m"] == 0
    assert result["shared_road_sections"] == []


def test_candidate_handedness_is_checked():
    doc = dict(town="Town03", coordinate_reference="base_link")
    for prefix in ("start", "goal"):
        doc[prefix+"_carla_transform"] = dict(x=1., y=2.)
        doc[prefix+"_ros_pose"] = dict(x=1., y=-2.)
    analysis.check_candidate_frame(doc)
    doc["goal_ros_pose"]["y"] = 2.
    with pytest.raises(ValueError, match="endpoint frame"):
        analysis.check_candidate_frame(doc)


def test_existing_output_refused_without_input_reads(tmp_path):
    with pytest.raises(ValueError, match="fresh disjoint"):
        analysis.main(["--catalog-maps-root", str(tmp_path/"catalog"),
                       "--baseline-preflight", str(tmp_path/"baseline.json"), "--output-dir", str(tmp_path)])
