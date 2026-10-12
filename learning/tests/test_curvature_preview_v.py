"""HH_260906 - Test arc consistency, preview braking and measured-state rejection independently."""
import numpy as np
import pytest
from scripts.e2e.probe_curvature_preview_v import smooth_route, speed_envelope, evaluate


def test_recomputed_arc_and_origin():
    xy, arc, _, _ = smooth_route([[1., 0.], [10., 0.], [15., -5.], [15., -15.]])
    assert xy[0] == pytest.approx([0., 0.])
    assert np.diff(arc) == pytest.approx(np.linalg.norm(np.diff(xy, axis=0), axis=1))
    assert np.all(np.diff(arc) > 0)


def test_preview_brakes_before_curve_without_resetting_initial_speed():
    arc = np.linspace(0., 50., 501)
    curvature = np.where((arc > 25.) & (arc < 35.), .2, 0.)
    speed = speed_envelope(arc, curvature, 6.)
    assert speed[0] == 6.
    assert speed[-1] == 0.
    assert speed[240] < speed[180]
    assert np.max(speed**2*curvature) <= 1.5+1e-8
    accel = np.diff(speed**2)/(2*np.diff(arc))
    assert accel.max() <= 1.5+1e-8
    assert accel.min() >= -2.-1e-8


def test_infeasible_initial_state_rejected():
    with pytest.raises(ValueError, match='exceeds feasible envelope'):
        speed_envelope(np.array([0., 1., 2.]), np.zeros(3), 8.)


def test_straight_is_not_turn_completion():
    result, _ = evaluate([[0., 0.], [30., 0.]], 0.)
    assert result['all_windows_pass']
    assert result['endpoint_speed_mps'] == 0.
    assert not result['geometric_turn_covered']
    assert not result['full_turn_reference_pass']


def test_right_angle_reference_covers_turn_and_stops():
    angle = np.linspace(0., np.pi/2, 80)
    points = [[0., 0.], [5., 0.]]
    points += np.column_stack([5.+10*np.sin(angle), -10+10*np.cos(angle)]).tolist()
    points += [[15., -15.], [15., -25.]]
    result, _ = evaluate(points, 0.)
    assert result['full_turn_reference_pass']
    assert result['endpoint_speed_mps'] == 0.
    assert result['windows_failed'] == 0
