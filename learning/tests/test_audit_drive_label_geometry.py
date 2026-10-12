"""HH_260906 - Keep the retrospective speed/yaw geometry arithmetic explicit."""

from __future__ import annotations

import math
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[2] / "tools/e2e/audit_drive_label_geometry.py"
SPEC = spec_from_file_location("audit_drive_label_geometry", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
geometry = module_from_spec(SPEC)
SPEC.loader.exec_module(geometry)


def test_straight_integration_matches_constant_speed_displacement():
    point = (0.0, 0.0)
    for _ in range(64):
        point = geometry.integrated_step(point, 2.0, 0.0, 2.0, 0.0)
    assert math.isclose(point[0], 12.8, abs_tol=1e-12)
    assert math.isclose(point[1], 0.0, abs_tol=1e-12)


def test_curvature_uses_frozen_initial_heading_and_skips_tiny_steps():
    k, heading, distance = geometry.path_curvature((0.0, 0.0), (0.1, 0.0), 0.0)
    assert k == 0.0 and heading == 0.0 and math.isclose(distance, 0.1)
    k, heading, _ = geometry.path_curvature((0.1, 0.0), (0.1, 0.0001), heading)
    assert k is None and heading == 0.0
    k, heading, _ = geometry.path_curvature((0.1, 0.0), (0.2, 0.1), heading)
    assert math.isclose(k, math.pi / (4 * math.sqrt(0.02)), rel_tol=1e-12)
    assert math.isclose(heading, math.pi / 4, rel_tol=1e-12)


def test_native_interpolation_wraps_yaw_and_requires_contiguous_bracket():
    earlier = {"frame": 5, "x": 0.0, "y": 0.0, "vx": 1.0, "vy": 0.0,
               "yaw": math.pi - 0.1}
    later = {"frame": 6, "x": 0.1, "y": 0.0, "vx": 1.0, "vy": 0.0,
             "yaw": -math.pi + 0.1}
    timeline = [(0, earlier), (50_000_000, later)]
    result = geometry.interpolate(timeline, [0, 50_000_000], 25_000_000)
    assert math.isclose(result["x"], 0.05, abs_tol=1e-12)
    assert math.isclose(abs(result["yaw"]), math.pi, abs_tol=1e-12)
    later["frame"] = 7
    try:
        geometry.interpolate(timeline, [0, 50_000_000], 25_000_000)
    except ValueError as error:
        assert "incomplete native bracket" in str(error)
    else:
        raise AssertionError("a missing native frame was accepted")


def test_prespecified_tick_windows_do_not_hide_other_ticks():
    assert geometry.region(295) == "turn"
    assert geometry.region(556) == "near_goal"
    assert geometry.region(200) == "outside_windows"
