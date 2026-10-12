"""HH_260906 - Verify ROS-to-CARLA position transforms before lane queries."""
import math
import pytest
from scripts.e2e.audit_offline_map_x import map_points


def test_zero_yaw_reflects_y():
    ego=dict(position_m=[10.,20.,0.],orientation_xyzw=[0.,0.,0.,1.])
    assert map_points([[2.,3.]],ego)[0].tolist()==pytest.approx([12.,-23.])


def test_quarter_turn():
    ego=dict(position_m=[10.,20.,0.],orientation_xyzw=[0.,0.,math.sqrt(.5),math.sqrt(.5)])
    assert map_points([[2.,0.]],ego)[0].tolist()==pytest.approx([10.,-22.])
