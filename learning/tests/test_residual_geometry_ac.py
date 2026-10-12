"""HH_260906 - Distinguish polygon holes from excess outside the exterior boundary."""
import pytest
from shapely.geometry import box,Polygon
from scripts.e2e.summarize_residual_geometry_ac import characterize


def test_exterior_strip_is_not_a_hole():
    r=characterize(box(0,0,4,4),box(1,3,3,4.001))
    assert r['hole_area_m2']==0
    assert r['short_side_m']==pytest.approx(.001)


def test_actual_hole_detected():
    lane=Polygon(box(0,0,4,4).exterior.coords,[box(1,1,2,2).exterior.coords])
    assert characterize(lane,box(.5,.5,2.5,2.5))['hole_area_m2']==pytest.approx(1.)
