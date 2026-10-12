"""HH_260906 - Verify measured bbox offsets and reject unsupported rotations."""
import pytest
from scripts.e2e.audit_prius_boundary_z import perimeter


def measurement():
    return dict(bounding_box=dict(extent=dict(x=2.,y=1.),location=dict(x=.1,y=.2),
        rotation=dict(roll=0.,pitch=0.,yaw=0.)),historical_collector_wheelbase_m=2.8)


def test_offset_and_reflection():
    points=perimeter(measurement())
    assert points[:,0].min()==pytest.approx(-.5)
    assert points[:,0].max()==pytest.approx(3.5)
    assert points[:,1].min()==pytest.approx(-1.2)
    assert points[:,1].max()==pytest.approx(.8)


def test_rotation_is_not_silently_ignored():
    m=measurement();m['bounding_box']['rotation']['yaw']=1.
    with pytest.raises(ValueError):perimeter(m)
