"""HH_261001 - Preserve full captures and reject failed owners during candidate inventory."""

import numpy as np
import pytest
from scripts.e2e import inventory_map_unused_routes_20261001 as audit


def test_spacing_is_not_quality_selection():
    points = np.array([[0,0],[.1,0],[.3,0],[.3,0],[.6,0]])
    assert audit.historical_retention(points) == [0,2,4]
    assert len(points) == 5


def test_same_site_geometry():
    assert audit.nearest(np.array([[0.,0.],[3.,4.]]),np.array([[0.,0.]])).tolist() == [0.,5.]


def test_failed_owner_rejected():
    with pytest.raises(ValueError, match='Failed owner'):
        audit.capture_gate({'map':'Town01','quality':'Epic','capture_mode':'mapping-probe'},
                           {'exit_code':1}, {'town':'Town01'})


def test_protected_site_rejected():
    with pytest.raises(ValueError, match='Town01 only'):
        audit.capture_gate({'map':'Town03'}, {}, {'town':'Town03'})


def test_seven_class_mapping():
    assert audit.TAGS == {1:1,2:2,24:3,7:4,8:5,6:6}
    assert audit.LABELS[3] == 'lane_marking'
