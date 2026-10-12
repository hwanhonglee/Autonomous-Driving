"""HH_260906 - Preserve the measured initial speed when testing slower future motion."""
import pytest
from scripts.e2e.check_arc_speed_retiming_u import bounded_speeds


def test_deceleration_does_not_reset_initial_speed():
    speeds=bounded_speeds([8.]*64,6.,.25)
    assert speeds[0]==pytest.approx(5.8)
    assert speeds[-1]==pytest.approx(2.)
    assert all(abs(b-a)<=.2000001 for a,b in zip([6.]+speeds,speeds))


def test_invalid_factor_rejected():
    with pytest.raises(ValueError):bounded_speeds([1.],0.,0.)
