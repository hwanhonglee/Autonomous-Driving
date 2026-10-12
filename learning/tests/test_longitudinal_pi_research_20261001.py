"""HH_261001 - Verify bounded PI state, antiwindup and no implicit sustaining-throttle floor."""

from dataclasses import replace

import pytest

from portable_e2e.longitudinal_pi_research_20261001 import LongitudinalPIConfig, LongitudinalPIResearch


def test_one_update_matches_declared_discrete_pi_equation():
    pi = LongitudinalPIResearch()
    request, audit = pi.update(1.5, 1.)
    assert pi.integral_error == pytest.approx(.05)
    assert request == pytest.approx(dict(throttle=.36, brake=0.))
    assert audit["timestep_s"] == .1 and audit["feedforward"] == 0.
    assert audit["throttle_floor_added"] is False


def test_zero_error_has_no_initial_throttle_floor():
    request, _ = LongitudinalPIResearch().update(1.5, 1.5)
    assert request == dict(throttle=0., brake=0.)


def test_past_positive_error_provides_integral_not_constant_feedforward():
    pi = LongitudinalPIResearch()
    pi.update(1.5, 1.)
    request, audit = pi.update(1.5, 1.5)
    assert request["throttle"] == pytest.approx(.01)
    assert request["brake"] == 0. and audit["feedforward"] == 0.
    pi.reset()
    assert pi.update(1.5, 1.5)[0]["throttle"] == 0.


@pytest.mark.parametrize("desired", [0., .01, .05])
def test_stop_returns_no_substitute_and_clears_integral(desired):
    pi = LongitudinalPIResearch()
    pi.update(1.5, 1.)
    request, audit = pi.update(desired, .5)
    assert request is None and pi.integral_error == 0.
    assert audit["stop_branch_preserved"] is True
    assert audit["integral_reset_reason"] == "model_stop_target"


def test_positive_saturation_does_not_wind_up_integral():
    pi = LongitudinalPIResearch()
    for _ in range(100):
        request, audit = pi.update(2.5, 0.)
        assert request == dict(throttle=.5, brake=0.)
        assert audit["integral_held_for_saturation"] is True
    assert pi.integral_error == 0.
    assert pi.update(1., 1.)[0]["throttle"] == 0.


def test_negative_saturation_does_not_wind_up_integral():
    pi = LongitudinalPIResearch()
    for _ in range(100):
        request, audit = pi.update(.5, 3.)
        assert request == dict(throttle=0., brake=1.)
        assert audit["integral_held_for_saturation"] is True
    assert pi.integral_error == 0.


def test_integral_bound_is_separate_from_actuator_saturation():
    pi = LongitudinalPIResearch()
    for _ in range(200):
        request, audit = pi.update(1.1, 1.)
    assert pi.integral_error == 1.
    assert audit["integral_bound_applied"] is True
    assert request["throttle"] == pytest.approx(.27)


def test_opposite_error_can_unwind_a_preexisting_integral():
    pi = LongitudinalPIResearch()
    for _ in range(100):
        pi.update(1.1, 1.)
    before = pi.integral_error
    request, _ = pi.update(1., 1.1)
    assert pi.integral_error < before
    assert request["throttle"] > 0. and request["brake"] == 0.


@pytest.mark.parametrize("desired,current", [(True, 0.), (float("nan"), 1.),
    (float("inf"), 1.), (-.001, 0.), (2.50001, 0.), (.5, False),
    (.5, float("nan")), (.5, float("inf")), (.5, -.10001)])
def test_invalid_observation_does_not_update_state(desired, current):
    pi = LongitudinalPIResearch()
    pi.update(1.5, 1.)
    before = pi.integral_error
    with pytest.raises(ValueError):
        pi.update(desired, current)
    assert pi.integral_error == before


@pytest.mark.parametrize("field,value", [("integral_gain", .20001),
    ("integral_gain", 0.), ("timestep_s", .05), ("maximum_throttle", .50001),
    ("maximum_integral_error", 1.00001), ("maximum_desired_speed_mps", 2.50001),
    ("stop_speed_threshold_mps", .1), ("proportional_gain", True),
    ("maximum_brake", float("nan"))])
def test_invalid_config_is_rejected(field, value):
    with pytest.raises(ValueError):
        LongitudinalPIResearch(replace(LongitudinalPIConfig(), **{field: value}))


def test_instances_do_not_share_integral_state():
    first, second = LongitudinalPIResearch(), LongitudinalPIResearch()
    first.update(1.5, 1.)
    assert second.integral_error == 0.


def test_negative_allowed_vx_is_not_treated_as_additional_positive_error():
    request, audit = LongitudinalPIResearch().update(.2, -.05)
    assert audit["error_mps"] == .2
    assert request["throttle"] == pytest.approx(.144)
