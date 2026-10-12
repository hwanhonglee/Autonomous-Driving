"""HH_261001 - Prove disclosed target headroom, single PI integration and preserved stop/reset authority."""

from dataclasses import replace

import pytest

from portable_e2e.longitudinal_pi_research_20261001 import LongitudinalPIConfig, LongitudinalPIResearch
from portable_e2e.longitudinal_pi_headroom_research_20261001 import (
    CONTROLLER_TARGET_CAP_MPS, LongitudinalPIHeadroomResearch)


@pytest.mark.parametrize("desired", [0., .01, .05, .051, .5, 2., 2.4])
def test_targets_below_cap_reproduce_original_PI_request_and_state(desired):
    original, governed = LongitudinalPIResearch(), LongitudinalPIHeadroomResearch()
    original.integral_error = governed.integral_error = .2
    first, before = original.update(desired, 1.)
    second, after = governed.update(desired, 1.)
    assert first == second and original.integral_error == governed.integral_error
    for key in before: assert after[key] == before[key], key
    assert after["original_model_target_mps"] == desired
    assert after["governed_controller_target_mps"] == desired
    assert after["controller_target_governed"] is False


def test_model_cap_target_is_explicitly_governed_before_exactly_one_integration():
    controller = LongitudinalPIHeadroomResearch()
    request, audit = controller.update(2.5, 2.)
    assert CONTROLLER_TARGET_CAP_MPS == 2.4
    assert controller.integral_error == pytest.approx(.04)
    assert request == pytest.approx(dict(throttle=.288, brake=0.))
    assert audit["original_model_target_mps"] == 2.5
    assert audit["governed_controller_target_mps"] == 2.4
    assert audit["observed_speed_mps"] == 2.
    assert audit["observed_speed_clipped"] is False
    assert audit["raw_NN_decoded_selected_trajectory_changed"] is False
    assert audit["original_9_kmh_target_tracking_claimed"] is False
    assert audit["model_only_performance_claimed"] is False


def test_previous_actual_failure_state_reduces_not_falsifies_sustaining_request():
    original, governed = LongitudinalPIResearch(), LongitudinalPIHeadroomResearch()
    original.integral_error = governed.integral_error = .699458367
    first, _ = original.update(2.499999761581421, 2.4957315921783447)
    second, audit = governed.update(2.499999761581421, 2.4957315921783447)
    assert second["throttle"] < first["throttle"]
    assert second["throttle"] == pytest.approx(.07096492703159177)
    assert audit["observed_speed_mps"] == 2.4957315921783447
    assert audit["error_mps"] < 0.
    # HH_261001 - This algebra test is not evidence that a physical trajectory will avoid overshoot.
    assert second["brake"] == 0.


@pytest.mark.parametrize("desired", [0., .01, .05])
def test_original_stop_resets_integral_without_replacing_the_stop_branch(desired):
    controller = LongitudinalPIHeadroomResearch()
    controller.integral_error = .9
    request, audit = controller.update(desired, 2.)
    assert request is None and controller.integral_error == 0.
    assert audit["unchanged_model_stop_branch_preserved"] is True
    assert audit["controller_target_governed"] is False


@pytest.mark.parametrize("desired,observed", [(2.500001, 1.), (True, 1.), (-.001, 1.),
    (float("nan"), 1.), (float("inf"), 1.), (2.5, True),
    (2.5, float("nan")), (2.5, float("inf"))])
def test_headroom_does_not_hide_invalid_original_target_or_observation(desired, observed):
    controller = LongitudinalPIHeadroomResearch()
    controller.integral_error = .3
    with pytest.raises(ValueError): controller.update(desired, observed)
    assert controller.integral_error == .3


def test_unchanged_antiwindup_gain_timestep_bounds_and_no_throttle_floor():
    controller = LongitudinalPIHeadroomResearch()
    for _ in range(30):
        request, audit = controller.update(2.5, 0.)
        assert request == dict(throttle=.5, brake=0.)
        assert audit["integral_held_for_saturation"] is True
    assert controller.integral_error == 0.
    request, audit = controller.update(2.5, 2.4)
    assert request == dict(throttle=0., brake=0.)
    assert audit["timestep_s"] == .1 and audit["feedforward"] == 0.
    assert audit["throttle_floor_added"] is False
    with pytest.raises(ValueError):
        LongitudinalPIHeadroomResearch(replace(LongitudinalPIConfig(), maximum_desired_speed_mps=2.4))


def test_controller_state_is_private_and_reset_clears_headroom_integral():
    first, second = LongitudinalPIHeadroomResearch(), LongitudinalPIHeadroomResearch()
    first.update(2.5, 2.)
    assert first.integral_error > 0. and second.integral_error == 0.
    first.reset()
    assert first.integral_error == 0.
