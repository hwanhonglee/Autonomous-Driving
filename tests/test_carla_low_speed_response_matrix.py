"""HH_260906 - Verify the exact nine-case identification plan without CARLA or workspace mutations."""

from dataclasses import replace
import math
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts/e2e"))
from carla_low_speed_response_matrix import (TwoStageCase, identification_matrix, hold_control,
    matrix_contract, progressive_identification_matrix, progressive_matrix_contract)


def test_exact_nine_case_priority_order_and_repeat_counts():
    cases = identification_matrix()
    assert len(cases) == len({case.case_id for case in cases}) == 9
    assert [case.kind for case in cases] == ["coast"] * 2 + ["throttle"] * 3 + ["throttle_ramp"] * 4
    assert [case.repeat for case in cases[:5]] == [1, 2, 1, 2, 3]
    assert [case.hold_seconds for case in cases] == [20.0] * 2 + [8.0] * 7
    assert [case.ramp_rate_per_second for case in cases[5:]] == [0.01, 0.025, 0.05, 0.10]


def test_coast_is_zero_pedals_and_constant_launch_is_exact_point_15():
    cases = identification_matrix()
    for case in cases[:2]:
        assert all(hold_control(case, index) == (0.0, 0.0) for index in range(400))
    for case in cases[2:5]:
        assert all(hold_control(case, index) == (0.15, 0.0) for index in range(160))


@pytest.mark.parametrize("case_index, rate, final", [(5, 0.01, 0.08), (6, 0.025, 0.20), (7, 0.05, 0.40), (8, 0.10, 0.40)])
def test_ramp_commands_are_tick_exact_bounded_monotonic_and_not_all_reach_cap(case_index, rate, final):
    case = identification_matrix()[case_index]
    commands = [hold_control(case, index)[0] for index in range(160)]
    assert commands[0] == pytest.approx(rate / 20)
    assert commands[-1] == pytest.approx(final)
    assert commands == sorted(commands)
    assert max(commands) <= 0.40
    assert all(after - before <= rate / 20 + 1e-12 for before, after in zip(commands, commands[1:]))
    assert all(hold_control(case, index)[1] == 0.0 for index in range(160))


@pytest.mark.parametrize("index", [-1, 160, 1.0, True])
def test_ramp_rejects_out_of_contract_tick_indices(index):
    with pytest.raises(ValueError):
        hold_control(identification_matrix()[-1], index)


def test_unknown_kind_or_invalid_ramp_is_not_silently_reinterpreted():
    case = identification_matrix()[-1]
    with pytest.raises(ValueError):
        hold_control(replace(case, kind="brake"), 0)
    with pytest.raises(ValueError):
        hold_control(replace(case, ramp_rate_per_second=0.0), 0)


def test_contract_explicitly_disclaims_training_and_physics_or_gear_changes():
    contract = matrix_contract()
    assert contract["matrix_id"] == "low_speed_v2"
    assert contract["ramp_policy"]["final_commanded_throttles"] == [0.08, 0.20, 0.40, 0.40]
    assert contract["ramp_policy"]["all_ramps_reach_cap"] is False
    for key in ("independent_route_claim", "manual_gear_changes", "physics_parameter_changes",
                "simultaneous_throttle_and_brake", "training_data", "automatic_quality_promotion"):
        assert contract[key] is False


def test_v3_six_case_matrix_is_fixed_and_does_not_mutate_v2():
    """HH_260906 - Keep the coast extension and measured launch matrix distinct from prior protocols."""
    old_cases, old_contract = identification_matrix(), matrix_contract()
    cases = progressive_identification_matrix()
    assert len(cases) == len({case.case_id for case in cases}) == 6
    assert [case.kind for case in cases] == ["coast"] * 2 + ["two_stage_ramp"] * 4
    assert [case.hold_seconds for case in cases] == [45.] * 2 + [12.] * 4
    assert [(case.handoff_speed_mps, case.ramp_rate_per_second) for case in cases[2:]] == [(.5, .05), (.5, .10), (1., .05), (1., .10)]
    assert all(isinstance(case, TwoStageCase) and case.launch_throttle == .15 and case.launch_maximum_seconds == 8. for case in cases[2:])
    assert identification_matrix() == old_cases and matrix_contract() == old_contract
    assert not hasattr(old_cases[0], "handoff_speed_mps")
    for case in cases[:2]:
        assert all(hold_control(case, index) == (0., 0.) for index in range(900))
        with pytest.raises(ValueError):
            hold_control(case, 900)


@pytest.mark.parametrize("index", [2, 3, 4, 5])
def test_two_stage_ramp_starts_after_point_15_and_caps_without_brake(index):
    """HH_260906 - The first ramp command belongs to the first post-handoff interval, not startup."""
    case = progressive_identification_matrix()[index]
    commands = [hold_control(case, tick) for tick in range(240)]
    assert commands[0] == pytest.approx((.15 + case.ramp_rate_per_second * .05, 0.))
    assert commands[-1] == (.4, 0.)
    assert all(brake == 0 and .15 < throttle <= .4 for throttle, brake in commands)
    assert [item[0] for item in commands] == sorted(item[0] for item in commands)
    with pytest.raises(ValueError):
        hold_control(case, 240)
    with pytest.raises(ValueError):
        hold_control(replace(case, ramp_rate_per_second=math.nan), 0)


def test_v3_contract_does_not_equate_deadline_completion_or_coast_to_driving_success():
    """HH_260906 - Declare all six cases, unchanged guards elsewhere, and observed-only stop dwell."""
    contract = progressive_matrix_contract()
    assert contract["matrix_id"] == "low_speed_v3" and len(contract["cases"]) == 6
    assert contract["coast_hold_ticks"] == 900 and contract["coast_hold_seconds"] == 45.
    assert contract["coast_prepare"]["throttle"] == .30
    assert contract["coast_prepare"]["maximum_seconds"] == 15.
    assert contract["coast_stop_observation"]["hold_stops_early"] is False
    assert contract["coast_stop_observation"]["brake_after_threshold"] is False
    policy = contract["two_stage_policy"]
    assert policy["launch_maximum_ticks"] == 160 and policy["maximum_post_handoff_ticks"] == 240
    assert policy["measured_speed_stop_mps"] == 30. / 3.6
    assert policy["time_limit_without_30kph_is_measurement_complete_not_cruise_success"] is True
    for key in ("independent_route_claim", "manual_gear_changes", "physics_parameter_changes",
                "simultaneous_throttle_and_brake", "training_data", "automatic_quality_promotion"):
        assert contract[key] is False
