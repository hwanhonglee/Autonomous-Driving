"""HH_260906 - Verify the exact nine-case identification plan without CARLA or workspace mutations."""

from dataclasses import replace
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts/e2e"))
from carla_low_speed_response_matrix import identification_matrix, hold_control, matrix_contract


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
