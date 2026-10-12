"""HH_261001 - Check native plot statistics and reject nonconsecutive or forged speeds."""

from copy import deepcopy

import pytest

from scripts.e2e.render_town07_native_pilot_results_20261001 import measured_summary


def samples():
    return [dict(frame=10+i, timestamp=.05*i, route_progress_m=progress,
                 speed_mps=1., world_velocity_carla=[1., 0., 0.], route_cte_m=.01,
                 collision_count=0, lane_invasion_count=0)
            for i, progress in enumerate([9.9, 10., 20., 30., 30.1])]


def test_entire_inclusive_window_and_closing_interval_are_retained():
    rows = samples()
    before = deepcopy(rows)
    result = measured_summary(rows)
    assert result["native_state_count"] == 5 and result["selected_state_count"] == 3
    assert result["selected_progress_first_last_m"] == [10., 30.]
    assert result["selected_span_seconds"] == pytest.approx(.1)
    assert result["selected_max_abs_scalar_rate_mps2"] == 0.
    assert result["training_data_approved"] is False and result["plot_is_actual_camera_footage"] is False
    assert rows == before


@pytest.mark.parametrize("field,value", [("speed_mps", 2.), ("frame", 100), ("timestamp", .2)])
def test_wrong_native_evidence_is_rejected(field, value):
    rows = samples()
    rows[2][field] = value
    with pytest.raises(ValueError):
        measured_summary(rows)


def test_closing_sample_and_bounded_row_budget_are_required():
    with pytest.raises(ValueError):
        measured_summary(samples()[:-1])
    with pytest.raises(ValueError):
        measured_summary([])
    with pytest.raises(ValueError):
        measured_summary(samples() * 261)


def test_entry_and_exit_rates_are_not_trimmed():
    rows = samples()
    rows[0].update(speed_mps=.8, world_velocity_carla=[.8, 0., 0.])
    rows[-1].update(speed_mps=1.3, world_velocity_carla=[1.3, 0., 0.])
    result = measured_summary(rows)
    assert result["selected_max_abs_scalar_rate_mps2"] == pytest.approx(6.)
