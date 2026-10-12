"""HH_261001 - Preserve boundary failures in diagnostic failed-episode windows."""

from scripts.e2e.audit_town07_failed_episode_windows_20261001 import exclusions, native_span_indices


def test_native_guard_and_future_right_bracket():
    assert native_span_indices([0, 50, 100, 150, 200], 2, 149) == (1, 3)
    assert native_span_indices([0, 50, 100, 150, 200], 0, 250) == (0, 4)


def test_bad_native_tick_not_hidden_by_good_discrete_future():
    row = {name: True for name in ('native_speed_bound', 'native_rate_bound', 'ack_bound',
        'body_bound', 'sweep_bound', 'native_clock_bound', 'snapshot_conversion_bound',
        'no_recorded_event', 'upstream_of_terminal', 'driving_phase')}
    future = {'discrete_bound_diagnostic': {'steps': [{'metrics': {'speed': [2.4, False]}}]}}
    assert exclusions(True, True, [row], [{'same_recorded_frame_and_timestamp': True}], future) == []
    bad = dict(row, native_rate_bound=False)
    assert exclusions(True, True, [row, bad], [], future) == ['native_rate_bound']
    assert exclusions(False, False, [dict(row, upstream_of_terminal=False)], [], future) == [
        'incomplete_ten_camera_history', 'incomplete_64_point_future', 'upstream_of_terminal']
    assert row['native_rate_bound'] is True
