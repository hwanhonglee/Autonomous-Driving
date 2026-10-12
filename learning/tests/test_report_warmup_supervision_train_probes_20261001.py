"""HH_261001 - Keep start, protected stops, and moving transitions separate in read-only reports."""

from scripts.e2e.report_warmup_supervision_train_probes_20261001 import consecutive_switches, groups


def row(index, sequence, speed, selected, phase=3, route="right"):
    return dict(index=index, episode_id="same_episode", sequence_index=sequence, current_speed_mps=speed,
        selected=selected, oracle=5, original_phase_id=phase, route_group=route, acquisition_phase="driving",
        oracle_agreement=selected == 5, selected_composite_regret=0., first5_selected_speed_mae_mps=.1,
        selected_runtime_gate_pass=True, selected_delta_speed_at_0p3_mps=-.2,
        target_delta_speed_at_0p3_mps=.3, slows_while_target_accelerates=True)


def test_sparse_probe_gaps_are_not_reported_as_consecutive_switches():
    records = [row(1, 10, 2., 5), row(2, 12, 2.1, 1)]
    assert consecutive_switches(records) == []
    records.insert(1, row(3, 11, 2.05, 5))
    switches = consecutive_switches(records)
    assert len(switches) == 1 and switches[0]["from_sequence"] == 11


def test_start_stop_and_moving_band_counts_do_not_conflate():
    records = [row(0, 0, 0., 3, phase=1), row(1, 1, 0., 6, phase=0), row(2, 2, 2., 1)]
    result = groups(records)
    assert result["original_immediate_start18"]["count"] == 1
    assert result["protected_stop_hold200"]["count"] == 1
    assert result["moving_speed_1p5_to_2p5"]["count"] == 1
    assert result["right_speed_1p5_to_2p5"]["count"] == 1
