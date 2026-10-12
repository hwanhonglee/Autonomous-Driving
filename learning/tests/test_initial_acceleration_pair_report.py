"""HH_261001 - Test full raw-run analysis separately from training and simulator dependencies."""

from pathlib import Path

from scripts.e2e.report_initial_acceleration_pair_20261001 import analyze_trial, finite_percentiles, pair_checks


def test_clean_parent_replay_stall_is_preserved_and_wrong_identity_is_not_accepted():
    root = Path(__file__).resolve().parents[2]
    run = root / "storage/experiments/validation/2026-10-01/ah_town07_straight_research_v2"
    sha = "4841d4076fb49145643ab2895d87befcae9021ad02f08b8307e75f77be137d91"
    report = analyze_trial(run, sha)[-1]
    assert report["delayed_acceleration_terminal_stall_detected"]
    assert report["acknowledged_learned_commands"] == 366
    assert report["next_state_ack_aligned_learned_commands"] == 365
    assert report["unique_raw_outputs_with_emergency_reason"] == 0
    assert report["trailing_stationary_seconds"] == 9.25
    assert all(pair_checks(report, report).values())
    assert not analyze_trial(run, "0" * 64)[-1]["all_command_checkpoint_identities_match"]


def test_empty_and_nonfinite_latency_is_not_a_pass():
    assert finite_percentiles([]) is None
    assert finite_percentiles([float("nan"), float("inf")]) is None
