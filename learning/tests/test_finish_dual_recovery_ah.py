"""HH_260906 - Ensure final comparison shows both turn and straight metrics."""
from scripts.e2e.finish_dual_recovery_ah import comparison_table


def test_comparison_keeps_regression_visible():
    report=dict(evaluation=dict(subgroups={
        'town03_right_turn_dev':dict(metrics=dict(selected_ade_m=2.1)),
        'town03_straight_dev':dict(metrics=dict(selected_ade_m=.8))}),
        versus_original_k1=dict(pass_count=6),strict_gate=dict(metric_pass_count=4))
    text=comparison_table({'AH':report})
    assert '| AH | 2.1000 | 0.8000 | 6/9 | 4/9 |' in text
