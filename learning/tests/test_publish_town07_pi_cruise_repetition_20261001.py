"""HH_261001 - Reject dropped failed attempts or inflated feature claims in the repetition gallery."""

import copy

import pytest

from scripts.e2e import publish_town07_pi_cruise_repetition_20261001 as publication


def valid_summary():
    return dict(schema=publication.contract.SCHEMA + ".repetition_summary",
        status="THREE_RESEARCH_CRUISE_SCREENS_PASS", total_predeclared_attempts=3,
        passed_attempts=3, failed_attempts=0, failed_attempts_excluded=0,
        full_feature_approval=False, full_RTE_01_approved=False,
        generalization_approved=False, outcome_selected_retry=False,
        attempts=[dict(trial=row, status="PASS_LOW_SPEED_CRUISE_RESEARCH_SCREEN_ONLY")
            for row in publication.contract.trial_specs()])


def test_original_three_trial_summary_accepted():
    publication.verify_summary(valid_summary())


@pytest.mark.parametrize("key,value", [
    ("passed_attempts", 2), ("failed_attempts_excluded", 1),
    ("full_RTE_01_approved", True), ("generalization_approved", True),
    ("outcome_selected_retry", True), ("total_predeclared_attempts", 2),
])
def test_inflated_or_selected_summary_rejected(key, value):
    summary = copy.deepcopy(valid_summary())
    summary[key] = value
    with pytest.raises(ValueError):
        publication.verify_summary(summary)


def test_swapped_trial_order_rejected():
    summary = valid_summary()
    summary["attempts"].reverse()
    with pytest.raises(ValueError):
        publication.verify_summary(summary)


def test_original_failure_is_kept_and_published():
    summary = valid_summary()
    summary.update(status="REPETITION_SCREEN_FAILED_RETAINED", passed_attempts=2, failed_attempts=1)
    summary["attempts"][2]["status"] = "FAIL_RETAINED"
    publication.verify_summary(summary)
