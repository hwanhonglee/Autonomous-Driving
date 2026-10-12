#!/usr/bin/env python3
"""HH_261001 - Extend frozen-plan QA to the explicitly approved 9 km/h comfortable_v2 revision."""

from contextlib import contextmanager
from pathlib import Path

from scripts.e2e import audit_town12_j15824_capture_20261001 as inherited


ORIGINAL_FROZEN_CONTRACT = inherited.frozen_contract


def revised_contract(plan, manifest, route, scout_receipt):
    result = ORIGINAL_FROZEN_CONTRACT(plan, manifest, route, scout_receipt)
    trial = plan["signal_expert_trial"]
    if trial["target_speed_kmh"] == 20:
        return result
    args = inherited.capture.collector.parse_args(trial["collector_arguments"])
    actual = manifest["capture_contract"]
    checks = dict(result["checks"])
    checks.pop("twenty_kmh")
    checks["target_speed_as_planned"] = trial["target_speed_kmh"] == args.target_speed_kmh == actual["target_speed_kmh"] == 9
    revision = trial.get("combined_protocol_revision", {})
    checks["approved_combined_revision"] = (revision.get("variant_id") == "town12_j15824_expert9_comfortable_v2_180s_v1"
        and revision.get("single_factor_causal_ab") is False
        and args.max_duration_sec == actual["maximum_total_duration_sec"] == 180
        and args.goal_stop_profile == actual["goal_stop_profile"]["profile_id"] == "comfortable_v2")
    checks["exact_revised_normal_stop_parameters"] = all(actual["goal_stop_profile"].get(key) == expected for key, expected in (
        ("desired_deceleration_mps2", 1.0), ("normal_brake_cap", .15), ("normal_throttle_cap", .20)))
    return {"status": "PASS" if all(checks.values()) else "FAIL", "checks": checks,
        "failed_checks": [name for name, passed in checks.items() if not passed],
        "versioned_plan_qa_extension_sha256": inherited.sha(Path(__file__)),
        "scope": "Only recognize approved9kmh/v2/180s plan; geometric, motion, timing, signal and admission gates are unchanged."}


@contextmanager
def configured_auditor():
    previous = inherited.frozen_contract
    try:
        inherited.frozen_contract = revised_contract
        yield
    finally:
        inherited.frozen_contract = previous


def main(argv=None):
    with configured_auditor():
        return inherited.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
