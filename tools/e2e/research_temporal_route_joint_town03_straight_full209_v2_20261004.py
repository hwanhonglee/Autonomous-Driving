"""HH_261004 - Prepare an immutable fresh full-route timing/diagnostic experiment, never regrade frozen V1."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from pathlib import Path

from . import research_temporal_route_joint_town03_straight_full209_v1_20261004 as previous
from . import collect_temporal_route_joint_town03_straight_full209_v2_20261004 as live
from . import audit_town03_full209_diagnostic_v2_20261004 as diagnostic


base, ROOT = previous.base, previous.ROOT
require, digest = previous.require, previous.digest
MODULE = "scripts.e2e.research_temporal_route_joint_town03_straight_full209_v2_20261004"
SCHEMA = live.OWNER_SCHEMA
PARENT_SHA256 = "46841432d4dac2b095677dda480e758c12246c6c0f747f38db6a4323e934f22c"
_fixed, _sources = previous.fixed_contract, previous.runtime_sources
_active = False


def paths(run_id):
    require(run_id == "v2", "fresh full209 V2 requires exact v2 identity")
    output = ROOT / "storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_full209_v2"
    return output, output.with_name(output.name + "_plan.json")


def capture_argv(run_id, longitudinal_pi, declaration_sha=base.PLAN_TOKEN):
    # HH_261004 - Preserve every original capture argument, including ports, exact110s ceiling and original2.5m goal.
    require(_active and run_id == "v2", "fresh capture requires its exclusive V2 owner scope")
    result = previous._original_capture(run_id, longitudinal_pi, declaration_sha)
    index = result.index("--max-duration-sec") + 1
    require(result[index] == "45", "original fullroute duration seam changed")
    result[index] = "110"
    return result


def fixed_contract(run_id, longitudinal_pi):
    """HH_261004 - Make the fresh identity and timing-only intervention explicit before any actual capture."""
    result = _fixed(run_id, longitudinal_pi)
    result.update(schema=SCHEMA, experiment="Fresh known-DEV full209 exact-math terminal timing and authenticated diagnostic V2",
        frozen_V1_preserved=True, capture_raw_ABI_schema=live.SCHEMA,
        terminal_scalar_timing_research=dict(
            source="learning/portable_e2e/temporal_navigation_terminal_policy_v3_timing_20261004.py",
            original_bridge_AST_inverse_required=True, every_original_check_and_operation_unchanged=True,
            scalar_stage_clocks_inside_original_policy_deadline=True,
            receipt_attachment_outside_deadline_like_inherited_receipt=True,
            no_GC_disable_or_collect_or_system_change=True, actual_latency_gain_not_yet_proven=True),
        authenticated_diagnostic_revision=dict(
            source="tools/e2e/audit_town03_full209_diagnostic_v2_20261004.py",
            unique_authentic_final_goal_ACK_required=True, original_manual1_brake_ABI_required=True,
            exact_VERIFIED_witness_D_and_arc_comparison_required=True,
            no_threshold_tolerance_increase=True, no_raw_receipt_relabel=True, original_FAIL_not_overwritten=True),
        official_completed_main_domains=0, official_completed_subfunctions=0,
        independent_validation=False, full_RTE01_approved=False, model_promoted=False)
    result["prospective_visual_publication"]["gallery"] = "docs/assets/validation/2026-10-04/portable_e2e_learning_cycle_v1/12_town03_full_route209_v2"
    require(result["policy_deadline_seconds"] == .1 and result["model_speed_cap_mps"] == 2.5
        and result["body_clearance_screen"]["lane_inset_m"] == .1
        and result["worker_command"][result["worker_command"].index("-m")+1] == MODULE,
        "fresh V2 altered original deadline, physical limits or worker identity")
    return result


def runtime_sources():
    """HH_261004 - Archive fresh source/tests plus the entire unchanged V1 closure before declaration."""
    require(digest(Path(previous.__file__)) == PARENT_SHA256, "frozen full209 V1 owner changed")
    result = _sources()
    seeds = [Path(__file__).resolve(), Path(live.__file__).resolve(),
        ROOT/"learning/portable_e2e/temporal_navigation_terminal_policy_v3_timing_20261004.py",
        ROOT/"tools/e2e/audit_town03_full209_diagnostic_v2_20261004.py",
        ROOT/"learning/tests/test_temporal_navigation_terminal_policy_v3_timing_20261004.py",
        ROOT/"learning/tests/test_audit_town03_full209_diagnostic_v2_20261004.py",
        ROOT/"learning/tests/test_town03_full209_owner_v2_20261004.py"]
    require(all(path.is_file() and not path.is_symlink() for path in seeds), "fresh V2 closure is not prepared")
    for path in base.imported_source_closure(seeds):
        name, sha = str(path.relative_to(ROOT)), digest(path)
        require(name not in result or result[name] == sha, "fresh V2 source closure conflict")
        result[name] = sha
    return result


def diagnostic_report(output, plan, owner):
    # HH_261004 - Independent new diagnostics retain original thresholds and every actual/failure row.
    result = diagnostic.diagnostic_report(output, plan, owner)
    result.update(schema=SCHEMA+".diagnostic_report", fresh_owner_revision="v2", original_V1_preserved=True,
        capture_raw_ABI_schema=live.SCHEMA, official_completed_main_domains=0,
        official_completed_subfunctions=0, full_RTE01_approved=False, model_promoted=False)
    return result


@contextmanager
def activated(pins):
    """HH_261004 - Reuse the complete frozen ownership lifecycle; no lease, PID, asset or cleanup guard is removed."""
    global _active
    require(not _active and digest(Path(previous.__file__)) == PARENT_SHA256, "fresh V2 owner cannot nest or use changed V1")
    changes = dict(paths=paths, capture_argv=capture_argv, fixed_contract=fixed_contract,
        runtime_sources=runtime_sources, diagnostic_report=diagnostic_report, live=live, MODULE=MODULE, SCHEMA=SCHEMA)
    saved = {name:getattr(previous,name) for name in changes}
    _active = True
    try:
        for name,value in changes.items():
            setattr(previous,name,value)
        with previous.activated(pins):
            yield
    finally:
        for name,value in reversed(tuple(saved.items())):
            setattr(previous,name,value)
        _active = False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ("weights","metadata","checkpoint","declaration"):
        parser.add_argument("--expected-"+name+"-sha256", required=True)
    args,remaining = parser.parse_known_args(argv)
    pins = {name+"_sha256":getattr(args,"expected_"+name+"_sha256") for name in ("weights","metadata","checkpoint","declaration")}
    with activated(pins):
        return base.main(remaining)


if __name__ == "__main__":
    raise SystemExit(main())
