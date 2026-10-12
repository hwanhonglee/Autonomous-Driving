"""HH_261002 - Authenticate V7 actual inputs and reuse all frozen V6 true64/geometry checks, without TRAIN or control approval."""

from __future__ import annotations

import argparse
import ast
from functools import partial
from pathlib import Path
from types import FunctionType

from scripts.e2e import town07_natural_goal_stop_brakefree_v7_20261002 as task_v7
from scripts.e2e import audit_town07_natural_goal_future64_brakefree_v6_20261002 as frozen_v6_audit

FROZEN_V6_AUDIT_PATH = "tools/e2e/audit_town07_natural_goal_future64_brakefree_v6_20261002.py"
FROZEN_V6_AUDIT_SHA = "dd6dd09103f3df97bfcebcfcb008f9432b054aec20acd7e4f935a855e30c12c3"
V7_EXPECTED_SOURCES = {
    "tools/e2e/town07_natural_goal_stop_brakefree_v7_20261002.py": "4a1a9c1f808c0cefbd1a4e88afe66216a3e28d731f57342e6e11fcd10d724613",
    "tools/e2e/collect_town07_natural_goal_stop_brakefree_v7_20261002.py": "327fc003215546407a5724d953881572772947010e4529c28e31799fe7e6e436",
    "tools/e2e/run_owned_town07_natural_goal_stop_brakefree_v7_20261002.py": "d230237c45b44a7add9180e2f650cc7b0be6a49a32408cb59c1f3d2ec44eb575",
}


def _identity_names(tree):
    # HH_261002 - Rename one legacy source-table identifier and one exported identity key, never a formula, bound or mask.
    changes = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Name) and node.id == "V6_SOURCE_PINS":
            changes.append((node, "id", node.id)); node.id = "V7_SOURCE_PINS"
        elif isinstance(node, ast.keyword) and node.arg == "actual_v6_source_sha256":
            changes.append((node, "arg", node.arg)); node.arg = "actual_v7_source_sha256"
    task_v7.require(sum(old == "V6_SOURCE_PINS" for _, _, old in changes) == 4
        and sum(old == "actual_v6_source_sha256" for _, _, old in changes) == 1,
        "exact frozen V6 identity identifier counts required")
    def undo():
        for node, field, old in changes: setattr(node, field, old)
    return undo


_literals = {
    "carla.actual_natural_goal_future64_brakefree_diagnostic.20261002.v6":
        "carla.actual_natural_goal_future64_brakefree_diagnostic.20261002.v7",
    "v6": "v7",
    "Disclosed_BasicAgent_manual1_1p50_latched_brakefree_coast_low_speed_hold_v6":
        "Disclosed_BasicAgent_manual1_1p50_latched_brakefree_coast_low_speed_hold_v7",
    "actual_native_config_and_installed_controls_bind_declared_v6":
        "actual_native_config_and_installed_controls_bind_declared_v7",
    "v6 diagnostic cannot nest or attach to an active provider": "v7 diagnostic cannot nest or attach to an active provider",
}
for _old_path, _old_sha in task_v7.V6_SOURCES.items():
    _new_path = _old_path.replace("brakefree_v6_", "brakefree_v7_")
    _literals[_old_path] = _new_path; _literals[_old_sha] = V7_EXPECTED_SOURCES[_new_path]
task_v7.clone_frozen_provider(frozen_v6_audit, FROZEN_V6_AUDIT_SHA, globals(), literals=_literals,
    imports={"town07_natural_goal_stop_brakefree_v6_20261002": "town07_natural_goal_stop_brakefree_v7_20261002"},
    intervention=_identity_names)
require(V7_SOURCE_PINS == V7_EXPECTED_SOURCES, "all three actual frozen V7 source bytes required")
SOURCE_PINS[FROZEN_V6_AUDIT_PATH] = FROZEN_V6_AUDIT_SHA
_identity_inspect, _identity_main, _identity_teacher_witness = inspect, main, teacher_revision_witness
CASE_PLAN_SHA256 = "db4c667a6828ea75b3c90efb88512955ae869048f51d2f0719813ce932be3c38"
INPUT_SCHEMA = "carla.actual_natural_goal_future64_brakefree_input_pins.20261002.v7"
EPISODE_FILES = ("manifest.json", "moving_task_plan.json", "states.jsonl", "camera_frames.jsonl", "route.json",
    "control_receipts.jsonl", "actual_teacher_control.jsonl", "actual_goal_governor.jsonl",
    "actual_goal_target_assignments.jsonl", "actual_hazard_overrides.jsonl")
OWNER_FILES = ("owner_result.json", "diagnostic_report.json", "derived_body/summary.json")


def exact_sha(value):
    return type(value) is str and len(value) == 64 and all(char in "0123456789abcdef" for char in value)


def teacher_revision_witness(plan, manifest):
    result = _identity_teacher_witness(plan, manifest)
    declared = plan["causal_brakefree_terminal_mission"]
    actual = manifest["capture_contract"]["teacher_controller"]["causal_brakefree_terminal_mission"]
    require(actual == declared == task.mission_contract()
        and actual["normal_COAST_and_HOLD_steering_zero"] is True
        and actual["original_GO_lateral_and_all_downstream_BasicAgent_hazards_unchanged"] is True
        and actual["normal_COAST_HOLD_override_not_applied_to_hazard_outputs"] is True
        and actual["terminal_only_straightness_precondition"]["whole_route_collinearity_claim"] is False,
        "actual V7 terminal-only steering declaration must match the pinned original source route")
    result.update(terminal_only_zero_normal_steering_declaration_verified=True,
        steering_and_actual_ACK_match_is_prior_strict_owner_evidence_not_relaxed_here=True,
        unchanged_true64_geometry_can_still_fail_and_never_grants_TRAIN=True)
    return result


def caller_inputs(episode, plan_path, plan_sha, caller_path, caller_sha):
    # HH_261002 - Actual raw hashes come from the caller after capture, never from a fabricated prospective receipt.
    reader = previous.previous
    episode, plan_path, caller_path = canonical(episode), canonical(plan_path), canonical(caller_path)
    output, expected_plan = task.paths("v7", "pilot_00")
    require(episode.parent == output and episode.name in ("episode", "episode.partial")
        and plan_path == expected_plan and plan_sha == CASE_PLAN_SHA256 and exact_sha(caller_sha),
        "only the genuinely declared V7 pilot and exact external case-plan SHA are eligible for diagnostics")
    pins = {}
    data = reader.document(caller_path, pins, caller_sha)
    require(type(data) is dict and data.get("schema") == INPUT_SCHEMA and data.get("case_plan_sha256") == CASE_PLAN_SHA256,
        "exact independently pinned V7 raw-input receipt required")
    expected = {str(episode / name) for name in EPISODE_FILES} | {str(output / name) for name in OWNER_FILES}
    actual = data.get("artifact_sha256")
    require(type(actual) is dict and set(actual) == expected and all(exact_sha(value) for value in actual.values()),
        "all13 actual raw/owner/body artifacts must be externally pinned with exact lowercase SHA256 values")
    reader.pinned_bytes(plan_path, pins, CASE_PLAN_SHA256)
    for name, digest in actual.items(): reader.pinned_bytes(canonical(name), pins, digest)
    reader.recheck(pins)
    return pins


def inspect(episode, plan_path, plan_sha, *, actual_input_pins, actual_input_pins_sha256):
    pins = caller_inputs(episode, plan_path, plan_sha, actual_input_pins, actual_input_pins_sha256)
    summary, windows, audited = _identity_inspect(episode, plan_path, plan_sha)
    reader = previous.previous
    for path, expected in pins.items(): reader.pinned_bytes(path, audited, expected)
    revision = summary["audit_identity_revision"]
    revision.update(frozen_v6_auditor_sha256=FROZEN_V6_AUDIT_SHA,
        new_identity_and_postconstructor_order_checks_only=False,
        new_identity_terminal_steering_declaration_and_external_input_pins_only=True)
    summary.update(externally_pinned_actual_V7_artifact_count=13,
        external_input_receipt_sha256=pins[str(canonical(actual_input_pins))], source_artifact_sha256=dict(audited))
    reader.recheck(audited)
    return summary, windows, audited


def main(argv=None):
    parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    parser.add_argument("--actual-input-pins", required=True)
    parser.add_argument("--actual-input-pins-sha256", required=True)
    supplied, remaining = parser.parse_known_args(argv)
    # HH_261002 - Execute the exact frozen main/output/FAIL-exit implementation under a private callable namespace; no global rebinding.
    callback = partial(inspect, actual_input_pins=supplied.actual_input_pins,
        actual_input_pins_sha256=supplied.actual_input_pins_sha256)
    revised = FunctionType(_identity_main.__code__, dict(_identity_main.__globals__, inspect=callback),
        _identity_main.__name__, _identity_main.__defaults__, _identity_main.__closure__)
    require(revised.__code__ is _identity_main.__code__, "original create-only and FAIL exit implementation unchanged")
    return revised(remaining)


if __name__ == "__main__": raise SystemExit(main())
