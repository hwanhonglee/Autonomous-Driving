"""HH_261002 - Bind frozen true64 mathematics to the declared v6 phase-hook-order revision. Never grant TRAIN permission."""

from __future__ import annotations

import ast
from contextlib import contextmanager
import copy
from pathlib import Path

from scripts.e2e import audit_town07_natural_goal_future64_brakefree_v5_20261002 as previous
from scripts.e2e import town07_natural_goal_stop_brakefree_v6_20261002 as task

ROOT, require, canonical = previous.ROOT, previous.require, previous.canonical
SCHEMA = "carla.actual_natural_goal_future64_brakefree_diagnostic.20261002.v6"
TASK_SCHEMA, PROFILE = task.SCHEMA, task.PROFILE
FROZEN_V5_AUDIT_PATH = "tools/e2e/audit_town07_natural_goal_future64_brakefree_v5_20261002.py"
FROZEN_V5_AUDIT_SHA = "e96450e85d0f0afc3044041d7a78a13f9f21a7a58475f456be111018ffbf308f"
V6_SOURCE_PINS = {
    "tools/e2e/town07_natural_goal_stop_brakefree_v6_20261002.py": "de86e3610992af663235d7b733868b7525aa4177d967fe936a1628ab642fb73c",
    "tools/e2e/collect_town07_natural_goal_stop_brakefree_v6_20261002.py": "bc83374a3dcd3f25b4776da3062b4009d73217ee15949d322b912d39bf239f73",
    "tools/e2e/run_owned_town07_natural_goal_stop_brakefree_v6_20261002.py": "07c628820da9e2eaae120ab7b0a4e595ce2f96c0063de03ac4f2b2964c498719",
}
SOURCE_PINS = dict(previous.SOURCE_PINS, **task.FROZEN, **V6_SOURCE_PINS)
SOURCE_PINS[FROZEN_V5_AUDIT_PATH] = FROZEN_V5_AUDIT_SHA
# HH_261002 - These are the very same frozen numerical function objects, not copied or redefined formulas.
defined_geometry, audit_core = previous.defined_geometry, previous.audit_core
intent_witness, governor_receipt_binding = previous.intent_witness, previous.governor_receipt_binding
_active = False


def verify_frozen():
    pins = {}
    for name, expected in SOURCE_PINS.items(): previous.previous.pinned_bytes(ROOT/name, pins, expected)
    return pins


def revised_function(name):
    """HH_261002 - Inverse-AST permit only fixed v6 identity constants; all numerical and source-verification statements stay exact."""
    require(name in ("definition", "teacher_revision_witness"), "only the two declared identity readers may be revised")
    verify_frozen()
    tree = ast.parse((ROOT/FROZEN_V5_AUDIT_PATH).read_bytes())
    original = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == name)
    node = copy.deepcopy(original)
    mapping = {"v5": "v6"} if name == "definition" else {
        "Disclosed_BasicAgent_manual1_1p50_latched_brakefree_coast_low_speed_hold_v5":
            "Disclosed_BasicAgent_manual1_1p50_latched_brakefree_coast_low_speed_hold_v6",
        "actual_native_config_and_installed_controls_bind_declared_v5":
            "actual_native_config_and_installed_controls_bind_declared_v6",
    }
    changed = []
    for value in ast.walk(node):
        if isinstance(value, ast.Constant) and type(value.value) is str and value.value in mapping:
            changed.append((value,"value",value.value)); value.value = mapping[value.value]
        elif isinstance(value, ast.keyword) and value.arg in mapping:
            changed.append((value,"arg",value.arg)); value.arg = mapping[value.arg]
    require(len(changed) == len(mapping) and {old for _,_,old in changed} == set(mapping),
        "one exact occurrence of each frozen identity constant required")
    revised = copy.deepcopy(node)
    for value,field,old in changed: setattr(value,field,old)
    require(ast.dump(node, include_attributes=False) == ast.dump(original, include_attributes=False),
        "any frozen formula, mask, bound or source-closure statement changed")
    namespace = dict(vars(previous), task=task, TASK_SCHEMA=TASK_SCHEMA, PROFILE=PROFILE, __file__=__file__)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[revised], type_ignores=[])),
        str(__file__)+":identity_only_"+name, "exec"), namespace)
    return namespace[name]


def definition(name):
    return revised_function("definition")(name)


def teacher_revision_witness(plan, manifest):
    result = revised_function("teacher_revision_witness")(plan, manifest)
    capture = manifest["capture_contract"]
    metadata, effective = capture["teacher_controller"], capture["goal_stop_profile"]["effective_control"]
    require(metadata.get("phase_hook_after_original_outer_cap") is True
        and effective.get("phase_hook_rebound_after_original_outer_cap") is True
        and effective.get("original_two_normal_cap_calls_and_requests_unchanged") is True
        and effective.get("downstream_BasicAgent_hazard_untouched") is True,
        "actual v6 constructor must preserve BOTH original normal caps before the authenticated HOLD hook")
    result.update(actual_postconstructor_phase_order_witness_verified=True,
        original_two_normal_cap_calls_and_requests_unchanged=True,
        original_strict_normal_ACK_replay_is_required_owner_evidence=True,
        order_witness_does_not_promote_FAILED_capture_or_defined_geometry=True)
    return result


@contextmanager
def bindings():
    # HH_261002 - Never reinterpret v5 bytes globally or nest an adapter around another live in-process capture.
    global _active
    require(not _active and not previous._active and not previous.support._active
        and not previous.task._active and not task._active, "v6 diagnostic cannot nest or attach to an active provider")
    verify_frozen()
    values = dict(task=task, SCHEMA=SCHEMA, TASK_SCHEMA=TASK_SCHEMA, PROFILE=PROFILE,
        SOURCE_PINS=SOURCE_PINS, V5_SOURCE_PINS=V6_SOURCE_PINS,
        definition=definition, teacher_revision_witness=teacher_revision_witness)
    saved = {k:getattr(previous,k) for k in values}; _active = True
    try:
        for k,v in values.items(): setattr(previous,k,v)
        yield
    finally:
        for k,v in saved.items(): setattr(previous,k,v)
        _active = False


@contextmanager
def activated():
    with bindings():
        with previous.activated(): yield


def inspect(episode, plan_path, plan_sha):
    first = {}; previous.previous.pinned_bytes(Path(__file__), first)
    with bindings():
        summary, windows, pins = previous.inspect(episode, plan_path, plan_sha)
        for path, expected in first.items(): previous.previous.pinned_bytes(path, pins, expected)
        summary.update(audit_identity_revision=dict(schema=SCHEMA, frozen_v5_auditor_sha256=FROZEN_V5_AUDIT_SHA,
            actual_v6_source_sha256=dict(V6_SOURCE_PINS), v5_numerical_masks_bounds_and_true64_function_objects_unchanged=True,
            new_identity_and_postconstructor_order_checks_only=True, training_authorized=False),
            source_artifact_sha256=dict(pins))
        previous.previous.recheck(pins)
        return summary, windows, pins


def main(argv=None):
    # HH_261002 - Keep the frozen create-only output, before/after pin checks and fail-retaining exit logic byte-identical.
    verify_frozen()
    tree = ast.parse((ROOT/FROZEN_V5_AUDIT_PATH).read_bytes())
    node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "main")
    namespace = dict(vars(previous), inspect=inspect, ROOT=ROOT, __doc__=__doc__)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[])),
        str(__file__)+":unchanged_create_only_main", "exec"), namespace)
    require(namespace["main"].__code__.co_code == previous.main.__code__.co_code,
        "frozen output and exit implementation must remain byte-identical")
    return namespace["main"](argv)


if __name__ == "__main__": raise SystemExit(main())
