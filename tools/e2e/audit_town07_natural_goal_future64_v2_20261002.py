"""HH_261002 - Thin v2 teacher binding for the frozen true-future64 diagnostic; never data admission."""

from __future__ import annotations

import argparse
import ast
from contextlib import contextmanager
import copy
import json
from pathlib import Path

from scripts.e2e import audit_town07_natural_goal_future64_20261002 as previous
from scripts.e2e import town07_natural_goal_stop_task_v2_20261002 as task


ROOT = previous.ROOT
CARLA_ROOT = Path("/home/a/carla-autoware-universe/CARLA_0.9.15")
SCHEMA = "carla.actual_natural_goal_future64_diagnostic.20261002.v2"
TASK_SCHEMA, PROFILE = task.SCHEMA, task.PROFILE
FROZEN_AUDIT_PATH = "tools/e2e/audit_town07_natural_goal_future64_20261002.py"
FROZEN_AUDIT_SHA = "1e8c3105b37705cdc0cb680f2371af9cc722945d585bf96c9570d5af0748fa68"
V2_SOURCE_PINS = {
    "tools/e2e/town07_natural_goal_stop_task_v2_20261002.py": "2e178170525ea8a00f4885d3007c0992de0c6b4bb9c8ddb79ba8afda94e4bdb0",
    "tools/e2e/collect_town07_natural_goal_stop_v2_20261002.py": "34ccfce44801462679cff8ea55122d55aaa6a6cdb4c12fc844bd96d244c3c396",
    "tools/e2e/run_owned_town07_natural_goal_stop_v2_20261002.py": "9d5923a7c715d0da536eed248cd1e0b6e49bb2a14fc5f31604297536417365e4",
}
SOURCE_PINS = dict(previous.SOURCE_PINS, **V2_SOURCE_PINS, **{FROZEN_AUDIT_PATH: FROZEN_AUDIT_SHA})
require, canonical = previous.require, previous.canonical
_active = False


def sha_value(value):
    return isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def verify_frozen():
    # HH_261002 - Read bytes without importing the physical owner, Torch, CARLA or a pickle checkpoint.
    pins = {}
    for name, expected in SOURCE_PINS.items():
        previous.pinned_bytes(ROOT/name, pins, expected)
    return pins


@contextmanager
def activated():
    """HH_261002 - Change version/provider names only; every frozen future, mask, bound and denial function remains exact."""
    global _active
    require(not _active and not task._active, "v2 future diagnostic cannot nest or attach to an active teacher provider")
    verify_frozen()
    values = dict(SCHEMA=SCHEMA, TASK_SCHEMA=TASK_SCHEMA, PROFILE=PROFILE,
        SOURCE_PINS=SOURCE_PINS, natural_task=task)
    saved = {name:getattr(previous, name) for name in values}
    _active = True
    try:
        for name, value in values.items():setattr(previous, name, value)
        yield
    finally:
        for name, value in saved.items():setattr(previous, name, value)
        _active = False


def inspector_revision():
    """HH_261002 - Mechanically replace only fixed_contract's literal run-id; prove all other inspector statements identical."""
    tree = ast.parse((ROOT/FROZEN_AUDIT_PATH).read_bytes())
    matches = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "inspect"]
    require(len(matches) == 1, "one frozen inspector required")
    function = copy.deepcopy(matches[0]); original = copy.deepcopy(function)
    calls = [node for node in ast.walk(function) if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute) and node.func.attr == "fixed_contract"
        and isinstance(node.func.value, ast.Name) and node.func.value.id == "natural_task"]
    require(len(calls) == 1 and not calls[0].keywords
        and ast.dump(calls[0], include_attributes=False) == ast.dump(
            ast.parse("natural_task.fixed_contract('v1','pilot_00')", mode="eval").body, include_attributes=False),
        "exact frozen v1 provider-call anchor required")
    calls[0].args[0].value = "v2"
    restored = copy.deepcopy(function)
    next(node for node in ast.walk(restored) if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute) and node.func.attr == "fixed_contract"
        and isinstance(node.func.value, ast.Name) and node.func.value.id == "natural_task").args[0].value = "v1"
    require(ast.dump(restored, include_attributes=False) == ast.dump(original, include_attributes=False),
        "all original inspector/source/pixel/owner/body/governor statements must remain AST-identical")
    namespace = dict(vars(previous))
    module = ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[]))
    exec(compile(module, str(Path(__file__))+":frozen_inspector_v2_run_id_only", "exec"), namespace)
    return namespace["inspect"]


def declared_closure(plan, episode, pins):
    """HH_261002 - Additionally bind every declared live/archive source, external API and asset without importing it."""
    sections = ("source_sha256", "external_source_sha256", "asset_sha256")
    for section in sections:
        require(isinstance(plan.get(section), dict) and plan[section], "complete declared source/API/asset closure required")
        require(all(sha_value(value) for value in plan[section].values()), "declared closure SHA must be exact lowercase hex")
    needed = dict(task.FROZEN_V1, **V2_SOURCE_PINS)
    require(all(plan["source_sha256"].get(name) == expected for name, expected in needed.items()),
        "all three frozen v1 and three prospective v2 teacher source pins required")
    output = episode.parent
    for name, expected in plan["source_sha256"].items():
        relative = Path(name)
        require(isinstance(name, str) and not relative.is_absolute() and ".." not in relative.parts,
            "workspace source must be a nonescaping relative path")
        live, archive = canonical(ROOT/relative), canonical(output/"source"/relative)
        require(live.is_relative_to(ROOT) and archive.is_relative_to(output/"source"), "source closure escaped fixed roots")
        previous.pinned_bytes(live, pins, expected);previous.pinned_bytes(archive, pins, expected)
    for section in sections[1:]:
        for name, expected in plan[section].items():
            source = Path(name)
            require(source.is_absolute(), "declared external/API asset must be absolute")
            source = canonical(source)
            if section == "external_source_sha256":
                require(source.is_relative_to(CARLA_ROOT), "external API escaped declared CARLA root")
            require(source.is_relative_to(ROOT) or source.is_relative_to(CARLA_ROOT), "asset escaped fixed ownership roots")
            prefix, root = ("source", ROOT) if source.is_relative_to(ROOT) else ("source_external", CARLA_ROOT)
            archive = canonical(output/prefix/source.relative_to(root))
            require(archive.is_relative_to(output/prefix), "asset archive escaped owner output")
            previous.pinned_bytes(source, pins, expected);previous.pinned_bytes(archive, pins, expected)
    return dict(all_declared_live_and_archive_bytes_rechecked=True,
        source_count=len(plan["source_sha256"]), external_source_count=len(plan["external_source_sha256"]),
        asset_count=len(plan["asset_sha256"]), import_graph_recomputed_by_this_adapter=False,
        physical_owner_source_closure_is_prior_pinned_evidence=True)


def teacher_revision_witness(plan, manifest):
    """HH_261002 - Authenticate the actually recorded prospective controller/configuration, not just its new name."""
    require(plan["schema"] == TASK_SCHEMA and plan["run_id"] == "v2", "prospectively declared v2 plan required")
    capture = manifest["capture_contract"]
    config = plan["natural_goal_configuration"]
    require(all(key in capture["goal_stop_profile"] and type(capture["goal_stop_profile"][key]) is type(value)
        and capture["goal_stop_profile"][key] == value for key, value in config.items()),
        "all actual native governor settings must equal the prospective v2 configuration")
    metadata, effective = capture["teacher_controller"], capture["goal_stop_profile"]["effective_control"]
    require(manifest.get("control_source") == "Disclosed_BasicAgent_natural_goal_stop_gentle_normal_brakes_manual1_v2"
        and metadata["normal_brake_cap"] == effective["normal_brake_cap"] == .01
        and metadata["normal_throttle_cap"] == effective["normal_throttle_cap"] == .20
        and metadata["prospective_desired_deceleration_mps2"] == .2
        and metadata["normal_launch_controller_revision"] == TASK_SCHEMA+".normal_control"
        and metadata["manual_gear_shift"] is True and type(metadata["gear"]) is int and metadata["gear"] == 1
        and metadata["native_float32_globals_modified"] is False
        and metadata["governor_math_changed"] is False
        and metadata["individual_factor_causal_effect_claim"] is False
        and metadata["inherited_BasicAgent_emergency_unchanged"] is True
        and effective["basic_agent_emergency_brake_modified"] is False,
        "actual installed v2 normal-cap/manual1/hazard metadata differs")
    return dict(actual_native_config_and_installed_normal_caps_bind_declared_v2=True,
        original_governor_math_and_BasicAgent_hazards_unchanged=True,
        normal_brake_cap=.01, desired_deceleration_mps2=.2,
        individual_factor_causal_effect_claim=False, measured_brake_capability_claimed=False)


def inspect(episode, plan_path, plan_sha):
    first_read = {}
    previous.pinned_bytes(Path(__file__).resolve(), first_read)
    with activated():
        summary, windows, pins = inspector_revision()(episode, plan_path, plan_sha)
        # HH_261002 - Preserve the adapter's first-read pin across all original future/pixel work.
        for name, expected in first_read.items():previous.pinned_bytes(name, pins, expected)
        plan = previous.document(plan_path, pins, plan_sha)
        closure = declared_closure(plan, canonical(episode), pins)
        manifest = previous.document(canonical(episode)/"manifest.json", pins)
        revision = teacher_revision_witness(plan, manifest)
        # HH_261002 - These remain prior physical receipts; the future-only adapter does not rerun actuation QA.
        for name in ("control_receipts.jsonl", "actual_teacher_control.jsonl"):
            previous.pinned_bytes(canonical(episode)/name, pins)
        require(plan["controller"]["normal_brake_cap"] == .01
            and plan["natural_goal_configuration"]["normal_brake_cap"] == .01
            and plan["natural_goal_configuration"]["desired_deceleration_mps2"] == .2
            and plan["prospective_two_factor_revision"]["individual_factor_causal_effect_claim"] is False,
            "exact prospective two-factor v2 teacher settings required")
        summary.update(diagnostic_adapter_schema=SCHEMA, teacher_task_schema=TASK_SCHEMA,
            exact_v2_teacher_profile=PROFILE, declared_normal_brake_cap=.01, declared_desired_deceleration_mps2=.2,
            v1_diagnostic_source_sha256=FROZEN_AUDIT_SHA,
            frozen_inspector_AST_changed_only_fixed_contract_run_id=True,
            all_frozen_true64_native_physical_camera_semantic_denial_checks_unchanged=True,
            full_declared_source_and_asset_closure=closure,
            actual_teacher_revision_witness=revision,
            individual_factor_causal_effect_claim=False, adapter_never_grants_training_or_vehicle_approval=True)
        previous.recheck(pins)
        return summary, windows, pins


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--episode", required=True);parser.add_argument("--case-plan", required=True)
    parser.add_argument("--case-plan-sha256", required=True);parser.add_argument("--output")
    args = parser.parse_args(argv)
    summary, windows, pins = inspect(args.episode, args.case_plan, args.case_plan_sha256)
    if args.output:
        output = canonical(args.output)
        require(output.is_relative_to(ROOT) and not output.exists()
            and not output.is_relative_to(canonical(args.episode).parent), "fresh derived output outside immutable pilot required")
        previous.recheck(pins)
        output.mkdir(parents=True, exist_ok=False)
        with (output/"summary.json").open("x", encoding="utf-8") as stream:
            json.dump(summary, stream, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False);stream.write("\n")
        with (output/"all_fixed_anchor_true_future64.jsonl").open("x", encoding="utf-8") as stream:
            for row in windows:stream.write(json.dumps(row, sort_keys=True, allow_nan=False)+"\n")
        previous.recheck(pins)
    print(json.dumps({k:v for k,v in summary.items() if k != "source_artifact_sha256"}, indent=2, allow_nan=False))
    return 0 if summary["failed_fixed_window_count"] == 0 and summary["full_raw_event_count"] == 0 \
        and not summary["source_capture_failed"] else 2


if __name__ == "__main__":raise SystemExit(main())
