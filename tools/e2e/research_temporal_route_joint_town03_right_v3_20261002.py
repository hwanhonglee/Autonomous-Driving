"""HH_261002 - Own a fresh true learned-hook Town03 turn and preserve both earlier failed attempts."""
from __future__ import annotations

import argparse
import ast
import copy
from contextlib import contextmanager
from pathlib import Path

from . import research_temporal_route_joint_town03_right_v2_20261002 as previous
from . import collect_temporal_route_joint_town03_right_v3_20261002 as live

base, numeric, ROOT = previous.base, previous.numeric, previous.ROOT
MODULE = "scripts.e2e.research_temporal_route_joint_town03_right_v3_20261002"
SCHEMA = live.OWNER_SCHEMA
V2_SHA = "a8af775ddaeb1fd907f4f021696f2c937bcfb05884688f841f2d0fa9f8318571"
_active = False
_original_fixed, _original_sources = previous.fixed_contract, previous.runtime_sources
_original_paths = previous.previous.paths
_original_diagnostic = previous.diagnostic_report
_original_complete = previous.previous.complete_interval


def paths(run_id):
    previous.previous.require(run_id == "v3", "new actual runtime binding requires fresh v3 output")
    return _original_paths(run_id)


def fixed_contract(run_id, longitudinal_pi):
    result = _original_fixed(run_id, longitudinal_pi)
    result.update(actual_scoped_camera_and_ack_transport_late_bound_before_capture=True,
        all_12_real_warmup_bundles_required_before_first_driving_command=True,
        unused_session_or_teacher_capture_is_not_learned_performance=True,
        actual_hazard_witness_denominator="last warmup plus original BasicAgent-run-step-true driving frames; authenticated terminal false only",
        v1_and_teacher_only_v2_failures_preserved=True)
    return result


def runtime_sources():
    previous.previous.require(previous.previous.digest(Path(previous.__file__)) == V2_SHA,
                              "frozen teacher-only v2 owner differs")
    result = _original_sources()
    for path in base.imported_source_closure((Path(__file__).resolve(),)):
        name, sha = str(path.relative_to(ROOT)), previous.previous.digest(path)
        previous.previous.require(name not in result or result[name] == sha, "new source closure conflict")
        result[name] = sha
    return result


def complete_interval(manifest, states, cameras, event):
    # HH_261002 - Authenticate and correct only the original capture's literal budget error, not its whole failed-tail denominator.
    old = "BasicAgent did not finish within the driving-phase tick budget before the maximum capture duration"
    exact = "BasicAgent did not finish within the driving budget reserved by the maximum total duration and stationary tail"
    live.verify_frozen()
    capture = ast.parse(Path(live.collector.__file__).read_bytes())
    original_capture = next(n for n in capture.body if isinstance(n, ast.FunctionDef) and n.name == "collect_episode")
    previous.previous.require(sum(isinstance(n, ast.Constant) and n.value == exact for n in ast.walk(original_capture)) == 1,
                              "exact frozen original budget error is required")
    tree = ast.parse(Path(previous.previous.__file__).read_bytes())
    original = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "complete_interval")
    changed = copy.deepcopy(original); count = 0
    for node in ast.walk(changed):
        if isinstance(node, ast.Constant) and node.value == old: node.value = exact; count += 1
    previous.previous.require(count == 1, "exact one failed-budget diagnostic literal required")
    restored = copy.deepcopy(changed)
    for node in ast.walk(restored):
        if isinstance(node, ast.Constant) and node.value == exact: node.value = old
    previous.previous.require(ast.dump(restored) == ast.dump(original), "another complete-interval criterion changed")
    namespace = dict(vars(previous.previous))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed], type_ignores=[])),
                 str(Path(__file__))+":original_full_failed_budget_literal", "exec"), namespace)
    return namespace["complete_interval"](manifest, states, cameras, event)


def expected_hazard_frames(states, manifest):
    require = previous.previous.require
    warmup = [s for s in states if s["capture_phase"] == "stationary_warmup"]
    require(len(warmup) == 24 and len(states) > 24, "complete actual warmup required for hazard census")
    driving = states[24:]; result = [warmup[-1]["frame"]]
    for state in driving:
        require(state["capture_phase"] == "driving", "entire subsequent actual driving interval required")
        observation = state.get("traffic_light_observation", {})
        flag = observation.get("basic_agent_run_step_this_frame")
        require(type(flag) is bool and observation.get("frame") == observation.get("frame_before")
            == observation.get("frame_after") == state["frame"], "actual original detector-call branch witness required")
        if flag: result.append(state["frame"])
        else:
            terminal = manifest.get("result", {})
            require(state is states[-1] and terminal.get("goal_reached") is True
                and terminal.get("termination_reason") == "catalog_goal_tolerance",
                "only the explicit final original catalog-goal branch may omit a BasicAgent decision")
    return result


def diagnostic_function():
    # HH_261002 - Correct only the frozen v2 expected detector-call frames and paired driving census, preserving all other checks.
    tree = ast.parse(Path(previous.__file__).read_bytes())
    original = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "diagnostic_report")
    changed = copy.deepcopy(original); counts = dict(expected=0, zip=0)
    for node in ast.walk(changed):
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name) \
                and node.targets[0].id == "expected_frames":
            node.value = ast.parse("expected_hazard_frames(states, manifest)", mode="eval").body; counts["expected"] += 1
        if isinstance(node, ast.For) and ast.unparse(node.iter) == "zip(driving, witness[1:])":
            node.iter.args[0] = ast.parse("[s for s in driving if s['frame'] in set(expected_frames)]", mode="eval").body
            counts["zip"] += 1
    previous.previous.require(counts == dict(expected=1, zip=1), "exact two hazard-denominator AST anchors required")
    restored = copy.deepcopy(changed)
    for node in ast.walk(restored):
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name) \
                and node.targets[0].id == "expected_frames":
            node.value = ast.parse("[states[23]['frame']] + [s['frame'] for s in driving]", mode="eval").body
        if isinstance(node, ast.For) and ast.unparse(node.iter).startswith("zip([s for s in driving"):
            node.iter.args[0] = ast.Name(id="driving", ctx=ast.Load())
    previous.previous.require(ast.dump(restored) == ast.dump(original), "another original fixture/body/motion/latency check changed")
    namespace = dict(vars(previous), live=live, SCHEMA=SCHEMA, expected_hazard_frames=expected_hazard_frames)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed], type_ignores=[])),
                 str(Path(__file__))+":actual_original_hazard_denominator", "exec"), namespace)
    return namespace["diagnostic_report"]


def diagnostic_report(output, plan, owner):
    result = diagnostic_function()(output, plan, owner)
    result["actual_learned_runtime_binding_passed"] = False
    try:
        episode = output/"episode" if (output/"episode").is_dir() else output/"episode.partial"
        proof = previous.previous.read_json(episode/"actual_runtime_binding.json")
        summary = previous.previous.read_json(episode/"learned_summary.json")
        previous.previous.require(proof["schema"] == live.SCHEMA+".actual_runtime_binding"
            and proof["actual_scoped_callbacks_bound"] is True and proof["first_driving_guard_passed"] is True
            and proof["warmup_observed_bundle_count"] == 12 and proof["warmup_native_count"] == 24
            and proof["observed_bundle_calls"] == result["actual_camera_bundle_count"], "full actual observed policy binding required")
        counts = summary["counts"]
        previous.previous.require(summary["handoff_occurred"] is True and counts["prefill_count"] >= 2
            and counts["inference_count"] > 0 and counts["learned_commands_acknowledged"] > 0,
            "actual learned handoff and accepted NN effect are required, not a teacher catalog completion")
        previous.previous.verify_closure(output, plan)
        previous.previous.require(all(previous.previous.digest(name) == sha for name, sha in result["raw_source_sha256"].items()),
                                  "runtime witness/raw inputs changed during audit")
        result["actual_learned_runtime_binding_passed"] = True
    except Exception as error: result["runtime_binding_diagnostic_error"] = type(error).__name__+": "+str(error)
    for key in ("learned_driving_screen_passed", "joint_turn_motion_body_policy_wall100ms_passed"):
        result[key] = result.get(key, False) and result["actual_learned_runtime_binding_passed"]
    result["status"] = "PASS_KNOWN_DEV_RIGHT_TURN_PRECURSOR" if result["joint_turn_motion_body_policy_wall100ms_passed"] else "FAIL_KNOWN_DEV_RIGHT_TURN_PRECURSOR"
    return result


@contextmanager
def activated(pins):
    global _active
    previous.previous.require(not _active, "new v3 owner cannot nest")
    previous.previous.require(previous.previous.digest(Path(previous.__file__)) == V2_SHA, "failed v2 owner must remain frozen")
    with previous.activated(pins):
        values = dict(paths=paths, fixed_contract=fixed_contract, runtime_sources=runtime_sources,
            diagnostic_report=diagnostic_report, live=live, MODULE=MODULE, SCHEMA=SCHEMA)
        targets = (previous.previous, previous, base)
        saved = {target: {key: getattr(target, key) for key in values} for target in targets}
        old_validate, old_numeric = live.frozen.validate_capture, numeric.MODULE
        old_complete = previous.previous.complete_interval
        _active = True
        try:
            for target in targets:
                for key, value in values.items(): setattr(target, key, value)
            live.frozen.validate_capture = live.validate_capture; numeric.MODULE = MODULE
            previous.previous.complete_interval = complete_interval
            yield
        finally:
            live.frozen.validate_capture = old_validate; numeric.MODULE = old_numeric
            previous.previous.complete_interval = old_complete
            for target in reversed(targets):
                for key, value in saved[target].items(): setattr(target, key, value)
            _active = False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ("weights", "metadata", "checkpoint", "declaration"):
        parser.add_argument("--expected-"+name+"-sha256", required=True)
    args, remaining = parser.parse_known_args(argv)
    pins = {name+"_sha256": getattr(args, "expected_"+name+"_sha256") for name in ("weights", "metadata", "checkpoint", "declaration")}
    with activated(pins): return base.main(remaining)


if __name__ == "__main__": raise SystemExit(main())
