"""HH_261002 - Prepared matched LEFT20k lateral-controller pair; ROOT alone owns physical execution."""
from __future__ import annotations

import ast
import copy
import math
from pathlib import Path
from types import FunctionType, SimpleNamespace

from . import research_temporal_left_joint_left45_20261002 as previous
from . import collect_temporal_left_joint_lateral_pair_20261002 as live

for _name, _value in vars(previous).items():
    if not _name.startswith("__"):
        globals()[_name] = _value
live = __import__("scripts.e2e.collect_temporal_left_joint_lateral_pair_20261002", fromlist=["live"])
SCHEMA, MODULE = live.OWNER_SCHEMA, "scripts.e2e.research_temporal_left_joint_lateral_pair_20261002"
ARMS = live.ARMS
FROZEN_OWNER_SHA = "7810e2285f6d90e4e66ab85d62b2a1eca87f55777859e6756d81643b8f21f7fa"
_scope, _active, _owner_plan_sha, _batch_sha = None, False, None, None
task, ASSETS, _originals = None, None, None
_owner_plan_sha_or_report = None


def verify_frozen():
    require(digest(Path(previous.__file__)) == FROZEN_OWNER_SHA, "exact frozen LEFT comparison owner required")
    live.verify_frozen()


def root_path():
    return ROOT / "storage/experiments/validation/2026-10-02/temporal_left_joint_lateral_pair_v1"


def definition(name):
    verify_frozen()
    original = next(n for n in ast.parse(Path(previous.__file__).read_bytes()).body
        if isinstance(n, ast.FunctionDef) and n.name == name)
    changed = copy.deepcopy(original)
    if name == "load_model":
        hits = [n for n in ast.walk(changed) if isinstance(n, ast.Compare)
            and ast.dump(n) == ast.dump(ast.parse('_scope.arm=="LEFT20000"', mode="eval").body)]
        require(len(hits) == 1, "one exact original model-loader arm branch required")
        hits[0].ops, hits[0].comparators = [ast.In()], [ast.Name(id="ARMS", ctx=ast.Load())]
    elif name == "main":
        hits = [n for n in ast.walk(changed) if isinstance(n, ast.Constant) and n.value == "LEFT20000"]
        paths = [n for n in ast.walk(changed) if isinstance(n, ast.Constant) and n.value == "parent6000"]
        files = [n for n in ast.walk(changed) if isinstance(n, ast.Constant) and n.value == "parent6000/owner_result.json"]
        require(len(hits) == len(paths) == len(files) == 1, "exact original sequential closed-owner guard required")
        hits[0].value = "short_lookahead"
        for node in paths: node.value = "baseline"
        files[0].value = "baseline/owner_result.json"
    restored = copy.deepcopy(changed)
    class Restore(ast.NodeTransformer):
        def visit_Compare(self, node):
            if name == "load_model" and ast.dump(node) == ast.dump(ast.parse("_scope.arm in ARMS", mode="eval").body):
                return ast.parse('_scope.arm=="LEFT20000"', mode="eval").body
            return self.generic_visit(node)
        def visit_Constant(self, node):
            if name == "main" and node.value == "short_lookahead": node.value = "LEFT20000"
            elif name == "main" and node.value == "baseline": node.value = "parent6000"
            elif name == "main" and node.value == "baseline/owner_result.json": node.value = "parent6000/owner_result.json"
            return node
    require(ast.dump(Restore().visit(restored)) == ast.dump(original), "another original owner statement changed")
    return ast.fix_missing_locations(changed)


for _name, _value in vars(previous).items():
    if isinstance(_value, FunctionType) and _value.__module__ == previous.__name__ and _name not in ("root_path", "load_model", "main"):
        exec(compile(ast.Module(body=[definition(_name)], type_ignores=[]), __file__, "exec"), globals())
for _name in ("load_model", "main"):
    exec(compile(ast.Module(body=[definition(_name)], type_ignores=[]), __file__, "exec"), globals())
_load_model = load_model
_decorate = decorate_task_contract
_diagnostic = diagnostic_report


def load_model(plan_sha=None):
    model = _load_model(plan_sha)
    model._research_lateral_lookahead_calibration = live.calibration_contract(_scope.arm)
    return model


def decorate_task_contract(result):
    value = _decorate(result)
    value.update(lateral_lookahead_calibration=live.calibration_contract(_scope.arm),
        experiment="Matched same LEFT20k fixed lateral lookahead A/B; not a new trained model",
        same_final_LEFT20k_weights_both_arms=True, matched_baseline_repeat=True,
        previous_GOOD6000_and_LEFT20k_failures_preserved=True,
        persegment_diagnostics_not_new_PASS_thresholds=True)
    return value


def configuration_report(episode):
    # HH_261002 - Preserve every original9-field/normal-ACK/STOP preview audit; change only its two declared lateral preset literals.
    proto = previous.preview_owner.proto
    require(digest(Path(proto.__file__)) == previous.preview_owner.previous.FROZEN_OWNER_SHA,
        "frozen configuration authenticator changed")
    original = next(n for n in ast.parse(Path(proto.__file__).read_bytes()).body
        if isinstance(n, ast.FunctionDef) and n.name == "configuration_report")
    changed = copy.deepcopy(original); profile = live.PROFILES[_scope.arm]
    speed = [n for n in ast.walk(changed) if isinstance(n, ast.Compare)
        and ast.dump(n.left) == ast.dump(ast.parse("controller['speed_preview_s']", mode="eval").body)]
    require(len(speed) == 1 and speed[0].comparators[0].value == .8, "original preview predicate changed")
    speed[0].comparators[0].value = .5
    settings = [n for n in ast.walk(changed) if isinstance(n, ast.keyword) and n.arg in ("minimum_lookahead_m", "lookahead_time_s")]
    require(len(settings) == 2 and {n.arg:n.value.value for n in settings} == dict(minimum_lookahead_m=1.5,lookahead_time_s=.8),
        "exact original lateral predicates required")
    for node in settings: node.value.value = profile[0 if node.arg == "minimum_lookahead_m" else 1]
    restored = copy.deepcopy(changed)
    for node in ast.walk(restored):
        if isinstance(node, ast.Compare) and ast.dump(node.left) == ast.dump(speed[0].left):node.comparators[0].value = .8
        if isinstance(node, ast.keyword) and node.arg == "minimum_lookahead_m":node.value.value = 1.5
        if isinstance(node, ast.keyword) and node.arg == "lookahead_time_s":node.value.value = .8
    require(ast.dump(restored) == ast.dump(original), "another original configuration receipt gate changed")
    namespace = dict(vars(proto),live=live.preview)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed],type_ignores=[])),__file__,"exec"),namespace)
    result = namespace["configuration_report"](episode)
    commands = {row["ack_receipt_sequence"]:row for row in jsonl(episode/"learned_control.jsonl")}
    wall = {row["command_frame"]:row for row in jsonl(episode/"wall_timing.jsonl")
        if row.get("command_reason") in ("initial_drive_control","next_driving_control") and row.get("command_frame") is not None}
    checked = 0
    for row in jsonl(episode/"speed_preview_configuration.jsonl"):
        receipt = row["tracker_receipt"]
        if receipt is None:continue
        raw = commands[wall[row["command_frame"]]["ACK_sequence"]]
        config = live.preview.runtime.ControllerConfig(**row["actual_policy_controller"])
        config = live.replace(config,speed_preview_s=receipt["effective_speed_preview_s"])
        if receipt["original_STOP_tracker_preserved"]:config = live.replace(config,minimum_lookahead_m=1.5,lookahead_time_s=.8)
        expected = live._base_tracker(SimpleNamespace(**raw["selected"]),raw["measured_current_speed_mps"],config)
        # HH_261002 - Match the producer's JSON representation of tuple pursuit points without rounding or tolerance changes.
        require(receipt["original_tracker_control"] == json.loads(json.dumps(expected,allow_nan=False)),
            "original learned selected geometry/lateral control math differs")
        checked += 1
    require(checked == result["fresh_tracker_count"],"all fresh lateral tracker receipts required")
    result.update(lateral_lookahead_calibration=live.calibration_contract(_scope.arm))
    return result


preview_owner = SimpleNamespace(**dict(vars(previous.preview_owner), configuration_report=configuration_report))


def segment_diagnostics(states, motion, decisions, receipts):
    # HH_261002 - Fixed arc bands preserve every native row; local pose curvature is geometry evidence, not an inferred internal physics cause.
    entry, exit_arc = motion["original_XY_turn_entry_m"], motion["original_XY_turn_exit_m"]
    centers = dict(entry=entry,middle=(entry+exit_arc)/2.,exit=exit_arc)
    decision_by_ACK = {r["ack_receipt_sequence"]:r for r in decisions}
    by_frame = {r["frame"]:i for i,r in enumerate(states)}
    bins = {name:[] for name in centers}
    for row in motion["all_driving_rows"]:
        i = by_frame[row["frame"]]; current = states[i]
        command = decision_by_ACK.get(row["prior_ACK_sequence"])
        actual = None
        if 0 < i < len(states)-1:
            a,b,c = states[i-1:i+2]; ab=math.hypot(b["x"]-a["x"],b["y"]-a["y"]);bc=math.hypot(c["x"]-b["x"],c["y"]-b["y"]);ac=math.hypot(c["x"]-a["x"],c["y"]-a["y"])
            if ab*bc*ac>0:actual=2*((b["x"]-a["x"])*(c["y"]-b["y"])-(b["y"]-a["y"])*(c["x"]-b["x"]))/ (ab*bc*ac)
        pursuit = None if command is None else command.get("control",{}).get("pursuit_xy_base_m")
        radius2 = None if pursuit is None else pursuit[0]**2+pursuit[1]**2
        predicted = None if not row["actual_prior_learned_ACK_effect"] or radius2 is None or radius2 == 0. else 2*pursuit[1]/radius2
        for name,arc in centers.items():
            if abs(row["unclamped_rear_XY_progress_m"]-arc)<=2.:
                bins[name].append(dict(row,actual_rear_three_pose_signed_curvature_1pm=actual,
                    prior_learned_pursuit_curvature_1pm=predicted,actual_rear_speed_mps=math.hypot(current["vx"],current["vy"])))
    return dict(status="DIAGNOSTIC_ONLY_NOT_CAUSAL_CONTROLLER_PROOF",arc_band_halfwidth_m=2.,fixed_arc_centers_m=centers,
        band_membership_may_overlap=True,all_in_band_rows=bins,source_native_denominator=len(states),
        stationary_undefined_curvature_not_replaced_by_zero=True,not_an_additional_PASS_threshold=True)


def diagnostic_report(output,plan,owner):
    result = _diagnostic(output,plan,owner)
    result["lateral_lookahead_calibration"] = live.calibration_contract(plan["arm"])
    if "actual_LEFT_motion" in result:
        episode = output/("episode" if (output/"episode").is_dir() else "episode.partial")
        result["fixed_turn_segment_diagnostics"] = segment_diagnostics(jsonl(episode/"states.jsonl"),result["actual_LEFT_motion"],
            jsonl(episode/"learned_control.jsonl"),jsonl(episode/"control_receipts.jsonl"))
        closure_check(output,plan)
        require(all(digest(p)==pin for p,pin in result["raw_source_sha256"].items()),"raw inputs changed during segment diagnostics")
    return result


del _name, _value
if __name__ == "__main__":raise SystemExit(main())
