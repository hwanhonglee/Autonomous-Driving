"""HH_261002 - Prepared fixed lateral-lookahead A/B; no new weights, selected paths or safety thresholds."""
from __future__ import annotations

import ast
import copy
from contextlib import contextmanager
from dataclasses import asdict, replace
from pathlib import Path
from types import FunctionType

from . import collect_temporal_left_joint_left45_20261002 as previous

# HH_261002 - Bind reviewed function bytecode to a private namespace, never to a frozen provider's mutable globals.
for _name, _value in vars(previous).items():
    if not _name.startswith("__"):
        globals()[_name] = _value
SCHEMA = "portable_e2e.actual_left_joint_lateral_pair.20261002.v1"
OWNER_SCHEMA = SCHEMA + ".owned"
CALIBRATION_SCHEMA = "portable_e2e.causal_selected_lateral_lookahead_pair.20261002.v1"
ARMS = ("baseline", "short_lookahead")
PROFILES = {"baseline": (1.5, .8), "short_lookahead": (1., .5)}
FROZEN_COLLECTOR_SHA = "884f146c5ad06dfe01587b68d2afb1c77e742989b1a6ebb5856398adac4689e5"
_active, _session, _owner = False, None, None
_base_tracker = previous.preview._trajectory_control


def verify_frozen():
    require(previous.hashlib.sha256(Path(previous.__file__).read_bytes()).hexdigest() == FROZEN_COLLECTOR_SHA,
        "exact frozen LEFT45 collector required")
    previous.teacher.verify_frozen(); previous.preview.verify_frozen()


def definition(name):
    verify_frozen()
    original = next(n for n in ast.parse(Path(previous.__file__).read_bytes()).body
        if isinstance(n, (ast.FunctionDef, ast.ClassDef)) and n.name == name)
    changed = copy.deepcopy(original)
    if name == "verify_model":
        hits = [n for n in ast.walk(changed) if isinstance(n, ast.Compare)
            and ast.dump(n) == ast.dump(ast.parse('scope.arm=="LEFT20000"', mode="eval").body)]
        require(len(hits) == 1, "one original endpoint branch required")
        hits[0].ops, hits[0].comparators = [ast.In()], [ast.Name(id="ARMS", ctx=ast.Load())]
    if name == "activated":
        hits = [n for n in ast.walk(changed) if isinstance(n, ast.Attribute)
            and ast.dump(n) == ast.dump(ast.parse("preview.actor_controller_config", mode="eval").body)]
        require(len(hits) == 1, "one original controller factory binding required")
        replacement = ast.Name(id="actor_controller_config", ctx=ast.Load())
        class Factory(ast.NodeTransformer):
            def visit_Attribute(self, node):
                return copy.deepcopy(replacement) if node is hits[0] else self.generic_visit(node)
        changed = Factory().visit(changed)
    # HH_261002 - Inverse AST proof isolates endpoint identity/factory dispatch; capture, transport and recorder statements remain exact.
    restored = copy.deepcopy(changed)
    class Restore(ast.NodeTransformer):
        def visit_Compare(self, node):
            if name == "verify_model" and node.lineno == hits[0].lineno and ast.dump(node) == ast.dump(ast.parse("scope.arm in ARMS", mode="eval").body):
                return ast.parse('scope.arm=="LEFT20000"', mode="eval").body
            return self.generic_visit(node)
        def visit_Name(self, node):
            if name == "activated" and node.id == "actor_controller_config":
                return ast.parse("preview.actor_controller_config", mode="eval").body
            return node
    require(ast.dump(Restore().visit(restored)) == ast.dump(original), "another frozen callback statement changed: " + name)
    return ast.fix_missing_locations(changed)


for _name in ("validate_capture", "task_namespace", "capture_ast", "owned", "parsing_scope", "parse_args",
              "DiagnosticCoastSession", "ObservedSession", "verify_model", "activated"):
    exec(compile(ast.Module(body=[definition(_name)], type_ignores=[]), __file__, "exec"), globals())
_capture_scope = activated
_observed_session = ObservedSession


class ObservedSession(_observed_session):
    def propose(self, transport, teacher, reason):
        command, decision = super().propose(transport, teacher, reason)
        if decision is not None:
            # HH_261002 - Supersede the inherited speed-only provenance, never pretend a changed lateral tracker was unchanged.
            decision.update(decoder_gate_tracker_math_unchanged_except_disclosed_headroom_and_speed_preview=False,
                decoder_gate_tracker_math_unchanged_except_disclosed_headroom_speed_preview_and_lateral=True,
                causal_lateral_lookahead_calibration=calibration_contract(_owner._scope.arm))
        return command, decision


def calibration_contract(arm):
    require(arm in ARMS, "one prospectively fixed lateral profile required")
    minimum, seconds = PROFILES[arm]
    return dict(schema=CALIBRATION_SCHEMA, arm=arm, minimum_lookahead_m=minimum, lookahead_time_s=seconds,
        original_minimum_lookahead_m=1.5, original_lookahead_time_s=.8,
        tracker_formula="max(minimum_lookahead_m,max(0,current_speed_mps)*lookahead_time_s)",
        selected_NN_trajectory_unchanged=True, NN_weights_unchanged=True, speed_preview_s=.5,
        PI_target_cap_mps=2.4, CPU_threads=4, CPU_interop=1,
        original_STOP_and_non_GO_tracker_preserved=True, original_safety_and_latency_gates_unchanged=True,
        controller_calibration_not_trained_model_improvement=True, improvement_unproven=True,
        model_promoted=False, vehicle_control_approved=False)


def source_contract():
    result = previous.source_contract()
    result.update(schema=SCHEMA, arms=list(ARMS), fixed_endpoint_updates=20000,
        both_profiles_use_same_LEFT20000_endpoint=True, matched_baseline_repeat_required=True,
        lateral_profiles={arm: calibration_contract(arm) for arm in ARMS}, outcome_selected_retry=False)
    return result


def actor_controller_config(physics, speed_cap_mps):
    require(_owner is not None and _owner._scope.arm in ARMS, "current private lateral arm required")
    original = preview.actor_controller_config(physics, speed_cap_mps)
    require(original.minimum_lookahead_m == 1.5 and original.lookahead_time_s == .8 and original.speed_preview_s == .5,
        "original controller preset changed")
    minimum, seconds = PROFILES[_owner._scope.arm]
    changed = replace(original, minimum_lookahead_m=minimum, lookahead_time_s=seconds)
    changed.validate()
    require({k:v for k,v in asdict(changed).items() if k not in ("minimum_lookahead_m", "lookahead_time_s")}
        == {k:v for k,v in asdict(original).items() if k not in ("minimum_lookahead_m", "lookahead_time_s")},
        "lateral preset changed geometry, speed or pedal configuration")
    return changed


def trajectory_control(selected, current_speed_mps, config):
    # HH_261002 - Frozen preview guards select0.3 only for STOP/non-GO; preserve the entire original lateral tracker there too.
    require(_active and _owner is not None, "exclusive lateral tracker scope required")
    expected = PROFILES[_owner._scope.arm]
    require((config.minimum_lookahead_m, config.lookahead_time_s) == expected, "actual lateral configuration drifted")
    used = replace(config, minimum_lookahead_m=1.5, lookahead_time_s=.8) if config.speed_preview_s == .3 else config
    require(config.speed_preview_s in (.3, .5), "only unchanged cruise/STOP preview supported")
    return _base_tracker(selected, current_speed_mps, used)


@contextmanager
def activated(model, args, owner):
    verify_frozen()
    require(not _active and preview._trajectory_control is _base_tracker and owner._scope.arm in ARMS,
        "unknown/nested lateral delegate")
    require(getattr(model, "_research_lateral_lookahead_calibration", None) == calibration_contract(owner._scope.arm),
        "exact prospective lateral owner binding required")
    preview._trajectory_control = trajectory_control
    try:
        with _capture_scope(model, args, owner):
            yield
    finally:
        preview._trajectory_control = _base_tracker


del _name, _value
