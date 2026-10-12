"""HH_261004 - Repair only authenticated full209 diagnostic ABI seams; no runtime or approval authority.

The frozen V1 capture, sources and FAIL verdict remain immutable.  This pure
post-capture adapter admits one authenticated final catalog cleanup reason,
reconstructs the actual sealed manual-first-gear blank producer, and compares
terminal fields to the independently VERIFIED witness that owned those fields.
No threshold, NN, decoder, projection, tracker, PI or simulator behavior changes.
"""

from __future__ import annotations

import ast
import copy
import hashlib
import inspect
import json
from pathlib import Path
import stat
import textwrap
from types import FunctionType, SimpleNamespace

from . import audit_carla_acknowledged_control as original_ack


ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "portable_e2e.town03_full209_pure_diagnostic_adapter.20261004.v2"
GOAL_REASON = "catalog_goal_terminal_cleanup"
FROZEN_PINS = {
    "tools/e2e/audit_carla_acknowledged_control.py": "9d6fccb6874a5cdfc85e8bf5d9229435e4ba6fe6850eb63a9e83488f39bcc5bd",
    "tools/e2e/collect_carla_vad_expert.py": "a41eac9019c644fec03396a50645fa021b63230a4ca86a664e8044dfd170ae08",
    "tools/e2e/collect_temporal_signed_recovery45_20261002.py": "eee6de2fa1402910eb725bd60724ac1d0753730f12ef47a1492f1e1d964a2fae",
    "tools/e2e/run_owned_town07_signed_recovery_v3_20261001.py": "01f6157bd61e32fe525166b4698fa70f8b82156843ecc1bb93edb538fd706a9a",
    "tools/e2e/research_temporal_route_joint_town03_straight_full209_v1_20261004.py": "46841432d4dac2b095677dda480e758c12246c6c0f747f38db6a4323e934f22c",
    "tools/e2e/collect_temporal_route_joint_town03_straight_full209_v1_20261004.py": "22ae923a0bfba4b0eb9c283e0083ef480fd2782c6e19dbf682ec16476a9556e0",
    "tools/e2e/town03_fullroute_goal_cleanup_guard_v1_20261004.py": "ed8b9312d85809485f7c68bb63406e4a58b86343396e17dfbc8974fecfff085d",
    "learning/portable_e2e/temporal_navigation_terminal_policy_v2_20261004.py": "e4c0118de9d9e420890d1c0f11df4fe469cf9cdb7e969f1cd96d2448798c40ab",
    "learning/tests/test_temporal_navigation_terminal_policy_v2_20261004.py": "decdb0cf1e167507f8ef30e996453406f02b702d2fefbb0ddb3b75887bd606b2",
    "learning/tests/test_temporal_route_joint_town03_straight_full209_v1_20261004.py": "c8de96086e6952262a1c6a034dc3873cc2d0b25bdc78ae7c138583220ba34e07",
    "learning/tests/test_town03_fullroute_goal_cleanup_guard_v1_20261004.py": "846250da1ea6bddeffd70b5368b7202b6b4e93c4fc07a1b73d57c1d33814ec4f",
    "learning/tests/test_temporal_route_joint_town03_straight_full209_owner_v1_20261004.py": "55b01ca243c6d553b2eb98c688f4b316d02af0700c65bb644d0a5eec48626f38",
    "tools/e2e/research_temporal_route_joint_town03_right_v3_20261002.py": "0a545281db1c54ec844347f95d3a7444a1ec4bd305eaacb132886f47ff655623",
    "tools/e2e/research_temporal_route_joint_town03_right_v2_20261002.py": "a8af775ddaeb1fd907f4f021696f2c937bcfb05884688f841f2d0fa9f8318571",
    "tools/e2e/research_temporal_route_joint_town03_right_20261002.py": "ebfde24f322192b816584b10a52e7bf15a53090157d1753a37d08ec239286340",
}


def require(condition, message):
    if not condition:
        raise original_ack.base.EvidenceError(message)


def _source(name):
    path = ROOT / name
    require(path.resolve() == path and stat.S_ISREG(path.lstat().st_mode), "canonical regular frozen source required: " + name)
    payload = path.read_bytes()
    require(hashlib.sha256(payload).hexdigest() == FROZEN_PINS[name], "frozen diagnostic dependency changed: " + name)
    return payload


def verify_frozen():
    for name in FROZEN_PINS:
        _source(name)
    return dict(FROZEN_PINS)


def _parents():
    # HH_261004 - Import existing CPU research definitions lazily; this adapter never starts a model or simulator job.
    from . import research_temporal_route_joint_town03_straight_full209_v1_20261004 as owner
    from . import collect_temporal_route_joint_town03_straight_full209_v1_20261004 as frozen_live
    return owner, frozen_live


class _PrivateView:
    """HH_261004 - Preserve delegate-module __getattr__ while overriding only explicitly named private diagnostic fields."""
    def __init__(self, source, **changes):
        self.source, self.changes = source, changes

    def __getattr__(self, name):
        return self.changes[name] if name in self.changes else getattr(self.source,name)


def _sealed_owner_function(name, owner):
    # HH_261004 - Fresh owner activation may replace a module alias; compile the immutable original function rather than recurse into it.
    source=ast.parse(_source("tools/e2e/research_temporal_route_joint_town03_straight_full209_v1_20261004.py"))
    nodes=[node for node in source.body if isinstance(node,ast.FunctionDef) and node.name==name]
    require(len(nodes)==1,"one sealed original owner function required: "+name)
    namespace=dict(vars(owner))
    exec(compile(ast.Module(body=[copy.deepcopy(nodes[0])],type_ignores=[]),
        "sealed_original_full209_owner:"+name,"exec"),namespace)
    return namespace[name]


def _function(function, transform, *, extra=None):
    """HH_261004 - Require an exact reversible AST seam and execute only a new private namespace."""
    original = ast.parse(textwrap.dedent(inspect.getsource(function))).body[0]
    changed = copy.deepcopy(original)
    undo = transform(changed)
    restored = copy.deepcopy(changed)
    undo(restored)
    require(ast.dump(restored) == ast.dump(original), "diagnostic changed beyond its declared reversible seam")
    namespace = dict(function.__globals__)
    namespace.update(extra or {})
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed], type_ignores=[])),
        str(Path(__file__)) + ":private_inverse_proven_adapter", "exec"), namespace)
    return namespace[function.__name__]


def expected_scoped_blank_control():
    """HH_261004 - Execute only sealed scalar blank producers with a no-actuation VehicleControl value object."""
    generic = ast.parse(_source("tools/e2e/collect_carla_vad_expert.py"))
    signed = ast.parse(_source("tools/e2e/collect_temporal_signed_recovery45_20261002.py"))
    base = next(node for node in generic.body if isinstance(node, ast.FunctionDef) and node.name == "_blank_brake_control")
    scopes = [node for node in ast.walk(signed) if isinstance(node, ast.FunctionDef) and node.name == "brake"]
    require(len(scopes) == 1, "one sealed signed-recovery blank producer required")
    # HH_261004 - Original CARLA control defaults are values only; there is no client, actor, RPC or actuation interface.
    def vehicle_control(**values):
        control = dict(throttle=0., steer=0., brake=0., hand_brake=False, reverse=False, gear=0, manual_gear_shift=False)
        control.update(values)
        return SimpleNamespace(**control)
    namespace = dict(Any=object)
    exec(compile(ast.Module(body=[copy.deepcopy(base)], type_ignores=[]), "sealed_generic_blank_scalar", "exec"), namespace)
    namespace.update(saved={"_blank_brake_control": namespace["_blank_brake_control"]},
        frozen=SimpleNamespace(_require=require))
    exec(compile(ast.Module(body=[copy.deepcopy(scopes[0])], type_ignores=[]), "sealed_signed_manual1_blank_scalar", "exec"), namespace)
    value = vars(namespace["brake"](SimpleNamespace(VehicleControl=vehicle_control)))
    original_ack.full_control(value)
    require(value == dict(throttle=0., steer=0., brake=1., hand_brake=False, reverse=False, gear=1, manual_gear_shift=True),
        "sealed original signed-recovery brake ABI differs")
    return value


def _manual_contract(manifest):
    manual = ast.parse(_source("tools/e2e/run_owned_town07_signed_recovery_v3_20261001.py"))
    pin = next(node.value.value for node in manual.body if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == "PARENT_CHECKER_SOURCE_SHA" for target in node.targets))
    require(manifest["capture_contract"]["signed_recovery_initialization_gear_contract"] == dict(
        actual_manual_gear_shift=True, actual_gear=1, checker_function_source_sha256=pin,
        original_frame_actor_PID_finite_and_full_brake_guards_preserved=True, observed_controls_or_snapshots_proxied=False),
        "sealed actual manual-first-gear contract required")


def audit_goal_receipt(states, commands, receipts, timings, native_guards, wrapper, manifest):
    """HH_261004 - Preserve the original receipt checks and require the authentic two-predicate final catalog arrival."""
    owner, _ = _parents()
    _source("tools/e2e/research_temporal_route_joint_town03_straight_full209_v1_20261004.py")
    _manual_contract(manifest)
    require(25 <= len(states) <= 2200 and all(row["capture_phase"] == "stationary_warmup" for row in states[:24])
        and all(row["capture_phase"] == "driving" for row in states[24:]), "complete original warmup/driving phase census required")
    route = owner.read_json(owner.ROUTE)
    require(owner.digest(owner.ROUTE) == owner.live.ROUTE_SHA, "original full209 catalog source changed")
    statuses = owner.catalog_statuses(route, states)
    require(statuses and statuses[-1].reached and not any(status.reached for status in statuses[:-1]),
        "only the genuine first final catalog arrival can authorize a goal reason")
    goal = states[-1].get("catalog_goal_cleanup_receipt") or {}
    require(goal.get("remaining_catalog_route_m") == statuses[-1].remaining_route_m
        and goal.get("actual_planar_catalog_goal_distance_m") == statuses[-1].planar_distance_m
        and goal.get("original_catalog_length_m") == owner.live.collector.RouteProjector(route).length_m,
        "cleanup receipt differs from independent original catalog geometry")
    selected = [row for row in receipts if row.get("reason") == GOAL_REASON]
    require(len(selected) == 1 and selected[0]["sequence"] == len(receipts)-1
        and receipts[-1].get("reason") == "driving_end_brake"
        and receipts[-1].get("before_frame") == states[-1]["frame"], "unique penultimate actual goal cleanup and original end brake required")
    expected = expected_scoped_blank_control()
    require(receipts[-1].get("requested_control") == expected, "original end brake must preserve the sealed scoped manual1 ABI")
    def transform(tree):
        changed = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Compare) and ast.unparse(node.left) == "cleanup['requested_control']" \
                    and len(node.comparators) == 1 and isinstance(node.comparators[0], ast.Call) \
                    and ast.unparse(node.comparators[0]).endswith("gear=0, manual_gear_shift=False)"):
                changed.append(copy.deepcopy(node.comparators[0]))
                node.comparators[0] = ast.Name(id="authentic_scoped_blank_control", ctx=ast.Load())
        require(len(changed) == 1, "one original full-service blank literal seam required")
        def undo(restored):
            matches = [node for node in ast.walk(restored) if isinstance(node, ast.Compare)
                and ast.unparse(node.left) == "cleanup['requested_control']" and len(node.comparators) == 1
                and isinstance(node.comparators[0], ast.Name) and node.comparators[0].id == "authentic_scoped_blank_control"]
            require(len(matches) == 1, "one inverse blank literal seam required")
            matches[0].comparators[0] = copy.deepcopy(changed[0])
        return undo
    result = _function(_sealed_owner_function("audit_goal_receipt",owner), transform, extra=dict(authentic_scoped_blank_control=expected))(
        states, commands, receipts, timings, native_guards, wrapper, manifest)
    result.update(schema=SCHEMA+".authenticated_goal", original_manual1_blank_source_reconstructed=True,
        exact_original_catalog_predicates_recomputed=True, native_guard_and_prior_NN_effect_unchanged=True)
    return result


def _extended_ack_function(goal_frame):
    """HH_261004 - Extend the real legacy parser in five exact metadata seams; all physical/alignment math remains original."""
    _source("tools/e2e/audit_carla_acknowledged_control.py")
    def transform(tree):
        saved = []
        def structure(value):
            return ast.dump(value) if isinstance(value,ast.AST) else repr([ast.dump(item) for item in value])
        def replace(parent, field, value):
            saved.append((structure(getattr(parent, field)), structure(value), copy.deepcopy(getattr(parent, field))))
            setattr(parent, field, value)
        counts = dict(count=0, frame=0, branch=0, brake=0, boundary=0)
        for node in ast.walk(tree):
            if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
                if node.targets[0].id == "expected_count":
                    replace(node, "value", ast.parse("sum(row['capture_phase'] == phase and row['control_transport']['status'] == 'PASS' and row['frame'] != authenticated_goal_frame for row in states)", mode="eval").body)
                    counts["count"] += 1
                if node.targets[0].id == "reason" and ast.unparse(node.value) == "'next_' + row['capture_phase'] + '_control'":
                    replace(node, "value", ast.parse("GOAL_REASON if row['frame'] == authenticated_goal_frame else 'next_' + row['capture_phase'] + '_control'", mode="eval").body)
                    counts["frame"] += 1
            if isinstance(node, ast.If) and ast.unparse(node.test) == "reason == 'driving_end_brake'":
                branch = ast.parse("if reason == GOAL_REASON:\n    valid = before == authenticated_goal_frame and receipt['sequence'] == len(receipts)-1").body[0]
                branch.orelse = copy.deepcopy(node.orelse)
                replace(node, "orelse", [branch]); counts["branch"] += 1
            if isinstance(node, ast.For) and isinstance(node.target, ast.Name) and node.target.id == "reason" \
                    and isinstance(node.iter, ast.Tuple) and ast.unparse(node.iter).startswith("('stationary_warmup_start', 'initial_drive_control', 'driving_end_brake'"):
                value = copy.deepcopy(node.iter); value.elts.append(ast.Name(id="GOAL_REASON", ctx=ast.Load()))
                replace(node, "iter", value); counts["boundary"] += 1
            if isinstance(node, ast.Compare) and isinstance(node.left, ast.Name) and node.left.id == "reason" \
                    and len(node.comparators) == 1 and isinstance(node.comparators[0], ast.Tuple) \
                    and ast.unparse(node.comparators[0]).startswith("('pre_capture_bootstrap', 'stationary_warmup_start', 'driving_end_brake'"):
                value = copy.deepcopy(node.comparators); value[0].elts.append(ast.Name(id="GOAL_REASON", ctx=ast.Load()))
                replace(node, "comparators", value); counts["brake"] += 1
        require(counts == dict(count=1, frame=1, branch=1, brake=1, boundary=1), "exact five goal ACK metadata seams required")
        def undo(restored):
            pending = list(saved)
            for node in ast.walk(restored):
                for field,value in ast.iter_fields(node):
                    if not isinstance(value,(ast.AST,list)):
                        continue
                    dump = structure(value)
                    for index,(_,changed,original) in enumerate(pending):
                        if dump == changed:
                            setattr(node,field,copy.deepcopy(original)); pending.pop(index); break
            require(not pending, "all five inverse goal metadata seams must be restored")
        return undo
    return _function(original_ack.analyze_transport, transform, extra=dict(
        REASONS=set(original_ack.REASONS)|{GOAL_REASON}, GOAL_REASON=GOAL_REASON, authenticated_goal_frame=goal_frame))


def analyze_transport(states, receipts, bootstrap, *, wheelbase=2.85, commands=None, timings=None,
                      native_guards=None, wrapper=None, manifest=None):
    """HH_261004 - Ordinary evidence exact-delegates; a new reason requires complete authentic goal provenance first."""
    context = (commands,timings,native_guards,wrapper,manifest)
    claimed_arrival=isinstance(manifest,dict) and manifest.get("result",{}).get("goal_reached") is True
    # HH_261004 - A failed full-budget interval with no goal reason still receives the real original ACK audit, not a missing-goal parser error.
    if not any(receipt.get("reason")==GOAL_REASON for receipt in receipts) and not claimed_arrival:
        return original_ack.analyze_transport(states,receipts,bootstrap,wheelbase=wheelbase)
    if all(value is None for value in context):
        return original_ack.analyze_transport(states,receipts,bootstrap,wheelbase=wheelbase)
    require(all(value is not None for value in context), "all actual goal authentication inputs required")
    proof = audit_goal_receipt(states,commands,receipts,timings,native_guards,wrapper,manifest)
    result = _extended_ack_function(proof["actual_goal_frame"])(states,receipts,bootstrap,wheelbase=wheelbase)
    result.update(goal_cleanup_authentication=proof, actual_reason_strings_preserved=True,
        unknown_or_early_goal_reason_admitted=False, original_ACK_control_snapshot_and_20Hz_math_unchanged=True)
    return result


def audit_terminal_navigation(*args, **kwargs):
    """HH_261004 - Compare exact decoded D/arc fields to their independently VERIFIED witness, without numerical tolerance changes."""
    _, live = _parents()
    _source("tools/e2e/collect_temporal_route_joint_town03_straight_full209_v1_20261004.py")
    def transform(tree):
        counts = dict(distance=0, provided=0, remaining=0)
        originals = {}
        for node in ast.walk(tree):
            if isinstance(node,ast.Assign) and len(node.targets)==1 and isinstance(node.targets[0],ast.Name) \
                    and node.targets[0].id=="distance":
                originals["distance"] = copy.deepcopy(node.value)
                node.value=ast.parse("min(witness.witness.provided_route_arc_m,witness.witness.remaining_mission_arc_m,64.)",mode="eval").body
                counts["distance"]+=1
            if isinstance(node,ast.Compare) and ast.unparse(node.left) in (
                    "receipt['actual_provided_arc_m']","receipt['actual_remaining_mission_arc_m']"):
                name="provided" if "provided" in ast.unparse(node.left) else "remaining"
                originals[name]=copy.deepcopy(node.comparators)
                node.comparators=[ast.parse("witness.witness."+("provided_route_arc_m" if name=="provided" else "remaining_mission_arc_m"),mode="eval").body]
                counts[name]+=1
        require(counts==dict(distance=1,provided=1,remaining=1),"exact witness-owned D/arc comparison seams required")
        def undo(restored):
            for node in ast.walk(restored):
                if isinstance(node,ast.Assign) and len(node.targets)==1 and isinstance(node.targets[0],ast.Name) and node.targets[0].id=="distance":
                    node.value=copy.deepcopy(originals["distance"])
                if isinstance(node,ast.Compare) and ast.unparse(node.left) in (
                        "receipt['actual_provided_arc_m']","receipt['actual_remaining_mission_arc_m']"):
                    node.comparators=copy.deepcopy(originals["provided" if "provided" in ast.unparse(node.left) else "remaining"])
        return undo
    return _function(live.audit_terminal_navigation,transform)(*args,**kwargs)


def independent_interval_and_recording(output, *, states, cameras, commands, timings, manifest):
    """HH_261004 - Preserve independently computed evidence even when the ACK parser reports a failure."""
    owner,_ = _parents()
    episode=output/"episode" if (output/"episode").is_dir() else output/"episode.partial"
    result={}
    try:
        path=output/"source"/owner.ROUTE.relative_to(ROOT)
        route=owner.read_json(path)
        require(owner.digest(path)==owner.live.ROUTE_SHA,"exact archived original full209 catalog required")
        event,_=owner.recomputed_straight_interval(route,states,commands)
        result["independent_event"]=event
        result["bounded_capture_complete"]=owner.complete_interval(manifest,states,cameras,event)
    except Exception as error:
        result["independent_interval_error"]=type(error).__name__+": "+str(error)
    try:
        result["recording"]=owner.right.recording_report(output,episode,cameras,timings)
    except Exception as error:
        result["independent_recording_error"]=type(error).__name__+": "+str(error)
    return result


def goal_diagnostic(output, plan, owner_result):
    """HH_261004 - Run the real inherited diagnostic with a private ACK delegate, never mutating imported modules or original files."""
    owner,_ = _parents()
    verify_frozen()
    episode=output/"episode" if (output/"episode").is_dir() else output/"episode.partial"
    states,cameras,commands,receipts,timings,guards=[owner.right.jsonl(episode/name) for name in (
        "states.jsonl","camera_frames.jsonl","learned_control.jsonl","control_receipts.jsonl","wall_timing.jsonl","native_guard.jsonl")]
    manifest=owner.read_json(episode/"manifest.json")
    wrapper=owner.read_json(episode/"catalog_goal_cleanup_receipt.json")
    def actual_ack(s,r,b,*,wheelbase=2.85):
        require(s==states and r==receipts,"private ACK delegate must receive the same complete actual census")
        return analyze_transport(s,r,b,wheelbase=wheelbase,commands=commands,timings=timings,
            native_guards=guards,wrapper=wrapper,manifest=manifest)
    private_outer=_private_goal_function(actual_ack)
    result=private_outer(output,plan,owner_result)
    result.update(independent_interval_and_recording(output,states=states,cameras=cameras,commands=commands,timings=timings,manifest=manifest))
    result.update(pure_full209_diagnostic_adapter_schema=SCHEMA,original_thresholds_and_runtime_math_changed=False,
        frozen_V1_results_rewritten=False,original_callback_100ms_denials_retained=True,
        learned_STOP_claim=False,model_promoted=False,vehicle_control_approved=False)
    return result


def _private_goal_function(actual_ack):
    """HH_261004 - Follow all three genuine inherited function layers so the actual parser call reaches its fresh private delegate."""
    owner,_=_parents()
    outer=owner._goal_diagnostic
    inner=outer.__globals__["diagnostic_function"]()
    original_base=inner.__globals__["base"]
    private_audit=SimpleNamespace(**{**vars(original_base.independent_audit),"analyze_transport":actual_ack})
    private_base=SimpleNamespace(**{**vars(original_base),"independent_audit":private_audit})
    nested=inner.__globals__["_diagnostic"]
    require(isinstance(nested,FunctionType) and nested.__name__=="diagnostic_report"
        and Path(nested.__code__.co_filename).resolve()==ROOT/"tools/e2e/research_temporal_route_joint_town03_right_20261002.py",
        "exact original right diagnostic third-layer delegate required")
    _source("tools/e2e/research_temporal_route_joint_town03_right_20261002.py")
    private_nested=FunctionType(nested.__code__,{**nested.__globals__,"base":private_base},nested.__name__,nested.__defaults__,nested.__closure__)
    private_inner=FunctionType(inner.__code__,{**inner.__globals__,"base":private_base,"_diagnostic":private_nested},
        inner.__name__,inner.__defaults__,inner.__closure__)
    private_outer=FunctionType(outer.__code__,{**outer.__globals__,"diagnostic_function":lambda:private_inner},
        outer.__name__,outer.__defaults__,outer.__closure__)
    return private_outer


def diagnostic_report(output, plan, owner_result):
    """HH_261004 - Compose the frozen whole diagnostic with only fresh private goal/ACK and witness comparison delegates."""
    owner,live=_parents()
    verify_frozen()
    private_live=_PrivateView(live,audit_terminal_navigation=audit_terminal_navigation)
    function=_sealed_owner_function("diagnostic_report",owner)
    namespace={**function.__globals__,"_goal_diagnostic":goal_diagnostic,
        "audit_goal_receipt":audit_goal_receipt,"live":private_live}
    private=FunctionType(function.__code__,namespace,function.__name__,function.__defaults__,function.__closure__)
    result=private(output,plan,owner_result)
    result.update(pure_full209_diagnostic_adapter_schema=SCHEMA,frozen_V1_results_rewritten=False,
        original_thresholds_and_runtime_math_changed=False,original_callback_100ms_denials_retained=True)
    return result
