"""HH_261002 - Minimal guarded AST revision of frozen teacher capture; actual coast/HOLD requests precede untouched hazards."""

from __future__ import annotations

import argparse
import ast
from contextlib import contextmanager
import copy
import math
import sys
from types import SimpleNamespace

from scripts.e2e import collect_town07_natural_goal_stop_20261002 as previous
from scripts.e2e import town07_natural_goal_stop_brakefree_v3_20261002 as task

base, manual = previous.base, previous.manual
native_float32 = previous.native_float32
_active = False


class BrakeFreeLaunchCoastHoldControl(previous.NaturalGoalLaunchControl):
    """HH_261002 - Normal coast never re-enters throttle; service brake1 requires an authentic prior low-speed goal observation."""
    def apply(self,control,*,goal_record,**kwargs):
        task.require(float(control.brake)==0. and 0<=float(control.throttle)<=native_float32(.20), "NEW zero normal brake/.20 throttle cap must precede the phase hook")
        frame,time,speed = kwargs["frame"],kwargs["timestamp"],kwargs["speed"]
        # HH_261002 - Only the existing kph-to-m/s roundtrip needs arithmetic tolerance; measured source timing and gates stay exact.
        task.require(goal_record["source_frame"] == frame and goal_record["source_timestamp"] == time
            and abs(goal_record["target_speed_mps"]-kwargs["target_speed_mps"])<=1e-12 and abs(goal_record["measured_speed_mps"]-speed)<=1e-12,
            "normal mission must use the exact current rear-base governor source")
        result = super().apply(control,**kwargs)
        phase = goal_record["terminal_mission_phase"]
        task.require(phase in ("GO","BRAKE_TO_STOP","HOLD"), "known causal mission phase required")
        if phase in ("BRAKE_TO_STOP","HOLD"):
            task.require(goal_record["coast_source_measurements"] is not None and kwargs["target_speed_mps"]==0., "coast target/latch cannot be silently reset")
            result.throttle,result.brake = 0.,0.
        if phase == "HOLD":
            source = goal_record["low_speed_service_hold_source"]
            task.require(source is not None and source["frame"]<=frame and source["timestamp"]<=time
                and source["remaining_route_arc_m"]>1e-6 and source["remaining_route_arc_m"]<=1.
                and source["planar_goal_distance_m"]<=1. and source["terminal_overshoot_m"]<=0.
                and 0<=source["measured_speed_mps"]<=.05, "service brake1 cannot precede measured upstream goal/near-stop")
            result.throttle,result.brake = 0.,1.
        self.last.update(schema=task.SCHEMA+".normal_control",terminal_mission_phase=phase,
            actual_governor_observation=dict(goal_record),profile_normal_control=base.control_dict(result),
            normal_braking_preserved=False,normal_cruise_and_coast_brake_cap=0.,service_hold_brake=1.,
            original_normal_pedals_unchanged=False,normal_pedals_changed_only_by_declared_mission=True,
            sameframe_actual_rear_base_speed_used=True,downstream_BasicAgent_hazard_override_untouched=True)
        return result


def runtime_provider(guarded_task):
    return SimpleNamespace(**{name:getattr(task,name) for name in ("configuration_from_args","pending_scope","reconstruct",
        "validate_original_physics","validate_applied_physics","measured_progress","canonical")},
        MovingTask=guarded_task,__file__=task.__file__)


@contextmanager
def parsing_scope():
    names = dict(moving_capture_module=lambda args:task,configuration_from_args=task.goal_configuration)
    saved = {k:getattr(base,k) for k in names}
    try:
        for k,v in names.items():setattr(base,k,v)
        yield
    finally:
        for k,v in saved.items():setattr(base,k,v)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(add_help=False,allow_abbrev=False)
    parser.add_argument("--natural-goal-run-id",required=True,choices=("v3",))
    ours,remaining = parser.parse_known_args(argv)
    original = task.configuration_from_args
    def attach(parsed):
        parsed.natural_goal_run_id = ours.natural_goal_run_id
        return original(parsed)
    task.configuration_from_args = attach
    try:
        with parsing_scope():args = base.parse_args(remaining)
    finally:task.configuration_from_args = original
    args.natural_goal_run_id = ours.natural_goal_run_id
    task.configuration_from_args(args)
    return args


def revised_scope():
    """HH_261002 - Invert an enumerated intervention and require exact AST equality with every frozen capture statement."""
    task.verify_frozen()
    tree = ast.parse(task.canonical(previous.__file__).read_bytes())
    node = next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=="scope")
    original = copy.deepcopy(node); changes = []
    def replace_attribute(target,name,value):
        # HH_261002 - Retain the original node identity so later ancestor restoration cannot detach an earlier child restoration.
        changes.append((target,name,getattr(target,name)))
        setattr(target,name,value)
    def unique(kind,predicate):
        values = [n for n in ast.walk(node) if isinstance(n,kind) and predicate(n)]
        task.require(len(values)==1,"unique frozen collector intervention anchor required")
        return values[0]
    governor = unique(ast.ClassDef,lambda n:n.name=="RecordedGovernor")
    replace_attribute(governor,"bases",[ast.Name(id="CoastToGoalGovernor",ctx=ast.Load())])
    snapshot = unique(ast.FunctionDef,lambda n:n.name=="snapshot")
    call = next(n for n in ast.walk(snapshot) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=="update")
    replace_attribute(call,"keywords",call.keywords+[ast.keyword(arg="latest_timestamp",value=ast.parse("float(p[0].timestamp.elapsed_seconds)",mode="eval").body)])
    update = next(n for n in governor.body if isinstance(n,ast.FunctionDef) and n.name=="update")
    call = next(n for n in ast.walk(update) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=="update")
    replace_attribute(call,"keywords",call.keywords+[ast.keyword(arg=k,value=ast.parse(v,mode="eval").body) for k,v in
        (("source_frame","session['latest_frame']"),("source_timestamp","session['latest_timestamp']"),("source_state","session['latest_state']"))])
    flag = unique(ast.keyword,lambda n:n.arg=="governor_math_changed")
    replace_attribute(flag,"value",ast.Constant(value=True))
    installer = unique(ast.Call,lambda n:isinstance(n.func,ast.Attribute) and n.func.attr=="install_normal_brake_cap")
    replace_attribute(installer,"args",[installer.args[0],ast.Constant(value=0.),installer.args[2]])
    proposal = unique(ast.Call,lambda n:isinstance(n.func,ast.Attribute) and isinstance(n.func.value,ast.Name) and n.func.value.id=="policy" and n.func.attr=="apply")
    speed = next(k for k in proposal.keywords if k.arg=="speed")
    replace_attribute(speed,"value",ast.parse("math.hypot(session['latest_state']['vx'],session['latest_state']['vy'])",mode="eval").body)
    replace_attribute(proposal,"keywords",proposal.keywords+[ast.keyword(arg="goal_record",value=ast.parse("session['goal']",mode="eval").body)])
    metadata = unique(ast.Call,lambda n: any(k.arg=="normal_launch_controller_revision" for k in n.keywords))
    comparison = next(k for k in metadata.keywords if k.arg=="normal_cap_readback_comparison")
    replace_attribute(comparison,"value",ast.Constant(value="Exact NEW zero cruise/coast brake cap; service brake1 after actual near-stop goal only; original emergency unchanged."))
    replace_attribute(metadata,"keywords",metadata.keywords+[ast.keyword(arg="causal_brakefree_terminal_mission",value=ast.parse("task.mission_contract()",mode="eval").body)])
    source = unique(ast.keyword,lambda n:n.arg=="control_source")
    replace_attribute(source,"value",ast.Constant(value="Disclosed_BasicAgent_manual1_1p50_latched_brakefree_coast_low_speed_hold_v3"))
    collect = unique(ast.FunctionDef,lambda n:n.name=="collect")
    replace_attribute(collect,"body",collect.body[:2]+ast.parse('(partial/"actual_hazard_overrides.jsonl").touch(exist_ok=False)').body+collect.body[2:])
    teacher = unique(ast.ClassDef,lambda n:n.name=="Teacher")
    hazard = ast.parse('''def add_emergency_stop(self,*p,**kw):
    result = super().add_emergency_stop(*p,**kw)
    native = self._vehicle.get_world().get_snapshot()
    journal("actual_hazard_overrides.jsonl",dict(schema=task.SCHEMA+".untouched_hazard",source_frame=int(native.frame),source_timestamp=float(native.timestamp.elapsed_seconds),returned_control=base.control_dict(result),original_return_unchanged=True))
    return result
''').body[0]
    replace_attribute(teacher,"body",teacher.body+[hazard])
    revised = copy.deepcopy(node)
    for target,name,value in reversed(changes):setattr(target,name,value)
    task.require(ast.dump(node,include_attributes=False)==ast.dump(original,include_attributes=False), "every unrelated frozen collector statement must remain AST-identical")
    namespace = dict(vars(previous),task=task,NaturalGoalLaunchControl=BrakeFreeLaunchCoastHoldControl,
        CoastToGoalGovernor=task.CoastToGoalGovernor,runtime_provider=runtime_provider,_active=False)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[revised],type_ignores=[])),str(__file__)+":enumerated_new_coast_scope","exec"),namespace)
    return namespace["scope"]


@contextmanager
def scope(args):
    global _active
    task.require(not _active and not previous._active,"NEW brakefree worker cannot nest or attach to another teacher")
    _active = True
    try:
        with revised_scope()(args):yield
    finally:_active = False


class ReconstructedFloat32Control(SimpleNamespace):
    """HH_261002 - Pure audit of CARLA's pedal representation; never substitute this for a live actor observation."""
    def __setattr__(self,name,value):
        if name in ("throttle","brake","steer"):value = native_float32(value)
        super().__setattr__(name,value)


def audit_mission(manifest,states,receipts,commands,updates,targets,route,hazards):
    """HH_261002 - Recompute every native mission update and normal request from actual causal state, with no learned model or future labels."""
    task.require(states and updates and commands and targets,"full mission/native/ACK/normal-target journals required")
    state_by_frame = {s["frame"]:s for s in states}
    task.require(len(state_by_frame)==len(states) and all(s["current_control"]["manual_gear_shift"] is True
        and type(s["current_control"]["gear"]) is int and s["current_control"]["gear"]==1 for s in states),"every actual native gear readback must be manual1")
    projector = base.RouteProjector(route)
    progress, geometry = 0.,{}
    for s in states:
        progress,cte,_ = task.goal.bounded_route_projection(projector.points,s["x"],s["y"],progress,1.)
        task.require(progress==s["route_progress_m"] and cte==s["route_cte_m"],"actual native route projection differs")
        geometry[s["frame"]] = (max(0.,projector.length_m-progress),math.hypot(s["x"]-projector.points[-1]["x"],s["y"]-projector.points[-1]["y"]),math.hypot(s["vx"],s["vy"]),task.goal.terminal_overshoot_m(projector.points,s["x"],s["y"]))
    governor = task.CoastToGoalGovernor(task.goal_configuration(type("Args",(),task.FIXED_ARGS)()),task.CRUISE,20.)
    by_frame, duplicates = {},0
    for i,row in enumerate(updates):
        frame = row["measured_state_frame"]; s = state_by_frame[frame]
        duplicate = i>0 and updates[i-1]["measured_state_frame"]==frame
        if duplicate:
            duplicates += 1
            task.require(s["capture_phase"]=="stationary_warmup" and s["goal_stop"].get("next_control_starts_driving") is True,"duplicate must be the original last-warmup initial engagement")
        task.require(row["schema"]==task.SCHEMA+".actual_governor_update" and row["governor_math_changed"] is True
            and row["measured_speed_clipped"] is False and row["count_hold"] is (s["capture_phase"]=="driving"),"original native phase/dwell accounting differs")
        expected = governor.update(*geometry[frame],count_hold=row["count_hold"],source_frame=frame,source_timestamp=s["timestamp"],source_state=s)
        task.require(expected==row["original_governor_update"],"actual coast/target/40interval dwell differs from independent causal replay")
        by_frame[frame] = expected
        if not row["count_hold"] and not duplicate:governor.target_speed_mps=0.
    task.require(duplicates==1 and len(updates)==len(states)+1 and set(by_frame)==set(state_by_frame),"every native update plus exactly one no-tick engagement required")
    ACKs = {r["sequence"]:r for r in receipts}
    task.require(len(ACKs)==len(receipts)==len(commands) and [r["ack_receipt"]["sequence"] for r in commands]==[r["sequence"] for r in receipts],"every actual ACK/teacher command must be retained")
    assignments = {r["source_frame"]:r for r in targets}
    expected_frames = [s["frame"] for s in states if s["capture_phase"]=="driving"]
    warmup = [s for s in states if s["capture_phase"]=="stationary_warmup"]
    task.require(warmup and len(assignments)==len(targets) and list(assignments)==[warmup[-1]["frame"]]+expected_frames,"all current native target assignments including first/final driving required")
    for frame,target in assignments.items():
        task.require(target["source_timestamp"]==state_by_frame[frame]["timestamp"] and target["target_changed_by_wrapper"] is False
            and abs(target["target_speed_mps"]-by_frame[frame]["target_speed_mps"])<=1e-12
            and abs(target["requested_target_kph"]-by_frame[frame]["target_speed_mps"]*3.6)<=1e-12,"assigned target differs from the sameframe governor")
    policy = BrakeFreeLaunchCoastHoldControl(); normal_count = coast_count = hold_count = 0
    for command in commands:
        ack = ACKs[command["ack_receipt"]["sequence"]]
        task.require(command["ack_receipt"]==ack and command["learned_control"] is False and command["teacher_control"] is True,"actual command/ACK identity differs")
        normal = command["normal_policy"]
        if normal is None:continue
        frame = normal["source_frame"]; s = state_by_frame[frame]; assignment = assignments[frame]
        task.require(frame==ack["before_frame"] and command["actual_governor_target_assignment"]==assignment,"normal request cannot use an older/future native target")
        raw = dict(normal["raw_normal_planner_proposal"])
        reconstructed = ReconstructedFloat32Control(**raw)
        reconstructed.brake = 0.; reconstructed.throttle = min(float(reconstructed.throttle),.20)
        task.require(base.control_dict(reconstructed)==normal["capped_normal_proposal"],"zero cap representation differs from actual normal planner proposal")
        policy.apply(reconstructed,goal_record=by_frame[frame],speed=geometry[frame][2],timestamp=s["timestamp"],frame=frame,
            target_speed_mps=assignment["target_speed_mps"],normal_proposal=raw)
        task.require(policy.last==normal,"original launch and NEW normal coast/HOLD control chronology differs")
        for name,value in base.control_dict(reconstructed).items():
            actual = command["final_BasicAgent_or_brake_proposal"][name]
            task.require(type(value) is type(actual) and (abs(value-actual)<=1e-6 if type(value) is float else value==actual),"normal return was superseded or altered; preserve failure, not coast success")
        task.require(command["final_BasicAgent_or_brake_proposal"]==ack["requested_control"],"final actual seven-field control differs from ACK")
        normal_count += 1; coast_count += normal["terminal_mission_phase"]=="BRAKE_TO_STOP"; hold_count += normal["terminal_mission_phase"]=="HOLD"
    task.require(normal_count==len(expected_frames) and governor.coast is not None and governor.service_hold is not None and not hazards,"complete normal denominator and hazard-free measured coast/HOLD required")
    final = [s for s in states if s["capture_phase"]=="driving"][-1]["goal_stop"]
    task.require(final["complete"] is True and final["hold_ticks"]>=40 and final["pilot_failure_reason"] is None,"no early service brake or first-low-speed success allowed")
    return dict(status="PASS_CAUSAL_MISSION_AND_NORMAL_ACK_ONLY",all_native_updates=len(updates),recorded_native_count=len(states),
        one_no_tick_engagement_update=True,all_teacher_ACK_count=len(commands),normal_policy_count=normal_count,
        normal_policy_denominator="Initial last-warmup engagement plus every driving state before the final measured-completion state; final state requests separate full brake",
        zero_pedal_coast_normal_command_count=coast_count,low_speed_service_hold_normal_command_count=hold_count,
        actual_coast_entry=governor.coast,actual_service_hold_entry=governor.service_hold,original_hazard_override_count=len(hazards),
        actual_hypot_v0_and_remaining_not_clipped=True,all20Hz_signed_and_absolute_bounds_checked_separately=True,
        future64_QA_or_TRAIN_authorization=False,model_oracle_used=False)


def main(argv=None):
    print("Private brakefree v3 worker; use run_owned_town07_natural_goal_stop_brakefree_v3_20261002 --mode execute",file=sys.stderr)
    return 1


if __name__=="__main__":raise SystemExit(main())
