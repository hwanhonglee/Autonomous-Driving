"""HH_261002 - Bound one known-DEV learned right turn without shortening navigation or claiming goal stopping."""

from __future__ import annotations

import ast
from contextlib import contextmanager
import copy
import hashlib
import json
import math
from pathlib import Path

from . import collect_temporal_route_joint_preview05_signed45_20261002 as previous
from . import collect_temporal_v2_simulation_research_20261001 as generic
from . import collect_temporal_v2_owned_camera_research_20261001 as owned
from . import collect_temporal_signed_recovery45_v3_20261002 as signed3
from . import collect_temporal_signed_recovery45_v2_20261002 as signed2
from . import collect_temporal_signed_recovery45_20261002 as signed1

frozen, collector = generic, generic.collector
SCHEMA = "portable_e2e.actual_route_joint_preview05_town03_known_dev_right.20261002.v1"
SEED, ENTRY, EXIT = 2026102500, 150.4933624267578, 168.12592417001724
POST_DISTANCE, STABLE_SECONDS, CTE_BAND, YAW_BAND_DEG = 10., 3., .35, 5.
ROUTE_LENGTH, EXPECTED_TURN_DEG = 208.98520161018993, -91.38348388671875
ROUTE_SHA = "47ea507b9a4228fdce79ef58eff1e1d2cf149c02dd2df413ebadf53756f771ee"
PREVIEW_SCHEMA, THREAD_SCHEMA = previous.PREVIEW_SCHEMA, previous.THREAD_SCHEMA
FROZEN_SHA = {
    "tools/e2e/collect_carla_vad_expert.py": "a41eac9019c644fec03396a50645fa021b63230a4ca86a664e8044dfd170ae08",
    "tools/e2e/collect_temporal_route_joint_preview05_signed45_20261002.py": "e0b3f5aeec01ce035545ccc92dd26cba274463024486928a3ef72de099892085",
    "tools/e2e/collect_temporal_v2_simulation_research_20261001.py": "e1978a21f77b5f4c7f9bdcadfbaf8157ab1baad07c88cc83c1aecef6ea8356a0",
}
_active, _observer = False, None


def raw_project_function():
    # HH_261002 - Keep the frozen spatial search/tie-break exactly, removing only its monotone reporting clamp for event evidence.
    tree=ast.parse(Path(collector.__file__).read_bytes())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=="RouteProjector")
    original=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=="project")
    changed=copy.deepcopy(original); count=0
    for node in ast.walk(changed):
        if isinstance(node,ast.Return) and ast.unparse(node.value)=="Projection(max(previous_progress_m, best[3]), best[0], best[2])":
            node.value.args[0]=ast.parse("best[3]",mode="eval").body;count+=1
    frozen._require(count==1,"exact projection clamp anchor required")
    namespace=dict(vars(collector))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed],type_ignores=[])),str(Path(__file__))+":unclamped_event_projection","exec"),namespace)
    return namespace["project"]


def verify_frozen():
    previous.verify_frozen()
    for name, sha in FROZEN_SHA.items():
        frozen._require(hashlib.sha256((frozen.ROOT/name).read_bytes()).hexdigest() == sha,
                        "reviewed right-turn dependency changed: "+name)


def profile(seed):
    frozen._require(type(seed) is int and 0 <= seed < 2**32 and seed == SEED,
                    "exact uint32 known-DEV right-turn seed required")
    return dict(case_id="town03_known_dev_right", seed=seed,
                signed_lateral_offset_ros_m=0., signed_yaw_offset_ros_deg=0.)


def validate_capture(args, route):
    profile(args.seed)
    frozen._require(args.host == "127.0.0.1" and args.port == 2440 and not args.allow_map_load
        and args.control_transport == "acknowledged_batch" and args.goal_stop_profile == "disabled"
        and args.agent_initialization == "after_bootstrap" and args.physics_hz == 20 and args.capture_hz == 10
        and args.stationary_warmup_sec == 1.2 and args.stationary_tail_sec == 0
        and args.max_duration_sec == 110 and args.target_speed_kmh == 9 and args.weather == "ClearNoon"
        and args.spawn_z_offset_m == .5 and args.wheelbase_m == 2.85
        and args.research_speed_cap_mps == 2.5 and args.research_max_cte_m == 1.
        and args.research_longitudinal_pi is True and route["town"] == "Town03" and route["scenario"] == "right"
        and len(route["route"]) == 213 and route["route_length_m"] == ROUTE_LENGTH
        and isinstance(args.research_source_sha256, str) and len(args.research_source_sha256) == 64
        and all(c in "0123456789abcdef" for c in args.research_source_sha256),
        "fixed known-DEV Town03 right-turn110 TOTAL contract differs")
    frozen._require(Path(args.route_file).is_file() and not Path(args.route_file).is_symlink()
        and hashlib.sha256(Path(args.route_file).read_bytes()).hexdigest() == ROUTE_SHA,
        "entire original213-point navigation bytes required")


class RightTurnObserver:
    """HH_261002 - Observe prior learned ACK effects, not outgoing annotations or teacher-controlled movement."""

    def __init__(self, route):
        self.route = route; self.projector = collector.RouteProjector(route)
        self.session = None; self.previous = None; self.progress = 0.
        self.entry_frame = self.entry_yaw = self.unwrapped_yaw = self.last_yaw = None
        self.exit_frame = None; self.post_distance = 0.; self.band_start = None
        self.completed = False; self.records = []; self.invalid_after_entry = False
        self.ACK_by_sequence={};self.ACK_cursor=0
        self.project = raw_project_function()

    def observe(self, state, *, learned_effect, outgoing_safe):
        frozen._require(type(learned_effect) is bool and type(outgoing_safe) is bool,
                        "explicit actual ACK effect/source predicates required")
        if self.completed: raise collector.CollectionError("turn event observer called after completion")
        frame, stamp = state["frame"], state["timestamp"]
        values = [state[k] for k in ("x", "y", "yaw")] + [stamp]
        frozen._require(type(frame) is int and all(type(v) in (int,float) and math.isfinite(v) for v in values),
                        "finite actual native rear pose required")
        if self.previous is not None:
            frozen._require(frame == self.previous["frame"]+1 and abs(stamp-self.previous["timestamp"]-.05) <= 1.e-6,
                            "every consecutive native driving observation required")
        projection = self.project(self.projector,state["x"], state["y"], self.progress)
        prior_progress, self.progress = self.progress, projection.progress_m
        points = self.route["route"]; index = projection.segment_index
        a,b = points[index],points[index+1]
        tangent = math.atan2(b["y"]-a["y"], b["x"]-a["x"])
        yaw_error = math.atan2(math.sin(state["yaw"]-tangent), math.cos(state["yaw"]-tangent))
        safe = learned_effect and outgoing_safe and state.get("capture_phase") == "driving" \
            and not state.get("collision") and not state.get("lane_invasion")
        if self.entry_frame is None and prior_progress < ENTRY <= self.progress:
            if safe:
                self.entry_frame, self.entry_yaw = frame, state["yaw"]
                self.unwrapped_yaw = self.last_yaw = state["yaw"]
            else: self.invalid_after_entry = True
        elif self.entry_frame is not None:
            self.unwrapped_yaw += math.atan2(math.sin(state["yaw"]-self.last_yaw), math.cos(state["yaw"]-self.last_yaw))
            self.last_yaw = state["yaw"]
        if self.entry_frame is not None and not safe: self.invalid_after_entry = True
        if self.entry_frame is not None and self.progress >= EXIT:
            if self.exit_frame is None: self.exit_frame = frame
            elif self.previous is not None:
                self.post_distance += math.hypot(state["x"]-self.previous["x"], state["y"]-self.previous["y"])
        in_band = self.exit_frame is not None and self.progress >= EXIT and safe and abs(projection.cross_track_error_m) <= CTE_BAND \
            and abs(yaw_error) <= math.radians(YAW_BAND_DEG)
        if in_band:
            if self.band_start is None: self.band_start = stamp
        else: self.band_start = None
        stable = stamp-self.band_start if self.band_start is not None else 0.
        actual_turn = math.degrees(self.unwrapped_yaw-self.entry_yaw) if self.entry_yaw is not None else None
        self.completed = not self.invalid_after_entry and actual_turn is not None and actual_turn <= -80. \
            and self.progress >= EXIT+POST_DISTANCE and self.post_distance >= POST_DISTANCE \
            and stable >= STABLE_SECONDS-1.e-6
        row = dict(frame=frame,timestamp=stamp,actual_rear_progress_m=self.progress,
            actual_rear_CTE_m=projection.cross_track_error_m,actual_yaw_error_deg=math.degrees(yaw_error),
            current_prior_ACK_is_learned_effect=learned_effect,outgoing_original_safety_branch_clear=outgoing_safe,
            turn_entry_frame=self.entry_frame,actual_unwrapped_turn_deg=actual_turn,post_exit_actual_rear_XY_arc_m=self.post_distance,
            stable_postturn_seconds=stable,invalid_after_entry=self.invalid_after_entry,event_complete=self.completed)
        self.records.append(row); self.previous = dict(state)
        return self.completed

    def summary(self):
        return dict(schema=SCHEMA+".event", completed=self.completed, entry_frame=self.entry_frame, exit_frame=self.exit_frame,
            observation_count=len(self.records), last_observation=self.records[-1] if self.records else None,
            expected_route_signed_turn_deg=EXPECTED_TURN_DEG, event_is_not_catalog_goal_or_learned_STOP=True,
            cleanup_brake_is_not_model_STOP=True, split="val", training_data_approved=False)


def observed_after_tick(states):
    frozen._require(_observer is not None and _observer.session is not None and states,
                    "owner-scoped actual turn session required")
    session, state = _observer.session, states[-1]
    sequence = state.get("control_transport",{}).get("expected_receipt_sequence")
    for row in session.wall_timing_records[_observer.ACK_cursor:]:
        seq=row["ACK_sequence"]
        frozen._require(type(seq) is int and seq not in _observer.ACK_by_sequence,"unique actual ACK sequence required")
        _observer.ACK_by_sequence[seq]=row
    _observer.ACK_cursor=len(session.wall_timing_records)
    matching = _observer.ACK_by_sequence.get(sequence,{})
    learned = matching.get("ACK_server_accepted") is True and matching.get("learned_control_proposed") is True \
        and matching.get("control_source") == "learned_trajectory_tracker" and state.get("control_transport",{}).get("status") == "PASS"
    completed = _observer.observe(state,learned_effect=learned,outgoing_safe=session.latched_emergency is None)
    with (session.output/"known_dev_right_turn_observations.jsonl").open("a") as stream:
        stream.write(json.dumps(_observer.records[-1],allow_nan=False)+"\n")
    return completed


def cloned_collect_episode():
    """HH_261002 - Three explicit AST insertions only; original native/ACK/camera/abort/cleanup authority is preserved."""
    verify_frozen(); tree=ast.parse(Path(collector.__file__).read_bytes())
    original=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name == "collect_episode")
    changed=copy.deepcopy(original); counts=dict(tick=0,result=0,accept=0)
    class Event(ast.NodeTransformer):
        def visit_Assign(self,node):
            node=self.generic_visit(node)
            if ast.unparse(node) == "(goal_status, basic_agent_done, stop_reason) = tick_and_record('driving')":
                counts["tick"]+=1
                return [node,ast.parse("if stop_reason is None and observed_after_tick(state_records):\n    break").body[0]]
            if any(ast.unparse(t)=="manifest['result']" for t in node.targets):
                counts["result"]+=1
                return [node,ast.parse("manifest['result']['known_dev_right_turn_event'] = _observer.summary()").body[0]]
            return node
        def visit_If(self,node):
            node=self.generic_visit(node)
            if ast.unparse(node.test) == "not goal_status.reached and (not (moving_task is not None and manifest['result']['task_complete']))":
                node.test=ast.BoolOp(op=ast.And(),values=[node.test,ast.parse("_observer.completed",mode="eval").body])
                node.test.values[-1]=ast.UnaryOp(op=ast.Not(),operand=node.test.values[-1]);counts["accept"]+=1
            return node
    changed=Event().visit(changed)
    frozen._require(counts == dict(tick=1,result=1,accept=1),"exact original tick/result/goal-rejection AST anchors required")
    # HH_261002 - Invert all three insertions and require complete original AST equality, not a copied subset of guards.
    restored=copy.deepcopy(changed)
    class Undo(ast.NodeTransformer):
        def visit_If(self,node):
            if ast.unparse(node.test) == "stop_reason is None and observed_after_tick(state_records)":return None
            node=self.generic_visit(node)
            if isinstance(node.test,ast.BoolOp) and ast.unparse(node.test.values[-1]) == "not _observer.completed":
                node.test=node.test.values[0]
            return node
        def visit_Assign(self,node):
            if ast.unparse(node.targets[0]) == "manifest['result']['known_dev_right_turn_event']":return None
            return self.generic_visit(node)
    frozen._require(ast.dump(Undo().visit(restored))==ast.dump(original),"another collector statement changed")
    namespace=dict(vars(collector),observed_after_tick=observed_after_tick,_observer=_observer)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed],type_ignores=[])),
        str(Path(__file__))+":post_recorded_actual_effect_event","exec"),namespace)
    function=namespace["collect_episode"]
    def capture(*args,**kwargs):
        return run_event_capture(function,*args,**kwargs)
    return capture


def run_event_capture(function,*args,**kwargs):
    # HH_261002 - Write the event receipt before collector.run renames episode.partial; preserve failures before any output move.
    manifest=args[6];partial=args[3]
    manifest.setdefault("capture_contract",{})["known_DEV_right_turn_event"]=dict(
        schema=SCHEMA,dataset_split="val",known_DEV_site_reused=True,original_navigation_sha256=ROUTE_SHA,
        original_navigation_points=213,entry_original_route_arc_m=ENTRY,exit_original_route_arc_m=EXIT,
        current_unclamped_route_arc_minimum_m=EXIT+POST_DISTANCE,actual_post_exit_rear_XY_arc_minimum_m=POST_DISTANCE,
        continuous_post_exit_stable_seconds=STABLE_SECONDS,current_prior_learned_ACK_required=True,
        observation_after_original_native_camera_control_ACK_and_recording=True,
        completion_is_not_catalog_goal_or_learned_STOP=True,training_data_approved=False)
    try:return function(*args,**kwargs)
    finally:
        frozen._require(_observer is not None,"actual turn observer missing at capture close")
        frozen._write_new(partial/"known_dev_right_turn_summary.json",_observer.summary())


def clone_node(module,name,namespace,*,train_literals=0):
    original=next(n for n in ast.parse(Path(module.__file__).read_bytes()).body
        if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name == name)
    changed=copy.deepcopy(original);count=0
    class Split(ast.NodeTransformer):
        def visit_Constant(self,node):
            nonlocal count
            if node.value == "train":node.value="val";count+=1
            return node
    changed=Split().visit(changed)
    frozen._require(count==train_literals,"exact known-DEV split literal census differs")
    restored=copy.deepcopy(changed)
    class Undo(ast.NodeTransformer):
        def visit_Constant(self,node):
            if node.value=="val":node.value="train"
            return node
    frozen._require(ast.dump(Undo().visit(restored))==ast.dump(original),"non-split session AST changed")
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed],type_ignores=[])),str(Path(__file__))+":known_DEV_val","exec"),namespace)
    return namespace[name]


def known_dev_session():
    # HH_261002 - Rebuild the four-level MRO so zero-argument super follows NEW classes, not patched frozen method bodies.
    # HH_261002 - Use actual restored delegate globals: policy/config aliases change inside the inherited activation.
    causal=clone_node(generic,"TemporalLiveResearchSessionV2",vars(generic),train_literals=2)
    immutable=clone_node(owned,"OwnedCameraTemporalLiveResearchSessionV2",vars(owned))
    threads=previous.previous
    threads._original_session=immutable
    thread=clone_node(threads,"Thread4ObservedSession",vars(threads))
    # HH_261002 - The0.5 class itself is an already-verified AST transform, obtained from the source0.8 class with its0.5 globals.
    node=previous.transformed_definition("Preview08ObservedSession")
    previous._original_session=thread
    ns=vars(previous)
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(Path(__file__))+":preview05_known_DEV","exec"),ns)
    preview=ns["Preview05ObservedSession"]
    class KnownDevRightObservedSession(preview):
        def __init__(self,*args,**kwargs):
            super().__init__(*args,**kwargs)
            frozen._require(_observer is not None and _observer.session is None,"one owner-scoped observed turn session")
            _observer.session=self
    return KnownDevRightObservedSession,thread


@contextmanager
def activated(model,args):
    """HH_261002 - Restore every private alias on all exits; no frozen file, observed input, or learned output is rewritten."""
    global _active,_observer
    frozen._require(not _active and _observer is None,"known-DEV turn cannot nest")
    verify_frozen();route=collector.load_route(args.route_file);validate_capture(args,route)
    previous.previous.previous.validate_model_binding(model,args)
    observer=RightTurnObserver(route);_active=True;_observer=observer
    names={previous:("Preview05ObservedSession","_original_session"),
        previous.previous:("Thread4ObservedSession","_original_session"),
        generic:("TemporalLiveResearchSessionV2","activated"),
        owned:("OwnedCameraTemporalLiveResearchSessionV2",),
        previous.previous.previous:("validate_capture",),
        signed3:("profile","validate_capture","SCHEMA"),signed2:("profile","SCHEMA"),
        collector:("collect_episode",)}
    saved={module:{name:getattr(module,name) for name in keys} for module,keys in names.items()}
    try:
        session,thread=known_dev_session()
        changes={
        previous:dict(Preview05ObservedSession=session),
        previous.previous.previous:dict(validate_capture=validate_capture),
        signed3:dict(profile=profile,validate_capture=validate_capture,SCHEMA=SCHEMA),
        signed2:dict(profile=profile,SCHEMA=SCHEMA),
        generic:dict(activated=clone_node(generic,"activated",vars(generic),train_literals=1)),
        collector:dict(collect_episode=cloned_collect_episode()),
        }
        for module,values in changes.items():
            for name,value in values.items():setattr(module,name,value)
        with previous.activated(model,args):yield
    finally:
        for module,values in reversed(tuple(saved.items())):
            for name,value in values.items():setattr(module,name,value)
        _active=False;_observer=None
