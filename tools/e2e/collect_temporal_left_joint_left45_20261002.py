"""HH_261002 - Prepared actual LEFT rolling comparison; never launch without a separate owned plan.

The trained endpoint, original GOOD parent and callback sources stay immutable.
Startup is a disclosed teacher; only actual learned ACK effects are NN evidence.
"""
from __future__ import annotations

import ast
import copy
from contextlib import contextmanager, ExitStack
import hashlib
import inspect
import json
import math
from pathlib import Path
from types import FunctionType, SimpleNamespace

from portable_e2e import temporal_left_joint_numeric_loader_20261002 as loader
from . import collect_temporal_route_joint_preview05_signed45_20261002 as preview
from . import collect_temporal_v2_headroom_research_20261001 as headroom
from . import collect_town01_low_speed_left45_v2_20261002 as teacher
from . import collect_temporal_intrinsic_town03_right_20261002 as recorder_proof

ROOT, collector, frozen = teacher.task.ROOT, teacher.base, headroom.owned.frozen
base = collector
task, manual, signals = teacher.task, teacher.manual, teacher.signals
SCHEMA = "portable_e2e.actual_left_joint_left45.20261002.v1"
OWNER_SCHEMA = SCHEMA + ".owned"
PREVIEW_SCHEMA, THREAD_SCHEMA = preview.PREVIEW_SCHEMA, preview.THREAD_SCHEMA
ARMS = ("parent6000", "LEFT20000")
ROUTE, ROUTE_SHA, SEED = task.ROUTE, task.ROUTE_SHA, task.CASE["seed"]
NAMES = ("_blank_brake_control", "checked_agent_initialization_observation", "apply_spawn_z_offset",
         "AcknowledgedControlTransport", "exact_camera_bundle", "_save_jpeg")
_active, _session, _owner = False, None, None
_capture = collector.collect_episode
_configuration = frozen.original.actor_controller_config
_tracker = preview.runtime.trajectory_control
_owned_session = headroom.owned.OwnedCameraTemporalLiveResearchSessionV2
_validator = frozen.validate_capture


def require(condition, message): frozen._require(condition, message)


def source_contract():
    return dict(schema=SCHEMA, arms=list(ARMS), original_TRAIN_route_sha256=ROUTE_SHA, seed=SEED,
        state_count=1055, camera_count=528, ACK_count=1060, JPEG_count=3168,
        stationary_warmup_native_ticks=24, driving_native_ticks=900, diagnostic_coast_ticks=130,
        phase_native_state_counts=dict(stationary_warmup=24,driving=901,stationary_tail=130),
        NN_effect_before_turn_entry_required=True, minimum_left_yaw_deg=60., planned_yaw_error_max_deg=15.,
        exit_plus_route_progress_m=10., stable_seconds=3., stable_CTE_m=.35, stable_heading_deg=5.,
        actor_scalar_dvdt_mps2=[-6.,3.], actual_rear_speed_max_mps=2.5, actor_speed_max_mps=2.7,
        body_inset_m=.1, policy_deadline_s=.1, observed_bundle_to_ACK_deadline_s=.1,
        minimum_post_handoff_progress_m=90., whole_motion_CTE_max_m=1., whole_motion_CTE_p95_m=.35,
        CPU_threads=4, CPU_interop=1, PI_target_cap_mps=2.4, speed_preview_s=.5,
        current_GO_support_m=18., HOLD_STOP_or_full_route_claim=False, independent_evaluation=False,
        final_driving_support_boundary_is_original_nonlearned_zero_pedal_context=True,
        source_route_was_used_for_training=True, model_promoted=False, vehicle_control_approved=False,
        CLI_zero_means_owned_complete_capture_only_not_driving_PASS=True,
        sensor_callback_entry_observed=False, exposure_to_ACK_wall_latency_proven=False)


def validate_capture(args, route):
    require(args.host=="127.0.0.1" and args.port==2440 and not args.allow_map_load
        and args.control_transport=="acknowledged_batch" and args.agent_initialization=="after_bootstrap" and args.goal_stop_profile=="disabled"
        and args.physics_hz==20 and args.capture_hz==10 and args.stationary_warmup_sec==1.2
        and args.stationary_tail_sec==6.5 and args.max_duration_sec==60. and args.target_speed_kmh==9
        and args.weather=="ClearNoon" and type(args.seed) is int and args.seed==SEED and 0<=args.seed<2**32
        and args.spawn_z_offset_m==.5 and args.research_speed_cap_mps==2.5 and args.research_max_cte_m==1.
        and args.research_longitudinal_pi is True and loader.is_sha(args.research_source_sha256)
        and Path(args.route_file).resolve()==ROUTE and teacher.task.sha(ROUTE)==ROUTE_SHA
        and route["town"]=="Town01" and route["scenario"]=="left",
        "fixed original TRAIN LEFT45/native/manual/policy contract differs")


def verify_model(model, args):
    scope=None if _owner is None else _owner._scope
    require(scope is not None and scope.arm in ARMS, "explicit private comparison arm required")
    binding=getattr(model,"_research_recovery_owner_binding",{})
    require(binding.get("purpose")=="owned_carla_research_only" and binding.get("owner_plan_sha256")==args.research_source_sha256
        and binding.get("mode")=="RESEARCH_ONLY" and binding.get("simulation_only") is True
        and binding.get("simulator_control_authorized") is True
        and getattr(model,"_research_recovery_simulator_control_authorized",False) is True
        and binding.get("model_promoted") is False and binding.get("vehicle_control_approved") is False,
        "numeric model lacks this exact prospective private owner plan")
    if scope.arm=="LEFT20000":
        loader.verify_scope(model._research_temporal_export_metadata)
        require(model._research_recovery_numeric_loader_schema==loader.SCHEMA and binding.get("schema")==loader.OWNER_SCHEMA
            and binding.get("fixed_endpoint_updates")==20000
            and binding.get("completed_run_authority_sha256")==scope.authority_sha,
            "new actual LEFT20k schema/authority required, not a6000 schema alias")
    else:
        require(model._research_recovery_numeric_loader_schema==_owner.parent_loader_schema
            and binding.get("schema")==_owner.parent_owner_schema and binding.get("fixed_endpoint_updates")==6000
            and model._research_temporal_export_metadata["optimizer_updates"]==6000,
            "original GOOD6000 loader/identity must remain unchanged")
    require(model._research_temporal_checkpoint_sha256==scope.pins["checkpoint_sha256"]
        and model._research_temporal_export_npz_sha256==scope.pins["weights_sha256"]
        and model._research_temporal_metadata_sha256==scope.pins["metadata_sha256"]
        and model._research_cpu_thread_numerical_revision==THREAD_SCHEMA
        and model._research_speed_preview_calibration==PREVIEW_SCHEMA, "predeclared model bytes/4thread/.5 calibration differ")


def task_namespace(owner):
    # HH_261002 - Exact frozen rolling clock/history/native law in a NEW diagnostic task namespace, never a new data admission.
    namespace=dict(vars(task),SCHEMA=SCHEMA+".observation_task",__file__=__file__,paths=owner.paths)
    def fixed(run_id, case_id):
        result=task.fixed_contract("v2","pilot_00")
        output,plan=owner.paths(run_id,case_id)
        result.update(source_contract())
        result.update(schema=namespace["SCHEMA"],run_id=run_id,case_id=case_id,output=str(output/"episode"),
            declaration_path=str(plan),teacher="Disclosed startup/hazards only; actual learned ACKs govern after handoff.",
            dataset_training_scope="NN_DIAGNOSTIC_NOT_AUTHORIZED_FOR_TRAIN",future_qualification_required=False,
            true_future_point_count=0,selected_windows_are_diagnostic_only=True,model_promoted=False,
            training_data_approved=False,vehicle_control_approved=False)
        # HH_261002 - The old teacher controller law remains solely the startup/unchanged hazard proposal law, not NN predictions.
        result["startup_controller"]=result.pop("controller");result["schema"]=namespace["SCHEMA"]
        return owner.decorate_task_contract(result)
    namespace["fixed_contract"]=fixed
    namespace["configuration_from_args"]=FunctionType(task.configuration_from_args.__code__,namespace)
    node=copy.deepcopy(task._node)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),__file__,"exec"),namespace)
    for name in ("reconstruct",): namespace[name]=FunctionType(getattr(task,name).__code__,namespace)
    return SimpleNamespace(**namespace)


class DiagnosticCoastSession(frozen.TemporalLiveResearchSessionV2):
    # HH_261002 - Preserve the original explicit support-boundary authority BEFORE decoding; this is not a learned STOP or a rewritten NN target.
    def propose(self,transport,teacher,reason):
        if reason=="next_driving_control" and self.states and self.states[-1].get("moving_task",{}).get("future_support_complete") is True:
            row=self.states[-1];record=row["moving_task"]
            require(row["capture_phase"]=="driving" and record.get("termination_reason")=="recovery_future_support_complete"
                and record.get("driving_relative_native_frames")==900,"only the exact declared final support observation may enter diagnostic coast")
            require(collector.control_dict(teacher)==dict(throttle=0.,brake=0.,steer=0.,gear=1,manual_gear_shift=True,reverse=False,hand_brake=False),
                "original support boundary must be exact zero-pedal manual1, not a model target")
            return teacher,dict(schema=SCHEMA,control_source="none",diagnostic_control_source="diagnostic_coast_context",learned_control_proposed=False,
                source_checkpoint_sha256=self.checkpoint_sha256,command_frame=row["frame"],inference_frame=None,
                control=collector.control_dict(teacher),diagnostic_context_only=True,learned_STOP_claimed=False)
        return super().propose(transport,teacher,reason)


class ObservedSession(preview.Preview05ObservedSession,DiagnosticCoastSession):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        require(_session is not None and _session.get("policy_session") is None,"one real causal session per capture required")
        _session["policy_session"]=self;self.actual_bundle_count=0
    def observe_bundle(self,*args,**kwargs):
        result=super().observe_bundle(*args,**kwargs);self.actual_bundle_count+=1;return result


def capture_ast():
    # HH_261002 - Exact three pre-reviewed LEFT setup/coast/closing insertions; source code is never edited.
    namespace=dict(vars(teacher),definition=teacher.definition)
    return FunctionType(teacher.capture_ast.__code__,namespace)()


def owned(args):
    require(_owner is not None,"fixture requires exact current private worker")
    plan=_owner.verify_case(args.recovery_run_id,args.recovery_case_id,args.moving_task_plan_sha256)
    _owner.base.assert_owned_worker(dict(plan,output=plan["owner_output"]),args.moving_task_plan_sha256)


@contextmanager
def parsing_scope(owner):
    original=collector.moving_capture_module; moving=task_namespace(owner)
    collector.moving_capture_module=lambda args:moving
    try:yield moving
    finally:collector.moving_capture_module=original


def parse_args(argv, owner):
    import argparse
    parser=argparse.ArgumentParser(add_help=False,allow_abbrev=False)
    parser.add_argument("--recovery-run-id",required=True);parser.add_argument("--recovery-case-id",required=True)
    ours,remaining=parser.parse_known_args(argv)
    with parsing_scope(owner) as moving:
        original=moving.configuration_from_args
        def attach(args):
            args.recovery_run_id,args.recovery_case_id=ours.recovery_run_id,ours.recovery_case_id
            return original(args)
        moving.configuration_from_args=attach
        args=frozen.parse_args(remaining)
    args.recovery_run_id,args.recovery_case_id=ours.recovery_run_id,ours.recovery_case_id
    moving.configuration_from_args=original;original(args);return args


@contextmanager
def activated(model,args,owner):
    global _active,_session,_owner
    require(not _active and _session is None and _owner is None and not preview._active
        and not headroom._active and frozen.validate_capture is _validator
        and frozen.original.actor_controller_config is _configuration and preview.runtime.trajectory_control is _tracker
        and headroom.owned.OwnedCameraTemporalLiveResearchSessionV2 is _owned_session,
        "unknown/nested learned LEFT callbacks/configuration")
    teacher.verify_frozen();preview.verify_frozen()
    _owner=owner
    try:verify_model(model,args);validate_capture(args,collector.load_route(ROUTE));owned(args)
    except BaseException:_owner=None;raise
    moving=task_namespace(owner);stack=ExitStack()
    names=(*NAMES,"collect_episode","acknowledged_snapshot_state","moving_capture_module")
    saved={name:getattr(collector,name) for name in names}
    _active=True;_session=dict(policy_session=None,states=[],cameras=[],fixture=None,manifest=None,partial=None)
    def blank(carla):
        control=saved["_blank_brake_control"](carla);control.gear,control.manual_gear_shift=1,True;return control
    def phase_control(carla,phase,record):
        control=blank(carla)
        if record is not None and record.get("termination_reason") in (None,"recovery_future_support_complete") \
                and (phase=="stationary_tail" or record.get("future_support_complete") is True):
            control.throttle,control.brake,control.steer=0.,0.,0.
        return control
    def phase_blank(carla):
        rows=_session["states"];row=rows[-1] if rows else {}
        return phase_control(carla,row.get("capture_phase"),row.get("moving_task"))
    def snapshot(*p,**kw):
        state,raw=saved["acknowledged_snapshot_state"](*p,**kw);_session["latest"]=(state,raw)
        if _session["fixture"] is not None:_session["fixture"].check()
        return state,raw
    class GuardedTask(moving.MovingTask):
        def observe(self,**kw):
            state,raw=_session["latest"];used=sum(p.stat().st_size for p in _session["partial"].rglob("*") if p.is_file())
            if "projector" not in _session:_session["projector"]=collector.RouteProjector(collector.load_route(ROUTE))
            projector=_session["projector"]
            return super().observe(**kw,native_actor_speed_mps=math.hypot(*raw["world_velocity_carla"][:2]),
                rear_base_CTE_m=projector.project(state["x"],state["y"],kw["progress"]).cross_track_error_m,observed_output_bytes=used)
    moving.MovingTask=GuardedTask
    def fixture(world,parsed,manifest,partial,green):
        value=stack.enter_context(signals.owned_fixture(world,parsed,manifest,green,owned));_session["fixture"]=value
        value.install();value.record.update(teacher_only=False,NN_control_performance_claimed=False,
            fixture_is_environment_only_not_NN_or_traffic_rule_evidence=True)
        manifest["capture_contract"]["owned_green_signal_fixture"]=value.record
    def save_pixels(image,path,quality):
        require(len(image.raw_data)==image.width*image.height*4,"exact original BGRA camera payload required")
        raw_sha=hashlib.sha256(image.raw_data).hexdigest();saved["_save_jpeg"](image,path,quality)
        record=dict(camera=path.parent.name,frame=int(image.frame),timestamp=float(image.timestamp),width=int(image.width),height=int(image.height),
            encoding="original_carla_bgra8",original_BGRA_sha256=raw_sha,saved_JPEG_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            saved_JPEG_relative_path=str(path.relative_to(_session["partial"])),JPEG_encoder_unchanged=True,pose_or_image_post_transformed=False)
        with (_session["partial"]/"actual_camera_pixels.jsonl").open("a") as stream:stream.write(json.dumps(record,allow_nan=False)+"\n")
        matches=[c for c in _session["cameras"] if c["frame"]==image.frame]
        require(len(matches)==1,"actual camera row must precede its after-ACK JPEG flush")
        for key,value in (("original_BGRA_source_sha256",raw_sha),("saved_JPEG_sha256",record["saved_JPEG_sha256"])):
            matches[0].setdefault(key,{})[record["camera"]]=value
    def capture(*p,**kw):
        _session.update(partial=p[3],states=p[4],cameras=p[5],manifest=p[6])
        recorder=recorder_proof._recorder()
        proof=dict(schema=SCHEMA+".actual_recording_binding",copied_provider_names=list(NAMES),all_copied_mutable_providers_current=True,
            enqueue_and_ACK_share_exact_recorder=True,sensor_callback_entry_observed=False,exposure_to_ACK_wall_latency_proven=False)
        transport=collector.AcknowledgedControlTransport
        class CheckedTransport(transport):
            def send(self,control,reason,expected_before_frame=None):
                if reason in ("exception_cleanup_abort","initial_governor_failure_abort","alignment_failure_abort"):
                    control=blank(self.carla)
                session=_session["policy_session"]
                if reason=="initial_drive_control":
                    require(session is not None and session.actual_bundle_count==12 and len(session.states)==24
                        and all(s["capture_phase"]=="stationary_warmup" for s in session.states)
                        and len(session.history)==len(session.observed_frames)==10
                        and session.bundle_frame==session.states[-1]["frame"]-1,
                        "all12 actual warmup bundles/history must reach the learned session before first driving command")
                    proof["warmup_session_guard_passed"]=True
                receipt=super().send(control,reason,expected_before_frame)
                if reason in ("initial_drive_control","next_driving_control"):
                    row=session.wall_timing_records[-1] if session is not None and session.wall_timing_records else {}
                    require(row.get("ACK_sequence")==receipt["sequence"] and row.get("ACK_server_accepted") is True
                        and row.get("command_frame")==session.states[-1]["frame"]
                        and (row.get("control_source") in ("expert_startup","learned_trajectory_tracker","emergency_brake")
                            or row.get("control_source")=="none" and session.states[-1].get("moving_task",{}).get("future_support_complete") is True),
                        "actual policy proposal/wall/ACK hooks must precede each driving physics tick")
                    if reason=="initial_drive_control":
                        require(session.policy is not None and session.counts["expert_startup_commands"]>=1,
                            "actual initial-drive startup proposal was not executed")
                        proof.update(initial_drive_proposal_ACK_bound=True,initial_drive_ACK_sequence=receipt["sequence"],
                            initial_drive_prefill_count=session.counts["prefill_count"],previous_camera_not_relabelled_fresh=True)
                    elif not proof.get("first_fresh_driving_proposal_checked") and session.bundle_frame==session.states[-1]["frame"]:
                        require(session.counts["prefill_count"]>=1 or session.latched_emergency is not None,
                            "first fresh proposal reached neither genuine prefill nor recorded emergency")
                        proof.update(first_fresh_driving_proposal_checked=True,first_fresh_driving_prefill_count=session.counts["prefill_count"],
                            first_fresh_driving_original_emergency=session.latched_emergency)
                return receipt
        collector.AcknowledgedControlTransport=CheckedTransport
        namespace=dict(vars(collector),install_green_fixture=fixture,manual_full_brake=blank,phase_control=phase_control)
        require(all(namespace[name] is getattr(collector,name) for name in NAMES)
            and any(cls.__name__=="ResearchTransport" for cls in transport.__mro__),
            "actual six providers/learned transport must be late-bound AFTER deferred recording")
        p[6]["capture_contract"]["learned_LEFT45_protocol"]=moving.fixed_contract(args.recovery_run_id,args.recovery_case_id)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[capture_ast()],type_ignores=[])),__file__,"exec"),namespace)
        try:return namespace["collect_episode"](*p,**kw)
        finally:
            collector.AcknowledgedControlTransport=transport
            stack.close()
            if _session["fixture"] is not None:frozen._write_new(p[3]/"owned_green_signal_fixture.json",_session["fixture"].record)
            proof.update(actual_recording_bundle_count=len(recorder.events),pending_source_camera_count=len(recorder.pending),
                actual_after_ACK_receipt_count=sum(row["acknowledged_before_recording"] for row in recorder.events))
            frozen._write_new(p[3]/"actual_recording_binding.json",proof)
    try:
        # HH_261002 - Reuse only the original Teacher class; NOT its teacher-only transport/journal or pixel-writing scope.
        from agents.navigation import basic_agent
        original_agent=basic_agent.BasicAgent
        teacher_namespace=dict(vars(teacher.parent),base=collector,task=moving,v2=SimpleNamespace(CombinedControl=manual.FirstGearRecoveryLaunchControl),
            TargetRamp=teacher.parent.TargetRamp,session=_session,original_agent=original_agent)
        original_scope=next(n for n in ast.parse(Path(teacher.parent.__file__).read_bytes()).body if isinstance(n,ast.FunctionDef) and n.name=="scope")
        agent=next(n for n in original_scope.body if isinstance(n,ast.ClassDef) and n.name=="Teacher")
        exec(compile(ast.fix_missing_locations(ast.Module(body=[copy.deepcopy(agent)],type_ignores=[])),__file__,"exec"),teacher_namespace)
        class StartupTeacher(teacher_namespace["Teacher"]):
            def __init__(self,*positional,**kwargs):
                super().__init__(*positional,**kwargs)
                _session["manifest"]["capture_contract"]["teacher_controller"].update(normal_launch_controls_inherited_v2=False,
                    normal_launch_throttle=.11,post_handoff_throttle_ramp_per_second=.025,manual_gear_shift=True,gear=1,
                    controller_is_disclosed_startup_or_original_hazard_proposal_not_NN=True)
        basic_agent.BasicAgent=StartupTeacher
        stack.callback(setattr,basic_agent,"BasicAgent",original_agent)
        collector.collect_episode,collector.acknowledged_snapshot_state=capture,snapshot
        collector._blank_brake_control,collector.checked_agent_initialization_observation=phase_blank,manual.checked_first_gear_initialization_observation
        collector._save_jpeg=save_pixels;collector.moving_capture_module=lambda parsed:moving
        frozen.validate_capture=validate_capture;headroom.owned.OwnedCameraTemporalLiveResearchSessionV2=ObservedSession
        frozen.original.actor_controller_config=preview.actor_controller_config;preview.runtime.trajectory_control=preview.trajectory_control
        preview._active=True
        with headroom.activated(model,args):yield
    finally:
        try:stack.close()
        finally:
            for name,value in saved.items():setattr(collector,name,value)
            frozen.validate_capture=_validator;headroom.owned.OwnedCameraTemporalLiveResearchSessionV2=_owned_session
            frozen.original.actor_controller_config=_configuration;preview.runtime.trajectory_control=_tracker;preview._active=False
            _active,_session,_owner=False,None,None
