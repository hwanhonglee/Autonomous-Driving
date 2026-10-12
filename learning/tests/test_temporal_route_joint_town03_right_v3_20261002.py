"""HH_261002 - Exercise the actual compiled capture and original generic observed/proposal/ACK hooks with fake CARLA only."""
from collections import deque
from contextlib import contextmanager, ExitStack
from dataclasses import dataclass
import inspect
import json
from pathlib import Path
import sys
import traceback
from types import ModuleType, SimpleNamespace
from unittest import mock

import pytest

from scripts.e2e import collect_temporal_route_joint_town03_right_v3_20261002 as live
from scripts.e2e import research_temporal_route_joint_town03_right_v3_20261002 as owner


def pins(): return {key+"_sha256": str(i)*64 for i,key in enumerate(("weights","metadata","checkpoint","declaration"),1)}


@contextmanager
def owner_scope():
    with mock.patch.object(owner.numeric,"verify_exports"), mock.patch.object(owner.previous.previous.threads,"evidence_assets",return_value={}):
        with owner.activated(pins()): yield


def arguments():
    with owner_scope(): return live.frozen.parse_args(owner.base.capture_argv("v3",True,"a"*64))


def test_real_public_plan_parser_preflight_and_all_bindings_restore():
    before=(owner.base.live,owner.base.MODULE,owner.numeric.MODULE,owner.previous.previous.live,live.previous.SCHEMA)
    imported=SimpleNamespace(stdout=str(owner.base.EGG)+"/carla.so\n"+str(owner.previous.previous.CARLA_ROOT/"PythonAPI/carla/agents/__init__.py")+"\n1.8.0\n")
    with owner_scope(), ExitStack() as stack:
        contract=owner.fixed_contract("v3",True)
        assert contract["town"]=="Town03" and contract["dataset_split"]=="val" and contract["schema"]==live.OWNER_SCHEMA
        assert contract["speed_preview_s"]==.5 and contract["controller_target_cap_mps"]==2.4
        assert contract["total_capture_seconds"]==110 and contract["full_original_navigation_points"]==213
        assert contract["model_speed_cap_mps"]==2.5 and contract["policy_deadline_seconds"]==.1
        assert contract["all_12_real_warmup_bundles_required_before_first_driving_command"]
        stack.enter_context(mock.patch.object(owner.base,"digest",return_value="a"*64))
        stack.enter_context(mock.patch.object(owner.base,"runtime_sources",return_value={"code":"a"*64}))
        stack.enter_context(mock.patch.object(owner.base,"external_sources",return_value={"API":"a"*64}))
        stack.enter_context(mock.patch.object(owner.base,"ASSETS",{p:"a"*64 for p in owner.base.ASSETS}))
        plan=owner.base.build_plan("v3",True)
        args=live.frozen.parse_args(owner.base.capture_argv("v3",True,"a"*64))
        live.frozen.validate_capture(args,live.collector.load_route(args.route_file))
        stack.enter_context(mock.patch.object(owner.base,"verify_plan",return_value=plan))
        stack.enter_context(mock.patch.object(owner.base.lifecycle,"_assert_no_other_carla"))
        stack.enter_context(mock.patch.object(owner.base,"ports_free"))
        stack.enter_context(mock.patch.object(owner.base,"disk_check",return_value={}))
        stack.enter_context(mock.patch.object(owner.base.subprocess,"run",return_value=imported))
        stack.enter_context(mock.patch.object(owner.base,"load_npz",return_value=SimpleNamespace(_research_temporal_checkpoint_sha256=pins()["checkpoint_sha256"])))
        assert owner.base.preflight("v3","a"*64)[1]["status"]=="PREFLIGHT_PASS_NO_SIMULATOR"
    assert before==(owner.base.live,owner.base.MODULE,owner.numeric.MODULE,owner.previous.previous.live,live.previous.SCHEMA)


def states_fixture(terminal=False):
    states=[dict(frame=i,capture_phase="stationary_warmup") for i in range(24)]
    for i in range(24,28):
        flag=not(terminal and i==27)
        states.append(dict(frame=i,capture_phase="driving",traffic_light_observation=dict(frame=i,frame_before=i,
            frame_after=i,basic_agent_run_step_this_frame=flag)))
    manifest=dict(result=dict(goal_reached=terminal,termination_reason="catalog_goal_tolerance" if terminal else None))
    return states,manifest


@pytest.mark.parametrize("terminal",[False,True])
def test_actual_detector_census_includes_every_real_call_not_terminal_blank(terminal):
    states,manifest=states_fixture(terminal)
    assert owner.expected_hazard_frames(states,manifest)==list(range(23,27 if terminal else 28))
    assert callable(owner.diagnostic_function()) and live.RightTurnObserver is live.previous.previous.RightTurnObserver


@pytest.mark.parametrize("kind",["middle","unknown","boolean","frame","missing","unsupportedterminal"])
def test_no_unmeasured_or_arbitrary_omitted_hazard_decisions(kind):
    states,manifest=states_fixture(True)
    if kind=="middle":states[25]["traffic_light_observation"]["basic_agent_run_step_this_frame"]=False
    if kind=="unknown":states[-1]["traffic_light_observation"]["basic_agent_run_step_this_frame"]=None
    if kind=="boolean":states[-1]["traffic_light_observation"]["basic_agent_run_step_this_frame"]=0
    if kind=="frame":states[-1]["traffic_light_observation"]["frame_after"]+=1
    if kind=="missing":states[-1].pop("traffic_light_observation")
    if kind=="unsupportedterminal":manifest["result"]["termination_reason"]="somethingelse"
    with pytest.raises(Exception):owner.expected_hazard_frames(states,manifest)


class Control(SimpleNamespace):
    def __init__(self,**kwargs):
        super().__init__(throttle=kwargs.get("throttle",0.),brake=kwargs.get("brake",0.),steer=kwargs.get("steer",0.),
            hand_brake=False,reverse=False,manual_gear_shift=True,gear=1)


def fake_modules(world):
    class Actor:
        id=9
        def __init__(self):self.control=Control(brake=1.)
        def get_control(self):return self.control
        def listen(self,callback):pass
        def destroy(self):return True
    ego=Actor()
    class Client:
        def __init__(self,*args):pass
        def set_timeout(self,*args):pass
        def get_world(self):return world
        def get_server_version(self):return "fake"
        def get_client_version(self):return "fake"
    class BasicAgent:
        _ignore_traffic_lights=_ignore_vehicles=False
        def __init__(self,*args,**kwargs):self.planner=SimpleNamespace(_vehicle_controller=SimpleNamespace())
        def set_global_plan(self,plan):pass
        def get_local_planner(self):return self.planner
        def done(self):return False
        def _vehicle_obstacle_detected(self,*args):return False,None,-1.
        def _affected_by_traffic_light(self,*args):return False,None
        def run_step(self):
            self._vehicle_obstacle_detected();self._affected_by_traffic_light();return Control(throttle=.11)
    class Planner:
        def __init__(self,*args):pass
        def trace_route(self,*args):return [0,1]
    carla=ModuleType("carla");carla.Client=Client;carla.VehicleControl=Control
    carla.Transform=lambda:None;carla.AttachmentType=SimpleNamespace(Rigid=1)
    carla.WeatherParameters=SimpleNamespace(ClearNoon=1);carla.TrafficLightState=SimpleNamespace(Green=1)
    agents=ModuleType("agents");navigation=ModuleType("agents.navigation")
    basic=ModuleType("agents.navigation.basic_agent");basic.BasicAgent=BasicAgent
    planner=ModuleType("agents.navigation.global_route_planner");planner.GlobalRoutePlanner=Planner
    navigation.basic_agent=basic
    return ego,{"carla":carla,"agents":agents,"agents.navigation":navigation,
        "agents.navigation.basic_agent":basic,"agents.navigation.global_route_planner":planner}


def run_compiled_capture(tmp_path,*,drop_bundle=False,drop_proposal=False):
    # HH_261002 - Execute the real full frozen collect_episode AST, not a replacement function that only checks copied names.
    args=arguments();partial=tmp_path/"episode.partial";partial.mkdir();args.output=partial
    route=live.collector.load_route(args.route_file);states=[];cameras=[];manifest={"capture_contract":{"control_transport":{}}}
    class World:
        frame=100
        settings=SimpleNamespace(synchronous_mode=True,fixed_delta_seconds=.05,no_rendering_mode=False,
            substepping=True,max_substep_delta_time=.01,max_substeps=5)
        def get_settings(self):return SimpleNamespace(**vars(self.settings))
        def apply_settings(self,s):self.settings=s
        def get_weather(self):return 1
        def set_weather(self,w):pass
        def get_map(self):return SimpleNamespace(name="/Maps/Town03")
        def get_snapshot(self):return SimpleNamespace(frame=self.frame,timestamp=SimpleNamespace(elapsed_seconds=self.frame*.05))
        def tick(self,*args):
            self.frame+=1
            if self.frame==127:raise RuntimeError("intentional next-tick failure after real first-fresh proposal")
            return self.frame
        def get_blueprint_library(self):return SimpleNamespace(find=lambda kind:SimpleNamespace(has_attribute=lambda n:False))
        def try_spawn_actor(self,*args):return ego
        def spawn_actor(self,*args,**kwargs):return SimpleNamespace(listen=lambda c:None,destroy=lambda:True)
    world=World();ego,modules=fake_modules(world);calls=[]
    class ACK:
        @staticmethod
        def checked_control(control):return live.collector.control_dict(control)
        def __init__(self,client,carla,ego,frame,persist):
            self.ego,self.carla,self.frame,self.persist=ego,carla,frame,persist;self.receipts=[];self.alignment_failures=0
        def send(self,control,reason,expected_before_frame=None):
            ego.control=control
            row=dict(sequence=len(self.receipts)+1,before_frame=self.frame(),after_ack_frame=self.frame(),actor_id=9,
                requested_control=self.checked_control(control),server_accepted=True,status="ACKNOWLEDGED",reason=reason)
            self.receipts.append(row);self.persist(row);calls.append(("ACK",reason));return row
        def compare_observation(self,control,receipt,frame,*args):
            return dict(status="PASS",expected_receipt_sequence=receipt["sequence"],observed_frame=frame)
    @dataclass
    class FakeController:
        speed_preview_s:float=.5
    class Session:
        def __init__(self,model,args,route,specs,states,output):
            self.states,self.output=states,output;self.bundle_frame=None;self.observed_frames=deque(maxlen=10);self.history=deque(maxlen=10)
            self.bundle_wall_timing={};self.wall_timing_records=[];self.checkpoint_sha256="b"*64;self.source_sha256="a"*64
            self.episode_id="fake";self.rig_id="c"*64
            self.counts=dict(expert_startup_commands=0,prefill_count=0,inference_count=0,learned_commands_acknowledged=0,emergency_commands=0)
            self.policy=None;self.handed_off=False;self.latched_emergency=None
            live.previous.previous._observer.session=self
        def observe_bundle(self,bundle,frame):
            calls.append(("observe",frame));self.bundle_frame=frame
            if not drop_bundle:
                stamp=int(round(self.states[-1]["timestamp"]*1.e9))
                self.observed_frames.append(SimpleNamespace(timestamp_ns=stamp));self.history.append((stamp,None,stamp))
        def propose(self,transport,teacher,reason):
            if reason not in ("initial_drive_control","next_driving_control"):return teacher,None
            calls.append(("propose",reason));self.policy=SimpleNamespace(controller=FakeController(),prefill_records=[])
            if drop_proposal:return teacher,None
            self.counts["expert_startup_commands"]+=1
            if self.bundle_frame==self.states[-1]["frame"]:self.counts["prefill_count"]+=1
            return teacher,dict(control_source="expert_startup",learned_control_proposed=False,source_checkpoint_sha256=self.checkpoint_sha256)
    class Fixture:
        def __init__(self,world,args,manifest,green):self.world=world;self.record={};self.restored=False
        def install(self):pass
        def check(self):return world.frame
        def restore(self):self.restored=True;calls.append(("restorefixture",world.frame))
    def blank(carla):return Control(brake=1.)
    def checked(world,ego,frame,record,**kwargs):
        record.update(status="PASS",frame_before=frame,frame_after=frame,timestamp=frame*.05,actor_id=9,
            actor_snapshot_transform_carla={},current_control=live.collector.control_dict(ego.get_control()),pid_past_steering=0.)
    manual=live.previous.previous.signed1.manual
    observer=live.RightTurnObserver(route)
    original_hooks=(live.collector.AcknowledgedControlTransport,live.collector.exact_camera_bundle)
    with ExitStack() as stack:
        stack.enter_context(mock.patch.dict(sys.modules,modules))
        stack.enter_context(mock.patch.object(live.collector,"AcknowledgedControlTransport",ACK))
        stack.enter_context(mock.patch.object(live.collector,"exact_camera_bundle",lambda queues,frame,*a,**kw:{name:SimpleNamespace(timestamp=frame*.05) for name in live.collector.MODEL_CAMERA_ORDER}))
        stack.enter_context(mock.patch.object(live.frozen,"TemporalLiveResearchSessionV2",Session))
        stack.enter_context(mock.patch.object(live.frozen,"validate_capture"))
        stack.enter_context(mock.patch.object(live.collector,"_blank_brake_control",blank))
        stack.enter_context(mock.patch.object(manual,"checked_first_gear_initialization_observation",checked))
        stack.enter_context(mock.patch.object(live.collector,"checked_agent_initialization_observation",checked))
        stack.enter_context(mock.patch.object(live.previous.previous,"_observer",observer))
        stack.enter_context(mock.patch.object(live.previous,"GreenSignalFixture",Fixture))
        capture=live.runtime_factory();function=inspect.getclosurevars(capture).nonlocals["function"]
        namespace=function.__globals__
        point=route["route"][0]
        values=dict(physics_substep_module=lambda a:None,moving_capture_module=lambda a:None,
            _preflight_exclusive=lambda w:None,verify_live_opendrive_sha256=lambda m,sha:(dict(actual_sha256=sha),b"same"),
            actor_body_geometry=lambda e:{},_carla_transform=lambda c,v:v,_carla_location=lambda c,v:v,
            _configure_camera_blueprint=lambda b,s:None,archive_map_opendrive=lambda *a,**kw:dict(status="RECORDED",file="map",sha256=args.expected_live_xodr_sha256),
            basic_agent_control_configuration=lambda *a:({},{}),_python_class_source_provenance=lambda c:{},
            acknowledged_snapshot_state=lambda snapshot,ego,w:(dict(x=point["x"],y=point["y"],z=0.,yaw=point["yaw"],vx=0.,vy=0.,ax=0.,ay=0.),dict(world_velocity_carla=[0.,0.,0.])),
            measured_vehicle_fields=lambda *a:{},_validate_image_geometry=lambda *a:None,_save_jpeg=lambda *a:None,
            traffic_light_observation=lambda *a,**kw:{},_server_available_for_cleanup=lambda *a:True,
            _destroy_actor=lambda *a:True)
        stack.enter_context(mock.patch.dict(namespace,values))
        stack.enter_context(mock.patch.object(live,"runtime_factory",return_value=capture))
        stack.enter_context(mock.patch.object(live.collector,"collect_episode",live.capture_factory()))
        specs=[SimpleNamespace(name=name,carla_type="sensor.camera.rgb",carla_extrinsic={}) for name in live.collector.MODEL_CAMERA_ORDER]
        with live.frozen.activated(SimpleNamespace(),args):
            active_hooks=(live.collector.AcknowledgedControlTransport,live.collector.exact_camera_bundle)
            error=""
            try:live.collector.collect_episode(args,route,specs,partial,states,cameras,manifest)
            except Exception:error=traceback.format_exc()
            assert error,"synthetic capture must retain its intentional exception"
            assert active_hooks==(live.collector.AcknowledgedControlTransport,live.collector.exact_camera_bundle)
    assert original_hooks==(live.collector.AcknowledgedControlTransport,live.collector.exact_camera_bundle)
    assert (partial/"actual_runtime_binding.json").is_file(),error
    return calls,states,manifest,error,json.loads((partial/"actual_runtime_binding.json").read_text())


def test_full_original_compiled_capture_reaches_real_generic_observe_propose_ACK_and_restores(tmp_path):
    calls,states,manifest,error,proof=run_compiled_capture(tmp_path)
    assert "intentional next-tick failure" in error
    assert error.rstrip().endswith("RuntimeError: intentional next-tick failure after real first-fresh proposal")
    assert len(states)==25 and proof["warmup_observed_bundle_count"]==12 and proof["observed_bundle_calls"]==13
    assert proof["initial_drive_proposal_ACK_bound"] and proof["initial_drive_prefill_count"]==0
    assert proof["first_fresh_driving_prefill_count"]==1
    assert ("propose","initial_drive_control") in calls and ("propose","next_driving_control") in calls
    assert sum(c[0]=="observe" for c in calls)==13 and sum(c[0]=="restorefixture" for c in calls)==1
    assert ("ACK","exception_cleanup_abort") in calls and manifest["cleanup"]["completed"]


@pytest.mark.parametrize("kind",["bundle","proposal"])
def test_compiled_capture_missing_runtime_hook_cannot_run_whole_teacher_episode(tmp_path,kind):
    calls,states,manifest,error,proof=run_compiled_capture(tmp_path,drop_bundle=kind=="bundle",drop_proposal=kind=="proposal")
    assert "intentional next-tick failure" not in error and len(states)<=24
    assert "actual" in error and ("ACK","exception_cleanup_abort") in calls
    assert sum(c[0]=="restorefixture" for c in calls)==1 and manifest["cleanup"]["completed"]


def test_uninstalled_original_runtime_callbacks_fail_before_capture(tmp_path):
    live.verify_frozen()
    with pytest.raises(Exception,match="hooks were not installed"):
        live.capture_factory()(None,None,None,tmp_path,None,None,{})


@pytest.mark.parametrize("corruption",[None,"gap","camera","warmup","error","event"])
def test_full_original_110s_failed_budget_is_retained_never_turn_success(corruption):
    states=[dict(frame=100+i,capture_phase="stationary_warmup" if i<24 else "driving") for i in range(2200)]
    cameras=[dict(frame=s["frame"]) for s in states[::2]]
    manifest=dict(status="failed",error="BasicAgent did not finish within the driving budget reserved by the maximum total duration and stationary tail",
        capture_contract=dict(capture_phase_schedule=dict(stationary_tail=dict(scheduled_ticks=0))))
    event=dict(completed=False)
    if corruption=="gap":states[100]["frame"]+=1
    if corruption=="camera":cameras.pop()
    if corruption=="warmup":states[23]["capture_phase"]="driving"
    if corruption=="error":manifest["error"]="invented budget"
    if corruption=="event":event["completed"]=True
    assert owner.complete_interval(manifest,states,cameras,event)==(corruption is None)
    assert event["completed"]==(corruption=="event")


@pytest.mark.parametrize("fail",[False,True])
def test_REAL_full_inherited_collector_activation_and_restoration(fail):
    old=live.previous.previous;recovery=old.signed1.previous;fast45=recovery.previous;args=arguments()
    basic=SimpleNamespace(BasicAgent=type("FakeBasicAgent",(),{}))
    fake=SimpleNamespace(model_id=recovery.MODEL_ID,modules=lambda:[],state_dict=lambda:{},
        _research_speed_preview_calibration=live.PREVIEW_SCHEMA,_research_cpu_thread_numerical_revision=live.THREAD_SCHEMA,
        _research_recovery_simulator_control_authorized=True,
        _research_recovery_owner_binding=dict(purpose="owned_carla_research_only",simulation_only=True,owner_plan_sha256="a"*64))
    before=(live.collector.collect_episode,live.collector.exact_camera_bundle,live.collector.AcknowledgedControlTransport,
        old.generic.TemporalLiveResearchSessionV2,old.generic.TemporalSimulationResearchPolicyV2,live.previous.capture_factory,
        live.previous.SCHEMA,live.previous.OWNER_SCHEMA,old.SCHEMA)
    with mock.patch.object(old.previous.previous.previous,"validate_model_binding"), \
        mock.patch.object(recovery,"RouteRecoveryTemporalModelV3",SimpleNamespace), \
        mock.patch.object(fast45,"CPUFiniteCachedTemporalModelV2",SimpleNamespace), \
        mock.patch.object(old.previous.previous.torch,"get_num_threads",return_value=4), \
        mock.patch.object(old.previous.previous.torch,"get_num_interop_threads",return_value=1), \
        mock.patch.dict(sys.modules,{"agents":SimpleNamespace(),"agents.navigation":SimpleNamespace(basic_agent=basic),
            "agents.navigation.basic_agent":basic}):
        def enter():
            with live.activated(fake,args):
                assert old.generic.TemporalLiveResearchSessionV2.__name__=="KnownDevRightObservedSession"
                assert old.generic.TemporalSimulationResearchPolicyV2 is recovery.RecoverySingleCopyPolicyV3
                assert live.previous.SCHEMA==old.SCHEMA==live.SCHEMA
                assert any(c.__name__=="ResearchTransport" for c in live.collector.AcknowledgedControlTransport.__mro__)
                assert live.collector.exact_camera_bundle is not before[1]
                if fail:raise RuntimeError("scope failure")
        if fail:
            with pytest.raises(RuntimeError,match="scope failure"):enter()
        else:enter()
    assert before==(live.collector.collect_episode,live.collector.exact_camera_bundle,live.collector.AcknowledgedControlTransport,
        old.generic.TemporalLiveResearchSessionV2,old.generic.TemporalSimulationResearchPolicyV2,live.previous.capture_factory,
        live.previous.SCHEMA,live.previous.OWNER_SCHEMA,old.SCHEMA)


def test_teacher_catalog_completion_cannot_pass_new_runtime_guard(tmp_path):
    result=dict(actual_camera_bundle_count=934,raw_source_sha256={},learned_driving_screen_passed=True,
        joint_turn_motion_body_policy_wall100ms_passed=True)
    proof=dict(schema=live.SCHEMA+".actual_runtime_binding",actual_scoped_callbacks_bound=True,first_driving_guard_passed=True,
        warmup_observed_bundle_count=12,warmup_native_count=24,observed_bundle_calls=934)
    summary=dict(handoff_occurred=False,counts=dict(prefill_count=0,inference_count=0,learned_commands_acknowledged=0))
    with mock.patch.object(owner,"diagnostic_function",return_value=lambda *a:dict(result)), \
        mock.patch.object(owner.previous.previous,"read_json",side_effect=[proof,summary]):
        actual=owner.diagnostic_report(tmp_path,{}, {})
    assert not actual["actual_learned_runtime_binding_passed"]
    assert not actual["learned_driving_screen_passed"] and not actual["joint_turn_motion_body_policy_wall100ms_passed"]
