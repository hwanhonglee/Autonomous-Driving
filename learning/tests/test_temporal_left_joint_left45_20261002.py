"""HH_261002 - Synthetic sources/compiled capture only; no actual model, raw dataset, CARLA or endpoint authority."""
import ast
from collections import deque
from contextlib import contextmanager, ExitStack
from dataclasses import dataclass
import copy
import json
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
from unittest import mock

import pytest

from scripts.e2e import collect_temporal_left_joint_left45_20261002 as live
from scripts.e2e import research_temporal_left_joint_left45_20261002 as owner
from scripts.e2e.defer_exact_jpeg_until_ACK_research_20261001 import deferred_recording


def test_declared_counts_and_scope_are_not_model_or_formal_approval():
    c=live.source_contract()
    assert c["arms"]==["parent6000","LEFT20000"]
    assert c["phase_native_state_counts"]==dict(stationary_warmup=24,driving=901,stationary_tail=130)
    assert sum(c["phase_native_state_counts"].values())==1055 and c["camera_count"]==528 and c["ACK_count"]==1060
    assert c["driving_native_ticks"]==900 and c["minimum_post_handoff_progress_m"]==90.
    assert c["whole_motion_CTE_max_m"]==1. and c["whole_motion_CTE_p95_m"]==.35
    assert not c["model_promoted"] and not c["independent_evaluation"] and not c["HOLD_STOP_or_full_route_claim"]
    assert not c["sensor_callback_entry_observed"] and not c["exposure_to_ACK_wall_latency_proven"]


def test_capture_AST_only_exact_original_LEFT_three_seams():
    original=live.teacher.capture_ast()
    assert ast.dump(live.capture_ast())==ast.dump(original)
    assert len([n for n in ast.walk(original) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name)
        and n.func.id=="install_green_fixture"])==1
    assert "left45_post_capture_cleanup_full_brake" in ast.dump(original)
    assert live.ObservedSession.__mro__.index(live.DiagnosticCoastSession)<live.ObservedSession.__mro__.index(live.frozen.TemporalLiveResearchSessionV2)


@contextmanager
def fake_owner_scope(tmp_path,monkeypatch,arm="parent6000"):
    paths={key:tmp_path/(key+".json") for key in ("route","mapping","calibration","xodr")}
    for path in paths.values():path.write_text("{}")
    task=SimpleNamespace(ROUTE=paths["route"],paths=lambda run,case=None:(tmp_path/arm,tmp_path/(arm+"_plan.json")),
        case=lambda case:dict(seed=live.SEED),fixed_contract=lambda run,case:dict(schema=owner.SCHEMA,arm=arm,town="Town01",case_id=case))
    scope=SimpleNamespace(arm=arm,authority_path=tmp_path/"authority.json",authority_sha="a"*64,
        pins={name:"b"*64 for name in owner.loader.PARENT},weights=tmp_path/"model.npz",metadata=tmp_path/"metadata.json")
    for name,value in dict(_scope=scope,task=task,ASSETS={path:owner.digest(path) for path in paths.values()},
            MAPPING=paths["mapping"],CALIBRATION=paths["calibration"],XODR=paths["xodr"]).items():monkeypatch.setattr(owner,name,value)
    monkeypatch.setattr(owner,"runtime_sources",lambda:{"new_source.py":"c"*64})
    monkeypatch.setattr(owner,"external_sources",lambda:{"Town01.umap":"d"*64})
    monkeypatch.setattr(owner,"disk_check",lambda:{"required_bytes":123})
    originals=owner.adapted_originals();monkeypatch.setattr(owner,"_originals",originals)
    yield originals


@pytest.mark.parametrize("arm",live.ARMS)
def test_REAL_original_build_verify_captureargv_parse_preflight(tmp_path,monkeypatch,arm):
    with fake_owner_scope(tmp_path,monkeypatch,arm) as ns:
        plan=ns["build_plan"]("v1","pilot_00")
        owner.write_new(tmp_path/(arm+"_plan.json"),plan);pin=owner.digest(tmp_path/(arm+"_plan.json"))
        assert ns["verify_case"]("v1","pilot_00",pin)==plan
        args=owner.capture_argv("v1","pilot_00",pin)
        assert args[args.index("--max-duration-sec")+1]=="60"
        assert args[args.index("--agent-initialization")+1]=="after_bootstrap"
        actual=[]
        def parse(argv):
            # HH_261002 - Real research/parser flags are parsed against synthetic-only files; task-auth is separately covered.
            parser=live.frozen.parse_args
            with mock.patch.object(live.collector,"moving_capture_module",return_value=None):
                result=parser(argv[4:])
            actual.append(result);return result
        ns["worker_module"].parse_args=parse
        with mock.patch.object(ns["lifecycle"].lifecycle,"_assert_no_other_carla"),mock.patch.object(ns["lifecycle"],"ports_free"), \
            mock.patch.object(owner.subprocess,"run",return_value=SimpleNamespace(stdout=str(owner.EGG)+"/carla.so\n"+str(owner.CARLA_ROOT/"PythonAPI/carla/agents/__init__.py")+"\n")):
            result,info=ns["preflight"]("v1",pin)
        assert result==plan and info["status"]=="PREFLIGHT_PASS_NO_SIMULATOR" and actual[0].research_source_sha256==pin
        assert ns["lifecycle"].external_sources()=={"Town01.umap":"d"*64}
        assert plan["server_command"][2]=="Town01" and plan["external_source_sha256"]=={"Town01.umap":"d"*64}


def test_external_query_remains_nonrecursive_when_base_is_patched(monkeypatch):
    p=owner.CARLA_ROOT/"CarlaUE4/Content/Carla/Maps/Town07.umap"
    monkeypatch.setattr(owner,"_external_sources",lambda:{str(p):"a"*64,"API":"b"*64})
    monkeypatch.setattr(owner,"digest",lambda path:"c"*64)
    monkeypatch.setattr(owner.base,"external_sources",owner.external_sources)
    actual=owner.external_sources()
    assert str(p) not in actual and actual[str(p.with_name("Town01.umap"))]=="c"*64


def test_REAL_new_task_namespace_buildplan_captureargv_NEWparser_and_preflight(tmp_path,monkeypatch):
    # HH_261002 - Unlike a mocked config, this exercises the real inherited declaration/args guard in the NEW complete task namespace.
    assets={name:tmp_path/(name+".json") for name in ("route","mapping","calibration","xodr")}
    for path in assets.values():path.write_text("{}")
    pins={name:"b"*64 for name in owner.loader.PARENT}
    scope=SimpleNamespace(arm="parent6000",authority_path=tmp_path/"authority.json",authority_sha="a"*64,
        pins=pins,weights=tmp_path/"weights.npz",metadata=tmp_path/"metadata.json")
    for name,value in dict(_scope=scope,MAPPING=assets["mapping"],CALIBRATION=assets["calibration"],XODR=assets["xodr"],
        ASSETS={p:owner.digest(p) for p in assets.values()}).items():monkeypatch.setattr(owner,name,value)
    monkeypatch.setattr(owner,"root_path",lambda:tmp_path)
    monkeypatch.setattr(live.task,"ROUTE",assets["route"]);monkeypatch.setattr(live.task,"ROUTE_SHA",owner.digest(assets["route"]))
    task=live.task_namespace(owner);monkeypatch.setattr(owner,"task",task)
    monkeypatch.setattr(owner,"runtime_sources",lambda:{})
    monkeypatch.setattr(owner,"external_sources",lambda:{})
    monkeypatch.setattr(owner,"disk_check",lambda:{})
    ns=owner.adapted_originals();monkeypatch.setattr(owner,"_originals",ns)
    plan=ns["build_plan"]("v1","pilot_00");owner.write_new(tmp_path/"parent6000_plan.json",plan)
    pin=owner.digest(tmp_path/"parent6000_plan.json")
    assert ns["verify_case"]("v1","pilot_00",pin)==plan
    parsed=live.parse_args(owner.capture_argv("v1","pilot_00",pin),owner)
    assert parsed.recovery_run_id=="v1" and parsed.recovery_case_id=="pilot_00" and parsed.moving_task_plan_sha256==pin
    assert parsed.research_source_sha256==pin and parsed.stationary_tail_sec==6.5 and parsed.agent_initialization=="after_bootstrap"
    with mock.patch.object(ns["lifecycle"].lifecycle,"_assert_no_other_carla"),mock.patch.object(ns["lifecycle"],"ports_free"), \
        mock.patch.object(owner.subprocess,"run",return_value=SimpleNamespace(stdout=str(owner.EGG)+"/carla.so\n"+str(owner.CARLA_ROOT/"PythonAPI/carla/agents/__init__.py")+"\n")):
        assert ns["preflight"]("v1",pin)[1]["status"]=="PREFLIGHT_PASS_NO_SIMULATOR"


@pytest.mark.parametrize("arm",live.ARMS)
@pytest.mark.parametrize("fault",[None,"plan","endpoint","thread","preview","deniedowner"])
def test_model_loader_identity_does_not_masquerade_6000_as_20k(monkeypatch,arm,fault):
    pins={name:"b"*64 for name in owner.loader.PARENT}
    scope=SimpleNamespace(arm=arm,pins=pins,authority_sha="c"*64)
    monkeypatch.setattr(live,"_owner",SimpleNamespace(_scope=scope,parent_loader_schema="parentloader",parent_owner_schema="parentowner"))
    monkeypatch.setattr(live.loader,"verify_scope",lambda d:None)
    binding=dict(schema=live.loader.OWNER_SCHEMA if arm=="LEFT20000" else "parentowner",purpose="owned_carla_research_only",mode="RESEARCH_ONLY",simulation_only=True,
        owner_plan_sha256="a"*64,simulator_control_authorized=True,model_promoted=False,vehicle_control_approved=False,
        fixed_endpoint_updates=20000 if arm=="LEFT20000" else 6000,completed_run_authority_sha256="c"*64)
    model=SimpleNamespace(_research_recovery_owner_binding=binding,_research_recovery_simulator_control_authorized=True,
        _research_recovery_numeric_loader_schema=live.loader.SCHEMA if arm=="LEFT20000" else "parentloader",
        _research_temporal_export_metadata=dict(optimizer_updates=20000 if arm=="LEFT20000" else 6000),
        _research_temporal_checkpoint_sha256="b"*64,_research_temporal_export_npz_sha256="b"*64,_research_temporal_metadata_sha256="b"*64,
        _research_cpu_thread_numerical_revision=live.THREAD_SCHEMA,_research_speed_preview_calibration=live.PREVIEW_SCHEMA)
    if fault=="plan":binding["owner_plan_sha256"]="d"*64
    if fault=="endpoint":binding["fixed_endpoint_updates"]=6000 if arm=="LEFT20000" else 20000
    if fault=="thread":model._research_cpu_thread_numerical_revision="old"
    if fault=="preview":model._research_speed_preview_calibration="old"
    if fault=="deniedowner":binding["simulator_control_authorized"]=False
    if fault is None:live.verify_model(model,SimpleNamespace(research_source_sha256="a"*64))
    else:
        with pytest.raises(Exception):live.verify_model(model,SimpleNamespace(research_source_sha256="a"*64))


@pytest.mark.parametrize("fault",[None,"schema","order","missing","case","authority"])
def test_both_batch_pins_required_for_private_worker_and_parent(tmp_path,monkeypatch,fault):
    monkeypatch.setattr(owner,"root_path",lambda:tmp_path)
    monkeypatch.setattr(owner,"_scope",SimpleNamespace(authority_sha="a"*64))
    cases={}
    for arm in live.ARMS:
        path=tmp_path/(arm+"_plan.json");path.write_text(json.dumps({"arm":arm}));cases[arm]=owner.digest(path)
    batch=dict(schema=owner.SCHEMA+".batch",arm_order=list(live.ARMS),physical_case_count=2,
        completed_run_authority_sha256="a"*64,case_sha256=cases)
    if fault=="schema":batch["schema"]="old"
    if fault=="order":batch["arm_order"].reverse()
    if fault=="missing":batch["case_sha256"].pop(live.ARMS[1])
    if fault=="case":(tmp_path/(live.ARMS[0]+"_plan.json")).write_text("mutation")
    if fault=="authority":batch["completed_run_authority_sha256"]="b"*64
    owner.write_new(owner.batch_path(),batch);pin=owner.digest(owner.batch_path())
    if fault is None:assert owner.verify_batch(pin)==batch
    else:
        with pytest.raises(Exception):owner.verify_batch(pin)


def synthetic_turn():
    points=[dict(x=float(x),y=0.) for x in range(11)]
    points.extend(dict(x=10.+10.*math.sin(math.radians(a)),y=10.-10.*math.cos(math.radians(a))) for a in range(5,91,5))
    end=len(points)-1
    points.extend(dict(x=20.,y=10.+float(y)) for y in range(1,101))
    route=dict(route=points,physical_turn_preflight=dict(selected_block=dict(start_index=9,end_index=end)))
    states=[];decisions=[];receipts=[]
    for i,p in enumerate(points):
        heading=math.atan2(points[min(i+1,len(points)-1)]["y"]-p["y"],points[min(i+1,len(points)-1)]["x"]-p["x"])
        if i==len(points)-1:heading=math.pi/2
        frame=100+i;seq=i+1
        states.append(dict(frame=frame,timestamp=i*.5,capture_phase="driving",x=p["x"],y=p["y"],yaw=heading,
            control_transport=dict(status="PASS",expected_receipt_sequence=seq)))
        decisions.append(dict(ack_receipt_sequence=seq,command_frame=frame-1,control_source="learned_trajectory_tracker",
            learned_control_proposed=True,application=dict(learned_control_applied=True),control=dict(desired_speed_mps=2.)))
        receipts.append(dict(sequence=seq,before_frame=frame-1,server_accepted=True))
    return states,route,decisions,receipts


@pytest.mark.parametrize("fault",[None,"latehandoff","nonlearned","zero","heading","ACK","angle","progress"])
def test_real_unclamped_LEFT_event_and_original_90m_screen(fault):
    states,route,decisions,receipts=synthetic_turn()
    if fault=="latehandoff":
        for row in decisions[:35]:row["application"]["learned_control_applied"]=False
    if fault=="nonlearned":decisions[-1]["control_source"]="emergency_brake"
    if fault=="zero":decisions[-1]["control"]["desired_speed_mps"]=0.
    if fault=="heading":
        for s in states[30:]:s["yaw"]+=math.radians(16.)
    if fault=="ACK":receipts[1]["server_accepted"]=False
    if fault=="angle":
        for s in states[10:]:s["yaw"]*=.7
    if fault=="progress":states=states[:50]
    r=owner.left_motion_report(states,route,decisions,receipts)
    assert r["passed"]==(fault is None)
    assert len(r["all_driving_rows"])==len(states)
    if fault is None:assert r["actual_turn_event"]["stable_seconds"]>=3. and r["learned_post_handoff_progress_m"]>=90.


def test_projection_never_clamps_actual_backtracking():
    route=[dict(x=0.,y=0.),dict(x=200.,y=0.)]
    assert owner.projection(route,150.,.2)[0]==150.
    assert owner.projection(route,50.,.2)[0]==50.


def test_accumulated_zigzag_cannot_fake_exit_plus_net10_or_stability():
    states,route,d,a=synthetic_turn()
    states=states[:30]
    for i in range(30):
        state=copy.deepcopy(states[-1]);state.update(frame=state["frame"]+1,timestamp=state["timestamp"]+.5,x=20.,y=12.+(i%2),yaw=math.pi/2)
        seq=len(d)+1;state["control_transport"]["expected_receipt_sequence"]=seq
        d.append(dict(d[-1],ack_receipt_sequence=seq,command_frame=state["frame"]-1));a.append(dict(a[-1],sequence=seq,before_frame=state["frame"]-1));states.append(state)
    r=owner.left_motion_report(states,route,d,a)
    assert r["all_driving_rows"][-1]["actual_postturn_XY_travel_m"]>=10. and r["actual_turn_event"] is None


def test_empty_partial_diagnostic_never_becomes_owned_complete_or_PASS(tmp_path,monkeypatch):
    (tmp_path/"episode.partial").mkdir();(tmp_path/"episode.partial/manifest.json").write_text("{}")
    result=owner.diagnostic_report(tmp_path,dict(arm="parent6000"),dict(cleanup_errors=[],source_hashes_match=True,external_source_hashes_match=True))
    assert result["status"]=="FAIL_RETAINED" and not result["joint_motion_body_policy_wall100ms_passed"]
    assert not result["bounded_capture_complete"] and not result["all_failed_tail_retained"]
    assert result["raw_source_sha256"] and result["diagnostic_error"]


@pytest.mark.parametrize("fault",["offset","phase","reason","pedal"])
def test_no_extra_normal_sample_or_nonzero_pedal_becomes_diagnostic_context(fault):
    zero=Control()
    state=dict(frame=900,capture_phase="driving",moving_task=dict(future_support_complete=True,
        termination_reason="recovery_future_support_complete",driving_relative_native_frames=900))
    if fault=="offset":state["moving_task"]["driving_relative_native_frames"]=899
    if fault=="phase":state["capture_phase"]="stationary_tail"
    if fault=="reason":state["moving_task"]["termination_reason"]="unsafe"
    if fault=="pedal":zero.throttle=.001
    session=SimpleNamespace(states=[state],checkpoint_sha256="b"*64)
    with pytest.raises(Exception):live.DiagnosticCoastSession.propose(session,None,zero,"next_driving_control")


@pytest.mark.parametrize("fault",[None,"warmup_rate","coast_rate","rear_speed","actor_speed","gap","nan"])
def test_full_native_rate_and_speed_no_setup_or_coast_exemption(fault):
    states=[dict(frame=i,timestamp=i*.05,vx=1.,vy=0.,world_velocity_carla=[1.,0.,0.]) for i in range(1055)]
    if fault in ("warmup_rate","coast_rate"):states[24 if fault=="warmup_rate" else 1000]["world_velocity_carla"][0]=1.16
    if fault=="rear_speed":states[1000]["vx"]=2.500001
    if fault=="actor_speed":
        for s in states:s["world_velocity_carla"][0]=2.700001
    if fault=="gap":states[24]["frame"]+=1
    if fault=="nan":states[-1]["vx"]=math.nan
    if fault in ("gap","nan"):
        with pytest.raises(Exception):owner.native_rates(states)
    else:
        r=owner.native_rates(states);assert r["all_native_interval_count"]==1054 and r["passed"]==(fault is None)


def timing_fixture():
    control=dict(desired_speed_mps=2.3,throttle=.1)
    d=dict(ack_receipt_sequence=1,command_frame=10,inference_frame=10,held_between_camera_frames=False,
        learned_control_proposed=True,total_seconds=.07,control=control,measured_current_speed_mps=2.2,
        longitudinal_pi_research=dict(controller_target_cap_mps=2.4,original_model_target_mps=2.3,governed_controller_target_mps=2.3,
            explicit_controller_calibration_intervention=True,observed_speed_clipped=False,observed_speed_mps=2.2,
            one_fresh_camera_update_one_PI_integration=True,proposed_normal_tracker_control=control))
    w=dict(ACK_sequence=1,ACK_server_accepted=True,command_frame=10,inference_frame=10,source_checkpoint_sha256="b"*64,
        stream_source_sha256="a"*64,learned_control_proposed=True,exact_bundle_delivery_to_ACK_seconds=.09)
    return [d],[w],[dict(sequence=1,server_accepted=True)]


@pytest.mark.parametrize("fault",[None,"wall","policy","PI","missing","duplicate","wrongframe","empty"])
def test_actual_total_seconds_and_bundle_ACK_are_independent_nonvacuous(fault,monkeypatch):
    d,w,a=timing_fixture();monkeypatch.setattr(owner,"_scope",SimpleNamespace(pins=dict(checkpoint_sha256="b"*64)))
    monkeypatch.setattr(owner,"_owner_plan_sha_or_report","a"*64)
    if fault=="wall":w[0]["exact_bundle_delivery_to_ACK_seconds"]=.100001
    if fault=="policy":d[0]["total_seconds"]=.100001
    if fault=="PI":d[0]["longitudinal_pi_research"]["observed_speed_clipped"]=True
    if fault=="missing":w=[]
    if fault=="duplicate":w*=2
    if fault=="wrongframe":w[0]["inference_frame"]=9
    if fault=="empty":d,w,a=[],[],[]
    if fault in ("missing","duplicate","wrongframe"):
        with pytest.raises(Exception):owner.timing_report(d,w,a)
    else:
        r=owner.timing_report(d,w,a)
        assert (r["policy_passed"] and r["wall_passed"] and r["PI_target_cap_and_one_fresh_update_passed"])==(fault is None)
        assert not r["exposure_to_ACK_wall_latency_proven"]


class Control(SimpleNamespace):
    def __init__(self,**kw):super().__init__(**dict(dict(throttle=0.,brake=0.,steer=0.,reverse=False,hand_brake=False,gear=1,manual_gear_shift=True),**kw))


def compiled_capture(tmp_path,monkeypatch,*,fail_at=None,drop_bundle=False,drop_proposal=False):
    # HH_261002 - Execute the entire original compile-time collect_episode and actual generic observe/propose/ACK/recording callbacks.
    collector=live.collector;partial=tmp_path/"episode.partial";partial.mkdir()
    route=dict(town="Town01",scenario="left",route=[dict(x=0.,y=0.,yaw=0.,vad_command=0,distance_m=0.),dict(x=200.,y=0.,yaw=0.,vad_command=0,distance_m=200.)],
        start_carla_transform=dict(x=0.,y=0.,z=0.,pitch=0.,roll=0.,yaw=0.),goal_carla_transform=dict(x=200.,y=0.,z=0.,pitch=0.,roll=0.,yaw=0.))
    parsed=collector.parse_args([str(partial),str(tmp_path/"route.json"),"--agent-initialization","after_bootstrap","--control-transport","acknowledged_batch",
        "--stationary-warmup-sec","1.2","--stationary-tail-sec","6.5","--max-duration-sec","60","--target-speed-kmh","9"])
    parsed.recovery_run_id="v1";parsed.recovery_case_id="pilot_00";parsed.research_source_sha256="a"*64
    parsed.research_max_cte_m=1.;parsed.research_longitudinal_pi=True
    states=[];cameras=[];manifest=dict(capture_contract=dict(control_transport={},moving_task={}))
    calls=[]
    class World:
        frame=100
        settings=SimpleNamespace(synchronous_mode=True,fixed_delta_seconds=.05,no_rendering_mode=False,substepping=True,max_substep_delta_time=.01,max_substeps=5)
        def get_settings(self):return copy.copy(self.settings)
        def apply_settings(self,s):self.settings=s
        def get_weather(self):return 1
        def set_weather(self,w):pass
        def get_map(self):return SimpleNamespace(name="Town01")
        def get_snapshot(self):return SimpleNamespace(frame=self.frame,timestamp=SimpleNamespace(elapsed_seconds=self.frame*.05))
        def tick(self,*a):
            self.frame+=1
            if self.frame==fail_at:raise RuntimeError("synthetic source failure")
            return self.frame
        def get_blueprint_library(self):return SimpleNamespace(find=lambda k:SimpleNamespace(has_attribute=lambda n:False))
        def try_spawn_actor(self,*a):return ego
        def spawn_actor(self,*a,**kw):return SimpleNamespace(listen=lambda c:None,destroy=lambda:True)
    world=World()
    class Ego:
        id=9
        control=Control(brake=1.)
        def get_control(self):return self.control
        def get_velocity(self):return SimpleNamespace(x=1.,y=0.)
        def get_world(self):return world
        def get_physics_control(self):return None
    ego=Ego()
    class BasicAgent:
        def __init__(self,*a,**kw):
            self._vehicle=ego;self.planner=SimpleNamespace(_vehicle=ego,_vehicle_controller=SimpleNamespace(past_steering=0.),run_step=lambda:Control(throttle=.11))
        def get_local_planner(self):return self.planner
        def set_global_plan(self,p):pass
        def set_target_speed(self,s):pass
        def run_step(self):return self.planner.run_step()
        def done(self):return False
    basic=ModuleType("agents.navigation.basic_agent");basic.BasicAgent=BasicAgent
    navigation=ModuleType("agents.navigation");navigation.basic_agent=basic
    planner=ModuleType("agents.navigation.global_route_planner");planner.GlobalRoutePlanner=lambda *a:SimpleNamespace(trace_route=lambda *a:[0,1])
    carla=ModuleType("carla");carla.VehicleControl=Control;carla.Transform=lambda:None
    carla.WeatherParameters=SimpleNamespace(ClearNoon=1);carla.AttachmentType=SimpleNamespace(Rigid=1);carla.TrafficLightState=SimpleNamespace(Green="Green")
    carla.Client=lambda *a:SimpleNamespace(set_timeout=lambda *a:None,get_world=lambda:world,get_server_version=lambda:"fake",get_client_version=lambda:"fake")
    class ACK:
        checked_control=staticmethod(collector.control_dict)
        def __init__(self,client,carla,ego,frame,persist):self.ego,self.carla,self.frame,self.persist=ego,carla,frame,persist;self.receipts=[];self.alignment_failures=0
        def send(self,c,reason,expected_before_frame=None):
            ego.control=c;r=dict(sequence=len(self.receipts)+1,before_frame=self.frame(),after_ack_frame=self.frame(),actor_id=9,requested_control=collector.control_dict(c),server_accepted=True,status="ACKNOWLEDGED",reason=reason)
            self.receipts.append(r);self.persist(r);calls.append(("ACK",reason,self.frame(),r["requested_control"]));return r
        def compare_observation(self,control,receipt,frame,*a):return dict(status="PASS",expected_receipt_sequence=receipt["sequence"],observed_frame=frame)
    @dataclass
    class Config:
        speed_preview_s:float=.5
    class Policy:
        def __init__(self,*a,**kw):self.controller=Config();self.prefill_records=[];self._prefill_count=0
        def prefill(self,*a,**kw):self._prefill_count+=1;return {}
        def reset(self,*a):pass
        def step(self,*a,**kw):
            calls.append(("NN",states[-1]["frame"]))
            return dict(control_source="learned_trajectory_tracker",learned_control_proposed=True,control=dict(throttle=.12,brake=0.,steer=0.,reverse=False,hand_brake=False),total_seconds=.001)
    class SyntheticObserved(live.ObservedSession):
        def __init__(self,model,args,route,specs,states,output):
            self.model,self.args,self.states,self.output=model,args,states,output;self.bundle_frame=None
            self.observed_frames=deque(maxlen=10);self.history=deque(maxlen=10);self.actual_bundle_count=0
            self.bundle_wall_timing={};self.wall_timing_records=[];self.policy=None;self.handed_off=False;self.latched_emergency=None
            self.last_prefill_frame=self.last_inference_frame=None;self.last_decision=None;self.checkpoint_sha256="b"*64;self.source_sha256="a"*64
            self.episode_id="synthetic";self.rig_id="c"*64
            self.counts=dict(expert_startup_commands=0,prefill_count=0,inference_count=0,learned_commands_acknowledged=0,emergency_commands=0)
            live._session["policy_session"]=self
        def observe_bundle(self,bundle,frame):
            if drop_bundle:return
            self.actual_bundle_count+=1;self.bundle_frame=frame;stamp=int(round(states[-1]["timestamp"]*1e9))
            self.observed_frames.append(SimpleNamespace(timestamp_ns=stamp));self.history.append((stamp,None,stamp));calls.append(("observe",frame))
        def propose(self,*a,**kw):return (a[1],None) if drop_proposal else super().propose(*a,**kw)
    class Moving:
        failure=support=None
        selected=[]
        def __init__(self,p):self.origin=None
        def fail(self,why):self.failure=why
        def observe(self,**kw):
            if kw["phase"]=="driving" and self.origin is None:self.origin=kw["frame"]
            offset=None if self.origin is None else kw["frame"]-self.origin
            done=kw["phase"]=="driving" and offset==900
            if done:self.support={"frame":kw["frame"]}
            row=dict(future_support_complete=done,termination_reason="recovery_future_support_complete" if done else None,driving_relative_native_frames=offset)
            return row,row["termination_reason"]
        def saved_camera(self,c):pass
        def result(self,**kw):return dict(task_complete=True)
    moving=SimpleNamespace(MovingTask=Moving,configuration_from_args=lambda a:{},pending_scope=lambda:{},
        measured_progress=lambda *a:0.,
        validate_original_physics=lambda a:{},validate_applied_physics=lambda *a:None,fixed_contract=lambda *a:live.source_contract(),reconstruct=lambda *a:SimpleNamespace(selected=[],support={"frame":1026}))
    class Fixture:
        record=dict(state="INSTALLED")
        def install(self):pass
        def check(self):pass
    @contextmanager
    def signal_scope(*a):
        f=Fixture();f.record={"state":"INSTALLED"}
        try:yield f
        finally:f.record["state"]="RESTORED";calls.append(("fixture_restored",world.frame))
    def checked(world,ego,frame,record,**kw):
        record.update(status="PASS",frame_before=frame,frame_after=frame,timestamp=frame*.05,actor_id=9,actor_snapshot_transform_carla={},current_control=collector.control_dict(ego.control),pid_past_steering=0.)
    def jpeg(image,path,quality):
        assert next(c for c in reversed(calls) if c[0]=="ACK")[2]>=image.frame
        path.write_bytes(b"synthetic JPEG");calls.append(("JPEG",image.frame))
    module=SimpleNamespace(_scope=SimpleNamespace(arm="parent6000"))
    originals={name:getattr(collector,name) for name in (*live.NAMES,"collect_episode","moving_capture_module","acknowledged_snapshot_state")}
    patches=dict(_preflight_exclusive=lambda *a:None,_preflight_no_walkers=lambda *a:None,
        physics_substep_module=lambda *a:None,load_route=lambda *a:route,actor_body_geometry=lambda *a:{},
        verify_live_opendrive_sha256=lambda m,s:(dict(actual_sha256=s),b"same"),_carla_transform=lambda c,v:v,_carla_location=lambda c,v:v,
        _configure_camera_blueprint=lambda *a:None,archive_map_opendrive=lambda *a,**k:dict(status="RECORDED",file="map",sha256=None),
        basic_agent_control_configuration=lambda *a:({},{}),_python_class_source_provenance=lambda *a:{},
        acknowledged_snapshot_state=lambda s,*a:(dict(x=0.,y=0.,z=0.,yaw=0.,vx=1.,vy=0.,ax=0.,ay=0.),dict(world_velocity_carla=[1.,0.,0.])),
        measured_vehicle_fields=lambda *a:{},_validate_image_geometry=lambda *a:None,traffic_light_observation=lambda *a,**kw:{},
        _server_available_for_cleanup=lambda *a:True,_destroy_actor=lambda *a:True,_save_jpeg=jpeg,AcknowledgedControlTransport=ACK,
        install_normal_brake_cap=lambda planner,*a:{},
        exact_camera_bundle=lambda queues,frame,*a,**kw:{name:SimpleNamespace(timestamp=frame*.05,frame=frame,width=1,height=1,raw_data=b"1234") for name in collector.MODEL_CAMERA_ORDER})
    with ExitStack() as stack:
        stack.enter_context(mock.patch.dict(sys.modules,{"carla":carla,"agents":ModuleType("agents"),"agents.navigation":navigation,
            "agents.navigation.basic_agent":basic,"agents.navigation.global_route_planner":planner}))
        for key,value in patches.items():stack.enter_context(mock.patch.object(collector,key,value))
        stack.enter_context(mock.patch.object(live,"ObservedSession",SyntheticObserved))
        stack.enter_context(mock.patch.object(live,"task_namespace",return_value=moving))
        stack.enter_context(mock.patch.object(live,"validate_capture"))
        stack.enter_context(mock.patch.object(live,"verify_model"))
        stack.enter_context(mock.patch.object(live,"owned"))
        stack.enter_context(mock.patch.object(live.manual,"checked_first_gear_initialization_observation",checked))
        stack.enter_context(mock.patch.object(live.signals,"owned_fixture",signal_scope))
        stack.enter_context(mock.patch.object(live.preview,"thread_readback",lambda:dict(intra_op_threads=4,inter_op_threads=1)))
        stack.enter_context(mock.patch.object(live.preview.previous.torch,"get_num_threads",return_value=4))
        stack.enter_context(mock.patch.object(live.preview.previous.torch,"get_num_interop_threads",return_value=1))
        # HH_261002 - Activation and copied collector/recorder are REAL; only physical API/synthetic policy/data builders are fake.
        with live.activated(SimpleNamespace(),parsed,module),deferred_recording(collector,partial):
            with mock.patch.object(live.frozen,"TemporalSimulationResearchPolicyV2",Policy),mock.patch.object(live.frozen.original,"actor_controller_config",return_value=Config()):
                error=None
                try:collector.collect_episode(parsed,route,[SimpleNamespace(name=n,carla_type="sensor.camera.rgb",carla_extrinsic={}) for n in collector.MODEL_CAMERA_ORDER],partial,states,cameras,manifest)
                except Exception as exc:error=exc
    assert all(getattr(collector,name) is value for name,value in originals.items())
    assert not live._active and live._session is None and live._owner is None and not live.preview._active
    return states,cameras,calls,manifest,error,json.loads((partial/"actual_recording_binding.json").read_text())


def test_REAL_compiled_capture_NN899_context900_all130priorzero_final_unobserved_cleanup(tmp_path,monkeypatch):
    states,cameras,calls,manifest,error,proof=compiled_capture(tmp_path,monkeypatch)
    assert error is None,str(error)
    assert len(states)==1055 and len(cameras)==528 and proof["actual_recording_bundle_count"]==528
    assert proof["actual_after_ACK_receipt_count"]==528 and proof["pending_source_camera_count"]==0
    driving=[r for r in states if r["capture_phase"]=="driving"]
    assert len(driving)==901 and driving[-2]["moving_task"]["driving_relative_native_frames"]==899
    assert driving[-2]["research_control_source"]=="learned_trajectory_tracker"
    assert driving[-1]["research_control_source"]=="none"
    commands=[json.loads(line) for line in (tmp_path/"episode.partial/learned_control.jsonl").read_text().splitlines()]
    assert commands[-1]["diagnostic_control_source"]=="diagnostic_coast_context"
    assert commands[-1]["application"]["learned_control_applied"] is False and not commands[-1]["learned_STOP_claimed"]
    assert driving[-1]["moving_task"]["driving_relative_native_frames"]==900
    boundary=[c for c in calls if c[:2]==("ACK","driving_end_brake")];assert len(boundary)==1
    assert boundary[0][3]==dict(throttle=0.,brake=0.,steer=0.,gear=1,manual_gear_shift=True,reverse=False,hand_brake=False)
    assert all(s["current_control"]==boundary[0][3] for s in states[-130:])
    cleanup=[c for c in calls if c[:2]==("ACK","left45_post_capture_cleanup_full_brake")]
    assert len(cleanup)==1 and cleanup[0][2]==states[-1]["frame"] and cleanup[0][3]["brake"]==1.
    assert proof["initial_drive_prefill_count"]==0 and proof["first_fresh_driving_prefill_count"]==1


@pytest.mark.parametrize("failure",["tick","bundle","proposal"])
def test_REAL_compiled_missing_hooks_or_exception_preserve_raw_and_restore(tmp_path,monkeypatch,failure):
    states,cameras,calls,manifest,error,proof=compiled_capture(tmp_path,monkeypatch,fail_at=127 if failure=="tick" else None,
        drop_bundle=failure=="bundle",drop_proposal=failure=="proposal")
    assert error is not None and len(states)<=25
    assert any(c[:2]==("ACK","exception_cleanup_abort") for c in calls)
    assert sum(c[0]=="fixture_restored" for c in calls)==1
    assert proof["pending_source_camera_count"]==0 and proof["actual_recording_bundle_count"]==len(cameras)
