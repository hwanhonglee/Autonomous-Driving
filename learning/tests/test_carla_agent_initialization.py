"""HH_260906 - Verify the explicit post-bootstrap constructor without CARLA, world access, packages, or PID-history rewriting."""

import copy
import math
from types import SimpleNamespace
import sys

import pytest

from scripts.e2e import collect_carla_vad_expert as capture
from test_carla_wall_timing_integration import fake_run, control


@pytest.fixture
def initialization_run(fake_run, monkeypatch):
    original_parse, original_mode = capture.parse_args, capture.agent_initialization_mode
    original_transport_init = capture.AcknowledgedControlTransport.__init__
    active = {}

    def transport_init(worker, client, carla, ego, frame_reader, persist):
        original_get = ego.get_control
        def get_control():
            if frame_reader() == 0:
                return control(steer=-8.0)
            return original_get()
        ego.get_control = get_control
        ego.test_frame_reader = frame_reader
        original_transport_init(worker, client, carla, ego, frame_reader, persist)
    monkeypatch.setattr(capture.AcknowledgedControlTransport, "__init__", transport_init)

    def install(args):
        mode = original_mode(args)
        module = sys.modules.get("agents.navigation.basic_agent")
        if module is None:
            return mode
        def build(ego, **kwargs):
            observed = ego.get_control()
            active["constructors"].append({"frame":ego.test_frame_reader(),"observed":capture.control_dict(observed),"options":kwargs})
            if active["failure"] == "constructor":
                raise RuntimeError("synthetic constructor failure")
            controller = SimpleNamespace(past_steering=observed.steer)
            planner = SimpleNamespace(_vehicle_controller=controller)
            if active["failure"] == "bad_constructor_history":
                controller.past_steering = .2
            if active["failure"] == "nan_constructor_history":
                controller.past_steering = float("nan")
            def done():
                if active["failure"] == "engagement_history":
                    controller.past_steering = .2
                return False
            def step():
                if active["failure"] == "first_step":
                    raise RuntimeError("synthetic initial step failure")
                steer = max(-.8, min(.8, min(max(0., controller.past_steering - .1), controller.past_steering + .1)))
                if active["failure"] == "first_steering":
                    steer = .2
                controller.past_steering = steer
                return control(throttle=.15,steer=steer)
            return SimpleNamespace(set_global_plan=lambda plan:active["plans"].append(copy.deepcopy(plan)),
                get_local_planner=lambda:planner,done=done,run_step=step)
        module.BasicAgent = build
        return mode
    monkeypatch.setattr(capture,"agent_initialization_mode",install)

    def execute(name, *, mode="after_bootstrap", failure=None, timing=False):
        active.update(constructors=[],plans=[],failure=failure)
        monkeypatch.setattr(capture,"parse_args",lambda argv:original_parse(argv+(["--agent-initialization",mode] if mode else [])))
        result=fake_run(name,enabled=timing)
        return result,copy.copy(active["constructors"]),copy.copy(active["plans"])
    return execute


def test_default_and_explicit_historical_mode_preserve_prebootstrap_behavior(initialization_run):
    default,constructors,_=initialization_run("default",mode=None)
    explicit,other,_=initialization_run("explicit",mode="before_bootstrap")
    assert default.error is explicit.error is None
    assert constructors[0]["frame"] == other[0]["frame"] == 0
    assert constructors[0]["observed"]["steer"] == -8
    assert default.states == explicit.states and default.cameras == explicit.cameras and default.events == explicit.events
    assert "agent_initialization" not in default.manifest["capture_contract"]
    assert next(row for row in default.states if row["capture_phase"]=="driving")["current_control"]["steer"] == -.8


def test_new_mode_uses_same_existing_tick_plan_and_clean_framebound_constructor_history(initialization_run):
    historical,old,_=initialization_run("historical",mode="before_bootstrap")
    current,new,plans=initialization_run("current")
    assert current.error is None
    assert len(new)==len(old)==1 and new[0]["frame"]==1 and new[0]["observed"]["steer"]==0
    assert all(new[0]["options"][key]==old[0]["options"][key] for key in ("target_speed","opt_dict","map_inst")) and plans==[[1,2]]
    assert [event for event in current.events if isinstance(event,tuple) and event[0] in ("tick","rpc")] == [event for event in historical.events if isinstance(event,tuple) and event[0] in ("tick","rpc")]
    assert len(current.states)==len(historical.states)==6 and len(current.cameras)==3
    proof=current.manifest["capture_contract"]["agent_initialization"]
    assert proof["additional_world_ticks"]==0 and proof["pid_history_overwritten"] is False
    assert proof["construction"]["status"] == proof["engagement"]["status"] == "PASS"
    assert proof["construction"]["after"]["pid_past_steering"]==proof["engagement"]["before"]["pid_past_steering"]==0
    assert proof["construction"]["before"]["frame_before"]==proof["construction"]["after"]["frame_after"]==1
    assert proof["engagement"]["before"]["frame_before"]==3 and proof["engagement"]["first_proposed_steering_delta"]==0
    assert proof["engagement"]["first_command_acknowledged"] is True
    assert proof["engagement"]["first_command_receipt_sequence"]==current.states[1]["control_transport"]["next_command_receipt_sequence"]
    assert next(row for row in current.states if row["capture_phase"]=="driving")["current_control"]["steer"]==0
    assert current.manifest["cleanup"]["completed"] is True


@pytest.mark.parametrize("failure,stage,state_count",[("constructor","construction",0),("bad_constructor_history","construction",0),
    ("nan_constructor_history","construction",0),("engagement_history","engagement",2),
    ("first_step","engagement",2),("first_steering","engagement",2)])
def test_failed_constructor_or_engagement_retains_proof_and_owned_abort_cleanup(initialization_run,failure,stage,state_count):
    result,constructors,_=initialization_run(failure,failure=failure)
    assert result.error is not None and result.manifest["status"]=="failed"
    assert len(constructors)==1 and constructors[0]["frame"]==1
    assert len(result.states)==state_count and result.manifest["cleanup"]["completed"] is True
    assert result.manifest["capture_contract"]["agent_initialization"][stage]["status"]=="FAIL"
    assert result.events.count("destroy_ego")==1 and result.events.count("destroy_sensor")==8
    assert result.events.count("settings")==result.events.count("weather")==2
    import json
    receipts=[json.loads(line) for line in (result.folder/"control_receipts.jsonl").read_text().splitlines()]
    assert receipts[-1]["reason"]=="exception_cleanup_abort" and receipts[-1]["server_accepted"] is True
    assert not any(row["reason"]=="initial_drive_control" for row in receipts)


def test_new_initialization_does_not_change_timing_state_camera_coverage(initialization_run):
    plain,_,_=initialization_run("plain")
    timed,_,_=initialization_run("timed",timing=True)
    assert plain.error is timed.error is None and plain.states==timed.states and plain.cameras==timed.cameras
    assert timed.manifest["result"]["wall_timing"]["status"]=="COMPLETE_DIAGNOSTIC"


def test_cli_default_and_explicit_ack_boundary():
    assert capture.parse_args(["output","route"]).agent_initialization=="before_bootstrap"
    assert capture.parse_args(["output","route","--control-transport","acknowledged_batch","--agent-initialization","after_bootstrap"]).agent_initialization=="after_bootstrap"
    for extra in (["--agent-initialization","after_bootstrap"],["--agent-initialization","unknown"],["--agent-init","after_bootstrap"]):
        with pytest.raises(SystemExit):capture.parse_args(["output","route",*extra])
    with pytest.raises(capture.CollectionError):capture.agent_initialization_mode(SimpleNamespace(agent_initialization="after_bootstrap",control_transport="legacy_async"))


@pytest.fixture
def observation():
    vector=SimpleNamespace(x=1.,y=2.,z=3.)
    transform=SimpleNamespace(location=vector,rotation=SimpleNamespace(roll=0.,pitch=0.,yaw=0.))
    actor=SimpleNamespace(id=42,get_transform=lambda:transform)
    snapshot=SimpleNamespace(frame=8,timestamp=SimpleNamespace(elapsed_seconds=.4),find=lambda actor_id:actor)
    world=SimpleNamespace(get_snapshot=lambda:snapshot)
    ego=SimpleNamespace(id=42,get_control=lambda:control(brake=1))
    controller=SimpleNamespace(past_steering=0.)
    return world,ego,controller,snapshot,actor


@pytest.mark.parametrize("mutation",["framebool","framenegative","framewrong","actor_missing","nanpose","nantime","nonzero_steer", "throttle", "brake", "handbrake", "manual", "nanpast", "boolpast", "missingpast", "wrongpast"])
def test_initialization_witness_rejects_invalid_or_unmatched_state(observation,mutation):
    world,ego,controller,snapshot,actor=observation
    if mutation=="framebool":snapshot.frame=True
    elif mutation=="framenegative":snapshot.frame=-1
    elif mutation=="framewrong":snapshot.frame=9
    elif mutation=="actor_missing":snapshot.find=lambda _:None
    elif mutation=="nanpose":actor.get_transform().location.x=float("nan")
    elif mutation=="nantime":snapshot.timestamp.elapsed_seconds=float("nan")
    elif mutation=="nonzero_steer":ego.get_control=lambda:control(brake=1,steer=.00000001)
    elif mutation=="throttle":ego.get_control=lambda:control(brake=1,throttle=.1)
    elif mutation=="brake":ego.get_control=lambda:control(brake=.9)
    elif mutation=="handbrake":ego.get_control=lambda:control(brake=1,hand_brake=True)
    elif mutation=="manual":ego.get_control=lambda:control(brake=1,manual_gear_shift=True)
    elif mutation=="nanpast":controller.past_steering=float("nan")
    elif mutation=="boolpast":controller.past_steering=False
    elif mutation=="missingpast":del controller.past_steering
    elif mutation=="wrongpast":controller.past_steering=.1
    record={}
    with pytest.raises(capture.CollectionError):capture.checked_agent_initialization_observation(world,ego,8,record,controller=controller)
    assert record["status"]=="FAIL"


def test_frame_advance_during_observation_is_not_relabelled(observation):
    world,ego,controller,snapshot,_=observation
    later=copy.copy(snapshot);later.frame=9
    frames=iter([snapshot,later]);world.get_snapshot=lambda:next(frames)
    record={}
    with pytest.raises(capture.CollectionError,match="advanced"):
        capture.checked_agent_initialization_observation(world,ego,8,record,controller=controller)
    assert record["frame_before"]==8 and record["frame_after"]==9 and record["status"]=="FAIL"
