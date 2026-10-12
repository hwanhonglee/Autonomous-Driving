"""HH_261002 - Synthetic raw-ledger and protocol tests, not a CARLA/vehicle performance claim."""

from copy import deepcopy
import hashlib
import importlib
import json
import math
from pathlib import Path
import struct
import sys
from types import SimpleNamespace

import pytest

from scripts.e2e import calibrate_town07_manual_first_coast_20261002 as worker


def owner():
    return importlib.import_module("scripts.e2e.run_owned_town07_manual_first_coast_20261002")


def test_protocol_is_one_nontraining_manual_first_zero_pedal_measurement():
    contract=worker.protocol()
    assert contract["seed"]==2026102300 and 0<=contract["seed"]<2**32
    assert contract["coast_ticks"]==900 and contract["coast_duration_seconds"]==45.
    assert contract["coast_control"]==dict(throttle=0.,brake=0.,steer=0.,hand_brake=False,reverse=False,gear=1,manual_gear_shift=True)
    assert contract["physical_decoder_acceleration_mps2"]==[-2.9,2.9]
    assert contract["runtime_speed_rate_mps2"]==[-6.,3.]
    assert contract["maximum_actual_speed_mps"]==2.5
    assert all(contract[key] is False for key in ("automatic_retry","camera_capture","learned_control","goal_stop","training_data_approved","vehicle_control_approved"))


def test_open_loop_has_no_speed_error_feedback_after_handoff():
    first=worker.OpenLoopPreparation(); second=worker.OpenLoopPreparation()
    assert first.throttle(0.,0.)==second.throttle(0.,0.)==.11
    assert first.throttle(.5,2.)==second.throttle(.6,2.)==.11125
    assert first.throttle(.51,3.)==second.throttle(1.49,3.)
    assert first.throttle(1.,12.)==.20


@pytest.mark.parametrize("speed,time",[(True,0.),(-.1,0.),(math.nan,0.),(.5,math.inf)])
def test_preparation_rejects_malformed_actual_measurements(speed,time):
    with pytest.raises(RuntimeError):worker.OpenLoopPreparation().throttle(speed,time)


def test_launch_handoff_timeout_and_reverse_clock_are_not_hidden():
    launch=worker.OpenLoopPreparation();launch.throttle(0.,1.)
    with pytest.raises(RuntimeError,match="timed out"):launch.throttle(.499,9.)
    with pytest.raises(RuntimeError,match="reversed"):launch.throttle(.3,0.)


def test_entry_timeout_has_no_fallback_or_forced_velocity():
    launch=worker.OpenLoopPreparation();launch.throttle(0.,0.);launch.throttle(.5,2.)
    with pytest.raises(RuntimeError,match="1.5"):launch.throttle(1.499,20.)


@pytest.mark.parametrize("changed",[{"manual_gear_shift":False},{"gear":True},{"gear":2},{"throttle":.00001},{"brake":.00001},{"steer":.001},{"hand_brake":True}])
def test_strict_actual_manual_coast_rejects_any_wrong_field(changed):
    with pytest.raises(RuntimeError):worker.measured_control_is_manual_first(dict(worker.protocol()["coast_control"],**changed),coast=True)


def scalar_row(frame,speed,phase="coast"):
    return dict(frame=frame,timestamp=frame*.05,vx=speed,vy=0.,capture_phase=phase)


def test_native_2_95_violation_and_uncomfortable_preparation_are_preserved():
    rows=[scalar_row(1,0.,"open_loop_launch"),scalar_row(2,.1475,"open_loop_launch"),scalar_row(3,.14)]
    result=worker.quality(rows)
    assert result["status"]=="FAIL" and result["failed_interval_count"]==1
    assert result["failed_intervals"][0]["failed_bounds"]==["physical_decoder_2_9"]
    assert result["coast_failed_interval_count"]==0


def test_native_runtime_minus6_violation_is_separate_from_physical_gate():
    result=worker.quality([scalar_row(1,1.),scalar_row(2,.4)])
    assert result["failed_intervals"][0]["failed_bounds"]==["physical_decoder_2_9","runtime_speed_rate_3_minus6"]
    assert result["minimum_native_speed_rate_mps2"]==pytest.approx(-12.)


def test_empty_and_skipped_native_cannot_pass():
    assert worker.quality([])["status"]=="FAIL"
    with pytest.raises(RuntimeError):worker.quality([scalar_row(1,1.),scalar_row(3,.9)])


def fixture():
    # HH_261002 - These are explicitly synthetic ledger rows; no pixel, physical source or driving claim is made.
    rows=[];receipts=[];pose_x=0.;ramp=worker.OpenLoopPreparation()
    for index in range(955):
        frame=index+1; timestamp=frame*.05
        if index==0: phase="bootstrap";speed=0.;throttle=0.;brake=1.
        elif index<=24: phase="stationary_warmup";speed=0.;throttle=0.;brake=1.
        elif index<=54:
            phase="open_loop_launch";speed=(index-24)*.05;brake=0.
            throttle=ramp.throttle(rows[-1]["vx"],rows[-1]["timestamp"])
        else:phase="coast";speed=1.5*(.997**(index-54));throttle=brake=0.
        throttle=struct.unpack("f",struct.pack("f",throttle))[0]
        pose_x+=speed*.05
        command=dict(throttle=throttle,brake=brake,steer=0.,hand_brake=False,reverse=False,gear=1,manual_gear_shift=True)
        pose=dict(x=pose_x,y=0.,z=0.,roll=0.,pitch=0.,yaw=0.)
        row=dict(frame=frame,timestamp=timestamp,capture_phase=phase,x=pose_x-1.425,y=0.,z=0.,yaw=0.,vx=speed,vy=0.,ax=0.,ay=0.,yaw_rate=0.,
            actor_snapshot_transform_carla=pose,world_velocity_carla=[speed,0.,0.],world_acceleration_carla=[0.,0.,0.],world_angular_velocity_carla_deg_s=[0.,0.,0.],
            current_control=deepcopy(command),requested_control=deepcopy(command),receipt_sequence=frame,actor_world_planar_speed_mps=speed,
            route_cte_m=0.,events_seen=[{},{}],control_alignment=dict(status="PASS",frame_binding_pass=True,expected_receipt_sequence=frame,
            expected_command_before_frame=frame-1,observed_frame=frame,control_read_frame_before=frame,control_read_frame_after=frame,mismatched_fields=[]))
        rows.append(row);receipts.append(dict(sequence=frame,actor_id=123,response_actor_id=123,before_frame=frame-1,expected_before_frame=frame-1,
            after_ack_frame=frame-1,server_accepted=True,status="ACKNOWLEDGED",do_tick=False,reason=phase,requested_control=deepcopy(command)))
    cleanup=deepcopy(rows[-1]);cleanup.update(frame=956,timestamp=956*.05,capture_phase="cleanup_full_brake",vx=0.,actor_world_planar_speed_mps=0.,
        world_velocity_carla=[0.,0.,0.],receipt_sequence=956)
    cleanup["current_control"]["brake"]=cleanup["requested_control"]["brake"]=1.
    cleanup["control_alignment"].update(expected_receipt_sequence=956,expected_command_before_frame=955,observed_frame=956,control_read_frame_before=956,control_read_frame_after=956)
    ack=deepcopy(receipts[-1]);ack.update(sequence=956,before_frame=955,expected_before_frame=955,after_ack_frame=955,reason="cleanup_full_brake",requested_control=deepcopy(cleanup["current_control"]));receipts.append(ack)
    def witness(row):return dict(status="PASS",actor_id=123,expected_frame=row["frame"],frame_before=row["frame"],frame_after=row["frame"],
        timestamp=row["timestamp"],current_control=deepcopy(row["current_control"]),actor_snapshot_transform_carla=deepcopy(row["actor_snapshot_transform_carla"]))
    result=dict(actor_id=123,initialization_witnesses=[witness(rows[0]),witness(rows[24])],
        original_world_settings_restored=True,original_weather_restored=True,
        entry=dict(frame=rows[54]["frame"],timestamp=rows[54]["timestamp"],measured_rear_base_speed_mps=rows[54]["vx"]),
        physics_snapshots={name:dict(frame=row["frame"],timestamp=row["timestamp"],actual_control=deepcopy(row["current_control"]),
            internal_engine_or_wheel_RPM_measured=False,physics=dict(schema="carla.vehicle_physics_snapshot.v1",values=dict(mass=1375.,wheels=[{}]*4)))
            for name,row in (("initial",rows[24]),("coast_entry",rows[54]),("coast_end",rows[-1]))},native_motion_quality=worker.quality(rows))
    return rows,[cleanup],receipts,result


def test_full_synthetic_900_coast_raw_ACK_and_cleanup_membership():
    assert owner().validate_raw(*fixture())["coast_native_count"]==900


@pytest.mark.parametrize("mutation",["pedal","gear","frame","ack","snapshot","entry","early_entry","initialization","physics","coast_cropped","launch_law","event","quality","cleanup_brake"])
def test_raw_independent_audit_rejects_ledger_or_scope_mutation(mutation):
    rows,cleanup,receipts,result=fixture()
    if mutation=="pedal":rows[100]["current_control"]["brake"]=.01
    elif mutation=="gear":rows[100]["current_control"]["manual_gear_shift"]=False
    elif mutation=="frame":rows[100]["frame"]+=1
    elif mutation=="ack":receipts[100]["after_ack_frame"]+=1
    elif mutation=="snapshot":rows[100]["vx"]+=.01
    elif mutation=="entry":result["entry"]["measured_rear_base_speed_mps"]=1.5+.01
    elif mutation=="early_entry":rows[53]["vx"]=1.501
    elif mutation=="initialization":result["initialization_witnesses"][1]["frame_after"]+=1
    elif mutation=="physics":result["physics_snapshots"]["coast_entry"]["frame"]+=1
    elif mutation=="coast_cropped":rows.pop()
    elif mutation=="launch_law":rows[40]["requested_control"]["throttle"]+=.001
    elif mutation=="event":rows[100]["events_seen"]=[{101:[{}]},{}]
    elif mutation=="quality":result["native_motion_quality"]["status"]="FAIL"
    elif mutation=="cleanup_brake":cleanup[0]["current_control"]["brake"]=0.
    with pytest.raises((RuntimeError,ValueError,KeyError)):owner().validate_raw(rows,cleanup,receipts,result)


def test_owner_scopes_only_private_lifecycle_and_restores_after_exception():
    module=owner();original=module.private.paths,module.private.preflight,module.private.MODULE,module.private.ASSETS
    with pytest.raises(ValueError):
        with module.activated():
            assert module.private.paths is module.paths and module.private.CHECKPOINT_SHA is None
            with pytest.raises(RuntimeError):
                with module.activated():pass
            raise ValueError("test")
    assert (module.private.paths,module.private.preflight,module.private.MODULE,module.private.ASSETS)==original


def test_exact_safe_paths_no_versions_or_escape():
    module=owner()
    for value in ("v2","../v1","v1/../x",""):
        with pytest.raises(RuntimeError):module.paths(value)
    contract=module.fixed_contract("v1")
    assert contract["route_sha256"]=="8285e70a790d5e8ae75803db1aa538122b56e6fcfc3586e60eaa947d330417c9"
    assert contract["no_cameras_or_model_loading"] is True
    assert contract["worker_command"][-1]==module.private.PLAN_TOKEN


def test_actual_weather_observation_is_finite_and_complete():
    fixture=SimpleNamespace(**{name:1. for name in worker.WEATHER_FIELDS})
    assert set(worker.weather_observation(fixture))==set(worker.WEATHER_FIELDS)
    fixture.wetness=math.nan
    with pytest.raises(RuntimeError):worker.weather_observation(fixture)


@pytest.mark.parametrize("bad_preparation",[False,True])
def test_worker_real_control_transport_ABI_and_full_coast_in_fake_owned_world(tmp_path,monkeypatch,bad_preparation):
    # HH_261002 - Exercise the actual worker/ACK helpers with float32 API doubles, never a physical simulator.
    class Control:
        def __init__(self,**kwargs):
            for name,value in kwargs.items():
                setattr(self,name,struct.unpack("f",struct.pack("f",value))[0] if name in ("throttle","brake","steer") else value)
    class Transform:
        def __init__(self,location=None,rotation=None):self.location,self.rotation=location,rotation
    def vector(**kwargs):return SimpleNamespace(**kwargs)
    zero=vector(x=0.,y=0.,z=0.);rotation=vector(roll=0.,pitch=0.,yaw=0.)
    class Actor:
        id=123;type_id="vehicle.toyota.prius"
        bounding_box=SimpleNamespace(location=zero,rotation=rotation,extent=vector(x=2.,y=1.,z=.75))
        def __init__(self,pose):self.pose=pose;self.command=None;self.speed=0.;self.launch_ticks=0
        def get_control(self):return self.command
        def get_physics_control(self):return object()
        def destroy(self):world.actor=None;return True
    class SnapshotActor:
        id=123
        def get_transform(self):return deepcopy(world.actor.pose)
        def get_velocity(self):return vector(x=world.actor.speed,y=0.,z=0.)
        def get_acceleration(self):return zero
        def get_angular_velocity(self):return zero
    class Sensor:
        def listen(self,callback):pass
        def stop(self):pass
        def destroy(self):return True
    class World:
        def __init__(self):
            self.frame=100;self.actor=None
            self.settings=SimpleNamespace(synchronous_mode=False,fixed_delta_seconds=None,no_rendering_mode=False,
                substepping=True,max_substep_delta_time=.01,max_substeps=10)
            self.weather=SimpleNamespace(**{name:0. for name in worker.WEATHER_FIELDS})
        def get_map(self):return SimpleNamespace(name="Carla/Maps/Town07",to_opendrive=lambda:xml.decode())
        def get_settings(self):return deepcopy(self.settings)
        def apply_settings(self,value):self.settings=deepcopy(value)
        def get_weather(self):return deepcopy(self.weather)
        def set_weather(self,value):self.weather=deepcopy(value)
        def get_actors(self):return [] if self.actor is None else [self.actor]
        def get_blueprint_library(self):return SimpleNamespace(find=lambda name:SimpleNamespace(set_attribute=lambda *args:None))
        def try_spawn_actor(self,blueprint,pose):self.actor=Actor(pose);return self.actor
        def spawn_actor(self,*args,**kwargs):return Sensor()
        def get_snapshot(self):
            return SimpleNamespace(frame=self.frame,timestamp=SimpleNamespace(elapsed_seconds=self.frame*.05),
                find=lambda identifier:SnapshotActor() if identifier==123 else None)
        def tick(self,timeout):
            self.frame+=1;c=self.actor.command
            if c.brake==1.:self.actor.speed=0.
            elif c.throttle>0:
                self.actor.launch_ticks+=1
                increment=.16 if bad_preparation and self.actor.launch_ticks==1 else .05
                self.actor.speed+=increment
            else:self.actor.speed*=.997
            self.actor.pose.location.x+=self.actor.speed*.05
            return self.frame
    class Client:
        def __init__(self,*args):pass
        def set_timeout(self,*args):pass
        def get_world(self):return world
        def get_client_version(self):return "SYNTHETIC_NOT_PHYSICAL"
        def get_server_version(self):return "SYNTHETIC_NOT_PHYSICAL"
        def apply_batch_sync(self,commands,do_tick):
            assert do_tick is False and len(commands)==1
            world.actor.command=commands[0].control
            return [SimpleNamespace(actor_id=123,error="",has_error=lambda:False)]
    xml=b'<OpenDRIVE version="synthetic" />';world=World()
    carla=SimpleNamespace(__file__="/synthetic/api.egg/carla/__init__.py",Client=Client,VehicleControl=Control,
        Transform=Transform,Location=vector,Rotation=vector,WeatherParameters=SimpleNamespace(ClearNoon=deepcopy(world.weather)),
        command=SimpleNamespace(ApplyVehicleControl=lambda identifier,value:SimpleNamespace(actor_id=identifier,control=value)))
    monkeypatch.setitem(sys.modules,"carla",carla)
    monkeypatch.setattr(worker.physics,"physics_snapshot",lambda value:dict(schema="carla.vehicle_physics_snapshot.v1",values=dict(mass=1375.,wheels=[{}]*4)))
    monkeypatch.setattr(worker.capture,"load_route",lambda path:json.loads(path.read_text()))
    monkeypatch.setattr(worker.shutil,"disk_usage",lambda path:SimpleNamespace(free=10*1024**3))
    route=dict(coordinate_reference="base_link",start_carla_transform=dict(x=0.,y=0.,z=0.,roll=0.,pitch=0.,yaw=0.),
        route=[dict(x=0.,y=0.,z=0.,distance_m=0.),dict(x=210.,y=0.,z=0.,distance_m=210.)])
    route_path=tmp_path/"route_input.json";route_path.write_text(json.dumps(route))
    plan=dict(protocol=worker.protocol(),python_api="/synthetic/api.egg",xodr_sha256=hashlib.sha256(xml).hexdigest(),
        route_file=str(route_path),route_sha256=hashlib.sha256(route_path.read_bytes()).hexdigest())
    output=tmp_path/"calibration";record=worker.run(output,plan,lambda:None)
    rows=[json.loads(line) for line in (output/"states.jsonl").read_text().splitlines()]
    cleanup=[json.loads(line) for line in (output/"cleanup_states.jsonl").read_text().splitlines()]
    receipts=[json.loads(line) for line in (output/"control_receipts.jsonl").read_text().splitlines()]
    assert record.get("measurement_completed") is True,record
    assert sum(row["capture_phase"]=="coast" for row in rows)==900
    assert cleanup[0]["frame"]==rows[-1]["frame"]+1 and cleanup[0]["current_control"]["brake"]==1.
    assert record["errors"]==record["cleanup_errors"]==[]
    assert record["status"]==("FAIL_RETAINED" if bad_preparation else "PASS_MEASURED_CALIBRATION_NOT_TRAIN")
    assert owner().validate_raw(rows,cleanup,receipts,record)["coast_native_count"]==900
    assert record["native_motion_quality"]["failed_interval_count"]==(1 if bad_preparation else 0)


def test_diagnostic_and_CLI_reject_actual_recomputed_quality_despite_worker_PASS(tmp_path,monkeypatch):
    # HH_261002 - A forged worker summary cannot override the independently recomputed native quality conjunction.
    module=owner();rows,cleanup,receipts,result=fixture()
    rows[100]["vx"]-=.4;rows[100]["world_velocity_carla"][0]=rows[100]["vx"]
    rows[100]["actor_world_planar_speed_mps"]=rows[100]["vx"]
    result.update(status="PASS_MEASURED_CALIBRATION_NOT_TRAIN",measurement_completed=True,
        protocol=worker.protocol(),native_motion_quality=worker.quality(rows))
    assert result["native_motion_quality"]["status"]=="FAIL"
    output=tmp_path/"owned";folder=output/"calibration";folder.mkdir(parents=True)
    for name,values in (("states.jsonl",rows),("cleanup_states.jsonl",cleanup),("control_receipts.jsonl",receipts)):
        (folder/name).write_text("".join(json.dumps(row)+"\n" for row in values))
    (folder/"result.json").write_text(json.dumps(result))
    (folder/"route.json").write_text(json.dumps(dict(coordinate_reference="base_link",route=[])))
    (folder/"active_map_opendrive.xodr").write_bytes(b"synthetic geometry only")
    manifest=dict(capture_contract=dict(protocol=worker.protocol()),raw_file_sha256={path.name:hashlib.sha256(path.read_bytes()).hexdigest() for path in folder.iterdir()})
    (folder/"manifest.json").write_text(json.dumps(manifest))
    plan=dict(protocol=worker.protocol(),route_sha256=module.digest(folder/"route.json"),xodr_sha256=module.digest(folder/"active_map_opendrive.xodr"))
    monkeypatch.setattr(module,"closure_check",lambda *args:None)
    monkeypatch.setitem(sys.modules,"carla",SimpleNamespace(__file__=str(module.EGG)+"/carla/__init__.py"))
    def synthetic_geometry(*args):
        count=len(args[2])
        census={kind+"_"+region:dict(status="PASS",assessed_count=count-(1 if kind=="previous_sweep" else 0))
            for kind in ("sampled_body","previous_sweep") for region in ("raw_lane","inset_10cm_lane")}
        return [{}]*count,census
    monkeypatch.setattr(module.geometry,"body_census",synthetic_geometry)
    outcome=module.diagnostic_report(output,plan,dict(cleanup_errors=[],source_hashes_match=True,external_source_hashes_match=True,worker_exit_code=0))
    assert outcome["errors"]==[] and outcome["bounded_capture_complete"] is True
    assert outcome["native_motion_quality"]["status"]=="FAIL"
    assert outcome["measurement_passed"] is False and outcome["status"]=="FAIL_RETAINED"
    (output/"diagnostic_report.json").write_text(json.dumps(outcome))
    monkeypatch.setattr(module,"paths",lambda run_id:(output,tmp_path/"plan.json"))
    monkeypatch.setattr(module.private,"execute",lambda *args:0)
    assert module.main(["--mode","execute","--plan-sha256","synthetic"] )==2
