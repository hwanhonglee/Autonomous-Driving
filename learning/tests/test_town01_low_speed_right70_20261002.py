"""HH_261002 - Synthetic RIGHT70 control/geometry/namespace proofs; no actual route write, simulator or data admission."""
import ast
import copy
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest

from scripts.e2e import prepare_town01_right70_capture_route_20261002 as container
from scripts.e2e import town01_low_speed_right70_task_20261002 as task
from scripts.e2e import collect_town01_low_speed_right70_20261002 as worker
from scripts.e2e import run_owned_town01_low_speed_right70_20261002 as owner
import test_town01_low_speed_left45_20261002 as fixtures


def fixture(name):
    # HH_261002 - Reuse complete original synthetic physics fixtures with only prospective count constants; no real files/actors.
    node=next(n for n in ast.parse(Path(fixtures.__file__).read_bytes()).body if isinstance(n,ast.FunctionDef) and n.name==name)
    node=copy.deepcopy(node)
    for item in ast.walk(node):
        if isinstance(item,ast.Constant) and type(item.value) is int and item.value in (1056,1055,925,924):
            item.value={1056:1556,1055:1555,925:1425,924:1424}[item.value]
        if isinstance(item,ast.Constant) and type(item.value) is str:
            item.value=item.value.replace("left45_post_capture","right70_post_capture")
    namespace=dict(vars(fixtures),task=task,worker=worker,owner=owner)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),__file__,"exec"),namespace)
    return namespace[name]


@pytest.mark.parametrize("drift",[False,True])
def test_complete597_actual_timestamps_full70support_and130_context(drift):
    value,states,cameras=fixture("complete")(drift)
    result=value.result(tail_states=130)
    assert result["task_complete"] and len(value.selected)==597
    assert len(states)==1555 and len(cameras)==778
    assert value.first["frame"]==105 and value.last["frame"]==1297 and value.support["frame"]==1425
    assert value.last["frame"]-value.first["frame"]==1192
    assert result["selected_driving_relative_time_ns"]==[4_000_000_000,63_600_000_000]
    assert [r["frame"] for r in value.history_support["camera_observations"]]==list(range(87,106,2))
    assert value.history_support["preceding_native_guard"]["frame"]==86
    assert "training_data_approved" not in result and result["training_admission_state"]=="PENDING_QA_NOT_AUTHORIZED"


@pytest.mark.parametrize("omitted",[87,105,1297])
def test_missing_history_or_anchor_cannot_be_subset(omitted):
    value,_,_=fixture("complete")(omitted=omitted)
    assert not value.result(tail_states=130)["task_complete"]


@pytest.mark.parametrize("event",[1,700,1549])
def test_startup_driving_tail_event_denies_whole_pilot(event):
    value,_,_=fixture("complete")(event=event)
    assert value.failure=="recovery_collision_or_lane_event" and not value.result(tail_states=130)["task_complete"]


def test_public_contract_counts_seed_paths_and_unchanged_physics(capsys):
    assert owner.main(["--mode","contract"])==0
    c=json.loads(capsys.readouterr().out)
    assert c["schema"]==task.SCHEMA and c["seed"]==c["fixed_arguments"]["seed"]==2026102600<2**32
    assert c["expected_native_states"]==1555 and c["expected_camera_bundles"]==778 and c["expected_JPEG_count"]==4668
    assert c["expected_fixed_anchor_count"]==597 and c["true_future_point_count"]==38208
    assert c["owner_wall_timeout_s"]==600. and c["fixed_arguments"]["max_duration_sec"]==90.
    assert c["controller"]==task.previous.fixed_contract("v1","pilot_00")["controller"]
    assert c["whole_native_quality"]==task.previous.fixed_contract("v1","pilot_00")["whole_native_quality"]
    assert c["diagnostic_tail_policy"]["stationary_or_STOP_supervision_claimed"] is False
    assert c["closing_receipt_reader_schema"]==owner.closing.SCHEMA and owner.MODULE in c["worker_command"]
    assert "training_data_approved" not in json.dumps(c)


@pytest.mark.parametrize("run,case",[("v2","pilot_00"),("v1","train_04"),("../v1","pilot_00")])
def test_only_declared_one_case_allowed(run,case):
    with pytest.raises(ValueError):task.fixed_contract(run,case)


def test_private_namespace_and_public_worker_argv_not_stale():
    assert worker.task is task and owner.task is task and owner.worker_module is worker
    assert worker.parse_args.__globals__["task"] is task
    assert worker._teacher["scope"].__wrapped__.__globals__["task"] is task
    assert worker._teacher["scope"].__wrapped__.__globals__["v2"].CombinedControl is worker.Launch
    assert owner.preflight.__globals__["runtime_sources"] is owner.runtime_sources
    assert owner.build_plan.__globals__["execution_contract"] is owner.execution_contract
    args=owner.capture_argv("v1","pilot_00","a"*64)
    assert args[args.index("--max-duration-sec")+1]=="90" and args[args.index("--seed")+1]=="2026102600"
    assert args[5]==str(task.ROUTE)
    assert task.configuration_from_args.__globals__ is vars(task) and task.reconstruct.__globals__ is vars(task)


def test_identity_inverse_guards_and_current_phase_capture_math():
    assert isinstance(task.definition(),ast.Module) and isinstance(worker.identity_definition(),ast.Module)
    assert isinstance(owner.identity_ast(),ast.Module)
    before=worker.previous.capture_ast();after=worker.capture_ast()
    for n in ast.walk(after):
        if isinstance(n,ast.Constant) and n.value=="right70_post_capture_cleanup_full_brake":n.value="left45_post_capture_cleanup_full_brake"
    assert ast.dump(before)==ast.dump(after)
    sources=next(n for n in owner.identity_ast().body if isinstance(n,ast.FunctionDef) and n.name=="runtime_sources")
    strings=[n.value for n in ast.walk(sources) if isinstance(n,ast.Constant) and type(n.value) is str]
    assert "tools/e2e/town01_train_left45_green_fixture_20261002.py" in strings
    assert not any("town01_train_right70_green_fixture" in s for s in strings)
    report=next(n for n in owner.identity_ast().body if isinstance(n,ast.FunctionDef) and n.name=="diagnostic_report")
    keys=[n.arg for n in ast.walk(report) if isinstance(n,ast.keyword)]
    assert "actual_right_turn_screen" in keys and "actual_left_turn_screen" not in keys


def test_real_source_closure_without_nominal_payload_or_capture_reader():
    with mock.patch.object(owner,"nominal_witness",return_value={}):sources=owner.runtime_sources()
    for name in ("prepare_town01_right70_capture_route_20261002.py","town01_low_speed_right70_task_20261002.py",
        "collect_town01_low_speed_right70_20261002.py","run_owned_town01_low_speed_right70_20261002.py",
        "audit_town01_right70_closing_control_20261002.py","town01_train_left45_green_fixture_20261002.py"):
        assert "tools/e2e/"+name in sources


@pytest.mark.parametrize("phase,record,brake",[("driving",dict(future_support_complete=True,termination_reason="recovery_future_support_complete"),0.),
    ("stationary_tail",dict(future_support_complete=True,termination_reason=None),0.),
    ("stationary_tail",dict(future_support_complete=True,termination_reason="collision"),1.),("stationary_warmup",None,1.)])
def test_same_phase_coast_control_and_abort_exact(phase,record,brake):
    with mock.patch.object(worker,"_session",dict(original_blank=fixtures.blank,states=[])):
        c=worker.phase_control(None,phase,record)
        assert c.brake==brake and c.throttle==c.steer==0. and c.gear==1 and c.manual_gear_shift is True


def test_all1401_normal_proposals_replay_and130coast_separate_cleanup():
    value=fixture("command_fixture")();result=owner.teacher_command_audit(*value)
    assert result["status"]=="PASS" and result["normal_proposal_count"]==1401 and result["diagnostic_coast_observations"]==130


@pytest.mark.parametrize("fault",["source","target","normal","tail","closing","missing"])
def test_complete_command_audit_failures_not_forgiven(fault):
    commands,receipts,states=fixture("command_fixture")()
    row=next(v for v in commands if v["normal_policy"] is not None)
    if fault=="source":row["normal_policy"]["source_frame"]+=1
    elif fault=="target":row["target_ramp"]["actual_teacher_target_mps"]=0.
    elif fault=="normal":row["normal_policy"]["handoff_latched"]=True
    elif fault=="tail":states[-1]["current_control"]["brake"]=1.
    elif fault=="closing":commands[-1]["ack_receipt"]["before_frame"]-=1
    else:commands.pop(1)
    with pytest.raises((ValueError,RuntimeError)):owner.teacher_command_audit(commands,receipts,states)


@pytest.mark.parametrize("phase",["stationary_warmup","driving","stationary_tail"])
def test_no_runtime_tail_or_startup_rate_exemption(phase):
    rows=fixtures.states_for_quality([1.,.5],phase=phase);rows[0]["capture_phase"]="driving"
    assert owner.motion_quality(rows)["status"]=="FAIL"


def test_new_protected_last_future_rate2p95_fail_and_outside_tail_diagnostic_retained():
    rows=[dict(frame=25+i,timestamp=i*.05,capture_phase="driving",vx=1.,vy=0.) for i in range(1401)]
    rows[-1]["vx"]=1.1475
    assert owner.motion_quality(rows)["violations"][-1]["reasons"]==["protected_native_scalar_dvdt_exceeds_physical2p9"]
    rows[-1]["frame"]+=1
    result=owner.motion_quality(rows)
    assert result["status"]=="PASS_SCALAR_ONLY" and result["all_native_abs2p9_diagnostic_violations"][-1]["protected_history_or_true_future"] is False


def right_turn(yaw=-math.pi/2,progress=100.,finish=1000):
    points=[dict(x=0.,y=0.,yaw=0.,road_option="LANEFOLLOW"),dict(x=20.,y=0.,yaw=0.,road_option="RIGHT"),
        dict(x=20.,y=-30.,yaw=-math.pi/2,road_option="RIGHT"),dict(x=20.,y=-230.,yaw=-math.pi/2,road_option="LANEFOLLOW")]
    rows=[]
    for i in range(1401):
        arc=progress*min(1.,i/finish)
        rows.append(dict(frame=25+i,timestamp=i*.05,capture_phase="driving",x=min(20.,arc),y=-max(0.,arc-20.),yaw=yaw*min(1.,i/500)))
    return rows,dict(route=points)


@pytest.mark.parametrize("yaw,progress,finish,passed",[(-math.pi/2,100.,1000,True),(math.pi/2,100.,1000,False),
    (-math.pi/4,100.,1000,False),(-2.,100.,1000,False),(-math.pi/2,40.,1000,False),(-math.pi/2,61.,1270,False)])
def test_RIGHT_heading_exit10_and_continuous3_beforelastanchor(yaw,progress,finish,passed):
    rows,route=right_turn(yaw,progress,finish);r=owner.turn_screen(rows,route)
    assert (r["status"]=="PASS")==passed and r["selected_native_denominator"]==597


def generated_route():
    a=dict(x=88.61998748779297,y=-90.58333587646484,z=0.,yaw=-1.5707971257432236)
    b=dict(x=270.8,y=-133.43,z=0.,yaw=0.)
    return dict(route_id="town01_right_centerline_6p1_108p1_24p1_20261002_v1",coordinate_reference="base_link",
        spawn_point_reference="base_link",town="Town01",weather="ClearNoon",route=[a,b],start_ros_pose=dict(a),goal_ros_pose=dict(b),
        spawn_point=",".join(format(v,".17g") for v in (a["x"],-a["y"],a["z"]+.3,0.,0.,-math.degrees(a["yaw"]))),
        generation_provenance=dict(schema="portable_e2e.town01_train_right_gentle_centerline_nominal.20261002.v1.generation",
            contract=dict(lane_triples=[[6,0,1],[108,0,1],[24,0,1]],source_formulas_reused_unchanged=True)))


def test_declared_rear_container_keeps_every_original_point_and_roundtrip_halfwheelbase_once():
    original=generated_route();before=copy.deepcopy(original);new=container.build(original)
    assert original==before and all(new[k]==v for k,v in before.items())
    for label in ("start","goal"):
        transform=new[label+"_carla_transform"]
        center=worker.base.shift_transform_local_x(transform,2.85/2)
        actual=worker.base.base_link_state(center,[0.,0.,0.],[0.,0.,0.],0.)
        assert actual["x"]==pytest.approx(new[label+"_ros_pose"]["x"],abs=1e-12)
        assert actual["y"]==pytest.approx(new[label+"_ros_pose"]["y"],abs=1e-12)
        assert actual["yaw"]==pytest.approx(new[label+"_ros_pose"]["yaw"],abs=1e-12)
    assert new["start_carla_transform"]["z"]==.3 and new["goal_carla_transform"]["z"]==0.
    assert new["capture_container_provenance"]["rear_to_actor_shift_applied_here"] is False


@pytest.mark.parametrize("fault",["identity","endpoint","spawn","NaN","already_transformed","topology"])
def test_capture_container_cannot_change_endpoint_translate_twice_or_guess_spawn(fault):
    route=generated_route()
    if fault=="identity":route["weather"]="WetNoon"
    elif fault=="endpoint":route["start_ros_pose"]["x"]+=.1
    elif fault=="spawn":route["spawn_point"]="1,"+route["spawn_point"].split(",",1)[1]
    elif fault=="NaN":route["start_ros_pose"]["yaw"]=math.nan
    elif fault=="already_transformed":route["start_carla_transform"]={}
    else:route["generation_provenance"]["contract"]["lane_triples"][0][0]=7
    with pytest.raises((ValueError,RuntimeError)):container.build(route)


def test_owned_strict_reader_scope_restored_no_nesting_or_foreign_provider():
    before=owner.lifecycle.independent_audit
    with mock.patch.object(owner,"runtime_sources",return_value={}):
        with pytest.raises(LookupError):
            with owner.activated():
                assert owner.lifecycle.independent_audit.analyze_transport is owner.closing.analyze_transport
                assert owner.lifecycle.MODULE==owner.MODULE
                with pytest.raises(RuntimeError):
                    with owner.activated():pass
                raise LookupError("synthetic worker failure")
    assert owner.lifecycle.independent_audit is before and not owner._active
    with mock.patch.object(owner.lifecycle,"independent_audit",object()):
        with pytest.raises(RuntimeError):
            with owner.activated():pass


def test_container_creation_requires_exact_separate_external_authority_and_is_create_only(tmp_path,capsys):
    output=tmp_path/"plans/capture.json";caller=tmp_path/"root_permission.json"
    raw=container.encoded(container.build(generated_route()))
    with mock.patch.object(container,"ROOT",tmp_path),mock.patch.object(container,"OUTPUT",output), \
            mock.patch.object(container,"prepared",return_value=(raw,{})), \
            mock.patch.object(container.shutil,"disk_usage",return_value=SimpleNamespace(free=10*1024**3)):
        assert container.main(["--print-contract"])==0
        contract=json.loads(capsys.readouterr().out)
        assert not output.exists() and contract["capture_authorized"] is False
        with pytest.raises(ValueError):container.main([])
        authority=container.encoded(dict(contract,container_creation_authorized=True));caller.write_bytes(authority)
        pin=hashlib.sha256(authority).hexdigest()
        with pytest.raises(ValueError):container.main(["--caller-pins",str(caller),"--caller-pins-sha256","0"*64])
        assert not output.exists()
        assert container.main(["--caller-pins",str(caller),"--caller-pins-sha256",pin])==0
        assert output.read_bytes()==raw
        with pytest.raises(ValueError):container.main(["--caller-pins",str(caller),"--caller-pins-sha256",pin])


def test_real_teacher_scope_restores_snapshot_manual_provider_and_original_BasicAgent():
    # HH_261002 - The original complete scope fixture is executed with the NEW task/worker/owner namespace, no simulator import.
    fixture("test_real_private_teacher_scope_provider_snapshot_blank_manual_and_restore")()
