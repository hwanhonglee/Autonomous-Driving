"""HH_261001 - Verify manual-first-gear teacher scope and strict observed initialization without simulation."""

import ast
import copy
import inspect
import math
from types import SimpleNamespace
from unittest import mock

import pytest

from scripts.e2e import run_owned_town07_signed_recovery_v3_20261001 as v3


def control(throttle=.20, brake=0., steer=.2, gear=0, manual=False):
    return SimpleNamespace(throttle=throttle, brake=brake, steer=steer, hand_brake=False, reverse=False,
        manual_gear_shift=manual, gear=gear)


def test_all_eleven_fresh_seeds_same_smaller_grid_and_reserved_holdout():
    assert len(v3.CASES) == 11
    assert {(r["signed_lateral_offset_ros_m"],r["signed_yaw_offset_ros_deg"]) for r in v3.CASES[:9]} == {
        (a,b) for a in (-.2,0.,.2) for b in (-1.5,0.,1.5)}
    assert not {r["seed"] for r in v3.CASES} & {r["seed"] for r in v3.v2.CASES}
    with v3.activated():
        contract = v3.task.fixed_contract("v3","train_04")
        assert contract["schema"] == v3.TASK_SCHEMA and contract["seed"] == 2026100514
        assert contract["controller"]["manual_gear_shift"] is True and contract["controller"]["gear"] == 1
        assert contract["controller"]["launch_throttle"] == .11
        assert contract["controller"]["native_acceleration_qa_limit_mps2"] == 3.
        assert contract["selected_driving_relative_time_ns"] == [900_000_000,31_000_000_000]
        assert contract["required_future_support_ns"] == 6_400_000_000
        assert "training_data_approved" not in contract


@pytest.mark.parametrize("failure",[False,True])
def test_all_nested_bindings_restore_and_frozen_v2_stays_original(failure):
    base = v3.previous.worker_module.base
    old_blank,old_init,old_control = base._blank_brake_control,base.checked_agent_initialization_observation,v3.previous.worker_module.v2.CombinedControl
    task_before = {k:getattr(v3.task,k) for k in ("CASES","SCHEMA","fixed_contract")}
    owner_before = {k:getattr(v3.previous,k) for k in ("MODULE","SCHEMA","CASE_IDS","runtime_sources","declare_batch")}
    v2_before = {k:getattr(v3.v2,k) for k in ("CASES","SCHEMA","annotate_manifest")}
    try:
        with v3.activated():
            assert base.checked_agent_initialization_observation is v3.checked_first_gear_initialization_observation
            assert v3.previous.worker_module.v2.CombinedControl is v3.FirstGearRecoveryLaunchControl
            with pytest.raises(RuntimeError):
                with v3.activated(): pass
            fake_carla = SimpleNamespace(VehicleControl=lambda **kw:control(**kw))
            stopped = base._blank_brake_control(fake_carla)
            assert stopped.throttle == stopped.steer == 0. and stopped.brake == 1.
            assert stopped.gear == 1 and stopped.manual_gear_shift is True
            if failure: raise OSError("retained failure")
    except OSError: assert failure
    assert base._blank_brake_control is old_blank and base.checked_agent_initialization_observation is old_init
    assert v3.previous.worker_module.v2.CombinedControl is old_control
    assert all(getattr(v3.task,k) is value for k,value in task_before.items())
    assert all(getattr(v3.previous,k) is value for k,value in owner_before.items())
    assert all(getattr(v3.v2,k) is value for k,value in v2_before.items())
    assert not v3._active and not v3.v2._active
    v3.verify_frozen_dependencies()


def test_normal_only_manual_gear_and_final_control_journal_match_without_raw_rewrite():
    policy = v3.FirstGearRecoveryLaunchControl()
    raw = dict(throttle=.20,brake=0.,steer=.2,gear=0,manual_gear_shift=False)
    proposal = control()
    result = policy.apply(proposal,speed=0.,timestamp=1.,frame=1,normal_proposal=raw)
    assert result is proposal and result.gear == 1 and result.manual_gear_shift is True
    assert result.throttle == .11 and result.brake == result.steer == 0.
    assert policy.last["raw_normal_planner_proposal"] == raw and raw["manual_gear_shift"] is False
    assert policy.last["profile_normal_control"]["manual_gear_shift"] is True
    assert policy.last["profile_normal_control"]["gear"] == 1
    assert policy.last["final_hazard_return_modified_after_override"] is False


def fixture(*,gear=1,manual=True,past=0.,change=None):
    observed = control(throttle=0.,brake=1.,steer=0.,gear=gear,manual=manual)
    if change:
        for k,v in change.items(): setattr(observed,k,v)
    transform = SimpleNamespace(location=SimpleNamespace(x=1.,y=2.,z=0.),rotation=SimpleNamespace(roll=0.,pitch=0.,yaw=0.))
    actor = SimpleNamespace(id=7,get_transform=lambda:transform)
    snap = SimpleNamespace(frame=10,timestamp=SimpleNamespace(elapsed_seconds=1.),find=lambda actor_id:actor)
    world = SimpleNamespace(get_snapshot=lambda:snap)
    ego = SimpleNamespace(id=7,get_control=lambda:observed)
    return world,ego,SimpleNamespace(past_steering=past)


def test_strict_checker_keeps_actual_observed_manual_first_control_without_rewrite():
    world,ego,pid = fixture()
    record = {}
    assert v3.checked_first_gear_initialization_observation(world,ego,10,record,controller=pid) is record
    assert record["status"] == "PASS" and record["current_control"]["gear"] == 1
    assert record["current_control"]["manual_gear_shift"] is True and ego.get_control().manual_gear_shift is True
    assert record["pid_past_steering"] == 0. and record["frame_before"] == record["frame_after"] == 10


@pytest.mark.parametrize("gear,manual",[(0,True),(2,True),(1,False)])
def test_wrong_declared_gear_or_automatic_mode_fail_with_original_observation_retained(gear,manual):
    world,ego,pid = fixture(gear=gear,manual=manual); record={}
    with pytest.raises(v3.previous.worker_module.base.CollectionError):
        v3.checked_first_gear_initialization_observation(world,ego,10,record,controller=pid)
    assert record["status"] == "FAIL" and record["current_control"]["gear"] == gear
    assert record["current_control"]["manual_gear_shift"] is manual


@pytest.mark.parametrize("change",[{"throttle":.001},{"brake":.999},{"steer":.001},{"hand_brake":True},{"reverse":True}])
def test_original_full_service_brake_and_flag_guards_never_weaken(change):
    world,ego,pid = fixture(change=change)
    with pytest.raises(v3.previous.worker_module.base.CollectionError):
        v3.checked_first_gear_initialization_observation(world,ego,10,{},controller=pid)


@pytest.mark.parametrize("past",[.01,None,True,math.nan])
def test_original_unknown_nonzero_or_nonfinite_PID_history_rejected(past):
    world,ego,pid = fixture(past=past)
    with pytest.raises(v3.previous.worker_module.base.CollectionError):
        v3.checked_first_gear_initialization_observation(world,ego,10,{},controller=pid)


def test_original_no_advance_frame_and_actor_checks_remain():
    world,ego,pid = fixture(); snap=world.get_snapshot()
    with pytest.raises(v3.previous.worker_module.base.CollectionError):
        v3.checked_first_gear_initialization_observation(world,ego,9,{},controller=pid)
    snapshots = [snap,SimpleNamespace(frame=11)]
    with mock.patch.object(world,"get_snapshot",side_effect=snapshots),pytest.raises(v3.previous.worker_module.base.CollectionError):
        v3.checked_first_gear_initialization_observation(world,ego,10,{},controller=pid)
    snap.find = lambda actor_id: None
    with pytest.raises(v3.previous.worker_module.base.CollectionError):
        v3.checked_first_gear_initialization_observation(world,ego,10,{},controller=pid)


def test_checker_ast_parity_except_exact_declared_gear_clause_and_local_base_alias():
    old = next(row for row in ast.parse(v3.PARENT_CHECKER_FILE.read_text()).body if isinstance(row,ast.FunctionDef)
        and row.name == "checked_agent_initialization_observation")
    new = ast.parse(inspect.getsource(v3.checked_first_gear_initialization_observation)).body[0]
    class Normalize(ast.NodeTransformer):
        def visit_Attribute(self,node):
            if isinstance(node.value,ast.Name) and node.value.id == "base": return ast.Name(id=node.attr,ctx=node.ctx)
            return self.generic_visit(node)
    def normalize(value):
        value = Normalize().visit(copy.deepcopy(value)); value.name="checker"; value.returns=None
        for argument in value.args.args+value.args.kwonlyargs: argument.annotation=None
        value.body=[row for row in value.body if not isinstance(row,ast.Expr) or not isinstance(row.value,ast.Constant) or not isinstance(row.value.value,str)]
        value.body=[row for row in value.body if not isinstance(row,ast.Assign) or not any(isinstance(t,ast.Name) and t.id=="base" for t in row.targets)]
        return value
    old,new = normalize(old),normalize(new)
    def gear_clause(tree):
        return [row for row in ast.walk(tree) if isinstance(row,ast.If) and any(isinstance(n,ast.Constant)
            and n.value=="manual_gear_shift" for n in ast.walk(row.test))]
    old_clauses,new_clauses = gear_clause(old),gear_clause(new)
    assert len(old_clauses)==len(new_clauses)==1
    assert "gear" in ast.unparse(new_clauses[0].test) and "is not True" in ast.unparse(new_clauses[0].test)
    new_clauses[0].test=copy.deepcopy(old_clauses[0].test)
    assert ast.dump(old,include_attributes=False)==ast.dump(new,include_attributes=False)


def test_all_eleven_static_geometry_and_import_only_source_closure_bind_v3():
    with v3.activated():
        result = v3.v2.initial_body_preflight()
        pins = v3.runtime_sources()
        assert result["status"] == "PASS" and result["schema"] == v3.SCHEMA+".predicted_initial_geometry"
        assert [r["seed"] for r in result["cases"]] == [r["seed"] for r in v3.CASES]
        assert pins["tools/e2e/run_owned_town07_signed_recovery_v3_20261001.py"] == v3.previous.digest(v3.ROOT/"tools/e2e/run_owned_town07_signed_recovery_v3_20261001.py")
        assert pins[str(v3.V2_SOURCE.relative_to(v3.ROOT))] == v3.V2_SOURCE_SHA


@pytest.mark.parametrize("run_id",["v1","v2","v4","../v3"])
def test_frozen_failed_cohorts_and_arbitrary_paths_not_reused(run_id):
    with pytest.raises(RuntimeError): v3.main(["--mode","contract","--run-id",run_id])
