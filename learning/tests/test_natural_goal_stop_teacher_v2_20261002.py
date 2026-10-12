"""HH_261002 - Synthetic v2 cap/provider/failure protocol only; no CARLA, model, GPU or actual archives."""

import ast
from contextlib import contextmanager
from pathlib import Path
import struct
import sys
from types import ModuleType, SimpleNamespace

import pytest

from scripts.e2e import town07_natural_goal_stop_task_v2_20261002 as task
from scripts.e2e import collect_town07_natural_goal_stop_v2_20261002 as collector
from scripts.e2e import run_owned_town07_natural_goal_stop_v2_20261002 as owner


def control(**kwargs):
    values = dict(throttle=.20,brake=0.,steer=.1,hand_brake=False,reverse=False,manual_gear_shift=False,gear=0)
    values.update(kwargs)
    return SimpleNamespace(**values)


def apply(policy,*,speed=1.,timestamp=1.,frame=1,target=2.4,**kwargs):
    c = control(**kwargs)
    return policy.apply(c,speed=speed,timestamp=timestamp,frame=frame,target_speed_mps=target,
        normal_proposal=collector.base.control_dict(c))


class NativeFloat32Control(SimpleNamespace):
    def __setattr__(self,name,value):
        if name in ("throttle","brake","steer"):value = collector.native_float32(value)
        super().__setattr__(name,value)


def test_two_declared_configuration_changes_no_measured_quality_change_or_permission():
    cfg = task.goal_configuration(SimpleNamespace(**task.FIXED_ARGS))
    old = task.previous.goal_configuration(SimpleNamespace(**task.FIXED_ARGS))
    changed = {name for name in vars(cfg) if getattr(cfg,name) != getattr(old,name)}
    assert changed == {"profile_id","normal_brake_cap","desired_deceleration_mps2"}
    assert cfg.normal_brake_cap == .01 and cfg.desired_deceleration_mps2 == .2
    assert cfg.stopped_speed_mps == .05 and cfg.hold_seconds == 2. and cfg.minimum_tail_seconds == 6.5
    assert cfg.target_acceleration_limit_mps2 == old.target_acceleration_limit_mps2
    plan = task.fixed_contract("v2","pilot_00")
    assert plan["controller"]["normal_brake_cap"] == .01
    assert plan["fixed_arguments"] == task.previous.FIXED_ARGS
    assert plan["prospective_two_factor_revision"]["individual_factor_causal_effect_claim"] is False
    assert plan["attempts"] == 1 and plan["automatic_retry"] is False
    assert plan["formal_feature_denominators"] == dict(main=9,sub=30)
    assert "training_data_approved" not in task.pending_scope()
    assert plan["full_RTE04_approved"] is False and plan["model_promoted"] is False


def test_unchanged_math_earlier_envelope_not_brake_capability_claim():
    new = task.goal.GoalStopGovernor(task.goal_configuration(SimpleNamespace(**task.FIXED_ARGS)),2.4,20.)
    old = task.goal.GoalStopGovernor(task.previous.goal_configuration(SimpleNamespace(**task.FIXED_ARGS)),2.4,20.)
    for _ in range(40):
        new.update(100.,100.,2.4,-100.);old.update(100.,100.,2.4,-100.)
    new_record = new.update(10.,10.,2.4,-10.)
    old_record = old.update(10.,10.,2.4,-10.)
    assert new_record["target_speed_mps"] < old_record["target_speed_mps"]
    for _ in range(40):result = new.update(.75,.75,.04,-.75)
    assert result["complete"] is False
    result = new.update(.75,.75,.04,-.75)
    assert result["complete"] is True and result["hold_duration_sec"] == 2.
    assert new.update(.75,.75,.050001,-.75)["complete"] is False


@pytest.mark.parametrize("run,case",[("v1","pilot_00"),("v2","train_00"),("../v2","pilot_00")])
def test_fresh_v2_namespace_only(run,case):
    with pytest.raises(Exception):task.paths(run,case)


def test_task_scope_restores_every_frozen_alias_even_on_exception():
    names = ("SCHEMA","PROFILE","FIXED_ARGS","paths","pending_scope","goal_configuration","fixed_contract","MovingTask")
    saved = {name:getattr(task.previous,name) for name in names}
    with pytest.raises(RuntimeError):
        with task.activated():
            assert task.previous.PROFILE == task.PROFILE
            assert task.fixed_contract("v2","pilot_00")["schema"] == task.SCHEMA
            assert "training_data_approved" not in task.pending_scope()
            with pytest.raises(Exception):
                with task.activated():pass
            raise RuntimeError("synthetic")
    assert all(getattr(task.previous,name) is value for name,value in saved.items()) and task._active is False


def test_event_membership_all_native_and_true_hold_tail_preserved_under_v2():
    instance = task.MovingTask(task.fixed_contract("v2","pilot_00"))
    def observe(frame,phase,complete=False):
        instance.observe(frame=frame,timestamp=frame*.05,phase=phase,progress=frame*.1,
            base_stop_reason="comfortable_goal_measured_stop_and_dwell" if frame == 800 else None,event_count=0,
            native_actor_speed_mps=0. if complete else 2.4,rear_base_CTE_m=0.,observed_output_bytes=0,
            goal_record=dict(profile_id=task.PROFILE,complete=complete),observed_free_disk_bytes=task.FLOOR+task.RESERVE)
        if frame%2 == 0:
            camera = dict(frame=frame,timestamp=frame*.05,capture_phase=phase,camera_order=task.original.CAMERAS,
                images={n:n+".jpg" for n in task.original.CAMERAS},source_timestamps={n:frame*.05 for n in task.original.CAMERAS})
            instance.saved_camera(camera)
            assert camera["moving_task_selected"] is (phase == "driving" and frame >= 80)
    for frame in range(801):observe(frame,"driving",frame == 800)
    for frame in range(801,931):observe(frame,"stationary_tail",True)
    result = instance.result(tail_states=130,goal_reached=True)
    assert result["schema"] == task.SCHEMA+".result" and result["task_complete"] is True
    assert len(instance.selected) == 361 and result["selected_camera_span_ns"] == 36_000_000_000
    assert instance.support["timestamp_ns"] >= instance.last["timestamp_ns"]+6_400_000_000
    assert result["startup_anchors_selected"] is False and result["stationary_tail_only_true_future_context"] is True


@pytest.mark.parametrize("brake",[0.,.001,.01])
def test_actual_float32_normal_brakes_preserved_without_observed_value_rewrite(brake):
    c = NativeFloat32Control()
    for name,value in vars(control(throttle=0. if brake else .20,brake=brake)).items():setattr(c,name,value)
    before = collector.base.control_dict(c)
    policy = collector.NaturalGoalLaunchControl()
    policy.apply(c,speed=1.,timestamp=1.,frame=1,target_speed_mps=8.64/3.6,normal_proposal=before)
    assert c.brake == before["brake"] and policy.last["capped_normal_proposal"] == before
    assert policy.last["native_float32_globals_modified"] is False
    assert policy.last["schema"] == task.SCHEMA+".normal_control"
    if brake:
        assert c.throttle == 0. and policy.last["normal_braking_preserved"] is True
        assert policy.started_at is None


def test_float32_next_above_new_cap_and_old_point1_rejected_without_global_change():
    old_bound = collector.previous.NORMAL_BRAKE_READBACK_BOUND
    bits = struct.unpack("!I",struct.pack("!f",.01))[0]
    next_value = struct.unpack("!f",struct.pack("!I",bits+1))[0]
    for brake in (next_value,.1):
        with pytest.raises(Exception):apply(collector.NaturalGoalLaunchControl(),brake=brake)
    assert collector.previous.NORMAL_BRAKE_READBACK_BOUND == old_bound == collector.native_float32(.1)


@pytest.mark.parametrize("requested",[.1,.01])
def test_both_actual_install_call_sites_use_exact_new_planner_cap(requested):
    recorded = []
    def installer(planner,cap,throttle):
        recorded.append((planner,cap,throttle));return dict(normal_brake_cap=cap,normal_throttle_cap=throttle)
    planner = object()
    result = collector.install_v2_cap(installer,planner,requested,.2)
    assert recorded == [(planner,.01,.2)] and result["normal_brake_cap"] == .01
    for cap,throttle in ((.11,.2),(.1,None),(.01,.21)):
        with pytest.raises(Exception):collector.install_v2_cap(installer,planner,cap,throttle)


def test_original_real_normal_cap_hook_keeps_hazard_after_normal_untouched():
    controller = SimpleNamespace(max_brake=.5,max_throt=.75)
    original = NativeFloat32Control()
    for name,value in vars(control(brake=1.)).items():setattr(original,name,value)
    planner = SimpleNamespace(_vehicle_controller=controller,run_step=lambda:original)
    metadata = collector.install_v2_cap(task.goal.install_normal_brake_cap,planner,.1,.2)
    result = planner.run_step()
    assert result.brake == collector.native_float32(.01)
    assert metadata["basic_agent_emergency_brake_modified"] is False
    assert metadata["empty_queue_brake_capped_before_hazard_override"] is True
    # HH_261002 - A later untouched BasicAgent hazard can still choose full brake.
    result.brake = 1.
    assert result.brake == 1.


def test_real_base_parse_new_provider_and_all_restore(monkeypatch):
    monkeypatch.setattr(task,"configuration_from_args",lambda args:dict(run=args.natural_goal_run_id))
    original = task.configuration_from_args
    old_helper = collector.base.configuration_from_args
    args = collector.parse_args(owner.capture_argv("v2","pilot_00"))
    assert args.natural_goal_run_id == "v2" and args.max_duration_sec == 140.
    assert args.stationary_tail_sec == 6.5 and args.agent_initialization == "after_bootstrap"
    assert args.seed == 2026102200 and args.target_speed_kmh == 8.64
    assert task.configuration_from_args is original and collector.base.configuration_from_args is old_helper


def test_collector_overlay_installed_metadata_and_all_scope_restoration(monkeypatch):
    agents,navigation,module = (ModuleType(name) for name in ("agents","agents.navigation","agents.navigation.basic_agent"))
    agents.navigation,navigation.basic_agent = navigation,module
    monkeypatch.setitem(sys.modules,"agents",agents);monkeypatch.setitem(sys.modules,"agents.navigation",navigation)
    monkeypatch.setitem(sys.modules,"agents.navigation.basic_agent",module)
    old_agent = type("OriginalAgent",(),{})
    module.BasicAgent = old_agent
    native_before = collector.previous.NORMAL_BRAKE_READBACK_BOUND
    manifest = dict(capture_contract={})
    installer_calls = []
    def install(planner,cap,throttle):
        installer_calls.append((cap,throttle));return dict(normal_brake_cap=cap,normal_throttle_cap=throttle)
    monkeypatch.setattr(collector.base,"install_normal_brake_cap",install)
    monkeypatch.setattr(collector.base,"collect_episode",lambda *args,**kwargs:"captured")
    monkeypatch.setattr(task,"verify_frozen",lambda:None)
    @contextmanager
    def frozen_scope(args):
        class FrozenTeacher:
            def __init__(self):
                metadata = collector.base.install_normal_brake_cap(object(),.1,.2)
                manifest["capture_contract"]["teacher_controller"] = dict(metadata,normal_cap_readback_comparison="OLD .10")
        module.BasicAgent = FrozenTeacher
        try:yield
        finally:module.BasicAgent = old_agent
    monkeypatch.setattr(collector.previous,"scope",frozen_scope)
    saved_task,saved_policy = collector.previous.task,collector.previous.NaturalGoalLaunchControl
    saved_install,saved_collect = collector.base.install_normal_brake_cap,collector.base.collect_episode
    with pytest.raises(RuntimeError):
        with collector.scope(SimpleNamespace()):
            assert collector.base.collect_episode(None,None,None,Path("/synthetic"),None,None,manifest) == "captured"
            module.BasicAgent()
            metadata = manifest["capture_contract"]["teacher_controller"]
            assert installer_calls == [(.01,.2)]
            assert "NEW .01" in metadata["normal_cap_readback_comparison"]
            assert metadata["prospective_desired_deceleration_mps2"] == .2
            assert manifest["control_source"].endswith("manual1_v2")
            with pytest.raises(Exception):
                with collector.scope(SimpleNamespace()):pass
            raise RuntimeError("synthetic")
    assert collector.previous.task is saved_task and collector.previous.NaturalGoalLaunchControl is saved_policy
    assert collector.base.install_normal_brake_cap is saved_install and collector.base.collect_episode is saved_collect
    assert module.BasicAgent is old_agent and collector.previous.NORMAL_BRAKE_READBACK_BOUND == native_before
    assert collector._active is False and task._active is False


def membership_fixture():
    replayed = dict(schema=task.SCHEMA+".result",task_complete=True,failure_reason=None,selected_camera_count=929,
        selected_camera_span_ns=92_800_000_000,source_tokens=["all","unchanged"])
    stored = dict(replayed,task_complete=False,failure_reason="moving_task_capture_or_cleanup_failed")
    manifest = dict(status="failed",error=owner.QUALITY_FAILURE)
    actual_owner = dict(worker_exit_code=1)
    quality = dict(status="FAIL",violation_count=2)
    return stored,replayed,manifest,actual_owner,quality


def test_only_authenticated_terminal_failure_comparison_markers_normalized_no_mutation():
    stored,replayed,manifest,actual_owner,quality = membership_fixture()
    result = owner.compare_membership(stored,replayed,manifest,actual_owner,quality,True)
    assert result["normalization_applied"] is True and result["source_capture_failure_preserved"] is True
    assert result["raw_source_result"]["task_complete"] is False
    assert result["raw_source_result"]["failure_reason"] == "moving_task_capture_or_cleanup_failed"
    assert result["training_authorization"] is False and result["original_native_quality_passed"] is False
    assert stored["task_complete"] is False and manifest["status"] == "failed" and quality["status"] == "FAIL"


@pytest.mark.parametrize("changed",["membership","unknown_marker","quality_pass","quality_mismatch","exit_zero","native_complete","unknown_error"])
def test_marker_comparison_cannot_hide_any_other_difference_or_unproven_failure(changed):
    stored,replayed,manifest,actual_owner,quality = membership_fixture()
    if changed == "membership":stored["selected_camera_count"] = 928
    elif changed == "unknown_marker":stored["failure_reason"] = "some_other_guard"
    elif changed == "quality_pass":quality["status"] = "PASS"
    elif changed == "exit_zero":actual_owner["worker_exit_code"] = 0
    elif changed == "native_complete":manifest["status"] = "complete"
    elif changed == "unknown_error":manifest["error"] = "some unrelated capture exception"
    with pytest.raises(Exception):owner.compare_membership(stored,replayed,manifest,actual_owner,quality,changed != "quality_mismatch")


def test_exact_membership_still_cannot_turn_quality_failed_capture_into_pass():
    required = ("ownership","capture_identity","transport","camera","native_chain","camera_chain","pixels","membership",
        "initialization","quality_matches","body","source_closure","raw_task_complete")
    checks = dict.fromkeys(required,True)
    manifest = dict(status="complete",result=dict(termination_reason="comfortable_goal_measured_stop_and_dwell"))
    assert not owner.capture_passed(manifest,dict(worker_exit_code=0),dict(checks,quality_passed=False),130)
    assert owner.capture_passed(manifest,dict(worker_exit_code=0),dict(checks,quality_passed=True),130)
    assert not owner.capture_passed(dict(manifest,status="failed"),dict(worker_exit_code=0),dict(checks,quality_passed=True),130)
    assert not owner.capture_passed(manifest,dict(worker_exit_code=1),dict(checks,quality_passed=True),130)


@pytest.mark.parametrize("corrupt_membership",[False,True])
def test_full_raw_body_cadence_quality_attempts_continue_after_failure(monkeypatch,tmp_path,corrupt_membership):
    stored,replayed,failed_manifest,actual_owner,quality = membership_fixture()
    if corrupt_membership:stored["selected_camera_count"] = 928
    plan = task.fixed_contract("v2","pilot_00")
    capture = dict(natural_goal_stop_teacher=plan,goal_stop_profile=dict(profile_id=task.PROFILE,bounds={}),
        control_transport=dict(bootstrap_observation={}),teacher_controller=dict(normal_brake_cap=.01,
        normal_throttle_cap=.2,prospective_desired_deceleration_mps2=.2))
    manifest = dict(failed_manifest,capture_contract=capture,result=dict(moving_task=stored,goal_stop_quality=quality,
        goal_reached=True,termination_reason="comfortable_goal_measured_stop_and_dwell"))
    states = [dict(frame=i,timestamp=i*.05,capture_phase="driving" if i<4 else "stationary_tail") for i in range(134)]
    cameras = [dict(frame=i,timestamp=i*.05) for i in range(0,134,2)]
    receipts = [dict(sequence=i) for i in range(139)]
    monkeypatch.setattr(owner,"read_json",lambda p:manifest if p.name == "manifest.json" else dict(route=[]))
    monkeypatch.setattr(owner,"jsonl",lambda p:{"states.jsonl":states,"camera_frames.jsonl":cameras,"control_receipts.jsonl":receipts}[p.name])
    monkeypatch.setattr(owner.lifecycle.independent_audit,"analyze_transport",lambda *a,**k:dict(status="PASS"))
    monkeypatch.setattr(owner.lifecycle.independent_audit,"analyze_camera_alignment",lambda *a,**k:dict(status="PASS"))
    monkeypatch.setattr(owner.previous,"pixel_audit",lambda *a:dict(source_pixels_passed=True))
    monkeypatch.setattr(owner.initialization,"initialization_audit",lambda *a:dict(status="PASS"))
    calls = []
    def measured(*args):
        calls.append(("quality",len(args[0]),len(args[1])));return quality
    monkeypatch.setattr(owner.worker_module.base,"measured_stop_quality",measured)
    monkeypatch.setattr(task,"reconstruct",lambda *a:SimpleNamespace(result=lambda **k:replayed))
    def body(output,plan,episode,actual_states,actual_receipts,manifest):
        calls.append(("body",len(actual_states),len(actual_receipts)))
        return dict(all_native_body_and_swept_clearance_passed=True,native_count=len(actual_states),sweep_count=len(actual_states)-1)
    monkeypatch.setattr(owner.previous,"body_report",body)
    monkeypatch.setattr(owner.previous,"closure_check",lambda *a:calls.append(("closure",)))
    actual_owner.update(cleanup_errors=[],source_hashes_match=True,external_source_hashes_match=True)
    report = owner.diagnostic_report(tmp_path,plan,actual_owner)
    assert ("quality",134,67) in calls and ("body",134,139) in calls and ("closure",) in calls
    assert report["status"] == "FAIL_RETAINED" and report["actual_teacher_capture_passed"] is False
    assert report["native_state_count"] == 134 and report["ACK_receipt_count"] == 139
    assert report["checks"]["native_chain"] is True and report["checks"]["body"] is True
    assert report["checks"]["source_closure"] is True
    assert report["checks"]["membership"] is not corrupt_membership
    assert report["training_data_approved"] is False and report["future_labels_qualified"] is False


def test_owner_scope_restore_private_identity_source_closure_and_frozen_bytes():
    names = ("task","worker_module","MODULE","SCHEMA","CASE_IDS","capture_argv","execution_contract","runtime_sources","disk_check","diagnostic_report")
    saved = {name:getattr(owner.previous,name) for name in names}
    with pytest.raises(RuntimeError):
        with owner.activated():
            assert owner.previous.task is task and owner.previous.worker_module is collector
            with pytest.raises(Exception):
                with owner.activated():pass
            raise RuntimeError("synthetic")
    assert all(getattr(owner.previous,name) is value for name,value in saved.items()) and owner._active is False
    source = Path(owner.previous.__file__).read_text()
    function = next(n for n in ast.parse(source).body if isinstance(n,ast.FunctionDef) and n.name == "worker")
    worker = ast.get_source_segment(source,function)
    assert worker.index("assert_owned_worker") < worker.index("with worker_module.scope") < worker.index("base.run")
    task.verify_frozen()
    closure = {str(p.relative_to(task.ROOT)) for p in owner.lifecycle.imported_source_closure((Path(owner.__file__).resolve(),))}
    assert set(task.FROZEN_V1).issubset(closure)
    assert {"tools/e2e/"+name for name in ("town07_natural_goal_stop_task_v2_20261002.py",
        "collect_town07_natural_goal_stop_v2_20261002.py","run_owned_town07_natural_goal_stop_v2_20261002.py")}.issubset(closure)


def test_reserve_and_total_native_tail_schedule_unchanged():
    floor = task.FLOOR+task.QUOTA+2*task.RESERVE
    assert owner.disk_check(floor)["prospective_cases"] == 1
    with pytest.raises(Exception):owner.disk_check(floor-1)
    schedule = collector.base.capture_phase_schedule(20.,140.,1.2,6.5)
    assert schedule["maximum_total_ticks"] == 2800 and schedule["stationary_tail"]["scheduled_ticks"] == 130
