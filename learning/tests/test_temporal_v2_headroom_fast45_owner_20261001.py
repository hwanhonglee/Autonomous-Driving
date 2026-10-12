"""HH_261001 - Verify prospective45x3 ownership, unchanged safety, all-failure denominators and no physical test jobs."""

import ast
import copy
import inspect
import json
from types import SimpleNamespace
import textwrap

import pytest

from scripts.e2e import research_temporal_v2_headroom_fast45_town07_20261001 as owner


def test_contract_declares45s90m_three_trials_and_separate_wall_deadline():
    with owner.scoped_trial(1):
        plan=owner.fixed_contract("v1",True)
    assert plan["total_capture_seconds"]==45 and plan["prospective_trial_count"]==3
    assert plan["precursor_motion_screen"]["net_learned_progress_minimum_m"]==90.
    assert plan["precursor_motion_screen"]["applied_original_model_stop_target_fresh_and_held_commands_maximum"]==0
    assert plan["sensor_and_ACK_screen"]["native_state_count"]==900
    assert plan["sensor_and_ACK_screen"]["camera_bundle_count"]==450
    assert plan["body_clearance_screen"]["all_snapshot_linear_sweeps"]==899
    assert plan["policy_deadline_seconds"]==.1 and plan["controller_target_cap_mps"]==2.4
    assert plan["model_speed_cap_mps"]==2.5 and plan["native_actor_planar_speed_maximum_mps"]==2.7
    assert plan["original_NN_decoder_selected_trajectory_math_unchanged"] is True
    assert plan["original_decoder_gate_tracker_math_unchanged"] is False
    assert plan["automatic_retry"] is False and plan["model_only_performance_claimed"] is False
    assert plan["observed_wall_latency_screen"]["sensor_callback_entry_observed"] is False
    assert plan["full_RTE01_approved"] is False and plan["vehicle_control_approved"] is False
    argv=plan["capture_argv"]
    assert argv[argv.index("--max-duration-sec")+1]=="45"
    assert argv[argv.index("--target-speed-kmh")+1]=="9"
    assert "--research-longitudinal-pi" in argv
    assert "--trial" in plan["worker_command"]


@pytest.mark.parametrize("trial", [-1,3,True,None])
def test_invalid_or_implicit_trial_rejected(trial):
    with pytest.raises(RuntimeError):
        with owner.scoped_trial(trial):pass


def test_scoped_lifecycle_and_capture_validation_restore_after_nesting_or_exception():
    names=("paths","fixed_contract","runtime_sources","live","load_npz","diagnostic_report","MODULE","SCHEMA")
    before={name:getattr(owner.base,name) for name in names}
    validation=owner.live.frozen.validate_capture
    with pytest.raises(RuntimeError,match="stop"):
        with owner.scoped_trial(0):
            assert owner.base.MODULE==owner.MODULE and owner.base.live is owner.live
            assert owner.live.frozen.validate_capture is owner.live.validate_capture
            with pytest.raises(RuntimeError):
                with owner.scoped_trial(1):pass
            raise RuntimeError("stop")
    assert all(getattr(owner.base,name) is value for name,value in before.items())
    assert owner.live.frozen.validate_capture is validation
    assert owner._trial is None and owner._active is False


def test_contract_mode_cannot_launch_processes(monkeypatch,capsys):
    monkeypatch.setattr(owner.base,"execute",lambda *_:pytest.fail("physical execute forbidden in contract test"))
    monkeypatch.setattr(owner.base.subprocess,"Popen",lambda *_a,**_k:pytest.fail("subprocess launch forbidden"))
    assert owner.main([])==0
    assert json.loads(capsys.readouterr().out)["total_capture_seconds"]==45


def test_import_only_source_closure_contains_new_and_lazy_dependencies():
    first=owner.runtime_sources();second=owner.runtime_sources()
    assert first==second
    for name in ("tools/e2e/research_temporal_v2_headroom_fast45_town07_20261001.py",
        "tools/e2e/collect_temporal_v2_headroom_fast45_research_20261001.py",
        "learning/portable_e2e/temporal_cpu_finite_validation_v2_20261001.py",
        "tools/e2e/carla_physics_substeps.py","tools/e2e/carla_moving_capture.py","tools/e2e/carla_wall_timing.py"):
        assert first[name]==owner.digest(owner.ROOT/name)
    assert all(first[name]==sha for name,sha in owner.FROZEN_REVISIONS.items())


def test_diagnostic_core_ast_diff_is_only_declared90045090_revision():
    old=ast.parse(textwrap.dedent(inspect.getsource(owner.base.diagnostic_report)))
    new=ast.parse(textwrap.dedent(inspect.getsource(owner.diagnostic_core)))
    old.body[0].name="diagnostic_core"
    class DeclaredConstants(ast.NodeTransformer):
        def visit_Constant(self,node):
            if type(node.value) is int and node.value in (400,200):node.value={400:900,200:450}[node.value]
            elif type(node.value) is float and node.value==30.:node.value=90.
            return node
    old=DeclaredConstants().visit(old)
    assert ast.dump(old,include_attributes=False)==ast.dump(new,include_attributes=False)


@pytest.mark.parametrize("seconds,states,cameras,error,expected", [(45.,900,450,owner.BUDGET_ERROR,True),
    (20.,400,200,owner.BUDGET_ERROR,False),(45.,899,450,owner.BUDGET_ERROR,False),
    (45.,900,449,owner.BUDGET_ERROR,False),(45.,900,450,"wrong",False)])
def test_budget_classifier_preserves_exact_predeclared_counts(seconds,states,cameras,error,expected):
    value=dict(status="failed",error=error,capture_contract=dict(physics_hz=20.,camera_hz=10.,maximum_total_duration_sec=seconds))
    assert owner.classify_bounded_capture(value,states,cameras)==expected


def test_full_batch_disk_reserve_is_predeclared():
    required=owner.base.FLOOR+3*owner.base.ARM_RESERVE+owner.base.SHARED_RESERVE
    assert owner.disk_check(required)["required_bytes"]==required
    with pytest.raises(RuntimeError):owner.disk_check(required-1)


def report_fixture(monkeypatch,tmp_path):
    core=dict(ownership_passed=True,bounded_capture_complete=True,
        predeclared_motion_core_screen_passed_before_body_swept_clearance=True,
        native_state_chain_passed=True,camera_frame_chain_passed=True,exact_camera_native_timestamp_match=True,
        predeclared_observed_bundle_to_ACK_wall_latency_screen_passed=False)
    control=dict(desired_speed_mps=2.5,throttle=.1,brake=0.,steer=0.)
    audit=dict(controller_target_cap_mps=2.4,original_model_target_mps=2.5,governed_controller_target_mps=2.4,
        explicit_controller_calibration_intervention=True,observed_speed_clipped=False,observed_speed_mps=2.4,
        one_fresh_camera_update_one_PI_integration=True,proposed_normal_tracker_control=dict(control))
    command=dict(learned_control_proposed=True,held_between_camera_frames=False,total_seconds=.08,
        application=dict(learned_control_applied=True),ack_receipt_sequence=1,
        measured_current_speed_mps=2.4,control=control,longitudinal_pi_research=audit)
    commands=[command,dict(copy.deepcopy(command),held_between_camera_frames=True,ack_receipt_sequence=2)]
    states=[dict(vx=2.4,vy=0.,world_velocity_carla=(2.4,0.,0.),control_transport=dict(expected_receipt_sequence=i)) for i in (1,2)]
    monkeypatch.setattr(owner,"diagnostic_core",lambda *_:dict(core))
    monkeypatch.setattr(owner,"jsonl",lambda p:commands if p.name=="learned_control.jsonl" else states)
    monkeypatch.setattr(owner,"body_report",lambda *_:dict(status="PASS",body_and_swept_clearance_passed=True))
    return core,commands,states


def test_driving_pass_cannot_hide_separate_wall_failure(monkeypatch,tmp_path):
    report_fixture(monkeypatch,tmp_path)
    with owner.scoped_trial(0):result=owner.diagnostic_report(tmp_path,{}, {})
    assert result["learned_driving_screen_passed"] is True
    assert result["wall_100ms_failure_is_separate_and_retained"] is True
    assert result["full_sensor_callback_to_ACK_100ms_approved"] is False


@pytest.mark.parametrize("failure", ["fresh_stop","held_stop","base_speed","actor_speed","policy_deadline",
    "body","controller_audit","missing_receipt"])
def test_each_new_gate_keeps_whole_failed_denominator(monkeypatch,tmp_path,failure):
    core,commands,states=report_fixture(monkeypatch,tmp_path)
    if failure in ("fresh_stop","held_stop"):
        row=commands[0 if failure=="fresh_stop" else 1]
        row["control"]["desired_speed_mps"]=0.
        row["longitudinal_pi_research"].update(original_model_target_mps=0.,governed_controller_target_mps=0.)
        row["longitudinal_pi_research"]["proposed_normal_tracker_control"]=dict(row["control"])
    elif failure=="base_speed":states[-1]["vx"]=2.500001
    elif failure=="actor_speed":states[-1]["world_velocity_carla"]=(2.700001,0.,0.)
    elif failure=="policy_deadline":commands[0]["total_seconds"]=.100001
    elif failure=="body":monkeypatch.setattr(owner,"body_report",lambda *_:dict(status="FAIL",body_and_swept_clearance_passed=False))
    elif failure=="controller_audit":commands[0]["longitudinal_pi_research"]["observed_speed_clipped"]=True
    elif failure=="missing_receipt":commands[0].pop("longitudinal_pi_research")
    with owner.scoped_trial(0):result=owner.diagnostic_report(tmp_path,{}, {})
    assert result["learned_driving_screen_passed"] is False
    assert result["full_post_handoff_speed_denominator"]==2
    if failure in ("fresh_stop","held_stop"):
        assert result["applied_original_model_stop_target_total_commands"]==1
        assert result["applied_original_model_stop_target_fresh_commands"]==(failure=="fresh_stop")
        assert result["applied_original_model_stop_target_held_commands"]==(failure=="held_stop")


def test_declare_creates_all_three_exact_plans_before_any_output_and_rejects_overwrite(monkeypatch,tmp_path):
    monkeypatch.setattr(owner,"ROOT",tmp_path)
    monkeypatch.setattr(owner,"disk_check",lambda *_:None)
    monkeypatch.setattr(owner.base,"build_plan",lambda run_id,_pi:dict(owner.fixed_contract(run_id,True)))
    result=owner.declare_batch("v1")
    output,batch=owner.batch_paths("v1")
    assert batch.is_file() and not output.exists()
    assert len(result["trials"])==3
    for row in result["trials"]:
        assert owner.digest(row["declaration_path"])==row["declaration_sha256"]
        assert owner.read_json(row["declaration_path"])["trial_index"]==row["trial_index"]
    with pytest.raises(RuntimeError):owner.declare_batch("v1")


def test_batch_summary_retains_missing_failure_and_joint_wall_requirement(monkeypatch,tmp_path):
    monkeypatch.setattr(owner,"ROOT",tmp_path)
    monkeypatch.setattr(owner,"verify_batch",lambda *_:{})
    for trial,wall,driving in ((0,True,True),(1,False,True)):
        with owner.scoped_trial(trial):output,_=owner.paths("v1")
        output.mkdir(parents=True)
        owner.write_new(output/"diagnostic_report.json",dict(status="PASS_BASIC_DRIVING_PRECURSOR",
            learned_driving_screen_passed=driving,predeclared_observed_bundle_to_ACK_wall_latency_screen_passed=wall))
    result=owner.batch_summary("v1","a"*64)
    assert len(result["all_three_trials"])==3
    assert result["all_three_trials"][2]["status"]=="NOT_EXECUTED_OR_NO_DIAGNOSTIC"
    assert result["all_trials_motion_body_policy_AND_wall100ms_passed"] is False


@pytest.mark.parametrize("corruption", [None,"live_code","archived_code","live_API","archived_API","live_asset","archived_asset"])
def test_post_body_source_closure_rejects_real_file_corruption(monkeypatch,tmp_path,corruption):
    root=tmp_path/"workspace";carla=tmp_path/"carla";output=tmp_path/"output"
    monkeypatch.setattr(owner,"ROOT",root);monkeypatch.setattr(owner,"CARLA_ROOT",carla)
    sources=(root/"module.py",carla/"api.egg",root/"route.json")
    archives=(output/"source/module.py",output/"source_external/api.egg",output/"source/route.json")
    for source,archive in zip(sources,archives):
        source.parent.mkdir(parents=True,exist_ok=True);archive.parent.mkdir(parents=True,exist_ok=True)
        source.write_bytes(b"original");archive.write_bytes(b"original")
    plan=dict(source_sha256={"module.py":owner.digest(sources[0])},
        external_source_sha256={str(sources[1]):owner.digest(sources[1])},
        asset_sha256={str(sources[2]):owner.digest(sources[2])})
    if corruption is not None:
        index={"live_code":0,"archived_code":0,"live_API":1,"archived_API":1,"live_asset":2,"archived_asset":2}[corruption]
        (sources if corruption.startswith("live") else archives)[index].write_bytes(b"corrupted")
        with pytest.raises(RuntimeError,match="post-body"):owner.verify_derived_source_closure(output,plan)
    else:owner.verify_derived_source_closure(output,plan)
