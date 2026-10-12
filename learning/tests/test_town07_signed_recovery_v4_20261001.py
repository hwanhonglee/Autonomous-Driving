"""HH_261001 - Test fixed rolling membership and complete retained native data with no simulator jobs."""

import ast
import copy
from pathlib import Path
from unittest import mock

import pytest

from scripts.e2e import run_owned_town07_signed_recovery_v4_20261001 as v4


def complete(clock_drift=False):
    task=v4.RollingRecoveryTaskV4(v4.fixed_contract("v4","train_04"))
    states,cameras=[],[]
    for frame in range(1,966):
        phase="stationary_warmup" if frame<=24 else "driving" if frame<=835 else "stationary_tail"
        stamp=frame*.05000000074505806 if clock_drift else frame*.05
        latest,_=task.observe(frame=frame,timestamp=stamp,phase=phase,progress=frame*.1,
            base_stop_reason=None,event_count=0,observed_free_disk_bytes=10*1024**3,
            native_actor_speed_mps=2.4,rear_base_CTE_m=0.,observed_output_bytes=0)
        states.append(dict(frame=frame,timestamp=stamp,capture_phase=phase,moving_task=latest))
        if frame%2:
            camera=dict(frame=frame,timestamp=stamp,capture_phase=phase,camera_order=list(v4.task.original.CAMERAS),
                source_timestamps={name:stamp for name in v4.task.original.CAMERAS},
                images={name:"real.jpg" for name in v4.task.original.CAMERAS})
            task.saved_camera(camera);cameras.append(camera)
    return task,states,cameras


@pytest.mark.parametrize("clock_drift",[False,True])
def test_all302_exact_rolling_anchors_actual_history_future_tail_and_full965_native(clock_drift):
    with v4.activated(): value,states,cameras=complete(clock_drift)
    result=value.result(tail_states=130)
    assert result["task_complete"] and len(value.selected)==302
    assert len(states)==965 and len(cameras)==483
    assert value.first["frame"]==105 and value.last["frame"]==707
    assert value.support["frame"]==835
    assert value.history_support["status"]=="PASS"
    assert [r["frame"] for r in value.history_support["camera_observations"]]==list(range(87,106,2))
    assert value.history_support["preceding_native_guard"]["frame"]==86
    assert value.history_support["preceding_native_guard"]["phase"]=="driving"
    assert value.history_support["preceding_guard_may_be_stationary_warmup"] is False
    assert result["selected_driving_relative_time_ns"]==[4_000_000_000,34_100_000_000]
    assert value.last["frame"]-value.first["frame"]==602
    assert abs(value.last["timestamp_ns"]-value.first["timestamp_ns"]-30_100_000_000)<=602
    assert len([r for r in states if r["capture_phase"]=="stationary_tail"])==130
    assert all(not r["moving_task_selected"] for r in cameras if r["frame"]<105 or r["frame"]>707)
    assert states[0]["frame"]==1 and states[-1]["frame"]==965


def test_same_full11_grid_fresh_cohort_and_explicit_not_startup_training():
    assert len(v4.CASES)==11 and not {r["seed"] for r in v4.CASES}&{r["seed"] for r in v4.v3.CASES}
    with v4.activated():
        result=v4.fixed_contract("v4","train_00")
        assert result["schema"]==v4.TASK_SCHEMA and result["seed"]==2026100710
        assert result["dataset_training_scope"]=="PROSPECTIVE_ROLLING_RECOVERY_ONLY_NOT_STARTUP_TRAIN"
        assert result["startup_training_or_startup_success_claimed"] is False
        assert result["per_case_adaptive_or_quality_crop"] is False and result["all_raw_startup_and_tail_states_retained"]
        assert result["expected_native_states"]==965 and result["expected_full_raw_sweeps"]==964
        assert result["expected_camera_bundles"]==483 and result["expected_JPEG_count"]==2898
        assert result["controller"]["gear"]==1 and result["controller"]["manual_gear_shift"] is True
        assert result["controller"]["launch_throttle"]==.11
        assert result["controller"]["post_handoff_throttle_ramp_per_second"]==.025
        assert result["controller"]["native_acceleration_qa_limit_mps2"]==3.
        assert result["first_history_driving_relative_time_ns"]==[3_100_000_000,4_000_000_000]
        assert result["minimum_selected_span_ns"]==30_000_000_000
        assert result["required_future_support_ns"]==6_400_000_000
        assert "training_data_approved" not in result


def test_frozen_moving_task_AST_exact_except_seven_predeclared_membership_literals():
    original=next(row for row in ast.parse(Path(v4.task.__file__).read_text()).body if isinstance(row,ast.ClassDef) and row.name=="MovingTask")
    node,counts=v4.rolling_task_ast();node=copy.deepcopy(node);node.name="MovingTask"
    assert counts==dict(close_end=1,selection_first=1,selection_last=1,preceding_phase=1,preceding_flag=1,result_first=1,result_last=1)
    for method in node.body:
        if not isinstance(method,ast.FunctionDef):continue
        for item in ast.walk(method):
            if method.name=="observe" and isinstance(item,ast.Compare) and isinstance(item.left,ast.Name) \
                    and item.left.id=="relative_frame" and len(item.ops)==1 and isinstance(item.ops[0],ast.Gt) \
                    and item.comparators[0].value==682: item.comparators[0].value=620
            if method.name=="observe" and isinstance(item,ast.keyword) and item.arg=="camera_selection_eligible":
                for part in ast.walk(item.value):
                    if isinstance(part,ast.Constant) and type(part.value) is int and part.value in (80,682):part.value=18 if part.value==80 else 620
            if method.name=="saved_camera" and isinstance(item,ast.Compare) and isinstance(item.left,ast.Subscript) \
                    and isinstance(item.left.value,ast.Name) and item.left.value.id=="guard":
                for part in item.comparators:
                    if isinstance(part,ast.Constant) and part.value=="driving":part.value="stationary_warmup"
            if method.name=="saved_camera" and isinstance(item,ast.keyword) and item.arg=="preceding_guard_may_be_stationary_warmup":item.value.value=True
            if method.name=="result" and isinstance(item,ast.keyword) and item.arg=="selected_driving_relative_time_ns":
                for part in ast.walk(item.value):
                    if isinstance(part,ast.Constant) and part.value in (4_000_000_000,34_100_000_000):part.value=900_000_000 if part.value==4_000_000_000 else 31_000_000_000
    assert ast.dump(node,include_attributes=False)==ast.dump(original,include_attributes=False)


def test_diagnostic_reuses_every_original_audit_with_only_four_exact_count_literals_changed():
    old=next(row for row in ast.parse(Path(v4.previous.__file__).read_text()).body if isinstance(row,ast.FunctionDef) and row.name=="diagnostic_report")
    # HH_261001 - The generated code constants expose exactly the unchanged source literal replacements.
    assert v4._diagnostic.__code__.co_consts.count(965)==1 and v4._diagnostic.__code__.co_consts.count(483)==1
    assert 903 not in v4._diagnostic.__code__.co_consts and 452 not in v4._diagnostic.__code__.co_consts
    assert sum(isinstance(r,ast.Constant) and type(r.value) is int and r.value==903 for r in ast.walk(old))==2
    assert sum(isinstance(r,ast.Constant) and type(r.value) is int and r.value==452 for r in ast.walk(old))==2


@pytest.mark.parametrize("change,reason",[({"native_actor_speed_mps":2.71},"recovery_native_actor_speed_limit"),
    ({"rear_base_CTE_m":1.001},"recovery_rear_base_CTE_limit"),({"event_count":1},"recovery_collision_or_lane_event")])
def test_no_guard_relaxation_or_startup_fault_filter_in_runtime(change,reason):
    with v4.activated():
        value=v4.RollingRecoveryTaskV4(v4.fixed_contract("v4","train_04"))
        values=dict(frame=1,timestamp=.05,phase="stationary_warmup",progress=0.,base_stop_reason=None,event_count=0,
            observed_free_disk_bytes=10*1024**3,native_actor_speed_mps=0.,rear_base_CTE_m=0.,observed_output_bytes=0)
        values.update(change);value.observe(**values)
        assert value.failure==reason


@pytest.mark.parametrize("failure",[False,True])
def test_every_outer_inner_class_callback_module_binding_restores(failure):
    saved_task={k:getattr(v4.task,k) for k in ("CASES","SCHEMA","fixed_contract","MovingTask")}
    saved_owner={k:getattr(v4.previous,k) for k in ("MODULE","SCHEMA","CASE_IDS","runtime_sources","declare_batch","diagnostic_report")}
    base=v4.previous.worker_module.base
    blank,check,policy=base._blank_brake_control,base.checked_agent_initialization_observation,v4.previous.worker_module.v2.CombinedControl
    try:
        with v4.activated():
            assert v4.task.MovingTask is v4.RollingRecoveryTaskV4
            assert v4.previous.worker_module.v2.CombinedControl is v4.v3.FirstGearRecoveryLaunchControl
            assert base.checked_agent_initialization_observation is v4.v3.checked_first_gear_initialization_observation
            with pytest.raises(RuntimeError):
                with v4.activated():pass
            if failure:raise OSError("retained")
    except OSError:assert failure
    assert all(getattr(v4.task,k) is value for k,value in saved_task.items())
    assert all(getattr(v4.previous,k) is value for k,value in saved_owner.items())
    assert base._blank_brake_control is blank and base.checked_agent_initialization_observation is check
    assert v4.previous.worker_module.v2.CombinedControl is policy
    assert not v4._active and not v4.v3._active and not v4.v3.v2._active


def test_full_raw_startup_over3_is_reported_not_erased_or_training_approval(tmp_path):
    (tmp_path/"episode").mkdir()
    state=dict(frame=7,timestamp=.35,capture_phase="driving",ax=3.2,ay=0.,current_control={"actual":"retained"},
        moving_task={"driving_relative_native_frames":2})
    with mock.patch.object(v4,"_diagnostic",return_value=dict(status="CAPTURE",actual_teacher_capture_passed=True)), \
            mock.patch.object(v4.previous,"jsonl",return_value=[state]):
        result=v4.diagnostic_report(tmp_path,{}, {})
    assert len(result["full_raw_driving_acceleration_over3_faults"])==1
    assert result["full_raw_driving_acceleration_over3_faults"][0]["original_current_control"]==state["current_control"]
    assert result["outside_protected_window_faults_not_erased_or_relabelled"] is True
    assert result["startup_training_or_startup_success_claimed"] is False


def test_all_eleven_offline_initial_boxes_and_import_only_closure_preserved():
    with v4.activated():
        result=v4.v3.v2.initial_body_preflight();pins=v4.runtime_sources()
        assert result["schema"]==v4.SCHEMA+".predicted_initial_geometry" and result["status"]=="PASS"
        assert [r["seed"] for r in result["cases"]]==[r["seed"] for r in v4.CASES]
        assert pins[str(v4.V3_SOURCE.relative_to(v4.ROOT))]==v4.V3_SOURCE_SHA
        assert "tools/e2e/run_owned_town07_signed_recovery_v4_20261001.py" in pins


@pytest.mark.parametrize("run_id",["v1","v2","v3","v5","../v4"])
def test_no_old_raw_cropping_or_arbitrary_cohort_path_reuse(run_id):
    with pytest.raises(RuntimeError):v4.main(["--mode","contract","--run-id",run_id])
