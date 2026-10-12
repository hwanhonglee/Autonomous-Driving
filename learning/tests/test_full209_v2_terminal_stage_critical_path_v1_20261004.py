"""HH_261004 - Synthetic scalar-only tests for exact bridge-stage and GC-overlap analysis; no drive or Torch execution."""

import copy

import pytest

from scripts.e2e import analyze_full209_v2_terminal_stage_critical_path_v1_20261004 as audit


def receipt(stages=None,status="COMPLETED"):
    names=stages or audit.STAGES
    value=dict(schema=audit.TIMING_SCHEMA,frozen_bridge_source_sha256="e4c0118de9d9e420890d1c0f11df4fe469cf9cdb7e969f1cd96d2448798c40ab",
        original_bridge_AST_inverse_verified=True,original_policy_deadline_seconds=.1,frame_timestamp_ns=1_000_000_000,
        status=status,exception_type=None if status=="COMPLETED" else "ContractError",
        call_started_monotonic_ns=1000,call_finished_monotonic_ns=1000+10*len(names),total_observed_ns=10*len(names),
        stages=[dict(stage=name,start_monotonic_ns=1000+10*i,finish_monotonic_ns=1010+10*i,elapsed_ns=10) for i,name in enumerate(names)])
    for name in ("original_policy_deadline_unchanged","stage_work_inside_original_policy_deadline","scalar_receipt_finalization_also_inside_policy_deadline",
        "stage_sum_excludes_final_scalar_receipt_attachment_overhead","original_inputs_math_checks_order_exceptions_unchanged","timing_only_research"):value[name]=True
    for name in ("source_rehashed_during_call","numerical_outputs_changed","weights_changed","learned_STOP_claim","vehicle_control_approved"):value[name]=False
    value["gc_observation"]=dict(same_thread_only=True,maximum_events=32,bounded_event_overflow_disclosed=True,
        collection_behavior_changed=False,collection_forced=False,collection_disabled=False,thresholds_changed=False,
        events=[],actual_start_count=0,actual_stop_count=0,overflow_start_count=0,unmatched_start_count=0)
    return value


def test_terminal_ordinary_and_failed_prefix_kept():
    assert audit.validate_timing(receipt())["branch"]=="terminal"
    assert audit.validate_timing(receipt(audit.ORDINARY))["branch"]=="ordinary"
    for n in range(1,len(audit.STAGES)+1):
        value=receipt(audit.STAGES[:n],"FAILED");audit.validate_timing(value)
        assert value["exception_type"]=="ContractError"


@pytest.mark.parametrize("fault",("schema","source","deadline","math","duplicate","gap","elapsed","tail","exception","GC_outside","GC_forced","GC_overflow_negative"))
def test_malformed_or_changed_actual_receipt_denies(fault):
    value=receipt()
    if fault=="schema":value["schema"]="wrong"
    elif fault=="source":value["frozen_bridge_source_sha256"]="f"*64
    elif fault=="deadline":value["original_policy_deadline_seconds"]=.11
    elif fault=="math":value["numerical_outputs_changed"]=True
    elif fault=="duplicate":value["stages"][-1]["stage"]="entry"
    elif fault=="gap":value["stages"][1]["start_monotonic_ns"]+=1
    elif fault=="elapsed":value["stages"][1]["elapsed_ns"]+=1
    elif fault=="tail":value["call_finished_monotonic_ns"]+=1
    elif fault=="exception":value["exception_type"]="RuntimeError"
    elif fault=="GC_outside":value["gc_observation"]["events"]=[dict(start_monotonic_ns=999,finish_monotonic_ns=1001)]
    elif fault=="GC_forced":value["gc_observation"]["collection_forced"]=True
    elif fault=="GC_overflow_negative":value["gc_observation"]["overflow_start_count"]=-1
    with pytest.raises(RuntimeError):audit.validate_timing(value)


def test_GC_overlap_union_not_nested_double_count():
    value=receipt();gc=value["gc_observation"]
    gc.update(events=[dict(start_monotonic_ns=1055,finish_monotonic_ns=1075),dict(start_monotonic_ns=1060,finish_monotonic_ns=1070)],actual_start_count=2,actual_stop_count=2)
    result=audit.validate_timing(value)
    assert result["closed_GC_interval_union_ms"]==20/1e6 and result["complete_gc_observation"]
    assert result["stage_GC_overlap_ms"]["observationserialization"]==5/1e6
    assert result["stage_GC_overlap_ms"]["witnessevaluation"]==10/1e6


def test_GC_overflow_and_unmatched_not_falsely_closed():
    value=receipt();value["gc_observation"].update(events=[dict(start_monotonic_ns=1050,finish_monotonic_ns=None)],
        actual_start_count=2,actual_stop_count=0,overflow_start_count=1,unmatched_start_count=1)
    result=audit.validate_timing(value)
    assert not result["complete_gc_observation"] and result["closed_GC_interval_union_ms"]==0


def fixture():
    value=receipt();command=dict(inference_frame=10,held_between_camera_frames=False,
        navigation_terminal_timing_attachment_outside_original_policy_timer=True,navigation_terminal_timing_research=value,
        navigation_terminal_research=dict(branch="authenticated_navigation_terminal_V2",frame_timestamp_ns=1_000_000_000))
    path=dict(frame=10,raw_NN_finished_ns=990,decoded_plan_ready_ns=1110,first_callback_plan_ms=110.,first_callback_ACK_ms=112.,
        plan100ms_failed=True,ACK100ms_failed=True)
    cameras=[dict(frame=10,timestamp=1.)]
    return command,path,cameras


def test_all_fresh_and_held_aliases_preserved():
    command,path,cameras=fixture();held=copy.deepcopy(command);held["held_between_camera_frames"]=True
    rows,aliases,uncalled=audit.timing_rows([command,held],[path],cameras)
    assert len(rows)==len(aliases)==1 and uncalled==[]
    assert rows[0]["plan100ms_failed"] and rows[0]["NN_to_bridge_started_ms"]==10/1e6


@pytest.mark.parametrize("fault",("missing","held_mutated","wrong_frame","clock","duplicate_fresh","branch","unbound_held"))
def test_missing_or_mismatched_complete_fresh_timing_denies(fault):
    command,path,cameras=fixture();commands=[command]
    if fault=="missing":command["navigation_terminal_timing_research"]=None
    elif fault=="held_mutated":
        held=copy.deepcopy(command);held["held_between_camera_frames"]=True;held["navigation_terminal_timing_research"]["exception_type"]="forged";commands.append(held)
    elif fault=="wrong_frame":command["inference_frame"]=11
    elif fault=="clock":path["raw_NN_finished_ns"]=1010
    elif fault=="duplicate_fresh":commands.append(copy.deepcopy(command))
    elif fault=="branch":command["navigation_terminal_research"]["branch"]="ordinary_exact_delegate"
    elif fault=="unbound_held":
        held=copy.deepcopy(command);held["held_between_camera_frames"]=True
        held["navigation_terminal_timing_research"]["frame_timestamp_ns"]+=1;commands.append(held)
    with pytest.raises(RuntimeError):audit.timing_rows(commands,[path],cameras)


def test_failed_and_uncalled_steps_retained_without_fabricated_plan():
    command,path,cameras=fixture();command["navigation_terminal_timing_research"]=receipt(audit.STAGES[:2],"FAILED")
    command["emergency_reason"]="genuine synthetic source-binding failure"
    skipped=dict(inference_frame=12,navigation_terminal_timing_attachment_outside_original_policy_timer=True,navigation_terminal_timing_research=None,emergency_reason="before decoder")
    rows,aliases,uncalled=audit.timing_rows([command,skipped],[],cameras)
    assert rows[0]["status"]=="FAILED" and "first_callback_plan_ms" not in rows[0]
    assert len(uncalled)==1 and aliases==[]
