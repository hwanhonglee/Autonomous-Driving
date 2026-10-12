"""HH_261004 - Synthetic-only tests for faithful complete-source, native-goal and quota timing reconstruction."""

import ast
import copy
import math

import pytest

from scripts.e2e import analyze_full209_actual_callback_critical_path_v1_20261004 as audit


def source_fixture():
    # HH_261004 - Six synthetic bundles provide a five-slot causal history, not actual sensor or timing evidence.
    value=dict(schema=audit.LIVE_SCHEMA+".callback_receipts",errors=[],inflight_callbacks=0,
        all_callbacks_including_bootstrap_or_unconsumed_preserved=True,
        camera_bindings=[dict(role=role,actor_id=100+i) for i,role in enumerate(audit.ROLES)],
        callbacks=[],bundles=[],policy_stage_marks=[])
    cameras=[]
    for ordinal in range(6):
        frame=10+2*ordinal;stamp=1_000_000_000+ordinal*100_000_000;base=10_000+ordinal*1000
        selected=[]
        for i,role in enumerate(audit.ROLES):
            row=dict(role=role,actor_id=100+i,native_frame=frame,image_object_id=1000+ordinal*6+i,
                receipt_sequence=len(value["callbacks"])+1,source_byte_count=640*360*4,
                original_queue_put_calls=1,original_queue_put_returned=True,original_queue_put_error=None,
                receipt_error=None,source_pixels_changed=False,source_timestamp_changed=False,
                callback_enter_monotonic_ns=base+10+i,queue_put_started_monotonic_ns=base+16+i,
                queue_put_finished_monotonic_ns=base+22+i,source_SHA_finished_monotonic_ns=base+28+i,
                source_image_timestamp_simulation_ns=stamp,source_BGRA_encoding_prefixed_sha256=("%064x"%(ordinal*6+i+1)))
            value["callbacks"].append(row)
            selected.append({name:row[name] for name in ("role","native_frame","image_object_id","source_image_timestamp_simulation_ns")})
        value["bundles"].append(dict(native_frame=frame,cameras=selected,bundle_binding_finished_monotonic_ns=base+60))
        cameras.append(dict(frame=frame,timestamp=stamp/1e9,camera_order=list(audit.ROLES),
            source_timestamps={role:stamp/1e9 for role in audit.ROLES}))
    stamp=value["callbacks"][-1]["source_image_timestamp_simulation_ns"];base=15_000
    for stage,clock in zip(audit.POLICY_STAGES,(base+90,base+100,base+110)):
        value["policy_stage_marks"].append(dict(frame_timestamp_simulation_ns=stamp,stage=stage,
            original_policy_step_statements_changed=False,policy_object_id=400,stage_finished_monotonic_ns=clock))
    history=[]
    for ordinal in range(1,6):
        selected=value["callbacks"][ordinal*6:ordinal*6+6]
        history.append(dict(timestamp_ns=selected[0]["source_image_timestamp_simulation_ns"],
            camera_sha256=[row["source_BGRA_encoding_prefixed_sha256"] for row in selected]))
    command=dict(learned_control_proposed=True,held_between_camera_frames=False,inference_frame=20,command_frame=20,
        ack_receipt_sequence=1,owned_camera_history=dict(source_camera_frames=history),
        navigation_terminal_research=dict(branch="ordinary_exact_delegate"))
    wall=dict(ACK_sequence=1,command_frame=20,inference_frame=20,ACK_server_accepted=True,learned_control_proposed=True,
        held_between_camera_frames=False,control_source="learned_trajectory_tracker",proposal_started_monotonic_ns=base+80,
        proposal_finished_monotonic_ns=base+115,ACK_started_monotonic_ns=base+120,ACK_finished_monotonic_ns=base+130,
        observed_bundle=dict(exact_bundle_delivery_monotonic_ns=base+65))
    return value,cameras,[command],[wall]


def native_fixture():
    states=[];cameras=[];marks=[]
    for ordinal,(position,phase) in enumerate(((0.,"warmup"),(5.,"driving"),(8.,"driving"))):
        frame=100+ordinal;base=ordinal*1000
        state=dict(frame=frame,x=position,y=0.,route_progress_m=position,capture_phase=phase)
        if ordinal==2:
            state["catalog_goal_cleanup_receipt"]=dict(command_frame=frame,original_catalog_two_predicates_passed=True,
                status="ACKNOWLEDGED",learned_STOP_proven=False)
        states.append(state)
        camera=ordinal%2==0
        if camera:cameras.append(dict(frame=frame))
        for i,stage in enumerate(audit.NATIVE_STAGES if camera else audit.NATIVE_STAGES[:7]):
            mark=dict(stage=stage,tick_index=ordinal,native_frame=None if i==0 else frame,capture_phase=phase,
                original_native_order_or_math_changed=False,stage_finished_monotonic_ns=base+i+1)
            if stage in ("agent_branch_started","agent_branch_finished"):mark["agent_run_step_invoked"]=ordinal==1
            marks.append(mark)
    return dict(native_stage_marks=marks),states,cameras,dict(route=[dict(x=0.,y=0.),dict(x=10.,y=0.)])


def critical_fixture():
    clocks=dict(first=1_000_000,last=2_000_000,outer=3_000_000,quota_start=4_000_000,quota_end=5_000_000,
        inner=6_000_000,delivery=7_000_000,build=8_000_000,outer_end=9_000_000,proposal=10_000_000,
        input=20_000_000,NN=50_000_000,plan=110_000_000,proposal_end=111_000_000,ACK_start=112_000_000,ACK=113_000_000)
    row=dict(frame=100,ACK_sequence=1,branch="authenticated_navigation_terminal_V2",first_callback_ns=clocks["first"],
        last_callback_ns=clocks["last"],input_ready_ns=clocks["input"],raw_NN_finished_ns=clocks["NN"],
        decoded_plan_ready_ns=clocks["plan"],ACK_finished_ns=clocks["ACK"],first_callback_plan_ms=109.,first_callback_ACK_ms=112.)
    native={100:{stage:dict(stage_finished_monotonic_ns=clocks[name]) for stage,name in (("exact_bundle_started","outer"),("exact_bundle_finished","outer_end"))}}
    quota=dict(schema=audit.LIVE_SCHEMA+".quota_walk_receipts",rows=[dict(sequence=1,native_frame=100,status="PASS",complete_walk=True,
        timestamp_simulation_ns=1_000_000_000,capture_phase="driving",returned_regular_file_bytes=200,regular_file_bytes=200,
        regular_files=2,source_payload_read=False,cached_or_incremental_byte_count_used=False,
        walk_started_monotonic_ns=clocks["quota_start"],walk_finished_monotonic_ns=clocks["quota_end"])])
    cameras=[dict(frame=100,timestamp=1.,capture_phase="driving")]
    wall=dict(ACK_sequence=1,observed_bundle=dict(exact_bundle_wait_started_monotonic_ns=clocks["inner"],
        exact_bundle_delivery_monotonic_ns=clocks["delivery"],observed_frame_build_finished_monotonic_ns=clocks["build"]),
        proposal_started_monotonic_ns=clocks["proposal"],proposal_finished_monotonic_ns=clocks["proposal_end"],ACK_started_monotonic_ns=clocks["ACK_start"])
    return [row],native,quota,cameras,[wall]


def test_synthetic_source_complete_join_preserves_unselected_repeated_frame():
    value,cameras,commands,walls=source_fixture()
    repeated=copy.deepcopy(value["callbacks"][-6]);repeated.update(receipt_sequence=37,image_object_id=9000,callback_enter_monotonic_ns=1)
    value["callbacks"].append(repeated)
    rows,census=audit.source_join(value,cameras,commands,walls)
    assert census==dict(all_callbacks=37,selected_callbacks=36,unselected_callbacks=1,fresh_NN_count=1,
        exact_past_source_SHA_checks=30,earliest_or_latest_repeated_callback_selected_by_latency=False)
    assert rows[0]["first_callback_ns"]==15_010
    assert rows[0]["selected_callback_receipt_sequences"]==list(range(31,37))


@pytest.mark.parametrize("fault",("same_object","wrong_actor","source_stamp","selected_object","historical_SHA","future_history","same_policy","plan_clock","ACK_source","drain","changed_math","put_count"))
def test_synthetic_source_faults_deny(fault):
    value,cameras,commands,walls=source_fixture()
    if fault=="same_object":
        duplicate=copy.deepcopy(value["callbacks"][-1]);duplicate["receipt_sequence"]=37;value["callbacks"].append(duplicate)
    elif fault=="wrong_actor":value["callbacks"][0]["actor_id"]=999
    elif fault=="source_stamp":value["callbacks"][0]["source_image_timestamp_simulation_ns"]+=1
    elif fault=="selected_object":value["bundles"][0]["cameras"][0]["image_object_id"]=-1
    elif fault=="historical_SHA":commands[0]["owned_camera_history"]["source_camera_frames"][0]["camera_sha256"][0]="f"*64
    elif fault=="future_history":commands[0]["owned_camera_history"]["source_camera_frames"][0]["timestamp_ns"]=2_000_000_000
    elif fault=="same_policy":value["policy_stage_marks"][-1]["policy_object_id"]+=1
    elif fault=="plan_clock":value["policy_stage_marks"][-1]["stage_finished_monotonic_ns"]=100_000
    elif fault=="ACK_source":walls[0]["control_source"]="teacher"
    elif fault=="drain":value["all_callbacks_including_bootstrap_or_unconsumed_preserved"]=False
    elif fault=="changed_math":value["policy_stage_marks"][0]["original_policy_step_statements_changed"]=True
    elif fault=="put_count":value["callbacks"][0]["original_queue_put_calls"]=2
    with pytest.raises((RuntimeError,KeyError)):audit.source_join(value,cameras,commands,walls)


def test_synthetic_genuine_final_goal_exception_not_learned_STOP():
    value,states,cameras,route=native_fixture()
    rows,groups=audit.native_paths(value,states,cameras,route)
    assert len(rows)==3 and len(value["native_stage_marks"])==7*3+2*2
    assert [row["agent_run_step_invoked"] for row in rows]==[False,True,False]
    assert set(groups)=={100,101,102}
    assert rows[-1]["outer_exact_provider_ms"]==1/1e6


@pytest.mark.parametrize("fault",("final_agent","warmup_agent","early_goal","no_goal","learned_STOP","duplicate_stage","missing_stage","phase","clock"))
def test_synthetic_native_faults_deny(fault):
    value,states,cameras,route=native_fixture()
    marks=value["native_stage_marks"]
    if fault=="final_agent":marks[-5]["agent_run_step_invoked"]=True
    elif fault=="warmup_agent":marks[4]["agent_run_step_invoked"]=True
    elif fault=="early_goal":states[1].update(x=8.,route_progress_m=8.)
    elif fault=="no_goal":states[-1].update(x=5.,route_progress_m=5.)
    elif fault=="learned_STOP":states[-1]["catalog_goal_cleanup_receipt"]["learned_STOP_proven"]=True
    elif fault=="duplicate_stage":marks.append(copy.deepcopy(marks[0]))
    elif fault=="missing_stage":marks.pop()
    elif fault=="phase":marks[1]["capture_phase"]="driving"
    elif fault=="clock":marks[1]["stage_finished_monotonic_ns"]=0
    with pytest.raises(RuntimeError):audit.native_paths(value,states,cameras,route)


def test_synthetic_timing_failure_retained_and_exact_segments():
    rows,walks=audit.critical_paths(*critical_fixture())
    assert len(rows)==len(walks)==1 and rows[0]["plan100ms_failed"] and rows[0]["ACK100ms_failed"]
    assert rows[0]["NN_decoded_plan_ms"]==60. and rows[0]["quota_walk_ms"]==1.
    assert walks[0]["regular_file_bytes"]==200 and rows[0]["quota_overlap_after_first_callback_ms"]==1.


@pytest.mark.parametrize("fault",("cache","bytes","stamp","phase","sequence","quota_clock","inner_clock","missing_quota"))
def test_synthetic_quota_or_stage_faults_deny(fault):
    rows,native,quota,cameras,walls=critical_fixture();q=quota["rows"][0]
    if fault=="cache":q["cached_or_incremental_byte_count_used"]=True
    elif fault=="bytes":q["returned_regular_file_bytes"]+=1
    elif fault=="stamp":q["timestamp_simulation_ns"]+=1
    elif fault=="phase":q["capture_phase"]="warmup"
    elif fault=="sequence":q["sequence"]=2
    elif fault=="quota_clock":q["walk_finished_monotonic_ns"]=0
    elif fault=="inner_clock":walls[0]["observed_bundle"]["exact_bundle_wait_started_monotonic_ns"]=1
    elif fault=="missing_quota":quota["rows"]=[]
    with pytest.raises(RuntimeError):audit.critical_paths(rows,native,quota,cameras,walls)


def test_linear_percentile_and_complete_denominator():
    value=audit.stats([1.,2.,3.,4.])
    assert value.pop("p95_linear")==pytest.approx(3.85)
    assert value==dict(count=4,minimum=1.,median=2.5,maximum=4.)
    for values in ([],[math.inf],[math.nan],[True]):
        with pytest.raises(RuntimeError):audit.stats(values)


@pytest.mark.parametrize("content",('{"x":1,"x":2}', '{"x":NaN}'))
def test_strict_json_and_jsonl_duplicate_nonfinite_deny(tmp_path,content):
    target=tmp_path/"synthetic.json";target.write_text(content)
    with pytest.raises(RuntimeError):audit.json_file(target)
    with pytest.raises(RuntimeError):audit.json_lines(target)


def test_bounded_json_and_alias_deny(tmp_path):
    source=tmp_path/"synthetic.json";source.write_text('{"a":1}')
    with pytest.raises(RuntimeError):audit.json_file(source,limit=2)
    alias=tmp_path/"alias.json";alias.symlink_to(source)
    with pytest.raises(RuntimeError):audit.json_file(alias)


def test_cli_requires_explicit_scope_and_existing_output_denies(monkeypatch,tmp_path):
    monkeypatch.setattr(audit,"OUTPUT",tmp_path)
    with pytest.raises(SystemExit):audit.main([])
    with pytest.raises(RuntimeError):audit.main(["--execute-readonly"])


def test_source_has_no_torch_carla_NN_or_runtime_activation():
    tree=ast.parse(audit.HERE.read_text())
    imported=[alias.name for node in ast.walk(tree) if isinstance(node,ast.Import) for alias in node.names]
    assert not any(name.startswith(("torch","carla","numpy","PIL")) for name in imported)
    assert "original_FAIL_not_regraded=True" in audit.HERE.read_text()
    assert "NN_forward_executed=False" in audit.HERE.read_text()
