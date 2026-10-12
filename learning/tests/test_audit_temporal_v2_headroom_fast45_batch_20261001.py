"""HH_261001 - Preserve full timing denominators, clock scope, native prior ACK and frozen geometry census."""

from copy import deepcopy

import pytest

from scripts.e2e import audit_temporal_v2_headroom_fast45_batch_20261001 as audit


def timing_fixture():
    command=dict(command_frame=20,inference_frame=20,ack_receipt_sequence=4,learned_control_proposed=True,
                 held_between_camera_frames=False,total_seconds=.092,raw_nn_seconds=.064,
                 owned_camera_history=dict(build_and_source_tensor_hash_seconds=.025))
    timing=dict(learned_control_proposed=True,held_between_camera_frames=False,command_frame=20,inference_frame=20,
                ACK_sequence=4,ACK_server_accepted=True,source_checkpoint_sha256=audit.CHECKPOINT_SHA,
                stream_source_sha256="a"*64,clock="time.perf_counter_ns_monotonic",policy_deadline_seconds=.1,
                policy_deadline_relaxed=False,sensor_callback_entry_observed=False,sensor_callback_entry_to_ACK_seconds=None,
                exposure_to_ACK_wall_latency_proven=False,ten_hz_end_to_end_proven=False,
                observed_bundle=dict(native_frame=20,source_image_timestamp_simulation_ns=1_000_000_000,
                    exact_bundle_delivery_monotonic_ns=1_000_000_000,
                    observed_frame_build_finished_monotonic_ns=1_001_000_000,exact_bundle_wait_seconds=.006),
                proposal_started_monotonic_ns=1_009_000_000,proposal_finished_monotonic_ns=1_101_000_000,
                ACK_started_monotonic_ns=1_101_010_000,ACK_finished_monotonic_ns=1_101_410_000,
                propose_seconds=.092,ACK_send_seconds=.0004,exact_bundle_delivery_to_ACK_seconds=.10141)
    receipt=dict(sequence=4,before_frame=20,server_accepted=True)
    return command,timing,receipt,[dict(frame=20)],[dict(frame=20,timestamp=1.)]


def test_components_exact_nonoverlap_and_separate_policy_vs_wall_failure():
    command,timing,_,_,_=timing_fixture()
    row=audit.decompose(timing,command)
    assert row["policy_100ms_passed"] is True and row["observed_delivery_to_ACK_100ms_passed"] is False
    assert row["components_ms"]["postobservation_preproposal_uninstrumented_ms"]==8.
    assert sum(row["components_ms"].values())==pytest.approx(101.41)
    assert row["exact_bundle_wait_before_delivery_ms"]==6.
    assert row["sensor_callback_entry_unmeasured"] and row["residuals_are_not_separately_instrumented"]


@pytest.mark.parametrize("change",["reverse_clock","bad_wall","bad_proposal","bad_ACK","negative_residual"])
def test_invalid_arithmetic_fails_closed(change):
    command,timing,_,_,_=timing_fixture()
    if change=="reverse_clock":timing["proposal_started_monotonic_ns"]=0
    elif change=="bad_wall":timing["exact_bundle_delivery_to_ACK_seconds"]+=.01
    elif change=="bad_proposal":timing["propose_seconds"]+=.01
    elif change=="bad_ACK":timing["ACK_send_seconds"]+=.01
    else:command["raw_nn_seconds"]=.1
    with pytest.raises(RuntimeError):audit.decompose(timing,command)


def test_fresh_rows_match_all_expected_actual_ACK_native_camera_sources():
    command,timing,receipt,states,cameras=timing_fixture()
    assert len(audit.audit_timing([command],[timing],[receipt],states,cameras,"a"*64))==1


@pytest.mark.parametrize("change",["drop","duplicate","source","frame","timestamp","waiver","sensorclaim"])
def test_no_fresh_subset_source_mutation_or_deadline_waiver(change):
    command,timing,receipt,states,cameras=timing_fixture()
    allrows=[timing]
    if change=="drop":allrows=[]
    elif change=="duplicate":allrows.append(deepcopy(timing))
    elif change=="source":timing["stream_source_sha256"]="b"*64
    elif change=="frame":timing["observed_bundle"]["native_frame"]+=1
    elif change=="timestamp":timing["observed_bundle"]["source_image_timestamp_simulation_ns"]+=1
    elif change=="waiver":timing["policy_deadline_relaxed"]=True
    else:timing["exposure_to_ACK_wall_latency_proven"]=True
    with pytest.raises((RuntimeError,KeyError)):audit.audit_timing([command],allrows,[receipt],states,cameras,"a"*64)


def test_latest_prior_any_source_never_outgoing_or_stale_learned():
    receipts=[dict(sequence=i,before_frame=10+i,status="ACKNOWLEDGED",server_accepted=True) for i in (1,2,3)]
    state=dict(frame=13,control_transport=dict(status="PASS",expected_receipt_sequence=2))
    assert audit.prior_ack(state,receipts)["sequence"]==2
    state["control_transport"]["expected_receipt_sequence"]=1
    with pytest.raises(RuntimeError):audit.prior_ack(state,receipts)


def test_frozen_geometry_area_gate_not_strictlycovered_redefined():
    # HH_261001 - A microscopic noncovered boundary below original area gate must not alter original classifications.
    old=audit.GEOMETRY
    audit.GEOMETRY={"test":("sampled_body","raw_lane",2)}
    try:
        geometry=[dict(frame=i,sampled_body=dict(raw_lane=dict(area_gate_clear=(i==1),strictly_covered=False,
                    outside_m2=1e-7 if i==1 else .2))) for i in (1,2)]
        report=dict(test=dict(assessed_count=2,violation_frames=[2],violation_count=1,status="FAIL",maximum_outside_m2=.2))
        assert audit.audit_body(geometry,report)["test"]["first_violation_frame"]==2
        report["test"]["violation_frames"]=[1,2]
        with pytest.raises(RuntimeError):audit.audit_body(geometry,report)
    finally:audit.GEOMETRY=old


def test_existing_output_never_overwritten(tmp_path,monkeypatch):
    monkeypatch.setattr(audit,"build_report",lambda:pytest.fail("must reject existing output before reading"))
    with pytest.raises(RuntimeError,match="create-only"):audit.publish(tmp_path)


def test_linear_percentile_matches_declared_endpoints():
    assert audit.percentile([1,2,3,4],.95)==pytest.approx(3.85)
    assert audit.stats([1])["p95"]==1

