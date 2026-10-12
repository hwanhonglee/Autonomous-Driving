"""HH_261002 - Synthetic true64future timing, undefined heading and immutable diagnostic tests."""

import copy
import hashlib
import json

import pytest

from scripts.e2e import audit_town07_natural_goal_future64_20261002 as audit


def state(index, *, speed=1., x=None, phase="driving"):
    return dict(frame=1000+index,timestamp=index*.05,x=index*.05 if x is None else x,y=0.,z=0.,yaw=0.,
        vx=speed,vy=0.,ax=0.,ay=0.,yaw_rate=0.,collision=[],lane_invasion=[],capture_phase=phase,
        goal_stop=dict(profile_id=audit.PROFILE,phase="cruise",remaining_route_arc_m=100.,
            goal_window=False,upstream_of_endpoint=True,terminal_overshoot_m=-100.,hold_duration_sec=0.,complete=False))


def fixture():
    plan = dict(schema=audit.TASK_SCHEMA,case_id="pilot_00",seed=2026102200,
        selected_first_driving_relative_native_frame=80,stationary_tail_native_ticks=130,
        stationary_tail_can_be_true_future_context=True,stationary_tail_is_training_anchor=False)
    states, cameras, x = [], [], 0.
    for index in range(954):
        phase = "stationary_warmup" if index < 24 else "driving" if index < 824 else "stationary_tail"
        speed = 0. if index < 24 else min(1.,max(0.,(781-index)*.025))
        x += speed*.05
        row = state(index,speed=speed,x=x,phase=phase)
        if index >= 741:
            row["goal_stop"].update(phase="approach" if index < 781 else "holding" if index < 821 else "complete",
                goal_window=index >= 781,remaining_route_arc_m=max(.75,(781-index)*.04+.75),
                terminal_overshoot_m=-.75,hold_duration_sec=max(0.,(index-781)*.05),complete=index >= 821)
        states.append(row)
        if index%2 == 0:
            cameras.append(dict(frame=row["frame"],timestamp=row["timestamp"],capture_phase=phase,
                camera_order=list(audit.contract.CAMERA_ORDER),images={k:k+".jpg" for k in audit.contract.CAMERA_ORDER},
                source_timestamps={k:row["timestamp"] for k in audit.contract.CAMERA_ORDER},
                moving_task_selected=104 <= index <= 823))
    chosen = [r for r in cameras if r["moving_task_selected"]]
    manifest = dict(status="complete",capture_contract=dict(natural_goal_stop_teacher=plan),
        result=dict(goal_stop_quality=dict(status="PASS"),moving_task=dict(task_complete=True,
            actual_goal_completion=dict(frame=states[823]["frame"],timestamp_ns=audit.stamp(states[823])),
            selected_camera_anchor_count=len(chosen),first_selected_camera=dict(frame=chosen[0]["frame"]),
            last_selected_camera=dict(frame=chosen[-1]["frame"]))))
    return plan,manifest,states,cameras


def test_all_prospective_windows_and_true_tail_labels_retained():
    plan,manifest,states,cameras = fixture()
    summary,windows = audit.audit_core(plan,manifest,states,cameras)
    assert summary["selected_anchor_count"] == 360
    assert summary["true_future_point_count"] == 360*64
    assert summary["failed_fixed_window_count"] == 0
    assert summary["research_training_authorized"] is False
    assert summary["training_data_approved"] is False
    assert summary["vehicle_control_approved"] is False
    assert windows[0]["actual_ten_ego_history_native_frames"] == [1086+2*i for i in range(10)]
    assert windows[0]["actual_model_five_camera_history_frames"] == [1088+4*i for i in range(5)]
    assert windows[0]["actual_model_five_camera_history_mask"] == [True]*5
    last = windows[-1]["true_future_rows"]
    assert len(last) == 64 and all(row["valid"] for row in last)
    assert all(row["speed_mps"] == 0. for row in last)
    assert all(row["xy"] == [0.,0.] for row in last)
    assert all(row["stationary_domain_metrics"]["xy_curvature"] == [None,False] for row in last)
    assert any("stationary_tail" in row["future_capture_phases"] for row in last)


def test_initial_acceleration_uses_actual_planar_v0_not_API_acceleration():
    states = [state(i,speed=1.+.05*i,x=(i+.05*i*i)*.05) for i in range(140)]
    states[0]["ax"] = 123.
    rows = audit.future_window([(audit.stamp(r),r) for r in states],states[0])
    assert rows[0]["initial_measured_planar_acceleration_mps2"] == pytest.approx(1.)
    assert rows[0]["speed_mps"] == pytest.approx(1.1)
    assert rows[0]["source_native_bracket_frames"] == [1002,1002]


def test_legacy_vx_initialization_is_retained_separate_from_measured_v0():
    states = [state(i,speed=0.,x=0.) for i in range(140)]
    for row in states:row["vy"] = .2
    rows = audit.future_window([(audit.stamp(r),r) for r in states],states[0])
    assert rows[0]["legacy_formula_metrics"]["speed_acceleration"][0] == pytest.approx(2.)
    assert rows[0]["stationary_domain_metrics"]["speed_acceleration"][0] == 0.


def test_actual_float_clock_is_interpolated_not_snapped():
    states = [state(i,x=i*.05) for i in range(150)]
    for i,row in enumerate(states):row["timestamp"] = i*.05000000074505806
    timeline = [(audit.stamp(r),r) for r in states]
    rows = audit.future_window(timeline,states[10])
    assert rows[0]["target_timestamp_ns"] == audit.stamp(states[10])+100_000_000
    assert rows[0]["source_native_bracket_timestamps_ns"] != [rows[0]["target_timestamp_ns"]]*2
    assert rows[0]["xy"][0] == pytest.approx(.1,abs=1e-8)


def test_stationary_noise_mask_never_zeros_or_changes_raw_future():
    states = [state(i,speed=.001,x=0.,phase="stationary_tail") for i in range(140)]
    for i,row in enumerate(states):
        row["y"] = (.0002 if i%4 == 2 else -.0002 if i%4 == 0 else 0.)
        row["goal_stop"].update(phase="complete",goal_window=True,remaining_route_arc_m=.75,
            terminal_overshoot_m=-.75,complete=True,hold_duration_sec=2.)
    original = copy.deepcopy(states)
    rows = audit.future_window([(audit.stamp(r),r) for r in states],states[0])
    assert states == original
    assert any(row["legacy_formula_metrics"]["xy_curvature"][1] for row in rows)
    assert all(row["stationary_domain_metrics"]["xy_curvature"] == [None,False] for row in rows)
    assert all(row["curvature_label_valid"] is False for row in rows)
    assert rows[0]["xy"] != [0.,0.]
    assert all(row["speed_mps"] == .001 for row in rows)


def test_true_hard_deceleration_not_hidden_by_stationary_mask():
    plan,manifest,states,cameras = fixture()
    states[767]["vx"],states[768]["vx"],states[769]["vx"] = 1.5371,.721294,.000851
    manifest["status"] = "failed";manifest["result"]["goal_stop_quality"]["status"] = "FAIL"
    manifest["result"]["moving_task"]["task_complete"] = False
    summary,windows = audit.audit_core(plan,manifest,states,cameras)
    assert summary["source_capture_failed"] is True
    assert summary["protected_native_failure_anchor_count"] > 0
    bad = summary["all_raw_native_interval_failures"]
    assert any(row["actual_scalar_dvdt_mps2"] < -16. for row in bad)
    assert any("protected_native20Hz_dynamics_or_events" in row["failure_reasons"] for row in windows)
    assert any(row["stationary_domain_metrics"]["speed_deceleration"][1]
        for window in windows for row in window["true_future_rows"] if row["valid"])
    assert summary["selected_anchor_count"] == 360


def test_physical2p95_native_spike_not_hidden_by100ms_average_or_native3_bound():
    states = [state(i,speed=1. if i == 0 else 1.1475) for i in range(140)]
    timeline = [(audit.stamp(row),row) for row in states]
    native = audit.native_intervals(timeline,0)
    assert native[0]["actual_scalar_dvdt_mps2"] == pytest.approx(2.95)
    assert native[0]["native_scalar_dvdt_exceeds3"] is False
    assert native[0]["native_scalar_dvdt_exceeds_original_physical2p9"] is True
    assert native[0]["failure_reasons"] == ["actual_native_scalar_dvdt_exceeds_physical2p9"]
    future = audit.future_window(timeline,states[0])
    assert future[0]["initial_measured_planar_acceleration_mps2"] == pytest.approx(1.475)
    assert future[0]["stationary_domain_metrics"]["speed_acceleration"][1] is False


@pytest.mark.parametrize("marker",("training_data_approved","development_only"))
def test_explicit_native_denial_preserved_not_lifted(marker):
    plan,manifest,states,cameras = fixture()
    manifest["result"][marker] = marker == "development_only"
    before = copy.deepcopy(manifest)
    summary,_ = audit.audit_core(plan,manifest,states,cameras)
    assert summary["native_explicit_denial_present"] is True
    assert summary["raw_native_denials_overridden"] is False
    assert summary["training_data_approved"] is False
    assert manifest == before


@pytest.mark.parametrize("kind",("native_gap","camera_gap","timestamp","order","membership","tail_anchor","hold"))
def test_missing_or_relabelled_evidence_rejected(kind):
    plan,manifest,states,cameras = fixture()
    if kind == "native_gap":states.pop(240)
    elif kind == "camera_gap":cameras.pop(240)
    elif kind == "timestamp":cameras[120]["source_timestamps"][audit.contract.CAMERA_ORDER[0]] += .001
    elif kind == "order":cameras[120]["camera_order"].reverse()
    elif kind == "membership":cameras[120]["moving_task_selected"] = False
    elif kind == "tail_anchor":cameras[-1]["moving_task_selected"] = True
    else:states[823]["goal_stop"]["hold_duration_sec"] = 1.95
    with pytest.raises(ValueError):audit.audit_core(plan,manifest,states,cameras)


def test_HOLD_cannot_replace_moving_or_outside_goal_actual_state():
    row = state(0,speed=.5)
    row["goal_stop"].update(phase="holding",goal_window=True,terminal_overshoot_m=-.75)
    assert audit.intent_witness(row)["valid"] is False
    row["vx"] = 0.;row["goal_stop"]["goal_window"] = False
    assert audit.intent_witness(row)["valid"] is False


def test_short_GO_support_reported_not_relabelled_BRAKE():
    plan,manifest,states,cameras = fixture()
    states[400]["goal_stop"]["remaining_route_arc_m"] = 10.
    summary,windows = audit.audit_core(plan,manifest,states,cameras)
    assert summary["GO_anchors_incompatible_with_existing_18m_support"] == 1
    window = next(w for w in windows if w["source_frame"] == 1400)
    assert window["current_semantic_witness"]["intent"] == "GO"


def test_missing_future_points_stay_in_fixed_denominator():
    states = [state(i) for i in range(10)]
    rows = audit.future_window([(audit.stamp(r),r) for r in states],states[0])
    assert len(rows) == 64 and sum(row["valid"] for row in rows) == 4
    assert rows[-1]["xy"] is None and rows[-1]["invalid_reason"] == "episode_end"


def test_payload_pins_bind_parsed_bytes_and_detect_mutation(tmp_path):
    path = tmp_path/"metadata.json";path.write_text('{"value":1}')
    pins = {};expected = hashlib.sha256(path.read_bytes()).hexdigest()
    assert audit.document(path,pins,expected) == {"value":1}
    path.write_text('{"value":2}')
    with pytest.raises(ValueError,match="changed"):audit.recheck(pins)


def test_duplicate_and_nonfinite_JSON_rejected():
    for payload in (b'{"a":1,"a":2}',b'{"a":NaN}'):
        with pytest.raises(ValueError):audit.parse_json(payload)


def test_symlink_ancestors_rejected(tmp_path):
    (tmp_path/"real").mkdir();(tmp_path/"link").symlink_to(tmp_path/"real",target_is_directory=True)
    with pytest.raises(ValueError):audit.canonical(tmp_path/"link"/"new.json")


def governor_pair():
    row = state(1)
    row["goal_stop"].update(planar_goal_distance_m=100.,measured_speed_mps=1.,distance_speed_envelope_mps=2.4,
        target_speed_mps=2.4,hold_ticks=0,required_hold_ticks=40)
    update = dict(schema=audit.TASK_SCHEMA+".actual_governor_update",measured_state_frame=row["frame"],
        governor_math_changed=False,measured_speed_clipped=False,original_governor_update=copy.deepcopy(row["goal_stop"]))
    return row,update


def test_recorded_governor_same_frame_semantic_binding():
    row,update = governor_pair()
    result = audit.governor_receipt_binding([row],[update])
    assert result["all_native_phase_labels_bind_actual_same_frame_governor_receipts"] is True
    update["original_governor_update"]["phase"] = "holding"
    with pytest.raises(ValueError):audit.governor_receipt_binding([row],[update])


def test_governor_label_frame_and_measured_v0_cannot_be_rewritten():
    row,update = governor_pair()
    update["measured_state_frame"] += 1
    with pytest.raises(ValueError):audit.governor_receipt_binding([row],[update])
    row,update = governor_pair();row["goal_stop"]["measured_speed_mps"] = 0.
    update["original_governor_update"]["measured_speed_mps"] = 0.
    with pytest.raises(ValueError):audit.governor_receipt_binding([row],[update])


def test_nondriving_target_zero_write_does_not_fabricate_HOLD_label():
    row,update = governor_pair();row["capture_phase"] = "stationary_warmup"
    row["goal_stop"]["target_speed_mps"] = 0.
    assert audit.governor_receipt_binding([row],[update])["native_frame_count"] == 1
    assert audit.intent_witness(row)["intent"] == "GO"
