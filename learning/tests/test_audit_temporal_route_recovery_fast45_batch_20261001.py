"""HH_261001 - Preserve all3 V3 attempts, causal ACKs, denied startup scope, and unchanged physical gates."""

from copy import deepcopy
import json
from pathlib import Path

import pytest

from scripts.e2e import audit_temporal_route_recovery_fast45_batch_20261001 as audit


def motion_fixture(count=30):
    states=[dict(frame=i,capture_phase="driving",control_transport=dict(status="PASS",expected_receipt_sequence=1),
        vx=2.4,vy=0.,world_velocity_carla=[2.4,0.,0.],collision=False,lane_invasion=False) for i in range(count)]
    geometry=[dict(frame=i,independent_actor_arc_m=90.*i/(count-1),independent_actor_route_distance_m=0.) for i in range(count)]
    commands=[dict(ack_receipt_sequence=1,control_source="learned_trajectory_tracker",learned_control_proposed=True,
        held_between_camera_frames=False,control=dict(desired_speed_mps=2.4),application=dict(learned_control_applied=True))]
    return states,geometry,commands


def test_basic_motion_not_full_feature_and_raw_full_interval():
    states,geometry,commands=motion_fixture()
    result=audit.independent_motion(states,geometry,commands)
    assert result["motion_screen_passed"] and result["post_handoff_native_count"]==30
    assert result["first_latched_failure"] is None and result["native_actor_progress_m"]==90.


@pytest.mark.parametrize("change",["last_nonlearned","last_phase","last_cte","p95_cte","speed_base","speed_world","collision","invasion","zero_fresh","zero_held","stop_proposal"])
def test_no_bad_tail_or_zero_target_can_be_removed(change):
    states,geometry,commands=motion_fixture()
    if change=="last_nonlearned": states[-1]["control_transport"]["expected_receipt_sequence"]=2
    elif change=="last_phase": states[-1]["capture_phase"]="braking_tail"
    elif change=="last_cte": geometry[-1]["independent_actor_route_distance_m"]=.750001
    elif change=="p95_cte":
        for row in geometry: row["independent_actor_route_distance_m"]=.350001
    elif change=="speed_base": states[-1]["vx"]=2.500001
    elif change=="speed_world": states[-1]["world_velocity_carla"][0]=2.700001
    elif change=="collision": states[0]["collision"]=True
    elif change=="invasion": states[0]["lane_invasion"]=True
    elif change=="zero_fresh": commands[0]["control"]["desired_speed_mps"]=.05
    elif change=="zero_held":
        held=deepcopy(commands[0]);held.update(held_between_camera_frames=True,ack_receipt_sequence=2)
        held["control"]["desired_speed_mps"]=.05;commands.append(held)
    else:
        rejected=deepcopy(commands[0]);rejected["application"]["learned_control_applied"]=False
        rejected["control"]["desired_speed_mps"]=0.;commands.append(rejected)
    result=audit.independent_motion(states,geometry,commands)
    assert result["motion_screen_passed"] is False and result["post_handoff_native_count"]==30
    if change=="zero_held": assert result["original_zero_target_applied_held"]==1


def test_stationary_dwell_strictly_less_than_one_second():
    states,geometry,commands=motion_fixture()
    for row in states[:19]: row["world_velocity_carla"]=[0.,0.,0.]
    assert audit.independent_motion(states,geometry,commands)["motion_screen_passed"] is True
    states[19]["world_velocity_carla"]=[0.,0.,0.]
    assert audit.independent_motion(states,geometry,commands)["motion_screen_passed"] is False


def test_terminal_emergency_not_invented_as_observed_and_tail_remains():
    states,geometry,commands=motion_fixture()
    commands.append(dict(control_source="emergency_brake",command_frame=29,ack_receipt_sequence=2,
        application=dict(learned_control_applied=False),emergency_reason="original guard"))
    result=audit.independent_motion(states,geometry,commands)
    assert not result["motion_screen_passed"] and result["first_latched_failure"]["terminal_ACK_without_next_observation"]
    assert result["first_latched_failure"]["first_observed_emergency_frame"] is None


def test_emergency_latch_cannot_resume_learned():
    states,geometry,commands=motion_fixture()
    commands.insert(0,dict(control_source="emergency_brake",command_frame=0,ack_receipt_sequence=2))
    with pytest.raises(RuntimeError,match="resumed"): audit.independent_motion(states,geometry,commands)


def test_timing_scope_restores_checkpoint_after_exception_and_rejects_nesting():
    original=audit.frozen.CHECKPOINT_SHA
    with pytest.raises(ValueError):
        with audit.declared_timing_checkpoint("b"*64):
            assert audit.frozen.CHECKPOINT_SHA=="b"*64
            with pytest.raises(RuntimeError):
                with audit.declared_timing_checkpoint("c"*64): pass
            raise ValueError("fixture")
    assert audit.frozen.CHECKPOINT_SHA==original and not audit._timing_active


def metadata_fixture():
    metadata=dict(schema=audit.EXPORT_SCHEMA,model_id=audit.MODEL_ID,optimizer_updates=3000,known_total_optimizer_updates=6200,
        training_episode_count=18,recovery_training_episode_count=9,replay_training_episode_count=9,holdout_episode_count=2,
        backbone_frozen=True,model_promoted=False,vehicle_control_approved=False,inference_loader_authorized=False,stop_intent_trained=False,
        recovery_data_revision="prospective_actual_signed_recovery_moving_v4",new_signed_recovery_cohort_moving_only=True,
        startup_recovery_fitted=False,raw_startup_retained_outside_new_signed_recovery_training=True,
        fixed_recovery_native_frame_offsets=[80,682],fixed_recovery_driving_relative_window_seconds=[4.,34.1],
        state_tensor_shapes={str(i):[1] for i in range(101)},state_tensor_dtypes={str(i):"float32" for i in range(101)},
        trainable_state_names=["a","b","c","d"],checkpoint_sha256="a"*64,
        source_sha256={str(i):"b"*64 for i in range(23)})
    metadata["source_sha256"]["tools/e2e/train_temporal_route_recovery_moving_v4_20261001.py"]="c"*64
    return metadata,dict(model_architecture_id=audit.MODEL_ID,source_checkpoint_sha256="a"*64,source_sha256=dict(metadata["source_sha256"]))


def test_new24th_source_and_new_training_schema_are_compatible_with_export():
    metadata,plan=metadata_fixture()
    audit.verify_export(metadata,plan)


@pytest.mark.parametrize("change",["source_missing","source_mutation","startup_claim","old_grid","step_short","different_model","old_export","fewer_tensors","promoted"])
def test_numeric_lineage_cannot_misrepresent_old_data_or_short_fit(change):
    metadata,plan=metadata_fixture()
    if change=="source_missing": plan["source_sha256"].pop("tools/e2e/train_temporal_route_recovery_moving_v4_20261001.py")
    elif change=="source_mutation": plan["source_sha256"]["0"]="e"*64
    elif change=="startup_claim": metadata["startup_recovery_fitted"]=True
    elif change=="old_grid": metadata["fixed_recovery_native_frame_offsets"]=[18,620]
    elif change=="step_short": metadata["optimizer_updates"]=2500
    elif change=="different_model": metadata["model_id"]="different"
    elif change=="old_export": metadata["schema"]="old"
    elif change=="fewer_tensors": metadata["state_tensor_shapes"].pop("0")
    else: metadata["model_promoted"]=True
    with pytest.raises(RuntimeError): audit.verify_export(metadata,plan)


def test_source_closure_deterministic_includes_independent_helpers_not_unrelated_files():
    seeds=(Path(audit.__file__),audit.ROOT/"tools/workspace_python.py",audit.ROOT/"tools/canonical_workspace.py")
    first=audit.source_closure(*seeds)
    assert first==audit.source_closure(*reversed(seeds))
    assert audit.ROOT/"tools/e2e/audit_carla_acknowledged_control.py" in first
    assert audit.ROOT/"tools/e2e/audit_carla_comfortable_v3_trial.py" in first
    assert audit.ROOT/"tools/e2e/research_temporal_route_recovery_fast45_town07_20261001.py" not in first


def test_existing_output_never_overwritten(tmp_path,monkeypatch):
    monkeypatch.setattr(audit,"build_report",lambda *a:pytest.fail("must reject output first"))
    with pytest.raises(RuntimeError,match="create-only"): audit.publish(tmp_path,"unused","a"*64,"unused","b"*64)


def test_conflicting_original_pin_rejected(tmp_path):
    source=tmp_path/"actual.json";source.write_text("original")
    pins={};audit.add_pin(pins,source)
    source.write_text("changed")
    with pytest.raises(RuntimeError): audit.add_pin(pins,source)


def test_no_accepted_first_inference_is_unproven_not_vacuous_success():
    command=dict(learned_control_proposed=False,held_between_camera_frames=False,total_seconds=.11,
        control_source="emergency_brake",command_frame=10,ack_receipt_sequence=1,application=dict(learned_control_applied=False))
    assert audit.timing_audit([command],[],[],[],[],"a"*64,"b"*64)==[]
    states,geometry,_=motion_fixture(count=900)
    for state in states:state["control_transport"]["expected_receipt_sequence"]=1
    result=audit.independent_motion(states,geometry,[command])
    assert result["motion_screen_passed"] is False and result["post_handoff_native_count"]==0
    assert result["native_actor_progress_m"] is None
    report=dict(driving_pass_count=0,body_pass_count=3,observed_delivery_to_ACK100ms_trial_pass_count=0,
        joint_motion_body_policy_wall_trial_pass_count=0,trials=[dict(result,trial_index=i,delivery_to_ACK_ms=dict(count=0),
            original_driving_screen_passed=False,fresh_observed_delivery_to_ACK100ms_passed=False) for i in range(3)])
    assert "미측정" in audit.markdown(report) and "미증명/FAIL" in audit.markdown(report)


def test_no_proposal_cannot_have_hidden_fresh_timing():
    with pytest.raises(RuntimeError):
        audit.timing_audit([], [dict(learned_control_proposed=True,held_between_camera_frames=False)],[],[],[],"a"*64,"b"*64)


def test_actual_files_all450_post_ACK_and2700_byte_pins_final_alias(tmp_path):
    # HH_261001 - This fixture uses tiny explicitly synthetic bytes, not inferred or claimed actual camera pixels.
    folder=tmp_path/"trial_00";episode=folder/"episode";episode.mkdir(parents=True)
    for name in audit.frozen.CAMERAS:(episode/"images"/name).mkdir(parents=True)
    cameras,recording,timing=[],[],[]
    for i in range(450):
        frame=1000+2*i
        images={name:"images/%s/%08d.jpg"%(name,frame) for name in audit.frozen.CAMERAS}
        for relative in images.values():(episode/relative).write_bytes(b"synthetic test bytes - not JPEG evidence")
        cameras.append(dict(frame=frame,images=images))
        recording.append(dict(frame=frame,enqueued_camera_count=6,written_camera_count=6,all_queued_images_written=True,
            failure=None,acknowledged_before_recording=True,ACK_receipt_sequence=i+1,
            recording_started_monotonic_ns=20,recording_finished_monotonic_ns=30,
            written_files=[str(folder/"episode.partial"/images[name]) for name in audit.frozen.CAMERAS]))
        timing.append(dict(ACK_sequence=i+1,ACK_server_accepted=True,command_frame=frame,ACK_finished_monotonic_ns=10))
    path=episode/"recording_after_ack.jsonl"
    path.write_text("".join(json.dumps(r)+"\n" for r in recording))
    pins={};report=audit.audit_post_ACK_recording(folder,episode,cameras,timing,pins)
    assert report["image_count"]==2700 and len(pins)==2701 and report["JPEG_pixels_decoded"] is False
    changed=episode/cameras[-1]["images"][audit.frozen.CAMERAS[-1]]
    changed.write_bytes(b"changed")
    with pytest.raises(RuntimeError):audit.frozen.assert_pins(pins)


def contract_fixture():
    return dict(schema=audit.OWNER_SCHEMA,trial_index=0,total_capture_seconds=45,candidate_count=1,
        model_architecture_id=audit.MODEL_ID,controller_target_cap_mps=2.4,model_speed_cap_mps=2.5,
        native_actor_planar_speed_maximum_mps=2.7,policy_deadline_seconds=.1,policy_deadline_relaxed=False,
        runtime_route_as_output_replacement=False,oracle_future_inputs=False,no_teacher_fallback_after_handoff=True,
        sensor_and_ACK_screen=dict(native_state_count=900,camera_bundle_count=450),
        body_clearance_screen=dict(all_native_boxes=900,all_snapshot_linear_sweeps=899,lane_inset_m=.1),
        precursor_motion_screen=dict(net_learned_progress_minimum_m=90.,unintended_stationary_dwell_strictly_below_s=1.,
            stationary_threshold_mps=.05,cte_p95_maximum_m=.35,cte_absolute_maximum_m=.75,
            collisions_maximum=0,lane_invasions_maximum=0,rejected_learned_proposals_maximum=0))


def test_whole45s_declaration_rejects_body_cadence_or_motion_gate_relaxation():
    plan=contract_fixture();audit.verify_contract(plan,0)
    plan["body_clearance_screen"]["lane_inset_m"]=0.
    with pytest.raises(RuntimeError):audit.verify_contract(plan,0)
    plan=contract_fixture();plan["precursor_motion_screen"]["cte_p95_maximum_m"]=.75
    with pytest.raises(RuntimeError):audit.verify_contract(plan,0)
