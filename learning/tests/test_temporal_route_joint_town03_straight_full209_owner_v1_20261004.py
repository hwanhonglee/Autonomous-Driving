"""HH_261004 - Exercise fresh full209 owner declarations and false-goal rejection on CPU only, not driving performance."""

import copy
from contextlib import contextmanager
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest

from scripts.e2e import research_temporal_route_joint_town03_straight_full209_v1_20261004 as m
from test_temporal_route_joint_town03_straight_v3_20261004 import pins, good6000_pins


@contextmanager
def scope(values=None):
    threads = m.straight.runtime_owner.previous.previous.threads
    with mock.patch.object(m.numeric,"verify_exports"), mock.patch.object(threads,"evidence_assets",return_value={}):
        with m.activated(pins() if values is None else values):
            yield


def rows(n=35):
    # HH_261004 - These scalar fixtures only test denominator/authentication code; they are never measured drives or TRAIN inputs.
    route = dict(town="Town03",scenario="straight",route=[dict(x=0.,y=0.,distance_m=0.,vad_command=0),
        dict(x=209.,y=0.,distance_m=209.,vad_command=0)])
    blank = dict(throttle=0.,steer=0.,brake=1.,hand_brake=False,reverse=False,gear=0,manual_gear_shift=False)
    states = []
    for index in range(n):
        progress = 0. if index < 24 else 207. if index == n-1 else 206. * (index-24)/(n-26)
        sequence = index+3
        frame = 100+index
        states.append(dict(frame=frame,timestamp=frame*.05,capture_phase="stationary_warmup" if index<24 else "driving",
            x=progress,y=0.,z=0.,yaw=0.,route_progress_m=progress,world_velocity_carla=[2.,0.,0.],
            current_control=copy.deepcopy(blank),collision=[],lane_invasion=[],
            control_transport=dict(status="PASS",expected_receipt_sequence=sequence,
                observed_frame=frame,frame_binding_pass=True,control_read_frame_before=frame,
                control_read_frame_after=frame,mismatched_fields=[])))
    frame, sequence = states[-1]["frame"], states[-1]["control_transport"]["expected_receipt_sequence"]
    guard = dict(frame=frame,measured_actor_planar_speed_mps=2.,maximum_actor_planar_speed_mps=2.7,
        free_disk_bytes=6*1024**3,required_free_bytes=5*1024**3+16*1024**2)
    receipt = dict(schema=m.goal_guard.SCHEMA,status="ACKNOWLEDGED",command_reason=m.goal_guard.GOAL_REASON,
        command_frame=frame,source_timestamp_simulation_seconds=states[-1]["timestamp"],actor_id=17,
        prior_learned_ACK_sequence=sequence,prior_learned_ACK_after_frame=frame-1,cleanup_ACK_sequence=sequence+1,
        original_goal_tolerance_m=2.5,original_catalog_length_m=209.,remaining_catalog_route_m=2.,
        actual_planar_catalog_goal_distance_m=2.,requested_original_cleanup_control=blank,
        original_catalog_two_predicates_passed=True,actual_prior_learned_ACK_effect_proven=True,
        owner_source_PID_check_passed=True,observed_event_snapshot_empty=True,existing_emergency_latch_clear=True,
        native_guard_attempted=True,native_guard_passed=True,cleanup_ACK_received=True,native_guard=guard,
        learned_control_proposed=False,learned_STOP_proven=False,physical_cleanup_actuation_proven=False,
        vehicle_control_approved=False,current_tick_hazard_detector_clear_proven=False)
    receipt["actual_state_binding_sha256"] = m.goal_guard._state_binding(states[-1])
    states[-1]["catalog_goal_cleanup_receipt"] = receipt
    cameras = [dict(frame=row["frame"]) for row in states[::2]]
    manifest = dict(status="complete",capture_contract=dict(learned_research=dict(checkpoint_sha256="a"*64,
        stream_identity=dict(source_sha256="b"*64)),capture_phase_schedule=dict(
        maximum_total_duration_sec=110.,maximum_total_ticks=2200,stationary_warmup=dict(scheduled_ticks=24),
        driving=dict(maximum_ticks=2176),stationary_tail=dict(scheduled_ticks=0))),
        result=dict(goal_reached=True,termination_reason="catalog_goal_tolerance",state_count=n,
            camera_anchor_count=len(cameras),goal_tolerance_m=2.5,final_remaining_route_m=2.,final_catalog_goal_distance_m=2.))
    commands = [dict(command_frame=row["frame"],ack_receipt_sequence=row["control_transport"]["expected_receipt_sequence"]+1,
        source_checkpoint_sha256="a"*64,application=dict(learned_control_applied=True),control_source="learned_trajectory_tracker") for row in states[24:-1]]
    receipts = [dict(sequence=index+1,reason="next_driving_control",status="ACKNOWLEDGED",server_accepted=True,
        mode="acknowledged_batch",do_tick=False,actor_id=17,response_actor_id=17,
        expected_before_frame=100+index-3,before_frame=100+index-3,after_ack_frame=100+index-3,
        requested_control=copy.deepcopy(blank)) for index in range(n+4)]
    receipts[sequence]["reason"] = m.goal_guard.GOAL_REASON
    timings = [dict(ACK_sequence=row["sequence"],ACK_server_accepted=True,learned_control_proposed=True,
        source_checkpoint_sha256="a"*64,stream_source_sha256="b"*64,
        control_source="learned_trajectory_tracker",command_frame=row["after_ack_frame"]) for row in receipts]
    wrapper = dict(schema=m.live.SCHEMA+".catalog_goal_cleanup",receipt=copy.deepcopy(receipt),
        failed_or_absent_receipt_retained=True,learned_STOP_claim=False)
    return route, states, cameras, manifest, commands, receipts, timings, [guard], wrapper


def interval(values,monkeypatch,event=None):
    route,states,cameras,manifest,*_ = values
    monkeypatch.setattr(m,"read_json",lambda _:route)
    monkeypatch.setattr(m,"digest",lambda _:m.live.ROUTE_SHA)
    return m.complete_interval(manifest,states,cameras,{"completed":True} if event is None else event)


def test_frozen_parent_and_callback_reader_sources():
    assert m.digest(Path(m.previous.__file__)) == m.PARENT_OWNER_SHA
    assert m.digest(Path(m.callback_owner.__file__)) == m.CALLBACK_READER_OWNER_SHA
    m.live.verify_frozen()


def test_fresh_contract_110_dynamic_gates_and_honest_intervention():
    with scope():
        plan = m.fixed_contract("v1",True)
        argv = m.capture_argv("v1",True,"a"*64)
        assert m.base.live is m.live
        assert m.right.independent_event is m.recomputed_straight_interval
        assert m.right.complete_interval is m.complete_interval
        assert m.right.geometry is m.previous.previous.start_geometry
        assert m.right.body_report is m.previous.previous.body_report
    assert plan["schema"] == m.live.OWNER_SCHEMA and plan["run_id"] == "v1"
    assert plan["total_capture_seconds"] == 110 and plan["maximum_native_state_count"] == 2200
    assert plan["maximum_camera_bundle_count"] == 1100 and plan["maximum_driving_native_ticks"] == 2176
    assert argv[argv.index("--max-duration-sec")+1] == "110"
    assert argv[argv.index("--port")+1] == "2440" and argv[argv.index("--stationary-tail-sec")+1] == "0"
    assert plan["source_manifest_sealed_before_declaration"] is True
    assert plan["original_NN_decoder_selected_trajectory_math_unchanged"] is False
    assert plan["original_NN_weights_unchanged"] is plan["original_physical_gate_math_unchanged"] is True
    assert plan["original_tracker_PI_math_unchanged"] is True
    assert plan["actual_callback_timing"]["maximum_callbacks"] == 24000
    assert plan["actual_callback_timing"]["maximum_callback_JSON_bytes"] == 48*1024**2
    assert plan["learned_STOP_claim"] is plan["full_RTE01_approved"] is False
    assert plan["official_completed_main_domains"] == plan["official_completed_subfunctions"] == 0
    media = plan["prospective_visual_publication"]
    assert media["complete_actual_interval_including_failures_required"] is True
    assert media["outcome_or_failure_dependent_frame_selection"] is False
    assert media["actual_Autoware_or_RViz_screenshot_claim"] is False
    assert "every5th" in media["GIF_camera_indices"] and "actual_camera_count-1" in media["PNG_camera_indices"]
    quota = plan["complete_tree_quota_counter_intervention"]
    assert quota["original_quota_and_free_disk_floor_and_reserves_unchanged"] is True
    assert quota["maximum_walk_count"] == 1100
    assert quota["sensor_latency_gain_claimed"] is False
    assert plan["precursor_motion_screen"]["cte_p95_maximum_m"] == .35
    assert plan["native_actor_planar_speed_maximum_mps"] == 2.7
    assert plan["model_speed_cap_mps"] == 2.5 and plan["controller_target_cap_mps"] == 2.4


@pytest.mark.parametrize("failure",[False,True])
def test_owner_restores_every_alias_even_exception(failure):
    before = (m.previous.live,m.previous.paths,m.right.independent_event,m.right.complete_interval,
        m.right.geometry,m.right.body_report,m.runtime_owner.complete_interval,m.base.live)
    try:
        with scope():
            with pytest.raises(RuntimeError):
                with m.activated(pins()):
                    pass
            if failure:
                raise ValueError("CPU fixture exit")
    except ValueError:
        assert failure
    assert (m.previous.live,m.previous.paths,m.right.independent_event,m.right.complete_interval,
        m.right.geometry,m.right.body_report,m.runtime_owner.complete_interval,m.base.live) == before
    assert m._active is False
    with pytest.raises(RuntimeError):
        m.capture_argv("v1",True)
    with pytest.raises(RuntimeError):
        m.paths("v2")


@pytest.mark.parametrize("n",[35,36,1755,2199,2200])
def test_actual_denominator_dynamic_without_stop_dwell_requirement(n,monkeypatch):
    assert interval(rows(n),monkeypatch)


@pytest.mark.parametrize("change",["partial","failure","tail","warmup","native_gap","camera_gap","time_gap",
    "goal_false","tolerance","remaining","planar","earlier_goal","receipt_missing","priorNN_missing",
    "cleanup_seq","latch","events_fact","stop_claim","guard","maximum","budget"])
def test_false_or_incomplete_goal_never_passes(change,monkeypatch):
    values = rows()
    _,states,cameras,manifest,*_ = values
    receipt = states[-1]["catalog_goal_cleanup_receipt"]
    if change == "partial": manifest["status"] = "failed"
    elif change == "failure": manifest["error"] = "CollectionError: safety abort"; manifest["status"] = "failed"
    elif change == "tail": states[-1]["capture_phase"] = "stationary_tail"
    elif change == "warmup": states[0]["capture_phase"] = "driving"
    elif change == "native_gap": states[-2]["frame"] += 1
    elif change == "camera_gap": cameras.pop()
    elif change == "time_gap": states[-1]["timestamp"] += .1
    elif change == "goal_false": manifest["result"]["goal_reached"] = False
    elif change == "tolerance": manifest["result"]["goal_tolerance_m"] = 3.
    elif change == "remaining": states[-1]["route_progress_m"] = 206.
    elif change == "planar": states[-1]["y"] = 3.
    elif change == "earlier_goal": states[-2].update(x=207.,route_progress_m=207.)
    elif change == "receipt_missing": states[-1].pop("catalog_goal_cleanup_receipt")
    elif change == "priorNN_missing": receipt["actual_prior_learned_ACK_effect_proven"] = False
    elif change == "cleanup_seq": receipt["cleanup_ACK_sequence"] += 1
    elif change == "latch": receipt["existing_emergency_latch_clear"] = False
    elif change == "events_fact": receipt["observed_event_snapshot_empty"] = False
    elif change == "stop_claim": receipt["learned_STOP_proven"] = True
    elif change == "guard": receipt["native_guard_passed"] = False
    elif change == "maximum": manifest["capture_contract"]["capture_phase_schedule"]["maximum_total_ticks"] = 2300
    elif change == "budget": manifest["result"]["termination_reason"] = "driving_budget"
    assert not interval(values,monkeypatch)


def test_recomputed_interval_keeps_final_priorNN_effect_and_no_new_learned_command():
    route,states,_,_,commands,*_ = rows()
    summary,observed = m.recomputed_straight_interval(route,states,commands)
    assert summary["completed"] is True
    assert len(observed) == len(states)-24
    assert observed[-1]["current_prior_ACK_is_learned_effect"] is True
    assert observed[-1]["outgoing_original_safety_branch_clear"] is True
    assert states[-1]["frame"] not in [row["command_frame"] for row in commands]
    commands[-1]["application"]["learned_control_applied"] = False
    assert m.recomputed_straight_interval(route,states,commands)[0]["completed"] is False


def test_independent_goal_receipt_actual_ACK_and_native_guard():
    _,states,_,manifest,commands,receipts,timings,guards,wrapper = rows()
    result = m.audit_goal_receipt(states,commands,receipts,timings,guards,wrapper,manifest)
    assert result["passed"] is True and result["learned_STOP_proven"] is False
    assert result["physical_cleanup_actuation_proven"] is False


@pytest.mark.parametrize("change",["actor","ACK_reject","cleanup_reason","prior_frame","duplicate","missing",
    "copied_wrapper","native_guard","disk","speed","state_binding","event","prior_model","prior_wall"])
def test_independent_goal_receipt_rejects_tamper(change):
    _,states,_,manifest,commands,receipts,timings,guards,wrapper = rows()
    seq = states[-1]["catalog_goal_cleanup_receipt"]["prior_learned_ACK_sequence"]
    if change == "actor": receipts[seq]["actor_id"] = 18
    elif change == "ACK_reject": receipts[seq]["server_accepted"] = False
    elif change == "cleanup_reason": receipts[seq]["reason"] = "next_driving_control"
    elif change == "prior_frame": receipts[seq-1]["after_ack_frame"] -= 1
    elif change == "duplicate": receipts[0]["sequence"] = 2
    elif change == "missing": receipts.pop()
    elif change == "copied_wrapper": wrapper["receipt"]["actor_id"] = 18
    elif change == "native_guard": guards[:] = []
    elif change == "disk": guards[0]["free_disk_bytes"] = 0
    elif change == "speed": states[-1]["world_velocity_carla"][0] = 2.8
    elif change == "state_binding": states[-1]["x"] += .01
    elif change == "event": states[-1]["lane_invasion"] = ["recorded_event"]
    elif change == "prior_model": commands[-1]["application"]["learned_control_applied"] = False
    elif change == "prior_wall": timings[seq-1]["ACK_server_accepted"] = False
    with pytest.raises((RuntimeError,KeyError)):
        m.audit_goal_receipt(states,commands,receipts,timings,guards,wrapper,manifest)


def callback_document():
    return dict(schema=m.live.SCHEMA+".callback_receipts",clock="time.perf_counter_ns_monotonic",
        callback_count=0,callbacks=[],camera_bindings=[{} for _ in range(6)],bundles=[],policy_stage_marks=[],errors=[],native_stage_marks=[])


def read_callback(tmp_path,value):
    path = tmp_path/"callback_timing_receipts.json"
    payload = json.dumps(value).encode()
    path.write_bytes(payload)
    return m.read_callback_receipts(path,hashlib.sha256(payload).hexdigest())


def test_callback_reader_new_bounds_and_no_generic_limit_change(tmp_path):
    generic_reader = m.read_json
    value = callback_document()
    value["native_stage_marks"] = [{}]*17600
    value["bundles"] = [dict(cameras=[{}]*6)]*1100
    value["callbacks"] = [{}]*24000
    value["callback_count"] = 24000
    value["policy_stage_marks"] = [{}]*3300
    assert len(read_callback(tmp_path,value)["callbacks"]) == 24000
    assert m.read_json is generic_reader


@pytest.mark.parametrize("change",["count","native","bundle","policy","schema","clock","duplicate","SHA","symlink"])
def test_callback_reader_fails_closed(tmp_path,change):
    value = callback_document()
    if change == "count": value["callback_count"] = 24001
    elif change == "native": value["native_stage_marks"] = [{}]*17601
    elif change == "bundle": value["bundles"] = [dict(cameras=[{}]*6)]*1101
    elif change == "policy": value["policy_stage_marks"] = [{}]*3303
    elif change == "schema": value["schema"] = m.callback_owner.live.SCHEMA+".callback_receipts"
    elif change == "clock": value["clock"] = "simulation"
    path = tmp_path/"callback_timing_receipts.json"
    payload = json.dumps(value).encode()
    if change == "duplicate": payload = payload[:-1]+b',"schema":"duplicate"}'
    if change == "symlink":
        target=tmp_path/"actual.json"; target.write_bytes(payload); path.symlink_to(target)
    else:
        path.write_bytes(payload)
    sha = "0"*64 if change == "SHA" else hashlib.sha256(payload).hexdigest()
    with pytest.raises((RuntimeError,OSError)):
        m.read_callback_receipts(path,sha)


def test_source_manifest_seals_all_new_sources_and_tests():
    with scope(good6000_pins()):
        source = m.runtime_sources()
    for name in ("tools/e2e/research_temporal_route_joint_town03_straight_full209_v1_20261004.py",
        "tools/e2e/collect_temporal_route_joint_town03_straight_full209_v1_20261004.py",
        "tools/e2e/town03_fullroute_goal_cleanup_guard_v1_20261004.py",
        "learning/portable_e2e/temporal_navigation_terminal_policy_v2_20261004.py",
        "learning/portable_e2e/navigation_terminal_retiming_v2_20261004.py",
        "learning/portable_e2e/terminaldecoder_v2_20261004.py", "learning/tests/test_temporal_route_joint_town03_straight_full209_owner_v1_20261004.py"):
        assert source[name] == m.digest(m.ROOT/name)


def test_full_budget_failure_retained_is_not_catalog_success(monkeypatch):
    values = rows(2200)
    _,states,cameras,manifest,*_ = values
    manifest["status"] = "failed"
    manifest["error"] = "CollectionError: BasicAgent did not finish within the driving budget reserved by the maximum total duration and stationary tail"
    manifest["result"].pop("goal_reached")
    assert m.complete_budget_failure_retained(manifest,states,cameras) is True
    assert interval(values,monkeypatch) is False
    states.pop()
    assert m.complete_budget_failure_retained(manifest,states,cameras) is False


@pytest.mark.parametrize("failed_part",["goal","recording","native","terminal","closure","quota"])
def test_diagnostic_subfailures_preserve_callback_measurements_without_PASS(tmp_path,monkeypatch,failed_part):
    values = rows()
    route,states,cameras,manifest,commands,receipts,timings,guards,wrapper = values
    output = tmp_path/"owned_full209_v1"
    episode = output/"episode.partial"
    episode.mkdir(parents=True)
    (episode/"callback_timing_receipts.json").write_bytes(b"{}")
    route_archive = output/"source"/m.ROUTE.relative_to(m.ROOT)
    route_archive.parent.mkdir(parents=True)
    route_archive.write_bytes(b"{}")
    inherited = dict(independent_event={"completed":True},joint_turn_motion_body_policy_wall100ms_passed=True,
        learned_driving_screen_passed=True,recording=dict(passed=True,JPEG_count=6*len(cameras)),raw_source_sha256={})
    monkeypatch.setattr(m,"_goal_diagnostic",lambda *a:copy.deepcopy(inherited))
    monkeypatch.setattr(m.previous,"_verify_v2_failure",lambda:None)
    monkeypatch.setattr(m.previous.previous,"_verify_v1_failure",lambda:None)
    proof = dict(schema=m.live.SCHEMA+".actual_recording_binding",all_copied_mutable_providers_current=True,
        copied_provider_names=["_blank_brake_control","checked_agent_initialization_observation","apply_spawn_z_offset",
            "AcknowledgedControlTransport","exact_camera_bundle","_save_jpeg"],source_image_transformed=False,
        source_timestamp_changed=False,enqueue_and_ACK_share_exact_recorder=True,controller_or_policy_math_changed=True,
        disclosed_navigation_terminal_controller_intervention=True,ordinary_decoder_gate_tracker_PI_unchanged=True,
        actual_recording_bundle_count=len(cameras),actual_after_ACK_receipt_count=len(cameras),pending_source_camera_count=0)
    documents = {"manifest.json":manifest,"catalog_goal_cleanup_receipt.json":wrapper,
        "actual_recording_binding.json":proof,"learned_summary.json":{"counts":{}},
        "quota_walk_timing_receipts.json":{}}
    lists = dict(zip(("states.jsonl","camera_frames.jsonl","learned_control.jsonl","control_receipts.jsonl",
        "wall_timing.jsonl","native_guard.jsonl"),(states,cameras,commands,receipts,timings,guards)))
    lists["recording_after_ack.jsonl"] = [dict(frame=row["frame"]) for row in cameras]
    monkeypatch.setattr(m,"read_json",lambda path:route if Path(path) == m.ROUTE else documents[Path(path).name])
    monkeypatch.setattr(m,"digest",lambda _:m.live.ROUTE_SHA)
    monkeypatch.setattr(m.right,"jsonl",lambda path:lists[Path(path).name])
    monkeypatch.setattr(m,"read_callback_receipts",lambda *a:{})
    callback = {key:True for key in ("callback_receipt_identity_and_SHA_passed","callback_plan_ACK_chain_passed",
        "fresh_callback_to_plan_wall100ms_passed","fresh_callback_to_ACK_wall100ms_passed","actual_six_camera_callback_observed")}
    monkeypatch.setattr(m.live,"audit_receipts",lambda *a:callback)
    native_goal = []
    def native(*a,goal_complete):
        native_goal.append(goal_complete)
        if failed_part == "native": raise RuntimeError("CPU native audit failure")
        return {"native_pre_delivery_stage_closure_passed":True}
    monkeypatch.setattr(m.live,"audit_native_markers",native)
    terminal = mock.Mock(return_value={"passed":failed_part != "terminal"})
    monkeypatch.setattr(m.live,"audit_terminal_navigation",terminal)
    monkeypatch.setattr(m.live,"audit_quota_walks",lambda *a:{"passed":failed_part != "quota"})
    def closure(*a):
        if failed_part == "closure": raise RuntimeError("CPU source closure failure")
    monkeypatch.setattr(m.right,"verify_closure",closure)
    if failed_part == "goal": manifest["result"]["goal_reached"] = False
    if failed_part == "recording": proof["enqueue_and_ACK_share_exact_recorder"] = False
    result = m.diagnostic_report(output,{},dict(declaration_sha256="b"*64))
    assert result["status"] == "FAIL_KNOWN_DEV_FULL209_RESEARCH"
    assert result["fresh_callback_to_plan_wall100ms_passed"] is True
    assert result["fresh_callback_to_ACK_wall100ms_passed"] is True
    assert result["actual_six_camera_callback_observed"] is True
    assert result["learned_driving_screen_passed"] is False
    assert terminal.call_args.kwargs["episode_id"] == "temporal_v2_"+output.name
    assert native_goal == [failed_part != "goal"]
    if failed_part == "native":
        assert result["native_pre_delivery_stage_closure_passed"] is False
        assert "native_pre_delivery_diagnostic_error" in result
    if failed_part == "goal": assert "catalog_goal_qualification_error" in result
    if failed_part == "recording": assert "actual_recording_binding_error" in result
    if failed_part == "closure": assert result["full_raw_and_source_closure_passed"] is False
