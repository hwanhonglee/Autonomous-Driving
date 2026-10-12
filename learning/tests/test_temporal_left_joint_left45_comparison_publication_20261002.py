"""HH_261002 - Synthetic-only publication guards; no actual capture, weights, future, simulator or remote jobs."""
import ast
import copy
import gzip
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

from PIL import Image
import pytest

from scripts.e2e import publish_temporal_left_joint_left45_comparison_20261002 as pub


def control(**changes):
    return dict(dict(throttle=0., brake=0., steer=0., gear=1, manual_gear_shift=True, reverse=False, hand_brake=False), **changes)


def fixture(n=4):
    receipts = [dict(sequence=1, before_frame=100, after_ack_frame=100, reason="pre_capture_bootstrap",
        status="ACKNOWLEDGED", server_accepted=True, requested_control=control(brake=1.)),
        dict(sequence=2, before_frame=100, after_ack_frame=100, reason="stationary_warmup_start",
        status="ACKNOWLEDGED", server_accepted=True, requested_control=control(brake=1.))]
    states, cameras, geometry = [], [], {}
    for i in range(n):
        frame, timestamp = 101+i, 1.+i*.05
        state = dict(frame=frame, timestamp=timestamp, capture_phase="stationary_warmup", x=float(i), y=0., yaw=0., vx=0., vy=0.,
            current_control=control(brake=1.), control_transport=dict(expected_receipt_sequence=i+2, status="PASS"))
        states.append(state); geometry[frame] = dict(frame=frame, body_polygon_carla_xy=[[0.,0.],[1.,0.],[1.,1.],[0.,1.]])
        receipts.append(dict(sequence=i+3, before_frame=frame, after_ack_frame=frame, reason="next_stationary_warmup_control",
            status="ACKNOWLEDGED", server_accepted=True, requested_control=control(brake=1.)))
        if i % 2 == 0:
            cameras.append(dict(frame=frame, timestamp=timestamp, capture_phase="stationary_warmup", camera_order=list(pub.ORDER),
                images={name:f"images/{name}/{frame:08d}.jpg" for name in pub.ORDER},
                source_timestamps={name:timestamp for name in pub.ORDER},
                saved_JPEG_sha256={name:"b"*64 for name in pub.ORDER}, original_BGRA_source_sha256={name:"a"*64 for name in pub.ORDER}))
    return dict(states=states, cameras=cameras, receipts=receipts, commands=[], geometry=geometry,
        by_frame={s["frame"]:s for s in states}, manifest=dict(status="failed"), arm="parent6000",
        route=[dict(x=0.,y=0.),dict(x=20.,y=0.)], endpoint={},
        diagnostic=dict(status="FAIL_RETAINED", bounded_capture_complete=False, joint_motion_body_policy_wall100ms_passed=False),
        observed_prior_learned_effect_count=0, recording_census={})


def test_source_only_contract_has_both_no_approval_and_exact_provider_closure():
    value = pub.public_contract()
    assert value["arm_order"] == ["parent6000","LEFT20000"] and value["expected_complete_counts"] == pub.COUNTS
    assert value["both_outcomes_required"] and value["no_success_frame_selection"]
    assert value["expected_complete_phases"] == dict(stationary_warmup=24,driving=901,stationary_tail=130)
    assert all(value["source_sha256"][name] == sha for name,sha in pub.FROZEN.items())
    assert not value["model_inference_executed"] and not value["vehicle_control_approved"] and not value["exposure_to_ACK_wall_latency_proven"]


def test_exact_two_body_guard_inverse_no_other_clock_or_ACK_change():
    source = ast.parse(Path(pub.recorded.__file__).read_bytes())
    old = next(n for n in source.body if isinstance(n,ast.FunctionDef) and n.name == "validate_records")
    function = pub.records_validator()
    assert function.__globals__["stamp"] is pub.recorded.stamp
    value = fixture()
    assert pub.validate_records(value["states"],value["cameras"],value["receipts"],[],value["geometry"],False) == value["by_frame"]
    assert len([n for n in ast.walk(old) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=="require"]) >= 10


@pytest.mark.parametrize("fault",["frame","clock","camera","order","body","ACK","control","bool","proposal"])
def test_whole_record_no_gap_dropped_pixel_or_future_ACK(fault):
    b = fixture()
    if fault == "frame": b["states"][1]["frame"] += 1
    elif fault == "clock": b["states"][1]["timestamp"] += .001
    elif fault == "camera": b["cameras"].pop()
    elif fault == "order": b["cameras"][0]["camera_order"] = list(pub.visual.CAMERAS)
    elif fault == "body": b["geometry"][100] = dict(frame=100)
    elif fault == "ACK": b["receipts"][2]["server_accepted"] = False
    elif fault == "control": b["states"][1]["current_control"]["throttle"] = .2
    elif fault == "bool": b["states"][0]["vx"] = True
    else: b["commands"] = [dict(ack_receipt_sequence=3,application=dict(acknowledged=False))]
    with pytest.raises(ValueError): pub.validate_records(b["states"],b["cameras"],b["receipts"],b["commands"],b["geometry"],False)


def learned_fixture():
    b = fixture(2); origin = b["states"][0]; state = b["states"][1]
    b["receipts"][2].update(reason="next_driving_control",requested_control=control(throttle=.1))
    state.update(capture_phase="driving", current_control=control(throttle=.1))
    decision = dict(ack_receipt_sequence=3, command_frame=101,inference_frame=101,control_source="learned_trajectory_tracker",
        application=dict(acknowledged=True,applied_source="learned_trajectory_tracker",learned_control_applied=True),
        learned_control_proposed=True, held_between_camera_frames=False, selected=dict(xy_base_m=[[float(i),0.] for i in range(64)]))
    b["commands"] = [decision]
    return b,state,decision


def test_latest_prior_ANY_ACK_uses_original_learned_math_and_ignores_sameframe_proposal():
    b,state,d = learned_fixture()
    b["receipts"].append(dict(sequence=5,before_frame=state["frame"],after_ack_frame=state["frame"],
        reason="next_driving_control",status="ACKNOWLEDGED",server_accepted=True,requested_control=control(brake=1.)))
    r,command,plan,label = pub.prior_ack(state,b["receipts"],b["commands"],b["by_frame"])
    assert r["sequence"]==3 and command is d and len(plan)==64 and "fresh" in label
    assert plan[0] == (b["states"][0]["x"],0.)


def test_actual_emergency_clears_older_purple_plan():
    b,state,d = learned_fixture(); d.update(control_source="emergency_brake")
    d["application"].update(applied_source="emergency_brake",learned_control_applied=False)
    _,_,plan,label = pub.prior_ack(state,b["receipts"],b["commands"],b["by_frame"])
    assert plan==[] and "EMERGENCY" in label


def context_fixture():
    b,state,d = learned_fixture(); b["states"][0]["moving_task"] = dict(future_support_complete=True)
    b["receipts"][2]["requested_control"] = control(); state.update(capture_phase="stationary_tail",current_control=control())
    d.update(control_source="none",diagnostic_control_source="diagnostic_coast_context",learned_control_proposed=False,
        diagnostic_context_only=True,learned_STOP_claimed=False,inference_frame=None)
    d["application"].update(applied_source="none",learned_control_applied=False)
    return b,state,d


def test_context_and_subsequent_zero_manual_coast_never_display_NN_stop():
    b,state,d = context_fixture(); _,command,plan,label = pub.prior_ack(state,b["receipts"],b["commands"],b["by_frame"])
    assert command is d and plan==[] and "NO NN STOP" in label
    b["commands"] = []; b["receipts"][2]["reason"] = "driving_end_brake"
    _,command,plan,label = pub.prior_ack(state,b["receipts"],[],b["by_frame"])
    assert command is None and not plan and "coast" in label


@pytest.mark.parametrize("fault",["future","source","STOP","learned","reason","gear","phase"])
def test_context_cannot_fake_STOP_or_normal_NN_effect(fault):
    b,state,d = context_fixture()
    if fault == "future": b["states"][0]["moving_task"]["future_support_complete"] = False
    elif fault == "source": d["control_source"] = "learned_trajectory_tracker"
    elif fault == "STOP": d["learned_STOP_claimed"] = True
    elif fault == "learned": d["application"]["learned_control_applied"] = True
    elif fault == "reason": b["receipts"][2]["reason"] = "other"
    else:
        b["commands"] = []; b["receipts"][2]["reason"] = "next_stationary_tail_control"
        if fault == "gear": b["receipts"][2]["requested_control"]["gear"] = 2
        else: state["capture_phase"] = "driving"
    with pytest.raises(ValueError): pub.prior_ack(state,b["receipts"],b["commands"],b["by_frame"])


def recorder(b, episode):
    records=[]
    for c in b["cameras"]:
        records.append(dict(schema="portable_e2e.exact_camera_recording_after_ACK.20261001.v1",frame=c["frame"],written_files=[str(episode/c["images"][name]) for name in pub.ORDER],
            enqueued_camera_count=6,written_camera_count=6,all_queued_images_written=True,failure=None,
            no_source_image_dropped=True,source_timestamp_changed=False,source_pixels_changed=False,
            acknowledged_before_recording=True,ACK_receipt_sequence=c["frame"]-101+3))
    proof=dict(schema=pub.CAPTURE_SCHEMA+".actual_recording_binding",copied_provider_names=list(pub.PROVIDERS),
        all_copied_mutable_providers_current=True,enqueue_and_ACK_share_exact_recorder=True,
        sensor_callback_entry_observed=False,exposure_to_ACK_wall_latency_proven=False,
        actual_recording_bundle_count=len(records),actual_after_ACK_receipt_count=len(records),pending_source_camera_count=0)
    return records,proof


@pytest.mark.parametrize("alias",["episode","episode.partial"])
def test_recording_exact_order_alias_and_full_ACK_proof(tmp_path,alias):
    b=fixture();b["episode"]=tmp_path/"episode"
    rows,proof=recorder(b,tmp_path/alias)
    assert pub.validate_recording(b,rows,proof)["all_recorded_after_ACK"] is True


@pytest.mark.parametrize("fault",["order","missing","duplicate","mixedalias","escape","ACK","source","count","claimedlatency"])
def test_recording_never_accepts_display_order_missing_sources_or_fake_ACK(tmp_path,fault):
    b=fixture();b["episode"]=tmp_path/"episode";rows,p=recorder(b,b["episode"])
    if fault=="order": rows[0]["written_files"].reverse()
    elif fault=="missing": rows.pop()
    elif fault=="duplicate": rows[0]["written_files"][1]=rows[0]["written_files"][0]
    elif fault=="mixedalias": rows[0]["written_files"][0]=rows[0]["written_files"][0].replace("/episode/","/episode.partial/")
    elif fault=="escape": rows[0]["written_files"][0]="/foreign/images/x.jpg"
    elif fault=="ACK": rows[0]["ACK_receipt_sequence"]=999
    elif fault=="source": p["copied_provider_names"].pop()
    elif fault=="count": p["actual_after_ACK_receipt_count"]-=1
    else:p["exposure_to_ACK_wall_latency_proven"]=True
    with pytest.raises(ValueError):pub.validate_recording(b,rows,p)


def test_failed_final_pending_pixels_retained_without_fabricating_ACK(tmp_path):
    b=fixture();b["episode"]=tmp_path/"episode.partial";rows,p=recorder(b,b["episode"])
    rows[-1].update(acknowledged_before_recording=False,ACK_receipt_sequence=None)
    p.update(actual_after_ACK_receipt_count=1,actual_recording_bundle_count=1,pending_source_camera_count=6)
    result=pub.validate_recording(b,rows,p)
    assert not result["all_recorded_after_ACK"] and result["pending_source_camera_count"]==6
    b["manifest"]["status"]="complete"
    with pytest.raises(ValueError):pub.validate_recording(b,rows,p)


def test_whole_clock_rule_and_fixed_time_indices_are_not_outcome_selected():
    cameras=[dict(timestamp=1.+i*.1) for i in range(528)]
    indices=pub.recorded.sampled_indices(528);durations=pub.recorded.playback_durations(cameras,indices)
    assert len(indices)==107 and indices[-1]==527 and sum(durations)==52800
    assert list(pub.screenshot_indices(528).values())==[0,132,264,395,527]
    assert list(pub.screenshot_indices(3).values())==[0,0,1,1,2]
    assert pub.recorded.playback_durations([dict(timestamp=1.)],[0])==[100]


def complete_fixture():
    b=fixture(1055);receipts=b["receipts"][:2];commands=[]
    def send(frame,reason,ctrl,source=None):
        seq=len(receipts)+1
        receipts.append(dict(sequence=seq,before_frame=frame,after_ack_frame=frame,reason=reason,
            status="ACKNOWLEDGED",server_accepted=True,requested_control=ctrl))
        if source:
            decision=dict(ack_receipt_sequence=seq,command_frame=frame,control_source=source,
                application=dict(acknowledged=True,applied_source="expert" if source=="expert_startup" else "none",learned_control_applied=False))
            if source=="none":decision.update(diagnostic_control_source="diagnostic_coast_context",learned_control_proposed=False,
                diagnostic_context_only=True,learned_STOP_claimed=False)
            commands.append(decision)
    for i,state in enumerate(b["states"]):
        phase="stationary_warmup" if i<24 else "driving" if i<925 else "stationary_tail"
        state.update(capture_phase=phase,current_control=copy.deepcopy(receipts[-1]["requested_control"]),
            control_transport=dict(expected_receipt_sequence=receipts[-1]["sequence"],status="PASS"))
        if phase=="stationary_warmup":
            send(state["frame"],"next_stationary_warmup_control",control(brake=1.))
            if i==23:send(state["frame"],"initial_drive_control",control(throttle=.1),"expert_startup")
        elif phase=="driving":
            if i==924:
                state["moving_task"]=dict(future_support_complete=True)
                send(state["frame"],"next_driving_control",control(),"none")
                send(state["frame"],"driving_end_brake",control())
            else:send(state["frame"],"next_driving_control",control(throttle=.1),"expert_startup")
        else:send(state["frame"],"next_stationary_tail_control",control())
    send(b["states"][-1]["frame"],"left45_post_capture_cleanup_full_brake",control(brake=1.))
    for camera in b["cameras"]:camera["capture_phase"]=b["by_frame"][camera["frame"]]["capture_phase"]
    b.update(receipts=receipts,commands=commands)
    return b


def test_full1055_528_1060_all_original_phases_and_nonlearned_boundary():
    b=complete_fixture()
    assert (len(b["states"]),len(b["cameras"]),len(b["receipts"]))==(1055,528,1060)
    assert pub.validate_records(b["states"],b["cameras"],b["receipts"],b["commands"],b["geometry"],True)==b["by_frame"]
    _,_,plan,label=pub.prior_ack(b["states"][925],b["receipts"],b["commands"],b["by_frame"])
    assert plan==[] and "coast" in label
    b["states"].pop()
    with pytest.raises(ValueError):pub.validate_records(b["states"],b["cameras"],b["receipts"],b["commands"],b["geometry"],True)


def test_no_authority_cannot_read_actual_attempts(monkeypatch):
    monkeypatch.setattr(pub.archive.Pins,"file",lambda *args:None)
    monkeypatch.setattr(pub.archive.Pins,"json",lambda *args:{})
    monkeypatch.setattr(pub,"public_contract",lambda:{})
    monkeypatch.setattr(pub.archive,"inventory",lambda *args:pytest.fail("cannot read archive before exactcaller"))
    with pytest.raises(ValueError):pub.load_pair("/synthetic/caller.json","a"*64)


def synthetic_images(tmp_path,arm):
    b=fixture();episode=tmp_path/arm/"episode";episode.mkdir(parents=True)
    for camera in b["cameras"]:
        for name in pub.ORDER:
            p=episode/camera["images"][name];p.parent.mkdir(parents=True,exist_ok=True)
            Image.new("RGB",(640,360),(20,30,40)).save(p,format="JPEG")
    b["episode"]=episode;b["arm"]=arm
    return b


def test_synthetic_renderer_short_header_full_FOV_and_prior_observed_phase(tmp_path):
    b=synthetic_images(tmp_path,"parent6000")
    before=pub.displayed_receipt(b,0)
    picture=pub.render(b,0)
    assert picture.size==(1700,860) and picture.mode=="RGB"
    assert before["prior_ACK_sequence"]==2 and before["purple_plan_displayed"] is False
    assert before["capture_phase"]=="stationary_warmup" and "warmup" in before["source_label"]


def test_validate_only_has_no_output_and_includes_BOTH_failures(tmp_path,monkeypatch):
    bundles=[synthetic_images(tmp_path,a) for a in pub.ARMS]
    monkeypatch.setattr(pub,"OUTPUT",tmp_path/"media")
    monkeypatch.setattr(pub,"load_pair",lambda *args,**kwargs:({},bundles,None))
    result=pub.publish(pub.OUTPUT,Path("/synthetic/caller.json"),"a"*64,validate_only=True)
    assert set(result["counts"])==set(pub.ARMS) and result["output_created"] is False and not pub.OUTPUT.exists()


def test_create_only_BOTH_actual_synthetic_GIF_PNG_and_postcheck(tmp_path,monkeypatch):
    bundles=[synthetic_images(tmp_path,a) for a in pub.ARMS];pins=SimpleNamespace(values={},recheck=lambda:None)
    monkeypatch.setattr(pub,"OUTPUT",tmp_path/"media");monkeypatch.setattr(pub,"PAIR",tmp_path)
    monkeypatch.setattr(pub,"load_pair",lambda *args,**kwargs:({"runs":{a:{} for a in pub.ARMS}},bundles,pins))
    monkeypatch.setattr(pub.archive,"inventory",lambda *args:set())
    monkeypatch.setattr(pub.shutil,"disk_usage",lambda *args:SimpleNamespace(free=20*1024**3))
    result=pub.publish(pub.OUTPUT,Path("/synthetic/caller.json"),"a"*64)
    assert result["GIF"]==2 and result["PNG"]==10
    report=json.loads((pub.OUTPUT/"publication_provenance.json").read_text())
    assert [r["arm"] for r in report["results"]]==list(pub.ARMS)
    assert all(r["original_diagnostic"]["status"]=="FAIL_RETAINED" for r in report["results"])
    assert all(r["GIF_camera_indices"]==[0,1] for r in report["results"])
    assert report["groundtruth_future_used_as_model_plan"] is False and report["model_promoted"] is False
    with pytest.raises(ValueError):pub.publish(pub.OUTPUT,Path("/synthetic/caller.json"),"a"*64)


def test_static_cli_requires_pins_and_never_approves(monkeypatch,capsys):
    monkeypatch.setattr(pub,"public_contract",lambda:dict(schema=pub.CALLER_SCHEMA,model_promoted=False))
    assert pub.main(["--print-contract"])==0
    assert json.loads(capsys.readouterr().out)["model_promoted"] is False
    with pytest.raises(ValueError):pub.main([])
    with pytest.raises(ValueError):pub.main(["--print-contract","--validate-only"])


def write_json(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,sort_keys=True,allow_nan=False))
    return pub.recorded.digest(path)


def write_rows(path,values):
    path.parent.mkdir(parents=True,exist_ok=True)
    payload="".join(json.dumps(v,allow_nan=False)+"\n" for v in values)
    if path.suffix==".gz":
        with gzip.open(path,"wt") as stream:stream.write(payload)
    else:path.write_text(payload)


def pinned_pair(tmp_path,monkeypatch):
    # HH_261002 - Real reader and first-read pins, all synthetic files; never open an actual route/model/capture.
    root=tmp_path.resolve();pair=root/"storage/experiments/validation/2026-10-02/pair"
    # HH_261002 - Other collected regression modules may import Torch; isolate ONLY this synthetic reader's process descriptor.
    monkeypatch.setattr(pub,"sys",SimpleNamespace(modules={k:v for k,v in pub.sys.modules.items() if k not in ("torch","carla")}))
    monkeypatch.setattr(pub,"ROOT",root);monkeypatch.setattr(pub,"PAIR",pair);monkeypatch.setattr(pub,"OUTPUT",root/"media")
    contract=dict(schema=pub.CALLER_SCHEMA,source_sha256={},font_sha256={},arm_order=list(pub.ARMS))
    monkeypatch.setattr(pub,"public_contract",lambda:copy.deepcopy(contract));monkeypatch.setattr(pub,"verify_frozen",lambda:None)
    sources={}
    for i in range(30):
        name=f"tools/source_{i}.py";p=root/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text("# synthetic\n")
        sources[name]=pub.recorded.digest(p)
    endpoint={k:"a"*64 for k in ("weights_sha256","metadata_sha256","checkpoint_sha256","declaration_sha256")}
    auth=dict(schema=pub.AUTHORITY_SCHEMA,mode="RESEARCH_ONLY",fixed_endpoint_completed=True,
        model_promoted=False,vehicle_control_approved=False,source_sha256=sources,parent=endpoint,endpoint=endpoint)
    authpath=root/"storage/experiments/plans/2026-10-02/authority.json";authsha=write_json(authpath,auth)
    route=dict(town="Town01",scenario="left",route=[dict(x=0.,y=0.),dict(x=20.,y=0.)])
    routebytes=json.dumps(route,sort_keys=True,allow_nan=False).encode();monkeypatch.setattr(pub,"ROUTE_SHA",hashlib.sha256(routebytes).hexdigest())
    plans={};inventories={}
    for arm in pub.ARMS:
        run=pair/arm;episode=run/"episode.partial";episode.mkdir(parents=True)
        plan=dict(schema=pub.CAPTURE_SCHEMA+".observation_task",arm=arm,arm_order=list(pub.ARMS),owner_output=str(run),
            selected_four_endpoint_sha256=endpoint,fixed_endpoint_updates=6000 if arm=="parent6000" else 20000,
            completed_run_authority_sha256=authsha,speed_preview_s=.5,PI_target_cap_mps=2.4,CPU_threads=4,CPU_interop=1,
            training_data_approved=False,model_promoted=False,vehicle_control_approved=False,
            source_sha256=sources,external_source_sha256={},asset_sha256={})
        for name in sources:
            p=run/"source"/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes((root/name).read_bytes())
        plans[arm]=write_json(pair/(arm+"_plan.json"),plan);write_json(run/"declaration.json",plan)
        owner=dict(declaration_sha256=plans[arm]);write_json(run/"owner_result.json",owner)
        b=fixture();b["episode"]=episode
        manifest=dict(status="failed",training_data_approved=False,vehicle_control_approved=False,
            capture_contract=dict(learned_LEFT45_protocol={k:v for k,v in plan.items() if k not in ("source_sha256","asset_sha256","external_source_sha256")}))
        write_json(episode/"manifest.json",manifest);write_json(episode/"route.json",route)
        for name,key in (("states.jsonl","states"),("camera_frames.jsonl","cameras"),("control_receipts.jsonl","receipts")):
            write_rows(episode/name,b[key])
        pixels=[]
        for camera in b["cameras"]:
            for name in pub.ORDER:
                p=episode/camera["images"][name];p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(b"synthetic not decoded JPEG")
                saved=pub.recorded.digest(p);camera["saved_JPEG_sha256"][name]=saved
                pixels.append(dict(frame=camera["frame"],camera=name,original_BGRA_sha256="a"*64,saved_JPEG_sha256=saved,
                    saved_JPEG_relative_path=camera["images"][name],timestamp=camera["source_timestamps"][name],
                    encoding="original_carla_bgra8",JPEG_encoder_unchanged=True,pose_or_image_post_transformed=False))
        write_rows(episode/"camera_frames.jsonl",b["cameras"]);write_rows(episode/"actual_camera_pixels.jsonl",pixels)
        rows,proof=recorder(b,episode);write_rows(episode/"recording_after_ack.jsonl",rows);write_json(episode/"actual_recording_binding.json",proof)
        body=dict(full_body_denominator=4,full_sweep_denominator=3,ten_centimeter_inset_preserved=True,status="FAIL")
        write_json(run/"derived_body/summary.json",body);write_rows(run/"derived_body/all_native_bodies_and_sweeps.jsonl.gz",list(b["geometry"].values()))
        raw={str(episode/name):pub.recorded.digest(episode/name) for name in pub.archive.inventory(episode)}
        diag=dict(schema=pub.OWNER_SCHEMA+".diagnostic_report",arm=arm,original_owner=owner,model_promoted=False,
            vehicle_control_approved=False,independent_validation=False,TRAIN_seen_route=True,raw_source_sha256=raw,
            full_capture_body_clearance=body,bounded_capture_complete=False,joint_motion_body_policy_wall100ms_passed=False,status="FAIL_RETAINED")
        write_json(run/"diagnostic_report.json",diag)
        inventories[arm]={name:pub.recorded.digest(run/name) for name in pub.archive.inventory(run)}
    batch=dict(schema=pub.OWNER_SCHEMA+".batch",arm_order=list(pub.ARMS),physical_case_count=2,case_sha256=plans,
        completed_run_authority_sha256=authsha,automatic_retry=False,outcome_selected_retry=False)
    batchsha=write_json(pair/"batch_plan.json",batch)
    caller=dict(contract=contract,batch_plan_sha256=batchsha,case_plan_sha256=plans,completed_run_authority=str(authpath),
        completed_run_authority_sha256=authsha,runs=inventories,publication_authorized=True)
    callerpath=root/"caller.json";callersha=write_json(callerpath,caller)
    return callerpath,callersha,caller


@pytest.mark.parametrize("module",["torch","carla"])
def test_actual_reader_rejects_model_or_simulator_process_before_caller_payload(monkeypatch,module):
    monkeypatch.setattr(pub,"sys",SimpleNamespace(modules={module:object()}))
    monkeypatch.setattr(pub.archive.Pins,"file",lambda *args:pytest.fail("must reject before payload"))
    with pytest.raises(ValueError):pub.load_pair(Path("/synthetic/caller.json"),"a"*64)


def test_REAL_full_pinned_pair_reader_sortedJSON_BOTH_failures_and_all30sources(tmp_path,monkeypatch):
    path,sha,caller=pinned_pair(tmp_path,monkeypatch)
    original,bundles,pins=pub.load_pair(path,sha)
    assert original==caller and [b["arm"] for b in bundles]==list(pub.ARMS)
    assert all(b["observed_prior_learned_effect_count"]==0 for b in bundles)
    assert all(not b["diagnostic"]["bounded_capture_complete"] for b in bundles)
    assert all(b["recording_census"]["actual_after_ACK_count"]==2 for b in bundles)
    assert len(pins.values)>130
    pins.recheck()


@pytest.mark.parametrize("fault",["missingarm","modifiedraw","inventory","casepin","authority","newsource","promote","fakePASS"])
def test_REAL_paired_reader_rejects_partial_selection_mutation_and_promotion(tmp_path,monkeypatch,fault):
    path,sha,caller=pinned_pair(tmp_path,monkeypatch)
    if fault=="missingarm":caller["runs"].pop("LEFT20000")
    elif fault=="modifiedraw":(pub.PAIR/"parent6000/episode.partial/states.jsonl").write_text("changed")
    elif fault=="inventory":(pub.PAIR/"LEFT20000/unpinned.txt").write_text("changed")
    elif fault=="casepin":caller["case_plan_sha256"]["parent6000"]="b"*64
    elif fault=="authority":caller["completed_run_authority_sha256"]="b"*64
    elif fault=="newsource":(pub.ROOT/"tools/source_0.py").write_text("changed")
    else:
        p=pub.PAIR/"parent6000/diagnostic_report.json";value=json.loads(p.read_text())
        value["model_promoted" if fault=="promote" else "joint_motion_body_policy_wall100ms_passed"]=True
        caller["runs"]["parent6000"]["diagnostic_report.json"]=write_json(p,value)
    sha=write_json(path,caller)
    with pytest.raises((ValueError,RuntimeError)):pub.load_pair(path,sha)
