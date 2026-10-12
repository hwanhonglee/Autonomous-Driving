"""HH_261002 - Test teacher-only publication with synthetic files; never read an actual capture or render its gallery."""

import copy
import hashlib
import json
from pathlib import Path

import pytest
from PIL import Image

from scripts.e2e import publish_town07_natural_goal_stop_teacher_v6_20261002 as pub


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, allow_nan=False))
    return path


def record_fixture():
    states = [dict(frame=100+i, timestamp=1.+i*.05, capture_phase="driving", x=i*.01, y=0., yaw=0., vx=.2, vy=0.,
                   current_control=dict(throttle=.11, brake=0., steer=0.),
                   control_transport=dict(expected_receipt_sequence=i+1),
                   goal_stop=dict(terminal_mission_phase=("GO", "GO", "BRAKE_TO_STOP", "BRAKE_TO_STOP", "HOLD")[i]))
              for i in range(5)]
    cameras = [dict(frame=states[i]["frame"], timestamp=states[i]["timestamp"], capture_phase="driving",
                    images={name:"images/"+str(i)+"_"+name+".jpg" for name in pub.visual.CAMERAS},
                    source_timestamps={name:states[i]["timestamp"] for name in pub.visual.CAMERAS},
                    original_BGRA_source_sha256={name:"a"*64 for name in pub.visual.CAMERAS},
                    saved_JPEG_sha256={name:"b"*64 for name in pub.visual.CAMERAS}) for i in (0,2,4)]
    receipts = [dict(sequence=i+1, before_frame=99+i, status="ACKNOWLEDGED", server_accepted=True,
                     applied_control=dict(throttle=.99, brake=.9, steer=.8)) for i in range(6)]
    geometry = {i:dict(frame=i, body_polygon_carla_xy=[[-1,-.5],[1,-.5],[1,.5],[-1,.5]]) for i in range(99,105)}
    return states, cameras, receipts, geometry, dict(native=5,camera=3,ACK=6)


def pixel_fixture(cameras):
    return [dict(frame=c["frame"], camera=name, original_BGRA_sha256=c["original_BGRA_source_sha256"][name],
                 saved_JPEG_sha256=c["saved_JPEG_sha256"][name], saved_JPEG_relative_path=c["images"][name],
                 timestamp=c["source_timestamps"][name], encoding="original_carla_bgra8", JPEG_encoder_unchanged=True,
                 pose_or_image_post_transformed=False) for c in cameras for name in pub.visual.CAMERAS]


def archive_fixture(tmp_path, monkeypatch, future=False):
    root=tmp_path.resolve();batch=root/"storage/experiments/validation/2026-10-02/town07_natural_goal_stop_teacher_brakefree_v6"
    case=batch/"pilot_00";episode=case/"episode.partial"
    monkeypatch.setattr(pub,"ROOT",root);monkeypatch.setattr(pub,"BATCH",batch);monkeypatch.setattr(pub,"CASE",case)
    roles={"batch_plan":batch.with_name(batch.name+"_plan.json"),"case_plan":batch/"pilot_00_plan.json",
           "owner":case/"owner_result.json","diagnostic":case/"diagnostic_report.json",
           "body_summary":case/"derived_body/summary.json","body":case/"derived_body/all_native_bodies_and_sweeps.jsonl.gz"}
    monkeypatch.setattr(pub,"OTHER_ROLES",roles)
    states,cameras,receipts,geometry,counts=record_fixture()
    plan=dict(schema=pub.TASK_SCHEMA,run_id="v6",case_id="pilot_00",source_sha256={},external_source_sha256={},asset_sha256={})
    write_json(roles["case_plan"],plan);case_sha=sha(roles["case_plan"])
    write_json(roles["batch_plan"],dict(cases=[dict(declaration_sha256=case_sha)]))
    write_json(roles["owner"],dict(declaration_sha256=case_sha,status="OWNER_FAILURE"))
    write_json(roles["diagnostic"],dict(status="FAIL_RETAINED",learned_control=False,teacher_completion_is_not_learned_goal_stop=True,input_sha256={}))
    write_json(roles["body_summary"],dict(input_sha256={},status="PASS"))
    import gzip
    roles["body"].parent.mkdir(parents=True,exist_ok=True)
    with gzip.open(roles["body"],"wt") as stream:
        for value in geometry.values():stream.write(json.dumps(value)+"\n")
    paths=dict(roles,**{role:episode/name for role,name in pub.EPISODE_ROLES.items()})
    write_json(paths["manifest"],dict(capture_contract=dict(natural_goal_stop_teacher=plan)))
    write_json(paths["route"],dict(route=[dict(x=0.,y=0.),dict(x=50.,y=0.)]))
    for camera in cameras:
        for j,name in enumerate(pub.visual.CAMERAS):
            image=episode/camera["images"][name];image.parent.mkdir(parents=True,exist_ok=True)
            Image.new("RGB",(640,360),(camera["frame"]%255,j*20,20)).save(image,format="JPEG")
            camera["saved_JPEG_sha256"][name]=sha(image)
    values=dict(states=states,cameras=cameras,receipts=receipts,pixels=pixel_fixture(cameras),teacher=[],governor=[],assignments=[],hazards=[])
    for role,value in values.items():
        paths[role].parent.mkdir(parents=True,exist_ok=True)
        paths[role].write_text("".join(json.dumps(row)+"\n" for row in value))
    refs={name:dict(path=str(path),sha256=sha(path)) for name,path in paths.items()}
    if future:
        qa_path=root/"storage/experiments/validation/2026-10-02/qa_v6/summary.json"
        qa=dict(schema=pub.FUTURE_QA_SCHEMA,teacher_task_schema=pub.TASK_SCHEMA,case_plan_sha256=case_sha,
                native_manifest_sha256=sha(paths["manifest"]),source_artifact_sha256={},status="TRUE_FUTURE_DIAGNOSTIC_ONLY_NOT_ADMITTED",
                selected_anchor_count=2,failed_fixed_window_count=1,source_capture_failed=True,training_data_approved=False,
                true_future_point_count=128,required_true_future_point_count=128)
        write_json(qa_path,qa)
        journal=qa_path.parent/"all_fixed_anchor_true_future64.jsonl"
        journal.write_text("".join(json.dumps(dict(necessary_defined_domain_checks_clear=clear,true_future_rows=[dict(valid=True) for _ in range(64)]))+"\n"
                                   for clear in (True,False)))
        refs.update(future_qa_summary=dict(path=str(qa_path),sha256=sha(qa_path)),future_qa_windows=dict(path=str(journal),sha256=sha(journal)))
    caller=write_json(root/"caller.json",dict(schema=pub.CALLER_SCHEMA,episode=str(episode),artifact_pins=refs,counts=counts))
    return caller,sha(caller),paths


@pytest.mark.parametrize("count,expected",[(1,[0]),(2,[0,1]),(6,[0,5]),(7,[0,5,6]),(1782,list(range(0,1782,5))+[1781])])
def test_whole_fixed_membership_includes_exact_last(count,expected):
    assert pub.sampled_indices(count)==expected


@pytest.mark.parametrize("bad",[0,-1,False,True,1.0])
def test_no_empty_or_boolean_membership(bad):
    with pytest.raises(ValueError):pub.sampled_indices(bad)


def test_actual_time_gaps_and_disclosed_terminal_display_hold():
    cameras=[dict(timestamp=5.+i*.100000001) for i in range(12)]
    assert pub.playback_durations(cameras,pub.sampled_indices(12))==[500,500,100,100]
    assert sum(pub.playback_durations(cameras,pub.sampled_indices(12)))==1200


def test_no_source_gap_disappears_at_gif_precision():
    with pytest.raises(ValueError):pub.playback_durations([dict(timestamp=1.),dict(timestamp=1.001)],[0,1])


def test_full_real_records_and_prior_ack_identity():
    states,cameras,receipts,geometry,counts=record_fixture()
    assert len(pub.validate_records(states,cameras,receipts,geometry,counts))==5
    assert pub.observed_prior_ack(states[0],receipts)["sequence"]==1
    assert states[0]["current_control"]["throttle"]==.11 != receipts[0]["applied_control"]["throttle"]


@pytest.mark.parametrize("kind",["native_missing","camera_missing","receipt_missing","body_missing","body_duplicate_extra", "camera_stamp", "camera_missing_sensor", "state_gap", "ack_current", "expected_ack", "bool_expected_ack", "bool_ack", "nan_native", "nan_pedal", "bool_camera_time"])
def test_full_records_fail_closed(kind):
    states,cameras,receipts,geometry,counts=record_fixture()
    if kind=="native_missing":states.pop()
    elif kind=="camera_missing":cameras.pop()
    elif kind=="receipt_missing":receipts.pop()
    elif kind=="body_missing":geometry.pop(102)
    elif kind=="body_duplicate_extra":geometry[200]=geometry[99]
    elif kind=="camera_stamp":cameras[1]["timestamp"]+=.001
    elif kind=="camera_missing_sensor":cameras[1]["images"].pop(pub.visual.CAMERAS[0])
    elif kind=="state_gap":states[1]["frame"]+=1
    elif kind=="ack_current":receipts[0]["before_frame"]=100
    elif kind=="expected_ack":states[1]["control_transport"]["expected_receipt_sequence"]=1
    elif kind=="bool_expected_ack":states[0]["control_transport"]["expected_receipt_sequence"]=True
    elif kind=="bool_ack":receipts[0]["sequence"]=True
    elif kind=="nan_native":states[2]["x"]=float("nan")
    elif kind=="nan_pedal":states[2]["current_control"]["brake"]=float("nan")
    elif kind=="bool_camera_time":cameras[0]["source_timestamps"][pub.visual.CAMERAS[0]]=True
    with pytest.raises((ValueError,KeyError)):pub.validate_records(states,cameras,receipts,geometry,counts)


def test_all_original_pixels_and_encoding_receipts():
    cameras=record_fixture()[1]
    pub.validate_pixels(cameras,pixel_fixture(cameras))


@pytest.mark.parametrize("key,value",[("pose_or_image_post_transformed",True),("JPEG_encoder_unchanged",False),
                                     ("timestamp",2.),("saved_JPEG_sha256","c"*64),("original_BGRA_sha256","d"*64),
                                     ("encoding","transformed_rgb"),("saved_JPEG_relative_path","../escape.jpg")])
def test_source_pixel_mismatch_never_becomes_a_real_camera(key,value):
    cameras=record_fixture()[1];pixels=pixel_fixture(cameras);pixels[0][key]=value
    with pytest.raises(ValueError):pub.validate_pixels(cameras,pixels)


def test_pixel_duplicate_and_missing_fail():
    cameras=record_fixture()[1];pixels=pixel_fixture(cameras)
    with pytest.raises(ValueError):pub.validate_pixels(cameras,pixels[:-1])
    pixels[-1]=copy.deepcopy(pixels[0])
    with pytest.raises(ValueError):pub.validate_pixels(cameras,pixels)


def test_png_phases_only_use_actual_camera_events():
    states,cameras,_,_,_=record_fixture();bundle=dict(states=states,cameras=cameras,by_frame={s["frame"]:s for s in states})
    assert pub.png_indices(bundle)=={"01_first.png":0,"05_last.png":2,"02_cruise.png":0,"03_coast.png":1,"04_HOLD.png":2}
    states[4]["goal_stop"]["terminal_mission_phase"]="GO"
    states[3]["goal_stop"]["terminal_mission_phase"]="HOLD"  # No camera exists at this native event.
    assert pub.missing_png_phases(bundle)=={"04_HOLD.png":"HOLD"}
    assert pub.png_indices(bundle)["04_HOLD.png"]==2


def test_empty_qa_is_not_pass():
    bundle=dict(diagnostic=dict(status="FAIL_RETAINED"),future=None)
    assert "NOT_PROVEN" in pub.outcome_annotation(bundle)


def test_failed149_full1665_is_prominent():
    bundle=dict(diagnostic=dict(status="FAIL_RETAINED"),future=dict(source_capture_failed=True,failed_fixed_window_count=149,selected_anchor_count=1665))
    assert pub.outcome_annotation(bundle)=="Owner: FAIL_RETAINED | future64: FAIL 149/1665 windows"


def test_synthetic_archive_failure_is_publishable_evidence_not_green_success(tmp_path,monkeypatch):
    caller,caller_sha,_=archive_fixture(tmp_path,monkeypatch,future=True)
    bundle=pub.load_archive(caller,caller_sha)
    assert len(bundle["states"])==5 and len(bundle["cameras"])==3 and len(bundle["receipts"])==6
    assert bundle["diagnostic"]["status"]=="FAIL_RETAINED" and bundle["future"]["failed_fixed_window_count"]==1
    assert "FAIL 1/2" in pub.outcome_annotation(bundle)


@pytest.mark.parametrize("kind",["caller_sha","escaped_episode","wrong_role","owner_plan","old_qa", "qa_wrong_native", "qa_missing_full_windows", "JPEG_changed", "missing_JPEG", "JPEG_not_original"])
def test_real_file_provenance_negative_paths(tmp_path,monkeypatch,kind):
    caller,caller_sha,paths=archive_fixture(tmp_path,monkeypatch,future=True)
    value=pub.document(caller)
    if kind=="caller_sha":caller_sha="0"*64
    elif kind=="escaped_episode":value["episode"]=str(pub.CASE/".."/"other"/"episode")
    elif kind=="wrong_role":value["artifact_pins"]["states"]["path"]=str(paths["cameras"])
    elif kind=="owner_plan":
        write_json(paths["owner"],dict(declaration_sha256="0"*64));value["artifact_pins"]["owner"]["sha256"]=sha(paths["owner"])
    elif kind in ("old_qa","qa_wrong_native"):
        path=Path(value["artifact_pins"]["future_qa_summary"]["path"]);qa=pub.document(path)
        qa["teacher_task_schema" if kind=="old_qa" else "native_manifest_sha256"]="v5" if kind=="old_qa" else "0"*64
        write_json(path,qa);value["artifact_pins"]["future_qa_summary"]["sha256"]=sha(path)
    elif kind=="qa_missing_full_windows":value["artifact_pins"].pop("future_qa_windows")
    else:
        camera=pub.rows(paths["cameras"])[0];image=paths["cameras"].parent/camera["images"][pub.visual.CAMERAS[0]]
        if kind=="missing_JPEG":image.unlink()
        elif kind=="JPEG_changed":image.write_bytes(b"changed")
        elif kind=="JPEG_not_original":
            Image.new("RGB",(640,360)).save(image,format="PNG")
            rows=pub.rows(paths["cameras"]);rows[0]["saved_JPEG_sha256"][pub.visual.CAMERAS[0]]=sha(image)
            paths["cameras"].write_text("".join(json.dumps(r)+"\n" for r in rows));value["artifact_pins"]["cameras"]["sha256"]=sha(paths["cameras"])
            pixels=pub.rows(paths["pixels"]);pixels[0]["saved_JPEG_sha256"]=sha(image)
            paths["pixels"].write_text("".join(json.dumps(r)+"\n" for r in pixels));value["artifact_pins"]["pixels"]["sha256"]=sha(paths["pixels"])
    if kind!="caller_sha":write_json(caller,value);caller_sha=sha(caller)
    with pytest.raises((ValueError,KeyError)):pub.load_archive(caller,caller_sha)


def test_original_change_after_read_never_silently_passes(tmp_path):
    path=tmp_path/"one";path.write_bytes(b"original");pins={};pub.add_pin(pins,path.resolve());path.write_bytes(b"mutation")
    with pytest.raises(ValueError):pub.recheck(pins)


def test_create_only_destination_blocks_any_load(tmp_path,monkeypatch):
    output=tmp_path/"existing";output.mkdir();monkeypatch.setattr(pub,"OUTPUT",output)
    monkeypatch.setattr(pub,"load_archive",lambda *a:pytest.fail("No original artifact reads allowed when destination exists"))
    with pytest.raises(ValueError):pub.publish(output,tmp_path/"caller","a"*64)


def test_synthetic_only_full_interval_gif_and_five_pngs_keep_failed_qa(tmp_path,monkeypatch):
    # HH_261002 - Render only small synthetic fixture evidence, not the user's actual V6 recording.
    caller,caller_sha,_=archive_fixture(tmp_path,monkeypatch,future=True)
    output=tmp_path/"fresh_media";monkeypatch.setattr(pub,"OUTPUT",output)
    result=pub.publish(output,caller,caller_sha)
    assert result["GIF"]==1 and result["PNG"]==5
    provenance=pub.document(output/"publication_provenance.json")
    assert provenance["GIF_camera_indices"]==[0,2]
    assert provenance["GIF_encoded_total_duration_ms"]==provenance["GIF_total_duration_ms"]==300
    assert provenance["actual_counts"]==dict(native=5,camera=3,ACK=6,JPEG=18)
    assert provenance["future_QA"]["failed_fixed_window_count"]==1
    assert provenance["training_data_approved"] is provenance["learned_control"] is provenance["autoware_UI_captured"] is False
    assert "FAIL 1/2" in (output/"README.md").read_text()
    assert len(list(output.glob("*.gif")))==1 and len(list(output.glob("*.png")))==5
    for path in output.glob("*.png"):
        with Image.open(path) as image:assert image.size==(1700,860)
    with pytest.raises(ValueError):pub.publish(output,caller,caller_sha)


@pytest.mark.parametrize("value",[None,False,"A"*64,"a"*63,"0"*65])
def test_external_pin_cannot_be_unspecified(value):
    with pytest.raises(ValueError):pub.valid_sha(value)
