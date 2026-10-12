"""HH_261002 - Synthetic-only LEFT teacher publication contracts; never actual raw/image/qualification execution."""
import copy
import hashlib
import inspect
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from PIL import Image

from scripts.e2e import publish_town01_low_speed_left45_teacher_v2_20261002 as pub
from learning.tests import test_town07_natural_goal_teacher_v6_publication_20261002 as base_tests


def record_fixture():
    states, cameras, receipts, geometry, counts = base_tests.record_fixture()
    geometry.pop(99)
    for state in states: state.pop("goal_stop")
    return states, cameras, receipts, geometry, counts


def full_record_fixture():
    states = [dict(frame=100+i, timestamp=1+i*.05, capture_phase="stationary_warmup" if i < 24 else "driving" if i < 925 else "stationary_tail",
        x=i*.05, y=0., yaw=0., vx=.2, vy=0., current_control=dict(throttle=.11, brake=0., steer=0.),
        control_transport=dict(expected_receipt_sequence=i+1)) for i in range(1055)]
    cameras = [dict(frame=s["frame"], timestamp=s["timestamp"], capture_phase=s["capture_phase"],
        images={k:f"images/{k}/{s['frame']:08d}.jpg" for k in pub.visual.CAMERAS},
        source_timestamps={k:s["timestamp"] for k in pub.visual.CAMERAS},
        original_BGRA_source_sha256={k:"a"*64 for k in pub.visual.CAMERAS}, saved_JPEG_sha256={k:"b"*64 for k in pub.visual.CAMERAS}) for s in states[::2]]
    receipts = [dict(sequence=i+1, before_frame=99+i, status="ACKNOWLEDGED", server_accepted=True,
        applied_control=dict(throttle=.99, brake=.8, steer=.7)) for i in range(1060)]
    geometry = {s["frame"]:dict(frame=s["frame"], body_polygon_carla_xy=[[-1,-.5],[1,-.5],[1,.5],[-1,.5]]) for s in states}
    return states, cameras, receipts, geometry


def test_exact_frozen_renderer_and_qualified_source_contract_no_payloads():
    pub.verify_frozen()
    value = pub.public_contract()
    assert value["qualification_source_count"] == 99
    assert value["counts"] == dict(native=1055,camera=528,ACK=1060,JPEG=3168)
    assert value["body_rows"] == 1055 and value["swept_pairs"] == 1054
    assert len(value["artifact_paths"]) == 16
    assert value["teacher_not_NN"] is True and value["STOP_HOLD_GO_semantics_or_model_approval"] is False
    assert value["source_sha256"][str(Path(pub.previous.__file__).resolve().relative_to(pub.ROOT))] == pub.PARENT_SHA
    assert value["source_sha256"][str(Path(pub.admission.__file__).resolve().relative_to(pub.ROOT))] == pub.ADMISSION_SHA


def test_inverse_guard_clone_does_not_mutate_original_validator_or_globals():
    original = pub.previous.validate_records
    namespace = dict(pub.previous.__dict__)
    validator = pub.records_validator()
    states,cameras,receipts,geometry,counts = record_fixture()
    assert len(validator(states,cameras,receipts,geometry,counts)) == 5
    with pytest.raises(ValueError): original(states,cameras,receipts,geometry,counts)
    assert pub.previous.validate_records is original
    assert pub.previous.__dict__ == namespace


def test_full_1055_528_1060_native_clock_ACK_and_body_denominators():
    states,cameras,receipts,geometry = full_record_fixture()
    assert len(pub.validate_records(states,cameras,receipts,geometry)) == 1055
    pub.previous.validate_pixels(cameras, base_tests.pixel_fixture(cameras))
    assert len(cameras)*6 == 3168
    assert pub.previous.observed_prior_ack(states[0],receipts)["sequence"] == 1
    assert states[0]["current_control"]["throttle"] == .11 != receipts[0]["applied_control"]["throttle"]


@pytest.mark.parametrize("kind", ["native_missing", "camera_missing", "ACK_missing", "body_missing", "fake_bootstrap", "duplicate_body", "native_gap",
    "camera_time", "cam6_missing", "native_nan", "native_bool_time", "pedal_nan", "same_frame_ACK", "ACK_sequence_bool", "expected_ACK_bool"])
def test_complete_records_reject_any_missing_or_synthetic_evidence(kind):
    states,cameras,receipts,geometry = full_record_fixture()
    if kind == "native_missing": states.pop()
    elif kind == "camera_missing": cameras.pop()
    elif kind == "ACK_missing": receipts.pop()
    elif kind == "body_missing": geometry.pop(states[4]["frame"])
    elif kind == "fake_bootstrap": geometry[99] = dict(frame=99)
    elif kind == "duplicate_body": geometry[99] = geometry[100]
    elif kind == "native_gap": states[3]["frame"] += 1
    elif kind == "camera_time": cameras[2]["timestamp"] += .001
    elif kind == "cam6_missing": cameras[1]["images"].pop(pub.visual.CAMERAS[0])
    elif kind == "native_nan": states[4]["vx"] = float("nan")
    elif kind == "native_bool_time": states[4]["timestamp"] = True
    elif kind == "pedal_nan": states[4]["current_control"]["brake"] = float("inf")
    elif kind == "same_frame_ACK": receipts[0]["before_frame"] = states[0]["frame"]
    elif kind == "ACK_sequence_bool": receipts[0]["sequence"] = True
    else: states[0]["control_transport"]["expected_receipt_sequence"] = True
    with pytest.raises((ValueError,KeyError)): pub.validate_records(states,cameras,receipts,geometry)


def test_full52_7s_fixed2hz_plus_exactlast_and_fixedfive_PNGs_not_phase_or_success_selection():
    _,cameras,_,_ = full_record_fixture()
    indices = pub.previous.sampled_indices(len(cameras))
    durations = pub.previous.playback_durations(cameras, indices)
    assert indices == list(range(0,528,5))+[527]
    assert len(indices) == 107 and indices[-1] == 527
    assert durations[-2:] == [200,100] and sum(durations) == 52800
    assert pub.previous.stamp(cameras[-1])-pub.previous.stamp(cameras[0]) == 52700000000
    assert list(pub.fixed_png_indices(dict(cameras=cameras)).values()) == [0,132,264,395,527]


@pytest.mark.parametrize("count", [0,1,527,529])
def test_fixedfive_PNG_count_cannot_be_adaptive(count):
    with pytest.raises(ValueError): pub.fixed_png_indices(dict(cameras=[{}]*count))


def test_real_bgra_jpeg_pixel_receipts_are_unchanged_original_math():
    _,cameras,_,_,_ = record_fixture()
    pixels = base_tests.pixel_fixture(cameras)
    pub.previous.validate_pixels(cameras,pixels)
    pixels[0]["saved_JPEG_relative_path"] = "../replaced.jpg"
    with pytest.raises(ValueError): pub.previous.validate_pixels(cameras,pixels)


def test_true_outcome_uses_full347_and_does_not_call_it_NN_or_stop_success():
    bundle = dict(diagnostic=dict(status="PASS_ACTUAL_TEACHER_PENDING_TRUE_FUTURE_QA"),
        future=dict(necessary_checks_clear=True, failed_fixed_window_count=0, fixed_anchor_count=347))
    value = pub.outcome_annotation(bundle)
    assert "0/347" in value and "existing research TRAIN" in value and "NO model approval" in value
    bundle["future"].update(necessary_checks_clear=False,failed_fixed_window_count=149)
    assert "FAIL_RETAINED 149/347" in pub.outcome_annotation(bundle)


def test_reader_checks_source_caller_all19_artifacts_before_original_qualification_or_raw(monkeypatch,tmp_path):
    caller = tmp_path/"caller.json"; caller.write_text(json.dumps(dict(contract={},publication_authorized=False)))
    monkeypatch.setattr(pub,"public_contract",lambda: {})
    monkeypatch.setattr(pub.admission,"evidence",lambda *a:pytest.fail("No native/QA reads without full rootcaller"))
    with pytest.raises(ValueError): pub.load_archive(caller,pub.digest(caller))


@pytest.mark.parametrize("fault", [None, "receipt", "teacher", "QA_count", "JPEG_shape"])
def test_full_reader_actual_ABI_synthetic_only_rederives_qualification_and_all_denominators(monkeypatch,tmp_path,fault):
    # HH_261002 - Every payload reader is hermetic; paths identify the fixed ABI but NO actual source/raw/JPEG is opened.
    wanted=dict(artifact_paths=[str(tmp_path/f"raw{i}") for i in range(16)],qualification=str(pub.admission.RECEIPT),
        QA_summary=str(pub.admission.QA_SUMMARY),QA_windows=str(pub.admission.QA_WINDOWS),source_sha256={},font_sha256={},
        episode=str(pub.CASE/"episode"))
    qualified=dict(source_sha256={f"synthetic_source{i}":"a"*64 for i in range(99)},input_sha256={},selected_camera_anchors=347,
        true_future_points=22208,research_training_authorized=True,model_promoted=False,vehicle_control_approved=False)
    future=dict(necessary_checks_clear=True,failed_fixed_window_count=0,fixed_anchor_count=347,true_future_point_count=22208)
    manifest=dict(capture_contract=dict(left45_protocol=pub.admission.task.fixed_contract("v2","pilot_00")),
        control_source="Disclosed_BasicAgent_original_LEFT_teacher_not_NN")
    plan=dict(schema=pub.admission.task.SCHEMA,run_id="v2",case_id="pilot_00",source_sha256={},external_source_sha256={},asset_sha256={})
    states,cameras,receipts,geometry=full_record_fixture()
    episode=Path(wanted["episode"]); authority=tmp_path/"authority.json"
    docs={str(pub.CASE_PLAN):plan,str(episode/"manifest.json"):manifest,str(pub.CASE/"diagnostic_report.json"):dict(status="SYNTHETIC"),
        str(episode/"route.json"):dict(route=[]),str(pub.admission.QA_SUMMARY):future}
    streams={str(episode/"states.jsonl"):states,str(episode/"camera_frames.jsonl"):cameras,str(episode/"control_receipts.jsonl"):receipts,
        str(pub.CASE/"derived_body/all_native_bodies_and_sweeps.jsonl.gz"):list(geometry.values()),
        str(episode/"actual_camera_pixels.jsonl"):base_tests.pixel_fixture(cameras)}
    caller=dict(contract=wanted,artifact_sha256={name:"a"*64 for name in {*wanted["artifact_paths"],wanted["qualification"],wanted["QA_summary"],wanted["QA_windows"]}},
        authority_path=str(authority),authority_sha256="a"*64,publication_authorized=True)
    caller_path=tmp_path/"caller.json";caller_path.write_text(json.dumps(caller))
    reads=[]
    def add_pin(pins,path,expected=None):
        path=Path(path);reads.append(str(path));pins[str(path)]=expected or "a"*64;return path
    monkeypatch.setattr(pub,"public_contract",lambda:wanted);monkeypatch.setattr(pub.previous,"add_pin",add_pin)
    monkeypatch.setattr(pub.previous,"document",lambda path:copy.deepcopy(docs[str(path)]))
    monkeypatch.setattr(pub,"rows",lambda path:copy.deepcopy(streams[str(path)]))
    monkeypatch.setattr(pub.admission,"evidence",lambda path,pin:copy.deepcopy(qualified))
    monkeypatch.setattr(pub.admission,"document",lambda path:dict(qualified,altered=True) if fault=="receipt" else copy.deepcopy(qualified))
    monkeypatch.setattr(pub.previous,"recheck",lambda pins:None)
    if fault=="teacher":manifest["control_source"]="NN"
    if fault=="QA_count":future["true_future_point_count"]-=1
    class SyntheticJPEG:
        format="JPEG";size=(639,360) if fault=="JPEG_shape" else (640,360)
        def __enter__(self):return self
        def __exit__(self,*args):pass
        def load(self):pass
    monkeypatch.setattr(pub.Image,"open",lambda path:SyntheticJPEG())
    if fault is not None:
        with pytest.raises(ValueError):pub.load_archive(caller_path,"a"*64)
    else:
        value=pub.load_archive(caller_path,"a"*64)
        assert len(value["states"])==1055 and len(value["cameras"])==528 and len(value["receipts"])==1060
        assert len([name for name in reads if "/images/" in name])==3168
        assert str(authority) in value["pins"] and value["qualification"]==qualified


@pytest.mark.parametrize("key", ["schema", "counts", "source_sha256", "output", "teacher_not_NN"])
def test_changed_contract_refused_before_payloads(monkeypatch,tmp_path,key):
    wanted = dict(schema=pub.CALLER_SCHEMA,counts=dict(pub.COUNTS),source_sha256={},output=str(pub.OUTPUT),teacher_not_NN=True)
    monkeypatch.setattr(pub,"public_contract",lambda: wanted)
    changed = copy.deepcopy(wanted); changed[key] = "changed"
    value = dict(contract=changed,artifact_sha256={},authority_path="/unused",authority_sha256="a"*64,publication_authorized=True)
    path=tmp_path/"caller.json"; path.write_text(json.dumps(value))
    monkeypatch.setattr(pub.admission,"evidence",lambda *a:pytest.fail("No native inspection for wrong contract"))
    with pytest.raises(ValueError): pub.load_archive(path,pub.digest(path))


def test_publisher_full_inverse_AST_scope_and_five_PNG_code_original_unchanged():
    original = pub.previous.publish
    saved = dict(pub.previous.__dict__)
    copied = pub.publisher_function()
    assert copied is not original and copied.__globals__["SCHEMA"] == pub.SCHEMA
    assert copied.__globals__["png_indices"] is pub.fixed_png_indices
    assert copied.__globals__["render"] is pub.render
    assert copied.__globals__["load_archive"] is pub.load_archive
    assert copied.__globals__["outcome_annotation"] is pub.outcome_annotation
    assert copied.__globals__["missing_png_phases"]({}) == {}
    assert pub.previous.publish is original and pub.previous.__dict__ == saved


def test_existing_output_and_lowdisk_refused_before_raw_access(tmp_path,monkeypatch):
    output=tmp_path/"existing";output.mkdir();monkeypatch.setattr(pub,"OUTPUT",output)
    monkeypatch.setattr(pub,"publisher_function",lambda:pytest.fail("No original reads"))
    with pytest.raises(ValueError): pub.publish(output,tmp_path/"caller","a"*64)
    fresh=tmp_path/"fresh";monkeypatch.setattr(pub,"OUTPUT",fresh)
    monkeypatch.setattr(pub.shutil,"disk_usage",lambda _:SimpleNamespace(free=5*1024**3))
    with pytest.raises(ValueError): pub.publish(fresh,tmp_path/"caller","a"*64)


def test_real_cloned_write_pipeline_synthetic_only_fullinterval_five_PNG_and_false_approval(tmp_path,monkeypatch):
    output=tmp_path/"media";monkeypatch.setattr(pub,"OUTPUT",output)
    states,cameras,receipts,geometry,_=record_fixture()
    bundle=dict(states=states,cameras=cameras,receipts=receipts,geometry=geometry,by_frame={s["frame"]:s for s in states},
        route=[],diagnostic=dict(status="SYNTHETIC_NOT_ACTUAL"),future=dict(necessary_checks_clear=True,failed_fixed_window_count=0,fixed_anchor_count=347),pins={})
    monkeypatch.setattr(pub,"load_archive",lambda *a:bundle)
    monkeypatch.setattr(pub,"render",lambda b,i:Image.new("RGB",(40,24),(i*40,2,3)))
    monkeypatch.setattr(pub,"fixed_png_indices",lambda b:dict(zip(("01_first.png","02_quarter.png","03_middle.png","04_three_quarter.png","05_last.png"),(0,0,1,1,2))))
    result=pub.publisher_function()(output,tmp_path/"not_actual_caller","a"*64)
    assert result["GIF"] == 1 and result["PNG"] == 5
    provenance=json.loads((output/"publication_provenance.json").read_text())
    assert provenance["GIF_camera_indices"] == [0,2]
    assert provenance["GIF_total_duration_ms"] == provenance["GIF_encoded_total_duration_ms"] == 300
    assert provenance["learned_control"] is False and provenance["training_data_approved"] is False and provenance["model_promoted"] is False
    assert "좌회전" in (output/"README.md").read_text() and "목적지 정차 교사 주행 v6" not in (output/"README.md").read_text()
    assert len(list(output.glob("*.png"))) == 5 and len(list(output.glob("*.gif"))) == 1
    with pytest.raises(ValueError): pub.publisher_function()(output,tmp_path/"caller","a"*64)


def test_no_actuator_GPU_weight_or_fake_rviz_operations_in_new_publisher():
    source = Path(pub.__file__).read_text()
    for forbidden in ("torch.load(","import torch","Client(","spawn_actor(","apply_control(","load_world(",".fit("):
        assert forbidden not in source
    assert "admission.evidence(authority_path, authority_sha)" in source
    assert "previous.recheck(pins)" in source
