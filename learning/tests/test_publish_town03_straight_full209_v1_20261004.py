"""HH_261004 - Check pure full209 publication sampling, pins and truthful timing without simulator/model imports."""

import copy
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

from scripts.e2e import publish_temporal_route_joint_town03_straight_full209_v1_20261004 as m


def bundle(n):
    states = [dict(frame=100+index,timestamp=5.+index*.05) for index in range(n)]
    cameras = [dict(frame=row["frame"],timestamp=row["timestamp"]) for row in states[::2]]
    return dict(states=states,cameras=cameras)


def test_publisher_source_is_pure_and_reviewed():
    assert m.prior.digest(Path(m.previous.__file__)) == m.PREVIOUS_SHA
    assert "torch" not in sys.modules and "carla" not in sys.modules
    sources = m.publication_sources()
    assert Path(m.__file__).resolve() in sources
    assert not any("research_temporal_route_joint_town03_straight_full209_v1" in str(path) for path in sources)


@pytest.mark.parametrize("count",[1,2,12,450,878,1100])
def test_prospective_fixed_bins_and_quarter_camera_selection(count):
    indices = m.camera_indices(count)
    assert indices == sorted(set(range(0,count,5))|{count-1})
    assert indices[0] == 0 and indices[-1] == count-1
    assert m.png_indices(count) == {name:(count-1)*number//4 for name,number in m.PNG_NUMERATORS.items()}
    assert len(indices) == len(set(indices))


@pytest.mark.parametrize("bad",[0,1101,True,1.5,"450"])
def test_camera_counts_fail_closed(bad):
    with pytest.raises(RuntimeError): m.camera_indices(bad)
    with pytest.raises(RuntimeError): m.png_indices(bad)


@pytest.mark.parametrize("n",[2,35,36,1755,2199,2200])
def test_actual_timestamp_bins_and_nominal_last_hold_are_not_invented_native_observations(n):
    value = bundle(n)
    durations = m.playback(value,m.camera_indices(len(value["cameras"])))
    assert durations[-1] == 100
    assert sum(durations) == len(value["cameras"])*100
    native_bin_ms = n*50
    assert sum(durations)-native_bin_ms == (50 if n%2 else 0)
    assert value["cameras"][-1]["frame"] <= value["states"][-1]["frame"]


def test_playback_denies_cropping_and_noncausal_timestamp_changes():
    value = bundle(2200)
    with pytest.raises(RuntimeError): m.playback(value,[0,100,1099])
    value["cameras"][5]["timestamp"] += .01
    with pytest.raises(RuntimeError): m.playback(value,m.camera_indices(1100))


def selection():
    return dict(gallery=str(m.OUTPUT.relative_to(m.ROOT)),
        GIF_camera_indices="every5th actual camera bundle starting0 plus exact last; unique sorted",
        GIF_playback="all selected timestamp intervals plus original final0.1s camera cadence",
        PNG_camera_indices="floor((actual_camera_count-1)*num/4), num=0,1,2,3,4; unique sorted",
        complete_actual_interval_including_failures_required=True,outcome_or_failure_dependent_frame_selection=False,
        actual_Autoware_or_RViz_screenshot_claim=False,illustrative_or_generated_image_allowed=False)


class FakePins:
    # HH_261004 - These CPU fixtures exercise schema/reference checks only; they are never actual media or experiment evidence.
    def __init__(self,documents):
        self.documents,self.files = documents,[]
    def file(self,path,sha): self.files.append((Path(path),sha))
    def json(self,path): return self.documents[Path(path)]


def policy_values():
    plan = dict(prospective_visual_publication=selection())
    plan_path = m.RUN.with_name(m.RUN.name+"_plan.json")
    policy = dict(schema=m.POLICY_SCHEMA,created_before_actual_episode=True,actual_episode_observed=False,
        run_root=str(m.RUN),output=str(m.OUTPUT),attempts=1,automatic_retry=False,
        media_selection=copy.deepcopy(plan["prospective_visual_publication"]),
        run_declaration=dict(path=str(plan_path),sha256="a"*64),known_DEV_site_reused=True,independent_validation=False,
        full_RTE01_approved=False,full_MAIN9_completion_claimed=False,training_data_approved=False,
        vehicle_control_approved=False,model_promoted=False)
    caller = dict(prospective_publication_policy=dict(path=str(m.POLICY_PATH),sha256="b"*64),
        run_files_sha256={"declaration.json":"a"*64})
    pins = FakePins({m.POLICY_PATH:policy,plan_path:plan})
    return caller,pins,plan,policy


def test_pre_capture_selection_policy_is_bound_to_executed_owner_declaration():
    caller,pins,plan,policy = policy_values()
    assert m.verify_policy(caller,pins,plan) is policy
    assert pins.files == [(m.POLICY_PATH,"b"*64),(m.RUN.with_name(m.RUN.name+"_plan.json"),"a"*64)]


@pytest.mark.parametrize("change",["schema","late","observed","run","output","retry","selection","GIF",
    "PNG","hold","crop","adapt","UI","imagegen","declaration","approval","independent"])
def test_prospective_policy_mismatches_are_denied(change):
    caller,pins,plan,policy = policy_values()
    if change == "schema": policy["schema"] = "legacy"
    elif change == "late": policy["created_before_actual_episode"] = False
    elif change == "observed": policy["actual_episode_observed"] = True
    elif change == "run": policy["run_root"] += "_other"
    elif change == "output": policy["output"] += "_other"
    elif change == "retry": policy["automatic_retry"] = True
    elif change == "selection": policy["media_selection"]["gallery"] += "_other"
    elif change == "GIF": policy["media_selection"]["GIF_camera_indices"] = "best scenes"
    elif change == "PNG": policy["media_selection"]["PNG_camera_indices"] = "near success"
    elif change == "hold": policy["media_selection"]["GIF_playback"] = "last native plus50ms"
    elif change == "crop": policy["media_selection"]["complete_actual_interval_including_failures_required"] = False
    elif change == "adapt": policy["media_selection"]["outcome_or_failure_dependent_frame_selection"] = True
    elif change == "UI": policy["media_selection"]["actual_Autoware_or_RViz_screenshot_claim"] = True
    elif change == "imagegen": policy["media_selection"]["illustrative_or_generated_image_allowed"] = True
    elif change == "declaration": caller["run_files_sha256"]["declaration.json"] = "c"*64
    elif change == "approval": policy["full_RTE01_approved"] = True
    elif change == "independent": policy["independent_validation"] = True
    with pytest.raises(RuntimeError): m.verify_policy(caller,pins,plan)


def test_readable_status_differentiates_camera_hold_native_span_and_no_STOP_approval():
    value = bundle(35)
    value["diagnostic"] = dict(status="FAIL_KNOWN_DEV_FULL209_RESEARCH",original_capture_error="preserved original failure")
    text = m.markdown(value,m.png_indices(len(value["cameras"])))
    assert "메인0/9·하위0/30" in text and "새 카메라·차량 상태·제어 관측이 아닙니다" in text
    assert "100ms" in text and "1.70초" in text and "1.80초" in text
    assert "preserved original failure" in text and "Autoware/RViz 화면 캡처가 아닙니다" in text


def test_validate_only_never_creates_media_or_runs_inference(tmp_path,monkeypatch):
    value = bundle(35)
    monkeypatch.setattr(m,"OUTPUT",tmp_path/"uncreated_gallery")
    seen = []
    monkeypatch.setattr(m,"load_archive",lambda *a,**kw:(seen.append(kw) or ({},value,object())))
    result = m.publish(SimpleNamespace(caller_pins=str(tmp_path/"caller.json"),caller_pins_sha256="a"*64,validate_only=True))
    assert result["validated"] is True and result["rendered"] is result["output_created"] is False
    assert seen == [{"decode":False}] and not m.OUTPUT.exists()


def test_create_only_media_revision_denies_overwrite(tmp_path,monkeypatch):
    output = tmp_path/"owned_gallery"
    output.mkdir()
    monkeypatch.setattr(m,"OUTPUT",output)
    monkeypatch.setattr(m,"load_archive",lambda *a,**kw:({},bundle(35),object()))
    with pytest.raises(RuntimeError):
        m.publish(SimpleNamespace(caller_pins=str(tmp_path/"caller.json"),caller_pins_sha256="a"*64,validate_only=True))
    assert list(output.iterdir()) == []


def test_early_archive_cross_run_denied_before_record_decode(tmp_path):
    path = tmp_path/"caller.json"
    payload = json.dumps(dict(schema=m.CALLER_SCHEMA,attempts=1,run_root=str(tmp_path),output=str(m.OUTPUT))).encode()
    path.write_bytes(payload)
    with pytest.raises(RuntimeError):
        m.load_archive(path,hashlib.sha256(payload).hexdigest())
