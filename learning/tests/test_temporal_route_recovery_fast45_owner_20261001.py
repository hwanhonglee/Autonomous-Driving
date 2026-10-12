"""HH_261001 - Check strict V3 endpoint ownership and byte-preserving post-ACK recording path aliases."""

from contextlib import contextmanager
from pathlib import Path
from unittest import mock

import pytest

from scripts.e2e import research_temporal_route_recovery_fast45_town07_20261001 as owner


class Picture:
    format, size = "JPEG", (640,360)
    def __enter__(self): return self
    def __exit__(self,*args): pass
    def load(self): pass


def fixture(output, *, recorded_alias="episode.partial", count=450):
    cameras, recordings, timings = [], [], []
    names = owner.live.frozen.collector.MODEL_CAMERA_ORDER
    for frame in range(1,count+1):
        images = {name: str(Path("images")/name/("%08d.jpg"%frame)) for name in names}
        cameras.append(dict(frame=frame,images=images))
        recordings.append(dict(frame=frame,enqueued_camera_count=6,written_camera_count=6,all_queued_images_written=True,
            failure=None,acknowledged_before_recording=True,ACK_receipt_sequence=frame,
            recording_started_monotonic_ns=20,recording_finished_monotonic_ns=30,
            written_files=[str(output/recorded_alias/images[name]) for name in names]))
        timings.append(dict(ACK_sequence=frame,ACK_server_accepted=True,command_frame=frame,ACK_finished_monotonic_ns=10))
    return dict(camera_frames=cameras,recording_after_ack=recordings,wall_timing=timings)


@contextmanager
def fake_sources(rows, *, digest=None, picture=None):
    with mock.patch.object(owner.previous,"jsonl",side_effect=lambda path: rows[path.stem]), \
        mock.patch.object(owner,"digest",side_effect=digest or (lambda path:"a"*64)), \
        mock.patch.object(owner.Image,"open",side_effect=picture or (lambda path:Picture())):
        yield


@pytest.mark.parametrize("actual_alias",["episode.partial","episode"])
@pytest.mark.parametrize("recorded_alias",["episode.partial","episode"])
def test_exact_recorded_partial_final_aliases_preserve_literal_order_after_rename(tmp_path,actual_alias,recorded_alias):
    output = tmp_path/"run"; (output/actual_alias).mkdir(parents=True)
    rows = fixture(output,recorded_alias=recorded_alias)
    before = [tuple(row["written_files"]) for row in rows["recording_after_ack"]]
    with fake_sources(rows): result = owner.recording_report(output)
    assert result["status"] == "PASS" and result["image_count"] == 2700
    assert result["partial_to_final_rename_alias_checked_without_rewriting_receipts"]
    assert [tuple(row["written_files"]) for row in rows["recording_after_ack"]] == before


@pytest.mark.parametrize("mutation",["outside","dotdot","order","camera_name","absolute_relative","wrong_ack_frame",
    "failed_ack","before_ack","unfinished_time","duplicate_camera","duplicate_ACK","partial_write","write_failure"])
def test_recording_alias_and_ACK_denominator_fail_closed(tmp_path,mutation):
    output = tmp_path/"run"; (output/"episode.partial").mkdir(parents=True)
    rows = fixture(output)
    record, camera, ack = rows["recording_after_ack"][0],rows["camera_frames"][0],rows["wall_timing"][0]
    first = owner.live.frozen.collector.MODEL_CAMERA_ORDER[0]
    if mutation == "outside": record["written_files"][0] = "/foreign/images/"+first+"/00000001.jpg"
    elif mutation == "dotdot": record["written_files"][0] = str(output/"episode.partial"/".."/"episode.partial"/camera["images"][first])
    elif mutation == "order": record["written_files"].reverse()
    elif mutation == "camera_name": camera["images"][first] = "images/NOT_THE_CAMERA/00000001.jpg"
    elif mutation == "absolute_relative": camera["images"][first] = "/tmp/00000001.jpg"
    elif mutation == "wrong_ack_frame": ack["command_frame"] = 2
    elif mutation == "failed_ack": ack["ACK_server_accepted"] = False
    elif mutation == "before_ack": record["recording_started_monotonic_ns"] = 9
    elif mutation == "unfinished_time": record["recording_finished_monotonic_ns"] = 19
    elif mutation == "duplicate_camera": rows["recording_after_ack"][1]["frame"] = 1
    elif mutation == "duplicate_ACK": rows["wall_timing"][1]["ACK_sequence"] = 1
    elif mutation == "partial_write": record["written_camera_count"] = 5
    elif mutation == "write_failure": record["failure"] = "OSError"
    with fake_sources(rows),pytest.raises((RuntimeError,KeyError)): owner.recording_report(output)


def test_any_raw_record_or_actual_JPEG_source_change_is_rejected(tmp_path):
    output = tmp_path/"run"; (output/"episode.partial").mkdir(parents=True)
    rows = fixture(output)
    counts = {}
    def changed(path):
        name = str(path); counts[name] = counts.get(name,0)+1
        return "a"*64 if counts[name] == 1 else "b"*64
    with fake_sources(rows,digest=changed),pytest.raises(RuntimeError,match="sources changed"):
        owner.recording_report(output)


def test_parent_symlink_alias_is_not_a_real_owned_image(tmp_path):
    output = tmp_path/"run"; episode = output/"episode.partial"; episode.mkdir(parents=True)
    foreign = tmp_path/"foreign"; foreign.mkdir()
    (episode/"images").symlink_to(foreign,target_is_directory=True)
    rows = fixture(output)
    with fake_sources(rows),pytest.raises(RuntimeError,match="canonical"):
        owner.recording_report(output)


@pytest.mark.parametrize("count",[0,449,451])
def test_all_450_bundles_required_not_any_nonempty_subset(tmp_path,count):
    output = tmp_path/"run"; (output/"episode.partial").mkdir(parents=True)
    rows = fixture(output,count=count)
    with fake_sources(rows),pytest.raises(RuntimeError,match="all450"):
        owner.recording_report(output)


@pytest.mark.parametrize("failure",[False,True])
def test_export_binding_restores_all_base_and_parent_globals_on_every_exit(tmp_path,failure):
    base_names = ("WEIGHTS","METADATA","WEIGHTS_SHA","METADATA_SHA","CHECKPOINT_SHA","ASSETS","worker")
    parent_names = ("batch_paths","fixed_contract","runtime_sources","load_model","live","MODULE","SCHEMA","diagnostic_report","CHECKPOINT_SHA","ASSETS")
    saved_base = {name:getattr(owner.previous.base,name) for name in base_names}
    saved_parent = {name:getattr(owner.previous,name) for name in parent_names}
    description = dict(schema=owner.EXPORT_SCHEMA,optimizer_updates=3000,npz_sha256="a"*64,checkpoint_sha256="c"*64)
    with mock.patch.object(owner,"read_json",return_value=description),mock.patch.object(owner,"digest",return_value="a"*64):
        try:
            with owner.bound_fixed_recovery_exports():
                assert owner._exports and owner._active
                assert owner.previous.base.WEIGHTS == owner.WEIGHTS
                assert owner.previous.CHECKPOINT_SHA == "c"*64
                with pytest.raises(RuntimeError,match="nest"):
                    with owner.bound_fixed_recovery_exports(): pass
                if failure: raise OSError("preflight failed")
        except OSError:
            assert failure
    assert all(getattr(owner.previous.base,name) is value for name,value in saved_base.items())
    assert all(getattr(owner.previous,name) is value for name,value in saved_parent.items())
    assert owner._exports is None and owner._active is False


@pytest.mark.parametrize("change",["schema","update","weights"])
def test_wrong_export_endpoint_fails_before_shared_bindings(tmp_path,change):
    description = dict(schema=owner.EXPORT_SCHEMA,optimizer_updates=3000,npz_sha256="a"*64,checkpoint_sha256="c"*64)
    if change == "schema": description["schema"] = "wrong"
    elif change == "update": description["optimizer_updates"] = 2999
    else: description["npz_sha256"] = "b"*64
    before = owner.previous.base.WEIGHTS
    with mock.patch.object(owner,"read_json",return_value=description),mock.patch.object(owner,"digest",return_value="a"*64),pytest.raises(RuntimeError):
        with owner.bound_fixed_recovery_exports(): pass
    assert owner.previous.base.WEIGHTS is before and owner._exports is None and not owner._active


@pytest.mark.parametrize("failure",[False,True])
def test_private_worker_authorizes_only_after_parent_plan_and_socket_checks_then_clears(failure):
    # HH_261001 - A numeric preflight is not a control grant; only the checked private worker owns its actual plan.
    expected, plan, calls = "a"*64, object(), []
    def verify(run_id,sha):
        assert owner._owner_plan_sha is None
        calls.append(("plan",run_id,sha)); return plan
    def parent(actual,sha):
        assert owner._owner_plan_sha is None and actual is plan
        calls.append(("parent",sha))
    def worker(run_id,sha):
        assert owner._owner_plan_sha == expected
        calls.append(("worker",run_id,sha))
        if failure: raise OSError("owned capture failed")
        return 7
    with mock.patch.object(owner,"_active",True),mock.patch.object(owner,"_exports",{}), \
        mock.patch.object(owner.previous.base,"verify_plan",side_effect=verify), \
        mock.patch.object(owner.previous.base,"assert_owned_worker",side_effect=parent), \
        mock.patch.object(owner,"_frozen_worker",side_effect=worker):
        if failure:
            with pytest.raises(OSError): owner.private_worker("v1_trial_00",expected)
        else: assert owner.private_worker("v1_trial_00",expected) == 7
    assert owner._owner_plan_sha is None
    assert [row[0] for row in calls] == ["plan","parent","worker"]


def test_failed_parent_check_cannot_grant_simulator_actuation():
    # HH_261001 - Refuse an unowned or collided simulator before setting the actuation capability.
    with mock.patch.object(owner,"_active",True),mock.patch.object(owner,"_exports",{}), \
        mock.patch.object(owner.previous.base,"verify_plan",return_value={}), \
        mock.patch.object(owner.previous.base,"assert_owned_worker",side_effect=RuntimeError("foreign owner")), \
        mock.patch.object(owner,"_frozen_worker") as worker:
        with pytest.raises(RuntimeError,match="foreign owner"):
            owner.private_worker("v1_trial_00","a"*64)
        worker.assert_not_called()
    assert owner._owner_plan_sha is None


@pytest.mark.parametrize("actual_plan_sha",[None,"b"*64])
def test_model_binding_is_offline_except_inside_actual_owned_worker(actual_plan_sha):
    # HH_261001 - Approval flags in the training export stay false even during an explicitly owned simulator test.
    import portable_e2e.temporal_route_recovery_export_loader_research_20261001 as numeric
    captured = {}
    def loader(*args,**kwargs): captured.update(kwargs["research_owner_binding"]); return object()
    with mock.patch.object(owner,"_exports",dict(weights_sha256="a"*64,metadata_sha256="c"*64)), \
        mock.patch.object(owner,"_owner_plan_sha",actual_plan_sha), \
        mock.patch.object(owner,"read_json",return_value=dict(checkpoint_sha256="d"*64,declaration_sha256="e"*64)), \
        mock.patch.object(numeric,"load_route_recovery_research_npz",side_effect=loader):
        owner.load_model(owner.WEIGHTS,owner.METADATA,weights_sha256="a"*64,metadata_sha256="c"*64)
    assert captured["simulator_control_authorized"] is (actual_plan_sha is not None)
    assert captured["purpose"] == ("owned_carla_research_only" if actual_plan_sha else "offline_cpu_numeric_validation_only")
    assert captured["owner_plan_sha256"] == actual_plan_sha
    assert captured["simulation_only"] and captured["vehicle_control_approved"] is False and captured["model_promoted"] is False


def test_incomplete_parent_diagnostic_retains_failure_without_missing_key_exception(tmp_path):
    # HH_261001 - A failed raw diagnostic cannot turn into success or lose its original evidence through a secondary exception.
    failed = dict(status="FAIL_DIAGNOSTIC",error="original body evidence missing",ownership_passed=False)
    with mock.patch.object(owner,"_diagnostic",return_value=failed):
        result = owner.diagnostic_report(tmp_path,{}, {})
    assert result["status"] == "FAIL_BASIC_DRIVING_PRECURSOR"
    assert result["error"] == "original body evidence missing"
    assert result["complete_post_ACK_recording"]["status"] == "NOT_PROVEN"
    assert result["learned_driving_screen_passed"] is False
    assert result["predeclared_observed_bundle_to_ACK_wall_latency_screen_passed"] is False
