"""HH_261001 - Test prospective frozen identities, deterministic closure, ownership/quotas and honest screen denominators."""

import errno
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import research_temporal_v2_owned_camera_town07_20261001 as owner


def test_contract_separates_candidate_one_precursor_and_unproven_wall_clock_approval():
    value=owner.fixed_contract("v1",True)
    assert value["optimizer_updates"]==3000 and value["candidate_count"]==1
    assert value["source_checkpoint_sha256"]==owner.CHECKPOINT_SHA
    assert value["total_capture_seconds"]==20 and value["policy_deadline_seconds"]==.1
    assert value["longitudinal_pi"] is True and value["automatic_retry"] is False
    assert value["full_RTE01_approved"] is value["vehicle_control_approved"] is False
    assert value["exact_normalized_tensor_archive_retained"] is False
    assert value["sampled_exact_input_quota_bytes"]==128*1024**2
    assert value["output_episode_quota_bytes"]==512*1024**2
    assert value["observed_wall_latency_screen"]["sensor_callback_entry_observed"] is False
    assert value["observed_wall_latency_screen"]["fresh_learned_bundle_delivery_to_ACK_maximum_s"]==.1
    assert value["precursor_motion_screen"]["unintended_stationary_dwell_strictly_below_s"]==1.


@pytest.mark.parametrize("value",["../v1","v0","v1/else","old","v1;touch","",None])
def test_paths_reject_broad_or_unsafe_version_targets(value):
    with pytest.raises(RuntimeError): owner.paths(value)


def test_capture_argv_has_exact_runtime_identity_hard_limits_and_no_map_load():
    argv=owner.capture_argv("v1",True,"a"*64)
    args=owner.live.frozen.parse_args(argv)
    assert args.research_source_sha256=="a"*64 and args.research_longitudinal_pi is True
    assert args.research_weights_sha256==owner.WEIGHTS_SHA and args.research_metadata_sha256==owner.METADATA_SHA
    assert args.max_duration_sec==20 and args.host=="127.0.0.1" and args.port==2440
    assert args.allow_map_load is False and args.wheelbase_m==2.85


def test_import_only_closure_is_deterministic_and_covers_lazy_worker_dependencies(monkeypatch):
    first=owner.runtime_sources()
    import scripts.e2e.carla_physics_substeps
    import scripts.e2e.carla_moving_capture
    second=owner.runtime_sources()
    assert first==second
    for name in ("tools/e2e/carla_physics_substeps.py","tools/e2e/carla_moving_capture.py",
                 "tools/e2e/carla_wall_timing.py","tools/e2e/qualify_carla_moving_common10.py"):
        assert name in first
    for name,sha in owner.FROZEN_GENERIC.items(): assert first[name]==sha
    assert not any(name.startswith("learning/tests/") for name in first)


def test_disk_reserve_covers_episode_cap_without_misusing_sample_cap():
    needed=owner.FLOOR+owner.ARM_RESERVE+owner.SHARED_RESERVE
    assert owner.disk_check(needed)["episode_quota_bytes"]==512*1024**2
    with pytest.raises(RuntimeError): owner.disk_check(needed-1)


def test_original_bind_checks_retry_only_actual_address_in_use():
    now=[0.];calls=[]
    def clock(): return now[0]
    def pause(value): now[0]+=value
    def probe(port):
        calls.append(port)
        if len(calls)<3: raise OSError(errno.EADDRINUSE,"in use")
    value=owner.wait_for_original_bind_release(clock=clock,pause=pause,probe=probe)
    assert calls==[2440]*3 and len(value["bind_failures_retained"])==2
    assert value["SO_REUSE_used"] is False
    def denied(_port): raise OSError(errno.EACCES,"denied")
    with pytest.raises(OSError) as error:
        owner.wait_for_original_bind_release(clock=clock,pause=pause,probe=denied)
    assert error.value.errno==errno.EACCES


def test_bind_release_failure_retained_after_predeclared_natural_limit():
    now=[0.]
    def clock(): return now[0]
    def pause(value): now[0]+=value
    def probe(_port): raise OSError(errno.EADDRINUSE,"still in use")
    with pytest.raises(RuntimeError,match="75s"):
        owner.wait_for_original_bind_release(clock=clock,pause=pause,probe=probe)
    assert now[0]==75.


def test_worker_refuses_missing_or_foreign_owner_without_contacting_simulator(monkeypatch,tmp_path):
    monkeypatch.setattr(owner,"read_json",lambda _path:dict(owner=dict(pid=999999),declaration_sha256="a"*64))
    with pytest.raises(RuntimeError,match="owning parent"):
        owner.assert_owned_worker(dict(output=str(tmp_path)),"a"*64)


def test_episode_quota_fail_closes_and_restores_observation_hook(monkeypatch,tmp_path):
    original=owner.live.frozen.collector.exact_camera_bundle
    monkeypatch.setattr(owner,"directory_bytes",lambda _path:owner.EPISODE_QUOTA)
    with owner.episode_quota(SimpleNamespace(output=tmp_path/"episode")):
        with pytest.raises(RuntimeError,match="quota"):
            owner.live.frozen.collector.exact_camera_bundle({},1)
    assert owner.live.frozen.collector.exact_camera_bundle is original


def test_directory_quota_rejects_symlink_escape(tmp_path):
    other=tmp_path/"elsewhere";other.mkdir()
    episode=tmp_path/"episode";episode.mkdir()
    (episode/"escape").symlink_to(other,target_is_directory=True)
    with pytest.raises(RuntimeError,match="symlink"):
        owner.directory_bytes(episode)


def test_real_asset_archive_supplies_pinned_route_for_independent_native_projection(monkeypatch,tmp_path):
    source_root=tmp_path/"source_workspace";source_root.mkdir()
    storage=source_root/"storage";storage.mkdir()
    route_path=storage/"route.json"
    route=dict(route=[dict(x=0.,y=0.,distance_m=0.,vad_command=3),dict(x=100.,y=0.,distance_m=100.,vad_command=3)])
    owner.write_new(route_path,route)
    rig=storage/"calibration.json";owner.write_new(rig,dict(rig="actual-test-calibration"))
    output=tmp_path/"owned_output";output.mkdir()
    monkeypatch.setattr(owner,"ROOT",source_root)
    monkeypatch.setattr(owner,"ROUTE",route_path)
    pins={str(route_path):owner.digest(route_path),str(rig):owner.digest(rig)}
    owner.archive_sources(output,dict(source_sha256={},external_source_sha256={},asset_sha256=pins))
    archived=output/"source"/route_path.relative_to(source_root)
    assert owner.digest(archived)==pins[str(route_path)]
    assert owner.digest(output/"source"/rig.relative_to(source_root))==pins[str(rig)]
    projector=owner.live.frozen.collector.RouteProjector(owner.read_json(archived))
    first=projector.project(2.,0.,0.)
    last=projector.project(35.,0.,first.progress_m)
    assert last.progress_m-first.progress_m==pytest.approx(33.)


def test_default_contract_mode_has_no_launch_or_declaration_write(monkeypatch,capsys):
    monkeypatch.setattr(owner,"execute",lambda *_args:pytest.fail("physical execution unauthorized"))
    monkeypatch.setattr(owner,"build_plan",lambda *_args:pytest.fail("declaration not requested"))
    assert owner.main([])==0
    assert '"full_RTE01_approved": false' in capsys.readouterr().out


def fixture_report(monkeypatch):
    states=[]
    for index in range(400):
        states.append(dict(frame=100+index,timestamp=10.+index*.05,capture_phase="driving",
            vx=2.,vy=0.,route_progress_m=index*.1,route_cte_m=.02,collision=[],lane_invasion=[],
            world_velocity_carla=[2.,0.,0.],actor_snapshot_transform_carla=dict(x=index*.1,y=-.02),
            control_transport=dict(status="PASS",expected_receipt_sequence=index+1)))
    cameras=[dict(frame=row["frame"],timestamp=row["timestamp"],
        source_timestamps={name:row["timestamp"] for name in owner.live.frozen.collector.MODEL_CAMERA_ORDER}) for row in states[::2]]
    commands=[dict(ack_receipt_sequence=index+1,learned_control_proposed=True,
        held_between_camera_frames=index%2==1,inference_frame=100+index-index%2,command_frame=100+index,
        control_source="learned_trajectory_tracker",application=dict(learned_control_applied=True)) for index in range(400)]
    timing=[dict(ACK_sequence=index+1,learned_control_proposed=True,held_between_camera_frames=False,
        inference_frame=100+index,command_frame=100+index,ACK_server_accepted=True,
        source_checkpoint_sha256=owner.CHECKPOINT_SHA,stream_source_sha256="a"*64,
        exact_bundle_delivery_to_ACK_seconds=.09,exact_bundle_delivery_to_proposal_seconds=.08,
        propose_seconds=.075,ACK_send_seconds=.002,
        observed_bundle=dict(native_frame=100+index,exact_bundle_delivery_monotonic_ns=int(20e9+index*50e6),
                             source_image_timestamp_simulation_ns=int(10e9+index*50e6))) for index in range(0,400,2)]
    ledger={"states.jsonl":states,"camera_frames.jsonl":cameras,"learned_control.jsonl":commands,"wall_timing.jsonl":timing,
            "control_receipts.jsonl":[dict(status="ACKNOWLEDGED")]*402}
    route=dict(route=[dict(x=0.,y=0.,distance_m=0.,vad_command=3),dict(x=200.,y=0.,distance_m=200.,vad_command=3)])
    monkeypatch.setattr(owner,"read_json",lambda path:dict(status="complete",coordinate_contract=dict(wheelbase_m=2.85))
                        if path.name=="manifest.json" else route)
    monkeypatch.setattr(owner,"digest",lambda _path:owner.ASSETS[owner.ROUTE])
    monkeypatch.setattr(Path,"is_file",lambda _path:True)
    monkeypatch.setattr(owner,"jsonl",lambda path:ledger[path.name])
    monkeypatch.setattr(owner,"directory_bytes",lambda _path:1)
    monkeypatch.setattr(owner.independent_audit,"analyze_transport",lambda *_a,**_kw:dict(status="PASS"))
    monkeypatch.setattr(owner.independent_audit,"analyze_camera_alignment",lambda *_a,**_kw:dict(status="PASS"))
    result=dict(cleanup_errors=[],source_hashes_match=True,external_source_hashes_match=True,declaration_sha256="a"*64)
    return ledger,lambda:owner.diagnostic_report(Path("owned_output"),owner.fixed_contract("v1",True),result)


def test_motion_and_latency_report_distinguish_precursor_from_full_feature(monkeypatch):
    ledger,report=fixture_report(monkeypatch)
    value=report()
    assert value["predeclared_motion_core_screen_passed_before_body_swept_clearance"] is True
    assert value["complete_fresh_learned_wall_receipt_denominator"] is True
    assert value["predeclared_observed_bundle_to_ACK_wall_latency_screen_passed"] is True
    assert value["full_RTE01_approved"] is value["learned_driving_screen_passed"] is False
    assert value["ten_hz_end_to_end_proven"] is False


@pytest.mark.parametrize("fault",["missing_timing","duplicate_timing","nonlearned_tick","missing_effect","audit_fail","one_second_stop","non_driving_tick"])
def test_reports_never_excise_incomplete_receipts_emergencies_or_stationary_failure(monkeypatch,fault):
    ledger,report=fixture_report(monkeypatch)
    if fault=="missing_timing": ledger["wall_timing.jsonl"].pop()
    elif fault=="duplicate_timing": ledger["wall_timing.jsonl"].append(ledger["wall_timing.jsonl"][-1])
    elif fault=="nonlearned_tick":
        ledger["learned_control.jsonl"][250]["application"]["learned_control_applied"]=False
        ledger["learned_control.jsonl"][250]["control_source"]="emergency_brake"
    elif fault=="missing_effect": ledger["states.jsonl"][250]["control_transport"]={}
    elif fault=="audit_fail": monkeypatch.setattr(owner.independent_audit,"analyze_transport",lambda *_a,**_kw:dict(status="FAIL"))
    elif fault=="one_second_stop":
        for row in ledger["states.jsonl"][200:220]: row["world_velocity_carla"]=[0.,0.,0.]
    elif fault=="non_driving_tick": ledger["states.jsonl"][250]["capture_phase"]="stationary_tail"
    value=report()
    if fault in ("missing_timing","duplicate_timing","audit_fail"):
        assert value["predeclared_observed_bundle_to_ACK_wall_latency_screen_passed"] is False
    if fault in ("nonlearned_tick","missing_effect","audit_fail","one_second_stop","non_driving_tick"):
        assert value["predeclared_motion_core_screen_passed_before_body_swept_clearance"] is False
        assert value["post_handoff_native_effect_interval_count"]==400
