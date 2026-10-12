"""HH_260906 - Challenge the independent timing auditor with synthetic clocks, tampered spans and frozen-source boundaries."""

import copy
import hashlib
import json
from pathlib import Path

import pytest

from scripts.e2e import audit_carla_wall_timing_quality as audit
from scripts.e2e.carla_wall_timing import WallTimingRecorder
from test_carla_wall_timing import Clock, complete_tick, summary


@pytest.fixture
def fixture():
    clock = Clock()
    recorder = WallTimingRecorder(enabled=True, clock_ns=clock, utc_now=lambda: "2026-09-08T00:00:00+00:00")
    for i, phase in enumerate((audit.PHASES[0], audit.PHASES[0], audit.PHASES[1], audit.PHASES[1], audit.PHASES[2])):
        complete_tick(recorder, clock, i, phase=phase)
        clock.advance(1_000_000)
    rows = recorder.records
    states = [{"frame": r["frame"], "timestamp": r["sim_timestamp"], "capture_phase": r["phase"]} for r in rows]
    cameras = states[::2]
    reported = summary(recorder)
    digest = hashlib.sha256("".join(json.dumps(row, sort_keys=True) + "\n" for row in rows).encode()).hexdigest()
    reported.update(journal_sha256=digest, memory_records_sha256=digest, journal_exactly_matches_memory=True)
    declared = {"status": reported["status"], "journal_sha256": digest, "journal_exactly_matches_memory": True,
        "dataset_admission": False, **{key: reported[key] for key in ("attempt_count", "native_wall_observation_hz",
        "camera_bundle_wall_completion_hz", "simulation_seconds_per_wall_second")}}
    return rows, states, cameras, reported, declared


def test_complete_independent_metrics_equal_measured_summary(fixture):
    rows, states, cameras, reported, declared = fixture
    independent = audit.analyze_timing(rows, states, cameras)
    assert independent["status"] == "PASS"
    result = audit.validate_recorded_summary(reported, independent, declared, journal_sha=reported["journal_sha256"], capture_succeeded=True)
    assert result["status"] == "PASS" and result["persistence_durations_independently_reconstructed"] is False
    assert independent["metrics"]["stages"]["jpeg_encode_write"]["count"] == 3
    assert independent["metrics"]["phase_counts"]["stationary_tail"]["states"] == 1


@pytest.mark.parametrize("mutation", ["duration", "overlap", "unattributed", "child_order", "missing_child", "child_outside",
    "fake_state_frame", "fake_camera_frame", "fake_timestamp", "wrong_phase", "camera_schedule", "complete_with_error",
    "clock_reversal", "stage_not_reached", "missing_marker", "failed_continues"])
def test_corrupted_spans_or_frame_ledgers_never_pass(fixture, mutation):
    rows, states, cameras, _, _ = fixture
    r = rows[0]
    if mutation == "duration": r["stages"]["world_tick_snapshot"]["duration_ns"] += 1
    elif mutation == "overlap": r["stages"]["control_rpc"]["start_ns"] = 0
    elif mutation == "unattributed": r["unattributed_ns"] += 1
    elif mutation == "child_order": r["stages"]["camera_queue_wait"]["cameras"].reverse()
    elif mutation == "missing_child": r["stages"]["jpeg_encode_write"]["cameras"].pop()
    elif mutation == "child_outside": r["stages"]["camera_queue_wait"]["cameras"][0]["end_ns"] += 1_000_000_000
    elif mutation == "fake_state_frame": states[1]["frame"] += 50
    elif mutation == "fake_camera_frame": cameras[0] = {**cameras[0], "frame": 999}
    elif mutation == "fake_timestamp": states[1]["timestamp"] += .1
    elif mutation == "wrong_phase": states[1]["capture_phase"] = "driving"
    elif mutation == "camera_schedule": r["camera_expected"] = False
    elif mutation == "complete_with_error": r["error_type"] = "TimeoutError"
    elif mutation == "clock_reversal": rows[1]["start_ns"] = 0
    elif mutation == "stage_not_reached": r["stages"]["control_rpc"] = dict(start_ns=None,end_ns=None,duration_ns=None,status="NOT_REACHED",cameras=[])
    elif mutation == "missing_marker": r["state_recorded"] = False
    elif mutation == "failed_continues": r["status"] = "PARTIAL_OR_FAILED"
    assert audit.analyze_timing(rows, states, cameras)["status"] == "FAIL"


@pytest.mark.parametrize("field,value", [("start_ns", True), ("start_ns", -1), ("sequence", 2), ("tick_index", True),
    ("camera_recorded", 1), ("sim_timestamp", float("nan")), ("phase", "unknown"), ("instrumentation_errors", {})])
def test_invalid_scalar_or_schema_is_rejected(fixture, field, value):
    rows, states, cameras, _, _ = fixture
    rows[0][field] = value
    with pytest.raises((audit.base.EvidenceError, TypeError, ValueError)):
        audit.analyze_timing(rows, states, cameras)


@pytest.mark.parametrize("mutation", ["metric", "scope", "hash", "capture", "flag", "status", "persistence_nan", "persistence_count", "admission"])
def test_summary_cannot_claim_more_than_raw(fixture, mutation):
    rows, states, cameras, reported, declared = fixture
    independent = audit.analyze_timing(rows, states, cameras)
    if mutation == "metric": reported["total_tick"]["mean_ms"] += 1
    elif mutation == "scope": reported["scope"]["gui_display_fps_measured"] = True
    elif mutation == "hash": reported["memory_records_sha256"] = "0" * 64
    elif mutation == "capture": reported["capture_succeeded"] = False
    elif mutation == "flag": reported["flags"]["camera_schedule_exact"] = False
    elif mutation == "status": declared["status"] = "FAILED_DIAGNOSTIC"
    elif mutation == "persistence_nan": reported["persistence"]["mean_ms"] = float("nan")
    elif mutation == "persistence_count": reported["persistence"]["count"] = len(rows) + 1
    elif mutation == "admission": declared["dataset_admission"] = True
    with pytest.raises(audit.base.EvidenceError):
        audit.validate_recorded_summary(reported, independent, declared, journal_sha=declared["journal_sha256"], capture_succeeded=True)


def test_recovery_retains_failed_diagnostic_even_with_complete_memory_rows(fixture):
    rows, states, cameras, reported, declared = fixture
    independent = audit.analyze_timing(rows, states, cameras)
    original, recovered = "0" * 64, reported["memory_records_sha256"]
    reported.update(journal_sha256=original,journal_exactly_matches_memory=False,recovery_sha256=recovered,status="PARTIAL_OR_FAILED_DIAGNOSTIC")
    declared.update(journal_sha256=original,journal_exactly_matches_memory=False,status="PARTIAL_OR_FAILED_DIAGNOSTIC")
    result = audit.validate_recorded_summary(reported, independent, declared, journal_sha=original,recovery_sha=recovered,capture_succeeded=True)
    assert result["status"] == "FAIL" and result["recovery_used"] is True
    with pytest.raises(audit.base.EvidenceError):
        audit.validate_recorded_summary(reported, independent, declared, journal_sha=original,recovery_sha=None,capture_succeeded=True)


def test_complete_prefix_then_timeout_retains_attempt(fixture):
    clock, recorder = Clock(), None
    recorder = WallTimingRecorder(enabled=True, clock_ns=clock, utc_now=lambda: "2026-09-08T00:00:00+00:00")
    complete_tick(recorder, clock, 0)
    with pytest.raises(TimeoutError):
        with recorder.tick("driving", 1, camera_expected=False) as tick:
            with tick.stage("world_tick_snapshot"):
                clock.advance(1_000_000)
                raise TimeoutError()
    row = recorder.records[0]
    states = [{"frame":row["frame"],"timestamp":row["sim_timestamp"],"capture_phase":"driving"}]
    result = audit.analyze_timing(recorder.records, states, states)
    assert result["status"] == "FAIL" and result["metrics"]["attempt_count"] == 2
    assert result["metrics"]["failed_or_incomplete_attempt_count"] == 1


def test_source_identity_cannot_use_unknown_path_or_commit():
    with pytest.raises(audit.base.EvidenceError): audit.recorded_bytes("main", "scripts/e2e/carla_wall_timing.py")
    with pytest.raises(audit.base.EvidenceError): audit.recorded_bytes(audit.COMMIT, "../secrets")


def test_source_read_disables_lazy_fetch(monkeypatch):
    def run(argv, **kwargs):
        assert "protocol.allow=never" in argv and kwargs["env"]["GIT_NO_LAZY_FETCH"] == "1"
        assert kwargs["env"]["GIT_ALLOW_PROTOCOL"] == "" and kwargs["timeout"] == 10
        return type("Result", (), {"returncode": 1, "stdout": b""})()
    monkeypatch.setattr(audit.subprocess, "run", run)
    with pytest.raises(audit.base.EvidenceError, match="unavailable"):
        audit.recorded_bytes(audit.COMMIT, "scripts/e2e/carla_wall_timing.py")


@pytest.fixture
def plan(monkeypatch):
    value = copy.deepcopy(audit.PLAN_FIXED)
    raw = b"reviewed source"
    value.update(declared_at_utc="2026-09-07T19:50:00+00:00",source_hashes={name:hashlib.sha256(raw).hexdigest() for name in audit.SOURCES},
        capture_flags=["--wall-timing"],comment="HH_260906 - Test",expected_risk="Test",retention="Test")
    monkeypatch.setattr(audit,"recorded_bytes",lambda *_:raw)
    monkeypatch.setattr(audit.v4,"validate_cli",lambda *_:None)
    return value


@pytest.mark.parametrize("mutation", ["extra", "source_missing", "source_extra", "source_wrong", "commit", "quality", "attempts", "timing", "late", "duplicate_flag"])
def test_prospective_plan_fail_closed(plan, mutation):
    if mutation == "extra": plan["allow_more"] = True
    elif mutation == "source_missing": plan["source_hashes"].pop("scripts/e2e/carla_wall_timing.py")
    elif mutation == "source_extra": plan["source_hashes"]["unknown.py"] = "0" * 64
    elif mutation == "source_wrong": plan["source_hashes"]["scripts/e2e/carla_wall_timing.py"] = "0" * 64
    elif mutation == "commit": plan["source_commit"] = "0" * 40
    elif mutation == "quality": plan["ordered_arms"][0]["quality"] = "Epic"
    elif mutation == "attempts": plan["maximum_attempts_per_quality"] = True
    elif mutation == "timing": plan["wall_timing"] = False
    elif mutation == "late": plan["declared_at_utc"] = "2026-09-09T00:00:00Z"
    elif mutation == "duplicate_flag": plan["capture_flags"].append("--wall-timing")
    with pytest.raises(audit.base.EvidenceError): audit.validate_plan(plan)


def test_missing_both_arms_is_incomplete_not_pass(tmp_path,plan):
    (tmp_path/"pilot_plan.json").write_text(json.dumps(plan))
    report,audits=audit.audit_campaign(tmp_path)
    assert report["status"] == "INCOMPLETE" and report["finalized_attempts"] == 0 and audits == []
    assert len(report["trials"]) == 2


def test_excess_attempt_not_hidden(tmp_path,plan):
    (tmp_path/"pilot_plan.json").write_text(json.dumps(plan))
    (tmp_path/"low/run_002").mkdir(parents=True)
    with pytest.raises(audit.base.EvidenceError,match="excess"):
        audit.audit_campaign(tmp_path)


def test_output_existing_raw_inside_or_symlink_rejected(tmp_path):
    raw=tmp_path/"raw";raw.mkdir()
    with pytest.raises(audit.base.EvidenceError): audit.v4.new_output(raw/"new",[raw])
    with pytest.raises(audit.base.EvidenceError): audit.v4.new_output(raw,[raw])
    link=tmp_path/"link";link.symlink_to(raw,target_is_directory=True)
    with pytest.raises(audit.base.EvidenceError): audit.v4.new_output(link/"new",[raw])


def test_auditor_does_not_import_measured_helper_or_carla():
    import ast
    source=ast.parse(Path(audit.__file__).read_text())
    imports=[node.module or "" for node in ast.walk(source) if isinstance(node,ast.ImportFrom)]
    imports += [alias.name for node in ast.walk(source) if isinstance(node,ast.Import) for alias in node.names]
    assert not any("carla_wall_timing" in name or name == "carla" or "collect_carla_vad_expert" in name for name in imports)


@pytest.fixture
def owned(tmp_path, plan):
    raw=b"reviewed source"
    for name in audit.SOURCES:
        path=tmp_path/"provenance"/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(raw)
    source_hashes=plan["source_hashes"]
    owner={"source_bytes_archived":True,"bounds_source_bytes_archived":True,"wall_timing_enabled":True,
        "wall_timing_source_bytes_archived":True,"wall_timing_schema":"carla.expert_wall_timing.v1",
        "source_sha256":source_hashes,"source_head_commit":audit.COMMIT,"host":"127.0.0.1","map":"Town07",
        "quality":"Low","capture_mode":"expert","worker_path":"scripts/e2e/collect_carla_vad_expert.py",
        "route_sha256":audit.v4.ROUTE_SHA,"learned_model_control":False,"vehicle_control_approved":False,
        "server_extra_options":["-RenderOffScreen","-nosound"],"collector_wall_timeout_sec":900,
        "finish_before_utc":plan["finish_before_utc"],"route_path":"route.json","port":2100,
        "collector_argv":[str(tmp_path/"episode"),"route.json","--host","127.0.0.1","--port","2100",*plan["capture_flags"]],
        "planned_at_utc":"2026-09-07T20:00:00+00:00"}
    result={"source_bytes_unchanged_and_archived":True,"source_checks":{name:True for name in audit.SOURCES},
        "capture_mode":"expert","learned_model_control":False,"vehicle_control_approved":False,
        "completed_at_utc":"2026-09-07T20:00:06+00:00","exit_code":0}
    started={"server_pid":42,"server_pgid":42,"port":2100,"map":"Town07","quality":"Low","started_at_utc":"2026-09-07T20:00:02+00:00"}
    (tmp_path/"server.log").write_bytes(b"server log")
    docs={"owner_plan.json":owner,"owner_result.json":result,"owner_started.json":started}
    for stage,sec in (("ready",1),("after_capture",4),("stopped",5)):
        docs["lifecycle/"+stage+".json"]={"status":"PASS","stage":stage,"read_only":True,"owner_pid":42,
            "owner_pgid":42,"generation_id":"expert_42","host":"127.0.0.1","port":2100,"expected_map":"Town07",
            "mode":"stopped" if stage=="stopped" else "running","port_released":stage=="stopped",
            "owner_process_state":None,"active_map_basename":"Town07","server_log":{"size_bytes":10,
            "sha256":hashlib.sha256(b"server log").hexdigest()},"checked_at":f"2026-09-07T20:00:0{sec}+00:00"}
    def save():
        for name,value in docs.items():
            path=tmp_path/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(value))
    save()
    return tmp_path,plan,docs,save


def test_eleven_sources_verified_without_current_worktree_dependency(owned):
    root,plan,_,_=owned
    owner,result,_,_=audit.verify_sources(root,"Low",plan,{})
    assert len(owner["source_sha256"]) == 11 and result["exit_code"] == 0


@pytest.mark.parametrize("mutation",["archive_corrupt","archive_missing","archive_extra","archive_symlink","postcheck","flag",
    "quality","map","learned","scope","head","argv","timeout","lateplan","start_before_ready","stopped_failed",
    "wrong_pid","port_busy","log_prefix","generation","wrong_arm"])
def test_owner_archive_lifecycle_prospective_proof_fail_closed(owned,mutation):
    root,plan,docs,save=owned
    owner=docs["owner_plan.json"];result=docs["owner_result.json"];stopped=docs["lifecycle/stopped.json"]
    name="scripts/e2e/carla_wall_timing.py"
    quality="Low"
    if mutation=="archive_corrupt": (root/"provenance"/name).write_bytes(b"corrupt")
    elif mutation=="archive_missing": (root/"provenance"/name).unlink()
    elif mutation=="archive_extra": (root/"provenance/extra").write_text("extra")
    elif mutation=="archive_symlink":
        path=root/"provenance"/name;path.unlink();path.symlink_to(root/"server.log")
    elif mutation=="postcheck": result["source_checks"][name]=False
    elif mutation=="flag": owner["wall_timing_source_bytes_archived"]=False
    elif mutation=="quality": owner["quality"]="Epic"
    elif mutation=="map": owner["map"]="Town01"
    elif mutation=="learned": result["learned_model_control"]=True
    elif mutation=="scope": owner["capture_mode"]="actuation"
    elif mutation=="head": owner["source_head_commit"]="main"
    elif mutation=="argv": owner["collector_argv"].append("--allow-map-load")
    elif mutation=="timeout": owner["collector_wall_timeout_sec"]=1800
    elif mutation=="lateplan": plan["declared_at_utc"]="2026-09-07T20:00:04+00:00"
    elif mutation=="start_before_ready": docs["owner_started.json"]["started_at_utc"]="2026-09-07T20:00:00+00:00"
    elif mutation=="stopped_failed": stopped["status"]="FAIL"
    elif mutation=="wrong_pid": stopped["owner_pid"]=43
    elif mutation=="port_busy": stopped["port_released"]=False
    elif mutation=="log_prefix": stopped["server_log"]["sha256"]="0"*64
    elif mutation=="generation": stopped["generation_id"]="expert_41"
    elif mutation=="wrong_arm": quality="Epic"
    save()
    with pytest.raises(audit.base.EvidenceError): audit.verify_sources(root,quality,plan,{})


def test_publication_cannot_write_under_visual_inputs(tmp_path):
    raw,visual=tmp_path/"raw",tmp_path/"visual"
    raw.mkdir();visual.mkdir()
    with pytest.raises(audit.base.EvidenceError,match="raw evidence"):
        audit.publish(raw,visual,visual/"nested")


def test_actual_plot_uses_full_ledger_and_creates_png(tmp_path,fixture):
    pytest.importorskip("matplotlib")
    from PIL import Image
    rows,states,cameras,_,_=fixture
    measured=audit.analyze_timing(rows,states,cameras)
    runs=[(tmp_path/quality,({"quality":quality,"timing_protocol":measured},[],rows)) for quality in ("Low","Epic")]
    output=tmp_path/"plot.png"
    audit.render_timing({},runs,output)
    with Image.open(output) as image:
        image.load()
        assert image.size == (1950,1140)


def test_raw_recheck_rejects_changed_bytes(tmp_path):
    path=tmp_path/"input.json";path.write_bytes(b"first")
    entries=[{"path":"input.json","sha256":hashlib.sha256(b"first").hexdigest(),"size_bytes":5}]
    audit.recheck(tmp_path,entries)
    path.write_bytes(b"other")
    with pytest.raises(audit.base.EvidenceError,match="changed"):
        audit.recheck(tmp_path,entries)
