"""HH_261002 - Synthetic variable-event publication tests; never read the actual Town03 attempt or run NN/CARLA."""
import hashlib
import io
import json
from types import SimpleNamespace

from PIL import Image
import pytest

from scripts.e2e import publish_temporal_route_joint_town03_right_20261002 as p


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(value if isinstance(value, bytes) else json.dumps(value, allow_nan=False).encode())
    return hashlib.sha256(path.read_bytes()).hexdigest()


def control():
    return dict(throttle=.1, brake=0., steer=.01, hand_brake=False, reverse=False, manual_gear_shift=False, gear=1)


def native_records(n=6):
    states, commands, receipts, cameras, geometry = [], [], [], [], []
    for i in range(n + 1):
        receipts.append(dict(sequence=i + 1, before_frame=999 + i, after_ack_frame=999 + i,
            status="ACKNOWLEDGED", server_accepted=True, reason="pre_capture_bootstrap" if i == 0 else "normal",
            requested_control=control()))
        if i:
            source = "expert_startup" if i == 1 else "learned_trajectory_tracker"
            commands.append(dict(ack_receipt_sequence=i + 1, command_frame=999 + i, inference_frame=999 + i,
                control_source=source, held_between_camera_frames=False, learned_control_proposed=i != 1,
                application=dict(acknowledged=True, applied_source="expert" if i == 1 else source, learned_control_applied=i != 1),
                selected=dict(xy_base_m=[[j * .1, 0.] for j in range(64)])))
        if i == n: continue
        frame, timestamp = 1000 + i, i * .05
        states.append(dict(frame=frame, timestamp=timestamp, x=timestamp, y=0., yaw=0., vx=1., vy=0., route_cte_m=0.,
            current_control=control(), capture_phase="driving", control_transport=dict(status="PASS", expected_receipt_sequence=i + 1)))
        geometry.append(dict(frame=frame, body_polygon_carla_xy=[[timestamp - 1., -1.], [timestamp + 1., -1.], [timestamp + 1., 1.], [timestamp - 1., 1.]]))
        if i % 2 == 0:
            cameras.append(dict(frame=frame, timestamp=timestamp, camera_order=list(p.MODEL_CAMERA_ORDER),
                source_timestamps={name: timestamp for name in p.visual.CAMERAS},
                images={name: "images/" + name + f"/{frame}.jpg" for name in p.visual.CAMERAS}))
    return states, cameras, commands, receipts, geometry


@pytest.fixture
def archive(tmp_path, monkeypatch):
    root = tmp_path / "workspace"; run = root / "storage/experiments/validation/2026-10-02/temporal_route_joint_town03_known_dev_right_v2"
    monkeypatch.setattr(p, "ROOT", root); monkeypatch.setattr(p, "OUTPUT", root / "docs/new16")
    monkeypatch.setattr(p, "HELPERS", {}); monkeypatch.setattr(p, "publication_sources", lambda: set())
    episode = run / "episode"; records = native_records(); states, cameras, commands, receipts, geometry = records
    for name, rows in zip(("states.jsonl", "camera_frames.jsonl", "learned_control.jsonl", "control_receipts.jsonl"), records[:4]):
        write(episode / name, b"\n".join(json.dumps(r).encode() for r in rows))
    write(episode / "learned_summary.json", dict(handoff_occurred=True,
        counts=dict(inference_count=5, prefill_count=2, learned_commands_acknowledged=5, emergency_commands=0)))
    write(episode / "manifest.json", dict(status="failed", training_data_approved=False, vehicle_control_approved=False))
    route_sha = write(episode / "route.json", dict(town="Town03", scenario="right", route=[dict(x=float(i), y=0.) for i in range(213)]))
    monkeypatch.setattr(p, "ROUTE_SHA", route_sha)
    picture = io.BytesIO(); Image.new("RGB", (640, 360), (51, 86, 114)).save(picture, format="JPEG")
    for camera in cameras:
        for name in camera["images"].values(): write(episode / name, picture.getvalue())
    plan = dict(schema=p.OWNER_SCHEMA[:-1] + "2", output=str(run), town="Town03", scenario="right", dataset_split="val",
        known_DEV_site_reused=True, independent_validation=False, attempts=1, automatic_retry=False, total_capture_seconds=110,
        optimizer_updates=6000, speed_preview_s=.5, controller_target_cap_mps=2.4, training_data_approved=False,
        full_RTE03_approved=False, source_sha256={}, external_source_sha256={}, asset_sha256={},
        actual_manual_first_gear_bootstrap_and_engagement_required=True,
        owned_green_signal_fixture=dict(original_own_world_traffic_actors="ALL", state="Green", frozen=True,
            ignore_traffic_lights=False, all_original_hazard_detectors_and_emergency_returns_unchanged=True,
            known_v1_hazard_subtype_proven=False, closed_course_basic_turn_only_not_traffic_rule_performance=True))
    plan_sha = write(run / "declaration.json", plan)
    owner = dict(declaration_sha256=plan_sha, status="CAPTURE_RETURNED_NONZERO_RETAINED")
    write(run / "owner_result.json", owner)
    body = dict(all_native_boxes=6, all_snapshot_linear_sweeps=5, lane_inset_m=.1, body_and_swept_clearance_passed=False)
    write(run / "derived_body/summary.json", body)
    write(run / "derived_body/all_native_boxes_and_sweeps.jsonl.gz", p.gzip.compress(b"\n".join(json.dumps(row).encode() for row in geometry)))
    diagnostic = dict(schema=p.OWNER_SCHEMA[:-1] + "2.diagnostic_report", original_owner=owner,
        full_MAIN9_completion_claimed=False, full_RTE03_approved=False, training_data_approved=False,
        vehicle_control_approved=False, model_promoted=False, original_capture_status="failed", independent_body=body,
        actual_native_state_count=6, actual_camera_bundle_count=3, actual_ACK_receipt_count=7,
        learned_driving_screen_passed=False, joint_turn_motion_body_policy_wall100ms_passed=False,
        observed_bundle_to_ACK_wall100ms_passed=False, status="FAIL_KNOWN_DEV_RIGHT_TURN_PRECURSOR",
        raw_source_sha256={str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in episode.rglob("*") if path.is_file()})
    write(run / "diagnostic_report.json", diagnostic)
    caller = dict(schema=p.CALLER_SCHEMA, output=str(p.output_for(run)), attempts=1,
        PNG_index_policy="first_quarter_half_threequarters_exact_last", run_root=str(run), publication_source_sha256={},
        font_sha256={str(path): p.prior.digest(path) for path in [p.Path("/usr/share/fonts/truetype/dejavu") / name for name in ("DejaVuSans.ttf", "DejaVuSans-Bold.ttf")]},
        run_files_sha256={name: p.prior.digest(run / name) for name in p.inventory(run)})
    path = tmp_path / "caller.json"; return path, write(path, caller), caller, run


def absent_preNN_fixture(archive):
    path, _, caller, old_run = archive
    run = old_run.with_name("temporal_route_joint_town03_known_dev_right_v1"); old_run.rename(run)
    episode = run / "episode"; (episode / "learned_control.jsonl").unlink()
    write(episode / "learned_summary.json", dict(handoff_occurred=False,
        counts=dict(inference_count=0, prefill_count=0, learned_commands_acknowledged=0, emergency_commands=0)))
    receipts = [json.loads(line) for line in (episode / "control_receipts.jsonl").read_bytes().splitlines()]
    for row in receipts[1:]: row["reason"] = "next_driving_control"
    write(episode / "control_receipts.jsonl", b"\n".join(json.dumps(row).encode() for row in receipts))
    plan = json.loads((run / "declaration.json").read_bytes()); plan.update(schema=p.OWNER_SCHEMA, output=str(run))
    plan_sha = write(run / "declaration.json", plan)
    owner = json.loads((run / "owner_result.json").read_bytes()); owner["declaration_sha256"] = plan_sha; write(run / "owner_result.json", owner)
    diagnostic = json.loads((run / "diagnostic_report.json").read_bytes())
    diagnostic.update(schema=p.OWNER_SCHEMA + ".diagnostic_report", original_owner=owner,
        raw_source_sha256={str(f): p.prior.digest(f) for f in episode.rglob("*") if f.is_file()})
    write(run / "diagnostic_report.json", diagnostic)
    caller.update(run_root=str(run), output=str(p.output_for(run)),
        run_files_sha256={name: p.prior.digest(run / name) for name in p.inventory(run)})
    return path, write(path, caller), caller, run


def test_full_variable_reader_retains_failure_and_camera_native_denominator(archive):
    _, bundle, pins = p.load_archive(*archive[:2]); pins.recheck()
    assert len(bundle["states"]) == 6 and len(bundle["cameras"]) == 3
    assert bundle["diagnostic"]["joint_turn_motion_body_policy_wall100ms_passed"] is False
    assert bundle["control_census"]["observed_prior_learned_native_effect_count"] == 4
    assert bundle["control_census"]["acknowledged_learned_command_count"] == 5


def test_zero_NN_effect_is_a_setup_outcome_without_fictional_wall_or_recording_receipts():
    states, cameras, commands, receipts, geometry = native_records()
    for command in commands:
        command.update(control_source="expert_startup", learned_control_proposed=False)
        command["application"].update(applied_source="expert", learned_control_applied=False)
        command.pop("selected")
    by_frame = p.verify_records(states, cameras, commands, receipts, geometry)
    bundle = dict(states=states, cameras=cameras, commands=commands, receipts=receipts, by_frame=by_frame,
        diagnostic=dict(joint_turn_motion_body_policy_wall100ms_passed=False))
    bundle["control_census"] = p.control_census(bundle)
    assert bundle["control_census"]["observed_prior_learned_native_effect_count"] == 0
    assert bundle["control_census"]["acknowledged_learned_command_count"] == 0
    assert bundle["control_census"]["fresh_learned_proposal_count"] == 0
    assert "NO observed NN effect" in p.outcome_text(bundle)[0]
    assert "NN 우회전 판정 불가" in p.outcome_text(bundle)[1]
    bundle["diagnostic"].update(original_capture_error="failed8s", diagnostic_error="recording_after_ack missing")
    prose = p.markdown(bundle, p.png_indices(len(cameras)))
    assert "NN 우회전 성능의 실패나 성공으로 세지 않습니다" in prose and "failed8s" in prose
    assert "110초 상한을 채웠다고 주장하지 않습니다" in prose and "recording_after_ack missing" in prose


def test_final_outgoing_NN_ACK_without_later_native_effect_is_not_observed_NN():
    states, cameras, commands, receipts, geometry = native_records()
    for command in commands[:-1]:
        command.update(control_source="expert_startup", learned_control_proposed=False)
        command["application"].update(applied_source="expert", learned_control_applied=False)
    by_frame = p.verify_records(states, cameras, commands, receipts, geometry)
    bundle = dict(states=states, commands=commands, receipts=receipts, by_frame=by_frame)
    census = p.control_census(bundle)
    assert census["observed_prior_learned_native_effect_count"] == 0
    assert census["acknowledged_learned_command_count"] == 1


@pytest.mark.parametrize("kind", ["unaccepted_unused_ACK", "unknown_outgoing_ACK", "unacknowledged_proposal"])
def test_terminal_and_nonobserved_ACK_rows_cannot_escape_direct_binding(kind):
    states, cameras, commands, receipts, geometry = native_records()
    if kind == "unaccepted_unused_ACK": receipts[-1]["server_accepted"] = False
    elif kind == "unknown_outgoing_ACK": commands[-1]["ack_receipt_sequence"] = 500
    else: commands[-1]["application"]["acknowledged"] = False
    with pytest.raises(Exception): p.verify_records(states, cameras, commands, receipts, geometry)


def test_cleanup_ACK_after_final_unrecorded_tick_is_retained_not_fictional_pose():
    states, cameras, commands, receipts, geometry = native_records()
    receipts.append(dict(sequence=8, before_frame=1006, after_ack_frame=1006, status="ACKNOWLEDGED",
        server_accepted=True, reason="cleanup", requested_control=control()))
    by_frame = p.verify_records(states, cameras, commands, receipts, geometry)
    assert len(by_frame) == 6 and 1006 not in by_frame and len(receipts) == 8


def test_optional_root_diagnostic_is_pinned_and_keeps_original_FAIL(archive):
    path, _, caller, run = archive
    audit = p.ROOT / "storage/experiments/diagnostics/2026-10-02/town03_launch/root_read_only_audit.json"
    expected = write(audit, dict(independent_transport="FAIL", native_rates="FAIL", model_handoff_count=0))
    caller["auxiliary_root_audit_sha256"] = {str(audit): expected}
    _, bundle, pins = p.load_archive(path, write(path, caller))
    assert bundle["auxiliary_root_audits"][str(audit)]["independent_transport"] == "FAIL"
    write(audit, {} )
    with pytest.raises(Exception): pins.recheck()


@pytest.mark.parametrize("n", [1, 2, 3, 5, 6, 449, 450, 1100])
def test_indices_include_real_last_without_favorable_cut(n):
    indices = p.camera_indices(n)
    assert indices[0] == 0 and indices[-1] == n - 1 and indices == sorted(set(range(0, n, 5)) | {n - 1})
    assert p.png_indices(n)["05_last.png"] == n - 1


@pytest.mark.parametrize("n", [False, 0, 1101, 5.0])
def test_camera_denominator_is_exact_typed_bounded(n):
    with pytest.raises(Exception): p.camera_indices(n)


@pytest.mark.parametrize("n", [6, 7, 24, 25, 900, 901, 2199, 2200])
def test_GIF_clock_retains_full_actual_native_interval(n):
    bundle = dict(states=[dict(timestamp=i * .05) for i in range(n)], cameras=[dict(timestamp=i * .05) for i in range(0, n, 2)])
    durations = p.playback(bundle, p.camera_indices(len(bundle["cameras"])))
    assert sum(durations) == n * 50 and all(0 < d <= 500 for d in durations)


def test_latest_ANY_emergency_ACK_clears_old_purple_without_same_frame_lookahead():
    states, cameras, commands, receipts, geometry = native_records()
    by_frame = p.verify_records(states, cameras, commands, receipts, geometry)
    command = commands[2]; command["control_source"] = "emergency_brake"
    command["application"].update(applied_source="emergency_brake", learned_control_applied=False)
    assert p.prior.observed_prior_ack(states[2], receipts, commands, by_frame)[2]
    assert p.prior.observed_prior_ack(states[3], receipts, commands, by_frame)[2] == []


@pytest.mark.parametrize("kind", ["camera_missing", "camera_future", "native_missing", "body_missing", "ACK_source", "pedal"])
def test_whole_snapshot_and_actual_prior_ACK_guards(kind):
    states, cameras, commands, receipts, geometry = native_records()
    if kind == "camera_missing": cameras.pop()
    elif kind == "camera_future": cameras[0]["timestamp"] += .1
    elif kind == "native_missing": states.pop()
    elif kind == "body_missing": geometry.pop()
    elif kind == "ACK_source": states[3]["control_transport"]["expected_receipt_sequence"] = 2
    else: states[3]["current_control"]["throttle"] += .01
    with pytest.raises(Exception): p.verify_records(states, cameras, commands, receipts, geometry)


@pytest.mark.parametrize("kind", ["extra", "missing", "symlink", "mutation"])
def test_complete_external_archive_inventory(archive, kind):
    path, sha, caller, run = archive
    if kind == "extra": write(run / "late", b"extra")
    elif kind == "missing": (run / "owner_result.json").unlink()
    elif kind == "symlink": (run / "alias").symlink_to(run / "owner_result.json")
    else: write(run / "owner_result.json", {})
    with pytest.raises(Exception): p.load_archive(path, sha)


def test_first_parsed_payload_and_after_checks_are_not_rehashed_new_truth(archive):
    _, _, pins = p.load_archive(*archive[:2]); write(archive[3] / "diagnostic_report.json", {})
    with pytest.raises(Exception): pins.recheck()


def test_validate_only_creates_no_media_or_output(archive):
    result = p.publish(SimpleNamespace(caller_pins=str(archive[0]), caller_pins_sha256=archive[1], validate_only=True))
    assert result["validated"] and not result["rendered"] and not p.OUTPUT.exists()


def test_complete_synthetic_publication_retains_exact_last_GIF_bin_and_original_FAIL(archive, monkeypatch):
    monkeypatch.setattr(p.shutil, "disk_usage", lambda _: SimpleNamespace(free=9 * 1024**3))
    result = p.publish(SimpleNamespace(caller_pins=str(archive[0]), caller_pins_sha256=archive[1], validate_only=False))
    assert result["GIF_count"] == 1 and result["PNG_count"] == 5 and result["GIF_frames"] == 2
    output = p.output_for(archive[3]); provenance = json.loads((output / "publication_provenance.json").read_bytes())
    assert provenance["original_diagnostic"]["joint_turn_motion_body_policy_wall100ms_passed"] is False
    assert provenance["GIF_camera_indices"] == [0, 2] and provenance["GIF_duration_ms"] == [200, 100]
    assert provenance["GIF_total_duration_ms"] == 300 and provenance["full_110s_capture_claimed"] is False
    assert provenance["failed_tail_cropped"] is False
    assert provenance["displayed_GIF_frames"][-1]["observed_frame"] == 1004
    assert provenance["last_native_frame"] == 1005 and provenance["last_camera_frame"] == 1004


def test_preNN_synthetic_publication_has_no_purple_or_model_failure_claim(archive, monkeypatch):
    path, _, caller, run = archive; episode = run / "episode"
    commands = [json.loads(line) for line in (episode / "learned_control.jsonl").read_bytes().splitlines()]
    for command in commands:
        command.update(control_source="expert_startup", learned_control_proposed=False)
        command["application"].update(applied_source="expert", learned_control_applied=False)
    write(episode / "learned_control.jsonl", b"\n".join(json.dumps(row).encode() for row in commands))
    diagnostic = json.loads((run / "diagnostic_report.json").read_bytes())
    diagnostic.update(original_capture_error="did not reach0.5m/swithin8s", diagnostic_error="recording_after_ack absent",
        raw_source_sha256={str(f): p.prior.digest(f) for f in episode.rglob("*") if f.is_file()})
    write(run / "diagnostic_report.json", diagnostic)
    caller["run_files_sha256"] = {name: p.prior.digest(run / name) for name in p.inventory(run)}
    caller_sha = write(path, caller)
    monkeypatch.setattr(p.shutil, "disk_usage", lambda _: SimpleNamespace(free=9 * 1024**3))
    p.publish(SimpleNamespace(caller_pins=str(path), caller_pins_sha256=caller_sha, validate_only=False))
    output = p.output_for(run); provenance = json.loads((output / "publication_provenance.json").read_bytes())
    assert provenance["control_census"]["observed_prior_learned_native_effect_count"] == 0
    assert all(r["prior_learned_plan_drawn"] is False for r in provenance["displayed_GIF_frames"])
    text = (output / "README.md").read_text()
    assert "NN 우회전 판정 불가" in text and "NN 우회전 성능의 실패나 성공으로 세지 않습니다" in text
    assert "recording_after_ack absent" in text and "110초 상한을 채웠다고 주장하지 않습니다" in text


def test_no_model_or_simulator_process_can_claim_this_publication(monkeypatch, tmp_path):
    monkeypatch.setitem(p.sys.modules, "torch", object())
    with pytest.raises(Exception): p.load_archive(tmp_path / "no_actual", "a" * 64)


def test_existing_user_output_is_not_overwritten(archive):
    output = p.output_for(archive[3]); output.mkdir(parents=True); sentinel = output / "user"; sentinel.write_text("preserve")
    with pytest.raises(Exception): p.publish(SimpleNamespace(caller_pins=str(archive[0]), caller_pins_sha256=archive[1], validate_only=True))
    assert sentinel.read_text() == "preserve"


def test_actually_absent_preNN_journal_and_private_renderer_do_not_fabricate_teacher(archive, monkeypatch):
    path, digest, caller, run = absent_preNN_fixture(archive)
    frozen_selector = p.prior.observed_prior_ack
    _, bundle, _ = p.load_archive(path, digest)
    assert bundle["commands"] == [] and bundle["unlogged_preNN"] is True
    assert bundle["control_census"]["observed_prior_learned_native_effect_count"] == 0
    assert bundle["control_census"]["teacher_startup_command_count"] == 0
    monkeypatch.setattr(p.shutil, "disk_usage", lambda _: SimpleNamespace(free=9 * 1024**3))
    p.publish(SimpleNamespace(caller_pins=str(path), caller_pins_sha256=digest, validate_only=False))
    receipt = json.loads((p.output_for(run) / "publication_provenance.json").read_bytes())
    assert receipt["actual_proposal_journal_absent"] is True
    assert all("PRE-NN original native ACK" in row["observed_source"] for row in receipt["displayed_GIF_frames"])
    assert p.prior.observed_prior_ack is frozen_selector and not (run / "episode/learned_control.jsonl").exists()


@pytest.mark.parametrize("key", ["inference_count", "prefill_count", "learned_commands_acknowledged", "emergency_commands", "handoff", "native_source"])
def test_absent_journal_requires_authenticated_zeroNN_not_missing_positive_records(archive, key):
    path, _, caller, run = absent_preNN_fixture(archive); episode = run / "episode"
    summary = json.loads((episode / "learned_summary.json").read_bytes())
    if key == "handoff": summary["handoff_occurred"] = True
    elif key == "native_source":
        states = [json.loads(line) for line in (episode / "states.jsonl").read_bytes().splitlines()]
        states[-1]["research_control_source"] = "learned_trajectory_tracker"
        write(episode / "states.jsonl", b"\n".join(json.dumps(row).encode() for row in states))
    else: summary["counts"][key] = 1
    write(episode / "learned_summary.json", summary)
    diagnostic = json.loads((run / "diagnostic_report.json").read_bytes())
    diagnostic["raw_source_sha256"] = {str(f): p.prior.digest(f) for f in episode.rglob("*") if f.is_file()}
    write(run / "diagnostic_report.json", diagnostic)
    caller["run_files_sha256"] = {name: p.prior.digest(run / name) for name in p.inventory(run)}
    with pytest.raises(Exception): p.load_archive(path, write(path, caller))


@pytest.mark.parametrize("kind", ["unknown_run", "cross_run_output", "unknown_output", "wrong_schema"])
def test_exact_v1_v2_run_schema_output_mapping_rejects_unknown_and_crossed_roles(archive, kind):
    path, _, caller, run = archive
    if kind == "unknown_run": caller["run_root"] = str(run.with_name(run.name[:-1] + "3"))
    elif kind == "cross_run_output": caller["output"] = str(p.OUTPUT / "v1_pre_nn_launch_failure")
    elif kind == "unknown_output": caller["output"] = str(p.OUTPUT / "best_run")
    else:
        plan = json.loads((run / "declaration.json").read_bytes()); plan["schema"] = p.OWNER_SCHEMA
        write(run / "declaration.json", plan); caller["run_files_sha256"] = {n: p.prior.digest(run / n) for n in p.inventory(run)}
    with pytest.raises(Exception): p.load_archive(path, write(path, caller))
