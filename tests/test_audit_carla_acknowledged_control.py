"""HH_260906 - Independently test frozen receipt/frame evidence without simulator, model or training access."""

import copy
import hashlib
import json
import math
import shutil

import pytest

from scripts.e2e import audit_carla_acknowledged_control as audit


def control(throttle=0.0, brake=0.0, **changes):
    value = dict(throttle=throttle, brake=brake, steer=0.0, hand_brake=False, reverse=False, gear=0, manual_gear_shift=False)
    value.update(changes)
    return value


def raw_row(frame, phase, current, expected):
    return {"frame": frame, "timestamp": frame * .05, "capture_phase": phase,
        "x": float(frame), "y": 0., "z": 0., "yaw": 0., "vx": 1., "vy": 0., "ax": 0., "ay": 0., "yaw_rate": 0.,
        "actor_snapshot_transform_carla": dict(x=frame + 1.425, y=0., z=0., yaw=0., pitch=0., roll=0.),
        "world_velocity_carla": [1., 0., 0.], "world_acceleration_carla": [0., 0., 0.],
        "world_angular_velocity_carla_deg_s": [0., 0., 0.], "current_control": copy.deepcopy(current),
        "control_transport": {"status": "PASS", "expected_receipt_sequence": expected,
            "expected_command_before_frame": frame - 1, "observed_frame": frame,
            "control_read_frame_before": frame, "control_read_frame_after": frame,
            "frame_binding_pass": True, "mismatched_fields": [], "absolute_tolerance": 1e-6,
            "automatic_gear_equality_required": False, "lookback_relabeling_allowed": False,
            "physical_actuation_proven": False}}


@pytest.fixture
def chain():
    receipts, states = [], []
    brake = control(brake=1.)
    def send(frame, reason, requested):
        receipt = dict(sequence=len(receipts) + 1, actor_id=42, response_actor_id=42, before_frame=frame, after_ack_frame=frame,
            mode="acknowledged_batch", do_tick=False, status="ACKNOWLEDGED", server_accepted=True,
            physical_actuation_proven=False, reason=reason, requested_control=copy.deepcopy(requested))
        receipts.append(receipt)
        return receipt["sequence"]
    send(99, "pre_capture_bootstrap", brake)
    bootstrap = raw_row(100, "stationary_warmup", brake, 1)
    bootstrap["alignment"] = bootstrap.pop("control_transport")
    bootstrap["recorded_training_state"] = False
    send(100, "stationary_warmup_start", brake)
    for frame in range(101, 108):
        phase = "stationary_warmup" if frame <= 102 else "driving" if frame <= 105 else "stationary_tail"
        row = raw_row(frame, phase, receipts[-1]["requested_control"], receipts[-1]["sequence"])
        next_control = control(throttle=(frame - 99) * .05) if 103 <= frame < 105 else brake
        row["next_control"] = copy.deepcopy(next_control)
        row["control_transport"]["next_command_receipt_sequence"] = send(frame, "next_" + phase + "_control", next_control)
        if frame == 102:
            row["next_control"] = control(throttle=.15)
            row["control_transport"]["next_command_receipt_sequence"] = send(frame, "initial_drive_control", row["next_control"])
        if frame == 105:
            row["control_transport"]["next_command_receipt_sequence"] = send(frame, "driving_end_brake", brake)
        states.append(row)
    return states, receipts, bootstrap


def test_complete_multiphase_receipts_are_bound_without_importing_worker(chain):
    original = copy.deepcopy(chain)
    report = audit.analyze_transport(*chain)
    assert report["status"] == "PASS" and report["receipt_count"] == 11
    assert report["recorded_state_count"] == 7 and report["bootstrap_observation_count"] == 1
    assert report["maximum_legacy_state_reconstruction_absolute_error"] < 1e-12
    assert report["physical_actuation_proven"] is False
    assert chain == original


@pytest.mark.parametrize("change", ["missing_receipt", "duplicate_sequence", "wrong_actor", "false_tick", "false_response"])
def test_contradictory_receipt_schema_never_produces_a_pass(chain, change):
    _, receipts, _ = chain
    if change == "missing_receipt": receipts.pop(3)
    elif change == "duplicate_sequence": receipts[3]["sequence"] = 3
    elif change == "wrong_actor": receipts[3]["actor_id"] = 43
    elif change == "false_tick": receipts[3]["do_tick"] = True
    else: receipts[3]["server_accepted"] = False
    with pytest.raises(audit.base.EvidenceError):
        audit.analyze_transport(*chain)


@pytest.mark.parametrize("change,flag", [
    (lambda s, r, b: s[3].update(x=999), "all_snapshot_conversions_match"),
    (lambda s, r, b: s[3]["world_acceleration_carla"].__setitem__(1, 2), "all_snapshot_conversions_match"),
    (lambda s, r, b: r[3].update(after_ack_frame=105), "command_frame_consistency"),
    (lambda s, r, b: s[3]["control_transport"].update(control_read_frame_after=105), "control_frame_binding"),
    (lambda s, r, b: s[3]["control_transport"].update(expected_receipt_sequence=1), "control_frame_binding"),
    (lambda s, r, b: s[3]["current_control"].update(throttle=.05), "observed_controls_match_prior_ack"),
    (lambda s, r, b: s[3]["next_control"].update(steer=.3), "next_command_links"),
    (lambda s, r, b: r[3].update(reason="driving_end_brake"), "phase_command_ledger"),
    (lambda s, r, b: s[3].update(timestamp=5.5), "twenty_hz_frame_chain"),
])
def test_realistic_evidence_disagreements_are_retained_as_failed_diagnostics(chain, change, flag):
    change(*chain)
    report = audit.analyze_transport(*chain)
    assert report["status"] == "FAIL" and flag in report["failed_flags"]


def test_two_row_control_match_is_not_a_permitted_relabeling(chain):
    states, _, _ = chain
    states[3]["current_control"] = copy.deepcopy(states[1]["next_control"])
    report = audit.analyze_transport(*chain)
    assert report["control_mismatch_count"] == 1
    assert "observed_controls_match_prior_ack" in report["failed_flags"]


def test_failed_final_abort_receipt_stays_in_denominator(chain):
    states, receipts, _ = chain
    receipts.append(dict(sequence=12, actor_id=42, mode="acknowledged_batch", do_tick=False, status="FAILED",
        server_accepted=False, physical_actuation_proven=False, reason="exception_cleanup_abort", before_frame=107,
        requested_control=control(brake=1.), error="transport timeout"))
    report = audit.analyze_transport(*chain)
    assert report["receipt_count"] == 12 and report["failed_receipt_count"] == 1
    assert report["status"] == "FAIL"


def test_hidden_unrecorded_tick_before_cleanup_is_not_accepted(chain):
    _, receipts, _ = chain
    receipt = copy.deepcopy(receipts[-1])
    receipt.update(sequence=12, before_frame=110, after_ack_frame=110, reason="exception_cleanup_abort")
    receipts.append(receipt)
    assert "command_frame_consistency" in audit.analyze_transport(*chain)["failed_flags"]


def test_same_aggregate_counts_cannot_hide_duplicate_and_missing_per_frame_commands(chain):
    states, receipts, bootstrap = chain
    moved = receipts.pop(7)
    moved.update(before_frame=104, after_ack_frame=104)
    receipts.insert(6, moved)
    remap = {receipt["sequence"]: index for index, receipt in enumerate(receipts, 1)}
    for index, receipt in enumerate(receipts, 1):
        receipt["sequence"] = index
    for row in states:
        for key in ("expected_receipt_sequence", "next_command_receipt_sequence"):
            row["control_transport"][key] = remap[row["control_transport"][key]]
    bootstrap["alignment"]["expected_receipt_sequence"] = remap[bootstrap["alignment"]["expected_receipt_sequence"]]
    report = audit.analyze_transport(*chain)
    assert report["status"] == "FAIL" and "phase_command_ledger" in report["failed_flags"]


@pytest.mark.parametrize("key,value", [("expected_command_before_frame", -999), ("automatic_gear_equality_required", True)])
def test_contradictory_observation_contract_metadata_cannot_pass(chain, key, value):
    chain[0][0]["control_transport"][key] = value
    assert "worker_alignment_claims_match" in audit.analyze_transport(*chain)["failed_flags"]


def test_automatic_gear_is_not_compared_but_manual_gear_and_flags_are():
    assert audit.mismatched_controls(control(gear=1), control(gear=0)) == []
    assert audit.mismatched_controls(control(gear=1, manual_gear_shift=True), control(gear=2, manual_gear_shift=True)) == ["gear"]
    assert audit.mismatched_controls(control(reverse=True), control()) == ["reverse"]
    with pytest.raises(audit.base.EvidenceError):
        audit.full_control(control(hand_brake=0))


def test_raw_pitch_translation_and_yaw_dynamics_are_independently_reconstructed():
    row = raw_row(1, "driving", control(), 1)
    row["actor_snapshot_transform_carla"].update(x=10., y=20., z=30., yaw=90., pitch=30., roll=15.)
    row["world_velocity_carla"] = [1., 2., 3.]
    row["world_acceleration_carla"] = [4., 5., 6.]
    row["world_angular_velocity_carla_deg_s"] = [7., 8., 90.]
    result = audit.recompute_snapshot_state(row, 2.)
    assert result["x"] == pytest.approx(10.)
    assert result["y"] == pytest.approx(-(20 - math.sqrt(3) / 2))
    assert result["z"] == pytest.approx(29.5)
    assert result["vx"] == pytest.approx(2.) and result["vy"] == pytest.approx(1. + math.pi / 2)
    assert result["ax"] == pytest.approx(5.) and result["ay"] == pytest.approx(4.)


def camera_fixture(states):
    return [dict(frame=row["frame"], timestamp=row["timestamp"], capture_phase=row["capture_phase"],
        camera_order=list(audit.base.CAMERAS), source_timestamps={name: row["timestamp"] for name in audit.base.CAMERAS},
        images={name: f"images/{name}/{row['frame']:08d}.jpg" for name in audit.base.CAMERAS}) for row in states[::2]]


def test_all_six_camera_sources_need_exact_frame_phase_and_timestamp(chain):
    states, _, _ = chain
    cameras = camera_fixture(states)
    assert audit.analyze_camera_alignment(states, cameras)["status"] == "PASS"
    cameras[1]["source_timestamps"][audit.base.CAMERAS[0]] += .01
    assert audit.analyze_camera_alignment(states, cameras)["status"] == "FAIL"
    assert audit.analyze_camera_alignment(states, camera_fixture(states)[:-1])["status"] == "FAIL"


def test_primary_source_evidence_keeps_all_sixteen_pins_without_binary_match_claim():
    report = audit.primary_source_evidence()
    assert len(report["source_files"]) == 16
    assert report["executed_server_or_python_binary_match_proven"] is False
    assert all(not item["path"].startswith("/") and len(item["sha256"]) == 64 for item in report["source_files"])


@pytest.fixture
def owned_trial(tmp_path, monkeypatch):
    # HH_260906 - Only this test fixture replaces source pins; no production CLI accepts a source hash override.
    root = tmp_path / "run_001"
    def write(relative, value):
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(value if isinstance(value, bytes) else json.dumps(value).encode())
    sources = {name: (name + " synthetic").encode() for name in audit.pilot.SOURCE_NAMES}
    hashes = {name: hashlib.sha256(raw).hexdigest() for name, raw in sources.items()}
    for name, raw in sources.items():
        write("provenance/" + name, raw)
    monkeypatch.setattr(audit, "REVIEWED_ACK_SOURCES", {name: hashes[name] for name in audit.REVIEWED_ACK_SOURCES})
    monkeypatch.setattr(audit.pilot, "recorded_commit_bytes", lambda _commit, name: sources[name])
    plan = dict(source_bytes_archived=True, bounds_source_bytes_archived=True, source_sha256=hashes,
        source_head_commit="a" * 40, host="127.0.0.1", map="Town07", port=2100, capture_mode="expert", quality="Low",
        route_sha256=audit.pilot.ROUTE_SHA, collector_argv=["collector.py", "--control-transport", "acknowledged_batch"])
    write("owner_plan.json", plan)
    write("owner_result.json", dict(source_bytes_unchanged_and_archived=True, source_checks={name: True for name in sources},
                                   learned_model_control=False, vehicle_control_approved=False))
    write("owner_started.json", dict(server_pid=123, server_pgid=123, map="Town07", port=2100, quality="Low"))
    log = b"synthetic lifecycle log\n"
    write("server.log", log)
    for i, stage in enumerate(("ready", "after_capture", "stopped")):
        write("lifecycle/" + stage + ".json", dict(status="PASS", stage=stage, read_only=True, owner_pid=123, owner_pgid=123,
            generation_id="expert_123", host="127.0.0.1", port=2100, expected_map="Town07", active_map_basename="Town07",
            mode="stopped" if i == 2 else "running", port_released=True, owner_process_state=None,
            checked_at=f"2026-09-08T00:0{i}:00+00:00", server_log=dict(size_bytes=len(log), sha256=hashlib.sha256(log).hexdigest())))
    return root


def test_new_owned_protocol_sources_are_verified_without_mutating_old_reviewed_pins(owned_trial):
    old = copy.deepcopy(audit.pilot.REVIEWED_SOURCES)
    plan, _ = audit.verify_owner_sources(owned_trial, {})
    assert plan["capture_mode"] == "expert" and audit.pilot.REVIEWED_SOURCES == old


@pytest.mark.parametrize("relative,mutation", [
    ("owner_plan.json", lambda value: value.update(quality="Epic")),
    ("owner_plan.json", lambda value: value["collector_argv"].__setitem__(-1, "legacy_async")),
    ("owner_plan.json", lambda value: value["source_sha256"].pop("portable_e2e/model.py")),
    ("owner_result.json", lambda value: value["source_checks"].update({"portable_e2e/model.py": False})),
    ("lifecycle/stopped.json", lambda value: value.update(port_released=False)),
    ("lifecycle/ready.json", lambda value: value["server_log"].update(sha256="0" * 64)),
])
def test_owner_source_transport_and_lifecycle_corruption_fails_closed(owned_trial, relative, mutation):
    path = owned_trial / relative
    value = json.loads(path.read_text())
    mutation(value)
    path.write_text(json.dumps(value))
    with pytest.raises(audit.base.EvidenceError):
        audit.verify_owner_sources(owned_trial, {})


def test_unreviewed_new_collector_is_rejected_even_when_git_archive_pin_agrees(owned_trial, monkeypatch):
    monkeypatch.setitem(audit.REVIEWED_ACK_SOURCES, "scripts/e2e/collect_carla_vad_expert.py", "0" * 64)
    with pytest.raises(audit.base.EvidenceError, match="unreviewed"):
        audit.verify_owner_sources(owned_trial, {})


def test_active_trial_does_not_get_an_empty_pass(tmp_path):
    root = tmp_path / "run_001"
    root.mkdir()
    with pytest.raises(audit.base.EvidenceError, match="INCOMPLETE"):
        audit.audit_trial(root)


def test_publication_cannot_write_inside_either_raw_input_tree(tmp_path, monkeypatch):
    ack, reference = tmp_path / "ack", tmp_path / "async"
    ack.mkdir(); reference.mkdir()
    monkeypatch.setattr(audit, "audit_trial", lambda _: pytest.fail("unsafe output must be rejected before reading"))
    for output in (ack / "new-output", reference / "run_001/new-output"):
        with pytest.raises(audit.base.EvidenceError, match="outside the raw"):
            audit.publish_trial(ack, output, reference)
        assert not output.exists()


@pytest.mark.parametrize("mutation", ["ack_source", "ack_image", "async_source"])
def test_publication_rechecks_earlier_sources_and_images_after_later_reference_reads(tmp_path, monkeypatch, mutation):
    ack, reference, output = tmp_path / "ack", tmp_path / "async", tmp_path / "output"
    ack.mkdir(); reference.mkdir()
    def create(root, name):
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"original")
        return {"path": name, "sha256": hashlib.sha256(b"original").hexdigest(), "size_bytes": 8}
    source, image = create(ack, "source.json"), create(ack, "image.jpg")
    references = [dict(trial_id=name, comparison_configuration={}, source_manifest=[create(reference / name, "source.json")])
                  for name in ("run_001", "run_002")]
    result = dict(trial_id="run_001", source_manifest=[source], comparison_configuration={}, transport_protocol=dict(status="PASS"))
    monkeypatch.setattr(audit, "validate_campaign_plan", lambda *_: dict(source_manifest=[], discovered_attempts=[]))
    monkeypatch.setattr(audit, "audit_trial", lambda _: (result, [image]))
    def later_read(_):
        path = {"ack_source": ack / "source.json", "ack_image": ack / "image.jpg",
                "async_source": reference / "run_001/source.json"}[mutation]
        path.write_bytes(b"changed during subsequent read")
        return {"trials": references}
    monkeypatch.setattr(audit, "audit_async_reference", later_read)
    with pytest.raises(audit.base.EvidenceError, match="changed before publication"):
        audit.publish_trial(ack, output, reference)
    assert not output.exists()


@pytest.fixture
def prospective_trial(tmp_path):
    campaign = tmp_path / "acknowledged_control_v1"
    root = campaign / "town07_straight_calibration/run_001"
    reference = tmp_path / "comfortable_goal_stop_v3/town07_straight_calibration"
    root.mkdir(parents=True)
    reference.mkdir(parents=True)
    plan = dict(copy.deepcopy(audit.FROZEN_CAMPAIGN), comment="HH_260906 - Frozen test plan.", notice="No admission.",
                declared_at_utc="2026-09-07T19:02:00Z")
    (campaign / "pilot_plan.json").write_text(json.dumps(plan))
    owner = dict(source_head_commit=plan["source_commit"], source_sha256=audit.REVIEWED_ACK_SOURCES,
        route_sha256=plan["route_sha256"], quality="Low", map="Town07", collector_wall_timeout_sec=900,
        finish_before_utc=plan["finish_before_utc"], planned_at_utc="2026-09-07T19:02:33Z",
        collector_argv=["output", "route", "--control-transport", "acknowledged_batch", "--physics-hz", "20",
            "--capture-hz", "10", "--target-speed-kmh", "28.8", "--max-duration-sec", "180", "--weather",
            "ClearNoon", "--seed", "0", "--goal-stop-profile", "comfortable_v3"])
    (root / "owner_plan.json").write_text(json.dumps(owner))
    (root / "owner_result.json").write_text(json.dumps(dict(completed_at_utc="2026-09-07T19:03:02Z", exit_code=1)))
    (root / "lifecycle").mkdir()
    for stage, at in (("ready", "2026-09-07T19:02:40Z"), ("stopped", "2026-09-07T19:03:01Z")):
        (root / "lifecycle" / (stage + ".json")).write_text(json.dumps(dict(checked_at=at)))
    return root, reference


def test_frozen_prospective_plan_binds_actual_scope_without_claiming_campaign_completion(prospective_trial):
    report = audit.validate_campaign_plan(*prospective_trial)
    assert report["discovered_attempt_count"] == 1 and report["maximum_attempt_count"] == 2
    assert report["campaign_completion_claimed"] is False
    assert report["discovered_attempts"][0]["owner_exit_code"] == 1
    assert report["historical_file_creation_time_proven"] is False
    assert all(not item["path"].startswith("/") for item in report["source_manifest"])


@pytest.mark.parametrize("mutation", [
    lambda p: p.update(maximum_attempts_per_revision=3), lambda p: p.update(seed=False),
    lambda p: p.update(automatic_retry=0), lambda p: p.update(quality="Epic"),
    lambda p: p.update(source_commit="a" * 40), lambda p: p.update(hidden_extra=True),
    lambda p: p.update(declared_at_utc="2026-09-07T19:02:34Z"),
    lambda p: p.update(declared_at_utc="2026-09-07T19:02:00"),
    lambda p: p.update(baseline_outputs=["../wrong/run_001", "../wrong/run_002"]),
])
def test_changed_or_retroactive_prospective_plan_never_passes(prospective_trial, mutation):
    root, reference = prospective_trial
    path = root.parent.parent / "pilot_plan.json"
    plan = json.loads(path.read_text())
    mutation(plan)
    path.write_text(json.dumps(plan))
    with pytest.raises(audit.base.EvidenceError):
        audit.validate_campaign_plan(root, reference)


def test_duplicate_prospective_keys_are_not_silently_last_value_wins(prospective_trial):
    root, reference = prospective_trial
    path = root.parent.parent / "pilot_plan.json"
    path.write_text(path.read_text().replace('"seed": 0', '"seed": 1, "seed": 0'))
    with pytest.raises(audit.base.EvidenceError, match="duplicate"):
        audit.validate_campaign_plan(root, reference)


@pytest.mark.parametrize("case", ["extra_attempt", "noncontiguous", "active_selected", "exceeded_deadline", "changed_cli"])
def test_attempt_scope_and_actual_execution_are_enforced(prospective_trial, case):
    root, reference = prospective_trial
    if case == "extra_attempt":
        (root.parent / "run_003").mkdir()
    elif case == "noncontiguous":
        new_root = root.with_name("run_002")
        root.rename(new_root)
        root = new_root
    elif case == "active_selected":
        (root / "owner_result.json").unlink()
    elif case == "exceeded_deadline":
        path = root / "owner_result.json"
        value = json.loads(path.read_text())
        value["completed_at_utc"] = "2026-09-08T01:01:00Z"
        path.write_text(json.dumps(value))
    else:
        path = root / "owner_plan.json"
        value = json.loads(path.read_text())
        value["collector_argv"][value["collector_argv"].index("--target-speed-kmh") + 1] = "30"
        path.write_text(json.dumps(value))
    with pytest.raises(audit.base.EvidenceError):
        audit.validate_campaign_plan(root, reference)


@pytest.fixture
def prospective_two_trials(prospective_trial, chain, monkeypatch):
    first, reference = prospective_trial
    second = first.with_name("run_002")
    shutil.copytree(first, second)
    for relative, field, value in (("owner_plan.json", "planned_at_utc", "2026-09-07T19:05:00Z"),
        ("owner_result.json", "completed_at_utc", "2026-09-07T19:06:02Z"),
        ("lifecycle/ready.json", "checked_at", "2026-09-07T19:05:10Z"),
        ("lifecycle/stopped.json", "checked_at", "2026-09-07T19:06:01Z")):
        path = second / relative
        value_dict = json.loads(path.read_text())
        value_dict[field] = value
        path.write_text(json.dumps(value_dict))
    episode = first / "episode.partial"
    episode.mkdir()
    for name, rows in (("states.jsonl", chain[0]), ("control_receipts.jsonl", chain[1])):
        (episode / name).write_text("".join(json.dumps(row) + "\n" for row in rows))
    (episode / "manifest.json").write_text(json.dumps(dict(capture_contract=dict(control_transport=dict(bootstrap_observation=chain[2])))))
    qa = dict(raw_scalar_quality_clear=False, event_counts=dict(collision=0, lane_invasion=0),
        final_driving=dict(goal_error_m=.8), speed_rate_qa={name: dict(by_phase=dict(all=dict(minimum_mps2=value)))
            for name, value in (("native_20hz", -8.), ("camera_10hz", -4.))})
    monkeypatch.setattr(audit.base, "summarize_trial", lambda *_: (dict(independent_qa=qa, owner_exit_code=1, source_manifest=[]), None))
    review = dict(comment="HH_260906 - Test review.", reason="One final bounded repeat.", reviewed_at_utc="2026-09-07T19:04:00Z",
        previous_output="town07_straight_calibration/run_001", previous_states_sha256=audit.base.sha(episode / "states.jsonl"),
        previous_receipts_sha256=audit.base.sha(episode / "control_receipts.jsonl"), owner_exit_code=1, source_checks_passed=10,
        lifecycle_ready_after_stopped="PASS", independent_transport_status="PASS", acknowledged_receipts=11, control_mismatch_count=0,
        native_scalar_quality="FAIL", minimum_native_speed_rate_mps2=-8., minimum_camera_speed_rate_mps2=-4., goal_distance_m=.8,
        collision_count=0, lane_invasion_count=0, next_output="town07_straight_calibration/run_002",
        source_and_parameters_changed=False, maximum_attempts_per_revision=2, automatic_retry=False, training_data_approved=False)
    (first.parent.parent / "run_002_preflight_review.json").write_text(json.dumps(review))
    return second, reference


def test_second_attempt_requires_bound_prior_review_and_retains_failed_first(prospective_two_trials):
    result = audit.validate_campaign_plan(*prospective_two_trials)
    assert result["prior_attempt_review_record_verified"] is True
    assert [row["owner_exit_code"] for row in result["discovered_attempts"]] == [1, 1]


@pytest.mark.parametrize("field,value", [("reviewed_at_utc", "2026-09-07T19:06:00Z"),
    ("previous_states_sha256", "0" * 64), ("previous_receipts_sha256", "0" * 64),
    ("acknowledged_receipts", 12), ("minimum_native_speed_rate_mps2", -2.),
    ("source_and_parameters_changed", True), ("training_data_approved", True)])
def test_second_attempt_review_cannot_rewrite_previous_evidence(prospective_two_trials, field, value):
    root, reference = prospective_two_trials
    path = root.parent.parent / "run_002_preflight_review.json"
    review = json.loads(path.read_text())
    review[field] = value
    path.write_text(json.dumps(review))
    with pytest.raises(audit.base.EvidenceError):
        audit.validate_campaign_plan(root, reference)


def test_complete_campaign_refuses_one_attempt_without_creating_output(prospective_trial, tmp_path):
    first, reference = prospective_trial
    output = tmp_path / "new-output"
    with pytest.raises(audit.base.EvidenceError, match="INCOMPLETE"):
        audit.publish_campaign(first.parent.parent, output, reference)
    assert not output.exists()


def test_complete_campaign_retains_all_four_failed_results_without_admission(prospective_two_trials, tmp_path, monkeypatch):
    second, reference = prospective_two_trials
    def result(root):
        return dict(trial_id=root.name, status="TRANSPORT_PASS_NOT_ADMITTED", transport_protocol=dict(status="PASS"),
                    owner_exit_code=1, source_manifest=[], comparison_configuration={}), []
    monkeypatch.setattr(audit, "audit_trial", result)
    monkeypatch.setattr(audit, "audit_async_reference", lambda _: dict(trials=[dict(trial_id=name, source_manifest=[],
        comparison_configuration={}, status="FAIL_NOT_ADMITTED") for name in ("run_001", "run_002")]))
    output = tmp_path / "new-output"
    report = audit.publish_campaign(second.parent.parent, output, reference)
    assert report["status"] == "COMPLETE_NOT_ADMITTED" and report["acknowledged_attempt_count"] == report["async_attempt_count"] == 2
    assert report["scope"]["dataset_admission"] is False and report["scope"]["physical_actuation_proven"] is False
    assert (output / "summary.json").is_file() and (output / "SHA256SUMS").is_file()
