"""HH_261004 - Test callback45 immutable whole-interval publication using only synthetic CPU evidence."""

import gzip
import hashlib
import io
import json
from pathlib import Path
import sys
from types import SimpleNamespace

from PIL import Image
import pytest

from learning.tests import test_publish_temporal_route_joint_town03_right_20261002 as right_fixture
from scripts.e2e import publish_temporal_route_joint_town03_straight_callback45_v3_20261004 as p


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(value if isinstance(value, bytes) else json.dumps(value, allow_nan=False).encode())
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path):
    return [json.loads(line) for line in path.read_bytes().splitlines()]


@pytest.fixture
def archive_factory(tmp_path, monkeypatch):
    def make(count=6):
        root = tmp_path / "workspace"
        run = root / "storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_callback45_v3"
        output = root / "docs/assets/validation/2026-10-04/portable_e2e_learning_cycle_v1/09_town03_callback_to_plan_v3"
        policy_path = root / ("storage/experiments/plans/2026-10-04/town03_known_dev_straight_callback45_v3/"
            "ROOT_prospective_publication_policy.json")
        monkeypatch.setattr(p, "ROOT", root)
        monkeypatch.setattr(p, "RUN", run)
        monkeypatch.setattr(p, "OUTPUT", output)
        monkeypatch.setattr(p, "POLICY_PATH", policy_path)
        monkeypatch.setattr(p, "publication_sources", lambda: set())
        states, cameras, commands, receipts, geometry = right_fixture.native_records(count)
        warmup = 24 if count == 900 else min(2, count)
        for index, state in enumerate(states):
            state["capture_phase"] = "stationary_warmup" if index < warmup else "driving"
        episode = run / "episode.partial"
        for name, content in zip(
                ("states.jsonl", "camera_frames.jsonl", "learned_control.jsonl", "control_receipts.jsonl"),
                (states, cameras, commands, receipts)):
            write(episode / name, b"\n".join(json.dumps(row).encode() for row in content))
        write(episode / "learned_summary.json", dict(handoff_occurred=True,
            counts=dict(inference_count=max(1, count - 2), prefill_count=2,
                learned_commands_acknowledged=max(1, count - 2), emergency_commands=0)))
        manifest = dict(status="failed", error="short synthetic failure",
            training_data_approved=False, vehicle_control_approved=False,
            result=dict(state_count=count, camera_anchor_count=len(cameras)),
            capture_contract=dict(capture_phase_schedule=dict(stationary_tail=dict(scheduled_ticks=0))))
        full = count == 900
        if full:
            manifest["error"] = p.EXPECTED_BUDGET_ERROR
        write(episode / "manifest.json", manifest)
        route_sha = write(episode / "route.json", dict(town="Town03", scenario="straight",
            route_length_m=209.370221122845,
            route=[dict(x=float(index), y=0.0, yaw=0.0) for index in range(211)]))
        monkeypatch.setattr(p, "ROUTE_SHA", route_sha)
        picture = io.BytesIO()
        Image.new("RGB", (640, 360), (51, 86, 114)).save(picture, format="JPEG")
        for camera in cameras:
            for relative in camera["images"].values():
                write(episode / relative, picture.getvalue())
        plan = dict(schema=p.OWNER_SCHEMA, output=str(run), town="Town03", scenario="straight",
            dataset_split="val", known_DEV_site_reused=True, independent_validation=False,
            attempts=1, automatic_retry=False, total_capture_seconds=45,
            maximum_native_state_count=900, maximum_camera_bundle_count=450,
            full_original_navigation_points=211, original_navigation_sha256=route_sha,
            optimizer_updates=6000, speed_preview_s=.5, controller_target_cap_mps=2.4,
            full_RTE01_approved=False, full_MAIN9_completion_claimed=False,
            training_data_approved=False, vehicle_control_approved=False, model_promoted=False,
            source_sha256={}, external_source_sha256={}, asset_sha256={},
            actual_scoped_camera_and_ack_transport_late_bound_before_capture=True,
            # HH_261004 - Synthetic V3 fixtures must carry the separate frozen start/recording contract.
            actual_deferred_JPEG_after_ACK_required=True,
            failed_callback45_v2_reference=dict(path=str(p.V2_RUN), sha256=p.V2_EVIDENCE_SHA,
                source_sha256=p.V2_SOURCE_SHA, status="FAIL_KNOWN_DEV_STRAIGHT_PRECURSOR", overwritten=False),
            actual_callback_timing=dict(schema=p.CALLBACK_SCHEMA, attempts=1,
                repeated_role_frame_or_simstamp_callbacks_preserved=True,
                repeated_native_frame_or_simstamp_alone_is_image_identity=False,
                selected_source_join_fields=["role", "native_frame", "image_object_id", "source_image_timestamp_simulation_ns"],
                selected_source_exactly_one_original_callback_required=True,
                ambiguous_same_role_native_frame_object_receipts_denied=True,
                source_actor_and_BGRA_SHA_required=True, causal_history_joins_consumed_bundle_identity_and_SHA_only=True,
                earliest_or_latest_duplicate_receipt_selected_by_latency=False,
                repeated_callbacks_with_different_pixels_permitted_only_as_distinct_unmodified_sources=True,
                native_dispatch_timing=dict(stages=list(p.NATIVE_STAGES), native_ticks_required=900,
                    native_camera_ticks_required=450, native_scalar_marks_required=7200,
                    first_tick_frame_unknown_until_actual_world_tick_returns=True,
                    physics_control_RPC_or_source_order_changed=False, scalar_clocks_only_no_waits=True,
                    external_CPU_load_or_hardware_exposure_causation_claimed=False),
                callback_receipt_JSON_maximum_bytes=16 * 1024**2,
                callback_receipt_JSON_SHA_and_schema_and_array_bounds_required=True,
                generic_JSON_reader_limit_changed=False, original_same_recorder_bundle_provider_replaced=False,
                timed_policy_globals_bound_after_complete_preview_PI_activation=True,
                latest_camera_bundles_required=450, latest_camera_callback_receipts_required=2700,
                first_callback_to_plan_maximum_seconds=.1, first_callback_to_ACK_maximum_seconds=.1,
                actual_base_policy_observation_statement_count=3,
                all_extra_bootstrap_or_unconsumed_callbacks_retained=True,
                source_math_inverse_AST_required=True,
                original_queue_put_argument_and_return_and_exception_preserved=True,
                callback_to_policy_source_SHA_required=True, every_causal_history_SHA_required=True,
                fresh_and_held_and_teacher_and_prefill_separate=True,
                additional_queue_or_source_image_buffer=False, input_control_or_inference_order_changed=False,
                hardware_exposure_wall_clock_observed=False, ten_hz_wall_rate_proven=False,
                callback_receipt_file_IO_during_capture=False, model_structure_changed=False,
                model_weights_changed=False, controller_math_changed=False,
                decoder_gate_or_tracker_math_changed=False, original_deadline_relaxed=False),
            authenticated_start_corridor=dict(upstream_lane=[67, 0, 2],
                enabled_only_when_body_arc_lower_bound_negative=True,
                original_navigation_bytes_unchanged=True),
            # HH_261004 - Synthetic V3 archives must disclose only the real fixture-schema intervention.
            fixture_owner_schema_admission=dict(declared_owner_schema=p.OWNER_SCHEMA,
                actual_fixture_module_global_required=True,
                inherited_v2_deferred_recording_unchanged=True,
                inherited_authenticated_start_corridor_unchanged=True,
                model_weights_changed=False, controller_changed=False, policy_changed=False,
                route_or_map_changed=False, camera_or_ACK_changed=False,
                speed_or_acceleration_gate_changed=False, latency_gate_changed=False,
                body_area_or_inset_gate_changed=False),
            owned_green_signal_fixture=dict(original_own_world_traffic_actors="ALL", state="Green",
                frozen=True, ignore_traffic_lights=False,
                all_original_hazard_detectors_and_emergency_returns_unchanged=True,
                closed_course_basic_straight_only_not_traffic_rule_performance=True))
        declaration_sha = write(run / "declaration.json", plan)
        prospective_plan = run.with_name(run.name + "_plan.json")
        assert write(prospective_plan, plan) == declaration_sha
        owner = dict(declaration_sha256=declaration_sha, cleanup_errors=[], source_hashes_match=True,
            external_source_hashes_match=True, status="CAPTURE_RETURNED_NONZERO_RETAINED")
        write(run / "owner_result.json", owner)
        body = dict(all_native_boxes=count, all_snapshot_linear_sweeps=count - 1,
            lane_inset_m=.1, body_and_swept_clearance_passed=False)
        write(run / "derived_body/summary.json", body)
        write(run / "derived_body/all_native_boxes_and_sweeps.jsonl.gz",
              gzip.compress(b"\n".join(json.dumps(row).encode() for row in geometry)))
        diagnostic = dict(schema=p.OWNER_SCHEMA + ".diagnostic_report", original_owner=owner,
            full_MAIN9_completion_claimed=False, full_RTE01_approved=False,
            training_data_approved=False, vehicle_control_approved=False, model_promoted=False,
            original_capture_status="failed", original_capture_error=manifest["error"], independent_body=body,
            actual_native_state_count=count, actual_camera_bundle_count=len(cameras),
            actual_ACK_receipt_count=len(receipts), bounded_capture_complete=full,
            fixed45_full_denominator_passed=full, learned_driving_screen_passed=False,
            straight_motion_body_policy_wall100ms_passed=False,
            callback_straight_motion_body_policy_wall100ms_passed=False,
            observed_bundle_to_ACK_wall100ms_passed=False,
            all_failed_tail_retained=full, all_available_raw_files_retained_and_hashed=True,
            raw_before_after_unchanged=True,
            hardware_exposure_to_ACK_proven=False, ten_hz_wall_rate_proven=False,
            status="FAIL_KNOWN_DEV_STRAIGHT_PRECURSOR",
            raw_source_sha256={str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                               for path in episode.rglob("*") if path.is_file()})
        write(run / "diagnostic_report.json", diagnostic)
        font_sha256 = {str(path): p.prior.digest(path) for path in
            [Path("/usr/share/fonts/truetype/dejavu") / name
             for name in ("DejaVuSans.ttf", "DejaVuSans-Bold.ttf")]}
        policy = dict(schema=p.POLICY_SCHEMA, created_before_actual_episode=True,
            actual_episode_observed=False, run_root=str(run), output=str(output), attempts=1,
            automatic_retry=False, run_declaration=dict(path=str(prospective_plan), sha256=declaration_sha),
            PNG_selection=dict(policy=p.PNG_INDEX_POLICY,
                camera_indices_if_full450=p.FULL450_PNG_CAMERA_INDICES,
                selection_depends_on_outcome_or_failure_location=False),
            GIF_selection=dict(regular_bin_seconds=.5, regular_camera_indices=list(range(0, 450, 5)),
                exact_last_camera_index=449, resolved_full450_camera_indices_count=91,
                resolved_full450_playback_duration_ms=45_000, failed_tail_cropped=False,
                selection_depends_on_outcome_or_failure_location=False),
            publication_source_sha256={},
            publisher=dict(path=str(Path(p.__file__).resolve()), sha256=p.prior.digest(Path(p.__file__).resolve())),
            font_sha256=font_sha256, known_DEV_site_reused=True, independent_validation=False,
            full_RTE01_approved=False, full_MAIN9_completion_claimed=False,
            training_data_approved=False, vehicle_control_approved=False, model_promoted=False)
        policy_sha = write(policy_path, policy)
        caller = dict(schema=p.CALLER_SCHEMA, output=str(output), attempts=1,
            PNG_index_policy=p.PNG_INDEX_POLICY,
            PNG_percent_numerators=p.PNG_PERCENT_NUMERATORS,
            expected_full_PNG_camera_indices=p.FULL450_PNG_CAMERA_INDICES,
            run_root=str(run), publication_source_sha256={}, font_sha256=font_sha256,
            prospective_publication_policy=dict(path=str(policy_path), sha256=policy_sha),
            run_files_sha256={name: p.prior.digest(run / name) for name in p.previous.inventory(run)})
        caller_path = tmp_path / "caller.json"
        return caller_path, write(caller_path, caller), caller, run
    return make


def refresh(value):
    caller_path, _, caller, run = value
    plan = json.loads((run / "declaration.json").read_bytes())
    owner = json.loads((run / "owner_result.json").read_bytes())
    owner["declaration_sha256"] = p.prior.digest(run / "declaration.json")
    write(run / "owner_result.json", owner)
    diagnostic = json.loads((run / "diagnostic_report.json").read_bytes())
    diagnostic["original_owner"] = owner
    episode = run / "episode.partial"
    diagnostic["raw_source_sha256"] = {str(path): p.prior.digest(path)
        for path in episode.rglob("*") if path.is_file()}
    write(run / "diagnostic_report.json", diagnostic)
    caller["run_files_sha256"] = {name: p.prior.digest(run / name) for name in p.previous.inventory(run)}
    return caller_path, write(caller_path, caller), caller, run


def test_predeclared_sampling_and_full_playback_are_result_independent():
    assert p.FULL450_PNG_CAMERA_INDICES == {
        "01_start.png": 0, "02_quarter.png": 112, "03_half.png": 224,
        "04_three_quarters.png": 337, "05_last.png": 449}
    assert p.png_indices(450) == p.FULL450_PNG_CAMERA_INDICES
    indices = p.camera_indices(450)
    assert indices == sorted(set(range(0, 450, 5)) | {449}) and len(indices) == 91
    bundle = dict(states=[dict(timestamp=index * .05) for index in range(900)],
                  cameras=[dict(timestamp=index * .1) for index in range(450)])
    assert sum(p.playback(bundle, indices)) == 45_000


@pytest.mark.parametrize("count", [1, 2, 3, 6, 449, 450])
def test_percentage_samples_include_actual_start_and_last(count):
    result = p.png_indices(count)
    assert result["01_start.png"] == 0 and result["05_last.png"] == count - 1
    assert list(result.values()) == sorted(result.values())


@pytest.mark.parametrize("value", [False, 0, 451, 5.0])
def test_invalid_camera_denominators_fail(value):
    with pytest.raises(Exception):
        p.camera_indices(value)
    with pytest.raises(Exception):
        p.png_indices(value)


def test_short_archive_is_retained_but_cannot_pass(archive_factory):
    value = archive_factory(6)
    _, bundle, pins = p.load_archive(*value[:2])
    pins.recheck()
    assert bundle["full_fixed45"] is False and len(bundle["states"]) == 6
    assert bundle["diagnostic"]["straight_motion_body_policy_wall100ms_passed"] is False
    assert bundle["control_census"]["observed_prior_learned_native_effect_count"] > 0


@pytest.mark.parametrize("field", ["bounded_capture_complete", "fixed45_full_denominator_passed",
                                    "learned_driving_screen_passed", "straight_motion_body_policy_wall100ms_passed"])
def test_short_archive_cannot_be_relabelled_pass(archive_factory, field):
    value = archive_factory(6)
    diagnostic = json.loads((value[3] / "diagnostic_report.json").read_bytes())
    diagnostic[field] = True
    write(value[3] / "diagnostic_report.json", diagnostic)
    value = refresh(value)
    with pytest.raises(Exception):
        p.load_archive(*value[:2])


@pytest.mark.parametrize("mutation", ["manifest_complete", "diagnostic_pass", "error_mismatch"])
def test_short_archive_requires_an_authentic_matching_fail_outcome(archive_factory, mutation):
    value = archive_factory(6)
    episode, run = value[3] / "episode.partial", value[3]
    manifest = json.loads((episode / "manifest.json").read_bytes())
    diagnostic = json.loads((run / "diagnostic_report.json").read_bytes())
    if mutation == "manifest_complete":
        manifest["status"] = "complete"
    elif mutation == "diagnostic_pass":
        diagnostic["status"] = "PASS_KNOWN_DEV_STRAIGHT_PRECURSOR"
    else:
        diagnostic["original_capture_error"] = "different failure"
    write(episode / "manifest.json", manifest)
    write(run / "diagnostic_report.json", diagnostic)
    value = refresh(value)
    with pytest.raises(Exception):
        p.load_archive(*value[:2])


def test_complete_900_450_archive_requires_exact_budget_tail(archive_factory):
    value = archive_factory(900)
    _, bundle, pins = p.load_archive(*value[:2])
    pins.recheck()
    assert bundle["full_fixed45"] is True
    assert len(bundle["states"]) == 900 and len(bundle["cameras"]) == 450


@pytest.mark.parametrize("mutation", ["error", "goal_false", "warmup", "count"])
def test_full_archive_rejects_noncanonical_denominator(archive_factory, mutation):
    value = archive_factory(900)
    episode = value[3] / "episode.partial"
    manifest = json.loads((episode / "manifest.json").read_bytes())
    if mutation == "error":
        manifest["error"] = "favorable early stop"
    elif mutation == "goal_false":
        manifest["result"]["goal_reached"] = False
    elif mutation == "count":
        manifest["result"]["state_count"] = 899
    else:
        state_values = rows(episode / "states.jsonl")
        state_values[23]["capture_phase"] = "driving"
        write(episode / "states.jsonl", b"\n".join(json.dumps(row).encode() for row in state_values))
    write(episode / "manifest.json", manifest)
    value = refresh(value)
    with pytest.raises(Exception):
        p.load_archive(*value[:2])


@pytest.mark.parametrize("field", ["PNG_index_policy", "PNG_percent_numerators",
                                    "expected_full_PNG_camera_indices", "output"])
def test_caller_cannot_change_sampling_or_output(archive_factory, field):
    value = archive_factory(6)
    if field == "output":
        value[2][field] += "_best"
    else:
        value[2][field] = {}
    digest = write(value[0], value[2])
    with pytest.raises(Exception):
        p.load_archive(value[0], digest)


@pytest.mark.parametrize("mutation", ["path", "sha", "bytes"])
def test_post_episode_caller_must_bind_unchanged_pre_episode_policy(archive_factory, mutation):
    value = archive_factory(6)
    reference = value[2]["prospective_publication_policy"]
    if mutation == "path":
        reference["path"] += ".other"
    elif mutation == "sha":
        reference["sha256"] = "0" * 64
    else:
        write(Path(reference["path"]), {})
    digest = write(value[0], value[2])
    with pytest.raises(Exception):
        p.load_archive(value[0], digest)


def test_validate_only_creates_no_gallery(archive_factory):
    value = archive_factory(6)
    result = p.publish(SimpleNamespace(caller_pins=str(value[0]), caller_pins_sha256=value[1],
        validate_only=True, render_all_media=False))
    assert result["validated"] and not result["rendered"] and not p.OUTPUT.exists()
    assert result["PNG_camera_indices"] == p.png_indices(3)


def test_short_synthetic_render_keeps_last_tail_and_labels_diagnostic_not_rviz(archive_factory, monkeypatch):
    value = archive_factory(6)
    monkeypatch.setattr(p.shutil, "disk_usage", lambda _: SimpleNamespace(free=9 * 1024**3))
    result = p.publish(SimpleNamespace(caller_pins=str(value[0]), caller_pins_sha256=value[1],
        validate_only=False, render_all_media=True))
    assert result["GIF_count"] == 1 and result["PNG_count"] == 5
    provenance = json.loads((p.OUTPUT / "publication_provenance.json").read_bytes())
    assert provenance["full_fixed45"] is False and provenance["failed_tail_cropped"] is False
    assert provenance["resolved_actual_PNG_camera_indices"]["05_last.png"] == 2
    assert provenance["displayed_GIF_frames"][-1]["observed_frame"] == 1004
    assert provenance["actual_prior_applied_NN_plan_displayed"] is True
    assert provenance["actual_prior_applied_NN_plan_GIF_frame_count"] > 0
    assert provenance["diagnostic_rendering_not_genuine_Autoware_RViz"] is True
    assert provenance["autoware_UI_captured"] is False and provenance["six_real_recorded_cameras"] is True
    text = (p.OUTPUT / "README.md").read_text()
    assert "메인 0/9·하위 0/30" in text and "실제 Autoware 또는 RViz 화면이 아닙니다" in text


def test_render_passes_full_route_traveled_body_and_prior_nn_plan_to_centered_panel(archive_factory, monkeypatch):
    value = archive_factory(6)
    _, bundle, _ = p.load_archive(*value[:2])
    calls = []
    original = p.visual.map_panel

    def observed(state, body, route, traveled, plan):
        calls.append((state, body, route, traveled, plan))
        return original(state, body, route, traveled, plan)

    monkeypatch.setattr(p.visual, "map_panel", observed)
    p.render(bundle, 1)
    state, body, route, traveled, plan = calls.pop()
    assert state["frame"] == bundle["cameras"][1]["frame"] and body["frame"] == state["frame"]
    assert len(route) == 211 and len(traveled) == 3
    expected = p.prior.observed_prior_ack(
        state, bundle["receipts"], bundle["commands"], bundle["by_frame"])[2]
    assert plan and plan == expected


@pytest.mark.parametrize("mutation", ["extra", "missing", "symlink", "raw_mutation"])
def test_complete_external_inventory_and_first_read_pins_reject_changes(archive_factory, mutation):
    value = archive_factory(6)
    run, episode = value[3], value[3] / "episode.partial"
    if mutation == "extra":
        write(run / "late", b"extra")
    elif mutation == "missing":
        (run / "owner_result.json").unlink()
    elif mutation == "symlink":
        (run / "alias").symlink_to(run / "owner_result.json")
    else:
        write(episode / "states.jsonl", b"{}\n")
    with pytest.raises(Exception):
        p.load_archive(*value[:2])


def test_existing_output_is_never_overwritten(archive_factory):
    value = archive_factory(6)
    p.OUTPUT.mkdir(parents=True)
    sentinel = p.OUTPUT / "user"
    sentinel.write_text("preserve")
    with pytest.raises(Exception):
        p.publish(SimpleNamespace(caller_pins=str(value[0]), caller_pins_sha256=value[1],
            validate_only=True, render_all_media=False))
    assert sentinel.read_text() == "preserve"


def test_model_or_simulator_import_cannot_enter_archive_reader(archive_factory, monkeypatch):
    value = archive_factory(6)
    monkeypatch.setitem(sys.modules, "torch", object())
    with pytest.raises(Exception):
        p.load_archive(*value[:2])


def test_publication_source_closure_includes_new_and_reviewed_renderer():
    sources = p.publication_sources()
    assert Path(p.__file__).resolve() in sources and Path(p.previous.__file__).resolve() in sources
    assert p.prior.digest(Path(p.previous.__file__)) == p.PREVIOUS_SHA


def test_cli_requires_exactly_one_noninference_mode(monkeypatch):
    called = []
    monkeypatch.setattr(p, "publish", lambda args: called.append(args) or {"ok": True})
    assert p.main(["--caller-pins", "/x", "--caller-pins-sha256", "a" * 64, "--validate-only"]) == 0
    assert called[-1].validate_only and not called[-1].render_all_media
    for options in ([], ["--validate-only", "--render-all-media"]):
        with pytest.raises(Exception):
            p.main(["--caller-pins", "/x", "--caller-pins-sha256", "a" * 64] + options)


# HH_261004 - V1 declarations cannot be silently reused as the corrected V3 evidence.
@pytest.mark.parametrize("key", ["actual_deferred_JPEG_after_ACK_required", "authenticated_start_corridor"])
def test_v3_recording_and_topology_contract_is_required(archive_factory, key):
    value = archive_factory()
    caller_path, caller_sha, caller, run = value
    plan = json.loads((run / "declaration.json").read_bytes())
    plan.pop(key)
    # HH_261004 - Re-pin every outer identity so rejection proves the missing V3 contract, not stale hashes.
    new_sha = write(run / "declaration.json", plan)
    write(run.with_name(run.name + "_plan.json"), plan)
    policy_path = Path(caller["prospective_publication_policy"]["path"])
    policy = json.loads(policy_path.read_bytes())
    policy["run_declaration"]["sha256"] = new_sha
    caller["prospective_publication_policy"]["sha256"] = write(policy_path, policy)
    value = refresh(value)
    with pytest.raises(Exception, match="exact V3 after-ACK and start-corridor declaration required"):
        p.load_archive(*value[:2])
    assert p.OWNER_SCHEMA.endswith(".callback45_v3")


# HH_261004 - Re-seal outer pins so denial specifically tests fixture admission, not an unrelated stale hash.
@pytest.mark.parametrize("mutation", ["absent", "schema", "guard", "weights", "latency"])
def test_v3_exact_owner_binding_without_gate_changes_is_required(archive_factory, mutation):
    value = archive_factory()
    _, _, caller, run = value
    plan = json.loads((run / "declaration.json").read_bytes())
    if mutation == "absent":
        plan.pop("fixture_owner_schema_admission")
    else:
        key, replacement = dict(schema=("declared_owner_schema", "other-owner"),
            guard=("actual_fixture_module_global_required", False),
            weights=("model_weights_changed", True), latency=("latency_gate_changed", True))[mutation]
        plan["fixture_owner_schema_admission"][key] = replacement
    new_sha = write(run / "declaration.json", plan)
    write(run.with_name(run.name + "_plan.json"), plan)
    policy_path = Path(caller["prospective_publication_policy"]["path"])
    policy = json.loads(policy_path.read_bytes())
    policy["run_declaration"]["sha256"] = new_sha
    caller["prospective_publication_policy"]["sha256"] = write(policy_path, policy)
    value = refresh(value)
    with pytest.raises(Exception, match="exact V3 fixture-only owner admission required"):
        p.load_archive(*value[:2])


# HH_261004 - Exercise the new measured-clock contract independently from the original V3 fixture contract.
@pytest.mark.parametrize("key", ["schema", "latest_camera_callback_receipts_required",
    "first_callback_to_plan_maximum_seconds", "source_math_inverse_AST_required",
    "original_queue_put_argument_and_return_and_exception_preserved", "ten_hz_wall_rate_proven"])
def test_callback_prospective_contract_cannot_be_omitted_or_relaxed(archive_factory, key):
    value = archive_factory()
    plan = json.loads((value[3] / "declaration.json").read_bytes())
    p.verify_callback_contract(plan)
    plan["actual_callback_timing"].pop(key)
    with pytest.raises(Exception, match="exact prospective callback-to-plan"):
        p.verify_callback_contract(plan)


@pytest.mark.parametrize("key", ["hardware_exposure_to_ACK_proven", "ten_hz_wall_rate_proven"])
def test_failed_timing_cannot_certify_an_unobserved_clock(archive_factory, key):
    value = archive_factory()
    diagnostic = json.loads((value[3] / "diagnostic_report.json").read_bytes())
    diagnostic[key] = True
    with pytest.raises(Exception, match="simulation callbacks cannot certify"):
        p.verify_callback_outcome(diagnostic, SimpleNamespace(), value[3] / "episode.partial", [], [])


def timing_pass_fixture(tmp_path):
    """HH_261004 - Create explicitly synthetic scalar receipts, never present them as a recorded test."""
    episode = tmp_path / "synthetic_episode"
    path = episode / "callback_timing_receipts.json"
    callbacks, bundles, cameras = [], [], []
    roles = p.MODEL_CAMERA_ORDER
    for index in range(450):
        frame, stamp, clock = 1000 + index * 2, (index + 1) * 100_000_000, 1_000_000_000 + index * 100_000_000
        bound = []
        for offset, role in enumerate(roles):
            source = dict(role=role, native_frame=frame, image_object_id=index * 6 + offset + 1,
                source_image_timestamp_simulation_ns=stamp)
            bound.append(source)
            callbacks.append(dict(**source, actor_id=offset + 1, receipt_sequence=len(callbacks) + 1,
                source_byte_count=640 * 360 * 4, source_BGRA_encoding_prefixed_sha256=str(offset + 1) * 64,
                original_queue_put_calls=1, original_queue_put_returned=True, original_queue_put_error=None,
                receipt_error=None, callback_enter_monotonic_ns=clock, queue_put_started_monotonic_ns=clock + 1,
                queue_put_finished_monotonic_ns=clock + 2, source_SHA_finished_monotonic_ns=clock + 3))
        bundles.append(dict(native_frame=frame, cameras=bound, bundle_binding_finished_monotonic_ns=clock + 15_000_000))
        cameras.append(dict(frame=frame, timestamp=stamp / 1.e9, camera_order=list(roles),
            source_timestamps={role: stamp / 1.e9 for role in roles}))
    # HH_261004 - Synthetic repeated native-frame sources have distinct identities and pixels, with no favorable timestamp selection.
    for original in callbacks[:6]:
        duplicate = dict(original, receipt_sequence=len(callbacks) + 1, image_object_id=5000 + len(callbacks),
            source_BGRA_encoding_prefixed_sha256="a" * 64)
        callbacks.append(duplicate)
    native = []
    states = [dict(frame=1000 + index, capture_phase="stationary_warmup" if index < 24 else "driving") for index in range(900)]
    for index, state in enumerate(states):
        selected = p.NATIVE_STAGES if index % 2 == 0 else p.NATIVE_STAGES[:7]
        base = 1_000_000_000 + index * 50_000_000 - 20_000_000
        for offset, stage in enumerate(selected):
            native.append(dict(tick_index=index, stage=stage, native_frame=None if offset == 0 else state["frame"],
                capture_phase=state["capture_phase"], stage_finished_monotonic_ns=base + (0, 27, 28, 29, 30, 31, 32, 33, 35)[offset] * 1_000_000,
                agent_run_step_invoked=state["capture_phase"] == "driving" if stage.startswith("agent_branch") else None,
                original_native_order_or_math_changed=False))
    receipt = dict(schema=p.CALLBACK_SCHEMA, errors=[], callback_count=2706,
        callbacks=callbacks, bundles=bundles, camera_bindings=[dict(role=role, actor_id=index + 1) for index, role in enumerate(roles)],
        policy_stage_marks=[dict(stage=stage, frame_timestamp_simulation_ns=500_000_000,
            policy_object_id=1, stage_finished_monotonic_ns=clock, original_policy_math_changed=False)
            for stage, clock in zip(p.POLICY_STAGES, (1_430_000_000, 1_450_000_000, 1_470_000_000))],
        native_stage_marks=native, clock="time.perf_counter_ns_monotonic", hardware_exposure_clock_observed=False,
        all_callbacks_including_bootstrap_or_unconsumed_preserved=True, source_math_inverse_AST_verified=True,
        inflight_callbacks=0, scalar_receipt_limit_exceeded=False,
        hardware_exposure_to_ACK_proven=False, ten_hz_wall_rate_proven=False)
    row = dict(inference_frame=1008, ACK_sequence=3, first_callback_monotonic_ns=1_400_000_000,
        last_callback_monotonic_ns=1_400_000_000, source_image_timestamp_simulation_ns=500_000_000,
        input_ready_monotonic_ns=1_430_000_000, raw_nn_finished_monotonic_ns=1_450_000_000,
        decoded_plan_ready_monotonic_ns=1_470_000_000, ACK_finished_monotonic_ns=1_480_000_000,
        selected_source_callback_receipt_sequences=list(range(25, 31)), selected_source_image_object_ids=list(range(25, 31)),
        callback_first_to_plan_seconds=.07, callback_first_to_ACK_seconds=.08)
    audit = dict(rows=[row], fresh_NN_plan_count=1, selected_callback_count=2700,
        callback_count=2706, extra_unselected_callback_count=6,
        callback_receipt_identity_and_SHA_passed=True, callback_plan_ACK_chain_passed=True,
        exact_consumed_image_object_join_passed=True, consumed_history_bundle_identity_and_SHA_passed=True,
        fresh_callback_to_plan_wall100ms_passed=True, fresh_callback_to_ACK_wall100ms_passed=True,
        hardware_exposure_to_ACK_proven=False, ten_hz_wall_rate_proven=False,
        metrics=dict(callback_first_to_plan_seconds=dict(count=1, maximum=.07),
                     callback_first_to_ACK_seconds=dict(count=1, maximum=.08)))
    diagnostic = dict(straight_motion_body_policy_wall100ms_passed=True,
        callback_straight_motion_body_policy_wall100ms_passed=True,
        hardware_exposure_to_ACK_proven=False, ten_hz_wall_rate_proven=False,
        archived_v3_evidence_preserved=True, callback_receipt_identity_and_SHA_passed=True,
        archived_callback45_v1_failure_preserved=True,
        archived_callback45_v2_failure_preserved=True, native_pre_delivery_stage_closure_passed=True,
        native_pre_delivery_timing=dict(native_tick_count=900, native_camera_tick_count=450, native_scalar_mark_count=7200,
            native_pre_delivery_stage_closure_passed=True, physics_control_RPC_order_changed=False,
            hardware_exposure_to_ACK_proven=False, CPU_scheduling_or_external_load_cause_proven=False,
            all_native_stage_receipts_retained=True, native_frame_invented_before_tick_return=False,
            native_mark_clock="time.perf_counter_ns_monotonic"),
        callback_plan_ACK_chain_passed=True, fresh_callback_to_plan_wall100ms_passed=True,
        fresh_callback_to_ACK_wall100ms_passed=True, actual_six_camera_callback_observed=True,
        actual_callback_timing=audit,
        callback_receipt_JSON_readback=dict(path=str(path), sha256="synthetic-only", actual_bytes=len(json.dumps(receipt).encode()),
            maximum_bytes=16 * 1024**2, strict_schema_and_array_bounds_passed=True,
            generic_JSON_reader_limit_changed=False))
    commands = [dict(inference_frame=1008, ack_receipt_sequence=3,
        learned_control_proposed=True, held_between_camera_frames=False,
        owned_camera_history=dict(source_camera_frames=[dict(timestamp_ns=(index + 1) * 100_000_000,
            camera_sha256=[str(offset + 1) * 64 for offset in range(6)]) for index in range(5)]))]
    wall = [dict(ACK_sequence=index + 1) for index in range(904)]
    wall[2].update(inference_frame=1008, command_frame=1008, ACK_server_accepted=True,
        control_source="learned_trajectory_tracker", proposal_started_monotonic_ns=1_420_000_000,
        proposal_finished_monotonic_ns=1_475_000_000, ACK_started_monotonic_ns=1_479_000_000,
        ACK_finished_monotonic_ns=1_480_000_000,
        observed_bundle=dict(exact_bundle_delivery_monotonic_ns=1_414_000_000))
    groups = {}
    for mark in native:
        groups.setdefault(mark["tick_index"], {})[mark["stage"]] = mark
    diagnostic["native_pre_delivery_timing"]["full_native_rows"] = [p.native_interval_row(index, state, groups[index], index % 2 == 0)
        for index, state in enumerate(states)]
    diagnostic["native_pre_delivery_timing"]["all_fresh_critical_path_rows"] = [dict(native_frame=1008, ACK_sequence=3,
        first_callback_before_world_tick_started=False, last_callback_before_world_tick_returned=True,
        world_tick_return_minus_last_callback_seconds=.007, bundle_call_start_minus_last_callback_seconds=.013,
        last_callback_to_original_bundle_delivery_seconds=.014, original_bundle_delivery_to_input_ready_seconds=.016,
        actual_input_ready_to_NN_finished_seconds=.02, actual_NN_finished_to_decoded_plan_seconds=.02,
        first_callback_to_decoded_plan_seconds=.07, first_callback_to_ACK_seconds=.08)]
    pins = SimpleNamespace(values={str(path): "synthetic-only"}, json=lambda actual: receipt,
                           payload=lambda actual: json.dumps(receipt).encode(),
                           rows=lambda actual: states if actual.name == "states.jsonl" else wall)
    return diagnostic, pins, episode, cameras, commands


def test_measured_success_keeps_full_callbacks_and_fresh_plan_denominator(tmp_path):
    p.verify_callback_outcome(*timing_pass_fixture(tmp_path))


@pytest.mark.parametrize("mutation", ["unobserved", "count", "deadline", "maximum", "frame", "sequence"])
def test_measured_success_rejects_bad_actual_identity_or_deadline(tmp_path, mutation):
    diagnostic, pins, episode, cameras, commands = timing_pass_fixture(tmp_path)
    row = diagnostic["actual_callback_timing"]["rows"][0]
    if mutation == "unobserved":
        diagnostic["actual_six_camera_callback_observed"] = False
    elif mutation == "count":
        diagnostic["actual_callback_timing"]["fresh_NN_plan_count"] = 2
    elif mutation == "deadline":
        row["ACK_finished_monotonic_ns"] += 30_000_000
    elif mutation == "maximum":
        diagnostic["actual_callback_timing"]["metrics"]["callback_first_to_plan_seconds"]["maximum"] = .01
    elif mutation == "frame":
        commands[0]["inference_frame"] += 1
    else:
        commands[0]["ack_receipt_sequence"] += 1
    with pytest.raises(Exception):
        p.verify_callback_outcome(diagnostic, pins, episode, cameras, commands)


# HH_261004 - V2 cannot inherit a PASS while losing the new byte-bound/readback or archived-failure prerequisites.
@pytest.mark.parametrize("fault", ["archived", "missing", "SHA", "bytes", "maximum", "generic", "stage", "oversize"])
def test_v2_callback_readback_and_unchanged_failed_archive_are_required(tmp_path, fault):
    diagnostic, pins, episode, cameras, commands = timing_pass_fixture(tmp_path)
    receipt = diagnostic["callback_receipt_JSON_readback"]
    if fault == "archived": diagnostic["archived_callback45_v1_failure_preserved"] = False
    elif fault == "missing": diagnostic.pop("callback_receipt_JSON_readback")
    elif fault == "SHA": receipt["sha256"] = "0" * 64
    elif fault == "bytes": receipt["actual_bytes"] += 1
    elif fault == "maximum": receipt["maximum_bytes"] += 1
    elif fault == "generic": receipt["generic_JSON_reader_limit_changed"] = True
    elif fault == "stage":
        actual = json.loads(pins.payload(episode / "callback_timing_receipts.json"))
        actual["policy_stage_marks"] = [{}, {}]
        pins.payload = lambda path: json.dumps(actual).encode()
        receipt["actual_bytes"] = len(pins.payload(None))
    else: pins.payload = lambda path: b" " * (16 * 1024**2 + 1)
    with pytest.raises(Exception):
        p.verify_callback_outcome(diagnostic, pins, episode, cameras, commands)


def reseal_timing_payload(diagnostic, pins, episode):
    """HH_261004 - Re-pin a deliberately synthetic mutation so tests reach the intended semantic guard."""
    diagnostic["callback_receipt_JSON_readback"]["actual_bytes"] = len(pins.payload(episode / "callback_timing_receipts.json"))


def test_measured_FAIL_keeps_valid_maxima_and_all_failed_latency_tail(tmp_path):
    diagnostic, pins, episode, cameras, commands = timing_pass_fixture(tmp_path)
    row = diagnostic["actual_callback_timing"]["rows"][0]
    row["ACK_finished_monotonic_ns"] += 30_000_000
    row["callback_first_to_ACK_seconds"] = .11
    pins.rows(episode / "wall_timing.jsonl")[2]["ACK_finished_monotonic_ns"] = row["ACK_finished_monotonic_ns"]
    diagnostic["actual_callback_timing"]["metrics"]["callback_first_to_ACK_seconds"]["maximum"] = .11
    diagnostic["actual_callback_timing"]["fresh_callback_to_ACK_wall100ms_passed"] = False
    diagnostic["fresh_callback_to_ACK_wall100ms_passed"] = False
    diagnostic["native_pre_delivery_timing"]["all_fresh_critical_path_rows"][0]["first_callback_to_ACK_seconds"] = .11
    diagnostic["callback_straight_motion_body_policy_wall100ms_passed"] = False
    diagnostic["status"] = "FAIL_KNOWN_DEV_STRAIGHT_PRECURSOR"
    p.verify_callback_outcome(diagnostic, pins, episode, cameras, commands)
    assert row["callback_first_to_ACK_seconds"] == .11
    assert diagnostic["fresh_callback_to_ACK_wall100ms_passed"] is False


def test_later_native_FAILURE_does_not_call_valid_callback_maxima_unmeasured(tmp_path):
    diagnostic, pins, episode, cameras, commands = timing_pass_fixture(tmp_path)
    diagnostic.update(native_pre_delivery_stage_closure_passed=False,
        callback_straight_motion_body_policy_wall100ms_passed=False, status="FAIL_KNOWN_DEV_STRAIGHT_PRECURSOR")
    diagnostic.pop("native_pre_delivery_timing")
    diagnostic.pop("callback_receipt_JSON_readback")
    p.verify_callback_outcome(diagnostic, pins, episode, cameras, commands)
    bundle = dict(diagnostic=diagnostic, control_census=dict(observed_prior_learned_native_effect_count=1),
        states=[dict(timestamp=0), dict(timestamp=.05)], cameras=cameras)
    text = p.markdown(bundle, {})
    assert "70.000ms" in text and "80.000ms" in text
    assert "전체 PASS로 승격하지도 않습니다" in text


@pytest.mark.parametrize("fault", ["ambiguous_unconsumed_object", "actor", "pixels", "clock", "past_SHA",
    "past_object", "role_order", "stamp", "unbound"])
def test_exact_source_object_identity_and_consumed_history_are_required_for_measured_FAIL_or_PASS(tmp_path, fault):
    diagnostic, pins, episode, cameras, commands = timing_pass_fixture(tmp_path)
    actual = pins.json(episode / "callback_timing_receipts.json")
    if fault == "ambiguous_unconsumed_object": actual["callbacks"][-6]["image_object_id"] = actual["callbacks"][0]["image_object_id"]
    elif fault == "actor": actual["callbacks"][24]["actor_id"] = 99
    elif fault == "pixels": actual["callbacks"][24]["source_BGRA_encoding_prefixed_sha256"] = "a" * 64
    elif fault == "clock": diagnostic["actual_callback_timing"]["rows"][0]["first_callback_monotonic_ns"] += 1
    elif fault == "past_SHA": commands[0]["owned_camera_history"]["source_camera_frames"][0]["camera_sha256"][0] = "a" * 64
    elif fault == "past_object": actual["bundles"][0]["cameras"][0]["image_object_id"] = actual["callbacks"][-6]["image_object_id"]
    elif fault == "role_order": actual["bundles"][4]["cameras"].reverse()
    elif fault == "stamp": actual["callbacks"][24]["source_image_timestamp_simulation_ns"] += 1
    else: diagnostic["actual_callback_timing"]["exact_consumed_image_object_join_passed"] = False
    reseal_timing_payload(diagnostic, pins, episode)
    with pytest.raises(Exception):
        p.verify_callback_outcome(diagnostic, pins, episode, cameras, commands)


@pytest.mark.parametrize("fault", ["missing", "duplicate", "invented_frame", "phase", "clock", "changed_order",
    "full_row", "fresh_row", "delivery", "header", "denominator"])
def test_full7200_native_stage_closure_and_all_reported_intervals_are_required(tmp_path, fault):
    diagnostic, pins, episode, cameras, commands = timing_pass_fixture(tmp_path)
    actual = pins.json(episode / "callback_timing_receipts.json")
    if fault == "missing": actual["native_stage_marks"].pop()
    elif fault == "duplicate": actual["native_stage_marks"].append(dict(actual["native_stage_marks"][0]))
    elif fault == "invented_frame": actual["native_stage_marks"][0]["native_frame"] = 1000
    elif fault == "phase": actual["native_stage_marks"][0]["capture_phase"] = "driving"
    elif fault == "clock": actual["native_stage_marks"][1]["stage_finished_monotonic_ns"] = 1
    elif fault == "changed_order": actual["native_stage_marks"][0]["original_native_order_or_math_changed"] = True
    elif fault == "full_row": diagnostic["native_pre_delivery_timing"]["full_native_rows"][0]["world_tick_seconds"] += .001
    elif fault == "fresh_row": diagnostic["native_pre_delivery_timing"]["all_fresh_critical_path_rows"][0]["first_callback_to_ACK_seconds"] = .01
    elif fault == "delivery": pins.rows(episode / "wall_timing.jsonl")[2]["observed_bundle"]["exact_bundle_delivery_monotonic_ns"] = 1
    elif fault == "header": diagnostic["native_pre_delivery_timing"]["native_frame_invented_before_tick_return"] = True
    else: diagnostic["native_pre_delivery_timing"]["native_scalar_mark_count"] = 7199
    reseal_timing_payload(diagnostic, pins, episode)
    with pytest.raises(Exception):
        p.verify_callback_outcome(diagnostic, pins, episode, cameras, commands)


@pytest.mark.parametrize("key", ["selected_source_join_fields", "selected_source_exactly_one_original_callback_required",
    "causal_history_joins_consumed_bundle_identity_and_SHA_only", "earliest_or_latest_duplicate_receipt_selected_by_latency",
    "native_dispatch_timing"])
def test_fresh_v3_prospective_identity_and_native_contract_is_not_optional(archive_factory, key):
    value = archive_factory()
    plan = json.loads((value[3] / "declaration.json").read_bytes())
    plan["actual_callback_timing"].pop(key)
    with pytest.raises(Exception):
        p.verify_callback_contract(plan)


@pytest.mark.parametrize("fault", ["absent", "path", "status", "SHA", "source", "overwrite"])
def test_actual_frozen_v2_FAIL_and_sources_cannot_be_regraded_or_dropped(archive_factory, fault):
    value = archive_factory()
    plan = json.loads((value[3] / "declaration.json").read_bytes())
    if fault == "absent": plan.pop("failed_callback45_v2_reference")
    else:
        ref = plan["failed_callback45_v2_reference"]
        if fault == "path": ref["path"] += ".other"
        elif fault == "status": ref["status"] = "PASS_KNOWN_DEV_STRAIGHT_PRECURSOR"
        elif fault == "SHA": ref["sha256"] = {}
        elif fault == "source": ref["source_sha256"] = {}
        else: ref["overwritten"] = True
    with pytest.raises(Exception):
        p.verify_v2_failure_reference(plan, p.previous.Pins())
