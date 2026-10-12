"""HH_261004 - Test V3 prospective publication and topology/recording identity using only synthetic CPU evidence."""

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
from scripts.e2e import publish_temporal_route_joint_town03_straight_v3_20261004 as p


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
        run = root / "storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_v3"
        output = root / "docs/assets/validation/2026-10-04/portable_e2e_learning_cycle_v1/04_town03_known_dev_straight_v3"
        policy_path = root / ("storage/experiments/plans/2026-10-04/town03_known_dev_straight_v3/"
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
            observed_bundle_to_ACK_wall100ms_passed=False,
            all_failed_tail_retained=full, all_available_raw_files_retained_and_hashed=True,
            raw_before_after_unchanged=True,
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
    assert p.OWNER_SCHEMA.endswith(".v3")


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
