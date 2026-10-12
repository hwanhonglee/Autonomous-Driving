"""HH_261002 - Synthetic fixed-pair publication proof; never load actual weights, archives, Torch, or CARLA."""
import ast
import copy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import publish_temporal_intrinsic_town03_right_20261002 as p


spec = importlib.util.spec_from_file_location("_town03_frozen_publication_test_helpers",
    Path(__file__).with_name("test_publish_temporal_route_joint_town03_right_20261002.py"))
old = importlib.util.module_from_spec(spec); spec.loader.exec_module(old)
write = old.write
NAMES = ["_blank_brake_control", "checked_agent_initialization_observation", "apply_spawn_z_offset",
    "AcknowledgedControlTransport", "exact_camera_bundle", "_save_jpeg"]


def refresh(path, caller, run):
    # HH_261002 - Reseal synthetic caller bytes only; never refresh a real source pin.
    plan_sha = p.previous.prior.digest(run/"declaration.json")
    owner = json.loads((run/"owner_result.json").read_bytes()); owner["declaration_sha256"] = plan_sha
    write(run/"owner_result.json", owner)
    diag = json.loads((run/"diagnostic_report.json").read_bytes())
    diag.update(original_owner=owner,
        raw_source_sha256={str(f): p.previous.prior.digest(f) for f in (run/"episode").rglob("*") if f.is_file()})
    write(run/"diagnostic_report.json", diag)
    caller["run_files_sha256"] = {name: p.previous.prior.digest(run/name) for name in p.previous.inventory(run)}
    return write(path, caller)


@pytest.fixture
def archive(tmp_path, monkeypatch):
    path, _, caller, old_run = old.archive.__wrapped__(tmp_path, monkeypatch)
    workspace = p.previous.ROOT
    pair_root = workspace/"storage/experiments/validation/2026-10-02/temporal_intrinsic_pair_town03_known_dev_right_v1"
    pair_root.mkdir(); run = pair_root/"control"; old_run.rename(run)
    monkeypatch.setattr(p, "PAIR_ROOT", pair_root); monkeypatch.setattr(p, "OUTPUT", workspace/"docs/actual_pair")
    # HH_261002 - Preserve exclusive frozen provider identity; the replacement closure is synthetic and empty.
    monkeypatch.setattr(p.previous, "publication_sources", p._sources)
    monkeypatch.setattr(p, "publication_sources", lambda: set())
    authority_path = workspace/"storage/experiments/plans/completed_pair.json"
    endpoints = {arm: dict(weights_sha256=("1" if arm == "control" else "2")*64,
        metadata_sha256="3"*64, checkpoint_sha256="4"*64, declaration_sha256="5"*64) for arm in p.ARMS}
    authority = dict(schema=p.AUTHORITY_SCHEMA, mode="RESEARCH_ONLY", endpoints=endpoints,
        both_fixed_endpoints_completed=True, arm_order=list(p.ARMS), model_promoted=False, vehicle_control_approved=False)
    authority_sha = write(authority_path, authority)
    write(run/"source"/authority_path.relative_to(workspace), authority_path.read_bytes())
    plan = json.loads((run/"declaration.json").read_bytes())
    plan.update(schema=p.OWNER_SCHEMA, output=str(run), arm="control", paired_arm_order=list(p.ARMS),
        both_pair_endpoints_completed_before_declaration=True, known_total_optimizer_updates=18200,
        completed_pair_authority=str(authority_path), completed_pair_authority_sha256=authority_sha,
        source_checkpoint_sha256=endpoints["control"]["checkpoint_sha256"],
        new_fixed_endpoint_export_sha256=dict(weights=endpoints["control"]["weights_sha256"], metadata=endpoints["control"]["metadata_sha256"]),
        asset_sha256={str(authority_path): authority_sha})
    write(run/"declaration.json", plan)
    diag = json.loads((run/"diagnostic_report.json").read_bytes()); diag["schema"] = p.OWNER_SCHEMA+".diagnostic_report"
    write(run/"diagnostic_report.json", diag)
    cameras = [json.loads(line) for line in (run/"episode/camera_frames.jsonl").read_bytes().splitlines()]
    rows = [dict(schema="portable_e2e.exact_camera_recording_after_ACK.20261001.v1", frame=camera["frame"],
        enqueued_camera_count=6, written_camera_count=6,
        written_files=[str(run/"episode"/f) for f in camera["images"].values()], all_queued_images_written=True,
        failure=None, no_source_image_dropped=True, source_timestamp_changed=False, source_pixels_changed=False,
        acknowledged_before_recording=True, ACK_receipt_sequence=2+2*i,
        recording_started_monotonic_ns=100+i*10, recording_finished_monotonic_ns=105+i*10,
        recording_seconds=5.e-9) for i, camera in enumerate(cameras)]
    write(run/"episode/recording_after_ack.jsonl", b"\n".join(json.dumps(r).encode() for r in rows))
    write(run/"episode/actual_recording_binding.json", dict(
        schema="portable_e2e.actual_intrinsic_pair_town03_known_dev_right.20261002.v1.actual_recording_binding",
        copied_provider_names=NAMES, all_copied_mutable_providers_current=True, enqueue_and_ACK_share_exact_recorder=True,
        actual_recording_bundle_count=3, pending_source_camera_count=0, actual_after_ACK_receipt_count=3,
        every_recording_event_preserved=True, source_image_transformed=False, exposure_to_ACK_latency_measured=False))
    caller.update(schema=p.CALLER_SCHEMA, arm="control", run_root=str(run), output=str(p.output_for(run)),
        completed_pair_authority_sha256=authority_sha)
    return path, refresh(path, caller, run), caller, run


def load(archive):
    with p.activated(): return p.load_archive(*archive[:2])


def edit(archive, relative, function):
    path, _, caller, run = archive; value = json.loads((run/relative).read_bytes()); function(value)
    write(run/relative, value); return path, refresh(path, caller, run), caller, run


def edit_rows(archive, function):
    path, _, caller, run = archive; source = run/"episode/recording_after_ack.jsonl"
    rows = [json.loads(line) for line in source.read_bytes().splitlines()]; function(rows)
    write(source, b"\n".join(json.dumps(row).encode() for row in rows))
    return path, refresh(path, caller, run), caller, run


def test_complete_control_archive_keeps_failure_full_native_and_six_camera_denominators(archive):
    _, bundle, pins = load(archive)
    assert bundle["paired_arm"] == "control" and len(bundle["states"]) == 6 and len(bundle["cameras"]) == 3
    assert bundle["actual_recording_census"] == dict(bundle_count=3, after_ACK_count=3,
        all_recording_after_ACK=True, failed_pending_final_bundle_flushed_without_ACK=False,
        full_tick_10Hz_wall_rate_claimed=False)
    assert bundle["diagnostic"]["joint_turn_motion_body_policy_wall100ms_passed"] is False
    assert bundle["control_census"]["observed_prior_learned_native_effect_count"] == 4
    pins.recheck()


@pytest.mark.parametrize("arm", p.ARMS)
def test_exact_output_arm_mapping_is_not_outcome_selected(archive, arm):
    root = archive[3].parent
    assert p.output_for(root/arm) == p.OUTPUT/arm


def test_intrinsic_arm_reads_its_own_exact_endpoint_not_control(archive):
    path, _, caller, original = archive; run = original.with_name("intrinsic_curve"); original.rename(run)
    plan = json.loads((run/"declaration.json").read_bytes())
    authority = json.loads(Path(plan["completed_pair_authority"]).read_bytes()); endpoint = authority["endpoints"][run.name]
    plan.update(output=str(run), arm=run.name, source_checkpoint_sha256=endpoint["checkpoint_sha256"],
        new_fixed_endpoint_export_sha256=dict(weights=endpoint["weights_sha256"], metadata=endpoint["metadata_sha256"]))
    write(run/"declaration.json", plan)
    source = run/"episode/recording_after_ack.jsonl"; rows = [json.loads(line) for line in source.read_bytes().splitlines()]
    for row in rows: row["written_files"] = [str(run/Path(f).relative_to(original)) for f in row["written_files"]]
    write(source, b"\n".join(json.dumps(row).encode() for row in rows))
    caller.update(run_root=str(run), output=str(p.output_for(run)), arm=run.name)
    _, bundle, _ = load((path, refresh(path, caller, run), caller, run))
    assert bundle["paired_arm"] == "intrinsic_curve" and bundle["selected_endpoint"]["weights_sha256"] == "2"*64


@pytest.mark.parametrize("kind", ["unknown", "wrong_parent", "symlink", "relative"])
def test_unknown_cross_parent_and_symlink_archive_cannot_select_output(archive, kind):
    run = archive[3]
    if kind == "unknown": target = run.with_name("best_checkpoint")
    elif kind == "wrong_parent": target = run.parent.parent/"control"
    elif kind == "relative": target = Path("control")
    else:
        target = run.with_name("alias"); target.symlink_to(run, target_is_directory=True)
    with pytest.raises(Exception): p.output_for(target)


@pytest.mark.parametrize("kind", ["arm", "order", "incomplete", "updates", "checkpoint", "weights", "old_schema"])
def test_declared_selected_endpoint_cannot_hide_wrong_lineage(archive, kind):
    def corrupt(plan):
        if kind == "arm": plan["arm"] = "intrinsic_curve"
        elif kind == "order": plan["paired_arm_order"].reverse()
        elif kind == "incomplete": plan["both_pair_endpoints_completed_before_declaration"] = False
        elif kind == "updates": plan["known_total_optimizer_updates"] = 12200
        elif kind == "checkpoint": plan["source_checkpoint_sha256"] = "a"*64
        elif kind == "weights": plan["new_fixed_endpoint_export_sha256"]["weights"] = "a"*64
        else: plan["schema"] = "portable_e2e.owned_route_joint_town03_known_dev_right.20261002.v3"
    with pytest.raises(Exception): load(edit(archive, "declaration.json", corrupt))


@pytest.mark.parametrize("kind", ["incomplete", "promotion", "approval", "order", "authority_sha"])
def test_root_completed_pair_authority_stays_exact_and_denied(archive, kind):
    path, _, caller, run = archive
    plan = json.loads((run/"declaration.json").read_bytes()); authority_path = Path(plan["completed_pair_authority"])
    value = json.loads(authority_path.read_bytes())
    if kind == "incomplete": value["both_fixed_endpoints_completed"] = False
    elif kind == "promotion": value["model_promoted"] = True
    elif kind == "approval": value["vehicle_control_approved"] = True
    elif kind == "order": value["arm_order"].reverse()
    else:
        caller["completed_pair_authority_sha256"] = "a"*64
        with pytest.raises(Exception): load((path, write(path, caller), caller, run))
        return
    expected = write(authority_path, value); write(run/"source"/authority_path.relative_to(p.previous.ROOT), authority_path.read_bytes())
    plan.update(completed_pair_authority_sha256=expected, asset_sha256={str(authority_path): expected})
    write(run/"declaration.json", plan); caller["completed_pair_authority_sha256"] = expected
    with pytest.raises(Exception): load((path, refresh(path, caller, run), caller, run))


@pytest.mark.parametrize("key", ["ignore_traffic_lights", "frozen", "all_original_hazard_detectors_and_emergency_returns_unchanged",
    "closed_course_basic_turn_only_not_traffic_rule_performance", "actual_manual"])
def test_green_fixture_and_manual_setup_do_not_become_traffic_rule_approval(archive, key):
    def corrupt(plan):
        if key == "actual_manual": plan["actual_manual_first_gear_bootstrap_and_engagement_required"] = False
        else: plan["owned_green_signal_fixture"][key] = not plan["owned_green_signal_fixture"][key]
    with pytest.raises(Exception): load(edit(archive, "declaration.json", corrupt))


@pytest.mark.parametrize("kind", ["schema", "frame", "missing", "extra", "pixels", "timestamps", "dropped", "failure", "ACK", "invented_preACK"])
def test_whole_recording_source_and_actual_ACK_binding_fail_closed(archive, kind):
    def corrupt(rows):
        row = rows[-1]
        if kind == "schema": row["schema"] = "unknown"
        elif kind == "frame": row["frame"] += 1
        elif kind == "missing": row["written_files"].pop()
        elif kind == "extra": row["written_files"].append(row["written_files"][0])
        elif kind == "pixels": row["source_pixels_changed"] = True
        elif kind == "timestamps": row["source_timestamp_changed"] = True
        elif kind == "dropped": row["no_source_image_dropped"] = False
        elif kind == "failure": row["failure"] = "encoding failed"
        elif kind == "ACK": row["ACK_receipt_sequence"] = 999
        else: row["acknowledged_before_recording"] = False
    with pytest.raises(Exception): load(edit_rows(archive, corrupt))


@pytest.mark.parametrize("kind", ["schema", "provider", "shared", "count", "pending", "ACKcount", "names", "all_events", "bool_count"])
def test_actual_provider_proof_requires_complete_real_binding(archive, kind):
    def corrupt(proof):
        if kind == "schema": proof["schema"] = "old"
        elif kind == "provider": proof["all_copied_mutable_providers_current"] = False
        elif kind == "shared": proof["enqueue_and_ACK_share_exact_recorder"] = False
        elif kind == "count": proof["actual_recording_bundle_count"] -= 1
        elif kind == "pending": proof["pending_source_camera_count"] = 6
        elif kind == "ACKcount": proof["actual_after_ACK_receipt_count"] -= 1
        elif kind == "names": proof["copied_provider_names"][-1] = "obsolete_save_jpeg"
        elif kind == "all_events": proof["every_recording_event_preserved"] = False
        else: proof["pending_source_camera_count"] = False
    with pytest.raises(Exception): load(edit(archive, "episode/actual_recording_binding.json", corrupt))


def test_last_failed_preACK_row_is_preserved_without_fictional_ACK(archive):
    archive = edit_rows(archive, lambda rows: rows[-1].update(acknowledged_before_recording=False, ACK_receipt_sequence=None))
    archive = edit(archive, "episode/actual_recording_binding.json", lambda proof: proof.update(actual_after_ACK_receipt_count=2))
    _, bundle, _ = load(archive)
    assert len(bundle["cameras"]) == 3 and bundle["actual_recording_census"]["after_ACK_count"] == 2
    assert bundle["actual_recording_census"]["all_recording_after_ACK"] is False
    assert bundle["diagnostic"]["joint_turn_motion_body_policy_wall100ms_passed"] is False
    assert len(bundle["receipts"]) == 7


def pending_final_archive(archive):
    archive = edit_rows(archive, lambda rows: rows[-1].update(acknowledged_before_recording=False, ACK_receipt_sequence=None))
    return edit(archive, "episode/actual_recording_binding.json", lambda proof:
        proof.update(actual_recording_bundle_count=2, pending_source_camera_count=6, actual_after_ACK_receipt_count=2))


def test_real_pre_send_failure_outer_flush_preserves_last_bundle_and_original_failure(archive):
    _, bundle, _ = load(pending_final_archive(archive))
    assert len(bundle["cameras"]) == 3 and bundle["actual_recording_census"]["after_ACK_count"] == 2
    assert bundle["actual_recording_census"]["failed_pending_final_bundle_flushed_without_ACK"] is True
    assert bundle["actual_recording_census"]["all_recording_after_ACK"] is False
    assert bundle["diagnostic"]["original_capture_status"] == "failed"
    assert bundle["diagnostic"]["joint_turn_motion_body_policy_wall100ms_passed"] is False


@pytest.mark.parametrize("kind", ["success", "partial_pending", "wrong_count", "last_ACK", "last_invented_ACK", "short_bundle"])
def test_pending_final_exception_cannot_relax_success_or_drop_sources(archive, kind):
    value = pending_final_archive(archive)
    if kind == "success":
        value = edit(value, "episode/manifest.json", lambda m: m.update(status="complete"))
        value = edit(value, "diagnostic_report.json", lambda d: d.update(original_capture_status="complete"))
    elif kind == "partial_pending": value = edit(value, "episode/actual_recording_binding.json", lambda proof: proof.update(pending_source_camera_count=5))
    elif kind == "wrong_count": value = edit(value, "episode/actual_recording_binding.json", lambda proof: proof.update(actual_recording_bundle_count=1))
    elif kind == "last_ACK": value = edit_rows(value, lambda rows: rows[-1].update(acknowledged_before_recording=True, ACK_receipt_sequence=6))
    elif kind == "last_invented_ACK": value = edit_rows(value, lambda rows: rows[-1].update(ACK_receipt_sequence=6))
    else: value = edit_rows(value, lambda rows: rows[-1]["written_files"].pop())
    with pytest.raises(Exception): load(value)


def test_reader_clone_changes_only_owner_schema_assignment(archive, monkeypatch):
    compiled = []
    def collect_compile(tree, filename, mode):
        compiled.append(copy.deepcopy(tree)); return compile(tree, filename, mode)
    monkeypatch.setattr(p, "compile", collect_compile, raising=False)
    with p.activated(): clone = p.archive_function()
    original = next(n for n in ast.parse((p.ROOT/"tools/e2e/publish_temporal_route_joint_town03_right_20261002.py").read_bytes()).body
        if isinstance(n, ast.FunctionDef) and n.name == "load_archive")
    assert len(compiled) == 1
    changed = compiled[0].body[0]; count = 0
    for node in ast.walk(changed):
        if isinstance(node, ast.Assign) and ast.unparse(node.targets[0]) == "owner_schema":
            assert ast.unparse(node.value) == "OWNER_SCHEMA"
            node.value = ast.parse("OWNER_SCHEMA[:-1] + run.name[-1]", mode="eval").body; count += 1
    assert count == 1 and ast.dump(changed, include_attributes=False) == ast.dump(original, include_attributes=False)
    assert clone.__globals__ is not vars(p.previous)


def test_inverse_AST_guard_rejects_another_reader_statement_change(archive, monkeypatch):
    deepcopy = p.copy.deepcopy; calls = 0
    def corrupt_first_copy(node):
        nonlocal calls
        result = deepcopy(node); calls += 1
        if calls == 1: result.body.append(ast.parse("unexpected_guard = False").body[0])
        return result
    monkeypatch.setattr(p.copy, "deepcopy", corrupt_first_copy)
    with p.activated():
        with pytest.raises(Exception, match="another archive"): p.archive_function()


def test_scope_restores_every_original_provider_on_exception(archive):
    keys = ("SCHEMA", "CALLER_SCHEMA", "OWNER_SCHEMA", "output_for", "publication_sources", "load_archive", "render", "markdown", "publish")
    saved = {k: getattr(p.previous, k) for k in keys}
    with pytest.raises(RuntimeError):
        with p.activated():
            assert p.previous.load_archive is p.load_archive
            raise RuntimeError("synthetic failure")
    assert {k: getattr(p.previous, k) for k in keys} == saved and p._active is False and p._loaded is None


def test_nested_and_unknown_providers_are_rejected_and_restored(archive, monkeypatch):
    with p.activated():
        with pytest.raises(Exception):
            with p.activated(): pass
    monkeypatch.setattr(p.previous, "render", lambda *_: None)
    with pytest.raises(Exception):
        with p.activated(): pass
    assert p._active is False


def test_postread_original_mutation_is_not_a_new_truth(archive):
    _, _, pins = load(archive); write(archive[3]/"episode/actual_recording_binding.json", {})
    with pytest.raises(Exception): pins.recheck()


def test_validate_only_uses_full_reader_without_creating_companion_or_media(archive, capsys):
    p.main(["--caller-pins", str(archive[0]), "--caller-pins-sha256", archive[1], "--validate-only"])
    result = json.loads(capsys.readouterr().out)
    assert result["validated"] is True and result["native_count"] == 6 and result["camera_count"] == 3
    assert result["output_created"] is False and not p.OUTPUT.exists()
    assert p._loaded is None


def test_full_synthetic_publication_adds_only_separate_pair_companion(archive, monkeypatch, capsys):
    monkeypatch.setattr(p.previous.shutil, "disk_usage", lambda _: SimpleNamespace(free=9*1024**3))
    p.main(["--caller-pins", str(archive[0]), "--caller-pins-sha256", archive[1]])
    result = json.loads(capsys.readouterr().out); output = p.output_for(archive[3])
    assert result["GIF_count"] == 1 and result["PNG_count"] == 5
    evidence = json.loads((output/"pair_and_recording_evidence.json").read_bytes())
    provenance = json.loads((output/"publication_provenance.json").read_bytes())
    assert evidence["arm"] == "control" and evidence["endpoint"]["weights_sha256"] == "1"*64
    assert evidence["base_publication_provenance_sha256"] == p.previous.prior.digest(output/"publication_provenance.json")
    assert evidence["actual_recording_census"]["after_ACK_count"] == 3
    assert evidence["model_promoted"] is evidence["vehicle_control_approved"] is False
    assert evidence["official_main_features_approved"] == evidence["official_subfeatures_approved"] == 0
    assert provenance["original_diagnostic"]["joint_turn_motion_body_policy_wall100ms_passed"] is False
    assert provenance["failed_tail_cropped"] is False and provenance["GIF_total_duration_ms"] == 300
    assert "pair_and_recording_evidence.json" not in provenance["media_sha256"] and len(provenance["media_sha256"]) == 6
    assert p._loaded is None and p._active is False


def test_post_reader_publication_failure_clears_cached_pinned_bundle(archive, monkeypatch):
    monkeypatch.setattr(p.previous.shutil, "disk_usage", lambda _: SimpleNamespace(free=1))
    with pytest.raises(Exception): p.main(["--caller-pins", str(archive[0]), "--caller-pins-sha256", archive[1]])
    assert p._loaded is None and p._active is False and not p.OUTPUT.exists()


def test_same_reader_checks_reject_model_or_simulator_process(archive, monkeypatch):
    monkeypatch.setitem(p.previous.sys.modules, "torch", object())
    with pytest.raises(Exception): load(archive)


def test_existing_publication_is_never_overwritten(archive):
    output = p.output_for(archive[3]); output.mkdir(parents=True); sentinel = output/"user.txt"; sentinel.write_text("preserve")
    with pytest.raises(Exception): p.main(["--caller-pins", str(archive[0]), "--caller-pins-sha256", archive[1], "--validate-only"])
    assert sentinel.read_text() == "preserve"
