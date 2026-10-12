"""HH_260906 - Preserve all eight cases and reject altered source bindings before creating public outputs."""

import copy
import hashlib
import json
from pathlib import Path

import pytest

from scripts.e2e import curate_carla_turn_launch_matrix as c


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(c.encode(value))


def pin(path, root):
    return {"path": str(path.relative_to(root)), "sha256": c.sha(path), "size_bytes": path.stat().st_size}


@pytest.fixture
def inputs(tmp_path, monkeypatch):
    campaign, audit_root, output = [tmp_path / name for name in ("campaign", "audit", "publication")]
    write(campaign / "pilot_plan.json", {"scope": "unchanged"})
    report = {"schema": "portable_e2e.turn_launch_matrix_audit.v1", "status": "AUDITED_NOT_ADMITTED",
        "planned_cases": 8, "discovered_cases": 8, "finalized_cases": 8, "not_run_cases": 0, "all_planned_cases_retained": True,
        "reviewed_execution_commit": c.audit.COMMIT, "prospective_plan_sha256": c.audit.ORIGINAL_PLAN_SHA,
        "continuation_reviews": [{}] * 7, "resume_authorization": {}, "audit_source_sha256": {},
        "source_manifest": [pin(campaign / "pilot_plan.json", campaign)], "historical_source_manifest": [], "cases": [],
        **{key: False for key in ("training_data_approved", "dataset_admission", "automatic_winner_selection",
            "full_future_xy_admission", "physical_actuation_proven", "repetitions_are_independent_routes")}}
    fresh = {}
    for expected in c.audit.expected_cases():
        root = campaign / expected["output"]; write(root / "meta.json", {"unchanged": True})
        image = root / "episode/images/front.jpg"; image.parent.mkdir(parents=True); image.write_bytes(b"actual JPEG placeholder")
        images = [pin(image, root)]
        clear = expected["profile"] == "turn_launch_013_v1"
        rate = {"minimum_mps2": -1., "maximum_mps2": 2. if clear else 3.5, "physical_decoder": {"violation_count": 0 if clear else 1}}
        qa = {"raw_scalar_quality_clear": clear, "phase_counts": {"driving": {"native_states": 2, "camera_anchors": 1}},
            "speed_rate_qa": {key: {"by_phase": {"all": copy.deepcopy(rate)}} for key in ("native_20hz", "camera_10hz")},
            "final_driving": {"goal_error_m": .8}, "goal_dwell_seconds": 2.}
        trial = {"independent_qa": qa, "handoff_diagnostic": None, "pilot_protocol": {"all_checks_pass": True},
            "source_manifest": [pin(root / "meta.json", root)], "image_byte_integrity": {"all_images_decoded": True,
                "content_or_visual_quality_approved": False, "file_count": 1, "total_size_bytes": image.stat().st_size,
                "canonical_sorted_file_ledger_sha256": hashlib.sha256(json.dumps(images, sort_keys=True, separators=(",", ":")).encode()).hexdigest()}}
        report["cases"].append({**expected, "status": "FINALIZED", "audit": trial, "independent_scalar_quality_clear": clear})
        fresh[str(root)] = copy.deepcopy(qa)
        write(audit_root / f"{expected['profile']}_run_{expected['replicate']:03d}_image_hashes.json", images)
    write(audit_root / "audit.json", report)
    monkeypatch.setattr(c, "AUDIT_SHA", c.sha(audit_root / "audit.json"))
    monkeypatch.setattr(c, "source_pins", lambda report: {})
    monkeypatch.setattr(c.audit.base, "summarize_trial", lambda root, _: ({"independent_qa": fresh[str(root)]}, {}))
    monkeypatch.setattr(c.audit, "handoff_diagnostic", lambda *args: None)
    monkeypatch.setattr(c.shutil, "which", lambda _: "/existing/ffmpeg")
    def render(item, destination):
        destination.mkdir(); (destination / "actual.png").write_bytes(b"PNG original bytes")
        return {"selection": "fixed", "training_data_approved": False}
    monkeypatch.setattr(c, "render_case", render)
    monkeypatch.setattr(c, "render_plots", lambda *args: None)
    return campaign, audit_root, output, report, fresh


def changed_report(inputs, monkeypatch):
    write(inputs[1] / "audit.json", inputs[3])
    monkeypatch.setattr(c, "AUDIT_SHA", c.sha(inputs[1] / "audit.json"))


def test_all_cases_preserved_with_exact_hash_inventory(inputs):
    campaign, audit_root, output, _, _ = inputs
    result = c.publish(campaign, audit_root, output)
    assert len(result["cases"]) == 8 and result["scalar_pass_count"] == 2
    assert result["native_states"] == 16 and result["camera_anchors"] == result["jpeg_count"] == 8
    assert not result["training_data_approved"] and not result["automatic_winner_selection"]
    assert (output / "audit.json").read_bytes() == (audit_root / "audit.json").read_bytes()
    for case in result["cases"]: assert (output / case["case_id"] / "actual.png").read_bytes() == b"PNG original bytes"
    entries = [line.split("  ") for line in (output / "SHA256SUMS").read_text().splitlines()]
    assert {name for _, name in entries} == {str(p.relative_to(output)) for p in output.rglob("*") if p.is_file()} - {"SHA256SUMS"}
    for digest, name in entries: assert c.sha(output / name) == digest
    assert b"/home/" not in (output / "publication_manifest.json").read_bytes()


def test_stale_reviewed_audit_rejected_before_output(inputs):
    write(inputs[1] / "audit.json", {"different": True})
    with pytest.raises(ValueError, match="audit SHA"): c.publish(*inputs[:3])
    assert not inputs[2].exists()


@pytest.mark.parametrize("key,value", [("status", "INCOMPLETE"), ("finalized_cases", 7), ("training_data_approved", True),
    ("automatic_winner_selection", True), ("repetitions_are_independent_routes", True), ("continuation_reviews", []),
    ("resume_authorization", None)])
def test_partial_promotion_and_incomplete_continuation_rejected(inputs, monkeypatch, key, value):
    inputs[3][key] = value; changed_report(inputs, monkeypatch)
    with pytest.raises(ValueError): c.publish(*inputs[:3])
    assert not inputs[2].exists()


def test_case_order_cannot_be_sorted_by_pass(inputs, monkeypatch):
    inputs[3]["cases"].reverse(); changed_report(inputs, monkeypatch)
    with pytest.raises(ValueError, match="case order"): c.publish(*inputs[:3])


def test_mutated_scalar_cannot_replace_fresh_measurements(inputs, monkeypatch):
    inputs[3]["cases"][0]["audit"]["independent_qa"]["raw_scalar_quality_clear"] = True
    changed_report(inputs, monkeypatch)
    with pytest.raises(ValueError, match="recomputation"): c.publish(*inputs[:3])


@pytest.mark.parametrize("which", ["metadata", "JPEG", "image_ledger", "audit_source"])
def test_changed_bound_inputs_rejected(inputs, which):
    campaign, audit_root, output, report, _ = inputs
    root = campaign / report["cases"][0]["output"]
    if which == "metadata": write(root / "meta.json", {"different": True})
    elif which == "JPEG": (root / "episode/images/front.jpg").write_bytes(b"modified JPEG")
    elif which == "image_ledger": write(audit_root / "turn_launch_015_v1_run_001_image_hashes.json", [])
    else: (campaign / "pilot_plan.json").write_bytes(b"mutated plan")
    with pytest.raises(ValueError): c.publish(campaign, audit_root, output)
    assert not output.exists()


def test_late_mutation_blocks_final_success_manifest(inputs, monkeypatch):
    monkeypatch.setattr(c, "render_plots", lambda *args: (inputs[0] / "pilot_plan.json").write_bytes(b"changed during render"))
    with pytest.raises(ValueError): c.publish(*inputs[:3])
    assert not (inputs[2] / "publication_manifest.json").exists()
    assert not (inputs[2] / "SHA256SUMS").exists()


@pytest.mark.parametrize("kind", ["existing", "inside_raw", "symlink"])
def test_output_is_create_only_and_outside_inputs(inputs, kind):
    output = inputs[2]
    if kind == "existing": output.mkdir()
    elif kind == "inside_raw": output = inputs[0] / "new_publication"
    else: output.symlink_to(inputs[0], target_is_directory=True)
    with pytest.raises(ValueError): c.publish(inputs[0], inputs[1], output)


def selection_data():
    states = [{"frame": i, "timestamp": i * .1, "capture_phase": "stationary_warmup" if i < 2 else "driving",
        "route_progress_m": i * .1, "goal_stop": {"complete": i >= 92}} for i in range(100)]
    return {"states": states, "frames": {r["frame"]: i for i, r in enumerate(states)},
        "cameras": [{"frame": i} for i in range(100)], "route": {"route": [
            {"road_option": "LANEFOLLOW", "distance_m": 0}, {"road_option": "LEFT", "distance_m": 3},
            {"road_option": "LEFT", "distance_m": 5}, {"road_option": "LANEFOLLOW", "distance_m": 9}]}}


def test_fixed_actual_selections_and_launch_only_stride():
    result = c.select_views(selection_data())
    assert result["snapshots"] == {"01_first_driving": 2, "02_catalog_left_midpoint": 40, "03_first_goal_complete": 92}
    assert result["launch_camera_indices"] == list(range(2, 83, 2))
    assert result["launch_window_seconds"] == 8 and result["nominal_preview_speedup"] == 2


def test_unreached_turn_or_goal_not_replaced_by_nearest_start_or_last():
    data = selection_data(); data["states"] = data["states"][:10]; data["cameras"] = data["cameras"][:10]
    result = c.select_views(data)
    assert result["snapshots"]["02_catalog_left_midpoint"] is None and result["snapshots"]["03_first_goal_complete"] is None
    assert not result["catalog_left_reached"] and not result["goal_complete_camera_available"]


def test_multiple_left_segments_cannot_select_an_ambiguous_midpoint():
    data = selection_data(); data["route"]["route"].append({"road_option": "LEFT", "distance_m": 12})
    with pytest.raises(ValueError, match="contiguous"): c.select_views(data)


def test_actual_auditor_source_pin_and_source_mutation(tmp_path, monkeypatch):
    with pytest.raises(ValueError, match="unreviewed"): c.source_pins({"audit_source_sha256": {}})
    file = tmp_path / "source.py"; file.write_bytes(b"reviewed")
    monkeypatch.setattr(c, "ROOT", tmp_path)
    pins = {"source.py": c.sha(file)}; c.verify_sources(pins)
    file.write_bytes(b"changed")
    with pytest.raises(ValueError, match="source changed"): c.verify_sources(pins)


def test_overlapping_reviewed_dependency_is_not_replaced_by_current_hash():
    report = {"audit_source_sha256": {"scripts/e2e/audit_carla_turn_launch_matrix.py": c.AUDITOR_SHA,
        str(Path(c.audit.v4.__file__).resolve().relative_to(c.ROOT)): "0" * 64}}
    with pytest.raises(ValueError, match="overlapping source"): c.source_pins(report)


def test_path_escape_or_duplicate_ledger_rejected(tmp_path):
    with pytest.raises(ValueError, match="unsafe"): c.safe_file(tmp_path, "../source")
    path = tmp_path / "x"; path.write_bytes(b"original"); entry = pin(path, tmp_path)
    with pytest.raises(ValueError, match="duplicate"): c.verify_entries(tmp_path, [entry, entry])


def test_displayed_images_bind_absolute_renderer_to_relative_cli(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    image = tmp_path / "campaign/run/episode/images/front.jpg"
    image.parent.mkdir(parents=True); image.write_bytes(b"real camera")
    item = {"root": Path("campaign/run"), "images": [pin(image, tmp_path / "campaign/run")]}
    data = {"episode": image.parent.parent, "displayed_image_sha256": {"images/front.jpg": c.sha(image)}}
    c.verify_displayed_images(data, item)
    data["displayed_image_sha256"]["images/front.jpg"] = "0" * 64
    with pytest.raises(ValueError, match="displayed camera"): c.verify_displayed_images(data, item)


def test_cli_abbreviation_disabled():
    with pytest.raises(SystemExit): c.main(["--campaign", "example"])
