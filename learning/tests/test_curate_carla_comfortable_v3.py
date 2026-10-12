"""HH_260906 - Reject stale audit or visual bindings before creating a new evidence category."""

import copy
import json
from pathlib import Path

import pytest

from scripts.e2e import curate_carla_comfortable_v3 as curate


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(curate.encode(value))


@pytest.fixture
def inputs(tmp_path, monkeypatch):
    pilot, visuals = tmp_path / "pilot", tmp_path / "visuals"
    results = []
    for i in (1, 2):
        name = f"run_{i:03}"
        (pilot / "town07_straight_calibration" / name).mkdir(parents=True)
        results.append(dict(trial_id=name, source_manifest=[], development_screen_clear=False, raw_quality_candidate=i == 2,
            private_path="/home/example/personal/hwanhong/portable_e2e/runs/example"))
    write(pilot / "pilot_plan.json", {"profile": "comfortable_v3"})
    report = dict(schema="portable_e2e.comfortable_v3_independent_audit.v1", trial_count=2, trials=copy.deepcopy(results),
        auditor_source_sha256=curate.audit.base.sha(Path(curate.audit.__file__)),
        base_auditor_source_sha256=curate.audit.base.sha(Path(curate.audit.base.__file__)),
        pilot_plan_source={"sha256": curate.audit.base.sha(pilot / "pilot_plan.json")},
        development_screen_clear_count=0, development_screen_fail_count=2, base_raw_quality_candidate_count=1)
    report_path = tmp_path / "private_report/summary.json"
    write(report_path, report)
    monkeypatch.setattr(curate.audit, "audit_trial", lambda p: copy.deepcopy(results[int(p.name[-1]) - 1]))
    monkeypatch.setattr(curate, "bind_visuals", lambda *args: ({"frame.png": b"\x89PNG\r\n\x1a\nunchanged"}, {}))
    monkeypatch.setattr(curate, "readme", lambda report: b"Both attempts; no admission.\n")
    return pilot, report_path, visuals, tmp_path / "published", results


def publish(inputs):
    pilot, report, visuals, output, _ = inputs
    return curate.curate(pilot, report, visuals, output, curate.audit.base.sha(report))


def test_publication_redacts_metadata_preserves_visual_bytes_and_binds_original_sha(inputs):
    result = publish(inputs)
    _, raw, _, output, _ = inputs
    published = json.loads((output / "summary.json").read_text())
    assert "/home/" not in json.dumps(published)
    assert published["raw_source_sha256"] == curate.audit.base.sha(raw)
    assert result["raw_audit_sha256"] == curate.audit.base.sha(raw)
    assert not result["training_data_approved"] and not result["automatic_promotion"]
    assert (output / "visuals/run_001/frame.png").read_bytes() == b"\x89PNG\r\n\x1a\nunchanged"
    for line in (output / "SHA256SUMS").read_text().splitlines():
        expected, name = line.split("  ")
        assert curate.audit.base.sha(output / name) == expected


@pytest.mark.parametrize("field,value", [("development_screen_clear_count", 1), ("base_raw_quality_candidate_count", 2),
    ("auditor_source_sha256", "0" * 64), ("trial_count", 1)])
def test_stale_counts_source_revision_or_attempt_count_fail_before_output(inputs, field, value):
    report_path = inputs[1]
    report = json.loads(report_path.read_text()); report[field] = value; write(report_path, report)
    with pytest.raises(curate.audit.base.EvidenceError): publish(inputs)
    assert not inputs[3].exists()


def test_modified_trial_report_cannot_override_fresh_measurements(inputs):
    report_path = inputs[1]
    report = json.loads(report_path.read_text()); report["trials"][0]["raw_quality_candidate"] = True; write(report_path, report)
    with pytest.raises(curate.audit.base.EvidenceError, match="source recomputation"): publish(inputs)
    assert not inputs[3].exists()


def test_unlisted_attempt_and_changed_pilot_plan_fail_closed(inputs):
    pilot = inputs[0]
    write(pilot / "pilot_plan.json", {"modified": True})
    with pytest.raises(curate.audit.base.EvidenceError, match="plan SHA"): publish(inputs)


def test_changed_expected_sha_or_existing_output_is_rejected(inputs):
    pilot, report, visuals, output, _ = inputs
    with pytest.raises(curate.audit.base.EvidenceError, match="audit SHA"):
        curate.curate(pilot, report, visuals, output, "0" * 64)
    publish(inputs)
    with pytest.raises(curate.audit.base.EvidenceError, match="new publication"): publish(inputs)


@pytest.fixture
def actual_visual_fixture(tmp_path):
    trial, visual = tmp_path / "trial", tmp_path / "visual"
    episode = trial / "episode.partial"
    cameras = []
    for i in range(6):
        relative = f"images/front_{i}.jpg"
        path = episode / relative
        path.parent.mkdir(parents=True, exist_ok=True); path.write_bytes(f"raw_{i}".encode())
        cameras.append(dict(images={"CAM_FRONT": relative}))
    for name in ("manifest.json", "route.json", "states.jsonl"):
        write(episode / name, {})
    (episode / "camera_frames.jsonl").write_text("".join(json.dumps(row) + "\n" for row in cameras))
    proof = dict(schema="carla_expert.raw_trial_visual_diagnostic.v1", source_episode_name="episode.partial",
        original_owner_exit_code=1, training_data_approved=False, learned_model_control=False, live_autoware_screenshot=False,
        source_renderer_sha256=curate.audit.base.sha(curate.audit.REPOSITORY / "scripts/e2e/render_carla_raw_trial.py"),
        source_layout_sha256=curate.audit.base.sha(curate.audit.REPOSITORY / "scripts/e2e/render_carla_vad_expert.py"),
        source_metadata_sha256={name: curate.audit.base.sha(episode / name) for name in ("manifest.json", "route.json", "states.jsonl", "camera_frames.jsonl")},
        camera_anchor_count=6, camera_stride=5, playback_fps=10.0, rendered_indices=[0, 5],
        png_indices={name: i for i, name in enumerate(curate.PNG_NAMES)},
        displayed_image_sha256={f"images/front_{i}.jpg": curate.audit.base.sha(episode / f"images/front_{i}.jpg") for i in range(6)})
    write(visual / "visual_provenance.json", proof)
    for name in curate.PNG_NAMES: (visual / f"{name}.png").write_bytes(b"\x89PNG\r\n\x1a\nsynthetic")
    (visual / "whole_recording_accelerated.gif").write_bytes(b"GIF89asynthetic")
    return trial, visual, dict(owner_exit_code=1)


def test_visual_binding_checks_original_images_and_exact_metadata(actual_visual_fixture):
    outputs, ledger = curate.bind_visuals(*actual_visual_fixture)
    assert len(outputs) == len(ledger) == 8
    assert outputs["01_start.png"] == b"\x89PNG\r\n\x1a\nsynthetic"
    trial, visual, expected = actual_visual_fixture
    (trial / "episode.partial/images/front_2.jpg").write_bytes(b"replaced")
    with pytest.raises(curate.audit.base.EvidenceError, match="raw image changed"):
        curate.bind_visuals(trial, visual, expected)


@pytest.mark.parametrize("field,value", [("source_renderer_sha256", "0" * 64), ("camera_stride", 1),
    ("rendered_indices", [0]), ("training_data_approved", True), ("source_episode_name", "../private")])
def test_visual_provenance_drift_is_not_published(actual_visual_fixture, field, value):
    trial, visual, expected = actual_visual_fixture
    path = visual / "visual_provenance.json"
    proof = json.loads(path.read_text()); proof[field] = value; write(path, proof)
    with pytest.raises(curate.audit.base.EvidenceError): curate.bind_visuals(trial, visual, expected)


def test_missing_displayed_image_pin_and_symlink_visual_are_rejected(actual_visual_fixture):
    trial, visual, expected = actual_visual_fixture
    path = visual / "visual_provenance.json"
    proof = json.loads(path.read_text()); proof["displayed_image_sha256"].pop("images/front_1.jpg"); write(path, proof)
    with pytest.raises(curate.audit.base.EvidenceError, match="image coverage"):
        curate.bind_visuals(trial, visual, expected)
