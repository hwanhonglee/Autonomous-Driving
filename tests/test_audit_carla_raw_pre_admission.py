"""HH_260906 - Exercise pre-admission diagnostics without a simulator, optimizer or Common10 export."""

import copy
from io import BytesIO
import json
import math
from pathlib import Path

from PIL import Image
import pytest

from scripts.e2e import audit_carla_raw_pre_admission as audit


def state(i, *, speed=1.0, phase="driving"):
    return dict(timestamp=i * .05, frame=1000 + i, x=i * .05, y=0., z=0., yaw=0., vx=speed, vy=0.,
        ax=0., ay=0., yaw_rate=0., capture_phase=phase)


def camera(i, phase="driving"):
    return dict(frame=1000 + i, timestamp=i * .05, capture_phase=phase, camera_order=list(audit.contract.CAMERA_ORDER),
        images={name: f"images/{name}/{1000 + i:08d}.jpg" for name in audit.contract.CAMERA_ORDER},
        source_timestamps={name: i * .05 for name in audit.contract.CAMERA_ORDER}, jpeg_quality=90, timestamp_span_sec=0.)


def image(path, size=(640, 360), format="JPEG"):
    path.parent.mkdir(parents=True, exist_ok=True)
    out = BytesIO(); Image.new("RGB", size, (12, 33, 55)).save(out, format=format)
    path.write_bytes(out.getvalue())


def test_complete_pixel_decode_hashes_all_pixels_without_transform(tmp_path):
    image(tmp_path / "camera.jpg")
    before = (tmp_path / "camera.jpg").read_bytes()
    result = audit.decode_image(tmp_path, "camera.jpg", (640, 360))
    assert result["decoded"] and result["sha256"] == audit.digest(before)
    assert result["width"] == 640 and result["height"] == 360 and len(result["decoded_pixel_sha256"]) == 64
    assert (tmp_path / "camera.jpg").read_bytes() == before


@pytest.mark.parametrize("variant", ["corrupt", "truncated", "wrong_size", "png"])
def test_bad_image_stays_in_report_as_failed_not_omitted(tmp_path, variant):
    path = tmp_path / "camera.jpg"
    image(path, size=(640, 359) if variant == "wrong_size" else (640, 360), format="PNG" if variant == "png" else "JPEG")
    if variant == "corrupt": path.write_bytes(b"not an image")
    if variant == "truncated": path.write_bytes(path.read_bytes()[:-120])
    result = audit.decode_image(tmp_path, "camera.jpg", (640, 360))
    assert not result["decoded"] and result["failure"] and result["sha256"] == audit.digest(path.read_bytes())


def test_symlink_and_path_escape_rejected_before_decode(tmp_path):
    image(tmp_path / "camera.jpg")
    (tmp_path / "alias.jpg").symlink_to(tmp_path / "camera.jpg")
    for name in ("alias.jpg", "../camera.jpg"):
        with pytest.raises(audit.contract.ContractError): audit.decode_image(tmp_path, name, (640, 360))


def test_measured_interpolation_reuses_existing_math_and_does_not_bridge_missing_ticks():
    timeline = audit.measured_timeline([state(i) for i in range(4)])
    stamps = [t for t, _ in timeline]
    exact, mode = audit.future_state(timeline, stamps, 50_000_000)
    assert mode == "exact_native_state" and exact["x"] == .05
    result, mode = audit.future_state(timeline, stamps, 75_000_000)
    assert mode == "linear_native_bracket"
    assert result == audit.adapter._interpolate_state(timeline, 75_000_000)
    sparse = (timeline[0], timeline[2])
    assert audit.future_state(sparse, [p[0] for p in sparse], 50_000_000) == (None, "sensor_gap")
    assert audit.future_state(timeline, stamps, 200_000_000) == (None, "episode_end")


@pytest.mark.parametrize("mutation", [lambda x: x[2].update(timestamp=.05), lambda x: x[2].update(frame=1001),
    lambda x: x[2].update(vx=math.nan), lambda x: x[2].update(capture_phase="stationary_warmup")])
def test_ambiguous_timeline_is_never_invented(mutation):
    rows = [state(i) for i in range(4)]; mutation(rows)
    with pytest.raises((audit.scalar.EvidenceError, audit.adapter.AdapterError)): audit.measured_timeline(rows)


def test_full_future_masks_and_original_warmup_policy_are_preserved():
    rows = [state(i, phase="stationary_warmup" if i < 2 else "driving" if i < 6 else "stationary_tail") for i in range(136)]
    timeline = audit.measured_timeline(rows)
    cameras = [camera(i, rows[i]["capture_phase"]) for i in range(0, 136, 2)]
    meta = [dict(timestamp_ns=round(c["timestamp"] * 1e9), same_recorded_frame_and_timestamp=True) for c in cameras]
    route = dict(route_length_m=100., route=[dict(x=0., y=0.), dict(x=100., y=0.)])
    before = copy.deepcopy((rows, cameras))
    summary, records = audit.measured_future_audit(timeline, cameras, route, meta)
    assert summary["disposition_counts"] == {"full_64_point_anchor": 3, "tail_label_context_only": 65}
    assert summary["full_anchor_counts_by_phase"] == {"stationary_warmup": 1, "driving": 2}
    assert summary["full_64_point_anchors"]["valid_step_count"] == 192
    assert all(records[i]["valid_mask"] == [True] * 64 for i in range(3))
    assert summary["full_64_point_anchors"]["metrics"]["xy_acceleration"]["violating_step_count"] == 0
    assert (rows, cameras) == before


def test_missing_tail_is_counted_without_extrapolation_or_fabricated_target_values():
    timeline = audit.measured_timeline([state(i) for i in range(11)])
    cameras = [camera(0)]
    meta = [dict(timestamp_ns=0, same_recorded_frame_and_timestamp=True)]
    summary, rows = audit.measured_future_audit(timeline, cameras, dict(route_length_m=100., route=[dict(x=0., y=0.), dict(x=100., y=0.)]), meta)
    assert summary["disposition_counts"] == {"incomplete_future_retained": 1}
    assert summary["full_64_point_anchors"]["sample_count"] == 0
    assert rows[0]["valid_points"] == 5 and rows[0]["valid_mask"] == [True] * 5 + [False] * 59
    assert rows[0]["invalid_reasons"] == [None] * 5 + ["episode_end"] * 59
    assert len(rows[0]["diagnostic"]["steps"]) == 5


def test_future_speed_spike_is_measured_not_filtered():
    timeline = audit.measured_timeline([state(i, speed=5. if i == 2 else 1.) for i in range(130)])
    summary, rows = audit.measured_future_audit(timeline, [camera(0)], dict(route_length_m=100., route=[dict(x=0., y=0.), dict(x=100., y=0.)]),
        [dict(timestamp_ns=0, same_recorded_frame_and_timestamp=True)])
    assert rows[0]["valid_points"] == 64
    assert summary["full_64_point_anchors"]["metrics"]["speed_acceleration"]["violating_step_count"] == 1
    assert summary["full_64_point_anchors"]["metrics"]["speed_deceleration"]["violating_step_count"] == 1


def test_bad_camera_binding_stays_in_anchor_denominator():
    summary, rows = audit.measured_future_audit(audit.measured_timeline([state(i) for i in range(130)]), [camera(0)],
        dict(route_length_m=100., route=[dict(x=0., y=0.), dict(x=100., y=0.)]),
        [dict(timestamp_ns=0, same_recorded_frame_and_timestamp=False)])
    assert summary["camera_anchor_count"] == 1 and len(rows) == 1
    assert rows[0]["disposition"] == "invalid_camera_state_binding"


@pytest.fixture
def pixel_trial(tmp_path, monkeypatch):
    timeline = audit.measured_timeline([state(i) for i in range(2)])
    manifest = {"capture_contract": {"jpeg_quality": 90}}
    cameras = [camera(0)]
    for path in cameras[0]["images"].values(): image(tmp_path / path)
    rig = {"cameras": [dict(name=name, width_px=640, height_px=360) for name in audit.contract.CAMERA_ORDER]}
    monkeypatch.setattr(audit.adapter, "_rig_document", lambda _: rig)
    monkeypatch.setattr(audit.contract, "_validate_rig", lambda *args: None)
    return tmp_path, manifest, cameras, timeline


def test_every_role_is_decoded_including_corrupt_and_orphan_pixels(pixel_trial):
    episode, manifest, cameras, timeline = pixel_trial
    (episode / next(iter(cameras[0]["images"].values()))).write_bytes(b"bad")
    image(episode / "images/orphan.jpg")
    report, images, anchors = audit.camera_audit(episode, manifest, cameras, timeline)
    assert report["image_count"] == 7 and report["referenced_image_count"] == 6
    assert report["decode_failure_count"] == 1 and report["unreferenced_jpeg_count"] == 1
    assert len(images) == 7 and not report["all_pixel_and_metadata_checks_clear"]


def test_camera_timestamp_mismatch_is_reported_without_discarding_images(pixel_trial):
    pixel_trial[2][0]["source_timestamps"][audit.contract.CAMERA_ORDER[0]] = .001
    report, images, anchors = audit.camera_audit(*pixel_trial)
    assert report["timestamp_or_phase_mismatch_count"] == 1 and len(images) == 6
    assert not report["all_pixel_and_metadata_checks_clear"]


def test_source_function_pins_cover_reused_formulas_and_no_conversion_entrypoint_is_called():
    identity = audit.source_identity()
    assert all(len(value) == 64 for value in identity["files"].values())
    assert any(name.endswith(".audit_sample") for name in identity["functions"])
    source = Path(audit.__file__).read_text()
    assert "adapter._load_native(" not in source and "adapter.prepare_dataset(" not in source
    assert "NativeEpisode(" not in source and '"dataset.json"' not in source and "load_training_examples(" not in source


def test_script_pin_and_new_output_are_checked_before_trial_access(tmp_path, monkeypatch):
    monkeypatch.setattr(audit, "audit_trial", lambda *args: pytest.fail("trial should not be opened"))
    with pytest.raises(audit.scalar.EvidenceError, match="script SHA"):
        audit.run([("run_001", tmp_path, "0" * 64)], tmp_path / "output", "0" * 64)
    with pytest.raises(audit.scalar.EvidenceError, match="outside raw"):
        audit.run([("run_001", tmp_path, "0" * 64)], tmp_path / "output", audit.scalar.sha(Path(audit.__file__)))


def test_cli_no_abbreviations():
    with pytest.raises(SystemExit): audit.main(["--tri", "x", "y", "z", "--output-dir", "new", "--expected-script-sha256", "0" * 64])


@pytest.fixture
def batch_fixture(tmp_path, monkeypatch):
    root = tmp_path / "raw"
    image(root / "episode.partial/images/camera.jpg")
    (root / "owner_plan.json").write_text("{}\n")
    images = [{"path": "images/camera.jpg", "sha256": audit.scalar.sha(root / "episode.partial/images/camera.jpg"), "decoded": True}]
    report = {"trial_id": "raw", "original_episode_directory": "episode.partial", "training_data_approved": False,
        "source_manifest": [{"path": "owner_plan.json", "sha256": audit.scalar.sha(root / "owner_plan.json")} ]}
    monkeypatch.setattr(audit, "audit_trial", lambda *args: (copy.deepcopy(report), copy.deepcopy(images), []))
    return [("run_001", root, audit.scalar.sha(root / "owner_plan.json"))], tmp_path / "diagnostic"


def test_batch_writes_only_diagnostic_outputs_and_immutable_hash_manifest(batch_fixture):
    trials, output = batch_fixture
    report = audit.run(trials, output, audit.scalar.sha(Path(audit.__file__)))
    assert report["total_jpeg_count"] == report["total_decoded_jpeg_count"] == 1
    assert set(p.name for p in output.iterdir()) == {"summary.json", "jpeg_audit.jsonl", "future_anchor_audit.jsonl", "SHA256SUMS"}
    assert not report["scope"]["common10_dataset_written"] and not report["scope"]["training_data_approved"]
    for line in (output / "SHA256SUMS").read_text().splitlines():
        expected, name = line.split("  ")
        assert audit.scalar.sha(output / name) == expected
    with pytest.raises(audit.scalar.EvidenceError): audit.run(trials, output, audit.scalar.sha(Path(audit.__file__)))


def test_raw_pixel_changed_after_trial_audit_blocks_batch_output(batch_fixture, monkeypatch):
    trials, output = batch_fixture
    original = audit.audit_trial
    def changed(*args):
        result = original(*args)
        (trials[0][1] / "episode.partial/images/camera.jpg").write_bytes(b"replaced")
        return result
    monkeypatch.setattr(audit, "audit_trial", changed)
    with pytest.raises(audit.scalar.EvidenceError, match="JPEG changed during full batch"):
        audit.run(trials, output, audit.scalar.sha(Path(audit.__file__)))
    assert not output.exists()


def test_duplicate_trial_path_never_doubles_evidence_denominator(batch_fixture):
    trials, output = batch_fixture
    duplicate = [trials[0], ("second_name", *trials[0][1:])]
    with pytest.raises(audit.scalar.EvidenceError, match="counted twice"):
        audit.run(duplicate, output, audit.scalar.sha(Path(audit.__file__)))
