"""HH_260906 - Verify compact raw-geometry publication with explicitly synthetic, test-local pinned streams."""

import copy
import hashlib
import json
from pathlib import Path

import pytest

from scripts.e2e import curate_carla_turn_launch_raw_geometry as curate


def write_inputs(root, summary, rows, monkeypatch):
    # HH_260906 - Only test-generated streams replace artifact pins; production has literal reviewed original hashes.
    (root / "summary.json").write_text(json.dumps(summary))
    for name, values in rows.items():
        (root / name).write_text("".join(json.dumps(v) + "\n" for v in values))
    pins = {n: curate.contract.sha256_file(root / n) for n in curate.INPUTS}
    (root / "SHA256SUMS").write_text("".join(f"{pins[n]}  {n}\n" for n in sorted(pins)))
    monkeypatch.setattr(curate, "ARTIFACT_SHA256", pins)
    monkeypatch.setattr(curate, "CHECKSUM_SHA256", curate.contract.sha256_file(root / "SHA256SUMS"))


@pytest.fixture
def evidence(tmp_path, monkeypatch):
    root = tmp_path / "diagnostic"
    root.mkdir()
    cases, matrix_cases, repeats = [], [], {}
    rows = {name: [] for name in curate.INPUTS if name.endswith(".jsonl")}
    for sequence, profile in enumerate(curate.diagnostic.ORDER, 1):
        repeats[profile] = repeats.get(profile, 0) + 1
        key = f"{sequence:02d}_{profile}_r{repeats[profile]}"
        basic = {"case_id": key, "profile": profile, "replicate": repeats[profile]}
        matrix_cases.append(dict(basic))
        samples = []
        for index in range(4):
            phase = "stationary_warmup" if index == 0 else "stationary_tail" if index == 3 else "driving"
            frame, stamp = 1000 + index*2, 1_000_000_000 + index*100_000_000
            row = {"case_id": key, "frame": frame, "capture_phase": phase, "anchor_timestamp_ns": stamp,
                   "training_data_approved": False, "disposition": "tail_label_context_only", "valid_future_points": 0}
            for camera in range(6):
                rows["jpeg_audit.jsonl"].append({"case_id": key, "path": f"images/camera{camera}/{frame}.jpg",
                    "decoded": True, "width": 640, "height": 360, "mode": "RGB", "referenced": True})
            if phase != "stationary_tail":
                xy = [[(i+1)*.1, 0.] for i in range(64)]
                if index == 0: xy[0] = [-.000125, 0.]
                planning = {"available": True, "dt_s": .1, "positions_base_xy_m": xy, "speed_mps": [1.]*64,
                    "yaw_rad": [0.]*64, "valid": [True]*64, "target_timestamp_ns": [stamp+(i+1)*100_000_000 for i in range(64)],
                    "invalid_reason": [None]*64}
                sample = {"sample_id": f"{key}:frame{frame}", "anchor_timestamp_ns": stamp,
                    "ego": {"linear_velocity_base_mps": [1., 0., 0.]},
                    "navigation": {"goal_base_m": [100., 0.], "route_anchor_arc_m": 0.}, "labels": {"planning": planning}}
                measured = curate.diagnostic.raw.targets.audit_sample(sample, route_length_m=100., episode_start_ns=1_000_000_000,
                    episode_end_ns=100_000_000_000)
                measured.update(episode_id=key, capture_phase=phase)
                samples.append(measured)
                row.update(disposition="full_64_point_anchor", valid_future_points=64, discrete_bound_diagnostic=measured,
                    diagnostic_future_xy_m=xy, diagnostic_body_yaw_delta_rad=[0.]*64,
                    diagnostic_target_timestamp_ns=planning["target_timestamp_ns"], diagnostic_endpoint_planar_speed_mps=[1.]*64,
                    current_recorded_velocity_base_mps=[1., 0., 0.])
            rows["future_geometry_audit.jsonl"].append(row)
        for index in range(8):
            phase = "stationary_warmup" if index < 2 else "stationary_tail" if index >= 6 else "driving"
            rows["native_snapshot_audit.jsonl"].append({"case_id": key, "frame": 1000+index,
                "timestamp_ns": 1_000_000_000+index*50_000_000, "capture_phase": phase})
        future = {"anchor_count_by_phase": {"stationary_warmup": 1, "driving": 2, "stationary_tail": 1},
            "disposition_counts": {"full_64_point_anchor": 3, "tail_label_context_only": 1}, "camera_anchor_count": 4,
            "available_64_point_anchor_count": 3, "bound_assessed_64_point_anchor_count": 3, "bound_unassessed_anchor_count": 0,
            "all_available_prefixes": {str(h): curate.diagnostic.raw.targets.summarize_samples(samples, h) for h in (10, 30, 64)}}
        cases.append({**basic, "status": "DIAGNOSED_NOT_ADMITTED", "training_data_approved": False,
            "original_training_data_approved": False, "original_development_only": True,
            "raw_native_state_count": 8, "raw_native_state_counts_by_phase": {"stationary_warmup": 2, "driving": 4, "stationary_tail": 2},
            "measured_future": future, "transport_protocol": {"status": "PASS"}, "initialization_protocol": {"status": "PASS"},
            "camera_pixels": {"image_count": 24, "decoded_count": 24, "camera_anchor_count": 4},
            "raw_scalar_quality": {"raw_scalar_quality_clear": sequence == 2,
                "speed_rate_qa": {"native_20hz": {"by_phase": {"all": {"minimum_mps2": -1., "maximum_mps2": 3.5}}}}}})
    scope = {k: False for k in ("training_data_approved", "dataset_admission", "common10_dataset_written", "source_flags_modified",
        "labels_modified", "training", "model_loaded", "model_inference", "live_simulator_access", "test_payload_read",
        "automatic_winner_selection", "native20hz_quality_replaced_by_10hz")}
    summary = {"schema": curate.diagnostic.SCHEMA, "status": "DIAGNOSED_NOT_ADMITTED", "planned_case_count": 8,
        "finalized_case_count": 8, "all_planned_cases_retained": True, "cases": cases, "scope": scope,
        "source_identity": {"files": {"scripts/e2e/audit_carla_turn_launch_raw_geometry.py": curate.DIAGNOSTIC_SHA256,
            "scripts/e2e/audit_carla_turn_launch_matrix.py": curate.diagnostic.MATRIX_AUDITOR_SHA256}},
        "independent_matrix_audit": {"cases": matrix_cases}, "total_jpeg_count": 192, "total_decoded_jpeg_count": 192}
    write_inputs(root, summary, rows, monkeypatch)
    return root, summary, rows, tmp_path / "publication"


def test_every_stream_count_and_curvature_conditioning_recomputed(evidence):
    root, _, _, _ = evidence
    summary, pins = curate.source_inputs(root)
    result = curate.validate_streams(root, summary, pins)
    assert result["image_count"] == 192 and result["native_state_count"] == 64 and result["camera_anchor_count"] == 32
    assert result["available_64_point_anchor_count"] == result["bound_assessed_64_point_anchor_count"] == 24
    for detail in result["curvature_conditioning_by_case"].values():
        assert detail["first_point"] > 0 and detail["later_points"] > 0
        assert detail["maximum_curvature_witness"]["xy_interval_distance_m"] == pytest.approx(.000125)
        assert detail["xy_interval_speed_buckets_mps"]["lt0p1"] > 0
    assert all(row["frame"] == 1004 for row in result["midpoint_driving_anchors"].values())


@pytest.mark.parametrize("mutation", ["source", "order", "case_id", "replicate", "matrix_id", "missing", "promotion"])
def test_unknown_source_case_or_approval_cannot_publish(evidence, mutation):
    _, summary, _, _ = evidence
    if mutation == "source": summary["source_identity"]["files"]["scripts/e2e/audit_carla_turn_launch_raw_geometry.py"] = "0"*64
    elif mutation == "order": summary["cases"].reverse()
    elif mutation == "case_id": summary["cases"][0]["case_id"] = "renamed"
    elif mutation == "replicate": summary["cases"][0]["replicate"] = 2
    elif mutation == "matrix_id": summary["independent_matrix_audit"]["cases"][0]["case_id"] = "renamed"
    elif mutation == "missing": summary["finalized_case_count"] = 7
    else: summary["scope"]["training_data_approved"] = True
    with pytest.raises(curate.diagnostic.raw.scalar.EvidenceError): curate.validate_summary(summary)


@pytest.mark.parametrize("mutation", ["missing_image", "duplicate_image", "frame_gap", "future_count", "curvature_count", "bad_displacement"])
def test_self_consistent_bytes_cannot_hide_stream_summary_contradiction(evidence, monkeypatch, mutation):
    root, summary, rows, _ = evidence
    if mutation == "missing_image": rows["jpeg_audit.jsonl"].pop()
    elif mutation == "duplicate_image": rows["jpeg_audit.jsonl"].append(copy.deepcopy(rows["jpeg_audit.jsonl"][0]))
    elif mutation == "frame_gap": rows["native_snapshot_audit.jsonl"][1]["frame"] += 2
    elif mutation == "future_count": summary["cases"][0]["measured_future"]["available_64_point_anchor_count"] += 1
    elif mutation == "curvature_count": summary["cases"][0]["measured_future"]["all_available_prefixes"]["64"]["metrics"]["xy_curvature"]["violating_step_count"] += 1
    else: rows["future_geometry_audit.jsonl"][0]["diagnostic_future_xy_m"][0][0] = 1.
    write_inputs(root, summary, rows, monkeypatch)
    current, pins = curate.source_inputs(root)
    with pytest.raises(curate.diagnostic.raw.scalar.EvidenceError): curate.validate_streams(root, current, pins)


def test_self_consistent_new_checksum_cannot_replace_reviewed_original(evidence):
    root, _, _, _ = evidence
    path = root / "summary.json"
    path.write_bytes(path.read_bytes() + b"\n")
    with pytest.raises(curate.diagnostic.raw.scalar.EvidenceError, match="checksum|original"):
        curate.source_inputs(root)


def test_publish_preserves_exact_summary_and_creates_only_new_public_outputs(evidence, monkeypatch):
    root, _, _, output = evidence
    original = {p.name: curate.contract.sha256_file(p) for p in root.iterdir()}
    monkeypatch.setattr(curate, "charts", lambda s, r, out: (out / "test_chart.png").write_bytes(b"synthetic test marker, not imagery"))
    result = curate.publish(root, output)
    assert result["training_data_approved"] is False
    assert (output / "summary.json").read_bytes() == (root / "summary.json").read_bytes()
    assert original == {p.name: curate.contract.sha256_file(p) for p in root.iterdir()}
    assert not (output / "future_geometry_audit.jsonl").exists()
    for line in (output / "SHA256SUMS").read_text().splitlines():
        digest, name = line.split("  ")
        assert curate.contract.sha256_file(output / name) == digest


@pytest.mark.parametrize("location", ["existing", "raw"])
def test_existing_or_raw_output_refused_before_any_input_read(evidence, monkeypatch, location):
    root, _, _, output = evidence
    if location == "existing": output.mkdir()
    else: output = root / "bad"
    monkeypatch.setattr(curate, "source_inputs", lambda *a: pytest.fail("output check must happen first"))
    with pytest.raises(curate.diagnostic.raw.scalar.EvidenceError): curate.publish(root, output)


def test_generic_private_metadata_guard_does_not_need_server_literals(evidence, monkeypatch):
    root, summary, rows, output = evidence
    summary["unrelated_comment"] = "private user@example.invalid"
    write_inputs(root, summary, rows, monkeypatch)
    with pytest.raises(curate.diagnostic.raw.scalar.EvidenceError, match="private account"):
        curate.publish(root, output)
    assert not output.exists()


def test_actual_chart_render_uses_streamed_midpoints(evidence, tmp_path):
    root, _, _, _ = evidence
    summary, pins = curate.source_inputs(root)
    counts = curate.validate_streams(root, summary, pins)
    output = tmp_path / "charts"
    output.mkdir()
    curate.charts(summary, counts, output)
    assert {p.name for p in output.iterdir()} == {"all_eight_scalar_and_xy_bounds.png", "eight_ego_centered_measured_futures.png", "curvature_low_displacement_conditioning.png"}
    from PIL import Image
    for path in output.iterdir():
        with Image.open(path) as picture:
            picture.load()
            assert picture.width > 1000 and picture.height > 1000


def test_source_changed_during_render_cannot_receive_publication_checksum(evidence, monkeypatch):
    root, _, _, output = evidence
    def mutate(*args):
        path = root / "summary.json"
        path.write_bytes(path.read_bytes() + b"\n")
    monkeypatch.setattr(curate, "charts", mutate)
    with pytest.raises(curate.diagnostic.raw.scalar.EvidenceError, match="changed during rendering"):
        curate.publish(root, output)
    assert not (output / "SHA256SUMS").exists()


def test_render_dependency_change_cannot_receive_publication_checksum(evidence, monkeypatch):
    root, _, _, output = evidence
    versions = iter(({"versions": {"numpy": "first"}}, {"versions": {"numpy": "changed"}}))
    monkeypatch.setattr(curate, "render_provenance", lambda: next(versions))
    monkeypatch.setattr(curate, "charts", lambda *args: None)
    with pytest.raises(curate.diagnostic.raw.scalar.EvidenceError, match="render environment"):
        curate.publish(root, output)
    assert not (output / "SHA256SUMS").exists()
