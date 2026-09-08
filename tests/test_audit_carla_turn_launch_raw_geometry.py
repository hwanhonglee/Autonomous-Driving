"""HH_260906 - Test frozen-matrix raw diagnostics without simulator access, data admission or conversion."""

import copy
import json
import math
from pathlib import Path

import pytest

from scripts.e2e import audit_carla_turn_launch_raw_geometry as audit
from tests.test_audit_carla_raw_pre_admission import state, camera, snapshot_state, image
from tests.test_prepare_carla_common10_dataset import _native_camera

ROUTE = {"route_length_m": 100., "route": [{"x": 0., "y": 0.}, {"x": 100., "y": 0.}]}


def geometry(rows, indices, *, valid=True, route=ROUTE):
    timeline = audit.raw.measured_timeline(rows)
    by_index = {r["frame"] - 1000: r for r in rows}
    cameras = [camera(i, by_index[i]["capture_phase"]) for i in indices]
    anchors = [{"timestamp_ns": round(c["timestamp"] * 1e9), "same_recorded_frame_and_timestamp": valid} for c in cameras]
    return audit.geometry_futures(timeline, cameras, route, anchors, "case_01")


def test_full_warmup_driving_and_65_tail_anchor_denominators_are_preserved():
    rows = [state(i, phase="stationary_warmup" if i < 70 else "driving" if i < 80 else "stationary_tail") for i in range(210)]
    original = copy.deepcopy(rows)
    report, anchors = geometry(rows, range(0, 210, 2))
    assert report["camera_anchor_count"] == 105
    assert report["anchor_count_by_phase"] == {"stationary_warmup": 35, "driving": 5, "stationary_tail": 65}
    assert report["disposition_counts"] == {"full_64_point_anchor": 40, "tail_label_context_only": 65}
    assert report["available_64_point_anchor_count"] == report["bound_assessed_64_point_anchor_count"] == 40
    assert all(r["valid_mask"] == [True] * 64 for r in anchors[:40])
    assert set(report["by_anchor_phase"]["driving"]) == {"10", "30", "64"}
    assert report["by_anchor_phase"]["stationary_warmup"]["64"]["sample_count"] == 35
    assert all(not r["training_data_approved"] for r in anchors)
    assert rows == original


def test_missing_tail_retains_prefix_nulls_and_no_fabricated_64_point_success():
    report, rows = geometry([state(i) for i in range(11)], [0])
    assert report["available_64_point_anchor_count"] == 0
    assert rows[0]["valid_future_points"] == 5
    assert rows[0]["invalid_reasons"] == [None] * 5 + ["episode_end"] * 59
    assert rows[0]["diagnostic_future_xy_m"][5:] == [[None, None]] * 59
    assert rows[0]["diagnostic_endpoint_planar_speed_mps"][5:] == [None] * 59


@pytest.mark.parametrize("gap", ["missing_mid_tick", "bad_frame_stride", "bad_clock"])
def test_exact_100ms_endpoint_cannot_hide_missing_intermediate_native_tick(gap):
    rows = [state(i) for i in range(130)]
    if gap == "missing_mid_tick": rows.pop(1)
    elif gap == "bad_frame_stride":
        for r in rows[1:]: r["frame"] += 1
    else: rows[1]["timestamp"] += .001
    report, anchors = geometry(rows, [0])
    assert report["native_gap_count"] >= 1
    assert anchors[0]["valid_future_points"] == 0
    assert anchors[0]["invalid_reasons"] == ["sensor_gap"] * 64
    assert "first_uncovered_native_interval" in anchors[0]
    assert report["full_64_point_anchors"]["sample_count"] == 0


def test_body_yaw_proxy_cannot_be_relabelled_as_xy_curvature_failure():
    rows = [state(i) for i in range(130)]
    baseline, _ = geometry(rows, [0])
    for r in rows[1:]: r["yaw"] = .5
    changed, _ = geometry(rows, [0])
    for key in ("xy_curvature", "xy_lateral_acceleration", "xy_acceleration"):
        assert changed["full_64_point_anchors"]["metrics"][key] == baseline["full_64_point_anchors"]["metrics"][key]
    assert changed["full_64_point_anchors"]["metrics"]["yaw_label_heading_envelope"]["violating_step_count"] > 0
    assert "proxy" in changed["definitions"]["body_yaw"].lower()


def test_endpoint_speed_spike_and_xy_interval_geometry_remain_separate():
    rows = [state(i) for i in range(130)]
    rows[2]["vx"] = 5.
    report, _ = geometry(rows, [0])
    metrics = report["full_64_point_anchors"]["metrics"]
    assert metrics["speed_acceleration"]["violating_step_count"] == 1
    assert metrics["xy_acceleration"]["violating_step_count"] == 0


def test_stationary_headings_remain_unassessed_not_passed():
    rows = [state(i, speed=0.) for i in range(130)]
    for r in rows: r["x"] = 0.
    report, _ = geometry(rows, [0])
    for key in ("xy_curvature", "xy_lateral_acceleration", "yaw_label_heading_envelope"):
        assert report["full_64_point_anchors"]["metrics"][key]["assessed_step_count"] == 0


def test_route_endpoint_failure_retains_actual_xy_and_unassessed_anchor():
    rows = [state(i) for i in range(130)]
    for r in rows: r["x"] += 100.
    report, anchors = geometry(rows, [0])
    assert report["available_64_point_anchor_count"] == 1
    assert report["bound_assessed_64_point_anchor_count"] == 0
    assert report["bound_unassessed_anchor_count"] == 1
    assert anchors[0]["valid_future_points"] == 64
    assert "bound_diagnostic_unavailable" in anchors[0]
    assert anchors[0]["diagnostic_future_xy_m"][0][0] == pytest.approx(.1)


def test_bad_camera_binding_is_retained_without_a_successful_horizon():
    report, rows = geometry([state(i) for i in range(130)], [0], valid=False)
    assert report["camera_anchor_count"] == 1
    assert rows[0]["disposition"] == "invalid_camera_state_binding"
    assert report["available_64_point_anchor_count"] == 0


def test_snapshot_full3d_world_vectors_are_preserved_without_physical_center_claim():
    rows = [snapshot_state(i, pitch=10., yaw=90.) for i in range(3)]
    original = copy.deepcopy(rows)
    report = audit.snapshot_diagnostic(rows, 2.85)
    assert report["coordinate_conversion_status"] == "RECORDED_FORMULA_MATCH"
    assert not report["physical_reference_point_identity_proven"]
    for before, after in zip(rows, report["observations"]):
        assert "actor_center_ros_xyz_m" not in after
        assert after["source_actor_snapshot_transform_carla"] == before["actor_snapshot_transform_carla"]
        assert after["source_world_acceleration_carla"] == before["world_acceleration_carla"]
        assert after["source_world_angular_velocity_carla_deg_s"] == before["world_angular_velocity_carla_deg_s"]
    assert rows == original


def test_old_v4_dispatch_and_source_identity_are_not_modified():
    assert audit.raw.scalar.sha(Path(audit.raw.__file__)) == audit.RAW_HELPER_SHA256
    with pytest.raises(audit.raw.scalar.EvidenceError, match="unreviewed raw profile"):
        audit.raw.protocol_kind({"capture_contract": {"goal_stop_profile": {"profile_id": "turn_launch_013_v1"},
            "control_transport": {"mode": "acknowledged_batch"}}})


def test_pending_or_modified_matrix_auditor_cannot_execute(monkeypatch):
    for pin in ("PENDING_REVIEW", "0" * 64):
        monkeypatch.setattr(audit, "MATRIX_AUDITOR_SHA256", pin)
        with pytest.raises(audit.raw.scalar.EvidenceError, match="independently reviewed frozen source"):
            audit.source_identity()


@pytest.fixture
def campaign_fixture(tmp_path, monkeypatch):
    # HH_260906 - Synthetic facts are injected only in tests; production always calls the pinned independent verifier.
    root = tmp_path / "campaign"
    root.mkdir()
    cases, audits, seen, repeats = [], [], [], {}
    for index, profile in enumerate(audit.ORDER, 1):
        repeats[profile] = repeats.get(profile, 0) + 1
        replicate = repeats[profile]
        relative = f"c_track_left/{profile}/run_{replicate:03d}"
        raw_root = root / relative
        raw_root.mkdir(parents=True)
        trial = {"owner_exit_code": 1 if index != 2 else 0, "independent_qa": {"raw_scalar_quality_clear": index == 2},
                 "source_manifest": []}
        cases.append({"case_id": f"case_{index}", "sequence": index, "profile": profile, "replicate": replicate,
                      "output": relative, "status": "FINALIZED", "audit": trial})
        audits.append((raw_root, (trial, [], None)))
    report = {"schema": "portable_e2e.turn_launch_matrix_audit.v1", "status": "AUDITED_NOT_ADMITTED",
        "reviewed_execution_commit": audit.EXECUTION_COMMIT, "prospective_plan_sha256": audit.PLAN_SHA256,
        "all_planned_cases_retained": True, "planned_cases": 8, "finalized_cases": 8,
        "cases": cases, "source_manifest": [], "historical_source_manifest": []}
    monkeypatch.setattr(audit, "source_identity", lambda: {"files": {}, "functions": {}})
    def verified(path):
        seen.append("independent_matrix_first")
        return report, audits
    monkeypatch.setattr(audit.matrix, "audit_campaign", verified)
    def diagnose(path, trial, images, timeline, case):
        assert seen and seen[0] == "independent_matrix_first"
        seen.append(case["case_id"])
        return {"case_id": case["case_id"], "status": "DIAGNOSED_NOT_ADMITTED", "training_data_approved": False,
                "raw_scalar_quality": trial["independent_qa"]}, [], [], []
    monkeypatch.setattr(audit, "diagnose_trial", diagnose)
    monkeypatch.setattr(audit, "recheck_inputs", lambda *args: seen.append("final_recheck"))
    return root, tmp_path / "new_diagnostics", report, audits, seen


def test_all_eight_failed_and_successful_trials_are_kept_without_common10_outputs(campaign_fixture, monkeypatch):
    root, output, report, audits, seen = campaign_fixture
    monkeypatch.setattr(audit.raw.adapter, "_load_native", lambda *a: pytest.fail("native conversion is forbidden"))
    monkeypatch.setattr(audit.raw.contract, "validate_dataset", lambda *a, **kw: pytest.fail("Common10 loading forbidden"))
    result = audit.run(root, output, audit.raw.scalar.sha(Path(audit.__file__)))
    assert result["status"] == "DIAGNOSED_NOT_ADMITTED"
    assert len(result["cases"]) == result["finalized_case_count"] == 8
    assert sum(r["raw_scalar_quality"]["raw_scalar_quality_clear"] for r in result["cases"]) == 1
    assert seen[0] == "independent_matrix_first" and seen[-1] == "final_recheck"
    assert result["scope"]["training_data_approved"] is result["scope"]["common10_dataset_written"] is False
    assert {p.name for p in output.iterdir()} == {"summary.json", "jpeg_audit.jsonl", "future_geometry_audit.jsonl", "native_snapshot_audit.jsonl", "SHA256SUMS"}
    for line in (output / "SHA256SUMS").read_text().splitlines():
        digest, filename = line.split("  ")
        assert audit.raw.scalar.sha(output / filename) == digest


def test_incomplete_campaign_preserves_all_unrun_denominators(campaign_fixture):
    root, output, report, audits, _ = campaign_fixture
    report.update(status="INCOMPLETE", finalized_cases=2)
    del audits[2:]
    for case in report["cases"][2:]: case["status"] = "NOT_RUN"
    result = audit.run(root, output, audit.raw.scalar.sha(Path(audit.__file__)))
    assert result["status"] == "INCOMPLETE"
    assert len(result["cases"]) == 8 and sum(c["status"] == "NOT_RUN" for c in result["cases"]) == 6


@pytest.mark.parametrize("mutation", ["omit_case", "wrong_order", "wrong_plan", "wrong_source", "wrong_count", "duplicate_root"])
def test_independent_campaign_contract_cannot_be_weakened(campaign_fixture, mutation):
    root, output, report, audits, _ = campaign_fixture
    if mutation == "omit_case": report["cases"].pop()
    elif mutation == "wrong_order": report["cases"][0]["profile"] = "comfortable_v4"
    elif mutation == "wrong_plan": report["prospective_plan_sha256"] = "0" * 64
    elif mutation == "wrong_source": report["reviewed_execution_commit"] = "0" * 40
    elif mutation == "wrong_count": report["finalized_cases"] = 7
    else: audits[1] = audits[0]
    with pytest.raises(audit.raw.scalar.EvidenceError): audit.run(root, output, audit.raw.scalar.sha(Path(audit.__file__)))
    assert not output.exists()


@pytest.mark.parametrize("location", ["exists", "raw_child", "bad_script"])
def test_output_preflight_refuses_raw_writes_and_overwrite_before_any_audit(campaign_fixture, location):
    root, output, _, _, seen = campaign_fixture
    expected = audit.raw.scalar.sha(Path(audit.__file__))
    if location == "exists": output.mkdir()
    elif location == "raw_child": output = root / "bad_output"
    else: expected = "0" * 64
    with pytest.raises(audit.raw.scalar.EvidenceError): audit.run(root, output, expected)
    assert not seen


def test_final_recheck_rejects_later_mutation_before_output(campaign_fixture, monkeypatch):
    root, output, _, _, _ = campaign_fixture
    def fail(*args):
        raise audit.raw.scalar.EvidenceError("raw/source/image changed during full geometry batch")
    monkeypatch.setattr(audit, "recheck_inputs", fail)
    with pytest.raises(audit.raw.scalar.EvidenceError, match="changed during"):
        audit.run(root, output, audit.raw.scalar.sha(Path(audit.__file__)))
    assert not output.exists()


def test_source_changes_during_analysis_block_output(campaign_fixture, monkeypatch):
    root, output, _, _, _ = campaign_fixture
    sequence = iter(({"files": {"a": "first"}}, {"files": {"a": "changed"}}))
    monkeypatch.setattr(audit, "source_identity", lambda: next(sequence))
    with pytest.raises(audit.raw.scalar.EvidenceError, match="source changed"):
        audit.run(root, output, audit.raw.scalar.sha(Path(audit.__file__)))
    assert not output.exists()


def test_cli_has_no_converter_or_source_override_escape_hatch():
    with pytest.raises(SystemExit):
        audit.main(["campaign", "--output-dir", "out", "--expected-script-sha256", "0" * 64, "--allow-denied"])


@pytest.fixture
def raw_trial(tmp_path):
    # HH_260906 - Small on-disk synthetic pixels/states exercise the independent-reader boundary, not real capture proof.
    root = tmp_path / "run_001"
    episode = root / "episode.partial"
    episode.mkdir(parents=True)
    case = {"case_id": "synthetic_case", "profile": "turn_launch_013_v1", "replicate": 1}
    manifest = {"status": "failed", "result": {"training_data_approved": False, "development_only": True},
        "coordinate_contract": {"wheelbase_m": 2.85}, "runtime": {"server_version": "0.9.15"},
        "capture_contract": {"jpeg_quality": 90, "goal_stop_profile": {"profile_id": case["profile"]}},
        "cameras": [_native_camera(name, index, yaw) for index, (name, yaw) in enumerate(zip(
            audit.raw.contract.CAMERA_ORDER, (0., math.pi, math.pi/4, 3*math.pi/4, -math.pi/4, -3*math.pi/4)))]}
    states = [snapshot_state(i) for i in range(4)]
    cameras = [camera(0), camera(2)]
    route = {"route_length_m": 100., "route": [{"x": 0., "y": -30.}, {"x": 100., "y": -30.}]}
    for name, value in (("manifest.json", manifest), ("route.json", route)):
        (episode / name).write_text(json.dumps(value))
    for name, rows in (("states.jsonl", states), ("camera_frames.jsonl", cameras)):
        (episode / name).write_text("".join(json.dumps(row) + "\n" for row in rows))
    for name in ("portable_e2e/model.py", "portable_e2e/runtime_contract.py"):
        path = root / "provenance" / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes((audit.ROOT / name).read_bytes())
    ledger = [{"path": p.relative_to(root).as_posix(), "sha256": audit.raw.scalar.sha(p), "size_bytes": p.stat().st_size}
              for p in root.rglob("*") if p.is_file()]
    image_ledger = []
    for c in cameras:
        for relative in c["images"].values():
            path = episode / relative
            image(path)
            image_ledger.append({"path": "episode.partial/" + relative, "sha256": audit.raw.scalar.sha(path), "size_bytes": path.stat().st_size})
    trial = {"reviewed_execution_commit": audit.EXECUTION_COMMIT, "all_eleven_archives_match_owner_and_reviewed_commits": True,
        "source_manifest": ledger, "raw_data_available": True, "owner_exit_code": 1,
        "independent_qa": {"raw_scalar_quality_clear": False}, "pilot_protocol": {"all_checks_pass": False},
        "transport_protocol": {"status": "PASS"}, "initialization_protocol": {"status": "PASS"}}
    return root, trial, image_ledger, {"native_states": states}, case


def test_actual_reader_decodes_all_synthetic_roles_and_keeps_failed_capture(raw_trial, monkeypatch):
    original = {p: audit.raw.scalar.sha(p) for p in raw_trial[0].rglob("*") if p.is_file()}
    monkeypatch.setattr(audit.raw.adapter, "_load_native", lambda *a: pytest.fail("conversion forbidden"))
    report, images, anchors, snapshot = audit.diagnose_trial(*raw_trial)
    assert report["original_capture_status"] == "failed" and report["original_owner_exit_code"] == 1
    assert not report["raw_scalar_quality"]["raw_scalar_quality_clear"]
    assert len(images) == report["camera_pixels"]["decoded_count"] == 12
    assert len(anchors) == 2 and len(snapshot) == 4
    assert report["measured_future"]["available_64_point_anchor_count"] == 0
    assert all(a["disposition"] == "incomplete_future_retained" for a in anchors)
    assert original == {p: audit.raw.scalar.sha(p) for p in raw_trial[0].rglob("*") if p.is_file()}


@pytest.mark.parametrize("mutation", ["bad_source_commit", "missing_proof", "changed_metadata", "changed_bounds", "image_inventory", "wrong_case"])
def test_actual_reader_fails_closed_on_source_or_image_binding(raw_trial, mutation):
    root, trial, images, timeline, case = raw_trial
    if mutation == "bad_source_commit": trial["reviewed_execution_commit"] = "0" * 40
    elif mutation == "missing_proof": trial["all_eleven_archives_match_owner_and_reviewed_commits"] = False
    elif mutation == "changed_metadata":
        path = root / "episode.partial/manifest.json"
        path.write_bytes(path.read_bytes() + b"\n")
    elif mutation == "changed_bounds":
        path = root / "provenance/portable_e2e/model.py"
        path.write_bytes(b"wrong frozen bounds")
    elif mutation == "image_inventory": images.pop()
    else: case["profile"] = "turn_launch_014_v1"
    with pytest.raises(audit.raw.scalar.EvidenceError): audit.diagnose_trial(root, trial, images, timeline, case)


def test_no_payload_finalized_failure_remains_visible(raw_trial, tmp_path):
    root, trial, _, _, case = raw_trial
    trial["raw_data_available"] = False
    empty = tmp_path / "no_capture"
    trial["source_manifest"] = [e for e in trial["source_manifest"] if e["path"].startswith("provenance/")]
    for entry in trial["source_manifest"]:
        target = empty / entry["path"]
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes((root / entry["path"]).read_bytes())
    result = audit.diagnose_trial(empty, trial, [], None, case)
    assert result[0]["status"] == "NO_RAW_PAYLOAD_RETAINED"
    assert result[1:] == ([], [], [])


def test_no_payload_flag_cannot_hide_existing_native_files(raw_trial):
    root, trial, _, _, case = raw_trial
    trial["raw_data_available"] = False
    with pytest.raises(audit.raw.scalar.EvidenceError, match="unexplained measurements"):
        audit.diagnose_trial(root, trial, [], None, case)


def test_finalized_empty_failed_capture_remains_in_denominator(raw_trial):
    root, trial, images, _, case = raw_trial
    for entry in images:
        (root / entry["path"]).unlink()
    for name in ("states.jsonl", "camera_frames.jsonl"):
        (root / "episode.partial" / name).write_bytes(b"")
    for entry in trial["source_manifest"]:
        entry.update(sha256=audit.raw.scalar.sha(root / entry["path"]), size_bytes=(root / entry["path"]).stat().st_size)
    result = audit.diagnose_trial(root, trial, [], None, case)
    assert result[0]["status"] == "NO_NATIVE_OBSERVATIONS_RETAINED"
    assert result[0]["raw_native_state_count"] == 0 and not result[0]["training_data_approved"]


def test_new_dispatch_dependencies_are_source_bound():
    identity = audit.source_identity()
    for module in (audit.matrix, audit.matrix.turn, audit.matrix.initialization_audit):
        name = Path(module.__file__).resolve().relative_to(audit.ROOT).as_posix()
        assert identity["files"][name] == audit.raw.scalar.sha(Path(module.__file__))
