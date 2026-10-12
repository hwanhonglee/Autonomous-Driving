"""HH_260906 - Verify create-only numerical publication, retained failures and exact file hashes with synthetic records."""

import copy
import hashlib
import json

import pytest

from scripts.e2e import curate_portable_decoder_forward as curate


def fixture():
    candidate = {"initialization_index": 0, "compared_scalar_count": 192, "original_runtime_gate": {"status": "FAIL", "reason": "kept"},
        "mismatches": [], "mismatch_scalar_count": 0, "status": "MATCH_WITH_PREDECLARED_TOLERANCE",
        "maximum_absolute_difference_by_component": {"x_m": 0., "y_m": 0., "speed_mps": 0.}}
    row = {"anchor_index": 0, "sample_id": "run_001:1", "trial_id": "run_001", "original_raw_violating_step_counts": {"xy_curvature": 1},
        "candidates": [{**copy.deepcopy(candidate), "initialization_index": k} for k in range(6)]}
    original = {**row, "candidates": [{"runtime_gate_status": "FAIL", "runtime_gate_reason": "kept"} for _ in range(6)]}
    summary = {"anchor_count": 1, "candidate_count": 6, "point_count": 384, "compared_scalar_count": 1152,
        "mismatch_scalar_count": 0, "mismatch_candidate_count": 0, "maximum_absolute_difference_by_component": candidate["maximum_absolute_difference_by_component"],
        "original_runtime_gate_counts": {"FAIL": 6}, "status": "MATCHED_NOT_ADMITTED"}
    return summary, [row], [original]


def test_recounts_original_failure_and_every_candidate():
    curate.validate_rows(*fixture())


@pytest.mark.parametrize("mode", ["omit_candidate", "hide_failure", "wrong_total", "wrong_status", "duplicate_mismatch", "inside_tolerance", "nonfinite_max"])
def test_false_pass_or_inconsistent_denominators_rejected(mode):
    summary, rows, originals = fixture(); c = rows[0]["candidates"][0]
    if mode == "omit_candidate": rows[0]["candidates"].pop()
    elif mode == "hide_failure": c["original_runtime_gate"] = {"status": "PASS", "reason": None}
    elif mode == "wrong_total": summary["compared_scalar_count"] = 1151
    elif mode == "wrong_status": summary["status"] = "UNVERIFIED_NOT_ADMITTED"
    elif mode == "nonfinite_max": c["maximum_absolute_difference_by_component"]["x_m"] = float("nan")
    else:
        delta = .00002 if mode == "duplicate_mismatch" else .000001
        item = {"point_index": 0, "component": "x_m", "recorded_gpu_value": 0., "replayed_cpu_value": delta, "absolute_difference": delta}
        c.update(mismatches=[item, item] if mode == "duplicate_mismatch" else [item], mismatch_scalar_count=2 if mode == "duplicate_mismatch" else 1,
            status="UNVERIFIED")
        c["maximum_absolute_difference_by_component"]["x_m"] = delta
    with pytest.raises(ValueError): curate.validate_rows(summary, rows, originals)


def test_actual_unverified_mismatch_is_retained_not_rejected_for_result_quality():
    summary, rows, originals = fixture(); c = rows[0]["candidates"][0]
    c.update(status="UNVERIFIED", mismatch_scalar_count=1, mismatches=[{"point_index": 63, "component": "speed_mps",
        "recorded_gpu_value": 0., "replayed_cpu_value": .00002, "absolute_difference": .00002}])
    c["maximum_absolute_difference_by_component"]["speed_mps"] = .00002
    summary.update(status="UNVERIFIED_NOT_ADMITTED", mismatch_scalar_count=1, mismatch_candidate_count=1,
        maximum_absolute_difference_by_component={"x_m": 0., "y_m": 0., "speed_mps": .00002})
    curate.validate_rows(summary, rows, originals)


def test_file_hashes_and_private_paths_fail_before_publication(tmp_path):
    path = tmp_path / "summary.json"; path.write_text("{}")
    sums = tmp_path / "SHA256SUMS"; sums.write_text(hashlib.sha256(b"{}").hexdigest() + "  summary.json\n")
    assert curate.exact_files(tmp_path, ("summary.json", "SHA256SUMS"), checksum_manifest=True)["summary.json"] == b"{}"
    path.write_text('{"changed":true}')
    with pytest.raises(ValueError): curate.exact_files(tmp_path, ("summary.json", "SHA256SUMS"), checksum_manifest=True)
    path.write_text('{"private":"/home/example/data"}')
    with pytest.raises(ValueError, match="private account"): curate.exact_files(tmp_path, ("summary.json",))


def test_symlink_and_parent_escape_rejected(tmp_path):
    (tmp_path / "real.json").write_text("{}")
    (tmp_path / "alias.json").symlink_to(tmp_path / "real.json")
    for name in ("alias.json", "../real.json"):
        with pytest.raises(ValueError): curate.exact_files(tmp_path, (name,))


def test_existing_or_input_nested_output_rejected_before_reads(tmp_path, monkeypatch):
    monkeypatch.setattr(curate, "exact_files", lambda *args, **kwargs: pytest.fail("must reject before reading"))
    for output in (tmp_path, tmp_path / "nested"):
        with pytest.raises(ValueError): curate.publish(tmp_path, tmp_path, tmp_path, tmp_path, output)


def test_json_duplicate_keys_are_rejected():
    with pytest.raises(ValueError): curate.parse_rows(b'{"x":1,"x":2}\n')


def test_initial_summary_must_bind_every_record_not_only_same_count():
    records = [{"anchor_index": 2, "initialization_index": 3, "absolute_difference_m2": .00002}]
    summary = {"initial_forward_unverified_count": 1, "initial_forward_unverified_candidates": copy.deepcopy(records),
        "status": "INITIALIZATION_UNVERIFIED_NOT_ADMITTED"}
    curate.validate_initial_summary(summary, records)
    summary["initial_forward_unverified_candidates"][0]["absolute_difference_m2"] = .000001
    with pytest.raises(ValueError): curate.validate_initial_summary(summary, records)


def test_initial_summary_cannot_upgrade_partial_numerical_disagreement_to_pass():
    with pytest.raises(ValueError): curate.validate_initial_summary({"initial_forward_unverified_count": 0,
        "initial_forward_unverified_candidates": [], "status": "VERIFIED_NOT_ADMITTED"}, [])
