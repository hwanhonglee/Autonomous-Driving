"""HH_260906 - Preserve raw target-audit evidence and count overlapping windows honestly."""

import copy
import hashlib
import json
from pathlib import Path

import pytest

from portable_e2e.contract import ContractError
from scripts.e2e import render_portable_target_feasibility as render


@pytest.fixture
def report():
    metric = {"violating_anchor_count": 6,
              "violating_anchors_by_capture_phase": {"pre_tail": 5, "stationary_tail": 3}}
    group = {"sample_count": 10, "horizons": {horizon: {"sample_count": 10,
        "horizon_eligible_anchor_count": 10, "invalid_step_count": 0,
        "metrics": {name: copy.deepcopy(metric) for name in (render.SPEED_METRIC, render.XY_METRIC)}}
        for horizon in render.HORIZONS}}
    return {"schema": "portable_e2e.target_feasibility_audit.v1", "status": "TARGET_ENVELOPE_AUDIT_COMPLETE",
        "scope": {"test_sample_data_opened": False, "labels_modified": False, "model_inference_run": False},
        "physical_decoder_limits": {"acceleration_and_deceleration_mps2": 2.9},
        "dataset_manifest_sha256": "1" * 64, "contract_sha256": "2" * 64, "model_source_sha256": "3" * 64,
        "audit_script_sha256": render._sha(render.REPO / "scripts/e2e/audit_portable_target_feasibility.py"),
        "input_manifest": [{"path": "episodes/train/samples.jsonl", "sha256": "4" * 64}],
        "splits": {split: copy.deepcopy(group) for split in ("train", "val")},
        "episodes": [{"episode_id": split, "map_id": "Town03", "split": split, "overall": copy.deepcopy(group)}
                     for split in ("train", "val")]}


def test_phase_partition_uses_union_and_intersection_not_double_counted_sum(report):
    data = render.derive_chart_data(report)
    assert len(data["horizons"]) == 6
    row = data["capture_phases_6p4s"][0]
    assert row["before_tail_only"] == 3
    assert row["both_phases"] == 2
    assert row["tail_only"] == 1
    assert row["total_violating_anchors"] == 6


@pytest.mark.parametrize("mutation,match", [
    (lambda r: r.update(status="INCOMPLETE"), "incomplete"),
    (lambda r: r["scope"].update(test_sample_data_opened=True), "scope"),
    (lambda r: r["input_manifest"][0].update(path="/home/private/file"), "relative paths"),
    (lambda r: r["splits"]["train"].update(sample_count=True), "nonnegative integers"),
    (lambda r: r["episodes"][0].update(split="test"), "held-out"),
    (lambda r: r["episodes"][0]["overall"]["horizons"]["6.4s"]["metrics"][render.SPEED_METRIC]
        .update(violating_anchor_count=9), "union is inconsistent"),
])
def test_incomplete_private_or_inconsistent_inputs_fail_closed(report, mutation, match):
    mutation(report)
    with pytest.raises(ContractError, match=match):
        render.derive_chart_data(report)


def test_publication_preserves_exact_bytes_real_png_and_all_checksums(report, tmp_path):
    pytest.importorskip("matplotlib")
    source = tmp_path / "source.json"
    source.write_text(json.dumps(report, indent=3) + "\n")
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    output = tmp_path / "category"
    proof = render.render_report(source, digest, output)
    assert (output / "target_feasibility.json").read_bytes() == source.read_bytes()
    assert proof["audit_executed_source_commit"] is None
    assert proof["runtime_gate_reference_only"]["used_for_chart_violation_counts"] is False
    assert len(list(output.glob("*.png"))) == 2
    for path in output.glob("*.png"):
        assert path.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    manifest = (output / "SHA256SUMS").read_text().splitlines()
    assert len(manifest) == 6
    for line in manifest:
        expected, name = line.split("  ", 1)
        assert hashlib.sha256((output / name).read_bytes()).hexdigest() == expected
    before = {path.name: path.read_bytes() for path in output.iterdir()}
    with pytest.raises(ContractError, match="already exists"):
        render.render_report(source, digest, output)
    assert before == {path.name: path.read_bytes() for path in output.iterdir()}
    with pytest.raises(ContractError, match="SHA-256 mismatch"):
        render.render_report(source, "0" * 64, tmp_path / "wrong-hash")
    assert not (tmp_path / "wrong-hash").exists()


def test_changed_audit_source_is_not_represented_as_executed_source(report, tmp_path):
    report["audit_script_sha256"] = "0" * 64
    source = tmp_path / "source.json"
    source.write_text(json.dumps(report))
    with pytest.raises(ContractError, match="executed source bytes"):
        render.render_report(source, render._sha(source), tmp_path / "category")
    assert not (tmp_path / "category").exists()
