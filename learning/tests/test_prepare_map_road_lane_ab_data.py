"""HH_260929 - Guard the frozen four-file road/lane data package boundary."""

from __future__ import annotations

import copy
import json

import numpy as np
import pytest

from scripts.e2e import prepare_map_road_lane_ab_data as package


def _town02_report():
    return json.loads(package.TOWN02_AUDIT.read_text(encoding="utf-8"))


def test_real_inputs_pass_read_only_preflight(tmp_path):
    ready = package.preflight(tmp_path / "fresh")
    assert len(ready["retained02"]) == len(ready["retained05"]) == 232
    assert ready["town02"]["small_sign_gate_passed"] is False
    assert not (tmp_path / "fresh").exists()


@pytest.mark.parametrize("change", ["town", "route", "small_sign", "retention", "class_pixels"])
def test_report_contract_rejects_source_or_split_drift(change):
    report = copy.deepcopy(_town02_report())
    if change == "town":
        report["provenance"]["town"] = "Town05_Opt"
    elif change == "route":
        report["provenance"]["route_sha256"] = "0" * 64
    elif change == "small_sign":
        report["small_sign_gate_passed"] = True
    elif change == "retention":
        report["retained_stems"][1] = report["retained_stems"][0]
    else:
        report["frames"][0]["class_pixels"][0] += 1
    with pytest.raises(ValueError):
        package.validate_report(report, town="Town02_Opt",
                                status="REJECTED_SMALL_SIGN_TRAIN_CANDIDATE",
                                route_sha256=package.town02_audit.ROUTE_SHA256)


def test_native_semantic_uses_exact_seven_class_table():
    semantic = np.zeros((360, 640, 4), dtype=np.uint8)
    semantic[0, :8, 2] = [0, 1, 2, 24, 7, 8, 6, 255]
    target = package.target_from_semantic(semantic)
    assert target.shape == (360, 640)
    assert target.dtype == np.uint8
    assert target[0, :8].tolist() == [0, 1, 2, 3, 4, 5, 6, 0]


def test_existing_output_refuses_before_source_audits(monkeypatch, tmp_path):
    existing = tmp_path / "existing"
    existing.mkdir()
    monkeypatch.setattr(package.town02_audit, "build_summary",
                        lambda: pytest.fail("source audit must not run"))
    with pytest.raises(ValueError, match="fresh output required"):
        package.preflight(existing)


def test_town01_source_path_overlap_stops_before_new_capture_audit(monkeypatch, tmp_path):
    parent = {"labels": list(package.LABELS), "splits": {
        "train": {"source_summary": {"route_sha256": "train"},
                  "source_hashes": {"same/raw.png": "sha"}},
        "dev": {"source_summary": {"route_sha256": "dev"},
                "source_hashes": {"same/raw.png": "sha"}},
    }}
    monkeypatch.setattr(package.expansion, "frozen_ai_v2", lambda _: parent)
    monkeypatch.setattr(package.town02_audit, "build_summary",
                        lambda: pytest.fail("new source audit must not run"))
    with pytest.raises(ValueError, match="source paths overlap"):
        package.preflight(tmp_path / "fresh")


def test_retained_frame_hash_drift_fails_before_decode(tmp_path):
    frame_dir = tmp_path / "frames"
    frame_dir.mkdir()
    (frame_dir / "0000.png").write_bytes(b"not an audited PNG")
    (frame_dir / "0000.npz").write_bytes(b"not an audited NPZ")
    with pytest.raises(ValueError, match="raw source hash changed"):
        package.decode_retained(tmp_path, {"stem": "frames/0000",
                                           "rgb_sha256": "0" * 64,
                                           "semantic_npz_sha256": "0" * 64})
