"""HH_260929 - Fail closed on Town02 p00 research policy, source, and package drift."""

from __future__ import annotations

import copy
import json

import numpy as np
import pytest

from scripts.e2e import prepare_map_town02_p00_route_holdout_data as package


def _report():
    return json.loads(package.P00_AUDIT.read_text(encoding="utf-8"))


def _scope():
    return json.loads(package.RESEARCH_SCOPE.read_text(encoding="utf-8"))


def _scope_hashes():
    return {"p00_audit_sha256": package.P00_AUDIT_SHA256,
            "qa_sha256": package.QA_SHA256,
            "plan_sha256": package.PLAN_SHA256,
            "v2_split_sha256": package.V2_SPLIT_SHA256}


def test_real_sources_pass_read_only_preflight(tmp_path):
    output = tmp_path / "fresh"
    ready = package.preflight(output)
    assert len(ready["rows00"]) == len(ready["rows02"]) == 232
    assert ready["hashes"]["research_scope_sha256"] == package.RESEARCH_SCOPE_SHA256
    assert 197.039 < ready["gap_m"] < 197.041
    assert not output.exists()


@pytest.mark.parametrize("key,value", [
    ("site", "Town05_Opt"),
    ("source_route_sha256", "0" * 64),
    ("source_audit_sha256", "0" * 64),
    ("visual_qa_result_sha256", "0" * 64),
    ("preregistered_plan_sha256", "0" * 64),
    ("parent_site_split_v2_sha256", "0" * 64),
    ("source_retained_frames", 231),
    ("exploratory_ab_train_authorized", False),
    ("dataset_admitted", True),
    ("independent_dev_approved", True),
    ("automatic_model_promotion", True),
    ("test_use_authorized", True),
    ("vehicle_control_approved", True),
])
def test_research_scope_rejects_expansion(key, value):
    scope = copy.deepcopy(_scope())
    scope[key] = value
    with pytest.raises(ValueError, match="research scope changed"):
        package.validate_research_scope(scope, _scope_hashes())


@pytest.mark.parametrize("change", ["town", "route", "sign", "retention", "histogram"])
def test_source_report_rejects_drift(change):
    report = copy.deepcopy(_report())
    if change == "town":
        report["provenance"]["town"] = "Town05_Opt"
    elif change == "route":
        report["provenance"]["route_sha256"] = "0" * 64
    elif change == "sign":
        report["retained_sign_frame_bins"]["1-4"] = 24
    elif change == "retention":
        report["retained_stems"][0] = report["retained_stems"][1]
    else:
        report["frames"][0]["class_pixels"][0] += 1
    with pytest.raises(ValueError):
        package.validate_audit(report, status="REJECTED_SIGN_RICH_TRAIN_CANDIDATE",
                               route_sha256=package.p00_audit.ROUTE_SHA256,
                               sign_one_to_four=3)


def test_semantic_uses_frozen_seven_class_tag_table():
    semantic = np.zeros((360, 640, 4), dtype=np.uint8)
    semantic[0, :8, 2] = [0, 1, 2, 24, 7, 8, 6, 255]
    target = package.target_from_semantic(semantic)
    assert target.shape == (360, 640)
    assert target.dtype == np.uint8
    assert target[0, :8].tolist() == [0, 1, 2, 3, 4, 5, 6, 0]


def test_existing_output_refuses_before_source_audits(monkeypatch, tmp_path):
    existing = tmp_path / "existing"
    existing.mkdir()
    monkeypatch.setattr(package.p00_audit, "build_summary",
                        lambda: pytest.fail("source audit must not run"))
    with pytest.raises(ValueError, match="fresh output required"):
        package.preflight(existing)


def test_raw_pair_hash_drift_fails_before_decode(tmp_path):
    frames = tmp_path / "frames"
    frames.mkdir()
    (frames / "0000.png").write_bytes(b"not original RGB")
    (frames / "0000.npz").write_bytes(b"not original sensors")
    with pytest.raises(ValueError, match="raw pair hash changed"):
        package.decode_retained(tmp_path, {
            "stem": "frames/0000", "rgb_sha256": "0" * 64,
            "semantic_npz_sha256": "0" * 64})


def test_package_has_independent_source_to_package_receipt(tmp_path):
    if not package.OUTPUT.exists():
        pytest.skip("new CPU package has not been built")
    ready = package.preflight(tmp_path / "fresh")
    receipt = package.verify_package(package.OUTPUT, ready)
    assert receipt["status"] == "SOURCE_TO_PACKAGE_VERIFIED"
    assert receipt["p00_source_rows"] == receipt["p02_source_rows"] == 232
    assert receipt["manifest_sha256"] == package.digest(package.OUTPUT / "manifest.json")
