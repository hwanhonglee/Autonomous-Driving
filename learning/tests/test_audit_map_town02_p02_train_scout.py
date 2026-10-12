"""HH_260906 - Guard Town02 p02 owner evidence and the fixed small-sign rejection."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "tools/e2e/audit_map_town02_p02_train_scout.py"
SPEC = importlib.util.spec_from_file_location("audit_map_town02_p02_train_scout", SOURCE)
assert SPEC is not None and SPEC.loader is not None
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)


@pytest.fixture(scope="module")
def source_summary() -> dict:
    return audit.build_summary()


def test_original_preregistered_boundaries_remain_unchanged() -> None:
    assert audit.base.MINIMUM_RETAINED == 120
    assert audit.base.MINIMUM_TINY_SIGN_FRAMES == 24
    assert audit.base.frame_histogram([0, 1, 4, 5, 64, 65]) == {
        "0": 1, "1-4": 2, "5-64": 2, ">=65": 1,
    }
    assert audit.base.qualification_status(232, {"1-4": 0}) == (
        "REJECTED_SIGN_RICH_TRAIN_CANDIDATE"
    )


def test_actual_p02_fails_only_small_sign_objective(source_summary: dict) -> None:
    assert source_summary["status"] == "REJECTED_SMALL_SIGN_TRAIN_CANDIDATE"
    assert source_summary["provenance"]["route_sha256"] == audit.ROUTE_SHA256
    assert source_summary["provenance"]["quality"] == "Epic"
    assert source_summary["provenance"]["archived_source_count"] == 13
    assert source_summary["provenance"]["collision_events"] == 0
    assert source_summary["provenance"]["live_replay_exact_match"] is True
    assert source_summary["raw_frames"] == 240
    assert source_summary["retained_frames_0p25m"] == 232
    assert source_summary["retained_gate_passed"] is True
    assert source_summary["small_sign_gate_passed"] is False
    assert source_summary["raw_sign_frame_bins"] == {
        "0": 0, "1-4": 0, "5-64": 149, ">=65": 91,
    }
    assert source_summary["retained_sign_frame_bins"] == {
        "0": 0, "1-4": 0, "5-64": 149, ">=65": 83,
    }
    assert source_summary["retained_tag8_pixel_sum"] == 58074
    assert source_summary["retained_1_to_4_stems"] == []
    assert source_summary["other_sign_scale_admission_not_decided"] is True
    assert source_summary["dataset_admitted"] is False
    assert source_summary["model_predictions_made"] is False
    assert source_summary["model_trained"] is False


def test_full_fov_published_artifacts_reproduce(source_summary: dict) -> None:
    assert source_summary["visual_stems"] == [
        "frames/0109", "frames/0083", "frames/0239",
    ]
    picture = audit.preview(source_summary)
    assert picture.size == (1940, 1306)
    audit.write_or_check(source_summary, picture)


def test_p02_owner_rejects_wrong_sha_and_scope(monkeypatch: pytest.MonkeyPatch) -> None:
    with pytest.raises(ValueError, match="pinned route bytes"):
        audit.base.audit_owner(audit.OWNER, audit.ROUTE, route_sha256="0" * 64,
                               research_split_intent=audit.RESEARCH_INTENT,
                               collector_scope=audit.COLLECTOR_SCOPE)

    original = audit.base.common.read_json

    def altered(path: Path):
        data = original(path)
        if path == audit.OWNER / "mapping/summary.json":
            data["research_split_intent"] = "town02_sign_train_scout_only"
        return data

    monkeypatch.setattr(audit.base.common, "read_json", altered)
    with pytest.raises(ValueError, match="owner proof failed"):
        audit.base.audit_owner(audit.OWNER, audit.ROUTE,
                               route_sha256=audit.ROUTE_SHA256,
                               research_split_intent=audit.RESEARCH_INTENT,
                               collector_scope=audit.COLLECTOR_SCOPE)
