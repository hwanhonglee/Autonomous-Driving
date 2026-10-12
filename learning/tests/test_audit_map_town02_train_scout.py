"""HH_260906 - Guard the preregistered Town02 sign scout audit and rejection."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = ROOT / "tools/e2e/audit_map_town02_train_scout.py"
SPEC = importlib.util.spec_from_file_location("audit_map_town02_train_scout", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)


def test_whole_frame_bin_boundaries_and_preregistered_gate() -> None:
    assert audit.frame_histogram([0, 1, 4, 5, 64, 65]) == {
        "0": 1, "1-4": 2, "5-64": 2, ">=65": 1,
    }
    assert audit.qualification_status(119, {"1-4": 24}) == (
        "REJECTED_INSUFFICIENT_DISTINCT_POSITIONS"
    )
    assert audit.qualification_status(120, {"1-4": 23}) == (
        "REJECTED_SIGN_RICH_TRAIN_CANDIDATE"
    )
    assert audit.qualification_status(120, {"1-4": 24}) == (
        "SCOUT_SIGN_GATE_PASS_NOT_ADMITTED"
    )
    with pytest.raises(ValueError, match="invalid tag-8"):
        audit.sign_bin(-1)


def test_retention_compares_every_previously_kept_town02_position() -> None:
    rows = [{"ego_x": x, "ego_y": 0.0} for x in (0.0, 0.3, 0.1, 0.6)]
    kept, _ = audit.common.retain_indices(rows, minimum_m=0.25)
    assert kept == [0, 1, 3]


def test_real_source_stays_rejected_without_admission() -> None:
    result = audit.build_summary()
    assert result["raw_frames"] == 240
    assert result["retained_frames_0p25m"] == 232
    assert result["raw_sign_frame_bins"] == {
        "0": 0, "1-4": 4, "5-64": 164, ">=65": 72,
    }
    assert result["retained_sign_frame_bins"] == {
        "0": 0, "1-4": 3, "5-64": 157, ">=65": 72,
    }
    assert result["raw_tag8_pixel_sum"] == 16596
    assert result["retained_tag8_pixel_sum"] == 16544
    assert result["retained_1_to_4_stems"] == [
        "frames/0006", "frames/0016", "frames/0018",
    ]
    assert result["retained_gate_passed"] is True
    assert result["small_sign_gate_passed"] is False
    assert result["status"] == "REJECTED_SIGN_RICH_TRAIN_CANDIDATE"
    assert result["dataset_admitted"] is False
    assert result["model_predictions_made"] is False
    assert result["model_trained"] is False


def test_owner_rejects_false_collision_receipt(monkeypatch: pytest.MonkeyPatch) -> None:
    original = audit.common.read_json

    def changed(path: Path):
        data = original(path)
        if path == audit.OWNER / "mapping/summary.json":
            data["collision_events"] = 1
        return data

    monkeypatch.setattr(audit.common, "read_json", changed)
    with pytest.raises(ValueError, match="owner proof failed"):
        audit.audit_owner()
