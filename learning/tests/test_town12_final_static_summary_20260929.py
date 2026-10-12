"""HH_260929 - Guard the completed Town12 static counts against promotion."""

from collections import Counter
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
DIR = ROOT / "docs/assets/validation/2026-09-29/drive_town12_right_static"
REPORT_SHA256 = "e4ebf3298c6a625126b98f78039dec3acc4cddbab7c2166883045c95d160c2c9"
LEDGER_SHA256 = "22538ac958051c5b3c9baded8344b56a9a30d01adbe3c1ae190dbd8a01af6a62"


def test_final_town12_scoped_static_result_and_split_barriers():
    report_path = DIR / "report_v4.json"
    report = json.loads(report_path.read_text())
    assert hashlib.sha256(report_path.read_bytes()).hexdigest() == REPORT_SHA256
    assert hashlib.sha256((DIR / "progress_v4.jsonl").read_bytes()).hexdigest() == LEDGER_SHA256
    assert report["progress_completed_junctions"] == 628
    assert report["progress_ledger_sha256"] == LEDGER_SHA256
    assert report["counts"]["physical_turn_pass"] == 4707
    assert {key: report["counts"][key] for key in
            ("nominal_pass", "nominal_fail", "nominal_unsupported")} == {
                "nominal_pass": 30, "nominal_fail": 4300, "nominal_unsupported": 377}
    routes = report["physical_pass_routes"]
    assert len(routes) == 4707
    passed = [row for row in routes if row["nominal"]["status"] == "PASS"]
    assert Counter(row["junction_id"] for row in passed) == {
        3742: 8, 15824: 16, 19928: 6}
    assert Counter((row["junction_id"], row["nominal"]["production_lane_gate_supported"])
                   for row in passed) == {
                       (3742, True): 4, (3742, False): 4,
                       (15824, True): 10, (15824, False): 6,
                       (19928, True): 6}
    assert all(any(row["approach_signals"].values()) and
               any(row["full_route_signals"].values()) and
               not row["other_junction_ids_in_30_146m"] for row in passed)
    split = json.loads((ROOT / "learning/config/site_split_guard_20260929.json").read_text())
    assert split["sites"]["Town12"]["drive"] == ["offline_train_scout"]
    assert report["capture_authorized"] is False
    assert report["dataset_admitted"] is False
    assert report["independent_dev_sites"] == 0
    assert report["training_executed"] is False
