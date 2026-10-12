"""HH_260928 - Bound static Town13 sign clustering without simulator contact."""

from __future__ import annotations

import hashlib
from pathlib import Path

import pytest

from scripts.e2e import audit_town13_sign_cluster_feasibility as audit


def synthetic_xodr(path: Path) -> str:
    content = (
        '<OpenDRIVE><road id="7" length="180"><planView>'
        '<geometry s="0" x="0" y="0" hdg="0" length="180"><line/></geometry>'
        '</planView><signals>'
        '<signal id="s1" name="Sign_Stop" s="50" t="3" orientation="+"/>'
        '<signal id="y1" name="Sign_Yield" s="70" t="3" orientation="+"/>'
        '<signal id="s2" name="Sign_Stop" s="90" t="3" orientation="+"/>'
        '<signal id="tl" name="Signal_3Light_Post01" s="100" t="0"/>'
        '</signals></road><road id="8" length="180"><planView>'
        '<geometry s="0" x="400" y="0" hdg="0" length="180"><line/></geometry>'
        '</planView><signals>'
        '<signal id="far" name="Sign_Stop" s="50" t="3" orientation="-"/>'
        '</signals></road></OpenDRIVE>')
    path.write_text(content, encoding="utf-8")
    return hashlib.sha256(content.encode("utf-8")).hexdigest()


def test_pinned_sha_is_checked_before_parsing(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    xodr = tmp_path / "map.xodr"
    synthetic_xodr(xodr)

    def forbidden_parse(_path: Path):
        raise AssertionError("untrusted bytes parsed")

    monkeypatch.setattr(audit, "parse_signs", forbidden_parse)
    with pytest.raises(audit.AuditError, match="SHA256 differs"):
        audit.build_report(xodr, "0" * 64)


def test_three_distinct_records_and_same_road_order(tmp_path: Path) -> None:
    xodr = tmp_path / "map.xodr"
    expected = synthetic_xodr(xodr)
    report = audit.build_report(xodr, expected)
    assert report["status"] == "EUCLIDEAN_NECESSARY_CONDITION_PRESENT"
    assert report["counts"]["stop_count"] == 3
    assert report["counts"]["yield_count"] == 1
    assert report["counts"]["distinct_record_count"] == 4
    best = report["best_euclidean_clusters"][0]
    assert best["record_ids"] == ["7:s1", "7:s2", "7:y1"]
    assert best["minimum_possible_route_m"] == 48.0
    assert best["graph_route_proven"] is False
    assert best["camera_visibility_proven"] is False
    assert report["best_same_road_triples"][0]["record_ids"] == [
        "7:s1", "7:y1", "7:s2"]
    assert report["best_same_road_triples"][0]["s_span_m"] == 40.0
    assert report["catalog_authorized"] is False
    assert report["capture_authorized"] is False


def test_two_records_do_not_count_as_three() -> None:
    first = {"record_id": "1:a", "road_id": 1, "name": "Sign_Stop", "x_m": 0.0, "y_m": 0.0}
    second = {"record_id": "1:b", "road_id": 1, "name": "Sign_Yield", "x_m": 1.0, "y_m": 0.0}
    assert audit.three_sign_route_lower_bound([first, second]) == []


def test_xy_lower_bound_excludes_triplet_beyond_250m() -> None:
    signs = [
        {"record_id": "1:a", "road_id": 1, "name": "Sign_Stop", "x_m": 0.0, "y_m": 0.0},
        {"record_id": "1:b", "road_id": 1, "name": "Sign_Stop", "x_m": 160.0, "y_m": 0.0},
        {"record_id": "1:c", "road_id": 1, "name": "Sign_Stop", "x_m": 320.0, "y_m": 0.0},
    ]
    rows = audit.three_sign_route_lower_bound(signs)
    assert rows[0]["minimum_possible_route_m"] == 328.0
    assert all(row["minimum_possible_route_m"] > 250.0 for row in rows)


def test_duplicate_signal_record_is_fail_closed(tmp_path: Path) -> None:
    path = tmp_path / "duplicate.xodr"
    path.write_text(
        '<OpenDRIVE><road id="7" length="100"><planView>'
        '<geometry s="0" x="0" y="0" hdg="0" length="100"><line/></geometry>'
        '</planView><signals>'
        '<signal id="same" name="Sign_Stop" s="20" t="1"/>'
        '<signal id="same" name="Sign_Yield" s="40" t="1"/>'
        '</signals></road></OpenDRIVE>', encoding="utf-8")
    with pytest.raises(audit.AuditError, match="duplicate or absent"):
        audit.parse_signs(path)
