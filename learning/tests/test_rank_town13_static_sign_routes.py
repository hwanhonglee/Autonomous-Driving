"""HH_260906 - Keep Town13 static route ranking independent of camera outcomes."""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from scripts.e2e import rank_town13_static_sign_routes as rank


def test_line_and_arc_signal_coordinates() -> None:
    line = [{"s": 0.0, "x": 0.0, "y": 0.0, "hdg": 0.0,
             "length": 10.0, "curvature": 0.0}]
    assert rank.signal_xy(line, 5.0, 2.0) == pytest.approx((5.0, 2.0))
    arc = [{"s": 0.0, "x": 0.0, "y": 0.0, "hdg": 0.0,
            "length": math.pi / 2, "curvature": 1.0}]
    assert rank.signal_xy(arc, math.pi / 2, 0.0) == pytest.approx((1.0, 1.0))
    assert rank.signal_xy(arc, math.pi / 2, 2.0) == pytest.approx((-1.0, 1.0))


def test_static_signs_are_only_stop_and_yield_on_used_roads(tmp_path: Path) -> None:
    path = tmp_path / "synthetic.xodr"
    path.write_text(
        '<OpenDRIVE><road id="7"><planView>'
        '<geometry s="0" x="0" y="11" hdg="0" length="100"><line/></geometry>'
        '</planView><signals>'
        '<signal id="s1" name="Sign_Stop" s="50" t="3" orientation="+"/>'
        '<signal id="l1" name="Signal_3Light_Post01" s="55" t="3"/>'
        '<signal id="y1" name="Sign_Yield" s="70" t="-3" orientation="-"/>'
        '</signals></road><road id="8"><planView>'
        '<geometry s="0" x="0" y="0" hdg="0" length="100"><line/></geometry>'
        '</planView><signals><signal id="s2" name="Sign_Stop" s="50" t="3"/>'
        '</signals></road></OpenDRIVE>', encoding="utf-8")
    signs = rank.mapped_signs(path, {7})
    assert [(item["signal_id"], item["name"]) for item in signs] == [
        ("s1", "Sign_Stop"), ("y1", "Sign_Yield")]
    assert signs[0]["x"] == pytest.approx(50.0)
    assert signs[0]["y"] == pytest.approx(14.0)


def test_road_id_proximity_approach_tail_and_fixed_order() -> None:
    def route(route_id: str, road: int) -> tuple[dict, dict]:
        item = {"id": route_id, "scenario": "straight", "path": f"routes/town13/{route_id}.json",
                "sha256": "a" * 64}
        data = {"route_length_m": 100.0,
                "route": [{"x": float(i), "y": 0.0, "distance_m": float(i), "road_id": road}
                          for i in range(0, 101, 10)]}
        return item, data

    signs = [{"road_id": 7, "signal_id": "middle", "name": "Sign_Stop",
              "orientation": "+", "x": 50.0, "y": 3.0},
             {"road_id": 7, "signal_id": "start", "name": "Sign_Stop",
              "orientation": "+", "x": 10.0, "y": 3.0},
             {"road_id": 7, "signal_id": "far", "name": "Sign_Yield",
              "orientation": "-", "x": 70.0, "y": 9.0},
             {"road_id": 8, "signal_id": "wrong_road", "name": "Sign_Stop",
              "orientation": "+", "x": 50.0, "y": 0.0}]
    rows = rank.rank_routes([route("b", 7), route("a", 7), route("c", 9)], signs)
    assert [row["route_id"] for row in rows] == ["a", "b", "c"]
    assert rows[0]["mapped_sign_count"] == 1
    assert rows[0]["mapped_sign_opportunities"][0]["signal_id"] == "middle"
    assert rows[0]["camera_visibility_proven"] is False
    assert rows[0]["capture_authorized"] is False
    assert rows[2]["mapped_sign_count"] == 0


def test_owner_failure_stops_before_catalog_read(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    owner = tmp_path / "owner"
    owner.mkdir()
    (owner / "owner_result.json").write_text(json.dumps({"schema":
        "portable_e2e.owned_town13_catalog_result.v1", "status": "FAIL"}), encoding="utf-8")
    real_read = rank.read_json

    def checked_read(path: Path) -> dict:
        if path.name == "route_catalog.json":
            raise AssertionError("partial catalog consumed")
        return real_read(path)

    monkeypatch.setattr(rank, "read_json", checked_read)
    with pytest.raises(rank.RankError, match="incomplete, or failed"):
        rank.build_report(owner, tmp_path / "synthetic.xodr")


def test_missing_owner_stops_without_rank_file(tmp_path: Path) -> None:
    with pytest.raises(rank.RankError, match="owner_result.json"):
        rank.build_report(tmp_path / "owner", tmp_path / "synthetic.xodr")
