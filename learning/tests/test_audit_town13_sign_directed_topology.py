"""HH_260928 - Test graph-screened sign triples without CARLA or UE4Editor."""

from __future__ import annotations

import hashlib
from pathlib import Path

import pytest

from scripts.e2e import audit_town13_sign_cluster_feasibility as cluster
from scripts.e2e import audit_town13_sign_directed_topology as topology


def source(path: Path, middle_orientation: str = "+") -> str:
    text = (
        '<OpenDRIVE><road id="1" length="60"><link>'
        '<successor elementType="road" elementId="2" contactPoint="start"/>'
        '</link><planView><geometry s="0" x="0" y="0" hdg="0" length="60">'
        '<line/></geometry></planView><signals>'
        '<signal id="a" name="Sign_Stop" s="40" t="1" orientation="+"/>'
        '</signals></road>'
        '<road id="2" length="60"><link>'
        '<predecessor elementType="road" elementId="1" contactPoint="end"/>'
        '<successor elementType="road" elementId="3" contactPoint="start"/>'
        '</link><planView><geometry s="0" x="60" y="0" hdg="0" length="60">'
        '<line/></geometry></planView><signals>'
        f'<signal id="b" name="Sign_Yield" s="30" t="1" orientation="{middle_orientation}"/>'
        '</signals></road>'
        '<road id="3" length="60"><link>'
        '<predecessor elementType="road" elementId="2" contactPoint="end"/>'
        '</link><planView><geometry s="0" x="120" y="0" hdg="0" length="60">'
        '<line/></geometry></planView><signals>'
        '<signal id="c" name="Sign_Stop" s="20" t="1" orientation="+"/>'
        '</signals></road>'
        '<junction id="9"><connection id="0" incomingRoad="1" '
        'connectingRoad="2" contactPoint="start"/></junction></OpenDRIVE>')
    path.write_text(text, encoding="utf-8")
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def test_directed_three_sign_sequence_and_distance(tmp_path: Path) -> None:
    path = tmp_path / "graph.xodr"
    expected = source(path)
    report = topology.build_report(path, expected)
    assert report["sequence_count"] >= 1
    first = report["best_sequences"][0]
    assert first["record_ids_in_travel_order"] == ["1:a", "2:b", "3:c"]
    assert first["sign_to_sign_route_span_m"] == 100.0
    assert first["minimum_route_with_20m_ends_m"] == 140.0
    assert first["lane_legal_route_proven"] is False
    assert report["nominal_distance_buckets"]["100"] == 0
    assert report["capture_authorized"] is False


def test_opposing_sign_orientation_is_not_counted(tmp_path: Path) -> None:
    path = tmp_path / "opposed.xodr"
    expected = source(path, middle_orientation="-")
    report = topology.build_report(path, expected)
    assert report["sequence_count"] == 0
    assert report["status"] == "STOP_NO_PARSED_DIRECTED_THREE_SIGN_SEQUENCE"


def test_junction_connection_tracks_incoming_road(tmp_path: Path) -> None:
    path = tmp_path / "junction.xodr"
    source(path)
    roads, junctions = topology.parse_road_graph_source(path)
    roads[1]["links"]["successor"] = {"elementType": "junction", "elementId": "9"}
    edges = topology.directed_neighbors(roads, junctions)
    assert edges[(1, "+")] == [(2, "+")]
    assert edges[(1, "-")] == []


def test_source_sha_mismatch_prevents_graph_parse(tmp_path: Path,
                                                   monkeypatch: pytest.MonkeyPatch) -> None:
    path = tmp_path / "graph.xodr"
    source(path)

    def forbidden(_path: Path):
        raise AssertionError("untrusted graph parsed")

    monkeypatch.setattr(topology, "parse_road_graph_source", forbidden)
    with pytest.raises(cluster.AuditError, match="SHA256 differs"):
        topology.build_report(path, "0" * 64)
