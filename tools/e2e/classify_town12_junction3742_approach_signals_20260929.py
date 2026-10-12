#!/usr/bin/env python3
"""HH_260929 - Classify pinned OpenDRIVE approach records without signal state."""

from __future__ import annotations

import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
DIR = ROOT / "docs/assets/validation/2026-09-29/drive_town12_right_static"
SOURCE = Path("/home/a/carla/Unreal/CarlaUE4/Content/Carla/Maps/Town12/OpenDrive/Town12.xodr")
DERIVATIVE = DIR / "junction3742_derivative.json"
OUTPUT = DIR / "junction3742_approach_signal_classification.json"
SOURCE_SHA256 = "26fa53d4902ac3397d35f948969f0c77c1bf023755ede2c67edf00bef817151f"
DERIVATIVE_SHA256 = "67fe9da01d529761e4276cec56167c46256e40c892fffa6ba3bb94ad35f5773c"
ROUTE_SHA256 = "629f4bfc07758e5fe787f51fcd01cef6315bce69f36fa0f3530e058dcac0eb31"
SNIPPETS = (
    (1062, "signal", "35843", 169503, 169503),
    (3743, "signal", "21131", 275054, 275059),
    (3745, "signalReference", "21131", 275146, 275151),
)


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def main() -> None:
    require(SOURCE.is_file() and not SOURCE.is_symlink() and
            DERIVATIVE.is_file() and not DERIVATIVE.is_symlink() and
            not OUTPUT.exists() and not OUTPUT.is_symlink(),
            "fixed input required and classification output must be fresh")
    source = SOURCE.read_bytes()
    require(digest(source) == SOURCE_SHA256 and
            digest(DERIVATIVE.read_bytes()) == DERIVATIVE_SHA256,
            "pinned XODR or route derivative changed")
    derivative = json.loads(DERIVATIVE.read_text())
    matches = [row for row in derivative["physical_pass_routes"]
               if row["route_sha256"] == ROUTE_SHA256]
    require(len(matches) == 1 and derivative["status"] ==
            "DERIVATIVE_PARTIAL_NOT_FULL_SCOUT", "selected route changed")
    route = matches[0]
    require(route["start"]["road_id"] == 1062 and route["start"]["lane_id"] == -1 and
            (route["junction_id"], route["connector_road_id"],
             route["connector_section_id"], route["connector_lane_id"]) ==
            (3742, 3745, 0, -1) and
            route["nominal"]["status"] == "PASS" and
            route["nominal"]["production_lane_gate_supported"] is True and
            route["approach_signals"] == {
                "direct_signal_road_ids": [1062],
                "signal_reference_road_ids": [3745],
            }, "selected route or approach-road inventory changed")
    roads = {int(road.attrib["id"]): road for road in ET.fromstring(source).findall("road")}
    lines = source.splitlines(keepends=True)
    records = []
    for road_id, kind, signal_id, first, last in SNIPPETS:
        require(road_id in roads and first > 0 and last <= len(lines),
                "signal owner or source span absent")
        elements = [element for element in roads[road_id].findall("./signals/" + kind)
                    if element.get("id") == signal_id]
        require(len(elements) == 1, "signal ID is not unique on pinned owner road")
        raw = b"".join(lines[first - 1:last])
        snippet = ET.fromstring(raw)
        element = elements[0]
        require(snippet.tag == kind and snippet.attrib == element.attrib and
                [(child.tag, child.attrib) for child in snippet.iter() if child is not snippet] ==
                [(child.tag, child.attrib) for child in element.iter() if child is not element],
                "raw XML span differs from pinned road element")
        records.append({
            "owner_road_id": road_id,
            "xml_kind": kind,
            "id": signal_id,
            "source_lines_inclusive": [first, last],
            "raw_xml_sha256": digest(raw),
            "raw_xml": raw.decode("utf-8"),
            "attributes": dict(element.attrib),
            "validity": [dict(item.attrib) for item in element.findall("validity")],
            "vector_signal": [dict(item.attrib) for item in
                              element.findall("./userData/vectorSignal")],
        })
    direct, definition, reference = records
    require(len(roads[1062].findall("./signals/signal")) == 1 and
            len(roads[3745].findall("./signals/signalReference")) == 1 and
            direct["attributes"]["name"] == "StopLine2" and
            direct["attributes"]["dynamic"] == "no" and
            direct["attributes"]["type"] == "-1" and
            definition["attributes"]["name"] == "Signal_3Light_Post01" and
            definition["attributes"]["dynamic"] == "yes" and
            definition["attributes"]["type"] == "1000001" and
            reference["validity"] == [{"fromLane": "-1", "toLane": "-1"}] and
            reference["vector_signal"][0]["turnRelation"] == "Right" and
            reference["vector_signal"][0]["signalId"] ==
            definition["vector_signal"][0]["signalId"],
            "static signal type, lane, or right-turn relation changed")
    report = {
        "schema": "portable_e2e.town12_junction3742_approach_signal_classification.v1",
        "status": "STATIC_XML_RECORD_CLASSIFICATION_ONLY",
        "source_xodr_path": str(SOURCE),
        "source_xodr_sha256": SOURCE_SHA256,
        "derivative_report_sha256": DERIVATIVE_SHA256,
        "selected_route_sha256": ROUTE_SHA256,
        "selected_connector": [3742, 3745, 0, -1],
        "records": records,
        "classification": {
            "road_1062_direct": "StopLine2, non-dynamic type -1; a stop-line-named record, not a stop-sign proof",
            "road_3745_reference": "signalReference 21131, validity lane -1 and userData turnRelation Right",
            "resolved_definition": "road 3743 Signal_3Light_Post01, dynamic yes, type 1000001; three-light-named signal asset",
            "matching_vector_signal_id": reference["vector_signal"][0]["signalId"],
            "orientation": "raw orientations retained; route-specific legal applicability and state not determined",
            "limitation": "OpenDRIVE metadata does not prove live signal state, stop policy, or safe entry",
        },
        "capture_authorized": False,
        "dataset_admitted": False,
        "independent_dev_sites": 0,
    }
    OUTPUT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"status": report["status"],
                      "signal_ids": [row["id"] for row in records]}, indent=2))


if __name__ == "__main__":
    main()
