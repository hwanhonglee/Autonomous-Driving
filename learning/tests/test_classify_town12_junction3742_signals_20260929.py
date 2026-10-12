"""HH_260929 - Keep the derivative signal classification tied to raw XML."""

import hashlib
import json
import xml.etree.ElementTree as ET

from scripts.e2e import classify_town12_junction3742_approach_signals_20260929 as scout


def test_junction3742_approach_records_have_pinned_raw_snippets():
    report = json.loads(scout.OUTPUT.read_text())
    assert report["status"] == "STATIC_XML_RECORD_CLASSIFICATION_ONLY"
    assert report["source_xodr_sha256"] == hashlib.sha256(scout.SOURCE.read_bytes()).hexdigest()
    assert report["derivative_report_sha256"] == hashlib.sha256(
        scout.DERIVATIVE.read_bytes()).hexdigest()
    records = report["records"]
    assert [(item["owner_road_id"], item["xml_kind"], item["id"])
            for item in records] == [(1062, "signal", "35843"),
                                    (3743, "signal", "21131"),
                                    (3745, "signalReference", "21131")]
    for item in records:
        raw = item["raw_xml"].encode("utf-8")
        assert hashlib.sha256(raw).hexdigest() == item["raw_xml_sha256"]
        assert ET.fromstring(raw).get("id") == item["id"]
    assert records[0]["attributes"]["dynamic"] == "no"
    assert records[0]["attributes"]["type"] == "-1"
    assert records[1]["attributes"]["dynamic"] == "yes"
    assert records[1]["attributes"]["type"] == "1000001"
    assert records[2]["validity"] == [{"fromLane": "-1", "toLane": "-1"}]
    assert records[2]["vector_signal"][0]["turnRelation"] == "Right"
    assert report["capture_authorized"] is False
