"""HH_260929 - Keep the focused derivative tied to completed V4 row 69."""

import hashlib
import json
from pathlib import Path

from scripts.e2e import scout_town12_junction3742_derivative_20260929 as scout
from scripts.e2e import scout_town12_right_static_20260929_v4 as v4


def test_derivative_pins_original_v4_source_and_junction_delta():
    plan = json.loads(scout.PREREG.read_text())
    assert plan["derivative_script_sha256"] == hashlib.sha256(
        Path(scout.__file__).read_bytes()).hexdigest()
    assert plan["v4_script_sha256"] == hashlib.sha256(Path(v4.__file__).read_bytes()).hexdigest()
    lines = v4.PROGRESS.read_bytes().splitlines(keepends=True)
    assert len(lines) >= 69
    first, second = json.loads(lines[67]), json.loads(lines[68])
    assert hashlib.sha256(lines[67]).hexdigest() == plan["row68_sha256"]
    assert hashlib.sha256(lines[68]).hexdigest() == plan["row69_sha256"]
    assert (first["completed_junctions"], second["completed_junctions"],
            second["last_completed_junction_id"]) == (68, 69, 3742)
    assert {key: second["cumulative_counts"][key] - first["cumulative_counts"][key]
            for key in scout.COUNT_KEYS} == plan["expected_count_delta"]
    assert plan["expected_count_delta"]["nominal_pass"] == 8
    assert plan["expected_count_delta"]["signal_record_free_nominal_pass"] == 0


def test_derivative_report_remains_partial_and_signal_constrained():
    report = json.loads(scout.OUTPUT.read_text())
    assert report["status"] == "DERIVATIVE_PARTIAL_NOT_FULL_SCOUT"
    assert report["preregistration_sha256"] == hashlib.sha256(
        scout.PREREG.read_bytes()).hexdigest()
    assert report["script_sha256"] == hashlib.sha256(Path(scout.__file__).read_bytes()).hexdigest()
    assert report["counts"] == json.loads(scout.PREREG.read_text())["expected_count_delta"]
    rows = report["physical_pass_routes"]
    assert len(rows) == len({row["route_sha256"] for row in rows}) == 10
    assert report["unique_physical_site_count"] == 1
    assert report["unique_connector_lane_section_count"] == 2
    nominal_pass = [row for row in rows if row["nominal"]["status"] == "PASS"]
    assert len(nominal_pass) == 8
    assert sum(row["nominal"]["production_lane_gate_supported"] for row in nominal_pass) == 4
    assert all(any(row["approach_signals"].values()) for row in nominal_pass)
    assert report["nominal_pass_signal_record_summary"]["approach_record_free_count"] == 0
    assert report["capture_authorized"] is False
    assert report["dataset_admitted"] is False


def test_derivative_route_rows_match_completed_v4_report():
    derivative = json.loads(scout.OUTPUT.read_text())
    full = json.loads(v4.OUTPUT.read_text())
    full_rows = {row["route_sha256"]: row for row in full["physical_pass_routes"]
                 if row["junction_id"] == scout.JUNCTION_ID}
    assert len(full_rows) == len(derivative["physical_pass_routes"]) == 10
    for row in derivative["physical_pass_routes"]:
        assert {key: value for key, value in row.items()
                if key != "route_xy_ros_m"} == full_rows[row["route_sha256"]]
