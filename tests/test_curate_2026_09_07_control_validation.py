import importlib.util
import json
import math
from pathlib import Path
import re
import struct


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = REPOSITORY_ROOT / "scripts/e2e/curate_2026_09_07_control_validation.py"
SPEC = importlib.util.spec_from_file_location("control_validation_curator", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
curator = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(curator)


def test_publication_destinations_are_unique_and_intentionally_bounded():
    destinations = [destination for _source, destination in curator._publication_specs()]

    assert len(destinations) == len(set(destinations))
    assert sum(name.endswith("autoware_drive_5fps.gif") for name in destinations) == 8
    strict_live = [
        name
        for name in destinations
        if name.startswith("05_60kph_readiness/strict10_live_v1/")
    ]
    assert len(strict_live) == 30
    assert sum(name.endswith(".gif") for name in strict_live) == 2
    assert all(not Path(name).is_absolute() for name in destinations)
    assert all(".." not in Path(name).parts for name in destinations)
    assert not any(name.endswith((".mkv", ".db3", ".log", ".bag")) for name in destinations)
    assert all(Path(name).suffix in {".png", ".gif", ".json", ".md"} for name in destinations)


def test_image_dimensions_reads_png_and_gif_headers(tmp_path):
    png = tmp_path / "frame.png"
    gif = tmp_path / "drive.gif"
    other = tmp_path / "evidence.json"
    png.write_bytes(b"\x89PNG\r\n\x1a\n" + b"\x00" * 8 + struct.pack(">II", 1920, 1080))
    gif.write_bytes(b"GIF89a" + struct.pack("<HH", 960, 540))
    other.write_text("{}\n", encoding="utf-8")

    assert curator._image_dimensions(png) == [1920, 1080]
    assert curator._image_dimensions(gif) == [960, 540]
    assert curator._image_dimensions(other) is None


def test_transformed_markdown_has_no_trailing_whitespace(tmp_path):
    report = tmp_path / "speed_limit_analysis.md"
    report.write_text("first  \nsecond\t\n", encoding="utf-8")

    transformed = curator._transform_published_file(
        report,
        "05_60kph_readiness/analysis/speed_limit_analysis.md",
    )

    assert transformed is True
    assert report.read_text(encoding="utf-8") == "first\nsecond\n"


def test_result_matrix_preserves_control_and_safety_boundaries():
    matrix = curator._read_json(
        curator.DEFAULT_OUTPUT / "00_summary/result_matrix.json"
    )

    assert matrix["control_owner"] == "autoware_vad"
    assert matrix["portable_e2e_role"] == "shadow_only"
    assert matrix["real_vehicle_ready"] is False
    assert len(matrix["arms"]) == 6
    assert not any(
        decision == "ACCEPT" for decision in matrix["campaign_decisions"].values()
    )
    town07_baseline = next(
        arm for arm in matrix["arms"] if arm["arm_id"] == "town07_baseline"
    )
    assert town07_baseline["retained_default_setting"] is True
    assert town07_baseline["arm_run_qualified"] is False
    assert "selected_setting" not in town07_baseline
    for arm in matrix["arms"]:
        assert arm["source_period_violation_count"] == 0
        assert arm["portable_shadow_analysis_status"] == "EVIDENCE_VALID"
        assert arm["camera_bundle_coverage_percent"] >= 99.0

    readiness = matrix["historical_60kph_readiness"]
    assert readiness["map"] == "Town06"
    assert readiness["speed_exposure_status"] == "FAIL"
    assert readiness["camera_source_frequency_hz"] == 5.0
    assert readiness["target_within_actuation_map_axis"] is False
    assert readiness["rerun_authorized"] is False
    assert len(readiness["blockers"]) == 3

    strict = matrix["strict10_live_60kph_result"]
    assert strict["pilot_status"] == "FAILED"
    assert strict["evidence_integrity_status"] == "PASS"
    assert strict["pre_engagement_runtime_health_status"] == "PASS"
    assert strict["standalone_post_run_camera_status"] == "PASS"
    assert strict["final_camera_transport_qualification_status"] == "FAILED"
    assert strict["physical_goal_completion_status"] == "PASS"
    assert strict["speed_exposure_contract_status"] == "FAILED"
    assert strict["simulation_pilot_acceptance_status"] == "FAILED"
    assert strict["real_vehicle_readiness_status"] == "BLOCKED"
    assert strict["goal_reached"] is True
    assert math.isclose(strict["maximum_observed_speed_mps"], 10.123258027166047)
    assert math.isclose(strict["maximum_observed_speed_kph"], 36.44372889779777)
    assert strict["maximum_sustained_15mps_duration_sec"] == 0.0
    assert math.isclose(strict["camera_bundle_receipt_p95_ms"], 45.71508195)
    assert strict["camera_bundle_receipt_limit_ms"] == 40.0
    assert strict["matched_camera_bundle_count"] == 864
    assert sorted(strict["camera_record_counts"].values()) == [864, 865, 865, 865, 865, 865]
    assert strict["full_record_set_source_stamp_integrity_status"] == "FAIL"
    assert len(strict["camera_failures"]) == 2
    assert matrix["campaign_decisions"]["60kph"] == "NO_GO_STRICT10_LIVE_V1"


def test_publish_replace_is_limited_to_same_managed_publication(tmp_path, monkeypatch):
    output = tmp_path / "publication"
    monkeypatch.setattr(curator, "_publication_specs", lambda: [])
    monkeypatch.setattr(
        curator,
        "_result_matrix",
        lambda: {"publication_id": curator.PUBLICATION_ID},
    )
    monkeypatch.setattr(curator, "_host_stall_summary", lambda: {"status": "PASS"})

    curator._publish(output)
    curator._verify(output)
    curator._publish(output, replace=True)
    curator._verify(output)

    manifest = output / "00_summary/publication_manifest.json"
    payload = curator._read_json(manifest)
    payload["publication_id"] = "different-publication"
    curator._write_json(manifest, payload)

    try:
        curator._publish(output, replace=True)
    except RuntimeError as error:
        assert str(error) == "refusing to replace a different publication"
    else:
        raise AssertionError("different publication identity was replaced")


def test_verify_rejects_manifest_schema_and_duplicate_destinations(tmp_path, monkeypatch):
    output = tmp_path / "publication"
    monkeypatch.setattr(curator, "_publication_specs", lambda: [])
    monkeypatch.setattr(
        curator,
        "_result_matrix",
        lambda: {"publication_id": curator.PUBLICATION_ID},
    )
    monkeypatch.setattr(curator, "_host_stall_summary", lambda: {"status": "PASS"})
    curator._publish(output)

    manifest_path = output / "00_summary/publication_manifest.json"
    manifest = curator._read_json(manifest_path)
    manifest["schema_id"] = "wrong-schema"
    curator._write_json(manifest_path, manifest)
    try:
        curator._verify(output)
    except RuntimeError as error:
        assert str(error) == "publication manifest schema mismatch"
    else:
        raise AssertionError("wrong publication manifest schema was accepted")

    manifest["schema_id"] = "autoware-e2e.curated-publication-manifest.v1"
    manifest["files"].append(dict(manifest["files"][0]))
    manifest["managed_file_count"] += 1
    curator._write_json(manifest_path, manifest)
    try:
        curator._verify(output)
    except RuntimeError as error:
        assert str(error).startswith("duplicate publication destination:")
    else:
        raise AssertionError("duplicate publication destination was accepted")


def test_published_markdown_relative_links_resolve():
    for markdown in curator.DEFAULT_OUTPUT.rglob("*.md"):
        for target in re.findall(r"\[[^]]*\]\(([^)]+)\)", markdown.read_text()):
            if target.startswith(("http://", "https://", "#")):
                continue
            relative = target.split("#", 1)[0]
            assert (markdown.parent / relative).resolve().exists(), (
                f"broken publication link in {markdown}: {target}"
            )


def test_published_runtime_stall_and_healthy_redraw_claims_are_reproducible():
    diagnostics = curator.DEFAULT_OUTPUT / "04_rejected_and_runtime_diagnostics"
    host = diagnostics / "host_stall"
    latency = json.loads((host / "e2e_latency.json").read_text())
    camera_periods = [
        event["receipt_period_sec"]["max"]
        for topic, event in latency["event_rates"].items()
        if "/sensing/camera/" in topic and topic.endswith("/camera_info")
    ]
    assert min(camera_periods) == 0.3111240863800049
    assert max(camera_periods) == 0.32337284088134766

    stall = json.loads((host / "host_runtime_stall_summary.json").read_text())
    route = stall["route_evaluation"]
    assert stall["status"] == "EVIDENCE_VALID"
    assert route["vmstat_sample_count"] == len(route["vmstat_samples"])
    assert max(item["run_queue"] for item in route["vmstat_samples"]) == 75
    assert route["run_queue_peak"] == 75
    assert len(stall["sources"]["vmstat"]["sha256"]) == 64

    healthy = json.loads(
        (diagnostics / "healthy_camera_reference/desktop_capture.json").read_text()
    )
    assert healthy["rviz_view_contract"]["redraw_rate_fps"] == 10
