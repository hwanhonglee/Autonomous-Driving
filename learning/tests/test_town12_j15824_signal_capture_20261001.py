"""HH_261001 - Verify camera ABI, signal crossing rules and the owned capture boundary."""

import json
from pathlib import Path
import subprocess

import pytest

from scripts.e2e import capture_town12_j15824_signal_expert_20261001 as capture
from scripts.e2e import run_owned_town12_j15824_signal_capture_20261001 as runner


def test_existing_six_camera_geometry_is_preserved_with_disclosed_rgb_effects():
    contract = capture.camera_contract()
    assert len(contract["source_specs"]) == len(contract["applied_specs"]) == 6
    for source, applied in zip(contract["source_specs"], contract["applied_specs"]):
        assert source["enable_postprocess_effects"] is False
        assert applied["enable_postprocess_effects"] is True
        assert {k: v for k, v in source.items() if k != "enable_postprocess_effects"} == {
            k: v for k, v in applied.items() if k != "enable_postprocess_effects"}
        assert (applied["width"], applied["height"], applied["sensor_tick"]) == (640, 360, 0.0)
    assert [spec["name"] for spec in contract["applied_specs"]] == list(capture.collector.MODEL_CAMERA_ORDER)


def test_fixed_collector_arguments_keep_signals_source_identity_and_twenty_kmh(tmp_path):
    argv = capture.collector_arguments(tmp_path, tmp_path / "route.json")
    args = capture.collector.parse_args(argv)
    assert args.target_speed_kmh == 20.0
    assert args.physics_hz == 20.0 and args.capture_hz == 10.0
    assert args.allow_map_load is False and args.observe_traffic_lights is True
    assert args.expected_live_xodr_sha256 == capture.scout.XODR_SHA256
    assert args.control_transport == "acknowledged_batch"
    assert args.agent_initialization == "after_bootstrap"
    assert args.role_name == "hero"
    assert args.stationary_tail_sec >= 6.5
    assert not any("ignore" in item for item in argv)
    assert capture.collector.configuration_from_args(args).profile_id == "comfortable_v1"


@pytest.mark.parametrize("before,state,distance,speed,crossed,failed", [
    (None, "Red", 20.0, 5.0, False, False),
    ("Red", "Red", 1.0, 0.0, False, False),
    ("Red", "Red", -0.1, 0.0, False, True),
    ("Green", "Green", -0.1, 4.0, False, False),
    ("Green", "Yellow", -0.1, 4.0, False, True),
    ("Red", "Green", -0.1, 4.0, False, True),
    ("Green", "Off", 20.0, 4.0, False, True),
    ("Red", "Red", 0.25, 2.0, False, True),
    ("Green", "Red", -5.0, 4.0, True, False),
])
def test_trigger_boundary_crossing_requires_observed_green(before, state, distance, speed, crossed, failed):
    previous = {"signal_state": before} if before is not None else None
    current = {"signal_state": state, "front_bumper_stop_distance_m": distance, "speed_mps": speed}
    _, error = capture.signal_verdict(current, previous, crossed)
    assert (error is not None) is failed


def test_runtime_plan_cannot_take_another_lane_or_maneuver():
    route = json.loads((capture.scout.STATIC_OUTPUT / "route.json").read_text())["route"]
    capture.validate_plan(route, route)
    changed = [dict(point) for point in route]
    changed[len(changed) // 2]["lane_id"] = -4
    with pytest.raises(capture.scout.ScoutError, match="lane sequence"):
        capture.validate_plan(changed, route)
    changed = [dict(point) for point in route]
    changed[10]["road_option"] = "CHANGELANELEFT"
    with pytest.raises(capture.scout.ScoutError, match="another maneuver"):
        capture.validate_plan(changed, route)


@pytest.mark.parametrize("failure", ["wrong_world_proof", "wrong_route_hash"])
def test_missing_or_changed_scout_evidence_cannot_enable_capture(tmp_path, failure):
    route = tmp_path / "route.json"
    route.write_text("{}")
    receipt = {"status": "FAIL" if failure == "wrong_world_proof" else "PASS",
        "xodr_sha256": capture.scout.XODR_SHA256, "route_polyline_sha256": capture.scout.ROUTE_SHA256,
        "hold_wall_seconds": 10.0, "route_file_sha256": "0" * 64}
    (tmp_path / "scout_result.json").write_text(json.dumps(receipt))
    with pytest.raises(capture.scout.ScoutError):
        capture.validate_scout_receipt(tmp_path)


def test_default_owner_preflight_is_read_only_and_restores_scope(monkeypatch, tmp_path, capsys):
    owner = runner.scout_owner.diagnostic.owner
    original_output, original_sources = runner.scout_owner.OUTPUT, runner.scout_owner.SOURCE_FILES
    monkeypatch.setattr(runner, "OUTPUT", tmp_path / "new")
    monkeypatch.setattr(runner, "capture_preflight", lambda: {})
    monkeypatch.setattr(runner.scout_owner, "binding", lambda: {})
    monkeypatch.setattr(owner, "preflight", lambda: ({}, {}, {}, {}))
    monkeypatch.setattr(subprocess, "run", lambda command, **kwargs: subprocess.CompletedProcess(command, 0, "", ""))
    monkeypatch.setattr(subprocess, "Popen", lambda *a, **kw: pytest.fail("simulator launched"))
    assert runner.main([]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["simulator_started"] is False and report["capture_on_execute"] is True
    assert report["postprocess_effects"] is True and report["target_speed_kmh"] == 20.0
    assert report["training_data_admitted"] is False
    assert runner.scout_owner.OUTPUT == original_output
    assert runner.scout_owner.SOURCE_FILES is original_sources
    assert not runner.OUTPUT.exists()


def test_capture_archive_contains_collector_agent_and_camera_inputs():
    with runner.configured_owner():
        paths = runner.scout_owner.diagnostic.source_paths()
        assert "tools/e2e/collect_carla_vad_expert.py" in paths
        assert "tools/e2e/capture_town12_j15824_signal_expert_20261001.py" in paths
        assert "external/carla/PythonAPI/carla/agents/navigation/controller.py" in paths
        assert str(capture.collector.DEFAULT_MAPPING.relative_to(capture.ROOT)) in paths
        assert str(capture.collector.DEFAULT_CALIBRATION.relative_to(capture.ROOT)) in paths
