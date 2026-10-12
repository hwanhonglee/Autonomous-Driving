"""HH_261001 - Check exact route, actor alignment and owner scope without CARLA RPC."""

import json
import math
import subprocess
import sys
from types import SimpleNamespace

import pytest

from scripts.e2e import scout_town12_j15824_live_20261001 as scout
from scripts.e2e import run_owned_town12_j15824_signal_scout_20261001 as runner


def points():
    return [{"x": float(x), "y": 0.0, "distance_m": float(x), "road_id": 588, "lane_id": 1}
            for x in range(0, 221)]


def waypoint(x, road=588, lane=1):
    return SimpleNamespace(road_id=road, section_id=0, lane_id=lane, s=10.0,
        transform=SimpleNamespace(location=SimpleNamespace(x=float(x), y=0.0, z=334.0),
                                  rotation=SimpleNamespace(yaw=0.0)))


def test_projection_uses_carla_to_ros_y_and_progress():
    route = [{"x": 0.0, "y": 0.0, "distance_m": 0.0, "road_id": 1, "lane_id": 1},
             {"x": 0.0, "y": 10.0, "distance_m": 10.0, "road_id": 1, "lane_id": 1}]
    result = scout.project(route, 0.5, -4.0)
    assert result["progress_m"] == pytest.approx(4.0)
    assert result["cross_track_m"] == pytest.approx(0.5)


@pytest.mark.parametrize("failure", [None, "absent", "duplicate", "dormant", "affected_lane", "stop_off_route", "trigger_lane"])
def test_signal_identity_and_stop_boundary_fail_closed(monkeypatch, failure):
    monkeypatch.setitem(sys.modules, "agents.tools.misc", SimpleNamespace(get_trafficlight_trigger_location=lambda a: None))
    stop = waypoint(70)
    if failure == "stop_off_route":
        stop.transform.location.y = 2.0
    actor = SimpleNamespace(id=42, is_dormant=failure == "dormant",
        get_opendrive_id=lambda: scout.SIGNAL_ID, get_state=lambda: "Red",
        get_affected_lane_waypoints=lambda: [waypoint(80, road=15851, lane=2 if failure == "affected_lane" else 1)],
        get_stop_waypoints=lambda: [stop])
    actors = [] if failure == "absent" else [actor, actor] if failure == "duplicate" else [actor]
    world = SimpleNamespace(get_actors=lambda: SimpleNamespace(filter=lambda pattern: actors))
    carla_map = SimpleNamespace(get_waypoint=lambda location: waypoint(70, lane=2 if failure == "trigger_lane" else 1))
    if failure:
        with pytest.raises(scout.ScoutError):
            scout.signal_actor_record(world, carla_map, points())
    else:
        result = scout.signal_actor_record(world, carla_map, points())
        assert result["actor_id"] == 42
        assert result["selected_stop_waypoint"]["route_projection"]["progress_m"] == 70.0
        assert result["painted_stop_line_alignment_proven"] is False
        assert result["basic_agent_red_hazard_runtime_proven"] is False


@pytest.mark.parametrize("failure", ["absent", "elevation"])
def test_ground_query_does_not_substitute_offline_waypoints_for_physical_geometry(failure):
    route = [dict(point, z=334.0) for point in points()]
    hit = None if failure == "absent" else SimpleNamespace(location=SimpleNamespace(z=340.0))
    world = SimpleNamespace(ground_projection=lambda *a: hit)
    carla = SimpleNamespace(Location=lambda *a: a)
    with pytest.raises(scout.ScoutError):
        scout.ground_evidence(world, carla, route)


def test_exact_saved_static_route_and_signal_definition():
    row = scout.static_binding()
    route_path = scout.STATIC_OUTPUT / "route.json"
    assert scout.digest(route_path) == runner.STATIC_ROUTE_SHA256
    route = json.loads(route_path.read_text())
    assert scout.route_digest(route["route"]) == scout.ROUTE_SHA256
    assert math.isclose(row["route_length_m"], 221.75968112638571)
    xml = scout.signal_xml()
    assert xml["signal_id"] == "21120"
    assert xml["definitions"][0]["road_id"] == 15844
    assert any(item["road_id"] == 15851 and "Right" in "".join(item["user_data_xml"])
               for item in xml["references"])


def test_read_only_owner_preflight_has_no_launch(monkeypatch, tmp_path, capsys):
    owner = runner.diagnostic.owner
    monkeypatch.setattr(runner, "OUTPUT", tmp_path / "attempt")
    monkeypatch.setattr(runner, "binding", lambda: {})
    monkeypatch.setattr(owner, "preflight", lambda: (
        {"executable": "/pinned/UE4Editor", "project": "/pinned/project"},
        {"load_name": owner.MAP_LOAD_NAME}, {}, {}))
    monkeypatch.setattr(subprocess, "Popen", lambda *a, **kw: pytest.fail("simulator launched"))
    original_output, original_sources = runner.diagnostic.OUTPUT, runner.diagnostic.SOURCE_FILES
    assert runner.main([]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["simulator_started"] is False
    assert report["source_editor_argv"][-2:] == [runner.diagnostic.OVERRIDE, runner.diagnostic.QUERY]
    assert runner.diagnostic.OUTPUT == original_output
    assert runner.diagnostic.SOURCE_FILES is original_sources
    assert not runner.OUTPUT.exists()


@pytest.mark.parametrize("worker_status", ["PASS", "FAIL", "wrong_route"])
def test_owner_accepts_only_complete_fixed_scout_receipt(monkeypatch, tmp_path, worker_status):
    owner = runner.diagnostic.owner
    monkeypatch.setattr(runner, "OUTPUT", tmp_path)
    monkeypatch.setattr(owner, "_wait_ready", lambda *a: None)
    monkeypatch.setattr(owner, "_read_live_xodr", lambda env: {"sha256": scout.XODR_SHA256})
    (tmp_path / "server.log").write_text(
        'LogConfig: Setting CVar [[r.GenerateMeshDistanceFields:0]]\n'
        'r.GenerateMeshDistanceFields = "0" LastSetBy: ProjectSetting\n')

    def fake_run(command, **kwargs):
        assert command[-4:] == ["--owner-pid", "12345", "--owner-pgid", "12345"]
        assert kwargs["timeout"] == 420
        receipt = {"status": "FAIL" if worker_status == "FAIL" else "PASS",
            "route_polyline_sha256": "wrong" if worker_status == "wrong_route" else scout.ROUTE_SHA256,
            "xodr_sha256": scout.XODR_SHA256, "hold_wall_seconds": 10.1,
            "controlling_signal": {"opendrive_id": scout.SIGNAL_ID}}
        (tmp_path / "scout/scout_result.json").write_text(json.dumps(receipt))
        return subprocess.CompletedProcess(command, 1 if worker_status == "FAIL" else 0)

    monkeypatch.setattr(subprocess, "run", fake_run)
    with runner.configured_owner():
        owner._wait_ready(SimpleNamespace(pid=12345), {})
        if worker_status == "PASS":
            assert owner._read_live_xodr({})["signal_scout"]["status"] == "PASS"
        else:
            with pytest.raises(owner.Town12OwnerError):
                owner._read_live_xodr({})


def test_source_provenance_includes_new_layers_and_external_signal_implementation():
    with runner.configured_owner():
        paths = runner.diagnostic.source_paths()
        assert "tools/e2e/scout_town12_j15824_live_20261001.py" in paths
        assert "external/carla/LibCarla/source/carla/client/TrafficLight.cpp" in paths
        assert str(scout.REPORT.relative_to(scout.ROOT)) in paths
        assert str((scout.STATIC_OUTPUT / "route.json").relative_to(scout.ROOT)) in paths
