"""HH_261001 - Source-backed lift, physical-check failures, and unchanged owner boundaries."""

import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest

from scripts.e2e import capture_town12_j15824_spawn_lift_20261001 as capture
from scripts.e2e import run_owned_town12_j15824_spawn_lift_20261001 as runner


def route():
    return json.loads((capture.scout.STATIC_OUTPUT / "route.json").read_text())


def record():
    return {**capture.spawn_geometry(route(), capture.nominal_body()), "ground_samples": [], "clearance_rays": []}


class Vector:
    def __init__(self, x=0, y=0, z=0):
        self.x, self.y, self.z = x, y, z


class World:
    def __init__(self, ground_delta=0, label="CityObjectLabel.Roads", hit=False, actors=()):
        self.ground_delta, self.label, self.hit, self.actors = ground_delta, label, hit, actors

    def get_map(self):
        return SimpleNamespace(get_waypoint=lambda *a, **kw: SimpleNamespace(road_id=588, section_id=0, lane_id=1))

    def get_actors(self):
        return self.actors

    def ground_projection(self, origin, distance):
        return SimpleNamespace(label=self.label, location=Vector(origin.x, origin.y, origin.z - 5 + self.ground_delta))

    def project_point(self, origin, direction, distance):
        return SimpleNamespace(label="Wall", location=origin) if self.hit else None


CARLA = SimpleNamespace(Location=Vector, Vector3D=Vector)


def test_exactly_one_argument_factor_changed_and_no_pose_motion(tmp_path):
    before = capture.ORIGINAL_ARGUMENTS(tmp_path, tmp_path / "route.json")
    after = capture.collector_arguments(tmp_path, tmp_path / "route.json")
    assert after == before + ["--spawn-z-offset-m", "0.5"]
    args = capture.collector.parse_args(after)
    assert args.spawn_z_offset_m == 0.5 and args.target_speed_kmh == 20
    assert args.observe_traffic_lights and not args.allow_map_load
    data = record()
    assert data["actor_spawn"]["z"] - data["map_actor_center"]["z"] == 0.5
    assert all(data["actor_spawn"][key] == data["map_actor_center"][key] for key in ("x", "y", "roll", "pitch", "yaw"))


def test_nominal_bounding_box_explains_possible_zero_offset_ground_overlap():
    data = record()
    assert data["nominal_body_bottom_z_m"] - data["map_actor_center"]["z"] == pytest.approx(0.4617525935)


def test_sparse_clearance_records_ground_rays_and_explicit_limits():
    data = record()
    capture.physical_spawn_checks(World(), CARLA, route(), data)
    assert len(data["ground_samples"]) == 9 and len(data["clearance_rays"]) == 27
    assert data["minimum_nominal_body_ground_gap_m"] > 0.46
    assert "not an exhaustive" in data["limitation"]
    assert data["preexisting_dynamic_actors"] == []


@pytest.mark.parametrize("world,reason", [
    (World(ground_delta=.3), "ground elevation"),
    (World(label="Sidewalks"), "not road surface"),
    (World(hit=True), "physical blocker"),
    (World(actors=[SimpleNamespace(id=17, type_id="vehicle.foreign")]), "foreign vehicle"),
    (World(actors=[SimpleNamespace(id=18, type_id="walker.pedestrian")]), "foreign vehicle"),
])
def test_blocked_spawn_check_does_not_spawn_retry_or_destroy(world, reason):
    with pytest.raises(capture.scout.ScoutError, match=reason):
        capture.physical_spawn_checks(world, CARLA, route(), record())


def test_missing_ground_and_wrong_start_lane_fail_closed():
    world = World()
    world.ground_projection = lambda *a: None
    with pytest.raises(capture.scout.ScoutError, match="ground is unavailable"):
        capture.physical_spawn_checks(world, CARLA, route(), record())
    world = World()
    world.get_map = lambda: SimpleNamespace(get_waypoint=lambda *a, **kw: None)
    with pytest.raises(capture.scout.ScoutError, match="fixed start lane"):
        capture.physical_spawn_checks(world, CARLA, route(), record())


def test_render_appearance_disclosure_preserves_all_geometry_and_blocks_legacy_adapter():
    contract = capture.camera_contract()
    assert contract["common10_legacy_adapter_compatible"] is False
    assert "map alone" in contract["ablation_caveat"]
    for source, applied in zip(contract["source_specs"], contract["applied_specs"]):
        assert source["enable_postprocess_effects"] is False and applied["enable_postprocess_effects"] is True
        assert {k: v for k, v in source.items() if k != "enable_postprocess_effects"} == {
            k: v for k, v in applied.items() if k != "enable_postprocess_effects"}


def test_worker_configuration_restores_originals_after_failure():
    before = capture.base.collector_arguments, capture.base.camera_contract, capture.collector.run
    with pytest.raises(RuntimeError):
        with capture.configured_worker():
            assert capture.base.collector_arguments is capture.collector_arguments
            raise RuntimeError("failure")
    assert (capture.base.collector_arguments, capture.base.camera_contract, capture.collector.run) == before


def test_dispatch_replaces_only_fixed_worker_and_keeps_owner_identity(monkeypatch):
    seen = []
    monkeypatch.setattr(subprocess, "run", lambda argv, **kw: seen.append((argv, kw)))
    command = ["python", str(runner.ROOT / "tools/workspace_python.py"),
        str(runner.ROOT / "tools/e2e/capture_town12_j15824_signal_expert_20261001.py"),
        "--execute", "--owner-pid", "101", "--owner-pgid", "101"]
    runner.dispatch_worker(command, timeout=900)
    assert seen[0][0][2] == str(runner.WORKER)
    assert seen[0][0][:2] + seen[0][0][3:] == command[:2] + command[3:]
    assert seen[0][1]["timeout"] == 900
    runner.dispatch_worker(["python", "-c", "import carla"])
    assert seen[1][0] == ["python", "-c", "import carla"]
    with pytest.raises(capture.scout.ScoutError, match="argv changed"):
        runner.dispatch_worker([command[2], "--execute"])


def test_new_owner_archive_and_scopes_restore():
    before = (runner.inherited.OUTPUT, runner.inherited.SOURCE_FILES, runner.inherited.capture,
              runner.inherited.subprocess, runner.inherited.scout_owner.EXTERNAL_SOURCES)
    with runner.configured_owner():
        assert runner.inherited.OUTPUT == runner.OUTPUT
        with runner.inherited.configured_owner():
            paths = runner.inherited.scout_owner.diagnostic.source_paths()
            assert "tools/e2e/capture_town12_j15824_spawn_lift_20261001.py" in paths
            assert "tools/e2e/capture_town12_j15824_signal_expert_20261001.py" in paths
            assert any(name.endswith("CarlaGameModeBase.cpp") for name in paths)
            assert str(capture.NOMINAL_MANIFEST.relative_to(capture.ROOT)) in paths
    assert (runner.inherited.OUTPUT, runner.inherited.SOURCE_FILES, runner.inherited.capture,
            runner.inherited.subprocess, runner.inherited.scout_owner.EXTERNAL_SOURCES) == before


def test_default_owner_never_launches_or_creates_output(monkeypatch, tmp_path, capsys):
    owner = runner.inherited.scout_owner.diagnostic.owner
    monkeypatch.setattr(runner, "OUTPUT", tmp_path / "new")
    monkeypatch.setattr(runner.inherited, "capture_preflight", lambda: {})
    monkeypatch.setattr(runner.inherited.scout_owner, "binding", lambda: {})
    monkeypatch.setattr(owner, "preflight", lambda: ({}, {}, {}, {}))
    monkeypatch.setattr(subprocess, "run", lambda command, **kwargs: subprocess.CompletedProcess(command, 0, "", ""))
    monkeypatch.setattr(subprocess, "Popen", lambda *a, **kw: pytest.fail("simulator launched"))
    assert runner.main([]) == 0
    assert json.loads(capsys.readouterr().out)["simulator_started"] is False
    assert not runner.OUTPUT.exists()
