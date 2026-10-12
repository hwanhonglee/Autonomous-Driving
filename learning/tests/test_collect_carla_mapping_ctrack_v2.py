"""HH_260929 - Test C-track capture gates without CARLA, GPU, or camera actors."""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path

import pytest

from portable_e2e.workspace_paths import source_path
from scripts.e2e import collect_carla_mapping_ctrack_v2 as collector


@pytest.fixture
def scoped_inputs(tmp_path, monkeypatch):
    # HH_260929 - Replace large installed assets with exact synthetic fixture bytes.
    routes = {}
    for scenario in ("straight", "left"):
        route = tmp_path / f"{scenario}.json"
        route.write_text(json.dumps({
            "town": collector.TOWN, "scenario": scenario,
            "coordinate_reference": "base_link",
            "start_carla_transform": {"x": 0, "y": 0, "z": 0, "yaw": 0},
            "route": [{"x": 0, "y": 0, "road_option_value": 4},
                      {"x": 1, "y": 0, "road_option_value": 4}],
        }))
        routes[scenario] = (route, hashlib.sha256(route.read_bytes()).hexdigest())
    monkeypatch.setattr(collector, "ROUTES", routes)
    carla_root = tmp_path / "fake-carla-package"
    assets = {}
    for relative in collector.PACKAGED_ASSETS:
        file = carla_root / relative
        file.parent.mkdir(parents=True, exist_ok=True)
        payload = b"<OpenDRIVE>fixture</OpenDRIVE>" if file.suffix == ".xodr" else b"fixture-umap"
        file.write_bytes(payload)
        assets[relative] = hashlib.sha256(payload).hexdigest()
    monkeypatch.setattr(collector, "PACKAGED_ASSETS", assets)
    monkeypatch.setattr(collector, "EXPECTED_LIVE_XODR_SHA256",
                        next(value for name, value in assets.items() if name.endswith(".xodr")))
    monkeypatch.setenv("CARLA_ROOT", str(carla_root))
    return {"routes": routes, "carla_root": carla_root,
            "output": tmp_path / "owned" / "mapping",
            "live_xodr": "<OpenDRIVE>fixture</OpenDRIVE>"}


def _argv(inputs, scenario):
    return [str(inputs["output"]), str(inputs["routes"][scenario][0]),
            "--host", "127.0.0.1", "--port", "2100",
            "--motion-mode", "basic-agent", "--frames", "240",
            "--ignore-traffic-lights", f"--ctrack-{scenario}"]


@pytest.mark.parametrize("scenario", ["straight", "left"])
def test_each_route_has_one_cpu_only_canonical_scope(scoped_inputs, scenario):
    args = collector.parse_args(_argv(scoped_inputs, scenario))
    route, digest = collector._route_binding(args.route_file, args.scenario)
    assert route["scenario"] == scenario
    assert digest == scoped_inputs["routes"][scenario][1]
    assert args.frames == 240 and args.motion_mode == "basic-agent"
    assert args.ignore_traffic_lights is True and args.allow_map_load is False
    assert not args.output.exists()


@pytest.mark.parametrize("change", [
    lambda args: args[:-1],
    lambda args: [*args[:-3], "239", *args[-2:]],
    lambda args: [*args[:-5], "kinematic", *args[-4:]],
    lambda args: [*args[:-2], args[-1]],
    lambda args: [*args, "--allow-map-load"],
    lambda args: [*args, "--ctrack-left"],
])
def test_unscoped_or_changed_motion_contract_is_rejected(scoped_inputs, change):
    with pytest.raises(SystemExit):
        collector.parse_args(change(_argv(scoped_inputs, "straight")))
    assert not scoped_inputs["output"].exists()


def test_route_path_hash_and_scenario_are_not_interchangeable(scoped_inputs):
    argv = _argv(scoped_inputs, "straight")
    argv[1] = str(scoped_inputs["routes"]["left"][0])
    with pytest.raises(SystemExit):
        collector.parse_args(argv)
    argv = _argv(scoped_inputs, "straight")
    straight = scoped_inputs["routes"]["straight"][0]
    straight.write_text(straight.read_text() + "\n")
    with pytest.raises(SystemExit):
        collector.parse_args(argv)


def test_existing_output_and_changed_packaged_assets_fail_before_carla(scoped_inputs):
    scoped_inputs["output"].parent.mkdir()
    scoped_inputs["output"].mkdir()
    with pytest.raises(SystemExit):
        collector.parse_args(_argv(scoped_inputs, "left"))
    scoped_inputs["output"].rmdir()
    asset = next(scoped_inputs["carla_root"].rglob("*.xodr"))
    asset.write_bytes(asset.read_bytes() + b"changed")
    with pytest.raises(SystemExit):
        collector.parse_args(_argv(scoped_inputs, "left"))


def test_v2_policy_and_frozen_v1_source_are_required(scoped_inputs, monkeypatch):
    assert len(collector._policy_preflight()) == 64
    monkeypatch.setattr(collector, "FROZEN_V1_COLLECTOR_SHA256", "0" * 64)
    with pytest.raises(SystemExit):
        collector.parse_args(_argv(scoped_inputs, "straight"))


def _owned_proof(scoped_inputs, args, monkeypatch):
    owner_root = args.output.parent
    owner_root.mkdir(parents=True)
    (owner_root / "lifecycle").mkdir()
    pid = 87654
    monkeypatch.setattr(collector.os, "getpgid", lambda observed: pid if observed == pid else -1)
    source_hashes = {}
    for logical in collector.REQUIRED_SOURCES:
        source = source_path(collector.ROOT, logical)
        data = source.read_bytes()
        archived = owner_root / "provenance" / logical
        archived.parent.mkdir(parents=True, exist_ok=True)
        archived.write_bytes(data)
        source_hashes[logical] = hashlib.sha256(data).hexdigest()
    route_sha = scoped_inputs["routes"][args.scenario][1]
    plan = {"capture_mode": collector.CAPTURE_MODE, "worker_path": collector.WORKER_SOURCE,
            "quality": "Epic", "map": collector.TOWN, "route_path": str(args.route_file),
            "route_sha256": route_sha, "host": args.host, "port": args.port,
            "collector_argv": collector._canonical_argv(args),
            "source_sha256": source_hashes}
    started = {"server_pid": pid, "server_pgid": pid, "quality": "Epic",
               "map": collector.TOWN, "route_sha256": route_sha, "port": args.port}
    ready = {"status": "PASS", "mode": "running", "stage": "ready",
             "generation_id": f"expert_{pid}", "owner_pid": pid, "owner_pgid": pid,
             "port": args.port, "host": args.host, "expected_map": collector.TOWN,
             "active_map_basename": collector.TOWN,
             "active_map_name": "Carla/Maps/C_track_1_0_7", "read_only": True}
    for name, value in (("owner_plan.json", plan), ("owner_started.json", started),
                        ("lifecycle/ready.json", ready)):
        (owner_root / name).write_text(json.dumps(value))
    return route_sha, plan, started, ready


def test_owner_proof_requires_epic_generation_and_archived_sources(scoped_inputs, monkeypatch):
    args = collector.parse_args(_argv(scoped_inputs, "left"))
    route_sha, plan, started, ready = _owned_proof(scoped_inputs, args, monkeypatch)
    proof = collector._owner_proof(args, route_sha, ready["active_map_name"])
    assert proof["owner_pid"] == started["server_pid"]
    plan["quality"] = "Low"
    (args.output.parent / "owner_plan.json").write_text(json.dumps(plan))
    with pytest.raises(ValueError, match="Epic plan"):
        collector._owner_proof(args, route_sha, ready["active_map_name"])
    plan["quality"] = "Epic"
    (args.output.parent / "owner_plan.json").write_text(json.dumps(plan))
    archived = args.output.parent / "provenance" / collector.WORKER_SOURCE
    archived.write_bytes(archived.read_bytes() + b"changed")
    with pytest.raises(ValueError, match="source provenance changed"):
        collector._owner_proof(args, route_sha, ready["active_map_name"])


def test_live_map_bytes_and_empty_world_are_required_before_actors(scoped_inputs, monkeypatch):
    args = collector.parse_args(_argv(scoped_inputs, "straight"))
    route_sha, _, _, _ = _owned_proof(scoped_inputs, args, monkeypatch)

    class FakeMap:
        name = "Carla/Maps/C_track_1_0_7"

        def to_opendrive(self):
            return scoped_inputs["live_xodr"]

    class FakeActors:
        occupied = False

        def filter(self, _kind):
            return [object()] if self.occupied else []

    class FakeWorld:
        carla_map = FakeMap()
        actors = FakeActors()

        def get_map(self):
            return self.carla_map

        def get_actors(self):
            return self.actors

        def spawn_actor(self, *_args, **_kwargs):
            raise AssertionError("CPU pre-actor test must never spawn an actor")

    world = FakeWorld()
    result = collector.validate_live_pre_actor(args, world, route_sha)
    assert result["live_xodr_sha256"] == collector.EXPECTED_LIVE_XODR_SHA256
    world.carla_map.name = "Other/Maps/C_track_1_0_7"
    with pytest.raises(ValueError, match="reviewed packaged map"):
        collector.validate_live_pre_actor(args, world, route_sha)
    world.carla_map.name = "Carla/Maps/C_track_1_0_7"
    scoped_inputs["live_xodr"] = "changed"
    with pytest.raises(ValueError, match="OpenDRIVE bytes differ"):
        collector.validate_live_pre_actor(args, world, route_sha)
    scoped_inputs["live_xodr"] = "<OpenDRIVE>fixture</OpenDRIVE>"
    world.actors.occupied = True
    with pytest.raises(ValueError, match="unexpected dynamic actors"):
        collector.validate_live_pre_actor(args, world, route_sha)
