"""HH_260928 - Keep a Town13 camera scout distinct from map DEV admission."""

from contextlib import contextmanager
import hashlib
import json
from pathlib import Path
import subprocess

import pytest

from scripts.e2e import run_owned_town13_mapping_scout as owner


def _binding():
    profile = {
        "kind": "source_editor", "executable": "/exact/UE4Editor",
        "project": "/exact/CarlaUE4.uproject", "python_api": "/exact/source.egg",
    }
    entry = dict(owner.catalog_owner.EXPECTED_MAP)
    assets = {"project": {"path": profile["project"], "ok": True}}
    route = owner.CATALOG_ROOT / owner.ROUTE_RELATIVE
    binding = {
        "catalog_path": str(owner.CATALOG_ROOT / "route_catalog.json"),
        "catalog_sha256": owner.CATALOG_SHA256,
        "catalog_owner_result_path": str(owner.CATALOG_ROOT / "owner_result.json"),
        "route_path": str(route), "route_sha256": owner.ROUTE_SHA256,
        "route_id": owner.ROUTE_ID, "start_spawn_index": 1797,
        "goal_spawn_index": 8186,
    }
    return profile, entry, assets, {}, binding


@contextmanager
def _owned_lock():
    yield


def test_default_is_read_only_and_exactly_one_route(monkeypatch, tmp_path, capsys):
    binding = _binding()
    monkeypatch.setattr(owner, "preflight", lambda: binding)
    monkeypatch.setattr(owner, "OUTPUT", tmp_path / "absent")
    monkeypatch.setattr(subprocess, "Popen", lambda *a, **kw: pytest.fail("launched CARLA"))
    assert owner.main([]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["status"] == "STATIC_READY_NOT_LIVE_TESTED"
    assert report["simulator_started"] is False
    assert report["map_dev_admitted"] is False
    assert report["training_data_admitted"] is False
    assert report["collector_argv"][-1] == "--town13-render-sensor-scout"
    assert not (tmp_path / "absent").exists()


def test_fixed_collector_argv_requires_physical_240_frame_scope(monkeypatch, tmp_path):
    monkeypatch.setattr(owner, "OUTPUT", tmp_path / "scout")
    args = owner.collector_arguments(_binding()[-1])
    owner._assert_collector_arguments(args)
    assert args[args.index("--frames") + 1] == "240"
    assert args[args.index("--motion-mode") + 1] == "basic-agent"
    assert "--ignore-traffic-lights" in args
    with pytest.raises(owner.ScoutOwnerError, match="fixed scope"):
        owner._assert_collector_arguments(args[:-1])
    with pytest.raises(owner.ScoutOwnerError, match="fixed scope"):
        owner._assert_collector_arguments(
            ["other", *args[1:]]
        )


def _write_catalog_fixture(tmp_path: Path, monkeypatch):
    catalog_root = tmp_path / "catalog"
    route_path = catalog_root / owner.ROUTE_RELATIVE
    route_path.parent.mkdir(parents=True)
    route = {
        "town": "Town13", "coordinate_reference": "base_link",
        "scenario": "right", "start_spawn_index": 1797,
        "goal_spawn_index": 8186, "route": [{"x": 1}, {"x": 2}],
    }
    route_path.write_text(json.dumps(route))
    route_sha = hashlib.sha256(route_path.read_bytes()).hexdigest()
    catalog = {
        "status": "complete", "map_id": "town13",
        "server_profile": owner.catalog_owner.PROFILE_ID,
        "load_name": owner.catalog_owner.MAP_LOAD_NAME,
        "routes": [{"id": owner.ROUTE_ID, "status": "ready",
                    "scenario": "right", "path": owner.ROUTE_RELATIVE.as_posix(),
                    "sha256": route_sha, "start_spawn_index": 1797,
                    "goal_spawn_index": 8186}],
    }
    catalog_path = catalog_root / "route_catalog.json"
    catalog_path.write_text(json.dumps(catalog))
    catalog_sha = hashlib.sha256(catalog_path.read_bytes()).hexdigest()
    (catalog_root / "owner_result.json").write_text(json.dumps({
        "status": "PASS", "catalog": {"sha256": catalog_sha},
        "source_assets_unchanged": True,
        "source_bytes_unchanged_and_archived": True,
        "owned_processes_stopped_and_ports_released": True,
    }))
    monkeypatch.setattr(owner, "CATALOG_ROOT", catalog_root)
    monkeypatch.setattr(owner, "CATALOG_SHA256", catalog_sha)
    monkeypatch.setattr(owner, "ROUTE_SHA256", route_sha)
    return catalog_root, route_path


def test_catalog_route_requires_owner_proof_and_exact_bytes(monkeypatch, tmp_path):
    catalog_root, route_path = _write_catalog_fixture(tmp_path, monkeypatch)
    binding = owner._catalog_route_binding()
    assert binding["route_sha256"] == owner.ROUTE_SHA256
    route_path.write_text(route_path.read_text() + " ")
    with pytest.raises(owner.ScoutOwnerError, match="bytes changed"):
        owner._catalog_route_binding()
    route_path.write_text(json.dumps({
        "town": "Town13", "coordinate_reference": "base_link",
        "scenario": "right", "start_spawn_index": 1797,
        "goal_spawn_index": 8186, "route": [{"x": 1}, {"x": 2}],
    }))
    result_path = catalog_root / "owner_result.json"
    result = json.loads(result_path.read_text())
    result["status"] = "FAIL"
    result_path.write_text(json.dumps(result))
    with pytest.raises(owner.ScoutOwnerError, match="owner proof"):
        owner._catalog_route_binding()


def test_source_route_and_catalog_archives_are_rechecked(monkeypatch, tmp_path):
    _write_catalog_fixture(tmp_path, monkeypatch)
    binding = owner._catalog_route_binding()
    root = tmp_path / "workspace"
    source = root / "tools/e2e/example.py"
    source.parent.mkdir(parents=True)
    source.write_text("# immutable capture source\n")
    monkeypatch.setattr(owner, "ROOT", root)
    monkeypatch.setattr(owner, "SOURCE_FILES", ("tools/e2e/example.py",))
    monkeypatch.setattr(owner, "OUTPUT", tmp_path / "scout")
    owner._make_output()
    hashes = owner._archive_sources_and_route(binding)
    assert hashes["route"] == owner.ROUTE_SHA256
    assert hashes["catalog.json"] == owner.CATALOG_SHA256
    assert owner._verify_sources_and_route(hashes, binding)
    (owner.OUTPUT / "provenance/catalog_owner_result.json").write_text("changed")
    assert not owner._verify_sources_and_route(hashes, binding)


@pytest.mark.parametrize("blocker", ["occupied_ports", "existing_output", "runtime_lock"])
def test_runtime_blockers_refuse_before_launch(monkeypatch, tmp_path, blocker):
    profile, entry, assets, env, binding = _binding()
    monkeypatch.setattr(owner, "preflight", lambda: (profile, entry, assets, env, binding))
    monkeypatch.setattr(owner, "OUTPUT", tmp_path / "scout")
    monkeypatch.setattr(owner.catalog_owner, "_assert_no_other_carla", lambda: None)
    monkeypatch.setattr(subprocess, "Popen", lambda *a, **kw: pytest.fail("launched CARLA"))
    if blocker == "runtime_lock":
        @contextmanager
        def busy():
            raise owner.catalog_owner.CatalogOwnerError("runtime lock occupied")
            yield
        monkeypatch.setattr(owner, "_workspace_lock", busy)
    else:
        monkeypatch.setattr(owner, "_workspace_lock", _owned_lock)
    if blocker == "occupied_ports":
        monkeypatch.setattr(owner.catalog_owner, "_assert_ports_free",
                            lambda: (_ for _ in ()).throw(owner.catalog_owner.CatalogOwnerError("port occupied")))
    else:
        monkeypatch.setattr(owner.catalog_owner, "_assert_ports_free", lambda: None)
    if blocker == "existing_output":
        owner.OUTPUT.mkdir()
    with pytest.raises((owner.ScoutOwnerError, owner.catalog_owner.CatalogOwnerError)):
        owner.execute(profile, entry, assets, env, binding)
    assert blocker == "existing_output" or not owner.OUTPUT.exists()


def test_probe_rejects_same_basename_under_other_full_map(monkeypatch, tmp_path):
    monkeypatch.setattr(owner, "OUTPUT", tmp_path)
    lifecycle = tmp_path / "lifecycle"
    lifecycle.mkdir()
    (lifecycle / "ready.json").write_text(json.dumps({
        "status": "PASS", "active_map_name": "/Game/Other/Town13",
    }))
    monkeypatch.setattr(subprocess, "run", lambda *a, **kw: subprocess.CompletedProcess(a[0], 0, "", ""))
    with pytest.raises(owner.ScoutOwnerError, match="full map name"):
        owner._probe("ready", 43210, {})
    (lifecycle / "ready.json").write_text(json.dumps({
        "status": "PASS", "active_map_name": "Carla/Maps/Town13/Town13",
    }))
    assert owner._probe("ready", 43210, {})["status"] == "PASS"


def test_capture_assessment_is_diagnostic_only_and_requires_every_pair(monkeypatch, tmp_path):
    monkeypatch.setattr(owner, "OUTPUT", tmp_path)
    mapping = tmp_path / "mapping"
    frames = mapping / "frames"
    frames.mkdir(parents=True)
    summary = {
        "status": "COMPLETE_ORACLE_PLUMBING_ONLY", "town": "Town13",
        "route_sha256": owner.ROUTE_SHA256,
        "research_split_intent": "town13_render_sensor_diagnostic_only",
        "frame_count": 240, "motion_mode": "basic-agent",
        "controller": "CARLA BasicAgent", "physics_driving": True,
        "ignores_traffic_lights": True, "learned_model_control": False,
        "hd_map_approved": False, "oracle_pose": True, "oracle_depth": True,
        "oracle_semantics": True, "collision_events": 0,
        "live_replay_exact_match": True, "cleanup_errors": [], "travel_m": 20.0,
    }
    (mapping / "summary.json").write_text(json.dumps(summary))
    (mapping / "map.json").write_text("{}")
    records = []
    for index in range(owner.FRAMES):
        stem = f"frames/{index:04d}"
        records.append({"stem": stem, "frame": index + 101, "timestamp_s": index * 0.1})
        (frames / f"{index:04d}.png").touch()
        (frames / f"{index:04d}.npz").touch()
    (mapping / "records.json").write_text(json.dumps(records))
    decoded = []
    monkeypatch.setattr(owner, "_validate_frame_files", lambda root, stem: decoded.append(stem))
    monkeypatch.setattr(owner, "_verify_sources_and_route", lambda *a: True)
    result = owner._assess_capture({"source_sha256": {}})
    assert result["status"] == "CAPTURE_DIAGNOSTIC_ONLY"
    assert result["map_dev_admitted"] is False
    assert result["training_data_admitted"] is False
    assert len(decoded) == 240
    (frames / "0239.npz").unlink()
    with pytest.raises(owner.ScoutOwnerError, match="count or naming"):
        owner._assess_capture({"source_sha256": {}})


@pytest.mark.parametrize("worker_status", [0, 1])
def test_execute_records_owned_cleanup_and_never_admits_dev(monkeypatch, tmp_path, worker_status):
    profile, entry, assets, env, binding = _binding()
    monkeypatch.setattr(owner, "OUTPUT", tmp_path / "scout")
    monkeypatch.setattr(owner, "preflight", lambda: (profile, entry, assets, env, binding))
    monkeypatch.setattr(owner, "_workspace_lock", _owned_lock)
    monkeypatch.setattr(owner.catalog_owner, "_assert_no_other_carla", lambda: None)
    monkeypatch.setattr(owner.catalog_owner, "_assert_ports_free", lambda: None)
    monkeypatch.setattr(owner.catalog_owner, "_verify_source_assets", lambda *a: True)
    monkeypatch.setattr(owner, "_archive_sources_and_route", lambda *a: {"route": owner.ROUTE_SHA256})
    monkeypatch.setattr(owner, "_verify_sources_and_route", lambda *a: True)
    monkeypatch.setattr(owner, "_wait_ready", lambda *a: {"status": "PASS"})
    stages = []
    monkeypatch.setattr(owner, "_probe", lambda stage, *a, **kw: stages.append(stage) or {"status": "PASS"})
    monkeypatch.setattr(owner, "_assess_capture", lambda *a: {"status": "CAPTURE_DIAGNOSTIC_ONLY"})
    stopped = []
    monkeypatch.setattr(owner.catalog_owner, "_stop_owned_group", lambda child: stopped.append(child.pid) or True)

    class Child:
        def __init__(self, pid, status):
            self.pid = pid
            self.returncode = status

        def wait(self, timeout):
            return self.returncode

        def poll(self):
            return None

    children = [Child(50001, 0), Child(50002, worker_status)]
    monkeypatch.setattr(subprocess, "Popen", lambda *a, **kw: children.pop(0))
    result = owner.execute(profile, entry, assets, env, binding)
    final = json.loads((owner.OUTPUT / "owner_result.json").read_text())
    assert result == worker_status
    assert final["status"] == ("PASS" if worker_status == 0 else "FAIL")
    assert final["map_dev_admitted"] is False
    assert final["training_data_admitted"] is False
    assert final["owned_processes_stopped_and_ports_released"] is True
    assert stopped == [50002, 50001]
    assert stages == (["after_capture", "stopped"] if worker_status == 0 else ["stopped"])
