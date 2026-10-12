"""HH_260906 - Keep Town13 route catalog ownership narrow before its first live launch."""

from contextlib import contextmanager
import json
import os
from pathlib import Path
import signal
import socket
import subprocess

import pytest

from scripts.e2e import run_owned_town13_route_catalog as owner


def _binding():
    profile = {
        "kind": "source_editor",
        "executable": "/exact/UE4Editor",
        "project": "/exact/CarlaUE4.uproject",
        "python_api": "/exact/source.egg",
    }
    entry = {**owner.EXPECTED_MAP}
    assets = {"project": {"path": "/exact/CarlaUE4.uproject", "ok": True}}
    return profile, entry, assets, {}


def _catalog():
    return {
        "status": "complete", "map_id": "town13",
        "load_name": owner.MAP_LOAD_NAME, "server_profile": owner.PROFILE_ID,
        "server": {
            "active_server_profile": owner.PROFILE_ID,
            "initial_map_name": owner.MAP_LOAD_NAME,
            "active_map_name": owner.MAP_LOAD_NAME,
            "map_load_allowed": False, "map_load_performed": False,
            "client_version": "0.9.15", "server_version": "0.9.15",
        },
        "generation": {
            "scenarios": ["lane_follow", "straight", "left", "right"],
            "seeds": [1, 2], "pairs_per_seed": 4,
            "minimum_distance_m": 60.0, "maximum_distance_m": 140.0,
            "preferred_distance_m": 100.0, "sampling_resolution_m": 1.0,
            "max_traces_per_scenario": 12000, "spawn_point_count": 5,
        },
        "routes": [{
            "status": "ready", "scenario": "right",
            "start_spawn_index": 1, "goal_spawn_index": 2,
            "path": "routes/town13/right/route.json", "sha256": "a" * 64,
        }],
    }


def test_default_invocation_is_read_only_without_popen(monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(owner, "preflight", _binding)
    monkeypatch.setattr(owner, "OUTPUT", tmp_path / "absent")
    monkeypatch.setattr(subprocess, "Popen", lambda *a, **kw: pytest.fail("launched a process"))
    assert owner.main([]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result["status"] == "STATIC_READY_NOT_LIVE_TESTED"
    assert result["simulator_started"] is False
    assert result["output_absent"] is True
    # HH_260928 - Static preflight must expose the process-local crash upload override.
    assert result["source_editor_argv"][-1] == (
        "-ini:EditorSettings:[/Script/UnrealEd.CrashReportsPrivacySettings]:"
        "bSendUnattendedBugReports=False"
    )
    assert not (tmp_path / "absent").exists()


@pytest.mark.parametrize("change", ["profile", "hash"])
def test_wrong_profile_or_hash_refuses_before_launch(monkeypatch, change):
    profile, entry, _, _ = _binding()
    if change == "profile":
        profile["kind"] = "packaged"
    manifest = {"maps": [entry], "server_profiles": {owner.PROFILE_ID: profile}}
    monkeypatch.setattr(owner, "load_manifest", lambda: (manifest, Path("/exact/manifest")))
    monkeypatch.setattr(owner, "inspect_file", lambda *a: {"ok": change != "hash"})
    monkeypatch.setattr(subprocess, "Popen", lambda *a, **kw: pytest.fail("launched a process"))
    assert owner.main(["--execute"]) == 2


@contextmanager
def _owned_lock():
    yield


@pytest.mark.parametrize("blocker", ["occupied_port", "existing_output"])
def test_runtime_blockers_refuse_before_launch(monkeypatch, tmp_path, blocker):
    profile, entry, assets, env = _binding()
    output = tmp_path / "catalog"
    monkeypatch.setattr(owner, "OUTPUT", output)
    monkeypatch.setattr(owner, "_workspace_lock", _owned_lock)
    monkeypatch.setattr(owner, "static_binding", lambda: (profile, entry, assets))
    monkeypatch.setattr(owner, "_assert_no_other_carla", lambda: None)
    monkeypatch.setattr(subprocess, "Popen", lambda *a, **kw: pytest.fail("launched a process"))
    if blocker == "occupied_port":
        with socket.socket() as occupied:
            occupied.bind(("127.0.0.1", 0))
            monkeypatch.setattr(owner, "PORTS", (occupied.getsockname()[1],))
            with pytest.raises(owner.CatalogOwnerError, match="occupied"):
                owner.execute(profile, entry, assets, env)
    else:
        output.mkdir()
        monkeypatch.setattr(owner, "_assert_ports_free", lambda: None)
        with pytest.raises(owner.CatalogOwnerError, match="existing output"):
            owner.execute(profile, entry, assets, env)
    assert blocker == "existing_output" or not output.exists()


def test_editor_and_catalog_commands_are_exact_and_no_world_switch():
    profile, entry, _, _ = _binding()
    command = owner.editor_command(profile, entry)
    # HH_260928 - Preserve all source-editor flags and add one process-local override.
    assert command == [
        profile["executable"], profile["project"], owner.MAP_LOAD_NAME,
        "-game", "-vulkan", "-prefernvidia", "-quality-level=Epic",
        "-carla-port=2100", "-RenderOffScreen", "-nosound", "-unattended",
        "-ini:EditorSettings:[/Script/UnrealEd.CrashReportsPrivacySettings]:"
        "bSendUnattendedBugReports=False",
    ]
    args = owner.catalog_arguments()
    owner._assert_catalog_arguments(args)
    assert "--allow-map-load" not in args
    assert args[args.index("--active-server-profile") + 1] == owner.PROFILE_ID
    changed = {**entry, "load_name": "/Game/Carla/Maps/Town12/Town12"}
    with pytest.raises(owner.CatalogOwnerError, match="not Town13"):
        owner.editor_command(profile, changed)


@pytest.mark.parametrize("field,value", [
    ("load_name", "/Game/Other/Town13"),
    ("server.active_map_name", "/Game/Other/Town13"),
    ("server.active_server_profile", "packaged_0915"),
    ("server.map_load_performed", True),
    ("generation.seeds", [3, 4]),
    ("routes", []),
])
def test_catalog_rejects_wrong_full_map_profile_world_load_and_search(field, value):
    catalog = _catalog()
    assert owner._validate_catalog_document(catalog) == 1
    target = catalog
    keys = field.split(".")
    for key in keys[:-1]:
        target = target[key]
    target[keys[-1]] = value
    with pytest.raises(owner.CatalogOwnerError, match="identity or route completeness"):
        owner._validate_catalog_document(catalog)


def test_catalog_accepts_only_observed_ue4_full_asset_path_variant():
    catalog = _catalog()
    catalog["server"]["initial_map_name"] = "Carla/Maps/Town13/Town13"
    catalog["server"]["active_map_name"] = "Carla/Maps/Town13/Town13"
    assert owner._validate_catalog_document(catalog) == 1
    catalog["server"]["active_map_name"] = "Other/Maps/Town13/Town13"
    with pytest.raises(owner.CatalogOwnerError, match="identity or route completeness"):
        owner._validate_catalog_document(catalog)


def test_catalog_route_requires_pinned_record_and_contained_file(monkeypatch, tmp_path):
    catalog = _catalog()
    assert owner._validate_catalog_document(catalog) == 1
    bad = _catalog()
    bad["routes"][0]["sha256"] = "not-a-hash"
    with pytest.raises(owner.CatalogOwnerError, match="invalid route record"):
        owner._validate_catalog_document(bad)
    monkeypatch.setattr(owner, "OUTPUT", tmp_path)
    (tmp_path / "route_catalog.json").write_text(json.dumps(catalog), encoding="utf-8")
    catalog["routes"][0]["path"] = "../outside.json"
    (tmp_path / "route_catalog.json").write_text(json.dumps(catalog), encoding="utf-8")
    with pytest.raises(owner.CatalogOwnerError, match="escapes its catalog"):
        owner._assess_catalog({"source_sha256": {"source": "a" * 64}})


def test_running_receipt_rejects_same_basename_on_other_map(monkeypatch, tmp_path):
    monkeypatch.setattr(owner, "OUTPUT", tmp_path)
    lifecycle = tmp_path / "lifecycle"
    lifecycle.mkdir()
    receipt = {
        "status": "PASS", "error": None,
        "active_map_name": "/Game/Other/Town13",
    }
    (lifecycle / "ready.json").write_text(json.dumps(receipt), encoding="utf-8")
    monkeypatch.setattr(
        subprocess, "run",
        lambda *a, **kw: subprocess.CompletedProcess(a[0], 0, "", ""),
    )
    with pytest.raises(owner.CatalogOwnerError, match="full map name"):
        owner._probe("ready", 43210, {})


def test_running_receipt_accepts_observed_ue4_full_asset_path(monkeypatch, tmp_path):
    monkeypatch.setattr(owner, "OUTPUT", tmp_path)
    lifecycle = tmp_path / "lifecycle"
    lifecycle.mkdir()
    receipt = {
        "status": "PASS", "error": None,
        "active_map_name": "Carla/Maps/Town13/Town13",
    }
    (lifecycle / "ready.json").write_text(json.dumps(receipt), encoding="utf-8")
    monkeypatch.setattr(
        subprocess, "run",
        lambda *a, **kw: subprocess.CompletedProcess(a[0], 0, "", ""),
    )
    assert owner._probe("ready", 43210, {}) == receipt


def test_cleanup_signals_only_its_owned_positive_process_group(monkeypatch):
    class Child:
        pid = 43210

        def poll(self):
            return None

        def wait(self, timeout):
            return 0

    signals = []
    monkeypatch.setattr(owner.os, "getpgrp", lambda: 99)
    monkeypatch.setattr(owner.os, "getpgid", lambda pid: pid)

    def killpg(pgid, sig):
        if sig == 0:
            raise ProcessLookupError
        signals.append((pgid, sig))

    monkeypatch.setattr(owner.os, "killpg", killpg)
    assert owner._stop_owned_group(Child()) is True
    assert signals == [(43210, signal.SIGINT)]
    assert owner._stop_owned_group(None) is True
    assert signals == [(43210, signal.SIGINT)]
    Child.pid = 0
    with pytest.raises(owner.CatalogOwnerError, match="non-owned"):
        owner._stop_owned_group(Child())
    assert signals == [(43210, signal.SIGINT)]
