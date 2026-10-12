"""HH_260929 - Verify Town12 ownership gates without starting UE4Editor."""

from contextlib import contextmanager
import json
from pathlib import Path
import socket
import subprocess

import pytest

from scripts.e2e import run_owned_town12_source_editor_probe as owner


def _binding():
    profile = {
        "kind": "source_editor", "executable": "/home/a/UnrealEngine_4.26/Engine/Binaries/Linux/UE4Editor",
        "project": "/home/a/carla/Unreal/CarlaUE4/CarlaUE4.uproject",
        "python_api": "/home/a/carla/PythonAPI/carla/dist/source.egg",
        "python_api_sha256": owner.EXPECTED_SHA256["python_api"],
    }
    entry = {
        **owner.EXPECTED_MAP,
        "level_path": "/home/a/carla/Unreal/CarlaUE4/Content/Carla/Maps/Town12/Town12.umap",
        "opendrive_path": "/home/a/carla/Unreal/CarlaUE4/Content/Carla/Maps/Town12/OpenDrive/Town12.xodr",
    }
    assets = {"level_path": {"path": entry["level_path"], "ok": True}}
    return profile, entry, assets, {}


@contextmanager
def _owned_lock():
    yield


def test_default_invocation_is_read_only(monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(owner, "preflight", _binding)
    monkeypatch.setattr(owner, "OUTPUT", tmp_path / "absent")
    monkeypatch.setattr(subprocess, "Popen", lambda *a, **kw: pytest.fail("launched UE4Editor"))
    assert owner.main([]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["status"] == "STATIC_READY_NOT_LIVE_TESTED"
    assert report["simulator_started"] is False
    assert report["capture_enabled"] is False
    assert report["source_editor_argv"][2] == owner.MAP_LOAD_NAME
    assert not (tmp_path / "absent").exists()


@pytest.mark.parametrize("failure", ["map", "profile", "hash"])
def test_changed_source_binding_fails_closed(monkeypatch, failure):
    profile, entry, _, _ = _binding()
    if failure == "map":
        entry["load_name"] = "/Game/Carla/Maps/Town13/Town13"
    if failure == "profile":
        profile["kind"] = "packaged"
    manifest = {"maps": [entry], "server_profiles": {owner.PROFILE_ID: profile}}
    monkeypatch.setattr(owner, "load_manifest", lambda: (manifest, Path("/manifest")))
    monkeypatch.setattr(owner, "inspect_file", lambda *a: {"ok": failure != "hash"})
    assert not owner.DEFAULT_MANIFEST.is_symlink()
    with pytest.raises(owner.Town12OwnerError):
        owner.static_binding()


def test_editor_command_uses_exact_town12_level():
    profile, entry, _, _ = _binding()
    command = owner.editor_command(profile, entry)
    assert command[:3] == [profile["executable"], profile["project"], owner.MAP_LOAD_NAME]
    assert "-carla-port=2100" in command
    assert "-quality-level=Epic" in command
    assert "-RenderOffScreen" in command
    assert any("bSendUnattendedBugReports=False" in item for item in command)


@pytest.mark.parametrize("blocker", ["occupied_port", "existing_output"])
def test_execute_blockers_refuse_before_launch(monkeypatch, tmp_path, blocker):
    binding = _binding()
    output = tmp_path / "attempt"
    monkeypatch.setattr(owner, "OUTPUT", output)
    monkeypatch.setattr(owner, "preflight", lambda: binding)
    monkeypatch.setattr(owner.shared_owner, "_workspace_lock", _owned_lock)
    monkeypatch.setattr(owner.shared_owner, "_assert_no_other_carla", lambda: None)
    monkeypatch.setattr(subprocess, "Popen", lambda *a, **kw: pytest.fail("launched UE4Editor"))
    if blocker == "occupied_port":
        with socket.socket() as occupied:
            occupied.bind(("127.0.0.1", 0))
            monkeypatch.setattr(owner.shared_owner, "PORTS", (occupied.getsockname()[1],))
            with pytest.raises(owner.Town12OwnerError, match="occupied"):
                owner.execute(*binding)
        assert not output.exists()
    else:
        output.mkdir()
        monkeypatch.setattr(owner, "_assert_port_triplet_free", lambda: None)
        with pytest.raises(owner.Town12OwnerError, match="existing output"):
            owner.execute(*binding)


def test_live_xodr_hash_must_match_before_success(monkeypatch):
    def fake_run(*args, **kwargs):
        return subprocess.CompletedProcess(args[0], 0, json.dumps({
            "map_name": owner.MAP_LOAD_NAME, "sha256": "0" * 64,
            "size_bytes": 71350074, "read_only": True,
        }), "")

    monkeypatch.setattr(subprocess, "run", fake_run)
    with pytest.raises(owner.Town12OwnerError, match="differs"):
        owner._read_live_xodr({})


def test_same_basename_from_another_map_is_rejected(monkeypatch, tmp_path):
    monkeypatch.setattr(owner, "OUTPUT", tmp_path)
    (tmp_path / "lifecycle").mkdir()
    (tmp_path / "lifecycle" / "ready.json").write_text(json.dumps({
        "status": "PASS", "error": None,
        "active_map_name": "/Game/Other/Town12",
    }), encoding="utf-8")
    monkeypatch.setattr(
        subprocess, "run",
        lambda *a, **kw: subprocess.CompletedProcess(a[0], 0, "", ""),
    )
    with pytest.raises(owner.Town12OwnerError, match="full map name"):
        owner._probe("ready", 43210, {})
