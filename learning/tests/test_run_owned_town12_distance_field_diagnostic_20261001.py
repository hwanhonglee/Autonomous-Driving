"""HH_261001 - Test numeric CVar evidence and inherited gates without a simulator."""

from contextlib import contextmanager
import json
from pathlib import Path
import socket
import subprocess

import pytest

from scripts.e2e import run_owned_town12_distance_field_diagnostic_20261001 as diagnostic


owner = diagnostic.owner


def binding():
    return ({"executable": "/pinned/UE4Editor", "project": "/pinned/CarlaUE4.uproject"},
            {"load_name": owner.MAP_LOAD_NAME}, {"level_path": {"ok": True}}, {})


def good_log():
    return (f"LogConfig: Setting CVar [[{diagnostic.CVAR}:0]]\n"
            f'LogConsoleResponse: {diagnostic.CVAR} = "0"      LastSetBy: ProjectSetting\n')


@contextmanager
def owned_lock():
    yield


def test_default_invocation_is_read_only_and_restores_owner(monkeypatch, tmp_path, capsys):
    old_output, old_command = owner.OUTPUT, owner.editor_command
    monkeypatch.setattr(diagnostic, "OUTPUT", tmp_path / "new")
    monkeypatch.setattr(diagnostic, "source_paths", lambda: {})
    monkeypatch.setattr(owner, "preflight", binding)
    monkeypatch.setattr(subprocess, "Popen", lambda *a, **kw: pytest.fail("simulator launched"))
    assert diagnostic.main([]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["simulator_started"] is False
    assert report["source_editor_argv"][-2:] == [diagnostic.OVERRIDE, diagnostic.QUERY]
    assert report["source_editor_argv"][:-2] == old_command(*binding()[:2])
    assert not diagnostic.OUTPUT.exists()
    assert owner.OUTPUT == old_output
    assert owner.editor_command is old_command


@pytest.mark.parametrize("text,status", [
    ("", "INCONCLUSIVE"),
    ('LogConfig: Setting CVar [[r.GenerateMeshDistanceFields:0]]\n', "INCONCLUSIVE"),
    (good_log(), "PASS"),
    (good_log().replace('= "0"', '= "1"'), "FAIL"),
    (good_log().replace('= "0"', '= "False # HH_250321"'), "FAIL"),
    (good_log().replace("ProjectSetting", "Console"), "FAIL"),
    (good_log().splitlines()[1], "FAIL"),
    (good_log() + good_log().replace('= "0"', '= "1"'), "FAIL"),
])
def test_effective_value_requires_query_evidence(tmp_path, text, status):
    log = tmp_path / "server.log"
    log.write_text(text, encoding="utf-8")
    assert diagnostic.renderer_receipt(log)["status"] == status


def test_missing_renderer_evidence_rejects_rpc_ready(monkeypatch, tmp_path):
    monkeypatch.setattr(diagnostic, "OUTPUT", tmp_path)
    monkeypatch.setattr(owner, "_wait_ready", lambda *a: None)
    with diagnostic.configured_owner():
        with pytest.raises(owner.Town12OwnerError, match="absent"):
            owner._wait_ready(None, {})
    assert json.loads((tmp_path / "effective_renderer.json").read_text())["status"] == "INCONCLUSIVE"


@pytest.mark.parametrize("gate", ["other_process", "port", "existing_output", "binding_changed"])
def test_inherited_launch_gates_remain_closed(monkeypatch, tmp_path, gate):
    output = tmp_path / "attempt"
    monkeypatch.setattr(diagnostic, "OUTPUT", output)
    monkeypatch.setattr(diagnostic, "source_paths", lambda: {})
    monkeypatch.setattr(owner, "preflight", binding)
    monkeypatch.setattr(owner.shared_owner, "_workspace_lock", owned_lock)
    monkeypatch.setattr(owner.shared_owner, "_assert_no_other_carla", lambda: None)
    monkeypatch.setattr(subprocess, "Popen", lambda *a, **kw: pytest.fail("simulator launched"))
    if gate == "other_process":
        def blocked():
            raise owner.shared_owner.CatalogOwnerError("other simulator")
        monkeypatch.setattr(owner.shared_owner, "_assert_no_other_carla", blocked)
    if gate == "existing_output":
        output.mkdir()
    provided = binding()
    if gate == "binding_changed":
        provided = (*provided[:3], {"changed": "environment"})
    with socket.socket() as occupied:
        occupied.bind(("127.0.0.1", 0))
        if gate == "port":
            monkeypatch.setattr(owner.shared_owner, "PORTS", (occupied.getsockname()[1],))
        else:
            monkeypatch.setattr(owner, "_assert_port_triplet_free", lambda: None)
        with diagnostic.configured_owner():
            with pytest.raises((owner.Town12OwnerError, owner.shared_owner.CatalogOwnerError)):
                owner.execute(*provided)
    assert output.exists() is (gate == "existing_output")


def test_all_sources_are_archived_and_changes_rejected(monkeypatch, tmp_path):
    source = tmp_path / "source.py"
    source.write_text("initial", encoding="utf-8")
    monkeypatch.setattr(diagnostic, "OUTPUT", tmp_path / "attempt")
    monkeypatch.setattr(diagnostic, "source_paths", lambda: {"source.py": source})
    hashes = diagnostic.archive_sources()
    assert diagnostic.verify_sources(hashes)
    source.write_text("changed", encoding="utf-8")
    assert not diagnostic.verify_sources(hashes)
    source.write_text("initial", encoding="utf-8")
    archived = diagnostic.OUTPUT / "provenance/source.py"
    archived.write_text("changed", encoding="utf-8")
    assert not diagnostic.verify_sources(hashes)
    assert not diagnostic.verify_sources({})


def test_provenance_includes_transitive_owner_sources_and_engine_evidence():
    paths = diagnostic.source_paths()
    assert set(owner.SOURCE_FILES) <= set(paths)
    assert set(owner.shared_owner.SOURCE_FILES) <= set(paths)
    assert "tools/e2e/run_owned_town12_distance_field_diagnostic_20261001.py" in paths
    assert "external/carla/Config/DefaultEngine.ini" in paths
    assert any(name.endswith("DistanceFieldAtlas.cpp") for name in paths)


def test_failed_startup_still_writes_effective_diagnostic(monkeypatch, tmp_path):
    monkeypatch.setattr(diagnostic, "OUTPUT", tmp_path)
    (tmp_path / "server.log").write_text(good_log(), encoding="utf-8")
    with diagnostic.configured_owner():
        owner._write_new_json(tmp_path / "owner_result.json", {
            "status": "FAIL", "error": "UE4Editor exited during startup: -11",
        })
    result = json.loads((tmp_path / "owner_result.json").read_text())
    assert result["status"] == "FAIL"
    assert result["renderer_diagnostic"]["status"] == "PASS"
    assert (tmp_path / "renderer_diagnostic.json").is_file()


def test_full_map_and_exact_xodr_gates_are_inherited():
    original_probe, original_read = owner._probe, owner._read_live_xodr
    with diagnostic.configured_owner():
        assert owner._probe is original_probe
        assert owner._read_live_xodr is original_read
        assert owner.LIVE_MAP_NAMES == frozenset((owner.MAP_LOAD_NAME, "Carla/Maps/Town12/Town12"))


def test_owner_configuration_restores_after_failure():
    old_output, old_archive = owner.OUTPUT, owner._archive_sources
    with pytest.raises(RuntimeError):
        with diagnostic.configured_owner():
            raise RuntimeError("test")
    assert owner.OUTPUT == old_output
    assert owner._archive_sources is old_archive


@pytest.mark.parametrize("startup", ["pass", "missing_query", "wrong_value", "crash"])
def test_owned_lifecycle_requires_renderer_evidence_and_always_cleans_up(monkeypatch, tmp_path, startup):
    output = tmp_path / "attempt"
    source = tmp_path / "input.py"
    source.write_text("frozen", encoding="utf-8")
    monkeypatch.setattr(diagnostic, "OUTPUT", output)
    monkeypatch.setattr(diagnostic, "source_paths", lambda: {"input.py": source})
    monkeypatch.setattr(owner, "preflight", binding)
    monkeypatch.setattr(owner.shared_owner, "_workspace_lock", owned_lock)
    monkeypatch.setattr(owner.shared_owner, "_assert_no_other_carla", lambda: None)
    monkeypatch.setattr(owner, "_assert_port_triplet_free", lambda: None)
    monkeypatch.setattr(owner.shared_owner, "_verify_source_assets", lambda *a: True)
    cleaned = []
    monkeypatch.setattr(owner.shared_owner, "_stop_owned_group", lambda proc: cleaned.append(proc.pid) or True)
    monkeypatch.setattr(owner, "_probe", lambda *a, **kw: {"status": "PASS"})
    monkeypatch.setattr(owner, "_read_live_xodr", lambda *a: {"sha256": owner.XODR_SHA256})

    class FakeEditor:
        pid = 123456

        def __init__(self, command, **kwargs):
            assert command[-2:] == [diagnostic.OVERRIDE, diagnostic.QUERY]
            assert kwargs["start_new_session"] is True
            text = good_log()
            if startup == "missing_query":
                text = text.splitlines()[0]
            elif startup == "wrong_value":
                text = text.replace('= "0"', '= "1"')
            kwargs["stdout"].write(text.encode("utf-8"))
            kwargs["stdout"].flush()

    def ready(*args):
        if startup == "crash":
            raise owner.Town12OwnerError("UE4Editor exited during startup: -11")

    monkeypatch.setattr(owner, "_wait_ready", ready)
    monkeypatch.setattr(subprocess, "Popen", FakeEditor)
    assert diagnostic.main(["--execute"]) == (0 if startup == "pass" else 1)
    result = json.loads((output / "owner_result.json").read_text())
    plan = json.loads((output / "owner_plan.json").read_text())
    assert result["status"] == ("PASS" if startup == "pass" else "FAIL")
    assert result["owned_processes_stopped_and_ports_released"] is True
    assert result["source_bytes_unchanged_and_archived"] is True
    assert result["capture_performed"] is False
    assert result["training_data_admitted"] is False
    assert plan["renderer_diagnostic"]["changed_renderer_cvar"] == diagnostic.CVAR
    assert cleaned == [123456]
