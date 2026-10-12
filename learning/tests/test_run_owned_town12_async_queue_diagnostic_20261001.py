"""HH_261001 - Verify the separate single-factor async queue startup diagnostic."""

from contextlib import contextmanager
import json
import subprocess

import pytest

from scripts.e2e import run_owned_town12_async_queue_diagnostic_20261001 as variant


diagnostic = variant.diagnostic
owner = diagnostic.owner


def binding():
    return ({"executable": "/pinned/UE4Editor", "project": "/pinned/CarlaUE4.uproject"},
            {"load_name": owner.MAP_LOAD_NAME}, {"level_path": {"ok": True}}, {})


@contextmanager
def owned_lock():
    yield


def test_async_queue_variant_changes_one_factor_and_restores_globals(monkeypatch, tmp_path, capsys):
    original_output, original_cvar = diagnostic.OUTPUT, diagnostic.CVAR
    original_command = owner.editor_command
    monkeypatch.setattr(variant, "OUTPUT", tmp_path / "new")
    monkeypatch.setattr(diagnostic, "source_paths", lambda: {})
    monkeypatch.setattr(owner, "preflight", binding)
    monkeypatch.setattr(subprocess, "Popen", lambda *a, **kw: pytest.fail("simulator launched"))
    assert variant.main([]) == 0
    report = json.loads(capsys.readouterr().out)
    command = report["source_editor_argv"]
    assert command[:-2] == original_command(*binding()[:2])
    assert command[-2:] == [variant.OVERRIDE, variant.QUERY]
    assert not any("r.GenerateMeshDistanceFields" in argument for argument in command)
    assert report["simulator_started"] is False
    assert report["capture_enabled"] is False
    assert not variant.OUTPUT.exists()
    assert diagnostic.CVAR == original_cvar
    assert diagnostic.OUTPUT == original_output
    assert owner.editor_command is original_command


@pytest.mark.parametrize("evidence", ["correct", "wrong_cvar", "wrong_value", "absent"])
def test_async_queue_full_lifecycle_preserves_evidence_gate_and_cleanup(monkeypatch, tmp_path, evidence):
    output = tmp_path / "attempt"
    source = tmp_path / "input.py"
    source.write_text("frozen", encoding="utf-8")
    monkeypatch.setattr(variant, "OUTPUT", output)
    monkeypatch.setattr(diagnostic, "source_paths", lambda: {"input.py": source})
    monkeypatch.setattr(owner, "preflight", binding)
    monkeypatch.setattr(owner.shared_owner, "_workspace_lock", owned_lock)
    monkeypatch.setattr(owner.shared_owner, "_assert_no_other_carla", lambda: None)
    monkeypatch.setattr(owner, "_assert_port_triplet_free", lambda: None)
    monkeypatch.setattr(owner.shared_owner, "_verify_source_assets", lambda *a: True)
    monkeypatch.setattr(owner, "_wait_ready", lambda *a: None)
    monkeypatch.setattr(owner, "_probe", lambda *a, **kw: {"status": "PASS"})
    monkeypatch.setattr(owner, "_read_live_xodr", lambda *a: {"sha256": owner.XODR_SHA256})
    cleaned = []
    monkeypatch.setattr(owner.shared_owner, "_stop_owned_group", lambda proc: cleaned.append(proc.pid) or True)

    class FakeEditor:
        pid = 123456

        def __init__(self, command, **kwargs):
            assert command[-2:] == [variant.OVERRIDE, variant.QUERY]
            assert kwargs["start_new_session"] is True
            variable = "r.GenerateMeshDistanceFields" if evidence == "wrong_cvar" else variant.CVAR
            value = "1" if evidence == "wrong_value" else "0"
            text = (f"LogConfig: Setting CVar [[{variant.CVAR}:0]]\n"
                    f'LogConsoleResponse: {variable} = "{value}" LastSetBy: ProjectSetting\n')
            kwargs["stdout"].write(("" if evidence == "absent" else text).encode("utf-8"))
            kwargs["stdout"].flush()

    monkeypatch.setattr(subprocess, "Popen", FakeEditor)
    assert variant.main(["--execute"]) == (0 if evidence == "correct" else 1)
    result = json.loads((output / "owner_result.json").read_text())
    assert result["status"] == ("PASS" if evidence == "correct" else "FAIL")
    assert result["owned_processes_stopped_and_ports_released"] is True
    assert result["source_bytes_unchanged_and_archived"] is True
    assert result["capture_performed"] is False
    assert result["training_data_admitted"] is False
    assert result["renderer_diagnostic"]["changed_renderer_cvar"] == variant.CVAR
    assert cleaned == [123456]


def test_async_variant_freezes_both_layers_and_restores_on_error():
    original_cvar, original_sources = diagnostic.CVAR, diagnostic.SOURCE_FILES
    with pytest.raises(RuntimeError):
        with variant.configured_diagnostic():
            paths = diagnostic.source_paths()
            assert set(original_sources) <= set(paths)
            assert "tools/e2e/run_owned_town12_async_queue_diagnostic_20261001.py" in paths
            assert "learning/tests/test_run_owned_town12_async_queue_diagnostic_20261001.py" in paths
            assert any(name.endswith("DistanceFieldAtlas.cpp") for name in paths)
            raise RuntimeError("test")
    assert diagnostic.CVAR == original_cvar
    assert diagnostic.SOURCE_FILES is original_sources


def test_scope_explicitly_limits_async_queue_hypothesis():
    contract = variant.diagnostic_contract()
    assert "Worker-thread requests still use the async queue" in contract["limitation"]
    assert "warmed DDC" in contract["limitation"]
    assert contract["capture_authorized"] is False
    assert variant.OUTPUT != diagnostic.OUTPUT
