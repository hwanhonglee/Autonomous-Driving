"""HH_261004 - Check launch boundaries synthetically without data, GPU or subprocesses."""
import hashlib
import json
import pytest
from scripts.e2e import launch_temporal_retention_pair_pilot_20261004 as launch


def test_duplicate_json_is_denied():
    with pytest.raises(ValueError, match="duplicate"):
        json.loads('{"a":1,"a":2}', object_pairs_hook=launch.unique)


def test_holdout_command_is_denied():
    with pytest.raises(ValueError, match="TRAIN-only"):
        launch.command({"asset_arguments": {"recovery-holdout": "irrelevant"}}, "fit", "0" * 64)


def test_relative_path_is_denied():
    from pathlib import Path
    with pytest.raises(ValueError, match="private"):
        launch.canonical(Path("relative"))


def test_pin_parses_same_buffer_and_rejects_mutation(tmp_path, monkeypatch):
    monkeypatch.setattr(launch, "PERSONAL", tmp_path)
    path = tmp_path / "receipt.json"
    raw = b'{"result":true}'
    path.write_bytes(raw)
    sha = hashlib.sha256(raw).hexdigest()
    assert launch.pinned(path, sha) == {"result": True}
    path.write_bytes(b'{"result":false}')
    with pytest.raises(ValueError, match="SHA"):
        launch.pinned(path, sha)


def test_symlink_is_denied(tmp_path, monkeypatch):
    monkeypatch.setattr(launch, "PERSONAL", tmp_path)
    real = tmp_path / "real.json"; real.write_text("{}")
    link = tmp_path / "alias.json"; link.symlink_to(real)
    with pytest.raises(ValueError, match="private"):
        launch.digest(link)
