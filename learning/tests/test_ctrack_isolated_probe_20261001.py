"""HH_261001 - Verify mount boundaries, ownership selection, and stationary output."""

from pathlib import Path
import struct
import zlib

import pytest

from scripts.e2e import prepare_ctrack_isolated_probe_20261001 as stage
from scripts.e2e import run_owned_ctrack_isolated_probe_20261001 as owner


def test_sandbox_shared_source_is_readonly_and_caches_are_private(tmp_path):
    argv = stage.sandbox_argv(tmp_path)
    assert argv[argv.index("--ro-bind") + 1:argv.index("--ro-bind") + 3] == ["/", "/"]
    bindings = dict(stage.private_bindings(tmp_path))
    assert bindings[tmp_path / "Config"] == stage.PROJECT / "Config"
    assert bindings[tmp_path / "UserConfig"] == Path("/home/a/.config")
    assert bindings[tmp_path / "UserCache"] == Path("/home/a/.cache")
    assert stage.ENGINE / "DerivedDataCache" not in bindings.values()
    assert "--unshare-net" not in argv
    assert argv[argv.index("--dev-bind") + 1:argv.index("--dev-bind") + 3] == ["/dev", "/dev"]


def test_private_startup_map_and_ddc_do_not_mutate_original_text(tmp_path):
    original = "[/Script/EngineSettings.GameMapsSettings]\nGameDefaultMap=Town10HD_Opt\n"
    config = stage.private_engine_config(original, tmp_path)
    assert original == "[/Script/EngineSettings.GameMapsSettings]\nGameDefaultMap=Town10HD_Opt\n"
    assert config.startswith(original)
    assert config.rsplit("GameDefaultMap=", 1)[1].splitlines()[0] == stage.MAP + ".C_track_1_0_7"
    assert f'Shared=(Type=FileSystem, ReadOnly=true' in config
    assert f'Path="{stage.ENGINE / "DerivedDataCache"}"' in config
    with pytest.raises(ValueError):
        stage.editor_argv(tmp_path, 65534)


def test_owner_descendants_exclude_unrelated_matching_process():
    records = [{"pid": 10, "ppid": 1}, {"pid": 11, "ppid": 10},
               {"pid": 12, "ppid": 11}, {"pid": 21, "ppid": 20}]
    assert {r["pid"] for r in owner.descendants(10, records)} == {11, 12}


def test_editor_must_be_exact_owned_descendant_and_same_pgid(monkeypatch):
    expected = [str(stage.EDITOR), str(stage.PROJECT / "CarlaUE4.uproject"), stage.MAP]
    record = {"pid": 12, "ppid": 11, "pgid": 10, "argv": expected, "state": "S"}
    records = [{"pid": 11, "ppid": 10, "pgid": 10, "argv": ["bwrap"], "state": "S"},
               record, {**record, "pid": 90, "ppid": 1, "pgid": 90}]
    monkeypatch.setattr(owner, "all_processes", lambda: records)
    assert owner.find_editor(10, expected) == record
    record["pgid"] = 12
    with pytest.raises(RuntimeError, match="process group"):
        owner.find_editor(10, expected)


def test_stationary_camera_and_png_preserve_raw_values(tmp_path):
    pose = owner.camera_pose({"start_carla_transform": {"x": 1, "y": 2, "z": 3, "yaw": 90}})
    assert pose["x"] == pytest.approx(1)
    assert pose["y"] == pytest.approx(3.5)
    assert pose["z"] == 5 and pose["pitch"] == -12
    path = tmp_path / "raw.png"
    owner.write_png(path, 2, 1, bytes([1, 1, 1, 24, 24, 24]))
    raw = path.read_bytes()
    assert raw[:8] == b"\x89PNG\r\n\x1a\n"
    offset, image_data = 8, b""
    while offset < len(raw):
        length = struct.unpack(">I", raw[offset:offset + 4])[0]
        kind = raw[offset + 4:offset + 8]
        if kind == b"IDAT":
            image_data += raw[offset + 8:offset + 8 + length]
        offset += 12 + length
    assert zlib.decompress(image_data) == bytes([0, 1, 1, 1, 24, 24, 24])


def test_run_output_cannot_be_shared_source_or_workspace_root():
    for path in (stage.PROJECT / "Saved/new_probe", stage.ROOT):
        with pytest.raises(ValueError, match="storage/experiments"):
            stage.validate_run_root(path, new=True)
