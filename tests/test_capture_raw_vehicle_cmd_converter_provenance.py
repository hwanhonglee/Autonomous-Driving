from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import pytest


MODULE_PATH = (
    Path(__file__).parents[1]
    / "scripts/e2e/capture_raw_vehicle_cmd_converter_provenance.py"
)
SPEC = importlib.util.spec_from_file_location(
    "capture_raw_vehicle_cmd_converter_provenance", MODULE_PATH
)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_maps(directory: Path) -> dict[str, Path]:
    directory.mkdir(parents=True)
    paths = {}
    for name, value in (("accel", "1"), ("brake", "-1"), ("steer", "0.5")):
        path = directory / f"{name}.csv"
        path.write_text(f"default,0\n0,{value}\n", encoding="utf-8")
        paths[name] = path.resolve()
    return paths


def write_config(path: Path, references: dict[str, str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "/**:\n"
        "  ros__parameters:\n"
        f"    csv_path_accel_map: {references['accel']}\n"
        f"    csv_path_brake_map: {references['brake']}\n"
        f"    csv_path_steer_map: {references['steer']}\n"
        "    convert_accel_cmd: true\n",
        encoding="utf-8",
    )


def test_captures_external_absolute_maps_with_hash_manifest(tmp_path):
    maps = write_maps(tmp_path / "external-maps")
    config = tmp_path / "config/converter.yaml"
    write_config(config, {name: str(path) for name, path in maps.items()})
    output = tmp_path / "artifact/provenance"

    manifest = MODULE.capture_provenance(config, output)

    recorded = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    assert recorded == manifest
    assert recorded["execution"] == {
        "selected_config": str(config.resolve()),
        "uses_artifact_copy": False,
        "uses_original_selected_config": True,
    }
    assert sha256(output / "raw_vehicle_cmd_converter.param.yaml") == sha256(config)
    for name, source in maps.items():
        record = recorded["files"][f"{name}_map"]
        assert record["source"] == str(source)
        assert record["reference_kind"] == "absolute"
        assert record["external_to_config_directory"] is True
        assert record["sha256"] == sha256(source)
        assert sha256(output / record["artifact"]) == sha256(source)


def test_resolves_find_package_share_and_records_package(tmp_path, monkeypatch):
    package_share = tmp_path / "install/share/test_maps"
    maps = write_maps(package_share)
    config = tmp_path / "config/converter.yaml"
    write_config(
        config,
        {
            name: f"$(find-pkg-share test_maps)/{path.name}"
            for name, path in maps.items()
        },
    )
    monkeypatch.setattr(MODULE, "_package_share_directory", lambda package: package_share)

    manifest = MODULE.capture_provenance(config, tmp_path / "provenance")

    for name in maps:
        record = manifest["files"][f"{name}_map"]
        assert record["reference_kind"] == "find-pkg-share"
        assert record["package"] == "test_maps"


@pytest.mark.parametrize(
    "reference, expected",
    [
        ("missing.csv", "relative map reference is ambiguous"),
        ("$(env HOME)/map.csv", "unsupported ROS substitution"),
        ("/definitely/missing/map.csv", "not a file"),
    ],
)
def test_rejects_unresolvable_reference_without_partial_output(
    tmp_path, reference, expected
):
    maps = write_maps(tmp_path / "maps")
    config = tmp_path / "config.yaml"
    references = {name: str(path) for name, path in maps.items()}
    references["accel"] = reference
    write_config(config, references)
    output = tmp_path / "provenance"

    with pytest.raises(ValueError, match=expected):
        MODULE.capture_provenance(config, output)

    assert not output.exists()


def test_rejects_missing_map_parameter(tmp_path):
    config = tmp_path / "config.yaml"
    config.write_text("/**:\n  ros__parameters:\n    csv_path_accel_map: /tmp/a.csv\n")

    with pytest.raises(ValueError, match="missing map parameters"):
        MODULE.capture_provenance(config, tmp_path / "provenance")
