import copy
import importlib.util
import json
import sys
import types
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/e2e/inventory_carla_training_maps.py"
MANIFEST = ROOT / "scripts/e2e/carla_expert_suite.yaml"


def load_module():
    spec = importlib.util.spec_from_file_location("inventory_carla_training_maps", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_canonical_manifest_contract():
    module = load_module()
    document, path = module.load_manifest(MANIFEST)

    assert path == MANIFEST.resolve()
    assert set(document["server_profiles"]) == {
        "packaged_0915",
        "source_editor_0915_4ws",
    }
    maps = {entry["id"]: entry for entry in document["maps"]}
    assert set(maps) == set(module.MAP_CONTRACT)
    assert maps["town10hd"]["load_name"] == "/Game/Carla/Maps/Town10HD"
    assert maps["town10hd"]["status"] == "unavailable"
    assert maps["town10hd"]["split"] == "excluded"
    assert maps["town10hd_opt"]["load_name"] == "/Game/Carla/Maps/Town10HD_Opt"
    assert maps["town10hd_opt"]["status"] == "ready"
    assert maps["town10hd_opt"]["split"] == "train"
    assert maps["town13"]["split"] == "validation"
    assert maps["town02_opt"]["load_name"] == "/Game/Carla/Maps/Town02_Opt"
    assert maps["town02_opt"]["status"] == "ready"
    assert maps["town05"]["status"] == "unavailable"
    assert maps["town05"]["split"] == "excluded"
    assert "use town05_opt" in maps["town05"]["reason"]
    assert maps["town05_opt"]["load_name"] == "/Game/Carla/Maps/Town05_Opt"
    assert maps["town05_opt"]["status"] == "ready"
    assert maps["town03"]["server_profile"] == "packaged_0915"
    assert maps["town03"]["status"] == "ready"
    assert maps["town03"]["split"] == "train"
    assert "without client.load_world" in maps["town03"]["reason"]
    assert "six-camera visual QA" in maps["town03"]["reason"]
    assert maps["c_track_1_0_7"]["load_name"] == "/Game/Carla/Maps/C_track_1_0_7"
    assert maps["c_track_1_0_7"]["status"] == "ready"
    assert maps["c_track_1_0_7"]["server_profile"] == "packaged_0915"
    assert maps["c_track_1_0_7"]["split"] == "train"
    assert "--quality Epic" in maps["c_track_1_0_7"]["reason"]
    assert maps["woraksan_1_0_3"]["load_name"].endswith(
        "/Woraksan_v1_0_3_parking_lot_hegiht_fit"
    )
    assert maps["woraksan_1_0_3"]["status"] == "ready"
    assert maps["woraksan_1_0_3"]["server_profile"] == "packaged_0915"
    assert maps["woraksan_1_0_3"]["split"] == "train"
    assert "spawn-4 exclusion" in maps["woraksan_1_0_3"]["reason"]


def test_unavailable_towns_are_explicitly_excluded():
    module = load_module()
    document, _ = module.load_manifest(MANIFEST)
    maps = {entry["id"]: entry for entry in document["maps"]}

    for map_id in ("town02", "town05", "town08", "town09", "town10hd"):
        assert maps[map_id]["status"] == "unavailable"
        assert maps[map_id]["split"] == "excluded"
        assert maps[map_id]["server_profile"] is None
        assert maps[map_id]["level_path"] is None
        assert maps[map_id]["opendrive_path"] is None
    assert "renderer crashes" in maps["town02"]["reason"]
    assert "use town10hd_opt" in maps["town10hd"]["reason"]
    for map_id in ("town08", "town09"):
        assert "unseen" in maps[map_id]["reason"]


def test_runtime_bindings_are_relocatable_and_missing_assets_are_reported(
    tmp_path, monkeypatch
):
    module = load_module()
    carla_root = tmp_path / "carla-package"
    carla_source_root = tmp_path / "carla-source"
    ue4_root = tmp_path / "unreal-engine"
    monkeypatch.setenv("CARLA_ROOT", str(carla_root))
    monkeypatch.setenv("CARLA_SOURCE_ROOT", str(carla_source_root))
    monkeypatch.setenv("UE4_ROOT", str(ue4_root))

    town02_level = carla_root / "CarlaUE4/Content/Carla/Maps/Town02_Opt.umap"
    town02_xodr = (
        carla_root / "CarlaUE4/Content/Carla/Maps/OpenDrive/Town02_Opt.xodr"
    )
    town02_level.parent.mkdir(parents=True)
    town02_xodr.parent.mkdir(parents=True)
    town02_level.write_bytes(b"umap")
    town02_xodr.write_text("<OpenDRIVE/>", encoding="utf-8")

    document, _ = module.load_manifest(MANIFEST)
    maps = {entry["id"]: entry for entry in document["maps"]}

    assert document["server_profiles"]["packaged_0915"]["executable"] == str(
        carla_root / "CarlaUE4.sh"
    )
    assert maps["town02_opt"]["level_path"] == str(town02_level)
    assert maps["town02_opt"]["opendrive_path"] == str(town02_xodr)
    assert maps["town12"]["level_path"].startswith(str(carla_source_root))
    assert maps["town10hd_opt"]["level_path"].endswith("/Town10HD_Opt.umap")
    assert maps["town10hd_opt"]["opendrive_path"].endswith("/Town10HD.xodr")

    static = module.collect_static_inventory(document)
    static_maps = {entry["id"]: entry for entry in static["maps"]}
    assert static_maps["town02_opt"]["assets_complete"] is True
    assert static_maps["woraksan_1_0_3"]["assets_complete"] is False
    assert "woraksan_1_0_3" in static["missing_asset_map_ids"]
    assert "/home/hong" not in MANIFEST.read_text(encoding="utf-8")


def test_runtime_root_environment_must_be_absolute(monkeypatch):
    module = load_module()
    monkeypatch.setenv("CARLA_ROOT", "relative/carla")

    with pytest.raises(module.ManifestError, match="CARLA_ROOT must be an absolute"):
        module.load_manifest(MANIFEST)


def test_validator_rejects_canonical_load_name_drift():
    module = load_module()
    document, _ = module.load_manifest(MANIFEST)
    changed = copy.deepcopy(document)
    next(item for item in changed["maps"] if item["id"] == "town12")[
        "load_name"
    ] = "Town12"

    with pytest.raises(module.ManifestError, match="canonical contract mismatch"):
        module.validate_manifest(changed)


def test_validator_rejects_c_track_removal_from_training_split():
    module = load_module()
    document, _ = module.load_manifest(MANIFEST)
    changed = copy.deepcopy(document)
    next(item for item in changed["maps"] if item["id"] == "c_track_1_0_7")[
        "split"
    ] = "excluded"

    with pytest.raises(module.ManifestError, match="split canonical contract mismatch"):
        module.validate_manifest(changed)


def test_validator_rejects_woraksan_reentry_into_evaluation_split():
    module = load_module()
    document, _ = module.load_manifest(MANIFEST)
    changed = copy.deepcopy(document)
    next(item for item in changed["maps"] if item["id"] == "woraksan_1_0_3")[
        "split"
    ] = "test"

    with pytest.raises(module.ManifestError, match="split canonical contract mismatch"):
        module.validate_manifest(changed)


def test_validator_rejects_town03_removal_from_training_split():
    module = load_module()
    document, _ = module.load_manifest(MANIFEST)
    changed = copy.deepcopy(document)
    next(item for item in changed["maps"] if item["id"] == "town03")["split"] = (
        "excluded"
    )

    with pytest.raises(module.ManifestError, match="split canonical contract mismatch"):
        module.validate_manifest(changed)


def test_validator_rejects_server_python_api_drift():
    module = load_module()
    document, _ = module.load_manifest(MANIFEST)
    changed = copy.deepcopy(document)
    changed["server_profiles"]["source_editor_0915_4ws"]["python_api"] = (
        "/tmp/carla.egg"
    )

    with pytest.raises(module.ManifestError, match="profile .* canonical contract mismatch"):
        module.validate_manifest(changed)


def test_validator_rejects_missing_map_and_duplicate_id():
    module = load_module()
    document, _ = module.load_manifest(MANIFEST)
    missing = copy.deepcopy(document)
    missing["maps"] = missing["maps"][:-1]
    with pytest.raises(module.ManifestError, match="canonical map set mismatch"):
        module.validate_manifest(missing)

    duplicate = copy.deepcopy(document)
    duplicate["maps"][-1]["id"] = "town01"
    with pytest.raises(module.ManifestError, match="duplicate map id"):
        module.validate_manifest(duplicate)


def test_live_inventory_only_inspects_current_world(monkeypatch):
    module = load_module()
    document, _ = module.load_manifest(MANIFEST)

    class FakeMap:
        name = "Carla/Maps/UEDPIE_0_Town01"

        @staticmethod
        def get_topology():
            return [1, 2, 3]

        @staticmethod
        def get_spawn_points():
            return [1, 2]

    class FakeWorld:
        @staticmethod
        def get_map():
            return FakeMap()

    class FakeClient:
        def __init__(self, host, port):
            assert (host, port) == ("127.0.0.1", 2100)

        def set_timeout(self, timeout):
            assert timeout == 3.0

        @staticmethod
        def get_available_maps():
            return [
                "/Game/Carla/Maps/Town01",
                "/Game/Carla/Maps/Town02",
            ]

        @staticmethod
        def get_world():
            return FakeWorld()

        @staticmethod
        def get_client_version():
            return "0.9.15"

        @staticmethod
        def get_server_version():
            return "0.9.15"

        def load_world(self, *_args, **_kwargs):
            raise AssertionError("inventory must not load a map")

    monkeypatch.setitem(sys.modules, "carla", types.SimpleNamespace(Client=FakeClient))
    report = module.collect_live_inventory(document, "127.0.0.1", 2100, 3.0)

    assert report["success"] is True
    assert report["map_loading_performed"] is False
    assert report["current_map"] == "/Game/Carla/Maps/Town01"
    assert report["current_topology_count"] == 3
    assert report["current_spawn_point_count"] == 2
    maps = {entry["id"]: entry for entry in report["maps"]}
    assert maps["town01"]["topology_count"] == 3
    assert maps["town02"]["topology_count"] is None


def test_static_cli_writes_atomic_json_without_carla(tmp_path, monkeypatch):
    module = load_module()
    output = tmp_path / "nested" / "inventory.json"
    monkeypatch.delitem(sys.modules, "carla", raising=False)

    assert module.main(["--manifest", str(MANIFEST), "--output", str(output)]) == 0
    report = json.loads(output.read_text(encoding="utf-8"))
    assert report["static"]["success"] is True
    assert report["static"]["map_count"] == 19
    assert report["static"]["status_counts"] == {
        "ready": 10,
        "source_editor_required": 4,
        "unavailable": 5,
    }
    assert len(report["static"]["maps"]) == 19
    assert report["static"]["runtime_roots"]["CARLA_ROOT"] == str(
        module.runtime_roots()["CARLA_ROOT"]
    )
    assert report["live"] is None
    assert not list(output.parent.glob("*.tmp"))


def test_source_contains_no_map_loading_call():
    source = SCRIPT.read_text(encoding="utf-8")
    forbidden_method = "load" + "_world"
    assert forbidden_method not in source
