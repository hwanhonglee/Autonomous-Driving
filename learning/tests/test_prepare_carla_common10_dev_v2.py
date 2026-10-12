"""HH_260906 - Verify DEV converter dispatch cannot bypass its receipt or mutate the legacy adapter."""
from types import ModuleType, SimpleNamespace
import sys

import pytest

from scripts import e2e
from scripts.e2e import prepare_carla_common10_dev_v2 as driver


@pytest.fixture
def authority(monkeypatch):
    module = ModuleType("scripts.e2e.qualify_carla_moving_dev_common10_v2")
    monkeypatch.setitem(sys.modules, module.__name__, module)
    monkeypatch.setattr(e2e, "qualify_carla_moving_dev_common10_v2", module, raising=False)
    return module


@pytest.mark.parametrize("args", [SimpleNamespace(), SimpleNamespace(moving_qualification="receipt"),
    SimpleNamespace(moving_qualification_sha256="a"*64)])
def test_missing_receipt_or_pin_rejected_before_conversion(args, monkeypatch):
    monkeypatch.setattr(driver.base, "prepare_dataset", lambda _: pytest.fail("converter called"))
    with pytest.raises(driver.base.AdapterError, match="paired"):
        driver.prepare_dataset(args)


def test_base_helpers_are_identical():
    assert driver._json is driver.base._json
    assert driver._state_timeline is driver.base._state_timeline
    assert driver.EpisodeInput is driver.base.EpisodeInput


@pytest.mark.parametrize("fail", [False, True])
def test_dispatch_reaches_actual_relative_import_and_restores_legacy(authority, monkeypatch, fail):
    old = driver.importlib.import_module("scripts.e2e.qualify_carla_moving_common10")
    args = SimpleNamespace(moving_qualification="receipt", moving_qualification_sha256="a"*64)
    def convert(value):
        from scripts.e2e import qualify_carla_moving_common10 as actual
        assert actual is authority and value is args
        if fail:
            raise ValueError("fixture rejection")
        return "fixture output"
    monkeypatch.setattr(driver.base, "prepare_dataset", convert)
    if fail:
        with pytest.raises(ValueError, match="fixture rejection"):
            driver.prepare_dataset(args)
    else:
        assert driver.prepare_dataset(args) == "fixture output"
    assert e2e.qualify_carla_moving_common10 is old


def test_main_has_no_unqualified_fallback(monkeypatch):
    monkeypatch.setattr(driver.base, "_parse_args", lambda _: SimpleNamespace())
    monkeypatch.setattr(driver.base, "main", lambda _: pytest.fail("base main called"))
    with pytest.raises(driver.base.AdapterError):
        driver.main([])


def test_identity_writer_only_changes_new_metadata_fields(authority, monkeypatch, tmp_path):
    captured = {}
    def writer(path, value):
        captured[path.name] = value
    monkeypatch.setattr(driver.base, "_write_json", writer)
    original = "carla_fixed_moving_task_to_common10_v1"
    collection = dict(adapter_id=original, native_capture_contract=dict(adapter_id="native unchanged"))
    episode = dict(source_provenance=dict(adapter_id=original, source_manifest_sha256="pin"))
    with driver._bound_dev_qualification():
        driver.base._write_json(tmp_path/"collection_config.json", collection)
        driver.base._write_json(tmp_path/"episode.json", episode)
        driver.base._write_json(tmp_path/"native_manifest.json", collection)
    assert captured["collection_config.json"]["adapter_id"] == driver.ADAPTER_ID
    assert captured["collection_config.json"]["native_capture_contract"] == collection["native_capture_contract"]
    assert captured["episode.json"]["source_provenance"]["adapter_id"] == driver.ADAPTER_ID
    assert captured["native_manifest.json"] is collection
    assert collection["adapter_id"] == original and episode["source_provenance"]["adapter_id"] == original
    assert driver.base._write_json is writer
