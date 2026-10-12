"""HH_260929 - Keep held-out and contaminated Towns closed to new training roles."""

import json
import sys

import pytest
import yaml

from scripts.e2e import site_split_guard as guard


@pytest.fixture
def registry():
    return json.loads(guard.REGISTRY.read_text(encoding="utf-8"))


def test_registry_has_no_independent_dev(registry):
    assert guard.validate_registry(registry)["independent_dev_sites"] == []


def test_every_current_suite_map_has_one_town_level_entry(registry):
    suite = yaml.safe_load((guard.ROOT / "tools/e2e/carla_expert_suite.yaml").read_text())
    names = [row["canonical_name"] for row in suite["maps"]]
    assert len(names) == len(set(names))
    assert set(names) == set(registry["sites"])


def test_existing_map_package_is_exploratory(registry):
    assert guard.check("Town01", "map", "reuse_existing_train", registry)
    assert guard.check("Town02_Opt", "map", "reuse_existing_train", registry)
    assert guard.check("Town05_Opt", "map", "reuse_exploratory_dev", registry)
    assert not guard.check("Town05_Opt", "map", "reuse_existing_train", registry)
    assert not guard.check("Town03", "map", "reuse_existing_train", registry)


def test_town12_only_offline_candidate_and_town13_quality_only(registry):
    assert guard.check("Town12", "drive", "offline_train_scout", registry)
    assert not guard.check("Town12", "drive", "reuse_existing_train", registry)
    assert guard.check("Town13", "map", "quality_diagnostic", registry)
    assert not guard.check("Town13", "map", "reuse_exploratory_dev", registry)


def test_sealed_towns_and_unknown_site_fail_closed(registry):
    for site in guard.SEALED | {"Town99"}:
        for model in guard.MODELS:
            for operation in guard.OPERATIONS:
                assert not guard.check(site, model, operation, registry)


def test_sealed_reopen_is_rejected(registry):
    registry["sites"]["Town04"]["map"] = ["offline_dev_scout"]
    with pytest.raises(ValueError, match="held-out site reopened"):
        guard.validate_registry(registry)


def test_unknown_operation_is_rejected(registry):
    with pytest.raises(ValueError, match="unknown model or operation"):
        guard.check("Town05_Opt", "map", "independent_dev", registry)


def test_contaminated_site_cannot_be_silently_reopened(registry):
    registry["sites"]["Town03"]["map"] = ["reuse_existing_train"]
    with pytest.raises(ValueError, match="locked site role changed"):
        guard.validate_registry(registry)


def test_unverified_site_cannot_be_silently_reopened(registry):
    registry["sites"]["Town11"]["drive"] = ["offline_train_scout"]
    with pytest.raises(ValueError, match="unavailable or unverified site reopened"):
        guard.validate_registry(registry)


def test_unregistered_site_grant_is_rejected(registry):
    registry["sites"]["Town99"] = {
        "map": ["reuse_existing_train"], "drive": [], "reason": "fake admission"}
    with pytest.raises(ValueError, match="site registry differs"):
        guard.validate_registry(registry)


def test_custom_site_role_cannot_be_silently_upgraded(registry):
    registry["sites"]["Woraksan_v1_0_3_parking_lot_hegiht_fit"]["drive"] = [
        "reuse_existing_train"]
    with pytest.raises(ValueError, match="locked site role changed"):
        guard.validate_registry(registry)


def test_partial_cli_role_cannot_look_like_registry_success(monkeypatch):
    # HH_260929 - Omitting a required operation must never return REGISTRY_VALID.
    monkeypatch.setattr(sys, "argv", ["site_split_guard.py", "--site", "Town12",
                                  "--model", "drive"])
    with pytest.raises(SystemExit) as error:
        guard.main()
    assert error.value.code == 2
