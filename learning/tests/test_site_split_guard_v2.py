"""HH_260929 - Keep C-track capture authorization separate from admission and V1."""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import sys

import pytest

from scripts.e2e import site_split_guard as v1
from scripts.e2e import site_split_guard_v2 as v2


@pytest.fixture
def registry():
    return json.loads(v2.REGISTRY.read_text(encoding="utf-8"))


def test_v1_bytes_and_historical_ctrack_denial_are_unchanged():
    # HH_260929 - V2 may not retrospectively rewrite the frozen V1 split.
    for relative, expected_sha256 in v2.PARENT_V1_SHA256.items():
        actual = hashlib.sha256((v2.ROOT / relative).read_bytes()).hexdigest()
        assert actual == expected_sha256
    old = json.loads(v1.REGISTRY.read_text(encoding="utf-8"))
    assert v1.validate_registry(old)["sites"][v2.TARGET_SITE]["map"] == []
    assert not v1.check(v2.TARGET_SITE, "map", "reuse_existing_train", old)
    with pytest.raises(ValueError, match="unknown model or operation"):
        v1.check(v2.TARGET_SITE, "map", v2.CAPTURE_OPERATION, old)


def test_v2_exactly_one_role_delta_and_no_admission(registry):
    assert v2.validate_registry(registry) is registry
    old = json.loads(v1.REGISTRY.read_text(encoding="utf-8"))
    assert len(registry["sites"]) == len(old["sites"]) == 19
    assert registry["independent_dev_sites"] == old["independent_dev_sites"] == []
    assert registry["operations"] == [*old["operations"], v2.CAPTURE_OPERATION]
    expected_sites = deepcopy(old["sites"])
    expected_sites[v2.TARGET_SITE]["map"] = [v2.CAPTURE_OPERATION]
    expected_sites[v2.TARGET_SITE]["reason"] = v2.V2_TARGET_REASON
    assert registry["sites"] == expected_sites
    assert registry["parent_v1_sha256"] == v2.PARENT_V1_SHA256
    assert registry["capture_contract"] == v2.CAPTURE_CONTRACT
    for field in ("dataset_admitted", "train_eligible", "independent_dev_approved",
                  "test_use_authorized"):
        assert registry["capture_contract"][field] is False


def test_only_ctrack_map_capture_allowed(registry):
    assert v2.check(v2.TARGET_SITE, "map", v2.CAPTURE_OPERATION, registry)
    assert not v2.check(v2.TARGET_SITE, "drive", v2.CAPTURE_OPERATION, registry)
    assert not v2.check(v2.TARGET_SITE, "map", "reuse_existing_train", registry)
    for site in set(registry["sites"]) - {v2.TARGET_SITE} | {"Town99"}:
        for model in v2.MODELS:
            assert not v2.check(site, model, v2.CAPTURE_OPERATION, registry)


def test_sealed_test_sites_remain_closed(registry):
    for site in v1.SEALED:
        for model in v2.MODELS:
            for operation in v2.OPERATIONS:
                assert not v2.check(site, model, operation, registry)


@pytest.mark.parametrize("site,model", [
    ("Town04", "map"), ("Town07", "drive"), ("Town03", "map"),
    ("Town05_Opt", "map"), ("Town12", "drive"),
])
def test_reopening_any_other_role_is_rejected(registry, site, model):
    registry["sites"][site][model].append(v2.CAPTURE_OPERATION)
    with pytest.raises(ValueError, match="single approved V1 role delta"):
        v2.validate_registry(registry)


def test_dataset_admission_or_independent_dev_cannot_be_claimed(registry):
    for field in ("dataset_admitted", "train_eligible", "independent_dev_approved",
                  "test_use_authorized"):
        changed = deepcopy(registry)
        changed["capture_contract"][field] = True
        with pytest.raises(ValueError, match="single approved V1 role delta"):
            v2.validate_registry(changed)
    registry["independent_dev_sites"] = [v2.TARGET_SITE]
    with pytest.raises(ValueError, match="single approved V1 role delta"):
        v2.validate_registry(registry)


def test_capture_grant_or_parent_hash_tamper_is_rejected(registry, monkeypatch):
    changed = deepcopy(registry)
    changed["sites"][v2.TARGET_SITE]["map"] = ["reuse_existing_train"]
    with pytest.raises(ValueError, match="single approved V1 role delta"):
        v2.validate_registry(changed)
    changed = deepcopy(registry)
    changed["parent_v1_sha256"][v2.PARENT_REGISTRY] = "0" * 64
    with pytest.raises(ValueError, match="single approved V1 role delta"):
        v2.validate_registry(changed)
    bad_sha = dict(v2.PARENT_V1_SHA256)
    bad_sha[v2.PARENT_REGISTRY] = "0" * 64
    monkeypatch.setattr(v2, "PARENT_V1_SHA256", bad_sha)
    with pytest.raises(ValueError, match="frozen V1 SHA256 changed"):
        v2.validate_registry(registry)


def test_unknown_operation_and_unregistered_site_fail_closed(registry):
    with pytest.raises(ValueError, match="unknown model or operation"):
        v2.check(v2.TARGET_SITE, "map", "independent_dev", registry)
    assert not v2.check("Town99", "map", v2.CAPTURE_OPERATION, registry)
    registry["sites"]["Town99"] = {
        "map": [v2.CAPTURE_OPERATION], "drive": [], "reason": "unreviewed"}
    with pytest.raises(ValueError, match="single approved V1 role delta"):
        v2.validate_registry(registry)


@pytest.mark.parametrize("site,operation,expected_exit,authorized", [
    ("C_track_1_0_7", v2.CAPTURE_OPERATION, 0, True),
    ("Town07", v2.CAPTURE_OPERATION, 2, False),
    ("C_track_1_0_7", "reuse_existing_train", 2, False),
])
def test_cli_distinguishes_capture_permission_from_training_admission(
        monkeypatch, capsys, site, operation, expected_exit, authorized):
    monkeypatch.setattr(sys, "argv", ["site_split_guard_v2.py", "--site", site,
                                      "--model", "map", "--operation", operation])
    if expected_exit:
        with pytest.raises(SystemExit) as error:
            v2.main()
        assert error.value.code == expected_exit
    else:
        v2.main()
    result = json.loads(capsys.readouterr().out)
    assert result["capture_authorized"] is authorized
    assert result["dataset_admitted"] is False
    assert result["train_eligible"] is False
    assert result["permission_scope"] == (
        "capture_only" if authorized else "no_capture_grant")
