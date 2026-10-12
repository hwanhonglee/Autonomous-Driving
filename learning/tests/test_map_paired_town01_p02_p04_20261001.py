"""HH_261001 - Test the new p02/p04 boundaries without launching CARLA."""

import ast
import hashlib
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import collect_map_paired_town01_p02_p04_20261001 as capture
from scripts.e2e import audit_map_paired_town01_p02_p04_20261001 as qa
from scripts.e2e import run_owned_map_paired_town01_p02_p04_20261001 as owner


@pytest.mark.parametrize("arm,filename,sha", [
    ("right", "town01_right_s0000_p02.json", "788665af9e3b5449634f1c5392e94b2a6258ae3262d26e69f3cf81b96791489a"),
    ("left", "town01_left_s0000_p04.json", "f4d474eee01892e16b8b78a69de5a3b32ccc8e4f227e30a9a89dc023dbfc25a7"),
])
def test_exact_catalog_routes_and_weather(arm, filename, sha):
    path = capture.ROUTE_ROOT / arm / filename
    route, observed = capture.checked_route(path)
    assert observed == sha == hashlib.sha256(path.read_bytes()).hexdigest()
    assert (route["town"], route["scenario"], route["weather"], route["coordinate_reference"]) == (
        "Town01", arm, "ClearNoon", "base_link")


def test_rejects_relocated_or_unrelated_route(tmp_path):
    source = capture.ROUTE_ROOT / "right/town01_right_s0000_p02.json"
    copy = tmp_path / source.name
    copy.write_bytes(source.read_bytes())
    with pytest.raises(ValueError, match="catalog path"):
        capture.checked_route(copy)
    with pytest.raises(ValueError, match="p02/p04 route bytes"):
        capture.checked_route(capture.ROUTE_ROOT / "right/town01_right_s0000_p01.json")


def test_parent_engine_scope_restores_frozen_routes_and_role():
    parent = capture.base
    old_routes, old_role = parent.ROUTES, parent.ROLE
    with capture.scoped_parent():
        assert set(parent.ROUTES) == set(capture.ROUTES)
        assert parent.ROLE == capture.ROLE
    assert parent.ROUTES is old_routes and parent.ROLE == old_role


def test_owner_default_is_no_launch_and_exact_caps(monkeypatch):
    monkeypatch.setattr(owner, "check_storage", lambda *_: None)
    monkeypatch.setattr(capture.base.shutil, "disk_usage", lambda *_: SimpleNamespace(free=7 * 1024**3))
    args = owner.parse_args([str(owner.CAMPAIGN / "p02"),
        str(capture.ROUTE_ROOT / "right/town01_right_s0000_p02.json")])
    assert args.execute is False and args.output_name == "p02"
    assert owner.MIN_START_FREE == 6 * 1024**3
    assert owner.MIN_RUNNING_FREE == 5 * 1024**3
    assert owner.MAX_OWNER_BYTES == 400 * 1024**2
    assert owner.MAX_CAMPAIGN_BYTES == 1024**3
    monkeypatch.setattr(owner, "execute", lambda *_: pytest.fail("preflight launched a simulator"))
    assert owner.main([str(owner.CAMPAIGN / "p02"),
        str(capture.ROUTE_ROOT / "right/town01_right_s0000_p02.json")]) == 0


def test_owner_rejects_unscoped_output_and_requires_prior_qa(monkeypatch):
    monkeypatch.setattr(owner, "check_storage", lambda *_: None)
    monkeypatch.setattr(capture.base.shutil, "disk_usage", lambda *_: SimpleNamespace(free=7 * 1024**3))
    right = capture.ROUTE_ROOT / "right/town01_right_s0000_p02.json"
    left = capture.ROUTE_ROOT / "left/town01_left_s0000_p04.json"
    with pytest.raises(SystemExit):
        owner.parse_args([str(owner.CAMPAIGN / "extra"), str(right)])
    with pytest.raises(SystemExit):
        owner.parse_args([str(owner.CAMPAIGN / "p04"), str(left)])


def test_owner_rejects_foreign_carla_package(monkeypatch):
    monkeypatch.setenv("CARLA_ROOT", "/tmp/not-the-pinned-package")
    with pytest.raises(SystemExit):
        owner.parse_args([str(owner.CAMPAIGN / "p02"),
            str(capture.ROUTE_ROOT / "right/town01_right_s0000_p02.json")])


def test_source_closure_and_shell_are_scoped():
    assert {
        "tools/e2e/collect_map_paired_rgb_20261001_v2.py",
        "tools/e2e/collect_map_paired_town01_p02_p04_20261001.py",
        "tools/e2e/audit_map_paired_town01_p02_p04_20261001.py",
        "learning/portable_e2e/mapping_projection.py",
    }.issubset(set(owner.SOURCES))
    for logical in owner.SOURCES:
        assert (owner.ROOT / logical).is_file()
    shell = (owner.ROOT / "tools/e2e/run_owned_map_paired_town01_p02_p04_20261001.sh").read_text()
    assert "workspace_python.py -m scripts.e2e.run_owned_map_paired_town01_p02_p04_20261001" in shell
    ast.parse((owner.ROOT / "tools/e2e/run_owned_map_paired_town01_p02_p04_20261001.py").read_text())


def test_auditor_refuses_sources_outside_new_campaign(tmp_path):
    with pytest.raises(ValueError, match="canonical p02 or p04"):
        qa.audit(tmp_path / "mapping")


def test_owner_recognizes_engine_and_startup_wrappers():
    assert owner.is_carla_process_argv([b"/path/CarlaUE4-Linux-Shipping"])
    assert owner.is_carla_process_argv([b"/bin/bash", b"/path/run_carla_map.sh"])
    assert owner.is_carla_process_argv([b"/bin/bash", b"/path/CarlaUE4.sh"])
    assert not owner.is_carla_process_argv([b"/bin/bash", b"/path/other.sh"])


def test_group_cleanup_refuses_a_reused_leader_pid(monkeypatch):
    class ReusedProcess:
        pid = 24680
        owned_start_ticks = 100

    monkeypatch.setattr(owner, "process_record", lambda _: ("S", 24680, 101))
    monkeypatch.setattr(owner.os, "getpgrp", lambda: 22222)
    monkeypatch.setattr(owner.os, "killpg", lambda *_: pytest.fail("foreign group signaled"))
    assert owner.stop_group(ReusedProcess()) is False
