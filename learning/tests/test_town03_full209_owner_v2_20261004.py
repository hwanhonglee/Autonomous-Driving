"""HH_261004 - Exercise fresh owner identity and exact frozen guard preservation on CPU, not actual driving."""

from contextlib import contextmanager
from pathlib import Path
from unittest import mock
from types import SimpleNamespace

import pytest

from scripts.e2e import research_temporal_route_joint_town03_straight_full209_v2_20261004 as m
from test_temporal_route_joint_town03_straight_v3_20261004 import pins


@contextmanager
def scope():
    old = m.previous
    threads = old.straight.runtime_owner.previous.previous.threads
    with mock.patch.object(old.numeric, "verify_exports"), mock.patch.object(threads, "evidence_assets", return_value={}):
        with m.activated(pins()):
            yield


def test_frozen_parent_unchanged():
    assert m.digest(Path(m.previous.__file__)) == m.PARENT_SHA256
    m.live.verify_frozen()


def test_fresh_contract_preserves_full_interval_physics_and_raw_ABI():
    old = m.previous
    before = {name:getattr(old,name) for name in ("paths","capture_argv","fixed_contract","runtime_sources","diagnostic_report","live","MODULE","SCHEMA")}
    with scope():
        plan = m.fixed_contract("v2", True)
        argv = m.capture_argv("v2", True, "a"*64)
        assert m.base.live is m.live
        assert old.previous.live is m.live
        assert m.base.paths("v2") == m.paths("v2")
        assert plan["run_id"] == "v2"
    assert {name:getattr(old,name) for name in before} == before
    assert plan["schema"] == m.SCHEMA == m.live.OWNER_SCHEMA
    assert plan["capture_raw_ABI_schema"] == old.live.SCHEMA
    assert plan["total_capture_seconds"] == 110
    assert plan["maximum_native_state_count"] == 2200
    assert plan["maximum_camera_bundle_count"] == 1100
    assert plan["policy_deadline_seconds"] == .1
    assert plan["model_speed_cap_mps"] == 2.5
    assert plan["native_actor_planar_speed_maximum_mps"] == 2.7
    assert plan["controller_target_cap_mps"] == 2.4
    assert plan["speed_preview_s"] == .5
    assert plan["body_clearance_screen"]["lane_inset_m"] == .1
    assert plan["precursor_motion_screen"]["cte_p95_maximum_m"] == .35
    assert plan["precursor_motion_screen"]["cte_absolute_maximum_m"] == .75
    assert plan["source_manifest_sealed_before_declaration"] is True
    assert plan["attempts"] == 1 and plan["automatic_retry"] is False
    assert plan["official_completed_main_domains"] == plan["official_completed_subfunctions"] == 0
    assert plan["full_RTE01_approved"] is plan["model_promoted"] is False
    assert argv[argv.index("--max-duration-sec")+1] == "110"
    assert argv[argv.index("--port")+1] == "2440"
    assert argv[argv.index("--stationary-tail-sec")+1] == "0"
    worker = plan["worker_command"]
    assert worker[worker.index("-m")+1] == m.MODULE
    assert worker[worker.index("--run-id")+1] == "v2"
    assert plan["authenticated_diagnostic_revision"]["no_threshold_tolerance_increase"] is True
    assert plan["terminal_scalar_timing_research"]["every_original_check_and_operation_unchanged"] is True
    assert plan["prospective_visual_publication"]["outcome_or_failure_dependent_frame_selection"] is False
    assert plan["prospective_visual_publication"]["gallery"].endswith("12_town03_full_route209_v2")


def test_nested_owner_rejects_and_restores_on_failure():
    original = m.previous.paths
    with pytest.raises(RuntimeError, match="fresh V2 owner cannot nest"):
        with scope():
            with m.activated(pins()):
                pass
    assert not m._active and not m.previous._active
    assert m.previous.paths is original


@pytest.mark.parametrize("run_id", ["v1","v3","../v2",None])
def test_wrong_fresh_identity_rejects(run_id):
    with pytest.raises(RuntimeError):
        m.paths(run_id)


def test_standalone_capture_rejects_without_owner():
    with pytest.raises(RuntimeError, match="exclusive V2 owner"):
        m.capture_argv("v2", True)


@contextmanager
def scalar_collector_scope(*, admitted_owner=None, parent_exception=None):
    """HH_261004 - Exercise the real fresh collector wrapper with a scalar parent scope, never CARLA or actuation."""
    fresh, old = m.live, m.live.previous
    fixture = old.fixture
    original = dict(owner=old.OWNER_SCHEMA, factory=old.navigation_terminal_policy_class,
        fixture_owner=fixture.OWNER_SCHEMA, fixture_schema=fixture.SCHEMA,
        fresh_active=fresh._active, parent_active=old._active)
    observations = []
    @contextmanager
    def fake_owned_parent(model, args):
        assert old.OWNER_SCHEMA == fresh.OWNER_SCHEMA
        assert old.SCHEMA == fresh.SCHEMA == m.previous.live.SCHEMA
        from portable_e2e.temporal_navigation_terminal_policy_v3_timing_20261004 import navigation_terminal_policy_class
        assert old.navigation_terminal_policy_class is navigation_terminal_policy_class
        assert fresh._active is True
        old._active = True
        try:
            with mock.patch.object(fixture, "OWNER_SCHEMA", admitted_owner or fresh.OWNER_SCHEMA), \
                    mock.patch.object(fixture, "SCHEMA", fresh.SCHEMA):
                observations.append(dict(assert_owner=fixture.assert_owned_capture.__globals__["OWNER_SCHEMA"],
                    constructor_owner=fixture.GreenSignalFixture.__init__.__globals__["OWNER_SCHEMA"],
                    actual_raw_schema=fixture.assert_owned_capture.__globals__["SCHEMA"]))
                if parent_exception is not None:
                    raise parent_exception
                yield
        finally:
            old._active = original["parent_active"]
    try:
        with mock.patch.object(old, "activated", fake_owned_parent):
            yield fresh, original, observations
    finally:
        assert old.OWNER_SCHEMA == original["owner"]
        assert old.navigation_terminal_policy_class is original["factory"]
        assert fixture.OWNER_SCHEMA == original["fixture_owner"]
        assert fixture.SCHEMA == original["fixture_schema"]
        assert fresh._active is original["fresh_active"]
        assert old._active is original["parent_active"]


def test_actual_two_fixture_globals_admit_fresh_owner_with_original_raw_ABI():
    with scalar_collector_scope() as (fresh, original, observations):
        with fresh.activated(object(), SimpleNamespace()):
            assert observations == [dict(assert_owner=fresh.OWNER_SCHEMA, constructor_owner=fresh.OWNER_SCHEMA,
                actual_raw_schema=fresh.SCHEMA)]
            assert fresh.previous.OWNER_SCHEMA != original["owner"]
            assert fresh.OWNER_SCHEMA.endswith(".v2")
            assert fresh.SCHEMA.endswith(".v1")
            assert fresh.previous._active and fresh._active


def test_stale_actual_fixture_owner_denies_and_restores_every_new_alias():
    with scalar_collector_scope(admitted_owner="STALE_SYNTHETIC_CPU_OWNER") as (fresh, original, observations):
        with pytest.raises(RuntimeError, match="actual Green fixture source guard"):
            with fresh.activated(object(), SimpleNamespace()):
                pytest.fail("stale real fixture global must never enter a capture scope")
        assert observations[0]["assert_owner"] == observations[0]["constructor_owner"] == "STALE_SYNTHETIC_CPU_OWNER"
        assert not fresh._active and not fresh.previous._active
        assert fresh.previous.navigation_terminal_policy_class is original["factory"]


def test_parent_scope_setup_exception_preserves_identity_and_restores_new_aliases():
    marker = RuntimeError("synthetic owned parent setup failed before any simulator action")
    with scalar_collector_scope(parent_exception=marker) as (fresh, original, observations):
        with pytest.raises(RuntimeError) as error:
            with fresh.activated(object(), SimpleNamespace()):
                pytest.fail("failed parent setup must not yield")
        assert error.value is marker and len(observations) == 1
        assert fresh.previous.navigation_terminal_policy_class is original["factory"]
        assert not fresh._active and not fresh.previous._active


def test_inner_body_exception_preserves_identity_and_restores_both_fixture_globals():
    marker = RuntimeError("synthetic body exception after owner admission, before simulator use")
    with scalar_collector_scope() as (fresh, original, observations):
        with pytest.raises(RuntimeError) as error:
            with fresh.activated(object(), SimpleNamespace()):
                assert observations[0]["assert_owner"] == observations[0]["constructor_owner"] == fresh.OWNER_SCHEMA
                raise marker
        assert error.value is marker
        assert not fresh._active and not fresh.previous._active
        assert fresh.previous.navigation_terminal_policy_class is original["factory"]
