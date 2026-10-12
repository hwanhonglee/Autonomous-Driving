"""HH_261002 - Exercise the original cached/global traffic API without CARLA, vehicles, model loads, or actual data."""
import ast
import hashlib
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest
from scripts.e2e import town01_train_left45_green_fixture_20261002 as helper


@pytest.fixture(autouse=True)
def clean_synthetic_scope(monkeypatch):
    monkeypatch.setattr(helper, "_active", None)


def fixture_types():
    # HH_261002 - Reuse only five synthetic API definitions, not the old teacher/NN test imports or actual fixtures.
    source = helper.ROOT/"learning/tests/test_temporal_route_joint_town03_right_v2_20261002.py"
    tree = ast.parse(source.read_bytes())
    names = ("State", "Light", "Actors", "World", "CachedWorld")
    selected = [n for name in names for n in tree.body if isinstance(n, ast.ClassDef) and n.name == name]
    namespace = dict(SimpleNamespace=SimpleNamespace)
    exec(compile(ast.Module(body=selected[:1], type_ignores=[]), __file__+":State", "exec"), namespace)
    namespace.update(GREEN=namespace["State"]("Green"), RED=namespace["State"]("Red"), YELLOW=namespace["State"]("Yellow"))
    exec(compile(ast.Module(body=selected[1:], type_ignores=[]), __file__+":cached_mock_only", "exec"), namespace)
    world = namespace["CachedWorld"](); world.town = "Town01"
    return world, namespace["GREEN"], namespace


def owned(args):
    assert args.port == 2440 and args.owner == "source-pinned-Town01"


def args(): return SimpleNamespace(port=2440, owner="source-pinned-Town01")


def test_original_source_and_only_actual_world_AST_replacement():
    helper.verify_frozen()
    source = ast.parse(helper.SOURCE.read_bytes())
    original = next(n for n in source.body if isinstance(n, ast.ClassDef) and n.name == "GreenSignalFixture")
    selected = helper.fixture_class(owned)
    assert selected.__init__.__globals__["SCHEMA"] == helper.SCHEMA
    assert selected.__init__.__globals__["assert_owned_capture"] is owned
    assert "Town01" in selected.__init__.__code__.co_consts and "Town03" not in selected.__init__.__code__.co_consts
    assert hashlib.sha256(helper.SOURCE.read_bytes()).hexdigest() == helper.SOURCE_SHA
    assert len([n for n in ast.walk(original) if isinstance(n, ast.Constant) and n.value == "Town03"]) == 1


def test_cached_global_install_two_outside_capture_ticks_exact_restore():
    world, green, ns = fixture_types(); manifest = {}
    original = [(a.state, a.frozen, list(a.times)) for a in world.lights]
    with helper.owned_fixture(world, args(), manifest, green, owned) as fixture:
        fixture.install(); assert world.frame == 51
        assert all(a.state is green and a.frozen for a in world.lights)
        assert fixture.check() == 51
        world.settings.synchronous_mode = False; world.settings.fixed_delta_seconds = None
        settings = vars(world.get_settings()).copy()
    assert world.frame == 52 and world.freeze_calls == [True, False]
    assert vars(world.get_settings()) == settings
    assert [(a.state, a.frozen, a.times) for a in world.lights] == original
    record = fixture.record
    assert record["state"] == "RESTORED" and record["teacher_only"] and not record["NN_control_performance_claimed"]
    for role in ("pre_vehicle_fixture_tick", "post_vehicle_restoration_tick"):
        assert record[role]["outside_capture"] and not record[role]["counted_as_NN_or_capture_native"]
        assert record[role]["after_timestamp"]-record[role]["before_timestamp"] == pytest.approx(.05)
    assert helper._active is None


@pytest.mark.parametrize("fault", ["town", "empty", "duplicate", "dead", "boolean", "duration", "heterogeneous"])
def test_exact_census_rejects_before_any_mutation(fault):
    world, green, ns = fixture_types()
    if fault == "town": world.town = "Town03"
    if fault == "empty": world.lights = []
    if fault == "duplicate": world.lights[1].id = 2
    if fault == "dead": world.lights[1].is_alive = False
    if fault == "boolean": world.lights[1].id = True
    if fault == "duration": world.lights[1].times[0] = float("nan")
    if fault == "heterogeneous": world.lights[0].frozen = True
    with pytest.raises(Exception):
        with helper.owned_fixture(world, args(), {}, green, owned): pass
    assert world.frame == 50 and world.freeze_calls == [] and world.pending == {} and helper._active is None


def test_nested_unknown_owner_and_wrong_enum_denied_before_install():
    world, green, ns = fixture_types()
    bad = args(); bad.owner = "wrong-owner"
    with pytest.raises(Exception):
        with helper.owned_fixture(world, bad, {}, green, owned): pass
    with pytest.raises(Exception):
        with helper.owned_fixture(world, args(), {}, ns["RED"], owned): pass
    with helper.owned_fixture(world, args(), {}, green, owned):
        with pytest.raises(Exception, match="overlap"):
            with helper.owned_fixture(world, args(), {}, green, owned): pass
    assert world.freeze_calls == [] and world.frame == 50 and helper._active is None


def test_partial_install_restores_all_originals_and_preserves_original_error():
    world, green, ns = fixture_types(); manifest = {}
    with pytest.raises(RuntimeError, match="original RPC failure"):
        with helper.owned_fixture(world, args(), manifest, green, owned) as fixture:
            original = fixture.actors[1].set_state
            def fail(value):
                fixture.actors[1].set_state = original
                raise RuntimeError("original RPC failure")
            fixture.actors[1].set_state = fail
            fixture.install()
    assert fixture.restored and world.frame == 51
    assert [(a.state.name, a.frozen) for a in world.lights] == [("Yellow", False), ("Red", False)]
    assert helper._active is None


@pytest.mark.parametrize("fault", ["async", "elapsed", "foreign_vehicle"])
def test_no_async_tick_timestamp_or_foreign_world_assumptions(fault):
    world, green, ns = fixture_types()
    with pytest.raises(Exception):
        with helper.owned_fixture(world, args(), {}, green, owned) as fixture:
            if fault == "async": world.settings.synchronous_mode = False
            if fault == "elapsed":
                original = world.get_snapshot
                def shifted():
                    row = original()
                    if row.frame > 50: row.timestamp.elapsed_seconds += .01
                    return row
                world.get_snapshot = shifted
            if fault == "foreign_vehicle":
                original = world.get_actors
                def occupied():
                    actors = original(); prior = actors.filter
                    actors.filter = lambda name: [SimpleNamespace(id=99, type_id="vehicle.other", attributes={})] if name=="vehicle.*" else prior(name)
                    return actors
                world.get_actors = occupied
            fixture.install()


def test_restore_failure_primary_error_retained_and_scope_blocked():
    world, green, ns = fixture_types(); manifest = {}
    with pytest.raises(RuntimeError, match="original capture failure"):
        with helper.owned_fixture(world, args(), manifest, green, owned) as fixture:
            fixture.install(); world.lights.pop()
            raise RuntimeError("original capture failure")
    assert helper._active is not None and manifest["cleanup_errors"]
    assert fixture.record["state"] == "RESTORE_FAILED" and not fixture.record["original_state_freeze_and_duration_restored"]
    with pytest.raises(Exception, match="overlap"):
        with helper.owned_fixture(world, args(), {}, green, owned): pass


def test_duplicate_or_restored_install_denied():
    world, green, ns = fixture_types()
    with helper.owned_fixture(world, args(), {}, green, owned) as fixture:
        fixture.install()
        with pytest.raises(Exception, match="duplicate"): fixture.install()
        fixture.restore()
        with pytest.raises(Exception, match="restored"): fixture.install()
    assert world.freeze_calls == [True, False]


def test_original_source_mutation_denied(monkeypatch):
    monkeypatch.setattr(helper, "SOURCE_SHA", "0"*64)
    world, green, ns = fixture_types()
    with pytest.raises(Exception, match="providers changed"):
        with helper.owned_fixture(world, args(), {}, green, owned): pass
    assert helper._active is None and world.frame == 50
