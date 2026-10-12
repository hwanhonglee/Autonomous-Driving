"""HH_261002 - Reuse the exact owned Green-fixture physics for a prospective Town01 teacher, not traffic-rule performance."""
from __future__ import annotations

import ast
import copy
from contextlib import contextmanager
import hashlib
import inspect
import math
from pathlib import Path
from types import SimpleNamespace
import sys

from . import collect_carla_vad_expert as collector

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "tools/e2e/collect_temporal_route_joint_town03_right_v2_20261002.py"
SOURCE_SHA = "3473cbb37d771137a01459d7d634a4ce372e551059043904dd0c538bce0ffa04"
COLLECTOR_SHA = "a41eac9019c644fec03396a50645fa021b63230a4ca86a664e8044dfd170ae08"
SCHEMA = "portable_e2e.town01_train_left45_green_fixture.20261002.v1"
_active = None


def require(condition, message):
    if not condition:
        raise collector.CollectionError(message)


def verify_frozen():
    require(hashlib.sha256(SOURCE.read_bytes()).hexdigest() == SOURCE_SHA
        and hashlib.sha256(Path(collector.__file__).read_bytes()).hexdigest() == COLLECTOR_SHA,
        "frozen signal/empty-world physics providers changed")


def fixture_class(assert_owned_capture):
    # HH_261002 - Authenticate the source before executing only its four fixture definitions, not its NN/owner imports.
    verify_frozen()
    require(inspect.isfunction(assert_owned_capture), "explicit source-owned capture assertion function required")
    tree = ast.parse(SOURCE.read_bytes())
    names = ("finite", "light_state", "actor_id", "GreenSignalFixture")
    selected = [copy.deepcopy(n) for name in names for n in tree.body
        if isinstance(n, (ast.FunctionDef, ast.ClassDef)) and n.name == name]
    require(len(selected) == 4, "exact original signal fixture definitions required")
    original = copy.deepcopy(selected[-1]); count = 0
    for node in ast.walk(selected[-1]):
        if isinstance(node, ast.Constant) and node.value == "Town03":
            node.value = "Town01"; count += 1
    require(count == 1, "exact actual-world name assertion must be the sole class change")
    undone = copy.deepcopy(selected[-1])
    for node in ast.walk(undone):
        if isinstance(node, ast.Constant) and node.value == "Town01": node.value = "Town03"
    require(ast.dump(undone) == ast.dump(original), "another signal physics/restore statement changed")
    namespace = dict(__name__=__name__, collector=collector, frozen=SimpleNamespace(_require=require),
        math=math, SCHEMA=SCHEMA, assert_owned_capture=assert_owned_capture)
    exec(compile(ast.fix_missing_locations(ast.Module(body=selected, type_ignores=[])),
        str(SOURCE)+":exact_fixture_Town01_private_owner", "exec"), namespace)
    return namespace["GreenSignalFixture"]


@contextmanager
def owned_fixture(world, args, manifest, green, assert_owned_capture):
    """Exit this scope ONLY after original capture destroys ego/sensors; its separate restoration tick is not capture evidence."""
    global _active
    require(_active is None, "owned Town01 Green fixtures cannot overlap or nest")
    green_name = getattr(green, "name", None)
    if not isinstance(green_name, str): green_name = str(green).rsplit(".", 1)[-1]
    require(green_name == "Green", "actual Green enum required")
    original = fixture_class(assert_owned_capture)
    token = object(); _active = token
    try:
        fixture = original(world, args, manifest, green)
    except BaseException:
        _active = None
        raise
    restore, install = fixture.restore, fixture.install
    lifetime = dict(install_attempted=False, restore_completed=False)
    def install_owned():
        require(_active is token and not lifetime["install_attempted"] and not lifetime["restore_completed"],
                "unknown, duplicate, or already-restored fixture installation")
        lifetime["install_attempted"] = True
        return install()
    def restore_owned():
        require(_active is token and not lifetime["restore_completed"], "unknown or already-restored fixture lifetime")
        # HH_261002 - A failed restoration keeps the process-level scope blocked; do not install another fixture over it.
        value = restore()
        lifetime["restore_completed"] = True
        return value
    fixture.restore, fixture.install = restore_owned, install_owned
    fixture.record.update(fixture_source_sha256=SOURCE_SHA, fixture_adapter_schema=SCHEMA,
        actual_world="Town01", teacher_only=True, NN_control_performance_claimed=False,
        owner_assertion_supplied_by_source_sealed_capture=True,
        original_fixture_class_AST_changed_only_actual_world_literal=True)
    try: yield fixture
    finally:
        original_failure = sys.exc_info()[0] is not None
        try:
            if lifetime["install_attempted"] and not lifetime["restore_completed"]: restore_owned()
        except Exception as error:
            fixture.record.update(state="RESTORE_FAILED", original_state_freeze_and_duration_restored=False,
                adapter_restoration_error=type(error).__name__+": "+str(error))
            manifest.setdefault("cleanup_errors", []).append("Town01 signal fixture: "+type(error).__name__+": "+str(error))
            if not original_failure: raise
        finally:
            if not lifetime["install_attempted"] or lifetime["restore_completed"]: _active = None
