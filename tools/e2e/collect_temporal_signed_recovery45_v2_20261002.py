"""HH_261002 - NEW uint32-safe signed collector preserves original exceptions and every unchanged physical guard."""

from __future__ import annotations

import ast
from contextlib import contextmanager
import copy
from pathlib import Path
import sys

from . import collect_temporal_signed_recovery45_20261002 as previous

frozen, collector, task, manual = previous.frozen, previous.collector, previous.task, previous.manual
SCHEMA = "portable_e2e.actual_signed_recovery45.20261002.v2"
V1_SHA = "eee6de2fa1402910eb725bd60724ac1d0753730f12ef47a1492f1e1d964a2fae"
PROFILES = (
    dict(case_id="negative_outward",seed=2026102100,signed_lateral_offset_ros_m=-.20,signed_yaw_offset_ros_deg=-1.5),
    dict(case_id="positive_outward",seed=2026102101,signed_lateral_offset_ros_m=.20,signed_yaw_offset_ros_deg=1.5),
    dict(case_id="centered_setup_comparator",seed=2026102102,signed_lateral_offset_ros_m=0.,signed_yaw_offset_ros_deg=0.),
)
_validate, _spawn, _profile = previous.validate_capture, previous.startup_spawn_scope, previous.profile
_active = False


def validate_seed(seed):
    # HH_261002 - Python Random accepts wider integers; the actual frozen NumPy seed ABI is unsigned32 and is decisive.
    frozen._require(type(seed) is int and 0 <= seed < 2**32, "actual NumPy/CARLA uint32 seed required before simulator launch")
    return seed


def profile(seed):
    validate_seed(seed)
    matches=[r for r in PROFILES if r["seed"] == seed]
    frozen._require(len(matches) == 1, "exact NEW v2 signed-recovery seed required")
    return dict(matches[0])


def verify_frozen():
    import hashlib
    frozen._require(hashlib.sha256(Path(previous.__file__).read_bytes()).hexdigest() == V1_SHA,
        "frozen failed v1 collector must remain byte-exact")
    previous.verify_frozen()
    for row in PROFILES: validate_seed(row["seed"])


def validate_capture(args, route):
    profile(args.seed)
    _validate(args,route)


def preserved_exception_spawn_scope():
    """HH_261002 - Copy the frozen scope with exactly one finalizer conjunct: never mask an active original exception."""
    tree=ast.parse(Path(previous.__file__).read_bytes())
    function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name == "startup_spawn_scope")
    original=copy.deepcopy(function); changed=[]
    for node in ast.walk(function):
        if isinstance(node,ast.If) and ast.unparse(node.test) == "session['spawn_calls'] != 1":
            node.test=ast.BoolOp(op=ast.And(),values=[node.test,ast.parse("sys.exc_info()[0] is None",mode="eval").body]); changed.append(node)
    frozen._require(len(changed) == 1, "exact original no-spawn finalizer anchor required")
    parity=copy.deepcopy(function)
    for node in ast.walk(parity):
        if isinstance(node,ast.If) and isinstance(node.test,ast.BoolOp) and ast.unparse(node.test) == "session['spawn_calls'] != 1 and sys.exc_info()[0] is None":
            node.test=node.test.values[0]
    frozen._require(ast.dump(parity,include_attributes=False) == ast.dump(original,include_attributes=False),
        "all original spawn/control/source restoration statements must remain unchanged")
    namespace=dict(vars(previous)); namespace.update(sys=sys,profile=profile,SCHEMA=SCHEMA)
    module=ast.fix_missing_locations(ast.Module(body=[function],type_ignores=[]))
    exec(compile(module,str(Path(__file__))+":preserve_original_exception","exec"),namespace)
    return namespace["startup_spawn_scope"]


@contextmanager
def activated(model,args):
    """HH_261002 - Fully restored NEW schema/profile/scope bindings; no old source, plan, captured image or control is modified."""
    global _active
    frozen._require(not _active and not previous._active, "v2 signed collector cannot nest")
    verify_frozen(); profile(args.seed)
    frozen._require(previous.profile is _profile and previous.validate_capture is _validate
        and previous.startup_spawn_scope is _spawn, "unknown original signed collector binding")
    names=dict(PROFILES=PROFILES,SCHEMA=SCHEMA,profile=profile,validate_capture=validate_capture,
        startup_spawn_scope=preserved_exception_spawn_scope())
    saved={name:getattr(previous,name) for name in names}; _active=True
    try:
        for name,value in names.items(): setattr(previous,name,value)
        with previous.activated(model,args): yield
    finally:
        for name,value in saved.items(): setattr(previous,name,value)
        _active=False
