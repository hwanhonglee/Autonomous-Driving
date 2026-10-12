"""HH_261002 - NEW explicit validator namespace works both in actual owner preflight and activated collector."""

from __future__ import annotations

import ast
from contextlib import contextmanager
from pathlib import Path

from . import collect_temporal_signed_recovery45_v2_20261002 as previous

frozen,collector,task,manual=previous.frozen,previous.collector,previous.task,previous.manual
PROFILES,profile,validate_seed=previous.PROFILES,previous.profile,previous.validate_seed
SCHEMA="portable_e2e.actual_signed_recovery45.20261002.v3"
V2_SHA="d3086074d4b5b66554582b442df89d0a91a6fbb9bf7bcc1e8191863c6797d19e"
_validate=previous.validate_capture
_active=False


def verify_frozen():
    import hashlib
    frozen._require(hashlib.sha256(Path(previous.__file__).read_bytes()).hexdigest() == V2_SHA,
        "already-declared v2 collector must remain byte-exact")
    previous.verify_frozen()


def explicit_original_validator():
    """HH_261002 - Original full validator AST is unchanged; its only explicit namespace change is NEW uint32 profile lookup."""
    original=previous.previous
    tree=ast.parse(Path(original.__file__).read_bytes())
    function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name == "validate_capture")
    namespace=dict(vars(original)); namespace["profile"]=profile
    module=ast.fix_missing_locations(ast.Module(body=[function],type_ignores=[]))
    exec(compile(module,str(Path(__file__))+":unchanged_validator_explicit_profile","exec"),namespace)
    return namespace["validate_capture"]


def validate_capture(args,route):
    profile(args.seed)
    return explicit_original_validator()(args,route)


@contextmanager
def activated(model,args):
    """HH_261002 - Bind public and worker validation identically; retain v2 original-error preservation and unchanged learned activation."""
    global _active
    frozen._require(not _active and not previous._active, "v3 signed collector cannot nest")
    verify_frozen(); profile(args.seed)
    frozen._require(previous.validate_capture is _validate, "unknown v2 validator binding")
    old_schema,old_validate=previous.SCHEMA,previous.validate_capture
    _active=True;previous.SCHEMA=SCHEMA;previous.validate_capture=validate_capture
    try:
        with previous.activated(model,args): yield
    finally:
        previous.SCHEMA=old_schema;previous.validate_capture=old_validate;_active=False
