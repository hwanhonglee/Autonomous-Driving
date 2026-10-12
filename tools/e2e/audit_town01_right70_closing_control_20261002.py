"""HH_261002 - Prospective RIGHT70 receipt identity/count revision; no observed cleanup STOP or LEFT requalification."""
from __future__ import annotations

import ast
import copy
import hashlib
from pathlib import Path

from scripts.e2e import audit_town01_left45_closing_control_20261002 as previous

SCHEMA = "carla.actual_town01_right70_closing_control.20261002.v1"
CLOSING_REASON = "right70_post_capture_cleanup_full_brake"
PREVIOUS_SHA = "2d567108d361b6b12f2c8538935cdc819d06b043dee364dbe8cc296e663fd313"
original, require = previous.original, previous.require
analyze_camera_alignment = original.analyze_camera_alignment
COUNTS = {1055: 1555, 901: 1401, 925: 1425, 900: 1400}
ERROR_IDENTITIES = {
    "one boundary and all1055 declared LEFT45 observations required": "one boundary and all1555 declared RIGHT70 observations required",
    "exact24 warmup/901 driving/130 diagnostic coast phase census required": "exact24 warmup/1401 driving/130 diagnostic coast phase census required"}


def verify_frozen():
    require(hashlib.sha256(Path(previous.__file__).read_bytes()).hexdigest() == PREVIOUS_SHA,
        "immutable LEFT closing reader changed")
    previous.verify_frozen()


def definition():
    # HH_261002 - Only four declared count values (six uses) and two count-bearing error labels differ; every predicate is inverse-AST identical.
    verify_frozen(); tree = ast.parse(Path(previous.__file__).read_bytes())
    old = ast.Module(body=[n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name != "verify_frozen"], type_ignores=[])
    require([n.name for n in old.body] == ["audit_definition", "private_analyze", "coast_context", "closing_receipt", "analyze_transport", "provider"],
        "exact frozen closing/coast/full transport function set required")
    new = copy.deepcopy(old); counts = dict.fromkeys(COUNTS, 0); labels = dict.fromkeys(ERROR_IDENTITIES, 0)
    for n in ast.walk(new):
        if isinstance(n, ast.Constant) and type(n.value) is int and n.value in COUNTS:
            value = n.value; counts[value] += 1; n.value = COUNTS[value]
        elif isinstance(n, ast.Constant) and type(n.value) is str and n.value in ERROR_IDENTITIES:
            value = n.value; labels[value] += 1; n.value = ERROR_IDENTITIES[value]
    require(counts == {1055: 1, 901: 1, 925: 2, 900: 2} and set(labels.values()) == {1},
        "RIGHT70 revision must match all exact count/label occurrences")
    inverse = copy.deepcopy(new)
    for n in ast.walk(inverse):
        if isinstance(n, ast.Constant) and type(n.value) is int and n.value in COUNTS.values():
            n.value = next(old for old, changed in COUNTS.items() if changed == n.value)
        elif isinstance(n, ast.Constant) and type(n.value) is str and n.value in ERROR_IDENTITIES.values():
            n.value = next(old for old, changed in ERROR_IDENTITIES.items() if changed == n.value)
    require(ast.dump(inverse) == ast.dump(old), "RIGHT70 changed an unrelated original receipt/frame/control predicate")
    return new


_namespace = dict(vars(previous), __file__=__file__, SCHEMA=SCHEMA, CLOSING_REASON=CLOSING_REASON, verify_frozen=verify_frozen)
exec(compile(ast.fix_missing_locations(definition()), __file__, "exec"), _namespace)
audit_definition, private_analyze, coast_context, closing_receipt, analyze_transport, provider = (
    _namespace[name] for name in ("audit_definition", "private_analyze", "coast_context", "closing_receipt", "analyze_transport", "provider"))
