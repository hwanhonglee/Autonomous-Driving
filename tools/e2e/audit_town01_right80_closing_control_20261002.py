"""HH_261002 - Exact RIGHT80 coast/closing census; final cleanup ACK never creates an observed STOP label."""
from __future__ import annotations

import ast
import copy
import hashlib
from pathlib import Path

from scripts.e2e import audit_town01_right70_closing_control_20261002 as previous

SCHEMA = "carla.actual_town01_right80_closing_control.20261002.v1"
CLOSING_REASON = "right80_post_capture_cleanup_full_brake"
PREVIOUS_SHA = "7a496cff25a9b5129edbaf6c4348dfc279f454d5c0838e166c3a1cef918b7f7d"
original, require = previous.original, previous.require
analyze_camera_alignment = previous.analyze_camera_alignment
COUNTS = {1555: 1755, 1401: 1601, 1425: 1625, 1400: 1600}


def verify_frozen():
    require(hashlib.sha256(Path(previous.__file__).read_bytes()).hexdigest() == PREVIOUS_SHA,
        "immutable RIGHT70 closing reader changed")
    previous.verify_frozen()


def definition():
    verify_frozen(); old = previous.definition(); new = copy.deepcopy(old); changes = []; counts = dict.fromkeys(COUNTS, 0)
    for node in ast.walk(new):
        if isinstance(node, ast.Constant) and type(node.value) is int and node.value in COUNTS:
            before = node.value; counts[before] += 1; changes.append((node, before)); node.value = COUNTS[before]
        elif isinstance(node, ast.Constant) and type(node.value) is str and node.value in previous.ERROR_IDENTITIES.values():
            before = node.value; changes.append((node, before)); node.value = before.replace("RIGHT70", "RIGHT80").replace("1555", "1755").replace("1401", "1601")
    require(counts == {1555: 1, 1401: 1, 1425: 2, 1400: 2} and len(changes) == 8, "four exact count values and two labels required")
    inverse = copy.deepcopy(new); nodes = list(ast.walk(new)); inverse_nodes = list(ast.walk(inverse))
    for node, before in changes: inverse_nodes[nodes.index(node)].value = before
    require(ast.dump(inverse) == ast.dump(old), "RIGHT80 changed receipt/frame/control/130coast predicates")
    return new


_namespace = dict(previous._namespace, __file__=__file__, SCHEMA=SCHEMA, CLOSING_REASON=CLOSING_REASON, verify_frozen=verify_frozen)
exec(compile(ast.fix_missing_locations(definition()), __file__, "exec"), _namespace)
audit_definition, private_analyze, coast_context, closing_receipt, analyze_transport, provider = (
    _namespace[name] for name in ("audit_definition", "private_analyze", "coast_context", "closing_receipt", "analyze_transport", "provider"))
