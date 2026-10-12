"""HH_261002 - Change only the selected learned-speed preview to0.5s in a new immutable study."""

from __future__ import annotations

import ast
from contextlib import contextmanager
import copy
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path
import time

from . import collect_temporal_route_joint_preview08_signed45_20261002 as source

runtime, previous = source.runtime, source.previous
frozen, collector, task, manual = source.frozen, source.collector, source.task, source.manual
PROFILES, profile, validate_seed, validate_capture = source.PROFILES, source.profile, source.validate_seed, source.validate_capture
THREAD_SCHEMA, thread_readback = source.THREAD_SCHEMA, source.thread_readback
SCHEMA = "portable_e2e.actual_route_joint_thread4_preview05_signed45.20261002.v1"
PREVIEW_SCHEMA = "portable_e2e.causal_selected_speed_preview05_calibration.20261002.v1"
FROZEN_COLLECTOR_SHA = "90a3788fa4be92136cb73b2f3413b4ddbbdd2c0a89f55f5998256e1abbba1498"
_original_session, _actor_config, _trajectory_control = source._original_session, source._actor_config, source._trajectory_control
_active, _proposal_session = False, None


def verify_frozen():
    frozen._require(hashlib.sha256(Path(source.__file__).read_bytes()).hexdigest() == FROZEN_COLLECTOR_SHA,
                    "frozen preview0.8 collector changed")
    source.verify_frozen()


def transformed_definition(name):
    """HH_261002 - Prove all frozen statements unchanged except the explicit speed-preview scalar/class identity."""
    verify_frozen()
    tree = ast.parse(Path(source.__file__).read_bytes())
    original = next(n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef)) and n.name == name)
    changed = copy.deepcopy(original)
    expected = {"actor_controller_config": 1, "preview_selection": 4, "trajectory_control": 0,
                "Preview08ObservedSession": 2, "activated": 1}
    counts = dict(scalar=0, identity=0)

    class Narrow(ast.NodeTransformer):
        def visit_Constant(self, node):
            if type(node.value) is float and node.value == .8:
                node.value = .5; counts["scalar"] += 1
            elif isinstance(node.value,str):
                node.value=node.value.replace("0.8","0.5")
            return node

        def visit_Name(self, node):
            if node.id == "Preview08ObservedSession":
                node.id = "Preview05ObservedSession"; counts["identity"] += 1
            return node

    changed = Narrow().visit(changed)
    if isinstance(changed, ast.ClassDef):
        changed.name = "Preview05ObservedSession"; counts["identity"] += 1
    frozen._require(counts == dict(scalar=expected[name], identity=int(name in ("Preview08ObservedSession", "activated"))),
                    "unexpected preview-only AST substitutions")
    restored = copy.deepcopy(changed)

    class Restore(ast.NodeTransformer):
        def visit_Constant(self, node):
            if type(node.value) is float and node.value == .5: node.value = .8
            elif isinstance(node.value,str):node.value=node.value.replace("0.5","0.8")
            return node

        def visit_Name(self, node):
            if node.id == "Preview05ObservedSession": node.id = "Preview08ObservedSession"
            return node

    restored = Restore().visit(restored)
    if isinstance(restored, ast.ClassDef): restored.name = "Preview08ObservedSession"
    frozen._require(ast.dump(restored) == ast.dump(original), "non-preview collector statement changed")
    return ast.fix_missing_locations(changed)


# HH_261002 - Each copied definition uses this new module's exclusive state; frozen0.8 globals are never patched.
for _name in ("actor_controller_config", "preview_selection", "trajectory_control", "Preview08ObservedSession", "activated"):
    exec(compile(ast.Module(body=[transformed_definition(_name)], type_ignores=[]),
                 str(Path(__file__)) + ":speed_preview0p5_only", "exec"), globals())
del _name
