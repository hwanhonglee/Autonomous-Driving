"""HH_261002 - Distinguish pinned catalog/serialized route arcs and retain tiny local native stop distances.

This new offline-only revision authenticates the frozen V1 source and retains
its physical limits, source denials, full true64 denominator and causal Witness.
Catalog arc is provenance, not causal support. No model, optimizer or simulator
is loaded, and no actual analysis is authorized by importing this module.
"""

from __future__ import annotations

import ast
import copy
import inspect
import math
from pathlib import Path
from types import FunctionType

try:
    from . import probe_town07_terminal_feasibility_v6_20261002 as previous
except ImportError:
    import probe_town07_terminal_feasibility_v6_20261002 as previous

FROZEN_V1_SHA = "d1fd9081d2b74a4aa76e7740737eb7bf47487788060e6ccabeaaf17f2040e128"
SCHEMA = "carla.actual_terminal_boundary_feasibility_probe.20261002.v2"
AUTH_SCHEMA = SCHEMA + ".external_authorization"
CATALOG_ARC_M = 210.5975914062836
SERIALIZED_GEOMETRIC_ARC_M = 210.59759824658207
SERIALIZED_POINT_COUNT = 213
ROOT = Path(__file__).resolve().parents[2]
FROZEN = dict(previous.FROZEN, **{
    "tools/e2e/probe_town07_terminal_feasibility_v6_20261002.py": FROZEN_V1_SHA,
})
require, number = previous.require, previous.number


def native_arc_between(states, start, end):
    # HH_261002 - Sum actual local nonnegative XY intervals, retaining signed post-event distance without global cancellation.
    require(type(start) is int and type(end) is int and 0 <= start < len(states) and 0 <= end < len(states),
        "exact valid native interval indices required")
    if start == end:
        return 0.
    low, high = min(start, end), max(start, end)
    length = math.fsum(math.hypot(number(b["x"])-number(a["x"]), number(b["y"])-number(a["y"]))
        for a, b in zip(states[low:high], states[low+1:high+1]))
    return length if end > start else -length


def _expression(source):
    return ast.parse(source, mode="eval").body


def _revised_function(function, changes, namespace):
    # HH_261002 - Replace only predeclared exact AST expressions and prove reverse identity with the frozen function.
    require(previous.digest_file(Path(previous.__file__)) == FROZEN_V1_SHA, "frozen V1 probe source changed")
    original = ast.parse(inspect.getsource(function)).body[0]
    revised = copy.deepcopy(original)
    substitutions = []
    for old_text, new_text in changes:
        old, new = _expression(old_text), _expression(new_text)
        matches = [node for node in ast.walk(revised) if ast.dump(node, include_attributes=False)
            == ast.dump(old, include_attributes=False)]
        require(len(matches) == 1, "one exact frozen expression required: " + old_text)
        target = matches[0]
        class Replace(ast.NodeTransformer):
            def visit(self, node):
                if node is target:
                    changed = ast.copy_location(copy.deepcopy(new), node)
                    substitutions.append((changed, node))
                    return changed
                return super().visit(node)
        revised = Replace().visit(revised)
    restored = copy.deepcopy(revised)
    # HH_261002 - Unique new expression identity is tested before exact inverse restoration; unrelated statements cannot change.
    for changed, original_node in reversed(substitutions):
        signature = ast.dump(changed, include_attributes=False)
        matches = [node for node in ast.walk(restored) if ast.dump(node, include_attributes=False) == signature]
        require(len(matches) == 1, "one exact revised expression required for inverse proof")
        target = matches[0]
        class Restore(ast.NodeTransformer):
            def visit(self, node):
                if node is target:
                    return ast.copy_location(copy.deepcopy(original_node), node)
                return super().visit(node)
        restored = Restore().visit(restored)
    require(ast.dump(restored, include_attributes=False) == ast.dump(original, include_attributes=False),
        "a frozen source/physical/cadence/Witness/future/denial statement changed")
    compiled = {}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[revised], type_ignores=[])),
        str(__file__) + ":authenticated_v1_expression_revision", "exec"), namespace, compiled)
    return compiled[function.__name__]


def build_functions():
    # HH_261002 - Construct a private namespace; neither successful calls nor exceptions modify any frozen V1 globals.
    namespace = dict(vars(previous), SCHEMA=SCHEMA, AUTH_SCHEMA=AUTH_SCHEMA, ROOT=ROOT,
        __file__=__file__, __doc__=__doc__, FROZEN=FROZEN, native_arc_between=native_arc_between,
        SERIALIZED_GEOMETRIC_ARC_M=SERIALIZED_GEOMETRIC_ARC_M, SERIALIZED_POINT_COUNT=SERIALIZED_POINT_COUNT)
    mission = _revised_function(previous.mission_proposal, [
        ('abs(witness._cumulative(points)[-1] - case["route_length_m"]) <= 1e-6',
         'len(points) == SERIALIZED_POINT_COUNT and witness._cumulative(points)[-1] == SERIALIZED_GEOMETRIC_ARC_M'),
    ], namespace)
    core = _revised_function(previous.analyze, [
        ('arcs[reference_index]-arcs[i]', 'native_arc_between(states,i,reference_index)'),
        ('arcs[left]-arcs[i]+math.hypot(true[0]["xy"][0]-sum_part(state,states[left])[0],true[0]["xy"][1]-sum_part(state,states[left])[1])',
         'math.fsum((native_arc_between(states,i,left),math.hypot(true[0]["xy"][0]-sum_part(state,states[left])[0],true[0]["xy"][1]-sum_part(state,states[left])[1])))'),
    ], namespace)
    def analyzed(*args, **kwargs):
        summary, rows = core(*args, **kwargs)
        summary.update(original_catalog_route_arc_m=CATALOG_ARC_M,
            exact_original_serialized_XY_geometric_arc_m=SERIALIZED_GEOMETRIC_ARC_M,
            serialized_minus_catalog_arc_m=SERIALIZED_GEOMETRIC_ARC_M-CATALOG_ARC_M,
            catalog_route_arc_not_causal_support=True,
            D_and_first100ms_local_native_arc_use_math_fsum=True,
            frozen_V1_probe_sha256=FROZEN_V1_SHA)
        return summary, rows
    namespace.update(mission_proposal=mission, analyze=analyzed)
    run = FunctionType(previous.run.__code__, namespace, previous.run.__name__)
    namespace["run"] = run
    main = FunctionType(previous.main.__code__, namespace, previous.main.__name__, previous.main.__defaults__)
    return mission, analyzed, run, main


mission_proposal, analyze, run, main = build_functions()

# HH_261002 - Expose unchanged pure helpers for independent synthetic parity fixtures, not source mutation.
encoded, sha, decode = previous.encoded, previous.sha, previous.decode
CASE_SHA, ROUTE_SHA, QA_SHA, WINDOWS_SHA = previous.CASE_SHA, previous.ROUTE_SHA, previous.QA_SHA, previous.WINDOWS_SHA
INPUT_KEYS, CAMERAS, witness = previous.INPUT_KEYS, previous.CAMERAS, previous.witness
exact_zero_reference, causal_observation = previous.exact_zero_reference, previous.causal_observation
boundary_checks, true_heading_diagnostics = previous.boundary_checks, previous.true_heading_diagnostics
native_projection_census = previous.native_projection_census

if __name__ == "__main__":
    raise SystemExit(main())
