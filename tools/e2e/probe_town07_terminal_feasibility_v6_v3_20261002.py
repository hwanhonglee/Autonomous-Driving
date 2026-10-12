"""HH_261002 - Convert only exact adjacent duplicate mission XY points for the frozen causal Witness ABI.

All213 original coordinates and their exhaustive211-point mapping remain
analysis provenance. This is a source-pinned offline navigation conversion,
never filtering actual states, true64 labels, failed windows or geometry masks.
No model, optimizer, GPU or simulator is loaded, and no TRAIN is authorized.
"""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from types import FunctionType

try:
    from . import probe_town07_terminal_feasibility_v6_v2_20261002 as previous
except ImportError:
    import probe_town07_terminal_feasibility_v6_v2_20261002 as previous

FROZEN_V2_SHA = "ee4debb4944811f82f2d5934bfdea83c68dc75bcbdb1a0e3a93cd9ae10db8587"
SCHEMA = "carla.actual_terminal_boundary_feasibility_probe.20261002.v3"
AUTH_SCHEMA = SCHEMA + ".external_authorization"
ROOT = Path(__file__).resolve().parents[2]
FROZEN = dict(previous.FROZEN, **{
    "tools/e2e/probe_town07_terminal_feasibility_v6_v2_20261002.py": FROZEN_V2_SHA,
})
require, number, encoded, decode, sha = previous.require, previous.number, previous.encoded, previous.decode, previous.sha
CASE_SHA, ROUTE_SHA, QA_SHA, WINDOWS_SHA = previous.CASE_SHA, previous.ROUTE_SHA, previous.QA_SHA, previous.WINDOWS_SHA
witness, INPUT_KEYS, CAMERAS = previous.witness, previous.INPUT_KEYS, previous.CAMERAS


def canonicalize_adjacent_XY(original):
    # HH_261002 - Equal canonical JSON bytes are required; near-equal, signed-zero-distinct and nonadjacent repetitions survive.
    points = original["polyline_map_xy_m"]
    require(type(points) is list and len(points) >= 2, "original mission XY list required")
    canonical, groups, original_to_canonical = [], [], []
    for index, point in enumerate(points):
        require(type(point) is list and len(point) == 2, "original exact two-coordinate XY ABI required")
        for value in point:
            number(value)
        if canonical and encoded(point) == encoded(canonical[-1]):
            groups[-1].append(index)
        else:
            canonical.append(deepcopy(point)); groups.append([index])
        original_to_canonical.append(len(canonical)-1)
    result = deepcopy(original); result["polyline_map_xy_m"] = canonical
    require(result["goal_map_xy_m"] == original["goal_map_xy_m"] == points[-1]
        and result["stop_reference_offset_m"] == original["stop_reference_offset_m"], "endpoint or stop reference changed")
    require(witness._cumulative(points)[-1] == witness._cumulative(canonical)[-1], "exact geometric arc changed")
    provenance = dict(original_point_count=len(points), canonical_point_count=len(canonical),
        original_all_XY_points=deepcopy(points), canonical_all_XY_points=deepcopy(canonical),
        exhaustive_original_to_canonical_index=original_to_canonical,
        exhaustive_canonical_to_original_index_groups=groups,
        collapsed_adjacent_exact_XY_index_groups=[group for group in groups if len(group)>1],
        adjacent_equality_rule="byte-equal canonical JSON XY values only",
        nonadjacent_repetitions_near_points_topology_and_all_source_values_preserved=True,
        source_route_artifact_sha256=original["source_route_artifact_sha256"],
        original_mission_payload_sha256=sha(encoded(original)), canonical_mission_payload_sha256=sha(encoded(result)),
        original_geometric_arc_m=witness._cumulative(points)[-1], canonical_geometric_arc_m=witness._cumulative(canonical)[-1],
        conversion_is_navigation_ABI_only_not_true_future_or_state_filtering=True)
    return result, provenance


def mission_with_provenance(case, route, *, case_sha, route_sha):
    require(previous.previous.digest_file(Path(previous.__file__)) == FROZEN_V2_SHA, "frozen V2 probe source changed")
    original = previous.mission_proposal(case, route, case_sha=case_sha, route_sha=route_sha)
    canonical, provenance = canonicalize_adjacent_XY(original)
    require(provenance["original_point_count"] == 213 and provenance["canonical_point_count"] == 211
        and provenance["collapsed_adjacent_exact_XY_index_groups"] == [[27,28],[115,116]],
        "exact retained213-to211 source conversion required")
    require(canonical["source_route_artifact_sha256"] == ROUTE_SHA
        and canonical["stop_reference_offset_m"] == .75
        and provenance["canonical_geometric_arc_m"] == previous.SERIALIZED_GEOMETRIC_ARC_M,
        "canonical mission reference differs from the exact pinned original")
    return canonical, provenance


def mission_proposal(case, route, *, case_sha, route_sha):
    # HH_261002 - The public PRINT-only helper still returns only exact frozen Witness mission keys.
    return mission_with_provenance(case,route,case_sha=case_sha,route_sha=route_sha)[0]


def _cell(value):
    return (lambda: value).__closure__[0]


def cloned_analysis(namespace):
    # HH_261002 - Preserve the entire authenticated V2 numerical body and its unchanged metadata wrapper bytecode.
    require(previous.analyze.__code__.co_freevars == ("core",) and len(previous.analyze.__closure__) == 1,
        "one frozen V2 analysis core closure required")
    original_core = previous.analyze.__closure__[0].cell_contents
    require(type(original_core) is FunctionType and original_core.__name__ == "analyze" and original_core.__closure__ is None,
        "exact frozen V2 core ABI required")
    core = FunctionType(original_core.__code__, namespace, original_core.__name__)
    outer = FunctionType(previous.analyze.__code__, dict(vars(previous)), previous.analyze.__name__,
        previous.analyze.__defaults__, (_cell(core),))
    return outer


def _namespace():
    require(previous.previous.digest_file(Path(previous.__file__)) == FROZEN_V2_SHA, "frozen V2 probe source changed")
    return dict(previous.run.__globals__, SCHEMA=SCHEMA, AUTH_SCHEMA=AUTH_SCHEMA, ROOT=ROOT,
        __file__=__file__, __doc__=__doc__, FROZEN=FROZEN, mission_proposal=mission_proposal)


def analyze(mission_payload, mission_sha, route_sha, states, cameras, windows, *, source_mission_provenance=None):
    namespace = _namespace()
    summary, rows = cloned_analysis(namespace)(mission_payload,mission_sha,route_sha,states,cameras,windows)
    summary.update(frozen_V2_probe_sha256=FROZEN_V2_SHA,
        navigation_adjacent_duplicate_conversion_provenance=deepcopy(source_mission_provenance),
        all_actual_source_states_true64_and_failed_geometry_masks_preserved=True)
    if source_mission_provenance is not None:
        require(source_mission_provenance["canonical_all_XY_points"] == decode(mission_payload)["polyline_map_xy_m"],
            "canonical mission and conversion provenance differ")
    return summary, rows


def run(args):
    # HH_261002 - Capture original-route provenance in a fresh per-call closure at the unchanged authenticated precheck only.
    namespace = _namespace(); captured = []
    def declared_mission(*positional, **keywords):
        require(not captured, "only one authenticated mission conversion allowed per probe run")
        mission, provenance = mission_with_provenance(*positional, **keywords)
        captured.append(provenance)
        return mission
    def analyzed(*positional, **keywords):
        require(len(captured) == 1, "full original213 point provenance required before actual analysis")
        return analyze(*positional, **keywords, source_mission_provenance=captured[0])
    namespace.update(mission_proposal=declared_mission, analyze=analyzed)
    return FunctionType(previous.run.__code__, namespace, previous.run.__name__)(args)


# HH_261002 - Preserve public argument parsing/source pin behavior while binding the new entrypoint, never frozen globals.
_main_namespace = _namespace(); _main_namespace["run"] = run
main = FunctionType(previous.main.__code__, _main_namespace, previous.main.__name__, previous.main.__defaults__)
native_arc_between, boundary_checks = previous.native_arc_between, previous.boundary_checks

if __name__ == "__main__":
    raise SystemExit(main())
