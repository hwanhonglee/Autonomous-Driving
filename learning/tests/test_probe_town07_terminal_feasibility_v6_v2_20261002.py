"""HH_261002 - Synthetic source-parity and tiny native arc tests; no actual mission/probe/model/vehicle jobs."""

from copy import deepcopy
import math
from pathlib import Path
import subprocess
import sys

import pytest

from scripts.e2e import probe_town07_terminal_feasibility_v6_v2_20261002 as probe
from test_probe_town07_terminal_feasibility_v6_20261002 import fixture


def original_catalog_fixture():
    # HH_261002 - Exactly213 synthetic serialized points reproduce the declared catalog/geometric distinction without real payload reads.
    points = [{"x": i*.5, "y": 0.} for i in range(212)] + [{"x": probe.SERIALIZED_GEOMETRIC_ARC_M, "y": 0.}]
    case = dict(schema="carla.actual_natural_goal_stop_teacher.20261002.v6", route_sha256=probe.ROUTE_SHA,
        route_length_m=probe.CATALOG_ARC_M, town="Town07", natural_goal_configuration=dict(stop_buffer_m=.75))
    route = dict(route=points, town="Town07", coordinate_reference="base_link", route_length_m=probe.CATALOG_ARC_M)
    return case, route


def test_catalog_arc_and_serialized_arc_are_distinct_exact_provenance_not_tolerance():
    case, route = original_catalog_fixture(); before = deepcopy((case, route))
    result = probe.mission_proposal(case, route, case_sha=probe.CASE_SHA, route_sha=probe.ROUTE_SHA)
    assert (case, route) == before
    assert result["polyline_map_xy_m"] == [[row["x"], row["y"]] for row in route["route"]]
    assert result["stop_reference_offset_m"] == .75
    assert probe.witness._cumulative(result["polyline_map_xy_m"])[-1] == probe.SERIALIZED_GEOMETRIC_ARC_M
    assert len(result["polyline_map_xy_m"]) == 213
    assert probe.SERIALIZED_GEOMETRIC_ARC_M != case["route_length_m"]
    with pytest.raises(ValueError):
        probe.previous.mission_proposal(case, route, case_sha=probe.CASE_SHA, route_sha=probe.ROUTE_SHA)


@pytest.mark.parametrize("kind", ["catalog", "geometry", "point_count", "offset", "routeSHA", "caseSHA"])
def test_exact_original_geometry_catalog_namespace_and_offset_fail_closed(kind):
    case, route = original_catalog_fixture(); cs, rs = probe.CASE_SHA, probe.ROUTE_SHA
    if kind == "catalog": case["route_length_m"] = probe.SERIALIZED_GEOMETRIC_ARC_M
    elif kind == "geometry": route["route"][-1]["x"] = math.nextafter(probe.SERIALIZED_GEOMETRIC_ARC_M, math.inf)
    elif kind == "point_count": route["route"].insert(1, {"x": .25, "y": 0.})
    elif kind == "offset": case["natural_goal_configuration"]["stop_buffer_m"] = .74
    elif kind == "routeSHA": rs = "a"*64
    else: cs = "a"*64
    with pytest.raises(ValueError): probe.mission_proposal(case, route, case_sha=cs, route_sha=rs)


def test_true_tiny_local_arc_survives_cumulative_global_cancellation_and_preserves_sign():
    states = [dict(x=0., y=0.), dict(x=0., y=1e20), dict(x=1e-12, y=1e20), dict(x=2e-12, y=1e20)]
    cumulative = [0.]
    for a, b in zip(states, states[1:]): cumulative.append(cumulative[-1]+math.dist([a["x"], a["y"]], [b["x"], b["y"]]))
    assert cumulative[3]-cumulative[1] == 0.
    assert probe.native_arc_between(states, 1, 3) == 2e-12
    assert probe.native_arc_between(states, 3, 1) == -2e-12
    assert probe.native_arc_between(states, 2, 2) == 0.


@pytest.mark.parametrize("start,end", [(True, 1), (0, False), (-1, 1), (0, 2), (0., 1)])
def test_local_arc_indices_fail_closed(start, end):
    with pytest.raises(ValueError): probe.native_arc_between([dict(x=0.,y=0.),dict(x=1.,y=0.)],start,end)


def test_v2_fullfloor_retains_v1_labels_denials_phases_native_projection_and_counts():
    mp, ms, states, cameras, windows, _ = fixture(); original = deepcopy((states,cameras,windows))
    old_summary, old_rows = probe.previous.analyze(mp, ms, "a"*64, states, cameras, windows)
    new_summary, new_rows = probe.analyze(mp, ms, "a"*64, states, cameras, windows)
    assert (states,cameras,windows) == original
    assert new_summary["schema"] == probe.SCHEMA and probe.previous.SCHEMA.endswith(".v1")
    assert new_summary["counts"] == old_summary["counts"] and new_summary["phase_counts"] == old_summary["phase_counts"]
    assert new_summary["source_failure_counts"] == old_summary["source_failure_counts"]
    assert new_summary["all_recorded_native_current_pose_projection_census"] == old_summary["all_recorded_native_current_pose_projection_census"]
    assert new_rows[0]["source_true_future_rows"] == old_rows[0]["source_true_future_rows"]
    assert new_rows[0]["causal_observation"] == old_rows[0]["causal_observation"]
    assert new_rows[0]["stream_Witness"] == old_rows[0]["stream_Witness"]
    for key in ("training_authorized","vehicle_control_approved","model_promoted"):
        assert new_summary[key] is old_summary[key] is False
    assert new_summary["D_and_first100ms_local_native_arc_use_math_fsum"]
    assert new_summary["catalog_route_arc_not_causal_support"]


def test_all_unchanged_helpers_are_same_frozen_objects_and_original_globals_never_mutate():
    names = ("exact_zero_reference","causal_observation","boundary_checks","true_heading_diagnostics","native_projection_census")
    before = dict(vars(probe.previous))
    for name in names: assert getattr(probe,name) is getattr(probe.previous,name)
    probe.build_functions()
    with pytest.raises(ValueError): probe.mission_proposal({}, {}, case_sha="a"*64,route_sha="b"*64)
    assert all(vars(probe.previous).get(k) is v for k,v in before.items())
    assert set(vars(probe.previous)) == set(before)
    assert probe.previous.digest_file(Path(probe.previous.__file__)) == probe.FROZEN_V1_SHA


def test_authenticated_AST_revision_rejects_wrong_or_duplicate_expression():
    with pytest.raises(ValueError):probe._revised_function(probe.previous.mission_proposal,[("missing_identifier","True")],dict(vars(probe.previous)))
    with pytest.raises(ValueError):probe._revised_function(probe.previous.analyze,[("state","states")],dict(vars(probe.previous)))


def test_new_run_namespace_binds_own_file_schema_and_both_frozen_providers():
    namespace = probe.run.__globals__
    assert namespace["__file__"] == probe.__file__ and namespace["SCHEMA"] == probe.SCHEMA
    assert namespace["AUTH_SCHEMA"] == probe.AUTH_SCHEMA and namespace["analyze"] is probe.analyze
    assert namespace["mission_proposal"] is probe.mission_proposal
    assert namespace["FROZEN"]["tools/e2e/probe_town07_terminal_feasibility_v6_20261002.py"] == probe.FROZEN_V1_SHA
    assert namespace["FROZEN"]["learning/portable_e2e/terminal_witness_v1_20261002.py"] == probe.previous.FROZEN["learning/portable_e2e/terminal_witness_v1_20261002.py"]
    assert probe.main.__globals__["run"] is probe.run


def test_no_boolean_finite_predicate_or_physical_limit_revision():
    for args in ((0.,0.,.01,"GO"),(.04,0.,.0005,"HOLD"),(1.,1.,.01,"GO")):
        assert probe.boundary_checks(*args,support=1.,terminal=True) == probe.previous.boundary_checks(*args,support=1.,terminal=True)


def test_source_only_import_no_torch_numpy_or_output():
    script="import runpy; runpy.run_path('tools/workspace_python.py')['configure'](); from scripts.e2e import probe_town07_terminal_feasibility_v6_v2_20261002 as p; import sys; assert 'torch' not in sys.modules and 'numpy' not in sys.modules; assert p.previous.digest_file(p.Path(p.previous.__file__))==p.FROZEN_V1_SHA"
    result=subprocess.run([sys.executable,"-c",script],cwd=probe.ROOT,capture_output=True,text=True)
    assert result.returncode == 0,result.stderr
