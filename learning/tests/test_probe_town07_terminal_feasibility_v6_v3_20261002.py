"""HH_261002 - Synthetic exact-duplicate Witness conversion and immutable full-floor tests, never actual raw jobs."""

from copy import deepcopy
import math
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

from scripts.e2e import probe_town07_terminal_feasibility_v6_v3_20261002 as probe
from test_probe_town07_terminal_feasibility_v6_20261002 import fixture, mission
from test_probe_town07_terminal_feasibility_v6_v2_20261002 import original_catalog_fixture


def retained_duplicate_fixture():
    case, route = original_catalog_fixture()
    canonical = [{"x": i*.5, "y": 0.} for i in range(210)] + [{"x": probe.previous.SERIALIZED_GEOMETRIC_ARC_M, "y": 0.}]
    original = deepcopy(canonical)
    original.insert(28,deepcopy(original[27])); original.insert(116,deepcopy(original[115]))
    route["route"] = original
    return case, route


def test_exact213_to211_preserves_all_coordinates_geometry_endpoint_and_exhaustive_mapping():
    case,route = retained_duplicate_fixture(); before = deepcopy((case,route))
    original = probe.previous.mission_proposal(case,route,case_sha=probe.CASE_SHA,route_sha=probe.ROUTE_SHA)
    canonical,proof = probe.mission_with_provenance(case,route,case_sha=probe.CASE_SHA,route_sha=probe.ROUTE_SHA)
    assert (case,route) == before and proof["original_all_XY_points"] == original["polyline_map_xy_m"]
    assert proof["original_point_count"] == 213 and proof["canonical_point_count"] == 211
    assert proof["collapsed_adjacent_exact_XY_index_groups"] == [[27,28],[115,116]]
    groups=proof["exhaustive_canonical_to_original_index_groups"]
    assert [index for group in groups for index in group] == list(range(213))
    for original_index,canonical_index in enumerate(proof["exhaustive_original_to_canonical_index"]):
        assert probe.encoded(original["polyline_map_xy_m"][original_index]) == probe.encoded(canonical["polyline_map_xy_m"][canonical_index])
        assert original_index in groups[canonical_index]
    assert canonical["goal_map_xy_m"] == original["goal_map_xy_m"]
    assert canonical["stop_reference_offset_m"] == original["stop_reference_offset_m"] == .75
    assert proof["original_geometric_arc_m"] == proof["canonical_geometric_arc_m"] == probe.previous.SERIALIZED_GEOMETRIC_ARC_M
    assert set(canonical) == set(probe.witness.MISSION_KEYS)
    assert canonical == probe.mission_proposal(case,route,case_sha=probe.CASE_SHA,route_sha=probe.ROUTE_SHA)


@pytest.mark.parametrize("points", [
    [[0.,0.],[1e-12,0.],[1.,0.]],
    [[0.,0.],[-0.,0.],[1.,0.]],
    [[0.,0.],[1.,0.],[0.,0.],[2.,0.]],
    [[0.,0.],[1.,0.],[1.+1e-12,0.],[2.,0.]],
])
def test_near_equal_signed_zero_distinct_and_nonadjacent_points_never_removed(points):
    source=mission(2.);source["polyline_map_xy_m"]=points;source["goal_map_xy_m"]=points[-1]
    result,proof=probe.canonicalize_adjacent_XY(source)
    assert result["polyline_map_xy_m"] == points and proof["collapsed_adjacent_exact_XY_index_groups"] == []
    assert proof["original_point_count"] == proof["canonical_point_count"] == len(points)


def test_nonadjacent_topology_ambiguity_still_fails_unchanged_Witness():
    points=[[0.,0.],[10.,0.],[0.,0.],[10.,0.]]
    source=mission(10.);source["polyline_map_xy_m"]=points;source["goal_map_xy_m"]=points[-1]
    result,proof=probe.canonicalize_adjacent_XY(source)
    arcs=probe.witness._cumulative(result["polyline_map_xy_m"])
    with pytest.raises(probe.witness.TerminalWitnessError) as error:
        probe.witness._project(result["polyline_map_xy_m"],arcs,[5.,.1,0.],(1.,0.))
    assert error.value.code == "AMBIGUOUS_MISSION_PROJECTION"
    assert proof["collapsed_adjacent_exact_XY_index_groups"] == []


@pytest.mark.parametrize("kind",["missing_dupes","wrong_dupe_index","near_not_exact","source_SHA","stop_offset"])
def test_retained_source_specific_conversion_rejects_other_mapping_or_source(kind):
    case,route=retained_duplicate_fixture();cs,rs=probe.CASE_SHA,probe.ROUTE_SHA
    if kind == "missing_dupes":case,route=original_catalog_fixture()
    elif kind == "wrong_dupe_index":route["route"][28]=deepcopy(route["route"][29])
    elif kind == "near_not_exact":route["route"][28]["x"]=math.nextafter(route["route"][28]["x"],math.inf)
    elif kind == "source_SHA":rs="a"*64
    else:case["natural_goal_configuration"]["stop_buffer_m"] = .74
    with pytest.raises(ValueError):probe.mission_proposal(case,route,case_sha=cs,route_sha=rs)


def test_original_duplicate_Witness_failure_and_canonical_current_input_floor_pass():
    case,route=retained_duplicate_fixture()
    raw=probe.previous.mission_proposal(case,route,case_sha=probe.CASE_SHA,route_sha=probe.ROUTE_SHA)
    canonical,proof=probe.mission_with_provenance(case,route,case_sha=probe.CASE_SHA,route_sha=probe.ROUTE_SHA)
    _,_,states,cameras,windows,_=fixture()
    with pytest.raises(ZeroDivisionError):probe.previous.causal_observation(probe.encoded(raw),probe.sha(probe.encoded(raw)),probe.ROUTE_SHA,states[0],cameras[0])
    payload=probe.encoded(canonical);ms=probe.sha(payload)
    decision,_=probe.previous.causal_observation(payload,ms,probe.ROUTE_SHA,states[0],cameras[0])
    assert decision.status == "VERIFIED"
    summary,rows=probe.analyze(payload,ms,probe.ROUTE_SHA,states,cameras,windows,source_mission_provenance=proof)
    assert len(rows) == 1 and summary["navigation_adjacent_duplicate_conversion_provenance"] == proof
    assert summary["all_actual_source_states_true64_and_failed_geometry_masks_preserved"]


def test_fullV2_analysis_bytecode_reused_counts_labels_failed_masks_and_denials_unchanged():
    mp,ms,states,cameras,windows,_=fixture();before=deepcopy((states,cameras,windows))
    old_summary,old_rows=probe.previous.analyze(mp,ms,"a"*64,states,cameras,windows)
    summary,rows=probe.analyze(mp,ms,"a"*64,states,cameras,windows)
    assert (states,cameras,windows) == before and rows == old_rows
    for key in ("counts","phase_counts","source_failure_counts","boundary_failure_counts","exact_zero_reference",
        "all_recorded_native_current_pose_projection_census"):
        assert summary[key] == old_summary[key]
    assert summary["schema"] == probe.SCHEMA
    assert all(summary[key] is False for key in ("training_authorized","vehicle_control_approved","model_promoted"))
    assert probe.native_arc_between is probe.previous.native_arc_between
    assert probe.boundary_checks is probe.previous.boundary_checks


def test_private_run_binds_new_source_and_preserves_original_owner_diagnostic_guard(monkeypatch):
    namespace=dict(probe.previous.run.__globals__)
    exec('''def fake_run(args):
        mission=mission_proposal(args.case,args.route,case_sha=CASE_SHA,route_sha=ROUTE_SHA)
        payload=encoded(mission)
        summary,rows=analyze(payload,sha(payload),ROUTE_SHA,*args.inputs)
        args.observed.update(file=__file__,schema=SCHEMA,auth_schema=AUTH_SCHEMA,sources=FROZEN,summary=summary,rows=rows)
        return 0
''',namespace)
    case,route=retained_duplicate_fixture();_,_,states,cameras,windows,_=fixture()
    sentinel=SimpleNamespace(case=case,route=route,inputs=(states,cameras,windows),observed={})
    # HH_261002 - A closure-free synthetic body exercises actual namespace/mission/analysis binding without any real input reads.
    monkeypatch.setattr(probe.previous,"run",namespace["fake_run"])
    assert probe.run(sentinel)==0 and probe.run(sentinel)==0
    assert sentinel.observed["file"]==probe.__file__ and sentinel.observed["schema"]==probe.SCHEMA
    assert sentinel.observed["auth_schema"]==probe.AUTH_SCHEMA and sentinel.observed["sources"]==probe.FROZEN
    assert sentinel.observed["summary"]["navigation_adjacent_duplicate_conversion_provenance"]["original_point_count"]==213
    assert len(sentinel.observed["rows"])==1
    assert "actual_teacher_capture_passed" in probe.previous.previous.run.__code__.co_consts


def test_main_and_namespace_source_closure_and_frozen_modules_unchanged_after_exception():
    old_v2=dict(vars(probe.previous));old_v1=dict(vars(probe.previous.previous))
    namespace=probe._namespace()
    assert namespace["SCHEMA"] == probe.SCHEMA and namespace["AUTH_SCHEMA"] == probe.AUTH_SCHEMA
    assert namespace["__file__"] == probe.__file__ and probe.main.__globals__["run"] is probe.run
    assert namespace["FROZEN"]["tools/e2e/probe_town07_terminal_feasibility_v6_v2_20261002.py"] == probe.FROZEN_V2_SHA
    with pytest.raises(ValueError):probe.mission_proposal({}, {},case_sha="x",route_sha="y")
    for module,snapshot in ((probe.previous,old_v2),(probe.previous.previous,old_v1)):
        assert set(vars(module)) == set(snapshot) and all(vars(module)[key] is value for key,value in snapshot.items())
    assert probe.previous.previous.digest_file(Path(probe.previous.__file__)) == probe.FROZEN_V2_SHA


def test_import_and_missing_auth_never_load_model_or_create_output(tmp_path):
    script="import runpy;runpy.run_path('tools/workspace_python.py')['configure']();from scripts.e2e import probe_town07_terminal_feasibility_v6_v3_20261002 as p;import sys;assert 'torch' not in sys.modules and 'numpy' not in sys.modules"
    result=subprocess.run([sys.executable,"-c",script],cwd=probe.ROOT,capture_output=True,text=True)
    assert result.returncode == 0,result.stderr
    with pytest.raises(ValueError):probe.main(["--mode","probe","--output",str(tmp_path/"out")])
    assert not (tmp_path/"out").exists()
