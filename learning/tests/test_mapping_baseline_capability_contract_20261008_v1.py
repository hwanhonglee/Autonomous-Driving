"""HH_261008 - Pure scalar/AST baseline-vocabulary checks only; no arrays, model, geometry projection or map writes."""
from __future__ import annotations

import ast
from dataclasses import FrozenInstanceError
from pathlib import Path
import sys

import pytest

from portable_e2e import mapping_baseline_capability_contract_20261008_v1 as contract


ROOT = Path(__file__).resolve().parents[2]


def admit(points=(), **changes):
    fields = dict(timestamp_s=1.0, frame_id="carla_ue:Town01", source="learned_prediction",
                  points=points, expected_frame_id="carla_ue:Town01")
    fields.update(changes)
    return contract.admit_indexed_observation(contract.MODEL_LABELS, **fields)


def literal_assignment(path, name):
    tree = ast.parse((ROOT / path).read_bytes())
    node = next(node for node in tree.body if isinstance(node, ast.Assign)
                and any(isinstance(target, ast.Name) and target.id == name for target in node.targets))
    return ast.literal_eval(node.value)


def test_actual_model_preparation_evaluator_and_oracle_source_vocabularies_are_distinct():
    assert literal_assignment("learning/portable_e2e/map_perception.py", "LABELS") == contract.MODEL_LABELS
    assert literal_assignment("tools/e2e/prepare_map_perception_pilot.py", "LABELS") == contract.MODEL_LABELS
    assert literal_assignment("tools/e2e/evaluate_ctrack_fixed_maps_20261001.py", "LABELS") == contract.MODEL_LABELS
    assert literal_assignment("tools/e2e/evaluate_ctrack_fixed_maps_20261001.py", "RAW_LABELS") == dict(contract.REVIEWED_ORACLE_TAG_LABELS)
    assert dict(contract.REVIEWED_ORACLE_TAG_LABELS)[24] == "lane_marking"
    assert contract.MODEL_LABELS.index("lane_marking") == 3


def test_six_label_definitions_not_all_generic_future_labels_or_accuracy_approval():
    report = contract.capability_report(contract.MODEL_LABELS)
    assert report.model_label_order == contract.MODEL_LABELS and len(report.static_output_labels) == 6
    assert set(report.unsupported_static_labels) == {"road_boundary", "crosswalk", "stop_line"}
    assert set(report.unsupported_dynamic_labels) == {"vehicle", "pedestrian", "cyclist", "signal_state"}
    assert not any((report.hd_map_approved, report.runtime_or_feature_approved,
                    report.traffic_light_fixture_is_signal_state, report.lane_topology_implemented,
                    report.learned_prediction_authenticated, report.sensor_provenance_authenticated,
                    report.calibrated_geometry_verified))
    assert "UNKNOWN" in report.depth_and_pose_provenance
    assert "NOT_ACCURACY" in report.capability_scope and "NOT_AN_ABSENCE" in report.unsupported_semantics


def test_complete_points_static_and_other_unknown_retained_without_zero_confidence_drop():
    points = ((1., 2., 0, .9), (-3., -4., 1, 0.), (5., 6., 4, .7), (7., 8., 0, 0.))
    result = admit(points)
    assert result.declared_point_count == 4
    assert result.static_observation.points == ((-3., -4., "road", 0.), (5., 6., "traffic_light_fixture", .7))
    assert result.other_unknown_points == (points[0], points[3])
    assert len(result.other_unknown_points) + len(result.static_observation.points) == result.declared_point_count
    assert not result.map_mutated and not result.model_inference_executed
    assert "UNVERIFIED" in result.coordinates_status


def test_all_six_supported_indices_keep_the_exact_actual_model_order():
    result = admit(tuple((float(i), 0., i, .5) for i in range(1, 7)))
    assert tuple(point[2] for point in result.static_observation.points) == contract.STATIC_OUTPUT_LABELS
    assert not result.other_unknown_points


def test_empty_observation_is_empty_unknown_evidence_not_detected_absence():
    result = admit()
    assert result.declared_point_count == 0 and result.static_observation.points == result.other_unknown_points == ()
    assert result.capabilities.other_semantics == "UNKNOWN_NOT_FREE_SPACE_OR_OBJECT_ABSENCE"


def test_envelope_observation_and_capability_objects_are_immutable():
    result = admit(((1., 2., 0, .5),))
    for value, field, changed in ((result, "map_mutated", True),
                                  (result.static_observation, "source", "oracle_sensor"),
                                  (result.capabilities, "hd_map_approved", True)):
        with pytest.raises(FrozenInstanceError):
            setattr(value, field, changed)
    with pytest.raises(TypeError):
        result.other_unknown_points[0] = (0., 0., 0, .5)


@pytest.mark.parametrize("labels", [list(contract.MODEL_LABELS), (), contract.MODEL_LABELS[:-1],
    tuple(reversed(contract.MODEL_LABELS)), (True, *contract.MODEL_LABELS[1:]),
    (*contract.MODEL_LABELS[:-1], "crosswalk"), (*contract.MODEL_LABELS[:-1], "signal_state"),
    (*contract.MODEL_LABELS[:-1], "traffic_light_state")])
def test_missing_reordered_foreign_or_dynamic_provider_vocabulary_denied(labels):
    with pytest.raises(ValueError):
        contract.capability_report(labels)


@pytest.mark.parametrize("index", [True, False, -1, 7, 24, 1.0, "road", "crosswalk", "signal_state", None])
def test_boolean_foreign_names_out_of_range_and_oracle_class_indices_denied(index):
    with pytest.raises(ValueError):
        admit(((1., 2., index, .5),))


@pytest.mark.parametrize("value", [True, False, float("nan"), float("inf"), -float("inf"), 10**400, "0.5", None])
@pytest.mark.parametrize("field", ["x", "y", "confidence"])
def test_nonfinite_boolean_or_nonbuiltin_point_scalars_denied(field, value):
    point = {"x": 1., "y": 2., "index": 1, "confidence": .5}
    point[field] = value
    with pytest.raises(ValueError):
        admit((tuple(point[name] for name in ("x", "y", "index", "confidence")),))


@pytest.mark.parametrize("value", [-.1, 1.1])
def test_invalid_confidence_denied_even_for_other_unknown(value):
    with pytest.raises(ValueError):
        admit(((1., 2., 0, value),))


@pytest.mark.parametrize("timestamp", [True, False, -.1, float("nan"), float("inf"), 10**400, "1", None])
def test_invalid_timestamp_denied(timestamp):
    with pytest.raises(ValueError):
        admit(timestamp_s=timestamp)


@pytest.mark.parametrize("frame", [None, True, "", " map", "map ", "map\n", "a" * 257, "foreign"])
@pytest.mark.parametrize("field", ["frame_id", "expected_frame_id"])
def test_foreign_missing_or_invalid_frames_denied(field, frame):
    with pytest.raises(ValueError):
        admit(**{field: frame})


@pytest.mark.parametrize("source", ["oracle_sensor", "", "unknown", "learned_prediction ", True, None])
def test_oracle_or_unspecified_source_not_silently_admitted(source):
    with pytest.raises(ValueError):
        admit(source=source)


@pytest.mark.parametrize("points", [[(1., 2., 1, .5)], ((1., 2., 1),),
    ((1., 2., 1, .5, "extra"),), ([1., 2., 1, .5],), (None,)])
def test_wrong_whole_container_or_point_shape_denied(points):
    with pytest.raises(ValueError):
        admit(points)


def test_complete_validation_before_observation_construction_or_partial_success(monkeypatch):
    calls = []
    monkeypatch.setattr(contract, "Observation", lambda *args: calls.append(args))
    valid = (1., 2., 1, .5)
    invalid_other = (3., float("nan"), 0, .5)
    with pytest.raises(ValueError):
        admit((valid, invalid_other))
    assert calls == []


def test_whole_fixed_scalar_cap_rejects_without_truncation_or_partial_observation(monkeypatch):
    point = (1., 2., 0, .5)
    result = admit((point,) * contract.MAX_POINTS)
    assert result.declared_point_count == len(result.other_unknown_points) == 32768
    assert not result.static_observation.points
    monkeypatch.setattr(contract, "Observation", lambda *a: pytest.fail("constructed before cap rejection"))
    with pytest.raises(ValueError):
        admit((point,) * (contract.MAX_POINTS + 1))


def test_new_factory_never_imports_model_or_updates_grid_or_launches_runtime():
    path = ROOT / "learning/portable_e2e/mapping_baseline_capability_contract_20261008_v1.py"
    tree = ast.parse(path.read_bytes())
    imports = [node.module if isinstance(node, ast.ImportFrom) else name.name
               for node in ast.walk(tree) if isinstance(node, (ast.Import, ast.ImportFrom))
               for name in (node.names if isinstance(node, ast.Import) else [None])]
    assert set(imports) <= {"__future__", "dataclasses", "math", "mapping"}
    calls = {ast.unparse(node.func) for node in ast.walk(tree) if isinstance(node, ast.Call)}
    assert not any(name.endswith(".update") or "forward" in name or "spawn" in name or "open" in name for name in calls)
    assert not any(name in sys.modules for name in ("torch", "numpy", "PIL", "carla", "rclpy"))
