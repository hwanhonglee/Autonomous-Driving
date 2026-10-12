"""HH_261008 - Disclose the current seven-class map vocabulary without adding a detector or changing frozen fusion.

This scalar admission factory does not import a model, decode arrays, project
depth, authenticate a sensor/provider, update a map, or approve a driving/HD-map
capability. Callers supply already projected coordinates; geometry remains
unverified. Generic mapping.py deliberately retains a wider future vocabulary.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

from .mapping import Observation


SCHEMA = "portable_e2e.mapping_baseline_capability_contract.20261008.v1"
MODEL_LABELS = ("other", "road", "sidewalk", "lane_marking",
                "traffic_light_fixture", "traffic_sign", "pole")
STATIC_OUTPUT_LABELS = MODEL_LABELS[1:]
UNSUPPORTED_STATIC_LABELS = ("road_boundary", "crosswalk", "stop_line")
UNSUPPORTED_DYNAMIC_LABELS = ("vehicle", "pedestrian", "cyclist", "signal_state")
MAX_POINTS = 32768
# HH_261008 - These are source-review references, not automatic provider authentication or permission to import models.
REFERENCE_SOURCE_PINS = (
    ("learning/portable_e2e/map_perception.py", "1b0076c8f9d34192ab5aedf6cdf96ce2c14458e2bb49986685a6811376b0750b"),
    ("learning/portable_e2e/mapping.py", "264f9dda759f6986bde9c614e5cf3ae21680a22ae58d855db11564ceb3b3b867"),
    ("learning/portable_e2e/mapping_projection.py", "c42ab7d20454475f544b8073d6a23a104a0a2bb1cc2cbb7be092fa3e871cbbee"),
    ("tools/e2e/evaluate_ctrack_fixed_maps_20261001.py", "fa210fbce9aefa99f4ffc13efbeae42e2a4fe9c7281d144e71f298a44700ed38"),
    ("tools/e2e/prepare_map_perception_pilot.py", "395e0bbba47cb9a7dd2629c9d6357e0997de7417d6c6beb65d694246d3e5f15f"),
)
# HH_261008 - CARLA oracle tag IDs are not learned class indices; state is not encoded by the fixture class.
REVIEWED_ORACLE_TAG_LABELS = ((1, "road"), (2, "sidewalk"), (24, "lane_marking"),
                            (7, "traffic_light_fixture"), (8, "traffic_sign"), (6, "pole"))


def require(condition, message):
    if not condition:
        raise ValueError(message)


def validate_provider_labels(provider_labels):
    require(type(provider_labels) is tuple and provider_labels == MODEL_LABELS
            and all(type(label) is str for label in provider_labels),
            "exact ordered seven learned labels required, not oracle IDs or a wider future vocabulary")


@dataclass(frozen=True)
class BaselineCapabilities:
    schema: str = SCHEMA + ".capabilities"
    model_label_order: tuple = MODEL_LABELS
    static_output_labels: tuple = STATIC_OUTPUT_LABELS
    unsupported_static_labels: tuple = UNSUPPORTED_STATIC_LABELS
    unsupported_dynamic_labels: tuple = UNSUPPORTED_DYNAMIC_LABELS
    other_semantics: str = "UNKNOWN_NOT_FREE_SPACE_OR_OBJECT_ABSENCE"
    capability_scope: str = "OUTPUT_VOCABULARY_ONLY_NOT_ACCURACY_OR_FEATURE_APPROVAL"
    unsupported_semantics: str = "NOT_OUTPUT_BY_THIS_MODEL_NOT_AN_ABSENCE_DETECTION"
    traffic_light_fixture_is_signal_state: bool = False
    learned_prediction_authenticated: bool = False
    sensor_provenance_authenticated: bool = False
    calibrated_geometry_verified: bool = False
    depth_and_pose_provenance: str = "UNKNOWN_NOT_VERIFIED_BY_FACTORY"
    lane_topology_implemented: bool = False
    hd_map_approved: bool = False
    runtime_or_feature_approved: bool = False


@dataclass(frozen=True)
class BaselineObservationAdmission:
    """HH_261008 - All points survive as either six-static evidence or separately retained class-zero UNKNOWNs."""
    static_observation: Observation
    other_unknown_points: tuple
    capabilities: BaselineCapabilities
    declared_point_count: int
    schema: str = SCHEMA + ".admitted_observation"
    coordinates_status: str = "UNVERIFIED_CALLER_PROVIDED_XY_NOT_A_3D_CALIBRATION_PROOF"
    map_mutated: bool = False
    model_inference_executed: bool = False


def capability_report(provider_labels):
    """HH_261008 - A correct label order declares output names only; it cannot attest that a model actually ran."""
    validate_provider_labels(provider_labels)
    return BaselineCapabilities()


def _frame(value):
    require(type(value) is str and 0 < len(value) <= 256
            and value == value.strip() and not any(ord(char) < 32 for char in value),
            "explicit bounded frame name required")


def _finite_number(value, name):
    require(type(value) in (int, float), "finite builtin scalar required: " + name)
    # HH_261008 - A huge builtin integer must fail closed as a scalar error, not leak a conversion OverflowError.
    try:
        finite = math.isfinite(value)
    except OverflowError:
        finite = False
    require(finite, "finite builtin scalar required: " + name)


def admit_indexed_observation(provider_labels, *, timestamp_s, frame_id, source,
                              points, expected_frame_id):
    """HH_261008 - Validate the whole immutable scalar input before constructing an envelope; never mutate a grid.

    points is a tuple of (x, y, learned_class_index, confidence) tuples. It is
    not a raster/NPZ input. Class zero remains UNKNOWN with its exact provided
    coordinate/confidence record retained. Confidence zero is not dropped.
    """
    capabilities = capability_report(provider_labels)
    _frame(frame_id)
    _frame(expected_frame_id)
    require(frame_id == expected_frame_id, "provided and expected frame differ")
    require(type(source) is str and source == "learned_prediction",
            "explicit learned_prediction required; oracle observations are a different contract")
    _finite_number(timestamp_s, "timestamp_s")
    require(timestamp_s >= 0, "nonnegative timestamp required")
    require(type(points) is tuple and len(points) <= MAX_POINTS,
            "bounded complete immutable scalar points required; never truncate")
    # HH_261008 - Validate background points too; separating UNKNOWN must not mask invalid values or failed tails.
    for point in points:
        require(type(point) is tuple and len(point) == 4, "exact scalar point tuple required")
        x, y, index, confidence = point
        _finite_number(x, "x")
        _finite_number(y, "y")
        require(type(index) is int and 0 <= index < len(MODEL_LABELS),
                "actual learned class index required; booleans, names and oracle tag IDs are denied")
        _finite_number(confidence, "confidence")
        require(0 <= confidence <= 1, "confidence must be in [0,1]")
    static_points = tuple((x, y, MODEL_LABELS[index], confidence)
                          for x, y, index, confidence in points if index != 0)
    unknown_points = tuple(point for point in points if point[2] == 0)
    require(len(static_points) + len(unknown_points) == len(points), "whole point denominator required")
    observation = Observation(timestamp_s, frame_id, source, static_points)
    return BaselineObservationAdmission(observation, unknown_points, capabilities, len(points))
